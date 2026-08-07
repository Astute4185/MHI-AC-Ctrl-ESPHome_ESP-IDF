#include "mhi_command_coordinator.h"

namespace esphome {
namespace mhi_ac_ctrl {

void MhiCommandCoordinator::reset() {
  confirmation_.reset();
  next_generation_ = 1U;
  command_in_flight_ = false;
  in_flight_envelope_ = {};
  in_flight_command_before_build_ = {};
  in_flight_runtime_before_build_ = {};
  in_flight_runtime_snapshot_valid_ = false;
  in_flight_staged_ms_ = 0U;
  staged_timeout_reported_ = false;
  final_confirmation_grace_active_ = false;
  coalesced_extended_patch_ = {};
  coalesced_extended_patch_pending_ = false;
  this->reset_attempts_();
}

bool MhiCommandCoordinator::prepare_next(MhiCommandState& command, MhiTxRuntime& runtime,
                                         const MhiTxBuildConfig& config, MhiFrameBuffer& frame,
                                         MhiTxBuildResult& result, MhiTxEnvelope& envelope) {
  if (command_in_flight_ || confirmation_.has_pending()) {
    return false;
  }

  this->apply_coalesced_extended_patch_(command);

  if (!MhiTxBuilder::build_next_frame(command, runtime, config, frame, result) || frame.len == 0U) {
    return false;
  }

  envelope = {};
  if (!envelope.set_frame(frame.bytes(), frame.len)) {
    return false;
  }

  if (result.has_encoded_commands()) {
    envelope.kind = MhiTxKind::COMMAND;
    envelope.generation = next_generation_++;
    if (next_generation_ == 0U) {
      next_generation_ = 1U;
    }
    envelope.command_mask = result.encoded_command_mask;
    envelope.intent = result.intent;
  }

  return true;
}

void MhiCommandCoordinator::on_stage_result(const MhiTxEnvelope& envelope, const MhiCommandState& command_before_build,
                                            MhiCommandState& command, bool staged, uint32_t staged_at_ms) {
  this->on_stage_result_(envelope, command_before_build, nullptr, command, staged, staged_at_ms);
}

void MhiCommandCoordinator::on_stage_result(const MhiTxEnvelope& envelope, const MhiCommandState& command_before_build,
                                            const MhiTxRuntime& runtime_before_build, MhiCommandState& command,
                                            bool staged, uint32_t staged_at_ms) {
  this->on_stage_result_(envelope, command_before_build, &runtime_before_build, command, staged, staged_at_ms);
}

void MhiCommandCoordinator::on_stage_result_(const MhiTxEnvelope& envelope, const MhiCommandState& command_before_build,
                                             const MhiTxRuntime* runtime_before_build, MhiCommandState& command,
                                             bool staged, uint32_t staged_at_ms) {
  if (!envelope.is_command()) {
    return;
  }

  if (!staged) {
    restore_command_mask_(command, command_before_build, envelope.command_mask);
    // A coalesced companion field may have been injected inside prepare_next()
    // after command_before_build was captured. Restore from the encoded intent
    // as a fallback so a queue failure cannot discard the latest composite target.
    restore_intent_mask_(command, envelope.intent, envelope.command_mask);
    return;
  }

  command_in_flight_ = true;
  in_flight_envelope_ = envelope;
  in_flight_command_before_build_ = command_before_build;
  if (runtime_before_build != nullptr) {
    in_flight_runtime_before_build_ = *runtime_before_build;
    in_flight_runtime_snapshot_valid_ = true;
  } else {
    in_flight_runtime_before_build_ = {};
    in_flight_runtime_snapshot_valid_ = false;
  }
  in_flight_attempt_ = next_attempt_;
  in_flight_staged_ms_ = staged_at_ms;
  staged_timeout_reported_ = false;
}

bool MhiCommandCoordinator::prepare_staged_replacement(const MhiCommandState& command, const MhiTxBuildConfig& config,
                                                       MhiStagedCommandReplacement& replacement) const {
  replacement = {};
  if (!command_in_flight_ || confirmation_.has_pending() || !in_flight_runtime_snapshot_valid_ ||
      !command.has_pending_command()) {
    return false;
  }

  MhiCommandState combined = command;

  // Rebuild the same not-yet-transmitted command generation plus the latest
  // queued user state. Existing queued fields take precedence, so a newer
  // value for the same field supersedes the staged value before transmission.
  restore_command_mask_(combined, in_flight_command_before_build_, in_flight_envelope_.command_mask);
  restore_intent_mask_(combined, in_flight_envelope_.intent, in_flight_envelope_.command_mask);

  replacement.expected_generation = in_flight_envelope_.generation;
  replacement.command_before_build = combined;
  replacement.command_after_build = combined;
  replacement.runtime_after_build = in_flight_runtime_before_build_;

  if (!MhiTxBuilder::build_next_frame(replacement.command_after_build, replacement.runtime_after_build, config,
                                      replacement.frame, replacement.build_result) ||
      replacement.frame.len == 0U || !replacement.build_result.has_encoded_commands()) {
    replacement = {};
    return false;
  }

  if (!replacement.envelope.set_frame(replacement.frame.bytes(), replacement.frame.len)) {
    replacement = {};
    return false;
  }

  replacement.envelope.kind = MhiTxKind::COMMAND;
  replacement.envelope.generation = next_generation_;
  replacement.envelope.command_mask = replacement.build_result.encoded_command_mask;
  replacement.envelope.intent = replacement.build_result.intent;
  return true;
}

bool MhiCommandCoordinator::commit_staged_replacement(const MhiStagedCommandReplacement& replacement,
                                                      MhiCommandState& command, MhiTxRuntime& runtime,
                                                      uint32_t staged_at_ms) {
  if (!replacement.valid() || !command_in_flight_ ||
      replacement.expected_generation != in_flight_envelope_.generation ||
      replacement.envelope.generation != next_generation_) {
    return false;
  }

  command = replacement.command_after_build;
  runtime = replacement.runtime_after_build;
  in_flight_envelope_ = replacement.envelope;
  in_flight_command_before_build_ = replacement.command_before_build;
  in_flight_attempt_ = 1U;
  next_attempt_ = 1U;
  in_flight_staged_ms_ = staged_at_ms;
  staged_timeout_reported_ = false;

  next_generation_++;
  if (next_generation_ == 0U) {
    next_generation_ = 1U;
  }
  return true;
}

bool MhiCommandCoordinator::on_tx_completion(const MhiTxCompletion& completion, MhiCommandState& command) {
  if (!completion.is_command()) {
    return false;
  }
  if (!command_in_flight_ || completion.generation != in_flight_envelope_.generation) {
    return false;
  }

  if (completion.success) {
    uint32_t confirm_mask = in_flight_envelope_.command_mask;

    // A horizontal and 3D request share DB16/DB17. If a request for either
    // field arrived while this frame was in flight and changes the transmitted
    // composite target, abandon the old extended confirmation and retain one
    // complete latest-state patch for the next generation.
    confirm_mask &= ~this->coalesce_extended_supersession_(in_flight_envelope_.intent, confirm_mask, command);

    // For independent fields, a newer request for the same field supersedes
    // semantic confirmation of the value that just completed transmission.
    MhiCommandConfirmation in_flight_confirmation{};
    in_flight_confirmation.stage(in_flight_envelope_.intent, confirm_mask, completion.completed_at_ms);
    confirm_mask &= ~in_flight_confirmation.supersede(command);

    confirmation_.stage(in_flight_envelope_.intent, confirm_mask, completion.completed_at_ms);
    confirmation_attempt_ = in_flight_attempt_;
    if (!confirmation_.has_pending()) {
      this->reset_attempts_();
    }
  } else {
    restore_command_mask_(command, in_flight_command_before_build_, in_flight_envelope_.command_mask);
    restore_intent_mask_(command, in_flight_envelope_.intent, in_flight_envelope_.command_mask);
  }

  command_in_flight_ = false;
  in_flight_envelope_ = {};
  in_flight_command_before_build_ = {};
  in_flight_runtime_before_build_ = {};
  in_flight_runtime_snapshot_valid_ = false;
  in_flight_attempt_ = 0U;
  in_flight_staged_ms_ = 0U;
  staged_timeout_reported_ = false;
  return true;
}

uint32_t MhiCommandCoordinator::observe_status(const MhiStatusState& status) {
  const uint32_t confirmed = confirmation_.observe_status(status);
  if (!confirmation_.has_pending()) {
    final_confirmation_grace_active_ = false;
    this->reset_attempts_();
  }
  return confirmed;
}

uint32_t MhiCommandCoordinator::settle_pending_mask(uint32_t mask) {
  const uint32_t settled = confirmation_.settle_pending_mask(mask);
  if (!confirmation_.has_pending()) {
    final_confirmation_grace_active_ = false;
    this->reset_attempts_();
  }
  return settled;
}

uint32_t MhiCommandCoordinator::supersede_pending(const MhiCommandState& patch) {
  uint32_t superseded = 0U;

  if (confirmation_.has_pending()) {
    const uint32_t obsolete_extended =
        this->coalesce_extended_supersession_(confirmation_.pending_intent(), confirmation_.pending_mask(), patch);
    superseded |= confirmation_.settle_pending_mask(obsolete_extended);
  }

  superseded |= confirmation_.supersede(patch);
  if (!confirmation_.has_pending()) {
    final_confirmation_grace_active_ = false;
    this->reset_attempts_();
  }
  return superseded;
}

MhiCommandTimeoutResult MhiCommandCoordinator::expire(uint32_t now_ms, MhiCommandState& command) {
  MhiCommandTimeoutResult result{};
  const uint8_t attempt = confirmation_attempt_ == 0U ? 1U : confirmation_attempt_;

  if (final_confirmation_grace_active_) {
    const MhiCommandExpiration expiration =
        confirmation_.expire(now_ms, normal_confirmation_timeout_ms_, final_confirmation_grace_ms_);
    if (!expiration.expired()) {
      return result;
    }

    result.attempt = attempt;
    const uint32_t already_queued = command.pending_command_mask() & expiration.mask;
    result.superseded_mask = already_queued;
    result.exhausted_mask = expiration.mask & ~already_queued;
    final_confirmation_grace_active_ = false;
    this->reset_attempts_();
    return result;
  }

  const MhiCommandExpiration pending_expiration =
      confirmation_.inspect_expiration(now_ms, normal_confirmation_timeout_ms_);
  if (!pending_expiration.expired()) {
    return result;
  }

  result.timed_out_mask = pending_expiration.mask;
  result.attempt = attempt;
  const uint32_t already_queued = command.pending_command_mask() & pending_expiration.mask;
  result.superseded_mask = already_queued;

  if (attempt < kMhiMaxCommandAttempts) {
    const MhiCommandExpiration expiration = confirmation_.expire(now_ms, normal_confirmation_timeout_ms_);
    result.retry_mask = restore_intent_mask_(command, expiration.intent, expiration.mask & ~already_queued);
    result.superseded_mask |= expiration.mask & ~(result.retry_mask | result.superseded_mask);
    if (result.retry_mask != 0U) {
      next_attempt_ = static_cast<uint8_t>(attempt + 1U);
    } else {
      this->reset_attempts_();
    }
    return result;
  }

  if (already_queued != 0U) {
    confirmation_.settle_pending_mask(already_queued);
  }

  if (!confirmation_.has_pending()) {
    this->reset_attempts_();
    return result;
  }

  if (final_confirmation_grace_ms_ != 0U) {
    final_confirmation_grace_active_ = true;
    result.grace_mask = confirmation_.pending_mask();
    return result;
  }

  const MhiCommandExpiration expiration = confirmation_.expire(now_ms, normal_confirmation_timeout_ms_);
  result.exhausted_mask = expiration.mask;
  this->reset_attempts_();
  return result;
}

uint32_t MhiCommandCoordinator::staged_timeout_mask(uint32_t now_ms, uint32_t timeout_ms) {
  if (!command_in_flight_ || staged_timeout_reported_ || in_flight_staged_ms_ == 0U || now_ms < in_flight_staged_ms_ ||
      (now_ms - in_flight_staged_ms_) < timeout_ms) {
    return 0U;
  }

  staged_timeout_reported_ = true;
  return in_flight_envelope_.command_mask;
}

uint32_t MhiCommandCoordinator::coalesce_extended_supersession_(const MhiCommandIntent& intent, uint32_t confirm_mask,
                                                                const MhiCommandState& patch) {
  const uint32_t extended_confirm_mask = confirm_mask & kMhiExtendedLouverCommandMask;
  const bool crosses_shared_domain =
      ((extended_confirm_mask & MHI_COMMAND_HORIZONTAL_VANE) != 0U && patch.three_d_auto_set) ||
      ((extended_confirm_mask & MHI_COMMAND_THREE_D_AUTO) != 0U && patch.horizontal_vane_set);
  if (extended_confirm_mask == 0U || !intent.has_extended_louver_context || !crosses_shared_domain) {
    return 0U;
  }

  MhiCommandState desired{};
  desired.horizontal_vane_set = true;
  desired.horizontal_vane = intent.horizontal_vane;
  desired.three_d_auto_set = true;
  desired.three_d_auto = intent.three_d_auto;

  if (coalesced_extended_patch_pending_) {
    if (coalesced_extended_patch_.horizontal_vane_set) {
      desired.horizontal_vane = coalesced_extended_patch_.horizontal_vane;
    }
    if (coalesced_extended_patch_.three_d_auto_set) {
      desired.three_d_auto = coalesced_extended_patch_.three_d_auto;
    }
  }

  if (patch.horizontal_vane_set) {
    desired.horizontal_vane = patch.horizontal_vane;
  }
  if (patch.three_d_auto_set) {
    desired.three_d_auto = patch.three_d_auto;
  }

  const bool composite_changed =
      desired.horizontal_vane != intent.horizontal_vane || desired.three_d_auto != intent.three_d_auto;
  if (!composite_changed) {
    return 0U;
  }

  coalesced_extended_patch_ = desired;
  coalesced_extended_patch_pending_ = true;
  return extended_confirm_mask;
}

void MhiCommandCoordinator::apply_coalesced_extended_patch_(MhiCommandState& command) {
  if (!coalesced_extended_patch_pending_) {
    return;
  }

  // Explicitly queued values are newer than the cached companion context and
  // therefore take precedence. Fill only the missing side of the composite.
  if (!command.horizontal_vane_set && coalesced_extended_patch_.horizontal_vane_set) {
    command.horizontal_vane_set = true;
    command.horizontal_vane = coalesced_extended_patch_.horizontal_vane;
  }
  if (!command.three_d_auto_set && coalesced_extended_patch_.three_d_auto_set) {
    command.three_d_auto_set = true;
    command.three_d_auto = coalesced_extended_patch_.three_d_auto;
  }

  coalesced_extended_patch_ = {};
  coalesced_extended_patch_pending_ = false;
}

void MhiCommandCoordinator::restore_command_mask_(MhiCommandState& destination, const MhiCommandState& source,
                                                  uint32_t mask) {
  if ((mask & MHI_COMMAND_POWER) != 0U && !destination.power_set) {
    destination.power_set = source.power_set;
    destination.power = source.power;
  }
  if ((mask & MHI_COMMAND_MODE) != 0U && !destination.mode_set) {
    destination.mode_set = source.mode_set;
    destination.mode = source.mode;
  }
  if ((mask & MHI_COMMAND_FAN) != 0U && !destination.fan_set) {
    destination.fan_set = source.fan_set;
    destination.fan = source.fan;
  }
  if ((mask & MHI_COMMAND_TARGET_TEMP) != 0U && !destination.target_temp_set) {
    destination.target_temp_set = source.target_temp_set;
    destination.target_temp_c = source.target_temp_c;
  }
  if ((mask & MHI_COMMAND_VERTICAL_VANE) != 0U && !destination.vertical_vane_set) {
    destination.vertical_vane_set = source.vertical_vane_set;
    destination.vertical_vane = source.vertical_vane;
  }
  if ((mask & MHI_COMMAND_HORIZONTAL_VANE) != 0U && !destination.horizontal_vane_set) {
    destination.horizontal_vane_set = source.horizontal_vane_set;
    destination.horizontal_vane = source.horizontal_vane;
  }
  if ((mask & MHI_COMMAND_THREE_D_AUTO) != 0U && !destination.three_d_auto_set) {
    destination.three_d_auto_set = source.three_d_auto_set;
    destination.three_d_auto = source.three_d_auto;
  }
  if ((mask & MHI_COMMAND_ROOM_TEMP_OVERRIDE) != 0U && !destination.room_temp_override_set) {
    destination.room_temp_override_set = source.room_temp_override_set;
    destination.room_temp_override_raw = source.room_temp_override_raw;
  }
  if ((mask & MHI_COMMAND_ERROR_OPDATA_REQUEST) != 0U) {
    destination.error_opdata_request = source.error_opdata_request;
  }
}

uint32_t MhiCommandCoordinator::restore_intent_mask_(MhiCommandState& destination, const MhiCommandIntent& intent,
                                                     uint32_t mask) {
  uint32_t restored = 0U;
  if ((mask & MHI_COMMAND_POWER) != 0U && !destination.power_set) {
    destination.power_set = true;
    destination.power = intent.power;
    restored |= MHI_COMMAND_POWER;
  }
  if ((mask & MHI_COMMAND_MODE) != 0U && !destination.mode_set) {
    destination.mode_set = true;
    destination.mode = intent.mode;
    restored |= MHI_COMMAND_MODE;
  }
  if ((mask & MHI_COMMAND_FAN) != 0U && !destination.fan_set) {
    destination.fan_set = true;
    destination.fan = intent.fan;
    restored |= MHI_COMMAND_FAN;
  }
  if ((mask & MHI_COMMAND_TARGET_TEMP) != 0U && !destination.target_temp_set) {
    destination.target_temp_set = true;
    destination.target_temp_c = intent.target_temp_c;
    restored |= MHI_COMMAND_TARGET_TEMP;
  }
  if ((mask & MHI_COMMAND_VERTICAL_VANE) != 0U && !destination.vertical_vane_set) {
    destination.vertical_vane_set = true;
    destination.vertical_vane = intent.vertical_vane;
    restored |= MHI_COMMAND_VERTICAL_VANE;
  }
  if ((mask & MHI_COMMAND_HORIZONTAL_VANE) != 0U && !destination.horizontal_vane_set) {
    destination.horizontal_vane_set = true;
    destination.horizontal_vane = intent.horizontal_vane;
    restored |= MHI_COMMAND_HORIZONTAL_VANE;
  }
  if ((mask & MHI_COMMAND_THREE_D_AUTO) != 0U && !destination.three_d_auto_set) {
    destination.three_d_auto_set = true;
    destination.three_d_auto = intent.three_d_auto;
    restored |= MHI_COMMAND_THREE_D_AUTO;
  }
  return restored;
}

void MhiCommandCoordinator::reset_attempts_() {
  next_attempt_ = 1U;
  confirmation_attempt_ = 0U;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
