#include "mhi_protocol_trace.h"

#include <algorithm>
#include <cmath>

#include "mhi_checksum.h"

namespace esphome {
namespace mhi_ac_ctrl {

namespace {

uint8_t clamp_u8(uint32_t value, uint8_t minimum, uint8_t maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return static_cast<uint8_t>(value);
}

bool float_equal(float lhs, float rhs) {
  return std::fabs(lhs - rhs) < 0.01f;
}

}  // namespace

void MhiProtocolTraceRecorder::configure(const MhiProtocolTraceConfig& config) {
  config_ = config;
  config_.capture_window_ms = std::max<uint32_t>(1000U, config_.capture_window_ms);
  config_.post_timeout_grace_ms = std::max<uint32_t>(250U, config_.post_timeout_grace_ms);
  config_.unchanged_heartbeat_ms = std::max<uint32_t>(250U, config_.unchanged_heartbeat_ms);
  config_.pre_command_frames =
      clamp_u8(config_.pre_command_frames, 0U,
               static_cast<uint8_t>(kMhiProtocolTraceMaxPreCommandFrames));
  config_.post_command_frames = clamp_u8(config_.post_command_frames, 1U, 64U);
  config_.max_records =
      clamp_u8(config_.max_records, 8U, static_cast<uint8_t>(kMhiProtocolTraceMaxRecords));
  const uint8_t max_pre_frames = static_cast<uint8_t>(config_.max_records - 4U);
  if (config_.pre_command_frames > max_pre_frames) {
    config_.pre_command_frames = max_pre_frames;
  }
  if (!config_.enabled) {
    this->reset();
  }
}

void MhiProtocolTraceRecorder::reset() {
  records_ = {};
  pre_mosi_ = {};
  deferred_mosi_ = {};
  deferred_mosi_count_ = 0U;
  record_count_ = 0U;
  pre_mosi_head_ = 0U;
  pre_mosi_size_ = 0U;
  baseline_mosi_.clear();
  baseline_miso_.clear();
  last_staged_miso_.clear();
  last_logged_mosi_.clear();
  baseline_semantic_ = {};
  requested_semantic_ = {};
  last_semantic_ = {};
  stable_semantic_ = {};
  active_ = false;
  ready_ = false;
  exporting_ = false;
  tx_staged_seen_ = false;
  tx_completion_seen_ = false;
  awaiting_tx_completion_ = false;
  tx_success_seen_ = false;
  same_transaction_rx_seen_ = false;
  first_rx_after_tx_seen_ = false;
  raw_change_seen_ = false;
  semantic_change_seen_ = false;
  stable_status_seen_ = false;
  requested_state_seen_ = false;
  confirmation_seen_ = false;
  timeout_seen_ = false;
  command_candidate_seen_ = false;
  stable_repeat_count_ = 0U;
  post_status_frames_ = 0U;
  tx_attempt_ = 0U;
  last_heartbeat_ms_ = 0U;
  timeout_seen_ms_ = 0U;
  tx_staged_us_ = 0U;
  tx_end_us_ = 0U;
  tx_bus_sequence_ = 0U;
  capture_started_ms_ = 0U;
  capture_started_us_ = 0U;
  capture_command_mask_ = 0U;
  capture_generation_ = 0U;
  completion_reason_ = MhiProtocolTraceCompletionReason::NONE;
  capture_label_[0] = '\0';
  stats_ = {};
  summary_ = {};
}

bool MhiProtocolTraceRecorder::start_capture(
    uint32_t command_mask, const MhiProtocolTraceSemanticState& requested,
    uint32_t confirmation_pending_mask, uint32_t now_ms, uint32_t now_us,
    const char* label) {
  if (!config_.enabled) {
    return false;
  }
  if (active_ || ready_ || exporting_) {
    stats_.busy_rejections++;
    return false;
  }

  records_ = {};
  deferred_mosi_ = {};
  deferred_mosi_count_ = 0U;
  record_count_ = 0U;
  active_ = true;
  ready_ = false;
  tx_staged_seen_ = false;
  tx_completion_seen_ = false;
  awaiting_tx_completion_ = false;
  tx_success_seen_ = false;
  same_transaction_rx_seen_ = false;
  first_rx_after_tx_seen_ = false;
  raw_change_seen_ = false;
  semantic_change_seen_ = false;
  stable_status_seen_ = false;
  requested_state_seen_ = false;
  confirmation_seen_ = false;
  timeout_seen_ = false;
  command_candidate_seen_ = false;
  stable_repeat_count_ = 0U;
  post_status_frames_ = 0U;
  tx_attempt_ = 0U;
  last_heartbeat_ms_ = now_ms;
  timeout_seen_ms_ = 0U;
  tx_staged_us_ = 0U;
  tx_end_us_ = 0U;
  tx_bus_sequence_ = 0U;
  last_staged_miso_.clear();
  last_logged_mosi_.clear();
  last_semantic_ = baseline_semantic_;
  stable_semantic_ = {};
  requested_semantic_ = requested;
  summary_ = {};
  capture_id_++;
  capture_started_ms_ = now_ms;
  capture_started_us_ = now_us;
  capture_command_mask_ = command_mask;
  capture_generation_ = 0U;
  completion_reason_ = MhiProtocolTraceCompletionReason::NONE;
  copy_label_(capture_label_, label);
  stats_.captures_started++;

  this->copy_pre_command_records_(now_us);
  this->append_record_(MhiProtocolTraceEvent::COMMAND_REQUESTED,
                       MhiProtocolTraceDirection::NONE, command_mask,
                       confirmation_pending_mask, 0U, 0U, 0U, now_us, now_us,
                       nullptr, &requested, true, true, false, false, 0U, 0U,
                       label);
  return true;
}

bool MhiProtocolTraceRecorder::mark_physical_result(
    const char* label, uint32_t now_us, uint32_t confirmation_pending_mask) {
  if (!active_) {
    return false;
  }
  return this->append_record_(
      MhiProtocolTraceEvent::PHYSICAL_RESULT_MARKER,
      MhiProtocolTraceDirection::NONE, capture_command_mask_,
      confirmation_pending_mask, capture_generation_, 0U, 0U, now_us, now_us,
      nullptr, nullptr, true, true, false, false, 0U, 0U, label);
}

void MhiProtocolTraceRecorder::force_complete(uint32_t now_us) {
  if (active_) {
    this->complete_(MhiProtocolTraceCompletionReason::FORCED, now_us);
  }
}

void MhiProtocolTraceRecorder::poll(uint32_t now_ms, uint32_t now_us) {
  if (!active_) {
    return;
  }

  if (timeout_seen_ && timeout_seen_ms_ != 0U &&
      (now_ms - timeout_seen_ms_) >= config_.post_timeout_grace_ms) {
    this->complete_(MhiProtocolTraceCompletionReason::TIMEOUT_GRACE, now_us);
    return;
  }

  if ((now_ms - capture_started_ms_) >= config_.capture_window_ms) {
    this->complete_(MhiProtocolTraceCompletionReason::CAPTURE_WINDOW, now_us);
  }
}

void MhiProtocolTraceRecorder::observe_tx_staged(
    const MhiTxEnvelope& envelope, uint32_t now_us,
    uint32_t confirmation_pending_mask) {
  if (!config_.enabled || !envelope.valid()) {
    return;
  }

  MhiFrameBuffer frame{};
  frame.len = envelope.len;
  std::memcpy(frame.data, envelope.frame.data(), envelope.len);

  if (!active_) {
    if (!ready_ && !exporting_ && !envelope.is_command()) {
      this->store_background_miso_(frame);
    }
    return;
  }
  if (!envelope.is_command() || capture_command_mask_ == 0U ||
      (envelope.command_mask & capture_command_mask_) == 0U) {
    return;
  }

  if (!this->direction_signature_valid_(
          MhiProtocolTraceDirection::MISO_CONTROLLER_TO_AC, frame)) {
    summary_.direction_errors++;
  }
  if (!this->checksum_valid_(frame)) {
    summary_.checksum_errors++;
  }

  tx_staged_seen_ = true;
  awaiting_tx_completion_ = true;
  tx_attempt_++;
  summary_.tx_attempts++;
  capture_generation_ = envelope.generation;
  last_staged_miso_ = frame;
  tx_staged_us_ = now_us;
  if (summary_.request_to_stage_us == 0U) {
    summary_.request_to_stage_us = elapsed_(now_us, capture_started_us_);
  }

  this->append_record_(
      MhiProtocolTraceEvent::MISO_FRAME_STAGED,
      MhiProtocolTraceDirection::MISO_CONTROLLER_TO_AC, envelope.command_mask,
      confirmation_pending_mask, envelope.generation, 0U, 0U, now_us, now_us,
      &frame, nullptr, true, true, false, false,
      static_cast<uint16_t>(envelope.len), 0U,
      tx_attempt_ > 1U ? "retry" : "initial");
}

void MhiProtocolTraceRecorder::observe_tx_completion(
    const MhiTxCompletion& completion, uint32_t observed_at_us,
    uint32_t confirmation_pending_mask) {
  if (!active_ || !completion.is_command() || capture_command_mask_ == 0U ||
      (completion.command_mask & capture_command_mask_) == 0U) {
    return;
  }

  tx_completion_seen_ = true;
  capture_generation_ = completion.generation;
  const uint32_t event_time_us = completion.frame_end_us != 0U
                                     ? completion.frame_end_us
                                     : (completion.completed_at_us != 0U
                                            ? completion.completed_at_us
                                            : observed_at_us);

  MhiFrameBuffer transmitted{};
  if (completion.transmitted_len == kMhiFrame20Bytes ||
      completion.transmitted_len == kMhiFrame33Bytes) {
    transmitted.len = completion.transmitted_len;
    std::memcpy(transmitted.data, completion.transmitted_frame.data(),
                completion.transmitted_len);
  }

  if (transmitted.view().valid()) {
    if (!this->direction_signature_valid_(
            MhiProtocolTraceDirection::MISO_CONTROLLER_TO_AC, transmitted)) {
      summary_.direction_errors++;
    }
    if (!this->checksum_valid_(transmitted)) {
      summary_.checksum_errors++;
    }
  }

  if (completion.success) {
    tx_success_seen_ = true;
    summary_.tx_successes++;
    tx_end_us_ = event_time_us;
    tx_bus_sequence_ = completion.bus_sequence;
    if (summary_.stage_to_wire_us == 0U && tx_staged_us_ != 0U) {
      summary_.stage_to_wire_us = elapsed_(event_time_us, tx_staged_us_);
    }
  } else {
    summary_.tx_failures++;
  }

  if (transmitted.view().valid() && last_staged_miso_.view().valid() &&
      !this->frame_matches_(transmitted, last_staged_miso_)) {
    summary_.on_wire_frame_mismatch = true;
  }

  const MhiFrameBuffer* frame = transmitted.view().valid()
                                    ? &transmitted
                                    : (last_staged_miso_.view().valid()
                                           ? &last_staged_miso_
                                           : nullptr);
  awaiting_tx_completion_ = false;
  const char* completion_label = nullptr;
  if (summary_.on_wire_frame_mismatch) {
    completion_label = "staged_on_wire_mismatch";
  } else if (!completion.success && !transmitted.view().valid()) {
    completion_label = "staged_reference_only";
  }
  this->append_record_(
      completion.success ? MhiProtocolTraceEvent::MISO_FRAME_TRANSMITTED
                         : MhiProtocolTraceEvent::MISO_TRANSMISSION_FAILED,
      MhiProtocolTraceDirection::MISO_CONTROLLER_TO_AC, completion.command_mask,
      confirmation_pending_mask, completion.generation, 0U,
      completion.bus_sequence, event_time_us, observed_at_us, frame, nullptr,
      completion.success, true, false, false, completion.expected_len,
      completion.actual_len, completion_label);
  this->replay_deferred_mosi_();
}

void MhiProtocolTraceRecorder::observe_staged_timeout(
    uint32_t timeout_mask, uint32_t now_us, uint32_t confirmation_pending_mask) {
  if (!active_ || timeout_mask == 0U ||
      (timeout_mask & capture_command_mask_) == 0U) {
    return;
  }
  summary_.staged_timeout_seen = true;
  this->append_record_(
      MhiProtocolTraceEvent::STAGED_TX_TIMEOUT,
      MhiProtocolTraceDirection::NONE, timeout_mask, confirmation_pending_mask,
      capture_generation_, 0U, 0U, now_us, now_us, nullptr, nullptr, false,
      true, false, false, 0U, 0U, "not_clocked_yet");
}

void MhiProtocolTraceRecorder::defer_mosi_observation_(
    const MhiFrameBuffer& frame, const MhiDecodedStatus& decoded,
    bool command_candidate, uint32_t catalog_sequence, uint32_t bus_sequence,
    uint32_t frame_end_us, uint32_t now_ms, uint32_t observed_at_us,
    uint32_t confirmation_pending_mask) {
  summary_.rx_while_waiting_for_completion++;
  if (deferred_mosi_count_ == deferred_mosi_.size()) {
    for (std::size_t i = 1U; i < deferred_mosi_.size(); i++) {
      deferred_mosi_[i - 1U] = deferred_mosi_[i];
    }
    deferred_mosi_count_--;
    summary_.deferred_mosi_overwrites++;
  }

  auto& observation = deferred_mosi_[deferred_mosi_count_++];
  observation.frame = frame;
  observation.decoded = decoded;
  observation.command_candidate = command_candidate;
  observation.catalog_sequence = catalog_sequence;
  observation.bus_sequence = bus_sequence;
  observation.frame_end_us = frame_end_us;
  observation.now_ms = now_ms;
  observation.observed_at_us = observed_at_us;
  observation.confirmation_pending_mask = confirmation_pending_mask;
}

void MhiProtocolTraceRecorder::replay_deferred_mosi_() {
  const auto observations = deferred_mosi_;
  const std::size_t count = deferred_mosi_count_;
  deferred_mosi_ = {};
  deferred_mosi_count_ = 0U;

  for (std::size_t i = 0U; i < count && active_; i++) {
    const auto& observation = observations[i];
    this->observe_mosi_status(
        observation.frame, observation.decoded, observation.command_candidate,
        observation.catalog_sequence, observation.bus_sequence,
        observation.frame_end_us, observation.now_ms,
        observation.observed_at_us, observation.confirmation_pending_mask);
  }
}

void MhiProtocolTraceRecorder::observe_mosi_status(
    const MhiFrameBuffer& frame, const MhiDecodedStatus& decoded,
    bool command_candidate, uint32_t catalog_sequence, uint32_t bus_sequence,
    uint32_t frame_end_us, uint32_t now_ms, uint32_t observed_at_us,
    uint32_t confirmation_pending_mask) {
  if (!config_.enabled || !frame.view().valid()) {
    return;
  }

  const uint32_t event_time_us = frame_end_us != 0U ? frame_end_us : observed_at_us;
  if (!active_) {
    if (!ready_ && !exporting_) {
      this->store_pre_command_mosi_(frame, decoded, catalog_sequence,
                                    bus_sequence, event_time_us, observed_at_us);
    }
    return;
  }

  if (capture_command_mask_ != 0U && awaiting_tx_completion_) {
    this->defer_mosi_observation_(
        frame, decoded, command_candidate, catalog_sequence, bus_sequence,
        frame_end_us, now_ms, observed_at_us, confirmation_pending_mask);
    return;
  }

  const bool signature_valid = this->direction_signature_valid_(
      MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER, frame);
  const bool checksum_valid = this->checksum_valid_(frame);
  if (!signature_valid) {
    summary_.direction_errors++;
  }
  if (!checksum_valid) {
    summary_.checksum_errors++;
  }
  if (!decoded.valid) {
    summary_.decoder_failures++;
  }
  if (command_candidate) {
    summary_.command_candidates++;
  }

  const MhiProtocolTraceSemanticState semantic = semantic_from_status(decoded);
  const bool raw_changed = baseline_mosi_.view().valid() &&
                           !this->frame_matches_(frame, baseline_mosi_);
  const bool semantic_changed = decoded.valid &&
                                this->semantic_changed_from_baseline_(semantic);
  const bool requested_match = decoded.valid &&
                               this->requested_state_matches_(semantic);

  bool logged = false;
  bool feedback_eligible = capture_command_mask_ == 0U;
  if (tx_success_seen_) {
    const bool same_transaction = tx_bus_sequence_ != 0U &&
                                  bus_sequence == tx_bus_sequence_;
    const bool after_tx_sequence =
        (tx_bus_sequence_ != 0U && bus_sequence > tx_bus_sequence_) ||
        ((tx_bus_sequence_ == 0U || bus_sequence == 0U) &&
         event_time_us != tx_end_us_ && elapsed_(event_time_us, tx_end_us_) < 0x80000000UL);

    if (same_transaction && !same_transaction_rx_seen_) {
      same_transaction_rx_seen_ = true;
      summary_.same_transaction_rx_seen = true;
      this->append_record_(
          MhiProtocolTraceEvent::MOSI_DURING_TX_TRANSACTION,
          MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER,
          capture_command_mask_, confirmation_pending_mask, capture_generation_,
          catalog_sequence, bus_sequence, event_time_us, observed_at_us, &frame,
          &semantic, true, decoded.valid, command_candidate, false, 0U,
          static_cast<uint16_t>(frame.len), "not_ack_eligible");
      logged = true;
    }

    feedback_eligible = after_tx_sequence;
    if (feedback_eligible) {
      summary_.rx_frames_after_tx++;
      if (!first_rx_after_tx_seen_) {
        first_rx_after_tx_seen_ = true;
        summary_.tx_to_first_rx_us = elapsed_(event_time_us, tx_end_us_);
        this->append_record_(
            MhiProtocolTraceEvent::FIRST_MOSI_AFTER_TX,
            MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER,
            capture_command_mask_, confirmation_pending_mask, capture_generation_,
            catalog_sequence, bus_sequence, event_time_us, observed_at_us, &frame,
            &semantic, true, decoded.valid, command_candidate, requested_match,
            0U, static_cast<uint16_t>(frame.len), nullptr);
        logged = true;
      }
    }
  }

  if (feedback_eligible && raw_changed) {
    if (!raw_change_seen_) {
      raw_change_seen_ = true;
      summary_.tx_to_first_raw_change_us =
          tx_end_us_ == 0U ? 0U : elapsed_(event_time_us, tx_end_us_);
    }
    summary_.raw_changes++;
    if (!last_logged_mosi_.view().valid() ||
        !this->frame_matches_(frame, last_logged_mosi_)) {
      this->append_record_(
          MhiProtocolTraceEvent::MOSI_RAW_CHANGE,
          MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER,
          capture_command_mask_, confirmation_pending_mask, capture_generation_,
          catalog_sequence, bus_sequence, event_time_us, observed_at_us, &frame,
          &semantic, true, decoded.valid, command_candidate, requested_match,
          0U, static_cast<uint16_t>(frame.len), nullptr);
      last_logged_mosi_ = frame;
      logged = true;
    }
  }

  if (feedback_eligible && semantic_changed) {
    if (!semantic_change_seen_) {
      semantic_change_seen_ = true;
      summary_.tx_to_first_semantic_change_us =
          tx_end_us_ == 0U ? 0U : elapsed_(event_time_us, tx_end_us_);
    }
    summary_.semantic_changes++;
    if (!this->semantic_matches_(semantic, last_semantic_,
                                 semantic.valid_fields | last_semantic_.valid_fields)) {
      this->append_record_(
          MhiProtocolTraceEvent::MOSI_SEMANTIC_CHANGE,
          MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER,
          capture_command_mask_, confirmation_pending_mask, capture_generation_,
          catalog_sequence, bus_sequence, event_time_us, observed_at_us, &frame,
          &semantic, true, decoded.valid, command_candidate, requested_match,
          0U, static_cast<uint16_t>(frame.len), nullptr);
      last_semantic_ = semantic;
      logged = true;
    }
  }

  if (feedback_eligible && command_candidate && !command_candidate_seen_) {
    command_candidate_seen_ = true;
    this->append_record_(
        MhiProtocolTraceEvent::MOSI_COMMAND_CANDIDATE,
        MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER,
        capture_command_mask_, confirmation_pending_mask, capture_generation_,
        catalog_sequence, bus_sequence, event_time_us, observed_at_us, &frame,
        &semantic, true, decoded.valid, true, requested_match, 0U,
        static_cast<uint16_t>(frame.len), "first_candidate");
    logged = true;
  }

  if (feedback_eligible && requested_match) {
    requested_state_seen_ = true;
    summary_.requested_state_seen = true;
  }

  if (feedback_eligible && (raw_changed || semantic_changed) && decoded.valid) {
    const uint32_t compare_fields = semantic.valid_fields | stable_semantic_.valid_fields;
    if (stable_repeat_count_ > 0U &&
        this->semantic_matches_(semantic, stable_semantic_, compare_fields)) {
      stable_repeat_count_++;
    } else {
      stable_semantic_ = semantic;
      stable_repeat_count_ = 1U;
    }

    if (!stable_status_seen_ && stable_repeat_count_ >= 3U) {
      stable_status_seen_ = true;
      summary_.stable_status_seen = true;
      this->append_record_(
          MhiProtocolTraceEvent::MOSI_STABLE_STATUS,
          MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER,
          capture_command_mask_, confirmation_pending_mask, capture_generation_,
          catalog_sequence, bus_sequence, event_time_us, observed_at_us, &frame,
          &semantic, true, decoded.valid, command_candidate, requested_match,
          0U, static_cast<uint16_t>(frame.len), nullptr);
      logged = true;
      if (capture_command_mask_ == 0U) {
        this->complete_(MhiProtocolTraceCompletionReason::PASSIVE_STABLE_STATUS,
                        observed_at_us);
        return;
      }
      if (confirmation_seen_) {
        this->complete_(MhiProtocolTraceCompletionReason::CONFIRMED,
                        observed_at_us);
        return;
      }
    }
  }

  if (!logged && (now_ms - last_heartbeat_ms_) >= config_.unchanged_heartbeat_ms) {
    last_heartbeat_ms_ = now_ms;
    this->append_record_(
        MhiProtocolTraceEvent::MOSI_STATUS_HEARTBEAT,
        MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER,
        capture_command_mask_, confirmation_pending_mask, capture_generation_,
        catalog_sequence, bus_sequence, event_time_us, observed_at_us, &frame,
        &semantic, true, decoded.valid, command_candidate, requested_match,
        0U, static_cast<uint16_t>(frame.len), "waiting");
  }

  if (post_status_frames_ < 0xFFU) {
    post_status_frames_++;
  }
  if (capture_command_mask_ == 0U && raw_change_seen_ &&
      post_status_frames_ >= config_.post_command_frames) {
    this->complete_(MhiProtocolTraceCompletionReason::CAPTURE_WINDOW,
                    observed_at_us);
    return;
  }

  this->poll(now_ms, observed_at_us);
}

void MhiProtocolTraceRecorder::observe_confirmation(
    uint32_t confirmed_mask, uint32_t pending_mask, uint32_t now_us) {
  if (!active_ || confirmed_mask == 0U ||
      (confirmed_mask & capture_command_mask_) == 0U) {
    return;
  }
  confirmation_seen_ = true;
  summary_.confirmations++;
  summary_.tx_to_confirmation_us = tx_end_us_ == 0U ? 0U : elapsed_(now_us, tx_end_us_);
  if (timeout_seen_) {
    summary_.late_confirmation = true;
  }
  this->append_record_(
      timeout_seen_ ? MhiProtocolTraceEvent::LATE_CONFIRMATION_MATCHED
                    : MhiProtocolTraceEvent::CONFIRMATION_MATCHED,
      MhiProtocolTraceDirection::NONE, confirmed_mask, pending_mask,
      capture_generation_, 0U, 0U, now_us, now_us, nullptr, nullptr, true,
      true, false, requested_state_seen_, 0U, 0U, nullptr);

  if (stable_status_seen_ || requested_state_seen_) {
    this->complete_(MhiProtocolTraceCompletionReason::CONFIRMED, now_us);
  }
}

void MhiProtocolTraceRecorder::observe_confirmation_timeout(
    uint32_t timeout_mask, uint32_t pending_mask, uint32_t now_us) {
  if (!active_ || timeout_mask == 0U ||
      (timeout_mask & capture_command_mask_) == 0U) {
    return;
  }
  timeout_seen_ = true;
  timeout_seen_ms_ = capture_started_ms_ + (elapsed_(now_us, capture_started_us_) / 1000U);
  summary_.confirmation_timeouts++;
  summary_.tx_to_timeout_us = tx_end_us_ == 0U ? 0U : elapsed_(now_us, tx_end_us_);
  this->append_record_(
      MhiProtocolTraceEvent::CONFIRMATION_TIMEOUT,
      MhiProtocolTraceDirection::NONE, timeout_mask, pending_mask,
      capture_generation_, 0U, 0U, now_us, now_us, nullptr, nullptr, false,
      true, false, requested_state_seen_, 0U, 0U, "grace_window_open");
}

bool MhiProtocolTraceRecorder::begin_export() {
  if (!ready_ || exporting_) {
    return false;
  }
  exporting_ = true;
  return true;
}

void MhiProtocolTraceRecorder::finish_export() {
  if (!exporting_) {
    return;
  }
  exporting_ = false;
  ready_ = false;
  record_count_ = 0U;
  completion_reason_ = MhiProtocolTraceCompletionReason::NONE;
}

MhiProtocolTraceSemanticState MhiProtocolTraceRecorder::semantic_from_command(
    const MhiCommandState& command, uint32_t mask) {
  MhiProtocolTraceSemanticState out{};
  if ((mask & MHI_COMMAND_POWER) != 0U) {
    out.valid_fields |= MHI_TRACE_STATE_POWER;
    out.power = command.power;
  }
  if ((mask & MHI_COMMAND_MODE) != 0U) {
    out.valid_fields |= MHI_TRACE_STATE_MODE;
    out.mode = command.mode;
  }
  if ((mask & MHI_COMMAND_FAN) != 0U) {
    out.valid_fields |= MHI_TRACE_STATE_FAN;
    out.fan = command.fan;
  }
  if ((mask & MHI_COMMAND_TARGET_TEMP) != 0U) {
    out.valid_fields |= MHI_TRACE_STATE_TARGET_TEMP;
    out.target_temp_c = command.target_temp_c;
  }
  if ((mask & MHI_COMMAND_VERTICAL_VANE) != 0U) {
    out.valid_fields |= MHI_TRACE_STATE_VERTICAL_VANE;
    out.vertical_vane = command.vertical_vane;
  }
  if ((mask & MHI_COMMAND_HORIZONTAL_VANE) != 0U) {
    out.valid_fields |= MHI_TRACE_STATE_HORIZONTAL_VANE |
                        MHI_TRACE_STATE_HORIZONTAL_SWING;
    out.horizontal_vane = command.horizontal_vane;
    out.horizontal_swing = command.horizontal_vane == 8U;
  }
  if ((mask & MHI_COMMAND_THREE_D_AUTO) != 0U) {
    out.valid_fields |= MHI_TRACE_STATE_THREE_D_AUTO;
    out.three_d_auto = command.three_d_auto;
  }
  if ((mask & MHI_COMMAND_ROOM_TEMP_OVERRIDE) != 0U) {
    out.valid_fields |= MHI_TRACE_STATE_ROOM_TEMP_OVERRIDE;
    out.room_temp_override_raw = command.room_temp_override_raw;
  }
  return out;
}

MhiProtocolTraceSemanticState MhiProtocolTraceRecorder::semantic_from_status(
    const MhiDecodedStatus& status) {
  MhiProtocolTraceSemanticState out{};
  if (!status.valid) {
    return out;
  }
  out.valid_fields = MHI_TRACE_STATE_POWER | MHI_TRACE_STATE_MODE |
                     MHI_TRACE_STATE_FAN | MHI_TRACE_STATE_TARGET_TEMP |
                     MHI_TRACE_STATE_VERTICAL_VANE |
                     MHI_TRACE_STATE_VERTICAL_SWING;
  out.power = status.power;
  out.mode = status.mode;
  out.fan = status.fan;
  out.target_temp_c = status.target_temp_c;
  out.vertical_vane = status.vertical_vane;
  out.vertical_swing = status.vertical_swing;
  if (status.has_horizontal_vane) {
    out.valid_fields |= MHI_TRACE_STATE_HORIZONTAL_VANE |
                        MHI_TRACE_STATE_HORIZONTAL_SWING;
    out.horizontal_vane = status.horizontal_vane;
    out.horizontal_swing = status.horizontal_swing;
  }
  if (status.has_3d_auto) {
    out.valid_fields |= MHI_TRACE_STATE_THREE_D_AUTO;
    out.three_d_auto = status.three_d_auto;
  }
  if (status.has_extended_louver_raw) {
    out.valid_fields |= MHI_TRACE_STATE_EXTENDED_RAW;
    out.extended_louver_db16 = status.extended_louver_db16;
    out.extended_louver_db17 = status.extended_louver_db17;
  }
  return out;
}

const char* MhiProtocolTraceRecorder::event_name(MhiProtocolTraceEvent event) {
  switch (event) {
    case MhiProtocolTraceEvent::BASELINE_MOSI: return "BASELINE_MOSI";
    case MhiProtocolTraceEvent::COMMAND_REQUESTED: return "COMMAND_REQUESTED";
    case MhiProtocolTraceEvent::MISO_FRAME_STAGED: return "MISO_FRAME_STAGED";
    case MhiProtocolTraceEvent::MISO_FRAME_TRANSMITTED: return "MISO_FRAME_TRANSMITTED";
    case MhiProtocolTraceEvent::MISO_TRANSMISSION_FAILED: return "MISO_TRANSMISSION_FAILED";
    case MhiProtocolTraceEvent::STAGED_TX_TIMEOUT: return "STAGED_TX_TIMEOUT";
    case MhiProtocolTraceEvent::MOSI_AWAITING_TX_COMPLETION:
      return "MOSI_AWAITING_TX_COMPLETION";
    case MhiProtocolTraceEvent::MOSI_DURING_TX_TRANSACTION:
      return "MOSI_DURING_TX_TRANSACTION";
    case MhiProtocolTraceEvent::FIRST_MOSI_AFTER_TX: return "FIRST_MOSI_AFTER_TX";
    case MhiProtocolTraceEvent::MOSI_STATUS_HEARTBEAT: return "MOSI_STATUS_HEARTBEAT";
    case MhiProtocolTraceEvent::MOSI_RAW_CHANGE: return "MOSI_RAW_CHANGE";
    case MhiProtocolTraceEvent::MOSI_SEMANTIC_CHANGE: return "MOSI_SEMANTIC_CHANGE";
    case MhiProtocolTraceEvent::MOSI_COMMAND_CANDIDATE: return "MOSI_COMMAND_CANDIDATE";
    case MhiProtocolTraceEvent::MOSI_STABLE_STATUS: return "MOSI_STABLE_STATUS";
    case MhiProtocolTraceEvent::CONFIRMATION_MATCHED: return "CONFIRMATION_MATCHED";
    case MhiProtocolTraceEvent::LATE_CONFIRMATION_MATCHED: return "LATE_CONFIRMATION_MATCHED";
    case MhiProtocolTraceEvent::CONFIRMATION_TIMEOUT: return "CONFIRMATION_TIMEOUT";
    case MhiProtocolTraceEvent::PHYSICAL_RESULT_MARKER: return "PHYSICAL_RESULT_MARKER";
    case MhiProtocolTraceEvent::CAPTURE_COMPLETE: return "CAPTURE_COMPLETE";
    default: return "UNKNOWN";
  }
}

const char* MhiProtocolTraceRecorder::direction_name(
    MhiProtocolTraceDirection direction) {
  switch (direction) {
    case MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER:
      return "MOSI_AC_TO_CONTROLLER";
    case MhiProtocolTraceDirection::MISO_CONTROLLER_TO_AC:
      return "MISO_CONTROLLER_TO_AC";
    default: return "NONE";
  }
}

const char* MhiProtocolTraceRecorder::completion_reason_name(
    MhiProtocolTraceCompletionReason reason) {
  switch (reason) {
    case MhiProtocolTraceCompletionReason::PASSIVE_STABLE_STATUS:
      return "passive_stable_status";
    case MhiProtocolTraceCompletionReason::CONFIRMED: return "confirmed";
    case MhiProtocolTraceCompletionReason::TIMEOUT_GRACE: return "timeout_grace";
    case MhiProtocolTraceCompletionReason::CAPTURE_WINDOW: return "capture_window";
    case MhiProtocolTraceCompletionReason::RECORD_LIMIT: return "record_limit";
    case MhiProtocolTraceCompletionReason::FORCED: return "forced";
    default: return "none";
  }
}

const char* MhiProtocolTraceRecorder::assessment_name(
    MhiProtocolTraceAssessment assessment) {
  switch (assessment) {
    case MhiProtocolTraceAssessment::PASSIVE_CHANGE_CAPTURED:
      return "passive_change_captured";
    case MhiProtocolTraceAssessment::NO_CHANGE_OBSERVED: return "no_change_observed";
    case MhiProtocolTraceAssessment::TX_NOT_STAGED: return "tx_not_staged";
    case MhiProtocolTraceAssessment::TX_NOT_CLOCKED: return "tx_not_clocked";
    case MhiProtocolTraceAssessment::TX_COMPLETION_MISSING:
      return "tx_completion_missing";
    case MhiProtocolTraceAssessment::TX_FAILED: return "tx_failed";
    case MhiProtocolTraceAssessment::ON_WIRE_FRAME_MISMATCH:
      return "on_wire_frame_mismatch";
    case MhiProtocolTraceAssessment::DIRECTION_OR_SIGNATURE_ERROR:
      return "direction_or_signature_error";
    case MhiProtocolTraceAssessment::NO_RX_AFTER_TX: return "no_rx_after_tx";
    case MhiProtocolTraceAssessment::NO_RAW_FEEDBACK_CHANGE:
      return "no_raw_feedback_change";
    case MhiProtocolTraceAssessment::RAW_CHANGE_NOT_DECODED:
      return "raw_change_not_decoded";
    case MhiProtocolTraceAssessment::DECODED_STATE_MISMATCH:
      return "decoded_state_mismatch";
    case MhiProtocolTraceAssessment::CONFIRMATION_LOGIC_MISMATCH:
      return "confirmation_logic_mismatch";
    case MhiProtocolTraceAssessment::ACK_AFTER_TIMEOUT: return "ack_after_timeout";
    case MhiProtocolTraceAssessment::CONFIRMED: return "confirmed";
    default: return "inconclusive";
  }
}

bool MhiProtocolTraceRecorder::append_record_(
    MhiProtocolTraceEvent event, MhiProtocolTraceDirection direction,
    uint32_t command_mask, uint32_t pending_mask, uint32_t generation,
    uint32_t catalog_sequence, uint32_t bus_sequence, uint32_t event_time_us,
    uint32_t observed_at_us, const MhiFrameBuffer* frame,
    const MhiProtocolTraceSemanticState* semantic, bool success,
    bool decoder_valid, bool command_candidate, bool requested_state_match,
    uint16_t expected_len, uint16_t actual_len, const char* label) {
  if (record_count_ >= config_.max_records || record_count_ >= records_.size()) {
    stats_.records_dropped++;
    if (active_) {
      this->complete_(MhiProtocolTraceCompletionReason::RECORD_LIMIT,
                      observed_at_us);
    }
    return false;
  }

  auto& record = records_[record_count_++];
  record = {};
  record.observed_at_us = observed_at_us;
  record.event_time_us = event_time_us == 0U ? observed_at_us : event_time_us;
  record.elapsed_us = elapsed_(record.event_time_us, capture_started_us_);
  record.processing_delay_us = elapsed_(observed_at_us, record.event_time_us);
  record.since_tx_us = tx_end_us_ == 0U ? 0U : elapsed_(record.event_time_us, tx_end_us_);
  record.catalog_sequence = catalog_sequence;
  record.bus_sequence = bus_sequence;
  record.command_generation = generation;
  record.tx_attempt = tx_attempt_;
  record.event = event;
  record.direction = direction;
  record.command_mask = command_mask;
  record.confirmation_pending_mask = pending_mask;
  record.success = success;
  record.decoder_valid = decoder_valid;
  record.command_candidate = command_candidate;
  record.requested_state_match = requested_state_match;
  record.expected_len = expected_len;
  record.actual_len = actual_len;
  if (frame != nullptr) {
    record.frame = *frame;
    record.changed_byte_mask = this->changed_byte_mask_(direction, *frame);
    record.signature_valid = this->direction_signature_valid_(direction, *frame);
    record.checksum_valid = this->checksum_valid_(*frame);
  }
  if (semantic != nullptr) {
    record.semantic = *semantic;
  }
  copy_label_(record.label, label);
  return true;
}

void MhiProtocolTraceRecorder::complete_(
    MhiProtocolTraceCompletionReason reason, uint32_t now_us) {
  if (!active_) {
    return;
  }
  completion_reason_ = reason;
  if (!tx_completion_seen_ && deferred_mosi_count_ > 0U &&
      record_count_ < config_.max_records && record_count_ < records_.size()) {
    const auto& observation = deferred_mosi_[deferred_mosi_count_ - 1U];
    const auto semantic = semantic_from_status(observation.decoded);
    this->append_record_(
        MhiProtocolTraceEvent::MOSI_AWAITING_TX_COMPLETION,
        MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER, capture_command_mask_,
        observation.confirmation_pending_mask, capture_generation_,
        observation.catalog_sequence, observation.bus_sequence,
        observation.frame_end_us, observation.observed_at_us,
        &observation.frame, &semantic, true, observation.decoded.valid,
        observation.command_candidate, false, 0U,
        static_cast<uint16_t>(observation.frame.len),
        "unresolved_before_tx_completion");
  }
  this->assess_();
  active_ = false;
  if (record_count_ < config_.max_records && record_count_ < records_.size()) {
    this->append_record_(
        MhiProtocolTraceEvent::CAPTURE_COMPLETE,
        MhiProtocolTraceDirection::NONE, capture_command_mask_, 0U,
        capture_generation_, 0U, tx_bus_sequence_, now_us, now_us, nullptr,
        nullptr, true, true, false, requested_state_seen_, 0U, 0U,
        assessment_name(summary_.assessment));
  }
  ready_ = true;
  stats_.captures_completed++;
}

void MhiProtocolTraceRecorder::assess_() {
  if (capture_command_mask_ == 0U) {
    summary_.assessment = (raw_change_seen_ || semantic_change_seen_)
                              ? MhiProtocolTraceAssessment::PASSIVE_CHANGE_CAPTURED
                              : MhiProtocolTraceAssessment::NO_CHANGE_OBSERVED;
    return;
  }
  if (!tx_staged_seen_) {
    summary_.assessment = MhiProtocolTraceAssessment::TX_NOT_STAGED;
    return;
  }
  if (!tx_completion_seen_) {
    summary_.assessment = summary_.rx_while_waiting_for_completion == 0U
                              ? MhiProtocolTraceAssessment::TX_NOT_CLOCKED
                              : MhiProtocolTraceAssessment::TX_COMPLETION_MISSING;
    return;
  }
  if (!tx_success_seen_) {
    summary_.assessment = MhiProtocolTraceAssessment::TX_FAILED;
    return;
  }
  if (summary_.on_wire_frame_mismatch) {
    summary_.assessment = MhiProtocolTraceAssessment::ON_WIRE_FRAME_MISMATCH;
    return;
  }
  if (summary_.direction_errors != 0U) {
    summary_.assessment = MhiProtocolTraceAssessment::DIRECTION_OR_SIGNATURE_ERROR;
    return;
  }
  if (!first_rx_after_tx_seen_) {
    summary_.assessment = MhiProtocolTraceAssessment::NO_RX_AFTER_TX;
    return;
  }
  if (confirmation_seen_) {
    summary_.assessment = timeout_seen_ ? MhiProtocolTraceAssessment::ACK_AFTER_TIMEOUT
                                        : MhiProtocolTraceAssessment::CONFIRMED;
    return;
  }
  if (timeout_seen_ && requested_state_seen_) {
    summary_.assessment = MhiProtocolTraceAssessment::CONFIRMATION_LOGIC_MISMATCH;
    return;
  }
  if (!raw_change_seen_) {
    summary_.assessment = MhiProtocolTraceAssessment::NO_RAW_FEEDBACK_CHANGE;
    return;
  }
  if (!semantic_change_seen_ || summary_.decoder_failures != 0U) {
    summary_.assessment = MhiProtocolTraceAssessment::RAW_CHANGE_NOT_DECODED;
    return;
  }
  if (!requested_state_seen_) {
    summary_.assessment = MhiProtocolTraceAssessment::DECODED_STATE_MISMATCH;
    return;
  }
  summary_.assessment = MhiProtocolTraceAssessment::INCONCLUSIVE;
}

void MhiProtocolTraceRecorder::store_pre_command_mosi_(
    const MhiFrameBuffer& frame, const MhiDecodedStatus& decoded,
    uint32_t catalog_sequence, uint32_t bus_sequence, uint32_t frame_end_us,
    uint32_t observed_at_us) {
  baseline_mosi_ = frame;
  baseline_semantic_ = semantic_from_status(decoded);
  if (config_.pre_command_frames == 0U) {
    return;
  }

  MhiProtocolTraceRecord record{};
  record.observed_at_us = observed_at_us;
  record.event_time_us = frame_end_us == 0U ? observed_at_us : frame_end_us;
  record.catalog_sequence = catalog_sequence;
  record.bus_sequence = bus_sequence;
  record.event = MhiProtocolTraceEvent::BASELINE_MOSI;
  record.direction = MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER;
  record.success = true;
  record.signature_valid = this->direction_signature_valid_(record.direction, frame);
  record.checksum_valid = this->checksum_valid_(frame);
  record.decoder_valid = decoded.valid;
  record.frame = frame;
  record.semantic = baseline_semantic_;

  pre_mosi_[pre_mosi_head_] = record;
  pre_mosi_head_ = (pre_mosi_head_ + 1U) % pre_mosi_.size();
  if (pre_mosi_size_ < config_.pre_command_frames) {
    pre_mosi_size_++;
  }
}

void MhiProtocolTraceRecorder::store_background_miso_(const MhiFrameBuffer& frame) {
  baseline_miso_ = frame;
}

void MhiProtocolTraceRecorder::copy_pre_command_records_(uint32_t now_us) {
  const std::size_t count = std::min<std::size_t>(pre_mosi_size_, config_.pre_command_frames);
  const std::size_t start = (pre_mosi_head_ + pre_mosi_.size() - count) % pre_mosi_.size();
  for (std::size_t i = 0U; i < count; i++) {
    const auto& source = pre_mosi_[(start + i) % pre_mosi_.size()];
    this->append_record_(
        MhiProtocolTraceEvent::BASELINE_MOSI,
        MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER, 0U, 0U, 0U,
        source.catalog_sequence, source.bus_sequence, now_us, now_us,
        &source.frame, &source.semantic, true, source.decoder_valid, false,
        false, 0U, static_cast<uint16_t>(source.frame.len), nullptr);
  }
}

bool MhiProtocolTraceRecorder::frame_matches_(const MhiFrameBuffer& lhs,
                                              const MhiFrameBuffer& rhs) const {
  return lhs.len == rhs.len && lhs.len != 0U &&
         std::memcmp(lhs.data, rhs.data, lhs.len) == 0;
}

bool MhiProtocolTraceRecorder::semantic_matches_(
    const MhiProtocolTraceSemanticState& lhs,
    const MhiProtocolTraceSemanticState& rhs, uint32_t field_mask) const {
  if ((field_mask & MHI_TRACE_STATE_POWER) != 0U && lhs.power != rhs.power) return false;
  if ((field_mask & MHI_TRACE_STATE_MODE) != 0U && lhs.mode != rhs.mode) return false;
  if ((field_mask & MHI_TRACE_STATE_FAN) != 0U && lhs.fan != rhs.fan) return false;
  if ((field_mask & MHI_TRACE_STATE_TARGET_TEMP) != 0U &&
      !float_equal(lhs.target_temp_c, rhs.target_temp_c)) return false;
  if ((field_mask & MHI_TRACE_STATE_VERTICAL_VANE) != 0U &&
      lhs.vertical_vane != rhs.vertical_vane) return false;
  if ((field_mask & MHI_TRACE_STATE_VERTICAL_SWING) != 0U &&
      lhs.vertical_swing != rhs.vertical_swing) return false;
  if ((field_mask & MHI_TRACE_STATE_HORIZONTAL_VANE) != 0U &&
      lhs.horizontal_vane != rhs.horizontal_vane) return false;
  if ((field_mask & MHI_TRACE_STATE_HORIZONTAL_SWING) != 0U &&
      lhs.horizontal_swing != rhs.horizontal_swing) return false;
  if ((field_mask & MHI_TRACE_STATE_THREE_D_AUTO) != 0U &&
      lhs.three_d_auto != rhs.three_d_auto) return false;
  if ((field_mask & MHI_TRACE_STATE_EXTENDED_RAW) != 0U &&
      (lhs.extended_louver_db16 != rhs.extended_louver_db16 ||
       lhs.extended_louver_db17 != rhs.extended_louver_db17)) return false;
  if ((field_mask & MHI_TRACE_STATE_ROOM_TEMP_OVERRIDE) != 0U &&
      lhs.room_temp_override_raw != rhs.room_temp_override_raw) return false;
  return true;
}

bool MhiProtocolTraceRecorder::semantic_changed_from_baseline_(
    const MhiProtocolTraceSemanticState& semantic) const {
  if (baseline_semantic_.valid_fields == 0U || semantic.valid_fields == 0U) {
    return false;
  }
  const uint32_t comparable = baseline_semantic_.valid_fields & semantic.valid_fields;
  return !this->semantic_matches_(semantic, baseline_semantic_, comparable);
}

bool MhiProtocolTraceRecorder::requested_state_matches_(
    const MhiProtocolTraceSemanticState& semantic) const {
  if (capture_command_mask_ == 0U || requested_semantic_.valid_fields == 0U) {
    return false;
  }
  if ((semantic.valid_fields & requested_semantic_.valid_fields) !=
      requested_semantic_.valid_fields) {
    return false;
  }
  return this->semantic_matches_(semantic, requested_semantic_,
                                 requested_semantic_.valid_fields);
}

bool MhiProtocolTraceRecorder::direction_signature_valid_(
    MhiProtocolTraceDirection direction, const MhiFrameBuffer& frame) const {
  if (!frame.view().valid()) {
    return false;
  }
  if (direction == MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER) {
    const bool first = frame.data[SB0] == kMhiMosiSignature0Default ||
                       frame.data[SB0] == kMhiMosiSignature0Alt;
    return first && frame.data[SB1] == kMhiMosiSignature1 &&
           frame.data[SB2] == kMhiMosiSignature2;
  }
  if (direction == MhiProtocolTraceDirection::MISO_CONTROLLER_TO_AC) {
    return frame.data[SB0] == kMhiMisoSignature0 &&
           frame.data[SB1] == kMhiMisoSignature1 &&
           frame.data[SB2] == kMhiMisoSignature2;
  }
  return false;
}

bool MhiProtocolTraceRecorder::checksum_valid_(const MhiFrameBuffer& frame) const {
  if (frame.len == kMhiFrame20Bytes) {
    return mhi_checksum_valid_20(frame.data);
  }
  if (frame.len == kMhiFrame33Bytes) {
    return mhi_checksum_valid_33(frame.data) || mhi_checksum_valid_20(frame.data);
  }
  return false;
}

uint64_t MhiProtocolTraceRecorder::changed_byte_mask_(
    MhiProtocolTraceDirection direction, const MhiFrameBuffer& frame) const {
  const MhiFrameBuffer* baseline = nullptr;
  if (direction == MhiProtocolTraceDirection::MOSI_AC_TO_CONTROLLER) {
    baseline = &baseline_mosi_;
  } else if (direction == MhiProtocolTraceDirection::MISO_CONTROLLER_TO_AC) {
    baseline = &baseline_miso_;
  }
  if (baseline == nullptr || !baseline->view().valid() || baseline->len != frame.len) {
    return 0U;
  }

  uint64_t mask = 0U;
  for (std::size_t i = 0U; i < frame.len && i < 64U; i++) {
    if (frame.data[i] != baseline->data[i]) {
      mask |= (1ULL << i);
    }
  }
  return mask;
}

void MhiProtocolTraceRecorder::copy_label_(char* destination, const char* source) {
  if (destination == nullptr) {
    return;
  }
  destination[0] = '\0';
  if (source == nullptr) {
    return;
  }
  std::strncpy(destination, source, kMhiProtocolTraceLabelBytes - 1U);
  destination[kMhiProtocolTraceLabelBytes - 1U] = '\0';
}

uint32_t MhiProtocolTraceRecorder::elapsed_(uint32_t later, uint32_t earlier) {
  return later - earlier;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
