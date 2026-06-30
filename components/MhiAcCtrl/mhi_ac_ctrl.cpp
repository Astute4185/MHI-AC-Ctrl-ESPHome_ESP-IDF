#include "mhi_ac_ctrl.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#if defined(USE_ESP32)
#include "esp_chip_info.h"
#endif

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_ac_ctrl";
static const char* const DIAG_TAG = "mhi.diag";
static constexpr uint32_t kDiagLogIntervalMs = 30000U;
static constexpr uint32_t kLoopBudgetUs = 30000U;
static constexpr uint32_t kMaxWorkerFramesPerLoop = 4U;
static constexpr uint32_t kRxWorkerHealthCheckIntervalMs = 1000U;
static constexpr uint32_t kRxWorkerStartupGraceMs = 10000U;
static constexpr uint32_t kRxWorkerStallWarnMs = 3000U;
static constexpr uint32_t kRxWorkerNotDrainingWarnMs = 3000U;
static constexpr uint32_t kRxWorkerFrameAgeHealthyMs = 5000U;
static constexpr uint32_t kNoPendingExtendedFeedbackMask = MHI_COMMAND_HORIZONTAL_VANE | MHI_COMMAND_THREE_D_AUTO;
static constexpr uint8_t kStableStartupExtendedFeedbackRepeats = 3U;
static constexpr uint32_t kExtendedLouverBootstrapDelayMs = kRxWorkerStartupGraceMs;
static constexpr uint8_t kStableCommandExtendedFeedbackRepeats = 3U;

namespace {

bool in_range(float value, float min_value, float max_value) {
  return value >= min_value && value <= max_value;
}

}  // namespace

bool MhiAcCtrl::request_horizontal_vane_command(uint8_t horizontal_vane) {
  if (horizontal_vane < 1U || horizontal_vane > 8U) {
    ESP_LOGW(DIAG_TAG, "command: unsupported horizontal vane request value=%u",
             static_cast<unsigned int>(horizontal_vane));
    return false;
  }

  auto& command = this->state_.command();
  const uint32_t queued_mask = command.pending_command_mask();
  const uint32_t pending_mask = this->command_confirmation_.pending_mask();
  const auto& pending_intent = this->command_confirmation_.pending_intent();

  if (command.horizontal_vane_set && command.horizontal_vane == horizontal_vane) {
    ESP_LOGD(DIAG_TAG, "command: duplicate queued horizontal vane request ignored value=%u",
             static_cast<unsigned int>(horizontal_vane));
    return false;
  }

  if ((pending_mask & MHI_COMMAND_HORIZONTAL_VANE) != 0U && pending_intent.horizontal_vane == horizontal_vane) {
    ESP_LOGD(DIAG_TAG, "command: duplicate pending horizontal vane request ignored value=%u",
             static_cast<unsigned int>(horizontal_vane));
    return false;
  }

  const bool extended_louver_busy = ((queued_mask | pending_mask) & kNoPendingExtendedFeedbackMask) != 0U;
  if (!extended_louver_busy && this->confirmed_extended_louver_matches_horizontal_(horizontal_vane)) {
    ESP_LOGD(DIAG_TAG, "command: confirmed horizontal vane request ignored value=%u",
             static_cast<unsigned int>(horizontal_vane));
    return false;
  }

  command.horizontal_vane_set = true;
  command.horizontal_vane = horizontal_vane;
  return true;
}

bool MhiAcCtrl::request_three_d_auto_command(bool enabled) {
  auto& command = this->state_.command();
  const uint32_t queued_mask = command.pending_command_mask();
  const uint32_t pending_mask = this->command_confirmation_.pending_mask();
  const auto& pending_intent = this->command_confirmation_.pending_intent();

  if (command.three_d_auto_set && command.three_d_auto == enabled) {
    ESP_LOGD(DIAG_TAG, "command: duplicate queued 3D auto request ignored state=%s", enabled ? "ON" : "OFF");
    return false;
  }

  if ((pending_mask & MHI_COMMAND_THREE_D_AUTO) != 0U && pending_intent.three_d_auto == enabled) {
    ESP_LOGD(DIAG_TAG, "command: duplicate pending 3D auto request ignored state=%s", enabled ? "ON" : "OFF");
    return false;
  }

  const bool extended_louver_busy = ((queued_mask | pending_mask) & kNoPendingExtendedFeedbackMask) != 0U;
  if (!extended_louver_busy && this->confirmed_extended_louver_matches_three_d_auto_(enabled)) {
    ESP_LOGD(DIAG_TAG, "command: confirmed 3D auto request ignored state=%s", enabled ? "ON" : "OFF");
    return false;
  }

  command.three_d_auto_set = true;
  command.three_d_auto = enabled;
  return true;
}

void MhiAcCtrl::refresh_publish_targets_() {
  this->publish_bridge_.set_targets(this->publish_targets_);
  this->publish_requested_ = true;
}

void MhiAcCtrl::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI AC Ctrl rewrite skeleton");

  this->diagnostics_.stats().reset();
  this->command_confirmation_.reset();
  this->frame_sync_.set_stats(&this->diagnostics_.stats());
  this->last_diag_log_ms_ = 0U;
  this->rx_worker_started_ms_ = 0U;
  this->last_rx_worker_health_check_ms_ = 0U;
  this->last_rx_worker_health_frames_ = 0U;
  this->last_rx_worker_health_drained_frames_ = 0U;
  this->last_rx_worker_health_overflows_ = 0U;
  this->rx_worker_startup_warning_logged_ = false;
  this->rx_worker_stall_warning_logged_ = false;
  this->rx_worker_not_draining_warning_logged_ = false;
  this->pending_extended_feedback_candidate_ = false;
  this->pending_extended_feedback_swing_ = false;
  this->pending_extended_feedback_vane_ = 0U;
  this->pending_extended_feedback_3d_auto_ = false;
  this->pending_extended_feedback_db16_ = 0U;
  this->pending_extended_feedback_db17_ = 0U;
  this->pending_extended_feedback_repeat_count_ = 0U;
  this->extended_louver_bootstrap_complete_ = false;
  this->last_protocol_health_valid_frames_ = 0U;
  this->last_protocol_health_invalid_frames_ = 0U;
  this->last_protocol_health_checksum_failures_ = 0U;
  this->last_protocol_health_signature_misses_ = 0U;
  this->last_protocol_health_sync_losses_ = 0U;
  this->last_protocol_health_dropped_bytes_ = 0U;

  this->chip_core_count_ = detect_chip_core_count_();
  this->rx_worker_enabled_ = mhi_rx_worker_mode_resolves_enabled(this->rx_worker_mode_, this->chip_core_count_);

  this->tx_config_.frame_size = this->frame_size_ == 33 ? kMhiFrame33Bytes : kMhiFrame20Bytes;

  this->tx_config_.enabled_opdata_mask = this->opdata_mask_;

  this->frame_sync_.reset();
  this->frame_sync_.set_mode(MhiFrameSyncMode::MOSI_ONLY);
  this->frame_sync_.set_33_byte_frames_enabled(this->frame_size_ == 33);

  this->transport_.set_diagnostics(&this->diagnostics_);

  this->transport_.configure(this->pins_.sck, this->pins_.mosi, this->pins_.miso, this->rx_driver_, this->tx_driver_,
                             static_cast<uint8_t>(this->frame_size_));

  this->transport_.setup();

  this->using_rx_worker_ = false;

  if (this->rx_worker_enabled_) {
    this->rx_worker_.configure(&this->transport_, &this->diagnostics_.stats(), static_cast<uint8_t>(this->frame_size_));
    this->using_rx_worker_ = this->rx_worker_.start();

    if (this->using_rx_worker_) {
      this->rx_worker_started_ms_ = millis();
      ESP_LOGCONFIG(TAG, "RX worker active: queue-backed FastGPIO sampling");
    } else {
      ESP_LOGW(TAG, "RX worker failed to start; falling back to synchronous RX sampling");
    }
  } else {
    ESP_LOGCONFIG(TAG, "RX worker disabled; using synchronous FastGPIO sampling");
  }
}

void MhiAcCtrl::loop() {
  const uint32_t loop_start_us = micros();
  bool state_changed = false;
  uint32_t drained_rx_frames = 0U;

  uint32_t section_start_us = loop_start_us;
  this->transport_.loop();
  const uint32_t transport_loop_us = elapsed_us_(section_start_us);

  uint32_t tx_stage_us = 0U;
  uint32_t rx_read_sync_us = 0U;

  if (this->using_rx_worker_) {
    section_start_us = micros();
    drained_rx_frames = this->drain_rx_worker_frames_(state_changed);
    rx_read_sync_us = elapsed_us_(section_start_us);

    if (drained_rx_frames > 0U) {
      section_start_us = micros();
      this->build_and_stage_tx_frame_();
      tx_stage_us = elapsed_us_(section_start_us);
    }
  } else {
    section_start_us = micros();
    this->build_and_stage_tx_frame_();
    tx_stage_us = elapsed_us_(section_start_us);

    section_start_us = micros();
    state_changed = this->read_and_sync_rx_frame_();
    rx_read_sync_us = elapsed_us_(section_start_us);
  }

  section_start_us = micros();
  if (state_changed || this->publish_requested_) {
    this->publish_bridge_.publish(this->state_);
    this->publish_requested_ = false;
  }
  const uint32_t publish_us = elapsed_us_(section_start_us);

  section_start_us = micros();
  this->check_command_confirmation_timeout_();
  const uint32_t command_housekeeping_us = elapsed_us_(section_start_us);

  const uint32_t loop_us = elapsed_us_(loop_start_us);
  this->diagnostics_.stats().on_loop_timing(loop_us, transport_loop_us, tx_stage_us, rx_read_sync_us, publish_us,
                                            command_housekeeping_us, kLoopBudgetUs, millis());

  this->check_rx_worker_health_();
  this->log_runtime_diagnostics_();
}

void MhiAcCtrl::dump_config() {
  const auto diag = this->diagnostics_.snapshot(millis());

  ESP_LOGCONFIG(TAG, "MHI AC Ctrl rewrite skeleton:");
  ESP_LOGCONFIG(TAG, "  Frame size: %d", this->frame_size_);
  ESP_LOGCONFIG(TAG, "  Room temp timeout: %ds", this->room_temp_api_timeout_s_);
  ESP_LOGCONFIG(TAG, "  Pins: SCK=%d MOSI=%d MISO=%d", this->pins_.sck, this->pins_.mosi, this->pins_.miso);
  ESP_LOGCONFIG(TAG, "  RX driver configured: %s", this->rx_driver_.c_str());
  ESP_LOGCONFIG(TAG, "  TX driver configured: %s", this->tx_driver_.c_str());
  ESP_LOGCONFIG(TAG, "  RX driver active: %s", diag.rx_driver_name);
  ESP_LOGCONFIG(TAG, "  TX driver active: %s", diag.tx_driver_name);
  ESP_LOGCONFIG(TAG, "  RX ready: %s", diag.rx_driver_ready ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  TX ready: %s", diag.tx_driver_ready ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  RX worker mode: %s", mhi_rx_worker_mode_to_string(this->rx_worker_mode_));
  ESP_LOGCONFIG(TAG, "  Chip cores: %u", static_cast<unsigned int>(this->chip_core_count_));
  ESP_LOGCONFIG(TAG, "  RX worker configured: %s", this->rx_worker_enabled_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  RX worker resolved: %s", this->rx_worker_enabled_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  RX worker active: %s", this->using_rx_worker_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  RX worker startup grace: %lums", static_cast<unsigned long>(kRxWorkerStartupGraceMs));
  ESP_LOGCONFIG(TAG, "  RX worker stall warning: %lums", static_cast<unsigned long>(kRxWorkerStallWarnMs));
  ESP_LOGCONFIG(TAG, "  Opdata request mask: 0x%08lx", static_cast<unsigned long>(this->opdata_mask_));
}

void MhiAcCtrl::check_rx_worker_health_() {
  if (!this->using_rx_worker_) {
    return;
  }

  const uint32_t now = millis();

  if (this->last_rx_worker_health_check_ms_ != 0U &&
      (now - this->last_rx_worker_health_check_ms_) < kRxWorkerHealthCheckIntervalMs) {
    return;
  }

  this->last_rx_worker_health_check_ms_ = now;

  const auto stats = this->diagnostics_.stats().snapshot();
  const bool startup_grace_expired =
      this->rx_worker_started_ms_ != 0U && (now - this->rx_worker_started_ms_) >= kRxWorkerStartupGraceMs;

  const uint32_t frame_age_ms = stats.last_rx_worker_frame_ms == 0U || now < stats.last_rx_worker_frame_ms
                                    ? 0U
                                    : now - stats.last_rx_worker_frame_ms;
  const uint32_t drain_age_ms =
      stats.last_rx_worker_drained_frame_ms == 0U || now < stats.last_rx_worker_drained_frame_ms
          ? 0U
          : now - stats.last_rx_worker_drained_frame_ms;

  const bool no_frames = startup_grace_expired && stats.rx_worker_frames == 0U;
  const bool stalled = startup_grace_expired && stats.rx_worker_frames > 0U &&
                       stats.rx_worker_frames == this->last_rx_worker_health_frames_ &&
                       frame_age_ms >= kRxWorkerStallWarnMs;
  const bool not_draining = startup_grace_expired && stats.rx_worker_queue_depth > 0U &&
                            stats.rx_worker_drained_frames == this->last_rx_worker_health_drained_frames_ &&
                            drain_age_ms >= kRxWorkerNotDrainingWarnMs;
  const bool overflow_increased = stats.rx_worker_queue_overflows > this->last_rx_worker_health_overflows_;

  this->diagnostics_.stats().on_rx_worker_health_check(no_frames, stalled, not_draining, now);

  if (no_frames && !this->rx_worker_startup_warning_logged_) {
    this->rx_worker_startup_warning_logged_ = true;
    ESP_LOGW(DIAG_TAG, "rx_worker: enabled but no frames after startup grace (%lums)",
             static_cast<unsigned long>(kRxWorkerStartupGraceMs));
  }

  if (stalled && !this->rx_worker_stall_warning_logged_) {
    this->rx_worker_stall_warning_logged_ = true;
    ESP_LOGW(DIAG_TAG, "rx_worker: stalled, no new frames for %lums frames=%lu drained=%lu",
             static_cast<unsigned long>(frame_age_ms), static_cast<unsigned long>(stats.rx_worker_frames),
             static_cast<unsigned long>(stats.rx_worker_drained_frames));
  }

  if (not_draining && !this->rx_worker_not_draining_warning_logged_) {
    this->rx_worker_not_draining_warning_logged_ = true;
    ESP_LOGW(DIAG_TAG, "rx_worker: queue not draining for %lums depth=%lu frames=%lu drained=%lu",
             static_cast<unsigned long>(drain_age_ms), static_cast<unsigned long>(stats.rx_worker_queue_depth),
             static_cast<unsigned long>(stats.rx_worker_frames),
             static_cast<unsigned long>(stats.rx_worker_drained_frames));
  }

  if (overflow_increased) {
    ESP_LOGW(DIAG_TAG, "rx_worker: queue overflow count increased total=%lu depth=%lu max_depth=%lu",
             static_cast<unsigned long>(stats.rx_worker_queue_overflows),
             static_cast<unsigned long>(stats.rx_worker_queue_depth),
             static_cast<unsigned long>(stats.rx_worker_queue_max_depth));
  }

  if (stats.rx_worker_frames > this->last_rx_worker_health_frames_) {
    this->rx_worker_stall_warning_logged_ = false;
  }

  if (stats.rx_worker_drained_frames > this->last_rx_worker_health_drained_frames_ ||
      stats.rx_worker_queue_depth == 0U) {
    this->rx_worker_not_draining_warning_logged_ = false;
  }

  this->last_rx_worker_health_frames_ = stats.rx_worker_frames;
  this->last_rx_worker_health_drained_frames_ = stats.rx_worker_drained_frames;
  this->last_rx_worker_health_overflows_ = stats.rx_worker_queue_overflows;
}

void MhiAcCtrl::log_runtime_diagnostics_() {
  const uint32_t now = millis();

  if (now < kDiagLogIntervalMs) {
    return;
  }

  if (this->last_diag_log_ms_ != 0U && (now - this->last_diag_log_ms_) < kDiagLogIntervalMs) {
    return;
  }

  this->last_diag_log_ms_ = now;

  const auto diag = this->diagnostics_.snapshot(now);
  const auto& stats = diag.stats;

  ESP_LOGI(DIAG_TAG,
           "runtime: rx_bytes=%lu rx_chunks=%lu candidate_frames=%lu valid_frames=%lu invalid_frames=%lu "
           "checksum_failures=%lu",
           static_cast<unsigned long>(stats.rx_bytes), static_cast<unsigned long>(stats.rx_chunks),
           static_cast<unsigned long>(stats.candidate_frames), static_cast<unsigned long>(stats.valid_frames),
           static_cast<unsigned long>(stats.invalid_frames), static_cast<unsigned long>(stats.checksum_failures));

  ESP_LOGI(DIAG_TAG,
           "runtime: signature_misses=%lu sync_losses=%lu dropped_bytes=%lu tx_frames=%lu tx_failures=%lu "
           "last_valid_frame_age_ms=%lu last_rx_byte_age_ms=%lu last_tx_frame_age_ms=%lu",
           static_cast<unsigned long>(stats.signature_misses), static_cast<unsigned long>(stats.sync_losses),
           static_cast<unsigned long>(stats.dropped_bytes), static_cast<unsigned long>(stats.tx_frames),
           static_cast<unsigned long>(stats.tx_failures), static_cast<unsigned long>(diag.last_valid_frame_age_ms),
           static_cast<unsigned long>(diag.last_rx_byte_age_ms), static_cast<unsigned long>(diag.last_tx_frame_age_ms));

  const uint32_t delta_valid_frames = stats.valid_frames - this->last_protocol_health_valid_frames_;
  const uint32_t delta_invalid_frames = stats.invalid_frames - this->last_protocol_health_invalid_frames_;
  const uint32_t delta_checksum_failures = stats.checksum_failures - this->last_protocol_health_checksum_failures_;
  const uint32_t delta_signature_misses = stats.signature_misses - this->last_protocol_health_signature_misses_;
  const uint32_t delta_sync_losses = stats.sync_losses - this->last_protocol_health_sync_losses_;
  const uint32_t delta_dropped_bytes = stats.dropped_bytes - this->last_protocol_health_dropped_bytes_;
  const bool protocol_healthy = delta_invalid_frames == 0U && delta_checksum_failures == 0U &&
                                delta_signature_misses == 0U && delta_sync_losses == 0U && delta_dropped_bytes == 0U;

  if (protocol_healthy) {
    ESP_LOGI(DIAG_TAG,
             "runtime: rx_protocol_health healthy=YES delta_valid=%lu delta_invalid=%lu delta_checksum_failures=%lu "
             "delta_signature_misses=%lu delta_sync_losses=%lu delta_dropped_bytes=%lu",
             static_cast<unsigned long>(delta_valid_frames), static_cast<unsigned long>(delta_invalid_frames),
             static_cast<unsigned long>(delta_checksum_failures), static_cast<unsigned long>(delta_signature_misses),
             static_cast<unsigned long>(delta_sync_losses), static_cast<unsigned long>(delta_dropped_bytes));
  } else {
    ESP_LOGW(DIAG_TAG,
             "runtime: rx_protocol_health healthy=NO delta_valid=%lu delta_invalid=%lu delta_checksum_failures=%lu "
             "delta_signature_misses=%lu delta_sync_losses=%lu delta_dropped_bytes=%lu",
             static_cast<unsigned long>(delta_valid_frames), static_cast<unsigned long>(delta_invalid_frames),
             static_cast<unsigned long>(delta_checksum_failures), static_cast<unsigned long>(delta_signature_misses),
             static_cast<unsigned long>(delta_sync_losses), static_cast<unsigned long>(delta_dropped_bytes));
  }

  this->last_protocol_health_valid_frames_ = stats.valid_frames;
  this->last_protocol_health_invalid_frames_ = stats.invalid_frames;
  this->last_protocol_health_checksum_failures_ = stats.checksum_failures;
  this->last_protocol_health_signature_misses_ = stats.signature_misses;
  this->last_protocol_health_sync_losses_ = stats.sync_losses;
  this->last_protocol_health_dropped_bytes_ = stats.dropped_bytes;

  ESP_LOGI(DIAG_TAG,
           "runtime: tx_command_frames=%lu tx_command_failures=%lu unsupported_commands=%lu "
           "last_tx_command_mask=0x%08lx last_unsupported_command_mask=0x%08lx "
           "last_tx_command_age_ms=%lu last_unsupported_command_age_ms=%lu",
           static_cast<unsigned long>(stats.tx_command_frames), static_cast<unsigned long>(stats.tx_command_failures),
           static_cast<unsigned long>(stats.unsupported_commands),
           static_cast<unsigned long>(stats.last_tx_command_mask),
           static_cast<unsigned long>(stats.last_unsupported_command_mask),
           static_cast<unsigned long>(diag.last_tx_command_frame_age_ms),
           static_cast<unsigned long>(diag.last_unsupported_command_age_ms));

  ESP_LOGI(DIAG_TAG,
           "runtime: command_confirmations=%lu command_confirmation_timeouts=%lu pending_confirmation_mask=0x%08lx "
           "last_confirmed_mask=0x%08lx last_timeout_mask=0x%08lx "
           "last_confirmed_age_ms=%lu last_timeout_age_ms=%lu",
           static_cast<unsigned long>(stats.command_confirmations),
           static_cast<unsigned long>(stats.command_confirmation_timeouts),
           static_cast<unsigned long>(this->command_confirmation_.pending_mask()),
           static_cast<unsigned long>(stats.last_confirmed_command_mask),
           static_cast<unsigned long>(stats.last_command_confirmation_timeout_mask),
           static_cast<unsigned long>(diag.last_command_confirmation_age_ms),
           static_cast<unsigned long>(diag.last_command_confirmation_timeout_age_ms));

  ESP_LOGI(DIAG_TAG, "runtime: loop_us last=%lu avg=%lu max=%lu over_budget=%lu budget=%lu last_over_budget_age_ms=%lu",
           static_cast<unsigned long>(stats.loop_last_us), static_cast<unsigned long>(stats.loop_avg_us),
           static_cast<unsigned long>(stats.loop_max_us), static_cast<unsigned long>(stats.loop_over_budget),
           static_cast<unsigned long>(stats.loop_budget_us),
           static_cast<unsigned long>(diag.last_loop_over_budget_age_ms));

  ESP_LOGI(
      DIAG_TAG,
      "runtime: section_us transport=%lu/%lu/%lu tx=%lu/%lu/%lu rx=%lu/%lu/%lu "
      "publish=%lu/%lu/%lu command=%lu/%lu/%lu",
      static_cast<unsigned long>(stats.transport_loop_last_us), static_cast<unsigned long>(stats.transport_loop_avg_us),
      static_cast<unsigned long>(stats.transport_loop_max_us), static_cast<unsigned long>(stats.tx_stage_last_us),
      static_cast<unsigned long>(stats.tx_stage_avg_us), static_cast<unsigned long>(stats.tx_stage_max_us),
      static_cast<unsigned long>(stats.rx_read_sync_last_us), static_cast<unsigned long>(stats.rx_read_sync_avg_us),
      static_cast<unsigned long>(stats.rx_read_sync_max_us), static_cast<unsigned long>(stats.publish_last_us),
      static_cast<unsigned long>(stats.publish_avg_us), static_cast<unsigned long>(stats.publish_max_us),
      static_cast<unsigned long>(stats.command_housekeeping_last_us),
      static_cast<unsigned long>(stats.command_housekeeping_avg_us),
      static_cast<unsigned long>(stats.command_housekeeping_max_us));

  ESP_LOGI(DIAG_TAG,
           "runtime: rx_worker samples=%lu frames=%lu drained=%lu queue_depth=%lu queue_max=%lu queue_overflows=%lu "
           "last_frame_age_ms=%lu last_overflow_age_ms=%lu",
           static_cast<unsigned long>(stats.rx_worker_samples), static_cast<unsigned long>(stats.rx_worker_frames),
           static_cast<unsigned long>(stats.rx_worker_drained_frames),
           static_cast<unsigned long>(stats.rx_worker_queue_depth),
           static_cast<unsigned long>(stats.rx_worker_queue_max_depth),
           static_cast<unsigned long>(stats.rx_worker_queue_overflows),
           static_cast<unsigned long>(diag.last_rx_worker_frame_age_ms),
           static_cast<unsigned long>(diag.last_rx_worker_queue_overflow_age_ms));

  ESP_LOGI(DIAG_TAG, "runtime: rx_worker_us last=%lu avg=%lu max=%lu",
           static_cast<unsigned long>(stats.rx_worker_last_us), static_cast<unsigned long>(stats.rx_worker_avg_us),
           static_cast<unsigned long>(stats.rx_worker_max_us));

  const bool worker_startup_grace_expired = this->using_rx_worker_ && this->rx_worker_started_ms_ != 0U &&
                                            (now - this->rx_worker_started_ms_) >= kRxWorkerStartupGraceMs;
  const bool worker_healthy = this->using_rx_worker_ && stats.rx_worker_frames > 0U &&
                              stats.rx_worker_queue_overflows == 0U &&
                              diag.last_rx_worker_frame_age_ms < kRxWorkerFrameAgeHealthyMs;

  ESP_LOGI(DIAG_TAG,
           "runtime: rx_worker_task_health active=%s healthy=%s startup_grace_expired=%s "
           "health_checks=%lu no_frame_windows=%lu stalls=%lu not_draining=%lu "
           "last_drain_age_ms=%lu last_stall_age_ms=%lu last_not_draining_age_ms=%lu",
           this->using_rx_worker_ ? "YES" : "NO", worker_healthy ? "YES" : "NO",
           worker_startup_grace_expired ? "YES" : "NO", static_cast<unsigned long>(stats.rx_worker_health_checks),
           static_cast<unsigned long>(stats.rx_worker_no_frame_windows),
           static_cast<unsigned long>(stats.rx_worker_stalls),
           static_cast<unsigned long>(stats.rx_worker_not_draining_windows),
           static_cast<unsigned long>(diag.last_rx_worker_drained_frame_age_ms),
           static_cast<unsigned long>(diag.last_rx_worker_stall_age_ms),
           static_cast<unsigned long>(diag.last_rx_worker_not_draining_age_ms));
}

uint32_t MhiAcCtrl::elapsed_us_(uint32_t start_us) {
  return static_cast<uint32_t>(micros() - start_us);
}

uint8_t MhiAcCtrl::detect_chip_core_count_() {
#if defined(USE_ESP32)
  esp_chip_info_t chip_info{};
  esp_chip_info(&chip_info);
  return static_cast<uint8_t>(chip_info.cores);
#else
  return 1U;
#endif
}

bool MhiAcCtrl::confirmed_extended_louver_matches_horizontal_(uint8_t horizontal_vane) const {
  const auto& status = this->state_.status();

  if (!this->extended_louver_bootstrap_complete_ || !status.has_extended_louver_raw || !status.has_horizontal_vane) {
    return false;
  }

  if (horizontal_vane == 8U) {
    return status.horizontal_vane_swing;
  }

  if (horizontal_vane >= 1U && horizontal_vane <= 7U) {
    return !status.horizontal_vane_swing && status.horizontal_vane == horizontal_vane;
  }

  return false;
}

bool MhiAcCtrl::confirmed_extended_louver_matches_three_d_auto_(bool enabled) const {
  const auto& status = this->state_.status();

  return this->extended_louver_bootstrap_complete_ && status.has_extended_louver_raw && status.has_3d_auto &&
         status.three_d_auto == enabled;
}

void MhiAcCtrl::refresh_extended_louver_tx_context_() {
  const auto& status = this->state_.status();

  this->tx_config_.has_extended_louver_state =
      status.has_horizontal_vane && status.has_3d_auto && status.has_extended_louver_raw;

  if (!this->tx_config_.has_extended_louver_state) {
    return;
  }

  this->tx_config_.extended_louver_db16 = status.extended_louver_db16;
  this->tx_config_.extended_louver_db17 = status.extended_louver_db17;
  this->tx_config_.extended_louver_horizontal_swing = status.horizontal_vane_swing;
  this->tx_config_.extended_louver_horizontal_vane = status.horizontal_vane;
  this->tx_config_.extended_louver_three_d_auto = status.three_d_auto;
}

void MhiAcCtrl::build_and_stage_tx_frame_() {
  this->suppress_duplicate_pending_commands_();
  this->refresh_extended_louver_tx_context_();

  MhiFrameBuffer tx_frame{};
  MhiTxBuildResult build_result{};

  const bool built = MhiTxBuilder::build_next_frame(this->state_.command(), this->tx_runtime_, this->tx_config_,
                                                    tx_frame, build_result);

  if (!built || tx_frame.len == 0U) {
    return;
  }

  const bool sent = this->transport_.send_tx(tx_frame.bytes(), tx_frame.len);
  this->record_tx_build_result_(build_result, tx_frame, sent);
}

void MhiAcCtrl::record_tx_build_result_(const MhiTxBuildResult& result, const MhiFrameBuffer& frame, bool sent) {
  const uint32_t now = millis();

  if (result.has_encoded_commands()) {
    if (sent) {
      this->diagnostics_.stats().on_tx_command_frame(result.encoded_command_mask, now);
      this->command_confirmation_.stage(result.intent, result.encoded_command_mask, now);
    } else {
      this->diagnostics_.stats().on_tx_command_failure(result.encoded_command_mask, now);
    }

    ESP_LOGI(DIAG_TAG,
             "command: staged=%s mask=0x%08lx len=%u db0=0x%02x db1=0x%02x db2=0x%02x db6=0x%02x "
             "db9=0x%02x db16=0x%02x db17=0x%02x",
             sent ? "YES" : "NO", static_cast<unsigned long>(result.encoded_command_mask),
             static_cast<unsigned int>(frame.len), frame.data[DB0], frame.data[DB1], frame.data[DB2], frame.data[DB6],
             frame.data[DB9], frame.len > DB16 ? frame.data[DB16] : 0U, frame.len > DB17 ? frame.data[DB17] : 0U);
  }

  if (result.has_unsupported_commands()) {
    this->diagnostics_.stats().on_unsupported_command(result.unsupported_command_mask, now);

    ESP_LOGW(DIAG_TAG, "command: unsupported mask=0x%08lx frame_size=%u; command dropped",
             static_cast<unsigned long>(result.unsupported_command_mask), static_cast<unsigned int>(frame.len));
  }
}

uint32_t MhiAcCtrl::drain_rx_worker_frames_(bool& state_changed) {
  uint32_t drained = 0U;
  MhiFrameBuffer raw_frame{};

  while (drained < kMaxWorkerFramesPerLoop && this->rx_worker_.pop_frame(raw_frame)) {
    drained++;

    if (raw_frame.len == 0U) {
      continue;
    }

    this->frame_sync_.push_bytes(raw_frame.data, raw_frame.len);

    MhiFrameBuffer frame{};

    while (this->frame_sync_.pop_frame(frame)) {
      this->diagnostics_.stats().on_valid_frame(millis());

      if (this->decode_frame_(frame)) {
        state_changed = true;
      }
    }
  }

  return drained;
}

bool MhiAcCtrl::read_and_sync_rx_frame_() {
  bool state_changed = false;

  uint8_t buffer[kMhiMaxFrameBytes]{};

  const std::size_t len = this->transport_.read_rx(buffer, sizeof(buffer));

  if (len > 0U) {
    this->frame_sync_.push_bytes(buffer, len);
  }

  MhiFrameBuffer frame{};

  while (this->frame_sync_.pop_frame(frame)) {
    this->diagnostics_.stats().on_valid_frame(millis());

    if (this->decode_frame_(frame)) {
      state_changed = true;
    }
  }

  return state_changed;
}

bool MhiAcCtrl::decode_frame_(const MhiFrameBuffer& frame) {
  bool decoded_anything = false;

  const MhiFrameView view = frame.view();

  MhiDecodedStatus decoded_status{};

  if (MhiStatusDecoder::decode_mosi(view, decoded_status)) {
    if (this->apply_status_update_(decoded_status, frame)) {
      decoded_anything = true;
    }
  }

  MhiDecodedOpData decoded_opdata{};

  if (MhiOpDataDecoder::decode_mosi(view, decoded_opdata)) {
    if (this->apply_opdata_update_(decoded_opdata, frame)) {
      decoded_anything = true;
    }
  }

  return decoded_anything;
}

bool MhiAcCtrl::apply_status_update_(const MhiDecodedStatus& decoded_status, const MhiFrameBuffer& frame) {
  if (!this->is_sane_status_(decoded_status, frame)) {
    return false;
  }

  auto& status = this->state_.status();
  const bool previous_valid = status.valid;
  const bool previous_horizontal_swing = status.horizontal_vane_swing;
  const uint8_t previous_horizontal_vane = status.horizontal_vane;
  const bool previous_3d_auto = status.three_d_auto;

  status.valid = decoded_status.valid;
  status.power = decoded_status.power;
  status.mode = decoded_status.mode;
  status.fan = decoded_status.fan;
  status.target_temp_c = decoded_status.target_temp_c;
  status.room_temp_c = decoded_status.room_temp_c;
  status.vertical_vane = decoded_status.vertical_vane;
  status.vanes_swing = decoded_status.vertical_swing;

  const bool accept_extended_feedback = (decoded_status.has_horizontal_vane || decoded_status.has_3d_auto) &&
                                        this->accept_extended_feedback_(decoded_status, frame);

  if (decoded_status.has_horizontal_vane && accept_extended_feedback) {
    status.has_horizontal_vane = true;
    status.horizontal_vane_swing = decoded_status.horizontal_swing;
    status.horizontal_vane = decoded_status.horizontal_vane;

    if (previous_valid && previous_horizontal_swing != status.horizontal_vane_swing) {
      this->log_suspicious_status_change_("horizontal_swing", previous_horizontal_swing ? 1 : 0,
                                          status.horizontal_vane_swing ? 1 : 0, frame);
    }

    if (previous_valid && previous_horizontal_vane != status.horizontal_vane) {
      this->log_suspicious_status_change_("horizontal_vane", previous_horizontal_vane, status.horizontal_vane, frame);
    }
  }

  if (decoded_status.has_3d_auto && accept_extended_feedback) {
    status.has_3d_auto = true;
    status.three_d_auto = decoded_status.three_d_auto;

    if (previous_valid && previous_3d_auto != status.three_d_auto) {
      this->log_suspicious_status_change_("three_d_auto", previous_3d_auto ? 1 : 0, status.three_d_auto ? 1 : 0, frame);
    }
  }

  if (accept_extended_feedback && decoded_status.has_extended_louver_raw) {
    status.has_extended_louver_raw = true;
    status.extended_louver_db16 = decoded_status.extended_louver_db16;
    status.extended_louver_db17 = decoded_status.extended_louver_db17;
  }

  status.error_code = decoded_status.error_code;
  status.last_update_ms = millis();

  this->diagnostics_.stats().set_last_error_code(decoded_status.error_code);
  this->update_command_confirmation_(status);

  return true;
}

bool MhiAcCtrl::apply_opdata_update_(const MhiDecodedOpData& decoded_opdata, const MhiFrameBuffer& frame) {
  auto& opdata = this->state_.opdata();
  bool accepted = false;

  opdata.valid = true;
  opdata.last_update_ms = millis();

  if (decoded_opdata.has_outdoor_temp) {
    if (in_range(decoded_opdata.outdoor_temp_c, -60.0f, 80.0f)) {
      opdata.has_outdoor_temp = true;
      opdata.outdoor_temp_c = decoded_opdata.outdoor_temp_c;
      accepted = true;
    } else {
      this->log_rejected_opdata_("outdoor_temp", decoded_opdata.outdoor_temp_c, frame);
    }
  }

  if (decoded_opdata.has_return_air) {
    if (in_range(decoded_opdata.return_air_c, -10.0f, 60.0f)) {
      opdata.has_return_air = true;
      opdata.return_air_c = decoded_opdata.return_air_c;
      accepted = true;
    } else {
      this->log_rejected_opdata_("return_air", decoded_opdata.return_air_c, frame);
    }
  }

  if (decoded_opdata.has_compressor_frequency) {
    if (in_range(decoded_opdata.compressor_frequency_hz, 0.0f, 250.0f)) {
      opdata.has_compressor_frequency = true;
      opdata.compressor_frequency_hz = decoded_opdata.compressor_frequency_hz;
      accepted = true;
    } else {
      this->log_rejected_opdata_("compressor_frequency", decoded_opdata.compressor_frequency_hz, frame);
    }
  }

  if (decoded_opdata.has_current) {
    if (in_range(decoded_opdata.current_a, 0.0f, 80.0f)) {
      opdata.has_current = true;
      opdata.current_a = decoded_opdata.current_a;
      accepted = true;
    } else {
      this->log_rejected_opdata_("current", decoded_opdata.current_a, frame);
    }
  }

  if (decoded_opdata.has_indoor_unit_fan_speed) {
    if (decoded_opdata.indoor_unit_fan_speed <= 15U) {
      opdata.has_indoor_unit_fan_speed = true;
      opdata.indoor_unit_fan_speed = decoded_opdata.indoor_unit_fan_speed;
      accepted = true;
    } else {
      this->log_rejected_opdata_("indoor_unit_fan_speed", decoded_opdata.indoor_unit_fan_speed, frame);
    }
  }

  if (decoded_opdata.has_outdoor_unit_fan_speed) {
    if (decoded_opdata.outdoor_unit_fan_speed <= 15U) {
      opdata.has_outdoor_unit_fan_speed = true;
      opdata.outdoor_unit_fan_speed = decoded_opdata.outdoor_unit_fan_speed;
      accepted = true;
    } else {
      this->log_rejected_opdata_("outdoor_unit_fan_speed", decoded_opdata.outdoor_unit_fan_speed, frame);
    }
  }

  if (decoded_opdata.has_total_indoor_runtime) {
    opdata.has_indoor_unit_total_run_time = true;
    opdata.indoor_unit_total_run_time_hours = decoded_opdata.total_indoor_runtime_hours;
    accepted = true;
  }

  if (decoded_opdata.has_total_compressor_runtime) {
    opdata.has_compressor_total_run_time = true;
    opdata.compressor_total_run_time_hours = decoded_opdata.total_compressor_runtime_hours;
    accepted = true;
  }

  if (decoded_opdata.has_energy_used) {
    if (in_range(decoded_opdata.energy_used_kwh, 0.0f, 1000000.0f)) {
      opdata.has_energy_used = true;
      opdata.energy_used_kwh = decoded_opdata.energy_used_kwh;
      accepted = true;
    } else {
      this->log_rejected_opdata_("energy_used", decoded_opdata.energy_used_kwh, frame);
    }
  }

  if (decoded_opdata.has_indoor_unit_thi_r1) {
    if (in_range(decoded_opdata.indoor_unit_thi_r1_c, -50.0f, 130.0f)) {
      opdata.has_indoor_unit_thi_r1 = true;
      opdata.indoor_unit_thi_r1_c = decoded_opdata.indoor_unit_thi_r1_c;
      accepted = true;
    } else {
      this->log_rejected_opdata_("indoor_unit_thi_r1", decoded_opdata.indoor_unit_thi_r1_c, frame);
    }
  }

  if (decoded_opdata.has_indoor_unit_thi_r2) {
    if (in_range(decoded_opdata.indoor_unit_thi_r2_c, -50.0f, 130.0f)) {
      opdata.has_indoor_unit_thi_r2 = true;
      opdata.indoor_unit_thi_r2_c = decoded_opdata.indoor_unit_thi_r2_c;
      accepted = true;
    } else {
      this->log_rejected_opdata_("indoor_unit_thi_r2", decoded_opdata.indoor_unit_thi_r2_c, frame);
    }
  }

  if (decoded_opdata.has_indoor_unit_thi_r3) {
    if (in_range(decoded_opdata.indoor_unit_thi_r3_c, -50.0f, 130.0f)) {
      opdata.has_indoor_unit_thi_r3 = true;
      opdata.indoor_unit_thi_r3_c = decoded_opdata.indoor_unit_thi_r3_c;
      accepted = true;
    } else {
      this->log_rejected_opdata_("indoor_unit_thi_r3", decoded_opdata.indoor_unit_thi_r3_c, frame);
    }
  }

  if (decoded_opdata.has_outdoor_unit_tho_r1) {
    if (in_range(decoded_opdata.outdoor_unit_tho_r1_c, -50.0f, 130.0f)) {
      opdata.has_outdoor_unit_tho_r1 = true;
      opdata.outdoor_unit_tho_r1_c = decoded_opdata.outdoor_unit_tho_r1_c;
      accepted = true;
    } else {
      this->log_rejected_opdata_("outdoor_unit_tho_r1", decoded_opdata.outdoor_unit_tho_r1_c, frame);
    }
  }

  if (decoded_opdata.has_outdoor_unit_expansion_valve) {
    opdata.has_outdoor_unit_expansion_valve = true;
    opdata.outdoor_unit_expansion_valve_pulses = decoded_opdata.outdoor_unit_expansion_valve_pulses;
    accepted = true;
  }

  if (decoded_opdata.has_outdoor_unit_discharge_pipe) {
    if (in_range(decoded_opdata.outdoor_unit_discharge_pipe_c, 0.0f, 140.0f)) {
      opdata.has_outdoor_unit_discharge_pipe = true;
      opdata.outdoor_unit_discharge_pipe_c = decoded_opdata.outdoor_unit_discharge_pipe_c;
      accepted = true;
    } else {
      this->log_rejected_opdata_("outdoor_unit_discharge_pipe", decoded_opdata.outdoor_unit_discharge_pipe_c, frame);
    }
  }

  if (decoded_opdata.has_outdoor_unit_discharge_pipe_super_heat) {
    if (in_range(decoded_opdata.outdoor_unit_discharge_pipe_super_heat_c, 0.0f, 120.0f)) {
      opdata.has_outdoor_unit_discharge_pipe_super_heat = true;
      opdata.outdoor_unit_discharge_pipe_super_heat_c = decoded_opdata.outdoor_unit_discharge_pipe_super_heat_c;
      accepted = true;
    } else {
      this->log_rejected_opdata_("outdoor_unit_discharge_pipe_super_heat",
                                 decoded_opdata.outdoor_unit_discharge_pipe_super_heat_c, frame);
    }
  }

  if (decoded_opdata.has_protection_state_number) {
    opdata.has_protection_state_number = true;
    opdata.protection_state_number = decoded_opdata.protection_state_number;
    accepted = true;
  }

  if (decoded_opdata.has_defrost) {
    opdata.has_defrost = true;
    opdata.defrost = decoded_opdata.defrost;
    accepted = true;
  }

  return accepted;
}

bool MhiAcCtrl::is_sane_status_(const MhiDecodedStatus& decoded_status, const MhiFrameBuffer& frame) const {
  if (!decoded_status.valid) {
    return false;
  }

  if (decoded_status.mode > 4U || decoded_status.fan > 4U || decoded_status.vertical_vane > 4U) {
    ESP_LOGW(DIAG_TAG,
             "decode_reject: field=status_enum mode=%u fan=%u vertical=%u len=%u db0=0x%02x "
             "db1=0x%02x db2=0x%02x db3=0x%02x db6=0x%02x db9=0x%02x",
             static_cast<unsigned int>(decoded_status.mode), static_cast<unsigned int>(decoded_status.fan),
             static_cast<unsigned int>(decoded_status.vertical_vane), static_cast<unsigned int>(frame.len),
             frame.data[DB0], frame.data[DB1], frame.data[DB2], frame.data[DB3], frame.data[DB6], frame.data[DB9]);
    return false;
  }

  if (!in_range(decoded_status.target_temp_c, 5.0f, 40.0f) || !in_range(decoded_status.room_temp_c, -10.0f, 60.0f)) {
    ESP_LOGW(DIAG_TAG,
             "decode_reject: field=status_temp target=%.2f room=%.2f len=%u db0=0x%02x db1=0x%02x "
             "db2=0x%02x db3=0x%02x db6=0x%02x db9=0x%02x",
             decoded_status.target_temp_c, decoded_status.room_temp_c, static_cast<unsigned int>(frame.len),
             frame.data[DB0], frame.data[DB1], frame.data[DB2], frame.data[DB3], frame.data[DB6], frame.data[DB9]);
    return false;
  }

  if (decoded_status.has_horizontal_vane && decoded_status.horizontal_vane > 7U) {
    ESP_LOGD(DIAG_TAG, "decode_reject: field=horizontal_vane value=%u len=%u db16=0x%02x db17=0x%02x db9=0x%02x",
             static_cast<unsigned int>(decoded_status.horizontal_vane), static_cast<unsigned int>(frame.len),
             frame.len > DB16 ? frame.data[DB16] : 0U, frame.len > DB17 ? frame.data[DB17] : 0U, frame.data[DB9]);
    return false;
  }

  return true;
}

bool MhiAcCtrl::accept_extended_feedback_(const MhiDecodedStatus& decoded_status, const MhiFrameBuffer& frame) {
  if (!decoded_status.has_horizontal_vane && !decoded_status.has_3d_auto) {
    this->pending_extended_feedback_candidate_ = false;
    this->pending_extended_feedback_repeat_count_ = 0U;
    return false;
  }

  if (!this->using_rx_worker_) {
    this->pending_extended_feedback_candidate_ = false;
    this->pending_extended_feedback_repeat_count_ = 0U;
    return true;
  }

  const uint32_t confirmation_pending = this->command_confirmation_.pending_mask() & kNoPendingExtendedFeedbackMask;
  const uint32_t queued_pending = this->state_.command().pending_command_mask() & kNoPendingExtendedFeedbackMask;
  const uint32_t pending_mask = confirmation_pending | queued_pending;

  const uint8_t db16 = frame.len > DB16 ? frame.data[DB16] : 0U;
  const uint8_t db17 = frame.len > DB17 ? frame.data[DB17] : 0U;
  const uint32_t now = millis();

  if (confirmation_pending != 0U) {
    const bool matches_pending = this->extended_feedback_matches_pending_(decoded_status);
    const bool same_candidate = this->pending_extended_feedback_candidate_ &&
                                this->pending_extended_feedback_swing_ == decoded_status.horizontal_swing &&
                                this->pending_extended_feedback_vane_ == decoded_status.horizontal_vane &&
                                this->pending_extended_feedback_3d_auto_ == decoded_status.three_d_auto &&
                                this->pending_extended_feedback_db16_ == db16 &&
                                this->pending_extended_feedback_db17_ == db17;

    if (same_candidate) {
      if (this->pending_extended_feedback_repeat_count_ < 255U) {
        this->pending_extended_feedback_repeat_count_++;
      }
    } else {
      this->pending_extended_feedback_candidate_ = true;
      this->pending_extended_feedback_swing_ = decoded_status.horizontal_swing;
      this->pending_extended_feedback_vane_ = decoded_status.horizontal_vane;
      this->pending_extended_feedback_3d_auto_ = decoded_status.three_d_auto;
      this->pending_extended_feedback_db16_ = db16;
      this->pending_extended_feedback_db17_ = db17;
      this->pending_extended_feedback_repeat_count_ = 1U;
    }

    const uint32_t pending_age_ms = this->command_confirmation_.pending_age_ms(now);
    const bool settle_delay_elapsed = pending_age_ms >= kMhiExtendedLouverSettleDelayMs;

    if (settle_delay_elapsed &&
        this->pending_extended_feedback_repeat_count_ >= kStableCommandExtendedFeedbackRepeats) {
      this->settled_extended_confirmation_mask_ |= confirmation_pending;

      ESP_LOGI(DIAG_TAG,
               "decode_filter: accepted settled worker extended feedback vane=%u swing=%s 3d=%s len=%u "
               "db9=0x%02x db16=0x%02x db17=0x%02x pending=0x%08lx repeats=%u age_ms=%lu "
               "matches_pending=%s",
               static_cast<unsigned int>(decoded_status.horizontal_vane),
               decoded_status.horizontal_swing ? "YES" : "NO", decoded_status.three_d_auto ? "YES" : "NO",
               static_cast<unsigned int>(frame.len), frame.data[DB9], db16, db17,
               static_cast<unsigned long>(pending_mask),
               static_cast<unsigned int>(this->pending_extended_feedback_repeat_count_),
               static_cast<unsigned long>(pending_age_ms), matches_pending ? "YES" : "NO");

      this->pending_extended_feedback_candidate_ = false;
      this->pending_extended_feedback_repeat_count_ = 0U;
      return true;
    }

    ESP_LOGD(DIAG_TAG,
             "decode_filter: held settling worker extended feedback vane=%u swing=%s 3d=%s len=%u "
             "db9=0x%02x db16=0x%02x db17=0x%02x pending=0x%08lx repeats=%u age_ms=%lu "
             "settle_elapsed=%s matches_pending=%s",
             static_cast<unsigned int>(decoded_status.horizontal_vane), decoded_status.horizontal_swing ? "YES" : "NO",
             decoded_status.three_d_auto ? "YES" : "NO", static_cast<unsigned int>(frame.len), frame.data[DB9], db16,
             db17, static_cast<unsigned long>(pending_mask),
             static_cast<unsigned int>(this->pending_extended_feedback_repeat_count_),
             static_cast<unsigned long>(pending_age_ms), settle_delay_elapsed ? "YES" : "NO",
             matches_pending ? "YES" : "NO");
    return false;
  }

  if (queued_pending != 0U) {
    this->pending_extended_feedback_candidate_ = true;
    this->pending_extended_feedback_swing_ = decoded_status.horizontal_swing;
    this->pending_extended_feedback_vane_ = decoded_status.horizontal_vane;
    this->pending_extended_feedback_3d_auto_ = decoded_status.three_d_auto;
    this->pending_extended_feedback_db16_ = db16;
    this->pending_extended_feedback_db17_ = db17;
    this->pending_extended_feedback_repeat_count_ = 1U;
    return false;
  }

  const bool unchanged = this->state_.status().has_horizontal_vane && this->state_.status().has_3d_auto &&
                         this->state_.status().horizontal_vane_swing == decoded_status.horizontal_swing &&
                         this->state_.status().horizontal_vane == decoded_status.horizontal_vane &&
                         this->state_.status().three_d_auto == decoded_status.three_d_auto;

  if (unchanged && this->extended_louver_bootstrap_complete_) {
    this->pending_extended_feedback_candidate_ = false;
    this->pending_extended_feedback_repeat_count_ = 0U;
    return true;
  }

  const bool bootstrap_delay_elapsed = this->rx_worker_started_ms_ == 0U || now < this->rx_worker_started_ms_ ||
                                       (now - this->rx_worker_started_ms_) >= kExtendedLouverBootstrapDelayMs;

  if (!this->extended_louver_bootstrap_complete_) {
    const bool same_candidate = this->pending_extended_feedback_candidate_ &&
                                this->pending_extended_feedback_swing_ == decoded_status.horizontal_swing &&
                                this->pending_extended_feedback_vane_ == decoded_status.horizontal_vane &&
                                this->pending_extended_feedback_3d_auto_ == decoded_status.three_d_auto &&
                                this->pending_extended_feedback_db16_ == db16 &&
                                this->pending_extended_feedback_db17_ == db17;

    if (same_candidate) {
      if (bootstrap_delay_elapsed && this->pending_extended_feedback_repeat_count_ < 255U) {
        this->pending_extended_feedback_repeat_count_++;
      }
    } else {
      this->pending_extended_feedback_candidate_ = true;
      this->pending_extended_feedback_swing_ = decoded_status.horizontal_swing;
      this->pending_extended_feedback_vane_ = decoded_status.horizontal_vane;
      this->pending_extended_feedback_3d_auto_ = decoded_status.three_d_auto;
      this->pending_extended_feedback_db16_ = db16;
      this->pending_extended_feedback_db17_ = db17;
      this->pending_extended_feedback_repeat_count_ = bootstrap_delay_elapsed ? 1U : 0U;
    }

    if (bootstrap_delay_elapsed &&
        this->pending_extended_feedback_repeat_count_ >= kStableStartupExtendedFeedbackRepeats) {
      ESP_LOGI(DIAG_TAG,
               "decode_filter: accepted bootstrap extended feedback vane=%u swing=%s 3d=%s len=%u "
               "db9=0x%02x db16=0x%02x db17=0x%02x repeats=%u",
               static_cast<unsigned int>(decoded_status.horizontal_vane),
               decoded_status.horizontal_swing ? "YES" : "NO", decoded_status.three_d_auto ? "YES" : "NO",
               static_cast<unsigned int>(frame.len), frame.data[DB9], db16, db17,
               static_cast<unsigned int>(this->pending_extended_feedback_repeat_count_));
      this->extended_louver_bootstrap_complete_ = true;
      this->pending_extended_feedback_candidate_ = false;
      this->pending_extended_feedback_repeat_count_ = 0U;
      return true;
    }

    ESP_LOGD(DIAG_TAG,
             "decode_filter: held bootstrap worker extended feedback vane=%u swing=%s 3d=%s len=%u "
             "db9=0x%02x db16=0x%02x db17=0x%02x repeats=%u delay_elapsed=%s",
             static_cast<unsigned int>(decoded_status.horizontal_vane), decoded_status.horizontal_swing ? "YES" : "NO",
             decoded_status.three_d_auto ? "YES" : "NO", static_cast<unsigned int>(frame.len), frame.data[DB9], db16,
             db17, static_cast<unsigned int>(this->pending_extended_feedback_repeat_count_),
             bootstrap_delay_elapsed ? "YES" : "NO");
    return false;
  }

  if (unchanged) {
    this->pending_extended_feedback_candidate_ = false;
    this->pending_extended_feedback_repeat_count_ = 0U;
    return true;
  }

  // Worker-sourced DB16/DB17 feedback is a composite horizontal/3D state.
  // After startup it is only authoritative for HA control state while
  // confirming a matching command; otherwise it is kept diagnostic-only.
  this->pending_extended_feedback_candidate_ = true;
  this->pending_extended_feedback_swing_ = decoded_status.horizontal_swing;
  this->pending_extended_feedback_vane_ = decoded_status.horizontal_vane;
  this->pending_extended_feedback_3d_auto_ = decoded_status.three_d_auto;
  this->pending_extended_feedback_db16_ = db16;
  this->pending_extended_feedback_db17_ = db17;
  this->pending_extended_feedback_repeat_count_ = 1U;

  ESP_LOGD(DIAG_TAG,
           "decode_filter: held uncommanded worker extended feedback vane=%u swing=%s 3d=%s len=%u "
           "db9=0x%02x db16=0x%02x db17=0x%02x pending=0x%08lx",
           static_cast<unsigned int>(decoded_status.horizontal_vane), decoded_status.horizontal_swing ? "YES" : "NO",
           decoded_status.three_d_auto ? "YES" : "NO", static_cast<unsigned int>(frame.len), frame.data[DB9], db16,
           db17, static_cast<unsigned long>(pending_mask));
  return false;
}

bool MhiAcCtrl::extended_feedback_matches_pending_(const MhiDecodedStatus& decoded_status) const {
  const uint32_t pending_mask = this->command_confirmation_.pending_mask();
  const auto& intent = this->command_confirmation_.pending_intent();

  if ((pending_mask & MHI_COMMAND_HORIZONTAL_VANE) != 0U) {
    if (!decoded_status.has_horizontal_vane) {
      return false;
    }

    if (intent.horizontal_vane == 8U) {
      if (!decoded_status.horizontal_swing) {
        return false;
      }
    } else if (intent.horizontal_vane >= 1U && intent.horizontal_vane <= 7U) {
      if (decoded_status.horizontal_swing || decoded_status.horizontal_vane != intent.horizontal_vane) {
        return false;
      }
    } else {
      return false;
    }
  }

  if ((pending_mask & MHI_COMMAND_THREE_D_AUTO) != 0U) {
    if (!decoded_status.has_3d_auto || decoded_status.three_d_auto != intent.three_d_auto) {
      return false;
    }

    if (intent.has_extended_louver_context) {
      if (!decoded_status.has_horizontal_vane) {
        return false;
      }

      if (intent.horizontal_vane == 8U) {
        if (!decoded_status.horizontal_swing) {
          return false;
        }
      } else if (intent.horizontal_vane >= 1U && intent.horizontal_vane <= 7U) {
        if (decoded_status.horizontal_swing || decoded_status.horizontal_vane != intent.horizontal_vane) {
          return false;
        }
      }
    }
  }

  return (pending_mask & kNoPendingExtendedFeedbackMask) != 0U;
}

void MhiAcCtrl::log_suspicious_status_change_(const char* field, int old_value, int new_value,
                                              const MhiFrameBuffer& frame) const {
  if (!this->using_rx_worker_) {
    return;
  }

  const uint32_t pending_mask =
      this->command_confirmation_.pending_mask() | this->state_.command().pending_command_mask();
  if ((pending_mask & kNoPendingExtendedFeedbackMask) != 0U) {
    return;
  }

  ESP_LOGD(DIAG_TAG,
           "decode_suspicious: field=%s old=%d new=%d source=worker len=%u db0=0x%02x db1=0x%02x "
           "db2=0x%02x db6=0x%02x db9=0x%02x db10=0x%02x db11=0x%02x db12=0x%02x "
           "db16=0x%02x db17=0x%02x pending=0x%08lx",
           field, old_value, new_value, static_cast<unsigned int>(frame.len), frame.data[DB0], frame.data[DB1],
           frame.data[DB2], frame.data[DB6], frame.data[DB9], frame.len > DB10 ? frame.data[DB10] : 0U,
           frame.len > DB11 ? frame.data[DB11] : 0U, frame.len > DB12 ? frame.data[DB12] : 0U,
           frame.len > DB16 ? frame.data[DB16] : 0U, frame.len > DB17 ? frame.data[DB17] : 0U,
           static_cast<unsigned long>(pending_mask));
}

void MhiAcCtrl::log_rejected_opdata_(const char* field, float value, const MhiFrameBuffer& frame) const {
  ESP_LOGW(DIAG_TAG,
           "decode_reject: field=%s value=%.2f len=%u db6=0x%02x db9=0x%02x db10=0x%02x db11=0x%02x db12=0x%02x", field,
           value, static_cast<unsigned int>(frame.len), frame.data[DB6], frame.data[DB9],
           frame.len > DB10 ? frame.data[DB10] : 0U, frame.len > DB11 ? frame.data[DB11] : 0U,
           frame.len > DB12 ? frame.data[DB12] : 0U);
}

void MhiAcCtrl::update_command_confirmation_(const MhiStatusState& status) {
  uint32_t confirmed_mask = this->command_confirmation_.observe_status(status);

  if (this->settled_extended_confirmation_mask_ != 0U) {
    confirmed_mask |= this->command_confirmation_.settle_pending_mask(this->settled_extended_confirmation_mask_);
    this->settled_extended_confirmation_mask_ = 0U;
  }

  if (confirmed_mask == 0U) {
    return;
  }

  const uint32_t now = millis();
  this->diagnostics_.stats().on_command_confirmed(confirmed_mask, now);

  if ((confirmed_mask & kNoPendingExtendedFeedbackMask) != 0U) {
    this->extended_louver_bootstrap_complete_ = true;
    this->pending_extended_feedback_candidate_ = false;
    this->pending_extended_feedback_repeat_count_ = 0U;
  }

  ESP_LOGI(DIAG_TAG, "command: confirmed mask=0x%08lx pending=0x%08lx", static_cast<unsigned long>(confirmed_mask),
           static_cast<unsigned long>(this->command_confirmation_.pending_mask()));
}

void MhiAcCtrl::check_command_confirmation_timeout_() {
  const uint32_t now = millis();
  const uint32_t timeout_mask = this->command_confirmation_.expire(now);

  if (timeout_mask == 0U) {
    return;
  }

  this->diagnostics_.stats().on_command_confirmation_timeout(timeout_mask, now);

  ESP_LOGW(DIAG_TAG, "command: confirmation timeout mask=0x%08lx", static_cast<unsigned long>(timeout_mask));
}

void MhiAcCtrl::suppress_duplicate_pending_commands_() {
  auto& command = this->state_.command();
  const uint32_t duplicate_mask = this->command_confirmation_.duplicate_pending_mask(command);

  if (duplicate_mask == 0U) {
    return;
  }

  command.clear_pending_mask(duplicate_mask);

  ESP_LOGD(DIAG_TAG, "command: duplicate pending request ignored mask=0x%08lx",
           static_cast<unsigned long>(duplicate_mask));
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
