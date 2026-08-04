#include "mhi_transport_manager.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_transport";

void MhiTransportManager::set_primary(IMhiTransport* transport) {
  primary_ = transport;
  active_ = transport;
  recovery_active_ = false;
  recovery_attempted_ = false;
  safe_mode_ = false;
  state_ = MhiTransportState::STOPPED;
  last_transport_error_ = {};
  primary_failure_ = {};
  recovery_failure_ = {};
  this->reset_transport_diagnostic_cursors_();
}

void MhiTransportManager::set_recovery(IMhiTransport* transport) {
  recovery_ = transport;
}

bool MhiTransportManager::setup() {
  active_ = primary_;
  recovery_active_ = false;
  recovery_attempted_ = false;
  safe_mode_ = false;
  state_ = MhiTransportState::STARTING;
  last_transport_error_ = {};
  primary_failure_ = {};
  recovery_failure_ = {};

  if (primary_ == nullptr) {
    last_transport_error_ =
        MhiTransportResult::failure(MhiTransportError::DRIVER_NOT_BOUND, "bind_primary_transport").error;
    ESP_LOGE(TAG, "No primary transport was injected by codegen");
    this->enter_safe_mode_(last_transport_error_);
    this->publish_driver_diagnostics_();
    return false;
  }

  ESP_LOGCONFIG(TAG, "Transport setup: primary=%s recovery=%s", this->primary_name(), this->recovery_name());

  primary_->set_auto_tx_flush(auto_tx_flush_);
  primary_->set_rx_byte_critical_sections(rx_byte_critical_sections_);

  MhiTransportResult setup_result = primary_->setup();
  if (setup_result.ok && (!primary_->rx_ready() || !primary_->tx_ready())) {
    setup_result = MhiTransportResult::failure(MhiTransportError::INTERNAL_INVARIANT, "primary_ready_after_setup", 0,
                                               "transport setup succeeded without ready RX/TX");
  }

  bool ready = setup_result.ok;
  if (!ready) {
    primary_failure_ = setup_result.error;
    last_transport_error_ = setup_result.error;
    ready = this->activate_recovery_(setup_result);
  } else {
    state_ = primary_->health().state;
  }

  this->publish_driver_diagnostics_();

  ESP_LOGCONFIG(TAG, "Transport active: strategy=%s RX=%s ready=%s TX=%s ready=%s recovery=%s state=%s",
                active_ == nullptr ? "none" : active_->name(), this->rx_name(), this->rx_ready() ? "YES" : "NO",
                this->tx_name(), this->tx_ready() ? "YES" : "NO", recovery_active_ ? "YES" : "NO",
                mhi_transport_state_name(this->state()));

  if (active_ != nullptr && active_->capabilities().uses_bus_marker) {
    ESP_LOGCONFIG(TAG, "TX ownership: split transport uses marker-armed real-time transmission");
  } else if (active_ != nullptr && active_->capabilities().integrated_duplex) {
    ESP_LOGCONFIG(TAG, "TX ownership: duplex transport stages one frame for the next GP-SPI transaction");
  }

  return ready && this->rx_ready() && this->tx_ready() && !safe_mode_;
}

bool MhiTransportManager::activate_recovery_(const MhiTransportResult& primary_result) {
  if (recovery_attempted_) {
    return this->enter_safe_mode_(primary_result.error);
  }
  recovery_attempted_ = true;

  if (transition_listener_ != nullptr) {
    transition_listener_->on_transport_switch_begin(primary_result.error);
  }

  IMhiTransport* failed_primary = primary_;
  active_ = nullptr;
  this->reset_transport_diagnostic_cursors_();

  if (failed_primary != nullptr) {
    failed_primary->shutdown();
  }

  if (recovery_ == nullptr || recovery_ == primary_) {
    ESP_LOGE(TAG, "Primary transport %s failed and no internal recovery transport is available",
             failed_primary == nullptr ? "none" : failed_primary->name());
    return this->enter_safe_mode_(primary_result.error);
  }

  ESP_LOGW(
      TAG, "Primary transport %s failed to start: error=%s operation=%s native=%ld; activating internal recovery %s",
      failed_primary == nullptr ? "none" : failed_primary->name(), mhi_transport_error_name(primary_result.error.code),
      primary_result.error.operation == nullptr ? "none" : primary_result.error.operation,
      static_cast<long>(primary_result.error.native_code), recovery_->name());

  recovery_->set_auto_tx_flush(auto_tx_flush_);
  recovery_->set_rx_byte_critical_sections(rx_byte_critical_sections_);

  MhiTransportResult recovery_result = recovery_->setup();
  if (recovery_result.ok && (!recovery_->rx_ready() || !recovery_->tx_ready())) {
    recovery_result = MhiTransportResult::failure(MhiTransportError::INTERNAL_INVARIANT, "recovery_ready_after_setup",
                                                  0, "recovery setup succeeded without ready RX/TX");
  }

  if (!recovery_result.ok) {
    recovery_failure_ = recovery_result.error;
    last_transport_error_ = recovery_result.error;
    recovery_->shutdown();
    ESP_LOGE(TAG, "Internal recovery transport %s failed: error=%s operation=%s native=%ld", recovery_->name(),
             mhi_transport_error_name(recovery_result.error.code),
             recovery_result.error.operation == nullptr ? "none" : recovery_result.error.operation,
             static_cast<long>(recovery_result.error.native_code));
    return this->enter_safe_mode_(recovery_result.error);
  }

  active_ = recovery_;
  recovery_active_ = true;
  safe_mode_ = false;
  state_ = MhiTransportState::RECOVERY_ACTIVE;
  this->reset_transport_diagnostic_cursors_();

  if (transition_listener_ != nullptr) {
    transition_listener_->on_transport_recovery_ready();
  }

  return true;
}

bool MhiTransportManager::enter_safe_mode_(const MhiTransportErrorDetail& reason) {
  if (active_ != nullptr) {
    active_->shutdown();
  }

  active_ = nullptr;
  recovery_active_ = false;
  safe_mode_ = true;
  state_ = MhiTransportState::SAFE_MODE;
  last_transport_error_ = reason;
  this->reset_transport_diagnostic_cursors_();

  if (transition_listener_ != nullptr) {
    transition_listener_->on_transport_safe_mode(reason);
  }

  ESP_LOGE(TAG, "Transport safe mode entered: error=%s operation=%s native=%ld", mhi_transport_error_name(reason.code),
           reason.operation == nullptr ? "none" : reason.operation, static_cast<long>(reason.native_code));
  return false;
}

void MhiTransportManager::loop() {
  if (active_ == nullptr || safe_mode_) {
    return;
  }
  active_->loop();
  if (!recovery_active_) {
    state_ = active_->health().state;
  }
  this->update_transport_diagnostics_();
}

void MhiTransportManager::shutdown() {
  if (active_ != nullptr) {
    active_->shutdown();
  }
  active_ = nullptr;
  state_ = MhiTransportState::STOPPED;
}

std::size_t MhiTransportManager::read_rx(uint8_t* dst, std::size_t max_len) {
  if (active_ == nullptr || safe_mode_) {
    return 0U;
  }

  const std::size_t len = active_->read(dst, max_len);
  if (len > 0U && diagnostics_ != nullptr) {
    const uint32_t now = millis();
    diagnostics_->stats().on_rx_chunk(now);
    diagnostics_->stats().on_rx_bytes(static_cast<uint32_t>(len), now);
  }
  this->update_transport_diagnostics_();
  return len;
}

bool MhiTransportManager::queue_tx(const MhiTxEnvelope& envelope) {
  if (active_ == nullptr || safe_mode_) {
    if (diagnostics_ != nullptr) {
      diagnostics_->stats().on_tx_failure();
    }
    return false;
  }

  const bool queued = active_->queue_tx(envelope);
  this->update_transport_diagnostics_();
  return queued;
}

bool MhiTransportManager::take_tx_completion(MhiTxCompletion& completion) {
  return active_ != nullptr && !safe_mode_ && active_->take_tx_completion(completion);
}

bool MhiTransportManager::has_pending_tx() const {
  return active_ != nullptr && !safe_mode_ && active_->has_pending_tx();
}

bool MhiTransportManager::flush_tx_on_bus_marker() {
  if (active_ == nullptr || safe_mode_) {
    return false;
  }
  const bool flushed = active_->flush_tx_on_bus_marker();
  this->update_transport_diagnostics_();
  return flushed;
}

std::size_t MhiTransportManager::tx_completion_queue_depth() const {
  return active_ == nullptr ? 0U : active_->tx_completion_queue_depth();
}

std::size_t MhiTransportManager::tx_completion_queue_high_water() const {
  return active_ == nullptr ? 0U : active_->tx_completion_queue_high_water();
}

uint32_t MhiTransportManager::tx_completion_queue_dropped() const {
  return active_ == nullptr ? 0U : active_->tx_completion_queue_dropped();
}

std::size_t MhiTransportManager::rx_queue_depth() const {
  return active_ == nullptr ? 0U : active_->rx_queue_depth();
}

std::size_t MhiTransportManager::rx_queue_high_water() const {
  return active_ == nullptr ? 0U : active_->rx_queue_high_water();
}

uint32_t MhiTransportManager::rx_queue_overwritten() const {
  return active_ == nullptr ? 0U : active_->rx_queue_overwritten();
}

void MhiTransportManager::set_auto_tx_flush(bool enabled) {
  auto_tx_flush_ = enabled;
  if (primary_ != nullptr) {
    primary_->set_auto_tx_flush(enabled);
  }
  if (recovery_ != nullptr && recovery_ != primary_) {
    recovery_->set_auto_tx_flush(enabled);
  }
}

bool MhiTransportManager::auto_tx_flush() const {
  return active_ != nullptr ? active_->auto_tx_flush() : auto_tx_flush_;
}

void MhiTransportManager::set_rx_byte_critical_sections(bool enabled) {
  rx_byte_critical_sections_ = enabled;
  if (primary_ != nullptr) {
    primary_->set_rx_byte_critical_sections(enabled);
  }
  if (recovery_ != nullptr && recovery_ != primary_) {
    recovery_->set_rx_byte_critical_sections(enabled);
  }
}

bool MhiTransportManager::rx_byte_critical_sections() const {
  return active_ != nullptr ? active_->rx_byte_critical_sections() : rx_byte_critical_sections_;
}

bool MhiTransportManager::tx_uses_bus_marker() const {
  return active_ != nullptr && active_->capabilities().uses_bus_marker;
}

const char* MhiTransportManager::rx_name() const {
  return active_ == nullptr ? "none" : active_->rx_name();
}

const char* MhiTransportManager::tx_name() const {
  return active_ == nullptr ? "none" : active_->tx_name();
}

MhiTransportState MhiTransportManager::state() const {
  if (safe_mode_) {
    return MhiTransportState::SAFE_MODE;
  }
  if (recovery_active_) {
    return MhiTransportState::RECOVERY_ACTIVE;
  }
  if (active_ != nullptr) {
    return active_->health().state;
  }
  return state_;
}

void MhiTransportManager::publish_driver_diagnostics_() {
  if (diagnostics_ == nullptr) {
    return;
  }
  diagnostics_->set_rx_driver_name(this->rx_name());
  diagnostics_->set_tx_driver_name(this->tx_name());
  diagnostics_->set_rx_driver_ready(this->rx_ready());
  diagnostics_->set_tx_driver_ready(this->tx_ready());
}

void MhiTransportManager::update_transport_diagnostics_() {
  if (active_ == nullptr || diagnostics_ == nullptr) {
    return;
  }

  const uint32_t completed = active_->completed_tx_frames();
  const uint32_t failures = active_->tx_failures();

  while (last_transport_tx_completed_ != completed) {
    diagnostics_->stats().on_tx_frame(millis());
    last_transport_tx_completed_++;
  }

  while (last_transport_tx_failures_ != failures) {
    diagnostics_->stats().on_tx_failure();
    last_transport_tx_failures_++;
  }
}

void MhiTransportManager::reset_transport_diagnostic_cursors_() {
  last_transport_tx_completed_ = active_ == nullptr ? 0U : active_->completed_tx_frames();
  last_transport_tx_failures_ = active_ == nullptr ? 0U : active_->tx_failures();
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
