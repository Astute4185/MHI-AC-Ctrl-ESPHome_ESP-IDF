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
  last_transport_error_ = {};
  this->reset_transport_diagnostic_cursors_();
}

void MhiTransportManager::set_recovery(IMhiTransport* transport) {
  recovery_ = transport;
}

bool MhiTransportManager::setup() {
  active_ = primary_;
  recovery_active_ = false;
  last_transport_error_ = {};

  if (primary_ == nullptr) {
    last_transport_error_ =
        MhiTransportResult::failure(MhiTransportError::DRIVER_NOT_BOUND, "bind_primary_transport").error;
    ESP_LOGE(TAG, "No primary transport was injected by codegen");
    this->publish_driver_diagnostics_();
    return false;
  }

  ESP_LOGCONFIG(TAG, "Transport setup: primary=%s recovery=%s", this->primary_name(), this->recovery_name());

  primary_->set_auto_tx_flush(auto_tx_flush_);
  primary_->set_rx_byte_critical_sections(rx_byte_critical_sections_);

  MhiTransportResult setup_result = primary_->setup();
  bool ready = setup_result.ok;
  if (!ready) {
    last_transport_error_ = setup_result.error;
    ready = this->activate_recovery_(setup_result);
  }

  this->publish_driver_diagnostics_();

  ESP_LOGCONFIG(TAG, "Transport active: strategy=%s RX=%s ready=%s TX=%s ready=%s recovery=%s",
                active_ == nullptr ? "none" : active_->name(), this->rx_name(), this->rx_ready() ? "YES" : "NO",
                this->tx_name(), this->tx_ready() ? "YES" : "NO", recovery_active_ ? "YES" : "NO");

  if (active_ != nullptr && active_->capabilities().uses_bus_marker) {
    ESP_LOGCONFIG(TAG, "TX ownership: split transport uses marker-armed real-time transmission");
  } else if (active_ != nullptr && active_->capabilities().integrated_duplex) {
    ESP_LOGCONFIG(TAG, "TX ownership: duplex transport stages one frame for the next GP-SPI transaction");
  }

  return ready && this->rx_ready() && this->tx_ready();
}

bool MhiTransportManager::activate_recovery_(const MhiTransportResult& primary_result) {
  (void)primary_result;
  if (recovery_ == nullptr || recovery_ == primary_) {
    return false;
  }

  ESP_LOGW(TAG,
           "Primary transport %s failed to start: error=%s operation=%s native=%ld; activating internal recovery %s",
           primary_ == nullptr ? "none" : primary_->name(), mhi_transport_error_name(primary_result.error.code),
           primary_result.error.operation == nullptr ? "none" : primary_result.error.operation,
           static_cast<long>(primary_result.error.native_code), recovery_->name());

  if (primary_ != nullptr) {
    primary_->shutdown();
  }

  recovery_->set_auto_tx_flush(auto_tx_flush_);
  recovery_->set_rx_byte_critical_sections(rx_byte_critical_sections_);
  active_ = recovery_;
  recovery_active_ = true;
  this->reset_transport_diagnostic_cursors_();

  const MhiTransportResult recovery_result = recovery_->setup();
  if (!recovery_result.ok) {
    last_transport_error_ = recovery_result.error;
    return false;
  }

  return true;
}

void MhiTransportManager::loop() {
  if (active_ == nullptr) {
    return;
  }
  active_->loop();
  this->update_transport_diagnostics_();
}

void MhiTransportManager::shutdown() {
  if (active_ != nullptr) {
    active_->shutdown();
  }
}

std::size_t MhiTransportManager::read_rx(uint8_t* dst, std::size_t max_len) {
  if (active_ == nullptr) {
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
  if (active_ == nullptr) {
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
  return active_ != nullptr && active_->take_tx_completion(completion);
}

bool MhiTransportManager::has_pending_tx() const {
  return active_ != nullptr && active_->has_pending_tx();
}

bool MhiTransportManager::flush_tx_on_bus_marker() {
  if (active_ == nullptr) {
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
