#include "mhi_duplex_transport_adapter.h"

#include "esphome/core/hal.h"

namespace esphome {
namespace mhi_ac_ctrl {

void MhiDuplexTransportAdapter::bind(IMhiDuplexTransport* backend, bool supports_classified_worker) {
  backend_ = backend;
  supports_classified_worker_ = supports_classified_worker;
  staging_failures_.store(0U, std::memory_order_relaxed);
  health_ = {};
  last_error_ = {};
}

MhiTransportResult MhiDuplexTransportAdapter::setup() {
  staging_failures_.store(0U, std::memory_order_relaxed);
  health_ = {};
  health_.state = MhiTransportState::STARTING;
  health_.setup_at_ms = millis();
  last_error_ = {};

  if (backend_ == nullptr) {
    last_error_ = MhiTransportResult::failure(MhiTransportError::DRIVER_NOT_BOUND, "bind_duplex").error;
    health_.state = MhiTransportState::FAILED;
    health_.fault_latched = true;
    health_.transport_errors = 1U;
    return {false, last_error_};
  }

  if (!backend_->setup(pins_)) {
    last_error_ = MhiTransportResult::failure(MhiTransportError::DUPLEX_SETUP_FAILED, backend_->name()).error;
    health_.state = MhiTransportState::FAILED;
    health_.fault_latched = true;
    health_.transport_errors = 1U;
    return {false, last_error_};
  }

  health_.state = MhiTransportState::WAITING_FOR_TRAFFIC;
  return MhiTransportResult::success();
}

void MhiDuplexTransportAdapter::loop() {
  if (backend_ != nullptr) {
    backend_->loop();
  }
}

void MhiDuplexTransportAdapter::shutdown() {
  if (backend_ != nullptr) {
    backend_->shutdown();
  }
  health_.state = MhiTransportState::STOPPED;
}

std::size_t MhiDuplexTransportAdapter::read(uint8_t* dst, std::size_t max_len) {
  const std::size_t len = backend_ == nullptr ? 0U : backend_->read(dst, max_len);
  if (len > 0U) {
    health_.traffic_seen = true;
    health_.last_rx_activity_ms = millis();
    health_.rx_bytes += static_cast<uint32_t>(len);
    health_.state = MhiTransportState::HEALTHY;
  }
  return len;
}

bool MhiDuplexTransportAdapter::queue_tx(const MhiTxEnvelope& envelope) {
  if (!this->active_mode()) {
    return false;
  }

  if (backend_ == nullptr || !backend_->ready() || !envelope.valid()) {
    staging_failures_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }

  const bool staged = backend_->send(envelope);
  if (!staged) {
    staging_failures_.fetch_add(1U, std::memory_order_relaxed);
  }
  return staged;
}

bool MhiDuplexTransportAdapter::take_tx_completion(MhiTxCompletion& completion) {
  return this->active_mode() && backend_ != nullptr && backend_->take_tx_completion(completion);
}

void MhiDuplexTransportAdapter::set_active_mode(bool enabled) {
  active_mode_enabled_.store(enabled, std::memory_order_release);
  if (backend_ != nullptr) {
    backend_->set_active_mode(enabled);
  }
}

const char* MhiDuplexTransportAdapter::name() const {
  return backend_ == nullptr ? "none" : backend_->name();
}

const char* MhiDuplexTransportAdapter::rx_name() const {
  return this->name();
}

const char* MhiDuplexTransportAdapter::tx_name() const {
  return this->name();
}

bool MhiDuplexTransportAdapter::rx_ready() const {
  return backend_ != nullptr && backend_->ready();
}

bool MhiDuplexTransportAdapter::tx_ready() const {
  return this->rx_ready();
}

MhiTransportCapabilities MhiDuplexTransportAdapter::capabilities() const {
  MhiTransportCapabilities capabilities{};
  capabilities.integrated_duplex = true;
  capabilities.uses_bus_marker = false;
  capabilities.supports_tx = true;
  capabilities.supports_classified_worker = supports_classified_worker_;
  capabilities.supports_rx_byte_critical_sections = false;
  capabilities.supports_active_mode = true;
  return capabilities;
}

MhiTransportHealth MhiDuplexTransportAdapter::health() const {
  MhiTransportHealth snapshot = health_;
  snapshot.tx_completed = this->completed_tx_frames();
  snapshot.tx_failures = this->tx_failures();
  if (snapshot.tx_failures > 0U && snapshot.state == MhiTransportState::HEALTHY) {
    snapshot.state = MhiTransportState::DEGRADED;
  }
  return snapshot;
}

uint32_t MhiDuplexTransportAdapter::completed_tx_frames() const {
  return backend_ == nullptr ? 0U : backend_->completed_tx_frames();
}

uint32_t MhiDuplexTransportAdapter::tx_failures() const {
  return (backend_ == nullptr ? 0U : backend_->tx_failures()) + staging_failures_.load(std::memory_order_relaxed);
}

std::size_t MhiDuplexTransportAdapter::tx_completion_queue_depth() const {
  return backend_ == nullptr ? 0U : backend_->tx_completion_queue_depth();
}

std::size_t MhiDuplexTransportAdapter::tx_completion_queue_high_water() const {
  return backend_ == nullptr ? 0U : backend_->tx_completion_queue_high_water();
}

uint32_t MhiDuplexTransportAdapter::tx_completion_queue_dropped() const {
  return backend_ == nullptr ? 0U : backend_->tx_completion_queue_dropped();
}

std::size_t MhiDuplexTransportAdapter::rx_queue_depth() const {
  return backend_ == nullptr ? 0U : backend_->rx_queue_depth();
}

std::size_t MhiDuplexTransportAdapter::rx_queue_high_water() const {
  return backend_ == nullptr ? 0U : backend_->rx_queue_high_water();
}

uint32_t MhiDuplexTransportAdapter::rx_queue_overwritten() const {
  return backend_ == nullptr ? 0U : backend_->rx_queue_overwritten();
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
