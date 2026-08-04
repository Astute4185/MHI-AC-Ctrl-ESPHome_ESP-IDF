#include "mhi_duplex_transport_adapter.h"

namespace esphome {
namespace mhi_ac_ctrl {

void MhiDuplexTransportAdapter::bind(IMhiDuplexTransport* backend, bool supports_classified_worker) {
  backend_ = backend;
  supports_classified_worker_ = supports_classified_worker;
  staging_failures_.store(0U, std::memory_order_relaxed);
}

bool MhiDuplexTransportAdapter::setup(const MhiTransportPins& pins) {
  staging_failures_.store(0U, std::memory_order_relaxed);
  return backend_ != nullptr && backend_->setup(pins);
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
}

std::size_t MhiDuplexTransportAdapter::read(uint8_t* dst, std::size_t max_len) {
  return backend_ == nullptr ? 0U : backend_->read(dst, max_len);
}

bool MhiDuplexTransportAdapter::queue_tx(const MhiTxEnvelope& envelope) {
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
  return backend_ != nullptr && backend_->take_tx_completion(completion);
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
  capabilities.supports_classified_worker = supports_classified_worker_;
  capabilities.supports_rx_byte_critical_sections = false;
  return capabilities;
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
