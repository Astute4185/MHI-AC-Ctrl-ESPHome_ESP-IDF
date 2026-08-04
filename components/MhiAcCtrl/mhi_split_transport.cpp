#include "mhi_split_transport.h"

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_split_transport";

void MhiSplitTransport::bind(IMhiRxDriver* rx, IMhiTxDriver* tx, bool supports_classified_worker,
                             bool uses_bus_marker) {
  rx_ = rx;
  tx_ = tx;
  supports_classified_worker_ = supports_classified_worker;
  uses_bus_marker_ = uses_bus_marker;
  rx_ready_ = false;
  tx_ready_ = false;
  health_ = {};
  last_error_ = {};
  this->reset_tx_state_();
}

MhiTransportResult MhiSplitTransport::setup(const MhiTransportPins& pins) {
  this->reset_tx_state_();
  completed_tx_frames_.store(0U, std::memory_order_relaxed);
  tx_failures_.store(0U, std::memory_order_relaxed);
  health_ = {};
  health_.state = MhiTransportState::STARTING;
  health_.setup_at_ms = millis();
  last_error_ = {};

  if (rx_ == nullptr) {
    rx_ready_ = false;
    tx_ready_ = tx_ == nullptr || tx_->setup(pins);
    last_error_ = MhiTransportResult::failure(MhiTransportError::DRIVER_NOT_BOUND, "bind_rx").error;
    health_.state = MhiTransportState::FAILED;
    health_.fault_latched = true;
    health_.transport_errors = 1U;
    return {false, last_error_};
  }

  rx_ready_ = rx_->setup(pins);
  tx_ready_ = tx_ == nullptr || tx_->setup(pins);

  if (!rx_ready_) {
    last_error_ = MhiTransportResult::failure(MhiTransportError::RX_SETUP_FAILED, rx_->name()).error;
  } else if (!tx_ready_) {
    last_error_ =
        MhiTransportResult::failure(MhiTransportError::TX_SETUP_FAILED, tx_ == nullptr ? "none" : tx_->name()).error;
  }

  if (last_error_.present()) {
    health_.state = MhiTransportState::FAILED;
    health_.fault_latched = true;
    health_.transport_errors = 1U;
    return {false, last_error_};
  }

  health_.state = MhiTransportState::WAITING_FOR_TRAFFIC;
  return MhiTransportResult::success();
}

void MhiSplitTransport::loop() {
  if (rx_ != nullptr) {
    rx_->loop();
  }
  if (tx_ != nullptr) {
    tx_->loop();
  }
  if (auto_tx_flush_) {
    this->flush_tx_on_bus_marker();
  }
}

void MhiSplitTransport::shutdown() {
  if (rx_ != nullptr) {
    rx_->shutdown();
  }
  this->reset_tx_state_();
  rx_ready_ = false;
  tx_ready_ = false;
  health_.state = MhiTransportState::STOPPED;
}

std::size_t MhiSplitTransport::read(uint8_t* dst, std::size_t max_len) {
  if (!rx_ready_ || rx_ == nullptr || dst == nullptr || max_len == 0U) {
    return 0U;
  }

  const std::size_t len = rx_->read(dst, max_len);
  if (len > 0U) {
    health_.traffic_seen = true;
    health_.last_rx_activity_ms = millis();
    health_.rx_bytes += static_cast<uint32_t>(len);
    health_.state = MhiTransportState::HEALTHY;
  }
  if (auto_tx_flush_) {
    this->flush_tx_on_bus_marker();
  }
  return len;
}

bool MhiSplitTransport::queue_tx(const MhiTxEnvelope& envelope) {
  if (!tx_ready_ || !envelope.valid()) {
    tx_failures_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }

  if (tx_ == nullptr) {
    tx_failures_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }

  if (this->tx_disabled_()) {
    if (envelope.is_command()) {
      MhiTxCompletion completion{};
      completion.generation = envelope.generation;
      completion.kind = envelope.kind;
      completion.command_mask = envelope.command_mask;
      completion.intent = envelope.intent;
      completion.success = false;
      completion.completed_at_ms = millis();

      this->lock_tx_();
      const bool stored = tx_completions_.push(completion);
      this->unlock_tx_();
      if (!stored) {
        tx_failures_.fetch_add(1U, std::memory_order_relaxed);
      }
    }
    return true;
  }

  this->queue_pending_tx_(envelope);
  return true;
}

bool MhiSplitTransport::take_tx_completion(MhiTxCompletion& completion) {
  this->lock_tx_();
  const bool available = tx_completions_.pop(completion);
  this->unlock_tx_();
  return available;
}

bool MhiSplitTransport::has_pending_tx() const {
  this->lock_tx_();
  const bool pending = pending_tx_ || tx_in_progress_;
  this->unlock_tx_();
  return pending;
}

bool MhiSplitTransport::flush_tx_on_bus_marker() {
  if (!uses_bus_marker_ || tx_ == nullptr || !tx_ready_ || rx_ == nullptr || !rx_ready_) {
    return false;
  }

  const MhiBusMarker marker = rx_->bus_marker();
  if (!marker.valid || marker.sequence == 0U) {
    return false;
  }

  const uint32_t marker_age_us = elapsed_us_(micros(), marker.frame_end_us);
  const uint32_t now_ms = millis();
  MhiTxEnvelope envelope{};
  uint32_t send_generation = 0U;

  this->lock_tx_();
  if (!this->pending_tx_available_() || tx_in_progress_) {
    this->unlock_tx_();
    return false;
  }
  if (tx_backoff_until_ms_ != 0U && static_cast<int32_t>(now_ms - tx_backoff_until_ms_) < 0) {
    this->unlock_tx_();
    return false;
  }
  if (marker.sequence == last_consumed_bus_marker_sequence_ ||
      marker.sequence == pending_tx_queued_after_marker_sequence_) {
    this->unlock_tx_();
    return false;
  }
  if (marker_age_us > tx_marker_arm_max_age_us_) {
    if (marker.sequence != last_stale_bus_marker_sequence_) {
      last_stale_bus_marker_sequence_ = marker.sequence;
      const std::size_t pending_len = pending_tx_envelope_.len;
      (void)pending_len;
      this->unlock_tx_();
      ESP_LOGVV(TAG, "TX armed marker expired before attempt: sequence=%lu age=%luus max=%luus len=%u",
                static_cast<unsigned long>(marker.sequence), static_cast<unsigned long>(marker_age_us),
                static_cast<unsigned long>(tx_marker_arm_max_age_us_), static_cast<unsigned int>(pending_len));
      return false;
    }
    this->unlock_tx_();
    return false;
  }

  last_consumed_bus_marker_sequence_ = marker.sequence;
  envelope = pending_tx_envelope_;
  send_generation = pending_tx_generation_;
  tx_in_progress_ = true;
  this->unlock_tx_();

  const bool ok = tx_->send(envelope.frame.data(), envelope.len);
  if (ok) {
    completed_tx_frames_.fetch_add(1U, std::memory_order_relaxed);
  } else {
    tx_failures_.fetch_add(1U, std::memory_order_relaxed);
  }

  MhiTxCompletion completion{};
  if (envelope.is_command()) {
    completion.generation = envelope.generation;
    completion.kind = envelope.kind;
    completion.command_mask = envelope.command_mask;
    completion.intent = envelope.intent;
    completion.success = ok;
    completion.completed_at_ms = millis();
  }

  this->lock_tx_();
  tx_in_progress_ = false;
  bool completion_stored = true;
  if (envelope.is_command()) {
    completion_stored = tx_completions_.push(completion);
  }
  if (ok) {
    if (pending_tx_generation_ == send_generation) {
      this->clear_pending_tx_();
    }
    tx_backoff_until_ms_ = 0U;
    this->unlock_tx_();
    if (!completion_stored) {
      tx_failures_.fetch_add(1U, std::memory_order_relaxed);
    }
    return true;
  }

  if (pending_tx_generation_ == send_generation) {
    tx_backoff_until_ms_ = millis() + tx_failure_backoff_ms_;
  }
  this->unlock_tx_();
  if (!completion_stored) {
    tx_failures_.fetch_add(1U, std::memory_order_relaxed);
  }

  ESP_LOGD(TAG, "TX frame missed armed bus marker sequence=%lu age=%luus len=%u backoff=%lums",
           static_cast<unsigned long>(marker.sequence), static_cast<unsigned long>(marker_age_us),
           static_cast<unsigned int>(envelope.len), static_cast<unsigned long>(tx_failure_backoff_ms_));
  return false;
}

void MhiSplitTransport::set_rx_byte_critical_sections(bool enabled) {
  if (rx_ != nullptr) {
    rx_->set_byte_critical_sections(enabled);
  }
  if (tx_ != nullptr) {
    tx_->set_byte_critical_sections(enabled);
  }
}

bool MhiSplitTransport::rx_byte_critical_sections() const {
  return rx_ != nullptr && rx_->byte_critical_sections();
}

const char* MhiSplitTransport::rx_name() const {
  return rx_ == nullptr ? "none" : rx_->name();
}

const char* MhiSplitTransport::tx_name() const {
  return tx_ == nullptr ? "none" : tx_->name();
}

MhiTransportCapabilities MhiSplitTransport::capabilities() const {
  MhiTransportCapabilities capabilities{};
  capabilities.integrated_duplex = false;
  capabilities.uses_bus_marker = uses_bus_marker_;
  capabilities.supports_tx = tx_ != nullptr && std::strcmp(tx_->name(), "none") != 0;
  capabilities.supports_classified_worker = supports_classified_worker_;
  capabilities.supports_rx_byte_critical_sections = rx_ != nullptr && rx_->supports_byte_critical_sections();
  return capabilities;
}

MhiTransportHealth MhiSplitTransport::health() const {
  MhiTransportHealth snapshot = health_;
  snapshot.tx_completed = completed_tx_frames_.load(std::memory_order_relaxed);
  snapshot.tx_failures = tx_failures_.load(std::memory_order_relaxed);
  if (snapshot.tx_failures > 0U && snapshot.state == MhiTransportState::HEALTHY) {
    snapshot.state = MhiTransportState::DEGRADED;
  }
  return snapshot;
}

std::size_t MhiSplitTransport::tx_completion_queue_depth() const {
  this->lock_tx_();
  const std::size_t value = tx_completions_.size();
  this->unlock_tx_();
  return value;
}

std::size_t MhiSplitTransport::tx_completion_queue_high_water() const {
  this->lock_tx_();
  const std::size_t value = tx_completions_.high_water_mark();
  this->unlock_tx_();
  return value;
}

uint32_t MhiSplitTransport::tx_completion_queue_dropped() const {
  this->lock_tx_();
  const uint32_t value = tx_completions_.dropped();
  this->unlock_tx_();
  return value;
}

void MhiSplitTransport::lock_tx_() const {
#ifdef USE_ESP_IDF
  portENTER_CRITICAL(&tx_mux_);
#else
  tx_mux_.lock();
#endif
}

void MhiSplitTransport::unlock_tx_() const {
#ifdef USE_ESP_IDF
  portEXIT_CRITICAL(&tx_mux_);
#else
  tx_mux_.unlock();
#endif
}

void MhiSplitTransport::reset_tx_state_() {
  this->lock_tx_();
  pending_tx_ = false;
  pending_tx_envelope_ = {};
  tx_completions_.reset();
  tx_in_progress_ = false;
  pending_tx_generation_ = 0U;
  pending_tx_queued_after_marker_sequence_ = 0U;
  this->unlock_tx_();
  last_consumed_bus_marker_sequence_ = 0U;
  last_stale_bus_marker_sequence_ = 0U;
  tx_backoff_until_ms_ = 0U;
}

void MhiSplitTransport::queue_pending_tx_(const MhiTxEnvelope& envelope) {
  this->lock_tx_();
  pending_tx_envelope_ = envelope;
  pending_tx_ = pending_tx_envelope_.valid();
  pending_tx_generation_++;
  const MhiBusMarker marker = rx_ == nullptr ? MhiBusMarker{} : rx_->bus_marker();
  pending_tx_queued_after_marker_sequence_ = marker.valid ? marker.sequence : 0U;
  this->unlock_tx_();
}

bool MhiSplitTransport::pending_tx_available_() const {
  return pending_tx_ && pending_tx_envelope_.valid();
}

void MhiSplitTransport::clear_pending_tx_() {
  pending_tx_ = false;
  pending_tx_envelope_ = {};
}

bool MhiSplitTransport::tx_disabled_() const {
  return tx_ != nullptr && std::strcmp(tx_->name(), "none") == 0;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
