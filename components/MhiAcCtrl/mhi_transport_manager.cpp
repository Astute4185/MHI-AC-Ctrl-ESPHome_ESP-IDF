#include "mhi_transport_manager.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_transport";

namespace {

inline uint32_t elapsed_us(uint32_t now_us, uint32_t then_us) {
  return static_cast<uint32_t>(now_us - then_us);
}

}  // namespace

void MhiTransportManager::configure(int sck_pin, int mosi_pin, int miso_pin, const std::string& rx_driver,
                                    const std::string& tx_driver, uint8_t frame_size_hint, uint32_t frame_start_idle_ms,
                                    uint32_t external_clock_byte_gap_us, uint32_t external_clock_frame_gap_us,
                                    uint32_t external_clock_min_edge_gap_us, const std::string& external_clock_edge,
                                    uint32_t external_clock_sample_delay_nops) {
  pins_.sck = sck_pin;
  pins_.mosi = mosi_pin;
  pins_.miso = miso_pin;

  rx_driver_name_ = rx_driver.empty() ? "fast_gpio_rx" : rx_driver;
  tx_driver_name_ = tx_driver.empty() ? "fast_gpio_tx" : tx_driver;

  MhiFastGpioRxConfig fast_gpio_rx_config{};
  fast_gpio_rx_config.frame_size_hint = frame_size_hint;
  fast_gpio_rx_config.frame_start_idle_ms = frame_start_idle_ms;
  fast_gpio_rx_.set_config(fast_gpio_rx_config);

#if MHI_ENABLE_SPLIT_TX_DRIVER
  MhiFastGpioTxConfig fast_gpio_tx_config{};
  fast_gpio_tx_config.frame_size_hint = frame_size_hint;
  fast_gpio_tx_config.frame_start_idle_ms = frame_start_idle_ms;
  fast_gpio_tx_config.max_exchange_time_ms = tx_marker_timeout_ms_;
  fast_gpio_tx_.set_config(fast_gpio_tx_config);
#endif

#if MHI_ENABLE_NATIVE_SPI_RX_DRIVER
  MhiNativeSpiRxConfig native_spi_rx_config{};
  native_spi_rx_config.frame_size_hint = frame_size_hint;
  native_spi_rx_.set_config(native_spi_rx_config);
#endif

#if MHI_ENABLE_EXTERNAL_CLOCK_RX_DRIVER
  MhiExternalClockRxConfig external_clock_rx_config{};
  external_clock_rx_config.frame_size_hint = frame_size_hint;
  external_clock_rx_config.byte_gap_reset_us = external_clock_byte_gap_us;
  external_clock_rx_config.frame_gap_reset_us = external_clock_frame_gap_us;
  external_clock_rx_config.min_edge_gap_us = external_clock_min_edge_gap_us;
  external_clock_rx_config.sample_edge =
      external_clock_edge == "falling" ? MhiExternalClockSampleEdge::FALLING : MhiExternalClockSampleEdge::RISING;
  external_clock_rx_config.sample_delay_nops = external_clock_sample_delay_nops;
  external_clock_rx_.set_config(external_clock_rx_config);
#endif

  portENTER_CRITICAL(&tx_mux_);
  pending_tx_ = false;
  pending_tx_len_ = 0U;
  tx_in_progress_ = false;
  pending_tx_generation_ = 0U;
  pending_tx_queued_after_marker_sequence_ = 0U;
  portEXIT_CRITICAL(&tx_mux_);
  last_consumed_bus_marker_sequence_ = 0U;
  last_stale_bus_marker_sequence_ = 0U;
  tx_backoff_until_ms_ = 0U;

  this->resolve_drivers();
}

void MhiTransportManager::resolve_drivers() {
#if MHI_ENABLE_SPLIT_TX_DRIVER
  if (rx_driver_name_ == "fast_gpio_rx" && tx_driver_name_ == "fast_gpio_tx") {
    rx_ = &fast_gpio_rx_;
    tx_ = &fast_gpio_tx_;
    return;
  }
#else
  if (rx_driver_name_ == "fast_gpio_rx" && tx_driver_name_ == "fast_gpio_tx") {
    rx_ = &fast_gpio_rx_;
    tx_ = nullptr;
    ESP_LOGW(TAG, "Split FastGPIO TX is not built for this target; TX disabled");
    return;
  }
#endif

#if MHI_ENABLE_NATIVE_SPI_RX_DRIVER
#if MHI_ENABLE_SPLIT_TX_DRIVER
  if (rx_driver_name_ == "native_spi_rx" && tx_driver_name_ == "fast_gpio_tx") {
    rx_ = &native_spi_rx_;
    tx_ = &fast_gpio_tx_;
    return;
  }
#endif
#endif

#if MHI_ENABLE_EXTERNAL_CLOCK_RX_DRIVER
  if (rx_driver_name_ == "external_clock_rx" && tx_driver_name_ == "none") {
    rx_ = &external_clock_rx_;
#if MHI_ENABLE_SPLIT_TX_DRIVER
    tx_ = &null_tx_;
#else
    tx_ = nullptr;
#endif
    return;
  }

#if MHI_ENABLE_SPLIT_TX_DRIVER
  if (rx_driver_name_ == "external_clock_rx" && tx_driver_name_ == "fast_gpio_tx") {
    rx_ = &external_clock_rx_;
    tx_ = &fast_gpio_tx_;
    return;
  }
#endif
#else
  if (rx_driver_name_ == "external_clock_rx") {
    ESP_LOGW(TAG, "external_clock_rx is only built for ESP32 and ESP32-S3; falling back to fast_gpio_rx/fast_gpio_tx");
  }
#endif

#if !MHI_ENABLE_NATIVE_SPI_RX_DRIVER
  if (rx_driver_name_ == "native_spi_rx") {
    ESP_LOGW(TAG, "native_spi_rx is only built for ESP32-S3; falling back to fast_gpio_rx/fast_gpio_tx");
  }
#endif

  ESP_LOGW(TAG, "Unsupported driver combination RX=%s TX=%s; falling back to fast_gpio_rx/fast_gpio_tx",
           rx_driver_name_.c_str(), tx_driver_name_.c_str());

  rx_driver_name_ = "fast_gpio_rx";
  tx_driver_name_ = "fast_gpio_tx";

  rx_ = &fast_gpio_rx_;
#if MHI_ENABLE_SPLIT_TX_DRIVER
  tx_ = &fast_gpio_tx_;
#else
  tx_ = nullptr;
#endif
}

bool MhiTransportManager::setup() {
  ESP_LOGCONFIG(TAG, "Transport setup: requested RX=%s TX=%s", rx_driver_name_.c_str(), tx_driver_name_.c_str());

  rx_ready_ = rx_ != nullptr && rx_->setup(pins_);
  tx_ready_ = tx_ == nullptr || tx_->setup(pins_);

  if (!rx_ready_ || !tx_ready_) {
    ESP_LOGW(TAG, "Transport RX=%s TX=%s failed to start; falling back to fast_gpio_rx/fast_gpio_tx",
             rx_driver_name_.c_str(), tx_driver_name_.c_str());

    rx_driver_name_ = "fast_gpio_rx";
    tx_driver_name_ = "fast_gpio_tx";
    rx_ = &fast_gpio_rx_;
#if MHI_ENABLE_SPLIT_TX_DRIVER
    tx_ = &fast_gpio_tx_;
#else
    tx_ = nullptr;
#endif
    rx_ready_ = rx_->setup(pins_);
    tx_ready_ = tx_ == nullptr || tx_->setup(pins_);
  }

  if (diagnostics_ != nullptr) {
    diagnostics_->set_rx_driver_name(this->rx_name());
    diagnostics_->set_tx_driver_name(this->tx_name());
    diagnostics_->set_rx_driver_ready(rx_ready_);
    diagnostics_->set_tx_driver_ready(tx_ready_);
  }

  ESP_LOGCONFIG(TAG, "Transport active: RX=%s ready=%s TX=%s ready=%s", this->rx_name(), rx_ready_ ? "YES" : "NO",
                this->tx_name(), tx_ready_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "TX armed marker scheduling: max_marker_age=%luus timeout=%lums fail_backoff=%lums",
                static_cast<unsigned long>(tx_marker_arm_max_age_us_),
                static_cast<unsigned long>(tx_marker_timeout_ms_), static_cast<unsigned long>(tx_failure_backoff_ms_));

  return rx_ready_ && tx_ready_;
}

void MhiTransportManager::loop() {
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

std::size_t MhiTransportManager::read_rx(uint8_t* dst, std::size_t max_len) {
  const std::size_t len = this->read_rx_raw_(dst, max_len);

  if (auto_tx_flush_) {
    this->flush_tx_on_bus_marker();
  }

  return len;
}

std::size_t MhiTransportManager::read_rx_for_worker(uint8_t* dst, std::size_t max_len) {
  // The RX worker must be side-effect free: it may drain RX bytes and update RX
  // diagnostics, but it must never flush pending TX or enter the blocking TX
  // shifter. TX remains owned by the main loop / future TX worker only.
  return this->read_rx_raw_(dst, max_len);
}

std::size_t MhiTransportManager::read_rx_raw_(uint8_t* dst, std::size_t max_len) {
  if (rx_ == nullptr || !rx_ready_ || dst == nullptr || max_len == 0U) {
    return 0U;
  }

  const std::size_t len = rx_->read(dst, max_len);

  if (len > 0U && diagnostics_ != nullptr) {
    const uint32_t now = millis();

    diagnostics_->stats().on_rx_chunk(now);
    diagnostics_->stats().on_rx_bytes(static_cast<uint32_t>(len), now);
  }

  return len;
}

bool MhiTransportManager::queue_tx(const uint8_t* data, std::size_t len) {
  if (tx_ == nullptr || !tx_ready_ || data == nullptr || len == 0U || len > kMhiMaxFrameBytes) {
    if (diagnostics_ != nullptr) {
      diagnostics_->stats().on_tx_failure();
    }

    return false;
  }

#if MHI_ENABLE_SPLIT_TX_DRIVER
  if (tx_ == &null_tx_) {
    return true;
  }
#endif

  this->queue_pending_tx_(data, len);
  return true;
}

bool MhiTransportManager::send_tx(const uint8_t* data, std::size_t len) {
  const bool queued = this->queue_tx(data, len);

  if (queued && auto_tx_flush_) {
    this->flush_tx_on_bus_marker();
  }

  return queued;
}

bool MhiTransportManager::flush_tx_on_bus_marker() {
  return this->flush_pending_tx_on_bus_marker_();
}

void MhiTransportManager::set_rx_byte_critical_sections(bool enabled) {
  fast_gpio_rx_.set_byte_critical_sections(enabled);

#if MHI_ENABLE_SPLIT_TX_DRIVER
  fast_gpio_tx_.set_byte_critical_sections(enabled);
#endif
}

bool MhiTransportManager::rx_byte_critical_sections() const {
  return fast_gpio_rx_.byte_critical_sections();
}

bool MhiTransportManager::tx_uses_bus_marker() const {
#if MHI_ENABLE_SPLIT_TX_DRIVER
  return tx_ == &fast_gpio_tx_;
#else
  return false;
#endif
}

const char* MhiTransportManager::rx_name() const {
  return rx_ == nullptr ? "none" : rx_->name();
}

const char* MhiTransportManager::tx_name() const {
  return tx_ == nullptr ? "none" : tx_->name();
}

void MhiTransportManager::queue_pending_tx_(const uint8_t* data, std::size_t len) {
  portENTER_CRITICAL(&tx_mux_);
  pending_tx_frame_.fill(0U);
  std::memcpy(pending_tx_frame_.data(), data, std::min<std::size_t>(len, pending_tx_frame_.size()));
  pending_tx_len_ = std::min<std::size_t>(len, pending_tx_frame_.size());
  pending_tx_ = pending_tx_len_ > 0U;
  pending_tx_generation_++;

  const MhiBusMarker marker = rx_ == nullptr ? MhiBusMarker{} : rx_->bus_marker();
  pending_tx_queued_after_marker_sequence_ = marker.valid ? marker.sequence : 0U;
  portEXIT_CRITICAL(&tx_mux_);
}

bool MhiTransportManager::pending_tx_available_() const {
  return pending_tx_ && pending_tx_len_ > 0U;
}

void MhiTransportManager::clear_pending_tx_() {
  pending_tx_ = false;
  pending_tx_len_ = 0U;
}

bool MhiTransportManager::flush_pending_tx_on_bus_marker_() {
  if (tx_ == nullptr || !tx_ready_ || rx_ == nullptr || !rx_ready_) {
    return false;
  }

  const MhiBusMarker marker = rx_->bus_marker();
  if (!marker.valid || marker.sequence == 0U) {
    return false;
  }

  const uint32_t marker_age_us = elapsed_us(micros(), marker.frame_end_us);
  const uint32_t now_ms = millis();

  std::array<uint8_t, kMhiMaxFrameBytes> frame{};
  std::size_t sent_len = 0U;
  uint32_t send_generation = 0U;

  portENTER_CRITICAL(&tx_mux_);

  if (!this->pending_tx_available_() || tx_in_progress_) {
    portEXIT_CRITICAL(&tx_mux_);
    return false;
  }

  if (tx_backoff_until_ms_ != 0U && static_cast<int32_t>(now_ms - tx_backoff_until_ms_) < 0) {
    portEXIT_CRITICAL(&tx_mux_);
    return false;
  }

  if (marker.sequence == last_consumed_bus_marker_sequence_ ||
      marker.sequence == pending_tx_queued_after_marker_sequence_) {
    portEXIT_CRITICAL(&tx_mux_);
    return false;
  }

  if (marker_age_us > tx_marker_arm_max_age_us_) {
    if (marker.sequence != last_stale_bus_marker_sequence_) {
      last_stale_bus_marker_sequence_ = marker.sequence;
      const std::size_t pending_len = pending_tx_len_;
      portEXIT_CRITICAL(&tx_mux_);
      ESP_LOGVV(TAG, "TX armed marker expired before attempt: sequence=%lu age=%luus max=%luus len=%u",
                static_cast<unsigned long>(marker.sequence), static_cast<unsigned long>(marker_age_us),
                static_cast<unsigned long>(tx_marker_arm_max_age_us_), static_cast<unsigned int>(pending_len));
      return false;
    }

    portEXIT_CRITICAL(&tx_mux_);
    return false;
  }

  last_consumed_bus_marker_sequence_ = marker.sequence;
  sent_len = pending_tx_len_;
  send_generation = pending_tx_generation_;
  frame = pending_tx_frame_;
  tx_in_progress_ = true;

  portEXIT_CRITICAL(&tx_mux_);

  const bool ok = tx_->send(frame.data(), sent_len);

#if MHI_ENABLE_SPLIT_TX_DRIVER
  const bool tx_disabled = tx_ == &null_tx_;
#else
  const bool tx_disabled = false;
#endif

  if (diagnostics_ != nullptr && !tx_disabled) {
    if (ok) {
      diagnostics_->stats().on_tx_frame(millis());
    } else {
      diagnostics_->stats().on_tx_failure();
    }
  }

  portENTER_CRITICAL(&tx_mux_);
  tx_in_progress_ = false;

  if (ok) {
    if (pending_tx_generation_ == send_generation) {
      this->clear_pending_tx_();
    }
    tx_backoff_until_ms_ = 0U;
    portEXIT_CRITICAL(&tx_mux_);
    return true;
  }

  if (pending_tx_generation_ == send_generation) {
    tx_backoff_until_ms_ = millis() + tx_failure_backoff_ms_;
  }
  portEXIT_CRITICAL(&tx_mux_);

  ESP_LOGD(TAG, "TX frame missed armed bus marker sequence=%lu age=%luus len=%u backoff=%lums",
           static_cast<unsigned long>(marker.sequence), static_cast<unsigned long>(marker_age_us),
           static_cast<unsigned int>(sent_len), static_cast<unsigned long>(tx_failure_backoff_ms_));
  return false;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
