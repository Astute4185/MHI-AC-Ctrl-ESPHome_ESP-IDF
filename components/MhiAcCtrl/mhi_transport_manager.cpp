#include "mhi_transport_manager.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_transport";

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

  pending_tx_ = false;
  pending_tx_len_ = 0U;
  pending_tx_queued_after_marker_sequence_ = 0U;
  last_consumed_bus_marker_sequence_ = 0U;

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

  return rx_ready_ && tx_ready_;
}

void MhiTransportManager::loop() {
  if (rx_ != nullptr) {
    rx_->loop();
  }

  if (tx_ != nullptr) {
    tx_->loop();
  }

  this->flush_pending_tx_on_bus_marker_();
}

std::size_t MhiTransportManager::read_rx(uint8_t* dst, std::size_t max_len) {
  if (rx_ == nullptr || !rx_ready_ || dst == nullptr || max_len == 0U) {
    return 0U;
  }

  const std::size_t len = rx_->read(dst, max_len);

  if (len > 0U && diagnostics_ != nullptr) {
    const uint32_t now = millis();

    diagnostics_->stats().on_rx_chunk(now);
    diagnostics_->stats().on_rx_bytes(static_cast<uint32_t>(len), now);
  }

  // FastGPIO RX publishes its marker from read(). External-clock RX publishes
  // its marker from the ISR when the frame chunk is emitted. In both cases,
  // use the marker to trigger queued TX rather than blind polling.
  this->flush_pending_tx_on_bus_marker_();

  return len;
}

bool MhiTransportManager::send_tx(const uint8_t* data, std::size_t len) {
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
  this->flush_pending_tx_on_bus_marker_();

  return true;
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
  pending_tx_frame_.fill(0U);
  std::memcpy(pending_tx_frame_.data(), data, std::min<std::size_t>(len, pending_tx_frame_.size()));
  pending_tx_len_ = std::min<std::size_t>(len, pending_tx_frame_.size());
  pending_tx_ = pending_tx_len_ > 0U;

  const MhiBusMarker marker = rx_ == nullptr ? MhiBusMarker{} : rx_->bus_marker();
  pending_tx_queued_after_marker_sequence_ = marker.valid ? marker.sequence : 0U;
}

bool MhiTransportManager::pending_tx_available_() const {
  return pending_tx_ && pending_tx_len_ > 0U;
}

void MhiTransportManager::clear_pending_tx_() {
  pending_tx_ = false;
  pending_tx_len_ = 0U;
}

void MhiTransportManager::flush_pending_tx_on_bus_marker_() {
  if (!this->pending_tx_available_() || tx_ == nullptr || !tx_ready_ || rx_ == nullptr || !rx_ready_) {
    return;
  }

  const MhiBusMarker marker = rx_->bus_marker();
  if (!marker.valid || marker.sequence == 0U || marker.sequence == last_consumed_bus_marker_sequence_ ||
      marker.sequence == pending_tx_queued_after_marker_sequence_) {
    return;
  }

  last_consumed_bus_marker_sequence_ = marker.sequence;

  const std::size_t sent_len = pending_tx_len_;
  const bool ok = tx_->send(pending_tx_frame_.data(), sent_len);
  this->clear_pending_tx_();

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

  if (!ok) {
    ESP_LOGD(TAG, "TX frame missed bus marker sequence=%lu len=%u", static_cast<unsigned long>(marker.sequence),
             static_cast<unsigned int>(sent_len));
  }
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
