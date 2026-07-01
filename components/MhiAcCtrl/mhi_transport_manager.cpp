#include "mhi_transport_manager.h"

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

  rx_driver_name_ = rx_driver.empty() || rx_driver == "none" ? "fast_gpio" : rx_driver;
  tx_driver_name_ = tx_driver.empty() ? "fast_gpio" : tx_driver;

  MhiFastGpioConfig fast_gpio_config{};
  fast_gpio_config.frame_size_hint = frame_size_hint;
  fast_gpio_config.frame_start_idle_ms = frame_start_idle_ms;
  fast_gpio_.set_config(fast_gpio_config);

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

  this->resolve_drivers();
}

void MhiTransportManager::resolve_drivers() {
  if (rx_driver_name_ == "fast_gpio" && tx_driver_name_ == "fast_gpio") {
    rx_ = &fast_gpio_;
    tx_ = &fast_gpio_;
    return;
  }

#if MHI_ENABLE_NATIVE_SPI_RX_DRIVER
  if (rx_driver_name_ == "native_spi_rx" && tx_driver_name_ == "fast_gpio") {
    rx_ = &native_spi_rx_;
    tx_ = &fast_gpio_tx_;
    return;
  }

#endif

#if MHI_ENABLE_EXTERNAL_CLOCK_RX_DRIVER
  if (rx_driver_name_ == "external_clock_rx" && tx_driver_name_ == "none") {
    rx_ = &external_clock_rx_;
    tx_ = &null_tx_;
    return;
  }

  if (rx_driver_name_ == "external_clock_rx" && tx_driver_name_ == "fast_gpio") {
    ESP_LOGW(TAG, "external_clock_rx + fast_gpio TX is intentionally blocked; use tx_driver: none for RX validation");
  }
#else
  if (rx_driver_name_ == "external_clock_rx") {
    ESP_LOGW(TAG, "external_clock_rx is only built for ESP32 and ESP32-S3; falling back to fast_gpio");
  }
#endif

#if !MHI_ENABLE_NATIVE_SPI_RX_DRIVER
  if (rx_driver_name_ == "native_spi_rx") {
    ESP_LOGW(TAG, "native_spi_rx is only built for ESP32-S3; falling back to fast_gpio");
  }
#endif

  ESP_LOGW(TAG, "Unsupported driver combination RX=%s TX=%s; falling back to fast_gpio", rx_driver_name_.c_str(),
           tx_driver_name_.c_str());

  rx_driver_name_ = "fast_gpio";
  tx_driver_name_ = "fast_gpio";

  rx_ = &fast_gpio_;
  tx_ = &fast_gpio_;
}

bool MhiTransportManager::setup() {
  ESP_LOGCONFIG(TAG, "Transport setup: requested RX=%s TX=%s", rx_driver_name_.c_str(), tx_driver_name_.c_str());

  if (rx_ == &fast_gpio_ && tx_ == &fast_gpio_) {
    // FastGPIO backs both RX and TX. Setup once to avoid duplicate GPIO reset/config work.
    rx_ready_ = fast_gpio_.setup(pins_);
    tx_ready_ = rx_ready_;
  } else {
    // Hybrid experimental path. Setup TX first so it can prepare MISO, or no-op cleanly in RX probe mode.
    tx_ready_ = tx_ != nullptr && tx_->setup(pins_);

    if (tx_ready_) {
      rx_ready_ = rx_ != nullptr && rx_->setup(pins_);
    } else {
      rx_ready_ = false;
    }

    if (!rx_ready_ || !tx_ready_) {
      ESP_LOGW(TAG, "Experimental transport RX=%s TX=%s failed to start; falling back to fast_gpio",
               rx_driver_name_.c_str(), tx_driver_name_.c_str());

      rx_driver_name_ = "fast_gpio";
      tx_driver_name_ = "fast_gpio";
      rx_ = &fast_gpio_;
      tx_ = &fast_gpio_;
      rx_ready_ = fast_gpio_.setup(pins_);
      tx_ready_ = rx_ready_;
    }
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

  // Avoid double-calling loop when the same driver backs RX and TX.
  if (tx_ != nullptr && tx_ != &fast_gpio_) {
    tx_->loop();
  }
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

  return len;
}

bool MhiTransportManager::send_tx(const uint8_t* data, std::size_t len) {
  if (tx_ == nullptr || !tx_ready_ || data == nullptr || len == 0U) {
    if (diagnostics_ != nullptr) {
      diagnostics_->stats().on_tx_failure();
    }

    return false;
  }

  const bool ok = tx_->send(data, len);

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

  return ok;
}

void MhiTransportManager::set_rx_byte_critical_sections(bool enabled) {
  fast_gpio_.set_rx_byte_critical_sections(enabled);

#if MHI_ENABLE_SPLIT_TX_DRIVER
  fast_gpio_tx_.set_byte_critical_sections(enabled);
#endif
}

bool MhiTransportManager::rx_byte_critical_sections() const {
  return fast_gpio_.rx_byte_critical_sections();
}

const char* MhiTransportManager::rx_name() const {
  return rx_ == nullptr ? "none" : rx_->name();
}

const char* MhiTransportManager::tx_name() const {
  return tx_ == nullptr ? "none" : tx_->name();
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
