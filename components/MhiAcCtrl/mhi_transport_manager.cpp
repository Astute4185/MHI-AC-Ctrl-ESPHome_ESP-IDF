#include "mhi_transport_manager.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_transport";

void MhiTransportManager::configure(int sck_pin, int mosi_pin, int miso_pin, const std::string& rx_driver,
                                    const std::string& tx_driver, uint8_t frame_size_hint,
                                    uint32_t frame_start_idle_ms) {
  pins_.sck = sck_pin;
  pins_.mosi = mosi_pin;
  pins_.miso = miso_pin;

  rx_driver_name_ = rx_driver.empty() || rx_driver == "none" ? "fast_gpio" : rx_driver;
  tx_driver_name_ = tx_driver.empty() || tx_driver == "none" ? "fast_gpio" : tx_driver;

  MhiFastGpioConfig fast_gpio_config{};
  fast_gpio_config.frame_size_hint = frame_size_hint;
  fast_gpio_config.frame_start_idle_ms = frame_start_idle_ms;
  fast_gpio_.set_config(fast_gpio_config);

  this->resolve_drivers();
}

void MhiTransportManager::resolve_drivers() {
  // Step 8 tangible baseline:
  // FastGPIO implements both RX and TX because MHI is externally clocked by the AC.
  if (rx_driver_name_ == "fast_gpio" && tx_driver_name_ == "fast_gpio") {
    rx_ = &fast_gpio_;
    tx_ = &fast_gpio_;
    return;
  }

  ESP_LOGW(TAG, "Unsupported driver combination RX=%s TX=%s; falling back to fast_gpio", rx_driver_name_.c_str(),
           tx_driver_name_.c_str());

  rx_driver_name_ = "fast_gpio";
  tx_driver_name_ = "fast_gpio";

  rx_ = &fast_gpio_;
  tx_ = &fast_gpio_;
}

bool MhiTransportManager::setup() {
  ESP_LOGCONFIG(TAG, "Transport setup: requested RX=%s TX=%s", rx_driver_name_.c_str(), tx_driver_name_.c_str());

  // Only FastGPIO exists at this stage, and it backs both RX and TX.
  // Setup once to avoid duplicate GPIO reset/config work.
  rx_ready_ = fast_gpio_.setup(pins_);
  tx_ready_ = rx_ready_;

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

  if (diagnostics_ != nullptr) {
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