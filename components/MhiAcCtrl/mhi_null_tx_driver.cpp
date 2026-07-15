#include "mhi_null_tx_driver.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const NULL_TX_TAG = "mhi_null_tx";

bool MhiNullTxDriver::setup(const MhiTransportPins& pins) {
  (void)pins;
  ready_ = true;
  ESP_LOGW(NULL_TX_TAG, "TX disabled. RX probe mode only; commands and opdata requests will not be sent to the AC.");
  return true;
}

bool MhiNullTxDriver::send(const uint8_t* data, std::size_t len) {
  if (!ready_ || data == nullptr || len == 0U) {
    return false;
  }

  dropped_frames_++;

  const uint32_t now = millis();
  if (last_log_ms_ == 0U || (now - last_log_ms_) >= 30000U) {
    last_log_ms_ = now;
    ESP_LOGI(NULL_TX_TAG, "probe: dropped_tx_frames=%lu", static_cast<unsigned long>(dropped_frames_));
  }

  return false;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
