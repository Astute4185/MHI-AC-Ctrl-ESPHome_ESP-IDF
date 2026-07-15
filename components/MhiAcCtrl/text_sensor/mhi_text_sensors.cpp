#include "mhi_text_sensors.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi.text_sensor";

void MhiTextSensors::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI text sensors");

  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "MHI text sensors have no parent component");
    return;
  }

  if (this->error_code_sensor_ != nullptr) {
    this->parent_->set_error_code_text_sensor(this->error_code_sensor_);
  }

  if (this->protection_state_sensor_ != nullptr) {
    this->parent_->set_protection_state_text_sensor(this->protection_state_sensor_);
  }
}

void MhiTextSensors::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Text Sensors");
  ESP_LOGCONFIG(TAG, "  Error code: %s", this->error_code_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Protection state: %s", this->protection_state_sensor_ != nullptr ? "YES" : "NO");
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome