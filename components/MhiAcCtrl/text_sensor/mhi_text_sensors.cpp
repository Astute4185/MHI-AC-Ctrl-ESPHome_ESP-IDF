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

  if (this->active_transport_sensor_ != nullptr) {
    this->parent_->set_active_transport_text_sensor(this->active_transport_sensor_);
  }

  if (this->transport_state_sensor_ != nullptr) {
    this->parent_->set_transport_state_text_sensor(this->transport_state_sensor_);
  }

  if (this->last_transport_error_sensor_ != nullptr) {
    this->parent_->set_last_transport_error_text_sensor(this->last_transport_error_sensor_);
  }
}

void MhiTextSensors::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Text Sensors");
  ESP_LOGCONFIG(TAG, "  Error code: %s", this->error_code_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Protection state: %s", this->protection_state_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Active transport: %s", this->active_transport_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Transport state: %s", this->transport_state_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Last transport error: %s", this->last_transport_error_sensor_ != nullptr ? "YES" : "NO");
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome