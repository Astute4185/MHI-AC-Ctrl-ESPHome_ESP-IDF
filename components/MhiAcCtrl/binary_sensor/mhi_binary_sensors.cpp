#include "mhi_binary_sensors.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi.binary_sensor";

void MhiBinarySensors::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI binary sensors");

  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "MHI binary sensors have no parent component");
    return;
  }

  if (this->power_sensor_ != nullptr) {
    this->parent_->set_power_binary_sensor(this->power_sensor_);
  }

  if (this->defrost_sensor_ != nullptr) {
    this->parent_->set_defrost_binary_sensor(this->defrost_sensor_);
  }

  if (this->vanes_3d_auto_enabled_sensor_ != nullptr) {
    this->parent_->set_vanes_3d_auto_enabled_binary_sensor(this->vanes_3d_auto_enabled_sensor_);
  }

  if (this->transport_healthy_sensor_ != nullptr) {
    this->parent_->set_transport_healthy_binary_sensor(this->transport_healthy_sensor_);
  }

  if (this->transport_recovery_active_sensor_ != nullptr) {
    this->parent_->set_transport_recovery_active_binary_sensor(this->transport_recovery_active_sensor_);
  }

  if (this->transport_safe_mode_sensor_ != nullptr) {
    this->parent_->set_transport_safe_mode_binary_sensor(this->transport_safe_mode_sensor_);
  }
}

void MhiBinarySensors::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Binary Sensors");
  ESP_LOGCONFIG(TAG, "  Power: %s", this->power_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Defrost: %s", this->defrost_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  3D Auto enabled: %s", this->vanes_3d_auto_enabled_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Transport healthy: %s", this->transport_healthy_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Transport recovery active: %s",
                this->transport_recovery_active_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Transport safe mode: %s", this->transport_safe_mode_sensor_ != nullptr ? "YES" : "NO");
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome