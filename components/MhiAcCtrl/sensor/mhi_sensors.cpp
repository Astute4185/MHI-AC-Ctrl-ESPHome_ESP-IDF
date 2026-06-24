#include "mhi_sensors.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi.sensors";

void MhiSensors::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI sensors");

  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "MHI sensors have no parent component");
    return;
  }

  if (this->room_temperature_sensor_ != nullptr) {
    this->parent_->set_room_temp_sensor(this->room_temperature_sensor_);
  }

  if (this->target_temperature_sensor_ != nullptr) {
    this->parent_->set_target_temp_sensor(this->target_temperature_sensor_);
  }

  if (this->outdoor_temperature_sensor_ != nullptr) {
    this->parent_->set_outdoor_temp_sensor(this->outdoor_temperature_sensor_);
  }

  if (this->return_air_temperature_sensor_ != nullptr) {
    this->parent_->set_return_air_sensor(this->return_air_temperature_sensor_);
  }

  if (this->compressor_frequency_sensor_ != nullptr) {
    this->parent_->set_compressor_frequency_sensor(this->compressor_frequency_sensor_);
  }

  if (this->current_sensor_ != nullptr) {
    this->parent_->set_current_sensor(this->current_sensor_);
  }

  if (this->indoor_unit_fan_speed_sensor_ != nullptr) {
    this->parent_->set_indoor_unit_fan_speed_sensor(this->indoor_unit_fan_speed_sensor_);
  }

  if (this->outdoor_unit_fan_speed_sensor_ != nullptr) {
    this->parent_->set_outdoor_unit_fan_speed_sensor(this->outdoor_unit_fan_speed_sensor_);
  }

  if (this->indoor_unit_total_run_time_sensor_ != nullptr) {
    this->parent_->set_indoor_unit_total_run_time_sensor(this->indoor_unit_total_run_time_sensor_);
  }

  if (this->compressor_total_run_time_sensor_ != nullptr) {
    this->parent_->set_compressor_total_run_time_sensor(this->compressor_total_run_time_sensor_);
  }

  if (this->energy_used_sensor_ != nullptr) {
    this->parent_->set_energy_used_sensor(this->energy_used_sensor_);
  }

  if (this->indoor_unit_thi_r1_sensor_ != nullptr) {
    this->parent_->set_indoor_unit_thi_r1_sensor(this->indoor_unit_thi_r1_sensor_);
  }

  if (this->indoor_unit_thi_r2_sensor_ != nullptr) {
    this->parent_->set_indoor_unit_thi_r2_sensor(this->indoor_unit_thi_r2_sensor_);
  }

  if (this->indoor_unit_thi_r3_sensor_ != nullptr) {
    this->parent_->set_indoor_unit_thi_r3_sensor(this->indoor_unit_thi_r3_sensor_);
  }

  if (this->outdoor_unit_tho_r1_sensor_ != nullptr) {
    this->parent_->set_outdoor_unit_tho_r1_sensor(this->outdoor_unit_tho_r1_sensor_);
  }

  if (this->outdoor_unit_expansion_valve_sensor_ != nullptr) {
    this->parent_->set_outdoor_unit_expansion_valve_sensor(this->outdoor_unit_expansion_valve_sensor_);
  }

  if (this->outdoor_unit_discharge_pipe_sensor_ != nullptr) {
    this->parent_->set_outdoor_unit_discharge_pipe_sensor(this->outdoor_unit_discharge_pipe_sensor_);
  }

  if (this->outdoor_unit_discharge_pipe_super_heat_sensor_ != nullptr) {
    this->parent_->set_outdoor_unit_discharge_pipe_super_heat_sensor(
        this->outdoor_unit_discharge_pipe_super_heat_sensor_);
  }

  if (this->protection_state_number_sensor_ != nullptr) {
    this->parent_->set_protection_state_number_sensor(this->protection_state_number_sensor_);
  }
}

void MhiSensors::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Sensors");
  ESP_LOGCONFIG(TAG, "  Room temperature: %s", this->room_temperature_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Target temperature: %s", this->target_temperature_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Outdoor temperature: %s", this->outdoor_temperature_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Return air temperature: %s", this->return_air_temperature_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Compressor frequency: %s", this->compressor_frequency_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Current: %s", this->current_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Indoor unit fan speed: %s", this->indoor_unit_fan_speed_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Outdoor unit fan speed: %s", this->outdoor_unit_fan_speed_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Indoor unit total run time: %s",
                this->indoor_unit_total_run_time_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Compressor total run time: %s",
                this->compressor_total_run_time_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Energy used: %s", this->energy_used_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Indoor unit THI R1: %s", this->indoor_unit_thi_r1_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Indoor unit THI R2: %s", this->indoor_unit_thi_r2_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Indoor unit THI R3: %s", this->indoor_unit_thi_r3_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Outdoor unit THO R1: %s", this->outdoor_unit_tho_r1_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Outdoor unit expansion valve: %s",
                this->outdoor_unit_expansion_valve_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Outdoor unit discharge pipe: %s",
                this->outdoor_unit_discharge_pipe_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Outdoor unit discharge pipe super heat: %s",
                this->outdoor_unit_discharge_pipe_super_heat_sensor_ != nullptr ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Protection state number: %s", this->protection_state_number_sensor_ != nullptr ? "YES" : "NO");
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
