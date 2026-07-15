#pragma once

#include "../mhi_ac_ctrl.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiSensors : public Component, public Parented<MhiAcCtrl> {
 public:
  void setup() override;
  void dump_config() override;

  void set_room_temperature(sensor::Sensor* sensor) {
    this->room_temperature_sensor_ = sensor;
  }

  void set_target_temperature(sensor::Sensor* sensor) {
    this->target_temperature_sensor_ = sensor;
  }

  void set_outdoor_temperature(sensor::Sensor* sensor) {
    this->outdoor_temperature_sensor_ = sensor;
  }

  void set_return_air_temperature(sensor::Sensor* sensor) {
    this->return_air_temperature_sensor_ = sensor;
  }

  void set_compressor_frequency(sensor::Sensor* sensor) {
    this->compressor_frequency_sensor_ = sensor;
  }

  void set_current_power(sensor::Sensor* sensor) {
    // Kept as current_power for YAML compatibility.
    // Internally this currently maps to AC current in amps.
    this->current_sensor_ = sensor;
  }

  void set_indoor_unit_fan_speed(sensor::Sensor* sensor) {
    this->indoor_unit_fan_speed_sensor_ = sensor;
  }

  void set_outdoor_unit_fan_speed(sensor::Sensor* sensor) {
    this->outdoor_unit_fan_speed_sensor_ = sensor;
  }

  void set_indoor_unit_total_run_time(sensor::Sensor* sensor) {
    this->indoor_unit_total_run_time_sensor_ = sensor;
  }

  void set_compressor_total_run_time(sensor::Sensor* sensor) {
    this->compressor_total_run_time_sensor_ = sensor;
  }

  void set_energy_used(sensor::Sensor* sensor) {
    this->energy_used_sensor_ = sensor;
  }

  void set_indoor_unit_thi_r1(sensor::Sensor* sensor) {
    this->indoor_unit_thi_r1_sensor_ = sensor;
  }

  void set_indoor_unit_thi_r2(sensor::Sensor* sensor) {
    this->indoor_unit_thi_r2_sensor_ = sensor;
  }

  void set_indoor_unit_thi_r3(sensor::Sensor* sensor) {
    this->indoor_unit_thi_r3_sensor_ = sensor;
  }

  void set_outdoor_unit_tho_r1(sensor::Sensor* sensor) {
    this->outdoor_unit_tho_r1_sensor_ = sensor;
  }

  void set_outdoor_unit_expansion_valve(sensor::Sensor* sensor) {
    this->outdoor_unit_expansion_valve_sensor_ = sensor;
  }

  void set_outdoor_unit_discharge_pipe(sensor::Sensor* sensor) {
    this->outdoor_unit_discharge_pipe_sensor_ = sensor;
  }

  void set_outdoor_unit_discharge_pipe_super_heat(sensor::Sensor* sensor) {
    this->outdoor_unit_discharge_pipe_super_heat_sensor_ = sensor;
  }

  void set_protection_state_number(sensor::Sensor* sensor) {
    this->protection_state_number_sensor_ = sensor;
  }

 protected:
  sensor::Sensor* room_temperature_sensor_{nullptr};
  sensor::Sensor* target_temperature_sensor_{nullptr};
  sensor::Sensor* outdoor_temperature_sensor_{nullptr};
  sensor::Sensor* return_air_temperature_sensor_{nullptr};
  sensor::Sensor* compressor_frequency_sensor_{nullptr};
  sensor::Sensor* current_sensor_{nullptr};
  sensor::Sensor* indoor_unit_fan_speed_sensor_{nullptr};
  sensor::Sensor* outdoor_unit_fan_speed_sensor_{nullptr};
  sensor::Sensor* indoor_unit_total_run_time_sensor_{nullptr};
  sensor::Sensor* compressor_total_run_time_sensor_{nullptr};
  sensor::Sensor* energy_used_sensor_{nullptr};
  sensor::Sensor* indoor_unit_thi_r1_sensor_{nullptr};
  sensor::Sensor* indoor_unit_thi_r2_sensor_{nullptr};
  sensor::Sensor* indoor_unit_thi_r3_sensor_{nullptr};
  sensor::Sensor* outdoor_unit_tho_r1_sensor_{nullptr};
  sensor::Sensor* outdoor_unit_expansion_valve_sensor_{nullptr};
  sensor::Sensor* outdoor_unit_discharge_pipe_sensor_{nullptr};
  sensor::Sensor* outdoor_unit_discharge_pipe_super_heat_sensor_{nullptr};
  sensor::Sensor* protection_state_number_sensor_{nullptr};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
