#pragma once

#include <cmath>
#include <cstdint>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "mhi_state.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiPublishTargets {
  climate::Climate* climate{nullptr};

  binary_sensor::BinarySensor* power_binary_sensor{nullptr};

  sensor::Sensor* room_temp_sensor{nullptr};
  sensor::Sensor* target_temp_sensor{nullptr};
  sensor::Sensor* outdoor_temp_sensor{nullptr};
  sensor::Sensor* return_air_sensor{nullptr};
  sensor::Sensor* compressor_frequency_sensor{nullptr};
  sensor::Sensor* current_sensor{nullptr};
  sensor::Sensor* indoor_unit_fan_speed_sensor{nullptr};
  sensor::Sensor* outdoor_unit_fan_speed_sensor{nullptr};
  sensor::Sensor* indoor_unit_total_run_time_sensor{nullptr};
  sensor::Sensor* compressor_total_run_time_sensor{nullptr};
  sensor::Sensor* energy_used_sensor{nullptr};
  sensor::Sensor* indoor_unit_thi_r1_sensor{nullptr};
  sensor::Sensor* indoor_unit_thi_r2_sensor{nullptr};
  sensor::Sensor* indoor_unit_thi_r3_sensor{nullptr};
  sensor::Sensor* outdoor_unit_tho_r1_sensor{nullptr};
  sensor::Sensor* outdoor_unit_expansion_valve_sensor{nullptr};
  sensor::Sensor* outdoor_unit_discharge_pipe_sensor{nullptr};
  sensor::Sensor* outdoor_unit_discharge_pipe_super_heat_sensor{nullptr};
  sensor::Sensor* protection_state_number_sensor{nullptr};

  binary_sensor::BinarySensor* defrost_binary_sensor{nullptr};
  binary_sensor::BinarySensor* vanes_3d_auto_enabled_binary_sensor{nullptr};

  switch_::Switch* vanes_3d_auto_switch{nullptr};

  text_sensor::TextSensor* error_code_text_sensor{nullptr};
  text_sensor::TextSensor* protection_state_text_sensor{nullptr};

  select::Select* vertical_vanes_select{nullptr};
  select::Select* horizontal_vanes_select{nullptr};
  select::Select* fan_speed_select{nullptr};
};

class MhiPublishBridge {
 public:
  void set_targets(const MhiPublishTargets& targets) {
    targets_ = targets;

    // Targets are registered by ESPHome component setup, which can happen after
    // the parent has already decoded stable frames. Force the next decoded
    // state to be republished so newly registered entities do not stay at NA
    // when the AC state has not changed.
    has_last_status_ = false;
    has_last_opdata_ = false;
  }

  void publish(const MhiStateStore& state);

 private:
  static climate::ClimateMode map_climate_mode(bool power, uint8_t mode);
  static climate::ClimateFanMode map_climate_fan(uint8_t fan);
  static climate::ClimateSwingMode map_climate_swing(const MhiStatusState& status);
  static const char* map_vertical_vane_option(const MhiStatusState& status);
  static const char* map_horizontal_vane_option(const MhiStatusState& status);
  static const char* map_fan_option(uint8_t fan);
  static const char* map_protection_state(uint8_t state);
  static bool float_changed(float old_value, float new_value);

  void publish_status(const MhiStatusState& status);
  void publish_opdata(const MhiOpDataState& opdata);

  MhiPublishTargets targets_{};

  bool has_last_status_{false};
  bool has_last_opdata_{false};

  MhiStatusState last_status_{};
  MhiOpDataState last_opdata_{};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome