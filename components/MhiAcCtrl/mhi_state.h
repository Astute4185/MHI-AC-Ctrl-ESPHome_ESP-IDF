#pragma once

#include <cstdint>

#include "mhi_command.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiStatusState {
  bool valid{false};

  bool power{false};
  uint8_t mode{0};
  uint8_t fan{0};

  float target_temp_c{0.0f};
  float room_temp_c{0.0f};

  uint8_t vertical_vane{0};

  bool has_horizontal_vane{false};
  bool horizontal_vane_swing{false};
  uint8_t horizontal_vane{0};

  bool vanes_swing{false};

  bool has_3d_auto{false};
  bool three_d_auto{false};

  bool has_extended_louver_raw{false};
  uint8_t extended_louver_db16{0};
  uint8_t extended_louver_db17{0};

  uint8_t error_code{0};

  uint32_t last_update_ms{0};
};

struct MhiOpDataState {
  bool valid{false};

  bool has_outdoor_temp{false};
  float outdoor_temp_c{0.0f};

  bool has_return_air{false};
  float return_air_c{0.0f};

  bool has_compressor_frequency{false};
  float compressor_frequency_hz{0.0f};

  bool has_current{false};
  float current_a{0.0f};

  bool has_indoor_unit_fan_speed{false};
  uint8_t indoor_unit_fan_speed{0};

  bool has_outdoor_unit_fan_speed{false};
  uint8_t outdoor_unit_fan_speed{0};

  bool has_indoor_unit_total_run_time{false};
  uint32_t indoor_unit_total_run_time_hours{0};

  bool has_compressor_total_run_time{false};
  uint32_t compressor_total_run_time_hours{0};

  bool has_energy_used{false};
  float energy_used_kwh{0.0f};

  bool has_indoor_unit_thi_r1{false};
  float indoor_unit_thi_r1_c{0.0f};

  bool has_indoor_unit_thi_r2{false};
  float indoor_unit_thi_r2_c{0.0f};

  bool has_indoor_unit_thi_r3{false};
  float indoor_unit_thi_r3_c{0.0f};

  bool has_outdoor_unit_tho_r1{false};
  float outdoor_unit_tho_r1_c{0.0f};

  bool has_outdoor_unit_expansion_valve{false};
  uint16_t outdoor_unit_expansion_valve_pulses{0};

  bool has_outdoor_unit_discharge_pipe{false};
  float outdoor_unit_discharge_pipe_c{0.0f};

  bool has_outdoor_unit_discharge_pipe_super_heat{false};
  float outdoor_unit_discharge_pipe_super_heat_c{0.0f};

  bool has_protection_state_number{false};
  uint8_t protection_state_number{0};

  bool has_defrost{false};
  bool defrost{false};

  uint32_t last_update_ms{0};
};

class MhiStateStore {
 public:
  MhiStatusState& status() {
    return status_;
  }
  const MhiStatusState& status() const {
    return status_;
  }

  MhiOpDataState& opdata() {
    return opdata_;
  }
  const MhiOpDataState& opdata() const {
    return opdata_;
  }

  MhiCommandState& command() {
    return command_;
  }
  const MhiCommandState& command() const {
    return command_;
  }

 private:
  MhiStatusState status_;
  MhiOpDataState opdata_;
  MhiCommandState command_;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
