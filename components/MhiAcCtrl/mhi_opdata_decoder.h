#pragma once

#include <cstdint>

#include "mhi_frame.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiDecodedOpData {
  bool valid{false};

  bool has_mode{false};
  uint8_t mode{0};

  bool has_setpoint{false};
  float setpoint_c{0.0f};

  bool has_return_air{false};
  float return_air_c{0.0f};

  bool has_outdoor_temp{false};
  float outdoor_temp_c{0.0f};

  bool has_compressor_frequency{false};
  float compressor_frequency_hz{0.0f};

  bool has_current{false};
  float current_a{0.0f};

  bool has_indoor_unit_fan_speed{false};
  uint8_t indoor_unit_fan_speed{0};

  bool has_outdoor_unit_fan_speed{false};
  uint8_t outdoor_unit_fan_speed{0};

  bool has_total_indoor_runtime{false};
  uint32_t total_indoor_runtime_hours{0};

  bool has_total_compressor_runtime{false};
  uint32_t total_compressor_runtime_hours{0};

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

  bool has_last_error{false};
  uint8_t last_error_code{0};
};

class MhiOpDataDecoder {
 public:
  static bool decode_mosi(const MhiFrameView& mosi, MhiDecodedOpData& out);
  static bool is_opdata_response(const MhiFrameView& mosi);
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
