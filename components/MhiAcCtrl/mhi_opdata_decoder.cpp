#include "mhi_opdata_decoder.h"

#include "mhi_defs.h"

namespace esphome {
namespace mhi_ac_ctrl {

namespace {

bool high_group(const MhiFrameView& mosi) {
  return (mosi[DB6] & 0x80U) != 0U;
}

bool opdata_type(const MhiFrameView& mosi) {
  return (mosi[DB10] & 0x30U) == 0x10U;
}

bool opdata_type_0x20(const MhiFrameView& mosi) {
  return (mosi[DB10] & 0x30U) == 0x20U;
}

float heat_exchanger_temp_c(uint8_t value) {
  return (0.327f * static_cast<float>(value)) - 11.4f;
}

float indoor_thi_r2_temp_c(uint8_t value) {
  return (0.275f * static_cast<float>(value)) - 47.0f;
}

float discharge_pipe_temp_c(uint8_t value) {
  if (value < 0x02U) {
    return 30.0f;
  }

  return (static_cast<float>(value) / 2.0f) + 32.0f;
}

}  // namespace

bool MhiOpDataDecoder::is_opdata_response(const MhiFrameView& mosi) {
  if (!mosi.valid()) {
    return false;
  }

  const uint8_t group = mosi[DB9];

  if (group == 0x00U || group == 0xFFU) {
    return false;
  }

  // DB6[7] is not enough to identify opdata; normal status/command feedback can
  // also carry it. Real opdata responses need a response marker in DB10.
  //
  // Common responses use DB10[5:4] == 0x10. Some high-bank temperature values
  // use DB10[5:4] == 0x20. A few legacy/error-history style values use DB10 in
  // the 0x30 range while DB6[7] is set.
  if (opdata_type(mosi) || opdata_type_0x20(mosi)) {
    return true;
  }

  if (high_group(mosi) && (mosi[DB10] & 0xF0U) == 0x30U) {
    return true;
  }

  return false;
}

bool MhiOpDataDecoder::decode_mosi(const MhiFrameView& mosi, MhiDecodedOpData& out) {
  if (!is_opdata_response(mosi)) {
    out = {};
    return false;
  }

  MhiDecodedOpData decoded{};
  decoded.valid = true;

  const uint8_t group = mosi[DB9];
  const uint8_t item = mosi[DB10];
  const uint8_t value = mosi[DB11];
  const bool high = high_group(mosi);
  const bool normal_opdata = opdata_type(mosi);

  switch (group) {
    case 0x01U:
      // Current Redux synthetic mapping: MODE in lower 4 bits of DB10.
      decoded.has_mode = true;
      decoded.mode = static_cast<uint8_t>(item & 0x0FU);
      break;

    case 0x02U:
      if (high) {
        // Legacy mapping: MODE request/response.
        decoded.has_mode = true;
        decoded.mode = static_cast<uint8_t>(item & 0x0FU);
      } else {
        // Current Redux synthetic mapping: SET TEMP: DB11 / 2.
        decoded.has_setpoint = true;
        decoded.setpoint_c = static_cast<float>(value & 0x7FU) / 2.0f;
      }
      break;

    case 0x03U:
      // Backward-compatible synthetic mapping kept for older unit fixtures.
      decoded.has_return_air = true;
      decoded.return_air_c = static_cast<float>(static_cast<int>(value) - 61) / 4.0f;
      break;

    case 0x05U:
      if (high && item == 0x13U) {
        // Legacy mapping: SET TEMP: DB11 / 2.
        decoded.has_setpoint = true;
        decoded.setpoint_c = static_cast<float>(value & 0x7FU) / 2.0f;
      }
      break;

    case 0x80U:
      if (high) {
        if (opdata_type_0x20(mosi)) {
          decoded.has_return_air = true;
          decoded.return_air_c = (static_cast<float>(value) * 0.25f) - 15.0f;
        }
      } else if (normal_opdata) {
        decoded.has_outdoor_temp = true;
        decoded.outdoor_temp_c = (static_cast<float>(value) - 94.0f) * 0.25f;
      }
      break;

    case 0x81U:
      if (high && opdata_type_0x20(mosi)) {
        decoded.has_indoor_unit_thi_r1 = true;
        decoded.indoor_unit_thi_r1_c = heat_exchanger_temp_c(value);
      } else if (!high && normal_opdata) {
        decoded.has_indoor_unit_thi_r2 = true;
        decoded.indoor_unit_thi_r2_c = indoor_thi_r2_temp_c(value);
      }
      break;

    case 0x87U:
      if (high && normal_opdata) {
        decoded.has_indoor_unit_thi_r3 = true;
        decoded.indoor_unit_thi_r3_c = heat_exchanger_temp_c(value);
      }
      break;

    case 0x82U:
      if (!high && normal_opdata) {
        decoded.has_outdoor_unit_tho_r1 = true;
        decoded.outdoor_unit_tho_r1_c = heat_exchanger_temp_c(value);
      }
      break;

    case 0x11U:
      if (!high && normal_opdata) {
        const uint16_t raw = static_cast<uint16_t>((static_cast<uint16_t>(item) << 8U) | value) & 0x0FFFU;
        const uint8_t high_byte = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
        const uint8_t low_byte = static_cast<uint8_t>(raw & 0xFFU);
        decoded.has_compressor_frequency = true;
        decoded.compressor_frequency_hz =
            (static_cast<float>(high_byte) * 25.6f) + (static_cast<float>(low_byte) * 0.1f);
      }
      break;

    case 0x85U:
      if (!high && normal_opdata) {
        decoded.has_outdoor_unit_discharge_pipe = true;
        decoded.outdoor_unit_discharge_pipe_c = discharge_pipe_temp_c(value);
      }
      break;

    case 0x90U:
      if (!high && normal_opdata) {
        decoded.has_current = true;
        decoded.current_a = static_cast<float>(value) * 14.0f / 51.0f;
      }
      break;

    case 0xB1U:
      if (!high && normal_opdata) {
        decoded.has_outdoor_unit_discharge_pipe_super_heat = true;
        decoded.outdoor_unit_discharge_pipe_super_heat_c = static_cast<float>(value) / 2.0f;
      }
      break;

    case 0x7CU:
      if (!high && normal_opdata) {
        decoded.has_protection_state_number = true;
        decoded.protection_state_number = value;
      }
      break;

    case 0x0CU:
      if (!high && normal_opdata) {
        decoded.has_defrost = true;
        decoded.defrost = (item & 0x01U) != 0U;
      }
      break;

    case 0x13U:
      if (!high && normal_opdata) {
        decoded.has_outdoor_unit_expansion_valve = true;
        decoded.outdoor_unit_expansion_valve_pulses =
            static_cast<uint16_t>((static_cast<uint16_t>(mosi[DB12]) << 8U) | value);
      }
      break;

    case 0x15U:
      // Backward-compatible synthetic mapping kept for older unit fixtures.
      decoded.has_outdoor_temp = true;
      decoded.outdoor_temp_c = static_cast<float>(value);
      break;

    case 0x18U:
      // Backward-compatible synthetic mapping kept for older unit fixtures.
      if (item >= 0x10U) {
        decoded.has_compressor_frequency = true;
        decoded.compressor_frequency_hz =
            (static_cast<float>(item - 0x10U) * 25.6f) + (static_cast<float>(value) * 0.1f);
      }
      break;

    case 0x1DU:
      // Backward-compatible synthetic mapping kept for older unit fixtures.
      decoded.has_current = true;
      decoded.current_a = static_cast<float>(value) * 14.0f / 51.0f;
      break;

    case 0x1EU:
      if (high) {
        if (normal_opdata) {
          decoded.has_total_indoor_runtime = true;
          decoded.total_indoor_runtime_hours = static_cast<uint32_t>(value) * 100U;
        }
      } else if (item == 0x11U) {
        decoded.has_total_compressor_runtime = true;
        decoded.total_compressor_runtime_hours = static_cast<uint32_t>(value) * 100U;
      }
      break;

    case 0x1FU:
      if (high) {
        decoded.has_indoor_unit_fan_speed = true;
        decoded.indoor_unit_fan_speed = static_cast<uint8_t>(item & 0x0FU);
      } else {
        decoded.has_outdoor_unit_fan_speed = true;
        decoded.outdoor_unit_fan_speed = static_cast<uint8_t>(item & 0x0FU);
      }
      break;

    case 0x25U:
      // Current Redux synthetic mapping: TOTAL COMP RUN: DB11 * 100 hours.
      decoded.has_total_compressor_runtime = true;
      decoded.total_compressor_runtime_hours = static_cast<uint32_t>(value) * 100U;
      break;

    case 0x94U:
      if (high && normal_opdata) {
        const uint16_t raw_kwh = static_cast<uint16_t>((static_cast<uint16_t>(mosi[DB12]) << 8U) | value);
        decoded.has_energy_used = true;
        decoded.energy_used_kwh = static_cast<float>(raw_kwh) * 0.25f;
      }
      break;

    default:
      // Unknown or unsupported opdata group.
      break;
  }

  out = decoded;
  return true;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
