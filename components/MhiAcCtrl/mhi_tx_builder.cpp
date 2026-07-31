#include "mhi_tx_builder.h"

#include "mhi_checksum.h"
#include "mhi_defs.h"
#include "mhi_louver_diag.h"

#ifdef USE_ESP32
#include "esphome/core/log.h"
#endif

namespace esphome {
namespace mhi_ac_ctrl {

namespace {

struct MhiOpdataRequest {
  uint8_t db6;
  uint8_t db9;
  uint32_t mask;
};
constexpr MhiOpdataRequest kMhiOpdataRequests[] = {
    {0xC0, 0x02, MHI_OPDATA_REQ_MODE},
    {0xC0, 0x05, MHI_OPDATA_REQ_TSETPOINT},
    {0xC0, 0x80, MHI_OPDATA_REQ_RETURN_AIR},
    {0xC0, 0x81, MHI_OPDATA_REQ_THI_R1},
    {0x40, 0x81, MHI_OPDATA_REQ_THI_R2},
    {0xC0, 0x87, MHI_OPDATA_REQ_THI_R3},
    {0xC0, 0x1F, MHI_OPDATA_REQ_IU_FANSPEED},
    {0xC0, 0x1E, MHI_OPDATA_REQ_TOTAL_IU_RUN},
    {0x40, 0x80, MHI_OPDATA_REQ_OUTDOOR},
    {0x40, 0x82, MHI_OPDATA_REQ_THO_R1},
    {0x40, 0x11, MHI_OPDATA_REQ_COMP},
    {0x40, 0x85, MHI_OPDATA_REQ_TD},
    {0x40, 0x90, MHI_OPDATA_REQ_CT},
    {0x40, 0xB1, MHI_OPDATA_REQ_TDSH},
    {0x40, 0x7C, MHI_OPDATA_REQ_PROTECTION_NO},
    {0x40, 0x1F, MHI_OPDATA_REQ_OU_FANSPEED},
    {0x40, 0x0C, MHI_OPDATA_REQ_DEFROST},
    {0x40, 0x1E, MHI_OPDATA_REQ_TOTAL_COMP_RUN},
    {0x40, 0x13, MHI_OPDATA_REQ_OU_EEV1},
    {0xC0, 0x94, MHI_OPDATA_REQ_KWH},
};
constexpr uint32_t kNoFramesPerOpDataCycle = 400;
constexpr uint8_t kMinEffectiveOpdataCount = 5;

uint32_t normalize_opdata_mask(uint32_t mask) {
  return mask == 0U ? kMhiDefaultOpdataMask : mask;
}

uint8_t get_enabled_opdata_count(uint32_t mask) {
  uint8_t count = 0;

  for (const auto& request : kMhiOpdataRequests) {
    if ((mask & request.mask) != 0U) {
      count++;
    }
  }

  return count;
}
const MhiOpdataRequest& get_enabled_opdata(uint32_t mask, uint8_t enabled_index) {
  uint8_t current = 0;

  for (const auto& request : kMhiOpdataRequests) {
    if ((mask & request.mask) != 0U) {
      if (current == enabled_index) {
        return request;
      }
      current++;
    }
  }

  return kMhiOpdataRequests[0];
}
uint8_t clamp_u8(float value, uint8_t min_value, uint8_t max_value) {
  if (value < static_cast<float>(min_value)) {
    return min_value;
  }
  if (value > static_cast<float>(max_value)) {
    return max_value;
  }
  return static_cast<uint8_t>(value);
}

#ifdef USE_ESP32
static const char* const TAG_LOUVER_DIAG = "mhi.louver_diag";
constexpr uint32_t kMhiLouverDiagCommandMask =
    MHI_COMMAND_VERTICAL_VANE | MHI_COMMAND_HORIZONTAL_VANE | MHI_COMMAND_THREE_D_AUTO;

unsigned int mhi_u8(uint8_t value) {
  return static_cast<unsigned int>(value);
}

void mhi_format_binary(uint8_t value, char (&out)[9]) {
  for (uint8_t index = 0; index < 8U; index++) {
    out[index] = (value & static_cast<uint8_t>(1U << (7U - index))) != 0U ? '1' : '0';
  }
  out[8] = '\0';
}

void mhi_log_louver_command(const MhiFrameBuffer& frame, const MhiTxBuildResult& result) {
  if ((result.encoded_command_mask & kMhiLouverDiagCommandMask) == 0U || frame.len != kMhiFrame33Bytes) {
    return;
  }

  const auto snapshot =
      mhi_decode_louver_diag_snapshot(frame.data[DB0], frame.data[DB1], frame.data[DB16], frame.data[DB17]);
  char db0_bits[9];
  char db1_bits[9];
  char db16_bits[9];
  char db17_bits[9];
  mhi_format_binary(snapshot.db0, db0_bits);
  mhi_format_binary(snapshot.db1, db1_bits);
  mhi_format_binary(snapshot.db16, db16_bits);
  mhi_format_binary(snapshot.db17, db17_bits);

  ESP_LOGI(TAG_LOUVER_DIAG, "TX command mask=0x%08x db0=0x%02x(%s) db1=0x%02x(%s) db16=0x%02x(%s) db17=0x%02x(%s)",
           static_cast<unsigned int>(result.encoded_command_mask), mhi_u8(snapshot.db0), db0_bits, mhi_u8(snapshot.db1),
           db1_bits, mhi_u8(snapshot.db16), db16_bits, mhi_u8(snapshot.db17), db17_bits);
  ESP_LOGI(TAG_LOUVER_DIAG,
           "TX candidates vertical{db0&c0=0x%02x db1&b0=0x%02x raw=0x%02x swing=%s vane=%u} "
           "horizontal{db16&07=0x%02x db17&01=%u raw=0x%02x swing=%s valid=%s vane=%u} "
           "3d{db17&04=%u state=%s} db17_command_bits{db17&0a=0x%02x}",
           mhi_u8(snapshot.vertical_db0_bits), mhi_u8(snapshot.vertical_db1_bits), mhi_u8(snapshot.vertical_raw),
           snapshot.vertical_swing ? "YES" : "NO", mhi_u8(snapshot.vertical_vane),
           mhi_u8(snapshot.horizontal_db16_bits), snapshot.horizontal_db17_bit != 0U ? 1U : 0U,
           mhi_u8(snapshot.horizontal_raw), snapshot.horizontal_swing ? "YES" : "NO",
           snapshot.horizontal_valid ? "YES" : "NO", mhi_u8(snapshot.horizontal_vane),
           snapshot.three_d_auto_bit != 0U ? 1U : 0U, snapshot.three_d_auto ? "ON" : "OFF",
           mhi_u8(snapshot.db17_command_bits));
}
#endif

}  // namespace
bool MhiTxBuilder::build_next_frame(MhiCommandState& command, MhiTxRuntime& runtime, const MhiTxBuildConfig& config,
                                    MhiFrameBuffer& out) {
  MhiTxBuildResult ignored{};
  return build_next_frame(command, runtime, config, out, ignored);
}

bool MhiTxBuilder::build_next_frame(MhiCommandState& command, MhiTxRuntime& runtime, const MhiTxBuildConfig& config,
                                    MhiFrameBuffer& out, MhiTxBuildResult& result) {
  result = {};
  if (config.frame_size != kMhiFrame20Bytes && config.frame_size != kMhiFrame33Bytes) {
    return false;
  }

  initialise_base_frame(out, config.frame_size);

  runtime.double_frame = !runtime.double_frame;

  out.data[DB14] = static_cast<uint8_t>(runtime.double_frame ? 0x04U : 0x00U);

  apply_opdata_request(out, runtime, config.enabled_opdata_mask);
  apply_commands(out, command, runtime, config, result);
#ifdef USE_ESP32
  mhi_log_louver_command(out, result);
#endif
  apply_checksums(out);

  return true;
}
void MhiTxBuilder::initialise_base_frame(MhiFrameBuffer& out, std::size_t frame_size) {
  out.clear();
  out.len = frame_size;

  out.data[SB0] = frame_size == kMhiFrame33Bytes ? 0xAAU : 0xA9U;
  out.data[SB1] = 0x00U;
  out.data[SB2] = 0x07U;

  out.data[DB3] = 0xFFU;
  out.data[DB9] = 0xFFU;
  out.data[DB10] = 0xFFU;
  out.data[DB11] = 0xFFU;
  out.data[DB12] = 0xFFU;
  out.data[DB13] = 0x0FU;
  if (frame_size == kMhiFrame33Bytes) {
    out.data[DB23] = 0xFFU;
    out.data[DB24] = 0xFFU;
    out.data[DB25] = 0xFFU;
  }
}

void MhiTxBuilder::apply_opdata_request(MhiFrameBuffer& out, MhiTxRuntime& runtime, uint32_t enabled_opdata_mask) {
  const uint32_t normalized_mask = normalize_opdata_mask(enabled_opdata_mask);
  const uint8_t enabled_count = get_enabled_opdata_count(normalized_mask);
  const uint8_t effective_cycle_count =
      enabled_count == 0U ? kMinEffectiveOpdataCount
                          : (enabled_count < kMinEffectiveOpdataCount ? kMinEffectiveOpdataCount : enabled_count);

  if ((runtime.frame_counter > (kNoFramesPerOpDataCycle / effective_cycle_count)) && runtime.double_frame) {
    runtime.frame_counter = 1U;
  }
  if (runtime.frame_counter++ <= 2U) {
    if (runtime.double_frame && runtime.error_opdata_count == 0U) {
      const uint8_t request_index =
          enabled_count == 0U ? 0U : static_cast<uint8_t>(runtime.opdata_index % enabled_count);

      const auto& request = get_enabled_opdata(normalized_mask, request_index);

      out.data[DB6] = request.db6;
      out.data[DB9] = request.db9;
      runtime.opdata_index = static_cast<uint8_t>((request_index + 1U) % (enabled_count == 0U ? 1U : enabled_count));
    }
  } else {
    out.data[DB6] = 0x80U;
    out.data[DB9] = 0xFFU;
  }
}

void MhiTxBuilder::apply_commands(MhiFrameBuffer& out, MhiCommandState& command, MhiTxRuntime& runtime,
                                  const MhiTxBuildConfig& config, MhiTxBuildResult& result) {
  if (runtime.double_frame) {
    out.data[DB0] = 0x00U;
    out.data[DB1] = 0x00U;
    out.data[DB2] = 0x00U;
    if (runtime.error_opdata_count > 0U) {
      out.data[DB6] = 0x80U;
      out.data[DB9] = 0xFFU;
      runtime.error_opdata_count--;
    }

    if (command.power_set) {
      out.data[DB0] |= static_cast<uint8_t>(0x02U | (command.power ? 0x01U : 0x00U));
      result.intent.power = command.power;
      command.power_set = false;
      result.encoded_command_mask |= MHI_COMMAND_POWER;
      result.intent.mask |= MHI_COMMAND_POWER;
    }
    if (command.mode_set) {
      out.data[DB0] |= encode_mode(command.mode);
      result.intent.mode = command.mode;
      command.mode_set = false;
      result.encoded_command_mask |= MHI_COMMAND_MODE;
      result.intent.mask |= MHI_COMMAND_MODE;
    }
    if (command.target_temp_set) {
      out.data[DB2] = encode_target_temp(command.target_temp_c);
      result.intent.target_temp_c = command.target_temp_c;
      command.target_temp_set = false;
      result.encoded_command_mask |= MHI_COMMAND_TARGET_TEMP;
      result.intent.mask |= MHI_COMMAND_TARGET_TEMP;
    }
    if (command.fan_set) {
      out.data[DB1] = encode_fan(command.fan);
      result.intent.fan = command.fan;
      command.fan_set = false;
      result.encoded_command_mask |= MHI_COMMAND_FAN;
      result.intent.mask |= MHI_COMMAND_FAN;
    }

    if (command.vertical_vane_set) {
      result.intent.vertical_vane = command.vertical_vane;
      if (command.vertical_vane == 5U) {
        out.data[DB0] |= 0xC0U;
      } else if (command.vertical_vane >= 1U && command.vertical_vane <= 4U) {
        out.data[DB0] |= 0x80U;
        out.data[DB1] |= static_cast<uint8_t>(0x80U | ((command.vertical_vane - 1U) << 4U));
      }

      command.vertical_vane_set = false;
      result.encoded_command_mask |= MHI_COMMAND_VERTICAL_VANE;
      result.intent.mask |= MHI_COMMAND_VERTICAL_VANE;
    }
    if (command.error_opdata_request) {
      out.data[DB6] = 0x80U;
      out.data[DB9] = 0x45U;
      command.error_opdata_request = false;
      result.encoded_command_mask |= MHI_COMMAND_ERROR_OPDATA_REQUEST;
      runtime.error_opdata_count = 2U;
    }
  }

  if (command.room_temp_override_set) {
    runtime.room_temp_override_raw = command.room_temp_override_raw;
    command.room_temp_override_set = false;
    result.encoded_command_mask |= MHI_COMMAND_ROOM_TEMP_OVERRIDE;
  }
  out.data[DB3] = runtime.room_temp_override_raw;

  if (out.len == kMhiFrame33Bytes) {
    const bool horizontal_requested = command.horizontal_vane_set;
    const bool three_d_requested = command.three_d_auto_set;
    const bool has_extended_request = horizontal_requested || three_d_requested;
    if (has_extended_request) {
      // DB16/DB17 are one composite register domain. Start from the last
      // confirmed state, overlay only the requested fields, then encode the
      // complete command. This prevents horizontal writes from clearing 3D
      // Auto and prevents 3D writes from changing the horizontal state.
      const bool has_preserved_context = config.has_extended_louver_state;
      bool desired_horizontal_swing = false;
      uint8_t desired_horizontal_vane = 1U;
      bool desired_three_d_auto = false;
      uint8_t preserved_swing_db16 = 0U;

      if (has_preserved_context) {
        desired_horizontal_swing = config.extended_louver_horizontal_swing;
        if (config.extended_louver_horizontal_vane >= 1U && config.extended_louver_horizontal_vane <= 7U) {
          desired_horizontal_vane = config.extended_louver_horizontal_vane;
        } else {
          const uint8_t raw_position = static_cast<uint8_t>(config.extended_louver_db16 & 0x07U);
          if (raw_position <= 6U) {
            desired_horizontal_vane = static_cast<uint8_t>(raw_position + 1U);
          }
        }
        desired_three_d_auto = config.extended_louver_three_d_auto;
        preserved_swing_db16 = config.extended_louver_db16;
      }

      if (horizontal_requested) {
        desired_horizontal_swing = command.horizontal_vane == 8U;
        if (!desired_horizontal_swing && command.horizontal_vane >= 1U && command.horizontal_vane <= 7U) {
          desired_horizontal_vane = command.horizontal_vane;
        }
        result.encoded_command_mask |= MHI_COMMAND_HORIZONTAL_VANE;
        result.intent.mask |= MHI_COMMAND_HORIZONTAL_VANE;
        command.horizontal_vane_set = false;
      }

      if (three_d_requested) {
        desired_three_d_auto = command.three_d_auto;
        result.encoded_command_mask |= MHI_COMMAND_THREE_D_AUTO;
        result.intent.mask |= MHI_COMMAND_THREE_D_AUTO;
        command.three_d_auto_set = false;
      }

      if (desired_horizontal_swing) {
        // MOSI feedback retains the previous fixed DB16 position while swing
        // is active. Preserve it when known and use DB17 bit 0 as the
        // authoritative swing state.
        out.data[DB16] = preserved_swing_db16;
      } else if (horizontal_requested || has_preserved_context) {
        out.data[DB16] = static_cast<uint8_t>(0x10U | (desired_horizontal_vane - 1U));
      } else {
        // Preserve the pre-spike fallback for a 3D-only request received
        // before the first extended-louver status has been learned.
        out.data[DB16] = 0x00U;
      }
      out.data[DB17] = static_cast<uint8_t>(0x0AU | (desired_horizontal_swing ? 0x01U : 0x00U) |
                                            (desired_three_d_auto ? 0x04U : 0x00U));

      result.intent.horizontal_vane = desired_horizontal_swing ? 8U : desired_horizontal_vane;
      result.intent.three_d_auto = desired_three_d_auto;
      // Companion-state confirmation is safe only when the preserved state is
      // known, or when both halves were explicitly supplied in this command.
      result.intent.has_extended_louver_context = horizontal_requested || has_preserved_context;
    }
  } else {
    if (command.horizontal_vane_set) {
      command.horizontal_vane_set = false;
      result.unsupported_command_mask |= MHI_COMMAND_HORIZONTAL_VANE;
    }
    if (command.three_d_auto_set) {
      command.three_d_auto_set = false;
      result.unsupported_command_mask |= MHI_COMMAND_THREE_D_AUTO;
    }
  }
}

void MhiTxBuilder::apply_checksums(MhiFrameBuffer& out) {
  uint16_t checksum = mhi_calc_checksum(out.data);
  out.data[CBH] = static_cast<uint8_t>((checksum >> 8U) & 0xFFU);
  out.data[CBL] = static_cast<uint8_t>(checksum & 0xFFU);
  if (out.len == kMhiFrame33Bytes) {
    checksum = mhi_calc_checksum_frame33(out.data);
    out.data[CBL2] = static_cast<uint8_t>(checksum & 0xFFU);
  }
}

uint8_t MhiTxBuilder::encode_mode(uint8_t mode) {
  // Input is normalised protocol mode:
  // 0=Auto, 1=Dry, 2=Cool, 3=Fan, 4=Heat.
  return static_cast<uint8_t>(0x20U | ((mode & 0x07U) << 2U));
}
uint8_t MhiTxBuilder::encode_fan(uint8_t fan) {
  // Preserve legacy fan command shape:
  // DB1[3] is the set bit, lower bits carry the old fan code.
  return static_cast<uint8_t>(0x08U | (fan & 0x07U));
}

uint8_t MhiTxBuilder::encode_target_temp(float target_temp_c) {
  // Protocol setpoint is half-degree units in DB2[6:0], with DB2[7] as set bit.
  const uint8_t half_degrees = clamp_u8((target_temp_c * 2.0f) + 0.5f, 0U, 0x7FU);
  return static_cast<uint8_t>(0x80U | half_degrees);
}
}  // namespace mhi_ac_ctrl
}  // namespace esphome
