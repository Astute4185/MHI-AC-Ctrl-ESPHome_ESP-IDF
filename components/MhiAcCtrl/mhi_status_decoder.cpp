#include "mhi_status_decoder.h"

#include "mhi_defs.h"

namespace esphome {
namespace mhi_ac_ctrl {

uint8_t MhiStatusDecoder::decode_mode(uint8_t db0) {
  return static_cast<uint8_t>((db0 & 0x1CU) >> 2U);
}

uint8_t MhiStatusDecoder::decode_fan(uint8_t db1, uint8_t db6) {
  if ((db6 & 0x40U) != 0U) {
    return 4U;
  }

  switch (db1 & 0x03U) {
    case 0x00U:
      return 1U;
    case 0x01U:
      return 2U;
    case 0x02U:
      return 3U;
    default:
      return 0U;
  }
}

bool MhiStatusDecoder::decode_mosi(const MhiFrameView& mosi, MhiDecodedStatus& out) {
  if (!mosi.valid()) {
    out = {};
    return false;
  }

  MhiDecodedStatus decoded{};
  decoded.valid = true;
  decoded.extended = mosi.is_33_byte();

  decoded.power = (mosi[DB0] & 0x01U) != 0U;

  decoded.mode_raw = static_cast<uint8_t>(mosi[DB0] & 0x1CU);
  decoded.mode = decode_mode(mosi[DB0]);

  decoded.fan_raw = static_cast<uint8_t>(mosi[DB1] & 0x07U);
  decoded.fan = decode_fan(mosi[DB1], mosi[DB6]);

  decoded.target_temp_c = static_cast<float>(mosi[DB2] & 0x7FU) / 2.0f;
  decoded.room_temp_c = static_cast<float>(static_cast<int>(mosi[DB3]) - 61) / 4.0f;

  decoded.error_code = mosi[DB4];

  decoded.vertical_raw = static_cast<uint8_t>((mosi[DB0] & 0xC0U) | ((mosi[DB1] & 0xB0U) >> 4U));

  decoded.vertical_swing = (mosi[DB0] & 0x40U) != 0U;

  if (decoded.vertical_swing) {
    decoded.vertical_vane = 0U;
  } else {
    decoded.vertical_vane = static_cast<uint8_t>(((mosi[DB1] >> 4U) & 0x03U) + 1U);
  }

  if (mosi.is_33_byte()) {
    decoded.has_horizontal_vane = true;

    decoded.horizontal_raw = static_cast<uint8_t>((mosi[DB16] & 0x07U) | ((mosi[DB17] & 0x01U) << 4U));

    decoded.horizontal_swing = (decoded.horizontal_raw & 0x10U) != 0U;

    if (decoded.horizontal_swing) {
      decoded.horizontal_vane = 0U;
    } else {
      decoded.horizontal_vane = static_cast<uint8_t>((decoded.horizontal_raw & 0x07U) + 1U);
    }

    decoded.has_3d_auto = true;
    decoded.three_d_auto = (mosi[DB17] & 0x04U) != 0U;
  }

  out = decoded;
  return true;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome