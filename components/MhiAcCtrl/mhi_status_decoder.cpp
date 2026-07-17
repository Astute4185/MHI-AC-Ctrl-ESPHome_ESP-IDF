#include "mhi_status_decoder.h"

#include "mhi_defs.h"

namespace esphome {
namespace mhi_ac_ctrl {

uint8_t MhiStatusDecoder::decode_mode(uint8_t db0) {
  return static_cast<uint8_t>((db0 & 0x1CU) >> 2U);
}

uint8_t MhiStatusDecoder::decode_fan(uint8_t db1, uint8_t) {
  const uint8_t fan = static_cast<uint8_t>(db1 & 0x07U);

  // Practical protocol fan codes:
  // 0=lowest fixed speed (Quiet on opt-in four-speed profile),
  // 1=Low, 2=Medium, 6=High, 7=Auto.
  // Do not map unknown codes to Auto, otherwise a bad command reports as Auto.
  switch (fan) {
    case 0U:
    case 1U:
    case 2U:
    case 6U:
    case 7U:
      return fan;

    default:
      return 0U;
  }
}

bool MhiStatusDecoder::is_extended_feedback_status_frame(const MhiFrameView& mosi) {
  if (!mosi.valid() || !mosi.is_33_byte()) {
    return false;
  }

  // DB9 carries opdata group IDs on opdata response frames. Normal 33-byte
  // status feedback uses the non-opdata marker values seen in status/command
  // traffic. Do not decode DB16/DB17 vane/3D feedback from opdata frames;
  // those bytes can contain unrelated data and cause false HA state changes.
  return mosi[DB9] == 0x00U || mosi[DB9] == 0xFFU;
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

  if (is_extended_feedback_status_frame(mosi)) {
    decoded.has_extended_louver_raw = true;
    decoded.extended_louver_db16 = mosi[DB16];
    decoded.extended_louver_db17 = mosi[DB17];

    decoded.horizontal_raw = static_cast<uint8_t>((mosi[DB16] & 0x07U) | ((mosi[DB17] & 0x01U) << 4U));

    decoded.horizontal_swing = (decoded.horizontal_raw & 0x10U) != 0U;

    if (decoded.horizontal_swing) {
      decoded.has_horizontal_vane = true;
      decoded.horizontal_vane = 0U;
    } else {
      const uint8_t horizontal_vane = static_cast<uint8_t>((decoded.horizontal_raw & 0x07U) + 1U);

      // Some frames report DB16[2:0] == 0x07 with no swing flag, which
      // normalises to vane 8. The public control surface only has positions
      // 1..7 plus swing, so treat this as unknown extended feedback rather than
      // publishing a false vane/3D state change or warning every frame.
      if (horizontal_vane <= 7U) {
        decoded.has_horizontal_vane = true;
        decoded.horizontal_vane = horizontal_vane;
      }
    }

    if (decoded.has_horizontal_vane) {
      decoded.has_3d_auto = true;
      decoded.three_d_auto = (mosi[DB17] & 0x04U) != 0U;
    }
  }

  out = decoded;
  return true;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome