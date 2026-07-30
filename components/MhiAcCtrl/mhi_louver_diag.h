#pragma once

#include <cstdint>

namespace esphome {
namespace mhi_ac_ctrl {

// Candidate louver fields currently used by the decoder and TX builder.
// The diagnostic spike logs the complete source bytes as well as these masks
// so the hardware trace can prove or disprove the current interpretation.
constexpr uint8_t kMhiVerticalDb0Mask = 0xC0U;
constexpr uint8_t kMhiVerticalDb1Mask = 0xB0U;
constexpr uint8_t kMhiHorizontalDb16Mask = 0x07U;
constexpr uint8_t kMhiHorizontalSwingDb17Mask = 0x01U;
constexpr uint8_t kMhiThreeDAutoDb17Mask = 0x04U;
constexpr uint8_t kMhiLouverCommandDb17Mask = 0x0AU;

struct MhiLouverDiagSnapshot {
  uint8_t db0{0U};
  uint8_t db1{0U};
  uint8_t db16{0U};
  uint8_t db17{0U};

  uint8_t vertical_db0_bits{0U};
  uint8_t vertical_db1_bits{0U};
  uint8_t vertical_raw{0U};
  bool vertical_swing{false};
  uint8_t vertical_vane{0U};  // 1..4, or 5 for swing.

  uint8_t horizontal_db16_bits{0U};
  uint8_t horizontal_db17_bit{0U};
  uint8_t horizontal_raw{0U};
  bool horizontal_swing{false};
  bool horizontal_valid{false};
  uint8_t horizontal_vane{0U};  // 1..7, or 8 for swing.

  uint8_t three_d_auto_bit{0U};
  bool three_d_auto{false};
  uint8_t db17_command_bits{0U};
};

constexpr MhiLouverDiagSnapshot mhi_decode_louver_diag_snapshot(uint8_t db0, uint8_t db1, uint8_t db16, uint8_t db17) {
  MhiLouverDiagSnapshot out{};
  out.db0 = db0;
  out.db1 = db1;
  out.db16 = db16;
  out.db17 = db17;

  out.vertical_db0_bits = static_cast<uint8_t>(db0 & kMhiVerticalDb0Mask);
  out.vertical_db1_bits = static_cast<uint8_t>(db1 & kMhiVerticalDb1Mask);
  out.vertical_raw = static_cast<uint8_t>(out.vertical_db0_bits | (out.vertical_db1_bits >> 4U));
  out.vertical_swing = (db0 & 0x40U) != 0U;
  out.vertical_vane = out.vertical_swing ? 5U : static_cast<uint8_t>(((db1 >> 4U) & 0x03U) + 1U);

  out.horizontal_db16_bits = static_cast<uint8_t>(db16 & kMhiHorizontalDb16Mask);
  out.horizontal_db17_bit = static_cast<uint8_t>(db17 & kMhiHorizontalSwingDb17Mask);
  out.horizontal_raw = static_cast<uint8_t>(out.horizontal_db16_bits | ((out.horizontal_db17_bit & 0x01U) << 4U));
  out.horizontal_swing = (out.horizontal_raw & 0x10U) != 0U;
  if (out.horizontal_swing) {
    out.horizontal_valid = true;
    out.horizontal_vane = 8U;
  } else {
    const uint8_t position = static_cast<uint8_t>((out.horizontal_raw & 0x07U) + 1U);
    out.horizontal_valid = position <= 7U;
    out.horizontal_vane = out.horizontal_valid ? position : 0U;
  }

  out.three_d_auto_bit = static_cast<uint8_t>(db17 & kMhiThreeDAutoDb17Mask);
  out.three_d_auto = out.three_d_auto_bit != 0U;
  out.db17_command_bits = static_cast<uint8_t>(db17 & kMhiLouverCommandDb17Mask);
  return out;
}

constexpr bool mhi_louver_diag_relevant_equal(const MhiLouverDiagSnapshot& lhs, const MhiLouverDiagSnapshot& rhs) {
  return lhs.vertical_db0_bits == rhs.vertical_db0_bits && lhs.vertical_db1_bits == rhs.vertical_db1_bits &&
         lhs.db16 == rhs.db16 && lhs.db17 == rhs.db17;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
