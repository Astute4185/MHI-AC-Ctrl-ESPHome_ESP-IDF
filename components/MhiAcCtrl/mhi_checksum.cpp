#include "mhi_checksum.h"

#include "mhi_defs.h"

namespace esphome {
namespace mhi_ac_ctrl {

uint16_t mhi_calc_checksum(const uint8_t* frame) {
  uint16_t checksum = 0;

  for (std::size_t i = 0; i < CBH; i++) {
    checksum += frame[i];
  }

  return checksum;
}

uint16_t mhi_calc_checksum_frame33(const uint8_t* frame) {
  uint16_t checksum = 0;

  for (std::size_t i = 0; i < CBL2; i++) {
    checksum += frame[i];
  }

  return checksum;
}

bool mhi_checksum_valid_20(const uint8_t* frame) {
  const uint16_t expected = mhi_calc_checksum(frame);
  const uint16_t actual = static_cast<uint16_t>((static_cast<uint16_t>(frame[CBH]) << 8U) | frame[CBL]);

  return expected == actual;
}

bool mhi_checksum_valid_33(const uint8_t* frame) {
  const uint16_t expected = mhi_calc_checksum_frame33(frame);

  // Current old layout only exposes CBL2 for the extended frame checksum byte.
  // Keep this validator conservative until the 33-byte checksum structure is confirmed.
  return static_cast<uint8_t>(expected & 0xFFU) == frame[CBL2];
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome