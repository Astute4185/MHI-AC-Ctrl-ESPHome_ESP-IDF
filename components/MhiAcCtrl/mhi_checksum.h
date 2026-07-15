#pragma once

#include <cstdint>

namespace esphome {
namespace mhi_ac_ctrl {

uint16_t mhi_calc_checksum(const uint8_t* frame);
uint16_t mhi_calc_checksum_frame33(const uint8_t* frame);

bool mhi_checksum_valid_20(const uint8_t* frame);
bool mhi_checksum_valid_33(const uint8_t* frame);

}  // namespace mhi_ac_ctrl
}  // namespace esphome