#pragma once

#include <cstdint>

#include "mhi_frame.h"

namespace esphome {
namespace mhi_ac_ctrl {

void mhi_protocol_capture_frame(const MhiFrameBuffer& frame, uint8_t kind_id, const char* kind_name,
                                uint16_t opdata_key, uint32_t sequence);
void mhi_protocol_capture_reset();

}  // namespace mhi_ac_ctrl
}  // namespace esphome
