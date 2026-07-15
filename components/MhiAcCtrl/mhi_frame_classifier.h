#pragma once

#include <cstdint>

#include "mhi_frame.h"

namespace esphome {
namespace mhi_ac_ctrl {

enum class MhiFrameKind : uint8_t {
  UNKNOWN = 0,
  STATUS,
  EXTENDED_STATUS,
  OPDATA,
};

constexpr uint16_t kMhiInvalidOpDataKey = 0xFFFFU;

struct MhiFrameClassification {
  MhiFrameKind kind{MhiFrameKind::UNKNOWN};
  uint16_t opdata_key{kMhiInvalidOpDataKey};
};

MhiFrameClassification classify_mhi_mosi_frame(const MhiFrameView& frame);
const char* mhi_frame_kind_to_string(MhiFrameKind kind);

}  // namespace mhi_ac_ctrl
}  // namespace esphome
