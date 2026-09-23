#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_frame.h"

namespace esphome {
namespace mhi_ac_ctrl {

static constexpr std::size_t kMhiCaptureSlotCount = 48U;
static constexpr std::size_t kMhiCaptureMaxReportedChanges = 12U;

enum class MhiCaptureDisposition : uint8_t {
  IGNORED = 0,
  BASELINE,
  UNCHANGED,
  CHANGED,
};

struct MhiCaptureChange {
  uint8_t raw_index{0U};
  uint8_t before{0U};
  uint8_t after{0U};
  uint8_t xor_mask{0U};
};

struct MhiCaptureObservation {
  MhiCaptureDisposition disposition{MhiCaptureDisposition::IGNORED};
  uint8_t frame_len{0U};
  uint8_t total_changes{0U};
  uint8_t reported_changes{0U};
  MhiCaptureChange changes[kMhiCaptureMaxReportedChanges]{};

  bool truncated() const { return total_changes > reported_changes; }
};

class MhiProtocolCaptureEngine {
 public:
  MhiCaptureObservation observe(const MhiFrameBuffer& frame, uint8_t kind_id, uint16_t opdata_key,
                                uint32_t sequence);
  void reset();

 private:
  struct Slot {
    bool used{false};
    uint8_t kind_id{0U};
    uint16_t opdata_key{0U};
    uint8_t len{0U};
    uint8_t bytes[kMhiMaxFrameBytes]{};
    uint32_t last_sequence{0U};
  };

  Slot* find_or_allocate_slot_(uint8_t kind_id, uint16_t opdata_key);
  static void copy_frame_(Slot& slot, const MhiFrameBuffer& frame, uint32_t sequence);

  Slot slots_[kMhiCaptureSlotCount]{};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
