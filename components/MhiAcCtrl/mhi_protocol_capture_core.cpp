#include "mhi_protocol_capture_core.h"

#include <cstring>

namespace esphome {
namespace mhi_ac_ctrl {

void MhiProtocolCaptureEngine::reset() {
  for (auto& slot : this->slots_) {
    slot = Slot{};
  }
}

MhiProtocolCaptureEngine::Slot* MhiProtocolCaptureEngine::find_or_allocate_slot_(uint8_t kind_id,
                                                                                 uint16_t opdata_key) {
  Slot* first_free = nullptr;
  Slot* oldest = nullptr;

  for (auto& slot : this->slots_) {
    if (slot.used && slot.kind_id == kind_id && slot.opdata_key == opdata_key) {
      return &slot;
    }
    if (!slot.used && first_free == nullptr) {
      first_free = &slot;
    }
    if (slot.used && (oldest == nullptr || slot.last_sequence < oldest->last_sequence)) {
      oldest = &slot;
    }
  }

  Slot* selected = first_free != nullptr ? first_free : oldest;
  if (selected == nullptr) {
    return nullptr;
  }

  *selected = Slot{};
  selected->used = true;
  selected->kind_id = kind_id;
  selected->opdata_key = opdata_key;
  return selected;
}

void MhiProtocolCaptureEngine::copy_frame_(Slot& slot, const MhiFrameBuffer& frame, uint32_t sequence) {
  slot.len = static_cast<uint8_t>(frame.len);
  std::memcpy(slot.bytes, frame.bytes(), frame.len);
  if (frame.len < kMhiMaxFrameBytes) {
    std::memset(slot.bytes + frame.len, 0, kMhiMaxFrameBytes - frame.len);
  }
  slot.last_sequence = sequence;
}

MhiCaptureObservation MhiProtocolCaptureEngine::observe(const MhiFrameBuffer& frame, uint8_t kind_id,
                                                         uint16_t opdata_key, uint32_t sequence) {
  MhiCaptureObservation observation{};
  if (!frame.view().valid()) {
    return observation;
  }

  observation.frame_len = static_cast<uint8_t>(frame.len);
  Slot* slot = this->find_or_allocate_slot_(kind_id, opdata_key);
  if (slot == nullptr) {
    return observation;
  }

  if (slot->len == 0U || slot->len != frame.len) {
    copy_frame_(*slot, frame, sequence);
    observation.disposition = MhiCaptureDisposition::BASELINE;
    return observation;
  }

  for (std::size_t i = 0U; i < frame.len; i++) {
    const bool checksum_byte = i == CBH || i == CBL || (frame.len == kMhiFrame33Bytes && i == CBL2);
    if (checksum_byte) {
      continue;
    }

    const uint8_t before = slot->bytes[i];
    const uint8_t after = frame.data[i];
    if (before == after) {
      continue;
    }

    observation.total_changes++;
    if (observation.reported_changes < kMhiCaptureMaxReportedChanges) {
      MhiCaptureChange& change = observation.changes[observation.reported_changes++];
      change.raw_index = static_cast<uint8_t>(i);
      change.before = before;
      change.after = after;
      change.xor_mask = static_cast<uint8_t>(before ^ after);
    }
  }

  slot->last_sequence = sequence;
  if (observation.total_changes == 0U) {
    observation.disposition = MhiCaptureDisposition::UNCHANGED;
    return observation;
  }

  observation.disposition = MhiCaptureDisposition::CHANGED;
  copy_frame_(*slot, frame, sequence);
  return observation;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
