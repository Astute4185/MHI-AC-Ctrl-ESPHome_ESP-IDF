#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "mhi_defs.h"

namespace esphome {
namespace mhi_ac_ctrl {

// Latest-value, one-shot TX mailbox used by duplex transports.
//
// Thread/ISR safety is deliberately external: callers must protect stage(),
// take(), and clear() with the transport's lock. Keeping the state container
// platform-neutral makes replacement and ownership semantics unit-testable.
class MhiDuplexTxMailbox {
 public:
  bool stage(const uint8_t* data, std::size_t len) {
    if (data == nullptr || (len != kMhiFrame20Bytes && len != kMhiFrame33Bytes)) {
      return false;
    }

    if (pending_) {
      overwritten_frames_++;
    }

    frame_.fill(0U);
    std::memcpy(frame_.data(), data, std::min<std::size_t>(len, frame_.size()));
    len_ = len;
    pending_ = true;
    generation_++;
    return true;
  }

  bool take(uint8_t* dst, std::size_t capacity, std::size_t& len, uint32_t& generation) {
    len = 0U;
    generation = 0U;

    if (!pending_ || dst == nullptr || capacity < len_) {
      return false;
    }

    std::memcpy(dst, frame_.data(), len_);
    len = len_;
    generation = generation_;
    pending_ = false;
    len_ = 0U;
    return true;
  }

  void clear() {
    frame_.fill(0U);
    len_ = 0U;
    pending_ = false;
    generation_ = 0U;
    overwritten_frames_ = 0U;
  }

  bool pending() const {
    return pending_;
  }

  std::size_t pending_len() const {
    return len_;
  }

  uint32_t generation() const {
    return generation_;
  }

  uint32_t overwritten_frames() const {
    return overwritten_frames_;
  }

 private:
  std::array<uint8_t, kMhiMaxFrameBytes> frame_{};
  std::size_t len_{0U};
  bool pending_{false};
  uint32_t generation_{0U};
  uint32_t overwritten_frames_{0U};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
