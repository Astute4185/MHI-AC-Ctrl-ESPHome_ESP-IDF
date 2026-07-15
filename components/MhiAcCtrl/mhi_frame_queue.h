#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "mhi_defs.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiCapturedFrame {
  std::array<uint8_t, kMhiMaxFrameBytes> data{};
  std::size_t len{0U};
  uint32_t sequence{0U};
  uint32_t frame_end_us{0U};
};

template <std::size_t Capacity>
class MhiFrameQueue {
  static_assert(Capacity > 0U, "MHI frame queue capacity must be greater than zero");

 public:
  void clear() {
    frames_ = {};
    head_ = 0U;
    tail_ = 0U;
    size_ = 0U;
    high_water_mark_ = 0U;
    overwritten_frames_ = 0U;
  }

  bool push(const uint8_t* data, std::size_t len, uint32_t sequence, uint32_t frame_end_us) {
    if (data == nullptr || len == 0U || len > kMhiMaxFrameBytes) {
      return false;
    }

    if (size_ == Capacity) {
      tail_ = increment_(tail_);
      size_--;
      overwritten_frames_++;
    }

    MhiCapturedFrame& frame = frames_[head_];
    frame = {};
    std::memcpy(frame.data.data(), data, len);
    frame.len = len;
    frame.sequence = sequence;
    frame.frame_end_us = frame_end_us;

    head_ = increment_(head_);
    size_++;
    if (size_ > high_water_mark_) {
      high_water_mark_ = size_;
    }
    return true;
  }

  bool pop(MhiCapturedFrame& out) {
    if (size_ == 0U) {
      return false;
    }

    out = frames_[tail_];
    frames_[tail_] = {};
    tail_ = increment_(tail_);
    size_--;
    return true;
  }

  std::size_t size() const {
    return size_;
  }

  constexpr std::size_t capacity() const {
    return Capacity;
  }

  std::size_t high_water_mark() const {
    return high_water_mark_;
  }

  uint32_t overwritten_frames() const {
    return overwritten_frames_;
  }

 private:
  static constexpr std::size_t increment_(std::size_t value) {
    return (value + 1U) % Capacity;
  }

  std::array<MhiCapturedFrame, Capacity> frames_{};
  std::size_t head_{0U};
  std::size_t tail_{0U};
  std::size_t size_{0U};
  std::size_t high_water_mark_{0U};
  uint32_t overwritten_frames_{0U};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
