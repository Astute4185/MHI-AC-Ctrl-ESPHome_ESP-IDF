#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_frame.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiRxFrameQueue {
 public:
  static constexpr std::size_t kCapacity = 4U;

  bool push(const uint8_t* data, std::size_t len) {
    if (data == nullptr || len == 0U || len > kMhiMaxFrameBytes || full()) {
      return false;
    }

    auto& frame = frames_[tail_];
    frame.clear();
    frame.len = len;

    for (std::size_t i = 0U; i < len; i++) {
      frame.data[i] = data[i];
    }

    tail_ = next_index_(tail_);
    count_++;

    return true;
  }

  bool pop(MhiFrameBuffer& out) {
    if (empty()) {
      return false;
    }

    out = frames_[head_];
    frames_[head_].clear();
    head_ = next_index_(head_);
    count_--;

    return true;
  }

  void clear() {
    for (auto& frame : frames_) {
      frame.clear();
    }

    head_ = 0U;
    tail_ = 0U;
    count_ = 0U;
  }

  bool empty() const {
    return count_ == 0U;
  }

  bool full() const {
    return count_ >= kCapacity;
  }

  std::size_t size() const {
    return count_;
  }

  std::size_t capacity() const {
    return kCapacity;
  }

 private:
  static std::size_t next_index_(std::size_t index) {
    return (index + 1U) % kCapacity;
  }

  MhiFrameBuffer frames_[kCapacity]{};
  std::size_t head_{0U};
  std::size_t tail_{0U};
  std::size_t count_{0U};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
