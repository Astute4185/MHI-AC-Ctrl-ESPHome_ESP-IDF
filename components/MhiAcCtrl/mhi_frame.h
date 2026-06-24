#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_defs.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiFrameView {
  const uint8_t* data{nullptr};
  std::size_t len{0};

  bool valid() const {
    return data != nullptr && (len == kMhiFrame20Bytes || len == kMhiFrame33Bytes);
  }

  bool is_20_byte() const {
    return len == kMhiFrame20Bytes;
  }
  bool is_33_byte() const {
    return len == kMhiFrame33Bytes;
  }

  uint8_t operator[](std::size_t index) const {
    return data[index];
  }
};

struct MhiFrameBuffer {
  uint8_t data[kMhiMaxFrameBytes]{};
  std::size_t len{0};

  uint8_t* bytes() {
    return data;
  }
  const uint8_t* bytes() const {
    return data;
  }

  void clear() {
    for (std::size_t i = 0; i < kMhiMaxFrameBytes; i++) {
      data[i] = 0;
    }
    len = 0;
  }

  MhiFrameView view() const {
    return MhiFrameView{data, len};
  }
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome