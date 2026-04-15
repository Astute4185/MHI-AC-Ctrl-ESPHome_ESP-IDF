#pragma once

#include <cstdint>

#include "esp_timer.h"

namespace esphome {
namespace mhi {

inline uint32_t mhi_now_ms() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

inline uint32_t mhi_now_us() {
  return static_cast<uint32_t>(esp_timer_get_time());
}

}  // namespace mhi
}  // namespace esphome