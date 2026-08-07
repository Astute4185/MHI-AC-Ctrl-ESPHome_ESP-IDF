#pragma once

#include <cstdint>

namespace esphome {
inline uint32_t test_millis_value = 0U;
inline uint32_t test_micros_value = 0U;

inline uint32_t millis() {
  return test_millis_value;
}

inline uint32_t micros() {
  return test_micros_value;
}
}  // namespace esphome
