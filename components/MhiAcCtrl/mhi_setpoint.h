#pragma once

#include <cmath>

namespace esphome {
namespace mhi_ac_ctrl {

// MHI DB2 represents half-degree increments. Restrict commands to the
// configured UI resolution (1.0°C by default, optionally 0.5°C).
inline float mhi_quantize_setpoint(float requested_c, float step_c) {
  return std::round(requested_c / step_c) * step_c;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
