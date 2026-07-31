#pragma once

#include <cstdint>

namespace esphome {
namespace mhi_ac_ctrl {

enum class MhiRmtCsSpiTarget : uint8_t {
  UNSUPPORTED = 0,
  ESP32,
  ESP32_S3,
};

constexpr bool mhi_rmt_cs_spi_target_supported(MhiRmtCsSpiTarget target) {
  return target == MhiRmtCsSpiTarget::ESP32 || target == MhiRmtCsSpiTarget::ESP32_S3;
}

// The original ESP32 GP-SPI slave peripheral needs a mode-3 receive-edge
// override when CPU FIFO transfers are used. ESP32-S3 retains the ESP-IDF
// mode configuration.
constexpr bool mhi_rmt_cs_spi_needs_mode3_edge_fix(MhiRmtCsSpiTarget target) {
  return target == MhiRmtCsSpiTarget::ESP32;
}

constexpr const char* mhi_rmt_cs_spi_driver_name() {
  return "rmt_cs_spi";
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
