#pragma once

#include <cstdint>

namespace esphome {
namespace mhi_ac_ctrl {

enum class MhiRmtCsSpiTarget : uint8_t {
  UNSUPPORTED = 0,
  ESP32,
  ESP32_S3,
};

enum class MhiRmtCsSpiBufferMode : uint8_t {
  DMA = 0,
  FIFO,
};

constexpr bool mhi_rmt_cs_spi_uses_dma(MhiRmtCsSpiBufferMode mode) {
  return mode == MhiRmtCsSpiBufferMode::DMA;
}

constexpr bool mhi_rmt_cs_spi_mode_supported(MhiRmtCsSpiTarget target, MhiRmtCsSpiBufferMode mode) {
  if (mode == MhiRmtCsSpiBufferMode::DMA) {
    return target == MhiRmtCsSpiTarget::ESP32_S3;
  }

  return target == MhiRmtCsSpiTarget::ESP32 || target == MhiRmtCsSpiTarget::ESP32_S3;
}

constexpr const char* mhi_rmt_cs_spi_driver_name(MhiRmtCsSpiBufferMode mode) {
  return mode == MhiRmtCsSpiBufferMode::DMA ? "rmt_cs_spi" : "rmt_cs_spi_nodma";
}

constexpr const char* mhi_rmt_cs_spi_buffer_mode_name(MhiRmtCsSpiBufferMode mode) {
  return mode == MhiRmtCsSpiBufferMode::DMA ? "DMA" : "FIFO";
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
