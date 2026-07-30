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

// The original ESP32 GP-SPI slave peripheral needs a mode-3 receive-edge
// override when the CPU FIFO path is used. ESP32-S3 and DMA-backed transports
// retain the ESP-IDF-provided mode configuration.
constexpr bool mhi_rmt_cs_spi_needs_fifo_mode3_edge_fix(MhiRmtCsSpiTarget target, MhiRmtCsSpiBufferMode mode) {
  return target == MhiRmtCsSpiTarget::ESP32 && mode == MhiRmtCsSpiBufferMode::FIFO;
}

constexpr const char* mhi_rmt_cs_spi_driver_name(MhiRmtCsSpiBufferMode mode) {
  return mode == MhiRmtCsSpiBufferMode::DMA ? "rmt_cs_spi" : "rmt_cs_spi_nodma";
}

constexpr const char* mhi_rmt_cs_spi_buffer_mode_name(MhiRmtCsSpiBufferMode mode) {
  return mode == MhiRmtCsSpiBufferMode::DMA ? "DMA" : "FIFO";
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
