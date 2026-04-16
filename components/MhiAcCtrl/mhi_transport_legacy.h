#pragma once

#include "mhi_transport.h"

namespace esphome {
namespace mhi {

class MhiTransportLegacy : public MhiTransport {
 public:
  void setup(const MhiTransportConfig &config) override;

  MhiFrameExchangeResult exchange_frame(
      const uint8_t *tx_frame,
      uint8_t *rx_frame,
      std::size_t rx_capacity,
      uint32_t max_time_ms) override;

 private:
  std::size_t determine_target_frame_bytes_(std::size_t rx_capacity) const;
  bool wait_for_frame_idle_(uint32_t start_ms, uint32_t max_time_ms) const;
  bool read_byte_(uint8_t tx_byte, uint8_t &rx_byte, uint32_t start_ms, uint32_t max_time_ms) const;
  bool wait_for_extension_start_(uint32_t start_ms, uint32_t max_time_ms) const;

  MhiTransportConfig config_{};
};

}  // namespace mhi
}  // namespace esphome
