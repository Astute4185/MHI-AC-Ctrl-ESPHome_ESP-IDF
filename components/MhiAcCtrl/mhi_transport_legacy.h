#pragma once

#include "mhi_transport.h"

namespace esphome {
namespace mhi {

class MhiTransportLegacy : public MhiTransport {
 public:
  void setup(const MhiTransportConfig &config) override;

  bool exchange_frame(
      const uint8_t *tx_frame,
      uint8_t *rx_frame,
      std::size_t frame_size,
      uint32_t max_time_ms) override;

 private:
  MhiTransportConfig config_;
};

}  // namespace mhi
}  // namespace esphome
