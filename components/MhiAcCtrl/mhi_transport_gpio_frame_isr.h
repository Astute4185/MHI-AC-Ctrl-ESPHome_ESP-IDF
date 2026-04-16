#pragma once

#include "mhi_transport.h"
#include "mhi_transport_legacy.h"

namespace esphome {
namespace mhi {

class MhiTransportGpioFrameIsr : public MhiTransport {
 public:
  void setup(const MhiTransportConfig &config) override;

  MhiFrameExchangeResult exchange_frame(
      const uint8_t *tx_frame,
      std::size_t tx_frame_size,
      uint8_t *rx_frame,
      std::size_t rx_frame_capacity,
      uint32_t max_time_ms) override;

 private:
  MhiTransportLegacy delegate_{};
};

}  // namespace mhi
}  // namespace esphome
