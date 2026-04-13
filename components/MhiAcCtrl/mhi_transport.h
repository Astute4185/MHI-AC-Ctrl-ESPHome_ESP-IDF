#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace mhi {

struct MhiTransportConfig {
  int sck_pin = -1;
  int mosi_pin = -1;
  int miso_pin = -1;
  uint8_t frame_size = 20;
};

class MhiTransport {
 public:
  virtual ~MhiTransport() = default;

  virtual void setup(const MhiTransportConfig &config) = 0;

  virtual bool exchange_frame(
      const uint8_t *tx_frame,
      uint8_t *rx_frame,
      std::size_t frame_size,
      uint32_t max_time_ms) = 0;
};

}  // namespace mhi
}  // namespace esphome
