#include "mhi_transport_gpio_frame_isr.h"

namespace esphome {
namespace mhi {

void MhiTransportGpioFrameIsr::setup(const MhiTransportConfig &config) {
  this->delegate_.setup(config);
}

MhiFrameExchangeResult MhiTransportGpioFrameIsr::exchange_frame(
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t rx_capacity,
    uint32_t max_time_ms) {
  return this->delegate_.exchange_frame(tx_frame, rx_frame, rx_capacity, max_time_ms);
}

}  // namespace mhi
}  // namespace esphome