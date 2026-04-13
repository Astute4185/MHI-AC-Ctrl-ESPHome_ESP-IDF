#include "mhi_transport_spi_idf.h"
#include "MHI-AC-Ctrl-core.h"

namespace esphome {
namespace mhi {

void MhiTransportSpiIdf::setup(const MhiTransportConfig &config) {
  this->config_ = config;
}

int MhiTransportSpiIdf::exchange_frame(
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t frame_size,
    uint32_t max_time_ms,
    bool &new_data_packet_received) {

  (void) tx_frame;
  (void) rx_frame;
  (void) frame_size;
  (void) max_time_ms;

  new_data_packet_received = false;

  return err_msg_timeout_SCK_low;  // placeholder
}

}  // namespace mhi
}  // namespace esphome