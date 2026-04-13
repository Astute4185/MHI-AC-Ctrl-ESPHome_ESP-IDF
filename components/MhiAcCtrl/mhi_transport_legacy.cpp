#include "mhi_transport_legacy.h"

#include "MHI-AC-Ctrl-core.h"

namespace esphome {
namespace mhi {

void MhiTransportLegacy::setup(const MhiTransportConfig &config) {
  this->config_ = config;

  pinMode(this->config_.sck_pin, INPUT);
  pinMode(this->config_.mosi_pin, INPUT);
  pinMode(this->config_.miso_pin, OUTPUT);
}

int MhiTransportLegacy::exchange_frame(
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t frame_size,
    uint32_t max_time_ms,
    bool &new_data_packet_received) {

  new_data_packet_received = false;

  uint32_t start_millis = millis();
  uint32_t sck_millis = millis();

  // wait for 5ms stable high signal to detect a frame start
  while (millis() - sck_millis < 5) {
    if (!digitalRead(this->config_.sck_pin)) {
      sck_millis = millis();
    }
    if (millis() - start_millis > max_time_ms) {
      return err_msg_timeout_SCK_low;
    }
  }

  for (std::size_t byte_cnt = 0; byte_cnt < frame_size; byte_cnt++) {
    uint8_t mosi_byte = 0;
    uint8_t bit_mask = 1;

    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      while (digitalRead(this->config_.sck_pin)) {
        if (millis() - start_millis > max_time_ms) {
          return err_msg_timeout_SCK_high;
        }
      }

      if ((tx_frame[byte_cnt] & bit_mask) > 0) {
        digitalWrite(this->config_.miso_pin, 1);
      } else {
        digitalWrite(this->config_.miso_pin, 0);
      }

      while (!digitalRead(this->config_.sck_pin)) {
      }

      if (digitalRead(this->config_.mosi_pin)) {
        mosi_byte += bit_mask;
      }

      bit_mask = static_cast<uint8_t>(bit_mask << 1);
    }

    if (rx_frame[byte_cnt] != mosi_byte) {
      new_data_packet_received = true;
      rx_frame[byte_cnt] = mosi_byte;
    }
  }

  digitalWrite(this->config_.miso_pin, 0);

  return 0;
}

}  // namespace mhi
}  // namespace esphome
