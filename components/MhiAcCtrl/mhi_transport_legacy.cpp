#include "mhi_transport_legacy.h"

#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>

#include "MHI-AC-Ctrl-core.h"
#include "mhi_time.h"

namespace esphome {
namespace mhi {

namespace {

constexpr uint32_t kTimeoutCheckMask = 0x3F;  // check time every 64 spins

inline bool timed_out(uint32_t start_ms, uint32_t max_time_ms, uint32_t &spin_counter) {
  spin_counter++;
  if ((spin_counter & kTimeoutCheckMask) != 0) {
    return false;
  }
  return (mhi_now_ms() - start_ms) > max_time_ms;
}

}  // namespace

void MhiTransportLegacy::setup(const MhiTransportConfig &config) {
  this->config_ = config;

  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.sck_pin));
  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.mosi_pin));
  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.miso_pin));

  gpio_set_direction(static_cast<gpio_num_t>(this->config_.sck_pin), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(this->config_.mosi_pin), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(this->config_.miso_pin), GPIO_MODE_OUTPUT);

  gpio_set_level(static_cast<gpio_num_t>(this->config_.miso_pin), 0);
}

int MhiTransportLegacy::exchange_frame(
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t frame_size,
    uint32_t max_time_ms,
    bool &new_data_packet_received) {
  new_data_packet_received = false;

  const gpio_num_t sck = static_cast<gpio_num_t>(this->config_.sck_pin);
  const gpio_num_t mosi = static_cast<gpio_num_t>(this->config_.mosi_pin);
  const gpio_num_t miso = static_cast<gpio_num_t>(this->config_.miso_pin);

  const uint32_t start_ms = mhi_now_ms();

  // Wait for 5 ms stable high on SCK to detect frame start.
  uint32_t sck_high_start_ms = mhi_now_ms();
  uint32_t wait_spin_counter = 0;
  while ((mhi_now_ms() - sck_high_start_ms) < 5U) {
    if (gpio_get_level(sck) == 0) {
      sck_high_start_ms = mhi_now_ms();
    }
    if (timed_out(start_ms, max_time_ms, wait_spin_counter)) {
      return err_msg_timeout_SCK_low;
    }
  }

  for (std::size_t byte_cnt = 0; byte_cnt < frame_size; byte_cnt++) {
    uint8_t mosi_byte = 0;
    uint8_t bit_mask = 1;

    uint32_t byte_spin_counter = 0;

    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      // Wait for falling edge: SCK high -> low
      while (gpio_get_level(sck) != 0) {
        if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
          return err_msg_timeout_SCK_high;
        }
      }

      // Drive outgoing bit on falling edge
      gpio_set_level(miso, (tx_frame[byte_cnt] & bit_mask) ? 1 : 0);

      // Wait for rising edge: SCK low -> high
      while (gpio_get_level(sck) == 0) {
        if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
          return err_msg_timeout_SCK_low;
        }
      }

      // Sample MOSI right after rising edge
      if (gpio_get_level(mosi) != 0) {
        mosi_byte = static_cast<uint8_t>(mosi_byte + bit_mask);
      }

      bit_mask = static_cast<uint8_t>(bit_mask << 1);
    }

    if (rx_frame[byte_cnt] != mosi_byte) {
      new_data_packet_received = true;
      rx_frame[byte_cnt] = mosi_byte;
    }

    // Coarse timeout check between bytes as well.
    if ((mhi_now_ms() - start_ms) > max_time_ms) {
      return err_msg_timeout_SCK_high;
    }
  }

  gpio_set_level(miso, 0);
  return 0;
}

}  // namespace mhi
}  // namespace esphome