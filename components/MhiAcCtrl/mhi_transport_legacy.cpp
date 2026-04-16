#include "mhi_transport_legacy.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_timer.h>

#include "MHI-AC-Ctrl-core.h"
#include "mhi_time.h"

#if defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) ||     defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) ||     defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32H2)
#include "soc/gpio_struct.h"
#define MHI_USE_FAST_GPIO 1
#else
#define MHI_USE_FAST_GPIO 0
#endif

namespace esphome {
namespace mhi {

namespace {

constexpr uint32_t kTimeoutCheckMask = 0x3F;
constexpr std::size_t kStandardFrameBytes = 20;
constexpr std::size_t kExtendedFrameBytes = 33;

#if MHI_USE_FAST_GPIO
inline bool IRAM_ATTR fast_gpio_read(int pin) {
  if (pin < 32) {
    return (GPIO.in >> pin) & 0x1;
  }
  return (GPIO.in1.val >> (pin - 32)) & 0x1;
}

inline void IRAM_ATTR fast_gpio_write_high(int pin) {
  if (pin < 32) {
    GPIO.out_w1ts = (1UL << pin);
  } else {
    GPIO.out1_w1ts.val = (1UL << (pin - 32));
  }
}

inline void IRAM_ATTR fast_gpio_write_low(int pin) {
  if (pin < 32) {
    GPIO.out_w1tc = (1UL << pin);
  } else {
    GPIO.out1_w1tc.val = (1UL << (pin - 32));
  }
}
#else
inline bool IRAM_ATTR fast_gpio_read(int pin) {
  return gpio_get_level(static_cast<gpio_num_t>(pin)) != 0;
}

inline void IRAM_ATTR fast_gpio_write_high(int pin) {
  gpio_set_level(static_cast<gpio_num_t>(pin), 1);
}

inline void IRAM_ATTR fast_gpio_write_low(int pin) {
  gpio_set_level(static_cast<gpio_num_t>(pin), 0);
}
#endif

inline bool IRAM_ATTR timed_out(uint32_t start_ms, uint32_t max_time_ms, uint32_t &spin_counter) {
  spin_counter++;
  if ((spin_counter & kTimeoutCheckMask) != 0) {
    return false;
  }
  return (mhi_now_ms() - start_ms) > max_time_ms;
}

int IRAM_ATTR read_one_byte(
    int sck_pin,
    int mosi_pin,
    int miso_pin,
    uint8_t tx_byte,
    uint8_t &rx_byte,
    uint32_t start_ms,
    uint32_t max_time_ms,
    bool first_falling_edge_already_seen) {
  uint8_t mosi_byte = 0;
  uint8_t bit_mask = 1;
  uint32_t spin_counter = 0;

  for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
    if (!(first_falling_edge_already_seen && bit_cnt == 0)) {
      while (fast_gpio_read(sck_pin)) {
        if (timed_out(start_ms, max_time_ms, spin_counter)) {
          return err_msg_timeout_SCK_high;
        }
      }
    }

    if ((tx_byte & bit_mask) != 0) {
      fast_gpio_write_high(miso_pin);
    } else {
      fast_gpio_write_low(miso_pin);
    }

    while (!fast_gpio_read(sck_pin)) {
      if (timed_out(start_ms, max_time_ms, spin_counter)) {
        return err_msg_timeout_SCK_low;
      }
    }

    if (fast_gpio_read(mosi_pin)) {
      mosi_byte = static_cast<uint8_t>(mosi_byte + bit_mask);
    }

    bit_mask = static_cast<uint8_t>(bit_mask << 1);
  }

  rx_byte = mosi_byte;
  return 0;
}

bool wait_for_next_falling_edge_us(int sck_pin, uint32_t timeout_us) {
  const int64_t start_us = esp_timer_get_time();
  while (fast_gpio_read(sck_pin)) {
    if ((esp_timer_get_time() - start_us) > static_cast<int64_t>(timeout_us)) {
      return false;
    }
  }
  return true;
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

MhiFrameExchangeResult IRAM_ATTR MhiTransportLegacy::exchange_frame(
    const uint8_t *tx_frame,
    std::size_t tx_frame_size,
    uint8_t *rx_frame,
    std::size_t rx_frame_capacity,
    uint32_t max_time_ms) {
  MhiFrameExchangeResult result{};

  const int sck_pin = this->config_.sck_pin;
  const int mosi_pin = this->config_.mosi_pin;
  const int miso_pin = this->config_.miso_pin;

  const std::size_t tx_limit = std::min(tx_frame_size, kExtendedFrameBytes);
  const std::size_t rx_limit = std::min(rx_frame_capacity, kExtendedFrameBytes);
  const std::size_t base_limit = std::min(rx_limit, kStandardFrameBytes);

  const uint32_t start_ms = mhi_now_ms();

  uint32_t sck_high_start_ms = mhi_now_ms();
  uint32_t wait_spin_counter = 0;
  while ((mhi_now_ms() - sck_high_start_ms) < this->config_.frame_idle_min_ms) {
    if (!fast_gpio_read(sck_pin)) {
      sck_high_start_ms = mhi_now_ms();
    }
    if (timed_out(start_ms, max_time_ms, wait_spin_counter)) {
      result.status = err_msg_timeout_SCK_low;
      return result;
    }
  }

  for (std::size_t byte_cnt = 0; byte_cnt < base_limit; byte_cnt++) {
    uint8_t rx_byte = 0;
    const uint8_t tx_byte = byte_cnt < tx_limit ? tx_frame[byte_cnt] : 0;
    const int rc = read_one_byte(sck_pin, mosi_pin, miso_pin, tx_byte, rx_byte, start_ms, max_time_ms, false);
    if (rc < 0) {
      result.status = rc;
      return result;
    }
    if (rx_frame[byte_cnt] != rx_byte) {
      result.new_data_packet_received = true;
      rx_frame[byte_cnt] = rx_byte;
    }
    result.bytes_received = byte_cnt + 1;
  }

  result.base_frame_complete = (result.bytes_received >= kStandardFrameBytes);
  if (!result.base_frame_complete) {
    result.status = err_msg_invalid_checksum;
    return result;
  }

  const uint8_t header = rx_frame[SB0];
  result.header_valid = ((header == 0x6C || header == 0x6D) && rx_frame[SB1] == 0x80 && rx_frame[SB2] == 0x04);

  bool attempt_extension = false;
  switch (this->config_.protocol_mode) {
    case MhiProtocolMode::STANDARD_ONLY:
      attempt_extension = false;
      break;
    case MhiProtocolMode::EXTENDED_PREFER:
      attempt_extension = true;
      break;
    case MhiProtocolMode::AUTO:
    default:
      attempt_extension = (header == 0x6D) || (this->config_.frame_size_hint >= kExtendedFrameBytes);
      break;
  }

  if (attempt_extension && rx_limit >= kExtendedFrameBytes) {
    if (wait_for_next_falling_edge_us(sck_pin, this->config_.extension_gap_timeout_us)) {
      result.extended_tail_present = true;
      for (std::size_t byte_cnt = kStandardFrameBytes; byte_cnt < kExtendedFrameBytes; byte_cnt++) {
        uint8_t rx_byte = 0;
        const uint8_t tx_byte = byte_cnt < tx_limit ? tx_frame[byte_cnt] : 0;
        const bool first_edge_seen = (byte_cnt == kStandardFrameBytes);
        const int rc = read_one_byte(sck_pin, mosi_pin, miso_pin, tx_byte, rx_byte, start_ms, max_time_ms, first_edge_seen);
        if (rc < 0) {
          result.status = rc;
          return result;
        }
        if (rx_frame[byte_cnt] != rx_byte) {
          result.new_data_packet_received = true;
          rx_frame[byte_cnt] = rx_byte;
        }
        result.bytes_received = byte_cnt + 1;
      }
    }
  }

  fast_gpio_write_low(miso_pin);

  if (result.bytes_received >= kExtendedFrameBytes && header == 0x6D) {
    result.detected_type = MhiFrameType::EXTENDED_33;
  } else if (result.bytes_received >= kStandardFrameBytes) {
    result.detected_type = MhiFrameType::STANDARD_20;
  } else {
    result.detected_type = MhiFrameType::UNKNOWN;
  }

  result.status = 0;
  return result;
}

}  // namespace mhi
}  // namespace esphome
