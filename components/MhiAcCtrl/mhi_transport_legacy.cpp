#include "mhi_transport_legacy.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

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
constexpr std::size_t kBaseFrameBytes = 20U;
constexpr std::size_t kExtendedFrameBytes = 33U;

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

inline uint64_t now_us() {
  return static_cast<uint64_t>(esp_timer_get_time());
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
    uint8_t *rx_frame,
    std::size_t rx_capacity,
    uint32_t max_time_ms) {
  MhiFrameExchangeResult result{};

  const std::size_t target_frame_size =
      std::min<std::size_t>(this->config_.frame_size_hint == 33 ? kExtendedFrameBytes : kBaseFrameBytes, rx_capacity);

  if (target_frame_size < kBaseFrameBytes) {
    result.status = err_msg_timeout_SCK_low;
    return result;
  }

  const int sck_pin = this->config_.sck_pin;
  const int mosi_pin = this->config_.mosi_pin;
  const int miso_pin = this->config_.miso_pin;

  const uint32_t start_ms = mhi_now_ms();

  uint32_t sck_high_start_ms = mhi_now_ms();
  uint32_t wait_spin_counter = 0;
  while ((mhi_now_ms() - sck_high_start_ms) < this->config_.frame_start_idle_ms) {
    if (!fast_gpio_read(sck_pin)) {
      sck_high_start_ms = mhi_now_ms();
    }
    if (timed_out(start_ms, max_time_ms, wait_spin_counter)) {
      result.status = err_msg_timeout_SCK_low;
      return result;
    }
  }

  for (std::size_t byte_cnt = 0; byte_cnt < kBaseFrameBytes; byte_cnt++) {
    uint8_t mosi_byte = 0;
    uint8_t bit_mask = 1;
    uint32_t byte_spin_counter = 0;

    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      while (fast_gpio_read(sck_pin)) {
        if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
          result.status = err_msg_timeout_SCK_high;
          fast_gpio_write_low(miso_pin);
          return result;
        }
      }

      if ((tx_frame[byte_cnt] & bit_mask) != 0) {
        fast_gpio_write_high(miso_pin);
      } else {
        fast_gpio_write_low(miso_pin);
      }

      while (!fast_gpio_read(sck_pin)) {
        if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
          result.status = err_msg_timeout_SCK_low;
          fast_gpio_write_low(miso_pin);
          return result;
        }
      }

      if (fast_gpio_read(mosi_pin)) {
        mosi_byte = static_cast<uint8_t>(mosi_byte + bit_mask);
      }

      bit_mask = static_cast<uint8_t>(bit_mask << 1);
    }

    if (rx_frame[byte_cnt] != mosi_byte) {
      result.new_data_packet_received = true;
      rx_frame[byte_cnt] = mosi_byte;
    }

    if (byte_cnt == 0U) {
      result.header_byte = mosi_byte;
    }
  }

  fast_gpio_write_low(miso_pin);

  result.status = 0;
  result.bytes_received = kBaseFrameBytes;
  result.detected_type = MhiFrameType::STANDARD_20;

  if (target_frame_size < kExtendedFrameBytes) {
    return result;
  }

  result.extension_probe_attempted = true;

  const uint64_t extension_wait_start_us = now_us();
  while (fast_gpio_read(sck_pin)) {
    if ((now_us() - extension_wait_start_us) > static_cast<uint64_t>(this->config_.extension_gap_max_us)) {
      return result;
    }
    if ((mhi_now_ms() - start_ms) > max_time_ms) {
      result.status = err_msg_timeout_SCK_high;
      return result;
    }
  }

  result.extension_start_seen = true;

  for (std::size_t byte_cnt = kBaseFrameBytes; byte_cnt < kExtendedFrameBytes; byte_cnt++) {
    uint8_t mosi_byte = 0;
    uint8_t bit_mask = 1;
    uint32_t byte_spin_counter = 0;

    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      if (!(byte_cnt == kBaseFrameBytes && bit_cnt == 0)) {
        while (fast_gpio_read(sck_pin)) {
          if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
            result.status = err_msg_timeout_SCK_high;
            fast_gpio_write_low(miso_pin);
            return result;
          }
        }
      }

      if ((tx_frame[byte_cnt] & bit_mask) != 0) {
        fast_gpio_write_high(miso_pin);
      } else {
        fast_gpio_write_low(miso_pin);
      }

      while (!fast_gpio_read(sck_pin)) {
        if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
          result.status = err_msg_timeout_SCK_low;
          fast_gpio_write_low(miso_pin);
          return result;
        }
      }

      if (fast_gpio_read(mosi_pin)) {
        mosi_byte = static_cast<uint8_t>(mosi_byte + bit_mask);
      }

      bit_mask = static_cast<uint8_t>(bit_mask << 1);
    }

    if (rx_frame[byte_cnt] != mosi_byte) {
      result.new_data_packet_received = true;
      rx_frame[byte_cnt] = mosi_byte;
    }
  }

  fast_gpio_write_low(miso_pin);
  result.bytes_received = kExtendedFrameBytes;
  result.detected_type = MhiFrameType::EXTENDED_33;
  return result;
}

}  // namespace mhi
}  // namespace esphome
