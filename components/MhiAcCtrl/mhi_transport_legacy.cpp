#include "mhi_transport_legacy.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <esp_attr.h>

#include "MHI-AC-Ctrl-core.h"
#include "mhi_time.h"

// Fast GPIO register access for ESP32-family targets.
// This keeps the transport model the same, but reduces per-bit overhead.
#if defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32H2)
#include "soc/gpio_struct.h"
#define MHI_USE_FAST_GPIO 1
#else
#define MHI_USE_FAST_GPIO 0
#endif

namespace esphome {
namespace mhi {

namespace {

constexpr uint32_t kTimeoutCheckMask = 0x3F;  // check elapsed time every 64 spins

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

std::size_t MhiTransportLegacy::determine_target_frame_bytes_(std::size_t rx_capacity) const {
  std::size_t target = static_cast<std::size_t>(this->config_.frame_size_hint);

  if (target != 20U && target != 33U) {
    target = 20U;
  }

  if (this->config_.protocol_mode == MhiProtocolMode::EXTENDED_PREFER) {
    target = 33U;
  } else if (this->config_.protocol_mode == MhiProtocolMode::STANDARD_ONLY) {
    target = 20U;
  }

  target = std::min<std::size_t>(target, static_cast<std::size_t>(this->config_.max_frame_size));
  target = std::min<std::size_t>(target, rx_capacity);
  return target;
}

MhiFrameExchangeResult IRAM_ATTR MhiTransportLegacy::exchange_frame(
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t rx_capacity,
    uint32_t max_time_ms) {
  MhiFrameExchangeResult result{};

  const std::size_t target_frame_size = this->determine_target_frame_bytes_(rx_capacity);
  if (target_frame_size == 0U) {
    result.status = err_msg_timeout_SCK_low;
    return result;
  }

  const int sck_pin = this->config_.sck_pin;
  const int mosi_pin = this->config_.mosi_pin;
  const int miso_pin = this->config_.miso_pin;

  const uint32_t start_ms = mhi_now_ms();

  // Wait for stable-high idle on SCK to detect frame boundary.
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

  for (std::size_t byte_cnt = 0; byte_cnt < target_frame_size; byte_cnt++) {
    uint8_t mosi_byte = 0;
    uint8_t bit_mask = 1;
    uint32_t byte_spin_counter = 0;

    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      // Wait for falling edge: SCK high -> low
      while (fast_gpio_read(sck_pin)) {
        if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
          result.status = err_msg_timeout_SCK_high;
          fast_gpio_write_low(miso_pin);
          return result;
        }
      }

      // Drive outgoing bit immediately on falling edge
      if ((tx_frame[byte_cnt] & bit_mask) != 0) {
        fast_gpio_write_high(miso_pin);
      } else {
        fast_gpio_write_low(miso_pin);
      }

      // Wait for rising edge: SCK low -> high
      while (!fast_gpio_read(sck_pin)) {
        if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
          result.status = err_msg_timeout_SCK_low;
          fast_gpio_write_low(miso_pin);
          return result;
        }
      }

      // Sample MOSI right after rising edge
      if (fast_gpio_read(mosi_pin)) {
        mosi_byte = static_cast<uint8_t>(mosi_byte + bit_mask);
      }

      bit_mask = static_cast<uint8_t>(bit_mask << 1);
    }

    if (rx_frame[byte_cnt] != mosi_byte) {
      result.new_data_packet_received = true;
      rx_frame[byte_cnt] = mosi_byte;
    }

    if (byte_cnt == 0) {
      result.header_byte = mosi_byte;
    }

    // Coarse timeout check between bytes
    if ((mhi_now_ms() - start_ms) > max_time_ms) {
      result.status = err_msg_timeout_SCK_high;
      fast_gpio_write_low(miso_pin);
      return result;
    }
  }

  fast_gpio_write_low(miso_pin);

  result.status = 0;
  result.bytes_received = target_frame_size;
  result.base_frame_complete = (target_frame_size >= 20U);

  if (target_frame_size >= 33U) {
    result.detected_type = MhiFrameType::EXTENDED_33;
    result.extended_tail_present = true;
  } else if (target_frame_size >= 20U) {
    result.detected_type = MhiFrameType::STANDARD_20;
  }

  return result;
}

}  // namespace mhi
}  // namespace esphome
