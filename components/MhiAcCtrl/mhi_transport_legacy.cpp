#include "mhi_transport_legacy.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_timer.h>

#include "MHI-AC-Ctrl-core.h"
#include "mhi_time.h"

// Fast GPIO register access for ESP32-family targets.
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

inline uint64_t IRAM_ATTR now_us() {
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

std::size_t MhiTransportLegacy::determine_target_frame_bytes_(std::size_t rx_capacity) const {
  std::size_t target = kBaseFrameBytes;

  switch (this->config_.protocol_mode) {
    case MhiProtocolMode::STANDARD_ONLY:
      target = kBaseFrameBytes;
      break;
    case MhiProtocolMode::EXTENDED_PREFER:
    case MhiProtocolMode::AUTO:
    default:
      // Phase 2 transport can safely capture a 20-byte base frame and then
      // optionally continue into the 13-byte extension. In AUTO, prefer having
      // room for the extension instead of forcing a 20-byte truncation.
      target = kExtendedFrameBytes;
      break;
  }

  target = std::max<std::size_t>(target, static_cast<std::size_t>(this->config_.frame_size_hint));
  target = std::min<std::size_t>(target, static_cast<std::size_t>(this->config_.max_frame_size));
  target = std::min<std::size_t>(target, rx_capacity);

  if (target < kBaseFrameBytes) {
    return 0U;
  }
  return target;
}

bool IRAM_ATTR MhiTransportLegacy::wait_for_frame_idle_(uint32_t start_ms, uint32_t max_time_ms) const {
  uint32_t sck_high_start_ms = mhi_now_ms();
  uint32_t wait_spin_counter = 0;

  while ((mhi_now_ms() - sck_high_start_ms) < this->config_.frame_start_idle_ms) {
    if (!fast_gpio_read(this->config_.sck_pin)) {
      sck_high_start_ms = mhi_now_ms();
    }
    if (timed_out(start_ms, max_time_ms, wait_spin_counter)) {
      return false;
    }
  }

  return true;
}

bool IRAM_ATTR MhiTransportLegacy::read_byte_(
    uint8_t tx_byte,
    uint8_t &rx_byte,
    uint32_t start_ms,
    uint32_t max_time_ms) const {
  rx_byte = 0;
  uint8_t bit_mask = 1;
  uint32_t byte_spin_counter = 0;

  for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
    // Wait for falling edge: SCK high -> low
    while (fast_gpio_read(this->config_.sck_pin)) {
      if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
        fast_gpio_write_low(this->config_.miso_pin);
        return false;
      }
    }

    // Drive outgoing bit immediately on falling edge.
    if ((tx_byte & bit_mask) != 0) {
      fast_gpio_write_high(this->config_.miso_pin);
    } else {
      fast_gpio_write_low(this->config_.miso_pin);
    }

    // Wait for rising edge: SCK low -> high
    while (!fast_gpio_read(this->config_.sck_pin)) {
      if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
        fast_gpio_write_low(this->config_.miso_pin);
        return false;
      }
    }

    // Sample MOSI right after rising edge.
    if (fast_gpio_read(this->config_.mosi_pin)) {
      rx_byte = static_cast<uint8_t>(rx_byte + bit_mask);
    }

    bit_mask = static_cast<uint8_t>(bit_mask << 1);
  }

  return true;
}

bool IRAM_ATTR MhiTransportLegacy::wait_for_extension_start_(uint32_t start_ms, uint32_t max_time_ms) const {
  const uint64_t wait_start_us = now_us();

  // The bus idles high. Extended frames add a short pause before the next byte.
  while (fast_gpio_read(this->config_.sck_pin)) {
    if ((now_us() - wait_start_us) > static_cast<uint64_t>(this->config_.extension_gap_max_us)) {
      return false;
    }
    if ((mhi_now_ms() - start_ms) > max_time_ms) {
      return false;
    }
  }

  return true;
}

MhiFrameExchangeResult IRAM_ATTR MhiTransportLegacy::exchange_frame(
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t rx_capacity,
    uint32_t max_time_ms) {
  MhiFrameExchangeResult result{};

  const std::size_t target_frame_size = this->determine_target_frame_bytes_(rx_capacity);
  if (target_frame_size < kBaseFrameBytes) {
    result.status = err_msg_timeout_SCK_low;
    return result;
  }

  const uint32_t start_ms = mhi_now_ms();
  if (!this->wait_for_frame_idle_(start_ms, max_time_ms)) {
    result.status = err_msg_timeout_SCK_low;
    return result;
  }

  // Always capture the 20-byte base frame first.
  for (std::size_t byte_cnt = 0; byte_cnt < kBaseFrameBytes; byte_cnt++) {
    uint8_t mosi_byte = 0;
    if (!this->read_byte_(tx_frame[byte_cnt], mosi_byte, start_ms, max_time_ms)) {
      result.status = err_msg_timeout_SCK_high;
      return result;
    }

    if (rx_frame[byte_cnt] != mosi_byte) {
      result.new_data_packet_received = true;
      rx_frame[byte_cnt] = mosi_byte;
    }

    if (byte_cnt == 0U) {
      result.header_byte = mosi_byte;
    }
  }

  fast_gpio_write_low(this->config_.miso_pin);

  result.status = 0;
  result.bytes_received = kBaseFrameBytes;
  result.base_frame_complete = true;

  if (result.header_byte == 0x6c) {
    result.detected_type = MhiFrameType::STANDARD_20;
  } else {
    result.detected_type = MhiFrameType::UNKNOWN;
  }

  // Standard-only mode stops here. AUTO and EXTENDED_PREFER leave room for the
  // 13-byte continuation used by extended frames.
  if (target_frame_size < kExtendedFrameBytes || this->config_.protocol_mode == MhiProtocolMode::STANDARD_ONLY) {
    return result;
  }

  // In AUTO, only chase the extension when the incoming header indicates the
  // extended wire format. In EXTENDED_PREFER, allow the configured preference to
  // override this and still probe for the continuation.
  const bool should_probe_extension =
      (this->config_.protocol_mode == MhiProtocolMode::EXTENDED_PREFER) ||
      (result.header_byte == 0x6d);

  if (!should_probe_extension) {
    return result;
  }

  if (!this->wait_for_extension_start_(start_ms, max_time_ms)) {
    // No continuation observed inside the extension window. Leave the result as
    // a complete 20-byte base frame and let later phases decide how to interpret
    // the header/type mismatch.
    return result;
  }

  for (std::size_t byte_cnt = kBaseFrameBytes; byte_cnt < kExtendedFrameBytes; byte_cnt++) {
    uint8_t mosi_byte = 0;
    if (!this->read_byte_(tx_frame[byte_cnt], mosi_byte, start_ms, max_time_ms)) {
      result.status = err_msg_timeout_SCK_high;
      fast_gpio_write_low(this->config_.miso_pin);
      return result;
    }

    if (rx_frame[byte_cnt] != mosi_byte) {
      result.new_data_packet_received = true;
      rx_frame[byte_cnt] = mosi_byte;
    }
  }

  fast_gpio_write_low(this->config_.miso_pin);

  result.bytes_received = kExtendedFrameBytes;
  result.detected_type = MhiFrameType::EXTENDED_33;
  result.extended_tail_present = true;
  return result;
}

}  // namespace mhi
}  // namespace esphome
