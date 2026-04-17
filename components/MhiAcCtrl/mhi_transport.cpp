#include "mhi_transport.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

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
constexpr std::size_t kInvalidHeaderOvercaptureBytes = 2U;

static portMUX_TYPE g_mhi_capture_mux = portMUX_INITIALIZER_UNLOCKED;

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

inline bool has_valid_header_prefix(const uint8_t *frame, std::size_t len) {
  return len >= 3U && (frame[0] == 0x6C || frame[0] == 0x6D) && frame[1] == 0x80 && frame[2] == 0x04;
}

int IRAM_ATTR read_one_byte(
    int sck_pin,
    int mosi_pin,
    int miso_pin,
    uint8_t tx_byte,
    uint8_t *rx_byte,
    uint32_t start_ms,
    uint32_t max_time_ms,
    bool first_falling_edge_already_seen) {
  uint8_t mosi_byte = 0;
  uint8_t bit_mask = 1;
  uint32_t byte_spin_counter = 0;

  for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
    if (!(first_falling_edge_already_seen && bit_cnt == 0U)) {
      while (fast_gpio_read(sck_pin)) {
        if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
          fast_gpio_write_low(miso_pin);
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
      if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
        fast_gpio_write_low(miso_pin);
        return err_msg_timeout_SCK_low;
      }
    }

    if (fast_gpio_read(mosi_pin)) {
      mosi_byte = static_cast<uint8_t>(mosi_byte + bit_mask);
    }

    bit_mask = static_cast<uint8_t>(bit_mask << 1);
  }

  fast_gpio_write_low(miso_pin);
  *rx_byte = mosi_byte;
  return 0;
}

int IRAM_ATTR read_frame_range(
    int sck_pin,
    int mosi_pin,
    int miso_pin,
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t start_index,
    std::size_t end_index,
    uint32_t start_ms,
    uint32_t max_time_ms,
    bool first_falling_edge_already_seen,
    bool *new_data_packet_received,
    uint8_t *header_byte) {
  for (std::size_t byte_cnt = start_index; byte_cnt < end_index; byte_cnt++) {
    uint8_t rx_byte = 0;
    const int rc = read_one_byte(
        sck_pin,
        mosi_pin,
        miso_pin,
        tx_frame[byte_cnt],
        &rx_byte,
        start_ms,
        max_time_ms,
        first_falling_edge_already_seen && byte_cnt == start_index);
    if (rc < 0) {
      return rc;
    }

    if (rx_frame[byte_cnt] != rx_byte) {
      *new_data_packet_received = true;
      rx_frame[byte_cnt] = rx_byte;
    }

    if (byte_cnt == 0U && header_byte != nullptr) {
      *header_byte = rx_byte;
    }
  }

  return 0;
}

bool wait_for_next_falling_edge_us(int sck_pin, uint32_t timeout_us, uint32_t start_ms, uint32_t max_time_ms) {
  const uint64_t begin_us = now_us();
  while (fast_gpio_read(sck_pin)) {
    if ((now_us() - begin_us) > static_cast<uint64_t>(timeout_us)) {
      return false;
    }
    if ((mhi_now_ms() - start_ms) > max_time_ms) {
      return false;
    }
  }
  return true;
}

}  // namespace

void MhiTransport::setup(const MhiTransportConfig &config) {
  this->config_ = config;

  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.sck_pin));
  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.mosi_pin));
  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.miso_pin));

  gpio_set_direction(static_cast<gpio_num_t>(this->config_.sck_pin), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(this->config_.mosi_pin), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(this->config_.miso_pin), GPIO_MODE_OUTPUT);

  gpio_set_level(static_cast<gpio_num_t>(this->config_.miso_pin), 0);
}

MhiFrameExchangeResult IRAM_ATTR MhiTransport::exchange_frame(
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

  result.critical_capture_used = true;

  int rc = 0;
  portENTER_CRITICAL(&g_mhi_capture_mux);
  rc = read_frame_range(
      sck_pin,
      mosi_pin,
      miso_pin,
      tx_frame,
      rx_frame,
      0U,
      kBaseFrameBytes,
      start_ms,
      max_time_ms,
      false,
      &result.new_data_packet_received,
      &result.header_byte);
  portEXIT_CRITICAL(&g_mhi_capture_mux);

  if (rc < 0) {
    result.status = rc;
    return result;
  }

  result.status = 0;
  result.bytes_received = kBaseFrameBytes;
  result.detected_type = MhiFrameType::STANDARD_20;

  if (target_frame_size < kExtendedFrameBytes) {
    // No extension requested.
  } else {
    result.extension_probe_attempted = true;

    // Keep the restart-edge wait and the extension-byte capture inside one
    // critical section. The previous version waited for the first post-gap
    // falling edge outside the lock and only entered the critical section when
    // starting byte 20, which still left a tiny preemption window. The
    // remaining failures were all consistent with losing alignment exactly at
    // the extension restart.
    bool extension_edge_seen = false;
    portENTER_CRITICAL(&g_mhi_capture_mux);
    extension_edge_seen = wait_for_next_falling_edge_us(
        sck_pin,
        this->config_.extension_gap_max_us,
        start_ms,
        max_time_ms);

    if (extension_edge_seen) {
      result.extension_start_seen = true;
      rc = read_frame_range(
          sck_pin,
          mosi_pin,
          miso_pin,
          tx_frame,
          rx_frame,
          kBaseFrameBytes,
          kExtendedFrameBytes,
          start_ms,
          max_time_ms,
          true,
          &result.new_data_packet_received,
          nullptr);
    }
    portEXIT_CRITICAL(&g_mhi_capture_mux);

    if (extension_edge_seen) {
      if (rc < 0) {
        result.status = rc;
        return result;
      }

      result.bytes_received = kExtendedFrameBytes;
      result.detected_type = MhiFrameType::EXTENDED_33;
    }
  }

  // Targeted overcapture only when the signature is already bad and we have a
  // full-sized frame. This helps prove start-of-frame slip without penalising
  // the normal fast path.
  if (result.bytes_received >= kExtendedFrameBytes && !has_valid_header_prefix(rx_frame, result.bytes_received)) {
    for (std::size_t i = 0; i < kInvalidHeaderOvercaptureBytes; i++) {
      uint8_t extra_byte = 0;
      rc = read_one_byte(
          sck_pin,
          mosi_pin,
          miso_pin,
          0x00,
          &extra_byte,
          start_ms,
          max_time_ms,
          false);
      if (rc < 0) {
        break;
      }
      result.overcapture_bytes[result.overcapture_len++] = extra_byte;
    }

    if (rx_frame[kExtendedFrameBytes - 1U] == 0x6D && result.overcapture_len >= 2U &&
        result.overcapture_bytes[0] == 0x80 && result.overcapture_bytes[1] == 0x04) {
      result.next_frame_signature_after_tail = true;
    }
    if (rx_frame[kExtendedFrameBytes - 1U] == 0x6C && result.overcapture_len >= 2U &&
        result.overcapture_bytes[0] == 0x80 && result.overcapture_bytes[1] == 0x04) {
      result.next_frame_signature_after_tail = true;
    }
  }

  fast_gpio_write_low(miso_pin);
  return result;
}

}  // namespace mhi
}  // namespace esphome