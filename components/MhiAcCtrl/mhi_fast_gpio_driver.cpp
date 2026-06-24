#include "mhi_fast_gpio_driver.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#if defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) ||                       \
    defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32H2)
#include "soc/gpio_struct.h"
#define MHI_USE_FAST_GPIO 1
#else
#define MHI_USE_FAST_GPIO 0
#endif

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_fast_gpio";

namespace {

static portMUX_TYPE g_mhi_fast_gpio_mux = portMUX_INITIALIZER_UNLOCKED;

constexpr uint32_t kTimeoutCheckMask = 0x3F;

inline uint32_t now_ms() {
  return millis();
}

inline bool timeout_expired(uint32_t start_ms, uint32_t max_time_ms, uint32_t& spin_counter) {
  spin_counter++;

  if ((spin_counter & kTimeoutCheckMask) != 0U) {
    return false;
  }

  return (now_ms() - start_ms) > max_time_ms;
}

#if MHI_USE_FAST_GPIO

inline bool IRAM_ATTR fast_gpio_read(int pin) {
  if (pin < 32) {
    return ((GPIO.in >> pin) & 0x1U) != 0U;
  }

  return ((GPIO.in1.val >> (pin - 32)) & 0x1U) != 0U;
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

int IRAM_ATTR read_one_byte_fast_gpio(int sck_pin, int mosi_pin, int miso_pin, uint8_t tx_byte, uint8_t* rx_byte,
                                      uint32_t start_ms, uint32_t max_time_ms) {
  uint8_t mosi_byte = 0U;
  uint8_t bit_mask = 1U;
  uint32_t spin_counter = 0U;

  for (uint8_t bit = 0U; bit < 8U; bit++) {
    // CPOL=1: clock idles high. Wait for falling edge / low phase.
    while (fast_gpio_read(sck_pin)) {
      if (timeout_expired(start_ms, max_time_ms, spin_counter)) {
        fast_gpio_write_low(miso_pin);
        return static_cast<int>(MhiFastGpioExchangeStatus::TIMEOUT_SCK_HIGH);
      }
    }

    // CPHA=1: data changes on falling edge.
    if ((tx_byte & bit_mask) != 0U) {
      fast_gpio_write_high(miso_pin);
    } else {
      fast_gpio_write_low(miso_pin);
    }

    // Sample MOSI on rising edge.
    while (!fast_gpio_read(sck_pin)) {
      if (timeout_expired(start_ms, max_time_ms, spin_counter)) {
        fast_gpio_write_low(miso_pin);
        return static_cast<int>(MhiFastGpioExchangeStatus::TIMEOUT_SCK_LOW);
      }
    }

    if (fast_gpio_read(mosi_pin)) {
      mosi_byte = static_cast<uint8_t>(mosi_byte | bit_mask);
    }

    bit_mask = static_cast<uint8_t>(bit_mask << 1U);
  }

  fast_gpio_write_low(miso_pin);
  *rx_byte = mosi_byte;

  return static_cast<int>(MhiFastGpioExchangeStatus::OK);
}

}  // namespace

bool MhiFastGpioDriver::setup(const MhiTransportPins& pins) {
  pins_ = pins;

  if (pins_.sck < 0 || pins_.mosi < 0 || pins_.miso < 0) {
    ESP_LOGE(TAG, "FastGPIO setup failed: invalid pins SCK=%d MOSI=%d MISO=%d", pins_.sck, pins_.mosi, pins_.miso);

    ready_ = false;
    return false;
  }

  gpio_reset_pin(static_cast<gpio_num_t>(pins_.sck));
  gpio_reset_pin(static_cast<gpio_num_t>(pins_.mosi));
  gpio_reset_pin(static_cast<gpio_num_t>(pins_.miso));

  gpio_set_direction(static_cast<gpio_num_t>(pins_.sck), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(pins_.mosi), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(pins_.miso), GPIO_MODE_OUTPUT);

  gpio_set_level(static_cast<gpio_num_t>(pins_.miso), 0);

  ready_ = true;

  ESP_LOGCONFIG(TAG, "FastGPIO driver ready: SCK=%d MOSI=%d MISO=%d", pins_.sck, pins_.mosi, pins_.miso);

  return true;
}

bool MhiFastGpioDriver::send(const uint8_t* data, std::size_t len) {
  if (data == nullptr || (len != kMhiFrame20Bytes && len != kMhiFrame33Bytes)) {
    return false;
  }

  portENTER_CRITICAL(&tx_mux_);
  std::memset(tx_frame_, 0, sizeof(tx_frame_));
  std::memcpy(tx_frame_, data, len);

  tx_len_ = len;
  tx_staged_ = true;
  portEXIT_CRITICAL(&tx_mux_);

  return true;
}

std::size_t MhiFastGpioDriver::read(uint8_t* dst, std::size_t max_len) {
  if (!ready_ || dst == nullptr || max_len < kMhiFrame20Bytes) {
    return 0U;
  }

  std::size_t bytes_received = 0U;

  const MhiFastGpioExchangeStatus status = this->exchange_frame(dst, max_len, bytes_received);

  if (status != MhiFastGpioExchangeStatus::OK) {
    return 0U;
  }

  return bytes_received;
}

MhiFastGpioExchangeStatus MhiFastGpioDriver::exchange_frame(uint8_t* rx_frame, std::size_t rx_capacity,
                                                            std::size_t& bytes_received) {
  bytes_received = 0U;

  if (!ready_) {
    return MhiFastGpioExchangeStatus::NOT_READY;
  }

  if (rx_frame == nullptr || rx_capacity < kMhiFrame20Bytes) {
    return MhiFastGpioExchangeStatus::BAD_ARGUMENT;
  }

  const std::size_t target_len =
      std::min<std::size_t>(config_.frame_size_hint == 33U ? kMhiFrame33Bytes : kMhiFrame20Bytes, rx_capacity);

  uint8_t exchange_tx_frame[kMhiMaxFrameBytes]{};
  std::size_t exchange_tx_len = target_len;

  portENTER_CRITICAL(&tx_mux_);
  if (!tx_staged_) {
    // Tangible baseline behaviour:
    // capture MOSI while clocking out zeros until the TX builder stages a frame.
    std::memset(tx_frame_, 0, sizeof(tx_frame_));
    tx_len_ = target_len;
    tx_staged_ = true;
  }

  std::memcpy(exchange_tx_frame, tx_frame_, sizeof(exchange_tx_frame));
  exchange_tx_len = tx_len_;
  portEXIT_CRITICAL(&tx_mux_);

  const uint32_t start_ms = now_ms();

  uint32_t idle_start_ms = now_ms();
  uint32_t spin_counter = 0U;

  // Wait until SCK has been idle-high long enough to treat the next low pulse
  // as the start of a fresh frame.
  while ((now_ms() - idle_start_ms) < config_.frame_start_idle_ms) {
    if (!fast_gpio_read(pins_.sck)) {
      idle_start_ms = now_ms();
    }

    if (timeout_expired(start_ms, config_.max_exchange_time_ms, spin_counter)) {
      fast_gpio_write_low(pins_.miso);
      return MhiFastGpioExchangeStatus::TIMEOUT_SCK_LOW;
    }
  }

  for (std::size_t index = 0U; index < target_len; index++) {
    uint8_t rx_byte = 0U;
    const uint8_t tx_byte = index < exchange_tx_len ? exchange_tx_frame[index] : 0U;

    // Keep the edge-to-byte sample timing protected, but do not block
    // interrupts for the entire 20/33-byte frame. The full-frame critical
    // section kept RX clean, but it could starve ESP-IDF drivers such as RMT
    // and Wi-Fi for one whole bus frame. Byte-bounded locking gives pending
    // interrupts a chance to run between bytes while preserving the tight
    // CPHA=1 timing inside each byte.
    portENTER_CRITICAL(&g_mhi_fast_gpio_mux);
    const int rc = read_one_byte_fast_gpio(pins_.sck, pins_.mosi, pins_.miso, tx_byte, &rx_byte, start_ms,
                                           config_.max_exchange_time_ms);
    portEXIT_CRITICAL(&g_mhi_fast_gpio_mux);

    if (rc < 0) {
      fast_gpio_write_low(pins_.miso);
      return static_cast<MhiFastGpioExchangeStatus>(rc);
    }

    rx_frame[index] = rx_byte;
  }

  fast_gpio_write_low(pins_.miso);

  bytes_received = target_len;
  return MhiFastGpioExchangeStatus::OK;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome