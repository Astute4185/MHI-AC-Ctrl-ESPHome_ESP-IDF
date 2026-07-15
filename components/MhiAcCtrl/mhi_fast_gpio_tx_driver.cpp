#include "mhi_fast_gpio_tx_driver.h"

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
#define MHI_USE_FAST_GPIO_TX 1
#else
#define MHI_USE_FAST_GPIO_TX 0
#endif

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_fast_gpio_tx";

namespace {

static portMUX_TYPE g_mhi_fast_gpio_tx_mux = portMUX_INITIALIZER_UNLOCKED;

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

#if MHI_USE_FAST_GPIO_TX

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32H2)

inline bool IRAM_ATTR fast_gpio_read(int pin) {
  if (pin < 0 || pin >= 32) {
    return false;
  }

  return ((GPIO.in.val >> pin) & 0x1U) != 0U;
}

inline void IRAM_ATTR fast_gpio_write_high(int pin) {
  if (pin < 0 || pin >= 32) {
    return;
  }

  GPIO.out_w1ts.val = (1UL << pin);
}

inline void IRAM_ATTR fast_gpio_write_low(int pin) {
  if (pin < 0 || pin >= 32) {
    return;
  }

  GPIO.out_w1tc.val = (1UL << pin);
}

#else

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

#endif

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

int IRAM_ATTR write_one_byte_fast_gpio(int sck_pin, int miso_pin, uint8_t tx_byte, uint32_t start_ms,
                                       uint32_t max_time_ms) {
  uint8_t bit_mask = 1U;
  uint32_t spin_counter = 0U;

  for (uint8_t bit = 0U; bit < 8U; bit++) {
    // CPOL=1: clock idles high. Wait for falling edge / low phase.
    while (fast_gpio_read(sck_pin)) {
      if (timeout_expired(start_ms, max_time_ms, spin_counter)) {
        fast_gpio_write_low(miso_pin);
        return -1;
      }
    }

    // CPHA=1: data changes on falling edge.
    if ((tx_byte & bit_mask) != 0U) {
      fast_gpio_write_high(miso_pin);
    } else {
      fast_gpio_write_low(miso_pin);
    }

    // Hold until the AC samples on the rising edge.
    while (!fast_gpio_read(sck_pin)) {
      if (timeout_expired(start_ms, max_time_ms, spin_counter)) {
        fast_gpio_write_low(miso_pin);
        return -2;
      }
    }

    bit_mask = static_cast<uint8_t>(bit_mask << 1U);
  }

  fast_gpio_write_low(miso_pin);
  return 0;
}

}  // namespace

bool MhiFastGpioTxDriver::setup(const MhiTransportPins& pins) {
  pins_ = pins;

  if (pins_.sck < 0 || pins_.miso < 0) {
    ESP_LOGE(TAG, "FastGPIO TX setup failed: invalid pins SCK=%d MISO=%d", pins_.sck, pins_.miso);
    ready_ = false;
    return false;
  }

  // Do not reset or reconfigure SCK/MOSI here. In native_spi_rx mode SCK/MOSI
  // belong to the SPI peripheral. This driver only owns MISO.
  gpio_input_enable(static_cast<gpio_num_t>(pins_.sck));
  gpio_reset_pin(static_cast<gpio_num_t>(pins_.miso));
  gpio_set_direction(static_cast<gpio_num_t>(pins_.miso), GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(pins_.miso), 0);

  ready_ = true;

  ESP_LOGCONFIG(TAG, "FastGPIO TX-only driver ready: SCK=%d MISO=%d", pins_.sck, pins_.miso);
  return true;
}

bool MhiFastGpioTxDriver::send(const uint8_t* data, std::size_t len) {
  if (!ready_ || data == nullptr || (len != kMhiFrame20Bytes && len != kMhiFrame33Bytes)) {
    return false;
  }

  uint8_t frame[kMhiMaxFrameBytes]{};
  std::memcpy(frame, data, std::min<std::size_t>(len, sizeof(frame)));

  return this->transmit_frame_(frame, len);
}

bool MhiFastGpioTxDriver::transmit_frame_(const uint8_t* data, std::size_t len) {
  if (data == nullptr || len == 0U || len > kMhiMaxFrameBytes) {
    return false;
  }

  const std::size_t target_len =
      std::min<std::size_t>(config_.frame_size_hint == 33U ? kMhiFrame33Bytes : kMhiFrame20Bytes, len);

  const uint32_t start_ms = now_ms();
  uint32_t idle_start_ms = now_ms();
  uint32_t spin_counter = 0U;

  // Wait until SCK has been idle-high long enough to align with the next AC frame.
  while ((now_ms() - idle_start_ms) < config_.frame_start_idle_ms) {
    if (!fast_gpio_read(pins_.sck)) {
      idle_start_ms = now_ms();
    }

    if (timeout_expired(start_ms, config_.max_exchange_time_ms, spin_counter)) {
      fast_gpio_write_low(pins_.miso);
      ESP_LOGD(TAG, "TX-only frame timeout waiting for idle-high SCK");
      return false;
    }
  }

  for (std::size_t index = 0U; index < target_len; index++) {
    int rc = 0;

    if (config_.byte_critical_sections) {
      portENTER_CRITICAL(&g_mhi_fast_gpio_tx_mux);
      rc = write_one_byte_fast_gpio(pins_.sck, pins_.miso, data[index], start_ms, config_.max_exchange_time_ms);
      portEXIT_CRITICAL(&g_mhi_fast_gpio_tx_mux);
    } else {
      rc = write_one_byte_fast_gpio(pins_.sck, pins_.miso, data[index], start_ms, config_.max_exchange_time_ms);
    }

    if (rc < 0) {
      fast_gpio_write_low(pins_.miso);
      ESP_LOGD(TAG, "TX-only frame timeout index=%u rc=%d", static_cast<unsigned int>(index), rc);
      return false;
    }
  }

  fast_gpio_write_low(pins_.miso);
  return true;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
