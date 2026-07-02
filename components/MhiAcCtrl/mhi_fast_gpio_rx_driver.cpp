#include "mhi_fast_gpio_rx_driver.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#if defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) ||                       \
    defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32H2)
#include "soc/gpio_struct.h"
#define MHI_USE_FAST_GPIO_RX 1
#else
#define MHI_USE_FAST_GPIO_RX 0
#endif

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_fast_gpio_rx";

namespace {

static portMUX_TYPE g_mhi_fast_gpio_rx_mux = portMUX_INITIALIZER_UNLOCKED;

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

#if MHI_USE_FAST_GPIO_RX

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32H2)

inline bool IRAM_ATTR fast_gpio_read(int pin) {
  if (pin < 0 || pin >= 32) {
    return false;
  }

  return ((GPIO.in.val >> pin) & 0x1U) != 0U;
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

inline void IRAM_ATTR fast_gpio_write_low(int pin) {
  gpio_set_level(static_cast<gpio_num_t>(pin), 0);
}

#endif

int IRAM_ATTR read_one_byte_fast_gpio(int sck_pin, int mosi_pin, int miso_pin, uint8_t* rx_byte, uint32_t start_ms,
                                      uint32_t max_time_ms) {
  uint8_t mosi_byte = 0U;
  uint8_t bit_mask = 1U;
  uint32_t spin_counter = 0U;

  for (uint8_t bit = 0U; bit < 8U; bit++) {
    // CPOL=1: clock idles high. Wait for falling edge / low phase.
    while (fast_gpio_read(sck_pin)) {
      if (timeout_expired(start_ms, max_time_ms, spin_counter)) {
        fast_gpio_write_low(miso_pin);
        return static_cast<int>(MhiFastGpioRxStatus::TIMEOUT_SCK_HIGH);
      }
    }

    // RX-only path does not own TX timing. Keep MISO low while sampling MOSI.
    fast_gpio_write_low(miso_pin);

    // Sample MOSI on rising edge.
    while (!fast_gpio_read(sck_pin)) {
      if (timeout_expired(start_ms, max_time_ms, spin_counter)) {
        fast_gpio_write_low(miso_pin);
        return static_cast<int>(MhiFastGpioRxStatus::TIMEOUT_SCK_LOW);
      }
    }

    if (fast_gpio_read(mosi_pin)) {
      mosi_byte = static_cast<uint8_t>(mosi_byte | bit_mask);
    }

    bit_mask = static_cast<uint8_t>(bit_mask << 1U);
  }

  fast_gpio_write_low(miso_pin);
  *rx_byte = mosi_byte;

  return static_cast<int>(MhiFastGpioRxStatus::OK);
}

}  // namespace

bool MhiFastGpioRxDriver::setup(const MhiTransportPins& pins) {
  pins_ = pins;

  if (pins_.sck < 0 || pins_.mosi < 0 || pins_.miso < 0) {
    ESP_LOGE(TAG, "FastGPIO RX setup failed: invalid pins SCK=%d MOSI=%d MISO=%d", pins_.sck, pins_.mosi, pins_.miso);

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

  portENTER_CRITICAL(&marker_mux_);
  marker_ = {};
  portEXIT_CRITICAL(&marker_mux_);

  ready_ = true;

  ESP_LOGCONFIG(TAG, "FastGPIO RX driver ready: SCK=%d MOSI=%d MISO=%d", pins_.sck, pins_.mosi, pins_.miso);

  return true;
}

std::size_t MhiFastGpioRxDriver::read(uint8_t* dst, std::size_t max_len) {
  if (!ready_ || dst == nullptr || max_len < kMhiFrame20Bytes) {
    return 0U;
  }

  std::size_t bytes_received = 0U;

  const MhiFastGpioRxStatus status = this->read_frame_(dst, max_len, bytes_received);

  if (status != MhiFastGpioRxStatus::OK) {
    return 0U;
  }

  this->update_bus_marker_(bytes_received);
  return bytes_received;
}

MhiBusMarker MhiFastGpioRxDriver::bus_marker() const {
  MhiBusMarker marker{};

  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&marker_mux_));
  marker = marker_;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&marker_mux_));

  return marker;
}

MhiFastGpioRxStatus MhiFastGpioRxDriver::read_frame_(uint8_t* rx_frame, std::size_t rx_capacity,
                                                     std::size_t& bytes_received) {
  bytes_received = 0U;

  if (!ready_) {
    return MhiFastGpioRxStatus::NOT_READY;
  }

  if (rx_frame == nullptr || rx_capacity < kMhiFrame20Bytes) {
    return MhiFastGpioRxStatus::BAD_ARGUMENT;
  }

  const std::size_t target_len =
      std::min<std::size_t>(config_.frame_size_hint == 33U ? kMhiFrame33Bytes : kMhiFrame20Bytes, rx_capacity);

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
      return MhiFastGpioRxStatus::TIMEOUT_SCK_LOW;
    }
  }

  for (std::size_t index = 0U; index < target_len; index++) {
    uint8_t rx_byte = 0U;

    int rc = 0;
    if (config_.byte_critical_sections) {
      portENTER_CRITICAL(&g_mhi_fast_gpio_rx_mux);
      rc = read_one_byte_fast_gpio(pins_.sck, pins_.mosi, pins_.miso, &rx_byte, start_ms, config_.max_exchange_time_ms);
      portEXIT_CRITICAL(&g_mhi_fast_gpio_rx_mux);
    } else {
      rc = read_one_byte_fast_gpio(pins_.sck, pins_.mosi, pins_.miso, &rx_byte, start_ms, config_.max_exchange_time_ms);
    }

    if (rc < 0) {
      fast_gpio_write_low(pins_.miso);
      return static_cast<MhiFastGpioRxStatus>(rc);
    }

    rx_frame[index] = rx_byte;
  }

  fast_gpio_write_low(pins_.miso);

  bytes_received = target_len;
  return MhiFastGpioRxStatus::OK;
}

void MhiFastGpioRxDriver::update_bus_marker_(std::size_t frame_len) {
  portENTER_CRITICAL(&marker_mux_);
  marker_.valid = true;
  marker_.sequence++;
  marker_.frame_end_us = micros();
  marker_.frame_len = frame_len;
  portEXIT_CRITICAL(&marker_mux_);
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
