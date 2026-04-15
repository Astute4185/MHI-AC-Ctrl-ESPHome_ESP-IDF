#include "mhi_transport_legacy.h"

#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <esp_attr.h>

#include "esphome/core/log.h"
#include "MHI-AC-Ctrl-core.h"
#include "mhi_time.h"

// Fast GPIO register access for ESP32-family targets.
// This keeps the transport model the same, but reduces per-bit overhead.
#if defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) ||     defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) ||     defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32H2)
#include "soc/gpio_struct.h"
#define MHI_USE_FAST_GPIO 1
#else
#define MHI_USE_FAST_GPIO 0
#endif

namespace esphome {
namespace mhi {

static const char *const TAG = "mhi.transport.fast";

namespace {

template<typename T> inline T clamp_value(T value, T lower, T upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

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

}  // namespace

bool IRAM_ATTR MhiTransportLegacy::overall_timed_out_(uint32_t start_us, uint32_t max_time_ms) const {
  return (mhi_now_us() - start_us) > (max_time_ms * 1000U);
}

void MhiTransportLegacy::reset_calibration_() {
  this->timing_.calibration_complete = false;
  this->stats_.frames_valid = 0;
  this->stats_.max_observed_high_us = 0;
  this->stats_.max_observed_low_us = 0;
  this->stats_.max_observed_byte_us = 0;
  this->stats_.max_observed_frame_start_high_us = 0;
}

void MhiTransportLegacy::update_frame_observation_(
    uint32_t stable_high_us,
    uint32_t max_high_wait_us,
    uint32_t max_low_wait_us,
    uint32_t max_byte_us) {
  if (stable_high_us > this->stats_.max_observed_frame_start_high_us) {
    this->stats_.max_observed_frame_start_high_us = stable_high_us;
  }
  if (max_high_wait_us > this->stats_.max_observed_high_us) {
    this->stats_.max_observed_high_us = max_high_wait_us;
  }
  if (max_low_wait_us > this->stats_.max_observed_low_us) {
    this->stats_.max_observed_low_us = max_low_wait_us;
  }
  if (max_byte_us > this->stats_.max_observed_byte_us) {
    this->stats_.max_observed_byte_us = max_byte_us;
  }
}

void MhiTransportLegacy::finalize_calibration_() {
  const uint32_t tuned_high = clamp_value<uint32_t>(
      this->stats_.max_observed_high_us + (this->stats_.max_observed_high_us / 2U) + 50U,
      this->timing_.min_phase_timeout_us,
      this->timing_.max_phase_timeout_us);
  const uint32_t tuned_low = clamp_value<uint32_t>(
      this->stats_.max_observed_low_us + (this->stats_.max_observed_low_us / 2U) + 50U,
      this->timing_.min_phase_timeout_us,
      this->timing_.max_phase_timeout_us);
  const uint32_t tuned_byte = clamp_value<uint32_t>(
      this->stats_.max_observed_byte_us + (this->stats_.max_observed_byte_us / 2U) + 100U,
      this->timing_.min_byte_timeout_us,
      this->timing_.max_byte_timeout_us);

  this->timing_.sck_high_timeout_us = tuned_high;
  this->timing_.sck_low_timeout_us = tuned_low;
  this->timing_.byte_timeout_us = tuned_byte;
  this->timing_.calibration_complete = true;

  ESP_LOGI(
      TAG,
      "Auto-calibration complete: frame_start_high=%uus sck_high_timeout=%uus sck_low_timeout=%uus byte_timeout=%uus valid_frames=%u",
      this->timing_.frame_start_high_us,
      this->timing_.sck_high_timeout_us,
      this->timing_.sck_low_timeout_us,
      this->timing_.byte_timeout_us,
      static_cast<unsigned>(this->stats_.frames_valid));
}

void MhiTransportLegacy::setup(const MhiTransportConfig &config) {
  this->config_ = config;
  this->timing_ = {};
  this->stats_ = {};
  this->last_observed_stable_high_us_ = 0;
  this->last_observed_max_high_wait_us_ = 0;
  this->last_observed_max_low_wait_us_ = 0;
  this->last_observed_max_byte_us_ = 0;
  this->last_exchange_completed_ = false;

  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.sck_pin));
  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.mosi_pin));
  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.miso_pin));

  gpio_set_direction(static_cast<gpio_num_t>(this->config_.sck_pin), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(this->config_.mosi_pin), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(this->config_.miso_pin), GPIO_MODE_OUTPUT);

  gpio_set_level(static_cast<gpio_num_t>(this->config_.miso_pin), 0);
}

int IRAM_ATTR MhiTransportLegacy::exchange_frame(
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t frame_size,
    uint32_t max_time_ms,
    bool &new_data_packet_received) {
  new_data_packet_received = false;
  this->last_exchange_completed_ = false;
  this->last_observed_stable_high_us_ = 0;
  this->last_observed_max_high_wait_us_ = 0;
  this->last_observed_max_low_wait_us_ = 0;
  this->last_observed_max_byte_us_ = 0;

  const int sck_pin = this->config_.sck_pin;
  const int mosi_pin = this->config_.mosi_pin;
  const int miso_pin = this->config_.miso_pin;

  const uint32_t start_us = mhi_now_us();

  uint32_t sck_high_start_us = mhi_now_us();
  while ((mhi_now_us() - sck_high_start_us) < this->timing_.frame_start_high_us) {
    if (!fast_gpio_read(sck_pin)) {
      sck_high_start_us = mhi_now_us();
    }
    if (this->overall_timed_out_(start_us, max_time_ms)) {
      fast_gpio_write_low(miso_pin);
      this->stats_.timeout_sck_low++;
      return err_msg_timeout_SCK_low;
    }
  }
  this->last_observed_stable_high_us_ = mhi_now_us() - sck_high_start_us;

  for (std::size_t byte_cnt = 0; byte_cnt < frame_size; byte_cnt++) {
    const uint32_t byte_start_us = mhi_now_us();
    uint8_t mosi_byte = 0;
    uint8_t bit_mask = 1;

    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      const uint32_t high_wait_start_us = mhi_now_us();
      while (fast_gpio_read(sck_pin)) {
        const uint32_t elapsed_us = mhi_now_us() - high_wait_start_us;
        if (elapsed_us > this->timing_.sck_high_timeout_us || this->overall_timed_out_(start_us, max_time_ms)) {
          fast_gpio_write_low(miso_pin);
          this->stats_.timeout_sck_high++;
          return err_msg_timeout_SCK_high;
        }
      }
      const uint32_t high_wait_us = mhi_now_us() - high_wait_start_us;
      if (high_wait_us > this->last_observed_max_high_wait_us_) {
        this->last_observed_max_high_wait_us_ = high_wait_us;
      }

      if ((tx_frame[byte_cnt] & bit_mask) != 0) {
        fast_gpio_write_high(miso_pin);
      } else {
        fast_gpio_write_low(miso_pin);
      }

      const uint32_t low_wait_start_us = mhi_now_us();
      while (!fast_gpio_read(sck_pin)) {
        const uint32_t elapsed_us = mhi_now_us() - low_wait_start_us;
        if (elapsed_us > this->timing_.sck_low_timeout_us || this->overall_timed_out_(start_us, max_time_ms)) {
          fast_gpio_write_low(miso_pin);
          this->stats_.timeout_sck_low++;
          return err_msg_timeout_SCK_low;
        }
      }
      const uint32_t low_wait_us = mhi_now_us() - low_wait_start_us;
      if (low_wait_us > this->last_observed_max_low_wait_us_) {
        this->last_observed_max_low_wait_us_ = low_wait_us;
      }

      if (fast_gpio_read(mosi_pin)) {
        mosi_byte = static_cast<uint8_t>(mosi_byte | bit_mask);
      }

      bit_mask = static_cast<uint8_t>(bit_mask << 1);
    }

    if (rx_frame[byte_cnt] != mosi_byte) {
      new_data_packet_received = true;
      rx_frame[byte_cnt] = mosi_byte;
    }

    const uint32_t byte_elapsed_us = mhi_now_us() - byte_start_us;
    if (byte_elapsed_us > this->last_observed_max_byte_us_) {
      this->last_observed_max_byte_us_ = byte_elapsed_us;
    }
    if (byte_elapsed_us > this->timing_.byte_timeout_us) {
      fast_gpio_write_low(miso_pin);
      this->stats_.timeout_byte++;
      return err_msg_timeout_SCK_high;
    }
  }

  fast_gpio_write_low(miso_pin);
  this->last_exchange_completed_ = true;
  return 0;
}

void MhiTransportLegacy::on_frame_result(bool valid_frame, int error_code) {
  this->stats_.frames_total++;

  if (valid_frame) {
    this->stats_.consecutive_errors = 0;
    this->stats_.frames_valid++;

    if (this->last_exchange_completed_) {
      this->update_frame_observation_(
          this->last_observed_stable_high_us_,
          this->last_observed_max_high_wait_us_,
          this->last_observed_max_low_wait_us_,
          this->last_observed_max_byte_us_);

      if (this->timing_.auto_calibrate &&
          !this->timing_.calibration_complete &&
          this->stats_.frames_valid >= this->timing_.valid_frames_needed) {
        this->finalize_calibration_();
      }
    }
  } else {
    this->stats_.consecutive_errors++;

    switch (error_code) {
      case err_msg_invalid_signature:
        this->stats_.frames_invalid_signature++;
        break;
      case err_msg_invalid_checksum:
        this->stats_.frames_invalid_checksum++;
        break;
      default:
        break;
    }

    if (this->timing_.auto_calibrate &&
        this->timing_.calibration_complete &&
        this->stats_.consecutive_errors >= this->timing_.recalibrate_after_consecutive_errors) {
      ESP_LOGW(TAG, "Too many consecutive transport/protocol errors, resetting auto-calibration");
      this->reset_calibration_();
      this->stats_.consecutive_errors = 0;
    }
  }

  this->last_exchange_completed_ = false;
}

}  // namespace mhi
}  // namespace esphome