#include "mhi_transport_gpio_frame_isr.h"

#include <driver/gpio.h>
#include <esp_err.h>

#include "MHI-AC-Ctrl-core.h"
#include "mhi_time.h"

namespace esphome {
namespace mhi {

void MhiTransportGpioFrameIsr::gpio_isr_handler_(void *arg) {
  auto *self = static_cast<MhiTransportGpioFrameIsr *>(arg);
  self->edge_count_++;

  BaseType_t higher_priority_task_woken = pdFALSE;
  if (self->waiter_task_ != nullptr) {
    vTaskNotifyGiveFromISR(self->waiter_task_, &higher_priority_task_woken);
  }
  if (higher_priority_task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void MhiTransportGpioFrameIsr::setup(const MhiTransportConfig &config) {
  this->config_ = config;
  this->timing_ = {};
  this->timing_.auto_calibrate = false;
  this->stats_ = {};
  this->waiter_task_ = nullptr;
  this->edge_count_ = 0;
  this->isr_registered_ = false;

  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.sck_pin));
  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.mosi_pin));
  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.miso_pin));

  gpio_set_direction(static_cast<gpio_num_t>(this->config_.sck_pin), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(this->config_.mosi_pin), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(this->config_.miso_pin), GPIO_MODE_OUTPUT);

  gpio_set_intr_type(static_cast<gpio_num_t>(this->config_.sck_pin), GPIO_INTR_ANYEDGE);
  gpio_set_level(static_cast<gpio_num_t>(this->config_.miso_pin), 0);

  esp_err_t err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    this->isr_registered_ = false;
    return;
  }

  err = gpio_isr_handler_add(
      static_cast<gpio_num_t>(this->config_.sck_pin),
      &MhiTransportGpioFrameIsr::gpio_isr_handler_,
      this);

  this->isr_registered_ = (err == ESP_OK || err == ESP_ERR_INVALID_STATE);
}

bool MhiTransportGpioFrameIsr::wait_for_frame_start_(uint32_t max_time_ms) {
  const gpio_num_t sck = static_cast<gpio_num_t>(this->config_.sck_pin);
  const uint32_t start_us = mhi_now_us();

  while ((mhi_now_us() - start_us) <= (max_time_ms * 1000U)) {
    this->waiter_task_ = xTaskGetCurrentTaskHandle();
    (void) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    this->waiter_task_ = nullptr;

    if ((mhi_now_us() - start_us) > (max_time_ms * 1000U)) {
      return false;
    }

    uint32_t sck_high_start_us = mhi_now_us();
    while ((mhi_now_us() - sck_high_start_us) < this->timing_.frame_start_high_us) {
      if (gpio_get_level(sck) == 0) {
        sck_high_start_us = mhi_now_us();
      }

      if ((mhi_now_us() - start_us) > (max_time_ms * 1000U)) {
        return false;
      }
    }

    return true;
  }

  return false;
}

int MhiTransportGpioFrameIsr::exchange_frame(
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t frame_size,
    uint32_t max_time_ms,
    bool &new_data_packet_received) {
  new_data_packet_received = false;

  if (!this->isr_registered_) {
    return err_msg_timeout_SCK_low;
  }

  const gpio_num_t sck = static_cast<gpio_num_t>(this->config_.sck_pin);
  const gpio_num_t mosi = static_cast<gpio_num_t>(this->config_.mosi_pin);
  const gpio_num_t miso = static_cast<gpio_num_t>(this->config_.miso_pin);

  const uint32_t start_us = mhi_now_us();

  if (!this->wait_for_frame_start_(max_time_ms)) {
    this->stats_.timeout_sck_low++;
    return err_msg_timeout_SCK_low;
  }

  for (std::size_t byte_cnt = 0; byte_cnt < frame_size; byte_cnt++) {
    const uint32_t byte_start_us = mhi_now_us();
    uint8_t mosi_byte = 0;
    uint8_t bit_mask = 1;

    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      const uint32_t high_wait_start_us = mhi_now_us();
      while (gpio_get_level(sck) != 0) {
        if ((mhi_now_us() - high_wait_start_us) > this->timing_.sck_high_timeout_us ||
            (mhi_now_us() - start_us) > (max_time_ms * 1000U)) {
          gpio_set_level(miso, 0);
          this->stats_.timeout_sck_high++;
          return err_msg_timeout_SCK_high;
        }
      }

      gpio_set_level(miso, (tx_frame[byte_cnt] & bit_mask) ? 1 : 0);

      const uint32_t low_wait_start_us = mhi_now_us();
      while (gpio_get_level(sck) == 0) {
        if ((mhi_now_us() - low_wait_start_us) > this->timing_.sck_low_timeout_us ||
            (mhi_now_us() - start_us) > (max_time_ms * 1000U)) {
          gpio_set_level(miso, 0);
          this->stats_.timeout_sck_low++;
          return err_msg_timeout_SCK_low;
        }
      }

      if (gpio_get_level(mosi) != 0) {
        mosi_byte = static_cast<uint8_t>(mosi_byte | bit_mask);
      }

      bit_mask = static_cast<uint8_t>(bit_mask << 1);
    }

    if (rx_frame[byte_cnt] != mosi_byte) {
      new_data_packet_received = true;
      rx_frame[byte_cnt] = mosi_byte;
    }

    if ((mhi_now_us() - byte_start_us) > this->timing_.byte_timeout_us) {
      gpio_set_level(miso, 0);
      this->stats_.timeout_byte++;
      return err_msg_timeout_SCK_high;
    }
  }

  gpio_set_level(miso, 0);
  return 0;
}

void MhiTransportGpioFrameIsr::on_frame_result(bool valid_frame, int error_code) {
  this->stats_.frames_total++;
  if (valid_frame) {
    this->stats_.frames_valid++;
    this->stats_.consecutive_errors = 0;
    return;
  }

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
}

}  // namespace mhi
}  // namespace esphome