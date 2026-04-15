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
  const uint32_t start_ms = mhi_now_ms();
  uint32_t observed_edge_count = this->edge_count_;

  while ((mhi_now_ms() - start_ms) <= max_time_ms) {
    const uint32_t remaining_ms = max_time_ms - (mhi_now_ms() - start_ms);
    const TickType_t wait_ticks =
        pdMS_TO_TICKS(remaining_ms > 10 ? 10 : remaining_ms + 1);

    this->waiter_task_ = xTaskGetCurrentTaskHandle();
    (void) ulTaskNotifyTake(pdTRUE, wait_ticks);

    if (this->edge_count_ == observed_edge_count) {
      continue;
    }
    observed_edge_count = this->edge_count_;

    if (gpio_get_level(sck) == 0) {
      continue;
    }

    const uint32_t high_start_ms = mhi_now_ms();
    while ((mhi_now_ms() - high_start_ms) < 5U) {
      if (gpio_get_level(sck) == 0) {
        observed_edge_count = this->edge_count_;
        goto next_candidate;
      }
      if ((mhi_now_ms() - start_ms) > max_time_ms) {
        return false;
      }
    }

    return true;

  next_candidate:
    continue;
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

  const uint32_t start_ms = mhi_now_ms();

  if (!this->wait_for_frame_start_(max_time_ms)) {
    return err_msg_timeout_SCK_low;
  }

  for (std::size_t byte_cnt = 0; byte_cnt < frame_size; byte_cnt++) {
    uint8_t mosi_byte = 0;
    uint8_t bit_mask = 1;

    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      while (gpio_get_level(sck) != 0) {
        if ((mhi_now_ms() - start_ms) > max_time_ms) {
          return err_msg_timeout_SCK_high;
        }
      }

      gpio_set_level(miso, (tx_frame[byte_cnt] & bit_mask) ? 1 : 0);

      while (gpio_get_level(sck) == 0) {
        if ((mhi_now_ms() - start_ms) > max_time_ms) {
          return err_msg_timeout_SCK_low;
        }
      }

      if (gpio_get_level(mosi) != 0) {
        mosi_byte = static_cast<uint8_t>(mosi_byte + bit_mask);
      }

      bit_mask = static_cast<uint8_t>(bit_mask << 1);
    }

    if (rx_frame[byte_cnt] != mosi_byte) {
      new_data_packet_received = true;
      rx_frame[byte_cnt] = mosi_byte;
    }
  }

  gpio_set_level(miso, 0);
  return 0;
}

}  // namespace mhi
}  // namespace esphome