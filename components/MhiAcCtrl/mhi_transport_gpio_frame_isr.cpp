#include "mhi_transport_gpio_frame_isr.h"

#include <algorithm>
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
  const uint32_t start_ms = mhi_now_ms();

  while ((mhi_now_ms() - start_ms) <= max_time_ms) {
    this->waiter_task_ = xTaskGetCurrentTaskHandle();
    (void) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    this->waiter_task_ = nullptr;

    if ((mhi_now_ms() - start_ms) > max_time_ms) {
      return false;
    }

    uint32_t sck_high_start_ms = mhi_now_ms();
    while ((mhi_now_ms() - sck_high_start_ms) < this->config_.frame_start_idle_ms) {
      if (gpio_get_level(sck) == 0) {
        sck_high_start_ms = mhi_now_ms();
      }

      if ((mhi_now_ms() - start_ms) > max_time_ms) {
        return false;
      }
    }

    return true;
  }

  return false;
}

std::size_t MhiTransportGpioFrameIsr::determine_target_frame_bytes_(std::size_t rx_capacity) const {
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

MhiFrameExchangeResult MhiTransportGpioFrameIsr::exchange_frame(
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t rx_capacity,
    uint32_t max_time_ms) {
  MhiFrameExchangeResult result{};

  if (!this->isr_registered_) {
    result.status = err_msg_timeout_SCK_low;
    return result;
  }

  const std::size_t target_frame_size = this->determine_target_frame_bytes_(rx_capacity);
  if (target_frame_size == 0U) {
    result.status = err_msg_timeout_SCK_low;
    return result;
  }

  const gpio_num_t sck = static_cast<gpio_num_t>(this->config_.sck_pin);
  const gpio_num_t mosi = static_cast<gpio_num_t>(this->config_.mosi_pin);
  const gpio_num_t miso = static_cast<gpio_num_t>(this->config_.miso_pin);

  const uint32_t start_ms = mhi_now_ms();

  if (!this->wait_for_frame_start_(max_time_ms)) {
    result.status = err_msg_timeout_SCK_low;
    return result;
  }

  for (std::size_t byte_cnt = 0; byte_cnt < target_frame_size; byte_cnt++) {
    uint8_t mosi_byte = 0;
    uint8_t bit_mask = 1;

    for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
      while (gpio_get_level(sck) != 0) {
        if ((mhi_now_ms() - start_ms) > max_time_ms) {
          result.status = err_msg_timeout_SCK_high;
          gpio_set_level(miso, 0);
          return result;
        }
      }

      gpio_set_level(miso, (tx_frame[byte_cnt] & bit_mask) ? 1 : 0);

      while (gpio_get_level(sck) == 0) {
        if ((mhi_now_ms() - start_ms) > max_time_ms) {
          result.status = err_msg_timeout_SCK_low;
          gpio_set_level(miso, 0);
          return result;
        }
      }

      if (gpio_get_level(mosi) != 0) {
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
  }

  gpio_set_level(miso, 0);

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
