#pragma once

#include <cstddef>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "mhi_transport.h"

namespace esphome {
namespace mhi {

class MhiTransportGpioFrameIsr : public MhiTransport {
 public:
  void setup(const MhiTransportConfig &config) override;

  MhiFrameExchangeResult exchange_frame(
      const uint8_t *tx_frame,
      uint8_t *rx_frame,
      std::size_t rx_capacity,
      uint32_t max_time_ms) override;

 private:
  static void gpio_isr_handler_(void *arg);
  bool wait_for_frame_start_(uint32_t max_time_ms);
  std::size_t determine_target_frame_bytes_(std::size_t rx_capacity) const;

  MhiTransportConfig config_{};
  TaskHandle_t waiter_task_{nullptr};
  volatile uint32_t edge_count_{0};
  bool isr_registered_{false};
};

}  // namespace mhi
}  // namespace esphome
