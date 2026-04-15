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

  int exchange_frame(
      const uint8_t *tx_frame,
      uint8_t *rx_frame,
      std::size_t frame_size,
      uint32_t max_time_ms,
      bool &new_data_packet_received) override;

  void on_frame_result(bool valid_frame, int error_code) override;

  const MhiTimingProfile &timing_profile() const override { return this->timing_; }
  const MhiTransportStats &stats() const override { return this->stats_; }

 private:
  static void gpio_isr_handler_(void *arg);
  bool wait_for_frame_start_(uint32_t max_time_ms);

  MhiTransportConfig config_{};
  MhiTimingProfile timing_{};
  MhiTransportStats stats_{};
  TaskHandle_t waiter_task_{nullptr};
  volatile uint32_t edge_count_{0};
  bool isr_registered_{false};
};

}  // namespace mhi
}  // namespace esphome