#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstddef>
#include <cstdint>

#include "esphome/core/hal.h"
#include "mhi_defs.h"
#include "mhi_rx_frame_queue.h"
#include "mhi_stats.h"
#include "mhi_transport_manager.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiRxWorker {
 public:
  void configure(MhiTransportManager* transport, MhiStats* stats, uint8_t frame_size) {
    transport_ = transport;
    stats_ = stats;
    frame_size_ = frame_size == 33U ? kMhiFrame33Bytes : kMhiFrame20Bytes;
  }

  bool start() {
    if (task_ != nullptr) {
      return true;
    }

    if (transport_ == nullptr) {
      return false;
    }

    running_ = true;

    const BaseType_t ok = xTaskCreatePinnedToCore(&MhiRxWorker::task_entry_, "mhi_rx_worker", 6144, this, 4, &task_, 0);

    if (ok != pdPASS) {
      running_ = false;
      task_ = nullptr;
      return false;
    }

    return true;
  }

  void stop() {
    running_ = false;
  }

  bool active() const {
    return task_ != nullptr && running_;
  }

  bool pop_frame(MhiFrameBuffer& frame) {
    portENTER_CRITICAL(&queue_mux_);
    const bool ok = queue_.pop(frame);
    const std::size_t depth = queue_.size();
    portEXIT_CRITICAL(&queue_mux_);

    if (ok && stats_ != nullptr) {
      stats_->on_rx_worker_frame_drained(static_cast<uint32_t>(depth), millis());
    }

    return ok;
  }

  uint32_t queue_depth() const {
    portENTER_CRITICAL(&queue_mux_);
    const uint32_t depth = static_cast<uint32_t>(queue_.size());
    portEXIT_CRITICAL(&queue_mux_);

    return depth;
  }

 private:
  static constexpr uint32_t k_startup_delay_ms_ = 5000U;

  static void task_entry_(void* arg) {
    auto* worker = static_cast<MhiRxWorker*>(arg);

    if (worker != nullptr) {
      worker->task_loop_();
    }

    vTaskDelete(nullptr);
  }

  void task_loop_() {
    uint8_t buffer[kMhiMaxFrameBytes]{};

    const uint32_t started_ms = millis();
    while (running_ && (millis() - started_ms) < k_startup_delay_ms_) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    while (running_) {
      const uint32_t start_us = micros();
      const std::size_t len = transport_->read_rx(buffer, frame_size_);
      const uint32_t elapsed_us = static_cast<uint32_t>(micros() - start_us);
      const uint32_t now = millis();

      if (stats_ != nullptr) {
        stats_->on_rx_worker_timing(elapsed_us, now);
      }

      if (len == 0U) {
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }

      this->push_frame_(buffer, len, now);

      // The worker is intentionally high priority for sampling integrity, so
      // yield after each completed frame to avoid monopolising the scheduler.
      taskYIELD();
    }

    task_ = nullptr;
  }

  void push_frame_(const uint8_t* data, std::size_t len, uint32_t now_ms) {
    portENTER_CRITICAL(&queue_mux_);
    const bool queued = queue_.push(data, len);
    const std::size_t depth = queue_.size();
    portEXIT_CRITICAL(&queue_mux_);

    if (stats_ == nullptr) {
      return;
    }

    if (queued) {
      stats_->on_rx_worker_frame_queued(static_cast<uint32_t>(depth), now_ms);
    } else {
      stats_->on_rx_worker_queue_overflow(static_cast<uint32_t>(depth), now_ms);
    }
  }

  MhiTransportManager* transport_{nullptr};
  MhiStats* stats_{nullptr};

  uint8_t frame_size_{kMhiFrame20Bytes};

  MhiRxFrameQueue queue_{};
  mutable portMUX_TYPE queue_mux_ = portMUX_INITIALIZER_UNLOCKED;

  TaskHandle_t task_{nullptr};
  volatile bool running_{false};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
