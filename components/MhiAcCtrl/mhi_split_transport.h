#pragma once

#ifdef USE_ESP_IDF
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#else
#include <mutex>
#endif

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "mhi_rx_driver.h"
#include "mhi_transport.h"
#include "mhi_tx_driver.h"

namespace esphome {
namespace mhi_ac_ctrl {

// Adapts an RX driver and a TX driver into one complete transport strategy.
// The marker-armed TX scheduler is intentionally kept here so the manager no
// longer needs to understand split-driver timing behaviour.
class MhiSplitTransport final : public IMhiTransport {
 public:
  void bind(IMhiRxDriver* rx, IMhiTxDriver* tx, bool supports_classified_worker, bool uses_bus_marker);

  void set_pins(int sck_pin, int mosi_pin, int miso_pin) {
    pins_ = {sck_pin, mosi_pin, miso_pin};
  }

  MhiTransportResult setup() override;
  void loop() override;
  void shutdown() override;

  std::size_t read(uint8_t* dst, std::size_t max_len) override;
  bool queue_tx(const MhiTxEnvelope& envelope) override;
  bool take_tx_completion(MhiTxCompletion& completion) override;

  void set_active_mode(bool enabled) override;
  bool active_mode() const override {
    return active_mode_enabled_.load(std::memory_order_acquire);
  }

  bool has_pending_tx() const override;
  bool flush_tx_on_bus_marker() override;

  void set_auto_tx_flush(bool enabled) override {
    auto_tx_flush_ = enabled;
  }
  bool auto_tx_flush() const override {
    return auto_tx_flush_;
  }

  void set_rx_byte_critical_sections(bool enabled) override;
  bool rx_byte_critical_sections() const override;

  const char* name() const override {
    return "split";
  }
  const char* rx_name() const override;
  const char* tx_name() const override;
  bool rx_ready() const override {
    return rx_ready_;
  }
  bool tx_ready() const override {
    return tx_ready_;
  }
  MhiTransportCapabilities capabilities() const override;
  MhiTransportHealth health() const override;
  MhiTransportErrorDetail last_error() const override {
    return last_error_;
  }

  uint32_t completed_tx_frames() const override {
    return completed_tx_frames_.load(std::memory_order_relaxed);
  }
  uint32_t tx_failures() const override {
    return tx_failures_.load(std::memory_order_relaxed);
  }
  std::size_t tx_completion_queue_depth() const override;
  std::size_t tx_completion_queue_high_water() const override;
  uint32_t tx_completion_queue_dropped() const override;

 private:
  static uint32_t elapsed_us_(uint32_t now_us, uint32_t then_us) {
    return static_cast<uint32_t>(now_us - then_us);
  }

  void lock_tx_() const;
  void unlock_tx_() const;

  void reset_tx_state_();
  void clear_tx_for_active_mode_();
  void queue_pending_tx_(const MhiTxEnvelope& envelope);
  bool pending_tx_available_() const;
  void clear_pending_tx_();
  bool tx_disabled_() const;

  MhiTransportPins pins_{};
  IMhiRxDriver* rx_{nullptr};
  IMhiTxDriver* tx_{nullptr};

  bool supports_classified_worker_{false};
  bool uses_bus_marker_{false};
  bool rx_ready_{false};
  bool tx_ready_{false};
  bool auto_tx_flush_{true};
  std::atomic<bool> active_mode_enabled_{true};

#ifdef USE_ESP_IDF
  mutable portMUX_TYPE tx_mux_ = portMUX_INITIALIZER_UNLOCKED;
#else
  mutable std::mutex tx_mux_{};
#endif
  MhiTxEnvelope pending_tx_envelope_{};
  MhiTxCompletionQueue<8U> tx_completions_{};
  bool pending_tx_{false};
  bool tx_in_progress_{false};
  uint32_t pending_tx_generation_{0U};
  uint32_t pending_tx_queued_after_marker_sequence_{0U};
  uint32_t last_consumed_bus_marker_sequence_{0U};
  uint32_t last_stale_bus_marker_sequence_{0U};
  uint32_t tx_backoff_until_ms_{0U};

  uint32_t tx_marker_arm_max_age_us_{3000U};
  uint32_t tx_failure_backoff_ms_{250U};

  std::atomic<uint32_t> completed_tx_frames_{0U};
  std::atomic<uint32_t> tx_failures_{0U};
  MhiTransportHealth health_{};
  MhiTransportErrorDetail last_error_{};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
