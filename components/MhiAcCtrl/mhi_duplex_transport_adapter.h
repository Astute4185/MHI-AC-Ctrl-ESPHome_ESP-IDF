#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "mhi_duplex_transport.h"
#include "mhi_transport.h"

namespace esphome {
namespace mhi_ac_ctrl {

// Adapts an existing integrated duplex backend to the unified transport
// contract without changing its transaction ownership or timing behaviour.
class MhiDuplexTransportAdapter final : public IMhiTransport {
 public:
  void bind(IMhiDuplexTransport* backend, bool supports_classified_worker = true);

  void set_pins(int sck_pin, int mosi_pin, int miso_pin) {
    pins_ = {sck_pin, mosi_pin, miso_pin};
  }

  MhiTransportResult setup() override;
  void loop() override;
  void shutdown() override;

  std::size_t read(uint8_t* dst, std::size_t max_len) override;
  bool queue_tx(const MhiTxEnvelope& envelope) override;
  bool take_tx_completion(MhiTxCompletion& completion) override;
  MhiTxReplaceResult replace_pending_command(uint32_t expected_generation, const MhiTxEnvelope& replacement) override;

  void set_active_mode(bool enabled) override;
  bool active_mode() const override {
    return active_mode_enabled_.load(std::memory_order_acquire);
  }

  bool has_pending_tx() const override {
    return false;
  }
  bool flush_tx_on_bus_marker() override {
    return false;
  }

  void set_auto_tx_flush(bool enabled) override {
    (void)enabled;
  }
  bool auto_tx_flush() const override {
    return false;
  }

  void set_rx_byte_critical_sections(bool enabled) override {
    (void)enabled;
  }
  bool rx_byte_critical_sections() const override {
    return false;
  }

  const char* name() const override;
  const char* rx_name() const override;
  const char* tx_name() const override;
  bool rx_ready() const override;
  bool tx_ready() const override;
  MhiTransportCapabilities capabilities() const override;
  MhiTransportHealth health() const override;
  MhiTransportErrorDetail last_error() const override {
    return last_error_;
  }

  uint32_t completed_tx_frames() const override;
  uint32_t tx_failures() const override;
  std::size_t tx_completion_queue_depth() const override;
  std::size_t tx_completion_queue_high_water() const override;
  uint32_t tx_completion_queue_dropped() const override;
  std::size_t rx_queue_depth() const override;
  std::size_t rx_queue_high_water() const override;
  uint32_t rx_queue_overwritten() const override;

 private:
  MhiTransportPins pins_{};
  IMhiDuplexTransport* backend_{nullptr};
  bool supports_classified_worker_{true};
  std::atomic<uint32_t> staging_failures_{0U};
  std::atomic<bool> active_mode_enabled_{true};
  MhiTransportHealth health_{};
  MhiTransportErrorDetail last_error_{};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
