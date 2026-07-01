#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mhi_defs.h"
#include "mhi_rx_driver.h"

#ifdef USE_ESP_IDF
#include <driver/spi_slave.h>
#endif

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiNativeSpiRxConfig {
  uint8_t frame_size_hint{20};
};

class MhiNativeSpiRxDriver final : public IMhiRxDriver {
 public:
  void set_config(const MhiNativeSpiRxConfig& config) {
    config_ = config;
  }

  bool setup(const MhiTransportPins& pins) override;
  void loop() override;
  std::size_t read(uint8_t* dst, std::size_t max_len) override;

  const char* name() const override {
    return "native_spi_rx";
  }
  bool ready() const override {
    return ready_;
  }

  uint32_t completed_transactions() const {
    return completed_transactions_;
  }
  uint32_t zero_length_transactions() const {
    return zero_length_transactions_;
  }
  uint32_t queue_errors() const {
    return queue_errors_;
  }
  uint32_t dropped_bytes() const {
    return dropped_bytes_;
  }

 private:
  static constexpr std::size_t kQueueDepth = 4U;
  static constexpr std::size_t kCaptureBytes = kMhiMaxFrameBytes;
  static constexpr std::size_t kRingBytes = kMhiMaxFrameBytes * 8U;

#ifdef USE_ESP_IDF
  struct TransactionSlot {
    spi_slave_transaction_t transaction{};
    alignas(4) uint8_t rx_buffer[kCaptureBytes]{};
  };

  bool queue_slot_(TransactionSlot& slot);
  void push_bytes_(const uint8_t* data, std::size_t len);
  std::size_t ring_available_() const;
#endif

  MhiNativeSpiRxConfig config_{};
  MhiTransportPins pins_{};
  bool ready_{false};

  uint32_t completed_transactions_{0};
  uint32_t zero_length_transactions_{0};
  uint32_t queue_errors_{0};
  uint32_t dropped_bytes_{0};
  uint32_t last_diag_log_ms_{0};

#ifdef USE_ESP_IDF
  spi_host_device_t host_{SPI2_HOST};
  std::array<TransactionSlot, kQueueDepth> slots_{};
  std::array<uint8_t, kRingBytes> ring_{};
  std::size_t ring_head_{0};
  std::size_t ring_tail_{0};
#endif
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
