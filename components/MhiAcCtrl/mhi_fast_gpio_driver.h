#pragma once

#include <freertos/FreeRTOS.h>

#include <cstddef>
#include <cstdint>

#include "mhi_defs.h"
#include "mhi_rx_driver.h"
#include "mhi_tx_driver.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiFastGpioConfig {
  uint8_t frame_size_hint{20};
  uint32_t frame_start_idle_ms{10};
  uint32_t max_exchange_time_ms{50};
  bool rx_byte_critical_sections{true};
};

enum class MhiFastGpioExchangeStatus : int {
  OK = 0,
  NOT_READY = -1,
  BAD_ARGUMENT = -2,
  TIMEOUT_SCK_HIGH = -3,
  TIMEOUT_SCK_LOW = -4,
};

class MhiFastGpioDriver final : public IMhiRxDriver, public IMhiTxDriver {
 public:
  void set_config(const MhiFastGpioConfig& config) {
    config_ = config;
  }

  void set_rx_byte_critical_sections(bool enabled) {
    config_.rx_byte_critical_sections = enabled;
  }

  bool rx_byte_critical_sections() const {
    return config_.rx_byte_critical_sections;
  }

  bool setup(const MhiTransportPins& pins) override;
  void loop() override {}

  std::size_t read(uint8_t* dst, std::size_t max_len) override;
  bool send(const uint8_t* data, std::size_t len) override;

  const char* name() const override {
    return "fast_gpio";
  }
  bool ready() const override {
    return ready_;
  }

 private:
  MhiFastGpioExchangeStatus exchange_frame(uint8_t* rx_frame, std::size_t rx_capacity, std::size_t& bytes_received);

  MhiTransportPins pins_{};
  MhiFastGpioConfig config_{};

  uint8_t tx_frame_[kMhiMaxFrameBytes]{};
  std::size_t tx_len_{kMhiFrame20Bytes};
  portMUX_TYPE tx_mux_ = portMUX_INITIALIZER_UNLOCKED;

  bool tx_staged_{false};
  bool ready_{false};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome