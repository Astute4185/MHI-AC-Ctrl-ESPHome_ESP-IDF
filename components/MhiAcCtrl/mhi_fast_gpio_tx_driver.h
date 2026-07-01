#pragma once

#include <freertos/FreeRTOS.h>

#include <cstddef>
#include <cstdint>

#include "mhi_defs.h"
#include "mhi_tx_driver.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiFastGpioTxConfig {
  uint8_t frame_size_hint{20};
  uint32_t frame_start_idle_ms{10};
  uint32_t max_exchange_time_ms{50};
  bool byte_critical_sections{true};
};

// TX-only clock follower used by the native_spi_rx experiment.
// It does not sample MOSI. It only drives MISO on the AC-provided SCK.
class MhiFastGpioTxDriver final : public IMhiTxDriver {
 public:
  void set_config(const MhiFastGpioTxConfig& config) {
    config_ = config;
  }

  void set_byte_critical_sections(bool enabled) {
    config_.byte_critical_sections = enabled;
  }

  bool setup(const MhiTransportPins& pins) override;
  void loop() override {}
  bool send(const uint8_t* data, std::size_t len) override;

  const char* name() const override {
    return "fast_gpio_tx";
  }
  bool ready() const override {
    return ready_;
  }

 private:
  bool transmit_frame_(const uint8_t* data, std::size_t len);

  MhiTransportPins pins_{};
  MhiFastGpioTxConfig config_{};
  bool ready_{false};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
