#pragma once

#include <freertos/FreeRTOS.h>

#include <cstddef>
#include <cstdint>

#include "mhi_defs.h"
#include "mhi_rx_driver.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiFastGpioRxConfig {
  uint8_t frame_size_hint{20};
  uint32_t frame_start_idle_ms{10};
  uint32_t max_exchange_time_ms{50};
  bool byte_critical_sections{true};
};

enum class MhiFastGpioRxStatus : int {
  OK = 0,
  NOT_READY = -1,
  BAD_ARGUMENT = -2,
  TIMEOUT_SCK_HIGH = -3,
  TIMEOUT_SCK_LOW = -4,
};

class MhiFastGpioRxDriver final : public IMhiRxDriver {
 public:
  void set_config(const MhiFastGpioRxConfig& config) {
    config_ = config;
  }

  void set_byte_critical_sections(bool enabled) {
    config_.byte_critical_sections = enabled;
  }

  bool byte_critical_sections() const {
    return config_.byte_critical_sections;
  }

  bool setup(const MhiTransportPins& pins) override;
  void loop() override {}

  std::size_t read(uint8_t* dst, std::size_t max_len) override;
  MhiBusMarker bus_marker() const override;

  const char* name() const override {
    return "fast_gpio_rx";
  }
  bool ready() const override {
    return ready_;
  }

 private:
  MhiFastGpioRxStatus read_frame_(uint8_t* rx_frame, std::size_t rx_capacity, std::size_t& bytes_received);
  void update_bus_marker_(std::size_t frame_len);

  MhiTransportPins pins_{};
  MhiFastGpioRxConfig config_{};

  bool ready_{false};
  MhiBusMarker marker_{};
  portMUX_TYPE marker_mux_ = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
