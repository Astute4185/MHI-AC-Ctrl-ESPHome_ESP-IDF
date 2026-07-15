#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_tx_driver.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiNullTxDriver final : public IMhiTxDriver {
 public:
  bool setup(const MhiTransportPins& pins) override;
  void loop() override {}
  bool send(const uint8_t* data, std::size_t len) override;

  const char* name() const override {
    return "none";
  }
  bool ready() const override {
    return ready_;
  }

 private:
  bool ready_{false};
  uint32_t dropped_frames_{0};
  uint32_t last_log_ms_{0};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
