#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_transport_pins.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiBusMarker {
  bool valid{false};
  uint32_t sequence{0};
  uint32_t frame_end_us{0};
  std::size_t frame_len{0};
};

class IMhiRxDriver {
 public:
  virtual bool setup(const MhiTransportPins& pins) = 0;
  virtual void loop() = 0;
  virtual void shutdown() {}

  // Reads received MOSI bytes into dst.
  virtual std::size_t read(uint8_t* dst, std::size_t max_len) = 0;

  // Returns the latest observed bus/frame boundary.
  // TX scheduling uses this as the safe marker for response-window attempts.
  virtual MhiBusMarker bus_marker() const {
    return {};
  }

  virtual const char* name() const = 0;
  virtual bool ready() const = 0;

  virtual ~IMhiRxDriver() = default;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
