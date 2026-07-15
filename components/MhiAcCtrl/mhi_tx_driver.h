#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_transport_pins.h"

namespace esphome {
namespace mhi_ac_ctrl {

class IMhiTxDriver {
 public:
  virtual bool setup(const MhiTransportPins& pins) = 0;
  virtual void loop() = 0;

  // Sends or stages a MISO frame.
  //
  // For FastGPIO this stages the frame. It is actually clocked out later while
  // RX is reading MOSI from the AC clock.
  virtual bool send(const uint8_t* data, std::size_t len) = 0;

  virtual const char* name() const = 0;
  virtual bool ready() const = 0;

  virtual ~IMhiTxDriver() = default;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome