#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_transport_pins.h"

namespace esphome {
namespace mhi_ac_ctrl {

class IMhiRxDriver {
 public:
  virtual bool setup(const MhiTransportPins& pins) = 0;
  virtual void loop() = 0;

  // Reads received MOSI bytes into dst.
  //
  // For FastGPIO, this performs the AC-clocked exchange:
  // - waits for SCK
  // - drives staged MISO bits
  // - samples MOSI bits
  virtual std::size_t read(uint8_t* dst, std::size_t max_len) = 0;

  virtual const char* name() const = 0;
  virtual bool ready() const = 0;

  virtual ~IMhiRxDriver() = default;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome