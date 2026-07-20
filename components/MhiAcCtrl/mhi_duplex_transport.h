#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_transport_pins.h"

namespace esphome {
namespace mhi_ac_ctrl {

// Combined full-duplex transport for hardware backends where RX and TX share
// one peripheral, one transaction boundary, and one set of buffers.
class IMhiDuplexTransport {
 public:
  virtual bool setup(const MhiTransportPins& pins) = 0;
  virtual void loop() = 0;
  virtual void shutdown() {}

  // Reads one completed MOSI frame into dst.
  virtual std::size_t read(uint8_t* dst, std::size_t max_len) = 0;

  // Stages one MISO frame for the next available bus transaction. A later
  // stage may replace an older frame that has not yet been handed to hardware.
  virtual bool send(const uint8_t* data, std::size_t len) = 0;

  virtual const char* name() const = 0;
  virtual bool ready() const = 0;

  // Monotonic transport counters used by the manager to feed common
  // diagnostics without coupling the backend to MhiDiagnostics.
  virtual uint32_t completed_tx_frames() const {
    return 0U;
  }
  virtual uint32_t tx_failures() const {
    return 0U;
  }

  virtual ~IMhiDuplexTransport() = default;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
