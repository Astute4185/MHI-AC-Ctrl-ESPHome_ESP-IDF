#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_transport_pins.h"
#include "mhi_tx_contract.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiTransportCapabilities {
  bool integrated_duplex{false};
  bool uses_bus_marker{false};
  bool supports_classified_worker{false};
  bool supports_rx_byte_critical_sections{false};
};

// Unified operational transport contract used by the manager.
//
// Concrete hardware drivers may remain split into RX and TX implementations,
// but the rest of the component sees one complete transport strategy.
class IMhiTransport {
 public:
  virtual bool setup(const MhiTransportPins& pins) = 0;
  virtual void loop() = 0;
  virtual void shutdown() = 0;

  virtual std::size_t read(uint8_t* dst, std::size_t max_len) = 0;
  virtual bool queue_tx(const MhiTxEnvelope& envelope) = 0;
  virtual bool take_tx_completion(MhiTxCompletion& completion) = 0;

  virtual bool has_pending_tx() const = 0;
  virtual bool flush_tx_on_bus_marker() = 0;

  virtual void set_auto_tx_flush(bool enabled) = 0;
  virtual bool auto_tx_flush() const = 0;

  virtual void set_rx_byte_critical_sections(bool enabled) = 0;
  virtual bool rx_byte_critical_sections() const = 0;

  virtual const char* name() const = 0;
  virtual const char* rx_name() const = 0;
  virtual const char* tx_name() const = 0;
  virtual bool rx_ready() const = 0;
  virtual bool tx_ready() const = 0;
  virtual MhiTransportCapabilities capabilities() const = 0;

  // Monotonic counters allow the manager to feed common diagnostics without
  // coupling transport implementations to MhiDiagnostics.
  virtual uint32_t completed_tx_frames() const {
    return 0U;
  }
  virtual uint32_t tx_failures() const {
    return 0U;
  }
  virtual std::size_t tx_completion_queue_depth() const {
    return 0U;
  }
  virtual std::size_t tx_completion_queue_high_water() const {
    return 0U;
  }
  virtual uint32_t tx_completion_queue_dropped() const {
    return 0U;
  }
  virtual std::size_t rx_queue_depth() const {
    return 0U;
  }
  virtual std::size_t rx_queue_high_water() const {
    return 0U;
  }
  virtual uint32_t rx_queue_overwritten() const {
    return 0U;
  }

  virtual ~IMhiTransport() = default;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
