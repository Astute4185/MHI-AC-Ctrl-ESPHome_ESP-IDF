#pragma once

#include "mhi_transport_result.h"

namespace esphome {
namespace mhi_ac_ctrl {

// Optional listener used by the transport manager to let the owning runtime
// quiesce command/protocol state before changing transports. The listener is
// non-owning and callbacks always run from normal component context.
class IMhiTransportTransitionListener {
 public:
  virtual void on_transport_switch_begin(const MhiTransportErrorDetail& reason) = 0;
  virtual void on_transport_recovery_ready() = 0;
  virtual void on_transport_safe_mode(const MhiTransportErrorDetail& reason) = 0;

  virtual ~IMhiTransportTransitionListener() = default;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
