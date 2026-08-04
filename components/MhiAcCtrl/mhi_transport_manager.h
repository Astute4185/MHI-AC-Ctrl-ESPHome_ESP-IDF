#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_defs.h"
#include "mhi_diag.h"
#include "mhi_transport.h"
#include "mhi_transport_transition.h"

namespace esphome {
namespace mhi_ac_ctrl {

// Generic, non-owning runtime coordinator for one configured primary transport
// and an optional internal recovery transport. Concrete transport construction,
// pin configuration, and driver-specific tuning are owned by ESPHome codegen.
class MhiTransportManager {
 public:
  void set_primary(IMhiTransport* transport);
  void set_recovery(IMhiTransport* transport);
  void set_transition_listener(IMhiTransportTransitionListener* listener) {
    transition_listener_ = listener;
  }

  void set_diagnostics(MhiDiagnostics* diagnostics) {
    diagnostics_ = diagnostics;
  }

  bool setup();
  void loop();
  void shutdown();

  std::size_t read_rx(uint8_t* dst, std::size_t max_len);
  bool queue_tx(const MhiTxEnvelope& envelope);
  bool take_tx_completion(MhiTxCompletion& completion);
  bool has_pending_tx() const;
  bool flush_tx_on_bus_marker();

  std::size_t tx_completion_queue_depth() const;
  std::size_t tx_completion_queue_high_water() const;
  uint32_t tx_completion_queue_dropped() const;
  std::size_t rx_queue_depth() const;
  std::size_t rx_queue_high_water() const;
  uint32_t rx_queue_overwritten() const;

  void set_auto_tx_flush(bool enabled);
  bool auto_tx_flush() const;
  void set_rx_byte_critical_sections(bool enabled);
  bool rx_byte_critical_sections() const;
  bool tx_uses_bus_marker() const;
  bool tx_uses_bus_window() const {
    return this->tx_uses_bus_marker();
  }

  const char* rx_name() const;
  const char* tx_name() const;
  const char* primary_name() const {
    return primary_ == nullptr ? "none" : primary_->name();
  }
  const char* recovery_name() const {
    return recovery_ == nullptr ? "none" : recovery_->name();
  }
  bool recovery_active() const {
    return recovery_active_;
  }
  bool recovery_attempted() const {
    return recovery_attempted_;
  }
  bool safe_mode() const {
    return safe_mode_;
  }
  MhiTransportState state() const;

  bool rx_ready() const {
    return active_ != nullptr && active_->rx_ready();
  }
  bool tx_ready() const {
    return active_ != nullptr && active_->tx_ready();
  }
  bool rx_supports_classified_worker() const {
    return this->rx_ready() && active_->capabilities().supports_classified_worker;
  }
  MhiTransportHealth transport_health() const {
    return active_ == nullptr ? MhiTransportHealth{} : active_->health();
  }
  MhiTransportErrorDetail last_transport_error() const {
    return last_transport_error_;
  }
  MhiTransportErrorDetail primary_failure() const {
    return primary_failure_;
  }
  MhiTransportErrorDetail recovery_failure() const {
    return recovery_failure_;
  }

 private:
  bool activate_recovery_(const MhiTransportResult& primary_result);
  bool enter_safe_mode_(const MhiTransportErrorDetail& reason);
  void update_transport_diagnostics_();
  void reset_transport_diagnostic_cursors_();
  void publish_driver_diagnostics_();

  IMhiTransport* primary_{nullptr};
  IMhiTransport* recovery_{nullptr};
  IMhiTransport* active_{nullptr};
  IMhiTransportTransitionListener* transition_listener_{nullptr};
  bool recovery_active_{false};
  bool recovery_attempted_{false};
  bool safe_mode_{false};
  MhiTransportState state_{MhiTransportState::STOPPED};

  bool auto_tx_flush_{true};
  bool rx_byte_critical_sections_{true};

  uint32_t last_transport_tx_completed_{0U};
  uint32_t last_transport_tx_failures_{0U};
  MhiTransportErrorDetail last_transport_error_{};
  MhiTransportErrorDetail primary_failure_{};
  MhiTransportErrorDetail recovery_failure_{};
  MhiDiagnostics* diagnostics_{nullptr};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
