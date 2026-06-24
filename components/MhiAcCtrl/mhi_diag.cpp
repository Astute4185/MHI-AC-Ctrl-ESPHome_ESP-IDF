#include "mhi_diag.h"

namespace esphome {
namespace mhi_ac_ctrl {

void MhiDiagnostics::set_rx_driver_name(const char* name) {
  rx_driver_name_ = name == nullptr ? "none" : name;
}

void MhiDiagnostics::set_tx_driver_name(const char* name) {
  tx_driver_name_ = name == nullptr ? "none" : name;
}

void MhiDiagnostics::set_rx_driver_ready(bool ready) {
  rx_driver_ready_ = ready;
}

void MhiDiagnostics::set_tx_driver_ready(bool ready) {
  tx_driver_ready_ = ready;
}

MhiDiagSnapshot MhiDiagnostics::snapshot(uint32_t now_ms) const {
  MhiDiagSnapshot out{};

  out.stats = stats_.snapshot();

  out.rx_driver_name = rx_driver_name_;
  out.tx_driver_name = tx_driver_name_;

  out.rx_driver_ready = rx_driver_ready_;
  out.tx_driver_ready = tx_driver_ready_;

  out.now_ms = now_ms;

  out.last_valid_frame_age_ms = age_or_zero(now_ms, out.stats.last_valid_frame_ms);
  out.last_rx_byte_age_ms = age_or_zero(now_ms, out.stats.last_rx_byte_ms);
  out.last_tx_frame_age_ms = age_or_zero(now_ms, out.stats.last_tx_frame_ms);
  out.last_tx_command_frame_age_ms = age_or_zero(now_ms, out.stats.last_tx_command_frame_ms);
  out.last_unsupported_command_age_ms = age_or_zero(now_ms, out.stats.last_unsupported_command_ms);
  out.last_command_confirmation_age_ms = age_or_zero(now_ms, out.stats.last_command_confirmation_ms);
  out.last_command_confirmation_timeout_age_ms = age_or_zero(now_ms, out.stats.last_command_confirmation_timeout_ms);
  out.last_loop_over_budget_age_ms = age_or_zero(now_ms, out.stats.last_loop_over_budget_ms);
  out.last_rx_worker_frame_age_ms = age_or_zero(now_ms, out.stats.last_rx_worker_frame_ms);
  out.last_rx_worker_drained_frame_age_ms = age_or_zero(now_ms, out.stats.last_rx_worker_drained_frame_ms);
  out.last_rx_worker_queue_overflow_age_ms = age_or_zero(now_ms, out.stats.last_rx_worker_queue_overflow_ms);
  out.last_rx_worker_no_frame_window_age_ms = age_or_zero(now_ms, out.stats.last_rx_worker_no_frame_window_ms);
  out.last_rx_worker_stall_age_ms = age_or_zero(now_ms, out.stats.last_rx_worker_stall_ms);
  out.last_rx_worker_not_draining_age_ms = age_or_zero(now_ms, out.stats.last_rx_worker_not_draining_ms);

  return out;
}

uint32_t MhiDiagnostics::age_or_zero(uint32_t now_ms, uint32_t event_ms) {
  if (event_ms == 0U || now_ms < event_ms) {
    return 0U;
  }

  return now_ms - event_ms;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome