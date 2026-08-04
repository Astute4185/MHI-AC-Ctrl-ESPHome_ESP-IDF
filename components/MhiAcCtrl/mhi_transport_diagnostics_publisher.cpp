#include "mhi_transport_diagnostics_publisher.h"

#include <cstdio>

namespace esphome {
namespace mhi_ac_ctrl {

void MhiTransportDiagnosticsPublisher::publish(const MhiTransportDiagnosticsSnapshot& snapshot, bool force) {
  if (!force && !this->changed_(snapshot)) {
    return;
  }

  if (targets_.healthy != nullptr) {
    targets_.healthy->publish_state(snapshot.transport_healthy);
  }
  if (targets_.recovery_active != nullptr) {
    targets_.recovery_active->publish_state(snapshot.recovery_active);
  }
  if (targets_.safe_mode != nullptr) {
    targets_.safe_mode->publish_state(snapshot.safe_mode);
  }
  if (targets_.active_transport != nullptr) {
    targets_.active_transport->publish_state(snapshot.active_transport_name);
  }
  if (targets_.state != nullptr) {
    targets_.state->publish_state(mhi_transport_state_name(snapshot.state));
  }
  if (targets_.last_error != nullptr) {
    char error_text[160]{};
    this->format_error_(snapshot.last_error, error_text, sizeof(error_text));
    targets_.last_error->publish_state(error_text);
  }

  this->update_cache_(snapshot);
}

void MhiTransportDiagnosticsPublisher::format_error_(const MhiTransportErrorDetail& error, char* dst,
                                                     std::size_t dst_size) {
  if (dst == nullptr || dst_size == 0U) {
    return;
  }

  if (!error.present()) {
    std::snprintf(dst, dst_size, "none");
    return;
  }

  const char* operation = error.operation == nullptr ? "unknown_operation" : error.operation;
  if (error.native_code != 0) {
    std::snprintf(dst, dst_size, "%s: %s (native=%ld)", mhi_transport_error_name(error.code), operation,
                  static_cast<long>(error.native_code));
    return;
  }

  std::snprintf(dst, dst_size, "%s: %s", mhi_transport_error_name(error.code), operation);
}

bool MhiTransportDiagnosticsPublisher::changed_(const MhiTransportDiagnosticsSnapshot& snapshot) const {
  if (!cache_valid_) {
    return true;
  }

  return cached_state_ != snapshot.state || cached_state_changes_ != snapshot.state_changes ||
         cached_error_ != snapshot.last_error.code || cached_native_error_ != snapshot.last_error.native_code ||
         cached_healthy_ != snapshot.transport_healthy || cached_recovery_active_ != snapshot.recovery_active ||
         cached_safe_mode_ != snapshot.safe_mode;
}

void MhiTransportDiagnosticsPublisher::update_cache_(const MhiTransportDiagnosticsSnapshot& snapshot) {
  cache_valid_ = true;
  cached_state_ = snapshot.state;
  cached_state_changes_ = snapshot.state_changes;
  cached_error_ = snapshot.last_error.code;
  cached_native_error_ = snapshot.last_error.native_code;
  cached_healthy_ = snapshot.transport_healthy;
  cached_recovery_active_ = snapshot.recovery_active;
  cached_safe_mode_ = snapshot.safe_mode;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
