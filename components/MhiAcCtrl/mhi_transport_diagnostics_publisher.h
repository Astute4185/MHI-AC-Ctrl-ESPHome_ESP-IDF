#pragma once

#include <cstddef>
#include <cstdint>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "mhi_transport_health.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiTransportDiagnosticTargets {
  binary_sensor::BinarySensor* healthy{nullptr};
  binary_sensor::BinarySensor* recovery_active{nullptr};
  binary_sensor::BinarySensor* safe_mode{nullptr};

  text_sensor::TextSensor* active_transport{nullptr};
  text_sensor::TextSensor* state{nullptr};
  text_sensor::TextSensor* last_error{nullptr};
};

class MhiTransportDiagnosticsPublisher {
 public:
  void set_healthy_binary_sensor(binary_sensor::BinarySensor* sensor) {
    targets_.healthy = sensor;
    cache_valid_ = false;
  }

  void set_recovery_active_binary_sensor(binary_sensor::BinarySensor* sensor) {
    targets_.recovery_active = sensor;
    cache_valid_ = false;
  }

  void set_safe_mode_binary_sensor(binary_sensor::BinarySensor* sensor) {
    targets_.safe_mode = sensor;
    cache_valid_ = false;
  }

  void set_active_transport_text_sensor(text_sensor::TextSensor* sensor) {
    targets_.active_transport = sensor;
    cache_valid_ = false;
  }

  void set_state_text_sensor(text_sensor::TextSensor* sensor) {
    targets_.state = sensor;
    cache_valid_ = false;
  }

  void set_last_error_text_sensor(text_sensor::TextSensor* sensor) {
    targets_.last_error = sensor;
    cache_valid_ = false;
  }

  void publish(const MhiTransportDiagnosticsSnapshot& snapshot, bool force = false);

 private:
  static void format_error_(const MhiTransportErrorDetail& error, char* dst, std::size_t dst_size);
  bool changed_(const MhiTransportDiagnosticsSnapshot& snapshot) const;
  void update_cache_(const MhiTransportDiagnosticsSnapshot& snapshot);

  MhiTransportDiagnosticTargets targets_{};
  bool cache_valid_{false};
  MhiTransportState cached_state_{MhiTransportState::STOPPED};
  uint32_t cached_state_changes_{0U};
  MhiTransportError cached_error_{MhiTransportError::NONE};
  int32_t cached_native_error_{0};
  bool cached_healthy_{false};
  bool cached_recovery_active_{false};
  bool cached_safe_mode_{false};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
