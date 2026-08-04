#pragma once

#include <cstdint>

#include "mhi_transport_result.h"

namespace esphome {
namespace mhi_ac_ctrl {

enum class MhiTransportState : uint8_t {
  STOPPED = 0,
  STARTING,
  WAITING_FOR_TRAFFIC,
  HEALTHY,
  DEGRADED,
  FAILED,
  RECOVERY_ACTIVE,
  SAFE_MODE,
};

inline const char* mhi_transport_state_name(MhiTransportState state) {
  switch (state) {
    case MhiTransportState::STOPPED:
      return "stopped";
    case MhiTransportState::STARTING:
      return "starting";
    case MhiTransportState::WAITING_FOR_TRAFFIC:
      return "waiting_for_traffic";
    case MhiTransportState::HEALTHY:
      return "healthy";
    case MhiTransportState::DEGRADED:
      return "degraded";
    case MhiTransportState::FAILED:
      return "failed";
    case MhiTransportState::RECOVERY_ACTIVE:
      return "recovery_active";
    case MhiTransportState::SAFE_MODE:
      return "safe_mode";
  }
  return "unknown";
}

struct MhiTransportHealth {
  MhiTransportState state{MhiTransportState::STOPPED};
  uint32_t setup_at_ms{0U};
  uint32_t last_rx_activity_ms{0U};
  uint32_t last_tx_activity_ms{0U};
  uint32_t rx_bytes{0U};
  uint32_t tx_completed{0U};
  uint32_t tx_failures{0U};
  uint32_t transport_errors{0U};
  bool traffic_seen{false};
  bool fault_latched{false};
};

struct MhiProtocolHealth {
  uint32_t last_valid_frame_ms{0U};
  uint32_t valid_frames{0U};
  uint32_t invalid_frames{0U};
  uint32_t checksum_failures{0U};
  uint32_t resync_events{0U};
};

struct MhiTransportDiagnosticsSnapshot {
  MhiTransportState state{MhiTransportState::STOPPED};
  const char* active_transport_name{"none"};
  const char* primary_transport_name{"none"};
  const char* recovery_transport_name{"none"};

  MhiTransportErrorDetail last_error{};
  MhiTransportErrorDetail primary_failure{};
  MhiTransportErrorDetail recovery_failure{};

  uint32_t state_since_ms{0U};
  uint32_t last_transition_ms{0U};
  uint32_t last_valid_frame_age_ms{0U};

  uint32_t state_changes{0U};
  uint32_t recovery_attempts{0U};
  uint32_t recovery_activations{0U};
  uint32_t recovery_failures{0U};
  uint32_t safe_mode_entries{0U};

  bool transport_healthy{false};
  bool recovery_active{false};
  bool safe_mode{false};
};

// Conservative defaults for automatic runtime recovery. These values are
// intentionally internal during the first hardware validation cycle.
struct MhiTransportHealthPolicy {
  uint32_t startup_grace_ms{10000U};
  uint32_t no_traffic_timeout_ms{15000U};
  uint32_t invalid_traffic_timeout_ms{15000U};
  uint32_t stalled_traffic_timeout_ms{15000U};
  uint32_t health_check_interval_ms{250U};
  uint32_t healthy_frame_count{4U};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
