#pragma once

#include <cstdint>

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
  uint32_t checksum_failures{0U};
  uint32_t resync_events{0U};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
