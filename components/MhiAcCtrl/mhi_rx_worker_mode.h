#pragma once

#include <cstdint>

namespace esphome {
namespace mhi_ac_ctrl {

enum class MhiRxWorkerMode : uint8_t {
  AUTO = 0,
  ENABLED,
  DISABLED,
};

inline const char* mhi_rx_worker_mode_to_string(MhiRxWorkerMode mode) {
  switch (mode) {
    case MhiRxWorkerMode::AUTO:
      return "auto";
    case MhiRxWorkerMode::ENABLED:
      return "true";
    case MhiRxWorkerMode::DISABLED:
      return "false";
  }

  return "unknown";
}

inline bool mhi_rx_worker_mode_resolves_enabled(MhiRxWorkerMode mode, uint8_t core_count) {
  switch (mode) {
    case MhiRxWorkerMode::AUTO:
      return core_count > 1U;
    case MhiRxWorkerMode::ENABLED:
      return true;
    case MhiRxWorkerMode::DISABLED:
      return false;
  }

  return false;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
