#pragma once

#include <cstdint>

#include "mhi_stats.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiDiagSnapshot {
  MhiStatsSnapshot stats{};

  const char* rx_driver_name{"none"};
  const char* tx_driver_name{"none"};

  bool rx_driver_ready{false};
  bool tx_driver_ready{false};

  uint32_t now_ms{0};

  uint32_t last_valid_frame_age_ms{0};
  uint32_t last_rx_byte_age_ms{0};
  uint32_t last_tx_frame_age_ms{0};
  uint32_t last_tx_command_frame_age_ms{0};
  uint32_t last_unsupported_command_age_ms{0};
  uint32_t last_command_confirmation_age_ms{0};
  uint32_t last_command_confirmation_timeout_age_ms{0};
  uint32_t last_loop_over_budget_age_ms{0};
};

class MhiDiagnostics {
 public:
  MhiStats& stats() {
    return stats_;
  }
  const MhiStats& stats() const {
    return stats_;
  }

  void set_rx_driver_name(const char* name);
  void set_tx_driver_name(const char* name);

  void set_rx_driver_ready(bool ready);
  void set_tx_driver_ready(bool ready);

  MhiDiagSnapshot snapshot(uint32_t now_ms) const;

 private:
  static uint32_t age_or_zero(uint32_t now_ms, uint32_t event_ms);

  MhiStats stats_{};

  const char* rx_driver_name_{"none"};
  const char* tx_driver_name_{"none"};

  bool rx_driver_ready_{false};
  bool tx_driver_ready_{false};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome