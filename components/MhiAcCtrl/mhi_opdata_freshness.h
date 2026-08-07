#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mhi_tx_builder.h"

namespace esphome {
namespace mhi_ac_ctrl {

constexpr std::size_t kMhiOpDataRequestCount = 20U;

struct MhiOpDataFreshnessSnapshot {
  uint32_t enabled_mask{0U};
  uint32_t observed_mask{0U};
  uint32_t pending_mask{0U};
  uint32_t stale_mask{0U};

  uint32_t oldest_age_ms{0U};
  uint32_t last_update_age_ms{0U};
  uint32_t timeout_events{0U};

  uint8_t stale_count{0U};
  bool enabled{false};
  bool fresh{false};
};

class MhiOpDataFreshnessTracker {
 public:
  void set_enabled_mask(uint32_t mask) {
    enabled_mask_ = mask;
  }

  uint32_t enabled_mask() const {
    return enabled_mask_;
  }

  void set_timeout_ms(uint32_t timeout_ms) {
    timeout_ms_ = timeout_ms;
  }

  uint32_t timeout_ms() const {
    return timeout_ms_;
  }

  void begin(uint32_t now_ms);
  void reset_observations(uint32_t now_ms);
  void observe(uint32_t accepted_mask, uint32_t now_ms);
  MhiOpDataFreshnessSnapshot evaluate(uint32_t now_ms);

 private:
  static uint8_t count_bits_(uint32_t value);

  uint32_t enabled_mask_{kMhiDefaultOpdataMask};
  uint32_t observed_mask_{0U};
  uint32_t stale_mask_{0U};
  uint32_t timeout_events_{0U};
  uint32_t timeout_ms_{120000U};
  uint32_t observation_start_ms_{0U};
  uint32_t last_update_ms_{0U};
  std::array<uint32_t, kMhiOpDataRequestCount> last_seen_ms_{};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
