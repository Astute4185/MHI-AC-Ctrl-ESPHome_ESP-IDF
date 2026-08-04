#pragma once

#include <cstdint>

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiPowerEstimateSnapshot {
  bool enabled{false};
  bool power_valid{false};
  bool energy_valid{false};

  float power_w{0.0f};
  double energy_kwh{0.0};

  uint32_t sample_count{0U};
  uint32_t integrated_intervals{0U};
  uint32_t skipped_intervals{0U};
  uint32_t last_sample_ms{0U};
};

class MhiPowerEstimator {
 public:
  void configure(float nominal_voltage_v, float power_factor, float standby_power_w, uint32_t max_sample_interval_ms);

  void set_enabled(bool enabled) {
    enabled_ = enabled;
  }

  bool enabled() const {
    return enabled_;
  }

  void begin();
  void reset_sample_window();

  bool observe_current(float current_a, bool power_state_valid, bool power_on, uint32_t now_ms);

  MhiPowerEstimateSnapshot snapshot() const;

  float nominal_voltage_v() const {
    return nominal_voltage_v_;
  }

  float power_factor() const {
    return power_factor_;
  }

  float standby_power_w() const {
    return standby_power_w_;
  }

  uint32_t max_sample_interval_ms() const {
    return max_sample_interval_ms_;
  }

 private:
  float calculate_power_w_(float current_a, bool power_state_valid, bool power_on) const;

  bool enabled_{false};
  float nominal_voltage_v_{230.0f};
  float power_factor_{1.0f};
  float standby_power_w_{0.0f};
  uint32_t max_sample_interval_ms_{300000U};

  bool has_previous_sample_{false};
  bool power_valid_{false};
  bool energy_valid_{false};
  float power_w_{0.0f};
  double energy_kwh_{0.0};
  uint32_t last_sample_ms_{0U};
  uint32_t sample_count_{0U};
  uint32_t integrated_intervals_{0U};
  uint32_t skipped_intervals_{0U};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
