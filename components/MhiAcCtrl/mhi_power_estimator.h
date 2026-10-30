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
  float voltage_v{230.0f};
  bool used_external_voltage{false};
  float power_factor{1.0f};
  bool used_external_power_factor{false};
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

  // A voltage update is considered fresh only until the configured timeout.
  // Invalid sensor values invalidate the external source immediately.
  void set_external_voltage_timeout_ms(uint32_t timeout_ms);
  bool update_external_voltage(float voltage_v, uint32_t now_ms);
  float effective_voltage_v(uint32_t now_ms) const;
  bool external_voltage_active(uint32_t now_ms) const;

  void set_external_power_factor_timeout_ms(uint32_t timeout_ms);
  bool update_external_power_factor(float power_factor, uint32_t now_ms);
  float effective_power_factor(uint32_t now_ms) const;
  bool external_power_factor_active(uint32_t now_ms) const;

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

  uint32_t external_voltage_timeout_ms() const {
    return external_voltage_timeout_ms_;
  }

  uint32_t external_power_factor_timeout_ms() const {
    return external_power_factor_timeout_ms_;
  }

  uint32_t max_sample_interval_ms() const {
    return max_sample_interval_ms_;
  }

 private:
  float calculate_power_w_(float current_a, float voltage_v, float power_factor, bool power_state_valid,
                           bool power_on) const;

  bool enabled_{false};
  float nominal_voltage_v_{230.0f};
  float power_factor_{1.0f};
  float standby_power_w_{0.0f};
  uint32_t max_sample_interval_ms_{300000U};
  uint32_t external_voltage_timeout_ms_{300000U};
  float external_voltage_v_{0.0f};
  uint32_t last_external_voltage_ms_{0U};
  bool external_voltage_valid_{false};
  float last_used_voltage_v_{230.0f};
  bool last_used_external_voltage_{false};
  uint32_t external_power_factor_timeout_ms_{300000U};
  float external_power_factor_{0.0f};
  uint32_t last_external_power_factor_ms_{0U};
  bool external_power_factor_valid_{false};
  float last_used_power_factor_{1.0f};
  bool last_used_external_power_factor_{false};

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
