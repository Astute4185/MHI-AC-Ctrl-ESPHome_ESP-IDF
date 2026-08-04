#include "mhi_power_estimator.h"

#include <algorithm>
#include <cmath>

namespace esphome {
namespace mhi_ac_ctrl {

void MhiPowerEstimator::configure(float nominal_voltage_v, float power_factor, float standby_power_w,
                                  uint32_t max_sample_interval_ms) {
  if (std::isfinite(nominal_voltage_v) && nominal_voltage_v > 0.0f) {
    nominal_voltage_v_ = nominal_voltage_v;
  }
  if (std::isfinite(power_factor) && power_factor > 0.0f && power_factor <= 1.0f) {
    power_factor_ = power_factor;
  }
  if (std::isfinite(standby_power_w) && standby_power_w >= 0.0f) {
    standby_power_w_ = standby_power_w;
  }
  if (max_sample_interval_ms > 0U) {
    max_sample_interval_ms_ = max_sample_interval_ms;
  }
}

void MhiPowerEstimator::begin() {
  has_previous_sample_ = false;
  power_valid_ = false;
  energy_valid_ = false;
  power_w_ = 0.0f;
  energy_kwh_ = 0.0;
  last_sample_ms_ = 0U;
  sample_count_ = 0U;
  integrated_intervals_ = 0U;
  skipped_intervals_ = 0U;
}

void MhiPowerEstimator::reset_sample_window() {
  has_previous_sample_ = false;
  power_valid_ = false;
  last_sample_ms_ = 0U;
}

bool MhiPowerEstimator::observe_current(float current_a, bool power_state_valid, bool power_on, uint32_t now_ms) {
  if (!enabled_ || !std::isfinite(current_a) || current_a < 0.0f) {
    return false;
  }

  const float new_power_w = calculate_power_w_(current_a, power_state_valid, power_on);

  if (has_previous_sample_) {
    const uint32_t elapsed_ms = now_ms - last_sample_ms_;
    if (elapsed_ms > 0U && elapsed_ms <= max_sample_interval_ms_) {
      const double average_power_w = (static_cast<double>(power_w_) + static_cast<double>(new_power_w)) * 0.5;
      energy_kwh_ += average_power_w * static_cast<double>(elapsed_ms) / 3600000000.0;
      integrated_intervals_++;
    } else if (elapsed_ms > max_sample_interval_ms_) {
      skipped_intervals_++;
    }
  }

  power_w_ = new_power_w;
  power_valid_ = true;
  energy_valid_ = true;
  has_previous_sample_ = true;
  last_sample_ms_ = now_ms;
  sample_count_++;
  return true;
}

MhiPowerEstimateSnapshot MhiPowerEstimator::snapshot() const {
  MhiPowerEstimateSnapshot snapshot{};
  snapshot.enabled = enabled_;
  snapshot.power_valid = power_valid_;
  snapshot.energy_valid = energy_valid_;
  snapshot.power_w = power_w_;
  snapshot.energy_kwh = energy_kwh_;
  snapshot.sample_count = sample_count_;
  snapshot.integrated_intervals = integrated_intervals_;
  snapshot.skipped_intervals = skipped_intervals_;
  snapshot.last_sample_ms = last_sample_ms_;
  return snapshot;
}

float MhiPowerEstimator::calculate_power_w_(float current_a, bool power_state_valid, bool power_on) const {
  const float measured_power_w = current_a * nominal_voltage_v_ * power_factor_;
  if (power_state_valid && !power_on) {
    return std::max(measured_power_w, standby_power_w_);
  }
  return measured_power_w;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
