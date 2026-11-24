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

void MhiPowerEstimator::set_external_voltage_timeout_ms(uint32_t timeout_ms) {
  if (timeout_ms > 0U) {
    external_voltage_timeout_ms_ = timeout_ms;
  }
}

bool MhiPowerEstimator::update_external_voltage(float voltage_v, uint32_t now_ms) {
  // Domestic mains voltage only. Reject NaN, infinity, zero and implausible readings.
  if (!std::isfinite(voltage_v) || voltage_v < 80.0f || voltage_v > 300.0f) {
    external_voltage_valid_ = false;
    return false;
  }
  external_voltage_v_ = voltage_v;
  last_external_voltage_ms_ = now_ms;
  external_voltage_valid_ = true;
  return true;
}

bool MhiPowerEstimator::external_voltage_active(uint32_t now_ms) const {
  return external_voltage_valid_ && (now_ms - last_external_voltage_ms_ < external_voltage_timeout_ms_);
}

float MhiPowerEstimator::effective_voltage_v(uint32_t now_ms) const {
  return external_voltage_active(now_ms) ? external_voltage_v_ : nominal_voltage_v_;
}

void MhiPowerEstimator::set_external_power_factor_timeout_ms(uint32_t timeout_ms) {
  if (timeout_ms > 0U) {
    external_power_factor_timeout_ms_ = timeout_ms;
  }
}

bool MhiPowerEstimator::update_external_power_factor(float power_factor, uint32_t now_ms) {
  // Power factor is a dimensionless ratio, not a percentage.
  if (!std::isfinite(power_factor) || power_factor <= 0.0f || power_factor > 1.0f) {
    external_power_factor_valid_ = false;
    return false;
  }
  external_power_factor_ = power_factor;
  last_external_power_factor_ms_ = now_ms;
  external_power_factor_valid_ = true;
  return true;
}

bool MhiPowerEstimator::external_power_factor_active(uint32_t now_ms) const {
  return external_power_factor_valid_ && (now_ms - last_external_power_factor_ms_ < external_power_factor_timeout_ms_);
}

float MhiPowerEstimator::effective_power_factor(uint32_t now_ms) const {
  return external_power_factor_active(now_ms) ? external_power_factor_ : power_factor_;
}

void MhiPowerEstimator::begin() {
  external_voltage_valid_ = false;
  last_external_voltage_ms_ = 0U;
  last_used_voltage_v_ = nominal_voltage_v_;
  last_used_external_voltage_ = false;
  external_power_factor_valid_ = false;
  last_external_power_factor_ms_ = 0U;
  last_used_power_factor_ = power_factor_;
  last_used_external_power_factor_ = false;
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

  last_used_external_voltage_ = external_voltage_active(now_ms);
  last_used_voltage_v_ = effective_voltage_v(now_ms);
  last_used_external_power_factor_ = external_power_factor_active(now_ms);
  last_used_power_factor_ = effective_power_factor(now_ms);
  const float new_power_w =
      calculate_power_w_(current_a, last_used_voltage_v_, last_used_power_factor_, power_state_valid, power_on);

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
  snapshot.voltage_v = last_used_voltage_v_;
  snapshot.used_external_voltage = last_used_external_voltage_;
  snapshot.power_factor = last_used_power_factor_;
  snapshot.used_external_power_factor = last_used_external_power_factor_;
  return snapshot;
}

float MhiPowerEstimator::calculate_power_w_(float current_a, float voltage_v, float power_factor,
                                            bool power_state_valid, bool power_on) const {
  const float measured_power_w = current_a * voltage_v * power_factor;
  if (power_state_valid && !power_on) {
    return std::max(measured_power_w, standby_power_w_);
  }
  return measured_power_w;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
