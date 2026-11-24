#include "mhi_power_estimator.h"
#include "mhi_test_common.h"

#include <cmath>

namespace mhi_unit_tests {

void power_estimator_calculates_instantaneous_power() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 0.9f, 0.0f, 300000U);
  estimator.set_enabled(true);
  estimator.begin();

  EXPECT_TRUE(estimator.observe_current(2.0f, true, true, 1000U));
  const auto snapshot = estimator.snapshot();

  EXPECT_TRUE(snapshot.power_valid);
  EXPECT_TRUE(snapshot.energy_valid);
  expect_near(snapshot.power_w, 414.0f, 0.001f);
  expect_near(static_cast<float>(snapshot.energy_kwh), 0.0f, 0.000001f);
  EXPECT_EQ(snapshot.sample_count, 1U);
}

void power_estimator_integrates_energy_between_fresh_samples() {
  MhiPowerEstimator estimator{};
  estimator.configure(100.0f, 1.0f, 0.0f, 7200000U);
  estimator.set_enabled(true);
  estimator.begin();

  estimator.observe_current(1.0f, true, true, 0U);
  estimator.observe_current(1.0f, true, true, 3600000U);

  const auto snapshot = estimator.snapshot();
  expect_near(snapshot.power_w, 100.0f, 0.001f);
  expect_near(static_cast<float>(snapshot.energy_kwh), 0.1f, 0.000001f);
  EXPECT_EQ(snapshot.integrated_intervals, 1U);
  EXPECT_EQ(snapshot.skipped_intervals, 0U);
}

void power_estimator_uses_trapezoidal_integration() {
  MhiPowerEstimator estimator{};
  estimator.configure(100.0f, 1.0f, 0.0f, 7200000U);
  estimator.set_enabled(true);
  estimator.begin();

  estimator.observe_current(1.0f, true, true, 0U);
  estimator.observe_current(3.0f, true, true, 3600000U);

  const auto snapshot = estimator.snapshot();
  expect_near(snapshot.power_w, 300.0f, 0.001f);
  expect_near(static_cast<float>(snapshot.energy_kwh), 0.2f, 0.000001f);
}

void power_estimator_skips_stale_sample_gaps() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 1.0f, 0.0f, 60000U);
  estimator.set_enabled(true);
  estimator.begin();

  estimator.observe_current(1.0f, true, true, 0U);
  estimator.observe_current(1.0f, true, true, 60001U);

  const auto snapshot = estimator.snapshot();
  expect_near(static_cast<float>(snapshot.energy_kwh), 0.0f, 0.000001f);
  EXPECT_EQ(snapshot.integrated_intervals, 0U);
  EXPECT_EQ(snapshot.skipped_intervals, 1U);
}

void power_estimator_applies_configured_standby_floor_only_when_off() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 1.0f, 8.0f, 300000U);
  estimator.set_enabled(true);
  estimator.begin();

  estimator.observe_current(0.01f, true, false, 1000U);
  expect_near(estimator.snapshot().power_w, 8.0f, 0.001f);

  estimator.observe_current(0.01f, true, true, 2000U);
  expect_near(estimator.snapshot().power_w, 2.3f, 0.001f);
}

void power_estimator_reset_window_preserves_accumulated_energy() {
  MhiPowerEstimator estimator{};
  estimator.configure(100.0f, 1.0f, 0.0f, 7200000U);
  estimator.set_enabled(true);
  estimator.begin();

  estimator.observe_current(1.0f, true, true, 0U);
  estimator.observe_current(1.0f, true, true, 3600000U);
  estimator.reset_sample_window();
  estimator.observe_current(1.0f, true, true, 7200000U);

  const auto snapshot = estimator.snapshot();
  expect_near(static_cast<float>(snapshot.energy_kwh), 0.1f, 0.000001f);
  EXPECT_EQ(snapshot.integrated_intervals, 1U);
}

void power_estimator_is_inactive_until_enabled() {
  MhiPowerEstimator estimator{};
  estimator.begin();

  EXPECT_FALSE(estimator.observe_current(2.0f, true, true, 1000U));
  const auto snapshot = estimator.snapshot();
  EXPECT_FALSE(snapshot.enabled);
  EXPECT_FALSE(snapshot.power_valid);
  EXPECT_EQ(snapshot.sample_count, 0U);
}

void power_estimator_uses_fresh_external_voltage_and_falls_back_on_timeout() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 0.95f, 0.0f, 300000U);
  estimator.set_external_voltage_timeout_ms(5000U);
  estimator.set_enabled(true);
  estimator.begin();

  EXPECT_TRUE(estimator.update_external_voltage(245.0f, 1000U));
  EXPECT_TRUE(estimator.external_voltage_active(5999U));
  EXPECT_TRUE(estimator.observe_current(2.0f, true, true, 2000U));
  auto reading = estimator.snapshot();
  expect_near(reading.power_w, 465.5f, 0.001f);
  EXPECT_TRUE(reading.used_external_voltage);
  expect_near(reading.voltage_v, 245.0f, 0.001f);

  EXPECT_FALSE(estimator.external_voltage_active(6000U));
  estimator.observe_current(2.0f, true, true, 6000U);
  reading = estimator.snapshot();
  expect_near(reading.power_w, 437.0f, 0.001f);
  EXPECT_FALSE(reading.used_external_voltage);
  expect_near(reading.voltage_v, 230.0f, 0.001f);

  EXPECT_TRUE(estimator.update_external_voltage(240.0f, 6100U));
  estimator.observe_current(2.0f, true, true, 6200U);
  expect_near(estimator.snapshot().power_w, 456.0f, 0.001f);
  EXPECT_TRUE(estimator.snapshot().used_external_voltage);
}

void power_estimator_rejects_invalid_external_voltage() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 1.0f, 0.0f, 300000U);
  estimator.set_enabled(true);
  estimator.begin();

  EXPECT_TRUE(estimator.update_external_voltage(240.0f, 0U));
  EXPECT_FALSE(estimator.update_external_voltage(NAN, 100U));
  EXPECT_FALSE(estimator.update_external_voltage(0.0f, 200U));
  EXPECT_FALSE(estimator.update_external_voltage(301.0f, 300U));
  EXPECT_FALSE(estimator.update_external_voltage(79.0f, 400U));
  estimator.observe_current(1.0f, true, true, 500U);
  expect_near(estimator.snapshot().power_w, 230.0f, 0.001f);
  EXPECT_FALSE(estimator.snapshot().used_external_voltage);
}

void power_estimator_external_voltage_handles_millis_rollover() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 1.0f, 0.0f, 300000U);
  estimator.set_external_voltage_timeout_ms(5000U);
  estimator.set_enabled(true);
  estimator.begin();

  const uint32_t last_update_ms = 0xFFFFFF00U;
  EXPECT_TRUE(estimator.update_external_voltage(240.0f, last_update_ms));
  EXPECT_TRUE(estimator.external_voltage_active(1000U));
  estimator.observe_current(1.0f, true, true, 1000U);
  expect_near(estimator.snapshot().power_w, 240.0f, 0.001f);
  EXPECT_FALSE(estimator.external_voltage_active(6000U));
  estimator.observe_current(1.0f, true, true, 6000U);
  expect_near(estimator.snapshot().power_w, 230.0f, 0.001f);
}

void power_estimator_integrates_voltage_changes_between_ct_samples() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 1.0f, 0.0f, 7200000U);
  estimator.set_external_voltage_timeout_ms(7200000U);
  estimator.set_enabled(true);
  estimator.begin();

  estimator.update_external_voltage(240.0f, 0U);
  estimator.observe_current(1.0f, true, true, 0U);
  estimator.update_external_voltage(250.0f, 1800000U);
  estimator.observe_current(1.0f, true, true, 3600000U);
  expect_near(static_cast<float>(estimator.snapshot().energy_kwh), 0.245f, 0.000001f);
}

void power_estimator_external_power_factor_and_voltage_are_independent() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 0.9f, 0.0f, 300000U);
  estimator.set_external_voltage_timeout_ms(5000U);
  estimator.set_external_power_factor_timeout_ms(2000U);
  estimator.set_enabled(true);
  estimator.begin();

  EXPECT_TRUE(estimator.update_external_voltage(240.0f, 1000U));
  EXPECT_TRUE(estimator.update_external_power_factor(0.8f, 1000U));
  estimator.observe_current(2.0f, true, true, 1500U);
  auto reading = estimator.snapshot();
  expect_near(reading.power_w, 384.0f, 0.001f);
  EXPECT_TRUE(reading.used_external_voltage);
  EXPECT_TRUE(reading.used_external_power_factor);
  expect_near(reading.power_factor, 0.8f, 0.001f);

  // PF expired, while voltage remains valid.
  estimator.observe_current(2.0f, true, true, 3000U);
  reading = estimator.snapshot();
  expect_near(reading.power_w, 432.0f, 0.001f);
  EXPECT_TRUE(reading.used_external_voltage);
  EXPECT_FALSE(reading.used_external_power_factor);
  expect_near(reading.power_factor, 0.9f, 0.001f);

  // Voltage expired, while PF has recovered.
  EXPECT_TRUE(estimator.update_external_power_factor(0.7f, 6000U));
  estimator.observe_current(2.0f, true, true, 6000U);
  reading = estimator.snapshot();
  expect_near(reading.power_w, 322.0f, 0.001f);
  EXPECT_FALSE(reading.used_external_voltage);
  EXPECT_TRUE(reading.used_external_power_factor);
}

void power_estimator_rejects_invalid_power_factor_and_recovers() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 0.95f, 0.0f, 300000U);
  estimator.set_enabled(true);
  estimator.begin();
  EXPECT_TRUE(estimator.update_external_power_factor(1.0f, 100U));
  EXPECT_FALSE(estimator.update_external_power_factor(NAN, 200U));
  EXPECT_FALSE(estimator.update_external_power_factor(INFINITY, 300U));
  EXPECT_FALSE(estimator.update_external_power_factor(-0.5f, 400U));
  EXPECT_FALSE(estimator.update_external_power_factor(1.01f, 500U));
  EXPECT_FALSE(estimator.update_external_power_factor(0.0f, 600U));
  estimator.observe_current(1.0f, true, true, 700U);
  expect_near(estimator.snapshot().power_w, 218.5f, 0.001f);
  EXPECT_FALSE(estimator.snapshot().used_external_power_factor);

  EXPECT_TRUE(estimator.update_external_power_factor(0.5f, 800U));
  estimator.observe_current(1.0f, true, true, 900U);
  expect_near(estimator.snapshot().power_w, 115.0f, 0.001f);
  EXPECT_TRUE(estimator.snapshot().used_external_power_factor);
}

void power_estimator_external_power_factor_handles_timer_rollover() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 0.95f, 0.0f, 300000U);
  estimator.set_external_power_factor_timeout_ms(5000U);
  estimator.set_enabled(true);
  estimator.begin();
  EXPECT_TRUE(estimator.update_external_power_factor(0.8f, 0xFFFFFF00U));
  EXPECT_TRUE(estimator.external_power_factor_active(1000U));
  estimator.observe_current(1.0f, true, true, 1000U);
  expect_near(estimator.snapshot().power_w, 184.0f, 0.001f);
  EXPECT_FALSE(estimator.external_power_factor_active(6000U));
  estimator.observe_current(1.0f, true, true, 6000U);
  expect_near(estimator.snapshot().power_w, 218.5f, 0.001f);
}

void power_estimator_integrates_voltage_and_power_factor_changes() {
  MhiPowerEstimator estimator{};
  estimator.configure(230.0f, 1.0f, 0.0f, 7200000U);
  estimator.set_external_voltage_timeout_ms(7200000U);
  estimator.set_external_power_factor_timeout_ms(7200000U);
  estimator.set_enabled(true);
  estimator.begin();
  estimator.update_external_voltage(240.0f, 0U);
  estimator.update_external_power_factor(0.8f, 0U);
  estimator.observe_current(1.0f, true, true, 0U);
  estimator.update_external_voltage(250.0f, 1800000U);
  estimator.update_external_power_factor(0.9f, 1800000U);
  estimator.observe_current(1.0f, true, true, 3600000U);
  expect_near(static_cast<float>(estimator.snapshot().energy_kwh), 0.2085f, 0.000001f);
}

}  // namespace mhi_unit_tests
