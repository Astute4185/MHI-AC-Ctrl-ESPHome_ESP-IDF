#include "mhi_power_estimator.h"
#include "mhi_test_common.h"

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

}  // namespace mhi_unit_tests
