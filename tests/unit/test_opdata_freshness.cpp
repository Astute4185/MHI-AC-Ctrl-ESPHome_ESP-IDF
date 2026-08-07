#include "mhi_opdata_freshness.h"
#include "mhi_opdata_freshness_publisher.h"
#include "mhi_test_common.h"

namespace mhi_unit_tests {

void opdata_freshness_requires_all_enabled_requests_before_fresh() {
  MhiOpDataFreshnessTracker tracker{};
  tracker.set_enabled_mask(MHI_OPDATA_REQ_MODE | MHI_OPDATA_REQ_OUTDOOR);
  tracker.set_timeout_ms(1000U);
  tracker.begin(100U);

  auto snapshot = tracker.evaluate(100U);
  EXPECT_FALSE(snapshot.fresh);
  EXPECT_EQ(snapshot.pending_mask, MHI_OPDATA_REQ_MODE | MHI_OPDATA_REQ_OUTDOOR);
  EXPECT_EQ(snapshot.stale_mask, 0U);

  tracker.observe(MHI_OPDATA_REQ_MODE, 200U);
  snapshot = tracker.evaluate(200U);
  EXPECT_FALSE(snapshot.fresh);
  EXPECT_EQ(snapshot.pending_mask, MHI_OPDATA_REQ_OUTDOOR);

  tracker.observe(MHI_OPDATA_REQ_OUTDOOR, 300U);
  snapshot = tracker.evaluate(300U);
  EXPECT_TRUE(snapshot.fresh);
  EXPECT_EQ(snapshot.pending_mask, 0U);
  EXPECT_EQ(snapshot.stale_mask, 0U);
}

void opdata_freshness_counts_stale_transitions_once() {
  MhiOpDataFreshnessTracker tracker{};
  tracker.set_enabled_mask(MHI_OPDATA_REQ_MODE | MHI_OPDATA_REQ_OUTDOOR);
  tracker.set_timeout_ms(1000U);
  tracker.begin(100U);
  tracker.observe(MHI_OPDATA_REQ_MODE, 200U);
  tracker.observe(MHI_OPDATA_REQ_OUTDOOR, 300U);

  auto snapshot = tracker.evaluate(1200U);
  EXPECT_EQ(snapshot.stale_mask, MHI_OPDATA_REQ_MODE);
  EXPECT_EQ(snapshot.stale_count, 1U);
  EXPECT_EQ(snapshot.timeout_events, 1U);

  snapshot = tracker.evaluate(1300U);
  EXPECT_EQ(snapshot.stale_mask, MHI_OPDATA_REQ_MODE | MHI_OPDATA_REQ_OUTDOOR);
  EXPECT_EQ(snapshot.stale_count, 2U);
  EXPECT_EQ(snapshot.timeout_events, 2U);

  snapshot = tracker.evaluate(1400U);
  EXPECT_EQ(snapshot.timeout_events, 2U);

  tracker.observe(MHI_OPDATA_REQ_MODE | MHI_OPDATA_REQ_OUTDOOR, 1500U);
  snapshot = tracker.evaluate(1500U);
  EXPECT_TRUE(snapshot.fresh);
  EXPECT_EQ(snapshot.stale_mask, 0U);

  snapshot = tracker.evaluate(2500U);
  EXPECT_EQ(snapshot.stale_count, 2U);
  EXPECT_EQ(snapshot.timeout_events, 4U);
}

void opdata_freshness_reset_keeps_counters_and_restarts_observation_window() {
  MhiOpDataFreshnessTracker tracker{};
  tracker.set_enabled_mask(MHI_OPDATA_REQ_MODE);
  tracker.set_timeout_ms(1000U);
  tracker.begin(0U);

  auto snapshot = tracker.evaluate(1000U);
  EXPECT_EQ(snapshot.timeout_events, 1U);
  EXPECT_EQ(snapshot.stale_count, 1U);

  tracker.reset_observations(2000U);
  snapshot = tracker.evaluate(2000U);
  EXPECT_EQ(snapshot.timeout_events, 1U);
  EXPECT_EQ(snapshot.stale_count, 0U);
  EXPECT_EQ(snapshot.pending_mask, MHI_OPDATA_REQ_MODE);

  snapshot = tracker.evaluate(3000U);
  EXPECT_EQ(snapshot.timeout_events, 2U);
  EXPECT_EQ(snapshot.stale_count, 1U);
}

void opdata_freshness_publisher_publishes_only_meaningful_changes() {
  esphome::binary_sensor::BinarySensor fresh{};
  esphome::sensor::Sensor oldest_age{};
  esphome::sensor::Sensor stale_count{};
  esphome::sensor::Sensor timeout_events{};

  MhiOpDataFreshnessPublisher publisher{};
  publisher.set_fresh_binary_sensor(&fresh);
  publisher.set_oldest_age_sensor(&oldest_age);
  publisher.set_stale_count_sensor(&stale_count);
  publisher.set_timeout_events_sensor(&timeout_events);

  MhiOpDataFreshnessSnapshot snapshot{};
  snapshot.enabled = true;
  snapshot.fresh = true;
  snapshot.oldest_age_ms = 1000U;
  publisher.publish(snapshot);

  EXPECT_TRUE(fresh.state);
  expect_near(oldest_age.state, 1.0f);
  EXPECT_EQ(fresh.publish_count, 1U);
  EXPECT_EQ(oldest_age.publish_count, 1U);

  snapshot.oldest_age_ms = 1500U;
  publisher.publish(snapshot);
  EXPECT_EQ(oldest_age.publish_count, 1U);

  snapshot.oldest_age_ms = 2000U;
  snapshot.fresh = false;
  snapshot.stale_count = 1U;
  snapshot.timeout_events = 1U;
  publisher.publish(snapshot);
  EXPECT_FALSE(fresh.state);
  expect_near(oldest_age.state, 2.0f);
  EXPECT_EQ(stale_count.state, 1.0f);
  EXPECT_EQ(timeout_events.state, 1.0f);
  EXPECT_EQ(fresh.publish_count, 2U);
}

}  // namespace mhi_unit_tests
