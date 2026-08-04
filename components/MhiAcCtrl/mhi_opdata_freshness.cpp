#include "mhi_opdata_freshness.h"

namespace esphome {
namespace mhi_ac_ctrl {

void MhiOpDataFreshnessTracker::begin(uint32_t now_ms) {
  timeout_events_ = 0U;
  this->reset_observations(now_ms);
}

void MhiOpDataFreshnessTracker::reset_observations(uint32_t now_ms) {
  observed_mask_ = 0U;
  stale_mask_ = 0U;
  observation_start_ms_ = now_ms;
  last_update_ms_ = 0U;
  last_seen_ms_.fill(0U);
}

void MhiOpDataFreshnessTracker::observe(uint32_t accepted_mask, uint32_t now_ms) {
  const uint32_t relevant_mask = accepted_mask & enabled_mask_;
  if (relevant_mask == 0U) {
    return;
  }

  for (std::size_t index = 0U; index < last_seen_ms_.size(); index++) {
    const uint32_t bit = 1UL << index;
    if ((relevant_mask & bit) != 0U) {
      last_seen_ms_[index] = now_ms;
    }
  }

  observed_mask_ |= relevant_mask;
  stale_mask_ &= ~relevant_mask;
  last_update_ms_ = now_ms;
}

MhiOpDataFreshnessSnapshot MhiOpDataFreshnessTracker::evaluate(uint32_t now_ms) {
  MhiOpDataFreshnessSnapshot snapshot{};
  snapshot.enabled_mask = enabled_mask_;
  snapshot.observed_mask = observed_mask_ & enabled_mask_;
  snapshot.pending_mask = enabled_mask_ & ~snapshot.observed_mask;
  snapshot.enabled = enabled_mask_ != 0U;

  uint32_t next_stale_mask = 0U;
  uint32_t oldest_age_ms = 0U;

  for (std::size_t index = 0U; index < last_seen_ms_.size(); index++) {
    const uint32_t bit = 1UL << index;
    if ((enabled_mask_ & bit) == 0U) {
      continue;
    }

    const bool observed = (observed_mask_ & bit) != 0U;
    const uint32_t reference_ms = observed ? last_seen_ms_[index] : observation_start_ms_;
    const uint32_t age_ms = now_ms - reference_ms;
    if (age_ms > oldest_age_ms) {
      oldest_age_ms = age_ms;
    }

    if (timeout_ms_ > 0U && age_ms >= timeout_ms_) {
      next_stale_mask |= bit;
    }
  }

  const uint32_t newly_stale_mask = next_stale_mask & ~stale_mask_;
  timeout_events_ += count_bits_(newly_stale_mask);
  stale_mask_ = next_stale_mask;

  snapshot.stale_mask = stale_mask_;
  snapshot.stale_count = count_bits_(stale_mask_);
  snapshot.oldest_age_ms = oldest_age_ms;
  snapshot.last_update_age_ms = last_update_ms_ == 0U ? now_ms - observation_start_ms_ : now_ms - last_update_ms_;
  snapshot.timeout_events = timeout_events_;
  snapshot.fresh = snapshot.enabled && snapshot.pending_mask == 0U && snapshot.stale_mask == 0U;
  return snapshot;
}

uint8_t MhiOpDataFreshnessTracker::count_bits_(uint32_t value) {
  uint8_t count = 0U;
  while (value != 0U) {
    value &= value - 1U;
    count++;
  }
  return count;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
