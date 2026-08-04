#include "mhi_opdata_freshness_publisher.h"

namespace esphome {
namespace mhi_ac_ctrl {

void MhiOpDataFreshnessPublisher::publish(const MhiOpDataFreshnessSnapshot& snapshot, bool force) {
  if (!force && !this->changed_(snapshot)) {
    return;
  }

  if (fresh_sensor_ != nullptr) {
    fresh_sensor_->publish_state(snapshot.fresh);
  }
  if (oldest_age_sensor_ != nullptr) {
    oldest_age_sensor_->publish_state(static_cast<float>(snapshot.oldest_age_ms) / 1000.0f);
  }
  if (stale_count_sensor_ != nullptr) {
    stale_count_sensor_->publish_state(snapshot.stale_count);
  }
  if (timeout_events_sensor_ != nullptr) {
    timeout_events_sensor_->publish_state(snapshot.timeout_events);
  }

  this->update_cache_(snapshot);
}

bool MhiOpDataFreshnessPublisher::changed_(const MhiOpDataFreshnessSnapshot& snapshot) const {
  if (!cache_valid_) {
    return true;
  }

  const uint32_t oldest_age_seconds = snapshot.oldest_age_ms / 1000U;
  return cached_fresh_ != snapshot.fresh || cached_oldest_age_seconds_ != oldest_age_seconds ||
         cached_stale_count_ != snapshot.stale_count || cached_timeout_events_ != snapshot.timeout_events;
}

void MhiOpDataFreshnessPublisher::update_cache_(const MhiOpDataFreshnessSnapshot& snapshot) {
  cache_valid_ = true;
  cached_fresh_ = snapshot.fresh;
  cached_oldest_age_seconds_ = snapshot.oldest_age_ms / 1000U;
  cached_stale_count_ = snapshot.stale_count;
  cached_timeout_events_ = snapshot.timeout_events;
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
