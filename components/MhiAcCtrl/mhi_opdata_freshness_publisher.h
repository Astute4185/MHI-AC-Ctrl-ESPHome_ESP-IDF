#pragma once

#include <cstdint>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "mhi_opdata_freshness.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiOpDataFreshnessPublisher {
 public:
  void set_fresh_binary_sensor(binary_sensor::BinarySensor* sensor) {
    fresh_sensor_ = sensor;
    cache_valid_ = false;
  }

  void set_oldest_age_sensor(sensor::Sensor* sensor) {
    oldest_age_sensor_ = sensor;
    cache_valid_ = false;
  }

  void set_stale_count_sensor(sensor::Sensor* sensor) {
    stale_count_sensor_ = sensor;
    cache_valid_ = false;
  }

  void set_timeout_events_sensor(sensor::Sensor* sensor) {
    timeout_events_sensor_ = sensor;
    cache_valid_ = false;
  }

  void publish(const MhiOpDataFreshnessSnapshot& snapshot, bool force = false);

 private:
  bool changed_(const MhiOpDataFreshnessSnapshot& snapshot) const;
  void update_cache_(const MhiOpDataFreshnessSnapshot& snapshot);

  binary_sensor::BinarySensor* fresh_sensor_{nullptr};
  sensor::Sensor* oldest_age_sensor_{nullptr};
  sensor::Sensor* stale_count_sensor_{nullptr};
  sensor::Sensor* timeout_events_sensor_{nullptr};

  bool cache_valid_{false};
  bool cached_fresh_{false};
  uint32_t cached_oldest_age_seconds_{0U};
  uint8_t cached_stale_count_{0U};
  uint32_t cached_timeout_events_{0U};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
