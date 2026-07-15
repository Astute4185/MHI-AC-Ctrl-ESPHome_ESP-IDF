#pragma once

namespace esphome {
namespace sensor {

class Sensor {
 public:
  void publish_state(float value) {
    this->state = value;
    this->publish_count++;
  }

  float state{0.0f};
  unsigned int publish_count{0};
};

}  // namespace sensor
}  // namespace esphome
