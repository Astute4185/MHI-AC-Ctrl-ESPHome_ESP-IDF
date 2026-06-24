#pragma once

namespace esphome {
namespace binary_sensor {

class BinarySensor {
 public:
  void publish_state(bool value) {
    this->state = value;
    this->publish_count++;
  }

  bool state{false};
  unsigned int publish_count{0};
};

}  // namespace binary_sensor
}  // namespace esphome
