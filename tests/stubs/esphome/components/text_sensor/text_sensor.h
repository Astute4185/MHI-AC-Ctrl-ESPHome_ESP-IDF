#pragma once

#include <string>

namespace esphome {
namespace text_sensor {

class TextSensor {
 public:
  void publish_state(const char* value) {
    this->state = value == nullptr ? "" : value;
    this->publish_count++;
  }

  std::string state{};
  unsigned int publish_count{0};
};

}  // namespace text_sensor
}  // namespace esphome
