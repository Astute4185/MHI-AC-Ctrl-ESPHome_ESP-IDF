#pragma once

#include <string>

namespace esphome {
namespace select {

class Select {
 public:
  void publish_state(const char* value) {
    this->state = value == nullptr ? "" : value;
    this->publish_count++;
  }

  void publish_state(const std::string& value) {
    this->state = value;
    this->publish_count++;
  }

  std::string state{};
  unsigned int publish_count{0};
};

}  // namespace select
}  // namespace esphome
