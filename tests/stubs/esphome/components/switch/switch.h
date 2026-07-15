#pragma once

namespace esphome {
namespace switch_ {

class Switch {
 public:
  void publish_state(bool value) {
    this->state = value;
    this->publish_count++;
  }

  bool state{false};
  unsigned int publish_count{0};
};

}  // namespace switch_
}  // namespace esphome
