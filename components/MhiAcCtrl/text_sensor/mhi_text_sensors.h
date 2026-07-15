#pragma once

#include "../mhi_ac_ctrl.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiTextSensors : public Component, public Parented<MhiAcCtrl> {
 public:
  void setup() override;
  void dump_config() override;

  void set_error_code(text_sensor::TextSensor* sensor) {
    this->error_code_sensor_ = sensor;
  }

  void set_protection_state(text_sensor::TextSensor* sensor) {
    this->protection_state_sensor_ = sensor;
  }

 protected:
  text_sensor::TextSensor* error_code_sensor_{nullptr};
  text_sensor::TextSensor* protection_state_sensor_{nullptr};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome