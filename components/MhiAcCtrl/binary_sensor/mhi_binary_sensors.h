#pragma once

#include "../mhi_ac_ctrl.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiBinarySensors : public Component, public Parented<MhiAcCtrl> {
 public:
  void setup() override;
  void dump_config() override;

  void set_power(binary_sensor::BinarySensor* sensor) {
    this->power_sensor_ = sensor;
  }

  void set_defrost(binary_sensor::BinarySensor* sensor) {
    this->defrost_sensor_ = sensor;
  }

  void set_vanes_3d_auto_enabled(binary_sensor::BinarySensor* sensor) {
    this->vanes_3d_auto_enabled_sensor_ = sensor;
  }

 protected:
  binary_sensor::BinarySensor* power_sensor_{nullptr};
  binary_sensor::BinarySensor* defrost_sensor_{nullptr};
  binary_sensor::BinarySensor* vanes_3d_auto_enabled_sensor_{nullptr};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome