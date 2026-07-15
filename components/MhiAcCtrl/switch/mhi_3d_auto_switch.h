#pragma once

#include "../mhi_ac_ctrl.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome {
namespace mhi_ac_ctrl {

class Mhi3dAutoSwitch : public switch_::Switch, public Component, public Parented<MhiAcCtrl> {
 public:
  void setup() override;
  void dump_config() override;

 protected:
  void write_state(bool state) override;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome