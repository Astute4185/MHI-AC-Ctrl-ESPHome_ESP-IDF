#pragma once

#include "../mhi_ac_ctrl.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiOutdoorUnitSilentModeSwitch : public switch_::Switch, public Component, public Parented<MhiAcCtrl> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  void write_state(bool state) override;

  bool feedback_seen_{false};
  bool feedback_state_{false};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
