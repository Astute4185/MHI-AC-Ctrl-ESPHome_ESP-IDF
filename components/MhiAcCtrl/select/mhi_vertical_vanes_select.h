#pragma once

#include "../mhi_ac_ctrl.h"
#include "esphome/components/select/select.h"
#include "esphome/core/component.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiVerticalVanesSelect : public Component, public select::Select, public Parented<MhiAcCtrl> {
 public:
  void setup() override;
  void dump_config() override;

 protected:
  void control(const std::string& value) override;
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome