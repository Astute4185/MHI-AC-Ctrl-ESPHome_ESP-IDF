#pragma once

#include "../mhi_ac_ctrl.h"
#include "esphome/components/select/select.h"
#include "esphome/core/component.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiFanSpeedSelect : public Component, public select::Select, public Parented<MhiAcCtrl> {
 public:
  void setup() override;
  void dump_config() override;

 protected:
  void control(const std::string& value) override;

 private:
  static bool fan_code_from_name_(MhiFanProfile profile, const std::string& value, uint8_t& out);
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome