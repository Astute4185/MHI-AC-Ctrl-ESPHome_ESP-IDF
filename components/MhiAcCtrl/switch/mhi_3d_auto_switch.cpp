#include "mhi_3d_auto_switch.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi.switch";

void Mhi3dAutoSwitch::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI 3D auto switch");

  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "MHI 3D auto switch has no parent component");
    return;
  }

  this->parent_->set_vanes_3d_auto_switch(this);
}

void Mhi3dAutoSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI 3D Auto Switch");
  ESP_LOGCONFIG(TAG, "  State: %s", this->state ? "ON" : "OFF");
}

void Mhi3dAutoSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Ignoring 3D auto command because parent is not set");
    return;
  }

  auto& command = this->parent_->state().command();

  command.three_d_auto_set = true;
  command.three_d_auto = state;

  ESP_LOGD(TAG, "3D auto command staged: %s", state ? "ON" : "OFF");
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
