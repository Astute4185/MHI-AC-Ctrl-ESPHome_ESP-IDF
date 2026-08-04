#include "mhi_active_mode_switch.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi.switch";

void MhiActiveModeSwitch::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI Active Mode switch");

  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "MHI Active Mode switch has no parent component");
    return;
  }

  this->parent_->set_active_mode_switch(this);
  this->publish_state(this->parent_->active_mode());
}

void MhiActiveModeSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Active Mode Switch");
  ESP_LOGCONFIG(TAG, "  State: %s", this->state ? "ON" : "OFF");
}

void MhiActiveModeSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Ignoring Active Mode command because parent is not set");
    return;
  }

  this->parent_->set_active_mode(state);
  this->publish_state(this->parent_->active_mode());
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
