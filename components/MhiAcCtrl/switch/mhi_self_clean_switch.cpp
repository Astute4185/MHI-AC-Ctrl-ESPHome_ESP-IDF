#include "mhi_self_clean_switch.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi.switch";

void MhiSelfCleanSwitch::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI Self Clean switch (feedback only)");

  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "MHI Self Clean switch has no parent component");
    return;
  }

  this->parent_->set_self_clean_switch(this);
}

void MhiSelfCleanSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Self Clean Switch");
  ESP_LOGCONFIG(TAG, "  Control: feedback only; setter not yet identified");
}

void MhiSelfCleanSwitch::write_state(bool state) {
  (void)state;
  ESP_LOGW(TAG, "Ignoring Self Clean command: protocol setter has not been identified yet");
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
