#include "mhi_silent_mode_switch.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi.switch";

void MhiOutdoorUnitSilentModeSwitch::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI outdoor-unit Silent Mode switch");

  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "MHI outdoor-unit Silent Mode switch has no parent component");
    return;
  }

  // Keep the protocol probe opt-in: only installations that expose this switch
  // add the 0xC0/0xDD request to the regular opdata cycle.
  this->parent_->add_opdata_mask(MHI_OPDATA_REQ_SILENT_MODE);
}

void MhiOutdoorUnitSilentModeSwitch::loop() {
  if (this->parent_ == nullptr) {
    return;
  }

  const auto& opdata = this->parent_->state().opdata();
  if (!opdata.has_silent_mode) {
    return;
  }

  if (!this->feedback_seen_ || this->feedback_state_ != opdata.silent_mode) {
    this->feedback_seen_ = true;
    this->feedback_state_ = opdata.silent_mode;
    this->publish_state(opdata.silent_mode);
  }
}

void MhiOutdoorUnitSilentModeSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Outdoor Unit Silent Mode Switch");
  ESP_LOGCONFIG(TAG, "  Feedback: %s", this->feedback_seen_ ? (this->feedback_state_ ? "ON" : "OFF") : "not observed");
}

void MhiOutdoorUnitSilentModeSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Ignoring Silent Mode command because parent is not set");
    return;
  }

  const auto& opdata = this->parent_->state().opdata();
  if (opdata.has_silent_mode && opdata.silent_mode == state) {
    return;
  }

  MhiCommandState patch{};
  patch.silent_mode_set = true;
  patch.silent_mode = state;
  const uint32_t accepted = this->parent_->request_command_patch(patch);
  if ((accepted & MHI_COMMAND_SILENT_MODE) != 0U) {
    ESP_LOGD(TAG, "Outdoor-unit Silent Mode command staged: %s", state ? "ON" : "OFF");
  }
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
