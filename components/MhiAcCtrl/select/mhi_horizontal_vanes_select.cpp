#include "mhi_horizontal_vanes_select.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi.select.horizontal_vanes";

void MhiHorizontalVanesSelect::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI horizontal vanes select");

  if (this->parent_ != nullptr) {
    this->parent_->set_horizontal_vanes_select(this);
  }
}

void MhiHorizontalVanesSelect::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Horizontal Vanes Select");
}

void MhiHorizontalVanesSelect::control(const std::string& value) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Ignoring horizontal vanes command because parent is not set");
    return;
  }

  auto index = this->index_of(value);
  if (!index.has_value()) {
    ESP_LOGW(TAG, "Unknown horizontal vanes option: %s", value.c_str());
    return;
  }

  auto& command = this->parent_->state().command();

  command.horizontal_vane_set = true;
  command.horizontal_vane = static_cast<uint8_t>(index.value() + 1U);

  ESP_LOGD(TAG, "Horizontal vanes command staged: %s", value.c_str());
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
