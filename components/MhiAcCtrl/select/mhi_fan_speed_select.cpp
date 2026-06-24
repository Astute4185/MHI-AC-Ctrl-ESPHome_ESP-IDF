#include "mhi_fan_speed_select.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi.select.fan";

namespace {

static constexpr const char* const kFanAuto = "Auto";
static constexpr const char* const kFanQuiet = "Quiet";
static constexpr const char* const kFanLow = "Low";
static constexpr const char* const kFanMedium = "Medium";
static constexpr const char* const kFanHigh = "High";

}  // namespace

void MhiFanSpeedSelect::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI fan speed select");

  if (this->parent_ != nullptr) {
    this->parent_->set_fan_speed_select(this);
  }
}

void MhiFanSpeedSelect::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Fan Speed Select");
}

void MhiFanSpeedSelect::control(const std::string& value) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Ignoring fan speed command because parent is not set");
    return;
  }

  auto& command = this->parent_->state().command();

  command.fan_set = true;
  command.fan = fan_code_from_name_(value);

  ESP_LOGD(TAG, "Fan speed command staged: %s; waiting for confirmed MOSI state", value.c_str());
}

uint8_t MhiFanSpeedSelect::fan_code_from_name_(const std::string& value) {
  if (value == kFanQuiet) {
    return 0U;
  }

  if (value == kFanLow) {
    return 1U;
  }

  if (value == kFanMedium) {
    return 2U;
  }

  if (value == kFanHigh) {
    return 6U;
  }

  return 7U;  // Auto
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome