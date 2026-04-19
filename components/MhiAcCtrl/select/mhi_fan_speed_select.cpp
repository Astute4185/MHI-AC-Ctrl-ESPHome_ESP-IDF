#include "esphome/core/log.h"
#include "mhi_fan_speed_select.h"

#include <string_view>

namespace esphome {
namespace mhi {

namespace {
static constexpr const char *const kFanAuto = "Auto";
static constexpr const char *const kFanQuiet = "Quiet";
static constexpr const char *const kFanLow = "Low";
static constexpr const char *const kFanMedium = "Medium";
static constexpr const char *const kFanHigh = "High";

int fan_code_from_name(std::string_view value) {
    if (value == kFanQuiet) {
        return 0;
    }
    if (value == kFanLow) {
        return 1;
    }
    if (value == kFanMedium) {
        return 2;
    }
    if (value == kFanHigh) {
        return 6;
    }
    return 7;
}

const char *fan_name_from_status(int value) {
    switch (value) {
        case 0: return kFanQuiet;
        case 1: return kFanLow;
        case 2: return kFanMedium;
        case 6: return kFanHigh;
        case 7: return kFanAuto;
        default: return kFanAuto;
    }
}
}  // namespace

static const char *const TAG = "mhi.select";

void MhiFanSpeedSelect::setup() {
    this->parent_->add_listener(this);
}

void MhiFanSpeedSelect::dump_config() {
    ESP_LOGCONFIG(TAG, "MHI Fan Speed Select");
}

void MhiFanSpeedSelect::control(const std::string &value) {
    this->parent_->set_fan(fan_code_from_name(value));
    this->publish_state(value);
}

void MhiFanSpeedSelect::update_status(ACStatus status, int value) {
    if (status != status_fan) {
        return;
    }

    const char *fan_mode = fan_name_from_status(value);
    this->publish_state(fan_mode);
    ESP_LOGV(TAG, "Fan speed status updated to: %s", fan_mode);
}

}  // namespace mhi
}  // namespace esphome