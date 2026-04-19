#include "esphome/core/log.h"
#include "mhi_vertical_vanes_select.h"

namespace esphome {
namespace mhi {

namespace {
static constexpr const char *const kVerticalVanes[] = {
    "Up",
    "Up/Center",
    "Center/Down",
    "Down",
    "Swing",
};

static constexpr int kVerticalVanesCount =
    static_cast<int>(sizeof(kVerticalVanes) / sizeof(kVerticalVanes[0]));
}  // namespace

static const char *const TAG = "mhi.select";

void MhiVerticalVanesSelect::setup() {
    this->parent_->add_listener(this);
}

void MhiVerticalVanesSelect::dump_config() {
    ESP_LOGCONFIG(TAG, "MHI Vertical Vanes Select");
}

void MhiVerticalVanesSelect::control(const std::string &value) {
    auto idx = this->index_of(value);
    if (idx.has_value() && idx.value() < kVerticalVanesCount) {
        this->parent_->set_vanes(idx.value() + 1);
    }
    this->publish_state(value);
}

void MhiVerticalVanesSelect::update_status(ACStatus status, int value) {
    if (status != status_vanes) {
        return;
    }

    const int index = value - 1;
    if (index >= 0 && index < kVerticalVanesCount) {
        this->publish_state(kVerticalVanes[index]);
        ESP_LOGV(TAG, "Vertical vanes status updated");
    } else {
        ESP_LOGW(TAG, "Failed to map vertical vanes status, value %i", value);
    }
}

}  // namespace mhi
}  // namespace esphome