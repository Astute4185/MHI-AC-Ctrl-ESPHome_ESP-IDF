#include "esphome/core/log.h"
#include "mhi_horizontal_vanes_select.h"

namespace esphome {
namespace mhi {

namespace {
static constexpr const char *const kHorizontalVanes[] = {
    "Left",
    "Left/Center",
    "Center",
    "Center/Right",
    "Right",
    "Wide",
    "Spot",
    "Swing",
};

static constexpr int kHorizontalVanesCount =
    static_cast<int>(sizeof(kHorizontalVanes) / sizeof(kHorizontalVanes[0]));
}  // namespace

static const char *const TAG = "mhi.select";

void MhiHorizontalVanesSelect::setup() {
    this->parent_->add_listener(this);
}

void MhiHorizontalVanesSelect::dump_config() {
    ESP_LOGCONFIG(TAG, "MHI Horizontal Vanes Select");
}

void MhiHorizontalVanesSelect::control(const std::string &value) {
    auto idx = this->index_of(value);
    if (idx.has_value() && idx.value() < kHorizontalVanesCount) {
        this->parent_->set_vanesLR(idx.value() + 1);
    }
    this->publish_state(value);
}

void MhiHorizontalVanesSelect::update_status(ACStatus status, int value) {
    if (status != status_vanesLR) {
        return;
    }

    const int index = value - 1;
    if (index >= 0 && index < kHorizontalVanesCount) {
        this->publish_state(kHorizontalVanes[index]);
        ESP_LOGV(TAG, "Horizontal vanes status updated");
    } else {
        ESP_LOGW(TAG, "Failed to map horizontal vanes status, value %i", value);
    }
}

}  // namespace mhi
}  // namespace esphome