#include "mhi_text_sensors.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mhi {

namespace {
static constexpr const char *const kProtectionStates[] = {
    "Normal",
    "Discharge pipe temperature protection control",
    "Discharge pipe temperature anomaly",
    "Current safe control of inverter primary current",
    "High pressure protection control",
    "High pressure anomaly",
    "Low pressure protection control",
    "Low pressure anomaly",
    "Anti-frost prevention control",
    "Current cut",
    "Power transistor protection control",
    "Power transistor anomaly (Overheat)",
    "Compression ratio control",
    "-",
    "Condensation prevention control",
    "Current safe control of inverter secondary current",
    "Stop by compressor rotor lock",
    "Stop by compressor startup failure",
};

static constexpr int kProtectionStateCount =
    static_cast<int>(sizeof(kProtectionStates) / sizeof(kProtectionStates[0]));
}  // namespace

static const char *TAG = "mhi.text_sensor";

void MhiTextSensors::set_protection_state(text_sensor::TextSensor *sensor) {
    this->protection_state_ = sensor;
}

void MhiTextSensors::setup() {
    this->parent_->add_listener(this);
}

void MhiTextSensors::dump_config() {
    ESP_LOGCONFIG(TAG, "MHI Text Sensors");
    if (this->protection_state_ != nullptr) {
        ESP_LOGCONFIG(TAG, "  protection_state: %s", this->protection_state_->state.c_str());
    }
}

void MhiTextSensors::update_status(ACStatus status, int value) {
    if (status != opdata_protection_no || this->protection_state_ == nullptr) {
        return;
    }

    if (value >= 0 && value < kProtectionStateCount) {
        this->protection_state_->publish_state(kProtectionStates[value]);
        ESP_LOGV(TAG, "Protection status updated");
    }
}

}  // namespace mhi
}  // namespace esphome