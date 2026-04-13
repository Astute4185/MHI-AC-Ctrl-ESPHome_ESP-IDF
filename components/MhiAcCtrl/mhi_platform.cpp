#include "mhi_platform.h"

int SCK_PIN = 14;
int MOSI_PIN = 13;
int MISO_PIN = 12;

namespace esphome {
namespace mhi {

static const char *TAG = "mhi.platform";

void MhiPlatform::setup() {

  if (this->sck_pin_ >= 0) {
    SCK_PIN = this->sck_pin_;
  }
  if (this->mosi_pin_ >= 0) {
    MOSI_PIN = this->mosi_pin_;
  }
  if (this->miso_pin_ >= 0) {
    MISO_PIN = this->miso_pin_;
  }

  this->mhi_ac_ctrl_core_.MHIAcCtrlStatus(this);
  this->mhi_ac_ctrl_core_.init();
  this->mhi_ac_ctrl_core_.set_frame_size(this->frame_size_);

  if (this->external_temperature_sensor_ != nullptr) {
    this->external_temperature_sensor_->add_on_state_callback(
        [this](float state) { this->transfer_room_temperature(state); });

    this->transfer_room_temperature(this->external_temperature_sensor_->state);
  }

  this->room_temp_api_timeout_start_ = millis();  // FIX: moved from header
}

void MhiPlatform::set_frame_size(int framesize) {
  this->frame_size_ = framesize;
}

void MhiPlatform::set_room_temp_api_timeout(int time_in_seconds) {
  this->room_temp_api_timeout_ = static_cast<uint32_t>(time_in_seconds);
}

void MhiPlatform::set_external_room_temperature_sensor(sensor::Sensor *sensor) {
  this->external_temperature_sensor_ = sensor;
}

void MhiPlatform::loop() {
  if (this->external_temperature_sensor_ != nullptr) {
    this->transfer_room_temperature(this->external_temperature_sensor_->state);
    this->room_temp_api_active_ = false;
  }

  if (this->room_temp_api_active_ &&
      millis() - this->room_temp_api_timeout_start_ >=
          this->room_temp_api_timeout_ * 1000) {

    mhi_ac_ctrl_core_.set_troom(0xff);
    ESP_LOGD(TAG,
             "did not receive a room_temp_api value, using IU temperature sensor");
    this->room_temp_api_active_ = false;
  }

  int ret = mhi_ac_ctrl_core_.loop(100);
  if (ret < 0) {
    ESP_LOGE(TAG, "mhi_ac_ctrl_core loop error: %i", ret);
  }
}

void MhiPlatform::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Platform");

  if (external_temperature_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  external_temperature_sensor enabled!");
  }

  ESP_LOGCONFIG(TAG, "  frame_size: %d", this->frame_size_);
  ESP_LOGCONFIG(TAG, "  room_temp_api_timeout: %u",
                this->room_temp_api_timeout_);
  ESP_LOGCONFIG(TAG, "  listeners count: %d",
                static_cast<int>(this->listeners_.size()));
}

void MhiPlatform::cbiStatusFunction(ACStatus status, int value) {
  ESP_LOGD(TAG, "received status=%i value=%i", status, value);

  for (MhiStatusListener *listener : this->listeners_) {
    listener->update_status(status, value);
  }
}

void MhiPlatform::set_room_temperature(float value) {
  this->room_temp_api_timeout_start_ = millis();
  this->room_temp_api_active_ = true;
  this->transfer_room_temperature(value);
}

bool MhiPlatform::get_room_temp_api_active() {
  return this->room_temp_api_active_;
}

float MhiPlatform::get_room_temp_offset() {
  return this->temperature_offset_;
}

void MhiPlatform::transfer_room_temperature(float value) {
  if (std::isnan(value)) {
    if (!std::isnan(this->last_room_temperature_)) {
      ESP_LOGD(TAG,
               "set room_temp_api: value is NaN, using internal sensor");
      mhi_ac_ctrl_core_.set_troom(0xff);
      this->last_room_temperature_ = NAN;
    }
    return;
  }

  if (this->temperature_offset_ > 0.0f) {
    value += this->temperature_offset_;
  }

  if (std::fabs(value - last_room_temperature_) < 0.01f) {
    return;
  }

  // FIX: logical operator (was bitwise &)
  if ((value > -10.0f) && (value < 48.0f)) {
    uint8_t tmp = static_cast<uint8_t>(value * 4 + 61);
    this->mhi_ac_ctrl_core_.set_troom(tmp);

    this->last_room_temperature_ = value;

    ESP_LOGD(TAG, "set room_temp_api: %f %i", value, tmp);
  }
}

void MhiPlatform::set_power(ACPower value) {
  this->mhi_ac_ctrl_core_.set_power(value);
}

void MhiPlatform::set_fan(int value) {
  this->mhi_ac_ctrl_core_.set_fan(static_cast<uint32_t>(value));
}

void MhiPlatform::set_mode(ACMode value) {
  this->mhi_ac_ctrl_core_.set_mode(value);
}

void MhiPlatform::set_tsetpoint(float value) {
  this->mhi_ac_ctrl_core_.set_tsetpoint(
      static_cast<uint32_t>(value * 2));

  ESP_LOGD(TAG, "set setpoint: %f", value);

  if (this->room_temp_api_active_ &&
      !std::isnan(this->last_room_temperature_)) {

    float last = this->last_room_temperature_;
    this->last_room_temperature_ = NAN;

    this->transfer_room_temperature(last);

    ESP_LOGD(TAG, "resending external troom: %f", last);
  }
}

void MhiPlatform::set_offset(float value) {
  if (value < 0.01f) {
    this->last_room_temperature_ -= this->temperature_offset_;
  }

  this->temperature_offset_ = value;
  ESP_LOGD(TAG, "set temperature offset: %f", value);
}

void MhiPlatform::set_vanes(int value) {
  this->mhi_ac_ctrl_core_.set_vanes(static_cast<uint32_t>(value));
  ESP_LOGD(TAG, "set vanes: %i", value);
}

void MhiPlatform::set_vanesLR(int value) {
  if (this->frame_size_ == 33) {
    ESP_LOGD(TAG, "setting vanesLR to: %i", value);
    this->mhi_ac_ctrl_core_.set_vanesLR(static_cast<uint32_t>(value));
  } else {
    ESP_LOGD(TAG, "Not setting vanesLR: %i", value);
  }
}

void MhiPlatform::set_3Dauto(bool value) {
  if (this->frame_size_ == 33) {
    this->mhi_ac_ctrl_core_.set_3Dauto(
        value ? AC3Dauto::Dauto_on : AC3Dauto::Dauto_off);

    ESP_LOGD(TAG, "set 3D auto: %i", value);
  } else {
    ESP_LOGD(TAG, "Not setting 3D auto: %i (frame_size != 33)", value);
  }
}

void MhiPlatform::add_listener(MhiStatusListener *listener) {
  this->listeners_.push_back(listener);
}

}  // namespace mhi
}  // namespace esphome
