#include "mhi_platform.h"

#include <cmath>
#include <cstdint>

#include "esphome/components/sensor/sensor.h"
#include "mhi_time.h"
#include "mhi_diagnostics.h"

#include "esphome/core/log.h"
namespace {
constexpr int kDefaultSckPin = 14;
constexpr int kDefaultMosiPin = 13;
constexpr int kDefaultMisoPin = 12;

int resolve_pin(int configured_pin, int fallback_pin) {
  return configured_pin >= 0 ? configured_pin : fallback_pin;
}
}  // namespace

namespace esphome {
namespace mhi {

static const char *TAG = "mhi.platform";

void MhiPlatform::setup() {
  MhiTransportConfig config{};
  config.sck_pin = resolve_pin(this->sck_pin_, kDefaultSckPin);
  config.mosi_pin = resolve_pin(this->mosi_pin_, kDefaultMosiPin);
  config.miso_pin = resolve_pin(this->miso_pin_, kDefaultMisoPin);
  config.frame_size_hint = static_cast<uint8_t>(this->frame_size_);

  ESP_LOGCONFIG(TAG, "Using transport backend: mhi_transport");
  ESP_LOGCONFIG(
      TAG,
      "Resolved transport config: sck=%d mosi=%d miso=%d frame_size_hint=%u",
      config.sck_pin,
      config.mosi_pin,
      config.miso_pin,
      config.frame_size_hint);

  this->mhi_ac_ctrl_core_.set_transport(&this->transport_);
  this->mhi_ac_ctrl_core_.set_transport_config(config);

  this->mhi_ac_ctrl_core_.MHIAcCtrlStatus(this);
  this->mhi_ac_ctrl_core_.init();
  this->mhi_ac_ctrl_core_.set_frame_size(static_cast<uint8_t>(this->frame_size_));

  this->mhi_ac_ctrl_core_.set_enabled_opdata_mask(this->opdata_mask_);

  if (this->external_temperature_sensor_ != nullptr) {
    this->external_temperature_sensor_->add_on_state_callback(
        [this](float state) { this->transfer_room_temperature(state); });

    this->transfer_room_temperature(this->external_temperature_sensor_->state);
  }

  this->room_temp_api_timeout_start_ = mhi_now_ms();
}

void MhiPlatform::set_frame_size(int framesize) {
  this->frame_size_ = framesize;
}

void MhiPlatform::set_room_temp_api_timeout(int time_in_seconds) {
  this->room_temp_api_timeout_ = static_cast<uint32_t>(time_in_seconds);
}

void MhiPlatform::set_external_room_temperature_sensor(
    esphome::sensor::Sensor *sensor) {
  this->external_temperature_sensor_ = sensor;
}

void MhiPlatform::loop() {
  if (this->room_temp_api_active_ &&
      mhi_now_ms() - this->room_temp_api_timeout_start_ >=
          this->room_temp_api_timeout_ * 1000U) {
    this->mhi_ac_ctrl_core_.set_troom(0xff);
    ESP_LOGD(TAG, "did not receive a room_temp_api value, using IU temperature sensor");
    this->room_temp_api_active_ = false;
  }

  const int ret = this->mhi_ac_ctrl_core_.loop(100);
  if (ret < 0) {
    esphome::mhi::mhi_log_platform_loop_status(
        TAG,
        {ret, this->mhi_ac_ctrl_core_.get_last_diag_reason()});
  }
}

void MhiPlatform::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Platform");

  if (this->external_temperature_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  external_temperature_sensor enabled!");
  }

  ESP_LOGCONFIG(TAG, "  frame_size_hint: %d", this->frame_size_);
  ESP_LOGCONFIG(TAG, "  room_temp_api_timeout: %u", this->room_temp_api_timeout_);
  ESP_LOGCONFIG(TAG, "  transport_backend: mhi_transport");
  ESP_LOGCONFIG(TAG, "  resolved_sck_pin: %d", resolve_pin(this->sck_pin_, kDefaultSckPin));
  ESP_LOGCONFIG(TAG, "  resolved_mosi_pin: %d", resolve_pin(this->mosi_pin_, kDefaultMosiPin));
  ESP_LOGCONFIG(TAG, "  resolved_miso_pin: %d", resolve_pin(this->miso_pin_, kDefaultMisoPin));
  ESP_LOGCONFIG(TAG, "  listeners count: %d", static_cast<int>(this->listeners_.size()));
}

void MhiPlatform::cbiStatusFunction(ACStatus status, int value) {
  ESP_LOGV(TAG, "received status=%i value=%i", status, value);

  for (MhiStatusListener *listener : this->listeners_) {
    listener->update_status(status, value);
  }
}

void MhiPlatform::set_room_temperature(float value) {
  this->room_temp_api_timeout_start_ = mhi_now_ms();
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
      ESP_LOGV(TAG, "set room_temp_api: value is NaN, using internal sensor");
      this->mhi_ac_ctrl_core_.set_troom(0xff);
      this->last_room_temperature_ = NAN;
    }
    return;
  }

  if (this->temperature_offset_ > 0.0f) {
    value += this->temperature_offset_;
  }

  if (std::fabs(value - this->last_room_temperature_) < 0.01f) {
    return;
  }

  if ((value > -10.0f) && (value < 48.0f)) {
    const uint8_t tmp = static_cast<uint8_t>(value * 4 + 61);
    this->mhi_ac_ctrl_core_.set_troom(tmp);
    this->last_room_temperature_ = value;
    ESP_LOGV(TAG, "set room_temp_api: %f %i", value, tmp);
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
  this->mhi_ac_ctrl_core_.set_tsetpoint(static_cast<uint32_t>(value * 2));

  ESP_LOGD(TAG, "set setpoint: %f", value);

  if (this->room_temp_api_active_ && !std::isnan(this->last_room_temperature_)) {
    const float last = this->last_room_temperature_;
    this->last_room_temperature_ = NAN;
    this->transfer_room_temperature(last);
    ESP_LOGV(TAG, "resending external troom: %f", last);
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