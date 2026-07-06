#include "mhi_climate.h"

#include <cmath>

#include "esphome/core/log.h"
#include "esphome/core/version.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi.climate";

void MhiClimate::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MHI climate entity");

  this->mode = climate::CLIMATE_MODE_OFF;
  this->fan_mode = climate::CLIMATE_FAN_AUTO;
  this->swing_mode = climate::CLIMATE_SWING_OFF;
  this->current_temperature = NAN;

  float restored_target_temperature = NAN;
  auto restore = this->restore_state_();

  if (restore.has_value()) {
    restored_target_temperature = restore->target_temperature;
  }

  if (!std::isnan(restored_target_temperature)) {
    this->target_temperature = restored_target_temperature;
  } else {
    this->target_temperature = 20.0f;
  }

  if (this->parent_ != nullptr) {
    this->parent_->set_climate_target(this);
  }

  this->publish_state();
}

void MhiClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "MHI Climate");
  ESP_LOGCONFIG(TAG, "  Min temperature: %.1f°C", this->minimum_temperature_);
  ESP_LOGCONFIG(TAG, "  Max temperature: %.1f°C", this->maximum_temperature_);
  ESP_LOGCONFIG(TAG, "  Temperature step: %.1f°C", this->temperature_step_);
  ESP_LOGCONFIG(TAG, "  Temperature offset enabled: %s", this->temperature_offset_enabled_ ? "YES" : "NO");
}

void MhiClimate::control(const climate::ClimateCall& call) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Ignoring climate command because parent is not set");
    return;
  }

  auto& command = this->parent_->state().command();
  bool had_change = false;

  if (call.get_target_temperature().has_value()) {
    const float requested_target = *call.get_target_temperature();

    float ac_target = requested_target;

    // Preserve the old concept:
    // if offset mode is enabled, round the AC setpoint upward and keep the user-facing
    // requested target locally. The actual DB3 room-temp-offset behaviour should be
    // wired later in the state/command layer, not inside the climate entity.
    if (this->temperature_offset_enabled_) {
      ac_target = std::ceil(requested_target);

      if (ac_target < kAcMinimumSetpointC) {
        ac_target = kAcMinimumSetpointC;
      }

      this->temperature_offset_ = ac_target - requested_target;
    } else {
      this->temperature_offset_ = 0.0f;
    }

    command.target_temp_set = true;
    command.target_temp_c = ac_target;

    had_change = true;

    ESP_LOGD(TAG, "Requested target %.1f°C, AC setpoint %.1f°C, offset %.1f°C", requested_target, ac_target,
             this->temperature_offset_);
  }

  if (call.get_mode().has_value()) {
    const auto requested_mode = *call.get_mode();

    if (requested_mode == climate::CLIMATE_MODE_OFF) {
      command.power_set = true;
      command.power = false;
    } else {
      command.power_set = true;
      command.power = true;

      command.mode_set = true;
      command.mode = this->mode_to_mhi_(requested_mode);
    }

    had_change = true;
  }

  if (call.get_fan_mode().has_value()) {
    const auto requested_fan = *call.get_fan_mode();

    command.fan_set = true;
    command.fan = this->fan_to_mhi_(requested_fan);

    had_change = true;
  }

  if (call.get_swing_mode().has_value()) {
    const auto requested_swing = *call.get_swing_mode();

    this->apply_swing_command_(requested_swing);

    had_change = true;
  }

  if (had_change) {
    ESP_LOGD(TAG, "Climate command staged; waiting for confirmed MOSI state");
  }
}

uint8_t MhiClimate::mode_to_mhi_(climate::ClimateMode mode) const {
  switch (mode) {
    case climate::CLIMATE_MODE_DRY:
      return 1U;

    case climate::CLIMATE_MODE_COOL:
      return 2U;

    case climate::CLIMATE_MODE_FAN_ONLY:
      return 3U;

    case climate::CLIMATE_MODE_HEAT:
      return 4U;

    case climate::CLIMATE_MODE_HEAT_COOL:
    default:
      return 0U;
  }
}

uint8_t MhiClimate::fan_to_mhi_(climate::ClimateFanMode fan) const {
  switch (fan) {
    case climate::CLIMATE_FAN_LOW:
      return 1U;

    case climate::CLIMATE_FAN_MEDIUM:
      return 2U;

    case climate::CLIMATE_FAN_HIGH:
      return 6U;

    case climate::CLIMATE_FAN_AUTO:
    default:
      return 7U;
  }
}

void MhiClimate::apply_swing_command_(climate::ClimateSwingMode swing) {
  if (this->parent_ == nullptr) {
    return;
  }

  auto& command = this->parent_->state().command();

  switch (swing) {
    case climate::CLIMATE_SWING_OFF:
      command.vertical_vane_set = true;
      command.vertical_vane = this->last_vertical_vane_position_;

      command.horizontal_vane_set = true;
      command.horizontal_vane = this->last_horizontal_vane_position_;
      break;

    case climate::CLIMATE_SWING_VERTICAL:
      command.vertical_vane_set = true;
      command.vertical_vane = 5U;  // TX builder treats 5 as vertical swing.
      break;

    case climate::CLIMATE_SWING_HORIZONTAL:
      command.horizontal_vane_set = true;
      command.horizontal_vane = 8U;  // TX builder treats 8 as horizontal swing.
      break;

    case climate::CLIMATE_SWING_BOTH:
    default:
      command.vertical_vane_set = true;
      command.vertical_vane = 5U;

      command.horizontal_vane_set = true;
      command.horizontal_vane = 8U;
      break;
  }
}

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
climate::ClimateTraits MhiClimate::traits() {
  auto traits = climate::ClimateTraits();

  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);

  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_HEAT_COOL,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_FAN_ONLY,
  });

  traits.set_visual_min_temperature(this->minimum_temperature_);
  traits.set_visual_max_temperature(this->maximum_temperature_);
  traits.set_visual_temperature_step(this->temperature_step_);

  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });

  traits.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
  });

  return traits;
}
#else
climate::ClimateTraits MhiClimate::traits() {
  auto traits = climate::ClimateTraits();

  traits.set_supports_current_temperature(true);

  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_HEAT_COOL,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_FAN_ONLY,
  });

  traits.set_supports_two_point_target_temperature(false);

  traits.set_visual_min_temperature(this->minimum_temperature_);
  traits.set_visual_max_temperature(this->maximum_temperature_);
  traits.set_visual_temperature_step(this->temperature_step_);

  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });

  traits.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
  });

  return traits;
}
#endif

}  // namespace mhi_ac_ctrl
}  // namespace esphome