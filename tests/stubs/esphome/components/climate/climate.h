#pragma once

#include <optional>

namespace esphome {
namespace climate {

enum ClimateMode {
  CLIMATE_MODE_OFF,
  CLIMATE_MODE_HEAT_COOL,
  CLIMATE_MODE_DRY,
  CLIMATE_MODE_COOL,
  CLIMATE_MODE_FAN_ONLY,
  CLIMATE_MODE_HEAT,
};

enum ClimateFanMode {
  CLIMATE_FAN_AUTO,
  CLIMATE_FAN_QUIET,
  CLIMATE_FAN_LOW,
  CLIMATE_FAN_MEDIUM,
  CLIMATE_FAN_HIGH,
};

enum ClimateSwingMode {
  CLIMATE_SWING_OFF,
  CLIMATE_SWING_VERTICAL,
};

class Climate {
 public:
  void publish_state() {
    this->publish_count++;
  }

  ClimateMode mode{CLIMATE_MODE_OFF};
  float target_temperature{0.0f};
  float current_temperature{0.0f};
  std::optional<ClimateFanMode> fan_mode{};
  ClimateSwingMode swing_mode{CLIMATE_SWING_OFF};
  unsigned int publish_count{0};
};

}  // namespace climate
}  // namespace esphome
