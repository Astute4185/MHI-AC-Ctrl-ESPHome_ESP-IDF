#pragma once

#include <cstdint>
#include <string>

namespace esphome {
namespace mhi_ac_ctrl {

enum class MhiFanProfile : uint8_t {
  THREE_SPEED = 0,
  FOUR_SPEED = 1,
};

enum class MhiFanMode : uint8_t {
  UNKNOWN = 0,
  QUIET,
  LOW,
  MEDIUM,
  HIGH,
  AUTO,
};

inline MhiFanProfile mhi_fan_profile_from_name(const std::string& name) {
  if (name == "three_speed") {
    return MhiFanProfile::THREE_SPEED;
  }
  return MhiFanProfile::FOUR_SPEED;
}

inline const char* mhi_fan_profile_name(MhiFanProfile profile) {
  return profile == MhiFanProfile::FOUR_SPEED ? "four_speed" : "three_speed";
}

inline bool mhi_fan_profile_supports_quiet(MhiFanProfile profile) {
  return profile == MhiFanProfile::FOUR_SPEED;
}

inline MhiFanMode mhi_fan_mode_from_code(MhiFanProfile profile, uint8_t code) {
  switch (code) {
    case 0U:
      return mhi_fan_profile_supports_quiet(profile) ? MhiFanMode::QUIET : MhiFanMode::LOW;
    case 1U:
      return MhiFanMode::LOW;
    case 2U:
      return MhiFanMode::MEDIUM;
    case 6U:
      return MhiFanMode::HIGH;
    case 7U:
      return MhiFanMode::AUTO;
    default:
      return MhiFanMode::UNKNOWN;
  }
}

inline bool mhi_fan_code_from_mode(MhiFanProfile profile, MhiFanMode mode, uint8_t& out) {
  switch (mode) {
    case MhiFanMode::QUIET:
      if (!mhi_fan_profile_supports_quiet(profile)) {
        return false;
      }
      out = 0U;
      return true;
    case MhiFanMode::LOW:
      out = 1U;
      return true;
    case MhiFanMode::MEDIUM:
      out = 2U;
      return true;
    case MhiFanMode::HIGH:
      out = 6U;
      return true;
    case MhiFanMode::AUTO:
      out = 7U;
      return true;
    case MhiFanMode::UNKNOWN:
    default:
      return false;
  }
}

inline const char* mhi_fan_mode_name(MhiFanMode mode) {
  switch (mode) {
    case MhiFanMode::QUIET:
      return "Quiet";
    case MhiFanMode::LOW:
      return "Low";
    case MhiFanMode::MEDIUM:
      return "Medium";
    case MhiFanMode::HIGH:
      return "High";
    case MhiFanMode::AUTO:
      return "Auto";
    case MhiFanMode::UNKNOWN:
    default:
      return nullptr;
  }
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
