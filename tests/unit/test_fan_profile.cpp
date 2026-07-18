#include "mhi_test_common.h"

namespace mhi_unit_tests {

void fan_profile_defaults_to_four_speed() {
  EXPECT_EQ(static_cast<uint8_t>(mhi_fan_profile_from_name("four_speed")),
            static_cast<uint8_t>(MhiFanProfile::FOUR_SPEED));
  EXPECT_EQ(static_cast<uint8_t>(mhi_fan_profile_from_name("unknown")),
            static_cast<uint8_t>(MhiFanProfile::FOUR_SPEED));
  EXPECT_TRUE(mhi_fan_profile_supports_quiet(MhiFanProfile::FOUR_SPEED));
  EXPECT_TRUE(std::string(mhi_fan_profile_name(MhiFanProfile::FOUR_SPEED)) == "four_speed");
}

void fan_profile_three_speed_collapses_code_zero_to_low() {
  EXPECT_EQ(static_cast<uint8_t>(mhi_fan_profile_from_name("three_speed")),
            static_cast<uint8_t>(MhiFanProfile::THREE_SPEED));
  EXPECT_FALSE(mhi_fan_profile_supports_quiet(MhiFanProfile::THREE_SPEED));
  EXPECT_EQ(static_cast<uint8_t>(mhi_fan_mode_from_code(MhiFanProfile::THREE_SPEED, 0U)),
            static_cast<uint8_t>(MhiFanMode::LOW));
  EXPECT_TRUE(std::string(mhi_fan_mode_name(mhi_fan_mode_from_code(MhiFanProfile::THREE_SPEED, 0U))) == "Low");
}

void fan_profile_four_speed_exposes_code_zero_as_quiet() {
  EXPECT_EQ(static_cast<uint8_t>(mhi_fan_mode_from_code(MhiFanProfile::FOUR_SPEED, 0U)),
            static_cast<uint8_t>(MhiFanMode::QUIET));
  EXPECT_TRUE(std::string(mhi_fan_mode_name(mhi_fan_mode_from_code(MhiFanProfile::FOUR_SPEED, 0U))) == "Quiet");
}

void fan_profile_encodes_quiet_only_for_four_speed() {
  uint8_t code = 0xFFU;
  EXPECT_FALSE(mhi_fan_code_from_mode(MhiFanProfile::THREE_SPEED, MhiFanMode::QUIET, code));
  EXPECT_EQ(code, 0xFFU);
  EXPECT_TRUE(mhi_fan_code_from_mode(MhiFanProfile::FOUR_SPEED, MhiFanMode::QUIET, code));
  EXPECT_EQ(code, 0U);
}

}  // namespace mhi_unit_tests
