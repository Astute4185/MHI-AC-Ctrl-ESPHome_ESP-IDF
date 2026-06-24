#include "mhi_test_common.h"

namespace mhi_unit_tests {

void command_confirmation_confirms_power_mode_and_vertical_vane() {
  MhiCommandConfirmation confirmation{};

  MhiCommandIntent intent{};
  intent.mask = MHI_COMMAND_POWER | MHI_COMMAND_MODE | MHI_COMMAND_VERTICAL_VANE;
  intent.power = true;
  intent.mode = 3U;
  intent.vertical_vane = 3U;

  confirmation.stage(intent, intent.mask, 1000U);
  EXPECT_EQ(confirmation.pending_mask(), intent.mask);

  MhiStatusState status{};
  status.valid = true;
  status.power = true;
  status.mode = 3U;
  status.vertical_vane = 3U;
  status.vanes_swing = false;

  const uint32_t confirmed = confirmation.observe_status(status);
  EXPECT_EQ(confirmed, intent.mask);
  EXPECT_FALSE(confirmation.has_pending());
}

void command_confirmation_keeps_partial_pending_until_later_status() {
  MhiCommandConfirmation confirmation{};

  MhiCommandIntent intent{};
  intent.mask = MHI_COMMAND_POWER | MHI_COMMAND_MODE;
  intent.power = true;
  intent.mode = 2U;

  confirmation.stage(intent, intent.mask, 1000U);

  MhiStatusState status{};
  status.valid = true;
  status.power = true;
  status.mode = 1U;

  EXPECT_EQ(confirmation.observe_status(status), static_cast<uint32_t>(MHI_COMMAND_POWER));
  EXPECT_EQ(confirmation.pending_mask(), static_cast<uint32_t>(MHI_COMMAND_MODE));

  status.mode = 2U;
  EXPECT_EQ(confirmation.observe_status(status), static_cast<uint32_t>(MHI_COMMAND_MODE));
  EXPECT_FALSE(confirmation.has_pending());
}

void command_confirmation_ignores_auto_fan_because_mosi_does_not_confirm_it() {
  MhiCommandConfirmation confirmation{};

  MhiCommandIntent intent{};
  intent.mask = MHI_COMMAND_FAN;
  intent.fan = 7U;

  confirmation.stage(intent, intent.mask, 1000U);
  EXPECT_FALSE(confirmation.has_pending());
}

void command_confirmation_confirms_supported_fan_codes() {
  MhiCommandConfirmation confirmation{};

  MhiCommandIntent intent{};
  intent.mask = MHI_COMMAND_FAN;
  intent.fan = 6U;

  confirmation.stage(intent, intent.mask, 1000U);
  EXPECT_EQ(confirmation.pending_mask(), static_cast<uint32_t>(MHI_COMMAND_FAN));

  MhiStatusState status{};
  status.valid = true;
  status.fan = 4U;

  EXPECT_EQ(confirmation.observe_status(status), static_cast<uint32_t>(MHI_COMMAND_FAN));
  EXPECT_FALSE(confirmation.has_pending());
}

void command_confirmation_times_out_unconfirmed_commands() {
  MhiCommandConfirmation confirmation{};

  MhiCommandIntent intent{};
  intent.mask = MHI_COMMAND_TARGET_TEMP;
  intent.target_temp_c = 22.5f;

  confirmation.stage(intent, intent.mask, 1000U);
  EXPECT_EQ(confirmation.expire(1000U + kMhiCommandConfirmationTimeoutMs - 1U), 0U);
  EXPECT_EQ(confirmation.expire(1000U + kMhiCommandConfirmationTimeoutMs),
            static_cast<uint32_t>(MHI_COMMAND_TARGET_TEMP));
  EXPECT_FALSE(confirmation.has_pending());
}

}  // namespace mhi_unit_tests

namespace mhi_unit_tests {

void command_confirmation_detects_duplicate_pending_commands() {
  MhiCommandConfirmation confirmation{};

  MhiCommandIntent intent{};
  intent.mask = MHI_COMMAND_FAN | MHI_COMMAND_TARGET_TEMP;
  intent.fan = 6U;
  intent.target_temp_c = 22.0f;

  confirmation.stage(intent, intent.mask, 1000U);

  MhiCommandState duplicate{};
  duplicate.fan_set = true;
  duplicate.fan = 6U;
  duplicate.target_temp_set = true;
  duplicate.target_temp_c = 22.0f;
  duplicate.mode_set = true;
  duplicate.mode = 2U;

  EXPECT_EQ(confirmation.duplicate_pending_mask(duplicate),
            static_cast<uint32_t>(MHI_COMMAND_FAN | MHI_COMMAND_TARGET_TEMP));

  duplicate.fan = 1U;
  EXPECT_EQ(confirmation.duplicate_pending_mask(duplicate), static_cast<uint32_t>(MHI_COMMAND_TARGET_TEMP));
}

void command_state_clears_pending_mask() {
  MhiCommandState command{};
  command.power_set = true;
  command.power = true;
  command.mode_set = true;
  command.mode = 2U;
  command.fan_set = true;
  command.fan = 6U;
  command.target_temp_set = true;
  command.target_temp_c = 22.0f;

  command.clear_pending_mask(MHI_COMMAND_FAN | MHI_COMMAND_TARGET_TEMP);

  EXPECT_TRUE(command.power_set);
  EXPECT_TRUE(command.mode_set);
  EXPECT_FALSE(command.fan_set);
  EXPECT_FALSE(command.target_temp_set);
  EXPECT_TRUE(command.has_pending_command());
  EXPECT_EQ(command.pending_command_mask(), static_cast<uint32_t>(MHI_COMMAND_POWER | MHI_COMMAND_MODE));
}

}  // namespace mhi_unit_tests
