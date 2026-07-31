#include "mhi_test_common.h"

namespace mhi_unit_tests {
namespace {

MhiTxBuildConfig make_extended_config(uint8_t db16, uint8_t db17, bool swing, uint8_t vane, bool three_d) {
  MhiTxBuildConfig config{};
  config.frame_size = kMhiFrame33Bytes;
  config.has_extended_louver_state = true;
  config.extended_louver_db16 = db16;
  config.extended_louver_db17 = db17;
  config.extended_louver_horizontal_swing = swing;
  config.extended_louver_horizontal_vane = vane;
  config.extended_louver_three_d_auto = three_d;
  return config;
}

MhiTxBuildResult build_extended_command(MhiCommandState& command, const MhiTxBuildConfig& config, MhiFrameBuffer& out) {
  MhiTxRuntime runtime{};
  MhiTxBuildResult result{};
  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));
  EXPECT_EQ(out.len, kMhiFrame33Bytes);
  EXPECT_TRUE(mhi_checksum_valid_20(out.data));
  EXPECT_TRUE(mhi_checksum_valid_33(out.data));
  return result;
}

void expect_3d_auto_command_bits(uint8_t preserved_db16, uint8_t preserved_db17, bool horizontal_swing,
                                 uint8_t horizontal_vane, bool requested_3d_auto, uint8_t expected_db17,
                                 uint8_t expected_intent_horizontal_vane) {
  MhiCommandState command{};
  command.three_d_auto_set = true;
  command.three_d_auto = requested_3d_auto;
  const auto config = make_extended_config(preserved_db16, preserved_db17, horizontal_swing, horizontal_vane,
                                           (preserved_db17 & 0x04U) != 0U);

  MhiFrameBuffer out{};
  const auto result = build_extended_command(command, config, out);

  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_THREE_D_AUTO));
  EXPECT_EQ(result.unsupported_command_mask, 0U);
  EXPECT_EQ(out.data[DB16], preserved_db16);
  EXPECT_EQ(out.data[DB17], expected_db17);
  EXPECT_EQ(out.data[DB17] & 0x0AU, 0x0AU);
  EXPECT_EQ(out.data[DB17] & 0x01U, preserved_db17 & 0x01U);
  EXPECT_EQ(out.data[DB17] & 0x04U, requested_3d_auto ? 0x04U : 0x00U);
  EXPECT_EQ(result.intent.mask, static_cast<uint32_t>(MHI_COMMAND_THREE_D_AUTO));
  EXPECT_TRUE(result.intent.has_extended_louver_context);
  EXPECT_EQ(result.intent.horizontal_vane, expected_intent_horizontal_vane);
  EXPECT_EQ(result.intent.three_d_auto, requested_3d_auto);
  EXPECT_FALSE(command.has_pending_command());
}

void fixed_horizontal_position_3d_auto_on_uses_0x0e() {
  expect_3d_auto_command_bits(0x14U, 0x00U, false, 5U, true, 0x0EU, 5U);
}

void fixed_horizontal_position_3d_auto_off_uses_0x0a() {
  expect_3d_auto_command_bits(0x14U, 0x04U, false, 5U, false, 0x0AU, 5U);
}

void horizontal_swing_3d_auto_on_uses_0x0f() {
  expect_3d_auto_command_bits(0x16U, 0x01U, true, 7U, true, 0x0FU, 8U);
}

void horizontal_swing_3d_auto_off_uses_0x0b() {
  expect_3d_auto_command_bits(0x16U, 0x05U, true, 7U, false, 0x0BU, 8U);
}

void three_d_without_preserved_state_keeps_legacy_db16_fallback() {
  MhiCommandState command{};
  command.three_d_auto_set = true;
  command.three_d_auto = true;
  MhiTxBuildConfig config{};
  config.frame_size = kMhiFrame33Bytes;

  MhiFrameBuffer out{};
  const auto result = build_extended_command(command, config, out);

  EXPECT_EQ(out.data[DB16], 0x00U);
  EXPECT_EQ(out.data[DB17], 0x0EU);
  EXPECT_EQ(result.intent.horizontal_vane, 1U);
  EXPECT_TRUE(result.intent.three_d_auto);
  EXPECT_FALSE(result.intent.has_extended_louver_context);
}

void horizontal_without_preserved_state_carries_transmitted_composite_context() {
  MhiCommandState command{};
  command.horizontal_vane_set = true;
  command.horizontal_vane = 4U;
  MhiTxBuildConfig config{};
  config.frame_size = kMhiFrame33Bytes;

  MhiFrameBuffer out{};
  const auto result = build_extended_command(command, config, out);

  EXPECT_EQ(out.data[DB16], 0x13U);
  EXPECT_EQ(out.data[DB17], 0x0AU);
  EXPECT_EQ(result.intent.horizontal_vane, 4U);
  EXPECT_FALSE(result.intent.three_d_auto);
  EXPECT_TRUE(result.intent.has_extended_louver_context);
}

void horizontal_fixed_write_preserves_3d_auto_on() {
  MhiCommandState command{};
  command.horizontal_vane_set = true;
  command.horizontal_vane = 4U;
  const auto config = make_extended_config(0x12U, 0x0EU, false, 3U, true);

  MhiFrameBuffer out{};
  const auto result = build_extended_command(command, config, out);

  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_HORIZONTAL_VANE));
  EXPECT_EQ(out.data[DB16], 0x13U);
  EXPECT_EQ(out.data[DB17], 0x0EU);
  EXPECT_TRUE(result.intent.has_extended_louver_context);
  EXPECT_EQ(result.intent.horizontal_vane, 4U);
  EXPECT_TRUE(result.intent.three_d_auto);
}

void horizontal_swing_write_preserves_3d_auto_on_and_last_position() {
  MhiCommandState command{};
  command.horizontal_vane_set = true;
  command.horizontal_vane = 8U;
  const auto config = make_extended_config(0x14U, 0x0EU, false, 5U, true);

  MhiFrameBuffer out{};
  const auto result = build_extended_command(command, config, out);

  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_HORIZONTAL_VANE));
  EXPECT_EQ(out.data[DB16], 0x14U);
  EXPECT_EQ(out.data[DB17], 0x0FU);
  EXPECT_EQ(result.intent.horizontal_vane, 8U);
  EXPECT_TRUE(result.intent.three_d_auto);
}

void combined_horizontal_and_3d_command_encodes_one_composite_state() {
  MhiCommandState command{};
  command.horizontal_vane_set = true;
  command.horizontal_vane = 6U;
  command.three_d_auto_set = true;
  command.three_d_auto = false;
  const auto config = make_extended_config(0x11U, 0x0EU, false, 2U, true);

  MhiFrameBuffer out{};
  const auto result = build_extended_command(command, config, out);

  EXPECT_EQ(result.encoded_command_mask,
            static_cast<uint32_t>(MHI_COMMAND_HORIZONTAL_VANE | MHI_COMMAND_THREE_D_AUTO));
  EXPECT_EQ(result.intent.mask, static_cast<uint32_t>(MHI_COMMAND_HORIZONTAL_VANE | MHI_COMMAND_THREE_D_AUTO));
  EXPECT_EQ(out.data[DB16], 0x15U);
  EXPECT_EQ(out.data[DB17], 0x0AU);
  EXPECT_EQ(result.intent.horizontal_vane, 6U);
  EXPECT_FALSE(result.intent.three_d_auto);
}

MhiStatusState make_extended_status(uint8_t horizontal_vane, bool swing, bool three_d) {
  MhiStatusState status{};
  status.valid = true;
  status.has_horizontal_vane = true;
  status.horizontal_vane = horizontal_vane;
  status.horizontal_vane_swing = swing;
  status.has_3d_auto = true;
  status.three_d_auto = three_d;
  return status;
}

void horizontal_confirmation_requires_preserved_3d_state() {
  MhiCommandConfirmation confirmation{};
  MhiCommandIntent intent{};
  intent.mask = MHI_COMMAND_HORIZONTAL_VANE;
  intent.horizontal_vane = 4U;
  intent.three_d_auto = true;
  intent.has_extended_louver_context = true;
  confirmation.stage(intent, MHI_COMMAND_HORIZONTAL_VANE, 100U);

  auto status = make_extended_status(4U, false, false);
  EXPECT_EQ(confirmation.observe_status(status), 0U);
  EXPECT_TRUE(confirmation.has_pending());

  status.three_d_auto = true;
  EXPECT_EQ(confirmation.observe_status(status), static_cast<uint32_t>(MHI_COMMAND_HORIZONTAL_VANE));
  EXPECT_FALSE(confirmation.has_pending());
}

void three_d_confirmation_requires_preserved_horizontal_state() {
  MhiCommandConfirmation confirmation{};
  MhiCommandIntent intent{};
  intent.mask = MHI_COMMAND_THREE_D_AUTO;
  intent.horizontal_vane = 4U;
  intent.three_d_auto = true;
  intent.has_extended_louver_context = true;
  confirmation.stage(intent, MHI_COMMAND_THREE_D_AUTO, 100U);

  auto status = make_extended_status(5U, false, true);
  EXPECT_EQ(confirmation.observe_status(status), 0U);
  EXPECT_TRUE(confirmation.has_pending());

  status.horizontal_vane = 4U;
  EXPECT_EQ(confirmation.observe_status(status), static_cast<uint32_t>(MHI_COMMAND_THREE_D_AUTO));
  EXPECT_FALSE(confirmation.has_pending());
}

void swing_confirmation_ignores_retained_position() {
  MhiCommandConfirmation confirmation{};
  MhiCommandIntent intent{};
  intent.mask = MHI_COMMAND_HORIZONTAL_VANE;
  intent.horizontal_vane = 8U;
  intent.three_d_auto = true;
  intent.has_extended_louver_context = true;
  confirmation.stage(intent, MHI_COMMAND_HORIZONTAL_VANE, 100U);

  const auto status = make_extended_status(7U, true, true);
  EXPECT_EQ(confirmation.observe_status(status), static_cast<uint32_t>(MHI_COMMAND_HORIZONTAL_VANE));
}

void three_d_timeout_retries_inside_matrix_case() {
  MhiCommandConfirmation confirmation{};
  MhiCommandIntent intent{};
  intent.mask = MHI_COMMAND_THREE_D_AUTO;
  intent.horizontal_vane = 3U;
  intent.three_d_auto = true;
  intent.has_extended_louver_context = true;
  confirmation.stage(intent, MHI_COMMAND_THREE_D_AUTO, 100U);

  EXPECT_FALSE(confirmation.expire(100U + kMhiThreeDAutoConfirmationTimeoutMs - 1U).expired());
  const auto expiration = confirmation.expire(100U + kMhiThreeDAutoConfirmationTimeoutMs);
  EXPECT_TRUE(expiration.expired());
  EXPECT_EQ(expiration.mask, static_cast<uint32_t>(MHI_COMMAND_THREE_D_AUTO));
}

}  // namespace

void tx_builder_3d_auto_command_bits_regression_suite() {
  fixed_horizontal_position_3d_auto_on_uses_0x0e();
  fixed_horizontal_position_3d_auto_off_uses_0x0a();
  horizontal_swing_3d_auto_on_uses_0x0f();
  horizontal_swing_3d_auto_off_uses_0x0b();
  three_d_without_preserved_state_keeps_legacy_db16_fallback();
  horizontal_without_preserved_state_carries_transmitted_composite_context();
  horizontal_fixed_write_preserves_3d_auto_on();
  horizontal_swing_write_preserves_3d_auto_on_and_last_position();
  combined_horizontal_and_3d_command_encodes_one_composite_state();
  horizontal_confirmation_requires_preserved_3d_state();
  three_d_confirmation_requires_preserved_horizontal_state();
  swing_confirmation_ignores_retained_position();
  three_d_timeout_retries_inside_matrix_case();
}

}  // namespace mhi_unit_tests
