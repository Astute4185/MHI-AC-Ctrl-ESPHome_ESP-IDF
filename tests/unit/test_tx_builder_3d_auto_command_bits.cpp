#include "mhi_test_common.h"

namespace mhi_unit_tests {
namespace {

void expect_3d_auto_command_bits(uint8_t preserved_db16, uint8_t preserved_db17,
                                 bool horizontal_swing, uint8_t horizontal_vane,
                                 bool requested_3d_auto, uint8_t expected_db17,
                                 uint8_t expected_intent_horizontal_vane) {
  MhiCommandState command{};
  command.three_d_auto_set = true;
  command.three_d_auto = requested_3d_auto;

  MhiTxRuntime runtime{};

  MhiTxBuildConfig config{};
  config.frame_size = kMhiFrame33Bytes;
  config.has_extended_louver_state = true;
  config.extended_louver_db16 = preserved_db16;
  config.extended_louver_db17 = preserved_db17;
  config.extended_louver_horizontal_swing = horizontal_swing;
  config.extended_louver_horizontal_vane = horizontal_vane;
  config.extended_louver_three_d_auto = (preserved_db17 & 0x04U) != 0U;

  MhiFrameBuffer out{};
  MhiTxBuildResult result{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));

  EXPECT_EQ(out.len, kMhiFrame33Bytes);
  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_THREE_D_AUTO));
  EXPECT_EQ(result.unsupported_command_mask, 0U);

  // The live louver position/state must be preserved while the command bits
  // and requested 3D Auto state are applied.
  EXPECT_EQ(out.data[DB16], preserved_db16);
  EXPECT_EQ(out.data[DB17], expected_db17);
  EXPECT_EQ(out.data[DB17] & 0x0AU, 0x0AU);  // 0x08 | 0x02 command indicators.
  EXPECT_EQ(out.data[DB17] & 0x01U, preserved_db17 & 0x01U);
  EXPECT_EQ(out.data[DB17] & 0x04U, requested_3d_auto ? 0x04U : 0x00U);

  EXPECT_EQ(result.intent.mask, static_cast<uint32_t>(MHI_COMMAND_THREE_D_AUTO));
  EXPECT_TRUE(result.intent.has_extended_louver_context);
  EXPECT_EQ(result.intent.horizontal_vane, expected_intent_horizontal_vane);
  EXPECT_EQ(result.intent.three_d_auto, requested_3d_auto);

  EXPECT_TRUE(mhi_checksum_valid_20(out.data));
  EXPECT_TRUE(mhi_checksum_valid_33(out.data));
  EXPECT_FALSE(command.has_pending_command());
}

void fixed_horizontal_position_3d_auto_on_uses_0x0e() {
  expect_3d_auto_command_bits(
      0x14U,  // Preserved fixed horizontal position.
      0x00U,  // Fixed position, 3D Auto currently off.
      false,
      4U,
      true,
      0x0EU,
      4U);
}

void fixed_horizontal_position_3d_auto_off_uses_0x0a() {
  expect_3d_auto_command_bits(
      0x14U,  // Preserved fixed horizontal position.
      0x04U,  // Fixed position, 3D Auto currently on.
      false,
      4U,
      false,
      0x0AU,
      4U);
}

void horizontal_swing_3d_auto_on_uses_0x0f() {
  expect_3d_auto_command_bits(
      0x1FU,  // Preserved horizontal swing context.
      0x01U,  // Horizontal swing, 3D Auto currently off.
      true,
      0U,
      true,
      0x0FU,
      8U);  // Intent code used by the builder for horizontal swing.
}

void horizontal_swing_3d_auto_off_uses_0x0b() {
  expect_3d_auto_command_bits(
      0x1FU,  // Preserved horizontal swing context.
      0x05U,  // Horizontal swing, 3D Auto currently on.
      true,
      0U,
      false,
      0x0BU,
      8U);  // Intent code used by the builder for horizontal swing.
}

}  // namespace

void tx_builder_3d_auto_command_bits_regression_suite() {
  fixed_horizontal_position_3d_auto_on_uses_0x0e();
  fixed_horizontal_position_3d_auto_off_uses_0x0a();
  horizontal_swing_3d_auto_on_uses_0x0f();
  horizontal_swing_3d_auto_off_uses_0x0b();
}

}  // namespace mhi_unit_tests
