#include "mhi_test_common.h"

namespace mhi_unit_tests {

void tx_builder_emits_valid_default_20_byte_frame() {
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  MhiTxBuildConfig config{};
  MhiFrameBuffer out{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out));

  EXPECT_EQ(out.len, kMhiFrame20Bytes);
  EXPECT_EQ(out.data[SB0], kMhiMisoSignature0);
  EXPECT_EQ(out.data[SB1], kMhiMisoSignature1);
  EXPECT_EQ(out.data[SB2], kMhiMisoSignature2);
  EXPECT_TRUE(mhi_checksum_valid_20(out.data));
}

void tx_builder_applies_pending_commands_once() {
  MhiCommandState command{};
  command.power_set = true;
  command.power = true;
  command.mode_set = true;
  command.mode = 2;
  command.target_temp_set = true;
  command.target_temp_c = 22.0f;
  command.fan_set = true;
  command.fan = 2;
  command.vertical_vane_set = true;
  command.vertical_vane = 4;

  MhiTxRuntime runtime{};
  MhiTxBuildConfig config{};
  MhiFrameBuffer out{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out));

  EXPECT_TRUE(mhi_checksum_valid_20(out.data));
  EXPECT_EQ(out.data[SB0], kMhiMisoSignature0);

  EXPECT_TRUE((out.data[DB0] & 0x02U) != 0U);  // power set bit.
  EXPECT_TRUE((out.data[DB0] & 0x01U) != 0U);  // power on value.
  EXPECT_TRUE((out.data[DB0] & 0x20U) != 0U);  // mode set bit.
  EXPECT_EQ(static_cast<uint8_t>((out.data[DB0] & 0x1CU) >> 2U), 2);

  EXPECT_EQ(out.data[DB2], static_cast<uint8_t>(0x80U | 44U));
  EXPECT_EQ(out.data[DB1] & 0x0FU, 0x0AU);
  EXPECT_TRUE((out.data[DB1] & 0x80U) != 0U);

  EXPECT_FALSE(command.has_pending_command());
}

void tx_builder_uses_configured_sensor_parity_opdata_mask() {
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  MhiTxBuildConfig config{};
  config.enabled_opdata_mask = MHI_OPDATA_REQ_IU_FANSPEED | MHI_OPDATA_REQ_TOTAL_IU_RUN |
                               MHI_OPDATA_REQ_OU_FANSPEED | MHI_OPDATA_REQ_TOTAL_COMP_RUN |
                               MHI_OPDATA_REQ_KWH;

  MhiFrameBuffer out{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out));

  EXPECT_TRUE(mhi_checksum_valid_20(out.data));
  EXPECT_EQ(out.data[DB6], 0xC0U);
  EXPECT_EQ(out.data[DB9], 0x1FU);
}

void tx_builder_uses_configured_sensor_parity_slice2_opdata_mask() {
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  MhiTxBuildConfig config{};
  config.enabled_opdata_mask = MHI_OPDATA_REQ_THI_R1 | MHI_OPDATA_REQ_THI_R2 | MHI_OPDATA_REQ_THI_R3 |
                               MHI_OPDATA_REQ_THO_R1 | MHI_OPDATA_REQ_TD | MHI_OPDATA_REQ_TDSH |
                               MHI_OPDATA_REQ_PROTECTION_NO | MHI_OPDATA_REQ_DEFROST |
                               MHI_OPDATA_REQ_OU_EEV1;

  MhiFrameBuffer out{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out));

  EXPECT_TRUE(mhi_checksum_valid_20(out.data));
  EXPECT_EQ(out.data[DB6], 0xC0U);
  EXPECT_EQ(out.data[DB9], 0x81U);
}


}  // namespace mhi_unit_tests
namespace mhi_unit_tests {

void tx_builder_reports_encoded_command_mask() {
  MhiCommandState command{};
  command.fan_set = true;
  command.fan = 2;
  command.vertical_vane_set = true;
  command.vertical_vane = 3;

  MhiTxRuntime runtime{};
  MhiTxBuildConfig config{};
  MhiFrameBuffer out{};
  MhiTxBuildResult result{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));

  EXPECT_TRUE(mhi_checksum_valid_20(out.data));
  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_FAN | MHI_COMMAND_VERTICAL_VANE));
  EXPECT_EQ(result.intent.mask, static_cast<uint32_t>(MHI_COMMAND_FAN | MHI_COMMAND_VERTICAL_VANE));
  EXPECT_EQ(result.intent.fan, 2U);
  EXPECT_EQ(result.intent.vertical_vane, 3U);
  EXPECT_EQ(result.unsupported_command_mask, 0U);
  EXPECT_FALSE(command.has_pending_command());
}

void tx_builder_keeps_double_frame_commands_pending_until_command_frame() {
  MhiCommandState command{};
  command.fan_set = true;
  command.fan = 6;

  MhiTxRuntime runtime{};
  runtime.double_frame = true;  // The next build toggles to the non-command half-frame.

  MhiTxBuildConfig config{};
  MhiFrameBuffer out{};
  MhiTxBuildResult result{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));
  EXPECT_EQ(result.encoded_command_mask, 0U);
  EXPECT_TRUE(command.has_pending_command());

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));
  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_FAN));
  EXPECT_FALSE(command.has_pending_command());
}

void tx_builder_drops_33_byte_only_commands_in_20_byte_mode() {
  MhiCommandState command{};
  command.horizontal_vane_set = true;
  command.horizontal_vane = 4;
  command.three_d_auto_set = true;
  command.three_d_auto = true;

  MhiTxRuntime runtime{};
  MhiTxBuildConfig config{};
  config.frame_size = kMhiFrame20Bytes;

  MhiFrameBuffer out{};
  MhiTxBuildResult result{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));

  EXPECT_TRUE(mhi_checksum_valid_20(out.data));
  EXPECT_EQ(result.encoded_command_mask, 0U);
  EXPECT_EQ(result.unsupported_command_mask,
            static_cast<uint32_t>(MHI_COMMAND_HORIZONTAL_VANE | MHI_COMMAND_THREE_D_AUTO));
  EXPECT_FALSE(command.has_pending_command());
}

void tx_builder_applies_3d_auto_in_33_byte_frame() {
  MhiCommandState command{};
  command.three_d_auto_set = true;
  command.three_d_auto = true;

  MhiTxRuntime runtime{};
  MhiTxBuildConfig config{};
  config.frame_size = kMhiFrame33Bytes;

  MhiFrameBuffer out{};
  MhiTxBuildResult result{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));

  EXPECT_EQ(out.len, kMhiFrame33Bytes);
  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_THREE_D_AUTO));
  EXPECT_EQ(result.unsupported_command_mask, 0U);
  EXPECT_EQ(out.data[DB17], static_cast<uint8_t>(0x0EU));
  EXPECT_EQ(result.intent.mask, static_cast<uint32_t>(MHI_COMMAND_THREE_D_AUTO));
  EXPECT_TRUE(result.intent.three_d_auto);
  EXPECT_FALSE(command.has_pending_command());
}

void tx_builder_reports_horizontal_vane_intent_in_33_byte_frame() {
  MhiCommandState command{};
  command.horizontal_vane_set = true;
  command.horizontal_vane = 4U;

  MhiTxRuntime runtime{};
  MhiTxBuildConfig config{};
  config.frame_size = kMhiFrame33Bytes;

  MhiFrameBuffer out{};
  MhiTxBuildResult result{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));

  EXPECT_EQ(out.len, kMhiFrame33Bytes);
  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_HORIZONTAL_VANE));
  EXPECT_EQ(result.intent.mask, static_cast<uint32_t>(MHI_COMMAND_HORIZONTAL_VANE));
  EXPECT_EQ(result.intent.horizontal_vane, 4U);
  EXPECT_EQ(out.data[DB16], static_cast<uint8_t>(0x13U));
  EXPECT_EQ(out.data[DB17], static_cast<uint8_t>(0x0AU));
  EXPECT_FALSE(command.has_pending_command());
}

void tx_builder_preserves_horizontal_context_for_3d_auto_command() {
  MhiCommandState command{};
  command.three_d_auto_set = true;
  command.three_d_auto = false;

  MhiTxRuntime runtime{};
  MhiTxBuildConfig config{};
  config.frame_size = kMhiFrame33Bytes;
  config.has_extended_louver_state = true;
  config.extended_louver_db16 = 0x1FU;
  config.extended_louver_db17 = 0x0FU;
  config.extended_louver_horizontal_swing = true;
  config.extended_louver_horizontal_vane = 0U;
  config.extended_louver_three_d_auto = true;

  MhiFrameBuffer out{};
  MhiTxBuildResult result{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));

  EXPECT_EQ(out.len, kMhiFrame33Bytes);
  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_THREE_D_AUTO));
  EXPECT_EQ(out.data[DB16], static_cast<uint8_t>(0x1FU));
  EXPECT_EQ(out.data[DB17], static_cast<uint8_t>(0x0BU));
  EXPECT_TRUE(result.intent.has_extended_louver_context);
  EXPECT_EQ(result.intent.horizontal_vane, 8U);
  EXPECT_FALSE(result.intent.three_d_auto);
  EXPECT_FALSE(command.has_pending_command());
}



void tx_builder_persists_external_room_temperature_override() {
  MhiCommandState command{};
  command.room_temp_override_set = true;
  command.room_temp_override_raw = 0x9DU;  // 24.0C: (24 * 4) + 61.

  MhiTxRuntime runtime{};
  MhiTxBuildConfig config{};
  MhiFrameBuffer out{};
  MhiTxBuildResult result{};

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));
  EXPECT_EQ(out.data[DB3], 0x9DU);
  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_ROOM_TEMP_OVERRIDE));
  EXPECT_FALSE(command.room_temp_override_set);

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));
  EXPECT_EQ(out.data[DB3], 0x9DU);
  EXPECT_EQ(result.encoded_command_mask, 0U);
}

void tx_builder_clears_external_room_temperature_override() {
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  runtime.room_temp_override_raw = 0x9DU;
  MhiTxBuildConfig config{};
  MhiFrameBuffer out{};
  MhiTxBuildResult result{};

  command.room_temp_override_set = true;
  command.room_temp_override_raw = 0xFFU;

  EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));
  EXPECT_EQ(out.data[DB3], 0xFFU);
  EXPECT_EQ(runtime.room_temp_override_raw, 0xFFU);
  EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_ROOM_TEMP_OVERRIDE));
}

}  // namespace mhi_unit_tests
