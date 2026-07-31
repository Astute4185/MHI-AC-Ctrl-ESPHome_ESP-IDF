#include "mhi_test_common.h"

namespace mhi_unit_tests {
namespace {

struct PreparedCommand {
  MhiCommandState before{};
  MhiFrameBuffer frame{};
  MhiTxBuildResult result{};
  MhiTxEnvelope envelope{};
};

MhiTxBuildConfig make_extended_config(uint8_t horizontal_vane, bool horizontal_swing, bool three_d_auto) {
  MhiTxBuildConfig config{};
  config.frame_size = kMhiFrame33Bytes;
  config.has_extended_louver_state = true;
  config.extended_louver_horizontal_vane = horizontal_vane;
  config.extended_louver_horizontal_swing = horizontal_swing;
  config.extended_louver_three_d_auto = three_d_auto;
  config.extended_louver_db16 = horizontal_vane >= 1U && horizontal_vane <= 7U
                                      ? static_cast<uint8_t>(0x0FU + horizontal_vane)
                                      : 0x16U;
  config.extended_louver_db17 = static_cast<uint8_t>(0x0AU | (horizontal_swing ? 0x01U : 0U) |
                                                      (three_d_auto ? 0x04U : 0U));
  return config;
}

PreparedCommand prepare_command(MhiCommandCoordinator& coordinator, MhiCommandState& command, MhiTxRuntime& runtime,
                                const MhiTxBuildConfig& config) {
  PreparedCommand prepared{};
  prepared.before = command;
  const bool built = coordinator.prepare_next(command, runtime, config, prepared.frame, prepared.result,
                                               prepared.envelope);
  EXPECT_TRUE(built);
  EXPECT_TRUE(prepared.envelope.valid());
  return prepared;
}

void stage_command(MhiCommandCoordinator& coordinator, const PreparedCommand& prepared, MhiCommandState& command,
                   uint32_t staged_at_ms) {
  coordinator.on_stage_result(prepared.envelope, prepared.before, command, true, staged_at_ms);
  EXPECT_TRUE(coordinator.has_command_in_flight());
}

void complete_command(MhiCommandCoordinator& coordinator, const PreparedCommand& prepared, MhiCommandState& command,
                      uint32_t completed_at_ms) {
  MhiTxCompletion completion{};
  completion.kind = MhiTxKind::COMMAND;
  completion.generation = prepared.envelope.generation;
  completion.command_mask = prepared.envelope.command_mask;
  completion.success = true;
  completion.completed_at_ms = completed_at_ms;
  const bool handled = coordinator.on_tx_completion(completion, command);
  EXPECT_TRUE(handled);
  EXPECT_TRUE(!coordinator.has_command_in_flight());
}

void pending_horizontal_swing_plus_3d_off_coalesces_immediately() {
  MhiCommandCoordinator coordinator{};
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  const auto config = make_extended_config(7U, false, true);

  command.horizontal_vane_set = true;
  command.horizontal_vane = 8U;
  const auto first = prepare_command(coordinator, command, runtime, config);
  EXPECT_TRUE(first.result.encoded_command_mask == MHI_COMMAND_HORIZONTAL_VANE);
  EXPECT_TRUE(first.frame.data[DB16] == 0x16U);
  EXPECT_TRUE(first.frame.data[DB17] == 0x0FU);
  stage_command(coordinator, first, command, 100U);
  complete_command(coordinator, first, command, 150U);
  EXPECT_TRUE(coordinator.pending_mask() == MHI_COMMAND_HORIZONTAL_VANE);

  MhiCommandState patch{};
  patch.three_d_auto_set = true;
  patch.three_d_auto = false;
  const uint32_t superseded = coordinator.supersede_pending(patch);
  EXPECT_TRUE(superseded == MHI_COMMAND_HORIZONTAL_VANE);
  EXPECT_TRUE(!coordinator.has_pending_confirmation());

  command.three_d_auto_set = true;
  command.three_d_auto = false;
  const auto combined = prepare_command(coordinator, command, runtime, config);
  EXPECT_TRUE(combined.result.encoded_command_mask == kMhiExtendedLouverCommandMask);
  EXPECT_TRUE(combined.frame.data[DB16] == 0x16U);
  EXPECT_TRUE(combined.frame.data[DB17] == 0x0BU);
}

void pending_fixed_horizontal_plus_3d_off_coalesces_immediately() {
  MhiCommandCoordinator coordinator{};
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  const auto config = make_extended_config(4U, false, true);

  command.horizontal_vane_set = true;
  command.horizontal_vane = 5U;
  const auto first = prepare_command(coordinator, command, runtime, config);
  EXPECT_TRUE(first.frame.data[DB16] == 0x14U);
  EXPECT_TRUE(first.frame.data[DB17] == 0x0EU);
  stage_command(coordinator, first, command, 100U);
  complete_command(coordinator, first, command, 150U);

  MhiCommandState patch{};
  patch.three_d_auto_set = true;
  patch.three_d_auto = false;
  EXPECT_TRUE(coordinator.supersede_pending(patch) == MHI_COMMAND_HORIZONTAL_VANE);
  command.three_d_auto_set = true;
  command.three_d_auto = false;

  const auto combined = prepare_command(coordinator, command, runtime, config);
  EXPECT_TRUE(combined.result.encoded_command_mask == kMhiExtendedLouverCommandMask);
  EXPECT_TRUE(combined.frame.data[DB16] == 0x14U);
  EXPECT_TRUE(combined.frame.data[DB17] == 0x0AU);
}

void pending_3d_plus_new_horizontal_preserves_3d_target() {
  MhiCommandCoordinator coordinator{};
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  const auto config = make_extended_config(3U, false, false);

  command.three_d_auto_set = true;
  command.three_d_auto = true;
  const auto first = prepare_command(coordinator, command, runtime, config);
  EXPECT_TRUE(first.result.encoded_command_mask == MHI_COMMAND_THREE_D_AUTO);
  EXPECT_TRUE(first.frame.data[DB16] == 0x12U);
  EXPECT_TRUE(first.frame.data[DB17] == 0x0EU);
  stage_command(coordinator, first, command, 100U);
  complete_command(coordinator, first, command, 150U);
  EXPECT_TRUE(coordinator.pending_mask() == MHI_COMMAND_THREE_D_AUTO);

  MhiCommandState patch{};
  patch.horizontal_vane_set = true;
  patch.horizontal_vane = 5U;
  EXPECT_TRUE(coordinator.supersede_pending(patch) == MHI_COMMAND_THREE_D_AUTO);
  command.horizontal_vane_set = true;
  command.horizontal_vane = 5U;

  const auto combined = prepare_command(coordinator, command, runtime, config);
  EXPECT_TRUE(combined.result.encoded_command_mask == kMhiExtendedLouverCommandMask);
  EXPECT_TRUE(combined.frame.data[DB16] == 0x14U);
  EXPECT_TRUE(combined.frame.data[DB17] == 0x0EU);
}

void in_flight_cross_field_request_supersedes_before_confirmation() {
  MhiCommandCoordinator coordinator{};
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  const auto config = make_extended_config(7U, false, true);

  command.horizontal_vane_set = true;
  command.horizontal_vane = 8U;
  const auto first = prepare_command(coordinator, command, runtime, config);
  stage_command(coordinator, first, command, 100U);

  command.three_d_auto_set = true;
  command.three_d_auto = false;
  complete_command(coordinator, first, command, 150U);
  EXPECT_TRUE(!coordinator.has_pending_confirmation());

  const auto combined = prepare_command(coordinator, command, runtime, config);
  EXPECT_TRUE(combined.result.encoded_command_mask == kMhiExtendedLouverCommandMask);
  EXPECT_TRUE(combined.frame.data[DB16] == 0x16U);
  EXPECT_TRUE(combined.frame.data[DB17] == 0x0BU);
}

void failed_queue_restores_both_coalesced_fields() {
  MhiCommandCoordinator coordinator{};
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  const auto config = make_extended_config(7U, false, true);

  command.horizontal_vane_set = true;
  command.horizontal_vane = 8U;
  const auto first = prepare_command(coordinator, command, runtime, config);
  stage_command(coordinator, first, command, 100U);
  complete_command(coordinator, first, command, 150U);

  MhiCommandState patch{};
  patch.three_d_auto_set = true;
  patch.three_d_auto = false;
  EXPECT_TRUE(coordinator.supersede_pending(patch) == MHI_COMMAND_HORIZONTAL_VANE);
  command.three_d_auto_set = true;
  command.three_d_auto = false;

  const auto combined = prepare_command(coordinator, command, runtime, config);
  EXPECT_TRUE(!command.horizontal_vane_set);
  EXPECT_TRUE(!command.three_d_auto_set);
  coordinator.on_stage_result(combined.envelope, combined.before, command, false, 200U);
  EXPECT_TRUE(command.horizontal_vane_set);
  EXPECT_TRUE(command.horizontal_vane == 8U);
  EXPECT_TRUE(command.three_d_auto_set);
  EXPECT_TRUE(!command.three_d_auto);
}

void same_composite_companion_does_not_supersede() {
  MhiCommandCoordinator coordinator{};
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  const auto config = make_extended_config(7U, false, true);

  command.horizontal_vane_set = true;
  command.horizontal_vane = 8U;
  const auto first = prepare_command(coordinator, command, runtime, config);
  stage_command(coordinator, first, command, 100U);
  complete_command(coordinator, first, command, 150U);

  MhiCommandState patch{};
  patch.three_d_auto_set = true;
  patch.three_d_auto = true;
  EXPECT_TRUE(coordinator.supersede_pending(patch) == 0U);
  EXPECT_TRUE(coordinator.pending_mask() == MHI_COMMAND_HORIZONTAL_VANE);
}

void same_field_3d_replacement_remains_single_field() {
  MhiCommandCoordinator coordinator{};
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  const auto config = make_extended_config(3U, false, false);

  command.three_d_auto_set = true;
  command.three_d_auto = true;
  const auto first = prepare_command(coordinator, command, runtime, config);
  stage_command(coordinator, first, command, 100U);
  complete_command(coordinator, first, command, 150U);
  EXPECT_TRUE(coordinator.pending_mask() == MHI_COMMAND_THREE_D_AUTO);

  MhiCommandState patch{};
  patch.three_d_auto_set = true;
  patch.three_d_auto = false;
  EXPECT_TRUE(coordinator.supersede_pending(patch) == MHI_COMMAND_THREE_D_AUTO);
  command.three_d_auto_set = true;
  command.three_d_auto = false;

  const auto replacement = prepare_command(coordinator, command, runtime, config);
  EXPECT_TRUE(replacement.result.encoded_command_mask == MHI_COMMAND_THREE_D_AUTO);
  EXPECT_TRUE(replacement.frame.data[DB16] == 0x12U);
  EXPECT_TRUE(replacement.frame.data[DB17] == 0x0AU);
}

void newest_explicit_values_override_cached_companion() {
  MhiCommandCoordinator coordinator{};
  MhiCommandState command{};
  MhiTxRuntime runtime{};
  const auto config = make_extended_config(7U, false, true);

  command.horizontal_vane_set = true;
  command.horizontal_vane = 8U;
  const auto first = prepare_command(coordinator, command, runtime, config);
  stage_command(coordinator, first, command, 100U);
  complete_command(coordinator, first, command, 150U);

  MhiCommandState patch{};
  patch.three_d_auto_set = true;
  patch.three_d_auto = false;
  EXPECT_TRUE(coordinator.supersede_pending(patch) == MHI_COMMAND_HORIZONTAL_VANE);

  // Simulate two newer requests arriving before the worker builds again.
  command.horizontal_vane_set = true;
  command.horizontal_vane = 5U;
  command.three_d_auto_set = true;
  command.three_d_auto = true;

  const auto combined = prepare_command(coordinator, command, runtime, config);
  EXPECT_TRUE(combined.result.encoded_command_mask == kMhiExtendedLouverCommandMask);
  EXPECT_TRUE(combined.frame.data[DB16] == 0x14U);
  EXPECT_TRUE(combined.frame.data[DB17] == 0x0EU);
}

}  // namespace

void command_coordinator_extended_louver_supersession_suite() {
  pending_horizontal_swing_plus_3d_off_coalesces_immediately();
  pending_fixed_horizontal_plus_3d_off_coalesces_immediately();
  pending_3d_plus_new_horizontal_preserves_3d_target();
  in_flight_cross_field_request_supersedes_before_confirmation();
  failed_queue_restores_both_coalesced_fields();
  same_composite_companion_does_not_supersede();
  same_field_3d_replacement_remains_single_field();
  newest_explicit_values_override_cached_companion();
}

}  // namespace mhi_unit_tests
