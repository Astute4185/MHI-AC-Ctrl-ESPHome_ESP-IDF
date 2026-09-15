#include "mhi_test_common.h"
#include "mhi_opdata_freshness.h"

namespace mhi_unit_tests {

namespace {

MhiFrameBuffer make_silent_mode_response(bool enabled, uint8_t db6 = 0x00U) {
  return make_legacy_opdata_frame(db6, 0xDDU, 0x80U, enabled ? 0x20U : 0x00U, 0x00U);
}

}  // namespace

void silent_mode_regression_suite() {
  {
    const MhiFrameBuffer frame = make_silent_mode_response(true, 0x00U);
    EXPECT_TRUE(MhiOpDataDecoder::is_opdata_response(frame.view()));

    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.valid);
    EXPECT_TRUE(decoded.has_silent_mode);
    EXPECT_TRUE(decoded.silent_mode);
  }

  {
    // The response can be observed on either frame phase; DB6 is not a gate.
    const MhiFrameBuffer frame = make_silent_mode_response(false, 0x80U);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_silent_mode);
    EXPECT_FALSE(decoded.silent_mode);
  }

  {
    const MhiFrameBuffer near_match = make_legacy_opdata_frame(0x00U, 0xDDU, 0x80U, 0x20U, 0x01U);
    EXPECT_FALSE(MhiOpDataDecoder::is_opdata_response(near_match.view()));
  }

  {
    MhiCommandState command{};
    MhiTxRuntime runtime{};
    MhiTxBuildConfig config{};
    config.enabled_opdata_mask = MHI_OPDATA_REQ_SILENT_MODE;
    MhiFrameBuffer out{};

    EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out));
    EXPECT_EQ(out.data[DB6], 0xC0U);
    EXPECT_EQ(out.data[DB9], 0xDDU);
    EXPECT_TRUE(mhi_checksum_valid_20(out.data));
  }

  {
    MhiCommandState command{};
    command.silent_mode_set = true;
    command.silent_mode = true;
    command.power_set = true;
    command.power = true;

    MhiTxRuntime runtime{};
    MhiTxBuildConfig config{};
    config.enabled_opdata_mask = MHI_OPDATA_REQ_SILENT_MODE;
    MhiFrameBuffer out{};
    MhiTxBuildResult result{};

    EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));
    EXPECT_EQ(out.data[DB6], 0x80U);
    EXPECT_EQ(out.data[DB9], 0x21U);
    EXPECT_EQ(out.data[DB10], 0x01U);
    EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_SILENT_MODE));
    EXPECT_EQ(result.intent.mask, static_cast<uint32_t>(MHI_COMMAND_SILENT_MODE));
    EXPECT_TRUE(result.intent.silent_mode);
    EXPECT_FALSE(command.silent_mode_set);
    EXPECT_TRUE(command.power_set);
    EXPECT_TRUE(mhi_checksum_valid_20(out.data));
  }

  {
    MhiCommandState command{};
    command.silent_mode_set = true;
    command.silent_mode = false;
    MhiTxRuntime runtime{};
    MhiTxBuildConfig config{};
    MhiFrameBuffer out{};
    MhiTxBuildResult result{};

    EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));
    EXPECT_EQ(out.data[DB6], 0x80U);
    EXPECT_EQ(out.data[DB9], 0x21U);
    EXPECT_EQ(out.data[DB10], 0x00U);
    EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_SILENT_MODE));
  }

  {
    MhiCommandConfirmation confirmation{};
    MhiCommandIntent intent{};
    intent.mask = MHI_COMMAND_SILENT_MODE;
    intent.silent_mode = true;
    confirmation.stage(intent, MHI_COMMAND_SILENT_MODE, 100U);

    MhiOpDataState opdata{};
    opdata.valid = true;
    opdata.has_silent_mode = true;
    opdata.silent_mode = false;
    EXPECT_EQ(confirmation.observe_opdata(opdata), 0U);
    EXPECT_EQ(confirmation.pending_mask(), static_cast<uint32_t>(MHI_COMMAND_SILENT_MODE));

    opdata.silent_mode = true;
    EXPECT_EQ(confirmation.observe_opdata(opdata), static_cast<uint32_t>(MHI_COMMAND_SILENT_MODE));
    EXPECT_FALSE(confirmation.has_pending());
  }

  {
    MhiCommandConfirmation confirmation{};
    MhiCommandIntent intent{};
    intent.mask = MHI_COMMAND_SILENT_MODE;
    intent.silent_mode = true;
    confirmation.stage(intent, MHI_COMMAND_SILENT_MODE, 100U);

    MhiCommandState replacement{};
    replacement.silent_mode_set = true;
    replacement.silent_mode = false;
    EXPECT_EQ(confirmation.supersede(replacement), static_cast<uint32_t>(MHI_COMMAND_SILENT_MODE));
    EXPECT_FALSE(confirmation.has_pending());
  }

  {
    MhiOpDataFreshnessTracker freshness{};
    freshness.set_enabled_mask(MHI_OPDATA_REQ_SILENT_MODE);
    freshness.begin(100U);
    EXPECT_FALSE(freshness.evaluate(100U).fresh);
    freshness.observe(MHI_OPDATA_REQ_SILENT_MODE, 200U);
    const MhiOpDataFreshnessSnapshot snapshot = freshness.evaluate(200U);
    EXPECT_TRUE(snapshot.fresh);
    EXPECT_EQ(snapshot.pending_mask, 0U);
  }

  {
    MhiWorkerDecodedStore store{};
    const MhiFrameBuffer frame = make_silent_mode_response(true);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    store.merge_opdata(decoded, frame, 1U, 100U);

    MhiDecodedOpDataSnapshot snapshot{};
    EXPECT_TRUE(store.take_opdata(snapshot));
    EXPECT_TRUE(snapshot.decoded.has_silent_mode);
    EXPECT_TRUE(snapshot.decoded.silent_mode);
  }

  {
    MhiCommandState command{};
    MhiTxRuntime runtime{};
    runtime.forced_opdata_mask = MHI_OPDATA_REQ_SILENT_MODE;
    MhiTxBuildConfig config{};
    config.enabled_opdata_mask = MHI_OPDATA_REQ_MODE;
    MhiFrameBuffer out{};

    EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out));
    EXPECT_EQ(out.data[DB6], 0xC0U);
    EXPECT_EQ(out.data[DB9], 0xDDU);
    EXPECT_EQ(runtime.forced_opdata_mask, 0U);
  }

  {
    // A semantic command wins the frame, and the forced probe is retained for
    // the next eligible background frame instead of being silently consumed.
    MhiCommandState command{};
    command.power_set = true;
    command.power = true;
    MhiTxRuntime runtime{};
    runtime.forced_opdata_mask = MHI_OPDATA_REQ_SILENT_MODE;
    MhiTxBuildConfig config{};
    MhiFrameBuffer out{};
    MhiTxBuildResult result{};

    EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out, result));
    EXPECT_EQ(result.encoded_command_mask, static_cast<uint32_t>(MHI_COMMAND_POWER));
    EXPECT_EQ(out.data[DB9], 0xFFU);
    EXPECT_EQ(runtime.forced_opdata_mask, static_cast<uint32_t>(MHI_OPDATA_REQ_SILENT_MODE));
  }

  {
    // Silent Mode gets one 3-second confirmation window and never enters the
    // generic command retransmission loop.
    MhiCommandCoordinator coordinator{};
    MhiCommandState command{};
    command.silent_mode_set = true;
    command.silent_mode = true;
    MhiTxRuntime runtime{};
    MhiTxBuildConfig config{};
    MhiFrameBuffer frame{};
    MhiTxBuildResult result{};
    MhiTxEnvelope envelope{};

    const MhiCommandState before = command;
    const MhiTxRuntime runtime_before = runtime;
    EXPECT_TRUE(coordinator.prepare_next(command, runtime, config, frame, result, envelope));
    EXPECT_TRUE(envelope.is_command());
    coordinator.on_stage_result(envelope, before, runtime_before, command, true, 100U);

    MhiTxCompletion completion{};
    completion.kind = MhiTxKind::COMMAND;
    completion.generation = envelope.generation;
    completion.command_mask = envelope.command_mask;
    completion.success = true;
    completion.completed_at_ms = 200U;
    EXPECT_TRUE(coordinator.on_tx_completion(completion, command));
    EXPECT_EQ(coordinator.pending_mask(), static_cast<uint32_t>(MHI_COMMAND_SILENT_MODE));

    MhiCommandTimeoutResult timeout = coordinator.expire(3199U, command);
    EXPECT_FALSE(timeout.actionable());
    EXPECT_FALSE(command.silent_mode_set);

    timeout = coordinator.expire(3200U, command);
    EXPECT_EQ(timeout.timed_out_mask, static_cast<uint32_t>(MHI_COMMAND_SILENT_MODE));
    EXPECT_EQ(timeout.retry_mask, 0U);
    EXPECT_EQ(timeout.exhausted_mask, static_cast<uint32_t>(MHI_COMMAND_SILENT_MODE));
    EXPECT_FALSE(coordinator.has_pending_confirmation());
    EXPECT_FALSE(command.silent_mode_set);
  }

  {
    // A forced confirmation probe may transmit while Silent Mode confirmation
    // is pending, but a newer semantic command must remain queued.
    MhiCommandCoordinator coordinator{};
    MhiCommandState command{};
    command.silent_mode_set = true;
    command.silent_mode = true;
    MhiTxRuntime runtime{};
    MhiTxBuildConfig config{};
    MhiFrameBuffer frame{};
    MhiTxBuildResult result{};
    MhiTxEnvelope envelope{};

    const MhiCommandState before = command;
    const MhiTxRuntime runtime_before = runtime;
    EXPECT_TRUE(coordinator.prepare_next(command, runtime, config, frame, result, envelope));
    coordinator.on_stage_result(envelope, before, runtime_before, command, true, 100U);

    MhiTxCompletion completion{};
    completion.kind = MhiTxKind::COMMAND;
    completion.generation = envelope.generation;
    completion.command_mask = envelope.command_mask;
    completion.success = true;
    completion.completed_at_ms = 200U;
    EXPECT_TRUE(coordinator.on_tx_completion(completion, command));

    command.power_set = true;
    command.power = true;
    runtime.forced_opdata_mask = MHI_OPDATA_REQ_SILENT_MODE;

    MhiTxEnvelope probe_envelope{};
    MhiFrameBuffer probe_frame{};
    MhiTxBuildResult probe_result{};

    // First build is the single phase; the forced request remains pending.
    EXPECT_TRUE(coordinator.prepare_next(command, runtime, config, probe_frame, probe_result, probe_envelope));
    EXPECT_FALSE(probe_envelope.is_command());
    EXPECT_TRUE(command.power_set);
    EXPECT_EQ(runtime.forced_opdata_mask, static_cast<uint32_t>(MHI_OPDATA_REQ_SILENT_MODE));

    // Next build is the double phase and carries the requested 0xDD probe.
    EXPECT_TRUE(coordinator.prepare_next(command, runtime, config, probe_frame, probe_result, probe_envelope));
    EXPECT_FALSE(probe_envelope.is_command());
    EXPECT_EQ(probe_frame.data[DB6], 0xC0U);
    EXPECT_EQ(probe_frame.data[DB9], 0xDDU);
    EXPECT_EQ(runtime.forced_opdata_mask, 0U);
    EXPECT_TRUE(command.power_set);
  }
}

}  // namespace mhi_unit_tests
