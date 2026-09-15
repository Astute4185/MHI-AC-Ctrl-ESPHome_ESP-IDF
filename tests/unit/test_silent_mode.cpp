#include "mhi_test_common.h"
#include "mhi_opdata_freshness.h"
#include "mhi_silent_mode_spike.h"

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
    MhiCommandState command{};
    MhiTxRuntime runtime{};
    runtime.double_frame = true;
    runtime.forced_opdata_mask = MHI_OPDATA_REQ_SILENT_MODE;
    MhiTxBuildConfig config{};
    MhiFrameBuffer out{};

    EXPECT_TRUE(MhiTxBuilder::build_next_frame(command, runtime, config, out));
    EXPECT_EQ(out.data[DB9], 0xFFU);
    EXPECT_EQ(runtime.forced_opdata_mask, static_cast<uint32_t>(MHI_OPDATA_REQ_SILENT_MODE));
  }

  {
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
    // The diagnostic spike deliberately removes Silent Mode from the generic
    // confirmation/retry lifecycle. One wire write is followed by explicit
    // 0xDD probes instead of retransmitting 0x21 every 1.5 seconds.
    MhiCommandConfirmation confirmation{};
    MhiCommandIntent intent{};
    intent.mask = MHI_COMMAND_SILENT_MODE;
    intent.silent_mode = true;
    confirmation.stage(intent, MHI_COMMAND_SILENT_MODE, 100U);
    EXPECT_FALSE(confirmation.has_pending());
    EXPECT_EQ(confirmation.pending_mask(), 0U);
  }

  {
    MhiSilentModeSpike spike{};
    spike.arm_request(true, true, false, 100U);
    EXPECT_TRUE(spike.awaiting_command_completion());
    spike.command_completed(true, 200U);
    EXPECT_TRUE(spike.active());
    EXPECT_FALSE(spike.poll_due(299U));
    EXPECT_TRUE(spike.poll_due(300U));
    EXPECT_EQ(spike.next_poll_offset_ms(), 100U);
    spike.mark_poll_staged(300U);
    EXPECT_EQ(spike.polls_staged(), 1U);
    EXPECT_FALSE(spike.poll_due(449U));
    EXPECT_TRUE(spike.poll_due(450U));
    EXPECT_EQ(static_cast<uint8_t>(spike.observe(false, 460U)),
              static_cast<uint8_t>(MhiSilentModeSpike::Observation::MISMATCH));
    EXPECT_TRUE(spike.active());
    EXPECT_EQ(static_cast<uint8_t>(spike.observe(true, 470U)),
              static_cast<uint8_t>(MhiSilentModeSpike::Observation::MATCH));
    EXPECT_FALSE(spike.active());
    EXPECT_TRUE(spike.matched());
  }

  {
    MhiSilentModeSpike spike{};
    spike.arm_request(false, true, true, 100U);
    spike.command_completed(false, 200U);
    for (const uint32_t offset : MhiSilentModeSpike::kPollOffsetsMs) {
      EXPECT_TRUE(spike.poll_due(200U + offset));
      spike.mark_poll_staged(200U + offset);
    }
    EXPECT_FALSE(spike.expire_if_due(12199U));
    EXPECT_TRUE(spike.expire_if_due(12200U));
    EXPECT_TRUE(spike.expired());
    EXPECT_FALSE(spike.active());
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
}

}  // namespace mhi_unit_tests
