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
}

}  // namespace mhi_unit_tests
