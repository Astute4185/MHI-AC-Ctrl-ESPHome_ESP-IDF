#include "mhi_test_common.h"

namespace mhi_unit_tests {
namespace {

MhiFrameBuffer make_self_clean_status(uint8_t db7, uint8_t db13) {
  MhiFrameBuffer frame = make_mosi_status_frame_33(1U, false, false);
  frame.data[DB7] = db7;
  frame.data[DB13] = db13;

  const uint16_t checksum = mhi_calc_checksum_frame33(frame.data);
  frame.data[CBL2] = static_cast<uint8_t>(checksum & 0xFFU);
  return frame;
}

}  // namespace

void self_clean_regression_suite() {
  {
    const MhiFrameBuffer frame = make_self_clean_status(0x00U, 0x00U);
    MhiDecodedStatus decoded{};

    EXPECT_TRUE(MhiStatusDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_self_clean_candidates);
    EXPECT_FALSE(decoded.self_clean_db7_candidate);
    EXPECT_FALSE(decoded.self_clean_db13_candidate);
    EXPECT_TRUE(decoded.has_self_clean);
    EXPECT_FALSE(decoded.self_clean);
  }

  {
    // DB13 bit 0 is unrelated to Self Clean. 0x05 has both bit 2 (candidate)
    // and bit 0 set, matching the observed running-state capture.
    const MhiFrameBuffer frame = make_self_clean_status(0x02U, 0x05U);
    MhiDecodedStatus decoded{};

    EXPECT_TRUE(MhiStatusDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_self_clean_candidates);
    EXPECT_TRUE(decoded.self_clean_db7_candidate);
    EXPECT_TRUE(decoded.self_clean_db13_candidate);
    EXPECT_TRUE(decoded.has_self_clean);
    EXPECT_TRUE(decoded.self_clean);
  }

  {
    // Observed transition while the AC turns off: DB13 0x05 -> 0x04.
    // Self Clean must remain active because candidate bit 2 remains set.
    MhiDecodedStatus before{};
    const MhiFrameBuffer before_frame = make_self_clean_status(0x02U, 0x05U);
    EXPECT_TRUE(MhiStatusDecoder::decode_mosi(before_frame.view(), before));
    EXPECT_TRUE(before.has_self_clean);
    EXPECT_TRUE(before.self_clean);

    MhiDecodedStatus after{};
    const MhiFrameBuffer after_frame = make_self_clean_status(0x02U, 0x04U);
    EXPECT_TRUE(MhiStatusDecoder::decode_mosi(after_frame.view(), after));
    EXPECT_TRUE(after.self_clean_db7_candidate);
    EXPECT_TRUE(after.self_clean_db13_candidate);
    EXPECT_TRUE(after.has_self_clean);
    EXPECT_TRUE(after.self_clean);
  }

  {
    // The older trace in MHI-AC-Trace issue #2 shows DB0 changing from the
    // normal Cool mode value to Fan while Self Clean is running. DB0 is the
    // existing HVAC mode field and must not be treated as a Self Clean flag.
    MhiFrameBuffer frame = make_mosi_status_frame();
    frame.data[DB0] = 0x4DU;

    const uint16_t checksum = mhi_calc_checksum(frame.data);
    frame.data[CBH] = static_cast<uint8_t>((checksum >> 8U) & 0xFFU);
    frame.data[CBL] = static_cast<uint8_t>(checksum & 0xFFU);

    MhiDecodedStatus decoded{};
    EXPECT_TRUE(MhiStatusDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_EQ(decoded.mode, 3U);
    EXPECT_FALSE(decoded.has_self_clean_candidates);
    EXPECT_FALSE(decoded.has_self_clean);
  }

  {
    MhiStateStore state{};
    auto& status = state.status();
    status.valid = true;
    status.has_self_clean = true;
    status.self_clean = true;

    esphome::switch_::Switch self_clean{};
    MhiPublishTargets targets{};
    targets.self_clean_switch = &self_clean;

    MhiPublishBridge bridge{};
    bridge.set_targets(targets);
    bridge.publish(state);

    EXPECT_EQ(self_clean.publish_count, 1U);
    EXPECT_TRUE(self_clean.state);

    status.self_clean = false;
    bridge.publish(state);

    EXPECT_EQ(self_clean.publish_count, 2U);
    EXPECT_FALSE(self_clean.state);
  }
}

}  // namespace mhi_unit_tests
