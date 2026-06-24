#include "mhi_test_common.h"

namespace mhi_unit_tests {

MhiFrameBuffer make_mosi_status_frame_33_with_valid_20_byte_prefix() {
  MhiFrameBuffer frame = make_mosi_status_frame();
  frame.len = kMhiFrame33Bytes;

  frame.data[DB15] = 0x01;
  frame.data[DB16] = 0x12;
  frame.data[DB17] = 0x0A;
  frame.data[DB18] = 0x34;
  frame.data[DB19] = 0x56;
  frame.data[DB20] = 0x78;
  frame.data[DB21] = 0x9A;
  frame.data[DB22] = 0xBC;
  frame.data[DB23] = 0xDE;
  frame.data[DB24] = 0xF0;
  frame.data[DB25] = 0x11;
  frame.data[DB26] = 0x22;

  const uint16_t checksum = mhi_calc_checksum_frame33(frame.data);
  frame.data[CBL2] = static_cast<uint8_t>(checksum & 0xFFU);

  return frame;
}

void frame_sync_discards_garbage_and_extracts_valid_frame() {
  const MhiFrameBuffer frame = make_mosi_status_frame();

  const uint8_t stream[] = {
      0x00, 0xFF, 0x12,
      frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4],
      frame.data[5], frame.data[6], frame.data[7], frame.data[8], frame.data[9],
      frame.data[10], frame.data[11], frame.data[12], frame.data[13], frame.data[14],
      frame.data[15], frame.data[16], frame.data[17], frame.data[18], frame.data[19],
  };

  MhiFrameSync sync{};
  EXPECT_TRUE(sync.push_bytes(stream, sizeof(stream)));
  EXPECT_TRUE(sync.frame_available());

  MhiFrameBuffer out{};
  EXPECT_TRUE(sync.pop_frame(out));

  EXPECT_EQ(out.len, kMhiFrame20Bytes);
  EXPECT_EQ(out.data[SB0], kMhiMosiSignature0Default);
  EXPECT_TRUE(mhi_checksum_valid_20(out.data));
}

void frame_sync_waits_for_partial_frame() {
  const MhiFrameBuffer frame = make_mosi_status_frame();

  MhiFrameSync sync{};
  EXPECT_FALSE(sync.push_bytes(frame.data, 10));
  EXPECT_FALSE(sync.frame_available());

  EXPECT_TRUE(sync.push_bytes(frame.data + 10, frame.len - 10));
  EXPECT_TRUE(sync.frame_available());
}

void frame_sync_records_resync_stats() {
  const MhiFrameBuffer frame = make_mosi_status_frame();

  const uint8_t stream[] = {
      0x00, 0xFF, 0x12,
      frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4],
      frame.data[5], frame.data[6], frame.data[7], frame.data[8], frame.data[9],
      frame.data[10], frame.data[11], frame.data[12], frame.data[13], frame.data[14],
      frame.data[15], frame.data[16], frame.data[17], frame.data[18], frame.data[19],
  };

  MhiStats stats{};
  MhiFrameSync sync{};
  sync.set_stats(&stats);
  sync.reset();

  EXPECT_TRUE(sync.push_bytes(stream, sizeof(stream)));

  MhiFrameBuffer out{};
  EXPECT_TRUE(sync.pop_frame(out));

  const auto snapshot = stats.snapshot();

  EXPECT_EQ(snapshot.frame_sync_resets, 1U);
  EXPECT_EQ(snapshot.signature_misses, 3U);
  EXPECT_EQ(snapshot.dropped_bytes, 3U);
  EXPECT_EQ(snapshot.candidate_frames, 1U);
  EXPECT_EQ(snapshot.invalid_frames, 0U);
  EXPECT_EQ(snapshot.checksum_failures, 0U);
}

void frame_sync_records_checksum_failure_stats() {
  MhiFrameBuffer frame = make_mosi_status_frame();
  frame.data[DB2] ^= 0x01U;

  MhiStats stats{};
  MhiFrameSync sync{};
  sync.set_stats(&stats);
  sync.reset();

  EXPECT_FALSE(sync.push_bytes(frame.data, frame.len));

  const auto snapshot = stats.snapshot();

  EXPECT_EQ(snapshot.candidate_frames, 1U);
  EXPECT_EQ(snapshot.invalid_frames, 1U);
  EXPECT_EQ(snapshot.checksum_failures, 1U);
  EXPECT_EQ(snapshot.sync_losses, 1U);
  EXPECT_TRUE(snapshot.dropped_bytes >= 1U);
}

void frame_sync_33_byte_mode_consumes_full_frame_without_tail_resync_noise() {
  const MhiFrameBuffer frame = make_mosi_status_frame_33_with_valid_20_byte_prefix();

  MhiStats stats{};
  MhiFrameSync sync{};
  sync.set_stats(&stats);
  sync.set_33_byte_frames_enabled(true);
  sync.reset();

  EXPECT_FALSE(sync.push_bytes(frame.data, kMhiFrame20Bytes));
  EXPECT_FALSE(sync.frame_available());

  auto snapshot = stats.snapshot();
  EXPECT_EQ(snapshot.candidate_frames, 0U);
  EXPECT_EQ(snapshot.signature_misses, 0U);
  EXPECT_EQ(snapshot.dropped_bytes, 0U);

  EXPECT_TRUE(sync.push_bytes(frame.data + kMhiFrame20Bytes, kMhiFrame33Bytes - kMhiFrame20Bytes));
  EXPECT_TRUE(sync.frame_available());

  MhiFrameBuffer out{};
  EXPECT_TRUE(sync.pop_frame(out));

  snapshot = stats.snapshot();
  EXPECT_EQ(out.len, kMhiFrame33Bytes);
  EXPECT_EQ(snapshot.candidate_frames, 1U);
  EXPECT_EQ(snapshot.invalid_frames, 0U);
  EXPECT_EQ(snapshot.checksum_failures, 0U);
  EXPECT_EQ(snapshot.signature_misses, 0U);
  EXPECT_EQ(snapshot.dropped_bytes, 0U);
}

}  // namespace mhi_unit_tests
