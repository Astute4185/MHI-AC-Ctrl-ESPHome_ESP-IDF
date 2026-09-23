#include "mhi_test_common.h"

#include "mhi_protocol_capture_core.h"

namespace mhi_unit_tests {
namespace {

MhiFrameBuffer make_capture_frame(std::size_t len, uint8_t fill = 0U) {
  MhiFrameBuffer frame{};
  frame.len = len;
  for (std::size_t i = 0; i < len; i++) {
    frame.data[i] = fill;
  }
  return frame;
}

}  // namespace

void protocol_capture_first_frame_is_baseline() {
  MhiProtocolCaptureEngine capture{};
  const auto result = capture.observe(make_capture_frame(kMhiFrame20Bytes), 1U, 0U, 1U);
  EXPECT_TRUE(result.disposition == MhiCaptureDisposition::BASELINE);
  EXPECT_EQ(result.frame_len, kMhiFrame20Bytes);
  EXPECT_EQ(result.total_changes, 0U);
}

void protocol_capture_unchanged_frame_is_suppressed() {
  MhiProtocolCaptureEngine capture{};
  const auto frame = make_capture_frame(kMhiFrame20Bytes, 0x11U);
  capture.observe(frame, 1U, 0U, 1U);
  const auto result = capture.observe(frame, 1U, 0U, 2U);
  EXPECT_TRUE(result.disposition == MhiCaptureDisposition::UNCHANGED);
  EXPECT_EQ(result.total_changes, 0U);
}

void protocol_capture_reports_payload_change_and_xor() {
  MhiProtocolCaptureEngine capture{};
  auto before = make_capture_frame(kMhiFrame20Bytes);
  auto after = before;
  before.data[6] = 0x20U;
  after.data[6] = 0x28U;
  capture.observe(before, 1U, 0U, 1U);

  const auto result = capture.observe(after, 1U, 0U, 2U);
  EXPECT_TRUE(result.disposition == MhiCaptureDisposition::CHANGED);
  EXPECT_EQ(result.total_changes, 1U);
  EXPECT_EQ(result.reported_changes, 1U);
  EXPECT_EQ(result.changes[0].raw_index, 6U);
  EXPECT_EQ(result.changes[0].before, 0x20U);
  EXPECT_EQ(result.changes[0].after, 0x28U);
  EXPECT_EQ(result.changes[0].xor_mask, 0x08U);
}

void protocol_capture_tracks_reverse_transition_from_latest_state() {
  MhiProtocolCaptureEngine capture{};
  auto off = make_capture_frame(kMhiFrame20Bytes);
  auto on = off;
  on.data[8] = 0x40U;

  capture.observe(off, 1U, 0U, 1U);
  const auto enabled = capture.observe(on, 1U, 0U, 2U);
  EXPECT_TRUE(enabled.disposition == MhiCaptureDisposition::CHANGED);
  EXPECT_EQ(enabled.changes[0].before, 0x00U);
  EXPECT_EQ(enabled.changes[0].after, 0x40U);

  const auto stable = capture.observe(on, 1U, 0U, 3U);
  EXPECT_TRUE(stable.disposition == MhiCaptureDisposition::UNCHANGED);

  const auto disabled = capture.observe(off, 1U, 0U, 4U);
  EXPECT_TRUE(disabled.disposition == MhiCaptureDisposition::CHANGED);
  EXPECT_EQ(disabled.changes[0].before, 0x40U);
  EXPECT_EQ(disabled.changes[0].after, 0x00U);
  EXPECT_EQ(disabled.changes[0].xor_mask, 0x40U);
}

void protocol_capture_ignores_checksum_only_change() {
  MhiProtocolCaptureEngine capture{};
  auto before = make_capture_frame(kMhiFrame20Bytes);
  auto after = before;
  after.data[kMhiFrame20Bytes - 2U] = 0x12U;
  after.data[kMhiFrame20Bytes - 1U] = 0x34U;
  capture.observe(before, 1U, 0U, 1U);

  const auto result = capture.observe(after, 1U, 0U, 2U);
  EXPECT_TRUE(result.disposition == MhiCaptureDisposition::UNCHANGED);
  EXPECT_EQ(result.total_changes, 0U);
}

void protocol_capture_keeps_opdata_keys_separate() {
  MhiProtocolCaptureEngine capture{};
  auto frame_a = make_capture_frame(kMhiFrame20Bytes);
  auto frame_b = frame_a;
  frame_b.data[10] = 0x44U;

  EXPECT_TRUE(capture.observe(frame_a, 2U, 0x8010U, 1U).disposition == MhiCaptureDisposition::BASELINE);
  EXPECT_TRUE(capture.observe(frame_b, 2U, 0x8011U, 2U).disposition == MhiCaptureDisposition::BASELINE);
  EXPECT_TRUE(capture.observe(frame_a, 2U, 0x8010U, 3U).disposition == MhiCaptureDisposition::UNCHANGED);
}

void protocol_capture_keeps_frame_kinds_separate() {
  MhiProtocolCaptureEngine capture{};
  auto frame_a = make_capture_frame(kMhiFrame20Bytes);
  auto frame_b = frame_a;
  frame_b.data[7] = 0x80U;

  EXPECT_TRUE(capture.observe(frame_a, 1U, 0U, 1U).disposition == MhiCaptureDisposition::BASELINE);
  EXPECT_TRUE(capture.observe(frame_b, 2U, 0U, 2U).disposition == MhiCaptureDisposition::BASELINE);
  EXPECT_TRUE(capture.observe(frame_a, 1U, 0U, 3U).disposition == MhiCaptureDisposition::UNCHANGED);
}

void protocol_capture_length_change_rebaselines_identity() {
  MhiProtocolCaptureEngine capture{};
  const auto frame20 = make_capture_frame(kMhiFrame20Bytes);
  const auto frame33 = make_capture_frame(kMhiFrame33Bytes);

  EXPECT_TRUE(capture.observe(frame20, 1U, 0U, 1U).disposition == MhiCaptureDisposition::BASELINE);
  EXPECT_TRUE(capture.observe(frame33, 1U, 0U, 2U).disposition == MhiCaptureDisposition::BASELINE);
  EXPECT_TRUE(capture.observe(frame33, 1U, 0U, 3U).disposition == MhiCaptureDisposition::UNCHANGED);
}

void protocol_capture_rejects_invalid_frame_length() {
  MhiProtocolCaptureEngine capture{};
  const auto invalid = make_capture_frame(21U);
  const auto result = capture.observe(invalid, 1U, 0U, 1U);
  EXPECT_TRUE(result.disposition == MhiCaptureDisposition::IGNORED);
}

void protocol_capture_33_byte_tracks_db26_and_ignores_checksums() {
  MhiProtocolCaptureEngine capture{};
  auto before = make_capture_frame(kMhiFrame33Bytes);
  auto after = before;
  after.data[CBH] = 0x12U;
  after.data[CBL] = 0x34U;
  after.data[DB26] = 0x80U;
  after.data[CBL2] = 0x56U;
  capture.observe(before, 1U, 0U, 1U);

  const auto result = capture.observe(after, 1U, 0U, 2U);
  EXPECT_TRUE(result.disposition == MhiCaptureDisposition::CHANGED);
  EXPECT_EQ(result.total_changes, 1U);
  EXPECT_EQ(result.reported_changes, 1U);
  EXPECT_EQ(result.changes[0].raw_index, DB26);
  EXPECT_EQ(result.changes[0].before, 0x00U);
  EXPECT_EQ(result.changes[0].after, 0x80U);
  EXPECT_EQ(result.changes[0].xor_mask, 0x80U);
}

void protocol_capture_caps_reported_changes_but_counts_all() {
  MhiProtocolCaptureEngine capture{};
  auto before = make_capture_frame(kMhiFrame33Bytes);
  auto after = before;
  for (std::size_t i = 0; i <= DB14; i++) {
    after.data[i] = static_cast<uint8_t>(i + 1U);
  }
  after.data[DB15] = 0x80U;
  after.data[DB16] = 0x81U;
  capture.observe(before, 1U, 0U, 1U);

  const auto result = capture.observe(after, 1U, 0U, 2U);
  EXPECT_TRUE(result.disposition == MhiCaptureDisposition::CHANGED);
  EXPECT_EQ(result.total_changes, 20U);
  EXPECT_EQ(result.reported_changes, kMhiCaptureMaxReportedChanges);
  EXPECT_TRUE(result.truncated());
}

void protocol_capture_reset_forces_new_baseline() {
  MhiProtocolCaptureEngine capture{};
  const auto frame = make_capture_frame(kMhiFrame20Bytes);
  EXPECT_TRUE(capture.observe(frame, 1U, 0U, 1U).disposition == MhiCaptureDisposition::BASELINE);
  capture.reset();
  EXPECT_TRUE(capture.observe(frame, 1U, 0U, 2U).disposition == MhiCaptureDisposition::BASELINE);
}

void protocol_capture_recycles_oldest_slot_when_full() {
  MhiProtocolCaptureEngine capture{};
  const auto frame = make_capture_frame(kMhiFrame20Bytes);
  for (std::size_t i = 0; i < kMhiCaptureSlotCount; i++) {
    const auto result = capture.observe(frame, 2U, static_cast<uint16_t>(i), static_cast<uint32_t>(i + 1U));
    EXPECT_TRUE(result.disposition == MhiCaptureDisposition::BASELINE);
  }

  EXPECT_TRUE(capture.observe(frame, 2U, 0xFFFFU, 100U).disposition == MhiCaptureDisposition::BASELINE);
  EXPECT_TRUE(capture.observe(frame, 2U, 0U, 101U).disposition == MhiCaptureDisposition::BASELINE);
}

}  // namespace mhi_unit_tests
