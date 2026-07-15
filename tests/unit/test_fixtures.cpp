#include "mhi_test_common.h"

namespace mhi_unit_tests {

void fixture_valid_status_frame_decodes() {
  const MhiFrameBuffer frame = load_frame_fixture("tests/fixtures/mosi_valid_status_20b.hex");

  EXPECT_EQ(frame.len, kMhiFrame20Bytes);
  EXPECT_TRUE(mhi_checksum_valid_20(frame.data));

  MhiDecodedStatus decoded{};
  EXPECT_TRUE(MhiStatusDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.power);
  EXPECT_EQ(decoded.mode, 2);
  EXPECT_EQ(decoded.fan, 2);
  EXPECT_EQ(decoded.vertical_vane, 3);
  expect_near(decoded.target_temp_c, 22.0f);
  expect_near(decoded.room_temp_c, 24.0f);
  EXPECT_EQ(decoded.error_code, 0);
}

void fixture_bad_checksum_rejects() {
  const MhiFrameBuffer frame = load_frame_fixture("tests/fixtures/mosi_bad_checksum_20b.hex");

  EXPECT_EQ(frame.len, kMhiFrame20Bytes);
  EXPECT_FALSE(mhi_checksum_valid_20(frame.data));
}

void fixture_garbage_then_valid_frame_resyncs() {
  const auto bytes = load_hex_fixture("tests/fixtures/mosi_garbage_then_valid_20b.hex");

  MhiFrameSync sync{};
  EXPECT_TRUE(sync.push_bytes(bytes.data(), bytes.size()));
  EXPECT_TRUE(sync.frame_available());

  MhiFrameBuffer out{};
  EXPECT_TRUE(sync.pop_frame(out));

  EXPECT_EQ(out.len, kMhiFrame20Bytes);
  EXPECT_EQ(out.data[SB0], kMhiMosiSignature0Default);
  EXPECT_EQ(out.data[SB1], kMhiMosiSignature1);
  EXPECT_EQ(out.data[SB2], kMhiMosiSignature2);
  EXPECT_TRUE(mhi_checksum_valid_20(out.data));
}

void fixture_opdata_outdoor_temp_decodes() {
  const MhiFrameBuffer frame = load_frame_fixture("tests/fixtures/mosi_opdata_outdoor_temp_20b.hex");

  EXPECT_EQ(frame.len, kMhiFrame20Bytes);
  EXPECT_TRUE(mhi_checksum_valid_20(frame.data));

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_outdoor_temp);
  expect_near(decoded.outdoor_temp_c, 27.0f);
}

void fixture_opdata_current_decodes() {
  const MhiFrameBuffer frame = load_frame_fixture("tests/fixtures/mosi_opdata_current_20b.hex");

  EXPECT_EQ(frame.len, kMhiFrame20Bytes);
  EXPECT_TRUE(mhi_checksum_valid_20(frame.data));

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_current);
  expect_near(decoded.current_a, 14.0f);
}

}  // namespace mhi_unit_tests
