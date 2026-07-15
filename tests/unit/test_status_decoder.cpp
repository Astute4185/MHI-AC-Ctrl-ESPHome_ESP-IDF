#include "mhi_test_common.h"

namespace mhi_unit_tests {

void status_decoder_decodes_core_fields() {
  const MhiFrameBuffer frame = make_mosi_status_frame();

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


void status_decoder_decodes_33_byte_vane_feedback() {
  const MhiFrameBuffer frame = make_mosi_status_frame_33(6U, false, true);

  MhiDecodedStatus decoded{};
  EXPECT_TRUE(MhiStatusDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.extended);
  EXPECT_TRUE(decoded.has_horizontal_vane);
  EXPECT_FALSE(decoded.horizontal_swing);
  EXPECT_EQ(decoded.horizontal_vane, 6U);
  EXPECT_TRUE(decoded.has_3d_auto);
  EXPECT_TRUE(decoded.three_d_auto);
}

void status_decoder_ignores_33_byte_vane_feedback_on_opdata_frames() {
  MhiFrameBuffer frame = make_mosi_status_frame_33(6U, false, true);
  frame.data[DB6] = 0x80U;
  frame.data[DB9] = 0x80U;
  frame.data[DB10] = 0x20U;
  frame.data[DB11] = 0x50U;
  frame.data[DB16] = 0x00U;
  frame.data[DB17] = 0x05U;

  const uint16_t checksum = mhi_calc_checksum_frame33(frame.data);
  frame.data[CBL2] = static_cast<uint8_t>(checksum & 0xFFU);

  MhiDecodedStatus decoded{};
  EXPECT_TRUE(MhiStatusDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.extended);
  EXPECT_FALSE(MhiStatusDecoder::is_extended_feedback_status_frame(frame.view()));
  EXPECT_FALSE(decoded.has_horizontal_vane);
  EXPECT_FALSE(decoded.has_3d_auto);
}

void status_decoder_ignores_unknown_horizontal_vane_feedback() {
  MhiFrameBuffer frame = make_mosi_status_frame();
  frame.len = kMhiFrame33Bytes;

  frame.data[DB16] = 0x07U;
  frame.data[DB17] = 0x00U;

  MhiDecodedStatus decoded{};
  EXPECT_TRUE(MhiStatusDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_FALSE(decoded.has_horizontal_vane);
}

}  // namespace mhi_unit_tests
