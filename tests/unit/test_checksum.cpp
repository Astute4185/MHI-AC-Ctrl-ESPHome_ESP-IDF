
#include "mhi_test_common.h"

namespace mhi_unit_tests {

void checksum_accepts_valid_20_byte_frame() {
  const MhiFrameBuffer frame = make_mosi_status_frame();

  EXPECT_EQ(mhi_calc_checksum(frame.data), static_cast<uint16_t>((frame.data[CBH] << 8U) | frame.data[CBL]));
  EXPECT_TRUE(mhi_checksum_valid_20(frame.data));
}

void checksum_rejects_bad_20_byte_frame() {
  MhiFrameBuffer frame = make_mosi_status_frame();
  frame.data[DB2] ^= 0x01U;

  EXPECT_FALSE(mhi_checksum_valid_20(frame.data));
}

}  // namespace mhi_unit_tests
