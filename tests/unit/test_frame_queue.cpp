#include "mhi_test_common.h"

namespace mhi_unit_tests {

void frame_queue_preserves_complete_frames() {
  MhiFrameQueue<2U> queue{};
  const MhiFrameBuffer first = make_mosi_status_frame();
  const MhiFrameBuffer second = make_mosi_status_frame_33(3U, false, true);

  EXPECT_TRUE(queue.push(first.data, first.len, 11U, 1000U));
  EXPECT_EQ(queue.high_water_mark(), 1U);
  EXPECT_TRUE(queue.push(second.data, second.len, 12U, 2000U));
  EXPECT_EQ(queue.size(), 2U);
  EXPECT_EQ(queue.high_water_mark(), 2U);
  EXPECT_EQ(queue.overwritten_frames(), 0U);

  MhiCapturedFrame captured{};
  EXPECT_TRUE(queue.pop(captured));
  EXPECT_EQ(captured.len, kMhiFrame20Bytes);
  EXPECT_EQ(captured.sequence, 11U);
  EXPECT_EQ(captured.frame_end_us, 1000U);
  EXPECT_EQ(captured.data[SB0], first.data[SB0]);
  EXPECT_EQ(captured.data[CBL], first.data[CBL]);

  EXPECT_TRUE(queue.pop(captured));
  EXPECT_EQ(captured.len, kMhiFrame33Bytes);
  EXPECT_EQ(captured.sequence, 12U);
  EXPECT_EQ(captured.frame_end_us, 2000U);
  EXPECT_EQ(captured.data[DB17], second.data[DB17]);
  EXPECT_EQ(captured.data[CBL2], second.data[CBL2]);
  EXPECT_FALSE(queue.pop(captured));
  EXPECT_EQ(queue.high_water_mark(), 2U);

  queue.clear();
  EXPECT_EQ(queue.size(), 0U);
  EXPECT_EQ(queue.high_water_mark(), 0U);
}

void frame_queue_overwrites_oldest_complete_frame() {
  MhiFrameQueue<2U> queue{};
  const MhiFrameBuffer first = make_mosi_status_frame();
  MhiFrameBuffer second = make_mosi_status_frame();
  MhiFrameBuffer third = make_mosi_status_frame();
  second.data[DB4] = 0x22U;
  third.data[DB4] = 0x33U;

  EXPECT_TRUE(queue.push(first.data, first.len, 1U, 100U));
  EXPECT_TRUE(queue.push(second.data, second.len, 2U, 200U));
  EXPECT_TRUE(queue.push(third.data, third.len, 3U, 300U));

  EXPECT_EQ(queue.size(), 2U);
  EXPECT_EQ(queue.high_water_mark(), 2U);
  EXPECT_EQ(queue.overwritten_frames(), 1U);

  MhiCapturedFrame captured{};
  EXPECT_TRUE(queue.pop(captured));
  EXPECT_EQ(captured.sequence, 2U);
  EXPECT_EQ(captured.data[DB4], 0x22U);

  EXPECT_TRUE(queue.pop(captured));
  EXPECT_EQ(captured.sequence, 3U);
  EXPECT_EQ(captured.data[DB4], 0x33U);
}

}  // namespace mhi_unit_tests
