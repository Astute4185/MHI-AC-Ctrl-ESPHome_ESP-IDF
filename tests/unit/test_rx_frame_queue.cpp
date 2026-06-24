#include "mhi_test_common.h"

namespace mhi_unit_tests {

void rx_frame_queue_preserves_fifo_order() {
  MhiRxFrameQueue queue{};

  const MhiFrameBuffer first = make_mosi_status_frame();
  MhiFrameBuffer second = make_mosi_status_frame();
  second.data[DB1] = 0x22U;

  EXPECT_TRUE(queue.push(first.data, first.len));
  EXPECT_TRUE(queue.push(second.data, second.len));
  EXPECT_EQ(queue.size(), 2U);

  MhiFrameBuffer out{};
  EXPECT_TRUE(queue.pop(out));
  EXPECT_EQ(out.len, first.len);
  EXPECT_EQ(out.data[DB1], first.data[DB1]);

  EXPECT_TRUE(queue.pop(out));
  EXPECT_EQ(out.len, second.len);
  EXPECT_EQ(out.data[DB1], second.data[DB1]);

  EXPECT_FALSE(queue.pop(out));
  EXPECT_TRUE(queue.empty());
}

void rx_frame_queue_rejects_overflow_and_invalid_inputs() {
  MhiRxFrameQueue queue{};
  const MhiFrameBuffer frame = make_mosi_status_frame();

  EXPECT_FALSE(queue.push(nullptr, frame.len));
  EXPECT_FALSE(queue.push(frame.data, 0U));
  EXPECT_FALSE(queue.push(frame.data, kMhiMaxFrameBytes + 1U));

  for (std::size_t i = 0U; i < queue.capacity(); i++) {
    EXPECT_TRUE(queue.push(frame.data, frame.len));
  }

  EXPECT_TRUE(queue.full());
  EXPECT_FALSE(queue.push(frame.data, frame.len));

  MhiFrameBuffer out{};
  EXPECT_TRUE(queue.pop(out));
  EXPECT_EQ(queue.size(), queue.capacity() - 1U);

  queue.clear();
  EXPECT_TRUE(queue.empty());
}

}  // namespace mhi_unit_tests
