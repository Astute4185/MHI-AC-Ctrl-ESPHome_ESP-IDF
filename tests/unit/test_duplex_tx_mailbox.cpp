#include "mhi_test_common.h"

#include <array>

#include "mhi_duplex_tx_mailbox.h"

namespace mhi_unit_tests {

void duplex_tx_mailbox_stages_and_consumes_20_byte_frame() {
  MhiDuplexTxMailbox mailbox{};
  const MhiFrameBuffer frame = make_mosi_status_frame();

  EXPECT_TRUE(mailbox.stage(frame.data, frame.len));
  EXPECT_TRUE(mailbox.pending());
  EXPECT_EQ(mailbox.pending_len(), kMhiFrame20Bytes);
  EXPECT_EQ(mailbox.generation(), 1U);

  std::array<uint8_t, kMhiMaxFrameBytes> destination{};
  std::size_t len = 0U;
  uint32_t generation = 0U;

  EXPECT_TRUE(mailbox.take(destination.data(), destination.size(), len, generation));
  EXPECT_EQ(len, kMhiFrame20Bytes);
  EXPECT_EQ(generation, 1U);
  EXPECT_EQ(destination[SB0], frame.data[SB0]);
  EXPECT_EQ(destination[CBL], frame.data[CBL]);
  EXPECT_FALSE(mailbox.pending());
  EXPECT_FALSE(mailbox.take(destination.data(), destination.size(), len, generation));
}

void duplex_tx_mailbox_latest_stage_replaces_unclaimed_frame() {
  MhiDuplexTxMailbox mailbox{};
  MhiFrameBuffer first = make_mosi_status_frame();
  MhiFrameBuffer second = make_mosi_status_frame_33(4U, false, false);
  first.data[DB4] = 0x11U;
  second.data[DB4] = 0x22U;

  EXPECT_TRUE(mailbox.stage(first.data, first.len));
  EXPECT_TRUE(mailbox.stage(second.data, second.len));
  EXPECT_EQ(mailbox.overwritten_frames(), 1U);
  EXPECT_EQ(mailbox.generation(), 2U);
  EXPECT_EQ(mailbox.pending_len(), kMhiFrame33Bytes);

  std::array<uint8_t, kMhiMaxFrameBytes> destination{};
  std::size_t len = 0U;
  uint32_t generation = 0U;

  EXPECT_TRUE(mailbox.take(destination.data(), destination.size(), len, generation));
  EXPECT_EQ(len, kMhiFrame33Bytes);
  EXPECT_EQ(generation, 2U);
  EXPECT_EQ(destination[DB4], 0x22U);
}

void duplex_tx_mailbox_rejects_invalid_frames_without_losing_pending_data() {
  MhiDuplexTxMailbox mailbox{};
  const MhiFrameBuffer frame = make_mosi_status_frame();
  const std::array<uint8_t, 21U> invalid{};

  EXPECT_TRUE(mailbox.stage(frame.data, frame.len));
  EXPECT_FALSE(mailbox.stage(nullptr, frame.len));
  EXPECT_FALSE(mailbox.stage(invalid.data(), invalid.size()));
  EXPECT_TRUE(mailbox.pending());
  EXPECT_EQ(mailbox.generation(), 1U);
  EXPECT_EQ(mailbox.overwritten_frames(), 0U);

  std::array<uint8_t, 10U> too_small{};
  std::size_t len = 0U;
  uint32_t generation = 0U;
  EXPECT_FALSE(mailbox.take(too_small.data(), too_small.size(), len, generation));
  EXPECT_TRUE(mailbox.pending());
}

}  // namespace mhi_unit_tests
