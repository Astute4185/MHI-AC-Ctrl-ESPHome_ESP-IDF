#include <algorithm>
#include <cstring>
#include <vector>

#include "mhi_rx_runtime.h"
#include "mhi_test_common.h"

namespace mhi_unit_tests {
namespace {

class RxRuntimeTestSource final : public IMhiRxSource {
 public:
  explicit RxRuntimeTestSource(const MhiFrameBuffer& frame)
      : bytes_(frame.data, frame.data + frame.len) {}

  std::size_t read_rx(uint8_t* dst, std::size_t max_len) override {
    if (read_) {
      return 0U;
    }
    const std::size_t len = std::min(max_len, bytes_.size());
    std::memcpy(dst, bytes_.data(), len);
    read_ = true;
    return len;
  }

 private:
  std::vector<uint8_t> bytes_{};
  bool read_{false};
};

}  // namespace

void rx_runtime_services_transport_and_catalogs_status() {
  MhiStats stats{};
  stats.reset();
  MhiRxRuntime runtime{};
  runtime.configure(&stats, false);

  RxRuntimeTestSource transport(make_mosi_status_frame());
  const MhiRxServiceResult result = runtime.service(transport, 4U, false);

  EXPECT_EQ(result.chunks, 1U);
  EXPECT_EQ(result.frames, 1U);
  EXPECT_EQ(stats.snapshot().valid_frames, 1U);

  MhiCatalogedFrame cataloged{};
  EXPECT_TRUE(runtime.take_latest_status(cataloged));
  EXPECT_EQ(static_cast<uint8_t>(cataloged.kind), static_cast<uint8_t>(MhiFrameKind::STATUS));
  EXPECT_EQ(cataloged.frame.len, kMhiFrame20Bytes);
  EXPECT_FALSE(runtime.take_latest_status(cataloged));
}

void rx_runtime_preserves_command_candidate_and_worker_handoff() {
  MhiStats stats{};
  stats.reset();
  MhiRxRuntime runtime{};
  runtime.configure(&stats, false);

  RxRuntimeTestSource transport(make_mosi_status_frame());
  const MhiRxServiceResult result = runtime.service(transport, 4U, true);
  EXPECT_EQ(result.frames, 1U);
  EXPECT_TRUE(runtime.decode_cataloged_frames_to_worker_store(true));

  MhiDecodedStatusSnapshot candidate{};
  EXPECT_TRUE(runtime.take_worker_command_candidate(candidate));
  EXPECT_TRUE(candidate.decoded.valid);
  EXPECT_EQ(candidate.frame.len, kMhiFrame20Bytes);

  MhiDecodedStatusSnapshot status{};
  EXPECT_TRUE(runtime.take_worker_status(status));
  EXPECT_TRUE(status.decoded.valid);

  runtime.clear_command_candidate();
  EXPECT_FALSE(runtime.take_worker_command_candidate(candidate));
}

void rx_runtime_reset_clears_catalog_and_worker_state() {
  MhiStats stats{};
  stats.reset();
  MhiRxRuntime runtime{};
  runtime.configure(&stats, false);

  RxRuntimeTestSource transport(make_mosi_status_frame());
  runtime.service(transport, 4U, true);
  runtime.decode_cataloged_frames_to_worker_store(true);
  runtime.reset();

  MhiCatalogedFrame cataloged{};
  MhiDecodedStatusSnapshot snapshot{};
  EXPECT_FALSE(runtime.take_latest_status(cataloged));
  EXPECT_FALSE(runtime.take_worker_status(snapshot));
  EXPECT_EQ(runtime.catalog_stats().ingested_frames, 0U);
  EXPECT_EQ(runtime.worker_store_stats().status_writes, 0U);
}

}  // namespace mhi_unit_tests
