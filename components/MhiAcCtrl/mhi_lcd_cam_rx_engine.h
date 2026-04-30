#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_transport.h"

namespace esphome {
namespace mhi {

class MhiLcdCamRxEngine {
 public:
  bool setup(const MhiTransportConfig &config);

  MhiFrameExchangeResult exchange_frame(
      const uint8_t *tx_frame,
      uint8_t *rx_frame,
      std::size_t rx_capacity,
      uint32_t max_time_ms);

 private:
  void maybe_log_chunk_(
      const uint8_t *frame,
      std::size_t len,
      bool extension_seen,
      int status,
      bool header_candidate,
      uint8_t pack_mode);

  void update_reference_stats_(const uint8_t *frame, std::size_t len, bool valid_header);

  MhiTransportConfig config_{};
  uint32_t capture_counter_{0};
  uint32_t bad_capture_counter_{0};
  uint32_t last_dump_ms_{0};
  bool has_reference_{false};
  std::size_t reference_len_{0};
  uint8_t reference_frame_[33]{};
  int first_bad_index_{-1};
  int last_good_index_{-1};
  int last_bad_index_{-1};
  uint32_t mismatch_count_{0};
  uint32_t identical_to_reference_count_{0};
  uint32_t post20_corrupt_count_{0};
  uint32_t post31_corrupt_count_{0};
  uint32_t tail_only_corrupt_count_{0};
};

}  // namespace mhi
}  // namespace esphome