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
      uint8_t pack_mode,
      bool base_gate_passed,
      bool extension_stage_complete,
      bool extension_gap_rejected,
      bool provisional_extension_rejected,
      bool frame_suppressed,
      uint32_t extension_gap_us);

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
  uint32_t boundary_focus_mismatch_counts_[33]{};
  uint32_t base_gate_pass_count_{0};
  uint32_t extension_probe_seen_count_{0};
  uint32_t extension_complete_count_{0};
  uint32_t extension_probe_timeout_count_{0};
  uint32_t extension_skipped_bad_base_count_{0};
  uint32_t extension_gap_good_count_{0};
  uint32_t extension_gap_short_count_{0};
  uint32_t extension_gap_reject_count_{0};
  uint32_t extension_publish_reject_count_{0};
  uint32_t resync_suppressed_count_{0};
  uint32_t resync_cooldown_return_count_{0};
  uint64_t resync_cooldown_until_us_{0};
  uint32_t last_extension_gap_us_{0};
};

}  // namespace mhi
}  // namespace esphome
