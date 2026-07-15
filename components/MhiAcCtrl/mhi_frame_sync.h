#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_frame.h"
#include "mhi_stats.h"

namespace esphome {
namespace mhi_ac_ctrl {

enum class MhiFrameSyncMode : uint8_t {
  MOSI_ONLY,
  MISO_ONLY,
  ANY,
};

class MhiFrameSync {
 public:
  void reset();

  void set_stats(MhiStats* stats) {
    stats_ = stats;
  }

  void set_mode(MhiFrameSyncMode mode) {
    mode_ = mode;
  }

  // Keep disabled by default until the 33-byte RX framing is fully confirmed.
  void set_33_byte_frames_enabled(bool enabled) {
    enable_33_byte_frames_ = enabled;
  }

  bool push_bytes(const uint8_t* data, std::size_t len);

  bool frame_available() const {
    return has_pending_frame_;
  }

  bool pop_frame(MhiFrameBuffer& out);

  std::size_t buffered_bytes() const {
    return buffer_len_;
  }

 private:
  static constexpr std::size_t kSyncBufferBytes = kMhiMaxFrameBytes * 3U;

  bool process_buffer();
  bool signature_matches_at(std::size_t index) const;
  bool mosi_signature_matches_at(std::size_t index) const;
  bool miso_signature_matches_at(std::size_t index) const;

  bool try_extract_frame_at_start();
  void discard_bytes(std::size_t count);
  void store_pending_frame(std::size_t frame_len);

  void record_signature_miss_();
  void record_checksum_failure_();
  void record_sync_loss_();

  MhiFrameSyncMode mode_{MhiFrameSyncMode::MOSI_ONLY};
  bool enable_33_byte_frames_{false};

  uint8_t buffer_[kSyncBufferBytes]{};
  std::size_t buffer_len_{0};

  MhiFrameBuffer pending_frame_{};
  bool has_pending_frame_{false};

  MhiStats* stats_{nullptr};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome