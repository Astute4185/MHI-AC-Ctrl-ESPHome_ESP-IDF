#include "mhi_frame_sync.h"

#include <cstring>

#include "mhi_checksum.h"
#include "mhi_defs.h"

namespace esphome {
namespace mhi_ac_ctrl {

void MhiFrameSync::reset() {
  if (stats_ != nullptr) {
    stats_->on_frame_sync_reset();
  }

  buffer_len_ = 0;
  pending_frame_.clear();
  has_pending_frame_ = false;
}

bool MhiFrameSync::push_bytes(const uint8_t* data, std::size_t len) {
  if (data == nullptr || len == 0U) {
    return false;
  }

  for (std::size_t i = 0; i < len; i++) {
    if (buffer_len_ >= kSyncBufferBytes) {
      // Avoid heap allocation and keep the newest stream data.
      record_sync_loss_();
      discard_bytes(1U);
    }

    buffer_[buffer_len_++] = data[i];

    if (!has_pending_frame_) {
      process_buffer();
    }
  }

  return has_pending_frame_;
}

bool MhiFrameSync::pop_frame(MhiFrameBuffer& out) {
  if (!has_pending_frame_) {
    return false;
  }

  out = pending_frame_;
  pending_frame_.clear();
  has_pending_frame_ = false;

  process_buffer();

  return true;
}

bool MhiFrameSync::process_buffer() {
  if (has_pending_frame_) {
    return true;
  }

  while (buffer_len_ >= 3U) {
    if (!signature_matches_at(0U)) {
      record_signature_miss_();
      discard_bytes(1U);
      continue;
    }

    return try_extract_frame_at_start();
  }

  return false;
}

bool MhiFrameSync::signature_matches_at(std::size_t index) const {
  switch (mode_) {
    case MhiFrameSyncMode::MOSI_ONLY:
      return mosi_signature_matches_at(index);

    case MhiFrameSyncMode::MISO_ONLY:
      return miso_signature_matches_at(index);

    case MhiFrameSyncMode::ANY:
      return mosi_signature_matches_at(index) || miso_signature_matches_at(index);

    default:
      return false;
  }
}

bool MhiFrameSync::mosi_signature_matches_at(std::size_t index) const {
  if ((index + 2U) >= buffer_len_) {
    return false;
  }

  const uint8_t sb0 = buffer_[index + SB0];

  const bool sb0_ok = sb0 == kMhiMosiSignature0Default || sb0 == kMhiMosiSignature0Alt;

  return sb0_ok && buffer_[index + SB1] == kMhiMosiSignature1 && buffer_[index + SB2] == kMhiMosiSignature2;
}

bool MhiFrameSync::miso_signature_matches_at(std::size_t index) const {
  if ((index + 2U) >= buffer_len_) {
    return false;
  }

  return buffer_[index + SB0] == kMhiMisoSignature0 && buffer_[index + SB1] == kMhiMisoSignature1 &&
         buffer_[index + SB2] == kMhiMisoSignature2;
}

bool MhiFrameSync::try_extract_frame_at_start() {
  const std::size_t expected_frame_len = enable_33_byte_frames_ ? kMhiFrame33Bytes : kMhiFrame20Bytes;

  if (buffer_len_ < expected_frame_len) {
    return false;
  }

  if (stats_ != nullptr) {
    stats_->on_candidate_frame();
  }

  if (enable_33_byte_frames_) {
    // In 33-byte mode, do not accept the valid 20-byte prefix early. FastGPIO
    // delivers the full configured frame chunk, so consuming the prefix leaves
    // the 13-byte extension to be miscounted as dropped garbage. The extended
    // checksum layout is still conservative; accept either the confirmed 33-byte
    // checksum or the valid 20-byte base checksum, but consume the full 33 bytes.
    if (mhi_checksum_valid_33(buffer_) || mhi_checksum_valid_20(buffer_)) {
      store_pending_frame(kMhiFrame33Bytes);
      discard_bytes(kMhiFrame33Bytes);
      return true;
    }

    record_checksum_failure_();
    discard_bytes(1U);
    return process_buffer();
  }

  if (mhi_checksum_valid_20(buffer_)) {
    store_pending_frame(kMhiFrame20Bytes);
    discard_bytes(kMhiFrame20Bytes);
    return true;
  }

  // Signature matched but checksum failed. Drop one byte and rescan.
  record_checksum_failure_();
  discard_bytes(1U);
  return process_buffer();
}

void MhiFrameSync::discard_bytes(std::size_t count) {
  if (count == 0U) {
    return;
  }

  if (count >= buffer_len_) {
    buffer_len_ = 0;
    return;
  }

  std::memmove(buffer_, buffer_ + count, buffer_len_ - count);
  buffer_len_ -= count;
}

void MhiFrameSync::store_pending_frame(std::size_t frame_len) {
  pending_frame_.clear();
  pending_frame_.len = frame_len;

  std::memcpy(pending_frame_.data, buffer_, frame_len);

  has_pending_frame_ = true;
}

void MhiFrameSync::record_signature_miss_() {
  if (stats_ == nullptr) {
    return;
  }

  stats_->on_signature_miss();
  stats_->on_dropped_bytes(1U);
}

void MhiFrameSync::record_checksum_failure_() {
  if (stats_ == nullptr) {
    return;
  }

  stats_->on_invalid_frame();
  stats_->on_checksum_failure();
  stats_->on_sync_loss();
  stats_->on_dropped_bytes(1U);
}

void MhiFrameSync::record_sync_loss_() {
  if (stats_ == nullptr) {
    return;
  }

  stats_->on_sync_loss();
  stats_->on_dropped_bytes(1U);
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
