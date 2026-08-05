#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_tx_contract.h"

namespace esphome {
namespace mhi_ac_ctrl {

// Latest-value, one-shot TX mailbox used by duplex transports.
//
// Thread/ISR safety is deliberately external: callers must protect stage(),
// take(), and clear() with the transport's lock. Keeping the state container
// platform-neutral makes replacement and ownership semantics unit-testable.
class MhiDuplexTxMailbox {
 public:
  bool stage(const MhiTxEnvelope& envelope) {
    if (!envelope.valid()) {
      return false;
    }

    // Commands are lifecycle events and must not be displaced by routine
    // background traffic. A later command may still replace a background
    // frame, preserving command priority in the latest-value mailbox.
    if (pending_ && envelope_.is_command() && !envelope.is_command()) {
      rejected_background_frames_++;
      return false;
    }

    if (pending_) {
      overwritten_frames_++;
    }

    envelope_ = envelope;
    pending_ = true;
    return true;
  }

  bool take(uint8_t* dst, std::size_t capacity, MhiTxEnvelope& envelope) {
    envelope = {};

    if (!pending_ || dst == nullptr || capacity < envelope_.len) {
      return false;
    }

    envelope = envelope_;
    std::memcpy(dst, envelope.frame.data(), envelope.len);
    pending_ = false;
    envelope_ = {};
    return true;
  }

  void clear() {
    envelope_ = {};
    pending_ = false;
    overwritten_frames_ = 0U;
    rejected_background_frames_ = 0U;
  }

  bool pending() const {
    return pending_;
  }

  std::size_t pending_len() const {
    return envelope_.len;
  }

  uint32_t generation() const {
    return envelope_.generation;
  }

  MhiTxEnvelope pending_envelope() const {
    return pending_ ? envelope_ : MhiTxEnvelope{};
  }

  uint32_t overwritten_frames() const {
    return overwritten_frames_;
  }

  uint32_t rejected_background_frames() const {
    return rejected_background_frames_;
  }

 private:
  MhiTxEnvelope envelope_{};
  bool pending_{false};
  uint32_t overwritten_frames_{0U};
  uint32_t rejected_background_frames_{0U};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
