#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace mhi_ac_ctrl {

// Temporary diagnostic state machine for characterising outdoor-unit Silent
// Mode. The production command lifecycle must not depend on these timings.
class MhiSilentModeSpike {
 public:
  enum class Observation : uint8_t {
    IDLE = 0,
    MISMATCH,
    MATCH,
  };

  static constexpr std::array<uint32_t, 7> kPollOffsetsMs{{100U, 250U, 500U, 1000U, 2000U, 5000U, 10000U}};
  static constexpr uint32_t kObservationWindowMs = 12000U;

  void reset() {
    *this = {};
  }

  void arm_request(bool target, bool baseline_known, bool baseline_state, uint32_t requested_at_ms) {
    this->target_ = target;
    this->baseline_known_ = baseline_known;
    this->baseline_state_ = baseline_state;
    this->requested_at_ms_ = requested_at_ms;
    this->command_completed_ms_ = 0U;
    this->last_poll_staged_ms_ = 0U;
    this->last_response_ms_ = 0U;
    this->poll_index_ = 0U;
    this->polls_staged_ = 0U;
    this->responses_seen_ = 0U;
    this->awaiting_command_completion_ = true;
    this->active_ = false;
    this->matched_ = false;
    this->expired_ = false;
  }

  void command_completed(bool target, uint32_t completed_at_ms) {
    this->target_ = target;
    this->command_completed_ms_ = completed_at_ms;
    this->last_poll_staged_ms_ = 0U;
    this->last_response_ms_ = 0U;
    this->poll_index_ = 0U;
    this->polls_staged_ = 0U;
    this->responses_seen_ = 0U;
    this->awaiting_command_completion_ = false;
    this->active_ = true;
    this->matched_ = false;
    this->expired_ = false;
  }

  void command_failed() {
    this->awaiting_command_completion_ = false;
    this->active_ = false;
  }

  bool poll_due(uint32_t now_ms) const {
    if (!this->active_ || this->command_completed_ms_ == 0U || this->poll_index_ >= kPollOffsetsMs.size() ||
        now_ms < this->command_completed_ms_) {
      return false;
    }
    return (now_ms - this->command_completed_ms_) >= kPollOffsetsMs[this->poll_index_];
  }

  uint32_t next_poll_offset_ms() const {
    return this->poll_index_ < kPollOffsetsMs.size() ? kPollOffsetsMs[this->poll_index_] : 0U;
  }

  void mark_poll_staged(uint32_t now_ms) {
    if (!this->active_ || this->poll_index_ >= kPollOffsetsMs.size()) {
      return;
    }
    this->last_poll_staged_ms_ = now_ms;
    this->polls_staged_++;
    this->poll_index_++;
  }

  Observation observe(bool state, uint32_t now_ms) {
    if (!this->active_) {
      return Observation::IDLE;
    }

    this->last_response_ms_ = now_ms;
    this->responses_seen_++;
    if (state == this->target_) {
      this->matched_ = true;
      this->active_ = false;
      return Observation::MATCH;
    }
    return Observation::MISMATCH;
  }

  bool expire_if_due(uint32_t now_ms) {
    if (!this->active_ || this->command_completed_ms_ == 0U || now_ms < this->command_completed_ms_ ||
        this->poll_index_ < kPollOffsetsMs.size() || (now_ms - this->command_completed_ms_) < kObservationWindowMs) {
      return false;
    }
    this->active_ = false;
    this->expired_ = true;
    return true;
  }

  bool active() const {
    return this->active_;
  }
  bool awaiting_command_completion() const {
    return this->awaiting_command_completion_;
  }
  bool target() const {
    return this->target_;
  }
  bool baseline_known() const {
    return this->baseline_known_;
  }
  bool baseline_state() const {
    return this->baseline_state_;
  }
  bool matched() const {
    return this->matched_;
  }
  bool expired() const {
    return this->expired_;
  }
  uint32_t requested_at_ms() const {
    return this->requested_at_ms_;
  }
  uint32_t command_completed_ms() const {
    return this->command_completed_ms_;
  }
  uint32_t last_poll_staged_ms() const {
    return this->last_poll_staged_ms_;
  }
  uint32_t last_response_ms() const {
    return this->last_response_ms_;
  }
  std::size_t poll_index() const {
    return this->poll_index_;
  }
  uint32_t polls_staged() const {
    return this->polls_staged_;
  }
  uint32_t responses_seen() const {
    return this->responses_seen_;
  }

 private:
  bool target_{false};
  bool baseline_known_{false};
  bool baseline_state_{false};
  bool awaiting_command_completion_{false};
  bool active_{false};
  bool matched_{false};
  bool expired_{false};
  uint32_t requested_at_ms_{0U};
  uint32_t command_completed_ms_{0U};
  uint32_t last_poll_staged_ms_{0U};
  uint32_t last_response_ms_{0U};
  std::size_t poll_index_{0U};
  uint32_t polls_staged_{0U};
  uint32_t responses_seen_{0U};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
