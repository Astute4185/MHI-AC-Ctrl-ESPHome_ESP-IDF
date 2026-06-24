#pragma once

#include <cmath>
#include <cstdint>

#include "mhi_command.h"
#include "mhi_state.h"

namespace esphome {
namespace mhi_ac_ctrl {

constexpr uint32_t kMhiCommandConfirmationTimeoutMs = 10000U;

class MhiCommandConfirmation {
 public:
  void reset() {
    this->pending_intent_ = {};
    this->pending_mask_ = 0U;
    this->staged_ms_ = 0U;
  }

  void stage(const MhiCommandIntent& intent, uint32_t encoded_command_mask, uint32_t now_ms) {
    const uint32_t confirmable = confirmable_mask(intent, encoded_command_mask);

    if (confirmable == 0U) {
      return;
    }

    this->pending_intent_ = intent;
    this->pending_intent_.mask = confirmable;
    this->pending_mask_ = confirmable;
    this->staged_ms_ = now_ms;
  }

  uint32_t observe_status(const MhiStatusState& status) {
    if (!status.valid || this->pending_mask_ == 0U) {
      return 0U;
    }

    uint32_t confirmed = 0U;

    if ((this->pending_mask_ & MHI_COMMAND_POWER) != 0U && status.power == this->pending_intent_.power) {
      confirmed |= MHI_COMMAND_POWER;
    }

    if ((this->pending_mask_ & MHI_COMMAND_MODE) != 0U && status.power && status.mode == this->pending_intent_.mode) {
      confirmed |= MHI_COMMAND_MODE;
    }

    if ((this->pending_mask_ & MHI_COMMAND_FAN) != 0U && status.fan == expected_status_fan(this->pending_intent_.fan)) {
      confirmed |= MHI_COMMAND_FAN;
    }

    if ((this->pending_mask_ & MHI_COMMAND_TARGET_TEMP) != 0U &&
        float_matches(status.target_temp_c, this->pending_intent_.target_temp_c)) {
      confirmed |= MHI_COMMAND_TARGET_TEMP;
    }

    if ((this->pending_mask_ & MHI_COMMAND_VERTICAL_VANE) != 0U) {
      if (this->pending_intent_.vertical_vane == 5U && status.vanes_swing) {
        confirmed |= MHI_COMMAND_VERTICAL_VANE;
      } else if (this->pending_intent_.vertical_vane >= 1U && this->pending_intent_.vertical_vane <= 4U &&
                 !status.vanes_swing && status.vertical_vane == this->pending_intent_.vertical_vane) {
        confirmed |= MHI_COMMAND_VERTICAL_VANE;
      }
    }

    this->pending_mask_ &= ~confirmed;
    this->pending_intent_.mask = this->pending_mask_;

    if (this->pending_mask_ == 0U) {
      this->staged_ms_ = 0U;
    }

    return confirmed;
  }

  uint32_t expire(uint32_t now_ms) {
    if (this->pending_mask_ == 0U || this->staged_ms_ == 0U || now_ms < this->staged_ms_) {
      return 0U;
    }

    if ((now_ms - this->staged_ms_) < kMhiCommandConfirmationTimeoutMs) {
      return 0U;
    }

    const uint32_t timed_out = this->pending_mask_;
    this->reset();
    return timed_out;
  }

  bool has_pending() const {
    return this->pending_mask_ != 0U;
  }

  uint32_t pending_mask() const {
    return this->pending_mask_;
  }

  const MhiCommandIntent& pending_intent() const {
    return this->pending_intent_;
  }

  uint32_t duplicate_pending_mask(const MhiCommandState& command) const {
    if (this->pending_mask_ == 0U) {
      return 0U;
    }

    uint32_t duplicate = 0U;

    if ((this->pending_mask_ & MHI_COMMAND_POWER) != 0U && command.power_set &&
        command.power == this->pending_intent_.power) {
      duplicate |= MHI_COMMAND_POWER;
    }

    if ((this->pending_mask_ & MHI_COMMAND_MODE) != 0U && command.mode_set &&
        command.mode == this->pending_intent_.mode) {
      duplicate |= MHI_COMMAND_MODE;
    }

    if ((this->pending_mask_ & MHI_COMMAND_FAN) != 0U && command.fan_set && command.fan == this->pending_intent_.fan) {
      duplicate |= MHI_COMMAND_FAN;
    }

    if ((this->pending_mask_ & MHI_COMMAND_TARGET_TEMP) != 0U && command.target_temp_set &&
        float_matches(command.target_temp_c, this->pending_intent_.target_temp_c)) {
      duplicate |= MHI_COMMAND_TARGET_TEMP;
    }

    if ((this->pending_mask_ & MHI_COMMAND_VERTICAL_VANE) != 0U && command.vertical_vane_set &&
        command.vertical_vane == this->pending_intent_.vertical_vane) {
      duplicate |= MHI_COMMAND_VERTICAL_VANE;
    }

    return duplicate;
  }

 private:
  static uint32_t confirmable_mask(const MhiCommandIntent& intent, uint32_t encoded_command_mask) {
    uint32_t mask =
        encoded_command_mask & static_cast<uint32_t>(MHI_COMMAND_POWER | MHI_COMMAND_MODE | MHI_COMMAND_FAN |
                                                     MHI_COMMAND_TARGET_TEMP | MHI_COMMAND_VERTICAL_VANE);

    if ((mask & MHI_COMMAND_FAN) != 0U && !fan_command_confirmable(intent.fan)) {
      mask &= ~static_cast<uint32_t>(MHI_COMMAND_FAN);
    }

    if ((mask & MHI_COMMAND_VERTICAL_VANE) != 0U && !(intent.vertical_vane >= 1U && intent.vertical_vane <= 5U)) {
      mask &= ~static_cast<uint32_t>(MHI_COMMAND_VERTICAL_VANE);
    }

    return mask;
  }

  static bool fan_command_confirmable(uint8_t command_fan) {
    return command_fan == 0U || command_fan == 1U || command_fan == 2U || command_fan == 6U;
  }

  static uint8_t expected_status_fan(uint8_t command_fan) {
    switch (command_fan) {
      case 0U:
        return 1U;
      case 1U:
        return 2U;
      case 2U:
        return 3U;
      case 6U:
        return 4U;
      default:
        return 0U;
    }
  }

  static bool float_matches(float actual, float expected) {
    return std::fabs(actual - expected) < 0.05F;
  }

  MhiCommandIntent pending_intent_{};
  uint32_t pending_mask_{0};
  uint32_t staged_ms_{0};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
