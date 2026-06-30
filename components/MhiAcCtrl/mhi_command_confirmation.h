#pragma once

#include <cmath>
#include <cstdint>

#include "mhi_command.h"
#include "mhi_state.h"

namespace esphome {
namespace mhi_ac_ctrl {

constexpr uint32_t kMhiCommandConfirmationTimeoutMs = 10000U;
constexpr uint32_t kMhiExtendedLouverConfirmationTimeoutMs = 20000U;
constexpr uint32_t kMhiExtendedLouverSettleDelayMs = 3000U;

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

    if ((this->pending_mask_ & MHI_COMMAND_HORIZONTAL_VANE) != 0U && status.has_horizontal_vane) {
      if (this->pending_intent_.horizontal_vane == 8U && status.horizontal_vane_swing) {
        confirmed |= MHI_COMMAND_HORIZONTAL_VANE;
      } else if (this->pending_intent_.horizontal_vane >= 1U && this->pending_intent_.horizontal_vane <= 7U &&
                 !status.horizontal_vane_swing && status.horizontal_vane == this->pending_intent_.horizontal_vane) {
        confirmed |= MHI_COMMAND_HORIZONTAL_VANE;
      }
    }

    if ((this->pending_mask_ & MHI_COMMAND_THREE_D_AUTO) != 0U && status.has_3d_auto &&
        status.three_d_auto == this->pending_intent_.three_d_auto) {
      if (!this->pending_intent_.has_extended_louver_context ||
          this->extended_louver_context_matches(status, this->pending_intent_)) {
        confirmed |= MHI_COMMAND_THREE_D_AUTO;
      }
    }

    this->pending_mask_ &= ~confirmed;
    this->pending_intent_.mask = this->pending_mask_;

    if (this->pending_mask_ == 0U) {
      this->staged_ms_ = 0U;
    }

    return confirmed;
  }

  uint32_t settle_pending_mask(uint32_t mask) {
    const uint32_t settled = this->pending_mask_ & mask;
    if (settled == 0U) {
      return 0U;
    }

    this->pending_mask_ &= ~settled;
    this->pending_intent_.mask = this->pending_mask_;

    if (this->pending_mask_ == 0U) {
      this->staged_ms_ = 0U;
    }

    return settled;
  }

  uint32_t pending_age_ms(uint32_t now_ms) const {
    if (this->pending_mask_ == 0U || this->staged_ms_ == 0U || now_ms < this->staged_ms_) {
      return 0U;
    }

    return now_ms - this->staged_ms_;
  }

  uint32_t expire(uint32_t now_ms) {
    if (this->pending_mask_ == 0U || this->staged_ms_ == 0U || now_ms < this->staged_ms_) {
      return 0U;
    }

    const uint32_t timeout_ms = (this->pending_mask_ & (MHI_COMMAND_HORIZONTAL_VANE | MHI_COMMAND_THREE_D_AUTO)) != 0U
                                    ? kMhiExtendedLouverConfirmationTimeoutMs
                                    : kMhiCommandConfirmationTimeoutMs;

    if ((now_ms - this->staged_ms_) < timeout_ms) {
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

    if ((this->pending_mask_ & MHI_COMMAND_HORIZONTAL_VANE) != 0U && command.horizontal_vane_set &&
        command.horizontal_vane == this->pending_intent_.horizontal_vane) {
      duplicate |= MHI_COMMAND_HORIZONTAL_VANE;
    }

    if ((this->pending_mask_ & MHI_COMMAND_THREE_D_AUTO) != 0U && command.three_d_auto_set &&
        command.three_d_auto == this->pending_intent_.three_d_auto) {
      duplicate |= MHI_COMMAND_THREE_D_AUTO;
    }

    return duplicate;
  }

 private:
  static uint32_t confirmable_mask(const MhiCommandIntent& intent, uint32_t encoded_command_mask) {
    uint32_t mask =
        encoded_command_mask &
        static_cast<uint32_t>(MHI_COMMAND_POWER | MHI_COMMAND_MODE | MHI_COMMAND_FAN | MHI_COMMAND_TARGET_TEMP |
                              MHI_COMMAND_VERTICAL_VANE | MHI_COMMAND_HORIZONTAL_VANE | MHI_COMMAND_THREE_D_AUTO);

    if ((mask & MHI_COMMAND_FAN) != 0U && !fan_command_confirmable(intent.fan)) {
      mask &= ~static_cast<uint32_t>(MHI_COMMAND_FAN);
    }

    if ((mask & MHI_COMMAND_VERTICAL_VANE) != 0U && !(intent.vertical_vane >= 1U && intent.vertical_vane <= 5U)) {
      mask &= ~static_cast<uint32_t>(MHI_COMMAND_VERTICAL_VANE);
    }

    if ((mask & MHI_COMMAND_HORIZONTAL_VANE) != 0U && !(intent.horizontal_vane >= 1U && intent.horizontal_vane <= 8U)) {
      mask &= ~static_cast<uint32_t>(MHI_COMMAND_HORIZONTAL_VANE);
    }

    return mask;
  }

  static bool extended_louver_context_matches(const MhiStatusState& status, const MhiCommandIntent& intent) {
    if (!status.has_horizontal_vane) {
      return false;
    }

    if (intent.horizontal_vane == 8U) {
      return status.horizontal_vane_swing;
    }

    if (intent.horizontal_vane >= 1U && intent.horizontal_vane <= 7U) {
      return !status.horizontal_vane_swing && status.horizontal_vane == intent.horizontal_vane;
    }

    return false;
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
