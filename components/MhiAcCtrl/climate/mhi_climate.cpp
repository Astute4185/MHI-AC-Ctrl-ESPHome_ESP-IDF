#include <cmath>
#include "esphome/core/log.h"
#include "esphome/core/version.h"
#include "mhi_climate.h"
#include "../mhi_time.h"

namespace esphome {
namespace mhi {

static const char *TAG = "mhi.climate";

bool MhiClimate::pending_window_active_() const {
    return (mhi_now_ms() - this->pending_until_ms_) <= 0x7FFFFFFFU &&
           mhi_now_ms() < this->pending_until_ms_;
}

void MhiClimate::start_pending_window_() {
    this->pending_until_ms_ = mhi_now_ms() + kCommandSettleWindowMs;
}

void MhiClimate::clear_expired_pending_() {
    if (this->pending_window_active_()) {
        return;
    }

    this->pending_power_valid_ = false;
    this->pending_mode_valid_ = false;
    this->pending_target_temperature_valid_ = false;
    this->pending_fan_valid_ = false;
    this->pending_swing_valid_ = false;
}

void MhiClimate::clear_all_pending_() {
    this->pending_power_valid_ = false;
    this->pending_mode_valid_ = false;
    this->pending_target_temperature_valid_ = false;
    this->pending_fan_valid_ = false;
    this->pending_swing_valid_ = false;
    this->pending_until_ms_ = 0;
}

void MhiClimate::setup() {
    this->power_ = power_off;
    this->current_temperature = NAN;

    float restored_target_temperature = NAN;
    auto restore = this->restore_state_();
    if (restore.has_value()) {
        restored_target_temperature = restore->target_temperature;
    }

    this->mode = climate::CLIMATE_MODE_OFF;
    this->fan_mode = climate::CLIMATE_FAN_AUTO;
    this->swing_mode = climate::CLIMATE_SWING_OFF;

    if (!std::isnan(restored_target_temperature)) {
        this->target_temperature = restored_target_temperature;
    } else {
        this->target_temperature = 20.0f;
    }

    this->vanesLR_pos_old_state_ = 4;
    this->vanes_pos_old_state_ = 4;

    this->platform_ = this->parent_;
    this->platform_->add_listener(this);

    this->publish_state();
}

void MhiClimate::dump_config() {
    ESP_LOGCONFIG(TAG, "MHI Climate");
}

void MhiClimate::update_status(ACStatus status, int value) {
    this->clear_expired_pending_();

    static int mode_tmp = 0xff;
    bool dirty = false;

    switch (status) {
    case status_power: {
        const ACPower new_power = (value == power_on) ? power_on : power_off;

        if (this->pending_power_valid_ && this->pending_window_active_() &&
            new_power != this->pending_power_) {
            ESP_LOGV(TAG, "Suppressing stale power status %d during pending window", value);
            break;
        }

        if (this->pending_power_valid_ && new_power == this->pending_power_) {
            this->pending_power_valid_ = false;
        }

        if (new_power == power_on) {
            if (this->power_ != power_on) {
                this->power_ = power_on;
                dirty = true;
            }
            update_status(status_mode, mode_tmp);
        } else {
            if (this->power_ != power_off) {
                this->power_ = power_off;
                dirty = true;
            }
            if (this->mode != climate::CLIMATE_MODE_OFF) {
                this->mode = climate::CLIMATE_MODE_OFF;
                dirty = true;
            }
            this->pending_mode_valid_ = false;
        }
        break;
    }

    case status_mode:
        mode_tmp = value;
        [[fallthrough]];
    case opdata_mode:
    case erropdata_mode: {
        auto new_mode = this->mode;

        switch (value) {
        case mode_auto:
            if (status != erropdata_mode && this->power_ > 0) {
                new_mode = climate::CLIMATE_MODE_HEAT_COOL;
            } else {
                new_mode = climate::CLIMATE_MODE_OFF;
            }
            break;
        case mode_dry:
            new_mode = climate::CLIMATE_MODE_DRY;
            break;
        case mode_cool:
            new_mode = climate::CLIMATE_MODE_COOL;
            break;
        case mode_fan:
            new_mode = climate::CLIMATE_MODE_FAN_ONLY;
            break;
        case mode_heat:
            new_mode = climate::CLIMATE_MODE_HEAT;
            break;
        default:
            ESP_LOGV(TAG, "unknown status mode value %i", value);
            break;
        }

        if (this->pending_mode_valid_ && this->pending_window_active_() &&
            new_mode != this->pending_mode_) {
            ESP_LOGV(TAG, "Suppressing stale mode status %d during pending window", value);
            break;
        }

        if (this->pending_mode_valid_ && new_mode == this->pending_mode_) {
            this->pending_mode_valid_ = false;
        }

        if (this->mode != new_mode) {
            this->mode = new_mode;
            dirty = true;
        }
        break;
    }

    case status_fan: {
        auto new_fan_mode = this->fan_mode;

        switch (value) {
        case 0:
            new_fan_mode = climate::CLIMATE_FAN_QUIET;
            break;
        case 1:
            new_fan_mode = climate::CLIMATE_FAN_LOW;
            break;
        case 2:
            new_fan_mode = climate::CLIMATE_FAN_MEDIUM;
            break;
        case 6:
            new_fan_mode = climate::CLIMATE_FAN_HIGH;
            break;
        case 7:
            new_fan_mode = climate::CLIMATE_FAN_AUTO;
            break;
        default:
            break;
        }

        if (this->pending_fan_valid_ && this->pending_window_active_() &&
            new_fan_mode != this->pending_fan_mode_) {
            ESP_LOGV(TAG, "Suppressing stale fan status %d during pending window", value);
            break;
        }

        if (this->pending_fan_valid_ && new_fan_mode == this->pending_fan_mode_) {
            this->pending_fan_valid_ = false;
        }

        if (this->fan_mode != new_fan_mode) {
            this->fan_mode = new_fan_mode;
            dirty = true;
        }
        break;
    }

    case status_vanes: {
        auto new_swing_mode = this->swing_mode;
        int new_vanes_pos_old_state = this->vanes_pos_old_state_;

        if (this->vanesLR_pos_state_ == vanesLR_swing) {
            switch (value) {
                case vanes_unknown:
                case vanes_1:
                case vanes_2:
                case vanes_3:
                case vanes_4:
                    new_swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
                    new_vanes_pos_old_state = value;
                    break;
                case vanes_swing:
                    new_swing_mode = climate::CLIMATE_SWING_BOTH;
                    break;
            }
        } else {
            switch (value) {
                case vanes_unknown:
                case vanes_1:
                case vanes_2:
                case vanes_3:
                case vanes_4:
                    new_swing_mode = climate::CLIMATE_SWING_OFF;
                    new_vanes_pos_old_state = value;
                    break;
                case vanes_swing:
                    new_swing_mode = climate::CLIMATE_SWING_VERTICAL;
                    break;
            }
        }

        if (this->pending_swing_valid_ && this->pending_window_active_() &&
            new_swing_mode != this->pending_swing_mode_) {
            ESP_LOGV(TAG, "Suppressing stale vertical vanes status %d during pending window", value);
            break;
        }

        if (this->pending_swing_valid_ && new_swing_mode == this->pending_swing_mode_) {
            this->pending_swing_valid_ = false;
        }

        if (this->swing_mode != new_swing_mode) {
            this->swing_mode = new_swing_mode;
            dirty = true;
        }
        if (this->vanes_pos_old_state_ != new_vanes_pos_old_state) {
            this->vanes_pos_old_state_ = new_vanes_pos_old_state;
            dirty = true;
        }
        if (this->vanes_pos_state_ != value) {
            this->vanes_pos_state_ = value;
            dirty = true;
        }
        break;
    }

    case status_vanesLR: {
        auto new_swing_mode = this->swing_mode;
        int new_vanesLR_pos_old_state = this->vanesLR_pos_old_state_;

        if (this->vanes_pos_state_ == vanes_swing) {
            switch (value) {
                case vanesLR_1:
                case vanesLR_2:
                case vanesLR_3:
                case vanesLR_4:
                case vanesLR_5:
                case vanesLR_6:
                case vanesLR_7:
                    new_swing_mode = climate::CLIMATE_SWING_VERTICAL;
                    new_vanesLR_pos_old_state = value;
                    break;
                case vanesLR_swing:
                    new_swing_mode = climate::CLIMATE_SWING_BOTH;
                    break;
            }
        } else {
            switch (value) {
                case vanesLR_1:
                case vanesLR_2:
                case vanesLR_3:
                case vanesLR_4:
                case vanesLR_5:
                case vanesLR_6:
                case vanesLR_7:
                    new_swing_mode = climate::CLIMATE_SWING_OFF;
                    new_vanesLR_pos_old_state = value;
                    break;
                case vanesLR_swing:
                    new_swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
                    break;
            }
        }

        if (this->pending_swing_valid_ && this->pending_window_active_() &&
            new_swing_mode != this->pending_swing_mode_) {
            ESP_LOGV(TAG, "Suppressing stale horizontal vanes status %d during pending window", value);
            break;
        }

        if (this->pending_swing_valid_ && new_swing_mode == this->pending_swing_mode_) {
            this->pending_swing_valid_ = false;
        }

        if (this->swing_mode != new_swing_mode) {
            this->swing_mode = new_swing_mode;
            dirty = true;
        }
        if (this->vanesLR_pos_old_state_ != new_vanesLR_pos_old_state) {
            this->vanesLR_pos_old_state_ = new_vanesLR_pos_old_state;
            dirty = true;
        }
        if (this->vanesLR_pos_state_ != value) {
            this->vanesLR_pos_state_ = value;
            dirty = true;
        }
        break;
    }

    case status_troom: {
        float new_current_temperature = ((value - 61) / 4.0f) - this->temperature_offset_;
        if (std::isnan(this->current_temperature) ||
            fabs(this->current_temperature - new_current_temperature) > 0.01f) {
            this->current_temperature = new_current_temperature;
            dirty = true;
        }
        break;
    }

    case status_tsetpoint: {
        float ac_setpoint = (value & 0x7f) / 2.0f;

        if (this->temperature_offset_ != 0.0f &&
            fabs(ac_setpoint - (this->target_temperature + this->temperature_offset_)) < 0.1f) {
            ESP_LOGV(TAG, "Ignoring tsetpoint echo from AC: %.1f", ac_setpoint);
            break;
        }

        if (this->pending_target_temperature_valid_ && this->pending_window_active_() &&
            fabs(ac_setpoint - this->pending_target_temperature_) > 0.11f) {
            ESP_LOGV(TAG, "Suppressing stale tsetpoint %.1f during pending window", ac_setpoint);
            break;
        }

        if (this->pending_target_temperature_valid_ &&
            fabs(ac_setpoint - this->pending_target_temperature_) <= 0.11f) {
            this->pending_target_temperature_valid_ = false;
        }

        if (fabs(this->target_temperature - ac_setpoint) > 0.01f) {
            this->target_temperature = ac_setpoint;
            dirty = true;
        }
        if (this->temperature_offset_ != 0.0f) {
            this->temperature_offset_ = 0.0f;
            dirty = true;
        }
        break;
    }

    default:
        break;
    }

    if (!this->pending_window_active_()) {
        this->clear_expired_pending_();
    } else if (!this->pending_power_valid_ && !this->pending_mode_valid_ &&
               !this->pending_target_temperature_valid_ && !this->pending_fan_valid_ &&
               !this->pending_swing_valid_) {
        this->pending_until_ms_ = 0;
    }

    if (dirty) {
        this->publish_state();
    }
}

void MhiClimate::control(const climate::ClimateCall &call) {
    bool had_change = false;

    if (call.get_target_temperature().has_value()) {
        float target_temp = *call.get_target_temperature();
        this->target_temperature = target_temp;

        const float ac_unit_min_temp = 18.0f;
        float setpoint = ceilf(target_temp);
        if (setpoint < ac_unit_min_temp)
            setpoint = ac_unit_min_temp;

        float offset = 0.0f;
        if (this->temperature_offset_enabled_) {
            offset = setpoint - target_temp;
        }
        this->platform_->set_offset(offset);
        this->temperature_offset_ = offset;
        this->platform_->set_tsetpoint(setpoint);

        this->pending_target_temperature_valid_ = true;
        this->pending_target_temperature_ = setpoint;
        had_change = true;

        ESP_LOGD(TAG, "Requested target_temperature %.1f°C (AC setpoint %.1f°C, offset %.1f°C)",
                 target_temp, setpoint, offset);
    }

    if (call.get_mode().has_value()) {
        this->mode = *call.get_mode();

        this->power_ = power_on;
        switch (this->mode) {
        case climate::CLIMATE_MODE_OFF:
            power_ = power_off;
            break;
        case climate::CLIMATE_MODE_COOL:
            mode_ = mode_cool;
            break;
        case climate::CLIMATE_MODE_HEAT:
            mode_ = mode_heat;
            break;
        case climate::CLIMATE_MODE_DRY:
            mode_ = mode_dry;
            break;
        case climate::CLIMATE_MODE_FAN_ONLY:
            mode_ = mode_fan;
            break;
        case climate::CLIMATE_MODE_HEAT_COOL:
        default:
            mode_ = mode_auto;
            break;
        }

        this->platform_->set_power(power_);
        this->platform_->set_mode(mode_);

        this->pending_power_valid_ = true;
        this->pending_power_ = power_;
        this->pending_mode_valid_ = true;
        this->pending_mode_ = this->mode;
        had_change = true;
    }

    if (call.get_fan_mode().has_value()) {
        this->fan_mode = *call.get_fan_mode();

        switch (this->fan_mode.value()) {
        case climate::CLIMATE_FAN_QUIET:
            fan_ = 0;
            break;
        case climate::CLIMATE_FAN_LOW:
            fan_ = 1;
            break;
        case climate::CLIMATE_FAN_MEDIUM:
            fan_ = 2;
            break;
        case climate::CLIMATE_FAN_HIGH:
            fan_ = 6;
            break;
        case climate::CLIMATE_FAN_AUTO:
        default:
            fan_ = 7;
            break;
        }

        this->platform_->set_fan(fan_);
        this->pending_fan_valid_ = true;
        this->pending_fan_mode_ = this->fan_mode.value();
        had_change = true;
    }

    if (call.get_swing_mode().has_value()) {
        this->swing_mode = *call.get_swing_mode();
        vanesLR_ = static_cast<ACVanesLR>(this->vanesLR_pos_old_state_);
        vanes_ = static_cast<ACVanes>(this->vanes_pos_old_state_);

        switch (this->swing_mode) {
        case climate::CLIMATE_SWING_OFF:
            break;
        case climate::CLIMATE_SWING_VERTICAL:
            vanes_ = vanes_swing;
            break;
        case climate::CLIMATE_SWING_HORIZONTAL:
            vanesLR_ = vanesLR_swing;
            break;
        case climate::CLIMATE_SWING_BOTH:
        default:
            vanesLR_ = vanesLR_swing;
            vanes_ = vanes_swing;
            break;
        }

        this->platform_->set_vanesLR(vanesLR_);
        this->platform_->set_vanes(vanes_);
        this->pending_swing_valid_ = true;
        this->pending_swing_mode_ = this->swing_mode;
        had_change = true;
    }

    if (had_change) {
        this->start_pending_window_();
    } else {
        this->clear_expired_pending_();
    }

    this->publish_state();
}

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
climate::ClimateTraits MhiClimate::traits() {
    auto traits = climate::ClimateTraits();
    traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
    traits.set_supported_modes({ climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT_COOL, climate::CLIMATE_MODE_COOL, climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_DRY, climate::CLIMATE_MODE_FAN_ONLY });
    traits.set_visual_min_temperature(this->minimum_temperature_);
    traits.set_visual_max_temperature(this->maximum_temperature_);
    traits.set_visual_temperature_step(this->temperature_step_);
    traits.set_supported_fan_modes({ climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_QUIET, CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_HIGH });
    traits.set_supported_swing_modes({ climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_BOTH, climate::CLIMATE_SWING_VERTICAL, climate::CLIMATE_SWING_HORIZONTAL });
    return traits;
}
#else
climate::ClimateTraits MhiClimate::traits() {
    auto traits = climate::ClimateTraits();
    traits.set_supports_current_temperature(true);
    traits.set_supported_modes({ CLIMATE_MODE_OFF, CLIMATE_MODE_HEAT_COOL, CLIMATE_MODE_COOL, CLIMATE_MODE_HEAT, CLIMATE_MODE_DRY, CLIMATE_MODE_FAN_ONLY });
    traits.set_supports_two_point_target_temperature(false);
    traits.set_visual_min_temperature(this->minimum_temperature_);
    traits.set_visual_max_temperature(this->maximum_temperature_);
    traits.set_visual_temperature_step(this->temperature_step_);
    traits.set_supported_fan_modes({ CLIMATE_FAN_AUTO, CLIMATE_FAN_QUIET, CLIMATE_FAN_LOW, CLIMATE_FAN_MEDIUM, CLIMATE_FAN_HIGH });
    traits.set_supported_swing_modes({ CLIMATE_SWING_OFF, CLIMATE_SWING_BOTH, CLIMATE_SWING_VERTICAL, CLIMATE_SWING_HORIZONTAL });
    return traits;
}
#endif

} // namespace mhi
} // namespace esphome