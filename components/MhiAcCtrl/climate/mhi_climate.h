#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/core/time.h"
#include <cstdint>
#include <vector>
#include "../mhi_platform.h"

using namespace esphome::climate;

namespace esphome {
namespace mhi {

class MhiClimate :
    public climate::Climate,
    public Component,
    public Parented<MhiPlatform>,
    protected MhiStatusListener {

protected:
    void setup() override;
    void dump_config() override;
    void control(const climate::ClimateCall& call) override;
    climate::ClimateTraits traits() override;
    void update_status(ACStatus status, int value) override;

private:
    static constexpr uint32_t kCommandSettleWindowMs = 1500U;

    bool temperature_offset_enabled_{false};
    float temperature_offset_{0.0f};
    float minimum_temperature_{18.0f};
    float maximum_temperature_{30.0f};
    float temperature_step_{0.5f};

    ACPower power_;
    ACMode mode_;
    float tsetpoint_;
    uint32_t fan_;
    ACVanes vanes_;
    ACVanesLR vanesLR_;
    int vanesLR_pos_old_state_;
    int vanesLR_pos_state_{0};
    int vanes_pos_old_state_;
    int vanes_pos_state_{0};
    MhiPlatform* platform_;

    uint32_t pending_until_ms_{0};

    bool pending_power_valid_{false};
    ACPower pending_power_{power_off};

    bool pending_mode_valid_{false};
    climate::ClimateMode pending_mode_{climate::CLIMATE_MODE_OFF};

    bool pending_target_temperature_valid_{false};
    float pending_target_temperature_{0.0f};

    bool pending_fan_valid_{false};
    climate::ClimateFanMode pending_fan_mode_{climate::CLIMATE_FAN_AUTO};

    bool pending_swing_valid_{false};
    climate::ClimateSwingMode pending_swing_mode_{climate::CLIMATE_SWING_OFF};

    bool startup_power_synced_{false};

    bool pending_window_active_() const;
    void start_pending_window_();
    void clear_expired_pending_();
    void clear_all_pending_();

public:
    void set_temperature_offset_enabled(bool enabled) {
        this->temperature_offset_enabled_ = enabled;
    }

    void set_minimum_temperature(float temp) {
        this->minimum_temperature_ = temp;
    }
};

}
}