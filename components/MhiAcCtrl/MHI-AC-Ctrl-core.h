#pragma once

#include <cstdint>

#include "mhi_transport.h"
#include "mhi_diagnostics.h"
#include "mhi_frame_layout.h"
#include "mhi_core_state.h"

// minimal time in ms used for Troom internal sensor changes for publishing to avoid jitter
static constexpr uint32_t MinTimeInternalTroomMs = 5000;

enum ErrMsg {
    err_msg_valid_frame = 0,
    err_msg_invalid_signature = -1,
    err_msg_invalid_checksum = -2,
    err_msg_timeout_SCK_low = -3,
    err_msg_timeout_SCK_high = -4
};

enum ACType {
    type_status = 0x40,
    type_opdata = 0x80,
    type_erropdata = 0xc0
};

enum ACStatus {
    status_power = type_status,
    status_mode,
    status_fan,
    status_vanes,
    status_vanesLR,
    status_3Dauto,
    status_troom,
    status_tsetpoint,
    status_errorcode,

    opdata_mode = type_opdata,
    opdata_kwh,
    opdata_tsetpoint,
    opdata_return_air,
    opdata_outdoor,
    opdata_tho_r1,
    opdata_iu_fanspeed,
    opdata_thi_r1,
    opdata_thi_r2,
    opdata_thi_r3,
    opdata_ou_fanspeed,
    opdata_total_iu_run,
    opdata_total_comp_run,
    opdata_comp,
    opdata_ct,
    opdata_td,
    opdata_tdsh,
    opdata_protection_no,
    opdata_defrost,
    opdata_ou_eev1,
    opdata_unknown,

    erropdata_mode = type_erropdata,
    erropdata_tsetpoint,
    erropdata_return_air,
    erropdata_thi_r1,
    erropdata_thi_r2,
    erropdata_thi_r3,
    erropdata_iu_fanspeed,
    erropdata_total_iu_run,
    erropdata_outdoor,
    erropdata_tho_r1,
    erropdata_comp,
    erropdata_td,
    erropdata_ct,
    erropdata_ou_fanspeed,
    erropdata_total_comp_run,
    erropdata_ou_eev1,
    erropdata_errorcode
};

enum ACPower {
    power_off = 0,
    power_on = 1
};

enum ACMode {
    mode_auto = 0b00000000,
    mode_dry = 0b00000100,
    mode_cool = 0b00001000,
    mode_fan  = 0b00001100,
    mode_heat = 0b00010000
};

enum ACVanes {
    vanes_unknown = 0,
    vanes_1 = 1,
    vanes_2 = 2,
    vanes_3 = 3,
    vanes_4 = 4,
    vanes_swing = 5
};

enum ACVanesLR {
    vanesLR_1 = 1,
    vanesLR_2 = 2,
    vanesLR_3 = 3,
    vanesLR_4 = 4,
    vanesLR_5 = 5,
    vanesLR_6 = 6,
    vanesLR_7 = 7,
    vanesLR_swing = 8
};

enum AC3Dauto {
    Dauto_off = 0b00000000,
    Dauto_on  = 0b00000100
};

class CallbackInterface_Status {
public:
    virtual ~CallbackInterface_Status() = default;
    virtual void cbiStatusFunction(ACStatus status, int value) = 0;
};

class MHI_AC_Ctrl_Core {
private:
    MhiStatusCacheState status_cache_{};
    MhiOpDataCacheState opdata_cache_{};

    MhiTxWriteState tx_write_state_{};
    uint8_t frameSize = 20;

    CallbackInterface_Status *m_cbiStatus = nullptr;

    esphome::mhi::MhiTransport *transport_ = nullptr;
    esphome::mhi::MhiTransportConfig transport_config_;
    MhiLoopRuntimeState loop_state_{};
    MhiDiagRuntimeState diag_state_{};
    esphome::mhi::MhiDiagReason last_diag_reason_ = esphome::mhi::MhiDiagReason::NONE;

public:
    void MHIAcCtrlStatus(CallbackInterface_Status *cb) {
        m_cbiStatus = cb;
    }

    void set_transport(esphome::mhi::MhiTransport *transport);
    void set_transport_config(const esphome::mhi::MhiTransportConfig &config);
    esphome::mhi::MhiDiagReason get_last_diag_reason() const { return this->last_diag_reason_; }

    void init();                         // initialization called once after boot
    void reset_old_values();             // resets the old variables so all status information is resent
    int loop(uint32_t max_time_ms);      // receive / transmit one frame
    void set_power(bool power);          // power on/off the AC
    void set_mode(ACMode mode);          // change AC mode
    void set_tsetpoint(uint32_t tsetpoint); // set the target temperature
    void set_fan(uint32_t fan);          // set the requested fan speed
    void set_vanes(uint32_t vanes);      // set the vanes horizontal position (or swing)
    void set_troom(uint8_t temperature); // set the room temperature used by AC
    void request_ErrOpData();            // request that the AC provides the error data
    float get_troom_offset();            // get troom offset
    void set_troom_offset(float offset); // set troom offset
    void set_frame_size(uint8_t framesize); // set frame size to 20 or 33
    void set_3Dauto(AC3Dauto Dauto);     // set the requested 3D auto mode
    void set_vanesLR(uint32_t vanesLR);  // set the vanes vertical position
};
