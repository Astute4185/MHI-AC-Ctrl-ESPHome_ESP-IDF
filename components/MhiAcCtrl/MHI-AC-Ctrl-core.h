#pragma once
#include <cstdint>

#include "mhi_transport.h"

// comment out the data you are not interested in, but leave at least one row
static constexpr uint8_t opdata[][2] = {
    {0xc0, 0x02},  //  1 "MODE"
    {0xc0, 0x05},  //  2 "SET-TEMP" [°C]
    {0xc0, 0x80},  //  3 "RETURN-AIR" [°C]
    {0xc0, 0x81},  //  5 "THI-R1" [°C]
    {0x40, 0x81},  //  6 "THI-R2" [°C]
    {0xc0, 0x87},  //  7 "THI-R3" [°C]
    {0xc0, 0x1f},  //  8 "IU-FANSPEED"
    {0xc0, 0x1e},  // 12 "TOTAL-IU-RUN" [h]
    {0x40, 0x80},  // 21 "OUTDOOR" [°C]
    {0x40, 0x82},  // 22 "THO-R1" [°C]
    {0x40, 0x11},  // 24 "COMP" [Hz]
    {0x40, 0x85},  // 27 "TD" [°C]
    {0x40, 0x90},  // 29 "CT" [A]
    {0x40, 0xb1},  // 32 "TDSH" [°C]
    {0x40, 0x7c},  // 33 "PROTECTION-No"
    {0x40, 0x1f},  // 34 "OU-FANSPEED"
    {0x40, 0x0c},  // 36 "DEFROST"
    {0x40, 0x1e},  // 37 "TOTAL-COMP-RUN" [h]
    {0x40, 0x13},  // 38 "OU-EEV" [Puls]
    {0xc0, 0x94},  //    "energy-used" [kWh]
};

// number of frames used for an OpData request cycle; will be 20s (20 frames are 1s)
static constexpr uint32_t NoFramesPerOpDataCycle = 400;

// minimal time in ms used for Troom internal sensor changes for publishing to avoid jitter
static constexpr uint32_t MinTimeInternalTroomMs = 5000;

// Declare extern variables for the pins.
// This allows the pins to be set in the configuration file.
extern int SCK_PIN;
extern int MOSI_PIN;
extern int MISO_PIN;

// constants for the frame
#define SB0 0
#define SB1 SB0 + 1
#define SB2 SB0 + 2
#define DB0 SB2 + 1
#define DB1 SB2 + 2
#define DB2 SB2 + 3
#define DB3 SB2 + 4
#define DB4 SB2 + 5
#define DB6 SB2 + 7
#define DB9 SB2 + 10
#define DB10 SB2 + 11
#define DB11 SB2 + 12
#define DB12 SB2 + 13
#define DB14 SB2 + 15
#define CBH DB14 + 1
#define CBL DB14 + 2
#define DB15 CBL + 1
#define DB16 CBL + 2
#define DB17 CBL + 3
#define DB18 CBL + 4
#define DB19 CBL + 5
#define DB20 CBL + 6
#define DB21 CBL + 7
#define DB22 CBL + 8
#define DB23 CBL + 9
#define DB24 CBL + 10
#define DB25 CBL + 11
#define DB26 CBL + 12
#define CBL2 DB26 + 1

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
    // old status
    uint8_t status_power_old;
    uint8_t status_mode_old;
    uint8_t status_fan_old;
    uint8_t status_vanes_old;
    uint8_t status_troom_old;
    uint8_t status_tsetpoint_old;
    uint8_t status_errorcode_old;

    uint8_t status_vanesLR_old;
    uint8_t status_3Dauto_old;

    // old operating data
    uint16_t op_kwh_old;
    uint8_t op_mode_old;
    uint8_t op_settemp_old;
    uint8_t op_return_air_old;
    uint8_t op_iu_fanspeed_old;
    uint8_t op_thi_r1_old;
    uint8_t op_thi_r2_old;
    uint8_t op_thi_r3_old;
    uint8_t op_total_iu_run_old;
    uint8_t op_outdoor_old;
    uint8_t op_tho_r1_old;
    uint8_t op_total_comp_run_old;
    uint8_t op_ct_old;
    uint8_t op_tdsh_old;
    uint8_t op_protection_no_old;
    uint8_t op_ou_fanspeed_old;
    uint8_t op_defrost_old;
    uint16_t op_comp_old;
    uint8_t op_td_old;
    uint16_t op_ou_eev1_old;

    // for writing to AC
    uint8_t new_Power = 0;
    uint8_t new_Mode = 0;
    uint8_t new_Tsetpoint = 0;
    uint8_t new_Fan = 0;
    uint8_t new_Vanes0 = 0;
    uint8_t new_Vanes1 = 0;
    bool request_erropData = false;
    uint8_t new_Troom = 0xff; // 0xff in DB3 indicates use of the internal room temperature sensor
    float Troom_offset = 0.0f;

    uint8_t new_VanesLR0 = 0;
    uint8_t new_VanesLR1 = 0;
    uint8_t new_3Dauto = 0;
    uint8_t frameSize = 20;

    CallbackInterface_Status *m_cbiStatus = nullptr;
    
    esphome::mhi::MhiTransport *transport_ = nullptr;
    esphome::mhi::MhiTransportConfig transport_config_;

public:
    void MHIAcCtrlStatus(CallbackInterface_Status *cb) {
        m_cbiStatus = cb;
    }

    void set_transport(esphome::mhi::MhiTransport *transport);
    void set_transport_config(const esphome::mhi::MhiTransportConfig &config);

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
