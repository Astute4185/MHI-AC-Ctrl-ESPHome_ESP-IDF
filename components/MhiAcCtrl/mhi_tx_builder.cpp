#include "mhi_tx_builder.h"

#include "mhi_frame_layout.h"

namespace {

// comment out the data you are not interested in, but leave at least one row
static constexpr uint8_t kMhiOpdata[][2] = {
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
static constexpr uint32_t kNoFramesPerOpDataCycle = 400;

}  // namespace

void MhiTxBuilder::prepare_next_frame(
    MhiLoopRuntimeState &loop_state,
    MhiTxWriteState &tx_state,
    uint8_t frame_size) {
  uint8_t *const miso_frame = loop_state.miso_frame;
  uint8_t &opdata_no = loop_state.opdata_no;
  uint8_t &erropdata_count = loop_state.erropdata_count;
  bool &doubleframe = loop_state.doubleframe;
  int &frame = loop_state.frame;

  const uint8_t opdata_count = static_cast<uint8_t>(sizeof(kMhiOpdata) / sizeof(kMhiOpdata[0]));

  miso_frame[0] = (frame_size == 33) ? 0xAA : 0xA9;

  doubleframe = !doubleframe;
  miso_frame[DB14] = static_cast<uint8_t>(doubleframe << 2);

  // Requesting all different opdata's is an opdata cycle. A cycle will take 20s.
  // With the current 20 different opdata's, every opdata request will take 1sec (interval).
  // If there are only 5 different opdata's defined, these 5 will be spread about the 20s cycle. The interval will increase.
  // requesting a new opdata will always start at a doubleframe start
  if ((frame > static_cast<int>(kNoFramesPerOpDataCycle / opdata_count)) && doubleframe) {
    frame = 1;
  }

  if (frame++ <= 2) {
    if (doubleframe) {
      if (erropdata_count == 0) {
        miso_frame[DB6] = kMhiOpdata[opdata_no][0];
        miso_frame[DB9] = kMhiOpdata[opdata_no][1];
        opdata_no = static_cast<uint8_t>((opdata_no + 1) % opdata_count);
      }
    }
  } else {
    miso_frame[DB6] = 0x80;
    miso_frame[DB9] = 0xFF;
  }

  if (doubleframe) {
    miso_frame[DB0] = 0x00;
    miso_frame[DB1] = 0x00;
    miso_frame[DB2] = 0x00;

    if (erropdata_count > 0) {
      miso_frame[DB6] = 0x80;
      miso_frame[DB9] = 0xFF;
      erropdata_count--;
    }

    miso_frame[DB0] = tx_state.new_power;
    tx_state.new_power = 0;

    miso_frame[DB0] |= tx_state.new_mode;
    tx_state.new_mode = 0;

    miso_frame[DB2] = tx_state.new_tsetpoint;
    tx_state.new_tsetpoint = 0;

    miso_frame[DB1] = tx_state.new_fan;
    tx_state.new_fan = 0;

    miso_frame[DB0] |= tx_state.new_vanes0;
    miso_frame[DB1] |= tx_state.new_vanes1;
    tx_state.new_vanes0 = 0;
    tx_state.new_vanes1 = 0;

    if (tx_state.request_erropdata) {
      miso_frame[DB6] = 0x80;
      miso_frame[DB9] = 0x45;
      tx_state.request_erropdata = false;
    }
  }

  miso_frame[DB3] = tx_state.new_troom;

  uint16_t checksum = mhi_calc_checksum(miso_frame);
  miso_frame[CBH] = static_cast<uint8_t>((checksum >> 8) & 0xFF);
  miso_frame[CBL] = static_cast<uint8_t>(checksum & 0xFF);

  if (frame_size == 33) {
    miso_frame[DB16] = 0;
    miso_frame[DB16] |= tx_state.new_vaneslr1;
    miso_frame[DB17] = 0;
    miso_frame[DB17] |= tx_state.new_vaneslr0;
    miso_frame[DB17] |= tx_state.new_3dauto;
    tx_state.new_3dauto = 0;
    tx_state.new_vaneslr0 = 0;
    tx_state.new_vaneslr1 = 0;

    checksum = mhi_calc_checksum_frame33(miso_frame);
    miso_frame[CBL2] = static_cast<uint8_t>(checksum & 0xFF);
  }
}
