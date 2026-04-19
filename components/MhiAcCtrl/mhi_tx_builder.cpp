#include "mhi_tx_builder.h"

#include <algorithm>

#include "mhi_frame_layout.h"

namespace {

struct MhiOpdataRequest {
  uint8_t db6;
  uint8_t db9;
  uint32_t mask;
};

static constexpr MhiOpdataRequest kMhiOpdata[] = {
    {0xc0, 0x02, MHI_OPDATA_REQ_MODE},            // MODE
    {0xc0, 0x05, MHI_OPDATA_REQ_TSETPOINT},       // SET-TEMP
    {0xc0, 0x80, MHI_OPDATA_REQ_RETURN_AIR},      // RETURN-AIR
    {0xc0, 0x81, MHI_OPDATA_REQ_THI_R1},          // THI-R1
    {0x40, 0x81, MHI_OPDATA_REQ_THI_R2},          // THI-R2
    {0xc0, 0x87, MHI_OPDATA_REQ_THI_R3},          // THI-R3
    {0xc0, 0x1f, MHI_OPDATA_REQ_IU_FANSPEED},     // IU-FANSPEED
    {0xc0, 0x1e, MHI_OPDATA_REQ_TOTAL_IU_RUN},    // TOTAL-IU-RUN
    {0x40, 0x80, MHI_OPDATA_REQ_OUTDOOR},         // OUTDOOR
    {0x40, 0x82, MHI_OPDATA_REQ_THO_R1},          // THO-R1
    {0x40, 0x11, MHI_OPDATA_REQ_COMP},            // COMP
    {0x40, 0x85, MHI_OPDATA_REQ_TD},              // TD
    {0x40, 0x90, MHI_OPDATA_REQ_CT},              // CT
    {0x40, 0xb1, MHI_OPDATA_REQ_TDSH},            // TDSH
    {0x40, 0x7c, MHI_OPDATA_REQ_PROTECTION_NO},   // PROTECTION-No
    {0x40, 0x1f, MHI_OPDATA_REQ_OU_FANSPEED},     // OU-FANSPEED
    {0x40, 0x0c, MHI_OPDATA_REQ_DEFROST},         // DEFROST
    {0x40, 0x1e, MHI_OPDATA_REQ_TOTAL_COMP_RUN},  // TOTAL-COMP-RUN
    {0x40, 0x13, MHI_OPDATA_REQ_OU_EEV1},         // OU-EEV
    {0xc0, 0x94, MHI_OPDATA_REQ_KWH},             // energy-used
};

// number of frames used for an OpData request cycle; 20 frames are ~1s.
static constexpr uint32_t kNoFramesPerOpDataCycle = 400;
// Keep sparse configurations from becoming excessively stale.
static constexpr uint8_t kMinEffectiveOpdataCount = 5;

uint32_t normalize_opdata_mask(uint32_t mask) {
  return mask == 0 ? kMhiDefaultOpdataMask : mask;
}

uint8_t get_enabled_opdata_count(uint32_t mask) {
  uint8_t count = 0;
  for (const auto &entry : kMhiOpdata) {
    if ((mask & entry.mask) != 0) {
      count++;
    }
  }
  return count;
}

const MhiOpdataRequest &get_enabled_opdata(uint32_t mask, uint8_t enabled_index) {
  uint8_t current = 0;
  for (const auto &entry : kMhiOpdata) {
    if ((mask & entry.mask) != 0) {
      if (current == enabled_index) {
        return entry;
      }
      current++;
    }
  }
  return kMhiOpdata[0];
}

}  // namespace

void MhiTxBuilder::prepare_next_frame(
    MhiLoopRuntimeState &loop_state,
    MhiTxWriteState &tx_state,
    uint8_t frame_size,
    uint32_t enabled_opdata_mask) {
  uint8_t *const miso_frame = loop_state.miso_frame;
  uint8_t &opdata_no = loop_state.opdata_no;
  uint8_t &erropdata_count = loop_state.erropdata_count;
  bool &doubleframe = loop_state.doubleframe;
  int &frame = loop_state.frame;

  const uint32_t normalized_mask = normalize_opdata_mask(enabled_opdata_mask);
  const uint8_t enabled_opdata_count = get_enabled_opdata_count(normalized_mask);
  const uint8_t effective_cycle_count =
      std::max<uint8_t>(enabled_opdata_count == 0 ? 1 : enabled_opdata_count, kMinEffectiveOpdataCount);

  miso_frame[0] = (frame_size == 33) ? 0xAA : 0xA9;

  doubleframe = !doubleframe;
  miso_frame[DB14] = static_cast<uint8_t>(doubleframe << 2);

  // Fewer enabled opdata items reduce request traffic, but keep a floor so
  // sparse configs do not become excessively stale.
  if ((frame > static_cast<int>(kNoFramesPerOpDataCycle / effective_cycle_count)) && doubleframe) {
    frame = 1;
  }

  if (frame++ <= 2) {
    if (doubleframe) {
      if (erropdata_count == 0) {
        const uint8_t request_index =
            (enabled_opdata_count == 0) ? 0 : static_cast<uint8_t>(opdata_no % enabled_opdata_count);
        const auto &request = get_enabled_opdata(normalized_mask, request_index);
        miso_frame[DB6] = request.db6;
        miso_frame[DB9] = request.db9;
        opdata_no = static_cast<uint8_t>((request_index + 1) % (enabled_opdata_count == 0 ? 1 : enabled_opdata_count));
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