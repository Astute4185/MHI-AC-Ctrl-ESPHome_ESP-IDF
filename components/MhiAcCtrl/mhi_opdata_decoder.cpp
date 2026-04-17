#include "mhi_opdata_decoder.h"

namespace {

inline bool is_opdata_type(const uint8_t *mosi_frame) {
  return (mosi_frame[DB10] & 0x30) == 0x10;
}

inline bool high_group(const uint8_t *mosi_frame) {
  return (mosi_frame[DB6] & 0x80) != 0;
}

inline void publish_if_changed_u8(
    uint8_t value,
    uint8_t &old_value,
    CallbackInterface_Status *callback,
    ACStatus status,
    int publish_value) {
  if (value != old_value) {
    old_value = value;
    callback->cbiStatusFunction(status, publish_value);
  }
}

inline void publish_if_changed_u16(
    uint16_t value,
    uint16_t &old_value,
    CallbackInterface_Status *callback,
    ACStatus status,
    int publish_value) {
  if (value != old_value) {
    old_value = value;
    callback->cbiStatusFunction(status, publish_value);
  }
}

}  // namespace

void MhiOpDataDecoder::decode(
    const uint8_t *mosi_frame,
    MhiLoopRuntimeState &loop_state,
    MhiOpDataCacheState &cache,
    CallbackInterface_Status *callback) {
  if (callback == nullptr) {
    return;
  }

  const bool mosi_type_opdata = is_opdata_type(mosi_frame);

  switch (mosi_frame[DB9]) {
    case 0x94:
      if (high_group(mosi_frame) && mosi_type_opdata) {
        const uint16_t value = static_cast<uint16_t>((mosi_frame[DB12] << 8) + mosi_frame[DB11]);
        publish_if_changed_u16(value, cache.op_kwh_old, callback, opdata_kwh, value);
      }
      break;

    case 0x02:
      if (high_group(mosi_frame)) {
        if (mosi_type_opdata) {
          publish_if_changed_u8(
              mosi_frame[DB10],
              cache.op_mode_old,
              callback,
              opdata_mode,
              (mosi_frame[DB10] & 0x0f) << 2);
        } else {
          callback->cbiStatusFunction(erropdata_mode, (mosi_frame[DB10] & 0x0f) << 2);
        }
      }
      break;

    case 0x05:
      if (high_group(mosi_frame)) {
        if (mosi_frame[DB10] == 0x13) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_settemp_old,
              callback,
              opdata_tsetpoint,
              mosi_frame[DB11]);
        } else if (mosi_frame[DB10] == 0x33) {
          callback->cbiStatusFunction(erropdata_tsetpoint, mosi_frame[DB11]);
        }
      }
      break;

    case 0x81:
      if (high_group(mosi_frame)) {
        if ((mosi_frame[DB10] & 0x30) == 0x20) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_thi_r1_old,
              callback,
              opdata_thi_r1,
              mosi_frame[DB11]);
        } else {
          callback->cbiStatusFunction(erropdata_thi_r1, mosi_frame[DB11]);
        }
      } else {
        if (mosi_type_opdata) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_thi_r2_old,
              callback,
              opdata_thi_r2,
              mosi_frame[DB11]);
        } else {
          callback->cbiStatusFunction(erropdata_thi_r2, mosi_frame[DB11]);
        }
      }
      break;

    case 0x87:
      if (high_group(mosi_frame)) {
        if (mosi_type_opdata) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_thi_r3_old,
              callback,
              opdata_thi_r3,
              mosi_frame[DB11]);
        } else {
          callback->cbiStatusFunction(erropdata_thi_r3, mosi_frame[DB11]);
        }
      }
      break;

    case 0x80:
      if (high_group(mosi_frame)) {
        if ((mosi_frame[DB10] & 0x30) == 0x20) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_return_air_old,
              callback,
              opdata_return_air,
              mosi_frame[DB11]);
        } else {
          callback->cbiStatusFunction(erropdata_return_air, mosi_frame[DB11]);
        }
      } else {
        if (mosi_type_opdata) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_outdoor_old,
              callback,
              opdata_outdoor,
              mosi_frame[DB11]);
        } else {
          callback->cbiStatusFunction(erropdata_outdoor, mosi_frame[DB11]);
        }
      }
      break;

    case 0x1f:
      if (high_group(mosi_frame)) {
        if (mosi_type_opdata) {
          if (mosi_frame[DB10] != cache.op_iu_fanspeed_old) {
            cache.op_iu_fanspeed_old = mosi_frame[DB10];
            callback->cbiStatusFunction(opdata_iu_fanspeed, cache.op_iu_fanspeed_old & 0x0f);
          }
        } else {
          callback->cbiStatusFunction(erropdata_iu_fanspeed, mosi_frame[DB10] & 0x0f);
        }
      } else {
        if (mosi_type_opdata) {
          if (mosi_frame[DB10] != cache.op_ou_fanspeed_old) {
            cache.op_ou_fanspeed_old = mosi_frame[DB10];
            callback->cbiStatusFunction(opdata_ou_fanspeed, cache.op_ou_fanspeed_old & 0x0f);
          }
        } else {
          callback->cbiStatusFunction(erropdata_ou_fanspeed, mosi_frame[DB10] & 0x0f);
        }
      }
      break;

    case 0x1e:
      if (high_group(mosi_frame)) {
        if (mosi_type_opdata) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_total_iu_run_old,
              callback,
              opdata_total_iu_run,
              mosi_frame[DB11]);
        } else {
          callback->cbiStatusFunction(erropdata_total_iu_run, mosi_frame[DB11]);
        }
      } else {
        if (mosi_frame[DB10] == 0x11) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_total_comp_run_old,
              callback,
              opdata_total_comp_run,
              mosi_frame[DB11]);
        } else {
          callback->cbiStatusFunction(erropdata_total_comp_run, mosi_frame[DB11]);
        }
      }
      break;

    case 0x82:
      if (!high_group(mosi_frame)) {
        if (mosi_type_opdata) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_tho_r1_old,
              callback,
              opdata_tho_r1,
              mosi_frame[DB11]);
        } else {
          callback->cbiStatusFunction(erropdata_tho_r1, mosi_frame[DB11]);
        }
      }
      break;

    case 0x11:
      if (!high_group(mosi_frame)) {
        const uint16_t value = static_cast<uint16_t>((mosi_frame[DB10] << 8) | mosi_frame[DB11]);
        if (mosi_type_opdata) {
          publish_if_changed_u16(value, cache.op_comp_old, callback, opdata_comp, value & 0x0fff);
        } else {
          callback->cbiStatusFunction(erropdata_comp, value & 0x0fff);
        }
      }
      break;

    case 0x85:
      if (!high_group(mosi_frame)) {
        if (mosi_type_opdata) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_td_old,
              callback,
              opdata_td,
              mosi_frame[DB11]);
        } else {
          callback->cbiStatusFunction(erropdata_td, mosi_frame[DB11]);
        }
      }
      break;

    case 0x90:
      if (!high_group(mosi_frame)) {
        if (mosi_type_opdata) {
          publish_if_changed_u8(
              mosi_frame[DB11],
              cache.op_ct_old,
              callback,
              opdata_ct,
              mosi_frame[DB11]);
        } else {
          callback->cbiStatusFunction(erropdata_ct, mosi_frame[DB11]);
        }
      }
      break;

    case 0xb1:
      if (!high_group(mosi_frame) && mosi_type_opdata) {
        if (mosi_frame[DB11] != cache.op_tdsh_old) {
          cache.op_tdsh_old = mosi_frame[DB11];
          callback->cbiStatusFunction(opdata_tdsh, cache.op_tdsh_old / 2);
        }
      }
      break;

    case 0x7c:
      if (!high_group(mosi_frame) && mosi_type_opdata) {
        publish_if_changed_u8(
            mosi_frame[DB11],
            cache.op_protection_no_old,
            callback,
            opdata_protection_no,
            mosi_frame[DB11]);
      }
      break;

    case 0x0c:
      if (!high_group(mosi_frame) && mosi_type_opdata) {
        if (mosi_frame[DB10] != cache.op_defrost_old) {
          cache.op_defrost_old = mosi_frame[DB10];
          callback->cbiStatusFunction(opdata_defrost, cache.op_defrost_old & 0b1);
        }
      }
      break;

    case 0x13:
      if (!high_group(mosi_frame)) {
        const uint16_t value = static_cast<uint16_t>((mosi_frame[DB12] << 8) | mosi_frame[DB11]);
        if (mosi_type_opdata) {
          publish_if_changed_u16(value, cache.op_ou_eev1_old, callback, opdata_ou_eev1, value);
        } else {
          callback->cbiStatusFunction(erropdata_ou_eev1, value);
        }
      }
      break;

    case 0x45:
      if (high_group(mosi_frame)) {
        if (mosi_frame[DB10] == 0x11) {
          callback->cbiStatusFunction(erropdata_errorcode, mosi_frame[DB11]);
        } else if (mosi_frame[DB10] == 0x12) {
          loop_state.erropdata_count = static_cast<uint8_t>(mosi_frame[DB11] + 4);
        }
      }
      break;

    case 0x00:
    case 0xff:
      break;

    default:
      callback->cbiStatusFunction(opdata_unknown, (mosi_frame[DB10] << 8) | mosi_frame[DB9]);
      break;
  }
}
