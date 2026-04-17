#include "mhi_status_decoder.h"

#include "MHI-AC-Ctrl-core.h"
#include "mhi_frame_layout.h"
#include "mhi_time.h"

namespace {

void publish_status(CallbackInterface_Status *status_cb, ACStatus status, int value) {
  if (status_cb != nullptr) {
    status_cb->cbiStatusFunction(status, value);
  }
}

void decode_extended_status(
    const uint8_t *mosi_frame,
    uint8_t frame_size,
    MhiStatusCacheState &status_cache,
    CallbackInterface_Status *status_cb) {
  if (frame_size != 33U) {
    return;
  }

  const uint8_t vaneslr_tmp = static_cast<uint8_t>((mosi_frame[DB16] & 0x07U) + ((mosi_frame[DB17] & 0x01U) << 4));
  if (vaneslr_tmp != status_cache.vaneslr_old) {
    if ((vaneslr_tmp & 0x10U) != 0U) {
      publish_status(status_cb, status_vanesLR, vanesLR_swing);
    } else {
      publish_status(status_cb, status_vanesLR, (vaneslr_tmp & 0x07U) + 1);
    }
    status_cache.vaneslr_old = vaneslr_tmp;
  }

  const uint8_t auto3d = static_cast<uint8_t>(mosi_frame[DB17] & 0x04U);
  if (auto3d != status_cache.auto3d_old) {
    status_cache.auto3d_old = auto3d;
    publish_status(status_cb, status_3Dauto, status_cache.auto3d_old);
  }
}

void decode_primary_status(
    const uint8_t *mosi_frame,
    MhiStatusCacheState &status_cache,
    CallbackInterface_Status *status_cb) {
  const uint8_t mode = static_cast<uint8_t>(mosi_frame[DB0] & 0x1cU);
  if (mode != status_cache.mode_old) {
    status_cache.mode_old = mode;
    publish_status(status_cb, status_mode, status_cache.mode_old);
  }

  const uint8_t power = static_cast<uint8_t>(mosi_frame[DB0] & 0x01U);
  if (power != status_cache.power_old) {
    status_cache.power_old = power;
    publish_status(status_cb, status_power, status_cache.power_old);
  }

  const uint8_t fan = static_cast<uint8_t>(mosi_frame[DB1] & 0x07U);
  if (fan != status_cache.fan_old) {
    status_cache.fan_old = fan;
    publish_status(status_cb, status_fan, status_cache.fan_old);
  }

  const uint8_t vanes = static_cast<uint8_t>((mosi_frame[DB0] & 0xc0U) + ((mosi_frame[DB1] & 0xB0U) >> 4));
  if (vanes != status_cache.vanes_old) {
    if ((vanes & 0x40U) != 0U) {
      publish_status(status_cb, status_vanes, vanes_swing);
    } else {
      publish_status(status_cb, status_vanes, (vanes & 0x03U) + 1);
    }
    status_cache.vanes_old = vanes;
  }

  if (mosi_frame[DB2] != status_cache.tsetpoint_old) {
    status_cache.tsetpoint_old = mosi_frame[DB2];
    publish_status(status_cb, status_tsetpoint, status_cache.tsetpoint_old);
  }

  if (mosi_frame[DB4] != status_cache.errorcode_old) {
    status_cache.errorcode_old = mosi_frame[DB4];
    publish_status(status_cb, status_errorcode, status_cache.errorcode_old);
  }
}

void decode_troom_status(
    const uint8_t *mosi_frame,
    const uint8_t *miso_frame,
    MhiStatusCacheState &status_cache,
    MhiLoopRuntimeState &loop_state,
    CallbackInterface_Status *status_cb) {
  if (mosi_frame[DB3] == status_cache.troom_old) {
    return;
  }

  if (miso_frame[DB3] != 0xffU) {
    status_cache.troom_old = mosi_frame[DB3];
    publish_status(status_cb, status_troom, status_cache.troom_old);
    loop_state.last_troom_internal_ms = 0U;
    return;
  }

  const uint32_t now_ms = esphome::mhi::mhi_now_ms();
  if ((now_ms - loop_state.last_troom_internal_ms) > MinTimeInternalTroomMs) {
    loop_state.last_troom_internal_ms = now_ms;
    status_cache.troom_old = mosi_frame[DB3];
    publish_status(status_cb, status_troom, status_cache.troom_old);
  }
}

}  // namespace

void MhiStatusDecoder::decode(
    const uint8_t *mosi_frame,
    const uint8_t *miso_frame,
    uint8_t frame_size,
    MhiStatusCacheState &status_cache,
    MhiLoopRuntimeState &loop_state,
    CallbackInterface_Status *status_cb) {
  decode_extended_status(mosi_frame, frame_size, status_cache, status_cb);
  decode_primary_status(mosi_frame, status_cache, status_cb);
  decode_troom_status(mosi_frame, miso_frame, status_cache, loop_state, status_cb);
}
