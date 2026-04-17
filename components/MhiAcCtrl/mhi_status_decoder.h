#pragma once

#include <cstdint>

#include "mhi_core_state.h"

class CallbackInterface_Status;

class MhiStatusDecoder {
public:
  static void decode(
      const uint8_t *mosi_frame,
      const uint8_t *miso_frame,
      uint8_t frame_size,
      MhiStatusCacheState &status_cache,
      MhiLoopRuntimeState &loop_state,
      CallbackInterface_Status *status_cb);
};
