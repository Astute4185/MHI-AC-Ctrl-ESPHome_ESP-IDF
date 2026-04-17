#pragma once

#include "MHI-AC-Ctrl-core.h"
#include "mhi_core_state.h"

class MhiOpDataDecoder {
 public:
  static void decode(
      const uint8_t *mosi_frame,
      MhiLoopRuntimeState &loop_state,
      MhiOpDataCacheState &cache,
      CallbackInterface_Status *callback);
};
