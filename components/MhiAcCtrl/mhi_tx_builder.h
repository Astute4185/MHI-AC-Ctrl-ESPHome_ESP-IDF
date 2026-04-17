#pragma once

#include <cstdint>

#include "mhi_core_state.h"

class MhiTxBuilder {
public:
  static void prepare_next_frame(
      MhiLoopRuntimeState &loop_state,
      MhiTxWriteState &tx_state,
      uint8_t frame_size);
};
