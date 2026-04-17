#pragma once

#include <cstdint>

#include "mhi_core_state.h"
#include "mhi_transport.h"
#include "mhi_diagnostics.h"

struct MhiRxValidationResult {
  int status{0};
  bool new_data_packet_received{false};
};

class MhiRxValidator {
public:
  static MhiRxValidationResult exchange_and_validate(
      esphome::mhi::MhiTransport *transport,
      uint8_t *miso_frame,
      uint8_t *mosi_frame,
      uint8_t frame_size,
      uint32_t max_time_ms,
      MhiDiagRuntimeState &diag_state,
      esphome::mhi::MhiDiagReason &last_diag_reason);
};
