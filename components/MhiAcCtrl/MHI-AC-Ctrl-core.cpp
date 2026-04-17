// MHI-AC-Ctrl-core
// implements the core loop (read & write SPI)

#include "mhi_time.h"
#include "MHI-AC-Ctrl-core.h"
#include "mhi_tx_builder.h"
#include "mhi_rx_validator.h"
#include "mhi_status_decoder.h"
#include "mhi_opdata_decoder.h"

int MHI_AC_Ctrl_Core::loop(uint32_t max_time_ms) {
  this->last_diag_reason_ = esphome::mhi::MhiDiagReason::NONE;

  MhiLoopRuntimeState &loop_state = this->loop_state_;
  uint8_t *const MOSI_frame = loop_state.mosi_frame;
  uint8_t *const MISO_frame = loop_state.miso_frame;

  loop_state.call_counter++;

  MhiTxBuilder::prepare_next_frame(
      loop_state,
      this->tx_write_state_,
      this->frameSize);

  const MhiRxValidationResult rx_result = MhiRxValidator::exchange_and_validate(
      this->transport_,
      MISO_frame,
      MOSI_frame,
      this->frameSize,
      max_time_ms,
      this->diag_state_,
      this->last_diag_reason_);

  if (rx_result.status < 0) {
    return rx_result.status;
  }

  if (rx_result.new_data_packet_received) {
    MhiStatusDecoder::decode(
        MOSI_frame,
        MISO_frame,
        this->frameSize,
        this->status_cache_,
        loop_state,
        this->m_cbiStatus);

    MhiOpDataDecoder::decode(
        MOSI_frame,
        loop_state,
        this->opdata_cache_,
        this->m_cbiStatus);
  }

  return static_cast<int>(loop_state.call_counter);
}