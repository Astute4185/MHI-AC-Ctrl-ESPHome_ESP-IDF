#include "MHI-AC-Ctrl-core.h"

void MHI_AC_Ctrl_Core::reset_old_values() {
  this->status_cache_ = MhiStatusCacheState{};
  this->opdata_cache_ = MhiOpDataCacheState{};
}

void MHI_AC_Ctrl_Core::set_transport(esphome::mhi::MhiTransport *transport) {
  this->transport_ = transport;
}

void MHI_AC_Ctrl_Core::set_transport_config(
    const esphome::mhi::MhiTransportConfig &config) {
  this->transport_config_ = config;
}

void MHI_AC_Ctrl_Core::init() {
  if (this->transport_ != nullptr) {
    this->transport_->setup(this->transport_config_);
  }

  this->loop_state_ = MhiLoopRuntimeState{};
  this->diag_state_ = MhiDiagRuntimeState{};
  this->tx_write_state_ = MhiTxWriteState{};
  this->last_diag_reason_ = esphome::mhi::MhiDiagReason::NONE;

  this->reset_old_values();
}

void MHI_AC_Ctrl_Core::set_power(bool power) {
  this->tx_write_state_.new_power = 0b00000010 | static_cast<uint8_t>(power);
}

void MHI_AC_Ctrl_Core::set_mode(ACMode mode) {
  this->tx_write_state_.new_mode = 0b00100000 | static_cast<uint8_t>(mode);
}

void MHI_AC_Ctrl_Core::set_tsetpoint(uint32_t tsetpoint) {
  this->tx_write_state_.new_tsetpoint =
      0b10000000 | static_cast<uint8_t>(tsetpoint);
}

void MHI_AC_Ctrl_Core::set_fan(uint32_t fan) {
  this->tx_write_state_.new_fan = 0b00001000 | static_cast<uint8_t>(fan);
}

void MHI_AC_Ctrl_Core::set_3Dauto(AC3Dauto Dauto) {
  this->tx_write_state_.new_3dauto =
      0b00001010 | static_cast<uint8_t>(Dauto);
}

void MHI_AC_Ctrl_Core::set_vanes(uint32_t vanes) {
  if (vanes == vanes_swing) {
    this->tx_write_state_.new_vanes0 = 0b11000000;
    this->tx_write_state_.new_vanes1 = 0;
  } else {
    this->tx_write_state_.new_vanes0 = 0b10000000;
    this->tx_write_state_.new_vanes1 =
        0b10000000 | (static_cast<uint8_t>(vanes - 1) << 4);
  }
}

void MHI_AC_Ctrl_Core::set_vanesLR(uint32_t vanesLR) {
  if (vanesLR == vanesLR_swing) {
    this->tx_write_state_.new_vaneslr0 = 0b00001011;
    this->tx_write_state_.new_vaneslr1 = 0;
  } else {
    this->tx_write_state_.new_vaneslr0 = 0b00001010;
    this->tx_write_state_.new_vaneslr1 =
        0b00010000 | static_cast<uint8_t>(vanesLR - 1);
  }
}

void MHI_AC_Ctrl_Core::request_ErrOpData() {
  this->tx_write_state_.request_erropdata = true;
}

void MHI_AC_Ctrl_Core::set_troom(uint8_t troom) {
  this->tx_write_state_.new_troom = troom;
}

float MHI_AC_Ctrl_Core::get_troom_offset() {
  return this->tx_write_state_.troom_offset;
}

void MHI_AC_Ctrl_Core::set_troom_offset(float offset) {
  this->tx_write_state_.troom_offset = offset;
}

void MHI_AC_Ctrl_Core::set_frame_size(uint8_t framesize) {
  if (framesize == 20 || framesize == 33) {
    this->frameSize = framesize;
  }
}