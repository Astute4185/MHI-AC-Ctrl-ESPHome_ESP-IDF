// MHI-AC-Ctrol-core
// implements the core functions (read & write SPI)

#include "mhi_time.h"
#include "MHI-AC-Ctrl-core.h"

#include <cstdio>

uint16_t calc_checksum(uint8_t *frame) {
  uint16_t checksum = 0;
  for (int i = 0; i < CBH; i++) {
    checksum += frame[i];
  }
  return checksum;
}

uint16_t calc_checksumFrame33(uint8_t *frame) {
  uint16_t checksum = 0;
  for (int i = 0; i < CBL2; i++) {
    checksum += frame[i];
  }
  return checksum;
}


namespace {
static const char *DIAG_TAG = "mhi.diag";
constexpr uint32_t kDiagSummaryIntervalMs = 10000U;
constexpr uint32_t kDiagSampleIntervalMs = 15000U;

struct DiagCounters {
  uint32_t ok_frames = 0;
  uint32_t ok_20 = 0;
  uint32_t ok_33 = 0;
  uint32_t short_capture = 0;
  uint32_t invalid_signature = 0;
  uint32_t base_checksum_fail = 0;
  uint32_t extended_checksum_fail = 0;
  uint32_t timeout_low = 0;
  uint32_t timeout_high = 0;
  uint32_t timeout_other = 0;
  uint32_t header_6c = 0;
  uint32_t header_6d = 0;
  uint32_t header_other = 0;
  uint32_t extension_probe_attempted = 0;
  uint32_t extension_start_seen = 0;
};

enum DiagReason : uint8_t {
  DIAG_REASON_SHORT_CAPTURE = 0,
  DIAG_REASON_INVALID_SIGNATURE = 1,
  DIAG_REASON_BASE_CHECKSUM = 2,
  DIAG_REASON_EXTENDED_CHECKSUM = 3,
  DIAG_REASON_TIMEOUT_LOW = 4,
  DIAG_REASON_TIMEOUT_HIGH = 5,
  DIAG_REASON_TIMEOUT_OTHER = 6,
  DIAG_REASON_COUNT = 7
};

void format_frame_hex(const uint8_t *frame, std::size_t len, char *out, std::size_t out_len) {
  if (out_len == 0) {
    return;
  }
  out[0] = '\0';
  std::size_t pos = 0;
  for (std::size_t i = 0; i < len && pos + 4 < out_len; i++) {
    const int written = std::snprintf(out + pos, out_len - pos, "%02X%s", frame[i], (i + 1 < len) ? " " : "");
    if (written <= 0) {
      break;
    }
    pos += static_cast<std::size_t>(written);
  }
  out[out_len - 1] = '\0';
}

bool should_log_reason(DiagReason reason, uint32_t now_ms, uint32_t *last_log_ms, uint32_t count) {
  if (count <= 1U) {
    last_log_ms[reason] = now_ms;
    return true;
  }
  if ((now_ms - last_log_ms[reason]) >= kDiagSampleIntervalMs) {
    last_log_ms[reason] = now_ms;
    return true;
  }
  return false;
}

void maybe_log_summary(const DiagCounters &counters, uint32_t now_ms, uint32_t &last_summary_ms) {
  if ((now_ms - last_summary_ms) < kDiagSummaryIntervalMs) {
    return;
  }
  last_summary_ms = now_ms;
  ESP_LOGI(
      DIAG_TAG,
      "summary ok=%u ok20=%u ok33=%u short=%u sig=%u basechk=%u extchk=%u t_low=%u t_high=%u t_other=%u hdr6c=%u hdr6d=%u hdrother=%u ext_probe=%u ext_seen=%u",
      counters.ok_frames,
      counters.ok_20,
      counters.ok_33,
      counters.short_capture,
      counters.invalid_signature,
      counters.base_checksum_fail,
      counters.extended_checksum_fail,
      counters.timeout_low,
      counters.timeout_high,
      counters.timeout_other,
      counters.header_6c,
      counters.header_6d,
      counters.header_other,
      counters.extension_probe_attempted,
      counters.extension_start_seen);
}

}  // namespace

void MHI_AC_Ctrl_Core::reset_old_values() {  // used e.g. when MQTT connection to broker is lost, to re-output data
  // old status
  status_power_old = 0xff;
  status_mode_old = 0xff;
  status_fan_old = 0xff;
  status_vanes_old = 0xff;
  status_troom_old = 0xfe;
  status_tsetpoint_old = 0x00;
  status_errorcode_old = 0xff;
  status_vanesLR_old = 0xff;
  status_3Dauto_old = 0xff;

  // old operating data
  op_kwh_old = 0xffff;
  op_mode_old = 0xff;
  op_settemp_old = 0xff;
  op_return_air_old = 0xff;
  op_iu_fanspeed_old = 0xff;
  op_thi_r1_old = 0x00;
  op_thi_r2_old = 0x00;
  op_thi_r3_old = 0x00;
  op_total_iu_run_old = 0;
  op_outdoor_old = 0xff;
  op_tho_r1_old = 0x00;
  op_total_comp_run_old = 0;
  op_ct_old = 0xff;
  op_tdsh_old = 0xff;
  op_protection_no_old = 0xff;
  op_ou_fanspeed_old = 0xff;
  op_defrost_old = 0x00;
  op_comp_old = 0xffff;
  op_td_old = 0x00;
  op_ou_eev1_old = 0xffff;
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
  MHI_AC_Ctrl_Core::reset_old_values();
}

void MHI_AC_Ctrl_Core::set_power(bool power) {
  new_Power = 0b10 | static_cast<uint8_t>(power);
}

void MHI_AC_Ctrl_Core::set_mode(ACMode mode) {
  new_Mode = 0b00100000 | static_cast<uint8_t>(mode);
}

void MHI_AC_Ctrl_Core::set_tsetpoint(uint32_t tsetpoint) {
  new_Tsetpoint = 0b10000000 | static_cast<uint8_t>(tsetpoint);
}

void MHI_AC_Ctrl_Core::set_fan(uint32_t fan) {
  new_Fan = 0b00001000 | static_cast<uint8_t>(fan);
}

void MHI_AC_Ctrl_Core::set_3Dauto(AC3Dauto Dauto) {
  new_3Dauto = 0b00001010 | static_cast<uint8_t>(Dauto);
}

void MHI_AC_Ctrl_Core::set_vanes(uint32_t vanes) {
  if (vanes == vanes_swing) {
    new_Vanes0 = 0b11000000;  // enable swing
  } else {
    new_Vanes0 = 0b10000000;  // disable swing
    new_Vanes1 = 0b10000000 | (static_cast<uint8_t>(vanes - 1) << 4);
  }
}

void MHI_AC_Ctrl_Core::set_vanesLR(uint32_t vanesLR) {
  if (vanesLR == vanesLR_swing) {
    new_VanesLR0 = 0b00001011;  // enable swing
  } else {
    new_VanesLR0 = 0b00001010;  // disable swing
    new_VanesLR1 = 0b00010000 | static_cast<uint8_t>(vanesLR - 1);
  }
}

void MHI_AC_Ctrl_Core::request_ErrOpData() {
  request_erropData = true;
}

void MHI_AC_Ctrl_Core::set_troom(uint8_t troom) {
  new_Troom = troom;
}

float MHI_AC_Ctrl_Core::get_troom_offset() {
  return Troom_offset;
}

void MHI_AC_Ctrl_Core::set_troom_offset(float offset) {
  Troom_offset = offset;
}

void MHI_AC_Ctrl_Core::set_frame_size(uint8_t framesize) {
  if (framesize == 20 || framesize == 33) {
    frameSize = framesize;
  }
}

int MHI_AC_Ctrl_Core::loop(uint32_t max_time_ms) {
  const uint8_t opdataCnt = static_cast<uint8_t>(sizeof(opdata) / sizeof(opdata[0]));
  static uint8_t opdataNo = 0;
  bool new_datapacket_received = false;
  static uint8_t erropdataCnt = 0;
  static bool doubleframe = false;
  static int frame = 1;
  static uint8_t MOSI_frame[33];
  //                            sb0   sb1   sb2   db0   db1   db2   db3   db4   db5   db6   db7   db8   db9  db10  db11  db12  db13  db14  chkH  chkL  db15  db16  db17  db18  db19  db20  db21  db22  db23  db24  db25  db26  chk2L
  static uint8_t MISO_frame[] = {0xA9, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x22};

  static uint32_t call_counter = 0;
  static uint32_t lastTroomInternalMillis = 0;

  if (frameSize == 33) {
    MISO_frame[0] = 0xAA;
  }

  call_counter++;

  // build the next MISO frame
  doubleframe = !doubleframe;
  MISO_frame[DB14] = static_cast<uint8_t>(doubleframe << 2);

  // Requesting all different opdata's is an opdata cycle. A cycle will take 20s.
  // With the current 20 different opdata's, every opdata request will take 1sec (interval).
  // If there are only 5 different opdata's defined, these 5 will be spread about the 20s cycle. The interval will increase.
  // requesting a new opdata will always start at a doubleframe start
  if ((frame > static_cast<int>(NoFramesPerOpDataCycle / opdataCnt)) && doubleframe) {
    frame = 1;
  }

  if (frame++ <= 2) {
    if (doubleframe) {
      if (erropdataCnt == 0) {
        MISO_frame[DB6] = opdata[opdataNo][0];
        MISO_frame[DB9] = opdata[opdataNo][1];
        opdataNo = static_cast<uint8_t>((opdataNo + 1) % opdataCnt);
      }
    }
  } else {
    MISO_frame[DB6] = 0x80;
    MISO_frame[DB9] = 0xff;
  }

  if (doubleframe) {
    MISO_frame[DB0] = 0x00;
    MISO_frame[DB1] = 0x00;
    MISO_frame[DB2] = 0x00;

    if (erropdataCnt > 0) {
      MISO_frame[DB6] = 0x80;
      MISO_frame[DB9] = 0xff;
      erropdataCnt--;
    }

    // set Power, Mode, Tsetpoint, Fan, Vanes
    MISO_frame[DB0] = new_Power;
    new_Power = 0;

    MISO_frame[DB0] |= new_Mode;
    new_Mode = 0;

    MISO_frame[DB2] = new_Tsetpoint;
    new_Tsetpoint = 0;

    MISO_frame[DB1] = new_Fan;
    new_Fan = 0;

    MISO_frame[DB0] |= new_Vanes0;
    MISO_frame[DB1] |= new_Vanes1;
    new_Vanes0 = 0;
    new_Vanes1 = 0;

    if (request_erropData) {
      MISO_frame[DB6] = 0x80;
      MISO_frame[DB9] = 0x45;
      request_erropData = false;
    }
  }

  MISO_frame[DB3] = new_Troom;

  uint16_t checksum = calc_checksum(MISO_frame);
  MISO_frame[CBH] = static_cast<uint8_t>((checksum >> 8) & 0xFF);
  MISO_frame[CBL] = static_cast<uint8_t>(checksum & 0xFF);

  if (frameSize == 33) {
    MISO_frame[DB16] = 0;
    MISO_frame[DB16] |= new_VanesLR1;
    MISO_frame[DB17] = 0;
    MISO_frame[DB17] |= new_VanesLR0;
    MISO_frame[DB17] |= new_3Dauto;
    new_3Dauto = 0;
    new_VanesLR0 = 0;
    new_VanesLR1 = 0;

    checksum = calc_checksumFrame33(MISO_frame);
    MISO_frame[CBL2] = static_cast<uint8_t>(checksum & 0xFF);
  }

  static DiagCounters diag_counters{};
  static uint32_t diag_last_summary_ms = 0;
  static uint32_t diag_last_sample_ms[DIAG_REASON_COUNT] = {0};

  if (this->transport_ == nullptr) {
    return err_msg_timeout_SCK_low;
  }

  const esphome::mhi::MhiFrameExchangeResult transport_result = this->transport_->exchange_frame(
      MISO_frame,
      MOSI_frame,
      sizeof(MOSI_frame),
      max_time_ms);

  new_datapacket_received = transport_result.new_data_packet_received;

  if (transport_result.extension_probe_attempted) {
    diag_counters.extension_probe_attempted++;
  }
  if (transport_result.extension_start_seen) {
    diag_counters.extension_start_seen++;
  }
  if (transport_result.header_byte == 0x6C) {
    diag_counters.header_6c++;
  } else if (transport_result.header_byte == 0x6D) {
    diag_counters.header_6d++;
  } else if (transport_result.bytes_received > 0U) {
    diag_counters.header_other++;
  }

  const uint32_t now_ms = esphome::mhi::mhi_now_ms();

  if (transport_result.status < 0) {
    if (transport_result.status == err_msg_timeout_SCK_low) {
      diag_counters.timeout_low++;
      if (should_log_reason(DIAG_REASON_TIMEOUT_LOW, now_ms, diag_last_sample_ms, diag_counters.timeout_low)) {
        ESP_LOGW(
            DIAG_TAG,
            "timeout_low count=%u frame_hint=%u bytes=%u hdr=%02X ext_probe=%s ext_seen=%s",
            diag_counters.timeout_low,
            frameSize,
            static_cast<unsigned>(transport_result.bytes_received),
            transport_result.header_byte,
            transport_result.extension_probe_attempted ? "true" : "false",
            transport_result.extension_start_seen ? "true" : "false");
      }
    } else if (transport_result.status == err_msg_timeout_SCK_high) {
      diag_counters.timeout_high++;
      if (should_log_reason(DIAG_REASON_TIMEOUT_HIGH, now_ms, diag_last_sample_ms, diag_counters.timeout_high)) {
        ESP_LOGW(
            DIAG_TAG,
            "timeout_high count=%u frame_hint=%u bytes=%u hdr=%02X ext_probe=%s ext_seen=%s",
            diag_counters.timeout_high,
            frameSize,
            static_cast<unsigned>(transport_result.bytes_received),
            transport_result.header_byte,
            transport_result.extension_probe_attempted ? "true" : "false",
            transport_result.extension_start_seen ? "true" : "false");
      }
    } else {
      diag_counters.timeout_other++;
      if (should_log_reason(DIAG_REASON_TIMEOUT_OTHER, now_ms, diag_last_sample_ms, diag_counters.timeout_other)) {
        ESP_LOGW(
            DIAG_TAG,
            "timeout_other code=%d count=%u frame_hint=%u bytes=%u hdr=%02X",
            transport_result.status,
            diag_counters.timeout_other,
            frameSize,
            static_cast<unsigned>(transport_result.bytes_received),
            transport_result.header_byte);
      }
    }
    maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);
    return transport_result.status;
  }

  if (frameSize == 33 && transport_result.bytes_received < 33U) {
    diag_counters.short_capture++;
    if (should_log_reason(DIAG_REASON_SHORT_CAPTURE, now_ms, diag_last_sample_ms, diag_counters.short_capture)) {
      char frame_hex[128];
      format_frame_hex(MOSI_frame, transport_result.bytes_received, frame_hex, sizeof(frame_hex));
      ESP_LOGW(
          DIAG_TAG,
          "short_capture count=%u expected=33 got=%u hdr=%02X ext_probe=%s ext_seen=%s rx=%s",
          diag_counters.short_capture,
          static_cast<unsigned>(transport_result.bytes_received),
          transport_result.header_byte,
          transport_result.extension_probe_attempted ? "true" : "false",
          transport_result.extension_start_seen ? "true" : "false",
          frame_hex);
    }
    maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);
    return err_msg_invalid_checksum;
  }

  checksum = calc_checksum(MOSI_frame);
  if (((MOSI_frame[SB0] & 0xfe) != 0x6c) | (MOSI_frame[SB1] != 0x80) | (MOSI_frame[SB2] != 0x04)) {
    diag_counters.invalid_signature++;
    if (should_log_reason(DIAG_REASON_INVALID_SIGNATURE, now_ms, diag_last_sample_ms, diag_counters.invalid_signature)) {
      char frame_hex[128];
      format_frame_hex(MOSI_frame, transport_result.bytes_received, frame_hex, sizeof(frame_hex));
      ESP_LOGW(
          DIAG_TAG,
          "invalid_signature count=%u bytes=%u hdr=%02X rx=%s",
          diag_counters.invalid_signature,
          static_cast<unsigned>(transport_result.bytes_received),
          transport_result.header_byte,
          frame_hex);
    }
    maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);
    return err_msg_invalid_signature;
  }
  if (((MOSI_frame[CBH] << 8) | MOSI_frame[CBL]) != checksum) {
    diag_counters.base_checksum_fail++;
    if (should_log_reason(DIAG_REASON_BASE_CHECKSUM, now_ms, diag_last_sample_ms, diag_counters.base_checksum_fail)) {
      char frame_hex[128];
      format_frame_hex(MOSI_frame, transport_result.bytes_received, frame_hex, sizeof(frame_hex));
      ESP_LOGW(
          DIAG_TAG,
          "base_checksum_fail count=%u bytes=%u hdr=%02X expected=%04X actual=%04X rx=%s",
          diag_counters.base_checksum_fail,
          static_cast<unsigned>(transport_result.bytes_received),
          transport_result.header_byte,
          checksum,
          static_cast<unsigned>(((MOSI_frame[CBH] << 8) | MOSI_frame[CBL])),
          frame_hex);
    }
    maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);
    return err_msg_invalid_checksum;
  }

  if (frameSize == 33) {
    checksum = calc_checksumFrame33(MOSI_frame);
    if (MOSI_frame[CBL2] != static_cast<uint8_t>(checksum & 0xFF)) {
      diag_counters.extended_checksum_fail++;
      if (should_log_reason(DIAG_REASON_EXTENDED_CHECKSUM, now_ms, diag_last_sample_ms, diag_counters.extended_checksum_fail)) {
        char frame_hex[128];
        format_frame_hex(MOSI_frame, transport_result.bytes_received, frame_hex, sizeof(frame_hex));
        ESP_LOGW(
            DIAG_TAG,
            "extended_checksum_fail count=%u bytes=%u hdr=%02X expected=%02X actual=%02X ext_probe=%s ext_seen=%s rx=%s",
            diag_counters.extended_checksum_fail,
            static_cast<unsigned>(transport_result.bytes_received),
            transport_result.header_byte,
            static_cast<unsigned>(checksum & 0xFF),
            static_cast<unsigned>(MOSI_frame[CBL2]),
            transport_result.extension_probe_attempted ? "true" : "false",
            transport_result.extension_start_seen ? "true" : "false",
            frame_hex);
      }
      maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);
      return err_msg_invalid_checksum;
    }
  }

  diag_counters.ok_frames++;
  if (transport_result.bytes_received >= 33U) {
    diag_counters.ok_33++;
  } else {
    diag_counters.ok_20++;
  }
  maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);

  if (new_datapacket_received) {
    if (frameSize == 33) {
      uint8_t vanesLRtmp = static_cast<uint8_t>((MOSI_frame[DB16] & 0x07) + ((MOSI_frame[DB17] & 0x01) << 4));
      if (vanesLRtmp != status_vanesLR_old) {
        if ((vanesLRtmp & 0x10) != 0) {
          m_cbiStatus->cbiStatusFunction(status_vanesLR, vanesLR_swing);
        } else {
          m_cbiStatus->cbiStatusFunction(status_vanesLR, (vanesLRtmp & 0x07) + 1);
        }
        status_vanesLR_old = vanesLRtmp;
      }

      if ((MOSI_frame[DB17] & 0x04) != status_3Dauto_old) {
        status_3Dauto_old = MOSI_frame[DB17] & 0x04;
        m_cbiStatus->cbiStatusFunction(status_3Dauto, status_3Dauto_old);
      }
    }

    if ((MOSI_frame[DB0] & 0x1c) != status_mode_old) {
      status_mode_old = MOSI_frame[DB0] & 0x1c;
      m_cbiStatus->cbiStatusFunction(status_mode, status_mode_old);
    }

    if ((MOSI_frame[DB0] & 0x01) != status_power_old) {
      status_power_old = MOSI_frame[DB0] & 0x01;
      m_cbiStatus->cbiStatusFunction(status_power, status_power_old);
    }

    uint32_t fantmp = MOSI_frame[DB1] & 0x07;
    if (fantmp != status_fan_old) {
      status_fan_old = static_cast<uint8_t>(fantmp);
      m_cbiStatus->cbiStatusFunction(status_fan, status_fan_old);
    }

    uint32_t vanestmp = (MOSI_frame[DB0] & 0xc0) + ((MOSI_frame[DB1] & 0xB0) >> 4);
    if (vanestmp != status_vanes_old) {
      if ((vanestmp & 0x40) != 0) {
        m_cbiStatus->cbiStatusFunction(status_vanes, vanes_swing);
      } else {
        m_cbiStatus->cbiStatusFunction(status_vanes, (vanestmp & 0x03) + 1);
      }
      status_vanes_old = static_cast<uint8_t>(vanestmp);
    }

    if (MOSI_frame[DB3] != status_troom_old) {
      if (MISO_frame[DB3] != 0xff) {
        status_troom_old = MOSI_frame[DB3];
        m_cbiStatus->cbiStatusFunction(status_troom, status_troom_old);
        lastTroomInternalMillis = 0;
      } else {
        const uint32_t now_ms = esphome::mhi::mhi_now_ms();
        if ((now_ms - lastTroomInternalMillis) > MinTimeInternalTroomMs) {
          lastTroomInternalMillis = now_ms;
          status_troom_old = MOSI_frame[DB3];
          m_cbiStatus->cbiStatusFunction(status_troom, status_troom_old);
        }
      }
    }

    if (MOSI_frame[DB2] != status_tsetpoint_old) {
      status_tsetpoint_old = MOSI_frame[DB2];
      m_cbiStatus->cbiStatusFunction(status_tsetpoint, status_tsetpoint_old);
    }

    if (MOSI_frame[DB4] != status_errorcode_old) {
      status_errorcode_old = MOSI_frame[DB4];
      m_cbiStatus->cbiStatusFunction(status_errorcode, status_errorcode_old);
    }

    bool MOSI_type_opdata = (MOSI_frame[DB10] & 0x30) == 0x10;

    switch (MOSI_frame[DB9]) {
      case 0x94:
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if (MOSI_type_opdata) {
            if (((MOSI_frame[DB12] << 8) + (MOSI_frame[DB11])) != op_kwh_old) {
              op_kwh_old = (MOSI_frame[DB12] << 8) + (MOSI_frame[DB11]);
              m_cbiStatus->cbiStatusFunction(opdata_kwh, op_kwh_old);
            }
          }
        }
        break;

      case 0x02:
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB10] != op_mode_old) {
              op_mode_old = MOSI_frame[DB10];
              m_cbiStatus->cbiStatusFunction(opdata_mode, (op_mode_old & 0x0f) << 2);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_mode, (MOSI_frame[DB10] & 0x0f) << 2);
          }
        }
        break;

      case 0x05:
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if (MOSI_frame[DB10] == 0x13) {
            if (MOSI_frame[DB11] != op_settemp_old) {
              op_settemp_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_tsetpoint, op_settemp_old);
            }
          } else if (MOSI_frame[DB10] == 0x33) {
            m_cbiStatus->cbiStatusFunction(erropdata_tsetpoint, MOSI_frame[DB11]);
          }
        }
        break;

      case 0x81:
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if ((MOSI_frame[DB10] & 0x30) == 0x20) {
            if (MOSI_frame[DB11] != op_thi_r1_old) {
              op_thi_r1_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_thi_r1, op_thi_r1_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_thi_r1, MOSI_frame[DB11]);
          }
        } else {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != op_thi_r2_old) {
              op_thi_r2_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_thi_r2, op_thi_r2_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_thi_r2, MOSI_frame[DB11]);
          }
        }
        break;

      case 0x87:
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != op_thi_r3_old) {
              op_thi_r3_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_thi_r3, op_thi_r3_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_thi_r3, MOSI_frame[DB11]);
          }
        }
        break;

      case 0x80:
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if ((MOSI_frame[DB10] & 0x30) == 0x20) {
            if (MOSI_frame[DB11] != op_return_air_old) {
              op_return_air_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_return_air, op_return_air_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_return_air, MOSI_frame[DB11]);
          }
        } else {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != op_outdoor_old) {
              op_outdoor_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_outdoor, op_outdoor_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_outdoor, MOSI_frame[DB11]);
          }
        }
        break;

      case 0x1f:
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB10] != op_iu_fanspeed_old) {
              op_iu_fanspeed_old = MOSI_frame[DB10];
              m_cbiStatus->cbiStatusFunction(opdata_iu_fanspeed, op_iu_fanspeed_old & 0x0f);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_iu_fanspeed, MOSI_frame[DB10] & 0x0f);
          }
        } else {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB10] != op_ou_fanspeed_old) {
              op_ou_fanspeed_old = MOSI_frame[DB10];
              m_cbiStatus->cbiStatusFunction(opdata_ou_fanspeed, op_ou_fanspeed_old & 0x0f);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_ou_fanspeed, MOSI_frame[DB10] & 0x0f);
          }
        }
        break;

      case 0x1e:
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != op_total_iu_run_old) {
              op_total_iu_run_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_total_iu_run, op_total_iu_run_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_total_iu_run, MOSI_frame[DB11]);
          }
        } else {
          if (MOSI_frame[DB10] == 0x11) {
            if (MOSI_frame[DB11] != op_total_comp_run_old) {
              op_total_comp_run_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_total_comp_run, op_total_comp_run_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_total_comp_run, MOSI_frame[DB11]);
          }
        }
        break;

      case 0x82:
        if ((MOSI_frame[DB6] & 0x80) == 0) {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != op_tho_r1_old) {
              op_tho_r1_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_tho_r1, op_tho_r1_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_tho_r1, MOSI_frame[DB11]);
          }
        }
        break;

      case 0x11:
        if ((MOSI_frame[DB6] & 0x80) == 0) {
          if (MOSI_type_opdata) {
            if (((MOSI_frame[DB10] << 8) | MOSI_frame[DB11]) != op_comp_old) {
              op_comp_old = (MOSI_frame[DB10] << 8) | MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_comp, op_comp_old & 0x0fff);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_comp, ((MOSI_frame[DB10] << 8) | MOSI_frame[DB11]) & 0x0fff);
          }
        }
        break;

      case 0x85:
        if ((MOSI_frame[DB6] & 0x80) == 0) {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != op_td_old) {
              op_td_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_td, op_td_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_td, MOSI_frame[DB11]);
          }
        }
        break;

      case 0x90:
        if ((MOSI_frame[DB6] & 0x80) == 0) {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != op_ct_old) {
              op_ct_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_ct, op_ct_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_ct, MOSI_frame[DB11]);
          }
        }
        break;

      case 0xb1:
        if ((MOSI_frame[DB6] & 0x80) == 0) {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != op_tdsh_old) {
              op_tdsh_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_tdsh, op_tdsh_old / 2);
            }
          }
        }
        break;

      case 0x7c:
        if ((MOSI_frame[DB6] & 0x80) == 0) {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB11] != op_protection_no_old) {
              op_protection_no_old = MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_protection_no, op_protection_no_old);
            }
          }
        }
        break;

      case 0x0c:
        if ((MOSI_frame[DB6] & 0x80) == 0) {
          if (MOSI_type_opdata) {
            if (MOSI_frame[DB10] != op_defrost_old) {
              op_defrost_old = MOSI_frame[DB10];
              m_cbiStatus->cbiStatusFunction(opdata_defrost, op_defrost_old & 0b1);
            }
          }
        }
        break;

      case 0x13:
        if ((MOSI_frame[DB6] & 0x80) == 0) {
          if (MOSI_type_opdata) {
            if (((MOSI_frame[DB12] << 8) | MOSI_frame[DB11]) != op_ou_eev1_old) {
              op_ou_eev1_old = (MOSI_frame[DB12] << 8) | MOSI_frame[DB11];
              m_cbiStatus->cbiStatusFunction(opdata_ou_eev1, op_ou_eev1_old);
            }
          } else {
            m_cbiStatus->cbiStatusFunction(erropdata_ou_eev1, (MOSI_frame[DB12] << 8) | MOSI_frame[DB11]);
          }
        }
        break;

      case 0x45:
        if ((MOSI_frame[DB6] & 0x80) != 0) {
          if (MOSI_frame[DB10] == 0x11) {
            m_cbiStatus->cbiStatusFunction(erropdata_errorcode, MOSI_frame[DB11]);
          } else if (MOSI_frame[DB10] == 0x12) {
            erropdataCnt = static_cast<uint8_t>(MOSI_frame[DB11] + 4);
          }
        }
        break;

      case 0x00:
      case 0xff:
        break;

      default:
        m_cbiStatus->cbiStatusFunction(opdata_unknown, (MOSI_frame[DB10] << 8) | MOSI_frame[DB9]);
        break;
    }
  }

  return static_cast<int>(call_counter);
}