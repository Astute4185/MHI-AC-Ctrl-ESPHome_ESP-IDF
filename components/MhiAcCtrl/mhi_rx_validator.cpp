#include "mhi_rx_validator.h"

#include "MHI-AC-Ctrl-core.h"
#include "mhi_frame_layout.h"
#include "mhi_time.h"

#include "esphome/core/log.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace {
static const char *DIAG_TAG = "mhi.diag";
constexpr uint32_t kDiagSummaryIntervalMs = 10000U;
constexpr uint32_t kDiagSampleIntervalMs = 15000U;
constexpr std::size_t kDiagMaxMismatchPreview = 12U;

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

void format_overcapture_hex(const esphome::mhi::MhiFrameExchangeResult &result, char *out, std::size_t out_len) {
  if (out_len == 0) {
    return;
  }
  if (result.overcapture_len == 0U) {
    std::snprintf(out, out_len, "none");
    return;
  }
  out[0] = '\0';
  std::size_t pos = 0;
  for (std::size_t i = 0; i < result.overcapture_len && pos + 4 < out_len; i++) {
    const int written = std::snprintf(
        out + pos,
        out_len - pos,
        "%02X%s",
        result.overcapture_bytes[i],
        (i + 1U < result.overcapture_len) ? " " : "");
    if (written <= 0) {
      break;
    }
    pos += static_cast<std::size_t>(written);
  }
  out[out_len - 1] = '\0';
}

void format_mismatch_preview(
    const uint8_t *good_frame,
    std::size_t good_len,
    const uint8_t *bad_frame,
    std::size_t bad_len,
    char *out,
    std::size_t out_len) {
  if (out_len == 0) {
    return;
  }
  out[0] = '\0';
  const std::size_t cmp_len = (good_len < bad_len) ? good_len : bad_len;
  std::size_t pos = 0;
  std::size_t shown = 0;
  for (std::size_t i = 0; i < cmp_len && shown < kDiagMaxMismatchPreview && pos + 16 < out_len; i++) {
    if (good_frame[i] == bad_frame[i]) {
      continue;
    }
    const uint8_t delta = static_cast<uint8_t>(good_frame[i] ^ bad_frame[i]);
    const int written = std::snprintf(
        out + pos,
        out_len - pos,
        "%u:%02X>%02X(^%02X)%s",
        static_cast<unsigned>(i),
        good_frame[i],
        bad_frame[i],
        delta,
        (shown + 1U < kDiagMaxMismatchPreview) ? " " : "");
    if (written <= 0) {
      break;
    }
    pos += static_cast<std::size_t>(written);
    shown++;
  }
  if (shown == 0 && good_len != bad_len && pos + 32 < out_len) {
    std::snprintf(out + pos, out_len - pos, "len:%u>%u", static_cast<unsigned>(good_len), static_cast<unsigned>(bad_len));
  }
  out[out_len - 1] = '\0';
}

void update_mismatch_histogram(
    uint32_t *histogram,
    const uint8_t *good_frame,
    std::size_t good_len,
    const uint8_t *bad_frame,
    std::size_t bad_len) {
  const std::size_t cmp_len = (good_len < bad_len) ? good_len : bad_len;
  for (std::size_t i = 0; i < cmp_len && i < kMhiMaxFrameBytes; i++) {
    if (good_frame[i] != bad_frame[i]) {
      histogram[i]++;
    }
  }
}

void format_histogram_top(const uint32_t *histogram, char *out, std::size_t out_len) {
  if (out_len == 0) {
    return;
  }
  out[0] = '\0';

  uint32_t top_counts[6] = {0};
  std::size_t top_indices[6] = {0};
  for (std::size_t i = 0; i < kMhiMaxFrameBytes; i++) {
    const uint32_t value = histogram[i];
    if (value == 0) {
      continue;
    }
    for (std::size_t slot = 0; slot < 6; slot++) {
      if (value > top_counts[slot]) {
        for (std::size_t shift = 5; shift > slot; shift--) {
          top_counts[shift] = top_counts[shift - 1];
          top_indices[shift] = top_indices[shift - 1];
        }
        top_counts[slot] = value;
        top_indices[slot] = i;
        break;
      }
    }
  }

  std::size_t pos = 0;
  bool first = true;
  for (std::size_t slot = 0; slot < 6 && pos + 16 < out_len; slot++) {
    if (top_counts[slot] == 0) {
      break;
    }
    const int written = std::snprintf(
        out + pos,
        out_len - pos,
        "%s%u:%u",
        first ? "" : " ",
        static_cast<unsigned>(top_indices[slot]),
        static_cast<unsigned>(top_counts[slot]));
    if (written <= 0) {
      break;
    }
    pos += static_cast<std::size_t>(written);
    first = false;
  }
  if (first) {
    std::snprintf(out, out_len, "none");
  }
  out[out_len - 1] = '\0';
}

bool should_log_reason(MhiDiagSampleReason reason, uint32_t now_ms, uint32_t *last_log_ms, uint32_t count) {
  if (count <= 1U) {
    last_log_ms[static_cast<std::size_t>(reason)] = now_ms;
    return true;
  }
  if ((now_ms - last_log_ms[static_cast<std::size_t>(reason)]) >= kDiagSampleIntervalMs) {
    last_log_ms[static_cast<std::size_t>(reason)] = now_ms;
    return true;
  }
  return false;
}

void maybe_log_summary(const MhiDiagCounters &counters, uint32_t now_ms, uint32_t &last_summary_ms) {
  if ((now_ms - last_summary_ms) < kDiagSummaryIntervalMs) {
    return;
  }
  last_summary_ms = now_ms;
  char mismatch_top[96];
  format_histogram_top(counters.byte_mismatch_counts, mismatch_top, sizeof(mismatch_top));
  ESP_LOGI(
      DIAG_TAG,
      "summary ok=%u ok20=%u ok33=%u short=%u sig=%u basechk=%u extchk=%u t_low=%u t_high=%u t_other=%u hdr6c=%u hdr6d=%u hdrother=%u ext_probe=%u ext_seen=%u crit=%u nextsig=%u mismatch_top=%s",
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
      counters.extension_start_seen,
      counters.critical_capture_frames,
      counters.next_frame_sig_after_tail,
      mismatch_top);
}

void maybe_log_bad_frame_with_reference(
    MhiDiagSampleReason reason,
    uint32_t now_ms,
    uint32_t *last_log_ms,
    uint32_t count,
    const uint8_t *bad_frame,
    std::size_t bad_len,
    const MhiLastGoodFrame &last_good,
    const char *prefix_fmt,
    ...) {
  if (!should_log_reason(reason, now_ms, last_log_ms, count)) {
    return;
  }

  char bad_hex[160];
  format_frame_hex(bad_frame, bad_len, bad_hex, sizeof(bad_hex));

  char good_hex[160];
  char mismatch_preview[256];
  if (last_good.valid) {
    format_frame_hex(last_good.frame, last_good.len, good_hex, sizeof(good_hex));
    format_mismatch_preview(last_good.frame, last_good.len, bad_frame, bad_len, mismatch_preview, sizeof(mismatch_preview));
  } else {
    std::snprintf(good_hex, sizeof(good_hex), "none");
    std::snprintf(mismatch_preview, sizeof(mismatch_preview), "no_last_good_frame");
  }

  char prefix[192];
  va_list args;
  va_start(args, prefix_fmt);
  std::vsnprintf(prefix, sizeof(prefix), prefix_fmt, args);
  va_end(args);

  ESP_LOGW(
      DIAG_TAG,
      "%s count=%u rx=%s ref=%s delta=%s",
      prefix,
      count,
      bad_hex,
      good_hex,
      mismatch_preview);
}

}  // namespace

MhiRxValidationResult MhiRxValidator::exchange_and_validate(
    esphome::mhi::MhiTransport *transport,
    uint8_t *miso_frame,
    uint8_t *mosi_frame,
    uint8_t frame_size,
    uint32_t max_time_ms,
    MhiDiagRuntimeState &diag_state,
    esphome::mhi::MhiDiagReason &last_diag_reason) {
  MhiRxValidationResult result{};
  MhiDiagCounters &diag_counters = diag_state.counters;
  MhiLastGoodFrame &diag_last_good_frame = diag_state.last_good_frame;
  uint32_t &diag_last_summary_ms = diag_state.last_summary_ms;
  uint32_t *diag_last_sample_ms = diag_state.last_sample_ms;

  if (transport == nullptr) {
    last_diag_reason = esphome::mhi::MhiDiagReason::TRANSPORT_UNAVAILABLE;
    result.status = err_msg_timeout_SCK_low;
    return result;
  }

  const esphome::mhi::MhiFrameExchangeResult transport_result = transport->exchange_frame(
      miso_frame,
      mosi_frame,
      kMhiMaxFrameBytes,
      max_time_ms);

  result.new_data_packet_received = transport_result.new_data_packet_received;

  if (transport_result.extension_probe_attempted) {
    diag_counters.extension_probe_attempted++;
  }
  if (transport_result.extension_start_seen) {
    diag_counters.extension_start_seen++;
  }
  if (transport_result.critical_capture_used) {
    diag_counters.critical_capture_frames++;
  }
  if (transport_result.next_frame_signature_after_tail) {
    diag_counters.next_frame_sig_after_tail++;
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
      last_diag_reason = esphome::mhi::MhiDiagReason::TIMEOUT_LOW;
      diag_counters.timeout_low++;
      if (should_log_reason(MhiDiagSampleReason::TIMEOUT_LOW, now_ms, diag_last_sample_ms, diag_counters.timeout_low)) {
        char flags[48];
        esphome::mhi::mhi_format_diag_flags(
            transport_result.extension_probe_attempted,
            transport_result.extension_start_seen,
            transport_result.critical_capture_used,
            transport_result.next_frame_signature_after_tail,
            flags,
            sizeof(flags));
        ESP_LOGW(
            DIAG_TAG,
            "timeout_low count=%u frame_hint=%u len=%u hdr=%02X flags=%s",
            diag_counters.timeout_low,
            frame_size,
            static_cast<unsigned>(transport_result.bytes_received),
            transport_result.header_byte,
            flags);
      }
    } else if (transport_result.status == err_msg_timeout_SCK_high) {
      last_diag_reason = esphome::mhi::MhiDiagReason::TIMEOUT_HIGH;
      diag_counters.timeout_high++;
      if (should_log_reason(MhiDiagSampleReason::TIMEOUT_HIGH, now_ms, diag_last_sample_ms, diag_counters.timeout_high)) {
        char flags[48];
        esphome::mhi::mhi_format_diag_flags(
            transport_result.extension_probe_attempted,
            transport_result.extension_start_seen,
            transport_result.critical_capture_used,
            transport_result.next_frame_signature_after_tail,
            flags,
            sizeof(flags));
        ESP_LOGW(
            DIAG_TAG,
            "timeout_high count=%u frame_hint=%u len=%u hdr=%02X flags=%s",
            diag_counters.timeout_high,
            frame_size,
            static_cast<unsigned>(transport_result.bytes_received),
            transport_result.header_byte,
            flags);
      }
    } else {
      last_diag_reason = esphome::mhi::MhiDiagReason::TIMEOUT_OTHER;
      diag_counters.timeout_other++;
      if (should_log_reason(MhiDiagSampleReason::TIMEOUT_OTHER, now_ms, diag_last_sample_ms, diag_counters.timeout_other)) {
        char flags[48];
        esphome::mhi::mhi_format_diag_flags(
            transport_result.extension_probe_attempted,
            transport_result.extension_start_seen,
            transport_result.critical_capture_used,
            transport_result.next_frame_signature_after_tail,
            flags,
            sizeof(flags));
        ESP_LOGW(
            DIAG_TAG,
            "timeout_other code=%d count=%u frame_hint=%u len=%u hdr=%02X flags=%s",
            transport_result.status,
            diag_counters.timeout_other,
            frame_size,
            static_cast<unsigned>(transport_result.bytes_received),
            transport_result.header_byte,
            flags);
      }
    }
    maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);
    result.status = transport_result.status;
    return result;
  }

  if (frame_size == 33 && transport_result.bytes_received < 33U) {
    last_diag_reason = esphome::mhi::MhiDiagReason::SHORT_CAPTURE;
    diag_counters.short_capture++;
    update_mismatch_histogram(
        diag_counters.byte_mismatch_counts,
        diag_last_good_frame.frame,
        diag_last_good_frame.len,
        mosi_frame,
        transport_result.bytes_received);
    char flags[48];
    esphome::mhi::mhi_format_diag_flags(
        transport_result.extension_probe_attempted,
        transport_result.extension_start_seen,
        transport_result.critical_capture_used,
        transport_result.next_frame_signature_after_tail,
        flags,
        sizeof(flags));
    maybe_log_bad_frame_with_reference(
        MhiDiagSampleReason::SHORT_CAPTURE,
        now_ms,
        diag_last_sample_ms,
        diag_counters.short_capture,
        mosi_frame,
        transport_result.bytes_received,
        diag_last_good_frame,
        "short_capture len=%u/33 hdr=%02X flags=%s",
        static_cast<unsigned>(transport_result.bytes_received),
        transport_result.header_byte,
        flags);
    maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);
    result.status = err_msg_invalid_checksum;
    return result;
  }

  uint16_t checksum = mhi_calc_checksum(mosi_frame);
  if (((mosi_frame[SB0] & 0xfe) != 0x6c) | (mosi_frame[SB1] != 0x80) | (mosi_frame[SB2] != 0x04)) {
    last_diag_reason = esphome::mhi::MhiDiagReason::INVALID_SIGNATURE;
    diag_counters.invalid_signature++;
    if (diag_last_good_frame.valid) {
      update_mismatch_histogram(diag_counters.byte_mismatch_counts, diag_last_good_frame.frame, diag_last_good_frame.len, mosi_frame, transport_result.bytes_received);
    }
    char over_hex[16];
    format_overcapture_hex(transport_result, over_hex, sizeof(over_hex));
    char flags[48];
    esphome::mhi::mhi_format_diag_flags(
        transport_result.extension_probe_attempted,
        transport_result.extension_start_seen,
        transport_result.critical_capture_used,
        transport_result.next_frame_signature_after_tail,
        flags,
        sizeof(flags));
    maybe_log_bad_frame_with_reference(
        MhiDiagSampleReason::INVALID_SIGNATURE,
        now_ms,
        diag_last_sample_ms,
        diag_counters.invalid_signature,
        mosi_frame,
        transport_result.bytes_received,
        diag_last_good_frame,
        "invalid_signature len=%u hdr=%02X sb=%02X %02X %02X flags=%s over=%s",
        static_cast<unsigned>(transport_result.bytes_received),
        transport_result.header_byte,
        mosi_frame[0],
        mosi_frame[1],
        mosi_frame[2],
        flags,
        over_hex);
    maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);
    result.status = err_msg_invalid_signature;
    return result;
  }

  if (((mosi_frame[CBH] << 8) | mosi_frame[CBL]) != checksum) {
    last_diag_reason = esphome::mhi::MhiDiagReason::BASE_CHECKSUM_FAIL;
    diag_counters.base_checksum_fail++;
    update_mismatch_histogram(
        diag_counters.byte_mismatch_counts,
        diag_last_good_frame.frame,
        diag_last_good_frame.len,
        mosi_frame,
        transport_result.bytes_received);
    char flags[48];
    esphome::mhi::mhi_format_diag_flags(
        transport_result.extension_probe_attempted,
        transport_result.extension_start_seen,
        transport_result.critical_capture_used,
        transport_result.next_frame_signature_after_tail,
        flags,
        sizeof(flags));
    maybe_log_bad_frame_with_reference(
        MhiDiagSampleReason::BASE_CHECKSUM,
        now_ms,
        diag_last_sample_ms,
        diag_counters.base_checksum_fail,
        mosi_frame,
        transport_result.bytes_received,
        diag_last_good_frame,
        "base_checksum_fail len=%u hdr=%02X chk=%04X!=%04X flags=%s",
        static_cast<unsigned>(transport_result.bytes_received),
        transport_result.header_byte,
        checksum,
        static_cast<unsigned>(((mosi_frame[CBH] << 8) | mosi_frame[CBL])),
        flags);
    maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);
    result.status = err_msg_invalid_checksum;
    return result;
  }

  if (frame_size == 33) {
    checksum = mhi_calc_checksum_frame33(mosi_frame);
    if (mosi_frame[CBL2] != static_cast<uint8_t>(checksum & 0xFF)) {
      last_diag_reason = esphome::mhi::MhiDiagReason::EXTENDED_CHECKSUM_FAIL;
      diag_counters.extended_checksum_fail++;
      update_mismatch_histogram(
          diag_counters.byte_mismatch_counts,
          diag_last_good_frame.frame,
          diag_last_good_frame.len,
          mosi_frame,
          transport_result.bytes_received);
      char flags[48];
      esphome::mhi::mhi_format_diag_flags(
          transport_result.extension_probe_attempted,
          transport_result.extension_start_seen,
          transport_result.critical_capture_used,
          transport_result.next_frame_signature_after_tail,
          flags,
          sizeof(flags));
      maybe_log_bad_frame_with_reference(
          MhiDiagSampleReason::EXTENDED_CHECKSUM,
          now_ms,
          diag_last_sample_ms,
          diag_counters.extended_checksum_fail,
          mosi_frame,
          transport_result.bytes_received,
          diag_last_good_frame,
          "extended_checksum_fail len=%u hdr=%02X chk=%02X!=%02X flags=%s",
          static_cast<unsigned>(transport_result.bytes_received),
          transport_result.header_byte,
          static_cast<unsigned>(checksum & 0xFF),
          static_cast<unsigned>(mosi_frame[CBL2]),
          flags);
      maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);
      result.status = err_msg_invalid_checksum;
      return result;
    }
  }

  last_diag_reason = esphome::mhi::MhiDiagReason::NONE;
  diag_counters.ok_frames++;
  if (transport_result.bytes_received >= 33U) {
    diag_counters.ok_33++;
  } else {
    diag_counters.ok_20++;
  }
  diag_last_good_frame.valid = true;
  diag_last_good_frame.len = transport_result.bytes_received;
  std::memcpy(diag_last_good_frame.frame, mosi_frame, transport_result.bytes_received);
  maybe_log_summary(diag_counters, now_ms, diag_last_summary_ms);

  result.status = err_msg_valid_frame;
  return result;
}
