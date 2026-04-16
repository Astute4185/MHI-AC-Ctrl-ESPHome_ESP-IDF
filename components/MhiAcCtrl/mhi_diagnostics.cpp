#include "mhi_diagnostics.h"

#include <cstdio>

#include "esphome/core/log.h"

namespace esphome {
namespace mhi {

const char *mhi_diag_reason_name(MhiDiagReason reason) {
  switch (reason) {
    case MhiDiagReason::NONE:
      return "none";
    case MhiDiagReason::SHORT_CAPTURE:
      return "short_capture";
    case MhiDiagReason::INVALID_SIGNATURE:
      return "invalid_signature";
    case MhiDiagReason::BASE_CHECKSUM_FAIL:
      return "base_checksum_fail";
    case MhiDiagReason::EXTENDED_CHECKSUM_FAIL:
      return "extended_checksum_fail";
    case MhiDiagReason::TIMEOUT_LOW:
      return "timeout_low";
    case MhiDiagReason::TIMEOUT_HIGH:
      return "timeout_high";
    case MhiDiagReason::TIMEOUT_OTHER:
      return "timeout_other";
    case MhiDiagReason::TRANSPORT_UNAVAILABLE:
      return "transport_unavailable";
    case MhiDiagReason::UNKNOWN:
    default:
      return "unknown";
  }
}

bool mhi_diag_reason_is_expected(MhiDiagReason reason) {
  switch (reason) {
    case MhiDiagReason::SHORT_CAPTURE:
    case MhiDiagReason::INVALID_SIGNATURE:
    case MhiDiagReason::BASE_CHECKSUM_FAIL:
    case MhiDiagReason::EXTENDED_CHECKSUM_FAIL:
    case MhiDiagReason::TIMEOUT_LOW:
    case MhiDiagReason::TIMEOUT_HIGH:
    case MhiDiagReason::TIMEOUT_OTHER:
      return true;

    case MhiDiagReason::NONE:
    case MhiDiagReason::TRANSPORT_UNAVAILABLE:
    case MhiDiagReason::UNKNOWN:
    default:
      return false;
  }
}

void mhi_format_diag_flags(
    bool extension_probe_attempted,
    bool extension_start_seen,
    bool critical_capture_used,
    bool next_frame_signature_after_tail,
    char *out,
    std::size_t out_len) {
  if (out == nullptr || out_len == 0U) {
    return;
  }

  out[0] = '\0';
  std::size_t pos = 0U;
  bool first = true;

  const auto append_flag = [&](const char *flag) {
    if (flag == nullptr || pos >= out_len) {
      return;
    }

    const int written = std::snprintf(
        out + pos,
        out_len - pos,
        "%s%s",
        first ? "" : "|",
        flag);

    if (written <= 0) {
      return;
    }

    const std::size_t used = static_cast<std::size_t>(written);
    if (used >= (out_len - pos)) {
      pos = out_len - 1U;
      out[pos] = '\0';
      first = false;
      return;
    }

    pos += used;
    first = false;
  };

  if (extension_probe_attempted) {
    append_flag("probe");
  }
  if (extension_start_seen) {
    append_flag("ext");
  }
  if (critical_capture_used) {
    append_flag("crit");
  }
  if (next_frame_signature_after_tail) {
    append_flag("nextsig");
  }

  if (first) {
    std::snprintf(out, out_len, "none");
  } else {
    out[out_len - 1U] = '\0';
  }
}

void mhi_log_platform_loop_status(const char *tag, const MhiDiagStatus &status) {
  if (status.error_code >= 0 || status.reason == MhiDiagReason::NONE) {
    return;
  }

  if (mhi_diag_reason_is_expected(status.reason)) {
    return;
  }

  ESP_LOGW(
      tag,
      "mhi_ac_ctrl_core loop issue: %s (%d)",
      mhi_diag_reason_name(status.reason),
      status.error_code);
}

}  // namespace mhi
}  // namespace esphome