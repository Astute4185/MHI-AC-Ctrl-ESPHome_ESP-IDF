#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace mhi {

enum class MhiDiagReason : uint8_t {
  NONE = 0,
  SHORT_CAPTURE,
  INVALID_SIGNATURE,
  BASE_CHECKSUM_FAIL,
  EXTENDED_CHECKSUM_FAIL,
  TIMEOUT_LOW,
  TIMEOUT_HIGH,
  TIMEOUT_OTHER,
  TRANSPORT_UNAVAILABLE,
  UNKNOWN,
};

struct MhiDiagStatus {
  int error_code{0};
  MhiDiagReason reason{MhiDiagReason::NONE};
};

const char *mhi_diag_reason_name(MhiDiagReason reason);
bool mhi_diag_reason_is_expected(MhiDiagReason reason);
void mhi_format_diag_flags(
    bool extension_probe_attempted,
    bool extension_start_seen,
    bool critical_capture_used,
    bool next_frame_signature_after_tail,
    char *out,
    std::size_t out_len);
void mhi_log_platform_loop_status(const char *tag, const MhiDiagStatus &status);

}  // namespace mhi
}  // namespace esphome