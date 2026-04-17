#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_frame_layout.h"

struct MhiTxWriteState {
  uint8_t new_power{0};
  uint8_t new_mode{0};
  uint8_t new_tsetpoint{0};
  uint8_t new_fan{0};
  uint8_t new_vanes0{0};
  uint8_t new_vanes1{0};
  bool request_erropdata{false};
  uint8_t new_troom{0xFF};
  float troom_offset{0.0f};
  uint8_t new_vaneslr0{0};
  uint8_t new_vaneslr1{0};
  uint8_t new_3dauto{0};
};

struct MhiLoopRuntimeState {
  uint8_t opdata_no{0};
  uint8_t erropdata_count{0};
  bool doubleframe{false};
  int frame{1};
  uint8_t mosi_frame[kMhiMaxFrameBytes] = {0};
  uint8_t miso_frame[kMhiMaxFrameBytes] = {
      0xA9, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00,
      0x00, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x22};
  uint32_t call_counter{0};
  uint32_t last_troom_internal_ms{0};
};


struct MhiStatusCacheState {
  uint8_t power_old{0xFF};
  uint8_t mode_old{0xFF};
  uint8_t fan_old{0xFF};
  uint8_t vanes_old{0xFF};
  uint8_t troom_old{0xFE};
  uint8_t tsetpoint_old{0x00};
  uint8_t errorcode_old{0xFF};
  uint8_t vaneslr_old{0xFF};
  uint8_t auto3d_old{0xFF};
};

struct MhiOpDataCacheState {
  uint16_t op_kwh_old{0xFFFF};
  uint8_t op_mode_old{0xFF};
  uint8_t op_settemp_old{0xFF};
  uint8_t op_return_air_old{0xFF};
  uint8_t op_iu_fanspeed_old{0xFF};
  uint8_t op_thi_r1_old{0x00};
  uint8_t op_thi_r2_old{0x00};
  uint8_t op_thi_r3_old{0x00};
  uint8_t op_total_iu_run_old{0};
  uint8_t op_outdoor_old{0xFF};
  uint8_t op_tho_r1_old{0x00};
  uint8_t op_total_comp_run_old{0};
  uint8_t op_ct_old{0xFF};
  uint8_t op_tdsh_old{0xFF};
  uint8_t op_protection_no_old{0xFF};
  uint8_t op_ou_fanspeed_old{0xFF};
  uint8_t op_defrost_old{0x00};
  uint16_t op_comp_old{0xFFFF};
  uint8_t op_td_old{0x00};
  uint16_t op_ou_eev1_old{0xFFFF};
};

struct MhiDiagCounters {
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
  uint32_t critical_capture_frames = 0;
  uint32_t next_frame_sig_after_tail = 0;
  uint32_t byte_mismatch_counts[kMhiMaxFrameBytes] = {0};
};

struct MhiLastGoodFrame {
  bool valid = false;
  std::size_t len = 0;
  uint8_t frame[kMhiMaxFrameBytes] = {0};
};

enum class MhiDiagSampleReason : uint8_t {
  SHORT_CAPTURE = 0,
  INVALID_SIGNATURE = 1,
  BASE_CHECKSUM = 2,
  EXTENDED_CHECKSUM = 3,
  TIMEOUT_LOW = 4,
  TIMEOUT_HIGH = 5,
  TIMEOUT_OTHER = 6,
  COUNT = 7,
};

constexpr std::size_t kMhiDiagReasonCount =
    static_cast<std::size_t>(MhiDiagSampleReason::COUNT);

struct MhiDiagRuntimeState {
  MhiDiagCounters counters{};
  MhiLastGoodFrame last_good_frame{};
  uint32_t last_summary_ms{0};
  uint32_t last_sample_ms[kMhiDiagReasonCount] = {0};
};
