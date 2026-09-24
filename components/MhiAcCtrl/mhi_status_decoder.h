#pragma once

#include <cstdint>

#include "mhi_frame.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiDecodedStatus {
  bool valid{false};
  bool extended{false};

  bool power{false};

  // Raw protocol mode bits: MOSI DB0[4:2]
  uint8_t mode_raw{0};

  // Normalised mode value: 0=Auto, 1=Dry, 2=Cool, 3=Fan, 4=Heat
  uint8_t mode{0};

  uint8_t fan_raw{0};
  uint8_t fan{0};

  float target_temp_c{0.0f};
  float room_temp_c{0.0f};

  bool vertical_swing{false};
  uint8_t vertical_vane{0};
  uint8_t vertical_raw{0};

  bool has_horizontal_vane{false};
  bool horizontal_swing{false};
  uint8_t horizontal_vane{0};
  uint8_t horizontal_raw{0};

  bool has_3d_auto{false};
  bool three_d_auto{false};

  // Self Clean feedback is currently provisional. Two independent bits were
  // observed moving together in recurring 33-byte status frames. Keep both
  // candidates visible until hardware captures establish a canonical field.
  bool has_self_clean_candidates{false};
  bool self_clean_db7_candidate{false};
  bool self_clean_db13_candidate{false};

  // Agreement-gated public feedback. This is present only when the two
  // observed candidate bits agree, so a disagreement cannot overwrite the
  // last confirmed state.
  bool has_self_clean{false};
  bool self_clean{false};

  bool has_extended_louver_raw{false};
  uint8_t extended_louver_db16{0};
  uint8_t extended_louver_db17{0};

  uint8_t error_code{0};
};

class MhiStatusDecoder {
 public:
  static bool decode_mosi(const MhiFrameView& mosi, MhiDecodedStatus& out);
  static bool is_extended_feedback_status_frame(const MhiFrameView& mosi);

 private:
  static uint8_t decode_mode(uint8_t db0);
  static uint8_t decode_fan(uint8_t db1, uint8_t db6);
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome