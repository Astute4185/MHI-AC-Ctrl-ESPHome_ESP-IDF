#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_command.h"
#include "mhi_frame.h"

namespace esphome {
namespace mhi_ac_ctrl {

enum MhiOpdataRequestMask : uint32_t {
  MHI_OPDATA_REQ_MODE = (1UL << 0),
  MHI_OPDATA_REQ_TSETPOINT = (1UL << 1),
  MHI_OPDATA_REQ_RETURN_AIR = (1UL << 2),
  MHI_OPDATA_REQ_THI_R1 = (1UL << 3),
  MHI_OPDATA_REQ_THI_R2 = (1UL << 4),
  MHI_OPDATA_REQ_THI_R3 = (1UL << 5),
  MHI_OPDATA_REQ_IU_FANSPEED = (1UL << 6),
  MHI_OPDATA_REQ_TOTAL_IU_RUN = (1UL << 7),
  MHI_OPDATA_REQ_OUTDOOR = (1UL << 8),
  MHI_OPDATA_REQ_THO_R1 = (1UL << 9),
  MHI_OPDATA_REQ_COMP = (1UL << 10),
  MHI_OPDATA_REQ_TD = (1UL << 11),
  MHI_OPDATA_REQ_CT = (1UL << 12),
  MHI_OPDATA_REQ_TDSH = (1UL << 13),
  MHI_OPDATA_REQ_PROTECTION_NO = (1UL << 14),
  MHI_OPDATA_REQ_OU_FANSPEED = (1UL << 15),
  MHI_OPDATA_REQ_DEFROST = (1UL << 16),
  MHI_OPDATA_REQ_TOTAL_COMP_RUN = (1UL << 17),
  MHI_OPDATA_REQ_OU_EEV1 = (1UL << 18),
  MHI_OPDATA_REQ_KWH = (1UL << 19),
};

constexpr uint32_t kMhiDefaultOpdataMask = MHI_OPDATA_REQ_MODE;

struct MhiTxRuntime {
  uint8_t opdata_index{0};
  uint8_t room_temp_override_raw{0xFF};
  uint8_t error_opdata_count{0};
  bool double_frame{false};
  uint32_t frame_counter{1};
};

struct MhiTxBuildConfig {
  std::size_t frame_size{kMhiFrame20Bytes};
  uint32_t enabled_opdata_mask{kMhiDefaultOpdataMask};

  // Last accepted 33-byte extended-louver feedback. DB16/DB17 carry a
  // composite horizontal-vane / horizontal-swing / 3D-auto state; 3D-only
  // commands must preserve the currently known horizontal part.
  bool has_extended_louver_state{false};
  uint8_t extended_louver_db16{0};
  uint8_t extended_louver_db17{0x0A};
  bool extended_louver_horizontal_swing{false};
  uint8_t extended_louver_horizontal_vane{1};
  bool extended_louver_three_d_auto{false};
};

struct MhiTxBuildResult {
  uint32_t encoded_command_mask{0};
  uint32_t unsupported_command_mask{0};
  MhiCommandIntent intent{};

  bool has_encoded_commands() const {
    return encoded_command_mask != 0U;
  }

  bool has_unsupported_commands() const {
    return unsupported_command_mask != 0U;
  }
};

class MhiTxBuilder {
 public:
  static bool build_next_frame(MhiCommandState& command, MhiTxRuntime& runtime, const MhiTxBuildConfig& config,
                               MhiFrameBuffer& out);

  static bool build_next_frame(MhiCommandState& command, MhiTxRuntime& runtime, const MhiTxBuildConfig& config,
                               MhiFrameBuffer& out, MhiTxBuildResult& result);

 private:
  static void initialise_base_frame(MhiFrameBuffer& out, std::size_t frame_size);

  static void apply_opdata_request(MhiFrameBuffer& out, MhiTxRuntime& runtime, uint32_t enabled_opdata_mask);

  static void apply_commands(MhiFrameBuffer& out, MhiCommandState& command, MhiTxRuntime& runtime,
                             const MhiTxBuildConfig& config, MhiTxBuildResult& result);

  static void apply_checksums(MhiFrameBuffer& out);

  static uint8_t encode_mode(uint8_t mode);
  static uint8_t encode_fan(uint8_t fan);
  static uint8_t encode_target_temp(float target_temp_c);
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
