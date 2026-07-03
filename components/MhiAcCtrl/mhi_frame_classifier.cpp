#include "mhi_frame_classifier.h"

#include "mhi_defs.h"

namespace esphome {
namespace mhi_ac_ctrl {
namespace {

bool mosi_signature_matches(const MhiFrameView& frame) {
  if (!frame.valid()) {
    return false;
  }

  const bool sig0 = frame[SB0] == kMhiMosiSignature0Default || frame[SB0] == kMhiMosiSignature0Alt;
  return sig0 && frame[SB1] == kMhiMosiSignature1 && frame[SB2] == kMhiMosiSignature2;
}

bool opdata_type_0x10(const MhiFrameView& frame) {
  return (frame[DB10] & 0x30U) == 0x10U;
}

bool opdata_type_0x20(const MhiFrameView& frame) {
  return (frame[DB10] & 0x30U) == 0x20U;
}

bool opdata_marker_present(const MhiFrameView& frame) {
  const uint8_t group = frame[DB9];

  if (group == 0x00U || group == 0xFFU) {
    return false;
  }

  // Match the currently accepted opdata decoder shape:
  // - normal observed opdata has DB10[5:4] == 0x10
  // - some high-bank values currently use DB6 bit 7 and DB10[5:4] == 0x20
  // - legacy/synthetic unit coverage also accepts DB6 bit 7.
  return opdata_type_0x10(frame) || opdata_type_0x20(frame) || (frame[DB6] & 0x80U) != 0U;
}

uint16_t opdata_key(const MhiFrameView& frame) {
  const uint16_t bank = (frame[DB6] & 0x80U) != 0U ? 0x8000U : 0x0000U;
  return static_cast<uint16_t>(bank | (static_cast<uint16_t>(frame[DB9]) << 8U) | frame[DB10]);
}

}  // namespace

MhiFrameClassification classify_mhi_mosi_frame(const MhiFrameView& frame) {
  if (!mosi_signature_matches(frame)) {
    return {};
  }

  if (opdata_marker_present(frame)) {
    return MhiFrameClassification{MhiFrameKind::OPDATA, opdata_key(frame)};
  }

  if (frame.is_33_byte() && (frame[DB9] == 0x00U || frame[DB9] == 0xFFU)) {
    return MhiFrameClassification{MhiFrameKind::EXTENDED_STATUS, kMhiInvalidOpDataKey};
  }

  return MhiFrameClassification{MhiFrameKind::STATUS, kMhiInvalidOpDataKey};
}

const char* mhi_frame_kind_to_string(MhiFrameKind kind) {
  switch (kind) {
    case MhiFrameKind::STATUS:
      return "status";
    case MhiFrameKind::EXTENDED_STATUS:
      return "extended_status";
    case MhiFrameKind::OPDATA:
      return "opdata";
    case MhiFrameKind::UNKNOWN:
    default:
      return "unknown";
  }
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
