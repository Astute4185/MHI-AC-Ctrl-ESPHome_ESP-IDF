#include "mhi_protocol_capture.h"

#include <cstdio>

#include "esphome/core/log.h"
#include "mhi_protocol_capture_core.h"

namespace esphome {
namespace mhi_ac_ctrl {
namespace {

static const char* const TAG = "mhi.capture";
MhiProtocolCaptureEngine capture_engine{};

void format_field_name(std::size_t raw_index, std::size_t frame_len, char* out, std::size_t out_len) {
  if (raw_index <= SB2) {
    std::snprintf(out, out_len, "SB%u", static_cast<unsigned int>(raw_index));
    return;
  }
  if (raw_index >= DB0 && raw_index <= DB14) {
    std::snprintf(out, out_len, "DB%u", static_cast<unsigned int>(raw_index - DB0));
    return;
  }
  if (raw_index == CBH) {
    std::snprintf(out, out_len, "CBH");
    return;
  }
  if (raw_index == CBL) {
    std::snprintf(out, out_len, "CBL");
    return;
  }
  if (frame_len == kMhiFrame33Bytes && raw_index >= DB15 && raw_index <= DB26) {
    std::snprintf(out, out_len, "DB%u", static_cast<unsigned int>(15U + (raw_index - static_cast<std::size_t>(DB15))));
    return;
  }
  if (frame_len == kMhiFrame33Bytes && raw_index == CBL2) {
    std::snprintf(out, out_len, "CBL2");
    return;
  }
  std::snprintf(out, out_len, "RAW%u", static_cast<unsigned int>(raw_index));
}

void format_frame_hex(const MhiFrameBuffer& frame, char* out, std::size_t out_len) {
  std::size_t used = 0U;
  for (std::size_t i = 0U; i < frame.len && used < out_len; i++) {
    const int written = std::snprintf(out + used, out_len - used, "%s%02x", i == 0U ? "" : " ",
                                      static_cast<unsigned int>(frame.data[i]));
    if (written <= 0) {
      break;
    }
    const std::size_t advance = static_cast<std::size_t>(written);
    if (advance >= out_len - used) {
      used = out_len - 1U;
      break;
    }
    used += advance;
  }
  out[used] = '\0';
}

void log_full_frame(const MhiFrameBuffer& frame, const char* kind_name, uint16_t opdata_key, uint32_t sequence,
                    const char* reason) {
  (void)kind_name;
  (void)opdata_key;
  (void)sequence;
  (void)reason;
  char frame_hex[128]{};
  format_frame_hex(frame, frame_hex, sizeof(frame_hex));
  ESP_LOGI(TAG, "%s kind=%s key=0x%04x seq=%lu len=%u bytes=%s", reason, kind_name,
           static_cast<unsigned int>(opdata_key), static_cast<unsigned long>(sequence),
           static_cast<unsigned int>(frame.len), frame_hex);
}

}  // namespace

void mhi_protocol_capture_reset() {
  capture_engine.reset();
}

void mhi_protocol_capture_frame(const MhiFrameBuffer& frame, uint8_t kind_id, const char* kind_name,
                                uint16_t opdata_key, uint32_t sequence) {
  if (kind_name == nullptr) {
    return;
  }

  const MhiCaptureObservation observation = capture_engine.observe(frame, kind_id, opdata_key, sequence);
  if (observation.disposition == MhiCaptureDisposition::IGNORED ||
      observation.disposition == MhiCaptureDisposition::UNCHANGED) {
    return;
  }

  if (observation.disposition == MhiCaptureDisposition::BASELINE) {
    log_full_frame(frame, kind_name, opdata_key, sequence, "baseline");
    return;
  }

  char changes[512]{};
  std::size_t used = 0U;
  for (std::size_t i = 0U; i < observation.reported_changes; i++) {
    const MhiCaptureChange& change = observation.changes[i];
    char field[12]{};
    format_field_name(change.raw_index, frame.len, field, sizeof(field));
    const int written =
        std::snprintf(changes + used, sizeof(changes) - used, "%s%s:0x%02x->0x%02x/xor=0x%02x", i == 0U ? "" : ",",
                      field, static_cast<unsigned int>(change.before), static_cast<unsigned int>(change.after),
                      static_cast<unsigned int>(change.xor_mask));
    if (written <= 0 || static_cast<std::size_t>(written) >= sizeof(changes) - used) {
      break;
    }
    used += static_cast<std::size_t>(written);
  }

  if (observation.truncated() && used < sizeof(changes)) {
    std::snprintf(changes + used, sizeof(changes) - used, ",+%u more",
                  static_cast<unsigned int>(observation.total_changes - observation.reported_changes));
  }

  ESP_LOGI(TAG, "change kind=%s key=0x%04x seq=%lu changed=%s", kind_name, static_cast<unsigned int>(opdata_key),
           static_cast<unsigned long>(sequence), changes);
  log_full_frame(frame, kind_name, opdata_key, sequence, "frame");
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
