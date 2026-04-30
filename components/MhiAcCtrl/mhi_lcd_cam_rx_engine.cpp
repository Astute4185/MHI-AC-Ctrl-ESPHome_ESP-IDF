#include "mhi_lcd_cam_rx_engine.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_timer.h>

#include "MHI-AC-Ctrl-core.h"
#include "esphome/core/log.h"
#include "mhi_frame_layout.h"
#include "mhi_time.h"

#if defined(ESP32) || defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) ||     defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3) ||     defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32H2)
#include "soc/gpio_struct.h"
#define MHI_LCD_CAM_RX_USE_FAST_GPIO 1
#else
#define MHI_LCD_CAM_RX_USE_FAST_GPIO 0
#endif

namespace esphome {
namespace mhi {

namespace {

static const char *const TAG = "mhi.lcdcam";
constexpr uint32_t kTimeoutCheckMask = 0x3F;
constexpr std::size_t kBaseFrameBytes = 20U;
constexpr std::size_t kExtendedFrameBytes = 33U;
constexpr std::size_t kMaxHeadPreviewBytes = 32U;
constexpr std::size_t kTailPreviewBytes = 8U;
constexpr std::size_t kFocusByteCount = 8U;
constexpr std::size_t kFocusIndices[kFocusByteCount] = {18U, 19U, 20U, 21U, 22U, 27U, 31U, 32U};
constexpr uint32_t kMinValidExtensionGapUs = 150U;

#if MHI_LCD_CAM_RX_USE_FAST_GPIO
inline bool IRAM_ATTR fast_gpio_read(int pin) {
  if (pin < 32) {
    return (::GPIO.in >> pin) & 0x1;
  }
  return (::GPIO.in1.val >> (pin - 32)) & 0x1;
}

inline void IRAM_ATTR fast_gpio_write_high(int pin) {
  if (pin < 32) {
    ::GPIO.out_w1ts = (1UL << pin);
  } else {
    ::GPIO.out1_w1ts.val = (1UL << (pin - 32));
  }
}

inline void IRAM_ATTR fast_gpio_write_low(int pin) {
  if (pin < 32) {
    ::GPIO.out_w1tc = (1UL << pin);
  } else {
    ::GPIO.out1_w1tc.val = (1UL << (pin - 32));
  }
}
#else
inline bool IRAM_ATTR fast_gpio_read(int pin) {
  return gpio_get_level(static_cast<gpio_num_t>(pin)) != 0;
}

inline void IRAM_ATTR fast_gpio_write_high(int pin) {
  gpio_set_level(static_cast<gpio_num_t>(pin), 1);
}

inline void IRAM_ATTR fast_gpio_write_low(int pin) {
  gpio_set_level(static_cast<gpio_num_t>(pin), 0);
}
#endif

inline bool IRAM_ATTR timed_out(uint32_t start_ms, uint32_t max_time_ms, uint32_t &spin_counter) {
  spin_counter++;
  if ((spin_counter & kTimeoutCheckMask) != 0U) {
    return false;
  }
  return (mhi_now_ms() - start_ms) > max_time_ms;
}

inline uint64_t now_us() {
  return static_cast<uint64_t>(esp_timer_get_time());
}

inline bool has_valid_header_prefix(const uint8_t *frame, std::size_t len) {
  return len >= 3U && (frame[0] == 0x6C || frame[0] == 0x6D) && frame[1] == 0x80 && frame[2] == 0x04;
}

inline bool has_valid_base_checksum(const uint8_t *frame, std::size_t len) {
  return len >= kBaseFrameBytes &&
         static_cast<uint16_t>((static_cast<uint16_t>(frame[CBH]) << 8) | frame[CBL]) == mhi_calc_checksum(frame);
}

inline bool has_valid_extended_checksum(const uint8_t *frame, std::size_t len) {
  return len >= kExtendedFrameBytes &&
         frame[CBL2] == static_cast<uint8_t>(mhi_calc_checksum_frame33(frame) & 0xFFU);
}

inline uint8_t reverse_bits(uint8_t value) {
  value = static_cast<uint8_t>(((value & 0xF0U) >> 4) | ((value & 0x0FU) << 4));
  value = static_cast<uint8_t>(((value & 0xCCU) >> 2) | ((value & 0x33U) << 2));
  value = static_cast<uint8_t>(((value & 0xAAU) >> 1) | ((value & 0x55U) << 1));
  return value;
}

enum PackMode : uint8_t {
  PACK_IDENTITY = 0,
  PACK_BIT_REVERSED = 1,
  PACK_INVERTED = 2,
  PACK_BIT_REVERSED_INVERTED = 3,
};

const char *pack_mode_name(uint8_t mode) {
  switch (mode) {
    case PACK_IDENTITY:
      return "identity";
    case PACK_BIT_REVERSED:
      return "bit_reverse";
    case PACK_INVERTED:
      return "invert";
    case PACK_BIT_REVERSED_INVERTED:
      return "bit_reverse_invert";
    default:
      return "unknown";
  }
}

bool detect_header_candidate(const uint8_t *frame, std::size_t len, uint8_t &pack_mode) {
  if (len < 3U) {
    pack_mode = PACK_IDENTITY;
    return false;
  }

  const uint8_t candidates[4][3] = {
      {frame[0], frame[1], frame[2]},
      {reverse_bits(frame[0]), reverse_bits(frame[1]), reverse_bits(frame[2])},
      {static_cast<uint8_t>(~frame[0]), static_cast<uint8_t>(~frame[1]), static_cast<uint8_t>(~frame[2])},
      {static_cast<uint8_t>(~reverse_bits(frame[0])), static_cast<uint8_t>(~reverse_bits(frame[1])), static_cast<uint8_t>(~reverse_bits(frame[2]))},
  };

  for (uint8_t mode = PACK_IDENTITY; mode <= PACK_BIT_REVERSED_INVERTED; mode++) {
    if ((candidates[mode][0] == 0x6C || candidates[mode][0] == 0x6D) &&
        candidates[mode][1] == 0x80 && candidates[mode][2] == 0x04) {
      pack_mode = mode;
      return true;
    }
  }

  pack_mode = PACK_IDENTITY;
  return false;
}

int IRAM_ATTR read_one_byte(
    int sck_pin,
    int mosi_pin,
    int miso_pin,
    uint8_t tx_byte,
    bool tx_suppress_during_capture,
    uint8_t *rx_byte,
    uint32_t start_ms,
    uint32_t max_time_ms,
    bool first_falling_edge_already_seen) {
  uint8_t mosi_byte = 0;
  uint8_t bit_mask = 1;
  uint32_t byte_spin_counter = 0;

  for (uint8_t bit_cnt = 0; bit_cnt < 8; bit_cnt++) {
    if (!(first_falling_edge_already_seen && bit_cnt == 0U)) {
      while (fast_gpio_read(sck_pin)) {
        if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
          fast_gpio_write_low(miso_pin);
          return err_msg_timeout_SCK_high;
        }
      }
    }

    const uint8_t driven_tx_byte = tx_suppress_during_capture ? 0U : tx_byte;

    if ((driven_tx_byte & bit_mask) != 0U) {
      fast_gpio_write_high(miso_pin);
    } else {
      fast_gpio_write_low(miso_pin);
    }

    while (!fast_gpio_read(sck_pin)) {
      if (timed_out(start_ms, max_time_ms, byte_spin_counter)) {
        fast_gpio_write_low(miso_pin);
        return err_msg_timeout_SCK_low;
      }
    }

    if (fast_gpio_read(mosi_pin)) {
      mosi_byte = static_cast<uint8_t>(mosi_byte + bit_mask);
    }

    bit_mask = static_cast<uint8_t>(bit_mask << 1);
  }

  fast_gpio_write_low(miso_pin);
  *rx_byte = mosi_byte;
  return 0;
}

int IRAM_ATTR read_frame_range(
    int sck_pin,
    int mosi_pin,
    int miso_pin,
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t start_index,
    std::size_t end_index,
    uint32_t start_ms,
    uint32_t max_time_ms,
    bool first_falling_edge_already_seen,
    bool *new_data_packet_received,
    uint8_t *header_byte,
    bool tx_suppress_during_capture) {
  for (std::size_t byte_cnt = start_index; byte_cnt < end_index; byte_cnt++) {
    uint8_t rx_byte = 0;
    const int rc = read_one_byte(
        sck_pin,
        mosi_pin,
        miso_pin,
        tx_frame[byte_cnt],
        tx_suppress_during_capture,
        &rx_byte,
        start_ms,
        max_time_ms,
        first_falling_edge_already_seen && byte_cnt == start_index);
    if (rc < 0) {
      return rc;
    }

    if (rx_frame[byte_cnt] != rx_byte) {
      *new_data_packet_received = true;
      rx_frame[byte_cnt] = rx_byte;
    }

    if (byte_cnt == 0U && header_byte != nullptr) {
      *header_byte = rx_byte;
    }
  }

  return 0;
}

bool wait_for_idle_gap_us(int sck_pin, uint32_t gap_us, uint32_t start_ms, uint32_t max_time_ms) {
  const uint64_t begin_us = now_us();
  uint64_t high_start_us = 0;
  uint32_t spin_counter = 0;

  while ((mhi_now_ms() - start_ms) <= max_time_ms) {
    if (fast_gpio_read(sck_pin)) {
      if (high_start_us == 0U) {
        high_start_us = now_us();
      }
      if ((now_us() - high_start_us) >= static_cast<uint64_t>(gap_us)) {
        return true;
      }
    } else {
      high_start_us = 0U;
    }

    if (timed_out(start_ms, max_time_ms, spin_counter)) {
      break;
    }
    if ((now_us() - begin_us) > static_cast<uint64_t>(max_time_ms) * 1000ULL) {
      break;
    }
  }
  return false;
}

bool wait_for_next_falling_edge_us(int sck_pin, uint32_t timeout_us, uint32_t start_ms, uint32_t max_time_ms) {
  const uint64_t begin_us = now_us();
  while (fast_gpio_read(sck_pin)) {
    if ((now_us() - begin_us) > static_cast<uint64_t>(timeout_us)) {
      return false;
    }
    if ((mhi_now_ms() - start_ms) > max_time_ms) {
      return false;
    }
  }
  return true;
}

void format_preview_hex(const uint8_t *src, std::size_t len, char *out, std::size_t out_len) {
  if (out_len == 0U) {
    return;
  }
  out[0] = '\0';
  std::size_t pos = 0;
  for (std::size_t i = 0; i < len && (pos + 4U) < out_len; i++) {
    const int written = std::snprintf(out + pos, out_len - pos, "%02X%s", src[i], (i + 1U < len) ? " " : "");
    if (written <= 0) {
      break;
    }
    pos += static_cast<std::size_t>(written);
  }
  out[out_len - 1U] = '\0';
}

void format_focus_hex(const uint8_t *src, std::size_t len, char *out, std::size_t out_len) {
  if (out_len == 0U) {
    return;
  }
  out[0] = '\0';
  std::size_t pos = 0;
  for (std::size_t i = 0; i < kFocusByteCount && (pos + 4U) < out_len; i++) {
    const std::size_t index = kFocusIndices[i];
    const uint8_t value = index < len ? src[index] : 0U;
    const int written = std::snprintf(out + pos, out_len - pos, "%02X%s", value, (i + 1U < kFocusByteCount) ? " " : "");
    if (written <= 0) {
      break;
    }
    pos += static_cast<std::size_t>(written);
  }
  out[out_len - 1U] = '\0';
}

}  // namespace

bool MhiLcdCamRxEngine::setup(const MhiTransportConfig &config) {
  this->config_ = config;

  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.sck_pin));
  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.mosi_pin));
  gpio_reset_pin(static_cast<gpio_num_t>(this->config_.miso_pin));

  gpio_set_direction(static_cast<gpio_num_t>(this->config_.sck_pin), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(this->config_.mosi_pin), GPIO_MODE_INPUT);
  gpio_set_direction(static_cast<gpio_num_t>(this->config_.miso_pin), GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(this->config_.miso_pin), 0);

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
  ESP_LOGW(TAG, "lcd_cam_rx backend is experimental and currently intended for ESP32-S3; continuing with GPIO-backed sampler");
#endif

  ESP_LOGCONFIG(
      TAG,
      "configured experimental rx engine: sck=%d mosi=%d miso=%d gap_us=%u ext_gap_min_us=%u tx_suppress=%s dump=%s rate_ms=%u preview=%u",
      this->config_.sck_pin,
      this->config_.mosi_pin,
      this->config_.miso_pin,
      this->config_.sync_gap_us,
      kMinValidExtensionGapUs,
      this->config_.tx_suppress_during_capture ? "on" : "off",
      this->config_.raw_dump_enable ? "on" : "off",
      this->config_.raw_dump_rate_ms,
      this->config_.raw_chunk_bytes);
  return true;
}

MhiFrameExchangeResult MhiLcdCamRxEngine::exchange_frame(
    const uint8_t *tx_frame,
    uint8_t *rx_frame,
    std::size_t rx_capacity,
    uint32_t max_time_ms) {
  MhiFrameExchangeResult result{};
  result.backend_used = MhiTransportBackend::LCD_CAM_RX;
  result.critical_capture_used = true;

  const std::size_t target_frame_size =
      std::min<std::size_t>(this->config_.frame_size_hint == 33 ? kExtendedFrameBytes : kBaseFrameBytes, rx_capacity);
  if (target_frame_size < kBaseFrameBytes) {
    result.status = err_msg_timeout_SCK_low;
    return result;
  }

  const uint32_t start_ms = mhi_now_ms();
  const int sck_pin = this->config_.sck_pin;
  const int mosi_pin = this->config_.mosi_pin;
  const int miso_pin = this->config_.miso_pin;

  if (!wait_for_idle_gap_us(sck_pin, this->config_.sync_gap_us, start_ms, max_time_ms)) {
    result.status = err_msg_timeout_SCK_low;
    return result;
  }

  int rc = read_frame_range(
      sck_pin,
      mosi_pin,
      miso_pin,
      tx_frame,
      rx_frame,
      0U,
      kBaseFrameBytes,
      start_ms,
      max_time_ms,
      false,
      &result.new_data_packet_received,
      &result.header_byte,
      this->config_.tx_suppress_during_capture);
  const bool base_header_ok = (rc >= 0) && has_valid_header_prefix(rx_frame, kBaseFrameBytes);
  const bool base_checksum_ok = base_header_ok && has_valid_base_checksum(rx_frame, kBaseFrameBytes);
  bool base_gate_passed = base_header_ok && base_checksum_ok;
  bool extension_stage_complete = false;
  bool extension_gap_rejected = false;
  bool provisional_extension_rejected = false;
  uint32_t extension_gap_us = 0U;
  if (rc < 0) {
    result.status = rc;
    result.raw_chunk_len = 0;
    this->maybe_log_chunk_(rx_frame, 0U, false, rc, false, PACK_IDENTITY, false, false, false, false, false, 0U);
    return result;
  }

  result.bytes_received = kBaseFrameBytes;
  result.raw_chunk_len = kBaseFrameBytes;
  result.detected_type = MhiFrameType::STANDARD_20;
  result.status = 0;

  if (target_frame_size >= kExtendedFrameBytes) {
    result.extension_probe_attempted = true;
    if (base_gate_passed) {
      this->base_gate_pass_count_++;
      const uint64_t extension_wait_begin_us = now_us();
      const bool extension_seen = wait_for_next_falling_edge_us(
          sck_pin,
          this->config_.extension_gap_max_us,
          start_ms,
          max_time_ms);
      extension_gap_us = static_cast<uint32_t>(now_us() - extension_wait_begin_us);
      this->last_extension_gap_us_ = extension_gap_us;
      if (extension_seen) {
        this->extension_probe_seen_count_++;
        result.extension_start_seen = true;
        if (extension_gap_us < kMinValidExtensionGapUs) {
          extension_gap_rejected = true;
          this->extension_gap_short_count_++;
          this->extension_gap_reject_count_++;
          result.extension_start_seen = false;
          result.bytes_received = kBaseFrameBytes;
          result.raw_chunk_len = kBaseFrameBytes;
          result.detected_type = MhiFrameType::STANDARD_20;
          result.frame_suppressed = true;
          result.new_data_packet_received = false;
        } else {
          this->extension_gap_good_count_++;
          rc = read_frame_range(
              sck_pin,
              mosi_pin,
              miso_pin,
              tx_frame,
              rx_frame,
              kBaseFrameBytes,
              kExtendedFrameBytes,
              start_ms,
              max_time_ms,
              true,
              &result.new_data_packet_received,
              nullptr,
              this->config_.tx_suppress_during_capture);
          if (rc < 0) {
            result.status = rc;
            result.raw_chunk_len = kBaseFrameBytes;
            this->maybe_log_chunk_(rx_frame, kBaseFrameBytes, true, rc, false, PACK_IDENTITY, base_gate_passed, false, false, false, false, extension_gap_us);
            return result;
          }
          extension_stage_complete = true;
          this->extension_complete_count_++;
          result.bytes_received = kExtendedFrameBytes;
          result.raw_chunk_len = kExtendedFrameBytes;
          result.detected_type = MhiFrameType::EXTENDED_33;
          if (!has_valid_header_prefix(rx_frame, kExtendedFrameBytes) || !has_valid_extended_checksum(rx_frame, kExtendedFrameBytes)) {
            provisional_extension_rejected = true;
            this->extension_publish_reject_count_++;
            result.frame_suppressed = true;
            result.new_data_packet_received = false;
          }
        }
      } else {
        this->extension_probe_timeout_count_++;
      }
    } else {
      this->extension_skipped_bad_base_count_++;
    }
  }

  uint8_t pack_mode = PACK_IDENTITY;
  const bool header_candidate = detect_header_candidate(rx_frame, result.bytes_received, pack_mode);
  result.pack_mode = pack_mode;
  result.header_candidate_seen = header_candidate;

  this->capture_counter_++;
  const bool valid_header = has_valid_header_prefix(rx_frame, result.bytes_received);
  if (!extension_gap_rejected && !provisional_extension_rejected && !result.frame_suppressed) {
    this->update_reference_stats_(rx_frame, result.bytes_received, valid_header);
  } else {
    this->first_bad_index_ = -1;
    this->last_bad_index_ = -1;
    this->last_good_index_ = valid_header ? static_cast<int>(result.bytes_received) - 1 : -1;
    this->mismatch_count_ = 0;
  }
  if (!valid_header && !result.frame_suppressed) {
    this->bad_capture_counter_++;
  }

  this->maybe_log_chunk_(
      rx_frame,
      result.bytes_received,
      result.extension_start_seen,
      result.status,
      header_candidate,
      pack_mode,
      base_gate_passed,
      extension_stage_complete,
      extension_gap_rejected,
      provisional_extension_rejected,
      result.frame_suppressed,
      extension_gap_us);

  fast_gpio_write_low(miso_pin);
  return result;
}

void MhiLcdCamRxEngine::maybe_log_chunk_(
    const uint8_t *frame,
    std::size_t len,
    bool extension_seen,
    int status,
    bool header_candidate,
    uint8_t pack_mode,
    bool base_gate_passed,
    bool extension_stage_complete,
    bool extension_gap_rejected,
    bool provisional_extension_rejected,
    bool frame_suppressed,
    uint32_t extension_gap_us) {
  const uint32_t now_ms = mhi_now_ms();
  const bool valid_header = has_valid_header_prefix(frame, len);
  const bool force_log = this->config_.raw_dump_enable || !valid_header || status < 0 || extension_gap_rejected || provisional_extension_rejected || frame_suppressed;

  if (!force_log) {
    return;
  }
  if ((now_ms - this->last_dump_ms_) < this->config_.raw_dump_rate_ms) {
    return;
  }
  this->last_dump_ms_ = now_ms;

  const std::size_t preview_len = std::min<std::size_t>(len, std::min<std::size_t>(this->config_.raw_chunk_bytes, kMaxHeadPreviewBytes));
  const std::size_t tail_len = std::min<std::size_t>(len, kTailPreviewBytes);
  char head_hex[160];
  char tail_hex[64];
  char focus_hex[64];
  format_preview_hex(frame, preview_len, head_hex, sizeof(head_hex));
  format_preview_hex(len > tail_len ? frame + (len - tail_len) : frame, tail_len, tail_hex, sizeof(tail_hex));
  format_focus_hex(frame, len, focus_hex, sizeof(focus_hex));

  ESP_LOGW(
      TAG,
      "chunk seq=%u bad=%u len=%u ext=%u status=%d hdr=%02X valid=%u candidate=%u pack=%s base_ok=%u ext_done=%u ext_reject=%u ext_pub_reject=%u suppress=%u ext_gap_us=%u first_bad=%d last_good=%d last_bad=%d mismatches=%u post20=%u post31=%u tail_only=%u tx_suppress=%u focus=%s head=%s tail=%s",
      this->capture_counter_,
      this->bad_capture_counter_,
      static_cast<unsigned>(len),
      extension_seen ? 1U : 0U,
      status,
      len > 0U ? frame[0] : 0U,
      valid_header ? 1U : 0U,
      header_candidate ? 1U : 0U,
      pack_mode_name(pack_mode),
      base_gate_passed ? 1U : 0U,
      extension_stage_complete ? 1U : 0U,
      extension_gap_rejected ? 1U : 0U,
      provisional_extension_rejected ? 1U : 0U,
      frame_suppressed ? 1U : 0U,
      extension_gap_us,
      this->first_bad_index_,
      this->last_good_index_,
      this->last_bad_index_,
      this->mismatch_count_,
      (this->mismatch_count_ > 0U && this->first_bad_index_ >= 20) ? 1U : 0U,
      (this->mismatch_count_ > 0U && this->first_bad_index_ >= 31) ? 1U : 0U,
      (this->mismatch_count_ > 0U && this->first_bad_index_ >= static_cast<int>(len >= 2U ? len - 2U : 0U)) ? 1U : 0U,
      this->config_.tx_suppress_during_capture ? 1U : 0U,
      focus_hex,
      head_hex,
      tail_hex);

  if (valid_header && this->has_reference_) {
    ESP_LOGW(
        TAG,
        "stats ref_len=%u same=%u post20_total=%u post31_total=%u tail_only_total=%u base_ok_total=%u ext_seen_total=%u ext_done_total=%u ext_timeout_total=%u ext_skipped_total=%u ext_gap_good_total=%u ext_gap_short_total=%u ext_gap_reject_total=%u ext_publish_reject_total=%u ext_gap_min_us=%u focus18/19/20/21/22/27/31/32=%u/%u/%u/%u/%u/%u/%u/%u last_gap_us=%u",
        static_cast<unsigned>(this->reference_len_),
        this->identical_to_reference_count_,
        this->post20_corrupt_count_,
        this->post31_corrupt_count_,
        this->tail_only_corrupt_count_,
        this->base_gate_pass_count_,
        this->extension_probe_seen_count_,
        this->extension_complete_count_,
        this->extension_probe_timeout_count_,
        this->extension_skipped_bad_base_count_,
        this->extension_gap_good_count_,
        this->extension_gap_short_count_,
        this->extension_gap_reject_count_,
        this->extension_publish_reject_count_,
        kMinValidExtensionGapUs,
        this->boundary_focus_mismatch_counts_[18],
        this->boundary_focus_mismatch_counts_[19],
        this->boundary_focus_mismatch_counts_[20],
        this->boundary_focus_mismatch_counts_[21],
        this->boundary_focus_mismatch_counts_[22],
        this->boundary_focus_mismatch_counts_[27],
        this->boundary_focus_mismatch_counts_[31],
        this->boundary_focus_mismatch_counts_[32],
        this->last_extension_gap_us_);
  }
}

void MhiLcdCamRxEngine::update_reference_stats_(const uint8_t *frame, std::size_t len, bool valid_header) {
  this->first_bad_index_ = -1;
  this->last_bad_index_ = -1;
  this->last_good_index_ = valid_header ? static_cast<int>(len) - 1 : -1;
  this->mismatch_count_ = 0;

  if (!valid_header || len == 0U) {
    return;
  }

  if (!this->has_reference_ || this->reference_len_ != len) {
    std::memcpy(this->reference_frame_, frame, len);
    this->reference_len_ = len;
    this->has_reference_ = true;
    this->identical_to_reference_count_ = 1;
    return;
  }

  for (std::size_t i = 0; i < len; i++) {
    if (this->reference_frame_[i] != frame[i]) {
      if (this->first_bad_index_ < 0) {
        this->first_bad_index_ = static_cast<int>(i);
        this->last_good_index_ = static_cast<int>(i) - 1;
      }
      this->last_bad_index_ = static_cast<int>(i);
      this->mismatch_count_++;
      if (i < 33U) {
        this->boundary_focus_mismatch_counts_[i]++;
      }
    }
  }

  if (this->mismatch_count_ == 0U) {
    this->identical_to_reference_count_++;
    this->last_good_index_ = static_cast<int>(len) - 1;
    return;
  }

  if (this->first_bad_index_ >= 20) {
    this->post20_corrupt_count_++;
  }
  if (this->first_bad_index_ >= 31) {
    this->post31_corrupt_count_++;
  }
  if (this->first_bad_index_ >= static_cast<int>(len >= 2U ? len - 2U : 0U)) {
    this->tail_only_corrupt_count_++;
  }
}

}  // namespace mhi
}  // namespace esphome