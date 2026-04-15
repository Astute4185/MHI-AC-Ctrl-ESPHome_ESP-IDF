#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace mhi {

struct MhiTransportConfig {
  int sck_pin{-1};
  int mosi_pin{-1};
  int miso_pin{-1};
  uint8_t frame_size{20};
};

struct MhiTimingProfile {
  bool auto_calibrate{true};
  bool calibration_complete{false};

  uint32_t frame_start_high_us{5000};
  uint32_t sck_high_timeout_us{4000};
  uint32_t sck_low_timeout_us{4000};
  uint32_t byte_timeout_us{30000};

  uint32_t min_phase_timeout_us{100};
  uint32_t max_phase_timeout_us{15000};
  uint32_t min_byte_timeout_us{2000};
  uint32_t max_byte_timeout_us{60000};

  uint8_t valid_frames_needed{10};
  uint8_t recalibrate_after_consecutive_errors{20};
};

struct MhiTransportStats {
  uint32_t frames_total{0};
  uint32_t frames_valid{0};
  uint32_t frames_invalid_signature{0};
  uint32_t frames_invalid_checksum{0};
  uint32_t timeout_sck_high{0};
  uint32_t timeout_sck_low{0};
  uint32_t timeout_byte{0};

  uint32_t max_observed_high_us{0};
  uint32_t max_observed_low_us{0};
  uint32_t max_observed_byte_us{0};
  uint32_t max_observed_frame_start_high_us{0};

  uint32_t consecutive_errors{0};
};

class MhiTransport {
 public:
  virtual ~MhiTransport() = default;

  virtual void setup(const MhiTransportConfig &config) = 0;

  virtual int exchange_frame(
      const uint8_t *tx_frame,
      uint8_t *rx_frame,
      std::size_t frame_size,
      uint32_t max_time_ms,
      bool &new_data_packet_received) = 0;

  virtual void on_frame_result(bool valid_frame, int error_code) = 0;
  virtual const MhiTimingProfile &timing_profile() const = 0;
  virtual const MhiTransportStats &stats() const = 0;
};

}  // namespace mhi
}  // namespace esphome