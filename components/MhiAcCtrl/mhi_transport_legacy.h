#pragma once

#include "mhi_transport.h"

namespace esphome {
namespace mhi {

class MhiTransportLegacy : public MhiTransport {
 public:
  void setup(const MhiTransportConfig &config) override;

  int exchange_frame(
      const uint8_t *tx_frame,
      uint8_t *rx_frame,
      std::size_t frame_size,
      uint32_t max_time_ms,
      bool &new_data_packet_received) override;

  void on_frame_result(bool valid_frame, int error_code) override;

  const MhiTimingProfile &timing_profile() const override { return this->timing_; }
  const MhiTransportStats &stats() const override { return this->stats_; }

 private:
  bool overall_timed_out_(uint32_t start_us, uint32_t max_time_ms) const;
  void reset_calibration_();
  void update_frame_observation_(
      uint32_t stable_high_us,
      uint32_t max_high_wait_us,
      uint32_t max_low_wait_us,
      uint32_t max_byte_us);
  void finalize_calibration_();

  MhiTransportConfig config_{};
  MhiTimingProfile timing_{};
  MhiTransportStats stats_{};

  uint32_t last_observed_stable_high_us_{0};
  uint32_t last_observed_max_high_wait_us_{0};
  uint32_t last_observed_max_low_wait_us_{0};
  uint32_t last_observed_max_byte_us_{0};
  bool last_exchange_completed_{false};
};

}  // namespace mhi
}  // namespace esphome