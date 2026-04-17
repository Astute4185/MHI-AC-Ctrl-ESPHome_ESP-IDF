#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace mhi {

enum class MhiFrameType : uint8_t {
  UNKNOWN = 0,
  STANDARD_20 = 20,
  EXTENDED_33 = 33,
};

struct MhiTransportConfig {
  int sck_pin{-1};
  int mosi_pin{-1};
  int miso_pin{-1};
  uint8_t frame_size_hint{20};
  uint32_t frame_start_idle_ms{5};
  uint32_t extension_gap_max_us{3000};
};

struct MhiFrameExchangeResult {
  int status{0};
  std::size_t bytes_received{0};
  MhiFrameType detected_type{MhiFrameType::UNKNOWN};
  bool new_data_packet_received{false};
  uint8_t header_byte{0};
  bool extension_probe_attempted{false};
  bool extension_start_seen{false};

  // Transport hardening / diagnostics.
  bool critical_capture_used{false};
  uint8_t overcapture_bytes[2]{0, 0};
  uint8_t overcapture_len{0};
  bool next_frame_signature_after_tail{false};
};

class MhiTransport {
 public:
  void setup(const MhiTransportConfig &config);

  MhiFrameExchangeResult exchange_frame(
      const uint8_t *tx_frame,
      uint8_t *rx_frame,
      std::size_t rx_capacity,
      uint32_t max_time_ms);

 private:
  MhiTransportConfig config_{};
};

}  // namespace mhi
}  // namespace esphome
