#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace mhi {

enum class MhiProtocolMode : uint8_t {
  AUTO = 0,
  STANDARD_ONLY = 1,
  EXTENDED_PREFER = 2,
};

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
  MhiProtocolMode protocol_mode{MhiProtocolMode::AUTO};
  uint32_t frame_idle_min_ms{5};
  uint32_t extension_gap_timeout_us{3000};
};

struct MhiFrameExchangeResult {
  int status{0};
  std::size_t bytes_received{0};
  MhiFrameType detected_type{MhiFrameType::UNKNOWN};
  bool new_data_packet_received{false};
  bool header_valid{false};
  bool base_frame_complete{false};
  bool extended_tail_present{false};
};

class MhiTransport {
 public:
  virtual ~MhiTransport() = default;

  virtual void setup(const MhiTransportConfig &config) = 0;

  virtual MhiFrameExchangeResult exchange_frame(
      const uint8_t *tx_frame,
      std::size_t tx_frame_size,
      uint8_t *rx_frame,
      std::size_t rx_frame_capacity,
      uint32_t max_time_ms) = 0;
};

}  // namespace mhi
}  // namespace esphome
