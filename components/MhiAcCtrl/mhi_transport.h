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

enum class MhiProtocolMode : uint8_t {
  AUTO = 0,
  STANDARD_ONLY = 1,
  EXTENDED_PREFER = 2,
};

struct MhiTransportConfig {
  int sck_pin{-1};
  int mosi_pin{-1};
  int miso_pin{-1};

  // Legacy compatibility hint only. This is no longer the source of truth
  // for protocol parsing, but Phase 0/1 still uses it to size the exchange.
  uint8_t frame_size_hint{20};

  // Upper bound for transport buffers.
  uint8_t max_frame_size{33};

  // Phase 1 contract field. Auto-detection is wired in later phases.
  MhiProtocolMode protocol_mode{MhiProtocolMode::AUTO};

  // Bus framing thresholds.
  uint32_t frame_start_idle_ms{5};
  uint32_t extension_gap_max_us{3000};
};

struct MhiFrameExchangeResult {
  int status{0};
  std::size_t bytes_received{0};
  MhiFrameType detected_type{MhiFrameType::UNKNOWN};
  bool base_frame_complete{false};
  bool extended_tail_present{false};
  bool new_data_packet_received{false};
  uint8_t header_byte{0};
};

class MhiTransport {
 public:
  virtual ~MhiTransport() = default;

  virtual void setup(const MhiTransportConfig &config) = 0;

  virtual MhiFrameExchangeResult exchange_frame(
      const uint8_t *tx_frame,
      uint8_t *rx_frame,
      std::size_t rx_capacity,
      uint32_t max_time_ms) = 0;
};

}  // namespace mhi
}  // namespace esphome
