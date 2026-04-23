#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace mhi {

class MhiLcdCamRxEngine;

enum class MhiFrameType : uint8_t {
  UNKNOWN = 0,
  STANDARD_20 = 20,
  EXTENDED_33 = 33,
};

enum class MhiTransportBackend : uint8_t {
  GPIO = 0,
  LCD_CAM_RX = 1,
};

// ESPHome's Python enum codegen may emit namespace-scoped names like mhi::GPIO
// and mhi::LCD_CAM_RX into generated main.cpp. Keep aliases here so both the
// generated form and the scoped enum-class form compile cleanly.
inline constexpr MhiTransportBackend GPIO = MhiTransportBackend::GPIO;
inline constexpr MhiTransportBackend LCD_CAM_RX = MhiTransportBackend::LCD_CAM_RX;

const char *mhi_transport_backend_name(MhiTransportBackend backend);

struct MhiTransportConfig {
  int sck_pin{-1};
  int mosi_pin{-1};
  int miso_pin{-1};
  uint8_t frame_size_hint{20};
  uint32_t frame_start_idle_ms{5};
  uint32_t extension_gap_max_us{3000};
  MhiTransportBackend backend{MhiTransportBackend::GPIO};

  // Experimental extclk / LCD-CAM style receive path diagnostics.
  bool raw_dump_enable{false};
  uint32_t raw_dump_rate_ms{15000};
  uint32_t raw_chunk_bytes{24};
  uint32_t sync_gap_us{5000};
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

  // Experimental backend diagnostics.
  MhiTransportBackend backend_used{MhiTransportBackend::GPIO};
  uint32_t raw_chunk_len{0};
  uint8_t pack_mode{0};
  bool header_candidate_seen{false};
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
  MhiLcdCamRxEngine *lcd_cam_rx_engine_{nullptr};
};

}  // namespace mhi
}  // namespace esphome