#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mhi_defs.h"
#include "mhi_rx_driver.h"

#ifdef USE_ESP_IDF
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#endif

namespace esphome {
namespace mhi_ac_ctrl {

enum class MhiExternalClockSampleEdge : uint8_t { RISING = 0U, FALLING = 1U };

struct MhiExternalClockRxConfig {
  uint8_t frame_size_hint{20};
  uint32_t byte_gap_reset_us{80};
  uint32_t frame_gap_reset_us{5000};
  uint32_t min_edge_gap_us{4};
  MhiExternalClockSampleEdge sample_edge{MhiExternalClockSampleEdge::FALLING};
  uint32_t sample_delay_nops{0};
};

// External-clock MOSI sampler for the no-CS MHI bus.
//
// This is the first safe RX probe after the native SPI-slave failure. It does
// not use the ESP-IDF SPI slave transaction model. It samples MOSI on the configured SCK edge, reconstructs LSB-first
// bytes, and emits signature-anchored frame chunks.
class MhiExternalClockRxDriver final : public IMhiRxDriver {
 public:
  void set_config(const MhiExternalClockRxConfig& config) {
    config_ = config;
  }

  bool setup(const MhiTransportPins& pins) override;
  void loop() override;
  std::size_t read(uint8_t* dst, std::size_t max_len) override;

  const char* name() const override {
    return "external_clock_rx";
  }
  bool ready() const override {
    return ready_;
  }

  uint32_t edges() const {
    return edges_;
  }
  uint32_t bytes() const {
    return bytes_;
  }
  uint32_t byte_gap_resets() const {
    return byte_gap_resets_;
  }
  uint32_t frame_gap_resets() const {
    return frame_gap_resets_;
  }
  uint32_t short_edges() const {
    return short_edges_;
  }
  uint32_t dropped_bytes() const {
    return dropped_bytes_;
  }

 private:
  static constexpr std::size_t kRingFrames = 128U;
  static constexpr std::size_t kRingBytes = kMhiMaxFrameBytes * kRingFrames;

#ifdef USE_ESP_IDF
  static void isr_trampoline_(void* arg);
  void handle_sck_edge_();
  void apply_sample_delay_from_isr_();
  void append_byte_from_isr_(uint8_t value);
  void push_current_frame_from_isr_();
  void discard_current_frame_from_isr_();
  void reset_signature_window_from_isr_();
  bool update_signature_window_from_isr_(uint8_t value);
  void start_frame_from_signature_window_from_isr_();
  std::size_t ring_available_() const;
  std::size_t ring_free_() const;
  std::size_t expected_frame_size_() const;
  const char* sample_edge_name_() const;
#endif

  MhiExternalClockRxConfig config_{};
  MhiTransportPins pins_{};
  bool ready_{false};

  uint32_t edges_{0};
  uint32_t bytes_{0};
  uint32_t byte_gap_resets_{0};
  uint32_t frame_gap_resets_{0};
  uint32_t full_byte_gap_resets_{0};
  uint32_t partial_byte_gap_resets_{0};
  uint32_t short_edges_{0};
  uint32_t dropped_bytes_{0};
  uint32_t emitted_frame_chunks_{0};
  uint32_t discarded_partial_frames_{0};
  uint32_t dropped_frame_chunks_{0};
  uint32_t peak_buffered_bytes_{0};
  uint32_t signature_starts_{0};
  uint32_t signature_restarts_during_frame_{0};
  uint32_t discarded_presync_bytes_{0};
  uint32_t bad_length_discards_{0};
  uint32_t last_diag_log_ms_{0};
  uint32_t min_observed_gap_us_{0};
  uint32_t max_observed_gap_us_{0};
  uint32_t last_gap_us_{0};

#ifdef USE_ESP_IDF
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  std::array<uint8_t, kRingBytes> ring_{};
  volatile std::size_t ring_head_{0};
  volatile std::size_t ring_tail_{0};

  std::array<uint8_t, kMhiMaxFrameBytes> current_frame_{};
  volatile std::size_t current_frame_len_{0};

  std::array<uint8_t, 3U> signature_window_{};
  volatile uint8_t signature_window_len_{0};

  volatile uint8_t current_byte_{0};
  volatile uint8_t bit_index_{0};
  volatile int64_t last_edge_us_{0};
#endif
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
