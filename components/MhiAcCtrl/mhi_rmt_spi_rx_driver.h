#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "mhi_defs.h"
#include "mhi_frame_queue.h"
#include "mhi_rx_driver.h"

#ifdef USE_ESP_IDF
#include <sdkconfig.h>
#endif

#if defined(USE_ESP_IDF) && defined(CONFIG_IDF_TARGET_ESP32S3)
#define MHI_RMT_SPI_RX_SUPPORTED 1
#include <driver/gpio_filter.h>
#include <driver/rmt_rx.h>
#include <driver/spi_slave.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <soc/soc_caps.h>
#else
#define MHI_RMT_SPI_RX_SUPPORTED 0
#endif

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiRmtSpiRxConfig {
  uint8_t frame_size_hint{20U};
  uint32_t frame_gap_us{1000U};
};

// ESP32-S3 RX backend for the no-CS MHI bus.
//
// RMT observes SCK and reports the inter-frame idle gap. The RMT callback
// pulses the SPI peripheral's internal CS input, allowing the hardware SPI
// slave and DMA engine to expose each 20/33-byte MHI frame as one transaction.
// MISO remains unclaimed so FastGPIO TX can continue to own that pin.
class MhiRmtSpiRxDriver final : public IMhiRxDriver {
 public:
  ~MhiRmtSpiRxDriver() override;

  void set_config(const MhiRmtSpiRxConfig& config) {
    config_ = config;
  }

  bool setup(const MhiTransportPins& pins) override;
  void loop() override;
  void shutdown() override;
  std::size_t read(uint8_t* dst, std::size_t max_len) override;
  MhiBusMarker bus_marker() const override;

  const char* name() const override {
    return "rmt_spi_rx";
  }

  bool ready() const override {
    return ready_;
  }

 private:
  static constexpr std::size_t kTransactionQueueDepth = 4U;
  static constexpr std::size_t kCompletedFrameQueueDepth = 2U;
  static constexpr std::size_t kDmaCaptureBytes = (kMhiMaxFrameBytes + 3U) & ~std::size_t{3U};
  static constexpr std::size_t kRmtSymbolBufferSize = (kMhiMaxFrameBytes * 8U) + 8U;

#if MHI_RMT_SPI_RX_SUPPORTED
  struct TransactionSlot {
    spi_slave_transaction_t transaction{};
    uint8_t* rx_buffer{nullptr};
  };

  static bool rmt_receive_done_callback_(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t* event_data,
                                         void* user_context);

  bool setup_spi_();
  bool setup_rmt_();
  bool setup_glitch_filter_();
  bool allocate_transaction_buffers_();
  bool queue_transaction_(TransactionSlot& slot);
  void process_completed_transactions_();
  void cleanup_();
  void connect_internal_cs_(bool inactive_high);
  void on_frame_boundary_from_isr_();

  spi_host_device_t host_{SPI2_HOST};
  std::array<TransactionSlot, kTransactionQueueDepth> transaction_slots_{};
  MhiFrameQueue<kCompletedFrameQueueDepth> completed_frames_{};
  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

  rmt_channel_handle_t rmt_channel_{nullptr};
  rmt_receive_config_t rmt_receive_config_{};
  std::array<rmt_symbol_word_t, kRmtSymbolBufferSize> rmt_symbols_{};

#if SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER
  gpio_glitch_filter_handle_t glitch_filter_{nullptr};
#endif

  bool spi_initialized_{false};
  bool rmt_enabled_{false};

  volatile uint32_t marker_sequence_{0U};
  volatile uint32_t marker_frame_end_us_{0U};
  volatile std::size_t marker_frame_len_{0U};
  volatile uint32_t rmt_boundaries_{0U};
  volatile uint32_t rmt_rearm_errors_{0U};
#endif

  MhiRmtSpiRxConfig config_{};
  MhiTransportPins pins_{};
  bool ready_{false};

  uint32_t completed_transactions_{0U};
  uint32_t frames_20_{0U};
  uint32_t frames_33_{0U};
  uint32_t invalid_length_transactions_{0U};
  uint32_t transaction_result_errors_{0U};
  uint32_t transaction_queue_errors_{0U};
  uint32_t dropped_frames_{0U};
  uint32_t last_diag_log_ms_{0U};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
