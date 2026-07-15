#include "mhi_rmt_spi_rx_driver.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#if MHI_RMT_SPI_RX_SUPPORTED
#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_rom_gpio.h>
#include <esp_timer.h>
#include <soc/gpio_sig_map.h>
#include <soc/spi_periph.h>
#endif

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_rmt_spi_rx";

MhiRmtSpiRxDriver::~MhiRmtSpiRxDriver() {
  this->shutdown();
}

void MhiRmtSpiRxDriver::shutdown() {
#if MHI_RMT_SPI_RX_SUPPORTED
  this->cleanup_();
#else
  ready_ = false;
#endif
}

bool MhiRmtSpiRxDriver::setup(const MhiTransportPins& pins) {
  pins_ = pins;

#if !MHI_RMT_SPI_RX_SUPPORTED
  ESP_LOGE(TAG, "rmt_spi_rx requires ESP-IDF on ESP32-S3");
  ready_ = false;
  return false;
#else
  this->cleanup_();

  if (pins_.sck < 0 || pins_.mosi < 0) {
    ESP_LOGE(TAG, "RMT/SPI RX setup failed: invalid pins SCK=%d MOSI=%d", pins_.sck, pins_.mosi);
    return false;
  }

  if (config_.frame_size_hint != kMhiFrame20Bytes && config_.frame_size_hint != kMhiFrame33Bytes) {
    ESP_LOGE(TAG, "RMT/SPI RX setup failed: unsupported frame size hint=%u",
             static_cast<unsigned int>(config_.frame_size_hint));
    return false;
  }

  if (config_.frame_gap_us < 500U || config_.frame_gap_us > 5000U) {
    ESP_LOGE(TAG, "RMT/SPI RX setup failed: frame gap must be 500..5000us, got %luus",
             static_cast<unsigned long>(config_.frame_gap_us));
    return false;
  }

  completed_transactions_ = 0U;
  frames_20_ = 0U;
  frames_33_ = 0U;
  invalid_length_transactions_ = 0U;
  transaction_result_errors_ = 0U;
  transaction_queue_errors_ = 0U;
  dropped_frames_ = 0U;
  last_diag_log_ms_ = 0U;

  portENTER_CRITICAL(&mux_);
  completed_frames_.clear();
  marker_sequence_ = 0U;
  marker_frame_end_us_ = 0U;
  marker_frame_len_ = config_.frame_size_hint;
  rmt_boundaries_ = 0U;
  rmt_rearm_errors_ = 0U;
  portEXIT_CRITICAL(&mux_);

  if (!this->allocate_transaction_buffers_() || !this->setup_spi_() || !this->setup_rmt_()) {
    this->cleanup_();
    return false;
  }

  for (auto& slot : transaction_slots_) {
    if (!this->queue_transaction_(slot)) {
      this->cleanup_();
      return false;
    }
  }

  const esp_err_t receive_result =
      rmt_receive(rmt_channel_, rmt_symbols_.data(), sizeof(rmt_symbols_), &rmt_receive_config_);
  if (receive_result != ESP_OK) {
    ESP_LOGE(TAG, "rmt_receive failed: %s", esp_err_to_name(receive_result));
    this->cleanup_();
    return false;
  }

  // Transactions are queued while CS is inactive. Pull the peripheral's
  // internal CS active only after SPI and RMT are both ready.
  this->connect_internal_cs_(false);
  ready_ = true;

  ESP_LOGW(TAG,
           "RMT/SPI RX enabled: host=SPI2 SCK=%d MOSI=%d MISO=unclaimed mode=3 LSB-first DMA=%u bytes "
           "transaction_queue=%u frame_queue=%u frame_gap=%luus",
           pins_.sck, pins_.mosi, static_cast<unsigned int>(kDmaCaptureBytes),
           static_cast<unsigned int>(kTransactionQueueDepth), static_cast<unsigned int>(kCompletedFrameQueueDepth),
           static_cast<unsigned long>(config_.frame_gap_us));

  return true;
#endif
}

void MhiRmtSpiRxDriver::loop() {
#if MHI_RMT_SPI_RX_SUPPORTED
  if (!ready_) {
    return;
  }

  this->process_completed_transactions_();

  const uint32_t now = millis();
  if (last_diag_log_ms_ == 0U || (now - last_diag_log_ms_) >= 30000U) {
    last_diag_log_ms_ = now;

    uint32_t boundaries = 0U;
    uint32_t rearm_errors = 0U;
    uint32_t overwritten = 0U;
    std::size_t queued_frames = 0U;
    std::size_t max_buffered_frames = 0U;

    portENTER_CRITICAL(&mux_);
    boundaries = rmt_boundaries_;
    rearm_errors = rmt_rearm_errors_;
    overwritten = completed_frames_.overwritten_frames();
    queued_frames = completed_frames_.size();
    max_buffered_frames = completed_frames_.high_water_mark();
    portEXIT_CRITICAL(&mux_);

    ESP_LOGI(TAG,
             "runtime: boundaries=%lu completed=%lu frame20=%lu frame33=%lu invalid_len=%lu "
             "result_errors=%lu queue_errors=%lu rmt_rearm_errors=%lu buffered_frames=%u max_buffered=%u "
             "overwritten=%lu dropped=%lu",
             static_cast<unsigned long>(boundaries), static_cast<unsigned long>(completed_transactions_),
             static_cast<unsigned long>(frames_20_), static_cast<unsigned long>(frames_33_),
             static_cast<unsigned long>(invalid_length_transactions_),
             static_cast<unsigned long>(transaction_result_errors_),
             static_cast<unsigned long>(transaction_queue_errors_), static_cast<unsigned long>(rearm_errors),
             static_cast<unsigned int>(queued_frames), static_cast<unsigned int>(max_buffered_frames),
             static_cast<unsigned long>(overwritten), static_cast<unsigned long>(dropped_frames_));
  }
#endif
}

std::size_t MhiRmtSpiRxDriver::read(uint8_t* dst, std::size_t max_len) {
#if !MHI_RMT_SPI_RX_SUPPORTED
  return 0U;
#else
  if (!ready_ || dst == nullptr || max_len == 0U) {
    return 0U;
  }

  MhiCapturedFrame frame{};

  portENTER_CRITICAL(&mux_);
  const bool available = completed_frames_.pop(frame);
  portEXIT_CRITICAL(&mux_);

  if (!available) {
    return 0U;
  }

  if (frame.len > max_len) {
    dropped_frames_++;
    ESP_LOGW(TAG, "RX destination too small: frame=%u destination=%u; frame dropped",
             static_cast<unsigned int>(frame.len), static_cast<unsigned int>(max_len));
    return 0U;
  }

  std::memcpy(dst, frame.data.data(), frame.len);
  return frame.len;
#endif
}

MhiBusMarker MhiRmtSpiRxDriver::bus_marker() const {
#if !MHI_RMT_SPI_RX_SUPPORTED
  return {};
#else
  MhiBusMarker marker{};

  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&mux_));
  marker.valid = marker_sequence_ != 0U;
  marker.sequence = marker_sequence_;
  marker.frame_end_us = marker_frame_end_us_;
  marker.frame_len = marker_frame_len_;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&mux_));

  return marker;
#endif
}

#if MHI_RMT_SPI_RX_SUPPORTED
bool IRAM_ATTR MhiRmtSpiRxDriver::rmt_receive_done_callback_(rmt_channel_handle_t channel,
                                                             const rmt_rx_done_event_data_t* event_data,
                                                             void* user_context) {
  (void)channel;
  if (event_data != nullptr && !event_data->flags.is_last) {
    return false;
  }

  if (user_context == nullptr) {
    return false;
  }

  auto* self = static_cast<MhiRmtSpiRxDriver*>(user_context);
  self->on_frame_boundary_from_isr_();
  return false;
}

bool MhiRmtSpiRxDriver::setup_spi_() {
  spi_bus_config_t bus_config{};
  bus_config.mosi_io_num = pins_.mosi;
  bus_config.miso_io_num = -1;
  bus_config.sclk_io_num = pins_.sck;
  bus_config.quadwp_io_num = -1;
  bus_config.quadhd_io_num = -1;
  bus_config.data4_io_num = -1;
  bus_config.data5_io_num = -1;
  bus_config.data6_io_num = -1;
  bus_config.data7_io_num = -1;
  bus_config.max_transfer_sz = static_cast<int>(kDmaCaptureBytes);
  bus_config.flags = SPICOMMON_BUSFLAG_GPIO_PINS | SPICOMMON_BUSFLAG_SLAVE;

  spi_slave_interface_config_t slave_config{};
  slave_config.spics_io_num = -1;
  slave_config.flags = SPI_SLAVE_BIT_LSBFIRST;
  slave_config.queue_size = static_cast<int>(kTransactionQueueDepth);
  slave_config.mode = 3;

  const esp_err_t result = spi_slave_initialize(host_, &bus_config, &slave_config, SPI_DMA_CH_AUTO);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "spi_slave_initialize failed: %s", esp_err_to_name(result));
    return false;
  }

  spi_initialized_ = true;
  this->connect_internal_cs_(true);
  return true;
}

bool MhiRmtSpiRxDriver::setup_rmt_() {
  rmt_rx_channel_config_t channel_config{};
  channel_config.gpio_num = static_cast<gpio_num_t>(pins_.sck);
  channel_config.clk_src = RMT_CLK_SRC_DEFAULT;
  channel_config.resolution_hz = 1000000U;
  channel_config.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;

  esp_err_t result = rmt_new_rx_channel(&channel_config, &rmt_channel_);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "rmt_new_rx_channel failed: %s", esp_err_to_name(result));
    return false;
  }

  rmt_rx_event_callbacks_t callbacks{};
  callbacks.on_recv_done = &MhiRmtSpiRxDriver::rmt_receive_done_callback_;

  result = rmt_rx_register_event_callbacks(rmt_channel_, &callbacks, this);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "rmt_rx_register_event_callbacks failed: %s", esp_err_to_name(result));
    return false;
  }

  result = rmt_enable(rmt_channel_);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(result));
    return false;
  }
  rmt_enabled_ = true;

  rmt_receive_config_ = {};
  rmt_receive_config_.signal_range_min_ns = 1000U;
  rmt_receive_config_.signal_range_max_ns = config_.frame_gap_us * 1000U;

  this->setup_glitch_filter_();
  return true;
}

bool MhiRmtSpiRxDriver::setup_glitch_filter_() {
#if SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER
  gpio_pin_glitch_filter_config_t filter_config{};
  filter_config.clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT;
  filter_config.gpio_num = static_cast<gpio_num_t>(pins_.sck);

  esp_err_t result = gpio_new_pin_glitch_filter(&filter_config, &glitch_filter_);
  if (result != ESP_OK) {
    ESP_LOGW(TAG, "gpio_new_pin_glitch_filter failed; continuing without filter: %s", esp_err_to_name(result));
    glitch_filter_ = nullptr;
    return false;
  }

  result = gpio_glitch_filter_enable(glitch_filter_);
  if (result != ESP_OK) {
    ESP_LOGW(TAG, "gpio_glitch_filter_enable failed; continuing without filter: %s", esp_err_to_name(result));
    gpio_del_glitch_filter(glitch_filter_);
    glitch_filter_ = nullptr;
    return false;
  }

  return true;
#else
  return false;
#endif
}

bool MhiRmtSpiRxDriver::allocate_transaction_buffers_() {
  for (auto& slot : transaction_slots_) {
    slot.rx_buffer = static_cast<uint8_t*>(
        heap_caps_calloc(kDmaCaptureBytes, sizeof(uint8_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (slot.rx_buffer == nullptr) {
      ESP_LOGE(TAG, "Failed to allocate %u-byte DMA RX buffer", static_cast<unsigned int>(kDmaCaptureBytes));
      return false;
    }
  }

  return true;
}

bool MhiRmtSpiRxDriver::queue_transaction_(TransactionSlot& slot) {
  if (slot.rx_buffer == nullptr) {
    return false;
  }

  std::memset(slot.rx_buffer, 0, kDmaCaptureBytes);
  slot.transaction = {};
  slot.transaction.length = kDmaCaptureBytes * 8U;
  slot.transaction.rx_buffer = slot.rx_buffer;
  slot.transaction.tx_buffer = nullptr;
  slot.transaction.user = &slot;

  const esp_err_t result = spi_slave_queue_trans(host_, &slot.transaction, 0);
  if (result != ESP_OK) {
    transaction_queue_errors_++;
    ESP_LOGE(TAG, "spi_slave_queue_trans failed: %s", esp_err_to_name(result));
    return false;
  }

  return true;
}

void MhiRmtSpiRxDriver::process_completed_transactions_() {
  while (true) {
    spi_slave_transaction_t* completed = nullptr;
    const esp_err_t result = spi_slave_get_trans_result(host_, &completed, 0);

    if (result == ESP_ERR_TIMEOUT) {
      return;
    }

    if (result != ESP_OK) {
      transaction_result_errors_++;
      ESP_LOGW(TAG, "spi_slave_get_trans_result failed: %s", esp_err_to_name(result));
      return;
    }

    if (completed == nullptr || completed->user == nullptr) {
      transaction_result_errors_++;
      ESP_LOGW(TAG, "Completed SPI transaction did not identify its transaction slot");
      return;
    }

    auto* slot = static_cast<TransactionSlot*>(completed->user);
    const std::size_t received_bits = completed->trans_len;
    const bool whole_bytes = (received_bits % 8U) == 0U;
    const std::size_t received_bytes = whole_bytes ? received_bits / 8U : 0U;

    if (whole_bytes && (received_bytes == kMhiFrame20Bytes || received_bytes == kMhiFrame33Bytes)) {
      uint32_t sequence = 0U;
      uint32_t frame_end_us = 0U;
      uint32_t overwritten_before = 0U;
      uint32_t overwritten_after = 0U;

      portENTER_CRITICAL(&mux_);
      sequence = marker_sequence_;
      frame_end_us = marker_frame_end_us_;
      overwritten_before = completed_frames_.overwritten_frames();
      completed_frames_.push(slot->rx_buffer, received_bytes, sequence, frame_end_us);
      overwritten_after = completed_frames_.overwritten_frames();
      marker_frame_len_ = received_bytes;
      portEXIT_CRITICAL(&mux_);

      dropped_frames_ += overwritten_after - overwritten_before;
      completed_transactions_++;
      if (received_bytes == kMhiFrame20Bytes) {
        frames_20_++;
      } else {
        frames_33_++;
      }
    } else {
      invalid_length_transactions_++;
      ESP_LOGV(TAG, "Discarded SPI transaction length=%u bits", static_cast<unsigned int>(received_bits));
    }

    if (!this->queue_transaction_(*slot)) {
      ready_ = false;
      return;
    }
  }
}

void MhiRmtSpiRxDriver::cleanup_() {
  ready_ = false;

  if (spi_initialized_) {
    this->connect_internal_cs_(true);
  }

#if SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER
  if (glitch_filter_ != nullptr) {
    gpio_glitch_filter_disable(glitch_filter_);
    gpio_del_glitch_filter(glitch_filter_);
    glitch_filter_ = nullptr;
  }
#endif

  if (rmt_channel_ != nullptr) {
    if (rmt_enabled_) {
      rmt_disable(rmt_channel_);
      rmt_enabled_ = false;
    }
    rmt_del_channel(rmt_channel_);
    rmt_channel_ = nullptr;
  }

  if (spi_initialized_) {
    spi_slave_free(host_);
    spi_initialized_ = false;
  }

  for (auto& slot : transaction_slots_) {
    if (slot.rx_buffer != nullptr) {
      heap_caps_free(slot.rx_buffer);
      slot.rx_buffer = nullptr;
    }
    slot.transaction = {};
  }

  portENTER_CRITICAL(&mux_);
  completed_frames_.clear();
  marker_sequence_ = 0U;
  marker_frame_end_us_ = 0U;
  marker_frame_len_ = 0U;
  portEXIT_CRITICAL(&mux_);
}

void MhiRmtSpiRxDriver::connect_internal_cs_(bool inactive_high) {
  const uint32_t signal = spi_periph_signal[host_].spics_in;
  const uint32_t source = inactive_high ? GPIO_MATRIX_CONST_ONE_INPUT : GPIO_MATRIX_CONST_ZERO_INPUT;
  esp_rom_gpio_connect_in_signal(source, signal, false);
}

void IRAM_ATTR MhiRmtSpiRxDriver::on_frame_boundary_from_isr_() {
  // End the current SPI transaction, then immediately arm the next queued
  // transaction. No physical CS pin is required.
  const uint32_t signal = spi_periph_signal[host_].spics_in;
  esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, signal, false);
  esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, signal, false);

  portENTER_CRITICAL_ISR(&mux_);
  marker_sequence_++;
  marker_frame_end_us_ = static_cast<uint32_t>(esp_timer_get_time());
  marker_frame_len_ = config_.frame_size_hint;
  rmt_boundaries_++;
  portEXIT_CRITICAL_ISR(&mux_);

  const esp_err_t result = rmt_receive(rmt_channel_, rmt_symbols_.data(), sizeof(rmt_symbols_), &rmt_receive_config_);
  if (result != ESP_OK) {
    portENTER_CRITICAL_ISR(&mux_);
    rmt_rearm_errors_++;
    portEXIT_CRITICAL_ISR(&mux_);
  }
}

#endif

}  // namespace mhi_ac_ctrl
}  // namespace esphome
