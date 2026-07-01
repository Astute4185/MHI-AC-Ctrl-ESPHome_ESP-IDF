#include "mhi_external_clock_rx_driver.h"

#include <algorithm>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_ESP_IDF
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_timer.h>
#endif

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const EXTERNAL_CLOCK_RX_TAG = "mhi_external_clock_rx";

bool MhiExternalClockRxDriver::setup(const MhiTransportPins& pins) {
  pins_ = pins;

#ifndef USE_ESP_IDF
  ESP_LOGE(EXTERNAL_CLOCK_RX_TAG, "external clock RX requires ESP-IDF");
  ready_ = false;
  return false;
#else
  if (pins_.sck < 0 || pins_.mosi < 0) {
    ESP_LOGE(EXTERNAL_CLOCK_RX_TAG, "External clock RX setup failed: invalid pins SCK=%d MOSI=%d", pins_.sck,
             pins_.mosi);
    ready_ = false;
    return false;
  }

  gpio_config_t mosi_config{};
  mosi_config.pin_bit_mask = 1ULL << static_cast<uint32_t>(pins_.mosi);
  mosi_config.mode = GPIO_MODE_INPUT;
  mosi_config.pull_up_en = GPIO_PULLUP_DISABLE;
  mosi_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  mosi_config.intr_type = GPIO_INTR_DISABLE;
  esp_err_t result = gpio_config(&mosi_config);
  if (result != ESP_OK) {
    ESP_LOGE(EXTERNAL_CLOCK_RX_TAG, "gpio_config MOSI failed: %s", esp_err_to_name(result));
    ready_ = false;
    return false;
  }

  gpio_config_t sck_config{};
  sck_config.pin_bit_mask = 1ULL << static_cast<uint32_t>(pins_.sck);
  sck_config.mode = GPIO_MODE_INPUT;
  sck_config.pull_up_en = GPIO_PULLUP_DISABLE;
  sck_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  sck_config.intr_type =
      config_.sample_edge == MhiExternalClockSampleEdge::FALLING ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
  result = gpio_config(&sck_config);
  if (result != ESP_OK) {
    ESP_LOGE(EXTERNAL_CLOCK_RX_TAG, "gpio_config SCK failed: %s", esp_err_to_name(result));
    ready_ = false;
    return false;
  }

  result = gpio_install_isr_service(0);
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(EXTERNAL_CLOCK_RX_TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(result));
    ready_ = false;
    return false;
  }

  result = gpio_isr_handler_add(static_cast<gpio_num_t>(pins_.sck), &MhiExternalClockRxDriver::isr_trampoline_, this);
  if (result != ESP_OK) {
    ESP_LOGE(EXTERNAL_CLOCK_RX_TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(result));
    ready_ = false;
    return false;
  }

  portENTER_CRITICAL(&mux_);
  ring_head_ = 0U;
  ring_tail_ = 0U;
  current_frame_len_ = 0U;
  signature_window_ = {};
  signature_window_len_ = 0U;
  current_byte_ = 0U;
  bit_index_ = 0U;
  last_edge_us_ = 0;
  portEXIT_CRITICAL(&mux_);

  ready_ = true;

  ESP_LOGW(EXTERNAL_CLOCK_RX_TAG,
           "External-clock RX probe enabled: SCK=%d MOSI=%d capture=%s-edge LSB-first byte_gap_reset=%luus "
           "frame_gap_reset=%luus min_edge_gap=%luus sample_delay=%lu NOPs ring=%u frames/%u bytes. "
           "TX should be disabled for first validation.",
           pins_.sck, pins_.mosi, this->sample_edge_name_(), static_cast<unsigned long>(config_.byte_gap_reset_us),
           static_cast<unsigned long>(config_.frame_gap_reset_us), static_cast<unsigned long>(config_.min_edge_gap_us),
           static_cast<unsigned long>(config_.sample_delay_nops), static_cast<unsigned int>(kRingFrames),
           static_cast<unsigned int>(kRingBytes));

  return true;
#endif
}

void MhiExternalClockRxDriver::loop() {
#ifndef USE_ESP_IDF
  return;
#else
  if (!ready_) {
    return;
  }

  const uint32_t now = millis();
  if (last_diag_log_ms_ == 0U || (now - last_diag_log_ms_) >= 30000U) {
    last_diag_log_ms_ = now;
    ESP_LOGI(EXTERNAL_CLOCK_RX_TAG,
             "probe: edge=%s delay=%lu edges=%lu bytes=%lu frame_chunks=%lu buffered=%u current_frame_len=%u "
             "signature_starts=%lu signature_restarts=%lu discarded_presync_bytes=%lu bad_length_discards=%lu "
             "byte_gap_resets=%lu frame_gap_resets=%lu full_byte_gap_resets=%lu partial_byte_gap_resets=%lu "
             "discarded_partial_frames=%lu dropped_frame_chunks=%lu short_edges=%lu dropped=%lu "
             "ring_capacity=%u peak_buffered=%lu "
             "gap_us last=%lu min=%lu max=%lu",
             this->sample_edge_name_(), static_cast<unsigned long>(config_.sample_delay_nops),
             static_cast<unsigned long>(edges_), static_cast<unsigned long>(bytes_),
             static_cast<unsigned long>(emitted_frame_chunks_), static_cast<unsigned int>(ring_available_()),
             static_cast<unsigned int>(current_frame_len_), static_cast<unsigned long>(signature_starts_),
             static_cast<unsigned long>(signature_restarts_during_frame_),
             static_cast<unsigned long>(discarded_presync_bytes_), static_cast<unsigned long>(bad_length_discards_),
             static_cast<unsigned long>(byte_gap_resets_), static_cast<unsigned long>(frame_gap_resets_),
             static_cast<unsigned long>(full_byte_gap_resets_), static_cast<unsigned long>(partial_byte_gap_resets_),
             static_cast<unsigned long>(discarded_partial_frames_), static_cast<unsigned long>(dropped_frame_chunks_),
             static_cast<unsigned long>(short_edges_), static_cast<unsigned long>(dropped_bytes_),
             static_cast<unsigned int>(kRingBytes), static_cast<unsigned long>(peak_buffered_bytes_),
             static_cast<unsigned long>(last_gap_us_), static_cast<unsigned long>(min_observed_gap_us_),
             static_cast<unsigned long>(max_observed_gap_us_));
  }
#endif
}

std::size_t MhiExternalClockRxDriver::read(uint8_t* dst, std::size_t max_len) {
#ifndef USE_ESP_IDF
  return 0U;
#else
  if (!ready_ || dst == nullptr || max_len == 0U) {
    return 0U;
  }

  std::size_t copied = 0U;

  portENTER_CRITICAL(&mux_);
  while (copied < max_len && ring_tail_ != ring_head_) {
    dst[copied++] = ring_[ring_tail_];
    ring_tail_ = (ring_tail_ + 1U) % kRingBytes;
  }
  portEXIT_CRITICAL(&mux_);

  return copied;
#endif
}

#ifdef USE_ESP_IDF
void IRAM_ATTR MhiExternalClockRxDriver::isr_trampoline_(void* arg) {
  if (arg == nullptr) {
    return;
  }

  auto* self = static_cast<MhiExternalClockRxDriver*>(arg);
  portENTER_CRITICAL_ISR(&self->mux_);
  self->handle_sck_edge_();
  portEXIT_CRITICAL_ISR(&self->mux_);
}

void IRAM_ATTR MhiExternalClockRxDriver::handle_sck_edge_() {
  const int64_t now_us = esp_timer_get_time();
  const int64_t previous_edge_us = last_edge_us_;
  last_edge_us_ = now_us;

  if (previous_edge_us != 0) {
    const uint32_t delta_us = static_cast<uint32_t>(now_us - previous_edge_us);

    if (delta_us < config_.min_edge_gap_us) {
      short_edges_++;
      return;
    }

    last_gap_us_ = delta_us;
    if (min_observed_gap_us_ == 0U || delta_us < min_observed_gap_us_) {
      min_observed_gap_us_ = delta_us;
    }
    if (delta_us > max_observed_gap_us_) {
      max_observed_gap_us_ = delta_us;
    }

    if (delta_us >= config_.frame_gap_reset_us) {
      if (bit_index_ != 0U) {
        partial_byte_gap_resets_++;
      }

      current_byte_ = 0U;
      bit_index_ = 0U;
      frame_gap_resets_++;
      discard_current_frame_from_isr_();
      reset_signature_window_from_isr_();
    } else if (delta_us >= config_.byte_gap_reset_us) {
      byte_gap_resets_++;
      if (bit_index_ != 0U) {
        current_byte_ = 0U;
        bit_index_ = 0U;
        partial_byte_gap_resets_++;
      } else {
        full_byte_gap_resets_++;
      }
    }
  }

  apply_sample_delay_from_isr_();

  if (gpio_get_level(static_cast<gpio_num_t>(pins_.mosi)) != 0) {
    current_byte_ = static_cast<uint8_t>(current_byte_ | (1U << bit_index_));
  }

  bit_index_++;
  edges_++;

  if (bit_index_ >= 8U) {
    append_byte_from_isr_(current_byte_);
    current_byte_ = 0U;
    bit_index_ = 0U;
  }
}

void IRAM_ATTR MhiExternalClockRxDriver::apply_sample_delay_from_isr_() {
  for (uint32_t i = 0U; i < config_.sample_delay_nops; i++) {
    asm volatile("nop");
  }
}

void IRAM_ATTR MhiExternalClockRxDriver::append_byte_from_isr_(uint8_t value) {
  bytes_++;

  const bool signature_seen = update_signature_window_from_isr_(value);

  if (signature_seen) {
    if (current_frame_len_ != 0U) {
      signature_restarts_during_frame_++;
      discard_current_frame_from_isr_();
    }

    start_frame_from_signature_window_from_isr_();
    return;
  }

  if (current_frame_len_ == 0U) {
    discarded_presync_bytes_++;
    return;
  }

  if (current_frame_len_ >= kMhiMaxFrameBytes) {
    bad_length_discards_++;
    discard_current_frame_from_isr_();
    return;
  }

  current_frame_[current_frame_len_] = value;
  current_frame_len_++;

  if (current_frame_len_ >= expected_frame_size_()) {
    push_current_frame_from_isr_();
  }
}

void IRAM_ATTR MhiExternalClockRxDriver::push_current_frame_from_isr_() {
  const std::size_t frame_len = current_frame_len_;
  if (frame_len == 0U) {
    return;
  }

  if (frame_len != expected_frame_size_()) {
    bad_length_discards_++;
    discard_current_frame_from_isr_();
    return;
  }

  if (ring_free_() < frame_len) {
    dropped_frame_chunks_++;
    dropped_bytes_ += static_cast<uint32_t>(frame_len);
    current_frame_len_ = 0U;
    return;
  }

  for (std::size_t i = 0U; i < frame_len; i++) {
    ring_[ring_head_] = current_frame_[i];
    ring_head_ = (ring_head_ + 1U) % kRingBytes;
  }

  const std::size_t buffered = ring_available_();
  if (buffered > peak_buffered_bytes_) {
    peak_buffered_bytes_ = static_cast<uint32_t>(buffered);
  }

  emitted_frame_chunks_++;
  current_frame_len_ = 0U;
}

void IRAM_ATTR MhiExternalClockRxDriver::discard_current_frame_from_isr_() {
  if (current_frame_len_ != 0U) {
    discarded_partial_frames_++;
  }

  current_frame_len_ = 0U;
}

void IRAM_ATTR MhiExternalClockRxDriver::reset_signature_window_from_isr_() {
  signature_window_[0] = 0U;
  signature_window_[1] = 0U;
  signature_window_[2] = 0U;
  signature_window_len_ = 0U;
}

bool IRAM_ATTR MhiExternalClockRxDriver::update_signature_window_from_isr_(uint8_t value) {
  signature_window_[0] = signature_window_[1];
  signature_window_[1] = signature_window_[2];
  signature_window_[2] = value;

  if (signature_window_len_ < 3U) {
    signature_window_len_++;
  }

  if (signature_window_len_ < 3U) {
    return false;
  }

  const bool valid_sig0 =
      signature_window_[0] == kMhiMosiSignature0Default || signature_window_[0] == kMhiMosiSignature0Alt;
  return valid_sig0 && signature_window_[1] == kMhiMosiSignature1 && signature_window_[2] == kMhiMosiSignature2;
}

void IRAM_ATTR MhiExternalClockRxDriver::start_frame_from_signature_window_from_isr_() {
  current_frame_[0] = signature_window_[0];
  current_frame_[1] = signature_window_[1];
  current_frame_[2] = signature_window_[2];
  current_frame_len_ = 3U;
  signature_starts_++;
}

const char* MhiExternalClockRxDriver::sample_edge_name_() const {
  return config_.sample_edge == MhiExternalClockSampleEdge::FALLING ? "falling" : "rising";
}

std::size_t MhiExternalClockRxDriver::ring_available_() const {
  const std::size_t head = ring_head_;
  const std::size_t tail = ring_tail_;

  if (head >= tail) {
    return head - tail;
  }

  return kRingBytes - tail + head;
}

std::size_t MhiExternalClockRxDriver::ring_free_() const {
  return (kRingBytes - 1U) - ring_available_();
}

std::size_t MhiExternalClockRxDriver::expected_frame_size_() const {
  if (config_.frame_size_hint == kMhiFrame33Bytes) {
    return kMhiFrame33Bytes;
  }

  return kMhiFrame20Bytes;
}
#endif

}  // namespace mhi_ac_ctrl
}  // namespace esphome
