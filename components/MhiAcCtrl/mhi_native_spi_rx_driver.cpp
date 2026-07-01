#include "mhi_native_spi_rx_driver.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_ESP_IDF
#include <esp_err.h>
#endif

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_native_spi_rx";

bool MhiNativeSpiRxDriver::setup(const MhiTransportPins& pins) {
  pins_ = pins;

#ifndef USE_ESP_IDF
  ESP_LOGE(TAG, "native SPI RX requires ESP-IDF");
  ready_ = false;
  return false;
#else
  if (pins_.sck < 0 || pins_.mosi < 0) {
    ESP_LOGE(TAG, "Native SPI RX setup failed: invalid pins SCK=%d MOSI=%d", pins_.sck, pins_.mosi);
    ready_ = false;
    return false;
  }

  spi_bus_config_t bus_config{};
  bus_config.mosi_io_num = pins_.mosi;
  bus_config.miso_io_num = -1;
  bus_config.sclk_io_num = pins_.sck;
  bus_config.quadwp_io_num = -1;
  bus_config.quadhd_io_num = -1;
  bus_config.max_transfer_sz = static_cast<int>(kCaptureBytes);

  spi_slave_interface_config_t slave_config{};
  slave_config.spics_io_num = -1;
  slave_config.flags = 0;
  slave_config.queue_size = static_cast<int>(kQueueDepth);
  slave_config.mode = 3;

  const esp_err_t init_result = spi_slave_initialize(this->host_, &bus_config, &slave_config, SPI_DMA_DISABLED);
  if (init_result != ESP_OK) {
    ESP_LOGE(TAG, "spi_slave_initialize failed with CS=-1: %s", esp_err_to_name(init_result));
    ready_ = false;
    return false;
  }

  for (auto& slot : slots_) {
    std::memset(slot.rx_buffer, 0, sizeof(slot.rx_buffer));
    if (!queue_slot_(slot)) {
      spi_slave_free(this->host_);
      ready_ = false;
      return false;
    }
  }

  ready_ = true;

  ESP_LOGW(TAG,
           "Native SPI RX probe enabled: host=SPI2 SCK=%d MOSI=%d MISO=disabled CS=-1 mode=3 capture=%u bytes. "
           "This is experimental because the ESP-IDF slave driver is still transaction/CS oriented.",
           pins_.sck, pins_.mosi, static_cast<unsigned int>(kCaptureBytes));

  return true;
#endif
}

void MhiNativeSpiRxDriver::loop() {
#ifndef USE_ESP_IDF
  return;
#else
  if (!ready_) {
    return;
  }

  while (true) {
    spi_slave_transaction_t* completed = nullptr;
    const esp_err_t result = spi_slave_get_trans_result(this->host_, &completed, 0);

    if (result == ESP_ERR_TIMEOUT) {
      break;
    }

    if (result != ESP_OK) {
      ESP_LOGW(TAG, "spi_slave_get_trans_result failed: %s", esp_err_to_name(result));
      break;
    }

    if (completed == nullptr || completed->user == nullptr) {
      ESP_LOGW(TAG, "native SPI RX completed transaction missing user slot");
      break;
    }

    auto* slot = static_cast<TransactionSlot*>(completed->user);
    const std::size_t rx_bits = completed->trans_len;
    const std::size_t rx_bytes = std::min<std::size_t>((rx_bits + 7U) / 8U, kCaptureBytes);

    if (rx_bytes == 0U) {
      zero_length_transactions_++;
    } else {
      push_bytes_(slot->rx_buffer, rx_bytes);
      completed_transactions_++;
    }

    std::memset(slot->rx_buffer, 0, sizeof(slot->rx_buffer));

    if (!queue_slot_(*slot)) {
      queue_errors_++;
      break;
    }
  }

  const uint32_t now = millis();
  if (last_diag_log_ms_ == 0U || (now - last_diag_log_ms_) >= 30000U) {
    last_diag_log_ms_ = now;
    ESP_LOGI(TAG, "probe: completed=%lu zero_len=%lu queued_errors=%lu buffered=%u dropped=%lu",
             static_cast<unsigned long>(completed_transactions_), static_cast<unsigned long>(zero_length_transactions_),
             static_cast<unsigned long>(queue_errors_), static_cast<unsigned int>(ring_available_()),
             static_cast<unsigned long>(dropped_bytes_));
  }
#endif
}

std::size_t MhiNativeSpiRxDriver::read(uint8_t* dst, std::size_t max_len) {
#ifndef USE_ESP_IDF
  return 0U;
#else
  if (!ready_ || dst == nullptr || max_len == 0U) {
    return 0U;
  }

  std::size_t copied = 0U;
  while (copied < max_len && ring_tail_ != ring_head_) {
    dst[copied++] = ring_[ring_tail_];
    ring_tail_ = (ring_tail_ + 1U) % kRingBytes;
  }

  return copied;
#endif
}

#ifdef USE_ESP_IDF
bool MhiNativeSpiRxDriver::queue_slot_(TransactionSlot& slot) {
  slot.transaction = {};
  slot.transaction.length = kCaptureBytes * 8U;
  slot.transaction.rx_buffer = slot.rx_buffer;
  slot.transaction.tx_buffer = nullptr;
  slot.transaction.user = &slot;

  const esp_err_t result = spi_slave_queue_trans(this->host_, &slot.transaction, 0);
  if (result != ESP_OK) {
    queue_errors_++;
    ESP_LOGW(TAG, "spi_slave_queue_trans failed: %s", esp_err_to_name(result));
    return false;
  }

  return true;
}

void MhiNativeSpiRxDriver::push_bytes_(const uint8_t* data, std::size_t len) {
  if (data == nullptr || len == 0U) {
    return;
  }

  for (std::size_t i = 0U; i < len; i++) {
    const std::size_t next_head = (ring_head_ + 1U) % kRingBytes;
    if (next_head == ring_tail_) {
      dropped_bytes_++;
      return;
    }

    ring_[ring_head_] = data[i];
    ring_head_ = next_head;
  }
}

std::size_t MhiNativeSpiRxDriver::ring_available_() const {
  if (ring_head_ >= ring_tail_) {
    return ring_head_ - ring_tail_;
  }

  return kRingBytes - ring_tail_ + ring_head_;
}
#endif

}  // namespace mhi_ac_ctrl
}  // namespace esphome
