#include "mhi_transport_manager.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_ac_ctrl {

static const char* const TAG = "mhi_transport";

void MhiTransportManager::configure(int sck_pin, int mosi_pin, int miso_pin, const std::string& rx_driver,
                                    const std::string& tx_driver, uint8_t frame_size_hint, uint32_t frame_start_idle_ms,
                                    uint32_t external_clock_byte_gap_us, uint32_t external_clock_frame_gap_us,
                                    uint32_t external_clock_min_edge_gap_us, const std::string& external_clock_edge,
                                    uint32_t external_clock_sample_delay_nops) {
  pins_.sck = sck_pin;
  pins_.mosi = mosi_pin;
  pins_.miso = miso_pin;
  requested_rx_driver_name_ = rx_driver.empty() ? "fast_gpio_rx" : rx_driver;
  const bool integrated_duplex = requested_rx_driver_name_ == "rmt_cs_spi";
  requested_tx_driver_name_ =
      tx_driver.empty() ? (integrated_duplex ? requested_rx_driver_name_ : "fast_gpio_tx") : tx_driver;

  (void)frame_size_hint;
  (void)frame_start_idle_ms;
  (void)external_clock_byte_gap_us;
  (void)external_clock_frame_gap_us;
  (void)external_clock_min_edge_gap_us;
  (void)external_clock_edge;
  (void)external_clock_sample_delay_nops;

#if MHI_ENABLE_FAST_GPIO_TRANSPORT
  MhiFastGpioRxConfig fast_gpio_rx_config{};
  fast_gpio_rx_config.frame_size_hint = frame_size_hint;
  fast_gpio_rx_config.frame_start_idle_ms = frame_start_idle_ms;
  fast_gpio_rx_.set_config(fast_gpio_rx_config);
#endif

#if MHI_ENABLE_SPLIT_TX_DRIVER
  MhiFastGpioTxConfig fast_gpio_tx_config{};
  fast_gpio_tx_config.frame_size_hint = frame_size_hint;
  fast_gpio_tx_config.frame_start_idle_ms = frame_start_idle_ms;
  fast_gpio_tx_config.max_exchange_time_ms = 60U;
  fast_gpio_tx_.set_config(fast_gpio_tx_config);
#endif

#if MHI_ENABLE_RMT_SPI_RX_DRIVER
  MhiRmtSpiRxConfig rmt_spi_rx_config{};
  rmt_spi_rx_config.frame_size_hint = frame_size_hint;
  rmt_spi_rx_config.frame_gap_us = rmt_spi_frame_gap_us_;
  rmt_spi_rx_.set_config(rmt_spi_rx_config);
#endif

#if MHI_ENABLE_RMT_CS_SPI_TRANSPORT
  MhiRmtCsSpiConfig rmt_cs_spi_config{};
  rmt_cs_spi_config.frame_size_hint = frame_size_hint;
  rmt_cs_spi_config.frame_gap_us = rmt_spi_frame_gap_us_;
  rmt_cs_spi_.set_config(rmt_cs_spi_config);
#endif

#if MHI_ENABLE_EXTERNAL_CLOCK_RX_DRIVER
  MhiExternalClockRxConfig external_clock_rx_config{};
  external_clock_rx_config.frame_size_hint = frame_size_hint;
  external_clock_rx_config.byte_gap_reset_us = external_clock_byte_gap_us;
  external_clock_rx_config.frame_gap_reset_us = external_clock_frame_gap_us;
  external_clock_rx_config.min_edge_gap_us = external_clock_min_edge_gap_us;
  external_clock_rx_config.sample_edge =
      external_clock_edge == "falling" ? MhiExternalClockSampleEdge::FALLING : MhiExternalClockSampleEdge::RISING;
  external_clock_rx_config.sample_delay_nops = external_clock_sample_delay_nops;
  external_clock_rx_.set_config(external_clock_rx_config);
#endif

  this->resolve_transports_();
  this->reset_transport_diagnostic_cursors_();
}

void MhiTransportManager::resolve_transports_() {
  primary_ = nullptr;
  recovery_ = nullptr;
  active_ = nullptr;
  recovery_active_ = false;

  if (requested_rx_driver_name_ == "rmt_cs_spi") {
#if MHI_ENABLE_RMT_CS_SPI_TRANSPORT
    primary_duplex_transport_.bind(&rmt_cs_spi_, true);
    primary_ = &primary_duplex_transport_;
#else
    ESP_LOGW(TAG, "rmt_cs_spi is not compiled; using FastGPIO transport");
#endif
  }

  if (primary_ == nullptr && requested_rx_driver_name_ == "fast_gpio_rx") {
#if MHI_ENABLE_FAST_GPIO_TRANSPORT
#if MHI_ENABLE_SPLIT_TX_DRIVER
    IMhiTxDriver* tx = requested_tx_driver_name_ == "none" ? static_cast<IMhiTxDriver*>(&null_tx_)
                                                           : static_cast<IMhiTxDriver*>(&fast_gpio_tx_);
    const bool uses_bus_marker = tx == &fast_gpio_tx_;
#else
    IMhiTxDriver* tx = nullptr;
    const bool uses_bus_marker = false;
    if (requested_tx_driver_name_ == "fast_gpio_tx") {
      ESP_LOGW(TAG, "Split FastGPIO TX is not built for this target; TX disabled");
    }
#endif
    primary_split_transport_.bind(&fast_gpio_rx_, tx, false, uses_bus_marker);
    primary_ = &primary_split_transport_;
#endif
  }

  if (primary_ == nullptr && requested_rx_driver_name_ == "rmt_spi_rx") {
#if MHI_ENABLE_RMT_SPI_RX_DRIVER && MHI_ENABLE_SPLIT_TX_DRIVER
    IMhiTxDriver* tx = requested_tx_driver_name_ == "none" ? static_cast<IMhiTxDriver*>(&null_tx_)
                                                           : static_cast<IMhiTxDriver*>(&fast_gpio_tx_);
    primary_split_transport_.bind(&rmt_spi_rx_, tx, true, tx == &fast_gpio_tx_);
    primary_ = &primary_split_transport_;
#else
    ESP_LOGW(TAG, "rmt_spi_rx is not compiled for this target; using FastGPIO transport");
#endif
  }

  if (primary_ == nullptr && requested_rx_driver_name_ == "external_clock_rx") {
#if MHI_ENABLE_EXTERNAL_CLOCK_RX_DRIVER && MHI_ENABLE_SPLIT_TX_DRIVER
    IMhiTxDriver* tx = requested_tx_driver_name_ == "none" ? static_cast<IMhiTxDriver*>(&null_tx_)
                                                           : static_cast<IMhiTxDriver*>(&fast_gpio_tx_);
    primary_split_transport_.bind(&external_clock_rx_, tx, true, tx == &fast_gpio_tx_);
    primary_ = &primary_split_transport_;
#else
    ESP_LOGW(TAG, "external_clock_rx is not compiled for this target; using FastGPIO transport");
#endif
  }

  if (primary_ == nullptr) {
#if MHI_ENABLE_FAST_GPIO_TRANSPORT
#if MHI_ENABLE_SPLIT_TX_DRIVER
    primary_split_transport_.bind(&fast_gpio_rx_, &fast_gpio_tx_, false, true);
#else
    primary_split_transport_.bind(&fast_gpio_rx_, nullptr, false, false);
#endif
    primary_ = &primary_split_transport_;
    requested_rx_driver_name_ = "fast_gpio_rx";
    requested_tx_driver_name_ = "fast_gpio_tx";
#else
    ESP_LOGE(TAG, "No compiled transport can satisfy RX=%s TX=%s", requested_rx_driver_name_.c_str(),
             requested_tx_driver_name_.c_str());
#endif
  }

#if MHI_ENABLE_FAST_GPIO_TRANSPORT && MHI_ENABLE_SPLIT_TX_DRIVER
  if (requested_rx_driver_name_ != "fast_gpio_rx") {
    recovery_split_transport_.bind(&fast_gpio_rx_, &fast_gpio_tx_, false, true);
    recovery_ = &recovery_split_transport_;
  }
#endif

  active_ = primary_;
}

bool MhiTransportManager::setup() {
  ESP_LOGCONFIG(TAG, "Transport setup: requested RX=%s TX=%s", requested_rx_driver_name_.c_str(),
                requested_tx_driver_name_.c_str());

  last_transport_error_ = {};
  MhiTransportResult setup_result =
      active_ == nullptr ? MhiTransportResult::failure(MhiTransportError::DRIVER_NOT_BOUND, "resolve_active_transport")
                         : active_->setup(pins_);
  bool ready = setup_result.ok;
  if (!ready) {
    last_transport_error_ = setup_result.error;
  }

  if (!ready && recovery_ != nullptr && recovery_ != active_) {
    ESP_LOGW(TAG,
             "Primary transport RX=%s TX=%s failed to start: error=%s operation=%s native=%ld; activating internal "
             "FastGPIO recovery",
             active_ == nullptr ? "none" : active_->rx_name(), active_ == nullptr ? "none" : active_->tx_name(),
             mhi_transport_error_name(setup_result.error.code),
             setup_result.error.operation == nullptr ? "none" : setup_result.error.operation,
             static_cast<long>(setup_result.error.native_code));
    if (active_ != nullptr) {
      active_->shutdown();
    }
    active_ = recovery_;
    recovery_active_ = true;
    this->reset_transport_diagnostic_cursors_();
    setup_result = active_->setup(pins_);
    ready = setup_result.ok;
    if (!ready) {
      last_transport_error_ = setup_result.error;
    }
  }

  if (diagnostics_ != nullptr) {
    diagnostics_->set_rx_driver_name(this->rx_name());
    diagnostics_->set_tx_driver_name(this->tx_name());
    diagnostics_->set_rx_driver_ready(this->rx_ready());
    diagnostics_->set_tx_driver_ready(this->tx_ready());
  }

  ESP_LOGCONFIG(TAG, "Transport active: strategy=%s RX=%s ready=%s TX=%s ready=%s recovery=%s",
                active_ == nullptr ? "none" : active_->name(), this->rx_name(), this->rx_ready() ? "YES" : "NO",
                this->tx_name(), this->tx_ready() ? "YES" : "NO", recovery_active_ ? "YES" : "NO");

  if (active_ != nullptr && active_->capabilities().uses_bus_marker) {
    ESP_LOGCONFIG(TAG, "TX ownership: split transport uses marker-armed real-time transmission");
  } else if (active_ != nullptr && active_->capabilities().integrated_duplex) {
    ESP_LOGCONFIG(TAG, "TX ownership: duplex transport stages one frame for the next GP-SPI transaction");
  }

  return ready && this->rx_ready() && this->tx_ready();
}

void MhiTransportManager::loop() {
  if (active_ == nullptr) {
    return;
  }
  active_->loop();
  this->update_transport_diagnostics_();
}

void MhiTransportManager::shutdown() {
  if (active_ != nullptr) {
    active_->shutdown();
  }
}

std::size_t MhiTransportManager::read_rx(uint8_t* dst, std::size_t max_len) {
  if (active_ == nullptr) {
    return 0U;
  }

  const std::size_t len = active_->read(dst, max_len);
  if (len > 0U && diagnostics_ != nullptr) {
    const uint32_t now = millis();
    diagnostics_->stats().on_rx_chunk(now);
    diagnostics_->stats().on_rx_bytes(static_cast<uint32_t>(len), now);
  }
  this->update_transport_diagnostics_();
  return len;
}

bool MhiTransportManager::queue_tx(const MhiTxEnvelope& envelope) {
  if (active_ == nullptr) {
    if (diagnostics_ != nullptr) {
      diagnostics_->stats().on_tx_failure();
    }
    return false;
  }

  const bool queued = active_->queue_tx(envelope);
  this->update_transport_diagnostics_();
  return queued;
}

bool MhiTransportManager::take_tx_completion(MhiTxCompletion& completion) {
  return active_ != nullptr && active_->take_tx_completion(completion);
}

bool MhiTransportManager::has_pending_tx() const {
  return active_ != nullptr && active_->has_pending_tx();
}

bool MhiTransportManager::flush_tx_on_bus_marker() {
  if (active_ == nullptr) {
    return false;
  }
  const bool flushed = active_->flush_tx_on_bus_marker();
  this->update_transport_diagnostics_();
  return flushed;
}

std::size_t MhiTransportManager::tx_completion_queue_depth() const {
  return active_ == nullptr ? 0U : active_->tx_completion_queue_depth();
}

std::size_t MhiTransportManager::tx_completion_queue_high_water() const {
  return active_ == nullptr ? 0U : active_->tx_completion_queue_high_water();
}

uint32_t MhiTransportManager::tx_completion_queue_dropped() const {
  return active_ == nullptr ? 0U : active_->tx_completion_queue_dropped();
}

std::size_t MhiTransportManager::rx_queue_depth() const {
  return active_ == nullptr ? 0U : active_->rx_queue_depth();
}

std::size_t MhiTransportManager::rx_queue_high_water() const {
  return active_ == nullptr ? 0U : active_->rx_queue_high_water();
}

uint32_t MhiTransportManager::rx_queue_overwritten() const {
  return active_ == nullptr ? 0U : active_->rx_queue_overwritten();
}

void MhiTransportManager::set_auto_tx_flush(bool enabled) {
  if (primary_ != nullptr) {
    primary_->set_auto_tx_flush(enabled);
  }
  if (recovery_ != nullptr && recovery_ != primary_) {
    recovery_->set_auto_tx_flush(enabled);
  }
}

bool MhiTransportManager::auto_tx_flush() const {
  return active_ != nullptr && active_->auto_tx_flush();
}

void MhiTransportManager::set_rx_byte_critical_sections(bool enabled) {
  if (primary_ != nullptr) {
    primary_->set_rx_byte_critical_sections(enabled);
  }
  if (recovery_ != nullptr && recovery_ != primary_) {
    recovery_->set_rx_byte_critical_sections(enabled);
  }
}

bool MhiTransportManager::rx_byte_critical_sections() const {
  return active_ != nullptr && active_->rx_byte_critical_sections();
}

bool MhiTransportManager::tx_uses_bus_marker() const {
  return active_ != nullptr && active_->capabilities().uses_bus_marker;
}

const char* MhiTransportManager::rx_name() const {
  return active_ == nullptr ? "none" : active_->rx_name();
}

const char* MhiTransportManager::tx_name() const {
  return active_ == nullptr ? "none" : active_->tx_name();
}

void MhiTransportManager::update_transport_diagnostics_() {
  if (active_ == nullptr || diagnostics_ == nullptr) {
    return;
  }

  const uint32_t completed = active_->completed_tx_frames();
  const uint32_t failures = active_->tx_failures();

  while (last_transport_tx_completed_ != completed) {
    diagnostics_->stats().on_tx_frame(millis());
    last_transport_tx_completed_++;
  }

  while (last_transport_tx_failures_ != failures) {
    diagnostics_->stats().on_tx_failure();
    last_transport_tx_failures_++;
  }
}

void MhiTransportManager::reset_transport_diagnostic_cursors_() {
  last_transport_tx_completed_ = active_ == nullptr ? 0U : active_->completed_tx_frames();
  last_transport_tx_failures_ = active_ == nullptr ? 0U : active_->tx_failures();
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome
