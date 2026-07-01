#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "mhi_diag.h"
#include "mhi_fast_gpio_driver.h"
#include "mhi_rx_driver.h"
#include "mhi_transport_pins.h"
#include "mhi_tx_driver.h"

#ifdef USE_ESP_IDF
#include <sdkconfig.h>
#endif

#if defined(USE_ESP_IDF) && defined(CONFIG_IDF_TARGET_ESP32S3)
#define MHI_ENABLE_EXPERIMENTAL_S3_DRIVER 1
#else
#define MHI_ENABLE_EXPERIMENTAL_S3_DRIVER 0
#endif

#if defined(USE_ESP_IDF) && (defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3))
#define MHI_ENABLE_EXTERNAL_CLOCK_RX_DRIVER 1
#define MHI_ENABLE_SPLIT_TX_DRIVER 1
#include "mhi_external_clock_rx_driver.h"
#include "mhi_fast_gpio_tx_driver.h"
#include "mhi_null_tx_driver.h"
#else
#define MHI_ENABLE_EXTERNAL_CLOCK_RX_DRIVER 0
#define MHI_ENABLE_SPLIT_TX_DRIVER 0
#endif

#if MHI_ENABLE_EXPERIMENTAL_S3_DRIVER
#include "mhi_native_spi_rx_driver.h"
#endif

#define MHI_ENABLE_NATIVE_SPI_RX_DRIVER MHI_ENABLE_EXPERIMENTAL_S3_DRIVER

namespace esphome {
namespace mhi_ac_ctrl {

class MhiTransportManager {
 public:
  void configure(int sck_pin, int mosi_pin, int miso_pin, const std::string& rx_driver, const std::string& tx_driver,
                 uint8_t frame_size_hint = 20U, uint32_t frame_start_idle_ms = 10U,
                 uint32_t external_clock_byte_gap_us = 80U, uint32_t external_clock_frame_gap_us = 5000U,
                 uint32_t external_clock_min_edge_gap_us = 4U, const std::string& external_clock_edge = "falling",
                 uint32_t external_clock_sample_delay_nops = 0U);

  void set_diagnostics(MhiDiagnostics* diagnostics) {
    diagnostics_ = diagnostics;
  }

  bool setup();
  void loop();

  std::size_t read_rx(uint8_t* dst, std::size_t max_len);
  bool send_tx(const uint8_t* data, std::size_t len);
  void set_rx_byte_critical_sections(bool enabled);
  bool rx_byte_critical_sections() const;

  const char* rx_name() const;
  const char* tx_name() const;

  bool rx_ready() const {
    return rx_ready_;
  }
  bool tx_ready() const {
    return tx_ready_;
  }

 private:
  void resolve_drivers();

  MhiTransportPins pins_{};

  std::string rx_driver_name_{"fast_gpio"};
  std::string tx_driver_name_{"fast_gpio"};

  MhiFastGpioDriver fast_gpio_{};
#if MHI_ENABLE_SPLIT_TX_DRIVER
  MhiFastGpioTxDriver fast_gpio_tx_{};
  MhiNullTxDriver null_tx_{};
#endif
#if MHI_ENABLE_NATIVE_SPI_RX_DRIVER
  MhiNativeSpiRxDriver native_spi_rx_{};
#endif
#if MHI_ENABLE_EXTERNAL_CLOCK_RX_DRIVER
  MhiExternalClockRxDriver external_clock_rx_{};
#endif

  IMhiRxDriver* rx_{&fast_gpio_};
  IMhiTxDriver* tx_{&fast_gpio_};

  bool rx_ready_{false};
  bool tx_ready_{false};

  MhiDiagnostics* diagnostics_{nullptr};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome