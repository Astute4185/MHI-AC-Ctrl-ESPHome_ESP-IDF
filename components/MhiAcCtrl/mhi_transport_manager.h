#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "mhi_diag.h"
#include "mhi_fast_gpio_driver.h"
#include "mhi_rx_driver.h"
#include "mhi_transport_pins.h"
#include "mhi_tx_driver.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiTransportManager {
 public:
  void configure(int sck_pin, int mosi_pin, int miso_pin, const std::string& rx_driver, const std::string& tx_driver,
                 uint8_t frame_size_hint = 20U, uint32_t frame_start_idle_ms = 10U);

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

  IMhiRxDriver* rx_{&fast_gpio_};
  IMhiTxDriver* tx_{&fast_gpio_};

  bool rx_ready_{false};
  bool tx_ready_{false};

  MhiDiagnostics* diagnostics_{nullptr};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome