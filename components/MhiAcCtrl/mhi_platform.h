#pragma once

#include <cmath>
#include <vector>

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#include "MHI-AC-Ctrl-core.h"
#include "mhi_status_listener.h"
#include "mhi_transport_gpio_frame_isr.h"
#include "mhi_transport_legacy.h"

namespace esphome {
namespace mhi {

enum TransportBackend : uint8_t {
  TRANSPORT_BACKEND_ESP32_FAST = 0,
  TRANSPORT_BACKEND_GPIO_FRAME_ISR = 1,
};

class MhiPlatform : public Component, public CallbackInterface_Status {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void cbiStatusFunction(ACStatus status, int value) override;

  void set_frame_size(int framesize);
  void set_protocol_mode(uint8_t mode);
  void set_room_temp_api_timeout(int time_in_seconds);
  void set_external_room_temperature_sensor(esphome::sensor::Sensor *sensor);

  void set_room_temperature(float value);
  bool get_room_temp_api_active();
  float get_room_temp_offset();

  void set_power(ACPower value);
  void set_fan(int value);
  void set_mode(ACMode value);
  void set_tsetpoint(float value);
  void set_offset(float value);
  void set_vanes(int value);
  void set_vanesLR(int value);
  void set_3Dauto(bool value);

  void add_listener(MhiStatusListener *listener);

  void set_transport_backend(uint8_t backend) {
    switch (backend) {
      case TRANSPORT_BACKEND_GPIO_FRAME_ISR:
        this->transport_backend_ = TRANSPORT_BACKEND_GPIO_FRAME_ISR;
        break;
      case TRANSPORT_BACKEND_ESP32_FAST:
      default:
        this->transport_backend_ = TRANSPORT_BACKEND_ESP32_FAST;
        break;
    }
  }
  void set_sck_pin(int pin) { this->sck_pin_ = pin; }
  void set_mosi_pin(int pin) { this->mosi_pin_ = pin; }
  void set_miso_pin(int pin) { this->miso_pin_ = pin; }

 protected:
  void transfer_room_temperature(float value);
  const char *get_transport_backend_name_() const;
  const char *get_protocol_mode_name_() const;

  MHI_AC_Ctrl_Core mhi_ac_ctrl_core_;
  MhiTransportLegacy transport_legacy_;
  MhiTransportGpioFrameIsr transport_gpio_frame_isr_;
  std::vector<MhiStatusListener *> listeners_;
  esphome::sensor::Sensor *external_temperature_sensor_{nullptr};

  int frame_size_{20};
  MhiProtocolMode protocol_mode_{MhiProtocolMode::AUTO};
  uint32_t room_temp_api_timeout_{60};
  uint32_t room_temp_api_timeout_start_{0};
  bool room_temp_api_active_{false};
  float last_room_temperature_{NAN};
  float temperature_offset_{0.0f};

  int sck_pin_{-1};
  int mosi_pin_{-1};
  int miso_pin_{-1};

  TransportBackend transport_backend_{TRANSPORT_BACKEND_ESP32_FAST};
};

}  // namespace mhi
}  // namespace esphome
