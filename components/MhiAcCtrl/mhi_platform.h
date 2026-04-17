#pragma once

#include <cmath>
#include <vector>

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#include "MHI-AC-Ctrl-core.h"
#include "mhi_status_listener.h"
#include "mhi_transport.h"

namespace esphome {
namespace mhi {

class MhiPlatform : public Component, public CallbackInterface_Status {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void cbiStatusFunction(ACStatus status, int value) override;

  void set_frame_size(int framesize);
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

  void set_sck_pin(int pin) { this->sck_pin_ = pin; }
  void set_mosi_pin(int pin) { this->mosi_pin_ = pin; }
  void set_miso_pin(int pin) { this->miso_pin_ = pin; }

 protected:
  void transfer_room_temperature(float value);

  MHI_AC_Ctrl_Core mhi_ac_ctrl_core_;
  MhiTransport transport_;
  std::vector<MhiStatusListener *> listeners_;
  esphome::sensor::Sensor *external_temperature_sensor_{nullptr};

  int frame_size_{20};
  uint32_t room_temp_api_timeout_{60};
  uint32_t room_temp_api_timeout_start_{0};
  bool room_temp_api_active_{false};
  float last_room_temperature_{NAN};
  float temperature_offset_{0.0f};

  int sck_pin_{-1};
  int mosi_pin_{-1};
  int miso_pin_{-1};
};

}  // namespace mhi
}  // namespace esphome
