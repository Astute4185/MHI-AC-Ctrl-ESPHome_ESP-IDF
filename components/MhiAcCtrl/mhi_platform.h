// Version 4.0
#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "MHI-AC-Ctrl-core.h"
#include "mhi_status_listener.h"
#include "mhi_transport_spi_idf.h"
#include "mhi_transport_legacy.h"

namespace esphome {
namespace sensor {
class Sensor;
}

namespace mhi {

class MhiPlatform : public Component, public CallbackInterface_Status {
 public:
  void setup() override;
  void set_frame_size(int framesize);
  void set_room_temp_api_timeout(int time_in_seconds);
  void loop() override;
  void dump_config() override;
  void cbiStatusFunction(ACStatus status, int value) override;

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
  void set_external_room_temperature_sensor(esphome::sensor::Sensor *sensor);
  void set_sck_pin(int pin) { this->sck_pin_ = pin; }
  void set_mosi_pin(int pin) { this->mosi_pin_ = pin; }
  void set_miso_pin(int pin) { this->miso_pin_ = pin; }
  void add_listener(MhiStatusListener *listener);

 private:
  void transfer_room_temperature(float value);

  float last_room_temperature_ = NAN;
  float temperature_offset_ = 0.0f;

  int frame_size_ = 20;
  uint32_t room_temp_api_timeout_start_ = 0;
  uint32_t room_temp_api_timeout_ = 0;
  bool room_temp_api_active_ = false;

  int sck_pin_ = -1;
  int mosi_pin_ = -1;
  int miso_pin_ = -1;

  MhiTransportLegacy transport_;
  MhiTransportSpiIdf transport_spi_;
  MHI_AC_Ctrl_Core mhi_ac_ctrl_core_;

  esphome::sensor::Sensor *external_temperature_sensor_{nullptr};

  std::vector<MhiStatusListener *> listeners_;
};

}  // namespace mhi
}  // namespace esphome