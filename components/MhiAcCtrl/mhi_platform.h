#pragma once

#include <cmath>
#include <cstdint>
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
  void set_transport_backend(MhiTransportBackend backend) { this->transport_backend_ = backend; }
  void set_raw_dump_enable(bool value) { this->raw_dump_enable_ = value; }
  void set_raw_dump_rate_ms(uint32_t value) { this->raw_dump_rate_ms_ = value; }
  void set_raw_chunk_bytes(uint32_t value) { this->raw_chunk_bytes_ = value; }
  void set_sync_gap_us(uint32_t value) { this->sync_gap_us_ = value; }
  void set_tx_suppress_during_capture(bool value) { this->tx_suppress_during_capture_ = value; }

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

  void add_opdata_mask(uint32_t mask) {
    this->opdata_mask_ |= mask;
    this->mhi_ac_ctrl_core_.set_enabled_opdata_mask(this->opdata_mask_);
  }

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
  MhiTransportBackend transport_backend_{MhiTransportBackend::GPIO};
  bool raw_dump_enable_{false};
  uint32_t raw_dump_rate_ms_{15000};
  uint32_t raw_chunk_bytes_{24};
  uint32_t sync_gap_us_{5000};
  bool tx_suppress_during_capture_{false};

  uint32_t opdata_mask_{kMhiDefaultOpdataMask};
};

}  // namespace mhi
}  // namespace esphome