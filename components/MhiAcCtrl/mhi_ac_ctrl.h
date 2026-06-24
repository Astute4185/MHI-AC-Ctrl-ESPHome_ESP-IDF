#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "mhi_command_confirmation.h"
#include "mhi_defs.h"
#include "mhi_diag.h"
#include "mhi_frame_sync.h"
#include "mhi_opdata_decoder.h"
#include "mhi_publish_bridge.h"
#include "mhi_rx_worker.h"
#include "mhi_rx_worker_mode.h"
#include "mhi_state.h"
#include "mhi_status_decoder.h"
#include "mhi_transport_manager.h"
#include "mhi_tx_builder.h"

namespace esphome {
namespace mhi_ac_ctrl {

struct MhiPins {
  int sck{-1};
  int mosi{-1};
  int miso{-1};
};

class MhiAcCtrl : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_frame_size(int frame_size) {
    this->frame_size_ = frame_size;
  }

  void set_sck_pin(int pin) {
    this->pins_.sck = pin;
  }
  void set_mosi_pin(int pin) {
    this->pins_.mosi = pin;
  }
  void set_miso_pin(int pin) {
    this->pins_.miso = pin;
  }

  void set_rx_driver(const std::string& driver) {
    this->rx_driver_ = driver;
  }
  void set_tx_driver(const std::string& driver) {
    this->tx_driver_ = driver;
  }

  void set_rx_worker_enabled(bool enabled) {
    this->rx_worker_mode_ = enabled ? MhiRxWorkerMode::ENABLED : MhiRxWorkerMode::DISABLED;
  }

  void set_rx_worker_mode_auto() {
    this->rx_worker_mode_ = MhiRxWorkerMode::AUTO;
  }

  void set_rx_worker_mode_enabled() {
    this->rx_worker_mode_ = MhiRxWorkerMode::ENABLED;
  }

  void set_rx_worker_mode_disabled() {
    this->rx_worker_mode_ = MhiRxWorkerMode::DISABLED;
  }

  void set_rx_worker_mode(MhiRxWorkerMode mode) {
    this->rx_worker_mode_ = mode;
  }

  // Compatibility with current __init__.py plumbing.
  void set_room_temp_api_timeout(int timeout_s) {
    this->room_temp_api_timeout_s_ = timeout_s;
  }

  void set_external_room_temperature_sensor(sensor::Sensor* sensor) {
    this->external_room_temperature_sensor_ = sensor;
  }

  // Compatibility aliases for old config names.
  void set_vanes(int position) {
    this->initial_vertical_vanes_position_ = position;
  }

  void set_vanesLR(int position) {
    this->initial_horizontal_vanes_position_ = position;
  }

  void set_initial_vertical_vanes_position(int position) {
    this->initial_vertical_vanes_position_ = position;
  }

  void set_initial_horizontal_vanes_position(int position) {
    this->initial_horizontal_vanes_position_ = position;
  }

  void add_opdata_mask(uint32_t mask) {
    this->opdata_mask_ |= mask;
    this->tx_config_.enabled_opdata_mask = this->opdata_mask_;
  }

  void set_publish_targets(const MhiPublishTargets& targets) {
    this->publish_targets_ = targets;
    this->refresh_publish_targets_();
  }

  void set_climate_target(climate::Climate* climate) {
    this->publish_targets_.climate = climate;
    this->refresh_publish_targets_();
  }

  void set_power_binary_sensor(binary_sensor::BinarySensor* sensor) {
    this->publish_targets_.power_binary_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_room_temp_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.room_temp_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_target_temp_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.target_temp_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_outdoor_temp_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.outdoor_temp_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_return_air_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.return_air_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_compressor_frequency_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.compressor_frequency_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_current_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.current_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_indoor_unit_fan_speed_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.indoor_unit_fan_speed_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_outdoor_unit_fan_speed_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.outdoor_unit_fan_speed_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_indoor_unit_total_run_time_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.indoor_unit_total_run_time_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_compressor_total_run_time_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.compressor_total_run_time_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_energy_used_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.energy_used_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_indoor_unit_thi_r1_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.indoor_unit_thi_r1_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_indoor_unit_thi_r2_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.indoor_unit_thi_r2_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_indoor_unit_thi_r3_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.indoor_unit_thi_r3_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_outdoor_unit_tho_r1_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.outdoor_unit_tho_r1_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_outdoor_unit_expansion_valve_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.outdoor_unit_expansion_valve_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_outdoor_unit_discharge_pipe_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.outdoor_unit_discharge_pipe_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_outdoor_unit_discharge_pipe_super_heat_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.outdoor_unit_discharge_pipe_super_heat_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_protection_state_number_sensor(sensor::Sensor* sensor) {
    this->publish_targets_.protection_state_number_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_defrost_binary_sensor(binary_sensor::BinarySensor* sensor) {
    this->publish_targets_.defrost_binary_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_vanes_3d_auto_enabled_binary_sensor(binary_sensor::BinarySensor* sensor) {
    this->publish_targets_.vanes_3d_auto_enabled_binary_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_vanes_3d_auto_switch(switch_::Switch* sw) {
    this->publish_targets_.vanes_3d_auto_switch = sw;
    this->refresh_publish_targets_();
  }

  void set_error_code_text_sensor(text_sensor::TextSensor* sensor) {
    this->publish_targets_.error_code_text_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_protection_state_text_sensor(text_sensor::TextSensor* sensor) {
    this->publish_targets_.protection_state_text_sensor = sensor;
    this->refresh_publish_targets_();
  }

  void set_vertical_vanes_select(select::Select* select) {
    this->publish_targets_.vertical_vanes_select = select;
    this->refresh_publish_targets_();
  }

  void set_horizontal_vanes_select(select::Select* select) {
    this->publish_targets_.horizontal_vanes_select = select;
    this->refresh_publish_targets_();
  }

  void set_fan_speed_select(select::Select* select) {
    this->publish_targets_.fan_speed_select = select;
    this->refresh_publish_targets_();
  }

  MhiStateStore& state() {
    return this->state_;
  }
  const MhiStateStore& state() const {
    return this->state_;
  }

  MhiDiagnostics& diagnostics() {
    return this->diagnostics_;
  }
  const MhiDiagnostics& diagnostics() const {
    return this->diagnostics_;
  }

 protected:
  void refresh_publish_targets_();
  void build_and_stage_tx_frame_();
  void record_tx_build_result_(const MhiTxBuildResult& result, const MhiFrameBuffer& frame, bool sent);
  uint32_t drain_rx_worker_frames_(bool& state_changed);
  bool read_and_sync_rx_frame_();
  bool decode_frame_(const MhiFrameBuffer& frame);
  void log_runtime_diagnostics_();
  void check_rx_worker_health_();
  void update_command_confirmation_(const MhiStatusState& status);
  void check_command_confirmation_timeout_();
  void suppress_duplicate_pending_commands_();

  static uint32_t elapsed_us_(uint32_t start_us);
  static uint8_t detect_chip_core_count_();

  int frame_size_{20};
  int room_temp_api_timeout_s_{60};

  int initial_vertical_vanes_position_{0};
  int initial_horizontal_vanes_position_{0};

  MhiPins pins_{};

  std::string rx_driver_{"fast_gpio"};
  std::string tx_driver_{"fast_gpio"};

  sensor::Sensor* external_room_temperature_sensor_{nullptr};

  uint32_t opdata_mask_{kMhiDefaultOpdataMask};

  MhiStateStore state_{};
  MhiFrameSync frame_sync_{};
  MhiTransportManager transport_{};
  MhiRxWorker rx_worker_{};
  MhiDiagnostics diagnostics_{};

  MhiPublishTargets publish_targets_{};
  MhiPublishBridge publish_bridge_{};

  MhiTxRuntime tx_runtime_{};
  MhiTxBuildConfig tx_config_{};
  MhiCommandConfirmation command_confirmation_{};

  MhiRxWorkerMode rx_worker_mode_{MhiRxWorkerMode::AUTO};
  bool rx_worker_enabled_{false};
  bool using_rx_worker_{false};
  uint8_t chip_core_count_{1};
  bool publish_requested_{false};

  uint32_t rx_worker_started_ms_{0};
  uint32_t last_rx_worker_health_check_ms_{0};
  uint32_t last_rx_worker_health_frames_{0};
  uint32_t last_rx_worker_health_drained_frames_{0};
  uint32_t last_rx_worker_health_overflows_{0};
  bool rx_worker_startup_warning_logged_{false};
  bool rx_worker_stall_warning_logged_{false};
  bool rx_worker_not_draining_warning_logged_{false};

  uint32_t last_diag_log_ms_{0};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
