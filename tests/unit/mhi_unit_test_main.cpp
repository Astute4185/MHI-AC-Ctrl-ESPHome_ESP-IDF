
#include <iostream>

#include "mhi_test_common.h"

int main() {
  using namespace mhi_unit_tests;

  checksum_accepts_valid_20_byte_frame();
  checksum_rejects_bad_20_byte_frame();

  frame_sync_discards_garbage_and_extracts_valid_frame();
  frame_sync_waits_for_partial_frame();
  frame_sync_records_resync_stats();
  frame_sync_records_checksum_failure_stats();
  frame_sync_33_byte_mode_consumes_full_frame_without_tail_resync_noise();

  status_decoder_decodes_core_fields();
  status_decoder_decodes_33_byte_vane_feedback();

  opdata_decoder_decodes_outdoor_temp();
  opdata_decoder_decodes_return_air_temp();
  opdata_decoder_decodes_compressor_frequency();
  opdata_decoder_decodes_current();
  opdata_decoder_decodes_indoor_unit_fan_speed();
  opdata_decoder_decodes_outdoor_unit_fan_speed();
  opdata_decoder_decodes_indoor_unit_total_run_time();
  opdata_decoder_decodes_compressor_total_run_time();
  opdata_decoder_decodes_energy_used();
  opdata_decoder_decodes_temperature_and_protection_slice2();

  publish_bridge_republishes_cached_state_after_targets_are_registered();
  publish_bridge_publishes_sensor_parity_slice1_on_first_opdata_publish();
  publish_bridge_publishes_sensor_parity_slice2_on_first_opdata_publish();
  publish_bridge_maps_unknown_protection_state();
  publish_bridge_publishes_sensor_parity_slice3_vane_feedback();

  tx_builder_emits_valid_default_20_byte_frame();
  tx_builder_applies_pending_commands_once();
  tx_builder_uses_configured_sensor_parity_opdata_mask();
  tx_builder_uses_configured_sensor_parity_slice2_opdata_mask();
  tx_builder_reports_encoded_command_mask();
  tx_builder_keeps_double_frame_commands_pending_until_command_frame();
  tx_builder_drops_33_byte_only_commands_in_20_byte_mode();
  tx_builder_applies_3d_auto_in_33_byte_frame();

  command_confirmation_confirms_power_mode_and_vertical_vane();
  command_confirmation_keeps_partial_pending_until_later_status();
  command_confirmation_ignores_auto_fan_because_mosi_does_not_confirm_it();
  command_confirmation_confirms_supported_fan_codes();
  command_confirmation_times_out_unconfirmed_commands();
  command_confirmation_detects_duplicate_pending_commands();
  command_state_clears_pending_mask();

  diagnostics_snapshot_reports_event_ages();
  diagnostics_snapshot_reports_command_event_ages();
  diagnostics_snapshot_reports_command_confirmation_event_ages();
  diagnostics_snapshot_reports_loop_budget_timing();
  diagnostics_snapshot_reports_rx_worker_timing();
  diagnostics_snapshot_reports_rx_worker_health_counters();
  diagnostics_snapshot_handles_missing_event_ages();

  rx_worker_mode_resolves_auto_by_core_count();
  rx_worker_mode_allows_explicit_override();
  rx_worker_mode_reports_config_names();
  rx_frame_queue_preserves_fifo_order();
  rx_frame_queue_rejects_overflow_and_invalid_inputs();

  fixture_valid_status_frame_decodes();
  fixture_bad_checksum_rejects();
  fixture_garbage_then_valid_frame_resyncs();
  fixture_opdata_outdoor_temp_decodes();
  fixture_opdata_current_decodes();

  std::cout << "MHI protocol unit tests passed\n";
  return 0;
}
