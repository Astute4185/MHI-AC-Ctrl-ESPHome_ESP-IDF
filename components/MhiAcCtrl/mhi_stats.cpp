#include "mhi_stats.h"

namespace esphome {
namespace mhi_ac_ctrl {

void MhiStats::reset() {
  stats_ = {};
}

void MhiStats::on_rx_bytes(uint32_t count, uint32_t now_ms) {
  stats_.rx_bytes += count;
  stats_.last_rx_byte_ms = now_ms;
}

void MhiStats::on_rx_chunk(uint32_t now_ms) {
  stats_.rx_chunks++;
  stats_.last_rx_byte_ms = now_ms;
}

void MhiStats::on_candidate_frame() {
  stats_.candidate_frames++;
}

void MhiStats::on_valid_frame(uint32_t now_ms) {
  stats_.valid_frames++;
  stats_.last_valid_frame_ms = now_ms;
}

void MhiStats::on_invalid_frame() {
  stats_.invalid_frames++;
}

void MhiStats::on_checksum_failure() {
  stats_.checksum_failures++;
}

void MhiStats::on_signature_miss() {
  stats_.signature_misses++;
}

void MhiStats::on_sync_loss() {
  stats_.sync_losses++;
}

void MhiStats::on_dropped_bytes(uint32_t count) {
  stats_.dropped_bytes += count;
}

void MhiStats::on_tx_frame(uint32_t now_ms) {
  stats_.tx_frames++;
  stats_.last_tx_frame_ms = now_ms;
}

void MhiStats::on_tx_failure() {
  stats_.tx_failures++;
}

void MhiStats::on_tx_command_frame(uint32_t command_mask, uint32_t now_ms) {
  stats_.tx_command_frames++;
  stats_.last_tx_command_frame_ms = now_ms;
  stats_.last_tx_command_mask = command_mask;
}

void MhiStats::on_tx_command_failure(uint32_t command_mask, uint32_t now_ms) {
  stats_.tx_command_failures++;
  stats_.last_tx_command_frame_ms = now_ms;
  stats_.last_tx_command_mask = command_mask;
}

void MhiStats::on_unsupported_command(uint32_t command_mask, uint32_t now_ms) {
  stats_.unsupported_commands++;
  stats_.last_unsupported_command_ms = now_ms;
  stats_.last_unsupported_command_mask = command_mask;
}

void MhiStats::on_command_confirmed(uint32_t command_mask, uint32_t now_ms) {
  stats_.command_confirmations++;
  stats_.last_command_confirmation_ms = now_ms;
  stats_.last_confirmed_command_mask = command_mask;
}

void MhiStats::on_command_confirmation_timeout(uint32_t command_mask, uint32_t now_ms) {
  stats_.command_confirmation_timeouts++;
  stats_.last_command_confirmation_timeout_ms = now_ms;
  stats_.last_command_confirmation_timeout_mask = command_mask;
}

void MhiStats::on_loop_timing(uint32_t loop_us, uint32_t transport_loop_us, uint32_t tx_stage_us,
                              uint32_t rx_read_sync_us, uint32_t publish_us, uint32_t command_housekeeping_us,
                              uint32_t budget_us, uint32_t now_ms) {
  stats_.loop_iterations++;
  stats_.loop_budget_us = budget_us;

  const uint32_t sample_count = stats_.loop_iterations;

  update_timing_sample(loop_us, sample_count, stats_.loop_last_us, stats_.loop_avg_us, stats_.loop_max_us);
  update_timing_sample(transport_loop_us, sample_count, stats_.transport_loop_last_us, stats_.transport_loop_avg_us,
                       stats_.transport_loop_max_us);
  update_timing_sample(tx_stage_us, sample_count, stats_.tx_stage_last_us, stats_.tx_stage_avg_us,
                       stats_.tx_stage_max_us);
  update_timing_sample(rx_read_sync_us, sample_count, stats_.rx_read_sync_last_us, stats_.rx_read_sync_avg_us,
                       stats_.rx_read_sync_max_us);
  update_timing_sample(publish_us, sample_count, stats_.publish_last_us, stats_.publish_avg_us, stats_.publish_max_us);
  update_timing_sample(command_housekeeping_us, sample_count, stats_.command_housekeeping_last_us,
                       stats_.command_housekeeping_avg_us, stats_.command_housekeeping_max_us);

  if (budget_us > 0U && loop_us > budget_us) {
    stats_.loop_over_budget++;
    stats_.last_loop_over_budget_ms = now_ms;
  }
}

void MhiStats::on_rx_worker_timing(uint32_t sample_us, uint32_t now_ms) {
  (void)now_ms;

  stats_.rx_worker_samples++;
  update_timing_sample(sample_us, stats_.rx_worker_samples, stats_.rx_worker_last_us, stats_.rx_worker_avg_us,
                       stats_.rx_worker_max_us);
}

void MhiStats::on_rx_worker_frame_queued(uint32_t queue_depth, uint32_t now_ms) {
  stats_.rx_worker_frames++;
  stats_.rx_worker_queue_depth = queue_depth;
  stats_.last_rx_worker_frame_ms = now_ms;

  if (queue_depth > stats_.rx_worker_queue_max_depth) {
    stats_.rx_worker_queue_max_depth = queue_depth;
  }
}

void MhiStats::on_rx_worker_frame_drained(uint32_t queue_depth, uint32_t now_ms) {
  stats_.rx_worker_drained_frames++;
  stats_.rx_worker_queue_depth = queue_depth;
  stats_.last_rx_worker_drained_frame_ms = now_ms;
}

void MhiStats::on_rx_worker_queue_overflow(uint32_t queue_depth, uint32_t now_ms) {
  stats_.rx_worker_queue_overflows++;
  stats_.queue_overflows++;
  stats_.rx_worker_queue_depth = queue_depth;
  stats_.last_rx_worker_queue_overflow_ms = now_ms;

  if (queue_depth > stats_.rx_worker_queue_max_depth) {
    stats_.rx_worker_queue_max_depth = queue_depth;
  }
}

void MhiStats::on_rx_worker_health_check(bool no_frames, bool stalled, bool not_draining, uint32_t now_ms) {
  stats_.rx_worker_health_checks++;

  if (no_frames) {
    stats_.rx_worker_no_frame_windows++;
    stats_.last_rx_worker_no_frame_window_ms = now_ms;
  }

  if (stalled) {
    stats_.rx_worker_stalls++;
    stats_.last_rx_worker_stall_ms = now_ms;
  }

  if (not_draining) {
    stats_.rx_worker_not_draining_windows++;
    stats_.last_rx_worker_not_draining_ms = now_ms;
  }
}

void MhiStats::on_frame_sync_reset() {
  stats_.frame_sync_resets++;
}

void MhiStats::on_queue_overflow() {
  stats_.queue_overflows++;
}

void MhiStats::set_last_error_code(uint8_t error_code) {
  stats_.last_error_code = error_code;
}

MhiStatsSnapshot MhiStats::snapshot() const {
  return stats_;
}

uint32_t MhiStats::updated_average(uint32_t current_average, uint32_t sample_count, uint32_t sample_us) {
  if (sample_count <= 1U) {
    return sample_us;
  }

  const uint64_t previous_total = static_cast<uint64_t>(current_average) * static_cast<uint64_t>(sample_count - 1U);
  const uint64_t next_total = previous_total + static_cast<uint64_t>(sample_us);

  return static_cast<uint32_t>(next_total / static_cast<uint64_t>(sample_count));
}

void MhiStats::update_timing_sample(uint32_t sample_us, uint32_t sample_count, uint32_t& last_us, uint32_t& avg_us,
                                    uint32_t& max_us) {
  last_us = sample_us;
  avg_us = updated_average(avg_us, sample_count, sample_us);

  if (sample_us > max_us) {
    max_us = sample_us;
  }
}

}  // namespace mhi_ac_ctrl
}  // namespace esphome