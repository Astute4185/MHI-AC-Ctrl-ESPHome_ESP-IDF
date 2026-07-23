#include "mhi_test_common.h"

namespace mhi_unit_tests {
namespace {

MhiDecodedStatus make_trace_status(bool power, uint8_t mode, uint8_t fan, float target,
                                   uint8_t horizontal, bool three_d) {
  MhiDecodedStatus status{};
  status.valid = true;
  status.power = power;
  status.mode = mode;
  status.fan = fan;
  status.target_temp_c = target;
  status.vertical_vane = 2U;
  status.has_horizontal_vane = true;
  status.horizontal_vane = horizontal;
  status.has_3d_auto = true;
  status.three_d_auto = three_d;
  status.has_extended_louver_raw = true;
  status.extended_louver_db16 = static_cast<uint8_t>(horizontal - 1U);
  status.extended_louver_db17 = three_d ? 0x04U : 0x00U;
  return status;
}

void finalize_trace_frame(MhiFrameBuffer& frame) {
  if (frame.len == kMhiFrame33Bytes) {
    const uint16_t checksum = mhi_calc_checksum_frame33(frame.data);
    frame.data[CBL2] = static_cast<uint8_t>(checksum & 0xFFU);
    return;
  }
  const uint16_t checksum = mhi_calc_checksum(frame.data);
  frame.data[CBH] = static_cast<uint8_t>((checksum >> 8U) & 0xFFU);
  frame.data[CBL] = static_cast<uint8_t>(checksum & 0xFFU);
}

void observe_trace_status(MhiProtocolTraceRecorder& recorder, const MhiFrameBuffer& frame,
                          const MhiDecodedStatus& status, bool candidate,
                          uint32_t sequence, uint32_t now_ms, uint32_t now_us,
                          uint32_t pending_mask) {
  recorder.observe_mosi_status(frame, status, candidate, sequence, sequence,
                               now_us - 50U, now_ms, now_us, pending_mask);
}

MhiTxEnvelope make_trace_command_envelope(const MhiFrameBuffer& frame, uint32_t generation,
                                          uint32_t mask) {
  MhiTxEnvelope envelope{};
  EXPECT_TRUE(envelope.set_frame(frame.data, frame.len));
  envelope.generation = generation;
  envelope.kind = MhiTxKind::COMMAND;
  envelope.command_mask = mask;
  return envelope;
}

MhiTxEnvelope make_trace_background_envelope(const MhiFrameBuffer& frame) {
  MhiTxEnvelope envelope{};
  EXPECT_TRUE(envelope.set_frame(frame.data, frame.len));
  envelope.kind = MhiTxKind::BACKGROUND;
  return envelope;
}

MhiTxCompletion completion_for_trace(const MhiTxEnvelope& envelope, bool success,
                                     uint32_t bus_sequence, uint32_t frame_end_us) {
  MhiTxCompletion completion{};
  completion.generation = envelope.generation;
  completion.kind = envelope.kind;
  completion.command_mask = envelope.command_mask;
  completion.intent = envelope.intent;
  completion.success = success;
  completion.completed_at_ms = frame_end_us / 1000U;
  completion.completed_at_us = frame_end_us;
  completion.bus_sequence = bus_sequence;
  completion.frame_end_us = frame_end_us;
  completion.expected_len = static_cast<uint16_t>(envelope.len);
  completion.actual_len = success ? static_cast<uint16_t>(envelope.len) : 0U;
  completion.set_transmitted_frame(envelope);
  return completion;
}

}  // namespace

void protocol_trace_captures_baseline_tx_and_stable_feedback() {
  MhiProtocolTraceRecorder recorder{};
  MhiProtocolTraceConfig config{};
  config.enabled = true;
  config.pre_command_frames = 2U;
  config.max_records = 32U;
  recorder.configure(config);

  MhiFrameBuffer baseline = make_mosi_status_frame_33(2U, false, false);
  const auto baseline_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, false);
  observe_trace_status(recorder, baseline, baseline_status, false, 1U, 100U, 100000U, 0U);
  observe_trace_status(recorder, baseline, baseline_status, false, 2U, 110U, 110000U, 0U);

  MhiFrameBuffer background = baseline;
  background.data[SB0] = kMhiMisoSignature0;
  background.data[SB1] = kMhiMisoSignature1;
  background.data[SB2] = kMhiMisoSignature2;
  finalize_trace_frame(background);
  recorder.observe_tx_staged(make_trace_background_envelope(background), 115000U, 0U);

  MhiCommandState command{};
  command.three_d_auto_set = true;
  command.three_d_auto = true;
  const auto requested = MhiProtocolTraceRecorder::semantic_from_command(
      command, MHI_COMMAND_THREE_D_AUTO);
  EXPECT_TRUE(recorder.start_capture(MHI_COMMAND_THREE_D_AUTO, requested, 0U,
                                     120U, 120000U, "3d_on"));
  EXPECT_EQ(recorder.record_count(), 3U);

  MhiFrameBuffer command_frame = background;
  command_frame.data[DB17] = 0x04U;
  finalize_trace_frame(command_frame);
  const auto envelope = make_trace_command_envelope(
      command_frame, 7U, MHI_COMMAND_THREE_D_AUTO);
  recorder.observe_tx_staged(envelope, 121000U, 0U);
  const auto completion = completion_for_trace(envelope, true, 3U, 122000U);
  recorder.observe_tx_completion(completion, 122100U, MHI_COMMAND_THREE_D_AUTO);

  observe_trace_status(recorder, baseline, baseline_status, true, 4U, 123U,
                       123000U, MHI_COMMAND_THREE_D_AUTO);
  EXPECT_FALSE(recorder.ready());

  MhiFrameBuffer changed = make_mosi_status_frame_33(2U, false, true);
  auto changed_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, true);
  observe_trace_status(recorder, changed, changed_status, true, 5U, 125U,
                       125000U, MHI_COMMAND_THREE_D_AUTO);
  observe_trace_status(recorder, changed, changed_status, true, 6U, 126U,
                       126000U, MHI_COMMAND_THREE_D_AUTO);
  observe_trace_status(recorder, changed, changed_status, true, 7U, 127U,
                       127000U, MHI_COMMAND_THREE_D_AUTO);

  EXPECT_TRUE(recorder.active());
  EXPECT_TRUE(recorder.summary().stable_status_seen);
  EXPECT_TRUE(recorder.summary().requested_state_seen);
  recorder.observe_confirmation(MHI_COMMAND_THREE_D_AUTO, 0U, 128000U);

  EXPECT_TRUE(recorder.ready());
  EXPECT_FALSE(recorder.active());
  EXPECT_EQ(static_cast<uint8_t>(recorder.completion_reason()),
            static_cast<uint8_t>(MhiProtocolTraceCompletionReason::CONFIRMED));
  EXPECT_EQ(static_cast<uint8_t>(recorder.summary().assessment),
            static_cast<uint8_t>(MhiProtocolTraceAssessment::CONFIRMED));
  EXPECT_EQ(recorder.capture_generation(), 7U);
  EXPECT_TRUE(recorder.summary().stage_to_wire_us > 0U);
  EXPECT_TRUE(recorder.summary().tx_to_first_rx_us > 0U);
  EXPECT_TRUE(recorder.begin_export());
  EXPECT_FALSE(recorder.begin_export());
  recorder.finish_export();
  EXPECT_FALSE(recorder.ready());
}

void protocol_trace_manual_capture_waits_for_a_real_change() {
  MhiProtocolTraceRecorder recorder{};
  MhiProtocolTraceConfig config{};
  config.enabled = true;
  config.pre_command_frames = 1U;
  config.post_command_frames = 12U;
  recorder.configure(config);

  MhiFrameBuffer baseline = make_mosi_status_frame();
  auto baseline_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, false);
  observe_trace_status(recorder, baseline, baseline_status, false, 1U, 10U,
                       10000U, 0U);
  EXPECT_TRUE(recorder.start_capture(0U, {}, 0U, 20U, 20000U, "remote_power"));

  for (uint32_t i = 0U; i < 4U; i++) {
    observe_trace_status(recorder, baseline, baseline_status, false, i + 2U,
                         21U + i, 21000U + (i * 1000U), 0U);
  }
  EXPECT_FALSE(recorder.ready());

  MhiFrameBuffer changed = baseline;
  changed.data[DB0] ^= 0x01U;
  finalize_trace_frame(changed);
  auto changed_status = baseline_status;
  changed_status.power = true;
  for (uint32_t i = 0U; i < 3U; i++) {
    observe_trace_status(recorder, changed, changed_status, false, i + 6U,
                         30U + i, 30000U + (i * 1000U), 0U);
  }
  EXPECT_TRUE(recorder.ready());
  EXPECT_EQ(static_cast<uint8_t>(recorder.completion_reason()),
            static_cast<uint8_t>(
                MhiProtocolTraceCompletionReason::PASSIVE_STABLE_STATUS));
  EXPECT_EQ(static_cast<uint8_t>(recorder.summary().assessment),
            static_cast<uint8_t>(
                MhiProtocolTraceAssessment::PASSIVE_CHANGE_CAPTURED));
}

void protocol_trace_rejects_overlapping_captures_and_is_bounded() {
  MhiProtocolTraceRecorder recorder{};
  MhiProtocolTraceConfig config{};
  config.enabled = true;
  config.pre_command_frames = 0U;
  config.max_records = 8U;
  config.unchanged_heartbeat_ms = 250U;
  recorder.configure(config);

  MhiFrameBuffer baseline = make_mosi_status_frame();
  auto status = make_trace_status(false, 2U, 1U, 20.0f, 2U, false);
  observe_trace_status(recorder, baseline, status, false, 1U, 1U, 1000U, 0U);

  EXPECT_TRUE(recorder.start_capture(MHI_COMMAND_POWER, {}, 0U, 2U, 2000U,
                                     "first"));
  EXPECT_FALSE(recorder.start_capture(MHI_COMMAND_MODE, {}, 0U, 3U, 3000U,
                                      "second"));
  EXPECT_EQ(recorder.stats().busy_rejections, 1U);

  MhiFrameBuffer tx = baseline;
  tx.data[SB0] = kMhiMisoSignature0;
  tx.data[SB1] = kMhiMisoSignature1;
  tx.data[SB2] = kMhiMisoSignature2;
  finalize_trace_frame(tx);
  const auto envelope = make_trace_command_envelope(tx, 3U, MHI_COMMAND_POWER);
  recorder.observe_tx_staged(envelope, 4000U, 0U);
  recorder.observe_tx_completion(completion_for_trace(envelope, true, 1U, 5000U),
                                 5100U, MHI_COMMAND_POWER);

  for (uint32_t i = 0U; i < 20U && recorder.active(); i++) {
    MhiFrameBuffer frame = baseline;
    frame.data[DB4] = static_cast<uint8_t>(i + 1U);
    finalize_trace_frame(frame);
    observe_trace_status(recorder, frame, status, true, i + 2U, 10U + i,
                         10000U + (i * 1000U), MHI_COMMAND_POWER);
  }
  EXPECT_TRUE(recorder.ready());
  EXPECT_EQ(static_cast<uint8_t>(recorder.completion_reason()),
            static_cast<uint8_t>(MhiProtocolTraceCompletionReason::RECORD_LIMIT));
  EXPECT_TRUE(recorder.record_count() <= 8U);
}

void protocol_trace_semantic_mapping_preserves_requested_fields() {
  MhiCommandState command{};
  command.power_set = true;
  command.power = true;
  command.fan_set = true;
  command.fan = 4U;
  command.horizontal_vane_set = true;
  command.horizontal_vane = 8U;
  const uint32_t mask =
      MHI_COMMAND_POWER | MHI_COMMAND_FAN | MHI_COMMAND_HORIZONTAL_VANE;
  const auto semantic = MhiProtocolTraceRecorder::semantic_from_command(command, mask);
  EXPECT_TRUE((semantic.valid_fields & MHI_TRACE_STATE_POWER) != 0U);
  EXPECT_TRUE((semantic.valid_fields & MHI_TRACE_STATE_FAN) != 0U);
  EXPECT_TRUE((semantic.valid_fields & MHI_TRACE_STATE_HORIZONTAL_VANE) != 0U);
  EXPECT_TRUE(semantic.power);
  EXPECT_EQ(semantic.fan, 4U);
  EXPECT_EQ(semantic.horizontal_vane, 8U);
  EXPECT_TRUE(semantic.horizontal_swing);
}

void protocol_trace_classifies_confirmation_logic_mismatch() {
  MhiProtocolTraceRecorder recorder{};
  MhiProtocolTraceConfig config{};
  config.enabled = true;
  config.post_timeout_grace_ms = 250U;
  recorder.configure(config);

  MhiFrameBuffer baseline = make_mosi_status_frame_33(2U, false, false);
  auto baseline_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, false);
  observe_trace_status(recorder, baseline, baseline_status, false, 1U, 10U,
                       10000U, 0U);

  MhiCommandState command{};
  command.three_d_auto_set = true;
  command.three_d_auto = true;
  const auto requested = MhiProtocolTraceRecorder::semantic_from_command(
      command, MHI_COMMAND_THREE_D_AUTO);
  EXPECT_TRUE(recorder.start_capture(MHI_COMMAND_THREE_D_AUTO, requested, 0U,
                                     20U, 20000U, "logic_mismatch"));

  MhiFrameBuffer tx = baseline;
  tx.data[SB0] = kMhiMisoSignature0;
  tx.data[SB1] = kMhiMisoSignature1;
  tx.data[SB2] = kMhiMisoSignature2;
  tx.data[DB17] = 0x04U;
  finalize_trace_frame(tx);
  const auto envelope = make_trace_command_envelope(
      tx, 9U, MHI_COMMAND_THREE_D_AUTO);
  recorder.observe_tx_staged(envelope, 21000U, 0U);
  recorder.observe_tx_completion(completion_for_trace(envelope, true, 2U, 22000U),
                                 22100U, MHI_COMMAND_THREE_D_AUTO);

  MhiFrameBuffer changed = make_mosi_status_frame_33(2U, false, true);
  auto changed_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, true);
  for (uint32_t i = 0U; i < 3U; i++) {
    observe_trace_status(recorder, changed, changed_status, true, i + 3U,
                         30U + i, 30000U + (i * 1000U),
                         MHI_COMMAND_THREE_D_AUTO);
  }
  recorder.observe_confirmation_timeout(MHI_COMMAND_THREE_D_AUTO, 0U, 40000U);
  recorder.poll(291U, 291000U);

  EXPECT_TRUE(recorder.ready());
  EXPECT_EQ(static_cast<uint8_t>(recorder.summary().assessment),
            static_cast<uint8_t>(
                MhiProtocolTraceAssessment::CONFIRMATION_LOGIC_MISMATCH));
}

void protocol_trace_replays_rx_observed_before_tx_completion() {
  MhiProtocolTraceRecorder recorder{};
  MhiProtocolTraceConfig config{};
  config.enabled = true;
  recorder.configure(config);

  MhiFrameBuffer baseline = make_mosi_status_frame_33(2U, false, false);
  auto baseline_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, false);
  recorder.observe_mosi_status(baseline, baseline_status, false, 1U, 10U,
                               10000U, 10U, 10100U, 0U);

  MhiCommandState command{};
  command.three_d_auto_set = true;
  command.three_d_auto = true;
  const auto requested = MhiProtocolTraceRecorder::semantic_from_command(
      command, MHI_COMMAND_THREE_D_AUTO);
  EXPECT_TRUE(recorder.start_capture(MHI_COMMAND_THREE_D_AUTO, requested, 0U,
                                     20U, 20000U, "deferred_rx"));

  MhiFrameBuffer tx = baseline;
  tx.data[SB0] = kMhiMisoSignature0;
  tx.data[SB1] = kMhiMisoSignature1;
  tx.data[SB2] = kMhiMisoSignature2;
  tx.data[DB17] = 0x04U;
  finalize_trace_frame(tx);
  const auto envelope = make_trace_command_envelope(
      tx, 14U, MHI_COMMAND_THREE_D_AUTO);
  recorder.observe_tx_staged(envelope, 21000U, 0U);

  MhiFrameBuffer changed = make_mosi_status_frame_33(2U, false, true);
  auto changed_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, true);

  // The worker may see RX before the main loop drains the matching TX
  // completion. These observations must be replayed once bus sequence N is
  // known, preserving the N versus N+1 acknowledgement boundary.
  recorder.observe_mosi_status(changed, changed_status, true, 2U, 50U,
                               22000U, 22U, 22100U,
                               MHI_COMMAND_THREE_D_AUTO);
  recorder.observe_mosi_status(changed, changed_status, true, 3U, 51U,
                               23000U, 23U, 23100U,
                               MHI_COMMAND_THREE_D_AUTO);
  EXPECT_FALSE(recorder.summary().same_transaction_rx_seen);
  EXPECT_EQ(recorder.summary().tx_to_first_rx_us, 0U);

  recorder.observe_tx_completion(completion_for_trace(envelope, true, 50U, 22000U),
                                 24000U, MHI_COMMAND_THREE_D_AUTO);

  EXPECT_TRUE(recorder.summary().same_transaction_rx_seen);
  EXPECT_EQ(recorder.summary().tx_to_first_rx_us, 1000U);
  EXPECT_TRUE(recorder.summary().requested_state_seen);
}

void protocol_trace_classifies_staged_command_not_clocked() {
  MhiProtocolTraceRecorder recorder{};
  MhiProtocolTraceConfig config{};
  config.enabled = true;
  recorder.configure(config);

  MhiFrameBuffer baseline = make_mosi_status_frame();
  auto baseline_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, false);
  observe_trace_status(recorder, baseline, baseline_status, false, 1U, 10U,
                       10000U, 0U);

  MhiCommandState command{};
  command.power_set = true;
  command.power = true;
  const auto requested = MhiProtocolTraceRecorder::semantic_from_command(
      command, MHI_COMMAND_POWER);
  EXPECT_TRUE(recorder.start_capture(MHI_COMMAND_POWER, requested, 0U, 20U,
                                     20000U, "not_clocked"));

  MhiFrameBuffer tx = baseline;
  tx.data[SB0] = kMhiMisoSignature0;
  tx.data[SB1] = kMhiMisoSignature1;
  tx.data[SB2] = kMhiMisoSignature2;
  finalize_trace_frame(tx);
  const auto envelope = make_trace_command_envelope(tx, 12U, MHI_COMMAND_POWER);
  recorder.observe_tx_staged(envelope, 21000U, 0U);
  recorder.observe_staged_timeout(MHI_COMMAND_POWER, 2021000U, 0U);
  recorder.force_complete(2022000U);

  EXPECT_TRUE(recorder.ready());
  EXPECT_TRUE(recorder.summary().staged_timeout_seen);
  EXPECT_EQ(static_cast<uint8_t>(recorder.summary().assessment),
            static_cast<uint8_t>(MhiProtocolTraceAssessment::TX_NOT_CLOCKED));
}

void protocol_trace_distinguishes_missing_completion_from_no_bus_clock() {
  MhiProtocolTraceRecorder recorder{};
  MhiProtocolTraceConfig config{};
  config.enabled = true;
  recorder.configure(config);

  MhiFrameBuffer baseline = make_mosi_status_frame();
  auto baseline_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, false);
  observe_trace_status(recorder, baseline, baseline_status, false, 1U, 10U,
                       10000U, 0U);

  MhiCommandState command{};
  command.power_set = true;
  command.power = true;
  const auto requested = MhiProtocolTraceRecorder::semantic_from_command(
      command, MHI_COMMAND_POWER);
  EXPECT_TRUE(recorder.start_capture(MHI_COMMAND_POWER, requested, 0U, 20U,
                                     20000U, "completion_missing"));

  MhiFrameBuffer tx = baseline;
  tx.data[SB0] = kMhiMisoSignature0;
  tx.data[SB1] = kMhiMisoSignature1;
  tx.data[SB2] = kMhiMisoSignature2;
  finalize_trace_frame(tx);
  const auto envelope = make_trace_command_envelope(tx, 15U, MHI_COMMAND_POWER);
  recorder.observe_tx_staged(envelope, 21000U, 0U);

  // RX activity after staging proves the bus is still transacting. If the
  // matching TX completion never arrives, classify the missing lifecycle
  // event separately from a genuinely idle/no-clock bus.
  recorder.observe_mosi_status(baseline, baseline_status, false, 2U, 40U,
                               22000U, 22U, 22100U, MHI_COMMAND_POWER);
  recorder.force_complete(23000U);

  EXPECT_TRUE(recorder.ready());
  EXPECT_EQ(recorder.summary().rx_while_waiting_for_completion, 1U);
  EXPECT_EQ(static_cast<uint8_t>(recorder.summary().assessment),
            static_cast<uint8_t>(
                MhiProtocolTraceAssessment::TX_COMPLETION_MISSING));
}

void protocol_trace_classifies_on_wire_frame_mismatch() {
  MhiProtocolTraceRecorder recorder{};
  MhiProtocolTraceConfig config{};
  config.enabled = true;
  recorder.configure(config);

  MhiFrameBuffer baseline = make_mosi_status_frame();
  auto baseline_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, false);
  observe_trace_status(recorder, baseline, baseline_status, false, 1U, 10U,
                       10000U, 0U);

  MhiCommandState command{};
  command.power_set = true;
  command.power = true;
  const auto requested = MhiProtocolTraceRecorder::semantic_from_command(
      command, MHI_COMMAND_POWER);
  EXPECT_TRUE(recorder.start_capture(MHI_COMMAND_POWER, requested, 0U, 20U,
                                     20000U, "wire_mismatch"));

  MhiFrameBuffer staged = baseline;
  staged.data[SB0] = kMhiMisoSignature0;
  staged.data[SB1] = kMhiMisoSignature1;
  staged.data[SB2] = kMhiMisoSignature2;
  staged.data[DB0] |= 0x01U;
  finalize_trace_frame(staged);
  const auto envelope = make_trace_command_envelope(
      staged, 13U, MHI_COMMAND_POWER);
  recorder.observe_tx_staged(envelope, 21000U, 0U);

  MhiFrameBuffer actual = staged;
  actual.data[DB0] ^= 0x02U;
  finalize_trace_frame(actual);
  auto completion = completion_for_trace(envelope, true, 2U, 22000U);
  completion.set_transmitted_frame(actual.data, actual.len);
  recorder.observe_tx_completion(completion, 22100U, MHI_COMMAND_POWER);
  recorder.force_complete(23000U);

  EXPECT_TRUE(recorder.ready());
  EXPECT_TRUE(recorder.summary().on_wire_frame_mismatch);
  EXPECT_EQ(static_cast<uint8_t>(recorder.summary().assessment),
            static_cast<uint8_t>(
                MhiProtocolTraceAssessment::ON_WIRE_FRAME_MISMATCH));
}

void protocol_trace_excludes_same_transaction_mosi_from_ack_timing() {
  MhiProtocolTraceRecorder recorder{};
  MhiProtocolTraceConfig config{};
  config.enabled = true;
  recorder.configure(config);

  MhiFrameBuffer baseline = make_mosi_status_frame_33(2U, false, false);
  auto baseline_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, false);
  recorder.observe_mosi_status(baseline, baseline_status, false, 1U, 10U,
                               10000U, 10U, 10100U, 0U);

  MhiCommandState command{};
  command.three_d_auto_set = true;
  command.three_d_auto = true;
  const auto requested = MhiProtocolTraceRecorder::semantic_from_command(
      command, MHI_COMMAND_THREE_D_AUTO);
  EXPECT_TRUE(recorder.start_capture(MHI_COMMAND_THREE_D_AUTO, requested, 0U,
                                     20U, 20000U, "same_transaction"));

  MhiFrameBuffer tx = baseline;
  tx.data[SB0] = kMhiMisoSignature0;
  tx.data[SB1] = kMhiMisoSignature1;
  tx.data[SB2] = kMhiMisoSignature2;
  tx.data[DB17] = 0x04U;
  finalize_trace_frame(tx);
  const auto envelope = make_trace_command_envelope(
      tx, 11U, MHI_COMMAND_THREE_D_AUTO);
  recorder.observe_tx_staged(envelope, 21000U, 0U);
  recorder.observe_tx_completion(completion_for_trace(envelope, true, 50U, 22000U),
                                 22100U, MHI_COMMAND_THREE_D_AUTO);

  MhiFrameBuffer changed = make_mosi_status_frame_33(2U, false, true);
  auto changed_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, true);

  // RX captured during the same full-duplex transaction is contemporaneous
  // input, not an acknowledgement to the MISO command shifted alongside it.
  recorder.observe_mosi_status(changed, changed_status, true, 2U, 50U,
                               22000U, 22U, 22150U,
                               MHI_COMMAND_THREE_D_AUTO);
  EXPECT_TRUE(recorder.summary().same_transaction_rx_seen);
  EXPECT_EQ(recorder.summary().tx_to_first_rx_us, 0U);
  EXPECT_EQ(recorder.summary().tx_to_first_raw_change_us, 0U);
  EXPECT_FALSE(recorder.summary().requested_state_seen);

  // The first later bus transaction is acknowledgement-eligible.
  recorder.observe_mosi_status(changed, changed_status, true, 3U, 51U,
                               23000U, 23U, 23100U,
                               MHI_COMMAND_THREE_D_AUTO);
  EXPECT_EQ(recorder.summary().tx_to_first_rx_us, 1000U);
  EXPECT_EQ(recorder.summary().tx_to_first_raw_change_us, 1000U);
  EXPECT_TRUE(recorder.summary().requested_state_seen);
}

void protocol_trace_detects_late_confirmation() {
  MhiProtocolTraceRecorder recorder{};
  MhiProtocolTraceConfig config{};
  config.enabled = true;
  config.post_timeout_grace_ms = 1000U;
  recorder.configure(config);

  MhiFrameBuffer baseline = make_mosi_status_frame();
  auto baseline_status = make_trace_status(false, 2U, 1U, 20.0f, 2U, false);
  observe_trace_status(recorder, baseline, baseline_status, false, 1U, 10U,
                       10000U, 0U);

  MhiCommandState command{};
  command.power_set = true;
  command.power = true;
  const auto requested = MhiProtocolTraceRecorder::semantic_from_command(
      command, MHI_COMMAND_POWER);
  EXPECT_TRUE(recorder.start_capture(MHI_COMMAND_POWER, requested, 0U, 20U,
                                     20000U, "late_ack"));

  MhiFrameBuffer tx = baseline;
  tx.data[SB0] = kMhiMisoSignature0;
  tx.data[SB1] = kMhiMisoSignature1;
  tx.data[SB2] = kMhiMisoSignature2;
  finalize_trace_frame(tx);
  const auto envelope = make_trace_command_envelope(tx, 10U, MHI_COMMAND_POWER);
  recorder.observe_tx_staged(envelope, 21000U, 0U);
  recorder.observe_tx_completion(completion_for_trace(envelope, true, 2U, 22000U),
                                 22100U, MHI_COMMAND_POWER);

  MhiFrameBuffer changed = baseline;
  changed.data[DB0] |= 0x01U;
  finalize_trace_frame(changed);
  auto changed_status = baseline_status;
  changed_status.power = true;
  for (uint32_t i = 0U; i < 3U; i++) {
    observe_trace_status(recorder, changed, changed_status, true, i + 3U,
                         30U + i, 30000U + (i * 1000U), MHI_COMMAND_POWER);
  }
  recorder.observe_confirmation_timeout(MHI_COMMAND_POWER, 0U, 40000U);
  recorder.observe_confirmation(MHI_COMMAND_POWER, 0U, 41000U);

  EXPECT_TRUE(recorder.ready());
  EXPECT_TRUE(recorder.summary().late_confirmation);
  EXPECT_EQ(static_cast<uint8_t>(recorder.summary().assessment),
            static_cast<uint8_t>(MhiProtocolTraceAssessment::ACK_AFTER_TIMEOUT));
}

}  // namespace mhi_unit_tests
