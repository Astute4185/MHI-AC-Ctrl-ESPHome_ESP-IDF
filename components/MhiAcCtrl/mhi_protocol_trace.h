#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "mhi_command.h"
#include "mhi_frame.h"
#include "mhi_status_decoder.h"
#include "mhi_tx_contract.h"

namespace esphome {
namespace mhi_ac_ctrl {

constexpr std::size_t kMhiProtocolTraceMaxRecords = 64U;
constexpr std::size_t kMhiProtocolTraceMaxPreCommandFrames = 8U;
constexpr std::size_t kMhiProtocolTraceLabelBytes = 32U;

// The names deliberately include the physical bus direction. On this bus the
// indoor unit is the SPI-like master: MOSI is AC -> controller and MISO is
// controller -> AC.
enum class MhiProtocolTraceEvent : uint8_t {
  BASELINE_MOSI = 0,
  COMMAND_REQUESTED,
  MISO_FRAME_STAGED,
  MISO_FRAME_TRANSMITTED,
  MISO_TRANSMISSION_FAILED,
  STAGED_TX_TIMEOUT,
  MOSI_AWAITING_TX_COMPLETION,
  MOSI_DURING_TX_TRANSACTION,
  FIRST_MOSI_AFTER_TX,
  MOSI_STATUS_HEARTBEAT,
  MOSI_RAW_CHANGE,
  MOSI_SEMANTIC_CHANGE,
  MOSI_COMMAND_CANDIDATE,
  MOSI_STABLE_STATUS,
  CONFIRMATION_MATCHED,
  LATE_CONFIRMATION_MATCHED,
  CONFIRMATION_TIMEOUT,
  PHYSICAL_RESULT_MARKER,
  CAPTURE_COMPLETE,
};

enum class MhiProtocolTraceDirection : uint8_t {
  NONE = 0,
  MOSI_AC_TO_CONTROLLER,
  MISO_CONTROLLER_TO_AC,
};

enum class MhiProtocolTraceCompletionReason : uint8_t {
  NONE = 0,
  PASSIVE_STABLE_STATUS,
  CONFIRMED,
  TIMEOUT_GRACE,
  CAPTURE_WINDOW,
  RECORD_LIMIT,
  FORCED,
};

enum class MhiProtocolTraceAssessment : uint8_t {
  INCONCLUSIVE = 0,
  PASSIVE_CHANGE_CAPTURED,
  NO_CHANGE_OBSERVED,
  TX_NOT_STAGED,
  TX_NOT_CLOCKED,
  TX_COMPLETION_MISSING,
  TX_FAILED,
  ON_WIRE_FRAME_MISMATCH,
  DIRECTION_OR_SIGNATURE_ERROR,
  NO_RX_AFTER_TX,
  NO_RAW_FEEDBACK_CHANGE,
  RAW_CHANGE_NOT_DECODED,
  DECODED_STATE_MISMATCH,
  CONFIRMATION_LOGIC_MISMATCH,
  ACK_AFTER_TIMEOUT,
  CONFIRMED,
};

enum MhiProtocolTraceStateField : uint32_t {
  MHI_TRACE_STATE_POWER = (1UL << 0),
  MHI_TRACE_STATE_MODE = (1UL << 1),
  MHI_TRACE_STATE_FAN = (1UL << 2),
  MHI_TRACE_STATE_TARGET_TEMP = (1UL << 3),
  MHI_TRACE_STATE_VERTICAL_VANE = (1UL << 4),
  MHI_TRACE_STATE_HORIZONTAL_VANE = (1UL << 5),
  MHI_TRACE_STATE_THREE_D_AUTO = (1UL << 6),
  MHI_TRACE_STATE_ROOM_TEMP_OVERRIDE = (1UL << 7),
  MHI_TRACE_STATE_HORIZONTAL_SWING = (1UL << 8),
  MHI_TRACE_STATE_VERTICAL_SWING = (1UL << 9),
  MHI_TRACE_STATE_EXTENDED_RAW = (1UL << 10),
};

struct MhiProtocolTraceConfig {
  bool enabled{false};
  uint32_t capture_window_ms{30000U};
  uint32_t post_timeout_grace_ms{5000U};
  uint32_t unchanged_heartbeat_ms{1000U};
  uint8_t pre_command_frames{4U};
  uint8_t post_command_frames{64U};
  uint8_t max_records{64U};
};

struct MhiProtocolTraceSemanticState {
  uint32_t valid_fields{0U};
  bool power{false};
  uint8_t mode{0U};
  uint8_t fan{0U};
  float target_temp_c{0.0f};
  uint8_t vertical_vane{0U};
  bool vertical_swing{false};
  uint8_t horizontal_vane{0U};
  bool horizontal_swing{false};
  bool three_d_auto{false};
  uint8_t extended_louver_db16{0U};
  uint8_t extended_louver_db17{0U};
  uint8_t room_temp_override_raw{0xFFU};
};

struct MhiProtocolTraceRecord {
  uint32_t observed_at_us{0U};
  uint32_t event_time_us{0U};
  uint32_t elapsed_us{0U};
  uint32_t processing_delay_us{0U};
  uint32_t since_tx_us{0U};
  uint32_t catalog_sequence{0U};
  uint32_t bus_sequence{0U};
  uint32_t command_generation{0U};
  uint8_t tx_attempt{0U};
  MhiProtocolTraceEvent event{MhiProtocolTraceEvent::BASELINE_MOSI};
  MhiProtocolTraceDirection direction{MhiProtocolTraceDirection::NONE};
  uint32_t command_mask{0U};
  uint32_t confirmation_pending_mask{0U};
  bool success{false};
  bool signature_valid{false};
  bool checksum_valid{false};
  bool decoder_valid{false};
  bool command_candidate{false};
  bool requested_state_match{false};
  uint16_t expected_len{0U};
  uint16_t actual_len{0U};
  uint64_t changed_byte_mask{0U};
  MhiFrameBuffer frame{};
  MhiProtocolTraceSemanticState semantic{};
  char label[kMhiProtocolTraceLabelBytes]{};
};

struct MhiProtocolTraceStats {
  uint32_t captures_started{0U};
  uint32_t captures_completed{0U};
  uint32_t busy_rejections{0U};
  uint32_t records_dropped{0U};
};

struct MhiProtocolTraceSummary {
  MhiProtocolTraceAssessment assessment{MhiProtocolTraceAssessment::INCONCLUSIVE};
  uint32_t tx_attempts{0U};
  uint32_t tx_successes{0U};
  uint32_t tx_failures{0U};
  uint32_t rx_frames_after_tx{0U};
  uint32_t rx_while_waiting_for_completion{0U};
  uint32_t deferred_mosi_overwrites{0U};
  bool same_transaction_rx_seen{false};
  uint32_t raw_changes{0U};
  uint32_t semantic_changes{0U};
  uint32_t command_candidates{0U};
  uint32_t direction_errors{0U};
  uint32_t checksum_errors{0U};
  uint32_t decoder_failures{0U};
  uint32_t confirmations{0U};
  uint32_t confirmation_timeouts{0U};
  bool staged_timeout_seen{false};
  bool on_wire_frame_mismatch{false};
  bool stable_status_seen{false};
  bool requested_state_seen{false};
  bool late_confirmation{false};
  uint32_t request_to_stage_us{0U};
  uint32_t stage_to_wire_us{0U};
  uint32_t tx_to_first_rx_us{0U};
  uint32_t tx_to_first_raw_change_us{0U};
  uint32_t tx_to_first_semantic_change_us{0U};
  uint32_t tx_to_confirmation_us{0U};
  uint32_t tx_to_timeout_us{0U};
};

class MhiProtocolTraceRecorder {
 public:
  void configure(const MhiProtocolTraceConfig& config);
  void reset();

  bool enabled() const { return config_.enabled; }
  bool active() const { return active_; }
  bool ready() const { return ready_; }
  bool exporting() const { return exporting_; }

  bool start_capture(uint32_t command_mask, const MhiProtocolTraceSemanticState& requested,
                     uint32_t confirmation_pending_mask, uint32_t now_ms, uint32_t now_us,
                     const char* label);
  bool mark_physical_result(const char* label, uint32_t now_us,
                            uint32_t confirmation_pending_mask);
  void force_complete(uint32_t now_us);
  void poll(uint32_t now_ms, uint32_t now_us);

  void observe_tx_staged(const MhiTxEnvelope& envelope, uint32_t now_us,
                         uint32_t confirmation_pending_mask);
  void observe_tx_completion(const MhiTxCompletion& completion, uint32_t observed_at_us,
                             uint32_t confirmation_pending_mask);
  void observe_staged_timeout(uint32_t timeout_mask, uint32_t now_us,
                              uint32_t confirmation_pending_mask);
  void observe_mosi_status(const MhiFrameBuffer& frame, const MhiDecodedStatus& decoded,
                           bool command_candidate, uint32_t catalog_sequence,
                           uint32_t bus_sequence, uint32_t frame_end_us, uint32_t now_ms,
                           uint32_t observed_at_us, uint32_t confirmation_pending_mask);
  void observe_confirmation(uint32_t confirmed_mask, uint32_t pending_mask, uint32_t now_us);
  void observe_confirmation_timeout(uint32_t timeout_mask, uint32_t pending_mask,
                                    uint32_t now_us);

  bool begin_export();
  void finish_export();

  std::size_t record_count() const { return record_count_; }
  const MhiProtocolTraceRecord& record(std::size_t index) const { return records_[index]; }
  uint32_t capture_id() const { return capture_id_; }
  uint32_t capture_command_mask() const { return capture_command_mask_; }
  uint32_t capture_generation() const { return capture_generation_; }
  MhiProtocolTraceCompletionReason completion_reason() const { return completion_reason_; }
  const char* capture_label() const { return capture_label_; }
  const MhiProtocolTraceStats& stats() const { return stats_; }
  const MhiProtocolTraceSummary& summary() const { return summary_; }

  static MhiProtocolTraceSemanticState semantic_from_command(const MhiCommandState& command,
                                                             uint32_t mask);
  static MhiProtocolTraceSemanticState semantic_from_status(const MhiDecodedStatus& status);
  static const char* event_name(MhiProtocolTraceEvent event);
  static const char* direction_name(MhiProtocolTraceDirection direction);
  static const char* completion_reason_name(MhiProtocolTraceCompletionReason reason);
  static const char* assessment_name(MhiProtocolTraceAssessment assessment);

 private:
  struct DeferredMosiObservation {
    MhiFrameBuffer frame{};
    MhiDecodedStatus decoded{};
    bool command_candidate{false};
    uint32_t catalog_sequence{0U};
    uint32_t bus_sequence{0U};
    uint32_t frame_end_us{0U};
    uint32_t now_ms{0U};
    uint32_t observed_at_us{0U};
    uint32_t confirmation_pending_mask{0U};
  };

  void defer_mosi_observation_(const MhiFrameBuffer& frame,
                               const MhiDecodedStatus& decoded,
                               bool command_candidate, uint32_t catalog_sequence,
                               uint32_t bus_sequence, uint32_t frame_end_us,
                               uint32_t now_ms, uint32_t observed_at_us,
                               uint32_t confirmation_pending_mask);
  void replay_deferred_mosi_();
  bool append_record_(MhiProtocolTraceEvent event, MhiProtocolTraceDirection direction,
                      uint32_t command_mask, uint32_t pending_mask, uint32_t generation,
                      uint32_t catalog_sequence, uint32_t bus_sequence, uint32_t event_time_us,
                      uint32_t observed_at_us, const MhiFrameBuffer* frame,
                      const MhiProtocolTraceSemanticState* semantic, bool success,
                      bool decoder_valid, bool command_candidate, bool requested_state_match,
                      uint16_t expected_len, uint16_t actual_len, const char* label);
  void complete_(MhiProtocolTraceCompletionReason reason, uint32_t now_us);
  void assess_();
  void store_pre_command_mosi_(const MhiFrameBuffer& frame, const MhiDecodedStatus& decoded,
                               uint32_t catalog_sequence, uint32_t bus_sequence,
                               uint32_t frame_end_us, uint32_t observed_at_us);
  void store_background_miso_(const MhiFrameBuffer& frame);
  void copy_pre_command_records_(uint32_t now_us);
  bool frame_matches_(const MhiFrameBuffer& lhs, const MhiFrameBuffer& rhs) const;
  bool semantic_matches_(const MhiProtocolTraceSemanticState& lhs,
                         const MhiProtocolTraceSemanticState& rhs,
                         uint32_t field_mask) const;
  bool semantic_changed_from_baseline_(const MhiProtocolTraceSemanticState& semantic) const;
  bool requested_state_matches_(const MhiProtocolTraceSemanticState& semantic) const;
  bool direction_signature_valid_(MhiProtocolTraceDirection direction,
                                  const MhiFrameBuffer& frame) const;
  bool checksum_valid_(const MhiFrameBuffer& frame) const;
  uint64_t changed_byte_mask_(MhiProtocolTraceDirection direction,
                              const MhiFrameBuffer& frame) const;
  static void copy_label_(char* destination, const char* source);
  static uint32_t elapsed_(uint32_t later, uint32_t earlier);

  MhiProtocolTraceConfig config_{};
  std::array<MhiProtocolTraceRecord, kMhiProtocolTraceMaxRecords> records_{};
  std::size_t record_count_{0U};

  std::array<MhiProtocolTraceRecord, kMhiProtocolTraceMaxPreCommandFrames> pre_mosi_{};
  std::array<DeferredMosiObservation, 4U> deferred_mosi_{};
  std::size_t deferred_mosi_count_{0U};
  std::size_t pre_mosi_head_{0U};
  std::size_t pre_mosi_size_{0U};
  MhiFrameBuffer baseline_mosi_{};
  MhiFrameBuffer baseline_miso_{};
  MhiFrameBuffer last_staged_miso_{};
  MhiFrameBuffer last_logged_mosi_{};
  MhiProtocolTraceSemanticState baseline_semantic_{};
  MhiProtocolTraceSemanticState requested_semantic_{};
  MhiProtocolTraceSemanticState last_semantic_{};
  MhiProtocolTraceSemanticState stable_semantic_{};

  bool active_{false};
  bool ready_{false};
  bool exporting_{false};
  bool tx_staged_seen_{false};
  bool tx_completion_seen_{false};
  bool awaiting_tx_completion_{false};
  bool tx_success_seen_{false};
  bool same_transaction_rx_seen_{false};
  bool first_rx_after_tx_seen_{false};
  bool raw_change_seen_{false};
  bool semantic_change_seen_{false};
  bool stable_status_seen_{false};
  bool requested_state_seen_{false};
  bool confirmation_seen_{false};
  bool timeout_seen_{false};
  bool command_candidate_seen_{false};
  uint8_t stable_repeat_count_{0U};
  uint8_t post_status_frames_{0U};
  uint8_t tx_attempt_{0U};
  uint32_t last_heartbeat_ms_{0U};
  uint32_t timeout_seen_ms_{0U};
  uint32_t tx_staged_us_{0U};
  uint32_t tx_end_us_{0U};
  uint32_t tx_bus_sequence_{0U};

  uint32_t capture_id_{0U};
  uint32_t capture_started_ms_{0U};
  uint32_t capture_started_us_{0U};
  uint32_t capture_command_mask_{0U};
  uint32_t capture_generation_{0U};
  MhiProtocolTraceCompletionReason completion_reason_{MhiProtocolTraceCompletionReason::NONE};
  char capture_label_[kMhiProtocolTraceLabelBytes]{};
  MhiProtocolTraceStats stats_{};
  MhiProtocolTraceSummary summary_{};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
