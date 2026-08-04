#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esphome/core/hal.h"
#include "mhi_duplex_transport_adapter.h"
#include "mhi_split_transport.h"
#include "mhi_transport_diagnostics_publisher.h"
#include "mhi_transport_manager.h"
#include "mhi_test_common.h"

namespace mhi_unit_tests {
namespace {

class FakeRxDriver final : public IMhiRxDriver {
 public:
  bool setup(const MhiTransportPins& pins) override {
    pins_ = pins;
    ready_ = setup_result;
    return ready_;
  }
  void loop() override {
    loop_count++;
  }
  void shutdown() override {
    ready_ = false;
    shutdown_count++;
  }
  std::size_t read(uint8_t* dst, std::size_t max_len) override {
    if (!ready_ || dst == nullptr || max_len < read_len) {
      return 0U;
    }
    std::memcpy(dst, read_data.data(), read_len);
    return read_len;
  }
  MhiBusMarker bus_marker() const override {
    return marker;
  }
  void set_byte_critical_sections(bool enabled) override {
    critical_sections = enabled;
  }
  bool byte_critical_sections() const override {
    return critical_sections;
  }
  const char* name() const override {
    return "fake_rx";
  }
  bool ready() const override {
    return ready_;
  }

  bool setup_result{true};
  bool critical_sections{true};
  bool ready_{false};
  int loop_count{0};
  int shutdown_count{0};
  MhiTransportPins pins_{};
  MhiBusMarker marker{};
  std::array<uint8_t, kMhiMaxFrameBytes> read_data{};
  std::size_t read_len{0U};
};

class FakeTxDriver final : public IMhiTxDriver {
 public:
  explicit FakeTxDriver(const char* driver_name = "fake_tx") : driver_name_(driver_name) {}

  bool setup(const MhiTransportPins& pins) override {
    pins_ = pins;
    ready_ = setup_result;
    return ready_;
  }
  void loop() override {
    loop_count++;
  }
  bool send(const uint8_t* data, std::size_t len) override {
    send_count++;
    last_len = len;
    if (data != nullptr && len <= last_frame.size()) {
      std::memcpy(last_frame.data(), data, len);
    }
    return send_result;
  }
  void set_byte_critical_sections(bool enabled) override {
    critical_sections = enabled;
  }
  bool byte_critical_sections() const override {
    return critical_sections;
  }
  const char* name() const override {
    return driver_name_;
  }
  bool ready() const override {
    return ready_;
  }

  const char* driver_name_{nullptr};
  bool setup_result{true};
  bool send_result{true};
  bool critical_sections{true};
  bool ready_{false};
  int loop_count{0};
  int send_count{0};
  std::size_t last_len{0U};
  MhiTransportPins pins_{};
  std::array<uint8_t, kMhiMaxFrameBytes> last_frame{};
};

class FakeDuplexTransport final : public IMhiDuplexTransport {
 public:
  bool setup(const MhiTransportPins& pins) override {
    pins_ = pins;
    ready_ = setup_result;
    return ready_;
  }
  void loop() override {
    loop_count++;
  }
  void shutdown() override {
    ready_ = false;
    shutdown_count++;
  }
  std::size_t read(uint8_t* dst, std::size_t max_len) override {
    if (dst == nullptr || max_len < read_len) {
      return 0U;
    }
    std::memcpy(dst, read_data.data(), read_len);
    return read_len;
  }
  bool send(const MhiTxEnvelope& envelope) override {
    send_count++;
    last_envelope = envelope;
    return send_result;
  }
  bool take_tx_completion(MhiTxCompletion& completion) override {
    if (!completion_available) {
      return false;
    }
    completion = next_completion;
    completion_available = false;
    return true;
  }
  const char* name() const override {
    return "fake_duplex";
  }
  bool ready() const override {
    return ready_;
  }
  uint32_t completed_tx_frames() const override {
    return completed;
  }
  uint32_t tx_failures() const override {
    return failures;
  }

  bool setup_result{true};
  bool send_result{true};
  bool ready_{false};
  bool completion_available{false};
  int loop_count{0};
  int shutdown_count{0};
  int send_count{0};
  uint32_t completed{0U};
  uint32_t failures{0U};
  std::size_t read_len{0U};
  MhiTransportPins pins_{};
  MhiTxEnvelope last_envelope{};
  MhiTxCompletion next_completion{};
  std::array<uint8_t, kMhiMaxFrameBytes> read_data{};
};

class FakeUnifiedTransport final : public IMhiTransport {
 public:
  explicit FakeUnifiedTransport(const char* transport_name) : transport_name_(transport_name) {}

  MhiTransportResult setup() override {
    setup_count++;
    ready_ = setup_result.ok;
    health_snapshot = {};
    health_snapshot.setup_at_ms = esphome::millis();
    health_snapshot.state = ready_ ? MhiTransportState::WAITING_FOR_TRAFFIC : MhiTransportState::FAILED;
    return setup_result;
  }
  void loop() override {
    loop_count++;
  }
  void shutdown() override {
    ready_ = false;
    shutdown_count++;
  }
  std::size_t read(uint8_t* dst, std::size_t max_len) override {
    if (!ready_ || dst == nullptr || max_len < read_len) {
      return 0U;
    }
    if (read_len > 0U) {
      std::memcpy(dst, read_data.data(), read_len);
      health_snapshot.traffic_seen = true;
      health_snapshot.last_rx_activity_ms = esphome::millis();
      health_snapshot.rx_bytes += static_cast<uint32_t>(read_len);
    }
    return read_len;
  }
  bool queue_tx(const MhiTxEnvelope& envelope) override {
    return ready_ && envelope.valid();
  }
  bool take_tx_completion(MhiTxCompletion& completion) override {
    (void) completion;
    return false;
  }
  bool has_pending_tx() const override {
    return false;
  }
  bool flush_tx_on_bus_marker() override {
    return false;
  }
  void set_auto_tx_flush(bool enabled) override {
    auto_tx_flush_ = enabled;
  }
  bool auto_tx_flush() const override {
    return auto_tx_flush_;
  }
  void set_rx_byte_critical_sections(bool enabled) override {
    critical_sections_ = enabled;
  }
  bool rx_byte_critical_sections() const override {
    return critical_sections_;
  }
  const char* name() const override {
    return transport_name_;
  }
  const char* rx_name() const override {
    return transport_name_;
  }
  const char* tx_name() const override {
    return transport_name_;
  }
  bool rx_ready() const override {
    return ready_;
  }
  bool tx_ready() const override {
    return ready_;
  }
  MhiTransportCapabilities capabilities() const override {
    MhiTransportCapabilities capabilities{};
    capabilities.supports_tx = true;
    return capabilities;
  }
  MhiTransportHealth health() const override {
    return health_snapshot;
  }
  MhiTransportErrorDetail last_error() const override {
    return runtime_error.present() ? runtime_error : setup_result.error;
  }

  const char* transport_name_{nullptr};
  MhiTransportResult setup_result{MhiTransportResult::success()};
  bool ready_{false};
  bool auto_tx_flush_{true};
  bool critical_sections_{true};
  int setup_count{0};
  int shutdown_count{0};
  int loop_count{0};
  std::size_t read_len{0U};
  std::array<uint8_t, kMhiMaxFrameBytes> read_data{};
  MhiTransportHealth health_snapshot{};
  MhiTransportErrorDetail runtime_error{};
};

class FakeTransportTransitionListener final : public IMhiTransportTransitionListener {
 public:
  void on_transport_switch_begin(const MhiTransportErrorDetail& reason) override {
    switch_begin_count++;
    switch_reason = reason;
    command_state_cleared = true;
  }

  void on_transport_recovery_ready() override {
    recovery_ready_count++;
    commands_enabled = true;
  }

  void on_transport_safe_mode(const MhiTransportErrorDetail& reason) override {
    safe_mode_count++;
    safe_mode_reason = reason;
    commands_enabled = false;
  }

  int switch_begin_count{0};
  int recovery_ready_count{0};
  int safe_mode_count{0};
  bool command_state_cleared{false};
  bool commands_enabled{false};
  MhiTransportErrorDetail switch_reason{};
  MhiTransportErrorDetail safe_mode_reason{};
};

MhiTxEnvelope make_command_envelope() {
  MhiTxEnvelope envelope{};
  envelope.len = kMhiFrame20Bytes;
  envelope.generation = 42U;
  envelope.kind = MhiTxKind::COMMAND;
  envelope.command_mask = 0x01U;
  envelope.frame[0] = 0xA9U;
  return envelope;
}

}  // namespace

void split_transport_delegates_rx_and_marker_armed_tx() {
  FakeRxDriver rx{};
  FakeTxDriver tx{};
  MhiSplitTransport transport{};
  transport.bind(&rx, &tx, true, true);

  const MhiTransportPins pins{8, 38, 39};
  transport.set_pins(pins.sck, pins.mosi, pins.miso);
  EXPECT_TRUE(transport.setup());
  EXPECT_TRUE(transport.rx_ready());
  EXPECT_TRUE(transport.tx_ready());
  EXPECT_TRUE(transport.capabilities().supports_classified_worker);
  EXPECT_TRUE(transport.capabilities().uses_bus_marker);

  const MhiTxEnvelope envelope = make_command_envelope();
  rx.marker = {true, 1U, 1000U, kMhiFrame20Bytes};
  EXPECT_TRUE(transport.queue_tx(envelope));
  EXPECT_TRUE(transport.has_pending_tx());

  rx.marker = {true, 2U, 2000U, kMhiFrame20Bytes};
  esphome::test_micros_value = 2000U;
  esphome::test_millis_value = 2U;
  EXPECT_TRUE(transport.flush_tx_on_bus_marker());
  EXPECT_EQ(tx.send_count, 1);
  EXPECT_EQ(tx.last_len, kMhiFrame20Bytes);
  EXPECT_EQ(transport.completed_tx_frames(), 1U);
  EXPECT_FALSE(transport.has_pending_tx());

  MhiTxCompletion completion{};
  EXPECT_TRUE(transport.take_tx_completion(completion));
  EXPECT_EQ(completion.generation, 42U);
  EXPECT_TRUE(completion.success);
}

void split_transport_preserves_null_tx_completion_contract() {
  FakeRxDriver rx{};
  FakeTxDriver null_tx{"none"};
  MhiSplitTransport transport{};
  transport.bind(&rx, &null_tx, true, false);
  transport.set_pins(8, 38, 39);
  EXPECT_TRUE(transport.setup());

  const MhiTxEnvelope envelope = make_command_envelope();
  EXPECT_TRUE(transport.queue_tx(envelope));
  EXPECT_EQ(null_tx.send_count, 0);

  MhiTxCompletion completion{};
  EXPECT_TRUE(transport.take_tx_completion(completion));
  EXPECT_EQ(completion.generation, 42U);
  EXPECT_FALSE(completion.success);
}

void duplex_transport_adapter_preserves_backend_contract() {
  FakeDuplexTransport backend{};
  MhiDuplexTransportAdapter transport{};
  transport.bind(&backend, true);
  transport.set_pins(8, 38, 39);

  EXPECT_TRUE(transport.setup());
  EXPECT_TRUE(transport.capabilities().integrated_duplex);
  EXPECT_TRUE(transport.capabilities().supports_classified_worker);
  EXPECT_FALSE(transport.capabilities().uses_bus_marker);
  EXPECT_TRUE(std::strcmp(transport.rx_name(), "fake_duplex") == 0);
  EXPECT_TRUE(std::strcmp(transport.tx_name(), "fake_duplex") == 0);

  const MhiTxEnvelope envelope = make_command_envelope();
  EXPECT_TRUE(transport.queue_tx(envelope));
  EXPECT_EQ(backend.send_count, 1);

  backend.completed = 3U;
  backend.failures = 2U;
  EXPECT_EQ(transport.completed_tx_frames(), 3U);
  EXPECT_EQ(transport.tx_failures(), 2U);

  backend.send_result = false;
  EXPECT_FALSE(transport.queue_tx(envelope));
  EXPECT_EQ(transport.tx_failures(), 3U);
}


void transport_result_preserves_error_context() {
  const MhiTransportResult result = MhiTransportResult::failure(
      MhiTransportError::RMT_SETUP_FAILED, "rmt_new_rx_channel", 0x105, "channel allocation failed");

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(static_cast<uint8_t>(result.error.code), static_cast<uint8_t>(MhiTransportError::RMT_SETUP_FAILED));
  EXPECT_EQ(result.error.native_code, 0x105);
  EXPECT_TRUE(std::strcmp(result.error.operation, "rmt_new_rx_channel") == 0);
  EXPECT_TRUE(std::strcmp(mhi_transport_error_name(result.error.code), "rmt_setup_failed") == 0);
}

void split_transport_reports_setup_failure_and_rx_health() {
  FakeRxDriver rx{};
  FakeTxDriver tx{};
  MhiSplitTransport transport{};
  transport.bind(&rx, &tx, true, true);

  transport.set_pins(8, 38, 39);
  rx.setup_result = false;
  const MhiTransportResult failed = transport.setup();
  EXPECT_FALSE(failed.ok);
  EXPECT_EQ(static_cast<uint8_t>(failed.error.code), static_cast<uint8_t>(MhiTransportError::RX_SETUP_FAILED));
  EXPECT_EQ(static_cast<uint8_t>(transport.health().state), static_cast<uint8_t>(MhiTransportState::FAILED));
  EXPECT_TRUE(transport.health().fault_latched);

  rx.setup_result = true;
  EXPECT_TRUE(transport.setup());
  EXPECT_EQ(static_cast<uint8_t>(transport.health().state),
            static_cast<uint8_t>(MhiTransportState::WAITING_FOR_TRAFFIC));

  rx.read_len = 3U;
  rx.read_data[0] = 0x6CU;
  rx.read_data[1] = 0x80U;
  rx.read_data[2] = 0x04U;
  std::array<uint8_t, 3U> dst{};
  esphome::test_millis_value = 75U;
  EXPECT_EQ(transport.read(dst.data(), dst.size()), 3U);

  const MhiTransportHealth health = transport.health();
  EXPECT_EQ(static_cast<uint8_t>(health.state), static_cast<uint8_t>(MhiTransportState::HEALTHY));
  EXPECT_TRUE(health.traffic_seen);
  EXPECT_EQ(health.last_rx_activity_ms, 75U);
  EXPECT_EQ(health.rx_bytes, 3U);
}

void duplex_transport_adapter_reports_missing_backend() {
  MhiDuplexTransportAdapter transport{};
  transport.set_pins(8, 38, 39);
  const MhiTransportResult result = transport.setup();

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(static_cast<uint8_t>(result.error.code), static_cast<uint8_t>(MhiTransportError::DRIVER_NOT_BOUND));
  EXPECT_EQ(static_cast<uint8_t>(transport.health().state), static_cast<uint8_t>(MhiTransportState::FAILED));
  EXPECT_TRUE(transport.last_error().present());
}


void transport_manager_uses_injected_primary_transport() {
  FakeUnifiedTransport primary{"primary"};
  MhiTransportManager manager{};
  manager.set_primary(&primary);

  EXPECT_TRUE(manager.setup());
  EXPECT_EQ(primary.setup_count, 1);
  EXPECT_TRUE(std::strcmp(manager.rx_name(), "primary") == 0);
  EXPECT_FALSE(manager.recovery_active());

  manager.loop();
  EXPECT_EQ(primary.loop_count, 1);
  manager.shutdown();
  EXPECT_EQ(primary.shutdown_count, 1);
}

void transport_manager_activates_injected_recovery_after_setup_failure() {
  FakeUnifiedTransport primary{"primary"};
  FakeUnifiedTransport recovery{"fast_gpio_recovery"};
  primary.setup_result = MhiTransportResult::failure(MhiTransportError::RX_SETUP_FAILED, "primary_setup");

  MhiTransportManager manager{};
  manager.set_primary(&primary);
  manager.set_recovery(&recovery);

  EXPECT_TRUE(manager.setup());
  EXPECT_EQ(primary.setup_count, 1);
  EXPECT_EQ(primary.shutdown_count, 1);
  EXPECT_EQ(recovery.setup_count, 1);
  EXPECT_TRUE(manager.recovery_active());
  EXPECT_TRUE(std::strcmp(manager.rx_name(), "fast_gpio_recovery") == 0);
}

void transport_manager_recovery_transition_is_ordered_and_latched() {
  FakeUnifiedTransport primary{"primary"};
  FakeUnifiedTransport recovery{"fast_gpio_recovery"};
  FakeTransportTransitionListener listener{};
  primary.setup_result = MhiTransportResult::failure(MhiTransportError::RMT_SETUP_FAILED, "primary_setup", 17);

  MhiTransportManager manager{};
  manager.set_transition_listener(&listener);
  manager.set_primary(&primary);
  manager.set_recovery(&recovery);

  MhiTransportHealthPolicy policy{};
  policy.health_check_interval_ms = 0U;
  policy.healthy_frame_count = 2U;
  manager.set_health_policy(policy);

  esphome::test_millis_value = 0U;
  EXPECT_TRUE(manager.setup());
  EXPECT_EQ(listener.switch_begin_count, 1);
  EXPECT_TRUE(listener.command_state_cleared);
  EXPECT_EQ(listener.recovery_ready_count, 0);
  EXPECT_EQ(listener.safe_mode_count, 0);
  EXPECT_FALSE(listener.commands_enabled);
  EXPECT_TRUE(manager.recovery_attempted());
  EXPECT_TRUE(manager.recovery_active());
  EXPECT_FALSE(manager.safe_mode());
  EXPECT_EQ(static_cast<uint8_t>(manager.state()),
            static_cast<uint8_t>(MhiTransportState::WAITING_FOR_TRAFFIC));
  EXPECT_EQ(primary.shutdown_count, 1);
  EXPECT_EQ(recovery.setup_count, 1);
  EXPECT_EQ(static_cast<uint8_t>(manager.primary_failure().code),
            static_cast<uint8_t>(MhiTransportError::RMT_SETUP_FAILED));
  EXPECT_FALSE(manager.recovery_failure().present());

  recovery.health_snapshot.traffic_seen = true;
  recovery.health_snapshot.last_rx_activity_ms = 10U;
  MhiProtocolHealth protocol_health{};
  protocol_health.valid_frames = 2U;
  protocol_health.last_valid_frame_ms = 10U;
  manager.observe_protocol_health(protocol_health);
  esphome::test_millis_value = 10U;
  manager.loop();

  EXPECT_EQ(listener.recovery_ready_count, 1);
  EXPECT_TRUE(listener.commands_enabled);
  EXPECT_EQ(static_cast<uint8_t>(manager.state()), static_cast<uint8_t>(MhiTransportState::RECOVERY_ACTIVE));
}

void transport_manager_enters_safe_mode_when_recovery_fails() {
  FakeUnifiedTransport primary{"primary"};
  FakeUnifiedTransport recovery{"fast_gpio_recovery"};
  FakeTransportTransitionListener listener{};
  primary.setup_result = MhiTransportResult::failure(MhiTransportError::RX_SETUP_FAILED, "primary_setup");
  recovery.setup_result = MhiTransportResult::failure(MhiTransportError::GPIO_SETUP_FAILED, "recovery_setup", 23);

  MhiTransportManager manager{};
  manager.set_transition_listener(&listener);
  manager.set_primary(&primary);
  manager.set_recovery(&recovery);

  EXPECT_FALSE(manager.setup());
  EXPECT_EQ(listener.switch_begin_count, 1);
  EXPECT_EQ(listener.recovery_ready_count, 0);
  EXPECT_EQ(listener.safe_mode_count, 1);
  EXPECT_FALSE(listener.commands_enabled);
  EXPECT_TRUE(manager.recovery_attempted());
  EXPECT_FALSE(manager.recovery_active());
  EXPECT_TRUE(manager.safe_mode());
  EXPECT_EQ(static_cast<uint8_t>(manager.state()), static_cast<uint8_t>(MhiTransportState::SAFE_MODE));
  EXPECT_TRUE(std::strcmp(manager.rx_name(), "none") == 0);
  EXPECT_TRUE(std::strcmp(manager.tx_name(), "none") == 0);
  EXPECT_EQ(primary.shutdown_count, 1);
  EXPECT_EQ(recovery.shutdown_count, 1);
  EXPECT_EQ(static_cast<uint8_t>(manager.recovery_failure().code),
            static_cast<uint8_t>(MhiTransportError::GPIO_SETUP_FAILED));

  const MhiTxEnvelope envelope = make_command_envelope();
  EXPECT_FALSE(manager.queue_tx(envelope));
  std::array<uint8_t, 4U> buffer{};
  EXPECT_EQ(manager.read_rx(buffer.data(), buffer.size()), 0U);
}

void transport_manager_enters_safe_mode_without_recovery() {
  FakeUnifiedTransport primary{"primary"};
  FakeTransportTransitionListener listener{};
  primary.setup_result = MhiTransportResult::failure(MhiTransportError::RX_SETUP_FAILED, "primary_setup");

  MhiTransportManager manager{};
  manager.set_transition_listener(&listener);
  manager.set_primary(&primary);

  EXPECT_FALSE(manager.setup());
  EXPECT_EQ(listener.switch_begin_count, 1);
  EXPECT_EQ(listener.safe_mode_count, 1);
  EXPECT_TRUE(manager.recovery_attempted());
  EXPECT_TRUE(manager.safe_mode());
  EXPECT_EQ(primary.shutdown_count, 1);
}


void transport_manager_recovers_after_startup_no_traffic() {
  FakeUnifiedTransport primary{"primary"};
  FakeUnifiedTransport recovery{"fast_gpio_recovery"};
  FakeTransportTransitionListener listener{};
  MhiTransportManager manager{};

  MhiTransportHealthPolicy policy{};
  policy.startup_grace_ms = 0U;
  policy.no_traffic_timeout_ms = 100U;
  policy.invalid_traffic_timeout_ms = 100U;
  policy.stalled_traffic_timeout_ms = 100U;
  policy.health_check_interval_ms = 0U;
  policy.healthy_frame_count = 2U;

  manager.set_health_policy(policy);
  manager.set_transition_listener(&listener);
  manager.set_primary(&primary);
  manager.set_recovery(&recovery);

  esphome::test_millis_value = 0U;
  EXPECT_TRUE(manager.setup());

  esphome::test_millis_value = 99U;
  manager.loop();
  EXPECT_FALSE(manager.recovery_active());

  esphome::test_millis_value = 100U;
  manager.loop();
  EXPECT_TRUE(manager.recovery_active());
  EXPECT_EQ(listener.switch_begin_count, 1);
  EXPECT_EQ(static_cast<uint8_t>(manager.primary_failure().code),
            static_cast<uint8_t>(MhiTransportError::NO_TRAFFIC));
  EXPECT_EQ(static_cast<uint8_t>(manager.state()),
            static_cast<uint8_t>(MhiTransportState::WAITING_FOR_TRAFFIC));
}

void transport_manager_recovers_after_invalid_startup_traffic() {
  FakeUnifiedTransport primary{"primary"};
  FakeUnifiedTransport recovery{"fast_gpio_recovery"};
  MhiTransportManager manager{};

  MhiTransportHealthPolicy policy{};
  policy.startup_grace_ms = 0U;
  policy.no_traffic_timeout_ms = 100U;
  policy.invalid_traffic_timeout_ms = 100U;
  policy.stalled_traffic_timeout_ms = 100U;
  policy.health_check_interval_ms = 0U;
  policy.healthy_frame_count = 2U;

  manager.set_health_policy(policy);
  manager.set_primary(&primary);
  manager.set_recovery(&recovery);

  esphome::test_millis_value = 0U;
  EXPECT_TRUE(manager.setup());
  primary.health_snapshot.traffic_seen = true;
  primary.health_snapshot.last_rx_activity_ms = 10U;

  esphome::test_millis_value = 10U;
  manager.loop();
  EXPECT_EQ(static_cast<uint8_t>(manager.state()), static_cast<uint8_t>(MhiTransportState::DEGRADED));

  esphome::test_millis_value = 109U;
  manager.loop();
  EXPECT_FALSE(manager.recovery_active());

  esphome::test_millis_value = 110U;
  manager.loop();
  EXPECT_TRUE(manager.recovery_active());
  EXPECT_EQ(static_cast<uint8_t>(manager.primary_failure().code),
            static_cast<uint8_t>(MhiTransportError::INVALID_TRAFFIC));
}

void transport_manager_marks_valid_protocol_traffic_healthy() {
  FakeUnifiedTransport primary{"primary"};
  MhiTransportManager manager{};

  MhiTransportHealthPolicy policy{};
  policy.startup_grace_ms = 0U;
  policy.no_traffic_timeout_ms = 100U;
  policy.invalid_traffic_timeout_ms = 100U;
  policy.stalled_traffic_timeout_ms = 100U;
  policy.health_check_interval_ms = 0U;
  policy.healthy_frame_count = 2U;

  manager.set_health_policy(policy);
  manager.set_primary(&primary);

  esphome::test_millis_value = 0U;
  EXPECT_TRUE(manager.setup());
  primary.health_snapshot.traffic_seen = true;
  primary.health_snapshot.last_rx_activity_ms = 10U;

  MhiProtocolHealth protocol_health{};
  protocol_health.valid_frames = 2U;
  protocol_health.last_valid_frame_ms = 10U;
  manager.observe_protocol_health(protocol_health);

  esphome::test_millis_value = 10U;
  manager.loop();
  EXPECT_EQ(static_cast<uint8_t>(manager.state()), static_cast<uint8_t>(MhiTransportState::HEALTHY));
  EXPECT_FALSE(manager.recovery_attempted());
}

void transport_manager_recovers_after_valid_traffic_stalls() {
  FakeUnifiedTransport primary{"primary"};
  FakeUnifiedTransport recovery{"fast_gpio_recovery"};
  MhiTransportManager manager{};

  MhiTransportHealthPolicy policy{};
  policy.startup_grace_ms = 0U;
  policy.no_traffic_timeout_ms = 100U;
  policy.invalid_traffic_timeout_ms = 100U;
  policy.stalled_traffic_timeout_ms = 100U;
  policy.health_check_interval_ms = 0U;
  policy.healthy_frame_count = 2U;

  manager.set_health_policy(policy);
  manager.set_primary(&primary);
  manager.set_recovery(&recovery);

  esphome::test_millis_value = 0U;
  EXPECT_TRUE(manager.setup());
  primary.health_snapshot.traffic_seen = true;
  primary.health_snapshot.last_rx_activity_ms = 10U;

  MhiProtocolHealth protocol_health{};
  protocol_health.valid_frames = 2U;
  protocol_health.last_valid_frame_ms = 10U;
  manager.observe_protocol_health(protocol_health);
  esphome::test_millis_value = 10U;
  manager.loop();
  EXPECT_EQ(static_cast<uint8_t>(manager.state()), static_cast<uint8_t>(MhiTransportState::HEALTHY));

  esphome::test_millis_value = 109U;
  manager.loop();
  EXPECT_FALSE(manager.recovery_active());

  esphome::test_millis_value = 110U;
  manager.loop();
  EXPECT_TRUE(manager.recovery_active());
  EXPECT_EQ(static_cast<uint8_t>(manager.primary_failure().code),
            static_cast<uint8_t>(MhiTransportError::RX_STALLED));
}


void transport_manager_recovers_after_latched_driver_fault() {
  FakeUnifiedTransport primary{"primary"};
  FakeUnifiedTransport recovery{"fast_gpio_recovery"};
  MhiTransportManager manager{};

  MhiTransportHealthPolicy policy{};
  policy.health_check_interval_ms = 0U;
  manager.set_health_policy(policy);
  manager.set_primary(&primary);
  manager.set_recovery(&recovery);

  esphome::test_millis_value = 0U;
  EXPECT_TRUE(manager.setup());

  primary.health_snapshot.fault_latched = true;
  primary.health_snapshot.state = MhiTransportState::FAILED;
  primary.runtime_error =
      MhiTransportResult::failure(MhiTransportError::RMT_SETUP_FAILED, "runtime_rmt_fault", 77).error;

  esphome::test_millis_value = 1U;
  manager.loop();
  EXPECT_TRUE(manager.recovery_active());
  EXPECT_EQ(static_cast<uint8_t>(manager.primary_failure().code),
            static_cast<uint8_t>(MhiTransportError::RMT_SETUP_FAILED));
  EXPECT_EQ(manager.primary_failure().native_code, 77);
}

void transport_manager_enters_safe_mode_when_recovery_has_no_traffic() {
  FakeUnifiedTransport primary{"primary"};
  FakeUnifiedTransport recovery{"fast_gpio_recovery"};
  FakeTransportTransitionListener listener{};
  primary.setup_result = MhiTransportResult::failure(MhiTransportError::RX_SETUP_FAILED, "primary_setup");

  MhiTransportHealthPolicy policy{};
  policy.startup_grace_ms = 0U;
  policy.no_traffic_timeout_ms = 100U;
  policy.invalid_traffic_timeout_ms = 100U;
  policy.stalled_traffic_timeout_ms = 100U;
  policy.health_check_interval_ms = 0U;
  policy.healthy_frame_count = 2U;

  MhiTransportManager manager{};
  manager.set_health_policy(policy);
  manager.set_transition_listener(&listener);
  manager.set_primary(&primary);
  manager.set_recovery(&recovery);

  esphome::test_millis_value = 0U;
  EXPECT_TRUE(manager.setup());
  EXPECT_TRUE(manager.recovery_active());
  EXPECT_FALSE(manager.safe_mode());

  esphome::test_millis_value = 100U;
  manager.loop();
  EXPECT_TRUE(manager.safe_mode());
  EXPECT_EQ(listener.safe_mode_count, 1);
  EXPECT_EQ(static_cast<uint8_t>(manager.recovery_failure().code),
            static_cast<uint8_t>(MhiTransportError::NO_TRAFFIC));
}


void transport_manager_diagnostics_snapshot_tracks_recovery_and_safe_mode() {
  FakeUnifiedTransport primary{"primary"};
  FakeUnifiedTransport recovery{"fast_gpio_recovery"};
  primary.setup_result = MhiTransportResult::failure(MhiTransportError::RX_SETUP_FAILED, "primary_setup");

  MhiTransportHealthPolicy policy{};
  policy.startup_grace_ms = 0U;
  policy.no_traffic_timeout_ms = 100U;
  policy.invalid_traffic_timeout_ms = 100U;
  policy.stalled_traffic_timeout_ms = 100U;
  policy.health_check_interval_ms = 0U;
  policy.healthy_frame_count = 2U;

  MhiTransportManager manager{};
  manager.set_health_policy(policy);
  manager.set_primary(&primary);
  manager.set_recovery(&recovery);

  esphome::test_millis_value = 0U;
  EXPECT_TRUE(manager.setup());

  MhiTransportDiagnosticsSnapshot snapshot = manager.diagnostics_snapshot(0U);
  EXPECT_EQ(snapshot.recovery_attempts, 1U);
  EXPECT_EQ(snapshot.recovery_activations, 0U);
  EXPECT_EQ(snapshot.recovery_failures, 0U);
  EXPECT_TRUE(snapshot.recovery_active);
  EXPECT_FALSE(snapshot.transport_healthy);
  EXPECT_TRUE(std::strcmp(snapshot.active_transport_name, "fast_gpio_recovery") == 0);

  recovery.health_snapshot.traffic_seen = true;
  recovery.health_snapshot.last_rx_activity_ms = 10U;
  MhiProtocolHealth protocol_health{};
  protocol_health.valid_frames = 2U;
  protocol_health.last_valid_frame_ms = 10U;
  manager.observe_protocol_health(protocol_health);
  esphome::test_millis_value = 10U;
  manager.loop();

  snapshot = manager.diagnostics_snapshot(10U);
  EXPECT_EQ(snapshot.recovery_activations, 1U);
  EXPECT_TRUE(snapshot.transport_healthy);
  EXPECT_EQ(static_cast<uint8_t>(snapshot.state), static_cast<uint8_t>(MhiTransportState::RECOVERY_ACTIVE));

  esphome::test_millis_value = 110U;
  manager.loop();
  snapshot = manager.diagnostics_snapshot(110U);
  EXPECT_TRUE(snapshot.safe_mode);
  EXPECT_EQ(snapshot.safe_mode_entries, 1U);
  EXPECT_EQ(snapshot.recovery_failures, 1U);
  EXPECT_EQ(static_cast<uint8_t>(snapshot.last_error.code), static_cast<uint8_t>(MhiTransportError::RX_STALLED));

  esphome::test_millis_value = 210U;
  manager.loop();
  snapshot = manager.diagnostics_snapshot(210U);
  EXPECT_EQ(snapshot.safe_mode_entries, 1U);
  EXPECT_EQ(snapshot.recovery_failures, 1U);
}

void transport_diagnostics_publisher_publishes_only_on_change() {
  esphome::binary_sensor::BinarySensor healthy{};
  esphome::binary_sensor::BinarySensor recovery_active{};
  esphome::binary_sensor::BinarySensor safe_mode{};
  esphome::text_sensor::TextSensor active_transport{};
  esphome::text_sensor::TextSensor state{};
  esphome::text_sensor::TextSensor last_error{};

  MhiTransportDiagnosticsPublisher publisher{};
  publisher.set_healthy_binary_sensor(&healthy);
  publisher.set_recovery_active_binary_sensor(&recovery_active);
  publisher.set_safe_mode_binary_sensor(&safe_mode);
  publisher.set_active_transport_text_sensor(&active_transport);
  publisher.set_state_text_sensor(&state);
  publisher.set_last_error_text_sensor(&last_error);

  MhiTransportDiagnosticsSnapshot snapshot{};
  snapshot.state = MhiTransportState::HEALTHY;
  snapshot.active_transport_name = "rmt_spi_rx";
  snapshot.transport_healthy = true;
  snapshot.state_changes = 1U;

  publisher.publish(snapshot);
  EXPECT_TRUE(healthy.state);
  EXPECT_FALSE(recovery_active.state);
  EXPECT_FALSE(safe_mode.state);
  EXPECT_TRUE(active_transport.state == "rmt_spi_rx");
  EXPECT_TRUE(state.state == "healthy");
  EXPECT_TRUE(last_error.state == "none");
  EXPECT_EQ(healthy.publish_count, 1U);

  publisher.publish(snapshot);
  EXPECT_EQ(healthy.publish_count, 1U);
  EXPECT_EQ(state.publish_count, 1U);

  snapshot.state = MhiTransportState::SAFE_MODE;
  snapshot.active_transport_name = "none";
  snapshot.transport_healthy = false;
  snapshot.safe_mode = true;
  snapshot.state_changes = 2U;
  snapshot.last_error =
      MhiTransportResult::failure(MhiTransportError::RMT_SETUP_FAILED, "rmt_new_rx_channel", 261).error;

  publisher.publish(snapshot);
  EXPECT_FALSE(healthy.state);
  EXPECT_TRUE(safe_mode.state);
  EXPECT_TRUE(active_transport.state == "none");
  EXPECT_TRUE(state.state == "safe_mode");
  EXPECT_TRUE(last_error.state == "rmt_setup_failed: rmt_new_rx_channel (native=261)");
  EXPECT_EQ(healthy.publish_count, 2U);
}

}  // namespace mhi_unit_tests
