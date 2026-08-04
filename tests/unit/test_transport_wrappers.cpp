#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esphome/core/hal.h"
#include "mhi_duplex_transport_adapter.h"
#include "mhi_split_transport.h"
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
  EXPECT_TRUE(transport.setup(pins));
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
  EXPECT_TRUE(transport.setup({8, 38, 39}));

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

  EXPECT_TRUE(transport.setup({8, 38, 39}));
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

}  // namespace mhi_unit_tests
