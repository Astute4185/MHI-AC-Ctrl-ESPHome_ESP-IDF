#pragma once

#include <cstdint>

namespace esphome {
namespace mhi_ac_ctrl {

enum class MhiTransportError : uint8_t {
  NONE = 0,
  INVALID_CONFIGURATION,
  INVALID_PIN,
  UNSUPPORTED_TARGET,
  COMPONENT_UNAVAILABLE,
  RESOURCE_ALLOCATION_FAILED,
  DRIVER_NOT_BOUND,
  RX_SETUP_FAILED,
  TX_SETUP_FAILED,
  DUPLEX_SETUP_FAILED,
  GPIO_SETUP_FAILED,
  INTERRUPT_SETUP_FAILED,
  RMT_SETUP_FAILED,
  SPI_SETUP_FAILED,
  DMA_SETUP_FAILED,
  QUEUE_SETUP_FAILED,
  NO_TRAFFIC,
  INVALID_TRAFFIC,
  RX_STALLED,
  TX_STALLED,
  TX_FAILURE_LIMIT,
  INTERNAL_INVARIANT,
};

inline const char* mhi_transport_error_name(MhiTransportError error) {
  switch (error) {
    case MhiTransportError::NONE:
      return "none";
    case MhiTransportError::INVALID_CONFIGURATION:
      return "invalid_configuration";
    case MhiTransportError::INVALID_PIN:
      return "invalid_pin";
    case MhiTransportError::UNSUPPORTED_TARGET:
      return "unsupported_target";
    case MhiTransportError::COMPONENT_UNAVAILABLE:
      return "component_unavailable";
    case MhiTransportError::RESOURCE_ALLOCATION_FAILED:
      return "resource_allocation_failed";
    case MhiTransportError::DRIVER_NOT_BOUND:
      return "driver_not_bound";
    case MhiTransportError::RX_SETUP_FAILED:
      return "rx_setup_failed";
    case MhiTransportError::TX_SETUP_FAILED:
      return "tx_setup_failed";
    case MhiTransportError::DUPLEX_SETUP_FAILED:
      return "duplex_setup_failed";
    case MhiTransportError::GPIO_SETUP_FAILED:
      return "gpio_setup_failed";
    case MhiTransportError::INTERRUPT_SETUP_FAILED:
      return "interrupt_setup_failed";
    case MhiTransportError::RMT_SETUP_FAILED:
      return "rmt_setup_failed";
    case MhiTransportError::SPI_SETUP_FAILED:
      return "spi_setup_failed";
    case MhiTransportError::DMA_SETUP_FAILED:
      return "dma_setup_failed";
    case MhiTransportError::QUEUE_SETUP_FAILED:
      return "queue_setup_failed";
    case MhiTransportError::NO_TRAFFIC:
      return "no_traffic";
    case MhiTransportError::INVALID_TRAFFIC:
      return "invalid_traffic";
    case MhiTransportError::RX_STALLED:
      return "rx_stalled";
    case MhiTransportError::TX_STALLED:
      return "tx_stalled";
    case MhiTransportError::TX_FAILURE_LIMIT:
      return "tx_failure_limit";
    case MhiTransportError::INTERNAL_INVARIANT:
      return "internal_invariant";
  }
  return "unknown";
}

struct MhiTransportErrorDetail {
  MhiTransportError code{MhiTransportError::NONE};
  int32_t native_code{0};
  const char* operation{nullptr};
  const char* message{nullptr};

  bool present() const {
    return code != MhiTransportError::NONE;
  }
};

struct MhiTransportResult {
  bool ok{true};
  MhiTransportErrorDetail error{};

  explicit operator bool() const {
    return ok;
  }

  static MhiTransportResult success() {
    return {};
  }

  static MhiTransportResult failure(MhiTransportError code, const char* operation, int32_t native_code = 0,
                                    const char* message = nullptr) {
    MhiTransportResult result{};
    result.ok = false;
    result.error.code = code;
    result.error.native_code = native_code;
    result.error.operation = operation;
    result.error.message = message;
    return result;
  }
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
