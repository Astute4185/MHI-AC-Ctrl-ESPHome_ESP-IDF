#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace mhi_ac_ctrl {

constexpr std::size_t kMhiFrame20Bytes = 20U;
constexpr std::size_t kMhiFrame33Bytes = 33U;
constexpr std::size_t kMhiMaxFrameBytes = kMhiFrame33Bytes;

// MOSI frame signature from AC
constexpr uint8_t kMhiMosiSignature0Default = 0x6C;
constexpr uint8_t kMhiMosiSignature0Alt = 0x6D;
constexpr uint8_t kMhiMosiSignature1 = 0x80;
constexpr uint8_t kMhiMosiSignature2 = 0x04;

// MISO frame signature from controller
constexpr uint8_t kMhiMisoSignature0 = 0xA9;
constexpr uint8_t kMhiMisoSignature1 = 0x00;
constexpr uint8_t kMhiMisoSignature2 = 0x07;

// Frame byte indexes
constexpr std::size_t SB0 = 0U;
constexpr std::size_t SB1 = SB0 + 1U;
constexpr std::size_t SB2 = SB0 + 2U;

constexpr std::size_t DB0 = SB2 + 1U;
constexpr std::size_t DB1 = SB2 + 2U;
constexpr std::size_t DB2 = SB2 + 3U;
constexpr std::size_t DB3 = SB2 + 4U;
constexpr std::size_t DB4 = SB2 + 5U;

// DB5 exists but was not explicitly used in the old layout.
constexpr std::size_t DB5 = SB2 + 6U;
constexpr std::size_t DB6 = SB2 + 7U;
constexpr std::size_t DB7 = SB2 + 8U;
constexpr std::size_t DB8 = SB2 + 9U;
constexpr std::size_t DB9 = SB2 + 10U;
constexpr std::size_t DB10 = SB2 + 11U;
constexpr std::size_t DB11 = SB2 + 12U;
constexpr std::size_t DB12 = SB2 + 13U;
constexpr std::size_t DB13 = SB2 + 14U;
constexpr std::size_t DB14 = SB2 + 15U;

constexpr std::size_t CBH = DB14 + 1U;
constexpr std::size_t CBL = DB14 + 2U;

// Extended 33-byte frame area
constexpr std::size_t DB15 = CBL + 1U;
constexpr std::size_t DB16 = CBL + 2U;
constexpr std::size_t DB17 = CBL + 3U;
constexpr std::size_t DB18 = CBL + 4U;
constexpr std::size_t DB19 = CBL + 5U;
constexpr std::size_t DB20 = CBL + 6U;
constexpr std::size_t DB21 = CBL + 7U;
constexpr std::size_t DB22 = CBL + 8U;
constexpr std::size_t DB23 = CBL + 9U;
constexpr std::size_t DB24 = CBL + 10U;
constexpr std::size_t DB25 = CBL + 11U;
constexpr std::size_t DB26 = CBL + 12U;

constexpr std::size_t CBL2 = DB26 + 1U;

static_assert(CBH == 18U, "MHI CBH index should be 18");
static_assert(CBL == 19U, "MHI CBL index should be 19");
static_assert(CBL2 == 32U, "MHI CBL2 index should be 32");
static_assert(kMhiMaxFrameBytes == 33U, "MHI max frame size should be 33 bytes");

}  // namespace mhi_ac_ctrl
}  // namespace esphome