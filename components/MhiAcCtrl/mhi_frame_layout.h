#pragma once

#include <cstddef>
#include <cstdint>

constexpr std::size_t kMhiMaxFrameBytes = 33U;

// constants for the frame
constexpr uint8_t SB0 = 0;
constexpr uint8_t SB1 = SB0 + 1;
constexpr uint8_t SB2 = SB0 + 2;
constexpr uint8_t DB0 = SB2 + 1;
constexpr uint8_t DB1 = SB2 + 2;
constexpr uint8_t DB2 = SB2 + 3;
constexpr uint8_t DB3 = SB2 + 4;
constexpr uint8_t DB4 = SB2 + 5;
constexpr uint8_t DB6 = SB2 + 7;
constexpr uint8_t DB9 = SB2 + 10;
constexpr uint8_t DB10 = SB2 + 11;
constexpr uint8_t DB11 = SB2 + 12;
constexpr uint8_t DB12 = SB2 + 13;
constexpr uint8_t DB14 = SB2 + 15;
constexpr uint8_t CBH = DB14 + 1;
constexpr uint8_t CBL = DB14 + 2;
constexpr uint8_t DB15 = CBL + 1;
constexpr uint8_t DB16 = CBL + 2;
constexpr uint8_t DB17 = CBL + 3;
constexpr uint8_t DB18 = CBL + 4;
constexpr uint8_t DB19 = CBL + 5;
constexpr uint8_t DB20 = CBL + 6;
constexpr uint8_t DB21 = CBL + 7;
constexpr uint8_t DB22 = CBL + 8;
constexpr uint8_t DB23 = CBL + 9;
constexpr uint8_t DB24 = CBL + 10;
constexpr uint8_t DB25 = CBL + 11;
constexpr uint8_t DB26 = CBL + 12;
constexpr uint8_t CBL2 = DB26 + 1;

inline uint16_t mhi_calc_checksum(const uint8_t *frame) {
  uint16_t checksum = 0;
  for (uint8_t i = 0; i < CBH; i++) {
    checksum += frame[i];
  }
  return checksum;
}

inline uint16_t mhi_calc_checksum_frame33(const uint8_t *frame) {
  uint16_t checksum = 0;
  for (uint8_t i = 0; i < CBL2; i++) {
    checksum += frame[i];
  }
  return checksum;
}
