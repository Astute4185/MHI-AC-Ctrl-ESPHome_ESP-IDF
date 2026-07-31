#include "mhi_test_common.h"

namespace mhi_unit_tests {

void rmt_cs_spi_supports_esp32_and_s3() {
  EXPECT_TRUE(mhi_rmt_cs_spi_target_supported(MhiRmtCsSpiTarget::ESP32));
  EXPECT_TRUE(mhi_rmt_cs_spi_target_supported(MhiRmtCsSpiTarget::ESP32_S3));
  EXPECT_FALSE(mhi_rmt_cs_spi_target_supported(MhiRmtCsSpiTarget::UNSUPPORTED));
}

void rmt_cs_spi_exposes_single_driver_name() {
  EXPECT_TRUE(std::string(mhi_rmt_cs_spi_driver_name()) == "rmt_cs_spi");
}

void rmt_cs_spi_applies_original_esp32_mode3_edge_fix() {
  EXPECT_TRUE(mhi_rmt_cs_spi_needs_mode3_edge_fix(MhiRmtCsSpiTarget::ESP32));
  EXPECT_FALSE(mhi_rmt_cs_spi_needs_mode3_edge_fix(MhiRmtCsSpiTarget::ESP32_S3));
  EXPECT_FALSE(mhi_rmt_cs_spi_needs_mode3_edge_fix(MhiRmtCsSpiTarget::UNSUPPORTED));
}

}  // namespace mhi_unit_tests
