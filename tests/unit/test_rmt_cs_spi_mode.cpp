#include "mhi_test_common.h"

namespace mhi_unit_tests {

void rmt_cs_spi_mode_keeps_dma_s3_only() {
  EXPECT_FALSE(mhi_rmt_cs_spi_mode_supported(MhiRmtCsSpiTarget::ESP32, MhiRmtCsSpiBufferMode::DMA));
  EXPECT_TRUE(mhi_rmt_cs_spi_mode_supported(MhiRmtCsSpiTarget::ESP32_S3, MhiRmtCsSpiBufferMode::DMA));
  EXPECT_FALSE(mhi_rmt_cs_spi_mode_supported(MhiRmtCsSpiTarget::UNSUPPORTED, MhiRmtCsSpiBufferMode::DMA));
  EXPECT_TRUE(mhi_rmt_cs_spi_uses_dma(MhiRmtCsSpiBufferMode::DMA));
}

void rmt_cs_spi_mode_supports_fifo_on_esp32_and_s3() {
  EXPECT_TRUE(mhi_rmt_cs_spi_mode_supported(MhiRmtCsSpiTarget::ESP32, MhiRmtCsSpiBufferMode::FIFO));
  EXPECT_TRUE(mhi_rmt_cs_spi_mode_supported(MhiRmtCsSpiTarget::ESP32_S3, MhiRmtCsSpiBufferMode::FIFO));
  EXPECT_FALSE(mhi_rmt_cs_spi_mode_supported(MhiRmtCsSpiTarget::UNSUPPORTED, MhiRmtCsSpiBufferMode::FIFO));
  EXPECT_FALSE(mhi_rmt_cs_spi_uses_dma(MhiRmtCsSpiBufferMode::FIFO));
}

void rmt_cs_spi_mode_exposes_distinct_driver_names() {
  EXPECT_TRUE(std::string(mhi_rmt_cs_spi_driver_name(MhiRmtCsSpiBufferMode::DMA)) == "rmt_cs_spi");
  EXPECT_TRUE(std::string(mhi_rmt_cs_spi_driver_name(MhiRmtCsSpiBufferMode::FIFO)) == "rmt_cs_spi_nodma");
  EXPECT_TRUE(std::string(mhi_rmt_cs_spi_buffer_mode_name(MhiRmtCsSpiBufferMode::DMA)) == "DMA");
  EXPECT_TRUE(std::string(mhi_rmt_cs_spi_buffer_mode_name(MhiRmtCsSpiBufferMode::FIFO)) == "FIFO");
}

}  // namespace mhi_unit_tests
