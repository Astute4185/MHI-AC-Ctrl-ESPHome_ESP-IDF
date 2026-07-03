#include "mhi_test_common.h"

namespace mhi_unit_tests {

void frame_classifier_classifies_status_opdata_and_extended_status() {
  const MhiFrameBuffer status = make_mosi_status_frame();
  const MhiFrameClassification status_classification = classify_mhi_mosi_frame(status.view());
  EXPECT_EQ(static_cast<int>(status_classification.kind), static_cast<int>(MhiFrameKind::STATUS));
  EXPECT_EQ(status_classification.opdata_key, kMhiInvalidOpDataKey);

  const MhiFrameBuffer opdata = make_legacy_opdata_frame(0x00, 0x80, 0x10, 202);
  const MhiFrameClassification opdata_classification = classify_mhi_mosi_frame(opdata.view());
  EXPECT_EQ(static_cast<int>(opdata_classification.kind), static_cast<int>(MhiFrameKind::OPDATA));
  EXPECT_EQ(opdata_classification.opdata_key, 0x8010U);

  const MhiFrameBuffer extended = make_mosi_status_frame_33(2U, false, false);
  const MhiFrameClassification extended_classification = classify_mhi_mosi_frame(extended.view());
  EXPECT_EQ(static_cast<int>(extended_classification.kind), static_cast<int>(MhiFrameKind::EXTENDED_STATUS));
  EXPECT_EQ(extended_classification.opdata_key, kMhiInvalidOpDataKey);
}

void frame_catalog_overwrites_repeated_status_with_latest() {
  MhiFrameCatalog catalog{};

  MhiFrameBuffer first = make_mosi_status_frame();
  first.data[DB3] = 100U;
  const uint16_t first_checksum = mhi_calc_checksum(first.data);
  first.data[CBH] = static_cast<uint8_t>((first_checksum >> 8U) & 0xFFU);
  first.data[CBL] = static_cast<uint8_t>(first_checksum & 0xFFU);

  MhiFrameBuffer second = make_mosi_status_frame();
  second.data[DB3] = 120U;
  const uint16_t second_checksum = mhi_calc_checksum(second.data);
  second.data[CBH] = static_cast<uint8_t>((second_checksum >> 8U) & 0xFFU);
  second.data[CBL] = static_cast<uint8_t>(second_checksum & 0xFFU);

  const auto first_result = catalog.ingest_mosi_frame(first.view(), 1U, 1000U);
  const auto second_result = catalog.ingest_mosi_frame(second.view(), 2U, 1010U);

  EXPECT_TRUE(first_result.stored);
  EXPECT_FALSE(first_result.overwritten);
  EXPECT_TRUE(second_result.stored);
  EXPECT_TRUE(second_result.overwritten);
  EXPECT_EQ(catalog.stats().status_frames, 2U);
  EXPECT_EQ(catalog.stats().overwritten_frames, 1U);

  MhiCatalogedFrame latest{};
  EXPECT_TRUE(catalog.take_latest_status(latest));
  EXPECT_EQ(latest.sequence, 2U);
  EXPECT_EQ(latest.last_update_ms, 1010U);
  EXPECT_EQ(latest.frame.data[DB3], 120U);
  EXPECT_FALSE(catalog.take_latest_status(latest));
}

void frame_catalog_keeps_opdata_slots_separate_by_key() {
  MhiFrameCatalog catalog{};

  const MhiFrameBuffer outdoor = make_legacy_opdata_frame(0x00, 0x80, 0x10, 202);
  const MhiFrameBuffer return_air = make_legacy_opdata_frame(0x80, 0x80, 0x20, 168);
  const MhiFrameBuffer newer_outdoor = make_legacy_opdata_frame(0x00, 0x80, 0x10, 210);

  const auto first = catalog.ingest_mosi_frame(outdoor.view(), 1U, 1000U);
  const auto second = catalog.ingest_mosi_frame(return_air.view(), 2U, 1010U);
  const auto third = catalog.ingest_mosi_frame(newer_outdoor.view(), 3U, 1020U);

  EXPECT_TRUE(first.stored);
  EXPECT_TRUE(second.stored);
  EXPECT_TRUE(third.stored);
  EXPECT_TRUE(third.overwritten);
  EXPECT_EQ(catalog.stats().opdata_frames, 3U);
  EXPECT_EQ(catalog.stats().overwritten_frames, 1U);

  MhiCatalogedFrame latest_outdoor{};
  EXPECT_TRUE(catalog.take_latest_opdata(first.opdata_key, latest_outdoor));
  EXPECT_EQ(latest_outdoor.sequence, 3U);
  EXPECT_EQ(latest_outdoor.frame.data[DB11], 210U);

  MhiCatalogedFrame latest_return_air{};
  EXPECT_TRUE(catalog.take_latest_opdata(second.opdata_key, latest_return_air));
  EXPECT_EQ(latest_return_air.sequence, 2U);
  EXPECT_EQ(latest_return_air.frame.data[DB11], 168U);
}

void frame_catalog_reports_unknown_frames() {
  MhiFrameCatalog catalog{};
  MhiFrameBuffer unknown = make_mosi_status_frame();
  unknown.data[SB0] = 0x00U;

  const auto result = catalog.ingest_mosi_frame(unknown.view(), 10U, 2000U);
  EXPECT_TRUE(result.stored);
  EXPECT_EQ(static_cast<int>(result.kind), static_cast<int>(MhiFrameKind::UNKNOWN));
  EXPECT_EQ(catalog.stats().unknown_frames, 1U);

  MhiCatalogedFrame latest{};
  EXPECT_TRUE(catalog.take_latest_unknown(latest));
  EXPECT_EQ(static_cast<int>(latest.kind), static_cast<int>(MhiFrameKind::UNKNOWN));
  EXPECT_EQ(latest.sequence, 10U);
}

}  // namespace mhi_unit_tests
