#include "mhi_test_common.h"

namespace mhi_unit_tests {

void opdata_decoder_decodes_outdoor_temp() {
  const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x80, 0x10, 202);

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_outdoor_temp);
  expect_near(decoded.outdoor_temp_c, 27.0f);
}

void opdata_decoder_decodes_return_air_temp() {
  const MhiFrameBuffer frame = make_legacy_opdata_frame(0x80, 0x80, 0x20, 168);

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_return_air);
  expect_near(decoded.return_air_c, 27.0f);
}

void opdata_decoder_decodes_compressor_frequency() {
  const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x11, 0x11, 100);

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_compressor_frequency);
  expect_near(decoded.compressor_frequency_hz, 35.6f);
}

void opdata_decoder_decodes_current() {
  const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x90, 0x10, 51);

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_current);
  expect_near(decoded.current_a, 14.0f);
}

void opdata_decoder_decodes_indoor_unit_fan_speed() {
  const MhiFrameBuffer frame = make_legacy_opdata_frame(0x80, 0x1F, 0x15, 0x00);

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_indoor_unit_fan_speed);
  EXPECT_EQ(decoded.indoor_unit_fan_speed, 5);
  EXPECT_FALSE(decoded.has_outdoor_unit_fan_speed);
}

void opdata_decoder_decodes_outdoor_unit_fan_speed() {
  const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x1F, 0x13, 0x00);

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_outdoor_unit_fan_speed);
  EXPECT_EQ(decoded.outdoor_unit_fan_speed, 3);
  EXPECT_FALSE(decoded.has_indoor_unit_fan_speed);
}

void opdata_decoder_decodes_indoor_unit_total_run_time() {
  const MhiFrameBuffer frame = make_legacy_opdata_frame(0x80, 0x1E, 0x10, 12);

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_total_indoor_runtime);
  EXPECT_EQ(decoded.total_indoor_runtime_hours, 1200U);
}

void opdata_decoder_decodes_compressor_total_run_time() {
  const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x1E, 0x11, 34);

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_total_compressor_runtime);
  EXPECT_EQ(decoded.total_compressor_runtime_hours, 3400U);
}

void opdata_decoder_decodes_energy_used() {
  const MhiFrameBuffer frame = make_legacy_opdata_frame(0x80, 0x94, 0x10, 0x20, 0x03);

  MhiDecodedOpData decoded{};
  EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));

  EXPECT_TRUE(decoded.valid);
  EXPECT_TRUE(decoded.has_energy_used);
  expect_near(decoded.energy_used_kwh, 200.0f);
}


void opdata_decoder_decodes_temperature_and_protection_slice2() {
  {
    const MhiFrameBuffer frame = make_legacy_opdata_frame(0x80, 0x81, 0x20, 100);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_indoor_unit_thi_r1);
    expect_near(decoded.indoor_unit_thi_r1_c, 21.3f, 0.01f);
  }

  {
    const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x81, 0x10, 200);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_indoor_unit_thi_r2);
    expect_near(decoded.indoor_unit_thi_r2_c, 8.0f, 0.01f);
  }

  {
    const MhiFrameBuffer frame = make_legacy_opdata_frame(0x80, 0x87, 0x10, 110);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_indoor_unit_thi_r3);
    expect_near(decoded.indoor_unit_thi_r3_c, 24.57f, 0.01f);
  }

  {
    const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x82, 0x10, 120);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_outdoor_unit_tho_r1);
    expect_near(decoded.outdoor_unit_tho_r1_c, 27.84f, 0.01f);
  }

  {
    const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x85, 0x10, 40);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_outdoor_unit_discharge_pipe);
    expect_near(decoded.outdoor_unit_discharge_pipe_c, 52.0f, 0.01f);
  }

  {
    const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0xB1, 0x10, 22);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_outdoor_unit_discharge_pipe_super_heat);
    expect_near(decoded.outdoor_unit_discharge_pipe_super_heat_c, 11.0f, 0.01f);
  }

  {
    const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x13, 0x10, 0x34, 0x12);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_outdoor_unit_expansion_valve);
    EXPECT_EQ(decoded.outdoor_unit_expansion_valve_pulses, 0x1234U);
  }

  {
    const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x7C, 0x10, 8);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_protection_state_number);
    EXPECT_EQ(decoded.protection_state_number, 8U);
  }

  {
    const MhiFrameBuffer frame = make_legacy_opdata_frame(0x00, 0x0C, 0x11, 0);
    MhiDecodedOpData decoded{};
    EXPECT_TRUE(MhiOpDataDecoder::decode_mosi(frame.view(), decoded));
    EXPECT_TRUE(decoded.has_defrost);
    EXPECT_TRUE(decoded.defrost);
  }
}

}  // namespace mhi_unit_tests
