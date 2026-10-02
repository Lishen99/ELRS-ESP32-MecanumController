// Tests for the CRSF protocol code: the checksum, channel packing, the frame
// parser and the telemetry frames the bridge sends to the receiver.
//
//   pio test -e native -f test_crsf
//
// Expected byte values in here are worked out by hand from the CRSF
// definitions (or are published reference frames), not produced by the code
// under test.
#include <unity.h>

#include <cstring>
#include <vector>

#include "eb_crsf.h"
#include "sticks.h"

namespace {

// ---------------------------------------------------------------------------
// Checksum

void test_crc8_matches_dvb_s2_check_value() {
  // CRC-8/DVB-S2 catalogue check value.
  TEST_ASSERT_EQUAL_HEX8(0xBC, eb::crsf::crc8((const uint8_t*)"123456789", 9));
}

// ---------------------------------------------------------------------------
// CRSF channels

void test_centred_channels_pack_to_reference_frame() {
  // The well-known "all channels centred" RC frame.
  static const uint8_t kReference[26] = {
      0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F,
      0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0xAD};
  uint16_t ticks[16];
  for (int i = 0; i < 16; ++i) ticks[i] = 992;
  uint8_t payload[22];
  eb::crsf::packChannels(ticks, payload);
  uint8_t frame[64];
  const size_t n = eb::crsf::buildFrame(frame, eb::crsf::kTypeRcChannels, payload, 22);
  TEST_ASSERT_EQUAL_UINT(26, n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kReference, frame, 26);
}

void test_channel_packing_is_lsb_first() {
  uint16_t ticks[16] = {0};
  ticks[0] = 0x7FF;
  uint8_t payload[22];
  eb::crsf::packChannels(ticks, payload);
  TEST_ASSERT_EQUAL_HEX8(0xFF, payload[0]);
  TEST_ASSERT_EQUAL_HEX8(0x07, payload[1]);
  for (int i = 2; i < 22; ++i) TEST_ASSERT_EQUAL_HEX8(0x00, payload[i]);

  memset(ticks, 0, sizeof ticks);
  ticks[15] = 0x7FF;
  eb::crsf::packChannels(ticks, payload);
  TEST_ASSERT_EQUAL_HEX8(0xE0, payload[20]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, payload[21]);
}

void test_channel_pack_unpack_round_trip() {
  uint16_t in[16];
  uint16_t out[16];
  for (int i = 0; i < 16; ++i) in[i] = (uint16_t)((i * 131 + 7) & 0x7FF);
  uint8_t payload[22];
  eb::crsf::packChannels(in, payload);
  eb::crsf::unpackChannels(payload, out);
  TEST_ASSERT_EQUAL_UINT16_ARRAY(in, out, 16);
}

void test_ticks_to_microseconds() {
  // us = (ticks - 992) * 5 / 8 + 1500
  TEST_ASSERT_EQUAL_UINT16(1500, eb::crsf::ticksToUs(992));
  TEST_ASSERT_EQUAL_UINT16(2000, eb::crsf::ticksToUs(1792));
  TEST_ASSERT_EQUAL_UINT16(2012, eb::crsf::ticksToUs(1811));  // 2011.875
  TEST_ASSERT_EQUAL_UINT16(987, eb::crsf::ticksToUs(172));    // 987.5
  TEST_ASSERT_EQUAL_UINT16(999, eb::crsf::ticksToUs(191));    // 999.375
  TEST_ASSERT_EQUAL_UINT16(880, eb::crsf::ticksToUs(0));
  TEST_ASSERT_EQUAL_UINT16(2159, eb::crsf::ticksToUs(2047));  // 2159.375

  TEST_ASSERT_EQUAL_UINT16(992, eb::crsf::usToTicks(1500));
  TEST_ASSERT_EQUAL_UINT16(192, eb::crsf::usToTicks(1000));
  TEST_ASSERT_EQUAL_UINT16(1792, eb::crsf::usToTicks(2000));
  TEST_ASSERT_EQUAL_UINT16(1811, eb::crsf::usToTicks(2012));
}

// ---------------------------------------------------------------------------
// CRSF parser

void test_parser_extracts_a_frame() {
  Sticks s;
  s.ticks[0] = 1811;
  const std::vector<uint8_t> f = rcFrame(s);
  eb::crsf::Parser parser;
  for (size_t i = 0; i + 1 < f.size(); ++i) TEST_ASSERT_FALSE(parser.feed(f[i]));
  TEST_ASSERT_TRUE(parser.feed(f.back()));
  TEST_ASSERT_EQUAL_HEX8(0xC8, parser.frame().addr);
  TEST_ASSERT_EQUAL_HEX8(0x16, parser.frame().type);
  TEST_ASSERT_EQUAL_UINT8(22, parser.frame().len);
  uint16_t ticks[16];
  eb::crsf::unpackChannels(parser.frame().payload, ticks);
  TEST_ASSERT_EQUAL_UINT16_ARRAY(s.ticks, ticks, 16);
}

void test_parser_resyncs_after_garbage() {
  static const uint8_t kGarbage[] = {0x00, 0xFF, 0xC8, 0x01, 0x55, 0xEC, 0xC8};
  const std::vector<uint8_t> f = rcFrame(Sticks());
  eb::crsf::Parser parser;
  int found = 0;
  for (size_t i = 0; i < sizeof kGarbage; ++i) found += parser.feed(kGarbage[i]);
  for (size_t i = 0; i < f.size(); ++i) found += parser.feed(f[i]);
  // The trailing 0xC8 of the garbage looks like a frame start with length
  // 0xC8, which is rejected, so the real frame right behind it is kept.
  TEST_ASSERT_EQUAL_INT(1, found);
}

void test_parser_rejects_bad_crc_and_keeps_next_frame() {
  std::vector<uint8_t> bad = rcFrame(Sticks());
  bad[10] ^= 0x01;
  const std::vector<uint8_t> good = rcFrame(Sticks());
  eb::crsf::Parser parser;
  int found = 0;
  for (size_t i = 0; i < bad.size(); ++i) found += parser.feed(bad[i]);
  TEST_ASSERT_EQUAL_INT(0, found);
  for (size_t i = 0; i < good.size(); ++i) found += parser.feed(good[i]);
  TEST_ASSERT_EQUAL_INT(1, found);
  TEST_ASSERT_TRUE(parser.errors() >= 1);
}

void test_parser_recovers_frames_swallowed_by_false_sync() {
  // A stray "C8 3E" claims the next 62 bytes. The genuine frames inside that
  // span must still come out once the false frame fails its CRC.
  const std::vector<uint8_t> f = rcFrame(Sticks());
  eb::crsf::Parser parser;
  int found = 0;
  found += parser.feed(0xC8);
  found += parser.feed(0x3E);
  for (int k = 0; k < 4; ++k) {
    for (size_t i = 0; i < f.size(); ++i) found += parser.feed(f[i]);
  }
  TEST_ASSERT_EQUAL_INT(4, found);
}

void test_parser_handles_back_to_back_frames() {
  const std::vector<uint8_t> f = rcFrame(Sticks());
  eb::crsf::Parser parser;
  int found = 0;
  for (int k = 0; k < 10; ++k) {
    for (size_t i = 0; i < f.size(); ++i) found += parser.feed(f[i]);
  }
  TEST_ASSERT_EQUAL_INT(10, found);
  TEST_ASSERT_EQUAL_UINT32(0, parser.errors());
}

// ---------------------------------------------------------------------------
// CRSF frames the bridge sends to the receiver

void test_bind_command_matches_betaflight() {
  // Byte-for-byte what Betaflight's `bind_rx` sends to a CRSF receiver.
  static const uint8_t kExpected[9] = {0xC8, 0x07, 0x32, 0xEC, 0xC8, 0x10, 0x01, 0x9E, 0xE8};
  uint8_t frame[64];
  TEST_ASSERT_EQUAL_UINT(9, eb::crsf::buildBindCommand(frame));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kExpected, frame, 9);
}

// Build a frame, check its framing through the parser, and compare payload.
void expectFrame(const uint8_t* frame, size_t len, uint8_t type, const uint8_t* payload,
                 size_t payloadLen) {
  TEST_ASSERT_EQUAL_UINT(payloadLen + 4, len);
  eb::crsf::Parser parser;
  bool ok = false;
  for (size_t i = 0; i < len; ++i) ok = parser.feed(frame[i]);
  TEST_ASSERT_TRUE_MESSAGE(ok, "frame did not parse");
  TEST_ASSERT_EQUAL_HEX8(type, parser.frame().type);
  TEST_ASSERT_EQUAL_UINT(payloadLen, parser.frame().len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, parser.frame().payload, payloadLen);
}

void test_battery_frame() {
  // 12.6 V -> 126 (0.1 V), 1.5 A -> 15 (0.1 A), 1234 mAh, 80 %
  static const uint8_t kPayload[] = {0x00, 0x7E, 0x00, 0x0F, 0x00, 0x04, 0xD2, 0x50};
  uint8_t frame[64];
  const size_t n = eb::crsf::buildBattery(frame, 12.6f, 1.5f, 1234.0f, 80);
  expectFrame(frame, n, 0x08, kPayload, sizeof kPayload);
}

void test_battery_frame_clamps_and_survives_nan() {
  uint8_t frame[64];
  // Negative current is sent as a signed value; -2.0 A -> -20 -> 0xFFEC.
  static const uint8_t kNegative[] = {0x00, 0x00, 0xFF, 0xEC, 0x00, 0x00, 0x00, 0x64};
  size_t n = eb::crsf::buildBattery(frame, -5.0f, -2.0f, -1.0f, 200);
  expectFrame(frame, n, 0x08, kNegative, sizeof kNegative);

  const float nan = 0.0f / 0.0f;
  static const uint8_t kZero[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  n = eb::crsf::buildBattery(frame, nan, nan, nan, 0);
  expectFrame(frame, n, 0x08, kZero, sizeof kZero);
}

void test_gps_frame() {
  // 37.7749000 N -> 377749000 = 0x1683FE08
  // 122.4194000 W -> -1224194000 = 0xB7084830
  // 2.5 m/s = 9.0 km/h -> 90; 270.5 deg -> 27050 = 0x69AA; 12 m -> 1012 = 0x03F4
  static const uint8_t kPayload[] = {0x16, 0x83, 0xFE, 0x08, 0xB7, 0x08, 0x48, 0x30,
                                     0x00, 0x5A, 0x69, 0xAA, 0x03, 0xF4, 0x09};
  uint8_t frame[64];
  const size_t n = eb::crsf::buildGps(frame, 377749000, -1224194000, 2.5f, 270.5f, 12.0f, 9);
  expectFrame(frame, n, 0x02, kPayload, sizeof kPayload);
}

void test_gps_heading_wraps() {
  uint8_t frame[64];
  // -90 deg is 270 deg -> 27000 = 0x6978
  eb::crsf::buildGps(frame, 0, 0, 0, -90.0f, 0, 0);
  TEST_ASSERT_EQUAL_HEX8(0x69, frame[3 + 10]);
  TEST_ASSERT_EQUAL_HEX8(0x78, frame[3 + 11]);
  // 360 deg wraps to 0
  eb::crsf::buildGps(frame, 0, 0, 0, 360.0f, 0, 0);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[3 + 10]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[3 + 11]);
}

void test_attitude_frame() {
  // Wire order is pitch, roll, yaw in units of 1/10000 rad.
  // pitch -0.2 -> -2000 = 0xF830; roll 0.1 -> 1000 = 0x03E8; yaw 3.0 -> 30000 = 0x7530
  static const uint8_t kPayload[] = {0xF8, 0x30, 0x03, 0xE8, 0x75, 0x30};
  uint8_t frame[64];
  const size_t n = eb::crsf::buildAttitude(frame, 0.1f, -0.2f, 3.0f);
  expectFrame(frame, n, 0x1E, kPayload, sizeof kPayload);
}

void test_attitude_wraps_into_int16_range() {
  // 4.0 rad is outside +-pi and would overflow int16 at 40000; it must wrap
  // to 4 - 2*pi = -2.28319 rad -> -22832 = 0xA6D0.
  uint8_t frame[64];
  eb::crsf::buildAttitude(frame, 0.0f, 0.0f, 4.0f);
  TEST_ASSERT_EQUAL_HEX8(0xA6, frame[3 + 4]);
  TEST_ASSERT_EQUAL_HEX8(0xD0, frame[3 + 5]);
}

void test_flight_mode_frame() {
  static const uint8_t kPayload[] = {'A', 'U', 'T', 'O', 0x00};
  uint8_t frame[64];
  size_t n = eb::crsf::buildFlightMode(frame, "AUTO");
  expectFrame(frame, n, 0x21, kPayload, sizeof kPayload);

  // Longer text is cut to 15 characters and still terminated.
  n = eb::crsf::buildFlightMode(frame, "ABCDEFGHIJKLMNOPQRSTUVWXYZ");
  TEST_ASSERT_EQUAL_UINT(15 + 1 + 4, n);
  TEST_ASSERT_EQUAL_HEX8('O', frame[3 + 14]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[3 + 15]);
}

void test_vario_frame() {
  static const uint8_t kUp[] = {0x00, 0x7D};    // 1.25 m/s -> 125 cm/s
  static const uint8_t kDown[] = {0xFF, 0xCE};  // -0.5 m/s -> -50 cm/s
  uint8_t frame[64];
  size_t n = eb::crsf::buildVario(frame, 1.25f);
  expectFrame(frame, n, 0x07, kUp, sizeof kUp);
  n = eb::crsf::buildVario(frame, -0.5f);
  expectFrame(frame, n, 0x07, kDown, sizeof kDown);
}

void test_baro_frame() {
  // 123.4 m -> 1234 dm + 10000 = 11234 = 0x2BE2; 0.5 m/s -> 50 cm/s
  static const uint8_t kLow[] = {0x2B, 0xE2, 0x00, 0x32};
  // Above 2276.7 m the altitude switches to whole metres with the top bit set.
  static const uint8_t kHigh[] = {0x8B, 0xB8, 0x00, 0x00};  // 3000 m
  // -50 m -> -500 dm + 10000 = 9500 = 0x251C
  static const uint8_t kNegative[] = {0x25, 0x1C, 0x00, 0x00};
  uint8_t frame[64];
  size_t n = eb::crsf::buildBaroAltitude(frame, 123.4f, 0.5f);
  expectFrame(frame, n, 0x09, kLow, sizeof kLow);
  n = eb::crsf::buildBaroAltitude(frame, 3000.0f, 0.0f);
  expectFrame(frame, n, 0x09, kHigh, sizeof kHigh);
  n = eb::crsf::buildBaroAltitude(frame, -50.0f, 0.0f);
  expectFrame(frame, n, 0x09, kNegative, sizeof kNegative);
}

void test_airspeed_frame() {
  static const uint8_t kPayload[] = {0x01, 0x68};  // 10 m/s = 36 km/h -> 360
  uint8_t frame[64];
  const size_t n = eb::crsf::buildAirspeed(frame, 10.0f);
  expectFrame(frame, n, 0x0A, kPayload, sizeof kPayload);
}

void test_rpm_frame() {
  // source id, then signed 24-bit values: 1000 = 0x0003E8, -200 = 0xFFFF38
  static const uint8_t kPayload[] = {0x00, 0x00, 0x03, 0xE8, 0xFF, 0xFF, 0x38};
  const int32_t rpm[2] = {1000, -200};
  uint8_t frame[64];
  const size_t n = eb::crsf::buildRpm(frame, 0, rpm, 2);
  expectFrame(frame, n, 0x0C, kPayload, sizeof kPayload);
  TEST_ASSERT_EQUAL_UINT(0, eb::crsf::buildRpm(frame, 0, rpm, 0));
}

void test_temperature_frame() {
  // source id, then 0.1 C units: 25.5 -> 255 = 0x00FF, -10.0 -> -100 = 0xFF9C
  static const uint8_t kPayload[] = {0x01, 0x00, 0xFF, 0xFF, 0x9C};
  const float degC[2] = {25.5f, -10.0f};
  uint8_t frame[64];
  const size_t n = eb::crsf::buildTemperature(frame, 1, degC, 2);
  expectFrame(frame, n, 0x0D, kPayload, sizeof kPayload);
}

void test_link_stats_decode() {
  // RSSI is sent as a positive magnitude; SNR is signed.
  static const uint8_t kPayload[10] = {80, 95, 97, 0xFD, 1, 7, 3, 0, 0, 0};
  eb::crsf::LinkStats s;
  TEST_ASSERT_TRUE(eb::crsf::decodeLinkStats(kPayload, 10, s));
  TEST_ASSERT_EQUAL_INT8(-80, s.upRssi1);
  TEST_ASSERT_EQUAL_INT8(-95, s.upRssi2);
  TEST_ASSERT_EQUAL_UINT8(97, s.upLq);
  TEST_ASSERT_EQUAL_INT8(-3, s.upSnr);
  TEST_ASSERT_EQUAL_UINT8(1, s.antenna);
  TEST_ASSERT_EQUAL_UINT8(7, s.rfMode);
  TEST_ASSERT_EQUAL_UINT16(100, s.txPowerMw);
  TEST_ASSERT_FALSE(eb::crsf::decodeLinkStats(kPayload, 9, s));

  // A two's-complement dBm value is accepted too: 0xC4 = -60.
  static const uint8_t kSigned[10] = {0xC4, 0, 0, 0, 0, 0, 8, 0, 0, 0};
  TEST_ASSERT_TRUE(eb::crsf::decodeLinkStats(kSigned, 10, s));
  TEST_ASSERT_EQUAL_INT8(-60, s.upRssi1);
  TEST_ASSERT_EQUAL_UINT16(50, s.txPowerMw);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc8_matches_dvb_s2_check_value);

  RUN_TEST(test_centred_channels_pack_to_reference_frame);
  RUN_TEST(test_channel_packing_is_lsb_first);
  RUN_TEST(test_channel_pack_unpack_round_trip);
  RUN_TEST(test_ticks_to_microseconds);

  RUN_TEST(test_parser_extracts_a_frame);
  RUN_TEST(test_parser_resyncs_after_garbage);
  RUN_TEST(test_parser_rejects_bad_crc_and_keeps_next_frame);
  RUN_TEST(test_parser_recovers_frames_swallowed_by_false_sync);
  RUN_TEST(test_parser_handles_back_to_back_frames);

  RUN_TEST(test_bind_command_matches_betaflight);
  RUN_TEST(test_battery_frame);
  RUN_TEST(test_battery_frame_clamps_and_survives_nan);
  RUN_TEST(test_gps_frame);
  RUN_TEST(test_gps_heading_wraps);
  RUN_TEST(test_attitude_frame);
  RUN_TEST(test_attitude_wraps_into_int16_range);
  RUN_TEST(test_flight_mode_frame);
  RUN_TEST(test_vario_frame);
  RUN_TEST(test_baro_frame);
  RUN_TEST(test_airspeed_frame);
  RUN_TEST(test_rpm_frame);
  RUN_TEST(test_temperature_frame);
  RUN_TEST(test_link_stats_decode);
  return UNITY_END();
}
