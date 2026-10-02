// Tests for the wire format shared by the firmware and the host libraries
// (elrs_bridge_protocol.h): framing, checksum, and each message's layout.
//
//   pio test -e native -f test_host_protocol
#include <unity.h>

#include <cstring>

#include <elrs_bridge_protocol.h>

namespace {

// ---------------------------------------------------------------------------
// Checksum

void test_crc16_matches_ccitt_false_check_value() {
  // CRC-16/CCITT-FALSE catalogue check value.
  TEST_ASSERT_EQUAL_HEX16(0x29B1, elrsb::crc16((const uint8_t*)"123456789", 9));
}

// ---------------------------------------------------------------------------
// Frames and messages

void test_host_frame_round_trip() {
  static const uint8_t kPayload[] = {1, 2, 3, 0xEB, 0x90, 0xFF};
  uint8_t frame[elrsb::kMaxFrame];
  const size_t n = elrsb::encodeFrame(frame, 0x42, kPayload, sizeof kPayload);
  TEST_ASSERT_EQUAL_UINT(sizeof kPayload + 6, n);
  TEST_ASSERT_EQUAL_HEX8(0xEB, frame[0]);
  TEST_ASSERT_EQUAL_HEX8(0x90, frame[1]);
  TEST_ASSERT_EQUAL_HEX8(sizeof kPayload, frame[2]);
  TEST_ASSERT_EQUAL_HEX8(0x42, frame[3]);

  elrsb::FrameParser parser;
  for (size_t i = 0; i + 1 < n; ++i) TEST_ASSERT_EQUAL(elrsb::kFeedConsumed, parser.feed(frame[i]));
  TEST_ASSERT_EQUAL(elrsb::kFeedFrame, parser.feed(frame[n - 1]));
  TEST_ASSERT_EQUAL_HEX8(0x42, parser.type());
  TEST_ASSERT_EQUAL_UINT8(sizeof kPayload, parser.length());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kPayload, parser.payload(), sizeof kPayload);
  TEST_ASSERT_TRUE(parser.idle());

  // Empty payload and the largest payload both work; one byte more does not.
  TEST_ASSERT_EQUAL_UINT(6, elrsb::encodeFrame(frame, 0x20, nullptr, 0));
  for (size_t i = 0; i < 5; ++i) parser.feed(frame[i]);
  TEST_ASSERT_EQUAL(elrsb::kFeedFrame, parser.feed(frame[5]));
  uint8_t big[elrsb::kMaxPayload + 1] = {0};
  TEST_ASSERT_EQUAL_UINT(elrsb::kMaxPayload + 6, elrsb::encodeFrame(frame, 1, big, elrsb::kMaxPayload));
  TEST_ASSERT_EQUAL_UINT(0, elrsb::encodeFrame(frame, 1, big, elrsb::kMaxPayload + 1));
}

void test_host_parser_separates_text_from_frames() {
  elrsb::FrameParser parser;
  TEST_ASSERT_EQUAL(elrsb::kFeedText, parser.feed('h'));
  TEST_ASSERT_EQUAL(elrsb::kFeedText, parser.feed('\n'));

  // A lone sync byte followed by ordinary text gives the text back.
  TEST_ASSERT_EQUAL(elrsb::kFeedConsumed, parser.feed(0xEB));
  TEST_ASSERT_EQUAL(elrsb::kFeedText, parser.feed('x'));
  TEST_ASSERT_TRUE(parser.idle());

  // Repeated first sync bytes still lead into a frame.
  uint8_t frame[elrsb::kMaxFrame];
  const size_t n = elrsb::encodeFrame(frame, 0x20, nullptr, 0);
  TEST_ASSERT_EQUAL(elrsb::kFeedConsumed, parser.feed(0xEB));
  elrsb::FeedResult r = elrsb::kFeedText;
  for (size_t i = 0; i < n; ++i) r = parser.feed(frame[i]);
  TEST_ASSERT_EQUAL(elrsb::kFeedFrame, r);
}

void test_host_parser_rejects_corruption() {
  static const uint8_t kPayload[] = {9, 8, 7};
  uint8_t frame[elrsb::kMaxFrame];
  const size_t n = elrsb::encodeFrame(frame, 0x30, kPayload, sizeof kPayload);
  for (size_t corrupt = 2; corrupt < n; ++corrupt) {
    elrsb::FrameParser parser;
    bool got = false;
    for (size_t i = 0; i < n; ++i) {
      const uint8_t b = (uint8_t)(i == corrupt ? frame[i] ^ 0x04 : frame[i]);
      got = got || parser.feed(b) == elrsb::kFeedFrame;
    }
    TEST_ASSERT_FALSE_MESSAGE(got, "a corrupted frame was accepted");
  }
}

void test_state_pack_unpack() {
  elrsb::State in;
  in.flags = elrsb::kFlagLinkUp | elrsb::kFlagArmed;
  in.lq = 99;
  in.rssi = -71;
  in.snr = -4;
  in.ageMs = 0x1234;
  in.seq = 200;
  in.count = 16;
  for (int i = 0; i < 16; ++i) in.ch[i] = (int16_t)(-1000 + i * 125);
  uint8_t payload[elrsb::kMaxPayload];
  const uint8_t n = elrsb::packState(payload, in);
  TEST_ASSERT_EQUAL_UINT8(8 + 32, n);
  // Spot-check the wire layout: little-endian fields at fixed offsets.
  TEST_ASSERT_EQUAL_HEX8(0x05, payload[0]);
  TEST_ASSERT_EQUAL_HEX8(0xB9, payload[2]);  // -71
  TEST_ASSERT_EQUAL_HEX8(0x34, payload[4]);
  TEST_ASSERT_EQUAL_HEX8(0x12, payload[5]);
  TEST_ASSERT_EQUAL_HEX8(0x18, payload[8]);  // -1000 = 0xFC18
  TEST_ASSERT_EQUAL_HEX8(0xFC, payload[9]);

  elrsb::State out;
  TEST_ASSERT_TRUE(elrsb::unpackState(payload, n, out));
  TEST_ASSERT_TRUE(out.linkUp());
  TEST_ASSERT_TRUE(out.armed());
  TEST_ASSERT_FALSE(out.failsafe());
  TEST_ASSERT_EQUAL_UINT8(99, out.lq);
  TEST_ASSERT_EQUAL_INT8(-71, out.rssi);
  TEST_ASSERT_EQUAL_INT8(-4, out.snr);
  TEST_ASSERT_EQUAL_UINT16(0x1234, out.ageMs);
  TEST_ASSERT_EQUAL_UINT8(200, out.seq);
  TEST_ASSERT_EQUAL_INT16_ARRAY(in.ch, out.ch, 16);

  TEST_ASSERT_FALSE(elrsb::unpackState(payload, n - 1, out));  // truncated
  in.count = 4;
  const uint8_t m = elrsb::packState(payload, in);
  TEST_ASSERT_EQUAL_UINT8(8 + 8, m);
  TEST_ASSERT_TRUE(elrsb::unpackState(payload, m, out));
  TEST_ASSERT_EQUAL_UINT8(4, out.count);
  TEST_ASSERT_EQUAL_INT16(0, out.ch[4]);  // channels not sent read as zero
}

void test_telemetry_message_pack_unpack() {
  uint8_t p[elrsb::kMaxPayload];

  elrsb::Battery b = {12.5f, -3.25f, 4321.0f, 77};
  elrsb::Battery b2;
  TEST_ASSERT_EQUAL_UINT8(13, elrsb::packBattery(p, b));
  // 12.5f is 0x41480000, little-endian on the wire.
  TEST_ASSERT_EQUAL_HEX8(0x00, p[0]);
  TEST_ASSERT_EQUAL_HEX8(0x48, p[2]);
  TEST_ASSERT_EQUAL_HEX8(0x41, p[3]);
  TEST_ASSERT_TRUE(elrsb::unpackBattery(p, 13, b2));
  TEST_ASSERT_EQUAL_FLOAT(12.5f, b2.volts);
  TEST_ASSERT_EQUAL_FLOAT(-3.25f, b2.amps);
  TEST_ASSERT_EQUAL_FLOAT(4321.0f, b2.usedMah);
  TEST_ASSERT_EQUAL_UINT8(77, b2.remainingPct);
  TEST_ASSERT_FALSE(elrsb::unpackBattery(p, 12, b2));

  elrsb::Gps g = {377749000, -1224194000, 1.5f, 90.0f, 30.0f, 12};
  elrsb::Gps g2;
  TEST_ASSERT_EQUAL_UINT8(21, elrsb::packGps(p, g));
  TEST_ASSERT_TRUE(elrsb::unpackGps(p, 21, g2));
  TEST_ASSERT_EQUAL_INT32(377749000, g2.latE7);
  TEST_ASSERT_EQUAL_INT32(-1224194000, g2.lonE7);
  TEST_ASSERT_EQUAL_FLOAT(90.0f, g2.headingDeg);
  TEST_ASSERT_EQUAL_UINT8(12, g2.sats);

  elrsb::Rpm r;
  r.source = 3;
  r.count = 2;
  r.rpm[0] = 123456;
  r.rpm[1] = -42;
  elrsb::Rpm r2;
  TEST_ASSERT_EQUAL_UINT8(10, elrsb::packRpm(p, r));
  TEST_ASSERT_TRUE(elrsb::unpackRpm(p, 10, r2));
  TEST_ASSERT_EQUAL_UINT8(3, r2.source);
  TEST_ASSERT_EQUAL_INT32(123456, r2.rpm[0]);
  TEST_ASSERT_EQUAL_INT32(-42, r2.rpm[1]);
  TEST_ASSERT_FALSE(elrsb::unpackRpm(p, 9, r2));
  p[1] = 0;  // a count of zero is not a valid message
  TEST_ASSERT_FALSE(elrsb::unpackRpm(p, 10, r2));
  p[1] = 9;  // nor is more than the maximum
  TEST_ASSERT_FALSE(elrsb::unpackRpm(p, 64, r2));

  elrsb::Link l;
  memset(&l, 0, sizeof l);
  l.upRssi1 = -55;
  l.txPowerMw = 250;
  l.uptimeMs = 0xDEADBEEF;
  l.rcFrames = 123456789;
  l.txDropped = 7;
  elrsb::Link l2;
  TEST_ASSERT_EQUAL_UINT8(32, elrsb::packLink(p, l));
  TEST_ASSERT_TRUE(elrsb::unpackLink(p, 32, l2));
  TEST_ASSERT_EQUAL_INT8(-55, l2.upRssi1);
  TEST_ASSERT_EQUAL_UINT16(250, l2.txPowerMw);
  TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, l2.uptimeMs);
  TEST_ASSERT_EQUAL_UINT32(123456789, l2.rcFrames);
  TEST_ASSERT_EQUAL_UINT16(7, l2.txDropped);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_matches_ccitt_false_check_value);

  RUN_TEST(test_host_frame_round_trip);
  RUN_TEST(test_host_parser_separates_text_from_frames);
  RUN_TEST(test_host_parser_rejects_corruption);
  RUN_TEST(test_state_pack_unpack);
  RUN_TEST(test_telemetry_message_pack_unpack);
  return UNITY_END();
}
