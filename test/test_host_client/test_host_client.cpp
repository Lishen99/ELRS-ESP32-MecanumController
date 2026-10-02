// Tests for the C++ host client (host/cpp/ElrsBridge), run against the real
// bridge core in the same process:
//
//   pio test -e native -f test_host_client
#include <unity.h>

#include <cstring>
#include <deque>
#include <memory>
#include <vector>

// Compile the Arduino serial wrapper too, against the stub Arduino.h that
// sits next to this file.
#define ARDUINO 100
#include <ElrsBridgeClient.h>

#include "eb_bridge.h"
#include "fake_platform.h"
#include "sticks.h"

namespace {
uint32_t fakeMillis = 0;
}

unsigned long millis() { return fakeMillis; }

namespace {

// The wire between a host board and one of the bridge's ports.
class FakeSerial {
 public:
  FakeSerial(eb::Bridge& bridge, eb::Bridge::PortId port) : bridge_(bridge), port_(port) {}

  int available() {
    uint8_t buf[64];
    size_t n;
    while ((n = bridge_.readPortTx(port_, buf, sizeof buf)) > 0) {
      rx_.insert(rx_.end(), buf, buf + n);
    }
    return (int)rx_.size();
  }
  int read() {
    if (rx_.empty()) return -1;
    const int b = rx_.front();
    rx_.pop_front();
    return b;
  }
  size_t write(const uint8_t* data, size_t len) {
    bridge_.feedPort(port_, data, len, fakeMillis);
    return len;
  }

 private:
  eb::Bridge& bridge_;
  eb::Bridge::PortId port_;
  std::deque<uint8_t> rx_;
};

struct Rig {
  FakePlatform platform;
  std::unique_ptr<eb::Bridge> bridge;
  FakeSerial hostWire;
  ElrsBridgeSerial<FakeSerial> client;
  uint16_t ticks[16];

  Rig()
      : bridge(new eb::Bridge(platform)),
        hostWire(*bridge, eb::Bridge::kPortHost),
        client(hostWire) {
    fakeMillis = 1000;
    bridge->begin(fakeMillis);
    for (int i = 0; i < 16; ++i) ticks[i] = i < 4 ? 992 : 191;
  }

  // Advance time; the handset sends at 100 Hz while `linked`, and the host
  // board polls its serial port every millisecond while `listening`.
  void run(uint32_t ms, bool linked = true, bool listening = true) {
    for (uint32_t i = 0; i < ms; ++i) {
      ++fakeMillis;
      if (linked && fakeMillis % 10 == 0) {
        uint8_t payload[22];
        eb::crsf::packChannels(ticks, payload);
        uint8_t frame[64];
        const size_t n = eb::crsf::buildFrame(frame, eb::crsf::kTypeRcChannels, payload, 22);
        bridge->feedCrsf(frame, n, fakeMillis);
      }
      bridge->tick(fakeMillis);
      if (listening) client.update();
    }
  }

  // CRSF frames the bridge queued for the receiver: type byte then payload.
  std::vector<std::vector<uint8_t> > crsfOut() {
    std::vector<std::vector<uint8_t> > frames;
    eb::crsf::Parser parser;
    uint8_t buf[128];
    size_t n;
    while ((n = bridge->readCrsfTx(buf, sizeof buf)) > 0) {
      for (size_t i = 0; i < n; ++i) {
        if (!parser.feed(buf[i])) continue;
        std::vector<uint8_t> f;
        f.push_back(parser.frame().type);
        f.insert(f.end(), parser.frame().payload, parser.frame().payload + parser.frame().len);
        frames.push_back(f);
      }
    }
    return frames;
  }
};

void expectCrsf(const std::vector<std::vector<uint8_t> >& frames, const uint8_t* expected,
                size_t len) {
  for (size_t i = 0; i < frames.size(); ++i) {
    if (frames[i][0] != expected[0]) continue;
    TEST_ASSERT_EQUAL_UINT(len, frames[i].size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frames[i].data(), len);
    return;
  }
  TEST_FAIL_MESSAGE("expected CRSF frame type was not sent");
}

void test_client_reports_failsafe_until_data_arrives() {
  Rig rig;
  TEST_ASSERT_FALSE(rig.client.fresh());
  TEST_ASSERT_FALSE(rig.client.armed());
  TEST_ASSERT_FALSE(rig.client.linkUp());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.client.ch(1));
  const elrsb::State s = rig.client.state();
  TEST_ASSERT_TRUE(s.failsafe());
  TEST_ASSERT_FALSE(s.armed());
  TEST_ASSERT_NULL(rig.client.link());
}

void test_client_decodes_the_stream() {
  Rig rig;
  rig.ticks[0] = 1811;
  rig.ticks[1] = 172;
  rig.ticks[5] = 992;
  rig.run(kArmOffHold);
  rig.ticks[4] = 1792;  // arm, after the switch was seen off for a moment
  rig.run(100);

  TEST_ASSERT_TRUE(rig.client.fresh());
  TEST_ASSERT_TRUE(rig.client.linkUp());
  TEST_ASSERT_TRUE(rig.client.armed());
  TEST_ASSERT_EQUAL_INT16(1000, rig.client.value(1));
  TEST_ASSERT_EQUAL_INT16(-1000, rig.client.value(2));
  TEST_ASSERT_EQUAL_INT16(0, rig.client.value(3));
  TEST_ASSERT_EQUAL_INT16(1000, rig.client.value(5));
  TEST_ASSERT_EQUAL_INT16(500, rig.client.value(6));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, rig.client.ch(1));
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, rig.client.ch(2));
  TEST_ASSERT_EQUAL_FLOAT(0.5f, rig.client.ch(6));
  // Channel numbers outside 1..16 read as zero rather than out of bounds.
  TEST_ASSERT_EQUAL_INT16(0, rig.client.value(0));
  TEST_ASSERT_EQUAL_INT16(0, rig.client.value(17));

  const elrsb::State s = rig.client.state();
  TEST_ASSERT_TRUE(s.linkUp());
  TEST_ASSERT_FALSE(s.failsafe());
  TEST_ASSERT_EQUAL_UINT8(16, s.count);
  TEST_ASSERT_TRUE(s.ageMs < 20);
  TEST_ASSERT_UINT32_WITHIN(1, 35, rig.client.stateCount());  // 50 Hz for 700 ms
  TEST_ASSERT_EQUAL_UINT32(0, rig.client.errors());

  rig.run(1500);
  const elrsb::Link* link = rig.client.link();
  TEST_ASSERT_NOT_NULL(link);
  TEST_ASSERT_UINT16_WITHIN(2, 100, link->rcRateHz);
  TEST_ASSERT_TRUE((link->flags & elrsb::kFlagArmed) != 0);
}

void test_client_follows_the_bridge_into_failsafe() {
  Rig rig;
  rig.ticks[0] = 1811;
  rig.run(kArmOffHold);
  rig.ticks[4] = 1792;
  rig.run(100);
  TEST_ASSERT_TRUE(rig.client.armed());

  rig.run(600, false);  // handset off; the bridge keeps streaming
  TEST_ASSERT_TRUE(rig.client.fresh());
  TEST_ASSERT_FALSE(rig.client.linkUp());
  TEST_ASSERT_FALSE(rig.client.armed());
  TEST_ASSERT_EQUAL_INT16(0, rig.client.value(1));
  TEST_ASSERT_TRUE(rig.client.state().failsafe());
}

void test_client_goes_failsafe_when_the_bridge_goes_quiet() {
  Rig rig;
  rig.ticks[0] = 1811;
  rig.run(kArmOffHold);
  rig.ticks[4] = 1792;
  rig.run(100);
  TEST_ASSERT_TRUE(rig.client.armed());
  TEST_ASSERT_EQUAL_INT16(1000, rig.client.value(1));

  // The cable comes off: nothing more reaches the client. The last update is
  // at most one 20 ms stream period old when that happens.
  rig.run(200, true, false);
  TEST_ASSERT_TRUE(rig.client.armed());  // still inside the 250 ms allowance
  rig.run(80, true, false);
  TEST_ASSERT_FALSE(rig.client.fresh());
  TEST_ASSERT_FALSE(rig.client.armed());
  TEST_ASSERT_FALSE(rig.client.linkUp());
  TEST_ASSERT_EQUAL_INT16(0, rig.client.value(1));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.client.ch(1));
  const elrsb::State s = rig.client.state();
  TEST_ASSERT_TRUE(s.failsafe());
  TEST_ASSERT_FALSE(s.armed());
  TEST_ASSERT_EQUAL_INT16(0, s.ch[0]);

  // Plugged back in, it recovers.
  rig.run(50);
  TEST_ASSERT_TRUE(rig.client.armed());
  TEST_ASSERT_EQUAL_INT16(1000, rig.client.value(1));
}

void test_client_is_never_handed_old_states() {
  // The host stops reading for a while (a long computation, a blocked loop)
  // while the operator lets go of the stick and disarms. The first thing it
  // reads afterwards must be the present, not a replay of what it missed.
  Rig rig;
  rig.ticks[0] = 1811;
  rig.run(kArmOffHold);
  rig.ticks[4] = 1792;
  rig.run(100);
  TEST_ASSERT_TRUE(rig.client.armed());
  const uint32_t before = rig.client.stateCount();

  rig.run(1000, true, false);
  rig.ticks[0] = 992;
  rig.ticks[4] = 191;
  rig.run(1000, true, false);

  rig.run(1);  // one poll of the serial port
  TEST_ASSERT_EQUAL_UINT32(before + 1, rig.client.stateCount());
  TEST_ASSERT_TRUE(rig.client.fresh());
  TEST_ASSERT_TRUE(rig.client.linkUp());
  TEST_ASSERT_FALSE(rig.client.armed());
  TEST_ASSERT_EQUAL_INT16(0, rig.client.value(1));
  TEST_ASSERT_TRUE(rig.client.state().ageMs < 20);
}

void test_client_staleness_survives_clock_wraparound() {
  // Staleness is latched: when the millisecond clock wraps back round to the
  // time of the last update, the old data must not come back to life.
  ElrsBridgeClient client;
  elrsb::State s;
  memset(&s, 0, sizeof s);
  s.flags = elrsb::kFlagLinkUp | elrsb::kFlagArmed;
  s.count = 16;
  s.ch[0] = 1000;
  uint8_t payload[elrsb::kMaxPayload];
  uint8_t frame[elrsb::kMaxFrame];
  const size_t n = elrsb::encodeFrame(frame, elrsb::kMsgState, payload, elrsb::packState(payload, s));

  const uint32_t t0 = 0xFFFFFF80u;
  for (size_t i = 0; i < n; ++i) client.feed(frame[i], t0);
  TEST_ASSERT_TRUE(client.armed(t0));
  TEST_ASSERT_TRUE(client.armed(t0 + 200));   // across the wrap, still fresh
  TEST_ASSERT_EQUAL_INT16(1000, client.value(1, t0 + 200));
  TEST_ASSERT_FALSE(client.armed(t0 + 300));
  // A full trip round the counter later the timestamps look 5 ms apart.
  TEST_ASSERT_FALSE(client.fresh(t0 + 5));
  TEST_ASSERT_FALSE(client.armed(t0 + 5));
  TEST_ASSERT_EQUAL_INT16(0, client.value(1, t0 + 5));
  TEST_ASSERT_TRUE(client.state(t0 + 5).failsafe());
}

void test_client_stale_window_is_adjustable() {
  Rig rig;
  rig.client.setStaleAfter(50);
  rig.run(100);
  TEST_ASSERT_TRUE(rig.client.fresh());
  // The last update is up to 20 ms old when the client stops listening.
  rig.run(25, true, false);
  TEST_ASSERT_TRUE(rig.client.fresh());
  rig.run(40, true, false);
  TEST_ASSERT_FALSE(rig.client.fresh());
}

void test_client_ignores_text_and_corruption() {
  ElrsBridgeClient client;
  // CLI text and boot messages share the port with frames.
  const char text[] = "\r\nelrs-bridge 0.1.0 (esp32) - type 'help' for commands\r\n> ";
  for (size_t i = 0; i < sizeof text - 1; ++i) TEST_ASSERT_FALSE(client.feed((uint8_t)text[i], 1));
  TEST_ASSERT_FALSE(client.fresh(1));

  elrsb::State s;
  memset(&s, 0, sizeof s);
  s.flags = elrsb::kFlagLinkUp | elrsb::kFlagArmed;
  s.count = 16;
  s.ch[0] = 777;
  uint8_t payload[elrsb::kMaxPayload];
  uint8_t frame[elrsb::kMaxFrame];
  const size_t n = elrsb::encodeFrame(frame, elrsb::kMsgState, payload, elrsb::packState(payload, s));

  // A frame with one flipped bit is dropped and counted...
  frame[12] ^= 0x10;
  for (size_t i = 0; i < n; ++i) TEST_ASSERT_FALSE(client.feed(frame[i], 2));
  TEST_ASSERT_FALSE(client.armed(2));
  TEST_ASSERT_EQUAL_UINT32(1, client.errors());
  frame[12] ^= 0x10;

  // ...and the intact one right behind it is accepted.
  bool got = false;
  for (size_t i = 0; i < n; ++i) got = client.feed(frame[i], 3);
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_TRUE(client.armed(3));
  TEST_ASSERT_EQUAL_INT16(777, client.value(1, 3));
}

void test_client_telemetry_frames() {
  Rig rig;
  rig.run(100);
  rig.crsfOut();

  rig.client.sendBattery(12.6f, 1.5f, 1234.0f, 80);
  rig.client.sendGps(377749000, -1224194000, 2.5f, 270.5f, 12.0f, 9);
  rig.client.sendAttitude(0.1f, -0.2f, 3.0f);
  rig.client.sendMode("AUTO");
  rig.client.sendVario(-0.5f);
  rig.client.sendBaro(123.4f, 0.5f);
  rig.client.sendAirspeed(10.0f);
  const int32_t rpm[2] = {1000, -200};
  rig.client.sendRpm(rpm, 2);
  const float degC[2] = {25.5f, -10.0f};
  rig.client.sendTemp(degC, 2, 1);
  rig.run(1000);

  // The same reference bytes as the firmware's own tests.
  const std::vector<std::vector<uint8_t> > frames = rig.crsfOut();
  static const uint8_t kBattery[] = {0x08, 0x00, 0x7E, 0x00, 0x0F, 0x00, 0x04, 0xD2, 0x50};
  static const uint8_t kGps[] = {0x02, 0x16, 0x83, 0xFE, 0x08, 0xB7, 0x08, 0x48, 0x30,
                                 0x00, 0x5A, 0x69, 0xAA, 0x03, 0xF4, 0x09};
  static const uint8_t kAtt[] = {0x1E, 0xF8, 0x30, 0x03, 0xE8, 0x75, 0x30};
  static const uint8_t kMode[] = {0x21, 'A', 'U', 'T', 'O', 0x00};
  static const uint8_t kVario[] = {0x07, 0xFF, 0xCE};
  static const uint8_t kBaro[] = {0x09, 0x2B, 0xE2, 0x00, 0x32};
  static const uint8_t kAspd[] = {0x0A, 0x01, 0x68};
  static const uint8_t kRpm[] = {0x0C, 0x00, 0x00, 0x03, 0xE8, 0xFF, 0xFF, 0x38};
  static const uint8_t kTemp[] = {0x0D, 0x01, 0x00, 0xFF, 0xFF, 0x9C};
  expectCrsf(frames, kBattery, sizeof kBattery);
  expectCrsf(frames, kGps, sizeof kGps);
  expectCrsf(frames, kAtt, sizeof kAtt);
  expectCrsf(frames, kMode, sizeof kMode);
  expectCrsf(frames, kVario, sizeof kVario);
  expectCrsf(frames, kBaro, sizeof kBaro);
  expectCrsf(frames, kAspd, sizeof kAspd);
  expectCrsf(frames, kRpm, sizeof kRpm);
  expectCrsf(frames, kTemp, sizeof kTemp);

  // A raw CRSF frame goes straight through.
  static const uint8_t kCells[] = {0x00, 0x10, 0x68, 0x10, 0x04};
  rig.client.sendCrsf(0x0E, kCells, sizeof kCells);
  static const uint8_t kCellFrame[] = {0x0E, 0x00, 0x10, 0x68, 0x10, 0x04};
  expectCrsf(rig.crsfOut(), kCellFrame, sizeof kCellFrame);

  // Oversized inputs are refused or clipped rather than overrunning buffers.
  uint8_t out[elrsb::kMaxFrame];
  uint8_t big[100] = {0};
  TEST_ASSERT_EQUAL_UINT(0, ElrsBridgeClient::packCrsf(out, 0x0E, big, 61));
  TEST_ASSERT_EQUAL_UINT(1 + 60 + 6, ElrsBridgeClient::packCrsf(out, 0x0E, big, 60));
  TEST_ASSERT_EQUAL_UINT(15 + 6, ElrsBridgeClient::packMode(out, "a very long mode string indeed"));
  int32_t many[20] = {0};
  TEST_ASSERT_EQUAL_UINT(2 + 4 * 8 + 6, ElrsBridgeClient::packRpm(out, many, 20));
}

void test_client_can_start_the_stream_on_the_usb_port() {
  Rig rig;
  FakeSerial usbWire(*rig.bridge, eb::Bridge::kPortUsb);
  ElrsBridgeSerial<FakeSerial> usb(usbWire);
  rig.run(100);
  usb.update();
  TEST_ASSERT_FALSE(usb.fresh());  // the USB port only offers the CLI until asked

  usb.setStream(elrsb::kStreamBinary, 100);
  rig.ticks[0] = 1811;
  for (int i = 0; i < 100; ++i) {
    rig.run(1);
    usb.update();
  }
  TEST_ASSERT_TRUE(usb.fresh());
  TEST_ASSERT_TRUE(usb.linkUp());
  TEST_ASSERT_EQUAL_INT16(1000, usb.value(1));
  TEST_ASSERT_UINT32_WITHIN(1, 10, usb.stateCount());  // 100 Hz for 100 ms
}

void test_client_keeps_the_usb_stream_alive() {
  // On the USB port the stream exists only while requested, and the bridge
  // forgets the request when it restarts. keepStream() asks again whenever
  // the updates dry up.
  Rig rig;
  FakeSerial usbWire(*rig.bridge, eb::Bridge::kPortUsb);
  ElrsBridgeSerial<FakeSerial> usb(usbWire);
  usb.keepStream(elrsb::kStreamBinary, 50);
  for (int i = 0; i < 100; ++i) {
    rig.run(1);
    usb.update();
  }
  TEST_ASSERT_TRUE(usb.fresh());

  // Simulate the restart: the stream is switched off behind the client's back.
  uint8_t off[elrsb::kMaxFrame];
  const size_t n = ElrsBridgeClient::packSetStream(off, elrsb::kStreamOff);
  rig.bridge->feedPort(eb::Bridge::kPortUsb, off, n, fakeMillis);
  for (int i = 0; i < 200; ++i) {
    rig.run(1);
    usb.update();
  }
  const uint32_t before = usb.stateCount();
  for (int i = 0; i < 1500; ++i) {
    rig.run(1);
    usb.update();
  }
  TEST_ASSERT_TRUE(usb.fresh());
  TEST_ASSERT_TRUE(usb.stateCount() > before + 20);

  // Once told to stop, the client no longer re-requests and just goes stale.
  usb.stopKeepingStream();
  rig.bridge->feedPort(eb::Bridge::kPortUsb, off, n, fakeMillis);
  for (int i = 0; i < 1500; ++i) {
    rig.run(1);
    usb.update();
  }
  TEST_ASSERT_FALSE(usb.fresh());
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_client_reports_failsafe_until_data_arrives);
  RUN_TEST(test_client_decodes_the_stream);
  RUN_TEST(test_client_follows_the_bridge_into_failsafe);
  RUN_TEST(test_client_goes_failsafe_when_the_bridge_goes_quiet);
  RUN_TEST(test_client_is_never_handed_old_states);
  RUN_TEST(test_client_staleness_survives_clock_wraparound);
  RUN_TEST(test_client_stale_window_is_adjustable);
  RUN_TEST(test_client_ignores_text_and_corruption);
  RUN_TEST(test_client_telemetry_frames);
  RUN_TEST(test_client_can_start_the_stream_on_the_usb_port);
  RUN_TEST(test_client_keeps_the_usb_stream_alive);
  return UNITY_END();
}
