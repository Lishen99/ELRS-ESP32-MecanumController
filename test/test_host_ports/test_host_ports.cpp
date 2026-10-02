// Tests for a host port that cannot keep up: too slow for the stream, busy
// with a long reply, or not being read at all. The host must always be looking
// at the present: old updates may be skipped, but never delivered late.
//
//   pio test -e native -f test_host_ports
#include <unity.h>

#include <algorithm>
#include <cstdio>
#include <string>

#include "bridge_rig.h"
#include "eb_bridge.h"

namespace {

// ---------------------------------------------------------------------------
// A port nobody reads

void test_bridge_survives_an_unread_port() {
  // Nobody reads the host port. Updates are replaced, not piled up; the
  // replaced ones are counted; and the bridge carries on.
  Fixture fx;
  Sticks s;
  s.ticks[0] = 1811;
  run(fx.b(), fx.now, 10000, &s, false);
  s.ticks[0] = 172;  // the stick goes to the other end while nobody is looking
  run(fx.b(), fx.now, 10000, &s, false);
  TEST_ASSERT_TRUE(fx.b().linkUp());
  // At most one update is ever waiting.
  TEST_ASSERT_TRUE(fx.b().portTxPending(eb::Bridge::kPortHost) <= eb::kStreamChunkMax);

  // When someone finally listens, the first thing they get is current.
  const Capture first = drain(fx.b(), eb::Bridge::kPortHost);
  TEST_ASSERT_EQUAL_INT(1, first.count(elrsb::kMsgState));
  const elrsb::State st = lastState(first);
  TEST_ASSERT_TRUE(st.linkUp());
  TEST_ASSERT_TRUE(st.ageMs < 20);
  TEST_ASSERT_EQUAL_INT16(-1000, st.ch[0]);

  const uint8_t want = elrsb::kMsgLink;
  sendHostFrame(fx.b(), eb::Bridge::kPortHost, elrsb::kMsgRequest, &want, 1, fx.now);
  const Capture cap = drain(fx.b(), eb::Bridge::kPortHost);
  elrsb::Link link;
  const HostFrame* f = cap.last(elrsb::kMsgLink);
  TEST_ASSERT_NOT_NULL(f);
  TEST_ASSERT_TRUE(elrsb::unpackLink(f->payload.data(), f->payload.size(), link));
  TEST_ASSERT_TRUE(link.txDropped > 900);  // 50 a second for 20 s, all but one replaced
}

// ---------------------------------------------------------------------------
// Ports slower than the stream

// Milliseconds from now until `wire` delivers a state update for which
// `matches` is true, or -1 if none arrives within `limitMs`.
template <typename Matches>
int msUntilState(Fixture& fx, SlowWire& wire, const Sticks* sticks, int limitMs, Matches matches) {
  for (int ms = 1; ms <= limitMs; ++ms) {
    const size_t seen = wire.tap.cap.frames.size();
    ++fx.now;
    if (sticks && fx.now % 10 == 0) sendRc(fx.b(), *sticks, fx.now);
    fx.b().tick(fx.now);
    wire.step();
    for (size_t i = seen; i < wire.tap.cap.frames.size(); ++i) {
      const HostFrame& f = wire.tap.cap.frames[i];
      elrsb::State st;
      if (f.type == elrsb::kMsgState &&
          elrsb::unpackState(f.payload.data(), f.payload.size(), st) && matches(st)) {
        return ms;
      }
    }
  }
  return -1;
}

void runOnWire(Fixture& fx, SlowWire& wire, uint32_t ms, const Sticks* sticks) {
  for (uint32_t i = 0; i < ms; ++i) {
    ++fx.now;
    if (sticks && fx.now % 10 == 0) sendRc(fx.b(), *sticks, fx.now);
    fx.b().tick(fx.now);
    wire.step();
  }
}

bool isDisarmed(const elrsb::State& st) { return !st.armed(); }

void test_stream_stays_current_on_a_slow_port() {
  // 9600 baud carries under a thousand bytes a second: less than half of the
  // default 50 updates. The host must still learn of a disarm at once, not
  // after the several seconds a queue of old updates would take to drain.
  Fixture fx;
  SlowWire wire(fx.b(), eb::Bridge::kPortHost, 9600);
  Sticks s;
  s.ticks[4] = 191;
  runOnWire(fx, wire, kArmOffHold, &s);
  s.ticks[4] = 1792;
  runOnWire(fx, wire, 3000, &s);
  TEST_ASSERT_TRUE(fx.b().armed());
  TEST_ASSERT_EQUAL_UINT32(0, wire.tap.parser.errors());
  TEST_ASSERT_TRUE(lastState(wire.tap.cap).armed());

  s.ticks[4] = 191;
  const int delay = msUntilState(fx, wire, &s, 5000, isDisarmed);
  // What is left is the wire itself: the update being sent, the UART's
  // 128-byte FIFO emptying at 0.96 bytes per millisecond, then the new update.
  TEST_ASSERT_GREATER_THAN_INT_MESSAGE(0, delay, "the disarm never reached the host");
  TEST_ASSERT_LESS_OR_EQUAL_INT_MESSAGE(350, delay, "the disarm reached the host too late");
  TEST_ASSERT_EQUAL_UINT32(0, wire.tap.parser.errors());
}

void test_stream_stays_current_when_the_rate_exceeds_the_port() {
  // 500 updates a second is twice what 115200 baud can carry.
  Fixture fx;
  cli(fx.b(), "set stream_hz 500", fx.now);
  SlowWire wire(fx.b(), eb::Bridge::kPortHost, 115200);
  Sticks s;
  s.ticks[4] = 191;
  runOnWire(fx, wire, kArmOffHold, &s);
  s.ticks[4] = 1792;
  runOnWire(fx, wire, 3000, &s);
  TEST_ASSERT_TRUE(lastState(wire.tap.cap).armed());

  s.ticks[4] = 191;
  const int delay = msUntilState(fx, wire, &s, 2000, isDisarmed);
  TEST_ASSERT_GREATER_THAN_INT_MESSAGE(0, delay, "the disarm never reached the host");
  TEST_ASSERT_LESS_OR_EQUAL_INT_MESSAGE(40, delay, "the disarm reached the host too late");
  TEST_ASSERT_EQUAL_UINT32(0, wire.tap.parser.errors());

  // The line is saturated, yet a command still gets its turn and its answer.
  wire.tap.cap = Capture();
  sendCli(fx.b(), eb::Bridge::kPortHost, "dump", 77, fx.now);
  runOnWire(fx, wire, 3000, &s);
  const std::string dump = cliReplyText(wire.tap.cap, 77);
  TEST_ASSERT_EQUAL_UINT(settingCount(), (size_t)std::count(dump.begin(), dump.end(), '\n'));
  TEST_ASSERT_EQUAL_UINT32(0, wire.tap.parser.errors());
}

void test_cli_output_does_not_hold_up_the_stream() {
  // A full settings dump is about 5 kB: nearly half a second of line time at
  // 115200 baud. Updates must keep flowing through it.
  Fixture fx;
  SlowWire wire(fx.b(), eb::Bridge::kPortHost, 115200);
  Sticks s;
  runOnWire(fx, wire, 300, &s);
  wire.tap.cap = Capture();
  sendCli(fx.b(), eb::Bridge::kPortHost, "dump", 42, fx.now);

  int longestGap = 0;
  int gap = 0;
  size_t states = 0;
  for (int ms = 0; ms < 1500; ++ms) {
    const size_t before = (size_t)wire.tap.cap.count(elrsb::kMsgState);
    runOnWire(fx, wire, 1, &s);
    const size_t after = (size_t)wire.tap.cap.count(elrsb::kMsgState);
    gap = after > before ? 0 : gap + 1;
    if (gap > longestGap) longestGap = gap;
    states = after;
  }
  TEST_ASSERT_LESS_OR_EQUAL_INT_MESSAGE(45, longestGap, "the stream stalled behind the dump");
  TEST_ASSERT_GREATER_THAN_UINT(60, states);

  const std::string dump = cliReplyText(wire.tap.cap, 42);
  TEST_ASSERT_EQUAL_UINT(settingCount(), (size_t)std::count(dump.begin(), dump.end(), '\n'));
  TEST_ASSERT_TRUE(contains(dump, "set aux4.fs_us 0\n"));
  TEST_ASSERT_EQUAL_UINT32(0, wire.tap.parser.errors());
}

void test_text_stream_never_splits_a_line() {
  // In text mode the host reads lines. A $RC line must never land in the
  // middle of a line of command output, however the two are interleaved.
  Fixture fx;
  SlowWire wire(fx.b(), eb::Bridge::kPortUsb, 115200);
  Sticks s;
  s.ticks[0] = 1811;
  typeLine(fx.b(), eb::Bridge::kPortUsb, "stream text 100", fx.now);
  runOnWire(fx, wire, 300, &s);
  // Start looking at a line boundary, not part-way through a $RC line.
  while (wire.tap.cap.text.empty() || wire.tap.cap.text[wire.tap.cap.text.size() - 1] != '\n') {
    runOnWire(fx, wire, 1, &s);
  }
  wire.tap.cap = Capture();
  typeLine(fx.b(), eb::Bridge::kPortUsb, "get", fx.now);
  runOnWire(fx, wire, 2000, &s);

  int streamLines = 0;
  int settingLines = 0;
  const std::string& text = wire.tap.cap.text;
  size_t start = 0;
  size_t end;
  while ((end = text.find("\r\n", start)) != std::string::npos) {
    const std::string line = text.substr(start, end - start);
    start = end + 2;
    if (line.compare(0, 4, "$RC,") == 0) {
      const size_t star = line.find('*');
      TEST_ASSERT_TRUE_MESSAGE(star != std::string::npos, line.c_str());
      uint8_t sum = 0;
      for (size_t i = 1; i < star; ++i) sum ^= (uint8_t)line[i];
      char expect[3];
      snprintf(expect, sizeof expect, "%02X", sum);
      TEST_ASSERT_EQUAL_STRING_MESSAGE(expect, line.substr(star + 1).c_str(), line.c_str());
      ++streamLines;
    } else {
      // Command output: a whole "name = value" line with no stream text in it.
      TEST_ASSERT_TRUE_MESSAGE(line.find('$') == std::string::npos, line.c_str());
      TEST_ASSERT_TRUE_MESSAGE(line.find(" = ") != std::string::npos, line.c_str());
      ++settingLines;
    }
  }
  TEST_ASSERT_EQUAL_INT(settingCount(), settingLines);
  // 100 a second for two seconds; the line has room for both.
  TEST_ASSERT_GREATER_THAN_INT(190, streamLines);
}

void test_replies_take_turns_with_a_saturating_stream() {
  // A line with room for exactly one update per period, and an update always
  // ready to fill it. A reply must still get through: the two take turns.
  Fixture fx;
  cli(fx.b(), "set link_hz 0", fx.now);  // every update the same size
  Sticks s;
  run(fx.b(), fx.now, 100, &s);
  drain(fx.b(), eb::Bridge::kPortHost);
  run(fx.b(), fx.now, 20, &s, false);  // one update is now waiting
  const size_t updateSize = fx.b().portTxPending(eb::Bridge::kPortHost);
  TEST_ASSERT_EQUAL_UINT(elrsb::kFrameOverhead + elrsb::kStateHeaderSize + 2 * 16, updateSize);

  sendCli(fx.b(), eb::Bridge::kPortHost, "version", 31, fx.now);
  Tap tap;
  uint8_t buf[256];
  int periods = 0;
  while (tap.cap.count(elrsb::kMsgCliReply) == 0 && periods < 50) {
    tap.take(buf, fx.b().readPortTx(eb::Bridge::kPortHost, buf, updateSize));
    run(fx.b(), fx.now, 20, &s, false);  // by the next slot an update is ready again
    ++periods;
  }
  // The reply is 48 bytes, so it needs two slots of the line.
  TEST_ASSERT_LESS_OR_EQUAL_INT(4, periods);
  TEST_ASSERT_TRUE(contains(cliReplyText(tap.cap, 31), "elrs-bridge"));

  // And the updates carry on around it, intact.
  for (int i = 0; i < 5; ++i) {
    tap.take(buf, fx.b().readPortTx(eb::Bridge::kPortHost, buf, updateSize));
    run(fx.b(), fx.now, 20, &s, false);
  }
  TEST_ASSERT_TRUE(tap.cap.count(elrsb::kMsgState) >= 4);
  TEST_ASSERT_EQUAL_UINT32(0, tap.parser.errors());
}

void test_stream_mode_change_discards_an_update_in_the_old_format() {
  // A binary update is waiting when the port is switched to text: the host
  // asked for text, so it must not be handed a binary frame first.
  Fixture fx;
  Sticks s;
  run(fx.b(), fx.now, 100, &s);
  drain(fx.b(), eb::Bridge::kPortHost);
  run(fx.b(), fx.now, 25, &s, false);
  TEST_ASSERT_TRUE(fx.b().portTxPending(eb::Bridge::kPortHost) > 0);

  typeLine(fx.b(), eb::Bridge::kPortHost, "stream text", fx.now);
  Capture cap = drain(fx.b(), eb::Bridge::kPortHost);
  TEST_ASSERT_EQUAL_UINT(0, cap.frames.size());
  TEST_ASSERT_EQUAL_STRING("ok\r\n", cap.text.c_str());
  run(fx.b(), fx.now, 100, &s);
  cap = drain(fx.b(), eb::Bridge::kPortHost);
  TEST_ASSERT_EQUAL_UINT(0, cap.frames.size());
  TEST_ASSERT_TRUE_MESSAGE(cap.text.compare(0, 4, "$RC,") == 0, cap.text.c_str());

  // And a text line waiting when the stream is switched off is not sent at all.
  run(fx.b(), fx.now, 25, &s, false);
  TEST_ASSERT_TRUE(fx.b().portTxPending(eb::Bridge::kPortHost) > 0);
  typeLine(fx.b(), eb::Bridge::kPortHost, "stream off", fx.now);
  cap = drain(fx.b(), eb::Bridge::kPortHost);
  TEST_ASSERT_EQUAL_STRING("ok\r\n> ", cap.text.c_str());
}

void test_text_stream_starts_after_an_unfinished_prompt_line() {
  // The interactive prompt "> " leaves the port part-way through a line. A
  // host that then asks for the text stream must still get whole lines.
  Fixture fx;
  run(fx.b(), fx.now, 20);
  const Capture banner = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_TRUE(contains(banner.text, "> "));

  elrsb::SetStream s;
  s.mode = elrsb::kStreamText;
  s.hz = 50;
  s.options = 0;
  uint8_t p[8];
  sendHostFrame(fx.b(), eb::Bridge::kPortUsb, elrsb::kMsgSetStream, p, elrsb::packSetStream(p, s),
                fx.now);
  run(fx.b(), fx.now, 200);
  const Capture cap = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_TRUE_MESSAGE(cap.text.compare(0, 6, "\r\n$RC,") == 0, cap.text.c_str());
  TEST_ASSERT_EQUAL_INT(10, (int)std::count(cap.text.begin(), cap.text.end(), '$'));
  TEST_ASSERT_EQUAL_UINT(cap.text.size() - 2, cap.text.rfind("\r\n"));
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_bridge_survives_an_unread_port);

  RUN_TEST(test_stream_stays_current_on_a_slow_port);
  RUN_TEST(test_stream_stays_current_when_the_rate_exceeds_the_port);
  RUN_TEST(test_cli_output_does_not_hold_up_the_stream);
  RUN_TEST(test_text_stream_never_splits_a_line);
  RUN_TEST(test_replies_take_turns_with_a_saturating_stream);
  RUN_TEST(test_stream_mode_change_discards_an_update_in_the_old_format);
  RUN_TEST(test_text_stream_starts_after_an_unfinished_prompt_line);
  return UNITY_END();
}
