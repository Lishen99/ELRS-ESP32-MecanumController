// Tests for the bridge as a whole: RC frames in on one side, and what the host
// ports, the receiver and the aux pins get out of it.
//
//   pio test -e native -f test_bridge
#include <unity.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "bridge_rig.h"
#include "eb_bridge.h"

namespace {

// ---------------------------------------------------------------------------
// Channels and link state to the host

void test_bridge_streams_failsafe_state_before_any_link() {
  Fixture fx;
  run(fx.b(), fx.now, 200);
  const Capture cap = drain(fx.b(), eb::Bridge::kPortHost);
  // 50 Hz for 200 ms.
  TEST_ASSERT_INT_WITHIN(1, 10, cap.count(elrsb::kMsgState));
  // link_hz 5 -> one every 200 ms.
  TEST_ASSERT_INT_WITHIN(1, 1, cap.count(elrsb::kMsgLink));
  const elrsb::State s = lastState(cap);
  TEST_ASSERT_FALSE(s.linkUp());
  TEST_ASSERT_TRUE(s.failsafe());
  TEST_ASSERT_FALSE(s.armed());
  TEST_ASSERT_EQUAL_UINT16(0xFFFF, s.ageMs);
  TEST_ASSERT_EQUAL_UINT8(16, s.count);
  for (int i = 0; i < 16; ++i) TEST_ASSERT_EQUAL_INT16(0, s.ch[i]);
}

void test_bridge_usb_port_starts_as_cli() {
  Fixture fx;
  run(fx.b(), fx.now, 100);
  const Capture cap = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_EQUAL_UINT(0, cap.frames.size());
  TEST_ASSERT_TRUE(contains(cap.text, "elrs-bridge"));
  TEST_ASSERT_TRUE(contains(cap.text, "> "));
}

void test_bridge_maps_channels_to_the_host() {
  Fixture fx;
  Sticks s;
  s.ticks[0] = 1811;                          // full right
  s.ticks[1] = 172;                           // full down
  s.ticks[2] = eb::crsf::usToTicks(1756);     // part way
  s.ticks[4] = 1792;                          // switch on (armed once toggled)
  s.ticks[5] = 992;                           // three-position switch in the middle
  Sticks low = s;
  low.ticks[4] = 191;
  run(fx.b(), fx.now, kArmOffHold, &low);
  run(fx.b(), fx.now, 100, &s);

  const Capture cap = drain(fx.b(), eb::Bridge::kPortHost);
  const elrsb::State st = lastState(cap);
  TEST_ASSERT_TRUE(st.linkUp());
  TEST_ASSERT_FALSE(st.failsafe());
  TEST_ASSERT_TRUE(st.armed());
  TEST_ASSERT_TRUE(st.ageMs < 20);
  TEST_ASSERT_EQUAL_INT16(1000, st.ch[0]);
  TEST_ASSERT_EQUAL_INT16(-1000, st.ch[1]);
  TEST_ASSERT_EQUAL_INT16(480, st.ch[2]);
  TEST_ASSERT_EQUAL_INT16(0, st.ch[3]);
  TEST_ASSERT_EQUAL_INT16(1000, st.ch[4]);
  TEST_ASSERT_EQUAL_INT16(500, st.ch[5]);
  TEST_ASSERT_EQUAL_INT16(0, st.ch[6]);

  // State frames carry a running sequence number.
  int prev = -1;
  for (size_t i = 0; i < cap.frames.size(); ++i) {
    if (cap.frames[i].type != elrsb::kMsgState) continue;
    const int seq = cap.frames[i].payload[6];
    if (prev >= 0) TEST_ASSERT_EQUAL_INT((prev + 1) & 0xFF, seq);
    prev = seq;
  }
}

void test_bridge_reports_failsafe_when_frames_stop() {
  Fixture fx;
  Sticks s;
  s.ticks[0] = 1811;
  // The receiver's last link report said all was well.
  static const uint8_t kStats[10] = {62, 0, 100, 9, 0, 7, 3, 0, 0, 0};
  uint8_t frame[64];
  const size_t n = eb::crsf::buildFrame(frame, eb::crsf::kTypeLinkStats, kStats, 10);
  fx.b().feedCrsf(frame, n, fx.now);
  run(fx.b(), fx.now, 100, &s);
  TEST_ASSERT_TRUE(fx.b().linkUp());
  TEST_ASSERT_EQUAL_UINT8(100, lastState(drain(fx.b(), eb::Bridge::kPortHost)).lq);

  run(fx.b(), fx.now, 490);
  TEST_ASSERT_TRUE(fx.b().linkUp());
  run(fx.b(), fx.now, 30);
  TEST_ASSERT_FALSE(fx.b().linkUp());
  const Capture cap = drain(fx.b(), eb::Bridge::kPortHost);
  const elrsb::State st = lastState(cap);
  TEST_ASSERT_TRUE(st.failsafe());
  TEST_ASSERT_FALSE(st.linkUp());
  TEST_ASSERT_EQUAL_INT16(0, st.ch[0]);
  // A stale "100 %" must not be passed on once the link is gone.
  TEST_ASSERT_EQUAL_UINT8(0, st.lq);
  TEST_ASSERT_EQUAL_INT8(-62, st.rssi);  // documented: RSSI keeps its last value

  const uint8_t want = elrsb::kMsgLink;
  sendHostFrame(fx.b(), eb::Bridge::kPortHost, elrsb::kMsgRequest, &want, 1, fx.now);
  const Capture linkCap = drain(fx.b(), eb::Bridge::kPortHost);
  elrsb::Link link;
  const HostFrame* lf = linkCap.last(elrsb::kMsgLink);
  TEST_ASSERT_NOT_NULL(lf);
  TEST_ASSERT_TRUE(elrsb::unpackLink(lf->payload.data(), lf->payload.size(), link));
  TEST_ASSERT_EQUAL_UINT8(0, link.upLq);
  TEST_ASSERT_EQUAL_UINT16(1, link.failsafeCount);
}

void test_bridge_link_statistics_reach_the_host() {
  Fixture fx;
  Sticks s;
  static const uint8_t kStats[10] = {62, 0, 100, 9, 0, 7, 3, 0, 0, 0};
  uint8_t frame[64];
  const size_t n = eb::crsf::buildFrame(frame, eb::crsf::kTypeLinkStats, kStats, 10);
  run(fx.b(), fx.now, 20, &s);
  fx.b().feedCrsf(frame, n, fx.now);
  run(fx.b(), fx.now, 1500, &s);

  const Capture cap = drain(fx.b(), eb::Bridge::kPortHost);
  const elrsb::State st = lastState(cap);
  TEST_ASSERT_EQUAL_UINT8(100, st.lq);
  TEST_ASSERT_EQUAL_INT8(-62, st.rssi);
  TEST_ASSERT_EQUAL_INT8(9, st.snr);

  const HostFrame* lf = cap.last(elrsb::kMsgLink);
  TEST_ASSERT_NOT_NULL(lf);
  elrsb::Link link;
  TEST_ASSERT_TRUE(elrsb::unpackLink(lf->payload.data(), lf->payload.size(), link));
  TEST_ASSERT_EQUAL_INT8(-62, link.upRssi1);
  TEST_ASSERT_EQUAL_UINT8(100, link.upLq);
  TEST_ASSERT_EQUAL_UINT16(100, link.txPowerMw);
  TEST_ASSERT_UINT16_WITHIN(2, 100, link.rcRateHz);
  TEST_ASSERT_TRUE(link.rcFrames > 100);
  TEST_ASSERT_EQUAL_UINT32(0, link.crsfErrors);
  TEST_ASSERT_TRUE((link.flags & elrsb::kFlagLinkUp) != 0);
}

// ---------------------------------------------------------------------------
// Commands

void test_bridge_cli_on_a_terminal() {
  Fixture fx;
  drain(fx.b(), eb::Bridge::kPortUsb);

  typeLine(fx.b(), eb::Bridge::kPortUsb, "set stream_hz 100", fx.now);
  Capture cap = drain(fx.b(), eb::Bridge::kPortUsb);
  // Typed characters are echoed, the answer follows, then a fresh prompt.
  TEST_ASSERT_TRUE(contains(cap.text, "set stream_hz 100\r\nok\r\n> "));
  TEST_ASSERT_EQUAL_UINT16(100, fx.b().config().streamHz);

  typeLine(fx.b(), eb::Bridge::kPortUsb, "get stream_hz", fx.now);
  cap = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_TRUE(contains(cap.text, "stream_hz = 100\r\n"));

  // Backspace edits the line.
  const char typed[] = "get stream_hzz\b\r";
  fx.b().feedPort(eb::Bridge::kPortUsb, (const uint8_t*)typed, sizeof typed - 1, fx.now);
  cap = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_TRUE(contains(cap.text, "stream_hz = 100\r\n"));

  typeLine(fx.b(), eb::Bridge::kPortUsb, "frobnicate", fx.now);
  cap = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_TRUE(contains(cap.text, "ERR unknown command"));

  // An over-long line is refused as a whole rather than run truncated.
  const std::string longLine = "set stream_hz " + std::string(200, '1');
  typeLine(fx.b(), eb::Bridge::kPortUsb, longLine.c_str(), fx.now);
  cap = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_TRUE(contains(cap.text, "ERR line too long"));
  TEST_ASSERT_EQUAL_UINT16(100, fx.b().config().streamHz);
}

void test_bridge_cli_over_binary_protocol() {
  Fixture fx;
  std::string out = cli(fx.b(), "version", fx.now);
  TEST_ASSERT_TRUE(contains(out, "elrs-bridge 0.1.0 protocol 1 board test"));

  TEST_ASSERT_EQUAL_STRING("ok\n", cli(fx.b(), "set out1.name steer", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("ok\n", cli(fx.b(), "set out1.rev=on", fx.now).c_str());
  out = cli(fx.b(), "get out1", fx.now);
  TEST_ASSERT_TRUE(contains(out, "out1.name = steer\n"));
  TEST_ASSERT_TRUE(contains(out, "out1.rev = on\n"));
  TEST_ASSERT_FALSE(contains(out, "out10."));  // "out1" must not match out10..out16

  out = cli(fx.b(), "set stream_hz 9999", fx.now);
  TEST_ASSERT_TRUE(contains(out, "ERR stream_hz: out of range (1..500)"));
  out = cli(fx.b(), "set nope 1", fx.now);
  TEST_ASSERT_TRUE(contains(out, "ERR unknown setting 'nope'"));
  out = cli(fx.b(), "set host_stream sideways", fx.now);
  TEST_ASSERT_TRUE(contains(out, "ERR host_stream"));
  TEST_ASSERT_TRUE(contains(out, "off,bin,text"));
  out = cli(fx.b(), "set crsf_rx_pin 3", fx.now);
  TEST_ASSERT_TRUE(contains(out, "ERR crsf_rx_pin"));
  out = cli(fx.b(), "set crsf_rx_pin 32", fx.now);
  TEST_ASSERT_TRUE(contains(out, "reboot"));

  out = cli(fx.b(), "diff", fx.now);
  TEST_ASSERT_TRUE(contains(out, "set out1.name steer\n"));
  TEST_ASSERT_TRUE(contains(out, "set crsf_rx_pin 32\n"));
  TEST_ASSERT_FALSE(contains(out, "failsafe_ms"));

  out = cli(fx.b(), "get stream_hz", fx.now);
  TEST_ASSERT_TRUE(contains(out, "range: 1..500"));

  out = cli(fx.b(), "help", fx.now);
  TEST_ASSERT_TRUE(contains(out, "status"));
  out = cli(fx.b(), "status", fx.now);
  TEST_ASSERT_TRUE(contains(out, "link: DOWN"));
  TEST_ASSERT_TRUE(contains(out, "changed but not saved"));
  out = cli(fx.b(), "ch", fx.now);
  TEST_ASSERT_TRUE(contains(out, "steer"));
}

void test_bridge_full_dump_fits_the_port_queue() {
  // `dump` is the largest reply; it must arrive whole while the stream runs.
  Fixture fx;
  run(fx.b(), fx.now, 5);
  const std::string out = cli(fx.b(), "dump", fx.now);
  const size_t lines = (size_t)std::count(out.begin(), out.end(), '\n');
  TEST_ASSERT_EQUAL_UINT(settingCount(), lines);
  TEST_ASSERT_TRUE(contains(out, "set crsf_rx_pin 16\n"));
  TEST_ASSERT_TRUE(contains(out, "set aux4.fs_us 0\n"));

  // And it replays cleanly into a bridge with different settings.
  Fixture other;
  cli(other.b(), "set stream_hz 7", other.now);
  size_t start = 0;
  while (start < out.size()) {
    const size_t end = out.find('\n', start);
    const std::string line = out.substr(start, end - start);
    const std::string reply = cli(other.b(), line.c_str(), other.now);
    TEST_ASSERT_TRUE_MESSAGE(reply.compare(0, 2, "ok") == 0, line.c_str());
    start = end + 1;
  }
  TEST_ASSERT_EQUAL_UINT16(50, other.b().config().streamHz);
}

void test_bridge_settings_survive_reboot() {
  Fixture fx;
  cli(fx.b(), "set stream_hz 100", fx.now);
  cli(fx.b(), "set out2.rev on", fx.now);
  cli(fx.b(), "set usb_stream bin", fx.now);
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "save", fx.now), "saved"));
  TEST_ASSERT_TRUE(contains(fx.platform.saved, "stream_hz 100\n"));
  cli(fx.b(), "set stream_hz 200", fx.now);  // changed after the save: must not persist

  fx.reboot();
  TEST_ASSERT_EQUAL_UINT16(100, fx.b().config().streamHz);
  TEST_ASSERT_EQUAL_UINT8(1, fx.b().config().out[1].rev);
  // usb_stream applies from boot: binary frames and no CLI banner.
  run(fx.b(), fx.now, 100);
  const Capture cap = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_TRUE(cap.count(elrsb::kMsgState) > 0);
  TEST_ASSERT_FALSE(contains(cap.text, "elrs-bridge"));

  TEST_ASSERT_TRUE(contains(cli(fx.b(), "defaults", fx.now), "defaults restored"));
  TEST_ASSERT_EQUAL_UINT16(50, fx.b().config().streamHz);
  cli(fx.b(), "save", fx.now);
  fx.reboot();
  TEST_ASSERT_EQUAL_UINT16(50, fx.b().config().streamHz);
  TEST_ASSERT_EQUAL_UINT8(0, fx.b().config().out[1].rev);
}

void test_bridge_reboot_command_asks_the_platform() {
  Fixture fx;
  TEST_ASSERT_EQUAL_INT(0, fx.platform.rebootRequests);
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "reboot", fx.now), "rebooting"));
  TEST_ASSERT_EQUAL_INT(1, fx.platform.rebootRequests);
}

// ---------------------------------------------------------------------------
// Interlocks: what is refused or held back while the robot could move

void test_bridge_refuses_disruptive_commands_while_armed() {
  Fixture fx;
  Sticks s;
  armRobot(fx, s);
  drainCrsf(fx.b());
  cli(fx.b(), "set stream_hz 100", fx.now);

  TEST_ASSERT_EQUAL_STRING("ERR disarm first\n", cli(fx.b(), "reboot", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("ERR disarm first\n", cli(fx.b(), "defaults", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("ERR disarm first\n", cli(fx.b(), "bind", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("ERR disarm first\n", cli(fx.b(), "save", fx.now).c_str());
  TEST_ASSERT_EQUAL_INT(0, fx.platform.rebootRequests);
  TEST_ASSERT_FALSE(fx.platform.haveSaved);
  TEST_ASSERT_EQUAL_UINT16(100, fx.b().config().streamHz);
  TEST_ASSERT_NULL(findCrsf(drainCrsf(fx.b()), 0x32));
  TEST_ASSERT_TRUE(fx.b().armed());

  // Disarmed, they go through again.
  s.ticks[4] = 191;
  run(fx.b(), fx.now, 50, &s);
  TEST_ASSERT_FALSE(fx.b().armed());
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "bind", fx.now), "bind command sent"));
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "save", fx.now), "saved"));
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "reboot", fx.now), "rebooting"));
  TEST_ASSERT_EQUAL_INT(1, fx.platform.rebootRequests);

  // With no arm switch configured "armed" just means "linked", so there is
  // nothing to disarm and the commands are not held back.
  cli(fx.b(), "set arm_ch 0", fx.now);
  run(fx.b(), fx.now, 600);     // the handset goes away...
  run(fx.b(), fx.now, 50, &s);  // ...and comes back
  TEST_ASSERT_TRUE(fx.b().armed());
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "reboot", fx.now), "rebooting"));
  TEST_ASSERT_EQUAL_INT(2, fx.platform.rebootRequests);
}

void test_bridge_changing_arming_settings_never_arms_by_itself() {
  // CH7 is on for some other purpose while the arm switch (CH5) is off.
  Fixture fx;
  Sticks s;
  s.ticks[6] = 1792;
  run(fx.b(), fx.now, kArmOffHold, &s);
  TEST_ASSERT_TRUE(fx.b().linkUp());
  TEST_ASSERT_FALSE(fx.b().armed());

  // Pointing the arm switch at CH7 must not arm just because CH7 is high.
  TEST_ASSERT_EQUAL_STRING("ok (disarmed: switch the arm switch off, then on, to arm)\n",
                           cli(fx.b(), "set arm_ch 7", fx.now).c_str());
  run(fx.b(), fx.now, 1000, &s);
  TEST_ASSERT_FALSE(fx.b().armed());
  // 'status' says why, since this is what people trip over.
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "status", fx.now), "switch it off, then on"));
  s.ticks[6] = 191;
  run(fx.b(), fx.now, kArmOffHold, &s);
  s.ticks[6] = 1792;
  run(fx.b(), fx.now, 30, &s);
  TEST_ASSERT_TRUE(fx.b().armed());

  // Back to disarmed (CH7 off), with CH5 on. Restoring the defaults makes
  // CH5 the arm switch again - and must not arm on the spot either.
  s.ticks[6] = 191;
  s.ticks[4] = 1792;
  run(fx.b(), fx.now, 100, &s);
  TEST_ASSERT_FALSE(fx.b().armed());
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "defaults", fx.now), "defaults restored"));
  run(fx.b(), fx.now, 1000, &s);
  TEST_ASSERT_FALSE(fx.b().armed());

  // The same goes for the threshold: CH5 sits at 1500 us, which is "off"
  // until the threshold is lowered beneath it.
  s.ticks[4] = eb::crsf::usToTicks(1500);
  run(fx.b(), fx.now, kArmOffHold, &s);
  cli(fx.b(), "set arm_us 1400", fx.now);
  run(fx.b(), fx.now, 1000, &s);
  TEST_ASSERT_FALSE(fx.b().armed());

  // Switching the guard off does not release what it was holding back...
  TEST_ASSERT_EQUAL_STRING("ok\n", cli(fx.b(), "set arm_guard off", fx.now).c_str());
  run(fx.b(), fx.now, 1000, &s);
  TEST_ASSERT_FALSE(fx.b().armed());

  // ...and neither does removing the arm switch: that starts with the next
  // connection.
  TEST_ASSERT_EQUAL_STRING("ok (disarmed: arms when the handset next connects)\n",
                           cli(fx.b(), "set arm_ch 0", fx.now).c_str());
  run(fx.b(), fx.now, 1000, &s);
  TEST_ASSERT_FALSE(fx.b().armed());
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "status", fx.now), "when the handset next connects"));
  run(fx.b(), fx.now, 600);
  TEST_ASSERT_FALSE(fx.b().linkUp());
  run(fx.b(), fx.now, 50, &s);
  TEST_ASSERT_TRUE(fx.b().armed());
  TEST_ASSERT_FALSE(contains(cli(fx.b(), "status", fx.now), "not armed"));

  // arm_guard only decides what the next link loss will ask for, so switching
  // it on does not disarm a robot that is being driven.
  TEST_ASSERT_EQUAL_STRING("ok\n", cli(fx.b(), "set arm_guard on", fx.now).c_str());
  run(fx.b(), fx.now, 50, &s);
  TEST_ASSERT_TRUE(fx.b().armed());
}

void test_bridge_arming_rules_apply_from_the_first_frame() {
  // arm_guard off means armed whenever the link is up and the switch is on.
  // That has to hold from the very first frame, even if the receiver is
  // already talking when the bridge starts.
  Fixture fx;
  cli(fx.b(), "set arm_guard off", fx.now);
  cli(fx.b(), "save", fx.now);
  fx.reboot();
  Sticks s;
  s.ticks[4] = 1792;
  sendRc(fx.b(), s, fx.now);  // before the first tick
  TEST_ASSERT_TRUE(fx.b().armed());

  // With the guard at its default the same first frame arms nothing.
  Fixture guarded;
  sendRc(guarded.b(), s, guarded.now);
  TEST_ASSERT_TRUE(guarded.b().linkUp());
  TEST_ASSERT_FALSE(guarded.b().armed());
}

// ---------------------------------------------------------------------------
// Host protocol messages

void test_bridge_cli_replies_carry_the_request_tag() {
  Fixture fx;
  drain(fx.b(), eb::Bridge::kPortHost);

  sendCli(fx.b(), eb::Bridge::kPortHost, "version", 0x5A, fx.now);
  sendCli(fx.b(), eb::Bridge::kPortHost, "get stream_hz", 0xA5, fx.now);
  const Capture cap = drain(fx.b(), eb::Bridge::kPortHost);
  // Two replies, each marked with the tag of the command it answers.
  std::string first;
  std::string second;
  for (size_t i = 0; i < cap.frames.size(); ++i) {
    if (cap.frames[i].type != elrsb::kMsgCliReply) continue;
    const std::vector<uint8_t>& p = cap.frames[i].payload;
    TEST_ASSERT_TRUE(p.size() >= 2);
    std::string& into = p[1] == 0x5A ? first : second;
    TEST_ASSERT_TRUE(p[1] == 0x5A || p[1] == 0xA5);
    into.append(p.begin() + 2, p.end());
  }
  TEST_ASSERT_TRUE(contains(first, "elrs-bridge 0.1.0"));
  TEST_ASSERT_TRUE(contains(second, "stream_hz = 50"));

  // The longest line that fits one frame is still run (and answered).
  const std::string longest(elrsb::kMaxPayload - 1, 'x');
  sendCli(fx.b(), eb::Bridge::kPortHost, longest.c_str(), 9, fx.now);
  TEST_ASSERT_TRUE(contains(cliReplyText(drain(fx.b(), eb::Bridge::kPortHost), 9),
                            "ERR unknown command"));

  // A CLI message with no tag at all is malformed.
  sendHostFrame(fx.b(), eb::Bridge::kPortHost, elrsb::kMsgCli, nullptr, 0, fx.now);
  const Capture bad = drain(fx.b(), eb::Bridge::kPortHost);
  const HostFrame* ack = bad.last(elrsb::kMsgAck);
  TEST_ASSERT_NOT_NULL(ack);
  TEST_ASSERT_EQUAL_HEX8(elrsb::kMsgCli, ack->payload[0]);
  TEST_ASSERT_EQUAL_HEX8(elrsb::kAckBadLength, ack->payload[1]);
}

void test_bridge_set_stream_message() {
  Fixture fx;
  drain(fx.b(), eb::Bridge::kPortUsb);
  elrsb::SetStream s;
  s.mode = elrsb::kStreamBinary;
  s.hz = 100;
  s.options = elrsb::kStreamOptRaw;
  uint8_t p[8];
  sendHostFrame(fx.b(), eb::Bridge::kPortUsb, elrsb::kMsgSetStream, p, elrsb::packSetStream(p, s),
                fx.now);
  Sticks sticks;
  sticks.ticks[2] = 1811;
  run(fx.b(), fx.now, 200, &sticks);
  Capture cap = drain(fx.b(), eb::Bridge::kPortUsb);
  const HostFrame* ack = cap.last(elrsb::kMsgAck);
  TEST_ASSERT_NOT_NULL(ack);
  TEST_ASSERT_EQUAL_HEX8(elrsb::kMsgSetStream, ack->payload[0]);
  TEST_ASSERT_EQUAL_HEX8(elrsb::kAckOk, ack->payload[1]);
  TEST_ASSERT_INT_WITHIN(1, 20, cap.count(elrsb::kMsgState));
  TEST_ASSERT_EQUAL_INT(cap.count(elrsb::kMsgState), cap.count(elrsb::kMsgRawChannels));

  const HostFrame* raw = cap.last(elrsb::kMsgRawChannels);
  uint16_t us[16];
  uint8_t count = 0;
  TEST_ASSERT_TRUE(elrsb::unpackRawChannels(raw->payload.data(), raw->payload.size(), us, count));
  TEST_ASSERT_EQUAL_UINT8(16, count);
  TEST_ASSERT_EQUAL_UINT16(1500, us[0]);
  TEST_ASSERT_EQUAL_UINT16(2012, us[2]);
  TEST_ASSERT_EQUAL_UINT16(999, us[4]);

  // A bad mode is refused and changes nothing.
  s.mode = 9;
  sendHostFrame(fx.b(), eb::Bridge::kPortUsb, elrsb::kMsgSetStream, p, elrsb::packSetStream(p, s),
                fx.now);
  cap = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_EQUAL_HEX8(elrsb::kAckBadValue, cap.last(elrsb::kMsgAck)->payload[1]);

  // Turning the stream off makes the port quiet again.
  s.mode = elrsb::kStreamOff;
  s.options = 0;
  sendHostFrame(fx.b(), eb::Bridge::kPortUsb, elrsb::kMsgSetStream, p, elrsb::packSetStream(p, s),
                fx.now);
  drain(fx.b(), eb::Bridge::kPortUsb);
  run(fx.b(), fx.now, 200, &sticks);
  TEST_ASSERT_EQUAL_INT(0, drain(fx.b(), eb::Bridge::kPortUsb).count(elrsb::kMsgState));
}

void test_bridge_info_request_and_unknown_messages() {
  Fixture fx;
  drain(fx.b(), eb::Bridge::kPortHost);
  sendHostFrame(fx.b(), eb::Bridge::kPortHost, elrsb::kMsgGetInfo, nullptr, 0, fx.now);
  Capture cap = drain(fx.b(), eb::Bridge::kPortHost);
  const HostFrame* f = cap.last(elrsb::kMsgInfo);
  TEST_ASSERT_NOT_NULL(f);
  elrsb::Info info;
  TEST_ASSERT_TRUE(elrsb::unpackInfo(f->payload.data(), f->payload.size(), info));
  TEST_ASSERT_EQUAL_UINT8(1, info.protocol);
  TEST_ASSERT_EQUAL_UINT8(16, info.channels);
  TEST_ASSERT_EQUAL_UINT8(4, info.auxOutputs);
  TEST_ASSERT_EQUAL_STRING("test", info.board);

  sendHostFrame(fx.b(), eb::Bridge::kPortHost, 0x7E, nullptr, 0, fx.now);
  cap = drain(fx.b(), eb::Bridge::kPortHost);
  f = cap.last(elrsb::kMsgAck);
  TEST_ASSERT_NOT_NULL(f);
  TEST_ASSERT_EQUAL_HEX8(0x7E, f->payload[0]);
  TEST_ASSERT_EQUAL_HEX8(elrsb::kAckUnknown, f->payload[1]);

  // A truncated telemetry message is refused.
  const uint8_t shortBattery[4] = {0, 0, 0, 0};
  sendHostFrame(fx.b(), eb::Bridge::kPortHost, elrsb::kMsgTlmBattery, shortBattery, 4, fx.now);
  cap = drain(fx.b(), eb::Bridge::kPortHost);
  TEST_ASSERT_EQUAL_HEX8(elrsb::kAckBadLength, cap.last(elrsb::kMsgAck)->payload[1]);

  // One-shot request.
  const uint8_t want = elrsb::kMsgLink;
  sendHostFrame(fx.b(), eb::Bridge::kPortHost, elrsb::kMsgRequest, &want, 1, fx.now);
  cap = drain(fx.b(), eb::Bridge::kPortHost);
  TEST_ASSERT_EQUAL_INT(1, cap.count(elrsb::kMsgLink));
}

void test_bridge_text_stream() {
  Fixture fx;
  drain(fx.b(), eb::Bridge::kPortUsb);
  typeLine(fx.b(), eb::Bridge::kPortUsb, "stream text 10", fx.now);
  Sticks s;
  s.ticks[0] = 1811;
  s.ticks[1] = 172;
  run(fx.b(), fx.now, 1000, &s);
  const Capture cap = drain(fx.b(), eb::Bridge::kPortUsb);

  int lines = 0;
  std::string last;
  size_t pos = 0;
  while ((pos = cap.text.find("$RC,", pos)) != std::string::npos) {
    const size_t end = cap.text.find("\r\n", pos);
    TEST_ASSERT_TRUE(end != std::string::npos);
    last = cap.text.substr(pos, end - pos);
    // Verify the NMEA-style checksum of every line.
    const size_t star = last.find('*');
    TEST_ASSERT_TRUE(star != std::string::npos);
    uint8_t sum = 0;
    for (size_t i = 1; i < star; ++i) sum ^= (uint8_t)last[i];
    char expect[3];
    snprintf(expect, sizeof expect, "%02X", sum);
    TEST_ASSERT_EQUAL_STRING(expect, last.substr(star + 1).c_str());
    ++lines;
    pos = end;
  }
  TEST_ASSERT_INT_WITHIN(1, 10, lines);
  // flags 1 = link up; lq 0 (no link statistics yet); then the 16 outputs.
  TEST_ASSERT_TRUE_MESSAGE(last.compare(0, 8, "$RC,1,0,") == 0, last.c_str());
  TEST_ASSERT_TRUE_MESSAGE(contains(last, ",1000,-1000,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"), last.c_str());

  // While streaming, typed commands are neither echoed nor prompted...
  typeLine(fx.b(), eb::Bridge::kPortUsb, "tlm bat 11.1", fx.now);
  Capture quiet = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_FALSE(contains(quiet.text, "tlm"));
  TEST_ASSERT_FALSE(contains(quiet.text, ">"));
  // ...and 'stream off' returns to the interactive prompt.
  typeLine(fx.b(), eb::Bridge::kPortUsb, "stream off", fx.now);
  quiet = drain(fx.b(), eb::Bridge::kPortUsb);
  TEST_ASSERT_TRUE(contains(quiet.text, "ok\r\n> "));
}

// ---------------------------------------------------------------------------
// Telemetry to the handset

void test_bridge_host_telemetry_reaches_the_receiver() {
  Fixture fx;
  Sticks s;
  run(fx.b(), fx.now, 50, &s);
  drainCrsf(fx.b());

  elrsb::Battery b = {12.6f, 1.5f, 1234.0f, 80};
  uint8_t p[32];
  sendHostFrame(fx.b(), eb::Bridge::kPortHost, elrsb::kMsgTlmBattery, p, elrsb::packBattery(p, b),
                fx.now);
  sendHostFrame(fx.b(), eb::Bridge::kPortHost, elrsb::kMsgTlmMode, (const uint8_t*)"AUTO", 4,
                fx.now);
  run(fx.b(), fx.now, 500, &s);

  const std::vector<std::vector<uint8_t> > frames = drainCrsf(fx.b());
  // One frame per tlm_period_ms (100 ms), alternating between the two items.
  TEST_ASSERT_INT_WITHIN(1, 5, (int)frames.size());
  const std::vector<uint8_t>* battery = findCrsf(frames, 0x08);
  TEST_ASSERT_NOT_NULL(battery);
  static const uint8_t kBattery[] = {0x08, 0x00, 0x7E, 0x00, 0x0F, 0x00, 0x04, 0xD2, 0x50};
  TEST_ASSERT_EQUAL_UINT(sizeof kBattery, battery->size());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kBattery, battery->data(), sizeof kBattery);
  const std::vector<uint8_t>* mode = findCrsf(frames, 0x21);
  TEST_ASSERT_NOT_NULL(mode);
  static const uint8_t kMode[] = {0x21, 'A', 'U', 'T', 'O', 0x00};
  TEST_ASSERT_EQUAL_UINT(sizeof kMode, mode->size());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kMode, mode->data(), sizeof kMode);

  // An empty mode message hands the line back to the bridge's own status.
  sendHostFrame(fx.b(), eb::Bridge::kPortHost, elrsb::kMsgTlmMode, nullptr, 0, fx.now);
  run(fx.b(), fx.now, 300, &s);
  const std::vector<std::vector<uint8_t> > after = drainCrsf(fx.b());
  const std::vector<uint8_t>* own = findCrsf(after, 0x21);
  TEST_ASSERT_NOT_NULL(own);
  TEST_ASSERT_EQUAL_STRING("DISARMED", (const char*)own->data() + 1);
}

void test_bridge_reports_its_own_mode_without_a_host() {
  Fixture fx;
  Sticks s;
  run(fx.b(), fx.now, kArmOffHold, &s);
  std::vector<std::vector<uint8_t> > frames = drainCrsf(fx.b());
  const std::vector<uint8_t>* mode = findCrsf(frames, 0x21);
  TEST_ASSERT_NOT_NULL(mode);
  TEST_ASSERT_EQUAL_STRING("DISARMED", (const char*)mode->data() + 1);

  s.ticks[4] = 1792;
  run(fx.b(), fx.now, 300, &s);
  frames = drainCrsf(fx.b());
  mode = &frames.back();
  TEST_ASSERT_EQUAL_HEX8(0x21, (*mode)[0]);
  TEST_ASSERT_EQUAL_STRING("ARMED", (const char*)mode->data() + 1);

  // The measured battery voltage joins in when the platform provides one.
  fx.b().setLocalVbat(11.1f, fx.now);
  run(fx.b(), fx.now, 300, &s);
  frames = drainCrsf(fx.b());
  const std::vector<uint8_t>* battery = findCrsf(frames, 0x08);
  TEST_ASSERT_NOT_NULL(battery);
  TEST_ASSERT_EQUAL_HEX8(0x00, (*battery)[1]);
  TEST_ASSERT_EQUAL_HEX8(111, (*battery)[2]);

  cli(fx.b(), "set tlm_auto_mode off", fx.now);
  cli(fx.b(), "set tlm_timeout_ms 200", fx.now);
  run(fx.b(), fx.now, 400, &s);
  drainCrsf(fx.b());
  run(fx.b(), fx.now, 400, &s);
  TEST_ASSERT_EQUAL_UINT(0, drainCrsf(fx.b()).size());
}

void test_bridge_stale_host_telemetry_expires() {
  Fixture fx;
  Sticks s;
  cli(fx.b(), "set tlm_auto_mode off", fx.now);
  cli(fx.b(), "set tlm_timeout_ms 1000", fx.now);
  run(fx.b(), fx.now, 50, &s);
  typeLine(fx.b(), eb::Bridge::kPortUsb, "tlm vario 1.25", fx.now);
  run(fx.b(), fx.now, 900, &s);
  std::vector<std::vector<uint8_t> > frames = drainCrsf(fx.b());
  TEST_ASSERT_TRUE(frames.size() >= 8);
  TEST_ASSERT_EQUAL_HEX8(0x07, frames[0][0]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frames[0][1]);
  TEST_ASSERT_EQUAL_HEX8(0x7D, frames[0][2]);

  // The host goes quiet: the value stops being repeated to the handset.
  run(fx.b(), fx.now, 300, &s);
  drainCrsf(fx.b());
  run(fx.b(), fx.now, 1000, &s);
  TEST_ASSERT_EQUAL_UINT(0, drainCrsf(fx.b()).size());
}

void test_bridge_sends_no_telemetry_without_a_link() {
  Fixture fx;
  typeLine(fx.b(), eb::Bridge::kPortUsb, "tlm bat 12.0", fx.now);
  run(fx.b(), fx.now, 1000);
  TEST_ASSERT_EQUAL_UINT(0, drainCrsf(fx.b()).size());
}

void test_bridge_telemetry_can_be_switched_off() {
  Fixture fx;
  Sticks s;
  cli(fx.b(), "set tlm_period_ms 0", fx.now);
  run(fx.b(), fx.now, 1000, &s);
  TEST_ASSERT_EQUAL_UINT(0, drainCrsf(fx.b()).size());
}

void test_bridge_cli_telemetry_forms() {
  Fixture fx;
  Sticks s;
  cli(fx.b(), "set tlm_auto_mode off", fx.now);
  run(fx.b(), fx.now, 50, &s);
  drainCrsf(fx.b());

  TEST_ASSERT_EQUAL_STRING("", cli(fx.b(), "tlm bat 12.6 1.5 1234 80", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("", cli(fx.b(), "tlm gps 37.7749 -122.4194 2.5 270.5 12 9", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("", cli(fx.b(), "tlm att 0.1 -0.2 3.0", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("", cli(fx.b(), "tlm mode WAYPOINT 3", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("", cli(fx.b(), "tlm vario -0.5", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("", cli(fx.b(), "tlm alt 123.4 0.5", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("", cli(fx.b(), "tlm aspd 10", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("", cli(fx.b(), "tlm rpm 1000 -200", fx.now).c_str());
  TEST_ASSERT_EQUAL_STRING("", cli(fx.b(), "tlm temp 25.5 -10", fx.now).c_str());
  run(fx.b(), fx.now, 1000, &s);

  const std::vector<std::vector<uint8_t> > frames = drainCrsf(fx.b());
  static const uint8_t kBattery[] = {0x08, 0x00, 0x7E, 0x00, 0x0F, 0x00, 0x04, 0xD2, 0x50};
  static const uint8_t kGps[] = {0x02, 0x16, 0x83, 0xFE, 0x08, 0xB7, 0x08, 0x48, 0x30,
                                 0x00, 0x5A, 0x69, 0xAA, 0x03, 0xF4, 0x09};
  static const uint8_t kAtt[] = {0x1E, 0xF8, 0x30, 0x03, 0xE8, 0x75, 0x30};
  static const uint8_t kMode[] = {0x21, 'W', 'A', 'Y', 'P', 'O', 'I', 'N', 'T', ' ', '3', 0x00};
  static const uint8_t kVario[] = {0x07, 0xFF, 0xCE};
  static const uint8_t kBaro[] = {0x09, 0x2B, 0xE2, 0x00, 0x32};
  static const uint8_t kAspd[] = {0x0A, 0x01, 0x68};
  static const uint8_t kRpm[] = {0x0C, 0x00, 0x00, 0x03, 0xE8, 0xFF, 0xFF, 0x38};
  static const uint8_t kTemp[] = {0x0D, 0x00, 0x00, 0xFF, 0xFF, 0x9C};
  struct Expect {
    const uint8_t* bytes;
    size_t len;
  };
  const Expect expected[] = {
      {kBattery, sizeof kBattery}, {kGps, sizeof kGps},   {kAtt, sizeof kAtt},
      {kMode, sizeof kMode},       {kVario, sizeof kVario}, {kBaro, sizeof kBaro},
      {kAspd, sizeof kAspd},       {kRpm, sizeof kRpm},   {kTemp, sizeof kTemp}};
  for (size_t i = 0; i < sizeof expected / sizeof expected[0]; ++i) {
    const std::vector<uint8_t>* f = findCrsf(frames, expected[i].bytes[0]);
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "a telemetry frame type is missing");
    TEST_ASSERT_EQUAL_UINT(expected[i].len, f->size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected[i].bytes, f->data(), expected[i].len);
  }

  TEST_ASSERT_TRUE(contains(cli(fx.b(), "tlm", fx.now), "tlm bat"));
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "tlm bat twelve", fx.now), "ERR"));
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "tlm gps 91 0", fx.now), "ERR"));
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "tlm nope 1", fx.now), "ERR"));
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "tlm att 1 2", fx.now), "ERR"));
}

void test_bridge_bind_and_raw_crsf() {
  Fixture fx;
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "bind", fx.now), "bind command sent"));
  uint8_t buf[64];
  size_t n = fx.b().readCrsfTx(buf, sizeof buf);
  static const uint8_t kBind[9] = {0xC8, 0x07, 0x32, 0xEC, 0xC8, 0x10, 0x01, 0x9E, 0xE8};
  TEST_ASSERT_EQUAL_UINT(9, n);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kBind, buf, 9);

  // The host can also send any CRSF frame it likes: type then payload.
  static const uint8_t kRaw[] = {0x07, 0x00, 0x7D};
  sendHostFrame(fx.b(), eb::Bridge::kPortHost, elrsb::kMsgCrsfTx, kRaw, sizeof kRaw, fx.now);
  const std::vector<std::vector<uint8_t> > frames = drainCrsf(fx.b());
  TEST_ASSERT_EQUAL_UINT(1, frames.size());
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kRaw, frames[0].data(), sizeof kRaw);
}

void test_bridge_forwards_other_crsf_frames_when_asked() {
  Fixture fx;
  static const uint8_t kPayload[] = {0xEA, 0xEE, 'x', 0x00};
  uint8_t frame[64];
  const size_t n = eb::crsf::buildFrame(frame, eb::crsf::kTypeDeviceInfo, kPayload, sizeof kPayload);

  fx.b().feedCrsf(frame, n, fx.now);
  TEST_ASSERT_EQUAL_INT(0, drain(fx.b(), eb::Bridge::kPortHost).count(elrsb::kMsgCrsfFrame));

  cli(fx.b(), "set fwd_crsf on", fx.now);
  fx.b().feedCrsf(frame, n, fx.now);
  const Capture cap = drain(fx.b(), eb::Bridge::kPortHost);
  const HostFrame* f = cap.last(elrsb::kMsgCrsfFrame);
  TEST_ASSERT_NOT_NULL(f);
  TEST_ASSERT_EQUAL_UINT(2 + sizeof kPayload, f->payload.size());
  TEST_ASSERT_EQUAL_HEX8(0xC8, f->payload[0]);
  TEST_ASSERT_EQUAL_HEX8(0x29, f->payload[1]);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(kPayload, f->payload.data() + 2, sizeof kPayload);
}

// ---------------------------------------------------------------------------
// Aux pins

void test_bridge_aux_outputs() {
  Fixture fx;
  TEST_ASSERT_EQUAL(eb::AuxCommand::kDisabled, fx.b().aux(0).kind);

  cli(fx.b(), "set aux1.pin 25", fx.now);   // servo following output 1, gated by arming
  cli(fx.b(), "set aux2.pin 26", fx.now);   // servo following output 6 (unipolar), ungated
  cli(fx.b(), "set aux2.src 6", fx.now);
  cli(fx.b(), "set aux2.gate off", fx.now);
  cli(fx.b(), "set aux2.fs_us 1100", fx.now);
  cli(fx.b(), "set aux3.pin 27", fx.now);   // "armed" indicator, inverted
  cli(fx.b(), "set aux3.mode armed", fx.now);
  cli(fx.b(), "set aux3.inv on", fx.now);
  cli(fx.b(), "set aux4.pin 14", fx.now);   // switch following output 6
  cli(fx.b(), "set aux4.mode switch", fx.now);
  cli(fx.b(), "set aux4.src 6", fx.now);
  cli(fx.b(), "set aux4.gate off", fx.now);

  // Pins are only attached at start-up: nothing changes until a reboot.
  TEST_ASSERT_EQUAL(eb::AuxCommand::kDisabled, fx.b().aux(0).kind);
  cli(fx.b(), "save", fx.now);
  fx.reboot();

  // No link: servo outputs sit at their failsafe pulse, levels are inactive.
  TEST_ASSERT_EQUAL(eb::AuxCommand::kPulse, fx.b().aux(0).kind);
  TEST_ASSERT_EQUAL_INT8(25, fx.b().aux(0).pin);
  TEST_ASSERT_EQUAL_UINT16(0, fx.b().aux(0).pulseUs);
  TEST_ASSERT_EQUAL_UINT16(1100, fx.b().aux(1).pulseUs);
  TEST_ASSERT_EQUAL(eb::AuxCommand::kLevel, fx.b().aux(2).kind);
  TEST_ASSERT_EQUAL_INT8(27, fx.b().aux(2).pin);
  TEST_ASSERT_TRUE(fx.b().aux(2).level);    // not armed, inverted
  TEST_ASSERT_FALSE(fx.b().aux(3).level);

  Sticks s;
  s.ticks[0] = 1811;
  s.ticks[5] = 1792;
  run(fx.b(), fx.now, kArmOffHold, &s);
  // Linked but disarmed: the gated servo stays at failsafe, the others follow.
  TEST_ASSERT_EQUAL_UINT16(0, fx.b().aux(0).pulseUs);
  TEST_ASSERT_EQUAL_UINT16(2000, fx.b().aux(1).pulseUs);
  TEST_ASSERT_TRUE(fx.b().aux(2).level);
  TEST_ASSERT_TRUE(fx.b().aux(3).level);

  s.ticks[4] = 1792;
  run(fx.b(), fx.now, 50, &s);
  TEST_ASSERT_TRUE(fx.b().armed());
  TEST_ASSERT_EQUAL_UINT16(2000, fx.b().aux(0).pulseUs);   // +1000 -> max_us
  TEST_ASSERT_FALSE(fx.b().aux(2).level);

  s.ticks[0] = 992;
  s.ticks[5] = 992;
  run(fx.b(), fx.now, 50, &s);
  TEST_ASSERT_EQUAL_UINT16(1500, fx.b().aux(0).pulseUs);   // 0 -> centre
  TEST_ASSERT_EQUAL_UINT16(1500, fx.b().aux(1).pulseUs);   // 500 of 1000 -> centre
  TEST_ASSERT_TRUE(fx.b().aux(3).level);                   // 500 counts as on

  s.ticks[0] = 172;
  s.ticks[5] = 191;
  run(fx.b(), fx.now, 50, &s);
  TEST_ASSERT_EQUAL_UINT16(1000, fx.b().aux(0).pulseUs);   // -1000 -> min_us
  TEST_ASSERT_EQUAL_UINT16(1000, fx.b().aux(1).pulseUs);
  TEST_ASSERT_FALSE(fx.b().aux(3).level);

  // Link lost: everything returns to the safe state.
  run(fx.b(), fx.now, 600);
  TEST_ASSERT_EQUAL_UINT16(0, fx.b().aux(0).pulseUs);
  TEST_ASSERT_EQUAL_UINT16(1100, fx.b().aux(1).pulseUs);
  TEST_ASSERT_TRUE(fx.b().aux(2).level);
  TEST_ASSERT_FALSE(fx.b().aux(3).level);
}

void test_bridge_aux_pin_stays_controlled_when_its_settings_change() {
  // An "armed" output on GPIO25 holding a relay, and a servo on GPIO26.
  Fixture fx;
  cli(fx.b(), "set aux1.pin 25", fx.now);
  cli(fx.b(), "set aux1.mode armed", fx.now);
  cli(fx.b(), "set aux2.pin 26", fx.now);
  cli(fx.b(), "set aux2.gate off", fx.now);
  cli(fx.b(), "set aux2.fs_us 1500", fx.now);
  cli(fx.b(), "save", fx.now);
  fx.reboot();

  Sticks s;
  s.ticks[1] = 1811;
  armRobot(fx, s);
  TEST_ASSERT_TRUE(fx.b().aux(0).level);
  TEST_ASSERT_EQUAL_UINT16(2000, fx.b().aux(1).pulseUs);

  // Someone reassigns both pins while the robot is armed. The new pins only
  // apply after a reboot; until then the ones that are physically attached
  // must keep being driven - and above all must still go safe.
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "set aux1.pin -1", fx.now), "reboot"));
  cli(fx.b(), "set aux1.mode servo", fx.now);
  cli(fx.b(), "set aux2.pin 33", fx.now);
  cli(fx.b(), "set aux2.mode link", fx.now);
  run(fx.b(), fx.now, 50, &s);
  TEST_ASSERT_EQUAL(eb::AuxCommand::kLevel, fx.b().aux(0).kind);
  TEST_ASSERT_EQUAL_INT8(25, fx.b().aux(0).pin);
  TEST_ASSERT_TRUE(fx.b().aux(0).level);
  TEST_ASSERT_EQUAL(eb::AuxCommand::kPulse, fx.b().aux(1).kind);
  TEST_ASSERT_EQUAL_INT8(26, fx.b().aux(1).pin);
  TEST_ASSERT_EQUAL_UINT16(2000, fx.b().aux(1).pulseUs);

  s.ticks[4] = 191;  // disarm
  run(fx.b(), fx.now, 50, &s);
  TEST_ASSERT_FALSE(fx.b().aux(0).level);

  run(fx.b(), fx.now, 600);  // link lost
  TEST_ASSERT_FALSE(fx.b().aux(0).level);
  TEST_ASSERT_EQUAL_UINT16(1500, fx.b().aux(1).pulseUs);
}

void test_bridge_warns_about_shared_pins() {
  Fixture fx;
  std::string out = cli(fx.b(), "set aux1.pin 17", fx.now);
  TEST_ASSERT_TRUE_MESSAGE(contains(out, "WARNING GPIO17 is assigned to both crsf_tx_pin and aux1.pin"),
                           out.c_str());
  out = cli(fx.b(), "set aux2.pin 21", fx.now);
  TEST_ASSERT_TRUE_MESSAGE(contains(out, "WARNING GPIO21 is assigned to both motor_sda and aux2.pin"),
                           out.c_str());
}

// ---------------------------------------------------------------------------
// The wheels

// A Bluetooth controller report: sticks as Bluepad32 gives them.
eb::GamepadInput padReport(int lx, int ly, int rx) {
  eb::GamepadInput pad;
  pad.connected = true;
  pad.lx = (int16_t)lx;
  pad.ly = (int16_t)ly;
  pad.rx = (int16_t)rx;
  pad.ry = 0;
  pad.l2 = false;
  pad.r2 = false;
  return pad;
}

// Run with the controller reporting every 5 ms.
void runWithPad(Fixture& fx, uint32_t ms, const eb::GamepadInput& pad) {
  for (uint32_t i = 0; i < ms; ++i) {
    ++fx.now;
    if (fx.now % 5 == 0) fx.b().setGamepad(pad, fx.now);
    fx.b().tick(fx.now);
    pump(fx.b(), eb::Bridge::kPortUsb);
    pump(fx.b(), eb::Bridge::kPortHost);
  }
}

void test_bridge_hands_commands_to_the_motor_board() {
  Fixture fx;
  Sticks s;
  s.ticks[1] = 1811;  // right stick up
  s.ticks[11] = 1792;  // speed wheel all the way up: 80 % of 60
  armRobot(fx, s);
  run(fx.b(), fx.now, 500, &s);

  // A fresh set every 10 ms, and nothing in between.
  int8_t cmd[eb::kNumMotors];
  TEST_ASSERT_TRUE(fx.b().takeMotorCommands(cmd));
  TEST_ASSERT_FALSE(fx.b().takeMotorCommands(cmd));
  const int8_t kForward[eb::kNumMotors] = {48, -48, -48, 48};
  TEST_ASSERT_EQUAL_INT8_ARRAY(kForward, cmd, eb::kNumMotors);
  int sets = 0;
  for (int i = 0; i < 100; ++i) {
    run(fx.b(), fx.now, 1, &s);
    sets += fx.b().takeMotorCommands(cmd);
  }
  TEST_ASSERT_INT_WITHIN(1, 10, sets);

  // Disarming stops them at once.
  s.ticks[4] = 191;
  run(fx.b(), fx.now, 10, &s);
  TEST_ASSERT_TRUE(fx.b().takeMotorCommands(cmd));
  const int8_t kStopped[eb::kNumMotors] = {0, 0, 0, 0};
  TEST_ASSERT_EQUAL_INT8_ARRAY(kStopped, cmd, eb::kNumMotors);
}

void test_bridge_tells_the_host_who_drives() {
  Fixture fx;
  Sticks s;
  armRobot(fx, s);
  run(fx.b(), fx.now, 100, &s);
  elrsb::State st = lastState(drain(fx.b(), eb::Bridge::kPortHost));
  TEST_ASSERT_TRUE(st.armed());
  TEST_ASSERT_TRUE(st.motorsLive());
  TEST_ASSERT_FALSE(st.gamepad());

  // Handset off, PS4 controller on.
  run(fx.b(), fx.now, 600);
  runWithPad(fx, 100, padReport(0, 0, 0));
  st = lastState(drain(fx.b(), eb::Bridge::kPortHost));
  TEST_ASSERT_FALSE(st.linkUp());
  TEST_ASSERT_FALSE(st.armed());
  TEST_ASSERT_TRUE(st.gamepad());
  TEST_ASSERT_TRUE(st.motorsLive());
}

void test_bridge_status_shows_the_wheels() {
  Fixture fx;
  std::string out = cli(fx.b(), "status", fx.now);
  TEST_ASSERT_TRUE_MESSAGE(contains(out, "drive: nobody in charge, wheels stopped"), out.c_str());
  TEST_ASSERT_TRUE(contains(out, "motors M1..M4: +0 +0 +0 +0"));
  TEST_ASSERT_TRUE(contains(out, "motor board: nothing sent to it yet"));
  TEST_ASSERT_TRUE(contains(out, "bluetooth: no controller"));

  runWithPad(fx, 50, padReport(0, 0, 0));
  runWithPad(fx, 60, padReport(0, -512, 0));  // part way up to speed
  int8_t cmd[eb::kNumMotors];
  fx.b().takeMotorCommands(cmd);
  fx.b().reportMotorBoard(false);
  out = cli(fx.b(), "status", fx.now);
  TEST_ASSERT_TRUE_MESSAGE(
      contains(out, "drive: Bluetooth controller in charge, wheels live at 50 % speed"),
      out.c_str());
  TEST_ASSERT_TRUE_MESSAGE(contains(out, "(heading for +30 -30 -30 +30)"), out.c_str());
  TEST_ASSERT_TRUE(contains(out, "motor board: NOT ANSWERING at I2C address 0x34"));
  TEST_ASSERT_TRUE(contains(out, "bluetooth: controller connected"));

  fx.b().reportMotorBoard(true);
  out = cli(fx.b(), "status", fx.now);
  TEST_ASSERT_TRUE_MESSAGE(contains(out, "motor board: answering, 1 of 2 commands failed"),
                           out.c_str());

  // The controller falls silent: after bt_timeout_ms it counts as gone.
  run(fx.b(), fx.now, 600);
  out = cli(fx.b(), "status", fx.now);
  TEST_ASSERT_TRUE_MESSAGE(contains(out, "bluetooth: no controller"), out.c_str());
  cli(fx.b(), "set bt_enable off", fx.now);
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "status", fx.now), "bluetooth: off"));

  // The handset in charge: its speed wheel sets the speed.
  Sticks s;
  s.ticks[11] = 1792;  // speed wheel all the way up: drive_speed_top
  armRobot(fx, s);
  out = cli(fx.b(), "status", fx.now);
  TEST_ASSERT_TRUE_MESSAGE(
      contains(out, "drive: ELRS handset in charge, wheels live at 80 % speed"), out.c_str());
  s.ticks[4] = 191;  // arm switch off
  run(fx.b(), fx.now, 20, &s);
  out = cli(fx.b(), "status", fx.now);
  TEST_ASSERT_TRUE_MESSAGE(
      contains(out, "drive: ELRS handset in charge, disarmed - wheels stopped"), out.c_str());
}

void test_bridge_motor_enable_pin_follows_the_wheels() {
  // An aux pin in "armed" mode is a motor-enable line: on whenever the wheels
  // may move, whether the handset or the Bluetooth controller drives them.
  Fixture fx;
  cli(fx.b(), "set aux1.pin 25", fx.now);
  cli(fx.b(), "set aux1.mode armed", fx.now);
  cli(fx.b(), "save", fx.now);
  fx.reboot();
  TEST_ASSERT_FALSE(fx.b().aux(0).level);

  runWithPad(fx, 50, padReport(0, 0, 0));
  TEST_ASSERT_TRUE(fx.b().aux(0).level);

  eb::GamepadInput gone = padReport(0, 0, 0);
  gone.connected = false;
  runWithPad(fx, 10, gone);
  TEST_ASSERT_FALSE(fx.b().aux(0).level);

  Sticks s;
  run(fx.b(), fx.now, 100, &s);  // handset linked, disarmed
  TEST_ASSERT_FALSE(fx.b().aux(0).level);
  armRobot(fx, s);
  TEST_ASSERT_TRUE(fx.b().aux(0).level);
}

void test_bridge_refuses_saving_while_the_wheels_turn() {
  // Writing the flash stalls the loop, and the motor board keeps the last
  // speed it was given meanwhile.
  Fixture fx;
  runWithPad(fx, 50, padReport(0, 0, 0));
  runWithPad(fx, 300, padReport(0, -512, 0));
  TEST_ASSERT_TRUE(fx.b().drive().moving());
  TEST_ASSERT_EQUAL_STRING("ERR stop the wheels first\n", cli(fx.b(), "save", fx.now).c_str());
  TEST_ASSERT_FALSE(fx.platform.haveSaved);

  runWithPad(fx, 1000, padReport(0, 0, 0));
  TEST_ASSERT_FALSE(fx.b().drive().moving());
  TEST_ASSERT_TRUE(contains(cli(fx.b(), "save", fx.now), "saved"));
}

// ---------------------------------------------------------------------------
// Rough conditions

void test_bridge_ignores_noise_on_the_receiver_port() {
  Fixture fx;
  Sticks s;
  s.ticks[0] = 1811;
  uint32_t seed = 12345;
  for (int i = 0; i < 200; ++i) {
    uint8_t junk[37];
    for (size_t k = 0; k < sizeof junk; ++k) {
      seed = seed * 1664525u + 1013904223u;
      junk[k] = (uint8_t)(seed >> 24);
    }
    fx.b().feedCrsf(junk, sizeof junk, fx.now);
    run(fx.b(), fx.now, 10, &s);
  }
  // Real frames interleaved with the junk keep the link up with correct data.
  TEST_ASSERT_TRUE(fx.b().linkUp());
  const elrsb::State st = lastState(drain(fx.b(), eb::Bridge::kPortHost));
  TEST_ASSERT_EQUAL_INT16(1000, st.ch[0]);
  TEST_ASSERT_EQUAL_INT16(0, st.ch[1]);
}

void test_bridge_works_across_timer_wraparound() {
  Fixture fx(0xFFFFFE00u);
  Sticks s;
  s.ticks[0] = 1811;
  run(fx.b(), fx.now, 2000, &s);  // runs through the 32-bit wrap
  TEST_ASSERT_TRUE(fx.now < 0x10000u);
  TEST_ASSERT_TRUE(fx.b().linkUp());
  const Capture cap = drain(fx.b(), eb::Bridge::kPortHost);
  TEST_ASSERT_INT_WITHIN(3, 100, cap.count(elrsb::kMsgState));
  TEST_ASSERT_EQUAL_INT16(1000, lastState(cap).ch[0]);
  TEST_ASSERT_TRUE(drainCrsf(fx.b()).size() >= 15);  // telemetry kept its pace too
  run(fx.b(), fx.now, 600);
  TEST_ASSERT_FALSE(fx.b().linkUp());
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_bridge_streams_failsafe_state_before_any_link);
  RUN_TEST(test_bridge_usb_port_starts_as_cli);
  RUN_TEST(test_bridge_maps_channels_to_the_host);
  RUN_TEST(test_bridge_reports_failsafe_when_frames_stop);
  RUN_TEST(test_bridge_link_statistics_reach_the_host);

  RUN_TEST(test_bridge_cli_on_a_terminal);
  RUN_TEST(test_bridge_cli_over_binary_protocol);
  RUN_TEST(test_bridge_full_dump_fits_the_port_queue);
  RUN_TEST(test_bridge_settings_survive_reboot);
  RUN_TEST(test_bridge_reboot_command_asks_the_platform);

  RUN_TEST(test_bridge_refuses_disruptive_commands_while_armed);
  RUN_TEST(test_bridge_changing_arming_settings_never_arms_by_itself);
  RUN_TEST(test_bridge_arming_rules_apply_from_the_first_frame);

  RUN_TEST(test_bridge_cli_replies_carry_the_request_tag);
  RUN_TEST(test_bridge_set_stream_message);
  RUN_TEST(test_bridge_info_request_and_unknown_messages);
  RUN_TEST(test_bridge_text_stream);

  RUN_TEST(test_bridge_host_telemetry_reaches_the_receiver);
  RUN_TEST(test_bridge_reports_its_own_mode_without_a_host);
  RUN_TEST(test_bridge_stale_host_telemetry_expires);
  RUN_TEST(test_bridge_sends_no_telemetry_without_a_link);
  RUN_TEST(test_bridge_telemetry_can_be_switched_off);
  RUN_TEST(test_bridge_cli_telemetry_forms);
  RUN_TEST(test_bridge_bind_and_raw_crsf);
  RUN_TEST(test_bridge_forwards_other_crsf_frames_when_asked);

  RUN_TEST(test_bridge_aux_outputs);
  RUN_TEST(test_bridge_aux_pin_stays_controlled_when_its_settings_change);
  RUN_TEST(test_bridge_warns_about_shared_pins);

  RUN_TEST(test_bridge_hands_commands_to_the_motor_board);
  RUN_TEST(test_bridge_tells_the_host_who_drives);
  RUN_TEST(test_bridge_status_shows_the_wheels);
  RUN_TEST(test_bridge_motor_enable_pin_follows_the_wheels);
  RUN_TEST(test_bridge_refuses_saving_while_the_wheels_turn);

  RUN_TEST(test_bridge_ignores_noise_on_the_receiver_port);
  RUN_TEST(test_bridge_works_across_timer_wraparound);
  return UNITY_END();
}
