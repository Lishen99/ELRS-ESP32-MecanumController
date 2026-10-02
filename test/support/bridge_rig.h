// A bridge on the test bench: a fake receiver feeding it RC frames on one
// side, and on the other what a host would see on each port.
//
// Time is a counter the tests advance, a millisecond at a time, so every test
// is exact and repeatable.
#pragma once

#include <unity.h>

#include <array>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "eb_bridge.h"
#include "fake_platform.h"
#include "sticks.h"
#include "text.h"

// ---------------------------------------------------------------------------
// What a host port has sent

struct HostFrame {
  uint8_t type;
  std::vector<uint8_t> payload;
};

struct Capture {
  std::vector<HostFrame> frames;
  std::string text;

  const HostFrame* last(uint8_t type) const {
    for (size_t i = frames.size(); i > 0; --i) {
      if (frames[i - 1].type == type) return &frames[i - 1];
    }
    return nullptr;
  }
  int count(uint8_t type) const {
    int n = 0;
    for (size_t i = 0; i < frames.size(); ++i) n += frames[i].type == type;
    return n;
  }
};

// The receiving end of one host port: splits what arrives into binary frames
// and plain text, as a host would.
struct Tap {
  elrsb::FrameParser parser;
  Capture cap;

  void take(const uint8_t* data, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      const elrsb::FeedResult r = parser.feed(data[i]);
      if (r == elrsb::kFeedFrame) {
        HostFrame f;
        f.type = parser.type();
        f.payload.assign(parser.payload(), parser.payload() + parser.length());
        cap.frames.push_back(f);
      } else if (r == elrsb::kFeedText) {
        cap.text.push_back((char)data[i]);
      }
    }
  }
};

// One tap per host port of every bridge a test creates.
inline std::map<const eb::Bridge*, std::array<Tap, 2> >& taps() {
  static std::map<const eb::Bridge*, std::array<Tap, 2> > all;
  return all;
}

// Carry away everything a port has ready, the way a fast UART would.
inline void pump(eb::Bridge& bridge, eb::Bridge::PortId id) {
  Tap& tap = taps()[&bridge][id];
  uint8_t buf[256];
  size_t n;
  while ((n = bridge.readPortTx(id, buf, sizeof buf)) > 0) tap.take(buf, n);
}

// Everything a host port has sent since the last call.
inline Capture drain(eb::Bridge& bridge, eb::Bridge::PortId id) {
  pump(bridge, id);
  Tap& tap = taps()[&bridge][id];
  TEST_ASSERT_EQUAL_UINT32(0, tap.parser.errors());
  Capture out;
  std::swap(out, tap.cap);
  return out;
}

inline elrsb::State lastState(const Capture& cap) {
  const HostFrame* f = cap.last(elrsb::kMsgState);
  TEST_ASSERT_NOT_NULL_MESSAGE(f, "no state frame");
  elrsb::State s;
  TEST_ASSERT_TRUE(elrsb::unpackState(f->payload.data(), f->payload.size(), s));
  return s;
}

// A UART too slow to keep up: a 128-byte hardware FIFO, like the ESP32's,
// emptied at a fixed number of bytes per millisecond. Bytes reach the tap when
// they leave the FIFO, so a test sees what the host sees, when it sees it.
struct SlowWire {
  eb::Bridge& bridge;
  eb::Bridge::PortId id;
  double bytesPerMs;
  double credit;
  std::deque<uint8_t> fifo;
  Tap tap;

  SlowWire(eb::Bridge& b, eb::Bridge::PortId port, double baud)
      : bridge(b), id(port), bytesPerMs(baud / 10000.0), credit(0) {}

  // One millisecond of UART activity.
  void step() {
    uint8_t buf[128];
    const size_t room = sizeof buf - fifo.size();
    const size_t n = room ? bridge.readPortTx(id, buf, room) : 0;
    fifo.insert(fifo.end(), buf, buf + n);

    credit += bytesPerMs;
    while (credit >= 1.0 && !fifo.empty()) {
      const uint8_t byte = fifo.front();
      fifo.pop_front();
      tap.take(&byte, 1);
      credit -= 1.0;
    }
    if (fifo.empty() && credit > 1.0) credit = 1.0;  // an idle line saves nothing up
  }
};

// ---------------------------------------------------------------------------
// What the bridge has sent to the receiver

// Every complete CRSF frame the bridge queued for the receiver: the type
// byte, then the payload.
inline std::vector<std::vector<uint8_t> > drainCrsf(eb::Bridge& bridge) {
  std::vector<std::vector<uint8_t> > frames;
  eb::crsf::Parser parser;
  uint8_t buf[256];
  size_t n;
  while ((n = bridge.readCrsfTx(buf, sizeof buf)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (!parser.feed(buf[i])) continue;
      const eb::crsf::Frame& f = parser.frame();
      std::vector<uint8_t> frame;
      frame.push_back(f.type);
      frame.insert(frame.end(), f.payload, f.payload + f.len);
      frames.push_back(frame);
    }
  }
  TEST_ASSERT_EQUAL_UINT32(0, parser.errors());
  return frames;
}

inline const std::vector<uint8_t>* findCrsf(const std::vector<std::vector<uint8_t> >& frames,
                                            uint8_t type) {
  for (size_t i = 0; i < frames.size(); ++i) {
    if (frames[i][0] == type) return &frames[i];
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// Driving the bridge

inline void sendRc(eb::Bridge& bridge, const Sticks& s, uint32_t now) {
  const std::vector<uint8_t> f = rcFrame(s);
  bridge.feedCrsf(f.data(), f.size(), now);
}

// Advance time one millisecond at a time, optionally feeding an RC frame
// every 10 ms the way a 100 Hz link would. The host ports are emptied every
// millisecond, as a UART that keeps up would; pass carryAway = false to model
// a port nobody is listening to.
inline void run(eb::Bridge& bridge, uint32_t& now, uint32_t ms, const Sticks* sticks = nullptr,
                bool carryAway = true) {
  for (uint32_t i = 0; i < ms; ++i) {
    ++now;
    if (sticks && now % 10 == 0) sendRc(bridge, *sticks, now);
    bridge.tick(now);
    if (carryAway) {
      pump(bridge, eb::Bridge::kPortUsb);
      pump(bridge, eb::Bridge::kPortHost);
    }
  }
}

inline void sendHostFrame(eb::Bridge& bridge, eb::Bridge::PortId id, uint8_t type,
                          const uint8_t* payload, size_t len, uint32_t now) {
  uint8_t frame[elrsb::kMaxFrame];
  const size_t n = elrsb::encodeFrame(frame, type, payload, len);
  TEST_ASSERT_TRUE(n > 0);
  bridge.feedPort(id, frame, n, now);
}

inline void typeLine(eb::Bridge& bridge, eb::Bridge::PortId id, const char* line, uint32_t now) {
  bridge.feedPort(id, (const uint8_t*)line, strlen(line), now);
  bridge.feedPort(id, (const uint8_t*)"\r\n", 2, now);
}

// Put together the text of a CLI reply from the frames in a capture, checking
// that every chunk carries the expected tag and that the reply is terminated.
inline std::string cliReplyText(const Capture& cap, uint8_t tag) {
  std::string text;
  bool sawLast = false;
  for (size_t i = 0; i < cap.frames.size(); ++i) {
    if (cap.frames[i].type != elrsb::kMsgCliReply) continue;
    TEST_ASSERT_FALSE(sawLast);
    const std::vector<uint8_t>& p = cap.frames[i].payload;
    TEST_ASSERT_TRUE(p.size() >= 2);
    TEST_ASSERT_EQUAL_UINT8(tag, p[1]);
    text.append(p.begin() + 2, p.end());
    sawLast = (p[0] & elrsb::kCliReplyLast) != 0;
  }
  TEST_ASSERT_TRUE_MESSAGE(sawLast, "CLI reply was not terminated");
  return text;
}

// Send a CLI command through the binary protocol: a tag byte, then the line.
inline void sendCli(eb::Bridge& bridge, eb::Bridge::PortId id, const char* line, uint8_t tag,
                    uint32_t now) {
  std::vector<uint8_t> payload(1, tag);
  payload.insert(payload.end(), line, line + strlen(line));
  sendHostFrame(bridge, id, elrsb::kMsgCli, payload.data(), payload.size(), now);
}

// Run a CLI command through the binary protocol and return its text output.
inline std::string cli(eb::Bridge& bridge, const char* line, uint32_t now) {
  static uint8_t tag = 0;
  ++tag;
  drain(bridge, eb::Bridge::kPortHost);
  sendCli(bridge, eb::Bridge::kPortHost, line, tag, now);
  return cliReplyText(drain(bridge, eb::Bridge::kPortHost), tag);
}

// How many settings there are: `dump` and `get` print one line for each.
inline int settingCount() {
  struct Counter {
    static void visit(void* count, const char*, const eb::ParamRef&) { ++*(int*)count; }
  };
  eb::Config cfg;
  eb::configDefaults(cfg);
  int count = 0;
  eb::paramForEach(cfg, &Counter::visit, &count);
  return count;
}

// ---------------------------------------------------------------------------
// The bench itself

struct Fixture {
  FakePlatform platform;
  std::unique_ptr<eb::Bridge> bridge;
  uint32_t now;

  explicit Fixture(uint32_t start = 1000) : bridge(new eb::Bridge(platform)), now(start) {
    taps().erase(bridge.get());  // a fresh bridge at a reused address starts with empty taps
    bridge->begin(now);
  }
  // Power-cycle the bridge: a new one, starting from whatever was saved.
  void reboot() {
    bridge.reset(new eb::Bridge(platform));
    taps().erase(bridge.get());
    bridge->begin(now);
  }
  eb::Bridge& b() { return *bridge; }
};

// Bring the link up with the arm switch off for long enough to lift the arm
// guard, then switch it on. Leaves `sticks` with the switch on.
inline void armRobot(Fixture& fx, Sticks& sticks) {
  sticks.ticks[4] = 191;
  run(fx.b(), fx.now, kArmOffHold, &sticks);
  sticks.ticks[4] = 1792;
  run(fx.b(), fx.now, 30, &sticks);
  TEST_ASSERT_TRUE_MESSAGE(fx.b().armed(), "the robot did not arm");
}
