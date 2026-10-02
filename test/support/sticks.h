// The handset, as the tests operate it: sixteen channel positions and the
// CRSF frame a receiver would send for them.
#pragma once

#include <stdint.h>

#include <vector>

#include "eb_crsf.h"

// Channel positions in CRSF ticks: 992 is a centred stick, 191 a switch that
// is off, 1792 a switch that is on, 172 and 1811 the ends of a stick.
struct Sticks {
  uint16_t ticks[16];
  Sticks() {
    for (int i = 0; i < 16; ++i) ticks[i] = i < 4 ? 992 : 191;  // centred, switches low
  }
};

// The bridge lifts its arm guard once the arm switch has read off for half a
// second. Tests hold it off a little longer than that.
const uint32_t kArmOffHold = 600;

inline std::vector<uint8_t> rcFrame(const Sticks& s) {
  uint8_t payload[22];
  eb::crsf::packChannels(s.ticks, payload);
  uint8_t frame[64];
  const size_t n = eb::crsf::buildFrame(frame, eb::crsf::kTypeRcChannels, payload, sizeof payload);
  return std::vector<uint8_t>(frame, frame + n);
}
