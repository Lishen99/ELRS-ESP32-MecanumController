// Turns decoded RC frames into the bridge's control state: link up/down,
// failsafe, arming, and the 16 mapped outputs.
#pragma once

#include <stdint.h>

#include "eb_config.h"

namespace eb {

class Rc {
 public:
  Rc();

  // A new RC frame arrived: 16 channel values in CRSF ticks.
  void onChannels(const uint16_t* ticks, uint32_t nowMs);

  // Re-evaluate link, arming and outputs. Call once before the first frame
  // (so the arming rules in `cfg` are in force from the start), after every
  // frame, and at least every few milliseconds so failsafe is detected on
  // time.
  void update(const Config& cfg, uint32_t nowMs);

  // Disarm, and stay disarmed until the operator arms again: the arm switch
  // has to be switched off, then on (or, with no arm switch, the handset has
  // to reconnect). Call when the arming settings change, so that a settings
  // change can disarm the robot but never arm it.
  void requireRearm();

  bool linkUp() const { return linkUp_; }
  bool failsafe() const { return !linkUp_; }
  bool armed() const { return armed_; }
  // The arm switch currently reads on.
  bool armSwitchOn() const { return armSwitch_; }
  // Arming is being held back until the operator acts, as described above.
  bool armGuarded() const { return armGuard_; }

  // Mapped output `i` (0-based): live value, or its failsafe value.
  int16_t out(int i) const { return out_[i]; }
  // Last received pulse width of CRSF channel `i` (0-based), microseconds.
  uint16_t rawUs(int i) const { return us_[i]; }

  bool everReceived() const { return haveFrame_; }
  // Milliseconds since the last RC frame, saturating at 0xFFFF (also the
  // value before any frame has arrived).
  uint16_t ageMs(uint32_t nowMs) const;
  uint32_t frames() const { return frames_; }
  uint16_t rateHz() const { return rateHz_; }
  uint16_t failsafeCount() const { return failsafeCount_; }

  // Scale one pulse width through an output's calibration.
  static int16_t mapOutput(const OutputCfg& cfg, uint16_t us);

 private:
  void updateArming(const Config& cfg, uint32_t nowMs);

  uint16_t us_[kNumOutputs];
  int16_t out_[kNumOutputs];
  int16_t live_[kNumOutputs];
  bool haveFrame_;      // at least one RC frame since boot
  bool fresh_;          // the latest frame is still within failsafe_ms
  bool ageSaturated_;   // the latest frame is older than ageMs() can express
  bool everLinked_;
  bool linkUp_;
  bool armed_;
  bool armSwitch_;      // the arm switch currently reads on
  bool armGuard_;       // arming is held back until the operator acts
  bool armOffTiming_;   // the switch reads off and armOffSinceMs_ is running
  uint32_t armOffSinceMs_;
  uint32_t lastFrameMs_;
  uint32_t frames_;
  uint32_t rateWindowMs_;
  uint32_t rateWindowFrames_;
  uint16_t rateHz_;
  uint16_t failsafeCount_;
};

}  // namespace eb
