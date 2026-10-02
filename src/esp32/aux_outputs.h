// The optional aux pins: servo/ESC pulses or plain digital levels that follow
// a channel, the arming state or the link state (settings aux1..aux4).
//
// The bridge core decides what each pin should be doing; this module only
// applies that to the hardware.
//
// Servo pulses come from the low-speed LEDC channels 4-7 on low-speed timer
// 3 - channels 12-15 in Arduino's ledc numbering. Any other PWM on this chip
// must keep off those (Arduino channels 0-11 are free), and must not use
// analogWrite(), which picks its channels from the top.
#pragma once

#include <stdint.h>

#include "eb_bridge.h"

class AuxOutputs {
 public:
  // Attach the configured pins and put each in its safe state.
  void begin(const eb::Bridge& bridge);

  // Apply whatever the bridge wants each pin to be doing now.
  void update(const eb::Bridge& bridge);

 private:
  void preloadPulses(const eb::AuxCommand* safe);
  void waitForLowPart(const eb::AuxCommand* safe) const;
  void apply(int index, const eb::AuxCommand& cmd);
  uint32_t duty(uint16_t pulseUs) const;

  uint16_t pwmHz_ = 50;
  uint8_t pwmBits_ = 14;
  int32_t applied_[eb::kNumAux] = {};  // last value written, to write only on change
};
