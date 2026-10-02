// Bridge: what each aux pin should be doing right now.
//
// Only the decision is made here, so it can be tested on the desktop; the
// platform layer turns it into servo pulses or a pin level.
#include "eb_bridge.h"

namespace eb {

AuxCommand Bridge::aux(int index) const {
  AuxCommand cmd;
  cmd.kind = AuxCommand::kDisabled;
  cmd.pin = -1;
  cmd.pulseUs = 0;
  cmd.level = false;
  if (index < 0 || index >= kNumAux) return cmd;

  // Pin and mode are the ones the output was started with. Changing those
  // settings takes a reboot; until then the pin that is physically attached
  // keeps being driven (and keeps going safe on failsafe).
  const AuxBoot& boot = auxBoot_[index];
  if (boot.pin < 0) return cmd;
  cmd.pin = boot.pin;

  const AuxCfg& a = cfg_.aux[index];
  const int src = clampT<int>(a.src, 1, kNumOutputs) - 1;
  // A pin follows its channel only while the link is up and, if gated, armed.
  const bool active = rc_.linkUp() && (!a.gate || rc_.armed());
  const bool invert = a.inv != 0;

  switch (boot.mode) {
    case kAuxServo: {
      cmd.kind = AuxCommand::kPulse;
      if (!active) {
        cmd.pulseUs = a.fsUs;
        break;
      }
      const int32_t value = rc_.out(src);
      const int32_t span = (int32_t)a.maxUs - (int32_t)a.minUs;
      const int32_t us = cfg_.out[src].type == kOutUnipolar
                             ? a.minUs + divRound(value * span, 1000)
                             : a.minUs + divRound((value + 1000) * span, 2000);
      const int32_t lo = a.minUs < a.maxUs ? a.minUs : a.maxUs;
      const int32_t hi = a.minUs < a.maxUs ? a.maxUs : a.minUs;
      cmd.pulseUs = (uint16_t)clampT<int32_t>(us, lo, hi);
      break;
    }
    case kAuxSwitch:
      cmd.kind = AuxCommand::kLevel;
      cmd.level = (active && rc_.out(src) >= 500) != invert;
      break;
    case kAuxArmed:
      // A motor-enable line: on whenever the wheels may move, whichever
      // controller is driving them.
      cmd.kind = AuxCommand::kLevel;
      cmd.level = drive_.live() != invert;
      break;
    case kAuxLink:
      cmd.kind = AuxCommand::kLevel;
      cmd.level = rc_.linkUp() != invert;
      break;
    default:
      break;
  }
  return cmd;
}

}  // namespace eb
