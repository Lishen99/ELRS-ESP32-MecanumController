// Driving a mecanum robot: the sticks of whichever controller is in charge
// become speed commands for the four wheels, changed gently.
//
// Who is in charge (drive_src auto, the default): the ELRS handset while its
// radio link is up - and then its arm switch decides whether the robot moves
// at all - otherwise a connected Bluetooth controller.
//
// The mix, the speed modes, the ramp and the motor order are taken from the
// robot's original PS4-controller firmware.
#pragma once

#include <stdint.h>

#include "eb_config.h"
#include "eb_gamepad.h"
#include "eb_rc.h"

namespace eb {

// How often the motor board gets a new set of commands.
const uint32_t kDrivePeriodMs = 10;

class Drive {
 public:
  enum Source {
    kNobody = 0,
    kElrs = 1,     // the ELRS handset
    kGamepad = 2   // the Bluetooth controller
  };

  Drive();

  // The latest report from the Bluetooth controller.
  void setGamepad(const GamepadInput& pad, uint32_t nowMs);

  // Work out what the wheels should do now. Returns true when a new set of
  // commands should go to the motor board: every kDrivePeriodMs, and at once
  // when the robot has to stop immediately.
  bool update(const Config& cfg, const Rc& rc, uint32_t nowMs);

  // Speed command for motor board channel i (0..3 = M1..M4), and the value
  // it is on its way to.
  int8_t command(int i) const { return command_[i]; }
  int8_t target(int i) const { return target_[i]; }

  Source source() const { return source_; }
  // The wheels follow the sticks of the controller in charge. False while
  // disarmed, while a Bluetooth controller that has just taken over waits
  // for its sticks to be centred, and while nobody is in charge.
  bool live() const { return live_; }
  // Some wheel is turning, or about to.
  bool moving() const;
  // The speed the controller in charge asks for, in percent of drive_max
  // (0 when the wheels are not live).
  int speedPercent() const { return percent_; }
  // A Bluetooth controller is connected and still reporting.
  bool gamepadConnected() const { return padFresh_; }
  // The Bluetooth controller is in charge, but its sticks have not yet been
  // seen centred.
  bool gamepadWaiting() const { return source_ == kGamepad && !padReady_; }

  // Scale a gamepad stick reading to -1000..1000, with the original
  // firmware's 10 % deadband.
  static int32_t padAxis(int16_t raw);

 private:
  // What the controller in charge asks for.
  struct Motion {
    int32_t forward;  // -1000..1000 each
    int32_t strafe;
    int32_t turn;
    int32_t percent;  // speed, percent of drive_max
  };

  Source pickSource(const Config& cfg, const Rc& rc) const;
  static Motion fromElrs(const Config& cfg, const Rc& rc);
  static Motion fromGamepad(const Config& cfg, const GamepadInput& pad);
  void aimAt(const Config& cfg, const Motion& m);
  static int8_t moveToward(int8_t current, int8_t target, const Config& cfg);

  GamepadInput pad_;
  uint32_t padMs_;
  bool padFresh_;   // connected, and heard from within bt_timeout_ms
  bool padReady_;   // its sticks have been centred since it took over
  Source source_;
  bool live_;
  int32_t percent_;
  bool started_;
  uint32_t lastStepMs_;
  int8_t target_[kNumMotors];
  int8_t command_[kNumMotors];
};

}  // namespace eb
