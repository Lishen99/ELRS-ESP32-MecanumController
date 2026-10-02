// The wheels: the speed commands the bridge core works out go to the motor
// board.
//
// Which controller drives, the mecanum mix, the speed modes and the gentle
// speeding up and slowing down are all decided in the core (eb_drive.cpp),
// where they are tested. This module only carries the result to the
// Hiwonder board over I2C, every 10 ms, and reports whether it answered.
#pragma once

#include <stdint.h>

#include "eb_bridge.h"
#include "hiwonder_board.h"

class Motors {
 public:
  // Start the motor board and stop every motor. Call first thing in setup():
  // after a restart the board may still be driving the motors.
  void begin(const eb::Config& cfg);

  // Send the latest commands, if there are new ones. Takes about 0.6 ms
  // every 10 ms (one I2C message).
  void update(eb::Bridge& bridge);

  // Stop every motor now.
  void stop();

 private:
  HiwonderBoard board_;
};
