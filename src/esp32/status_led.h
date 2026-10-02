// Status LED.
//
//   slow blink   no controller: neither the ELRS handset nor a Bluetooth one
//   steady       a controller is connected, but the wheels are not live
//                (disarmed, or waiting for the sticks to be let go)
//   fast blink   the wheels follow the sticks
//
// On an addressable RGB LED the three states are also red, green and blue.
#pragma once

#include <stdint.h>

#include "eb_bridge.h"

class StatusLed {
 public:
  void begin(const eb::Config& cfg);
  void update(const eb::Bridge& bridge, uint32_t nowMs);

 private:
  void write(bool on, uint8_t red, uint8_t green, uint8_t blue);

  int8_t pin_ = -1;
  uint8_t mode_ = eb::kLedActiveHigh;
  int shown_ = -1;  // what the LED currently displays, so it is only written on change
};
