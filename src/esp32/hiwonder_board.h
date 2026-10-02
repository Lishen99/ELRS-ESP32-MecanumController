// The Hiwonder 4-channel encoder motor driver, on I2C.
//
// The board runs its own closed-loop speed control. The ESP32 tells it once
// which motors are fitted, and after that, every 10 ms, how fast each of its
// four channels (M1..M4) should turn.
//
// The board keeps turning the motors at the last speed it was given, even if
// the ESP32 restarts or crashes. So begin() registers emergencyStop() with the
// output guard, which sends a stop by driving the I2C lines by hand - that
// works even from the crash handler.
#pragma once

#include <stdint.h>

class HiwonderBoard {
 public:
  // Start I2C on these pins, stop every motor, and tell the board which
  // motors it has. Returns false if the board did not answer.
  bool begin(int sdaPin, int sclPin, uint8_t address, uint8_t motorType,
             uint8_t encoderPolarity);

  // Closed-loop speed for channels M1..M4, in the board's units (encoder
  // pulses per 10 ms). Returns false if the board did not acknowledge.
  bool setSpeeds(const int8_t (&speeds)[4]);

  // Stop every motor at once, from any context - a crash handler included.
  // Does nothing until begin() has run.
  static void emergencyStop();

 private:
  bool writeRegister(uint8_t reg, const uint8_t* data, uint8_t len);

  uint8_t address_ = 0;
  bool started_ = false;
};
