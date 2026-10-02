// A Bluetooth game controller, as the platform layer hands it to the core.
//
// The values are the ones Bluepad32 reports for a PS4 controller, so the
// platform layer only copies them; what they mean for the robot is decided in
// eb_drive.cpp, where it can be tested.
#pragma once

#include <stdint.h>

namespace eb {

struct GamepadInput {
  bool connected;  // a controller is paired and talking
  int16_t lx;      // left stick: -512 (left) .. 511 (right)
  int16_t ly;      // left stick: -512 (up) .. 511 (down)
  int16_t rx;      // right stick, the same
  int16_t ry;
  bool l2;         // L2 held down
  bool r2;         // R2 held down
};

}  // namespace eb
