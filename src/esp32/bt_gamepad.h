// The Bluetooth game controller: a PS4 DualShock 4, or anything else
// Bluepad32 supports.
//
// This module only passes the controller's reports to the bridge core, which
// decides what they do to the wheels (eb_drive.cpp), and shows on the
// controller's light bar who is in charge:
//   green   the controller is driving
//   amber   it is in charge, but waits for its sticks to be let go
//   red     the ELRS handset is in charge (or drive_src says not to use it)
//
// To pair a PS4 controller, hold Share and PS until the light bar flashes.
#pragma once

#include <stdint.h>

#include "eb_bridge.h"

class BtGamepad {
 public:
  // Start Bluetooth, unless bt_enable is off.
  void begin(const eb::Config& cfg);

  // Pass on the latest report, and keep the light bar up to date. Must not
  // block.
  void update(eb::Bridge& bridge, uint32_t nowMs);

 private:
  bool started_ = false;
  bool wasConnected_ = false;
  uint32_t lastPollMs_ = 0;
  const void* shownOn_ = nullptr;  // the controller the light bar colour was last set on
  int shownColour_ = -1;
};
