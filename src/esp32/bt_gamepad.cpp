#include "bt_gamepad.h"

#include <Arduino.h>
#include <Bluepad32.h>

namespace {

// Controllers Bluepad32 has connected. The first gamepad among them is the
// one that drives; any others are ignored.
ControllerPtr controllers[BP32_MAX_GAMEPADS];

void onConnected(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (controllers[i] == nullptr) {
      controllers[i] = ctl;
      return;
    }
  }
}

void onDisconnected(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (controllers[i] == ctl) controllers[i] = nullptr;
  }
}

ControllerPtr activeGamepad() {
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    ControllerPtr ctl = controllers[i];
    if (ctl && ctl->isConnected() && ctl->isGamepad()) return ctl;
  }
  return nullptr;
}

int16_t axis(int32_t value) {
  return (int16_t)(value < -512 ? -512 : (value > 512 ? 512 : value));
}

enum Colour { kGreen, kAmber, kRed };

}  // namespace

void BtGamepad::begin(const eb::Config& cfg) {
  if (!cfg.btEnable) {
    // Bluetooth itself is started by the Bluepad32 core; just turn pairing off.
    BP32.enableNewBluetoothConnections(false);
    return;
  }
  BP32.setup(&onConnected, &onDisconnected);
  // A DualShock 4's touchpad would otherwise appear as a second device, a mouse.
  BP32.enableVirtualDevice(false);
  started_ = true;
}

void BtGamepad::update(eb::Bridge& bridge, uint32_t nowMs) {
  if (!started_) return;
  // A controller reports a few hundred times a second at most, so looking
  // once a millisecond catches every report.
  if (nowMs == lastPollMs_) return;
  lastPollMs_ = nowMs;

  BP32.update();
  ControllerPtr ctl = activeGamepad();
  if (!ctl) {
    if (wasConnected_) {
      eb::GamepadInput gone = {};
      bridge.setGamepad(gone, nowMs);
      wasConnected_ = false;
    }
    shownOn_ = nullptr;
    return;
  }
  wasConnected_ = true;

  if (ctl->hasData()) {
    eb::GamepadInput pad;
    pad.connected = true;
    pad.lx = axis(ctl->axisX());
    pad.ly = axis(ctl->axisY());
    pad.rx = axis(ctl->axisRX());
    pad.ry = axis(ctl->axisRY());
    pad.l2 = ctl->l2();
    pad.r2 = ctl->r2();
    bridge.setGamepad(pad, nowMs);
  }

  // The light bar shows who is in charge. Set only when that changes: each
  // change is a message over Bluetooth.
  const eb::Drive& drive = bridge.drive();
  const int colour = drive.source() != eb::Drive::kGamepad ? kRed
                     : drive.live()                        ? kGreen
                                                           : kAmber;
  if (ctl != shownOn_ || colour != shownColour_) {
    static const uint8_t kRgb[3][3] = {{0, 64, 0}, {64, 24, 0}, {64, 0, 0}};
    ctl->setColorLED(kRgb[colour][0], kRgb[colour][1], kRgb[colour][2]);
    shownOn_ = ctl;
    shownColour_ = colour;
  }
}
