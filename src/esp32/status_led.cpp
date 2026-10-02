#include "status_led.h"

#include <Arduino.h>

namespace {
enum Pattern { kWaiting = 0, kConnected = 1, kLive = 2 };
}  // namespace

void StatusLed::begin(const eb::Config& cfg) {
  pin_ = cfg.ledPin;
  mode_ = cfg.ledMode;
  if (pin_ >= 0 && mode_ != eb::kLedRgb) pinMode(pin_, OUTPUT);
}

void StatusLed::update(const eb::Bridge& bridge, uint32_t nowMs) {
  if (pin_ < 0) return;

  const eb::Drive& drive = bridge.drive();
  Pattern pattern;
  bool on;
  if (drive.live()) {
    pattern = kLive;
    on = (nowMs % 200) < 100;
  } else if (bridge.linkUp() || drive.gamepadConnected()) {
    pattern = kConnected;
    on = true;
  } else {
    pattern = kWaiting;
    on = (nowMs % 1000) < 500;
  }

  const int showing = pattern * 2 + (on ? 1 : 0);
  if (showing == shown_) return;
  shown_ = showing;

  switch (pattern) {
    case kWaiting: write(on, 24, 0, 0); break;
    case kConnected: write(on, 0, 24, 0); break;
    case kLive: write(on, 0, 0, 32); break;
  }
}

void StatusLed::write(bool on, uint8_t red, uint8_t green, uint8_t blue) {
  if (mode_ == eb::kLedRgb) {
    if (on) {
      neopixelWrite(pin_, red, green, blue);
    } else {
      neopixelWrite(pin_, 0, 0, 0);
    }
  } else {
    const bool level = on != (mode_ == eb::kLedActiveLow);
    digitalWrite(pin_, level ? HIGH : LOW);
  }
}
