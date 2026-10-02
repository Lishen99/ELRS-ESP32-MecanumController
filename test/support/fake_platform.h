// The hardware, as far as the bridge core can tell: a stand-in for the ESP32
// in tests. Settings are "saved" to a string instead of flash.
#pragma once

#include <cstring>
#include <string>

#include "eb_config.h"
#include "eb_platform.h"

class FakePlatform : public eb::Platform {
 public:
  FakePlatform() : haveSaved(false), rebootRequests(0) {}

  const char* boardName() override { return "test"; }
  const char* resetReason() override { return "power-on"; }
  void boardDefaults(eb::Config&) override {}
  // The classic ESP32's rules: 1/3 are the USB serial port, 6-11 the flash,
  // 34-39 are input only, and (simplified) 32-39 are the ADC pins.
  bool pinUsable(int pin, eb::PinUse use) override {
    if (pin < 0 || pin > 39) return false;
    if (pin == 1 || pin == 3) return false;
    if (pin >= 6 && pin <= 11) return false;
    if (use == eb::kPinOutput && pin >= 34) return false;
    if (use == eb::kPinAnalog && pin < 32) return false;
    return true;
  }
  bool loadConfigText(char* buf, size_t cap, size_t& len) override {
    if (!haveSaved || saved.size() > cap) return false;
    memcpy(buf, saved.data(), saved.size());
    len = saved.size();
    return true;
  }
  bool saveConfigText(const char* buf, size_t len) override {
    saved.assign(buf, len);
    haveSaved = true;
    return true;
  }
  void requestReboot() override { ++rebootRequests; }

  std::string saved;
  bool haveSaved;
  int rebootRequests;
};
