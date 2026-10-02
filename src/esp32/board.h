// The ESP32 as the bridge core sees it: which board this is, which pins it
// may hand out, where the settings are kept, and how to restart. Also owns
// the watchdog that restarts the chip if the main loop stalls.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "eb_config.h"
#include "eb_platform.h"

class Esp32Board : public eb::Platform {
 public:
  const char* boardName() override;
  const char* resetReason() override;
  void boardDefaults(eb::Config& cfg) override;
  bool pinUsable(int pin, eb::PinUse use) override;
  bool loadConfigText(char* buf, size_t cap, size_t& len) override;
  bool saveConfigText(const char* buf, size_t len) override;
  void requestReboot() override;

  // Restart the chip if the main loop stops running for about a second. Call
  // at the end of setup(), then feedWatchdog() on every pass of the loop.
  void startWatchdog();
  void feedWatchdog();

  // True once a requested restart should happen. It is held back briefly so
  // the reply to the `reboot` command can leave the serial port first.
  bool rebootDue(uint32_t nowMs) const;

 private:
  bool rebootRequested_ = false;
  uint32_t rebootRequestedMs_ = 0;
};
