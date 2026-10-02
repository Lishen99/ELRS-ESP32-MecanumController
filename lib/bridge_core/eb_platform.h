// Everything the portable bridge core needs from the hardware it runs on.
// The ESP32 firmware and the desktop simulator each provide one of these.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace eb {

struct Config;

enum PinUse {
  kPinInput,   // digital input (UART receive)
  kPinOutput,  // digital output (UART transmit, LED, aux pins)
  kPinAnalog   // ADC input (battery voltage)
};

class Platform {
 public:
  virtual ~Platform() {}

  // Short board identifier reported to the host, e.g. "esp32".
  virtual const char* boardName() = 0;

  // Why the bridge last started ("power-on", "watchdog", ...), for `status`.
  virtual const char* resetReason() = 0;

  // Adjust the generic defaults (pins, LED type) for this board.
  virtual void boardDefaults(Config& cfg) = 0;

  // Whether `pin` may be assigned to a bridge function of this kind on this
  // board.
  virtual bool pinUsable(int pin, PinUse use) = 0;

  // Persistent storage for the saved configuration text. load returns false
  // when nothing has been saved yet.
  virtual bool loadConfigText(char* buf, size_t cap, size_t& len) = 0;
  virtual bool saveConfigText(const char* buf, size_t len) = 0;

  // Restart once pending output has been flushed.
  virtual void requestReboot() = 0;
};

}  // namespace eb
