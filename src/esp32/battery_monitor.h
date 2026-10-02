// Battery voltage measured through a resistor divider on an ADC pin (settings
// vbat_pin and vbat_scale). The reading is given to the bridge, which sends
// it to the handset and reports it to the host.
#pragma once

#include <stdint.h>

#include "eb_bridge.h"

class BatteryMonitor {
 public:
  void begin(const eb::Config& cfg);
  void update(eb::Bridge& bridge, uint32_t nowMs);

 private:
  int8_t pin_ = -1;
  float volts_ = 0.0f;
  bool haveReading_ = false;
  uint32_t lastSampleMs_ = 0;
};
