#include "battery_monitor.h"

#include <Arduino.h>

namespace {
const uint32_t kSamplePeriodMs = 100;
// Low-pass filter weight of each new sample; settles in about a second.
const float kFilterGain = 0.2f;
}  // namespace

void BatteryMonitor::begin(const eb::Config& cfg) { pin_ = cfg.vbatPin; }

void BatteryMonitor::update(eb::Bridge& bridge, uint32_t nowMs) {
  if (pin_ < 0) return;
  if ((uint32_t)(nowMs - lastSampleMs_) < kSamplePeriodMs) return;
  lastSampleMs_ = nowMs;

  // analogReadMilliVolts applies the chip's factory ADC calibration. The
  // divider ratio is read each time so `set vbat_scale` can be tuned live.
  const float sample = analogReadMilliVolts(pin_) * 0.001f * bridge.config().vbatScale;
  volts_ = haveReading_ ? volts_ + kFilterGain * (sample - volts_) : sample;
  haveReading_ = true;
  bridge.setLocalVbat(volts_, nowMs);
}
