#include "eb_telemetry.h"

#include <string.h>

#include "eb_crsf.h"

namespace eb {

Telemetry::Telemetry()
    : vario_(0), airspeed_(0), localVolts_(0), cursor_(kItemCount - 1), started_(false),
      lastSendMs_(0), sent_(0) {
  memset(host_, 0, sizeof host_);
  memset(&localBattery_, 0, sizeof localBattery_);
  memset(&localMode_, 0, sizeof localMode_);
  memset(&battery_, 0, sizeof battery_);
  memset(&gps_, 0, sizeof gps_);
  memset(&attitude_, 0, sizeof attitude_);
  memset(mode_, 0, sizeof mode_);
  memset(&baro_, 0, sizeof baro_);
  memset(&rpm_, 0, sizeof rpm_);
  memset(&temp_, 0, sizeof temp_);
  memset(localModeText_, 0, sizeof localModeText_);
}

void Telemetry::touch(Slot& slot, uint32_t nowMs) {
  slot.valid = true;
  slot.stampMs = nowMs;
}

void Telemetry::setBattery(const elrsb::Battery& v, uint32_t nowMs) {
  battery_ = v;
  touch(host_[kBattery], nowMs);
}

void Telemetry::setGps(const elrsb::Gps& v, uint32_t nowMs) {
  gps_ = v;
  touch(host_[kGps], nowMs);
}

void Telemetry::setAttitude(const elrsb::Attitude& v, uint32_t nowMs) {
  attitude_ = v;
  touch(host_[kAttitude], nowMs);
}

void Telemetry::setMode(const char* text, size_t len, uint32_t nowMs) {
  if (len == 0) {
    // An empty string hands the mode line back to the bridge's own status.
    host_[kMode].valid = false;
    return;
  }
  if (len > elrsb::kMaxModeText) len = elrsb::kMaxModeText;
  memcpy(mode_, text, len);
  mode_[len] = '\0';
  touch(host_[kMode], nowMs);
}

void Telemetry::setVario(float vspeedMps, uint32_t nowMs) {
  vario_ = vspeedMps;
  touch(host_[kVario], nowMs);
}

void Telemetry::setBaro(const elrsb::Baro& v, uint32_t nowMs) {
  baro_ = v;
  touch(host_[kBaro], nowMs);
}

void Telemetry::setAirspeed(float speedMps, uint32_t nowMs) {
  airspeed_ = speedMps;
  touch(host_[kAirspeed], nowMs);
}

void Telemetry::setRpm(const elrsb::Rpm& v, uint32_t nowMs) {
  rpm_ = v;
  touch(host_[kRpm], nowMs);
}

void Telemetry::setTemp(const elrsb::Temp& v, uint32_t nowMs) {
  temp_ = v;
  touch(host_[kTemp], nowMs);
}

void Telemetry::setLocalBattery(float volts, uint32_t nowMs) {
  localVolts_ = volts;
  touch(localBattery_, nowMs);
}

void Telemetry::setLocalMode(const char* text, uint32_t nowMs) {
  strncpy(localModeText_, text, sizeof localModeText_ - 1);
  localModeText_[sizeof localModeText_ - 1] = '\0';
  touch(localMode_, nowMs);
}

void Telemetry::expire(const Config& cfg, uint32_t nowMs) {
  // Expiry is latched here instead of being derived from the timestamps at
  // send time, so a wrapped millisecond counter cannot revive an old value.
  for (int i = 0; i < kItemCount; ++i) {
    if (host_[i].valid && (uint32_t)(nowMs - host_[i].stampMs) > cfg.tlmTimeoutMs) {
      host_[i].valid = false;
    }
  }
  if (localBattery_.valid && (uint32_t)(nowMs - localBattery_.stampMs) > cfg.tlmTimeoutMs) {
    localBattery_.valid = false;
  }
  if (localMode_.valid && (uint32_t)(nowMs - localMode_.stampMs) > cfg.tlmTimeoutMs) {
    localMode_.valid = false;
  }
}

size_t Telemetry::build(Item item, uint8_t* frame) const {
  if (!host_[item].valid) {
    if (item == kBattery && localBattery_.valid) {
      return crsf::buildBattery(frame, localVolts_, 0.0f, 0.0f, 0);
    }
    if (item == kMode && localMode_.valid) return crsf::buildFlightMode(frame, localModeText_);
    return 0;
  }
  switch (item) {
    case kBattery:
      return crsf::buildBattery(frame, battery_.volts, battery_.amps, battery_.usedMah,
                                battery_.remainingPct);
    case kGps:
      return crsf::buildGps(frame, gps_.latE7, gps_.lonE7, gps_.speedMps, gps_.headingDeg,
                            gps_.altM, gps_.sats);
    case kAttitude:
      return crsf::buildAttitude(frame, attitude_.roll, attitude_.pitch, attitude_.yaw);
    case kMode:
      return crsf::buildFlightMode(frame, mode_);
    case kVario:
      return crsf::buildVario(frame, vario_);
    case kBaro:
      return crsf::buildBaroAltitude(frame, baro_.altM, baro_.vspeedMps);
    case kAirspeed:
      return crsf::buildAirspeed(frame, airspeed_);
    case kRpm:
      return crsf::buildRpm(frame, rpm_.source, rpm_.rpm, rpm_.count);
    case kTemp:
      return crsf::buildTemperature(frame, temp_.source, temp_.degC, temp_.count);
    default:
      return 0;
  }
}

size_t Telemetry::poll(uint8_t* frame, const Config& cfg, uint32_t nowMs) {
  if (cfg.tlmPeriodMs == 0) return 0;
  if (started_ && (uint32_t)(nowMs - lastSendMs_) < cfg.tlmPeriodMs) return 0;
  for (int step = 1; step <= kItemCount; ++step) {
    const int item = (cursor_ + step) % kItemCount;
    const size_t len = build((Item)item, frame);
    if (len) {
      cursor_ = item;
      started_ = true;
      lastSendMs_ = nowMs;
      ++sent_;
      return len;
    }
  }
  return 0;
}

char Telemetry::source(Item item) const {
  if (host_[item].valid) return 'h';
  if (item == kBattery && localBattery_.valid) return 'l';
  if (item == kMode && localMode_.valid) return 'l';
  return '-';
}

const char* Telemetry::itemName(Item item) {
  switch (item) {
    case kBattery: return "bat";
    case kGps: return "gps";
    case kAttitude: return "att";
    case kMode: return "mode";
    case kVario: return "vario";
    case kBaro: return "alt";
    case kAirspeed: return "aspd";
    case kRpm: return "rpm";
    case kTemp: return "temp";
    default: return "?";
  }
}

}  // namespace eb
