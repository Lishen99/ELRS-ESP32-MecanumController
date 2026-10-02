// Holds the telemetry values supplied by the host and hands them to the
// receiver one CRSF frame at a time.
//
// The radio downlink is slow (tens of bytes per second at common settings),
// and the receiver only keeps the newest frame of each type, so frames are
// sent round-robin at a modest fixed pace instead of as fast as the host
// updates them. A value the host stops refreshing expires, so the handset
// shows "sensor lost" rather than a stale number.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <elrs_bridge_protocol.h>

#include "eb_config.h"

namespace eb {

class Telemetry {
 public:
  enum Item {
    kBattery,
    kGps,
    kAttitude,
    kMode,
    kVario,
    kBaro,
    kAirspeed,
    kRpm,
    kTemp,
    kItemCount
  };

  Telemetry();

  // Values supplied by the host.
  void setBattery(const elrsb::Battery& v, uint32_t nowMs);
  void setGps(const elrsb::Gps& v, uint32_t nowMs);
  void setAttitude(const elrsb::Attitude& v, uint32_t nowMs);
  void setMode(const char* text, size_t len, uint32_t nowMs);
  void setVario(float vspeedMps, uint32_t nowMs);
  void setBaro(const elrsb::Baro& v, uint32_t nowMs);
  void setAirspeed(float speedMps, uint32_t nowMs);
  void setRpm(const elrsb::Rpm& v, uint32_t nowMs);
  void setTemp(const elrsb::Temp& v, uint32_t nowMs);

  // Fallbacks produced by the bridge itself. They are only sent while the
  // host is not supplying the same item.
  void setLocalBattery(float volts, uint32_t nowMs);
  void setLocalMode(const char* text, uint32_t nowMs);

  // Drop values not refreshed within tlm_timeout_ms. Call every tick.
  void expire(const Config& cfg, uint32_t nowMs);

  // Build the next frame if one is due. Returns its length, or 0.
  size_t poll(uint8_t* frame, const Config& cfg, uint32_t nowMs);

  // 'h' = host value, 'l' = local fallback, '-' = nothing to send.
  char source(Item item) const;
  static const char* itemName(Item item);
  uint32_t framesSent() const { return sent_; }

 private:
  struct Slot {
    bool valid;
    uint32_t stampMs;
  };

  void touch(Slot& slot, uint32_t nowMs);
  size_t build(Item item, uint8_t* frame) const;

  Slot host_[kItemCount];
  Slot localBattery_;
  Slot localMode_;

  elrsb::Battery battery_;
  elrsb::Gps gps_;
  elrsb::Attitude attitude_;
  char mode_[elrsb::kMaxModeText + 1];
  float vario_;
  elrsb::Baro baro_;
  float airspeed_;
  elrsb::Rpm rpm_;
  elrsb::Temp temp_;

  float localVolts_;
  char localModeText_[elrsb::kMaxModeText + 1];

  int cursor_;
  bool started_;
  uint32_t lastSendMs_;
  uint32_t sent_;
};

}  // namespace eb
