// Host-side client for the ELRS Bridge, for microcontrollers and C++ programs.
//
// Header-only, C++11, no dynamic memory. Works on Teensy, STM32, ESP32, AVR
// and desktop compilers. On Arduino-style boards use ElrsBridgeSerial:
//
//   #include <ElrsBridgeClient.h>
//   ElrsBridgeSerial<HardwareSerial> bridge(Serial1);
//
//   void setup() { Serial1.begin(115200); }
//   void loop() {
//     bridge.update();
//     const elrsb::State s = bridge.state();
//     if (s.motorsLive()) { /* the robot is being driven */ }
//     bridge.sendBattery(12.4f);
//   }
//
// Wire the bridge's host TX pin to this board's RX pin (and its RX to this
// board's TX if you want to send telemetry), plus ground. The bridge's pins
// are 3.3 V and NOT 5 V tolerant.
//
// Safety: state(), armed() and ch() never return old data. If no update has
// arrived for staleAfterMs (250 ms by default) they report failsafe: every
// channel zero and armed() false.
#ifndef ELRS_BRIDGE_CLIENT_H
#define ELRS_BRIDGE_CLIENT_H

#include "elrs_bridge_protocol.h"

class ElrsBridgeClient {
 public:
  ElrsBridgeClient()
      : fresh_(false), haveLink_(false), stateMs_(0), staleAfterMs_(250), states_(0) {
    memset(&state_, 0, sizeof state_);
    memset(&link_, 0, sizeof link_);
  }

  // How long an update stays valid without a newer one arriving.
  void setStaleAfter(uint16_t ms) { staleAfterMs_ = ms; }

  // Feed one received byte; `nowMs` is this board's millisecond clock.
  // Returns true when the byte completed a new state update.
  bool feed(uint8_t byte, uint32_t nowMs) {
    if (parser_.feed(byte) != elrsb::kFeedFrame) return false;
    if (parser_.type() == elrsb::kMsgState) {
      elrsb::State s;
      if (!elrsb::unpackState(parser_.payload(), parser_.length(), s)) return false;
      state_ = s;
      stateMs_ = nowMs;
      fresh_ = true;
      ++states_;
      return true;
    }
    if (parser_.type() == elrsb::kMsgLink) {
      haveLink_ = elrsb::unpackLink(parser_.payload(), parser_.length(), link_) || haveLink_;
    }
    return false;
  }

  // True while updates are arriving. Call regularly: staleness is latched, so
  // a wrapped millisecond counter can never make old data look fresh again.
  bool fresh(uint32_t nowMs) {
    if (fresh_ && (uint32_t)(nowMs - stateMs_) > staleAfterMs_) fresh_ = false;
    return fresh_;
  }

  // The latest update, or a failsafe state if it has gone stale.
  elrsb::State state(uint32_t nowMs) {
    if (fresh(nowMs)) return state_;
    elrsb::State s;
    memset(&s, 0, sizeof s);
    s.flags = elrsb::kFlagFailsafe;
    s.ageMs = 0xFFFF;
    s.count = elrsb::kMaxChannels;
    return s;
  }

  bool linkUp(uint32_t nowMs) { return fresh(nowMs) && state_.linkUp(); }
  bool armed(uint32_t nowMs) { return fresh(nowMs) && state_.armed(); }

  // Output `number` (1-based, like the handset's CH1..CH16) as an integer:
  // -1000..1000 for bipolar outputs, 0..1000 for unipolar. Zero when stale.
  int16_t value(uint8_t number, uint32_t nowMs) {
    if (!fresh(nowMs) || number < 1 || number > state_.count || number > elrsb::kMaxChannels) {
      return 0;
    }
    return state_.ch[number - 1];
  }

  // The same, scaled to -1.0..1.0 or 0.0..1.0.
  float ch(uint8_t number, uint32_t nowMs) {
    return (float)value(number, nowMs) / (float)elrsb::kOutputScale;
  }

  // Link statistics, or a null pointer until the first one has arrived.
  const elrsb::Link* link() const { return haveLink_ ? &link_ : (const elrsb::Link*)0; }

  // Number of state updates received (wraps).
  uint32_t stateCount() const { return states_; }
  // Frames rejected for a bad checksum: a rising count means a noisy line or
  // a baud rate mismatch.
  uint32_t errors() const { return parser_.errors(); }

  // --- Frames to send to the bridge ---------------------------------------
  // Each function writes one complete frame into `out`, which must hold
  // elrsb::kMaxFrame bytes, and returns the number of bytes to transmit.

  static size_t packBattery(uint8_t* out, float volts, float amps = 0.0f, float usedMah = 0.0f,
                            uint8_t remainingPct = 0) {
    elrsb::Battery b;
    b.volts = volts;
    b.amps = amps;
    b.usedMah = usedMah;
    b.remainingPct = remainingPct;
    uint8_t p[elrsb::kBatterySize];
    return elrsb::encodeFrame(out, elrsb::kMsgTlmBattery, p, elrsb::packBattery(p, b));
  }

  // Latitude/longitude in degrees * 1e7, speed in m/s, heading in degrees,
  // altitude in metres.
  static size_t packGps(uint8_t* out, int32_t latE7, int32_t lonE7, float speedMps = 0.0f,
                        float headingDeg = 0.0f, float altM = 0.0f, uint8_t sats = 0) {
    elrsb::Gps g;
    g.latE7 = latE7;
    g.lonE7 = lonE7;
    g.speedMps = speedMps;
    g.headingDeg = headingDeg;
    g.altM = altM;
    g.sats = sats;
    uint8_t p[elrsb::kGpsSize];
    return elrsb::encodeFrame(out, elrsb::kMsgTlmGps, p, elrsb::packGps(p, g));
  }

  // Angles in radians.
  static size_t packAttitude(uint8_t* out, float roll, float pitch, float yaw) {
    elrsb::Attitude a;
    a.roll = roll;
    a.pitch = pitch;
    a.yaw = yaw;
    uint8_t p[elrsb::kAttitudeSize];
    return elrsb::encodeFrame(out, elrsb::kMsgTlmAttitude, p, elrsb::packAttitude(p, a));
  }

  // Short status text shown on the handset as the flight mode (15 characters
  // at most; EdgeTX shows the first 13). An empty string hands the line back
  // to the bridge's own ARMED/DISARMED status.
  static size_t packMode(uint8_t* out, const char* text) {
    uint8_t p[elrsb::kMaxModeText];
    return elrsb::encodeFrame(out, elrsb::kMsgTlmMode, p, elrsb::packMode(p, text));
  }

  static size_t packVario(uint8_t* out, float vspeedMps) {
    uint8_t p[4];
    return elrsb::encodeFrame(out, elrsb::kMsgTlmVario, p, elrsb::packFloat(p, vspeedMps));
  }

  static size_t packBaro(uint8_t* out, float altM, float vspeedMps = 0.0f) {
    elrsb::Baro b;
    b.altM = altM;
    b.vspeedMps = vspeedMps;
    uint8_t p[elrsb::kBaroSize];
    return elrsb::encodeFrame(out, elrsb::kMsgTlmBaro, p, elrsb::packBaro(p, b));
  }

  static size_t packAirspeed(uint8_t* out, float speedMps) {
    uint8_t p[4];
    return elrsb::encodeFrame(out, elrsb::kMsgTlmAirspeed, p, elrsb::packFloat(p, speedMps));
  }

  // Up to elrsb::kMaxRpmValues values, e.g. one per wheel.
  static size_t packRpm(uint8_t* out, const int32_t* rpm, uint8_t count, uint8_t source = 0) {
    elrsb::Rpm r;
    r.source = source;
    r.count = count > elrsb::kMaxRpmValues ? elrsb::kMaxRpmValues : count;
    for (uint8_t i = 0; i < r.count; ++i) r.rpm[i] = rpm[i];
    uint8_t p[2 + 4 * elrsb::kMaxRpmValues];
    return elrsb::encodeFrame(out, elrsb::kMsgTlmRpm, p, elrsb::packRpm(p, r));
  }

  // Up to elrsb::kMaxTempValues temperatures in degrees Celsius.
  static size_t packTemp(uint8_t* out, const float* degC, uint8_t count, uint8_t source = 0) {
    elrsb::Temp t;
    t.source = source;
    t.count = count > elrsb::kMaxTempValues ? elrsb::kMaxTempValues : count;
    for (uint8_t i = 0; i < t.count; ++i) t.degC[i] = degC[i];
    uint8_t p[2 + 4 * elrsb::kMaxTempValues];
    return elrsb::encodeFrame(out, elrsb::kMsgTlmTemp, p, elrsb::packTemp(p, t));
  }

  // Any CRSF frame for the receiver: the bridge adds sync, length and CRC.
  // Returns 0 if the payload is longer than elrsb::kMaxCrsfPayload.
  static size_t packCrsf(uint8_t* out, uint8_t frameType, const uint8_t* payload, uint8_t len) {
    if (len > elrsb::kMaxCrsfPayload) return 0;
    uint8_t p[1 + elrsb::kMaxCrsfPayload];
    p[0] = frameType;
    if (len) memcpy(p + 1, payload, len);
    return elrsb::encodeFrame(out, elrsb::kMsgCrsfTx, p, (size_t)len + 1);
  }

  // Ask the bridge to change what it sends on this port (until it reboots).
  // The host UART streams binary from boot, so this is only needed on the
  // USB port or to change the rate. hz = 0 keeps the configured stream_hz.
  static size_t packSetStream(uint8_t* out, uint8_t mode = elrsb::kStreamBinary, uint16_t hz = 0,
                              uint8_t options = 0) {
    elrsb::SetStream s;
    s.mode = mode;
    s.hz = hz;
    s.options = options;
    uint8_t p[elrsb::kSetStreamSize];
    return elrsb::encodeFrame(out, elrsb::kMsgSetStream, p, elrsb::packSetStream(p, s));
  }

 private:
  elrsb::FrameParser parser_;
  elrsb::State state_;
  elrsb::Link link_;
  bool fresh_;
  bool haveLink_;
  uint32_t stateMs_;
  uint16_t staleAfterMs_;
  uint32_t states_;
};

#if defined(ARDUINO)
#include <Arduino.h>

// ElrsBridgeClient bound to an Arduino serial port (anything with
// available(), read() and write(buffer, length)). Call update() every loop.
template <class SerialT>
class ElrsBridgeSerial : public ElrsBridgeClient {
 public:
  explicit ElrsBridgeSerial(SerialT& serial)
      : serial_(serial), keepStream_(false), keepMode_(elrsb::kStreamBinary), keepHz_(0),
        keepOptions_(0), lastRequestMs_(0) {}

  // Read everything that has arrived. Returns true if there is a new update.
  bool update() {
    bool updated = false;
    while (serial_.available() > 0) {
      if (feed((uint8_t)serial_.read(), millis())) updated = true;
    }
    const uint32_t now = millis();
    if (keepStream_ && !fresh(now) && (uint32_t)(now - lastRequestMs_) >= 1000) {
      lastRequestMs_ = now;
      setStream(keepMode_, keepHz_, keepOptions_);
    }
    return updated;
  }

  // For a host on the bridge's USB port. There the stream only runs while it
  // has been asked for, and the bridge forgets the request when it restarts.
  // This asks now, and again (at most once a second) whenever updates stop.
  // Not needed on the host UART, which streams from power-up.
  void keepStream(uint8_t mode = elrsb::kStreamBinary, uint16_t hz = 0, uint8_t options = 0) {
    keepStream_ = true;
    keepMode_ = mode;
    keepHz_ = hz;
    keepOptions_ = options;
    lastRequestMs_ = millis();
    setStream(mode, hz, options);
  }
  void stopKeepingStream() { keepStream_ = false; }

  using ElrsBridgeClient::armed;
  using ElrsBridgeClient::ch;
  using ElrsBridgeClient::fresh;
  using ElrsBridgeClient::linkUp;
  using ElrsBridgeClient::state;
  using ElrsBridgeClient::value;
  bool fresh() { return fresh(millis()); }
  elrsb::State state() { return state(millis()); }
  bool linkUp() { return linkUp(millis()); }
  bool armed() { return armed(millis()); }
  int16_t value(uint8_t number) { return value(number, millis()); }
  float ch(uint8_t number) { return ch(number, millis()); }

  void sendBattery(float volts, float amps = 0.0f, float usedMah = 0.0f,
                   uint8_t remainingPct = 0) {
    uint8_t f[elrsb::kMaxFrame];
    serial_.write(f, packBattery(f, volts, amps, usedMah, remainingPct));
  }
  void sendGps(int32_t latE7, int32_t lonE7, float speedMps = 0.0f, float headingDeg = 0.0f,
               float altM = 0.0f, uint8_t sats = 0) {
    uint8_t f[elrsb::kMaxFrame];
    serial_.write(f, packGps(f, latE7, lonE7, speedMps, headingDeg, altM, sats));
  }
  void sendAttitude(float roll, float pitch, float yaw) {
    uint8_t f[elrsb::kMaxFrame];
    serial_.write(f, packAttitude(f, roll, pitch, yaw));
  }
  void sendMode(const char* text) {
    uint8_t f[elrsb::kMaxFrame];
    serial_.write(f, packMode(f, text));
  }
  void sendVario(float vspeedMps) {
    uint8_t f[elrsb::kMaxFrame];
    serial_.write(f, packVario(f, vspeedMps));
  }
  void sendBaro(float altM, float vspeedMps = 0.0f) {
    uint8_t f[elrsb::kMaxFrame];
    serial_.write(f, packBaro(f, altM, vspeedMps));
  }
  void sendAirspeed(float speedMps) {
    uint8_t f[elrsb::kMaxFrame];
    serial_.write(f, packAirspeed(f, speedMps));
  }
  void sendRpm(const int32_t* rpm, uint8_t count, uint8_t source = 0) {
    uint8_t f[elrsb::kMaxFrame];
    serial_.write(f, packRpm(f, rpm, count, source));
  }
  void sendTemp(const float* degC, uint8_t count, uint8_t source = 0) {
    uint8_t f[elrsb::kMaxFrame];
    serial_.write(f, packTemp(f, degC, count, source));
  }
  void sendCrsf(uint8_t frameType, const uint8_t* payload, uint8_t len) {
    uint8_t f[elrsb::kMaxFrame];
    const size_t n = packCrsf(f, frameType, payload, len);
    if (n) serial_.write(f, n);
  }
  void setStream(uint8_t mode = elrsb::kStreamBinary, uint16_t hz = 0, uint8_t options = 0) {
    uint8_t f[elrsb::kMaxFrame];
    serial_.write(f, packSetStream(f, mode, hz, options));
  }

 private:
  SerialT& serial_;
  bool keepStream_;
  uint8_t keepMode_;
  uint16_t keepHz_;
  uint8_t keepOptions_;
  uint32_t lastRequestMs_;
};
#endif  // ARDUINO

#endif  // ELRS_BRIDGE_CLIENT_H
