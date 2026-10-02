// ELRS Bridge host protocol - wire format.
//
// This header is the single source of truth for the binary protocol spoken
// between the ESP32 bridge and whatever is driving the robot (Jetson, Teensy,
// another MCU...). The firmware and the host client both include it, so the
// two ends cannot drift apart.
//
// Header-only, C++11, no dependencies beyond <stdint.h>/<string.h>. Builds on
// AVR, ARM, ESP32 and desktop compilers.
//
// Frame layout (all multi-byte fields little-endian):
//
//   0xEB 0x90 | LEN | TYPE | PAYLOAD[LEN] | CRC16_LO CRC16_HI
//
// CRC16 is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over LEN, TYPE and
// PAYLOAD. Both sync bytes are >= 0x80, so a frame can never be mistaken for
// the printable ASCII of the text CLI that shares the same port.
#ifndef ELRS_BRIDGE_PROTOCOL_H
#define ELRS_BRIDGE_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace elrsb {

const uint8_t kProtocolVersion = 1;

const uint8_t kSync1 = 0xEB;
const uint8_t kSync2 = 0x90;
const uint8_t kMaxPayload = 128;
const uint8_t kFrameOverhead = 6;  // 2 sync + len + type + 2 crc
const uint16_t kMaxFrame = kMaxPayload + kFrameOverhead;

const uint8_t kMaxChannels = 16;
const uint8_t kMaxModeText = 15;   // telemetry mode string, excluding NUL
const uint8_t kMaxRpmValues = 8;
const uint8_t kMaxTempValues = 8;
const uint8_t kMaxCrsfPayload = 60;

// Output value range. Bipolar outputs span -kOutputScale..+kOutputScale,
// unipolar outputs span 0..+kOutputScale.
const int16_t kOutputScale = 1000;

enum MsgType {
  // bridge -> host
  kMsgState = 0x01,       // mapped channels + link flags (the periodic stream)
  kMsgLink = 0x02,        // link statistics and bridge counters
  kMsgRawChannels = 0x03, // unmapped channel pulse widths in microseconds
  kMsgCrsfFrame = 0x04,   // a CRSF frame forwarded verbatim from the receiver
  kMsgInfo = 0x10,        // reply to kMsgGetInfo
  kMsgAck = 0x11,         // result of a command
  kMsgCliReply = 0x12,    // u8 flags, u8 tag, then a chunk of a kMsgCli command's text output

  // host -> bridge
  kMsgGetInfo = 0x20,
  kMsgSetStream = 0x21,   // choose stream format/rate for this port (not saved)
  kMsgCli = 0x22,         // u8 tag, then a CLI command line; answered by kMsgCliReply frames
                          // carrying the same tag, so a late reply cannot be mistaken
                          // for the answer to a newer command
  kMsgRequest = 0x23,     // one-shot request for kMsgState/kMsgLink/kMsgRawChannels
  kMsgTlmBattery = 0x30,
  kMsgTlmGps = 0x31,
  kMsgTlmAttitude = 0x32,
  kMsgTlmMode = 0x33,
  kMsgTlmVario = 0x34,
  kMsgTlmBaro = 0x35,
  kMsgTlmAirspeed = 0x36,
  kMsgTlmRpm = 0x37,
  kMsgTlmTemp = 0x38,
  kMsgCrsfTx = 0x3F       // send an arbitrary CRSF frame to the receiver
};

enum StateFlags {
  kFlagLinkUp = 0x01,      // RC frames are arriving from the handset
  kFlagFailsafe = 0x02,    // channel values are failsafe substitutes, not live sticks
  kFlagArmed = 0x04,       // arm switch is on and the arming rules are satisfied
  kFlagGamepad = 0x08,     // the Bluetooth controller is in charge of the wheels
  kFlagMotorsLive = 0x10   // the wheels follow the sticks of whichever controller is in charge
};

enum StreamMode {
  kStreamOff = 0,     // port is quiet; text CLI with echo and prompt
  kStreamBinary = 1,  // kMsgState frames
  kStreamText = 2     // "$RC,..." lines
};

enum StreamOptions {
  kStreamOptRaw = 0x01  // also send kMsgRawChannels with every kMsgState
};

enum AckStatus {
  kAckOk = 0,
  kAckBadLength = 1,
  kAckBadValue = 2,
  kAckUnknown = 3,
  kAckBusy = 4
};

enum CliReplyFlags {
  kCliReplyLast = 0x01  // final chunk of this command's output
};

// ---------------------------------------------------------------------------
// CRC and little-endian helpers

inline uint16_t crc16(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; ++i) {
    crc = (uint16_t)(crc ^ ((uint16_t)data[i] << 8));
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((uint16_t)(crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

inline void putU16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}
inline void putI16(uint8_t* p, int16_t v) { putU16(p, (uint16_t)v); }
inline void putU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}
inline void putI32(uint8_t* p, int32_t v) { putU32(p, (uint32_t)v); }
inline void putF32(uint8_t* p, float v) {
  uint32_t u;
  memcpy(&u, &v, sizeof u);
  putU32(p, u);
}

inline uint16_t getU16(const uint8_t* p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
inline int16_t getI16(const uint8_t* p) { return (int16_t)getU16(p); }
inline uint32_t getU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}
inline int32_t getI32(const uint8_t* p) { return (int32_t)getU32(p); }
inline float getF32(const uint8_t* p) {
  uint32_t u = getU32(p);
  float v;
  memcpy(&v, &u, sizeof v);
  return v;
}

// ---------------------------------------------------------------------------
// Framing

// Writes a complete frame to `out` (which must hold len + kFrameOverhead
// bytes). Returns the frame size, or 0 if the payload is too long.
inline size_t encodeFrame(uint8_t* out, uint8_t type, const uint8_t* payload,
                          size_t len) {
  if (len > kMaxPayload) return 0;
  out[0] = kSync1;
  out[1] = kSync2;
  out[2] = (uint8_t)len;
  out[3] = type;
  if (len) memcpy(out + 4, payload, len);
  const uint16_t crc = crc16(out + 2, len + 2);
  putU16(out + 4 + len, crc);
  return len + kFrameOverhead;
}

enum FeedResult {
  kFeedText = 0,      // byte is not part of a frame (plain text on a shared port)
  kFeedConsumed = 1,  // byte belongs to a frame still being received
  kFeedFrame = 2      // byte completed a valid frame: see type()/payload()/length()
};

// Byte-at-a-time frame decoder.
class FrameParser {
 public:
  FrameParser() : state_(kIdle), len_(0), pos_(0), type_(0), crcLo_(0), errors_(0) {
    memset(buf_, 0, sizeof buf_);
  }

  FeedResult feed(uint8_t b) {
    switch (state_) {
      case kIdle:
        if (b != kSync1) return kFeedText;
        state_ = kGotSync1;
        return kFeedConsumed;
      case kGotSync1:
        if (b == kSync2) {
          state_ = kLen;
          return kFeedConsumed;
        }
        if (b == kSync1) return kFeedConsumed;
        state_ = kIdle;
        return kFeedText;
      case kLen:
        if (b > kMaxPayload) {
          state_ = kIdle;
          ++errors_;
          return kFeedConsumed;
        }
        len_ = b;
        state_ = kType;
        return kFeedConsumed;
      case kType:
        type_ = b;
        pos_ = 0;
        state_ = len_ ? kPayload : kCrcLo;
        return kFeedConsumed;
      case kPayload:
        buf_[pos_++] = b;
        if (pos_ >= len_) state_ = kCrcLo;
        return kFeedConsumed;
      case kCrcLo:
        crcLo_ = b;
        state_ = kCrcHi;
        return kFeedConsumed;
      case kCrcHi:
      default: {
        state_ = kIdle;
        const uint8_t header[2] = {len_, type_};
        const uint16_t crc = crc16(buf_, len_, crc16(header, 2));
        if (crc == (uint16_t)((uint16_t)crcLo_ | ((uint16_t)b << 8))) return kFeedFrame;
        ++errors_;
        return kFeedConsumed;
      }
    }
  }

  // True when no frame is in progress.
  bool idle() const { return state_ == kIdle; }
  // Abandon a partially received frame (use after a long gap between bytes).
  void reset() { state_ = kIdle; }

  uint8_t type() const { return type_; }
  uint8_t length() const { return len_; }
  const uint8_t* payload() const { return buf_; }
  uint32_t errors() const { return errors_; }

 private:
  enum ParseState { kIdle, kGotSync1, kLen, kType, kPayload, kCrcLo, kCrcHi };
  uint8_t state_;
  uint8_t len_;
  uint8_t pos_;
  uint8_t type_;
  uint8_t crcLo_;
  uint32_t errors_;
  uint8_t buf_[kMaxPayload];
};

// ---------------------------------------------------------------------------
// Payloads. Each pack function returns the payload length; each unpack
// function returns false if the payload is too short.

// kMsgState
struct State {
  uint8_t flags;          // StateFlags
  uint8_t lq;             // uplink link quality at the robot, 0..100 %
  int8_t rssi;            // uplink RSSI at the robot, dBm
  int8_t snr;             // uplink SNR, dB
  uint16_t ageMs;         // time since the last RC frame (0xFFFF = none yet / saturated)
  uint8_t seq;            // increments with every State frame sent on this port
  uint8_t count;          // number of valid entries in ch[]
  int16_t ch[kMaxChannels];  // mapped outputs, see kOutputScale

  bool linkUp() const { return (flags & kFlagLinkUp) != 0; }
  bool failsafe() const { return (flags & kFlagFailsafe) != 0; }
  bool armed() const { return (flags & kFlagArmed) != 0; }
  bool gamepad() const { return (flags & kFlagGamepad) != 0; }
  bool motorsLive() const { return (flags & kFlagMotorsLive) != 0; }
};
const uint8_t kStateHeaderSize = 8;

inline uint8_t packState(uint8_t* p, const State& s) {
  const uint8_t n = s.count > kMaxChannels ? kMaxChannels : s.count;
  p[0] = s.flags;
  p[1] = s.lq;
  p[2] = (uint8_t)s.rssi;
  p[3] = (uint8_t)s.snr;
  putU16(p + 4, s.ageMs);
  p[6] = s.seq;
  p[7] = n;
  for (uint8_t i = 0; i < n; ++i) putI16(p + kStateHeaderSize + 2 * i, s.ch[i]);
  return (uint8_t)(kStateHeaderSize + 2 * n);
}

inline bool unpackState(const uint8_t* p, size_t len, State& s) {
  if (len < kStateHeaderSize) return false;
  const uint8_t n = p[7];
  if (n > kMaxChannels || len < (size_t)(kStateHeaderSize + 2 * n)) return false;
  s.flags = p[0];
  s.lq = p[1];
  s.rssi = (int8_t)p[2];
  s.snr = (int8_t)p[3];
  s.ageMs = getU16(p + 4);
  s.seq = p[6];
  s.count = n;
  for (uint8_t i = 0; i < kMaxChannels; ++i) {
    s.ch[i] = i < n ? getI16(p + kStateHeaderSize + 2 * i) : (int16_t)0;
  }
  return true;
}

// kMsgLink
struct Link {
  int8_t upRssi1;         // uplink RSSI antenna 1, dBm
  int8_t upRssi2;         // uplink RSSI antenna 2, dBm
  uint8_t upLq;           // uplink link quality, %
  int8_t upSnr;           // uplink SNR, dB
  uint8_t antenna;        // active antenna
  uint8_t rfMode;         // ELRS packet-rate index as reported by the receiver
  uint16_t txPowerMw;     // handset transmit power
  int8_t downRssi;        // downlink (telemetry) RSSI at the handset, dBm
  uint8_t downLq;         // downlink link quality, %
  int8_t downSnr;         // downlink SNR, dB
  uint8_t flags;          // StateFlags
  uint16_t rcRateHz;      // measured RC frames per second
  uint16_t vbatMv;        // battery voltage measured by the bridge ADC, 0 = not fitted
  uint32_t uptimeMs;
  uint32_t rcFrames;      // RC frames decoded since boot
  uint32_t crsfErrors;    // CRSF frames rejected (bad CRC / resync)
  uint16_t failsafeCount; // link-loss events since boot
  uint16_t txDropped;     // frames dropped on this port because it could not keep up
};
const uint8_t kLinkSize = 32;

inline uint8_t packLink(uint8_t* p, const Link& l) {
  p[0] = (uint8_t)l.upRssi1;
  p[1] = (uint8_t)l.upRssi2;
  p[2] = l.upLq;
  p[3] = (uint8_t)l.upSnr;
  p[4] = l.antenna;
  p[5] = l.rfMode;
  putU16(p + 6, l.txPowerMw);
  p[8] = (uint8_t)l.downRssi;
  p[9] = l.downLq;
  p[10] = (uint8_t)l.downSnr;
  p[11] = l.flags;
  putU16(p + 12, l.rcRateHz);
  putU16(p + 14, l.vbatMv);
  putU32(p + 16, l.uptimeMs);
  putU32(p + 20, l.rcFrames);
  putU32(p + 24, l.crsfErrors);
  putU16(p + 28, l.failsafeCount);
  putU16(p + 30, l.txDropped);
  return kLinkSize;
}

inline bool unpackLink(const uint8_t* p, size_t len, Link& l) {
  if (len < kLinkSize) return false;
  l.upRssi1 = (int8_t)p[0];
  l.upRssi2 = (int8_t)p[1];
  l.upLq = p[2];
  l.upSnr = (int8_t)p[3];
  l.antenna = p[4];
  l.rfMode = p[5];
  l.txPowerMw = getU16(p + 6);
  l.downRssi = (int8_t)p[8];
  l.downLq = p[9];
  l.downSnr = (int8_t)p[10];
  l.flags = p[11];
  l.rcRateHz = getU16(p + 12);
  l.vbatMv = getU16(p + 14);
  l.uptimeMs = getU32(p + 16);
  l.rcFrames = getU32(p + 20);
  l.crsfErrors = getU32(p + 24);
  l.failsafeCount = getU16(p + 28);
  l.txDropped = getU16(p + 30);
  return true;
}

// kMsgRawChannels: u8 count, then count x u16 pulse width in microseconds.
inline uint8_t packRawChannels(uint8_t* p, const uint16_t* us, uint8_t count) {
  if (count > kMaxChannels) count = kMaxChannels;
  p[0] = count;
  for (uint8_t i = 0; i < count; ++i) putU16(p + 1 + 2 * i, us[i]);
  return (uint8_t)(1 + 2 * count);
}

inline bool unpackRawChannels(const uint8_t* p, size_t len, uint16_t* us, uint8_t& count) {
  if (len < 1) return false;
  const uint8_t n = p[0];
  if (n > kMaxChannels || len < (size_t)(1 + 2 * n)) return false;
  for (uint8_t i = 0; i < n; ++i) us[i] = getU16(p + 1 + 2 * i);
  count = n;
  return true;
}

// kMsgInfo: fixed header followed by the board name (not NUL-terminated).
struct Info {
  uint8_t protocol;
  uint8_t fwMajor;
  uint8_t fwMinor;
  uint8_t fwPatch;
  uint8_t channels;
  uint8_t auxOutputs;
  char board[24];
};
const uint8_t kInfoHeaderSize = 6;

inline uint8_t packInfo(uint8_t* p, const Info& info) {
  p[0] = info.protocol;
  p[1] = info.fwMajor;
  p[2] = info.fwMinor;
  p[3] = info.fwPatch;
  p[4] = info.channels;
  p[5] = info.auxOutputs;
  size_t n = 0;
  while (n < sizeof info.board - 1 && info.board[n] != '\0') ++n;
  memcpy(p + kInfoHeaderSize, info.board, n);
  return (uint8_t)(kInfoHeaderSize + n);
}

inline bool unpackInfo(const uint8_t* p, size_t len, Info& info) {
  if (len < kInfoHeaderSize) return false;
  info.protocol = p[0];
  info.fwMajor = p[1];
  info.fwMinor = p[2];
  info.fwPatch = p[3];
  info.channels = p[4];
  info.auxOutputs = p[5];
  size_t n = len - kInfoHeaderSize;
  if (n > sizeof info.board - 1) n = sizeof info.board - 1;
  memcpy(info.board, p + kInfoHeaderSize, n);
  info.board[n] = '\0';
  return true;
}

// kMsgSetStream
struct SetStream {
  uint8_t mode;     // StreamMode
  uint16_t hz;      // 0 = use the configured stream_hz
  uint8_t options;  // StreamOptions
};
const uint8_t kSetStreamSize = 4;

inline uint8_t packSetStream(uint8_t* p, const SetStream& s) {
  p[0] = s.mode;
  putU16(p + 1, s.hz);
  p[3] = s.options;
  return kSetStreamSize;
}

inline bool unpackSetStream(const uint8_t* p, size_t len, SetStream& s) {
  if (len < kSetStreamSize) return false;
  s.mode = p[0];
  s.hz = getU16(p + 1);
  s.options = p[3];
  return true;
}

// kMsgTlmBattery
struct Battery {
  float volts;
  float amps;
  float usedMah;
  uint8_t remainingPct;
};
const uint8_t kBatterySize = 13;

inline uint8_t packBattery(uint8_t* p, const Battery& b) {
  putF32(p, b.volts);
  putF32(p + 4, b.amps);
  putF32(p + 8, b.usedMah);
  p[12] = b.remainingPct;
  return kBatterySize;
}

inline bool unpackBattery(const uint8_t* p, size_t len, Battery& b) {
  if (len < kBatterySize) return false;
  b.volts = getF32(p);
  b.amps = getF32(p + 4);
  b.usedMah = getF32(p + 8);
  b.remainingPct = p[12];
  return true;
}

// kMsgTlmGps
struct Gps {
  int32_t latE7;      // degrees * 1e7
  int32_t lonE7;      // degrees * 1e7
  float speedMps;     // ground speed, m/s
  float headingDeg;   // course over ground, degrees
  float altM;         // altitude, metres
  uint8_t sats;
};
const uint8_t kGpsSize = 21;

inline uint8_t packGps(uint8_t* p, const Gps& g) {
  putI32(p, g.latE7);
  putI32(p + 4, g.lonE7);
  putF32(p + 8, g.speedMps);
  putF32(p + 12, g.headingDeg);
  putF32(p + 16, g.altM);
  p[20] = g.sats;
  return kGpsSize;
}

inline bool unpackGps(const uint8_t* p, size_t len, Gps& g) {
  if (len < kGpsSize) return false;
  g.latE7 = getI32(p);
  g.lonE7 = getI32(p + 4);
  g.speedMps = getF32(p + 8);
  g.headingDeg = getF32(p + 12);
  g.altM = getF32(p + 16);
  g.sats = p[20];
  return true;
}

// kMsgTlmAttitude - radians
struct Attitude {
  float roll;
  float pitch;
  float yaw;
};
const uint8_t kAttitudeSize = 12;

inline uint8_t packAttitude(uint8_t* p, const Attitude& a) {
  putF32(p, a.roll);
  putF32(p + 4, a.pitch);
  putF32(p + 8, a.yaw);
  return kAttitudeSize;
}

inline bool unpackAttitude(const uint8_t* p, size_t len, Attitude& a) {
  if (len < kAttitudeSize) return false;
  a.roll = getF32(p);
  a.pitch = getF32(p + 4);
  a.yaw = getF32(p + 8);
  return true;
}

// kMsgTlmMode: up to kMaxModeText characters of text, no terminator.
inline uint8_t packMode(uint8_t* p, const char* text) {
  size_t n = strlen(text);
  if (n > kMaxModeText) n = kMaxModeText;
  memcpy(p, text, n);
  return (uint8_t)n;
}

// kMsgTlmVario: f32 vertical speed, m/s.
// kMsgTlmAirspeed: f32 speed, m/s.
inline uint8_t packFloat(uint8_t* p, float v) {
  putF32(p, v);
  return 4;
}

inline bool unpackFloat(const uint8_t* p, size_t len, float& v) {
  if (len < 4) return false;
  v = getF32(p);
  return true;
}

// kMsgTlmBaro
struct Baro {
  float altM;       // altitude, metres
  float vspeedMps;  // vertical speed, m/s
};
const uint8_t kBaroSize = 8;

inline uint8_t packBaro(uint8_t* p, const Baro& b) {
  putF32(p, b.altM);
  putF32(p + 4, b.vspeedMps);
  return kBaroSize;
}

inline bool unpackBaro(const uint8_t* p, size_t len, Baro& b) {
  if (len < kBaroSize) return false;
  b.altM = getF32(p);
  b.vspeedMps = getF32(p + 4);
  return true;
}

// kMsgTlmRpm: u8 source id, u8 count, then count x i32 rpm.
struct Rpm {
  uint8_t source;
  uint8_t count;
  int32_t rpm[kMaxRpmValues];
};

inline uint8_t packRpm(uint8_t* p, const Rpm& r) {
  const uint8_t n = r.count > kMaxRpmValues ? kMaxRpmValues : r.count;
  p[0] = r.source;
  p[1] = n;
  for (uint8_t i = 0; i < n; ++i) putI32(p + 2 + 4 * i, r.rpm[i]);
  return (uint8_t)(2 + 4 * n);
}

inline bool unpackRpm(const uint8_t* p, size_t len, Rpm& r) {
  if (len < 2) return false;
  const uint8_t n = p[1];
  if (n < 1 || n > kMaxRpmValues || len < (size_t)(2 + 4 * n)) return false;
  r.source = p[0];
  r.count = n;
  for (uint8_t i = 0; i < n; ++i) r.rpm[i] = getI32(p + 2 + 4 * i);
  return true;
}

// kMsgTlmTemp: u8 source id, u8 count, then count x f32 degrees Celsius.
struct Temp {
  uint8_t source;
  uint8_t count;
  float degC[kMaxTempValues];
};

inline uint8_t packTemp(uint8_t* p, const Temp& t) {
  const uint8_t n = t.count > kMaxTempValues ? kMaxTempValues : t.count;
  p[0] = t.source;
  p[1] = n;
  for (uint8_t i = 0; i < n; ++i) putF32(p + 2 + 4 * i, t.degC[i]);
  return (uint8_t)(2 + 4 * n);
}

inline bool unpackTemp(const uint8_t* p, size_t len, Temp& t) {
  if (len < 2) return false;
  const uint8_t n = p[1];
  if (n < 1 || n > kMaxTempValues || len < (size_t)(2 + 4 * n)) return false;
  t.source = p[0];
  t.count = n;
  for (uint8_t i = 0; i < n; ++i) t.degC[i] = getF32(p + 2 + 4 * i);
  return true;
}

}  // namespace elrsb

#endif  // ELRS_BRIDGE_PROTOCOL_H
