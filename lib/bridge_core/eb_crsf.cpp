#include "eb_crsf.h"

#include <math.h>
#include <string.h>

#include "eb_util.h"

namespace eb {
namespace crsf {

namespace {

const float kPi = 3.14159265358979f;

bool isSyncByte(uint8_t b) {
  return b == kAddrFlightController || b == kAddrReceiver || b == kAddrTxModule ||
         b == kAddrRadio;
}

// Scale a float and round it to an integer clamped to [lo, hi]. NaN gives 0.
int32_t scaled(float value, float scale, int32_t lo, int32_t hi) {
  if (!(value == value)) return 0;
  const double x = (double)value * (double)scale;
  if (x <= (double)lo) return lo;
  if (x >= (double)hi) return hi;
  return (int32_t)(x >= 0 ? x + 0.5 : x - 0.5);
}

// Telemetry payloads are big-endian.
void putBe16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}

void putBe24(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)((v >> 16) & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)(v & 0xFF);
}

void putBe32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)((v >> 24) & 0xFF);
  p[1] = (uint8_t)((v >> 16) & 0xFF);
  p[2] = (uint8_t)((v >> 8) & 0xFF);
  p[3] = (uint8_t)(v & 0xFF);
}

// Wrap an angle into [-pi, pi]. Non-finite input gives 0.
float wrapPi(float a) {
  if (!(a == a) || isinf(a)) return 0.0f;
  a = fmodf(a, 2.0f * kPi);
  if (a > kPi) a -= 2.0f * kPi;
  if (a < -kPi) a += 2.0f * kPi;
  return a;
}

// The receiver reports RSSI as a positive magnitude (80 means -80 dBm). Some
// CRSF devices send a two's-complement dBm value instead; accept both.
int8_t rssiToDbm(uint8_t raw) {
  return raw <= 127 ? (int8_t)(-(int)raw) : (int8_t)raw;
}

const uint16_t kTxPowerMw[] = {0, 10, 25, 100, 500, 1000, 2000, 250, 50};

}  // namespace

uint8_t crc8(const uint8_t* data, size_t len, uint8_t poly, uint8_t crc) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? (uint8_t)((uint8_t)(crc << 1) ^ poly) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

Parser::Parser() : size_(0), pendingDrop_(0), frames_(0), errors_(0) {
  memset(buf_, 0, sizeof buf_);
  frame_.addr = 0;
  frame_.type = 0;
  frame_.len = 0;
  frame_.payload = buf_;
}

void Parser::drop(size_t n) {
  if (n >= size_) {
    size_ = 0;
    return;
  }
  memmove(buf_, buf_ + n, size_ - n);
  size_ = (uint8_t)(size_ - n);
}

bool Parser::feed(uint8_t byte) {
  if (pendingDrop_) {
    drop(pendingDrop_);
    pendingDrop_ = 0;
  }
  if (size_ >= kMaxFrameSize) drop(1);
  buf_[size_++] = byte;

  while (size_ > 0) {
    if (!isSyncByte(buf_[0])) {
      drop(1);
      continue;
    }
    if (size_ < 2) break;
    const uint8_t len = buf_[1];
    if (len < 2 || len > kMaxFrameSize - 2) {
      drop(1);
      continue;
    }
    if (size_ < (size_t)len + 2) break;
    if (crc8(buf_ + 2, (size_t)len - 1) == buf_[len + 1]) {
      frame_.addr = buf_[0];
      frame_.type = buf_[2];
      frame_.len = (uint8_t)(len - 2);
      frame_.payload = buf_ + 3;
      pendingDrop_ = (uint8_t)(len + 2);
      ++frames_;
      return true;
    }
    // Not a frame after all: slide one byte and look again, so a real frame
    // that started inside this false one is still found.
    ++errors_;
    drop(1);
  }
  return false;
}

void unpackChannels(const uint8_t* payload, uint16_t* ticks) {
  uint32_t acc = 0;
  int bits = 0;
  int ch = 0;
  for (size_t i = 0; i < kChannelsPayloadSize && ch < kNumChannels; ++i) {
    acc |= (uint32_t)payload[i] << bits;
    bits += 8;
    while (bits >= 11 && ch < kNumChannels) {
      ticks[ch++] = (uint16_t)(acc & 0x7FF);
      acc >>= 11;
      bits -= 11;
    }
  }
}

void packChannels(const uint16_t* ticks, uint8_t* payload) {
  uint32_t acc = 0;
  int bits = 0;
  size_t i = 0;
  for (int ch = 0; ch < kNumChannels; ++ch) {
    acc |= (uint32_t)(ticks[ch] & 0x7FF) << bits;
    bits += 11;
    while (bits >= 8) {
      payload[i++] = (uint8_t)(acc & 0xFF);
      acc >>= 8;
      bits -= 8;
    }
  }
}

uint16_t ticksToUs(uint16_t ticks) {
  const int32_t us = 1500 + divRound(((int32_t)ticks - kTicksMid) * 5, 8);
  return (uint16_t)clampT<int32_t>(us, 0, 65535);
}

uint16_t usToTicks(uint16_t us) {
  const int32_t ticks = kTicksMid + divRound(((int32_t)us - 1500) * 8, 5);
  return (uint16_t)clampT<int32_t>(ticks, 0, 2047);
}

bool decodeLinkStats(const uint8_t* payload, size_t len, LinkStats& out) {
  if (len < kLinkStatsPayloadSize) return false;
  out.upRssi1 = rssiToDbm(payload[0]);
  out.upRssi2 = rssiToDbm(payload[1]);
  out.upLq = payload[2];
  out.upSnr = (int8_t)payload[3];
  out.antenna = payload[4];
  out.rfMode = payload[5];
  const uint8_t power = payload[6];
  out.txPowerMw = power < sizeof kTxPowerMw / sizeof kTxPowerMw[0] ? kTxPowerMw[power] : 0;
  out.downRssi = rssiToDbm(payload[7]);
  out.downLq = payload[8];
  out.downSnr = (int8_t)payload[9];
  return true;
}

size_t buildFrame(uint8_t* out, uint8_t type, const uint8_t* payload, size_t len) {
  if (len > kMaxPayload) return 0;
  out[0] = kSync;
  out[1] = (uint8_t)(len + 2);
  out[2] = type;
  if (len) memcpy(out + 3, payload, len);
  out[3 + len] = crc8(out + 2, len + 1);
  return len + 4;
}

size_t buildBindCommand(uint8_t* out) {
  // The frame Betaflight sends for its `bind_rx` command: a command frame
  // addressed to the receiver, subcommand "receiver" / "bind". Command frames
  // carry an inner CRC (poly 0xBA) ahead of the normal frame CRC.
  out[0] = kSync;
  out[1] = 7;
  out[2] = kTypeCommand;
  out[3] = kAddrReceiver;
  out[4] = kAddrFlightController;
  out[5] = 0x10;
  out[6] = 0x01;
  out[7] = crc8(out + 2, 5, kPolyCommand);
  out[8] = crc8(out + 2, 6, kPolyFrame);
  return 9;
}

size_t buildBattery(uint8_t* out, float volts, float amps, float usedMah,
                    uint8_t remainingPct) {
  uint8_t p[8];
  putBe16(p, (uint16_t)scaled(volts, 10.0f, 0, 32767));            // 0.1 V
  putBe16(p + 2, (uint16_t)(int16_t)scaled(amps, 10.0f, -32768, 32767));  // 0.1 A
  putBe24(p + 4, (uint32_t)scaled(usedMah, 1.0f, 0, 0xFFFFFF));    // mAh
  p[7] = remainingPct > 100 ? 100 : remainingPct;
  return buildFrame(out, kTypeBattery, p, sizeof p);
}

size_t buildGps(uint8_t* out, int32_t latE7, int32_t lonE7, float speedMps, float headingDeg,
                float altM, uint8_t sats) {
  uint8_t p[15];
  putBe32(p, (uint32_t)latE7);
  putBe32(p + 4, (uint32_t)lonE7);
  putBe16(p + 8, (uint16_t)scaled(speedMps, 36.0f, 0, 65535));     // 0.1 km/h
  float heading = (headingDeg == headingDeg && !isinf(headingDeg)) ? fmodf(headingDeg, 360.0f)
                                                                   : 0.0f;
  if (heading < 0) heading += 360.0f;
  putBe16(p + 10, (uint16_t)scaled(heading, 100.0f, 0, 35999));    // 0.01 deg
  const float alt = (altM == altM) ? altM : 0.0f;
  putBe16(p + 12, (uint16_t)scaled(alt + 1000.0f, 1.0f, 0, 65535));  // m, offset 1000
  p[14] = sats;
  return buildFrame(out, kTypeGps, p, sizeof p);
}

size_t buildAttitude(uint8_t* out, float rollRad, float pitchRad, float yawRad) {
  uint8_t p[6];
  putBe16(p, (uint16_t)(int16_t)scaled(wrapPi(pitchRad), 10000.0f, -32768, 32767));
  putBe16(p + 2, (uint16_t)(int16_t)scaled(wrapPi(rollRad), 10000.0f, -32768, 32767));
  putBe16(p + 4, (uint16_t)(int16_t)scaled(wrapPi(yawRad), 10000.0f, -32768, 32767));
  return buildFrame(out, kTypeAttitude, p, sizeof p);
}

size_t buildFlightMode(uint8_t* out, const char* text) {
  uint8_t p[kMaxFlightModeText + 1];
  size_t n = 0;
  while (n < kMaxFlightModeText && text[n] != '\0') {
    const char c = text[n];
    p[n] = (c >= 0x20 && c <= 0x7E) ? (uint8_t)c : (uint8_t)'?';
    ++n;
  }
  p[n++] = '\0';
  return buildFrame(out, kTypeFlightMode, p, n);
}

size_t buildVario(uint8_t* out, float vspeedMps) {
  uint8_t p[2];
  putBe16(p, (uint16_t)(int16_t)scaled(vspeedMps, 100.0f, -32768, 32767));  // cm/s
  return buildFrame(out, kTypeVario, p, sizeof p);
}

size_t buildBaroAltitude(uint8_t* out, float altM, float vspeedMps) {
  uint8_t p[4];
  const float alt = (altM == altM) ? altM : 0.0f;
  // Decimetres offset by 10000 when that fits in 15 bits (-1000.0 m up to
  // 2276.7 m), otherwise whole metres with the top bit set.
  const int32_t dm = scaled(alt, 10.0f, -10000, 400000);
  uint16_t packed;
  if (dm <= 22767) {
    packed = (uint16_t)(dm + 10000);
  } else {
    packed = (uint16_t)(0x8000u | (uint16_t)scaled(alt, 1.0f, 0, 0x7FFE));
  }
  putBe16(p, packed);
  putBe16(p + 2, (uint16_t)(int16_t)scaled(vspeedMps, 100.0f, -32768, 32767));  // cm/s
  return buildFrame(out, kTypeBaroAltitude, p, sizeof p);
}

size_t buildAirspeed(uint8_t* out, float speedMps) {
  uint8_t p[2];
  putBe16(p, (uint16_t)scaled(speedMps, 36.0f, 0, 65535));  // 0.1 km/h
  return buildFrame(out, kTypeAirspeed, p, sizeof p);
}

size_t buildRpm(uint8_t* out, uint8_t source, const int32_t* rpm, uint8_t count) {
  if (count < 1 || count > kMaxRpmValues) return 0;
  uint8_t p[1 + 3 * kMaxRpmValues];
  p[0] = source;
  for (uint8_t i = 0; i < count; ++i) {
    const int32_t v = clampT<int32_t>(rpm[i], -0x800000, 0x7FFFFF);
    putBe24(p + 1 + 3 * i, (uint32_t)v);
  }
  return buildFrame(out, kTypeRpm, p, (size_t)(1 + 3 * count));
}

size_t buildTemperature(uint8_t* out, uint8_t source, const float* degC, uint8_t count) {
  if (count < 1 || count > kMaxTempValues) return 0;
  uint8_t p[1 + 2 * kMaxTempValues];
  p[0] = source;
  for (uint8_t i = 0; i < count; ++i) {
    putBe16(p + 1 + 2 * i, (uint16_t)(int16_t)scaled(degC[i], 10.0f, -32768, 32767));  // 0.1 C
  }
  return buildFrame(out, kTypeTemperature, p, (size_t)(1 + 2 * count));
}

}  // namespace crsf
}  // namespace eb
