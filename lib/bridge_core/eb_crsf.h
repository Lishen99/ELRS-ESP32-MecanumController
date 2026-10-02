// CRSF (Crossfire serial protocol) codec as spoken by ExpressLRS receivers.
//
// Wire format of every frame:
//
//   SYNC | LEN | TYPE | PAYLOAD | CRC8
//
// LEN counts TYPE + PAYLOAD + CRC8. CRC8 is DVB-S2 (poly 0xD5) over TYPE and
// PAYLOAD. The receiver talks 420000 baud 8N1, not inverted, by default.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace eb {
namespace crsf {

const uint8_t kSync = 0xC8;

const uint8_t kAddrBroadcast = 0x00;
const uint8_t kAddrFlightController = 0xC8;
const uint8_t kAddrRadio = 0xEA;      // the handset
const uint8_t kAddrReceiver = 0xEC;
const uint8_t kAddrTxModule = 0xEE;

const uint8_t kPolyFrame = 0xD5;      // frame CRC
const uint8_t kPolyCommand = 0xBA;    // inner CRC of command (0x32) frames

const size_t kMaxFrameSize = 64;      // sync + len + type + payload + crc
const size_t kMaxPayload = 60;
const int kNumChannels = 16;
const size_t kChannelsPayloadSize = 22;
const size_t kLinkStatsPayloadSize = 10;

// Limits applied by the telemetry frame builders.
const uint8_t kMaxFlightModeText = 15;  // characters, excluding the terminator
const uint8_t kMaxRpmValues = 8;
const uint8_t kMaxTempValues = 8;

// Channel values are 11-bit "ticks". 172..1811 is the normal stick range and
// corresponds to 988..2012 microseconds; 992 is centre (1500 us).
const uint16_t kTicksMin = 172;
const uint16_t kTicksMid = 992;
const uint16_t kTicksMax = 1811;

enum FrameType {
  kTypeGps = 0x02,
  kTypeVario = 0x07,
  kTypeBattery = 0x08,
  kTypeBaroAltitude = 0x09,
  kTypeAirspeed = 0x0A,
  kTypeHeartbeat = 0x0B,
  kTypeRpm = 0x0C,
  kTypeTemperature = 0x0D,
  kTypeLinkStats = 0x14,
  kTypeRcChannels = 0x16,
  kTypeAttitude = 0x1E,
  kTypeFlightMode = 0x21,
  kTypeDevicePing = 0x28,
  kTypeDeviceInfo = 0x29,
  kTypeCommand = 0x32
};

uint8_t crc8(const uint8_t* data, size_t len, uint8_t poly = kPolyFrame, uint8_t crc = 0);

struct Frame {
  uint8_t addr;            // sync / destination address byte
  uint8_t type;
  uint8_t len;             // payload length (type and CRC excluded)
  const uint8_t* payload;
};

// Byte-stream frame extractor. Keeps a sliding window so it resynchronises
// after garbage or a mid-frame start without discarding the good frame that
// follows.
class Parser {
 public:
  Parser();

  // Feed one byte. Returns true when a CRC-valid frame is available through
  // frame(); that frame stays valid until the next call to feed().
  bool feed(uint8_t byte);
  const Frame& frame() const { return frame_; }

  uint32_t frames() const { return frames_; }
  uint32_t errors() const { return errors_; }

 private:
  void drop(size_t n);

  uint8_t buf_[kMaxFrameSize];
  uint8_t size_;
  uint8_t pendingDrop_;
  uint32_t frames_;
  uint32_t errors_;
  Frame frame_;
};

// RC channels: 16 x 11 bit, packed LSB first into 22 bytes.
void unpackChannels(const uint8_t* payload, uint16_t* ticks);
void packChannels(const uint16_t* ticks, uint8_t* payload);
uint16_t ticksToUs(uint16_t ticks);
uint16_t usToTicks(uint16_t us);

struct LinkStats {
  int8_t upRssi1;      // dBm
  int8_t upRssi2;      // dBm
  uint8_t upLq;        // %
  int8_t upSnr;        // dB
  uint8_t antenna;
  uint8_t rfMode;
  uint16_t txPowerMw;
  int8_t downRssi;     // dBm
  uint8_t downLq;      // %
  int8_t downSnr;      // dB
};

bool decodeLinkStats(const uint8_t* payload, size_t len, LinkStats& out);

// Frame builders. `out` must hold kMaxFrameSize bytes; each returns the frame
// length, or 0 if the input cannot be encoded.
size_t buildFrame(uint8_t* out, uint8_t type, const uint8_t* payload, size_t len);
size_t buildBindCommand(uint8_t* out);
size_t buildBattery(uint8_t* out, float volts, float amps, float usedMah, uint8_t remainingPct);
size_t buildGps(uint8_t* out, int32_t latE7, int32_t lonE7, float speedMps, float headingDeg,
                float altM, uint8_t sats);
size_t buildAttitude(uint8_t* out, float rollRad, float pitchRad, float yawRad);
size_t buildFlightMode(uint8_t* out, const char* text);
size_t buildVario(uint8_t* out, float vspeedMps);
size_t buildBaroAltitude(uint8_t* out, float altM, float vspeedMps);
size_t buildAirspeed(uint8_t* out, float speedMps);
size_t buildRpm(uint8_t* out, uint8_t source, const int32_t* rpm, uint8_t count);
size_t buildTemperature(uint8_t* out, uint8_t source, const float* degC, uint8_t count);

}  // namespace crsf
}  // namespace eb
