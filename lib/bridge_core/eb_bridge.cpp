// Bridge: start-up, the receiver side, and the periodic work.
//
// The rest of the class lives in the files named for what they do:
//   eb_host_ports.cpp   the host protocol and the two host-facing ports
//   eb_cli.cpp          the text commands
//   eb_aux.cpp          what the aux pins should be doing
#include "eb_bridge.h"

#include <string.h>

namespace eb {

static_assert(crsf::kNumChannels == kNumOutputs, "one mapped output per CRSF channel");
static_assert(elrsb::kMaxChannels == kNumOutputs, "host protocol carries every output");
static_assert(crsf::kMaxFlightModeText == elrsb::kMaxModeText, "mode text limits must match");
static_assert(crsf::kMaxRpmValues == elrsb::kMaxRpmValues, "rpm limits must match");
static_assert(crsf::kMaxTempValues == elrsb::kMaxTempValues, "temperature limits must match");
static_assert(crsf::kMaxPayload == elrsb::kMaxCrsfPayload, "raw CRSF limits must match");

Bridge::Bridge(Platform& platform)
    : plat_(platform),
      dirty_(false),
      loadSkipped_(0),
      haveLink_(false),
      haveCrsf_(false),
      crsfBytes_(0),
      motorCommandsDue_(false),
      motorWrites_(0),
      motorFailures_(0),
      motorBoardOk_(false),
      localVbat_(0),
      haveLocalVbat_(false),
      nowMs_(0),
      crsfTx_(crsfTxBuf_, sizeof crsfTxBuf_) {
  memset(&link_, 0, sizeof link_);
  configDefaults(defaults_);
  cfg_ = defaults_;
  for (int i = 0; i < kNumAux; ++i) {
    auxBoot_[i].pin = -1;
    auxBoot_[i].mode = kAuxServo;
  }
}

void Bridge::begin(uint32_t nowMs) {
  nowMs_ = nowMs;
  configDefaults(defaults_);
  plat_.boardDefaults(defaults_);
  cfg_ = defaults_;

  size_t len = 0;
  if (plat_.loadConfigText(cfgText_, sizeof cfgText_, len)) {
    loadSkipped_ = configFromText(cfg_, cfgText_, len, plat_);
  }
  dirty_ = false;
  rc_.update(cfg_, nowMs);  // puts the loaded arming rules in force

  // The aux pins are attached once, at start-up. Remember which pin and mode
  // each one started with, so a later change to those settings (which only
  // applies after a reboot) cannot leave a live pin unattended.
  for (int i = 0; i < kNumAux; ++i) {
    auxBoot_[i].pin = cfg_.aux[i].pin;
    auxBoot_[i].mode = cfg_.aux[i].mode;
  }

  setStreamMode(ports_[kPortUsb], cfg_.usbStream);
  setStreamMode(ports_[kPortHost], cfg_.hostStream);
  for (int i = 0; i < kNumPorts; ++i) {
    if (ports_[i].streamMode == elrsb::kStreamOff) banner(ports_[i]);
  }
}

// ---------------------------------------------------------------------------
// Receiver side

void Bridge::feedCrsf(const uint8_t* data, size_t len, uint32_t nowMs) {
  nowMs_ = nowMs;
  crsfBytes_ += (uint32_t)len;
  for (size_t i = 0; i < len; ++i) {
    if (crsfParser_.feed(data[i])) onCrsfFrame(crsfParser_.frame(), nowMs);
  }
}

void Bridge::onCrsfFrame(const crsf::Frame& frame, uint32_t nowMs) {
  haveCrsf_ = true;

  if (frame.type == crsf::kTypeRcChannels && frame.len == crsf::kChannelsPayloadSize) {
    uint16_t ticks[crsf::kNumChannels];
    crsf::unpackChannels(frame.payload, ticks);
    rc_.onChannels(ticks, nowMs);
    rc_.update(cfg_, nowMs);
    return;
  }
  if (frame.type == crsf::kTypeLinkStats) {
    if (crsf::decodeLinkStats(frame.payload, frame.len, link_)) haveLink_ = true;
    return;
  }

  // Anything else is only of interest to a host that asked for it.
  if (!cfg_.fwdCrsf || (size_t)frame.len + 2 > elrsb::kMaxPayload) return;
  uint8_t payload[elrsb::kMaxPayload];
  payload[0] = frame.addr;
  payload[1] = frame.type;
  memcpy(payload + 2, frame.payload, frame.len);
  for (int i = 0; i < kNumPorts; ++i) {
    if (ports_[i].streamMode == elrsb::kStreamBinary) {
      sendFrame(ports_[i], elrsb::kMsgCrsfFrame, payload, (size_t)frame.len + 2);
    }
  }
}

bool Bridge::sendCrsf(const uint8_t* frame, size_t len) {
  return len > 0 && crsfTx_.push(frame, len);
}

// ---------------------------------------------------------------------------
// Bluetooth controller and wheels

void Bridge::setGamepad(const GamepadInput& pad, uint32_t nowMs) {
  drive_.setGamepad(pad, nowMs);
}

bool Bridge::takeMotorCommands(int8_t (&out)[kNumMotors]) {
  if (!motorCommandsDue_) return false;
  motorCommandsDue_ = false;
  for (int i = 0; i < kNumMotors; ++i) out[i] = drive_.command(i);
  return true;
}

void Bridge::reportMotorBoard(bool ok) {
  ++motorWrites_;
  if (!ok) ++motorFailures_;
  motorBoardOk_ = ok;
}

void Bridge::setLocalVbat(float volts, uint32_t nowMs) {
  localVbat_ = volts;
  haveLocalVbat_ = true;
  tlm_.setLocalBattery(volts, nowMs);
}

// ---------------------------------------------------------------------------
// Periodic work

void Bridge::tick(uint32_t nowMs) {
  nowMs_ = nowMs;
  rc_.update(cfg_, nowMs);
  if (drive_.update(cfg_, rc_, nowMs)) motorCommandsDue_ = true;

  if (cfg_.tlmAutoMode) tlm_.setLocalMode(rc_.armed() ? "ARMED" : "DISARMED", nowMs);
  tlm_.expire(cfg_, nowMs);
  // Telemetry only reaches the handset while the link is up.
  if (rc_.linkUp()) {
    uint8_t frame[crsf::kMaxFrameSize];
    const size_t len = tlm_.poll(frame, cfg_, nowMs);
    if (len) sendCrsf(frame, len);
  }

  for (int i = 0; i < kNumPorts; ++i) streamPort(ports_[i], nowMs);
}

}  // namespace eb
