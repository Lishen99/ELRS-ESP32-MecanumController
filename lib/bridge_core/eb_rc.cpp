#include "eb_rc.h"

#include "eb_crsf.h"
#include "eb_util.h"

namespace eb {

namespace {

// The arm switch must drop this far below arm_us before it counts as off.
const uint16_t kArmHysteresisUs = 50;

// How long the switch must read off before the guard lifts. One frame is not
// enough: right after the link comes up, a receiver may report a channel it
// has not yet received from the handset at a placeholder value. ELRS sends
// most switch channels in turn, one per packet, so at its slowest packet rate
// a full round takes about a third of a second.
const uint32_t kArmOffHoldMs = 500;

const int32_t kScale = 1000;

}  // namespace

Rc::Rc()
    : haveFrame_(false),
      fresh_(false),
      ageSaturated_(false),
      everLinked_(false),
      linkUp_(false),
      armed_(false),
      armSwitch_(false),
      armGuard_(true),
      armOffTiming_(false),
      armOffSinceMs_(0),
      lastFrameMs_(0),
      frames_(0),
      rateWindowMs_(0),
      rateWindowFrames_(0),
      rateHz_(0),
      failsafeCount_(0) {
  for (int i = 0; i < kNumOutputs; ++i) {
    us_[i] = 1500;
    out_[i] = 0;
    live_[i] = 0;
  }
}

void Rc::onChannels(const uint16_t* ticks, uint32_t nowMs) {
  for (int i = 0; i < kNumOutputs; ++i) us_[i] = crsf::ticksToUs(ticks[i]);
  haveFrame_ = true;
  fresh_ = true;
  ageSaturated_ = false;
  lastFrameMs_ = nowMs;
  ++frames_;
  ++rateWindowFrames_;
}

uint16_t Rc::ageMs(uint32_t nowMs) const {
  if (!haveFrame_ || ageSaturated_) return 0xFFFF;
  const uint32_t age = nowMs - lastFrameMs_;
  return age > 0xFFFF ? (uint16_t)0xFFFF : (uint16_t)age;
}

void Rc::requireRearm() {
  armed_ = false;
  armSwitch_ = false;
  armOffTiming_ = false;
  armGuard_ = true;
}

void Rc::update(const Config& cfg, uint32_t nowMs) {
  // Staleness is latched rather than recomputed from the timestamps, so the
  // millisecond counter wrapping (every 49.7 days) can never make an old
  // frame look recent again.
  const uint32_t age = nowMs - lastFrameMs_;
  if (fresh_ && age > cfg.failsafeMs) fresh_ = false;
  if (haveFrame_ && age >= 0xFFFF) ageSaturated_ = true;

  const bool up = fresh_;
  if (linkUp_ && !up && failsafeCount_ < 0xFFFF) ++failsafeCount_;
  linkUp_ = up;

  const uint32_t window = nowMs - rateWindowMs_;
  if (window >= 1000) {
    const uint32_t hz = (rateWindowFrames_ * 1000 + window / 2) / window;
    rateHz_ = hz > 0xFFFF ? (uint16_t)0xFFFF : (uint16_t)hz;
    rateWindowFrames_ = 0;
    rateWindowMs_ = nowMs;
  }

  updateArming(cfg, nowMs);

  for (int i = 0; i < kNumOutputs; ++i) {
    const OutputCfg& c = cfg.out[i];
    const bool mapped = c.src >= 1 && c.src <= kNumOutputs;
    if (up && mapped) {
      live_[i] = mapOutput(c, us_[c.src - 1]);
      out_[i] = live_[i];
    } else if (mapped && c.hold && everLinked_) {
      out_[i] = live_[i];
    } else {
      out_[i] = c.fs;
    }
  }
  if (up) everLinked_ = true;
}

// The guard is what stands between "the arm switch reads on" and "armed".
// It goes up when the link is lost (if arm_guard is set) and whenever the
// arming settings change; only the operator can take it down again.
void Rc::updateArming(const Config& cfg, uint32_t nowMs) {
  const bool hasSwitch = cfg.armCh >= 1 && cfg.armCh <= kNumOutputs;

  if (!linkUp_) {
    armed_ = false;
    armSwitch_ = false;
    armOffTiming_ = false;
    // What it will take to arm once the link is back.
    armGuard_ = hasSwitch && cfg.armGuard != 0;
    return;
  }

  if (!hasSwitch) {
    // No arm switch: armed follows the link. If that was set up while the
    // link was already up, it starts with the next connection.
    armed_ = !armGuard_;
    return;
  }

  const uint16_t sw = us_[cfg.armCh - 1];
  if (sw > cfg.armUs) {
    armSwitch_ = true;
  } else if ((uint32_t)sw + kArmHysteresisUs <= cfg.armUs) {
    armSwitch_ = false;
  }

  if (armSwitch_) {
    armOffTiming_ = false;
    if (!armGuard_) armed_ = true;
    return;
  }

  armed_ = false;
  // Seeing the switch off, and staying off for a moment, takes the guard down.
  if (!armOffTiming_) {
    armOffTiming_ = true;
    armOffSinceMs_ = nowMs;
  }
  if ((uint32_t)(nowMs - armOffSinceMs_) >= kArmOffHoldMs) armGuard_ = false;
}

int16_t Rc::mapOutput(const OutputCfg& cfg, uint16_t usIn) {
  const int32_t us = usIn;
  const int32_t db = cfg.db;
  int32_t value;

  if (cfg.type == kOutUnipolar) {
    const int32_t low = (int32_t)cfg.minUs + db;
    const int32_t span = (int32_t)cfg.maxUs - low;
    if (span <= 0) {
      value = us >= cfg.maxUs ? kScale : 0;
    } else {
      value = divRound((us - low) * kScale, span);
    }
    value = clampT<int32_t>(value, 0, kScale);
    if (cfg.rev) value = kScale - value;
  } else {
    const int32_t offset = us - (int32_t)cfg.midUs;
    if (offset > db) {
      const int32_t span = (int32_t)cfg.maxUs - cfg.midUs - db;
      value = span > 0 ? divRound((offset - db) * kScale, span) : kScale;
    } else if (offset < -db) {
      const int32_t span = (int32_t)cfg.midUs - cfg.minUs - db;
      value = span > 0 ? -divRound((-offset - db) * kScale, span) : -kScale;
    } else {
      value = 0;
    }
    value = clampT<int32_t>(value, -kScale, kScale);
    if (cfg.rev) value = -value;
  }
  return (int16_t)value;
}

}  // namespace eb
