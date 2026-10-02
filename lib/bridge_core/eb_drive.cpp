#include "eb_drive.h"

#include <string.h>

#include "eb_util.h"

namespace eb {

namespace {

const int32_t kFull = 1000;          // full stick, in thousandths
const int32_t kPadDeadband = 100;    // 10 % of full stick
const int32_t kPadFullScale = 512;   // Bluepad32 stick range is -512..511

// One mapped output as -1000..1000, or 0 when none is assigned.
int32_t outputValue(const Rc& rc, uint8_t output) {
  return (output >= 1 && output <= kNumOutputs) ? rc.out(output - 1) : 0;
}

int32_t absValue(int32_t v) { return v < 0 ? -v : v; }

}  // namespace

Drive::Drive()
    : padMs_(0),
      padFresh_(false),
      padReady_(false),
      source_(kNobody),
      live_(false),
      percent_(0),
      started_(false),
      lastStepMs_(0) {
  memset(&pad_, 0, sizeof pad_);
  memset(target_, 0, sizeof target_);
  memset(command_, 0, sizeof command_);
}

void Drive::setGamepad(const GamepadInput& pad, uint32_t nowMs) {
  pad_ = pad;
  padMs_ = nowMs;
  padFresh_ = pad.connected;
}

bool Drive::update(const Config& cfg, const Rc& rc, uint32_t nowMs) {
  // A controller that falls silent counts as gone. Latched, like the radio
  // link, so the clock wrapping round can never bring an old report back.
  if (padFresh_ && (uint32_t)(nowMs - padMs_) > cfg.btTimeoutMs) padFresh_ = false;

  const Source previous = source_;
  source_ = pickSource(cfg, rc);
  // A Bluetooth controller that takes over must first show its sticks
  // centred, so the robot cannot lurch off with wherever they happen to be.
  if (source_ != previous) padReady_ = false;

  Motion m = {0, 0, 0, 0};
  bool stopAtOnce = false;
  live_ = false;
  switch (source_) {
    case kElrs:
      if (rc.armed()) {
        m = fromElrs(cfg, rc);
        live_ = true;
      } else {
        stopAtOnce = true;  // the arm switch is the kill switch
      }
      break;
    case kGamepad: {
      const Motion asked = fromGamepad(cfg, pad_);
      if (asked.forward == 0 && asked.strafe == 0 && asked.turn == 0) padReady_ = true;
      if (padReady_) {
        m = asked;
        live_ = true;
      }
      break;
    }
    default:
      // Nobody in charge: the link was lost or the controller went away.
      stopAtOnce = cfg.driveSrc == kDriveOff || cfg.driveFs == kDriveFsStop;
      break;
  }
  aimAt(cfg, m);
  percent_ = m.percent;  // still all zero unless live

  if (stopAtOnce) {
    bool moving = false;
    for (int i = 0; i < kNumMotors; ++i) moving |= command_[i] != 0;
    if (moving) {
      memset(command_, 0, sizeof command_);
      lastStepMs_ = nowMs;
      return true;
    }
  }

  if (started_ && (uint32_t)(nowMs - lastStepMs_) < kDrivePeriodMs) return false;
  started_ = true;
  lastStepMs_ = nowMs;
  for (int i = 0; i < kNumMotors; ++i) command_[i] = moveToward(command_[i], target_[i], cfg);
  return true;
}

bool Drive::moving() const {
  for (int i = 0; i < kNumMotors; ++i) {
    if (command_[i] != 0 || target_[i] != 0) return true;
  }
  return false;
}

Drive::Source Drive::pickSource(const Config& cfg, const Rc& rc) const {
  switch (cfg.driveSrc) {
    case kDriveAuto:
      // While its link is up the ELRS handset is in charge, armed or not.
      if (rc.linkUp()) return kElrs;
      return padFresh_ ? kGamepad : kNobody;
    case kDriveElrs:
      return rc.linkUp() ? kElrs : kNobody;
    case kDriveBt:
      return padFresh_ ? kGamepad : kNobody;
    default:
      return kNobody;
  }
}

Drive::Motion Drive::fromElrs(const Config& cfg, const Rc& rc) {
  Motion m;
  m.forward = outputValue(rc, cfg.driveFwd);
  m.strafe = outputValue(rc, cfg.driveStrafe);
  m.turn = outputValue(rc, cfg.driveTurn);
  m.percent = cfg.driveNormal;
  if (cfg.driveSpeed < 1 || cfg.driveSpeed > kNumOutputs) return m;

  const bool unipolar = cfg.out[cfg.driveSpeed - 1].type == kOutUnipolar;
  const int32_t v = rc.out(cfg.driveSpeed - 1);
  if (cfg.driveSpeedType == kSpeedThrottle) {
    // Anywhere from stopped at the bottom to drive_speed_top at the top.
    const int32_t wayUp = clampT<int32_t>(unipolar ? v : (v + kFull) / 2, 0, kFull);
    m.percent = wayUp * cfg.driveSpeedTop / kFull;
  } else {
    // A three-position switch: low slow, middle normal, high fast. The
    // thresholds sit halfway between the positions, whichever way the
    // output is scaled.
    if (v < (unipolar ? 250 : -500)) {
      m.percent = cfg.driveSlow;
    } else if (v > (unipolar ? 750 : 500)) {
      m.percent = cfg.driveFast;
    }
  }
  return m;
}

int32_t Drive::padAxis(int16_t raw) {
  const int32_t v = clampT<int32_t>((int32_t)raw * kFull / kPadFullScale, -kFull, kFull);
  return absValue(v) < kPadDeadband ? 0 : v;
}

Drive::Motion Drive::fromGamepad(const Config& cfg, const GamepadInput& pad) {
  // Left stick moves, right stick turns; L2 held is slow, R2 held is fast.
  Motion m;
  m.forward = -padAxis(pad.ly);  // the stick reads negative when pushed up
  m.strafe = padAxis(pad.lx);
  m.turn = padAxis(pad.rx);
  m.percent = pad.l2 ? cfg.driveSlow : (pad.r2 ? cfg.driveFast : cfg.driveNormal);
  return m;
}

void Drive::aimAt(const Config& cfg, const Motion& m) {
  // The standard mecanum mix.
  int32_t wheel[4];
  wheel[kWheelFrontLeft] = m.forward + m.strafe + m.turn;
  wheel[kWheelFrontRight] = m.forward - m.strafe - m.turn;
  wheel[kWheelBackLeft] = m.forward - m.strafe + m.turn;
  wheel[kWheelBackRight] = m.forward + m.strafe - m.turn;

  // If any wheel is asked for more than full speed, scale all four down
  // together so the robot keeps its direction.
  int32_t most = kFull;
  for (int i = 0; i < 4; ++i) {
    if (absValue(wheel[i]) > most) most = absValue(wheel[i]);
  }
  for (int i = 0; i < 4; ++i) {
    const int32_t v = wheel[i] * kFull / most * m.percent / 100;
    wheel[i] = v * cfg.driveMax / kFull;  // motor board units, rounded toward zero
  }

  for (int i = 0; i < kNumMotors; ++i) {
    const MotorCfg& motor = cfg.motor[i];
    int32_t v = motor.wheel < 4 ? wheel[motor.wheel] : 0;
    if (motor.rev) v = -v;
    target_[i] = (int8_t)clampT<int32_t>(v, -(int32_t)cfg.driveMax, cfg.driveMax);
  }
}

// One ramp step from `current` towards `target`: drive_accel while speeding
// up, drive_decel while slowing down.
int8_t Drive::moveToward(int8_t current, int8_t target, const Config& cfg) {
  int32_t step = absValue(target) < absValue(current) ? cfg.driveDecel : cfg.driveAccel;
  // When reversing, come down to zero at the slowing-down rate first.
  if ((current > 0 && target < 0) || (current < 0 && target > 0)) step = cfg.driveDecel;

  int32_t next = current;
  if (current < target) {
    next = current + step;
    if (next > target) next = target;
  } else if (current > target) {
    next = current - step;
    if (next < target) next = target;
  }
  return (int8_t)next;
}

}  // namespace eb
