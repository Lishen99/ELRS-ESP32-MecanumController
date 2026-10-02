#include "eb_config.h"

#include <stdio.h>
#include <string.h>

#include "eb_platform.h"
#include "eb_util.h"

namespace eb {

namespace {

const char kOnOff[] = "off,on";
const char kStreamModes[] = "off,bin,text";
const char kOutTypes[] = "bipolar,unipolar";
const char kAuxModes[] = "servo,switch,armed,link";
const char kLedModes[] = "high,low,rgb";
const char kDriveSources[] = "off,auto,elrs,bt";
const char kDriveFailsafes[] = "ramp,stop";
const char kSpeedControls[] = "switch,throttle";
const char kWheels[] = "fl,fr,bl,br";
const char kMotorTypes[] = "no_encoder,tt,n20,jgb37";

#define EB_G(field) ((uint16_t)offsetof(Config, field))
#define EB_O(field) ((uint16_t)offsetof(OutputCfg, field))
#define EB_A(field) ((uint16_t)offsetof(AuxCfg, field))
#define EB_M(field) ((uint16_t)offsetof(MotorCfg, field))

const ParamDef kGlobalParams[] = {
    {"crsf_rx_pin", kParamI8, kParamReboot | kParamPinIn, EB_G(crsfRxPin), 0, 48, nullptr,
     "GPIO wired to the receiver's TX pad"},
    {"crsf_tx_pin", kParamI8, kParamReboot | kParamPinOut, EB_G(crsfTxPin), 0, 48, nullptr,
     "GPIO wired to the receiver's RX pad"},
    {"crsf_baud", kParamU32, kParamReboot, EB_G(crsfBaud), 9600, 5250000, nullptr,
     "receiver UART baud rate"},
    {"crsf_invert", kParamU8, kParamReboot, EB_G(crsfInvert), 0, 1, kOnOff,
     "invert the receiver UART signals"},
    {"host_rx_pin", kParamI8, kParamReboot | kParamPinIn, EB_G(hostRxPin), -1, 48, nullptr,
     "host UART: GPIO wired to the host's TX (-1 = off)"},
    {"host_tx_pin", kParamI8, kParamReboot | kParamPinOut, EB_G(hostTxPin), -1, 48, nullptr,
     "host UART: GPIO wired to the host's RX (-1 = off)"},
    {"host_baud", kParamU32, kParamReboot, EB_G(hostBaud), 9600, 3000000, nullptr,
     "host UART baud rate"},
    {"usb_baud", kParamU32, kParamReboot, EB_G(usbBaud), 9600, 3000000, nullptr,
     "USB serial baud rate"},
    {"host_stream", kParamU8, kParamStream, EB_G(hostStream), 0, 2, kStreamModes,
     "what the host UART sends from boot"},
    {"usb_stream", kParamU8, kParamStream, EB_G(usbStream), 0, 2, kStreamModes,
     "what the USB serial port sends from boot"},
    {"stream_hz", kParamU16, kParamStream, EB_G(streamHz), 1, 500, nullptr,
     "channel updates per second"},
    {"stream_ch", kParamU8, 0, EB_G(streamCh), 1, 16, nullptr,
     "number of outputs in each update"},
    {"link_hz", kParamU8, 0, EB_G(linkHz), 0, 50, nullptr,
     "link statistics messages per second (0 = off)"},
    {"fwd_crsf", kParamU8, 0, EB_G(fwdCrsf), 0, 1, kOnOff,
     "forward other CRSF frames from the receiver to the host"},
    {"failsafe_ms", kParamU16, 0, EB_G(failsafeMs), 100, 10000, nullptr,
     "time without RC frames before failsafe"},
    {"arm_ch", kParamU8, 0, EB_G(armCh), 0, 16, nullptr,
     "CRSF channel of the arm switch (0 = armed whenever linked)"},
    // Limited so that a switch at either end of its travel (about 1000 and
    // 2000 us) can always reach both the on and the off state.
    {"arm_us", kParamU16, 0, EB_G(armUs), 1100, 1900, nullptr,
     "arm switch counts as on above this pulse width"},
    {"arm_guard", kParamU8, 0, EB_G(armGuard), 0, 1, kOnOff,
     "require the arm switch to be seen off before arming"},
    {"tlm_period_ms", kParamU16, 0, EB_G(tlmPeriodMs), 0, 10000, nullptr,
     "gap between telemetry frames (0 = telemetry off)"},
    {"tlm_timeout_ms", kParamU16, 0, EB_G(tlmTimeoutMs), 100, 60000, nullptr,
     "stop sending a value the host has not refreshed for this long"},
    {"tlm_auto_mode", kParamU8, 0, EB_G(tlmAutoMode), 0, 1, kOnOff,
     "report ARMED/DISARMED as the mode when the host sends none"},
    {"vbat_pin", kParamI8, kParamReboot | kParamPinAdc, EB_G(vbatPin), -1, 48, nullptr,
     "ADC GPIO for battery voltage (-1 = off)"},
    {"vbat_scale", kParamF32, 0, EB_G(vbatScale), 0, 1000, nullptr,
     "battery divider ratio: pack volts per ADC volt"},
    {"led_pin", kParamI8, kParamReboot | kParamPinOut, EB_G(ledPin), -1, 48, nullptr,
     "status LED GPIO (-1 = off)"},
    {"led_mode", kParamU8, kParamReboot, EB_G(ledMode), 0, 2, kLedModes,
     "LED type: active high, active low, or RGB pixel"},
    {"pwm_hz", kParamU16, kParamReboot, EB_G(pwmHz), 50, 400, nullptr,
     "servo pulse rate of the aux outputs"},
    {"drive_src", kParamU8, 0, EB_G(driveSrc), 0, 3, kDriveSources,
     "who drives: auto = ELRS while linked, else the Bluetooth controller"},
    {"drive_fwd", kParamU8, 0, EB_G(driveFwd), 0, 16, nullptr,
     "output that drives forward / back (0 = none)"},
    {"drive_strafe", kParamU8, 0, EB_G(driveStrafe), 0, 16, nullptr,
     "output that drives sideways (0 = none)"},
    {"drive_turn", kParamU8, 0, EB_G(driveTurn), 0, 16, nullptr,
     "output that turns on the spot (0 = none)"},
    {"drive_speed", kParamU8, 0, EB_G(driveSpeed), 0, 16, nullptr,
     "output of the speed control (0 = always normal speed)"},
    {"drive_speed_type", kParamU8, 0, EB_G(driveSpeedType), 0, 1, kSpeedControls,
     "speed control: slow/normal/fast switch, or throttle (wheel/stick: bottom stopped, top full)"},
    {"drive_speed_top", kParamU8, 0, EB_G(driveSpeedTop), 0, 100, nullptr,
     "speed with the throttle all the way up, percent of drive_max"},
    {"drive_max", kParamU8, 0, EB_G(driveMax), 0, 100, nullptr,
     "largest speed command sent to the motor board"},
    {"drive_slow", kParamU8, 0, EB_G(driveSlow), 0, 100, nullptr,
     "slow speed mode, percent of drive_max"},
    {"drive_normal", kParamU8, 0, EB_G(driveNormal), 0, 100, nullptr,
     "normal speed mode, percent of drive_max"},
    {"drive_fast", kParamU8, 0, EB_G(driveFast), 0, 100, nullptr,
     "fast speed mode, percent of drive_max"},
    {"drive_accel", kParamU8, 0, EB_G(driveAccel), 1, 100, nullptr,
     "speed steps per 10 ms while speeding up"},
    {"drive_decel", kParamU8, 0, EB_G(driveDecel), 1, 100, nullptr,
     "speed steps per 10 ms while slowing down"},
    {"drive_fs", kParamU8, 0, EB_G(driveFs), 0, 1, kDriveFailsafes,
     "when nobody is in control any more: ramp down or stop at once"},
    {"bt_enable", kParamU8, kParamReboot, EB_G(btEnable), 0, 1, kOnOff,
     "accept a Bluetooth game controller (PS4 and others)"},
    {"bt_timeout_ms", kParamU16, 0, EB_G(btTimeoutMs), 100, 10000, nullptr,
     "silence from the Bluetooth controller before it counts as gone"},
    {"motor_sda", kParamI8, kParamReboot | kParamPinOut, EB_G(motorSdaPin), 0, 48, nullptr,
     "GPIO wired to the motor board's SDA"},
    {"motor_scl", kParamI8, kParamReboot | kParamPinOut, EB_G(motorSclPin), 0, 48, nullptr,
     "GPIO wired to the motor board's SCL"},
    {"motor_addr", kParamU8, kParamReboot, EB_G(motorAddr), 1, 127, nullptr,
     "I2C address of the motor board (52 = 0x34)"},
    {"motor_type", kParamU8, kParamReboot, EB_G(motorType), 0, 3, kMotorTypes,
     "motors fitted to the motor board"},
    {"motor_enc_pol", kParamU8, kParamReboot, EB_G(motorEncPol), 0, 1, nullptr,
     "encoder polarity: change it if a motor races instead of holding its speed"},
};

const ParamDef kOutputParams[] = {
    {"name", kParamStr, 0, EB_O(name), 0, 0, nullptr, "label for this output"},
    {"src", kParamU8, 0, EB_O(src), 0, 16, nullptr, "CRSF channel feeding this output (0 = none)"},
    {"type", kParamU8, 0, EB_O(type), 0, 1, kOutTypes,
     "bipolar = -1000..1000 (sticks), unipolar = 0..1000 (switches)"},
    {"rev", kParamU8, 0, EB_O(rev), 0, 1, kOnOff, "reverse direction"},
    {"min", kParamU16, 0, EB_O(minUs), 800, 2200, nullptr, "pulse width at the low end (us)"},
    {"mid", kParamU16, 0, EB_O(midUs), 800, 2200, nullptr, "pulse width at zero, bipolar only (us)"},
    {"max", kParamU16, 0, EB_O(maxUs), 800, 2200, nullptr, "pulse width at the high end (us)"},
    {"db", kParamU16, 0, EB_O(db), 0, 400, nullptr, "deadband (us)"},
    {"fs", kParamI16, 0, EB_O(fs), -1000, 1000, nullptr, "value reported during failsafe"},
    {"hold", kParamU8, 0, EB_O(hold), 0, 1, kOnOff, "on failsafe keep the last value instead of fs"},
};

const ParamDef kAuxParams[] = {
    {"pin", kParamI8, kParamReboot | kParamPinOut, EB_A(pin), -1, 48, nullptr,
     "GPIO for this output (-1 = off)"},
    {"mode", kParamU8, kParamReboot, EB_A(mode), 0, 3, kAuxModes,
     "servo pulses, switch level, wheels-live level, or link level"},
    {"src", kParamU8, 0, EB_A(src), 1, 16, nullptr, "mapped output driving this pin"},
    {"gate", kParamU8, 0, EB_A(gate), 0, 1, kOnOff, "only active while armed"},
    {"inv", kParamU8, 0, EB_A(inv), 0, 1, kOnOff, "invert the level (digital modes)"},
    {"min_us", kParamU16, 0, EB_A(minUs), 500, 2500, nullptr, "servo pulse at the low end"},
    {"max_us", kParamU16, 0, EB_A(maxUs), 500, 2500, nullptr, "servo pulse at the high end"},
    {"fs_us", kParamU16, 0, EB_A(fsUs), 0, 2500, nullptr,
     "servo pulse on failsafe or when disarmed (0 = no pulses)"},
};

const ParamDef kMotorParams[] = {
    {"wheel", kParamU8, 0, EB_M(wheel), 0, 3, kWheels,
     "wheel this motor board channel turns: front/back, left/right"},
    {"rev", kParamU8, 0, EB_M(rev), 0, 1, kOnOff, "reverse this motor"},
};

template <typename T, size_t N>
size_t countOf(const T (&)[N]) {
  return N;
}

char lower(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c; }

// Index of `text` within a comma-separated label list, ignoring case; -1 if absent.
int labelIndex(const char* labels, const char* text) {
  int index = 0;
  const char* p = labels;
  while (*p) {
    const char* t = text;
    while (*p && *p != ',' && *t && lower(*p) == lower(*t)) {
      ++p;
      ++t;
    }
    if ((*p == ',' || *p == '\0') && *t == '\0') return index;
    while (*p && *p != ',') ++p;
    if (*p == ',') ++p;
    ++index;
  }
  return -1;
}

bool labelText(const char* labels, long index, char* buf, size_t cap) {
  if (index < 0) return false;
  const char* p = labels;
  for (long i = 0; i < index; ++i) {
    while (*p && *p != ',') ++p;
    if (*p != ',') return false;
    ++p;
  }
  size_t n = 0;
  while (p[n] && p[n] != ',' && n + 1 < cap) {
    buf[n] = p[n];
    ++n;
  }
  buf[n] = '\0';
  return n > 0;
}

long readInt(const ParamRef& ref) {
  switch (ref.def->type) {
    case kParamU8: return *(const uint8_t*)ref.ptr;
    case kParamI8: return *(const int8_t*)ref.ptr;
    case kParamU16: return *(const uint16_t*)ref.ptr;
    case kParamI16: return *(const int16_t*)ref.ptr;
    case kParamU32: return (long)*(const uint32_t*)ref.ptr;
    default: return 0;
  }
}

void writeInt(const ParamRef& ref, long value) {
  switch (ref.def->type) {
    case kParamU8: *(uint8_t*)ref.ptr = (uint8_t)value; break;
    case kParamI8: *(int8_t*)ref.ptr = (int8_t)value; break;
    case kParamU16: *(uint16_t*)ref.ptr = (uint16_t)value; break;
    case kParamI16: *(int16_t*)ref.ptr = (int16_t)value; break;
    case kParamU32: *(uint32_t*)ref.ptr = (uint32_t)value; break;
    default: break;
  }
}

const ParamDef* findDef(const ParamDef* table, size_t count, const char* name) {
  for (size_t i = 0; i < count; ++i) {
    if (strcmp(table[i].name, name) == 0) return &table[i];
  }
  return nullptr;
}

// Matches "<prefix><number>." and yields the 1-based number and what follows.
bool groupIndex(const char* name, const char* prefix, int maxIndex, int& index,
                const char*& rest) {
  const size_t n = strlen(prefix);
  if (strncmp(name, prefix, n) != 0) return false;
  const char* p = name + n;
  if (*p < '0' || *p > '9') return false;
  int value = 0;
  while (*p >= '0' && *p <= '9') {
    value = value * 10 + (*p - '0');
    if (value > 99) return false;
    ++p;
  }
  if (*p != '.' || value < 1 || value > maxIndex) return false;
  index = value;
  rest = p + 1;
  return true;
}

struct TextCtx {
  const Config* cfg;
  const Config* defaults;
  char* buf;
  size_t cap;
  size_t len;
  bool overflow;
};

void textVisitor(void* opaque, const char* name, const ParamRef& ref) {
  TextCtx& ctx = *(TextCtx*)opaque;
  ParamRef def = ref;
  def.ptr = (char*)ctx.defaults + ((const char*)ref.ptr - (const char*)ctx.cfg);
  char current[32];
  char standard[32];
  paramFormat(ref, current, sizeof current);
  paramFormat(def, standard, sizeof standard);
  if (strcmp(current, standard) == 0) return;
  const size_t room = ctx.cap - ctx.len;
  const int n = snprintf(ctx.buf + ctx.len, room, "%s %s\n", name, current);
  if (n < 0 || (size_t)n >= room) {
    ctx.overflow = true;
    return;
  }
  ctx.len += (size_t)n;
}

const char kConfigHeader[] = "# elrs-bridge settings v1\n";

}  // namespace

void configDefaults(Config& cfg) {
  memset(&cfg, 0, sizeof cfg);

  cfg.crsfRxPin = 16;
  cfg.crsfTxPin = 17;
  cfg.crsfInvert = 0;
  cfg.crsfBaud = 420000;

  cfg.hostRxPin = 19;
  cfg.hostTxPin = 18;
  cfg.hostBaud = 115200;
  cfg.usbBaud = 115200;
  cfg.hostStream = 1;  // binary
  cfg.usbStream = 0;   // off: interactive CLI
  cfg.streamHz = 50;
  cfg.streamCh = kNumOutputs;
  cfg.linkHz = 5;
  cfg.fwdCrsf = 0;

  cfg.failsafeMs = 500;
  cfg.armCh = 5;       // ExpressLRS treats CH5 (AUX1) as the arm switch
  cfg.armUs = 1600;
  cfg.armGuard = 1;

  cfg.tlmPeriodMs = 100;
  cfg.tlmTimeoutMs = 3000;
  cfg.tlmAutoMode = 1;
  cfg.vbatPin = -1;
  cfg.vbatScale = 11.0f;  // 100k over 10k divider

  cfg.ledPin = 2;
  cfg.ledMode = kLedActiveHigh;
  cfg.pwmHz = 50;

  // Driving. The ELRS side follows the usual layout of a mode 2 handset for a
  // rover: the right stick moves (CH2 forward, CH1 sideways) and the left
  // stick turns (CH4). The speed comes from a scroll wheel or knob mixed to
  // CH12, the aux channel ExpressLRS sends in the finest steps in either
  // switch mode. The other numbers are the ones the robot's original PS4
  // firmware used.
  cfg.driveSrc = kDriveAuto;
  cfg.driveFwd = 2;
  cfg.driveStrafe = 1;
  cfg.driveTurn = 4;
  cfg.driveSpeed = 12;
  cfg.driveSpeedType = kSpeedThrottle;
  cfg.driveSpeedTop = 80;
  cfg.driveMax = 60;
  cfg.driveSlow = 30;
  cfg.driveNormal = 50;
  cfg.driveFast = 100;
  cfg.driveAccel = 2;
  cfg.driveDecel = 1;
  cfg.driveFs = kDriveFsRamp;

  cfg.btEnable = 1;
  cfg.btTimeoutMs = 500;

  cfg.motorSdaPin = 21;
  cfg.motorSclPin = 22;
  cfg.motorAddr = 0x34;
  cfg.motorType = kMotorJgb37;
  cfg.motorEncPol = 0;
  // How the original robot is wired: M1 right rear, M2 right front
  // (reversed), M3 left rear (reversed), M4 left front.
  static const MotorCfg kMotors[kNumMotors] = {
      {kWheelBackRight, 0}, {kWheelFrontRight, 1}, {kWheelBackLeft, 1}, {kWheelFrontLeft, 0}};
  for (int i = 0; i < kNumMotors; ++i) cfg.motor[i] = kMotors[i];

  for (int i = 0; i < kNumOutputs; ++i) {
    OutputCfg& o = cfg.out[i];
    snprintf(o.name, sizeof o.name, "ch%d", i + 1);
    o.src = (uint8_t)(i + 1);
    // Channels 1-4 are the sticks; the rest are switches and sliders. In its
    // standard packet modes ExpressLRS sends the sticks as 988..2012 us but
    // the aux channels as 1000..2000 us, so each group gets matching end
    // points and a switch reads exactly 0 or 1000.
    const bool stick = i < 4;
    o.type = stick ? kOutBipolar : kOutUnipolar;
    o.rev = 0;
    o.hold = 0;
    o.fs = 0;
    o.minUs = stick ? 988 : 1000;
    o.midUs = 1500;
    o.maxUs = stick ? 2012 : 2000;
    o.db = stick ? 20 : 0;
  }

  for (int i = 0; i < kNumAux; ++i) {
    AuxCfg& a = cfg.aux[i];
    a.pin = -1;
    a.mode = kAuxServo;
    a.src = (uint8_t)(i + 1);
    a.gate = 1;
    a.inv = 0;
    a.minUs = 1000;
    a.maxUs = 2000;
    a.fsUs = 0;
  }
}

bool paramFind(Config& cfg, const char* name, ParamRef& ref) {
  int index = 0;
  const char* rest = nullptr;
  if (groupIndex(name, "out", kNumOutputs, index, rest)) {
    const ParamDef* def = findDef(kOutputParams, countOf(kOutputParams), rest);
    if (!def) return false;
    ref.def = def;
    ref.ptr = (char*)&cfg.out[index - 1] + def->offset;
    return true;
  }
  if (groupIndex(name, "aux", kNumAux, index, rest)) {
    const ParamDef* def = findDef(kAuxParams, countOf(kAuxParams), rest);
    if (!def) return false;
    ref.def = def;
    ref.ptr = (char*)&cfg.aux[index - 1] + def->offset;
    return true;
  }
  if (groupIndex(name, "motor", kNumMotors, index, rest)) {
    const ParamDef* def = findDef(kMotorParams, countOf(kMotorParams), rest);
    if (!def) return false;
    ref.def = def;
    ref.ptr = (char*)&cfg.motor[index - 1] + def->offset;
    return true;
  }
  const ParamDef* def = findDef(kGlobalParams, countOf(kGlobalParams), name);
  if (!def) return false;
  ref.def = def;
  ref.ptr = (char*)&cfg + def->offset;
  return true;
}

const char* paramSet(const ParamRef& ref, const char* text, Platform& platform) {
  const ParamDef& def = *ref.def;

  if (def.type == kParamStr) {
    const size_t n = strlen(text);
    if (n < 1 || n >= (size_t)kNameSize) return "name must be 1-11 characters";
    for (size_t i = 0; i < n; ++i) {
      const char c = text[i];
      const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                      (c >= '0' && c <= '9') || c == '_' || c == '-';
      if (!ok) return "name may only contain letters, digits, _ and -";
    }
    memset(ref.ptr, 0, kNameSize);
    memcpy(ref.ptr, text, n);
    return nullptr;
  }

  if (def.type == kParamF32) {
    float value;
    if (!parseFloat(text, value)) return "not a number";
    if (value < (float)def.min || value > (float)def.max) return "out of range";
    *(float*)ref.ptr = value;
    return nullptr;
  }

  long value;
  const int label = def.labels ? labelIndex(def.labels, text) : -1;
  if (label >= 0) {
    value = label;
  } else if (!parseLong(text, value)) {
    return def.labels ? "not one of the allowed values" : "not a number";
  }
  if (value < def.min || value > def.max) return "out of range";
  if ((def.flags & kParamPinMask) && value >= 0) {
    const PinUse use = (def.flags & kParamPinOut)   ? kPinOutput
                       : (def.flags & kParamPinAdc) ? kPinAnalog
                                                    : kPinInput;
    if (!platform.pinUsable((int)value, use)) {
      return "that GPIO cannot be used for this on this board";
    }
  }
  writeInt(ref, value);
  return nullptr;
}

void paramFormat(const ParamRef& ref, char* buf, size_t cap) {
  const ParamDef& def = *ref.def;
  if (def.type == kParamStr) {
    snprintf(buf, cap, "%s", (const char*)ref.ptr);
    return;
  }
  if (def.type == kParamF32) {
    snprintf(buf, cap, "%.6g", (double)*(const float*)ref.ptr);
    return;
  }
  const long value = readInt(ref);
  if (def.labels && labelText(def.labels, value, buf, cap)) return;
  snprintf(buf, cap, "%ld", value);
}

void paramForEach(Config& cfg, ParamVisitor visit, void* ctx) {
  char name[24];
  ParamRef ref;
  for (size_t i = 0; i < countOf(kGlobalParams); ++i) {
    ref.def = &kGlobalParams[i];
    ref.ptr = (char*)&cfg + ref.def->offset;
    visit(ctx, ref.def->name, ref);
  }
  for (int o = 0; o < kNumOutputs; ++o) {
    for (size_t i = 0; i < countOf(kOutputParams); ++i) {
      ref.def = &kOutputParams[i];
      ref.ptr = (char*)&cfg.out[o] + ref.def->offset;
      snprintf(name, sizeof name, "out%d.%s", o + 1, ref.def->name);
      visit(ctx, name, ref);
    }
  }
  for (int a = 0; a < kNumAux; ++a) {
    for (size_t i = 0; i < countOf(kAuxParams); ++i) {
      ref.def = &kAuxParams[i];
      ref.ptr = (char*)&cfg.aux[a] + ref.def->offset;
      snprintf(name, sizeof name, "aux%d.%s", a + 1, ref.def->name);
      visit(ctx, name, ref);
    }
  }
  for (int m = 0; m < kNumMotors; ++m) {
    for (size_t i = 0; i < countOf(kMotorParams); ++i) {
      ref.def = &kMotorParams[i];
      ref.ptr = (char*)&cfg.motor[m] + ref.def->offset;
      snprintf(name, sizeof name, "motor%d.%s", m + 1, ref.def->name);
      visit(ctx, name, ref);
    }
  }
}

bool configToText(const Config& cfg, const Config& defaults, char* buf, size_t cap,
                  size_t& len) {
  const size_t headerLen = sizeof kConfigHeader - 1;
  if (cap <= headerLen) return false;
  memcpy(buf, kConfigHeader, headerLen);

  TextCtx ctx;
  ctx.cfg = &cfg;
  ctx.defaults = &defaults;
  ctx.buf = buf;
  ctx.cap = cap;
  ctx.len = headerLen;
  ctx.overflow = false;
  // The visitor only reads through the refs it is handed.
  paramForEach(const_cast<Config&>(cfg), textVisitor, &ctx);
  if (ctx.overflow) return false;
  len = ctx.len;
  return true;
}

int configFromText(Config& cfg, const char* text, size_t len, Platform& platform) {
  int skipped = 0;
  size_t pos = 0;
  while (pos < len) {
    size_t end = pos;
    while (end < len && text[end] != '\n') ++end;
    const size_t n = end - pos;

    char line[64];
    if (n >= sizeof line) {
      ++skipped;
    } else {
      memcpy(line, text + pos, n);
      line[n] = '\0';
      size_t trimmed = n;
      while (trimmed > 0 && (line[trimmed - 1] == '\r' || line[trimmed - 1] == ' ' ||
                             line[trimmed - 1] == '\t')) {
        line[--trimmed] = '\0';
      }
      char* name = line;
      while (*name == ' ' || *name == '\t') ++name;
      if (*name != '\0' && *name != '#') {
        char* value = name;
        while (*value && *value != ' ' && *value != '\t' && *value != '=') ++value;
        if (*value == '\0') {
          ++skipped;
        } else {
          *value++ = '\0';
          while (*value == ' ' || *value == '\t' || *value == '=') ++value;
          ParamRef ref;
          if (!paramFind(cfg, name, ref) || paramSet(ref, value, platform) != nullptr) {
            ++skipped;
          }
        }
      }
    }
    pos = end + 1;
  }
  return skipped;
}

}  // namespace eb
