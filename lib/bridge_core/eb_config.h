// Runtime configuration of the bridge and the name-based parameter registry
// behind the CLI's get/set/dump and the saved-settings format.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace eb {

class Platform;

const int kNumOutputs = 16;   // mapped outputs reported to the host
const int kNumAux = 4;        // optional direct PWM / digital pins
const int kNumMotors = 4;     // channels of the motor board
const int kNameSize = 12;     // output name storage, including the terminator

// Largest saved-settings text. Settings are stored as "name value" lines for
// the parameters that differ from the defaults, so this is rarely approached.
const size_t kConfigTextMax = 6144;

enum OutType {
  kOutBipolar = 0,   // -1000..+1000 around the centre, for sticks
  kOutUnipolar = 1   // 0..+1000 from the low end, for switches and sliders
};

enum AuxMode {
  kAuxServo = 0,   // servo/ESC pulses following a mapped output
  kAuxSwitch = 1,  // digital level, high when a mapped output is in its upper half
  kAuxArmed = 2,   // digital level, high while the wheels may move (a motor-enable line)
  kAuxLink = 3     // digital level, high while the radio link is up
};

enum LedMode {
  kLedActiveHigh = 0,
  kLedActiveLow = 1,
  kLedRgb = 2        // addressable RGB LED (a WS2812 pixel)
};

// Who may drive the wheels.
enum DriveSource {
  kDriveOff = 0,    // nobody: the motors stay stopped
  kDriveAuto = 1,   // the ELRS handset while its link is up, otherwise the Bluetooth controller
  kDriveElrs = 2,   // only the ELRS handset
  kDriveBt = 3      // only the Bluetooth controller
};

// How the ELRS speed control (drive_speed) is read.
enum SpeedControl {
  kSpeedSwitch = 0,   // a three-position switch: slow / normal / fast
  kSpeedThrottle = 1  // a scroll wheel, knob or throttle stick: stopped at the bottom, full speed at the top
};

// What the wheels do once nobody is in control any more: the radio link is
// lost, or the Bluetooth controller has gone.
enum DriveFailsafe {
  kDriveFsRamp = 0,  // slow down at drive_decel, as when the sticks are let go
  kDriveFsStop = 1   // stop at once
};

enum Wheel {
  kWheelFrontLeft = 0,
  kWheelFrontRight = 1,
  kWheelBackLeft = 2,
  kWheelBackRight = 3
};

// The motor board's motor types (Hiwonder numbering).
enum MotorType {
  kMotorNoEncoder = 0,
  kMotorTT = 1,
  kMotorN20 = 2,
  kMotorJgb37 = 3   // JGB37-520, 12 V, 110 rpm
};

// One mapped output: which CRSF channel feeds it and how it is scaled.
struct OutputCfg {
  char name[kNameSize];
  uint8_t src;      // CRSF channel 1..16, 0 = unused (always reports `fs`)
  uint8_t type;     // OutType
  uint8_t rev;      // reverse direction
  uint8_t hold;     // on failsafe: 1 = keep the last live value, 0 = use `fs`
  int16_t fs;       // failsafe value in output units
  uint16_t minUs;   // pulse width mapped to the low end
  uint16_t midUs;   // pulse width mapped to zero (bipolar only)
  uint16_t maxUs;   // pulse width mapped to the high end
  uint16_t db;      // deadband in microseconds
};

// One direct pin driven by the bridge itself, independent of the host.
struct AuxCfg {
  int8_t pin;       // GPIO, -1 = disabled
  uint8_t mode;     // AuxMode
  uint8_t src;      // mapped output 1..16 (servo and switch modes)
  uint8_t gate;     // 1 = only active while armed
  uint8_t inv;      // invert the level (digital modes)
  uint16_t minUs;   // servo pulse at the low end of the output
  uint16_t maxUs;   // servo pulse at the high end of the output
  uint16_t fsUs;    // servo pulse on failsafe or when gated off, 0 = no pulses
};

// One channel of the motor board: which wheel it turns, and which way round.
struct MotorCfg {
  uint8_t wheel;    // Wheel
  uint8_t rev;      // a positive command turns this wheel backwards
};

struct Config {
  // receiver (CRSF) port
  int8_t crsfRxPin;
  int8_t crsfTxPin;
  uint8_t crsfInvert;
  uint32_t crsfBaud;

  // host ports
  int8_t hostRxPin;
  int8_t hostTxPin;
  uint32_t hostBaud;
  uint32_t usbBaud;
  uint8_t hostStream;   // elrsb::StreamMode at boot for the host UART
  uint8_t usbStream;    // elrsb::StreamMode at boot for the USB serial port
  uint16_t streamHz;
  uint8_t streamCh;
  uint8_t linkHz;
  uint8_t fwdCrsf;

  // failsafe and arming
  uint16_t failsafeMs;
  uint8_t armCh;
  uint16_t armUs;
  uint8_t armGuard;

  // telemetry
  uint16_t tlmPeriodMs;
  uint16_t tlmTimeoutMs;
  uint8_t tlmAutoMode;
  int8_t vbatPin;
  float vbatScale;

  // indicators and direct outputs
  int8_t ledPin;
  uint8_t ledMode;
  uint16_t pwmHz;

  // driving: what the sticks do to the wheels
  uint8_t driveSrc;      // DriveSource
  uint8_t driveFwd;      // mapped output for forward / back (1..16, 0 = none)
  uint8_t driveStrafe;   // mapped output for sideways
  uint8_t driveTurn;     // mapped output for turning on the spot
  uint8_t driveSpeed;    // mapped output of the speed control (0 = always normal speed)
  uint8_t driveSpeedType;  // SpeedControl
  uint8_t driveSpeedTop;   // speed with a throttle all the way up, percent of driveMax
  uint8_t driveMax;      // largest speed command sent to the motor board
  uint8_t driveSlow;     // speed modes, in percent of driveMax
  uint8_t driveNormal;
  uint8_t driveFast;
  uint8_t driveAccel;    // speed steps per 10 ms while speeding up
  uint8_t driveDecel;    // speed steps per 10 ms while slowing down
  uint8_t driveFs;       // DriveFailsafe

  // Bluetooth game controller
  uint8_t btEnable;
  uint16_t btTimeoutMs;  // silence before the controller counts as gone

  // motor board (Hiwonder 4-channel encoder motor driver, I2C)
  int8_t motorSdaPin;
  int8_t motorSclPin;
  uint8_t motorAddr;
  uint8_t motorType;     // MotorType
  uint8_t motorEncPol;   // encoder polarity

  OutputCfg out[kNumOutputs];
  AuxCfg aux[kNumAux];
  MotorCfg motor[kNumMotors];
};

// Board-independent defaults. Platform::boardDefaults() then adjusts pins.
void configDefaults(Config& cfg);

enum ParamType { kParamU8, kParamI8, kParamU16, kParamI16, kParamU32, kParamF32, kParamStr };

enum ParamFlags {
  kParamReboot = 0x01,  // takes effect after save + reboot
  kParamPinIn = 0x02,   // GPIO used as a digital input
  kParamPinOut = 0x04,  // GPIO used as a digital output
  kParamPinAdc = 0x08,  // GPIO used as an analog input
  kParamStream = 0x10   // changes a port's stream setup immediately
};
const uint8_t kParamPinMask = kParamPinIn | kParamPinOut | kParamPinAdc;

struct ParamDef {
  const char* name;     // for outputs/aux this is the part after "outN." / "auxN."
  uint8_t type;         // ParamType
  uint8_t flags;        // ParamFlags
  uint16_t offset;      // within Config, OutputCfg or AuxCfg
  int32_t min;
  int32_t max;
  const char* labels;   // comma-separated names for enumerated values, or null
  const char* help;
};

struct ParamRef {
  const ParamDef* def;
  void* ptr;
};

// Look up "stream_hz", "out3.rev", "aux1.pin", "motor2.wheel", ...
bool paramFind(Config& cfg, const char* name, ParamRef& ref);

// Parse `text` and store it. Returns null on success or a short error message.
const char* paramSet(const ParamRef& ref, const char* text, Platform& platform);

// Human-readable current value.
void paramFormat(const ParamRef& ref, char* buf, size_t cap);

typedef void (*ParamVisitor)(void* ctx, const char* name, const ParamRef& ref);
void paramForEach(Config& cfg, ParamVisitor visit, void* ctx);

// Saved-settings text: one "name value" line per parameter that differs from
// `defaults`. Returns false if `buf` is too small.
bool configToText(const Config& cfg, const Config& defaults, char* buf, size_t cap, size_t& len);

// Apply saved-settings text on top of `cfg`. Unknown or invalid lines are
// skipped so settings survive firmware updates; returns how many were skipped.
int configFromText(Config& cfg, const char* text, size_t len, Platform& platform);

}  // namespace eb
