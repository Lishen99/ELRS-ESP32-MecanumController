// Tests for the settings: their names, ranges and labels, and the text form
// they are saved in.
//
//   pio test -e native -f test_settings
#include <unity.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

#include <elrs_bridge_protocol.h>

#include "eb_config.h"
#include "fake_platform.h"
#include "text.h"

namespace {

// ---------------------------------------------------------------------------
// Names, ranges and labels

void test_param_set_and_format() {
  FakePlatform platform;
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::ParamRef ref;
  char buf[32];

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "stream_hz", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "100", platform));
  TEST_ASSERT_EQUAL_UINT16(100, cfg.streamHz);
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "0", platform));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "501", platform));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "abc", platform));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "12x", platform));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "", platform));
  TEST_ASSERT_EQUAL_UINT16(100, cfg.streamHz);  // rejected values leave it alone
  eb::paramFormat(ref, buf, sizeof buf);
  TEST_ASSERT_EQUAL_STRING("100", buf);

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "out3.fs", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "-750", platform));
  TEST_ASSERT_EQUAL_INT16(-750, cfg.out[2].fs);

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "vbat_scale", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "10.98", platform));
  eb::paramFormat(ref, buf, sizeof buf);
  TEST_ASSERT_EQUAL_STRING("10.98", buf);
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "nan", platform));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "-1", platform));

  // A leading zero is decimal, not octal.
  TEST_ASSERT_TRUE(eb::paramFind(cfg, "failsafe_ms", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "0800", platform));
  TEST_ASSERT_EQUAL_UINT16(800, cfg.failsafeMs);
}

void test_param_labels() {
  FakePlatform platform;
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::ParamRef ref;
  char buf[32];

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "host_stream", ref));
  eb::paramFormat(ref, buf, sizeof buf);
  TEST_ASSERT_EQUAL_STRING("bin", buf);
  TEST_ASSERT_NULL(eb::paramSet(ref, "text", platform));
  TEST_ASSERT_EQUAL_UINT8(elrsb::kStreamText, cfg.hostStream);
  TEST_ASSERT_NULL(eb::paramSet(ref, "OFF", platform));
  TEST_ASSERT_EQUAL_UINT8(elrsb::kStreamOff, cfg.hostStream);
  TEST_ASSERT_NULL(eb::paramSet(ref, "1", platform));
  TEST_ASSERT_EQUAL_UINT8(elrsb::kStreamBinary, cfg.hostStream);
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "binary", platform));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "3", platform));

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "out3.rev", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "on", platform));
  TEST_ASSERT_EQUAL_UINT8(1, cfg.out[2].rev);
  eb::paramFormat(ref, buf, sizeof buf);
  TEST_ASSERT_EQUAL_STRING("on", buf);

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "out7.type", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "bipolar", platform));
  TEST_ASSERT_EQUAL_UINT8(eb::kOutBipolar, cfg.out[6].type);

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "aux2.mode", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "armed", platform));
  TEST_ASSERT_EQUAL_UINT8(eb::kAuxArmed, cfg.aux[1].mode);
}

void test_param_names_and_lookup() {
  FakePlatform platform;
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::ParamRef ref;

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "out1.name", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "steer", platform));
  TEST_ASSERT_EQUAL_STRING("steer", cfg.out[0].name);
  TEST_ASSERT_NULL(eb::paramSet(ref, "left_wheel1", platform));  // 11 characters
  TEST_ASSERT_EQUAL_STRING("left_wheel1", cfg.out[0].name);
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "left_wheel12", platform));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "has space", platform));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "", platform));
  TEST_ASSERT_EQUAL_STRING("left_wheel1", cfg.out[0].name);

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "out16.src", ref));
  TEST_ASSERT_TRUE(eb::paramFind(cfg, "aux4.pin", ref));
  TEST_ASSERT_FALSE(eb::paramFind(cfg, "out0.src", ref));
  TEST_ASSERT_FALSE(eb::paramFind(cfg, "out17.src", ref));
  TEST_ASSERT_FALSE(eb::paramFind(cfg, "aux5.pin", ref));
  TEST_ASSERT_FALSE(eb::paramFind(cfg, "out1.nope", ref));
  TEST_ASSERT_FALSE(eb::paramFind(cfg, "out1", ref));
  TEST_ASSERT_FALSE(eb::paramFind(cfg, "nope", ref));
  TEST_ASSERT_FALSE(eb::paramFind(cfg, "", ref));
}

void test_param_pin_validation() {
  FakePlatform platform;
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::ParamRef ref;

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "crsf_rx_pin", ref));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "3", platform));    // USB serial pin
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "7", platform));    // flash pin
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "-1", platform));   // the receiver port cannot be off
  TEST_ASSERT_NULL(eb::paramSet(ref, "34", platform));       // input-only pin is fine for RX
  TEST_ASSERT_EQUAL_INT8(34, cfg.crsfRxPin);

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "crsf_tx_pin", ref));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "34", platform));   // ...but cannot transmit
  TEST_ASSERT_NULL(eb::paramSet(ref, "25", platform));

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "aux1.pin", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "-1", platform));
  TEST_ASSERT_NULL(eb::paramSet(ref, "26", platform));
  TEST_ASSERT_EQUAL_INT8(26, cfg.aux[0].pin);

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "vbat_pin", ref));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "16", platform));   // not an ADC pin
  TEST_ASSERT_NULL(eb::paramSet(ref, "34", platform));
  TEST_ASSERT_NULL(eb::paramSet(ref, "-1", platform));

  // I2C drives its lines, so an input-only pin will not do.
  TEST_ASSERT_TRUE(eb::paramFind(cfg, "motor_sda", ref));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "35", platform));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "-1", platform));
  TEST_ASSERT_NULL(eb::paramSet(ref, "25", platform));
  TEST_ASSERT_EQUAL_INT8(25, cfg.motorSdaPin);
}

void test_drive_settings() {
  // The defaults are the robot's original PS4 firmware and wiring.
  FakePlatform platform;
  eb::Config cfg;
  eb::configDefaults(cfg);
  TEST_ASSERT_EQUAL_UINT8(eb::kDriveAuto, cfg.driveSrc);
  TEST_ASSERT_EQUAL_UINT8(12, cfg.driveSpeed);  // a scroll wheel mixed to CH12
  TEST_ASSERT_EQUAL_UINT8(eb::kSpeedThrottle, cfg.driveSpeedType);
  TEST_ASSERT_EQUAL_UINT8(80, cfg.driveSpeedTop);  // the wheel at its top: 80 %
  TEST_ASSERT_EQUAL_UINT8(60, cfg.driveMax);
  TEST_ASSERT_EQUAL_UINT8(30, cfg.driveSlow);
  TEST_ASSERT_EQUAL_UINT8(50, cfg.driveNormal);
  TEST_ASSERT_EQUAL_UINT8(100, cfg.driveFast);
  TEST_ASSERT_EQUAL_UINT8(2, cfg.driveAccel);
  TEST_ASSERT_EQUAL_UINT8(1, cfg.driveDecel);
  TEST_ASSERT_EQUAL_INT8(21, cfg.motorSdaPin);
  TEST_ASSERT_EQUAL_INT8(22, cfg.motorSclPin);
  TEST_ASSERT_EQUAL_UINT8(0x34, cfg.motorAddr);
  TEST_ASSERT_EQUAL_UINT8(eb::kMotorJgb37, cfg.motorType);
  TEST_ASSERT_EQUAL_UINT8(eb::kWheelBackRight, cfg.motor[0].wheel);
  TEST_ASSERT_EQUAL_UINT8(0, cfg.motor[0].rev);
  TEST_ASSERT_EQUAL_UINT8(eb::kWheelFrontRight, cfg.motor[1].wheel);
  TEST_ASSERT_EQUAL_UINT8(1, cfg.motor[1].rev);
  TEST_ASSERT_EQUAL_UINT8(eb::kWheelBackLeft, cfg.motor[2].wheel);
  TEST_ASSERT_EQUAL_UINT8(1, cfg.motor[2].rev);
  TEST_ASSERT_EQUAL_UINT8(eb::kWheelFrontLeft, cfg.motor[3].wheel);
  TEST_ASSERT_EQUAL_UINT8(0, cfg.motor[3].rev);

  eb::ParamRef ref;
  char buf[32];
  TEST_ASSERT_TRUE(eb::paramFind(cfg, "drive_src", ref));
  eb::paramFormat(ref, buf, sizeof buf);
  TEST_ASSERT_EQUAL_STRING("auto", buf);
  TEST_ASSERT_NULL(eb::paramSet(ref, "bt", platform));
  TEST_ASSERT_EQUAL_UINT8(eb::kDriveBt, cfg.driveSrc);
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "ps4", platform));

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "drive_speed_type", ref));
  eb::paramFormat(ref, buf, sizeof buf);
  TEST_ASSERT_EQUAL_STRING("throttle", buf);
  TEST_ASSERT_NULL(eb::paramSet(ref, "switch", platform));
  TEST_ASSERT_EQUAL_UINT8(eb::kSpeedSwitch, cfg.driveSpeedType);
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "slider", platform));
  TEST_ASSERT_TRUE(eb::paramFind(cfg, "drive_speed_top", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "70", platform));
  TEST_ASSERT_EQUAL_UINT8(70, cfg.driveSpeedTop);
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "101", platform));  // a percentage

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "motor2.wheel", ref));
  eb::paramFormat(ref, buf, sizeof buf);
  TEST_ASSERT_EQUAL_STRING("fr", buf);
  TEST_ASSERT_NULL(eb::paramSet(ref, "BL", platform));
  TEST_ASSERT_EQUAL_UINT8(eb::kWheelBackLeft, cfg.motor[1].wheel);
  TEST_ASSERT_TRUE(eb::paramFind(cfg, "motor4.rev", ref));
  TEST_ASSERT_FALSE(eb::paramFind(cfg, "motor5.rev", ref));
  TEST_ASSERT_FALSE(eb::paramFind(cfg, "motor0.wheel", ref));

  // "motor_type" and the rest are plain settings, not motor channels.
  TEST_ASSERT_TRUE(eb::paramFind(cfg, "motor_type", ref));
  eb::paramFormat(ref, buf, sizeof buf);
  TEST_ASSERT_EQUAL_STRING("jgb37", buf);
  TEST_ASSERT_TRUE(eb::paramFind(cfg, "motor_addr", ref));
  TEST_ASSERT_NULL(eb::paramSet(ref, "0x35", platform));
  TEST_ASSERT_EQUAL_UINT8(0x35, cfg.motorAddr);
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "128", platform));

  TEST_ASSERT_TRUE(eb::paramFind(cfg, "drive_accel", ref));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "0", platform));  // it would never move
  TEST_ASSERT_TRUE(eb::paramFind(cfg, "drive_max", ref));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "101", platform));
}

// ---------------------------------------------------------------------------
// The saved text form

void test_settings_text_round_trip() {
  FakePlatform platform;
  eb::Config defaults;
  eb::configDefaults(defaults);
  eb::Config cfg = defaults;
  char text[eb::kConfigTextMax];
  size_t len = 0;

  // Unchanged settings save as just the header comment.
  TEST_ASSERT_TRUE(eb::configToText(cfg, defaults, text, sizeof text, len));
  TEST_ASSERT_EQUAL_CHAR('#', text[0]);
  TEST_ASSERT_EQUAL_UINT(1, std::count(text, text + len, '\n'));

  cfg.streamHz = 100;
  cfg.hostStream = elrsb::kStreamText;
  cfg.vbatScale = 10.98f;
  cfg.vbatPin = 34;
  cfg.out[0].rev = 1;
  cfg.out[0].src = 2;
  strcpy(cfg.out[1].name, "throttle");
  cfg.out[15].fs = -1000;
  cfg.aux[3].pin = 27;
  cfg.aux[3].mode = eb::kAuxLink;
  TEST_ASSERT_TRUE(eb::configToText(cfg, defaults, text, sizeof text, len));
  const std::string saved(text, len);
  TEST_ASSERT_TRUE(contains(saved, "stream_hz 100\n"));
  TEST_ASSERT_TRUE(contains(saved, "host_stream text\n"));
  TEST_ASSERT_TRUE(contains(saved, "out2.name throttle\n"));
  TEST_ASSERT_TRUE(contains(saved, "aux4.mode link\n"));
  TEST_ASSERT_FALSE(contains(saved, "failsafe_ms"));

  eb::Config loaded = defaults;
  TEST_ASSERT_EQUAL_INT(0, eb::configFromText(loaded, text, len, platform));
  TEST_ASSERT_EQUAL_MEMORY(&cfg, &loaded, sizeof cfg);
}

void test_settings_text_with_everything_changed_fits() {
  // Worst case for the save buffer: every parameter differs from its default.
  FakePlatform platform;
  eb::Config defaults;
  eb::configDefaults(defaults);
  eb::Config cfg = defaults;
  cfg.crsfRxPin = 32;
  cfg.crsfTxPin = 33;
  cfg.crsfInvert = 1;
  cfg.crsfBaud = 921600;
  cfg.hostRxPin = 21;
  cfg.hostTxPin = 22;
  cfg.hostBaud = 1000000;
  cfg.usbBaud = 921600;
  cfg.hostStream = elrsb::kStreamText;
  cfg.usbStream = elrsb::kStreamBinary;
  cfg.streamHz = 250;
  cfg.streamCh = 8;
  cfg.linkHz = 10;
  cfg.fwdCrsf = 1;
  cfg.failsafeMs = 1000;
  cfg.armCh = 14;
  cfg.armUs = 1700;
  cfg.armGuard = 0;
  cfg.tlmPeriodMs = 250;
  cfg.tlmTimeoutMs = 10000;
  cfg.tlmAutoMode = 0;
  cfg.vbatPin = 35;
  cfg.vbatScale = 123.456f;
  cfg.ledPin = 4;
  cfg.ledMode = eb::kLedActiveLow;
  cfg.pwmHz = 333;
  for (int i = 0; i < eb::kNumOutputs; ++i) {
    eb::OutputCfg& o = cfg.out[i];
    snprintf(o.name, sizeof o.name, "output_%02d_x", i);
    o.src = (uint8_t)(16 - i);
    o.type = o.type ? 0 : 1;
    o.rev = 1;
    o.hold = 1;
    o.fs = -1000;
    o.minUs = 1111;
    o.midUs = 1555;
    o.maxUs = 1999;
    o.db = 123;
  }
  static const int8_t kPins[4] = {25, 26, 27, 14};
  for (int i = 0; i < eb::kNumAux; ++i) {
    eb::AuxCfg& a = cfg.aux[i];
    a.pin = kPins[i];
    a.mode = eb::kAuxSwitch;
    a.src = (uint8_t)(16 - i);
    a.gate = 0;
    a.inv = 1;
    a.minUs = 1100;
    a.maxUs = 1900;
    a.fsUs = 1500;
  }

  char text[eb::kConfigTextMax];
  size_t len = 0;
  TEST_ASSERT_TRUE(eb::configToText(cfg, defaults, text, sizeof text, len));
  TEST_ASSERT_TRUE(len < eb::kConfigTextMax * 3 / 4);  // keep some headroom

  eb::Config loaded = defaults;
  TEST_ASSERT_EQUAL_INT(0, eb::configFromText(loaded, text, len, platform));
  TEST_ASSERT_EQUAL_MEMORY(&cfg, &loaded, sizeof cfg);
}

void test_settings_text_skips_what_it_cannot_apply() {
  FakePlatform platform;
  eb::Config cfg;
  eb::configDefaults(cfg);
  static const char kText[] =
      "# comment\n"
      "\n"
      "stream_hz 200\r\n"
      "removed_in_this_version 5\n"
      "failsafe_ms banana\n"
      "crsf_rx_pin 3\n"
      "out2.rev=on\n"
      "nonsense\n"
      "link_hz 2";
  TEST_ASSERT_EQUAL_INT(4, eb::configFromText(cfg, kText, sizeof kText - 1, platform));
  TEST_ASSERT_EQUAL_UINT16(200, cfg.streamHz);
  TEST_ASSERT_EQUAL_UINT16(500, cfg.failsafeMs);
  TEST_ASSERT_EQUAL_INT8(16, cfg.crsfRxPin);
  TEST_ASSERT_EQUAL_UINT8(1, cfg.out[1].rev);
  TEST_ASSERT_EQUAL_UINT8(2, cfg.linkHz);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_param_set_and_format);
  RUN_TEST(test_param_labels);
  RUN_TEST(test_param_names_and_lookup);
  RUN_TEST(test_param_pin_validation);
  RUN_TEST(test_drive_settings);

  RUN_TEST(test_settings_text_round_trip);
  RUN_TEST(test_settings_text_with_everything_changed_fits);
  RUN_TEST(test_settings_text_skips_what_it_cannot_apply);
  return UNITY_END();
}
