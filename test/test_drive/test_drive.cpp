// Tests for the driving: who is in charge, what the sticks do to the four
// wheels, and how the wheels speed up, slow down and stop.
//
//   pio test -e native -f test_drive
//
// The expected wheel speeds are the ones the robot's original PS4 firmware
// produced for the same stick positions, worked out by hand.
#include <unity.h>

#include <stdio.h>
#include <string.h>

#include "eb_config.h"
#include "eb_drive.h"
#include "eb_gamepad.h"
#include "eb_rc.h"
#include "sticks.h"

namespace {

// ---------------------------------------------------------------------------
// The bench: a handset feeding the Rc, a Bluetooth controller, and the Drive,
// with a millisecond clock.

const uint16_t kStickUp = 1811;    // full stick, as CRSF ticks
const uint16_t kStickDown = 172;
const uint16_t kSwitchLow = 191;
const uint16_t kSwitchMid = 992;
const uint16_t kSwitchHigh = 1792;

struct Bench {
  eb::Config cfg;
  eb::Rc rc;
  eb::Drive drive;
  Sticks sticks;
  uint32_t now;
  bool linked;              // the handset sends RC frames every 10 ms
  bool padOn;               // the Bluetooth controller reports every 5 ms
  eb::GamepadInput pad;
  int sends;                // how often the drive asked for commands to be sent

  Bench() : now(1000), linked(false), padOn(false), sends(0) {
    eb::configDefaults(cfg);
    memset(&pad, 0, sizeof pad);
    pad.connected = true;
    // The speed wheel (CH12) half way, and reaching full speed at its top, so
    // the sticks give the original firmware's normal speed: 50 % of 60.
    cfg.driveSpeedTop = 100;
    sticks.ticks[11] = kSwitchMid;
    rc.update(cfg, now);
  }

  // One millisecond.
  bool step() {
    ++now;
    if (linked && now % 10 == 0) rc.onChannels(sticks.ticks, now);
    if (padOn && now % 5 == 0) drive.setGamepad(pad, now);
    rc.update(cfg, now);
    const bool send = drive.update(cfg, rc, now);
    if (send) ++sends;
    return send;
  }

  void run(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) step();
  }

  // Bring the link up and arm with the switch, sticks where they are.
  void arm() {
    linked = true;
    sticks.ticks[4] = kSwitchLow;
    run(kArmOffHold);
    sticks.ticks[4] = kSwitchHigh;
    run(30);
    TEST_ASSERT_TRUE_MESSAGE(rc.armed(), "did not arm");
  }

  // The controller's sticks and triggers, as Bluepad32 reports them.
  void padSticks(int lx, int ly, int rx, bool l2 = false, bool r2 = false) {
    pad.lx = (int16_t)lx;
    pad.ly = (int16_t)ly;
    pad.rx = (int16_t)rx;
    pad.l2 = l2;
    pad.r2 = r2;
  }

  // Connect the controller with its sticks centred, so that it takes charge.
  void connectPad() {
    padSticks(0, 0, 0);
    padOn = true;
    run(20);
  }
};

void expectMotors(const eb::Drive& d, int m1, int m2, int m3, int m4) {
  const int expected[4] = {m1, m2, m3, m4};
  for (int i = 0; i < 4; ++i) {
    char what[40];
    snprintf(what, sizeof what, "motor M%d command", i + 1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(expected[i], d.command(i), what);
  }
}

void expectTargets(const eb::Drive& d, int m1, int m2, int m3, int m4) {
  const int expected[4] = {m1, m2, m3, m4};
  for (int i = 0; i < 4; ++i) {
    char what[40];
    snprintf(what, sizeof what, "motor M%d target", i + 1);
    TEST_ASSERT_EQUAL_INT_MESSAGE(expected[i], d.target(i), what);
  }
}

// ---------------------------------------------------------------------------
// The mix: the same wheel speeds as the original firmware

void test_full_forward_on_the_handset() {
  // Right stick fully up, normal speed: every wheel at 50 % of 60. M2 and M3
  // are mounted the other way round, so they get the negative command.
  Bench b;
  b.sticks.ticks[1] = kStickUp;
  b.arm();
  b.run(500);
  TEST_ASSERT_EQUAL(eb::Drive::kElrs, b.drive.source());
  TEST_ASSERT_TRUE(b.drive.live());
  expectMotors(b.drive, 30, -30, -30, 30);
}

void test_sideways_and_turning() {
  Bench b;
  b.arm();

  b.sticks.ticks[0] = kStickUp;  // right stick right: strafe right
  b.run(500);
  expectMotors(b.drive, 30, 30, 30, 30);

  b.sticks.ticks[0] = kSwitchMid;
  b.sticks.ticks[3] = kStickUp;  // left stick right: turn right on the spot
  b.run(1000);
  expectMotors(b.drive, -30, 30, -30, 30);
}

void test_combined_moves_keep_their_direction() {
  // Forward and sideways together ask two wheels for twice full speed. All
  // four are scaled down together, so the robot still goes diagonally.
  Bench b;
  b.sticks.ticks[1] = kStickUp;
  b.sticks.ticks[0] = kStickUp;
  b.arm();
  b.run(500);
  expectMotors(b.drive, 30, 0, 0, 30);

  // Half forward and half a right turn: the left wheels at full, the right
  // ones still, so the robot pivots round its right side.
  b.sticks.ticks[0] = kSwitchMid;
  b.sticks.ticks[1] = eb::crsf::usToTicks(1766);  // 500 once past the deadband
  b.sticks.ticks[3] = eb::crsf::usToTicks(1766);
  b.run(1000);
  expectTargets(b.drive, 0, 0, -30, 30);
}

void test_speed_wheel_sets_the_speed() {
  // As on a drone: the speed wheel sets how fast the robot may go, and how
  // far the sticks are pushed sets how much of that it uses. All the way up
  // is drive_speed_top: 80 % of drive_max.
  Bench b;
  eb::configDefaults(b.cfg);        // the real top, not the bench's 100 %
  b.sticks.ticks[1] = kStickUp;     // right stick fully up
  b.sticks.ticks[11] = kSwitchLow;  // speed wheel (CH12) all the way down
  b.arm();
  b.run(1000);
  TEST_ASSERT_TRUE(b.drive.live());
  TEST_ASSERT_EQUAL_INT(0, b.drive.speedPercent());
  TEST_ASSERT_FALSE(b.drive.moving());  // armed and pushed, but no speed

  b.sticks.ticks[11] = kSwitchHigh;  // all the way up: 80 % of 60
  b.run(1000);
  TEST_ASSERT_EQUAL_INT(80, b.drive.speedPercent());
  expectMotors(b.drive, 48, -48, -48, 48);

  b.sticks.ticks[11] = eb::crsf::usToTicks(1250);  // a quarter of the way up
  b.run(1000);
  TEST_ASSERT_EQUAL_INT(20, b.drive.speedPercent());
  expectMotors(b.drive, 12, -12, -12, 12);

  // The wheel half way with the stick fully up goes as fast as the wheel all
  // the way up with the stick half way.
  b.sticks.ticks[11] = kSwitchMid;
  b.run(1000);
  expectMotors(b.drive, 24, -24, -24, 24);
  b.sticks.ticks[11] = kSwitchHigh;
  b.sticks.ticks[1] = eb::crsf::usToTicks(1766);  // 500 once past the deadband
  b.run(1000);
  expectMotors(b.drive, 24, -24, -24, 24);

  // With drive_speed_top 100 the top of the wheel is full speed.
  b.cfg.driveSpeedTop = 100;
  b.sticks.ticks[1] = kStickUp;
  b.run(1000);
  TEST_ASSERT_EQUAL_INT(100, b.drive.speedPercent());
  expectMotors(b.drive, 60, -60, -60, 60);

  // The throttle stick (CH3) works as well, from its bottom to its top.
  b.cfg.driveSpeed = 3;
  b.sticks.ticks[2] = kStickDown;
  b.run(1000);
  TEST_ASSERT_FALSE(b.drive.moving());
  b.sticks.ticks[2] = eb::crsf::usToTicks(1234);  // a quarter of the way up
  b.run(1000);
  expectMotors(b.drive, 15, -15, -15, 15);
  b.sticks.ticks[2] = kStickUp;
  b.run(1000);
  expectMotors(b.drive, 60, -60, -60, 60);
}

void test_speed_switch() {
  // A three-position switch on CH6 instead of the throttle.
  Bench b;
  b.cfg.driveSpeed = 6;
  b.cfg.driveSpeedType = eb::kSpeedSwitch;
  b.sticks.ticks[1] = kStickUp;
  b.arm();

  b.sticks.ticks[5] = kSwitchLow;
  b.run(1000);
  expectMotors(b.drive, 18, -18, -18, 18);  // slow: 30 % of 60

  b.sticks.ticks[5] = kSwitchHigh;
  b.run(1000);
  expectMotors(b.drive, 60, -60, -60, 60);  // fast: 100 %

  b.sticks.ticks[5] = kSwitchMid;
  b.run(1000);
  expectMotors(b.drive, 30, -30, -30, 30);  // normal: 50 %

  // With no speed switch assigned, it is always normal speed.
  b.cfg.driveSpeed = 0;
  b.sticks.ticks[5] = kSwitchHigh;
  b.run(1000);
  expectMotors(b.drive, 30, -30, -30, 30);
}

void test_motor_wiring_is_configurable() {
  Bench b;
  b.sticks.ticks[1] = kStickUp;
  // Say M1 is really the front left wheel, and mounted the other way round.
  b.cfg.motor[0].wheel = eb::kWheelFrontLeft;
  b.cfg.motor[0].rev = 1;
  b.cfg.driveMax = 100;
  b.sticks.ticks[11] = kSwitchHigh;  // speed wheel all the way up: full speed
  b.arm();
  b.run(1000);
  expectMotors(b.drive, -100, -100, -100, 100);
}

// ---------------------------------------------------------------------------
// Speeding up and slowing down

void test_speeds_up_and_slows_down_gently() {
  // 2 steps every 10 ms up, 1 step every 10 ms down, as in the original.
  Bench b;
  b.arm();
  b.run(10);
  expectMotors(b.drive, 0, 0, 0, 0);

  b.sticks.ticks[1] = kStickUp;
  int steps = 0;
  int previous = 0;
  for (int ms = 0; ms < 1000 && b.drive.command(0) < 30; ++ms) {
    if (b.step() && b.drive.command(0) != previous) {
      TEST_ASSERT_EQUAL_INT(previous + 2, b.drive.command(0));
      previous = b.drive.command(0);
      ++steps;
    }
  }
  TEST_ASSERT_EQUAL_INT(15, steps);  // 150 ms from standstill to 30
  TEST_ASSERT_EQUAL_INT(-30, b.drive.command(1));

  b.sticks.ticks[1] = kSwitchMid;  // let go
  steps = 0;
  for (int ms = 0; ms < 1000 && b.drive.command(0) > 0; ++ms) {
    if (b.step() && b.drive.command(0) != previous) {
      TEST_ASSERT_EQUAL_INT(previous - 1, b.drive.command(0));
      previous = b.drive.command(0);
      ++steps;
    }
  }
  TEST_ASSERT_EQUAL_INT(30, steps);  // 300 ms from 30 back to standstill
  expectMotors(b.drive, 0, 0, 0, 0);
}

void test_reversing_slows_to_zero_first() {
  Bench b;
  b.sticks.ticks[1] = kStickUp;
  b.arm();
  b.run(500);
  TEST_ASSERT_EQUAL_INT(30, b.drive.command(0));

  b.sticks.ticks[1] = kStickDown;  // full reverse
  // Down to zero at the slowing-down rate: 30 steps of 10 ms...
  b.run(295);
  TEST_ASSERT_TRUE(b.drive.command(0) > 0);
  b.run(15);
  TEST_ASSERT_TRUE(b.drive.command(0) <= 0);
  // ...then up to full reverse at the speeding-up rate: 15 more.
  b.run(160);
  TEST_ASSERT_EQUAL_INT(-30, b.drive.command(0));
  TEST_ASSERT_EQUAL_INT(30, b.drive.command(1));
}

void test_commands_go_out_every_10_ms() {
  Bench b;
  b.arm();
  b.sends = 0;
  b.run(1000);
  TEST_ASSERT_INT_WITHIN(1, 100, b.sends);
}

// ---------------------------------------------------------------------------
// Stopping

void test_arm_switch_off_stops_at_once() {
  Bench b;
  b.sticks.ticks[1] = kStickUp;
  b.arm();
  b.run(500);
  expectMotors(b.drive, 30, -30, -30, 30);

  // The switch goes off in the frame that arrives at the next 10 ms mark.
  b.sticks.ticks[4] = kSwitchLow;
  while (b.rc.armed()) b.step();
  // That same millisecond the wheels are stopped: no ramp.
  expectMotors(b.drive, 0, 0, 0, 0);
  expectTargets(b.drive, 0, 0, 0, 0);
  TEST_ASSERT_FALSE(b.drive.live());
}

void test_a_stop_goes_out_immediately() {
  Bench b;
  b.sticks.ticks[1] = kStickUp;
  b.arm();
  b.run(500);
  // Step to just after a periodic send, then disarm: the stop must not wait.
  while (!b.step()) {
  }
  b.sticks.ticks[4] = kSwitchLow;
  b.rc.onChannels(b.sticks.ticks, b.now + 1);
  b.rc.update(b.cfg, b.now + 1);
  TEST_ASSERT_FALSE(b.rc.armed());
  TEST_ASSERT_TRUE(b.drive.update(b.cfg, b.rc, b.now + 1));
  expectMotors(b.drive, 0, 0, 0, 0);
}

void test_link_loss_slows_down_and_stops() {
  Bench b;
  b.sticks.ticks[1] = kStickUp;
  b.arm();
  b.run(500);

  // The handset goes out of range. For failsafe_ms the last frame still
  // counts; after that the wheels ramp down, as when the sticks are let go.
  b.linked = false;
  b.run(490);
  TEST_ASSERT_EQUAL_INT(30, b.drive.command(0));
  b.run(20);
  TEST_ASSERT_FALSE(b.rc.linkUp());
  TEST_ASSERT_EQUAL(eb::Drive::kNobody, b.drive.source());
  TEST_ASSERT_TRUE(b.drive.command(0) > 25);
  b.run(300);
  expectMotors(b.drive, 0, 0, 0, 0);
}

void test_link_loss_can_stop_at_once() {
  Bench b;
  b.cfg.driveFs = eb::kDriveFsStop;
  b.sticks.ticks[1] = kStickUp;
  b.arm();
  b.run(500);
  b.linked = false;
  b.run(490);
  TEST_ASSERT_EQUAL_INT(30, b.drive.command(0));
  b.run(20);
  expectMotors(b.drive, 0, 0, 0, 0);
}

void test_drive_off_never_moves() {
  Bench b;
  b.cfg.driveSrc = eb::kDriveOff;
  b.sticks.ticks[1] = kStickUp;
  b.arm();
  b.connectPad();
  b.padSticks(0, -512, 0);
  b.run(1000);
  TEST_ASSERT_EQUAL(eb::Drive::kNobody, b.drive.source());
  expectMotors(b.drive, 0, 0, 0, 0);
  TEST_ASSERT_FALSE(b.drive.moving());
}

// ---------------------------------------------------------------------------
// The Bluetooth controller

void test_gamepad_drives_when_the_handset_is_off() {
  // Left stick up is forward, as in the original PS4 firmware.
  Bench b;
  b.connectPad();
  TEST_ASSERT_EQUAL(eb::Drive::kGamepad, b.drive.source());
  TEST_ASSERT_TRUE(b.drive.gamepadConnected());
  TEST_ASSERT_TRUE(b.drive.live());

  b.padSticks(0, -512, 0);
  b.run(500);
  expectMotors(b.drive, 30, -30, -30, 30);

  b.padSticks(511, 0, 0);  // left stick right: sideways
  b.run(1000);
  expectMotors(b.drive, 29, 29, 29, 29);  // 511 of 512 is not quite full

  b.padSticks(0, 0, -512);  // right stick left: turn left
  b.run(1000);
  expectMotors(b.drive, 30, -30, 30, -30);
}

void test_gamepad_triggers_pick_the_speed() {
  Bench b;
  b.connectPad();
  b.padSticks(0, -512, 0, true, false);  // L2 held: slow
  b.run(1000);
  expectMotors(b.drive, 18, -18, -18, 18);
  b.padSticks(0, -512, 0, false, true);  // R2 held: fast
  b.run(1000);
  expectMotors(b.drive, 60, -60, -60, 60);
  b.padSticks(0, -512, 0, true, true);   // both: slow wins
  b.run(1000);
  expectMotors(b.drive, 18, -18, -18, 18);
}

void test_gamepad_deadband() {
  // Under 10 % of stick travel is nothing, as in the original firmware.
  TEST_ASSERT_EQUAL_INT(0, eb::Drive::padAxis(0));
  TEST_ASSERT_EQUAL_INT(0, eb::Drive::padAxis(51));
  TEST_ASSERT_EQUAL_INT(0, eb::Drive::padAxis(-51));
  TEST_ASSERT_EQUAL_INT(101, eb::Drive::padAxis(52));
  TEST_ASSERT_EQUAL_INT(-101, eb::Drive::padAxis(-52));
  TEST_ASSERT_EQUAL_INT(-1000, eb::Drive::padAxis(-512));
  TEST_ASSERT_EQUAL_INT(998, eb::Drive::padAxis(511));
  TEST_ASSERT_EQUAL_INT(1000, eb::Drive::padAxis(600));  // clamped

  Bench b;
  b.connectPad();
  b.padSticks(40, -45, 30);  // resting a thumb on the sticks
  b.run(500);
  TEST_ASSERT_FALSE(b.drive.moving());
}

void test_gamepad_must_be_centred_before_it_drives() {
  // A controller that connects - or takes over from the handset - with a
  // stick pushed does nothing until the sticks have been let go once.
  Bench b;
  b.padSticks(0, -512, 0);
  b.padOn = true;
  b.run(500);
  TEST_ASSERT_EQUAL(eb::Drive::kGamepad, b.drive.source());
  TEST_ASSERT_TRUE(b.drive.gamepadWaiting());
  TEST_ASSERT_FALSE(b.drive.live());
  expectMotors(b.drive, 0, 0, 0, 0);

  b.padSticks(0, 0, 0);
  b.run(20);
  TEST_ASSERT_FALSE(b.drive.gamepadWaiting());
  b.padSticks(0, -512, 0);
  b.run(500);
  expectMotors(b.drive, 30, -30, -30, 30);
}

void test_the_handset_takes_charge_from_the_gamepad() {
  // Both are on: the handset is in charge while its link is up, and its arm
  // switch decides. The gamepad is ignored meanwhile.
  Bench b;
  b.connectPad();
  b.padSticks(0, -512, 0);
  b.run(500);
  expectMotors(b.drive, 30, -30, -30, 30);

  b.linked = true;  // the handset is switched on, arm switch off
  b.run(20);
  TEST_ASSERT_EQUAL(eb::Drive::kElrs, b.drive.source());
  expectMotors(b.drive, 0, 0, 0, 0);  // stopped at once
  b.run(1000);
  expectMotors(b.drive, 0, 0, 0, 0);

  // Armed, the handset's sticks drive; the gamepad still has no say.
  b.sticks.ticks[1] = kStickDown;
  b.sticks.ticks[4] = kSwitchHigh;
  b.run(1000);
  TEST_ASSERT_TRUE(b.rc.armed());
  expectMotors(b.drive, -30, 30, 30, -30);

  // Handset out of range: the gamepad takes over again, once centred.
  b.linked = false;
  b.run(1500);
  TEST_ASSERT_EQUAL(eb::Drive::kGamepad, b.drive.source());
  TEST_ASSERT_TRUE(b.drive.gamepadWaiting());
  expectMotors(b.drive, 0, 0, 0, 0);
  b.padSticks(0, 0, 0);
  b.run(20);
  b.padSticks(0, -512, 0);
  b.run(1000);
  expectMotors(b.drive, 30, -30, -30, 30);
}

void test_a_silent_gamepad_counts_as_gone() {
  Bench b;
  b.connectPad();
  b.padSticks(0, -512, 0);
  b.run(500);
  expectMotors(b.drive, 30, -30, -30, 30);

  // The reports stop, as when the controller goes out of range before
  // Bluetooth itself notices: after bt_timeout_ms the wheels ramp down.
  b.padOn = false;
  b.run(490);
  TEST_ASSERT_EQUAL_INT(30, b.drive.command(0));
  b.run(20);
  TEST_ASSERT_FALSE(b.drive.gamepadConnected());
  TEST_ASSERT_EQUAL(eb::Drive::kNobody, b.drive.source());
  b.run(400);
  expectMotors(b.drive, 0, 0, 0, 0);
}

void test_a_disconnected_gamepad_is_gone_at_once() {
  Bench b;
  b.connectPad();
  b.padSticks(0, -512, 0);
  b.run(500);
  b.pad.connected = false;
  b.run(6);
  TEST_ASSERT_EQUAL(eb::Drive::kNobody, b.drive.source());
  TEST_ASSERT_TRUE(b.drive.target(0) == 0);
}

void test_source_settings() {
  // drive_src elrs: the gamepad is never used.
  Bench elrsOnly;
  elrsOnly.cfg.driveSrc = eb::kDriveElrs;
  elrsOnly.connectPad();
  elrsOnly.padSticks(0, -512, 0);
  elrsOnly.run(500);
  TEST_ASSERT_EQUAL(eb::Drive::kNobody, elrsOnly.drive.source());
  expectMotors(elrsOnly.drive, 0, 0, 0, 0);

  // drive_src bt: the handset is never used, even armed.
  Bench btOnly;
  btOnly.cfg.driveSrc = eb::kDriveBt;
  btOnly.sticks.ticks[1] = kStickUp;
  btOnly.arm();
  btOnly.run(500);
  TEST_ASSERT_EQUAL(eb::Drive::kNobody, btOnly.drive.source());
  expectMotors(btOnly.drive, 0, 0, 0, 0);
  btOnly.connectPad();
  TEST_ASSERT_EQUAL(eb::Drive::kGamepad, btOnly.drive.source());
  btOnly.padSticks(0, 511, 0);  // left stick down: backwards
  btOnly.run(1000);
  expectMotors(btOnly.drive, -29, 29, 29, -29);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_full_forward_on_the_handset);
  RUN_TEST(test_sideways_and_turning);
  RUN_TEST(test_combined_moves_keep_their_direction);
  RUN_TEST(test_speed_wheel_sets_the_speed);
  RUN_TEST(test_speed_switch);
  RUN_TEST(test_motor_wiring_is_configurable);

  RUN_TEST(test_speeds_up_and_slows_down_gently);
  RUN_TEST(test_reversing_slows_to_zero_first);
  RUN_TEST(test_commands_go_out_every_10_ms);

  RUN_TEST(test_arm_switch_off_stops_at_once);
  RUN_TEST(test_a_stop_goes_out_immediately);
  RUN_TEST(test_link_loss_slows_down_and_stops);
  RUN_TEST(test_link_loss_can_stop_at_once);
  RUN_TEST(test_drive_off_never_moves);

  RUN_TEST(test_gamepad_drives_when_the_handset_is_off);
  RUN_TEST(test_gamepad_triggers_pick_the_speed);
  RUN_TEST(test_gamepad_deadband);
  RUN_TEST(test_gamepad_must_be_centred_before_it_drives);
  RUN_TEST(test_the_handset_takes_charge_from_the_gamepad);
  RUN_TEST(test_a_silent_gamepad_counts_as_gone);
  RUN_TEST(test_a_disconnected_gamepad_is_gone_at_once);
  RUN_TEST(test_source_settings);
  return UNITY_END();
}
