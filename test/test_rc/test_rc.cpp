// Tests for the control state: how a pulse width becomes an output value, and
// when the link counts as up, failed or armed.
//
//   pio test -e native -f test_rc
#include <unity.h>

#include "eb_config.h"
#include "eb_crsf.h"
#include "eb_rc.h"
#include "fake_platform.h"
#include "sticks.h"

namespace {

// ---------------------------------------------------------------------------
// Output mapping

eb::OutputCfg stickCfg() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  return cfg.out[0];
}

eb::OutputCfg switchCfg() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  return cfg.out[4];
}

void test_bipolar_mapping() {
  const eb::OutputCfg c = stickCfg();  // 988 / 1500 / 2012, deadband 20
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 1500));
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 1510));   // inside the deadband
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 1490));
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 1520));   // edge of the deadband
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 1480));
  TEST_ASSERT_EQUAL_INT16(2, eb::Rc::mapOutput(c, 1521));   // 1 / 492 -> 2.03
  TEST_ASSERT_EQUAL_INT16(-2, eb::Rc::mapOutput(c, 1479));
  TEST_ASSERT_EQUAL_INT16(480, eb::Rc::mapOutput(c, 1756));  // 236 / 492 -> 479.7
  TEST_ASSERT_EQUAL_INT16(1000, eb::Rc::mapOutput(c, 2012));
  TEST_ASSERT_EQUAL_INT16(-1000, eb::Rc::mapOutput(c, 988));
  TEST_ASSERT_EQUAL_INT16(1000, eb::Rc::mapOutput(c, 2200));  // clamped
  TEST_ASSERT_EQUAL_INT16(-1000, eb::Rc::mapOutput(c, 800));
}

void test_bipolar_reverse_and_asymmetric_ends() {
  eb::OutputCfg c = stickCfg();
  c.rev = 1;
  TEST_ASSERT_EQUAL_INT16(-1000, eb::Rc::mapOutput(c, 2012));
  TEST_ASSERT_EQUAL_INT16(1000, eb::Rc::mapOutput(c, 988));
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 1500));

  // Each side scales to full range from its own end point.
  c.rev = 0;
  c.db = 0;
  c.minUs = 1100;
  c.midUs = 1500;
  c.maxUs = 1700;
  TEST_ASSERT_EQUAL_INT16(500, eb::Rc::mapOutput(c, 1600));
  TEST_ASSERT_EQUAL_INT16(-500, eb::Rc::mapOutput(c, 1300));
  TEST_ASSERT_EQUAL_INT16(1000, eb::Rc::mapOutput(c, 1700));
  TEST_ASSERT_EQUAL_INT16(-1000, eb::Rc::mapOutput(c, 1100));
}

void test_unipolar_mapping() {
  eb::OutputCfg c = switchCfg();  // 1000 .. 2000, no deadband
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 1000));
  TEST_ASSERT_EQUAL_INT16(500, eb::Rc::mapOutput(c, 1500));
  TEST_ASSERT_EQUAL_INT16(1000, eb::Rc::mapOutput(c, 2000));
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 999));     // ELRS switch low
  TEST_ASSERT_EQUAL_INT16(1000, eb::Rc::mapOutput(c, 2012));
  c.rev = 1;
  TEST_ASSERT_EQUAL_INT16(1000, eb::Rc::mapOutput(c, 1000));
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 2000));
  c.rev = 0;
  c.db = 100;  // dead zone at the low end
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 1100));
  TEST_ASSERT_EQUAL_INT16(500, eb::Rc::mapOutput(c, 1550));
  TEST_ASSERT_EQUAL_INT16(1000, eb::Rc::mapOutput(c, 2000));
}

void test_mapping_survives_degenerate_calibration() {
  eb::OutputCfg c = stickCfg();
  c.minUs = c.midUs = c.maxUs = 1500;
  c.db = 0;
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 1500));
  TEST_ASSERT_EQUAL_INT16(1000, eb::Rc::mapOutput(c, 1501));
  TEST_ASSERT_EQUAL_INT16(-1000, eb::Rc::mapOutput(c, 1499));
  c = switchCfg();
  c.minUs = c.maxUs = 1500;
  TEST_ASSERT_EQUAL_INT16(0, eb::Rc::mapOutput(c, 1499));
  TEST_ASSERT_EQUAL_INT16(1000, eb::Rc::mapOutput(c, 1500));
}

// ---------------------------------------------------------------------------
// Link, failsafe and arming

void test_rc_starts_in_failsafe() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::Rc rc;
  rc.update(cfg, 100);
  TEST_ASSERT_FALSE(rc.linkUp());
  TEST_ASSERT_TRUE(rc.failsafe());
  TEST_ASSERT_FALSE(rc.armed());
  TEST_ASSERT_EQUAL_UINT16(0xFFFF, rc.ageMs(100));
  for (int i = 0; i < 16; ++i) TEST_ASSERT_EQUAL_INT16(0, rc.out(i));
}

void test_failsafe_after_timeout_and_recovery() {
  eb::Config cfg;
  eb::configDefaults(cfg);  // failsafe_ms = 500
  cfg.out[1].fs = -250;
  eb::Rc rc;
  Sticks s;
  s.ticks[0] = 1811;
  s.ticks[1] = 1811;

  rc.onChannels(s.ticks, 1000);
  rc.update(cfg, 1000);
  TEST_ASSERT_TRUE(rc.linkUp());
  TEST_ASSERT_EQUAL_INT16(1000, rc.out(0));
  TEST_ASSERT_EQUAL_INT16(1000, rc.out(1));

  rc.update(cfg, 1500);  // exactly the timeout: still alive
  TEST_ASSERT_TRUE(rc.linkUp());
  TEST_ASSERT_EQUAL_UINT16(500, rc.ageMs(1500));

  rc.update(cfg, 1501);
  TEST_ASSERT_FALSE(rc.linkUp());
  TEST_ASSERT_TRUE(rc.failsafe());
  TEST_ASSERT_EQUAL_INT16(0, rc.out(0));
  TEST_ASSERT_EQUAL_INT16(-250, rc.out(1));
  TEST_ASSERT_EQUAL_UINT16(1, rc.failsafeCount());

  rc.onChannels(s.ticks, 1600);
  rc.update(cfg, 1600);
  TEST_ASSERT_TRUE(rc.linkUp());
  TEST_ASSERT_EQUAL_INT16(1000, rc.out(0));
  TEST_ASSERT_EQUAL_UINT16(1, rc.failsafeCount());
}

void test_failsafe_hold_keeps_last_value() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  cfg.out[0].hold = 1;
  eb::Rc rc;

  // Nothing to hold before the first link: the failsafe value is used.
  cfg.out[0].fs = 123;
  rc.update(cfg, 10);
  TEST_ASSERT_EQUAL_INT16(123, rc.out(0));

  Sticks s;
  s.ticks[0] = 1811;
  s.ticks[1] = 1811;
  rc.onChannels(s.ticks, 1000);
  rc.update(cfg, 1000);
  rc.update(cfg, 2000);
  TEST_ASSERT_TRUE(rc.failsafe());
  TEST_ASSERT_EQUAL_INT16(1000, rc.out(0));  // held
  TEST_ASSERT_EQUAL_INT16(0, rc.out(1));     // not held
}

void test_unmapped_output_reports_failsafe_value() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  cfg.out[2].src = 0;
  cfg.out[2].fs = 77;
  eb::Rc rc;
  Sticks s;
  s.ticks[2] = 1811;
  rc.onChannels(s.ticks, 1000);
  rc.update(cfg, 1000);
  TEST_ASSERT_TRUE(rc.linkUp());
  TEST_ASSERT_EQUAL_INT16(77, rc.out(2));
}

void test_output_source_remapping() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  cfg.out[0].src = 3;  // output 1 follows CRSF channel 3
  eb::Rc rc;
  Sticks s;
  s.ticks[2] = 172;
  rc.onChannels(s.ticks, 1000);
  rc.update(cfg, 1000);
  TEST_ASSERT_EQUAL_INT16(-1000, rc.out(0));
  TEST_ASSERT_EQUAL_INT16(-1000, rc.out(2));
}

// Feed one RC frame and let the state machine see it.
void rcFrameAt(eb::Rc& rc, const eb::Config& cfg, const uint16_t* ticks, uint32_t t) {
  rc.onChannels(ticks, t);
  rc.update(cfg, t);
}

// Hold the controls where they are for `ms`, as a 100 Hz link would: a frame
// every 10 ms. Advances `t` past the last frame.
void rcHold(eb::Rc& rc, const eb::Config& cfg, const uint16_t* ticks, uint32_t& t, uint32_t ms) {
  for (const uint32_t end = t + ms; t < end; t += 10) rcFrameAt(rc, cfg, ticks, t);
}

void test_arming_follows_switch() {
  eb::Config cfg;
  eb::configDefaults(cfg);  // arm_ch 5, arm_us 1600, arm_guard on
  eb::Rc rc;
  Sticks s;
  uint32_t t = 1000;

  rcHold(rc, cfg, s.ticks, t, kArmOffHold);
  TEST_ASSERT_TRUE(rc.linkUp());
  TEST_ASSERT_FALSE(rc.armed());

  s.ticks[4] = 1792;
  rcFrameAt(rc, cfg, s.ticks, t);
  TEST_ASSERT_TRUE(rc.armed());

  s.ticks[4] = 191;
  rcFrameAt(rc, cfg, s.ticks, t + 10);
  TEST_ASSERT_FALSE(rc.armed());

  // Once the guard has been lifted the switch arms and disarms at will.
  s.ticks[4] = 1792;
  rcFrameAt(rc, cfg, s.ticks, t + 20);
  TEST_ASSERT_TRUE(rc.armed());
}

void test_arm_guard_blocks_arming_with_switch_already_on() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::Rc rc;
  Sticks s;
  uint32_t t = 1000;
  s.ticks[4] = 1792;  // switch is on when the link first comes up

  rcHold(rc, cfg, s.ticks, t, 1000);
  TEST_ASSERT_TRUE(rc.linkUp());
  TEST_ASSERT_FALSE(rc.armed());

  s.ticks[4] = 191;
  rcHold(rc, cfg, s.ticks, t, kArmOffHold);
  TEST_ASSERT_FALSE(rc.armed());

  s.ticks[4] = 1792;
  rcFrameAt(rc, cfg, s.ticks, t);
  TEST_ASSERT_TRUE(rc.armed());
}

void test_arm_guard_needs_the_switch_off_for_a_moment() {
  // Right after the link comes up a receiver can report a channel it has not
  // received yet at a placeholder value. A single "off" reading must not be
  // enough to lift the guard.
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::Rc rc;
  Sticks s;
  uint32_t t = 1000;

  s.ticks[4] = 191;
  rcFrameAt(rc, cfg, s.ticks, t);  // one placeholder frame reading "off"
  s.ticks[4] = 1792;
  t += 10;
  rcHold(rc, cfg, s.ticks, t, 500);
  TEST_ASSERT_TRUE(rc.linkUp());
  TEST_ASSERT_FALSE(rc.armed());

  // Off for 400 ms is still too short: ELRS at its slowest takes about a
  // third of a second to send every switch channel once.
  s.ticks[4] = 191;
  rcHold(rc, cfg, s.ticks, t, 400);
  s.ticks[4] = 1792;
  rcHold(rc, cfg, s.ticks, t, 100);
  TEST_ASSERT_FALSE(rc.armed());

  // The 400 ms did not count towards the next attempt either...
  s.ticks[4] = 191;
  rcHold(rc, cfg, s.ticks, t, 400);
  s.ticks[4] = 1792;
  rcHold(rc, cfg, s.ticks, t, 100);
  TEST_ASSERT_FALSE(rc.armed());

  // ...but off for a good half second is enough.
  s.ticks[4] = 191;
  rcHold(rc, cfg, s.ticks, t, kArmOffHold);
  s.ticks[4] = 1792;
  rcFrameAt(rc, cfg, s.ticks, t);
  TEST_ASSERT_TRUE(rc.armed());
}

void test_link_loss_disarms_and_requires_rearm() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::Rc rc;
  Sticks s;
  uint32_t t = 1000;
  rcHold(rc, cfg, s.ticks, t, kArmOffHold);
  s.ticks[4] = 1792;
  rcFrameAt(rc, cfg, s.ticks, t);
  TEST_ASSERT_TRUE(rc.armed());

  t += 1000;
  rc.update(cfg, t);
  TEST_ASSERT_FALSE(rc.linkUp());
  TEST_ASSERT_FALSE(rc.armed());

  // The link returns with the switch still on: stay disarmed, however long.
  rcHold(rc, cfg, s.ticks, t, 1000);
  TEST_ASSERT_TRUE(rc.linkUp());
  TEST_ASSERT_FALSE(rc.armed());

  s.ticks[4] = 191;
  rcHold(rc, cfg, s.ticks, t, kArmOffHold);
  s.ticks[4] = 1792;
  rcFrameAt(rc, cfg, s.ticks, t);
  TEST_ASSERT_TRUE(rc.armed());
}

void test_changed_arming_rules_require_a_fresh_switch_cycle() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::Rc rc;
  Sticks s;
  uint32_t t = 1000;
  s.ticks[6] = 1792;  // CH7 happens to be on; the arm switch (CH5) is off
  rcHold(rc, cfg, s.ticks, t, kArmOffHold);
  TEST_ASSERT_FALSE(rc.armed());

  // The arm switch is moved to CH7, which is already high.
  cfg.armCh = 7;
  rc.requireRearm();
  rcHold(rc, cfg, s.ticks, t, 1000);
  TEST_ASSERT_FALSE(rc.armed());
  TEST_ASSERT_TRUE(rc.armSwitchOn());
  TEST_ASSERT_TRUE(rc.armGuarded());

  s.ticks[6] = 191;
  rcHold(rc, cfg, s.ticks, t, kArmOffHold);
  TEST_ASSERT_FALSE(rc.armGuarded());
  s.ticks[6] = 1792;
  rcFrameAt(rc, cfg, s.ticks, t);
  TEST_ASSERT_TRUE(rc.armed());

  // Requiring a re-arm while armed disarms at once...
  rc.requireRearm();
  TEST_ASSERT_FALSE(rc.armed());
  // ...and the time the switch spends off is counted from then, not before.
  s.ticks[6] = 191;
  rcHold(rc, cfg, s.ticks, t, kArmOffHold);
  rc.requireRearm();
  rcHold(rc, cfg, s.ticks, t, 100);
  s.ticks[6] = 1792;
  rcHold(rc, cfg, s.ticks, t, 100);
  TEST_ASSERT_FALSE(rc.armed());
}

void test_changed_arming_rules_never_arm_even_without_a_guard() {
  // arm_guard off means "no need to cycle the switch after a link loss". It
  // does not mean a settings change may arm the robot.
  eb::Config cfg;
  eb::configDefaults(cfg);
  cfg.armGuard = 0;
  eb::Rc rc;
  Sticks s;
  uint32_t t = 1000;
  rc.update(cfg, t);
  s.ticks[6] = 1792;
  rcHold(rc, cfg, s.ticks, t, 100);
  TEST_ASSERT_FALSE(rc.armed());

  cfg.armCh = 7;
  rc.requireRearm();
  rcHold(rc, cfg, s.ticks, t, 1000);
  TEST_ASSERT_FALSE(rc.armed());
  s.ticks[6] = 191;
  rcHold(rc, cfg, s.ticks, t, kArmOffHold);
  s.ticks[6] = 1792;
  rcFrameAt(rc, cfg, s.ticks, t);
  TEST_ASSERT_TRUE(rc.armed());

  // Removing the arm switch altogether: armed will follow the link, but only
  // from the next time the link comes up.
  s.ticks[6] = 191;
  rcHold(rc, cfg, s.ticks, t, 100);
  TEST_ASSERT_FALSE(rc.armed());
  cfg.armCh = 0;
  rc.requireRearm();
  rcHold(rc, cfg, s.ticks, t, 1000);
  TEST_ASSERT_FALSE(rc.armed());
  t += 1000;
  rc.update(cfg, t);
  TEST_ASSERT_FALSE(rc.linkUp());
  rcFrameAt(rc, cfg, s.ticks, t);
  TEST_ASSERT_TRUE(rc.armed());
}

void test_arm_threshold_limits_keep_both_states_reachable() {
  // Whatever arm_us the settings allow, an ELRS switch (999 us low, 2000 us
  // high) must be able to both arm and disarm.
  FakePlatform platform;
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::ParamRef ref;
  TEST_ASSERT_TRUE(eb::paramFind(cfg, "arm_us", ref));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "1000", platform));
  TEST_ASSERT_NOT_NULL(eb::paramSet(ref, "2000", platform));

  static const char* const kLimits[] = {"1100", "1900"};
  for (size_t i = 0; i < 2; ++i) {
    TEST_ASSERT_NULL(eb::paramSet(ref, kLimits[i], platform));
    eb::Rc rc;
    Sticks s;
    uint32_t t = 1000;
    rcHold(rc, cfg, s.ticks, t, kArmOffHold);
    s.ticks[4] = 1792;
    rcFrameAt(rc, cfg, s.ticks, t);
    TEST_ASSERT_TRUE_MESSAGE(rc.armed(), kLimits[i]);
    s.ticks[4] = 191;
    rcFrameAt(rc, cfg, s.ticks, t + 10);
    TEST_ASSERT_FALSE_MESSAGE(rc.armed(), kLimits[i]);
  }
}

void test_arm_guard_off_rearms_with_the_link() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  cfg.armGuard = 0;
  eb::Rc rc;
  Sticks s;
  s.ticks[4] = 1792;
  rc.update(cfg, 900);  // the rules are in force before the first frame
  rc.onChannels(s.ticks, 1000);
  rc.update(cfg, 1000);
  TEST_ASSERT_TRUE(rc.armed());
  rc.update(cfg, 2000);
  TEST_ASSERT_FALSE(rc.armed());
  rc.onChannels(s.ticks, 2100);
  rc.update(cfg, 2100);
  TEST_ASSERT_TRUE(rc.armed());

  // One that was never told the rules assumes the guard is wanted.
  eb::Rc untold;
  untold.onChannels(s.ticks, 1000);
  untold.update(cfg, 1000);
  TEST_ASSERT_TRUE(untold.linkUp());
  TEST_ASSERT_FALSE(untold.armed());
}

void test_no_arm_channel_means_armed_when_linked() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  cfg.armCh = 0;
  eb::Rc rc;
  Sticks s;
  rc.update(cfg, 500);
  TEST_ASSERT_FALSE(rc.armed());
  rc.onChannels(s.ticks, 1000);
  rc.update(cfg, 1000);
  TEST_ASSERT_TRUE(rc.armed());
  rc.update(cfg, 2000);
  TEST_ASSERT_FALSE(rc.armed());
}

void test_arm_switch_hysteresis() {
  eb::Config cfg;
  eb::configDefaults(cfg);  // arm_us 1600, 50 us hysteresis
  eb::Rc rc;
  uint16_t ticks[16];
  uint32_t t = 1000;
  for (int i = 0; i < 16; ++i) ticks[i] = 992;
  ticks[4] = eb::crsf::usToTicks(1000);
  rcHold(rc, cfg, ticks, t, kArmOffHold);
  ticks[4] = eb::crsf::usToTicks(1650);
  rcFrameAt(rc, cfg, ticks, t);
  TEST_ASSERT_TRUE(rc.armed());
  ticks[4] = eb::crsf::usToTicks(1580);  // below the threshold but inside the band
  rcFrameAt(rc, cfg, ticks, t + 10);
  TEST_ASSERT_TRUE(rc.armed());
  ticks[4] = eb::crsf::usToTicks(1540);
  rcFrameAt(rc, cfg, ticks, t + 20);
  TEST_ASSERT_FALSE(rc.armed());
}

void test_failsafe_is_not_undone_by_timer_wraparound() {
  // The millisecond clock wraps every 49.7 days. A frame that went stale must
  // stay stale when the clock comes back round to the same value.
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::Rc rc;
  Sticks s;
  s.ticks[0] = 1811;
  const uint32_t t0 = 0xFFFFFF00u;
  rc.onChannels(s.ticks, t0);
  rc.update(cfg, t0);
  TEST_ASSERT_TRUE(rc.linkUp());

  rc.update(cfg, t0 + 400);  // crosses the wrap, still inside the timeout
  TEST_ASSERT_TRUE(rc.linkUp());
  TEST_ASSERT_EQUAL_UINT16(400, rc.ageMs(t0 + 400));

  rc.update(cfg, t0 + 600);
  TEST_ASSERT_FALSE(rc.linkUp());

  // Step round the whole counter in chunks, as the periodic tick would.
  uint32_t t = t0 + 600;
  for (int i = 0; i < 64; ++i) {
    t += 0x04000000u;
    rc.update(cfg, t);
    TEST_ASSERT_FALSE(rc.linkUp());
  }
  rc.update(cfg, t0 + 10);  // numerically 10 ms "after" the old frame
  TEST_ASSERT_FALSE(rc.linkUp());
  TEST_ASSERT_EQUAL_INT16(0, rc.out(0));
  TEST_ASSERT_EQUAL_UINT16(0xFFFF, rc.ageMs(t0 + 10));
}

void test_rc_rate_measurement() {
  eb::Config cfg;
  eb::configDefaults(cfg);
  eb::Rc rc;
  Sticks s;
  for (uint32_t t = 1; t <= 3000; ++t) {
    if (t % 4 == 0) rc.onChannels(s.ticks, t);  // 250 Hz
    rc.update(cfg, t);
  }
  TEST_ASSERT_UINT16_WITHIN(1, 250, rc.rateHz());
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_bipolar_mapping);
  RUN_TEST(test_bipolar_reverse_and_asymmetric_ends);
  RUN_TEST(test_unipolar_mapping);
  RUN_TEST(test_mapping_survives_degenerate_calibration);

  RUN_TEST(test_rc_starts_in_failsafe);
  RUN_TEST(test_failsafe_after_timeout_and_recovery);
  RUN_TEST(test_failsafe_hold_keeps_last_value);
  RUN_TEST(test_unmapped_output_reports_failsafe_value);
  RUN_TEST(test_output_source_remapping);
  RUN_TEST(test_arming_follows_switch);
  RUN_TEST(test_arm_guard_blocks_arming_with_switch_already_on);
  RUN_TEST(test_arm_guard_needs_the_switch_off_for_a_moment);
  RUN_TEST(test_link_loss_disarms_and_requires_rearm);
  RUN_TEST(test_changed_arming_rules_require_a_fresh_switch_cycle);
  RUN_TEST(test_changed_arming_rules_never_arm_even_without_a_guard);
  RUN_TEST(test_arm_threshold_limits_keep_both_states_reachable);
  RUN_TEST(test_arm_guard_off_rearms_with_the_link);
  RUN_TEST(test_no_arm_channel_means_armed_when_linked);
  RUN_TEST(test_arm_switch_hysteresis);
  RUN_TEST(test_failsafe_is_not_undone_by_timer_wraparound);
  RUN_TEST(test_rc_rate_measurement);
  return UNITY_END();
}
