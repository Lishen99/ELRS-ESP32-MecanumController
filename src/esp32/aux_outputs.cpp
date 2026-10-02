#include "aux_outputs.h"

#include <Arduino.h>
#include <driver/ledc.h>
#include <soc/ledc_struct.h>

#include "output_guard.h"

namespace {

const ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
const ledc_timer_t kTimer = LEDC_TIMER_3;
const int kFirstChannel = 4;

// A servo pulse keeps its width even while the chip changes its CPU and bus
// clock speed, as it does when restarting: the timer runs from the 1 MHz
// reference tick, which does not follow them.
const ledc_clk_cfg_t kClock = LEDC_USE_REF_TICK;
const uint32_t kClockHz = 1000000;

const uint8_t kMaxBits = 14;
const uint8_t kMinBits = 8;

ledc_channel_t ledcChannel(int index) { return (ledc_channel_t)(kFirstChannel + index); }

// The finest timer resolution the clock can provide at this pulse rate: the
// timer counts 2^bits steps per period and cannot count faster than its clock.
// At 50 Hz that is 14 bits, about 1.2 us per step.
uint8_t resolutionBits(uint32_t pwmHz) {
  uint8_t bits = kMaxBits;
  while (bits > kMinBits && ((uint64_t)pwmHz << bits) > kClockHz) --bits;
  return bits;
}

// One number that identifies what a pin is being told to do.
int32_t signature(const eb::AuxCommand& cmd) {
  return cmd.kind == eb::AuxCommand::kPulse ? cmd.pulseUs : (cmd.level ? 1 : 0);
}

}  // namespace

void AuxOutputs::begin(const eb::Bridge& bridge) {
  pwmHz_ = bridge.config().pwmHz;
  pwmBits_ = resolutionBits(pwmHz_);

  // With no link yet, what the bridge asks for is each output's safe state.
  eb::AuxCommand safe[eb::kNumAux];
  bool anyPulses = false;
  for (int i = 0; i < eb::kNumAux; ++i) {
    safe[i] = bridge.aux(i);
    if (safe[i].kind == eb::AuxCommand::kDisabled) continue;
    output_guard::add(safe[i].pin);
    applied_[i] = signature(safe[i]);
    if (safe[i].kind == eb::AuxCommand::kPulse) anyPulses = true;
  }

  if (anyPulses) {
    preloadPulses(safe);
    waitForLowPart(safe);
  }

  for (int i = 0; i < eb::kNumAux; ++i) {
    const eb::AuxCommand& cmd = safe[i];
    if (cmd.kind == eb::AuxCommand::kPulse) {
      ledc_channel_config_t channel = {};
      channel.gpio_num = cmd.pin;
      channel.speed_mode = kSpeedMode;
      channel.channel = ledcChannel(i);
      channel.intr_type = LEDC_INTR_DISABLE;
      channel.timer_sel = kTimer;
      channel.duty = duty(cmd.pulseUs);
      channel.hpoint = 0;
      ledc_channel_config(&channel);
    } else if (cmd.kind == eb::AuxCommand::kLevel) {
      // Set the level before enabling the driver, so the pin comes up in its
      // inactive state with no glitch.
      digitalWrite(cmd.pin, cmd.level ? HIGH : LOW);
      pinMode(cmd.pin, OUTPUT);
    }
  }
}

// Start the pulse timer and load every servo channel with its safe pulse
// width, while no pin is connected yet.
//
// A restart does not reset the PWM hardware: a channel still holds the pulse
// width it had before, and a new one only takes effect when the timer next
// wraps round. Without this step the first pulse on a freshly connected pin
// could be an old one.
void AuxOutputs::preloadPulses(const eb::AuxCommand* safe) {
  ledc_timer_config_t timer = {};
  timer.speed_mode = kSpeedMode;
  timer.timer_num = kTimer;
  timer.duty_resolution = (ledc_timer_bit_t)pwmBits_;
  timer.freq_hz = pwmHz_;
  timer.clk_cfg = kClock;
  ledc_timer_config(&timer);

  for (int i = 0; i < eb::kNumAux; ++i) {
    if (safe[i].kind != eb::AuxCommand::kPulse) continue;
    ledc_bind_channel_timer(kSpeedMode, ledcChannel(i), kTimer);
    ledc_set_duty(kSpeedMode, ledcChannel(i), duty(safe[i].pulseUs));
    ledc_update_duty(kSpeedMode, ledcChannel(i));
  }
  delay(1000 / pwmHz_ + 5);  // one full period, and a little more
}

// Wait until the timer is past every channel's pulse, so the pins are
// connected in the low part of a period and each one's first pulse is a
// whole one. (At 50 Hz that part is some 18 ms long. Only a safe pulse that
// fills nearly the whole period leaves no room, and then the wait gives up
// after a period.)
void AuxOutputs::waitForLowPart(const eb::AuxCommand* safe) const {
  uint32_t longest = 0;
  for (int i = 0; i < eb::kNumAux; ++i) {
    if (safe[i].kind == eb::AuxCommand::kPulse && duty(safe[i].pulseUs) > longest) {
      longest = duty(safe[i].pulseUs);
    }
  }
  const uint32_t steps = 1u << pwmBits_;
  const uint32_t margin = steps / 64;  // room to connect all four pins
  const uint32_t periodUs = 1000000u / pwmHz_;
  const uint32_t start = micros();
  while ((uint32_t)(micros() - start) < periodUs) {
    const uint32_t count = LEDC.timer_group[kSpeedMode].timer[kTimer].value.timer_cnt;
    if (count > longest && count + margin < steps) return;
  }
}

void AuxOutputs::update(const eb::Bridge& bridge) {
  for (int i = 0; i < eb::kNumAux; ++i) {
    const eb::AuxCommand cmd = bridge.aux(i);
    if (cmd.kind == eb::AuxCommand::kDisabled) continue;
    const int32_t wanted = signature(cmd);
    if (wanted == applied_[i]) continue;
    applied_[i] = wanted;
    apply(i, cmd);
  }
}

void AuxOutputs::apply(int index, const eb::AuxCommand& cmd) {
  if (cmd.kind == eb::AuxCommand::kPulse) {
    ledc_set_duty(kSpeedMode, ledcChannel(index), duty(cmd.pulseUs));
    ledc_update_duty(kSpeedMode, ledcChannel(index));
  } else {
    digitalWrite(cmd.pin, cmd.level ? HIGH : LOW);
  }
}

// Duty = pulse width as a fraction of the PWM period, in timer steps. A pulse
// width of 0 gives a constant low: no pulses at all.
uint32_t AuxOutputs::duty(uint16_t pulseUs) const {
  const uint32_t steps = 1u << pwmBits_;
  const uint64_t value = (uint64_t)pulseUs * pwmHz_ * steps / 1000000ull;
  return (uint32_t)min<uint64_t>(value, steps - 1);
}
