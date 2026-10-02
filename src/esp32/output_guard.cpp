#include "output_guard.h"

#include <Arduino.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>

namespace {

const int kMaxPins = 24;
const int kMaxActions = 4;

// Kept in RAM (not flash) because trip() can run while the flash cache is
// unavailable, in the middle of a crash.
int8_t pins[kMaxPins];
volatile uint8_t pinCount = 0;
void (*actions[kMaxActions])();
volatile uint8_t actionCount = 0;

// Switch one pin to high impedance. The same pair of writes ESP-IDF uses to
// disable an output: the pin's own output driver off, and whatever peripheral
// (a PWM timer, say) was routed to it disconnected - a routed peripheral
// brings its own output enable, which the first write does not reach.
void IRAM_ATTR releasePin(int pin) {
  if (pin < 32) {
    GPIO.enable_w1tc = 1u << pin;
  } else {
    GPIO.enable1_w1tc.val = 1u << (pin - 32);
  }
  GPIO.func_out_sel_cfg[pin].val = SIG_GPIO_OUT_IDX;
}

}  // namespace

namespace output_guard {

// Runs in whatever context the restart began in, so it touches nothing but
// registers and the functions registered for exactly this: no driver calls,
// no locks, no flash.
void IRAM_ATTR trip() {
  const uint8_t pinTotal = pinCount;
  for (uint8_t i = 0; i < pinTotal; ++i) releasePin(pins[i]);
  const uint8_t actionTotal = actionCount;
  for (uint8_t i = 0; i < actionTotal; ++i) actions[i]();
}

bool add(int pin) {
  if (pin < 0) return true;
  for (uint8_t i = 0; i < pinCount; ++i) {
    if (pins[i] == pin) return true;
  }
  if (pinCount >= kMaxPins) return false;
  pins[pinCount] = (int8_t)pin;
  pinCount = pinCount + 1;  // only once the entry is in place
  return true;
}

bool addStopAction(void (*action)()) {
  if (actionCount >= kMaxActions) return false;
  actions[actionCount] = action;
  actionCount = actionCount + 1;
  return true;
}

}  // namespace output_guard

// The hooks. Each runs first thing on its way to a restart, then carries on.
extern "C" {

// The task watchdog calls this from its interrupt, before it aborts. ESP-IDF
// declares it weak so that an application can supply its own.
void IRAM_ATTR esp_task_wdt_isr_user_handler(void) { output_guard::trip(); }

// A crash (or the abort after a watchdog trip) enters ESP-IDF's panic
// handler, which has no hook in this version. The linker option
// -Wl,--wrap=esp_panic_handler routes it through here first.
void __real_esp_panic_handler(void* info);

void IRAM_ATTR __wrap_esp_panic_handler(void* info) {
  output_guard::trip();
  __real_esp_panic_handler(info);
}

// Every restart that resets only the CPUs ends here: esp_restart() (the
// reboot command), a panic, and the brownout detector, which on this chip
// restarts without passing any other hook. -Wl,--wrap=esp_restart_noos.
void __real_esp_restart_noos(void) __attribute__((noreturn));

void IRAM_ATTR __attribute__((noreturn)) __wrap_esp_restart_noos(void) {
  output_guard::trip();
  __real_esp_restart_noos();
}

}  // extern "C"
