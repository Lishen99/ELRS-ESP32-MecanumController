// Makes every output safe the moment the firmware restarts.
//
// A reboot command, a watchdog trip, a crash and (on this chip) a brownout
// all restart only the CPUs. The GPIO and PWM hardware is not reset, and
// neither is anything outside the chip: left alone, each pin keeps its last
// level or pulse train and the motor board keeps the last speed it was given,
// until the firmware is running again a second or so later.
//
// So at the start of every restart, whatever caused it:
//   - every registered pin is switched to high impedance, and stays that way
//     until its module sets it up again after boot. To the hardware behind
//     it that looks like power-up, so that hardware must treat an undriven
//     input as "off" (add a pull-down where it does not);
//   - every registered stop action runs, e.g. the one that tells the motor
//     board to stop.
//
// Power-up and the reset button reset the pins by themselves. The hooks are
// linker wraps (see platformio.ini), so they are live from the first
// instruction after reset.
#pragma once

namespace output_guard {

// Register an output pin. Call before the pin is first driven. Returns false
// if the table is full.
bool add(int pin);

// Register a function that stops something the chip controls from outside
// its own pins. It runs in whatever context the restart began in - a task,
// the watchdog interrupt, the crash handler - so it must be IRAM_ATTR and
// touch nothing but hardware registers and ROM functions. Returns false if
// the table is full.
bool addStopAction(void (*action)());

// Make everything safe now: release the pins, run the stop actions. Safe to
// call from any context, and more than once.
void trip();

}  // namespace output_guard
