// Robot controller firmware for the ESP32: start-up and the main loop.
//
// Each job has its own file:
//
//   lib/bridge_core   everything that is not hardware: CRSF decoding, channel
//                     mapping, failsafe and arming, who drives and how the
//                     wheels move, telemetry, the host protocol and the CLI
//   board             board identity, pin rules, settings storage, watchdog
//   motors            carries the wheel speeds to the motor board
//   hiwonder_board    the Hiwonder I2C motor driver itself
//   bt_gamepad        the Bluetooth game controller (Bluepad32)
//   serial_ports      the three UARTs
//   aux_outputs       optional servo / digital pins
//   output_guard      makes every output safe the moment the chip restarts
//   status_led        the status LED
//   battery_monitor   battery voltage on an ADC pin
//
// Nothing in the loop waits for anything but one short I2C message every
// 10 ms, so every module is serviced many times per millisecond.
#include <Arduino.h>

#include "aux_outputs.h"
#include "battery_monitor.h"
#include "board.h"
#include "bt_gamepad.h"
#include "eb_bridge.h"
#include "motors.h"
#include "output_guard.h"
#include "serial_ports.h"
#include "status_led.h"

namespace {

Esp32Board board;
eb::Bridge bridge(board);

Motors motors;
SerialPorts ports;
BtGamepad gamepad;
AuxOutputs auxOutputs;
StatusLed statusLed;
BatteryMonitor battery;

}  // namespace

void setup() {
  bridge.begin(millis());  // loads the saved settings
  const eb::Config& cfg = bridge.config();

  motors.begin(cfg);  // first: stops a motor board left running by a restart
  ports.begin(cfg);
  gamepad.begin(cfg);
  auxOutputs.begin(bridge);
  statusLed.begin(cfg);
  battery.begin(cfg);

  board.startWatchdog();
}

void loop() {
  const uint32_t now = millis();
  board.feedWatchdog();

  ports.receive(bridge, now);
  gamepad.update(bridge, now);
  bridge.tick(now);

  motors.update(bridge);
  auxOutputs.update(bridge);

  ports.transmit(bridge);
  statusLed.update(bridge, now);
  battery.update(bridge, now);

  if (board.rebootDue(now)) {
    motors.stop();
    output_guard::trip();  // a restart does not reset the pins: let go of them first
    ports.flushUsb();
    ESP.restart();
  }
}
