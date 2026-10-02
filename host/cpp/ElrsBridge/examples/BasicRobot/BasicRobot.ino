// Follow the robot controller from another microcontroller: see what the
// handset sends and who is driving, and send telemetry back to the handset.
// (The robot controller drives the wheels itself.)
//
// Works on any 3.3 V board with a spare hardware serial port: Teensy, STM32,
// ESP32, Pico, Due... On a 5 V board (Uno, Mega) put a level shifter or a
// resistor divider on the line going INTO the robot controller - its pins are
// not 5 V tolerant.
//
// Wiring (the robot controller's default host pins):
//
//   controller GPIO18 (host TX) -> this board's Serial1 RX
//   controller GPIO19 (host RX) <- this board's Serial1 TX   (only needed for telemetry)
//   controller GND              -- this board's GND
//
// The host port streams from power-up, so there is nothing to configure:
// call update() and read the state.
#include <ElrsBridgeClient.h>

ElrsBridgeSerial<HardwareSerial> bridge(Serial1);

float readBatteryVolts() { return 12.0f; }  // replace with your measurement

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // must match the controller's host_baud
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif
}

void loop() {
  bridge.update();

  // state() turns into a failsafe state (everything off) as soon as the
  // updates stop, so it is always safe to act on.
  const elrsb::State s = bridge.state();

#ifdef LED_BUILTIN
  // Light the LED while the wheels follow a controller's sticks.
  digitalWrite(LED_BUILTIN, s.motorsLive() ? HIGH : LOW);
#endif

  // Twice a second, send something back to the handset's telemetry screen.
  static uint32_t lastTelemetryMs = 0;
  if (millis() - lastTelemetryMs >= 500) {
    lastTelemetryMs = millis();
    bridge.sendBattery(readBatteryVolts());
    bridge.sendMode(s.motorsLive() ? "DRIVING" : "IDLE");
  }

  // Once a second, print what is happening.
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs >= 1000) {
    lastPrintMs = millis();
    Serial.print(bridge.fresh() ? "controller ok" : "NO DATA");
    Serial.print(s.linkUp() ? "  ELRS link up" : "  ELRS link down");
    Serial.print(s.armed() ? "  armed" : "  disarmed");
    if (s.motorsLive()) Serial.print(s.gamepad() ? "  PS4 driving" : "  handset driving");
    Serial.print("  LQ ");
    Serial.print(s.lq);
    Serial.print("%  ch1 ");
    Serial.print(bridge.value(1));
    Serial.print("  ch2 ");
    Serial.println(bridge.value(2));
  }
}
