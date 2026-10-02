# ElrsBridge C++ client

A header-only C++11 client for the mecanum controller's companion-computer
link. It lets a second microcontroller, or a desktop C++ program, follow the
robot: the handset's channels, the link quality, whether the robot is armed
and who is driving. It also sends telemetry back to the handset.

- No dynamic memory, no dependencies beyond the C++ standard headers.
- Works on any Arduino-compatible board with a hardware serial port (Teensy,
  STM32, ESP32, RP2040, Due, …) and with desktop compilers.
- Never returns old data: once updates stop for 250 ms, every accessor
  reports failsafe.

## Installation

Copy this folder into the `lib` folder of a PlatformIO project, or into the
Arduino `libraries` folder.

## Wiring

Use the controller's companion-computer UART (see
[hardware.md](../../../docs/hardware.md#companion-computer-optional)):

| Controller (ESP32) | This board |
|---|---|
| GPIO18 (host TX) | a serial RX pin |
| GPIO19 (host RX) | a serial TX pin, only needed to send telemetry |
| GND | GND |

The controller's pins are 3.3 V and not 5 V tolerant: a 5 V board needs a
level shifter or a resistor divider on the line into the ESP32. The UART runs
at 115200 baud (`host_baud`) and streams from power-up, so there is nothing
to configure.

## Usage

```cpp
#include <ElrsBridgeClient.h>

ElrsBridgeSerial<HardwareSerial> bridge(Serial1);

void setup() {
  Serial1.begin(115200);
}

void loop() {
  bridge.update();                        // read everything that has arrived

  const elrsb::State s = bridge.state();  // failsafe once updates stop
  if (s.motorsLive()) {
    // the robot is being driven, by the PS4 controller if s.gamepad()
  }
  float forward = bridge.ch(2);           // channel 2 scaled to -1.0..1.0

  bridge.sendBattery(12.4f);              // to the handset's telemetry screen
}
```

[`examples/BasicRobot`](examples/BasicRobot/BasicRobot.ino) is a complete
sketch.

## API

`ElrsBridgeSerial<SerialT>` binds the client to anything with `available()`,
`read()` and `write(buffer, length)`.

| Member | Returns |
|---|---|
| `update()` | reads the received bytes; `true` if a new update arrived |
| `state()` | the latest `elrsb::State`, or a failsafe state when stale |
| `fresh()` | `true` while updates are arriving |
| `linkUp()`, `armed()` | the handset's link and arming state; `false` when stale |
| `value(n)` | channel *n* (1–16) as −1000…1000 or 0…1000; 0 when stale |
| `ch(n)` | channel *n* scaled to −1.0…1.0 or 0.0…1.0 |
| `link()` | the latest link statistics, or a null pointer |
| `errors()` | frames rejected for a bad checksum (noise or a baud-rate mismatch) |
| `setStaleAfter(ms)` | changes the 250 ms staleness limit |

An `elrsb::State` holds `flags`, `lq`, `rssi`, `snr`, `ageMs`, `seq`, `count`
and `ch[]`, with the accessors `linkUp()`, `failsafe()`, `armed()`,
`gamepad()` and `motorsLive()`.

Telemetry: `sendBattery()`, `sendGps()`, `sendAttitude()`, `sendMode()`,
`sendVario()`, `sendBaro()`, `sendAirspeed()`, `sendRpm()`, `sendTemp()` and
`sendCrsf()`.

**On the controller's USB port** the stream only runs while it has been
requested, and a restart of the controller forgets the request. Call
`keepStream()` once in `setup()`: it requests the stream and repeats the
request whenever updates stop. The UART does not need this.

**Without Arduino**, use `ElrsBridgeClient` directly: pass every received
byte to `feed(byte, nowMs)`, read the state with `state(nowMs)`, and transmit
the frames built by the static `pack…()` functions.

The wire format is specified in [protocol.md](../../../docs/protocol.md).
