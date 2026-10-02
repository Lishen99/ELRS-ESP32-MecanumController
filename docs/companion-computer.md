# Companion computer

The ESP32 drives the wheels on its own. A companion computer, such as a
Jetson, a Raspberry Pi, a PC or another microcontroller, can follow
everything the controller does and send telemetry to the handset's screen.
It does not command the wheels; the controllers in the robot operator's hands
always do.

The link is called the *bridge protocol*, and the clients for it are
`elrs_bridge.py` (Python) and `ElrsBridge` (C++).

## What the companion computer gets

50 times a second (`stream_hz`), a state update with:

| Field | Meaning |
|---|---|
| link up | the handset's radio link is up |
| failsafe | the channel values are failsafe substitutes, not live sticks |
| armed | the handset is armed |
| gamepad | the PS4 controller, not the handset, is in charge of the wheels |
| motors live | the wheels follow the sticks of the controller in charge |
| LQ, RSSI, SNR | link quality and signal strength at the robot |
| age | how old the stick data is, in milliseconds |
| channels | the handset's 16 channels, mapped to −1000…1000 (sticks) or 0…1000 (switches) |

Link statistics arrive 5 times a second (`link_hz`). Raw pulse widths and the
receiver's other CRSF frames can be requested too.

In the other direction it can send battery state, GPS position, attitude, a
mode text, vertical speed, altitude, airspeed, RPM and temperatures. The ESP32
forwards them to the handset, where they appear as telemetry sensors (see
[radio-setup.md](radio-setup.md#telemetry)).

## Connecting

Connect by USB or by three wires to a UART, as described in
[hardware.md](hardware.md#companion-computer-optional). Both run at 115200
baud by default (`usb_baud`, `host_baud`). The UART streams binary updates
from power-up. The USB port starts in command-line mode; the Python client
switches it to streaming by itself, and the C++ client does so after
`keepStream()`.

## Python

[`host/python/elrs_bridge.py`](../host/python/elrs_bridge.py) is a single file
that needs Python 3.6 or newer and `pyserial`. Copy it next to your program,
or put `host/python` on the import path.

```bash
pip install pyserial
```

```python
from elrs_bridge import ElrsBridge

with ElrsBridge("/dev/ttyUSB0") as bridge:
    while True:
        s = bridge.wait_state()          # the next update, 50 a second
        if s.motors_live:
            print("driven by", "PS4" if s.gamepad else "the handset")
        print("forward stick", s.ch(2), "link quality", s.lq)
        bridge.send_battery(volts=12.4, amps=1.8)
        bridge.send_mode("MAPPING")
```

`s.ch(n)` is channel *n* scaled to −1…1 for sticks and 0…1 for switches. The
connection looks after itself: if the ESP32 restarts it asks for the stream
again, and if the USB cable is unplugged it reconnects when the port returns.
Meanwhile, and whenever updates stop for 250 ms, `state` and `wait_state()`
report a failsafe state with every channel at zero.

The command line is available from code too:

```python
bridge.cli("status")             # any command, returns its text output
bridge.get("drive_max")          # one setting
bridge.set("drive_speed_top", 90)
bridge.save()
```

The module is also a command-line tool:

```bash
python host/python/elrs_bridge.py /dev/ttyUSB0 monitor        # live channels and link
python host/python/elrs_bridge.py /dev/ttyUSB0 cli status     # run one command
python host/python/elrs_bridge.py /dev/ttyUSB0 backup robot.txt
python host/python/elrs_bridge.py /dev/ttyUSB0 restore robot.txt
```

[`example_host.py`](../host/python/example_host.py) is a complete program.
More detail is in [host/python/README.md](../host/python/README.md).

## C++ and Arduino

[`host/cpp/ElrsBridge`](../host/cpp/ElrsBridge) is a header-only C++11 library
without dynamic memory, for a microcontroller wired to the ESP32's UART or for
desktop C++ programs. Copy the folder into an Arduino or PlatformIO `lib`
folder.

```cpp
#include <ElrsBridgeClient.h>

ElrsBridgeSerial<HardwareSerial> bridge(Serial1);

void setup() { Serial1.begin(115200); }

void loop() {
  bridge.update();
  const elrsb::State s = bridge.state();   // failsafe once updates stop
  if (s.motorsLive()) { /* the robot is being driven */ }
  bridge.sendBattery(12.4f);
}
```

See the [library's README](../host/cpp/ElrsBridge/README.md) and the
[BasicRobot example](../host/cpp/ElrsBridge/examples/BasicRobot/BasicRobot.ino).

## Without a library

`set host_stream text` (or `usb_stream text`) switches a port to one text line
per update:

```
$RC,21,100,-60,8,366,-366,0,0,1000,500,0,0,0,0,0,0,0,0,0,0*29
```

The fields are the flags, link quality, RSSI, data age in milliseconds and
the 16 channels, followed by an NMEA-style checksum. Telemetry goes the other
way as command lines, such as `tlm bat 12.4 1.8`. The format is specified in
[protocol.md](protocol.md#text-mode), which also documents the binary
protocol for writing a client in another language.

## No data means stop

If you parse the stream yourself, treat a stream that stops as failsafe: no
message arrives to say that a cable was pulled or the ESP32 restarted. The
provided clients switch to failsafe after 250 ms without an update.
