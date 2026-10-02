# Python tools

A client for the controller's companion-computer link, with command-line
tools, an example program and the simulator test harness.

| File | Purpose |
|---|---|
| [`elrs_bridge.py`](elrs_bridge.py) | Client library and command-line tool |
| [`example_host.py`](example_host.py) | Example companion-computer program |
| [`fake_receiver.py`](fake_receiver.py) | Plays the ExpressLRS receiver for the desktop simulator |
| [`test_with_simulator.py`](test_with_simulator.py) | End-to-end tests against the simulator |

Requirements: Python 3.6 or newer and [pyserial](https://pypi.org/project/pyserial/).

```bash
pip install pyserial
```

`elrs_bridge.py` is a single file: copy it next to your program, or add this
folder to the import path.

## Client library

```python
from elrs_bridge import ElrsBridge

with ElrsBridge("/dev/ttyUSB0") as bridge:
    while True:
        s = bridge.wait_state()
        if s.motors_live:
            print("driven by", "PS4" if s.gamepad else "the handset")
        bridge.send_battery(volts=12.4, amps=1.8)
```

The port can be a serial device (`/dev/ttyUSB0`, `/dev/ttyTHS1`, `COM7`) or a
pyserial URL such as `socket://localhost:5761` for the simulator.

### Connection

`ElrsBridge(port, baud=115200, stream_hz=None, stale_after=0.25, raw=False, on_state=None, reconnect=True)`

| Argument | Meaning |
|---|---|
| `baud` | must match the controller's `usb_baud` or `host_baud` |
| `stream_hz` | updates per second to ask for; `None` keeps the controller's `stream_hz` |
| `stale_after` | seconds without an update before `state` reports failsafe |
| `raw` | also receive unmapped pulse widths (`raw_channels`) |
| `on_state` | callback run on the reader thread for every update |
| `reconnect` | reopen the port if it disappears, for example when a USB cable is pulled out and plugged back in |

Use it as a context manager or call `open()` and `close()`. Once open, the
connection recovers by itself: if the controller restarts, the stream is
requested again, and a vanished port is reopened when it returns.

### Reading the state

| Member | Returns |
|---|---|
| `state` | the latest update, or a failsafe state if it is older than `stale_after` |
| `wait_state(timeout=None)` | blocks for the next update; a failsafe state on timeout |
| `connected` | `True` while updates are arriving |
| `link` | the latest link statistics and counters, or `None` |
| `raw_channels` | pulse widths in microseconds (needs `raw=True`), or `None` |
| `crsf_frames()` | CRSF frames forwarded from the receiver since the last call (needs the setting `fwd_crsf on`) |

A `State` has the fields `flags`, `lq`, `rssi`, `snr`, `age_ms`, `seq`,
`channels` and `timestamp`, the properties `link_up`, `failsafe`, `armed`,
`gamepad` and `motors_live`, and `ch(n)`, which returns channel *n* (1 to 16)
scaled to −1…1 for sticks and 0…1 for switches. A failsafe state has every
channel at zero and every flag except `failsafe` false, so a control loop can
act on it without further checks.

### Telemetry to the handset

`send_battery(volts, amps=0, used_mah=0, remaining_pct=0)`,
`send_gps(lat, lon, speed=0, heading=0, alt=0, sats=0)`,
`send_attitude(roll, pitch, yaw)` (radians), `send_mode(text)`,
`send_vario(vspeed)`, `send_baro(alt, vspeed=0)`, `send_airspeed(speed)`,
`send_rpm(values, source=0)`, `send_temp(values, source=0)` and
`send_crsf(frame_type, payload)`.

Telemetry is best effort: these calls return `False` instead of raising when
the port is down, so a pulled cable never throws an exception into a control
loop. Send values a few times a second; the controller forwards the newest
value of each to the handset and drops values that stop being refreshed.

### Settings and commands

| Method | Does |
|---|---|
| `cli(command)` | runs a command-line command and returns its output |
| `get(name)` | the current value of a setting, as text |
| `set(name, value)` | changes a setting in RAM |
| `save()` | stores the settings in flash |
| `settings()` | every setting as a dictionary |
| `info()` | firmware version, protocol version and board |
| `set_stream(mode, hz=None, raw=None)` | changes what the controller sends on this port |

These raise `BridgeError` when the controller reports an error or does not
answer. They must not be called from the `on_state` callback.

## Command-line tool

```bash
python elrs_bridge.py PORT monitor            # live view of every channel and the link
python elrs_bridge.py PORT info               # firmware version and board
python elrs_bridge.py PORT cli status         # run one command
python elrs_bridge.py PORT backup robot.txt   # save the changed settings to a file
python elrs_bridge.py PORT restore robot.txt  # load them back and save them
```

`--baud` selects a baud rate other than 115200.

## Example program

[`example_host.py`](example_host.py) shows a companion computer's main loop:
it follows who is driving, reports the link, and sends battery and mode
telemetry to the handset.

```bash
python example_host.py /dev/ttyUSB0
```

## Simulator and end-to-end tests

The desktop simulator (`pio run -e sim`) runs the controller's core with TCP
sockets in place of its UARTs. `fake_receiver.py` connects to it as the
receiver and sends moving sticks (`sweep`) or centred ones (`centre`); `--arm`
switches the arm channel on after two seconds.

```bash
.pio/build/sim/program
python host/python/fake_receiver.py sweep --arm
python host/python/elrs_bridge.py socket://localhost:5761 monitor
```

`test_with_simulator.py` starts the simulator by itself and runs the
end-to-end tests:

```bash
python host/python/test_with_simulator.py
```
