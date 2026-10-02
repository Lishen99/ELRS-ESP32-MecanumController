# Settings and commands

Everything about the controller is adjusted at run time with text commands.
Open a serial terminal on the ESP32's USB port at 115200 baud
(`pio device monitor` from the project folder) and type `help`.

Settings live in RAM until you `save` them; saved settings survive power
cycles and firmware updates. Settings marked *reboot* take effect after
`save` and `reboot`; everything else applies the moment you set it.

The same commands work from a program: `bridge.cli("set drive_max 70")` in
Python, or as plain text lines on a port in text mode. See
[companion-computer.md](companion-computer.md) and [protocol.md](protocol.md).

## Commands

| Command | What it does |
|---|---|
| `status` | Link, arming, who is driving, the wheel commands, the motor board, Bluetooth, counters and warnings. Start here when something is not working. |
| `ch` | Raw pulse width and mapped value of every channel. |
| `get` | Show every setting. |
| `get drive` | Show a group of settings (`drive`, `motor1`, `out1`, `aux2`, `tlm`, …). |
| `get drive_max` | Show one setting with its description and allowed range. |
| `set <name> <value>` | Change a setting. |
| `save` | Store the settings in flash. |
| `defaults` | Restore the factory settings in RAM (then `save` to keep them). |
| `diff` | Print the settings that differ from the defaults, as `set` lines. |
| `dump` | Print every setting, as `set` lines. |
| `stream <off\|bin\|text> [hz]` | Change what this port sends, until reboot. |
| `tlm <kind> <values…>` | Send a telemetry value to the handset; `tlm` alone lists the forms. |
| `bind` | Put the ELRS receiver into bind mode (ELRS 3.4 or newer). |
| `reboot` | Restart the ESP32. |
| `version` | Firmware and protocol version. |

`save`, `reboot`, `defaults` and `bind` are refused while the robot could be
moving: `ERR disarm first` while the ELRS handset is armed, `ERR stop the
wheels first` while the wheels turn. Each would take control away for a
moment (writing the flash pauses everything else on the chip, and the motor
board would carry on at its last speed meanwhile).

A failed command answers with a line starting `ERR`.

To back up or clone a configuration, copy the output of `diff` into a file and
paste it back in later, or let the Python tool do it (replace the port with
the ESP32's, for example `COM7` on Windows):

```bash
python host/python/elrs_bridge.py /dev/ttyUSB0 backup my_robot.txt
python host/python/elrs_bridge.py /dev/ttyUSB0 restore my_robot.txt
```

## Driving

| Setting | Default | Values | Meaning |
|---|---|---|---|
| `drive_src` | auto | off, auto, elrs, bt | who may drive: `auto` = the ELRS handset while its link is up, otherwise the Bluetooth controller |
| `drive_fwd` | 2 | output 0 to 16 | ELRS output that drives forward / back (0 = none) |
| `drive_strafe` | 1 | output 0 to 16 | ELRS output that drives sideways |
| `drive_turn` | 4 | output 0 to 16 | ELRS output that turns on the spot |
| `drive_speed` | 12 | output 0 to 16 | ELRS output that sets the speed (0 = always `drive_normal`) |
| `drive_speed_type` | throttle | switch, throttle | how `drive_speed` is read: `throttle` = a scroll wheel, knob or throttle stick, stopped at the bottom and `drive_speed_top` at the top; `switch` = a slow / normal / fast switch |
| `drive_speed_top` | 80 | 0 to 100 | speed with the throttle all the way up, percent of `drive_max` |
| `drive_max` | 60 | 0 to 100 | the largest speed command the motor board is sent |
| `drive_slow` | 30 | 0 to 100 | slow speed mode, percent of `drive_max` (the PS4's L2, or a speed switch) |
| `drive_normal` | 50 | 0 to 100 | normal speed mode |
| `drive_fast` | 100 | 0 to 100 | fast speed mode (the PS4's R2) |
| `drive_accel` | 2 | 1 to 100 | speed steps added every 10 ms while speeding up |
| `drive_decel` | 1 | 1 to 100 | speed steps taken off every 10 ms while slowing down |
| `drive_fs` | ramp | ramp, stop | when nobody is in control any more (link lost, controller gone): slow down at `drive_decel`, or stop at once |

At the defaults the robot goes from standstill to normal speed (30) in
150 ms and back in 300 ms, and when it reverses it slows to a stop before
speeding up the other way. [driving.md](driving.md#speed) shows what the
speed settings mean in practice.

**Who is in charge.** With `drive_src auto` the ELRS handset decides while its
link is up - armed or not, so its arm switch stops the robot whatever the
Bluetooth controller does - and the Bluetooth controller drives when the
handset is off or out of range. A Bluetooth controller that takes over only
does so once its sticks have been seen centred.

**The ELRS controls** are outputs (see [Outputs](#outputs-out1--out16)), so
everything there applies: `set out2.rev on` reverses forward/back, `outN.db`
sets the deadband, and so on. The defaults suit a mode 2 handset: right stick
moves, left stick turns, and a scroll wheel mixed to CH12 sets the speed.

**The speed control.** As a `throttle` it works like a drone's throttle: from
stopped at the bottom to `drive_speed_top` (80 % of `drive_max`) at the top,
and the sticks use as much of that as they are pushed - the wheel half way
with the stick fully forward goes as fast as the wheel at the top with the
stick half way. CH12 is the default because ExpressLRS
sends it in the finest steps in either switch mode: 16 steps in Hybrid, 64 to
128 in Wide, where CH6 to CH11 get only 6 steps in Hybrid. As a `switch` it
reads slow in its lower third, fast in its upper third, normal in between.

**The Bluetooth controller's controls** are fixed: left stick moves, right
stick left/right turns, L2 held is slow, R2 held is fast. Stick movements
under 10 % are ignored.

**When the wheels stop at once:** the ELRS arm switch goes off, the handset
takes charge while disarmed, `drive_src off`, and every restart of the ESP32.
Letting go of the sticks, and (with `drive_fs ramp`) losing the controller,
slow the robot down instead.

## Motor board

The Hiwonder 4-channel encoder motor driver, on I²C.

| Setting | Default | Values | Reboot | Meaning |
|---|---|---|---|---|
| `motor_sda` | 21 | GPIO | yes | GPIO wired to the board's SDA |
| `motor_scl` | 22 | GPIO | yes | GPIO wired to the board's SCL |
| `motor_addr` | 52 (0x34) | 1 to 127 | yes | the board's I2C address |
| `motor_type` | jgb37 | no_encoder, tt, n20, jgb37 | yes | motors fitted: jgb37 = JGB37-520, 12 V, 110 rpm |
| `motor_enc_pol` | 0 | 0, 1 | yes | encoder polarity; change it if a motor races at full speed instead of holding its speed |
| `motorN.wheel` | see below | fl, fr, bl, br | | which wheel channel MN drives |
| `motorN.rev` | see below | off, on | | channel MN is mounted the other way round |

The defaults match the reference robot's wiring: `motor1` br, `motor2` fr
reversed, `motor3` bl reversed, `motor4` fl (see
[hardware.md](hardware.md#motor-driver)).

To check them, put the robot on a stand and push forward: every wheel should
turn forward. A wheel turning backwards needs its `motorN.rev` flipped. If the
robot turns when it should go sideways, two channels are on each other's wheels:
swap their `motorN.wheel`.

## Bluetooth controller

| Setting | Default | Values | Reboot | Meaning |
|---|---|---|---|---|
| `bt_enable` | on | off, on | yes | accept a Bluetooth game controller (PS4 DualShock 4, and others Bluepad32 supports) |
| `bt_timeout_ms` | 500 | 100 to 10000 | | silence from the controller before it counts as gone |

A PS4 controller sends reports all the time, so half a second of silence means
it is out of range or has gone - sooner than Bluetooth itself notices. Some
other controllers only report changes; with those, raise `bt_timeout_ms`.

## General settings

| Setting | Default | Values | Reboot | Meaning |
|---|---|---|---|---|
| `crsf_rx_pin` | 16 | GPIO | yes | GPIO wired to the receiver's TX pad |
| `crsf_tx_pin` | 17 | GPIO | yes | GPIO wired to the receiver's RX pad |
| `crsf_baud` | 420000 | 9600 to 5250000 | yes | receiver UART baud rate |
| `crsf_invert` | off | off, on | yes | invert the receiver UART signals |
| `host_rx_pin` | 19 | GPIO, −1 = off | yes | host UART: GPIO wired to the host's TX |
| `host_tx_pin` | 18 | GPIO, −1 = off | yes | host UART: GPIO wired to the host's RX |
| `host_baud` | 115200 | 9600 to 3000000 | yes | host UART baud rate |
| `usb_baud` | 115200 | 9600 to 3000000 | yes | USB serial baud rate |
| `host_stream` | bin | off, bin, text | | what the host UART sends from boot |
| `usb_stream` | off | off, bin, text | | what the USB serial port sends from boot |
| `stream_hz` | 50 | 1 to 500 | | channel updates per second |
| `stream_ch` | 16 | 1 to 16 | | number of outputs in each update |
| `link_hz` | 5 | 0 to 50 | | link statistics messages per second (0 = off) |
| `fwd_crsf` | off | off, on | | forward other CRSF frames from the receiver to the host |
| `failsafe_ms` | 500 | 100 to 10000 | | time without RC frames before the link counts as lost |
| `arm_ch` | 5 | 0 to 16 | | CRSF channel of the arm switch (0 = armed whenever linked) |
| `arm_us` | 1600 | 1100 to 1900 | | the arm switch counts as on above this pulse width |
| `arm_guard` | on | off, on | | after boot or a link loss, arm only once the switch has been off |
| `tlm_period_ms` | 100 | 0 to 10000 | | gap between telemetry frames (0 = telemetry off) |
| `tlm_timeout_ms` | 3000 | 100 to 60000 | | stop sending a value the host has not refreshed for this long |
| `tlm_auto_mode` | on | off, on | | report ARMED/DISARMED as the mode text when the host sends none |
| `vbat_pin` | −1 | ADC GPIO, −1 = off | yes | ADC pin measuring the battery through a divider |
| `vbat_scale` | 11 | 0 to 1000 | | divider ratio: battery volts per volt at the pin |
| `led_pin` | 2 | GPIO, −1 = off | yes | status LED |
| `led_mode` | high | high, low, rgb | yes | LED type: active high, active low, or RGB pixel |
| `pwm_hz` | 50 | 50 to 400 | yes | servo pulse rate of the aux outputs |

The ESP32 refuses a pin it cannot use for that job: the USB serial pins, the
flash pins, input-only pins as outputs, non-ADC pins for `vbat_pin`. It warns
if two functions share a pin. Because the USB serial pins cannot be
reassigned, the command line is always reachable to undo a mistake.

Rates that do not divide 1000 evenly are rounded to a whole number of
milliseconds: `stream_hz 60` sends every 17 ms.

If a stream rate needs more bytes than the port's baud rate can carry, updates
are skipped instead of falling behind: what the host receives is always the
newest state. `status` shows the count. One update is 46 bytes in binary mode,
up to about 120 in text mode; 115200 baud carries 11520 bytes a second.

### Arming

The arm switch is on when its channel is above `arm_us`, and off again once it
drops 50 µs below that. An ExpressLRS switch channel reads about 1000 µs off
and 2000 µs on.

With `arm_guard on`, a robot that starts up or regains its link with the
switch already on stays disarmed. The switch has to be off for half a second
first; `status` says so while that is what is holding it back. With
`arm_guard off` the robot re-arms as soon as the link returns.

Changing `arm_ch` or `arm_us` (or running `defaults`) disarms the robot, and it
stays disarmed until the arm switch has been switched off and on again under
the new rule, whatever `arm_guard` says. With `arm_ch 0` there is no switch to
cycle, so arming then starts the next time the handset connects. Changing
`arm_guard` itself does nothing at the time: it decides what the next start-up
or link loss will ask for.

## Outputs: `out1` … `out16`

Each output is one handset channel, scaled. The ELRS drive controls read
outputs, and all sixteen are streamed to the host. By default output N follows
CRSF channel N.

| Setting | Default | Values | Meaning |
|---|---|---|---|
| `outN.name` | chN | 1–11 letters, digits, `_`, `-` | label shown by `ch` and the host tools |
| `outN.src` | N | 0 to 16 | CRSF channel feeding this output (0 = none: always reports `fs`) |
| `outN.type` | bipolar (1–4), unipolar (5–16) | bipolar, unipolar | bipolar = −1000…1000 for sticks, unipolar = 0…1000 for switches and sliders |
| `outN.rev` | off | off, on | reverse direction |
| `outN.min` | 988 (1–4), 1000 (5–16) | 800 to 2200 | pulse width at the low end, µs |
| `outN.mid` | 1500 | 800 to 2200 | pulse width at zero (bipolar only), µs |
| `outN.max` | 2012 (1–4), 2000 (5–16) | 800 to 2200 | pulse width at the high end, µs |
| `outN.db` | 20 (1–4), 0 (5–16) | 0 to 400 | deadband, µs |
| `outN.fs` | 0 | −1000 to 1000 | value reported during failsafe |
| `outN.hold` | off | off, on | during failsafe keep the last live value instead of `fs` |

How a pulse width becomes a value:

- **bipolar**: within `db` of `mid` reads 0. Above that it rises linearly to
  +1000 at `max`; below, it falls linearly to −1000 at `min`. Each side is
  scaled from its own end point, so an off-centre `mid` still gives full range
  both ways.
- **unipolar**: `min + db` and below reads 0, rising linearly to 1000 at `max`.

Values are clamped at the ends. The end-point defaults match what ExpressLRS
sends: 988–2012 µs on the four stick channels and 1000–2000 µs on the aux
channels, so a two-position switch reads exactly 0 or 1000 and the middle of a
three-position switch reads about 500.

To calibrate a channel to your handset, watch `ch` while moving the control to
each end and set `min`, `mid` and `max` to the raw values you see.

`fs` and `hold` only change what the host is told during failsafe: the wheels
always stop.

## Aux pins: `aux1` … `aux4`

Optional pins the ESP32 drives by itself. All are off by default.

| Setting | Default | Values | Reboot | Meaning |
|---|---|---|---|---|
| `auxN.pin` | −1 | GPIO, −1 = off | yes | the pin |
| `auxN.mode` | servo | servo, switch, armed, link | yes | see below |
| `auxN.src` | N | 1 to 16 | | output driving this pin (servo and switch modes) |
| `auxN.gate` | on | off, on | | only active while the ELRS handset is armed (servo and switch modes) |
| `auxN.inv` | off | off, on | | invert the level (switch, armed and link modes) |
| `auxN.min_us` | 1000 | 500 to 2500 | | servo pulse at the low end of the output |
| `auxN.max_us` | 2000 | 500 to 2500 | | servo pulse at the high end of the output |
| `auxN.fs_us` | 0 | 0 to 2500 | | servo pulse during failsafe or while gated off; 0 = no pulses |

| Mode | The pin… |
|---|---|
| `servo` | outputs servo/ESC pulses following output `src`. Set `min_us` above `max_us` to reverse. |
| `switch` | is high while output `src` is 500 or more. |
| `armed` | is high while the wheels may move: the handset armed, or the Bluetooth controller driving. A motor-enable line. |
| `link` | is high while the radio link is up. |

A pin is attached when the ESP32 starts, so a new `pin` or `mode` takes effect
after `save` and `reboot`. Until then the output keeps working on the pin and
in the mode it started with, and still goes safe on failsafe. The other aux
settings apply at once.

On failsafe a `servo` pin goes to `fs_us` and a `switch` pin goes inactive.
With `fs_us 0` the pulses stop altogether, which every servo and ESC treats as
signal loss.

Add a pull-down resistor to any aux pin whose load must be off while the ESP32
boots, restarts or is flashed: the ESP32 does not drive the pin then.

## Recipes

Let only the PS4 controller drive, or only the handset:

```
set drive_src bt
set drive_src elrs
save
```

Full speed with the speed wheel all the way up (or a lower number for a
gentler robot):

```
set drive_speed_top 100
save
```

The speed on the throttle stick (CH3) instead of a scroll wheel:

```
set drive_speed 3
save
```

The speed on a three-position switch on CH6 (slow / normal / fast, like the
PS4's triggers):

```
set drive_speed 6
set drive_speed_type switch
save
```

A faster robot, with quicker acceleration:

```
set drive_max 70
set drive_accel 3
save
```

Stop at once when the controller is lost, instead of slowing down:

```
set drive_fs stop
save
```

The robot goes backwards when the stick is pushed forward (a handset that
sends the elevator channel the other way round):

```
set out2.rev on
save
```

Turning on the other stick: `drive_turn` names the output, so `set drive_turn 1`
turns with the right stick's left/right and `set drive_strafe 4` moves sideways
with the left one. Keep the drive controls off CH3: on most handsets the
throttle stick does not spring back to the centre, so the robot would keep
going when you let go.

Battery voltage on the handset, measured by the ESP32 (100 kΩ / 10 kΩ divider
into GPIO34):

```
set vbat_pin 34
set vbat_scale 11
save
reboot
```

Compare the `vbat` line in `status` with a multimeter and nudge `vbat_scale`
until they agree.

## Changing the built-in defaults

The factory defaults are in `configDefaults()` in
[`lib/bridge_core/eb_config.cpp`](../lib/bridge_core/eb_config.cpp). Change
them there if you want every freshly flashed board to start with your
configuration.
