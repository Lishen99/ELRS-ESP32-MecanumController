# Bridge protocol

How the ESP32 running this firmware ("the bridge" below) talks to a companion
computer ("the host"). This page is only needed for writing a new client: the
Python module (`host/python/elrs_bridge.py`) and the C++ library
(`host/cpp/ElrsBridge`) already implement it, as described in
[companion-computer.md](companion-computer.md).

The same protocol runs on both host-facing ports: the USB serial port and the
host UART. Each port can be in one of three modes:

| Mode | What the bridge sends | Default on |
|---|---|---|
| `off` | Nothing unprompted. Text CLI with echo and a `> ` prompt. | USB |
| `bin` | Binary frames (below) at `stream_hz`. | host UART |
| `text` | `$RC,...` lines at `stream_hz`. | — |

The boot mode of each port is a setting (`usb_stream`, `host_stream`). A client
can also switch the mode of the port it is connected to for the current
session with a `SET_STREAM` frame or the `stream` command. That choice is
forgotten when the ESP32 restarts, so a client relying on it must ask again if
the stream stops. The Python client does this by itself and the C++ client
does after `keepStream()`: once a second for as long as no updates arrive.

Binary frames and CLI text share a port. In any mode the bridge accepts both
binary frames and text command lines, and in binary mode it may emit text (the
reply to a typed command). A parser must therefore skip bytes that are not part
of a frame. Both sync bytes are `>= 0x80`, so they never occur in CLI text.

## Binary framing

```
+------+------+-----+------+----------------+--------+--------+
| 0xEB | 0x90 | LEN | TYPE | PAYLOAD (LEN)  | CRC lo | CRC hi |
+------+------+-----+------+----------------+--------+--------+
```

- `LEN` is the payload length, 0 to 128.
- The CRC is CRC-16/CCITT-FALSE (polynomial `0x1021`, initial value `0xFFFF`,
  no reflection, no final XOR) over `LEN`, `TYPE` and the payload. The check
  value for the ASCII string `123456789` is `0x29B1`.
- All multi-byte fields are little-endian. Floats are IEEE-754 single precision.

If a frame stalls part-way, the bridge abandons it once 100 ms pass without a
byte.

## Bridge to host

### `0x01` STATE — the periodic stream

Sent `stream_hz` times a second whether or not the radio link is up.

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | flags | bit 0 `LINK_UP`, bit 1 `FAILSAFE`, bit 2 `ARMED`, bit 3 `GAMEPAD`, bit 4 `MOTORS_LIVE` |
| 1 | u8 | lq | uplink link quality at the robot, 0–100 %. 0 while the link is down. |
| 2 | i8 | rssi | uplink RSSI, dBm (last reported value) |
| 3 | i8 | snr | uplink SNR, dB (last reported value) |
| 4 | u16 | age_ms | time since the last RC frame from the receiver. `0xFFFF` = none yet, or too long ago to express. |
| 6 | u8 | seq | increments with every STATE frame built for this port; a gap means updates were skipped or lost |
| 7 | u8 | count | number of channel values that follow (`stream_ch`, 1–16) |
| 8 | i16 × count | ch | mapped outputs |

Output values are −1000…+1000 for bipolar outputs and 0…+1000 for unipolar
ones.

- `LINK_UP`: RC frames are arriving (the newest is younger than `failsafe_ms`).
- `FAILSAFE`: the channel values are failsafe substitutes, not live sticks.
  Currently always the opposite of `LINK_UP`.
- `ARMED`: the link is up and the ELRS arming rules are satisfied.
- `GAMEPAD`: the Bluetooth controller, not the ELRS handset, is in charge of
  the wheels.
- `MOTORS_LIVE`: the wheels follow the sticks of whichever controller is in
  charge. False while disarmed, while nobody is in charge, and while a
  Bluetooth controller that has just taken over waits for its sticks to be
  centred.

The channel values are always the ELRS handset's, whoever is driving.

**A host must treat missing STATE frames as failsafe.** If the cable is pulled
or the bridge resets, no frame will arrive to say so. The provided clients
report failsafe when no STATE has arrived for 250 ms.

**The stream is never late.** The bridge keeps only the newest update for each
port. If the port is too slow for the rate asked of it, or is busy with a long
reply, an update that has not started to go out is replaced by the next one
rather than queued behind it. A host therefore sees gaps in `seq`, but every
STATE it does receive was current when it left. Replies (`ACK`, `INFO`,
`CLI_REPLY`, requested frames) are queued and take turns with the stream, so
neither holds the other up.

### `0x02` LINK — link statistics and counters

Sent `link_hz` times a second on ports in binary mode (5 by default).

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | i8 | up_rssi1 | uplink RSSI, antenna 1, dBm |
| 1 | i8 | up_rssi2 | uplink RSSI, antenna 2, dBm |
| 2 | u8 | up_lq | uplink link quality, % |
| 3 | i8 | up_snr | uplink SNR, dB |
| 4 | u8 | antenna | active antenna |
| 5 | u8 | rf_mode | ELRS packet-rate index as the receiver reports it (numbering differs between ELRS versions; use `rc_rate_hz` instead) |
| 6 | u16 | tx_power_mw | handset transmit power |
| 8 | i8 | down_rssi | downlink RSSI, dBm |
| 9 | u8 | down_lq | downlink link quality, % |
| 10 | i8 | down_snr | downlink SNR, dB |
| 11 | u8 | flags | as in STATE |
| 12 | u16 | rc_rate_hz | RC frames per second, measured by the bridge |
| 14 | u16 | vbat_mv | battery voltage measured by the bridge's ADC; 0 if not fitted |
| 16 | u32 | uptime_ms | |
| 20 | u32 | rc_frames | RC frames decoded since boot |
| 24 | u32 | crsf_errors | CRSF frames rejected (bad CRC / resync) |
| 28 | u16 | failsafe_count | link-loss events since boot |
| 30 | u16 | tx_dropped | updates skipped (and replies dropped) on this port because it could not keep up |

ELRS receivers leave the three downlink fields at zero and only report the
handset power in the Wide and full-resolution switch modes.

### `0x03` RAW_CHANNELS

`u8 count`, then `count` × `u16` pulse widths in microseconds, straight from
the receiver before any mapping. Sent with every STATE when the stream option
`RAW` is set, or once on request.

### `0x04` CRSF_FRAME

A CRSF frame from the receiver that the bridge does not consume itself,
forwarded when `fwd_crsf` is on: `u8 address`, `u8 type`, then the CRSF payload.

### `0x10` INFO

Reply to `GET_INFO`: `u8 protocol_version`, `u8 fw_major`, `u8 fw_minor`,
`u8 fw_patch`, `u8 channels`, `u8 aux_outputs`, then the board name as ASCII
filling the rest of the payload.

### `0x11` ACK

`u8 type` (the message being answered), `u8 status`: 0 ok, 1 bad length,
2 bad value, 3 unknown message, 4 busy.

Sent for `SET_STREAM`, and for any message that is unknown or malformed.
Telemetry messages are not acknowledged when they are accepted.

### `0x12` CLI_REPLY

`u8 flags`, `u8 tag`, then a chunk of the command's text output (lines end
with `\n`). Bit 0 of flags marks the last chunk; `tag` is the tag of the `CLI`
message being answered. Every `CLI` message gets exactly one terminated reply,
possibly spread over several frames. Output that begins with `ERR` means the
command failed.

Match replies to commands by the tag. A client that gives up waiting and sends
another command may still receive the first one's reply; with a fresh tag per
command it cannot be mistaken for the second one's.

## Host to bridge

| Type | Name | Payload |
|---|---|---|
| `0x20` | GET_INFO | none |
| `0x21` | SET_STREAM | `u8 mode` (0 off, 1 binary, 2 text), `u16 hz` (0 = use `stream_hz`, max 500), `u8 options` (bit 0: also send RAW_CHANNELS). Applies to this port until reboot. |
| `0x22` | CLI | `u8 tag` (any value; echoed in the reply), then one command line as ASCII, no line ending, up to 127 characters |
| `0x23` | REQUEST | `u8 type`: send one STATE (`0x01`), LINK (`0x02`) or RAW_CHANNELS (`0x03`) now |
| `0x30` | TLM_BATTERY | `f32 volts`, `f32 amps`, `f32 used_mah`, `u8 remaining_percent` |
| `0x31` | TLM_GPS | `i32 latitude` and `i32 longitude` in degrees × 10⁷, `f32 speed` m/s, `f32 heading` degrees, `f32 altitude` m, `u8 satellites` |
| `0x32` | TLM_ATTITUDE | `f32 roll`, `f32 pitch`, `f32 yaw`, radians |
| `0x33` | TLM_MODE | up to 15 ASCII characters; empty = hand the line back to the bridge |
| `0x34` | TLM_VARIO | `f32` vertical speed, m/s |
| `0x35` | TLM_BARO | `f32 altitude` m, `f32 vertical speed` m/s |
| `0x36` | TLM_AIRSPEED | `f32` speed, m/s |
| `0x37` | TLM_RPM | `u8 source_id`, `u8 count` (1–8), `i32 rpm` × count |
| `0x38` | TLM_TEMP | `u8 source_id`, `u8 count` (1–8), `f32 °C` × count |
| `0x3F` | CRSF_TX | `u8 crsf_type`, then up to 60 payload bytes. The bridge adds sync, length and CRC and sends the frame to the receiver immediately. |

### How telemetry is forwarded

The bridge keeps the latest value of each telemetry item and sends one CRSF
frame to the receiver every `tlm_period_ms` (100 ms), taking the items in turn.
Send each value as often as you like — a few times a second is plenty — the
handset only ever gets the newest one.

An item the host stops refreshing is dropped after `tlm_timeout_ms` (3 s), so
the handset reports the sensor as lost instead of showing a stale number.
Nothing is sent while the radio link is down.

`CRSF_TX` bypasses all of this: the frame goes out at once and is not repeated.

What the handset can show depends on its firmware and the receiver's:

| Item | CRSF frame | Needs |
|---|---|---|
| battery, GPS, attitude, mode text, vario | `0x08` `0x02` `0x1E` `0x21` `0x07` | any ELRS 3.x / 4.x |
| baro altitude | `0x09` | ELRS 3.1.2 or newer |
| airspeed, RPM, temperature | `0x0A` `0x0C` `0x0D` | ELRS 3.5.5 or newer and EdgeTX 2.11.2 or newer |
| anything else via `CRSF_TX` | any | ELRS 3.5.6 or newer forwards arbitrary frame types; older versions drop ones they do not know |

EdgeTX shows at most the first 13 characters of the mode text.

## Text mode

For hosts that would rather parse a line than link a library. Switch a port to
it with the setting `host_stream text` / `usb_stream text`, or for one session
by sending the line `stream text` (optionally with a rate: `stream text 20`).

The bridge then sends one line per update:

```
$RC,<flags>,<lq>,<rssi>,<age_ms>,<ch1>,...,<chN>*<CS>\r\n
```

```
$RC,21,100,-60,8,366,-366,0,0,1000,500,0,0,0,0,0,0,0,0,0,0*29
```

That line says: flags 21 (link up, armed, wheels live), link quality 100 %,
RSSI −60 dBm, stick data 8 ms old, then the 16 outputs. The fields are the
same as in the STATE frame, in decimal. `<CS>` is two
uppercase hex digits: the XOR of every character between `$` and `*`, as in
NMEA. Check it, and treat a line that fails — or lines that stop arriving — as
failsafe.

Everything the host sends is a CLI command line ending in `\r` or `\n`. While a
port is streaming, the bridge does not echo what it receives or print a prompt,
and the `tlm` commands answer nothing unless something is wrong:

```
tlm bat 12.4 1.8 350 80
tlm mode DRIVING
stream off
```

Replies to other commands arrive as ordinary lines between the `$RC` lines;
anything that does not start with `$` is a command reply. A `$RC` line is
never inserted into the middle of another line.
