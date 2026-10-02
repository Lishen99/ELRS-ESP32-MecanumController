"""Host-side client for the ELRS Bridge.

Talks the bridge's binary protocol over a serial port (the ESP32's USB port or
its host UART). One file, needs only pyserial, runs on Python 3.6+.

    from elrs_bridge import ElrsBridge

    with ElrsBridge("/dev/ttyUSB0") as bridge:
        while True:
            s = bridge.wait_state()
            if s.motors_live:
                print("driving by", "PS4" if s.gamepad else "ELRS", s.ch(1), s.ch(2))
            bridge.send_battery(volts=12.4, amps=1.8)

`bridge.state` and `bridge.wait_state()` never hand back old data: if the
bridge stops talking (cable pulled, bridge reset) they return a failsafe state
with every channel at zero and every flag but `failsafe` false.

Run the file for a few command line tools:

    python elrs_bridge.py COM5 monitor
    python elrs_bridge.py COM5 cli status
    python elrs_bridge.py COM5 backup my_robot.txt
"""

import os
import struct
import sys
import threading
import time
import traceback
from typing import Callable, Dict, List, NamedTuple, Optional, Sequence, Tuple

import serial

__all__ = ["ElrsBridge", "State", "Link", "Info", "BridgeError"]

PROTOCOL_VERSION = 1

SYNC1 = 0xEB
SYNC2 = 0x90
MAX_PAYLOAD = 128
OUTPUT_SCALE = 1000

# bridge -> host
MSG_STATE = 0x01
MSG_LINK = 0x02
MSG_RAW_CHANNELS = 0x03
MSG_CRSF_FRAME = 0x04
MSG_INFO = 0x10
MSG_ACK = 0x11
MSG_CLI_REPLY = 0x12
# host -> bridge
MSG_GET_INFO = 0x20
MSG_SET_STREAM = 0x21
MSG_CLI = 0x22
MSG_REQUEST = 0x23
MSG_TLM_BATTERY = 0x30
MSG_TLM_GPS = 0x31
MSG_TLM_ATTITUDE = 0x32
MSG_TLM_MODE = 0x33
MSG_TLM_VARIO = 0x34
MSG_TLM_BARO = 0x35
MSG_TLM_AIRSPEED = 0x36
MSG_TLM_RPM = 0x37
MSG_TLM_TEMP = 0x38
MSG_CRSF_TX = 0x3F

FLAG_LINK_UP = 0x01
FLAG_FAILSAFE = 0x02
FLAG_ARMED = 0x04
FLAG_GAMEPAD = 0x08
FLAG_MOTORS_LIVE = 0x10

STREAM_OFF = 0
STREAM_BINARY = 1
STREAM_TEXT = 2
STREAM_OPT_RAW = 0x01

ACK_OK = 0
_ACK_NAMES = {0: "ok", 1: "bad length", 2: "bad value", 3: "unknown message", 4: "busy"}

CLI_REPLY_LAST = 0x01

MAX_MODE_TEXT = 15
MAX_RPM_VALUES = 8
MAX_TEMP_VALUES = 8
MAX_CRSF_PAYLOAD = 60


class BridgeError(Exception):
    """The bridge did not answer, or rejected a command."""


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_frame(msg_type, payload=b""):
    """Wrap a payload as `EB 90 len type payload crc16`."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long: %d bytes" % len(payload))
    body = bytes((len(payload), msg_type)) + bytes(payload)
    return bytes((SYNC1, SYNC2)) + body + struct.pack("<H", crc16(body))


class FrameParser:
    """Splits a byte stream into protocol frames and leftover text."""

    _IDLE, _SYNC, _LEN, _TYPE, _PAYLOAD, _CRC_LO, _CRC_HI = range(7)

    def __init__(self):
        self._state = self._IDLE
        self._len = 0
        self._type = 0
        self._payload = bytearray()
        self._crc_lo = 0
        self.errors = 0

    def feed(self, data):
        """Returns (frames, text): a list of (type, payload) and any non-frame bytes."""
        frames = []  # type: List[Tuple[int, bytes]]
        text = bytearray()
        for b in data:
            st = self._state
            if st == self._IDLE:
                if b == SYNC1:
                    self._state = self._SYNC
                else:
                    text.append(b)
            elif st == self._SYNC:
                if b == SYNC2:
                    self._state = self._LEN
                elif b != SYNC1:
                    self._state = self._IDLE
                    text.append(b)
            elif st == self._LEN:
                if b > MAX_PAYLOAD:
                    self.errors += 1
                    self._state = self._IDLE
                else:
                    self._len = b
                    self._state = self._TYPE
            elif st == self._TYPE:
                self._type = b
                self._payload = bytearray()
                self._state = self._PAYLOAD if self._len else self._CRC_LO
            elif st == self._PAYLOAD:
                self._payload.append(b)
                if len(self._payload) >= self._len:
                    self._state = self._CRC_LO
            elif st == self._CRC_LO:
                self._crc_lo = b
                self._state = self._CRC_HI
            else:
                self._state = self._IDLE
                body = bytes((self._len, self._type)) + bytes(self._payload)
                if crc16(body) == (self._crc_lo | (b << 8)):
                    frames.append((self._type, bytes(self._payload)))
                else:
                    self.errors += 1
        return frames, bytes(text)


class State(NamedTuple("State", [
        ("flags", int),
        ("lq", int),
        ("rssi", int),
        ("snr", int),
        ("age_ms", int),
        ("seq", int),
        ("channels", Tuple[int, ...]),
        ("timestamp", float),
])):
    """One update from the bridge.

    channels   mapped outputs as integers: -1000..1000 for bipolar outputs
               (sticks), 0..1000 for unipolar ones (switches, sliders)
    lq         uplink link quality at the robot, percent (0 while unlinked)
    rssi, snr  uplink signal strength (dBm) and signal-to-noise ratio (dB)
    age_ms     how old the stick data was when the bridge sent this update
    timestamp  time.monotonic() when this update was received
    """

    __slots__ = ()

    @property
    def link_up(self):
        return bool(self.flags & FLAG_LINK_UP)

    @property
    def failsafe(self):
        """True when `channels` holds failsafe values rather than live sticks."""
        return bool(self.flags & FLAG_FAILSAFE)

    @property
    def armed(self):
        """The ELRS handset's arm switch is on and the arming rules are met."""
        return bool(self.flags & FLAG_ARMED)

    @property
    def gamepad(self):
        """The Bluetooth controller, not the ELRS handset, is in charge of the wheels."""
        return bool(self.flags & FLAG_GAMEPAD)

    @property
    def motors_live(self):
        """The wheels follow the sticks of whichever controller is in charge."""
        return bool(self.flags & FLAG_MOTORS_LIVE)

    def ch(self, number):
        """Output `number` (1-based, like the handset's CH1..CH16) scaled to
        -1.0..1.0 (bipolar) or 0.0..1.0 (unipolar). Missing outputs read 0."""
        if 1 <= number <= len(self.channels):
            return self.channels[number - 1] / float(OUTPUT_SCALE)
        return 0.0


def _failsafe_state():
    return State(FLAG_FAILSAFE, 0, 0, 0, 0xFFFF, 0, (0,) * 16, 0.0)


Link = NamedTuple("Link", [
    ("up_rssi1", int),        # dBm, antenna 1
    ("up_rssi2", int),        # dBm, antenna 2
    ("up_lq", int),           # %
    ("up_snr", int),          # dB
    ("antenna", int),
    ("rf_mode", int),         # ELRS packet-rate index as reported by the receiver
    ("tx_power_mw", int),     # handset transmit power
    ("down_rssi", int),
    ("down_lq", int),
    ("down_snr", int),
    ("flags", int),
    ("rc_rate_hz", int),      # measured RC frames per second
    ("vbat_mv", int),         # battery voltage measured by the bridge, 0 = not fitted
    ("uptime_ms", int),
    ("rc_frames", int),
    ("crsf_errors", int),
    ("failsafe_count", int),
    ("tx_dropped", int),      # frames the bridge dropped on this port
])

Info = NamedTuple("Info", [
    ("protocol", int),
    ("version", str),
    ("channels", int),
    ("aux_outputs", int),
    ("board", str),
])

_LINK_FORMAT = "<bbBbBBHbBbBHHIIIHH"


def _parse_state(payload, timestamp):
    if len(payload) < 8:
        return None
    flags, lq, rssi, snr, age_ms, seq, count = struct.unpack_from("<BBbbHBB", payload)
    if count > 16 or len(payload) < 8 + 2 * count:
        return None
    channels = struct.unpack_from("<%dh" % count, payload, 8)
    return State(flags, lq, rssi, snr, age_ms, seq, tuple(channels), timestamp)


class ElrsBridge:
    """Connection to an ELRS Bridge.

    port         serial device ("COM5", "/dev/ttyUSB0") or a pyserial URL
                 ("socket://localhost:5761" for the desktop simulator)
    baud         must match the bridge's usb_baud / host_baud setting
    stream_hz    updates per second to ask for; None keeps the bridge's stream_hz
    stale_after  seconds without an update before `state` reports failsafe
    raw          also receive unmapped pulse widths (see `raw_channels`)
    on_state     called from the reader thread for every update
    reconnect    keep trying to reopen the port if it disappears (a USB cable
                 knocked out and pushed back in); state reads failsafe meanwhile

    Once open, the connection looks after itself: if the bridge restarts it is
    asked for the stream again, and if the port vanishes it is reopened when it
    returns. Neither needs any handling in the control loop.
    """

    def __init__(self, port, baud=115200, stream_hz=None, stale_after=0.25, raw=False,
                 on_state=None, reconnect=True):
        self.port = port
        self.baud = baud
        self.stream_hz = stream_hz
        self.stale_after = stale_after
        self.raw = raw
        self.on_state = on_state  # type: Optional[Callable[[State], None]]
        self.reconnect = reconnect

        self._serial = None
        self._port_lock = threading.Lock()  # guards writes and swapping _serial
        self._thread = None  # type: Optional[threading.Thread]
        self._running = False
        self._parser = FrameParser()
        self._cond = threading.Condition()
        self._state = None  # type: Optional[State]
        self._state_count = 0
        self._link = None  # type: Optional[Link]
        self._raw = None  # type: Optional[Tuple[int, ...]]
        self._info = None  # type: Optional[Info]
        self._acks = {}  # type: Dict[int, int]
        self._cli_lock = threading.Lock()
        self._cli_tag = 0                # tag of the CLI command now awaiting its reply
        self._cli_chunks = []  # type: List[bytes]
        self._cli_done = False
        self._crsf_frames = []  # type: List[Tuple[int, int, bytes]]
        self._stream_mode = STREAM_BINARY
        self._keep_stream = False        # re-request the stream when it stops
        self._last_stream_request = 0.0
        self._callback_failed = False
        self.text = bytearray()  # plain text seen on the port (boot messages, CLI output)

    # -- lifecycle ---------------------------------------------------------

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def open(self, timeout=5.0):
        """Open the port and start the binary stream. Raises BridgeError if the
        port cannot be opened or nothing that looks like a bridge answers within
        `timeout` seconds."""
        if self._thread is not None:
            return
        # Nothing from an earlier session may count as evidence of a bridge now.
        with self._cond:
            self._state = None
            self._state_count = 0
            self._link = None
            self._raw = None
            self._info = None
            self._acks.clear()
        try:
            self._open_port()
        except (OSError, ValueError, serial.SerialException) as exc:
            raise BridgeError("cannot open %s: %s" % (self.port, exc))
        self._running = True
        self._thread = threading.Thread(target=self._reader, name="elrs-bridge-reader",
                                        daemon=True)
        self._thread.start()

        # Opening the port can reset the board, so keep asking until it has
        # booted. A host UART wired receive-only never acknowledges; seeing the
        # stream is enough in that case.
        deadline = time.monotonic() + timeout
        acked = False
        while time.monotonic() < deadline and not acked:
            with self._cond:
                self._acks.pop(MSG_SET_STREAM, None)
            self._last_stream_request = time.monotonic()
            self._send_quiet(MSG_SET_STREAM, self._stream_request())
            self._send_quiet(MSG_GET_INFO)
            with self._cond:
                self._cond.wait_for(lambda: MSG_SET_STREAM in self._acks, 0.4)
                acked = self._acks.get(MSG_SET_STREAM) == ACK_OK
        with self._cond:
            streaming = self._state_count > 0
        if not acked and not streaming:
            self.close()
            raise BridgeError("no ELRS bridge answered on %s at %d baud" % (self.port, self.baud))
        self._keep_stream = True

    def close(self):
        self._running = False
        self._keep_stream = False
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.5)
        self._thread = None
        self._drop_port()

    # -- control data ------------------------------------------------------

    @property
    def state(self):
        """The latest update, or a failsafe state if it is older than
        `stale_after` seconds (or none has arrived)."""
        with self._cond:
            state = self._state
        if state is None or time.monotonic() - state.timestamp > self.stale_after:
            return _failsafe_state()
        return state

    def wait_state(self, timeout=None):
        """Block until the next update and return it. If none arrives within
        `timeout` (default: `stale_after`) a failsafe state is returned."""
        if timeout is None:
            timeout = self.stale_after
        with self._cond:
            seen = self._state_count
            if self._cond.wait_for(lambda: self._state_count != seen, timeout):
                return self._state
        return _failsafe_state()

    @property
    def connected(self):
        """True while updates are arriving from the bridge."""
        with self._cond:
            state = self._state
        return state is not None and time.monotonic() - state.timestamp <= self.stale_after

    @property
    def link(self):
        """Latest link statistics and bridge counters, or None."""
        with self._cond:
            return self._link

    @property
    def raw_channels(self):
        """Unmapped pulse widths in microseconds (needs raw=True), or None."""
        with self._cond:
            return self._raw

    def crsf_frames(self):
        """CRSF frames forwarded by the bridge since the last call, as
        (address, type, payload) tuples. Needs the bridge setting fwd_crsf on."""
        with self._cond:
            frames, self._crsf_frames = self._crsf_frames, []
        return frames

    # -- telemetry to the handset ------------------------------------------
    #
    # Telemetry is best effort. These calls never raise because the port is
    # down - they return False instead - so a lost cable cannot throw an
    # exception into the middle of a control loop. Bad arguments still raise.

    def send_battery(self, volts, amps=0.0, used_mah=0.0, remaining_pct=0):
        pct = max(0, min(100, int(round(remaining_pct))))
        return self._send_quiet(MSG_TLM_BATTERY, struct.pack("<fffB", volts, amps, used_mah, pct))

    def send_gps(self, lat, lon, speed=0.0, heading=0.0, alt=0.0, sats=0):
        """lat/lon in degrees, speed in m/s, heading in degrees, alt in metres."""
        return self._send_quiet(MSG_TLM_GPS, struct.pack(
            "<iifffB", int(round(lat * 1e7)), int(round(lon * 1e7)), speed, heading, alt,
            max(0, min(255, int(sats)))))

    def send_attitude(self, roll, pitch, yaw):
        """Angles in radians."""
        return self._send_quiet(MSG_TLM_ATTITUDE, struct.pack("<fff", roll, pitch, yaw))

    def send_mode(self, text):
        """Short status text (shown as the flight mode). EdgeTX displays the
        first 13 characters. An empty string hands the line back to the
        bridge's own ARMED/DISARMED status."""
        return self._send_quiet(MSG_TLM_MODE, text.encode("ascii", "replace")[:MAX_MODE_TEXT])

    def send_vario(self, vspeed):
        """Vertical speed in m/s."""
        return self._send_quiet(MSG_TLM_VARIO, struct.pack("<f", vspeed))

    def send_baro(self, alt, vspeed=0.0):
        """Altitude in metres and vertical speed in m/s."""
        return self._send_quiet(MSG_TLM_BARO, struct.pack("<ff", alt, vspeed))

    def send_airspeed(self, speed):
        """Speed in m/s."""
        return self._send_quiet(MSG_TLM_AIRSPEED, struct.pack("<f", speed))

    def send_rpm(self, values, source=0):
        """Up to 8 RPM values, e.g. one per wheel or motor."""
        values = [int(round(v)) for v in values]
        if not 1 <= len(values) <= MAX_RPM_VALUES:
            raise ValueError("send_rpm takes 1..%d values" % MAX_RPM_VALUES)
        return self._send_quiet(MSG_TLM_RPM, struct.pack("<BB%di" % len(values), source,
                                                         len(values), *values))

    def send_temp(self, values, source=0):
        """Up to 8 temperatures in degrees Celsius."""
        values = [float(v) for v in values]
        if not 1 <= len(values) <= MAX_TEMP_VALUES:
            raise ValueError("send_temp takes 1..%d values" % MAX_TEMP_VALUES)
        return self._send_quiet(MSG_TLM_TEMP, struct.pack("<BB%df" % len(values), source,
                                                          len(values), *values))

    def send_crsf(self, frame_type, payload=b""):
        """Send any CRSF frame to the receiver. The bridge adds sync, length
        and CRC; `payload` is what follows the type byte."""
        if len(payload) > MAX_CRSF_PAYLOAD:
            raise ValueError("CRSF payload too long")
        return self._send_quiet(MSG_CRSF_TX, bytes((frame_type,)) + bytes(payload))

    # -- configuration -----------------------------------------------------

    def cli(self, command, timeout=3.0):
        """Run one CLI command on the bridge and return its output. Raises
        BridgeError if the bridge reports an error or does not answer."""
        self._not_from_callback("cli")
        with self._cli_lock:
            # Each command carries a fresh tag that the bridge echoes in its
            # reply, so an answer that arrives late (after this call gave up)
            # can never be taken for the answer to the next command.
            with self._cond:
                self._cli_tag = self._cli_tag % 255 + 1
                tag = self._cli_tag
                self._cli_chunks = []
                self._cli_done = False
            self._send(MSG_CLI, bytes((tag,)) + command.encode("ascii"))
            with self._cond:
                if not self._cond.wait_for(lambda: self._cli_done, timeout):
                    raise BridgeError("no reply to %r" % command)
                output = b"".join(self._cli_chunks).decode("ascii", "replace")
        if output.startswith("ERR"):
            raise BridgeError(output.strip())
        return output

    def get(self, name):
        """Current value of one setting, as text."""
        for line in self.cli("get " + name).splitlines():
            key, sep, value = line.partition(" = ")
            if sep and key == name:
                return value
        raise BridgeError("unexpected reply to get " + name)

    def set(self, name, value):
        """Change a setting in RAM. Call save() to keep it across reboots."""
        if isinstance(value, bool):
            value = "on" if value else "off"
        return self.cli("set %s %s" % (name, value)).strip()

    def save(self):
        return self.cli("save").strip()

    def settings(self):
        """Every setting as a {name: value} dict."""
        result = {}
        for line in self.cli("dump").splitlines():
            parts = line.split(None, 2)
            if len(parts) == 3 and parts[0] == "set":
                result[parts[1]] = parts[2]
        return result

    def names(self):
        """Names of the 16 outputs, in order."""
        settings = self.settings()
        return [settings.get("out%d.name" % i, "ch%d" % i) for i in range(1, 17)]

    def info(self, timeout=2.0):
        self._not_from_callback("info")
        with self._cond:
            self._info = None
        self._send(MSG_GET_INFO)
        with self._cond:
            if not self._cond.wait_for(lambda: self._info is not None, timeout):
                raise BridgeError("no reply to the info request")
            return self._info

    def set_stream(self, mode=STREAM_BINARY, hz=None, raw=None, timeout=2.0):
        """Change what the bridge sends on this port. The request is repeated
        automatically if the bridge restarts."""
        self._not_from_callback("set_stream")
        if raw is not None:
            self.raw = raw
        if hz is not None:
            self.stream_hz = hz
        self._stream_mode = mode
        with self._cond:
            self._acks.pop(MSG_SET_STREAM, None)
        self._last_stream_request = time.monotonic()
        self._send(MSG_SET_STREAM, self._stream_request())
        with self._cond:
            if not self._cond.wait_for(lambda: MSG_SET_STREAM in self._acks, timeout):
                raise BridgeError("no reply to the stream request")
            status = self._acks[MSG_SET_STREAM]
        if status != ACK_OK:
            raise BridgeError("stream request refused: " + _ACK_NAMES.get(status, str(status)))

    # -- internals ---------------------------------------------------------

    def _stream_request(self):
        options = STREAM_OPT_RAW if self.raw else 0
        return struct.pack("<BHB", self._stream_mode, self.stream_hz or 0, options)

    def _open_port(self):
        ser = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.05,
                                    write_timeout=1.0, do_not_open=True)
        # On ESP32 dev boards DTR/RTS drive the reset and boot pins. Leave both
        # released so opening the port does not hold the board in reset.
        try:
            ser.dtr = False
            ser.rts = False
        except (AttributeError, OSError, serial.SerialException):
            pass
        ser.open()
        self._parser = FrameParser()
        with self._port_lock:
            self._serial = ser

    def _drop_port(self):
        with self._port_lock:
            ser, self._serial = self._serial, None
        if ser is not None:
            try:
                ser.close()
            except (OSError, serial.SerialException):
                pass

    def _send(self, msg_type, payload=b""):
        """Write one frame. Raises BridgeError if the port is down."""
        frame = encode_frame(msg_type, payload)
        with self._port_lock:
            ser = self._serial
            if ser is None:
                raise BridgeError("not connected to the bridge on %s" % self.port)
            try:
                ser.write(frame)
            except (OSError, serial.SerialException) as exc:
                raise BridgeError("serial write failed: %s" % exc)

    def _send_quiet(self, msg_type, payload=b""):
        """Write one frame if possible. Returns whether it was written."""
        try:
            self._send(msg_type, payload)
            return True
        except BridgeError:
            return False

    def _reader(self):
        next_attempt = 0.0
        while self._running:
            ser = self._serial
            if ser is None:
                # The port went away. State goes stale on its own, which the
                # owner sees as failsafe; keep trying to get the port back.
                if not self.reconnect:
                    break
                if time.monotonic() < next_attempt:
                    time.sleep(0.05)
                    continue
                next_attempt = time.monotonic() + 1.0
                try:
                    self._open_port()
                    self._last_stream_request = 0.0
                except (OSError, ValueError, serial.SerialException):
                    pass
                continue

            try:
                data = ser.read(max(1, getattr(ser, "in_waiting", 0) or 1))
            except (OSError, serial.SerialException, TypeError, AttributeError):
                if not self._running:
                    break
                self._drop_port()
                next_attempt = time.monotonic() + 1.0
                continue

            if data:
                frames, text = self._parser.feed(data)
                if text:
                    self.text.extend(text)
                    del self.text[:-4096]
                for msg_type, payload in frames:
                    self._handle(msg_type, payload)
            self._keep_stream_alive()

    def _keep_stream_alive(self):
        # The bridge forgets a requested stream when it restarts (its USB port
        # comes back up as a plain CLI). If updates stop, ask again, at most
        # once a second.
        if not self._keep_stream or self._stream_mode != STREAM_BINARY:
            return
        now = time.monotonic()
        if now - self._last_stream_request < 1.0:
            return
        with self._cond:
            last = self._state.timestamp if self._state is not None else 0.0
        if now - last < 1.0:
            return
        self._last_stream_request = now
        self._send_quiet(MSG_SET_STREAM, self._stream_request())

    def _handle(self, msg_type, payload):
        callback_state = None
        with self._cond:
            if msg_type == MSG_STATE:
                state = _parse_state(payload, time.monotonic())
                if state is not None:
                    self._state = state
                    self._state_count += 1
                    callback_state = state
            elif msg_type == MSG_LINK:
                if len(payload) >= struct.calcsize(_LINK_FORMAT):
                    self._link = Link(*struct.unpack_from(_LINK_FORMAT, payload))
            elif msg_type == MSG_RAW_CHANNELS:
                if payload and len(payload) >= 1 + 2 * payload[0]:
                    self._raw = struct.unpack_from("<%dH" % payload[0], payload, 1)
            elif msg_type == MSG_CRSF_FRAME:
                if len(payload) >= 2:
                    self._crsf_frames.append((payload[0], payload[1], payload[2:]))
                    del self._crsf_frames[:-64]
            elif msg_type == MSG_INFO:
                if len(payload) >= 6:
                    self._info = Info(payload[0], "%d.%d.%d" % tuple(payload[1:4]), payload[4],
                                      payload[5], payload[6:].decode("ascii", "replace"))
            elif msg_type == MSG_ACK:
                if len(payload) >= 2:
                    self._acks[payload[0]] = payload[1]
            elif msg_type == MSG_CLI_REPLY:
                # flags, tag, text. A reply to anything but the command now
                # being waited for is a late one, and is dropped.
                if len(payload) >= 2 and payload[1] == self._cli_tag and not self._cli_done:
                    self._cli_chunks.append(payload[2:])
                    if payload[0] & CLI_REPLY_LAST:
                        self._cli_done = True
            self._cond.notify_all()
        if callback_state is not None and self.on_state is not None:
            # A bug in the callback must not stop the reader: without it the
            # connection would silently stop updating and reconnecting.
            try:
                self.on_state(callback_state)
            except Exception:  # noqa: BLE001 - report it and carry on
                if not self._callback_failed:
                    self._callback_failed = True
                    sys.stderr.write("elrs_bridge: the on_state callback raised an exception "
                                     "(reported once):\n")
                    traceback.print_exc()

    def _not_from_callback(self, what):
        # These calls wait for an answer that only the reader thread can
        # deliver, so from inside on_state they could never return.
        if threading.current_thread() is self._thread:
            raise BridgeError("%s() cannot be called from the on_state callback" % what)


# ---------------------------------------------------------------------------
# Command line tools


def _bar(value, bipolar, width=21):
    """A small text gauge for one output."""
    cells = [" "] * width
    if bipolar:
        mid = width // 2
        pos = mid + int(round(value / float(OUTPUT_SCALE) * mid))
        lo, hi = sorted((mid, max(0, min(width - 1, pos))))
        for i in range(lo, hi + 1):
            cells[i] = "="
        cells[mid] = "|"
    else:
        fill = int(round(max(0, min(OUTPUT_SCALE, value)) / float(OUTPUT_SCALE) * width))
        for i in range(fill):
            cells[i] = "="
    return "[" + "".join(cells) + "]"


def _monitor(bridge):
    settings = bridge.settings()
    names = [settings.get("out%d.name" % i, "ch%d" % i) for i in range(1, 17)]
    bipolar = [settings.get("out%d.type" % i, "bipolar") == "bipolar" for i in range(1, 17)]
    if os.name == "nt":
        os.system("")  # switches the classic Windows console to ANSI escape codes
    print("\x1b[2J", end="")
    try:
        while True:
            s = bridge.wait_state(0.5)
            connected = bridge.connected
            link = bridge.link
            raw = bridge.raw_channels
            lines = []
            if not connected:
                lines.append("NO DATA FROM THE BRIDGE - showing failsafe values")
            else:
                lines.append("link %-4s  armed %-3s  LQ %3d%%  RSSI %4d dBm  SNR %3d dB  age %d ms" % (
                    "UP" if s.link_up else "DOWN", "yes" if s.armed else "no", s.lq, s.rssi,
                    s.snr, s.age_ms))
                if s.motors_live:
                    lines.append("wheels live: %s driving" % (
                        "Bluetooth controller" if s.gamepad else "ELRS handset"))
                else:
                    lines.append("wheels stopped")
            if connected and link is not None:
                lines.append("rc %d Hz  handset %d mW  failsafes %d  crsf errors %d  dropped %d" % (
                    link.rc_rate_hz, link.tx_power_mw, link.failsafe_count, link.crsf_errors,
                    link.tx_dropped))
            lines.append("")
            for i, value in enumerate(s.channels):
                us = ("%5d us" % raw[i]) if connected and raw and i < len(raw) else ""
                lines.append("%2d %-11s %6d %s %s" % (i + 1, names[i], value,
                                                      _bar(value, bipolar[i]), us))
            lines.append("")
            lines.append("Ctrl-C to quit")
            # Home the cursor, redraw every line, then clear whatever is left below.
            print("\x1b[H" + "\n".join(line + "\x1b[K" for line in lines) + "\x1b[J", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


def _backup(bridge, path):
    text = bridge.cli("diff")
    with open(path, "w") as f:
        f.write(text)
    count = sum(1 for line in text.splitlines() if line.startswith("set "))
    print("wrote %d changed setting(s) to %s" % (count, path))


def _restore(bridge, path):
    with open(path) as f:
        lines = [line.strip() for line in f]
    commands = [line for line in lines if line.startswith("set ")]
    print(bridge.cli("defaults").strip())
    for command in commands:
        reply = bridge.cli(command).strip()
        print("%s -> %s" % (command, reply))
    print(bridge.save())
    print("reboot the bridge for pin and baud rate changes to take effect")


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(description="ELRS Bridge host tools")
    parser.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    sub = parser.add_subparsers(dest="command")
    sub.required = True
    sub.add_parser("monitor", help="live view of every channel and the link")
    sub.add_parser("info", help="firmware version and board")
    p = sub.add_parser("cli", help="run one bridge CLI command, e.g. cli status")
    p.add_argument("words", nargs="+")
    p = sub.add_parser("backup", help="write the changed settings to a file")
    p.add_argument("file")
    p = sub.add_parser("restore", help="load settings from a backup file and save them")
    p.add_argument("file")
    args = parser.parse_args(argv)

    try:
        with ElrsBridge(args.port, args.baud, stream_hz=20 if args.command == "monitor" else None,
                        raw=args.command == "monitor") as bridge:
            if args.command == "monitor":
                _monitor(bridge)
            elif args.command == "info":
                info = bridge.info()
                print("elrs-bridge %s  protocol %d  board %s  %d channels  %d aux outputs" % (
                    info.version, info.protocol, info.board, info.channels, info.aux_outputs))
            elif args.command == "cli":
                print(bridge.cli(" ".join(args.words)), end="")
            elif args.command == "backup":
                _backup(bridge, args.file)
            elif args.command == "restore":
                _restore(bridge, args.file)
    except BridgeError as exc:
        print("error: %s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
