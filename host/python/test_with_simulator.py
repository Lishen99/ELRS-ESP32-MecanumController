"""End-to-end tests: the Python host client against the real bridge core.

The bridge core is compiled for the desktop (the simulator) and its serial
ports are reached over TCP. A fake receiver plays the ELRS side using its own
independent CRSF implementation, so these tests exercise the whole chain:

    fake receiver --CRSF--> bridge core --host protocol--> elrs_bridge.py

Build the simulator first, then run this file:

    pio run -e sim
    python host/python/test_with_simulator.py
"""

import io
import os
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stderr, redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
SIMULATOR = os.path.join(ROOT, ".pio", "build", "sim",
                         "program.exe" if os.name == "nt" else "program")

sys.path.insert(0, HERE)
import elrs_bridge  # noqa: E402
from elrs_bridge import BridgeError, ElrsBridge  # noqa: E402
from fake_receiver import FakeReceiver, us_to_ticks  # noqa: E402

# The bridge only arms after it has seen the arm switch off for half a second;
# tests hold it off a little longer than that.
ARM_OFF_HOLD = 0.7


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_for(predicate, timeout=3.0, step=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


def div_round(num, den):
    """Integer division rounding half away from zero."""
    return (num + den // 2) // den if num >= 0 else -((-num + den // 2) // den)


def ticks_to_us(ticks):
    return 1500 + div_round((ticks - 992) * 5, 8)


def expected_bipolar(us, low=988, mid=1500, high=2012, deadband=20):
    """What a default stick output should read for a pulse width."""
    offset = us - mid
    if offset > deadband:
        value = div_round((offset - deadband) * 1000, high - mid - deadband)
    elif offset < -deadband:
        value = -div_round((-offset - deadband) * 1000, mid - low - deadband)
    else:
        value = 0
    return max(-1000, min(1000, value))


def expected_unipolar(us, low=1000, high=2000):
    """What a default switch output should read for a pulse width."""
    return max(0, min(1000, div_round((us - low) * 1000, high - low)))


class Simulator:
    def __init__(self, settings_path, ports=None):
        # Pass the ports of an earlier instance to come back up where it was,
        # like a bridge that was unplugged and plugged in again.
        self.crsf_port, self.host_port, self.usb_port = ports or (
            free_port(), free_port(), free_port())
        self.process = subprocess.Popen(
            [SIMULATOR, "--crsf", str(self.crsf_port), "--host", str(self.host_port),
             "--usb", str(self.usb_port), "--config", settings_path],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if not wait_for(lambda: self._listening() or self.process.poll() is not None, 5.0, 0.05) \
                or self.process.poll() is not None:
            self.stop()
            raise RuntimeError("the simulator did not start - rebuild it with: pio run -e sim")

    def _listening(self):
        try:
            socket.create_connection(("127.0.0.1", self.usb_port), timeout=0.2).close()
            return True
        except OSError:
            return False

    @property
    def ports(self):
        return self.crsf_port, self.host_port, self.usb_port

    @property
    def host_url(self):
        return "socket://127.0.0.1:%d" % self.host_port

    @property
    def usb_url(self):
        return "socket://127.0.0.1:%d" % self.usb_port

    def stop(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait(timeout=5)


class UsbTerminal:
    """A plain text connection to the simulator's USB port, like a serial monitor."""

    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.sock.settimeout(0.05)
        self.buffer = b""

    def send(self, line):
        self.sock.sendall(line.encode("ascii") + b"\r\n")

    def read_until(self, needle, timeout=3.0):
        deadline = time.monotonic() + timeout
        while needle.encode("ascii") not in self.buffer and time.monotonic() < deadline:
            try:
                self.buffer += self.sock.recv(4096)
            except socket.timeout:
                pass
        text = self.buffer.decode("ascii", "replace")
        self.buffer = b""
        return text

    def close(self):
        self.sock.close()


class ProtocolTests(unittest.TestCase):
    """The client's framing code on its own; no simulator needed."""

    def test_crc16_check_value(self):
        # CRC-16/CCITT-FALSE catalogue check value.
        self.assertEqual(elrs_bridge.crc16(b"123456789"), 0x29B1)

    def test_frame_layout(self):
        frame = elrs_bridge.encode_frame(0x42, b"\x01\x02\x03")
        self.assertEqual(frame[:4], bytes((0xEB, 0x90, 3, 0x42)))
        self.assertEqual(frame[4:7], b"\x01\x02\x03")
        crc = elrs_bridge.crc16(bytes((3, 0x42)) + b"\x01\x02\x03")
        self.assertEqual(frame[7:], bytes((crc & 0xFF, crc >> 8)))
        with self.assertRaises(ValueError):
            elrs_bridge.encode_frame(1, bytes(129))

    def test_parser_splits_frames_from_text(self):
        parser = elrs_bridge.FrameParser()
        stream = (b"boot text\r\n" + elrs_bridge.encode_frame(0x10, b"abc") + b"> "
                  + elrs_bridge.encode_frame(0x20))
        frames, text = [], b""
        for i in range(0, len(stream), 3):      # arrive in awkward pieces
            got, plain = parser.feed(stream[i:i + 3])
            frames += got
            text += plain
        self.assertEqual(frames, [(0x10, b"abc"), (0x20, b"")])
        self.assertEqual(text, b"boot text\r\n> ")
        self.assertEqual(parser.errors, 0)

    def test_parser_rejects_corrupted_frames(self):
        good = elrs_bridge.encode_frame(0x01, bytes(range(40)))
        for position in range(2, len(good)):
            damaged = bytearray(good)
            damaged[position] ^= 0x04
            parser = elrs_bridge.FrameParser()
            frames, _ = parser.feed(bytes(damaged))
            self.assertEqual(frames, [], "accepted a frame damaged at byte %d" % position)
        # ...and the parser recovers for the next intact frame.
        parser = elrs_bridge.FrameParser()
        damaged = bytearray(good)
        damaged[10] ^= 0xFF
        frames, _ = parser.feed(bytes(damaged) + good)
        self.assertEqual(frames, [(0x01, bytes(range(40)))])
        self.assertEqual(parser.errors, 1)

    def test_state_payload_decoding(self):
        import struct
        payload = struct.pack("<BBbbHBB4h", 5, 99, -71, -4, 0x1234, 200, 4, -1000, 0, 500, 1000)
        state = elrs_bridge._parse_state(payload, 12.5)
        self.assertEqual(state.channels, (-1000, 0, 500, 1000))
        self.assertTrue(state.link_up and state.armed and not state.failsafe)
        self.assertEqual((state.lq, state.rssi, state.snr, state.age_ms, state.seq),
                         (99, -71, -4, 0x1234, 200))
        self.assertEqual(state.ch(1), -1.0)
        self.assertEqual(state.ch(3), 0.5)
        self.assertEqual(state.ch(5), 0.0)     # not sent: reads as zero
        self.assertIsNone(elrs_bridge._parse_state(payload[:-1], 0.0))   # truncated
        self.assertIsNone(elrs_bridge._parse_state(b"\x00" * 7 + b"\x11" + bytes(34), 0.0))


class BridgeTestCase(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.settings_path = os.path.join(self.tempdir.name, "settings.txt")
        self.sim = Simulator(self.settings_path)
        self.rx = FakeReceiver(port=self.sim.crsf_port)
        self.rx.link = False
        self.rx.start()
        self.bridge = ElrsBridge(self.sim.host_url)
        self.bridge.open()

    def tearDown(self):
        self.bridge.close()
        self.rx.stop()
        self.sim.stop()
        self.tempdir.cleanup()

    def link_up(self):
        self.rx.link = True
        self.assertTrue(wait_for(lambda: self.bridge.state.link_up), "link did not come up")

    def arm(self):
        self.rx.set_us(5, 1000)
        self.link_up()
        time.sleep(ARM_OFF_HOLD)
        self.rx.set_us(5, 2000)
        self.assertTrue(wait_for(lambda: self.bridge.state.armed), "did not arm")


class ConnectionTests(BridgeTestCase):
    def test_info(self):
        info = self.bridge.info()
        self.assertEqual(info.protocol, elrs_bridge.PROTOCOL_VERSION)
        self.assertEqual(info.board, "sim")
        self.assertEqual(info.channels, 16)
        self.assertEqual(info.aux_outputs, 4)
        self.assertEqual(info.version, "0.1.0")

    def test_failsafe_before_the_handset_connects(self):
        state = self.bridge.wait_state(1.0)
        self.assertTrue(self.bridge.connected)
        self.assertFalse(state.link_up)
        self.assertTrue(state.failsafe)
        self.assertFalse(state.armed)
        self.assertEqual(state.channels, (0,) * 16)
        self.assertEqual(state.age_ms, 0xFFFF)

    def test_stream_rate(self):
        first = self.bridge.wait_state(1.0)
        time.sleep(1.0)
        last = self.bridge.wait_state(1.0)
        frames = (last.seq - first.seq) % 256
        self.assertTrue(40 <= frames <= 60, "expected about 50 updates/s, got %d" % frames)

    def test_requested_stream_rate(self):
        self.bridge.set_stream(hz=100)
        first = self.bridge.wait_state(1.0)
        time.sleep(1.0)
        last = self.bridge.wait_state(1.0)
        frames = (last.seq - first.seq) % 256
        self.assertTrue(85 <= frames <= 115, "expected about 100 updates/s, got %d" % frames)

    def test_no_bridge_on_the_port_is_reported(self):
        dead = free_port()
        with self.assertRaises((BridgeError, OSError)):
            ElrsBridge("socket://127.0.0.1:%d" % dead).open(timeout=0.5)

    def test_state_turns_failsafe_when_the_bridge_disappears(self):
        self.arm()
        self.assertTrue(self.bridge.state.armed)
        self.sim.stop()
        self.assertTrue(wait_for(lambda: not self.bridge.connected, 2.0))
        state = self.bridge.state
        self.assertTrue(state.failsafe)
        self.assertFalse(state.armed)
        self.assertFalse(state.link_up)
        self.assertEqual(state.channels, (0,) * 16)
        # wait_state must not hand back the old armed state either.
        waited = self.bridge.wait_state(0.3)
        self.assertTrue(waited.failsafe)
        self.assertFalse(waited.armed)


class RecoveryTests(BridgeTestCase):
    def test_stream_resumes_after_the_bridge_reboots(self):
        # On the USB port the stream exists only because this client asked for
        # it, and a reboot makes the bridge forget that. The client has to
        # notice the silence and ask again.
        with ElrsBridge(self.sim.usb_url, stream_hz=20) as usb:
            self.assertTrue(wait_for(lambda: usb.connected))
            usb.cli("reboot")
            self.assertTrue(wait_for(lambda: not usb.connected, 2.0), "stream never stopped")
            self.assertTrue(usb.state.failsafe)
            self.assertTrue(wait_for(lambda: usb.connected, 5.0), "stream did not come back")
            self.rx.set_us(1, 2012)
            self.rx.link = True
            self.assertTrue(wait_for(lambda: usb.state.channels[0] == 1000))

    def test_client_reconnects_when_the_port_comes_back(self):
        self.rx.set_us(1, 2012)
        self.link_up()
        ports = self.sim.ports
        self.sim.stop()
        self.assertTrue(wait_for(lambda: not self.bridge.connected, 2.0))
        self.assertTrue(self.bridge.state.failsafe)
        # Telemetry is best effort: with the port gone it is dropped, not raised,
        # so a control loop is never knocked over between two motor commands.
        self.bridge.send_battery(12.0)
        self.bridge.send_mode("STILL HERE")
        # Commands that need an answer do say so.
        with self.assertRaises(BridgeError):
            self.bridge.cli("status")

        self.rx.stop()
        self.sim = Simulator(self.settings_path, ports)
        self.rx = FakeReceiver(port=self.sim.crsf_port)
        self.rx.set_us(1, 2012)
        self.rx.start()
        self.assertTrue(wait_for(lambda: self.bridge.connected, 6.0), "client did not reconnect")
        self.assertTrue(wait_for(lambda: self.bridge.state.channels[0] == 1000, 3.0))
        self.assertEqual(self.bridge.get("stream_hz"), "50")
        self.bridge.send_battery(12.6, 1.5, 1234, 80)
        self.assertTrue(wait_for(lambda: self.rx.frames(0x08), 3.0))


class ClientTests(BridgeTestCase):
    """How the client copes when the program using it misbehaves."""

    def test_a_failing_callback_does_not_stop_the_updates(self):
        calls = []

        def on_state(state):
            calls.append(state)
            raise RuntimeError("a bug in the robot's own code")

        complaints = io.StringIO()
        with redirect_stderr(complaints):
            with ElrsBridge(self.sim.usb_url, stream_hz=50, on_state=on_state) as usb:
                self.assertTrue(wait_for(lambda: len(calls) > 20), "updates stopped")
                self.assertTrue(usb.connected)
                self.assertEqual(usb.get("stream_hz"), "50")
        # Reported once, not fifty times a second.
        self.assertEqual(complaints.getvalue().count("on_state callback raised"), 1)
        self.assertIn("RuntimeError", complaints.getvalue())

    def test_blocking_calls_are_refused_inside_the_callback(self):
        # cli() waits for an answer that only the reader thread can deliver, so
        # calling it from the reader thread's own callback would hang for good.
        caught = []

        def on_state(state):
            if not caught:
                try:
                    usb.cli("status")
                    caught.append("no error")
                except BridgeError as exc:
                    caught.append(str(exc))

        usb = ElrsBridge(self.sim.usb_url, stream_hz=50, on_state=on_state)
        with usb:
            self.assertTrue(wait_for(lambda: caught))
            self.assertIn("cannot be called from the on_state callback", caught[0])
            self.assertIn("link:", usb.cli("status"))   # fine from any other thread

    def test_a_late_reply_is_not_taken_for_the_next_answer(self):
        # The caller gives up on the first command before its answer arrives.
        with self.assertRaises(BridgeError):
            self.bridge.cli("dump", timeout=0)
        self.assertTrue(self.bridge.cli("version").startswith("elrs-bridge"))
        self.assertEqual(self.bridge.get("stream_hz"), "50")

    def test_reopening_does_not_trust_the_previous_session(self):
        # Pointed at a port with nothing behind it, the client must say so
        # rather than report success because the last session worked.
        silent = socket.socket()
        silent.bind(("127.0.0.1", 0))
        silent.listen(1)
        try:
            self.assertTrue(wait_for(lambda: self.bridge.connected))
            self.bridge.close()
            self.bridge.port = "socket://127.0.0.1:%d" % silent.getsockname()[1]
            with self.assertRaises(BridgeError):
                self.bridge.open(timeout=0.6)
            self.assertFalse(self.bridge.connected)
        finally:
            silent.close()


class ChannelTests(BridgeTestCase):
    def test_sticks_and_switches(self):
        self.rx.set_us(1, 2012)
        self.rx.set_us(2, 988)
        self.rx.set_us(3, 1500)
        self.rx.set_us(6, 1500)
        self.rx.set_us(7, 2000)
        self.link_up()
        self.assertTrue(wait_for(lambda: self.bridge.state.channels[0] == 1000))
        state = self.bridge.state
        self.assertFalse(state.failsafe)
        self.assertEqual(state.channels[0], 1000)
        self.assertEqual(state.channels[1], -1000)
        self.assertEqual(state.channels[2], 0)
        self.assertEqual(state.channels[5], 500)
        self.assertEqual(state.channels[6], 1000)
        self.assertEqual(state.ch(1), 1.0)
        self.assertEqual(state.ch(2), -1.0)
        self.assertEqual(state.ch(6), 0.5)
        self.assertEqual(state.ch(99), 0.0)
        self.assertLess(state.age_ms, 50)

    def test_mapping_matches_the_documented_formula(self):
        self.link_up()
        for us in (988, 1000, 1100, 1300, 1479, 1480, 1490, 1500, 1510, 1520, 1521, 1600, 1756,
                   1900, 2012):
            self.rx.set_us(1, us)   # a stick: bipolar with deadband
            self.rx.set_us(8, us)   # a switch channel: unipolar
            seen_us = ticks_to_us(us_to_ticks(us))
            want_stick = expected_bipolar(seen_us)
            want_switch = expected_unipolar(seen_us)
            ok = wait_for(lambda: (self.bridge.state.channels[0], self.bridge.state.channels[7])
                          == (want_stick, want_switch))
            state = self.bridge.state
            self.assertTrue(ok, "%d us: got %d / %d, expected %d / %d" % (
                us, state.channels[0], state.channels[7], want_stick, want_switch))

    def test_raw_channels_and_link_statistics(self):
        self.bridge.set_stream(raw=True)
        self.rx.rssi_dbm = -73
        self.rx.lq = 97
        self.rx.snr_db = -2
        self.rx.set_us(1, 1234)
        self.rx.set_us(16, 1777)
        self.link_up()
        self.assertTrue(wait_for(lambda: self.bridge.raw_channels is not None
                                 and self.bridge.raw_channels[0] != 1500))
        raw = self.bridge.raw_channels
        self.assertEqual(len(raw), 16)
        self.assertEqual(raw[0], ticks_to_us(us_to_ticks(1234)))
        self.assertEqual(raw[15], ticks_to_us(us_to_ticks(1777)))
        self.assertTrue(abs(raw[0] - 1234) <= 1)

        self.assertTrue(wait_for(lambda: self.bridge.state.lq == 97))
        state = self.bridge.state
        self.assertEqual(state.rssi, -73)
        self.assertEqual(state.snr, -2)
        # The rate is counted over one-second windows, so the first window
        # after the link comes up is a partial one; wait for a full one.
        self.assertTrue(wait_for(lambda: self.bridge.link is not None
                                 and 90 <= self.bridge.link.rc_rate_hz <= 110, 4.0),
                        "rc rate %r" % (self.bridge.link,))
        link = self.bridge.link
        self.assertEqual(link.up_rssi1, -73)
        self.assertEqual(link.up_lq, 97)
        self.assertEqual(link.up_snr, -2)
        self.assertEqual(link.tx_power_mw, 100)
        self.assertEqual(link.crsf_errors, 0)

    def test_reversed_and_remapped_output(self):
        self.bridge.set("out1.rev", True)
        self.bridge.set("out2.src", 1)
        self.rx.set_us(1, 2012)
        self.rx.set_us(2, 1500)
        self.link_up()
        self.assertTrue(wait_for(lambda: self.bridge.state.channels[0] == -1000))
        self.assertEqual(self.bridge.state.channels[1], 1000)

    def test_noise_on_the_receiver_line_does_not_corrupt_channels(self):
        self.rx.set_us(1, 2012)
        self.link_up()
        for i in range(50):
            self.rx.send_raw(bytes((i * 37 + k * 11) & 0xFF for k in range(23)))
            time.sleep(0.01)
            state = self.bridge.state
            self.assertTrue(state.link_up)
            self.assertEqual(state.channels[0], 1000)
            self.assertEqual(state.channels[1], 0)


class SafetyTests(BridgeTestCase):
    def test_arm_switch(self):
        self.arm()
        self.rx.set_us(5, 1000)
        self.assertTrue(wait_for(lambda: not self.bridge.state.armed))
        self.assertTrue(self.bridge.state.link_up)

    def test_arming_needs_the_switch_to_start_off(self):
        self.rx.set_us(5, 2000)
        self.link_up()
        time.sleep(1.0)
        self.assertFalse(self.bridge.state.armed)
        self.rx.set_us(5, 1000)
        self.assertTrue(wait_for(lambda: self.bridge.state.channels[4] == 0))
        time.sleep(ARM_OFF_HOLD)
        self.rx.set_us(5, 2000)
        self.assertTrue(wait_for(lambda: self.bridge.state.armed))

    def test_changing_the_arm_channel_does_not_arm(self):
        # CH7 is on for some other reason. Making it the arm switch must not
        # arm the robot there and then: it has to be switched off and on again.
        self.rx.set_us(7, 2000)
        self.link_up()
        time.sleep(ARM_OFF_HOLD)
        self.bridge.set("arm_ch", 7)
        time.sleep(1.0)
        self.assertFalse(self.bridge.state.armed)
        self.rx.set_us(7, 1000)
        time.sleep(ARM_OFF_HOLD)
        self.rx.set_us(7, 2000)
        self.assertTrue(wait_for(lambda: self.bridge.state.armed))

    def test_link_loss_gives_failsafe_within_the_timeout(self):
        self.rx.set_us(1, 2012)
        self.arm()
        self.rx.link = False
        lost_at = time.monotonic()
        self.assertTrue(wait_for(lambda: self.bridge.state.failsafe, 2.0, 0.002))
        elapsed = time.monotonic() - lost_at
        # failsafe_ms is 500; allow for the 100 Hz frame period and the 50 Hz stream.
        self.assertTrue(0.4 <= elapsed <= 0.75, "failsafe after %.3f s" % elapsed)
        state = self.bridge.state
        self.assertFalse(state.link_up)
        self.assertFalse(state.armed)
        self.assertEqual(state.channels[0], 0)
        self.assertEqual(state.lq, 0)

        # The link returns with the arm switch still on: linked, but not armed.
        self.rx.link = True
        self.assertTrue(wait_for(lambda: self.bridge.state.link_up))
        time.sleep(1.0)
        state = self.bridge.state
        self.assertEqual(state.channels[0], 1000)
        self.assertFalse(state.armed)
        self.assertTrue(wait_for(lambda: self.bridge.link is not None
                                 and self.bridge.link.failsafe_count == 1, 2.0))

    def test_disruptive_commands_are_refused_while_armed(self):
        self.arm()
        for command in ("reboot", "defaults", "bind", "save"):
            with self.assertRaises(BridgeError) as caught:
                self.bridge.cli(command)
            self.assertIn("disarm first", str(caught.exception))
        self.assertTrue(self.bridge.state.armed)
        self.assertEqual(self.rx.frames(0x32), [])
        self.assertFalse(os.path.exists(self.settings_path))

    def test_failsafe_values_and_hold(self):
        self.bridge.set("out1.fs", -400)
        self.bridge.set("out2.hold", "on")
        self.bridge.set("failsafe_ms", 200)
        self.rx.set_us(1, 2012)
        self.rx.set_us(2, 2012)
        self.link_up()
        self.assertTrue(wait_for(lambda: self.bridge.state.channels[1] == 1000))
        self.rx.link = False
        self.assertTrue(wait_for(lambda: self.bridge.state.failsafe, 2.0))
        state = self.bridge.state
        self.assertEqual(state.channels[0], -400)
        self.assertEqual(state.channels[1], 1000)
        self.assertEqual(state.channels[2], 0)


class TelemetryTests(BridgeTestCase):
    def wait_frame(self, frame_type):
        self.assertTrue(wait_for(lambda: self.rx.frames(frame_type), 3.0),
                        "receiver never got frame type 0x%02X" % frame_type)
        return self.rx.frames(frame_type)[-1][1]

    def test_battery(self):
        self.link_up()
        self.bridge.send_battery(12.6, 1.5, 1234, 80)
        # 0.1 V, 0.1 A, 24-bit mAh, percent - all big-endian
        self.assertEqual(self.wait_frame(0x08), bytes.fromhex("007E 000F 0004D2 50"))

    def test_gps(self):
        self.link_up()
        self.bridge.send_gps(37.7749, -122.4194, speed=2.5, heading=270.5, alt=12, sats=9)
        payload = self.wait_frame(0x02)
        self.assertEqual(len(payload), 15)
        self.assertEqual(int.from_bytes(payload[0:4], "big", signed=True), 377749000)
        self.assertEqual(int.from_bytes(payload[4:8], "big", signed=True), -1224194000)
        self.assertEqual(int.from_bytes(payload[8:10], "big"), 90)       # 9.0 km/h
        self.assertEqual(int.from_bytes(payload[10:12], "big"), 27050)   # 270.50 deg
        self.assertEqual(int.from_bytes(payload[12:14], "big"), 1012)    # 12 m + 1000
        self.assertEqual(payload[14], 9)

    def test_attitude(self):
        self.link_up()
        self.bridge.send_attitude(roll=0.1, pitch=-0.2, yaw=3.0)
        payload = self.wait_frame(0x1E)
        values = [int.from_bytes(payload[i:i + 2], "big", signed=True) for i in (0, 2, 4)]
        self.assertEqual(values, [-2000, 1000, 30000])  # pitch, roll, yaw in 1/10000 rad

    def test_mode_text(self):
        self.link_up()
        # Without a host value the bridge reports its own arming state.
        self.assertEqual(self.wait_frame(0x21), b"DISARMED\x00")
        self.bridge.send_mode("AUTO")
        self.assertTrue(wait_for(lambda: self.rx.frames(0x21)[-1][1] == b"AUTO\x00"))
        self.bridge.send_mode("")
        self.assertTrue(wait_for(lambda: self.rx.frames(0x21)[-1][1] == b"DISARMED\x00"))

    def test_other_sensors(self):
        self.link_up()
        self.bridge.send_vario(-0.5)
        self.bridge.send_baro(123.4, 0.5)
        self.bridge.send_airspeed(10)
        self.bridge.send_rpm([1000, -200])
        self.bridge.send_temp([25.5, -10], source=1)
        self.assertEqual(self.wait_frame(0x07), bytes.fromhex("FFCE"))
        self.assertEqual(self.wait_frame(0x09), bytes.fromhex("2BE2 0032"))
        self.assertEqual(self.wait_frame(0x0A), bytes.fromhex("0168"))
        self.assertEqual(self.wait_frame(0x0C), bytes.fromhex("00 0003E8 FFFF38"))
        self.assertEqual(self.wait_frame(0x0D), bytes.fromhex("01 00FF FF9C"))

    def test_raw_crsf_frame(self):
        self.link_up()
        self.bridge.send_crsf(0x0E, bytes.fromhex("00 1068 1004"))  # two cell voltages
        self.assertEqual(self.wait_frame(0x0E), bytes.fromhex("00 1068 1004"))

    def test_telemetry_is_paced(self):
        self.link_up()
        for _ in range(30):
            self.bridge.send_battery(12.0)
            self.bridge.send_attitude(0, 0, 0)
            time.sleep(0.005)
        self.rx.clear()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self.bridge.send_battery(12.0)
            self.bridge.send_attitude(0, 0, 0)
            time.sleep(0.005)
        # tlm_period_ms is 100: about ten frames a second however fast the host sends.
        count = len(self.rx.frames())
        self.assertTrue(8 <= count <= 12, "%d telemetry frames in one second" % count)

    def test_nothing_is_sent_without_a_link(self):
        self.bridge.send_battery(12.0)
        time.sleep(0.5)
        self.assertEqual(self.rx.frames(), [])

    def test_bind_command(self):
        self.assertIn("bind", self.bridge.cli("bind"))
        self.assertTrue(wait_for(lambda: self.rx.frames(0x32)))
        # Destination receiver, origin flight controller, "receiver / bind", inner CRC.
        self.assertEqual(self.rx.frames(0x32)[-1][1], bytes.fromhex("EC C8 10 01 9E"))


class SettingsTests(BridgeTestCase):
    def test_get_set(self):
        self.assertEqual(self.bridge.get("stream_hz"), "50")
        self.assertEqual(self.bridge.set("stream_hz", 100), "ok")
        self.assertEqual(self.bridge.get("stream_hz"), "100")
        self.bridge.set("out3.name", "throttle")
        self.assertEqual(self.bridge.names()[2], "throttle")
        self.assertEqual(self.bridge.settings()["arm_ch"], "5")

    def test_errors_raise(self):
        with self.assertRaises(BridgeError):
            self.bridge.set("stream_hz", 9999)
        with self.assertRaises(BridgeError):
            self.bridge.set("no_such_setting", 1)
        with self.assertRaises(BridgeError):
            self.bridge.cli("frobnicate")
        with self.assertRaises(BridgeError):
            self.bridge.set("crsf_rx_pin", 3)  # reserved for USB serial
        self.assertEqual(self.bridge.get("stream_hz"), "50")

    def test_settings_survive_a_reboot(self):
        self.bridge.set("out1.rev", "on")
        self.bridge.set("failsafe_ms", 750)
        self.assertIn("saved", self.bridge.save())
        self.bridge.set("failsafe_ms", 900)  # not saved: must be gone after the reboot
        with open(self.settings_path) as f:
            saved = f.read()
        self.assertIn("out1.rev on\n", saved)
        self.assertIn("failsafe_ms 750\n", saved)

        self.bridge.cli("reboot")
        self.assertTrue(wait_for(lambda: self._value("failsafe_ms") == "750"))
        self.assertEqual(self.bridge.get("out1.rev"), "on")
        self.rx.set_us(1, 2012)
        self.link_up()
        self.assertTrue(wait_for(lambda: self.bridge.state.channels[0] == -1000))

    def _value(self, name):
        try:
            return self.bridge.get(name)
        except BridgeError:
            return None

    def test_backup_and_restore_tools(self):
        backup = os.path.join(self.tempdir.name, "backup.txt")
        self.bridge.set("out2.name", "steer")
        self.bridge.set("stream_hz", 25)
        self.bridge.save()
        self.bridge.close()

        with redirect_stdout(io.StringIO()) as out:
            self.assertEqual(elrs_bridge.main([self.sim.host_url, "backup", backup]), 0)
        self.assertIn("2 changed", out.getvalue())
        with open(backup) as f:
            self.assertEqual(f.read(), "set stream_hz 25\nset out2.name steer\n")

        with redirect_stdout(io.StringIO()):
            self.assertEqual(elrs_bridge.main([self.sim.host_url, "cli", "defaults"]), 0)
            self.assertEqual(elrs_bridge.main([self.sim.host_url, "cli", "save"]), 0)
        with redirect_stdout(io.StringIO()) as out:
            self.assertEqual(elrs_bridge.main([self.sim.host_url, "cli", "get", "stream_hz"]), 0)
        self.assertIn("stream_hz = 50", out.getvalue())

        with redirect_stdout(io.StringIO()):
            self.assertEqual(elrs_bridge.main([self.sim.host_url, "restore", backup]), 0)
        with redirect_stdout(io.StringIO()) as out:
            self.assertEqual(elrs_bridge.main([self.sim.host_url, "cli", "diff"]), 0)
        self.assertEqual(out.getvalue(), "set stream_hz 25\nset out2.name steer\n")

        with redirect_stdout(io.StringIO()) as out:
            self.assertEqual(elrs_bridge.main([self.sim.host_url, "info"]), 0)
        self.assertIn("board sim", out.getvalue())
        self.bridge.open()


class UsbPortTests(BridgeTestCase):
    def setUp(self):
        super().setUp()
        self.term = UsbTerminal(self.sim.usb_port)

    def tearDown(self):
        self.term.close()
        super().tearDown()

    def test_cli_in_a_terminal(self):
        self.term.send("status")
        text = self.term.read_until("> ")
        self.assertIn("status\r\n", text)          # echo
        self.assertIn("link: DOWN", text)
        self.assertIn("usb port: stream off", text)
        self.assertTrue(text.endswith("> "))

        self.term.send("set out1.name steer")
        self.assertIn("\r\nok\r\n> ", self.term.read_until("> "))
        self.term.send("ch")
        self.assertIn("steer", self.term.read_until("> "))
        self.term.send("help")
        self.assertIn("bind", self.term.read_until("> "))

    def test_text_stream_for_hosts_without_a_library(self):
        self.rx.set_us(1, 2012)
        self.rx.set_us(2, 988)
        self.link_up()
        self.term.send("stream text 20")
        self.term.read_until("ok\r\n")
        time.sleep(1.0)
        text = self.term.read_until("\x00", timeout=0.2)
        # The capture can start and stop mid-line; keep only whole lines.
        lines = [line for line in text.split("\r\n")[:-1] if line.startswith("$RC,")]
        self.assertTrue(15 <= len(lines) <= 30, "%d lines in about a second" % len(lines))
        for line in lines:
            body, _, checksum = line[1:].partition("*")
            value = 0
            for char in body:
                value ^= ord(char)
            self.assertEqual("%02X" % value, checksum, line)
        fields = lines[-1][1:].partition("*")[0].split(",")
        self.assertEqual(fields[0], "RC")
        self.assertEqual(int(fields[1]), elrs_bridge.FLAG_LINK_UP)
        self.assertEqual(int(fields[2]), 100)      # link quality
        self.assertEqual(int(fields[3]), -60)      # RSSI
        self.assertEqual([int(v) for v in fields[5:]], [1000, -1000] + [0] * 14)

        # Telemetry can be sent as plain text lines too.
        self.term.send("tlm bat 11.1 2.0")
        self.assertTrue(wait_for(lambda: self.rx.frames(0x08), 3.0))
        self.assertEqual(self.rx.frames(0x08)[-1][1][:4], bytes.fromhex("006F 0014"))

    def test_both_ports_work_at_once(self):
        # A binary client on the USB port alongside the one on the host UART.
        self.term.close()
        with ElrsBridge(self.sim.usb_url, stream_hz=20) as usb:
            self.rx.set_us(1, 2012)
            self.link_up()
            self.assertTrue(wait_for(lambda: usb.state.channels[0] == 1000))
            self.assertEqual(usb.get("stream_hz"), "50")
            self.assertEqual(self.bridge.state.channels[0], 1000)
        self.term = UsbTerminal(self.sim.usb_port)


if __name__ == "__main__":
    if not os.path.exists(SIMULATOR):
        sys.exit("simulator not built - run:  pio run -e sim")
    unittest.main(verbosity=2)
