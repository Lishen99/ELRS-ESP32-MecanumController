"""Stands in for the ExpressLRS receiver when running the desktop simulator.

Connects to the simulator's CRSF port and behaves like a receiver with a
handset linked to it: it sends RC channel frames and link statistics, and
records the telemetry frames the bridge sends back. With it you can develop
and test host code with no radio hardware at all:

    pio run -e sim
    .pio/build/sim/program                     # terminal 1: the bridge
    python host/python/fake_receiver.py sweep  # terminal 2: moving sticks
    python host/python/elrs_bridge.py socket://localhost:5761 monitor

The CRSF encoding here is written independently of the firmware's, so the
automated tests that use it also cross-check the firmware's codec.
"""

import argparse
import math
import socket
import threading
import time

SYNC = 0xC8
TYPE_LINK_STATS = 0x14
TYPE_RC_CHANNELS = 0x16


def crc8(data, poly=0xD5):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(frame_type, payload):
    """A complete CRSF frame: sync, length, type, payload, CRC."""
    body = bytes((frame_type,)) + bytes(payload)
    return bytes((SYNC, len(body) + 1)) + body + bytes((crc8(body),))


def us_to_ticks(us):
    """Pulse width in microseconds to an 11-bit CRSF channel value."""
    return max(0, min(2047, int(round((us - 1500) * 8 / 5.0 + 992))))


def pack_channels(ticks):
    """16 channels x 11 bits, least significant bit first, 22 bytes."""
    value = 0
    for i, t in enumerate(ticks):
        value |= (t & 0x7FF) << (11 * i)
    return value.to_bytes(22, "little")


def parse_frames(buffer):
    """Pull complete CRSF frames off the front of `buffer` (a bytearray).
    Returns a list of (type, payload)."""
    frames = []
    while True:
        while buffer and buffer[0] != SYNC:
            del buffer[0]
        if len(buffer) < 2:
            break
        length = buffer[1]
        if length < 2 or length > 62:
            del buffer[0]
            continue
        if len(buffer) < length + 2:
            break
        body = bytes(buffer[2:length + 1])
        if crc8(body) == buffer[length + 1]:
            frames.append((body[0], body[1:]))
            del buffer[:length + 2]
        else:
            del buffer[0]
    return frames


class FakeReceiver:
    """A scriptable receiver. Channel values are pulse widths in microseconds;
    channels are numbered from 1 like on the handset."""

    def __init__(self, host="127.0.0.1", port=5760, rate_hz=100):
        self.host = host
        self.port = port
        self.rate_hz = rate_hz
        self.us = [1500] * 4 + [1000] * 12
        self.link = True          # False = handset switched off: no frames at all
        self.lq = 100
        self.rssi_dbm = -60
        self.snr_db = 10
        self.power_index = 3      # 100 mW
        self.received = []        # (type, payload) frames sent by the bridge
        self._lock = threading.Lock()
        self._sock = None
        self._thread = None
        self._running = False

    def start(self):
        self._sock = socket.create_connection((self.host, self.port), timeout=5)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(0)
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1)
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    def set_us(self, channel, us):
        with self._lock:
            self.us[channel - 1] = us

    def frames(self, frame_type=None):
        with self._lock:
            return [f for f in self.received if frame_type is None or f[0] == frame_type]

    def clear(self):
        with self._lock:
            del self.received[:]

    def send_raw(self, data):
        """Inject arbitrary bytes, e.g. line noise."""
        self._sock.sendall(data)

    def _run(self):
        period = 1.0 / self.rate_hz
        next_rc = time.monotonic()
        next_stats = next_rc
        incoming = bytearray()
        while self._running:
            now = time.monotonic()
            if now >= next_rc:
                next_rc += period
                if now - next_rc > 1.0:
                    next_rc = now + period
                if self.link:
                    with self._lock:
                        ticks = [us_to_ticks(us) for us in self.us]
                    self._send(frame(TYPE_RC_CHANNELS, pack_channels(ticks)))
            if now >= next_stats:
                next_stats = now + 0.1
                if self.link:
                    stats = bytes((min(255, -self.rssi_dbm), 0, self.lq, self.snr_db & 0xFF, 0, 7,
                                   self.power_index, 0, 0, 0))
                    self._send(frame(TYPE_LINK_STATS, stats))
            try:
                data = self._sock.recv(4096)
                if data:
                    incoming.extend(data)
                    found = parse_frames(incoming)
                    if found:
                        with self._lock:
                            self.received.extend(found)
            except (BlockingIOError, InterruptedError):
                pass
            except OSError:
                break
            time.sleep(0.001)

    def _send(self, data):
        try:
            self._sock.sendall(data)
        except OSError:
            self._running = False


def main():
    parser = argparse.ArgumentParser(description="Fake ELRS receiver for the bridge simulator")
    parser.add_argument("pattern", nargs="?", default="sweep", choices=["centre", "sweep"],
                        help="centre: sticks at rest; sweep: sticks move slowly (default)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5760, help="simulator CRSF port")
    parser.add_argument("--rate", type=int, default=100, help="RC frames per second")
    parser.add_argument("--arm", action="store_true", help="flip the arm switch (CH5) on after 2 s")
    args = parser.parse_args()

    rx = FakeReceiver(args.host, args.port, args.rate).start()
    print("sending RC frames to %s:%d at %d Hz - Ctrl-C to stop" % (args.host, args.port, args.rate))
    start = time.monotonic()
    shown = set()
    try:
        while True:
            t = time.monotonic() - start
            if args.pattern == "sweep":
                for ch in range(1, 5):
                    rx.set_us(ch, 1500 + 512 * math.sin(t * 0.8 + ch * 1.3))
            if args.arm and t > 2.0:
                rx.set_us(5, 2000)
            for ftype, payload in rx.frames():
                if ftype not in shown:
                    shown.add(ftype)
                    print("bridge is sending telemetry frame type 0x%02X (%d payload bytes)" % (
                        ftype, len(payload)))
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop()


if __name__ == "__main__":
    main()
