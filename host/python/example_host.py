"""What a host computer (a Jetson, say) does alongside the robot.

The ESP32 drives the wheels itself, from the ELRS handset or the Bluetooth
controller. The host sees everything the handset sends and who is driving,
and sends telemetry back for the handset's screen.

    python example_host.py /dev/ttyUSB0        # Jetson / Linux, ESP32 on USB
    python example_host.py /dev/ttyTHS1        # Jetson header UART wired to the ESP32
    python example_host.py COM5                # Windows
    python example_host.py socket://localhost:5761   # the desktop simulator

Replace read_battery() with your robot's own measurement.
"""

import sys
import time

from elrs_bridge import BridgeError, ElrsBridge


def read_battery():
    """Return (volts, amps, used_mah, remaining_percent)."""
    return 12.4, 1.8, 350, 80


def main(port):
    with ElrsBridge(port) as bridge:
        info = bridge.info()
        print("connected to elrs-bridge %s on %s" % (info.version, info.board))

        last_telemetry = 0.0
        while True:
            # Blocks until the next update (50 a second by default). If the
            # ESP32 stops talking this returns a failsafe state instead of old
            # data, so the flags are always current.
            state = bridge.wait_state()

            if state.motors_live:
                driver = "Bluetooth controller" if state.gamepad else "ELRS handset"
                status = "wheels live (%s)" % driver
            else:
                status = "wheels stopped"
            print("\r%-40s  CH1 %+.2f  CH2 %+.2f  LQ %3d%%" % (
                status, state.ch(1), state.ch(2), state.lq), end="", flush=True)

            # Telemetry for the handset, twice a second.
            now = time.monotonic()
            if now - last_telemetry >= 0.5:
                last_telemetry = now
                volts, amps, used_mah, percent = read_battery()
                bridge.send_battery(volts, amps, used_mah, percent)
                bridge.send_mode("DRIVING" if state.motors_live else "IDLE")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    try:
        main(sys.argv[1])
    except BridgeError as exc:
        sys.exit("error: %s" % exc)
    except KeyboardInterrupt:
        print()
