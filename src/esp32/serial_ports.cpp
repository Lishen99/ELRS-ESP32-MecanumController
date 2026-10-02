#include "serial_ports.h"

#include <Arduino.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_pins.h>
#include <soc/gpio_sig_map.h>

namespace {

HardwareSerial CrsfSerial(1);
HardwareSerial HostSerial(2);

// Receive buffers. The receiver sends up to about 26 kB/s at the fastest
// packet rates; its buffer is sized to absorb a stall in the main loop. (It
// cannot help during a flash write: the UART interrupt is held off then, and
// anything beyond the 128-byte hardware FIFO is lost. The bridge simply
// resynchronises on the next frame.)
const size_t kCrsfRxBuffer = 2048;
const size_t kHostRxBuffer = 1024;
const size_t kChunk = 128;

// Read the next chunk of the `left` bytes a UART had waiting. Returns the
// number read, or 0 when there is nothing more.
size_t nextChunk(HardwareSerial& serial, int& left, uint8_t* buf, size_t cap) {
  if (left <= 0) return 0;
  const size_t n = serial.read(buf, min((size_t)left, cap));
  left = n ? left - (int)n : 0;
  return n;
}

// How many of `pending` bytes the UART can take right now without blocking.
// With no transmit ring buffer configured, a write that fits the hardware
// FIFO returns immediately.
size_t roomFor(HardwareSerial& serial, size_t pending, size_t cap) {
  if (pending == 0) return 0;
  const int room = serial.availableForWrite();
  if (room <= 0) return 0;
  return min(min((size_t)room, pending), cap);
}

}  // namespace

void SerialPorts::begin(const eb::Config& cfg) {
  // Buffer sizes must be set before begin().
  Serial.setRxBufferSize(kHostRxBuffer);
  Serial.begin(cfg.usbBaud);

  CrsfSerial.setRxBufferSize(kCrsfRxBuffer);
  CrsfSerial.begin(cfg.crsfBaud, SERIAL_8N1, cfg.crsfRxPin, cfg.crsfTxPin, cfg.crsfInvert != 0);

  hostReceives_ = cfg.hostRxPin >= 0;
  hostTransmits_ = cfg.hostTxPin >= 0;
  if (hostReceives_ || hostTransmits_) {
    HostSerial.setRxBufferSize(kHostRxBuffer);
    HostSerial.begin(cfg.hostBaud, SERIAL_8N1, cfg.hostRxPin, cfg.hostTxPin);
    if (!hostReceives_) {
      // With no receive pin, the UART's input is not necessarily quiet: by
      // default it is routed from a fixed pad (GPIO16 on a classic ESP32 -
      // the receiver's pin). Tie it to the idle level so it hears nothing.
      esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, U2RXD_IN_IDX, false);
    }
  }
}

void SerialPorts::receive(eb::Bridge& bridge, uint32_t nowMs) {
  uint8_t buf[kChunk];
  size_t n;

  int left = CrsfSerial.available();
  while ((n = nextChunk(CrsfSerial, left, buf, sizeof buf)) > 0) {
    bridge.feedCrsf(buf, n, nowMs);
  }

  left = Serial.available();
  while ((n = nextChunk(Serial, left, buf, sizeof buf)) > 0) {
    bridge.feedPort(eb::Bridge::kPortUsb, buf, n, nowMs);
  }

  // A UART with no receive pin assigned must not be read: its input is then
  // whatever the chip routes there by default, not the host.
  if (hostReceives_) {
    left = HostSerial.available();
    while ((n = nextChunk(HostSerial, left, buf, sizeof buf)) > 0) {
      bridge.feedPort(eb::Bridge::kPortHost, buf, n, nowMs);
    }
  }
}

void SerialPorts::transmit(eb::Bridge& bridge) {
  uint8_t buf[kChunk];

  // Only as much as each UART can take right now is fetched. What stays in
  // the bridge is kept current there instead of going stale in a buffer.
  size_t n = roomFor(CrsfSerial, bridge.crsfTxPending(), sizeof buf);
  if (n) CrsfSerial.write(buf, bridge.readCrsfTx(buf, n));

  n = roomFor(Serial, bridge.portTxPending(eb::Bridge::kPortUsb), sizeof buf);
  if (n) Serial.write(buf, bridge.readPortTx(eb::Bridge::kPortUsb, buf, n));

  if (hostTransmits_) {
    n = roomFor(HostSerial, bridge.portTxPending(eb::Bridge::kPortHost), sizeof buf);
    if (n) HostSerial.write(buf, bridge.readPortTx(eb::Bridge::kPortHost, buf, n));
  } else {
    // No host transmit pin: discard the output rather than let it queue.
    while (bridge.readPortTx(eb::Bridge::kPortHost, buf, sizeof buf) > 0) {
    }
  }
}

void SerialPorts::flushUsb() { Serial.flush(); }
