// The bridge's three serial ports, and the transfer of bytes between them and
// the bridge core.
//
//   USB serial (UART0)  setup CLI; a host computer can use it too
//   receiver   (UART1)  CRSF to and from the ELRS receiver
//   host       (UART2)  the robot's main computer, if there is one
//
// Nothing here blocks: a byte is only written when the UART has room for it,
// so a host that stops listening cannot stall the control loop.
#pragma once

#include <stdint.h>

#include "eb_bridge.h"

class SerialPorts {
 public:
  // Open the ports on the pins and baud rates from the settings.
  void begin(const eb::Config& cfg);

  // Hand everything that has arrived to the bridge.
  void receive(eb::Bridge& bridge, uint32_t nowMs);

  // Send as much of the bridge's pending output as the UARTs will take now.
  void transmit(eb::Bridge& bridge);

  // Wait for the USB port's hardware buffer to empty (used before a restart).
  void flushUsb();

 private:
  // The host UART may be wired in one direction only, or not at all.
  bool hostReceives_ = false;
  bool hostTransmits_ = false;
};
