// Minimal stand-in for the Arduino core so the ElrsBridgeSerial wrapper can be
// compiled and tested on the desktop.
#pragma once

#include <stddef.h>
#include <stdint.h>

unsigned long millis();
