#include "hiwonder_board.h"

#include <Arduino.h>
#include <Wire.h>
#include <esp_rom_sys.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>

#include "output_guard.h"

namespace {

// The board's registers, as the robot's original firmware used them.
const uint8_t kRegMotorType = 0x14;
const uint8_t kRegEncoderPolarity = 0x15;
const uint8_t kRegFixedSpeed = 0x33;  // four signed bytes: closed-loop speed for M1..M4

const uint32_t kClockHz = 100000;
// A board that stops answering must not hold up the main loop for long.
const uint16_t kTimeoutMs = 5;

// Where emergencyStop() finds the board. In RAM, not flash, for the crash
// handler.
volatile int8_t stopSda = -1;
volatile int8_t stopScl = -1;
volatile uint8_t stopAddress = 0;

// The I2C lines driven by hand. They are open drain: "high" means let go and
// let the pull-up raise the line.
void IRAM_ATTR lineLow(int pin) {
  if (pin < 32) {
    GPIO.out_w1tc = 1u << pin;
  } else {
    GPIO.out1_w1tc.val = 1u << (pin - 32);
  }
}

void IRAM_ATTR lineHigh(int pin) {
  if (pin < 32) {
    GPIO.out_w1ts = 1u << pin;
  } else {
    GPIO.out1_w1ts.val = 1u << (pin - 32);
  }
}

bool IRAM_ATTR lineIsHigh(int pin) {
  return pin < 32 ? ((GPIO.in >> pin) & 1u) != 0 : ((GPIO.in1.val >> (pin - 32)) & 1u) != 0;
}

// Take a line away from the I2C controller and drive it from the GPIO
// output register instead, open drain, starting released.
void IRAM_ATTR takeLine(int pin) {
  lineHigh(pin);
  GPIO.pin[pin].pad_driver = 1;
  GPIO.func_out_sel_cfg[pin].val = SIG_GPIO_OUT_IDX;
  if (pin < 32) {
    GPIO.enable_w1ts = 1u << pin;
  } else {
    GPIO.enable1_w1ts.val = 1u << (pin - 32);
  }
}

// Half a bit time at about 100 kHz.
void IRAM_ATTR halfBit() { esp_rom_delay_us(5); }

// Let SCL go high. A device may hold it low while it is busy (clock
// stretching), so wait for it - but not for ever: a stuck bus must not hang a
// crash handler.
void IRAM_ATTR releaseClock(int scl) {
  lineHigh(scl);
  for (int i = 0; i < 100 && !lineIsHigh(scl); ++i) esp_rom_delay_us(5);
  halfBit();
}

// Clock one byte out, most significant bit first, then clock the
// acknowledge bit (which nothing here could act on, so it is not read).
void IRAM_ATTR sendByte(int sda, int scl, uint8_t value) {
  for (int bit = 7; bit >= 0; --bit) {
    if (value & (1u << bit)) {
      lineHigh(sda);
    } else {
      lineLow(sda);
    }
    halfBit();
    releaseClock(scl);
    lineLow(scl);
  }
  lineHigh(sda);
  halfBit();
  releaseClock(scl);
  lineLow(scl);
}

}  // namespace

bool HiwonderBoard::begin(int sdaPin, int sclPin, uint8_t address, uint8_t motorType,
                          uint8_t encoderPolarity) {
  address_ = address;
  if (!Wire.begin(sdaPin, sclPin, kClockHz)) return false;
  Wire.setTimeOut(kTimeoutMs);
  started_ = true;

  stopAddress = address;
  stopSda = (int8_t)sdaPin;
  stopScl = (int8_t)sclPin;
  output_guard::addStopAction(&HiwonderBoard::emergencyStop);

  // Stop first: after a restart the board may still be turning the motors
  // at the last speed it was given.
  const int8_t stop[4] = {0, 0, 0, 0};
  bool ok = setSpeeds(stop);
  ok = writeRegister(kRegMotorType, &motorType, 1) && ok;
  delay(10);  // the original firmware left this gap between the two settings
  ok = writeRegister(kRegEncoderPolarity, &encoderPolarity, 1) && ok;
  return ok;
}

bool HiwonderBoard::setSpeeds(const int8_t (&speeds)[4]) {
  return writeRegister(kRegFixedSpeed, (const uint8_t*)speeds, 4);
}

bool HiwonderBoard::writeRegister(uint8_t reg, const uint8_t* data, uint8_t len) {
  if (!started_) return false;
  Wire.beginTransmission(address_);
  Wire.write(reg);
  Wire.write(data, len);
  return Wire.endTransmission() == 0;
}

// The same message setSpeeds() sends with four zeros, but with the lines
// driven by hand, so it works with the I2C driver in any state - even
// half-way through a transfer when the chip crashed. About 0.7 ms.
void IRAM_ATTR HiwonderBoard::emergencyStop() {
  const int sda = stopSda;
  const int scl = stopScl;
  if (sda < 0 || scl < 0) return;
  takeLine(sda);
  takeLine(scl);

  // Free the bus. A device caught part-way through a byte may be holding SDA
  // low; nine clocks let it finish, and a STOP then resets everything on it.
  for (int i = 0; i < 9; ++i) {
    lineLow(scl);
    halfBit();
    releaseClock(scl);
  }
  lineLow(scl);
  lineLow(sda);
  halfBit();
  releaseClock(scl);
  lineHigh(sda);
  halfBit();

  // START, the address, the speed register, four zero speeds, STOP.
  lineLow(sda);
  halfBit();
  lineLow(scl);
  halfBit();
  sendByte(sda, scl, (uint8_t)(stopAddress << 1));
  sendByte(sda, scl, kRegFixedSpeed);
  for (int i = 0; i < 4; ++i) sendByte(sda, scl, 0);
  lineLow(sda);
  halfBit();
  releaseClock(scl);
  lineHigh(sda);
  halfBit();
}
