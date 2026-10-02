#include "board.h"

#include <Arduino.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

namespace {

// Settings are stored as one text blob in the default NVS partition.
const char kNvsNamespace[] = "elrsbridge";
const char kNvsKey[] = "cfg";

const uint32_t kRebootDelayMs = 250;

// How long the main loop may stall before the chip is restarted. A flash
// write can legitimately hold the loop for longer than that (it may have to
// erase a sector first), so the limit is relaxed while settings are saved.
// Saving is refused while armed or while the wheels turn, so that is never
// while the robot is moving.
const uint32_t kWatchdogSeconds = 1;
const uint32_t kWatchdogSecondsWhileSaving = 5;

}  // namespace

const char* Esp32Board::boardName() { return "esp32"; }

const char* Esp32Board::resetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_EXT: return "reset pin";
    case ESP_RST_SW: return "software restart";
    case ESP_RST_PANIC: return "crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout (supply voltage dipped)";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    default: return "unknown";
  }
}

void Esp32Board::boardDefaults(eb::Config& cfg) {
  (void)cfg;  // the generic defaults are the ESP32 DevKit pins
}

bool Esp32Board::pinUsable(int pin, eb::PinUse use) {
  if (pin < 0 || pin >= SOC_GPIO_PIN_COUNT) return false;
  if (!GPIO_IS_VALID_GPIO(pin)) return false;
  if (use == eb::kPinOutput && !GPIO_IS_VALID_OUTPUT_GPIO(pin)) return false;
  if (use == eb::kPinAnalog && digitalPinToAnalogChannel(pin) < 0) return false;
  if (pin == 1 || pin == 3) return false;   // UART0: USB serial / CLI
  if (pin >= 6 && pin <= 11) return false;  // SPI flash
  return true;
}

bool Esp32Board::loadConfigText(char* buf, size_t cap, size_t& len) {
  Preferences prefs;
  if (!prefs.begin(kNvsNamespace, true)) return false;
  const size_t stored = prefs.getBytesLength(kNvsKey);
  bool ok = false;
  if (stored > 0 && stored <= cap) {
    len = prefs.getBytes(kNvsKey, buf, cap);
    ok = len == stored;
  }
  prefs.end();
  return ok;
}

bool Esp32Board::saveConfigText(const char* buf, size_t len) {
  esp_task_wdt_init(kWatchdogSecondsWhileSaving, true);

  Preferences prefs;
  bool ok = false;
  if (prefs.begin(kNvsNamespace, false)) {
    ok = prefs.putBytes(kNvsKey, buf, len) == len;
    prefs.end();
  }

  esp_task_wdt_reset();
  esp_task_wdt_init(kWatchdogSeconds, true);
  return ok;
}

void Esp32Board::requestReboot() {
  rebootRequested_ = true;
  rebootRequestedMs_ = millis();
}

void Esp32Board::startWatchdog() {
  esp_task_wdt_init(kWatchdogSeconds, true);
  esp_task_wdt_add(nullptr);  // the task running setup() and loop()
  // Only that loop is watched. The other core runs the Bluetooth stack, whose
  // timing is not this watchdog's business: if it stalls, the controller's
  // reports stop arriving and the drive treats the controller as gone.
  esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));
}

void Esp32Board::feedWatchdog() { esp_task_wdt_reset(); }

bool Esp32Board::rebootDue(uint32_t nowMs) const {
  // Signed: the request may have been timed a moment after `nowMs` was read,
  // in the same pass of the loop.
  return rebootRequested_ && (int32_t)(nowMs - rebootRequestedMs_) > (int32_t)kRebootDelayMs;
}
