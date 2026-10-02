// Bridge: the text commands. The same code answers a person typing on a
// serial terminal and a program sending kMsgCli frames; only the Out sink
// differs.
//
// Every failure starts with "ERR " so a program can detect it.
#include <stdio.h>
#include <string.h>

#include "eb_bridge.h"

namespace eb {

namespace {

const int kMaxArgs = 12;

// Split a line into whitespace-separated tokens, in place.
int tokenize(char* line, char** argv, int maxArgs) {
  int argc = 0;
  char* p = line;
  while (*p && argc < maxArgs) {
    while (*p == ' ' || *p == '\t') ++p;
    if (!*p) break;
    argv[argc++] = p;
    while (*p && *p != ' ' && *p != '\t') ++p;
    if (*p) *p++ = '\0';
  }
  return argc;
}

// "out1" selects out1.* but not out10.*, "tlm" selects tlm_*: a prefix only
// matches at a word boundary.
bool prefixMatches(const char* name, const char* prefix) {
  const size_t n = strlen(prefix);
  if (n == 0) return true;
  if (strncmp(name, prefix, n) != 0) return false;
  const char last = prefix[n - 1];
  const char next = name[n];
  return next == '\0' || next == '.' || next == '_' || last == '.' || last == '_';
}

const char* streamName(uint8_t mode) {
  switch (mode) {
    case elrsb::kStreamBinary: return "bin";
    case elrsb::kStreamText: return "text";
    default: return "off";
  }
}

struct GetCtx {
  Out* out;
  const char* prefix;
  int shown;
};

void getVisitor(void* opaque, const char* name, const ParamRef& ref) {
  GetCtx& ctx = *(GetCtx*)opaque;
  if (!prefixMatches(name, ctx.prefix)) return;
  char value[32];
  paramFormat(ref, value, sizeof value);
  if (strcmp(name, ctx.prefix) == 0) {
    // An exact match is someone asking about one setting: explain it.
    ctx.out->printf("%s = %s\n  %s\n", name, value, ref.def->help);
    if (ref.def->labels) {
      ctx.out->printf("  values: %s\n", ref.def->labels);
    } else if (ref.def->type != kParamStr) {
      ctx.out->printf("  range: %ld..%ld\n", (long)ref.def->min, (long)ref.def->max);
    }
    if (ref.def->flags & kParamReboot) ctx.out->print("  takes effect after save + reboot\n");
  } else {
    ctx.out->printf("%s = %s\n", name, value);
  }
  ++ctx.shown;
}

struct DumpCtx {
  Out* out;
  const Config* cfg;
  const Config* defaults;
  bool all;
  int shown;
};

void dumpVisitor(void* opaque, const char* name, const ParamRef& ref) {
  DumpCtx& ctx = *(DumpCtx*)opaque;
  char value[32];
  paramFormat(ref, value, sizeof value);
  if (!ctx.all) {
    ParamRef def = ref;
    def.ptr = (char*)ctx.defaults + ((const char*)ref.ptr - (const char*)ctx.cfg);
    char standard[32];
    paramFormat(def, standard, sizeof standard);
    if (strcmp(value, standard) == 0) return;
  }
  ctx.out->printf("set %s %s\n", name, value);
  ++ctx.shown;
}

}  // namespace

void Bridge::runCli(Out& out, char* line, Port& port) {
  char* argv[kMaxArgs];
  const int argc = tokenize(line, argv, kMaxArgs);
  if (argc == 0) return;
  const char* cmd = argv[0];

  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
    cmdHelp(out);
  } else if (strcmp(cmd, "status") == 0) {
    cmdStatus(out, port);
  } else if (strcmp(cmd, "ch") == 0) {
    cmdChannels(out);
  } else if (strcmp(cmd, "get") == 0) {
    cmdGet(out, argc > 1 ? argv[1] : "");
  } else if (strcmp(cmd, "set") == 0) {
    // Accept both "set name value" and "set name=value".
    char* eq = argc == 2 ? strchr(argv[1], '=') : nullptr;
    if (eq) {
      *eq = '\0';
      cmdSet(out, argv[1], eq + 1);
    } else if (argc == 3) {
      cmdSet(out, argv[1], argv[2]);
    } else {
      out.print("ERR usage: set <name> <value>\n");
    }
  } else if (strcmp(cmd, "save") == 0) {
    // Writing flash stalls the whole loop for a moment.
    if (!refusedWhileDriving(out)) cmdSave(out);
  } else if (strcmp(cmd, "defaults") == 0) {
    if (!refusedWhileDriving(out)) cmdDefaults(out);
  } else if (strcmp(cmd, "dump") == 0) {
    cmdDump(out, true);
  } else if (strcmp(cmd, "diff") == 0) {
    cmdDump(out, false);
  } else if (strcmp(cmd, "stream") == 0) {
    cmdStream(out, port, argc, argv);
  } else if (strcmp(cmd, "tlm") == 0) {
    cmdTlm(out, argc, argv);
  } else if (strcmp(cmd, "bind") == 0) {
    // Binding drops the radio link.
    if (!refusedWhileDriving(out)) cmdBind(out);
  } else if (strcmp(cmd, "reboot") == 0) {
    if (!refusedWhileDriving(out)) {
      out.print("rebooting\n");
      plat_.requestReboot();
    }
  } else if (strcmp(cmd, "version") == 0) {
    out.printf("elrs-bridge %u.%u.%u protocol %u board %s\n", kVersionMajor, kVersionMinor,
               kVersionPatch, elrsb::kProtocolVersion, plat_.boardName());
  } else {
    out.printf("ERR unknown command '%s' (try 'help')\n", cmd);
  }
}

// Commands that would take control away for a moment (writing the flash
// stalls everything else on the chip) are refused while the robot could be
// moving: while the arm switch is on, and while the wheels are turning.
// With no arm switch configured "armed" only means "linked", so then only
// turning wheels count.
bool Bridge::refusedWhileDriving(Out& out) {
  if (cfg_.armCh != 0 && rc_.armed()) {
    out.print("ERR disarm first\n");
    return true;
  }
  if (drive_.moving()) {
    out.print("ERR stop the wheels first\n");
    return true;
  }
  return false;
}

void Bridge::cmdHelp(Out& out) {
  out.print(
      "status                      link, arming, telemetry and counters\n"
      "ch                          raw and mapped value of every channel\n"
      "get [name|prefix]           show settings, e.g. 'get out1' or 'get stream_hz'\n"
      "set <name> <value>          change a setting (kept in RAM until 'save')\n"
      "save                        store the settings in flash\n"
      "defaults                    restore the default settings (then 'save')\n"
      "diff                        settings that differ from the defaults\n"
      "dump                        every setting, as 'set' lines\n"
      "stream <off|bin|text> [hz]  what this port sends, until reboot\n"
      "tlm <kind> <values...>      send telemetry to the handset ('tlm' lists kinds)\n"
      "bind                        put the receiver into bind mode\n"
      "reboot                      restart the bridge\n"
      "version\n");
}

void Bridge::cmdStatus(Out& out, Port& port) {
  const uint32_t now = nowMs_;
  out.printf("elrs-bridge %u.%u.%u (%s), up %lu s, last reset: %s\n", kVersionMajor,
             kVersionMinor, kVersionPatch, plat_.boardName(), (unsigned long)(now / 1000),
             plat_.resetReason());
  out.printf("link: %s   armed: %s   failsafes: %u\n", rc_.linkUp() ? "UP" : "DOWN",
             rc_.armed() ? "yes" : "no", (unsigned)rc_.failsafeCount());
  // The usual reason for "it will not arm".
  if (rc_.linkUp() && !rc_.armed() && rc_.armGuarded()) {
    if (cfg_.armCh == 0) {
      out.print("  not armed: with no arm switch, arming starts when the handset next connects\n");
    } else if (rc_.armSwitchOn()) {
      out.print("  not armed: the arm switch is on but has to be off first - switch it off, then on\n");
    }
  }
  statusDrive(out);

  if (rc_.everReceived()) {
    out.printf("rc: %u Hz, last frame %u ms ago, %lu frames\n", (unsigned)rc_.rateHz(),
               (unsigned)rc_.ageMs(now), (unsigned long)rc_.frames());
  } else {
    out.print("rc: no channel frames received yet\n");
  }
  if (haveLink_) {
    // Once the link is down these are the receiver's last report, not live.
    out.printf("%s: LQ %u%%  RSSI %d dBm  SNR %d dB  handset power %u mW  rf mode %u\n",
               rc_.linkUp() ? "uplink" : "last uplink", (unsigned)link_.upLq,
               (int)(link_.antenna ? link_.upRssi2 : link_.upRssi1), (int)link_.upSnr,
               (unsigned)link_.txPowerMw, (unsigned)link_.rfMode);
    // ELRS receivers leave the downlink figures at zero; only show real ones.
    if (link_.downLq || link_.downRssi || link_.downSnr) {
      out.printf("downlink: LQ %u%%  RSSI %d dBm  SNR %d dB\n", (unsigned)link_.downLq,
                 (int)link_.downRssi, (int)link_.downSnr);
    }
  }
  out.printf("crsf: %lu bytes in, %lu frames, %lu rejected\n", (unsigned long)crsfBytes_,
             (unsigned long)crsfParser_.frames(), (unsigned long)crsfParser_.errors());
  if (crsfBytes_ == 0) {
    out.printf("  nothing received on GPIO%d yet. An ELRS receiver is silent until the\n"
               "  handset is on and bound; otherwise check the wiring and receiver power.\n",
               (int)cfg_.crsfRxPin);
  } else if (!haveCrsf_) {
    out.print("  bytes arrive but none form a valid CRSF frame: check crsf_baud and\n"
              "  crsf_invert, and that the receiver's serial protocol is CRSF.\n");
  }

  out.print("telemetry:");
  for (int i = 0; i < Telemetry::kItemCount; ++i) {
    const Telemetry::Item item = (Telemetry::Item)i;
    out.printf(" %s=%c", Telemetry::itemName(item), tlm_.source(item));
  }
  out.printf("  (h=host l=local -=none), %lu frames sent\n", (unsigned long)tlm_.framesSent());

  if (haveLocalVbat_) out.printf("vbat: %.2f V\n", (double)localVbat_);

  static const char* const kPortNames[kNumPorts] = {"usb", "host"};
  for (int i = 0; i < kNumPorts; ++i) {
    const Port& p = ports_[i];
    const unsigned hz = p.streamHz ? p.streamHz : cfg_.streamHz;
    out.printf("%s port: stream %s", kPortNames[i], streamName(p.streamMode));
    if (p.streamMode != elrsb::kStreamOff) out.printf(" at %u Hz", hz);
    out.printf(", %u frames dropped%s\n", (unsigned)p.dropped, &p == &port ? "  (this port)" : "");
  }
  if (cfg_.hostRxPin < 0 && cfg_.hostTxPin < 0) out.print("host port: disabled (no pins set)\n");

  if (loadSkipped_) {
    out.printf("settings: %d saved line(s) were not understood and were ignored\n", loadSkipped_);
  }
  if (dirty_) out.print("settings: changed but not saved\n");
  warnPinConflicts(out);
}

// Who is driving, what the wheels are doing, and whether the motor board and
// the Bluetooth controller are there.
void Bridge::statusDrive(Out& out) {
  const Drive& d = drive_;
  if (cfg_.driveSrc == kDriveOff) {
    out.print("drive: off (drive_src off), wheels stopped\n");
  } else if (d.source() == Drive::kElrs) {
    if (d.live()) {
      out.printf("drive: ELRS handset in charge, wheels live at %d %% speed\n", d.speedPercent());
    } else {
      out.print("drive: ELRS handset in charge, disarmed - wheels stopped\n");
    }
  } else if (d.source() == Drive::kGamepad) {
    if (d.live()) {
      out.printf("drive: Bluetooth controller in charge, wheels live at %d %% speed\n",
                 d.speedPercent());
    } else {
      out.print("drive: Bluetooth controller in charge, centre its sticks to start\n");
    }
  } else {
    out.print("drive: nobody in charge, wheels stopped\n");
  }

  out.printf("  motors M1..M4: %+d %+d %+d %+d", d.command(0), d.command(1), d.command(2),
             d.command(3));
  bool ramping = false;
  for (int i = 0; i < kNumMotors; ++i) ramping |= d.command(i) != d.target(i);
  if (ramping) {
    out.printf("  (heading for %+d %+d %+d %+d)", d.target(0), d.target(1), d.target(2),
               d.target(3));
  }
  out.print("\n");

  if (motorWrites_ == 0) {
    out.print("  motor board: nothing sent to it yet\n");
  } else if (motorBoardOk_) {
    out.printf("  motor board: answering, %lu of %lu commands failed\n",
               (unsigned long)motorFailures_, (unsigned long)motorWrites_);
  } else {
    out.printf("  motor board: NOT ANSWERING at I2C address 0x%02X - check its wiring and power"
               " (%lu of %lu commands failed)\n",
               (unsigned)cfg_.motorAddr, (unsigned long)motorFailures_,
               (unsigned long)motorWrites_);
  }

  if (d.gamepadConnected()) {
    out.print("bluetooth: controller connected\n");
  } else if (!cfg_.btEnable) {
    out.print("bluetooth: off (bt_enable off)\n");
  } else {
    out.print("bluetooth: no controller (to pair a PS4 controller, hold Share and PS)\n");
  }
}

void Bridge::cmdChannels(Out& out) {
  out.printf("link %s, armed %s\n", rc_.linkUp() ? "UP" : "DOWN", rc_.armed() ? "yes" : "no");
  out.print(" #  name         src  raw_us    out\n");
  for (int i = 0; i < kNumOutputs; ++i) {
    const OutputCfg& o = cfg_.out[i];
    if (o.src >= 1 && o.src <= kNumOutputs) {
      out.printf("%2d  %-11s  %3u  %6u  %5d\n", i + 1, o.name, (unsigned)o.src,
                 (unsigned)rc_.rawUs(o.src - 1), (int)rc_.out(i));
    } else {
      out.printf("%2d  %-11s    -       -  %5d\n", i + 1, o.name, (int)rc_.out(i));
    }
  }
}

void Bridge::cmdGet(Out& out, const char* prefix) {
  GetCtx ctx;
  ctx.out = &out;
  ctx.prefix = prefix;
  ctx.shown = 0;
  paramForEach(cfg_, getVisitor, &ctx);
  if (ctx.shown == 0) out.printf("ERR no setting matches '%s'\n", prefix);
}

void Bridge::cmdSet(Out& out, const char* name, const char* value) {
  ParamRef ref;
  if (!paramFind(cfg_, name, ref)) {
    out.printf("ERR unknown setting '%s'\n", name);
    return;
  }
  const char* error = paramSet(ref, value, plat_);
  if (error) {
    if (ref.def->labels) {
      out.printf("ERR %s: %s (%s)\n", name, error, ref.def->labels);
    } else if (ref.def->type != kParamStr) {
      out.printf("ERR %s: %s (%ld..%ld)\n", name, error, (long)ref.def->min, (long)ref.def->max);
    } else {
      out.printf("ERR %s: %s\n", name, error);
    }
    return;
  }
  dirty_ = true;

  // Stream settings act on their port straight away.
  if (ref.ptr == &cfg_.usbStream) {
    setStreamMode(ports_[kPortUsb], cfg_.usbStream);
  } else if (ref.ptr == &cfg_.hostStream) {
    setStreamMode(ports_[kPortHost], cfg_.hostStream);
  } else if (ref.ptr == &cfg_.streamHz) {
    for (int i = 0; i < kNumPorts; ++i) ports_[i].streamHz = 0;
  }

  // Changing what counts as "the arm switch is on" may disarm the robot but
  // must never arm it: that takes the operator, under the new rule.
  // (arm_guard needs nothing here: it only decides what the next link loss
  // will ask for.)
  const bool armingRule = ref.ptr == &cfg_.armCh || ref.ptr == &cfg_.armUs;
  if (armingRule) rc_.requireRearm();

  if (ref.def->flags & kParamReboot) {
    out.print("ok (save and reboot to apply)\n");
  } else if (armingRule) {
    out.print(cfg_.armCh ? "ok (disarmed: switch the arm switch off, then on, to arm)\n"
                         : "ok (disarmed: arms when the handset next connects)\n");
  } else {
    out.print("ok\n");
  }
  if (ref.def->flags & kParamPinMask) warnPinConflicts(out);
}

void Bridge::cmdSave(Out& out) {
  size_t len = 0;
  if (!configToText(cfg_, defaults_, cfgText_, sizeof cfgText_, len)) {
    out.print("ERR settings do not fit in the save buffer\n");
    return;
  }
  if (!plat_.saveConfigText(cfgText_, len)) {
    out.print("ERR could not write the settings to flash\n");
    return;
  }
  dirty_ = false;
  out.printf("saved (%u bytes)\n", (unsigned)len);
}

void Bridge::cmdDefaults(Out& out) {
  cfg_ = defaults_;
  dirty_ = true;
  setStreamMode(ports_[kPortUsb], cfg_.usbStream);
  setStreamMode(ports_[kPortHost], cfg_.hostStream);
  for (int i = 0; i < kNumPorts; ++i) ports_[i].streamHz = 0;
  // The arm switch may now be a different channel: arming starts afresh.
  rc_.requireRearm();
  out.print("defaults restored in RAM - 'save' to keep them, 'reboot' to discard\n");
}

void Bridge::cmdDump(Out& out, bool all) {
  DumpCtx ctx;
  ctx.out = &out;
  ctx.cfg = &cfg_;
  ctx.defaults = &defaults_;
  ctx.all = all;
  ctx.shown = 0;
  paramForEach(cfg_, dumpVisitor, &ctx);
  if (ctx.shown == 0) out.print("# no settings differ from the defaults\n");
}

void Bridge::cmdBind(Out& out) {
  uint8_t frame[crsf::kMaxFrameSize];
  if (sendCrsf(frame, crsf::buildBindCommand(frame))) {
    out.print("bind command sent to the receiver\n");
  } else {
    out.print("ERR receiver port busy\n");
  }
}

void Bridge::cmdStream(Out& out, Port& port, int argc, char** argv) {
  if (argc < 2) {
    const unsigned hz = port.streamHz ? port.streamHz : cfg_.streamHz;
    out.printf("stream %s %u\n", streamName(port.streamMode), hz);
    return;
  }
  uint8_t mode;
  if (strcmp(argv[1], "off") == 0) {
    mode = elrsb::kStreamOff;
  } else if (strcmp(argv[1], "bin") == 0) {
    mode = elrsb::kStreamBinary;
  } else if (strcmp(argv[1], "text") == 0) {
    mode = elrsb::kStreamText;
  } else {
    out.print("ERR usage: stream <off|bin|text> [hz]\n");
    return;
  }
  long hz = 0;
  if (argc > 2 && (!parseLong(argv[2], hz) || hz < 1 || hz > 500)) {
    out.print("ERR rate must be 1..500 Hz\n");
    return;
  }
  out.print("ok\n");
  setStreamMode(port, mode);
  if (argc > 2) port.streamHz = (uint16_t)hz;
}

void Bridge::cmdTlm(Out& out, int argc, char** argv) {
  static const char kUsage[] =
      "tlm bat <volts> [amps] [mah] [percent]\n"
      "tlm gps <lat_deg> <lon_deg> [speed_mps] [heading_deg] [alt_m] [sats]\n"
      "tlm att <roll_rad> <pitch_rad> <yaw_rad>\n"
      "tlm mode <text>             up to 15 characters, '-' to clear\n"
      "tlm vario <mps>\n"
      "tlm alt <metres> [vspeed_mps]\n"
      "tlm aspd <mps>\n"
      "tlm rpm <v1> [v2 ...]       up to 8 values\n"
      "tlm temp <c1> [c2 ...]      up to 8 values, degrees C\n";
  if (argc < 3) {
    out.print(argc < 2 ? kUsage : "ERR missing value - 'tlm' lists the forms\n");
    return;
  }
  const char* kind = argv[1];
  const int nvals = argc - 2;
  char** vals = argv + 2;
  const uint32_t now = nowMs_;

  // Telemetry is sent over and over, so success is silent.
  if (strcmp(kind, "mode") == 0) {
    char text[elrsb::kMaxModeText + 1];
    size_t n = 0;
    for (int i = 0; i < nvals; ++i) {
      if (i > 0 && n < sizeof text - 1) text[n++] = ' ';
      for (const char* p = vals[i]; *p && n < sizeof text - 1; ++p) text[n++] = *p;
    }
    text[n] = '\0';
    if (strcmp(text, "-") == 0) n = 0;
    tlm_.setMode(text, n, now);
    return;
  }

  if (strcmp(kind, "gps") == 0) {
    double lat = 0;
    double lon = 0;
    float rest[4] = {0, 0, 0, 0};  // speed, heading, altitude, satellites
    if (nvals < 2 || nvals > 6 || !parseDouble(vals[0], lat) || !parseDouble(vals[1], lon) ||
        lat < -90 || lat > 90 || lon < -180 || lon > 180) {
      out.print("ERR usage: tlm gps <lat_deg> <lon_deg> [speed_mps] [heading_deg] [alt_m] [sats]\n");
      return;
    }
    for (int i = 2; i < nvals; ++i) {
      if (!parseFloat(vals[i], rest[i - 2])) {
        out.printf("ERR '%s' is not a number\n", vals[i]);
        return;
      }
    }
    elrsb::Gps g;
    g.latE7 = (int32_t)(lat * 1e7 + (lat >= 0 ? 0.5 : -0.5));
    g.lonE7 = (int32_t)(lon * 1e7 + (lon >= 0 ? 0.5 : -0.5));
    g.speedMps = rest[0];
    g.headingDeg = rest[1];
    g.altM = rest[2];
    g.sats = (uint8_t)clampT<float>(rest[3], 0.0f, 255.0f);
    tlm_.setGps(g, now);
    return;
  }

  float v[elrsb::kMaxRpmValues] = {0};
  if (nvals > (int)elrsb::kMaxRpmValues) {
    out.print("ERR too many values\n");
    return;
  }
  for (int i = 0; i < nvals; ++i) {
    if (!parseFloat(vals[i], v[i])) {
      out.printf("ERR '%s' is not a number\n", vals[i]);
      return;
    }
  }

  if (strcmp(kind, "bat") == 0 && nvals <= 4) {
    elrsb::Battery b;
    b.volts = v[0];
    b.amps = v[1];
    b.usedMah = v[2];
    b.remainingPct = (uint8_t)clampT<float>(v[3], 0.0f, 100.0f);
    tlm_.setBattery(b, now);
  } else if (strcmp(kind, "att") == 0 && nvals == 3) {
    elrsb::Attitude a;
    a.roll = v[0];
    a.pitch = v[1];
    a.yaw = v[2];
    tlm_.setAttitude(a, now);
  } else if (strcmp(kind, "vario") == 0 && nvals == 1) {
    tlm_.setVario(v[0], now);
  } else if (strcmp(kind, "alt") == 0 && nvals <= 2) {
    elrsb::Baro b;
    b.altM = v[0];
    b.vspeedMps = v[1];
    tlm_.setBaro(b, now);
  } else if (strcmp(kind, "aspd") == 0 && nvals == 1) {
    tlm_.setAirspeed(v[0], now);
  } else if (strcmp(kind, "rpm") == 0) {
    elrsb::Rpm r;
    r.source = 0;
    r.count = (uint8_t)nvals;
    for (int i = 0; i < nvals; ++i) r.rpm[i] = (int32_t)clampT<float>(v[i], -8388608.0f, 8388607.0f);
    tlm_.setRpm(r, now);
  } else if (strcmp(kind, "temp") == 0) {
    elrsb::Temp t;
    t.source = 0;
    t.count = (uint8_t)nvals;
    for (int i = 0; i < nvals; ++i) t.degC[i] = v[i];
    tlm_.setTemp(t, now);
  } else {
    out.print("ERR unknown telemetry form - 'tlm' lists them\n");
  }
}

void Bridge::warnPinConflicts(Out& out) {
  struct Use {
    int pin;
    const char* name;
  };
  char auxNames[kNumAux][12];
  Use uses[8 + kNumAux];
  int count = 0;
  uses[count].pin = cfg_.crsfRxPin;
  uses[count++].name = "crsf_rx_pin";
  uses[count].pin = cfg_.crsfTxPin;
  uses[count++].name = "crsf_tx_pin";
  uses[count].pin = cfg_.hostRxPin;
  uses[count++].name = "host_rx_pin";
  uses[count].pin = cfg_.hostTxPin;
  uses[count++].name = "host_tx_pin";
  uses[count].pin = cfg_.vbatPin;
  uses[count++].name = "vbat_pin";
  uses[count].pin = cfg_.ledPin;
  uses[count++].name = "led_pin";
  uses[count].pin = cfg_.motorSdaPin;
  uses[count++].name = "motor_sda";
  uses[count].pin = cfg_.motorSclPin;
  uses[count++].name = "motor_scl";
  for (int i = 0; i < kNumAux; ++i) {
    snprintf(auxNames[i], sizeof auxNames[i], "aux%d.pin", i + 1);
    uses[count].pin = cfg_.aux[i].pin;
    uses[count++].name = auxNames[i];
  }
  for (int i = 0; i < count; ++i) {
    if (uses[i].pin < 0) continue;
    for (int j = i + 1; j < count; ++j) {
      if (uses[j].pin == uses[i].pin) {
        out.printf("WARNING GPIO%d is assigned to both %s and %s\n", uses[i].pin, uses[i].name,
                   uses[j].name);
      }
    }
  }
}

}  // namespace eb
