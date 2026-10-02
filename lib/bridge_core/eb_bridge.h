// The bridge: CRSF from the receiver and reports from a Bluetooth controller
// on one side; the wheels, the host protocol and the CLI on the other; and the
// control state (link, arming, mapped channels, who is driving) in between.
//
// Pure logic over byte streams and values. The platform layer moves bytes
// between this class and the real UARTs, hands over the Bluetooth
// controller's reports, and reads the control state to drive hardware.
//
// The implementation is split by job:
//   eb_bridge.cpp       start-up, the receiver side, periodic work
//   eb_host_ports.cpp   the host protocol and the two host-facing ports
//   eb_cli.cpp          the text commands
//   eb_aux.cpp          what the aux pins should be doing
// and the driving itself is its own class, in eb_drive.cpp.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <elrs_bridge_protocol.h>

#include "eb_config.h"
#include "eb_crsf.h"
#include "eb_drive.h"
#include "eb_gamepad.h"
#include "eb_platform.h"
#include "eb_rc.h"
#include "eb_telemetry.h"
#include "eb_util.h"

namespace eb {

const uint8_t kVersionMajor = 0;
const uint8_t kVersionMinor = 1;
const uint8_t kVersionPatch = 0;

const size_t kPortTxSize = 8192;      // queued replies per host port; holds a full `dump`
const size_t kStreamChunkMax = 160;   // one periodic update: state + raw + link frames, or a text line
const size_t kCrsfTxSize = 512;
const size_t kLineMax = 128;

// What one aux pin should be doing right now.
struct AuxCommand {
  enum Kind { kDisabled, kPulse, kLevel };
  uint8_t kind;
  int8_t pin;        // the GPIO this output was started on at boot
  uint16_t pulseUs;  // kPulse: servo pulse width, 0 = no pulses
  bool level;        // kLevel: pin state
};

class Bridge {
 public:
  enum PortId { kPortUsb = 0, kPortHost = 1, kNumPorts = 2 };

  explicit Bridge(Platform& platform);

  // Load the saved settings. Call once before using config().
  void begin(uint32_t nowMs);

  // Bytes received from the receiver / from a host port.
  void feedCrsf(const uint8_t* data, size_t len, uint32_t nowMs);
  void feedPort(PortId id, const uint8_t* data, size_t len, uint32_t nowMs);

  // The latest report from the Bluetooth controller. Call whenever one
  // arrives, and once with `connected` false when it disconnects.
  void setGamepad(const GamepadInput& pad, uint32_t nowMs);

  // Run timers: failsafe, driving, telemetry schedule, stream output. Call
  // every loop.
  void tick(uint32_t nowMs);

  // Bytes waiting to be transmitted to the receiver / to a host port. Take
  // only as many as the UART can accept right now: what is left behind is
  // kept current rather than allowed to go stale in a buffer.
  size_t crsfTxPending() const { return crsfTx_.size(); }
  size_t readCrsfTx(uint8_t* out, size_t max) { return crsfTx_.pop(out, max); }
  size_t portTxPending(PortId id) const;
  size_t readPortTx(PortId id, uint8_t* out, size_t max);

  // Control state, for whatever drives the hardware.
  const Config& config() const { return cfg_; }
  bool linkUp() const { return rc_.linkUp(); }
  bool armed() const { return rc_.armed(); }
  const Rc& rc() const { return rc_; }
  const Drive& drive() const { return drive_; }

  // New speed commands for the motor board's four channels, when there are
  // any: every 10 ms, and at once when the robot has to stop. Returns false
  // when nothing has changed since the last call.
  bool takeMotorCommands(int8_t (&out)[kNumMotors]);

  // Whether the motor board accepted the commands last sent to it.
  void reportMotorBoard(bool ok);

  // Desired state of aux pin `index` (0-based).
  AuxCommand aux(int index) const;

  // Battery voltage measured by the platform's ADC.
  void setLocalVbat(float volts, uint32_t nowMs);

 private:
  // One host-facing serial port.
  //
  // Output has two lanes. Replies (CLI text, acknowledgements, one-shot
  // answers) wait in a queue. The periodic update does not: there is a single
  // slot holding the newest one, and it is slipped in between queued items.
  // So however slow the port is, the host is never handed a backlog of old
  // states - it gets the latest one, as often as the line allows.
  struct Port {
    Port();

    // replies
    uint8_t txBuf[kPortTxSize];
    ByteQueue tx;

    // the newest periodic update not yet sent
    uint8_t streamBuf[kStreamChunkMax];
    uint8_t streamLen;     // bytes waiting in streamBuf, 0 = none
    uint8_t streamPos;     // how many of them have been handed out

    // where the reply byte stream stands, so an update is only inserted
    // where it cannot split a frame or a line of text
    uint8_t txState;       // TxState
    uint16_t txRemaining;  // bytes left in the frame being sent
    bool txLineStart;      // the last reply text sent ended a line
    bool lastWasStream;    // the last complete item sent was a periodic update

    // input
    elrsb::FrameParser parser;
    char line[kLineMax];
    uint8_t lineLen;
    bool lineOverflow;
    bool lastWasCr;
    uint32_t lastRxMs;

    // stream setup
    uint8_t streamMode;    // elrsb::StreamMode in effect
    uint16_t streamHz;     // 0 = follow stream_hz
    uint8_t streamOpts;    // elrsb::StreamOptions
    uint8_t seq;
    uint32_t lastStreamMs;
    uint32_t lastLinkMs;
    uint16_t dropped;
  };

  // The aux settings that only take effect at boot, as they were at boot.
  struct AuxBoot {
    int8_t pin;
    uint8_t mode;
  };

  // eb_bridge.cpp
  void onCrsfFrame(const crsf::Frame& frame, uint32_t nowMs);
  bool sendCrsf(const uint8_t* frame, size_t len);

  // eb_host_ports.cpp
  void banner(Port& port);
  void prompt(Port& port);
  void setStreamMode(Port& port, uint8_t mode);
  void streamPort(Port& port, uint32_t nowMs);
  void offerStream(Port& port, const uint8_t* chunk, size_t len);
  bool streamTurn(const Port& port) const;
  uint8_t flags() const;
  void fillState(elrsb::State& state) const;
  size_t buildState(Port& port, uint8_t* frame);
  size_t buildTextState(char* line, size_t cap);
  size_t buildLink(const Port& port, uint8_t* frame);
  size_t buildRawChannels(uint8_t* frame);
  bool queueBytes(Port& port, const uint8_t* data, size_t len);
  bool sendFrame(Port& port, uint8_t type, const uint8_t* payload, size_t len);
  void sendAck(Port& port, uint8_t type, uint8_t status);
  void sendInfo(Port& port);
  void handleFrame(Port& port, uint8_t type, const uint8_t* payload, size_t len);
  void handleTextByte(Port& port, uint8_t byte);

  // eb_cli.cpp
  void runCli(Out& out, char* line, Port& port);
  bool refusedWhileDriving(Out& out);
  void cmdHelp(Out& out);
  void cmdStatus(Out& out, Port& port);
  void cmdChannels(Out& out);
  void cmdGet(Out& out, const char* prefix);
  void cmdSet(Out& out, const char* name, const char* value);
  void cmdSave(Out& out);
  void cmdDefaults(Out& out);
  void cmdDump(Out& out, bool all);
  void cmdBind(Out& out);
  void cmdStream(Out& out, Port& port, int argc, char** argv);
  void cmdTlm(Out& out, int argc, char** argv);
  void statusDrive(Out& out);
  void warnPinConflicts(Out& out);

  Platform& plat_;
  Config cfg_;
  Config defaults_;
  AuxBoot auxBoot_[kNumAux];
  bool dirty_;          // settings changed since they were last loaded or saved
  int loadSkipped_;     // saved lines that could not be applied at boot

  crsf::Parser crsfParser_;
  crsf::LinkStats link_;
  bool haveLink_;       // link statistics have been received at least once
  bool haveCrsf_;       // at least one valid CRSF frame has been received
  uint32_t crsfBytes_;
  Rc rc_;
  Drive drive_;
  bool motorCommandsDue_;
  uint32_t motorWrites_;     // commands sent to the motor board
  uint32_t motorFailures_;   // ... and not acknowledged
  bool motorBoardOk_;        // the last one was acknowledged
  Telemetry tlm_;
  float localVbat_;
  bool haveLocalVbat_;
  uint32_t nowMs_;

  uint8_t crsfTxBuf_[kCrsfTxSize];
  ByteQueue crsfTx_;
  Port ports_[kNumPorts];
  char cfgText_[kConfigTextMax];
};

}  // namespace eb
