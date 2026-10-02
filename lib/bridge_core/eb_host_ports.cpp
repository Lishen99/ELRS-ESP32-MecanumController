// Bridge: the two host-facing ports (USB serial and the host UART).
//
// Each port carries the binary host protocol and the text CLI side by side.
// This file sends the periodic update, answers protocol messages, and turns
// typed lines into CLI commands. The wire format itself is defined in
// elrs_bridge_protocol.h, shared with the host libraries.
#include <stdio.h>
#include <string.h>

#include "eb_bridge.h"

namespace eb {

namespace {

static_assert(kStreamChunkMax <= 255, "stream chunk lengths are kept in a byte");
static_assert((elrsb::kFrameOverhead + elrsb::kStateHeaderSize + 2 * elrsb::kMaxChannels) +
                      (elrsb::kFrameOverhead + 1 + 2 * elrsb::kMaxChannels) +
                      (elrsb::kFrameOverhead + elrsb::kLinkSize) <=
                  kStreamChunkMax,
              "a state, a raw-channel and a link frame must fit one stream chunk");

// Progress through the queued reply bytes as they are handed to the UART.
enum TxState { kTxText, kTxSync, kTxLen, kTxBody };

// CLI text written straight to a serial port; terminals want CR LF.
class TextOut : public Out {
 public:
  explicit TextOut(ByteQueue& queue) : queue_(queue) {}

  void write(const char* text, size_t len) override {
    size_t start = 0;
    for (size_t i = 0; i < len; ++i) {
      if (text[i] != '\n') continue;
      if (i > start) queue_.push((const uint8_t*)text + start, i - start);
      queue_.push((const uint8_t*)"\r\n", 2);
      start = i + 1;
    }
    if (len > start) queue_.push((const uint8_t*)text + start, len - start);
  }

 private:
  ByteQueue& queue_;
};

// CLI text carried back to a binary host as a run of kMsgCliReply frames,
// each marked with the tag of the command it answers.
class FrameOut : public Out {
 public:
  FrameOut(ByteQueue& queue, uint16_t& dropped, uint8_t tag)
      : queue_(queue), dropped_(dropped), len_(0) {
    chunk_[1] = tag;
  }

  void write(const char* text, size_t len) override {
    for (size_t i = 0; i < len; ++i) {
      chunk_[kHeader + len_++] = (uint8_t)text[i];
      if (len_ == kChunk) flush(false);
    }
  }

  void finish() { flush(true); }

 private:
  static const size_t kHeader = 2;  // flags, tag
  static const size_t kChunk = 96;

  void flush(bool last) {
    chunk_[0] = last ? (uint8_t)elrsb::kCliReplyLast : (uint8_t)0;
    uint8_t frame[elrsb::kMaxFrame];
    const size_t n = elrsb::encodeFrame(frame, elrsb::kMsgCliReply, chunk_, kHeader + len_);
    if (!queue_.push(frame, n) && dropped_ < 0xFFFF) ++dropped_;
    len_ = 0;
  }

  ByteQueue& queue_;
  uint16_t& dropped_;
  size_t len_;
  uint8_t chunk_[kHeader + kChunk];
};

void countDrop(uint16_t& dropped) {
  if (dropped < 0xFFFF) ++dropped;
}

}  // namespace

Bridge::Port::Port()
    : tx(txBuf, sizeof txBuf),
      streamLen(0),
      streamPos(0),
      txState(kTxText),
      txRemaining(0),
      txLineStart(true),
      lastWasStream(false),
      lineLen(0),
      lineOverflow(false),
      lastWasCr(false),
      lastRxMs(0),
      streamMode(elrsb::kStreamOff),
      streamHz(0),
      streamOpts(0),
      seq(0),
      lastStreamMs(0),
      lastLinkMs(0),
      dropped(0) {}

void Bridge::banner(Port& port) {
  TextOut out(port.tx);
  out.printf("\nelrs-bridge %u.%u.%u (%s) - type 'help' for commands\n", kVersionMajor,
             kVersionMinor, kVersionPatch, plat_.boardName());
  prompt(port);
}

void Bridge::prompt(Port& port) { port.tx.push((const uint8_t*)"> ", 2); }

void Bridge::setStreamMode(Port& port, uint8_t mode) {
  port.streamMode = mode;
  // An update built for the old mode must not go out in the new one. One that
  // is already part-way out is finished.
  if (port.streamPos == 0) port.streamLen = 0;
  port.lastStreamMs = nowMs_;
  port.lastLinkMs = nowMs_;
}

// ---------------------------------------------------------------------------
// The periodic update

void Bridge::streamPort(Port& port, uint32_t nowMs) {
  if (port.streamMode == elrsb::kStreamOff) return;

  const uint32_t hz = port.streamHz ? port.streamHz : cfg_.streamHz;
  uint32_t period = hz ? (1000 + hz / 2) / hz : 20;
  if (period == 0) period = 1;
  if ((uint32_t)(nowMs - port.lastStreamMs) < period) return;
  // Advance by whole periods to hold the cadence, but start over rather than
  // burst if the loop fell behind.
  port.lastStreamMs += period;
  if ((uint32_t)(nowMs - port.lastStreamMs) >= period) port.lastStreamMs = nowMs;

  uint8_t chunk[kStreamChunkMax];
  size_t len = 0;
  bool withLink = false;
  if (port.streamMode == elrsb::kStreamText) {
    len = buildTextState((char*)chunk, sizeof chunk);
  } else {
    len = buildState(port, chunk);
    if (port.streamOpts & elrsb::kStreamOptRaw) len += buildRawChannels(chunk + len);
    withLink = cfg_.linkHz && (uint32_t)(nowMs - port.lastLinkMs) >= 1000u / cfg_.linkHz;
    if (withLink) len += buildLink(port, chunk + len);
  }

  // The port keeps one update: the newest. If the line is too slow to have
  // sent the previous one, that one is replaced, so the host never receives
  // old states from a backlog. A host can see the gap in the sequence number.
  if (port.streamPos > 0) {
    countDrop(port.dropped);  // the previous update is part-way out: let it finish
    return;
  }
  if (port.streamLen > 0) countDrop(port.dropped);
  offerStream(port, chunk, len);
  if (withLink) port.lastLinkMs = nowMs;
}

void Bridge::offerStream(Port& port, const uint8_t* chunk, size_t len) {
  if (len == 0 || len > kStreamChunkMax) return;
  memcpy(port.streamBuf, chunk, len);
  port.streamLen = (uint8_t)len;
  port.streamPos = 0;
}

// Whether the periodic update is the next thing to send on this port.
bool Bridge::streamTurn(const Port& port) const {
  if (port.streamLen == 0) return false;
  if (port.streamPos > 0) return true;  // finish what was started
  // Never in the middle of a reply frame, nor (in text mode) of a text line.
  if (port.txState != kTxText) return false;
  if (port.streamMode == elrsb::kStreamText && !port.txLineStart) return false;
  // Take turns with queued replies so neither can starve the other.
  return !port.lastWasStream || port.tx.size() == 0;
}

size_t Bridge::portTxPending(PortId id) const {
  const Port& port = ports_[id];
  return port.tx.size() + (size_t)(port.streamLen - port.streamPos);
}

size_t Bridge::readPortTx(PortId id, uint8_t* out, size_t max) {
  Port& port = ports_[id];
  size_t n = 0;
  while (n < max) {
    if (streamTurn(port)) {
      size_t k = (size_t)(port.streamLen - port.streamPos);
      if (k > max - n) k = max - n;
      memcpy(out + n, port.streamBuf + port.streamPos, k);
      port.streamPos = (uint8_t)(port.streamPos + k);
      n += k;
      if (port.streamPos == port.streamLen) {
        port.streamLen = 0;
        port.streamPos = 0;
        port.lastWasStream = true;
      }
      continue;
    }

    uint8_t byte;
    if (port.tx.pop(&byte, 1) == 0) {
      // Nothing queued. Frames are queued whole, so none can be in progress.
      port.txState = kTxText;
      // If reply text stopped part-way through a line while a text update is
      // waiting, end the line so the update can follow on a line of its own.
      if (port.streamLen > 0 && port.streamMode == elrsb::kStreamText && !port.txLineStart) {
        if (max - n < 2) break;
        out[n++] = '\r';
        out[n++] = '\n';
        port.txLineStart = true;
        continue;
      }
      break;
    }
    out[n++] = byte;

    // Follow the reply bytes to know where one item ends and the next begins.
    switch (port.txState) {
      case kTxText:
        if (byte == elrsb::kSync1) {
          port.txState = kTxSync;
        } else {
          port.txLineStart = byte == '\n';
        }
        break;
      case kTxSync:
        if (byte == elrsb::kSync2) {
          port.txState = kTxLen;
        } else {
          port.txState = kTxText;
          port.txLineStart = byte == '\n';
        }
        break;
      case kTxLen:
        port.txRemaining = (uint16_t)(byte + 3);  // type, payload, two CRC bytes
        port.txState = kTxBody;
        break;
      default:
        if (--port.txRemaining == 0) port.txState = kTxText;
        break;
    }
    const bool itemDone = port.txState == kTxText &&
                          (port.streamMode != elrsb::kStreamText || port.txLineStart);
    if (itemDone) port.lastWasStream = false;
  }
  return n;
}

// ---------------------------------------------------------------------------
// Messages to the host

uint8_t Bridge::flags() const {
  uint8_t f = rc_.linkUp() ? (uint8_t)elrsb::kFlagLinkUp : (uint8_t)elrsb::kFlagFailsafe;
  if (rc_.armed()) f |= elrsb::kFlagArmed;
  if (drive_.source() == Drive::kGamepad) f |= elrsb::kFlagGamepad;
  if (drive_.live()) f |= elrsb::kFlagMotorsLive;
  return f;
}

void Bridge::fillState(elrsb::State& state) const {
  state.flags = flags();
  // Link quality is zeroed while the link is down; RSSI and SNR keep the
  // last values the receiver reported.
  state.lq = (haveLink_ && rc_.linkUp()) ? link_.upLq : (uint8_t)0;
  state.rssi = haveLink_ ? (link_.antenna ? link_.upRssi2 : link_.upRssi1) : (int8_t)0;
  state.snr = haveLink_ ? link_.upSnr : (int8_t)0;
  state.ageMs = rc_.ageMs(nowMs_);
  state.seq = 0;
  state.count = clampT<uint8_t>(cfg_.streamCh, 1, kNumOutputs);
  for (int i = 0; i < kNumOutputs; ++i) state.ch[i] = rc_.out(i);
}

size_t Bridge::buildState(Port& port, uint8_t* frame) {
  elrsb::State state;
  fillState(state);
  state.seq = port.seq++;
  uint8_t payload[elrsb::kMaxPayload];
  return elrsb::encodeFrame(frame, elrsb::kMsgState, payload, elrsb::packState(payload, state));
}

size_t Bridge::buildTextState(char* line, size_t cap) {
  elrsb::State state;
  fillState(state);
  int n = snprintf(line, cap, "$RC,%u,%u,%d,%u", (unsigned)state.flags, (unsigned)state.lq,
                   (int)state.rssi, (unsigned)state.ageMs);
  for (int i = 0; i < state.count; ++i) {
    n += snprintf(line + n, cap - (size_t)n, ",%d", (int)state.ch[i]);
  }
  // NMEA-style checksum: XOR of everything between '$' and '*'.
  uint8_t sum = 0;
  for (int i = 1; i < n; ++i) sum ^= (uint8_t)line[i];
  n += snprintf(line + n, cap - (size_t)n, "*%02X\r\n", (unsigned)sum);
  return (size_t)n;
}

size_t Bridge::buildLink(const Port& port, uint8_t* frame) {
  elrsb::Link link;
  memset(&link, 0, sizeof link);
  if (haveLink_) {
    link.upRssi1 = link_.upRssi1;
    link.upRssi2 = link_.upRssi2;
    link.upLq = rc_.linkUp() ? link_.upLq : (uint8_t)0;
    link.upSnr = link_.upSnr;
    link.antenna = link_.antenna;
    link.rfMode = link_.rfMode;
    link.txPowerMw = link_.txPowerMw;
    link.downRssi = link_.downRssi;
    link.downLq = link_.downLq;
    link.downSnr = link_.downSnr;
  }
  link.flags = flags();
  link.rcRateHz = rc_.rateHz();
  link.vbatMv =
      haveLocalVbat_ ? (uint16_t)clampT<float>(localVbat_ * 1000.0f + 0.5f, 0.0f, 65535.0f) : 0;
  link.uptimeMs = nowMs_;
  link.rcFrames = rc_.frames();
  link.crsfErrors = crsfParser_.errors();
  link.failsafeCount = rc_.failsafeCount();
  link.txDropped = port.dropped;
  uint8_t payload[elrsb::kMaxPayload];
  return elrsb::encodeFrame(frame, elrsb::kMsgLink, payload, elrsb::packLink(payload, link));
}

size_t Bridge::buildRawChannels(uint8_t* frame) {
  uint16_t us[kNumOutputs];
  for (int i = 0; i < kNumOutputs; ++i) us[i] = rc_.rawUs(i);
  uint8_t payload[elrsb::kMaxPayload];
  return elrsb::encodeFrame(frame, elrsb::kMsgRawChannels, payload,
                            elrsb::packRawChannels(payload, us, kNumOutputs));
}

// Queue a complete reply item. It is all or nothing, so a frame is never
// queued in part.
bool Bridge::queueBytes(Port& port, const uint8_t* data, size_t len) {
  if (len == 0 || !port.tx.push(data, len)) {
    countDrop(port.dropped);
    return false;
  }
  return true;
}

bool Bridge::sendFrame(Port& port, uint8_t type, const uint8_t* payload, size_t len) {
  uint8_t frame[elrsb::kMaxFrame];
  return queueBytes(port, frame, elrsb::encodeFrame(frame, type, payload, len));
}

void Bridge::sendAck(Port& port, uint8_t type, uint8_t status) {
  const uint8_t payload[2] = {type, status};
  sendFrame(port, elrsb::kMsgAck, payload, sizeof payload);
}

void Bridge::sendInfo(Port& port) {
  elrsb::Info info;
  memset(&info, 0, sizeof info);
  info.protocol = elrsb::kProtocolVersion;
  info.fwMajor = kVersionMajor;
  info.fwMinor = kVersionMinor;
  info.fwPatch = kVersionPatch;
  info.channels = kNumOutputs;
  info.auxOutputs = kNumAux;
  strncpy(info.board, plat_.boardName(), sizeof info.board - 1);
  uint8_t payload[elrsb::kMaxPayload];
  sendFrame(port, elrsb::kMsgInfo, payload, elrsb::packInfo(payload, info));
}

// ---------------------------------------------------------------------------
// Bytes from the host

void Bridge::feedPort(PortId id, const uint8_t* data, size_t len, uint32_t nowMs) {
  nowMs_ = nowMs;
  Port& port = ports_[id];
  if (len == 0) return;
  // A frame that stalled part-way must not swallow whatever arrives later.
  if (!port.parser.idle() && (uint32_t)(nowMs - port.lastRxMs) > 100) port.parser.reset();
  port.lastRxMs = nowMs;

  for (size_t i = 0; i < len; ++i) {
    switch (port.parser.feed(data[i])) {
      case elrsb::kFeedFrame:
        handleFrame(port, port.parser.type(), port.parser.payload(), port.parser.length());
        break;
      case elrsb::kFeedText:
        handleTextByte(port, data[i]);
        break;
      default:
        break;
    }
  }
}

void Bridge::handleFrame(Port& port, uint8_t type, const uint8_t* payload, size_t len) {
  const uint32_t now = nowMs_;
  uint8_t frame[elrsb::kMaxFrame];

  switch (type) {
    case elrsb::kMsgGetInfo:
      sendInfo(port);
      break;

    case elrsb::kMsgSetStream: {
      elrsb::SetStream s;
      if (!elrsb::unpackSetStream(payload, len, s)) {
        sendAck(port, type, elrsb::kAckBadLength);
      } else if (s.mode > elrsb::kStreamText || s.hz > 500) {
        sendAck(port, type, elrsb::kAckBadValue);
      } else {
        setStreamMode(port, s.mode);
        port.streamHz = s.hz;
        port.streamOpts = s.options;
        sendAck(port, type, elrsb::kAckOk);
      }
      break;
    }

    case elrsb::kMsgCli: {
      // Payload: a tag chosen by the host, then the command line. Always
      // answered with a terminated reply carrying that tag, so the host never
      // has to time out to learn that a command was rejected.
      if (len < 1) {
        sendAck(port, type, elrsb::kAckBadLength);
        break;
      }
      FrameOut out(port.tx, port.dropped, payload[0]);
      char line[kLineMax];
      if (len - 1 >= sizeof line) {
        out.print("ERR line too long\n");
      } else {
        memcpy(line, payload + 1, len - 1);
        line[len - 1] = '\0';
        runCli(out, line, port);
      }
      out.finish();
      break;
    }

    case elrsb::kMsgRequest:
      if (len < 1) {
        sendAck(port, type, elrsb::kAckBadLength);
      } else if (payload[0] == elrsb::kMsgState) {
        queueBytes(port, frame, buildState(port, frame));
      } else if (payload[0] == elrsb::kMsgLink) {
        queueBytes(port, frame, buildLink(port, frame));
      } else if (payload[0] == elrsb::kMsgRawChannels) {
        queueBytes(port, frame, buildRawChannels(frame));
      } else {
        sendAck(port, type, elrsb::kAckBadValue);
      }
      break;

    // Telemetry is sent repeatedly, so it is only answered when malformed.
    case elrsb::kMsgTlmBattery: {
      elrsb::Battery v;
      if (elrsb::unpackBattery(payload, len, v)) tlm_.setBattery(v, now);
      else sendAck(port, type, elrsb::kAckBadLength);
      break;
    }
    case elrsb::kMsgTlmGps: {
      elrsb::Gps v;
      if (elrsb::unpackGps(payload, len, v)) tlm_.setGps(v, now);
      else sendAck(port, type, elrsb::kAckBadLength);
      break;
    }
    case elrsb::kMsgTlmAttitude: {
      elrsb::Attitude v;
      if (elrsb::unpackAttitude(payload, len, v)) tlm_.setAttitude(v, now);
      else sendAck(port, type, elrsb::kAckBadLength);
      break;
    }
    case elrsb::kMsgTlmMode:
      tlm_.setMode((const char*)payload, len, now);
      break;
    case elrsb::kMsgTlmVario: {
      float v;
      if (elrsb::unpackFloat(payload, len, v)) tlm_.setVario(v, now);
      else sendAck(port, type, elrsb::kAckBadLength);
      break;
    }
    case elrsb::kMsgTlmBaro: {
      elrsb::Baro v;
      if (elrsb::unpackBaro(payload, len, v)) tlm_.setBaro(v, now);
      else sendAck(port, type, elrsb::kAckBadLength);
      break;
    }
    case elrsb::kMsgTlmAirspeed: {
      float v;
      if (elrsb::unpackFloat(payload, len, v)) tlm_.setAirspeed(v, now);
      else sendAck(port, type, elrsb::kAckBadLength);
      break;
    }
    case elrsb::kMsgTlmRpm: {
      elrsb::Rpm v;
      if (elrsb::unpackRpm(payload, len, v)) tlm_.setRpm(v, now);
      else sendAck(port, type, elrsb::kAckBadLength);
      break;
    }
    case elrsb::kMsgTlmTemp: {
      elrsb::Temp v;
      if (elrsb::unpackTemp(payload, len, v)) tlm_.setTemp(v, now);
      else sendAck(port, type, elrsb::kAckBadLength);
      break;
    }

    case elrsb::kMsgCrsfTx: {
      if (len < 1 || len - 1 > crsf::kMaxPayload) {
        sendAck(port, type, elrsb::kAckBadLength);
        break;
      }
      uint8_t crsfFrame[crsf::kMaxFrameSize];
      const size_t n = crsf::buildFrame(crsfFrame, payload[0], payload + 1, len - 1);
      if (!sendCrsf(crsfFrame, n)) sendAck(port, type, elrsb::kAckBusy);
      break;
    }

    default:
      sendAck(port, type, elrsb::kAckUnknown);
      break;
  }
}

void Bridge::handleTextByte(Port& port, uint8_t byte) {
  // Echo and prompt are for a person at a terminal. A port that is streaming
  // is being read by a program, which wants neither.
  const bool interactive = port.streamMode == elrsb::kStreamOff;

  if (byte == '\r' || byte == '\n') {
    if (byte == '\n' && port.lastWasCr) {
      port.lastWasCr = false;
      return;
    }
    port.lastWasCr = byte == '\r';
    TextOut out(port.tx);
    if (interactive) out.print("\n");
    if (port.lineOverflow) {
      out.print("ERR line too long\n");
    } else if (port.lineLen > 0) {
      port.line[port.lineLen] = '\0';
      runCli(out, port.line, port);
    }
    port.lineLen = 0;
    port.lineOverflow = false;
    // The command may itself have switched streaming on or off.
    if (port.streamMode == elrsb::kStreamOff) prompt(port);
    return;
  }

  port.lastWasCr = false;
  if (byte == 0x08 || byte == 0x7F) {
    if (port.lineLen > 0) {
      --port.lineLen;
      if (interactive) port.tx.push((const uint8_t*)"\b \b", 3);
    }
    return;
  }
  if (byte < 0x20 || byte > 0x7E) return;
  if (port.lineLen < kLineMax - 1) {
    port.line[port.lineLen++] = (char)byte;
    if (interactive) port.tx.push(&byte, 1);
  } else {
    port.lineOverflow = true;
  }
}

}  // namespace eb
