#include "G30DashLib.h"
#include <string.h>

namespace G30Dash {

uint16_t checksum(const uint8_t* data, size_t len) {
  if (!data && len) return 0;
  uint16_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum = static_cast<uint16_t>(sum + data[i]);
  }
  return static_cast<uint16_t>(~sum);
}

static FrameType classifyFields(uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg) {
  (void)arg;
  if (src == ADDR_DASH && dst == ADDR_ESC && cmd == CMD_DASH_IO) return DashToEsc64;
  if (src == ADDR_DASH && dst == ADDR_ESC && cmd == CMD_DASH_INPUT) return DashToEsc65;
  if (src == ADDR_ESC && dst == ADDR_DASH && cmd == CMD_DASH_IO) return EscToDash64;
  if (src == ADDR_APP && dst == ADDR_ESC && cmd == CMD_READ) return AppToEscRead;
  if (src == ADDR_ESC && dst == ADDR_APP && cmd == CMD_READ_REPLY) return EscToAppReadReply;
  return FrameOther;
}

FrameType classify(const Frame& frame) {
  return classifyFields(frame.src, frame.dst, frame.cmd, frame.arg);
}

size_t encode(uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg,
              const uint8_t* payload, uint8_t payloadLen,
              uint8_t* out, size_t outMax) {
  const size_t total = static_cast<size_t>(payloadLen) + 9u;
  if (!out || outMax < total) return 0;
  if (payloadLen && !payload) return 0;

  out[0] = HDR1;
  out[1] = HDR2;
  out[2] = payloadLen;
  out[3] = src;
  out[4] = dst;
  out[5] = cmd;
  out[6] = arg;

  if (payloadLen) memcpy(out + 7, payload, payloadLen);

  const uint16_t ck = checksum(out + 2, static_cast<size_t>(payloadLen) + 5u);
  out[7 + payloadLen] = static_cast<uint8_t>(ck & 0xFFu);
  out[8 + payloadLen] = static_cast<uint8_t>(ck >> 8);
  return total;
}

size_t encode(const Frame& frame, uint8_t* out, size_t outMax) {
  return encode(frame.src, frame.dst, frame.cmd, frame.arg,
                frame.payload, frame.payload_len, out, outMax);
}

static void fillFrame(Frame& out, const uint8_t* body) {
  out.len = body[0];
  out.src = body[1];
  out.dst = body[2];
  out.cmd = body[3];
  out.arg = body[4];
  out.payload_len = body[0];
  if (out.payload_len) memcpy(out.payload, body + 5, out.payload_len);
  out.checksum = static_cast<uint16_t>(
      body[5 + out.payload_len] |
      (static_cast<uint16_t>(body[6 + out.payload_len]) << 8));
  out.type = classify(out);
}

bool parse(const uint8_t* raw, size_t rawLen, Frame& out) {
  if (!raw || rawLen < 9) return false;
  if (raw[0] != HDR1 || raw[1] != HDR2) return false;

  const uint8_t len = raw[2];
  const size_t total = static_cast<size_t>(len) + 9u;
  if (rawLen != total) return false;
  if (len > sizeof(out.payload)) return false;

  const uint16_t expected = checksum(raw + 2, static_cast<size_t>(len) + 5u);
  const uint16_t received = static_cast<uint16_t>(
      raw[7 + len] | (static_cast<uint16_t>(raw[8 + len]) << 8));
  if (expected != received) return false;

  // body starts at LEN
  uint8_t body[192];
  const size_t bodyLen = static_cast<size_t>(len) + 7u;
  memcpy(body, raw + 2, bodyLen);
  fillFrame(out, body);
  return true;
}

ParseResult feed(Parser& p, uint8_t b, Frame& out) {
  if (!p.inFrame) {
    if (!p.saw5A) {
      if (b == HDR1) {
        p.saw5A = true;
        return NeedMore;
      }
      return Rejected;
    }

    if (b == HDR2) {
      p.inFrame = true;
      p.index = 0;
      p.expectedBody = 0;
      p.runningSum = 0;
      return NeedMore;
    }

    // repeated 5A keeps us armed, anything else resets
    p.saw5A = (b == HDR1);
    return Rejected;
  }

  if (p.index >= sizeof(p.body)) {
    p.reset();
    return Rejected;
  }

  p.body[p.index] = b;

  if (p.index == 0) {
    const uint16_t expected = static_cast<uint16_t>(b) + 7u;
    if (expected > sizeof(p.body) || b > sizeof(out.payload)) {
      p.reset();
      return Rejected;
    }
    p.expectedBody = static_cast<uint8_t>(expected);
  }

  p.index++;

  if (p.index == p.expectedBody) {
    const uint8_t len = p.body[0];
    const uint16_t received = static_cast<uint16_t>(
        p.body[5 + len] | (static_cast<uint16_t>(p.body[6 + len]) << 8));
    const uint16_t expected = checksum(p.body, static_cast<size_t>(len) + 5u);

    if (received != expected) {
      p.reset();
      return Rejected;
    }

    fillFrame(out, p.body);
    p.reset();
    return PacketReady;
  }

  return NeedMore;
}

bool decodeDash65(const Frame& frame, uint8_t& gas, uint8_t& brake) {
  if (frame.src != ADDR_DASH || frame.dst != ADDR_ESC ||
      frame.cmd != CMD_DASH_INPUT || frame.arg != 0x00 ||
      frame.payload_len < 3) {
    return false;
  }

  gas = frame.payload[1];
  brake = frame.payload[2];
  return true;
}

bool buildEsc64(uint8_t mode, uint8_t battery, bool lightOn,
                uint8_t beep, uint8_t speed, uint8_t error,
                Frame& out) {
  out = Frame();
  out.src = ADDR_ESC;
  out.dst = ADDR_DASH;
  out.cmd = CMD_DASH_IO;
  out.arg = 0x00;
  out.payload_len = 6;
  out.len = 6;
  out.payload[0] = mode;
  out.payload[1] = battery;
  out.payload[2] = lightOn ? 0x64 : 0x00;
  out.payload[3] = beep;
  out.payload[4] = speed;
  out.payload[5] = error;
  out.type = EscToDash64;
  return true;
}

bool buildArg10SerialReply(const uint8_t serial14[14], Frame& out) {
  if (!serial14) return false;
  out = Frame();
  out.src = ADDR_ESC;
  out.dst = ADDR_APP;
  out.cmd = CMD_READ_REPLY;
  out.arg = 0x10;
  out.payload_len = 14;
  out.len = 14;
  memcpy(out.payload, serial14, 14);
  out.type = EscToAppReadReply;
  return true;
}

bool buildWriteU16(uint8_t src, uint8_t dst, uint8_t reg, uint16_t value, Frame& out) {
  out = Frame();
  out.src = src;
  out.dst = dst;
  out.cmd = CMD_WRITE;
  out.arg = reg;
  out.payload_len = 2;
  out.len = 2;
  out.payload[0] = static_cast<uint8_t>(value & 0xFFu);
  out.payload[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  out.type = FrameOther;
  return true;
}

} // namespace G30Dash
