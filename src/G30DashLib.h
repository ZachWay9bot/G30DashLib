#pragma once
#include <Arduino.h>

namespace G30Dash {

// Ninebot G30 legacy wire protocol, verified against stock DRV firmware:
//   5A A5 LEN SRC DST CMD ARG PAYLOAD[LEN] CK_LO CK_HI
// LEN is payload bytes only. Total frame size is LEN + 9.
// Checksum = ~(sum from LEN through final payload byte), little-endian.

static constexpr uint8_t HDR1 = 0x5A;
static constexpr uint8_t HDR2 = 0xA5;

enum Address : uint8_t {
  ADDR_ESC  = 0x20,
  ADDR_DASH = 0x21,
  ADDR_BMS  = 0x22,
  ADDR_BMS2 = 0x23,
  ADDR_APP  = 0x3E,
  ADDR_PC   = 0x3F
};

enum Command : uint8_t {
  CMD_READ       = 0x01,
  CMD_WRITE      = 0x02,
  CMD_WRITE_NR   = 0x03,
  CMD_READ_REPLY = 0x04,
  CMD_DASH_IO    = 0x64,
  CMD_DASH_INPUT = 0x65
};

enum FrameType : uint8_t {
  FrameOther = 0,
  DashToEsc64,
  DashToEsc65,
  EscToDash64,
  AppToEscRead,
  EscToAppReadReply
};

struct Frame {
  uint8_t len;             // payload length exactly as on wire
  uint8_t src;
  uint8_t dst;
  uint8_t cmd;
  uint8_t arg;
  uint8_t payload[128];
  uint8_t payload_len;
  uint16_t checksum;
  FrameType type;

  Frame()
  : len(0), src(0), dst(0), cmd(0), arg(0),
    payload{0}, payload_len(0), checksum(0), type(FrameOther) {}
};

struct Parser {
  bool saw5A;
  bool inFrame;
  uint8_t index;
  uint8_t expectedBody;
  uint16_t runningSum;
  uint8_t body[192];

  Parser() { reset(); }
  void reset() {
    saw5A = false;
    inFrame = false;
    index = 0;
    expectedBody = 0;
    runningSum = 0;
  }
};

enum ParseResult : uint8_t {
  NeedMore = 0,
  PacketReady,
  Rejected
};

uint16_t checksum(const uint8_t* data, size_t len);

size_t encode(uint8_t src, uint8_t dst, uint8_t cmd, uint8_t arg,
              const uint8_t* payload, uint8_t payloadLen,
              uint8_t* out, size_t outMax);

size_t encode(const Frame& frame, uint8_t* out, size_t outMax);

bool parse(const uint8_t* raw, size_t rawLen, Frame& out);
ParseResult feed(Parser& parser, uint8_t byte, Frame& out);

FrameType classify(const Frame& frame);

// G30 dashboard helpers, based on stock frames / verified community RE.
// Dash -> ESC CMD65 payload observed on stock G30:
//   [0]=04, [1]=gas, [2]=brake, [3]=02, [4]=00
bool decodeDash65(const Frame& frame, uint8_t& gas, uint8_t& brake);

// ESC -> Dash CMD64 payload is six bytes:
//   [mode, battery, light, beep, speed, error]
// For this user's stock DRV126 target, light ON is 0x64 and OFF is 0x00.
bool buildEsc64(uint8_t mode, uint8_t battery, bool lightOn,
                uint8_t beep, uint8_t speed, uint8_t error,
                Frame& out);

// Exact stock-style ARG10 serial reply helper.
bool buildArg10SerialReply(const uint8_t serial14[14], Frame& out);

} // namespace G30Dash
