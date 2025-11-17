#pragma once
#include <Arduino.h>

namespace G30Dash {

enum CrcMode : uint8_t {
  CRC_INV16,
  CRC_SUM16,
  CRC_XOR8
};

struct Config {
  CrcMode crcMode;
  uint8_t escAddress;
  uint8_t dashAddress;

  Config()
  : crcMode(CRC_INV16),
    escAddress(0x20),
    dashAddress(0x21) {}
};

enum FrameType : uint8_t {
  FrameOther   = 0,
  EscToDash64  = 1,
  DashToEsc65  = 2
};

struct Frame {
  uint8_t dst;
  uint8_t src;
  uint8_t cmd;
  uint8_t len;
  uint8_t payload[64];
  uint8_t payload_len;
  uint16_t crc;
  FrameType type;

  Frame()
  : dst(0), src(0), cmd(0), len(0),
    payload{0}, payload_len(0),
    crc(0), type(FrameOther) {}
};

struct Heartbeat {
  uint32_t intervalMs;
  uint32_t lastTxMs;
  Heartbeat(uint32_t interval = 60)
  : intervalMs(interval), lastTxMs(0) {}
};

uint16_t calcCrc16(const uint8_t* data, size_t n, CrcMode mode);

bool parse(const uint8_t* buf, size_t n, Frame& out,
           const Config& cfg = Config());

size_t encode(const Frame& frame, uint8_t* out, size_t outMax,
              const Config& cfg = Config());

bool build65Reply(const Frame& req64,
                  const uint8_t* payload, size_t payloadLen,
                  Frame& out,
                  const Config& cfg = Config());

bool heartbeatDue(Heartbeat& hb, uint32_t nowMs);

} // namespace G30Dash
