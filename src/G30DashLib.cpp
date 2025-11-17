#include "G30DashLib.h"

namespace G30Dash {

static uint16_t crc_sum16(const uint8_t* data, size_t n) {
  uint32_t sum = 0;
  for (size_t i = 0; i < n; ++i) {
    sum += data[i];
  }
  return static_cast<uint16_t>(sum & 0xFFFFu);
}

static uint16_t crc_inv16(const uint8_t* data, size_t n) {
  uint16_t s = crc_sum16(data, n);
  return static_cast<uint16_t>(~s);
}

static uint16_t crc_xor8(const uint8_t* data, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; ++i) {
    x ^= data[i];
  }
  // Store in low byte, high byte zeroed
  return static_cast<uint16_t>(x);
}

uint16_t calcCrc16(const uint8_t* data, size_t n, CrcMode mode) {
  if (!data || n == 0) return 0;
  switch (mode) {
    case CRC_INV16:  return crc_inv16(data, n);
    case CRC_SUM16:  return crc_sum16(data, n);
    case CRC_XOR8:   return crc_xor8(data, n);
    default:         return crc_inv16(data, n);
  }
}

bool parse(const uint8_t* buf, size_t n, Frame& out,
           const Config& cfg) {
  if (!buf || n < 7) return false; // minimal header + crc
  if (buf[0] != 0x5A || buf[1] != 0xA5) return false;

  uint8_t L = buf[2];
  size_t expected = static_cast<size_t>(L) + 5; // 2 hdr + 1 len + L + 2 crc
  if (n < expected) return false;

  const uint8_t* crcPtr = buf + 3 + L;
  uint16_t crc_rx = static_cast<uint16_t>(crcPtr[0]) |
                    (static_cast<uint16_t>(crcPtr[1]) << 8);

  uint16_t crc_calc = calcCrc16(buf + 2, static_cast<size_t>(1 + L), cfg.crcMode);
  if (crc_rx != crc_calc) {
    return false;
  }

  out.len = L;
  out.dst = buf[3];
  out.src = buf[4];
  out.cmd = buf[5];

  uint8_t payloadLen = (L >= 3) ? (L - 3) : 0;
  if (payloadLen > sizeof(out.payload)) {
    payloadLen = sizeof(out.payload);
  }

  out.payload_len = payloadLen;
  for (uint8_t i = 0; i < payloadLen; ++i) {
    out.payload[i] = buf[6 + i];
  }

  out.crc = crc_rx;
  out.type = FrameOther;

  if (out.cmd == 0x64 && out.dst == cfg.dashAddress && out.src == cfg.escAddress) {
    out.type = EscToDash64;
  } else if (out.cmd == 0x65 && out.dst == cfg.escAddress && out.src == cfg.dashAddress) {
    out.type = DashToEsc65;
  }

  return true;
}

size_t encode(const Frame& frame, uint8_t* out, size_t outMax,
              const Config& cfg) {
  if (!out || outMax < 7) return 0;

  uint8_t payloadLen = frame.payload_len;
  if (payloadLen > sizeof(frame.payload)) {
    payloadLen = sizeof(frame.payload);
  }

  uint8_t L = static_cast<uint8_t>(3 + payloadLen); // dst,src,cmd + payload
  size_t needed = static_cast<size_t>(L) + 5;
  if (outMax < needed) return 0;

  out[0] = 0x5A;
  out[1] = 0xA5;
  out[2] = L;
  out[3] = frame.dst;
  out[4] = frame.src;
  out[5] = frame.cmd;

  for (uint8_t i = 0; i < payloadLen; ++i) {
    out[6 + i] = frame.payload[i];
  }

  uint16_t crc = calcCrc16(out + 2, static_cast<size_t>(1 + L), cfg.crcMode);
  size_t crcPos = 3 + static_cast<size_t>(L);
  out[crcPos + 0] = static_cast<uint8_t>(crc & 0xFFu);
  out[crcPos + 1] = static_cast<uint8_t>((crc >> 8) & 0xFFu);

  return needed;
}

bool build65Reply(const Frame& req64,
                  const uint8_t* payload, size_t payloadLen,
                  Frame& out,
                  const Config& cfg) {
  if (req64.cmd != 0x64) return false;
  if (!payload && payloadLen > 0) return false;
  if (payloadLen > sizeof(out.payload)) {
    payloadLen = sizeof(out.payload);
  }

  out.dst = req64.src; // mirror back
  out.src = req64.dst;
  out.cmd = 0x65;
  out.payload_len = static_cast<uint8_t>(payloadLen);
  out.len = static_cast<uint8_t>(3 + out.payload_len);

  for (size_t i = 0; i < payloadLen; ++i) {
    out.payload[i] = payload[i];
  }

  out.type = DashToEsc65;
  out.crc = 0;

  (void)cfg; // currently only used by encode()
  return true;
}

bool heartbeatDue(Heartbeat& hb, uint32_t nowMs) {
  if (hb.intervalMs == 0) return false;
  if (nowMs - hb.lastTxMs >= hb.intervalMs) {
    hb.lastTxMs = nowMs;
    return true;
  }
  return false;
}

} // namespace G30Dash
