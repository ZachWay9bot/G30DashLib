# G30DashLib v1

Core helper library for the **Ninebot G30** dashboard/ESC UART protocol  
(`0x5A 0xA5 ... 0x64/0x65` frames).

This is meant as a **small, readable kernel** you can drop into your own
projects when you want to:

- Parse incoming `0x5A 0xA5` frames from the G30 bus
- Build correct frames with length + CRC for replies / simulators
- Focus on *what* you want to send (mode, battery, speed, flags…),
  not *how* to pack bytes and checksums

It is intentionally minimal and opinionated: G30‑first, ESC/Dash
addresses baked in as defaults, and only the parts that are stable and
well‑understood live here.

---

## Features

- Recognises and parses `0x5A 0xA5` frames into a simple `Frame` struct
- Auto‑classification of command frames:
  - `0x64` (Dash ↔ ESC control)
  - `0x65` (ESC ↔ Dash status / ACK)
- Configurable checksum mode (`INV16`, `SUM16`, `XOR8`) via `Config`
- Helper to **build 0x65 replies** from incoming `0x64` frames
- Small `Heartbeat` helper to schedule periodic status frames
- Designed for **ESP32 / Arduino** but works on any Arduino‑compatible
  platform

This lib does **not** try to be a full protocol implementation. It
just does the heavy lifting around framing and CRC so your application
logic stays clean.

---

## Installation

### Arduino IDE

1. Download the ZIP:  
   `G30DashLib_v1_GitHub.zip`
2. In Arduino IDE: `Sketch → Include Library → Add .ZIP Library…`
3. Select the ZIP – it will install as `G30DashLib`.

You should then see the examples under  
`File → Examples → G30DashLib`.

---

## API Overview

Header:

```cpp
#include <G30DashLib.h>
using namespace G30Dash;
```

Core types:

```cpp
namespace G30Dash {

enum CrcMode : uint8_t {
  CRC_INV16,   // 16-bit sum, then bitwise NOT  (~sum)
  CRC_SUM16,   // plain 16-bit sum
  CRC_XOR8     // 8-bit XOR, stored in low byte
};

struct Config {
  CrcMode crcMode;   // default: CRC_INV16
  uint8_t escAddress;  // default: 0x20
  uint8_t dashAddress; // default: 0x21
};

enum FrameType : uint8_t {
  FrameOther   = 0,
  EscToDash64  = 1,   // cmd 0x64 with esc→dash addresses (by convention)
  DashToEsc65  = 2    // cmd 0x65 with dash→esc addresses (by convention)
};

struct Frame {
  uint8_t dst;
  uint8_t src;
  uint8_t cmd;
  uint8_t len;           // L from the frame (dst..payload)
  uint8_t payload[64];   // payload bytes (without dst/src/cmd)
  uint8_t payload_len;   // usually len - 3
  uint16_t crc;          // raw CRC bytes as in the frame
  FrameType type;        // auto‑classified, or FrameOther
};

struct Heartbeat {
  uint32_t intervalMs;
  uint32_t lastTxMs;
};
```

Core functions:

```cpp
// Generic CRC helper, can be reused outside of frames:
uint16_t calcCrc16(const uint8_t* data, size_t n, CrcMode mode);

// Parse 5A A5 frame from buffer → Frame
// Returns true on valid frame (header + length + CRC OK).
bool parse(const uint8_t* buf, size_t n, Frame& out,
           const Config& cfg = Config());

// Encode Frame → raw bytes (5A A5 LEN ... CRC0 CRC1)
// Returns number of bytes written, or 0 on error.
size_t encode(const Frame& frame, uint8_t* out, size_t outMax,
              const Config& cfg = Config());

// Convenience helper:
//   Takes an incoming 0x64 frame, flips src/dst and cmd → 0x65,
//   injects your payload and fills CRC/length.
bool build65Reply(const Frame& req64,
                  const uint8_t* payload, size_t payloadLen,
                  Frame& out,
                  const Config& cfg = Config());

// Small scheduling helper: returns true if it's time to send another
// status frame.
bool heartbeatDue(Heartbeat& hb, uint32_t nowMs);
```

Defaults (if you don’t touch `Config`):

- ESC address: `0x20`
- Dash address: `0x21`
- CRC mode: `CRC_INV16` (typical for G30 ESC/Dash)

---

## Example: Simple ESC Simulator

`examples/G30Dash_EscSimulator/G30Dash_EscSimulator.ino`

This sketch:

- Opens `Serial1` as G30 bus @ 115200
- Periodically sends a `0x65` “I am a fake ESC” frame
- Slowly ramps a fake speed value up and down

Minimal skeleton:

```cpp
#include <Arduino.h>
#include <G30DashLib.h>

using namespace G30Dash;

#ifndef RX_PIN
#define RX_PIN 9
#endif
#ifndef TX_PIN
#define TX_PIN 8
#endif

HardwareSerial& BUS = Serial1;

Config cfg;               // defaults are fine for stock G30
Heartbeat hb{60};         // send every 60 ms

void setup() {
  Serial.begin(115200);
  BUS.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("G30Dash ESC Simulator");
}

void loop() {
  uint32_t now = millis();

  if (heartbeatDue(hb, now)) {
    static uint8_t speed = 0;
    static int8_t dir = 1;

    speed += dir;
    if (speed > 45) { speed = 45; dir = -1; }
    if (speed <  0) { speed = 0;  dir = +1; }

    Frame f{};
    f.dst = cfg.dashAddress;
    f.src = cfg.escAddress;
    f.cmd = 0x65;

    // VERY simplified payload: [mode, battery, speed_kmh, reserved]
    const uint8_t payload[4] = { 1, 80, speed, 0 };

    f.payload_len = sizeof(payload);
    f.len = 3 + f.payload_len; // dst + src + cmd + payload

    for (uint8_t i = 0; i < f.payload_len; ++i)
      f.payload[i] = payload[i];

    uint8_t raw[80];
    size_t n = encode(f, raw, sizeof(raw), cfg);
    if (n) {
      BUS.write(raw, n);
    }
  }

  // Optional: read incoming frames for debug printing…
}
```

This does **not** claim to be 100% register accurate – it’s a
playground/demo that speaks “good enough” ESCish G30 to make a dash
happy and give you a starting point for your own simulator.

---

## Example: Dash Tester / Sniffer

`examples/G30Dash_Tester/G30Dash_Tester.ino` shows a very small
streaming parser:

- Reads bytes from `Serial1`
- Scans for `0x5A 0xA5`
- Once a whole frame is buffered, calls `G30Dash::parse()`
- Prints decoded info (cmd, len, payload bytes) to `Serial`

From there you can:

- Map throttle/brake bytes to LEDs
- Dump payloads for reverse‑engineering
- Log unknown frames and experiment

---

## Versioning

This is **G30DashLib v1.0.0** – first public, “core only” release.

Things deliberately left for future versions:

- More helpers for specific registers / payload layouts
- Built‑in LED helpers (winglight visualisation, etc.)
- Optional higher‑level “state” object (mode, speed, faults…)

The idea is to keep this file small and understandable, and layer any
project‑specific magic on top in your own sketches or helper modules.

---

## License

MIT – see [`LICENSE`](LICENSE).

Do whatever you like with it, but **don’t** ship it into traffic
without understanding what it does. 😉
