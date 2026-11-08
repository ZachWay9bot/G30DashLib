# G30DashLib v2 — stock-verified Ninebot G30 protocol core

This library is now deliberately **G30-only** and follows the stock Ninebot wire format verified from
stock DRV firmware disassembly and real G30 captures.

> **v1 warning:** v1 modeled the length and field layout incorrectly. Do not use v1 as a protocol reference.

## Verified wire format

```
5A A5 LEN SRC DST CMD ARG PAYLOAD[LEN] CK_LO CK_HI
```

- `LEN` is **payload byte count only**.
- Total frame size is `LEN + 9`.
- `SRC` and `DST` are on-wire source/destination addresses in that order.
- `ARG` is a real protocol field and is not payload.
- checksum is `~sum(LEN..last payload byte)`, 16-bit, little-endian.

Addresses:

- ESC `0x20`
- Dashboard/BLE `0x21`
- BMS `0x22`
- App `0x3E`
- PC `0x3F`

## Golden stock G30 frames

Real Dash -> ESC throttle/brake frame:

```
5A A5 05 21 20 65 00 04 29 21 02 00 04 FF
```

This decodes as:

- LEN = 5
- SRC = 0x21 Dashboard
- DST = 0x20 ESC
- CMD = 0x65
- ARG = 0x00
- payload = `04 29 21 02 00`
- gas = `0x29`
- brake = `0x21`

Real Dash -> ESC CMD64:

```
5A A5 07 21 20 64 00 06 29 21 02 00 17 01 E9 FE
```

Real ESC -> Dash CMD64, light off:

```
5A A5 06 20 21 64 00 02 00 00 00 00 00 52 FF
```

For the stock DRV126 target used for this project, the six-byte ESC->Dash CMD64 payload is treated as:

```
[mode, battery, light, beep, speed, error]
```

with light:

- OFF = `0x00`
- ON  = `0x64`

Exact ARG10 request observed on the real stock bus:

```
5A A5 04 3E 20 01 10 0E 02 29 21 32 FF
```

Exact stock serial reply:

```
5A A5 0E 20 3E 04 10 4E 34 47 53 44 31 31 31 31 43 35 34 38 37 40 FC
```

## API

```cpp
Frame f;
if (parse(raw, rawLen, f)) {
    // f.src, f.dst, f.cmd, f.arg, f.payload_len, f.payload[]
}
```

Streaming:

```cpp
Parser parser;
Frame f;
if (feed(parser, byte, f) == PacketReady) {
    // valid stock-format frame
}
```

Build a frame:

```cpp
uint8_t raw[64];
size_t n = encode(ADDR_ESC, ADDR_DASH, CMD_DASH_IO, 0x00,
                  payload, payloadLen, raw, sizeof(raw));
```

Dashboard helpers:

```cpp
uint8_t gas, brake;
decodeDash65(frame, gas, brake);

Frame status;
buildEsc64(mode, battery, lightOn, beep, speed, error, status);
```

## Verification

`tests/golden_vectors.cpp` checks byte-for-byte against real stock captures:

- Dash -> ESC CMD65
- Dash -> ESC CMD64
- ESC -> Dash CMD64
- ARG10 request
- ARG10 serial reply
- light-on encoding `0x64`
- streaming parser

The protocol layout is also consistent with the stock DRV decompilation published in
Vicy-DE/Ninebot-G30-Custom: `LEN = payload count`, frame total `LEN + 9`.

## Scope

This library is a **wire/protocol core**, not a fake scooter application. It does not invent heartbeat
frames or generic M365 behavior. Higher-level G30 behavior must be backed by stock G30 captures or
verified G30 firmware reverse engineering.
