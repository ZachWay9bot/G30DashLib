/*
  G30Dash_EscSimulator

  Minimal fake ESC for a Ninebot G30 dash.
  - Talks 0x5A 0xA5 frames on Serial1
  - Periodically sends a 0x65 "status" frame with ramping speed
*/

#include <Arduino.h>
#include <G30DashLib.h>

using namespace G30Dash;

#ifndef RX_PIN
#define RX_PIN 9   // G30 bus RX into ESP32
#endif
#ifndef TX_PIN
#define TX_PIN 8   // G30 bus TX from ESP32
#endif

HardwareSerial& BUS = Serial1;

Config cfg;               // defaults: esc=0x20, dash=0x21, CRC_INV16
Heartbeat hb{60};         // send every 60 ms

void setup() {
  Serial.begin(115200);
  while (!Serial) {;}
  Serial.println();
  Serial.println(F("[G30Dash_EscSimulator] booting…"));

  BUS.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println(F("BUS @ 115200 ready."));
}

void loop() {
  uint32_t now = millis();

  // --- Periodic fake ESC status (0x65) ---
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

    // Very rough, simplified payload:
    //   [ driveMode, batteryPct, speedKmh, reserved ]
    const uint8_t payload[4] = {
      1,         // D-mode
      80,        // 80% battery
      speed,     // fake speed
      0x00
    };

    f.payload_len = sizeof(payload);
    f.len = 3 + f.payload_len;
    for (uint8_t i = 0; i < f.payload_len; ++i) {
      f.payload[i] = payload[i];
    }

    uint8_t raw[80];
    size_t n = encode(f, raw, sizeof(raw), cfg);
    if (n) {
      BUS.write(raw, n);
      BUS.flush();
      Serial.print(F("TX 65 speed="));
      Serial.println(speed);
    }
  }

  // --- Optional: sniff incoming frames for debugging ---
  static uint8_t buf[96];
  static size_t filled = 0;

  while (BUS.available()) {
    uint8_t b = static_cast<uint8_t>(BUS.read());
    if (filled < sizeof(buf)) {
      buf[filled++] = b;
    } else {
      // simple overflow protection: reset buffer
      filled = 0;
      buf[filled++] = b;
    }

    // Try to find a 5A A5 header in the buffer
    if (filled >= 7) {
      for (size_t i = 0; i + 6 < filled; ++i) {
        if (buf[i] == 0x5A && buf[i + 1] == 0xA5) {
          uint8_t L = buf[i + 2];
          size_t frameLen = static_cast<size_t>(L) + 5;
          if (i + frameLen <= filled) {
            Frame f{};
            if (parse(buf + i, frameLen, f, cfg)) {
              Serial.print(F("RX cmd=0x"));
              Serial.print(f.cmd, HEX);
              Serial.print(F(" len="));
              Serial.print(f.len);
              Serial.print(F(" type="));
              Serial.println(f.type);
            }
            // Drop everything up to end of this frame
            size_t remain = filled - (i + frameLen);
            memmove(buf, buf + i + frameLen, remain);
            filled = remain;
            break;
          }
        }
      }
    }
  }
}
