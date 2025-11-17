/*
  G30Dash_Tester

  Simple sniffer / tester for the Ninebot G30 dash bus.

  - Listens on Serial1 for 0x5A 0xA5 frames
  - Uses G30DashLib::parse() to validate and decode frames
  - Prints a readable dump to Serial
*/

#include <Arduino.h>
#include <G30DashLib.h>

using namespace G30Dash;

#ifndef RX_PIN
#define RX_PIN 9   // G30 bus RX into ESP32
#endif
#ifndef TX_PIN
#define TX_PIN 8   // G30 bus TX from ESP32 (unused here)
#endif

HardwareSerial& BUS = Serial1;

Config cfg;

static void dumpFrame(const Frame& f) {
  Serial.print(F("FRAME cmd=0x"));
  Serial.print(f.cmd, HEX);
  Serial.print(F(" len="));
  Serial.print(f.len);
  Serial.print(F(" dst=0x"));
  Serial.print(f.dst, HEX);
  Serial.print(F(" src=0x"));
  Serial.print(f.src, HEX);
  Serial.print(F(" type="));
  Serial.print(static_cast<uint8_t>(f.type));
  Serial.print(F(" crc=0x"));
  Serial.println(f.crc, HEX);

  Serial.print(F("  payload["));
  Serial.print(f.payload_len);
  Serial.println(F("]:"));

  for (uint8_t i = 0; i < f.payload_len; ++i) {
    if ((i % 16) == 0) {
      Serial.print(F("  "));
    }
    uint8_t b = f.payload[i];
    if (b < 0x10) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(' ');
    if ((i % 16) == 15 || i + 1 == f.payload_len) {
      Serial.println();
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {;}
  Serial.println();
  Serial.println(F("[G30Dash_Tester] booting…"));

  BUS.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println(F("BUS @ 115200 ready. Listening for 0x5A 0xA5…"));
}

void loop() {
  static uint8_t buf[128];
  static size_t filled = 0;

  while (BUS.available()) {
    uint8_t b = static_cast<uint8_t>(BUS.read());
    if (filled < sizeof(buf)) {
      buf[filled++] = b;
    } else {
      filled = 0;
      buf[filled++] = b;
    }

    if (filled >= 7) {
      // scan for header
      for (size_t i = 0; i + 6 < filled; ++i) {
        if (buf[i] == 0x5A && buf[i + 1] == 0xA5) {
          uint8_t L = buf[i + 2];
          size_t frameLen = static_cast<size_t>(L) + 5;
          if (i + frameLen <= filled) {
            Frame f{};
            bool ok = parse(buf + i, frameLen, f, cfg);
            if (ok) {
              dumpFrame(f);
            } else {
              Serial.println(F("Invalid frame (CRC/len mismatch)"));
            }
            // drop consumed bytes
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
