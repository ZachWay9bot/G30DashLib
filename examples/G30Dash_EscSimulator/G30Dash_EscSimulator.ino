/*
  G30Dash_EscSimulator v2

  Stock-format Ninebot G30 ESC-side dashboard example.
  This is intentionally protocol-only. It does not emulate pairing/auth.
*/
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
Parser parser;

static void send64(bool lightOn) {
  Frame f;
  buildEsc64(0x02, 0x00, lightOn, 0x00, 0x00, 0x00, f);
  uint8_t raw[32];
  size_t n=encode(f,raw,sizeof(raw));
  if(n) {
    BUS.write(raw,n);
    BUS.flush();
  }
}

void setup() {
  Serial.begin(115200);
  BUS.begin(115200,SERIAL_8N1,RX_PIN,TX_PIN);
  Serial.println("G30DashLib v2 stock-format example");
}

void loop() {
  while(BUS.available()) {
    uint8_t b=(uint8_t)BUS.read();
    Frame f;
    if(feed(parser,b,f)==PacketReady) {
      if(f.type==DashToEsc65) {
        uint8_t gas=0, brake=0;
        if(decodeDash65(f,gas,brake)) {
          Serial.printf("GAS=%u BRAKE=%u\n",gas,brake);
        }
      } else if(f.type==DashToEsc64) {
        // Demonstration reply only. Real application state must decide light/mode/etc.
        send64(false);
      }
    }
  }
}
