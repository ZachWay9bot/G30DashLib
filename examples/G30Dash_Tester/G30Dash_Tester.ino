/*
  G30Dash_Tester v2
  Passive stock-format G30 frame decoder.
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

HardwareSerial& BUS=Serial1;
Parser parser;

static void dump(const Frame& f) {
  Serial.printf("%02X->%02X CMD=%02X ARG=%02X LEN=%u :",
                f.src,f.dst,f.cmd,f.arg,f.payload_len);
  for(uint8_t i=0;i<f.payload_len;i++) Serial.printf(" %02X",f.payload[i]);
  Serial.println();

  if(f.type==DashToEsc65) {
    uint8_t gas=0,brake=0;
    if(decodeDash65(f,gas,brake))
      Serial.printf("  GAS=%u BRAKE=%u\n",gas,brake);
  }
}

void setup() {
  Serial.begin(115200);
  BUS.begin(115200,SERIAL_8N1,RX_PIN,TX_PIN);
  Serial.println("G30DashLib v2 stock-format tester");
}

void loop() {
  while(BUS.available()) {
    Frame f;
    if(feed(parser,(uint8_t)BUS.read(),f)==PacketReady) dump(f);
  }
}
