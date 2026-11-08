#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "../src/G30DashLib.h"

static int fails = 0;

static void expect(bool ok, const char* what) {
  if (!ok) {
    printf("FAIL: %s\n", what);
    ++fails;
  } else {
    printf("PASS: %s\n", what);
  }
}

static bool eq(const uint8_t* a, const uint8_t* b, size_t n) {
  return memcmp(a,b,n)==0;
}

int main() {
  using namespace G30Dash;

  const uint8_t dash65[] = {
    0x5A,0xA5,0x05,0x21,0x20,0x65,0x00,0x04,0x29,0x21,0x02,0x00,0x04,0xFF
  };
  const uint8_t dash64[] = {
    0x5A,0xA5,0x07,0x21,0x20,0x64,0x00,0x06,0x29,0x21,0x02,0x00,0x17,0x01,0xE9,0xFE
  };
  const uint8_t esc64_off[] = {
    0x5A,0xA5,0x06,0x20,0x21,0x64,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x52,0xFF
  };
  const uint8_t arg10_req[] = {
    0x5A,0xA5,0x04,0x3E,0x20,0x01,0x10,0x0E,0x02,0x29,0x21,0x32,0xFF
  };
  const uint8_t arg10_rsp[] = {
    0x5A,0xA5,0x0E,0x20,0x3E,0x04,0x10,
    0x4E,0x34,0x47,0x53,0x44,0x31,0x31,0x31,0x31,0x43,0x35,0x34,0x38,0x37,
    0x40,0xFC
  };

  Frame f;
  expect(parse(dash65,sizeof(dash65),f), "parse stock Dash->ESC CMD65");
  expect(f.len==5 && f.src==0x21 && f.dst==0x20 && f.cmd==0x65 && f.arg==0,
         "CMD65 field positions");
  uint8_t gas=0, brake=0;
  expect(decodeDash65(f,gas,brake) && gas==0x29 && brake==0x21,
         "CMD65 gas=0x29 brake=0x21");

  expect(parse(dash64,sizeof(dash64),f), "parse stock Dash->ESC CMD64");
  expect(f.len==7 && f.src==0x21 && f.dst==0x20 && f.cmd==0x64 && f.arg==0,
         "CMD64 dash field positions");

  expect(parse(esc64_off,sizeof(esc64_off),f), "parse stock ESC->Dash CMD64 off");
  expect(f.payload_len==6 && f.payload[0]==0x02 && f.payload[2]==0x00,
         "ESC64 six-byte payload");

  expect(parse(arg10_req,sizeof(arg10_req),f), "parse exact ARG10 request");
  expect(f.len==4 && f.src==0x3E && f.dst==0x20 && f.cmd==0x01 && f.arg==0x10,
         "ARG10 request fields");

  expect(parse(arg10_rsp,sizeof(arg10_rsp),f), "parse exact ARG10 response");
  expect(f.len==14 && f.src==0x20 && f.dst==0x3E && f.cmd==0x04 && f.arg==0x10,
         "ARG10 response fields");

  uint8_t out[64];
  size_t n = encode(f,out,sizeof(out));
  expect(n==sizeof(arg10_rsp) && eq(out,arg10_rsp,n),
         "re-encode ARG10 response byte-exact");

  const uint8_t serial[14] = {
    'N','4','G','S','D','1','1','1','1','C','5','4','8','7'
  };
  Frame a;
  expect(buildArg10SerialReply(serial,a), "build ARG10 serial reply");
  n=encode(a,out,sizeof(out));
  expect(n==sizeof(arg10_rsp) && eq(out,arg10_rsp,n),
         "ARG10 helper exact golden frame");

  Frame e;
  expect(buildEsc64(0x02,0x00,false,0,0,0,e), "build ESC64 light off");
  n=encode(e,out,sizeof(out));
  expect(n==sizeof(esc64_off) && eq(out,esc64_off,n),
         "ESC64 OFF exact golden frame");

  Frame on;
  expect(buildEsc64(0x02,0x00,true,0,0,0,on), "build ESC64 light on");
  n=encode(on,out,sizeof(out));
  expect(n==15 && out[2]==6 && out[3]==0x20 && out[4]==0x21 &&
         out[5]==0x64 && out[6]==0 && out[9]==0x64,
         "ESC64 ON uses stock DRV126 light byte 0x64");

  const uint8_t light_on_write[] = {
    0x5A,0xA5,0x02,0x3E,0x20,0x02,0x90,0x01,0x00,0x0C,0xFF
  };
  const uint8_t light_off_write[] = {
    0x5A,0xA5,0x02,0x3E,0x20,0x02,0x90,0x00,0x00,0x0D,0xFF
  };

  Frame w;
  expect(buildWriteU16(ADDR_APP,ADDR_ESC,REG_HEADLIGHT_IOT_SW,1,w),
         "build headlight register 0x90 ON write");
  n=encode(w,out,sizeof(out));
  expect(n==sizeof(light_on_write) && eq(out,light_on_write,n),
         "headlight 0x90 ON exact CMD02 frame");

  expect(buildWriteU16(ADDR_APP,ADDR_ESC,REG_HEADLIGHT_IOT_SW,0,w),
         "build headlight register 0x90 OFF write");
  n=encode(w,out,sizeof(out));
  expect(n==sizeof(light_off_write) && eq(out,light_off_write,n),
         "headlight 0x90 OFF exact CMD02 frame");

  Parser p;
  Frame sf;
  ParseResult rr=NeedMore;
  for(size_t i=0;i<sizeof(dash65);++i) rr=feed(p,dash65[i],sf);
  expect(rr==PacketReady && sf.cmd==0x65 && sf.payload_len==5,
         "streaming parser stock CMD65");

  printf("\nTOTAL: %s (%d failures)\n", fails ? "FAIL" : "PASS", fails);
  return fails ? 1 : 0;
}
