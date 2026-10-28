#!/usr/bin/env python3
"""Run actual touch driver and SweetHID event methods under ASan/UBSan.

Host programs, all built from the real sources (a third one, built for both
boards, runs fuxi_touch_init: salami starts the thread only with fuxitouch=1):
  1. FTS (fuxi): report/FIFO decoder and SweetHID event methods (1080x2400 and
     the salami 1080x2412 panel).
  2. TouchComm (salami S3908): the I2C transport (GPI TRE encoders and event
     decoding, FIFO engine), the TouchComm v1 framing, command and bring-up
     layers, the report parser, contact dispatch, coordinate mapping and the
     knob lookup, driven by a TouchComm device model with synthetic frames
     built from the source-derived formats (design section 7.2).
Only local production code is compiled. IOKit allocation, dispatch, registers,
the GPI/FIFO hardware and the S3908 are host stand-ins; no upstream source,
device, firmware or network is used.
"""
from pathlib import Path
import argparse
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'xnu/fuxi-h1'


def function(source, name):
    match = re.search(r'^(?:static |extern "C" )?(?:void|int|bool|uint64_t|uint32_t|int64_t|unsigned)\n' +
                      re.escape(name) + r'\([^;]*?\)\n\{', source, re.M)
    assert match, name
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


MODEL_HID = r'''
#include <cassert>
#include <cstdint>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include "fuxi_key_state.h"
using UInt32 = uint32_t;
using IOFixed = int32_t;
using vm_offset_t = uintptr_t;
#define TRUE 1
#define SWEET_TOUCH_FINGERS 10
#define SWEET_TOUCH_EDGE_PX 48
// Field names model the IOHIDEvent accessor boundary; serialized ABI is not modeled.
enum {
 kIOHIDEventFieldDigitizerCollection, kIOHIDEventFieldDigitizerIndex,
 kIOHIDEventFieldDigitizerIdentity, kIOHIDEventFieldDigitizerTouch,
 kIOHIDEventFieldDigitizerEventMask, kIOHIDEventFieldDigitizerX,
 kIOHIDEventFieldDigitizerY, kIOHIDEventFieldDigitizerRange,
 kIOHIDEventFieldDigitizerButtonMask
};
enum {kIOHIDDigitizerTransducerTypeHand, kIOHIDDigitizerTransducerTypeFinger};
enum {
 kIOHIDDigitizerEventRange=1, kIOHIDDigitizerEventTouch=2,
 kIOHIDDigitizerEventPosition=4, kIOHIDDigitizerEventIdentity=32,
 kIOHIDDigitizerEventFromEdgeTip=0x800
};
static uint64_t clock_now=100000;   // 1 abs unit = 1 us
static uint64_t mach_absolute_time() { return clock_now++; }
static void absolutetime_to_nanoseconds(uint64_t abs, uint64_t *ns) { *ns=abs*1000; }
static unsigned perf, wake_calls, logs;
static std::vector<std::string> loglines;
static void fuxi_perf_input() { perf++; }
static bool fuxi_keys_touch_wake() { wake_calls++;return true; }
static int model_printf(const char *format, ...) {
 char line[1024];va_list ap;va_start(ap,format);vsnprintf(line,sizeof(line),format,ap);va_end(ap);
 loglines.push_back(line);logs++;return 0;
}
#define printf model_printf
#define IOLog model_printf
static int lock_depth;
static void IOLockLock(int *) { assert(!lock_depth);lock_depth++; }
static void IOLockUnlock(int *) { assert(lock_depth==1);lock_depth--; }
struct EventData {
 uint64_t time;
 UInt32 type, transducer;
 std::map<int,int> fields;
 std::vector<EventData> children;
};
static std::vector<EventData> dispatched;
static int fail_after=-1, append_fail_after=-1, live;
class IOHIDEvent;
struct OSArray {
 std::vector<IOHIDEvent *> objects;
 unsigned getCount() { return objects.size(); }
};
class IOHIDEvent {
 int refs=1;
 OSArray children;
public:
 EventData data;
 IOHIDEvent() { live++; }
 ~IOHIDEvent() { for(auto *c:children.objects)c->release();live--; }
 void release() { if(!--refs)delete this; }
 void appendChild(IOHIDEvent *child) {
  if(append_fail_after==0)return;
  if(append_fail_after>0)append_fail_after--;
  child->refs++;children.objects.push_back(child);
 }
 OSArray *getChildren() { return children.objects.empty()?nullptr:&children; }
 void setIntegerValue(int field, int value) { data.fields[field]=value; }
 void setFixedValue(int field, IOFixed value) { data.fields[field]=value; }
 EventData snapshot() {
  auto result=data;
  for(auto *c:children.objects)result.children.push_back(c->snapshot());
  return result;
 }
 static IOHIDEvent *digitizerEvent(uint64_t time, UInt32 id, UInt32 type,
                                 bool range, UInt32 buttons, IOFixed x, IOFixed y) {
  if(fail_after==0)return nullptr;
  if(fail_after>0)fail_after--;
  auto *event=new IOHIDEvent;
  event->data.time=time;event->data.type=type;event->data.transducer=id;
  event->setIntegerValue(kIOHIDEventFieldDigitizerRange,range);
  event->setIntegerValue(kIOHIDEventFieldDigitizerButtonMask,buttons);
  event->setFixedValue(kIOHIDEventFieldDigitizerX,x);
  event->setFixedValue(kIOHIDEventFieldDigitizerY,y);
  return event;
 }
};
class SweetHIDTouch {
public:
 int storage=0, *fLock=&storage;
 int fLogicalW=1125, fLogicalH=2436, fOffX=22, fOffY=18;
 bool fDown[SWEET_TOUCH_FINGERS]={}, fEdge[SWEET_TOUCH_FINGERS]={};
 IOFixed fX[SWEET_TOUCH_FINGERS]={}, fY[SWEET_TOUCH_FINGERS]={};
 void touch(int finger, bool down, int x, int y);
 bool report(UInt32 mask, const int *x, const int *y);
 void dispatchEvent(IOHIDEvent *e) { assert(!lock_depth);dispatched.push_back(e->snapshot()); }
};
static SweetHIDTouch service;
static SweetHIDTouch *gSweetTouch=&service;
uint64_t sweet_hid_last_input;
[[maybe_unused]] static int field(const EventData &e, int key) { return e.fields.at(key); }
[[maybe_unused]] static bool logged(const char *needle) {
 for(auto &l:loglines)if(l.find(needle)!=std::string::npos)return true;
 return false;
}
'''

MODEL_FTS = r'''
#define FT_MAX 10u
#define FT_EV_READY 0x03u
#define FT_EV_ENTER 0x13u
#define FT_EV_MOTION 0x23u
#define FT_EV_LEAVE 0x33u
#define FT_TYPE_HOVER 0x05u
static std::vector<uint8_t> fifo;
static unsigned reads, fail_read;
static int ft_read(const uint8_t *cmd, unsigned nc, uint8_t *out, unsigned n) {
 assert(nc==1 && cmd[0]==0x87);reads++;
 if(reads==fail_read)return 5;
 assert(fifo.size()>=n);memcpy(out,fifo.data(),n);fifo.erase(fifo.begin(),fifo.begin()+n);
 return 0;
}
'''

TEST = r'''
static void reset(bool batch=true) {
 assert(!live && !lock_depth);
 ft={};ft.batch=batch;service=SweetHIDTouch();service.fLock=&service.storage;
 dispatched.clear();fifo.clear();reads=fail_read=0;
 fail_after=append_fail_after=-1;perf=logs=wake_calls=0;
}
static const EventData &child(unsigned id) {
 for(auto &c:dispatched.back().children)
  if(field(c,kIOHIDEventFieldDigitizerIndex)==(int)id)return c;
 assert(false);return dispatched.back();
}
static void check(UInt32 down, UInt32 lifts) {
 assert(!dispatched.empty() && !live && !lock_depth);
 auto &e=dispatched.back();
 assert(e.type==kIOHIDDigitizerTransducerTypeHand);
 assert(field(e,kIOHIDEventFieldDigitizerCollection)==1);
 assert(field(e,kIOHIDEventFieldDigitizerTouch)==(down!=0));
 assert(field(e,kIOHIDEventFieldDigitizerRange)==(down!=0));
 assert(field(e,kIOHIDEventFieldDigitizerButtonMask)==(down!=0));
 UInt32 seen=0, merged=0;
 for(auto &c:e.children) {
  unsigned id=field(c,kIOHIDEventFieldDigitizerIndex);
  assert(id<10 && !(seen&(1u<<id)));seen|=1u<<id;
  assert(c.time==e.time && c.type==kIOHIDDigitizerTransducerTypeFinger);
  assert(c.transducer==id+2 && field(c,kIOHIDEventFieldDigitizerIdentity)==(int)id+2);
  assert(field(c,kIOHIDEventFieldDigitizerTouch)==bool(down&(1u<<id)));
  assert(field(c,kIOHIDEventFieldDigitizerRange)==bool(down&(1u<<id)));
  if(lifts&(1u<<id))assert(field(c,kIOHIDEventFieldDigitizerEventMask)&kIOHIDDigitizerEventTouch);
  merged|=field(c,kIOHIDEventFieldDigitizerEventMask);
 }
 assert(seen==(down|lifts));
 assert(field(e,kIOHIDEventFieldDigitizerEventMask)==(int)merged);
}
static void event(unsigned kind, unsigned id, unsigned type, unsigned x, unsigned y) {
 unsigned xx=x*10, yy=y*10;
 uint8_t e[8]={(uint8_t)kind,(uint8_t)((id<<4)|type),(uint8_t)xx,(uint8_t)(xx>>8),
               (uint8_t)yy,(uint8_t)(yy>>8),0,0};
 fifo.insert(fifo.end(),e,e+8);
}
static int poll(bool deliver=true) {
 if(fifo.empty())fifo.resize(8);
 fifo[7]=(uint8_t)(fifo.size()/8-1);reads=0;
 int error=ft_poll(deliver);
 if(!error)assert(fifo.empty());
 return error;
}
static void batching() {
 reset();
 ft.x[2]=400;ft.y[2]=700;ft.x[7]=600;ft.y[7]=900;
 ft_report((1u<<2)|(1u<<7));check((1u<<2)|(1u<<7),0);
 assert(dispatched.size()==1 && ft.c_multi==1 && ft.c_max==2 && !ft.c_lifts);
 // Both move: one report, with simultaneous new positions.
 ft.x[2]+=10;ft.x[7]+=20;ft_report(ft.active);check(ft.active,0);
 assert(dispatched.size()==2 && ft.c_multi==2);
 assert(field(child(2),kIOHIDEventFieldDigitizerX)==((410+22)*65536/1125));
 assert(field(child(7),kIOHIDEventFieldDigitizerX)==((620+22)*65536/1125));
 int oldx=field(child(2),kIOHIDEventFieldDigitizerX);
 ft.x[2]=0;ft.y[2]=0;ft_report(1u<<7);check(1u<<7,1u<<2);
 assert(field(child(2),kIOHIDEventFieldDigitizerX)==oldx);
 assert(!(field(child(7),kIOHIDEventFieldDigitizerEventMask)&kIOHIDDigitizerEventTouch));
 assert(ft.c_lifts==1 && ft.c_max==2);
 ft_report(0);check(0,1u<<7);
 assert(ft.contacts==1 && ft.c_lifts==2 && ft.c_reports==4);
 unsigned count=dispatched.size();ft_report(0);assert(dispatched.size()==count);
 // Same count, entirely different identities: lift old and introduce new.
 ft_report((1u<<1)|(1u<<9));check(ft.active,0);
 ft_report((1u<<4)|(1u<<8));check(ft.active,(1u<<1)|(1u<<9));
 assert(ft.c_lifts==2 && ft.c_multi==2 && ft.c_max==2);
 ft_report(0);check(0,(1u<<4)|(1u<<8));
 // Ten fingers, and ten simultaneous lifts, without a packed-report capacity limit.
 ft_report(0x3ff);check(0x3ff,0);
 assert(dispatched.back().children.size()==10 && ft.c_max==10);
 ft_report(0);check(0,0x3ff);assert(ft.c_lifts==10);
 // Half the hardware IDs replaced at a constant five fingers down.
 ft_report(0x155);ft_report(0x2aa);check(0x2aa,0x155);
 assert(dispatched.back().children.size()==10 && ft.c_lifts==5);
 ft_report(0);
 puts("PASS: batched movement, ID-based partial/final/replacement lifts, ten fingers and report counters");
}
static void coordinates_edges_and_legacy() {
 reset();ft.x[0]=0;ft.y[0]=2399;ft_report(1);check(1,0);
 assert(field(child(0),kIOHIDEventFieldDigitizerEventMask)&kIOHIDDigitizerEventFromEdgeTip);
 ft.x[0]=500;ft.y[0]=1000;ft_report(1);
 assert(field(child(0),kIOHIDEventFieldDigitizerEventMask)&kIOHIDDigitizerEventFromEdgeTip);
 ft_report(0);assert(field(child(0),kIOHIDEventFieldDigitizerEventMask)&kIOHIDDigitizerEventFromEdgeTip);
 ft_report(1);assert(!(field(child(0),kIOHIDEventFieldDigitizerEventMask)&kIOHIDDigitizerEventFromEdgeTip));
 unsigned count=dispatched.size();ft_report(1);assert(dispatched.size()==count); // unchanged suppression
 ft_report(0);
 service.fLogicalW=1080;service.fLogicalH=2400;service.fOffX=service.fOffY=0;
 ft.x[5]=-100;ft.y[5]=3000;ft_report(1u<<5);check(1u<<5,0);
 assert(field(child(5),kIOHIDEventFieldDigitizerX)==0);
 assert(field(child(5),kIOHIDEventFieldDigitizerY)==2399*65536/2400);
 ft_report(0);
 // Absent/zero option keeps per-finger dispatch, same ID state and counters.
 reset(false);assert(!ft.batch);
 ft.x[2]=400;ft.y[2]=700;ft.x[7]=600;ft.y[7]=900;
 ft_report((1u<<2)|(1u<<7));assert(dispatched.size()==2);check(ft.active,0);
 ft.x[2]++;ft.x[7]++;ft_report(ft.active);assert(dispatched.size()==4);
 ft_report(1u<<7);assert(dispatched.size()==5);check(1u<<7,1u<<2);
 ft_report(0);assert(dispatched.size()==6);check(0,1u<<7);
 assert(ft.c_multi==2 && ft.c_max==2 && ft.c_lifts==2);
 puts("PASS: centered/native mapping, clamp, edge lifetime, unchanged suppression and legacy default");
}
static void salami_panel() {
 // salami 1080x2412: logical 1125x2436 centred with offY 12; last row 2411, edge band from 2363.
 reset();service.fOffY=12;
 ft.x[0]=500;ft.y[0]=3000;ft_report(1);check(1,0);
 assert(field(child(0),kIOHIDEventFieldDigitizerY)==((2411+12)*65536/2436));
 assert(field(child(0),kIOHIDEventFieldDigitizerEventMask)&kIOHIDDigitizerEventFromEdgeTip);
 ft_report(0);ft.y[0]=2363;ft_report(1);
 assert(!(field(child(0),kIOHIDEventFieldDigitizerEventMask)&kIOHIDDigitizerEventFromEdgeTip));
 ft_report(0);ft.y[0]=2364;ft_report(1);
 assert(field(child(0),kIOHIDEventFieldDigitizerEventMask)&kIOHIDDigitizerEventFromEdgeTip);
 ft_report(0);
 service.fLogicalW=1080;service.fLogicalH=2412;service.fOffX=service.fOffY=0;
 ft.x[3]=1079;ft.y[3]=2411;ft_report(1u<<3);check(1u<<3,0);
 assert(field(child(3),kIOHIDEventFieldDigitizerY)==2411*65536/2412);
 ft_report(0);
 puts("PASS: salami 1080x2412 panel: offY 12, clamp to row 2411, edge band and native mode");
}
static void allocation_failures() {
 for(int failure=0;failure<4;failure++) {
  reset();ft_report(7);unsigned count=dispatched.size();
  fail_after=failure;ft_report(0);
  assert(dispatched.size()==count && !live && !lock_depth && ft.retry);
  for(int id=0;id<3;id++)assert(service.fDown[id]);
  unsigned reports=ft.c_reports, lifts=ft.c_lifts;
  ft_retry_report();assert(ft.retry && dispatched.size()==count);
  fail_after=-1;ft_retry_report();check(0,7);
  assert(!ft.retry && ft.c_reports==reports && ft.c_lifts==lifts);
 }
 for(int failure=0;failure<3;failure++) {
  reset();append_fail_after=failure;ft_report(7);
  assert(dispatched.empty() && !live && !lock_depth && ft.retry);
  for(bool down:service.fDown)assert(!down);
  append_fail_after=-1;ft_retry_report();check(7,0);assert(!ft.retry);
 }
 reset();gSweetTouch=nullptr;
 ft_report(1);assert(sweet_hid_last_input && dispatched.empty() && ft.retry);
 // A failed down superseded by up must not become a stale delayed down.
 gSweetTouch=&service;ft_report(0);ft_retry_report();assert(dispatched.empty() && !ft.retry);
 // A new ID reconciles a failed previous final-up against the HID ID state.
 reset();ft_report(1);fail_after=0;ft_report(0);assert(ft.retry);
 fail_after=-1;ft_report(1u<<7);check(1u<<7,1);assert(!ft.retry);
 ft_report(0);
 puts("PASS: allocation/append failures publish no partial set, retry final lifts, latest-set replacement and last-input entry point");
}
static void fifo_tests() {
 reset();
 event(FT_EV_ENTER,2,1,400,700);event(FT_EV_ENTER,7,1,600,900);
 assert(!poll() && reads==2 && dispatched.size()==1);check(ft.active,0);
 // A drain is a delta stream: no new events cannot lift stationary contacts.
 assert(!poll() && ft.active==((1u<<2)|(1u<<7)) && dispatched.size()==1);
 event(FT_EV_MOTION,7,1,650,950);event(FT_EV_LEAVE,2,0,0,0);
 assert(!poll() && dispatched.size()==2);check(1u<<7,1u<<2);
 // Hover becomes a lift; type zero without LEAVE and invalid IDs are ignored.
 event(FT_EV_MOTION,7,FT_TYPE_HOVER,0,0);assert(!poll());check(0,1u<<7);
 unsigned count=dispatched.size();
 event(FT_EV_ENTER,1,0,100,100);event(FT_EV_ENTER,15,1,100,100);
 assert(!poll() && dispatched.size()==count);
 event(FT_EV_ENTER,3,1,100,100);assert(!poll(false) && dispatched.size()==count);
 event(FT_EV_LEAVE,3,0,0,0);assert(!poll(false));
 event(FT_EV_ENTER,3,1,100,100);assert(!poll());check(1u<<3,0);
 event(FT_EV_READY,0,0,0,0);assert(!poll() && ft.ic_reset);
 ft_report(0);check(0,1u<<3); // thread's reset/display-off/error release call
 // Failed first/second reads do not apply half a batch.
 for(unsigned failure=1;failure<=2;failure++) {
  reset();event(FT_EV_ENTER,2,1,400,700);event(FT_EV_ENTER,7,1,600,900);
  fail_read=failure;assert(poll()==5 && !ft.active && dispatched.empty());
 }
 // Tap-to-wake's suppressed IDs remain hidden until they lift.
 reset();
 event(FT_EV_ENTER,4,1,500,1000);assert(!poll(false) && wake_calls==1 && ft.wake.woke);
 event(FT_EV_MOTION,4,1,500,900);assert(!poll() && dispatched.empty());
 event(FT_EV_LEAVE,4,0,0,0);assert(!poll() && dispatched.empty());
 event(FT_EV_ENTER,4,1,500,1000);assert(!poll());check(1u<<4,0);
 ft_report(0);
 puts("PASS: actual two-read FIFO decoder, ID/type/hover handling, reset release, read failures and wake suppression");
}
static void randomized_sets() {
 reset();uint32_t seed=12345, previous=0;
 for(unsigned n=0;n<2000;n++) {
  seed=seed*1664525+1013904223;uint32_t mask=(seed>>16)&0x3ff;
  for(unsigned id=0;id<10;id++) {
   seed=seed*1664525+1013904223;ft.x[id]=(int)(seed%1300)-100;
   seed=seed*1664525+1013904223;ft.y[id]=(int)(seed%2700)-100;
  }
  unsigned count=dispatched.size();ft_report(mask);
  if(dispatched.size()>count)check(mask,previous&~mask);
  for(unsigned id=0;id<10;id++)assert(service.fDown[id]==bool(mask&(1u<<id)));
  previous=mask;
 }
 ft_report(0);if(previous)check(0,previous);
 puts("PASS: 2,000 randomized full contact sets, ID uniqueness, masks, lifts and bounded coordinates");
}
int main() {
 batching();coordinates_edges_and_legacy();salami_panel();allocation_failures();fifo_tests();randomized_sets();
 assert(!live && !lock_depth);
}
'''

# ---------------------------------------------------------------------------
# TouchComm (S3908) program: hardware and device stand-ins.
# ---------------------------------------------------------------------------
TCM_MODEL = r'''
#include <deque>
#include <array>
#define bzero(p, n) memset((p), 0, (n))
#define PAGE_SIZE 16384u
#define NSEC_PER_MSEC 1000000ull
using addr64_t = uint64_t;
static void flush_dcache64(addr64_t, unsigned, int) {}
static void dev_tick();
static unsigned sleeps_ms;
static void delay(int us) { clock_now+=(uint64_t)us;dev_tick(); }
static void ft_sleep(unsigned ms) { clock_now+=ms*1000ull;sleeps_ms+=ms;dev_tick(); }
struct PE_Video {
 unsigned long v_baseAddr, v_rowBytes, v_width, v_height, v_depth, v_display;
 char v_pixelFormat[64];
 unsigned long v_offset, v_length;
 unsigned char v_rotate, v_scale;
 char reserved1[2];
 long reserved2;
};
static unsigned long console_w=1080, console_h=2412;
static int PE_current_console(PE_Video *v) {
 v->v_width=console_w;v->v_height=console_h;v->v_baseAddr=console_w?0x9c000000ul:0;return 0;
}
// Boot-args: xnu-8796.141.3 pexpert/gen/bootargs.c rules (exact names, '=' value,
// 0x hex, 0b binary, leading-0 octal, k/m/g multipliers, sign, string copy, last wins).
static std::string bootline;
static std::map<std::string,std::vector<uint8_t>> adt;
static bool isargsep(char c) { return c==' ' || c=='\0' || c=='\t'; }
enum {NUM, STR};
static int getval(const char *s, long long *val) {
 unsigned long long radix, intval;
 unsigned char c;
 int sign=1;
 bool has_value=false;
 if(*s=='=') { s++;has_value=true; }
 if(has_value) {
  if(*s=='-') { sign=-1;s++; }
  intval=(unsigned long long)(*s++ - '0');
  radix=10;
  if(intval==0) {
   switch(*s) {
   case 'x': radix=16;s++;break;
   case 'b': radix=2;s++;break;
   case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7':
    intval=(unsigned long long)(*s-'0');s++;radix=8;break;
   default: if(!isargsep(*s))return STR;
   }
  } else if(intval>=radix) {
   return STR;
  }
  for(;;) {
   c=(unsigned char)*s++;
   if(isargsep((char)c))break;
   if(radix<=10 && c>='0' && c<=('9'-(10-radix)))c-='0';
   else if(radix==16 && c>='0' && c<='9')c-='0';
   else if(radix==16 && c>='a' && c<='f')c-='a'-10;
   else if(radix==16 && c>='A' && c<='F')c-='A'-10;
   else if(c=='k' || c=='K') { sign*=1024;break; }
   else if(c=='m' || c=='M') { sign*=1024*1024;break; }
   else if(c=='g' || c=='G') { sign*=1024*1024*1024;break; }
   else return STR;
   if(c>=radix)return STR;
   intval*=radix;intval+=c;
  }
  if(!isargsep((char)c) && !isargsep(*s))return STR;
  *val=(long long)(intval*(unsigned long long)(long long)sign);
  return NUM;
 }
 *val=1;
 return NUM;
}
static void argnumcpy(long long v, void *to, int maxlen) {
 switch(maxlen) {
 case 1: *(int8_t *)to=(int8_t)v;break;
 case 2: *(int16_t *)to=(int16_t)v;break;
 case 8: *(int64_t *)to=(int64_t)v;break;
 default: *(int32_t *)to=(int32_t)v;break;
 }
}
static void argstrcpy2(const char *from, char *to, int maxlen) {
 int i=0;
 while(!isargsep(*from) && i<maxlen) { i++;*to++=*from++; }
 *to=0;
}
static bool PE_parse_boot_argn(const char *name, void *ptr, int maxlen) {
 const char *args=bootline.c_str();
 bool found=false;
 while(*args) {
  const char *cp=args;
  while(!isargsep(*cp) && *cp!='=')cp++;
  char c=*cp;
  size_t i=(size_t)(cp-args);
  if(!strncmp(args,name,i) && i==strlen(name)) {
   while(*cp && isargsep(*cp))cp++;
   if(!(*cp=='=' && c!='=')) {
    long long val=0;
    if(getval(cp,&val)==NUM) argnumcpy(val,ptr,maxlen);
    else if(*cp=='=') argstrcpy2(cp+1,(char *)ptr,maxlen-1);
    else argstrcpy2("1",(char *)ptr,maxlen);
    found=true;
   }
  }
  while(!isargsep(*args))args++;
  while(*args && isargsep(*args))args++;
 }
 return found;
}
static bool PE_get_default(const char *name, void *ptr, unsigned max) {
 auto it=adt.find(name);
 if(it==adt.end() || it->second.size()>max)return false;
 memcpy(ptr,it->second.data(),it->second.size());return true;
}
static std::vector<uint8_t> u32le(uint32_t v) { return {(uint8_t)v,(uint8_t)(v>>8),(uint8_t)(v>>16),(uint8_t)(v>>24)}; }

// ---------------- S3908 TouchComm v1 device model ----------------
static std::vector<uint8_t> msg(unsigned code, const std::vector<uint8_t> &pl) {
 std::vector<uint8_t> m={0xa5,(uint8_t)code,(uint8_t)pl.size(),(uint8_t)(pl.size()>>8)};
 m.insert(m.end(),pl.begin(),pl.end());m.push_back(0x5a);return m;
}
using Msgs=std::vector<std::vector<uint8_t>>;
struct Dev {
 unsigned addr=0x4b;
 std::deque<std::vector<uint8_t>> q;
 std::vector<uint8_t> rest;
 std::vector<std::vector<uint8_t>> writes, cmds;
 std::vector<unsigned> rsizes;
 std::vector<uint8_t> partial;
 unsigned need=0;
 std::map<unsigned,std::deque<Msgs>> resp;   // per command: answers in order, the last one repeats
 int nack_r=0, nack_w=0;
 bool garbage=false, in_reset=false;
 size_t garbage_read=0;   // corrupt the n-th read (1-based) since rsizes was cleared
 std::vector<uint8_t> boot_msg, boot;   // boot_msg is queued boot_ms after every reset release
 uint64_t boot_at=0;
 unsigned boot_ms=40;
 bool pending() const { return !q.empty() || !rest.empty(); }
 int read(uint8_t *out, unsigned n) {
  rsizes.push_back(n);
  if(nack_r) { nack_r--;return -1; }
  std::vector<uint8_t> src;
  if(!rest.empty()) {
   src={0xa5,0x03};size_t k=std::min<size_t>(rest.size(),n-2);
   src.insert(src.end(),rest.begin(),rest.begin()+k);rest.erase(rest.begin(),rest.begin()+k);
  } else if(!q.empty()) {
   src=q.front();q.pop_front();
   if(src.size()>n)rest.assign(src.begin()+n,src.end());
  } else {
   src={0xa5,0x00,0x00,0x00};   // assumption: idle read = A5 00 00 00 + 0x00 filler
  }
  if(rest.size()<=1)rest.clear(); // an unread final pad byte is dropped
  for(unsigned i=0;i<n;i++)out[i]=i<src.size()?src[i]:0;
  if(garbage || (garbage_read && rsizes.size()==garbage_read)) { garbage=false;memset(out,0xff,n); }
  return 0;
 }
 void respond(unsigned c) {
  auto &l=resp[c];
  if(l.empty()) { q.push_back(msg(0x01,{}));return; }
  Msgs r=l.front();if(l.size()>1)l.pop_front();
  for(auto &m:r)q.push_back(m);
 }
 int write(const uint8_t *in, unsigned n) {
  writes.emplace_back(in,in+n);
  if(nack_w) { nack_w--;return -1; }
  if(need) {
   assert(in[0]==0x01);partial.insert(partial.end(),in+1,in+n);
  } else {
   if(n<3)return 0;
   partial.assign(in,in+n);need=3u+(in[1]|(in[2]<<8));
  }
  assert(partial.size()<=need);
  if(partial.size()==need) { cmds.push_back(partial);need=0;respond(partial[0]); }
  return 0;
 }
};
static Dev dev;
static void dev_tick() { if(!dev.boot.empty() && clock_now>=dev.boot_at) { dev.q.push_back(dev.boot);dev.boot.clear(); } }

// ---------------- registers: TLMM, SE (FIFO engine), GPI, SMMU, GCC ----------------
enum : uintptr_t { B_TLMM=0x10000000, B_SE=0x20000000, B_GPI=0x30000000, B_SMMU=0x40000000, B_GCC=0x50000000 };
static uint8_t *dma;
static const uint64_t DMA_PA=0x880000000ull;
static std::map<uint32_t,uint32_t> tlmm, se, smmu, gcc;
static std::vector<std::pair<uint32_t,uint32_t>> tlmm_w;
static unsigned attn_pin=25, rst_pin=24;
static uint32_t gp_len;
struct Fifo {
 bool busy=false;
 unsigned op=0, n=0, depth=16;
 std::vector<uint8_t> tx;
 std::deque<uint32_t> rx, pend;
 uint32_t irq=0;
 std::vector<uint32_t> cmd0, ctrl;
 int nack=0, hang=0, stubborn=0;
};
static Fifo fm;
static void fifo_fill() {
 while(!fm.pend.empty() && fm.rx.size()<fm.depth) { fm.rx.push_back(fm.pend.front());fm.pend.pop_front(); }
 if(fm.busy && fm.op==2 && fm.pend.empty() && !fm.hang) { fm.irq|=1u;fm.busy=false; }
}
static void fifo_cmd(uint32_t v) {
 fm.cmd0.push_back(v);fm.op=v>>27;fm.busy=true;fm.tx.clear();
 assert(((v>>9)&0x7f)==dev.addr);
 fm.n=se[fm.op==2?0x270u:0x26cu];
 if(fm.nack) { fm.nack--;fm.irq|=(1u<<10)|1u;fm.busy=false;return; }
 if(fm.hang)return;
 if(fm.op==2) {
  std::vector<uint8_t> b(fm.n+4,0);
  if(dev.read(b.data(),fm.n)) { fm.irq|=(1u<<10)|1u;fm.busy=false;return; }
  for(unsigned i=0;i<fm.n;i+=4)fm.pend.push_back(b[i]|(b[i+1]<<8)|(b[i+2]<<16)|((uint32_t)b[i+3]<<24));
  fifo_fill();
 }
}
static void fifo_tx(uint32_t w) {
 assert(fm.busy && fm.op==1);
 for(unsigned j=0;j<4 && fm.tx.size()<fm.n;j++)fm.tx.push_back((uint8_t)(w>>(8*j)));
 if(fm.tx.size()==fm.n && !fm.hang) {
  if(dev.write(fm.tx.data(),fm.n))fm.irq|=(1u<<10)|1u;else fm.irq|=1u;
  fm.busy=false;
 }
}
struct Gpi {
 unsigned st_ch[2]={0,0}, st_ev=0, rp[2]={0,0}, rxw=0, ev_wp=0, allocs=0;
 std::vector<std::array<uint32_t,4>> tre[2];
 std::vector<unsigned> db;
 std::map<uint32_t,uint32_t> reg;
 int style=1, drop=0, benign=0, arb=0;
};
static Gpi gpi;
static std::array<uint32_t,4> tre_at(unsigned k, unsigned i) {
 const uint32_t *t=(const uint32_t *)(dma+0x1000+0x400*k+16*i);return {t[0],t[1],t[2],t[3]};
}
static void gpi_event(uint32_t d0, uint32_t d2, uint32_t d3) {
 uint32_t *e=(uint32_t *)(dma+16*gpi.ev_wp);e[0]=d0;e[1]=0;e[2]=d2;e[3]=d3;gpi.ev_wp=(gpi.ev_wp+1)%256;
}
static void gpi_fail(unsigned chid) {
 if(gpi.arb) { gpi.arb--;gpi_event(1u<<13,0,0x31u<<16);return; }
 if(gpi.style==1)gpi_event(0,16u<<24,(chid<<24)|(0x22u<<16)|0x40u);
 else gpi_event(1u<<10,0,0x31u<<16);
}
static void gpi_doorbell(unsigned k, uint32_t v) {
 gpi.db.push_back(k);
 unsigned idx=(v-((uint32_t)DMA_PA+0x1000u+0x400u*k))/16;
 assert(idx<64);
 if(k==1) { gpi.rxw=idx;return; }
 while(gpi.rp[0]!=idx) {
  auto t=tre_at(0,gpi.rp[0]);gpi.rp[0]=(gpi.rp[0]+1)%64;gpi.tre[0].push_back(t);
  unsigned type=(t[3]>>16)&0xff;
  if(type==0x22)continue;                       // CONFIG0
  assert(type==0x20);                           // GO
  unsigned op=t[0]&0xff;
  assert(((t[0]>>8)&0x7f)==dev.addr);
  if(op==1) {
   assert(gpi.rp[0]!=idx);
   auto d=tre_at(0,gpi.rp[0]);gpi.rp[0]=(gpi.rp[0]+1)%64;gpi.tre[0].push_back(d);
   assert(((d[3]>>16)&0xff)==0x10);
   uint64_t pa=d[0]|((uint64_t)d[1]<<32);unsigned n=d[2]&0xffffff;
   if(gpi.drop) { gpi.drop--;continue; }
   if(gpi.benign) { gpi.benign--;gpi_event(1u<<6,0,0x31u<<16); }
   if(gpi.arb || dev.write(dma+(pa-DMA_PA),n))gpi_fail(0);
   else gpi_event(0,(2u<<24)|n,0x22u<<16);
  } else {
   assert(op==2 && gpi.rp[1]!=gpi.rxw);
   auto d=tre_at(1,gpi.rp[1]);gpi.rp[1]=(gpi.rp[1]+1)%64;gpi.tre[1].push_back(d);
   unsigned n=t[2];assert((d[2]&0xffffff)==n);
   uint64_t pa=d[0]|((uint64_t)d[1]<<32);
   if(gpi.drop) { gpi.drop--;continue; }
   if(t[3]&0x100)gpi_event(0,4u<<24,0x22u<<16);   // EOB on chid 0 (GO with IEOB)
   if(gpi.benign) { gpi.benign--;gpi_event(1u<<6,0,0x31u<<16); }
   if(gpi.arb || dev.read(dma+(pa-DMA_PA),n))gpi_fail(1);
   else gpi_event(0,(2u<<24)|n,(1u<<24)|(0x22u<<16));
  }
 }
}
static uint32_t ft_rd(vm_offset_t base, uint32_t off) {
 switch(base) {
 case B_TLMM:
  dev_tick();
  if(off==attn_pin*0x1000u+4)return dev.pending()?0u:1u;
  return tlmm[off];
 case B_SE:
  if(off==0x040)return fm.busy?1u:0u;
  if(off==0x610) { fifo_fill();return fm.irq|((fm.busy && fm.op==1 && se[0x80c])?(1u<<30):0u); }
  if(off==0x804) { fifo_fill();return (uint32_t)fm.rx.size(); }
  if(off==0x780) { assert(!fm.rx.empty());uint32_t w=fm.rx.front();fm.rx.pop_front();return w; }
  if(off==0x800)return 0;
  if(off==0x910)return gp_len;
  return se[off];
 case B_GPI:
  if(off==0x0000 || off==0x0080)return (gpi.reg[off]&~0xf00000u)|(gpi.st_ch[off/0x80]<<20);
  if(off==0x1000)return gpi.st_ev<<20;
  if(off==0x1010)return (uint32_t)DMA_PA+16*gpi.ev_wp;
  return gpi.reg[off];
 case B_SMMU: return smmu[off];
 case B_GCC: return gcc[off];
 default: assert(false);return 0;
 }
}
static void ft_wr(vm_offset_t base, uint32_t off, uint32_t v) {
 switch(base) {
 case B_TLMM:
  tlmm_w.push_back({off,v});
  if(off==rst_pin*0x1000u+4) {
   if(!(v&2u))dev.in_reset=true;
   else if(dev.in_reset) { dev.in_reset=false;dev.q.clear();dev.rest.clear();if(!dev.boot_msg.empty()) { dev.boot=dev.boot_msg;dev.boot_at=clock_now+dev.boot_ms*1000ull; } }
  }
  tlmm[off]=v;return;
 case B_SE:
  se[off]=v;
  if(off==0x600)fifo_cmd(v);
  else if(off==0x700)fifo_tx(v);
  else if(off==0x618)fm.irq&=~v;
  else if(off==0x604) {
   fm.ctrl.push_back(v);
   if(v==4 && !fm.stubborn) { fm.irq|=1u<<4;fm.busy=false;fm.pend.clear(); }
   if(v==2) { fm.irq|=1u<<5;fm.busy=false;fm.pend.clear(); }
  }
  return;
 case B_GPI:
  if(off==0x3008) {
   unsigned cmd=v>>24, ch=v&0xff;assert(ch<2);
   if(cmd==0) { gpi.st_ch[ch]=1;gpi.rp[ch]=0;gpi.allocs+=ch==0; }
   else if(cmd==1)gpi.st_ch[ch]=2;
   else if(cmd==2)gpi.st_ch[ch]=3;
   else if(cmd==9)gpi.st_ch[ch]=1;
   else if(cmd==10)gpi.st_ch[ch]=0;
   return;
  }
  if(off==0x3010) { unsigned cmd=v>>24;gpi.st_ev=cmd==10?0u:1u;if(cmd==0)gpi.ev_wp=0;return; }
  if(off==0x2000 || off==0x2008) { gpi_doorbell(off==0x2008,v);return; }
  gpi.reg[off]=v;return;
 case B_SMMU: smmu[off]=v;return;
 case B_GCC: gcc[off]=v;return;
 default: assert(false);
 }
}
'''

TCM_TEST = r'''
static int tcm_dispatch(bool deliver);
'''

TCM_MAIN = r'''
// ---------------- fixtures (synthetic, design 7.2) ----------------
static const std::vector<uint8_t> CFG_OPLUS={0x10,0x08,0xc6,0x30,0x01,0x06,0x04,0x07,0x04,0x08,
 0x10,0x09,0x10,0x0b,0x0c,0x0c,0x0c,0xcb,0x20,0x03,0x00};
static const std::vector<uint8_t> CFG_MIN={0x01,0x06,0x04,0x07,0x04,0x08,0x10,0x09,0x10,0x03,0x00};
static std::vector<uint8_t> identify(unsigned mode, unsigned maxwr=0) {
 std::vector<uint8_t> p={0x01,(uint8_t)mode};
 const char *part="S3908-SYNTHETIC";
 for(unsigned i=0;i<16;i++)p.push_back(i<strlen(part)?(uint8_t)part[i]:0);
 auto b=u32le(0x00123456);p.insert(p.end(),b.begin(),b.end());
 p.push_back((uint8_t)maxwr);p.push_back((uint8_t)(maxwr>>8));
 return p;
}
static void le16(std::vector<uint8_t> &p, unsigned v) { p.push_back((uint8_t)v);p.push_back((uint8_t)(v>>8)); }
static std::vector<uint8_t> app_info(unsigned status, unsigned maxx=5759, unsigned maxy=12863, unsigned cfgmax=128) {
 std::vector<uint8_t> p;
 le16(p,1);le16(p,status);le16(p,0);le16(p,0);le16(p,0);le16(p,0);le16(p,cfgmax);le16(p,127);
 const char *id="AA513008";
 for(unsigned i=0;i<16;i++)p.push_back(i<8?(uint8_t)id[i]:0);
 le16(p,maxx);le16(p,maxy);le16(p,10);le16(p,0);le16(p,17);le16(p,38);le16(p,0);
 assert(p.size()==46);return p;
}
struct Obj { unsigned id, cls, x, y; };
static std::vector<uint8_t> touch_oplus(const std::vector<Obj> &objs) {
 std::vector<uint8_t> p={0,0,0,0,0,0,0};   // gesture id + 6 bytes gesture info
 for(auto &o:objs) {
  p.push_back((uint8_t)(o.id|(o.cls<<4)));le16(p,o.x);le16(p,o.y);
  p.push_back(0x34);p.push_back(0x12);p.push_back(0x05);    // x/y width 12+12 bits
  p.push_back(0x00);p.push_back(0x03);p.push_back(0x00);p.push_back(0x01); // grip (Android shape)
 }
 return p;
}
static std::vector<uint8_t> touch_min(const std::vector<Obj> &objs) {
 std::vector<uint8_t> p;
 for(auto &o:objs) { p.push_back((uint8_t)(o.id|(o.cls<<4)));le16(p,o.x);le16(p,o.y); }
 return p;
}
static std::vector<uint8_t> padded(const std::vector<uint8_t> &c, unsigned n=128) { auto p=c;p.resize(n,0);return p; }
static void set_cfg(const std::vector<uint8_t> &c) { memcpy(tc.cfg,c.data(),c.size());tc.cfg_len=(unsigned)c.size(); }
static std::vector<uint8_t> bytes(const uint8_t *p, unsigned n) { return std::vector<uint8_t>(p,p+n); }

static void hid_reset() {
 assert(!live && !lock_depth);
 service=SweetHIDTouch();service.fLock=&service.storage;service.fOffY=12;  // salami 1080x2412
 gSweetTouch=&service;dispatched.clear();fail_after=append_fail_after=-1;perf=wake_calls=0;
}
static uint8_t dmabuf[2*16384];
// Fresh driver and hardware state; knobs from bootline/adt (set before the call).
static void setup(unsigned engine=TC_ENG_GPI, bool sensing=true) {
 hid_reset();
 ft={};tc={};fk={};dev=Dev();gpi=Gpi();fm=Fifo();
 tlmm.clear();se.clear();smmu.clear();gcc.clear();tlmm_w.clear();loglines.clear();gp_len=0;sleeps_ms=0;
 dma=(uint8_t *)(((uintptr_t)dmabuf+16383)&~(uintptr_t)16383);
 ft.tlmm=B_TLMM;ft.se=B_SE;ft.gpi=B_GPI;ft.smmu=B_SMMU;ft.gcc=B_GCC;
 ft.dma=(vm_offset_t)dma;ft.dma_pa=DMA_PA;
 se[0x068]=0x300;se[0x064]=engine==TC_ENG_GPI;se[0xe24]=16u<<16;se[0x908]=3;
 ft_config();
 tc.on=true;tc.engine=engine;tc.depth=16;tc.proto=3;
 ft_i2c_timing();ft.level=0;tc.c0_d2=tc.div;
 if(engine==TC_ENG_GPI)assert(!ft_gpi_init());else assert(!ft_se_fifo());
 tc.sx=5760;tc.sy=12864;tc.ow=1080;tc.oh=2412;tc.rdlen=9;
 set_cfg(CFG_OPLUS);
 tc.sensing=sensing;tc.deliver=true;tc.sense_abs=clock_now;
 dev.writes.clear();dev.rsizes.clear();gpi.tre[0].clear();gpi.tre[1].clear();gpi.db.clear();
}

static void t_framing() {
 for(unsigned engine:{TC_ENG_GPI,TC_ENG_FIFO}) {
  bootline="";adt.clear();setup(engine);
  auto m1=msg(0x11,touch_oplus({{0,1,4723,7314}}));assert(m1.size()==24);
  // T > R: predictive 9, continued read of (T - R) + 2 = 17 ("a5 03" + 15 bytes).
  dev.q.push_back(m1);assert(!tcm_read_msg());
  assert(tc.code==0x11 && tc.len==19 && bytes(tc.msg,24)==m1);
  assert((dev.rsizes==std::vector<unsigned>{9,17}) && tc.rdlen==24);
  // T <= R: one read of the adapted length.
  dev.rsizes.clear();dev.q.push_back(m1);assert(!tcm_read_msg() && bytes(tc.msg,24)==m1);
  assert((dev.rsizes==std::vector<unsigned>{24}));
  // T == R + 1: the pad is synthesized, nothing more is read.
  dev.rsizes.clear();tc.rdlen=23;dev.q.push_back(m1);assert(!tcm_read_msg() && bytes(tc.msg,24)==m1);
  assert((dev.rsizes==std::vector<unsigned>{23}));
  assert(!tcm_read_msg() && tc.code==0 && tc.len==0);  // IDLE: nothing left behind
  // Ten fingers (T = 132) after a one-finger read: 24 + continued 110.
  std::vector<Obj> ten;for(unsigned i=0;i<10;i++)ten.push_back({i,1,100*i,200*i});
  auto m10=msg(0x11,touch_oplus(ten));assert(m10.size()==132);
  dev.rsizes.clear();tc.rdlen=24;dev.q.push_back(m10);assert(!tcm_read_msg() && bytes(tc.msg,132)==m10);
  assert((dev.rsizes==std::vector<unsigned>{24,110}) && tc.rdlen==132);
  // Bad marker: ENXIO; bad padding: EIO and a pad-error fact; IDLE/BUSY: no code to dispatch.
  dev.garbage=true;assert(tcm_read_msg()==ENXIO);
  auto bad=m1;bad.back()=0;dev.q.push_back(bad);tc.rdlen=24;assert(tcm_read_msg()==EIO && tc.pad_errs==1);
  assert(logged("salami-touch: pad err #1 last=0"));
  dev.q.push_back(msg(0x02,{}));assert(!tcm_read_msg() && tc.code==0x02 && tc.len==0);
  // A continued read that is not "a5 03" is a framing error.
  dev.rsizes.clear();dev.garbage_read=2;tc.rdlen=24;dev.q.push_back(m10);
  assert(tcm_read_msg()==EIO && logged("salami-touch: continued read ff ff"));
  dev.garbage_read=0;dev.q.clear();dev.rest.clear();
  // A fixed fuxitrdlen is honoured; the cap is 1020.
  bootline="fuxitrdlen=1020";setup(engine);dev.q.push_back(m1);assert(!tcm_read_msg());
  assert((dev.rsizes==std::vector<unsigned>{1020}) && bytes(tc.msg,24)==m1);
  bootline="fuxitrdlen=9";setup(engine);dev.q.push_back(m1);dev.q.push_back(m1);
  assert(!tcm_read_msg() && !tcm_read_msg());assert((dev.rsizes==std::vector<unsigned>{9,17,9,17}));
 }
 bootline="";
 puts("PASS: TouchComm v1 framing on GPI and FIFO: T<=R, T==R+1 pad, continued reads, 132-byte report, adaptive and fixed R, marker/pad/idle/busy");
}

static void t_command_write() {
 bootline="";adt.clear();setup();
 assert(!tcm_write_cmd(0x20,nullptr,0) && (dev.writes.back()==std::vector<uint8_t>{0x20,0,0}));
 assert(!tcm_write_cmd(0x25,nullptr,0) && (dev.writes.back()==std::vector<uint8_t>{0x25,0,0}));
 auto pl=padded(CFG_OPLUS);
 assert(!tcm_write_cmd(0x26,pl.data(),128));
 std::vector<uint8_t> want={0x26,0x80,0x00};want.insert(want.end(),pl.begin(),pl.end());
 assert(dev.writes.back()==want && dev.cmds.back()==want);
 uint8_t dc[3]={0xe6,0x01,0x00};
 assert(!tcm_write_cmd(0x24,dc,3) && (dev.writes.back()==std::vector<uint8_t>{0x24,0x03,0x00,0xe6,0x01,0x00}));
 // max_write_size 64: "26 80 00" + 61, then "01" + 63, then "01" + 4.
 dev.writes.clear();tc.maxwr=64;assert(!tcm_write_cmd(0x26,pl.data(),128));
 assert(dev.writes.size()==3 && dev.writes[0].size()==64 && dev.writes[1].size()==64 && dev.writes[2].size()==5);
 assert(dev.writes[0][0]==0x26 && dev.writes[1][0]==0x01 && dev.writes[2][0]==0x01);
 assert(bytes(&dev.writes[1][1],63)==std::vector<uint8_t>(pl.begin()+61,pl.begin()+124));
 assert(dev.cmds.back()==want);
 // Flash, bootloader, config-commit (0x28), config-id (0x2b) and SPI-master (0x15/0x43)
 // commands never leave the driver; the refused set is exact over all 256 codes.
 static const unsigned refused[]={0x11,0x12,0x13,0x15,0x16,0x1f,0x28,0x2b,0x43};
 for(unsigned c=0;c<256;c++)assert(tcm_flash_cmd(c)==(std::find(std::begin(refused),std::end(refused),c)!=std::end(refused)));
 dev.writes.clear();uint8_t id16[16]={'A','A','5','1','3','0','0','8'};
 for(unsigned c:refused)assert(tcm_write_cmd(c,nullptr,0)==EPERM && tcm_cmd(c,nullptr,0,0x01,100)==EPERM && tcm_write_cmd(c,id16,16)==EPERM);
 assert(dev.writes.empty() && logged("salami-touch: cmd 11 refused (flash/bootloader command)"));
 assert(logged("salami-touch: cmd 28 refused") && logged("salami-touch: cmd 2b refused") && logged("salami-touch: cmd 43 refused"));
 // Neighbours stay allowed: 0x27 REZERO, 0x29 DESCRIBE_DYNAMIC_CONFIG, 0x2a PRODUCTION_TEST.
 for(unsigned c:{0x27u,0x29u,0x2au})assert(!tcm_write_cmd(c,nullptr,0));
 assert(dev.writes.size()==3);
 puts("PASS: command bytes 20/25/26 (128 B)/24 E6 01 00, CONTINUE_WRITE chunking at max_write_size 64, flash/commit/config-id/SPI-master commands refused (exact set)");
}

static void t_cmd() {
 bootline="";adt.clear();setup();
 dev.resp[0x20]={{msg(0x01,{1,2,3})}};assert(!tcm_cmd(0x20,nullptr,0,0x01,1000) && tc.len==3 && tc.msg[6]==3);
 dev.resp[0x2a]={{msg(0x0f,{0x03})}};assert(tcm_cmd(0x2a,nullptr,0,0x01,1000)==EIO);
 assert(logged("salami-touch: cmd 2a -> status 0f err 03"));
 dev.resp[0x27]={{msg(0x02,{}),msg(0x01,{9})}};assert(!tcm_cmd(0x27,nullptr,0,0x01,1000) && tc.msg[4]==9);
 tc.cmd_reset=false;dev.resp[0x25]={{msg(0x10,identify(0x01))}};
 assert(tcm_cmd(0x25,nullptr,0,0x01,1000)==EIO && tc.cmd_reset && tc.mode==1);
 uint64_t t0=clock_now;dev.resp[0x2e]=std::deque<Msgs>(1);
 assert(tcm_cmd(0x2e,nullptr,0,0x01,1000)==ETIMEDOUT && clock_now-t0>=1000000);
 // OK before the identify of a run-application: keep waiting for the report.
 dev.resp[0x14]={{msg(0x01,{}),msg(0x10,identify(0x01))}};assert(!tcm_cmd(0x14,nullptr,0,0x10,1000) && tc.mode==1);
 // A touch report during a command is dispatched once sensing, dropped during bring-up.
 dev.resp[0x2c]={{msg(0x11,touch_oplus({{2,1,1000,2000}})),msg(0x01,{})}};
 assert(!tcm_cmd(0x2c,nullptr,0,0x01,1000) && ft.active==(1u<<2));
 tc.sensing=false;dev.resp[0x2c]={{msg(0x11,touch_oplus({{5,1,1000,2000}})),msg(0x01,{})}};
 assert(!tcm_cmd(0x2c,nullptr,0,0x01,1000) && ft.active==(1u<<2));
 // Three read errors in a row fail the command.
 tc.sensing=true;dev.resp[0x2d]={{msg(0x01,{})}};dev.nack_r=3;assert(tcm_cmd(0x2d,nullptr,0,0x01,1000));
 dev.q.clear();
 ft_report(0);
 puts("PASS: tcm_cmd OK/0x0f error/BUSY then OK/identify mid-command/timeout/OK before identify/0x11 kept when sensing, dropped in bring-up");
}

static void boot_device(unsigned mode=0x01) {
 dev.boot_msg=msg(0x10,identify(mode));
 dev.resp[0x20]={{msg(0x01,app_info(0))}};
 dev.resp[0x25]={{msg(0x01,padded({0x01,0x06,0x04,0x07,0x04,0x08,0x10,0x09,0x10,0x0c,0x10,0x03,0x00}))},
                 {msg(0x01,padded(CFG_OPLUS))}};
 dev.resp[0x26]={{msg(0x01,{})}};
}
static std::vector<unsigned> cmd_codes() { std::vector<unsigned> v;for(auto &c:dev.cmds)v.push_back(c[0]);return v; }

static void t_bringup() {
 bootline="";adt.clear();setup(TC_ENG_GPI,false);boot_device();
 assert(!tcm_bringup() && tc.sensing && tc.mode==1);
 assert((cmd_codes()==std::vector<unsigned>{0x20,0x25,0x26,0x25}));
 assert(dev.cmds[2].size()==3+128 && bytes(dev.cmds[2].data()+3,21)==CFG_OPLUS);
 assert(tc.cfg_len==128 && bytes(tc.cfg,21)==CFG_OPLUS && tc.sx==5760 && tc.sy==12864);
 assert(logged("salami-touch: attn after reset 4") && logged("salami-touch: identify len=24 ver=1 mode=0x1 part=\"S3908-SYNTHETIC\" build=1193046 maxwr=0"));
 assert(logged("salami-touch: app len=46 ver=1 status=0 cfgmax=128 paymax=127 id=\"AA513008\" maxx=5759 maxy=12863 objs=10 rows=17 cols=38 hybrid=0"));
 assert(logged("salami-touch: span 5760x12864 (app)") && logged("salami-touch: fw cfg len=128 010604070408100910") && logged("salami-touch: set cfg mode=2 len=128 -> 0; active len=128 1008c630"));
 assert(logged("salami-touch: pins gpio10 ctl=") && logged("salami-touch: ios before=0x3 after=0x3") && logged("salami-touch: first read 9 B: a5101800"));
 // Power sequence: ATTN input pull-up 8 mA, SDA/SCL mux 0x2007, reset low, AVDD high, then reset high.
 std::vector<std::pair<uint32_t,uint32_t>> want={{0x19000,0xc3},{0x2c000,0x2007},{0x2d000,0x2007},
  {0x18004,0},{0x18000,0x2c0},{0xa004,2},{0xa000,0x201},{0x18004,2},{0x18000,0x2c0}};
 assert(tlmm_w==want);
 // Bootloader mode: RUN_APPLICATION_FIRMWARE, identify report, application mode.
 setup(TC_ENG_GPI,false);boot_device(0x0b);dev.resp[0x14]={{msg(0x10,identify(0x01))}};
 assert(!tcm_bringup() && (cmd_codes()==std::vector<unsigned>{0x14,0x20,0x25,0x26,0x25}));
 // Host download (zeroflash) contradicts the flash verdict: stop, no 0x14.
 setup(TC_ENG_GPI,false);boot_device(0x02);
 assert(tcm_bringup()==ENODEV && !tc.sensing && (cmd_codes()==std::vector<unsigned>{0x10}));
 assert(logged("salami-touch: firmware mode 0x2 (host download): touch stopped"));
 // Bootloader that stays: fuxitrun attempts, then stop with the boot info.
 setup(TC_ENG_GPI,false);boot_device(0x0b);dev.resp[0x14]={{msg(0x10,identify(0x0b))}};
 dev.resp[0x10]={{msg(0x01,{1,0xfe,0,0,0,0,0,0,0,0x05})}};
 assert(tcm_bringup()==ENODEV && (cmd_codes()==std::vector<unsigned>{0x14,0x14,0x10}));
 assert(logged("salami-touch: boot status=0xfe reset_reason=0x5"));
 // App status BOOTING twice: re-poll every 100 ms.
 setup(TC_ENG_GPI,false);boot_device();dev.resp[0x20]={{msg(0x01,app_info(1))},{msg(0x01,app_info(1))},{msg(0x01,app_info(0))}};
 unsigned s0=sleeps_ms;assert(!tcm_bringup() && (cmd_codes()==std::vector<unsigned>{0x20,0x20,0x20,0x25,0x26,0x25}));
 assert(sleeps_ms-s0>=200);
 // BAD_APP_CONFIG stops.
 setup(TC_ENG_GPI,false);boot_device();dev.resp[0x20]={{msg(0x01,app_info(0xff))}};
 assert(tcm_bringup()==ENODEV && logged("bad app config"));
 // SET refused: the firmware's config is what the parser uses.
 setup(TC_ENG_GPI,false);boot_device();dev.resp[0x26]={{msg(0x0f,{0x01})}};
 dev.resp[0x25]={{msg(0x01,padded(CFG_MIN))}};
 assert(!tcm_bringup() && bytes(tc.cfg,11)==CFG_MIN && logged("set cfg mode=2 len=128 -> 0x5"));
 // Max config size below the config length: not sent, firmware config kept.
 setup(TC_ENG_GPI,false);boot_device();dev.resp[0x20]={{msg(0x01,app_info(0,5759,12863,16))}};
 dev.resp[0x25]={{msg(0x01,padded(CFG_MIN,16))}};
 assert(!tcm_bringup() && (cmd_codes()==std::vector<unsigned>{0x20,0x25,0x25}) && bytes(tc.cfg,11)==CFG_MIN);
 // No identify pending (loader consumed it): CMD_IDENTIFY answers with OK + identify.
 bootline="fuxitpwr=0";setup(TC_ENG_GPI,false);boot_device();dev.boot_msg.clear();
 dev.resp[0x02]={{msg(0x01,identify(0x01,64))}};
 assert(!tcm_bringup() && tc.maxwr==64 && (cmd_codes()==std::vector<unsigned>{0x02,0x20,0x25,0x26,0x25}));
 assert(tlmm_w.size()==1 && logged("salami-touch: attn not seen in 300 ms"));
 // CMD_IDENTIFY unanswered: CMD_RESET, identify report.
 setup(TC_ENG_GPI,false);boot_device();dev.boot_msg.clear();dev.resp[0x02]=std::deque<Msgs>(1);
 dev.resp[0x04]={{msg(0x10,identify(0x01))}};
 assert(!tcm_bringup() && (cmd_codes()==std::vector<unsigned>{0x02,0x04,0x20,0x25,0x26,0x25}));
 // fuxitrcfg=0 keeps the firmware config (no SET); 1 sends the minimal one; 3 the ADT blob.
 bootline="fuxitrcfg=0";setup(TC_ENG_GPI,false);boot_device();
 assert(!tcm_bringup() && (cmd_codes()==std::vector<unsigned>{0x20,0x25,0x25}));
 bootline="fuxitrcfg=1";setup(TC_ENG_GPI,false);boot_device();
 assert(!tcm_bringup() && bytes(dev.cmds[2].data()+3,11)==CFG_MIN);
 bootline="fuxitrcfg=3";adt["fuxitrcfgb"]={0x01,0x06,0x04,0x07,0x04,0x08,0x10,0x09,0x10,0x0d,0x08,0x03,0x00};
 setup(TC_ENG_GPI,false);boot_device();
 assert(!tcm_bringup() && dev.cmds[2].size()==3+128 && dev.cmds[2][3+9]==0x0d && dev.cmds[2][3+12]==0x00);
 adt.clear();bootline="";
 // FIFO engine: the same bring-up.
 setup(TC_ENG_FIFO,false);boot_device();assert(!tcm_bringup() && tc.sensing);
 // tcm_start retries once; fuxitpwr bit 3 power-cycles in between (GPIO24 low, GPIO10 low).
 bootline="fuxitpwr=0xf";setup(TC_ENG_GPI,false);boot_device();dev.resp[0x20]=std::deque<Msgs>(1);
 assert(tcm_start()==ETIMEDOUT);
 assert(std::count(tlmm_w.begin(),tlmm_w.end(),std::make_pair(0xa004u,0u))==1);
 bootline="";
 puts("PASS: bring-up B1-B6 (pending identify, CMD_IDENTIFY, CMD_RESET), bootloader -> 0x14, host download/bootloader/bad config stop, booting re-poll, SET refusal, rcfg 0/1/2/3, power pin order, FIFO engine");
}

static void t_parser() {
 bootline="";adt.clear();setup();
 auto p=touch_oplus({{0,1,4723,7314},{3,2,100,12000}});
 assert(!tcm_parse(p.data(),(unsigned)p.size()));
 assert(tc.pseen==((1u<<0)|(1u<<3)) && tc.px[0]==4723 && tc.py[0]==7314 && tc.pcls[0]==1);
 assert(tc.px[3]==100 && tc.py[3]==12000 && tc.pcls[3]==2);
 // OPLUS bit offsets: object block at bit 56, 96 bits each.
 assert(tcm_bits(p.data(),(unsigned)p.size(),56+8,16)==4723 && tcm_bits(p.data(),(unsigned)p.size(),56+96+4,4)==2);
 // A trailing partial object (5 bytes) is not entered; no objects at all is empty.
 auto q=p;q.resize(q.size()+5,0x33);assert(!tcm_parse(q.data(),(unsigned)q.size()) && tc.pseen==((1u<<0)|(1u<<3)));
 auto none=touch_oplus({});assert(!tcm_parse(none.data(),7) && tc.pseen==0);
 // Minimal layout: 5 bytes per object, no header.
 set_cfg(CFG_MIN);auto m=touch_min({{9,1,1079,2411},{1,8,5,6}});
 assert(!tcm_parse(m.data(),(unsigned)m.size()) && tc.pseen==((1u<<9)|(1u<<1)) && tc.px[9]==1079 && tc.pcls[1]==8);
 // NUM_OF_ACTIVE_OBJECTS = 0 skips the block; = 2 parses exactly two.
 std::vector<uint8_t> act={0x18,0x08,0x01,0x06,0x04,0x07,0x04,0x08,0x10,0x09,0x10,0x03,0x00};set_cfg(act);
 std::vector<uint8_t> z={0x00,0x21,0x10,0x00,0x20,0x00};assert(!tcm_parse(z.data(),6) && tc.pseen==0);
 std::vector<uint8_t> two={0x02};auto t2=touch_min({{4,1,7,8},{6,1,9,10},{7,1,11,12}});two.insert(two.end(),t2.begin(),t2.end());
 assert(!tcm_parse(two.data(),(unsigned)two.size()) && tc.pseen==((1u<<4)|(1u<<6)));
 // Unknown entity 0x1a (width 8) before the block is skipped by its width; 0x80 inside too.
 std::vector<uint8_t> unk={0x1a,0x08,0x01,0x06,0x04,0x07,0x04,0x08,0x10,0x80,0x08,0x09,0x10,0x03,0x00};set_cfg(unk);
 std::vector<uint8_t> u={0xee,0x12,0x34,0x12,0x77,0x78,0x56};
 assert(!tcm_parse(u.data(),(unsigned)u.size()) && tc.pseen==(1u<<2) && tc.px[2]==0x1234 && tc.py[2]==0x5678 && tc.pcls[2]==1);
 // Object index >= 10: the report is dropped.
 set_cfg(CFG_OPLUS);auto bi=touch_oplus({{12,1,5,5}});assert(tcm_parse(bi.data(),(unsigned)bi.size())==EINVAL);
 // A config whose loop consumes nothing terminates; a missing width byte is an error.
 std::vector<uint8_t> loop0={0x01,0x04,0x03,0x00};set_cfg(loop0);assert(!tcm_parse(u.data(),7));
 std::vector<uint8_t> trunc={0x08};set_cfg(trunc);assert(tcm_parse(u.data(),7)==EINVAL);
 assert(tcm_cfg_len(CFG_OPLUS.data(),21)==21 && tcm_cfg_len(padded(CFG_MIN).data(),128)==11);
 // Object fields after a completed FOREACH_OBJECT loop (index 10): dropped, never written past
 // px/py/pcls (UBSan bounds traps the old code); OBJECT_INDEX re-bounds the index after the loop.
 std::vector<uint8_t> r11(11,0x21);
 for(uint8_t f:{0x07,0x08,0x09}) {
  std::vector<uint8_t> after={0x02,0x08,0x08,0x03,f,0x04,0x00};set_cfg(after);
  assert(tcm_parse(r11.data(),11)==EINVAL);
 }
 std::vector<uint8_t> reidx={0x02,0x08,0x08,0x03,0x06,0x04,0x07,0x04,0x00};set_cfg(reidx);
 for(unsigned i=0;i<10;i++)r11[i]=(uint8_t)(10+i);
 r11[10]=0x23;assert(!tcm_parse(r11.data(),11) && tc.pseen==(1u<<3) && tc.pcls[3]==2 && tc.px[0]==10 && tc.px[9]==19);
 // An object field before any loop is object 0, as before.
 std::vector<uint8_t> flat={0x07,0x04,0x08,0x10,0x09,0x10,0x00};set_cfg(flat);
 std::vector<uint8_t> f5={0x41,0x23,0x01,0x00,0x00};assert(!tcm_parse(f5.data(),5) && tc.pseen==1u && tc.pcls[0]==1 && tc.px[0]==0x1234);
 set_cfg(CFG_OPLUS);
 puts("PASS: report parser: OPLUS 12-byte objects, minimal 5-byte, partial tail, NUM_OF_ACTIVE 0/2, unknown entities by width, index >= 10, object fields after a completed loop, degenerate configs");
}

static void report(const std::vector<Obj> &o) { dev.q.push_back(msg(0x11,touch_oplus(o))); }
static void t_contacts() {
 bootline="";adt.clear();setup();
 // Default fuxitcls 0x12e: 1 finger, 2 glove, 3 stylus, 5 small, 8 edge down; 4, 6 palm, 9 hover not.
 report({{0,1,1000,1000},{1,2,1100,1000},{2,3,1200,1000},{3,4,1300,1000},{4,5,1400,1000},
         {5,6,1500,1000},{6,8,1600,1000},{7,9,1700,1000}});
 assert(!tcm_poll(true) && ft.active==0x57);
 // A missing object lifts; an IDLE poll changes nothing.
 report({{0,1,1000,1000}});assert(!tcm_poll(true) && ft.active==1);
 assert(!tcm_poll(true) && ft.active==1 && dev.q.empty());
 // Hover lifts.
 report({{0,9,1000,1000}});assert(!tcm_poll(true) && ft.active==0);
 // fuxitpalm=1: any palm lifts every contact until a report without palm.
 bootline="fuxitpalm=1";setup();
 report({{0,1,1000,1000}});assert(!tcm_poll(true) && ft.active==1);
 report({{0,1,1000,1000},{4,6,3000,9000}});assert(!tcm_poll(true) && ft.active==0);
 report({{0,1,1000,1000}});assert(!tcm_poll(true) && ft.active==1);
 ft_report(0);
 // fuxitcls=0x2: fingers only.
 bootline="fuxitcls=0x2";setup();report({{0,1,1,1},{1,2,2,2}});assert(!tcm_poll(true) && ft.active==1);
 ft_report(0);bootline="";
 puts("PASS: contacts: default class mask, missing object and hover lift, IDLE never lifts, palm policies, fuxitcls");
}

static void t_coalescing() {
 bootline="";adt.clear();setup();
 // Down and up inside one drain: two HID reports, the tap is not lost.
 report({{0,1,2880,6432}});report({{0,1,2900,6432}});report({});
 size_t n=dispatched.size();assert(!tcm_poll(true));
 assert(dispatched.size()==n+2 && ft.active==0 && ft.c_reports==2);
 // Pure motion inside one drain: one report with the last position.
 report({{0,1,2880,6432}});assert(!tcm_poll(true));n=dispatched.size();
 report({{0,1,2900,6432}});report({{0,1,2920,6432}});report({{0,1,2940,6432}});
 assert(!tcm_poll(true) && dispatched.size()==n+1 && ft.x[0]==2940*1080/5760);
 // fuxitdrain caps the messages per poll; the rest waits for the next poll.
 n=dispatched.size();for(unsigned i=0;i<6;i++)report({{0,1,3000+20*i,6432}});
 assert(!tcm_poll(true) && dev.q.size()==2 && dispatched.size()==n+1);
 assert(!tcm_poll(true) && dev.q.empty());
 // Unchanged snapshots report nothing.
 n=dispatched.size();report({{0,1,3100,6432}});report({{0,1,3100,6432}});assert(!tcm_poll(true) && dispatched.size()==n);
 ft_report(0);ft_retry_report();
 puts("PASS: coalescing: down/up in one drain gives two reports, motion gives one, fuxitdrain cap, unchanged snapshots silent");
}

static void t_wake() {
 bootline="";adt.clear();setup();
 fuxi_wake_touch_display(&ft.wake,true);
 report({{4,1,500,1000}});assert(!tcm_poll(false) && wake_calls==1 && ft.wake.woke && dispatched.empty());
 fuxi_wake_touch_display(&ft.wake,false);
 report({{4,1,500,900}});assert(!tcm_poll(true) && dispatched.empty() && !ft.active);
 report({});assert(!tcm_poll(true) && dispatched.empty());
 report({{4,1,500,1000}});assert(!tcm_poll(true) && ft.active==(1u<<4));
 // Display off without a wake press: dropped, nothing hidden afterwards.
 ft_report(0);fuxi_wake_touch_display(&ft.wake,true);wake_calls=0;ft.wake.woke=false;
 report({{1,1,5,5}});assert(!tcm_poll(false) && !ft.active);
 puts("PASS: tap-to-wake with snapshots: waking contact hidden until absent, display-off reports dropped");
}

static void t_mapping() {
 bootline="";adt.clear();setup();int x, y;
 tcm_map(0,0,&x,&y);assert(x==0 && y==0);
 tcm_map(5759,12863,&x,&y);assert(x==1079 && y==2411);
 tcm_map(4723,7314,&x,&y);assert(x==885 && y==1371);
 tc.sx=1440;tc.sy=3216;tcm_map(1439,3215,&x,&y);assert(x==1079 && y==2411);
 tcm_map(99999,99999,&x,&y);assert(x==1079 && y==2411);
 tc.sx=5760;tc.sy=12864;
 fk.xy=2;tcm_map(0,0,&x,&y);assert(x==1079 && y==0);
 fk.xy=4;tcm_map(0,0,&x,&y);assert(x==0 && y==2411);
 fk.xy=1;tc.sx=12864;tc.sy=5760;tcm_map(7314,4723,&x,&y);assert(x==885 && y==1371);
 fk.xy=0;tc.sx=5760;tc.sy=12864;fk.ox=-12;fk.oy=30;tcm_map(0,12863,&x,&y);assert(x==0 && y==2411);
 tcm_map(5759,0,&x,&y);assert(x==1067 && y==30);fk.ox=fk.oy=0;
 console_w=1080;console_h=2412;tcm_out_space();assert(tc.ow==1080 && tc.oh==2412 && logged("salami-touch: out 1080x2412 (console) off 0,0"));
 console_w=0;console_h=0;tcm_out_space();assert(tc.ow==1080 && tc.oh==2412 && logged("(fallback)"));
 bootline="fuxitow=1440 fuxitoh=3216";setup();tcm_out_space();assert(tc.ow==1440 && tc.oh==3216 && logged("(knob)"));
 console_w=1080;console_h=2412;bootline="";
 puts("PASS: mapping vectors (0,0) (5759,12863) (4723,7314)->(885,1371) 1440x3216 span, clamp, swap/flips, offsets, output space");
}

static void t_knobs() {
 adt.clear();
 bootline="fuxitclk=400k fuxitaddr=075 fuxitox=-12 fuxitcmd=0x110105 fuxitrdlen=5 fuxitdrain=8 fuxitto=fast fuxitirq";
 adt["fuxitdrain"]=u32le(2);adt["fuxitpwr"]=u32le(3);adt["fuxitxy"]=u32le(6);adt["fuxitcb"]=u32le(0xffffffff);
 adt["fuxitoy"]=u32le((uint32_t)-7);
 setup();
 assert(fk.clk==400 && logged("salami-touch: cfg reject fuxitclk=409600 (bootarg)"));
 assert(fk.addr==61 && logged("salami-touch: cfg fuxitaddr=61 (bootarg)"));
 assert(fk.ox==-12 && fk.oy==-7 && fk.cmd==0x110105 && fk.rdlen==0 && logged("cfg reject fuxitrdlen=5"));
 assert(fk.drain==8 && fk.pwr==3 && fk.xy==6 && fk.cb==-1 && logged("salami-touch: cfg fuxitpwr=3 (adt)"));
 assert(fk.to==3000 && logged("cfg reject fuxitto=") && fk.irq==1);
 assert(fk.i2ct==0 && fk.lvl==-1 && fk.pinctl==0x2007 && fk.rcfg==2 && fk.cls==0x12e && fk.sweep==32 && fk.recent==150 && fk.dbg==1);
 assert(tc.pin_avdd==10 && tc.pin_rst==24 && tc.pin_attn==25);
 // fuxitcmd 64-bit decode: bytes [cmd][n][payload].
 bootline="fuxitcmd=0x0605040302010629";adt.clear();adt["fuxitdc"]=u32le(0xe60001);setup();
 tcm_extra();
 assert((dev.cmds[0]==std::vector<uint8_t>{0x24,0x03,0x00,0xe6,0x01,0x00}));
 assert((dev.cmds[1]==std::vector<uint8_t>{0x29,0x06,0x00,1,2,3,4,5,6}));
 bootline="fuxitcmd=0x110105";adt.clear();setup();tcm_extra();
 assert((dev.cmds[0]==std::vector<uint8_t>{0x05,0x01,0x00,0x11}));
 // fuxitcmd (boot-arg or ADT) cannot commit the config, set the config id or reach the SPI master.
 for(const char *b:{"fuxitcmd=0x28","fuxitcmd=0x1022b","fuxitcmd=0x15","fuxitcmd=0x43","fuxitcmd=0x1f"}) {
  bootline=b;adt.clear();setup();dev.writes.clear();tcm_extra();
  unsigned c=(unsigned)(fk.cmd&0xff), n=(unsigned)((fk.cmd>>8)&0xff);char want[48];
  snprintf(want,sizeof(want),"salami-touch: fuxitcmd %02x len %u -> %d",c,n,EPERM);
  assert(fk.cmd && dev.writes.empty() && dev.cmds.empty() && logged(want));
 }
 bootline="";adt["fuxitcmd"]=u32le(0x28);setup();dev.writes.clear();tcm_extra();
 assert(fk.cmd==0x28 && dev.writes.empty() && logged("salami-touch: cmd 28 refused"));
 adt.clear();
 // 8-byte ADT value; an n above 6 is rejected; a 64-bit boot-arg wins over the ADT.
 bootline="";adt["fuxitcmd"]={0x27,0x00,0,0,0,0,0,0};setup();assert(fk.cmd==0x27);
 adt["fuxitcmd"]=u32le(0x00000927);setup();assert(fk.cmd==0 && logged("cfg reject fuxitcmd=0x927"));
 bootline="fuxitcmd=0x27";setup();assert(fk.cmd==0x27);
 // Timing: table rows and the override; a zero field is rejected.
 bootline="fuxitclk=100";adt.clear();setup();assert(tc.c0_d0==0x031a0a0b && tc.div==7 && tc.khz==105);
 bootline="fuxitclk=1000";setup();assert(tc.c0_d0==0x03120309 && tc.div==1 && tc.khz==1066);
 bootline="fuxiti2ct=0x02050c18";setup();assert(tc.c0_d0==0x0318050c && tc.div==2 && tc.t_cycle==24);
 bootline="fuxiti2ct=0x02000c18";setup();assert(fk.i2ct==0 && tc.c0_d0==0x0318070a);
 bootline="fuxitgpio=0x30180a";setup();assert(fk.gpio==0 && tc.pin_attn==25);
 bootline="fuxitgpio=0x1b170b";setup();assert(tc.pin_avdd==11 && tc.pin_rst==23 && tc.pin_attn==27);
 bootline="fuxitic=3 fuxitlvl=-1 fuxitlvl=5";setup();assert(fk.ic==0 && fk.lvl==5);
 bootline="";adt.clear();
 puts("PASS: knobs: boot-arg > ADT (u32 sign-extended, u64) > default, 400k/octal/string/bare traps, ranges, fuxitcmd/fuxitdc encodings, timing table and override, pins");
}

static void t_gpi_vectors() {
 bootline="";adt.clear();setup();
 // Clock level search: level 3 is XO undivided on this model.
 gcc[0x18504]=1;gcc[0x1850c]=0x101;gcc[0x18510]=0x205;gcc[0x18514]=0x003;gcc[0x18518]=0x001;
 se[0x068]=0x300;se[0x064]=1;
 assert(!ft_ahb() && !ft_detect() && tc.on && tc.engine==TC_ENG_GPI);
 smmu[0x20]=8;smmu[0x800]=0x80000040u;smmu[0xc00]=0x5;
 assert(!tcm_init_bus() && ft.level==3 && tc.c0_d2==0x00030002 && se[0x07c]==3 && se[0x048]==0x21);
 assert(smmu[0x804]==0x800000b6u && smmu[0xc04]==5 && smmu[0x808]==0x800000a3u && smmu[0xc08]==5);
 assert(logged("salami-touch: smmu nsmr=8 dwc3_s2cr=0x5 gpi_smr=-1 qup_smr=-1 s2cr_used=0x5"));
 assert(logged("salami-touch: clk level=3 dfs_en=1 div=2 t=7/10/24 nominal_khz=400"));
 assert(gpi.reg[0x0060]==0x00010034 && gpi.reg[0x00e0]==0x00000034);
 uint32_t lo=(uint32_t)DMA_PA, hi=(uint32_t)(DMA_PA>>32);
 using T=std::array<uint32_t,4>;
 gpi.tre[0].clear();gpi.tre[1].clear();gpi.db.clear();
 uint8_t w3[3]={0x20,0,0};assert(!ft_i2c_write(w3,3));
 assert((gpi.tre[0]==std::vector<T>{{0x0318070a,0,0x00030002,0x00220001},{0x00004b01,0,0,0x00200001},{lo+0x1800,hi,3,0x00100200}}));
 assert((gpi.db==std::vector<unsigned>{0}));
 gpi.tre[0].clear();gpi.db.clear();uint8_t r[24];
 dev.q.clear();dev.q.push_back(msg(0x11,touch_oplus({{0,1,1,1}})));assert(!ft_i2c_read(r,24) && r[0]==0xa5 && r[23]==0x5a);
 assert((gpi.tre[1]==std::vector<T>{{lo+0x1c00,hi,24,0x00100200}}));
 assert((gpi.tre[0]==std::vector<T>{{0x0318070a,0,0x00030002,0x00220001},{0x00004b02,0,24,0x00200900}}));
 assert((gpi.db==std::vector<unsigned>{1,0}) && logged("salami-touch: gpi read ok flags=0x200900"));
 // fuxitrgo=1: GO read with LINK only; fuxitc0=1: CONFIG0 once per GPI init.
 bootline="fuxitrgo=1 fuxitc0=1 fuxitaddr=0x20";setup();dev.addr=0x20;
 gpi.tre[0].clear();assert(!ft_i2c_read(r,9));
 assert(gpi.tre[0].size()==2 && gpi.tre[0][1][0]==0x00002002 && gpi.tre[0][1][3]==0x00200800);
 gpi.tre[0].clear();assert(!ft_i2c_read(r,9) && gpi.tre[0].size()==1);
 assert(!ft_gpi_init());gpi.tre[0].clear();assert(!ft_i2c_write(w3,3) && gpi.tre[0].size()==3 && gpi.tre[0][1][0]==0x00002001);
 // FTS keeps its SPI scratch and CONFIG0.
 tc.on=false;assert(!ft_gpi_init() && gpi.reg[0x0060]==0x00010014 && gpi.reg[0x00e0]==0x14);
 gpi.tre[0].clear();ft.level=1;ft_c0();assert(gpi.tre[0].empty() && ft.wp[0]==1);
 assert((tre_at(0,0)==T{0x03000004,0,(1u<<16)|2,0x00220001}));
 tc.on=true;bootline="";
 // Ring wrap: 100 transfers through the 64-entry rings.
 setup();for(unsigned i=0;i<100;i++) { dev.q.push_back(msg(0x01,{})); assert(!ft_i2c_read(r,9) && r[1]==1); assert(!ft_i2c_write(w3,3)); }
 puts("PASS: GPI golden TREs (CONFIG0 0x0318070a/lvl<<16|2, GO 0x4b01/0x4b02, DMA, IEOB/LINK flags), doorbell order, rgo/c0/addr knobs, SCRATCH_0 0x34/0x14, SMMU S2CR copy, ring wrap");
}

static void t_gpi_errors() {
 bootline="";adt.clear();setup();uint8_t r[16];
 unsigned a=gpi.allocs;dev.nack_r=1;gp_len=0;
 assert(ft_i2c_read(r,9)==EIO && tc.last_err==TC_ERR_ANACK && gpi.allocs==a+1);
 assert(logged("salami-touch: bus err kind=addr_nack ev=0x10000000 0x1220040 gp_len=0"));
 dev.nack_r=1;gp_len=3;assert(ft_i2c_read(r,9)==EIO && tc.last_err==TC_ERR_DNACK && gpi.allocs==a+2);
 gpi.style=2;dev.nack_r=1;gp_len=0;assert(ft_i2c_read(r,9)==EIO && tc.last_err==TC_ERR_ANACK && gpi.allocs==a+3);
 gpi.arb=1;assert(ft_i2c_read(r,9)==EIO && tc.last_err==TC_ERR_ARB && gpi.allocs==a+4);
 gpi.drop=1;assert(ft_i2c_read(r,9)==ETIMEDOUT && tc.last_err==TC_ERR_TIME && gpi.allocs==a+5);
 gpi.benign=1;assert(!ft_i2c_read(r,9) && gpi.allocs==a+5 && logged("salami-touch: gpi notify without error bits d0=0x40"));
 // Writes: address NACK on the TX channel.
 gpi.style=1;dev.nack_w=1;uint8_t w[3]={0x20,0,0};assert(ft_i2c_write(w,3)==EIO && gpi.allocs==a+6);
 // After every error the next transfer works (fresh rings, CONFIG0 again).
 assert(!ft_i2c_write(w,3) && !ft_i2c_read(r,9));
 assert(ft_bus_reinit()==0 && gpi.allocs==a+7 && logged("salami-touch: bus re-init ios=0x3 -> 0"));
 assert(tc.bus_errs==6);
 puts("PASS: GPI errors: addr/data NACK (status 0x40 + M_GP_LENGTH), QUP-notify NACK and ARB_LOST, timeout, benign notify, one GPII re-init per error");
}

static void t_fifo() {
 bootline="";adt.clear();setup(TC_ENG_FIFO);
 assert(se[0x260]==0x0007f8fe && se[0x288]==0x000ffefe && se[0x048]==0x21 && se[0x278]==0x00702818 && se[0x258]==0);
 assert(se[0x810]==15 && se[0x814]==16 && se[0xe18]==0);
 uint8_t w3[3]={0x20,0,0};assert(!ft_i2c_write(w3,3) && fm.cmd0.back()==0x08009600 && se[0x26c]==3);
 assert((dev.writes.back()==std::vector<uint8_t>{0x20,0,0}));
 std::vector<Obj> ten;for(unsigned i=0;i<10;i++)ten.push_back({i,1,300+i,400+i});
 auto m=msg(0x11,touch_oplus(ten));uint8_t r[132];dev.q.clear();dev.q.push_back(m);
 assert(!ft_i2c_read(r,132) && fm.cmd0.back()==0x10009600 && se[0x270]==132 && bytes(r,132)==m);
 // A 131-byte write goes through several TX watermark refills.
 auto pl=padded(CFG_OPLUS);assert(!tcm_write_cmd(0x26,pl.data(),128) && dev.cmds.back().size()==131);
 // NACK completes the command: no cancel/abort when the SE is idle afterwards.
 fm.nack=1;assert(ft_i2c_read(r,9)==EIO && tc.last_err==TC_ERR_ANACK && fm.ctrl.empty());
 // A hung command: cancel, then abort when the cancel is not acknowledged.
 fm.hang=1;fm.stubborn=1;assert(ft_i2c_read(r,9)==ETIMEDOUT && (fm.ctrl==std::vector<uint32_t>{4,2}));
 fm.hang=0;fm.stubborn=0;fm.ctrl.clear();
 assert(!ft_bus_reinit() && (fm.ctrl==std::vector<uint32_t>{4}));
 dev.q.clear();dev.q.push_back(msg(0x01,{7}));assert(!ft_i2c_read(r,9) && r[4]==7);
 // fuxitclk=100 on the FIFO engine: M_CLK_CFG 0x71, SCL counters 0x00a02c1a.
 bootline="fuxitclk=100";setup(TC_ENG_FIFO);assert(se[0x048]==0x71 && se[0x278]==0x00a02c1a);
 bootline="";
 puts("PASS: FIFO engine: init/packing, M_CMD0 0x08009600/0x10009600, TRANS_LEN, word unpacking, multi-refill TX, NACK without abort, cancel then abort");
}

static void t_fifo_depth() {
 // SE_HW_PARAM_0 FIFO depth is bits 23:16 (Linux GENMASK(23, 16), QUP v3.10+): 64 and more
 // words read right (the old 0x3f mask gave 0 or a wrong depth); bits above 23 are not depth.
 for(unsigned d:{16u,64u,192u,255u}) {
  bootline="";adt.clear();setup(TC_ENG_FIFO);
  se[0x064]=0;se[0xe24]=0xff000000u|(d<<16)|0x1234u;
  assert(!ft_detect() && tc.on && tc.engine==TC_ENG_FIFO && tc.depth==d);
  char want[32];snprintf(want,sizeof(want),"fifo_depth=%u ",d);assert(logged(want));
  // Init drains stale RX words up to the full depth and sets the watermarks from it.
  fm=Fifo();fm.depth=d;for(unsigned i=0;i<d;i++)fm.pend.push_back(0xdead0000u|i);
  assert(!ft_se_fifo() && fm.rx.empty() && fm.pend.empty() && se[0x810]==d-1 && se[0x814]==d);
  // A ten-finger report (132 B) and a 131-byte command through that FIFO.
  std::vector<Obj> ten;for(unsigned i=0;i<10;i++)ten.push_back({i,1,300+i,400+i});
  auto m=msg(0x11,touch_oplus(ten));uint8_t r[132];dev.q.clear();dev.q.push_back(m);
  assert(!ft_i2c_read(r,132) && bytes(r,132)==m);
  auto pl=padded(CFG_OPLUS);assert(!tcm_write_cmd(0x26,pl.data(),128) && dev.cmds.back().size()==131);
 }
 // Cancel/abort also drains more than 64 stale words (a length error with 200 words queued).
 bootline="";setup(TC_ENG_FIFO);fm.depth=255;fm.rx.assign(200,0x5a5a5a5au);uint8_t r9[9];
 assert(ft_i2c_read(r9,9)==EIO && tc.last_err==TC_ERR_LEN && fm.rx.empty() && (fm.ctrl==std::vector<uint32_t>{4}));
 puts("PASS: FIFO depth from SE_HW_PARAM_0[23:16] (16/64/192/255, high bits ignored), watermarks, init and cancel drains up to 255 words, 132/131-byte transfers");
}

static void t_detect_power_resume() {
 bootline="";adt.clear();setup(TC_ENG_GPI,false);
 se[0x068]=0x300;se[0x064]=0;assert(!ft_detect() && tc.on && tc.engine==TC_ENG_FIFO);
 se[0x064]=1;assert(!ft_detect() && tc.engine==TC_ENG_GPI);
 fk.mode=1;assert(ft_detect()==ENODEV);fk.mode=0;
 fk.ic=1;assert(ft_detect()==ENODEV);fk.ic=0;
 se[0x068]=0x100;tc.on=true;assert(!ft_detect() && !tc.on);
 se[0x068]=0x300;fm.busy=true;assert(ft_detect()==ENODEV);fm.busy=false;
 assert(logged("salami-touch: se4 fw=0x300 proto=3 if_disable=0x1"));
 // Self-reset while sensing: identify -> ic_reset; resume repeats B2-B6.
 setup();boot_device();dev.q.push_back(msg(0x10,identify(0x01)));
 assert(!tcm_poll(true) && ft.ic_reset && tc.mode==1);
 ft.ic_reset=false;dev.cmds.clear();assert(!tcm_resume() && tc.sensing);
 assert((cmd_codes()==std::vector<unsigned>{0x20,0x25,0x26,0x25}) && logged("salami-touch: ic reset #1 mode=0x1"));
 // Resume in bootloader mode: RUN_APPLICATION_FIRMWARE first.
 dev.cmds.clear();tc.mode=0x0b;dev.resp[0x14]={{msg(0x10,identify(0x01))}};assert(!tcm_resume());
 assert(cmd_codes()[0]==0x14);
 // A resume that keeps failing stops after three attempts.
 dev.cmds.clear();dev.resp[0x20]=std::deque<Msgs>(1);assert(tcm_resume()==ETIMEDOUT && !tc.sensing && dev.cmds.size()==3);
 // fuxitoff=1 deep sleep commands; tcm_recover re-runs bring-up after a power cycle.
 bootline="fuxitoff=1 fuxitpwr=0xf";setup();boot_device();tcm_display(true);assert(tc.sleeping && dev.cmds.back()[0]==0x2c);
 tcm_display(false);assert(!tc.sleeping && dev.cmds.back()[0]==0x2d);
 // An identify instead of the sleep/wake answer: the IC reset under it, so the shared loop
 // runs tcm_resume (ft.ic_reset) and the report config is applied again.
 dev.resp[0x2c]={{msg(0x10,identify(0x01))}};ft.ic_reset=false;tcm_display(true);
 assert(!tc.sleeping && tc.cmd_reset && ft.ic_reset);
 dev.resp[0x2d]={{msg(0x10,identify(0x01))}};ft.ic_reset=false;tcm_display(false);assert(ft.ic_reset);
 // A stale cmd_reset from an earlier command does not fake one.
 dev.resp[0x2c]={{msg(0x01,{})}};dev.resp[0x2d]={{msg(0x01,{})}};ft.ic_reset=false;tc.cmd_reset=true;
 tcm_display(true);assert(tc.sleeping && !ft.ic_reset);
 tcm_display(false);assert(!tc.sleeping && !ft.ic_reset);
 dev.cmds.clear();assert(!tcm_recover() && tc.sensing && (cmd_codes()==std::vector<unsigned>{0x20,0x25,0x26,0x25}));
 assert(std::find(tlmm_w.begin(),tlmm_w.end(),std::make_pair(0xa004u,0u))!=tlmm_w.end());
 dev.cmds.clear();assert(tcm_recover()==EBUSY && dev.cmds.empty());   // once a minute
 clock_now+=61000000;assert(!tcm_recover() && dev.cmds.size()==4);
 // fuxitstale releases contacts after a report gap; "no touch reports" fact after 10 s.
 bootline="fuxitstale=100";setup();report({{0,1,5,5}});assert(!tcm_poll(true) && ft.active==1);
 clock_now+=150000;assert(!tcm_poll(true) && ft.active==0);
 setup();tc.touch_seen=false;tc.sense_abs=clock_now;clock_now+=11000000;assert(!tcm_poll(true));
 assert(logged("salami-touch: no touch reports yet"));
 // Scripts: flash commands refused raw or not, unknown pins refused, ops logged.
 bootline="";adt["fuxitscr1"]={1,1,0x27, 2,3,0x12,0,0, 1,1,0x1f, 3,1,4, 4,2,10,0, 5,5,24,0,0,8,1, 5,5,30,0,0,8,1, 9,0};
 setup();dev.cmds.clear();dev.writes.clear();tcm_script(1,fk.scr1);
 assert((cmd_codes()==std::vector<unsigned>{0x27}) && dev.writes.size()==1);
 assert(logged("salami-touch: scr1 op1 -> 0") && logged("salami-touch: scr1 op2 -> 1") && logged("salami-touch: scr1 read 4 B: a5000000"));
 assert(logged("salami-touch: scr1 op5 -> 0") && logged("salami-touch: scr1 op5 -> 22") && logged("salami-touch: scr1 op9 unknown"));
 // Config commit/config id/SPI master by script, as a command (op1) or raw (op2): refused.
 adt["fuxitscr1"]={1,1,0x28, 1,3,0x2b,0x41,0x41, 2,3,0x28,0,0, 2,4,0x15,0x01,0x00,0x00, 1,1,0x43, 2,3,0x2b,0,0, 1,1,0x29, 0};
 setup();dev.cmds.clear();dev.writes.clear();tcm_script(1,fk.scr1);
 assert((cmd_codes()==std::vector<unsigned>{0x29}) && dev.writes.size()==1);
 assert(std::count(loglines.begin(),loglines.end(),std::string("salami-touch: scr1 op1 -> 1\n"))==3);
 assert(std::count(loglines.begin(),loglines.end(),std::string("salami-touch: scr1 op2 -> 1\n"))==3);
 assert(logged("salami-touch: cmd 28 refused") && logged("salami-touch: cmd 2b refused") && logged("salami-touch: cmd 43 refused"));
 adt.clear();ft_report(0);
 puts("PASS: detection (protocol/GSI-only/forced engine/backend), self-reset resume (3 attempts), deep sleep (identify under sleep/wake resumes), recovery power cycle, stale guard, quiet fact, scripts (commit/config-id/SPI-master refused)");
}

static void t_randomized() {
 static const unsigned classes[8]={1,1,1,2,6,9,0,5};
 bootline="";adt.clear();setup();uint32_t seed=777;
 for(unsigned n=0;n<2000;n++) {
  std::vector<Obj> o;uint32_t want=0;
  for(unsigned id=0;id<10;id++) {
   seed=seed*1664525+1013904223;if((seed>>28)<9)continue;
   seed=seed*1664525+1013904223;unsigned cls=classes[(seed>>16)&7];
   seed=seed*1664525+1013904223;unsigned x=(seed>>8)%6000, y=(seed>>3)%13000;
   o.push_back({id,cls,x,y});if((0x12eu>>cls)&1u)want|=1u<<id;
  }
  report(o);assert(!tcm_poll(true));
  assert(ft.active==want);
  for(unsigned id=0;id<10;id++) {
   assert(service.fDown[id]==bool(want&(1u<<id)));
   if(want&(1u<<id))assert(ft.x[id]>=0 && ft.x[id]<=1079 && ft.y[id]>=0 && ft.y[id]<=2411);
  }
 }
 ft_report(0);
 puts("PASS: 2,000 randomized TouchComm snapshots: HID down set equals the delivered class mask, coordinates in 1080x2412");
}

int main() {
 setvbuf(stdout,nullptr,_IONBF,0);
 t_framing();t_command_write();t_cmd();t_bringup();t_parser();t_contacts();t_coalescing();t_wake();
 t_mapping();t_knobs();t_gpi_vectors();t_gpi_errors();t_fifo();t_fifo_depth();t_detect_power_resume();t_randomized();
 assert(!live && !lock_depth);
}
'''

# fuxi_touch_init on both boards: thread start stand-ins, the real XNU-rule boot-arg parser.
INIT_TEST = r'''
using thread_t = int *;
using wait_result_t = int;
using thread_continue_t = void (*)(void *, wait_result_t);
#define KERN_SUCCESS 0
static int thread_obj, starts, deallocs;
static void ft_thread(void *, wait_result_t) {}
static int kernel_thread_start_priority(thread_continue_t fn, void *arg, int prio, thread_t *t) {
 assert(fn==ft_thread && !arg && prio==FT_PRIORITY);starts++;*t=&thread_obj;return KERN_SUCCESS;
}
static void thread_deallocate(thread_t t) { assert(t==&thread_obj);deallocs++; }
void fuxi_touch_init(void);
'''

INIT_MAIN = r'''
static void init_case(const char *line, bool start, bool off_line) {
 bootline=line;adt.clear();ft={};tc={};fk={};loglines.clear();starts=deallocs=0;
 fuxi_touch_init();
 assert(ft.started && starts==(int)start && deallocs==(int)start);
 // Nothing but the boot-arg lookup: no register map, read or write of any block.
 assert(tlmm.empty() && tlmm_w.empty() && se.empty() && gcc.empty() && smmu.empty() && gpi.reg.empty());
 unsigned lines=0;
 for(auto &l:loglines)lines+=l.rfind("salami-touch: ",0)==0;
 assert(lines==(off_line?1u:0u) && logged("boot-arg fuxitouch=1 enables it")==off_line);
 fuxi_touch_init();assert(starts==(int)start && loglines.size()==lines);   // once per boot
}
int main() {
 setvbuf(stdout,nullptr,_IONBF,0);
#if defined(FUXI_SALAMI)
 init_case("",false,true);
 init_case("fuxitouch=0",false,true);
 init_case("fuxitouchdelay=5 fuxitouch_poll_us=2000 fuxitaptowake=1",false,true);
 init_case("fuxitouch=1",true,false);
 init_case("fuxitouch",true,false);   // bare boot-arg is 1 (XNU rule)
 init_case("fuxitouch=0 fuxitouch=1",true,false);
 puts("PASS: salami: touch thread off by default (no register access, one salami-touch line), fuxitouch=1 starts it");
#else
 init_case("",true,false);
 init_case("fuxitouch=0",false,false);
 init_case("fuxitouch=1",true,false);
 puts("PASS: fuxi: touch thread on by default, fuxitouch=0 keeps it off, no salami-touch line");
#endif
}
'''

TCM_FUNCS = (
    'ft_gpio', 'ft_gpio_ctl', 'ft_on', 'ft_ahb', 'ft_clock', 'ft_dma_sync', 'ft_state', 'ft_gpi_cmd',
    'ft_smmu', 'ft_gpi_init', 'ft_se_gsi', 'ft_tre', 'ft_ring_db', 'ft_i2c_event', 'ft_gpi_wait',
    'ft_c0', 'ft_us', 'ft_contact_end', 'ft_report', 'ft_retry_report', 'ft_knob', 'ft_knob_blob',
    'ft_config', 'ft_detect', 'ft_smmu_cb', 'ft_i2c_timing', 'ft_i2c_budget', 'ft_se_fifo',
    'ft_fifo_pop', 'ft_fifo_stop', 'ft_i2c_fifo', 'ft_i2c_gpi_write', 'ft_i2c_gpi_read', 'tcm_hex',
    'ft_i2c', 'ft_i2c_write', 'ft_i2c_read', 'ft_bus_reinit', 'ft_irq_line', 'tcm_flash_cmd',
    'tcm_le16', 'tcm_bits', 'tcm_cfg_len', 'tcm_cfg_skip', 'tcm_cfg_body', 'tcm_parse', 'tcm_map',
    'tcm_other', 'tcm_read_msg', 'tcm_write_cmd', 'tcm_take_identify', 'tcm_log_identify', 'tcm_cmd',
    'tcm_app_mode', 'tcm_app_info', 'tcm_set_config', 'tcm_script_gpio', 'tcm_script', 'tcm_extra',
    'tcm_setup', 'tcm_power', 'tcm_wait_attn', 'tcm_bringup', 'tcm_start', 'tcm_recover',
    'tcm_resume', 'tcm_stats', 'tcm_dispatch', 'tcm_poll', 'tcm_display', 'tcm_out_space',
    'tcm_init_bus')

# Every device-only question of the design (section 8) has its once-only fact line.
FACTS = (
    'se4 fw=%#x proto=%u if_disable=%#x status=%#x hwp0=%#x fifo_depth=%u hwp2=%#x rtl=%u',
    'clk level=%u dfs_en=%u div=%u t=%u/%u/%u nominal_khz=%u', 'bus est_khz=%llu from %llu B in %llu us',
    'smmu nsmr=%u dwc3_s2cr=%#x gpi_smr=%d qup_smr=%d s2cr_used=%#x', 'pins gpio%u ctl=%#x io=%#x',
    'ios before=%#x after=%#x', 'attn after reset %u ms', 'attn not seen in %u ms', 'first read %u B: %s',
    'identify len=%u ver=%u mode=%#x part=\\"%s\\" build=%u maxwr=%u',
    'app len=%u ver=%u status=%u cfgmax=%u paymax=%u id=\\"%s\\" maxx=%u maxy=%u ',
    'objs=%u rows=%u cols=%u hybrid=%u', 'boot status=%#x reset_reason=%#x', 'span %ux%u (%s)',
    'fw cfg len=%u %s', 'set cfg mode=%u len=%u -> %#x; active len=%u %s', 'idle read %s',
    'attn after read high=%u', 'first touch report after %llu ms sensing', 'no touch reports yet',
    'contact %u raw=%u,%u cls=%u -> %d,%d',
    'contact stats reports=%u in %llu ms gap_max=%u ms raw_max=%u,%u classes=%#x',
    'bus err kind=%s ev=%#x %#x gp_len=%u ch_state=%#x/%#x', 'gpi read ok flags=%#x',
    'ic reset #%u mode=%#x', 'code %#x len %u (first)', 'out %ux%u (%s) off %d,%d', 'pad err #%u last=%#x')


def static_checks(touch):
    """Textual contracts: fact lines, flash safety, no RPMh from XNU, wake-filter call sites."""
    assert '#define TC_LOG(...)  printf("salami-touch: " __VA_ARGS__)' in touch
    for fact in FACTS:
        assert fact in touch, fact
    assert touch.count('fuxi_wake_touch_reset(&ft.wake)') == 2
    assert touch.count('fuxi_wake_touch_event(&ft.wake, !deliver') == 2
    for word in ('rh_send', '0x40800', '0x40900', 'rpmh', 'RPMH_'):
        assert word not in touch, word
    # No flash/bootloader/config-commit/config-id/SPI-master command code is defined or passed
    # literally to the command layer, and tcm_flash_cmd refuses exactly that set.
    flash = {0x11, 0x12, 0x13, 0x15, 0x16, 0x1f, 0x28, 0x2b, 0x43}
    refused = {int(v, 16) for v in re.findall(r'cmd == (0x[0-9a-f]+)u', function(touch, 'tcm_flash_cmd'))}
    assert refused == flash, sorted(refused ^ flash)
    defs = dict(re.findall(r'^#define (TCM_C_\w+)\s+(0x[0-9a-fA-F]+)u', touch, re.M))
    assert defs and not {int(v, 16) for v in defs.values()} & flash, defs
    for call in re.findall(r'tcm_(?:cmd|write_cmd)\(\s*([^,]+),', touch):
        call = call.strip()
        if call.startswith('0x'):
            assert int(call.rstrip('u'), 16) not in flash, call
        else:
            assert call in defs or call in ('cmd', 'c', 'd[0]', 'unsigned cmd'), call
    assert 'tcm_flash_cmd(cmd)' in function(touch, 'tcm_write_cmd')
    assert 'tcm_flash_cmd(d[0])' in function(touch, 'tcm_script')
    # The salami build refuses FTS before any register write, and thread start stays as before.
    thread = function(touch, 'ft_thread')
    assert thread.index('salami && fk.ic == 1u') < thread.index('ft_sleep(delay_s * 1000u)')
    assert thread.index('salami && !tc.on') < thread.index('ft_smmu(4)')
    assert thread.index('ft_clock(2, fk.lvl)') < thread.index('ft_detect()')
    # salami: the thread is off unless fuxitouch=1 (default 0 set before the boot-arg is read);
    # fuxi keeps default 1. A flag, not an early return inside #if (-Werror unreachable code).
    init = function(touch, 'fuxi_touch_init')
    block = '#if defined(FUXI_SALAMI)\n\tsalami = true;\n\tenabled = 0;\n#endif\n'
    assert 'int enabled = 1;' in init and init.count('#if') == 1 and block in init
    assert init.index(block) < init.index('PE_parse_boot_argn("fuxitouch"') < init.index('kernel_thread_start_priority')


def defines(touch):
    head = touch[touch.index('#define FT_SE_PA'):touch.index('static struct {')]
    dma = touch[touch.index('#define FT_EV_OFF'):touch.index('#define GPI_GPII')]
    dma += touch[touch.index('#define GPI_GPII'):].split('\n', 1)[0] + '\n'
    return head + dma


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xnu-buildtree', type=Path, help='optional saved XNU tree for full-TU syntax checks')
    args = parser.parse_args()
    subprocess.run(['python3', str(ROOT / 'tools/sync-fuxi-h1.py'), '--check'], check=True)
    hid = (SRC / 'SweetHID.cpp').read_text()
    touch = (SRC / 'fuxi_touch.c').read_text()
    state = touch[touch.index('static struct {'):touch.index('} ft;') + len('} ft;')]
    methods = ''.join(function(hid, name) for name in (
        'SweetHIDTouch::touch', 'SweetHIDTouch::report',
        'sweet_hid_touch', 'sweet_hid_touch_report'))
    driver = ''.join(function(touch, name) for name in (
        'ft_us', 'ft_contact_end', 'ft_report', 'ft_retry_report', 'ft_poll'))
    assert 'unsigned batch;' in state
    assert 'PE_parse_boot_argn("fuxitbatch", &ft.batch, sizeof(ft.batch))' in touch
    assert 'ft.batch =' not in touch  # static-zero default; only boot parsing enables it
    assert 'multi %u max %u lifts %u' in touch
    assert function(touch, 'ft_thread').count('ft_retry_report();') == 1
    static_checks(touch)
    states = touch[touch.index('static struct {'):touch.index('} fk;') + len('} fk;')]
    states = states.replace('static int ft_chan;\n', '')
    tcm = ''.join(function(touch, name) for name in TCM_FUNCS)
    # ft_ring_db's arm64 barrier becomes a compiler barrier on the host.
    tcm = tcm.replace('__asm__ volatile("dsb sy" ::: "memory");', '__asm__ volatile("" ::: "memory");')
    flags = ['-std=c++17', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
             '-fno-sanitize-recover=all']
    with tempfile.TemporaryDirectory(prefix='fuxi-touch-test-') as tmp:
        folder = Path(tmp)
        (folder / 'fuxi_key_state.h').write_text((SRC / 'fuxi_key_state.h').read_text())
        source, binary = folder / 'touch.cpp', folder / 'touch'
        source.write_text(MODEL_HID + MODEL_FTS + state + methods + driver + TEST)
        subprocess.run(['clang++', *flags, str(source), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        source, binary = folder / 'tcm.cpp', folder / 'tcm'
        source.write_text('#define FUXI_SALAMI 1\n' + MODEL_HID + TCM_MODEL + defines(touch) + states +
                          methods + TCM_TEST + tcm + TCM_MAIN)
        subprocess.run(['clang++', *flags, str(source), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        # Thread start policy per board; the model's unused helpers are not under test here.
        init = function(touch, 'fuxi_touch_init')
        for board, head in (('salami', '#define FUXI_SALAMI 1\n'), ('fuxi', '')):
            source, binary = folder / ('init-' + board + '.cpp'), folder / ('init-' + board)
            source.write_text(head + MODEL_HID + TCM_MODEL + defines(touch) + states + INIT_TEST +
                              init + INIT_MAIN)
            subprocess.run(['clang++', *flags, '-Wno-unused-function', '-Wno-unused-variable',
                            '-Wno-unused-const-variable', str(source), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
    print('PASS: fact lines, flash-command safety, no RPMh, wake filter call sites, salami FTS refusal order')
    if args.xnu_buildtree:
        kernel_syntax(args.xnu_buildtree.resolve())


def kernel_syntax(buildtree):
    """Check our full TUs with saved 8796 headers/flags; run no tree scripts."""
    config = buildtree / 'build/xnu.obj/DEVELOPMENT_ARM64_FUXI'
    if not config.exists():
        config = buildtree / 'build/xnu.obj/DEVELOPMENT_ARM64_SWEET'
    with tempfile.TemporaryDirectory(prefix='fuxi-touch-headers-') as temp:
        temp = Path(temp)
        for name in ('fuxi_touch.c', 'fuxi_key_state.h', 'SweetHID.cpp'):
            (temp / name).write_bytes((SRC / name).read_bytes())
        for component, name in (('bsd', 'fuxi_touch.c'), ('iokit', 'SweetHID.cpp')):
            cwd = config / component / 'DEVELOPMENT'
            old_root = re.search(r'^SOURCE_DIR=(.+)/xnu$', (cwd / 'Makefile').read_text(), re.M)[1]
            raw = (cwd / '.CFLAGS').read_text().replace('\0', '')
            raw = re.sub(r'(-DCONFIG_NMBCLUSTERS=\(\(1024 \* 512\) / MCLBYTES\))',
                         lambda m: shlex.quote(m[1]), raw)
            saved = shlex.split(raw)[1:]
            if component == 'iokit':
                saved += shlex.split((cwd / '.CXXFLAGS').read_text().replace('\0', ''))[1:]
            command = ['clang++' if component == 'iokit' else 'clang',
                       '--target=arm64-apple-macos14', '-fsyntax-only', '-DFUXI',
                       '-mgeneral-regs-only', '-Werror', '-Wno-c99-designator', '-Wno-reorder-init-list',
                       '-D__builtin_xnu_type_signature(...)=""', '-D__builtin_xnu_type_summary(...)=0',
                       '-D__builtin_xnu_types_compatible(...)=1',
                       '-I' + str(buildtree / 'xnu/iokit/Kernel/arm/hid')]
            skip = False
            for arg in saved:
                if skip:
                    skip = False
                    continue
                if arg == '-arch':
                    skip = True
                    continue
                arg = arg.replace(old_root, str(buildtree))
                arg = re.sub(r'-I.*?\.sdk/+System/Library/Frameworks/Kernel\.framework/Versions/A/PrivateHeaders/platform',
                             '-I' + str(buildtree / 'sdk-kernel-platform/platform'), arg)
                command.append(arg)
            subprocess.run(command + [str(temp / name)], cwd=cwd, check=True)
            print('PASS: complete ' + name + ' arm64 syntax against saved XNU headers/flags')


if __name__ == '__main__':
    main()
