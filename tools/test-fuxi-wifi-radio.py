#!/usr/bin/env python3
"""K2 host tests: the radio scan handoff and the fuxiwifiios=3/4 backends. No phone, no radio, no network.

In WSL: CC=cc python3 -B tools/test-fuxi-wifi-radio.py

Production C of xnu/fuxi-wifi-ios (#include lines stripped) on the model of tools/test-fuxi-wifi-ios.py
(imported, not copied), one translation unit, -Wall -Wextra -Werror, ASan/UBSan, pthreads; a Fuxi and a
salami build. Each scenario is a process of its own (the radio state is one word for the boot):
  T1/T2 parse     frames into records (every drop reason, DS and HT corrections, hidden names), elements
                  kept whole within a room, the name and lock first, in their order
  T3 fuzz         20000 seeded mutations of a beacon through the parser and the element fit
  T4 table        dedupe by BSSID, a hidden beacon keeps a probe response's name, 70 networks -> the 64
                  strongest, a swap during a walk ends it, failed scans leave nothing, the pre-parsed path
  T5 radio        fuxiwifiios=3 with a fake radio lane: before READY, the 38 passive channels, a scan forced
                  passive, GET 11 field by field (ages from the model clock), rooms 100/0/short, {36},
                  outside the list, joins refused (result 2, event 9, no link), 3 timeouts -> Wi-Fi off
     stop         the radio lane stops mid-scan: the waiter is woken, ENETDOWN at once, nothing heard, off
     cancel       Wi-Fi turned off during a scan: ECANCELED, the producer sees the cancel; on again works
     early        the radio lane said NOT_REQUESTED before init: off once, after attach
     silent-*     no producer in this kernel: off when iOS first asks (a scan, the state) or after 30 s
     rearm        READY lands while the backstop re-arms (on XNU the re-arm replaces that entry): told at once
     nochannel    ready() names no listed channel: Wi-Fi off with code 0x7000n, not on with an empty list
  T7 canned       fuxiwifiios=4: four canned networks (one hidden, WPA2, DFS), the radio lane ignored
  T8 default      no fuxiwifiios: producer calls only store the state word; nothing allocated or said
  T6 static       the leaf lock rule, the producer never sleeps, no wl_ symbol on the iOS side, and any
                  fuxi_wlan.c prototype of the producer API equals the header (none yet: 0099 hooks deferred)
  T9 mutants      17 source mutations, each must fail a scenario or a static rule
"""
import importlib.util
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'xnu/fuxi-wifi-ios'
WLAN = ROOT / 'xnu/fuxi-wlan/fuxi_wlan.c'
CC = os.environ.get('CC', 'cc')
FLAGS = ['-std=gnu11', '-g', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function', '-Wno-unused-parameter',
         '-pthread']
SAN = ['-fsanitize=address,undefined', '-fno-sanitize-recover=all']
SCENARIOS = ('parse', 'fuzz', 'table', 'radio', 'stop', 'cancel', 'early', 'silent-scan', 'silent-state', 'backstop',
             'rearm', 'nochannel', 'canned', 'default')
RESULTS = []

spec = importlib.util.spec_from_file_location('test_wifi_ios', ROOT / 'tools/test-fuxi-wifi-ios.py')
W = importlib.util.module_from_spec(spec)
spec.loader.exec_module(W)

TEST = r'''
extern unsigned int alarm(unsigned int seconds);   /* (not <unistd.h>: its link() would meet fuxi_wifi_ios.c's) */
/* ---- what every scenario uses ---------------------------------------------------------------------- */
bool fuxi_wifi_net_attach(const struct fuxi_wifi_net_link *p) {
 assert(p->wireless && !p->up && p->address[0]==2 && !strcmp(p->name,"radio-scan")); attached++; return true;
}
static void settle(void) {
 struct timespec pause = { .tv_nsec=1000000 };
 for (unsigned i=0;i<5000;i++) { if (!fuxi_wifi_sta_busy()) return; nanosleep(&pause,NULL); }
 assert(!"station timeout");
}
static int request_with(bool set,int type,int value,void *p,unsigned n,int *val) {
 struct apple80211req req = { .req_if_name="en0",.req_type=type,.req_val=value,.req_len=n,.req_data=(uintptr_t)p };
 int error=fuxi_wifi_apple80211_ioctl(set?SIOCSA80211:SIOCGA80211,&req);
 if (val) *val=req.req_val;
 return error;
}
static int request(bool set,int type,void *p,unsigned n,int *val) { return request_with(set,type,0,p,n,val); }
static unsigned said_times(const char *what) {
 unsigned n=0;
 pthread_mutex_lock(&log_lock);
 for (const char *at=logbuf; (at=strstr(at,what))!=NULL; at+=strlen(what)) n++;
 pthread_mutex_unlock(&log_lock);
 return n;
}
static bool said(const char *what) { return said_times(what)!=0; }
static void wait_said(const char *what) {
 struct timespec pause = { .tv_nsec=1000000 };
 for (unsigned i=0;i<5000 && !said(what);i++) nanosleep(&pause,NULL);
 if (!said(what)) { fprintf(stderr,"never said: %s\n",what); abort(); }
}
static void wait_count(atomic_uint *v,unsigned want) {
 struct timespec pause = { .tv_nsec=200000 };
 for (unsigned i=0;i<25000 && atomic_load(v)<want;i++) nanosleep(&pause,NULL);
 assert(atomic_load(v)>=want);
}
/* nothing a network carries reaches the log: names (all test names begin "zq"), BSSIDs, keys */
static void clean_log(void) {
 pthread_mutex_lock(&log_lock);
 assert(!strstr(logbuf,"zq") && !strstr(logbuf,"Fuxi Canned") && !strstr(logbuf,"fuxitest1"));
 assert(!strstr(logbuf,"02:10:20") && !strstr(logbuf,"10:20:30") && !strstr(logbuf,"58:4a") && !strstr(logbuf,"584a"));
 pthread_mutex_unlock(&log_lock);
}
static unsigned power_now(void) {
 struct { uint32_t version,radios,state[4]; } p={0};
 assert(!request(false,19,&p,sizeof(p),NULL)); return p.state[0];
}
static int power_set(bool on) {
 struct { uint32_t version,radios,state[4]; } p={.version=1,.radios=1,.state={on}};
 int e=request(true,19,&p,sizeof(p),NULL); settle(); return e;
}
static struct apple80211_scan_request scan_req;
static int scan_ask(uint32_t type,unsigned channel,uint32_t band) {
 memset(&scan_req,0,sizeof(scan_req)); scan_req.version=1; scan_req.scan_type=type; scan_req.channel_count=1;
 scan_req.channels[0]=(struct apple80211_channel){ 1, channel, band };
 return request(true,10,&scan_req,sizeof(scan_req),NULL);
}
static struct apple80211_scan_result res;
static int result_get(unsigned room) { memset(&res,0,sizeof(res)); res.elements_length=(uint16_t)room; return request(false,11,&res,sizeof(res),NULL); }
static char line[512];
#define SAY(...) (snprintf(line,sizeof(line),__VA_ARGS__),line)
static uint16_t all38[38];
static void fill_all(void) {
 struct fuxi_wifi_sta_channel c[40];
 assert(fuxi_wifi_radio_channels(c,40)==38 && fuxi_wifi_radio_channels(c,3)==3 && fuxi_wifi_radio_channels(NULL,9)==0);
 for (unsigned i=0;i<38;i++) all38[i]=c[i].frequency;
}
/* frames */
struct ies { uint8_t b[2048]; unsigned n; };
static void put(struct ies *l,uint8_t id,const void *data,unsigned len) {
 assert(len<=255 && l->n+2+len<=sizeof(l->b)); l->b[l->n]=id; l->b[l->n+1]=(uint8_t)len;
 if (len) memcpy(l->b+l->n+2,data,len);
 l->n+=2+len;
}
static const uint8_t RSN[20]={ 1,0, 0x00,0x0f,0xac,4, 1,0, 0x00,0x0f,0xac,4, 1,0, 0x00,0x0f,0xac,2, 0x0c,0 };
static const uint8_t RATES[8]={ 0x82,0x84,0x8b,0x96,0x0c,0x12,0x18,0x24 };
static const uint8_t RATES5[8]={ 0x8c,0x12,0x98,0x24,0xb0,0x48,0x60,0x6c };
static size_t frame_of(uint8_t *f,uint8_t kind,const uint8_t *bssid,uint16_t capability,const struct ies *l) {
 memset(f,0,36); f[0]=kind; memset(f+4,0xff,6); memcpy(f+10,bssid,6); memcpy(f+16,bssid,6);
 f[32]=100; f[34]=(uint8_t)capability; f[35]=(uint8_t)(capability>>8);
 memcpy(f+36,l->b,l->n); return 36+l->n;
}
static bool has_element(const uint8_t *e,unsigned n,uint8_t id) {
 for (unsigned at=0; at+2<=n && at+2u+e[at+1]<=n; at+=2u+e[at+1]) if (e[at]==id) return true;
 return false;
}
/* `out` is whole elements of `in`, in their order; RSN / WPA among them */
static unsigned whole_in_order(const uint8_t *in,unsigned in_n,const uint8_t *out,unsigned out_n,bool *rsn,bool *wpa) {
 unsigned at=0,from=0,count=0; *rsn=*wpa=false;
 while (at<out_n) {
  assert(at+2<=out_n && at+2u+out[at+1]<=out_n);
  unsigned size=2u+out[at+1]; bool found=false;
  for (unsigned i=from; i+2<=in_n && i+2u+in[i+1]<=in_n; i+=2u+in[i+1]) {
   if (2u+in[i+1]==size && !memcmp(in+i,out+at,size)) { from=i+size; found=true; break; }
  }
  assert(found);
  *rsn|=out[at]==48; *wpa|=out[at]==221 && out[at+1]>=4 && !memcmp(out+at+2,"\x00\x50\xf2\x01",4);
  at+=size; count++;
 }
 return count;
}
static enum fuxi_wifi_radio_state changed_last; static atomic_uint changed_calls;
static void changed_stub(enum fuxi_wifi_radio_state s) { changed_last=s; atomic_fetch_add(&changed_calls,1); }

/* ---- T1/T2: the parser and the element fit ---------------------------------------------------------- */
static void scenario_parse(void) {
 static const uint8_t A[6]={0x02,0x10,0x20,0x30,0x40,0x01};
 struct fuxi_wifi_sta_bss *out=calloc(1,sizeof(*out)); static uint8_t f[2200]; struct ies l; size_t n; int k;
 fill_all();
 #define PARSE(len,mhz,rssi,noise) fuxi_wifi_radio_parse(f,len,mhz,rssi,noise,all38,38,out)
 memset(&l,0,sizeof(l)); put(&l,0,"zqA",3); put(&l,1,RATES,8); put(&l,3,"\x06",1); put(&l,48,RSN,20);
 n=frame_of(f,0x80,A,0x0011,&l);
 /* heard on channel 5, says 6: a record on 6, everything as received */
 assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_KEPT);
 assert(!memcmp(out->bssid,A,6) && out->ssid_length==3 && !memcmp(out->ssid,"zqA",3) && out->frequency==2437);
 assert(out->rssi==-50 && out->noise==-95 && out->beacon_interval==100 && out->capability==0x11 && !out->age_ms);
 assert(out->elements_length==l.n && !memcmp(out->elements,l.b,l.n));
 f[0]=0x50; assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_KEPT);        /* probe response */
 /* probe request, action, protected, to/from DS */
 f[0]=0x40; assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_KIND);
 f[0]=0xd0; assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_KIND);
 f[0]=0x88; assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_KIND); f[0]=0x80;
 f[1]=0x40; assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_KIND);
 f[1]=0x01; assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_KIND);
 f[1]=0x02; assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_KIND); f[1]=0;
 /* lengths */
 assert(PARSE(35,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_LENGTH && PARSE(0,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_LENGTH);
 assert(PARSE(0x10000,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_LENGTH);
 assert(fuxi_wifi_radio_parse(NULL,n,2432,-50,-95,all38,38,out)==FUXI_WIFI_RADIO_DROP_LENGTH);
 assert(fuxi_wifi_radio_parse(f,n,2432,-50,-95,all38,38,NULL)==FUXI_WIFI_RADIO_DROP_LENGTH);
 assert(PARSE(36,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_SSID);            /* the fixed fields alone: no name */
 /* BSSID: multicast, zero; IBSS */
 f[16]=0x01; assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_BSSID);
 memset(f+16,0,6); assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_BSSID); memcpy(f+16,A,6);
 f[34]=0x02; assert(PARSE(n,2432,-50,-95)==FUXI_WIFI_RADIO_DROP_IBSS); f[34]=0x11;
 /* bands: 6 GHz, below 2.4, between */
 assert(PARSE(n,5955,-50,-95)==FUXI_WIFI_RADIO_DROP_BAND && PARSE(n,2300,-50,-95)==FUXI_WIFI_RADIO_DROP_BAND);
 assert(PARSE(n,4899,-50,-95)==FUXI_WIFI_RADIO_DROP_BAND && PARSE(n,5900,-50,-95)==FUXI_WIFI_RADIO_DROP_BAND);
 /* a channel that is not the producer's */
 uint16_t only[1]={2412};
 assert(fuxi_wifi_radio_parse(f,n,2432,-50,-95,only,1,out)==FUXI_WIFI_RADIO_DROP_UNLISTED);
 assert(fuxi_wifi_radio_parse(f,n,2432,-50,-95,NULL,38,out)==FUXI_WIFI_RADIO_DROP_UNLISTED);
 /* DS channel 0, 15: out of range; 14 (2484) is not listed */
 const char *ds[3]={"\x00","\x0f","\x0e"}; int want[3]={FUXI_WIFI_RADIO_DROP_CHANNEL,FUXI_WIFI_RADIO_DROP_CHANNEL,FUXI_WIFI_RADIO_DROP_UNLISTED};
 for (k=0;k<3;k++) {
  memset(&l,0,sizeof(l)); put(&l,0,"zqA",3); put(&l,3,ds[k],1); n=frame_of(f,0x80,A,1,&l);
  assert(PARSE(n,2412,-50,-95)==want[k]);
 }
 /* the name: 33 bytes, missing, cut by the end of the frame */
 uint8_t long_name[33]; memset(long_name,'A',sizeof(long_name)); long_name[0]='z'; long_name[1]='q';
 memset(&l,0,sizeof(l)); put(&l,0,long_name,33); n=frame_of(f,0x80,A,1,&l);
 assert(PARSE(n,2412,-50,-95)==FUXI_WIFI_RADIO_DROP_SSID);
 memset(&l,0,sizeof(l)); put(&l,1,RATES,8); n=frame_of(f,0x80,A,1,&l); assert(PARSE(n,2412,-50,-95)==FUXI_WIFI_RADIO_DROP_SSID);
 memset(&l,0,sizeof(l)); put(&l,1,RATES,8); put(&l,0,"zqAAAAAA",8); n=frame_of(f,0x80,A,1,&l);
 assert(PARSE(n-3,2412,-50,-95)==FUXI_WIFI_RADIO_DROP_SSID);
 /* hidden: an empty name, and a name of zero bytes */
 memset(&l,0,sizeof(l)); put(&l,0,"",0); put(&l,48,RSN,20); n=frame_of(f,0x80,A,0x11,&l);
 assert(PARSE(n,2412,-50,-95)==FUXI_WIFI_RADIO_KEPT && out->ssid_length==0 && out->elements_length==l.n);
 memset(&l,0,sizeof(l)); put(&l,0,"\0\0\0\0\0",5); n=frame_of(f,0x80,A,1,&l);
 assert(PARSE(n,2412,-50,-95)==FUXI_WIFI_RADIO_KEPT && out->ssid_length==0);
 for (k=0;k<32;k++) assert(!out->ssid[k]);
 memset(&l,0,sizeof(l)); put(&l,0,"\0zq",3); n=frame_of(f,0x80,A,1,&l);
 assert(PARSE(n,2412,-50,-95)==FUXI_WIFI_RADIO_KEPT && out->ssid_length==3);
 /* 5 GHz: HT operation's primary channel (40, heard on 36); 0 and 180 out of range; none: where it was heard */
 uint8_t ht[22]={0}; ht[0]=40;
 memset(&l,0,sizeof(l)); put(&l,0,"zqB",3); put(&l,1,RATES5,8); put(&l,61,ht,22); n=frame_of(f,0x80,A,1,&l);
 assert(PARSE(n,5180,-60,0)==FUXI_WIFI_RADIO_KEPT && out->frequency==5200 && out->noise==0);
 f[36+5+10+2]=0; assert(PARSE(n,5180,-60,0)==FUXI_WIFI_RADIO_DROP_CHANNEL);
 f[36+5+10+2]=180; assert(PARSE(n,5180,-60,0)==FUXI_WIFI_RADIO_DROP_CHANNEL);
 f[36+5+10+2]=144; assert(PARSE(n,5180,-60,0)==FUXI_WIFI_RADIO_KEPT && out->frequency==5720);
 memset(&l,0,sizeof(l)); put(&l,0,"zqB",3); n=frame_of(f,0x80,A,1,&l);
 assert(PARSE(n,5180,-60,0)==FUXI_WIFI_RADIO_KEPT && out->frequency==5180);
 assert(PARSE(n,2484,-60,0)==FUXI_WIFI_RADIO_DROP_UNLISTED);
 /* the order bit: an HT control field after the header */
 memset(&l,0,sizeof(l)); put(&l,0,"zqA",3); put(&l,3,"\x06",1); n=frame_of(f,0x80,A,0x0411,&l);
 memmove(f+28,f+24,n-24); memset(f+24,0xcc,4); f[1]=0x80; n+=4;
 assert(PARSE(n,2437,-50,-95)==FUXI_WIFI_RADIO_KEPT && out->capability==0x0411 && out->frequency==2437 && out->elements_length==l.n);
 /* dBm clamped to -127..0 */
 memset(&l,0,sizeof(l)); put(&l,0,"zqA",3); n=frame_of(f,0x80,A,1,&l);
 assert(PARSE(n,2412,5,7)==FUXI_WIFI_RADIO_KEPT && out->rssi==0 && out->noise==0);
 assert(PARSE(n,2412,-200,-300)==FUXI_WIFI_RADIO_KEPT && out->rssi==-127 && out->noise==-127);
 /* an element cut by the end is left out, the rest kept */
 memset(&l,0,sizeof(l)); put(&l,0,"zqA",3); put(&l,1,RATES,8); n=frame_of(f,0x80,A,1,&l);
 f[n]=48; f[n+1]=20; f[n+2]=1;
 assert(PARSE(n+3,2412,-50,0)==FUXI_WIFI_RADIO_KEPT && out->elements_length==l.n && !has_element(out->elements,out->elements_length,48));

 /* T2: 1400 bytes of elements, RSN last: at most 1024 kept, whole, in their order, RSN and WPA among them */
 static const uint8_t wpa[22]={ 0x00,0x50,0xf2,1, 1,0, 0x00,0x50,0xf2,2, 1,0, 0x00,0x50,0xf2,2, 1,0, 0x00,0x50,0xf2,2 };
 uint8_t filler[255]; memset(filler,0x5a,sizeof(filler)); filler[0]=0x00; filler[1]=0x10; filler[2]=0x18;
 uint8_t zeros[26]={0};
 struct ies big; memset(&big,0,sizeof(big));
 put(&big,0,"zqX",3); put(&big,1,RATES,8); put(&big,3,"\x01",1); put(&big,221,wpa,22); put(&big,45,zeros,26);
 put(&big,127,zeros,8);
 for (k=0;k<4;k++) put(&big,221,filler,228);
 put(&big,42,filler,22); put(&big,221,filler,250); put(&big,221,filler,100); put(&big,48,RSN,20);
 assert(big.n==1400);
 static uint8_t got_b[1500]; bool rsn,w; unsigned got;
 got=fuxi_wifi_elements_fit(big.b,big.n,got_b,1024);
 assert(got==1022 && whole_in_order(big.b,big.n,got_b,got,&rsn,&w)==11 && rsn && w);
 assert(got_b[got-22]==48);                                           /* RSN, still last */
 /* room 100: the name, RSN and WPA first, then rates, channel, HT while they fit */
 got=fuxi_wifi_elements_fit(big.b,big.n,got_b,100);
 assert(got==92 && whole_in_order(big.b,big.n,got_b,got,&rsn,&w)==6 && rsn && w);
 assert(!has_element(got_b,got,127) && has_element(got_b,got,45));
 got=fuxi_wifi_elements_fit(big.b,big.n,got_b,60);
 assert(got==54 && whole_in_order(big.b,big.n,got_b,got,&rsn,&w)==4 && rsn && w && has_element(got_b,got,3));
 got=fuxi_wifi_elements_fit(big.b,big.n,got_b,50);       /* (in their order: WPA before RSN, and only one fits) */
 assert(got==42 && whole_in_order(big.b,big.n,got_b,got,&rsn,&w)==4 && !rsn && w);
 /* room 0: nothing; room for all: all of it, as it was */
 assert(fuxi_wifi_elements_fit(big.b,big.n,got_b,0)==0);
 assert(fuxi_wifi_elements_fit(big.b,big.n,got_b,1400)==1400 && !memcmp(got_b,big.b,1400));
 assert(fuxi_wifi_elements_fit(big.b,big.n-1,got_b,1400)==1378);        /* RSN cut by one byte: left out */
 assert(fuxi_wifi_elements_fit(NULL,9,got_b,9)==0 && fuxi_wifi_elements_fit(big.b,9,NULL,9)==0);
 /* WMM and HE operation count as the second rank, other vendors and extensions as the third */
 struct ies v; memset(&v,0,sizeof(v));
 put(&v,221,"\x00\x10\x18\x01",4); put(&v,255,"\x10\x00",2); put(&v,221,"\x00\x50\xf2\x02\x01",5); put(&v,255,"\x24\x00",2);
 got=fuxi_wifi_elements_fit(v.b,v.n,got_b,12);
 assert(got==11 && got_b[0]==221 && got_b[4]==0xf2 && got_b[5]==2 && got_b[7]==255 && got_b[9]==0x24);
 got=fuxi_wifi_elements_fit(v.b,v.n,got_b,v.n-1);
 assert(got==17 && got_b[0]==221 && got_b[4]==0x18 && got_b[6]==221 && got_b[13]==255);
 free(out);
 puts("parse: PASS");
}

/* ---- T3: fuzz ------------------------------------------------------------------------------------------- */
static uint32_t rng=0x4b32;
static uint32_t rnd(void) { rng=rng*1103515245u+12345u; return rng>>8; }
static void scenario_fuzz(void) {
 static const uint8_t A[6]={0x02,0x10,0x20,0x30,0x40,0x02};
 static const uint16_t mhz[12]={2412,2437,2462,2484,5180,5260,5500,5825,5955,6000,2300,4900};
 struct fuxi_wifi_sta_bss *out=calloc(1,sizeof(*out)); static uint8_t base[800],f[900],room_out[1100];
 struct ies l; memset(&l,0,sizeof(l));
 uint8_t pad[120]; memset(pad,0x33,sizeof(pad)); pad[0]=0x00; pad[1]=0x10; pad[2]=0x18;
 uint8_t ht[26]={0}; ht[0]=36;
 put(&l,0,"zqFuzzNet",9); put(&l,1,RATES,8); put(&l,3,"\x06",1); put(&l,7,"IL \x01\x0d\x14",6); put(&l,48,RSN,20);
 put(&l,45,ht,26); put(&l,61,ht,22); put(&l,127,ht,8); put(&l,221,"\x00\x50\xf2\x01\x01\x00",6);
 put(&l,221,"\x00\x50\xf2\x02\x01\x01",6); put(&l,255,"\x23\x01\x02",3); put(&l,221,pad,120);
 size_t base_n=frame_of(base,0x80,A,0x0411,&l);
 fill_all();
 unsigned kept=0;
 for (unsigned i=0;i<20000;i++) {
  size_t n=base_n; memcpy(f,base,n);
  for (unsigned m=1+rnd()%4; m; m--) {
   switch (rnd()%6) {
   case 0: if (n) f[rnd()%n]^=(uint8_t)(1u<<(rnd()%8)); break;
   case 1: if (n>36) f[36+rnd()%(n-36)]=(uint8_t)rnd(); break;
   case 2: n=rnd()%(n+1); break;
   case 3: for (unsigned j=2+rnd()%9; j && n<sizeof(f); j--) f[n++]=(uint8_t)rnd(); break;
   case 4: f[1]=(uint8_t)(rnd()%3==0 ? rnd() : 0); break;
   default: f[0]=(uint8_t)(rnd()%2 ? 0x80 : rnd()); break;
   }
  }
  int16_t rssi=(int16_t)((int)(rnd()%300)-200);
  int why=fuxi_wifi_radio_parse(f,n,mhz[rnd()%12],rssi,-95,all38,38,out);
  assert(why>=0 && why<FUXI_WIFI_RADIO_DROP_REASONS && why!=FUXI_WIFI_RADIO_DROP_STALE && why!=FUXI_WIFI_RADIO_DROP_FULL);
  if (why==FUXI_WIFI_RADIO_KEPT) {
   bool ok=false,r,w; kept++;
   for (unsigned c=0;c<38;c++) ok|=all38[c]==out->frequency;
   assert(ok && out->ssid_length<=32 && out->rssi>=-127 && out->rssi<=0 && !(out->bssid[0]&1) && (out->capability&1));
   assert(out->elements_length<=1024 && n>=36);
   (void)whole_in_order(f+(f[1]&0x80?40:36),(unsigned)n-(f[1]&0x80?40u:36u),out->elements,out->elements_length,&r,&w);
  }
  unsigned room=rnd()%1100, got=fuxi_wifi_elements_fit(f,(unsigned)n,room_out,room);
  bool r,w;
  assert(got<=room); (void)whole_in_order(f,(unsigned)n,room_out,got,&r,&w);
 }
 assert(kept>1000);
 free(out);
 printf("fuzz: PASS (20000 mutations, %u kept)\n",kept);
}

/* ---- T4: the table, through the API itself (no iOS side) ----------------------------------------------- */
static struct fuxi_wifi_radio_request consumer_what; static atomic_int consumer_status; static atomic_bool consumer_done;
static void *consumer_run(void *a) {
 (void)a; atomic_store(&consumer_status,fuxi_wifi_radio_scan(&consumer_what,15000,NULL)); atomic_store(&consumer_done,true);
 return NULL;
}
static pthread_t consumer_start(void) {
 pthread_t t; memset(&consumer_what,0,sizeof(consumer_what));
 memcpy(consumer_what.freqs,all38,sizeof(all38)); consumer_what.freq_count=38; consumer_what.dwell_ms=150;
 atomic_store(&consumer_done,false); assert(!pthread_create(&t,NULL,consumer_run,NULL)); return t;
}
static uint32_t take_wait(struct fuxi_wifi_radio_request *r) {
 struct timespec pause = { .tv_nsec=200000 };
 for (unsigned i=0;i<25000;i++) { if (fuxi_wifi_radio_take(r)) return r->seq; nanosleep(&pause,NULL); }
 abort();
}
struct collected { unsigned n; struct fuxi_wifi_sta_bss b[64]; };
static void collect(const struct fuxi_wifi_sta_bss *bss,void *c) { struct collected *k=c; assert(k->n<64); k->b[k->n++]=*bss; }
static uint32_t swap_seq; static unsigned swap_calls;
static void swap_each(const struct fuxi_wifi_sta_bss *bss,void *c) {
 (void)bss; (void)c; if (swap_calls++==0) fuxi_wifi_radio_done(swap_seq,0);
}
static void scenario_table(void) {
 static uint8_t f[600]; struct ies l; size_t n; struct fuxi_wifi_radio_request r; uint32_t seq; pthread_t t;
 struct collected *k=calloc(1,sizeof(*k));
 static const uint8_t A[6]={0x02,0x10,0x20,0x30,0x40,0x01}, B[6]={0x02,0x10,0x20,0x30,0x40,0x02};
 fill_all();
 assert(fuxi_wifi_radio_use(false,changed_stub) && !fuxi_wifi_radio_use(false,changed_stub));
 fuxi_wifi_radio_attached(); other_settle();
 fuxi_wifi_radio_starting(); fuxi_wifi_radio_ready(all38,38); other_settle();
 assert(said("FUXI WIFI IOS: radio: radio producer ready, 38 channels (13 at 2.4 GHz, 25 at 5 GHz), passive only\n"));
 assert(changed_last==FUXI_WIFI_RADIO_READY && atomic_load(&changed_calls)==1);
 /* scan 1: the same BSSID twice (the newer frame wins), a hidden beacon after a named probe response */
 t=consumer_start(); seq=take_wait(&r);
 assert(r.freq_count==38 && r.passive && r.dwell_ms==150 && !fuxi_wifi_radio_cancelled(seq) && !fuxi_wifi_radio_take(&r));
 memset(&l,0,sizeof(l)); put(&l,0,"zqA",3); put(&l,3,"\x01",1); n=frame_of(f,0x80,A,1,&l);
 assert(fuxi_wifi_radio_frame(seq,f,n,2412,-70,-95)==FUXI_WIFI_RADIO_KEPT);
 put(&l,48,RSN,20); n=frame_of(f,0x80,A,0x11,&l);
 assert(fuxi_wifi_radio_frame(seq,f,n,2412,-40,-95)==FUXI_WIFI_RADIO_KEPT);
 memset(&l,0,sizeof(l)); put(&l,0,"zqB",3); n=frame_of(f,0x50,B,1,&l);
 assert(fuxi_wifi_radio_frame(seq,f,n,5180,-50,-95)==FUXI_WIFI_RADIO_KEPT);
 memset(&l,0,sizeof(l)); put(&l,0,"",0); n=frame_of(f,0x80,B,1,&l);
 assert(fuxi_wifi_radio_frame(seq,f,n,5180,-45,-95)==FUXI_WIFI_RADIO_KEPT);
 assert(fuxi_wifi_radio_frame(seq+1,f,n,5180,-45,-95)==FUXI_WIFI_RADIO_DROP_STALE);
 assert(fuxi_wifi_radio_frame(seq,f,n,5955,-45,-95)==FUXI_WIFI_RADIO_DROP_BAND);
 fuxi_wifi_radio_done(seq+1,0); assert(!fuxi_wifi_radio_cancelled(seq));
 fuxi_wifi_radio_done(seq,0); pthread_join(t,NULL); assert(atomic_load(&consumer_status)==0);
 assert(fuxi_wifi_radio_cancelled(seq));
 assert(fuxi_wifi_radio_results(collect,k)==2 && k->n==2);
 assert(!memcmp(k->b[0].bssid,A,6) && k->b[0].rssi==-40 && k->b[0].capability==0x11 && has_element(k->b[0].elements,k->b[0].elements_length,48));
 assert(!memcmp(k->b[1].bssid,B,6) && k->b[1].ssid_length==3 && !memcmp(k->b[1].ssid,"zqB",3) && k->b[1].rssi==-45);
 assert(said("FUXI WIFI IOS: radio: scan 1: 38 channels passive, 2 heard (1 at 2.4 GHz, 1 at 5 GHz, 1 RSN, 0 hidden), 0 ms, status 0\n"));
 assert(said("FUXI WIFI IOS: radio: scan 1: 5 frames, 1 not kept (length 0, kind 0, BSSID 0, IBSS 0, SSID 0, band 1, channel 0, unlisted 0, full 0)\n"));
 /* scan 2: 70 networks from the strongest down, then a stronger one: the 64 strongest stay */
 t=consumer_start(); seq=take_wait(&r);
 memset(&l,0,sizeof(l)); put(&l,0,"zqN",3); put(&l,3,"\x01",1);
 for (unsigned i=0;i<71;i++) {
  uint8_t id[6]={0x02,0x77,0,0,0,(uint8_t)i};
  n=frame_of(f,0x80,id,1,&l);
  int16_t rssi=(int16_t)(i<70 ? -30-(int)i : -20);
  assert(fuxi_wifi_radio_frame(seq,f,n,2412,rssi,-95)==(i<64 || i==70 ? FUXI_WIFI_RADIO_KEPT : FUXI_WIFI_RADIO_DROP_FULL));
 }
 fuxi_wifi_radio_done(seq,0); pthread_join(t,NULL); assert(atomic_load(&consumer_status)==0);
 k->n=0; assert(fuxi_wifi_radio_results(collect,k)==64);
 int weakest=0, strongest=-127;
 for (unsigned i=0;i<64;i++) { weakest=k->b[i].rssi<weakest?k->b[i].rssi:weakest; strongest=k->b[i].rssi>strongest?k->b[i].rssi:strongest; }
 assert(weakest==-92 && strongest==-20);
 assert(said("FUXI WIFI IOS: radio: scan 2: 38 channels passive, 64 heard (64 at 2.4 GHz, 0 at 5 GHz, 0 RSN, 0 hidden), 0 ms, status 0\n"));
 assert(said("FUXI WIFI IOS: radio: scan 2: 71 frames, 6 not kept (length 0, kind 0, BSSID 0, IBSS 0, SSID 0, band 0, channel 0, unlisted 0, full 6)\n"));
 /* scan 3: a new scan swapped in while the last one's records are walked: the walk ends there */
 t=consumer_start(); seq=take_wait(&r);
 for (unsigned i=0;i<3;i++) { uint8_t id[6]={0x02,0x78,0,0,0,(uint8_t)i}; n=frame_of(f,0x80,id,1,&l);
  assert(fuxi_wifi_radio_frame(seq,f,n,2412,-50,-95)==FUXI_WIFI_RADIO_KEPT); }
 swap_seq=seq; swap_calls=0;
 assert(fuxi_wifi_radio_results(swap_each,NULL)==1 && swap_calls==1);
 pthread_join(t,NULL); assert(atomic_load(&consumer_status)==0);
 k->n=0; assert(fuxi_wifi_radio_results(collect,k)==3 && k->b[0].bssid[1]==0x78);
 /* a failed scan, and a negative status (EIO), leave nothing to hear */
 t=consumer_start(); seq=take_wait(&r); n=frame_of(f,0x80,A,1,&l);
 assert(fuxi_wifi_radio_frame(seq,f,n,2412,-50,-95)==FUXI_WIFI_RADIO_KEPT);
 fuxi_wifi_radio_done(seq,EIO); pthread_join(t,NULL); assert(atomic_load(&consumer_status)==EIO);
 k->n=0; assert(fuxi_wifi_radio_results(collect,k)==0);
 t=consumer_start(); seq=take_wait(&r); fuxi_wifi_radio_done(seq,-7); pthread_join(t,NULL); assert(atomic_load(&consumer_status)==EIO);
 /* a request with no listed channel asks the radio nothing; bad requests */
 consumer_what.freq_count=1; consumer_what.freqs[0]=2484;
 assert(fuxi_wifi_radio_scan(&consumer_what,100,NULL)==0 && !fuxi_wifi_radio_take(&r));
 k->n=0; assert(fuxi_wifi_radio_results(collect,k)==0);
 consumer_what.freq_count=65; assert(fuxi_wifi_radio_scan(&consumer_what,100,NULL)==EINVAL);
 assert(fuxi_wifi_radio_scan(NULL,100,NULL)==EINVAL && fuxi_wifi_radio_results(NULL,NULL)==0);
 /* records parsed elsewhere: the same rules */
 t=consumer_start(); seq=take_wait(&r);
 struct fuxi_wifi_sta_bss *b=calloc(1,sizeof(*b));
 memcpy(b->bssid,A,6); memcpy(b->ssid,"zqC",3); b->ssid_length=3; b->frequency=5260; b->rssi=10; b->noise=-95;
 b->capability=0x11; memcpy(b->elements,"\x00\x03zqC\x30\x14",7); memcpy(b->elements+7,RSN,20); b->elements[27]=1;
 b->elements_length=28;   /* the last byte: an element cut short */
 assert(fuxi_wifi_radio_bss(seq,b)==FUXI_WIFI_RADIO_KEPT);
 b->bssid[0]=1; assert(fuxi_wifi_radio_bss(seq,b)==FUXI_WIFI_RADIO_DROP_BSSID); b->bssid[0]=2;
 b->capability=0x10; assert(fuxi_wifi_radio_bss(seq,b)==FUXI_WIFI_RADIO_DROP_IBSS); b->capability=0x11;
 b->ssid_length=33; assert(fuxi_wifi_radio_bss(seq,b)==FUXI_WIFI_RADIO_DROP_SSID); b->ssid_length=3;
 b->frequency=5955; assert(fuxi_wifi_radio_bss(seq,b)==FUXI_WIFI_RADIO_DROP_BAND);
 b->frequency=2484; assert(fuxi_wifi_radio_bss(seq,b)==FUXI_WIFI_RADIO_DROP_UNLISTED); b->frequency=5260;
 b->elements_length=1025; assert(fuxi_wifi_radio_bss(seq,b)==FUXI_WIFI_RADIO_DROP_LENGTH); b->elements_length=28;
 assert(fuxi_wifi_radio_bss(seq,NULL)==FUXI_WIFI_RADIO_DROP_LENGTH);
 memcpy(b->bssid,B,6); memset(b->ssid,0,3); assert(fuxi_wifi_radio_bss(seq,b)==FUXI_WIFI_RADIO_KEPT);
 fuxi_wifi_radio_done(seq,0); pthread_join(t,NULL); assert(atomic_load(&consumer_status)==0);
 k->n=0; assert(fuxi_wifi_radio_results(collect,k)==2);
 assert(k->b[0].rssi==0 && k->b[0].noise==-95 && k->b[0].elements_length==27 && k->b[0].ssid_length==3);
 assert(k->b[1].ssid_length==0);
 assert(fuxi_wifi_radio_bss(seq,b)==FUXI_WIFI_RADIO_DROP_STALE);
 /* a stop clears what was heard, and tells the consumer once */
 fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | 9);
 k->n=0; assert(fuxi_wifi_radio_results(collect,k)==0 && fuxi_wifi_radio_state(NULL)==FUXI_WIFI_RADIO_STOPPED);
 other_settle(); assert(changed_last==FUXI_WIFI_RADIO_STOPPED && atomic_load(&changed_calls)==2);
 assert(said("FUXI WIFI IOS: radio: the radio lane stopped (code 0x40009); Wi-Fi reported off, no retry this boot\n"));
 consumer_what.freq_count=1; assert(fuxi_wifi_radio_scan(&consumer_what,100,NULL)==ENETDOWN);
 other_settle(); free(b); free(k); clean_log();
 puts("table: PASS");
}

/* ---- T5: fuxiwifiios=3 with a fake radio lane --------------------------------------------------------- */
enum { LANE_ANSWER, LANE_GATED, LANE_SILENT, LANE_HOLD };
static atomic_int lane_mode; static atomic_uint lane_takes, lane_cancels, lane_fed, lane_go; static atomic_bool lane_quit;
static pthread_mutex_t lane_lock = PTHREAD_MUTEX_INITIALIZER; static struct fuxi_wifi_radio_request lane_last;
struct lane_frame { uint8_t f[600]; size_t n; uint16_t mhz; int16_t rssi; };
static struct lane_frame lane_frames[8]; static unsigned lane_frame_count;
static bool asked_for(const struct fuxi_wifi_radio_request *r,uint16_t mhz) {
 for (unsigned i=0;i<r->freq_count;i++) if (r->freqs[i]==mhz) return true;
 return false;
}
static void lane_feed(const struct fuxi_wifi_radio_request *r) {
 for (unsigned i=0;i<lane_frame_count;i++) if (asked_for(r,lane_frames[i].mhz))
  assert(fuxi_wifi_radio_frame(r->seq,lane_frames[i].f,lane_frames[i].n,lane_frames[i].mhz,lane_frames[i].rssi,-95)==FUXI_WIFI_RADIO_KEPT);
}
static void *lane_run(void *a) {
 struct timespec pause = { .tv_nsec=200000 };
 (void)a;
 while (!atomic_load(&lane_quit)) {
  struct fuxi_wifi_radio_request r;
  if (!fuxi_wifi_radio_take(&r)) { nanosleep(&pause,NULL); continue; }
  int mode=atomic_load(&lane_mode); unsigned go=atomic_load(&lane_go);
  pthread_mutex_lock(&lane_lock); lane_last=r; pthread_mutex_unlock(&lane_lock);
  atomic_fetch_add(&lane_takes,1);
  if (mode==LANE_ANSWER || mode==LANE_GATED) { lane_feed(&r); atomic_fetch_add(&lane_fed,1); }
  if (mode!=LANE_ANSWER) {
   while (!fuxi_wifi_radio_cancelled(r.seq) && !atomic_load(&lane_quit) && atomic_load(&lane_go)==go) nanosleep(&pause,NULL);
   if (fuxi_wifi_radio_cancelled(r.seq)) {
    /* what comes after the cancel is dropped, and its end changes nothing */
    if (lane_frame_count) assert(fuxi_wifi_radio_frame(r.seq,lane_frames[0].f,lane_frames[0].n,lane_frames[0].mhz,-40,-95)==FUXI_WIFI_RADIO_DROP_STALE);
    fuxi_wifi_radio_done(r.seq,0);
    atomic_fetch_add(&lane_cancels,1);
    continue;
   }
   if (mode==LANE_HOLD) lane_feed(&r);
  }
  fuxi_wifi_radio_done(r.seq,0);
 }
 return NULL;
}
static pthread_t lane_thread;
static void lane_start(int mode) { atomic_store(&lane_mode,mode); assert(!pthread_create(&lane_thread,NULL,lane_run,NULL)); }
static void lane_end(void) { atomic_store(&lane_quit,true); pthread_join(lane_thread,NULL); }
static const uint8_t NA[6]={0x02,0x10,0x20,0x30,0x40,0x0a}, NB[6]={0x02,0x10,0x20,0x30,0x40,0x0b},
 NC[6]={0x02,0x10,0x20,0x30,0x40,0x0c}, ND[6]={0x02,0x10,0x20,0x30,0x40,0x0d};
static struct ies ies_a, ies_b, ies_c, ies_d;
static void lane_frames_make(void) {
 uint8_t filler[200]; memset(filler,0x5a,sizeof(filler)); filler[0]=0x00; filler[1]=0x10; filler[2]=0x18;
 uint8_t ht[22]={0}; ht[0]=52;
 put(&ies_a,0,"zqA-wpa2",8); put(&ies_a,1,RATES,8); put(&ies_a,3,"\x06",1); put(&ies_a,221,filler,200); put(&ies_a,48,RSN,20);
 put(&ies_b,0,"zqB-open",8); put(&ies_b,1,RATES5,8);
 put(&ies_c,0,"\0\0\0\0\0\0\0\0\0",9); put(&ies_c,1,RATES,8); put(&ies_c,3,"\x0b",1); put(&ies_c,48,RSN,20);
 put(&ies_d,0,"zqD-dfs",7); put(&ies_d,1,RATES5,8); put(&ies_d,48,RSN,20); put(&ies_d,61,ht,22);
 struct { const uint8_t *id; uint16_t cap; struct ies *l; uint16_t mhz; int16_t rssi; } list[5]={
  { NA, 0x11, &ies_a, 2432, -60 }, { NA, 0x11, &ies_a, 2437, -48 }, { NB, 0x01, &ies_b, 5180, -61 },
  { NC, 0x11, &ies_c, 2462, -70 }, { ND, 0x11, &ies_d, 5260, -74 } };
 for (unsigned i=0;i<5;i++) {
  lane_frames[i].n=frame_of(lane_frames[i].f,0x80,list[i].id,list[i].cap,list[i].l);
  lane_frames[i].mhz=list[i].mhz; lane_frames[i].rssi=list[i].rssi;
 }
 lane_frame_count=5;
}
static void radio_boot(void) {
 assert(!atomic_load(&kallocs));
 opt=3; fuxi_wifi_ios_init();
 assert(said_times("FUXI WIFI IOS: enabled, radio scan backend (scan only, no join); Apple80211 16.7 ABI unverified\n")==1);
 assert(attached==1 && registry==1 && fuxi_wifi_sta_present() && atomic_load(&events[1])>=1);
 assert(atomic_load(&kallocs)>=4);   /* the tables and the scratch, on this boot only */
 other_settle();
}
static void radio_ready_now(void) {
 fill_all(); fuxi_wifi_radio_starting(); fuxi_wifi_radio_ready(all38,38);
 wait_said("FUXI WIFI IOS: radio: radio producer ready, 38 channels (13 at 2.4 GHz, 25 at 5 GHz), passive only\n");
 other_settle();
}
static void scenario_radio(void) {
 int v; char name[256];
 fill_all(); lane_frames_make(); radio_boot();
 assert(other_armed()==30*NSEC_PER_SEC);                 /* the backstop, until the producer speaks */
 fuxi_wifi_radio_starting();                              /* as fuxi_wlan_init will, before launchd */
 lane_start(LANE_ANSWER);
 /* before READY: the scan is over at once, nothing heard, nothing asked of the radio */
 assert(!scan_ask(3,0,0)); settle();
 assert(atomic_load(&events[10])==1 && result_get(1024)==EIO && atomic_load(&lane_takes)==0);
 assert(said(SAY("FUXI WIFI IOS: radio: scan 1: the radio is starting, nothing asked of it, status %d\n",ENETDOWN)));
 assert(said(SAY("FUXI WIFI IOS: apple80211: the station says: a scan is over, %d\n",ENETDOWN)));
 /* the backstop does nothing once the producer has said STARTING */
 now_abs=31*NSEC_PER_SEC; assert(other_fire()); other_settle(); assert(!other_armed() && !said("Wi-Fi reported off"));
 /* the channels: 38, all passive, radar on 52-64 and 100-144; 11a/b/g */
 struct apple80211_channels *ch=calloc(1,sizeof(*ch));
 assert(!request(false,27,ch,sizeof(*ch),NULL) && ch->count==38);
 unsigned dfs=0;
 for (unsigned i=0;i<38;i++) {
  struct apple80211_channel c=ch->channels[i]; unsigned f=all38[i];
  bool radar=(f>=5260 && f<=5320) || (f>=5500 && f<=5720);
  assert(c.version==1 && !(c.flags & CHANNEL_ACTIVE));
  assert(c.flags==(f<4000 ? 0x0au : radar ? 0x112u : 0x12u)); dfs+=radar;
 }
 assert(dfs==16 && ch->channels[0].channel==1 && ch->channels[12].channel==13 && ch->channels[37].channel==165);
 free(ch);
 struct apple80211_channels_info *info=calloc(1,sizeof(*info));
 assert(!request(false,207,info,sizeof(*info),NULL) && info->count==38);
 dfs=0; for (unsigned i=0;i<38;i++) { assert(info->passive[i]==1); dfs+=info->dfs[i]; }
 assert(dfs==16); free(info);
 struct apple80211_phy_mode phy={0};
 assert(!request(false,14,&phy,sizeof(phy),NULL) && phy.supported==(PHY_MODE_AUTO|PHY_MODE_11A|PHY_MODE_11B|PHY_MODE_11G));
 /* READY with what the firmware may use: a 6 GHz channel, a repeat and a 0 are left out */
 uint16_t fw[41]; memcpy(fw,all38,sizeof(all38)); fw[38]=5955; fw[39]=2412; fw[40]=0;
 fuxi_wifi_radio_ready(fw,41);
 wait_said("FUXI WIFI IOS: radio: radio producer ready, 38 channels (13 at 2.4 GHz, 25 at 5 GHz), passive only\n");
 /* IO80211's fallback scan (type 3, {1, 0, 0}), asked as active: 38 channels, passive, 150 ms; five frames */
 atomic_store(&lane_mode,LANE_GATED);
 now_abs=40*NSEC_PER_SEC; assert(!scan_ask(1,0,0)); wait_count(&lane_fed,1);
 now_abs=42500*1000000ull; atomic_fetch_add(&lane_go,1); settle();
 pthread_mutex_lock(&lane_lock); struct fuxi_wifi_radio_request seen=lane_last; pthread_mutex_unlock(&lane_lock);
 assert(seen.seq && seen.freq_count==38 && seen.passive && seen.dwell_ms==150 && !seen.ssid_length);
 assert(!memcmp(seen.freqs,all38,sizeof(all38)));
 assert(atomic_load(&events[10])==2);
 assert(said("FUXI WIFI IOS: radio: scan 2: 38 channels passive, 4 heard (2 at 2.4 GHz, 2 at 5 GHz, 3 RSN, 1 hidden), 2500 ms, status 0\n"));
 /* GET 11, field by field (ios_scan_plan 1.1, the radio column) */
 const uint8_t *raw=(const uint8_t *)&res;
 assert(!result_get(1024));
 assert(res.version==1 && res.channel.version==1 && res.channel.channel==6 && res.channel.flags==0x0a);
 assert(raw[0x08]==6 && raw[0x0c]==0x0a && raw[0x10]==1 && raw[0x60]==8);
 assert(res.noise_known==1 && res.noise==-95 && res.snr==47 && res.rssi==-48 && res.beacon_interval==100 && res.capability==0x11);
 assert(!memcmp(res.bssid,NA,6) && !memcmp(raw+0x1c,NA,6) && res.rate_count==8);
 assert(res.rates[0]==1 && res.rates[1]==2 && res.rates[2]==5 && res.rates[3]==11 && res.rates[7]==18 && !res.unused_0);
 assert(res.ssid_length==8 && !memcmp(res.ssid,"zqA-wpa2",8) && res.age_ms==2500 && !res.flags);
 assert(res.elements_length==ies_a.n && !memcmp(res.elements,ies_a.b,ies_a.n));
 assert(said("FUXI WIFI IOS: apple80211: first SCAN_RESULT room 1024 bytes (wifid's 1024 in 1164 bytes)\n"));
 /* scan 3: the rooms. 100: name, rates, channel and RSN, the filler left out; 0 counts as 1024; a short buffer */
 atomic_store(&lane_mode,LANE_ANSWER);
 assert(!scan_ask(3,0,0)); settle(); assert(atomic_load(&events[10])==3);
 assert(!result_get(100) && res.elements_length==45 && has_element(res.elements,45,48) && !has_element(res.elements,45,221));
 bool rsn,w; assert(whole_in_order(ies_a.b,ies_a.n,res.elements,45,&rsn,&w)==4 && rsn);
 assert(!result_get(0) && !memcmp(res.bssid,NB,6) && res.elements_length==ies_b.n && res.channel.channel==36 && res.channel.flags==0x12);
 assert(res.capability==1 && res.snr==34 && res.rate_count==8 && res.rates[0]==6 && res.rates[7]==54 && !has_element(res.elements,res.elements_length,48));
 memset(&res,0xee,sizeof(res));
 { struct apple80211req req={ .req_if_name="en0",.req_type=11,.req_len=0x8c+50,.req_data=(uintptr_t)&res };
   assert(!fuxi_wifi_apple80211_ioctl(SIOCGA80211,&req)); }
 assert(!memcmp(res.bssid,NC,6) && res.ssid_length==0 && res.elements_length==ies_c.n && ies_c.n<=50);
 assert(!memcmp(res.elements,ies_c.b,ies_c.n) && raw[0x8c+50]==0xee && res.channel.channel==11);
 for (unsigned i=0;i<32;i++) assert(!res.ssid[i]);
 assert(!result_get(1024) && !memcmp(res.bssid,ND,6) && res.channel.channel==52 && res.channel.flags==0x112 && res.snr==21);
 assert(result_get(1024)==EIO);
 assert(said_times("first SCAN_RESULT room")==1);
 /* channel 36 alone: a request with 5180 only */
 assert(!scan_ask(1,36,CHANNEL_5GHZ)); settle();
 pthread_mutex_lock(&lane_lock); seen=lane_last; pthread_mutex_unlock(&lane_lock);
 assert(seen.freq_count==1 && seen.freqs[0]==5180 && seen.passive);
 assert(!result_get(1024) && !memcmp(res.bssid,NB,6) && result_get(1024)==EIO);
 /* channel 14 (2484, not listed): over at once, nothing asked, nothing heard */
 unsigned takes=atomic_load(&lane_takes);
 assert(!scan_ask(1,14,CHANNEL_2GHZ)); settle();
 assert(atomic_load(&lane_takes)==takes && result_get(1024)==EIO && atomic_load(&events[10])==5);
 assert(said("FUXI WIFI IOS: radio: scan 5: none of the channels asked for is the radio's, nothing heard, status 0\n"));
 /* joins: refused without the radio, result 2, event 9, no link, never "wrong password" */
 struct apple80211_assoc join={.version=1,.ssid_length=8}; memcpy(join.ssid,"zqB-open",8);
 assert(!request(true,20,&join,sizeof(join),NULL)); settle();
 assert(!request(false,21,NULL,0,&v) && v==2 && atomic_load(&events[9])==1 && !atomic_load(&events[4]) && !link_up);
 memcpy(join.ssid,"zqA-wpa2",8); join.key_cipher=CIPHER_PASSPHRASE; join.key_length=9; memcpy(join.key,"fuxitest1",9);
 assert(!request(true,20,&join,sizeof(join),NULL)); settle();
 assert(!request(false,21,NULL,0,&v) && v==2 && atomic_load(&events[9])==2 && !atomic_load(&events[4]) && !link_up);
 assert(said_times("FUXI WIFI IOS: radio: join refused (scan-only backend), no radio access\n")==2);
 assert(said_times(SAY("FUXI WIFI IOS: apple80211: the station says: a join is over, %d\n",EOPNOTSUPP))==2);
 assert(!request(false,13,NULL,0,&v) && v==0 && request(false,103,&res,sizeof(res),NULL)==ENXIO);
 assert(request(false,1,name,32,NULL)==ENXIO && request(false,9,name,6,NULL)==ENXIO);
 memset(name,1,sizeof(name));
 assert(!request(false,43,name,sizeof(name),NULL) && !strcmp(name,"Fuxi Wi-Fi iOS shim (radio backend, scan only)"));
 /* 3 scans in a row that the radio takes and never answers: each times out (9550 ms), then Wi-Fi is off */
 atomic_store(&lane_mode,LANE_SILENT);
 for (unsigned i=0;i<3;i++) {
  takes=atomic_load(&lane_takes); uint64_t start=(uint64_t)(60+20*i)*NSEC_PER_SEC; now_abs=start;
  assert(!scan_ask(3,0,0)); wait_count(&lane_takes,takes+1);
  now_abs=start+9549*1000000ull; struct timespec pause={.tv_nsec=5000000}; nanosleep(&pause,NULL);
  assert(fuxi_wifi_sta_busy());
  now_abs=start+9550*1000000ull; settle(); wait_count(&lane_cancels,i+1);
  assert(said(SAY("FUXI WIFI IOS: radio: scan %u: 38 channels passive, 0 heard (0 at 2.4 GHz, 0 at 5 GHz, 0 RSN, 0 hidden), 9550 ms, status %d\n",6+i,ETIMEDOUT)));
 }
 wait_said("FUXI WIFI IOS: radio: scans timed out 3 times in a row (code 0x60003); Wi-Fi reported off, no retry this boot\n");
 other_settle();
 assert(atomic_load(&events[1])==2 && !power_now() && !fuxi_wifi_sta_powered());
 assert(scan_ask(3,0,0)==ENETDOWN);
 assert(!power_set(true) && !power_now() && atomic_load(&events[1])==3);
 assert(said_times("FUXI WIFI IOS: apple80211: the station says: power, 0\n")==2);
 assert(said_times("Wi-Fi reported off")==1);
 lane_end(); other_settle(); clean_log();
 puts("radio: PASS");
}
static void scenario_stop(void) {
 lane_frames_make(); radio_boot(); radio_ready_now(); lane_start(LANE_HOLD);
 assert(!scan_ask(3,0,0)); wait_count(&lane_takes,1);
 assert(fuxi_wifi_sta_busy());
 struct timespec t0,t1; clock_gettime(CLOCK_MONOTONIC,&t0);
 unsigned mark=woken_mark();
 fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | 0x1c05);
 assert(woken_since(mark,&radio_done_seq));
 settle(); clock_gettime(CLOCK_MONOTONIC,&t1);
 assert((t1.tv_sec-t0.tv_sec)*1000000000L+(t1.tv_nsec-t0.tv_nsec)<100000000L);
 assert(said(SAY("FUXI WIFI IOS: apple80211: the station says: a scan is over, %d\n",ENETDOWN)));
 wait_said("FUXI WIFI IOS: radio: the radio lane stopped (code 0x41c05); Wi-Fi reported off, no retry this boot\n");
 other_settle(); wait_count(&lane_cancels,1);
 struct collected *k=calloc(1,sizeof(*k)); assert(fuxi_wifi_radio_results(collect,k)==0);
 assert(result_get(1024)==EIO && !power_now() && atomic_load(&events[1])==2);
 assert(!power_set(true) && !power_now() && atomic_load(&events[1])==3);
 /* terminal: later calls change nothing */
 fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | 1); fuxi_wifi_radio_starting(); fuxi_wifi_radio_ready(all38,38); other_settle();
 assert(fuxi_wifi_radio_state(NULL)==FUXI_WIFI_RADIO_STOPPED && said_times("Wi-Fi reported off")==1 && !said("0x40001"));
 struct fuxi_wifi_radio_request r; assert(!fuxi_wifi_radio_take(&r) && fuxi_wifi_radio_cancelled(1));
 assert(scan_ask(3,0,0)==ENETDOWN && !request(false,13,NULL,0,NULL));
 lane_end(); other_settle(); free(k); clean_log();
 puts("stop: PASS");
}
static void scenario_cancel(void) {
 int v;
 lane_frames_make(); radio_boot(); radio_ready_now(); lane_start(LANE_HOLD);
 assert(!scan_ask(3,0,0)); wait_count(&lane_takes,1);
 assert(!request(false,13,NULL,0,&v) && v==1);              /* scanning */
 struct { uint32_t version,radios,state[4]; } p={.version=1,.radios=1};
 assert(!request(true,19,&p,sizeof(p),NULL)); settle();
 wait_count(&lane_cancels,1);
 assert(said(SAY("FUXI WIFI IOS: apple80211: the station says: a scan is over, %d\n",ECANCELED)));
 assert(said(SAY("FUXI WIFI IOS: radio: scan 1: 38 channels passive, 0 heard (0 at 2.4 GHz, 0 at 5 GHz, 0 RSN, 0 hidden), 0 ms, status %d\n",ECANCELED)));
 assert(!power_now() && result_get(1024)==EIO);
 /* on again: the radio is READY, so it is on, and a scan is heard */
 assert(!power_set(true) && power_now()==1);
 atomic_store(&lane_mode,LANE_ANSWER);
 assert(!scan_ask(3,0,0)); settle();
 for (unsigned i=0;i<4;i++) assert(!result_get(1024));
 assert(result_get(1024)==EIO && !said("Wi-Fi reported off"));
 lane_end(); other_settle(); clean_log();
 puts("cancel: PASS");
}
static void scenario_early(void) {
 fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_NOT_REQUESTED);   /* fuxi_wlan_init without fuxiwlan, before this init */
 assert(!loglen && !atomic_load(&kallocs));
 radio_boot();
 wait_said("FUXI WIFI IOS: radio: the radio lane stopped (code 0x10000); Wi-Fi reported off, no retry this boot\n");
 other_settle();
 assert(atomic_load(&events[1])==2 && !power_now() && !other_armed());
 assert(scan_ask(3,0,0)==ENETDOWN);
 fill_all(); fuxi_wifi_radio_starting(); fuxi_wifi_radio_ready(all38,38); other_settle();
 assert(fuxi_wifi_radio_state(NULL)==FUXI_WIFI_RADIO_STOPPED && !said("producer ready") && said_times("Wi-Fi reported off")==1);
 assert(!power_set(true) && !power_now());
 clean_log();
 puts("early: PASS");
}
static void scenario_silent(bool by_state) {
 int v;
 radio_boot();
 assert(other_armed()==30*NSEC_PER_SEC && !said("Wi-Fi reported off"));
 if (by_state) {
  assert(!request(false,13,NULL,0,&v) && v==0);
 } else {
  assert(!scan_ask(3,0,0)); settle();
  assert(said(SAY("FUXI WIFI IOS: radio: scan 1: the radio is stopped, nothing asked of it, status %d\n",ENETDOWN)));
 }
 wait_said("FUXI WIFI IOS: radio: no radio lane reported before iOS first asked (code 0x50001); Wi-Fi reported off, no retry this boot\n");
 other_settle();
 assert(!power_now() && atomic_load(&events[1])==2 && scan_ask(3,0,0)==ENETDOWN);
 fill_all(); fuxi_wifi_radio_starting(); fuxi_wifi_radio_ready(all38,38); other_settle();
 assert(fuxi_wifi_radio_state(NULL)==FUXI_WIFI_RADIO_STOPPED && !power_set(true) && !power_now());
 clean_log();
 puts(by_state ? "silent-state: PASS" : "silent-scan: PASS");
}
static void scenario_backstop(void) {
 radio_boot();
 assert(other_armed()==30*NSEC_PER_SEC);
 now_abs=29999*1000000ull; assert(!other_fire());
 now_abs=30*NSEC_PER_SEC; assert(other_fire()); other_settle();
 wait_said("FUXI WIFI IOS: radio: no radio lane reported within 30 s of attach (code 0x50002); Wi-Fi reported off, no retry this boot\n");
 assert(!other_armed() && !power_now() && atomic_load(&events[1])==2);
 fuxi_wifi_radio_starting(); assert(fuxi_wifi_radio_state(NULL)==FUXI_WIFI_RADIO_STOPPED);
 assert(said_times("Wi-Fi reported off")==1);
 clean_log();
 puts("backstop: PASS");
}
/* READY between the backstop's look (ABSENT) and its re-arm, which on XNU replaces the entry READY made:
   told at once all the same, not when the 30 s are up */
static void ready_in_rearm(void) { fill_all(); fuxi_wifi_radio_starting(); fuxi_wifi_radio_ready(all38,38); }
static void scenario_rearm(void) {
 other_hook=ready_in_rearm;
 radio_boot();
 wait_said("FUXI WIFI IOS: radio: radio producer ready, 38 channels (13 at 2.4 GHz, 25 at 5 GHz), passive only\n");
 other_settle();
 assert(!other_hook && !now_abs && !other_armed() && power_now()==1 && fuxi_wifi_radio_state(NULL)==FUXI_WIFI_RADIO_READY);
 assert(said_times("producer ready")==1 && !said("Wi-Fi reported off"));
 clean_log();
 puts("rearm: PASS");
}
/* ready() naming no listed channel (channel numbers for MHz, 6 GHz): Wi-Fi off with the cause, not on and empty */
static void scenario_nochannel(void) {
 uint16_t wrong[4]={1,6,11,5955};
 radio_boot(); fuxi_wifi_radio_starting();
 fuxi_wifi_radio_ready(wrong,4);
 wait_said("FUXI WIFI IOS: radio: the radio lane named no usable channel (code 0x70004); Wi-Fi reported off, no retry this boot\n");
 other_settle();
 assert(fuxi_wifi_radio_state(NULL)==FUXI_WIFI_RADIO_STOPPED && !said("producer ready"));
 assert(!power_now() && atomic_load(&events[1])==2 && scan_ask(3,0,0)==ENETDOWN);
 fill_all(); fuxi_wifi_radio_ready(all38,38); other_settle();
 assert(fuxi_wifi_radio_state(NULL)==FUXI_WIFI_RADIO_STOPPED && !power_set(true) && !power_now());
 assert(said_times("Wi-Fi reported off")==1 && !said("producer ready"));
 clean_log();
 puts("nochannel: PASS");
}

/* ---- T7: fuxiwifiios=4, the canned producer ----------------------------------------------------------- */
static void canned_check(void) {
 static const uint8_t base[5]={0x02,0x46,0x55,0x58,0x4a};
 assert(!result_get(1024) && res.ssid_length==16 && !memcmp(res.ssid,"Fuxi Canned WPA2",16) && !memcmp(res.bssid,base,5));
 assert(res.bssid[5]==1 && res.channel.channel==6 && res.channel.flags==0x0a && res.rssi==-48 && res.noise==-95 && res.snr==47);
 assert(res.capability==0x11 && res.rate_count==12 && res.rates[11]==54 && has_element(res.elements,res.elements_length,48));
 assert(!result_get(1024) && !res.ssid_length && res.bssid[5]==3 && res.channel.channel==11 && res.rssi==-70);
 assert(has_element(res.elements,res.elements_length,48) && res.capability==0x11);
 for (unsigned i=0;i<32;i++) assert(!res.ssid[i]);
 assert(!result_get(1024) && res.ssid_length==16 && !memcmp(res.ssid,"Fuxi Canned Open",16) && res.bssid[5]==2);
 assert(res.channel.channel==36 && res.channel.flags==0x12 && res.rssi==-61 && res.capability==1 && res.rate_count==8);
 assert(!has_element(res.elements,res.elements_length,48));
 assert(!result_get(1024) && res.ssid_length==15 && !memcmp(res.ssid,"Fuxi Canned DFS",15) && res.bssid[5]==4);
 assert(res.channel.channel==52 && res.channel.flags==0x112 && res.rssi==-74 && has_element(res.elements,res.elements_length,48));
 assert(result_get(1024)==EIO);
}
static void scenario_canned(void) {
 int v; char name[256];
 fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_NOT_REQUESTED);   /* what the radio lane said is forgotten */
 opt=4; fuxi_wifi_ios_init();
 assert(said_times("FUXI WIFI IOS: enabled, canned scan backend (no radio); Apple80211 16.7 ABI unverified\n")==1);
 assert(attached==1);
 wait_said("FUXI WIFI IOS: radio: canned producer ready, 38 channels (13 at 2.4 GHz, 25 at 5 GHz), passive only\n");
 other_settle(); assert(!other_armed());
 /* the radio lane is ignored */
 fill_all(); uint16_t two[2]={2412,5180};
 fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | 7); fuxi_wifi_radio_starting(); fuxi_wifi_radio_ready(two,2);
 struct fuxi_wifi_radio_request r; assert(!fuxi_wifi_radio_take(&r) && fuxi_wifi_radio_cancelled(1));
 assert(fuxi_wifi_radio_state(NULL)==FUXI_WIFI_RADIO_READY && !said("Wi-Fi reported off") && power_now()==1);
 /* M1: a type-3 scan, four networks, four answers then EIO */
 assert(!scan_ask(3,0,0)); settle();
 assert(said("FUXI WIFI IOS: radio: scan 1: 38 channels passive, 4 heard (2 at 2.4 GHz, 2 at 5 GHz, 3 RSN, 1 hidden), 0 ms, status 0\n"));
 assert(said("FUXI WIFI IOS: apple80211: the station says: a scan is over, 0\n"));
 canned_check();
 assert(said_times("fixture gets 11 (SCAN_RESULT), value 0, 1164 bytes: answered\n")==4);
 assert(said_times("fixture gets 11 (SCAN_RESULT), value 0, 1164 bytes: error 5\n")==1);
 assert(atomic_load(&model_radio_waits[1])==38);              /* one dwell a channel */
 /* the same again */
 assert(!scan_ask(3,0,0)); settle(); canned_check();
 /* channel 6 alone: the WPA2 network is heard on 5, so nothing */
 assert(!scan_ask(1,6,CHANNEL_2GHZ)); settle(); assert(result_get(1024)==EIO);
 /* joins refused */
 struct apple80211_assoc join={.version=1,.ssid_length=16}; memcpy(join.ssid,"Fuxi Canned Open",16);
 assert(!request(true,20,&join,sizeof(join),NULL)); settle();
 assert(!request(false,21,NULL,0,&v) && v==2);
 memcpy(join.ssid,"Fuxi Canned WPA2",16); join.key_cipher=CIPHER_PASSPHRASE; join.key_length=9; memcpy(join.key,"fuxitest1",9);
 assert(!request(true,20,&join,sizeof(join),NULL)); settle();
 assert(!request(false,21,NULL,0,&v) && v==2 && atomic_load(&events[9])==2 && !atomic_load(&events[4]) && !link_up);
 assert(said_times("FUXI WIFI IOS: radio: join refused (scan-only backend), no radio access\n")==2);
 memset(name,1,sizeof(name));
 assert(!request(false,43,name,sizeof(name),NULL) && !strcmp(name,"Fuxi Wi-Fi iOS shim (canned scan backend)"));
 other_settle(); clean_log();
 puts("canned: PASS");
}

/* ---- T8: no fuxiwifiios ----------------------------------------------------------------------------- */
static void scenario_default(void) {
 static uint8_t f[200]; struct ies l; memset(&l,0,sizeof(l)); put(&l,0,"zqA",3);
 static const uint8_t A[6]={0x02,0x10,0x20,0x30,0x40,0x01};
 size_t n=frame_of(f,0x80,A,1,&l);
 struct fuxi_wifi_sta_bss *b=calloc(1,sizeof(*b)); struct fuxi_wifi_radio_request r;
 fill_all();
 fuxi_wifi_radio_starting(); fuxi_wifi_radio_ready(all38,38);
 assert(!fuxi_wifi_radio_take(&r) && !fuxi_wifi_radio_take(NULL) && fuxi_wifi_radio_cancelled(1) && fuxi_wifi_radio_cancelled(0));
 assert(fuxi_wifi_radio_frame(1,f,n,2412,-40,-95)==FUXI_WIFI_RADIO_DROP_STALE && fuxi_wifi_radio_bss(1,b)==FUXI_WIFI_RADIO_DROP_STALE);
 fuxi_wifi_radio_done(1,0); fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | 3); fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | 4);
 opt=0; fuxi_wifi_ios_init(); opt=5; fuxi_wifi_ios_init();
 assert(radio_state==FUXI_WIFI_RADIO_STOPPED && radio_code==(FUXI_WIFI_RADIO_EXITED | 3));
 assert(!atomic_load(&kallocs) && !loglen && !other_call.func && !attached && !registry);
 assert(fuxi_wifi_radio_results(collect,NULL)==0);
 free(b);
 puts("default: PASS");
}

int main(int argc,char **argv) {
 assert(argc==2); alarm(40);
 const char *s=argv[1];
 if (!strcmp(s,"parse")) scenario_parse();
 else if (!strcmp(s,"fuzz")) scenario_fuzz();
 else if (!strcmp(s,"table")) scenario_table();
 else if (!strcmp(s,"radio")) scenario_radio();
 else if (!strcmp(s,"stop")) scenario_stop();
 else if (!strcmp(s,"cancel")) scenario_cancel();
 else if (!strcmp(s,"early")) scenario_early();
 else if (!strcmp(s,"silent-scan")) scenario_silent(false);
 else if (!strcmp(s,"silent-state")) scenario_silent(true);
 else if (!strcmp(s,"backstop")) scenario_backstop();
 else if (!strcmp(s,"rearm")) scenario_rearm();
 else if (!strcmp(s,"nochannel")) scenario_nochannel();
 else if (!strcmp(s,"canned")) scenario_canned();
 else if (!strcmp(s,"default")) scenario_default();
 else return 2;
 return 0;
}
'''


def check(name, ok, detail=''):
    RESULTS.append((name, ok))
    print(('PASS ' if ok else 'FAIL ') + name, flush=True)
    if not ok and detail:
        print('    ' + str(detail).replace('\n', '\n    '), flush=True)
    return ok


def build(tmp, name, src=SRC, defs=(), sanitize=True):
    path = tmp / (name + '.c')
    path.write_text(W.unit_source(W.MODEL, src) + TEST)
    exe = tmp / name
    p = subprocess.run([CC, *FLAGS, *(SAN if sanitize else ()), *defs, str(path), '-o', str(exe)],
                       capture_output=True, text=True)
    return exe if p.returncode == 0 else p.stderr[-3000:]


def run(exe, scenario, timeout=60):
    try:
        p = subprocess.run([str(exe), scenario], capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, 'timeout'
    return p.returncode == 0 and f'{scenario}: PASS' in p.stdout, (p.stdout + p.stderr)[-2500:]


# ---- T6: static rules ------------------------------------------------------------------------------------
LOCKED_CALLS = {'memcpy', 'bzero', 'memcmp', 'assert_wait_timeout', 'lck_mtx_sleep'}
NOT_CALLS = {'if', 'for', 'while', 'switch', 'return', 'sizeof'}
PRODUCER = ('fuxi_wifi_radio_starting', 'fuxi_wifi_radio_ready', 'fuxi_wifi_radio_take', 'fuxi_wifi_radio_cancelled',
            'fuxi_wifi_radio_frame', 'fuxi_wifi_radio_bss', 'fuxi_wifi_radio_done', 'fuxi_wifi_radio_stopped')
PRODUCER_INSIDE = ('radio_starting', 'radio_ready', 'radio_cancelled', 'radio_frame', 'radio_bss', 'radio_done',
                   'radio_keep', 'radio_count', 'radio_reset', 'radio_summarize', 'fuxi_wifi_radio_parse',
                   'fuxi_wifi_elements_fit', 'element_rank', 'radio_listed', 'radio_dbm')
SLEEPS = {'lck_mtx_sleep', 'assert_wait', 'assert_wait_timeout', 'assert_wait_deadline', 'thread_block', 'delay',
          'IOSleep', 'kalloc_data', 'kfree_data', 'printf', 'lck_mtx_sleep_deadline', 'thread_call_enter_delayed'}


def uncomment(text):
    text = re.sub(r'/\*.*?\*/', lambda m: re.sub(r'[^\n]', ' ', m.group(0)), text, flags=re.S)
    return re.sub(r'"(?:\\.|[^"\\])*"', '""', text)


def calls(text):
    return [c for c in re.findall(r'\b([A-Za-z_]\w*)\s*\(', text) if c not in NOT_CALLS]


def bodies(code):
    """name -> body text of every function defined in the XNU style (name at the start of a line)."""
    out = {}
    for m in re.finditer(r'^([A-Za-z_]\w*)\(([^;{]*?)\)\s*\n\{', code, flags=re.M | re.S):
        depth, at = 0, m.end() - 1
        while True:
            depth += {'{': 1, '}': -1}.get(code[at], 0)
            at += 1
            if depth == 0:
                break
        out[m.group(1)] = (m.end(), code[m.end():at - 1])
    return out


def static_rules(src=SRC):
    problems = []
    code = uncomment((Path(src) / 'fuxi_wifi_radio.c').read_text())
    funcs = bodies(code)
    # the leaf lock: every lock has its unlock with no lock, return or goto between; only plain work inside
    spans = []
    locks = [m.end() for m in re.finditer(r'lck_mtx_lock\(&radio_lock\);', code)]
    if len(locks) != len(re.findall(r'lck_mtx_unlock\(&radio_lock\);', code)):
        problems.append('radio_lock: lock and unlock counts differ')
    for at in locks:
        end = code.find('lck_mtx_unlock(&radio_lock);', at)
        region = code[at:end] if end >= 0 else ''
        if end < 0 or 'lck_mtx_lock(' in region or re.search(r'\b(return|goto)\b', region):
            problems.append(f'radio_lock region at {at}: not closed before another lock, a return or a goto')
        spans.append((at, end))
        for c in calls(region):
            if c not in LOCKED_CALLS and not c.endswith('_locked'):
                problems.append(f'radio_lock region at {at}: calls {c}')
    for name, (at, body) in funcs.items():
        if name.endswith('_locked'):
            spans.append((at, at + len(body)))
            for c in calls(body):
                if c not in LOCKED_CALLS - {'assert_wait_timeout', 'lck_mtx_sleep'} and not c.endswith('_locked'):
                    problems.append(f'{name}: calls {c} (under radio_lock)')
            if 'lck_mtx_' in body:
                problems.append(f'{name}: takes or drops a lock')
    for m in re.finditer(r'\b(\w+_locked)\s*\(', code):
        line_start = code.rfind('\n', 0, m.start()) + 1
        if m.start() == line_start:
            continue                                    # its definition
        if not any(a <= m.start() < b for a, b in spans):
            problems.append(f'{m.group(1)} called without radio_lock held')
    # the producer never sleeps, allocates, logs or calls into the iOS side
    for name in PRODUCER + PRODUCER_INSIDE:
        if name not in funcs:
            problems.append(f'{name}: not found')
            continue
        for c in calls(funcs[name][1]):
            if c in SLEEPS or c.startswith('fuxi_wifi_sta_') or c.startswith('fuxi_wifi_apple80211_'):
                problems.append(f'producer {name}: calls {c}')
    if re.search(r'\bfuxi_wifi_(sta|apple80211|net)_\w+\s*\(', code):
        problems.append('fuxi_wifi_radio.c calls the station, Apple80211 or en0 directly')
    # the iOS side never names the radio lane or MMIO/DMA
    for name in ('fuxi_wifi_radio.c', 'fuxi_wifi_ios.c', 'fuxi_wifi_apple80211.c', 'fuxi_wifi_sta.c'):
        text = uncomment((Path(src) / name).read_text())
        bad = re.findall(r'\b(wl_\w+|wl\d+\w*|fw_\w+|ml_phys\w*|IOMemoryDescriptor|pmap_\w+)\b', text)
        if bad:
            problems.append(f'{name}: names {sorted(set(bad))}')
    # fuxi_wlan.c (patch 0099, not touched by K2): only the producer API, prototypes as in the header
    header = re.sub(r'\s+', ' ', uncomment((Path(src) / 'fuxi_wifi_radio.h').read_text()))
    if WLAN.exists():
        wlan = uncomment(WLAN.read_text())
        for m in re.finditer(r'\bfuxi_wifi_\w+', wlan):
            if m.group(0) not in PRODUCER and not m.group(0).startswith('FUXI_WIFI_RADIO_'):
                problems.append(f'fuxi_wlan.c names {m.group(0)}')
        for m in re.finditer(r'extern\s+([^;]*\bfuxi_wifi_radio_\w+\s*\([^;]*\));', wlan):
            proto = re.sub(r'\s+', ' ', m.group(1)).strip()
            if proto.replace(' (', '(') not in header.replace(' (', '('):
                problems.append(f'fuxi_wlan.c prototype differs from the header: {proto}')
    return problems


# ---- T9: mutants -----------------------------------------------------------------------------------------
MUTANTS = [
    ('no wakeup in stopped', 'fuxi_wifi_radio.c',
     '\tlck_mtx_unlock(&radio_lock);\n\tthread_wakeup((event_t)&radio_done_seq);\n\tif (call != NULL) {',
     '\tlck_mtx_unlock(&radio_lock);\n\tif (call != NULL) {'),
    ('RSN not prioritised', 'fuxi_wifi_radio.c', 'case 0: case 48:', 'case 0:'),
    ('< for <= in the element walk', 'fuxi_wifi_radio.c', 'at + 2 + in[at + 1] <= length', 'at + 2 + in[at + 1] < length'),
    ('no DS correction', 'fuxi_wifi_radio.c', 'frequency = ds == 14 ? 2484 : 2407 + 5 * (unsigned)ds;', '(void)ds;'),
    ('no generation bump on a swap', 'fuxi_wifi_radio.c', 'radio_front = table;\n\t\tradio_generation++;',
     'radio_front = table;'),
    ('no timeout', 'fuxi_wifi_radio.c', 'return late ? ETIMEDOUT : -1;', 'return -1;'),
    ('ACTIVE in the channel flags', 'fuxi_wifi_radio.c', '#define RADIO_P         FUXI_WIFI_STA_CHANNEL_PASSIVE',
     '#define RADIO_P         0'),
    ('EAUTH from join', 'fuxi_wifi_ios.c', '\treturn EOPNOTSUPP;\n}\n\nstatic int\nrscan_leave(void)',
     '\treturn EAUTH;\n}\n\nstatic int\nrscan_leave(void)'),
    ('SSID in the log', 'fuxi_wifi_radio.c', '\t\teach(radio_copy, context);',
     '\t\tprintf("FUXI WIFI IOS: radio: heard %.*s\\n", (int)radio_copy->ssid_length, (const char *)radio_copy->ssid);\n'
     '\t\teach(radio_copy, context);'),
    ('front not cleared on stop', 'fuxi_wifi_radio.c',
     '\tradio_live = 0;\n\tif (radio_front != NULL) {\n\t\tradio_front->count = 0;\n\t}\n\tradio_generation++;\n\treturn',
     '\tradio_live = 0;\n\tradio_generation++;\n\treturn'),
    ('lock not taken in take()', 'fuxi_wifi_radio.c',
     '\tlck_mtx_lock(&radio_lock);\n\ttable = radio_take_locked(out, true);\n\tlck_mtx_unlock(&radio_lock);',
     '\ttable = radio_take_locked(out, true);'),
    ('passive not forced', 'fuxi_wifi_radio.c', 'radio_request.passive = true;', 'radio_request.passive = what->passive;'),
    ('a name of zero bytes is not hidden', 'fuxi_wifi_radio.c', 'named |= ssid[i] != 0;', 'named = true;'),
    ('no giving up after 3 timeouts', 'fuxi_wifi_radio.c', '++radio_timeouts >= RADIO_TIMEOUTS_MOST',
     '++radio_timeouts >= 1000'),
    ('a room of 0 stays 0', 'fuxi_wifi_apple80211.c', '(asked != 0 || !whole_elements)', 'true'),
    ('no entry after the backstop re-arm', 'fuxi_wifi_radio.c', 'again = radio_state != FUXI_WIFI_RADIO_ABSENT;',
     'again = false;'),
    ('READY with no usable channel', 'fuxi_wifi_radio.c', 'if (n == 0) {', 'if (false) {'),
]


def mutant_suite(tmp):
    killed, survived = [], []
    for name, file, old, new in MUTANTS:
        text = (SRC / file).read_text()
        if text.count(old) != 1:
            survived.append(f'{name}: pattern found {text.count(old)} times')
            continue
        mdir = tmp / ('mut-' + re.sub(r'\W+', '-', name))
        shutil.copytree(SRC, mdir)
        (mdir / file).write_text(text.replace(old, new))
        if static_rules(mdir):
            killed.append(name)
            print(f'    mutant "{name}": caught by T6 static', flush=True)
            continue
        exe = build(tmp, 'm-' + re.sub(r'\W+', '-', name), mdir, sanitize=False)
        if isinstance(exe, str):
            survived.append(f'{name}: does not compile: {exe[-300:]}')
            continue
        by = next((s for s in SCENARIOS if not run(exe, s, timeout=45)[0]), None)
        (killed if by else survived).append(name)
        print(f'    mutant "{name}": ' + (f'caught by {by}' if by else 'SURVIVED'), flush=True)
    check(f'T9 mutants: {len(killed)} of {len(MUTANTS)} source mutations caught', not survived, survived)


def main():
    subprocess.run(['python3', str(ROOT / 'tools/sync-fuxi-wifi-ios.py'), '--check'], check=True)
    problems = static_rules()
    check('T6 static: radio_lock is a leaf lock, the producer never sleeps or calls the iOS side, no wl_/MMIO on the '
          'iOS side, fuxi_wlan.c names only the producer API', not problems, problems)
    with tempfile.TemporaryDirectory(prefix='fuxi-wifi-radio-') as t:
        tmp = Path(t)
        for label, defs in (('fuxi', ()), ('salami', ('-DFUXI_SALAMI=1',))):
            exe = build(tmp, 'radio-' + label, defs=defs)
            if not check(f'{label}: production units with the handoff compile warning-free (-Wall -Wextra -Werror, '
                         f'ASan/UBSan)', not isinstance(exe, str), exe if isinstance(exe, str) else ''):
                continue
            for s in SCENARIOS:
                ok, out = run(exe, s)
                check(f'{label} {s}', ok, out)
        mutant_suite(tmp)
    bad = [n for n, ok in RESULTS if not ok]
    print(f'wifi radio tests: {len(RESULTS) - len(bad)} PASS, {len(bad)} FAIL', flush=True)
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
