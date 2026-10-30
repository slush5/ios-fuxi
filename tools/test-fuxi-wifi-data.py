#!/usr/bin/env python3
"""Host tests: the radio data handoff (fuxi_wifi_data.c) and the fuxiwifiios=5 backend. No phone, no radio.

In WSL: CC=cc python3 -B tools/test-fuxi-wifi-data.py

Production C of xnu/fuxi-wifi-ios (#include lines stripped) on the model of tools/test-fuxi-wifi-ios.py
(imported, not copied), one translation unit, -Wall -Wextra -Werror, ASan/UBSan, pthreads; a Fuxi and a
salami build. Each scenario is a process of its own (the handoff is one state for the boot):
  D1 convert     802.11 -> 802.3 vectors (Data, QoS, HT Control, ARP, multicast, EAPOL, 802.1H tunnel, every
                 drop reason, 14 and 1514 byte edges), 802.3 -> 802.11 vectors, 2000 encapsulation round trips,
                 30000 fuzzed MPDUs
  D2 queue       before use(), no producer, bad joins, a join up; RX 32 then FULL, order, small room, 1000
                 frames through the ring, the counts wrapping past 2^32, EAPOL apart (8); TX 32 then FULL,
                 every send drop, no room at take, 32 in flight, done ok/failed/stale, 1000 frames; a leave
                 with frames everywhere (all flushed, the producer told), the counters
  D3 races       link_up for an untaken, wrong, zero or old request or another BSSID; EAPOL while joining,
                 data not; the producer refuses (its errno, or ECONNREFUSED); a join that times out (taken or
                 not) and a late link_up; give_up; a loss with frames queued and in flight; ready(false)
  D4 stress      a pthread producer flapping the link while frames pass both ways and the consumer joins and
                 leaves: frames in order, never across links, the counters add up
  D5 backend     fuxiwifiios=5 end to end with a pthread fake radio lane (scans and data): refused with no
                 data producer, scanned as =3, an open join posted and up (events 9 and 4, GET 13/1/9), RX to
                 en0 as Ethernet, EAPOL kept apart, TX to the AP as an MPDU, a loss, a rejoin to a named
                 BSSID, a leave, WPA2/protected/unheard refused, a producer refusal, a timeout (result 9), off
                 while joined, the radio lane stopping while joined; nothing a network carries in the log
  D6 noproducer  fuxiwifiios=5 with a radio lane that never reports: joins refused cleanly, then Wi-Fi off
  D7 default     fuxiwifiios absent, 0, 6, 7, -1: nothing made, said or allocated; the producer's calls inert
  D8 salami      fuxiwifiios absent/1/2/3/4: the same transcript with this tree as with the salami merge base's
                 sources (the answers in order, every log line byte for byte); every other 0104 source
                 byte-identical, fuxi_wifi_ios.c only the =5 hook
  D9 static      data_lock and rdata_lock are leaf locks, the producer never sleeps or calls the iOS side, the
                 handoff never calls the station, the backend never touches the handoff's state, no MAC or
                 SSID format in the log, no wl_/MMIO names
  D10 mutants    source mutations, each must fail a scenario or a static rule
"""
import difflib
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
CC = os.environ.get('CC', 'cc')
FLAGS = ['-std=gnu11', '-g', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function', '-Wno-unused-parameter',
         '-pthread']
SAN = ['-fsanitize=address,undefined', '-fno-sanitize-recover=all']
SCENARIOS = ('convert', 'queue', 'races', 'stress', 'backend', 'noproducer', 'default')
RESULTS = []


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


W = load('test_wifi_ios', ROOT / 'tools/test-fuxi-wifi-ios.py')
R = load('test_wifi_radio', ROOT / 'tools/test-fuxi-wifi-radio.py')

# en0's input, recorded (the wifi-ios model's stub takes only 0x88xx frames)
RX_STUB = 'static void fuxi_wifi_net_input(const uint8_t *p,size_t n) { assert(p[12]==0x88 && n>=14); received++; }'
RX_RECORD = ('static pthread_mutex_t rx_lock = PTHREAD_MUTEX_INITIALIZER; static uint8_t rx_got[64][1514]; static size_t rx_got_n[64];\n'
             'static void fuxi_wifi_net_input(const uint8_t *p,size_t n) { assert(n>=14 && n<=1514); pthread_mutex_lock(&rx_lock);\n'
             ' if (received<64) { memcpy(rx_got[received],p,n); rx_got_n[received]=n; } received++; pthread_mutex_unlock(&rx_lock); }')
# the radio test's helpers: requests, the log, scans, beacons
HELPERS = R.TEST[R.TEST.index('static void settle(void) {'):R.TEST.index('/* ---- T1/T2')]

TEST = r'''
extern unsigned int alarm(unsigned int seconds);   /* (not <unistd.h>: its link() would meet fuxi_wifi_ios.c's) */
bool fuxi_wifi_net_attach(const struct fuxi_wifi_net_link *p) {
 assert(p->wireless && !p->up && p->address[0]==2 && !strcmp(p->name,"radio-data")); attached++; return true;
}
''' + HELPERS + r'''
/* ---- frames ------------------------------------------------------------------------------------------- */
static const uint8_t SELF[6]={0x02,0x46,0x55,0x58,0x49,0x01};          /* the radio backends' address */
static const uint8_t AP[6]={0x02,0x10,0x20,0x30,0x40,0x0a}, AP2[6]={0x02,0x10,0x20,0x30,0x40,0x0b};
static const uint8_t PEER[6]={0x02,0x10,0x20,0x30,0x40,0x77};          /* a host behind the AP */
static const uint8_t BCAST[6]={0xff,0xff,0xff,0xff,0xff,0xff}, MCAST[6]={0x01,0x00,0x5e,0x00,0x00,0xfb};
static const uint8_t SNAP[6]={0xaa,0xaa,0x03,0,0,0}, TUNNEL[6]={0xaa,0xaa,0x03,0,0,0xf8};
#define KEPT FUXI_WIFI_DATA_KEPT
/* an 802.11 header: QoS Control after a QoS Data one, and HT Control after that with the Order bit */
static size_t hdr(uint8_t *f,uint8_t fc0,uint8_t fc1,const uint8_t *a1,const uint8_t *a2,const uint8_t *a3) {
 size_t h=24; memset(f,0,30); f[0]=fc0; f[1]=fc1; memcpy(f+4,a1,6); memcpy(f+10,a2,6); memcpy(f+16,a3,6);
 if (fc0==0x88) { f[24]=0x05; h+=2; if (fc1&0x80) h+=4; }
 return h;
}
static size_t mk(uint8_t *f,uint8_t fc0,uint8_t fc1,const uint8_t *a1,const uint8_t *a2,const uint8_t *a3,const uint8_t *snap,
 uint16_t type,size_t payload,uint8_t seed) {
 size_t h=hdr(f,fc0,fc1,a1,a2,a3); memcpy(f+h,snap,6); f[h+6]=(uint8_t)(type>>8); f[h+7]=(uint8_t)type;
 for (size_t i=0;i<payload;i++) f[h+8+i]=(uint8_t)(seed+i*7);
 return h+8+payload;
}
static size_t eth(uint8_t *e,const uint8_t *da,const uint8_t *sa,uint16_t type,size_t payload,uint8_t seed) {
 memcpy(e,da,6); memcpy(e+6,sa,6); e[12]=(uint8_t)(type>>8); e[13]=(uint8_t)type;
 for (size_t i=0;i<payload;i++) e[14+i]=(uint8_t)(seed+i*7);
 return 14+payload;
}
static uint32_t rng2=0x5d17;
static uint32_t rnd2(void) { rng2=rng2*1103515245u+12345u; return rng2>>8; }
static void counts(struct fuxi_wifi_data_counts *c) { fuxi_wifi_data_counts(c); }
static unsigned sum(const uint32_t *v) { unsigned s=0; for (unsigned i=0;i<FUXI_WIFI_DATA_DROP_REASONS;i++) s+=v[i]; return s; }
static struct fuxi_wifi_data_link now_link(void) { struct fuxi_wifi_data_link l; fuxi_wifi_data_link(&l); return l; }

/* ---- D1: conversion ------------------------------------------------------------------------------------ */
static void scenario_convert(void) {
 static uint8_t f[2400],o[1600],e[1600],m[1600],w[1600]; size_t n,len,el; bool ea;
 #define TO(l) fuxi_wifi_data_to_8023(f,(l),AP,SELF,o,&n,&ea)
 /* IPv4 from a host behind the AP: DA, SA (Address 3), type, payload */
 len=mk(f,0x08,0x02,SELF,AP,PEER,SNAP,0x0800,100,1); el=eth(e,SELF,PEER,0x0800,100,1);
 assert(len==132 && TO(len)==KEPT && n==el && !memcmp(o,e,el) && !ea);
 /* a sequence number is no fragment; Retry, Power Management and More Data change nothing */
 f[22]=0x50; f[23]=0x12; f[1]=0x02|0x08|0x10|0x20; assert(TO(len)==KEPT && n==el && !memcmp(o,e,el));
 /* QoS Data: 2 more; with the Order bit, HT Control: 4 more; the Order bit of plain Data adds nothing */
 len=mk(f,0x88,0x02,SELF,AP,PEER,SNAP,0x0800,100,1); assert(len==134 && TO(len)==KEPT && n==el && !memcmp(o,e,el));
 len=mk(f,0x88,0x82,SELF,AP,PEER,SNAP,0x0800,100,1); assert(len==138 && TO(len)==KEPT && n==el && !memcmp(o,e,el));
 len=mk(f,0x08,0x82,SELF,AP,PEER,SNAP,0x0800,100,1); assert(len==132 && TO(len)==KEPT && n==el && !memcmp(o,e,el));
 /* ARP to everyone, IPv6 to a group, EAPOL to us (flagged: the supplicant's) */
 len=mk(f,0x08,0x02,BCAST,AP,PEER,SNAP,0x0806,28,2); el=eth(e,BCAST,PEER,0x0806,28,2);
 assert(TO(len)==KEPT && n==el && !memcmp(o,e,el) && !ea);
 len=mk(f,0x88,0x02,MCAST,AP,PEER,SNAP,0x86dd,70,3); el=eth(e,MCAST,PEER,0x86dd,70,3);
 assert(TO(len)==KEPT && n==el && !memcmp(o,e,el) && !ea);
 len=mk(f,0x88,0x02,SELF,AP,AP,SNAP,0x888e,95,4); el=eth(e,SELF,AP,0x888e,95,4);
 assert(TO(len)==KEPT && n==el && !memcmp(o,e,el) && ea);
 /* 802.1H: the bridge tunnel for AppleTalk ARP and IPX, and whatever type it carries; RFC 1042 with those two is not ours */
 len=mk(f,0x08,0x02,SELF,AP,PEER,TUNNEL,0x80f3,40,5); el=eth(e,SELF,PEER,0x80f3,40,5); assert(TO(len)==KEPT && n==el && !memcmp(o,e,el));
 len=mk(f,0x08,0x02,SELF,AP,PEER,TUNNEL,0x8137,40,5); el=eth(e,SELF,PEER,0x8137,40,5); assert(TO(len)==KEPT && n==el && !memcmp(o,e,el));
 len=mk(f,0x08,0x02,SELF,AP,PEER,TUNNEL,0x0800,40,5); el=eth(e,SELF,PEER,0x0800,40,5); assert(TO(len)==KEPT && n==el && !memcmp(o,e,el));
 len=mk(f,0x08,0x02,SELF,AP,PEER,SNAP,0x80f3,40,5); assert(TO(len)==FUXI_WIFI_DATA_DROP_LLC && !n);
 len=mk(f,0x08,0x02,SELF,AP,PEER,SNAP,0x8137,40,5); assert(TO(len)==FUXI_WIFI_DATA_DROP_LLC);
 len=mk(f,0x08,0x02,SELF,AP,PEER,SNAP,0x05dc,40,5); assert(TO(len)==FUXI_WIFI_DATA_DROP_LLC);
 len=mk(f,0x08,0x02,SELF,AP,PEER,TUNNEL,0x0100,40,5); assert(TO(len)==FUXI_WIFI_DATA_DROP_LLC);
 static const uint8_t oui[6]={0xaa,0xaa,0x03,0x00,0x00,0x01}, ipx[6]={0xe0,0xe0,0x03,0,0,0}, ctl[6]={0xaa,0xaa,0x13,0,0,0};
 len=mk(f,0x08,0x02,SELF,AP,PEER,oui,0x0800,40,5); assert(TO(len)==FUXI_WIFI_DATA_DROP_LLC);
 len=mk(f,0x08,0x02,SELF,AP,PEER,ipx,0x0800,40,5); assert(TO(len)==FUXI_WIFI_DATA_DROP_LLC);
 len=mk(f,0x08,0x02,SELF,AP,PEER,ctl,0x0800,40,5); assert(TO(len)==FUXI_WIFI_DATA_DROP_LLC);
 /* lengths: the header and SNAP alone make a 14-byte frame; a byte less is short */
 assert(TO(0)==FUXI_WIFI_DATA_DROP_LENGTH && TO(23)==FUXI_WIFI_DATA_DROP_LENGTH);
 len=mk(f,0x08,0x02,SELF,AP,PEER,SNAP,0x0800,0,6); el=eth(e,SELF,PEER,0x0800,0,6);
 assert(len==32 && TO(32)==KEPT && n==14 && !memcmp(o,e,14) && TO(31)==FUXI_WIFI_DATA_DROP_LENGTH && TO(24)==FUXI_WIFI_DATA_DROP_LENGTH);
 len=mk(f,0x88,0x02,SELF,AP,PEER,SNAP,0x0800,0,6);
 assert(len==34 && TO(34)==KEPT && n==14 && TO(33)==FUXI_WIFI_DATA_DROP_LENGTH && TO(25)==FUXI_WIFI_DATA_DROP_LENGTH);
 len=mk(f,0x88,0x82,SELF,AP,PEER,SNAP,0x0800,0,6);
 assert(len==38 && TO(38)==KEPT && n==14 && TO(37)==FUXI_WIFI_DATA_DROP_LENGTH);
 /* 1500 bytes of payload: 1514 and not a byte written past it; 1501: over, nothing written */
 memset(o,0xee,sizeof(o));
 len=mk(f,0x88,0x82,SELF,AP,PEER,SNAP,0x0800,1500,7); el=eth(e,SELF,PEER,0x0800,1500,7);
 assert(TO(len)==KEPT && n==1514 && !memcmp(o,e,1514) && o[1514]==0xee);
 memset(o,0xee,sizeof(o)); len=mk(f,0x08,0x02,SELF,AP,PEER,SNAP,0x0800,1501,7);
 assert(TO(len)==FUXI_WIFI_DATA_DROP_LENGTH && !n && o[0]==0xee);
 len=mk(f,0x08,0x02,SELF,AP,PEER,SNAP,0x0800,2296,7); assert(TO(len)==FUXI_WIFI_DATA_DROP_LENGTH);
 /* kinds: beacon, probe request, ACK, Null, QoS Null, CF subtypes, version 1, 2, 3, a management type */
 len=mk(f,0x08,0x02,SELF,AP,PEER,SNAP,0x0800,10,8);
 static const uint8_t kinds[]={0x80,0x40,0xd4,0x48,0xc8,0x18,0x28,0x38,0x98,0xa8,0xb8,0x09,0x0a,0x0b,0x00,0x0c};
 for (unsigned i=0;i<sizeof(kinds);i++) { f[0]=kinds[i]; assert(TO(len)==FUXI_WIFI_DATA_DROP_KIND); }
 f[0]=0x08; assert(TO(len)==KEPT);
 /* still protected (not decrypted); fragments */
 f[1]=0x42; assert(TO(len)==FUXI_WIFI_DATA_DROP_PROTECTED);
 f[1]=0x06; assert(TO(len)==FUXI_WIFI_DATA_DROP_FRAGMENT);
 f[1]=0x02; f[22]=0x01; assert(TO(len)==FUXI_WIFI_DATA_DROP_FRAGMENT);
 f[22]=0x0f; assert(TO(len)==FUXI_WIFI_DATA_DROP_FRAGMENT);
 f[22]=0xf0; f[23]=0xff; assert(TO(len)==KEPT);
 /* DS bits: IBSS (0), to an AP (1), four addresses (3) */
 f[1]=0x00; assert(TO(len)==FUXI_WIFI_DATA_DROP_DS);
 f[1]=0x01; assert(TO(len)==FUXI_WIFI_DATA_DROP_DS);
 f[1]=0x03; assert(TO(len)==FUXI_WIFI_DATA_DROP_DS);
 f[1]=0x02;
 /* the addresses: another BSS, another station, our own frame reflected, a group source */
 memcpy(f+10,AP2,6); assert(TO(len)==FUXI_WIFI_DATA_DROP_BSSID); memcpy(f+10,AP,6);
 memcpy(f+4,PEER,6); assert(TO(len)==FUXI_WIFI_DATA_DROP_DEST); memcpy(f+4,SELF,6);
 memcpy(f+16,SELF,6); assert(TO(len)==FUXI_WIFI_DATA_DROP_SOURCE);
 memcpy(f+4,BCAST,6); assert(TO(len)==FUXI_WIFI_DATA_DROP_SOURCE); memcpy(f+4,SELF,6);
 memcpy(f+16,MCAST,6); assert(TO(len)==FUXI_WIFI_DATA_DROP_SOURCE); memcpy(f+16,PEER,6);
 assert(TO(len)==KEPT);
 /* an A-MSDU */
 len=mk(f,0x88,0x02,SELF,AP,PEER,SNAP,0x0800,10,8); f[24]|=0x80; assert(TO(len)==FUXI_WIFI_DATA_DROP_AMSDU);
 f[24]&=0x7f; assert(TO(len)==KEPT);
 /* nothing to read or nowhere to write */
 assert(fuxi_wifi_data_to_8023(NULL,len,AP,SELF,o,&n,&ea)==FUXI_WIFI_DATA_DROP_LENGTH);
 assert(fuxi_wifi_data_to_8023(f,len,NULL,SELF,o,&n,&ea)==FUXI_WIFI_DATA_DROP_LENGTH);
 assert(fuxi_wifi_data_to_8023(f,len,AP,NULL,o,&n,&ea)==FUXI_WIFI_DATA_DROP_LENGTH);
 assert(fuxi_wifi_data_to_8023(f,len,AP,SELF,NULL,&n,&ea)==FUXI_WIFI_DATA_DROP_LENGTH);
 assert(fuxi_wifi_data_to_8023(f,len,AP,SELF,o,NULL,&ea)==FUXI_WIFI_DATA_DROP_LENGTH);
 assert(fuxi_wifi_data_to_8023(f,len,AP,SELF,o,&n,NULL)==FUXI_WIFI_DATA_DROP_LENGTH);

 /* 802.3 -> 802.11: Data, To DS; Address 1 the BSSID, 2 us, 3 the destination; RFC 1042, the type, the payload */
 #define FROM(l,room) fuxi_wifi_data_from_8023(e,(l),AP,SELF,m,(room),&n)
 el=eth(e,PEER,SELF,0x0800,100,9);
 assert(FROM(el,sizeof(m))==KEPT && n==132);
 memset(w,0,24); w[0]=0x08; w[1]=0x01; memcpy(w+4,AP,6); memcpy(w+10,SELF,6); memcpy(w+16,PEER,6);
 memcpy(w+24,SNAP,6); w[30]=0x08; w[31]=0x00; memcpy(w+32,e+14,100); assert(!memcmp(m,w,132));
 el=eth(e,BCAST,SELF,0x80f3,30,9); assert(FROM(el,sizeof(m))==KEPT && n==62 && !memcmp(m+24,TUNNEL,6) && m[30]==0x80 && m[31]==0xf3);
 el=eth(e,BCAST,SELF,0x8137,30,9); assert(FROM(el,sizeof(m))==KEPT && !memcmp(m+24,TUNNEL,6) && !memcmp(m+16,BCAST,6));
 el=eth(e,AP,SELF,0x888e,99,9); assert(FROM(el,sizeof(m))==KEPT && !memcmp(m+24,SNAP,6) && m[31]==0x8e);   /* (send() refuses it from en0) */
 el=eth(e,PEER,SELF,0x0600,0,9); assert(FROM(el,sizeof(m))==KEPT && n==32);
 el=eth(e,PEER,SELF,0x05dc,40,9); assert(FROM(el,sizeof(m))==FUXI_WIFI_DATA_DROP_LLC && !n);
 el=eth(e,PEER,PEER,0x0800,40,9); assert(FROM(el,sizeof(m))==FUXI_WIFI_DATA_DROP_SOURCE);
 el=eth(e,PEER,SELF,0x0800,1500,9);
 assert(FROM(el,1532)==KEPT && n==1532 && FROM(el,1531)==FUXI_WIFI_DATA_DROP_LENGTH && !n);
 assert(FROM(13,sizeof(m))==FUXI_WIFI_DATA_DROP_LENGTH && FROM(1515,sizeof(m))==FUXI_WIFI_DATA_DROP_LENGTH);
 assert(fuxi_wifi_data_from_8023(e,el,AP,SELF,NULL,0,&n)==KEPT && n==1532);
 assert(fuxi_wifi_data_from_8023(NULL,el,AP,SELF,m,1600,&n)==FUXI_WIFI_DATA_DROP_LENGTH);
 assert(fuxi_wifi_data_from_8023(e,el,NULL,SELF,m,1600,&n)==FUXI_WIFI_DATA_DROP_LENGTH);
 assert(fuxi_wifi_data_from_8023(e,el,AP,NULL,m,1600,&n)==FUXI_WIFI_DATA_DROP_LENGTH);
 assert(fuxi_wifi_data_from_8023(e,el,AP,SELF,m,1600,NULL)==FUXI_WIFI_DATA_DROP_LENGTH);

 /* the encapsulation both ways: what from_8023 puts after its header, to_8023 reads back after an AP's */
 static const uint16_t types[]={0x0800,0x0806,0x86dd,0x888e,0x80f3,0x8137,0x8100,0x0600,0xffff};
 static const uint8_t fcs[3][2]={{0x08,0x02},{0x88,0x02},{0x88,0x82}};
 for (unsigned i=0;i<2000;i++) {
  uint16_t type=i%3 ? types[rnd2()%9] : (uint16_t)(0x0600+rnd2()%0xfa00);
  size_t payload=i<10 ? i : i<20 ? 1490+i-10 : rnd2()%1501;
  const uint8_t *da=i%5==0 ? BCAST : i%5==1 ? MCAST : PEER;
  el=eth(e,da,SELF,type,payload,(uint8_t)i);
  assert(FROM(el,sizeof(m))==KEPT && n==el+18 && !memcmp(m+16,da,6));
  unsigned k=rnd2()%3; size_t h=hdr(f,fcs[k][0],fcs[k][1],SELF,AP,PEER);
  memcpy(f+h,m+24,n-24); size_t got;
  assert(fuxi_wifi_data_to_8023(f,h+n-24,AP,SELF,o,&got,&ea)==KEPT && got==el);
  assert(!memcmp(o,SELF,6) && !memcmp(o+6,PEER,6) && !memcmp(o+12,e+12,el-12) && ea==(type==0x888e));
 }

 /* 30000 mutations of a QoS Data frame: never out of bounds, and what is kept is sane */
 static uint8_t base[400]; size_t base_n=mk(base,0x88,0x82,SELF,AP,PEER,SNAP,0x0800,200,10); unsigned kept=0;
 for (unsigned i=0;i<30000;i++) {
  size_t l=base_n; memcpy(f,base,l);
  for (unsigned k=1+rnd2()%4;k;k--) {
   switch (rnd2()%7) {
   case 0: if (l) f[rnd2()%l]^=(uint8_t)(1u<<(rnd2()%8)); break;
   case 1: f[rnd2()%2]=(uint8_t)rnd2(); break;
   case 2: l=rnd2()%(l+1); break;
   case 3: for (unsigned j=1+rnd2()%40;j && l<sizeof(f);j--) f[l++]=(uint8_t)rnd2(); break;
   case 4: f[22]=(uint8_t)(rnd2()%4==0 ? rnd2() : 0); break;
   case 5: f[30+rnd2()%8]=(uint8_t)rnd2(); break;
   default: memcpy(f+4+6*(rnd2()%3),rnd2()%2 ? SELF : rnd2()%2 ? BCAST : PEER,6); break;
   }
  }
  int why=TO(l);
  assert(why>=0 && why<FUXI_WIFI_DATA_DROP_REASONS && why!=FUXI_WIFI_DATA_DROP_DOWN && why!=FUXI_WIFI_DATA_DROP_FULL &&
   why!=FUXI_WIFI_DATA_DROP_EAPOL);
  if (why==KEPT) {
   kept++;
   assert(n>=14 && n<=1514 && (f[1]&0x47)==0x02 && (f[0]==0x08 || f[0]==0x88) && !memcmp(f+10,AP,6));
   assert(!memcmp(o,f+4,6) && !memcmp(o+6,f+16,6) && !(o[6]&1) && memcmp(o+6,SELF,6) && ((o[0]&1) || !memcmp(o,SELF,6)));
   assert(((unsigned)o[12]<<8|o[13])>=0x600 && ea==(o[12]==0x88 && o[13]==0x8e));
  } else {
   assert(!n && !ea);
  }
  size_t room=rnd2()%1600, ll=rnd2()%1600;
  if (fuxi_wifi_data_from_8023(f,ll,AP,SELF,m,room,&n)==KEPT) assert(n<=room && n==ll+18); else assert(!n);
 }
 assert(kept>3000);
 printf("convert: PASS (2000 round trips, 30000 mutations, %u kept)\n",kept);
}

/* ---- the consumer's joins, on a thread of their own (the station thread's part) ------------------------ */
static struct fuxi_wifi_data_request jreq; static struct fuxi_wifi_data_link jlink;
static atomic_int jstatus; static atomic_bool jdone, jstop; static unsigned jtimeout=5000;
static bool jgive(void) { return atomic_load(&jstop); }
static void *join_run(void *a) { (void)a; atomic_store(&jstatus,fuxi_wifi_data_join(&jreq,jtimeout,jgive,&jlink)); atomic_store(&jdone,true); return NULL; }
static pthread_t join_start(const uint8_t *bssid,uint16_t mhz) {
 struct timespec pause={.tv_nsec=100000}; pthread_t t; uint32_t before=now_link().seq;
 memset(&jreq,0,sizeof(jreq)); memcpy(jreq.ssid,"zqNet",5); jreq.ssid_length=5; memcpy(jreq.bssid,bssid,6); jreq.mhz=mhz;
 atomic_store(&jdone,false); atomic_store(&jstop,false); assert(!pthread_create(&t,NULL,join_run,NULL));
 for (unsigned i=0;i<50000 && !atomic_load(&jdone) && now_link().seq==before;i++) nanosleep(&pause,NULL);
 return t;
}
static int join_end(pthread_t t) { pthread_join(t,NULL); return atomic_load(&jstatus); }
static void take_req(struct fuxi_wifi_data_request *r) {
 struct timespec pause={.tv_nsec=100000};
 for (unsigned i=0;i<50000;i++) { if (fuxi_wifi_data_take(r)) return; nanosleep(&pause,NULL); }
 abort();
}
static bool still_waiting(void) { struct timespec pause={.tv_nsec=3000000}; nanosleep(&pause,NULL); return !atomic_load(&jdone); }
static uint32_t up_at(const uint8_t *bssid,uint16_t mhz) {
 struct fuxi_wifi_data_request r; pthread_t t=join_start(bssid,mhz); take_req(&r);
 assert(!r.leave && !memcmp(r.bssid,bssid,6) && r.mhz==mhz);
 fuxi_wifi_data_link_up(r.seq,bssid,mhz);
 assert(!join_end(t) && jlink.state==FUXI_WIFI_DATA_UP && jlink.seq==r.seq && jlink.mhz==mhz && !memcmp(jlink.bssid,bssid,6));
 return r.seq;
}
static uint8_t qf[1600], qo[1600], qe[1600];
static int rx_one(unsigned i,int16_t rssi) { size_t len=mk(qf,0x88,0x02,SELF,AP,PEER,SNAP,0x0800,60,(uint8_t)i); return fuxi_wifi_data_rx(qf,len,rssi); }
static void pull_check(unsigned i) {
 size_t n,el=eth(qe,SELF,PEER,0x0800,60,(uint8_t)i);
 assert(fuxi_wifi_data_pull(qo,sizeof(qo),&n) && n==el && !memcmp(qo,qe,el));
}
static int send_one(unsigned i) { size_t el=eth(qe,PEER,SELF,0x0800,100,(uint8_t)i); return fuxi_wifi_data_send(qe,el); }
static uint32_t take_check(unsigned i) {
 static uint8_t m[1600],w[1600]; size_t n,wn; uint32_t id=0;
 size_t el=eth(qe,PEER,SELF,0x0800,100,(uint8_t)i);
 assert(fuxi_wifi_data_from_8023(qe,el,AP,SELF,w,sizeof(w),&wn)==KEPT);
 n=fuxi_wifi_data_tx_take(m,sizeof(m),&id);
 assert(n==wn && id && !memcmp(m,w,n));
 return id;
}

/* ---- D2: the queues ------------------------------------------------------------------------------------ */
static void scenario_queue(void) {
 static uint8_t o[1600],e[1600]; size_t n,el; uint32_t id; struct fuxi_wifi_data_counts c; struct fuxi_wifi_data_request r;
 /* before use(): everything dropped, nothing stored but the ready word */
 assert(rx_one(0,-50)==FUXI_WIFI_DATA_DROP_DOWN && !fuxi_wifi_data_take(&r) && send_one(0)==FUXI_WIFI_DATA_DROP_DOWN);
 memset(&jreq,0,sizeof(jreq)); memcpy(jreq.bssid,AP,6); assert(fuxi_wifi_data_join(&jreq,100,NULL,NULL)==EOPNOTSUPP);
 counts(&c); assert(!c.rx_frames && !c.tx_frames && !c.joins && !atomic_load(&kallocs));
 assert(fuxi_wifi_data_use(SELF) && !fuxi_wifi_data_use(SELF) && atomic_load(&kallocs)==1);
 /* no producer yet: refused at once, nothing posted */
 assert(!fuxi_wifi_data_present() && fuxi_wifi_data_join(&jreq,100,NULL,NULL)==EOPNOTSUPP && !fuxi_wifi_data_take(&r));
 fuxi_wifi_data_ready(true); assert(fuxi_wifi_data_present());
 /* bad requests */
 assert(fuxi_wifi_data_join(NULL,100,NULL,NULL)==EINVAL);
 memset(jreq.bssid,0,6); assert(fuxi_wifi_data_join(&jreq,100,NULL,NULL)==EINVAL);
 memcpy(jreq.bssid,MCAST,6); assert(fuxi_wifi_data_join(&jreq,100,NULL,NULL)==EINVAL); memcpy(jreq.bssid,AP,6);
 jreq.ssid_length=33; assert(fuxi_wifi_data_join(&jreq,100,NULL,NULL)==EINVAL); jreq.ssid_length=0;
 jreq.leave=true; assert(fuxi_wifi_data_join(&jreq,100,NULL,NULL)==EINVAL);
 assert(!fuxi_wifi_data_take(&r) && !fuxi_wifi_data_take(NULL));
 /* a join: posted, taken once, waited for until link_up */
 pthread_t t=join_start(AP,2437); take_req(&r);
 assert(r.seq && !r.leave && r.ssid_length==5 && !memcmp(r.ssid,"zqNet",5) && !memcmp(r.bssid,AP,6) && r.mhz==2437 && !r.security);
 assert(still_waiting() && !fuxi_wifi_data_take(&r) && now_link().state==FUXI_WIFI_DATA_JOINING);
 fuxi_wifi_data_link_up(r.seq,AP,2442);
 assert(!join_end(t) && jlink.state==FUXI_WIFI_DATA_UP && jlink.seq==r.seq && jlink.mhz==2442 && !jlink.status);
 uint32_t epoch=jlink.epoch;
 /* RX: 32 wait, then FULL; in order; a room under 1514 takes nothing */
 for (unsigned i=0;i<40;i++) assert(rx_one(i,(int16_t)(-40-(int)i))==(i<32 ? KEPT : FUXI_WIFI_DATA_DROP_FULL));
 assert(now_link().rssi==-71);
 assert(!fuxi_wifi_data_pull(o,1513,&n) && !n && !fuxi_wifi_data_pull(NULL,1600,&n) && !fuxi_wifi_data_pull(o,1600,NULL));
 for (unsigned i=0;i<32;i++) pull_check(i);
 assert(!fuxi_wifi_data_pull(o,sizeof(o),&n) && !n);
 assert(rx_one(0,20)==KEPT && now_link().rssi==0 && rx_one(1,-300)==KEPT && now_link().rssi==-127);
 pull_check(0); pull_check(1);
 /* 1000 through 32 slots, 7 at a time */
 for (unsigned in=0,out=0;out<1000;) {
  for (unsigned k=0;k<7 && in<1000;k++) assert(rx_one(in++,-50)==KEPT);
  while (out<in) pull_check(out++);
 }
 /* the counts wrap past 2^32: the slots go on in order */
 pthread_mutex_lock(&data_lock); data_rx.head=data_rx.tail=0xfffffff0u; pthread_mutex_unlock(&data_lock);
 for (unsigned i=0;i<33;i++) assert(rx_one(i,-50)==(i<32 ? KEPT : FUXI_WIFI_DATA_DROP_FULL));
 for (unsigned i=0;i<32;i++) pull_check(i);
 assert(data_rx.head==0x10u && data_rx.tail==0x10u);
 /* EAPOL: 8 of its own, apart from the data */
 for (unsigned i=0;i<10;i++) { size_t len=mk(qf,0x88,0x02,SELF,AP,AP,SNAP,0x888e,99,(uint8_t)i);
  assert(fuxi_wifi_data_rx(qf,len,-50)==(i<8 ? KEPT : FUXI_WIFI_DATA_DROP_FULL)); }
 assert(rx_one(5,-50)==KEPT && !fuxi_wifi_data_eapol_pull(o,1513,&n));
 for (unsigned i=0;i<8;i++) { el=eth(e,SELF,AP,0x888e,99,(uint8_t)i); assert(fuxi_wifi_data_eapol_pull(o,sizeof(o),&n) && n==el && !memcmp(o,e,el)); }
 assert(!fuxi_wifi_data_eapol_pull(o,sizeof(o),&n)); pull_check(5); assert(!fuxi_wifi_data_pull(o,sizeof(o),&n));
 counts(&c);
 assert(c.rx_frames==40+2+1000+33+10+1 && c.rx_kept==32+2+1000+32+8+1 && c.rx_eapol==8 && c.rx_pulled==32+2+1000+32+1);
 assert(c.eapol_pulled==8 && c.rx_dropped[FUXI_WIFI_DATA_DROP_FULL]==8+1+2 && c.rx_frames==c.rx_kept+sum(c.rx_dropped));
 /* TX: 32 wait, then FULL; what en0 may not send */
 for (unsigned i=0;i<33;i++) assert(send_one(i)==(i<32 ? KEPT : FUXI_WIFI_DATA_DROP_FULL));
 el=eth(e,AP,SELF,0x888e,50,0); assert(fuxi_wifi_data_send(e,el)==FUXI_WIFI_DATA_DROP_EAPOL);
 el=eth(e,PEER,PEER,0x0800,50,0); assert(fuxi_wifi_data_send(e,el)==FUXI_WIFI_DATA_DROP_SOURCE);
 el=eth(e,PEER,SELF,0x05dc,50,0); assert(fuxi_wifi_data_send(e,el)==FUXI_WIFI_DATA_DROP_LLC);
 assert(fuxi_wifi_data_send(e,13)==FUXI_WIFI_DATA_DROP_LENGTH && fuxi_wifi_data_send(e,1515)==FUXI_WIFI_DATA_DROP_LENGTH);
 assert(fuxi_wifi_data_send(NULL,60)==FUXI_WIFI_DATA_DROP_LENGTH);
 /* no room for the first: it is dropped and counted; bad arguments take nothing */
 static uint8_t m[1600]; id=9;
 assert(!fuxi_wifi_data_tx_take(m,131,&id) && !id && !fuxi_wifi_data_tx_take(NULL,1600,&id) && !fuxi_wifi_data_tx_take(m,1600,NULL));
 uint32_t ids[64];
 for (unsigned i=1;i<32;i++) ids[i]=take_check(i);
 assert(!fuxi_wifi_data_tx_take(m,sizeof(m),&id) && !id);
 /* 32 in flight at most: the next waits for a done() */
 for (unsigned i=0;i<5;i++) assert(send_one(40+i)==KEPT);
 ids[32]=take_check(40);
 assert(!fuxi_wifi_data_tx_take(m,sizeof(m),&id));
 fuxi_wifi_data_tx_done(ids[1],0); ids[33]=take_check(41); assert(!fuxi_wifi_data_tx_take(m,sizeof(m),&id));
 fuxi_wifi_data_tx_done(999999,0); fuxi_wifi_data_tx_done(0,0); fuxi_wifi_data_tx_done(ids[1],0);
 fuxi_wifi_data_tx_done(ids[2],EIO); fuxi_wifi_data_tx_done(ids[2],0);
 for (unsigned i=3;i<34;i++) fuxi_wifi_data_tx_done(ids[i],0);
 for (unsigned i=42;i<45;i++) fuxi_wifi_data_tx_done(take_check(i),0);
 assert(!fuxi_wifi_data_tx_take(m,sizeof(m),&id));
 /* 1000 frames, 5 at a time */
 for (unsigned in=0,out=0;out<1000;) {
  for (unsigned k=0;k<5 && in<1000;k++) assert(send_one(in++)==KEPT);
  while (out<in) fuxi_wifi_data_tx_done(take_check(out++),0);
 }
 counts(&c);
 assert(c.tx_frames==33+6+5+1000 && c.tx_queued==32+5+1000 && c.tx_taken==1+31+5+1000 && c.tx_ok==1+31+3+1000);
 assert(c.tx_failed==1 && c.tx_stale==4 && c.tx_dropped[FUXI_WIFI_DATA_DROP_FULL]==1 && c.tx_dropped[FUXI_WIFI_DATA_DROP_EAPOL]==1);
 assert(c.tx_dropped[FUXI_WIFI_DATA_DROP_LENGTH]==3+1 && c.tx_dropped[FUXI_WIFI_DATA_DROP_SOURCE]==1 && c.tx_dropped[FUXI_WIFI_DATA_DROP_LLC]==1);
 assert(c.tx_frames==c.tx_queued+sum(c.tx_dropped)-1);
 /* a leave with frames everywhere: all flushed, the producer told, the old ids stale */
 for (unsigned i=0;i<3;i++) assert(rx_one(i,-50)==KEPT);
 { size_t len=mk(qf,0x88,0x02,SELF,AP,AP,SNAP,0x888e,99,1); assert(fuxi_wifi_data_rx(qf,len,-50)==KEPT && fuxi_wifi_data_rx(qf,len,-50)==KEPT); }
 for (unsigned i=0;i<4;i++) assert(send_one(i)==KEPT);
 ids[0]=take_check(0); ids[1]=take_check(1);
 fuxi_wifi_data_leave();
 struct fuxi_wifi_data_link l=now_link();
 assert(l.state==FUXI_WIFI_DATA_IDLE && !l.seq && !l.status && l.epoch==epoch+1 && !l.rssi);
 take_req(&r); assert(r.leave && r.seq>jlink.seq && !r.ssid_length && !r.bssid[0] && !fuxi_wifi_data_take(&r));
 assert(!fuxi_wifi_data_pull(o,sizeof(o),&n) && !fuxi_wifi_data_eapol_pull(o,sizeof(o),&n) && !fuxi_wifi_data_tx_take(m,sizeof(m),&id));
 fuxi_wifi_data_tx_done(ids[0],0); fuxi_wifi_data_tx_done(ids[1],0);
 counts(&c); assert(c.links_left==1 && c.rx_flushed==5 && c.tx_flushed==4 && c.tx_stale==6 && c.links_up==1 && c.joins==1);
 /* down: nothing passes either way; a second leave changes nothing */
 assert(rx_one(0,-50)==FUXI_WIFI_DATA_DROP_DOWN && send_one(0)==FUXI_WIFI_DATA_DROP_DOWN);
 fuxi_wifi_data_leave(); assert(now_link().epoch==epoch+1 && !fuxi_wifi_data_take(&r));
 assert(!loglen);
 puts("queue: PASS");
}

/* ---- D3: link-state races ------------------------------------------------------------------------------ */
static void scenario_races(void) {
 static uint8_t o[1600]; size_t n,len; struct fuxi_wifi_data_request r, r2; struct fuxi_wifi_data_counts c; uint32_t id;
 assert(fuxi_wifi_data_use(SELF)); fuxi_wifi_data_ready(true);
 /* link_up for a request not yet taken, for another seq, for 0, from another BSSID, and link_down for another seq */
 pthread_t t=join_start(AP,2412);
 fuxi_wifi_data_link_up(data_seq,AP,2412); assert(still_waiting());
 take_req(&r);
 fuxi_wifi_data_link_up(r.seq+1,AP,2412); fuxi_wifi_data_link_up(0,AP,2412); fuxi_wifi_data_link_up(r.seq,AP2,2412);
 fuxi_wifi_data_link_up(r.seq,NULL,2412); fuxi_wifi_data_link_down(r.seq+5,EIO); fuxi_wifi_data_link_down(0,EIO);
 assert(still_waiting() && now_link().state==FUXI_WIFI_DATA_JOINING);
 counts(&c); assert(c.link_stale==6 && !c.links_up);      /* (a NULL BSSID is not even counted) */
 /* while joining: EAPOL from the BSSID asked for is kept (the 4-way handshake comes before the link), data is not */
 len=mk(qf,0x88,0x02,SELF,AP,AP,SNAP,0x888e,99,1); assert(fuxi_wifi_data_rx(qf,len,-50)==KEPT);
 len=mk(qf,0x88,0x02,SELF,AP2,AP2,SNAP,0x888e,99,1); assert(fuxi_wifi_data_rx(qf,len,-50)==FUXI_WIFI_DATA_DROP_BSSID);
 assert(rx_one(1,-50)==FUXI_WIFI_DATA_DROP_DOWN && send_one(1)==FUXI_WIFI_DATA_DROP_DOWN);
 fuxi_wifi_data_link_up(r.seq,AP,5180);
 assert(!join_end(t) && jlink.mhz==5180);
 assert(fuxi_wifi_data_eapol_pull(o,sizeof(o),&n) && n==113);          /* the link that came of it keeps it */
 /* a new join replaces the link: what waited is flushed; the producer refuses (its errno) */
 assert(rx_one(2,-50)==KEPT && send_one(2)==KEPT);
 t=join_start(AP,2412); take_req(&r2); assert(r2.seq>r.seq && !r2.leave);
 assert(!fuxi_wifi_data_pull(o,sizeof(o),&n) && !fuxi_wifi_data_tx_take(o,sizeof(o),&id));
 fuxi_wifi_data_link_down(r.seq,EIO);                  /* the old link's late word changes nothing */
 assert(still_waiting());
 fuxi_wifi_data_link_down(r2.seq,EHOSTUNREACH); assert(join_end(t)==EHOSTUNREACH && jlink.state==FUXI_WIFI_DATA_IDLE);
 assert(now_link().status==EHOSTUNREACH);
 /* refused without an errno: ECONNREFUSED */
 t=join_start(AP,2412); take_req(&r2); fuxi_wifi_data_link_down(r2.seq,0); assert(join_end(t)==ECONNREFUSED);
 t=join_start(AP,2412); take_req(&r2); fuxi_wifi_data_link_down(r2.seq,-5); assert(join_end(t)==ECONNREFUSED);
 /* a join that times out after the producer took it: ETIMEDOUT, the producer told to leave, a late link_up ignored */
 t=join_start(AP,2412); take_req(&r2);
 now_abs+=4999*NSEC_PER_MSEC; assert(still_waiting());
 now_abs+=NSEC_PER_MSEC; assert(join_end(t)==ETIMEDOUT);
 take_req(&r); assert(r.leave && r.seq>r2.seq);
 fuxi_wifi_data_link_up(r2.seq,AP,2412); assert(now_link().state==FUXI_WIFI_DATA_IDLE && now_link().status==ETIMEDOUT);
 /* ... before it took it: withdrawn, nothing to leave */
 t=join_start(AP,2412); now_abs+=5*NSEC_PER_SEC; assert(join_end(t)==ETIMEDOUT && !fuxi_wifi_data_take(&r));
 /* the longest wait is 10 s whatever is asked (0 counts as 10 s too) */
 jtimeout=60000; t=join_start(AP,2412); take_req(&r2); now_abs+=9999*NSEC_PER_MSEC; assert(still_waiting());
 now_abs+=NSEC_PER_MSEC; assert(join_end(t)==ETIMEDOUT); take_req(&r); assert(r.leave);
 jtimeout=0; t=join_start(AP,2412); take_req(&r2); now_abs+=10*NSEC_PER_SEC; assert(join_end(t)==ETIMEDOUT); take_req(&r);
 jtimeout=5000;
 /* given up (Wi-Fi off asked for): ECANCELED, the producer told */
 t=join_start(AP,2412); take_req(&r2); atomic_store(&jstop,true); assert(join_end(t)==ECANCELED);
 take_req(&r); assert(r.leave);
 /* a leave by the consumer during a join: the waiter sees ECANCELED */
 t=join_start(AP,2412); take_req(&r2); fuxi_wifi_data_leave(); assert(join_end(t)==ECANCELED); take_req(&r); assert(r.leave);
 /* lost while frames wait and are in flight: all flushed, every done() stale, nothing passes */
 uint32_t seq=up_at(AP,2412);
 for (unsigned i=0;i<4;i++) { assert(rx_one(i,-50)==KEPT && send_one(i)==KEPT); }
 uint32_t a=take_check(0), b=take_check(1);
 counts(&c); unsigned stale=c.tx_stale, lost=c.links_lost, flushed_rx=c.rx_flushed, flushed_tx=c.tx_flushed;
 fuxi_wifi_data_link_down(seq,ECONNABORTED);
 struct fuxi_wifi_data_link l=now_link(); assert(l.state==FUXI_WIFI_DATA_IDLE && l.status==ECONNABORTED && !l.seq);
 assert(!fuxi_wifi_data_pull(o,sizeof(o),&n) && !fuxi_wifi_data_tx_take(o,sizeof(o),&id));
 fuxi_wifi_data_tx_done(a,0); fuxi_wifi_data_tx_done(b,EIO);
 assert(rx_one(0,-50)==FUXI_WIFI_DATA_DROP_DOWN && send_one(0)==FUXI_WIFI_DATA_DROP_DOWN);
 counts(&c); assert(c.tx_stale==stale+2 && c.links_lost==lost+1 && c.rx_flushed==flushed_rx+4 && c.tx_flushed==flushed_tx+4);
 assert(!fuxi_wifi_data_take(&r));                     /* the producer said it: no leave for it */
 /* lost without an errno: ECONNRESET; the second word for it is stale */
 seq=up_at(AP,2412); fuxi_wifi_data_link_down(seq,0); assert(now_link().status==ECONNRESET);
 counts(&c); unsigned st=c.link_stale; fuxi_wifi_data_link_down(seq,0); counts(&c); assert(c.link_stale==st+1);
 /* the producer goes away: the link with it (ENETDOWN), what waited for take() dropped, joins refused */
 seq=up_at(AP,2412); assert(rx_one(0,-50)==KEPT);
 fuxi_wifi_data_leave(); fuxi_wifi_data_ready(false);
 assert(!fuxi_wifi_data_take(&r) && !fuxi_wifi_data_present() && now_link().state==FUXI_WIFI_DATA_IDLE);
 fuxi_wifi_data_ready(true); seq=up_at(AP,2412); fuxi_wifi_data_ready(false);
 assert(now_link().status==ENETDOWN && now_link().state==FUXI_WIFI_DATA_IDLE);
 memset(&jreq,0,sizeof(jreq)); memcpy(jreq.bssid,AP,6); assert(fuxi_wifi_data_join(&jreq,100,NULL,NULL)==EOPNOTSUPP);
 fuxi_wifi_data_ready(true); seq=up_at(AP,2412);
 counts(&c); assert(c.rx_frames==c.rx_kept+sum(c.rx_dropped) && c.links_up==c.links_lost+c.links_left+1);
 assert(!loglen);
 puts("races: PASS");
}

/* ---- D4: stress ---------------------------------------------------------------------------------------- */
static atomic_bool s_quit; static atomic_uint s_ups, s_downs, s_done_calls, s_pulled, s_rx_calls;
static void *s_producer(void *a) {
 static uint8_t f[1600], m[1600]; uint32_t cur=0, index=0, id; (void)a;
 while (!atomic_load(&s_quit)) {
  struct fuxi_wifi_data_request r; size_t n;
  if (fuxi_wifi_data_take(&r)) {
   if (r.leave) cur=0;
   else if (rnd2()%10) { fuxi_wifi_data_link_up(r.seq,r.bssid,r.mhz); cur=r.seq; index=0; atomic_fetch_add(&s_ups,1); }
   else { fuxi_wifi_data_link_down(r.seq,ECONNREFUSED); cur=0; }
  }
  if (cur) {
   for (unsigned k=rnd2()%8;k;k--) {
    size_t len=mk(f,0x88,0x02,SELF,AP,PEER,SNAP,0x0800,40,0);
    memcpy(f+34,&cur,4); memcpy(f+38,&index,4); index++;
    int why=fuxi_wifi_data_rx(f,len,-50); atomic_fetch_add(&s_rx_calls,1);
    assert(why==KEPT || why==FUXI_WIFI_DATA_DROP_FULL || why==FUXI_WIFI_DATA_DROP_DOWN);
   }
   if (rnd2()%300==0) { fuxi_wifi_data_link_down(cur,ECONNRESET); cur=0; atomic_fetch_add(&s_downs,1); }
   if (rnd2()%50==0) { fuxi_wifi_data_link_up(cur+1,AP,2412); fuxi_wifi_data_link_down(cur ? cur-1 : 7,EIO); }
  }
  while ((n=fuxi_wifi_data_tx_take(m,sizeof(m),&id))!=0) {
   assert(n==132 && !memcmp(m+4,AP,6) && !memcmp(m+10,SELF,6));
   if (rnd2()%4) { fuxi_wifi_data_tx_done(id,rnd2()%5 ? 0 : EIO); atomic_fetch_add(&s_done_calls,1); }
  }
  if (rnd2()%3==0) sched_yield();
 }
 return NULL;
}
static void *s_puller(void *a) {
 static uint8_t o[1600]; uint32_t last_seq=0, last_index=0; size_t n; (void)a;
 while (!atomic_load(&s_quit)) {
  while (fuxi_wifi_data_pull(o,sizeof(o),&n)) {
   uint32_t seq, index; assert(n==54); memcpy(&seq,o+14,4); memcpy(&index,o+18,4);
   /* in order, and never a frame of an older link after a newer one's */
   assert(seq>last_seq || (seq==last_seq && index>last_index));
   last_seq=seq; last_index=index; atomic_fetch_add(&s_pulled,1);
  }
  sched_yield();
 }
 return NULL;
}
static void scenario_stress(void) {
 pthread_t p, q; struct fuxi_wifi_data_counts c; unsigned joined=0;
 assert(fuxi_wifi_data_use(SELF)); fuxi_wifi_data_ready(true);
 assert(!pthread_create(&p,NULL,s_producer,NULL) && !pthread_create(&q,NULL,s_puller,NULL));
 for (unsigned i=0;i<400;i++) {
  memset(&jreq,0,sizeof(jreq)); memcpy(jreq.bssid,AP,6); jreq.mhz=2412;
  int status=fuxi_wifi_data_join(&jreq,5000,NULL,NULL);
  assert(status==0 || status==ECONNREFUSED || status==ECONNRESET || status==ECANCELED);
  joined+=status==0;
  for (unsigned k=0;k<20;k++) { int why=send_one(k); assert(why==KEPT || why==FUXI_WIFI_DATA_DROP_FULL || why==FUXI_WIFI_DATA_DROP_DOWN); }
  if (i%3==0) fuxi_wifi_data_leave();
  if (i%50==49) { fuxi_wifi_data_ready(false); fuxi_wifi_data_ready(true); }
  sched_yield();
 }
 atomic_store(&s_quit,true); pthread_join(p,NULL); pthread_join(q,NULL);
 counts(&c);
 assert(c.rx_frames==atomic_load(&s_rx_calls) && c.rx_frames==c.rx_kept+sum(c.rx_dropped));
 assert(c.rx_kept==c.rx_pulled+c.rx_flushed+(data_rx.head-data_rx.tail) && c.rx_pulled==atomic_load(&s_pulled));
 assert(c.tx_ok+c.tx_failed+c.tx_stale==atomic_load(&s_done_calls) && c.tx_frames==c.tx_queued+sum(c.tx_dropped));
 assert(c.joins==400 && joined>100 && c.links_up==atomic_load(&s_ups) && c.rx_pulled>1000 && c.tx_taken>1000);
 printf("stress: PASS (%u joins up, %u lost by the producer, %u frames pulled, %u sent)\n",joined,atomic_load(&s_downs),
  c.rx_pulled,c.tx_taken);
}

/* ---- D5: fuxiwifiios=5 end to end with a fake radio lane ----------------------------------------------- */
enum { P_ACCEPT, P_REFUSE, P_SILENT };
static atomic_int p_mode; static atomic_bool p_quit; static atomic_uint p_joins, p_leaves, p_tx, p_scans, p_cmd;
static pthread_mutex_t p_lock=PTHREAD_MUTEX_INITIALIZER; static struct fuxi_wifi_data_request p_last;
static uint8_t p_txf[8][1600]; static size_t p_txn[8];
static uint8_t p_in[2400]; static size_t p_in_n; static atomic_int p_in_state; static int p_in_why;
static struct ies ies_open, ies_lock; static uint8_t b_open[600], b_lock[600]; static size_t b_open_n, b_lock_n;
static void *producer_run(void *a) {
 static uint8_t mpdu[1600]; struct timespec pause={.tv_nsec=200000}; uint32_t live=0; (void)a;
 while (!atomic_load(&p_quit)) {
  struct fuxi_wifi_radio_request s; struct fuxi_wifi_data_request d; uint32_t id; size_t n;
  if (fuxi_wifi_radio_take(&s)) {
   for (unsigned i=0;i<s.freq_count;i++) {
    if (s.freqs[i]==2437) assert(fuxi_wifi_radio_frame(s.seq,b_open,b_open_n,2437,-55,-95)==FUXI_WIFI_RADIO_KEPT);
    if (s.freqs[i]==5180) assert(fuxi_wifi_radio_frame(s.seq,b_lock,b_lock_n,5180,-60,-95)==FUXI_WIFI_RADIO_KEPT);
   }
   fuxi_wifi_radio_done(s.seq,0); atomic_fetch_add(&p_scans,1);
  }
  if (fuxi_wifi_data_take(&d)) {
   pthread_mutex_lock(&p_lock); p_last=d; pthread_mutex_unlock(&p_lock);
   if (d.leave) { live=0; atomic_fetch_add(&p_leaves,1); }
   else {
    int mode=atomic_load(&p_mode);
    if (mode==P_ACCEPT) { live=d.seq; fuxi_wifi_data_link_up(d.seq,d.bssid,d.mhz); }
    else if (mode==P_REFUSE) fuxi_wifi_data_link_down(d.seq,ECONNREFUSED);
    atomic_fetch_add(&p_joins,1);
   }
  }
  while ((n=fuxi_wifi_data_tx_take(mpdu,sizeof(mpdu),&id))!=0) {
   unsigned k=atomic_load(&p_tx); if (k<8) { memcpy(p_txf[k],mpdu,n); p_txn[k]=n; }
   fuxi_wifi_data_tx_done(id,0); atomic_fetch_add(&p_tx,1);
  }
  if (atomic_load(&p_in_state)==1) { p_in_why=fuxi_wifi_data_rx(p_in,p_in_n,-47); atomic_store(&p_in_state,2); }
  if (atomic_exchange(&p_cmd,0)==1) fuxi_wifi_data_link_down(live,ECONNABORTED);
  nanosleep(&pause,NULL);
 }
 return NULL;
}
static int inject(const uint8_t *f,size_t n) {
 struct timespec pause={.tv_nsec=100000};
 memcpy(p_in,f,n); p_in_n=n; atomic_store(&p_in_state,1);
 for (unsigned i=0;i<50000 && atomic_load(&p_in_state)!=2;i++) nanosleep(&pause,NULL);
 assert(atomic_load(&p_in_state)==2); atomic_store(&p_in_state,0); return p_in_why;
}
static void wait_true(bool (*f)(void)) {
 struct timespec pause={.tv_nsec=500000};
 for (unsigned i=0;i<10000 && !f();i++) nanosleep(&pause,NULL);
 assert(f());
}
static unsigned rx_seen(void) { pthread_mutex_lock(&rx_lock); unsigned n=received; pthread_mutex_unlock(&rx_lock); return n; }
static bool link_is_down(void) { return !__atomic_load_n(&link_up,__ATOMIC_SEQ_CST); }
static void join_ask(const char *ssid,const uint8_t *bssid,const char *key,bool wait) {
 struct apple80211_assoc j; memset(&j,0,sizeof(j)); j.version=1; j.ssid_length=(uint32_t)strlen(ssid); memcpy(j.ssid,ssid,j.ssid_length);
 if (bssid) memcpy(j.bssid,bssid,6);
 if (key) { j.key_cipher=CIPHER_PASSPHRASE; j.key_length=(uint32_t)strlen(key); memcpy(j.key,key,j.key_length); }
 assert(!request(true,20,&j,sizeof(j),NULL)); if (wait) settle();
}
static int assoc_result(void) { int v; assert(!request(false,21,NULL,0,&v)); return v; }
static int state_now(void) { int v; assert(!request(false,13,NULL,0,&v)); return v; }
static void beacons_make(void) {
 put(&ies_open,0,"zqOpen",6); put(&ies_open,1,RATES,8); put(&ies_open,3,"\x06",1);
 b_open_n=frame_of(b_open,0x80,AP,0x01,&ies_open);
 put(&ies_lock,0,"zqLock",6); put(&ies_lock,1,RATES5,8); put(&ies_lock,48,RSN,20);
 b_lock_n=frame_of(b_lock,0x80,AP2,0x11,&ies_lock);
}
static void boot5(void) {
 assert(!atomic_load(&kallocs));
 opt=5; fuxi_wifi_ios_init();
 assert(said_times("FUXI WIFI IOS: enabled, radio data backend (radio scans, open joins, en0 frames through the data handoff); "
  "Apple80211 16.7 ABI unverified\n")==1);
 assert(attached==1 && registry==1 && fuxi_wifi_sta_present() && atomic_load(&kallocs)>=5 && data_used && data_store);
 other_settle();
}
static void scenario_backend(void) {
 static uint8_t f[1600], e[1600], w[1600]; size_t len, el, wn, n; char name[256]; pthread_t producer;
 beacons_make(); fill_all(); boot5();
 memset(name,1,sizeof(name));
 assert(!request(false,43,name,sizeof(name),NULL) && !strcmp(name,"Fuxi Wi-Fi iOS shim (radio data backend)"));
 /* the radio lane: READY with its channels, as with =3 */
 fuxi_wifi_radio_starting(); fuxi_wifi_radio_ready(all38,38);
 wait_said("FUXI WIFI IOS: radio: radio producer ready, 38 channels (13 at 2.4 GHz, 25 at 5 GHz), passive only\n");
 other_settle();
 assert(!pthread_create(&producer,NULL,producer_run,NULL));
 /* no data producer yet: refused at once, nothing posted (ASSOCIATE_RESULT 2, event 9, no link) */
 join_ask("zqOpen",NULL,NULL,true);
 assert(assoc_result()==2 && atomic_load(&events[9])==1 && !atomic_load(&events[4]) && !link_up && !atomic_load(&p_joins));
 assert(said(SAY("FUXI WIFI IOS: data: join refused (no radio data producer), no radio access, status %d\n",EOPNOTSUPP)));
 /* a scan, as =3 makes it */
 assert(!scan_ask(3,0,0)); settle();
 assert(atomic_load(&events[10])==1 && atomic_load(&p_scans)==1);
 assert(said("FUXI WIFI IOS: radio: scan 1: 38 channels passive, 2 heard (1 at 2.4 GHz, 1 at 5 GHz, 1 RSN, 0 hidden), 0 ms, status 0\n"));
 /* the data producer is there: the open network joined, by the name alone */
 fuxi_wifi_data_ready(true);
 join_ask("zqOpen",NULL,NULL,true);
 pthread_mutex_lock(&p_lock); struct fuxi_wifi_data_request d=p_last; pthread_mutex_unlock(&p_lock);
 assert(!d.leave && d.seq && d.ssid_length==6 && !memcmp(d.ssid,"zqOpen",6) && !memcmp(d.bssid,AP,6) && d.mhz==2437);
 assert(d.security==FUXI_WIFI_STA_OPEN && atomic_load(&p_joins)==1);
 assert(assoc_result()==1 && atomic_load(&events[9])==2 && atomic_load(&events[4])==1 && link_up && state_now()==4);
 assert(said("FUXI WIFI IOS: data: join done, 2437 MHz, open, 0 ms, status 0\n"));
 char ssid[32]={0}; uint8_t bssid[6]={0};
 assert(!request(false,1,ssid,32,NULL) && !memcmp(ssid,"zqOpen",6) && !request(false,9,bssid,6,NULL) && !memcmp(bssid,AP,6));
 /* RX: IPv4 and ARP reach en0 as Ethernet; EAPOL waits apart for a supplicant; a protected frame goes nowhere */
 len=mk(f,0x88,0x02,SELF,AP,PEER,SNAP,0x0800,200,1); el=eth(e,SELF,PEER,0x0800,200,1);
 assert(inject(f,len)==KEPT);
 struct timespec pause={.tv_nsec=500000};
 for (unsigned i=0;i<10000 && rx_seen()<1;i++) nanosleep(&pause,NULL);
 assert(rx_seen()==1 && rx_got_n[0]==el && !memcmp(rx_got[0],e,el));
 len=mk(f,0x08,0x02,BCAST,AP,PEER,SNAP,0x0806,28,2); el=eth(e,BCAST,PEER,0x0806,28,2);
 assert(inject(f,len)==KEPT);
 for (unsigned i=0;i<10000 && rx_seen()<2;i++) nanosleep(&pause,NULL);
 assert(rx_seen()==2 && rx_got_n[1]==el && !memcmp(rx_got[1],e,el));
 len=mk(f,0x88,0x02,SELF,AP,AP,SNAP,0x888e,95,3); assert(inject(f,len)==KEPT);
 f[1]|=0x40; assert(inject(f,len)==FUXI_WIFI_DATA_DROP_PROTECTED);
 nanosleep(&pause,NULL); nanosleep(&pause,NULL);
 assert(rx_seen()==2 && fuxi_wifi_data_eapol_pull(w,sizeof(w),&n) && n==109);
 struct fuxi_wifi_sta_state *st=calloc(1,sizeof(*st));
 fuxi_wifi_sta_state(st); assert(st->phase==FUXI_WIFI_STA_JOINED && st->bss.rssi==-47 && st->bss.frequency==2437 && !st->last_error);
 /* TX: en0's frame to the AP as an MPDU; EAPOL and an oversize frame from en0 refused */
 el=eth(e,PEER,SELF,0x0800,300,4); assert(transmit(e,el));
 for (unsigned i=0;i<10000 && atomic_load(&p_tx)<1;i++) nanosleep(&pause,NULL);
 assert(fuxi_wifi_data_from_8023(e,el,AP,SELF,w,sizeof(w),&wn)==KEPT && p_txn[0]==wn && !memcmp(p_txf[0],w,wn));
 assert(p_txf[0][1]==0x01 && !memcmp(p_txf[0]+4,AP,6) && !memcmp(p_txf[0]+16,PEER,6));
 el=eth(e,AP,SELF,0x888e,50,5); assert(!transmit(e,el)); assert(!transmit(e,1515));
 /* the radio loses the link: en0 down, wifid told it was lost (event 4), the counts in the log */
 atomic_store(&p_cmd,1); wait_true(link_is_down); other_settle();
 for (unsigned i=0;i<10000 && atomic_load(&events[4])<2;i++) nanosleep(&pause,NULL);
 assert(atomic_load(&events[4])==2 && state_now()==0);
 assert(said(SAY("FUXI WIFI IOS: data: link lost, status %d; rx 4 frames, 3 kept (1 EAPOL), 1 dropped; tx 2 frames, 1 queued, "
  "1 dropped, 1 ok, 0 failed\n",ECONNABORTED)));
 assert(said(SAY("FUXI WIFI IOS: apple80211: the station says: the link, %d\n",ECONNABORTED)));
 fuxi_wifi_sta_state(st); assert(st->phase==FUXI_WIFI_STA_IDLE && st->last_error==ECONNABORTED);
 el=eth(e,PEER,SELF,0x0800,300,4); assert(!transmit(e,el));
 /* joined again, to the BSSID wifid names; then left when asked: the producer told, no loss said */
 join_ask("zqOpen",AP,NULL,true);
 assert(assoc_result()==1 && link_up && atomic_load(&events[4])==3 && atomic_load(&p_joins)==2);
 unsigned leaves=atomic_load(&p_leaves);
 assert(!request(true,22,NULL,0,NULL)); settle();
 for (unsigned i=0;i<10000 && atomic_load(&p_leaves)<leaves+1;i++) nanosleep(&pause,NULL);
 assert(atomic_load(&p_leaves)==leaves+1 && !link_up && atomic_load(&events[4])==4 && said_times("data: link lost")==1);
 assert(said("FUXI WIFI IOS: data: left, status 0; rx 4 frames, 3 kept (1 EAPOL), 1 dropped; tx 3 frames, 1 queued, 2 dropped"));
 assert(said("FUXI WIFI IOS: apple80211: the station says: the link, 0\n"));
 /* refused without the radio: WPA2, an open join to a protected network, a network not heard, a BSSID not heard */
 unsigned joins=atomic_load(&p_joins);
 join_ask("zqLock",NULL,"fuxitest1",true); assert(assoc_result()==2);
 assert(said(SAY("FUXI WIFI IOS: data: join refused (WPA2 needs a supplicant, not in this build), no radio access, status %d\n",EOPNOTSUPP)));
 join_ask("zqLock",NULL,NULL,true); assert(assoc_result()==2);
 assert(said(SAY("FUXI WIFI IOS: data: join refused (the network is protected, open joins only), no radio access, status %d\n",EOPNOTSUPP)));
 join_ask("zqNone",NULL,NULL,true); assert(assoc_result()==2);
 assert(said(SAY("FUXI WIFI IOS: data: join refused (not heard by the last scan), no radio access, status %d\n",ENOENT)));
 join_ask("zqOpen",AP2,NULL,true); assert(assoc_result()==2 && atomic_load(&p_joins)==joins && !link_up);
 /* the producer refuses */
 atomic_store(&p_mode,P_REFUSE); join_ask("zqOpen",NULL,NULL,true);
 assert(assoc_result()==2 && !link_up && atomic_load(&p_joins)==joins+1);
 assert(said(SAY("FUXI WIFI IOS: data: join failed, 2437 MHz, open, 0 ms, status %d\n",ECONNREFUSED)));
 /* the producer never answers: after 10 s, ETIMEDOUT (ASSOCIATE_RESULT 9), and it is told to give up */
 atomic_store(&p_mode,P_SILENT); leaves=atomic_load(&p_leaves);
 join_ask("zqOpen",NULL,NULL,false);
 for (unsigned i=0;i<10000 && atomic_load(&p_joins)<joins+2;i++) nanosleep(&pause,NULL);
 assert(state_now()==2 && fuxi_wifi_sta_busy());
 now_abs=10*NSEC_PER_SEC; settle();
 assert(assoc_result()==9 && !link_up);
 for (unsigned i=0;i<10000 && atomic_load(&p_leaves)<leaves+1;i++) nanosleep(&pause,NULL);
 assert(atomic_load(&p_leaves)==leaves+1);
 assert(said(SAY("FUXI WIFI IOS: data: join failed, 2437 MHz, open, 10000 ms, status %d\n",ETIMEDOUT)));
 /* off while joined: the producer told to leave, en0 down; on again */
 atomic_store(&p_mode,P_ACCEPT); join_ask("zqOpen",NULL,NULL,true); assert(assoc_result()==1 && link_up);
 leaves=atomic_load(&p_leaves); unsigned ev4=atomic_load(&events[4]);
 assert(!power_set(false) && !power_now() && !link_up && atomic_load(&events[4])==ev4+1);
 for (unsigned i=0;i<10000 && atomic_load(&p_leaves)<leaves+1;i++) nanosleep(&pause,NULL);
 assert(atomic_load(&p_leaves)==leaves+1 && !power_set(true) && power_now()==1);
 /* the radio lane stops while joined: the link lost with it, Wi-Fi off for the boot */
 join_ask("zqOpen",NULL,NULL,true); assert(assoc_result()==1 && link_up);
 ev4=atomic_load(&events[4]); leaves=atomic_load(&p_leaves);
 fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | 5);
 wait_said("FUXI WIFI IOS: radio: the radio lane stopped (code 0x40005); Wi-Fi reported off, no retry this boot\n");
 other_settle(); settle();
 assert(!link_up && atomic_load(&events[4])==ev4+1 && !power_now());
 assert(said(SAY("FUXI WIFI IOS: data: link lost with the radio, status %d;",ENETDOWN)));
 for (unsigned i=0;i<10000 && atomic_load(&p_leaves)<leaves+1;i++) nanosleep(&pause,NULL);
 assert(atomic_load(&p_leaves)==leaves+1 && now_link().state==FUXI_WIFI_DATA_IDLE);
 atomic_store(&p_quit,true); pthread_join(producer,NULL);
 free(st); other_settle(); clean_log();
 puts("backend: PASS");
}

/* ---- D6: no producer at all ---------------------------------------------------------------------------- */
static void scenario_noproducer(void) {
 boot5();
 assert(other_armed()==30*NSEC_PER_SEC);
 /* nothing reported: a join is refused cleanly at once, nothing posted, no link */
 join_ask("zqOpen",NULL,NULL,true);
 assert(assoc_result()==2 && !link_up && atomic_load(&events[9])==1 && !atomic_load(&events[4]));
 assert(said(SAY("FUXI WIFI IOS: data: join refused (no radio data producer), no radio access, status %d\n",EOPNOTSUPP)));
 /* a data producer alone, no scan producer: nothing heard, so nothing to join */
 fuxi_wifi_data_ready(true); join_ask("zqOpen",NULL,NULL,true); assert(assoc_result()==2 && !link_up);
 assert(said(SAY("FUXI WIFI IOS: data: join refused (not heard by the last scan), no radio access, status %d\n",ENOENT)));
 struct fuxi_wifi_data_request r; assert(!fuxi_wifi_data_take(&r));
 /* the first scan finds no radio lane: Wi-Fi off, as =3 */
 assert(!scan_ask(3,0,0)); settle();
 wait_said("FUXI WIFI IOS: radio: no radio lane reported before iOS first asked (code 0x50001); Wi-Fi reported off, no retry this boot\n");
 other_settle();
 assert(!power_now() && scan_ask(3,0,0)==ENETDOWN);
 struct apple80211_assoc j={.version=1,.ssid_length=6}; memcpy(j.ssid,"zqOpen",6);
 assert(request(true,20,&j,sizeof(j),NULL)==ENETDOWN && !fuxi_wifi_data_take(&r));
 clean_log();
 puts("noproducer: PASS");
}

/* ---- D7: no fuxiwifiios=5 ------------------------------------------------------------------------------- */
static void scenario_default(void) {
 struct fuxi_wifi_data_request r; uint8_t m[1600]; uint32_t id=3; size_t n=4;
 fuxi_wifi_data_ready(true);
 assert(rx_one(0,-50)==FUXI_WIFI_DATA_DROP_DOWN && !fuxi_wifi_data_take(&r) && !fuxi_wifi_data_tx_take(m,sizeof(m),&id) && !id);
 fuxi_wifi_data_link_up(1,AP,2412); fuxi_wifi_data_link_down(1,EIO); fuxi_wifi_data_tx_done(1,0); fuxi_wifi_data_leave();
 opt=0; fuxi_wifi_ios_init(); opt=6; fuxi_wifi_ios_init(); opt=7; fuxi_wifi_ios_init(); opt=-1; fuxi_wifi_ios_init();
 assert(!atomic_load(&kallocs) && !loglen && !other_call.func && !attached && !registry && !fuxi_wifi_sta_present());
 assert(!data_used && !data_store && data_ready_word && !fuxi_wifi_data_present() && !fuxi_wifi_data_pull(m,sizeof(m),&n) && !n);
 struct fuxi_wifi_data_counts c; counts(&c);
 for (unsigned i=0;i<sizeof(c);i++) assert(!((uint8_t *)&c)[i]);
 assert(now_link().state==FUXI_WIFI_DATA_IDLE && !now_link().epoch);
 puts("default: PASS");
}

int main(int argc,char **argv) {
 assert(argc==2); alarm(60);
 const char *s=argv[1];
 if (!strcmp(s,"convert")) scenario_convert();
 else if (!strcmp(s,"queue")) scenario_queue();
 else if (!strcmp(s,"races")) scenario_races();
 else if (!strcmp(s,"stress")) scenario_stress();
 else if (!strcmp(s,"backend")) scenario_backend();
 else if (!strcmp(s,"noproducer")) scenario_noproducer();
 else if (!strcmp(s,"default")) scenario_default();
 else return 2;
 return 0;
}
'''

# ---- D8: the default backends, as salami has them ------------------------------------------------------
# One transcript of a boot with fuxiwifiios absent/1/2/3/4: what is said and answered. It uses only what
# salami's units have, so it builds against both trees; the two outputs must be equal byte for byte.
TRANSCRIPT = r'''
extern unsigned int alarm(unsigned int seconds);
bool fuxi_wifi_net_attach(const struct fuxi_wifi_net_link *p) {
 printf("T net_attach %s wireless %d up %d\n",p->name,p->wireless,p->up); attached++; return true;
}
static void settle(void) {
 struct timespec pause={.tv_nsec=1000000};
 for (unsigned i=0;i<5000;i++) { if (!fuxi_wifi_sta_busy()) return; nanosleep(&pause,NULL); }
 assert(!"station timeout");
}
static int ask(bool set,int type,int value,void *p,unsigned n,int *val) {
 struct apple80211req req={.req_if_name="en0",.req_type=type,.req_val=value,.req_len=n,.req_data=(uintptr_t)p};
 int error=fuxi_wifi_apple80211_ioctl(set?SIOCSA80211:SIOCGA80211,&req);
 if (val) *val=req.req_val;
 return error;
}
static void wait_log(const char *what) {
 struct timespec pause={.tv_nsec=1000000};
 for (unsigned i=0;i<5000;i++) { pthread_mutex_lock(&log_lock); bool in=strstr(logbuf,what)!=NULL; pthread_mutex_unlock(&log_lock);
  if (in) return;
  nanosleep(&pause,NULL);
 }
 assert(!"never said");
}
static atomic_bool lane_quit;
static uint8_t beacon[128]; static size_t beacon_n;
static void *lane(void *a) {
 struct timespec pause={.tv_nsec=200000}; (void)a;
 while (!atomic_load(&lane_quit)) {
  struct fuxi_wifi_radio_request r;
  if (!fuxi_wifi_radio_take(&r)) { nanosleep(&pause,NULL); continue; }
  for (unsigned i=0;i<r.freq_count;i++) if (r.freqs[i]==2437) (void)fuxi_wifi_radio_frame(r.seq,beacon,beacon_n,2437,-51,-95);
  fuxi_wifi_radio_done(r.seq,0);
 }
 return NULL;
}
int main(int argc,char **argv) {
 int v=0, e; char name[256]; pthread_t t; bool laned=false;
 assert(argc==2); alarm(30); opt=atoi(argv[1]);
 fuxi_wifi_ios_init();
 printf("T init attached %u registry %u present %d kallocs %u call %d\n",attached,registry,fuxi_wifi_sta_present(),
  atomic_load(&kallocs),other_call.func!=NULL);
 if (!fuxi_wifi_sta_present()) { fuxi_wifi_ios_init(); printf("T end, log %zu bytes\n",loglen); return 0; }
 other_settle();
 if (opt==3) {
  static const uint8_t b[]={0x80,0,0,0, 0xff,0xff,0xff,0xff,0xff,0xff, 2,0x54,0x52,0x41,0x4e,1, 2,0x54,0x52,0x41,0x4e,1, 0,0,
   0,0,0,0,0,0,0,0, 100,0, 0x01,0, 0,4,'T','r','a','n', 1,2,0x82,0x84, 3,1,6};
  uint16_t fr[3]={2412,2437,5180};
  memcpy(beacon,b,sizeof(b)); beacon_n=sizeof(b);
  fuxi_wifi_radio_starting(); fuxi_wifi_radio_ready(fr,3); wait_log("producer ready"); other_settle();
  assert(!pthread_create(&t,NULL,lane,NULL)); laned=true;
 }
 if (opt==4) { wait_log("producer ready"); other_settle(); }
 struct apple80211_scan_request scan; memset(&scan,0,sizeof(scan)); scan.version=1; scan.scan_type=3;
 e=ask(true,10,0,&scan,sizeof(scan),NULL); settle(); printf("T scan %d, events %u\n",e,atomic_load(&events[10]));
 for (unsigned i=0;i<8;i++) {
  struct apple80211_scan_result res; memset(&res,0,sizeof(res)); res.elements_length=1024;
  e=ask(false,11,0,&res,sizeof(res),NULL);
  uint32_t h=2166136261u; for (unsigned k=0;k<sizeof(res);k++) h=(h^((uint8_t *)&res)[k])*16777619u;
  printf("T result %d: %u bytes hashed %08x\n",e,(unsigned)sizeof(res),h);
  if (e) break;
 }
 memset(name,0,sizeof(name)); e=ask(false,43,0,name,sizeof(name),NULL); printf("T name %d %s\n",e,name);
 struct apple80211_assoc j; memset(&j,0,sizeof(j)); j.version=1; j.ssid_length=13; memcpy(j.ssid,"Fuxi Loopback",13);
 e=ask(true,20,0,&j,sizeof(j),NULL); settle(); ask(false,21,0,NULL,0,&v);
 printf("T join %d result %d link %d events %u %u\n",e,v,link_up,atomic_load(&events[9]),atomic_load(&events[4]));
 ask(false,13,0,NULL,0,&v); printf("T state %d\n",v);
 uint8_t frame[100]={0}; frame[12]=0x88; frame[13]=0xb5;
 printf("T transmit %d received %u\n",transmit(frame,60),received);
 frame[13]=0x8e; printf("T transmit eapol %d\n",transmit(frame,60));
 frame[12]=0x08; frame[13]=0x00; printf("T transmit ip %d\n",transmit(frame,60));
 e=ask(true,22,0,NULL,0,NULL); settle(); printf("T leave %d link %d\n",e,link_up);
 struct { uint32_t version,radios,state[4]; } p={.version=1,.radios=1};
 e=ask(true,19,0,&p,sizeof(p),NULL); settle(); printf("T off %d powered %d\n",e,fuxi_wifi_sta_powered());
 p.state[0]=1; e=ask(true,19,0,&p,sizeof(p),NULL); settle(); printf("T on %d powered %d\n",e,fuxi_wifi_sta_powered());
 printf("T events %u %u %u %u, kallocs %u\n",atomic_load(&events[1]),atomic_load(&events[4]),atomic_load(&events[9]),
  atomic_load(&events[10]),atomic_load(&kallocs));
 if (laned) { atomic_store(&lane_quit,true); pthread_join(t,NULL); }
 other_settle();
 return 0;
}
'''

# the =5 hook in fuxi_wifi_ios.c, exactly: salami's file is this file with each new text put back to the old
HOOK = [
    (' * fuxiwifiios=3/4 (K2) list real scan results from the handoff of sys/fuxi_wifi_radio.h, scan only. */',
     ' * fuxiwifiios=3/4 (K2) list real scan results from the handoff of sys/fuxi_wifi_radio.h, scan only.\n'
     ' * fuxiwifiios=5 adds joins and en0\'s frames through sys/fuxi_wifi_data.h (bsd/dev/fuxi_wifi_data.c). */'),
    ('#include <sys/fuxi_wifi_radio.h>\n', '#include <sys/fuxi_wifi_radio.h>\n#include <sys/fuxi_wifi_data.h>\n'),
    ('\tif (on < 1 || on > 4 || enabled) {', '\tif (on < 1 || on > 5 || enabled) {'),
    ('\tenabled = true;\n\tif (on >= 3) {',
     '\tenabled = true;\n\tif (on == 5) {\n\t\tlink.name = "radio-data";\n\t\tfuxi_wifi_data_init(&rscan_backend);\n'
     '\t\treturn;\n\t}\n\tif (on >= 3) {'),
]
NEW_FILES = ('fuxi_wifi_data.c', 'fuxi_wifi_data.h')


def check(name, ok, detail=''):
    RESULTS.append((name, ok))
    print(('PASS ' if ok else 'FAIL ') + name, flush=True)
    if not ok and detail:
        print('    ' + str(detail).replace('\n', '\n    '), flush=True)
    return ok


def unit(model, src, headers, units):
    source = model + ''.join(W.stripped(name, src) for name in headers)
    source += 'bool fuxi_wifi_net_attach(const struct fuxi_wifi_net_link *);\n'
    return source + ''.join(W.stripped(name, src) for name in units)


def compile_unit(tmp, name, source, defs=(), sanitize=True):
    path = tmp / (name + '.c')
    path.write_text(source)
    exe = tmp / name
    p = subprocess.run([CC, *FLAGS, *(SAN if sanitize else ()), *defs, str(path), '-o', str(exe)],
                       capture_output=True, text=True)
    return exe if p.returncode == 0 else p.stderr[-3000:]


def build(tmp, name, src=SRC, defs=(), sanitize=True):
    return compile_unit(tmp, name, W.unit_source(W.MODEL.replace(RX_STUB, RX_RECORD), src) + TEST, defs, sanitize)


def run(exe, scenario, timeout=90):
    try:
        p = subprocess.run([str(exe), scenario], capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, 'timeout'
    return p.returncode == 0 and f'{scenario}: PASS' in p.stdout, (p.stdout + p.stderr)[-2500:]


# ---- D8 -------------------------------------------------------------------------------------------------
def git(*args):
    return subprocess.run(['git', '-C', str(ROOT), *args], capture_output=True, text=True)


def salami_base():
    for ref in ('origin/salami', 'salami'):
        if git('rev-parse', '--verify', '--quiet', ref + '^{commit}').returncode == 0:
            base = git('merge-base', 'HEAD', ref)
            if base.returncode == 0:
                return ref, base.stdout.strip()
    return None, None


def salami_suite(tmp):
    ref, base = salami_base()
    if base is None:
        print('SKIP D8 salami: no salami branch in this checkout (fetch it to compare)', flush=True)
        return
    listing = git('ls-tree', '--name-only', base, 'xnu/fuxi-wifi-ios/')
    if not check(f'D8 salami: the merge base with {ref} ({base[:12]}) has the 0104 sources', listing.returncode == 0 and
                 listing.stdout.strip(), listing.stderr):
        return
    old = tmp / 'salami-src'
    old.mkdir()
    differ = []
    for path in listing.stdout.split():
        name = Path(path).name
        text = git('show', f'{base}:{path}').stdout
        (old / name).write_text(text)
        current = (SRC / name).read_text() if (SRC / name).exists() else None
        if name == 'fuxi_wifi_ios.c' and current is not None and current != text:
            hooked = current
            for before, after in HOOK:
                hooked = hooked.replace(after, before, 1) if hooked.count(after) == 1 else '\0'
            if hooked != text:
                differ.append(name + ' (more than the =5 hook)')
        elif current != text:
            differ.append(name)
    extra = sorted(p.name for p in SRC.iterdir() if not (old / p.name).exists() and p.name not in NEW_FILES)
    check('D8 salami: every 0104 source but fuxi_wifi_ios.c byte-identical to the salami merge base, fuxi_wifi_ios.c '
          'only the =5 hook, no new file but fuxi_wifi_data.c/.h', not differ and not extra, differ + extra)
    have = {p.name for p in old.iterdir()}
    exes = {}
    for label, src in (('salami', old), ('this', SRC)):
        names = {p.name for p in Path(src).iterdir()}
        headers = [h for h in W.HEADERS if h in names]
        units = [u for u in W.UNITS if u in names]
        exe = compile_unit(tmp, 'transcript-' + label, unit(W.MODEL, src, headers, units) + TRANSCRIPT)
        if not check(f'D8 salami: the transcript builds against {label} sources', not isinstance(exe, str),
                     exe if isinstance(exe, str) else ''):
            return
        exes[label] = exe
    assert have
    for value in ('0', '1', '2', '3', '4'):
        outs = {}
        for label, exe in exes.items():
            p = subprocess.run([str(exe), value], capture_output=True, text=True, timeout=60)
            outs[label] = (p.returncode, p.stdout)
        # the answers in their order; the log line for line (a station thread's line and the ioctl's own can
        # come in either order, in both trees)
        def key(out):
            lines = out[1].splitlines(True)
            return out[0], [x for x in lines if x.startswith('T ')], sorted(lines)
        same = key(outs['salami']) == key(outs['this']) and outs['this'][0] == 0
        diff = '' if same else ''.join(difflib.unified_diff(outs['salami'][1].splitlines(True),
                                                            outs['this'][1].splitlines(True), 'salami', 'this'))
        what = 'absent' if value == '0' else value
        check(f'D8 salami: fuxiwifiios={what}: answers and every log line byte-identical to salami '
              f'({len(outs["this"][1])} bytes)', same, diff[-2500:] or outs)


# ---- D9: static rules -----------------------------------------------------------------------------------
LOCKED_CALLS = {'memcpy', 'bzero', 'memcmp', 'assert_wait_timeout'}
PRODUCER = ('fuxi_wifi_data_ready', 'fuxi_wifi_data_take', 'fuxi_wifi_data_link_up', 'fuxi_wifi_data_link_down',
            'fuxi_wifi_data_rx', 'fuxi_wifi_data_tx_take', 'fuxi_wifi_data_tx_done')
PRODUCER_INSIDE = ('fuxi_wifi_data_to_8023', 'fuxi_wifi_data_from_8023', 'data_tunnelled')
IOS_SIDE = re.compile(r'\b(fuxi_wifi_(sta|apple80211|net|radio|ios)_\w+|rdata_\w+)\s*\(')


def static_rules(src=SRC):
    problems = []
    raw = (Path(src) / 'fuxi_wifi_data.c').read_text()
    code = R.uncomment(raw)
    funcs = R.bodies(code)
    spans = {}
    for lock in ('data_lock', 'rdata_lock'):
        spans[lock] = []
        locks = [m.end() for m in re.finditer(r'lck_mtx_lock\(&%s\);' % lock, code)]
        if len(locks) != len(re.findall(r'lck_mtx_unlock\(&%s\);' % lock, code)):
            problems.append(f'{lock}: lock and unlock counts differ')
        for at in locks:
            end = code.find(f'lck_mtx_unlock(&{lock});', at)
            region = code[at:end] if end >= 0 else ''
            if end < 0 or 'lck_mtx_lock(' in region or re.search(r'\b(return|goto)\b', region):
                problems.append(f'{lock} region at {at}: not closed before another lock, a return or a goto')
            spans[lock].append((at, end))
            for c in R.calls(region):
                if c not in LOCKED_CALLS and not (lock == 'data_lock' and c.endswith('_locked')):
                    problems.append(f'{lock} region at {at}: calls {c}')
    locked_spans = list(spans['data_lock'])
    for name, (at, body) in funcs.items():
        if name.endswith('_locked'):
            locked_spans.append((at, at + len(body)))
            for c in R.calls(body):
                if c not in LOCKED_CALLS - {'assert_wait_timeout'} and not c.endswith('_locked'):
                    problems.append(f'{name}: calls {c} (under data_lock)')
            if 'lck_mtx_' in body:
                problems.append(f'{name}: takes or drops a lock')
    for m in re.finditer(r'\b(\w+_locked)\s*\(', code):
        if m.start() == code.rfind('\n', 0, m.start()) + 1:
            continue
        if not any(a <= m.start() < b for a, b in locked_spans):
            problems.append(f'{m.group(1)} called without data_lock held')
    for name in PRODUCER + PRODUCER_INSIDE:
        if name not in funcs:
            problems.append(f'{name}: not found')
            continue
        for c in R.calls(funcs[name][1]):
            if c in R.SLEEPS or IOS_SIDE.match(c + '('):
                problems.append(f'producer {name}: calls {c}')
    backend = {n for n in funcs if n.startswith('rdata_') or n == 'fuxi_wifi_data_init'}
    for name, (at, body) in funcs.items():
        if name in backend:
            touched = sorted(set(re.findall(r'(?<![\w])data_\w+', body)))
            if touched:
                problems.append(f'backend {name}: touches the handoff\'s own {touched}')
        else:
            if IOS_SIDE.search(body) or re.search(r'\brdata_\w+', body):
                problems.append(f'handoff {name}: calls the iOS side or the backend')
    bad = re.findall(r'\b(wl_\w+|wl\d+\w*|fw_\w+|ml_phys\w*|IOMemoryDescriptor|pmap_\w+)\b', code)
    if bad:
        problems.append(f'fuxi_wifi_data.c names {sorted(set(bad))}')
    for m in re.finditer(r'printf\("((?:\\.|[^"\\])*)"', raw):
        if re.search(r'%[0-9]*x|%\.\*s|%s', m.group(1)) and 'join refused (%s)' not in m.group(1) and \
                'data: %s, status' not in m.group(1) and 'join %s,' not in m.group(1):
            problems.append(f'a format that could carry a name or an address: {m.group(1)[:60]}')
    ios = R.uncomment((Path(src) / 'fuxi_wifi_ios.c').read_text())
    if sorted(set(re.findall(r'\bfuxi_wifi_data_\w+', ios))) != ['fuxi_wifi_data_init']:
        problems.append('fuxi_wifi_ios.c names more of the data handoff than fuxi_wifi_data_init')
    return problems


# ---- D10: mutants ---------------------------------------------------------------------------------------
MUTANTS = [
    ('no flush on a link change', 'fuxi_wifi_data.c', '\tif (state != FUXI_WIFI_DATA_UP) {\n\t\tdata_flush_locked();', '\tif (false) {\n\t\tdata_flush_locked();'),
    ('To DS accepted', 'fuxi_wifi_data.c', 'if ((mpdu[1] & 0x03) != 0x02) {', 'if ((mpdu[1] & 0x02) != 0x02) {'),
    ('QoS Control not stepped over', 'fuxi_wifi_data.c', 'header += 2;                            /* QoS Control */',
     'header += 0;'),
    ('HT Control not stepped over', 'fuxi_wifi_data.c', 'header += 4;', 'header += 0;'),
    ('Protected passes', 'fuxi_wifi_data.c', 'if ((mpdu[1] & 0x40) != 0) {', 'if (false) {'),
    ('a fragment number passes', 'fuxi_wifi_data.c', '|| (mpdu[22] & 0x0f) != 0', ''),
    ('an A-MSDU passes', 'fuxi_wifi_data.c', 'if (subtype == 8 && (mpdu[24] & 0x80) != 0) {', 'if (false) {'),
    ('Null subtypes pass', 'fuxi_wifi_data.c', '(subtype != 0 && subtype != 8)', '(subtype & 3) != 0'),
    ('EAPOL with the data', 'fuxi_wifi_data.c', '*eapol = type == FUXI_WIFI_DATA_ETHERTYPE_EAPOL;', '*eapol = false;'),
    ('IPX not tunnelled', 'fuxi_wifi_data.c', 'return type == 0x80f3 || type == 0x8137;', 'return type == 0x80f3;'),
    ('one frame over the ring', 'fuxi_wifi_data.c', 'if (ring->head - ring->tail >= most) {',
     'if (ring->head - ring->tail > most) {'),
    ('1515 bytes kept', 'fuxi_wifi_data.c', 'if (payload > FUXI_WIFI_DATA_FRAME_MAX - 14) {',
     'if (payload > FUXI_WIFI_DATA_FRAME_MAX - 13) {'),
    ('a stale link_up taken', 'fuxi_wifi_data.c', 'JOINING && seq != 0 && seq == data_live && !data_posted &&',
     'JOINING && seq != 0 &&'),
    ('link_up from any BSSID', 'fuxi_wifi_data.c', '!data_posted &&\n\t    memcmp(bssid, data_bssid, sizeof(data_bssid)) == 0) {',
     '!data_posted) {'),
    ('a stale link_down taken', 'fuxi_wifi_data.c', 'seq != 0 && seq == data_live && !data_posted && data_state != FUXI_WIFI_DATA_IDLE',
     'data_state != FUXI_WIFI_DATA_IDLE'),
    ('a join never times out', 'fuxi_wifi_data.c', 'return late ? ETIMEDOUT : -1;', 'return -1;'),
    ('no leave after a timeout', 'fuxi_wifi_data.c', '\t\t(void)data_post_locked(NULL);', '\t\t;'),
    ('to anyone', 'fuxi_wifi_data.c', 'if ((mpdu[4] & 1) == 0 && memcmp(mpdu + 4, self, 6) != 0) {', 'if (false) {'),
    ('our own frames back', 'fuxi_wifi_data.c', ' || memcmp(mpdu + 16, self, 6) == 0', ''),
    ('EAPOL from en0', 'fuxi_wifi_data.c', 'why = FUXI_WIFI_DATA_DROP_EAPOL;', 'why = FUXI_WIFI_DATA_KEPT;'),
    ('data while joining', 'fuxi_wifi_data.c', '(why == FUXI_WIFI_DATA_KEPT && !eapol && data_state != FUXI_WIFI_DATA_UP)',
     'false'),
    ('no in-flight bound', 'fuxi_wifi_data.c',
     'data_state == FUXI_WIFI_DATA_UP && data_inflight_count < FUXI_WIFI_DATA_TX_INFLIGHT',
     'data_state == FUXI_WIFI_DATA_UP && data_inflight_count < 64'),
    ('a loss not told to wifid', 'fuxi_wifi_data.c', '\t\t\trdata_tell();\n\t\t\tfuxi_wifi_sta_lost(error);',
     '\t\t\trdata_tell();'),
    ('en0 up after a leave', 'fuxi_wifi_data.c', '\tfuxi_wifi_data_leave();\n\trdata_tell();\n\treturn was;',
     '\tfuxi_wifi_data_leave();\n\treturn was;'),
    ('joins without a producer posted', 'fuxi_wifi_data.c', 'if (!fuxi_wifi_data_present()) {', 'if (false) {'),
    ('WPA2 joined open', 'fuxi_wifi_data.c', '} else if (what->security != FUXI_WIFI_STA_OPEN) {', '} else if (false) {'),
    ('the radio stop keeps the link', 'fuxi_wifi_data.c', '\tif (rdata_drop()) {\n\t\trdata_say_counts("link lost with',
     '\tif (false) {\n\t\trdata_say_counts("link lost with'),
    ('the lock not taken in rx()', 'fuxi_wifi_data.c',
     '\tlck_mtx_lock(&data_lock);\n\tdata_counted.rx_frames++;', '\tdata_counted.rx_frames++;'),
    ('the producer logs', 'fuxi_wifi_data.c', '\tif (!used) {\n\t\treturn FUXI_WIFI_DATA_DROP_DOWN;\n\t}\n\t/* (data_store',
     '\tif (!used) {\n\t\tprintf("FUXI WIFI IOS: data: rx before use\\n");\n\t\treturn FUXI_WIFI_DATA_DROP_DOWN;\n\t}\n\t/* (data_store'),
    ('a BSSID in the log', 'fuxi_wifi_data.c', '\tprintf("FUXI WIFI IOS: data: join %s, %u MHz',
     '\tprintf("FUXI WIFI IOS: data: %02x\\n", request.bssid[5]);\n\tprintf("FUXI WIFI IOS: data: join %s, %u MHz'),
    ('=5 is the scan backend', 'fuxi_wifi_ios.c', '\t\tfuxi_wifi_data_init(&rscan_backend);', '\t\trscan_init(false);'),
    ('=6 enabled', 'fuxi_wifi_ios.c', 'if (on < 1 || on > 5 || enabled) {', 'if (on < 1 || on > 6 || enabled) {'),
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
            print(f'    mutant "{name}": caught by D9 static', flush=True)
            continue
        exe = build(tmp, 'm-' + re.sub(r'\W+', '-', name), mdir, sanitize=False)
        if isinstance(exe, str):
            survived.append(f'{name}: does not compile: {exe[-300:]}')
            continue
        by = next((s for s in SCENARIOS if not run(exe, s, timeout=60)[0]), None)
        (killed if by else survived).append(name)
        print(f'    mutant "{name}": ' + (f'caught by {by}' if by else 'SURVIVED'), flush=True)
    check(f'D10 mutants: {len(killed)} of {len(MUTANTS)} source mutations caught', not survived, survived)


def main():
    subprocess.run(['python3', str(ROOT / 'tools/sync-fuxi-wifi-ios.py'), '--check'], check=True)
    problems = static_rules()
    check('D9 static: data_lock and rdata_lock are leaf locks, the producer never sleeps or calls the iOS side, the '
          'handoff and the backend apart, no address or name format in the log', not problems, problems)
    with tempfile.TemporaryDirectory(prefix='fuxi-wifi-data-') as t:
        tmp = Path(t)
        for label, defs in (('fuxi', ()), ('salami', ('-DFUXI_SALAMI=1',))):
            exe = build(tmp, 'data-' + label, defs=defs)
            if not check(f'{label}: production units with the data handoff compile warning-free (-Wall -Wextra -Werror, '
                         f'ASan/UBSan)', not isinstance(exe, str), exe if isinstance(exe, str) else ''):
                continue
            for s in SCENARIOS:
                ok, out = run(exe, s)
                check(f'{label} {s}', ok, out)
        salami_suite(tmp)
        mutant_suite(tmp)
    bad = [n for n, ok in RESULTS if not ok]
    print(f'wifi data tests: {len(RESULTS) - len(bad)} PASS, {len(bad)} FAIL', flush=True)
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
