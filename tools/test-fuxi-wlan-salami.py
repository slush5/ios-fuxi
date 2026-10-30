#!/usr/bin/env python3
"""K-W1 and K-W1b (salami, OnePlus 11): run the production WLAN C under -DFUXI_SALAMI=1.

Host-only, synthetic cmd-db and RSC registers, fake boot-args and a fake ADT
/defaults. No device, firmware, kernel build or network. Checks:
  - patch 0099 embeds the exact fuxi_wlan.c, fuxi_wlan_ce.h (W20) and wlan.c;
  - step 1 expects the salami cmd-db (ldob15 0x40b00), refuses 0x40a00
    (salami ldob12) with reason 5 and any other mismatch with reason 2;
  - steps 2 and 4 send exactly the salami rails ON at the salami live-DT mV;
  - rh_send_request allows, on the eight salami rails only, the live-DT mV,
    enable 1 and mode 7 (K-W1b), and refuses every OFF, every other mode
    value, headroom, 0x40a00, other VRMs and a VRM address in a BCM batch
    before any TCS write;
  - fuxiwlanrails=1/2 (K-W1b) sends exactly the 18 salami ON-only commands
    (voltage, enable 1, mode 7 for six rails, salami table); cyclerails never
    sends an OFF, and a Fuxi cmd-db table stops the re-vote before any write;
  - fuxi_wlan_init starts the thread only with fuxiwlan>0 AND fuxiwlansalami=1,
    accepts fuxiwlanrails 0..2, refuses cyclerails and invalid selectors
    before it, and selects the firmware basenames (salami Android defaults,
    boot-arg overrides);
  - every fuxiwlan* knob comes from the boot-args first, else from ADT
    /defaults (PE_get_default semantics: absent or too large = absent), in the
    encoding tools/mkadt-salami.py --wlan-defaults writes; a boot-arg wins,
    absent from both is off, and the result equals the same knobs as boot-args;
  - mkadt-salami.py --wlan-defaults: the knob table is the driver's, typed as
    the driver reads it, refuses what the kernel could not read, and reports
    the exact bytes each property adds;
  - the Fuxi build (no FUXI_SALAMI) keeps its defaults, ignores /defaults and
    starts as before;
  - W20 (fuxiwlanqrtr=6) and W23 (fuxiwlanqrtr=7, C2) start only with all 28
    steps; qrtr outside 0..7, keep/stop22 with qrtr 6/7, fuxiwlanw20 without
    qrtr 6/7, w20 bits outside the set of the selected stage (0x08 ACTIVE in
    none) and 0x01 with 0x02 are refused before any access; the init lines
    print the real qrtr;
  - C2c (W26): qrtr 7 takes w20 0x100 (only with 0x02), 0x200 and 0x400, alone
    or together, from the cmdline and from ADT /defaults (the kits' route: the
    r1c2 cmdline, a 3- or 4-digit w20 in /defaults); qrtr 6 refuses all three;
    keep/stop22 with them is refused; a boot without them prints no W26 line;
  - C2d (W27): qrtr 7 takes w20 0x800 and 0x1000, each only with 0x100, from the
    cmdline and from ADT /defaults (6402, 6914); qrtr 6 refuses both; a boot
    without them prints no W27 line.
"""
from pathlib import Path
import importlib.util
import os
import re
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
DRIVER = (ROOT / 'xnu/fuxi-wlan/fuxi_wlan.c').read_text()
CE_HEADER = (ROOT / 'xnu/fuxi-wlan/fuxi_wlan_ce.h').read_text()
SHIM = (ROOT / 'shim/fuxi-probe/wlan.c').read_text()
CC = [os.environ.get('CC', 'cc'), '-std=c11', '-D_DEFAULT_SOURCE', '-g',
      '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
SALAMI_RAILS = [0x40b00, 0x40200, 0x40400, 0x40500, 0x44000, 0x40600, 0x43900, 0x43a00]


def between(text, start, end, keep_start=True):
    body = text.split(start, 1)[1].split(end, 1)[0]
    return (start + body) if keep_start else body


def no_asm(text):
    return re.sub(r'__asm__ volatile\([^;]+;', '/* host: no barrier */;', text)


def load_mkadt():
    spec = importlib.util.spec_from_file_location('mkadt_salami', ROOT / 'tools/mkadt-salami.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_patch():
    patch = (ROOT / 'xnu/patches/0099-fuxi-wlan.patch').read_text()
    for dest, source in (('fuxi_wlan.c', DRIVER), ('fuxi_wlan_ce.h', CE_HEADER), ('wlan.c', SHIM)):
        added = patch.split(f'+++ b/bsd/dev/{dest}\n', 1)[1].split('\n--- a/', 1)[0]
        count = re.match(r'@@ -0,0 \+1,(\d+) @@\n', added)
        assert count and int(count[1]) == len(source.splitlines()), dest
        body = '\n'.join(l[1:] for l in added.splitlines() if l.startswith('+')) + '\n'
        assert body == source, f'0099 {dest} differs from its canonical source'
    # W20 (C1): the header is a new file of its own, after fuxi_wlan.c and before wlan.c (the m6 tests
    # read wlan.c to the end of the patch); header only, so bsd/conf/files is unchanged.
    order = [m for m in re.findall(r'^\+\+\+ b/(\S+)$', patch, re.M)]
    assert order == ['bsd/conf/files', 'bsd/dev/sweet_ufs.c', 'bsd/dev/fuxi_wlan.c', 'bsd/dev/fuxi_wlan_ce.h',
                     'bsd/dev/wlan.c'], order
    assert '--- a/bsd/dev/fuxi_wlan_ce.h\n+++ b/bsd/dev/fuxi_wlan_ce.h\n@@ -0,0 +1,' in patch
    assert DRIVER.count('#include "fuxi_wlan_ce.h"') == 1 and 'fuxi_wlan_ce' not in patch.split('--- a/bsd/dev/fuxi_wlan.c', 1)[0]
    assert 'The Clear BSD License' in CE_HEADER and '#include' not in CE_HEADER
    # The Fuxi branch keeps its literals; salami-only code is all under FUXI_SALAMI.
    assert 'static const uint32_t expect[8] = {0x40a00, 0x40200, 0x40400, 0x40500, 0x44000, 0x40600, 0x43900, 0x43a00};' in SHIM
    assert 'static const uint16_t mv[6] = {1000, 1800, 980, 952, 1350, 1900};' in SHIM
    assert '#define WL14_REGDB_NAME "regdb_xiaomi.bin"' in DRIVER and '#define WL14_M3_NAME "m3.bin"' in DRIVER
    assert DRIVER.count('"regdb_xiaomi.bin"') == 1 and DRIVER.count('"m3.bin"') == 1
    assert 'wl14_download(WL14_REGDB_NAME,' in DRIVER and 'wl14_read(WL14_M3_NAME,' in DRIVER
    step18 = between(DRIVER, '\tcase 18: {', '\tcase 19:')
    assert step18.index('if (wl_amss_path[0]) { wl_fw_path = wl_amss_path; }') < step18.index('if (wl_fw) {')
    # The policy check precedes every TCS register access in rh_send_request.
    req = between(DRIVER, 'rh_send_request(unsigned t,', 'static uint32_t\nrh_send_batch(')
    assert req.index('wl_salami_vote_ok(&cmd[i], regulator)') < req.index('usb_rd(') < req.index('rh_wr(')
    # K-W1b: the rails table differs only in the salami ldob15 row; Fuxi keeps 0x40a00.
    assert ('#if defined(FUXI_SALAMI)\n\t{0, 0x40b00, 1800, "ldob15"},\n#else\n\t{0, 0x40a00, 1800, "ldob15"},\n#endif\n'
            '\t{4, 0x44000, 1000, "smpg5"},') in DRIVER
    # K-W1b: the knob source exists only under FUXI_SALAMI, before the first parser call, and is the
    # only place that reads /defaults or calls the real parsers.
    head = '#if defined(FUXI_SALAMI)\n/* ---- K-W1b salami knob source ----'
    assert DRIVER.count(head) == 1 and DRIVER.count('/* ---- end K-W1b salami knob source ---- */\n#endif\n') == 1
    block = between(DRIVER, '/* ---- K-W1b salami knob source ----', '/* ---- end K-W1b salami knob source ---- */')
    first_call = min(DRIVER.index('PE_parse_boot_argn("'), DRIVER.index('PE_parse_boot_arg_str("'))
    assert DRIVER.index(head) < first_call
    assert DRIVER.count('PE_get_default(') == 1 and 'PE_get_default(' in block
    assert DRIVER.count('(PE_parse_boot_argn)(') == 1 and DRIVER.count('(PE_parse_boot_arg_str)(') == 1
    assert '#define PE_parse_boot_argn(name, buf, size) wl_arg((name), (buf), (size))' in block
    assert '#define PE_parse_boot_arg_str(name, buf, size) wl_arg_str((name), (buf), (size))' in block
    assert 'PE_parse_boot' not in SHIM and 'PE_get_default' not in SHIM
    print('0099 embeds fuxi_wlan.c, fuxi_wlan_ce.h and wlan.c; Fuxi literals, salami guards and the knob source are where expected')


RPMH_PRE = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
static char logs[262144];
static size_t loglen;
static void test_printf(const char *fmt, ...) {
    va_list ap; va_start(ap,fmt);
    int n=vsnprintf(logs+loglen,sizeof(logs)-loglen,fmt,ap);
    va_end(ap); assert(n>=0 && (size_t)n<sizeof(logs)-loglen); loglen+=n;
}
#define printf test_printf
#define RH_DRV_PA 0x17a20000UL
#define RH_TCS_PA 0x17a20d00UL
#define TS_TLMM 0x0f100000UL
#define GCC_PA 0x00100000UL
#define CMDDB_PA 0x81c60000UL
#define WL_O_CYCLERAILS 256u
static uint8_t db[0x20000];
static uint32_t rsc[1024], fw_faults, fw_fault_esr, probe_fault_esr;
static uint64_t fw_fault_far, probe_fault_far, clk;
static unsigned tcs_writes, other_writes, triggers, gpio_calls;
struct sent { uint32_t addr, data, msgid, trigger; };
static struct sent sent[256];
static unsigned nsent;
static uint64_t host_rsr(const char *r) { return strcmp(r,"cntfrq_el0") ? (clk += 1000) : 1000000; }
#define rsr(r) host_rsr(r)
static uint32_t usb_rd(uintptr_t base, uint32_t off) {
    if(base==CMDDB_PA) {
        assert(!(off&3) && off<=sizeof(db)-4);
        return db[off] | (uint32_t)db[off+1]<<8 | (uint32_t)db[off+2]<<16 | (uint32_t)db[off+3]<<24;
    }
    if(base==RH_DRV_PA) { assert(off==0 || off==12); return off ? (16u<<27 | 3u<<12) : 3u<<16; }
    if(base==RH_TCS_PA) { assert(!(off&3) && off/4<1024); return rsc[off/4]; }
    if(base==GCC_PA && off==0) return 0x80000000u; /* GPLL0 locked */
    return 0;
}
static void ts_wr(uintptr_t base, uint32_t off, uint32_t v) {
    if(base!=RH_TCS_PA) { other_writes++; return; }
    assert(off/4<1024 && !(off&3));
    tcs_writes++;
    if(off==8) { rsc[1]&=~v; return; }
    rsc[off/4]=v;
    if(off==0x24 && v==0x1010000) {
        triggers++;
        for(unsigned i=0;i<16 && (rsc[0x2c/4] & (1u<<i));i++) {
            assert(nsent<256);
            sent[nsent++]=(struct sent){rsc[(0x38+i*24)/4],rsc[(0x3c+i*24)/4],rsc[(0x34+i*24)/4],triggers};
            rsc[(0x40+i*24)/4]=0x10100;
        }
        rsc[1]|=1; rsc[0x28/4]=1;
    }
}
static void fw_delay(unsigned us) { (void)us; }
static void delay(unsigned us) { (void)us; }
static void ts_gpio(unsigned n, unsigned func, unsigned pull, unsigned ma, int out) { (void)n; (void)func; (void)pull; (void)ma; (void)out; gpio_calls++; }
static uint32_t fw_safe_rd32(uint64_t pa, uint32_t *faulted) { (void)pa; *faulted=0; return 0; }
static unsigned fw_safe_wr32(uint64_t pa, uint32_t value) { (void)pa; (void)value; other_writes++; return 0; }
#define probe_safe_rd32 fw_safe_rd32
#define probe_safe_wr32 fw_safe_wr32
static unsigned wl_rails_variant, wl_seq_flags;
static void wl_sleep(unsigned ms) { (void)ms; }
'''

RPMH_TESTS = r'''
static const char *const names[] = {"ldob15","smpe4","smpg2","smpg4","smpg5","smpg6","ldoe1","ldoe3","clka1","vrm.wlan","ldob12","ldob4"};
static void put16(unsigned at, unsigned v) { db[at]=(uint8_t)v; db[at+1]=(uint8_t)(v>>8); }
static void put32(unsigned at, uint32_t v) { put16(at,v&0xffff); put16(at+2,v>>16); }
static void cmddb(const uint32_t *addr) {
    memset(db,0,sizeof(db));
    put32(4,0x0c0330db);                 /* magic */
    put16(8,3); put16(10,0); put16(14,12); /* one VRM slave: entries at 144, 12 of them */
    for(unsigned i=0;i<12;i++) {
        unsigned e=144+24*i;
        memcpy(db+e,names[i],strlen(names[i]));
        put32(e+16,addr[i]);
    }
}
static void reset_tcs(void) { memset(rsc,0,sizeof(rsc)); rsc[0x28/4]=1; rh_poisoned=false; fw_faults=0; }
static const uint32_t salami_db[12]={0x40b00,0x40200,0x40400,0x40500,0x44000,0x40600,0x43900,0x43a00,0x45000,0x46808,0x40a00,0x40800};
/* salami live DT mV in wl_salami_rail order (test's own copy, not the driver's table) */
static const uint32_t salami_mv[8]={1800,950,980,1350,1000,1900,880,1200};
static unsigned refused(uint32_t addr, uint32_t data) {
    unsigned w=tcs_writes, n=nsent; size_t l=loglen;
    unsigned e=rh_send(0,addr,data);
    assert(tcs_writes==w && nsent==n);  /* nothing reached the TCS */
    assert(strstr(logs+l,"salami refused RPMh regulator"));
    return e;
}
static void allowed(uint32_t addr, uint32_t data) {
    unsigned w=tcs_writes, n=nsent, t=triggers; size_t l=loglen;
    assert(rh_send(0,addr,data)==0);
    assert(nsent==n+1 && sent[n].addr==addr && sent[n].data==data && sent[n].msgid==0x10108u && sent[n].trigger==t+1);
    assert(tcs_writes>w && !strstr(logs+l,"refused"));
}
int main(void) {
    reset_tcs();
    /* Step 1: salami cmd-db passes and records the name-resolved table. */
    cmddb(salami_db);
    wl3_faults=0;
    assert(wl3_step(1)==0);
    for(unsigned i=0;i<8;i++) assert(wl3_report[20+i]==wl_salami_rail[i]);
    /* Fuxi-valued ldob15 (0x40a00 = salami ldob12) is refused by name, reason 5. */
    uint32_t fuxi_db[12]; memcpy(fuxi_db,salami_db,sizeof(fuxi_db)); fuxi_db[0]=0x40a00;
    cmddb(fuxi_db); assert(wl3_step(1)==5);
    uint32_t other[12]; memcpy(other,salami_db,sizeof(other)); other[2]=0x40401;
    cmddb(other); assert(wl3_step(1)==2);
    other[2]=0x40400; other[7]=0x40a00;  /* 0x40a00 under any name */
    cmddb(other); assert(wl3_step(1)==5);
    assert(tcs_writes==0 && nsent==0);
    /* Steps 2 and 4 through the production transport: exact salami votes. */
    cmddb(salami_db); assert(wl3_step(1)==0);
    assert(wl3_step(2)==0);
    static const uint32_t s2[][2]={{0x44000,1000},{0x44004,1},{0x40b00,1800},{0x40b04,1},{0x40400,980},{0x40404,1},
                                   {0x40200,950},{0x40204,1},{0x40500,1350},{0x40504,1},{0x40600,1900},{0x40604,1}};
    assert(nsent==12);
    for(unsigned i=0;i<12;i++) assert(sent[i].addr==s2[i][0] && sent[i].data==s2[i][1] && sent[i].trigger==i+1);
    assert(wl3_step(4)==0);
    static const uint32_t s4[][2]={{0x43900,880},{0x43904,1},{0x43a00,1200},{0x43a04,1}};
    assert(nsent==16);
    for(unsigned i=0;i<4;i++) assert(sent[12+i].addr==s4[i][0] && sent[12+i].data==s4[i][1]);
    for(unsigned i=0;i<nsent;i++) assert((sent[i].addr&~0xffu)!=0x40a00 && !((sent[i].addr&0xff)==4 && sent[i].data!=1));
    /* K-W1b: on each of the eight salami rails, mode 7 (HPM) is allowed, as are its DT mV and enable 1. */
    for(unsigned i=0;i<8;i++) {
        allowed(SALAMI(i)+8,7);
        allowed(SALAMI(i),salami_mv[i]);
        allowed(SALAMI(i)+4,1);
    }
    /* Every other regulator command is refused before any TCS write. */
    for(unsigned i=0;i<8;i++) {
        uint32_t a=SALAMI(i);
        assert(refused(a+4,0));                                  /* OFF */
        assert(refused(a+4,2) && refused(a+4,7));                /* enable value other than 1 */
        for(uint32_t m=0;m<32;m++) if(m!=7) assert(refused(a+8,m));   /* any other mode value */
        assert(refused(a+8,0x107) && refused(a+8,0x70) && refused(a+8,0xffffffffu));
        assert(refused(a+12,0) && refused(a+12,7));              /* headroom */
        assert(refused(a,1) && refused(a,7) && refused(a,salami_mv[i]+1) && refused(a,salami_mv[i]-1));  /* wrong mV */
        assert(refused(a+9,7) && refused(a+0x10,7) && refused(a+0x88,7) && refused(a+1,7));  /* other registers */
    }
    /* ldob15 (also the USB monitor's eUSB2 repeater, codec, amps, NFC): exactly 1800 mV, enable 1, mode 7. */
    assert(refused(0x40b00,1799) && refused(0x40b00,1801) && refused(0x40b00,1850) && refused(0x40b00,0));
    assert(refused(0x40b04,0) && refused(0x40b08,6) && refused(0x40b08,8) && refused(0x40b08,0));
    /* Mode 7 (or any vote) on any other address is refused: 0x40a00 (ldob12, panel vddio) above all. */
    assert(refused(0x40a00,1800) && refused(0x40a04,1) && refused(0x40a04,0) && refused(0x40a08,7));
    assert(refused(0x40200,952));                         /* the Fuxi S4E value */
    assert(refused(0x40800,1800) && refused(0x40804,1) && refused(0x40808,7));  /* ldob4 (touch) */
    static const uint32_t not_wlan[]={0x45000,0x46808,0x40300,0x40700,0x40c00,0x43800,0x43b00,0x44100,0x40000,0x50000};
    for(unsigned i=0;i<sizeof(not_wlan)/sizeof(not_wlan[0]);i++) {
        assert(refused(not_wlan[i]+8,7) && refused(not_wlan[i]+4,1) && refused(not_wlan[i],1800));
    }
    assert(refused(0x46808,1) && refused(0x40b01,1800) && refused(0x40b0b,7));
    /* BCM batches: a VRM address anywhere in the batch refuses the whole batch, mode 7 too. */
    {
        struct wl_rh_cmd b[2]={{0x50010,0x60004001,0},{0x40b04,0,1}};
        unsigned w=tcs_writes,n=nsent;
        assert(rh_send_batch(0,b,2)==1 && tcs_writes==w && nsent==n);
        assert(strstr(logs,"salami refused RPMh batch addr 0x40b04"));
        b[1]=(struct wl_rh_cmd){0x40b08,7,1};
        assert(rh_send_batch(0,b,2)==1 && tcs_writes==w && nsent==n);
        assert(strstr(logs,"salami refused RPMh batch addr 0x40b08 data 0x7"));
        b[1]=(struct wl_rh_cmd){0x50020,0x40000001,1};
        assert(rh_send_batch(0,b,2)==0 && nsent==n+2 && sent[n].addr==0x50010 && sent[n+1].addr==0x50020);
    }
    /* K-W1b fuxiwlanrails ON-only on salami: the salami table, exactly these 18 commands, one waited
     * regulator request each, in wl_rails order (ldob15 smpg5 smpg2 smpe4 smpg6 smpg4). */
    static const uint32_t on[18][2]={{0x40b00,1800},{0x40b04,1},{0x40b08,7},{0x44000,1000},{0x44004,1},{0x44008,7},
        {0x40400,980},{0x40404,1},{0x40408,7},{0x40200,950},{0x40204,1},{0x40208,7},
        {0x40600,1900},{0x40604,1},{0x40608,7},{0x40500,1350},{0x40504,1},{0x40508,7}};
    cmddb(salami_db); assert(wl3_step(1)==0);
    for(unsigned v=1;v<=2;v++) {
        unsigned n=nsent, t=triggers; size_t l=loglen;
        wl_rails_variant=v;
        assert(wl_rails_on(3-v,"other call site")==0 && nsent==n && triggers==t);   /* the other variant: nothing */
        assert(wl_rails_on(v,"test")==0);
        assert(nsent==n+18 && triggers==t+18);
        for(unsigned i=0;i<18;i++) {
            assert(sent[n+i].addr==on[i][0] && sent[n+i].data==on[i][1]);
            assert(sent[n+i].msgid==0x10108u && sent[n+i].trigger==t+1+i);
        }
        assert(!strstr(logs+l,"refused") && strstr(logs+l,"complete ON-only"));
    }
    wl_rails_variant=0;
    { unsigned n=nsent; assert(wl_rails_on(1,"off")==0 && wl_rails_on(2,"off")==0 && nsent==n); }
    /* cyclerails, if it were ever reached on salami (init refuses it): never an OFF vote. */
    unsigned n0=nsent, w0=tcs_writes; size_t l=loglen;
    wl_seq_flags=WL_O_CYCLERAILS;
    assert(wl_m6o_rails(false)==1 && strstr(logs+l,"salami refuses the cyclerails rail OFF votes"));
    assert(nsent==n0 && tcs_writes==w0);
    assert(wl_m6o_rails(true)==0 && nsent==n0+12);   /* its ON half: only allowed ON votes, mV then enable 1 */
    for(unsigned i=n0;i<nsent;i++) {
        uint32_t reg=sent[i].addr&0xff;
        assert((reg==0 && sent[i].data>=950) || (reg==4 && sent[i].data==1));
    }
    wl_rails_variant=1; n0=nsent;
    assert(wl_m6o_rails(false)==1 && wl_m6o_rails(true)==1 && nsent==n0);   /* rails + cyclerails conflict */
    /* A Fuxi table in r[20..] (ldob15 at 0x40a00): the salami re-vote table refuses before any write. */
    static const uint32_t fuxi_r[8]={0x40a00,0x40200,0x40400,0x40500,0x44000,0x40600,0x43900,0x43a00};
    memcpy(wl3_report+20,fuxi_r,sizeof(fuxi_r));
    n0=nsent; w0=tcs_writes;
    wl_seq_flags=0; wl_rails_variant=1; l=loglen;
    assert(wl_rails_on(1,"test")==1 && strstr(logs+l,"rails skip ldob15 cmd-db addr 0x40a00 expected 0x40b00"));
    wl_rails_variant=2;
    assert(wl_rails_on(2,"test")==1);
    wl_rails_variant=0; wl_seq_flags=WL_O_CYCLERAILS;
    assert(wl_m6o_rails(false)==1 && wl_m6o_rails(true)==1);
    assert(nsent==n0 && tcs_writes==w0);
    puts("K-W1/K-W1b RPMh: salami step 1/2/4 votes exact; mode 7 allowed on the 8 salami rails only; rails ON-only "
         "sends the 18 salami commands; OFF/other modes/headroom/0x40a00/other VRMs refused before any TCS write");
    return 0;
}
'''

GATE_PRE = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>
#include <assert.h>
static char logs[65536];
static size_t loglen;
static void test_printf(const char *fmt, ...) {
    va_list ap; va_start(ap,fmt);
    int n=vsnprintf(logs+loglen,sizeof(logs)-loglen,fmt,ap);
    va_end(ap); assert(n>=0 && (size_t)n<sizeof(logs)-loglen); loglen+=n;
}
#define printf test_printf
typedef uintptr_t vm_offset_t;
typedef size_t vm_size_t;
typedef int wait_result_t;
typedef void *thread_t;
#define KERN_SUCCESS 0
#define MIN(a,b) ((a)<(b)?(a):(b))
static const char *bootargs = "", *adt = "";
static unsigned adt_reads;
/* XNU-style lookup: name=value tokens; numbers into ints, other values copied
 * as strings of at most size-1 bytes plus NUL. */
static const char *arg_value(const char *name) {
    size_t n=strlen(name);
    for(const char *p=bootargs;*p;) {
        while(*p==' ') p++;
        if(!strncmp(p,name,n) && p[n]=='=') return p+n+1;
        while(*p && *p!=' ') p++;
    }
    return NULL;
}
static bool PE_parse_boot_arg_str(const char *name, char *out, int size) {
    const char *v=arg_value(name);
    if(!v) return false;
    int i=0;
    for(;i<size-1 && v[i] && v[i]!=' ';i++) out[i]=v[i];
    out[i]=0;
    return true;
}
static bool PE_parse_boot_argn(const char *name, void *out, int size) {
    const char *v=arg_value(name);
    if(!v) return false;
    if(size==(int)sizeof(int) && ((*v>='0' && *v<='9') || *v=='-')) {
        int x=(int)strtol(v,NULL,0); memcpy(out,&x,sizeof(x)); return true;
    }
    return PE_parse_boot_arg_str(name,out,size);
}
/* XNU pexpert/gen/bootargs.c PE_get_default over a fake /defaults "name:hex,name:hex": absent or larger
 * than max = FALSE, otherwise exactly the property's bytes are copied (nothing else of the buffer). */
static bool PE_get_default(const char *name, void *out, unsigned int max) {
    size_t n=strlen(name);
    for(const char *p=adt;*p;) {
        const char *colon=strchr(p,':'), *end=strchr(p,',');
        if(!end) end=p+strlen(p);
        assert(colon && colon<end && !((end-colon-1)&1));
        if((size_t)(colon-p)==n && !strncmp(p,name,n)) {
            size_t len=(size_t)(end-colon-1)/2;
            if(len>max) return false;
            for(size_t i=0;i<len;i++) {
                unsigned b; assert(sscanf(colon+1+2*i,"%2x",&b)==1); ((uint8_t *)out)[i]=(uint8_t)b;
            }
            adt_reads++;
            return true;
        }
        p=*end?end+1:end;
    }
    return false;
}
static unsigned starts, start_limit;
static void fw_thread(void *arg, wait_result_t r) { (void)arg; (void)r; }
static int kernel_thread_start(void (*fn)(void *, wait_result_t), void *arg, thread_t *t) {
    (void)fn; *t=NULL; starts++; start_limit=(unsigned)(uintptr_t)arg; return KERN_SUCCESS;
}
static void thread_deallocate(thread_t t) { (void)t; }
'''

GATE_MAIN = r'''
int main(int argc, char **argv) {
    assert(argc==3);
    bootargs=argv[1]; adt=argv[2];
    fuxi_wlan_init();
    printf("RESULT starts=%u limit=%u adt=%u\n", starts, start_limit, adt_reads);
#if defined(FUXI_SALAMI)
    printf("NAMES bdf=%s regdb=%s m3=%s amss=%s\n", wl14_bdf_name, wl14_regdb_name, wl14_m3_name, wl_amss_path);
#else
    printf("NAMES bdf=%s regdb=%s m3=%s amss=-\n", wl14_bdf_name, WL14_REGDB_NAME, WL14_M3_NAME);
#endif
    fputs(logs, stdout);
    return 0;
}
'''


def build(tmp, name, source, defs):
    c, exe = Path(tmp) / f'{name}.c', Path(tmp) / name
    c.write_text(source)
    subprocess.run(CC + defs + [str(c), '-o', str(exe)], check=True)
    return exe


def rpmh_suite(tmp):
    transport = no_asm(between(DRIVER, '#define RH_STRIDE', '#define FUXI_WLAN_XNU'))
    guard = between(DRIVER, '/* ---- K-W1 salami RPMh policy ----', '/* ---- end K-W1 salami RPMh policy ---- */')
    rails = between(DRIVER, '/* ---- opt-in ON-only', '/* ---- end opt-in ON-only rail re-votes ---- */')
    cycle = between(DRIVER, '/* Explicit cyclerails only', 'struct wl_seq_config')
    source = (RPMH_PRE + transport +
              'static bool wl_m6o_parf(void) { return false; }\nstatic int wl_m6o_host(unsigned s) { (void)s; return 0; }\n'
              '#define FUXI_WLAN_XNU 1\n' + no_asm(SHIM) + guard + rails + cycle +
              '#define SALAMI(i) ((uint32_t)wl_salami_rail[i])\n' + RPMH_TESTS)
    exe = build(tmp, 'rpmh', source, ['-DFUXI=1', '-DFUXI_SALAMI=1'])
    out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout
    print(out.strip())


def gate_suite(tmp, mk):
    knobs = between(DRIVER, '/* ---- K-W1b salami knob source ----', '/* ---- end K-W1b salami knob source ---- */')
    region = between(DRIVER, '/* ---- milestone 5:', 'static void\nwl_sleep(')
    init = between(DRIVER, 'void\nfuxi_wlan_init(void)\n{', '\n#endif /* FUXI */') + '\n'
    source = GATE_PRE + '#if defined(FUXI_SALAMI)\n' + knobs + '\n#endif\n' + region + init + GATE_MAIN
    salami = build(tmp, 'gate-salami', source, ['-DFUXI=1', '-DFUXI_SALAMI=1'])
    fuxi = build(tmp, 'gate-fuxi', source, ['-DFUXI=1'])

    def enc(text):
        """The /defaults properties mkadt-salami.py --wlan-defaults would write, for the fake ADT."""
        return ','.join(f'{k}:{v.hex()}' for k, v in mk.wlan_defaults([text])) if text else ''

    def run(exe, args, adt=''):
        out = subprocess.run([str(exe), args, adt], check=True, capture_output=True, text=True).stdout
        m = re.search(r'RESULT starts=(\d+) limit=(\d+) adt=(\d+)', out)
        names = dict(kv.split('=', 1) for kv in re.search(r'NAMES (.*)', out)[1].split())
        return int(m[1]), int(m[2]), names, out, int(m[3])

    W14 = 'fuxiwlanqrtr=4'
    SET = 'fuxiwlanclk=1 fuxiwlanprobe=civac fuxiwlanseq=cycle,parf,dbi,eq,hostirq,msi,por,prefix'
    cases = [
        # boot-args, starts, limit, required log text
        ('', 0, 0, None),
        ('fuxiwlansalami=1', 0, 0, None),
        ('fuxiwlan=0 fuxiwlansalami=1', 0, 0, None),
        ('fuxiwlan=11', 0, 0, 'refused on salami (fuxiwlan=11 needs fuxiwlansalami=1)'),
        ('fuxiwlan=11 fuxiwlansalami=0', 0, 0, 'needs fuxiwlansalami=1'),
        ('fuxiwlan=11 fuxiwlansalami=2', 0, 0, 'needs fuxiwlansalami=1'),
        ('fuxiwlan=11 fuxiwlansalami=yes', 0, 0, 'needs fuxiwlansalami=1'),
        ('fuxiwlan=11 fuxiwlansalami=1', 1, 11, 'salami K-W1 gate open (fuxiwlansalami=1, fuxiwlanrails 0)'),
        ('fuxiwlan=12 fuxiwlansalami=1', 1, 12, 'enabled (fuxiwlan=12'),
        ('fuxiwlan=16 fuxiwlansalami=1', 1, 16, None),
        ('fuxiwlan=1 fuxiwlansalami=1', 1, 28, None),
        ('fuxiwlan=99 fuxiwlansalami=1', 1, 28, None),
        # K-W1b: fuxiwlanrails 0..2 now start on salami; any other value is an invalid selector.
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=1', 1, 11, 'gate open (fuxiwlansalami=1, fuxiwlanrails 1)'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=2', 1, 11, 'gate open (fuxiwlansalami=1, fuxiwlanrails 2)'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=0', 1, 11, 'gate open (fuxiwlansalami=1, fuxiwlanrails 0)'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanrails=1 fuxiwlanseq=cycle', 1, 28, 'fuxiwlanrails 1)'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=3', 0, 0, 'invalid selector 1, cyclerails 0'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=-1', 0, 0, 'invalid selector 1, cyclerails 0'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=1 fuxiwlanseq=cyclerails', 0, 0, 'invalid selector 1, cyclerails 1'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanseq=cyclerails', 0, 0, 'invalid selector 0, cyclerails 1'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanseq=cycle,parf,cyclerails', 0, 0, 'cyclerails 1'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanseq=cycle,bogus', 0, 0, 'invalid selector 1'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanprobe=bogus', 0, 0, 'invalid selector 1'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanprobe=civac fuxiwlanseq=cycle', 1, 11, None),
        (f'fuxiwlan=1 fuxiwlansalami=1 {SET} {W14}', 1, 28, 'salami K-W1 gate open'),
        ('fuxiwlan=1 fuxiwlansalami=1 fuxiwlanw16=9 fuxiwlanqrtr=5', 0, 0, 'invalid selector 1'),
        # W20 (C1): qrtr 6 needs all 28 steps; keep/stop22 with qrtr 6, w20 without qrtr 6/7 and, with
        # qrtr 6, every w20 bit but 0x01/0x04/0x40 are refused (the C2 bits too).
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6', 1, 28, 'W20 selected qrtr=6 (CE/HTC/WMI init to READY) w20 0 valid 1'),
        ('fuxiwlan=1 fuxiwlansalami=1 fuxiwlanqrtr=6', 1, 28, 'W12 selected qrtr=6 target memory cap'),
        ('fuxiwlan=99 fuxiwlansalami=1 fuxiwlanqrtr=6', 1, 28, 'W15 variant 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x45', 1, 28, 'w20 0x45 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=1', 1, 28, 'w20 0x1 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw15=1 fuxiwlanw16=3', 1, 28, 'W16 variant 3'),
        ('fuxiwlan=27 fuxiwlansalami=1 fuxiwlanqrtr=6', 0, 0, 'needs all 28 steps (fuxiwlan=1 or 28), limit 27; refused'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanqrtr=6', 0, 0, 'invalid selector 1'),
        # W23 (C2): qrtr 7 = W20..W22 + the passive scan, the same 28-step gate; 8 (C3) and beyond refused.
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7', 1, 28,
         'W20 selected qrtr=7 (CE/HTC/WMI init, then W23 passive scan) w20 0 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7', 1, 28, 'W23 selected passive scan of 38 channels, dwell 150 ms, '
         'nothing transmitted; BSSIDs OUI only, pdev 0, htt version not asked'),
        ('fuxiwlan=1 fuxiwlansalami=1 fuxiwlanqrtr=7', 1, 28, 'W12 selected qrtr=7 target memory cap'),
        ('fuxiwlan=27 fuxiwlansalami=1 fuxiwlanqrtr=7', 0, 0, 'W20 fuxiwlanqrtr=7 needs all 28 steps (fuxiwlan=1 or 28), limit 27; refused'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanqrtr=7', 0, 0, 'invalid selector 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x92', 1, 28,
         'W23 selected passive scan of 13 channels (2.4 GHz only), dwell 150 ms, nothing transmitted; BSSIDs OUI only, '
         'pdev from EXT caps, htt version asked'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0xd6', 1, 28, 'w20 0xd6 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0xd5', 1, 28, 'w20 0xd5 valid 1'),
        # 0x20 FULLBSSID (whole BSSIDs in the log) is refused at every qrtr: the owner masks every BSSID.
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x20', 0, 0, 'w20 0x20 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0xb2', 0, 0, 'w20 0xb2 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0xf6', 0, 0, 'w20 0xf6 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=8', 0, 0, 'w20 0x8 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0xfe', 0, 0, 'w20 0xfe valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=3', 0, 0, 'w20 0x3 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x100', 0, 0, 'w20 0x100 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=-1', 0, 0, 'w20 0xffffffff valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlankeep=1', 0, 0, 'w20 0 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanstop22=1', 0, 0, 'w20 0 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=8', 0, 0, 'W20 fuxiwlanqrtr=8 is not 0..7 in this build; refused'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=9', 0, 0, 'W20 fuxiwlanqrtr=9 is not 0..7'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=-1', 0, 0, 'W20 fuxiwlanqrtr=-1 is not 0..7'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlankeep=1', 0, 0, 'w20 0 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanstop22=1', 0, 0, 'w20 0 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=8', 0, 0, 'w20 0x8 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=2', 0, 0, 'w20 0x2 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x10', 0, 0, 'w20 0x10 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x20', 0, 0, 'w20 0x20 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x80', 0, 0, 'w20 0x80 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x100', 0, 0, 'w20 0x100 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=-1', 0, 0, 'w20 0xffffffff valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=5 fuxiwlanw20=1', 0, 0, 'W20 fuxiwlanw20 0x1 needs fuxiwlanqrtr=6; refused'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanw20=4', 0, 0, 'needs fuxiwlanqrtr=6'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=5 fuxiwlanw20=0', 1, 28, None),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=5', 1, 28, 'W12 selected qrtr=5 target memory cap'),
        # C2c (W26): 0x100 RXRING (needs 0x02), 0x200 IDLELINK and 0x400 CHAINMASK with qrtr 7, alone or together;
        # the kit values in decimal (512, 1026, 770, 1794, 258, 1282, 528, 786, 1810) and hex.
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=512', 1, 28,
         'W26 selected: rx rings no; wbm idle list 32767 link descriptors (w20 0x200); chain mask no; nothing selected to host rings'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=512', 1, 28, 'htt version not asked'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x201', 1, 28, 'w20 0x201 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x202', 1, 28, 'w20 0x202 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x400', 1, 28,
         'W26 selected: rx rings no; wbm idle list no; chain mask pdev tx/rx (w20 0x400); nothing selected to host rings'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=1026', 1, 28, 'w20 0x402 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=258', 1, 28,
         'W26 selected: rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list no; chain mask no; nothing '
         'selected to host rings'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=770', 1, 28, 'w20 0x302 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=1282', 1, 28, 'w20 0x502 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x602', 1, 28, 'w20 0x602 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=1794', 1, 28,
         'W26 selected: rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list 32767 link descriptors (w20 '
         '0x200); chain mask pdev tx/rx (w20 0x400); nothing selected to host rings'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x702', 1, 28, 'w20 0x702 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=528', 1, 28, 'w20 0x210 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=786', 1, 28, 'W23 selected passive scan of 13 channels'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=1810', 1, 28, 'w20 0x712 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x782', 1, 28, 'w20 0x782 valid 1'),
        # refused: 0x100 without 0x02 (also with 0x01, and 0x01 with 0x02), the W26 bits with qrtr 6, keep/stop22 with
        # them, 0x08/0x20 and an unknown bit next to them
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x300', 0, 0, 'w20 0x300 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x101', 0, 0, 'w20 0x101 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x301', 0, 0, 'w20 0x301 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x103', 0, 0, 'w20 0x103 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x500', 0, 0, 'w20 0x500 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x208', 0, 0, 'w20 0x208 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x420', 0, 0, 'w20 0x420 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x2f02', 0, 0, 'w20 0x2f02 valid 0'),
        # C2d (W27): 0x800 RXSEL and 0x1000 REORX with qrtr 7, each needing 0x100; the kit values 6402 (0x1902) and 6914
        # (0x1b02) in decimal; refused without 0x100, with qrtr 6, with 0x08 or 0x20 next to them
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=6402', 1, 28,
         'W27 selected: ring selection mac0/mac1 2048 B, mpdu-start/packet/msdu-end tlvs, data filters (w20 0x800); reo rx '
         'path rings 131/0/1-8, buffers back to ring 160 (w20 0x1000); nothing transmitted'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=6914', 1, 28, 'w20 0x1b02 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x902', 1, 28, 'reo rx path no; nothing transmitted'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x1102', 1, 28, 'W27 selected: ring selection no; reo rx path rings'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x1f02', 1, 28, 'w20 0x1f02 valid 1'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x802', 0, 0, 'w20 0x802 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x1002', 0, 0, 'w20 0x1002 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x1a02', 0, 0, 'w20 0x1a02 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x1900', 0, 0, 'w20 0x1900 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x190a', 0, 0, 'w20 0x190a valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x1922', 0, 0, 'w20 0x1922 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x1902 fuxiwlankeep=1', 0, 0, 'w20 0x1902 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x1902', 0, 0, 'w20 0x1902 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x800', 0, 0, 'w20 0x800 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=1794 fuxiwlankeep=1', 0, 0, 'w20 0x702 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=512 fuxiwlanstop22=1', 0, 0, 'w20 0x200 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x200', 0, 0, 'w20 0x200 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x400', 0, 0, 'w20 0x400 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x702', 0, 0, 'w20 0x702 valid 0'),
        ('fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=5 fuxiwlanw20=512', 0, 0, 'W20 fuxiwlanw20 0x200 needs fuxiwlanqrtr=6; refused'),
        ('fuxiwlan=27 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=512', 0, 0, 'needs all 28 steps'),
    ]
    for args, starts, limit, text in cases:
        s, lim, names, out, reads = run(salami, args)
        assert (s, lim) == (starts, limit), (args, s, lim, out)
        assert reads == 0 and 'from ADT /defaults' not in out, (args, out)   # empty /defaults: never read
        if text:
            assert text in out, (args, out)
        if not args.startswith('fuxiwlan=') or 'fuxiwlan=0' in args:
            assert 'FUXI WLAN' not in out, (args, out)  # absent args: nothing at all
        if not starts and 'needs fuxiwlansalami' in (text or ''):
            assert 'M6g IOVA' not in out  # refused before any configuration
        if 'fuxiwlanrails' not in args:
            assert 'rails variant' not in out, (args, out)
        if not re.search(r'fuxiwlanqrtr=([6-9]|-)|fuxiwlanw20=[^0]', args):
            assert 'W20' not in out, (args, out)  # every other boot logs exactly as before W20
        m20 = re.search(r'fuxiwlanw20=(\S+)', args)  # C2c: a W26 line only for a valid qrtr 7 boot with a W26 bit
        w26 = bool(m20 and 'fuxiwlanqrtr=7' in args and int(m20[1], 0) & 0x700 and re.search(r'w20 0x[0-9a-f]+ valid 1', out))
        assert ('W26' in out) == w26, (args, out)
        w27 = bool(w26 and int(m20[1], 0) & 0x1800)  # C2d: a W27 line only for a valid qrtr 7 boot with a W27 bit
        assert ('W27' in out) == w27, (args, out)
    # Firmware names: salami Android defaults, boot-arg overrides, bad names refused.
    s, _, names, out, _ = run(salami, f'fuxiwlan=1 fuxiwlansalami=1 {W14}')
    assert s == 1 and names == {'bdf': 'bdwlan.b0e', 'regdb': 'regdb.bin', 'm3': 'phy_ucode20.elf', 'amss': ''}, names
    assert 'salami fuxiwlanregdb regdb.bin (salami default)' in out and 'W14 selected BDF bdwlan.b0e' in out
    s, _, names, out, _ = run(salami, f'fuxiwlan=1 fuxiwlansalami=1 {W14} fuxiwlanbdf=bdwlan.elf '
                                      'fuxiwlanregdb=regdb_xiaomi.bin fuxiwlanm3=m3.bin fuxiwlanamss=amss20-c7.bin')
    assert s == 1 and names == {'bdf': 'bdwlan.elf', 'regdb': 'regdb_xiaomi.bin', 'm3': 'm3.bin',
                                'amss': '/usr/local/share/fuxi/wlan/amss20-c7.bin'}, names
    s, _, names, _, _ = run(salami, 'fuxiwlan=1 fuxiwlansalami=1 fuxiwlanamss=amss.bin')
    assert s == 1 and names['amss'] == '/usr/local/share/fuxi/wlan/amss.bin' and names['regdb'] == ''
    long = 'a' * 64
    for bad in ('fuxiwlanregdb=../regdb.bin', 'fuxiwlanregdb=', 'fuxiwlanm3=a..b', f'fuxiwlanm3={long}',
                'fuxiwlanbdf=/odm/bdwlan.b0e', 'fuxiwlanamss=x/amss20.bin', f'fuxiwlanamss={long}',
                'fuxiwlanamss=.amss', 'fuxiwlanamss=amss20.bin fuxiwlanfw=stock'):
        s, _, _, out, _ = run(salami, f'fuxiwlan=1 fuxiwlansalami=1 {W14} {bad}')
        assert s == 0 and 'refused on salami before any access (invalid selector 1' in out, (bad, out)
    s, _, names, _, _ = run(salami, f'fuxiwlan=1 fuxiwlansalami=1 {W14} fuxiwlanm3={"b" * 63}')
    assert s == 1 and names['m3'] == 'b' * 63

    # K-W1b /defaults fallback. (a) The same knobs from ADT /defaults alone (mkadt encoding) give the same
    # result and the same log as from the boot-args, less one "from ADT /defaults" line per read.
    def strip_adt(out):
        return '\n'.join(l for l in out.splitlines() if 'from ADT /defaults' not in l and not l.startswith('RESULT '))
    # read outside the gate's code region (M6i/M6h, after the thread starts): never read here
    later = {'fuxiwlanclk', 'fuxiwlaniboff', 'fuxiwlanicc', 'fuxiwlaniccbw'}
    equal = [
        'fuxiwlan=11 fuxiwlansalami=1',
        'fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=1',
        'fuxiwlan=1 fuxiwlansalami=1 fuxiwlanrails=2 fuxiwlanqrtr=5 fuxiwlanmlo=0x2d fuxiwlanw15=1 fuxiwlanw16=3 '
        'fuxiwlanstop22=1 fuxiwlankeep=1 fuxiwlanoldiova=1 fuxiwlanaspm=1',
        f'fuxiwlan=1 fuxiwlansalami=1 {W14} fuxiwlanbdf=bdwlan.elf fuxiwlanregdb=regdb_xiaomi.bin fuxiwlanm3=m3.bin '
        'fuxiwlanamss=amss20-c7.bin fuxiwlanpre=warm fuxiwlanbhi=full fuxiwlanvec=early fuxiwlancnss=pdc '
        'fuxiwlanprobe=civac fuxiwlanseq=cycle,parf,dbi',
        f'fuxiwlan=1 fuxiwlansalami=1 {SET} {W14}',
        f'fuxiwlan=1 fuxiwlansalami=1 {W14} fuxiwlanm3={"b" * 63}',
        'fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=1 fuxiwlanseq=cyclerails',
        'fuxiwlan=11 fuxiwlansalami=1 fuxiwlanseq=cyclerails',
        'fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=3',
        'fuxiwlan=11 fuxiwlansalami=2',
        'fuxiwlan=11',
        'fuxiwlan=0 fuxiwlansalami=1',
        'fuxiwlan=1 fuxiwlansalami=1 fuxiwlanamss=amss20.bin fuxiwlanfw=stock',
        'fuxiwlan=1 fuxiwlansalami=1 fuxiwlanw16=9 fuxiwlanqrtr=5',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=0x45 fuxiwlanmlo=0x17d',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=0x92',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=8',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=5 fuxiwlanw20=4',
        # C2c: every kit value (3- and 4-digit w20) and two refusals
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=512',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=1026',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=770',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=1794',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=258',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=1282',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=528',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=786',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=1810',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=256',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=6 fuxiwlanw20=1794',
        # C2d: the two kit values and a refusal
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=6402',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=6914',
        'fuxiwlan=28 fuxiwlansalami=1 fuxiwlanqrtr=7 fuxiwlanw20=2050',
    ]
    for args in equal:
        want = run(salami, args)
        got = run(salami, '', enc(args))
        assert got[:3] == want[:3] and strip_adt(got[3]) == strip_adt(want[3]), (args, want[3], got[3])
        assert want[4] == 0 and got[4] == got[3].count('from ADT /defaults') and got[4] >= 1, (args, got[3])
        assert 'salami fuxiwlan from ADT /defaults' in got[3], (args, got[3])
        if want[0]:  # a started run read every knob it was given (from /defaults here)
            for key in {t.split('=', 1)[0] for t in args.split()} - later:
                assert f'salami {key} from ADT /defaults' in got[3], (args, key, got[3])
        # (b) Half on the cmdline, half in /defaults: the same again.
        toks = args.split()
        mixed = run(salami, ' '.join(toks[:2]), enc(' '.join(toks[2:])))
        assert mixed[:3] == want[:3] and strip_adt(mixed[3]) == strip_adt(want[3]), (args, mixed[3])
    # C2c kit route (build_c2c_kits.sh): r1c2's cmdline (fuxiwlan=28 fuxiwlanclk=1 fuxiwlanqrtr=7, no w20) with the S4
    # set and a decimal fuxiwlanw20 in /defaults: the knob is read from /defaults and gives the intended W26 selection.
    s4 = 'fuxiwlansalami=1 fuxiwlanprobe=civac fuxiwlanseq=cycle,parf,dbi,eq,hostirq,msi,por,prefix'
    kits = {512: (0x200, 'rx rings no; wbm idle list 32767 link descriptors (w20 0x200); chain mask no'),
            1026: (0x402, 'rx rings no; wbm idle list no; chain mask pdev tx/rx (w20 0x400)'),
            770: (0x302, 'rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list 32767 link descriptors (w20 '
                  '0x200); chain mask no'),
            1794: (0x702, 'rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list 32767 link descriptors (w20 '
                   '0x200); chain mask pdev tx/rx (w20 0x400)'),
            258: (0x102, 'rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list no; chain mask no'),
            1282: (0x502, 'rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list no; chain mask pdev tx/rx'),
            528: (0x210, 'wbm idle list 32767 link descriptors (w20 0x200); chain mask no'),
            786: (0x312, 'wbm idle list 32767 link descriptors (w20 0x200); chain mask no'),
            1810: (0x712, 'chain mask pdev tx/rx (w20 0x400)'),
            6402: (0x1902, 'rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list no; chain mask no'),
            6914: (0x1b02, 'rx rings 4095 buffers, rings 160/161/169/186 (w20 0x100); wbm idle list 32767 link descriptors (w20 '
                   '0x200); chain mask no')}
    for dec, (hexv, sel) in kits.items():
        s, lim, _, out, reads = run(salami, 'fuxiwlan=28 fuxiwlanclk=1 fuxiwlanqrtr=7', enc(f'{s4} fuxiwlanw20={dec}'))
        assert (s, lim) == (1, 28) and 'salami fuxiwlanw20 from ADT /defaults' in out and f'w20 {hexv:#x} valid 1' in out, (dec, out)
        assert 'FUXI WLAN: W26 selected: ' in out and sel in out and 'nothing selected to host rings' in out, (dec, out)
        assert ('htt version asked' in out) == bool(hexv & 2) and ('13 channels' in out) == bool(hexv & 0x10), (dec, out)
        assert 'salami fuxiwlanqrtr from ADT' not in out and reads >= 4, (dec, reads, out)
        assert ('FUXI WLAN: W27 selected: ring selection mac0/mac1' in out and 'reo rx path rings 131/0/1-8' in out) == \
            bool(hexv & 0x1800), (dec, out)
    # (c) A boot-arg wins over /defaults, number or string, also when the boot-arg says less.
    s, lim, names, out, _ = run(salami, 'fuxiwlan=12 fuxiwlansalami=1 fuxiwlanrails=2 fuxiwlanqrtr=4 fuxiwlanbdf=a.bin',
                                enc('fuxiwlan=11 fuxiwlansalami=0 fuxiwlanrails=1 fuxiwlanbdf=b.bin fuxiwlanm3=c.elf'))
    assert (s, lim) == (1, 12) and 'fuxiwlanrails 2)' in out and names['bdf'] == 'a.bin' and names['m3'] == 'c.elf', out
    assert 'salami fuxiwlanm3 from ADT /defaults' in out and 'salami fuxiwlanm3 c.elf (boot-arg or ADT)' in out
    for key in ('fuxiwlan', 'fuxiwlansalami', 'fuxiwlanrails', 'fuxiwlanbdf'):
        assert f'salami {key} from ADT' not in out, (key, out)
    s, _, _, out, _ = run(salami, 'fuxiwlansalami=0', enc('fuxiwlan=11 fuxiwlansalami=1'))
    assert s == 0 and 'refused on salami (fuxiwlan=11 needs fuxiwlansalami=1)' in out, out
    s, _, _, out, _ = run(salami, 'fuxiwlanseq=off', enc('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanseq=cyclerails'))
    assert s == 1 and 'cyclerails' not in out and 'salami fuxiwlanseq from ADT' not in out, out
    s, _, _, out, _ = run(salami, 'fuxiwlanrails=0', enc('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=1'))
    assert s == 1 and 'fuxiwlanrails 0)' in out, out
    # (d) Absent from both = off: nothing is read beyond the gate, nothing is said.
    for adt in ('', enc('fuxiwlansalami=1 fuxiwlanrails=1 fuxiwlanqrtr=5'), enc('fuxiwlan=0 fuxiwlansalami=1')):
        s, _, _, out, _ = run(salami, '', adt)
        assert s == 0 and 'enabled' not in out and 'M6g IOVA' not in out, (adt, out)
    s, _, _, out, reads = run(salami, '', enc('fuxiwlansalami=1 fuxiwlanrails=1'))
    assert reads == 0 and 'FUXI WLAN' not in out, out
    # (e) What PE_get_default cannot hand over whole is absent (larger than the buffer), or caught by the
    # caller's own sentinel (exactly the buffer, no NUL); mkadt refuses both (mkadt_suite).
    gate = 'fuxiwlan:0b000000,fuxiwlansalami:01000000'
    s, _, _, out, _ = run(salami, '', gate + ',fuxiwlanseq:' + (b'cyclerails,' + b'x' * 86).hex())   # 97 bytes
    assert s == 1 and 'M6o seq mask 0' in out and 'salami fuxiwlanseq from ADT' not in out, out
    s, _, _, out, _ = run(salami, '', gate + ',fuxiwlanseq:' + (b'c' * 96).hex())   # 96 bytes, no NUL
    assert s == 0 and 'invalid selector 1' in out, out
    s, _, names, out, _ = run(salami, '', gate + ',fuxiwlanqrtr:04000000,fuxiwlanbdf:' + (b'a' * 66).hex())
    assert s == 1 and names['bdf'] == 'bdwlan.b0e', out
    s, _, _, out, _ = run(salami, '', gate + ',fuxiwlanqrtr:04000000,fuxiwlanbdf:' + (b'a' * 64 + b'\0').hex())
    assert s == 0 and 'W14 invalid fuxiwlanbdf basename' in out, out
    # Fuxi build: unchanged gate and defaults; salami selectors are unknown there, /defaults never read.
    for args, starts, limit in (('', 0, 0), ('fuxiwlan=11', 1, 11), ('fuxiwlan=1 fuxiwlanrails=1', 1, 28),
                                ('fuxiwlan=1 fuxiwlanseq=cyclerails', 1, 28)):
        for adt in ('', enc('fuxiwlan=12 fuxiwlansalami=1 fuxiwlanrails=2 fuxiwlanseq=cycle')):
            s, lim, _, out, reads = run(fuxi, args, adt)
            assert (s, lim) == (starts, limit) and 'salami' not in out and reads == 0, (args, out)
            assert out == run(fuxi, args)[3]
    s, _, names, _, _ = run(fuxi, f'fuxiwlan=1 {W14} fuxiwlanregdb=x.bin fuxiwlanamss=y.bin')
    assert s == 1 and names == {'bdf': 'bd_m3gl.elf', 'regdb': 'regdb_xiaomi.bin', 'm3': 'm3.bin', 'amss': '-'}, names
    # W20 on the Fuxi build: same parsing; an invalid selector stops there at step 17 (wl5_step 0x6c).
    s, lim, _, out, _ = run(fuxi, 'fuxiwlan=28 fuxiwlanqrtr=6 fuxiwlanw20=0x40')
    assert (s, lim) == (1, 28) and 'W20 selected qrtr=6 (CE/HTC/WMI init to READY) w20 0x40 valid 1' in out, out
    s, lim, _, out, _ = run(fuxi, 'fuxiwlan=28 fuxiwlanqrtr=7 fuxiwlanw20=0x10')
    assert (s, lim) == (1, 28) and 'W20 selected qrtr=7 (CE/HTC/WMI init, then W23 passive scan) w20 0x10 valid 1' in out, out
    s, _, _, out, _ = run(fuxi, 'fuxiwlan=28 fuxiwlanqrtr=8')
    assert s == 1 and 'W20 fuxiwlanqrtr=8 is not 0..7 in this build; refused' in out, out
    # C2c on the Fuxi build: the same gate (no ADT route there)
    s, lim, _, out, _ = run(fuxi, 'fuxiwlan=28 fuxiwlanqrtr=7 fuxiwlanw20=0x702')
    assert (s, lim) == (1, 28) and 'w20 0x702 valid 1' in out and 'W26 selected: rx rings 4095 buffers' in out, out
    for bad in ('fuxiwlan=28 fuxiwlanqrtr=6 fuxiwlanw20=0x200', 'fuxiwlan=28 fuxiwlanqrtr=7 fuxiwlanw20=0x100'):
        s, _, _, out, _ = run(fuxi, bad)
        assert s == 1 and re.search(r'w20 0x[0-9a-f]+ valid 0', out) and 'W26' not in out, (bad, out)
    print(f'K-W1 gate: {len(cases)} salami boot-arg cases, firmware names/overrides/refusals, Fuxi control passed')
    print(f'K-W1b /defaults: {len(equal)} knob sets equal from ADT alone and split cmdline/ADT; boot-arg wins; '
          'absent from both is off; oversize/unterminated properties; Fuxi ignores /defaults')
    print(f'C2c kit route: r1c2 cmdline with fuxiwlanw20 from ADT /defaults for {len(kits)} kit values (512 .. 1810), each '
          'read from /defaults and giving its W26 selection; W26 bits refused with qrtr 6, keep/stop22, 0x100 without 0x02')


def mkadt_suite(mk):
    # The table is every fuxiwlan* knob of the driver, typed as the driver reads it.
    knobs = set(re.findall(r'"(fuxiwlan[a-z0-9]*)"', DRIVER))
    assert set(mk.WLAN_KNOBS) == knobs, sorted(set(mk.WLAN_KNOBS) ^ knobs)
    ints = {}
    for name, amp, var, size in re.findall(r'PE_parse_boot_argn\("(fuxiwlan\w*)", (&?)(\w+), sizeof\((\w+)\)\)', DRIVER):
        assert var == size, name
        if amp:
            assert re.search(r'\bint\b[^;{}()]*\b' + var + r'\b', DRIVER), (name, var)
            ints[name] = var
    assert {k for k, v in mk.WLAN_KNOBS.items() if v == 'i'} == set(ints), sorted(ints)
    strings = {k: v for k, v in mk.WLAN_KNOBS.items() if v != 'i'}
    assert strings == {'fuxiwlanseq': 95, 'fuxiwlanpre': 15, 'fuxiwlanfw': 15, 'fuxiwlanbhi': 15, 'fuxiwlanvec': 15,
                       'fuxiwlancnss': 15, 'fuxiwlanprobe': 15, 'fuxiwlanbdf': 63, 'fuxiwlanregdb': 63,
                       'fuxiwlanm3': 63, 'fuxiwlanamss': 63}
    seq = between(DRIVER, 'wl_seq_configure(void)\n{', '\n}\n')
    assert 'char arg[96] = {0};' in seq and 'if (arg[sizeof(arg)-1]) { wl_variant_invalid = true; return; }' in seq
    assert 'char arg[16] = {0};\n\tif (!PE_parse_boot_argn(name, arg, sizeof(arg))) { return 0; }\n\targ[sizeof(arg) - 1] = 0;' in DRIVER
    assert 'static char wl14_bdf_name[65];' in DRIVER and 'static char wl14_regdb_name[65], wl14_m3_name[65];' in DRIVER
    # Encodings and the bytes each property adds (32-byte name + u32 size + value padded to 4).
    p = mk.wlan_defaults(['fuxiwlanrails=1 fuxiwlanmlo=0x22d', 'fuxiwlanseq=cycle,parf fuxiwlan=-1 fuxiwlanm3=' + 'm' * 63])
    assert p == [('fuxiwlanrails', b'\1\0\0\0'), ('fuxiwlanmlo', b'\x2d\2\0\0'), ('fuxiwlanseq', b'cycle,parf\0'),
                 ('fuxiwlan', b'\xff\xff\xff\xff'), ('fuxiwlanm3', b'm' * 63 + b'\0')], p
    assert [mk.adt_prop_bytes(v) for _, v in p] == [40, 40, 48, 40, 100]
    assert mk.adt_prop_bytes(b'regdb.bin\0') == 48 and mk.adt_prop_bytes(b'warm\0') == 44 and mk.adt_prop_bytes(b'x' * 95 + b'\0') == 132
    assert mk.wlan_defaults(['fuxiwlanqrtr=2147483647'])[0][1] == struct.pack('<i', 2 ** 31 - 1)
    for bad in ('', 'fuxiwlanrails', 'fuxiwlanrails=', 'fuxiwlanrail=1', 'fuxitic=2', 'FUXIWLANRAILS=1',
                'fuxiwlanrails=1 fuxiwlanrails=2', 'fuxiwlanrails=one', 'fuxiwlanrails=010', 'fuxiwlanrails=1_0',
                'fuxiwlanrails=0x', 'fuxiwlanrails=1.0', 'fuxiwlanqrtr=2147483648', 'fuxiwlanqrtr=-2147483649',
                'fuxiwlanseq=', 'fuxiwlanpre=' + 'p' * 16, 'fuxiwlanseq=' + 's' * 96, 'fuxiwlanbdf=' + 'b' * 64,
                'fuxiwlanbdf=b\xe9.bin', 'fuxiwlanseq=cycle\x01'):
        try:
            mk.wlan_defaults([bad])
        except ValueError:
            continue
        raise AssertionError(f'--wlan-defaults accepted {bad!r}')
    assert mk.wlan_defaults(['fuxiwlanpre=' + 'p' * 15, 'fuxiwlanseq=' + 's' * 95, 'fuxiwlanbdf=' + 'b' * 63])
    # /defaults gains exactly the reported bytes; a second write of a knob, or no /defaults, is refused.
    import adt
    import mkadt
    root = mkadt.node('device-tree', children=[mkadt.node('defaults', [('no-effaceable-storage', struct.pack('<I', 1))]),
                                               mkadt.node('chosen')])
    before = len(adt.serialize(root))
    added = mk.add_wlan_defaults(root, mk.wlan_defaults(['fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=1 fuxiwlanbdf=bdwlan.b0e']))
    assert added == [('fuxiwlan', 40), ('fuxiwlansalami', 40), ('fuxiwlanrails', 40), ('fuxiwlanbdf', 48)], added
    blob = adt.serialize(root)
    assert len(blob) - before == 168
    back, _ = adt.parse(blob)
    d = next(n for n in back.children if n.name == 'defaults')
    assert d.prop('fuxiwlanrails') == b'\1\0\0\0' and d.prop('fuxiwlanbdf') == b'bdwlan.b0e\0'
    for tree, props in ((root, mk.wlan_defaults(['fuxiwlanrails=2'])),
                        (mkadt.node('device-tree', children=[mkadt.node('chosen')]), mk.wlan_defaults(['fuxiwlan=1']))):
        try:
            mk.add_wlan_defaults(tree, props)
        except ValueError:
            continue
        raise AssertionError('add_wlan_defaults accepted a repeat or a tree without /defaults')
    print(f'mkadt --wlan-defaults: {len(mk.WLAN_KNOBS)} knobs = the driver\'s, typed as read; encodings, refusals '
          'and per-property bytes (number 40, string 36 + len+1 rounded to 4) checked')


def main():
    check_patch()
    mk = load_mkadt()
    mkadt_suite(mk)
    with tempfile.TemporaryDirectory(prefix='fuxi-wlan-salami-') as tmp:
        rpmh_suite(tmp)
        gate_suite(tmp, mk)
    print('K-W1/K-W1b salami WLAN host tests passed')


if __name__ == '__main__':
    main()
