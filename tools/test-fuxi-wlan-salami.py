#!/usr/bin/env python3
"""K-W1 (salami, OnePlus 11): run the production WLAN C under -DFUXI_SALAMI=1.

Host-only, synthetic cmd-db and RSC registers, fake boot-args. No device,
firmware, kernel build or network. Checks:
  - patch 0099 embeds the exact fuxi_wlan.c and wlan.c;
  - step 1 expects the salami cmd-db (ldob15 0x40b00), refuses 0x40a00
    (salami ldob12) with reason 5 and any other mismatch with reason 2;
  - steps 2 and 4 send exactly the salami rails ON at the salami live-DT mV;
  - rh_send_request refuses every OFF/mode/headroom vote, 0x40a00, other
    VRMs and a VRM address in a BCM batch before any TCS write, also when
    the legacy rails/cyclerails code is reached;
  - fuxi_wlan_init starts the thread only with fuxiwlan>0 AND fuxiwlansalami=1,
    refuses fuxiwlanrails, cyclerails and invalid selectors before it, and
    selects the firmware basenames (salami Android defaults, boot-arg overrides);
  - the Fuxi build (no FUXI_SALAMI) keeps its defaults and starts as before.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
DRIVER = (ROOT / 'xnu/fuxi-wlan/fuxi_wlan.c').read_text()
SHIM = (ROOT / 'shim/fuxi-probe/wlan.c').read_text()
CC = [os.environ.get('CC', 'cc'), '-std=c11', '-D_DEFAULT_SOURCE', '-g',
      '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
SALAMI_RAILS = [0x40b00, 0x40200, 0x40400, 0x40500, 0x44000, 0x40600, 0x43900, 0x43a00]


def between(text, start, end, keep_start=True):
    body = text.split(start, 1)[1].split(end, 1)[0]
    return (start + body) if keep_start else body


def no_asm(text):
    return re.sub(r'__asm__ volatile\([^;]+;', '/* host: no barrier */;', text)


def check_patch():
    patch = (ROOT / 'xnu/patches/0099-fuxi-wlan.patch').read_text()
    for dest, source in (('fuxi_wlan.c', DRIVER), ('wlan.c', SHIM)):
        added = patch.split(f'+++ b/bsd/dev/{dest}\n', 1)[1].split('\n--- a/', 1)[0]
        count = re.match(r'@@ -0,0 \+1,(\d+) @@\n', added)
        assert count and int(count[1]) == len(source.splitlines()), dest
        body = '\n'.join(l[1:] for l in added.splitlines() if l.startswith('+')) + '\n'
        assert body == source, f'0099 {dest} differs from its canonical source'
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
    print('0099 embeds both sources; Fuxi literals and salami guards are where expected')


RPMH_PRE = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
static struct sent sent[64];
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
            assert(nsent<64);
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
static unsigned refused(uint32_t addr, uint32_t data) {
    unsigned w=tcs_writes, n=nsent; size_t l=loglen;
    unsigned e=rh_send(0,addr,data);
    assert(tcs_writes==w && nsent==n);  /* nothing reached the TCS */
    assert(strstr(logs+l,"salami refused RPMh regulator"));
    return e;
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
    /* Every forbidden regulator command is refused before any TCS write. */
    for(unsigned i=0;i<8;i++) {
        assert(refused(wl_salami_rail[i]+4,0));          /* OFF */
        assert(refused(wl_salami_rail[i]+4,2));          /* enable value other than 1 */
        assert(refused(wl_salami_rail[i]+8,7));          /* mode HPM */
        assert(refused(wl_salami_rail[i]+8,1));          /* any mode value */
        assert(refused(wl_salami_rail[i]+12,0));         /* headroom */
        assert(refused(wl_salami_rail[i],1));            /* wrong mV */
    }
    assert(refused(0x40a00,1800) && refused(0x40a04,1) && refused(0x40a04,0) && refused(0x40a08,7));
    assert(refused(0x40200,952));                         /* the Fuxi S4E value */
    assert(refused(0x40800,1800) && refused(0x40804,1));  /* ldob4 (touch): not a WLAN rail */
    assert(refused(0x46808,1) && refused(0x45000,1) && refused(0x40b01,1800));
    /* BCM batches: a VRM address anywhere in the batch refuses the whole batch. */
    {
        struct wl_rh_cmd b[2]={{0x50010,0x60004001,0},{0x40b04,0,1}};
        unsigned w=tcs_writes,n=nsent;
        assert(rh_send_batch(0,b,2)==1 && tcs_writes==w && nsent==n);
        assert(strstr(logs,"salami refused RPMh batch addr 0x40b04"));
        b[1]=(struct wl_rh_cmd){0x50020,0x40000001,1};
        assert(rh_send_batch(0,b,2)==0 && nsent==n+2 && sent[n].addr==0x50010 && sent[n+1].addr==0x50020);
    }
    /* The legacy rails code, if it were ever reached on salami: nothing is sent. */
    unsigned n0=nsent, w0=tcs_writes;
    wl_rails_variant=1; assert(wl_rails_on(1,"test")==1);         /* table compare: 0x40b00 != 0x40a00 */
    wl_rails_variant=0; wl_seq_flags=WL_O_CYCLERAILS;
    assert(wl_m6o_rails(false)==1 && wl_m6o_rails(true)==1);
    assert(nsent==n0 && tcs_writes==w0);
    /* Even with a Fuxi table in r[20..], the guard stops the first vote. */
    static const uint32_t fuxi_r[8]={0x40a00,0x40200,0x40400,0x40500,0x44000,0x40600,0x43900,0x43a00};
    memcpy(wl3_report+20,fuxi_r,sizeof(fuxi_r));
    size_t l=loglen;
    assert(wl_m6o_rails(false)==1 && strstr(logs+l,"salami refused RPMh regulator addr 0x40504 data 0"));
    wl_seq_flags=0; wl_rails_variant=1; l=loglen;
    assert(wl_rails_on(1,"test")==1 && strstr(logs+l,"salami refused RPMh regulator addr 0x40a00 data 0x708"));
    wl_rails_variant=2; l=loglen;
    assert(wl_rails_on(2,"test")==1 && strstr(logs+l,"salami refused RPMh regulator addr 0x40a00"));
    assert(nsent==n0 && tcs_writes==w0);
    puts("K-W1 RPMh: salami step 1/2/4 votes exact; OFF/mode/headroom/0x40a00/other VRMs refused before any TCS write");
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
static const char *bootargs = "";
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
static unsigned starts, start_limit;
static void fw_thread(void *arg, wait_result_t r) { (void)arg; (void)r; }
static int kernel_thread_start(void (*fn)(void *, wait_result_t), void *arg, thread_t *t) {
    (void)fn; *t=NULL; starts++; start_limit=(unsigned)(uintptr_t)arg; return KERN_SUCCESS;
}
static void thread_deallocate(thread_t t) { (void)t; }
'''

GATE_MAIN = r'''
int main(int argc, char **argv) {
    assert(argc==2);
    bootargs=argv[1];
    fuxi_wlan_init();
    printf("RESULT starts=%u limit=%u\n", starts, start_limit);
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
              '#define FUXI_WLAN_XNU 1\n' + no_asm(SHIM) + guard + rails + cycle + RPMH_TESTS)
    exe = build(tmp, 'rpmh', source, ['-DFUXI=1', '-DFUXI_SALAMI=1'])
    out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout
    print(out.strip())


def gate_suite(tmp):
    region = between(DRIVER, '/* ---- milestone 5:', 'static void\nwl_sleep(')
    init = between(DRIVER, 'void\nfuxi_wlan_init(void)\n{', '\n#endif /* FUXI */') + '\n'
    source = GATE_PRE + region + init + GATE_MAIN
    salami = build(tmp, 'gate-salami', source, ['-DFUXI=1', '-DFUXI_SALAMI=1'])
    fuxi = build(tmp, 'gate-fuxi', source, ['-DFUXI=1'])

    def run(exe, args):
        out = subprocess.run([str(exe), args], check=True, capture_output=True, text=True).stdout
        m = re.search(r'RESULT starts=(\d+) limit=(\d+)', out)
        names = dict(kv.split('=', 1) for kv in re.search(r'NAMES (.*)', out)[1].split())
        return int(m[1]), int(m[2]), names, out

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
        ('fuxiwlan=11 fuxiwlansalami=1', 1, 11, 'salami K-W1 gate open'),
        ('fuxiwlan=12 fuxiwlansalami=1', 1, 12, 'enabled (fuxiwlan=12'),
        ('fuxiwlan=16 fuxiwlansalami=1', 1, 16, None),
        ('fuxiwlan=1 fuxiwlansalami=1', 1, 28, None),
        ('fuxiwlan=99 fuxiwlansalami=1', 1, 28, None),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=1', 0, 0, 'fuxiwlanrails 1, cyclerails 0'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=2', 0, 0, 'refused on salami before any access'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanrails=0', 0, 0, 'refused on salami before any access'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanseq=cyclerails', 0, 0, 'cyclerails 1'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanseq=cycle,parf,cyclerails', 0, 0, 'cyclerails 1'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanseq=cycle,bogus', 0, 0, 'invalid selector 1'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanprobe=bogus', 0, 0, 'invalid selector 1'),
        ('fuxiwlan=11 fuxiwlansalami=1 fuxiwlanprobe=civac fuxiwlanseq=cycle', 1, 11, None),
        (f'fuxiwlan=1 fuxiwlansalami=1 {SET} {W14}', 1, 28, 'salami K-W1 gate open'),
        ('fuxiwlan=1 fuxiwlansalami=1 fuxiwlanw16=9 fuxiwlanqrtr=5', 0, 0, 'invalid selector 1'),
    ]
    for args, starts, limit, text in cases:
        s, lim, names, out = run(salami, args)
        assert (s, lim) == (starts, limit), (args, s, lim, out)
        if text:
            assert text in out, (args, out)
        if not args.startswith('fuxiwlan=') or 'fuxiwlan=0' in args:
            assert 'FUXI WLAN' not in out, (args, out)  # absent args: nothing at all
        if not starts and 'needs fuxiwlansalami' in (text or ''):
            assert 'M6g IOVA' not in out  # refused before any configuration
    # Firmware names: salami Android defaults, boot-arg overrides, bad names refused.
    s, _, names, out = run(salami, f'fuxiwlan=1 fuxiwlansalami=1 {W14}')
    assert s == 1 and names == {'bdf': 'bdwlan.b0e', 'regdb': 'regdb.bin', 'm3': 'phy_ucode20.elf', 'amss': ''}, names
    assert 'salami fuxiwlanregdb regdb.bin (salami default)' in out and 'W14 selected BDF bdwlan.b0e' in out
    s, _, names, out = run(salami, f'fuxiwlan=1 fuxiwlansalami=1 {W14} fuxiwlanbdf=bdwlan.elf '
                                   'fuxiwlanregdb=regdb_xiaomi.bin fuxiwlanm3=m3.bin fuxiwlanamss=amss20-c7.bin')
    assert s == 1 and names == {'bdf': 'bdwlan.elf', 'regdb': 'regdb_xiaomi.bin', 'm3': 'm3.bin',
                                'amss': '/usr/local/share/fuxi/wlan/amss20-c7.bin'}, names
    s, _, names, _ = run(salami, 'fuxiwlan=1 fuxiwlansalami=1 fuxiwlanamss=amss.bin')
    assert s == 1 and names['amss'] == '/usr/local/share/fuxi/wlan/amss.bin' and names['regdb'] == ''
    long = 'a' * 64
    for bad in ('fuxiwlanregdb=../regdb.bin', 'fuxiwlanregdb=', 'fuxiwlanm3=a..b', f'fuxiwlanm3={long}',
                'fuxiwlanbdf=/odm/bdwlan.b0e', 'fuxiwlanamss=x/amss20.bin', f'fuxiwlanamss={long}',
                'fuxiwlanamss=.amss', 'fuxiwlanamss=amss20.bin fuxiwlanfw=stock'):
        s, _, _, out = run(salami, f'fuxiwlan=1 fuxiwlansalami=1 {W14} {bad}')
        assert s == 0 and 'refused on salami before any access (invalid selector 1' in out, (bad, out)
    s, _, names, _ = run(salami, f'fuxiwlan=1 fuxiwlansalami=1 {W14} fuxiwlanm3={"b" * 63}')
    assert s == 1 and names['m3'] == 'b' * 63
    # Fuxi build: unchanged gate and defaults; salami selectors are unknown there.
    for args, starts, limit in (('', 0, 0), ('fuxiwlan=11', 1, 11), ('fuxiwlan=1 fuxiwlanrails=1', 1, 28),
                                ('fuxiwlan=1 fuxiwlanseq=cyclerails', 1, 28)):
        s, lim, _, out = run(fuxi, args)
        assert (s, lim) == (starts, limit) and 'salami' not in out, (args, out)
    s, _, names, _ = run(fuxi, f'fuxiwlan=1 {W14} fuxiwlanregdb=x.bin fuxiwlanamss=y.bin')
    assert s == 1 and names == {'bdf': 'bd_m3gl.elf', 'regdb': 'regdb_xiaomi.bin', 'm3': 'm3.bin', 'amss': '-'}, names
    print(f'K-W1 gate: {len(cases)} salami boot-arg cases, firmware names/overrides/refusals, Fuxi control passed')


def main():
    check_patch()
    with tempfile.TemporaryDirectory(prefix='fuxi-wlan-salami-') as tmp:
        rpmh_suite(tmp)
        gate_suite(tmp)
    print('K-W1 salami WLAN host tests passed')


if __name__ == '__main__':
    main()
