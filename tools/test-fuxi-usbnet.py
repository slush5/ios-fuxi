#!/usr/bin/env python3
"""Route U host tests (offline: host cc, no phone, no pyusb). In WSL: CC=cc python3 -B tools/test-fuxi-usbnet.py

1. Layout and switches: shim/fuxi-probe/usbnet.h, xnu/fuxi-wifi-ios/fuxi_usbnet_ring.h,
   tools/fuxi_usbnet_ring.py and gpu-handoff.h agree on every value; FUXI_USBNET defaults to 1
   only in the salami XNU loader; probe.c, loader.c, loader.ld and build.sh text (the requests
   sit under FUXI_USBNET outside the salami pre-filter and skip the setup log; the handoff
   follows the ADT copy; the region is linked and checked).
2. Loader request logic: usbnet.h compiled unchanged against a fake region at a fixed address
   and fake ep0 buffers, driven with the request sequence LoaderRingSim runs; every STALL
   and drop decision, every reply and the region bytes must agree, and the scenario asserts
   the spec with AMENDMENT A1 (only a malformed request STALLs): before launch and before
   the handoff (0x5d dropped, 0x5e all zero), no kmagic, kstate bit0 clear, bad wValue,
   wLength 13/14/1514/1515 and 191/192/4096/4097, RX full (dropped, out_stall_full),
   residue mismatch, every counter across 2^32, wIndex across 2^16, host_tx_tail rebuild
   edges, TX list truncation at exact fit, corrupted TX slots, region refusals, and a host
   that keeps fewer than 32 frames in flight never sees a STALL or a drop.
3. probe.c's own glue: ud_event and the route U branches of ud_setup (pasted from the file)
   on fake DWC3 registers: no STALL before the launch, before the handoff, without a kernel
   or with a full ring; STALLs for malformed requests only; the OUT data stage length and
   residue rule, IN replies of exactly wLength (zeros when not ready), the setup log skip.
4. ADT: mkadt-salami.py add_usbnet writes the exact placeholder; the loader patch finds it and
   refuses zero or two matches and look-alikes (other length, placeholder flag, other name).
5. Kernel ring helpers: fuxi_usbnet_ring.h compiled unchanged against KernelSim (TX produce,
   full, sizes; RX consume, bad len/seq/fnv; wraparound), and its handoff and header checks.
6. Kernel backend: fuxi_wifi_ios.c on the test-fuxi-wifi-ios.py model with usbnet.h as the
   loader, fuxiwifiios=2: each handoff failure falls back to the synthetic loopback with one
   line; with a region: kmagic, kstate bits, TX through 0x5e, RX through 0x5d to en0, poll
   intervals; fuxiwifiios=1 is unchanged.
7. UsbnetTransport over the simulators: both directions with no STALL and no drop on the
   phone (flow control from the header), busy without a request, zeros before the launch
   and the handoff, a phone reboot under a running bridge, resync on bad seq or fnv,
   wraparound, a restarted bridge.
8. Mutants: every STALL and drop rule and bound of usbnet.h and of the kernel ring broken
   once; parts 2 and 5 must fail for each.
"""
import importlib.util
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PROBE_DIR = ROOT / 'shim/fuxi-probe'
WIFI_DIR = ROOT / 'xnu/fuxi-wifi-ios'
PROBE = (PROBE_DIR / 'probe.c').read_text()
USBNET_H = (PROBE_DIR / 'usbnet.h').read_text()
RING_H = (WIFI_DIR / 'fuxi_usbnet_ring.h').read_text()
CC = os.environ.get('CC', 'cc')
FLAGS = ['-std=gnu11', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function', '-fno-omit-frame-pointer']
SAN = ['-fsanitize=address,undefined', '-fno-sanitize-recover=all']
BASE = 0x6f5500000000          # the fake region: 16 KiB aligned, outside every FG_* window
RESULTS = []
sys.path.insert(0, str(ROOT / 'tools'))
import fuxi_usbnet_ring as R  # noqa: E402


def check(name, ok, detail=''):
    RESULTS.append((name, bool(ok)))
    print(('PASS ' if ok else 'FAIL ') + name + ('' if ok or not detail else f' :: {str(detail)[:1500]}'), flush=True)
    return bool(ok)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REATTACH = load('test_reattach', ROOT / 'tools/test-fuxi-usb-reattach.py')
function, between = REATTACH.function, REATTACH.between


def compile_c(tmp, name, code, defines=(), includes=(PROBE_DIR,), sanitize=True, extra=()):
    src, exe = tmp / f'{name}.c', tmp / name
    src.write_text(code)
    cmd = [CC, *FLAGS, *extra, *(SAN if sanitize else []), *[f'-D{d}' for d in defines],
           *[f'-I{i}' for i in includes], str(src), '-o', str(exe)]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode and sanitize and 'sanitize' in p.stderr:
        return compile_c(tmp, name, code, defines, includes, False, extra)
    return exe if p.returncode == 0 else 'compile: ' + p.stderr


def run_exe(exe, stdin='', args=(), timeout=120):
    if isinstance(exe, str):
        return 99, exe
    p = subprocess.run([str(exe), *args], input=stdin, capture_output=True, text=True, timeout=timeout)
    return p.returncode, p.stdout + p.stderr


def frame(n, tag):
    """A deterministic Ethernet frame of n bytes (n may be below 14 for size refusals)."""
    head = bytes.fromhex('024655584901') + bytes.fromhex('02aabbccdd') + bytes([tag & 0xff]) + b'\x88\xb5'
    return (head + bytes((tag * 7 + i * 13) & 0xff for i in range(max(0, n - 14))))[:n]


def entries(raw):
    """(len, seq, fnv, frame) of a 0x5e reply's TX list."""
    out, pos = [], R.IN_MIN
    while pos + R.SLOT_HDR <= len(raw):
        n, _flags, seq, want, _zero = struct.unpack_from('<HHIII', raw, pos)
        if n == 0:
            break
        out.append((n, seq, want, raw[pos + 16:pos + 16 + n]))
        pos += 16 + ((n + 3) & ~3)
    return out


# ---- 1. layout, switches, text -----------------------------------------------------------
CONSTANTS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "usbnet.h"
#include "fuxi_usbnet_ring.h"
#define SAME(a, b) _Static_assert((a) == (b), #a " == " #b)
SAME(UN_SIZE, FXNET_SIZE); SAME(UN_ALIGN, FXNET_ALIGN); SAME(UN_SLOT_SIZE, FXNET_SLOT_SIZE); SAME(UN_SLOTS, FXNET_SLOTS);
SAME(UN_RX_OFF, FXNET_RX_OFF); SAME(UN_TX_OFF, FXNET_TX_OFF); SAME(UN_SLOT_HDR, FXNET_SLOT_HDR);
SAME(UN_FRAME_MIN, FXNET_FRAME_MIN); SAME(UN_FRAME_MAX, FXNET_FRAME_MAX); SAME(UN_VERSION, FXNET_VERSION);
SAME(UN_MAGIC_LO, FXNET_MAGIC_LO); SAME(UN_MAGIC_HI, FXNET_MAGIC_HI); SAME(UN_KMAGIC_LO, FXNET_KMAGIC_LO);
SAME(UN_KMAGIC_HI, FXNET_KMAGIC_HI); SAME(UN_L0_MAGIC, FXNET_L0_MAGIC); SAME(UN_L0_VERSION, FXNET_L0_VERSION);
SAME(UN_L0_SIZE, FXNET_L0_SIZE); SAME(UN_L0_SELF, FXNET_L0_SELF); SAME(UN_L0_SLOT_SIZE, FXNET_L0_SLOT_SIZE);
SAME(UN_L0_SLOTS, FXNET_L0_SLOTS); SAME(UN_L0_RX_OFF, FXNET_L0_RX_OFF); SAME(UN_L0_TX_OFF, FXNET_L0_TX_OFF);
SAME(UN_K_MAGIC, FXNET_K_MAGIC); SAME(UN_K_STATE, FXNET_K_STATE); SAME(UN_K_RX_TAIL, FXNET_K_RX_TAIL);
SAME(UN_K_TX_HEAD, FXNET_K_TX_HEAD); SAME(UN_K_RX_FRAMES, FXNET_K_RX_FRAMES); SAME(UN_K_RX_BAD, FXNET_K_RX_BAD);
SAME(UN_K_TX_FRAMES, FXNET_K_TX_FRAMES); SAME(UN_K_TX_FULL, FXNET_K_TX_FULL); SAME(UN_K_TX_SIZE, FXNET_K_TX_SIZE);
SAME(UN_L_RX_HEAD, FXNET_L_RX_HEAD); SAME(UN_L_HOST_TAIL, FXNET_L_HOST_TAIL);
SAME(UN_KSTATE_UP, FXNET_KSTATE_UP); SAME(UN_KSTATE_LINK, FXNET_KSTATE_LINK);
SAME(FXNET_RAM_BASE, FG_RAM_BASE); SAME(FXNET_RAM_SIZE, FG_RAM_SIZE); SAME(FXNET_GPU_SCRATCH, FG_SCRATCH);
SAME(FXNET_GPU_SCRATCH_SIZE, FG_SCRATCH_SIZE);
SAME(UN_RX_OFF + UN_SLOTS * UN_SLOT_SIZE, UN_TX_OFF); SAME(UN_TX_OFF + UN_SLOTS * UN_SLOT_SIZE, UN_SIZE);
_Static_assert(UN_SLOT_HDR + ((UN_FRAME_MAX + 3) & ~3u) <= UN_SLOT_SIZE, "a slot holds the largest frame");
_Static_assert(UN_L_TAIL_BAD < 0xc0 && UN_K_TX_SIZE < 0x80, "every field in its header line");
int main(void) {
 uint32_t m[4] = {UN_MAGIC_LO, UN_MAGIC_HI, UN_KMAGIC_LO, UN_KMAGIC_HI};
 assert(!memcmp(m, "FXNETL01FXNETK01", 16));
 printf("C %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n", UN_SIZE, UN_ALIGN, UN_SLOT_SIZE, UN_SLOTS, UN_RX_OFF,
        UN_TX_OFF, UN_SLOT_HDR, UN_FRAME_MIN, UN_FRAME_MAX, UN_IN_MIN, UN_IN_MAX, UN_VALUE_OUT, UN_VALUE_IN,
        UN_REQ_OUT, UN_REQ_IN);
 printf("F %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n", UN_L0_VERSION, UN_L0_SIZE,
        UN_L0_SLOT_SIZE, UN_L0_SLOTS, UN_L0_RX_OFF, UN_L0_TX_OFF, UN_L0_ADT, UN_K_STATE, UN_K_RX_TAIL, UN_K_TX_HEAD,
        UN_K_RX_FRAMES, UN_K_RX_BAD, UN_K_TX_FRAMES, UN_K_TX_FULL, UN_K_TX_SIZE, UN_L_RX_HEAD, UN_L_HOST_TAIL,
        UN_L_OUT_OK, UN_L_OUT_FULL, UN_L_OUT_OTHER, UN_L_IN_COUNT, UN_L_TAIL_BAD);
 printf("G %llu %llu %llu %llu\n", (unsigned long long)FG_RAM_BASE, (unsigned long long)FG_RAM_SIZE,
        (unsigned long long)FG_SCRATCH, (unsigned long long)FG_SCRATCH_SIZE);
 return 0;
}
'''


def layout_suite(tmp):
    exe = compile_c(tmp, 'constants', CONSTANTS, includes=(PROBE_DIR, WIFI_DIR), sanitize=False)
    rc, out = run_exe(exe)
    if not check('layout: usbnet.h == fuxi_usbnet_ring.h == gpu-handoff.h (compile-time asserts)', rc == 0, out):
        return
    c = [int(x) for x in re.search(r'^C (.*)$', out, re.M)[1].split()]
    f = [int(x) for x in re.search(r'^F (.*)$', out, re.M)[1].split()]
    g = [int(x) for x in re.search(r'^G (.*)$', out, re.M)[1].split()]
    check('layout: tools/fuxi_usbnet_ring.py constants == usbnet.h',
          c == [R.SIZE, R.ALIGN, R.SLOT_SIZE, R.SLOTS, R.RX_OFF, R.TX_OFF, R.SLOT_HDR, R.FRAME_MIN, R.FRAME_MAX,
                R.IN_MIN, R.IN_MAX, R.VALUE_OUT, R.VALUE_IN, R.REQ_OUT, R.REQ_IN], c)
    names = ['version', 'size', 'slot_size', 'slots', 'rx_off', 'tx_off', 'adt_status', 'kstate', 'rx_tail', 'tx_head',
             'rx_frames', 'rx_bad', 'tx_frames', 'tx_full_drops', 'tx_size_drops', 'rx_head', 'host_tx_tail', 'out_ok',
             'out_stall_full', 'out_stall_other', 'in_count', 'tail_bad']
    check('layout: Python header field offsets == usbnet.h', f == [R.FIELDS[n] for n in names] and len(R.FIELDS) == len(names), f)
    check('layout: Python RAM/scratch windows == gpu-handoff.h', g == [R.RAM_BASE, R.RAM_SIZE, R.SCRATCH, R.SCRATCH_SIZE], g)
    check('layout: header lines (spec): line 0 at 0x00, line 1 at 0x40 (kernel), line 2 at 0x80, 0x5e copies 0xc0',
          all(o < 0x40 for o in f[:7]) and all(0x40 <= o < 0x80 for o in f[7:15]) and all(0x80 <= o < 0xc0 for o in f[15:]))
    # FUXI_USBNET defaults and refusals.
    code = '#include <stdio.h>\n#include "usbnet.h"\nint main(void) { printf("%d\\n", FUXI_USBNET); return 0; }\n'
    want = {(): '0', ('FUXI_SALAMI=1',): '0', ('FUXI_XNU_LOADER=1',): '0', ('FUXI_SALAMI=1', 'FUXI_XNU_LOADER=1'): '1',
            ('FUXI_SALAMI=1', 'FUXI_XNU_LOADER=1', 'FUXI_USBNET=0'): '0'}
    got = {}
    for i, (d, _) in enumerate(want.items()):
        rc, out = run_exe(compile_c(tmp, f'default{i}', code, d, sanitize=False))
        got[d] = out.strip() if rc == 0 else f'rc {rc}'
    check('FUXI_USBNET: 1 only in the salami XNU loader (FUXI_SALAMI + FUXI_XNU_LOADER), 0 = off', got == want, got)
    for d, why in ((('FUXI_USBNET=1',), 'salami XNU loader'), (('FUXI_SALAMI=1', 'FUXI_XNU_LOADER=1', 'FUXI_USBNET=2'), 'must be 0 or 1')):
        res = compile_c(tmp, 'default-bad', code, d, sanitize=False)
        check(f'FUXI_USBNET refused at compile time: {" ".join(d)}', isinstance(res, str) and why in res, res)
    # probe.c: the requests, outside the salami pre-filter, the setup log, the data stage.
    setup = function(PROBE, 'ud_setup')
    prefilter = between(setup, '#if defined(FUXI_SALAMI)\n\t/* Lanes not ported', '#endif\n\tif (type == 0xc0 && (req == 0x40')
    check('probe.c: 0x5d/0x5e are not in the salami pre-filter STALL list',
          not re.search(r'0x5d|0x5e|UN_REQ', prefilter) and 'req == 0x5b' in prefilter, prefilter[-400:])
    check('probe.c: the setup printf skips route U after the first of each (un_log_setup)',
          'if ((req != 0x5b || value == 0) && un_log_setup(req))' in setup)
    route = between(setup, '#if FUXI_USBNET\n    } else if (type == 0x40 && req == UN_REQ_OUT) {', '#endif\n    } else if (type == 0xc0 && req == 0x48)')
    check('probe.c: 0x5d (OUT 0x40) and 0x5e (IN 0xc0) under FUXI_USBNET, next to the audio and 0x48 requests',
          'if (un_out_ok(&un, value, length)) {' in route and
          'n = un_in(&un, monitor_report[MON_LAUNCH] != 0, value, (uint16_t)(st[4] | st[5] << 8), length, data);' in route
          and 'ok = n != 0;' in route)
    event = function(PROBE, 'ud_event')
    check('probe.c: the 0x5d data stage uses the audio residue rule, then un_out_data (MON_LAUNCH re-read), under FUXI_USBNET',
          re.search(r'#if FUXI_USBNET\n\t+if \(ud\.usbnet_out\) \{.*?uint32_t want = \(\(uint32_t\)ud\.out_length \+ 63u\) & ~63u;'
                    r'.*?\(void\)un_out_data\(&un, monitor_report\[MON_LAUNCH\] != 0, want - left == ud\.out_length,',
                    event, re.S) is not None)
    check('probe.c: OUT TRB of (wLength + 63) & ~63 for route U frames, 64 otherwise',
          'chunk = chunk || ud.usbnet_out;' in setup and 'ud_start(0, UD_DATA, chunk ? ((uint32_t)length + 63u) & ~63u : 64u, TRBCTL_DATA);' in setup)
    check('probe.c: dma_area check: UD_DATA + 1536 and + 4096 (+ zero fill) stay inside the 16 KiB area',
          '_Static_assert(UD_DATA + ((UN_FRAME_MAX + 63u) & ~63u) <= UD_SIZE && UD_DATA + UN_IN_MAX + 4 <= UD_SIZE' in PROBE)
    asm = re.findall(r'__asm__[^;]*;', USBNET_H)
    check('usbnet.h: no MMIO, no system register: its only asm is dsb sy', asm == ['__asm__ volatile("dsb sy" ::: "memory");'], asm)
    loader = (ROOT / 'shim/fuxi-loader/loader.c').read_text()
    check('loader.c: probe_usbnet_handoff right after the ADT copy, under FUXI_USBNET, never monitor_error',
          re.search(r'copy64\(adt_pa, rd_start \+ p->adt_off, p->adt_size\);\n#if FUXI_USBNET\n(\t[/ ]\*[^\n]*\n)+'
                    r'\t\(void\)probe_usbnet_handoff\(adt_pa, p->adt_size\);\n#endif\n', loader) is not None)
    ld = (ROOT / 'shim/fuxi-loader/loader.ld').read_text()
    build = (ROOT / 'shim/fuxi-loader/build.sh').read_text()
    check('loader.ld/build.sh: .usbnet after .data; usbnet.S 0x24000 zero bytes, 16 KiB aligned; -un0 for FUXI_USBNET=0',
          ' .data : { *(.data*) }\n .usbnet : { *(.usbnet) }\n . = ALIGN(16384);' in ld and
          r'.balign 16384\n.global _usbnet_start\n_usbnet_start:\n.zero 0x24000' in build and
          '[[ $FUXI_USBNET == 1 ]] || usbnet=-un0' in build and 'if [[ $board == salami ]]; then usbnet_on=1; fi' in build
          and "assert image[start:end] == bytes(end - start), 'usbnet region is not zero in the Image'" in build)
    mon = (ROOT / 'tools/fuxi-xnu-monitor.py').read_text()
    check('fuxi-xnu-monitor.py header: read-only except 0x5d (route U)',
          'Memory is read-only to the host except request 0x5d (route U' in mon[:3000])


# ---- 2 and 5. the C sides against the Python models, one scenario --------------------------
LOADER_C = r'''
#define _GNU_SOURCE
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include "usbnet.h"
#define BASE 0x6f5500000000ull
static struct un un;
static bool launched;
static uint8_t ep0[0x2000] __attribute__((aligned(64)));
static uint8_t adt[0x10000] __attribute__((aligned(16)));
static char line[1 << 18];
static size_t unhex(char *s, uint8_t *out, size_t max)
{
 size_t n = 0;
 while (*s == ' ') s++;
 if (*s == '-') return 0;
 for (; s[0] && s[1] && s[0] != '\n'; s += 2) {
  unsigned v;
  assert(n < max && sscanf(s, "%2x", &v) == 1);
  out[n++] = (uint8_t)v;
 }
 return n;
}
static void hex(const volatile uint8_t *p, size_t n)
{
 for (size_t i = 0; i < n; i++) printf("%02x", p[i]);
 putchar('\n');
}
int main(void)
{
 if (mmap((void *)BASE, UN_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0) != (void *)BASE) {
  perror("mmap"); return 3;
 }
 while (fgets(line, sizeof line, stdin)) {
  char *p = line + 1;
  unsigned long long a = 0, b = 0, c = 0, d = 0;
  switch (line[0]) {
  case 'L': launched = strtoul(p, NULL, 0) != 0; break;
  case 'W': a = strtoull(p, &p, 16); b = strtoull(p, &p, 16); assert(a < UN_SIZE && !(a & 3));
   *(volatile uint32_t *)(BASE + a) = (uint32_t)b; break;
  case 'H': { size_t n = unhex(p, adt, sizeof adt); uint32_t s = un_handoff(&un, BASE, (uintptr_t)adt, n);
   printf("H %u ", s); hex(adt, n); break; }
  case 'Z': { a = strtoull(p, &p, 16); size_t n = unhex(p, adt, sizeof adt); struct un u2 = {0};
   uint32_t s = un_handoff(&u2, (uintptr_t)a, (uintptr_t)adt, n); printf("Z %u %d ", s, u2.base != 0); hex(adt, n); break; }
  case 'R': a = strtoull(p, &p, 16); printf("R %d\n", un_region_ok(a)); break;
  case 'O': { a = strtoull(p, &p, 16); b = strtoull(p, &p, 16); c = strtoull(p, &p, 16); d = strtoull(p, &p, 16);
   memset(ep0, 0xa5, sizeof ep0);
   assert(unhex(p, ep0, sizeof ep0) == d);
   if (!un_out_ok(&un, (uint16_t)a, (uint16_t)c)) { puts("O STALL"); break; }
   puts(un_out_data(&un, launched, d == c, ep0, (uint16_t)c, (uint16_t)b) ? "O OK" : "O DROP"); break; }
  case 'I': { a = strtoull(p, &p, 16); b = strtoull(p, &p, 16); c = strtoull(p, &p, 16);
   memset(ep0, 0xa5, sizeof ep0);
   uint32_t n = un_in(&un, launched, (uint16_t)a, (uint16_t)b, (uint16_t)c, ep0);
   if (!n) { puts("I STALL"); break; }
   assert(n == c && c <= sizeof ep0 - 4);
   printf("I "); hex(ep0, n); break; }
  case 'D': { char *path = strtok(p, " \n"); FILE *f = fopen(path, "wb"); assert(f);
   assert(fwrite((const void *)BASE, 1, UN_SIZE, f) == UN_SIZE); fclose(f); break; }
  default: assert(!"op");
  }
  fflush(stdout);
 }
 return 0;
}
'''

KERNEL_C = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fuxi_usbnet_ring.h"
static uint8_t region[FXNET_SIZE] __attribute__((aligned(16384)));
static uint8_t frame[2048];
static char line[1 << 16];
int main(void)
{
 const uintptr_t r = (uintptr_t)region;
 while (fgets(line, sizeof line, stdin)) {
  char *p = line + 1;
  unsigned long long a, b;
  switch (line[0]) {
  case 'W': a = strtoull(p, &p, 16); b = strtoull(p, &p, 16); assert(a < FXNET_SIZE && !(a & 3));
   *(volatile uint32_t *)(r + a) = (uint32_t)b; break;
  case 'A': fxnet_attach(r); break;
  case 'S': fxnet_kstate(r, (uint32_t)strtoul(p, NULL, 16)); break;
  case 'T': { size_t n = 0; unsigned v; while (*p == ' ') p++;
   for (; p[0] && p[1] && p[0] != '\n'; p += 2) { assert(sscanf(p, "%2x", &v) == 1 && n < sizeof frame); frame[n++] = (uint8_t)v; }
   int put = fxnet_tx_put(r, frame, n);
   puts(put == FXNET_TX_OK ? "T OK" : put == FXNET_TX_FULL ? "T FULL" : put == FXNET_TX_SIZE ? "T SIZE" : "T ?"); break; }
  case 'X': { size_t n = 0; memset(frame, 0x5a, sizeof frame);
   int got = fxnet_rx_take(r, frame, &n);
   if (got == FXNET_RX_EMPTY) { puts("X EMPTY"); break; }
   if (got == FXNET_RX_OK) { printf("X OK "); for (size_t i = 0; i < n; i++) printf("%02x", frame[i]); putchar('\n'); }
   else puts("X BAD");
   fxnet_rx_done(r, got == FXNET_RX_OK); break; }
  case 'D': { char *path = strtok(p, " \n"); FILE *f = fopen(path, "wb"); assert(f);
   assert(fwrite(region, 1, FXNET_SIZE, f) == FXNET_SIZE); fclose(f); break; }
  default: assert(!"op");
  }
  fflush(stdout);
 }
 return 0;
}
'''


class Dual:
    """One scenario on the Python models, written down as op lines for the two C programs.
    Each program also gets the other side's stores (W lines) before its next op."""

    def __init__(self):
        self.loader = R.LoaderRingSim(BASE)
        self.kernel = R.KernelSim(self.loader)
        self.lops, self.lexp, self.lview = [], [], bytearray(R.SIZE)
        self.kops, self.kexp, self.kview = [], [], bytearray(R.SIZE)
        self.dumps = []

    @property
    def region(self):
        return self.loader.region

    def u32(self, name):
        return self.region.u32(R.FIELDS[name])

    def poke(self, off, value):
        """A raw store (test setup, or a broken side); both programs get it as a W line."""
        self.region.write(off, value)

    def _sync(self, view, ops):
        cur = self.region.bytes
        for page in range(0, R.SIZE, 0x1000):
            if cur[page:page + 0x1000] != view[page:page + 0x1000]:
                for off in range(page, page + 0x1000, 4):
                    if cur[off:off + 4] != view[off:off + 4]:
                        ops.append(f'W {off:x} {struct.unpack_from("<I", cur, off)[0]:x}')
                view[page:page + 0x1000] = cur[page:page + 0x1000]

    def _op(self, side, line, run):
        view, ops, exp = (self.lview, self.lops, self.lexp) if side == 'L' else (self.kview, self.kops, self.kexp)
        self._sync(view, ops)
        out = run()
        ops.append(line)
        if out is not None:
            exp.append(out)
        view[:] = self.region.bytes
        return out

    # loader side
    def launch(self, on):
        self.loader.launched = bool(on)
        self.lops.append(f'L {int(bool(on))}')

    def handoff(self, adt):
        adt = bytearray(adt)
        line = f'H {adt.hex() or "-"}'
        return self._op('L', line, lambda: f'H {self.loader.handoff(adt)} {adt.hex()}')

    def refused(self, pa, adt):
        adt = bytearray(adt)
        sim = R.LoaderRingSim(pa)

        def run():
            status = sim.handoff(adt)
            return f'Z {status} {int(sim.published)} {adt.hex()}'
        return self._op('L', f'Z {pa:x} {adt.hex() or "-"}', run)

    def region_ok(self, pa):
        return self._op('L', f'R {pa:x}', lambda: f'R {int(R.region_ok(pa))}')

    def out(self, value, index, length, data, received=None):
        received = length if received is None else received
        data = bytes(data)[:received]

        def run():
            if not self.loader.out_ok(value, length):
                return 'O STALL'
            return 'O OK' if self.loader.out_data(received == length, data, length, index) else 'O DROP'
        return self._op('L', f'O {value:x} {index:x} {length:x} {received:x} {data.hex() or "-"}', run)

    def in_(self, value, index, length):
        def run():
            reply = self.loader.in_reply(value, index, length)
            return 'I STALL' if reply is None else 'I ' + reply.hex()
        out = self._op('L', f'I {value:x} {index:x} {length:x}', run)
        return None if out == 'I STALL' else bytes.fromhex(out[2:])

    # kernel side
    def k_attach(self):
        self._op('K', 'A', lambda: self.kernel.attach())

    def k_state(self, bit, on):
        self._op('K', f'S {(self.kernel.kstate | bit if on else self.kernel.kstate & ~bit):x}',
                 lambda: self.kernel.set_state(bit, on))

    def k_tx(self, data):
        return self._op('K', f'T {bytes(data).hex()}', lambda: 'T ' + self.kernel.tx(bytes(data)).upper())

    def k_rx(self):
        def run():
            got, data = self.kernel.rx_take()
            if got != 'empty':
                self.kernel.rx_done(got == 'ok')
            return 'X ' + got.upper() + (' ' + data.hex() if got == 'ok' else '')
        out = self._op('K', 'X', run)
        return bytes.fromhex(out[5:]) if out.startswith('X OK') else out[2:]

    def dump(self, tmp):
        n = len(self.dumps)
        self._sync(self.lview, self.lops)
        self._sync(self.kview, self.kops)
        self.lops.append(f'D {tmp}/loader-{n}.bin')
        self.kops.append(f'D {tmp}/kernel-{n}.bin')
        self.dumps.append(bytes(self.region.bytes))


def sample_adts():
    """The synthetic ADTs of part 4: mkadt-salami.py add_usbnet on a small tree, and look-alikes."""
    mk = load('mkadt_salami', ROOT / 'tools/mkadt-salami.py')
    import adt
    import mkadt

    def tree(*extra_chosen, more=()):
        chosen = mkadt.node('chosen', [('firmware-version', b'salami-shim\0'), *extra_chosen])
        return mkadt.node('device-tree', [('compatible', b'D22AP\0')],
                          children=[mkadt.node('defaults', [('no-effaceable-storage', struct.pack('<I', 1))]), chosen, *more])
    one = tree()
    added = mk.add_usbnet(one)
    two = tree()
    mk.add_usbnet(two)
    two.children.append(mkadt.node('arm-io', [('fuxi-usbnet', R.PLACEHOLDER)]))
    flagged = tree()
    flagged.children[1].props.append(('fuxi-usbnet', 1, R.PLACEHOLDER))
    return mk, adt, {
        'one': adt.serialize(one), 'none': adt.serialize(tree()), 'two': adt.serialize(two),
        'short': adt.serialize(tree(('fuxi-usbnet', R.PLACEHOLDER[:8]))),
        'flag': adt.serialize(flagged), 'name': adt.serialize(tree(('fuxi-usbnetX', R.PLACEHOLDER))),
        'value': adt.serialize(tree(('fuxi-usbnet', b'FXNET-PLACEHOLDr'))),
    }, added


def scenario(d, tmp, adts):
    ok = []

    def expect(name, cond, detail=''):
        ok.append((name, bool(cond), detail))

    # A. Before the launch and the handoff (SPEC A1: a STALL drops the device from WinUSB):
    # 0x5d takes its data stage and drops the frame, 0x5e answers wLength zero bytes; nothing
    # to count in, the region untouched. Only a malformed request STALLs.
    expect('A: before launch 0x5d is taken and dropped, no STALL', d.out(R.VALUE_OUT, 0, 60, frame(60, 1)) == 'O DROP')
    expect('A: before launch 0x5e answers 192 zero bytes, no STALL', d.in_(R.VALUE_IN, 0, 192) == bytes(192))
    d.launch(1)
    expect('A: launched but unpublished: 0x5d dropped, 0x5e 4096 and 193 zero bytes, region untouched',
           d.out(R.VALUE_OUT, 0, 1514, frame(1514, 1)) == 'O DROP' and d.in_(R.VALUE_IN, 0x10, 4096) == bytes(4096) and
           d.in_(R.VALUE_IN, 0, 193) == bytes(193) and not any(d.region.bytes))
    expect('A: unpublished, malformed requests still STALL (0x5d wLength 13, wValue NS; 0x5e wLength 191, 4097)',
           d.out(R.VALUE_OUT, 0, 13, frame(13, 1)) == 'O STALL' and d.out(R.VALUE_IN, 0, 60, frame(60, 1)) == 'O STALL' and
           d.in_(R.VALUE_IN, 0, 191) is None and d.in_(R.VALUE_IN, 0, 4097) is None and not any(d.region.bytes))
    # Region refusals: nothing written, nothing published, the ADT left alone.
    for pa in (R.RAM_BASE, R.RAM_BASE + R.RAM_SIZE - 0x4000, R.RAM_BASE - 0x20000, R.SCRATCH,
               R.SCRATCH + R.SCRATCH_SIZE - 0x4000, BASE + 0x2000, 0, (1 << 64) - 0x4000):
        expect(f'region {pa:#x} refused', d.region_ok(pa) == 'R 0')
    for pa in (R.RAM_BASE - R.SIZE, R.SCRATCH + R.SCRATCH_SIZE, 0x80000000, BASE):
        expect(f'region {pa:#x} accepted', d.region_ok(pa) == 'R 1')
    for pa in (R.RAM_BASE + 0x100000, R.SCRATCH, BASE + 0x1000):
        out = d.refused(pa, adts['one'])
        expect(f'handoff refuses {pa:#x}: status 4, unpublished, ADT untouched', out == f'Z 4 0 {adts["one"].hex()}', out[:40])
    # B. The handoff: ADTs without exactly one placeholder are left alone (status 2 or 3, the
    # region still published for 0x5e); then the real one: line 0, the placeholder patched.
    for name, status in (('none', 2), ('two', 3), ('short', 2), ('flag', 2), ('name', 2), ('value', 2)):
        expect(f'B: handoff with ADT "{name}": status {status}, ADT unchanged, 0x5e shows it',
               d.handoff(adts[name]) == f'H {status} {adts[name].hex()}' and
               R.parse_header(d.in_(R.VALUE_IN, 0, 192))['adt_status'] == status)
    out = d.handoff(adts['one'])
    patched = bytes.fromhex(out.split()[2])
    at = adts['one'].index(R.PLACEHOLDER)
    expect('B: handoff status 1, the ADT value is {pa, 0x24000}, nothing else changed',
           out.startswith('H 1 ') and patched[at:at + 16] == struct.pack('<QQ', BASE, R.SIZE) and
           patched[:at] == adts['one'][:at] and patched[at + 16:] == adts['one'][at + 16:])
    h = R.parse_header(d.in_(R.VALUE_IN, 0, 192))
    expect('B: 0x5e header: magic, version, size, self_pa, slots, offsets, adt_status, no kernel yet',
           h['loader_ok'] and h['self_pa'] == BASE and (h['slot_size'], h['slots'], h['rx_off'], h['tx_off']) ==
           (2048, 32, 0x4000, 0x14000) and h['adt_status'] == 1 and not h['kernel_up'] and h['in_count'] == 1, h)
    # C. 0x5d once published. Without the kernel's opt-in the frame is taken and dropped
    # (out_stall_other), leaving host_tx_tail alone even when wIndex would rebuild (0xfff0 is
    # inside the window of tx_head 0); only a malformed request STALLs (counted too).
    expect('C: no kmagic: dropped, out_stall_other, host_tx_tail and tail_bad untouched',
           d.out(R.VALUE_OUT, 0xfff0, 60, frame(60, 2)) == 'O DROP' and d.u32('out_stall_other') == 1 and
           d.u32('host_tx_tail') == 0 and d.u32('tail_bad') == 0)
    # A 0x5e whose wIndex is refused (0x100 is 2^16 - 0x100 behind tx_head 0: tail_bad) keeps
    # host_tx_tail, so its header shows what the drop left there (to the C program too).
    h = R.parse_header(d.in_(R.VALUE_IN, 0x100, 192))
    expect('C: the 0x5e after the drop shows host_tx_tail 0 (wIndex refused: tail_bad 1)',
           h['host_tx_tail'] == 0 and h['tail_bad'] == 1 and h['out_stall_other'] == 1, h)
    d.k_state(R.KSTATE_UP, True)
    expect('C: kstate bit0 without kmagic: dropped', d.out(R.VALUE_OUT, 0, 60, frame(60, 2)) == 'O DROP')
    d.poke(0x40, 0x454e5846)
    d.poke(0x44, 0x31304c54)
    expect('C: the loader magic in line 1 is not kmagic: dropped', d.out(R.VALUE_OUT, 0, 60, frame(60, 2)) == 'O DROP')
    d.k_state(R.KSTATE_UP, False)
    d.k_attach()
    expect('C: kmagic but kstate bit0 clear: dropped', d.out(R.VALUE_OUT, 0, 60, frame(60, 2)) == 'O DROP' and d.u32('out_stall_other') == 4)
    d.k_state(R.KSTATE_LINK, True)
    expect('C: kstate bit1 alone is not up: dropped', d.out(R.VALUE_OUT, 0, 60, frame(60, 2)) == 'O DROP')
    d.k_state(R.KSTATE_UP, True)
    expect('C: wValue NS on 0x5d: STALL (malformed)', d.out(R.VALUE_IN, 0, 60, frame(60, 2)) == 'O STALL')
    expect('C: wLength 13: STALL', d.out(R.VALUE_OUT, 0, 13, frame(13, 2)) == 'O STALL')
    expect('C: wLength 1515: STALL', d.out(R.VALUE_OUT, 0, 1515, frame(1515, 2)) == 'O STALL')
    expect('C: drops and STALLs all counted out_stall_other, none full, nothing in the ring',
           d.u32('out_stall_other') == 8 and d.u32('out_stall_full') == 0 and d.u32('rx_head') == 0 and d.u32('host_tx_tail') == 0)
    expect('C: wLength 14 accepted', d.out(R.VALUE_OUT, 0, 14, frame(14, 3)) == 'O OK')
    expect('C: wLength 1514 accepted', d.out(R.VALUE_OUT, 0, 1514, frame(1514, 4)) == 'O OK')
    expect('C: wLength 64 (exact packet multiple) accepted', d.out(R.VALUE_OUT, 0, 64, frame(64, 5)) == 'O OK')
    d.launch(0)
    in_count = d.u32('in_count')
    expect('C: MON_LAUNCH clear: 0x5d dropped (counted), 0x5e zero bytes; in_count, host_tx_tail, tail_bad untouched',
           d.out(R.VALUE_OUT, 0, 60, frame(60, 6)) == 'O DROP' and d.in_(R.VALUE_IN, 0xfff0, 192) == bytes(192) and
           d.u32('out_stall_other') == 9 and d.u32('rx_head') == 3 and d.u32('in_count') == in_count and
           d.u32('host_tx_tail') == 0 and d.u32('tail_bad') == 1)
    d.launch(1)
    # D. 0x5e lengths and wValue.
    expect('D: 0x5e wLength 191 STALL, 4097 STALL, wValue NR STALL',
           d.in_(R.VALUE_IN, 0, 191) is None and d.in_(R.VALUE_IN, 0, 4097) is None and d.in_(R.VALUE_OUT, 0, 192) is None)
    r192, r4096, r193 = d.in_(R.VALUE_IN, 0, 192), d.in_(R.VALUE_IN, 0, 4096), d.in_(R.VALUE_IN, 0, 193)
    expect('D: 0x5e replies exactly wLength (192, 193, 4096), zero padded', r192 is not None and len(r192) == 192 and
           len(r193) == 193 and len(r4096) == 4096 and not any(r4096[192:]) and not any(r193[192:]))
    # E. The kernel consumes RX: frames as sent, then empty.
    expect('E: kernel RX gets the three frames in order',
           [d.k_rx() for _ in range(3)] == [frame(14, 3), frame(1514, 4), frame(64, 5)] and d.k_rx() == 'EMPTY')
    # F. RX full: 32 frames in, the 33rd is taken and dropped as out_stall_full (never a
    # STALL); full together with any other reason counts out_stall_other.
    for i in range(32):
        d.out(R.VALUE_OUT, 0, 60 + i, frame(60 + i, 10 + i))
    full_before, other_before, head = d.u32('out_stall_full'), d.u32('out_stall_other'), d.u32('rx_head')
    expect('F: RX ring full: 0x5d taken and dropped, out_stall_full + 1, rx_head unchanged',
           d.out(R.VALUE_OUT, 0, 60, frame(60, 9)) == 'O DROP' and d.u32('out_stall_full') == full_before + 1 and
           d.u32('out_stall_other') == other_before and d.u32('rx_head') == head)
    expect('F: full plus bad wLength: STALL (malformed), out_stall_other', d.out(R.VALUE_OUT, 0, 13, frame(13, 9)) == 'O STALL' and
           d.u32('out_stall_other') == other_before + 1 and d.u32('out_stall_full') == full_before + 1)
    expect('F: full plus a residue mismatch: dropped as out_stall_other',
           d.out(R.VALUE_OUT, 0, 60, frame(61, 9), received=59) == 'O DROP' and
           d.u32('out_stall_other') == other_before + 2 and d.u32('out_stall_full') == full_before + 1)
    d.k_state(R.KSTATE_UP, False)
    expect('F: full plus kstate bit0 clear: dropped as out_stall_other', d.out(R.VALUE_OUT, 0, 60, frame(60, 9)) == 'O DROP' and
           d.u32('out_stall_other') == other_before + 3 and d.u32('out_stall_full') == full_before + 1)
    d.k_state(R.KSTATE_UP, True)
    got = [d.k_rx() for _ in range(5)]
    expect('F: the kernel frees five slots, five more frames go in, the next is dropped again (full)',
           got == [frame(60 + i, 10 + i) for i in range(5)] and
           all(d.out(R.VALUE_OUT, 0, 100, frame(100, 50 + i)) == 'O OK' for i in range(5)) and
           d.out(R.VALUE_OUT, 0, 100, frame(100, 60)) == 'O DROP' and d.u32('out_stall_full') == full_before + 2 and
           d.u32('rx_head') == head + 5)
    # G. Residue mismatch: the setup passes, the short or long data stage is dropped.
    while d.k_rx() != 'EMPTY':
        pass
    head, other = d.u32('rx_head'), d.u32('out_stall_other')
    expect('G: residue mismatch (59 of 60, 61 of 60) dropped as out_stall_other, rx_head unchanged',
           d.out(R.VALUE_OUT, 0, 60, frame(61, 70), received=59) == 'O DROP' and
           d.out(R.VALUE_OUT, 0, 60, frame(61, 70), received=61) == 'O DROP' and
           d.u32('rx_head') == head and d.u32('out_stall_other') == other + 2 and d.k_rx() == 'EMPTY')
    # H. Kernel TX listed by 0x5e: exact fit, truncation, consumption through wIndex.
    for i in range(5):
        d.k_tx(frame(60, 80 + i))
    exact = d.in_(R.VALUE_IN, 0, 192 + 76 * 2)
    short = d.in_(R.VALUE_IN, 0, 192 + 76 * 2 - 1)
    e = entries(exact)
    expect('H: exact fit: two 60-byte entries fill 192 + 2 * 76 bytes; one byte less lists one',
           [x[3] for x in e] == [frame(60, 80), frame(60, 81)] and [x[1] for x in e] == [0, 1] and
           [x[3] for x in entries(short)] == [frame(60, 80)] and all(R.fnv1a32(x[3]) == x[2] for x in e))
    all5 = entries(d.in_(R.VALUE_IN, 2, 4096))
    expect('H: wIndex 2: host_tx_tail 2, the list starts at seq 2', [x[1] for x in all5] == [2, 3, 4] and
           d.u32('host_tx_tail') == 2)
    # Odd lengths: the pad bytes in the reply are zero even if the slot holds garbage there.
    d.k_tx(frame(61, 90))
    slot = R.TX_OFF + 5 * R.SLOT_SIZE
    d.poke(slot + 16 + 60, 0xeeeeee00 | frame(61, 90)[60])
    e = entries(d.in_(R.VALUE_IN, 5, 4096))
    raw = d.in_(R.VALUE_IN, 5, 4096)
    expect('H: a 61-byte frame is listed with its pad zeroed', e and e[0][3] == frame(61, 90) and
           raw[192 + 16 + 61:192 + 16 + 64] == b'\0\0\0')
    # TX full: the kernel stops at host_tx_tail + 32 and counts.
    for i in range(31):
        d.k_tx(frame(40, 100 + i))
    expect('H: TX ring full at tx_head - host_tx_tail == 32: tx_full_drops',
           d.k_tx(frame(40, 1)) == 'T FULL' and d.u32('tx_head') == 37 and d.u32('tx_full_drops') == 1)
    expect('H: kernel TX sizes 13 and 1515 refused (tx_size_drops)', d.k_tx(frame(13, 1)) == 'T SIZE' and
           d.k_tx(frame(1515, 1)) == 'T SIZE' and d.u32('tx_size_drops') == 2)
    # Corrupted TX slots end the list: a wrong seq, then a len out of range.
    d.in_(R.VALUE_IN, 30, 192)
    for off, value, what in ((R.TX_OFF + 33 % 32 * R.SLOT_SIZE + 4, 999, 'seq'), (R.TX_OFF + 33 % 32 * R.SLOT_SIZE, 13, 'len 13'),
                             (R.TX_OFF + 33 % 32 * R.SLOT_SIZE, 1515, 'len 1515')):
        keep = d.region.u32(off)
        d.poke(off, value)
        e = entries(d.in_(R.VALUE_IN, 30, 4096))
        expect(f'H: a corrupted TX slot ({what}) ends the list', [x[1] for x in e] == [30, 31, 32])
        d.poke(off, keep)
    expect('H: repaired, the list runs to tx_head', [x[1] for x in entries(d.in_(R.VALUE_IN, 30, 4096))] == [30, 31, 32, 33, 34, 35, 36])
    # I. host_tx_tail rebuild edges (tx_head 37).
    bad = d.u32('tail_bad')
    d.in_(R.VALUE_IN, 37, 192)
    expect('I: wIndex = tx_head: accepted', d.u32('host_tx_tail') == 37)
    d.in_(R.VALUE_IN, 5, 192)
    expect('I: wIndex = tx_head - 32: accepted', d.u32('host_tx_tail') == 5)
    d.in_(R.VALUE_IN, 4, 192)
    expect('I: wIndex = tx_head - 33: refused, kept, tail_bad', d.u32('host_tx_tail') == 5 and d.u32('tail_bad') == bad + 1)
    d.in_(R.VALUE_IN, 38, 192)
    expect('I: wIndex = tx_head + 1: refused (it would be 2^16 behind)', d.u32('host_tx_tail') == 5 and d.u32('tail_bad') == bad + 2)
    d.out(R.VALUE_OUT, 37, 60, frame(60, 120))
    expect('I: 0x5d carries wIndex too: host_tx_tail 37', d.u32('host_tx_tail') == 37)
    d.k_rx()
    d.dump(tmp)
    # J. Every counter across 2^32 and wIndex across 2^16, both directions, through the ring.
    near = 0xffffffe8
    for name in ('rx_head', 'rx_tail', 'tx_head', 'host_tx_tail', 'out_ok', 'out_stall_full', 'out_stall_other',
                 'in_count', 'tail_bad', 'rx_frames', 'rx_bad', 'tx_frames', 'tx_full_drops', 'tx_size_drops'):
        d.poke(R.FIELDS[name], near + 4 if name not in ('rx_head', 'rx_tail', 'tx_head', 'host_tx_tail') else near)
    consumed, phone, host, outs = near, [], [], []
    sent = [frame(20 + 37 * i % 1400, i) for i in range(70)]
    up = [frame(14 + 41 * i % 1500, 200 + i) for i in range(70)]
    put = rx_i = 0
    for rnd in range(300):
        while put < len(up) and d.k_tx(up[put]) == 'T OK':
            put += 1
        raw = d.in_(R.VALUE_IN, consumed & 0xffff, 4096)
        for n, seq, want, data in entries(raw):
            expect(f'J: seq {seq:#x} in order with a good fnv', seq == consumed and R.fnv1a32(data) == want)
            host.append(data)
            consumed = (consumed + 1) & R.M32
        # The host's side of A1: never 32 frames in flight (rx_head - rx_tail from the header).
        while rx_i < len(sent) and (d.u32('rx_head') - d.u32('rx_tail')) & R.M32 < R.SLOTS:
            outs.append(d.out(R.VALUE_OUT, consumed & 0xffff, len(sent[rx_i]), sent[rx_i]))
            rx_i += 1
        for _ in range(rnd % 3 + 1):
            got = d.k_rx()
            if got == 'EMPTY':
                break
            phone.append(got)
        if len(host) == len(up) and len(phone) == len(sent):
            break
    d.in_(R.VALUE_IN, consumed & 0xffff, 192)
    expect('J: 70 frames each way across 2^32 (and wIndex across 2^16), in order, none lost',
           host == up and phone == sent, (len(host), len(phone)))
    expect('J: A1: a host keeping fewer than 32 frames in flight sees no STALL and no drop',
           outs == ['O OK'] * len(sent) and d.u32('out_stall_full') == (near + 4) & R.M32 and
           d.u32('out_stall_other') == (near + 4) & R.M32, (outs[:3], d.u32('out_stall_full'), d.u32('out_stall_other')))
    expect('J: indexes and counters wrapped', d.u32('rx_head') == (near + 70) & R.M32 and d.u32('tx_head') == (near + 70) & R.M32 and
           d.u32('host_tx_tail') == d.u32('tx_head') and d.u32('rx_tail') == d.u32('rx_head') and d.u32('out_ok') < 100 and
           d.u32('in_count') < 200 and d.u32('rx_frames') < 100 and d.u32('tx_frames') < 100)
    # wIndex across 2^16 alone: tx_head 0x1fff0 .. 0x20010.
    d.poke(R.FIELDS['tx_head'], 0x1fff0)
    d.poke(R.FIELDS['host_tx_tail'], 0x1fff0)
    consumed, got = 0x1fff0, []
    for i in range(40):
        d.k_tx(frame(30, i))
        if i % 3 == 2:
            for n, seq, want, data in entries(d.in_(R.VALUE_IN, consumed & 0xffff, 4096)):
                got.append(seq)
                consumed += 1
    for n, seq, want, data in entries(d.in_(R.VALUE_IN, consumed & 0xffff, 4096)):
        got.append(seq)
        consumed += 1
    expect('J: wIndex across 2^16 (0xfff0 .. 0x0010): every seq once, in order', got == list(range(0x1fff0, 0x1fff0 + 40)), got[:5])
    # K. RX slots the kernel refuses (a broken loader would write them): consumed, counted rx_bad.
    # The len cases carry a matching fnv, so only the len rule refuses them.
    head = d.u32('rx_head')

    def bad_len(s, n):
        d.poke(s, n)
        d.poke(s + 8, R.fnv1a32(bytes(d.region.bytes[s + 16:s + 16 + n])))
    for fix, what in ((lambda s: d.poke(s + 8, d.region.u32(s + 8) ^ 1), 'fnv'), (lambda s: d.poke(s + 4, 7), 'seq'),
                      (lambda s: bad_len(s, 13), 'len 13'), (lambda s: bad_len(s, 1515), 'len 1515')):
        d.out(R.VALUE_OUT, 0x10, 80, frame(80, 7))
        fix(R.RX_OFF + head % 32 * R.SLOT_SIZE)
        bad = d.u32('rx_bad')
        expect(f'K: kernel RX refuses a bad {what}: consumed, rx_bad', d.k_rx() == 'BAD' and d.u32('rx_bad') == bad + 1 and
               d.u32('rx_tail') == head + 1)
        head += 1
    d.dump(tmp)
    return ok


def run_programs(tmp, d, loader_exe, kernel_exe, label=''):
    """Both C programs on the scenario: (loader outputs ok, kernel outputs ok, dumps ok, detail)."""
    rc1, out1 = run_exe(loader_exe, '\n'.join(d.lops) + '\n')
    rc2, out2 = run_exe(kernel_exe, '\n'.join(d.kops) + '\n')
    l_ok = rc1 == 0 and out1.splitlines() == d.lexp
    k_ok = rc2 == 0 and out2.splitlines() == d.kexp
    dumps = []
    for i, want in enumerate(d.dumps):
        for side in ('loader', 'kernel'):
            path = tmp / f'{side}-{i}.bin'
            dumps.append(path.exists() and path.read_bytes() == want)
            if path.exists():
                path.unlink()
    detail = ''
    if not l_ok:
        lines = out1.splitlines()
        first = next((i for i, (a, b) in enumerate(zip(lines, d.lexp)) if a != b), min(len(lines), len(d.lexp)))
        detail += f'loader rc {rc1}, first difference at output {first}: {lines[first:first + 1]} vs {d.lexp[first:first + 1]} {out1[-300:] if rc1 else ""}'
    if not k_ok:
        lines = out2.splitlines()
        first = next((i for i, (a, b) in enumerate(zip(lines, d.kexp)) if a != b), min(len(lines), len(d.kexp)))
        detail += f' kernel rc {rc2}, first difference at output {first}: {lines[first:first + 1]} vs {d.kexp[first:first + 1]} {out2[-300:] if rc2 else ""}'
    return l_ok, k_ok, all(dumps), detail


def ring_suite(tmp, adts):
    d = Dual()
    expectations = scenario(d, tmp, adts)
    bad = [(n, x) for n, ok, x in expectations if not ok]
    check(f'scenario (Python models): {len(expectations)} spec expectations hold', not bad, bad[:5])
    loader_exe = compile_c(tmp, 'loader-ring', LOADER_C)
    kernel_exe = compile_c(tmp, 'kernel-ring', KERNEL_C, includes=(WIFI_DIR,))
    l_ok, k_ok, dumps_ok, detail = run_programs(tmp, d, loader_exe, kernel_exe)
    check(f'usbnet.h (C) == LoaderRingSim: {len(d.lexp)} replies and STALL/drop decisions, byte for byte', l_ok, detail)
    check(f'fuxi_usbnet_ring.h (C) == KernelSim: {len(d.kexp)} TX/RX results, byte for byte', k_ok, detail)
    check(f'region bytes equal at {len(d.dumps)} checkpoints (C loader, C kernel, Python)', dumps_ok)
    return d


# ---- 3. probe.c glue: ud_setup's route U branches and ud_event on fake DWC3 registers -------
GLUE_HEAD = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "monitor.h"
#include "usbnet.h"
#define printf(...) ((void)0)
static uint64_t ticks = 1000;
static uint64_t fake_sysreg(const char *r) { return !strcmp(r, "cntpct_el0") ? ticks : 19200000; }
#define rsr(r) fake_sysreg(r)
volatile uint64_t monitor_report[MON_FIELDS];
char monitor_error[128];
uint8_t _usbnet_start[0x24000] __attribute__((aligned(0x4000)));
'''
GLUE_STUBS = r'''
static struct { uint64_t configured, speed; } probe_info;
static uint8_t dma_area[0x4000] __attribute__((aligned(0x4000)));
static uint32_t regs[0x10000 / 4];
static unsigned stalls, starts[2];
static void ud_sync(uint32_t off, uint32_t len) { (void)off; (void)len; }
static void delay(unsigned us) { (void)us; }
static void au_out_data(uint16_t i, const volatile uint8_t *p, uint16_t n) { (void)i; (void)p; (void)n; assert(!"audio"); }
static void dwc_wr(uint32_t off, uint32_t v)
{
 if (off >= DEPCMD(0) && off <= DEPCMD(1) && !((off - DEPCMD(0)) % 0x10)) {
  unsigned ep = (off - DEPCMD(0)) / 0x10;
  if ((v & 0xf) == DEPCMD_SETSTALL) stalls++;
  if ((v & 0xf) == DEPCMD_STARTTRANSFER) starts[ep]++;
  regs[off / 4] = 0;
  return;
 }
 regs[off / 4] = v;
}
static uint32_t dwc_rd(uint32_t off) { return regs[off / 4]; }
'''
GLUE_MAIN = r'''
#undef printf
static uint32_t trb_len(unsigned ep) { return ((volatile uint32_t *)(dma_area + (ep ? UD_TRB_IN : UD_TRB_OUT)))[2]; }
static void setup(uint8_t type, uint8_t req, uint16_t value, uint16_t index, uint16_t length)
{
 uint8_t *s = dma_area + UD_SETUP;
 s[0] = type; s[1] = req; s[2] = (uint8_t)value; s[3] = value >> 8; s[4] = (uint8_t)index; s[5] = index >> 8;
 s[6] = (uint8_t)length; s[7] = length >> 8;
 assert(ud.state == EP0_SETUP);
 ud_event(0x040);                       /* ep0 OUT XferComplete: the SETUP packet */
}
static void finish_out(const uint8_t *data, uint32_t got)
{
 assert(ud.state == EP0_DATA && !ud.data_in);
 memset(dma_area + UD_DATA, 0xa5, 2048);
 memcpy(dma_area + UD_DATA, data, got);
 ((volatile uint32_t *)(dma_area + UD_TRB_OUT))[2] = trb_len(0) - got;   /* the untransferred remainder */
 ud_event(0x040);                       /* the data stage */
 assert(ud.state == EP0_WAIT_STATUS3 && !ud.usbnet_out);
 ud_event(0x20c2); ud_event(0x042);     /* status stage on ep0 IN */
 assert(ud.state == EP0_SETUP);
}
static void finish_in(void)
{
 assert(ud.state == EP0_DATA && ud.data_in);
 ud_event(0x042); ud_event(0x20c0); ud_event(0x040);
 assert(ud.state == EP0_SETUP);
}
static uint32_t rg(uint32_t off) { return *(volatile uint32_t *)(_usbnet_start + off); }
/* An IN data stage of n bytes that came back all zero (the not-ready reply). */
static void zero_reply(uint16_t n)
{
 memset(dma_area + UD_DATA, 0xa5, 4100);
 setup(0xc0, 0x5e, 0x4e53, 0, n);
 assert(ud.state == EP0_DATA && ud.data_in && trb_len(1) == n);
 for (unsigned i = 0; i < n; i++) assert(dma_area[UD_DATA + i] == 0);
 finish_in();
}
int main(void)
{
 static uint8_t adt[52], f[1514];
 for (unsigned i = 0; i < sizeof f; i++) f[i] = (uint8_t)(i * 7 + 3);
 ud.dma = (uintptr_t)dma_area; ud.dma_pa = ud.dma; ud_start_setup();
 unsigned s0 = stalls;
 /* Unpublished (and before the launch), SPEC A1: 0x5d takes its data stage and drops the
  * frame, 0x5e answers wLength zero bytes; no STALL, nothing in the region. */
 setup(0x40, 0x5d, 0x4e52, 0, 60);
 assert(ud.state == EP0_DATA && ud.usbnet_out && trb_len(0) == 64);
 finish_out(f, 60);
 zero_reply(192);
 monitor_report[MON_LAUNCH] = 1;
 zero_reply(4096); zero_reply(193);
 setup(0x40, 0x5d, 0x4e52, 0, 1514); finish_out(f, 1514);
 assert(stalls == s0 && !rg(0));
 memcpy(adt, "fuxi-usbnet", 11); adt[32] = 16; memcpy(adt + 36, "FXNET-PLACEHOLDR", 16);
 assert(probe_usbnet_handoff((uintptr_t)adt, sizeof adt) == UN_ADT_PATCHED);
 uint64_t pa; memcpy(&pa, adt + 36, 8);
 assert(pa == (uintptr_t)_usbnet_start && rg(0) == UN_MAGIC_LO && rg(UN_L0_ADT) == 1);
 setup(0x40, 0x5d, 0x4e52, 0, 60); finish_out(f, 60);                   /* no kernel yet: dropped */
 assert(stalls == s0 && rg(UN_L_OUT_OTHER) == 1 && !rg(UN_L_RX_HEAD));
 *(volatile uint32_t *)(_usbnet_start + UN_K_MAGIC) = UN_KMAGIC_LO;
 *(volatile uint32_t *)(_usbnet_start + UN_K_MAGIC + 4) = UN_KMAGIC_HI;
 *(volatile uint32_t *)(_usbnet_start + UN_K_STATE) = UN_KSTATE_UP;
 /* 0x5d 60: OUT TRB 64, the frame lands in RX slot 0. */
 unsigned o0 = starts[0];
 setup(0x40, 0x5d, 0x4e52, 0, 60);
 assert(stalls == s0 && ud.state == EP0_DATA && ud.usbnet_out && trb_len(0) == 64 && starts[0] == o0 + 1);
 finish_out(f, 60);
 assert(rg(UN_L_RX_HEAD) == 1 && rg(UN_L_OUT_OK) == 1 && rg(UN_RX_OFF) == 60 && rg(UN_RX_OFF + 4) == 0 &&
        !memcmp(_usbnet_start + UN_RX_OFF + 16, f, 60) && !rg(UN_RX_OFF + 16 + 60));
 /* Residue: 59 of 60 dropped, the status stage still completes. */
 setup(0x40, 0x5d, 0x4e52, 0, 60); finish_out(f, 59);
 assert(rg(UN_L_RX_HEAD) == 1 && rg(UN_L_OUT_OTHER) == 2);
 /* 1514: TRB 1536; 64: TRB 64, nothing left over. */
 setup(0x40, 0x5d, 0x4e52, 0, 1514); assert(trb_len(0) == 1536); finish_out(f, 1514);
 setup(0x40, 0x5d, 0x4e52, 0, 64); assert(trb_len(0) == 64); finish_out(f, 64);
 assert(rg(UN_L_RX_HEAD) == 3 && !memcmp(_usbnet_start + UN_RX_OFF + 2048 + 16, f, 1514));
 /* Malformed requests STALL in the SETUP stage: 13, 1515, wValue (counted), wrong type. */
 setup(0x40, 0x5d, 0x4e52, 0, 13); setup(0x40, 0x5d, 0x4e52, 0, 1515); setup(0x40, 0x5d, 0x4e53, 0, 60);
 setup(0xc0, 0x5d, 0x4e52, 0, 60); setup(0x40, 0x5e, 0x4e53, 0, 192);
 assert(stalls == s0 + 5 && ud.state == EP0_SETUP && rg(UN_L_OUT_OTHER) == 5);
 /* 0x5e: IN TRB of exactly wLength (4096 and 4095), header copied. */
 unsigned i0 = starts[1];
 setup(0xc0, 0x5e, 0x4e53, 0, 4096);
 assert(trb_len(1) == 4096 && starts[1] == i0 + 1 && !memcmp(dma_area + UD_DATA, "FXNETL01", 8) && rg(UN_L_IN_COUNT) == 1);
 finish_in();
 setup(0xc0, 0x5e, 0x4e53, 0, 4095); assert(trb_len(1) == 4095); finish_in();
 setup(0xc0, 0x5e, 0x4e53, 0, 4097); setup(0xc0, 0x5e, 0x4e53, 0, 191); setup(0xc0, 0x5e, 0x4e52, 0, 192);
 assert(stalls == s0 + 8 && rg(UN_L_IN_COUNT) == 2);
 /* RX full (nothing reads it here): 29 more fill the ring, the next is taken and dropped. */
 for (unsigned i = 0; i < 29; i++) { setup(0x40, 0x5d, 0x4e52, 0, 100); finish_out(f, 100); }
 assert(rg(UN_L_RX_HEAD) == 32 && rg(UN_L_OUT_FULL) == 0);
 setup(0x40, 0x5d, 0x4e52, 0, 100); assert(ud.usbnet_out); finish_out(f, 100);
 assert(rg(UN_L_RX_HEAD) == 32 && rg(UN_L_OUT_FULL) == 1 && stalls == s0 + 8);
 /* MON_LAUNCH clear again: 0x5d dropped (out_stall_other), 0x5e zero bytes; no STALL. */
 monitor_report[MON_LAUNCH] = 0;
 unsigned other = rg(UN_L_OUT_OTHER), in = rg(UN_L_IN_COUNT);
 setup(0x40, 0x5d, 0x4e52, 0, 60); finish_out(f, 60);
 zero_reply(192);
 assert(stalls == s0 + 8 && rg(UN_L_OUT_OTHER) == other + 1 && rg(UN_L_IN_COUNT) == in && rg(UN_L_OUT_FULL) == 1);
 /* The setup log: route U said once each, anything else always. */
 assert(!un_log_setup(0x5d) && !un_log_setup(0x5e) && un_log_setup(0x46) && un_log_setup(0x5b));
 puts("glue: all assertions held");
 return 0;
}
'''


def glue_suite(tmp):
    config = between(PROBE, '#ifndef FUXI_USB_REATTACH', '#define UDIAG(x) ((void)0)\n#endif\n')
    defines = between(PROBE, '#define GCTL ', '/* DMA area layout */', include_end=False)
    state = between(PROBE, '/* DMA area layout */', '} ud;\n')
    route = between(PROBE, '#if FUXI_USBNET\n/* Route U (usbnet.h). The 0x5d OUT', '#define un_log_setup(req) true\n#endif\n')
    setup = function(PROBE, 'ud_setup')
    head = setup[:setup.index('#if defined(FUXI_SALAMI)\n\t/* Lanes not ported')]
    branch = between(setup, '    } else if (type == 0x40 && req == UN_REQ_OUT) {', '#endif\n    } else if (type == 0xc0 && req == 0x48)',
                     include_end=False)
    tail = setup[setup.index('\tif (!ok) {\n\t\tud_stall();'):]
    mini = head + '\tif (0) {\n' + branch + '    } else {\n        ok = false;\n    }\n' + tail
    funcs = ''.join(function(PROBE, n) for n in ('ud_epcmd', 'ud_start', 'ud_start_setup', 'ud_stall'))
    code = (GLUE_HEAD + config + defines + state + route + GLUE_STUBS + funcs + mini + function(PROBE, 'ud_event') + GLUE_MAIN)
    exe = compile_c(tmp, 'glue', code, ('FUXI_SALAMI=1', 'FUXI_XNU_LOADER=1'), extra=('-Wno-unused-variable',))
    rc, out = run_exe(exe)
    check('probe.c glue: no STALL before launch/handoff, without a kernel or with RX full (dropped, zero 0x5e); '
          'OUT TRB (len + 63) & ~63, residue drop, IN reply = wLength, only malformed requests STALL, log once',
          rc == 0 and 'glue: all assertions held' in out, out[-1500:])


# ---- 4. ADT -------------------------------------------------------------------------------
def adt_suite(tmp, mk, adt_mod, adts, added):
    one = adts['one']
    root, _ = adt_mod.parse(one)
    chosen = next(n for n in root.children if n.name == 'chosen')
    check('mkadt add_usbnet: /chosen fuxi-usbnet = the 16-byte placeholder, 52 bytes of ADT, flag 0',
          chosen.prop('fuxi-usbnet') == b'FXNET-PLACEHOLDR' and added == 52 and
          len(one) - len(adts['none']) == 52 and one.count(R.adt_property()) == 1 and
          [f for k, f, _ in chosen.props if k == 'fuxi-usbnet'] == [0])
    try:
        mk.add_usbnet(root)
        refused = False
    except ValueError:
        refused = True
    check('mkadt add_usbnet: a second one is refused', refused)
    src = (ROOT / 'tools/mkadt-salami.py').read_text()
    check('mkadt-salami.py: --usbnet option wired (add_usbnet, help names fuxiwifiios=2)',
          "ap.add_argument('--usbnet'" in src and 'usbnet_bytes = add_usbnet(root) if args.usbnet else 0' in src and
          'fuxiwifiios=2' in src)
    # The C loader patch on each ADT (fresh region each time).
    exe = compile_c(tmp, 'loader-adt', LOADER_C)
    want = {'one': 1, 'none': 2, 'two': 3, 'short': 2, 'flag': 2, 'name': 2, 'value': 2}
    got = {}
    for name, blob in adts.items():
        rc, out = run_exe(exe, f'H {blob.hex()}\n')
        status, patched = (int(out.split()[1]), bytes.fromhex(out.split()[2])) if rc == 0 and out.startswith('H ') else (None, b'')
        sim = R.LoaderRingSim(BASE)
        mine = bytearray(blob)
        got[name] = (status, sim.handoff(mine), patched == bytes(mine), patched == blob)
    check('loader ADT patch: one placeholder patched; none, two, 8 bytes, flag bit, other name, other value refused (C == sim)',
          all(got[n][0] == got[n][1] == want[n] and got[n][2] for n in want) and
          all(got[n][3] for n in want if n != 'one') and not got['one'][3], got)
    back, _ = adt_mod.parse(bytes.fromhex(run_exe(exe, f'H {one.hex()}\n')[1].split()[2]))
    value = next(n for n in back.children if n.name == 'chosen').prop('fuxi-usbnet')
    check('loader ADT patch: the kernel reads {pa, 0x24000} from the patched /chosen fuxi-usbnet',
          value == struct.pack('<QQ', BASE, R.SIZE), value)
    # A real mkadt-salami.py run, when this workstation has its inputs (as logo-boot/s31_build.sh).
    fdt = Path('/mnt/c/Users/troniclab/op11-tools/probe/run-now-145959/live.dtb')
    d22 = Path('/home/troniclab/op11-ios/rootbuild/devicetree/d22ap.adt')
    if fdt.is_file() and d22.is_file():
        outs = {}
        for flag in ('', '--usbnet'):
            path = tmp / f'real{flag}.adt'
            p = subprocess.run([sys.executable, '-B', str(ROOT / 'tools/mkadt-salami.py'), str(path), '--live-fdt', str(fdt),
                                '--smp', '--wlan-node', '--d22ap-adt', str(d22), *([flag] if flag else [])],
                               capture_output=True, text=True)
            outs[flag] = (p.returncode, path.read_bytes() if path.exists() else b'', p.stdout + p.stderr)
        plain, net = outs[''][1], outs['--usbnet'][1]
        rc, out = run_exe(exe, f'H {net.hex()}\n')
        check('mkadt-salami.py --usbnet (real inputs): 52 bytes more, one placeholder, the loader patches it (status 1)',
              outs[''][0] == 0 and outs['--usbnet'][0] == 0 and len(net) - len(plain) == 52 and
              net.count(R.adt_property()) == 1 and rc == 0 and out.startswith('H 1 ') and
              'fuxi-usbnet placeholder 52 bytes' in outs['--usbnet'][2], outs['--usbnet'][2][-300:])
    else:
        print('SKIP mkadt-salami.py real run: live FDT or D22AP ADT not on this machine (the synthetic tree above stands)')


# ---- 5b. kernel handoff and header checks -----------------------------------------------
KERNEL_CHECKS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "fuxi_usbnet_ring.h"
static uint8_t region[FXNET_SIZE] __attribute__((aligned(16384)));
static const char *h(uint64_t pa, uint64_t size, unsigned bytes, uint64_t *out)
{
 uint8_t v[16];
 memcpy(v, &pa, 8); memcpy(v + 8, &size, 8);
 return fxnet_handoff(v, bytes, out);
}
static void line0(uint64_t self)
{
 const uint32_t w[10] = {FXNET_MAGIC_LO, FXNET_MAGIC_HI, FXNET_VERSION, FXNET_SIZE, (uint32_t)self, (uint32_t)(self >> 32),
                         FXNET_SLOT_SIZE, FXNET_SLOTS, FXNET_RX_OFF, FXNET_TX_OFF};
 memcpy(region, w, sizeof w);
}
#define HAS(s, part) ((s) && strstr((s), (part)))
int main(void)
{
 uint64_t pa = 0;
 uint8_t ph[16]; memcpy(ph, "FXNET-PLACEHOLDR", 16);
 assert(HAS(fxnet_handoff(ph, 16, &pa), "placeholder"));
 assert(HAS(h(0xa0000000, FXNET_SIZE, 15, &pa), "not 16 bytes") && HAS(h(0xa0000000, FXNET_SIZE, 17, &pa), "not 16 bytes"));
 assert(HAS(h(0xa0000000, FXNET_SIZE + 1, 16, &pa), "0x24000") && HAS(h(0xa0000000, 0x20000, 16, &pa), "0x24000"));
 assert(HAS(h(0xa0002000, FXNET_SIZE, 16, &pa), "aligned") && HAS(h(0, FXNET_SIZE, 16, &pa), "aligned"));
 assert(HAS(h(0xffffffffffffc000ull, FXNET_SIZE, 16, &pa), "aligned"));
 assert(HAS(h(FXNET_RAM_BASE, FXNET_SIZE, 16, &pa), "XNU RAM") && HAS(h(FXNET_RAM_BASE - 0x20000, FXNET_SIZE, 16, &pa), "XNU RAM"));
 assert(HAS(h(FXNET_RAM_BASE + FXNET_RAM_SIZE - 0x4000, FXNET_SIZE, 16, &pa), "XNU RAM"));
 assert(HAS(h(FXNET_GPU_SCRATCH + FXNET_GPU_SCRATCH_SIZE - 0x4000, FXNET_SIZE, 16, &pa), "GPU scratch"));
 assert(pa == 0);
 assert(!h(FXNET_RAM_BASE - FXNET_SIZE, FXNET_SIZE, 16, &pa) && pa == FXNET_RAM_BASE - FXNET_SIZE);
 assert(!h(FXNET_GPU_SCRATCH + FXNET_GPU_SCRATCH_SIZE, FXNET_SIZE, 16, &pa) && pa == FXNET_GPU_SCRATCH + FXNET_GPU_SCRATCH_SIZE);
 assert(!h(0xa8000000, FXNET_SIZE, 16, &pa) && pa == 0xa8000000);
 const uintptr_t r = (uintptr_t)region;
 assert(HAS(fxnet_header(r, 0xa8000000), "magic"));
 line0(0xa8000000); assert(!fxnet_header(r, 0xa8000000));
 assert(HAS(fxnet_header(r, 0xa8004000), "address"));
 static const struct { uint32_t off, value; const char *why; } breaks[] = {
  {0x00, 0, "magic"}, {0x04, 0x31304b54, "magic"}, {0x08, 2, "version 1"}, {0x0c, 0x24004, "size"},
  {0x14, 1, "address"}, {0x18, 4096, "layout"}, {0x1c, 16, "layout"}, {0x20, 0x8000, "layout"}, {0x24, 0x14004, "layout"}};
 for (unsigned i = 0; i < sizeof breaks / sizeof breaks[0]; i++) {
  line0(0xa8000000);
  *(volatile uint32_t *)(r + breaks[i].off) = breaks[i].value;
  assert(HAS(fxnet_header(r, 0xa8000000), breaks[i].why));
 }
 line0(0xa8000000); fxnet_attach(r);
 assert(!memcmp(region + 0x40, "FXNETK01", 8) && !fxnet_header(r, 0xa8000000));
 puts("kernel checks: all assertions held");
 return 0;
}
'''


def kernel_checks(tmp, include=WIFI_DIR, name='kernel-checks', sanitize=True):
    rc, out = run_exe(compile_c(tmp, name, KERNEL_CHECKS, includes=(include,), sanitize=sanitize))
    return rc == 0 and 'kernel checks: all assertions held' in out, out


# ---- 6. the kernel backend on the wifi-ios model ----------------------------------------
BACKEND_MAIN = r'''
#include "usbnet.h"
bool fuxi_wifi_net_attach(const struct fuxi_wifi_net_link *p) {
 assert(p->wireless && !p->up && p->address[0]==2); attached++; return true;
}
static uint8_t region[0x24000] __attribute__((aligned(16384)));
static uint8_t adt[52], reply[4100];
static struct un un;
static void settle(void) {
 struct timespec pause = { .tv_nsec=1000000 };
 for (unsigned i=0;i<5000 && fuxi_wifi_sta_busy();i++) nanosleep(&pause,NULL);
 assert(!fuxi_wifi_sta_busy());
}
static int request(bool set,int type,void *p,unsigned n) {
 struct apple80211req req = { .req_if_name="en0",.req_type=type,.req_len=n,.req_data=(uintptr_t)p };
 return fuxi_wifi_apple80211_ioctl(set?SIOCSA80211:SIOCGA80211,&req);
}
static void join(void) {
 struct apple80211_assoc j={.version=1,.ssid_length=13}; memcpy(j.ssid,"Fuxi Loopback",13);
 assert(!request(true,20,&j,sizeof(j))); settle(); assert(link_up);
}
static uint32_t rg(uint32_t off) { return *(volatile uint32_t *)(region + off); }
static bool wait_for(bool (*cond)(void)) {
 struct timespec pause = { .tv_nsec=1000000 };
 for (unsigned i=0;i<5000;i++) { if (cond()) return true; nanosleep(&pause,NULL); }
 return false;
}
static bool kstate_up(void) { return rg(UN_K_STATE) & UN_KSTATE_UP; }
static unsigned want_received;
static bool got_frames(void) { pthread_mutex_lock(&rx_lock); bool r=received>=want_received; pthread_mutex_unlock(&rx_lock); return r; }
static uint32_t want_done;   /* rx_frames + rx_bad: fxnet_rx_done bumps them after the tail */
static bool done_at(void) { return rg(UN_K_RX_FRAMES)+rg(UN_K_RX_BAD)==want_done; }
/* What a broken loader could write: a whole slot with a wrong fnv, then rx_head + 1. */
static void bad_slot(void) {
 uint32_t head=rg(UN_L_RX_HEAD), slot=UN_RX_OFF+(head%32)*2048;
 *(volatile uint32_t *)(region+slot)=60; *(volatile uint32_t *)(region+slot+4)=head;
 *(volatile uint32_t *)(region+slot+8)=0x12345678; *(volatile uint32_t *)(region+slot+12)=0;
 __atomic_thread_fence(__ATOMIC_SEQ_CST);
 *(volatile uint32_t *)(region+UN_L_RX_HEAD)=head+1;
}
static void placeholder(void) { memset(adt,0,sizeof adt); memcpy(adt,"fuxi-usbnet",11); adt[32]=16; memcpy(adt+36,"FXNET-PLACEHOLDR",16); }
static void prop(const uint8_t *v, unsigned n) { model_usbnet_prop=v; model_usbnet_size=n; }
static void prop_of(uint64_t pa, uint64_t size) { memcpy(adt+36,&pa,8); memcpy(adt+44,&size,8); prop(adt+36,16); }
static int fallback(const char *why) {
 opt=2; fuxi_wifi_ios_init();
 char want[300]; snprintf(want,sizeof(want),"FUXI WIFI IOS: usbnet: %s; synthetic loopback instead\n",why);
 assert(said_times(want)==1 && said_times("FUXI WIFI IOS: usbnet:")==1);
 assert(said_times("FUXI WIFI IOS: enabled, synthetic backend; Apple80211 16.7 ABI unverified\n")==1);
 assert(attached==1 && model_usbnet_lookups==1 && rg(UN_K_MAGIC)==0);
 /* the synthetic loopback, unchanged */
 join();
 uint8_t f[60]={0}; f[12]=0x88; f[13]=0xb5;
 assert(transmit(f,60) && received==1 && rg(UN_K_TX_HEAD)==0);
 puts("fallback: PASS");
 return 0;
}
int main(int argc, char **argv) {
 assert(argc==2);
 const char *s=argv[1];
 model_usbnet_region=(uintptr_t)region; model_usbnet_pa=(uintptr_t)region;
 placeholder();
 if (!strcmp(s,"opt1")) {
  opt=1; fuxi_wifi_ios_init();
  assert(said_times("FUXI WIFI IOS: enabled, synthetic backend; Apple80211 16.7 ABI unverified\n")==1 && attached==1);
  assert(!model_usbnet_lookups && !model_usbnet_maps && !said_times("usbnet"));
  puts("opt1: PASS"); return 0;
 }
 if (!strcmp(s,"none")) return fallback("no /chosen fuxi-usbnet (mkadt-salami.py --usbnet)");
 if (!strcmp(s,"placeholder")) { prop(adt+36,16); return fallback("the placeholder is still there (no FUXI_USBNET loader, or it refused its region)"); }
 if (!strcmp(s,"short")) { prop(adt+36,8); return fallback("/chosen fuxi-usbnet is not 16 bytes"); }
 if (!strcmp(s,"size")) { prop_of((uintptr_t)region,0x20000); return fallback("the region is not 0x24000 bytes"); }
 if (!strcmp(s,"ram")) { prop_of(0x8c0000000ull+0x100000,0x24000); return fallback("the region is inside XNU RAM"); }
 if (!strcmp(s,"scratch")) { prop_of(0x9fe000000ull,0x24000); return fallback("the region is inside the GPU scratch"); }
 if (!strcmp(s,"unaligned")) { prop_of((uintptr_t)region+0x1000,0x24000); return fallback("the region is not 16 KiB aligned"); }
 if (!strcmp(s,"magic")) { prop_of((uintptr_t)region,0x24000); int r=fallback("no loader magic in the region"); assert(model_usbnet_maps==1); return r; }
 if (!strcmp(s,"self")) {
  assert(un_handoff(&un,(uintptr_t)region,(uintptr_t)adt,sizeof adt)==UN_ADT_PATCHED);
  *(volatile uint32_t *)(region+UN_L0_SELF)^=0x4000;
  prop(adt+36,16); return fallback("the region header names another size or address");
 }
 assert(!strcmp(s,"ok"));
 /* The real flow: mkadt's placeholder, the loader's handoff patches it, the kernel reads it. */
 assert(un_handoff(&un,(uintptr_t)region,(uintptr_t)adt,sizeof adt)==UN_ADT_PATCHED);
 prop(adt+36,16);
 assert(un_out_ok(&un,UN_VALUE_OUT,60) && !un_out_data(&un,true,true,reply,60,0) && rg(UN_L_OUT_OTHER)==1);  /* no kernel yet: dropped */
 opt=2; fuxi_wifi_ios_init();
 char want[300];
 snprintf(want,sizeof(want),"FUXI WIFI IOS: usbnet: loader region pa %#llx size 0x24000, en0 frames over the USB monitor\n",
          (unsigned long long)(uintptr_t)region);
 assert(said_times(want)==1 && said_times("FUXI WIFI IOS: enabled, usbnet backend; Apple80211 16.7 ABI unverified\n")==1);
 assert(attached==1 && registry==1 && model_usbnet_maps==1 && !memcmp(region+UN_K_MAGIC,"FXNETK01",8));
 assert(wait_for(kstate_up) && !(rg(UN_K_STATE)&UN_KSTATE_LINK));
 /* join: kstate bit1 follows en0's link */
 join(); assert(rg(UN_K_STATE)==(UN_KSTATE_UP|UN_KSTATE_LINK));
 /* TX: en0 -> the TX ring -> 0x5e */
 uint8_t f[1514];
 for (unsigned i=0;i<sizeof f;i++) f[i]=(uint8_t)(i*5+1);
 memcpy(f,"\xff\xff\xff\xff\xff\xff\x02\x46\x55\x58\x49\x01\x08\x06",14);
 assert(transmit(f,60) && transmit(f,1514));
 assert(un_in(&un,true,UN_VALUE_IN,0,4096,reply)==4096);
 uint16_t len; uint32_t seq, fnv;
 memcpy(&len,reply+0xc0,2); memcpy(&seq,reply+0xc4,4); memcpy(&fnv,reply+0xc8,4);
 assert(len==60 && seq==0 && !memcmp(reply+0xc0+16,f,60) && fnv==fg_hash(f,60));
 memcpy(&len,reply+0xc0+76,2); assert(len==1514 && !memcmp(reply+0xc0+76+16,f,1514));
 assert(!transmit(f,13) && !transmit(f,1515) && rg(UN_K_TX_SIZE)==0);  /* transmit() bounds first */
 /* TX full: 30 more fill the ring (host_tx_tail 0), the next is dropped and counted */
 for (unsigned i=0;i<30;i++) assert(transmit(f,100));
 assert(!transmit(f,100) && rg(UN_K_TX_FULL)==1 && rg(UN_K_TX_FRAMES)==32);
 assert(un_in(&un,true,UN_VALUE_IN,32,192,reply)==192 && transmit(f,100));     /* the host consumed all 32 */
 /* RX: 0x5d -> the RX ring -> the thread -> en0 */
 assert(un_out_ok(&un,UN_VALUE_OUT,60) && un_out_data(&un,true,true,f,60,33));
 want_received=1; assert(wait_for(got_frames));
 pthread_mutex_lock(&rx_lock); assert(rx_last_len==60 && !memcmp(rx_last,f,60)); pthread_mutex_unlock(&rx_lock);
 /* 40 more as a host does it (SPEC A1): never 32 in flight, so none is dropped */
 for (unsigned i=0;i<40;i++) { while (rg(UN_L_RX_HEAD)-rg(UN_K_RX_TAIL)>=32) nanosleep(&(struct timespec){.tv_nsec=100000},NULL);
  assert(un_out_ok(&un,UN_VALUE_OUT,(uint16_t)(14+i*37)) && un_out_data(&un,true,true,f,(uint16_t)(14+i*37),33)); }
 want_received=41; assert(wait_for(got_frames));
 want_done=41; assert(wait_for(done_at));
 pthread_mutex_lock(&rx_lock); assert(rx_last_len==14+39*37 && !memcmp(rx_last,f,rx_last_len)); pthread_mutex_unlock(&rx_lock);
 assert(rg(UN_K_RX_FRAMES)==41 && rg(UN_K_RX_BAD)==0 && rg(UN_K_RX_TAIL)==41);
 assert(rg(UN_L_OUT_OK)==41 && rg(UN_L_OUT_FULL)==0 && rg(UN_L_OUT_OTHER)==1);
 /* a bad slot (the loader never writes one): consumed, counted, not delivered */
 bad_slot();
 want_done=42; assert(wait_for(done_at));
 assert(rg(UN_K_RX_BAD)==1 && rg(UN_K_RX_TAIL)==42 && received==41);
 /* polls: 1 ms while frames moved in the last second, 20 ms when idle (time stands still here) */
 unsigned fast=atomic_load(&model_usbnet_waits[0]);
 assert(fast>0);
 now_abs=5*NSEC_PER_SEC;
 struct timespec pause={.tv_nsec=20000000}; nanosleep(&pause,NULL);
 unsigned slow=atomic_load(&model_usbnet_waits[1]);
 nanosleep(&pause,NULL);
 assert(atomic_load(&model_usbnet_waits[1])>slow);
 fast=atomic_load(&model_usbnet_waits[0]); nanosleep(&pause,NULL);
 assert(atomic_load(&model_usbnet_waits[0])==fast);
 /* a frame sent makes it fast again */
 assert(transmit(f,60)); nanosleep(&pause,NULL);
 assert(atomic_load(&model_usbnet_waits[0])>fast);
 /* leave: kstate bit1 cleared, bit0 stays */
 assert(!request(true,22,NULL,0)); settle(); assert(!link_up && rg(UN_K_STATE)==UN_KSTATE_UP);
 puts("ok: PASS");
 return 0;
}
'''
RX_STUB = 'static void fuxi_wifi_net_input(const uint8_t *p,size_t n) { assert(p[12]==0x88 && n>=14); received++; }'
RX_RECORD = ('static pthread_mutex_t rx_lock = PTHREAD_MUTEX_INITIALIZER; static uint8_t rx_last[1514]; static size_t rx_last_len;\n'
             'static void fuxi_wifi_net_input(const uint8_t *p,size_t n) { assert(n>=14 && n<=1514); pthread_mutex_lock(&rx_lock);\n'
             ' memcpy(rx_last,p,n); rx_last_len=n; received++; pthread_mutex_unlock(&rx_lock); }')


def backend_suite(tmp):
    W = load('test_wifi_ios', ROOT / 'tools/test-fuxi-wifi-ios.py')
    if not check('backend: the wifi-ios model has the net_input stub to record frames', W.MODEL.count(RX_STUB) == 1):
        return
    said = W.TEST[W.TEST.index('static unsigned said_times'):W.TEST.index('static bool said_since')]
    # every kernel unit of the shim, the radio handoff (K2) included, as test-fuxi-wifi-ios.py builds them
    source = W.unit_source(W.MODEL.replace(RX_STUB, RX_RECORD)) + said + BACKEND_MAIN
    exes = {}
    for defs, label in (((), 'fuxi'), (('-DFUXI_SALAMI=1',), 'salami')):
        path = tmp / f'backend-{label}.c'
        path.write_text(source)
        exe = tmp / f'backend-{label}'
        p = subprocess.run([CC, '-std=gnu11', '-g', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                            '-Wno-unused-parameter', '-fsanitize=address,undefined', '-pthread', *defs, '-I', str(PROBE_DIR),
                            '-I', str(WIFI_DIR), str(path), '-o', str(exe)], capture_output=True, text=True)
        check(f'backend ({label}): fuxi_wifi_ios.c with the route U backend compiles warning-free (-Wall -Wextra -Werror)',
              p.returncode == 0, p.stderr[-1500:])
        exes[label] = exe if p.returncode == 0 else None
    for name in ('opt1', 'none', 'placeholder', 'short', 'size', 'ram', 'scratch', 'unaligned', 'magic', 'self', 'ok'):
        for label, exe in exes.items():
            if exe is None:
                continue
            rc, out = run_exe(exe, args=(name,), timeout=60)
            what = {'opt1': 'fuxiwifiios=1 unchanged: same line, no /chosen lookup',
                    'ok': 'region: kmagic, kstate bit0/bit1, TX via 0x5e, RX via 0x5d to en0, full, bad slot, 1/20 ms polls'
                    }.get(name, f'{name}: one usbnet line, synthetic loopback kept')
            check(f'backend ({label}) {what}', rc == 0 and 'PASS' in out, out[-1500:])


# ---- 7. the transport over the simulators ----------------------------------------------
def transport_suite():
    t, loader, kernel = R.sim_pair(pa=0xa0000000)
    up = [frame(14 + 97 * i % 1500, i) for i in range(100)]
    down = [frame(60 + 53 * i % 1450, 100 + i) for i in range(100)]
    got_up, got_down, put, sent = [], [], 0, 0
    for _ in range(400):
        while put < len(up) and kernel.tx(up[put]) == 'ok':
            put += 1
        got_up += t.poll()
        while sent < len(down) and t.send_frame(down[sent]):
            sent += 1
        got_down += kernel.rx(7)
        if len(got_up) == len(up) and len(got_down) == len(down):
            break
    check('transport: 100 frames each way in order over the simulators; busy only from flow control, '
          'no STALL, nothing dropped on the phone (SPEC A1)',
          got_up == up and got_down == down and t.stats['busy'] > 0 and t.stats['stall'] == 0 and t.stats['resync'] == 0 and
          t.stats['phone_drops'] == 0 and loader.region.u32(0x8c) == 0 and loader.region.u32(0x90) == 0, t.stats)
    # Flow control: never 32 frames in flight; busy sends no 0x5d at all.
    t, loader, kernel = R.sim_pair()
    accepted = sum(t.send_frame(frame(100, i)) for i in range(40))
    check('transport: 32 frames go out, the next 8 are busy (no 0x5d, no drop on the phone), True again after the kernel reads',
          accepted == 32 and t.stats['busy'] == 8 and t.stats['stall'] == 0 and loader.region.u32(0x80) == 32 and
          loader.region.u32(0x8c) == 0 and loader.region.u32(0x90) == 0 and len(kernel.rx()) == 32 and
          t.send_frame(frame(100, 1)), t.stats)
    kernel.set_state(R.KSTATE_UP, False)
    t.header()
    other = loader.region.u32(0x90)
    check('transport: kstate bit0 clear in the header: busy, no 0x5d sent, not an error',
          t.send_frame(frame(60, 1)) is False and loader.region.u32(0x90) == other and t.stats['stall'] == 0)
    # The kernel goes down after the last header: that frame is taken and dropped on the phone.
    t, loader, kernel = R.sim_pair()
    t.header()
    kernel.set_state(R.KSTATE_UP, False)
    went = t.send_frame(frame(60, 2))
    t.header()
    check('transport: a kernel that went down after the header: the frame is taken and dropped there '
          '(out_stall_other, phone_drops 1), no STALL',
          went and loader.region.u32(0x90) == 1 and t.stats['phone_drops'] == 1 and t.stats['stall'] == 0, t.stats)
    # Before the launch and the handoff: zeros, nothing sent, no STALL; then the handoff.
    loader = R.LoaderRingSim(0xa0000000)
    t = R.UsbnetTransport(loader.ctrl)
    pre = (t.poll(), t.header()['loader_ok'], t.send_frame(frame(60, 1)), t.consumed)
    loader.launched = True
    mid = (t.poll(), t.header()['loader_ok'], t.send_frame(frame(60, 1)), t.consumed)
    check('transport: before the launch and before the handoff 0x5e answers zeros: no frames, not ready, '
          'nothing sent, no STALL, region untouched',
          pre == ([], False, False, None) and mid == pre and t.stats['stall'] == 0 and t.stats['not_ready'] >= 6 and
          not any(loader.region.bytes), (pre, mid, t.stats))
    loader.handoff()
    kernel = R.KernelSim(loader)
    kernel.up()
    for i in range(3):
        kernel.tx(frame(70, i))
    first = t.poll()
    check('transport: then the handoff and the kernel: frames both ways from seq 0',
          first == [frame(70, i) for i in range(3)] and t.send_frame(frame(70, 9)) and kernel.rx() == [frame(70, 9)] and
          t.consumed == 3 and t.stats['resync'] == 0, t.stats)
    # The phone reboots under the running bridge: zeros while the loader waits, then a new region
    # whose counts start at 0 again; the transport starts over (no resync, nothing skipped).
    loader.launched, loader.published = False, False
    loader.region.bytes[:] = bytes(R.SIZE)
    gap = t.poll()
    loader.launched = True
    loader.handoff()
    kernel = R.KernelSim(loader)
    kernel.up()
    for i in range(2):
        kernel.tx(frame(70, 20 + i))
    after = t.poll()
    check('transport: a phone reboot under a running bridge: nothing during it, then the new boot from seq 0',
          gap == [] and after == [frame(70, 20), frame(70, 21)] and t.consumed == 2 and t.stats['resync'] == 0 and
          t.stats['stall'] == 0, (len(after), t.stats))
    # A bridge with one thread polling and one sending, the phone in a third; all three share
    # the device lock (the simulators are not thread safe, the transport must be).
    import threading
    import time
    lock = threading.Lock()
    t, loader, kernel = R.sim_pair()
    t = R.UsbnetTransport(loader.ctrl, lock)
    ups = [frame(30 + 7 * i % 900, i) for i in range(300)]
    downs = [frame(40 + 11 * i % 800, i) for i in range(300)]
    got_up, got_down, errors, deadline = [], [], [], time.monotonic() + 60

    def guard(body):
        def run():
            try:
                body()
            except Exception as error:      # reported by the check below
                errors.append(repr(error))
        return run

    def poller():
        while len(got_up) < len(ups) and time.monotonic() < deadline:
            got_up.extend(t.poll())

    def sender():
        for f in downs:
            while not t.send_frame(f) and time.monotonic() < deadline:
                time.sleep(0.0001)

    def phone():
        put = 0
        while (put < len(ups) or len(got_down) < len(downs)) and time.monotonic() < deadline:
            with lock:
                while put < len(ups) and kernel.tx(ups[put]) == 'ok':
                    put += 1
                got_down.extend(kernel.rx())
            time.sleep(0.0001)
    threads = [threading.Thread(target=guard(f)) for f in (poller, sender, phone)]
    for th in threads:
        th.start()
    for th in threads:
        th.join(90)
    check('transport: one thread polling, one sending, the phone in a third (shared device lock): '
          '300 frames each way in order, no STALL, no drop, no resync',
          not errors and not any(th.is_alive() for th in threads) and got_up == ups and got_down == downs and
          t.stats['stall'] == 0 and t.stats['resync'] == 0 and loader.region.u32(0x8c) == 0 and
          loader.region.u32(0x90) == 0, (errors, len(got_up), len(got_down), t.stats))

    class Timeout(OSError):
        pass

    def broken(*a):
        raise Timeout(110, 'Operation timed out')
    t2 = R.UsbnetTransport(broken)
    try:
        t2.send_frame(frame(60, 1))
        raised = False
    except Timeout:
        raised = True
    check('transport: a USB error other than STALL is raised, never taken for busy', raised)
    # Resync on a bad fnv (a broken kernel) and on a bad seq.
    t, loader, kernel = R.sim_pair()
    for i in range(6):
        kernel.tx(frame(80, i))
    loader.region.write(R.TX_OFF + 3 * R.SLOT_SIZE + 8, loader.region.u32(R.TX_OFF + 3 * R.SLOT_SIZE + 8) ^ 1)
    first = t.poll()
    kernel.tx(frame(80, 6))
    after = t.poll()
    check('transport: a bad fnv returns the frames before it, resyncs to tx_head, then goes on',
          first == [frame(80, i) for i in range(3)] and t.stats['bad_fnv'] == 1 and t.stats['resync'] == 1 and
          after == [frame(80, 6)], (len(first), after, t.stats))
    t.consumed = (t.consumed + 1000) & R.M32
    kernel.tx(frame(80, 7))
    lost = t.poll()
    kernel.tx(frame(80, 8))
    again = t.poll()
    check('transport: a wrong own count (the loader refuses it, tail_bad) resyncs to tx_head',
          lost == [] and t.stats['resync'] == 2 and again == [frame(80, 8)] and loader.region.u32(0x98) >= 1, t.stats)
    # Wraparound: every index near 2^32 (a long session), both ways.
    t, loader, kernel = R.sim_pair()
    for off in (0x4c, 0x50, 0x80, 0x84):
        loader.region.write(off, 0xfffffff0)
    t = R.UsbnetTransport(loader.ctrl)
    ups = [frame(30 + i, i) for i in range(50)]
    downs = [frame(40 + i, i) for i in range(50)]
    got_up, got_down, put, sent = [], [], 0, 0
    for _ in range(200):
        while put < 50 and kernel.tx(ups[put]) == 'ok':
            put += 1
        got_up += t.poll()
        while sent < 50 and t.send_frame(downs[sent]):
            sent += 1
        got_down += kernel.rx()
        if len(got_up) == 50 and len(got_down) == 50:
            break
    check('transport: across 2^32 both ways, nothing lost or repeated, no resync, no STALL, no drop',
          got_up == ups and got_down == downs and t.stats['resync'] == 0 and loader.region.u32(0x50) == 0x22 and
          t.stats['stall'] == 0 and loader.region.u32(0x8c) == 0 and loader.region.u32(0x90) == 0, t.stats)
    # A restarted bridge picks up at host_tx_tail: no frame lost, none repeated.
    t, loader, kernel = R.sim_pair()
    for i in range(10):
        kernel.tx(frame(50, i))
    first = t.poll()
    t.header()                          # the old bridge's last request carries its count
    for i in range(10, 15):
        kernel.tx(frame(50, i))
    t3 = R.UsbnetTransport(loader.ctrl)
    rest = t3.poll()
    check('transport: a new bridge starts at the loader host_tx_tail (no loss, no repeat)',
          first == [frame(50, i) for i in range(10)] and rest == [frame(50, i) for i in range(10, 15)] and
          t3.stats['resync'] == 0, t3.stats)
    t5 = R.UsbnetTransport(loader.ctrl)
    for i in range(15, 18):
        kernel.tx(frame(50, i))
    again = t5.poll()
    check('transport: without that last request, a new bridge sees the unacknowledged frames again (at most 32)',
          again == [frame(50, i) for i in range(10, 18)] or again == [frame(50, i) for i in range(15, 18)], len(again))
    t4 = R.UsbnetTransport(loader.ctrl, in_length=192)
    check('transport: a 192-byte poll lists nothing and resyncs nothing; header() decodes',
          t4.poll() == [] and t4.stats['resync'] == 0 and t4.header()['loader_ok'] and t4.header()['kernel_up'])
    p = subprocess.run([sys.executable, '-B', str(ROOT / 'tools/fuxi_usbnet_ring.py'), '--selftest'], capture_output=True, text=True)
    check('fuxi_usbnet_ring.py --selftest', p.returncode == 0 and 'route U selftest: 40 frames' in p.stdout, p.stdout + p.stderr)


# ---- 8. mutants --------------------------------------------------------------------------
LOADER_MUTANTS = [
    ('0x5d launched rule (drop)', '!ok || !launched || !un_kernel_up(b)', '!ok || !un_kernel_up(b)'),
    ('0x5d published rule', ' if (!b) return false;\n const bool other', ' const bool other'),
    ('0x5d kmagic rule (drop)', ' return un_rd(b, UN_K_MAGIC) == UN_KMAGIC_LO && un_rd(b, UN_K_MAGIC + 4) == UN_KMAGIC_HI &&\n', ' return\n'),
    ('0x5d kstate bit0 rule (drop)', ' &&\n        (un_rd(b, UN_K_STATE) & UN_KSTATE_UP);', ';'),
    ('0x5d wValue rule (STALL)', 'if (value == UN_VALUE_OUT && length', 'if (length'),
    ('0x5d wLength min 14 (STALL)', 'length >= UN_FRAME_MIN && length <= UN_FRAME_MAX) return true;',
     'length >= UN_FRAME_MIN - 1 && length <= UN_FRAME_MAX) return true;'),
    ('0x5d wLength max 1514 (STALL)', 'length >= UN_FRAME_MIN && length <= UN_FRAME_MAX) return true;',
     'length >= UN_FRAME_MIN && length <= UN_FRAME_MAX + 1) return true;'),
    ('0x5d a STALL is counted', ' if (b) un_bump(b, UN_L_OUT_OTHER);\n', ''),
    ('0x5d a busy phone STALLs again', ' if (value == UN_VALUE_OUT && length >= UN_FRAME_MIN && length <= UN_FRAME_MAX) return true;',
     ' if (b && un_kernel_up(b) && value == UN_VALUE_OUT && length >= UN_FRAME_MIN && length <= UN_FRAME_MAX) return true;'),
    ('0x5d RX full rule (drop)', 'head - un_rd(b, UN_K_RX_TAIL) >= UN_SLOTS;', 'head - un_rd(b, UN_K_RX_TAIL) > UN_SLOTS;'),
    ('0x5d full/other counting', 'other ? UN_L_OUT_OTHER : UN_L_OUT_FULL', 'full ? UN_L_OUT_FULL : UN_L_OUT_OTHER'),
    ('0x5d a drop leaves host_tx_tail', '  un_bump(b, other ? UN_L_OUT_OTHER : UN_L_OUT_FULL);\n  return false;',
     '  un_bump(b, other ? UN_L_OUT_OTHER : UN_L_OUT_FULL);\n  un_host_tail(b, index);\n  return false;'),
    ('0x5d residue rule', ' const bool other = !ok || ', ' const bool other = '),
    ('0x5d fnv over 64-bit words', '  for (uint32_t k = 0; k < 8; k++) hash', '  for (uint32_t k = 0; k < 4; k++) hash'),
    ('0x5d frame pad mask', '  if (length - i < 4) w &= (1u << 8 * (length - i)) - 1;\n', ''),
    ('0x5d updates host_tx_tail', ' un_host_tail(b, index);\n un_bump(b, UN_L_OUT_OK);', ' un_bump(b, UN_L_OUT_OK);'),
    ('0x5e launched rule (zeros)', ' if (!launched || !b) {\n  un_zero(out, 0, length);', ' if (!b) {\n  un_zero(out, 0, length);'),
    ('0x5e published rule (zeros)', ' if (!launched || !b) {\n  un_zero(out, 0, length);', ' if (!launched) {\n  un_zero(out, 0, length);'),
    ('0x5e not ready STALLs again', '  un_zero(out, 0, length);\n  return length;', '  return 0;'),
    ('0x5e wValue rule', ' if (value != UN_VALUE_IN || length', ' if (length'),
    ('0x5e wLength min 192', 'length < UN_IN_MIN || length > UN_IN_MAX) return 0;', 'length < UN_IN_MIN - 1 || length > UN_IN_MAX) return 0;'),
    ('0x5e wLength max 4096', 'length < UN_IN_MIN || length > UN_IN_MAX) return 0;', 'length < UN_IN_MIN || length > UN_IN_MAX + 1) return 0;'),
    ('0x5e in_count', ' un_bump(b, UN_L_IN_COUNT);\n', ''),
    ('0x5e zero fill after the list', ' un_zero(out, pos, length);\n return length;', ' return length;'),
    ('TX list pad word never by 64 bits', '   for (; i + 8 < bytes; i += 8)', '   for (; i + 8 <= bytes; i += 8)'),
    ('rebuild window lower edge', ' if (head - t <= UN_SLOTS) un_wr', ' if (head - t < UN_SLOTS) un_wr'),
    ('rebuild window upper edge', ' if (head - t <= UN_SLOTS) un_wr', ' if (head - t <= UN_SLOTS + 1) un_wr'),
    ('rebuild 2^16 borrow', ' if (t > head) t -= 0x10000u;\n', ''),
    ('TX list slot len min', '  if (len < UN_FRAME_MIN || len > UN_FRAME_MAX || un_rd(b, slot + 4) != k) break;',
     '  if (len < UN_FRAME_MIN - 1 || len > UN_FRAME_MAX || un_rd(b, slot + 4) != k) break;'),
    ('TX list slot len max', '  if (len < UN_FRAME_MIN || len > UN_FRAME_MAX || un_rd(b, slot + 4) != k) break;',
     '  if (len < UN_FRAME_MIN || len > UN_FRAME_MAX + 1 || un_rd(b, slot + 4) != k) break;'),
    ('TX list slot seq', ' || un_rd(b, slot + 4) != k) break;', ') break;'),
    ('TX list exact fit', '  if (bytes > length - pos) break;', '  if (bytes >= length - pos) break;'),
    ('TX list pad mask', '   if (i + 4 == bytes && (len & 3)) w &= (1u << 8 * (len & 3)) - 1;\n', ''),
    ('ADT exactly one', ' if (found == 1) {\n  un_wr(adt', ' if (found >= 1) {\n  un_wr(adt'),
    ('ADT counts two', ' o + 4 * UN_ADT_WORDS <= size && found < 2;', ' o + 4 * UN_ADT_WORDS <= size && found < 1;'),
    ('ADT length word', "0, 0, 0, 0, 0, 16,\n", "0, 0, 0, 0, 0, 0x80000010u,\n"),
    ('ADT value word', "UN_W('O', 'L', 'D', 'R')", "UN_W('O', 'L', 'D', 'r')"),
    ('region alignment', ' if (!pa || (pa & (UN_ALIGN - 1)) || end < pa) return false;', ' if (!pa || (pa & (UN_ALIGN / 4 - 1)) || end < pa) return false;'),
    ('region zero', ' if (!pa || (pa & (UN_ALIGN - 1))', ' if ((pa & (UN_ALIGN - 1))'),
    ('region XNU RAM', ' if (pa < FG_RAM_BASE + FG_RAM_SIZE && FG_RAM_BASE < end) return false;', ' if (pa < FG_RAM_BASE + FG_RAM_SIZE && FG_RAM_BASE < pa) return false;'),
    ('region GPU scratch', ' if (pa < FG_SCRATCH + FG_SCRATCH_SIZE && FG_SCRATCH < end) return false;', ''),
]
KERNEL_MUTANTS = [
    ('kernel TX full rule', '\tif (head - tail >= FXNET_SLOTS) {', '\tif (head - tail > FXNET_SLOTS) {'),
    ('kernel TX size min', '\tif (length < FXNET_FRAME_MIN || length > FXNET_FRAME_MAX) {', '\tif (length < FXNET_FRAME_MIN - 1 || length > FXNET_FRAME_MAX) {'),
    ('kernel TX size max', '\tif (length < FXNET_FRAME_MIN || length > FXNET_FRAME_MAX) {', '\tif (length < FXNET_FRAME_MIN || length > FXNET_FRAME_MAX + 1) {'),
    ('kernel RX len min', '\tif (len < FXNET_FRAME_MIN || len > FXNET_FRAME_MAX || fxnet_rd(region, slot + 4) != tail) {',
     '\tif (len < FXNET_FRAME_MIN - 1 || len > FXNET_FRAME_MAX || fxnet_rd(region, slot + 4) != tail) {'),
    ('kernel RX len max', '\tif (len < FXNET_FRAME_MIN || len > FXNET_FRAME_MAX || fxnet_rd(region, slot + 4) != tail) {',
     '\tif (len < FXNET_FRAME_MIN || len > FXNET_FRAME_MAX + 1 || fxnet_rd(region, slot + 4) != tail) {'),
    ('kernel RX seq', ' || fxnet_rd(region, slot + 4) != tail) {', ') {'),
    ('kernel RX fnv', '\tif (check != hash) {', '\tif (0) {'),
    ('kernel RX counting', 'good ? FXNET_K_RX_FRAMES : FXNET_K_RX_BAD', 'good ? FXNET_K_RX_BAD : FXNET_K_RX_FRAMES'),
    ('kernel handoff placeholder', '\tif (same == 16) {', '\tif (same == 17) {'),
    ('kernel handoff size', '\tif (bytes != FXNET_SIZE) {', '\tif (0) {'),
    ('kernel handoff alignment', '(at & (FXNET_ALIGN - 1)) != 0', '(at & (FXNET_ALIGN / 4 - 1)) != 0'),
    ('kernel handoff XNU RAM', '\tif (at < FXNET_RAM_BASE + FXNET_RAM_SIZE && FXNET_RAM_BASE < at + FXNET_SIZE) {',
     '\tif (at < FXNET_RAM_BASE + FXNET_RAM_SIZE && FXNET_RAM_BASE < at) {'),
    ('kernel handoff GPU scratch', '\tif (at < FXNET_GPU_SCRATCH + FXNET_GPU_SCRATCH_SIZE && FXNET_GPU_SCRATCH < at + FXNET_SIZE) {', '\tif (0) {'),
    ('kernel header self_pa', ' || self != pa) {', ') {'),
    ('kernel header version', '\tif (fxnet_rd(region, FXNET_L0_VERSION) != FXNET_VERSION) {', '\tif (0) {'),
    ('kernel header layout', '\tif (fxnet_rd(region, FXNET_L0_SLOT_SIZE) != FXNET_SLOT_SIZE || fxnet_rd(region, FXNET_L0_SLOTS) != FXNET_SLOTS ||',
     '\tif (fxnet_rd(region, FXNET_L0_SLOTS) != FXNET_SLOTS ||'),
]


def mutant_suite(tmp, d):
    import shutil
    killed, survived = [], []
    for kind, mutants in (('loader', LOADER_MUTANTS), ('kernel', KERNEL_MUTANTS)):
        for name, old, new in mutants:
            text = USBNET_H if kind == 'loader' else RING_H
            if text.count(old) != 1:
                survived.append(f'{name}: pattern found {text.count(old)} times')
                continue
            mdir = tmp / ('mut-' + re.sub(r'\W+', '-', name))
            mdir.mkdir()
            nowerror = ('-Wno-error',)          # a broken rule leaves a parameter unused: still a program
            if kind == 'loader':
                (mdir / 'usbnet.h').write_text(text.replace(old, new))
                shutil.copy(PROBE_DIR / 'gpu-handoff.h', mdir / 'gpu-handoff.h')
                exe = compile_c(mdir, 'loader', LOADER_C, includes=(mdir,), sanitize=False, extra=nowerror)
                rc, out = run_exe(exe, '\n'.join(d.lops) + '\n')
                dumps_ok = all((tmp / f'loader-{i}.bin').exists() and (tmp / f'loader-{i}.bin').read_bytes() == want
                               for i, want in enumerate(d.dumps))
                caught = rc != 0 or out.splitlines() != d.lexp or not dumps_ok
            else:
                (mdir / 'fuxi_usbnet_ring.h').write_text(text.replace(old, new))
                exe = compile_c(mdir, 'kernel', KERNEL_C, includes=(mdir,), sanitize=False, extra=nowerror)
                rc, out = run_exe(exe, '\n'.join(d.kops) + '\n')
                dumps_ok = all((tmp / f'kernel-{i}.bin').exists() and (tmp / f'kernel-{i}.bin').read_bytes() == want
                               for i, want in enumerate(d.dumps))
                checks = compile_c(mdir, 'checks', KERNEL_CHECKS, includes=(mdir,), sanitize=False, extra=nowerror)
                crc, cout = run_exe(checks)
                caught = rc != 0 or out.splitlines() != d.kexp or not dumps_ok or crc != 0
            for f in tmp.glob('*-[0-9]*.bin'):
                f.unlink()
            if isinstance(exe, str):
                survived.append(f'{name}: the mutant does not compile ({exe[-200:]})')
            else:
                (killed if caught else survived).append(name)
    check(f'mutants: {len(killed)} of {len(LOADER_MUTANTS) + len(KERNEL_MUTANTS)} broken rules and bounds caught '
          f'({len(LOADER_MUTANTS)} loader, {len(KERNEL_MUTANTS)} kernel)', not survived, survived)


def main():
    with tempfile.TemporaryDirectory(prefix='fuxi-usbnet-') as t:
        tmp = Path(t)
        layout_suite(tmp)
        mk, adt_mod, adts, added = sample_adts()
        d = ring_suite(tmp, adts)
        glue_suite(tmp)
        adt_suite(tmp, mk, adt_mod, adts, added)
        ok, out = kernel_checks(tmp)
        check('kernel handoff and header checks: placeholder, 16 bytes, size, alignment, XNU RAM, scratch, magic, '
              'version, self_pa, layout', ok, out[-1500:])
        backend_suite(tmp)
        transport_suite()
        mutant_suite(tmp, d)
    bad = [n for n, ok in RESULTS if not ok]
    print(f'usbnet tests: {len(RESULTS) - len(bad)} PASS, {len(bad)} FAIL', flush=True)
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
