#!/usr/bin/env python3
"""HD1 (fuxiwlandiag), host-tested: the read-only hang window and force-wake lines.

Builds a small model with ASan/UBSan and -Werror against the production code cut
out of xnu/fuxi-wlan/fuxi_wlan.c (the HD1 defines and state, the fuxiwlandiag
parse, wl_soc_read and the HD1 block) and runs one scenario per part:
  hang:  0x01 hexdumps the first allocated type-1 segment at the CAP 0x1e offset
         (offset 0 without it), 384 bytes cut to the segment, in 64-byte rows,
         then the printable ASCII runs of 4 or more on one line; every byte is
         checked against the segment, which ASan bounds and which must not change;
         no device access at all.
  wake:  0x02 makes exactly the allowlisted reads (BAR 0x310c, BAR 0x3004, the
         window select, SoC 0x01E04040 through the window, the restore, BAR 0x310c)
         and prints the raw values; the only writes are wl_soc_read's window select
         and restore, and the window ends where it began; nothing after a latched
         fault but the plain reads; nothing before the BAR is established.
  knob:  absent or 0 = no line and no access; 1..3 accepted; anything else refused.
  lines: every line at its widest within the console's 254 bytes.
Mutants (each must be caught): a force-wake write instead of the read, a write
into the DMA segment, the dump not cut to the segment, the allowlist widened to
another register, the call in wl5_abort moved after the MHI reset.
Static: the call at every BHI ERRDBG print and before the MHI reset in
wl5_abort, before every free of wl_target; wl_target freed only there; HD1 has no
other device path. With the vendor ISC tree ($KIWI_VENDOR_SRC) and Linux ath12k
v6.19 ($ATH12K_SRC) on disk the two offsets and the window constants are
re-derived from them (each file's licence notice checked first), else skipped.
No device, no network.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
DRIVER = (ROOT / 'xnu/fuxi-wlan/fuxi_wlan.c').read_text()
CC = [os.environ.get('CC', 'cc'), '-std=c11', '-D_DEFAULT_SOURCE', '-g', '-Wall', '-Wextra', '-Werror',
      '-Wno-unused-function', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
L19 = Path(os.environ.get('ATH12K_SRC', Path.home() / 'op11-ios/src/linux-ath12k/drivers/net/wireless/ath/ath12k'))
VENDOR = Path(os.environ.get('KIWI_VENDOR_SRC', Path.home() / 'op11-ios/src/sm8550-modules/qcom/opensource/wlan'))


def between(text, start, end, keep_end=False):
    assert text.count(start) == 1, start
    body = text.split(start, 1)[1]
    assert end in body, end
    body = body.split(end, 1)[0]
    return start + body + (end if keep_end else '')


def production(driver):
    defs = between(driver, '/* HD1 (fuxiwlandiag, docs-lanes', 'static unsigned wlhd_cap_len;\n', True)
    parse = between(driver, '\twlhd_bits = 0; wlhd_cap_off_seen', '\tif (wl_w14) {')
    soc = between(driver, 'static int\nwl_soc_read(uint32_t off, uint32_t *val)\n{', '\n}\n', True)
    block = between(driver, '/* ---- HD1 (fuxiwlandiag): read-only', '/* ---- end HD1 ---- */')
    assert block.count('__asm__ volatile("dsb sy" ::: "memory");') == 1
    block = block.replace('__asm__ volatile("dsb sy" ::: "memory");', 'model_barrier();')
    assert '__asm__' not in block
    return defs + soc + block + 'static void\nhd_parse(void)\n{\n' + parse + '}\n'


MODEL = r'''
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char logbuf[1 << 16];
static size_t loglen;
static void fail(const char *m) { fprintf(stderr, "MODEL FAIL: %s\n", m); fputs(logbuf, stderr); exit(3); }
static void
test_printf(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(logbuf + loglen, sizeof(logbuf) - loglen, fmt, ap);
	va_end(ap);
	assert(n >= 0 && (size_t)n < sizeof(logbuf) - loglen);
	loglen += (size_t)n;
	for (size_t i = loglen - (size_t)n; i < loglen; i++) {
		if (logbuf[i] != '\n') { continue; }
		size_t s = i;
		while (s && logbuf[s - 1] != '\n') { s--; }
		if (strncmp(logbuf + s, "FUXI WLAN: ", 11)) { fail("a line without the FUXI WLAN prefix"); }
		if (i - s > 254) { fail("a FUXI WLAN line longer than 254 bytes (console cut)"); }
	}
}
#define printf test_printf

typedef uintptr_t vm_offset_t;
typedef size_t vm_size_t;
struct wl_dma { vm_offset_t va; vm_size_t size; uint64_t pa, iova; bool nc; };
#define WL_BAR0 0x60400000u
#define WL_TARGET_MAX 52u
static unsigned fw_faults;
static bool wl_mhi_valid, wl_variant_invalid;
static struct wl_dma wl_target[WL_TARGET_MAX];
static uint32_t wl_target_size[WL_TARGET_MAX], wl_target_type[WL_TARGET_MAX];
static unsigned wl_target_n;

/* fuxiwlandiag only: present or absent, an int */
static bool arg_present;
static int arg_value;
static bool
PE_parse_boot_argn(const char *name, void *buf, int size)
{
	if (strcmp(name, "fuxiwlandiag") || size != (int)sizeof(int)) { fail("HD1 parsed another knob"); }
	if (!arg_present) { return false; }
	memcpy(buf, &arg_value, sizeof(int));
	return true;
}

/* BAR0: the window register (read back), PCIE_SOC_WAKE at 0x3004 and, through
 * window 0x3c only, PCIE_SCRATCH_0 at SoC 0x01E04040. Every other access fails. */
#define WIN_INIT 0x40000010u
static uint32_t bar_win = WIN_INIT, soc_wake = 0x1u, scratch0 = 0x2u;
static unsigned n_reads, n_win_writes, n_barriers, fault_on_scratch;
static char trace[4096];
static size_t tlen;
static void ev(const char *fmt, uint64_t a, uint32_t v)
{
	int n = snprintf(trace + tlen, sizeof(trace) - tlen, fmt, (unsigned long long)a, v);
	assert(n >= 0 && (size_t)n < sizeof(trace) - tlen);
	tlen += (size_t)n;
}
static void model_barrier(void) { n_barriers++; }
static uint32_t
wl_prd(uint64_t pa)
{
	n_reads++;
	if (pa == WL_BAR0 + 0x310cull) { ev("R%llx=%x ", pa - WL_BAR0, bar_win); return bar_win; }
	if (pa == WL_BAR0 + 0x3004ull) { ev("R%llx=%x ", pa - WL_BAR0, soc_wake); return soc_wake; }
	if (pa == WL_BAR0 + 0x80000ull + 0x4040ull) {
		if (bar_win != ((WIN_INIT & ~0x3fu) | 0x40000000u | 0x3cu)) { fail("window read with another window selected"); }
		ev("Rsoc%llx=%x ", 0x01e04040ull + 0 * pa, scratch0);
		if (fault_on_scratch) { fw_faults++; return ~0u; }
		return scratch0;
	}
	fail("read outside the HD1 allowlist");
	return 0;
}
static void
model_write(uint64_t pa, uint32_t v, bool contain)
{
	if (pa != WL_BAR0 + 0x310cull) { fail("register write: only the window register may be written"); }
	if (v != WIN_INIT && v != ((WIN_INIT & ~0x3fu) | 0x40000000u | 0x3cu)) { fail("window register written with another value"); }
	if (!contain && v == WIN_INIT && bar_win == WIN_INIT) { fail("window rewritten without a select"); }
	ev("W%llx=%x ", pa - WL_BAR0, v);
	bar_win = v; n_win_writes++;
}
static void wl_pwr(uint64_t pa, uint32_t v) { if (fw_faults) { return; } model_write(pa, v, false); }  /* fw_access: refused after a fault */
static int wl_contain_wr(uint64_t pa, uint32_t v) { model_write(pa, v, true); return 0; }
static uint32_t wl_mrd(uint32_t off) { return wl_prd((uint64_t)WL_BAR0 + off); }
static void wl_mwr(uint32_t off, uint32_t v) { wl_pwr((uint64_t)WL_BAR0 + off, v); }

#include "hd-prod.inc"

/* ---- scenarios ---- */
static uint8_t *segs[WL_TARGET_MAX], *shadow[WL_TARGET_MAX];
static void
reset(void)
{
	for (unsigned i = 0; i < WL_TARGET_MAX; i++) {
		free(segs[i]); free(shadow[i]); segs[i] = shadow[i] = NULL;
		memset(&wl_target[i], 0, sizeof(wl_target[i])); wl_target_size[i] = wl_target_type[i] = 0;
	}
	wl_target_n = 0; fw_faults = 0; wl_mhi_valid = true; wl_variant_invalid = false;
	bar_win = WIN_INIT; soc_wake = 0x1u; scratch0 = 0x2u; fault_on_scratch = 0;
	n_reads = n_win_writes = n_barriers = 0; tlen = 0; trace[0] = 0; loglen = 0; logbuf[0] = 0;
	wlhd_bits = 0; wlhd_cap_off_seen = wlhd_cap_len_seen = false; wlhd_cap_off = 0; wlhd_cap_len = 0;
}
/* segment i: exactly size bytes on the heap (ASan bounds every read), a copy to prove it never changes */
static void
seg(unsigned i, uint32_t type, uint32_t size, bool alloc)
{
	wl_target_type[i] = type; wl_target_size[i] = size;
	if (i + 1 > wl_target_n) { wl_target_n = i + 1; }
	if (!alloc) { return; }
	segs[i] = malloc(size); shadow[i] = malloc(size); assert(segs[i] && shadow[i]);
	for (uint32_t k = 0; k < size; k++) { segs[i][k] = (uint8_t)(0x80u | (k * 7u + i * 13u + 1u));  /* never printable */ }
	wl_target[i] = (struct wl_dma){ (vm_offset_t)segs[i], size, 0x900000000ull + i * 0x100000u, 0xa0000000ull + i * 0x100000u, true };
}
static void
seal(void)
{
	for (unsigned i = 0; i < WL_TARGET_MAX; i++) { if (segs[i]) { memcpy(shadow[i], segs[i], wl_target_size[i]); } }
}
static void
unchanged(void)
{
	for (unsigned i = 0; i < WL_TARGET_MAX; i++) {
		if (segs[i] && memcmp(shadow[i], segs[i], wl_target_size[i])) { fail("HD1 wrote into a DMA segment"); }
	}
}
static void
expect(const char *text)
{
	if (!strstr(logbuf, text)) { fprintf(stderr, "missing: %s\n", text); fail("expected line missing"); }
}
static unsigned
count(const char *text)
{
	unsigned n = 0;
	for (const char *p = logbuf; (p = strstr(p, text)); p++) { n++; }
	return n;
}
/* the rows the dump must print for segment s from off, len bytes */
static void
expect_rows(unsigned s, uint32_t off, unsigned len)
{
	static const char hex[] = "0123456789abcdef";
	for (unsigned r = 0; r < len; r += 64) {
		char want[256];
		int o = snprintf(want, sizeof(want), "FUXI WLAN: HD1 hex +%#x: ", off + r);
		for (unsigned i = 0; i < 64 && r + i < len; i++) {
			uint8_t b = segs[s][off + r + i];
			if (i && !(i & 3)) { want[o++] = ' '; }
			want[o++] = hex[b >> 4]; want[o++] = hex[b & 15];
		}
		want[o++] = '\n'; want[o] = 0;
		expect(want);
	}
	if (count("FUXI WLAN: HD1 hex +") != (len + 63) / 64) { fail("row count"); }
}

static void
s_hang(void)
{
	reset();
	arg_present = true; arg_value = 1; hd_parse();
	assert(wlhd_bits == 1 && !wl_variant_invalid);
	expect("FUXI WLAN: HD1 selected fuxiwlandiag 0x1: hang window hexdump yes, force-wake reads no; read-only");
	/* W13 order: a type-0 segment, an unsupported type-1 left unallocated (addr 0), then the type-1 we gave */
	seg(0, 0, 0x8000, true); seg(1, 1, 0x4000, false); seg(2, 1, 0x40000, true); seg(3, 1, 0x40000, true); seg(4, 3, 0x2000, true);
	const uint32_t off = 0x1f00;
	memcpy(segs[2] + off + 8, "cmnos_thread_patch.c", 20);
	memcpy(segs[2] + off + 40, "\x01WLAN_MODE\x00ab\x7f" "xyz!", 17);
	wlhd_cap_off = off; wlhd_cap_off_seen = true; wlhd_cap_len = 384; wlhd_cap_len_seen = true;
	seal(); loglen = 0; logbuf[0] = 0;
	wl_hd_diag("W15 MODE timeout");
	unchanged();
	if (n_reads || n_win_writes) { fail("the hang part touched the device"); }
	if (n_barriers != 1) { fail("no barrier before the loads"); }
	expect("FUXI WLAN: HD1 hang W15 MODE timeout: segment 2 type 1 size 262144 IOVA 0xa0200000; offset 0x1f00 (cap 0x1e), "
	    "cap length 384; dump 384 bytes\n");
	expect_rows(2, off, 384);
	expect("FUXI WLAN: HD1 ascii 3 runs 33 chars: cmnos_thread_patch.c|WLAN_MODE|xyz!\n");
	if (count("FUXI WLAN: HD1 ascii ") != 1 || count("FUXI WLAN: HD1 wake") != 0) { fail("ascii line count / wake line without 0x02"); }
	/* no cap 0x1e: offset 0; a window that runs past the segment end: cut to the segment */
	reset(); wlhd_bits = 1; seg(0, 1, 0x1000, true); seal();
	wl_hd_diag("W13 RESPOND_MEM");
	expect("segment 0 type 1 size 4096 IOVA 0xa0000000; offset 0 (no cap 0x1e, 0), cap length 0 (no cap 0x1f); dump 384 bytes");
	expect_rows(0, 0, 384); unchanged();
	reset(); wlhd_bits = 1; seg(0, 1, 0x1000, true); wlhd_cap_off = 0x1000 - 100; wlhd_cap_off_seen = true; seal();
	wl_hd_diag("cut");
	expect("dump 100 bytes"); expect_rows(0, 0x1000 - 100, 100); unchanged();
	reset(); wlhd_bits = 1; seg(0, 1, 0x1000, true); wlhd_cap_off = 0x1000; wlhd_cap_off_seen = true; seal();
	wl_hd_diag("past");
	expect("FUXI WLAN: HD1 hang past: offset 0x1000 not inside the segment; nothing read\n");
	if (count("HD1 hex") || count("HD1 ascii")) { fail("dumped past the segment"); }
	/* no type-1 segment allocated (before QMI, or only unallocated ones) */
	reset(); wlhd_bits = 1; seg(0, 0, 0x1000, true); seg(1, 1, 0x1000, false);
	wl_hd_diag("M6 step 18");
	expect("FUXI WLAN: HD1 hang M6 step 18: no type-1 segment allocated (requested 2); nothing read\n");
	/* a segment not Normal-NC (allocation failed halfway) is not a dump source */
	reset(); wlhd_bits = 1; seg(0, 1, 0x1000, true); wl_target[0].nc = false;
	wl_hd_diag("nc"); expect("no type-1 segment allocated (requested 1)");
	/* all printable: 384 characters of runs, cut to the line, counts of every run */
	reset(); wlhd_bits = 1; seg(0, 1, 0x1000, true);
	for (unsigned k = 0; k < 0x1000; k++) { segs[0][k] = (uint8_t)('A' + k % 26); }
	seal(); wl_hd_diag("0123456789012345678901234567890123456789-overlong-label");
	expect("FUXI WLAN: HD1 ascii 1 runs 384 chars (cut): ABCDEFGHIJ");
	expect("FUXI WLAN: HD1 hang 0123456789012345678901234567890123456789: segment");
	/* runs of exactly 3 are not printed, 4 are */
	reset(); wlhd_bits = 1; seg(0, 1, 0x1000, true); memset(segs[0], 0, 0x1000);
	memcpy(segs[0], "abc\0defg\0hi\0jklmn", 18); seal(); wl_hd_diag("runs");
	expect("FUXI WLAN: HD1 ascii 2 runs 9 chars: defg|jklmn\n");
	reset(); wlhd_bits = 1; seg(0, 1, 0x1000, true); memset(segs[0], 0xff, 0x1000); seal(); wl_hd_diag("none");
	expect("FUXI WLAN: HD1 ascii 0 runs 0 chars: -\n");
	printf("FUXI WLAN: model hang ok\n");
}

static void
s_wake(void)
{
	reset();
	arg_present = true; arg_value = 2; hd_parse(); assert(wlhd_bits == 2);
	seg(0, 1, 0x1000, true); seal(); loglen = 0; logbuf[0] = 0;
	soc_wake = 0x1u; scratch0 = 0x2u;
	wl_hd_diag("W28 scan COMPLETED timeout");
	if (strcmp(trace, "R310c=40000010 R3004=1 R310c=40000010 W310c=4000003c R310c=4000003c Rsoc1e04040=2 "
	    "W310c=40000010 R310c=40000010 R310c=40000010 ")) { fprintf(stderr, "trace %s\n", trace); fail("access sequence"); }
	if (bar_win != WIN_INIT || n_win_writes != 2) { fail("window not restored"); }
	unchanged();
	expect("FUXI WLAN: HD1 wake W28 scan COMPLETED timeout: soc-wake 0x3004 0x1, scratch0 0x1e04040 0x2 (umac awake); "
	    "win 0x40000010->0x40000010 restored; faults 0; read-only\n");
	if (count("HD1 hang") || count("HD1 hex")) { fail("hang lines without 0x01"); }
	/* raw values as read: 0x1 = down (vendor if_pci.c:4022), other */
	reset(); wlhd_bits = 2; scratch0 = 0x1u; soc_wake = 0; wl_hd_diag("a");
	expect("soc-wake 0x3004 0, scratch0 0x1e04040 0x1 (umac down)");
	reset(); wlhd_bits = 2; scratch0 = 0xdeadbeefu; soc_wake = 0xffffffffu; wl_hd_diag("b");
	expect("soc-wake 0x3004 0xffffffff, scratch0 0x1e04040 0xdeadbeef (other)");
	/* a fault latched before: wl_soc_read touches nothing, no write at all */
	reset(); wlhd_bits = 2; fw_faults = 1; wl_hd_diag("contain before MHI reset");
	if (n_win_writes || strcmp(trace, "R310c=40000010 R3004=1 R310c=40000010 ")) { fprintf(stderr, "trace %s\n", trace); fail("access after a latched fault"); }
	expect("scratch0 0x1e04040 0xffffffff (not read) (unknown); win 0x40000010->0x40000010 restored; faults 0;");
	/* a fault on the window read: the restore still runs (containment write), the read is reported failed */
	reset(); wlhd_bits = 2; fault_on_scratch = 1; wl_hd_diag("f");
	if (bar_win != WIN_INIT) { fail("window not restored after a fault"); }
	expect("(not read) (unknown); win 0x40000010->0x40000010 restored; faults 1;");
	/* BAR/MHI not established (M4 step 16): no access */
	reset(); wlhd_bits = 2; wl_mhi_valid = false; wl_hd_diag("M4 step 16");
	if (n_reads || n_win_writes) { fail("access before the BAR was established"); }
	expect("FUXI WLAN: HD1 wake M4 step 16: not read (BAR/MHI not established)\n");
	/* both parts, widest values and label: within 254 bytes (test_printf fails longer lines) */
	reset(); arg_value = 3; hd_parse(); assert(wlhd_bits == 3);
	seg(51, 1, 0xffffffffu, false); wl_target_n = WL_TARGET_MAX; seg(50, 1, 0x1000, true); wl_target[50].iova = ~0ull;
	wlhd_cap_off = 0x1000 - 1; wlhd_cap_off_seen = true; wlhd_cap_len = 65535; wlhd_cap_len_seen = true;
	soc_wake = scratch0 = 0xffffffffu; fault_on_scratch = 1;
	wl_hd_diag("0123456789012345678901234567890123456789");
	expect("FUXI WLAN: HD1 hang 0123456789012345678901234567890123456789: segment 50 type 1 size 4096 IOVA 0xffffffffffffffff;");
	expect("FUXI WLAN: HD1 wake 0123456789012345678901234567890123456789: soc-wake 0x3004 0xffffffff, scratch0 0x1e04040 0xffffffff (not read)");
	printf("FUXI WLAN: model wake ok\n");
}

static void
s_knob(void)
{
	reset(); arg_present = false; hd_parse();
	if (wlhd_bits || wl_variant_invalid || loglen) { fail("absent knob"); }
	seg(0, 1, 0x1000, true); seal(); wl_hd_diag("contain before MHI reset");
	if (loglen || n_reads || n_win_writes || n_barriers) { fail("HD1 printed or touched something without the knob"); }
	reset(); arg_present = true; arg_value = 0; hd_parse();
	if (wlhd_bits || wl_variant_invalid || loglen) { fail("knob 0"); }
	wl_hd_diag("x"); if (loglen || n_reads) { fail("knob 0 printed"); }
	const int bad[] = { 4, 7, 0x10, -1, 0x7fffffff };
	for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		reset(); arg_present = true; arg_value = bad[i]; hd_parse();
		if (wlhd_bits || !wl_variant_invalid) { fail("bad knob accepted"); }
		expect("FUXI WLAN: HD1 fuxiwlandiag "); expect(" refused: bits 0x3 only\n");
	}
	for (int v = 1; v <= 3; v++) {
		reset(); arg_present = true; arg_value = v; hd_parse();
		if (wlhd_bits != (unsigned)v || wl_variant_invalid) { fail("good knob refused"); }
	}
	printf("FUXI WLAN: model knob ok\n");
}

int
main(int argc, char **argv)
{
	if (argc != 2) { return 2; }
	if (!strcmp(argv[1], "hang")) { s_hang(); }
	else if (!strcmp(argv[1], "wake")) { s_wake(); }
	else if (!strcmp(argv[1], "knob")) { s_knob(); }
	else { return 2; }
	fputs(logbuf, stdout);
	reset();
	return 0;
}
'''


def build(tmp, name, driver):
    d = Path(tmp) / name
    d.mkdir()
    (d / 'hd-prod.inc').write_text(production(driver))
    (d / 'model.c').write_text(MODEL)
    exe = d / 'model'
    r = subprocess.run(CC + ['-I', str(d), str(d / 'model.c'), '-o', str(exe)], capture_output=True, text=True)
    return exe if r.returncode == 0 else None, r.stderr


def run(exe, scenario):
    return subprocess.run([str(exe), scenario], capture_output=True, text=True)


def static_checks():
    block = between(DRIVER, '/* ---- HD1 (fuxiwlandiag): read-only', '/* ---- end HD1 ---- */')
    code = re.sub(r'/\*.*?\*/', '', block, flags=re.S)
    # One device path: wl_mrd for the two BAR registers, wl_soc_read for the window; nothing that writes or sends.
    for banned in ('wl_mwr', 'wl_pwr', 'wl_contain_wr', 'wl_soc_write', 'wl_win(', 'wl_mwr64', 'fw_safe_wr32', 'ts_wr',
                   'wl_ce_', 'wl12_send', 'wl_dma_civac', 'wl_dma_clean', 'wl_dma_free', 'wl_dma_alloc', 'flush_dcache'):
        assert banned not in code, banned
    assert code.count('wl_mrd(') == 1 and code.count('wl_soc_read(') == 1
    assert code.count('wlhd_rd(') == 5   # the definition and the four reads
    assert 'return window ? off == WLHD_SCRATCH0 : (off == WLHD_SOC_WAKE || off == WLHD_WINREG);' in code
    assert '#define WLHD_SOC_WAKE 0x3004u' in block and '#define WLHD_SCRATCH0 0x01e04040u' in block
    assert '#define WLHD_WINREG 0x310cu' in block and '#define WLHD_LEN 384u' in block
    # Every BHI ERRDBG print of the lane is followed (or, M6f, preceded) by the call in the same function.
    sites = {
        'M4 step 16': between(DRIVER, '\tcase 16: { /* MHI + BHI, read-only */', '\treturn 0xff;'),
        'wl5_status': between(DRIVER, 'static void\nwl5_status(unsigned step)\n{', '\n}\n'),
        'wl_wlfw_diagnostics': between(DRIVER, 'static void\nwl_wlfw_diagnostics(const char *label)\n{', '\n}\n'),
        'wl28_crash': between(DRIVER, 'static void\nwl28_crash(const char *where)\n{', '\n}\n'),
    }
    for name, body in sites.items():
        assert 'ERRDBG' in body or 'errdbg' in body, name
        last = max(body.rfind('ERRDBG'), body.rfind('errdbg'))
        assert body.count('wl_hd_diag(') == 1 and body.index('wl_hd_diag(') > last, name
    boot = between(DRIVER, 'static void\nwl_boot_logs(void)\n{', '\n}\n')
    assert boot.index('wl_hd_diag("M6f boot logs");') < boot.index('PCIE_BHI_ERRDBG2')
    bhi = between(DRIVER, 'static uint32_t\nwl_bhi_diag(unsigned sample)\n{', '\n}\n')
    assert bhi.index('wl5_status(22);') < bhi.index('ERRDBG')   # covered by wl5_status
    # Every function with a string naming ERRDBG is one of the sites above (or wl17_errdec, inside wl_wlfw_diagnostics).
    starts = [(m.start(), m[1]) for m in re.finditer(r'\n(\w+)\([^;{]*\)\n\{\n', DRIVER)]
    errdbg_fns = set()
    code_only = re.sub(r'/\*.*?\*/', lambda m: ' ' * len(m[0]), DRIVER, flags=re.S)
    for m in re.finditer(r'"[^"\n]*(?:ERRDBG|errdbg)[^"\n]*"', code_only):
        errdbg_fns.add(max((s for s in starts if s[0] < m.start()), key=lambda s: s[0])[1])
    assert errdbg_fns == {'wl4_step', 'wl_boot_logs', 'wl5_status', 'wl_bhi_diag', 'wl_wlfw_diagnostics', 'wl28_crash',
                          'wl17_errdec'}, errdbg_fns
    assert 'wl17_errdec(label, ee, code, d1, d2, d3); }\n\twl_hd_diag(label);' in sites['wl_wlfw_diagnostics']
    # wl5_abort: the call before the MHI reset write and before every free; wl_target freed nowhere else.
    abort = between(DRIVER, 'static void\nwl5_abort(unsigned step)\n{', '\n}\n')
    call = abort.index('wl_hd_diag("contain before MHI reset");')
    assert call < abort.index('wl_contain_wr((uint64_t)WL_BAR0 + 0x38, 2)') < abort.index('wl_dma_free(&wl_target[i])')
    assert abort.index('wl28_crash("contain before MHI reset");') < call
    assert call < abort.index('wl26_free();') and call < abort.index('wl_dma_free(')
    assert DRIVER.count('wl_dma_free(&wl_target[i])') == 1 and DRIVER.count('wl_dma_free(&wl_target') == 1
    # A type-1 allocation is never replaced: W12 refuses an allocated slot and a duplicate REQUEST_MEM.
    assert 'if (wl_target[i].va || wl_dma_alloc(&wl_target[i], wl_target_size[i]) ||' in DRIVER
    assert 'if (wl12_requested) { return wl_w11_fail("duplicate REQUEST_MEM; no reprovision"); }' in DRIVER
    # The knob: parsed once, the only writer of wlhd_bits besides its reset; the CAP TLVs are stored, the lines kept.
    decl = DRIVER.index('static void wl_hd_diag(const char *where);')
    assert decl < min(m.start() for m in re.finditer(r'\bwl_hd_diag\("', DRIVER))
    assert DRIVER.count('"fuxiwlandiag"') == 1 and len(re.findall(r'\bwlhd_bits = ', DRIVER)) == 2
    assert 'wlhd_cap_off = wl_get32(v); wlhd_cap_off_seen = true;' in DRIVER
    assert 'wlhd_cap_len = wl_get16(v); wlhd_cap_len_seen = true;' in DRIVER
    assert 'printf("FUXI WLAN: W14 cap hang_data_addr_offset %#x (reported only)\\n", wl_get32(v)); break;' in DRIVER
    for c in (chr(0x2013), chr(0x2014)):
        assert c not in block
    print('static: HD1 at every BHI ERRDBG print and before the MHI reset (before every free of wl_target, freed only '
          'in wl5_abort), one read path per register, no write, doorbell or send, the knob parsed once')


def notice(path, isc):
    head = path.read_text(errors='replace')[:3000]
    if isc:
        assert 'Permission to use, copy, modify, and/or distribute this software' in head, path
    else:
        assert 'SPDX-License-Identifier: BSD-3-Clause-Clear' in head, path
    assert 'GPL' not in head, path
    return path.read_text(errors='replace')


def line_of(text, needle):
    assert text.count(needle) >= 1, needle
    return text[:text.index(needle)].count('\n') + 1


def sources():
    """The offsets re-derived from the ISC vendor host (qca-wifi-host-cmn) and from L19, when on disk."""
    cmn = VENDOR / 'qca-wifi-host-cmn'
    if (cmn / 'hif/src/pcie/if_pci.h').exists():
        h = notice(cmn / 'hif/src/pcie/if_pci.h', True)
        c = notice(cmn / 'hif/src/pcie/if_pci.c', True)
        io = notice(cmn / 'hif/src/hif_io32.h', True)
        assert re.search(r'#define PCIE_REG_WAKE_UMAC_OFFSET 0x3004\n', h)
        assert re.search(r'#define PCIE_SOC_PCIE_REG_PCIE_SCRATCH_0_SOC_PCIE_REG \(0x01E04000 \+ 0x40\)\n', h)
        assert '#define HIF_POLL_UMAC_WAKE 0x2\n' in c
        assert 'hif_write32_mb(scn, scn->mem + PCIE_REG_WAKE_UMAC_OFFSET, 1);' in c
        assert 'PCIE_SOC_PCIE_REG_PCIE_SCRATCH_0_SOC_PCIE_REG);\n\t\tif (value == HIF_POLL_UMAC_WAKE)' in c
        for k, v in (('MAX_UNWINDOWED_ADDRESS', '0x80000'), ('WINDOW_REG_ADDRESS', '0x310C'), ('WINDOW_SHIFT', '19'),
                     ('WINDOW_VALUE_MASK', '0x3F'), ('WINDOW_RANGE_MASK', '0x7FFFF')):
            assert re.search(rf'#define {k} {v}\n', io), k
        assert re.search(r'defined\(QCA_WIFI_KIWI\)\n#define WINDOW_ENABLE_BIT 0x40000000\n', io)
        assert 'ret = qdf_ioread32(sc->mem + WINDOW_START +\n\t\t       (offset & WINDOW_RANGE_MASK));' in io
        assert (0x01E04040 >> 19) & 0x3f == 0x3c and 0x01E04040 & 0x7ffff == 0x4040 and 0x3004 < 0x80000
        print(f'vendor: PCIE_REG_WAKE_UMAC_OFFSET 0x3004 if_pci.h:{line_of(h, "#define PCIE_REG_WAKE_UMAC_OFFSET")}, '
              f'SCRATCH_0 0x01E04040 if_pci.h:{line_of(h, "#define PCIE_SOC_PCIE_REG_PCIE_SCRATCH_0")}, poll 0x2 '
              f'if_pci.c:{line_of(c, "#define HIF_POLL_UMAC_WAKE")}, windowed read hif_io32.h:'
              f'{line_of(io, "static inline uint32_t hif_read32_mb_reg_window")} (ISC notices checked)')
    else:
        print(f'vendor re-derivation SKIPPED: no vendor wlan tree at {VENDOR} (set KIWI_VENDOR_SRC)')
    if (L19 / 'pci.h').exists():
        h = notice(L19 / 'pci.h', False)
        c = notice(L19 / 'pci.c', False)
        assert re.search(r'#define PCIE_SOC_WAKE_PCIE_LOCAL_REG\t\t0x3004\n', h)
        assert re.search(r'#define PCIE_SCRATCH_0_SOC_PCIE_REG\t\t0x4040\n', h)
        assert 'PCIE_SCRATCH_0_SOC_PCIE_REG' not in c   # defined, never used
        assert '#define WINDOW_REG_ADDRESS\t\t0x310c\n' in c and '#define WINDOW_START\t\t\t0x80000\n' in c
        assert '#define WINDOW_ENABLE_BIT\t\t0x40000000\n' in c
        wcn = between(c, '\tcase WCN7850_DEVICE_ID:', '\t\tbreak;')
        assert 'ab->static_window_map = false;' in wcn
        assert 'ath12k_pci_write32(ab, PCIE_SOC_WAKE_PCIE_LOCAL_REG, 1);' in c
        print(f'L19: PCIE_SOC_WAKE_PCIE_LOCAL_REG 0x3004 pci.h:{line_of(h, "#define PCIE_SOC_WAKE_PCIE_LOCAL_REG")}, '
              f'PCIE_SCRATCH_0_SOC_PCIE_REG 0x4040 (BAR offset, unused) pci.h:{line_of(h, "#define PCIE_SCRATCH_0_SOC_PCIE_REG")}, '
              f'WCN7850 dynamic window pci.c:{line_of(c, "ab->static_window_map = false;")} (BSD-3-Clause-Clear checked)')
    else:
        print(f'L19 re-derivation SKIPPED: no ath12k tree at {L19} (set ATH12K_SRC)')


MUTANTS = [
    ('force-wake write instead of the read',
     '\telse { v = wl_mrd(off); *err = fw_faults != before; }',
     '\telse { if (off == WLHD_SOC_WAKE) { wl_mwr(off, 1); } v = wl_mrd(off); *err = fw_faults != before; }'),
    ('a write into the DMA segment',
     'for (unsigned i = 0; i < len; i++) { buf[i] = src[i]; }',
     'for (unsigned i = 0; i < len; i++) { buf[i] = src[i]; ((volatile uint8_t *)src)[i] = 0; }'),
    ('the dump not cut to the segment',
     'const unsigned len = off < size ? (size - off < WLHD_LEN ? size - off : WLHD_LEN) : 0;',
     'const unsigned len = WLHD_LEN;'),
    ('the allowlist widened to the MHI control register',
     'return window ? off == WLHD_SCRATCH0 : (off == WLHD_SOC_WAKE || off == WLHD_WINREG);',
     'return window ? off == WLHD_SCRATCH0 : (off == WLHD_SOC_WAKE || off == WLHD_WINREG || off == 0x38u);'),
]


def mutants(tmp):
    global DRIVER
    caught = 0
    for i, (name, old, new) in enumerate(MUTANTS):
        assert DRIVER.count(old) == 1, name
        mutant = DRIVER.replace(old, new)
        if name.startswith('the allowlist'):
            mutant = mutant.replace('const uint32_t win1 = wlhd_rd(WLHD_WINREG, false, &e_win1);',
                                    'const uint32_t win1 = wlhd_rd(WLHD_WINREG, false, &e_win1); (void)wlhd_rd(0x38u, false, &e_win1);')
        exe, err = build(tmp, f'mutant{i}', mutant)
        dead = exe is None or any(run(exe, s).returncode != 0 for s in ('hang', 'wake', 'knob'))
        assert dead, f'mutant survived: {name}'
        caught += 1
    # The wl5_abort order is a static rule: the call moved after the reset write must fail static_checks.
    saved = DRIVER
    abort = between(DRIVER, 'static void\nwl5_abort(unsigned step)\n{', '\n}\n')
    moved = abort.replace('\twl_hd_diag("contain before MHI reset");  /* HD1: read-only, before the reset and every free; '
                          'silent without fuxiwlandiag */\n', '')
    moved = moved.replace('\t/* PCI command upper word is W1C status', '\twl_hd_diag("contain before MHI reset");\n'
                          '\t/* PCI command upper word is W1C status')
    assert moved != abort
    DRIVER = saved.replace(abort, moved)
    try:
        static_checks()
    except AssertionError:
        caught += 1
    else:
        raise AssertionError('mutant survived: the call after the MHI reset')
    finally:
        DRIVER = saved
    print(f'mutants: {caught} of {len(MUTANTS) + 1} caught (force-wake write, DMA write, uncut dump, widened allowlist, '
          'call after the MHI reset)')


def main():
    static_checks()
    sources()
    with tempfile.TemporaryDirectory(prefix='fuxi-wlan-hd-') as tmp:
        exe, err = build(tmp, 'prod', DRIVER)
        assert exe, err
        for s in ('hang', 'wake', 'knob'):
            r = run(exe, s)
            assert r.returncode == 0, (s, r.stderr)
            assert f'FUXI WLAN: model {s} ok' in r.stdout, s
            print(f'scenario {s}: ok')
        mutants(tmp)
    print('HD1 host tests passed')


if __name__ == '__main__':
    main()
