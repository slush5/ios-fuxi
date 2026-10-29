#!/usr/bin/env python3
"""Host test of the loader USB monitor re-attach and of its request 0x59 record (offline,
host cc, no phone, no pyusb).

1. shim/fuxi-probe/usb-reattach.h compiled unchanged: every restart trigger (a USB reset or
   a disconnect while ep0 is out of SETUP, three usb_poll guard hits in a row, ep0 out of
   SETUP for more than 2 s), what must not trigger, restart spacing, counting by cause, a
   failed usb_start, the cpu0 handover, the event rings and the 0x59 reply layout.
2. probe.c's own ud_epcmd, ud_start, ud_start_setup, ud_stall, ud_event, usb_poll and
   usb_reattach (pasted from the file, with its FUXI_USB_* block, register defines and ud
   state) on fake DWC3 registers, driven like the cpu@100 loop (usb_poll, then
   usb_reattach): a restart never runs inside the usb_poll event loop, usb_poll acks
   GEVNTCOUNT before usb_start runs again, each trigger restarts once and is counted under
   its cause; FUXI_USB_REATTACH=0 and the Fuxi build never restart.
3. probe.c text: usb_start is called only by probe_main and usb_reattach, usb_reattach only by
   the cpu@100 loop right after usb_poll; request 0x59 writes nothing and reads exactly
   DSTS, DCTL, GSTS, GUSB2PHYCFG, GEVNTADRLO/HI/SIZ; build defaults per board.
4. A 0x59 reply built by ura_reply decoded by tools/fuxi-xnu-monitor.py usb_diag_decode.
"""
import ast
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PROBE_DIR = ROOT / 'shim/fuxi-probe'
PROBE = (PROBE_DIR / 'probe.c').read_text()
CC = os.environ.get('CC', 'cc')
FLAGS = ['-std=gnu11', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
         '-Wno-unused-variable', '-fno-omit-frame-pointer']
SAN = ['-fsanitize=address,undefined', '-fno-sanitize-recover=all']
RESULTS = []


def check(name, ok, detail=''):
    RESULTS.append((name, bool(ok)))
    print(('PASS ' if ok else 'FAIL ') + name + ('' if ok or not detail else f' :: {str(detail)[:2000]}'), flush=True)


def function(source, name):
    """The definition of probe.c function NAME (return type on the same or the line above)."""
    m = re.search(r'^(?:static )?(?:inline )?(?:void|int|uint32_t)\s*\n?' + name + r'\([^;{]*\)\s*\{', source, re.M)
    assert m, name
    depth, end = 1, m.end()
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[m.start():end] + '\n'


def between(source, start, end, include_end=True):
    i = source.index(start)
    j = source.index(end, i)
    return source[i:j + (len(end) if include_end else 0)]


def run_c(tmp, name, code, defines=(), sanitize=True):
    src, exe = tmp / f'{name}.c', tmp / name
    src.write_text(code)
    cmd = [CC, *FLAGS, *(SAN if sanitize else []), *[f'-D{d}' for d in defines], f'-I{PROBE_DIR}',
           str(src), '-o', str(exe)]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode:
        if sanitize and 'sanitize' in p.stderr:
            return run_c(tmp, name, code, defines, False)
        return p.returncode, 'compile: ' + p.stderr
    p = subprocess.run([str(exe)], capture_output=True, text=True, timeout=60)
    return p.returncode, p.stdout + p.stderr


# ---- 1. usb-reattach.h alone -----------------------------------------------------------
PURE = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "usb-reattach.h"
#define F 19200000ull
static struct ura u;
static void fresh(void) { memset(&u, 0, sizeof u); }
int main(void) {
 /* A reset or disconnect while ep0 waits for SETUP is normal enumeration: counted only. */
 fresh(); ura_reset(&u, true, false, 0, 100);
 assert(u.resets == 1 && !u.pending && !u.snap_why && !ura_due(&u, true, 100 * F, F));
 fresh(); ura_disconnect(&u, true, false, 0, 100);
 assert(u.disconnects == 1 && !u.pending && !ura_due(&u, true, 100 * F, F));
 /* ... but a configured session that sees one gets a snapshot (session loss). */
 fresh(); ura_reset(&u, true, true, 0, 100); assert(!u.pending && u.snap_why == URA_SNAP_RESET && u.snap_t == 100);
 fresh(); ura_disconnect(&u, true, true, 0, 100); assert(!u.pending && u.snap_why == URA_SNAP_DISCONNECT);
 /* Out of SETUP (data, status 2/3, status): a restart cause, state and snapshot kept. */
 for (uint32_t s = 1; s <= 4; s++) {
  fresh(); ura_reset(&u, false, false, s, 100);
  assert(u.pending == URA_RESET && u.reset_state == s && u.snap_why == URA_SNAP_RESET && u.snap_state == s);
  assert(ura_due(&u, false, 101, F) == URA_RESET && ura_due(&u, true, 101, F) == URA_RESET);
  fresh(); ura_disconnect(&u, false, true, s, 100);
  assert(u.pending == URA_DISCONNECT && u.disc_state == s && u.snap_why == URA_SNAP_DISCONNECT);
  assert(ura_due(&u, false, 101, F) == URA_DISCONNECT);
 }
 fresh(); ura_conndone(&u); ura_conndone(&u); assert(u.conndones == 2 && !u.pending);
 /* Guard: three hits in a row, a clean poll in between starts over. */
 fresh(); ura_guard(&u, true, 0x1004); ura_guard(&u, true, 0x1004); assert(!u.pending);
 ura_guard(&u, false, 8); ura_guard(&u, true, 6); ura_guard(&u, true, 6);
 assert(!u.pending && u.guard_hits == 4 && u.guard_run == 2);
 ura_guard(&u, true, 6); assert(u.pending == URA_GUARD && u.guard_count == 6 && u.guard_hits == 5);
 assert(ura_due(&u, true, 1, F) == URA_GUARD);
 /* Timeout: the clock starts when ep0 leaves SETUP, more than 2 s, stops in SETUP. */
 fresh(); ura_ep0(&u, true, 10); assert(!u.left_setup);
 ura_ep0(&u, false, 1000); ura_ep0(&u, false, 5000); assert(u.left_setup == 1000);
 assert(!ura_due(&u, false, 1000 + 2 * F, F) && ura_due(&u, false, 1000 + 2 * F + 1, F) == URA_TIMEOUT);
 ura_ep0(&u, true, 1000 + 2 * F); assert(!u.left_setup && !ura_due(&u, true, 1000 * F, F));
 fresh(); ura_ep0(&u, false, 0); assert(u.left_setup == 1);
 /* A restart counts once, under its lowest cause, clears every cause and the run. */
 fresh(); u.pending = URA_RESET | URA_GUARD; u.guard_run = 2; ura_ep0(&u, false, 50);
 ura_restarted(&u, URA_RESET | URA_TIMEOUT, true, 5000);
 assert(u.restarts[0] == 1 && !u.restarts[1] && !u.restarts[2] && !u.restarts[3] && !u.restart_fail);
 assert(!u.pending && !u.guard_run && !u.left_setup && u.last_why == (URA_RESET | URA_TIMEOUT) && u.last_restart == 5000);
 for (uint32_t c = 0; c < 4; c++) {
  fresh(); ura_restarted(&u, 1u << c, true, 9); assert(u.restarts[c] == 1);
 }
 /* Spacing: a cause within URA_SPACING_S of the last restart waits, it is not dropped. */
 fresh(); ura_restarted(&u, URA_GUARD, true, 5000);
 ura_disconnect(&u, false, false, 1, 5001);
 assert(!ura_due(&u, false, 5000 + F - 1, F) && u.pending == URA_DISCONNECT);
 assert(ura_due(&u, false, 5000 + F, F) == URA_DISCONNECT);
 /* A usb_start that failed early leaves ep0 out of SETUP: counted, retried 2 s later. */
 fresh(); ura_ep0(&u, false, 100); u.start_rc = URA_START_STUCK;
 ura_restarted(&u, URA_TIMEOUT, false, 100 + 3 * F);
 assert(u.restart_fail == 1 && u.restarts[3] == 1 && u.left_setup == 100 + 3 * F);
 assert(!ura_due(&u, false, 100 + 5 * F, F) && ura_due(&u, false, 100 + 5 * F + 1, F) == URA_TIMEOUT);
 /* cpu0 hands USB over in SETUP: earlier causes are stale. */
 fresh(); ura_reset(&u, false, false, 1, 5); for (int i = 0; i < 3; i++) ura_guard(&u, true, 7);
 ura_ep0(&u, false, 7); ura_handover(&u);
 assert(!ura_due(&u, true, 100 * F, F) && u.resets == 1 && u.guard_hits == 3);
 /* ud_epcmd failures. */
 fresh(); ura_epcmd(&u, 1, 0x406, 3, false); assert(u.epcmd_fail == 1 && u.epcmd_last == (6u | 1u << 4 | 3u << 12));
 ura_epcmd(&u, 0, 9, 0, true); assert(u.epcmd_fail == 2 && u.epcmd_last == (9u | 1u << 31));
 /* Rings: the snapshot is the last URA_RING events, oldest first. */
 fresh(); for (uint32_t i = 0; i < 20; i++) ura_event(&u, 0x100 + i, 0x100000000ull + 1000 + i);
 ura_snapshot(&u, URA_SNAP_RESTART, 2, 2000); assert(u.snap_n == 16 && u.snap_state == 2);
 for (uint32_t i = 0; i < 16; i++) assert(u.snap[i].ev == 0x104 + i && u.snap[i].t == 1004 + i);
 fresh(); for (uint32_t i = 0; i < 3; i++) ura_event(&u, 7 + i, 70 + i);
 ura_snapshot(&u, URA_SNAP_RESET, 0, 80);
 assert(u.snap_n == 3 && u.snap[0].ev == 7 && u.snap[2].ev == 9 && !u.snap[3].ev && !u.snap[15].t);
 /* Reply layout: fixed size, never a multiple of the 64-byte ep0 packet. */
 assert(URA_BYTES == 388 && URA_BYTES % 64 && URA_WORDS * 4 == URA_BYTES);
 static uint32_t w[URA_WORDS];
 const uint32_t regs[URA_REGS] = {0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7};
 fresh();
 u.restarts[0] = 1; u.restarts[1] = 2; u.restarts[2] = 3; u.restarts[3] = 4; u.restart_fail = 1;
 u.last_why = URA_RESET | URA_TIMEOUT; u.start_rc = URA_START_HALTED; u.last_restart = 0x500000000ull;
 u.resets = 5; u.disconnects = 6; u.conndones = 7; u.reset_state = 1; u.disc_state = 3; u.pending = URA_GUARD;
 u.guard_hits = 8; u.guard_count = 0x1003; ura_epcmd(&u, 1, 6, 2, false); ura_epcmd(&u, 0, 9, 0, true);
 for (uint32_t i = 0; i < 20; i++) ura_event(&u, i == 19 ? 0x101 : 0x40 + i, 0x400000000ull + 19200 * i);
 ura_snapshot(&u, URA_SNAP_RESET, 3, 0x400000000ull + 19200 * 19);
 ura_event(&u, 0x20c2, 0x400000000ull + 19200 * 25);
 uint64_t now = 0x400000000ull + 19200 * 30;
 assert(ura_reply(&u, w, regs, 0, now, F, true) == URA_BYTES);
 assert(w[0] == URA_MAGIC && w[1] == (1u | 16u << 8 | 1u << 16) && w[2] == 1 && w[5] == 4 && w[6] == 1);
 assert(w[7] == (9u | 3u << 8) && w[8] == 5 && w[11] == (1u | 3u << 8 | 0u << 16 | 4u << 24));
 assert(w[14] == 2 && w[15] == (9u | 1u << 31) && w[16] == 21 && w[17] == (1u | 16u << 8 | 3u << 16));
 assert(w[18] == F && w[19] == (uint32_t)now && w[20] == 4 && w[25] == 0xd1 && w[31] == 0xd7 && w[96] == URA_END);
 assert(w[32] == 0x40 + 4 && w[62] == 0x101 && w[64] == 0x40 + 5 && w[94] == 0x20c2);
 assert(ura_reply(&u, w, regs, 2, now, F, false) == URA_BYTES && !(w[1] >> 16) && (w[11] >> 16 & 0xff) == 2);
 /* A fresh record: no restart, no snapshot, empty rings. */
 fresh(); ura_reply(&u, w, regs, 0, 77, F, true);
 assert(!w[7] && !w[17] && !w[23] && !w[32] && !w[64] && w[96] == URA_END);
 /* The reply the monitor decoder is checked against (part 4). */
 fresh();
 u.restarts[0] = 1; u.restarts[1] = 2; u.restarts[2] = 3; u.restarts[3] = 4; u.restart_fail = 1;
 u.last_why = URA_RESET | URA_TIMEOUT; u.start_rc = URA_START_HALTED; u.last_restart = 0x400000000ull;
 u.resets = 5; u.disconnects = 6; u.conndones = 7; u.reset_state = 1; u.disc_state = 3; u.pending = URA_GUARD;
 u.guard_hits = 8; u.guard_count = 0x1003; ura_epcmd(&u, 1, 6, 2, false);
 for (uint32_t i = 0; i < 20; i++) ura_event(&u, i == 19 ? 0x101 : 0x40 + i, 0x400000000ull + 19200 * i);
 ura_snapshot(&u, URA_SNAP_RESET, 3, 0x400000000ull + 19200 * 19);
 ura_event(&u, 0x20c2, 0x400000000ull + 19200 * 25);
 ura_reply(&u, w, regs, 0, 0x400000000ull + 19200 * 30, F, true);
 printf("REPLY ");
 for (uint32_t i = 0; i < URA_WORDS; i++) printf("%08x", w[i]);
 printf("\nCONST %u %u %u %u\n", URA_BYTES, URA_MAGIC, URA_END, URA_RING);
 puts("pure: all assertions held");
 return 0;
}
'''

# ---- 2. probe.c's event path and restart on fake DWC3 registers ------------------------
HARNESS_HEAD = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define printf(...) ((void)0)
static uint64_t ticks = 1000;
static uint64_t fake_sysreg(const char *r)
{
 if (!strcmp(r, "cntpct_el0")) return ticks;
 assert(!strcmp(r, "cntfrq_el0"));
 return 19200000;
}
#define rsr(r) fake_sysreg(r)
'''
HARNESS_STUBS = r'''
static struct { uint64_t configured, speed; } probe_info;
static uint8_t dma_area[0x4000] __attribute__((aligned(0x4000)));
static uint32_t regs[0x10000 / 4];
static unsigned seq, ack_seq, start_seq, in_poll, starts, setups, depcmd_status, depcmd_stuck, wpos;
static unsigned start_result;  /* URA_START_* the fake usb_start reports */
static void ud_sync(uint32_t off, uint32_t len) { (void)off; (void)len; }
static void delay(unsigned us) { (void)us; }
static void au_out_data(uint16_t i, const volatile uint8_t *p, uint16_t n) { (void)i; (void)p; (void)n; }
static void dwc_wr(uint32_t off, uint32_t v)
{
 if (off == GEVNTCOUNT) { assert(v <= regs[off / 4]); regs[off / 4] -= v; ack_seq = ++seq; return; }
 if (off >= DEPCMD(0) && off <= DEPCMD(1) && !((off - DEPCMD(0)) % 0x10)) {
  assert(v & DEPCMD_CMDACT);
  regs[off / 4] = depcmd_stuck ? DEPCMD_CMDACT : depcmd_status << 12;
  return;
 }
 regs[off / 4] = v;
}
static uint32_t dwc_rd(uint32_t off) { return regs[off / 4]; }
static void ud_stall(void);
static void ud_start(unsigned ep, uint32_t buf, uint32_t len, uint32_t ctl);
/* ud_setup stand-in: the request kind the next SETUP carries. */
static int next_setup;  /* 0 IN with a data stage, 1 no data stage, 2 STALL */
static void ud_setup(void)
{
 setups++;
 if (next_setup == 2) { ud_stall(); return; }
 if (next_setup == 1) { ud.state = EP0_WAIT_STATUS2; return; }
 ud.state = EP0_DATA;
 ud_start(1, UD_DATA, 64, TRBCTL_DATA);
}
static void ud_start_setup(void);
/* usb_start stand-in: the boot path's effect on this state; never inside usb_poll. */
static void usb_start(uintptr_t dwc)
{
 assert(!in_poll);
 assert(dwc == ud.dwc);
 starts++; start_seq = ++seq;
 regs[GEVNTCOUNT / 4] = 0; ud.evt_pos = 0; wpos = 0;
#if FUXI_USB_DIAG
 ura.start_rc = start_result;
#endif
 if (start_result == 0) ud_start_setup();
}
'''
HARNESS_MAIN = r'''
#define EV_RESET 0x101u
#define EV_DISCONNECT 0x001u
#define EV_CONNDONE 0x201u
#define EV_SETUP_DONE 0x040u          /* ep0 OUT XferComplete */
#define EV_IN_DONE 0x042u             /* ep0 IN XferComplete */
#define EV_STATUS_READY 0x20c2u       /* ep0 IN XferNotReady, control status */
#define S(x) ((uint64_t)(x) * 19200000ull)
static void push(uint32_t ev)
{
 memcpy(dma_area + UD_EVT + wpos, &ev, 4);
 wpos = (wpos + 4) % UD_EVT_SIZE;
 regs[GEVNTCOUNT / 4] += 4;
}
/* One pass of the cpu@100 loop of probe_secondary_main: usb_poll, then usb_reattach.
 * usb_start asserts it never runs inside usb_poll; a poll that acked events acked them
 * before the restart (last_ack/last_start: this pass's ack and usb_start sequence numbers). */
static unsigned last_ack, last_start;
static unsigned loop_once(void)
{
 unsigned before = starts;
 ack_seq = 0;
 in_poll = 1; usb_poll(); in_poll = 0;
 last_ack = ack_seq;
#if FUXI_USB_REATTACH
 usb_reattach();
#endif
 last_start = starts != before ? start_seq : 0;
 if (last_start && last_ack) assert(last_ack < last_start);
 assert(ack_seq == last_ack);  /* nothing acked after the poll returned */
 return starts - before;
}
static void boot(void)
{
 memset(regs, 0, sizeof regs); memset(&ud, 0, sizeof ud); memset(&probe_info, 0, sizeof probe_info);
#if FUXI_USB_DIAG
 memset(&ura, 0, sizeof ura);
#endif
 ud.dwc = 0xa600000; ud.dma = (uintptr_t)dma_area; ud.dma_pa = ud.dma;
 wpos = 0; ud.evt_pos = 0; depcmd_status = depcmd_stuck = 0; start_result = 0; next_setup = 0;
 ticks = 1000;
 ud_start_setup();
 starts = 0;
}
static void transfer(void)  /* one whole IN control transfer, all events in one poll */
{
 next_setup = 0; push(EV_SETUP_DONE); push(EV_IN_DONE); push(EV_STATUS_READY); push(EV_IN_DONE);
}
int main(void)
{
 unsigned r = FUXI_USB_REATTACH;
 /* A. Normal enumeration and transfers: resets and disconnects only in SETUP. */
 boot();
 push(EV_RESET); push(EV_CONNDONE); assert(!loop_once());
 for (int i = 0; i < 50; i++) { transfer(); assert(!loop_once()); ticks += S(1) / 10; }
 next_setup = 1; push(EV_SETUP_DONE); push(EV_STATUS_READY); push(EV_IN_DONE); assert(!loop_once());
 next_setup = 2; push(EV_SETUP_DONE); assert(!loop_once());
 probe_info.configured = 1; push(EV_RESET); push(EV_DISCONNECT); push(EV_CONNDONE); assert(!loop_once());
 ticks += S(30); assert(!loop_once()); assert(ud.state == EP0_SETUP && !regs[GEVNTCOUNT / 4]);
#if FUXI_USB_DIAG
 /* The configured session's reset took the snapshot (it clears configured, so the
  * disconnect after it is enumeration noise). */
 assert(ura.resets == 2 && ura.disconnects == 1 && ura.conndones == 2 && ura.snap_why == URA_SNAP_RESET);
 assert(ura.nev == 2 + 200 + 3 + 1 + 3 && !ura.pending && !ura.restarts[0] && !ura.guard_hits);
#endif
 /* B. A reset while a data stage is pending (host abort): restart after the poll. */
 boot(); push(EV_SETUP_DONE); push(EV_RESET); push(EV_CONNDONE);
 assert(loop_once() == r);
 assert(last_ack && (!r || last_ack < last_start));  /* the events were acked, then usb_start */
#if FUXI_USB_DIAG
 assert(ura.resets == 1 && ura.reset_state == EP0_DATA && ura.snap_why == URA_SNAP_RESET);
 assert(ura.restarts[0] == r && (r || ura.pending == URA_RESET));
#endif
 if (r) assert(ud.state == EP0_SETUP);
 /* C. A disconnect in the status stage, in a later poll than the SETUP. */
 boot(); push(EV_SETUP_DONE); push(EV_IN_DONE); assert(!loop_once()); assert(ud.state == EP0_WAIT_STATUS3);
 push(EV_DISCONNECT); assert(loop_once() == r);
 assert(last_ack && (!r || last_ack < last_start));
#if FUXI_USB_DIAG
 assert(ura.restarts[1] == r && ura.disc_state == EP0_WAIT_STATUS3);
#endif
 /* D. GEVNTCOUNT guard: no ack, two hits wait, the third restarts. */
 boot(); regs[GEVNTCOUNT / 4] = 6;
 assert(!loop_once() && regs[GEVNTCOUNT / 4] == 6 && !ack_seq);
 assert(!loop_once() && regs[GEVNTCOUNT / 4] == 6);
#if FUXI_USB_REATTACH
 in_poll = 1; usb_poll(); in_poll = 0;            /* third hit: nothing restarts in the poll */
 assert(!starts); usb_reattach(); assert(starts == 1 && !regs[GEVNTCOUNT / 4]);
#else
 assert(!loop_once());
#endif
#if FUXI_USB_DIAG
 assert(ura.guard_hits == 3 && ura.guard_count == 6 && ura.restarts[2] == r);
#endif
 regs[GEVNTCOUNT / 4] = 0x1004; wpos = 0; assert(!loop_once());  /* one oversized hit, then fine */
 regs[GEVNTCOUNT / 4] = 0; assert(!loop_once());
 /* E. ep0 out of SETUP with no event (an aborted transfer): restart after more than 2 s. */
 boot(); push(EV_SETUP_DONE); assert(!loop_once()); assert(ud.state == EP0_DATA);
 ticks += S(1); assert(!loop_once());
 ticks += S(1); assert(!loop_once());
 ticks += 1; assert(loop_once() == r);
#if FUXI_USB_DIAG
 assert(ura.restarts[3] == r && (r ? ura.snap_why == URA_SNAP_RESTART : !ura.snap_why));
#endif
#if FUXI_USB_REATTACH
 {
  /* F. Spacing: a second cause 0.5 s after a restart waits until 1 s has passed. */
  uint64_t t0 = ticks;
  push(EV_SETUP_DONE); push(EV_RESET); ticks = t0 + S(1) / 2; assert(!loop_once());
  ticks = t0 + S(1) - 1; assert(!loop_once());
  ticks = t0 + S(1); assert(loop_once() == 1 && !last_ack);  /* acked two passes ago */
  assert(ura.restarts[0] == 1 && ura.restarts[3] == 1);
  /* G. usb_start failing early (CSFTRST stuck) leaves ep0 out of SETUP: retried 2 s later. */
  boot(); push(EV_SETUP_DONE); push(EV_RESET); start_result = 1; assert(loop_once() == 1);
  assert(ud.state == EP0_DATA && ura.restart_fail == 1);
  ticks += S(2); assert(!loop_once()); ticks += 1; start_result = 0; assert(loop_once() == 1);
  assert(ura.restarts[0] == 1 && ura.restarts[3] == 1 && ura.restart_fail == 1 && ud.state == EP0_SETUP);
 }
#endif
 /* H. ud_epcmd failures: a command status, then a timeout (CMDACT never clears). */
 boot(); depcmd_status = 3; transfer(); loop_once(); depcmd_status = 0;
 depcmd_stuck = 1; transfer(); loop_once(); depcmd_stuck = 0;
#if FUXI_USB_DIAG
 assert(ura.epcmd_fail >= 2 && (ura.epcmd_last >> 31));
#endif
 printf_real("events: reattach %u diag %u: all assertions held\n", r, (unsigned)FUXI_USB_DIAG);
 return 0;
}
'''


def harness():
    config = between(PROBE, '#ifndef FUXI_USB_REATTACH', '#define UDIAG(x) ((void)0)\n#endif\n')
    defines = between(PROBE, '#define GCTL ', '/* DMA area layout */', include_end=False)
    state = between(PROBE, '/* DMA area layout */', '} ud;\n')  # UD_* layout and the ud struct
    funcs = [function(PROBE, n) for n in ('ud_epcmd', 'ud_start', 'ud_start_setup', 'ud_stall',
                                          'ud_event', 'usb_poll')]
    reattach = function(PROBE, 'usb_reattach')
    return (HARNESS_HEAD + 'static int printf_real(const char *f, ...) __attribute__((format(printf, 1, 2)));\n'
            + config + defines + state + HARNESS_STUBS + ''.join(funcs)
            + '#if FUXI_USB_REATTACH\n' + reattach + '#endif\n' + HARNESS_MAIN
            + '#undef printf\n#include <stdarg.h>\nstatic int printf_real(const char *f, ...)\n'
              '{ va_list a; va_start(a, f); int n = vprintf(f, a); va_end(a); return n; }\n')


def static_checks():
    calls = [m.start() for m in re.finditer(r'\busb_start\(', PROBE)]
    main = function(PROBE, 'probe_main')
    reattach = function(PROBE, 'usb_reattach')
    secondary = function(PROBE, 'probe_secondary_main')
    check('usb_start( appears 3 times: its definition, probe_main (boot), usb_reattach',
          len(calls) == 3 and main.count('usb_start(') == 1 and reattach.count('usb_start(') == 1, len(calls))
    inside = {n: function(PROBE, n) for n in ('ud_event', 'usb_poll', 'ud_setup', 'ud_epcmd', 'ud_start',
                                              'ud_start_setup', 'ud_stall', 'thermal_monitor_idle')}
    bad = [n for n, body in inside.items() if 'usb_start(' in body or 'usb_reattach(' in body]
    check('the event path (ud_event, usb_poll, ud_setup, ...) never calls usb_start or usb_reattach', not bad, bad)
    check('usb_reattach( appears 3 times: declaration, definition, the cpu@100 loop',
          len(re.findall(r'\busb_reattach\(', PROBE)) == 3 and secondary.count('usb_reattach();') == 1)
    check('cpu@100 loop: usb_reattach right after usb_poll, before the thermal idle, under FUXI_USB_REATTACH',
          re.search(r'usb_poll\(\);\n#if FUXI_USB_REATTACH\n\s+usb_reattach\(\);[^\n]*\n#endif\n\s+thermal_monitor_idle\(\);',
                    secondary) is not None)
    check('probe_main: the cpu0 handover clears stale causes before DELEGATE=2',
          re.search(r'ud\.state==EP0_SETUP\) \{\n\s+UDIAG\(ura_handover\(&ura\)\);[^\n]*\n\s+__asm__ volatile\("dsb sy" ::: "memory"\);\n'
                    r'\s+monitor_report\[MON_DELEGATE\]=2;', main) is not None)
    body = between(PROBE, 'req == 0x59', '#endif')
    reads = re.findall(r'dwc_rd\((\w+)\)', body)
    check('0x59 reads exactly DSTS, DCTL, GSTS, GUSB2PHYCFG, GEVNTADRLO/HI/SIZ and writes nothing',
          reads == ['DSTS', 'DCTL', 'GSTS', 'GUSB2PHYCFG', 'GEVNTADRLO', 'GEVNTADRHI', 'GEVNTSIZ']
          and not re.search(r'dwc_wr|usb_wr|\*\(volatile|ud_epcmd|ud_start', body), (reads, body))
    check('0x59 is a vendor IN request with value 0 and index 0 only',
          'type == 0xc0 && req == 0x59 && value == 0 && st[4] == 0 && st[5] == 0' in PROBE)
    check('GSTS 0xc118, GUSB2PHYCFG 0xc200',
          re.search(r'#define GSTS\s+0xc118\b', PROBE) and re.search(r'#define GUSB2PHYCFG\s+0xc200\b', PROBE))
    check('the usb_start body is the boot path (one CSFTRST, event buffer, ep0, VBUS override, RUN_STOP)',
          all(s in function(PROBE, 'usb_start') for s in (
              'dwc_wr(DCTL, DCTL_CSFTRST);', 'dwc_wr(GEVNTADRLO,', 'DEPCMD_DEPSTARTCFG', 'ud_start_setup();',
              'UTMI_OTG_VBUS_VALID | SW_SESSVLD_SEL', 'dwc_wr(DCTL, dwc_rd(DCTL) | DCTL_RUN_STOP);')))


def defaults(tmp):
    config = between(PROBE, '#ifndef FUXI_USB_REATTACH', '#define UDIAG(x) ((void)0)\n#endif\n')
    code = ('#include <stdio.h>\n#include <stdint.h>\n' + config +
            'int main(void) { printf("%d %d\\n", FUXI_USB_REATTACH, FUXI_USB_DIAG); return 0; }\n')
    want = {(): '0 0', ('FUXI_SALAMI=1',): '1 1', ('FUXI_SALAMI=1', 'FUXI_USB_REATTACH=0'): '0 1',
            ('FUXI_SALAMI=1', 'FUXI_USB_REATTACH=1'): '1 1', ('FUXI_USB_REATTACH=0',): '0 0',
            ('FUXI_USB_REATTACH=1',): '1 1'}
    got = {}
    for i, (d, w) in enumerate(want.items()):
        rc, out = run_c(tmp, f'defaults{i}', code, d, sanitize=False)
        got[d] = out.strip() if rc == 0 else f'rc {rc}: {out[-200:]}'
    check('defaults: Fuxi 0/0, salami 1/1, salami REATTACH=0 keeps 0x59 (0/1), REATTACH=1 anywhere 1/1',
          got == want, got)
    rc, out = run_c(tmp, 'defaults-bad', code, ('FUXI_SALAMI=1', 'FUXI_USB_REATTACH=2'), sanitize=False)
    check('FUXI_USB_REATTACH=2 is refused at compile time', rc != 0 and 'must be 0 or 1' in out, out[-300:])


def monitor_decoder():
    tree = ast.parse((ROOT / 'tools/fuxi-xnu-monitor.py').read_text())
    keep = [n for n in tree.body if
            (isinstance(n, ast.Assign) and all(isinstance(t, (ast.Name, ast.Tuple)) for t in n.targets) and
             any(name.startswith('USB_') for t in n.targets for name in
                 ([t.id] if isinstance(t, ast.Name) else [e.id for e in t.elts if isinstance(e, ast.Name)]))) or
            (isinstance(n, ast.FunctionDef) and n.name in ('usb_event_name', 'usb_diag_decode', 'usb_diag_brief'))]
    ns = {'struct': struct}
    exec(compile(ast.Module(body=keep, type_ignores=[]), '<monitor 0x59 decoder>', 'exec'), ns)
    return ns


def decode_checks(out):
    m = re.search(r'^REPLY ([0-9a-f]+)$', out, re.M)
    c = re.search(r'^CONST (\d+) (\d+) (\d+) (\d+)$', out, re.M)
    if not (m and c):
        check('decoder: the C reply was printed', False, out[-500:])
        return
    raw = b''.join(struct.pack('<I', int(m.group(1)[i:i + 8], 16)) for i in range(0, len(m.group(1)), 8))
    ns = monitor_decoder()
    nbytes, magic, end, ring = (int(x) for x in c.groups())
    check('decoder constants match usb-reattach.h (388 bytes, URA1/URAZ, 16-entry rings)',
          (ns['USB_DIAG_BYTES'], ns['USB_DIAG_MAGIC'], ns['USB_DIAG_END']) == (nbytes, magic, end) and
          len(raw) == nbytes and ring == 16)
    d = ns['usb_diag_decode'](raw)
    exp = {'version': 1, 'ring': 16, 'reattach': True,
           'restarts': {'reset': 1, 'disconnect': 2, 'guard': 3, 'timeout': 4}, 'restart_failed': 1,
           'resets': 5, 'disconnects': 6, 'connect_done': 7, 'ep0_at_last_reset': 'data',
           'ep0_at_last_disconnect': 'wait_status3', 'ep0_now': 'setup', 'pending': ['guard'],
           'guard_hits': 8, 'guard_last_count': '0x1003', 'epcmd_failures': 1,
           'epcmd_last': {'command': 6, 'ep': 1, 'status': 2, 'timeout': False}, 'events': 21,
           'cntfrq': 19200000, 'complete': True}
    bad = {k: (d.get(k), v) for k, v in exp.items() if d.get(k) != v}
    check('decoder: counters, states, pending causes, epcmd failure', not bad, bad)
    lr = d['last_restart']
    check('decoder: last restart causes reset+timeout, usb_start halted, 30 ms ago',
          lr == {'causes': ['reset', 'timeout'], 'start': 'halted', 'age_ms': 30.0}, lr)
    check('decoder: registers in order DSTS..GEVNTSIZ',
          d['regs'] == {k: f'{v:#010x}' for k, v in zip(ns['USB_DIAG_REGS'], range(0xd1, 0xd8))}, d['regs'])
    s = d['snapshot']
    check('decoder: snapshot at the reset (ep0 wait_status3), 16 events oldest first ending with the reset',
          s and s['why'] == 'reset' and s['ep0'] == 'wait_status3' and s['age_ms'] == 11.0 and
          len(s['events']) == 16 and s['events'][-1]['event'] == 'reset' and s['events'][-1]['before_ms'] == 0.0 and
          s['events'][0]['raw'] == '0x00000044' and s['events'][0]['before_ms'] == 15.0, s)
    live = d['live_events']
    check('decoder: live ring of 16, last the status XferNotReady 5 ms before the read',
          len(live) == 16 and live[-1]['event'] == 'ep1 xfer_not_ready status 2' and live[-1]['before_ms'] == 5.0 and
          live[-2]['event'] == 'reset' and live[0]['raw'] == '0x00000045', live[-3:])
    short = ns['usb_diag_decode'](raw[:128])
    check('decoder: a 128-byte reply gives the summary, empty rings, not complete',
          short['restarts'] == exp['restarts'] and short['snapshot']['events'] == [] and short['live_events'] == []
          and not short['complete'])
    for name, blob in (('short', raw[:124]), ('magic', b'\0' * 388)):
        try:
            ns['usb_diag_decode'](blob)
            ok = False
        except ValueError:
            ok = True
        check(f'decoder: refuses a {name} reply with ValueError', ok)
    check('decoder: brief line names restarts by cause and DSTS',
          ns['usb_diag_brief'](d).startswith('restarts reset 1 disconnect 2 guard 3 timeout 4 (failed 1)') and
          'DSTS 0x000000d1' in ns['usb_diag_brief'](d))


def main():
    with tempfile.TemporaryDirectory(prefix='fuxi-usb-reattach-') as t:
        tmp = Path(t)
        rc, out = run_c(tmp, 'pure', PURE)
        check('usb-reattach.h: triggers, non-triggers, spacing, counting, handover, rings, reply layout',
              rc == 0 and 'pure: all assertions held' in out, out[-2000:])
        code = harness()
        for label, defs, want in (('salami (reattach on)', ('FUXI_SALAMI=1',), 'reattach 1 diag 1'),
                                  ('salami FUXI_USB_REATTACH=0', ('FUXI_SALAMI=1', 'FUXI_USB_REATTACH=0'),
                                   'reattach 0 diag 1'),
                                  ('Fuxi (neither)', (), 'reattach 0 diag 0')):
            rc, res = run_c(tmp, 'harness-' + re.sub(r'\W+', '-', label), code, defs)
            check(f'probe.c event path + restart, {label}: every trigger restarts once after usb_poll '
                  f'returned (never inside it), ack first'
                  if 'on' in label else f'probe.c event path, {label}: never restarts, records as built',
                  rc == 0 and want in res, res[-2000:])
        static_checks()
        defaults(tmp)
        decode_checks(out)
    bad = [n for n, ok in RESULTS if not ok]
    print(f'usb reattach tests: {len(RESULTS) - len(bad)} PASS, {len(bad)} FAIL', flush=True)
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
