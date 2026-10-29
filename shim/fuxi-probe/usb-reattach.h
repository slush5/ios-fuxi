/* Loader USB monitor bookkeeping and restart decisions (salami, probe.c FUXI_USB_DIAG).
 * Pure C: no MMIO and no system registers. probe.c passes the counter time and register
 * values in; tools/test-fuxi-usb-reattach.py compiles this file unchanged on the host.
 *
 * The event path (ud_event, usb_poll, both inside the usb_poll event loop) only records.
 * Restart causes, as ura_due reports them to the cpu@100 main loop after usb_poll returns:
 *  URA_RESET       a USB reset arrived while ep0 was not waiting for a SETUP packet
 *  URA_DISCONNECT  a disconnect arrived while ep0 was not waiting for a SETUP packet
 *  URA_GUARD       URA_GUARD_RUN usb_poll guard hits in a row (GEVNTCOUNT > the buffer
 *                  or not a multiple of 4: the poll returns without an ack)
 *  URA_TIMEOUT     ep0 out of SETUP for more than URA_TIMEOUT_S seconds (an aborted
 *                  control transfer: the host runs setup, data and status in one URB)
 * Restarts are at least URA_SPACING_S apart (a pending cause waits, it is not dropped).
 * Each restart counts once, under its lowest cause bit; last_why keeps all of them.
 *
 * Request 0x59 reply (ura_reply, URA_BYTES, little-endian 32-bit words):
 *  0 'URA1'  1 version | URA_RING << 8 | reattach << 16
 *  2..5 restarts by cause (reset, disconnect, guard, timeout)
 *  6 restarts whose usb_start did not finish  7 last restart causes | usb_start result << 8
 *  8 USB resets  9 disconnects  10 connect-done events
 *  11 ep0 state at the last reset | at the last disconnect << 8 | now << 16 | pending << 24
 *  12 guard hits  13 GEVNTCOUNT at the last guard hit  14 ud_epcmd failures
 *  15 last ud_epcmd failure: command | physical ep << 4 | status << 12 | timeout << 31
 *  16 events seen  17 snapshot why | entries << 8 | ep0 state << 16  18 counter frequency
 *  19,20 counter now  21,22 counter at the snapshot  23,24 counter at the last restart
 *  25..31 DSTS, DCTL, GSTS, GUSB2PHYCFG, GEVNTADRLO, GEVNTADRHI, GEVNTSIZ
 *  32..63 snapshot: URA_RING x (raw event, counter low 32 bits), oldest first, taken at a
 *         reset or disconnect while configured or out of SETUP, and at a guard/timeout restart
 *  64..95 the last URA_RING events (live ring), oldest first; unused entries are zero
 *  96 'URAZ' (388 bytes in all: never a multiple of the 64-byte ep0 packet, so a host
 *     that asks for more still gets a short packet)
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define URA_RESET       1u
#define URA_DISCONNECT  2u
#define URA_GUARD       4u
#define URA_TIMEOUT     8u
#define URA_CAUSES      4u
#define URA_GUARD_RUN   3u
#define URA_TIMEOUT_S   2u
#define URA_SPACING_S   1u
#define URA_RING        16u
#define URA_SNAP_RESET      1u
#define URA_SNAP_DISCONNECT 2u
#define URA_SNAP_RESTART    3u
#define URA_START_OK     0u     /* usb_start reached a running controller */
#define URA_START_STUCK  1u     /* CSFTRST did not clear */
#define URA_START_EP0    2u     /* an ep0 configuration command failed */
#define URA_START_HALTED 3u     /* RUN_STOP set, DEVCTRLHLT still set after 500 ms */
#define URA_REGS        7u
#define URA_WORDS       97u
#define URA_BYTES       (URA_WORDS * 4u)
#define URA_MAGIC       0x31415255u     /* 'URA1' */
#define URA_END         0x5a415255u     /* 'URAZ' */
#define URA_VERSION     1u

struct ura_ev { uint32_t ev, t; };
struct ura {
 uint64_t left_setup;           /* counter time ep0 left SETUP; 0 while in SETUP */
 uint64_t last_restart;         /* counter time of the last restart; 0 before any */
 uint64_t snap_t;               /* counter time of the snapshot */
 uint32_t pending;              /* URA_* causes seen by the event path, for the main loop */
 uint32_t guard_run;            /* usb_poll guard hits in a row */
 uint32_t restarts[URA_CAUSES];
 uint32_t restart_fail, last_why, start_rc;
 uint32_t resets, disconnects, conndones, reset_state, disc_state;
 uint32_t guard_hits, guard_count, epcmd_fail, epcmd_last;
 uint32_t nev;                  /* events recorded (the live ring index) */
 uint32_t snap_n, snap_why, snap_state;
 struct ura_ev ring[URA_RING], snap[URA_RING];
};

static inline void ura_event(struct ura *u, uint32_t ev, uint64_t now)
{
 u->ring[u->nev % URA_RING].ev = ev;
 u->ring[u->nev % URA_RING].t = (uint32_t)now;
 u->nev++;
}

static inline void ura_snapshot(struct ura *u, uint32_t why, uint32_t state, uint64_t now)
{
 uint32_t n = u->nev < URA_RING ? u->nev : URA_RING;
 for (uint32_t i = 0; i < n; i++) u->snap[i] = u->ring[(u->nev - n + i) % URA_RING];
 for (uint32_t i = n; i < URA_RING; i++) u->snap[i].ev = u->snap[i].t = 0;
 u->snap_n = n; u->snap_why = why; u->snap_state = state; u->snap_t = now;
}

/* A USB reset (device event 1). in_setup: ep0 was waiting for a SETUP packet;
 * configured: SET_CONFIGURATION had run (the host had a session). */
static inline void ura_reset(struct ura *u, bool in_setup, bool configured, uint32_t state, uint64_t now)
{
 u->resets++; u->reset_state = state;
 if (!in_setup) u->pending |= URA_RESET;
 if (!in_setup || configured) ura_snapshot(u, URA_SNAP_RESET, state, now);
}

/* A disconnect (device event 0), as ura_reset. */
static inline void ura_disconnect(struct ura *u, bool in_setup, bool configured, uint32_t state, uint64_t now)
{
 u->disconnects++; u->disc_state = state;
 if (!in_setup) u->pending |= URA_DISCONNECT;
 if (!in_setup || configured) ura_snapshot(u, URA_SNAP_DISCONNECT, state, now);
}

static inline void ura_conndone(struct ura *u) { u->conndones++; }

/* Every usb_poll: hit = the GEVNTCOUNT guard returned without an ack. */
static inline void ura_guard(struct ura *u, bool hit, uint32_t count)
{
 if (!hit) { u->guard_run = 0; return; }
 u->guard_hits++; u->guard_count = count;
 if (++u->guard_run >= URA_GUARD_RUN) u->pending |= URA_GUARD;
}

/* After every event: start the out-of-SETUP clock when ep0 leaves SETUP, stop it there. */
static inline void ura_ep0(struct ura *u, bool in_setup, uint64_t now)
{
 if (in_setup && u->start_rc == URA_START_OK) u->left_setup = 0;
 else if (!u->left_setup) u->left_setup = now ? now : 1;
}

static inline void ura_epcmd(struct ura *u, unsigned ep, uint32_t cmd, uint32_t status, bool timeout)
{
 u->epcmd_fail++;
 u->epcmd_last = (cmd & 0xfu) | (ep & 0x1fu) << 4 | (status & 0xfu) << 12 | (timeout ? 1u << 31 : 0u);
}

/* cpu0 hands USB to cpu@100 with ep0 in SETUP (a 0x46 value 1 just went through): causes
 * seen before then are stale and must not restart the link under the host. The host just
 * completed a transfer, so the controller runs whatever the boot usb_start reported (a slow
 * DevCtrlHlt can read as HALTED): forget that result too, or the failed-start retry in
 * ura_due would restart the link about 2 s later, before the 0x49 handoff. */
static inline void ura_handover(struct ura *u)
{
 u->pending = 0; u->guard_run = 0; u->left_setup = 0; u->start_rc = URA_START_OK;
}

/* Main loop only (never inside the event loop): the causes of a restart due now, or 0. */
static inline uint32_t ura_due(const struct ura *u, bool in_setup, uint64_t now, uint64_t frq)
{
 uint32_t why = u->pending;
 /* Out of SETUP too long, or the last usb_start did not finish (retry even in SETUP). */
 if ((!in_setup || u->start_rc != URA_START_OK) && u->left_setup &&
     now - u->left_setup > URA_TIMEOUT_S * frq) why |= URA_TIMEOUT;
 if (why && u->last_restart && now - u->last_restart < URA_SPACING_S * frq) return 0;
 return why;
}

/* After the main loop re-ran usb_start for WHY (start_rc set by usb_start). in_setup: ep0
 * state afterwards. When usb_start did not finish (or left ep0 out of SETUP) a timeout
 * comes again URA_TIMEOUT_S later, also with ep0 in SETUP. */
static inline void ura_restarted(struct ura *u, uint32_t why, bool in_setup, uint64_t now)
{
 for (uint32_t c = 0; c < URA_CAUSES; c++)
  if (why & (1u << c)) { u->restarts[c]++; break; }
 if (u->start_rc != URA_START_OK) u->restart_fail++;
 u->last_why = why; u->last_restart = now ? now : 1;
 u->pending = 0; u->guard_run = 0;
 u->left_setup = (in_setup && u->start_rc == URA_START_OK) ? 0 : (now ? now : 1);
}

/* Request 0x59: fill w[URA_WORDS] (layout above), return the byte count. */
static inline uint32_t ura_reply(const struct ura *u, uint32_t *w, const uint32_t regs[URA_REGS],
                                 uint32_t state, uint64_t now, uint64_t frq, bool reattach)
{
 uint32_t n = u->nev < URA_RING ? u->nev : URA_RING;
 for (uint32_t i = 0; i < URA_WORDS; i++) w[i] = 0;
 w[0] = URA_MAGIC;
 w[1] = URA_VERSION | URA_RING << 8 | (reattach ? 1u << 16 : 0u);
 for (uint32_t c = 0; c < URA_CAUSES; c++) w[2 + c] = u->restarts[c];
 w[6] = u->restart_fail;
 w[7] = (u->last_why & 0xffu) | (u->last_restart ? (u->start_rc & 0xffu) << 8 : 0u);
 w[8] = u->resets; w[9] = u->disconnects; w[10] = u->conndones;
 w[11] = (u->reset_state & 0xffu) | (u->disc_state & 0xffu) << 8 | (state & 0xffu) << 16 |
         (u->pending & 0xffu) << 24;
 w[12] = u->guard_hits; w[13] = u->guard_count; w[14] = u->epcmd_fail; w[15] = u->epcmd_last;
 w[16] = u->nev;
 w[17] = (u->snap_why & 0xffu) | (u->snap_n & 0xffu) << 8 | (u->snap_state & 0xffu) << 16;
 w[18] = (uint32_t)frq;
 w[19] = (uint32_t)now; w[20] = (uint32_t)(now >> 32);
 w[21] = (uint32_t)u->snap_t; w[22] = (uint32_t)(u->snap_t >> 32);
 w[23] = (uint32_t)u->last_restart; w[24] = (uint32_t)(u->last_restart >> 32);
 for (uint32_t r = 0; r < URA_REGS; r++) w[25 + r] = regs[r];
 for (uint32_t i = 0; i < URA_RING && i < u->snap_n; i++) {
  w[32 + 2 * i] = u->snap[i].ev; w[33 + 2 * i] = u->snap[i].t;
 }
 for (uint32_t i = 0; i < n; i++) {
  const struct ura_ev *e = &u->ring[(u->nev - n + i) % URA_RING];
  w[64 + 2 * i] = e->ev; w[65 + 2 * i] = e->t;
 }
 w[96] = URA_END;
 return URA_BYTES;
}
