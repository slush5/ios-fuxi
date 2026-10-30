/*
 * W20 (fuxiwlanqrtr=6, C1) host model: a fake WCN7850 copy-engine/HTC/WMI
 * target behind the production code. tools/test-fuxi-wlan-ce.py cuts the
 * glue out of xnu/fuxi-wlan/fuxi_wlan.c into fuxi-wlan-ce-prod.inc (stage
 * enum, W20 knobs, the WLAN_CFG CE tables, wl_w11_fail, the W20 block and
 * wl5_abort) and builds this file with ASan/UBSan and -Werror against the
 * pure xnu/fuxi-wlan/fuxi_wlan_ce.h. What the model enforces:
 *   BAR0: the window register 0x310c (read back; restored after every
 *   access), the shadow registers 0x8fc + 4i routed through the same
 *   wl15_shadow_v3 table to the R2 register each names, and window accesses
 *   to its own list of CE registers (R0/R2 of the 7 C1 rings, DEST_CTRL of
 *   CE1/CE2). Any other register access aborts the run.
 *   DMA: the device touches the mapped arena only: rings, rdp, the RX buffers
 *   of the pipe it fills (one 2 KiB buffer per message) and the TX slots of
 *   the pipe it reads.
 *   CE engine: ring setup checked against L19 values written down here again
 *   (not taken from the header), DEST_CTRL max length 2048 written after its
 *   ring went live; SRC descriptors checked (address in the pipe's TX slots,
 *   length, swap/gather 0, meta = endpoint) and handed to the firmware; RX
 *   takes the next posted DST entry (address bits 39:32 only in word 1),
 *   fills the buffer, writes a STATUS entry and the rdp words.
 *   Firmware: HTC READY, connect replies, SERVICE_READY (+AVAILABLE, EXT,
 *   EXT2), READY on INIT (only after EXT/EXT2 reached the host), the HW mode
 *   response, an unprompted CC_EXT event, credits in trailers with its own
 *   credit ledger (header included); INIT and HW mode are checked field by
 *   field.
 *   Order: every rdp load is followed by the read barrier, every pointer
 *   publish of a live ring is preceded by the write barrier.
 *   C2 (fuxiwlanqrtr=7): CE4 TX slots and the HTT version exchange; the
 *   channel list (every word, the APPEND split, the array header L19 sends
 *   4 short), VDEV_CREATE, NSS, three STA power-save parameters and power
 *   save off in L19 order, START_SCAN word by word (passive, no SSID, no IE,
 *   no probe), then a scripted scan: STARTED, per channel FOREIGN_CHAN and
 *   CHAN_INFO, MGMT_RX beacons and probe responses (malformed IEs, truncated
 *   frames, zero-length and all-zero SSIDs, duplicate BSSIDs, more than 64
 *   BSSIDs, dropped statuses and channels), COMPLETED; STOP_SCAN checked
 *   field by field and only before the containment latch. SERVICE_READY_EXT
 *   lists SINGLE and DBS with the MAC/PHY caps of both (L19's preferred-mode
 *   PHYs feed the streams); a guard failure after START_SCAN and a STOP_SCAN
 *   whose TX never completes are scripted too.
 *   C2b: CHAN_INFO START_RESP carries realistic MAC-clock counters (320 MHz,
 *   about 150 ms per dwell, 3 % busy plus 300 us per frame heard there),
 *   END_RESP none; the DBS PHYs have their own pdev ids (1 and 2, as each
 *   WCN7850 MAC); REQUEST_STATS is checked field by field (PDEV_STAT, vdev
 *   0, MAC 0, the preferred hw mode's PHY pdev ids in PHY order) and only
 *   after COMPLETED and before the latch, and answered with an L19-shaped
 *   UPDATE_STATS for that pdev (fixed TLV 0x46, ARRAY_BYTE of vdev, beacon
 *   and pdev entries in L19's order) or, by scenario, not at all, malformed,
 *   short, empty, twice, late, at its widest, after a credit drought or
 *   behind a stuck TX descriptor. Every FUXI WLAN line must fit the
 *   console's 254 bytes (model_line).
 * No device, no network. L19 = Linux ath12k v6.19 (BSD-3-Clause-Clear).
 */
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

static char logbuf[1 << 21];
static size_t loglen;
static void model_line(const char *line, size_t len);
static void
test_printf(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(logbuf + loglen, sizeof(logbuf) - loglen, fmt, ap);
	va_end(ap);
	assert(n >= 0 && (size_t)n < sizeof(logbuf) - loglen);
	loglen += (size_t)n;
	for (size_t i = loglen - (size_t)n; i < loglen; i++) {  /* every line this call completed */
		if (logbuf[i] != '\n') { continue; }
		size_t s = i;
		while (s && logbuf[s - 1] != '\n') { s--; }
		model_line(logbuf + s, i - s);
	}
}
#define printf test_printf

static void model_event(char kind);
#define WLCE_RDP_LOAD(ce, id) (model_event('L'), (ce)->rdp[(id)])
#include "fuxi_wlan_ce.h"

/* ---- what the production excerpts expect from fuxi_wlan.c ---- */
typedef uintptr_t vm_offset_t;
typedef size_t vm_size_t;
struct wl_dma { vm_offset_t va; vm_size_t size; uint64_t pa, iova; bool nc; };
#define WL_BAR0 0x60400000u
#define WL_EPCFG 0x60100000ull
#define TS_TLMM 0x0f100000UL
#define WL_TARGET_MAX 52u
static unsigned fw_faults;
static unsigned wl15_variant, wl12_stage, wl_w11_error, wl_seq_flags;
static bool wl_w15, wl_w12, wl_w11_ok, wl_w11_attempted, wl_mhi_valid;
static uint64_t wl_iova_next, wl_iova_base = 0xa0000000ull, wl_iova_end = 0xb0000000ull;
static struct wl_dma wl_ce_arena, wl_msi_sink, wl_tables, wl_rxvectors, wl_rddm, wl_target[WL_TARGET_MAX];
static struct wl_dma wl14_regdb, wl14_bdf, wl14_m3, wl_ipcr, wl_contexts, wl_vectors, wl_amss, wl_image;
static uint32_t wl_mrd(uint32_t off);
static void wl_mwr(uint32_t off, uint32_t v);
static uint32_t wl_prd(uint64_t pa);
static void wl_pwr(uint64_t pa, uint32_t v);
static int wl_contain_wr(uint64_t pa, uint32_t value);
static int wl_dma_alloc(struct wl_dma *d, vm_size_t bytes);
static int wl_dma_nc(struct wl_dma *d);
static void wl_dma_free(struct wl_dma *d);
static int wl12_map_target(struct wl_dma *d);
static int wl_w11_events(void);
static int wl_w11_guard(void);
static void wl_w11_stop(void);
static void wl_sleep(unsigned ms);
static unsigned wl_m6o_cap(unsigned id);
static void wl_wlfw_diagnostics(const char *label);
static void wl5_status(unsigned step);
static int wl_smmu_fault(unsigned step);
static int wl_m6o_msi_undo(void);
static int wl_m6o_undo(bool contained);
static int wl_smmu_undo(void);
static void model_barrier(char kind);

#include "fuxi-wlan-ce-prod.inc"

/* ---- model state ---- */
#define ARENA 0x80000u
#define CE_BASE 0x01b80000u
#define CE_END 0x01b8a000u
#define SHADOW0 0x8fcu
#define WIN_INIT 0x40000010u            /* some unrelated window the glue must restore */
enum { K_SRC, K_DST, K_STS };
struct dring {
	uint32_t id, kind, ce, entries, edw, r0, r2;
	bool enabled;
	uint64_t base, ptr_addr;
	uint32_t size, host, dev;           /* SRC/DST: host = HP, dev = device TP; STS: host = TP, dev = HP */
};
/* L19 rings of C1 (ce.c:145-220, hal.h:419-468, hal_desc.h entry sizes), written down independently. */
static const struct dring ring_plan[7] = {
	{64, K_SRC, 0, 16, 4, 0, 0, false, 0, 0, 0, 0, 0}, {82, K_DST, 1, 512, 2, 0, 0, false, 0, 0, 0, 0, 0},
	{101, K_STS, 1, 512, 4, 0, 0, false, 0, 0, 0, 0, 0}, {83, K_DST, 2, 64, 2, 0, 0, false, 0, 0, 0, 0, 0},
	{102, K_STS, 2, 64, 4, 0, 0, false, 0, 0, 0, 0, 0}, {67, K_SRC, 3, 32, 4, 0, 0, false, 0, 0, 0, 0, 0},
	{68, K_SRC, 4, 2048, 4, 0, 0, false, 0, 0, 0, 0, 0},
};
static struct dring dr[7];
static uint32_t ce_reg[(CE_END - CE_BASE) / 4];
static uint8_t *arena_host;
static uint64_t arena_iova;
static bool arena_mapped;
static unsigned arena_allocs, arena_frees;
static uint32_t bar_win, epcmd, perst_io, devctl, mhictrl;
static unsigned win_selects, ce_writes, ce_reads, shadow_writes, direct_publishes, shadow_publishes, sleeps, stops;
static uint64_t now;
static char last_ev;
static unsigned n_ev[128];

struct cfg {
	unsigned credits, credit_size, ready_ce, ready_eid, htt_eid, wmi_eid, max_msg, htt_status;
	int ready_stslen;
	bool silent_ready, ready_trailer_long, bad_report, svc_tlv_past, ext_nested_overrun, no_ext2_service;
	bool refund_in_ready, refund_later, fail_bm, guard_fail_on_connect;
	bool dest_ctrl_ignore, ready_trailer_zero, empty_msgs, big_event, ready_mac;
	uint32_t mem_reqs, ready_status, mrrs_old, ext_delay;
	unsigned rdp_glitch, win_mismatch_at, fault_after, restore_lie_at;
	int sts_rdp_delta;                  /* one CE2 STATUS rdp write, relative to the host TP, after 3 messages */
	/* C2 */
	unsigned scan_mode;                 /* 0 normal, 1 START_FAILED, 2 no STARTED, 3 no COMPLETED, 4 COMPLETED first */
	bool no_start_refund, htt_silent, bad_scan_tlv, wrong_vdev;
	bool ext_single_last;               /* EXT lists DBS first, SINGLE (1x1) last */
	bool guard_after_start;             /* the W11 guard fails once the firmware took START_SCAN */
	bool stop_tx_stuck;                 /* CE3 never consumes the STOP_SCAN descriptor */
	unsigned extra_unknown;             /* unknown WMI events after READY (the 16-line budget) */
	uint32_t phy_bands, phy_mask, htt_major, htt_minor, exp_n, exp_pdev;
	/* C2b */
	unsigned stats_mode;                /* see fw_stats_answer */
	bool stats_tx_stuck;                /* CE3 never completes the REQUEST_STATS descriptor */
	bool guard_on_stats;                /* the W11 guard fails once the firmware took REQUEST_STATS */
	uint32_t start_refund_at;           /* with no_start_refund: the START_SCAN credit comes back this late */
	unsigned ci_repeat;                 /* extra CHAN_INFO START_RESP per channel (the 128-line cap) */
	uint32_t dbs_pdev;                  /* pdev id of DBS PHY 0; PHY 1 has dbs_pdev + 1 (WCN7850: one per MAC) */
	bool dbs_same_pdev;                 /* both DBS PHYs report dbs_pdev */
	bool ext_single_only;               /* EXT lists SINGLE alone: no preferred-mode PHY (L19's fw_pdev[0] stays 0) */
	unsigned on_line;                   /* a device-side action when a given line is printed (model_line) */
};
static struct cfg C;
static unsigned restores;
static bool restore_lie, sts_glitched;

/* firmware */
struct qmsg { unsigned ce; uint64_t due; uint32_t len; int stslen; uint8_t d[WLCE_BUF]; };
static struct qmsg q[512];
static unsigned q_head, q_tail;
static bool fw_ready_sent, fw_setup_done;
static int fw_credits;
static unsigned fw_seq[9], fw_eid_used, fw_wmi_msgs, fw_init_ok, fw_hw_mode_ok, fw_echoes, fw_tx_msgs, fw_ext_q, fw_ext2_q;
static bool fw_svc281, guard_bad;
static uint32_t fw_chains = 2;
/* C2 firmware: command order step, channels seen, counters */
static unsigned fw_c2_step, fw_chan_seen, fw_chan_cmds, fw_scan_cmds, fw_stop_cmds, fw_htt_req, fw_stop_pdev;
static unsigned fw_stats_req;
static uint64_t fw_stats_at;
static uint32_t fw_stats_pdev = 1;  /* the pdev the last REQUEST_STATS named; UPDATE_STATS carries it back */
struct mbeacon { unsigned idx; uint32_t rx_channel, snr, status, chan_freq, mode, len; uint8_t f[400]; };
static struct mbeacon beacons[96];
static unsigned n_beacons;

static void
fail(const char *why)
{
	fputs(logbuf, stderr);
	fprintf(stderr, "MODEL FAILURE: %s\n", why);
	abort();
}
#define CHECK(c, why) do { if (!(c)) { fail(why); } } while (0)

static uint32_t g32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static void p32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

static void
model_event(char kind)
{
	if (last_ev == 'L') { CHECK(kind == 'R', "an rdp load not followed by the read barrier"); }
	if (kind == 'P') { CHECK(last_ev == 'W', "a pointer publish without the write barrier before it"); }
	last_ev = kind;
	n_ev[(unsigned char)kind]++;
}
static void model_barrier(char kind) { model_event(kind); }

/* ---- device memory: the mapped arena only ---- */
static uint8_t *
dev_mem(uint64_t iova, uint32_t len, uint32_t lo, uint32_t hi, const char *what)
{
	CHECK(arena_host && arena_mapped, "device DMA before the arena is mapped");
	CHECK(iova >= arena_iova + lo && iova + len <= arena_iova + hi && hi <= ARENA, what);
	return arena_host + (iova - arena_iova);
}

static struct dring *
ring_of_reg(uint32_t off, uint32_t *rel_r0, uint32_t *rel_r2)
{
	for (unsigned i = 0; i < 7; i++) {
		struct dring *r = &dr[i];
		if (off >= r->r0 && off < r->r0 + (r->kind == K_STS ? 0x28u : 0x38u)) { *rel_r0 = off - r->r0; *rel_r2 = ~0u; return r; }
		if (off == r->r2 || off == r->r2 + 4) { *rel_r0 = ~0u; *rel_r2 = off - r->r2; return r; }
	}
	return NULL;
}

/* The model's own allowed set: R0 regs per ring kind (hw.c:1196-1253 minus
 * the R0 base), R2 HP/TP, DEST_CTRL of CE1/CE2 (hal.h:203-210). */
static bool
model_reg_ok(uint32_t off)
{
	static const uint32_t host[] = {0x00, 0x04, 0x08, 0x10, 0x1c, 0x20, 0x30, 0x34};
	static const uint32_t sts[] = {0x00, 0x04, 0x08, 0x10, 0x14, 0x18, 0x24};
	for (unsigned i = 0; i < 7; i++) {
		const struct dring *r = &dr[i];
		const uint32_t *set = r->kind == K_STS ? sts : host;
		unsigned n = r->kind == K_STS ? 7 : 8;
		for (unsigned j = 0; j < n; j++) { if (off == r->r0 + set[j]) { return true; } }
		if (off == r->r2 || off == r->r2 + 4) { return true; }
		if (r->kind == K_DST && off == r->r0 + 0xb0) { return true; }
	}
	return false;
}

static void q_push(unsigned ce, uint32_t delay, const uint8_t *d, uint32_t len, int stslen);
static void fw_rx(unsigned ce, uint32_t eid, const uint8_t *d, uint32_t len);
static void fw_rings_ready(void);

static void
write_rdp(struct dring *r, uint32_t v)
{
	p32(dev_mem(r->ptr_addr, 4, 0, 0x1000, "rdp word outside the rdp page"), v);
}

/* Host publishes a pointer of a live ring. */
static void
dev_publish(struct dring *r, uint32_t v)
{
	CHECK(v < r->size && v % r->edw == 0, "published pointer outside the ring or misaligned");
	if (r->kind == K_SRC) {
		/* TX slots: CE0 16 x 2 KiB, CE3 32 x 2 KiB, CE4 16 x 256 B (C2 HTT only) */
		uint32_t pipe_lo = r->ce == 0 ? 0x10000u : r->ce == 3 ? 0x18000u : 0x66000u;
		uint32_t pipe_hi = r->ce == 0 ? 0x18000u : r->ce == 3 ? 0x28000u : 0x67000u, bsz = r->ce == 4 ? 256u : WLCE_BUF;
		CHECK(r->ce == 0 || r->ce == 3 || r->ce == 4, "TX on a pipe without TX slots");
		while (r->dev != v) {
			uint8_t *e = dev_mem(r->base + 4ull * r->dev, 16, 0x1000, 0x10000, "SRC descriptor outside the rings");
			uint32_t w0 = g32(e), w1 = g32(e + 4), w2 = g32(e + 8), len = w1 >> 16;
			uint64_t a = w0 | (uint64_t)(w1 & 0xff) << 32;
			CHECK((w1 & 0xff00u) == 0, "SRC descriptor byte swap/gather/hash bits set");
			CHECK(len && len <= bsz, "SRC length 0 or above the pipe's buffer size");
			CHECK(a == arena_iova + pipe_lo + (uint64_t)(r->dev / 4 % ((pipe_hi - pipe_lo) / bsz)) * bsz,
			    "SRC buffer is not the TX slot of its entry");
			uint8_t *buf = dev_mem(a, len, pipe_lo, pipe_hi, "TX read outside the pipe's TX slots");
			uint8_t copy[WLCE_BUF];
			memcpy(copy, buf, len);
			r->dev = (r->dev + 4) % r->size;
			/* stop_tx_stuck / stats_tx_stuck: the STOP_SCAN / REQUEST_STATS descriptor's completion never shows in the
			 * rdp TP word */
			bool stuck = r->ce == 3 && len >= 12 && (((C.stop_tx_stuck && (g32(copy + 8) & 0xffffffu) == 0x3002u)) ||
			    (C.stats_tx_stuck && (g32(copy + 8) & 0xffffffu) == 0x16001u));
			if (!stuck) { write_rdp(r, r->dev); }
			fw_tx_msgs++;
			fw_rx(r->ce, w2 & 0xffffu, copy, len);
		}
	} else if (r->kind == K_DST) {
		CHECK(r->ce == 1 || r->ce == 2, "RX post on a pipe without RX buffers");
		r->host = v;
		fw_rings_ready();
	} else {
		uint32_t old = r->host;
		CHECK((v + r->size - old) % r->size <= (r->dev + r->size - old) % r->size, "STATUS TP passed the device HP");
		r->host = v;
	}
}

static void
ce_access(uint32_t off, bool wr, uint32_t *v, bool shadow)
{
	uint32_t rel0 = 0, rel2 = 0;
	CHECK(off >= CE_BASE && off < CE_END && !(off & 3) && model_reg_ok(off), "CE register outside the C1 set");
	struct dring *r = ring_of_reg(off, &rel0, &rel2);
	uint32_t *reg = &ce_reg[(off - CE_BASE) / 4];
	if (!wr) { *v = *reg; ce_reads++; return; }
	if (C.fault_after && ce_writes >= C.fault_after) { fail("a CE register write after the latched fault"); }
	if (!r) {  /* DEST_CTRL: after its DST ring went live, max length = src_sz_max (hal.c:2437-2442, ce.c:597-598) */
		const struct dring *d = off == dr[1].r0 + 0xb0 ? &dr[1] : &dr[3];
		CHECK(off == d->r0 + 0xb0 && d->enabled, "DEST_CTRL written before its DST ring went live");
		CHECK((*v & 0xffffu) == 2048u, "DEST_CTRL max length is not 2048");
	}
	if (r || !C.dest_ctrl_ignore) { *reg = *v; }  /* dest_ctrl_ignore: the register does not take the write */
	ce_writes++;
	if (C.fault_after && ce_writes == C.fault_after) { fw_faults++; }
	if (!r) { return; }
	if (rel0 == 0x10) {  /* MISC: the ring goes live; its setup must be L19's */
		uint32_t base_lsb = ce_reg[(r->r0 - CE_BASE) / 4], base_msb = ce_reg[(r->r0 + 4 - CE_BASE) / 4];
		uint32_t id = ce_reg[(r->r0 + 8 - CE_BASE) / 4];
		CHECK(!r->enabled, "MISC written twice");
		r->base = base_lsb | (uint64_t)(base_msb & 0xff) << 32;
		r->size = base_msb >> 8;
		CHECK(r->size == r->entries * r->edw, "ring size in BASE_MSB is not entries x entry size");
		CHECK(r->base % 8 == 0, "ring base not 8-byte aligned (ce.h:108)");
		(void)dev_mem(r->base, r->size * 4, 0x1000, 0x10000, "ring memory outside the ring area");
		if (r->kind == K_STS) {
			CHECK(id == (r->id << 8 | 4), "STATUS ring ID register is not id << 8 | 4");
			CHECK(ce_reg[(r->r0 + 0x24 - CE_BASE) / 4] == 0, "STATUS producer thresholds set");
			r->ptr_addr = ce_reg[(r->r0 + 0x14 - CE_BASE) / 4] | (uint64_t)ce_reg[(r->r0 + 0x18 - CE_BASE) / 4] << 32;
			CHECK(*v == 0x40, "STATUS MISC is not SRNG_ENABLE");
		} else {
			CHECK(id == r->edw, "ring ID register is not the entry size");
			CHECK(ce_reg[(r->r0 + 0x30 - CE_BASE) / 4] == 0 && ce_reg[(r->r0 + 0x34 - CE_BASE) / 4] == 0, "consumer thresholds set");
			r->ptr_addr = ce_reg[(r->r0 + 0x1c - CE_BASE) / 4] | (uint64_t)ce_reg[(r->r0 + 0x20 - CE_BASE) / 4] << 32;
			CHECK(*v == 0x42, "MISC is not LOOPCNT_DISABLE | SRNG_ENABLE");
		}
		CHECK(r->ptr_addr == arena_iova + 4ull * r->id, "pointer address is not rdp + 4 x ring id");
		CHECK(ce_reg[(r->r2 - CE_BASE) / 4] == 0 && ce_reg[(r->r2 + 4 - CE_BASE) / 4] == 0, "R2 not zeroed before enable");
		CHECK(g32(dev_mem(r->ptr_addr, 4, 0, 0x1000, "rdp")) == 0, "rdp word not zeroed before enable");
		for (uint32_t i = 0; i < r->size; i++) { CHECK(g32(dev_mem(r->base + 4ull * i, 4, 0x1000, 0x10000, "ring")) == 0, "ring memory not zeroed"); }
		r->enabled = true;
		return;
	}
	if (rel2 == ~0u) { CHECK(!r->enabled, "R0 written after the ring went live"); return; }
	if (!r->enabled) { CHECK(*v == 0, "R2 written non-zero before enable"); return; }
	CHECK(rel2 == (r->kind == K_STS ? 4u : 0u), "the host wrote the device's pointer");
	model_event('P');
	if (shadow) { shadow_publishes++; } else { direct_publishes++; }
	CHECK(shadow == !wl15_variant, "pointer published by the wrong route for the shadow mode");
	dev_publish(r, *v);
	if (r->kind == K_DST && r->ce == 1 && dr[1].enabled && dr[3].enabled) { fw_rings_ready(); }
}

/* ---- RX delivery ---- */
static bool
deliver(const struct qmsg *m)
{
	struct dring *d = &dr[m->ce == 1 ? 1 : 3], *s = &dr[m->ce == 1 ? 2 : 4];
	uint32_t lo = m->ce == 1 ? 0x28000u : 0x47000u, hi = lo + 62u * WLCE_BUF;
	if (!d->enabled || !s->enabled || d->dev == d->host || (s->dev + 4) % s->size == s->host) { return false; }
	uint8_t *e = dev_mem(d->base + 4ull * d->dev, 8, 0x1000, 0x10000, "DST descriptor");
	CHECK((g32(e + 4) & ~0xffu) == 0, "DST descriptor word 1 carries more than address bits 39:32 (hal.c:1941-1947)");
	uint64_t a = g32(e) | (uint64_t)(g32(e + 4) & 0xff) << 32;
	CHECK((a - arena_iova - lo) % WLCE_BUF == 0, "RX buffer not on a 2 KiB slot");
	CHECK(m->len <= (ce_reg[(d->r0 + 0xb0 - CE_BASE) / 4] & 0xffffu), "message above DEST_CTRL max length");
	uint8_t *buf = dev_mem(a, m->len, lo, hi, "RX write outside the pipe's RX buffers");
	uint32_t slot = (uint32_t)(a - arena_iova);
	(void)dev_mem(a, m->len, slot, slot + 2048u, "RX write past its own 2 KiB buffer");
	memcpy(buf, m->d, m->len);
	d->dev = (d->dev + 2) % d->size;
	write_rdp(d, d->dev);
	uint8_t *st = dev_mem(s->base + 4ull * s->dev, 16, 0x1000, 0x10000, "STATUS entry");
	p32(st, (uint32_t)(m->stslen >= 0 ? (uint32_t)m->stslen : m->len) << 16);
	p32(st + 4, 0); p32(st + 8, 0); p32(st + 12, 0);
	s->dev = (s->dev + 4) % s->size;
	write_rdp(s, s->dev);
	return true;
}

static void
dev_tick(void)
{
	while (q_head != q_tail && q[q_head % 512].due <= now && deliver(&q[q_head % 512])) { q_head++; }
	if (C.rdp_glitch == 1 && fw_ready_sent && q_head) { write_rdp(&dr[4], 4096); }
	if (C.rdp_glitch == 2 && fw_ready_sent && q_head) { write_rdp(&dr[4], 63u * 4u); }
	/* TX TP backwards, once the host has seen the CE0 TP at 8 (two messages done) */
	if (C.rdp_glitch == 3 && wl_ce.pipe[0].src.tp == 8 && dr[0].dev == 8) { write_rdp(&dr[0], 4); dr[0].dev = 4; }
	/* CE2 STATUS HP moved by a delta once the host consumed 3 entries (READY, 2 connect replies) */
	if (C.sts_rdp_delta && !sts_glitched && wl_ce.pipe[2].sts.tp == 12 && dr[4].dev == 12) {
		write_rdp(&dr[4], (uint32_t)((int)dr[4].size + 12 + C.sts_rdp_delta) % dr[4].size);
		sts_glitched = true;
	}
}

static void
q_push(unsigned ce, uint32_t delay, const uint8_t *d, uint32_t len, int stslen)
{
	CHECK(q_tail - q_head < 512 && len <= WLCE_BUF, "model queue");
	struct qmsg *m = &q[q_tail++ % 512];
	m->ce = ce; m->due = now + delay; m->len = len; m->stslen = stslen;
	memcpy(m->d, d, len);
}

/* ---- firmware ---- */
static uint8_t mb[WLCE_BUF];
static uint32_t ml;
static void m_reset(void) { ml = 0; memset(mb, 0, sizeof(mb)); }
static void m_u32(uint32_t v) { p32(mb + ml, v); ml += 4; }
static void m_tlv(uint32_t tag, uint32_t len) { m_u32(tag << 16 | len); }
static void m_zero(uint32_t n) { ml += n; }

/* HTC message on CE2 (WMI/control downlink per the service map) or CE1. */
static void
htc_out(unsigned ce, uint32_t eid, const uint8_t *pl, uint32_t plen, int refund_eid, unsigned refund, uint32_t delay)
{
	uint8_t d[WLCE_BUF];
	uint32_t tlen = refund_eid >= 0 ? 8 : 0;
	p32(d, eid | (tlen ? 2u : 0u) << 8 | (plen + tlen) << 16);
	p32(d + 4, tlen);
	memcpy(d + 8, pl, plen);
	if (tlen) {
		uint8_t t[8] = {1, 4, 0, 0, (uint8_t)refund_eid, (uint8_t)refund, 0, 0};
		memcpy(d + 8 + plen, t, 8);
		fw_credits += (int)refund;  /* granted now, usable by the host once received */
	}
	q_push(ce, delay, d, 8 + plen + tlen, -1);
}

static void
wmi_out(uint32_t id, unsigned refund, uint32_t delay)
{
	uint8_t pl[WLCE_BUF];
	p32(pl, id);
	memcpy(pl + 4, mb, ml);
	htc_out(2, C.wmi_eid, pl, 4 + ml, refund ? (int)C.wmi_eid : -1, refund, delay);
}

static void
fw_rings_ready(void)
{
	if (fw_ready_sent || C.silent_ready || !dr[1].enabled || !dr[3].enabled || dr[1].host == 0 || dr[3].host == 0) { return; }
	fw_ready_sent = true;
	uint8_t d[WLCE_BUF];
	uint32_t w0 = 1u | C.credits << 16, w1 = C.credit_size | 9u << 16;
	if (C.empty_msgs) {  /* an empty HTC message on EP0, no trailer: L19 drops it (htc.c:321-323) */
		p32(d, 0); p32(d + 4, 0);
		q_push(C.ready_ce, 2, d, 8, -1);
	}
	if (C.ready_trailer_long || C.ready_trailer_zero) {
		p32(d, C.ready_eid | 2u << 8 | 8u << 16); p32(d + 4, C.ready_trailer_zero ? 0u : 12u); p32(d + 8, w0); p32(d + 12, w1);
		q_push(C.ready_ce, 3, d, 16, -1);
		return;
	}
	p32(d, C.ready_eid | 8u << 16); p32(d + 4, 0); p32(d + 8, w0); p32(d + 12, w1);
	q_push(C.ready_ce, 3, d, 16, C.ready_stslen);
}

/* One MAC/PHY capability TLV (108 bytes): hw mode, pdev (1 for SINGLE; C.dbs_pdev + phy for DBS, each MAC its own
 * as on WCN7850, or C.dbs_pdev for both), phy id, supported_bands (word 4), the 2 GHz (12/13) and 5 GHz (20/21)
 * tx/rx chain masks, the rest 0 (wmi.h ath12k_wmi_mac_phy_caps_params). */
static void
m_mac_phy(uint32_t hw_mode, uint32_t phy, uint32_t bands, uint32_t mask)
{
	m_tlv(0x210, 108); m_u32(hw_mode); m_u32(hw_mode ? C.dbs_pdev + (C.dbs_same_pdev ? 0u : phy) : 1u); m_u32(phy); m_u32(0);
	m_u32(bands); m_zero(7 * 4);
	m_u32(mask); m_u32(mask); m_zero(6 * 4); m_u32(mask); m_u32(mask); m_zero(5 * 4);
}

static void
fw_service_ready_set(void)
{
	if (C.empty_msgs) {  /* an empty HTC message on the WMI endpoint, no trailer: dropped as on EP0 */
		uint8_t d[8];
		p32(d, C.wmi_eid); p32(d + 4, 0);
		q_push(2, 1, d, 8, -1);
	}
	m_reset();  /* SERVICE_READY: TLV 0x20 (128 bytes), base bitmap with service 77 */
	m_tlv(0x20, 128);
	m_u32(0x12345678); m_zero(6 * 4); m_u32(0x7); m_u32(0); m_u32(fw_chains); m_zero(8 * 4); m_u32(C.mem_reqs);
	m_u32(64); m_zero(12 * 4);
	m_tlv(0x10, 128);
	for (unsigned i = 0; i < 32; i++) { m_u32(i == 19 ? 0x2u : 0u); }
	if (C.svc_tlv_past) { m_tlv(0x10, 0x100); m_u32(0); }
	if (C.bad_report) {
		uint8_t pl[WLCE_BUF], d[WLCE_BUF];
		p32(pl, 0x1); memcpy(pl + 4, mb, ml);
		uint32_t plen = 4 + ml;
		const uint8_t t[12] = {1, 8, 0, 0, (uint8_t)C.wmi_eid, 1, 0, 0, 12, 5, 0, 0};
		p32(d, C.wmi_eid | 2u << 8 | (plen + 12) << 16); p32(d + 4, 12);
		memcpy(d + 8, pl, plen); memcpy(d + 8 + plen, t, 12);
		fw_credits += 1;  /* the endpoint 12 report is never applied (htc.c:161-162) */
		q_push(2, 2, d, 8 + plen + 12, -1);
	} else {
		wmi_out(0x1, 0, 2);
	}
	m_reset();  /* SERVICE_AVAILABLE: 220 in segment word 2 bit 28, 281 in ext word 0 bit 25 */
	m_tlv(0x22f, 20); m_u32(0); m_u32(0); m_u32(0); m_u32(C.no_ext2_service ? 0u : 1u << 28); m_u32(0);
	m_tlv(0x10, 16); m_u32(1u << 25); m_u32(0); m_u32(0); m_u32(0);
	fw_svc281 = true;
	wmi_out(0x3, 0, 3);
	/* SERVICE_READY_EXT: fixed, 2 hw modes, SINGLE (PHY map 0x1) then DBS (0x3), and the 3 MAC/PHY caps that
	 * follow from them, pdev 1 for SINGLE, C.dbs_pdev (+ 1 for PHY 1) for DBS (m_mac_phy). SINGLE: both bands,
	 * 1x1. DBS PHY 0: 2 GHz (of C.phy_bands), mask C.phy_mask & 1; DBS PHY 1: 5 GHz (of C.phy_bands), mask
	 * C.phy_mask. ext_single_last lists DBS first; C2b: ext_single_only lists SINGLE and its one MAC/PHY cap. */
	unsigned nmodes = C.ext_single_only ? 1u : 2u, ncaps = C.ext_single_only ? 1u : 3u;
	m_reset();
	m_tlv(0x1ac, 76); m_zero(76);
	m_tlv(0x212, 8); m_u32(nmodes); m_u32(0);
	m_tlv(0x12, nmodes * 16);
	for (unsigned i = 0; i < nmodes; i++) {
		unsigned mode = C.ext_single_last ? 1 - i : i;  /* 0 SINGLE, 1 DBS */
		m_tlv(0x211, 12); m_u32(mode); m_u32(mode ? 3u : 1u); m_u32(mode);
	}
	if (C.ext_nested_overrun) {
		m_tlv(0x12, 40u); m_tlv(0x210, 108u); m_u32(0); m_u32(1); m_zero(28u);
	} else {
		m_tlv(0x12, ncaps * (4u + 108u));
		for (unsigned i = 0; i < ncaps; i++) {
			bool single = C.ext_single_last ? i == 2 : i == 0;
			unsigned phy = single ? 0u : C.ext_single_last ? i : i - 1;
			m_mac_phy(single ? 0u : 1u, phy, single ? 3u : C.phy_bands & (phy ? 2u : 1u), single ? 1u : phy ? C.phy_mask : C.phy_mask & 1u);
		}
	}
	fw_ext_q = q_tail;  /* queue position: delivered once q_head passed it */
	wmi_out(0x4009, 0, C.ext_delay ? C.ext_delay : 4);
	if (!C.no_ext2_service) {
		m_reset();
		m_tlv(0x334, 56); m_zero(56); m_tlv(0x12, 0);
		fw_ext2_q = q_tail;
		wmi_out(0x4022, 0, 5);
	}
}

/* INIT checked field by field against L19 (wmi.c:260-304, 3941-4124, wmi.h:2512-2605), coded here again. */
static void
fw_check_init(const uint8_t *p, uint32_t len)
{
	uint32_t want[73] = {0};
	CHECK(len == 332 && g32(p) == 0x1, "INIT length/command id");
	CHECK(g32(p + 4) == (0x4au << 16 | 28u), "INIT_CMD TLV header");
	for (unsigned i = 8; i < 36; i++) { CHECK(p[i] == 0, "INIT host abi / num_host_mem_chunks not 0"); }
	const uint8_t *rc = p + 36;
	CHECK(g32(rc) == (0x4bu << 16 | 288u), "RESOURCE_CONFIG TLV header");
	want[1] = 4; want[2] = 16; want[3] = 3; want[4] = 3; want[5] = 2; want[6] = 32; want[7] = 16;
	want[8] = want[9] = (1u << fw_chains) - 1; want[10] = want[11] = want[12] = 100; want[13] = 40;
	want[14] = 1; want[15] = 4; want[16] = 3; want[17] = 3; want[18] = 8; want[22] = 1024; want[28] = 2;
	want[29] = 0x400; want[30] = 0xa; want[31] = 1; want[32] = 8; want[33] = 2; want[34] = 0x20; want[35] = 0x16;
	want[44] = 1u << 5 | 1u << 18; want[54] = 1; want[57] = 1; want[68] = 1u << 9; want[69] = fw_svc281 ? 1u << 4 : 0;
	for (unsigned i = 1; i < 73; i++) {
		if (g32(rc + 4 * i) != want[i]) { fprintf(stderr, "INIT word %u = %#x, want %#x\n", i, g32(rc + 4 * i), want[i]); fail("INIT resource config"); }
	}
	CHECK(g32(rc + 292) == (0x12u << 16), "INIT trailing ARRAY_STRUCT not empty");
	fw_init_ok++;
}

/* ---- C2 firmware: values written down here again from L19 and the owner's channel list ---- */
static uint32_t
m_chan(unsigned i)  /* 2.4 GHz 1-13, then 36-48, 52-64, 100-144, 149-165 */
{
	static const uint32_t five[25] = {5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320, 5500, 5520, 5540, 5560, 5580,
	                                  5600, 5620, 5640, 5660, 5680, 5700, 5720, 5745, 5765, 5785, 5805, 5825};
	return i < 13 ? 2412u + 5u * i : five[i - 13];
}
static uint32_t m_chan_no(uint32_t mhz) { return mhz < 3000 ? (mhz - 2407) / 5 : (mhz - 5000) / 5; }

/* a credit report in a trailer-only HTC message, DELAY ms later */
static void
fw_refund(unsigned need, uint32_t delay)
{
	uint8_t d[16] = {0, 0, 0, 0, 8, 0, 0, 0, 1, 4, 0, 0, (uint8_t)C.wmi_eid, (uint8_t)need, 0, 0};
	p32(d, C.wmi_eid | 2u << 8 | 8u << 16);
	fw_credits += (int)need;
	q_push(2, delay, d, 16, -1);
}

static void
fw_scan_event(uint32_t type, uint32_t reason, uint32_t freq, uint32_t vdev, uint32_t at)
{
	m_reset(); m_tlv(0x24, 28); m_u32(type); m_u32(reason); m_u32(freq); m_u32(1); m_u32(0xa000); m_u32(vdev); m_u32(at);
	if (C.bad_scan_tlv && type == 8 && freq == 2437) { mb[0] = 0x40; }  /* TLV length 64: past the event */
	wmi_out(0x3001, 0, at);
}

/* CHAN_INFO {err 0, freq, flags, nf, rx_clear, cycle, tx power range 0, tp 0, rx_frame, my_bss_rx 0, 11b 0, tx_frame 0,
 * mac_clk, vdev 0} (wmi.h:4586-4601). C2b: a START_RESP carries the dwell's MAC-clock counters as L19 reads them
 * (survey time = cycle / (mac_clk x 1000) ms, wmi.c:7546-7561): 320 MHz, 150 ms less a few us per channel, 3 % busy
 * plus 300 us of frame per frame heard there, nothing sent; an END_RESP (L19 only notes it) carries none. */
static void
fw_chan_info(uint32_t flags, uint32_t freq, int32_t nf, unsigned idx, unsigned frames, uint32_t at)
{
	uint32_t cycle = 48000000u - 13000u * (idx % 7u), rx = 96000u * frames, busy = 1440000u + rx + 4000u * idx;
	m_reset(); m_tlv(0x26, 56); m_u32(0); m_u32(freq); m_u32(flags); m_u32((uint32_t)nf);
	m_u32(flags ? 0u : busy); m_u32(flags ? 0u : cycle); m_zero(2 * 4); m_u32(flags ? 0u : rx); m_zero(3 * 4);
	m_u32(flags ? 0u : 320u); m_u32(0);
	wmi_out(0x4002, 0, at);
}

/* MGMT_RX: TLV 0x2c (68 bytes) + the frame in an ARRAY_BYTE padded to 4. Modes: 1 no frame TLV, 2 buf_len
 * past the byte array, 3 header TLV of 64 bytes. */
static void
fw_mgmt_rx(const struct mbeacon *b, uint32_t at)
{
	m_reset();
	m_tlv(0x2c, b->mode == 3 ? 64u : 68u);
	m_u32(b->rx_channel); m_u32(b->snr); m_u32(6000); m_u32(b->rx_channel <= 14 ? 1u : 0u);
	m_u32(b->mode == 2 ? b->len + 8 : b->len); m_u32(b->status); m_zero(16); m_u32(0); m_u32(b->snr - 95); m_zero(12);
	m_u32(1);
	if (b->mode != 3) { m_u32(b->chan_freq); }
	if (b->mode != 1) {
		uint32_t pad = (b->len + 3) & ~3u;
		m_tlv(0x11, pad); memcpy(mb + ml, b->f, b->len); ml += pad;
	}
	wmi_out(0x7001, 0, at);
}

/* An 802.11 management frame: subtype 8 beacon, 5 probe response, 4 probe request. SSID_LEN < 0: no SSID
 * element; SSID NULL with a length: that many zero bytes. */
static uint32_t
mk_frame(uint8_t *f, unsigned sub, const uint8_t *bssid, uint16_t cap, const char *ssid, int ssid_len, unsigned ds,
    unsigned ht, bool rsn)
{
	static const uint8_t rsn_ie[20] = {1, 0, 0x00, 0x0f, 0xac, 4, 1, 0, 0x00, 0x0f, 0xac, 4, 1, 0, 0x00, 0x0f, 0xac, 2, 0, 0};
	uint32_t o = 36;
	memset(f, 0, 400);
	f[0] = (uint8_t)(sub << 4);
	memset(f + 4, 0xff, 6); memcpy(f + 10, bssid, 6); memcpy(f + 16, bssid, 6);
	f[32] = 100; f[34] = (uint8_t)cap; f[35] = (uint8_t)(cap >> 8);
	if (ssid_len >= 0) { f[o++] = 0; f[o++] = (uint8_t)ssid_len; if (ssid) { memcpy(f + o, ssid, (size_t)ssid_len); } o += (uint32_t)ssid_len; }
	f[o++] = 1; f[o++] = 4; f[o++] = 0x82; f[o++] = 0x84; f[o++] = 0x8b; f[o++] = 0x96;
	if (ds) { f[o++] = 3; f[o++] = 1; f[o++] = (uint8_t)ds; }
	if (rsn) { f[o++] = 48; f[o++] = 20; memcpy(f + o, rsn_ie, 20); o += 20; }
	if (ht) { f[o++] = 61; f[o++] = 22; f[o++] = (uint8_t)ht; o += 21; }
	return o;
}

static struct mbeacon *
add_frame(unsigned idx, uint32_t snr, unsigned sub, const uint8_t *bssid, uint16_t cap, const char *ssid, int ssid_len,
    unsigned ds, unsigned ht, bool rsn)
{
	CHECK(n_beacons < sizeof(beacons) / sizeof(beacons[0]), "model beacon table");
	struct mbeacon *b = &beacons[n_beacons++];
	memset(b, 0, sizeof(*b));
	b->idx = idx; b->snr = snr; b->chan_freq = m_chan(idx); b->rx_channel = m_chan_no(b->chan_freq);
	b->len = mk_frame(b->f, sub, bssid, cap, ssid, ssid_len, ds, ht, rsn);
	return b;
}

/* START_SCAN accepted: the scan as a script in time order. Per channel I (150 ms): FOREIGN_CHAN at t0,
 * CHAN_INFO START at t0 + 1, the frames heard there from t0 + 2, CHAN_INFO END at t0 + 148. */
static void
fw_scan_script(unsigned n)
{
	uint32_t t = 2;
	if (C.scan_mode == 1) { fw_scan_event(1u << 6, 4, 0, 0, t); return; }
	if (C.scan_mode == 2) { return; }
	if (C.scan_mode == 4) { fw_scan_event(1u << 1, 0, 0, 0, t); return; }
	fw_scan_event(1u << 0, 0, 0, 0, t);
	if (C.wrong_vdev) { fw_scan_event(1u << 3, 0, 2412, 3, t); }
	for (unsigned i = 0; i < n; i++) {
		uint32_t t0 = 5 + 150 * i, k = 0, heard = 0;
		for (unsigned j = 0; j < n_beacons; j++) { heard += beacons[j].idx == i; }
		fw_scan_event(1u << 3, 0, m_chan(i), 0, t0);
		for (unsigned r = 0; r <= C.ci_repeat; r++) { fw_chan_info(0, m_chan(i), -96 - (int32_t)(i % 5), i, heard, t0 + 1); }
		if (C.ci_repeat && i == 0) { fw_chan_info(2, m_chan(i), -96, i, heard, t0 + 1); }  /* C2b: flags neither START nor END */
		if (i == 2) {  /* an unprompted event mid-scan: logged by id only */
			m_reset(); m_tlv(0x400, 12); m_zero(12); wmi_out(0x402b, 0, t0 + 1);
		}
		for (unsigned j = 0; j < n_beacons; j++) {
			if (beacons[j].idx == i) { fw_mgmt_rx(&beacons[j], t0 + 2 + k++); }
		}
		fw_chan_info(1, m_chan(i), 0, i, 0, t0 + 148);
	}
	if (C.scan_mode != 3) { fw_scan_event(1u << 1, 0, 0, 0, 5 + 150 * n + 1); }
}

static void
fw_c2_cmd(uint32_t id, const uint8_t *pl, uint32_t plen, unsigned need)
{
	const uint8_t *v = pl + 8;   /* the fixed TLV's value */
	uint32_t exp_n = C.exp_n, pdev = C.exp_pdev;
	if (id == 0x3003) {  /* SCAN_CHAN_LIST (wmi.c:2880-2997, reg.c:183-238) */
		CHECK(fw_init_ok == 1 && (fw_hw_mode_ok == 1 || (wl20_bits & WL20_NOHWMODE)), "channel list before INIT/HW mode");
		CHECK(!(wl20_bits & WL20_HTTVER) || fw_htt_req == 1, "channel list before the HTT version exchange");
		CHECK(fw_c2_step == 0, "channel list out of order");
		uint32_t k = g32(v), flags = g32(v + 4);
		CHECK(g32(pl + 4) == (0x4fu << 16 | 12u) && k && fw_chan_seen + k <= exp_n && g32(v + 8) == pdev,
		    "SCAN_CHAN_LIST fixed TLV (tag, count, pdev)");
		CHECK(flags == (fw_chan_cmds ? 1u : 0u), "SCAN_CHAN_LIST APPEND flag only after the first command");
		CHECK(plen == 20 + 4 + 28 * k && g32(pl + 20) == (0x12u << 16 | (28 * k - 4)), "SCAN_CHAN_LIST array header (L19: 4 short)");
		for (uint32_t i = 0; i < k; i++) {
			const uint8_t *c = pl + 24 + 28 * i;
			uint32_t mhz = m_chan(fw_chan_seen + i);
			bool dfs = (mhz >= 5260 && mhz <= 5320) || (mhz >= 5500 && mhz <= 5720);
			uint32_t info = (mhz < 3000 ? 1u : 0u) | 1u << 7 | 1u << 17 | (dfs ? 1u << 10 : 0u);
			CHECK(g32(c) == (0x50u << 16 | 24u) && g32(c + 4) == mhz && g32(c + 8) == mhz && g32(c + 12) == 0, "channel TLV/mhz/cfreq");
			CHECK(g32(c + 16) == info, "channel info is not MODE | PASSIVE | ALLOW_HE (| DFS)");
			CHECK(g32(c + 20) == (40u << 8 | 40u << 16) && g32(c + 24) == 40u << 8, "channel reg_info");
		}
		fw_chan_seen += k; fw_chan_cmds++;
		if (fw_chan_seen == exp_n) { fw_c2_step = 1; }
	} else if (id == 0x5001) {  /* VDEV_CREATE (wmi.c:923-1010, mac.c:9727-9780) */
		static const uint8_t mac[8] = {0x02, 0x46, 0x55, 0x58, 0x49, 0x01, 0, 0};
		uint32_t tx = (uint32_t)__builtin_popcount(C.phy_mask), want_tx[2];
		if (!C.phy_bands || !tx) { tx = fw_chains; }
		uint32_t bands = C.phy_bands && C.phy_mask ? C.phy_bands : 3u;
		want_tx[0] = bands & 1 ? tx : 0; want_tx[1] = bands & 2 ? tx : 0;
		CHECK(fw_c2_step == 1, "VDEV_CREATE before the whole channel list");
		CHECK(plen == 88 && g32(pl + 4) == (0x56u << 16 | 44u) && g32(v) == 0 && g32(v + 4) == 2 && g32(v + 8) == 0,
		    "VDEV_CREATE length/tag/vdev 0/type STA/subtype 0");
		CHECK(!memcmp(v + 12, mac, 8), "VDEV_CREATE MAC is not the K-W2 address + 2 pad");
		CHECK(g32(v + 20) == 2 && g32(v + 24) == pdev && g32(v + 28) == 1 && g32(v + 32) == 0 && g32(v + 36) == 1 &&
		    g32(v + 40) == 0, "VDEV_CREATE streams/pdev/mbssid/stats id");
		CHECK(g32(pl + 52) == (0x12u << 16 | 32u), "VDEV_CREATE txrx streams array header");
		for (unsigned b = 0; b < 2; b++) {
			const uint8_t *s = pl + 56 + 16 * b;
			CHECK(g32(s) == (0x19bu << 16 | 12u) && g32(s + 4) == b && g32(s + 8) == want_tx[b] && g32(s + 12) == want_tx[b],
			    "VDEV_TXRX_STREAMS");
		}
		fw_c2_step = 2;
	} else if (id == 0x5008) {  /* VDEV_SET_PARAM NSS (mac.c:10091-10093) */
		uint32_t nss = C.phy_bands && C.phy_mask ? (uint32_t)__builtin_popcount(C.phy_mask) : fw_chains;
		CHECK(fw_c2_step == 2 && plen == 20 && g32(pl + 4) == (0x5fu << 16 | 12u) && g32(v) == 0 && g32(v + 4) == 0x22 &&
		    g32(v + 8) == nss, "VDEV_SET_PARAM is not NSS after VDEV_CREATE");
		fw_c2_step = 3;
	} else if (id == 0x9002) {  /* STA_POWERSAVE_PARAM: RX wake WAKE, TX wake ALWAYS, PS-Poll NO_MAX (mac.c:10121-10149) */
		static const uint32_t want[3][2] = {{0, 0}, {1, 1}, {2, 0}};
		unsigned i = fw_c2_step - 3;
		CHECK(fw_c2_step >= 3 && fw_c2_step <= 5 && plen == 20 && g32(pl + 4) == (0x6du << 16 | 12u) && g32(v) == 0 &&
		    g32(v + 4) == want[i][0] && g32(v + 8) == want[i][1], "STA_POWERSAVE_PARAM order/values");
		fw_c2_step++;
	} else if (id == 0x9001) {  /* STA_POWERSAVE_MODE disabled (mac.c:10151) */
		CHECK(fw_c2_step == 6 && plen == 16 && g32(pl + 4) == (0x6cu << 16 | 8u) && g32(v) == 0 && g32(v + 4) == 0,
		    "STA_POWERSAVE_MODE is not off after the three parameters");
		fw_c2_step = 7;
	} else if (id == 0x3001) {  /* START_SCAN (wmi.c:2541-2830, mac.c:5335-5366) */
		uint32_t want[40] = {0};
		want[0] = 0x4du << 16 | 156u; want[1] = 0xa000; want[2] = 1; want[3] = 0; want[4] = 0; want[5] = 0x5f;
		want[6] = 50; want[7] = 150; want[8] = 50; want[9] = 500; want[13] = exp_n * 150 * 3 / 2; want[14] = 5;
		want[15] = 0x11; want[17] = exp_n; want[18] = 1; want[36] = 0; want[37] = 70; want[38] = 70;
		CHECK(fw_c2_step == 7, "START_SCAN before the vdev was set up");
		CHECK(plen == 4 + 160 + 4 + 4 * exp_n + 4 + 4 + 8 + 4, "START_SCAN length");
		for (unsigned i = 0; i < 40; i++) {
			if (g32(pl + 4 + 4 * i) != want[i]) { fprintf(stderr, "START_SCAN word %u = %#x, want %#x\n", i, g32(pl + 4 + 4 * i), want[i]); fail("START_SCAN"); }
		}
		const uint8_t *p = pl + 164;
		CHECK(g32(p) == (0x10u << 16 | 4 * exp_n), "START_SCAN channel array header");
		for (unsigned i = 0; i < exp_n; i++) { CHECK(g32(p + 4 + 4 * i) == m_chan(i), "START_SCAN channel"); }
		p += 4 + 4 * exp_n;
		CHECK(g32(p) == (0x13u << 16), "START_SCAN SSID array is not empty");
		CHECK(g32(p + 4) == (0x13u << 16 | 8u) && !memcmp(p + 8, "\xff\xff\xff\xff\xff\xff\0\0", 8), "START_SCAN BSSID array");
		CHECK(g32(p + 16) == (0x11u << 16), "START_SCAN IE array is not empty");
		fw_scan_cmds++; fw_c2_step = 8;
		if (C.guard_after_start) { guard_bad = true; }  /* e.g. AMSS left EE 2 or an SMMU CB fault */
		if (!C.no_start_refund) { fw_refund(need, 1); }
		fw_scan_script(exp_n);
		/* C2b: a credit drought that ends; queued after the script (the queue is delivered in order) */
		if (C.no_start_refund && C.start_refund_at) { fw_refund(need, C.start_refund_at); }
		return;
	} else if (id == 0x3002) {  /* STOP_SCAN (mac.c:4883-4896, wmi.c:2832-2878) */
		CHECK(fw_c2_step == 8 && !wl_ce.stopping, "STOP_SCAN without a scan, or after the latch");
		CHECK(plen == 28 && g32(pl + 4) == (0x4eu << 16 | 20u) && g32(v) == 0 && g32(v + 4) == 0xa000 && g32(v + 8) == 0 &&
		    g32(v + 12) == 0, "STOP_SCAN requester/scan id/STOP_ONE/vdev");
		fw_stop_pdev = g32(v + 16);
		fw_stop_cmds++;
		return;
	} else {
		fail("unexpected C2 WMI command");
	}
	fw_refund(need, 1);
}

/* ---- C2b firmware: REQUEST_STATS and UPDATE_STATS, written down here again from L19 ---- */
/* The pdev entry (struct ath12k_wmi_pdev_stats_params, 44 words: base 0-6 chan_nf, tx_frame_count, rx_frame_count,
 * rx_clear_count, cycle_count, phy_err_count, chan_tx_pwr; tx 7-29 comp_queued .. txop_ovf; rx 30-43
 * mid_ppdu_route_change, status_rcvd, r0..r3_frags, htt_msdus, htt_mpdus, loc_msdus, loc_mpdus, oversize_amsdu,
 * phy_errs, phy_err_drop, mpdu_errs). Realistic for the model's passive scan: nothing sent (tx 0), the last
 * channel's noise floor, about 150 ms of cycles, the frames heard counted as local MPDUs and the CRC one as an MPDU
 * error. DISTINCT: 1000 + word index everywhere, chan_nf -97 (every field's offset shows). */
static void
m_pdev_stats(bool distinct)
{
	uint32_t w[44] = {0};
	if (distinct) {
		for (unsigned i = 1; i < 44; i++) { w[i] = 1000u + i; }
		w[0] = (uint32_t)-97;
	} else {
		w[0] = (uint32_t)-98; w[2] = 2457600; w[3] = 5068800; w[4] = 47961000; w[5] = 112;
		w[31] = 20; w[38] = 19; w[39] = 19; w[41] = 112; w[43] = 1;
	}
	for (unsigned i = 0; i < 44; i++) { m_u32(w[i]); }
}

/* struct wmi_stats_event, LEN bytes (44 = whole): stats id, num pdev, num vdev, peer/bcnflt/chan/mib 0, pdev id
 * (the requested one, fw_stats_pdev), num bcn, the rest 0. */
static void
m_stats_fixed(uint32_t stats_id, uint32_t npdev, uint32_t nvdev, uint32_t nbcn, uint32_t len)
{
	m_tlv(0x46, len); m_u32(stats_id); m_u32(npdev); m_u32(nvdev); m_zero(4 * 4); m_u32(fw_stats_pdev); m_u32(nbcn);
	m_zero(len - 36);
}

/* UPDATE_STATS (0x1d004), the credit back in its trailer. Modes: 0 L19-shaped {fixed TLV: stats id 4, 1 pdev;
 * ARRAY_BYTE of one 176-byte pdev entry}; 1 no answer (the credit in a trailer-only message); 2 the byte array's
 * TLV runs past the event; 3 one pdev announced, 100 bytes given; 4 refused: stats id 0, no entry (L19 never
 * completes fw_stats_done then); 5 L19's order: two vdev (164) and a beacon (12) entry before the pdev entry, distinct
 * values; 6 first a 40-byte fixed TLV (unusable), 3 ms later mode 0; 7 the byte array before the fixed TLV;
 * 8 every pdev word 0x80000000 (each printed field at its widest). The fixed TLV names the requested pdev. */
static void
fw_stats_answer(unsigned need)
{
	m_reset();
	switch (C.stats_mode) {
	case 1: fw_refund(need, 1); return;
	case 2: m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 512); m_pdev_stats(false); break;
	case 3: m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 100); m_zero(100); break;
	case 4: m_stats_fixed(0, 0, 0, 0, 44); m_tlv(0x11, 0); break;
	case 5:
		m_stats_fixed(4, 1, 2, 1, 44); m_tlv(0x11, 2 * 164 + 12 + 176);
		for (unsigned i = 0; i < 82; i++) { m_u32(0xeeeeeeeeu); }
		for (unsigned i = 0; i < 3; i++) { m_u32(0xddddddddu); }
		m_pdev_stats(true);
		break;
	case 6:
		m_stats_fixed(4, 1, 0, 0, 40); m_tlv(0x11, 176); m_pdev_stats(false);
		wmi_out(0x1d004, 0, 2);
		m_reset(); m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 176); m_pdev_stats(false);
		wmi_out(0x1d004, need, 5);
		return;
	case 7: m_tlv(0x11, 176); m_pdev_stats(false); m_stats_fixed(4, 1, 0, 0, 44); break;
	case 8:
		m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 176);
		for (unsigned i = 0; i < 44; i++) { m_u32(0x80000000u); }
		break;
	default: m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 176); m_pdev_stats(false); break;
	}
	wmi_out(0x1d004, need, 2);
}

/* The pdev ids of the preferred hw mode's (DBS) PHYs in PHY order, each once: L19's fw_pdev[] (wmi.c:528-532);
 * SINGLE alone: none, L19's zeroed fw_pdev[0], pdev 0. Returns how many. */
static unsigned
fw_phy_pdevs(uint32_t *out)
{
	if (C.ext_single_only) { out[0] = 0; return 1; }
	out[0] = C.dbs_pdev; out[1] = C.dbs_pdev + 1u;
	return C.dbs_same_pdev ? 1u : 2u;
}

/* REQUEST_STATS (wmi.c:890-920; debugfs.c:1405-1407): {tag 0x8f, 20 bytes: stats id PDEV_STAT 4, vdev 0, peer MAC 0
 * + 2 pad, pdev}, one per pdev of the preferred hw mode's PHYs in PHY order (fw_pdev[0] first, L19's no-vif target;
 * then the one L19 picks by band, mac.c:967-1009), only after COMPLETED and before the containment latch. */
static void
fw_stats_cmd(const uint8_t *pl, uint32_t plen, unsigned need)
{
	uint32_t want[2] = {0, 0};
	unsigned n = fw_phy_pdevs(want);
	CHECK(wl_w23 && fw_c2_step == 8 && wl_scan.done && !wl_ce.stopping && fw_stats_req < n,
	    "REQUEST_STATS before COMPLETED, after the latch or more than once per pdev");
	CHECK(plen == 28 && g32(pl + 4) == (0x8fu << 16 | 20u) && g32(pl + 8) == 4 && g32(pl + 12) == 0 && g32(pl + 16) == 0 &&
	    g32(pl + 20) == 0 && g32(pl + 24) == want[fw_stats_req], "REQUEST_STATS is not {PDEV_STAT, vdev 0, MAC 0, target pdev}");
	fw_stats_pdev = want[fw_stats_req];
	fw_stats_req++; fw_stats_at = now;
	if (C.guard_on_stats) { guard_bad = true; }  /* e.g. the firmware asserts on the request */
	fw_stats_answer(need);
}

static void
fw_rx(unsigned ce, uint32_t eid, const uint8_t *d, uint32_t len)
{
	CHECK(len >= 8, "HTC frame shorter than its header");
	uint32_t w0 = g32(d), w1 = g32(d + 4), heid = w0 & 0xff, flags = (w0 >> 8) & 0xff, plen = w0 >> 16;
	const uint8_t *pl = d + 8;
	CHECK(heid == eid && heid < 9 && len == 8 + plen, "HTC header endpoint/length vs CE meta/length");
	CHECK(((w1 >> 8) & 0xff) == fw_seq[eid]++ % 256 && (w1 & 0xff) == 0, "HTC sequence or control byte 0");
	if (eid == 0) {
		CHECK(ce == 0 && flags == 0, "HTC control message off CE0 or with flags");
		uint32_t id = g32(pl) & 0xffff;
		if (id == 2) {
			CHECK(plen == 8, "connect request length");
			uint32_t svc = g32(pl) >> 16, f = g32(pl + 4);
			uint8_t r[12];
			if (C.guard_fail_on_connect) { guard_bad = true; }  /* e.g. AMSS left EE 2 */
			if (svc == 0x300) {
				CHECK(f == 0x8, "HTT connect flags are not DISABLE_CREDIT_FLOW_CTRL only");
				p32(r, 3 | svc << 16); p32(r + 4, C.htt_status | C.htt_eid << 8 | C.max_msg << 16); p32(r + 8, 0);
				fw_eid_used |= 1u << C.htt_eid;
			} else {
				CHECK(svc == 0x100, "connect for an unexpected service");
				CHECK(f == ((C.credits & 0xffu) << 8), "WMI connect flags are not RECV_ALLOC = all credits");
				fw_credits = (int)(C.credits & 0xff);
				p32(r, 3 | svc << 16); p32(r + 4, C.wmi_eid << 8 | C.max_msg << 16); p32(r + 8, 0);
			}
			htc_out(2, 0, r, 12, -1, 0, 2);
		} else {
			CHECK(id == 5 && plen == 12 && !g32(pl + 4) && !g32(pl + 8), "unexpected HTC control message");
			fw_setup_done = true;
			fw_service_ready_set();
		}
		return;
	}
	if (eid != C.wmi_eid && eid == C.htt_eid) {  /* C2 w20 0x02: HTT VERSION_REQ (dp_tx.c:1277-1305) on CE4 */
		CHECK(ce == 4 && flags == 0 && plen == 8 && g32(pl) == 0 && g32(pl + 4) == (5u | 4u << 8 | 2u << 16),
		    "HTT VERSION_REQ is not {msg 0, TCL metadata option tag 5 len 4 value 2} on CE4");
		CHECK(fw_init_ok == 1 && fw_c2_step == 0, "HTT VERSION_REQ out of order");
		fw_htt_req++;
		if (!C.htt_silent) {
			uint8_t r[4];
			p32(r, 0u | C.htt_minor << 8 | C.htt_major << 16);
			htc_out(1, C.htt_eid, r, 4, -1, 0, 2);
		}
		return;
	}
	CHECK(eid == C.wmi_eid && ce == 3 && flags == 1, "WMI message off its endpoint/pipe or without NEED_CREDIT_UPDATE");
	int need = (int)((len + C.credit_size - 1) / C.credit_size);
	CHECK(fw_credits >= need, "the host sent WMI without credits");
	fw_credits -= need;
	fw_wmi_msgs++;
	uint32_t id = g32(pl) & 0xffffff;
	if (id == 0x1) {
		static const uint8_t mac[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
		/* L19 completes service ready only from EXT, or EXT2 when service 220 is set (wmi.c:4940-4941, 5645) */
		CHECK(q_head > fw_ext_q && (C.no_ext2_service || q_head > fw_ext2_q), "INIT before SERVICE_READY_EXT/EXT2 reached the host");
		fw_check_init(pl, plen);
		m_reset();  /* READY: TLV 0x23, 60 bytes: fw abi words non-zero, MAC at 24 zero unless ready_mac */
		m_tlv(0x23, 60);
		for (unsigned i = 0; i < 6; i++) { m_u32(0x11111111u * (i + 1)); }
		if (C.ready_mac) { memcpy(mb + ml, mac, 6); }
		m_zero(8); m_u32(C.ready_status); m_u32(0); m_u32(0); m_u32(16); m_zero(12);
		wmi_out(0x2, C.refund_in_ready ? (unsigned)need : 0u, 3);
		m_reset(); m_tlv(0x3aa, 40); m_zero(40);
		wmi_out(0x3a003, 0, 4);
		if (C.big_event) {  /* one transfer of exactly 2048 bytes (the DEST_CTRL max): an unknown event, logged only */
			m_reset(); m_zero(2048u - 8u - 4u);
			wmi_out(0x7ff2, 0, 5);
		}
		for (unsigned i = 0; i < C.extra_unknown; i++) {  /* unknown events that use up the 16-line budget */
			m_reset(); m_tlv(0x400, 4); m_zero(4);
			wmi_out(0x7ff3, 0, 5);
		}
		if (C.refund_later) {  /* trailer-only HTC message 50 ms later: credits, no payload */
			uint8_t d2[16] = {0, 0, 0, 0, 8, 0, 0, 0, 1, 4, 0, 0, (uint8_t)C.wmi_eid, (uint8_t)need, 0, 0};
			p32(d2, C.wmi_eid | 2u << 8 | 8u << 16);
			fw_credits += need;
			q_push(2, 50, d2, 16, -1);
		}
	} else if (id == 0x401e) {
		CHECK(plen == 20 && g32(pl + 4) == (0x203u << 16 | 12u) && g32(pl + 8) == 0 && g32(pl + 12) == 1 && g32(pl + 16) == 0,
		    "PDEV_SET_HW_MODE is not {SOC, DBS, 0 band maps}");
		fw_hw_mode_ok++;
		m_reset(); m_tlv(0x206, 8); m_zero(8);
		wmi_out(0x4010, (unsigned)need, 3);
	} else if (id == 0x3001 || id == 0x3002 || id == 0x3003 || id == 0x5001 || id == 0x5008 || id == 0x9001 || id == 0x9002) {
		CHECK(wl_w23, "a C2 command with qrtr 6");
		fw_c2_cmd(id, pl, plen, (unsigned)need);
	} else if (id == 0x16001) {
		fw_stats_cmd(pl, plen, (unsigned)need);
	} else {
		CHECK(id == 0x7ff0, "unexpected WMI command");
		fw_echoes++;
		m_reset(); m_tlv(0x11, 4); m_u32(fw_echoes);
		wmi_out(0x7ff1, (unsigned)need, 1);
	}
}

/* ---- stubs of the fuxi_wlan.c functions the excerpts call ---- */
static uint32_t
wl_mrd(uint32_t off)
{
	if (off == 0x310c) {
		if (restore_lie) { restore_lie = false; return bar_win ^ 1u; }  /* the readback that follows a restore */
		if (C.win_mismatch_at && win_selects == C.win_mismatch_at && (bar_win & 0x3f) == 0x37) { return bar_win ^ 1u; }
		return bar_win;
	}
	CHECK(off == 0x38, "BAR0 read outside the window register and MHICTRL");
	return mhictrl;
}

static void
wl_mwr(uint32_t off, uint32_t v)
{
	if (off == 0x310c) {
		CHECK((v & 0x40000000u) && (v & ~0x3fu) == (WIN_INIT & ~0x3fu), "window select lost the saved upper bits");
		bar_win = v; win_selects++;
		return;
	}
	CHECK(off >= SHADOW0 && off < SHADOW0 + 160 && !(off & 3), "BAR0 write outside window and shadow registers");
	CHECK(bar_win == WIN_INIT, "shadow write while the window is moved");
	unsigned i = (off - SHADOW0) / 4;
	CHECK(i < 36, "shadow register beyond the WLAN_CFG table");
	shadow_writes++;
	ce_access(wl15_shadow_v3[i], true, &v, true);
}

static uint32_t
wl_prd(uint64_t pa)
{
	if (pa >= WL_BAR0 + 0x80000ull && pa < WL_BAR0 + 0x100000ull) {
		uint32_t v = 0;
		CHECK(bar_win & 0x40000000u, "window access with the window disabled");
		ce_access((bar_win & 0x3fu) << 19 | (uint32_t)(pa - WL_BAR0 - 0x80000ull), false, &v, false);
		return v;
	}
	if (pa == WL_EPCFG + 4) { return epcmd; }
	if (pa == WL_EPCFG + 0x70 + 8) { return devctl; }
	CHECK(pa == TS_TLMM + 0x1000ull * 94 + 4, "physical read outside the model");
	return perst_io;
}

static void
wl_pwr(uint64_t pa, uint32_t v)
{
	if (pa >= WL_BAR0 + 0x80000ull && pa < WL_BAR0 + 0x100000ull) {
		CHECK(bar_win & 0x40000000u, "window access with the window disabled");
		ce_access((bar_win & 0x3fu) << 19 | (uint32_t)(pa - WL_BAR0 - 0x80000ull), true, &v, false);
		return;
	}
	CHECK(pa == WL_EPCFG + 0x70 + 8 && (C.mrrs_old || !v), "physical write outside the model");
	CHECK(v <= 0xffffu, "Device Control write touched the W1C status half");
	devctl = v;
}

static int
wl_contain_wr(uint64_t pa, uint32_t value)
{
	if (pa == WL_BAR0 + 0x310c) { bar_win = value; restore_lie = C.restore_lie_at && ++restores == C.restore_lie_at; return 0; }
	if (pa == WL_BAR0 + 0x38) { mhictrl = 0; return 0; }
	if (pa == WL_EPCFG + 4) { if (!C.fail_bm) { epcmd = value; } return 0; }
	CHECK(pa == TS_TLMM + 0x1000ull * 94 + 4, "containment write outside the model");
	perst_io = value;
	return 0;
}

static int
wl_dma_alloc(struct wl_dma *d, vm_size_t bytes)
{
	CHECK(d == &wl_ce_arena && bytes == ARENA && !d->va, "unexpected DMA allocation");
	CHECK(!(wl_iova_next & (ARENA - 1)) && wl_iova_next >= wl_iova_base && wl_iova_next + ARENA <= wl_iova_end,
	    "arena IOVA not aligned to its size inside the pool");
	void *p = aligned_alloc(16384, ARENA);
	CHECK(p != NULL, "host allocation");
	memset(p, 0xa5, ARENA);  /* stricter than KMA_ZERO: the code must zero what it relies on */
	d->va = (vm_offset_t)p; d->size = ARENA; d->iova = wl_iova_next; d->pa = 0x880000000ull;
	wl_iova_next += ARENA;
	arena_host = p; arena_iova = d->iova; arena_allocs++;
	return 0;
}
static int wl_dma_nc(struct wl_dma *d) { CHECK(d == &wl_ce_arena, "NC conversion of another buffer"); d->nc = true; return 0; }
static void
wl_dma_free(struct wl_dma *d)
{
	if (!d->va) { return; }
	CHECK(d == &wl_ce_arena, "free of a buffer the model never allocated");
	free((void *)d->va);
	arena_host = NULL; arena_mapped = false; arena_frees++;
	memset(d, 0, sizeof(*d));
}
static int wl12_map_target(struct wl_dma *d) { CHECK(d == &wl_ce_arena && d->nc, "map of an unconverted buffer"); arena_mapped = true; return 0; }
static int wl_w11_guard(void) { return fw_faults || guard_bad ? wl_w11_fail("AMSS/M0, ownership, discovery or access/SMMU fault") : 0; }
static int wl_w11_events(void) { return wl_w11_guard(); }
static void wl_w11_stop(void) { CHECK(!wl_ce_arena.va || wl_ce.stopping, "W11 stop before the CE containment latch"); stops++; }
static void wl_sleep(unsigned ms) { now += ms; sleeps += ms; dev_tick(); }
static unsigned wl_m6o_cap(unsigned id) { CHECK(id == 0x10, "capability other than PCIe"); return 0x70; }
static void wl_wlfw_diagnostics(const char *label) { printf("FUXI WLAN: %s diagnostic (model)\n", label); }
static void wl5_status(unsigned step) { (void)step; }
static int wl_smmu_fault(unsigned step) { (void)step; return 0; }
static int wl_m6o_msi_undo(void) { return 0; }
static int wl_m6o_undo(bool contained) { (void)contained; return 0; }
static int wl_smmu_undo(void) { return 0; }

/* An io for pure-layer checks that must never reach hardware. */
static int io_never_wr(void *ctx, uint32_t off, uint32_t v) { (void)ctx; (void)off; (void)v; fail("pure code reached hardware"); return 1; }
static int io_never_rd(void *ctx, uint32_t off, uint32_t *v) { (void)ctx; (void)off; *v = ~0u; fail("pure code reached hardware"); return 1; }
static void io_never_barrier(void *ctx) { (void)ctx; fail("pure code reached a barrier"); }
/* C2 pure CE4 test: shadow publishes accepted and counted nowhere, barriers in the model's order check */
static int io_sink_bar(void *ctx, uint32_t off, uint32_t v) { (void)ctx; (void)off; (void)v; return 0; }
static void io_model_w(void *ctx) { (void)ctx; model_barrier('W'); }
static void io_model_r(void *ctx) { (void)ctx; model_barrier('R'); }

/* ---- scenarios ---- */
static void
reset(void)
{
	if (wl_ce_arena.va) { free((void *)wl_ce_arena.va); memset(&wl_ce_arena, 0, sizeof(wl_ce_arena)); }
	memset(&C, 0, sizeof(C));
	C.credits = 4; C.credit_size = 2048; C.ready_ce = 2; C.htt_eid = 1; C.wmi_eid = 2; C.max_msg = 2048; C.ready_stslen = -1;
	C.phy_bands = 3; C.phy_mask = 3; C.htt_major = 3; C.htt_minor = 4; C.exp_n = 38; C.exp_pdev = 0;
	C.dbs_pdev = 1;
	loglen = 0; logbuf[0] = 0;
	memcpy(dr, ring_plan, sizeof(dr));
	for (unsigned i = 0; i < 7; i++) {
		uint32_t block = dr[i].kind == K_SRC ? 0x01b80000u + 0x2000u * dr[i].ce : 0x01b81000u + 0x2000u * dr[i].ce;
		dr[i].r0 = block + (dr[i].kind == K_STS ? 0x58u : 0u);
		dr[i].r2 = block + (dr[i].kind == K_STS ? 0x408u : 0x400u);
	}
	memset(ce_reg, 0, sizeof(ce_reg));
	arena_host = NULL; arena_mapped = false; arena_allocs = arena_frees = 0;
	bar_win = WIN_INIT; epcmd = 0x406; perst_io = 2; devctl = 0; mhictrl = 0;
	win_selects = ce_writes = ce_reads = shadow_writes = direct_publishes = shadow_publishes = sleeps = stops = 0;
	now = 0; last_ev = 0; memset(n_ev, 0, sizeof(n_ev));
	q_head = q_tail = 0; fw_ready_sent = fw_setup_done = false; fw_credits = 0;
	memset(fw_seq, 0, sizeof(fw_seq)); fw_eid_used = fw_wmi_msgs = fw_init_ok = fw_hw_mode_ok = fw_echoes = fw_tx_msgs = 0;
	fw_ext_q = fw_ext2_q = ~0u; restores = 0; restore_lie = sts_glitched = false;
	fw_svc281 = guard_bad = false; fw_chains = 2;
	fw_faults = 0; wl15_variant = 0; wl20_bits = 0; wl_w11_error = 0; wl_seq_flags = 0;
	wl_w15 = wl_w12 = wl_w20 = true; wl_w11_ok = false; wl_w11_attempted = true; wl_mhi_valid = true;
	wl12_stage = WL15_OBSERVE; wl_iova_next = 0xa1234000ull;
	memset(&wl_ce, 0, sizeof(wl_ce)); wl_ce_started = false;
	wl_w23 = false; memset(&wl_scan, 0, sizeof(wl_scan)); wl23_scan_sent = wl23_htt_seen = false;
	wl23_lines = wl23_pdev = wl23_htt_major = wl23_htt_minor = wl23_rssi_conv = 0; wl23_guard_bad = false;
	fw_c2_step = fw_chan_seen = fw_chan_cmds = fw_scan_cmds = fw_stop_cmds = fw_htt_req = 0; fw_stop_pdev = ~0u;
	n_beacons = 0;
	memset(&wl_stats, 0, sizeof(wl_stats)); wl23_ci_lines = wl23_ci_drop = 0; fw_stats_req = 0; fw_stats_at = 0; fw_stats_pdev = 1;
}

static void
expect(const char *s)
{
	if (!strstr(logbuf, s)) { fputs(logbuf, stderr); fprintf(stderr, "MODEL: missing log text: %s\n", s); abort(); }
}

static void
expect_not(const char *s)
{
	if (strstr(logbuf, s)) { fputs(logbuf, stderr); fprintf(stderr, "MODEL: unexpected log text: %s\n", s); abort(); }
}

static void
expect_order(const char *const *lines, unsigned n)
{
	const char *at = logbuf;
	for (unsigned i = 0; i < n; i++) {
		const char *hit = strstr(at, lines[i]);
		if (!hit) { fputs(logbuf, stderr); fprintf(stderr, "MODEL: missing or out of order: %s\n", lines[i]); abort(); }
		at = hit + strlen(lines[i]);
	}
}

/* wl_w14_run calls wl_w20_run after the MODE reply; fw_run then ends in
 * wl5_abort either way (keep is refused with qrtr 6). */
static int
run(void)
{
	int e = wl_w20_run();
	CHECK(e || wl_w11_error == 0, "success with a latched W11 error");
	return e;
}
static void contain(void) { wl5_abort(28); }

static void
contained_and_freed(void)
{
	contain();
	expect("FUXI WLAN: W20 ce contain posted");
	expect("FUXI WLAN: abort step 28 MHI reset done, bus master off yes, PERST asserted yes");
	expect("FUXI WLAN: abort buffers freed");
	CHECK(arena_frees == 1 && !wl_ce_arena.va && stops == 1, "arena not freed after proven containment");
	CHECK(bar_win == WIN_INIT, "window not restored at the end");
}

static void
happy_lines(bool c2)
{
	const char *const lines[] = {
		"FUXI WLAN: W20 ce arena IOVA 0xa1280000 bytes 0x80000 Normal-NC mapped; rdp 0xa1280000",
		"FUXI WLAN: W20 ce ring 64 entries 16 base 0xa1281000 misc 0x42 readback ok",
		"FUXI WLAN: W20 ce ring 82 entries 512 base 0xa1282000 misc 0x42 dest-ctrl 0x800 readback ok",
		"FUXI WLAN: W20 ce ring 101 entries 512 base 0xa1283000 misc 0x40 readback ok",
		"FUXI WLAN: W20 ce ring 83 entries 64 base 0xa1285000 misc 0x42 dest-ctrl 0x800 readback ok",
		"FUXI WLAN: W20 ce ring 102 entries 64 base 0xa1286000 misc 0x40 readback ok",
		"FUXI WLAN: W20 ce ring 67 entries 32 base 0xa1287000 misc 0x42 readback ok",
		"FUXI WLAN: W20 ce ring 68 entries 2048 base 0xa1288000 misc 0x42 readback ok",
		"FUXI WLAN: W20 ce init ok rings 7 writes 70 window restore checked",  /* 5 x 10 + 2 x 9 + 2 DEST_CTRL */
		"FUXI WLAN: W20 ce rx posted CE1 62 CE2 62",
		"FUXI WLAN: W20 htc ready credits 4 size 2048 max-ep 9 after ",
		"FUXI WLAN: W21 htc svc 0x300 status 0 eid 1 maxmsg 2048 pipes 4/1",
		"FUXI WLAN: W21 htc svc 0x100 status 0 eid 2 maxmsg 2048 pipes 3/2 credits 4",
		"FUXI WLAN: W21 htc setup complete sent",
		"FUXI WLAN: W21 wmi service ready fw 0x12345678 rf-chains 2 mem-reqs 0 scan-chans 64",
		"FUXI WLAN: W21 wmi services ext 1 ext2 1 reg-cc-ext 1 (available event 1)",
		"FUXI WLAN: W21 wmi service ready ext hw-modes 2 caps 2 pdev 1",
		"FUXI WLAN: W21 wmi service ready ext2 bytes 64",
		"FUXI WLAN: W21 wmi service ready complete",
		"FUXI WLAN: W22 wmi init sent bytes 332 (vdevs 4 peers 16 chains 0x3 flag1 0x40020 flags2 0x200 host-svc 0x10 mem-chunks 0)",
		"FUXI WLAN: W22 wmi ready status 0 mac-zero 1 peers 16",
		"FUXI WLAN: W22 hw mode dbs sent",
		"FUXI WLAN: W22 wmi event 0x3a003 bytes 44 (logged only)",
		"FUXI WLAN: W22 wmi event 0x4010 bytes 12 (logged only)",
		"FUXI WLAN: W22 hw mode response seen; wmi events 7 unknown 1",
		c2 ? "FUXI WLAN: W22 complete; W23 scan next" : "FUXI WLAN: W22 complete; stop/contain",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	CHECK(fw_init_ok == 1 && fw_hw_mode_ok == 1, "INIT / HW mode not seen by the firmware");
}

static void
s_happy_shadow(void)
{
	reset();
	CHECK(run() == 0, "happy path (shadow) failed");
	happy_lines(false);
	expect("FUXI WLAN: W20 ce shadow 29 30 31 32 33 34 35 (from WLAN_CFG table)");
	/* services 256..: only the 4 words of the AVAILABLE array, nothing of the RX buffer behind it */
	CHECK(!wl_wmi.svc_ext2[4] && !wl_wmi.svc_ext2[5] && !wl_wmi.svc_ext2[6] && !wl_wmi.svc_ext2[7],
	    "SERVICE_AVAILABLE words read past its 4-word array");
	CHECK(direct_publishes == 0 && shadow_publishes > 20 && shadow_writes == shadow_publishes, "shadow route");
	CHECK(n_ev['W'] == shadow_publishes && n_ev['L'] == n_ev['R'] && n_ev['L'] > 100, "barrier counts");
	expect_not("W20 failure");
	contained_and_freed();
	/* 10 messages on CE2 (READY, 2 connect replies, 4 service-ready events, READY, CC_EXT, HW mode
	 * response), each buffer posted again; 3 HTC control + 2 WMI messages sent */
	expect("FUXI WLAN: W20 ce contain posted 134 reaped 10 tx 5");
	puts("W20 happy path (shadow): 7 rings, 62+62 RX, HTC READY, HTT/WMI connect, SERVICE_READY/AVAILABLE/EXT/EXT2, "
	     "INIT (field by field), READY, HW mode DBS, contained and freed");
}

static void
s_happy_direct(void)
{
	reset();
	wl15_variant = 1;
	CHECK(run() == 0, "happy path (direct R2) failed");
	happy_lines(false);
	expect("FUXI WLAN: W20 ce shadow off (fuxiwlanw15=1): pointers go to R2 through the window");
	CHECK(shadow_writes == 0 && direct_publishes > 20 && n_ev['W'] == direct_publishes, "direct route");
	contained_and_freed();
	puts("W20 fuxiwlanw15=1: every pointer published to R2 through the window, no shadow write");
}

static void
s_bits(void)
{
	reset();
	wl20_bits = WL20_NOHTT | WL20_NOHWMODE | WL20_MRRS128;
	C.wmi_eid = 1; C.mrrs_old = 0x2810; devctl = 0x2810;
	CHECK(run() == 0, "w20 0x45 failed");
	expect("FUXI WLAN: W20 endpoint devctl cap 0x70 MRRS 128: old 0x2810 read 0x810");
	expect("FUXI WLAN: W21 htc HTT connect skipped (w20 0x01)");
	expect("FUXI WLAN: W21 htc svc 0x100 status 0 eid 1 maxmsg 2048 pipes 3/2 credits 4");
	expect("FUXI WLAN: W22 hw mode skipped (w20 0x40)");
	expect_not("htc svc 0x300");
	CHECK(fw_hw_mode_ok == 0 && fw_init_ok == 1 && devctl == 0x810, "w20 bits");
	contained_and_freed();
	puts("W20 w20 0x45: MRRS 128 on the endpoint (low half only), no HTT connect (WMI on eid 1), no HW mode");
}

static void
s_credits(void)
{
	reset();
	C.credits = 1; C.refund_later = true;
	CHECK(run() == 0, "credit refill failed");
	expect("FUXI WLAN: W21 htc svc 0x100 status 0 eid 2 maxmsg 2048 pipes 3/2 credits 1");
	expect("FUXI WLAN: W22 WMI PDEV_SET_HW_MODE waits for 1 HTC credits (has 0)");
	expect("FUXI WLAN: W22 hw mode dbs sent");
	CHECK(wl_htc.ep[2].credits == 1, "refund of the HW mode response");
	contained_and_freed();
	reset();
	C.credits = 1;
	CHECK(run() == 1, "credit starvation passed");
	expect("FUXI WLAN: W22 timeout HTC credits (3000 ms); contain, no retry");
	expect("FUXI WLAN: W20 snapshot HTC credits:");
	expect("FUXI WLAN: W22 failure: HTC credits; contain, no retry");
	CHECK(fw_hw_mode_ok == 0, "HW mode sent without credits");
	contained_and_freed();
	reset();
	C.credits = 0x104;  /* RECV_ALLOC and the endpoint's credits are the u8 of the count, as L19 */
	CHECK(run() == 0, "credit count above 255");
	expect("pipes 3/2 credits 4");
	contained_and_freed();
	reset();
	C.credit_size = 336;  /* the 340-byte INIT frame costs 2 credits with its header, 1 without (htc.c:90-93) */
	CHECK(run() == 0, "credit size 336");
	expect("FUXI WLAN: W20 htc ready credits 4 size 336 max-ep 9");
	CHECK(wl_htc.ep[2].credits == 4 - 2 - 1 + 1, "INIT not charged ceil((8 + 332) / 336) = 2 credits");
	contained_and_freed();
	puts("W20 credits: exhaustion waits for a trailer-only refund, starvation times out at 3000 ms and contains, u8 allocation, "
	     "cost counts the HTC header");
}

static void
s_ready_missing(void)
{
	reset();
	C.silent_ready = true;
	CHECK(run() == 1, "missing HTC READY passed");
	expect("FUXI WLAN: W20 timeout HTC READY (3000 ms); contain, no retry");
	expect("FUXI WLAN: W20 snapshot ring 102 hp 0 tp 0 rdp 0 seen 0");
	expect("FUXI WLAN: W20 snapshot tx CE0 0/0 CE3 0/0 rx CE1 62/0 CE2 62/0 (posted/reaped)");
	expect("FUXI WLAN: W20 failure: HTC READY; contain, no retry");
	contained_and_freed();
	reset();
	C.silent_ready = true; C.fail_bm = true;
	CHECK(run() == 1, "missing HTC READY passed");
	contain();
	expect("FUXI WLAN: abort step 28 MHI reset done, bus master off unconfirmed, PERST asserted yes");
	expect("FUXI WLAN: abort buffers quarantined until reboot");
	CHECK(arena_frees == 0 && wl_ce_arena.va, "arena freed although bus master off is unproven");
	puts("W20 no HTC READY: 3000 ms timeout, snapshot, contained; arena freed only when containment is proven");
}

static void
fails_with(const char *why, const char *line)
{
	CHECK(run() == 1, why);
	expect(line);
	contained_and_freed();
}

static void
s_faults(void)
{
	reset(); C.ready_stslen = 0;
	fails_with("STATUS length 0", "FUXI WLAN: W20 failure: CE STATUS length 0 or above the RX buffer; contain, no retry");
	reset(); C.ready_stslen = 2049;
	fails_with("STATUS length 2049", "FUXI WLAN: W20 failure: CE STATUS length 0 or above the RX buffer; contain, no retry");
	reset(); C.ready_eid = 9;
	fails_with("bad endpoint", "FUXI WLAN: W20 failure: HTC endpoint id out of range; contain, no retry");
	reset(); C.ready_eid = 5;
	fails_with("unconnected endpoint", "FUXI WLAN: W20 failure: HTC message on an unconnected endpoint; contain, no retry");
	reset(); C.ready_trailer_long = true;
	fails_with("long trailer", "FUXI WLAN: W20 failure: HTC trailer length outside 4..payload; contain, no retry");
	reset(); C.ready_trailer_zero = true;
	fails_with("trailer length 0", "FUXI WLAN: W20 failure: HTC trailer length outside 4..payload; contain, no retry");
	reset(); C.ready_stslen = 12;  /* STATUS says 12 bytes, the HTC READY frame is 16 */
	fails_with("HTC payload past the transfer", "FUXI WLAN: W20 failure: HTC payload longer than the CE transfer; contain, no retry");
	reset(); C.svc_tlv_past = true;
	fails_with("TLV past buffer", "FUXI WLAN: W21 failure: WMI SERVICE_READY TLV past the buffer; contain, no retry");
	reset(); C.ext_nested_overrun = true;
	fails_with("nested overrun", "FUXI WLAN: W21 failure: WMI EXT nested TLV past its array; contain, no retry");
	reset(); C.rdp_glitch = 1;
	fails_with("rdp outside", "FUXI WLAN: W20 failure: CE device ring pointer outside the ring; contain, no retry");
	reset(); C.rdp_glitch = 2;
	fails_with("rdp beyond posted", "failure: CE STATUS entries beyond the posted buffers; contain, no retry");
	reset(); C.rdp_glitch = 3;
	fails_with("TX TP backwards", "FUXI WLAN: W21 failure: CE device TP passed the host HP or went backwards; contain, no retry");
	reset(); C.sts_rdp_delta = 2;
	fails_with("rdp misaligned", "FUXI WLAN: W21 failure: CE device ring pointer outside the ring; contain, no retry");
	/* STATUS HP back 1 entry on CE2 = 63 new entries for 62 buffers out; back 2 = 62, inside that bound,
	 * stopped by the zero length of the first, never written, entry */
	reset(); C.sts_rdp_delta = -4;
	fails_with("STATUS back 1", "FUXI WLAN: W21 failure: CE STATUS entries beyond the posted buffers; contain, no retry");
	reset(); C.sts_rdp_delta = -8;
	fails_with("STATUS back 2", "FUXI WLAN: W21 failure: CE STATUS length 0 or above the RX buffer; contain, no retry");
	reset(); C.mem_reqs = 2;
	fails_with("mem reqs", "FUXI WLAN: W21 failure: W21 SERVICE_READY asks for host memory chunks (plan rule, not L19); contain, no retry");
	reset(); C.htt_status = 1;
	fails_with("connect refused", "FUXI WLAN: W21 failure: HTC service connect refused by the target; contain, no retry");
	reset(); C.htt_eid = 0;
	fails_with("connect to eid 0", "FUXI WLAN: W21 failure: HTC endpoint already in use; contain, no retry");
	reset(); C.guard_fail_on_connect = true;
	fails_with("guard", "FUXI WLAN: W21 failure: AMSS/M0, ownership, discovery or access/SMMU fault; contain, no retry");
	puts("W20 faults: STATUS length 0/2049, endpoint 9, unconnected endpoint, long trailer, trailer length 0, HTC payload past "
	     "the transfer, TLV past the buffer, nested overrun, rdp outside/beyond posted/misaligned, STATUS back 1/2, TX TP "
	     "backwards, mem-reqs, connect refused/eid 0, guard: all contained");
}

static void
s_tolerated(void)
{
	reset(); C.bad_report = true;
	CHECK(run() == 0, "unknown-endpoint credit report must be tolerated");
	CHECK(wl_htc.bad_reports == 1 && wl_htc.ep[2].credits == 4 + 1 - 1 - 1 + 1, "credit report for endpoint 12");
	contained_and_freed();
	reset(); C.no_ext2_service = true;
	CHECK(run() == 0, "no EXT2 service");
	expect("FUXI WLAN: W21 wmi services ext 1 ext2 0 reg-cc-ext 1");
	expect_not("W21 wmi service ready ext2 bytes");
	contained_and_freed();
	reset(); C.ready_ce = 1;
	CHECK(run() == 0, "HTC READY on CE1");
	contained_and_freed();
	reset(); C.empty_msgs = true;
	CHECK(run() == 0, "empty HTC messages on EP0 and the WMI endpoint");
	CHECK(wl_htc.htt_rx == 0 && wl_wmi.events == 7, "an empty HTC message reached a handler");
	contained_and_freed();
	reset(); C.ready_status = 3;  /* L19 stores it and goes on (wmi.c:6910) */
	CHECK(run() == 0, "READY status 3");
	expect("FUXI WLAN: W22 wmi ready status 3 mac-zero 1 peers 16");
	expect("FUXI WLAN: W22 complete; stop/contain");
	contained_and_freed();
	reset(); C.ready_mac = true;
	CHECK(run() == 0, "READY with a MAC");
	expect("FUXI WLAN: W22 wmi ready status 0 mac-zero 0 peers 16");
	CHECK(!memcmp(wl_wmi.mac, (const uint8_t[6]){0x02, 0x11, 0x22, 0x33, 0x44, 0x55}, 6), "READY MAC not taken from bytes 24..29");
	contained_and_freed();
	reset(); C.big_event = true;
	CHECK(run() == 0, "a 2048-byte transfer");
	expect("FUXI WLAN: W22 wmi event 0x7ff2 bytes 2036 (logged only)");
	contained_and_freed();
	puts("W20 tolerated: credit report for endpoint 12 (counted, ends the record as L19), no EXT2 service, HTC READY on CE1, "
	     "empty HTC messages (dropped as L19), READY status 3 (logged as L19), READY MAC at 24, a 2048-byte transfer");
}

/* L19 sends INIT only after EXT (or EXT2 with service 220) completed service
 * ready; the firmware checks it, with EXT later than the AVAILABLE look. */
static void
s_order(void)
{
	reset(); C.ext_delay = 1500;
	CHECK(run() == 0, "EXT after 1500 ms");
	expect("FUXI WLAN: W21 wmi service ready ext2 bytes 64");
	contained_and_freed();
	reset(); C.ext_delay = 1500; C.no_ext2_service = true;
	CHECK(run() == 0, "EXT after 1500 ms, no EXT2 service");
	expect("FUXI WLAN: W21 wmi service ready ext hw-modes 2 caps 2 pdev 1");
	contained_and_freed();
	puts("W20 order: INIT only after SERVICE_READY_EXT (and EXT2) reached the host, EXT 1500 ms late with and without EXT2");
}

static void
s_window_and_fault(void)
{
	reset(); C.win_mismatch_at = 1;
	CHECK(run() == 1, "window readback mismatch passed");
	CHECK(ce_writes == 0 && bar_win == WIN_INIT, "a CE write despite the window mismatch");
	expect("FUXI WLAN: W20 failure: CE register write refused or failed; contain, no retry");
	contained_and_freed();
	reset(); C.fault_after = 5;
	CHECK(run() == 1, "fault mid-programming passed");
	CHECK(ce_writes == 5 && bar_win == WIN_INIT, "writes after the latched fault");
	contained_and_freed();
	reset(); C.restore_lie_at = 1;  /* 0x310c reads back wrong once, right after the first restore */
	CHECK(run() == 1, "window restore readback mismatch passed");
	CHECK(ce_writes == 1 && bar_win == WIN_INIT, "a CE write after the restore readback mismatch");
	expect("FUXI WLAN: W20 failure: CE register write refused or failed; contain, no retry");
	contained_and_freed();
	reset(); C.dest_ctrl_ignore = true;
	CHECK(run() == 1, "DEST_CTRL that did not take passed");
	expect("FUXI WLAN: W20 ce ring 82 setup after 21 writes (base 0 misc 0 dest-ctrl 0)");
	expect("FUXI WLAN: W20 failure: CE DEST_CTRL readback; contain, no retry");
	CHECK(wl_ce.pipe[1].posted == 0 && wl_ce.pipe[2].posted == 0, "RX buffers posted with an unconfirmed DEST_CTRL");
	contained_and_freed();
	puts("W20 window readback mismatch: no CE write; fault latched after write 5: no later write; window restore read back "
	     "wrong: no later write; DEST_CTRL that does not take: no RX buffer posted; all contained");
}

/* After wl_ce_contain nothing is written, published, posted, sent or consumed. */
static void
s_latch(void)
{
	reset();
	CHECK(run() == 0, "happy path before the latch");
	wl_ce_contain(28);
	expect("FUXI WLAN: W20 ce contain posted 134 reaped 10 tx 5");
	unsigned w = ce_writes, s = shadow_writes, sel = win_selects, rx = wl_ce.pipe[2].reaped, tx = wl_ce.pipe[3].posted;
	uint8_t cmd[8] = {0};
	CHECK(wlce_wr(&wl_ce, &wl_ce_io, dr[1].r0 + 0xb0, 0x800) == 1, "register write after the latch");
	CHECK(wlce_publish(&wl_ce, &wl_ce_io, &wl_ce.pipe[3].src) == 1, "pointer publish after the latch");
	CHECK(wlce_send(&wl_ce, &wl_ce_io, &wl_ce.pipe[3], wl_htc.wmi_eid, cmd, sizeof(cmd)) == 1, "send after the latch");
	CHECK(wlce_post_rx(&wl_ce, &wl_ce_io, &wl_ce.pipe[2], 1) == 1, "RX post after the latch");
	CHECK(wlce_reap_rx(&wl_ce, &wl_ce_io, &wl_ce.pipe[2], wl_ce_rx, NULL) == 0 &&
	    wlce_reap_tx(&wl_ce, &wl_ce_io, &wl_ce.pipe[3]) == 0, "reap after the latch is not a silent no-op");
	CHECK(ce_writes == w && shadow_writes == s && win_selects == sel && wl_ce.pipe[2].reaped == rx &&
	    wl_ce.pipe[3].posted == tx, "hardware touched after the latch");
	contained_and_freed();
	puts("W20 latch: after wl_ce_contain no register write, publish, send, RX post or reap");
}

static void
s_wrap(void)
{
	reset();
	CHECK(run() == 0, "happy path before wrap");
	uint8_t cmd[8];
	p32(cmd, 0x7ff0); p32(cmd + 4, 0);
	for (unsigned i = 0; i < 140; i++) {
		CHECK(wl_ce_send(wl_htc.wmi_eid, cmd, sizeof(cmd), "echo") == 0, "echo send");
		CHECK(wl_ce_observe(WL20_W_NONE, 20) == 0, "echo poll");
	}
	CHECK(fw_echoes == 140 && wl_ce.pipe[3].posted == 142 && wl_ce.pipe[2].reaped >= 140 + 8, "wrap counts");
	CHECK(wl_ce.pipe[3].posted > 2 * 32 && wl_ce.pipe[2].reaped > 2 * 64, "CE3/CE2 did not wrap twice");
	contained_and_freed();
	puts("W20 ring wrap: 140 WMI commands through CE3 (32 entries) and 148 events through CE2 (64 entries), credits returned");
}

static void
s_pure(void)
{
	/* the header's register set equals the model's, address by address */
	reset();
	for (uint32_t off = CE_BASE - 0x1000; off < CE_END + 0x1000; off += 4) {
		CHECK(wlce_reg_ok(off) == model_reg_ok(off), "wlce_reg_ok differs from the model's register set");
	}
	CHECK(!wlce_reg_ok(CE_BASE + 2) && !wlce_reg_ok(0x01b8a400), "misaligned / CE5 register allowed");
	/* strict TLV iterator */
	uint8_t b[16] = {0};
	struct wlwmi_it it;
	wlwmi_put_tlv(b, 0x20, 8); wlwmi_it_init(&it, b, 12);
	CHECK(wlwmi_next(&it) == 1 && it.tlen == 8 && wlwmi_next(&it) == 0, "TLV iterator happy");
	wlwmi_it_init(&it, b, 11);
	CHECK(wlwmi_next(&it) == -1, "TLV value past the buffer accepted");
	wlwmi_it_init(&it, b, 2);
	CHECK(wlwmi_next(&it) == -1, "TLV header past the buffer accepted");
	/* trailer records */
	struct wlhtc h;
	CHECK(wlhtc_init(&h, wl15_service_ce, 14) == 0 && h.ep[0].ul_pipe == 0 && h.ep[0].dl_pipe == 2, "EP0 pipes");
	uint8_t t1[8] = {1, 8, 0, 0, 1, 1, 0, 0};
	CHECK(wlhtc_trailer(&h, t1, 8) == 1, "credit record longer than the trailer accepted");
	uint8_t t2[8] = {9, 4, 0, 0, 1, 1, 0, 0};
	CHECK(wlhtc_trailer(&h, t2, 8) == 0, "unknown record id must be skipped");
	uint8_t t3[6] = {1, 0, 0, 0, 0, 0};
	h.why = NULL;
	CHECK(wlhtc_trailer(&h, t3, 6) == 1, "credit record shorter than a report accepted");
	/* service map pipes as pci.c:1006-1046 */
	uint8_t ul = 9, dl = 9;
	CHECK(wlhtc_pipes(wl15_service_ce, 14, 0x100, &ul, &dl) == 0 && ul == 3 && dl == 2, "WMI pipes");
	CHECK(wlhtc_pipes(wl15_service_ce, 14, 0x300, &ul, &dl) == 0 && ul == 4 && dl == 1, "HTT pipes");
	CHECK(wlhtc_pipes(wl15_service_ce, 14, 0x105, &ul, &dl) == 1, "unknown service has pipes");
	/* service bits */
	struct wlwmi w;
	memset(&w, 0, sizeof(w));
	w.svc_base[19] = 2; w.svc_seg[2] = 1u << 28; w.svc_ext2[0] = 1u << 25;
	CHECK(wlwmi_svc(&w, 77) && wlwmi_svc(&w, 220) && wlwmi_svc(&w, 281) && !wlwmi_svc(&w, 76) && !wlwmi_svc(&w, 600), "service bits");
	/* HTC frame and credits */
	CHECK(wlhtc_parse_connect(&h, 0x100, true, 4, &(uint32_t){0}, &(uint32_t){0}) == 1, "connect without a reply accepted");
	/* RX post: never more buffers out than the pipe has, even where the ring has room (CE1 512, CE2 64) */
	static const struct wlce_io no_io = { io_never_wr, io_never_rd, io_never_wr, io_never_barrier, io_never_barrier, NULL };
	uint8_t *mem = aligned_alloc(4096, ARENA);
	struct wlce l;
	CHECK(mem != NULL && wlce_layout(&l, mem, 0xa0000000ull, ARENA, wl15_shadow_v3, 36, true) == 0, "local layout");
	for (unsigned pi = 1; pi <= 2; pi++) {
		CHECK(wlce_post_rx(&l, &no_io, &l.pipe[pi], 63) == 1 && l.pipe[pi].posted == 62 &&
		    !strcmp(l.why, "CE RX post beyond the buffers"), "RX post beyond the 62 buffers accepted");
	}
	free(mem);
	/* shadow writer: 4-byte aligned registers 0x8fc..0x998 only */
	CHECK(wl_ce_io_bar_wr(NULL, WLCE_SHADOW_BASE + 2, 0) == 1 && wl_ce_io_bar_wr(NULL, WLCE_SHADOW_BASE - 4, 0) == 1 &&
	    wl_ce_io_bar_wr(NULL, WLCE_SHADOW_BASE + 4 * WLCE_SHADOW_REGS, 0) == 1 && shadow_writes == 0, "shadow writer bounds");
	puts("W20 pure: register set, strict TLV iterator, trailer records, service map pipes, service bits, RX post bound, "
	     "shadow writer bounds");
}

/* ---- C2 scenarios (fuxiwlanqrtr=7) ---- */
static void c2_reset(void) { reset(); wl_w23 = true; }

static unsigned
count_text(const char *s)
{
	unsigned n = 0;
	for (const char *p = strstr(logbuf, s); p; p = strstr(p + 1, s)) { n++; }
	return n;
}

static const uint8_t B_HOME[6] = {0x3c, 0x84, 0x6a, 0x11, 0x22, 0x33}, B_CAFE[6] = {0x00, 0x1a, 0x2b, 0x44, 0x55, 0x66};
static const uint8_t B_HID0[6] = {0xa0, 0xb1, 0xc2, 1, 2, 3}, B_HIDZ[6] = {0xa0, 0xb1, 0xc2, 4, 5, 6};
static const uint8_t B_OFF[6] = {0x74, 0x83, 0xc2, 0xaa, 0xbb, 0xcc}, B_LAB[6] = {0x74, 0x83, 0xc2, 0xdd, 0xee, 0xff};
static const uint8_t B_X1[6] = {2, 0, 0, 0, 0, 1}, B_X2[6] = {2, 0, 0, 0, 0, 2}, B_X3[6] = {2, 0, 0, 0, 0, 3};
static const uint8_t B_X4[6] = {2, 0, 0, 0, 0, 4}, B_X5[6] = {2, 0, 0, 0, 0, 5}, B_BC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* What the happy scan hears, by channel index (0 = 2412, 4 = 2432, 5 = 2437, 10 = 2462, 13 = 5180, 21 = 5500). */
static void
happy_frames(void)
{
	struct mbeacon *b;
	char a33[34];
	memset(a33, 'A', 33); a33[33] = 0;
	add_frame(0, 30, 8, B_CAFE, 0x0401, "Caf\xc3\xa9 \"Free\"", 12, 1, 0, false);
	b = add_frame(0, 22, 8, B_X2, 0x0411, "Trunc", 5, 1, 0, true); b->len = 59;     /* RSN element cut after 5 of 20 bytes */
	b = add_frame(0, 18, 8, B_X3, 0x0401, "LongName12", 10, 1, 0, false); b->len = 41;  /* the SSID element itself cut */
	b = add_frame(0, 40, 8, B_X4, 0x0401, "Short", 5, 1, 0, false); b->len = 30;     /* shorter than the fixed fields */
	b = add_frame(0, 40, 8, B_X4, 0x0401, "CrcBad", 6, 1, 0, false); b->status = 0x01;
	b = add_frame(0, 40, 8, B_X4, 0x0401, "Chan0", 5, 1, 0, false); b->rx_channel = 0; b->chan_freq = 0;
	add_frame(0, 40, 4, B_BC, 0, "ProbeReq", 8, 0, 0, false);                       /* a probe request: not a BSS */
	add_frame(4, 20, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);                   /* adjacent-channel copy, weaker */
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	add_frame(5, 40, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	add_frame(5, 45, 5, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);                   /* probe response, same BSSID */
	b = add_frame(5, 40, 8, B_X5, 0x0401, "NoFrame", 7, 6, 0, false); b->mode = 1;
	b = add_frame(5, 40, 8, B_X5, 0x0401, "BufLen", 6, 6, 0, false); b->mode = 2;
	b = add_frame(5, 40, 8, B_X5, 0x0401, "ShortHdr", 8, 6, 0, false); b->mode = 3;
	b = add_frame(5, 40, 8, B_X5, 0x0401, "SixGHz", 6, 6, 0, false); b->chan_freq = 5955;
	add_frame(10, 25, 8, B_HID0, 0x0411, NULL, 0, 11, 0, true);                       /* hidden: zero-length SSID */
	add_frame(10, 10, 8, B_X1, 0x0401, a33, 33, 11, 0, false);                         /* SSID element of 33 bytes */
	add_frame(13, 50, 8, B_OFF, 0x0011, "Office-5G", 9, 0, 36, true);                  /* channel from HT operation */
	add_frame(13, 35, 8, B_HIDZ, 0x0011, NULL, 8, 0, 36, true);                        /* hidden: 8 zero bytes */
	add_frame(21, 33, 8, B_LAB, 0x0011, "Lab DFS", 7, 0, 100, true);
}

static const char *const happy_w24[] = {
	"FUXI WLAN: W24 bss 00:1a:2b:xx:xx:xx ch 1 freq 2412 dBm -65 privacy 0 rsn 0 beacon frames 1 ssid \"Caf\\xc3\\xa9 \\x22Free\\x22\"\n",
	"FUXI WLAN: W24 bss 02:00:00:xx:xx:xx ch 1 freq 2412 dBm -73 privacy 1 rsn 0 beacon frames 1 ssid \"Trunc\"\n",
	"FUXI WLAN: W24 bss 02:00:00:xx:xx:xx ch 1 freq 2412 dBm -77 privacy 0 rsn 0 beacon frames 1 ssid <none>\n",
	"FUXI WLAN: W24 bss 3c:84:6a:xx:xx:xx ch 6 freq 2437 dBm -48 privacy 1 rsn 1 beacon+probe-resp frames 4 ssid \"HomeNet\"\n",
	"FUXI WLAN: W24 bss a0:b1:c2:xx:xx:xx ch 11 freq 2462 dBm -70 privacy 1 rsn 1 beacon frames 1 ssid <hidden>\n",
	"FUXI WLAN: W24 bss 02:00:00:xx:xx:xx ch 11 freq 2462 dBm -85 privacy 0 rsn 0 beacon frames 1 ssid <invalid length 33>\n",
	"FUXI WLAN: W24 bss 74:83:c2:xx:xx:xx ch 36 freq 5180 dBm -45 privacy 1 rsn 1 beacon frames 1 ssid \"Office-5G\"\n",
	"FUXI WLAN: W24 bss a0:b1:c2:xx:xx:xx ch 36 freq 5180 dBm -60 privacy 1 rsn 1 beacon frames 1 ssid <hidden>\n",
	"FUXI WLAN: W24 bss 74:83:c2:xx:xx:xx ch 100 freq 5500 dBm -62 privacy 1 rsn 1 beacon frames 1 ssid \"Lab DFS\"\n",
	"FUXI WLAN: W24 scan summary frames 12 bss 9 ssids 5\n",
	("FUXI WLAN: W24 scan detail mgmt-rx 20 hidden 3 table-full frames 0 dropped status 1 channel 2 malformed 4 other-mgmt 1 "
	 "ie-truncated 2 noise-floor -95 dBm (L19 default; rssi-conv events 1, not applied)\n"),
	"FUXI WLAN: W23 complete; stop/contain\n",
};

static void
s_scan_happy(void)
{
	c2_reset();
	C.wrong_vdev = true;
	happy_frames();
	CHECK(run() == 0, "C2 happy path failed");
	happy_lines(true);
	static const char *const lines[] = {
		"FUXI WLAN: W22 complete; W23 scan next\n",
		"FUXI WLAN: W23 chan list 38 passive (2.4 GHz 13, 5 GHz 25, dfs 16) pdev 0 commands 1 bytes 1088\n",
		"FUXI WLAN: W23 vdev 0 sta created mac 02:46:55:58:49:01 pdev 0 bands 0x3 streams tx 2 rx 2\n",
		"FUXI WLAN: W23 vdev 0 nss 2, sta ps rx-wake 0 tx-wake 1 pspoll 0, ps mode off\n",
		"FUXI WLAN: W23 scan start id 0xa000 vdev 0 channels 38 passive dwell 150 ms max 8550 ms events 0x5f flags 0x11\n",
		"FUXI WLAN: W23 scan started after ",
		"FUXI WLAN: W23 scan foreign freq 2412\n",
		"FUXI WLAN: W23 scan foreign freq 2422\n",
		"FUXI WLAN: W23 wmi event 0x402b bytes 16 (logged only)\n",
		"FUXI WLAN: W23 scan foreign freq 5500\n",
		"FUXI WLAN: W23 scan foreign freq 5825\n",
		"FUXI WLAN: W23 scan completed reason 0 after ",
		" ms (foreign 38 chan-info 38)\n",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	expect_order(happy_w24, sizeof(happy_w24) / sizeof(happy_w24[0]));
	CHECK(count_text("W23 scan foreign freq") == 38 && count_text("FUXI WLAN: W24 bss ") == 9, "foreign/bss line counts");
	CHECK(fw_chan_cmds == 1 && fw_scan_cmds == 1 && fw_c2_step == 8 && wl_scan.wrong_vdev == 1 && wl_scan.ci_end == 38,
	    "C2 firmware view");
	CHECK(wl_ce.pipe[2].reaped > 2 * 64, "CE2 did not wrap twice during the scan");
	CHECK(n_ev['L'] == n_ev['R'] && n_ev['W'] == shadow_publishes, "barriers");
	expect_not("W23 failure");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: scan over, no STOP_SCAN\n");
	CHECK(fw_stop_cmds == 0, "STOP_SCAN after a completed scan");
	/* C2b: a detail line per CHAN_INFO (38 START with the dwell's counters, 38 END), after COMPLETED one
	 * REQUEST_STATS per DBS PHY pdev (1: 2 GHz, 2: 5 GHz), each answer in six lines, then the after-scan deltas;
	 * the contain line counts both exchanges too (tx 13 + 2, rx 156 + 2) */
	static const char *const c2b[] = {
		"FUXI WLAN: W23 scan foreign freq 2412\n",
		"FUXI WLAN: W23 chan-info freq 2412 flags START nf -96 rx-clear 2112000 cycle 48000000 rx-frame 672000 tx-frame 0 "
		"my-bss-rx 0 mac-clk 320\n",
		"FUXI WLAN: W23 chan-info freq 2412 flags END nf 0 rx-clear 0 cycle 0 rx-frame 0 tx-frame 0 my-bss-rx 0 mac-clk 0\n",
		"FUXI WLAN: W23 scan foreign freq 2417\n",
		"FUXI WLAN: W23 chan-info freq 2417 flags START nf -97 rx-clear 1444000 cycle 47987000 rx-frame 0 tx-frame 0 "
		"my-bss-rx 0 mac-clk 320\n",
		"FUXI WLAN: W23 chan-info freq 2437 flags START nf -96 rx-clear 2132000 cycle 47935000 rx-frame 672000 tx-frame 0 "
		"my-bss-rx 0 mac-clk 320\n",
		"FUXI WLAN: W23 chan-info freq 5825 flags END nf 0 rx-clear 0 cycle 0 rx-frame 0 tx-frame 0 my-bss-rx 0 mac-clk 0\n",
		"FUXI WLAN: W23 scan completed reason 0 after ",
	};
	expect_order(c2b, sizeof(c2b) / sizeof(c2b[0]));
	expect_not("FUXI WLAN: W24 no beacon");
	static const char *const stats[] = {
		"FUXI WLAN: W24 scan detail mgmt-rx 20 ",
		"FUXI WLAN: W23 stats request pdev-stat pdev 1 bands 0x1 vdev 0 sent\n",
		"FUXI WLAN: W23 stats pdev 1 after 2 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 1 unusable 0; ce tx 1 rx 1)\n",
		"FUXI WLAN: W23 stats base chan-nf -98 rx-frame 2457600 rx-clear 5068800 cycle 47961000 tx-frame 0 phy-err 112 chan-tx-pwr 0\n",
		"FUXI WLAN: W23 stats rx1 status-rcvd 20 r0-frags 0 r1-frags 0 r2-frags 0 r3-frags 0 htt-msdus 0 htt-mpdus 0\n",
		("FUXI WLAN: W23 stats rx2 loc-msdus 19 loc-mpdus 19 oversize-amsdu 0 phy-errs 112 phy-err-drop 0 mpdu-errs 1 "
		 "mid-ppdu-route-change 0\n"),
		("FUXI WLAN: W23 stats tx1 comp-queued 0 comp-delivered 0 msdu-enqued 0 mpdu-enqued 0 local-enqued 0 local-freed 0 "
		 "hw-queued 0\n"),
		"FUXI WLAN: W23 stats tx2 hw-reaped 0 underrun 0 tx-abort 0 mpdus-requed 0 tx-ko 0 self-triggers 0 pdev-resets 0\n",
		"FUXI WLAN: W23 stats request pdev-stat pdev 2 bands 0x2 vdev 0 sent\n",
		"FUXI WLAN: W23 stats pdev 2 after 2 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 1 unusable 0; ce tx 1 rx 1)\n",
		"FUXI WLAN: W23 stats tx2 hw-reaped 0 ",
		"FUXI WLAN: W23 stats after-scan wmi mgmt-rx +0 frames +0 bss +0 scan-events +0 chan-info +0 other +0\n",
		"FUXI WLAN: W23 complete; stop/contain\n",
		"FUXI WLAN: W23 contain: scan over, no STOP_SCAN\n",
		"FUXI WLAN: W20 ce contain posted 282 reaped 158 tx 15\n",
	};
	expect_order(stats, sizeof(stats) / sizeof(stats[0]));
	CHECK(count_text("FUXI WLAN: W23 chan-info freq ") == 76 && count_text(" flags START ") == 38 && fw_stats_req == 2 &&
	    wl_stats.have && wl_scan.ci_seen == 76 && count_text("FUXI WLAN: W23 stats base ") == 2,
	    "C2b lines and one stats request per DBS PHY pdev");
	expect_not("chan-info lines suppressed");
	expect_not("wmi event 0x1d004");  /* UPDATE_STATS feeds the stats parser, not the unknown-event lines */
	puts("W23 happy path: 38 passive channels in one command, STA vdev (K-W2 MAC) + NSS + 3 STA PS params + PS off, "
	     "passive START_SCAN word by word, 38 foreign channels, CHAN_INFO, 20 MGMT_RX into 9 BSS (dedup, probe response, "
	     "hidden, all-zero, 33-byte SSID, cut IEs, HT channel, escaped SSID), drops counted, contained without STOP_SCAN; "
	     "C2b: 76 CHAN_INFO lines with the dwell counters, REQUEST_STATS for pdev 1 then 2 after COMPLETED, pdev "
	     "base/rx/tx lines, after-scan deltas");
}

static void
s_scan_bits(void)
{
	c2_reset();
	wl20_bits = WL20_HTTVER | WL20_BAND24 | WL20_PDEVCAP;
	C.exp_n = 13; C.exp_pdev = 1;
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	CHECK(run() == 0, "C2 w20 0x92 failed");
	static const char *const lines[] = {
		"FUXI WLAN: W23 htt version 3.4 (w20 0x02)\n",
		"FUXI WLAN: W23 chan list 13 passive (2.4 GHz 13, 5 GHz 0, dfs 0) pdev 1 commands 1 bytes 388\n",
		"FUXI WLAN: W23 vdev 0 sta created mac 02:46:55:58:49:01 pdev 1 bands 0x3 streams tx 2 rx 2\n",
		"FUXI WLAN: W23 scan start id 0xa000 vdev 0 channels 13 passive dwell 150 ms max 2925 ms events 0x5f flags 0x11\n",
		"FUXI WLAN: W24 bss 3c:84:6a:xx:xx:xx ch 6 freq 2437 dBm -48 privacy 1 rsn 1 beacon frames 1 ssid \"HomeNet\"\n",
		"FUXI WLAN: W24 scan summary frames 1 bss 1 ssids 1\n",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	CHECK(fw_htt_req == 1 && wl_ce.pipe[4].posted == 1 && wl_ce.pipe[4].reaped == 1 && count_text("scan foreign freq") == 13,
	    "HTT version on CE4, 13 channels");
	expect_not("11:22:33");  /* owner decision 6: no whole BSSID in any line */
	contained_and_freed();
	/* CE0 + CE3 + the one CE4 send; C2b: the two REQUEST_STATS (tx) and their answers (rx), as their lines say */
	expect("FUXI WLAN: W23 stats pdev 1 after 2 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 1 unusable 0; ce tx 1 rx 1)\n");
	expect("FUXI WLAN: W23 stats pdev 2 after 2 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 1 unusable 0; ce tx 1 rx 1)\n");
	expect("FUXI WLAN: W20 ce contain posted 188 reaped 64 tx 16\n");
	/* HTT major 2: L19 refuses it (dp_tx.c:1319-1323); a silent target: 3 s */
	c2_reset(); wl20_bits = WL20_HTTVER; C.htt_major = 2;
	CHECK(run() == 1, "HTT major 2 accepted");
	expect("FUXI WLAN: W23 htt version 2.4 (w20 0x02)\n");
	expect("FUXI WLAN: W23 failure: W23 HTT major version is not 3 (L19 refuses it too, dp_tx.c:1319-1323); contain, no retry\n");
	contained_and_freed();
	expect_not("W23 contain");
	c2_reset(); wl20_bits = WL20_HTTVER; C.htt_silent = true;
	CHECK(run() == 1, "no HTT version reply accepted");
	expect("FUXI WLAN: W23 timeout HTT VERSION_CONF (3000 ms); contain, no retry\n");
	expect(" CE4 1/1 rx CE1 ");  /* the snapshot shows the one CE4 send and its completion */
	CHECK(fw_c2_step == 0 && fw_chan_cmds == 0, "commands after a failed HTT exchange");
	contained_and_freed();
	puts("W23 w20 0x92: HTT version 3.4 over CE4/CE1 first, 13 channels (2.4 GHz), pdev from EXT caps, BSSIDs masked, "
	     "CE4 in the contain/snapshot lines; HTT major 2 refused, no HTT reply times out at 3000 ms");
}

static void
s_scan_split_and_caps(void)
{
	c2_reset(); C.max_msg = 300;  /* (300 - 20) / 28 = 10 channels per command, as L19 splits (wmi.c:2895-2915) */
	CHECK(run() == 0, "split channel list");
	expect("FUXI WLAN: W23 chan list 38 passive (2.4 GHz 13, 5 GHz 25, dfs 16) pdev 0 commands 4 bytes 1160\n");
	CHECK(fw_chan_cmds == 4 && fw_chan_seen == 38, "channel list split");
	contained_and_freed();
	c2_reset(); C.phy_bands = 0;
	CHECK(run() == 0, "no MAC/PHY bands");
	expect("FUXI WLAN: W23 vdev 0 sta created mac 02:46:55:58:49:01 pdev 0 bands 0x3 streams tx 2 rx 2 (rf-chains: no MAC/PHY caps)\n");
	contained_and_freed();
	c2_reset(); C.phy_bands = 1; C.phy_mask = 1;  /* a 2.4 GHz-only 1x1 PHY: no 5 GHz streams, NSS 1 */
	CHECK(run() == 0, "2.4 GHz-only caps");
	expect("FUXI WLAN: W23 vdev 0 sta created mac 02:46:55:58:49:01 pdev 0 bands 0x1 streams tx 1 rx 1\n");
	expect("FUXI WLAN: W23 vdev 0 nss 1, sta ps");
	contained_and_freed();
	/* L19's preferred hw mode (DBS over SINGLE, wmi.c:4538-4551) whatever the listing order: with DBS first and a
	 * 1x1 SINGLE last, the streams and NSS still come from the DBS PHYs (indices 0 and 1, the last one wins) */
	c2_reset(); C.ext_single_last = true;
	CHECK(run() == 0, "SINGLE listed after DBS");
	expect("FUXI WLAN: W23 vdev 0 sta created mac 02:46:55:58:49:01 pdev 0 bands 0x3 streams tx 2 rx 2\n");
	expect("FUXI WLAN: W23 vdev 0 nss 2, sta ps");
	CHECK(wl_wmi.phy_bands == 3 && wl_wmi.phy_tx_mask == 3 && wl_wmi.phy_rx_mask == 3, "chain masks not from the DBS PHYs");
	contained_and_freed();
	puts("W23 channel list split by a 300-byte WMI max message (4 commands, APPEND after the first), streams from the "
	     "preferred hw mode's MAC/PHY caps or rf-chains, 2.4 GHz-only caps, SINGLE listed after DBS");
}

static void
s_scan_full(void)
{
	c2_reset();
	for (unsigned i = 0; i < 70; i++) {
		uint8_t bssid[6] = {0x10, 0x20, 0x30, 0, 0, (uint8_t)i};
		char ssid[16];
		snprintf(ssid, sizeof(ssid), "Net-%02u", i);
		add_frame(i % 38, 30, 8, bssid, 0x0401, ssid, 6, 0, 0, false);
		if (i == 37) {  /* channel 37 comes after the table filled up: heard twice more, counted per frame */
			add_frame(i % 38, 31, 8, bssid, 0x0401, ssid, 6, 0, 0, false);
			add_frame(i % 38, 32, 5, bssid, 0x0401, ssid, 6, 0, 0, false);
		}
	}
	CHECK(run() == 0, "70 BSSIDs");
	expect("FUXI WLAN: W24 scan summary frames 72 bss 64 ssids 64\n");
	expect("FUXI WLAN: W24 scan detail mgmt-rx 72 hidden 0 table-full frames 8 ");
	CHECK(count_text("FUXI WLAN: W24 bss ") == 64 && wl_scan.nbss == 64, "64 BSS lines");
	expect("FUXI WLAN: W24 bss 10:20:30:xx:xx:xx ch 1 freq 2412 dBm -65 privacy 0 rsn 0 beacon frames 1 ssid \"Net-00\"\n");
	contained_and_freed();
	puts("W23 BSS table: 70 BSSIDs heard, 64 kept and printed; 6 BSSIDs past the table in 8 frames, counted as "
	     "table-full frames");
}

static void
s_scan_failures(void)
{
	c2_reset(); C.scan_mode = 1;
	CHECK(run() == 1, "START_FAILED accepted");
	expect("FUXI WLAN: W23 scan START_FAILED reason 4 (ladder: w20 0x80, then 0x10, then 0x40)\n");
	expect("FUXI WLAN: W23 failure: W23 scan START_FAILED; contain, no retry\n");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: scan start failed, no STOP_SCAN\n");
	CHECK(fw_stop_cmds == 0, "STOP_SCAN after START_FAILED");
	/* no STARTED in 1 s (L19 mac.c:5046-5053): STOP_SCAN before the latch, values of mac.c:4883-4896 */
	c2_reset(); C.scan_mode = 2; wl20_bits = WL20_PDEVCAP; C.exp_pdev = 1;
	CHECK(run() == 1, "no STARTED accepted");
	expect("FUXI WLAN: W23 timeout scan STARTED (1000 ms); contain, no retry\n");
	expect("FUXI WLAN: W20 snapshot scan started 0 done 0 start-failed 0 events 0 ");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: STOP_SCAN sent (best effort), tx drained 1 after 1 ms\n");
	CHECK(fw_stop_cmds == 1 && fw_stop_pdev == 1, "STOP_SCAN fields");
	/* the same, but CE3 never reports the STOP_SCAN descriptor done: the drain gives up at exactly 100 ms */
	c2_reset(); C.scan_mode = 2; C.stop_tx_stuck = true;
	CHECK(run() == 1, "no STARTED (stuck STOP_SCAN) accepted");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: STOP_SCAN sent (best effort), tx drained 0 after 100 ms\n");
	CHECK(fw_stop_cmds == 1 && !wlce_tx_idle(&wl_ce.pipe[3]), "stuck STOP_SCAN drain");
	/* the W11 guard fails once the firmware took START_SCAN (AMSS left EE 2, SMMU fault, ...): no STOP_SCAN, no
	 * doorbell, no descriptor (scan_plan.md 4: a healthy transport only) */
	c2_reset(); C.guard_after_start = true;
	CHECK(run() == 1, "guard failure after START_SCAN accepted");
	expect("FUXI WLAN: W23 failure: AMSS/M0, ownership, discovery or access/SMMU fault; contain, no retry\n");
	CHECK(fw_scan_cmds == 1 && !wl_scan.done && !wl_scan.start_failed && wl_ce.errors == 0, "guard case set-up");
	{
		unsigned tx = wl_ce.pipe[3].posted, sw = shadow_writes, cw = ce_writes;
		contained_and_freed();
		expect("FUXI WLAN: W23 contain: STOP_SCAN not sent (transport not healthy)\n");
		CHECK(fw_stop_cmds == 0 && wl_ce.pipe[3].posted == tx && shadow_writes == sw && ce_writes == cw,
		    "STOP_SCAN or a doorbell after a guard failure");
	}
	/* COMPLETED before STARTED is ignored as L19 does (wmi.c:6363-6381): still waiting, then STOP_SCAN */
	c2_reset(); C.scan_mode = 4;
	CHECK(run() == 1, "COMPLETED before STARTED accepted");
	CHECK(wl_scan.ignored == 1 && !wl_scan.done, "early COMPLETED not ignored");
	contained_and_freed();
	CHECK(fw_stop_cmds == 1 && fw_stop_pdev == 0, "STOP_SCAN after an ignored COMPLETED");
	/* STARTED, then no COMPLETED: the table is still printed, then STOP_SCAN */
	c2_reset(); C.scan_mode = 3;
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	CHECK(run() == 1, "no COMPLETED accepted");
	static const char *const lines[] = {
		"FUXI WLAN: W23 timeout scan COMPLETED (8750 ms); report, contain, no retry\n",
		"FUXI WLAN: W24 bss 3c:84:6a:xx:xx:xx ch 6 freq 2437 dBm -48 privacy 1 rsn 1 beacon frames 1 ssid \"HomeNet\"\n",
		"FUXI WLAN: W24 scan summary frames 1 bss 1 ssids 1\n",
		"FUXI WLAN: W20 snapshot scan COMPLETED: ",
		"FUXI WLAN: W20 snapshot scan started 1 done 0 start-failed 0 events 39 foreign 38 chan-info 38 mgmt-rx 1 frames 1 bss 1 why -\n",
		"FUXI WLAN: W23 failure: scan COMPLETED; contain, no retry\n",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: STOP_SCAN sent (best effort)");
	CHECK(fw_stop_cmds == 1, "STOP_SCAN after a scan timeout");
	/* no credits left for STOP_SCAN: skipped, never waited for */
	c2_reset(); C.credits = 1; C.refund_later = true; C.no_start_refund = true; C.scan_mode = 2;
	CHECK(run() == 1, "no STARTED with no credits accepted");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: STOP_SCAN not sent (no HTC credits)\n");
	CHECK(fw_stop_cmds == 0, "STOP_SCAN without credits");
	/* a SCAN_EVENT TLV past its event: stops the run (as C1 does for TLVs); no STOP_SCAN on a refused transport */
	c2_reset(); C.bad_scan_tlv = true;
	CHECK(run() == 1, "SCAN_EVENT past the buffer accepted");
	expect("FUXI WLAN: W23 failure: WMI SCAN_EVENT TLV past the buffer; contain, no retry\n");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: STOP_SCAN not sent (transport not healthy)\n");
	CHECK(fw_stop_cmds == 0, "STOP_SCAN over a refused transport");
	puts("W23 failures: START_FAILED (no STOP_SCAN), no STARTED in 1 s and COMPLETED before STARTED (STOP_SCAN with "
	     "L19's fields before the latch; a stuck drain ends at 100 ms), a guard failure after START_SCAN (no STOP_SCAN, "
	     "no doorbell), no COMPLETED (table printed, then STOP_SCAN), no credits (STOP_SCAN skipped), SCAN_EVENT TLV "
	     "past the event (stop, no STOP_SCAN); every run contained and freed");
}

static void
s_scan_quiet(void)
{
	c2_reset(); wl15_variant = 1;
	CHECK(run() == 0, "silent scan");
	static const char *const lines[] = {
		"FUXI WLAN: W24 scan summary frames 0 bss 0 ssids 0\n",
		"FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung W24b = w20 0x02\n",
		"FUXI WLAN: W23 complete; stop/contain\n",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	CHECK(shadow_writes == 0 && direct_publishes > 100, "direct route during the scan");
	contained_and_freed();
	/* 20 unknown events after READY use up the 16-line budget in W22; the RSSI dBm conversion event (0x402b),
	 * whose noise floor L19 would apply to MGMT_RX, is still logged once and counted in the W24 detail line */
	c2_reset(); C.extra_unknown = 20;
	CHECK(run() == 0, "silent scan after 20 unknown events");
	CHECK(count_text("(logged only)") == 17 && count_text("W22 wmi event 0x7ff3") == 15 && wl23_rssi_conv == 1, "16-line budget");
	expect("FUXI WLAN: W23 wmi event 0x402b bytes 16 (logged only)\n");
	expect("noise-floor -95 dBm (L19 default; rssi-conv events 1, not applied)\n");
	contained_and_freed();
	puts("W23 nothing heard (and fuxiwlanw15=1, every pointer to R2): COMPLETED with CHAN_INFO, summary 0, W24b hint; "
	     "0x402b logged past the 16-line budget and counted");
}

static void
s_scan_pure(void)
{
	struct wlscan_bss b;
	char t[WLSCAN_SSID_TEXT];
	uint8_t f[400];
	uint32_t tr = 0, len;
	memset(&b, 0, sizeof(b));
	b.ssid_state = WLSCAN_SSID_OK; b.ssid_len = 5; memcpy(b.ssid, "a\"\\\x7f\x01", 5);
	wlscan_ssid_text(&b, t); CHECK(!strcmp(t, "ssid \"a\\x22\\x5c\\x7f\\x01\""), "SSID escaping");
	b.ssid_len = 32; memset(b.ssid, 0xff, 32);
	wlscan_ssid_text(&b, t); CHECK(strlen(t) == 6 + 32 * 4 + 1 && strlen(t) < WLSCAN_SSID_TEXT, "32-byte SSID text size");
	b.ssid_state = WLSCAN_SSID_BAD; b.ssid_len = 255;
	wlscan_ssid_text(&b, t); CHECK(!strcmp(t, "ssid <invalid length 255>"), "invalid length text");
	b.ssid_state = WLSCAN_SSID_HIDDEN; wlscan_ssid_text(&b, t); CHECK(!strcmp(t, "ssid <hidden>"), "hidden text");
	b.ssid_state = WLSCAN_SSID_NONE; wlscan_ssid_text(&b, t); CHECK(!strcmp(t, "ssid <none>"), "none text");
	/* parser */
	len = mk_frame(f, 8, B_HOME, 0x0411, "X", 1, 6, 0, true);
	memset(&b, 0, sizeof(b));
	CHECK(wl80211_parse_bss(f, len, &b, &tr) == 0 && b.chan == 6 && b.rsn && b.privacy && b.beacon && !tr, "beacon parse");
	CHECK(wl80211_parse_bss(f, 1, &b, &tr) == 2 && wl80211_parse_bss(f, 35, &b, &tr) == 2, "too short");
	f[0] = 0x08; CHECK(wl80211_parse_bss(f, len, &b, &tr) == 1, "data frame taken as a BSS");
	f[0] = 0x40; CHECK(wl80211_parse_bss(f, len, &b, &tr) == 1, "probe request taken as a BSS");
	len = mk_frame(f, 5, B_HOME, 0x0001, "Y", 1, 0, 0, false);
	f[len] = 61; f[len + 1] = 1; f[len + 2] = 44; f[len + 3] = 3; f[len + 4] = 1; f[len + 5] = 40;  /* HT op 44, then DS 40 */
	memset(&b, 0, sizeof(b));
	CHECK(wl80211_parse_bss(f, len + 6, &b, &tr) == 0 && b.chan == 40 && b.probe_resp && !b.privacy, "DS wins over HT operation");
	CHECK(wl80211_parse_bss(f, len + 7, &b, &tr) == 0 && tr == 1, "one stray byte after the last element");
	/* the exact element bound: the DS body sits one byte past the frame (still inside f[], so only the bound
	 * stops the read): truncated, the channel from HT operation */
	memset(&b, 0, sizeof(b)); tr = 0;
	CHECK(wl80211_parse_bss(f, len + 5, &b, &tr) == 0 && b.chan == 44 && tr == 1, "DS body one byte past the frame");
	/* table: dedup keeps the strongest frequency, 64 entries */
	struct wlscan *s = calloc(1, sizeof(*s));
	CHECK(s != NULL, "alloc");
	wlscan_init(s, 38);
	memset(&b, 0, sizeof(b)); b.ssid_state = WLSCAN_SSID_HIDDEN; b.dbm = -80; b.freq = 2432; b.frames = 1;
	wlscan_bss_add(s, &b);
	b.ssid_state = WLSCAN_SSID_OK; b.ssid_len = 1; b.ssid[0] = 'Z'; b.dbm = -40; b.freq = 2437;
	wlscan_bss_add(s, &b);
	CHECK(s->nbss == 1 && s->bss[0].frames == 2 && s->bss[0].dbm == -40 && s->bss[0].freq == 2437 &&
	    s->bss[0].ssid_state == WLSCAN_SSID_OK, "dedup update");
	for (unsigned i = 1; i < 70; i++) { b.bssid[5] = (uint8_t)i; wlscan_bss_add(s, &b); }
	CHECK(s->nbss == 64 && s->table_full == 6 && wlscan_ssids(s) == 1, "64 entries, one distinct SSID");
	free(s);
	/* encoders: bounds and the channel-per-command limit */
	uint8_t out[2048];
	CHECK(wlwmi_chan_list_max(2048) == 72 && wlwmi_chan_list_max(300) == 10 && wlwmi_chan_list_max(47) == 0 &&
	    wlwmi_chan_list_max(0) == 0, "channels per command");
	CHECK(wlwmi_encode_chan_list(0, 38, false, 0, out, 1087, &len) == 1 && wlwmi_encode_chan_list(30, 9, false, 0, out, 2048, &len) == 1 &&
	    wlwmi_encode_chan_list(0, 0, false, 0, out, 2048, &len) == 1, "channel list bounds");
	CHECK(wlwmi_encode_start_scan(39, 1, out, 2048, &len) == 1 && wlwmi_encode_start_scan(38, 1, out, 339, &len) == 1 &&
	    wlwmi_encode_start_scan(38, 1, out, 340, &len) == 0 && len == 340, "start scan bounds");
	CHECK(wlwmi_encode_vdev_create(B_HOME, 0, 3, 2, 2, out, 87, &len) == 1, "vdev create bounds");
	CHECK(wlscan_max_time(38) == 8550 && wlscan_max_time(13) == 2925, "max scan time n x 150 x 3/2");
	/* CE4: 16 TX slots on 2048 SRC entries; with 16 sends in flight the 17th is refused before it reuses slot 0 */
	static const struct wlce_io sink_io = { io_never_wr, io_never_rd, io_sink_bar, io_model_w, io_model_r, NULL };
	uint8_t *mem = aligned_alloc(4096, ARENA), msg[8] = {1, 2, 3, 4, 5, 6, 7, 8};
	struct wlce l;
	CHECK(mem != NULL, "alloc");
	memset(mem, 0, ARENA);
	CHECK(wlce_layout(&l, mem, 0xa0000000ull, ARENA, wl15_shadow_v3, 36, true) == 0, "C2 local layout");
	for (unsigned i = 0; i < 16; i++) { CHECK(wlce_send(&l, &sink_io, &l.pipe[4], 1, msg, sizeof(msg)) == 0, "CE4 send"); }
	CHECK(wlce_send(&l, &sink_io, &l.pipe[4], 1, msg, sizeof(msg)) == 1 && l.why && !strcmp(l.why, "CE TX slots all in flight") &&
	    l.pipe[4].posted == 16, "a 17th CE4 send with 16 in flight accepted");
	free(mem);
	puts("W23 pure: SSID escaping/hidden/none/invalid text, 802.11 parse (types, short, DS over HT, stray byte, element "
	     "body one byte past the frame), table dedup/64, encoder bounds, channels per command, max scan time, CE4 slots "
	     "in flight bound");
}

/* ---- C2b scenarios (fuxiwlanqrtr=7): CHAN_INFO detail and the pdev stats requests ---- */
/* Every completed log line. XNU's console keeps at most 255 bytes of a printf line, newline included, and drops the
 * rest with the newline when drain_uart_sync is off, as on the kits (osfmk/console/serial_general.c:159-188,
 * SERIAL_CONS_BUF_SIZE 256): no FUXI WLAN line may pass 254 bytes. Then the scenario's device-side action at a line
 * that has no poll between it and the next production step: C.on_line 1, CE3's device TP word one entry back right
 * after the W24 detail line (the next CE3 access is the first REQUEST_STATS send); 2, a beacon MGMT_RX 1 ms after
 * the first REQUEST_STATS went out (inside its TX drain / UPDATE_STATS wait); 3, the W11 guard fails once the
 * last channel's FOREIGN_CHAN line is out (the scan wait fails). */
static void
model_line(const char *l, size_t len)
{
	static const uint8_t late[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
	if (len >= 9 && !strncmp(l, "FUXI WLAN", 9) && len > 254) { fail("a FUXI WLAN line longer than 254 bytes (console cut)"); }
	if (C.on_line == 1 && len >= 26 && !strncmp(l, "FUXI WLAN: W24 scan detail", 26)) {
		C.on_line = 0;
		write_rdp(&dr[5], (dr[5].dev + dr[5].size - 4u) % dr[5].size);
	}
	if (C.on_line == 3 && len >= 37 && !strncmp(l, "FUXI WLAN: W23 scan foreign freq 5825", 37)) {
		C.on_line = 0;
		guard_bad = true;  /* the W11 guard fails in the scan wait after the last channel */
	}
	if (C.on_line == 2 && len >= 44 && !strncmp(l, "FUXI WLAN: W23 stats request pdev-stat ", 39) && !strncmp(l + len - 5, " sent", 5)) {
		C.on_line = 0;
		fw_mgmt_rx(add_frame(0, 40, 8, late, 0x11, "Late", 4, 1, 0, false), 1);
	}
}

/* The stats parser alone on the message in mb: the reason an event is unusable, or "usable". */
static const char *
c2b_parse(struct wlstats *st)
{
	memset(st, 0, sizeof(*st));
	wlstats_parse(st, mb, ml);
	CHECK(st->events == 1 && st->have == !st->unusable && st->unusable <= 1 && (st->have || st->why), "stats parse counts");
	return st->have ? "usable" : st->why;
}

static void
s_c2b_pure(void)
{
	struct wlstats st;
	fw_stats_pdev = 1;  /* the fixed TLV's pdev id */
	/* L19's shape, every word where L19 has it (struct ath12k_wmi_pdev_stats_params, wmi_stats_event) */
	m_reset(); m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 176); m_pdev_stats(true);
	CHECK(!strcmp(c2b_parse(&st), "usable") && (int32_t)st.w[WLWMI_PS_CHAN_NF] == -97 && st.ev[WLWMI_SE_STATS_ID] == 4 &&
	    st.ev[WLWMI_SE_NUM_PDEV_STATS] == 1 && st.ev[WLWMI_SE_PDEV_ID] == 1 && st.bytes == 176, "L19-shaped event");
	for (unsigned i = 1; i < WLWMI_PS_WORDS; i++) { CHECK(st.w[i] == 1000u + i, "pdev stats word"); }
	CHECK(st.w[WLWMI_PS_RX_FRAME_COUNT] == 1002 && st.w[WLWMI_PS_CYCLE_COUNT] == 1004 && st.w[WLWMI_PS_COMP_QUEUED] == 1007 &&
	    st.w[WLWMI_PS_TXOP_OVF] == 1029 && st.w[WLWMI_PS_STATUS_RCVD] == 1031 && st.w[WLWMI_PS_HTT_MPDUS] == 1037 &&
	    st.w[WLWMI_PS_LOC_MPDUS] == 1039 && st.w[WLWMI_PS_MPDU_ERRS] == 1043, "named pdev fields at L19's word offsets");
	/* a second, different usable event (firmware sends UPDATE_STATS back to back) is counted, not taken */
	m_reset(); m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 176); m_pdev_stats(false);
	wlstats_parse(&st, mb, ml);
	CHECK(st.events == 2 && st.have && st.w[WLWMI_PS_HTT_MPDUS] == 1037 && (int32_t)st.w[WLWMI_PS_CHAN_NF] == -97,
	    "a later event is counted, the first one kept");
	/* unusable: the reason is kept, nothing fails */
	m_reset(); m_stats_fixed(4, 1, 0, 0, 40); m_tlv(0x11, 176); m_pdev_stats(false);
	CHECK(!strcmp(c2b_parse(&st), "stats fixed TLV shorter than 44 bytes"), "short fixed TLV");
	m_reset(); m_tlv(0x11, 176); m_pdev_stats(false); m_stats_fixed(4, 1, 0, 0, 44);
	CHECK(!strcmp(c2b_parse(&st), "stats byte array before the fixed TLV"), "byte array first (L19: no ev, -EPROTO)");
	m_reset(); m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 177); m_pdev_stats(false);
	CHECK(!strcmp(c2b_parse(&st), "stats TLV past the event"), "TLV past the event");
	m_reset(); m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 172); m_zero(172);
	CHECK(!strcmp(c2b_parse(&st), "stats byte array shorter than its entries"), "pdev entry cut");
	/* the entry bounds at the edge: one byte short of a pdev entry, of a vdev entry */
	m_reset(); m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 175); m_zero(175);
	CHECK(!strcmp(c2b_parse(&st), "stats byte array shorter than its entries"), "pdev entry 1 byte short");
	m_reset(); m_stats_fixed(4, 1, 1, 0, 44); m_tlv(0x11, 163); m_zero(163);
	CHECK(!strcmp(c2b_parse(&st), "stats byte array shorter than its entries"), "vdev entry 1 byte short");
	m_reset(); m_stats_fixed(4, 0, 0, 0, 44); m_tlv(0x11, 176); m_pdev_stats(false);
	CHECK(!strcmp(c2b_parse(&st), "no pdev stats in the event"), "no pdev entry announced");
	m_reset();
	CHECK(!strcmp(c2b_parse(&st), "no stats fixed TLV"), "empty event");
	m_reset(); m_stats_fixed(4, 1, 0xffffffffu, 0, 44); m_tlv(0x11, 352); m_zero(352);
	CHECK(!strcmp(c2b_parse(&st), "stats byte array shorter than its entries"), "4 billion vdev entries: bounded by the array");
	/* L19's order: 2 vdev entries, 1 beacon entry, then 2 pdev entries; the first pdev entry is the one */
	m_reset(); m_stats_fixed(4, 2, 2, 1, 44); m_tlv(0x11, 2 * 164 + 12 + 2 * 176); m_zero(2 * 164 + 12);
	m_pdev_stats(true); m_pdev_stats(false);
	CHECK(!strcmp(c2b_parse(&st), "usable") && st.w[WLWMI_PS_RX_FRAME_COUNT] == 1002 && st.bytes == 2 * 164 + 12 + 2 * 176 &&
	    st.ev[WLWMI_SE_NUM_VDEV_STATS] == 2 && st.ev[WLWMI_SE_NUM_BCN_STATS] == 1, "vdev, beacon, then pdev entries");
	/* a later byte array that is short spoils the event, as L19's -EPROTO does */
	m_reset(); m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 176); m_pdev_stats(true); m_tlv(0x11, 8); m_zero(8);
	CHECK(!strcmp(c2b_parse(&st), "stats byte array shorter than its entries"), "second byte array short");
	/* other TLVs (per-chain RSSI 0x220, an empty ARRAY_STRUCT) are skipped */
	m_reset(); m_tlv(0x220, 4); m_u32(0); m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x12, 0); m_tlv(0x11, 176); m_pdev_stats(true);
	CHECK(!strcmp(c2b_parse(&st), "usable"), "unrelated TLVs");
	/* routing: an unknown event until the request is out, then the stats parser; never a WMI failure */
	{
		static uint8_t ev[WLCE_BUF];
		struct wlwmi w;
		uint32_t id = 0;
		memset(&w, 0, sizeof(w));
		m_reset(); m_stats_fixed(4, 1, 0, 0, 44); m_tlv(0x11, 999);
		p32(ev, 0x1d004); memcpy(ev + 4, mb, ml);
		CHECK(wlwmi_rx(&w, ev, 4 + ml, &id) == 0 && id == 0x1d004 && w.unknown_events == 1, "UPDATE_STATS before the request");
		w.stats = &st; memset(&st, 0, sizeof(st));
		CHECK(wlwmi_rx(&w, ev, 4 + ml, &id) == 0 && !w.why && st.unusable == 1 && w.unknown_events == 1 && w.events == 2,
		    "a malformed UPDATE_STATS failed WMI");
	}
	/* CHAN_INFO: every word of a well-formed event kept (the C2b line reads them), nothing from a short one */
	{
		struct wlscan *s = calloc(1, sizeof(*s));
		CHECK(s != NULL, "alloc");
		wlscan_init(s, 38);
		s->started = true;
		m_reset(); m_tlv(0x26, 56);
		for (unsigned i = 0; i < 14; i++) { m_u32(0x100u + i); }
		CHECK(wlscan_parse_chan_info(s, mb, ml) == 0 && s->ci_seen == 1, "CHAN_INFO kept");
		for (unsigned i = 0; i < 14; i++) { CHECK(s->ci[i] == 0x100u + i, "CHAN_INFO word"); }
		CHECK(s->ci[WLWMI_CI_FREQ] == 0x101 && s->ci[WLWMI_CI_RX_CLEAR_COUNT] == 0x104 && s->ci[WLWMI_CI_CYCLE_COUNT] == 0x105 &&
		    s->ci[WLWMI_CI_RX_FRAME_COUNT] == 0x108 && s->ci[WLWMI_CI_MY_BSS_RX_CYCLE_COUNT] == 0x109 &&
		    s->ci[WLWMI_CI_TX_FRAME_CNT] == 0x10b && s->ci[WLWMI_CI_MAC_CLK_MHZ] == 0x10c, "CHAN_INFO fields at L19's offsets");
		m_reset(); m_tlv(0x26, 52); m_zero(52);
		CHECK(wlscan_parse_chan_info(s, mb, ml) == 0 && s->ci_seen == 1 && s->bad_events == 1, "short CHAN_INFO kept");
		free(s);
	}
	/* encoder: 28 bytes, bounds */
	{
		uint8_t out[64];
		uint32_t len = 0;
		CHECK(wlwmi_encode_stats_req(1, out, 27, &len) == 1, "stats request in 27 bytes");
		CHECK(wlwmi_encode_stats_req(2, out, 28, &len) == 0 && len == 28 && g32(out) == 0x16001 && g32(out + 4) == (0x8fu << 16 | 20u) &&
		    g32(out + 8) == 4 && !g32(out + 12) && !g32(out + 16) && !g32(out + 20) && g32(out + 24) == 2, "stats request words");
	}
	puts("C2b pure: UPDATE_STATS parse (L19 shape, every pdev word, a later different event counted, the first kept; short "
	     "fixed TLV, array first, TLV past the event, cut entry, pdev and vdev entries 1 byte short, no pdev entry, empty, 4 "
	     "billion vdev entries, L19's vdev/beacon/pdev order with its counts and bytes, a spoiling second array, unrelated "
	     "TLVs), routing only after the request and never fatal, CHAN_INFO words kept, request encoder");
}

static void
s_c2b_stats(void)
{
	/* L19's entry order: two vdev entries and a beacon entry first; every pdev field shows at its own offset, the event's counts
	 * and the byte array length on the first line */
	c2_reset(); C.stats_mode = 5;
	CHECK(run() == 0, "C2b L19 order");
	static const char *const lines[] = {
		"FUXI WLAN: W23 stats request pdev-stat pdev 1 bands 0x1 vdev 0 sent\n",
		"FUXI WLAN: W23 stats pdev 1 after 2 ms (stats-id 0x4 pdevs 1 vdevs 2 bcns 1 bytes 516 events 1 unusable 0; ce tx 1 rx 1)\n",
		"FUXI WLAN: W23 stats base chan-nf -97 rx-frame 1002 rx-clear 1003 cycle 1004 tx-frame 1001 phy-err 1005 chan-tx-pwr 1006\n",
		("FUXI WLAN: W23 stats rx1 status-rcvd 1031 r0-frags 1032 r1-frags 1033 r2-frags 1034 r3-frags 1035 htt-msdus 1036 "
		 "htt-mpdus 1037\n"),
		("FUXI WLAN: W23 stats rx2 loc-msdus 1038 loc-mpdus 1039 oversize-amsdu 1040 phy-errs 1041 phy-err-drop 1042 "
		 "mpdu-errs 1043 mid-ppdu-route-change 1030\n"),
		("FUXI WLAN: W23 stats tx1 comp-queued 1007 comp-delivered 1008 msdu-enqued 1009 mpdu-enqued 1010 local-enqued 1012 "
		 "local-freed 1013 hw-queued 1014\n"),
		("FUXI WLAN: W23 stats tx2 hw-reaped 1015 underrun 1016 tx-abort 1017 mpdus-requed 1018 tx-ko 1019 self-triggers 1021 "
		 "pdev-resets 1026\n"),
		"FUXI WLAN: W23 stats request pdev-stat pdev 2 bands 0x2 vdev 0 sent\n",
		"FUXI WLAN: W23 stats pdev 2 after 2 ms (stats-id 0x4 pdevs 1 vdevs 2 bcns 1 bytes 516 events 1 unusable 0; ce tx 1 rx 1)\n",
		"FUXI WLAN: W23 stats after-scan wmi mgmt-rx +0 frames +0 bss +0 scan-events +0 chan-info +0 other +0\n",
		"FUXI WLAN: W23 complete; stop/contain\n",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	contained_and_freed();
	/* every pdev word at its widest (0x80000000: -2147483648 signed): each line still fits the console's 254 bytes
	 * (model_line fails any longer FUXI WLAN line) */
	c2_reset(); C.stats_mode = 8;
	CHECK(run() == 0, "C2b widest pdev fields");
	expect("FUXI WLAN: W23 stats rx2 loc-msdus -2147483648 loc-mpdus -2147483648 oversize-amsdu -2147483648 phy-errs "
	       "-2147483648 phy-err-drop -2147483648 mpdu-errs -2147483648 mid-ppdu-route-change -2147483648\n");
	expect("FUXI WLAN: W23 stats tx2 hw-reaped -2147483648 underrun -2147483648 tx-abort -2147483648 mpdus-requed -2147483648 "
	       "tx-ko 2147483648 self-triggers 2147483648 pdev-resets 2147483648\n");
	contained_and_freed();
	/* the pdevs: the preferred hw mode's (DBS) PHYs in PHY order, L19's fw_pdev[0] and fw_pdev[1], not the first MAC/PHY
	 * entry; with the DBS listed first too; one id for both PHYs: one request, both bands */
	c2_reset(); C.dbs_pdev = 2;
	CHECK(run() == 0, "C2b DBS pdevs 2 and 3");
	expect("FUXI WLAN: W21 wmi service ready ext hw-modes 2 caps 2 pdev 1\n");
	static const char *const p23[] = {
		"FUXI WLAN: W23 stats request pdev-stat pdev 2 bands 0x1 vdev 0 sent\n",
		"FUXI WLAN: W23 stats pdev 2 after 2 ms ",
		"FUXI WLAN: W23 stats request pdev-stat pdev 3 bands 0x2 vdev 0 sent\n",
		"FUXI WLAN: W23 stats pdev 3 after 2 ms ",
	};
	expect_order(p23, sizeof(p23) / sizeof(p23[0]));
	CHECK(fw_stats_req == 2, "two pdev requests");
	contained_and_freed();
	c2_reset(); C.dbs_pdev = 2; C.ext_single_last = true;
	CHECK(run() == 0, "C2b DBS pdevs 2 and 3, DBS listed first");
	expect("FUXI WLAN: W21 wmi service ready ext hw-modes 2 caps 2 pdev 2\n");
	expect_order(p23, sizeof(p23) / sizeof(p23[0]));
	contained_and_freed();
	c2_reset(); C.dbs_same_pdev = true;
	CHECK(run() == 0, "C2b one pdev for both DBS PHYs");
	expect("FUXI WLAN: W23 stats request pdev-stat pdev 1 bands 0x3 vdev 0 sent\n");
	CHECK(fw_stats_req == 1 && count_text("W23 stats request pdev-stat") == 1, "one request for one pdev id");
	contained_and_freed();
	/* SINGLE alone: no PHY of the preferred mode, L19's zeroed fw_pdev[0]: pdev 0, no bands */
	c2_reset(); C.ext_single_only = true; C.phy_bands = 0;
	CHECK(run() == 0, "C2b SINGLE only");
	expect("FUXI WLAN: W21 wmi service ready ext hw-modes 1 caps 1 pdev 1\n");
	expect("FUXI WLAN: W23 stats request pdev-stat pdev 0 bands 0 vdev 0 sent\n");
	CHECK(fw_stats_req == 1, "one request with no preferred-mode PHY");
	contained_and_freed();
	/* an unusable event (fixed TLV of 40 bytes), then a usable one: the wait goes on until the usable one */
	c2_reset(); C.stats_mode = 6;
	CHECK(run() == 0, "C2b unusable, then usable");
	expect("FUXI WLAN: W23 stats pdev 1 after 5 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 2 unusable 1; ce tx 1 rx 2)\n");
	expect("FUXI WLAN: W23 stats pdev 2 after 5 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 2 unusable 1; ce tx 1 rx 2)\n");
	contained_and_freed();
	/* a credit drought that ends 200 ms after COMPLETED: the bounded credit wait, then the requests */
	c2_reset(); C.credits = 1; C.refund_later = true; C.no_start_refund = true; C.start_refund_at = 5906;
	CHECK(run() == 0, "C2b late credits");
	static const char *const late[] = {
		"FUXI WLAN: W24 scan detail ",
		"FUXI WLAN: W23 stats request waits for 1 HTC credits (has 0)\n",
		"FUXI WLAN: W23 stats request pdev-stat pdev 1 bands 0x1 vdev 0 sent\n",
		"FUXI WLAN: W23 stats pdev 1 after 2 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 1 unusable 0; ce tx 1 rx 2)\n",
		"FUXI WLAN: W23 stats request pdev-stat pdev 2 bands 0x2 vdev 0 sent\n",
		"FUXI WLAN: W23 stats after-scan wmi ",
		"FUXI WLAN: W23 complete; stop/contain\n",
	};
	expect_order(late, sizeof(late) / sizeof(late[0]));
	contained_and_freed();
	/* a beacon that arrives in the first exchange: parsed into the table as C2 parses it, and reported (late, not
	 * filtered); the W24 lines are not printed again */
	c2_reset(); C.on_line = 2;
	CHECK(run() == 0, "C2b late MGMT_RX");
	static const char *const lmgmt[] = {
		"FUXI WLAN: W24 scan summary frames 0 bss 0 ssids 0\n",
		"FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung W24b = w20 0x02\n",
		"FUXI WLAN: W23 stats request pdev-stat pdev 1 bands 0x1 vdev 0 sent\n",
		"FUXI WLAN: W23 stats pdev 1 after 2 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 1 unusable 0; ce tx 1 rx 2)\n",
		"FUXI WLAN: W23 stats after-scan wmi mgmt-rx +1 frames +1 bss +1 scan-events +0 chan-info +0 other +0\n",
		"FUXI WLAN: W23 complete; stop/contain\n",
	};
	expect_order(lmgmt, sizeof(lmgmt) / sizeof(lmgmt[0]));
	CHECK(wl_scan.mgmt == 1 && wl_scan.frames == 1 && wl_scan.nbss == 1 && count_text("FUXI WLAN: W24 bss ") == 0, "late beacon");
	contained_and_freed();
	puts("C2b stats: L19's vdev/beacon/pdev order with every field at its offset and the counts and bytes printed, the "
	     "widest fields within 254 bytes, one request per pdev id of the preferred mode's PHYs in PHY order (DBS listed "
	     "second and first; one id for both PHYs; SINGLE only: pdev 0), an unusable event before a usable one, credits "
	     "back 200 ms after COMPLETED, a beacon inside the exchange reported as after-scan");
}

static void
s_c2b_stats_failures(void)
{
	/* no answer: exactly 500 ms per pdev, one line each, then containment; the run still passes */
	c2_reset(); C.stats_mode = 1;
	CHECK(run() == 0, "C2b: no UPDATE_STATS failed the run");
	CHECK(now - fw_stats_at == 500 && fw_stats_req == 2, "the UPDATE_STATS wait is not 500 ms");
	expect("FUXI WLAN: W23 stats pdev 1 no usable UPDATE_STATS in 500 ms (events 0 unusable 0; ce tx 1 rx 1)\n"
	       "FUXI WLAN: W23 stats request pdev-stat pdev 2 bands 0x2 vdev 0 sent\n"
	       "FUXI WLAN: W23 stats pdev 2 no usable UPDATE_STATS in 500 ms (events 0 unusable 0; ce tx 1 rx 1)\n"
	       "FUXI WLAN: W23 stats after-scan wmi mgmt-rx +0 frames +0 bss +0 scan-events +0 chan-info +0 other +0\n"
	       "FUXI WLAN: W23 complete; stop/contain\n");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: scan over, no STOP_SCAN\n");
	/* malformed, short, refused (empty), array first: each logged with its reason, then containment */
	static const struct { unsigned mode; const char *why; } bad[] = {
		{2, "stats TLV past the event"}, {3, "stats byte array shorter than its entries"}, {4, "no pdev stats in the event"},
		{7, "stats byte array before the fixed TLV"},
	};
	for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		char want[256];
		c2_reset(); C.stats_mode = bad[i].mode;
		CHECK(run() == 0, "C2b: an unusable UPDATE_STATS failed the run");
		CHECK(now - fw_stats_at == 500, "the UPDATE_STATS wait is not 500 ms");
		for (unsigned k = 1; k <= 2; k++) {
			snprintf(want, sizeof(want), "FUXI WLAN: W23 stats pdev %u no usable UPDATE_STATS in 500 ms (events 1 unusable 1, "
			         "last: %s; ce tx 1 rx 1)\n", k, bad[i].why);
			expect(want);
		}
		expect_not("W23 failure");
		expect("FUXI WLAN: W23 complete; stop/contain\n");
		contained_and_freed();
	}
	/* credit starvation: the bounded credit wait (3000 ms), no request, no second pdev, the run passes */
	c2_reset(); C.credits = 1; C.refund_later = true; C.no_start_refund = true;
	CHECK(run() == 0, "C2b: no credits failed the run");
	expect("FUXI WLAN: W23 stats request waits for 1 HTC credits (has 0)\nFUXI WLAN: W23 stats request pdev-stat pdev 1 bands "
	       "0x1 not sent: no HTC credits after 3000 ms (ce tx 0 rx 0); continue to containment\nFUXI WLAN: W23 stats after-scan "
	       "wmi mgmt-rx +0 frames +0 bss +0 scan-events +0 chan-info +0 other +0\nFUXI WLAN: W23 complete; stop/contain\n");
	CHECK(fw_stats_req == 0, "REQUEST_STATS without credits");
	contained_and_freed();
	/* the request's descriptor never completes: the TX drain gives up at 1000 ms, no wait for the answer, no second pdev */
	c2_reset(); C.stats_tx_stuck = true; C.stats_mode = 1;
	CHECK(run() == 0, "C2b: a stuck stats TX failed the run");
	expect("FUXI WLAN: W23 stats request TX not drained after 1000 ms (ce tx 1 rx 1); continue to containment\n"
	       "FUXI WLAN: W23 stats after-scan wmi mgmt-rx +0 frames +0 bss +0 scan-events +0 chan-info +0 other +0\n"
	       "FUXI WLAN: W23 complete; stop/contain\n");
	CHECK(fw_stats_req == 1 && !wlce_tx_idle(&wl_ce.pipe[3]), "stuck stats drain");
	contained_and_freed();
	/* the same, but the firmware answered meanwhile: the answer is printed (already parsed and bounded) */
	c2_reset(); C.stats_tx_stuck = true;
	CHECK(run() == 0, "C2b: a stuck stats TX with an answer failed the run");
	expect("FUXI WLAN: W23 stats request TX not drained after 1000 ms (ce tx 1 rx 1); continue to containment\n"
	       "FUXI WLAN: W23 stats pdev 1 after 1000 ms (stats-id 0x4 pdevs 1 vdevs 0 bcns 0 bytes 176 events 1 unusable 0; "
	       "ce tx 1 rx 1)\nFUXI WLAN: W23 stats base chan-nf -98 ");
	expect("FUXI WLAN: W23 stats tx2 hw-reaped 0 underrun 0 tx-abort 0 mpdus-requed 0 tx-ko 0 self-triggers 0 pdev-resets 0\n"
	       "FUXI WLAN: W23 stats after-scan wmi ");
	CHECK(fw_stats_req == 1 && wl_stats.have, "stuck stats drain with an answer");
	contained_and_freed();
	/* the transport fails during the exchange (the W11 guard): that ends the run as in every W20..W23 wait */
	c2_reset(); C.guard_on_stats = true;
	CHECK(run() == 1, "C2b: a guard failure in the stats wait accepted");
	expect("FUXI WLAN: W23 stats request pdev-stat pdev 1 bands 0x1 vdev 0 sent\n");
	expect("FUXI WLAN: W23 failure: AMSS/M0, ownership, discovery or access/SMMU fault; contain, no retry\n");
	expect_not("W23 complete");
	expect_not("W23 stats after-scan");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: scan over, no STOP_SCAN\n");
	/* a CE-layer integrity fault in the send (CE3's device TP one entry back): fatal as in wl_ce_send, with the
	 * snapshot; nothing sent */
	c2_reset(); C.on_line = 1;
	CHECK(run() == 1, "C2b: a CE fault in the stats send accepted");
	static const char *const ce[] = {
		"FUXI WLAN: W24 scan detail ",
		"FUXI WLAN: W23 WMI REQUEST_STATS FAILED: CE device TP passed the host HP or went backwards\n",
		"FUXI WLAN: W20 snapshot WMI REQUEST_STATS: ",
		"FUXI WLAN: W23 failure: CE device TP passed the host HP or went backwards; contain, no retry\n",
	};
	expect_order(ce, sizeof(ce) / sizeof(ce[0]));
	expect_not("W23 stats request pdev-stat");
	expect_not("W23 complete");
	CHECK(fw_stats_req == 0 && wl_w11_error && wl_ce.errors, "CE fault in the stats send");
	contained_and_freed();
	puts("C2b stats failures: no answer (500 ms per pdev), TLV past the event, short entry, refused (no pdev entry), array "
	     "before the fixed TLV, credit starvation (3000 ms, nothing sent), a stuck TX descriptor (1000 ms; with an answer "
	     "it is printed): all logged and the run passes; a W11 guard failure in the exchange and a CE fault in the send "
	     "end the run; every run contained and freed");
}

static void
s_c2b_cap(void)
{
	/* 4 START_RESP per channel and one with flags 2: 191 events, 128 lines, the rest counted before W23 complete */
	c2_reset(); C.ci_repeat = 3;
	CHECK(run() == 0, "C2b 191 CHAN_INFO");
	CHECK(count_text("FUXI WLAN: W23 chan-info freq ") == 128 && wl_scan.ci_seen == 191, "128 CHAN_INFO lines");
	expect("FUXI WLAN: W23 chan-info freq 2412 flags 0x2 nf -96 rx-clear 0 cycle 0 rx-frame 0 tx-frame 0 my-bss-rx 0 mac-clk 0\n");
	expect("FUXI WLAN: W23 scan completed reason 0 after 5704 ms (foreign 38 chan-info 152)\n");
	static const char *const lines[] = {
		"FUXI WLAN: W23 stats tx2 ",
		"FUXI WLAN: W23 stats after-scan wmi ",
		"FUXI WLAN: W23 chan-info lines suppressed 63 (cap 128)\n",
		"FUXI WLAN: W23 complete; stop/contain\n",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	contained_and_freed();
	/* the same with no COMPLETED: the count before the snapshot, no stats request on the failure path */
	c2_reset(); C.ci_repeat = 3; C.scan_mode = 3;
	CHECK(run() == 1, "C2b no COMPLETED accepted");
	static const char *const timeout[] = {
		"FUXI WLAN: W23 timeout scan COMPLETED (8750 ms); report, contain, no retry\n",
		"FUXI WLAN: W24 scan summary ",
		"FUXI WLAN: W23 chan-info lines suppressed 63 (cap 128)\n",
		"FUXI WLAN: W20 snapshot scan COMPLETED: ",
		"FUXI WLAN: W23 failure: scan COMPLETED; contain, no retry\n",
	};
	expect_order(timeout, sizeof(timeout) / sizeof(timeout[0]));
	CHECK(fw_stats_req == 0, "REQUEST_STATS after a scan timeout");
	expect_not("W23 stats");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: STOP_SCAN sent (best effort)");
	/* a failed poll in the scan wait (the guard, after the last FOREIGN_CHAN): the count still printed, after the
	 * failure; and the same in the stats wait */
	c2_reset(); C.ci_repeat = 3; C.on_line = 3;
	CHECK(run() == 1, "C2b guard failure in the scan wait accepted");
	static const char *const pollfail[] = {
		"FUXI WLAN: W23 scan foreign freq 5825\n",
		"FUXI WLAN: W23 failure: AMSS/M0, ownership, discovery or access/SMMU fault; contain, no retry\n",
		"FUXI WLAN: W23 chan-info lines suppressed 58 (cap 128)\n",
	};
	expect_order(pollfail, sizeof(pollfail) / sizeof(pollfail[0]));
	CHECK(wl_scan.ci_seen == 186 && fw_stats_req == 0, "186 CHAN_INFO before the guard failure");
	contained_and_freed();
	c2_reset(); C.ci_repeat = 3; C.guard_on_stats = true;
	CHECK(run() == 1, "C2b guard failure in the stats wait accepted");
	static const char *const statsfail[] = {
		"FUXI WLAN: W23 stats request pdev-stat pdev 1 bands 0x1 vdev 0 sent\n",
		"FUXI WLAN: W23 failure: AMSS/M0, ownership, discovery or access/SMMU fault; contain, no retry\n",
		"FUXI WLAN: W23 chan-info lines suppressed 63 (cap 128)\n",
	};
	expect_order(statsfail, sizeof(statsfail) / sizeof(statsfail[0]));
	contained_and_freed();
	puts("C2b CHAN_INFO cap: 191 events (one with flags 0x2), 128 lines, 63 counted before W23 complete or before the "
	     "COMPLETED-timeout snapshot (no stats request there), and after a failed poll in the scan wait (58) and in the "
	     "stats wait (63)");
}

static void
dump_c2(void)
{
	uint8_t out[2048];
	uint32_t len = 0;
	const uint32_t vp[3] = {0, WLWMI_VDEV_PARAM_NSS, 2};
	const uint32_t stop[5] = {0, WLWMI_SCAN_ID, WLWMI_SCAN_STOP_ONE, WLWMI_VDEV_ID, 0};
	CHECK(wlwmi_encode_chan_list(0, 38, false, 0, out, sizeof(out), &len) == 0, "dump chan list");
	for (uint32_t i = 0; i < len / 4; i++) { fprintf(stdout, "chanlist %u %u\n", i, g32(out + 4 * i)); }
	CHECK(wlwmi_encode_vdev_create((const uint8_t *)"\x02\x46\x55\x58\x49\x01", 0, 3, 2, 2, out, sizeof(out), &len) == 0, "dump vdev");
	for (uint32_t i = 0; i < len / 4; i++) { fprintf(stdout, "vdev %u %u\n", i, g32(out + 4 * i)); }
	CHECK(wlwmi_encode_start_scan(38, wlscan_max_time(38), out, sizeof(out), &len) == 0, "dump scan");
	for (uint32_t i = 0; i < len / 4; i++) { fprintf(stdout, "scan %u %u\n", i, g32(out + 4 * i)); }
	CHECK(wlwmi_encode_words(WLWMI_CMD_STOP_SCAN, WLWMI_TAG_STOP_SCAN_CMD, stop, 5, out, sizeof(out), &len) == 0, "dump stop");
	for (uint32_t i = 0; i < len / 4; i++) { fprintf(stdout, "stop %u %u\n", i, g32(out + 4 * i)); }
	CHECK(wlwmi_encode_words(WLWMI_CMD_VDEV_SET_PARAM, WLWMI_TAG_VDEV_SET_PARAM_CMD, vp, 3, out, sizeof(out), &len) == 0, "dump nss");
	for (uint32_t i = 0; i < len / 4; i++) { fprintf(stdout, "nss %u %u\n", i, g32(out + 4 * i)); }
	wlhtt_version_req(out);
	fprintf(stdout, "htt 0 %u\nhtt 1 %u\n", g32(out), g32(out + 4));
	CHECK(wlwmi_encode_stats_req(1, out, sizeof(out), &len) == 0, "dump stats");  /* C2b, pdev 1 */
	for (uint32_t i = 0; i < len / 4; i++) { fprintf(stdout, "stats %u %u\n", i, g32(out + 4 * i)); }
	fflush(stdout);
}

static void
dump_init(void)
{
	uint8_t out[WLWMI_INIT_TOTAL];
	uint32_t len = 0;
	struct wlwmi w;
	memset(&w, 0, sizeof(w));
	w.num_rf_chains = 2; w.svc_ext2[0] = 1u << 25;
	CHECK(wlwmi_encode_init(&w, out, sizeof(out), &len) == 0 && len == 332, "encode");
	for (unsigned i = 0; i < 73; i++) { fprintf(stdout, "%u %u\n", i, g32(out + 36 + 4 * i)); }
}

static void
s_no_w20(void)
{
	reset();
	wl_ce_contain(17);
	CHECK(loglen == 0, "containment hook printed without W20");
	wl5_abort(17);
	expect_not("W20");
	CHECK(arena_allocs == 0 && ce_writes == 0 && win_selects == 0, "W20 hardware access without W20");
	puts("W20 never ran: the containment hook is silent and touches nothing");
}

int
main(int argc, char **argv)
{
	if (argc == 2 && !strcmp(argv[1], "dump-init")) { dump_init(); return 0; }
	if (argc == 2 && !strcmp(argv[1], "dump-c2")) { dump_c2(); return 0; }
	s_pure();
	s_no_w20();
	s_happy_shadow();
	s_happy_direct();
	s_bits();
	s_credits();
	s_ready_missing();
	s_faults();
	s_tolerated();
	s_order();
	s_window_and_fault();
	s_latch();
	s_wrap();
	s_scan_pure();
	s_scan_happy();
	s_scan_bits();
	s_scan_split_and_caps();
	s_scan_full();
	s_scan_failures();
	s_scan_quiet();
	s_c2b_pure();
	s_c2b_stats();
	s_c2b_stats_failures();
	s_c2b_cap();
	reset();
	puts("W20 model: all scenarios passed");
	return 0;
}
