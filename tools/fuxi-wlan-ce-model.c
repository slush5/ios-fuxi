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
 *   C3 (fuxiwlanqrtr=8): the W25 service on a simulated clock (wl_sleep
 *   moves it; mach_absolute_time reads it) behind a fake iOS side that queues
 *   scan requests and records ready(), frames and done() statuses; the
 *   firmware checks every START_SCAN (fresh id, PASSIVE only, the request's
 *   listed channels and dwell, max time) and STOP_SCAN (the scan's id), plays
 *   per-scan scripts (START_FAILED, no STARTED, no COMPLETED, DEQUEUED, a
 *   stale event of the previous id), drops what a stopped scan would still
 *   have sent, can starve the host of credits, and fails the run when a due
 *   RX message finds no posted buffer for 1 s. Built with -DFUXI_WLAN_E2E,
 *   the fake is left out and the real fuxi_wifi_radio.c (linked from
 *   tools/test-fuxi-wlan-ce.py's iOS-side unit) is the other end.
 * No device, no network. L19 = Linux ath12k v6.19 (BSD-3-Clause-Clear).
 */
#include <assert.h>
#include <errno.h>
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
/* C2c: every device-written word of the RX arena is a load the read barrier must follow; the one host pointer
 * publish into it (wrp[0]) is checked when it happens (model_rx_publish) */
#define WLRX_LOAD(p) (model_event('L'), *(p))
static void model_rx_publish(volatile uint32_t *p, uint32_t v);
#define WLRX_PUBLISH(p, v) model_rx_publish((p), (v))
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
static struct wl_dma wl_rx_arena, wl_link_arena;  /* C2c */
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
/* C3: the service's clock is the model's (milliseconds) */
static uint64_t now;
static uint64_t mach_absolute_time(void) { return now * 1000000ull; }
static void absolutetime_to_nanoseconds(uint64_t t, uint64_t *ns) { *ns = t; }

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
static unsigned sleeps_1, sleeps_10;        /* C3: wl_sleep(1) and wl_sleep(10) calls */
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
	/* C2c */
	unsigned rx_rule;                   /* MGMT_RX: 0 always (C2), 1 needs links, 2 buffers and links, 3 buffers, 4 chain masks, 5 never */
	uint32_t wbm_pre[9];                /* what the nine WBM pre-reads return (D1) */
	unsigned wbm_rb_bad;                /* 1 BASE_LSB, 2 MISC reads back wrong */
	unsigned wbm_fault_after;           /* fw_faults latched after that many ring 120 writes */
	bool r2_unreadable;                 /* R2 HP/TP read ~0u at the watches */
	bool rx_alloc_fail, link_map_fail;  /* the RX arena allocation / the link arena map fails */
	bool late_write;                    /* RXDMA writes a posted buffer during containment, before bus master off */
	bool links_used;                    /* each received frame takes a link descriptor (with 0x200) */
	unsigned links_returned;            /* link descriptors WBM puts back on ring 120 (writes the ring, moves R2 HP) */
	uint32_t rx_take, rx_post[2];       /* refill entries the firmware takes at setup, and posts to each MAC ring */
	unsigned rx_frames;                 /* frames RXDMA receives per channel (mac0 on 2.4 GHz, mac1 on 5 GHz) */
	unsigned rx_frames_ch0;             /* extra frames on channel 0 (mac0) */
	uint32_t mac_rbm, mac_cookie_xor;   /* MAC ring entries the firmware writes: RBM override (0 = as taken), cookie XOR */
	unsigned mac_foreign;               /* MAC0 foreign entries, posted last: in turn 64 bytes into a slot, the start of
	                                     * never-posted slot 4095, a posted slot's start with address bits 39:32 = 1 */
	unsigned guard_on_sring;            /* the W11 guard fails once the firmware took that SRING_SETUP (1..4) */
	uint32_t ci_tx;                     /* CHAN_INFO START tx-frame (D4) */
	uint32_t stats_tx_frame;            /* the pdev stats base tx-frame (D4), every other tx counter 0 */
	uint32_t phy_rx_mask;               /* the MAC/PHY caps' rx chain mask, ANDed with the tx one (0 = the same) */
	unsigned setup_delay;               /* the firmware takes and posts refill entries this many ms after the 4th SRING_SETUP */
	unsigned ce4_inflight;              /* at the HTT version line CE4 shows 16 sends in flight */
	unsigned spill;                     /* bit 0 a slot's upper half, bit 1 slot 4095, bit 2 the tail's start, bit 3 the
	                                     * tail's end, bit 4 the gaps (rdp[170], 0x1c040) */
	bool write32;                       /* frames write bytes 32..35 only */
	unsigned err_n;                     /* RXDMA error entries pushed during the scan, code err_code, RBM err_rbm */
	uint32_t err_code, err_rbm;
	uint32_t rdp_err_raw;               /* rdp[186] written raw (bad) at channel 5 */
	uint32_t refill_tp_raw[2];          /* rdp[160] rewritten raw at channel 3 and channel 30 (backwards / past hp) */
	unsigned take_more;                 /* refill entries taken during the scan and kept (kept rises) */
	/* C3: per START_SCAN number (1..16, else 0) 0 normal, 1 START_FAILED, 2 no STARTED, 3 no COMPLETED,
	 * 4 DEQUEUED after STARTED */
	unsigned c3_mode[17];
	bool c3_stop_silent;                /* STOP_SCAN never answered */
	bool c3_stale;                      /* from the 2nd scan on, a COMPLETED of the previous id comes first */
	unsigned c3_starve_after;           /* no credit refund for START_SCAN number >= this (0: always) */
	unsigned c3_guard_after;            /* the W11 guard fails once START_SCAN number this was taken (0: never) */
	/* C3 fix */
	uint32_t c3_echo_id;                /* every SCAN_EVENT carries this id, not the START_SCAN's (0: the START_SCAN's) */
	uint32_t c3_stop_late_ms;           /* STOP_SCAN answered this late (0: at 2 ms) */
	unsigned c3_fail_at;                /* this START_SCAN number (past 16) gets START_FAILED (0: none) */
	unsigned c3_hold;                   /* the first N START_SCAN credits are never refunded */
	bool c3_refund_at_end;              /* a START_SCAN credit comes back only after its scan's COMPLETED */
};
static struct cfg C;
static unsigned restores;
static bool restore_lie, sts_glitched;

/* firmware; C3: a message carries the scan id it belongs to (0: none) and is dropped once that scan was stopped */
struct qmsg { unsigned ce; uint64_t due; uint32_t len; int stslen; uint32_t tag; uint8_t d[WLCE_BUF]; };
static struct qmsg q[512];
static unsigned q_head, q_tail;
static uint32_t fw_tag, fw_dead_tag;
static unsigned fw_dropped;
static uint64_t starved_since;
/* C3 firmware view of the service */
static unsigned fw_c3_scans, fw_c3_mode_now;
static uint32_t fw_c3_id, fw_c3_n, fw_c3_dwell, fw_c3_max, fw_c3_flags, fw_c3_chan[64], fw_c3_stop_id;
static uint64_t fw_c3_start_at, fw_c3_stop_at;
static uint32_t fw_c3_stop_after[17];   /* ms from a START_SCAN (by number) to its STOP_SCAN */
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
static uint32_t fw_cmd_log[64];
static unsigned fw_cmd_n;
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
static void dev_acts(void);
static void wbm_access(uint32_t off, bool wr, uint32_t *v, bool shadow);

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
	dev_acts();  /* C2c: RXDMA, WBM and the firmware's ring use, in time order */
	while (q_head != q_tail) {
		struct qmsg *m = &q[q_head % 512];
		if (m->tag && m->tag == fw_dead_tag) { q_head++; fw_dropped++; continue; }  /* C3: a stopped scan's rest */
		if (m->due > now || !deliver(m)) { break; }
		q_head++;
	}
	/* C3: a due message that finds no posted RX buffer for 1 s: the host stopped posting buffers again */
	if (wl_w25 && q_head != q_tail && q[q_head % 512].due <= now && !wl_ce.stopping) {
		const struct dring *d = &dr[q[q_head % 512].ce == 1 ? 1 : 3];
		if (d->enabled && d->dev == d->host) {
			if (!starved_since) { starved_since = now; }
			CHECK(now - starved_since < 1000, "RX ring starved: a due message found no posted RX buffer for 1000 ms");
		} else {
			starved_since = 0;
		}
	}
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
	m->ce = ce; m->due = now + delay; m->len = len; m->stslen = stslen; m->tag = fw_tag;
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
	const uint32_t rxm = C.phy_rx_mask ? C.phy_rx_mask & mask : mask;  /* C2c: the rx mask apart from the tx one */
	m_tlv(0x210, 108); m_u32(hw_mode); m_u32(hw_mode ? C.dbs_pdev + (C.dbs_same_pdev ? 0u : phy) : 1u); m_u32(phy); m_u32(0);
	m_u32(bands); m_zero(7 * 4);
	m_u32(mask); m_u32(rxm); m_zero(6 * 4); m_u32(mask); m_u32(rxm); m_zero(5 * 4);
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

/* ---- C2c device: the RX and link arenas, WBM ring 120, RXDMA and the firmware's use of the rings ----
 * Written down here again from L19 and rx_path_plan.md 2.5-2.7 (not taken from the header): the arena layout and
 * the device's permission per region (a device read of an unposted half or slot, a write to a host-only word, or
 * any access outside the arenas fails the test, except the injected spills), the four SRING_SETUP words, the WBM
 * pre-read and write order, the idle and refill entries the host must have written before each publish. */
#define RXA 0x1040000u
#define LNA 0x440000u
#define M_IDLE_R0 0x00a34d3cu
#define M_IDLE_HP 0x00a370b8u
static const uint32_t m_pre_addr[9] = { M_IDLE_R0, M_IDLE_R0 + 4u, M_IDLE_R0 + 0x10u, M_IDLE_HP, M_IDLE_HP + 4u, 0x00a34240u,
	0x00a34244u, 0x00a34250u, 0x00a34254u };
static const uint32_t m_wbm_waddr[8] = { M_IDLE_R0, M_IDLE_R0 + 4u, M_IDLE_R0 + 8u, M_IDLE_R0 + 0x30u, M_IDLE_R0 + 0x34u,
	M_IDLE_HP, M_IDLE_HP + 4u, M_IDLE_R0 + 0x10u };
static const uint32_t m_mac_off[2] = { 0xc000u, 0x10000u }, m_mac_id[2] = { 161u, 169u };
static uint8_t *rx_host, *ln_host;
static uint64_t rx_iova, ln_iova;
static bool rx_mapped, ln_mapped, rx_pub, smmu_undone, fw_rx_ready, idle_live, idle_pub, rx_inject;
static unsigned rx_allocs, rx_frees, ln_allocs, ln_frees, wbm_reads, wbm_writes, wbm_rbs, wbm_watch_reads, wbm_pubs;
static unsigned fw_sring_n, fw_pset_n, fw_frames_written, fw_nobuf, fw_links, fw_taken_n, fw_posted_n, fw_htt_conf_q;
static uint32_t m_idle_hp, m_idle_tp, fw_refill_tp, fw_mac_hp[2], fw_mac_tp[2], fw_err_hp, fw_pset_w[2][4];
static uint32_t fw_taken[4096][2];
static uint8_t slot_listed[4096];
static uint32_t wbm_log[64][3];
static unsigned wbm_log_n;
struct dact { uint64_t due; unsigned kind; uint32_t a, b; };
static struct dact acts[256];
static unsigned n_acts, acts_done;
enum { A_FRAMES, A_ERR, A_SPILL, A_RDP, A_TAKE, A_SETUP };
/* What RXDMA writes from a received frame (into buffers, link descriptors, error-entry frame fields): no log line may
 * show any of it (the owner rule: counts only). The error entry's frame fields (L19 hal_desc.h:1033, 1039, 1044):
 * MPDU byte count 0x2bad in w5 21:8, fragment 0xb in w6 10:7, sequence 0x5ee in w6 30:19. */
#define M_FRAME_WORD 0x5eedf00du
#define M_ERR_W5 (0x00c00000u | 0x2badu << 8)            /* DEST_IND 3 | the byte count */
#define M_ERR_W6_FRAME (0xbu << 7 | 0x5eeu << 19)
#define M_ERR_W6_KEEP (1u << 13)                          /* SW_EXCEPTION_RING_VLD: not from the frame */

static void act(uint64_t due, unsigned kind, uint32_t a, uint32_t b) { CHECK(n_acts < 256, "model actions"); acts[n_acts++] = (struct dact){ due, kind, a, b }; }

/* A device access to the RX arena by offset, checked against what the device may do there. */
static uint8_t *
rxd(uint32_t off, uint32_t len, bool wr, const char *what)
{
	bool ok = false;
	CHECK(rx_host && rx_mapped && (epcmd & 4u), "RX arena DMA unmapped or with bus mastering off");
	CHECK(len && off + len <= RXA, what);
	if (off < 0x400u) {
		ok = wr && len == 4 && (off == 4u * 160u || off == 4u * 161u || off == 4u * 169u || off == 4u * 186u);
	} else if (off < 0x400u + 4u * 47u) {
		uint32_t i = (off - 0x400u) / 4u;
		ok = len == 4 && !(off & 3u) && (wr ? i == 1 || i == 9 : i == 0 || i == 1 || i == 9 || i == 26);
	} else if (off >= 0x4000u && off + len <= 0xc000u) {
		ok = !wr;
	} else if (off >= 0xc000u && off + len <= 0x14000u) {
		ok = true;
	} else if (off >= 0x14000u && off + len <= 0x1c000u) {
		ok = wr;
	} else if (off >= 0x20000u && off < 0x1020000u) {
		uint32_t slot = (off - 0x20000u) / 0x1000u, o = (off - 0x20000u) % 0x1000u;
		ok = wr && slot < 4095u && o + len <= 2048u;
	}
	CHECK(ok || rx_inject, what);
	return rx_host + off;
}

/* A device access to the link arena: ring 120 read/write, listed descriptors read/write, slot 32767 never. */
static uint8_t *
lnd(uint32_t off, uint32_t len, const char *what)
{
	CHECK(ln_host && ln_mapped && (epcmd & 4u) && idle_pub, "link arena DMA unmapped, unpublished or with bus mastering off");
	CHECK(len && (off + len <= 0x40000u || (off >= 0x40000u && off + len <= LNA && (off - 0x40000u) / 128u < 32767u &&
	    (off - 0x40000u) / 128u == (off + len - 1u - 0x40000u) / 128u)), what);
	return ln_host + off;
}

/* The host's one pointer publish into the RX arena: wrp[0] = 8190 right after the write barrier, with every refill
 * entry, sentinel and zero word in place, and before any SRING_SETUP. */
static void
model_rx_publish(volatile uint32_t *p, uint32_t v)
{
	CHECK(rx_host && (uintptr_t)p == (uintptr_t)(rx_host + 0x400u), "a host pointer publish into the RX arena other than wrp[0]");
	CHECK(last_ev == 'W', "wrp[0] published without the write barrier right before it");
	CHECK(!rx_pub && !fw_sring_n && !wl_rx.stopping, "wrp[0] published twice, after an SRING_SETUP or after the containment latch");
	CHECK(v == 8190u, "wrp[0] is not 4095 entries in words");
	for (uint32_t o = 0; o < 0x20000u; o++) {
		if (o >= 0x4000u && o < 0x4000u + 8u * 4095u) { continue; }
		CHECK(!rx_host[o], "rdp, wrp, a ring or a gap not zero at the wrp[0] publish");
	}
	for (uint32_t k = 0; k < 4096u; k++) {
		const uint8_t *s = rx_host + 0x20000u + 0x1000u * k;
		uint64_t a = rx_iova + 0x20000u + 0x1000ull * k;
		if (k < 4095u) {
			const uint8_t *e = rx_host + 0x4000u + 8u * k;
			CHECK(g32(e) == (uint32_t)a && g32(e + 4) == (((uint32_t)(a >> 32) & 0xffu) | 6u << 8 | k << 12),
			    "refill entry not in place at the wrp[0] publish");
		}
		for (unsigned j = 0; j < 4096u; j++) { CHECK(s[j] == (k < 4095u && j < 64u ? 0xa5u : 0u), "sentinel or zero not in place at the wrp[0] publish"); }
	}
	for (uint32_t o = 0x1020000u; o < RXA; o++) { CHECK(!rx_host[o], "tail not zero at the wrp[0] publish"); }
	*p = v;
	rx_pub = true;
}

static bool
m_refused(void)  /* the pre-reads (D1): ~0u anywhere, SRNG_ENABLE, a ring base, scatter mode: no WBM write */
{
	for (unsigned i = 0; i < 9; i++) { if (C.wbm_pre[i] == ~0u) { return true; } }
	return (C.wbm_pre[2] & 0x40u) || C.wbm_pre[0] || (C.wbm_pre[1] & 0xffu) || (C.wbm_pre[5] & 2u);
}

/* WBM ring 120: the nine pre-reads in order, then (unless they refuse) the eight writes in L19's order with L19's
 * values, BASE_LSB and MISC read back, one HP publish of 65534 (shadow 19, or R2 with fuxiwlanw15=1) right after the
 * write barrier with all 32767 entries in place, then R2 HP/TP reads at the watches; nothing after wl26_contain. */
static void
wbm_access(uint32_t off, bool wr, uint32_t *v, bool shadow)
{
	CHECK(!wl_rx.stopping, "a WBM access after wl26_contain");
	CHECK(wl20_bits & WL20_IDLELINK, "a WBM access without w20 0x200");
	if (wbm_log_n < 64) { wbm_log[wbm_log_n][0] = off; wbm_log[wbm_log_n][1] = wr ? *v : 0u; wbm_log[wbm_log_n][2] = wr | (shadow ? 2u : 0u); wbm_log_n++; }
	if (!wr) {
		CHECK(!shadow, "model: shadow read");
		if (!wbm_writes && wbm_reads < 9u) {
			CHECK(off == m_pre_addr[wbm_reads], "a WBM read before or outside the nine pre-reads, or out of their order");
			*v = C.wbm_pre[wbm_reads++];
			return;
		}
		CHECK(wbm_writes == 8u, "a WBM read after a refusal or in the middle of the ring 120 writes");
		if (wbm_rbs < 2u) {
			CHECK(off == (wbm_rbs ? M_IDLE_R0 + 0x10u : M_IDLE_R0), "the readback is not BASE_LSB, then MISC");
			*v = wbm_rbs ? (C.wbm_rb_bad == 2 ? 0x03u : 0x43u) : (uint32_t)ln_iova ^ (C.wbm_rb_bad == 1 ? 0x1000u : 0u);
			wbm_rbs++;
			return;
		}
		CHECK(idle_pub && (off == M_IDLE_HP || off == M_IDLE_HP + 4u), "a WBM read other than R2 HP/TP after the setup");
		*v = C.r2_unreadable ? ~0u : off == M_IDLE_HP ? m_idle_hp : m_idle_tp;
		wbm_watch_reads++;
		return;
	}
	if (wbm_writes < 8u) {
		const uint32_t want[8] = { (uint32_t)ln_iova, 0x01000000u, 2u, 0x01000002u, 0u, 0u, 0u, 0x43u };
		CHECK(wbm_reads == 9u && !m_refused(), "a WBM write before the nine pre-reads, or although they refuse");
		CHECK(!(C.wbm_fault_after && wbm_writes >= C.wbm_fault_after), "a WBM write after the latched fault");
		if (shadow || off != m_wbm_waddr[wbm_writes] || *v != want[wbm_writes]) {
			fprintf(stderr, "ring 120 write %u: %#x = %#x (shadow %u), want %#x = %#x\n", wbm_writes, off, *v, shadow,
			    m_wbm_waddr[wbm_writes], want[wbm_writes]);
			fail("ring 120 write order or value is not L19's");
		}
		wbm_writes++;
		if (C.wbm_fault_after && wbm_writes == C.wbm_fault_after) { fw_faults++; }
		idle_live = wbm_writes == 8u;
		return;
	}
	CHECK(off == M_IDLE_HP && wbm_rbs == 2u && !idle_pub, "a WBM write after the setup other than the one HP publish");
	CHECK(shadow == !wl15_variant, "idle HP published by the wrong route for the shadow mode");
	model_event('P');
	CHECK(*v == 65534u, "idle HP is not 32767 entries in words");
	for (uint32_t i = 0; i < 32768u; i++) {
		const uint8_t *e = ln_host + 8u * i;
		uint64_t a = ln_iova + 0x40000u + 128ull * i;
		uint32_t cookie = ((32766u - i) + 0x4000u) << 3;
		if (i < 32767u) {
			CHECK(g32(e) == (uint32_t)a && g32(e + 4) == (((uint32_t)(a >> 32) & 0xffu) | 1u << 8 | cookie << 12),
			    "idle entry not in place at the HP publish");
		} else { CHECK(!g32(e) && !g32(e + 4), "idle entry 32767 listed"); }
	}
	for (uint32_t o = 0x40000u; o < LNA; o++) { CHECK(!ln_host[o], "link descriptors not zero at the HP publish"); }
	idle_pub = true; m_idle_hp = *v; m_idle_tp = 0; wbm_pubs++;
}

/* SRING_SETUP words for the RX arena at rx_iova (rx_path_plan.md 2.6.2, the verifier's table), written down again. */
static void
m_sring_want(unsigned i, uint32_t *w)
{
	static const uint32_t t[4][7] = {
		{ 0x0205000bu, 0x04000u, 0x00022000u, 0x400u, 0x280u, 0x003e0000u, 0x400u },
		{ 0x0100010bu, 0x0c000u, 0x02021000u, 0x404u, 0x284u, 0x003e0000u, 0x200u },
		{ 0x0100020bu, 0x10000u, 0x02021000u, 0x424u, 0x2a4u, 0x003e0000u, 0x200u },
		{ 0x0007010bu, 0x14000u, 0x00082000u, 0x2e8u, 0x468u, 0x00200008u, 0x000u },
	};
	memset(w, 0, 13 * 4);
	w[0] = t[i][0]; w[1] = (uint32_t)rx_iova + t[i][1]; w[3] = t[i][2]; w[4] = (uint32_t)rx_iova + t[i][3];
	w[6] = (uint32_t)rx_iova + t[i][4]; w[11] = t[i][5]; w[12] = t[i][6];
}

static void
fw_mac_post(unsigned m, const uint32_t *e)
{
	uint8_t *d;
	CHECK((fw_mac_hp[m] + 2u) % 4096u != fw_mac_tp[m], "model: MAC ring full");
	d = rxd(m_mac_off[m] + 4u * fw_mac_hp[m], 8, true, "MAC ring entry");
	p32(d, e[0]); p32(d + 4, e[1]);
	fw_mac_hp[m] = (fw_mac_hp[m] + 2u) % 4096u;
	p32(rxd(0x400u + 4u * (m_mac_id[m] - 160u), 4, true, "wrp of a MAC ring"), fw_mac_hp[m]);
}

/* The firmware takes N refill entries at the TP: each the start of a posted slot, RBM 6, cookie = slot, once. */
static void
fw_take(unsigned n)
{
	for (unsigned k = 0; k < n; k++) {
		const uint8_t *e = rxd(0x4000u + 4u * fw_refill_tp, 8, false, "refill entry");
		uint32_t w0 = g32(e), w1 = g32(e + 4), slot = (w0 - (uint32_t)rx_iova - 0x20000u) / 0x1000u;
		CHECK(fw_refill_tp != 8190u, "model: a take past the refill HP");
		CHECK(w0 >= (uint32_t)rx_iova + 0x20000u && !((w0 - (uint32_t)rx_iova - 0x20000u) % 0x1000u) && slot < 4095u &&
		    !(w1 & 0xffu), "refill entry is not the start of a posted slot");
		CHECK(((w1 >> 8) & 0xfu) == 6u && (w1 >> 12) == slot, "refill entry RBM or cookie");
		CHECK(!slot_listed[slot]++, "a slot listed twice");
		fw_taken[fw_taken_n][0] = w0; fw_taken[fw_taken_n][1] = w1; fw_taken_n++;
		fw_refill_tp += 2u;
	}
	if (n) { p32(rxd(4u * 160u, 4, true, "rdp[160]"), fw_refill_tp); }
}

/* After the 4th SRING_SETUP: wrp[0] read, C.rx_take entries taken, C.rx_post[m] of them written to each MAC ring
 * (RBM / cookie as given), then C.mac_foreign foreign entries on mac0 (64 bytes into a slot, slot 4095's start, a
 * posted slot's start with address bits 39:32 = 1, in turn: foreign by address alone, 2.8). */
static void
fw_rx_setup(void)
{
	CHECK(!fw_rx_ready && fw_sring_n == 4, "model: refill taken before the 4th SRING_SETUP or twice");
	CHECK(g32(rxd(0x400u, 4, false, "wrp[0]")) == 8190u, "refill HP is not 4095 entries");
	fw_take(C.rx_take);
	for (unsigned m = 0; m < 2; m++) {
		for (unsigned j = 0; j < C.rx_post[m]; j++) {
			uint32_t e[2] = { fw_taken[fw_posted_n][0], fw_taken[fw_posted_n][1] };
			if (C.mac_rbm) { e[1] = (e[1] & ~0xf00u) | C.mac_rbm << 8; }
			e[1] ^= C.mac_cookie_xor << 12;
			fw_mac_post(m, e);
			fw_posted_n++;
		}
	}
	for (unsigned j = 0; j < C.mac_foreign; j++) {
		const uint32_t slot = j % 3u == 1u ? 4095u : 5u + j;
		const uint32_t e[2] = { (uint32_t)rx_iova + 0x20000u + 0x1000u * slot + (j % 3u ? 0u : 64u),
		    6u << 8 | slot << 12 | (j % 3u == 2u ? 1u : 0u) };
		fw_mac_post(0, e);
	}
	fw_rx_ready = true;
}

static void
fw_sring(const uint8_t *pl, uint32_t plen)
{
	uint32_t want[13];
	CHECK(wl20_bits & WL20_RXRING, "SRING_SETUP without w20 0x100");
	CHECK(plen == 52 && fw_sring_n < 4, "SRING_SETUP length, or more than four");
	CHECK(fw_htt_req == 1 && q_head > fw_htt_conf_q && fw_c2_step == 0 && !fw_chan_cmds && !fw_pset_n,
	    "SRING_SETUP before VERSION_CONF reached the host, or after the channel list or the chain masks");
	CHECK(rx_pub && (!(wl20_bits & WL20_IDLELINK) || idle_pub || m_refused()), "SRING_SETUP before wrp[0] or the idle HP");
	m_sring_want(fw_sring_n, want);
	for (unsigned k = 0; k < 13; k++) {
		if (g32(pl + 4 * k) != want[k]) {
			fprintf(stderr, "SRING_SETUP %u word %u = %#x, want %#x\n", fw_sring_n, k, g32(pl + 4 * k), want[k]);
			fail("SRING_SETUP word");
		}
	}
	CHECK(!(g32(pl + 48) & (1u << 19)), "SRING_SETUP with RESPONSE_REQUIRED");
	fw_sring_n++;
	if (C.guard_on_sring == fw_sring_n) { guard_bad = true; }
	if (fw_sring_n == 4) {
		if (C.setup_delay) { act(now + C.setup_delay, A_SETUP, 0, 0); }  /* the host's 100 ms look must wait for it */
		else { fw_rx_setup(); }
	}
}

/* One frame RXDMA receives for MAC M: the next buffer of its MAC ring (bytes 0..255, or 32..35), with 0x200 a link
 * descriptor from ring 120 (WBM may put back the first C.links_returned). No buffer: counted, nothing written. */
static void
fw_rx_frame(unsigned m)
{
	if (fw_rx_ready) {
		if (fw_mac_tp[m] == fw_mac_hp[m]) { fw_nobuf++; }
		else {
			const uint8_t *e = rxd(m_mac_off[m] + 4u * fw_mac_tp[m], 8, false, "MAC ring entry");
			uint32_t a = g32(e), slot = (a - (uint32_t)rx_iova - 0x20000u) / 0x1000u;
			CHECK(a >= (uint32_t)rx_iova + 0x20000u && !((a - (uint32_t)rx_iova - 0x20000u) % 0x1000u) && slot < 4095u,
			    "model: RXDMA would write to a foreign address");
			if (C.write32) { p32(rxd(0x20000u + 0x1000u * slot + 32u, 4, true, "frame"), M_FRAME_WORD); }
			else {
				uint8_t *f = rxd(0x20000u + 0x1000u * slot, 256, true, "frame");
				for (unsigned k = 0; k < 256u; k += 4u) { p32(f + k, M_FRAME_WORD); }
			}
			fw_mac_tp[m] = (fw_mac_tp[m] + 2u) % 4096u;
			p32(rxd(4u * m_mac_id[m], 4, true, "rdp of a MAC ring"), fw_mac_tp[m]);
			fw_frames_written++;
		}
	}
	if (idle_pub && C.links_used && m_idle_tp != m_idle_hp) {
		const uint8_t *e = lnd(4u * m_idle_tp, 8, "idle entry");
		uint32_t a = g32(e), w1 = g32(e + 4), d = (a - (uint32_t)ln_iova - 0x40000u) / 128u;
		CHECK(a >= (uint32_t)ln_iova + 0x40000u && !((a - (uint32_t)ln_iova - 0x40000u) % 128u) && d < 32767u &&
		    ((w1 >> 8) & 0xfu) == 1u, "idle entry is not a listed 128-byte descriptor with RBM 1");
		p32(lnd(0x40000u + 128u * d, 4, "link descriptor"), M_FRAME_WORD);
		m_idle_tp = (m_idle_tp + 2u) % 65536u;
		if (++fw_links <= C.links_returned) {
			uint8_t *h = lnd(4u * m_idle_hp, 8, "idle entry return");
			p32(h, a); p32(h + 4, w1);
			m_idle_hp = (m_idle_hp + 2u) % 65536u;
		}
	}
}

/* One RXDMA error entry at rdp[186] (struct hal_reo_entrance_ring: w1 RBM 11:8, w5 with the frame's byte count, w6
 * push 1:0 | code 6:2 with the frame's fragment and sequence numbers). */
static void
fw_err_push(void)
{
	uint8_t *d;
	if (!fw_rx_ready || fw_err_hp / 8u >= 1023u) { return; }
	d = rxd(0x14000u + 4u * fw_err_hp, 32, true, "RXDMA error entry");
	p32(d, (uint32_t)rx_iova + 0x20000u); p32(d + 4, C.err_rbm << 8); p32(d + 8, 0); p32(d + 12, 0); p32(d + 16, 0);
	p32(d + 20, M_ERR_W5); p32(d + 24, C.err_code << 2 | M_ERR_W6_FRAME | M_ERR_W6_KEEP); p32(d + 28, 0x7u << 20);
	fw_err_hp += 8u;
	p32(rxd(4u * 186u, 4, true, "rdp[186]"), fw_err_hp);
}

static void
dev_acts(void)
{
	while (acts_done < n_acts && acts[acts_done].due <= now) {
		const struct dact *x = &acts[acts_done++];
		if (!(epcmd & 4u)) { continue; }
		switch (x->kind) {
		case A_FRAMES: for (unsigned i = 0; i < x->b; i++) { fw_rx_frame(x->a); } break;
		case A_ERR: for (unsigned i = 0; i < x->a; i++) { fw_err_push(); } break;
		case A_SPILL:  /* RXDMA writes past what it was given: 4 dwords in slot 7's upper half, 2 in slot 4095, 3 at the
		                * tail's start, 2 near its end, and in the gaps 1 in rdp[170] and 3 after the error ring */
			rx_inject = true;
			if (C.spill & 1u) { memset(rxd(0x20000u + 0x1000u * 7u + 2048u, 16, true, "spill"), 0x33, 16); }
			if (C.spill & 2u) { memset(rxd(0x20000u + 0x1000u * 4095u + 100u, 8, true, "spill"), 0x33, 8); }
			if (C.spill & 4u) { memset(rxd(0x1020000u + 0x100u, 12, true, "spill"), 0x33, 12); }
			if (C.spill & 8u) { memset(rxd(0x103ff00u, 8, true, "spill"), 0x33, 8); }
			if (C.spill & 16u) { p32(rxd(4u * 170u, 4, true, "spill"), 0x33u); memset(rxd(0x1c040u, 12, true, "spill"), 0x33, 12); }
			rx_inject = false;
			break;
		case A_RDP: rx_inject = true; p32(rxd(4u * x->a, 4, true, "raw rdp"), x->b); rx_inject = false; break;
		case A_SETUP: fw_rx_setup(); break;
		default: fw_take(x->a); break;
		}
	}
}

static bool
fw_rule_ok(void)  /* C.rx_rule: what the firmware needs before it hands MGMT_RX to the host */
{
	const bool links = idle_pub, bufs = fw_rx_ready && fw_refill_tp, chain = fw_pset_n == 2;
	switch (C.rx_rule) {
	case 0: return true;
	case 1: return links;
	case 2: return links && bufs;
	case 3: return bufs;
	case 4: return chain;
	default: return false;
	}
}

static void
fw_scan_event_id(uint32_t type, uint32_t reason, uint32_t freq, uint32_t vdev, uint32_t id, uint32_t at)
{
	if (C.c3_echo_id) { id = C.c3_echo_id; }  /* C3: a target that answers with an id of its own */
	m_reset(); m_tlv(0x24, 28); m_u32(type); m_u32(reason); m_u32(freq); m_u32(1); m_u32(id); m_u32(vdev); m_u32(at);
	if (C.bad_scan_tlv && type == 8 && freq == 2437) { mb[0] = 0x40; }  /* TLV length 64: past the event */
	wmi_out(0x3001, 0, at);
}
static void
fw_scan_event(uint32_t type, uint32_t reason, uint32_t freq, uint32_t vdev, uint32_t at)
{
	fw_scan_event_id(type, reason, freq, vdev, 0xa000, at);
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
	m_u32(flags ? 0u : busy); m_u32(flags ? 0u : cycle); m_zero(2 * 4); m_u32(flags ? 0u : rx); m_zero(2 * 4);
	m_u32(flags ? 0u : C.ci_tx);  /* tx_frame_cnt: 0 unless a C2c scenario sets it (D4) */
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
	const bool deliver_mgmt = fw_rule_ok();  /* C2c: MGMT_RX only with what the scenario's firmware needs */
	for (unsigned i = 0; i < n; i++) {
		uint32_t t0 = 5 + 150 * i, k = 0, heard = 0;
		for (unsigned j = 0; j < n_beacons; j++) { heard += beacons[j].idx == i; }
		/* C2c device actions on this channel (RXDMA frames, error entries, spills, raw pointer words, takes) */
		if (C.rx_frames) { act(now + t0 + 3, A_FRAMES, i < 13 ? 0u : 1u, C.rx_frames); }
		if (i == 0 && C.rx_frames_ch0) { act(now + t0 + 3, A_FRAMES, 0, C.rx_frames_ch0); }
		if (i == 1 && C.take_more) { act(now + t0 + 4, A_TAKE, C.take_more, 0); }
		if (i == 2 && C.err_n) { act(now + t0 + 4, A_ERR, C.err_n, 0); }
		if (i == 3 && C.refill_tp_raw[0]) { act(now + t0 + 4, A_RDP, 160, C.refill_tp_raw[0]); }
		if (i == 4 && C.spill) { act(now + t0 + 4, A_SPILL, 0, 0); }
		if (i == 5 && C.rdp_err_raw) { act(now + t0 + 4, A_RDP, 186, C.rdp_err_raw); }
		if (i == 30 && C.refill_tp_raw[1]) { act(now + t0 + 4, A_RDP, 160, C.refill_tp_raw[1]); }
		fw_scan_event(1u << 3, 0, m_chan(i), 0, t0);
		for (unsigned r = 0; r <= C.ci_repeat; r++) { fw_chan_info(0, m_chan(i), -96 - (int32_t)(i % 5), i, heard, t0 + 1); }
		if (C.ci_repeat && i == 0) { fw_chan_info(2, m_chan(i), -96, i, heard, t0 + 1); }  /* C2b: flags neither START nor END */
		if (i == 2) {  /* an unprompted event mid-scan: logged by id only */
			m_reset(); m_tlv(0x400, 12); m_zero(12); wmi_out(0x402b, 0, t0 + 1);
		}
		for (unsigned j = 0; j < n_beacons; j++) {
			if (beacons[j].idx == i && deliver_mgmt) { fw_mgmt_rx(&beacons[j], t0 + 2 + k++); }
		}
		fw_chan_info(1, m_chan(i), 0, i, 0, t0 + 148);
	}
	if (C.scan_mode != 3) { fw_scan_event(1u << 1, 0, 0, 0, 5 + 150 * n + 1); }
}

/* ---- C3 firmware: the W25 service's scans ---- */
/* what the (fake) iOS side asked of the scan in flight, worked out by the model itself (h_take) */
#if !defined(FUXI_WLAN_E2E)
static uint16_t h_expect[64];
#endif
static unsigned h_expect_n;
static uint32_t h_expect_dwell;
static bool h_req_passive;
static bool m_listed(uint32_t mhz, unsigned n)
{
	for (unsigned i = 0; i < n; i++) { if (m_chan(i) == mhz) { return true; } }
	return false;
}

/* The scan of START_SCAN: STARTED, per channel (DWELL ms each) FOREIGN_CHAN, CHAN_INFO START, the frames
 * heard there, CHAN_INFO END, then COMPLETED; or the scripted failure. Tagged with the scan id. */
static void
fw_c3_script(unsigned mode, uint32_t id, uint32_t n, uint32_t dwell)
{
	uint32_t t = 2;
	fw_tag = id;
	if (mode == 1) { fw_scan_event_id(1u << 6, 4, 0, 0, id, t); fw_tag = 0; return; }
	if (mode == 2) { fw_tag = 0; return; }
	fw_scan_event_id(1u << 0, 0, 0, 0, id, t);
	/* a late COMPLETED of the previous scan, once this one runs: taken, it would end this scan at 3 ms */
	if (C.c3_stale && fw_c3_scans >= 2) { fw_scan_event_id(1u << 1, 0, 0, 0, id - 1u, t + 1); }
	if (mode == 4) { fw_scan_event_id(1u << 4, 0, 0, 0, id, t + 3); fw_tag = 0; return; }
	const bool deliver_mgmt = fw_rule_ok();  /* C2c: MGMT_RX only with what the scenario's firmware needs */
	for (uint32_t i = 0; i < n; i++) {
		uint32_t t0 = 5 + dwell * i, k = 0, heard = 0, ix = 0;
		while (ix < 38 && m_chan(ix) != fw_c3_chan[i]) { ix++; }
		for (unsigned j = 0; j < n_beacons; j++) { heard += beacons[j].idx == ix; }
		/* C2c: RXDMA frames in the first W25 scan only (the model's action list is not recycled) */
		if (C.rx_frames && fw_c3_scans == 1) { act(now + t0 + 3, A_FRAMES, ix < 13 ? 0u : 1u, C.rx_frames); }
		fw_scan_event_id(1u << 3, 0, fw_c3_chan[i], 0, id, t0);
		fw_chan_info(0, fw_c3_chan[i], -96, ix, heard, t0 + 1);
		for (unsigned j = 0; j < n_beacons; j++) {
			if (beacons[j].idx == ix && deliver_mgmt) { fw_mgmt_rx(&beacons[j], t0 + 2 + k++); }
		}
		fw_chan_info(1, fw_c3_chan[i], 0, ix, 0, t0 + dwell - 2);
	}
	if (mode != 3) { fw_scan_event_id(1u << 1, 0, 0, 0, id, 5 + dwell * n + 1); }
	fw_tag = 0;
}

/* START_SCAN from the service, word by word: a fresh id in 0xa001..0xafff, PASSIVE | CHAN_STAT only (no
 * broadcast probe, no ACTIVE), the dwell and channels the request asked for (listed, each once, in its
 * order), max_scan_time n x dwell x 3/2, no SSID, no IE, no probe. */
static void
fw_c3_start(const uint8_t *pl, uint32_t plen, unsigned need)
{
	const uint8_t *f = pl + 4;
	uint32_t want[40] = {0}, n = g32(f + 4 * 17), id = g32(f + 4), dwell = g32(f + 4 * 7);
	CHECK(fw_c2_step == 7 || fw_c2_step == 8, "W25 START_SCAN before the vdev was set up");
	CHECK(n >= 1 && n <= 38 && plen == 4 + 160 + 4 + 4 * n + 4 + 4 + 8 + 4, "W25 START_SCAN length / channel count");
	CHECK(id >= 0xa001 && id <= 0xafff && id != fw_c3_id, "W25 scan id not fresh in 0xa001..0xafff");
	CHECK(dwell >= 50 && dwell <= 200, "W25 passive dwell outside 50..200");
	want[0] = 0x4du << 16 | 156u; want[1] = id; want[2] = 1; want[5] = 0x5f; want[6] = 50; want[7] = dwell; want[8] = 50;
	want[9] = 500; want[13] = n * dwell * 3 / 2; want[14] = 5; want[15] = 0x11; want[17] = n; want[18] = 1; want[37] = 70;
	want[38] = 70;
	for (unsigned i = 0; i < 40; i++) {
		if (g32(f + 4 * i) != want[i]) { fprintf(stderr, "W25 START_SCAN word %u = %#x, want %#x\n", i, g32(f + 4 * i), want[i]); fail("W25 START_SCAN"); }
	}
	const uint8_t *p = f + 160;
	CHECK(g32(p) == (0x10u << 16 | 4 * n), "W25 START_SCAN channel array header");
	for (unsigned i = 0; i < n; i++) {
		fw_c3_chan[i] = g32(p + 4 + 4 * i);
		CHECK(m_listed(fw_c3_chan[i], 38), "W25 START_SCAN channel off the 38-channel list");
		for (unsigned j = 0; j < i; j++) { CHECK(fw_c3_chan[j] != fw_c3_chan[i], "W25 START_SCAN channel twice"); }
	}
	p += 4 + 4 * n;
	CHECK(g32(p) == (0x13u << 16) && g32(p + 4) == (0x13u << 16 | 8u) && !memcmp(p + 8, "\xff\xff\xff\xff\xff\xff\0\0", 8) &&
	    g32(p + 16) == (0x11u << 16), "W25 START_SCAN SSID/BSSID/IE arrays");
#if !defined(FUXI_WLAN_E2E)
	CHECK(h_req_passive, "a request that was not passive reached the firmware as a START_SCAN");
	CHECK(n == h_expect_n && dwell == h_expect_dwell, "W25 START_SCAN is not the request's channel count / dwell");
	for (unsigned i = 0; i < n; i++) { CHECK(fw_c3_chan[i] == h_expect[i], "W25 START_SCAN is not the request's channels"); }
#endif
	fw_c3_scans++; fw_scan_cmds++; fw_c2_step = 8;
	fw_c3_id = id; fw_c3_n = n; fw_c3_dwell = dwell; fw_c3_max = want[13]; fw_c3_flags = g32(f + 4 * 15);
	fw_c3_start_at = now;
	fw_c3_mode_now = fw_c3_scans < 17 ? C.c3_mode[fw_c3_scans] : fw_c3_scans == C.c3_fail_at ? 1u : 0u;
	if (C.c3_guard_after && fw_c3_scans >= C.c3_guard_after) { guard_bad = true; }
	bool held = (C.c3_starve_after && fw_c3_scans >= C.c3_starve_after) || fw_c3_scans <= C.c3_hold;
	if (!held && !C.c3_refund_at_end) { fw_refund(need, 1); }
	fw_c3_script(fw_c3_mode_now, id, n, dwell);
	if (!held && C.c3_refund_at_end) { fw_refund(need, 5 + dwell * n + 2); }   /* right after its COMPLETED */
}

static void
fw_c2_cmd(uint32_t id, const uint8_t *pl, uint32_t plen, unsigned need)
{
	const uint8_t *v = pl + 8;   /* the fixed TLV's value */
	uint32_t exp_n = C.exp_n, pdev = C.exp_pdev;
	const uint32_t rx_mask = C.phy_rx_mask ? C.phy_rx_mask & C.phy_mask : C.phy_mask;  /* the last DBS PHY's, as m_mac_phy */
	if (id == 0x4003) {  /* C2c W26m: PDEV_SET_PARAM TX then RX chain mask (mac.c:8562-8590, wmi.c:1620-1650) */
		CHECK(wl20_bits & WL20_CHAINMASK, "PDEV_SET_PARAM without w20 0x400");
		CHECK(fw_c2_step == 0 && !fw_chan_cmds && fw_pset_n < 2, "PDEV_SET_PARAM after the channel list, or more than two");
		CHECK((!(wl20_bits & WL20_HTTVER) || (fw_htt_req == 1 && q_head > fw_htt_conf_q)) &&
		    (!(wl20_bits & WL20_RXRING) || fw_sring_n == 4), "PDEV_SET_PARAM before VERSION_CONF or the four SRING_SETUP");
		CHECK(plen == 20 && g32(pl + 4) == (0x52u << 16 | 12u) && g32(v) == pdev && g32(v + 4) == fw_pset_n + 1u &&
		    g32(v + 8) == (fw_pset_n ? rx_mask : C.phy_mask) && C.phy_mask,
		    "PDEV_SET_PARAM is not {pdev, TX 1 with the PHY tx mask, then RX 2 with its rx mask}");
		for (unsigned i = 0; i < 4; i++) { fw_pset_w[fw_pset_n][i] = g32(pl + 4 + 4 * i); }
		fw_pset_n++;
		fw_refund(need, 1);
		return;
	}
	if (id == 0x3003) {  /* SCAN_CHAN_LIST (wmi.c:2880-2997, reg.c:183-238) */
		CHECK(fw_init_ok == 1 && (fw_hw_mode_ok == 1 || (wl20_bits & WL20_NOHWMODE)), "channel list before INIT/HW mode");
		CHECK(!(wl20_bits & WL20_HTTVER) || fw_htt_req == 1, "channel list before the HTT version exchange");
		CHECK(!(wl20_bits & WL20_RXRING) || fw_sring_n == 4, "channel list before the four SRING_SETUP");
		CHECK(!(wl20_bits & WL20_CHAINMASK) || !C.phy_mask || !C.phy_bands || fw_pset_n == 2,
		    "channel list before the two PDEV_SET_PARAM");
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
		uint32_t tx = (uint32_t)__builtin_popcount(C.phy_mask), rx = (uint32_t)__builtin_popcount(rx_mask), want_tx[2], want_rx[2];
		if (!C.phy_bands || !tx) { tx = rx = fw_chains; }
		uint32_t bands = C.phy_bands && C.phy_mask ? C.phy_bands : 3u;
		want_tx[0] = bands & 1 ? tx : 0; want_tx[1] = bands & 2 ? tx : 0;
		want_rx[0] = bands & 1 ? rx : 0; want_rx[1] = bands & 2 ? rx : 0;
		CHECK(fw_c2_step == 1, "VDEV_CREATE before the whole channel list");
		CHECK(plen == 88 && g32(pl + 4) == (0x56u << 16 | 44u) && g32(v) == 0 && g32(v + 4) == 2 && g32(v + 8) == 0,
		    "VDEV_CREATE length/tag/vdev 0/type STA/subtype 0");
		CHECK(!memcmp(v + 12, mac, 8), "VDEV_CREATE MAC is not the K-W2 address + 2 pad");
		CHECK(g32(v + 20) == 2 && g32(v + 24) == pdev && g32(v + 28) == 1 && g32(v + 32) == 0 && g32(v + 36) == 1 &&
		    g32(v + 40) == 0, "VDEV_CREATE streams/pdev/mbssid/stats id");
		CHECK(g32(pl + 52) == (0x12u << 16 | 32u), "VDEV_CREATE txrx streams array header");
		for (unsigned b = 0; b < 2; b++) {
			const uint8_t *s = pl + 56 + 16 * b;
			CHECK(g32(s) == (0x19bu << 16 | 12u) && g32(s + 4) == b && g32(s + 8) == want_tx[b] && g32(s + 12) == want_rx[b],
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
	} else if (id == 0x3001 && wl_w25) {  /* C3: the service's START_SCAN */
		fw_c3_start(pl, plen, need);
		return;
	} else if (id == 0x3002 && wl_w25) {  /* C3: STOP_SCAN of the scan in flight (mid-service, or containment) */
		CHECK(fw_c2_step == 8 && !wl_ce.stopping, "W25 STOP_SCAN without a scan, or after the latch");
		CHECK(plen == 28 && g32(pl + 4) == (0x4eu << 16 | 20u) && g32(v) == 0 && g32(v + 4) == fw_c3_id && g32(v + 8) == 0 &&
		    g32(v + 12) == 0, "W25 STOP_SCAN requester/scan id of the scan in flight/STOP_ONE/vdev");
		fw_stop_pdev = g32(v + 16); fw_stop_cmds++; fw_c3_stop_id = fw_c3_id; fw_c3_stop_at = now;
		if (fw_c3_scans < 17) { fw_c3_stop_after[fw_c3_scans] = (uint32_t)(now - fw_c3_start_at); }
		fw_dead_tag = fw_c3_id;  /* what that scan would still have sent never comes */
		if (C.c3_stop_late_ms) { fw_refund(need, 1); }   /* (first: the late answer must not hold the credit back) */
		if (!C.c3_stop_silent) {
			uint32_t at = C.c3_stop_late_ms ? C.c3_stop_late_ms : 2;
			if (fw_c3_mode_now == 2) { fw_scan_event_id(1u << 4, 0, 0, 0, fw_c3_id, at); }   /* never started: DEQUEUED */
			else { fw_scan_event_id(1u << 1, WLWMI_SCAN_REASON_CANCELLED, 0, 0, fw_c3_id, at); }
		}
		if (!C.c3_stop_late_ms) { fw_refund(need, 1); }
		return;
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
		w[1] = C.stats_tx_frame;  /* C2c D4: 0 unless a scenario sets it */
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
		if (ce == 4 && flags == 0 && plen >= 4 && (g32(pl) & 0xffu) == 0xbu) { fw_sring(pl, plen); return; }  /* C2c */
		CHECK(ce == 4 && flags == 0 && plen == 8 && g32(pl) == 0 && g32(pl + 4) == (5u | 4u << 8 | 2u << 16),
		    "HTT VERSION_REQ is not {msg 0, TCL metadata option tag 5 len 4 value 2} on CE4");
		CHECK(fw_init_ok == 1 && fw_c2_step == 0, "HTT VERSION_REQ out of order");
		fw_htt_req++;
		if (!C.htt_silent) {
			uint8_t r[4];
			p32(r, 0u | C.htt_minor << 8 | C.htt_major << 16);
			fw_htt_conf_q = q_tail;
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
	if (fw_cmd_n < 64) { fw_cmd_log[fw_cmd_n++] = id; }  /* C2c: the WMI command stream, for comparisons */
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
	} else if (id == 0x3001 || id == 0x3002 || id == 0x3003 || id == 0x5001 || id == 0x5008 || id == 0x9001 || id == 0x9002 ||
	    id == 0x4003) {
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

/* ---- C3: a fake iOS side behind the scan handoff (the producer API of patch 0104), scripted ----
 * A scenario queues requests with the time after take() at which the iOS side cancels them (-1 never);
 * ready(), the frames (checked to be in the arena's RX buffers, i.e. handed over in place) and every done()
 * are recorded. starting() and stopped() are fuxi_wlan_init's and fw_thread's, never the W20 block's.
 * Built with -DFUXI_WLAN_E2E the real fuxi_wifi_radio.c is linked instead. */
#if !defined(FUXI_WLAN_E2E)
#define H_NONE 999
#define H_AUTO 255u                     /* the slot of the generated requests (h_auto) */
struct h_req { struct fuxi_wifi_radio_request r; int cancel_ms; };
static struct h_req hq[256];
static unsigned hq_n, hq_next, h_ready, h_ready_n, h_takes, h_dones, h_live_idx;
static unsigned h_gone_after;           /* C3 fix: the iOS side stops for the boot after this many done() (0: never) */
static unsigned h_auto, h_auto_ok, h_auto_frames, h_ready_s1, h_ready_s10;   /* generated {2437} requests */
#define M_CREDITS 3                     /* the WMI endpoint's credits between commands: 4 less INIT's, never refunded */
static uint16_t h_ready_f[64];
static uint32_t h_live, h_first_id, h_last_id;
static uint64_t h_taken_at, h_ready_at;
static int h_status[256];
struct h_frame { uint32_t seq; uint8_t bssid[6]; uint16_t mhz; int16_t rssi, noise; uint32_t len; };
static struct h_frame h_fr[8192];
static unsigned h_fr_n;

static void
h_reset(void)
{
	hq_n = hq_next = h_ready = h_ready_n = h_takes = h_dones = h_live_idx = h_fr_n = h_gone_after = 0;
	h_auto = h_auto_ok = h_auto_frames = h_ready_s1 = h_ready_s10 = 0;
	h_live = h_first_id = h_last_id = 0; h_taken_at = h_ready_at = 0; h_expect_n = 0; h_expect_dwell = 0; h_req_passive = false;
	for (unsigned i = 0; i < 256; i++) { h_status[i] = H_NONE; }
}

static void
h_push(const uint16_t *f, unsigned n, bool passive, uint16_t dwell, int cancel_ms)
{
	CHECK(hq_n < 256 && n <= 64, "model request queue");
	struct h_req *h = &hq[hq_n];
	memset(h, 0, sizeof(*h));
	h->r.seq = hq_n + 1; h->r.freq_count = (uint16_t)n; h->r.passive = passive; h->r.dwell_ms = dwell;
	memcpy(h->r.freqs, f, n * sizeof(f[0]));
	h->cancel_ms = cancel_ms;
	hq_n++;
}

void
fuxi_wifi_radio_starting(void)
{
	fail("fuxi_wifi_radio_starting from the W20 block");
}

void
fuxi_wifi_radio_stopped(unsigned code)
{
	(void)code;
	fail("fuxi_wifi_radio_stopped from the W20 block");
}

void
fuxi_wifi_radio_ready(const uint16_t *freqs, unsigned count)
{
	CHECK(!h_ready && count && count <= 64 && !h_live, "ready() twice, empty, or with a request in flight");
	memcpy(h_ready_f, freqs, count * sizeof(freqs[0]));
	h_ready = 1; h_ready_n = count; h_ready_at = now; h_ready_s1 = sleeps_1; h_ready_s10 = sleeps_10;
}

/* As fuxi_wifi_radio.c: nothing before READY, one request at a time. The model works out on its own
 * what the radio must scan (the request's listed channels, each once, in order; the dwell rule). */
/* STOPPED (as fuxi_wifi_radio.c after its own 3 timeouts): terminal, take() gives nothing more */
static bool h_gone(void) { return h_gone_after && h_dones >= h_gone_after; }

bool
fuxi_wifi_radio_wanted(void)
{
	CHECK(h_ready && !h_live, "wanted() before ready() or with a request in flight");
	return !h_gone();
}

bool
fuxi_wifi_radio_take(struct fuxi_wifi_radio_request *out)
{
	if (!h_ready || h_live || h_gone() || (hq_next == hq_n && !h_auto)) { return false; }
	if (hq_next == hq_n) {  /* a generated request: {2437}, passive, 150 ms, seq 1000 + k */
		struct h_req *a = &hq[H_AUTO];
		memset(a, 0, sizeof(*a));
		a->r.seq = 1000u + h_takes; a->r.freq_count = 1; a->r.freqs[0] = 2437; a->r.passive = true; a->r.dwell_ms = 150;
		a->cancel_ms = -1;
		h_auto--;
	}
	unsigned idx = hq_next < hq_n ? hq_next++ : H_AUTO;
	const struct h_req *h = &hq[idx];
	unsigned nlist = (wl20_bits & WL20_BAND24) ? 13u : 38u;
	*out = h->r;
	h_live = h->r.seq; h_live_idx = idx; h_taken_at = now; h_takes++; h_req_passive = h->r.passive;
	h_expect_n = 0;
	for (unsigned i = 0; i < h->r.freq_count; i++) {
		bool again = false;
		for (unsigned j = 0; j < h_expect_n; j++) { again |= h_expect[j] == h->r.freqs[i]; }
		if (!again && m_listed(h->r.freqs[i], nlist)) { h_expect[h_expect_n++] = h->r.freqs[i]; }
	}
	h_expect_dwell = h->r.dwell_ms >= 50 && h->r.dwell_ms <= 200 ? h->r.dwell_ms : 150;
	return true;
}

bool
fuxi_wifi_radio_cancelled(uint32_t seq)
{
	const struct h_req *h = &hq[h_live_idx];
	return seq != h_live || (h->cancel_ms >= 0 && now >= h_taken_at + (uint64_t)h->cancel_ms);
}

int
fuxi_wifi_radio_frame(uint32_t seq, const uint8_t *frame, size_t length, uint16_t rx_mhz, int16_t rssi_dbm, int16_t noise_dbm)
{
	CHECK(seq && seq == h_live, "a frame for a request not in flight");
	CHECK(frame >= arena_host + 0x28000u && frame + length <= arena_host + 0x66000u,
	    "a frame not in the CE1/CE2 RX buffers (not handed over in place)");
	CHECK(length >= 36 && length <= WLCE_BUF, "frame length");
	if (h_live_idx == H_AUTO) { h_auto_frames++; return 0; }
	CHECK(h_fr_n < sizeof(h_fr) / sizeof(h_fr[0]), "model frame table");
	struct h_frame *r = &h_fr[h_fr_n++];
	r->seq = seq; memcpy(r->bssid, frame + 16, 6); r->mhz = rx_mhz; r->rssi = rssi_dbm; r->noise = noise_dbm;
	r->len = (uint32_t)length;
	return 0;
}

void
fuxi_wifi_radio_done(uint32_t seq, int status)
{
	CHECK(seq && seq == h_live && (h_live_idx == H_AUTO || h_status[h_live_idx] == H_NONE),
	    "done() for a request not in flight, or twice");
	if (h_live_idx == H_AUTO) { h_auto_ok += status == 0; } else { h_status[h_live_idx] = status; }
	if (status != EINVAL) { h_last_id = fw_c3_id; h_first_id = h_first_id ? h_first_id : fw_c3_id; }
	h_live = 0; h_dones++;
}
#else
void e2e_ios_start(void);
void e2e_tick(uint64_t ms);
int e2e_ios_finish(void);
#endif

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
	if (wl15_shadow_v3[i] == M_IDLE_HP) { wbm_access(M_IDLE_HP, true, &v, true); return; }  /* C2c: ring 120's HP */
	ce_access(wl15_shadow_v3[i], true, &v, true);
}

/* A window access: the WBM block (C2c) or the CE registers. */
static void
win_access(uint64_t pa, bool wr, uint32_t *v)
{
	uint32_t off = (bar_win & 0x3fu) << 19 | (uint32_t)(pa - WL_BAR0 - 0x80000ull);
	CHECK(bar_win & 0x40000000u, "window access with the window disabled");
	if (off >= 0x00a30000u && off < 0x00a40000u) { wbm_access(off, wr, v, false); return; }
	ce_access(off, wr, v, false);
}

static uint32_t
wl_prd(uint64_t pa)
{
	if (pa >= WL_BAR0 + 0x80000ull && pa < WL_BAR0 + 0x100000ull) {
		uint32_t v = 0;
		win_access(pa, false, &v);
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
		win_access(pa, true, &v);
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
	if (pa == WL_BAR0 + 0x38) {
		/* C2c: RXDMA still writes a posted buffer during the MHI reset, before bus master off */
		if (C.late_write && fw_rx_ready && (epcmd & 4u)) { memset(rxd(0x20000u + 0x1000u * 1u + 8u, 4, true, "late frame"), 0x77, 4); }
		mhictrl = 0;
		return 0;
	}
	if (pa == WL_EPCFG + 4) { if (!C.fail_bm) { epcmd = value; } return 0; }
	CHECK(pa == TS_TLMM + 0x1000ull * 94 + 4, "containment write outside the model");
	perst_io = value;
	return 0;
}

/* C2c arenas: 2 MiB-aligned IOVA inside the pool, 0xa5 everywhere (the code must zero what it relies on). */
static int
w26_alloc(struct wl_dma *d, vm_size_t bytes)
{
	const bool rx = d == &wl_rx_arena;
	CHECK(!d->va && bytes == (rx ? RXA : LNA) && (wl20_bits & (rx ? WL20_RXRING : WL20_IDLELINK)), "unexpected W26 allocation");
	CHECK(!(wl_iova_next & 0x1fffffu) && wl_iova_next + bytes <= wl_iova_end, "W26 arena IOVA not 2 MiB aligned inside the pool");
	CHECK(!rx || !(wl20_bits & WL20_IDLELINK) || ln_host, "RX arena allocated before the link arena");
	if (rx && C.rx_alloc_fail) { return 1; }
	void *p = aligned_alloc(16384, bytes);
	CHECK(p != NULL, "host allocation");
	memset(p, 0xa5, bytes);
	d->va = (vm_offset_t)p; d->size = bytes; d->iova = wl_iova_next; d->pa = 0x890000000ull;
	wl_iova_next += bytes;
	if (rx) { rx_host = p; rx_iova = d->iova; rx_allocs++; } else { ln_host = p; ln_iova = d->iova; ln_allocs++; }
	return 0;
}

static int
wl_dma_alloc(struct wl_dma *d, vm_size_t bytes)
{
	if (d == &wl_rx_arena || d == &wl_link_arena) { return w26_alloc(d, bytes); }
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
static int
wl_dma_nc(struct wl_dma *d)
{
	CHECK(d == &wl_ce_arena || d == &wl_rx_arena || d == &wl_link_arena, "NC conversion of another buffer");
	d->nc = true;
	return 0;
}
static void
wl_dma_free(struct wl_dma *d)
{
	if (!d->va) { return; }
	if (d == &wl_rx_arena || d == &wl_link_arena) {  /* C2c: only after bus master off, PERST and the SMMU undo, zeroed */
		const uint8_t *p = (const uint8_t *)d->va;
		CHECK(!(epcmd & 4u) && !(perst_io & 2u) && smmu_undone, "W26 arena freed before bus master off, PERST and the SMMU undo");
		for (vm_size_t i = 0; i < d->size; i++) { CHECK(!p[i], "W26 arena not zeroed before its free"); }
		free((void *)d->va);
		if (d == &wl_rx_arena) { rx_host = NULL; rx_mapped = false; rx_frees++; } else { ln_host = NULL; ln_mapped = false; ln_frees++; }
		memset(d, 0, sizeof(*d));
		return;
	}
	CHECK(d == &wl_ce_arena, "free of a buffer the model never allocated");
	free((void *)d->va);
	arena_host = NULL; arena_mapped = false; arena_frees++;
	memset(d, 0, sizeof(*d));
}
static int
wl12_map_target(struct wl_dma *d)
{
	CHECK((d == &wl_ce_arena || d == &wl_rx_arena || d == &wl_link_arena) && d->nc, "map of an unconverted buffer");
	if (d == &wl_link_arena && C.link_map_fail) { return wl_w11_fail("target DMA leaf readback"); }
	if (d == &wl_rx_arena) { rx_mapped = true; } else if (d == &wl_link_arena) { ln_mapped = true; } else { arena_mapped = true; }
	return 0;
}
static int wl_w11_guard(void) { return fw_faults || guard_bad ? wl_w11_fail("AMSS/M0, ownership, discovery or access/SMMU fault") : 0; }
static int wl_w11_events(void) { return wl_w11_guard(); }
static void wl_w11_stop(void) { CHECK(!wl_ce_arena.va || wl_ce.stopping, "W11 stop before the CE containment latch"); stops++; }
static void
wl_sleep(unsigned ms)
{
	now += ms; sleeps += ms;
	sleeps_1 += ms == 1; sleeps_10 += ms == 10;
	/* C3: no service outlives its lifetime by more than a scan */
	CHECK(!wl_w25 || now < 1830000u, "the W25 service ran past its 1800 s lifetime");
	dev_tick();
#if defined(FUXI_WLAN_E2E)
	e2e_tick(now);
#endif
}
static unsigned wl_m6o_cap(unsigned id) { CHECK(id == 0x10, "capability other than PCIe"); return 0x70; }
static void wl_wlfw_diagnostics(const char *label) { printf("FUXI WLAN: %s diagnostic (model)\n", label); }
static void wl5_status(unsigned step) { (void)step; }
static int wl_smmu_fault(unsigned step) { (void)step; return 0; }
static int wl_m6o_msi_undo(void) { return 0; }
static int wl_m6o_undo(bool contained) { (void)contained; return 0; }
static int wl_smmu_undo(void) { CHECK(!(epcmd & 4u) && !(perst_io & 2u), "SMMU undo before bus master off and PERST"); smmu_undone = true; return 0; }

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
	/* C2c: arenas a quarantined or failed scenario left, and the W26 model state */
	if (wl_rx_arena.va) { free((void *)wl_rx_arena.va); memset(&wl_rx_arena, 0, sizeof(wl_rx_arena)); }
	if (wl_link_arena.va) { free((void *)wl_link_arena.va); memset(&wl_link_arena, 0, sizeof(wl_link_arena)); }
	rx_host = ln_host = NULL; rx_iova = ln_iova = 0;
	rx_mapped = ln_mapped = rx_pub = smmu_undone = fw_rx_ready = idle_live = idle_pub = rx_inject = false;
	rx_allocs = rx_frees = ln_allocs = ln_frees = wbm_reads = wbm_writes = wbm_rbs = wbm_watch_reads = wbm_pubs = 0;
	fw_sring_n = fw_pset_n = fw_frames_written = fw_nobuf = fw_links = fw_taken_n = fw_posted_n = 0; fw_htt_conf_q = ~0u;
	m_idle_hp = m_idle_tp = fw_refill_tp = fw_err_hp = 0;
	memset(fw_mac_hp, 0, sizeof(fw_mac_hp)); memset(fw_mac_tp, 0, sizeof(fw_mac_tp)); memset(fw_pset_w, 0, sizeof(fw_pset_w));
	memset(slot_listed, 0, sizeof(slot_listed)); wbm_log_n = 0; n_acts = acts_done = 0; fw_cmd_n = 0;
	memset(&wl_rx, 0, sizeof(wl_rx)); wl26_wbm_writes = 0; wl26_heard_none = false;
	memset(&C, 0, sizeof(C));
	C.rx_take = 2048; C.rx_post[0] = C.rx_post[1] = 1024;
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
	/* C3 */
	wl_w25 = false; sleeps_1 = sleeps_10 = 0;
	fw_tag = fw_dead_tag = 0; fw_dropped = 0; starved_since = 0;
	fw_c3_scans = fw_c3_mode_now = 0; fw_c3_id = fw_c3_n = fw_c3_dwell = fw_c3_max = fw_c3_flags = fw_c3_stop_id = 0;
	fw_c3_start_at = fw_c3_stop_at = 0; h_expect_n = 0; h_expect_dwell = 0; h_req_passive = false;
	memset(fw_c3_stop_after, 0, sizeof(fw_c3_stop_after));
#if !defined(FUXI_WLAN_E2E)
	h_reset();
#endif
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
	if (C.ce4_inflight && len >= 26 && !strncmp(l, "FUXI WLAN: W23 htt version", 26)) {  /* C2c: CE4 shows 16 in flight */
		C.ce4_inflight = 0;
		wl_ce.pipe[4].posted += 16u;
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

/* ---- C2c scenarios (fuxiwlanqrtr=7, w20 0x100 RXRING, 0x200 IDLELINK, 0x400 CHAINMASK) ---- */
static void c2c_reset(unsigned bits) { c2_reset(); wl20_bits = bits; }
static void
show(const char *title)  /* C2C_SHOW=1: the scenario's log on stderr (to read the lines while writing expectations) */
{
	if (getenv("C2C_SHOW")) { fprintf(stderr, "==== %s\n%s", title, logbuf); }
}

/* The owner rule (counts only): no log line shows what RXDMA wrote from a received frame: the frame word of the
 * buffers and link descriptors (hex or decimal), an error entry's unmasked w5 or w6 (hex or decimal). */
static void
no_frame_text(void)
{
	static const char *const bad[] = { "5eedf00d", "1592651789", "0xebad00", "15445248", "0x2f7", "7958787" };
	for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		if (strstr(logbuf, bad[i])) { fprintf(stderr, "frame text %s\n", bad[i]); fail("a received frame's content in the log (owner rule: counts only)"); }
	}
}

/* Proven containment: the C1 checks, then both W26 arenas zeroed (wl_dma_free checks every byte) and freed only after
 * bus master off, PERST and the SMMU undo (wl_dma_free and wl_smmu_undo check the order). */
static void
c2c_contained(void)
{
	const unsigned rx = rx_allocs, ln = ln_allocs;
	/* the host-owned words the device reads stayed as published: wrp[0] (refill HP), wrp[26] (error ring TP, never
	 * reaped), every refill entry; nothing but the device moved after the publish (2.8) */
	if (rx_pub && rx_host) {
		CHECK(g32(rx_host + 0x400u) == 8190u && !g32(rx_host + 0x400u + 4u * 26u), "a host write to wrp[0] or wrp[26] after the publish");
		for (uint32_t k = 0; k < 4096u; k++) {
			uint64_t a = rx_iova + 0x20000u + 0x1000ull * k;
			CHECK(k == 4095u ? !g32(rx_host + 0x4000u + 8u * k) : g32(rx_host + 0x4000u + 8u * k) == (uint32_t)a,
			    "a refill entry rewritten after the publish");
		}
	}
	contained_and_freed();
	no_frame_text();
	CHECK(rx_frees == rx && ln_frees == ln && !wl_rx_arena.va && !wl_link_arena.va, "W26 arenas not freed after proven containment");
	if (rx || ln) {
		static const char *const order[] = {
			"FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo\n",
			"FUXI WLAN: W20 ce contain posted ",
			"FUXI WLAN: abort step 28 MHI reset done, bus master off yes, PERST asserted yes\n",
			"FUXI WLAN: W26 arenas zeroed ",
			"FUXI WLAN: abort buffers freed\n",
		};
		expect_order(order, sizeof(order) / sizeof(order[0]));
	} else { expect_not("W26 rx contain"); expect_not("W26 arenas zeroed"); }
}

static void
s_c2c_pure(void)
{
	static struct wlrx rx;
	static const struct wlce_io io = { io_never_wr, io_never_rd, io_never_wr, io_model_w, io_model_r, NULL };
	uint8_t *ra = aligned_alloc(16384, RXA), *la = aligned_alloc(16384, LNA);
	uint32_t w[13], want[13], regs[WLRX_IDLE_REGS][2];
	reset();
	CHECK(ra && la, "alloc");
	/* the allowlist is exactly the model's (the nine pre-read addresses; the eight write addresses), and the two
	 * register sets do not meet */
	for (uint32_t off = 0x00a30000u; off < 0x00a40000u; off += 4) {
		bool r = false, wr = false;
		for (unsigned i = 0; i < 9; i++) { r |= off == m_pre_addr[i]; }
		for (unsigned i = 0; i < 8; i++) { wr |= off == m_wbm_waddr[i]; }
		CHECK(wlrx_wbm_ok(off, false) == r && wlrx_wbm_ok(off, true) == wr && !wlce_reg_ok(off), "WBM allowlist differs from the model's");
	}
	for (uint32_t off = CE_BASE; off < CE_END; off += 4) { CHECK(!wlrx_wbm_ok(off, false) && !wlrx_wbm_ok(off, true), "WBM allowlist has a CE register"); }
	CHECK(!wlrx_wbm_ok(M_IDLE_R0 + 2, false) && !wlrx_wbm_ok(0x00a34000u + 0x900u, true) && !wlrx_wbm_ok(0x00a3b020u, true),
	    "a misaligned, TCL or REO register allowed");
	/* D1: the pre-read reading */
	{
		uint32_t p[9] = {0};
		CHECK(wlrx_idle_owned(p) == WLRX_IDLE_FREE, "zero pre-reads not free");
		p[2] = 0x40; CHECK(wlrx_idle_owned(p) == WLRX_IDLE_SRNG, "SRNG_ENABLE"); p[2] = 0x3;
		CHECK(wlrx_idle_owned(p) == WLRX_IDLE_FREE, "MISC without SRNG_ENABLE refused");
		p[0] = 0x1000; CHECK(wlrx_idle_owned(p) == WLRX_IDLE_BASE, "base lsb"); p[0] = 0;
		p[1] = 0x01000001; CHECK(wlrx_idle_owned(p) == WLRX_IDLE_BASE, "base msb 7:0"); p[1] = 0x01000000;
		CHECK(wlrx_idle_owned(p) == WLRX_IDLE_FREE, "a size field alone refused");
		p[5] = 2; CHECK(wlrx_idle_owned(p) == WLRX_IDLE_AMBIGUOUS, "scatter mode with base 0 and SRNG_ENABLE 0");
		p[2] = 0x40; CHECK(wlrx_idle_owned(p) == WLRX_IDLE_SRNG, "scatter mode with SRNG_ENABLE"); p[2] = 0; p[5] = 0;
		p[8] = ~0u; CHECK(wlrx_idle_owned(p) == WLRX_IDLE_UNREADABLE, "a ~0 scattered base read"); p[8] = 0x5;
		CHECK(wlrx_idle_owned(p) == WLRX_IDLE_FREE, "a scattered base value alone refused");
	}
	/* layout, SRING_SETUP words, entries, ring 120 registers against the model's own table */
	CHECK(wlrx_layout(&rx, ra, 0xa1a00000ull, RXA) == 0 && wlrx_link_layout(&rx, la, 0xa1400000ull, LNA) == 0, "pure layout");
	CHECK(wlrx_layout(&rx, ra, 0xa1a00800ull, RXA) == 1 && wlrx_layout(&rx, ra, 0xa1a00000ull, RXA - 4) == 1 &&
	    wlrx_layout(&rx, ra, 0xff000000ull, RXA) == 1, "RX layout bounds");
	CHECK(wlrx_layout(&rx, ra, 0xa1a00000ull, RXA) == 0, "pure layout again");
	rx_iova = 0xa1a00000ull; ln_iova = 0xa1400000ull;
	for (unsigned i = 0; i < 4; i++) {
		wlrx_sring_words(&rx, i, w);
		m_sring_want(i, want);
		CHECK(!memcmp(w, want, sizeof(w)), "SRING_SETUP words differ from the model's table");
	}
	wlrx_idle_regs(&rx, regs);
	{
		const uint32_t v[8] = { 0xa1400000u, 0x01000000u, 2u, 0x01000002u, 0u, 0u, 0u, 0x43u };
		for (unsigned i = 0; i < 8; i++) { CHECK(regs[i][0] == m_wbm_waddr[i] && regs[i][1] == v[i], "ring 120 register list"); }
	}
	CHECK(wlrx_fill_refill(&rx) == 0 && wlrx_link_fill(&rx) == 0, "fills");
	CHECK(g32(ra + 0x4000) == 0xa1a20000u && g32(ra + 0x4004) == (6u << 8) && g32(ra + 0x4000 + 8 * 4094) == 0xa1a20000u + 4094u * 0x1000u &&
	    g32(ra + 0x4000 + 8 * 4094 + 4) == (6u << 8 | 4094u << 12) && !g32(ra + 0x4000 + 8 * 4095), "first/last refill entry");
	CHECK(g32(la) == 0xa1440000u && g32(la + 4) == (1u << 8 | 0x5fff0u << 12) && g32(la + 8 * 32766) == 0xa1440000u + 32766u * 128u &&
	    g32(la + 8 * 32766 + 4) == (1u << 8 | (0x4000u << 3) << 12) && !g32(la + 8 * 32767), "first/last idle entry");
	CHECK(ra[0x20000] == 0xa5 && ra[0x20000 + 63] == 0xa5 && !ra[0x20000 + 64] && !ra[0x20000 + 2048] && !ra[0x20000 + 4095u * 4096u],
	    "sentinel");
	/* the guards and the gaps (2.7 "none"): the named pointer words and the rings are not gaps; a word past rdp, past
	 * wrp, at the first page's end, after the error ring and at the tail's very end are */
	{
		static const uint32_t named[] = { 4u * 160u, 4u * 161u, 4u * 169u, 4u * 186u, 0x400u, 0x404u, 0x424u, 0x468u, 0x4000u,
			0x1bffcu, 0x20000u };
		static const uint32_t gaps[] = { 4u * 170u, 0x33cu, 0x4bcu, 0x3ffcu, 0x1c000u, 0x1fffcu };
		uint32_t u, s, l, t, g;
		wlrx_guard_count(&rx, &u, &s, &l, &t, &g);
		CHECK(!u && !s && !l && !t && !g, "guards or gaps not clean after the fill");
		for (unsigned i = 0; i < sizeof(named) / sizeof(named[0]); i++) { ra[named[i] + 1u] ^= 0x10u; }
		for (unsigned i = 0; i < sizeof(gaps) / sizeof(gaps[0]); i++) { ra[gaps[i]] = 1; }
		ra[0x103fffcu] = 1;
		wlrx_guard_count(&rx, &u, &s, &l, &t, &g);
		CHECK(!u && !s && !l && t == 1 && g == 6, "gap count: a named word counted, or a gap or the tail's end missed");
		for (unsigned i = 0; i < sizeof(named) / sizeof(named[0]); i++) { ra[named[i] + 1u] ^= 0x10u; }
		for (unsigned i = 0; i < sizeof(gaps) / sizeof(gaps[0]); i++) { ra[gaps[i]] = 0; }
		ra[0x103fffcu] = 0;
	}
	/* watch arithmetic (2.8, D2) */
	memset(rx.w, 0, sizeof(rx.w)); rx.rx_on = rx.idle_on = true;
	rx.w[0].refill_tp = 4096; rx.w[0].mac_hp[0] = rx.w[0].mac_hp[1] = 2048; rx.w[0].idle_hp = 65534;
	wlrx_watch_derive(&rx, 0);
	CHECK(rx.w[0].refill_taken == 2048 && rx.w[0].refill_left == 2047 && rx.w[0].mac_avail[0] == 1024 && rx.w[0].kept == 0 &&
	    rx.w[0].kept_valid && rx.w[0].idle_avail == 32767 && !rx.w[0].exhausted && !rx.w[0].bad && !rx.w[0].unreadable, "setup derive");
	rx.w[1] = rx.w[0]; rx.w[1].mac_tp[0] = 2048; rx.w[1].err_hp = 8184; rx.w[1].idle_tp = 65534;
	wlrx_watch_derive(&rx, 1);
	CHECK(rx.w[1].exhausted == (WLRX_F_MAC0 | WLRX_F_ERR | WLRX_F_IDLE) && rx.w[1].err_fill == 1023, "mac0 drained, err full, idle empty");
	rx.w[1] = rx.w[0]; rx.w[1].err_hp = 8176;
	wlrx_watch_derive(&rx, 1);
	CHECK(!rx.w[1].exhausted && rx.w[1].err_fill == 1022, "err fill 1022 is not full");
	rx.w[0].kept = 1;  /* D2 baseline: the setup watch kept one buffer */
	rx.w[1] = rx.w[0]; rx.w[1].refill_tp = 8190; rx.w[1].mac_hp[0] = rx.w[1].mac_hp[1] = 4094;  /* all taken, all posted */
	wlrx_watch_derive(&rx, 1);
	CHECK(!rx.w[1].exhausted && rx.w[1].refill_empty && rx.w[1].refill_left == 0 && rx.w[1].kept == 1, "refill-empty note");
	rx.w[1].mac_hp[1] = 2048; rx.w[1].mac_tp[1] = 2048;  /* mac1 avail 0 too */
	wlrx_watch_derive(&rx, 1);
	CHECK(rx.w[1].exhausted == (WLRX_F_REFILL | WLRX_F_MAC1) && !rx.w[1].refill_empty, "refill left 0 with a MAC ring at 0");
	rx.w[1].mac_hp[1] = 2050; rx.w[1].mac_tp[1] = 0;  /* kept 1023 > the setup's 1 */
	wlrx_watch_derive(&rx, 1);
	CHECK(rx.w[1].exhausted == WLRX_F_REFILL && rx.w[1].kept == 1023, "refill left 0 with kept rising");
	/* D2: a MAC ring the firmware never used (hp == tp == 0) is not an exhausted one (2.8: avail 0 once its tp moved) */
	rx.w[0].kept = 2048;  /* the setup watch: all 4095 taken, 2047 posted to mac0, none to mac1 */
	rx.w[1] = rx.w[0]; rx.w[1].refill_tp = 8190; rx.w[1].mac_hp[0] = 4094; rx.w[1].mac_tp[0] = 26; rx.w[1].mac_hp[1] = rx.w[1].mac_tp[1] = 0;
	wlrx_watch_derive(&rx, 1);
	CHECK(!rx.w[1].exhausted && rx.w[1].refill_empty && rx.w[1].kept == 2048 && rx.w[1].mac_avail[0] == 2034,
	    "refill left 0 with a never-used MAC ring is the refill-empty note");
	rx.w[1] = rx.w[0]; rx.w[1].refill_tp = 8191; rx.w[1].mac_tp[1] = 4096; rx.w[1].err_hp = ~0u; rx.w[1].idle_tp = 3;
	wlrx_watch_derive(&rx, 1);
	CHECK(rx.w[1].bad == (WLRX_F_REFILL | WLRX_F_MAC1 | WLRX_F_IDLE) && rx.w[1].unreadable == WLRX_F_ERR && !rx.w[1].exhausted &&
	    !rx.w[1].kept_valid, "bad and unreadable words");
	/* D5 rows */
	memset(&rx.c, 0, sizeof(rx.c)); memset(rx.w, 0, sizeof(rx.w)); rx.rx_on = rx.idle_on = false;
	rx.idle_tried = true; rx.idle_own = WLRX_IDLE_SRNG;
	CHECK(!strncmp(wlrx_next(&rx, 0x200), "fallback r1c2c-a", 16), "firmware-owned list: fallback");
	rx.idle_own = WLRX_IDLE_AMBIGUOUS;
	CHECK(!strncmp(wlrx_next(&rx, 0x302), "none: owner review (WBM idle pre-read", 37), "ambiguous: owner review");
	rx.idle_tried = false; rx.chain_on = false;
	CHECK(!strncmp(wlrx_next(&rx, 0x402), "none: kit void", 14), "chain mask not sent");
	rx.chain_on = true;
	CHECK(!strncmp(wlrx_next(&rx, 0x402), "1.2 rows L3/L4", 14), "rx-frame 0 everywhere");
	rx.ci_rx_frame_max = 5;
	CHECK(!strncmp(wlrx_next(&rx, 0x402), "r1c2c-all", 9), "rx-frame above 0");
	rx.w[3].taken = true; rx.w[3].exhausted = WLRX_F_MAC0;
	CHECK(!strncmp(wlrx_next(&rx, 0x302), "the same kit with w20 0x10", 26) && !strncmp(wlrx_next(&rx, 0x312), "none: owner review (a second", 28),
	    "exhaustion: repeat with 0x10, then owner review");
	rx.w[3].exhausted = 0; rx.rx_on = true; rx.c.guard_tail = 1;
	CHECK(!strncmp(wlrx_next(&rx, 0x302), "none: stop", 10), "guard spill");
	/* the HTT type count, and nothing after the latch */
	rx.htt_t2h[0x24] = 0; wlrx_htt_count(&rx, (const uint8_t *)"\x24\0\0\0", 4); wlrx_htt_count(&rx, (const uint8_t *)"\x99", 1);
	CHECK(rx.htt_t2h[0x24] == 1 && rx.htt_t2h_other == 1, "HTT type count");
	rx.stopping = true;
	CHECK(wlrx_fill_refill(&rx) == 1 && wlrx_link_fill(&rx) == 1 && wlrx_publish_refill(&rx, &io) == 1, "host writes after the latch");
	free(ra); free(la);
	reset();
	puts("C2c pure: WBM allowlist = the model's (9 reads, 8 writes, no CE/TCL/REO register), D1 pre-read readings (SRNG_ENABLE, "
	     "base, ambiguous scatter, ~0), layout bounds, the four SRING_SETUP words, ring 120's 8 registers, first/last refill and "
	     "idle entries and the sentinel, guard and gap counts (named pointer words and rings excluded, the tail's end counted), "
	     "watch arithmetic (D2 refill-empty vs exhaustion, a never-used MAC ring), D5 rows, HTT type count, latch");
}

/* The happy 0x702 run: the firmware needs buffers and links before it forwards MGMT_RX; RXDMA receives 2 frames per
 * channel (mac0 on 2.4 GHz, mac1 on 5 GHz), each into a posted buffer with a link descriptor. */
static void
c2c_happy_run(unsigned w15)
{
	c2c_reset(0x702); wl15_variant = w15;
	C.rx_rule = 2; C.links_used = true; C.rx_frames = 2;
	C.setup_delay = 50;  /* the refill entries are taken 50 ms after the 4th SRING_SETUP: the setup watch must wait */
	happy_frames();
	CHECK(run() == 0, "C2c 0x702 happy path failed");
}

static void
s_c2c_happy(void)
{
	for (unsigned v = 0; v < 2; v++) {
		c2c_happy_run(v);
		show(v ? "0x702 w15=1" : "0x702");
		static const char *const lines[] = {
			"FUXI WLAN: W20 ce init ok rings 7 writes 70 window restore checked\n",
			"FUXI WLAN: W26 link arena IOVA 0xa1400000 bytes 0x440000 Normal-NC mapped\n",
			"FUXI WLAN: W26 rx arena IOVA 0xa1a00000 bytes 0x1040000 Normal-NC mapped; rdp 0xa1a00000 wrp 0xa1a00400\n",
			("FUXI WLAN: W26 wbm idle before: base-lsb 0 base-msb 0 misc 0 r2-hp 0 r2-tp 0 idle-list-cntl 0 idle-list-size 0 "
			 "scatter-base 0/0; not owned\n"),
			("FUXI WLAN: W26 wbm idle ring 120 base 0xa1400000 65536 words, 32767 link descriptors (rbm 1, our cookie) misc 0x43 "
			 "readback ok, hp 65534 via "),
			("FUXI WLAN: W26 refill ring 160 filled 4095 buffers (rbm 6, 2048 B at a 4096 B stride, sentinel 0xa5 x64, slots "
			 "0xa1a20000-0xa2a1e000) hp 8190 in wrp[0]\n"),
			"FUXI WLAN: W20 ce rx posted CE1 62 CE2 62\n",
			"FUXI WLAN: W23 htt version 3.4 (w20 0x02)\n",
			"FUXI WLAN: W26 htt sring setup refill 160 info0 0x0205000b info1 0x00022000 intr 0x003e0000 low 1024\n",
			"FUXI WLAN: W26 htt sring setup mac0 161 info0 0x0100010b info1 0x02021000 intr 0x003e0000 low 512\n",
			"FUXI WLAN: W26 htt sring setup mac1 169 info0 0x0100020b info1 0x02021000 intr 0x003e0000 low 512\n",
			"FUXI WLAN: W26 htt sring setup rxdma-err 186 info0 0x0007010b info1 0x00082000 intr 0x00200008 low 0\n",
			("FUXI WLAN: W26 rx watch setup+100ms refill tp 4096 taken 2048 left 2047 mac0 hp 2048 tp 0 avail 1024 mac1 hp 2048 tp 0 "
			 "avail 1024 err hp 0 fill 0; exhausted none\n"),
			"FUXI WLAN: W26 idle watch setup+100ms hp 65534 tp 0 avail 32767; exhausted none\n",
			"FUXI WLAN: W26 chain mask pdev 0 tx 0x3 rx 0x3 sent (PDEV_SET_PARAM 0x4003 ids 1, 2)\n",
			"FUXI WLAN: W23 chan list 38 passive",
			"FUXI WLAN: W23 scan started after ",
			"FUXI WLAN: W26 rx watch started refill tp 4096 taken 2048 left 2047 mac0 hp 2048 tp 0 avail 1024 ",
			"FUXI WLAN: W26 idle watch started hp 65534 tp 0 avail 32767; exhausted none\n",
			"FUXI WLAN: W26 rx watch mid (foreign 19) refill tp 4096 taken 2048 left 2047 mac0 hp 2048 tp 52 avail 998 ",
			"FUXI WLAN: W26 idle watch mid (foreign 19) ",
			"FUXI WLAN: W23 scan completed reason 0 after ",
			("FUXI WLAN: W26 rx watch completed refill tp 4096 taken 2048 left 2047 mac0 hp 2048 tp 52 avail 998 mac1 hp 2048 tp 100 "
			 "avail 974 err hp 0 fill 0; exhausted none\n"),
			"FUXI WLAN: W26 idle watch completed hp 65534 tp 152 avail 32691; exhausted none\n",
			"FUXI WLAN: W24 scan summary frames 12 bss 9 ssids 5\n",
			"FUXI WLAN: W23 stats after-scan wmi ",
			"FUXI WLAN: W26 rx content mac0 entries 1024 valid 1024 foreign 0 rbm-other 0 cookie-other 0\n",
			"FUXI WLAN: W26 rx content mac1 entries 1024 valid 1024 foreign 0 rbm-other 0 cookie-other 0\n",
			"FUXI WLAN: W26 rx content refill taken 2048 kept 0; buffers written 76 of 4095\n",
			"FUXI WLAN: W26 rx guard nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 0\n",
			"FUXI WLAN: W26 rxdma err entries 0 (hp 0) push err 0 routing 0 flush 0 other 0; rbm link 0 buf 0 other 0\n",
			"FUXI WLAN: W26 link content descriptors written 76 of 32767, unlisted slot clean; idle ring entries changed 0\n",
			"FUXI WLAN: W26 counters htt-t2h 1 htt-rx 1 wmi-events ",
			"FUXI WLAN: W26 htt-t2h types 0:1\n",
			"FUXI WLAN: W23 complete; stop/contain\n",
		};
		expect_order(lines, sizeof(lines) / sizeof(lines[0]));
		expect(v ? "hp 65534 via R2 0xa370b8 (fuxiwlanw15=1); writes 8 window restore checked\n" :
		    "hp 65534 via shadow 19 (0x948); writes 8 window restore checked\n");
		expect_not("W24 no beacon"); expect_not("TX COUNTERS"); expect_not("rxdma err codes"); expect_not("SPILL");
		CHECK(fw_sring_n == 4 && fw_pset_n == 2 && wbm_writes == 8 && wbm_pubs == 1 && fw_frames_written == 76 && fw_links == 76 &&
		    !fw_nobuf && wl_scan.frames == 12, "C2c firmware view");
		CHECK(fw_pset_w[0][1] == 0 && fw_pset_w[0][2] == 1 && fw_pset_w[0][3] == 3 && fw_pset_w[1][2] == 2, "PDEV_SET_PARAM words");
		CHECK(n_ev['L'] == n_ev['R'] || n_ev['R'] > n_ev['L'], "barriers");
		c2c_contained();
		static const char *const tail[] = {
			"FUXI WLAN: W23 complete; stop/contain\n",
			"FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo\n",
			"FUXI WLAN: W23 contain: scan over, no STOP_SCAN\n",
			"FUXI WLAN: W20 ce contain posted ",
			"FUXI WLAN: abort step 28 MHI reset done, bus master off yes, PERST asserted yes\n",
			"FUXI WLAN: W26 rx guard after containment nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 0\n",
			"FUXI WLAN: W26 arenas zeroed 0x1480000 bytes before free\n",
			"FUXI WLAN: abort buffers freed\n",
		};
		expect_order(tail, sizeof(tail) / sizeof(tail[0]));
	}
	puts("C2c 0x702 (shadow 19, then fuxiwlanw15=1 R2): link and RX arenas, pre-reads not owned, ring 120 in L19's order with "
	     "readback and the HP published after the entries, 4095 refill entries with sentinels and wrp[0], the four SRING_SETUP "
	     "word by word after VERSION_CONF, the chain masks before the channel list, four watches, 76 frames into buffers with "
	     "links, MGMT_RX forwarded, content/guard/err/link/counter lines, contained, recounted, zeroed and freed");
}

/* 0x200 alone: no HTT ring message, no RX arena, only ring 120 watches, the link content; per MGMT_RX rule. */
static void
s_c2c_idle(void)
{
	for (unsigned rule = 1; rule <= 5; rule++) {
		c2c_reset(0x200); C.rx_rule = rule; C.links_used = true; C.rx_frames = 1;
		add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
		CHECK(run() == 0, "C2c 0x200 failed");
		if (rule == 1) { show("0x200 needs links"); }
		static const char *const lines[] = {
			"FUXI WLAN: W26 link arena IOVA 0xa1400000 bytes 0x440000 Normal-NC mapped\n",
			"FUXI WLAN: W26 wbm idle before: ",
			"FUXI WLAN: W26 wbm idle ring 120 base 0xa1400000 ",
			"FUXI WLAN: W20 ce rx posted CE1 62 CE2 62\n",
			"FUXI WLAN: W22 complete; W23 scan next\n",
			"FUXI WLAN: W26 idle watch setup hp 65534 tp 0 avail 32767; exhausted none\n",
			"FUXI WLAN: W23 chan list 38 passive",
			"FUXI WLAN: W26 idle watch started hp 65534 tp 0 avail 32767; exhausted none\n",
			"FUXI WLAN: W26 idle watch mid (foreign 19) hp 65534 tp 36 avail 32749; exhausted none\n",
			"FUXI WLAN: W26 idle watch completed hp 65534 tp 76 avail 32729; exhausted none\n",
			"FUXI WLAN: W26 link content descriptors written 38 of 32767, unlisted slot clean; idle ring entries changed 0\n",
			"FUXI WLAN: W26 counters htt-t2h 0 htt-rx 0 wmi-events ",
			"FUXI WLAN: W26 htt-t2h types none\n",
		};
		expect_order(lines, sizeof(lines) / sizeof(lines[0]));
		expect_not("W26 rx "); expect_not("sring setup"); expect_not("chain mask"); expect_not("W23 htt version");
		CHECK(!rx_allocs && ln_allocs == 1 && !fw_sring_n && !fw_pset_n && fw_links == 38 && wl_htc.htt_fn == wl23_htt_rx,
		    "0x200: no RX arena, no ring message, links taken, HTT types counted");
		if (rule == 1) {
			expect("FUXI WLAN: W24 scan summary frames 1 bss 1 ssids 1\n");
			expect_not("W24 no beacon");
		} else {
			expect("FUXI WLAN: W24 scan summary frames 0 bss 0 ssids 0\n");
			static const char *const hint[] = {
				"FUXI WLAN: W26 htt-t2h types none\n",
				("FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung r1c2c w20 0x302 (770) (links used, "
				 "nothing forwarded: H1b or H4)\n"),
				"FUXI WLAN: W23 complete; stop/contain\n",
			};
			expect_order(hint, sizeof(hint) / sizeof(hint[0]));
		}
		c2c_contained();
		expect("FUXI WLAN: W26 arenas zeroed 0x440000 bytes before free\n");
		expect_not("W26 rx guard after containment");
	}
	/* links never used: "links untouched" */
	c2c_reset(0x200); C.rx_rule = 5;
	CHECK(run() == 0, "C2c 0x200 untouched failed");
	expect("next rung r1c2c w20 0x302 (770) (links untouched: RXDMA pushed nothing)\n");
	c2c_contained();
	puts("C2c 0x200 alone, each MGMT_RX rule (needs links: forwarded; buffers and links, buffers, chain masks, never: not): no "
	     "RX arena, no HTT ring message, ring 120 watches only, link content, HTT types counted without w20 0x02, the D5 hint "
	     "r1c2c 0x302 (links used / untouched), arena zeroed and freed");
}

/* D1: a refusing pre-read writes nothing to the WBM, drops 0x200 for the boot, the run goes on (with 0x302 the RX part
 * still runs). */
static void
s_c2c_preread(void)
{
	static const struct { unsigned i; uint32_t v; const char *why, *next; } cases[] = {
		{2, 0x40, "SRNG_ENABLE set (firmware owns the list)", "fallback r1c2c-a w20 0x102 (258), then 0x502 (1282) (the firmware owns the WBM idle list)"},
		{0, 0x1234000, "base non-zero (firmware owns the list)", "fallback r1c2c-a w20 0x102 (258)"},
		{5, 2, "scatter mode set with base 0 and SRNG_ENABLE 0 (AMBIGUOUS)", "none: owner review (WBM idle pre-read ambiguous or unreadable)"},
		{7, ~0u, "a pre-read is ~0 (unreadable, ownership not established)", "none: owner review (WBM idle pre-read ambiguous"},
	};
	for (unsigned k = 0; k < 8; k++) {
		char want[256];
		const unsigned i = k % 4, rings = k >= 4;
		c2c_reset(rings ? 0x302 : 0x200); C.rx_rule = 2; C.links_used = true; C.rx_frames = 1;
		C.wbm_pre[cases[i].i] = cases[i].v;
		if (cases[i].i == 5) { C.wbm_pre[6] = 0x123; C.wbm_pre[7] = 0x8000; }  /* list size and a scattered base are printed only */
		CHECK(run() == 0, "C2c pre-read refusal failed the run");
		if (k == 2) { show("0x200 ambiguous"); }
		snprintf(want, sizeof(want), "FUXI WLAN: W26 wbm idle list not programmed: %s; w20 0x200 dropped for this boot\n", cases[i].why);
		expect(want);
		expect("; refused\n");
		snprintf(want, sizeof(want), "next rung %s", cases[i].next);
		expect(want);
		CHECK(wbm_reads == 9 && !wbm_writes && !wbm_pubs && !idle_pub, "a WBM write after a refusing pre-read");
		expect_not("W26 idle watch"); expect_not("W26 link content"); expect_not("W26 wbm idle ring 120");
		CHECK(rings ? fw_sring_n == 4 && count_text("W26 rx watch") == 4 && rx_allocs == 1 : !fw_sring_n, "the RX part with 0x302");
		c2c_contained();
	}
	expect("FUXI WLAN: W26 wbm idle before: base-lsb 0 base-msb 0 misc 0 r2-hp 0 r2-tp 0 idle-list-cntl 0 idle-list-size 0 "
	       "scatter-base 0xffffffff/0; refused\n");
	puts("C2c pre-reads (D1): SRNG_ENABLE, a ring base, scatter mode with base 0 (AMBIGUOUS), a ~0 read: nine reads, no WBM write, "
	     "0x200 dropped, run goes on (0x302: rings and buffers still), fallback or owner-review hint");
}

/* 0x302 with the needs-links rule, 0x402 with needs-chain-masks, chain masks 0, 0x480 (EXT pdev). */
static void
s_c2c_bits(void)
{
	c2c_reset(0x302); C.rx_rule = 1; C.links_used = true; C.rx_frames = 1;
	happy_frames();
	CHECK(run() == 0, "C2c 0x302 failed");
	expect("FUXI WLAN: W24 scan summary frames 12 bss 9 ssids 5\n");
	expect_not("chain mask"); expect_not("W24 no beacon");
	c2c_contained();
	c2c_reset(0x402); C.rx_rule = 4;
	happy_frames();
	CHECK(run() == 0, "C2c 0x402 failed");
	show("0x402");
	static const char *const m[] = {
		"FUXI WLAN: W23 htt version 3.4 (w20 0x02)\n",
		"FUXI WLAN: W26 chain mask pdev 0 tx 0x3 rx 0x3 sent (PDEV_SET_PARAM 0x4003 ids 1, 2)\n",
		"FUXI WLAN: W23 chan list 38 passive",
		"FUXI WLAN: W24 scan summary frames 12 bss 9 ssids 5\n",
		"FUXI WLAN: W26 counters htt-t2h 1 htt-rx 1 ",
		"FUXI WLAN: W26 htt-t2h types 0:1\n",
		"FUXI WLAN: W23 complete; stop/contain\n",
	};
	expect_order(m, sizeof(m) / sizeof(m[0]));
	expect_not("W26 rx"); expect_not("W26 idle"); expect_not("W26 link"); expect_not("W26 arenas");
	CHECK(fw_pset_n == 2 && !rx_allocs && !ln_allocs, "0x402: two PDEV_SET_PARAM, no arena");
	c2c_contained();
	/* the needs-chain-masks firmware without them: no MGMT_RX, hint per the readout */
	c2c_reset(0x002); C.rx_rule = 4;
	happy_frames();
	CHECK(run() == 0, "C2c 0x002 needs-chain-masks failed");
	expect("FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung C2c = RX data path\n");
	expect_not("W26");
	c2c_contained();
	/* masks 0: nothing sent, a line, the WMI command stream otherwise the same as without 0x400 */
	{
		uint32_t a[64], b[64];
		unsigned na, nb;
		c2c_reset(0x002); C.phy_bands = 0;
		CHECK(run() == 0, "0x002 without caps");
		memcpy(a, fw_cmd_log, sizeof(a)); na = fw_cmd_n;
		c2c_contained();
		c2c_reset(0x402); C.phy_bands = 0;
		CHECK(run() == 0, "0x402 without caps");
		memcpy(b, fw_cmd_log, sizeof(b)); nb = fw_cmd_n;
		expect("FUXI WLAN: W26 chain mask not sent: no MAC/PHY caps (tx 0 rx 0)\n");
		expect("next rung none: kit void, owner review (chain mask not sent: no MAC/PHY caps)\n");
		CHECK(na == nb && !memcmp(a, b, sizeof(uint32_t) * na) && !fw_pset_n, "0x400 without masks changed the command stream");
		c2c_contained();
	}
	/* 0x480: the EXT MAC/PHY pdev, as VDEV_CREATE */
	c2c_reset(0x480); C.exp_pdev = 1;
	CHECK(run() == 0, "C2c 0x480 failed");
	expect("FUXI WLAN: W26 chain mask pdev 1 tx 0x3 rx 0x3 sent (PDEV_SET_PARAM 0x4003 ids 1, 2)\n");
	CHECK(fw_pset_w[0][1] == 1 && fw_pset_w[1][1] == 1, "0x480 PDEV_SET_PARAM pdev");
	c2c_contained();
	/* the PHYs' 1x1 masks with SERVICE_READY's 2 rf chains: the masks are the preferred hw mode's, not (1 << chains) - 1 */
	c2c_reset(0x400); C.phy_mask = 1;
	CHECK(run() == 0, "C2c 0x400 1x1 failed");
	expect("FUXI WLAN: W26 chain mask pdev 0 tx 0x1 rx 0x1 sent (PDEV_SET_PARAM 0x4003 ids 1, 2)\n");
	CHECK(fw_pset_w[0][3] == 1 && fw_pset_w[1][3] == 1, "1x1 chain masks");
	c2c_contained();
	/* tx mask 0x3, rx mask 0x1: each PDEV_SET_PARAM carries its own (TX 1 = tx, RX 2 = rx), VDEV_CREATE the streams */
	c2c_reset(0x400); C.phy_rx_mask = 1;
	CHECK(run() == 0, "C2c 0x400 tx 3 rx 1 failed");
	expect("FUXI WLAN: W26 chain mask pdev 0 tx 0x3 rx 0x1 sent (PDEV_SET_PARAM 0x4003 ids 1, 2)\n");
	expect("streams tx 2 rx 1\n");
	CHECK(fw_pset_w[0][2] == 1 && fw_pset_w[0][3] == 3 && fw_pset_w[1][2] == 2 && fw_pset_w[1][3] == 1, "tx 3 / rx 1 chain masks");
	c2c_contained();
	puts("C2c bits: 0x302 forwards with links (needs-links firmware), 0x402 two PDEV_SET_PARAM before the channel list and "
	     "forwards (needs-chain-masks firmware; without them C2b's hint), masks 0 send nothing and leave the WMI stream as "
	     "without 0x400, 0x480 the EXT pdev, tx 0x3 with rx 0x1 each in its own parameter");
}

/* Readings: firmware ignores the rings, a foreign MAC entry, spills, a write at offset 32, kept > 0, RBM/cookie others,
 * exhaustion (mac0 drained before the mid watch; err ring full; refill-empty note; a second inconclusive with 0x10),
 * bad pointer words, R2 unreadable, the D4 TX line. */
static void
s_c2c_readings(void)
{
	c2c_reset(0x302); C.rx_rule = 5; C.rx_take = 0; C.rx_post[0] = C.rx_post[1] = 0;
	CHECK(run() == 0, "rings ignored");
	expect("FUXI WLAN: W26 rx watch completed refill tp 0 taken 0 left 4095 mac0 hp 0 tp 0 avail 0 mac1 hp 0 tp 0 avail 0 err hp 0 "
	       "fill 0; exhausted none\n");
	expect("next rung C2d step 1 after reading the W26 htt-t2h types (the firmware ignored the rings)\n");
	c2c_contained();
	/* foreign MAC entries, by address alone: 64 bytes into a slot, never-posted slot 4095's start, a posted slot's start
	 * with address bits 39:32 set; counted, stop */
	c2c_reset(0x302); C.rx_rule = 5; C.mac_foreign = 3; C.rx_frames = 1;
	CHECK(run() == 0, "foreign");
	expect("FUXI WLAN: W26 rx content mac0 entries 1027 valid 1024 foreign 3 rbm-other 0 cookie-other 0\n");
	expect("next rung none: stop, owner review before any further kit (guard spill or foreign MAC entry)\n");
	c2c_contained();
	/* spills into a slot's upper half, slot 4095, the tail (its start and its end) and the gaps (rdp[170], after the
	 * error ring): counted, SPILL, normal containment; the D3 recount */
	c2c_reset(0x302); C.rx_rule = 5; C.spill = 31; C.rx_frames = 1;
	CHECK(run() == 0, "spill");
	show("spill");
	expect("FUXI WLAN: W26 rx guard SPILL nonzero dwords: slot-upper 4 (slots 1), slot 4095 2, tail 5, gaps 4; stop; owner review "
	       "before any further kit\n");
	expect("next rung none: stop, owner review before any further kit (guard spill or foreign MAC entry)\n");
	c2c_contained();
	expect("FUXI WLAN: W26 rx guard after containment nonzero dwords: slot-upper 4 (slots 1), slot 4095 2, tail 5, gaps 4; SPILL: "
	       "owner review before any further kit\n");
	/* the gaps alone (an error ring overrun past entry 1023, a stray rdp write): SPILL and stop as well */
	c2c_reset(0x302); C.rx_rule = 5; C.spill = 16; C.rx_frames = 1;
	CHECK(run() == 0, "gap spill");
	expect("FUXI WLAN: W26 rx guard SPILL nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 4; stop; owner review "
	       "before any further kit\n");
	expect("next rung none: stop, owner review before any further kit (guard spill or foreign MAC entry)\n");
	c2c_contained();
	expect("FUXI WLAN: W26 rx guard after containment nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 4; SPILL: "
	       "owner review before any further kit\n");
	/* a frame that writes bytes 32..35 only: written */
	c2c_reset(0x302); C.rx_rule = 5; C.write32 = true; C.rx_frames = 1;
	CHECK(run() == 0, "write32");
	expect("FUXI WLAN: W26 rx content refill taken 2048 kept 0; buffers written 38 of 4095\n");
	expect("next rung C2d step 1, then the H8 pdev-parameter rung (frames landed, not forwarded)\n");
	c2c_contained();
	/* kept > 0; RBM 4 and another cookie (counted, valid, no stop) */
	c2c_reset(0x302); C.rx_rule = 5; C.rx_take = 3000; C.mac_rbm = 4; C.mac_cookie_xor = 1;
	CHECK(run() == 0, "kept, rbm, cookie");
	expect("FUXI WLAN: W26 rx content mac0 entries 1024 valid 1024 foreign 0 rbm-other 1024 cookie-other 1024\n");
	expect("FUXI WLAN: W26 rx content refill taken 3000 kept 952; buffers written 0 of 4095\n");
	expect("next rung r1c2c-m w20 0x402 (1026) if not yet booted, else BDF/calibration (buffers placed, nothing written)\n");
	c2c_contained();
	/* mac0 drained before the mid watch (100 buffers, 120 frames on channel 0): exhaustion, inconclusive */
	c2c_reset(0x302); C.rx_rule = 5; C.rx_post[0] = 100; C.rx_frames_ch0 = 120;
	CHECK(run() == 0, "mac0 drained");
	show("mac0 drained");
	expect("FUXI WLAN: W26 rx watch mid (foreign 19) refill tp 4096 taken 2048 left 2047 mac0 hp 200 tp 200 avail 0 mac1 hp 2048 tp 0 "
	       "avail 1024 err hp 0 fill 0; exhausted mac0\n");
	expect("next rung the same kit with w20 0x10 added (an exhaustion flag: inconclusive, nothing killed)\n");
	c2c_contained();
	/* the err ring full (1023 entries, codes counted), and a second inconclusive run with 0x10 */
	c2c_reset(0x312); C.exp_n = 13; C.rx_rule = 5; C.rx_frames = 1; C.err_n = 1100; C.err_code = 0; C.err_rbm = 1;
	CHECK(run() == 0, "err full");
	show("err full");
	expect("FUXI WLAN: W26 rx watch completed ");
	expect("err hp 8184 fill 1023; exhausted err\n");
	expect("FUXI WLAN: W26 rxdma err entries 1023 (hp 8184) push err 1023 routing 0 flush 0 other 0; rbm link 1023 buf 0 other 0\n");
	expect("FUXI WLAN: W26 rxdma err codes overflow 1023\n");
	/* w5 keeps DEST_IND without the frame's byte count; w6 the push reason, code and SW_EXCEPTION_RING_VLD without the
	 * frame's fragment and sequence numbers (no_frame_text checks that neither raw word appears) */
	expect("FUXI WLAN: W26 rxdma err entry 3 w1 0x100 w5 0xc00000 w6 0x2000 w7 0x700000\n");
	expect_not("rxdma err entry 4 ");
	expect("next rung none: owner review (a second inconclusive run, w20 0x10 already set)\n");
	c2c_contained();
	/* D2: every refill entry taken and posted (2047 + 2047, kept 1): refill-empty, not exhaustion */
	c2c_reset(0x302); C.rx_rule = 5; C.rx_take = 4095; C.rx_post[0] = C.rx_post[1] = 2047; C.rx_frames = 1;
	CHECK(run() == 0, "refill-empty");
	expect("FUXI WLAN: W26 rx watch setup+100ms refill tp 8190 taken 4095 left 0 mac0 hp 4094 tp 0 avail 2047 mac1 hp 4094 tp 0 "
	       "avail 2047 err hp 0 fill 0; exhausted none; refill-empty\n");
	expect_not("exhausted refill");
	c2c_contained();
	/* the firmware takes more during the scan and keeps them: refill left 0 with kept rising = exhaustion */
	c2c_reset(0x302); C.rx_rule = 5; C.rx_take = 4000; C.rx_post[0] = C.rx_post[1] = 2000; C.take_more = 95;
	CHECK(run() == 0, "kept rising");
	expect("FUXI WLAN: W26 rx watch completed refill tp 8190 taken 4095 left 0 ");
	expect("; exhausted refill\n");
	c2c_contained();
	/* D2, the hoarding case: every refill entry taken, 2047 posted to mac0, mac1 never posted (hp == tp == 0, as a 2.4 GHz
	 * scan may leave it): mac1 unused is not an exhausted MAC ring, so refill left 0 is the refill-empty note, no flag */
	c2c_reset(0x302); C.rx_rule = 5; C.rx_take = 4095; C.rx_post[0] = 2047; C.rx_post[1] = 0; C.rx_frames = 1;
	CHECK(run() == 0, "refill-empty, mac1 unused");
	show("mac1 unused");
	expect("FUXI WLAN: W26 rx watch completed refill tp 8190 taken 4095 left 0 mac0 hp 4094 tp 26 avail 2034 mac1 hp 0 tp 0 avail 0 "
	       "err hp 0 fill 0; exhausted none; refill-empty\n");
	expect_not("exhausted refill"); expect_not("w20 0x10 added");
	c2c_contained();
	/* the same with mac0 drained (100 posted, 120 frames on channel 0): an exhausted MAC ring, so refill left 0 flags too */
	c2c_reset(0x302); C.rx_rule = 5; C.rx_take = 4095; C.rx_post[0] = 100; C.rx_post[1] = 0; C.rx_frames_ch0 = 120;
	CHECK(run() == 0, "refill exhausted with mac0 drained");
	expect("FUXI WLAN: W26 rx watch mid (foreign 19) refill tp 8190 taken 4095 left 0 mac0 hp 200 tp 200 avail 0 mac1 hp 0 tp 0 "
	       "avail 0 err hp 0 fill 0; exhausted refill mac0\n");
	expect("next rung the same kit with w20 0x10 added (an exhaustion flag: inconclusive, nothing killed)\n");
	c2c_contained();
	/* bad words: rdp[186] beyond the ring (clamped, bad), refill tp backwards then past the HP (printed raw, never acted on),
	 * R2 unreadable at the watches (run continues) */
	c2c_reset(0x702); C.rx_rule = 5; C.rdp_err_raw = 9000; C.refill_tp_raw[0] = 1000; C.refill_tp_raw[1] = 8193; C.r2_unreadable = true;
	CHECK(run() == 0, "bad words");
	show("bad words");
	expect("FUXI WLAN: W26 rx watch mid (foreign 19) refill tp 1000 taken 500 left 3595 ");
	expect("FUXI WLAN: W26 rx watch completed refill tp 8193 bad mac0 hp 2048 tp 0 avail 1024 mac1 hp 2048 tp 0 avail 1024 err hp 9000 "
	       "bad; exhausted none\n");
	expect("FUXI WLAN: W26 idle watch completed hp 4294967295 tp 4294967295 unreadable; exhausted none\n");
	expect("FUXI WLAN: W26 rxdma err entries 0 (hp 9000 bad) ");
	expect("FUXI WLAN: W26 rx content refill taken unknown (no usable completed watch); buffers written 0 of 4095\n");
	expect("next rung none matched: owner review (rx_path_plan.md 2.11)\n");
	CHECK(wbm_watch_reads == 8, "R2 read twice per watch");
	c2c_contained();
	/* D4: a non-zero CHAN_INFO tx-frame and stats tx counters on a W26 boot: a reading line, the run contains normally */
	c2c_reset(0x402); C.ci_tx = 7; C.stats_mode = 5;
	CHECK(run() == 0, "TX counters");
	expect("FUXI WLAN: W26 TX COUNTERS NON-ZERO: owner review (chan-info tx-frame events 38; stats answers 2 with local-enqued 2 "
	       "hw-queued 2 mpdu-enqued 2 hw-reaped 2 base tx-frame 2)\n");
	c2c_contained();
	/* D4: only the pdev stats base tx-frame (the pdev-wide twin of CHAN_INFO's) non-zero */
	c2c_reset(0x402); C.stats_tx_frame = 9;
	CHECK(run() == 0, "base tx-frame");
	expect("FUXI WLAN: W26 TX COUNTERS NON-ZERO: owner review (chan-info tx-frame events 0; stats answers 2 with local-enqued 0 "
	       "hw-queued 0 mpdu-enqued 0 hw-reaped 0 base tx-frame 2)\n");
	c2c_contained();
	c2c_reset(0x002); C.ci_tx = 7; C.stats_mode = 5;  /* without a W26 bit: C2b's lines only */
	CHECK(run() == 0, "TX counters without W26");
	expect_not("W26");
	c2c_contained();
	puts("C2c readings: rings ignored, foreign MAC entries by address (64 bytes in, slot 4095, bits 39:32; stop), spills "
	     "(slot halves, slot 4095, both ends of the tail, the gaps; SPILL, D3 recount after containment), a write at offset 32 "
	     "(written), kept 952 with RBM 4 and other cookies (counted, valid), mac0 drained before mid and the err ring full "
	     "(exhaustion; a second inconclusive with 0x10; error entries printed without their frame fields), D2 refill-empty vs "
	     "kept rising and vs a never-used MAC ring, bad and unreadable words, the D4 TX line (with the base tx-frame alone); "
	     "each with its D5 hint");
}

static void
fails_c2c(const char *why)
{
	CHECK(run() == 1, why);
	c2c_contained();
}

static void
s_c2c_failures(void)
{
	/* RX arena allocation failure: no HTT ring message, no WBM write (the link arena came first), contain */
	c2c_reset(0x302); C.rx_alloc_fail = true;
	fails_c2c("RX arena allocation failure accepted");
	expect("FUXI WLAN: W20 failure: W26 rx arena allocation/NC/map; contain, no retry\n");
	CHECK(!wbm_reads && !wbm_writes && !fw_sring_n && ln_allocs == 1, "WBM or ring message after an RX arena failure");
	/* link arena map failure: no WBM access at all */
	c2c_reset(0x200); C.link_map_fail = true;
	fails_c2c("link arena map failure accepted");
	expect("FUXI WLAN: W20 failure: target DMA leaf readback; contain, no retry\n");
	CHECK(!wbm_reads && !wbm_writes, "WBM access after a link arena failure");
	/* readback mismatch on BASE_LSB and on MISC: no HP publish */
	c2c_reset(0x200); C.wbm_rb_bad = 1;
	fails_c2c("BASE_LSB mismatch accepted");
	expect("FUXI WLAN: W26 wbm idle readback mismatch base-lsb wrote 0xa1400000 read 0xa1401000\n");
	CHECK(wbm_writes == 8 && !wbm_pubs && !idle_pub, "HP after a BASE_LSB mismatch");
	c2c_reset(0x200); C.wbm_rb_bad = 2;
	fails_c2c("MISC mismatch accepted");
	expect("FUXI WLAN: W26 wbm idle readback mismatch misc wrote 0x43 read 0x3\n");
	CHECK(!wbm_pubs, "HP after a MISC mismatch");
	/* a fault latched after the 3rd WBM write: no later write */
	c2c_reset(0x200); C.wbm_fault_after = 3;
	fails_c2c("WBM fault accepted");
	expect("FUXI WLAN: W26 wbm idle write 3 of 8 (0xa34d44) failed; contain\n");  /* the faulting write itself reports */
	CHECK(wbm_writes == 3 && !wbm_pubs, "a WBM write after the latched fault");
	/* 16 CE4 sends in flight at the first SRING_SETUP: not sent, contain */
	c2c_reset(0x302); C.ce4_inflight = 1;
	fails_c2c("CE4 in flight accepted");
	expect("FUXI WLAN: W26 htt sring setup refill 160 not sent: CE4 slots all in flight\n");
	CHECK(!fw_sring_n, "an SRING_SETUP with 16 CE4 slots in flight");
	/* HTT major 2: no ring message */
	c2c_reset(0x302); C.htt_major = 2;
	fails_c2c("HTT major 2 accepted");
	CHECK(!fw_sring_n && !fw_pset_n, "a ring message after HTT major 2");
	/* the W11 guard fails once the firmware took the 2nd SRING_SETUP: no further message */
	c2c_reset(0x702); C.guard_on_sring = 2;
	fails_c2c("guard between the SRING_SETUPs accepted");
	expect("FUXI WLAN: W23 failure: AMSS/M0, ownership, discovery or access/SMMU fault; contain, no retry\n");
	CHECK(fw_sring_n == 2 && !fw_pset_n, "a message after the guard failure");
	/* the W11 guard fails in the post-scan stats exchange of a 0x302 boot with CHAN_INFO tx-frame non-zero: the W26
	 * content, guard, error, counter, D4 and D5 lines still print (host reads), after the failure line, before containment */
	c2c_reset(0x302); C.rx_rule = 5; C.rx_frames = 1; C.ci_tx = 7; C.guard_on_stats = true;
	CHECK(run() == 1, "a guard failure in the stats exchange accepted");
	show("stats guard");
	{
		static const char *const lines[] = {
			"FUXI WLAN: W23 stats request pdev-stat pdev 1 bands 0x1 vdev 0 sent\n",
			"FUXI WLAN: W23 failure: AMSS/M0, ownership, discovery or access/SMMU fault; contain, no retry\n",
			"FUXI WLAN: W26 rx content mac0 entries 1024 valid 1024 foreign 0 rbm-other 0 cookie-other 0\n",
			"FUXI WLAN: W26 rx content refill taken 2048 kept 0; buffers written 38 of 4095\n",
			"FUXI WLAN: W26 rx guard nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 0\n",
			"FUXI WLAN: W26 rxdma err entries 0 (hp 0) ",
			"FUXI WLAN: W26 counters htt-t2h 1 ",
			"FUXI WLAN: W26 TX COUNTERS NON-ZERO: owner review (chan-info tx-frame events 38; stats answers 0 ",
			("FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung C2d step 1, then the H8 pdev-parameter "
			 "rung (frames landed, not forwarded)\n"),
		};
		expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	}
	expect_not("W23 complete"); expect_not("W23 stats after-scan");
	c2c_contained();
	/* no COMPLETED on a 0x302 boot: the W26 lines before the snapshot and the failure, then normal containment */
	c2c_reset(0x302); C.rx_rule = 5; C.rx_frames = 1; C.scan_mode = 3;
	CHECK(run() == 1, "no COMPLETED accepted");
	show("no COMPLETED");
	{
		static const char *const lines[] = {
			"FUXI WLAN: W23 timeout scan COMPLETED ",
			"FUXI WLAN: W26 rx watch completed refill tp 4096 taken 2048 left 2047 ",
			"FUXI WLAN: W26 rx content mac0 entries 1024 valid 1024 foreign 0 rbm-other 0 cookie-other 0\n",
			"FUXI WLAN: W26 rx guard nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 0\n",
			"FUXI WLAN: W26 counters htt-t2h 1 ",
			"FUXI WLAN: W20 snapshot scan COMPLETED: ",
			"FUXI WLAN: W23 failure: scan COMPLETED; contain, no retry\n",
		};
		expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	}
	c2c_contained();
	puts("C2c failures: RX arena allocation (no WBM write, no ring message), link arena map (no WBM access), BASE_LSB and MISC "
	     "readback mismatch (no HP), a fault after the 3rd WBM write (no later write), CE4 slots in flight, HTT major 2 and a "
	     "guard failure after the 2nd SRING_SETUP (no further message), a guard failure in the stats exchange and no COMPLETED "
	     "(the W26, D4 and D5 lines still before containment); every run contained, zeroed and freed");
}

/* Containment: unproven (both arenas quarantined, not zeroed), proven with RXDMA writing until bus master off (zeroed
 * after it, freed); after wl26_contain no host write to either arena and no WBM access. */
static void
s_c2c_contain(void)
{
	c2c_reset(0x702); C.rx_rule = 2; C.links_used = true; C.rx_frames = 1; C.fail_bm = true;
	CHECK(run() == 0, "C2c unproven containment run");
	contain();
	expect("FUXI WLAN: abort step 28 MHI reset done, bus master off unconfirmed, PERST asserted yes\n");
	expect("FUXI WLAN: abort buffers quarantined until reboot");
	expect_not("W26 arenas zeroed"); expect_not("guard after containment");
	CHECK(wl_rx_arena.va && wl_link_arena.va && !rx_frees && !ln_frees && rx_host[0x20000u + 0x1000u * 100u] == 0xa5,
	    "W26 arenas freed or zeroed although bus master off is unproven");
	c2c_reset(0x702); C.rx_rule = 2; C.links_used = true; C.rx_frames = 1; C.late_write = true;
	CHECK(run() == 0, "C2c late write run");
	c2c_contained();  /* wl_dma_free: every byte zero, after bus master off, PERST, SMMU undo */
	c2c_reset(0x702); C.rx_rule = 2; C.links_used = true; C.rx_frames = 1;
	CHECK(run() == 0, "C2c latch run");
	wl_ce_contain(28);
	{
		uint32_t v = 0x1234;
		const unsigned wb = wbm_reads + wbm_writes + wbm_watch_reads;
		CHECK(wlrx_fill_refill(&wl_rx) == 1 && wlrx_link_fill(&wl_rx) == 1 && wlrx_publish_refill(&wl_rx, &wl_ce_io) == 1,
		    "a W26 host write after the latch");
		CHECK(wl26_wbm(M_IDLE_HP, true, &v) == 1 && wl26_wbm(M_IDLE_HP, false, &v) == 1 && wbm_reads + wbm_writes + wbm_watch_reads == wb,
		    "a WBM access after the latch");
		wl26_watch(WLRX_P_COMPLETED);
		CHECK(wbm_reads + wbm_writes + wbm_watch_reads == wb, "a watch after the latch");
	}
	c2c_contained();
	puts("C2c containment: unproven keeps both arenas (not zeroed, not freed); proven zeroes them after bus master off, PERST "
	     "and the SMMU undo although RXDMA wrote until bus master off, then frees them; after wl26_contain no fill, publish or "
	     "WBM access");
}

/* Every W26 line at its widest: ~0u and out-of-ring words, every exhaustion flag, the widest label, all 19 error codes and
 * the 64 HTT types at 0xffffffff, the widest pre-read and ring 120 lines; model_line fails any line past 254 bytes. */
static void
s_c2c_widest(void)
{
	uint32_t big[65];
	for (unsigned i = 0; i < 65; i++) { big[i] = 0xffffffffu; }
	c2c_happy_run(0);
	wl_scan.foreign = 0xffffffffu;
	for (unsigned k = 0; k < 3; k++) {  /* 0: every word ~0; 1: every word bad; 2: every flag (refill, mac0, mac1, err, idle) */
		struct wlrx_watch *w = &wl_rx.w[WLRX_P_MID];
		w->taken = false;
		rx_inject = true;
		p32(rxd(4u * 160u, 4, true, "w"), k == 0 ? ~0u : k == 1 ? 0xfffffffeu : 8190u);
		p32(rxd(0x400u + 4u, 4, true, "w"), k == 0 ? ~0u : k == 1 ? 0xfffffffeu : 4094u);
		p32(rxd(4u * 161u, 4, true, "w"), k == 2 ? 4094u : 0xfffffffdu);
		p32(rxd(0x400u + 36u, 4, true, "w"), k == 2 ? 4094u : k == 1 ? 0xfffffffeu : ~0u);
		p32(rxd(4u * 169u, 4, true, "w"), k == 2 ? 4094u : k == 1 ? 0xfffffffeu : ~0u);
		p32(rxd(4u * 186u, 4, true, "w"), k == 0 ? ~0u : k == 1 ? 0xfffffffeu : 8184u);
		rx_inject = false;
		C.r2_unreadable = k == 0;
		if (k == 2) { m_idle_tp = m_idle_hp; }
		wl26_watch(WLRX_P_MID);
	}
	expect("FUXI WLAN: W26 idle watch mid (foreign 4294967295) hp 65534 tp 65534 avail 0; exhausted idle\n");
	wl26_list("W26 rxdma err codes", wl26_err_code, big, 19, 0xffffffffu);
	wl26_list("W26 htt-t2h types", NULL, big, 64, 0xffffffffu);
	CHECK(count_text("FUXI WLAN: W26 rxdma err codes") >= 3 && count_text("FUXI WLAN: W26 htt-t2h types") >= 4, "list lines");
	expect("FUXI WLAN: W26 rx watch mid (foreign 4294967295) refill tp unreadable mac0 hp 4294967295 tp 4294967293 unreadable ");
	expect("; exhausted refill mac0 mac1 err\n");
	c2c_contained();
	c2c_reset(0x200);
	for (unsigned i = 0; i < 9; i++) { C.wbm_pre[i] = 0xfffffffeu; }
	C.wbm_pre[5] = 2; C.wbm_pre[2] = 0; C.wbm_pre[0] = 0; C.wbm_pre[1] = 0;
	CHECK(run() == 0, "widest pre-read");
	expect("scatter-base 0xfffffffe/0xfffffffe; refused\n");
	c2c_contained();
	puts("C2c line widths: watch lines with unreadable, bad and widest words and every flag, the widest label, 19 error codes "
	     "and 64 HTT types at 0xffffffff over several lines, the widest pre-read line: every FUXI WLAN line within 254 bytes");
}

static void
s_c2c_none(void)
{
	/* every qrtr 7 boot without a W26 bit: no W26 line, no W26 arena, the HTT handler as C2 */
	static const unsigned bits[] = { 0x0, 0x2, 0x10, 0x80, 0x92, 0x42 };
	for (unsigned i = 0; i < sizeof(bits) / sizeof(bits[0]); i++) {
		c2c_reset(bits[i]); C.exp_n = bits[i] & 0x10 ? 13u : 38u; C.exp_pdev = bits[i] & 0x80 ? 1u : 0u;
		happy_frames();
		CHECK(run() == 0, "C2 run");
		expect_not("W26");
		CHECK(!rx_allocs && !ln_allocs && !wbm_reads && !fw_sring_n && !fw_pset_n && (bits[i] & 2 || !wl_htc.htt_fn), "W26 without a bit");
		c2c_contained();
	}
	puts("C2c off: qrtr 7 with w20 0, 0x2, 0x10, 0x80, 0x92, 0x42: no W26 line, arena, WBM access, ring message or HTT handler");
}

/* For the L19 re-derivation in tools/test-fuxi-wlan-ce.py: the four SRING_SETUP, the two PDEV_SET_PARAM, the first and
 * last refill and idle entries and the ring 120 register list, for an RX arena at 0xa1a00000 and links at 0xa1400000. */
static void
dump_c2c(void)
{
	static struct wlrx rx;
	uint8_t *ra = aligned_alloc(16384, RXA), *la = aligned_alloc(16384, LNA), out[64];
	uint32_t w[13], regs[WLRX_IDLE_REGS][2], len = 0;
	CHECK(ra && la && !wlrx_layout(&rx, ra, 0xa1a00000ull, RXA) && !wlrx_link_layout(&rx, la, 0xa1400000ull, LNA) &&
	    !wlrx_fill_refill(&rx) && !wlrx_link_fill(&rx), "dump layout");
	for (unsigned i = 0; i < 4; i++) {
		wlrx_sring_words(&rx, i, w);
		for (unsigned k = 0; k < 13; k++) { fprintf(stdout, "sring%u %u %u\n", i, k, w[k]); }
	}
	for (unsigned i = 0; i < 2; i++) {
		const uint32_t p[3] = { 0, i ? WLWMI_PDEV_PARAM_RX_CHAIN_MASK : WLWMI_PDEV_PARAM_TX_CHAIN_MASK, 3 };
		CHECK(!wlwmi_encode_words(WLWMI_CMD_PDEV_SET_PARAM, WLWMI_TAG_PDEV_SET_PARAM_CMD, p, 3, out, sizeof(out), &len), "dump pset");
		for (uint32_t k = 0; k < len / 4; k++) { fprintf(stdout, "pset%u %u %u\n", i, k, g32(out + 4 * k)); }
	}
	fprintf(stdout, "refill 0 %u\nrefill 1 %u\nrefill 2 %u\nrefill 3 %u\n", g32(ra + 0x4000), g32(ra + 0x4004),
	    g32(ra + 0x4000 + 8 * 4094), g32(ra + 0x4000 + 8 * 4094 + 4));
	fprintf(stdout, "idle 0 %u\nidle 1 %u\nidle 2 %u\nidle 3 %u\n", g32(la), g32(la + 4), g32(la + 8 * 32766), g32(la + 8 * 32766 + 4));
	wlrx_idle_regs(&rx, regs);
	for (unsigned i = 0; i < WLRX_IDLE_REGS; i++) { fprintf(stdout, "wbm %u %u\nwbmv %u %u\n", i, regs[i][0], i, regs[i][1]); }
	for (unsigned i = 0; i < WLRX_PRE_N; i++) { fprintf(stdout, "pre %u %u\n", i, wlrx_pre_addr(i)); }
	for (unsigned j = 0; j < 36; j++) { if (wl15_shadow_v3[j] == WLRX_WBM_IDLE_HP) { fprintf(stdout, "shadow 0 %u\n", j); } }
	fprintf(stdout, "hp 0 %u\n", WLRX_NDESC * WLRX_IDLE_DW);
	fflush(stdout);
	free(ra); free(la);
}

/* ---- C3 scenarios (fuxiwlanqrtr=8): the W25 scan service, on the model's clock ---- */
static void c3_reset(void) { c2_reset(); wl_w25 = true; }

/* lifetime end: 1800 s after ready(), within one idle poll; the service line and the containment after it */
static void
c3_lifetime_end(void)
{
#if !defined(FUXI_WLAN_E2E)
	CHECK(now - h_ready_at >= 1800000u && now - h_ready_at <= 1800010u, "the service did not end at its 1800 s lifetime");
#endif
	expect("FUXI WLAN: W25 complete: lifetime over (1800 s); stop/contain\n");
}

#if !defined(FUXI_WLAN_E2E)
static char sayb[1024];
#define SAY(...) (snprintf(sayb, sizeof(sayb), __VA_ARGS__), sayb)

static void
s_svc_requests(void)
{
	static const uint16_t two[2] = {2437, 5180}, mixed[5] = {5180, 2437, 2437, 2484, 5955}, unlisted[2] = {2484, 5955};
	static const uint16_t dfs[1] = {5500};
	uint16_t all[38];
	for (unsigned i = 0; i < 38; i++) { all[i] = (uint16_t)m_chan(i); }
	c3_reset(); C.c3_stale = true;
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	add_frame(5, 45, 5, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);   /* its probe response */
	add_frame(13, 50, 8, B_OFF, 0x0011, "Office-5G", 9, 0, 36, true);
	add_frame(21, 33, 8, B_LAB, 0x0011, "Lab DFS", 7, 0, 100, true);
	h_push(all, 38, true, 150, -1);       /* the iOS type-3 fallback scan: every channel */
	h_push(two, 2, true, 0, -1);          /* dwell 0: 150 */
	h_push(mixed, 5, true, 110, -1);      /* 2484 and 6 GHz dropped, 2437 once, request order kept */
	h_push(two, 2, false, 150, -1);       /* not passive: refused, nothing sent */
	h_push(unlisted, 2, true, 150, -1);   /* no listed channel: refused, nothing sent */
	h_push(dfs, 1, true, 200, -1);
	CHECK(run() == 0, "W25 service with six requests failed");
	CHECK(h_ready_n == 38 && !memcmp(h_ready_f, all, sizeof(all)), "ready() is not the 38 channels");
	CHECK(h_status[0] == 0 && h_status[1] == 0 && h_status[2] == 0 && h_status[3] == EINVAL && h_status[4] == EINVAL &&
	    h_status[5] == 0 && h_takes == 6 && h_dones == 6, "request statuses");
	CHECK(fw_c3_scans == 4 && fw_c3_n == 1 && fw_c3_chan[0] == 5500 && fw_c3_dwell == 200 && fw_c3_flags == 0x11 &&
	    fw_c3_max == 300 && fw_c3_id == 0xa004, "the last START_SCAN");
	CHECK(wl_scan.wrong_id == 1 && wl_scan.done && !wl_scan.dequeued, "the stale COMPLETED of the previous scan id was not ignored");
	/* what went to iOS: seq 1 heard the four frames, each in place, with its RX MHz, SNR + floor, floor -95 */
	unsigned s1 = 0;
	for (unsigned i = 0; i < h_fr_n; i++) {
		const struct h_frame *r = &h_fr[i];
		CHECK(r->noise == -95, "noise floor handed to iOS");
		if (r->seq != 1) { continue; }
		s1++;
		if (!memcmp(r->bssid, B_HOME, 6)) { CHECK(r->mhz == 2437 && (r->rssi == -48 || r->rssi == -50), "HomeNet frame"); }
		else if (!memcmp(r->bssid, B_OFF, 6)) { CHECK(r->mhz == 5180 && r->rssi == -45, "Office frame"); }
		else { CHECK(!memcmp(r->bssid, B_LAB, 6) && r->mhz == 5500 && r->rssi == -62, "Lab frame"); }
	}
	CHECK(s1 == 4 && h_fr_n == 4 + 3 + 3 + 1, "frames handed over per request");
	static const char *const lines[] = {
		"FUXI WLAN: W25 service ready: 38 channels passive to iOS (2.4 GHz 13, 5 GHz 25), lifetime 1800 s\n",
		"FUXI WLAN: W23 scan foreign freq 2412\n",
		"FUXI WLAN: W23 scan foreign freq 5825\n",
		"FUXI WLAN: W24 bss 3c:84:6a:xx:xx:xx ch 6 freq 2437 dBm -48 privacy 1 rsn 1 beacon+probe-resp frames 2 ssid \"HomeNet\"\n",
		"FUXI WLAN: W24 bss 74:83:c2:xx:xx:xx ch 36 freq 5180 dBm -45 privacy 1 rsn 1 beacon frames 1 ssid \"Office-5G\"\n",
		"FUXI WLAN: W24 bss 74:83:c2:xx:xx:xx ch 100 freq 5500 dBm -62 privacy 1 rsn 1 beacon frames 1 ssid \"Lab DFS\"\n",
		"FUXI WLAN: W24 scan summary frames 4 bss 3 ssids 3\n",
		"FUXI WLAN: W25 scan 1 seq 1 id 0xa001: 38 channels passive dwell 150 ms, completed after 5706 ms, reason 0, status 0; "
		"frames 4 bss 3, to iOS 4 kept 4; scan events 40, of another id 0 (last 0)\n",
		"FUXI WLAN: W25 ignored scan event 0x2 of id 0xa001 (in flight 0xa002)\n",
		"FUXI WLAN: W25 scan 2 seq 2 id 0xa002: 2 channels passive dwell 150 ms, completed after 306 ms, reason 0, status 0; "
		"frames 3 bss 2, to iOS 3 kept 3; scan events 5, of another id 1 (last 0xa001)\n",
		"FUXI WLAN: W25 scan 3 seq 3 id 0xa003: 2 channels passive dwell 110 ms, completed after 226 ms, reason 0, status 0; "
		"frames 3 bss 2, to iOS 3 kept 3; scan events 5, of another id 1 (last 0xa002)\n",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	expect(SAY("FUXI WLAN: W25 scan 4 seq 4: not passive, refused, nothing sent, status %d\n", EINVAL));
	expect(SAY("FUXI WLAN: W25 scan 5 seq 5: no listed channel, refused, nothing sent, status %d\n", EINVAL));
	expect("FUXI WLAN: W25 scan 6 seq 6 id 0xa004: 1 channels passive dwell 200 ms, completed after 206 ms, reason 0, status 0; "
	    "frames 1 bss 1, to iOS 1 kept 1; scan events 4, of another id 1 (last 0xa003)\n");
	CHECK(count_text("W23 scan foreign freq") == 38 && count_text("W24 scan summary") == 1 && count_text("FUXI WLAN: W24 bss ") == 3,
	    "foreign lines of the first scan only, W24 lines of the first completed scan only");
	CHECK(count_text("W25 ignored scan event") == 3, "one line per ignored stale COMPLETED");
	CHECK(sleeps_10 - h_ready_s10 >= 179000 && sleeps_1 - h_ready_s1 < 8000, "polls: 10 ms idle, 1 ms while scanning");
	c3_lifetime_end();
	contained_and_freed();
	expect(SAY("FUXI WLAN: W25 service end: lifetime over after 1800 s; scans 6 (completed 4, failed 0, cancelled 0, "
	    "refused 2), frames to iOS 11 kept 11; stops confirmed late 0, scan events of another id 3, credit waits 0;\n"
	    "FUXI WLAN: W25 service end rings: rx CE1 62/0 CE2 %u/%u tx CE3 %u/%u credits %d\n", wl_ce.pipe[2].posted,
	    wl_ce.pipe[2].reaped, wl_ce.pipe[3].posted, wl_ce.pipe[3].reaped, M_CREDITS));
	expect("FUXI WLAN: W23 contain: scan over, no STOP_SCAN\n");
	CHECK(wl_ce.pipe[2].posted - wl_ce.pipe[2].reaped == 62 && wl_ce.pipe[3].posted == wl_ce.pipe[3].reaped && fw_stop_cmds == 0,
	    "RX buffers out / TX reaped / no STOP_SCAN");
	expect_not("11:22:33"); expect_not("44:55:66");
	puts("W25 requests: ready(38), the 38-channel type-3 scan, {2437,5180} at dwell 0 -> 150, a mixed list (2484 and 6 GHz "
	     "dropped, a repeat once, order kept, dwell 110), a non-passive and an unlisted request refused with EINVAL and "
	     "nothing sent, a DFS channel at 200 ms; ids 0xa001.., a stale COMPLETED of the previous id ignored; every "
	     "beacon/probe response handed over in its RX buffer with MHz, SNR + floor and floor; foreign lines and W24 "
	     "lines once; 10 ms idle polls; lifetime end at 1800 s, contained, freed");
}

static void
s_svc_cancel(void)
{
	static const uint16_t six[1] = {2437};
	uint16_t all[38];
	for (unsigned i = 0; i < 38; i++) { all[i] = (uint16_t)m_chan(i); }
	c3_reset();
	for (unsigned i = 0; i < 38; i++) {
		uint8_t bssid[6] = {0x10, 0x20, 0x30, 0, 0, (uint8_t)i};
		char ssid[16];
		snprintf(ssid, sizeof(ssid), "Net-%02u", i);
		add_frame(i, 30, 8, bssid, 0x0401, ssid, 6, 0, 0, false);
	}
	h_push(all, 38, true, 150, 400);     /* iOS gives up after 400 ms (a power-off, a newer scan) */
	h_push(six, 1, true, 150, -1);
	CHECK(run() == 0, "W25 cancel");
	CHECK(h_status[0] == ECANCELED && h_status[1] == 0 && fw_stop_cmds == 1 && fw_c3_stop_id == 0xa001 && fw_c3_scans == 2,
	    "a cancel: STOP_SCAN of that scan, ECANCELED, the next request served");
	CHECK(fw_c3_stop_after[1] >= 400 && fw_c3_stop_after[1] <= 402 && fw_dropped > 0, "STOP_SCAN 400 ms in; the rest dropped");
	unsigned s1 = 0, s2 = 0;
	for (unsigned i = 0; i < h_fr_n; i++) { s1 += h_fr[i].seq == 1; s2 += h_fr[i].seq == 2; }
	CHECK(s1 == 3 && s2 == 1, "frames handed over before the cancel (channels 1, 2, 3) and after");
	expect(SAY("FUXI WLAN: W25 scan 1 seq 1 id 0xa001: 38 channels passive dwell 150 ms, cancelled, stop confirmed after 402 ms, "
	    "reason %u, status %d; frames 3 bss 3, to iOS 3 kept 3; scan events ", WLWMI_SCAN_REASON_CANCELLED, ECANCELED));
	expect("FUXI WLAN: W24 bss 10:20:30:xx:xx:xx ch 6 freq 2437 dBm -65 privacy 0 rsn 0 beacon frames 1 ssid \"Net-05\"\n");
	CHECK(count_text("W23 scan foreign freq") == 3 && count_text("W24 scan summary") == 1, "foreign lines of the first scan");
	c3_lifetime_end();
	contained_and_freed();
	expect("scans 2 (completed 1, failed 0, cancelled 1, refused 0)");
	/* a cancel whose STOP_SCAN is never answered: ECANCELED to iOS after the 300 ms, a failed scan all the same, and
	 * the next START_SCAN only 3 s after the STOP_SCAN (L19's wait) */
	c3_reset(); C.c3_stop_silent = true;
	h_push(six, 1, true, 150, 100); h_push(six, 1, true, 150, -1);
	CHECK(run() == 0, "W25 unanswered cancel");
	CHECK(h_status[0] == ECANCELED && h_status[1] == 0 && fw_stop_cmds == 1 && fw_c3_scans == 2 &&
	    fw_c3_start_at >= fw_c3_stop_at + 3000, "unanswered cancel: ECANCELED, then the next START_SCAN 3 s after the STOP_SCAN");
	expect(SAY("FUXI WLAN: W25 scan 1 seq 1 id 0xa001: 1 channels passive dwell 150 ms, cancelled, stop not confirmed after 400 ms, "
	    "reason 0, status %d; frames 0 bss 0, to iOS 0 kept 0; scan events ", ECANCELED));
	c3_lifetime_end();
	contained_and_freed();
	expect("scans 2 (completed 1, failed 1, cancelled 0, refused 0)");
	/* a STOP_SCAN answered 1 s late: ECANCELED to iOS after the 300 ms; the late end is waited for (a cancel, not a
	 * failure) and the next START_SCAN goes only after it */
	c3_reset(); C.c3_stop_late_ms = 1000;
	h_push(six, 1, true, 150, 100); h_push(six, 1, true, 150, -1);
	CHECK(run() == 0, "W25 late STOP_SCAN answer");
	CHECK(h_status[0] == ECANCELED && h_status[1] == 0 && fw_stop_cmds == 1 && fw_c3_scans == 2 &&
	    fw_c3_start_at >= fw_c3_stop_at + 1000 && fw_c3_start_at <= fw_c3_stop_at + 1003,
	    "late stop: the next START_SCAN right after the late end, not before");
	expect(SAY("FUXI WLAN: W25 scan 1 seq 1 id 0xa001: 1 channels passive dwell 150 ms, cancelled, stop confirmed late after 400 "
	    "ms, reason %u, status %d; ", WLWMI_SCAN_REASON_CANCELLED, ECANCELED));
	c3_lifetime_end();
	contained_and_freed();
	expect("scans 2 (completed 1, failed 0, cancelled 1, refused 0), frames to iOS 0 kept 0; stops confirmed late 1,");
	puts("W25 cancel: iOS cancels 400 ms into a 38-channel scan: STOP_SCAN with that scan's id, its COMPLETED (cancelled) "
	     "within the 300 ms bound, ECANCELED, what the target still had queued dropped, 3 frames handed over; the next "
	     "request is served and gives the W24 lines; an unanswered cancel is a failure and holds the next START_SCAN "
	     "3 s; a STOP_SCAN answered 1 s late is a cancel and the next START_SCAN follows the late end");
}

static void
s_svc_failures(void)
{
	static const uint16_t two[2] = {2437, 5180};
	uint16_t all[38];
	for (unsigned i = 0; i < 38; i++) { all[i] = (uint16_t)m_chan(i); }
	/* START_FAILED, a good scan (resets the count), no COMPLETED, DEQUEUED, no STARTED (38 channels: max 8550 ms,
	 * so the 1000 ms STARTED bound comes first): the third failure in a row contains */
	c3_reset();
	C.c3_mode[1] = 1; C.c3_mode[3] = 3; C.c3_mode[4] = 4; C.c3_mode[5] = 2;
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	for (unsigned i = 0; i < 4; i++) { h_push(two, 2, true, 150, -1); }
	h_push(all, 38, true, 150, -1); h_push(two, 2, true, 150, -1);
	CHECK(run() == 1, "3 failed scans in a row did not contain");
	CHECK(h_status[0] == EIO && h_status[1] == 0 && h_status[2] == ETIMEDOUT && h_status[3] == EIO && h_status[4] == ETIMEDOUT &&
	    h_status[5] == H_NONE && h_takes == 5, "statuses: START_FAILED EIO, ok, overrun ETIMEDOUT, DEQUEUED EIO, no STARTED ETIMEDOUT");
	CHECK(fw_stop_cmds == 2 && fw_c3_stop_after[3] >= 650 && fw_c3_stop_after[3] <= 651 && fw_c3_stop_after[5] >= 1000 &&
	    fw_c3_stop_after[5] <= 1001, "STOP_SCAN at max_scan_time + 200 ms (450 + 200) and at 1000 ms without STARTED");
	expect(SAY("FUXI WLAN: W25 scan 1 seq 1 id 0xa001: 2 channels passive dwell 150 ms, start failed after 2 ms, reason 4, "
	    "status %d; frames 0 bss 0, to iOS 0 kept 0; scan events 1, of another id 0 (last 0)\n", EIO));
	expect(SAY("FUXI WLAN: W25 scan 3 seq 3 id 0xa003: 2 channels passive dwell 150 ms, no end by max + 200 ms, stop confirmed "
	    "after 652 ms, reason %u, status %d; frames 1 bss 1, to iOS 1 kept 1; ", WLWMI_SCAN_REASON_CANCELLED, ETIMEDOUT));
	expect(SAY("FUXI WLAN: W25 scan 4 seq 4 id 0xa004: 2 channels passive dwell 150 ms, dequeued after 5 ms, reason 0, status %d; ",
	    EIO));
	expect(SAY("FUXI WLAN: W25 scan 5 seq 5 id 0xa005: 38 channels passive dwell 150 ms, no STARTED, stop confirmed after 1002 ms, "
	    "reason 0, status %d; ", ETIMEDOUT));
	expect("FUXI WLAN: W25 failure: W25 3 failed scans in a row; contain, no retry\n");
	contained_and_freed();
	expect("FUXI WLAN: W25 service end: 3 failed scans in a row after ");
	expect("; scans 5 (completed 1, failed 4, cancelled 0, refused 0)");
	expect("FUXI WLAN: W23 contain: scan over, no STOP_SCAN\n");
	/* a STOP_SCAN the target never answers: not confirmed after 300 ms, a failed scan; the next START_SCAN only 3 s
	 * after that STOP_SCAN (L19 waits 3 s for the end), and its id still works */
	c3_reset();
	C.c3_mode[1] = 3; C.c3_stop_silent = true;
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	h_push(two, 2, true, 150, -1); h_push(two, 2, true, 150, -1);
	CHECK(run() == 0, "unconfirmed STOP_SCAN");
	CHECK(h_status[0] == ETIMEDOUT && h_status[1] == 0 && fw_c3_id == 0xa002, "unconfirmed stop, then the next scan");
	CHECK(fw_c3_start_at >= fw_c3_stop_at + 3000 && fw_c3_start_at <= fw_c3_stop_at + 3003,
	    "the next START_SCAN 3 s after an unanswered STOP_SCAN, not before");
	expect(SAY("FUXI WLAN: W25 scan 1 seq 1 id 0xa001: 2 channels passive dwell 150 ms, no end by max + 200 ms, stop not "
	    "confirmed after 950 ms, reason 0, status %d; ", ETIMEDOUT));
	c3_lifetime_end();
	contained_and_freed();
	expect("scans 2 (completed 1, failed 1, cancelled 0, refused 0)");
	/* three START_FAILED in a row: contained, no STOP_SCAN */
	c3_reset();
	C.c3_mode[1] = C.c3_mode[2] = C.c3_mode[3] = 1;
	for (unsigned i = 0; i < 4; i++) { h_push(two, 2, true, 150, -1); }
	CHECK(run() == 1 && h_takes == 3 && h_status[2] == EIO && h_status[3] == H_NONE, "3 x START_FAILED");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: scan start failed, no STOP_SCAN\n");
	CHECK(fw_stop_cmds == 0, "STOP_SCAN after START_FAILED");
	/* three unanswered STOP_SCANs in a row: contained with the third scan in flight on a healthy transport, so
	 * containment's best-effort STOP_SCAN goes out and must name that scan (0xa003), not C2's 0xa000 */
	c3_reset();
	C.c3_mode[1] = C.c3_mode[2] = C.c3_mode[3] = 3; C.c3_stop_silent = true;
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	for (unsigned i = 0; i < 4; i++) { h_push(two, 2, true, 150, -1); }
	CHECK(run() == 1, "3 unconfirmed stops did not contain");
	CHECK(h_takes == 3 && h_status[2] == ETIMEDOUT && h_status[3] == H_NONE && !wl_scan.done && fw_stop_cmds == 3,
	    "the third scan is still in flight");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: STOP_SCAN sent (best effort), tx drained 1");
	CHECK(fw_stop_cmds == 4 && fw_c3_stop_id == 0xa003, "containment STOP_SCAN of the W25 scan in flight");
	/* an overrun at the request's dwell (200 ms, 2 channels): STOP_SCAN at max_scan_time 600 + 200 ms */
	c3_reset();
	C.c3_mode[1] = 3;
	h_push(two, 2, true, 200, -1);
	CHECK(run() == 0, "overrun at dwell 200");
	CHECK(h_status[0] == ETIMEDOUT && fw_c3_dwell == 200 && fw_c3_max == 600 && fw_c3_stop_after[1] >= 800 &&
	    fw_c3_stop_after[1] <= 801, "STOP_SCAN at max_scan_time + 200 ms with the request's dwell (600 + 200)");
	c3_lifetime_end();
	contained_and_freed();
	puts("W25 failures: START_FAILED -> EIO, a good scan resets the count, no COMPLETED by max_scan_time + 200 ms -> "
	     "STOP_SCAN at 650 ms, ETIMEDOUT; DEQUEUED -> EIO; no STARTED in 1000 ms -> STOP_SCAN, ETIMEDOUT; the third in "
	     "a row contains; an unanswered STOP_SCAN gives up at 300 ms (a failure), the next START_SCAN 3 s after it, the "
	     "next id works; 3 x START_FAILED; 3 unanswered stops: containment's STOP_SCAN names the scan in flight; an "
	     "overrun at dwell 200 stops at 800 ms");
}

static void
s_svc_guard_and_credits(void)
{
	static const uint16_t six[1] = {2437};
	/* the W11 guard fails after the 2nd START_SCAN (AMSS left EE 2, SMMU fault, ...): contained mid-scan, no STOP_SCAN */
	c3_reset(); C.c3_guard_after = 2;
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	h_push(six, 1, true, 150, -1); h_push(six, 1, true, 150, -1); h_push(six, 1, true, 150, -1);
	CHECK(run() == 1, "guard failure mid-service passed");
	CHECK(h_status[0] == 0 && h_status[1] == H_NONE && h_takes == 2, "the scan in flight is left to fw_thread's STOPPED");
	expect("FUXI WLAN: W25 failure: AMSS/M0, ownership, discovery or access/SMMU fault; contain, no retry\n");
	{
		unsigned tx = wl_ce.pipe[3].posted;
		contained_and_freed();
		CHECK(fw_stop_cmds == 0 && wl_ce.pipe[3].posted == tx, "a STOP_SCAN after a guard failure");
	}
	expect("FUXI WLAN: W25 service end: a failure (above) after 0 s; scans 2 (completed 1, failed 0, cancelled 0, refused 0)");
	expect("FUXI WLAN: W23 contain: STOP_SCAN not sent (transport not healthy)\n");
	/* the target stops returning credits from the 2nd START_SCAN on: the scans use up what the endpoint holds
	 * (3: INIT's credit is never returned by this firmware), then a START_SCAN waits 3 s */
	c3_reset(); C.c3_starve_after = 2;
	for (unsigned i = 0; i < 6; i++) { h_push(six, 1, true, 150, -1); }
	CHECK(run() == 1, "credit starvation passed");
	CHECK(fw_c3_scans == 1u + M_CREDITS && h_status[M_CREDITS] == 0 && h_status[M_CREDITS + 1] == H_NONE &&
	    h_takes == 2u + M_CREDITS, "the scans the credits allow, then none");
	expect("FUXI WLAN: W25 WMI START_SCAN waits for 1 HTC credits (has 0)\n");
	expect("FUXI WLAN: W25 timeout HTC credits (3000 ms); contain, no retry\n");
	contained_and_freed();
	expect("FUXI WLAN: W23 contain: STOP_SCAN not sent (no HTC credits)\n");
	expect("credits 0\n");
	puts("W25 guard failure after a START_SCAN: contained at once, no STOP_SCAN, no doorbell; credits never returned: the "
	     "6th START_SCAN waits 3000 ms and contains, no STOP_SCAN without credits");
}

static void
s_svc_lifetime_and_band(void)
{
	static const uint16_t five[1] = {5180}, mix[2] = {2412, 5180};
	/* no request at all: 30 minutes of 10 ms polls, then contain */
	c3_reset();
	CHECK(run() == 0, "idle service");
	c3_lifetime_end();
	CHECK(sleeps_10 - h_ready_s10 >= 179990 && sleeps_10 - h_ready_s10 <= 180001 && sleeps_1 == h_ready_s1 &&
	    fw_c3_scans == 0 && h_takes == 0, "idle: one 10 ms poll after another, nothing sent");
	contained_and_freed();
	expect("FUXI WLAN: W25 service end: lifetime over after 1800 s; scans 0 (completed 0, failed 0, cancelled 0, refused 0)");
	expect_not("W23 contain:");
	/* w20 0x10: iOS gets the 13 2.4 GHz channels; 5 GHz requests are refused or cut */
	c3_reset(); wl20_bits = WL20_BAND24; C.exp_n = 13;
	h_push(five, 1, true, 150, -1); h_push(mix, 2, true, 150, -1);
	CHECK(run() == 0, "2.4 GHz-only service");
	CHECK(h_ready_n == 13 && h_ready_f[12] == 2472 && h_status[0] == EINVAL && h_status[1] == 0 && fw_c3_n == 1 &&
	    fw_c3_chan[0] == 2412, "w20 0x10: 13 channels to iOS, 5180 refused, {2412, 5180} -> {2412}");
	expect("FUXI WLAN: W25 service ready: 13 channels passive to iOS (2.4 GHz 13, 5 GHz 0), lifetime 1800 s\n");
	c3_lifetime_end();
	contained_and_freed();
	puts("W25 lifetime: with no request, 180000 polls of 10 ms and the end at 1800 s, contained, freed; w20 0x10: 13 "
	     "channels to iOS, a 5 GHz-only request refused, a mixed one cut to 2.4 GHz");
}

/* Hundreds of scans: every RX buffer posted again as it is consumed, TX completions reaped, credits refunded; and
 * the scan id wraps from 0xafff back to 0xa001. 4100 scans of {2437} with 3 frames each, about 640 s, then idle. */
static void
s_svc_recycle(void)
{
	c3_reset();
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	add_frame(5, 45, 5, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	add_frame(5, 40, 8, B_OFF, 0x0011, "Office-2G", 9, 6, 0, true);
	h_auto = 4100;
	CHECK(run() == 0, "4100 scans");
	CHECK(h_auto_ok == 4100 && h_dones == 4100 && fw_c3_scans == 4100 && h_auto_frames == 3 * 4100, "every scan heard its 3 frames");
	CHECK(h_first_id == 0xa001 && h_last_id == 0xa001 + (4099 % 0xfff) && fw_c3_stop_after[1] == 0, "scan ids wrap 0xafff -> 0xa001");
	CHECK(wl_ce.pipe[2].posted - wl_ce.pipe[2].reaped == 62 && wl_ce.pipe[1].posted - wl_ce.pipe[1].reaped == 62 &&
	    wl_ce.pipe[2].reaped >= 4100u * 9u, "RX: 62 buffers out on each pipe after 36900+ messages");
	CHECK(wl_ce.pipe[3].posted == wl_ce.pipe[3].reaped && wl_ce.pipe[3].posted > 4100 &&
	    wl_htc.ep[wl_htc.wmi_eid].credits == M_CREDITS, "TX: every START_SCAN reaped; every credit refunded");
	CHECK(count_text("FUXI WLAN: W25 scan ") == 24 + 41 && count_text("W24 scan summary") == 1, "W25 lines: 1..24, then every 100th");
	expect("FUXI WLAN: W25 scan 4100 seq 5099 id 0xa005: 1 channels passive dwell 150 ms, completed after 156 ms, reason 0, "
	    "status 0; frames 3 bss 2, to iOS 3 kept 3; scan events 3, of another id 0 (last 0)\n");
	c3_lifetime_end();
	contained_and_freed();
	expect("FUXI WLAN: W25 service end: lifetime over after 1800 s; scans 4100 (completed 4100, failed 0, cancelled 0, "
	    "refused 0), frames to iOS 12300 kept 12300");
	puts("W25 4100 scans: RX buffers re-posted as consumed (62 out on CE1/CE2 after 36900+ messages), every TX "
	     "completion reaped, every credit refunded, scan ids wrap after 0xafff, one W25 line for scans 1..24 and every "
	     "100th, W24 once");
}

/* The iOS side stops for the boot (its own 3 timeouts, say) while the radio lane is idle: the service ends
 * there and contains, instead of idling with the chip powered to its lifetime. */
static void
s_svc_ios_stopped(void)
{
	static const uint16_t six[1] = {2437};
	c3_reset();
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	h_push(six, 1, true, 150, -1); h_push(six, 1, true, 150, -1); h_push(six, 1, true, 150, -1);
	h_gone_after = 2;
	CHECK(run() == 0, "the iOS side's stop");
	CHECK(h_takes == 2 && h_status[1] == 0 && h_status[2] == H_NONE && now - h_ready_at < 1000,
	    "the service did not end when the iOS side stopped");
	expect("FUXI WLAN: W25 complete: the iOS side stopped; stop/contain\n");
	expect_not("lifetime over");
	contained_and_freed();
	expect("FUXI WLAN: W25 service end: the iOS side stopped after 0 s; scans 2 (completed 2, failed 0, cancelled 0, refused 0)");
	expect("FUXI WLAN: W23 contain: scan over, no STOP_SCAN\n");
	puts("W25 iOS side stopped: once it is STOPPED (nothing can be asked this boot) the idle service ends at its next "
	     "poll and contains, no STOP_SCAN, instead of idling to its lifetime");
}

/* A target that answers every SCAN_EVENT with an id of its own (0xa000, the C2 one): the id filter ignores them
 * all, so every scan looks unanswered, but the log says why: the ignored events (8 lines), the per-scan count
 * and last id, the service total. */
static void
s_svc_other_id(void)
{
	static const uint16_t two[2] = {2437, 5180};
	c3_reset(); C.c3_echo_id = 0xa000;
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	for (unsigned i = 0; i < 4; i++) { h_push(two, 2, true, 150, -1); }
	CHECK(run() == 1, "a target echoing another id did not contain after 3 scans");
	CHECK(h_takes == 3 && h_status[0] == ETIMEDOUT && h_status[1] == ETIMEDOUT && h_status[2] == ETIMEDOUT, "three timeouts");
	expect("FUXI WLAN: W25 ignored scan event 0x1 of id 0xa000 (in flight 0xa001)\n");
	CHECK(count_text("W25 ignored scan event") == 8, "ignored-id lines: 8 in the service");
	/* (2 channels: the 650 ms overrun bound comes before the 1000 ms STARTED one; STARTED, 2 FOREIGN_CHAN, the
	 * target's COMPLETED and its answer to STOP_SCAN, all of id 0xa000) */
	expect(SAY("FUXI WLAN: W25 scan 1 seq 1 id 0xa001: 2 channels passive dwell 150 ms, no STARTED, stop not confirmed after "
	    "950 ms, reason 0, status %d; frames 1 bss 1, to iOS 1 kept 1; scan events 5, of another id 5 (last 0xa000)\n",
	    ETIMEDOUT));
	expect("FUXI WLAN: W25 failure: W25 3 failed scans in a row; contain, no retry\n");
	contained_and_freed();
	expect("scans 3 (completed 0, failed 3, cancelled 0, refused 0), frames to iOS 3 kept 3; stops confirmed late 0, "
	    "scan events of another id 15, credit waits 0;");
	puts("W25 another scan id: every SCAN_EVENT ignored; 8 lines name the id the target used, each W25 line counts them "
	     "with the last id, the service end gives the total; 3 timeouts contain");
}

/* A target that returns a START_SCAN's credit only after that scan's COMPLETED, with one credit in play: every
 * START_SCAN from the 4th on waits. The wait line comes 16 times, then every 100th; the end line counts them
 * all. Scan 30 gets START_FAILED: a failed scan past 24 still gets its W25 line. */
static void
s_svc_credit_waits(void)
{
	c3_reset(); C.c3_hold = 2; C.c3_refund_at_end = true; C.c3_fail_at = 30;
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	h_auto = 250;
	CHECK(run() == 0, "250 scans with late credits");
	CHECK(h_auto_ok == 249 && fw_c3_scans == 250, "every scan but the 30th completed");
	CHECK(count_text("W25 WMI START_SCAN waits for 1 HTC credits (has 0)") == 18, "W25 credit-wait lines past the cap");
	CHECK(count_text("FUXI WLAN: W25 scan ") == 24 + 2 + 1, "W25 lines: 1..24, every 100th and the failed 30th");
	expect(SAY("FUXI WLAN: W25 scan 30 seq 1029 id 0xa01e: 1 channels passive dwell 150 ms, start failed after 2 ms, reason 4, "
	    "status %d; ", EIO));
	expect_not("FUXI WLAN: W25 scan 31 ");
	c3_lifetime_end();
	contained_and_freed();
	expect("scans 250 (completed 249, failed 1, cancelled 0, refused 0), frames to iOS 249 kept 249; stops confirmed late 0, "
	    "scan events of another id 0, credit waits 247;");
	puts("W25 late credits: 247 waits, 18 wait lines (1..16, 100th, 200th), all counted in the end line; a failed scan "
	     "past 24 still gets its line");
}

static void
s_svc_pure(void)
{
	uint8_t a[2048], b[2048];
	uint32_t la = 0, lb = 0;
	static const uint16_t first38[38] = {2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 2467, 2472,
		5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320, 5500, 5520, 5540, 5560, 5580, 5600, 5620, 5640, 5660, 5680,
		5700, 5720, 5745, 5765, 5785, 5805, 5825};
	/* the list encoder: the C2 START_SCAN but for the id, the dwell and the channels */
	CHECK(wlwmi_encode_start_scan(38, 38 * 150 * 3 / 2, a, sizeof(a), &la) == 0 &&
	    wlwmi_encode_start_scan_list(first38, 38, 0xa001, 150, 38 * 150 * 3 / 2, b, sizeof(b), &lb) == 0 && la == lb,
	    "list encoder");
	for (uint32_t i = 0; i < la; i += 4) {
		uint32_t w = i / 4;
		CHECK(w == 2 ? g32(b + i) == 0xa001 && g32(a + i) == 0xa000 : g32(a + i) == g32(b + i), "list encoder differs beyond the id");
	}
	static const uint16_t two[2] = {5180, 2437}, bad6[1] = {5955}, ch14[1] = {2484};
	CHECK(wlwmi_encode_start_scan_list(two, 2, 0xafff, 110, 330, b, sizeof(b), &lb) == 0 && g32(b + 4 + 4 * 7) == 110 &&
	    g32(b + 4 + 4 * 13) == 330 && g32(b + 4 + 4 * 15) == 0x11 && g32(b + 168) == 5180 && g32(b + 172) == 2437, "two channels");
	CHECK(wlwmi_encode_start_scan_list(bad6, 1, 0xa001, 150, 225, b, sizeof(b), &lb) == 1 &&
	    wlwmi_encode_start_scan_list(ch14, 1, 0xa001, 150, 225, b, sizeof(b), &lb) == 1 &&
	    wlwmi_encode_start_scan_list(two, 0, 0xa001, 150, 0, b, sizeof(b), &lb) == 1 &&
	    wlwmi_encode_start_scan_list(two, 39, 0xa001, 150, 0, b, sizeof(b), &lb) == 1 &&
	    wlwmi_encode_start_scan_list(two, 2, 0xa000, 150, 450, b, sizeof(b), &lb) == 1 &&
	    wlwmi_encode_start_scan_list(two, 2, 0xb000, 150, 450, b, sizeof(b), &lb) == 1 &&
	    wlwmi_encode_start_scan_list(two, 2, 0xa001, 49, 147, b, sizeof(b), &lb) == 1 &&
	    wlwmi_encode_start_scan_list(two, 2, 0xa001, 201, 603, b, sizeof(b), &lb) == 1 &&
	    wlwmi_encode_start_scan_list(NULL, 2, 0xa001, 150, 450, b, sizeof(b), &lb) == 1 &&
	    wlwmi_encode_start_scan_list(two, 2, 0xa001, 150, 450, b, 195, &lb) == 1 &&
	    wlwmi_encode_start_scan_list(two, 2, 0xa001, 150, 450, b, 196, &lb) == 0 && lb == 196, "list encoder refusals");
	CHECK(wlscan_listed(2472, 13) && !wlscan_listed(5180, 13) && wlscan_listed(5825, 38) && !wlscan_listed(2484, 38) &&
	    !wlscan_listed(5955, 38), "listed channels");
	/* the scan id filter: another id's events are counted and ignored; 0 takes any (C2) */
	struct wlscan *s = calloc(1, sizeof(*s));
	uint8_t ev[4 + 28];
	CHECK(s != NULL, "alloc");
	wlscan_init(s, 1);
	CHECK(!s->want_id && !s->heard, "wlscan_init leaves C3 off");
	s->want_id = 0xa002;
	wlwmi_put_tlv(ev, 0x24, 28); memset(ev + 4, 0, 28); p32(ev + 4, 1); p32(ev + 4 + 16, 0xa001);
	CHECK(wlscan_parse_event(s, ev, sizeof(ev)) == 0 && !s->started && s->wrong_id == 1, "another id's STARTED ignored");
	p32(ev + 4 + 16, 0xa002);
	CHECK(wlscan_parse_event(s, ev, sizeof(ev)) == 0 && s->started && s->wrong_id == 1, "this id's STARTED taken");
	free(s);
	/* credits stop at WLHTC_CREDITS_MOST */
	struct wlhtc h;
	uint8_t t[8] = {1, 4, 0, 0, 2, 255, 0, 0};
	CHECK(wlhtc_init(&h, wl15_service_ce, 14) == 0, "htc init");
	h.ep[2].used = h.ep[2].flow = true; h.ep[2].credits = WLHTC_CREDITS_MOST - 100;
	CHECK(wlhtc_trailer(&h, t, 8) == 0 && h.ep[2].credits == WLHTC_CREDITS_MOST, "credits past 0xffff");
	CHECK(wlhtc_trailer(&h, t, 8) == 0 && h.ep[2].credits == WLHTC_CREDITS_MOST, "credits at the bound");
	h.ep[2].credits = 3;
	CHECK(wlhtc_trailer(&h, t, 8) == 0 && h.ep[2].credits == 258, "credits below the bound");
	puts("W25 pure: the list encoder is C2's START_SCAN but for id, dwell and channels; refuses unlisted/6 GHz/ch 14, "
	     "0 or 39 channels, ids outside 0xa001..0xafff, dwells outside 50..200, a short buffer; the scan id filter; "
	     "credits stop at 0xffff");
}

/* C2c under qrtr 8: the W26 setup exactly as under qrtr 7 (W20 arenas, idle list and refill ring; at W23 the four
 * SRING_SETUP after VERSION_CONF and the chain masks before the channel list), the started, mid and completed watches
 * and the W26 lines after the first W25 scan only, C2b's chan-info lines in the first scan only, no C2b pdev stats
 * request in any scan. A firmware that hands MGMT_RX on only with host buffers and links (rx_rule 2): with w20 0x702
 * the frames reach iOS in every scan, without a W26 bit none do. */
static void
s_svc_c2c(void)
{
	uint16_t all[38];
	for (unsigned i = 0; i < 38; i++) { all[i] = (uint16_t)m_chan(i); }
	c3_reset(); C.rx_rule = 2;
	happy_frames();
	h_push(all, 38, true, 150, -1);
	CHECK(run() == 0, "W25 without W26 failed");
	CHECK(h_fr_n == 0 && h_status[0] == 0 && fw_stats_req == 0 && fw_sring_n == 0, "frames reached iOS without host RX rings");
	expect_not("W26"); expect_not("W23 stats");
	CHECK(count_text("W23 chan-info freq") == 76, "chan-info lines of the one scan");
	c3_lifetime_end();
	contained_and_freed();
	c3_reset(); wl20_bits = 0x702;
	C.rx_rule = 2; C.links_used = true; C.rx_frames = 2; C.setup_delay = 50;
	happy_frames();
	h_push(all, 38, true, 150, -1);
	h_push(all, 38, true, 150, -1);
	CHECK(run() == 0, "W25 with w20 0x702 failed");
	show("W25 0x702");
	CHECK(h_status[0] == 0 && h_status[1] == 0 && h_fr_n == 2u * wl_scan.frames && wl_scan.frames == 12, "frames to iOS per scan");
	CHECK(fw_sring_n == 4 && fw_pset_n == 2 && wbm_writes == 8 && wbm_pubs == 1 && fw_frames_written == 76 && fw_links == 76 &&
	    fw_stats_req == 0, "C2c firmware view under qrtr 8");
	static const char *const lines[] = {
		"FUXI WLAN: W26 link arena IOVA 0xa1400000 bytes 0x440000 Normal-NC mapped\n",
		"FUXI WLAN: W26 refill ring 160 filled 4095 buffers",
		"FUXI WLAN: W20 ce rx posted CE1 62 CE2 62\n",
		"FUXI WLAN: W23 htt version 3.4 (w20 0x02)\n",
		"FUXI WLAN: W26 htt sring setup refill 160 info0 0x0205000b info1 0x00022000 intr 0x003e0000 low 1024\n",
		"FUXI WLAN: W26 htt sring setup rxdma-err 186 info0 0x0007010b info1 0x00082000 intr 0x00200008 low 0\n",
		"FUXI WLAN: W26 rx watch setup+100ms refill tp 4096 taken 2048 left 2047 ",
		"FUXI WLAN: W26 chain mask pdev 0 tx 0x3 rx 0x3 sent (PDEV_SET_PARAM 0x4003 ids 1, 2)\n",
		"FUXI WLAN: W23 chan list 38 passive",
		"FUXI WLAN: W25 service ready: 38 channels passive to iOS (2.4 GHz 13, 5 GHz 25), lifetime 1800 s\n",
		"FUXI WLAN: W26 rx watch started refill tp 4096 taken 2048 left 2047 mac0 hp 2048 tp 0 avail 1024 ",
		"FUXI WLAN: W26 idle watch started hp 65534 tp 0 avail 32767; exhausted none\n",
		"FUXI WLAN: W26 rx watch mid (foreign 19) refill tp 4096 taken 2048 left 2047 mac0 hp 2048 tp 52 avail 998 ",
		"FUXI WLAN: W26 rx watch completed refill tp 4096 taken 2048 left 2047 mac0 hp 2048 tp 52 avail 998 mac1 hp 2048 tp 100 "
		"avail 974 err hp 0 fill 0; exhausted none\n",
		"FUXI WLAN: W26 idle watch completed hp 65534 tp 152 avail 32691; exhausted none\n",
		"FUXI WLAN: W24 scan summary frames 12 bss 9 ssids 5\n",
		"FUXI WLAN: W25 scan 1 seq 1 id 0xa001: 38 channels passive dwell 150 ms, completed after ",
		"FUXI WLAN: W26 rx content mac0 entries 1024 valid 1024 foreign 0 rbm-other 0 cookie-other 0\n",
		"FUXI WLAN: W26 rx content refill taken 2048 kept 0; buffers written 76 of 4095\n",
		"FUXI WLAN: W26 rx guard nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 0\n",
		"FUXI WLAN: W26 link content descriptors written 76 of 32767, unlisted slot clean; idle ring entries changed 0\n",
		"FUXI WLAN: W26 counters htt-t2h 1 htt-rx 1 wmi-events ",
		"FUXI WLAN: W25 scan 2 seq 2 id 0xa002: 38 channels passive dwell 150 ms, completed after ",
		"FUXI WLAN: W25 complete: lifetime over (1800 s); stop/contain\n",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	CHECK(count_text("W26 rx watch") == 4 && count_text("W26 idle watch") == 4 && count_text("W26 rx content mac") == 2 &&
	    count_text("W26 counters") == 1, "W26 watches and lines of the first W25 scan only");
	CHECK(count_text("W23 chan-info freq") == 76 && count_text("W23 scan foreign freq") == 38, "C2b and C2 lines of the first scan only");
	expect_not("W23 stats"); expect_not("W24 no beacon"); expect_not("TX COUNTERS"); expect_not("SPILL");
	c3_lifetime_end();
	c2c_contained();
	static const char *const tail[] = {
		"FUXI WLAN: W25 service end: lifetime over after 1800 s; scans 2 (completed 2, failed 0, cancelled 0, refused 0), "
		"frames to iOS 24 kept 24;",
		"FUXI WLAN: W25 service end rings: ",
		"FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo\n",
		"FUXI WLAN: W23 contain: scan over, no STOP_SCAN\n",
		"FUXI WLAN: W26 arenas zeroed 0x1480000 bytes before free\n",
	};
	expect_order(tail, sizeof(tail) / sizeof(tail[0]));
	puts("W25 with C2c (w20 0x702): the W26 setup as under qrtr 7, watches and W26 lines after the first scan only, C2b "
	     "chan-info lines in the first scan only, no pdev stats request; a buffers-and-links firmware forwards MGMT_RX to "
	     "iOS in both scans with 0x702 and in none without a W26 bit");
}
#endif

#if defined(FUXI_WLAN_E2E)
/* END TO END: the real fuxi_wifi_radio.c (iOS side, tools/test-fuxi-wlan-ce.py's second unit) asks for {2437,
 * 5180}; it must come out as one passive START_SCAN of those channels, and the networks this firmware beacons
 * there must come back through fuxi_wifi_radio_results with their SSID, BSSID, channel and RSSI. */
static void
s_e2e(void)
{
	c3_reset();
	add_frame(5, 47, 8, B_HOME, 0x0411, "HomeNet", 7, 6, 0, true);
	add_frame(13, 50, 8, B_OFF, 0x0011, "Office-5G", 9, 0, 36, true);
	add_frame(0, 30, 8, B_CAFE, 0x0401, "Cafe", 4, 1, 0, false);       /* on 2412: not asked for, never scanned */
	e2e_ios_start();
	CHECK(run() == 0, "e2e service");
	CHECK(fw_c3_scans == 1 && fw_c3_n == 2 && fw_c3_chan[0] == 2437 && fw_c3_chan[1] == 5180 && fw_c3_flags == 0x11 &&
	    fw_c3_dwell == 150 && fw_c3_max == 450, "the iOS request is not one passive START_SCAN of 2437 and 5180 at 150 ms");
	c3_lifetime_end();
	contained_and_freed();
	fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | 0);   /* as fw_thread, after the containment */
	CHECK(e2e_ios_finish() == 0, "the iOS side");
	puts("W25 end to end: a consumer request through the real fuxi_wifi_radio.c came out as one passive START_SCAN of "
	     "its channels; the two networks beaconed there came back through fuxi_wifi_radio_results with SSID, BSSID, "
	     "channel and RSSI; EXITED turned Wi-Fi off");
}
#endif

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
	if (argc == 2 && !strcmp(argv[1], "dump-c2c")) { dump_c2c(); return 0; }
#if defined(FUXI_WLAN_E2E)
	if (argc == 2 && !strcmp(argv[1], "e2e")) { s_e2e(); return 0; }
	return 2;
#else
	if (argc == 2 && !strcmp(argv[1], "c3")) {  /* the C3 scenarios alone */
		s_svc_pure(); s_svc_requests(); s_svc_cancel(); s_svc_failures(); s_svc_guard_and_credits(); s_svc_lifetime_and_band();
		s_svc_recycle(); s_svc_ios_stopped(); s_svc_other_id(); s_svc_credit_waits(); s_svc_c2c(); reset();
		puts("W25 model: all scenarios passed");
		return 0;
	}
#endif
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
	s_c2c_pure();
	s_c2c_none();
	s_c2c_happy();
	s_c2c_idle();
	s_c2c_preread();
	s_c2c_bits();
	s_c2c_readings();
	s_c2c_failures();
	s_c2c_contain();
	s_c2c_widest();
#if !defined(FUXI_WLAN_E2E)
	s_svc_pure();
	s_svc_requests();
	s_svc_cancel();
	s_svc_failures();
	s_svc_guard_and_credits();
	s_svc_lifetime_and_band();
	s_svc_recycle();
	s_svc_ios_stopped();
	s_svc_other_id();
	s_svc_credit_waits();
	s_svc_c2c();
#endif
	reset();
	puts("W20 model: all scenarios passed");
	return 0;
}
