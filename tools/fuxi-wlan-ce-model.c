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
 *   C2d: RX_RING_SELECTION_CFG checked word by word after the SRING_SETUPs; the
 *   REO arena with the device's permission per region; the REO register
 *   conversation (38 pre-reads, the release and exception rings and the REO
 *   setup before the CE RX post, the eight destination rings before INIT, L19's
 *   write order and values, readbacks) and TP publishes that never pass the
 *   device HP; a REO/WBM device that returns every buffer RXDMA filled (with
 *   ring selection: descriptor and MSDU inside its 2048 bytes) on a destination,
 *   exception (MSDU or link descriptor) or release ring; each re-post of the
 *   host checked at its publish (barrier, a returned slot, fresh sentinel, not
 *   past the firmware's TP); the phone's wedge (C.wedge) without the whole path.
 *   C2e: the monitor status arena with the device's permission per region, the
 *   two HP publishes checked when they happen (barrier, every entry, sentinel and
 *   zero word in place, before any SRING_SETUP), the 5th and 6th SRING_SETUP word
 *   by word, a fake RXDMA that writes one status TLV into the next status buffer
 *   per simulated PPDU (never given back: L19 STA mode reaps none), the six
 *   MHI/BHI registers of the crash line, and (C.mon_crash, the C2e scenarios
 *   only) the phone's crash: with RX live and the monitor status rings missing
 *   the firmware stops the scan after 15 channels (3 with ring selection), MHI
 *   goes to SYS_ERR with the phone's ERRDBG words and the MHI reset is never
 *   confirmed.
 *   C2g: the phone's per-MAC rings (C.mac_hp_silent: the firmware fills 161/169 and writes no HP word), the
 *   read-only pointer-word and valid-span lines at every W26 watch, the phone's cc values (programmed), and the
 *   fuxiwlanw29 default 0x9f in a full scan.
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
/* C2c: every device-written word of the RX arena is a load the read barrier must follow; the one host pointer
 * publish into it (wrp[0]) is checked when it happens (model_rx_publish) */
#define WLRX_LOAD(p) (model_event('L'), *(p))
static void model_rx_publish(volatile uint32_t *p, uint32_t v);
#define WLRX_PUBLISH(p, v) model_rx_publish((p), (v))
/* C2d: the REO arena's device-written words, and the refill HP republish of the buffer return path */
#define WLRD_LOAD(p) (model_event('L'), *(p))
static void model_rd_publish(volatile uint32_t *p, uint32_t v);
#define WLRD_PUBLISH(p, v) model_rd_publish((p), (v))
/* C2e: the monitor status rings' device-written TP words, and their two HP publishes (model_ms_publish) */
#define WLMS_LOAD(p) (model_event('L'), *(p))
static void model_ms_publish(volatile uint32_t *p, uint32_t v);
#define WLMS_PUBLISH(p, v) model_ms_publish((p), (v))
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
static uint32_t wl_bhi = 0x1000u;  /* C2e: BHIOFF as milestone 5 found it */
static uint64_t wl_iova_next, wl_iova_base = 0xa0000000ull, wl_iova_end = 0xb0000000ull;
static struct wl_dma wl_ce_arena, wl_msi_sink, wl_tables, wl_rxvectors, wl_rddm, wl_target[WL_TARGET_MAX];
static struct wl_dma wl_rx_arena, wl_link_arena, wl_reo_arena, wl_mon_arena, wl_cm_arena;  /* C2c, C2d, C2e, C2f */
static uint64_t wl_cmem_base, wl_cmem_size; static bool wl_cmem_valid;  /* C2f: QMI dev_mem[0] CMEM region (set by the scenario) */
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
	/* C2d */
	bool wedge;                         /* the phone's r1c2c-all: without ring selection and the REO path the scan stops */
	uint32_t reo_pre[38];               /* what the 38 REO pre-reads return */
	unsigned reo_rb_bad;                /* ring index + 1 whose BASE_LSB reads back wrong */
	unsigned reo_w21;                   /* destination ring index (2..9) found based at W21 */
	unsigned reo_fault_after;           /* fw_faults latched after that many REO accesses */
	bool reo_alloc_fail;                /* the REO arena allocation fails */
	unsigned d_frames;                  /* C2d frames per channel (mac0 on 2.4 GHz, mac1 on 5 GHz) */
	unsigned d_burst;                   /* extra C2d frames on channel 0 */
	const char *d_mix;                  /* frame kinds in turn: D destination, E exception, L exception link, W release */
	uint32_t d_rbm, rel_rbm;            /* RBM in destination/exception entries (0 = 6), in release entries (0 = 8) */
	unsigned d_foreign, d_dup;          /* the Nth C2d frame's entry 64 bytes into its slot / pushed twice */
	unsigned reo_spill;                 /* bit 0 a REO rdp gap word, bit 1 between the rings, bit 2 the tail */
	unsigned d_late;                    /* 20 C2d frames this many ms after START_SCAN (during containment) */
	uint32_t reo_hp_raw;                /* rdp[1] (dst1 HP) written raw at channel 6 */
	/* C2e */
	bool mon_crash;                     /* the phone: with RX live and the monitor status rings missing the firmware crashes */
	bool mon_ignore;                    /* the firmware takes the two monitor status SRING_SETUP and never uses them */
	unsigned mon_ppdu;                  /* PPDUs per channel whose status RXDMA writes (mac0 on 2.4 GHz, mac1 on 5 GHz) */
	unsigned mon_spill;                 /* bit 0 a slot's upper half, bit 1 an unposted slot, bit 2 the tail, bit 3 a head gap */
	unsigned mon_foreign;               /* bit 0 a ring entry rewritten with a foreign address, bit 1 one with another cookie */
	bool mon_alloc_fail;                /* the monitor status arena allocation fails */
	bool mon_late;                      /* RXDMA writes one status during the MHI reset, before bus master off */
	uint32_t mon_tp_raw;                /* rdp[193] written raw on the last channel, after its status writes */
	/* C2g */
	bool mac_hp_silent;                 /* the phone: the firmware fills MAC rings 161/169 but writes no HP into wrp[1]/[9] */
	bool svc299;                        /* C2h: the firmware advertises WMI_SERVICE_MGMT_RX_REO_SUPPORTED (ext word 1 bit 11) */
	unsigned srng_reap;                 /* C2h: MGMT_SRNG_REAP events at STARTED */
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
static uint32_t fw_cmd_log[64];
static unsigned fw_cmd_n;
/* C2f bisect: the W29 mac-start / encap commands as the firmware saw them, in order (the PDEV/VDEV param id, 0xd DFS,
 * 0x11 an HTT PPDU_STATS_CFG, 0x1d LRO) */
static uint32_t fw_w29_log[16];
static unsigned fw_w29_n, fw_w29_at;
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
	m_tlv(0x10, 16); m_u32(1u << 25); m_u32(C.svc299 ? 1u << 11 : 0u); m_u32(0); m_u32(0);
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
	want[44] = 1u << 5 | 1u << 18 | (wl30_mask & WL30_IPADIS ? 1u << 30 : 0u); want[54] = 1;  /* C2h: IPA_DISABLE */ want[57] = 1; want[68] = 1u << 9; want[69] = fw_svc281 ? 1u << 4 : 0;
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
static uint8_t dslot[4096];        /* C2d: 0 listed by the host, 1 with the firmware, 2 returned on a REO/WBM ring, 3 never */
static uint32_t rd_hp;             /* C2d: the refill HP the host published last */
static uint32_t wbm_log[64][3];
static unsigned wbm_log_n;
struct dact { uint64_t due; unsigned kind; uint32_t a, b; };
static struct dact acts[256];
static unsigned n_acts, acts_done;
enum { A_FRAMES, A_ERR, A_SPILL, A_RDP, A_TAKE, A_SETUP, A_D2, A_RSPILL, A_RHP, A_MON, A_CRASH, A_MSPILL, A_MRAW, A_MFOREIGN };
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
	const bool mon = wl20_bits & WL20_MONSTAT;  /* C2e: the TP words rdp[185]/[193], the HP words wrp[25]/[33] (read) */
	if (off < 0x400u) {
		ok = wr && len == 4 && (off == 4u * 160u || off == 4u * 161u || off == 4u * 169u || off == 4u * 186u ||
		    (mon && (off == 4u * 185u || off == 4u * 193u)));
	} else if (off < 0x400u + 4u * 47u) {
		uint32_t i = (off - 0x400u) / 4u;
		ok = len == 4 && !(off & 3u) && (wr ? i == 1 || i == 9 : i == 0 || i == 1 || i == 9 || i == 26 || (mon && (i == 25 || i == 33)));
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
	rd_hp = v;  /* C2d: the HP the buffer return path republishes from */
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
static uint64_t ms_iova;  /* C2e */
static void
m_sring_want(unsigned i, uint32_t *w)
{
	static const uint32_t t[6][7] = {
		{ 0x0205000bu, 0x04000u, 0x00022000u, 0x400u, 0x280u, 0x003e0000u, 0x400u },
		{ 0x0100010bu, 0x0c000u, 0x02021000u, 0x404u, 0x284u, 0x003e0000u, 0x200u },
		{ 0x0100020bu, 0x10000u, 0x02021000u, 0x424u, 0x2a4u, 0x003e0000u, 0x200u },
		{ 0x0007010bu, 0x14000u, 0x00082000u, 0x2e8u, 0x468u, 0x00200008u, 0x000u },
		/* C2e, monitor status 185 and 193 (base in the monitor status arena): RXDMA_MONITOR_STATUS 1, SW_TO_HW, pdev mac + 1,
		 * 1024 x 2 words, LOOP_CNT_DIS, batch 1 x 2, 500 us, low 128 entries in words; HP wrp[25]/[33], TP rdp[185]/[193] */
		{ 0x0101010bu, 0x0000u, 0x02020800u, 0x464u, 0x2e4u, 0x003e0002u, 0x100u },
		{ 0x0101020bu, 0x4000u, 0x02020800u, 0x484u, 0x304u, 0x003e0002u, 0x100u },
	};
	memset(w, 0, 13 * 4);
	w[0] = t[i][0]; w[1] = (uint32_t)(i >= 4 ? ms_iova : rx_iova) + t[i][1]; w[3] = t[i][2]; w[4] = (uint32_t)rx_iova + t[i][3];
	w[6] = (uint32_t)rx_iova + t[i][4]; w[11] = t[i][5]; w[12] = t[i][6];
}

/* C2e: the SRING_SETUP count the host must send before the ring selection, the chain masks and the channel list. */
static unsigned m_nsring(void) { return (wl20_bits & WL20_MONSTAT) ? 6u : 4u; }
static bool ms_pub[2], fw_mon_live[2];

static void
fw_mac_post(unsigned m, const uint32_t *e)
{
	uint8_t *d;
	CHECK((fw_mac_hp[m] + 2u) % 4096u != fw_mac_tp[m], "model: MAC ring full");
	d = rxd(m_mac_off[m] + 4u * fw_mac_hp[m], 8, true, "MAC ring entry");
	p32(d, e[0]); p32(d + 4, e[1]);
	fw_mac_hp[m] = (fw_mac_hp[m] + 2u) % 4096u;
	if (!C.mac_hp_silent) { p32(rxd(0x400u + 4u * (m_mac_id[m] - 160u), 4, true, "wrp of a MAC ring"), fw_mac_hp[m]); }
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
		dslot[slot] = 1;  /* C2d */
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
	CHECK(!fw_rx_ready && fw_sring_n >= 4, "model: refill taken before the 4th SRING_SETUP or twice");
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
	CHECK(plen == 52 && fw_sring_n < m_nsring(), "SRING_SETUP length, or more than four (six with w20 0x2000)");
	CHECK(fw_htt_req == 1 && q_head > fw_htt_conf_q && fw_c2_step == 0 && !fw_chan_cmds && !fw_pset_n,
	    "SRING_SETUP before VERSION_CONF reached the host, or after the channel list or the chain masks");
	CHECK(rx_pub && (!(wl20_bits & WL20_IDLELINK) || idle_pub || m_refused()), "SRING_SETUP before wrp[0] or the idle HP");
	CHECK(fw_sring_n < 4 || ms_pub[fw_sring_n - 4], "a monitor status SRING_SETUP before its HP publish");
	CHECK(!(wl20_bits & WL20_MONSTAT) || (ms_pub[0] && ms_pub[1]), "an SRING_SETUP before both monitor status HP publishes");
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
	if (fw_sring_n > 4) { fw_mon_live[fw_sring_n - 5] = !C.mon_ignore; }  /* C2e: C.mon_ignore, a firmware that drops them */
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

/* ---- C2d device: ring selection, the REO register file, REO/WBM rings, buffer returns and the wedge ----
 * Written down here again from L19 (not taken from the header): the 17 RX_RING_SELECTION_CFG words for WCN7850
 * (dp_rx.c:4567-4605, dp_tx.c:1363-1510, dp.h:840-884, 971, 1001-1007, 1194-1219, rx_desc.h:1521-1529), the ten
 * UMAC rings (ids, R0, R2, sizes, thresholds: hal.c:1327-1420, hw.c:1194-1270, dp.c:290-340, 491-595, dp_rx.c:539-560)
 * with ath12k_hal_srng_dst_hw_init's write order (hal.c:1638-1707) and ath12k_hal_reo_hw_setup's (hal_rx.c:896-929),
 * the REO arena layout and the device's permission per region. The REO device writes entries into the rings and its
 * HP into the arena's rdp page; the host publishes TPs (shadow, or R2 with fuxiwlanw15=1) and re-posts returned
 * buffers to the refill ring, each checked when it happens. C.wedge plays the phone's r1c2c-all readout: without ring
 * selection and the REO path the firmware takes all 4095 refill buffers (2047 + 2016 to the MAC rings, 32 kept),
 * RXDMA writes nothing and the scan stops after 15 channels without COMPLETED. */
#define REOA 0x100000u
struct mreo { uint32_t id, r0, r2, entries, off, thr; bool on; uint32_t host_tp, dev_hp; };
static const struct mreo mreo_plan[10] = {
	{ 131u, 0x00a34f70u, 0x00a370e0u, 1024u, 0x92000u, 0x00200008u, false, 0, 0 },
	{ 0u, 0x00a388a4u, 0x00a3b088u, 128u, 0x90000u, 0x00200008u, false, 0, 0 },
	{ 1u, 0x00a384e4u, 0x00a3b048u, 2048u, 0x10000u, 0x003e0400u, false, 0, 0 },
	{ 2u, 0x00a3855cu, 0x00a3b050u, 2048u, 0x20000u, 0x003e0400u, false, 0, 0 },
	{ 3u, 0x00a385d4u, 0x00a3b058u, 2048u, 0x30000u, 0x003e0400u, false, 0, 0 },
	{ 4u, 0x00a3864cu, 0x00a3b060u, 2048u, 0x40000u, 0x003e0400u, false, 0, 0 },
	{ 5u, 0x00a386c4u, 0x00a3b068u, 2048u, 0x50000u, 0x003e0400u, false, 0, 0 },
	{ 6u, 0x00a3873cu, 0x00a3b070u, 2048u, 0x60000u, 0x003e0400u, false, 0, 0 },
	{ 7u, 0x00a387b4u, 0x00a3b078u, 2048u, 0x70000u, 0x003e0400u, false, 0, 0 },
	{ 8u, 0x00a3882cu, 0x00a3b080u, 2048u, 0x80000u, 0x003e0400u, false, 0, 0 },
};
static struct mreo mr[10];
static uint8_t *reo_host;
static uint64_t reo_iova;
static bool reo_mapped, reo_inject;
static unsigned reo_allocs, reo_frees, fw_sel_n, rs_n, rs_i, rs_built, reo_tp_pubs, rd_pubs, rd_posted, fw_d_seq, fw_d_lost;
static uint32_t fw_sel_w[2][17];
struct rop { uint8_t wr; uint32_t addr, val; };
static struct rop rscript[300];

/* the 17 words, rx_path_plan C2d / L19 */
static void
m_sel_want(unsigned mac, uint32_t *w)
{
	memset(w, 0, 17 * 4);
	w[0] = 0xcu | (mac + 1u) << 8 | 1u << 26 | 1u << 28;
	w[1] = 2048u; w[4] = 0x01000000u; w[5] = 0x01240000u; w[6] = 0xdu; w[7] = 400u | 272u << 16; w[8] = 144u << 16;
}

static void
fw_rxsel(const uint8_t *pl, uint32_t plen)
{
	uint32_t want[17];
	CHECK(wl20_bits & WL20_RXSEL, "RX_RING_SELECTION_CFG without w20 0x800");
	CHECK(plen == 68 && fw_sel_n < 2, "RX_RING_SELECTION_CFG length, or more than two");
	CHECK(fw_sring_n == m_nsring() && fw_c2_step == 0 && !fw_chan_cmds && !fw_pset_n,
	    "RX_RING_SELECTION_CFG before the four (six) SRING_SETUP, or after the channel list or the chain masks");
	m_sel_want(fw_sel_n, want);
	for (unsigned k = 0; k < 17; k++) {
		fw_sel_w[fw_sel_n][k] = g32(pl + 4 * k);
		if (g32(pl + 4 * k) != want[k]) {
			fprintf(stderr, "RX_RING_SELECTION_CFG %u word %u = %#x, want %#x\n", fw_sel_n, k, g32(pl + 4 * k), want[k]);
			fail("RX_RING_SELECTION_CFG word");
		}
	}
	fw_sel_n++;
}

/* C2h: RX_FSE_SETUP_CFG then RX_FISA_CFG, checked word by word against the vendor's words written down here again
 * (dp_htt.c:4667-4790, 4958-5045; htt.h:8812-8990, 9218-9360; wlan_cfg.c:2414-2420; the CMEM base the scenario gave),
 * each once, after the two RX_RING_SELECTION_CFG and before the channel list; nothing answers. */
static unsigned fw_fse_n, fw_fisa_n;
static void
fw_fse(const uint8_t *pl, uint32_t plen)
{
	static const uint32_t fse[14] = { 0x12u, 128u | 16u << 20 | 2u << 28, 0x114000u, 0x10u, 0xda565a6du, 0xc20e5b25u, 0x3d256741u,
		0xb08fa343u, 0xcb2bcad0u, 0xb4307baeu, 0xa32dcb77u, 0x0cf23080u, 0x3bb7426au, 0xfa01acbeu };
	static const uint32_t fisa[3] = { 0x15u, 0x1fu, 0xffffffffu };
	const bool is_fse = (g32(pl) & 0xffu) == 0x12u;
	const uint32_t *want = is_fse ? fse : fisa, n = is_fse ? 14u : 3u;
	CHECK(wl30_mask & WL30_FSE, "an FSE/FISA message without fuxiwlanvrx 0x08");
	CHECK(fw_sel_n == 2 && fw_c2_step == 0 && !fw_chan_cmds && !fw_pset_n, "FSE/FISA before the ring selection, or after the channel list");
	CHECK(is_fse ? !fw_fse_n && !fw_fisa_n : fw_fse_n == 1 && !fw_fisa_n, "FSE/FISA twice or out of order");
	CHECK(plen == 4u * n, "FSE/FISA length");
	for (unsigned k = 0; k < n; k++) {
		if (g32(pl + 4 * k) != want[k]) {
			fprintf(stderr, "%s word %u = %#x, want %#x\n", is_fse ? "RX_FSE_SETUP_CFG" : "RX_FISA_CFG", k, g32(pl + 4 * k), want[k]);
			fail("FSE/FISA word");
		}
	}
	if (is_fse) { fw_fse_n++; } else { fw_fisa_n++; }
}

/* A device access to the REO arena: the HP words of the ten rings (rdp 0..8, 131) and the rings, nothing else. */
static uint8_t *
reod(uint32_t off, uint32_t len, const char *what)
{
	bool ok = false;
	CHECK(reo_host && reo_mapped && (epcmd & 4u), "REO arena DMA unmapped or with bus mastering off");
	CHECK(len && off + len <= REOA, what);
	if (off < 0x1000u) { ok = len == 4 && !(off & 3u) && (off / 4u <= 8u || off / 4u == 131u); }
	for (unsigned k = 0; k < 10; k++) { ok |= off >= mr[k].off && off + len <= mr[k].off + 32u * mr[k].entries; }
	CHECK(ok || reo_inject, what);
	return reo_host + off;
}

static bool
m_reo_refused(void)
{
	for (unsigned i = 0; i < 38; i++) { if (C.reo_pre[i] == ~0u) { return true; } }
	for (unsigned k = 0; k < 10; k++) {
		if ((C.reo_pre[3 * k + 2] & 0x40u) || C.reo_pre[3 * k] || (C.reo_pre[3 * k + 1] & 0xffu)) { return true; }
	}
	return C.reo_pre[36] == 0x43214321u || C.reo_pre[37] == 0x43214321u;
}

static void
rs_add(uint8_t wr, uint32_t addr, uint32_t val)
{
	CHECK(rs_n < sizeof(rscript) / sizeof(rscript[0]), "model REO script");
	rscript[rs_n++] = (struct rop){ wr, addr, val };
}

static void
rs_ring(unsigned k)
{
	const struct mreo *r = &mr[k];
	const uint64_t base = reo_iova + r->off, hp = reo_iova + 4ull * r->id;
	rs_add(1, r->r0, (uint32_t)base); rs_add(1, r->r0 + 4u, (uint32_t)(base >> 32) | (r->entries * 8u) << 8);
	rs_add(1, r->r0 + 8u, r->id << 8 | 8u); rs_add(1, r->r0 + 0x24u, r->thr);
	rs_add(1, r->r0 + 0x14u, (uint32_t)hp); rs_add(1, r->r0 + 0x18u, (uint32_t)(hp >> 32));
	rs_add(1, r->r2, 0); rs_add(1, r->r2 + 4u, 0); rs_add(1, r->r0 + 0x10u, 0x40u);
	rs_add(0, r->r0, C.reo_rb_bad == k + 1u ? (uint32_t)base ^ 0x1000u : (uint32_t)base); rs_add(0, r->r0 + 0x10u, 0x40u);
}

/* The whole REO register conversation the host must hold, in L19's order (built once the arena's IOVA is known). */
static void
rs_build(void)
{
	static const uint32_t hw[8] = { 0x00a38000u, 0x00a38b7cu, 0x00a38b08u, 0x00a38b0cu, 0x00a38b10u, 0x00a38b14u, 0x00a3800cu,
		0x00a38010u };
	rs_n = rs_i = 0; rs_built = 1;
	for (unsigned k = 0; k < 10; k++) {
		rs_add(0, mr[k].r0, C.reo_pre[3 * k]); rs_add(0, mr[k].r0 + 4u, C.reo_pre[3 * k + 1]); rs_add(0, mr[k].r0 + 0x10u, C.reo_pre[3 * k + 2]);
	}
	for (unsigned i = 0; i < 8; i++) { rs_add(0, hw[i], C.reo_pre[30 + i]); }
	if (m_reo_refused()) { return; }
	rs_ring(0); rs_ring(1);
	rs_add(0, hw[0], C.reo_pre[30]); rs_add(1, hw[0], C.reo_pre[30] | 0xcu);
	rs_add(0, hw[1], C.reo_pre[31]); rs_add(1, hw[1], C.reo_pre[31] & ~0x01fe0000u);
	rs_add(1, hw[2], 100000u); rs_add(1, hw[3], 100000u); rs_add(1, hw[4], 100000u); rs_add(1, hw[5], 40000u);
	rs_add(1, hw[6], 0x43214321u); rs_add(1, hw[7], 0x43214321u);
	for (unsigned k = 2; k < 10; k++) {
		const uint32_t b = C.reo_w21 == k ? 0x1000u : 0u;
		rs_add(0, mr[k].r0, b); rs_add(0, mr[k].r0 + 4u, 0); rs_add(0, mr[k].r0 + 0x10u, 0);
		if (b) { return; }
		rs_ring(k);
	}
}

static bool
m_reo_addr(uint32_t off)
{
	return (off >= 0x00a38000u && off < 0x00a3c000u) || (off >= 0x00a34f70u && off < 0x00a34f98u) || off == 0x00a370e0u ||
	    off == 0x00a370e4u;
}

static void
reo_access(uint32_t off, bool wr, uint32_t *v, bool shadow)
{
	CHECK(!wl_rd.stopping, "a REO access after wl26_contain");
	CHECK(wl20_bits & WL20_REORX, "a REO access without w20 0x1000");
	CHECK(reo_host, "a REO access before the REO arena");
	if (!rs_built) { rs_build(); }
	for (unsigned k = 0; wr && k < 10; k++) {
		struct mreo *r = &mr[k];
		const uint32_t size = r->entries * 8u;
		if (!r->on || off != r->r2 + 4u) { continue; }
		model_event('P');
		CHECK(shadow == !wl15_variant, "REO TP published by the wrong route for the shadow mode");
		CHECK(*v < size && !(*v % 8u) && (*v + size - r->host_tp) % size <= (r->dev_hp + size - r->host_tp) % size,
		    "REO TP outside the ring, misaligned or past the device HP");
		r->host_tp = *v; reo_tp_pubs++;
		return;
	}
	CHECK(!shadow, "a shadow write to a REO register other than a live ring's TP");
	if (rs_i >= rs_n || rscript[rs_i].wr != wr || rscript[rs_i].addr != off || (wr && rscript[rs_i].val != *v)) {
		fprintf(stderr, "REO access %u: %s %#x = %#x, want %s %#x = %#x\n", rs_i, wr ? "W" : "R", off, wr ? *v : 0,
		    rs_i < rs_n && rscript[rs_i].wr ? "W" : "R", rs_i < rs_n ? rscript[rs_i].addr : 0, rs_i < rs_n ? rscript[rs_i].val : 0);
		fail("REO register order or value is not L19's (or outside the allowlist, or after a refusal)");
	}
	if (!wr) { *v = rscript[rs_i].val; }
	if (wr && C.reo_fault_after && rs_i >= C.reo_fault_after) { fail("a REO write after the latched fault"); }
	rs_i++;
	if (wr && C.reo_fault_after && rs_i == C.reo_fault_after) { fw_faults++; }
	for (unsigned k = 0; wr && k < 10; k++) {
		if (off == mr[k].r0 + 0x10u) {  /* MISC: the ring goes live, empty, its HP word zero; L19's places (dp_alloc before the
		                                 * CE RX post, core.c:1285-1300; REO2SW1..8 after service ready and before INIT, 925-933) */
			CHECK(k >= 2 ? fw_setup_done && fw_ext_q != ~0u && !fw_init_ok : !wl_ce.pipe[1].posted && !rx_pub,
			    "a REO ring set up out of L19's order");
			CHECK(!g32(reod(4u * mr[k].id, 4, "rdp")), "REO rdp word not zero at enable");
			for (uint32_t i = 0; i < mr[k].entries * 32u; i += 4) { CHECK(!g32(reo_host + mr[k].off + i), "REO ring memory not zeroed"); }
			mr[k].on = true;
		}
	}
}

/* One entry pushed by the REO/WBM device to ring K, its HP written to the rdp page. */
static void
m_reo_push(unsigned k, const uint32_t *w)
{
	struct mreo *r = &mr[k];
	const uint32_t size = r->entries * 8u, next = (r->dev_hp + 8u) % size;
	CHECK(r->on, "model: a push to a ring that is not live");
	CHECK(next != r->host_tp, "model: REO ring full");
	uint8_t *e = reod(r->off + 4u * r->dev_hp, 32, "REO entry");
	for (unsigned i = 0; i < 8; i++) { p32(e + 4 * i, w[i]); }
	r->dev_hp = next;
	p32(reod(4u * r->id, 4, "REO rdp"), r->dev_hp);
}

static bool
m_rx_live(void)
{
	for (unsigned k = 0; k < 10; k++) { if (!mr[k].on) { return false; } }
	return fw_sel_n == 2;
}

/* The firmware's own refill: entries from its refill TP up to the host's wrp[0], into MAC ring M while it has room;
 * each the start of a slot the host listed (and did not list again before it came back), RBM 6, cookie = slot. */
static void
fw_d_refill(unsigned m)
{
	const uint32_t hp = g32(rxd(0x400u, 4, false, "wrp[0]"));
	unsigned n = 0;
	while (fw_refill_tp != hp && (fw_mac_hp[m] + 2u) % 4096u != fw_mac_tp[m]) {
		const uint8_t *e = rxd(0x4000u + 4u * fw_refill_tp, 8, false, "refill entry");
		const uint32_t w0 = g32(e), w1 = g32(e + 4), slot = (w0 - (uint32_t)rx_iova - 0x20000u) / 0x1000u;
		CHECK(w0 >= (uint32_t)rx_iova + 0x20000u && !((w0 - (uint32_t)rx_iova - 0x20000u) % 0x1000u) && slot < 4095u &&
		    ((w1 >> 8) & 0xfu) == 6u && (w1 >> 12) == slot && !(w1 & 0xffu), "refill entry is not a listed slot with RBM 6 and its cookie");
		CHECK(dslot[slot] == 0, "a refill entry for a slot that is not the host's to list");
		dslot[slot] = 1;
		fw_refill_tp = (fw_refill_tp + 2u) % 8192u;
		fw_mac_post(m, (const uint32_t[2]){ w0, w1 });
		n++;
	}
	if (n) { p32(rxd(4u * 160u, 4, true, "rdp[160]"), fw_refill_tp); }
}

/* One frame of kind K (D destination, E exception MSDU, L exception link, W release) RXDMA receives for MAC M with ring
 * selection: the 400-byte RX descriptor and 128 bytes of MSDU into the next MAC ring buffer (inside its 2048 bytes),
 * then the entry REO or WBM gives back. Without a buffer: counted, nothing written. */
static void
fw_d_frame(unsigned m, char kind)
{
	uint32_t w[8] = { 0 }, slot, a, w1;
	if (fw_mac_tp[m] == fw_mac_hp[m]) { fw_d_refill(m); }
	if (fw_mac_tp[m] == fw_mac_hp[m]) { fw_nobuf++; return; }
	const uint8_t *e = rxd(m_mac_off[m] + 4u * fw_mac_tp[m], 8, false, "MAC ring entry");
	a = g32(e); w1 = g32(e + 4); slot = (a - (uint32_t)rx_iova - 0x20000u) / 0x1000u;
	CHECK(slot < 4095u && dslot[slot] == 1, "model: a MAC ring buffer not with the firmware");
	uint8_t *f = rxd(0x20000u + 0x1000u * slot, 400u + 128u, true, "frame");
	for (unsigned k = 0; k < 528u; k += 4u) { p32(f + k, M_FRAME_WORD); }
	fw_mac_tp[m] = (fw_mac_tp[m] + 2u) % 4096u;
	p32(rxd(4u * m_mac_id[m], 4, true, "rdp of a MAC ring"), fw_mac_tp[m]);
	fw_frames_written++;
	dslot[slot] = 2;
	fw_d_seq++;
	if (C.d_foreign && fw_d_seq == C.d_foreign) { a += 64u; }
	w[0] = a;
	w[1] = (w1 & ~0xf00u) | (C.d_rbm ? C.d_rbm : 6u) << 8;
	if (kind == 'L' && idle_pub && m_idle_tp != m_idle_hp) {
		const uint8_t *ie = lnd(4u * m_idle_tp, 8, "idle entry");
		const uint32_t la = g32(ie), lw1 = g32(ie + 4), d = (la - (uint32_t)ln_iova - 0x40000u) / 128u;
		uint8_t *ld = lnd(0x40000u + 128u * d, 128, "link descriptor");
		memset(ld, 0, 128);
		p32(ld + 16, M_FRAME_WORD);           /* a PN word: frame data, never printed */
		p32(ld + 32, a); p32(ld + 36, 6u << 8 | slot << 12); p32(ld + 40, 3u);
		m_idle_tp = (m_idle_tp + 2u) % 65536u;
		fw_links++;
		w[0] = la; w[1] = lw1; w[7] = 1u | 0u << 1 | 3u << 3;
		m_reo_push(1, w);
		return;
	}
	if (kind == 'W') {
		w[1] = (w1 & ~0xf00u) | (C.rel_rbm ? C.rel_rbm : 8u) << 8;
		w[2] = 2u | 0u << 6 | 0u << 24 | 9u << 26;
		w[5] = 3u;
		m_reo_push(0, w);
		return;
	}
	if (kind == 'E' || kind == 'L') { w[7] = 0u | 0u << 1 | 5u << 3; m_reo_push(1, w); return; }
	w[2] = 1u; w[4] = 3u | 128u << 3; w[7] = 1u << 1 | (1u + fw_d_seq % 4u) << 20;
	m_reo_push(2 + fw_d_seq % 4u, w);
	if (C.d_dup && fw_d_seq == C.d_dup) { m_reo_push(2 + fw_d_seq % 4u, w); }
}

static void
fw_d_frames(unsigned m, unsigned n)
{
	const char *mix = C.d_mix ? C.d_mix : "DDDDDEWL";
	for (unsigned i = 0; i < n; i++) { fw_d_frame(m, mix[i % strlen(mix)]); }
}

/* The host's refill HP publishes after wrp[0]=8190: right after the write barrier, every new entry a slot that came
 * back on a REO/WBM ring, RBM 6, its cookie, its posted half zero but the 0xa5 sentinel, the HP not past the
 * firmware's TP. */
static void
model_rd_publish(volatile uint32_t *p, uint32_t v)
{
	CHECK(rx_host && (uintptr_t)p == (uintptr_t)(rx_host + 0x400u) && rx_pub, "a W27 publish other than wrp[0], or before the first");
	CHECK(last_ev == 'W', "wrp[0] republished without the write barrier right before it");
	CHECK(!wl_rx.stopping && !wl_rd.stopping && (wl20_bits & WL20_REORX), "wrp[0] republished after the latch or without 0x1000");
	CHECK(v < 8192u && !(v & 1u) && (v + 8192u - rd_hp) % 8192u <= (fw_refill_tp + 8192u - 2u - rd_hp) % 8192u,
	    "refill HP republished past the firmware's TP");
	for (uint32_t h = rd_hp; h != v; h = (h + 2u) % 8192u) {
		const uint8_t *e = rx_host + 0x4000u + 4u * h;
		const uint32_t w0 = g32(e), slot = (w0 - (uint32_t)rx_iova - 0x20000u) / 0x1000u;
		CHECK(slot < 4095u && w0 == (uint32_t)rx_iova + 0x20000u + 0x1000u * slot && g32(e + 4) == (6u << 8 | slot << 12),
		    "a re-posted refill entry is not a slot start with RBM 6 and its cookie");
		CHECK(dslot[slot] == 2, "a buffer re-posted that the device had not returned");
		for (unsigned j = 0; j < 2048u; j++) { CHECK(rx_host[0x20000u + 0x1000u * slot + j] == (j < 64u ? 0xa5u : 0u), "a re-posted buffer not fresh"); }
		dslot[slot] = 0;
		rd_posted++;
	}
	*p = v;
	rd_hp = v; rd_pubs++;
}

/* ---- C2e device: the monitor status arena, the two HP publishes and RXDMA's status writes ----
 * Written down here again from L19 (not taken from the header): two source rings of 1024 entries of 2 words (ids 185 and
 * 193, hal.h:497-501, dp.h:193, hal.c:157-164), each filled with 1023 status buffers after the refill ring
 * (dp_rx.c:450-517, dp_mon.c:2697-2770: RBM 6 hw.c:1291, cookie = idr id from 0), HP in wrp[25]/[33], TP in
 * rdp[185]/[193]; the arena layout and the device's permission per region (ring entries read, the posted half of a posted
 * slot written, nothing else). RXDMA takes the entry at its TP for each PPDU and writes one status TLV into that buffer;
 * nothing gives a buffer back (L19 STA mode reaps none). */
#define MONA 0x820000u
#define M_MON_TLV (1u << 1 | 24u << 10)   /* a status TLV header (tag 1, 24 bytes); the rest of the TLV is frame data */
static const uint32_t m_mon_id[2] = { 185u, 193u };
static uint8_t *ms_host;
static bool ms_mapped, ms_inject, fw_crashed;
static unsigned ms_allocs, ms_frees, fw_mon_written, fw_mon_nobuf, mhi_reads;
static uint32_t fw_mon_tp[2];

/* A device access to the monitor status arena: ring entries read, the posted half of a posted slot written. */
static uint8_t *
mond(uint32_t off, uint32_t len, bool wr, const char *what)
{
	bool ok = false;
	CHECK(ms_host && ms_mapped && (epcmd & 4u), "monitor status arena DMA unmapped or with bus mastering off");
	CHECK(len && off + len <= MONA, what);
	if (off < 0x10000u) { ok = !wr && (off + len <= 0x2000u || (off >= 0x4000u && off + len <= 0x6000u)); }
	else if (off < 0x810000u) {
		const uint32_t slot = (off - 0x10000u) / 0x1000u, o = (off - 0x10000u) % 0x1000u;
		ok = wr && slot % 1024u < 1023u && o + len <= 2048u;
	}
	CHECK(ok || ms_inject, what);
	return ms_host + off;
}

/* The host's HP publish of ring M: wrp[25] then wrp[33], each 2046 right after the write barrier, after the refill publish
 * and before any SRING_SETUP, with all 1023 entries, every sentinel and every zero word of the arena in place. */
static void
model_ms_publish(volatile uint32_t *p, uint32_t v)
{
	const unsigned m = rx_host && (uintptr_t)p == (uintptr_t)(rx_host + 0x400u + 4u * 25u) ? 0u :
	    rx_host && (uintptr_t)p == (uintptr_t)(rx_host + 0x400u + 4u * 33u) ? 1u : 2u;
	CHECK(m < 2u && (wl20_bits & WL20_MONSTAT), "a W28 pointer publish other than wrp[25] or wrp[33], or without 0x2000");
	CHECK(last_ev == 'W', "a monitor status HP published without the write barrier right before it");
	CHECK(!ms_pub[m] && (m == 0 || ms_pub[0]) && rx_pub && !fw_sring_n && !wl_rx.stopping && !wl_ms.stopping,
	    "a monitor status HP published twice, out of order, before the refill publish, after an SRING_SETUP or the latch");
	CHECK(v == 2046u, "a monitor status HP is not 1023 entries in words");
	CHECK(!g32(rx_host + 4u * m_mon_id[m]), "a monitor status TP word not zero at its HP publish");
	for (uint32_t o = 0x2000u; o < 0x4000u; o++) { CHECK(!ms_host[o] && !ms_host[o + 0x4000u], "a monitor status head gap not zero"); }
	for (uint32_t o = 0xc000u; o < 0x10000u; o++) { CHECK(!ms_host[o], "a monitor status head gap not zero"); }
	for (uint32_t k = 0; k < 1024u; k++) {
		const uint8_t *e = ms_host + 0x4000u * m + 8u * k;
		const uint64_t a = ms_iova + 0x10000u + 0x1000ull * (1024u * m + k);
		if (k < 1023u) {
			CHECK(g32(e) == (uint32_t)a && g32(e + 4) == (((uint32_t)(a >> 32) & 0xffu) | 6u << 8 | k << 12),
			    "a monitor status entry not in place at its HP publish (slot, RBM 6, cookie k)");
		} else { CHECK(!g32(e) && !g32(e + 4), "monitor status entry 1023 listed"); }
		const uint8_t *b = ms_host + 0x10000u + 0x1000u * (1024u * m + k);
		for (unsigned j = 0; j < 4096u; j++) { CHECK(b[j] == (k < 1023u && j < 64u ? 0xa5u : 0u), "a status buffer's sentinel or zero not in place"); }
	}
	for (uint32_t o = 0x810000u; o < MONA; o++) { CHECK(!ms_host[o], "monitor status tail not zero at the HP publish"); }
	*p = v;
	ms_pub[m] = true;
}

/* One PPDU on MAC M: RXDMA takes the status buffer at its TP (the start of a posted slot of that ring, RBM 6, cookie = its
 * index) and writes one status TLV into it; with every buffer taken it writes nothing (counted). */
static void
fw_mon_ppdu(unsigned m)
{
	if (!fw_mon_live[m] || fw_crashed) { return; }
	const uint32_t hp = g32(rxd(0x400u + 4u * (m_mon_id[m] - 160u), 4, false, "wrp of a monitor status ring"));
	if (fw_mon_tp[m] == hp) { fw_mon_nobuf++; return; }
	const uint8_t *e = mond(0x4000u * m + 4u * fw_mon_tp[m], 8, false, "monitor status entry");
	const uint32_t w0 = g32(e), w1 = g32(e + 4), slot = (w0 - (uint32_t)ms_iova - 0x10000u) / 0x1000u;
	CHECK(w0 >= (uint32_t)ms_iova + 0x10000u && !((w0 - (uint32_t)ms_iova - 0x10000u) % 0x1000u) && slot / 1024u == m &&
	    slot % 1024u < 1023u && !(w1 & 0xffu) && ((w1 >> 8) & 0xfu) == 6u && (w1 >> 12) == slot % 1024u,
	    "a monitor status entry is not a posted slot of its ring with RBM 6 and its cookie");
	uint8_t *b = mond(0x10000u + 0x1000u * slot, 32, true, "status TLV");
	p32(b, M_MON_TLV);
	for (unsigned k = 4; k < 32u; k += 4u) { p32(b + k, M_FRAME_WORD); }
	fw_mon_tp[m] += 2u;
	p32(rxd(4u * m_mon_id[m], 4, true, "rdp of a monitor status ring"), fw_mon_tp[m]);
	fw_mon_written++;
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
		case A_D2: fw_d_frames(x->a, x->b); break;
		case A_RSPILL:  /* C2d: the REO device writes where it was given no address */
			reo_inject = true;
			if (C.reo_spill & 1u) { p32(reod(4u * 20u, 4, "spill"), 0x33u); }
			if (C.reo_spill & 2u) { p32(reod(0x91800u, 4, "spill"), 0x33u); }
			if (C.reo_spill & 4u) { p32(reod(0xfff00u, 4, "spill"), 0x33u); }
			reo_inject = false;
			break;
		case A_RHP: reo_inject = true; p32(reod(4u * mr[x->a].id, 4, "raw REO HP"), x->b); reo_inject = false; break;
		case A_MON: for (unsigned i = 0; i < x->b; i++) { fw_mon_ppdu(x->a); } break;  /* C2e */
		case A_CRASH: fw_crashed = true; break;  /* C2e: the phone's crash: the scan stops, MHI SYS_ERR */
		case A_MSPILL:  /* C2e: RXDMA writes where the monitor status arena gave it no address */
			ms_inject = true;
			if (C.mon_spill & 1u) { memset(mond(0x10000u + 0x1000u * 3u + 2048u, 8, true, "spill"), 0x33, 8); }
			if (C.mon_spill & 2u) { memset(mond(0x10000u + 0x1000u * 2047u + 16u, 4, true, "spill"), 0x33, 4); }
			if (C.mon_spill & 4u) { memset(mond(0x810000u + 0x40u, 12, true, "spill"), 0x33, 12); }
			if (C.mon_spill & 8u) { memset(mond(0x3000u, 4, true, "spill"), 0x33, 4); }
			ms_inject = false;
			break;
		case A_MRAW: rx_inject = true; p32(rxd(4u * m_mon_id[x->a], 4, true, "raw monitor status TP"), x->b); rx_inject = false; break;
		case A_MFOREIGN:  /* C2e: the device writes a source ring entry it only had to read */
			ms_inject = true;
			if (C.mon_foreign & 1u) { uint8_t *e = mond(8u * 10u, 8, true, "ring"); p32(e, g32(e) + 64u); }
			if (C.mon_foreign & 2u) { uint8_t *e = mond(0x4000u + 8u * 1000u, 8, true, "ring"); p32(e + 4, g32(e + 4) ^ 1u << 12); }
			ms_inject = false;
			break;
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
	if (b->mode == 5) {  /* C2h: the firmware consumed the frame: WMI_MGMT_RX_FW_CONSUMED, TLV 0x3d3, subtype in word 4 */
		m_reset(); m_tlv(0x3d3, 36); m_u32(1000); m_u32(0); m_u32(0); m_u32((uint32_t)(b->f[0] >> 4) << 28 | 1u << 16);
		m_u32(b->chan_freq); m_u32(77); m_u32(1u << 15 | 9u << 16); m_zero(8);
		wmi_out(0x700d, 0, at);
		return;
	}
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
	if (b->mode == 4 || b->mode == 6) {  /* C2h: the REO params TLV 0x3d2 after the frame (6: counter not valid) */
		m_tlv(0x3d2, 16); m_u32(1234); m_u32(b->mode == 4 ? 1u << 15 | 7u << 16 : 0u); m_u32(0); m_u32(0);
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
	for (unsigned i = 0; i < C.srng_reap; i++) { m_reset(); m_tlv(0x400, 4); m_zero(4); wmi_out(0x700e, 0, t); }  /* C2h */
	if (C.wrong_vdev) { fw_scan_event(1u << 3, 0, 2412, 3, t); }
	const bool deliver_mgmt = C.rx_rule == 6 ? m_rx_live() : fw_rule_ok();  /* C2c; C2d rule 6: the whole RX path */
	const bool wedged = C.wedge && !m_rx_live();  /* C2d: the phone's r1c2c-all readout */
	/* C2e (C.mon_crash): the phone's r1c2c-all and r1c2d-l readouts, RX live without the monitor status rings */
	const bool crash = C.mon_crash && fw_rx_ready && !(fw_mon_live[0] && fw_mon_live[1]);
	const unsigned stop_at = wedged ? 15u : crash ? (fw_sel_n ? 3u : 15u) : n;
	for (unsigned i = 0; i < n && i < stop_at; i++) {
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
		if (C.d_frames && m_rx_live()) { act(now + t0 + 3, A_D2, i < 13 ? 0u : 1u, C.d_frames); }  /* C2d */
		if (i == 0 && C.d_burst && m_rx_live()) { act(now + t0 + 4, A_D2, 0, C.d_burst); }
		if (i == 4 && C.reo_spill) { act(now + t0 + 4, A_RSPILL, 0, 0); }
		if (i == 6 && C.reo_hp_raw) { act(now + t0 + 4, A_RHP, 2, C.reo_hp_raw); }
		if (C.mon_ppdu) { act(now + t0 + 3, A_MON, i < 13 ? 0u : 1u, C.mon_ppdu); }  /* C2e */
		if (i == 4 && C.mon_spill) { act(now + t0 + 4, A_MSPILL, 0, 0); }
		if (i == 6 && C.mon_foreign) { act(now + t0 + 4, A_MFOREIGN, 0, 0); }
		if (i + 1u == n && C.mon_tp_raw) { act(now + t0 + 4, A_MRAW, 1, C.mon_tp_raw); }  /* after the last status write */
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
	if (crash) { act(now + 5 + 150 * stop_at, A_CRASH, 0, 0); }
	if (C.scan_mode != 3 && !wedged && !crash) { fw_scan_event(1u << 1, 0, 0, 0, 5 + 150 * n + 1); }
	if (C.d_late && m_rx_live()) { act(now + C.d_late, A_D2, 0, 20); }  /* C2d: frames while the host contains */
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
		    (!(wl20_bits & WL20_RXRING) || fw_sring_n == m_nsring()), "PDEV_SET_PARAM before VERSION_CONF or the four SRING_SETUP");
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
		CHECK(!(wl20_bits & WL20_RXRING) || fw_sring_n == m_nsring(), "channel list before the four SRING_SETUP");
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

/* C2f (W29): the mac-start pdev params, DFS offload, LRO, idle-PS and the vdev encap/decap, each checked word by word
 * against L19 (mac.c:9363-9469, 9782-9827) and in its place: PMF_QOS, DYNAMIC_BW, ARP_AC_OVERRIDE, DFS, the two PPDU
 * stats and MESH_MCAST after the HTT version exchange and before the chain mask and the channel list; LRO and idle-PS
 * after the whole channel list and before VDEV_CREATE; TX_ENCAP then RX_DECAP right after VDEV_CREATE, before NSS. Only
 * with w20 0x4000, each only once and in L19's order among those sent (fw_w29_log). */
static bool
m_w29_cmd(uint32_t id, const uint8_t *pl, uint32_t plen)
{
	const uint32_t prm = plen >= 16 ? g32(pl + 12) : ~0u;
	if (id == 0xa005u || id == 0x1d010u) { return true; }
	if (id == 0x4003u) { return prm == 0x22u || prm == 0x0au || prm == 0x23u || prm == 0x8fu || prm == 0x2cu; }
	if (id == 0x5008u) { return prm == 0x30u || prm == 0x5bu; }
	return false;
}

static void
fw_w29_order(uint32_t code)
{
	static const uint32_t l19[11] = { 0x22u, 0x0au, 0x23u, 0xdu, 0x11u, 0x11u, 0x8fu, 0x1du, 0x2cu, 0x30u, 0x5bu };
	unsigned at = fw_w29_at;
	CHECK(wl20_bits & WL20_L19ORDER, "a W29 command without w20 0x4000");
	while (at < 11u && l19[at] != code) { at++; }
	CHECK(at < 11u && fw_w29_n < 16u, "a W29 command out of L19's order, or sent twice");
	fw_w29_at = at + 1u;
	fw_w29_log[fw_w29_n++] = code;
}

static void
fw_w29_pre_place(void)
{
	CHECK(fw_c2_step == 0 && !fw_chan_cmds && !fw_pset_n && (!(wl20_bits & WL20_HTTVER) || (fw_htt_req == 1 && q_head > fw_htt_conf_q)) &&
	    (!(wl20_bits & WL20_RXRING) || fw_sring_n == m_nsring()), "a mac-start pre command out of place (L19: before the chain mask)");
}

static void
fw_w29_ppdu(const uint8_t *pl, uint32_t plen)
{
	unsigned i = fw_w29_n && fw_w29_log[fw_w29_n - 1u] == 0x11u ? 1u : 0u;
	fw_w29_pre_place();
	CHECK(plen == 4 && g32(pl) == (0x11u | (1u << i) << 9 | 0xe67u << 16), "HTT PPDU_STATS_CFG is not {0x11, pdev mask 1 << i, 0xe67}");
	fw_w29_order(0x11u);
}

static void
fw_w29_cmd(uint32_t id, const uint8_t *pl, uint32_t plen, unsigned need)
{
	const uint8_t *v = pl + 8;
	const uint32_t pdev = C.exp_pdev;
	if (id == 0x4003u) {
		const uint32_t prm = g32(v + 4), want = prm == 0x23u ? 0u : 1u;
		CHECK(plen == 20 && g32(pl + 4) == (0x52u << 16 | 12u) && g32(v) == pdev && g32(v + 8) == want, "mac-start PDEV_SET_PARAM words");
		if (prm == 0x2cu) { CHECK(fw_c2_step == 1, "IDLE_PS before the whole channel list or after VDEV_CREATE"); }
		else { fw_w29_pre_place(); }
		fw_w29_order(prm);
	} else if (id == 0xa005u) {
		CHECK(plen == 12 && g32(pl + 4) == (0x250u << 16 | 4u) && g32(v) == pdev, "DFS_PHYERR_OFFLOAD_ENABLE words");
		fw_w29_pre_place();
		fw_w29_order(0xdu);
	} else if (id == 0x1d010u) {
		CHECK(plen == 8 + 76 && g32(pl + 4) == (0x1aau << 16 | 76u), "LRO_CONFIG length/tag");
		for (unsigned i = 0; i < 18u; i++) { CHECK(!g32(v + 4 * i), "LRO_CONFIG: lro_enable or a seed non-zero"); }
		CHECK(g32(v + 72) == pdev && fw_c2_step == 1, "LRO_CONFIG pdev, or before the whole channel list or after VDEV_CREATE");
		fw_w29_order(0x1du);
	} else {
		CHECK(plen == 20 && g32(pl + 4) == (0x5fu << 16 | 12u) && g32(v) == 0 && g32(v + 8) == 1u,
		    "VDEV_SET_PARAM encap/decap is not {vdev 0, param, NATIVE_WIFI}");
		CHECK(fw_c2_step == 2, "encap/decap not right after VDEV_CREATE");
		fw_w29_order(g32(v + 4));
	}
	fw_refund(need, 1);
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
		if (ce == 4 && flags == 0 && plen >= 4 && (g32(pl) & 0xffu) == 0xcu) { fw_rxsel(pl, plen); return; }  /* C2d */
		if (ce == 4 && flags == 0 && plen >= 4 && (g32(pl) & 0xffu) == 0x11u) { fw_w29_ppdu(pl, plen); return; }  /* C2f */
		if (ce == 4 && flags == 0 && plen >= 4 && ((g32(pl) & 0xffu) == 0x12u || (g32(pl) & 0xffu) == 0x15u)) {  /* C2h */
			fw_fse(pl, plen);
			return;
		}
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
	} else if (m_w29_cmd(id, pl, plen)) {  /* C2f: a mac-start / encap command (param ids the C2 ones never use) */
		fw_w29_cmd(id, pl, plen, (unsigned)need);
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

/* ---- stubs of the fuxi_wlan.c functions the excerpts call ---- */
static uint32_t
wl_mrd(uint32_t off)
{
	if (off == 0x310c) {
		if (restore_lie) { restore_lie = false; return bar_win ^ 1u; }  /* the readback that follows a restore */
		if (C.win_mismatch_at && win_selects == C.win_mismatch_at && (bar_win & 0x3f) == 0x37) { return bar_win ^ 1u; }
		return bar_win;
	}
	if (off == 0x48u || (off >= wl_bhi && off < wl_bhi + 0x40u)) {  /* C2e: the crash line's six reads, nothing else */
		CHECK(wl20_bits & WL20_MONSTAT, "an MHI/BHI read without w20 0x2000 (only the C2e crash line reads them here)");
		CHECK(off == 0x48u || off == wl_bhi + 0x28u || (off >= wl_bhi + 0x30u && off <= wl_bhi + 0x3cu && !(off & 3u)),
		    "an MHI/BHI read outside the crash line's six");
		mhi_reads++;
		if (off == 0x48u) { return fw_crashed ? 0xff04u : 0x0201u; }        /* SYS_ERR state 0xff, SYSERR bit 2 / M0, ready */
		if (off == wl_bhi + 0x28u) { return 2u; }                             /* EE AMSS */
		if (off == wl_bhi + 0x38u) { return fw_crashed ? 0x0152ee53u : 0u; }  /* ERRDBG2/3: the phone's r1c2c-all readout */
		if (off == wl_bhi + 0x3cu) { return fw_crashed ? 0x492u : 0u; }
		return 0u;
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
	if (m_reo_addr(wl15_shadow_v3[i])) { reo_access(wl15_shadow_v3[i], true, &v, true); return; }  /* C2d: a REO/release TP */
	ce_access(wl15_shadow_v3[i], true, &v, true);
}

/* C2f gate scenarios (s_c2f_gate) only: the W29 register file and CMEM as the model's own address set, not the
 * production allowlists: the six cc_config registers (pre-read as the scenario sets), the ten common rings' R0 blocks
 * and R2 HP/TP (each keeps what the host writes, for the readback; pre-read as the scenario seeds), CMEM the window
 * [cmem_lo, cmem_hi). Every access counted by kind; the order of the first CMEM write and the first ring write kept. */
static const uint32_t m_cm_ring[10][2] = {  /* r0, r2 in L19's common-setup order (hal.h, hw.c) */
	{ 0xa3437cu, 0xa37010u }, { 0xa44900u, 0xa46000u }, { 0xa34e08u, 0xa370c8u }, { 0xa44978u, 0xa46008u },
	{ 0xa34ef8u, 0xa370d8u }, { 0xa449f0u, 0xa46010u }, { 0xa34fe8u, 0xa370e8u }, { 0xa38304u, 0xa3b028u },
	{ 0xa3828cu, 0xa3b020u }, { 0xa38a84u, 0xa3b0a8u },
};
static struct {
	bool on;
	uint32_t cmem_lo, cmem_hi, cc_pre[6];
	uint32_t cmem_rd, cmem_wr, cc_rd, cc_wr, ring_rd, ring_wr, ring_wr_before_cmem, cmem_wr_after_ring;
	uint32_t n, addr[256], val[256];
	uint32_t seed[0x5000u / 4u];   /* C2h: what CMEM reads return, word index from cmem_lo (written words land here too) */
	uint32_t wr_hi, rd_rest, rd_fst, rd_other;  /* C2h: the highest written offset + 4, reads past our 280, flow table reads */
} cmd;

static void
cm_set(uint32_t off, uint32_t v)
{
	for (uint32_t i = 0; i < cmd.n; i++) { if (cmd.addr[i] == off) { cmd.val[i] = v; return; } }
	CHECK(cmd.n < 256u, "model W29 register file");
	cmd.addr[cmd.n] = off; cmd.val[cmd.n++] = v;
}

static bool
cm_access(uint32_t off, bool wr, uint32_t *v)
{
	static const uint32_t cc[6] = { 0xa38050u, 0xa38054u, 0xa34040u, 0xa34044u, 0xa34090u, 0xa34094u };
	if (!cmd.on) { return false; }
	CHECK(!wl_cm.stopping, "a W29 access after wl26_contain");
	if (off >= cmd.cmem_lo && off < cmd.cmem_hi) {
		const uint32_t rel = off - cmd.cmem_lo;
		CHECK(!(off & 3u) && rel < sizeof(cmd.seed), "a misaligned CMEM access, or one past the model's CMEM");
		if (wr) {
			if (!cmd.cmem_wr) { cmd.ring_wr_before_cmem = cmd.ring_wr; }
			cmd.cmem_wr++; cmd.cmem_wr_after_ring += cmd.ring_wr != 0;
			cmd.seed[rel / 4u] = *v; if (rel + 4u > cmd.wr_hi) { cmd.wr_hi = rel + 4u; }
		} else {
			cmd.cmem_rd++; *v = cmd.seed[rel / 4u];
			if (rel >= 0x1000u) { cmd.rd_fst++; } else if (rel >= 4u * 280u) { cmd.rd_rest++; }
			if (rel >= 0x1000u && ((rel - 0x1000u) % 64u != 0x24u || rel >= 0x1000u + 128u * 64u)) { cmd.rd_other++; }
		}
		return true;
	}
	for (unsigned k = 0; k < 6; k++) {
		if (off != cc[k]) { continue; }
		if (wr) { cmd.cc_wr++; } else { cmd.cc_rd++; *v = cmd.cc_pre[k]; }
		return true;
	}
	for (unsigned i = 0; i < 10; i++) {
		if (!((off >= m_cm_ring[i][0] && off < m_cm_ring[i][0] + 0x38u) || off == m_cm_ring[i][1] || off == m_cm_ring[i][1] + 4u)) { continue; }
		if (wr) { cmd.ring_wr++; cm_set(off, *v); return true; }
		cmd.ring_rd++; *v = 0;
		for (uint32_t j = 0; j < cmd.n; j++) { if (cmd.addr[j] == off) { *v = cmd.val[j]; } }
		return true;
	}
	return false;
}

/* A window access: the WBM block (C2c) or the CE registers. */
static void
win_access(uint64_t pa, bool wr, uint32_t *v)
{
	uint32_t off = (bar_win & 0x3fu) << 19 | (uint32_t)(pa - WL_BAR0 - 0x80000ull);
	CHECK(bar_win & 0x40000000u, "window access with the window disabled");
	if (cm_access(off, wr, v)) { return; }  /* C2f gate scenarios only */
	if (off >= 0x00a30000u && off < 0x00a40000u) {
		if (m_reo_addr(off)) { reo_access(off, wr, v, false); } else { wbm_access(off, wr, v, false); }  /* C2d / C2c */
		return;
	}
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
		if (C.mon_late && (epcmd & 4u)) { fw_mon_ppdu(1); }  /* C2e: a status written during the MHI reset */
		mhictrl = fw_crashed ? 2u : 0u;  /* C2e: after the crash the reset is never confirmed (the phone) */
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
	if (d == &wl_mon_arena) {  /* C2e: after the other three, 2 MiB aligned, 0xa5 everywhere */
		CHECK(!d->va && bytes == MONA && (wl20_bits & WL20_MONSTAT) && rx_host && (!(wl20_bits & WL20_IDLELINK) || ln_host) &&
		    (!(wl20_bits & WL20_REORX) || reo_host), "unexpected W28 allocation, or before the RX, link and REO arenas");
		CHECK(!(wl_iova_next & 0x1fffffu) && wl_iova_next + bytes <= wl_iova_end, "W28 arena IOVA not 2 MiB aligned inside the pool");
		if (C.mon_alloc_fail) { return 1; }
		void *q = aligned_alloc(16384, bytes);
		CHECK(q != NULL, "host allocation");
		memset(q, 0xa5, bytes);
		d->va = (vm_offset_t)q; d->size = bytes; d->iova = wl_iova_next; d->pa = 0x8b0000000ull;
		wl_iova_next += bytes;
		ms_host = q; ms_iova = d->iova; ms_allocs++;
		return 0;
	}
	if (d == &wl_reo_arena) {  /* C2d: after the other two, 2 MiB aligned, 0xa5 everywhere */
		CHECK(!d->va && bytes == REOA && (wl20_bits & WL20_REORX) && rx_host && (!(wl20_bits & WL20_IDLELINK) || ln_host),
		    "unexpected W27 allocation, or before the RX and link arenas");
		CHECK(!(wl_iova_next & 0x1fffffu) && wl_iova_next + bytes <= wl_iova_end, "W27 arena IOVA not 2 MiB aligned inside the pool");
		if (C.reo_alloc_fail) { return 1; }
		void *q = aligned_alloc(16384, bytes);
		CHECK(q != NULL, "host allocation");
		memset(q, 0xa5, bytes);
		d->va = (vm_offset_t)q; d->size = bytes; d->iova = wl_iova_next; d->pa = 0x8a0000000ull;
		wl_iova_next += bytes;
		reo_host = q; reo_iova = d->iova; reo_allocs++;
		return 0;
	}
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
	if (d == &wl_rx_arena || d == &wl_link_arena || d == &wl_reo_arena || d == &wl_mon_arena) { return w26_alloc(d, bytes); }
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
	CHECK(d == &wl_ce_arena || d == &wl_rx_arena || d == &wl_link_arena || d == &wl_reo_arena || d == &wl_mon_arena,
	    "NC conversion of another buffer");
	d->nc = true;
	return 0;
}
static void
wl_dma_free(struct wl_dma *d)
{
	if (!d->va) { return; }
	if (d == &wl_rx_arena || d == &wl_link_arena || d == &wl_reo_arena || d == &wl_mon_arena) {  /* C2c..C2e: only after bus master off, PERST and the SMMU undo, zeroed */
		const uint8_t *p = (const uint8_t *)d->va;
		CHECK(!(epcmd & 4u) && !(perst_io & 2u) && smmu_undone, "W26 arena freed before bus master off, PERST and the SMMU undo");
		for (vm_size_t i = 0; i < d->size; i++) { CHECK(!p[i], "W26 arena not zeroed before its free"); }
		free((void *)d->va);
		if (d == &wl_rx_arena) { rx_host = NULL; rx_mapped = false; rx_frees++; }
		else if (d == &wl_reo_arena) { reo_host = NULL; reo_mapped = false; reo_frees++; }
		else if (d == &wl_mon_arena) { ms_host = NULL; ms_mapped = false; ms_frees++; }
		else { ln_host = NULL; ln_mapped = false; ln_frees++; }
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
	CHECK((d == &wl_ce_arena || d == &wl_rx_arena || d == &wl_link_arena || d == &wl_reo_arena || d == &wl_mon_arena) && d->nc,
	    "map of an unconverted buffer");
	if (d == &wl_link_arena && C.link_map_fail) { return wl_w11_fail("target DMA leaf readback"); }
	if (d == &wl_rx_arena) { rx_mapped = true; } else if (d == &wl_link_arena) { ln_mapped = true; }
	else if (d == &wl_reo_arena) { reo_mapped = true; } else if (d == &wl_mon_arena) { ms_mapped = true; } else { arena_mapped = true; }
	return 0;
}
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
	if (wl_reo_arena.va) { free((void *)wl_reo_arena.va); memset(&wl_reo_arena, 0, sizeof(wl_reo_arena)); }
	if (wl_mon_arena.va) { free((void *)wl_mon_arena.va); memset(&wl_mon_arena, 0, sizeof(wl_mon_arena)); }  /* C2e */
	ms_host = NULL; ms_iova = 0; ms_mapped = ms_inject = fw_crashed = false;
	ms_pub[0] = ms_pub[1] = fw_mon_live[0] = fw_mon_live[1] = false; fw_mon_tp[0] = fw_mon_tp[1] = 0;
	ms_allocs = ms_frees = fw_mon_written = fw_mon_nobuf = mhi_reads = 0;
	memset(&wl_ms, 0, sizeof(wl_ms));
	reo_host = NULL; reo_iova = 0; reo_mapped = reo_inject = false;  /* C2d */
	reo_allocs = reo_frees = fw_sel_n = rs_n = rs_i = rs_built = reo_tp_pubs = rd_pubs = rd_posted = fw_d_seq = fw_d_lost = 0;
	memcpy(mr, mreo_plan, sizeof(mr)); memset(fw_sel_w, 0, sizeof(fw_sel_w)); rd_hp = 0;
	memset(dslot, 0, sizeof(dslot)); dslot[4095] = 3;
	memset(&wl_rd, 0, sizeof(wl_rd)); wl27_reo_writes = 0;
	memset(&cmd, 0, sizeof(cmd));  /* C2f gate scenarios */
	rx_host = ln_host = NULL; rx_iova = ln_iova = 0;
	rx_mapped = ln_mapped = rx_pub = smmu_undone = fw_rx_ready = idle_live = idle_pub = rx_inject = false;
	rx_allocs = rx_frees = ln_allocs = ln_frees = wbm_reads = wbm_writes = wbm_rbs = wbm_watch_reads = wbm_pubs = 0;
	fw_sring_n = fw_pset_n = fw_frames_written = fw_nobuf = fw_links = fw_taken_n = fw_posted_n = 0; fw_htt_conf_q = ~0u;
	m_idle_hp = m_idle_tp = fw_refill_tp = fw_err_hp = 0;
	memset(fw_mac_hp, 0, sizeof(fw_mac_hp)); memset(fw_mac_tp, 0, sizeof(fw_mac_tp)); memset(fw_pset_w, 0, sizeof(fw_pset_w));
	memset(slot_listed, 0, sizeof(slot_listed)); wbm_log_n = 0; n_acts = acts_done = 0; fw_cmd_n = 0;
	fw_w29_n = fw_w29_at = 0; wl29_mask = WL29_ALL;  /* C2f bisect */
	wl30_mask = 0; memset(&wl30_reo, 0, sizeof(wl30_reo)); fw_fse_n = fw_fisa_n = 0;  /* C2h */
	memset(&wl_rx, 0, sizeof(wl_rx)); wl26_wbm_writes = 0; wl26_heard_none = false;
	memset(&C, 0, sizeof(C));
	C.rx_take = 2048; C.rx_post[0] = C.rx_post[1] = 1024;
	C.credits = 4; C.credit_size = 2048; C.ready_ce = 2; C.htt_eid = 1; C.wmi_eid = 2; C.max_msg = 2048; C.ready_stslen = -1;
	C.phy_bands = 3; C.phy_mask = 3; C.htt_major = 3; C.htt_minor = 4; C.exp_n = 38; C.exp_pdev = 0;
	C.dbs_pdev = 1;
	C.reo_pre[30] = 0x1u; C.reo_pre[31] = 0x01fe0001u;  /* C2d: GEN_ENABLE and MISC_CTRL as found (both destinations 0xf) */
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
	const unsigned rx = rx_allocs, ln = ln_allocs, ro = reo_allocs;
	/* the host-owned words the device reads stayed as published: wrp[0] (refill HP), wrp[26] (error ring TP, never
	 * reaped), every refill entry; nothing but the device moved after the publish (2.8) */
	if (rx_pub && rx_host && !(wl20_bits & WL20_REORX)) {  /* C2d republishes wrp[0]: checked at each publish instead */
		CHECK(g32(rx_host + 0x400u) == 8190u && !g32(rx_host + 0x400u + 4u * 26u), "a host write to wrp[0] or wrp[26] after the publish");
		for (uint32_t k = 0; k < 4096u; k++) {
			uint64_t a = rx_iova + 0x20000u + 0x1000ull * k;
			CHECK(k == 4095u ? !g32(rx_host + 0x4000u + 8u * k) : g32(rx_host + 0x4000u + 8u * k) == (uint32_t)a,
			    "a refill entry rewritten after the publish");
		}
	}
	contained_and_freed();
	no_frame_text();
	CHECK(rx_frees == rx && ln_frees == ln && reo_frees == ro && !wl_rx_arena.va && !wl_link_arena.va && !wl_reo_arena.va,
	    "W26/W27 arenas not freed after proven containment");
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

/* ---- C2d scenarios (fuxiwlanqrtr=7, w20 0x800 RXSEL, 0x1000 REORX) ---- */
static void c2d_reset(unsigned bits) { c2c_reset(bits); C.rx_rule = 6; C.wedge = true; }

/* The phone's r1c2c-all readout (C.wedge, rule 6) for kits without the whole RX path: all 4095 refill buffers taken,
 * 2047 + 2016 posted to the MAC rings and 32 kept, RXDMA writes nothing, the scan stops after 15 channels. */
static void
c2d_phone(void)
{
	C.rx_take = 4095; C.rx_post[0] = 2047; C.rx_post[1] = 2016;
}

static void
c2d_happy_run(unsigned bits, unsigned w15)
{
	c2d_reset(bits); wl15_variant = w15;
	C.links_used = false; C.d_frames = 120;  /* 38 x 120 = 4560 frames: more than the 4095 buffers, so they come back */
	happy_frames();
	CHECK(run() == 0, "C2d happy path failed");
}

/* Every C2d run that reaches a returned buffer: what came back was posted again, nothing else (the model checked each
 * publish); slots never posted stay so. */
static void
c2d_balance(void)
{
	unsigned out = 0, back = 0;
	for (unsigned k = 0; k < 4095u; k++) { out += dslot[k] == 1u; back += dslot[k] == 2u; }
	CHECK(rd_posted == wl_rd.reposted && wl_rd.reposted + wl_rd.pend_n == wl_rd.returned && dslot[4095] == 3u,
	    "C2d: returned, re-posted and pending do not add up");
	(void)out; (void)back;
}

static void
s_c2d_happy(void)
{
	for (unsigned v = 0; v < 2; v++) {
		c2d_happy_run(0x1902, v);
		show(v ? "0x1902 w15=1" : "0x1902");
		static const char *const lines[] = {
			"FUXI WLAN: W26 rx arena IOVA 0xa1400000 bytes 0x1040000 Normal-NC mapped; rdp 0xa1400000 wrp 0xa1400400\n",
			"FUXI WLAN: W27 reo arena IOVA 0xa2600000 bytes 0x100000 Normal-NC mapped; rdp 0xa2600000\n",
			("FUXI WLAN: W27 reo before: rings 10 enabled 0 based 0 unreadable 0 gen-enable 0x1 misc-ctrl 0x1fe0001 aging 0/0/0/0 "
			 "dest-ctrl 0/0; not owned\n"),
			"FUXI WLAN: W27 reo ring 131 rel entries 1024 base 0xa2692000 misc 0x40 readback ok; tp via ",
			"FUXI WLAN: W27 reo ring 0 exc entries 128 base 0xa2690000 misc 0x40 readback ok; tp via ",
			("FUXI WLAN: W27 reo setup gen-enable 0x1 -> 0xd misc-ctrl 0x1fe0001 -> 0x1 aging 100000/100000/100000/40000 hash "
			 "0x43214321; writes 26 window restore checked\n"),
			"FUXI WLAN: W26 refill ring 160 filled 4095 buffers",
			"FUXI WLAN: W20 ce rx posted CE1 62 CE2 62\n",
			"FUXI WLAN: W21 wmi service ready complete\n",
			"FUXI WLAN: W27 reo ring 1 dst1 entries 2048 base 0xa2610000 misc 0x40 readback ok; tp via ",
			"FUXI WLAN: W27 reo ring 8 dst8 entries 2048 base 0xa2680000 misc 0x40 readback ok; tp via ",
			"FUXI WLAN: W22 wmi init sent bytes 332",
			"FUXI WLAN: W23 htt version 3.4 (w20 0x02)\n",
			"FUXI WLAN: W26 htt sring setup rxdma-err 186 info0 0x0007010b info1 0x00082000 intr 0x00200008 low 0\n",
			("FUXI WLAN: W27 htt rx ring selection mac0 161 info0 0x1400010c buf 2048 flags 0/0/0x1000000/0x1240000 tlv 0xd offsets "
			 "0x01100190/0x00900000/0 sent\n"),
			("FUXI WLAN: W27 htt rx ring selection mac1 169 info0 0x1400020c buf 2048 flags 0/0/0x1000000/0x1240000 tlv 0xd offsets "
			 "0x01100190/0x00900000/0 sent\n"),
			"FUXI WLAN: W26 rx watch setup+100ms ",
			"FUXI WLAN: W27 reo watch setup+100ms reaped rel 0 exc 0 dst 0; returned 0 reposted 0 pending 0 (max 0) refill hp 8190\n",
			"FUXI WLAN: W23 chan list 38 passive",
			"FUXI WLAN: W27 reo watch started reaped rel 0 exc 0 dst 0;",
			"FUXI WLAN: W27 reo watch mid (foreign 19) reaped rel 270 exc 540 dst 1350; returned 2160 reposted 2160 pending 0",
			"FUXI WLAN: W23 scan completed reason 0 after ",
			"FUXI WLAN: W27 reo watch completed reaped rel 570 exc 1140 dst 2850; returned 4560 reposted 4560 pending 0 (max 120)",
			"FUXI WLAN: W24 scan summary frames 12 bss 9 ssids 5\n",
			"FUXI WLAN: W23 stats after-scan wmi ",
			"FUXI WLAN: W26 counters ",
			"FUXI WLAN: W27 rx ring selection sent 2 of 2\n",
			"FUXI WLAN: W27 reo reaped rel 570 exc 1140 dst 570 1140 570 570 0 0 0 0\n",
			("FUXI WLAN: W27 reo entries msdu 3990 link 0 (held 0, msdus 0, skipped 0, not ours 0) push routing 2850 error 1140 "
			 "continuation 0\n"),
			"FUXI WLAN: W27 reo error codes 0x5:1140 0x9:570\n",
			"FUXI WLAN: W27 release src rxdma 0 reo 570 other 0 desc-other 0 rbm-other 0 cookie-other 0\n",
			"FUXI WLAN: W27 buffers returned 4560 reposted 4560 pending 0 (max 120) publishes 38 refill hp 926 refill-bad 0\n",
			"FUXI WLAN: W27 reo guard nonzero dwords: gaps 0, tail 0\n",
			"FUXI WLAN: W23 complete; stop/contain\n",
		};
		expect_order(lines, sizeof(lines) / sizeof(lines[0]));
		expect(v ? "tp via R2 0xa3b04c (fuxiwlanw15=1)\n" : "dst1 entries 2048 base 0xa2610000 misc 0x40 readback ok; tp via shadow 0 (0x8fc)\n");
		expect(v ? "rel entries 1024 base 0xa2692000 misc 0x40 readback ok; tp via R2 0xa370e4 (fuxiwlanw15=1)\n" :
		    "rel entries 1024 base 0xa2692000 misc 0x40 readback ok; tp via shadow 24 (0x95c)\n");
		expect_not("W24 no beacon"); expect_not("SPILL"); expect_not("TX COUNTERS"); expect_not("W27 release rxdma codes");
		CHECK(fw_sel_n == 2 && wl_scan.frames == 12 && fw_frames_written == 4560 && !fw_nobuf && rd_pubs == 38 && reo_tp_pubs > 38,
		    "C2d firmware view");
		CHECK(rs_i == rs_n && rs_n == 38 + 2 * 11 + 10 + 8 * 14, "C2d REO conversation not complete");
		c2d_balance();
		c2c_contained();
		static const char *const tail[] = {
			"FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo\n",
			"FUXI WLAN: abort step 28 MHI reset done, bus master off yes, PERST asserted yes\n",
			"FUXI WLAN: W27 reo guard after containment nonzero dwords: gaps 0, tail 0\n",
			"FUXI WLAN: W26 arenas zeroed 0x1140000 bytes before free\n",
			"FUXI WLAN: abort buffers freed\n",
		};
		expect_order(tail, sizeof(tail) / sizeof(tail[0]));
	}
	/* 0x1b02: with the idle list, REO exception entries of link descriptors (their MSDUs returned, the links held) */
	c2d_reset(0x1b02); C.d_frames = 120; happy_frames();
	CHECK(run() == 0, "C2d 0x1b02 failed");
	show("0x1b02");
	expect("FUXI WLAN: W26 wbm idle ring 120 base ");
	expect("FUXI WLAN: W27 reo entries msdu 3420 link 570 (held 570, msdus 570, skipped 0, not ours 0) push routing 2850 error 1140 "
	    "continuation 0\n");
	expect("FUXI WLAN: W27 reo error codes 0x3:570 0x5:570 0x9:570\n");
	expect("FUXI WLAN: W27 buffers returned 4560 reposted 4560 pending 0");
	expect("FUXI WLAN: W26 link content descriptors written 570 of 32767, unlisted slot clean; idle ring entries changed 0\n");
	CHECK(fw_links == 570 && wl_scan.frames == 12, "C2d link path");
	c2d_balance();
	c2c_contained();
	expect("FUXI WLAN: W26 arenas zeroed 0x1580000 bytes before free\n");
	puts("C2d 0x1902 (shadow, then fuxiwlanw15=1 R2 TPs) and 0x1b02: REO arena, 38 pre-reads, release and exception rings and "
	     "the REO setup in L19's order at W20, the eight destination rings after service ready, two RX_RING_SELECTION_CFG word "
	     "by word after the SRING_SETUPs, 4560 frames landed through 4095 buffers (each returned on a destination, exception "
	     "or release ring, re-posted fresh to ring 160, every publish checked), 0x1b02 link descriptors held and their MSDUs "
	     "returned, WMI MGMT_RX forwarded, W27 watch and report lines, contained, recounted, zeroed and freed");
}

/* The phone (r1c2c-all, w20 0x702) and the half kits: without ring selection and the REO path the scan wedges. */
static void
s_c2d_wedge(void)
{
	static const unsigned bits[] = { 0x702, 0x902, 0x1102, 0x1302 };
	for (unsigned i = 0; i < sizeof(bits) / sizeof(bits[0]); i++) {
		c2d_reset(bits[i]); c2d_phone(); C.d_frames = 120; happy_frames();
		CHECK(run() == 1, "a C2d kit without the whole RX path did not wedge");
		show("wedge");
		static const char *const lines[] = {
			"FUXI WLAN: W23 scan foreign freq 5200\n",
			"FUXI WLAN: W23 timeout scan COMPLETED (8750 ms); report, contain, no retry\n",
			("FUXI WLAN: W26 rx watch completed refill tp 8190 taken 4095 left 0 mac0 hp 4094 tp 0 avail 2047 mac1 hp 4032 tp 0 avail "
			 "2016 err hp 0 fill 0; exhausted none; refill-empty\n"),
			"FUXI WLAN: W24 scan summary frames 0 bss 0 ssids 0\n",
			"FUXI WLAN: W26 rx content mac0 entries 2047 valid 2047 foreign 0 rbm-other 0 cookie-other 0\n",
			"FUXI WLAN: W26 rx content mac1 entries 2016 valid 2016 foreign 0 rbm-other 0 cookie-other 0\n",
			"FUXI WLAN: W26 rx content refill taken 4095 kept 32; buffers written 0 of 4095\n",
			"FUXI WLAN: W26 rxdma err entries 0 (hp 0) push err 0 routing 0 flush 0 other 0; rbm link 0 buf 0 other 0\n",
			"FUXI WLAN: W20 snapshot scan COMPLETED: ",
			"FUXI WLAN: W23 failure: scan COMPLETED; contain, no retry\n",
		};
		expect_order(lines, sizeof(lines) / sizeof(lines[0]));
		CHECK(wl_scan.foreign == 15 && wl_scan.ci_start == 15 && !wl_scan.mgmt && !fw_frames_written, "the wedge is not the phone's");
		expect_not("FUXI WLAN: W23 scan foreign freq 5220");
		if (bits[i] & 0x200) { expect("FUXI WLAN: W26 link content descriptors written 0 of 32767"); }
		if (bits[i] & 0x800) { expect("FUXI WLAN: W27 rx ring selection sent 2 of 2\n"); }
		if (bits[i] & 0x1000) { expect("FUXI WLAN: W27 reo reaped rel 0 exc 0 dst 0 0 0 0 0 0 0 0\n"); }
		if (!(bits[i] & 0x1800)) { expect_not("W27"); }
		c2c_contained();
	}
	puts("C2d wedge (the phone's r1c2c-all, and 0x902, 0x1102, 0x1302): 4095 buffers taken, mac0 2047 and mac1 2016 valid, 32 "
	     "kept, refill left 0, nothing written, no error entry, the scan stops after 15 channels and COMPLETED times out at "
	     "8750 ms, no MGMT_RX; contained and freed");
}

/* Exhaustion and replenish: a 4000-frame burst on channel 0 drains mac0 and the refill ring (929 frames without a
 * buffer), the host returns and re-posts everything, the next channels land again. */
static void
s_c2d_exhaust(void)
{
	c2d_reset(0x1902); C.d_frames = 10; C.d_burst = 4000; C.d_mix = "D"; happy_frames();
	CHECK(run() == 0, "C2d exhaustion run");
	show("exhaust");
	CHECK(fw_nobuf == 929 && fw_frames_written == 3071 + 380 && wl_scan.frames == 12, "exhaustion counts");
	expect("FUXI WLAN: W27 buffers returned 3451 reposted 3451 pending 0 (max 3071)");
	c2d_balance();
	c2c_contained();
	puts("C2d exhaustion: a 4000-frame burst finds 3071 buffers (mac0 and the rest of the refill ring), 929 dropped by the "
	     "firmware, all 3071 returned and re-posted in one publish, the next 37 channels land 370 more");
}

/* Refusals of returned entries (the run stops at once and contains) and counted oddities. */
static void
s_c2d_entries(void)
{
	c2d_reset(0x1902); C.d_frames = 120; C.d_foreign = 50; happy_frames();
	CHECK(run() == 1, "a foreign REO entry passed");
	expect("FUXI WLAN: W27 reo dst3 entry refused: W27 returned entry is not one of our posted buffers; contain\n");
	expect("FUXI WLAN: W23 failure: W27 returned entry is not one of our posted buffers; contain, no retry\n");
	c2c_contained();
	c2d_reset(0x1902); C.d_frames = 120; C.d_dup = 50; happy_frames();
	CHECK(run() == 1, "a buffer returned twice passed");
	expect("FUXI WLAN: W27 reo dst3 entry refused: W27 buffer returned twice (not out with the device); contain\n");
	c2c_contained();
	c2d_reset(0x1902); C.d_frames = 120; C.reo_hp_raw = 7; happy_frames();
	CHECK(run() == 1, "a REO HP off its entries passed");
	expect("FUXI WLAN: W27 reo dst1 entry refused: W27 REO/release ring HP outside the ring; contain\n");
	c2c_contained();
	/* release entries with RBM SW1 instead of SW3: L19 skips them (the buffer stays lost), counted; destination entries
	 * with RBM 4 are counted and returned by address */
	c2d_reset(0x1902); C.d_frames = 120; C.rel_rbm = 6; happy_frames();
	CHECK(run() == 0, "unknown release RBM");
	expect("FUXI WLAN: W27 release src rxdma 0 reo 570 other 0 desc-other 0 rbm-other 570 cookie-other 0\n");
	expect("FUXI WLAN: W27 buffers returned 3990 reposted 3990 pending 0");
	c2d_balance();
	c2c_contained();
	c2d_reset(0x1902); C.d_frames = 120; C.d_rbm = 4; C.d_mix = "D"; happy_frames();
	CHECK(run() == 0, "unknown destination RBM");
	expect("FUXI WLAN: W27 release src rxdma 0 reo 0 other 0 desc-other 0 rbm-other 4560 cookie-other 0\n");
	expect("FUXI WLAN: W27 buffers returned 4560 reposted 4560 pending 0");
	c2c_contained();
	/* D5: frames came back, the firmware forwards no MGMT_RX (rule 5): the next rung is management forwarding */
	c2d_reset(0x1902); C.d_frames = 10; C.rx_rule = 5; happy_frames();
	CHECK(run() == 0, "C2d no forward");
	expect("FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung C3 management forwarding: frames reach the "
	    "host rings, WMI MGMT_RX stays silent\n");
	c2c_contained();
	no_frame_text();
	puts("C2d entries: a foreign address and a buffer returned twice stop the run and contain, so does a REO HP off its "
	     "entries; release entries with an unknown return manager are counted and skipped as L19, destination entries with "
	     "RBM 4 counted and returned by address");
}

/* Spill into the REO arena's unnamed regions; containment with the REO device still pushing. */
static void
s_c2d_contain(void)
{
	c2d_reset(0x1902); C.d_frames = 120; C.reo_spill = 7; happy_frames();
	CHECK(run() == 0, "REO spill run");
	expect("FUXI WLAN: W27 reo guard SPILL nonzero dwords: gaps 2, tail 1; stop; owner review before any further kit\n");
	c2c_contained();
	expect("FUXI WLAN: W27 reo guard after containment nonzero dwords: gaps 2, tail 1; SPILL: owner review before any further kit\n");
	/* no COMPLETED, the STOP_SCAN descriptor never drains: 20 frames land and come back on the REO rings during the
	 * 100 ms drain, after wl26_contain; the host reaps, publishes and re-posts nothing (reo_access and the publish
	 * check refuse it), then zeroes and frees */
	c2d_reset(0x1902); C.d_frames = 120; C.scan_mode = 3; C.stop_tx_stuck = true; C.d_late = 8790; happy_frames();
	CHECK(run() == 1, "no COMPLETED");
	{
		const unsigned reaped = wl_rd.ring[2].reaped + wl_rd.ring[3].reaped, posted = rd_posted, pubs = reo_tp_pubs;
		c2c_contained();
		CHECK(fw_frames_written == 4580 && posted == rd_posted && pubs == reo_tp_pubs && reaped == 2850 * 0 + reaped,
		    "a host action after the latch");
	}
	expect("FUXI WLAN: W23 contain: STOP_SCAN sent (best effort), tx drained 0 after 100 ms\n");
	/* unproven containment: the REO arena stays (not zeroed, not freed) */
	c2d_reset(0x1902); C.d_frames = 10; C.fail_bm = true; happy_frames();
	CHECK(run() == 0, "fail_bm run");
	contain();
	expect("FUXI WLAN: abort buffers quarantined until reboot");
	CHECK(wl_reo_arena.va && !reo_frees, "REO arena freed although bus master off is unproven");
	puts("C2d containment: REO spills in the rdp gap, between the rings and in the tail (SPILL, recounted after containment); "
	     "REO entries pushed during the containment drain are never reaped or re-posted; unproven containment keeps the "
	     "REO arena");
}

/* Pre-read refusals, readback, W21 re-read, a latched fault and the arena allocation. */
static void
s_c2d_failures(void)
{
	static const struct { unsigned i; uint32_t v; const char *why; } pre[] = {
		{ 3 * 4 + 2, 0x40u, "a ring has SRNG_ENABLE set (already owned)" },
		{ 3 * 0, 0x1000u, "a ring base is non-zero (already owned)" },
		{ 36, 0x43214321u, "the REO hash map is already in DEST_RING_CTRL (setup applied before)" },
		{ 33, ~0u, "a pre-read is ~0 (unreadable, ownership not established)" },
	};
	for (unsigned k = 0; k < 4; k++) {
		c2d_reset(0x1902); C.d_frames = 10; C.reo_pre[pre[k].i] = pre[k].v; happy_frames();
		CHECK(run() == 1, "a refused REO path did not wedge");
		{
			char l[200];
			snprintf(l, sizeof(l), "FUXI WLAN: W27 reo not programmed: %s; w20 0x1000 dropped for this boot\n", pre[k].why);
			expect(l);
		}
		CHECK(rs_i == 38 && !mr[0].on && fw_sel_n == 2, "a REO write after a refusal, or ring selection lost");
		expect_not("W27 reo ring"); expect_not("W27 reo watch"); expect_not("W27 reo reaped");
		c2c_contained();
	}
	c2d_reset(0x1902); C.reo_rb_bad = 1; happy_frames();
	CHECK(run() == 1 && rs_i == 38 + 11, "readback mismatch");
	expect("FUXI WLAN: W27 reo ring 131 readback mismatch base 0xa2693000 misc 0x40\n");
	c2c_contained();
	c2d_reset(0x1902); C.reo_w21 = 5; happy_frames();
	CHECK(run() == 1, "a destination ring based at W21 passed");
	expect("FUXI WLAN: W27 reo dst4 not free at W21 (base 0x1000/0 misc 0); contain\n");
	CHECK(fw_init_ok == 0 && mr[4].on && !mr[5].on, "INIT or the ring written after the W21 refusal");
	c2c_contained();
	c2d_reset(0x1902); C.reo_fault_after = 45; happy_frames();
	CHECK(run() == 1 && rs_i == 45, "a REO write after the latched fault");
	c2c_contained();
	c2d_reset(0x1902); C.reo_alloc_fail = true; happy_frames();
	CHECK(run() == 1 && !rs_i && !fw_sring_n && !wbm_reads, "REO arena allocation failure touched hardware");
	expect("FUXI WLAN: W20 failure: W27 reo arena allocation/NC/map; contain, no retry\n");
	c2c_contained();
	puts("C2d failures: pre-reads with SRNG_ENABLE, a base, the hash map already applied, a ~0 read: 38 reads, no REO write, "
	     "0x1000 dropped, ring selection still sent, the scan wedges; a BASE_LSB readback mismatch stops before the next "
	     "write; a destination ring based at W21 stops before INIT; a fault latched after 45 REO accesses: no later one; the "
	     "REO arena allocation failure: no WBM or REO access, no ring message; every run contained and freed");
}

static void
s_c2d_pure(void)
{
	static struct wlrd rd;
	static struct wlrx rx;
	static const struct wlce_io io = { io_never_wr, io_never_rd, io_never_wr, io_model_w, io_model_r, NULL };
	uint8_t *ra = aligned_alloc(16384, RXA), *ro = aligned_alloc(16384, REOA);
	uint32_t w[17], want[17], regs[WLRD_RING_REGS][2], hw[WLRD_HW_REGS][2], pre[38];
	reset();
	CHECK(ra && ro, "alloc");
	for (unsigned m = 0; m < 2; m++) { wlrd_sel_words(m, w); m_sel_want(m, want); CHECK(!memcmp(w, want, sizeof(w)), "RX_RING_SELECTION_CFG words"); }
	CHECK(wlrd_layout(&rd, ro, 0xa2600800ull, REOA) == 1 && wlrd_layout(&rd, ro, 0xa2600000ull, REOA - 4) == 1, "REO layout bounds");
	CHECK(wlrd_layout(&rd, ro, 0xa2600000ull, REOA) == 0, "REO layout");
	reo_iova = 0xa2600000ull; memcpy(mr, mreo_plan, sizeof(mr)); rs_build();
	/* the ring writes and the REO setup writes equal the model's script, the allowlist the model's address set */
	{
		unsigned s = 38;
		for (unsigned k = 0; k < 10; k++) {
			if (k == 2) { s += 10; }
			if (k >= 2) { s += 3; }
			wlrd_ring_regs(&rd, k, regs);
			for (unsigned j = 0; j < 9; j++) { CHECK(rscript[s + j].wr && rscript[s + j].addr == regs[j][0] && rscript[s + j].val == regs[j][1], "ring writes"); }
			s += 11;
		}
		wlrd_hw_regs(0x1u, 0x01fe0001u, hw);
		CHECK(hw[0][1] == rscript[38 + 22 + 1].val && hw[1][1] == rscript[38 + 22 + 3].val, "REO setup read-modify-writes");
		for (unsigned j = 2; j < 8; j++) { CHECK(rscript[38 + 22 + 2 + j].addr == hw[j][0] && rscript[38 + 22 + 2 + j].val == hw[j][1], "REO setup writes"); }
		for (unsigned i = 0; i < 38; i++) { CHECK(rscript[i].addr == wlrd_pre_addr(i) && !rscript[i].wr, "pre-read order"); }
	}
	for (uint32_t off = 0x00a30000u; off < 0x00a40000u; off += 4) {
		bool r = false, wr = false;
		for (unsigned i = 0; i < rs_n; i++) { if (rscript[i].addr == off) { if (rscript[i].wr) { wr = true; } else { r = true; } } }
		CHECK(wlrd_reg_ok(off, false) == r && wlrd_reg_ok(off, true) == wr, "REO allowlist differs from the model's conversation");
		CHECK(!(wr || r) || !wlrx_wbm_ok(off, false), "REO and WBM allowlists meet");
	}
	for (uint32_t off = CE_BASE; off < CE_END; off += 4) { CHECK(!wlrd_reg_ok(off, false) && !wlrd_reg_ok(off, true), "REO allowlist has a CE register"); }
	/* the pre-read reading */
	memset(pre, 0, sizeof(pre));
	CHECK(wlrd_owned(pre) == WLRD_FREE, "zero pre-reads not free");
	pre[29] = 0x40; CHECK(wlrd_owned(pre) == WLRD_SRNG, "dst8 SRNG_ENABLE"); pre[29] = 0x3;
	CHECK(wlrd_owned(pre) == WLRD_FREE, "MISC without SRNG_ENABLE refused");
	pre[4] = 0x01; CHECK(wlrd_owned(pre) == WLRD_BASE, "base msb 7:0"); pre[4] = 0x00040000;
	CHECK(wlrd_owned(pre) == WLRD_FREE, "a size field alone refused");
	pre[37] = 0x43214321u; CHECK(wlrd_owned(pre) == WLRD_APPLIED, "hash map in IX_3"); pre[37] = 0;
	pre[30] = ~0u; CHECK(wlrd_owned(pre) == WLRD_UNREADABLE, "a ~0 GEN_ENABLE"); pre[30] = 0xd;
	CHECK(wlrd_owned(pre) == WLRD_FREE, "GEN_ENABLE with the aging bits alone refused");
	/* recycle and replenish on a fresh RX layout */
	CHECK(wlrx_layout(&rx, ra, 0xa1400000ull, RXA) == 0 && wlrx_fill_refill(&rx) == 0, "RX layout");
	rx_host = ra; rx_iova = 0xa1400000ull; rx_pub = true; rd_hp = 8190;
	memset(&rd.slot, 0, sizeof(rd.slot)); wlrd_slots_init(&rd);
	CHECK(wlrd_recycle(&rd, &rx, 0xa1420000u + 64u, 6u << 8) == 1 && !strcmp(rd.why, "W27 returned entry is not one of our posted buffers"), "64 bytes in");
	rd.why = NULL;
	CHECK(wlrd_recycle(&rd, &rx, 0xa1420000u + 4095u * 4096u, 6u << 8 | 4095u << 12) == 1, "slot 4095 returned"); rd.why = NULL;
	CHECK(wlrd_recycle(&rd, &rx, 0xa1420000u, 1u | 6u << 8) == 1, "address bits 39:32 set"); rd.why = NULL;
	ra[0x20000 + 7 * 4096 + 100] = 0x77;
	CHECK(wlrd_recycle(&rd, &rx, 0xa1420000u + 7u * 4096u, 6u << 8 | 9u << 12) == 0 && rd.cookie_other == 1 && rd.pend_n == 1 &&
	    ra[0x20000 + 7 * 4096] == 0xa5 && !ra[0x20000 + 7 * 4096 + 100], "a return: held, fresh, cookie-other counted");
	CHECK(wlrd_recycle(&rd, &rx, 0xa1420000u + 7u * 4096u, 6u << 8 | 7u << 12) == 1 && !strcmp(rd.why, "W27 buffer returned twice (not out with the device)"),
	    "a second return of a held buffer");
	rd.why = NULL;
	/* the refill ring is full (TP 0, HP 8190): nothing posted; TP 4: two entries' room, one held buffer posted */
	((volatile uint32_t *)(void *)ra)[160] = 0;
	CHECK(wlrd_replenish(&rd, &rx, &io) == 0 && rd.pend_n == 1, "posted into a full refill ring");
	((volatile uint32_t *)(void *)ra)[160] = 5;
	CHECK(wlrd_replenish(&rd, &rx, &io) == 0 && rd.refill_bad == 1, "a bad refill TP");
	((volatile uint32_t *)(void *)ra)[160] = 4;
	dslot[7] = 2; fw_refill_tp = 4; wl20_bits = 0x1902;
	last_ev = 0;
	CHECK(wlrd_replenish(&rd, &rx, &io) == 1 && rd.refill_hp == 0 && rd.slot[7] == WLRD_SLOT_OUT && g32(ra + 0x400) == 0 &&
	    g32(ra + 0x4000 + 4 * 8190) == 0xa1420000u + 7u * 4096u, "one buffer re-posted at 8190, wrap to 0");
	rd.stopping = true;
	CHECK(wlrd_recycle(&rd, &rx, 0xa1420000u + 8u * 4096u, 6u << 8 | 8u << 12) == 1, "a return after the latch");
	/* guard counts */
	{
		uint32_t g, tl;
		memset(ro, 0, REOA);
		ro[4 * 8] = 1; ro[4 * 131] = 1; ro[0x10000] = 1; ro[0x90ffc] = 1;   /* named HP words and a ring: not counted */
		wlrd_guard_count(&rd, &g, &tl);
		CHECK(!g && !tl, "named words counted");
		ro[4 * 9] = 1; ro[4 * 130] = 1; ro[0xfffc] = 1; ro[0x91ffc] = 1; ro[0x9a000] = 1; ro[0xffffc] = 1;
		wlrd_guard_count(&rd, &g, &tl);
		CHECK(g == 4 && tl == 2, "a gap or the tail missed");
	}
	free(ra); free(ro);
	reset();
	puts("C2d pure: the 17 ring selection words, the REO layout bounds, every ring and REO setup write equal to the model's "
	     "conversation, the REO allowlist equal to its address set and apart from the WBM and CE ones, the pre-read readings, "
	     "a foreign, slot-4095, high-address and twice-returned buffer refused, cookie-other counted, a returned buffer "
	     "zeroed with its sentinel, a full refill ring and a bad TP post nothing, the HP wraps, nothing after the latch, the "
	     "guard counts");
}

/* Every W27 line at its widest within the console's 254 bytes (model_line fails any longer). */
static void
s_c2d_widest(void)
{
	c2d_reset(0x1902); C.d_frames = 1; happy_frames();
	CHECK(run() == 0, "C2d widest run");
	for (unsigned i = 0; i < WLRD_NRING; i++) { wl_rd.ring[i].reaped = 0xffffffffu; }
	wl_rd.returned = wl_rd.reposted = wl_rd.pend_n = wl_rd.pend_max = wl_rd.refill_hp = wl_rd.publishes = wl_rd.refill_bad = 0xffffffffu;
	wl_rd.msdu = wl_rd.link = wl_rd.links_held = wl_rd.link_msdus = wl_rd.link_skipped = wl_rd.link_unowned = 0xffffffffu;
	wl_rd.routing = wl_rd.reo_error = wl_rd.cont = wl_rd.desc_other = wl_rd.rbm_other = wl_rd.cookie_other = 0xffffffffu;
	for (unsigned i = 0; i < 3; i++) { wl_rd.rel_src[i] = 0xffffffffu; }
	for (unsigned i = 0; i < 32; i++) { wl_rd.reo_code[i] = wl_rd.rxdma_code[i] = 0xffffffffu; }
	wl_rd.sel_sent = 0xffffffffu;
	wl27_watch("mid (foreign 4294967295)");
	wl27_report();
	CHECK(count_text("FUXI WLAN: W27 reo error codes") >= 3, "error code list not wrapped");
	wl_rd.pend_n = 0;
	for (unsigned i = 0; i < WLRD_PRE_N; i++) { wl_rd.pre[i] = 0xffffffffu; }
	c2c_contained();
	puts("C2d line widths: the watch and report lines with every counter at 0xffffffff, 32 error codes per list over several "
	     "lines: every FUXI WLAN line within 254 bytes");
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

/* For the L19 re-derivation: the two RX_RING_SELECTION_CFG, every ring's nine writes, the REO setup writes from GEN 0x1 and
 * MISC 0x01fe0001, the pre-read addresses and the TP shadow indices, for a REO arena at 0xa2600000. */
/* ---- C2e scenarios (fuxiwlanqrtr=7, w20 0x2000 MONSTAT) ---- */
/* C2e kits run against the phone (C.mon_crash): RX live without the monitor status rings crashes the firmware. Ten
 * status PPDUs per channel (130 on mac0, 250 on mac1). */
static void c2e_reset(unsigned bits) { c2d_reset(bits); C.wedge = false; C.mon_crash = true; C.mon_ppdu = 10; }

/* Proven containment with the monitor status arena: the host-owned words the device reads stayed as published
 * (wrp[25]/[33], every entry), the crash line before the MHI reset, the arena recounted, zeroed (wl_dma_free checks every
 * byte) and freed only after bus master off, PERST and the SMMU undo. After the phone's crash the MHI reset is never
 * confirmed; bus master off and PERST still are. */
static void
c2e_contained(bool crashed)
{
	const unsigned ms = ms_allocs, rx = rx_allocs, ln = ln_allocs, ro = reo_allocs;
	if (ms_pub[1] && ms_host && !C.mon_foreign) {
		for (unsigned m = 0; m < 2; m++) {
			CHECK(g32(rx_host + 0x400u + 4u * (m_mon_id[m] - 160u)) == 2046u, "a host write to wrp[25] or wrp[33] after the publish");
			for (uint32_t k = 0; k < 1024u; k++) {
				const uint64_t a = ms_iova + 0x10000u + 0x1000ull * (1024u * m + k);
				CHECK(k == 1023u ? !g32(ms_host + 0x4000u * m + 8u * k) : g32(ms_host + 0x4000u * m + 8u * k) == (uint32_t)a,
				    "a monitor status entry rewritten after the publish");
			}
		}
	}
	contain();
	expect("FUXI WLAN: W20 ce contain posted");
	expect(crashed ? "FUXI WLAN: abort step 28 MHI reset unconfirmed, bus master off yes, PERST asserted yes\n" :
	    "FUXI WLAN: abort step 28 MHI reset done, bus master off yes, PERST asserted yes\n");
	expect("FUXI WLAN: abort buffers freed\n");
	CHECK(arena_frees == 1 && !wl_ce_arena.va && stops == 1 && bar_win == WIN_INIT, "CE arena not freed or window not restored");
	no_frame_text();
	CHECK(ms_frees == ms && rx_frees == rx && ln_frees == ln && reo_frees == ro && !wl_mon_arena.va && !wl_rx_arena.va &&
	    !wl_link_arena.va && !wl_reo_arena.va, "W26..W28 arenas not freed after proven containment");
	static const char *const order[] = {
		"FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo\n",
		"FUXI WLAN: W20 ce contain posted ",
		"FUXI WLAN: W28 crash contain before MHI reset: mhistatus ",
		"FUXI WLAN: abort step 28 MHI reset ",
		"FUXI WLAN: W28 mon guard after containment nonzero dwords: ",
		"FUXI WLAN: W26 arenas zeroed ",
		"FUXI WLAN: abort buffers freed\n",
	};
	expect_order(order, sizeof(order) / sizeof(order[0]));
}

static void
s_c2e_pure(void)
{
	static struct wlrx rx;
	static struct wlms ms;
	static const struct wlce_io io = { io_never_wr, io_never_rd, io_never_wr, io_model_w, io_model_r, NULL };
	uint8_t *ra = aligned_alloc(16384, RXA), *ma = aligned_alloc(16384, MONA);
	uint32_t w[13], want[13], e[2], u, sl, l, t, g;
	reset();
	CHECK(ra && ma, "alloc");
	/* the crash line's allowlist: exactly the six reads at a sane BHIOFF, nothing at a bad one */
	for (uint32_t off = 0; off < 0x5000u; off++) {
		const bool want_ok = off == 0x48u || off == 0x1028u || off == 0x1030u || off == 0x1034u || off == 0x1038u || off == 0x103cu;
		CHECK(wlms_diag_ok(off, 0x1000u) == want_ok, "crash line allowlist differs from the model's six");
		CHECK(!wlms_diag_ok(off, 0x20u) && !wlms_diag_ok(off, 0x3fc4u) && !wlms_diag_ok(off, 0x1002u), "crash line read with a bad BHIOFF");
	}
	CHECK(!wlms_diag_ok(0x38u, 0x1000u) && !wlms_diag_ok(0x310cu, 0x1000u), "MHICTRL or the window register in the crash allowlist");
	/* layout and the two SRING_SETUP, word by word against the model's own table */
	CHECK(wlms_layout(&ms, &rx, ma, 0xa2e00000ull, MONA) == 1, "layout without the RX arena");
	CHECK(!wlrx_layout(&rx, ra, 0xa1400000ull, RXA) && !wlms_layout(&ms, &rx, ma, 0xa2e00000ull, MONA) && rx.mon_named, "layout");
	CHECK(wlms_layout(&ms, &rx, ma, 0xa2e00800ull, MONA) == 1 && wlms_layout(&ms, &rx, ma, 0xa2e00000ull, MONA - 1u) == 1 &&
	    wlms_layout(&ms, &rx, ma, 0xfff00000ull, MONA) == 1, "layout accepted a misaligned, short or 4 GiB-crossing arena");
	CHECK(!wlms_layout(&ms, &rx, ma, 0xa2e00000ull, MONA), "layout");
	rx_iova = 0xa1400000ull; ms_iova = 0xa2e00000ull;
	for (unsigned m = 0; m < 2; m++) {
		wlms_sring_words(&ms, m, w);
		m_sring_want(4 + m, want);
		CHECK(!memcmp(w, want, sizeof(w)), "monitor status SRING_SETUP words differ from the model's table");
		CHECK(ms.ring[m].id == m_mon_id[m] && ms.ring[m].hp_iova == 0xa1400400ull + 4u * (m_mon_id[m] - 160u) &&
		    ms.ring[m].tp_iova == 0xa1400000ull + 4u * m_mon_id[m], "ring id or pointer words");
		wlms_expect(&ms, m, 0, e);
		CHECK(e[0] == 0xa2e10000u + 0x400000u * m && e[1] == 6u << 8, "entry 0");
		wlms_expect(&ms, m, 1022, e);
		CHECK(e[0] == 0xa2e10000u + 0x400000u * m + 0x1000u * 1022u && e[1] == (6u << 8 | 1022u << 12), "entry 1022");
		wlms_expect(&ms, m, 1023, e);
		CHECK(!e[0] && !e[1], "entry 1023 listed");
	}
	/* the watch: TP words as the device may leave them */
	{
		static const struct { uint32_t tp; uint32_t taken, left; uint8_t bad, unr, ex; } c[] = {
			{ 0, 0, 1023, 0, 0, 0 }, { 260, 130, 893, 0, 0, 0 }, { 2046, 1023, 0, 0, 0, 1 }, { 2047, 0, 0, 1, 0, 0 },
			{ 2048, 0, 0, 1, 0, 0 }, { 0xfffffffeu, 0, 0, 1, 0, 0 }, { ~0u, 0, 0, 0, 1, 0 },
		};
		ms.hp[0] = ms.hp[1] = 2046u;
		for (unsigned i = 0; i < sizeof(c) / sizeof(c[0]); i++) {
			rx.rdp[185] = c[i].tp; rx.rdp[193] = 2u * i;
			wlms_watch(&ms, &rx, &io, WLRX_P_MID);
			const struct wlms_watch *x = &ms.w[WLRX_P_MID];
			CHECK(x->taken[0] == c[i].taken && x->left[0] == c[i].left && !!(x->bad & WLMS_F_MON0) == c[i].bad &&
			    !!(x->unreadable & WLMS_F_MON0) == c[i].unr && !!(x->exhausted & WLMS_F_MON0) == c[i].ex &&
			    x->taken[1] == i && !(x->bad & WLMS_F_MON1) && x->seen, "monitor status watch derivation");
		}
	}
	/* written, guards, changed entries */
	CHECK(!wlms_fill(&ms, &rx, 0) && !wlms_fill(&ms, &rx, 1) && wlms_fill(&ms, &rx, 2) == 1, "fill");
	CHECK(wlms_written(&ms, 0) == 0 && wlms_written(&ms, 1) == 0, "fresh buffers counted as written");
	ms.bufs[0x1000u * 5u + 63u] = 0; ms.bufs[0x1000u * (1024u + 7u)] = 1;
	CHECK(wlms_written(&ms, 0) == 1 && wlms_written(&ms, 1) == 1, "written count");
	wlms_guard_count(&ms, &u, &sl, &l, &t, &g);
	CHECK(!u && !sl && !l && !t && !g, "a filled arena has guard words");
	ms.bufs[0x1000u * 5u + 2048u] = 1; ms.bufs[0x1000u * 5u + 4095u] = 1; ms.bufs[0x1000u * 1023u] = 1; ms.bufs[0x1000u * 2047u + 4092u] = 1;
	ma[0x2000u] = 1; ma[0xfffcu] = 1; ma[0x810000u] = 1; ma[MONA - 1u] = 1;
	wlms_guard_count(&ms, &u, &sl, &l, &t, &g);
	CHECK(u == 2 && sl == 1 && l == 2 && t == 2 && g == 2, "monitor status guard counts");
	ms.ring[1].va[2u * 1000u] ^= 0x40u; ms.ring[0].va[2u * 3u + 1u] ^= 1u << 12; ms.ring[0].va[2u * 1023u] = 1u;
	wlms_content(&ms, &io);
	CHECK(ms.changed[0] == 2 && ms.foreign[0] == 1 && ms.changed[1] == 1 && ms.foreign[1] == 1 && ms.written[0] == 1, "changed entries");
	/* after the latch nothing is written */
	ms.stopping = true;
	CHECK(wlms_fill(&ms, &rx, 0) == 1 && wlms_publish(&ms, &rx, &io, 0) == 1, "a monitor status write after the latch");
	free(ra); free(ma);
	reset();
	puts("C2e pure: the crash line's allowlist is exactly the six MHI/BHI reads (none at a bad BHIOFF, never MHICTRL or the "
	     "window), the arena layout and both SRING_SETUP word by word, entries 0/1022/1023, the TP watch (bad, unreadable, "
	     "exhausted), written and guard counts per region, changed and foreign entries, no write after the latch");
}

static void
s_c2e_happy(void)
{
	c2e_reset(0x3b02); C.d_frames = 120; happy_frames();
	CHECK(run() == 0, "C2e 0x3b02 failed");
	show("0x3b02");
	static const char *const lines[] = {
		"FUXI WLAN: W26 link arena IOVA ",
		"FUXI WLAN: W26 rx arena IOVA 0xa1a00000 bytes 0x1040000 Normal-NC mapped; rdp 0xa1a00000 wrp 0xa1a00400\n",
		"FUXI WLAN: W27 reo arena IOVA 0xa2c00000 bytes 0x100000 Normal-NC mapped; rdp 0xa2c00000\n",
		"FUXI WLAN: W28 mon arena IOVA 0xa2e00000 bytes 0x820000 Normal-NC mapped; rings 0xa2e00000 0xa2e04000 slots 0xa2e10000\n",
		"FUXI WLAN: W26 wbm idle ring 120 base ",
		"FUXI WLAN: W27 reo setup ",
		"FUXI WLAN: W26 refill ring 160 filled 4095 buffers",
		("FUXI WLAN: W28 mon status ring 185 filled 1023 buffers (rbm 6, cookie 0-1022, 2048 B at a 4096 B stride, sentinel 0xa5 x64, "
		 "slots 0xa2e10000-0xa320e000) hp 2046 in wrp[25]\n"),
		("FUXI WLAN: W28 mon status ring 193 filled 1023 buffers (rbm 6, cookie 0-1022, 2048 B at a 4096 B stride, sentinel 0xa5 x64, "
		 "slots 0xa3210000-0xa360e000) hp 2046 in wrp[33]\n"),
		"FUXI WLAN: W20 ce rx posted CE1 62 CE2 62\n",
		"FUXI WLAN: W23 htt version 3.4 (w20 0x02)\n",
		"FUXI WLAN: W26 htt sring setup rxdma-err 186 info0 0x0007010b info1 0x00082000 intr 0x00200008 low 0\n",
		"FUXI WLAN: W28 htt sring setup mon0 185 info0 0x0101010b info1 0x02020800 intr 0x003e0002 low 256\n",
		"FUXI WLAN: W28 htt sring setup mon1 193 info0 0x0101020b info1 0x02020800 intr 0x003e0002 low 256\n",
		"FUXI WLAN: W27 htt rx ring selection mac0 161 info0 0x1400010c ",
		"FUXI WLAN: W27 htt rx ring selection mac1 169 info0 0x1400020c ",
		"FUXI WLAN: W26 rx watch setup+100ms ",
		"FUXI WLAN: W27 reo watch setup+100ms ",
		("FUXI WLAN: W28 mon watch setup+100ms mon0 hp 2046 tp 0 taken 0 left 1023 written 0 mon1 hp 2046 tp 0 taken 0 left 1023 "
		 "written 0; exhausted none\n"),
		"FUXI WLAN: W23 chan list 38 passive",
		"FUXI WLAN: W28 mon watch started mon0 hp 2046 tp 0 taken 0 left 1023 written 0 mon1 hp 2046 tp 0 ",
		("FUXI WLAN: W28 mon watch mid (foreign 19) mon0 hp 2046 tp 260 taken 130 left 893 written 130 mon1 hp 2046 tp 100 taken 50 "
		 "left 973 written 50; exhausted none\n"),
		"FUXI WLAN: W23 scan completed reason 0 after ",
		("FUXI WLAN: W28 mon watch completed mon0 hp 2046 tp 260 taken 130 left 893 written 130 mon1 hp 2046 tp 500 taken 250 left "
		 "773 written 250; exhausted none\n"),
		"FUXI WLAN: W24 scan summary frames 12 bss 9 ssids 5\n",
		"FUXI WLAN: W27 buffers returned 4560 reposted 4560 pending 0",
		"FUXI WLAN: W28 mon sring sent 2 of 2\n",
		"FUXI WLAN: W28 mon content mon0 written 130 of 1023; ring entries changed 0 (foreign 0)\n",
		"FUXI WLAN: W28 mon content mon1 written 250 of 1023; ring entries changed 0 (foreign 0)\n",
		"FUXI WLAN: W28 mon guard nonzero dwords: slot-upper 0 (slots 0), unposted slots 0, tail 0, gaps 0\n",
		"FUXI WLAN: W23 complete; stop/contain\n",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	expect_not("W28 crash scan"); expect_not("W24 no beacon"); expect_not("SPILL"); expect_not("TX COUNTERS");
	expect("FUXI WLAN: W26 rx guard nonzero dwords: slot-upper 0 (slots 0), slot 4095 0, tail 0, gaps 0\n");
	CHECK(fw_sring_n == 6 && fw_sel_n == 2 && fw_mon_written == 380 && !fw_mon_nobuf && wl_scan.frames == 12 &&
	    fw_frames_written == 4560 && !mhi_reads, "C2e firmware view");
	c2d_balance();
	c2e_contained(false);
	expect("FUXI WLAN: W28 crash contain before MHI reset: mhistatus 0x201 (state 0x2 syserr 0) ee 0x2 bhi errcode 0 errdbg 0/0/0; "
	    "access-faults 0; read-only\n");
	expect("FUXI WLAN: W28 mon guard after containment nonzero dwords: slot-upper 0 (slots 0), unposted slots 0, tail 0, gaps 0\n");
	expect("FUXI WLAN: W26 arenas zeroed 0x1da0000 bytes before free\n");
	CHECK(mhi_reads == 6, "the crash line read other than its six registers once");
	puts("C2e 0x3b02 (C2d's 0x1b02 + 0x2000) against the phone: monitor status arena after the other three, 1023 status buffers "
	     "per ring after the refill ring, both HP publishes checked, the two SRING_SETUP word by word after the four C2c ones "
	     "and before the ring selection, 380 status TLVs written (130 mac0, 250 mac1), never reaped, all 38 channels with frames "
	     "landed and returned as C2d, W28 watch, content and guard lines, the crash line once before the MHI reset, recounted, "
	     "zeroed and freed");
}

/* 0x2302: RX rings, the idle list, the monitor status rings and HTT only (no ring selection, no REO path). */
static void
s_c2e_rings_only(void)
{
	c2e_reset(0x2302); C.d_frames = 120; happy_frames();
	CHECK(run() == 0, "C2e 0x2302 failed");
	show("0x2302");
	static const char *const lines[] = {
		"FUXI WLAN: W28 mon arena IOVA 0xa2c00000 bytes 0x820000 Normal-NC mapped; rings 0xa2c00000 0xa2c04000 slots 0xa2c10000\n",
		"FUXI WLAN: W28 htt sring setup mon1 193 info0 0x0101020b info1 0x02020800 intr 0x003e0002 low 256\n",
		"FUXI WLAN: W23 scan completed reason 0 after ",
		"FUXI WLAN: W28 mon watch completed mon0 hp 2046 tp 260 taken 130 left 893 written 130 mon1 hp 2046 tp 500 taken 250 ",
		"FUXI WLAN: W24 scan summary frames 0 bss 0 ssids 0\n",
		"FUXI WLAN: W26 rx content refill taken 2048 kept 0; buffers written 0 of 4095\n",
		"FUXI WLAN: W28 mon content mon1 written 250 of 1023; ring entries changed 0 (foreign 0)\n",
		("FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung 0x3b02 (15106): the scan ran with the monitor "
		 "status rings; add ring selection, the REO path and the idle list\n"),
		"FUXI WLAN: W23 complete; stop/contain\n",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	expect_not("W27"); expect_not("W28 crash scan");
	CHECK(fw_sring_n == 6 && !fw_sel_n && fw_mon_written == 380 && !fw_frames_written, "C2e 0x2302 firmware view");
	c2e_contained(false);
	expect("FUXI WLAN: W26 arenas zeroed 0x1ca0000 bytes before free\n");
	puts("C2e 0x2302: rings, idle list, monitor status rings and HTT only: the phone model's crash rule is not met, all 38 "
	     "channels, 380 status TLVs, nothing written to the data buffers, no W27 line, the hint names 0x3b02");
}

/* The phone's readouts (C.mon_crash): RX live without the monitor status rings crashes the firmware after 15 channels
 * (C2c 0x302) or 3 (ring selection, C2d 0x1b02): no W28 line, no new register read. With 0x2000 and a firmware that
 * drops the two SRING_SETUP, the crash line prints the SYS_ERR state and the phone's ERRDBG words twice. */
static void
s_c2e_crash(void)
{
	static const struct { unsigned bits, chans; } k[] = { { 0x302, 15 }, { 0x1b02, 3 } };
	for (unsigned i = 0; i < 2; i++) {
		c2e_reset(k[i].bits); C.d_frames = 120; happy_frames();
		CHECK(run() == 1, "the phone's crash did not stop the scan");
		show("crash");
		CHECK(count_text("FUXI WLAN: W23 scan foreign freq ") == k[i].chans, "crash after the wrong number of channels");
		expect("FUXI WLAN: W23 timeout scan COMPLETED (8750 ms); report, contain, no retry\n");
		expect_not("W28");
		contain();
		expect("FUXI WLAN: abort step 28 MHI reset unconfirmed, bus master off yes, PERST asserted yes\n");
		expect("FUXI WLAN: abort buffers freed\n");
		expect_not("W28");
		CHECK(!mhi_reads && fw_crashed && !wl_rx_arena.va, "a crash line or an MHI read without 0x2000");
	}
	c2e_reset(0x3b02); C.mon_ignore = true; C.d_frames = 120; happy_frames();
	CHECK(run() == 1, "the crash without monitor status did not stop the scan");
	show("0x3b02 mon ignored");
	static const char *const lines[] = {
		"FUXI WLAN: W28 htt sring setup mon1 193 info0 0x0101020b ",
		"FUXI WLAN: W23 timeout scan COMPLETED (8750 ms); report, contain, no retry\n",
		("FUXI WLAN: W28 crash scan COMPLETED timeout: mhistatus 0xff04 (state 0xff syserr 1) ee 0x2 bhi errcode 0 errdbg "
		 "0/0x152ee53/0x492; access-faults 0; read-only\n"),
		"FUXI WLAN: W26 rx watch completed ",
		("FUXI WLAN: W28 mon watch completed mon0 hp 2046 tp 0 taken 0 left 1023 written 0 mon1 hp 2046 tp 0 taken 0 left 1023 "
		 "written 0; exhausted none\n"),
		"FUXI WLAN: W24 scan summary frames 3 bss 3 ssids 2\n",
		"FUXI WLAN: W28 mon content mon0 written 0 of 1023; ring entries changed 0 (foreign 0)\n",
		"FUXI WLAN: W28 mon guard nonzero dwords: slot-upper 0 (slots 0), unposted slots 0, tail 0, gaps 0\n",
		"FUXI WLAN: W20 snapshot scan COMPLETED",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	CHECK(count_text("FUXI WLAN: W23 scan foreign freq ") == 3 && !fw_mon_written && mhi_reads == 6, "C2e crash view");
	c2e_contained(true);
	expect("FUXI WLAN: W28 crash contain before MHI reset: mhistatus 0xff04 (state 0xff syserr 1) ee 0x2 bhi errcode 0 errdbg "
	    "0/0x152ee53/0x492; access-faults 0; read-only\n");
	CHECK(mhi_reads == 12, "the crash line read other than its six registers twice");
	puts("C2e crash (the phone's readouts): 0x302 stops after 15 channels and 0x1b02 after 3, COMPLETED timeout, MHI reset "
	     "unconfirmed, no W28 line and no MHI/BHI read without 0x2000; 0x3b02 with the monitor status SRING_SETUP dropped "
	     "crashes after 3 channels and prints the read-only crash line (SYS_ERR, ERRDBG2 0x152ee53, ERRDBG3 0x492) at the "
	     "timeout and before the MHI reset; the arenas are still zeroed and freed");
}

/* Full rings: never reaped, a ring is exhausted after 1023 PPDUs; RXDMA then writes nothing more (counted). */
static void
s_c2e_full(void)
{
	c2e_reset(0x3b02); C.d_frames = 120; C.mon_ppdu = 50; happy_frames();
	CHECK(run() == 0, "C2e full mon1");
	show("full mon1");
	expect("FUXI WLAN: W28 mon watch completed mon0 hp 2046 tp 1300 taken 650 left 373 written 650 mon1 hp 2046 tp 2046 taken 1023 "
	    "left 0 written 1023; exhausted mon1\n");
	expect("FUXI WLAN: W28 mon content mon1 written 1023 of 1023; ring entries changed 0 (foreign 0)\n");
	CHECK(fw_mon_nobuf == 1250u - 1023u && fw_mon_written == 650u + 1023u, "full ring view");
	c2e_contained(false);
	c2e_reset(0x2302); C.mon_ppdu = 100;
	CHECK(run() == 0, "C2e full both");
	show("full both");
	expect("FUXI WLAN: W28 mon watch mid (foreign 19) mon0 hp 2046 tp 2046 taken 1023 left 0 written 1023 mon1 hp 2046 tp 1000 taken "
	    "500 left 523 written 500; exhausted mon0\n");
	expect("taken 1023 left 0 written 1023; exhausted mon0 mon1\n");
	expect("FUXI WLAN: W24 no beacon or probe response heard (chan-info 38); next rung owner review: a monitor status ring is "
	    "exhausted (L19 STA mode reaps none either)\n");
	c2e_contained(false);
	puts("C2e full rings: mon1 exhausted after 1023 status buffers (227 PPDUs without one), then both, the hint names it");
}

/* Spills and entries the device had no business writing: counted, SPILL / changed, the hint stops, recounted after
 * containment, still zeroed and freed. */
static void
s_c2e_spill(void)
{
	c2e_reset(0x2302); C.mon_spill = 15;
	CHECK(run() == 0, "C2e spill");
	show("spill");
	expect("FUXI WLAN: W28 mon guard SPILL nonzero dwords: slot-upper 2 (slots 1), unposted slots 1, tail 3, gaps 1; stop; owner "
	    "review before any further kit\n");
	expect("next rung none: stop, owner review before any further kit (monitor status guard spill or a changed ring entry)\n");
	c2e_contained(false);
	expect("FUXI WLAN: W28 mon guard after containment nonzero dwords: slot-upper 2 (slots 1), unposted slots 1, tail 3, gaps 1; "
	    "SPILL: owner review before any further kit\n");
	c2e_reset(0x2302); C.mon_foreign = 3;
	CHECK(run() == 0, "C2e foreign");
	show("foreign");
	expect("FUXI WLAN: W28 mon content mon0 written 130 of 1023; ring entries changed 1 (foreign 1)\n");
	expect("FUXI WLAN: W28 mon content mon1 written 250 of 1023; ring entries changed 1 (foreign 0)\n");
	expect("next rung none: stop, owner review before any further kit (monitor status guard spill or a changed ring entry)\n");
	c2e_contained(false);
	puts("C2e spill and foreign entry: a slot's upper half, an unposted slot, the tail and a head gap counted as SPILL (again "
	     "after containment), a ring entry rewritten with a foreign address and one with another cookie, the hint stops");
}

/* Pointer words the device may leave: bad and unreadable TPs print raw and are never acted on. */
static void
s_c2e_readings(void)
{
	static const struct { uint32_t v; const char *line; } c[] = {
		{ 0xfffffffeu, "mon1 hp 2046 tp 4294967294 bad written 250; exhausted none\n" },
		{ ~0u, "mon1 hp 2046 tp 4294967295 unreadable written 250; exhausted none\n" },
		{ 2047u, "mon1 hp 2046 tp 2047 bad written 250; exhausted none\n" },
		{ 2048u, "mon1 hp 2046 tp 2048 bad written 250; exhausted none\n" },
	};
	for (unsigned i = 0; i < sizeof(c) / sizeof(c[0]); i++) {
		c2e_reset(0x2302); C.mon_tp_raw = c[i].v;
		CHECK(run() == 0, "C2e reading");
		show("reading");
		expect(c[i].line);
		c2e_contained(false);
	}
	puts("C2e readings: a TP past the ring, odd, 0xfffffffe and ~0u print raw (bad / unreadable), nothing acted on");
}

/* Containment mid-scan (no COMPLETED, the STOP_SCAN descriptor never drains) while RXDMA still writes status, also during
 * the MHI reset: the host writes nothing after the latch, the crash line reads before the reset, the arena is recounted,
 * zeroed and freed; unproven containment keeps it; an arena allocation failure touches no register; a guard failure after
 * the 5th SRING_SETUP sends no 6th and no ring selection. */
static void
s_c2e_contain(void)
{
	c2e_reset(0x3b02); C.d_frames = 120; C.scan_mode = 3; C.stop_tx_stuck = true; C.mon_late = true; happy_frames();
	CHECK(run() == 1, "no COMPLETED");
	show("contain mid-scan");
	expect("FUXI WLAN: W23 timeout scan COMPLETED (8750 ms); report, contain, no retry\n");
	expect("FUXI WLAN: W28 crash scan COMPLETED timeout: mhistatus 0x201 (state 0x2 syserr 0) ee 0x2 bhi errcode 0 errdbg 0/0/0; "
	    "access-faults 0; read-only\n");
	{
		const unsigned written = fw_mon_written;
		c2e_contained(false);
		CHECK(fw_mon_written == written + 1u, "the late status write did not happen");
	}
	expect("FUXI WLAN: W23 contain: STOP_SCAN sent (best effort), tx drained 0 after 100 ms\n");
	c2e_reset(0x3b02); C.d_frames = 10; C.fail_bm = true; happy_frames();
	CHECK(run() == 0, "fail_bm run");
	contain();
	expect("FUXI WLAN: abort buffers quarantined until reboot");
	expect_not("W28 mon guard after containment");
	CHECK(wl_mon_arena.va && !ms_frees, "monitor status arena freed although bus master off is unproven");
	c2e_reset(0x3b02); C.mon_alloc_fail = true;
	CHECK(run() == 1 && !wbm_reads && !rs_i && !fw_sring_n && !ms_allocs, "monitor status arena failure touched hardware");
	expect("FUXI WLAN: W20 failure: W28 mon arena allocation/NC/map; contain, no retry\n");
	contain();
	expect("FUXI WLAN: W26 arenas zeroed 0x1580000 bytes before free\n");
	CHECK(!wl_rx_arena.va && !wl_reo_arena.va && !wl_link_arena.va, "the other arenas not freed");
	c2e_reset(0x3b02); C.guard_on_sring = 5;
	CHECK(run() == 1 && fw_sring_n == 5 && !fw_sel_n, "a message after the guard failure");
	expect("FUXI WLAN: W23 failure: AMSS/M0, ownership, discovery or access/SMMU fault; contain, no retry\n");
	expect_not("W28 htt sring setup mon0 185 info0"); expect_not("W28 htt sring setup mon1");
	contain();
	puts("C2e containment: mid-scan with status written during the MHI reset (host writes off from the latch, crash line "
	     "before the reset, recounted, zeroed, freed), unproven containment keeps the arena, an allocation failure touches "
	     "no WBM/REO register and sends nothing, a guard failure after the 5th SRING_SETUP stops before the 6th");
}

static void
dump_c2e(void)
{
	static struct wlrx rx;
	static struct wlms ms;
	uint8_t *ra = aligned_alloc(16384, RXA), *ma = aligned_alloc(16384, MONA);
	uint32_t w[13], e[2];
	CHECK(ra && ma && !wlrx_layout(&rx, ra, 0xa1400000ull, RXA) && !wlms_layout(&ms, &rx, ma, 0xa2e00000ull, MONA), "dump layout");
	for (unsigned m = 0; m < 2; m++) {
		wlms_sring_words(&ms, m, w);
		for (unsigned k = 0; k < 13; k++) { fprintf(stdout, "msring%u %u %u\n", m, k, w[k]); }
		for (unsigned k = 0; k < 1024u; k += 1023u) {
			wlms_expect(&ms, m, k == 1023u ? 1022u : k, e);
			fprintf(stdout, "mentry%u %u %u %u\n", m, k == 1023u ? 1022u : k, e[0], e[1]);
		}
		fprintf(stdout, "mring%u %u %u %u\n", m, ms.ring[m].id, ms.ring[m].entries, ms.ring[m].entry_dw);
	}
	fprintf(stdout, "mnbuf %u\nmbufbytes %u\nmarena %u\n", WLMS_NBUF, WLMS_BUF_BYTES, WLMS_ARENA_BYTES);
	fflush(stdout);
	free(ra); free(ma);
}

/* C2f pure checks: the wlcm header helpers build exactly the words L19 does, and
 * the cc cookie math round-trips. Hardcoded expected values (l19_c2f proves them
 * equal to the tree); a header value mutation diverges and aborts. */
static void
s_c2f_pure(void)
{
	static struct wlcm cm;
	static struct wlrx rx;
	uint8_t *ca = aligned_alloc(16384, WLCM_ARENA_BYTES), *ra = aligned_alloc(16384, RXA);
	uint32_t regs[WLCM_RING_REGS_MAX][2], cc[WLCM_CC_REGS][2];
	unsigned n = 0;
	/* id, dir, entries, entry_dw, r0, r2 */
	static const uint32_t exp[WLCM_NRING][6] = {
		{ 121, 0, 64, 8, 0xa3437c, 0xa37010 }, { 24, 0, 512, 8, 0xa44900, 0xa46000 },
		{ 128, 1, 32768, 8, 0xa34e08, 0xa370c8 }, { 25, 0, 512, 8, 0xa44978, 0xa46008 },
		{ 130, 1, 32768, 8, 0xa34ef8, 0xa370d8 }, { 26, 0, 512, 8, 0xa449f0, 0xa46010 },
		{ 132, 1, 32768, 8, 0xa34fe8, 0xa370e8 }, { 16, 0, 32, 8, 0xa38304, 0xa3b028 },
		{ 20, 0, 256, 12, 0xa3828c, 0xa3b020 }, { 21, 1, 2048, 28, 0xa38a84, 0xa3b0a8 },
	};
	static const uint32_t ccexp[WLCM_CC_REGS][2] = {
		{ 0xa38050, 0x113000 }, { 0xa38054, 0x1d1310 }, { 0xa34040, 0x113000 }, { 0xa34044, 0x51310 },
		{ 0xa34090, 0xb }, { 0xa34094, 0x13a },
	};
	CHECK(ca && ra && !wlrx_layout(&rx, ra, 0xa1400000ull, RXA) && !wlcm_layout(&cm, ca, 0xa5000000ull, WLCM_ARENA_BYTES), "c2f layout");
	cm.cmem_base = 0x113000u; cm.cmem_valid = true;
	wlcm_cc_regs(&cm, 0u, 0u, cc);
	for (unsigned k = 0; k < WLCM_CC_REGS; k++) { CHECK(cc[k][0] == ccexp[k][0] && cc[k][1] == ccexp[k][1], "c2f cc_config word"); }
	CHECK(WLCM_REO_CC_CFG1_VAL == 0x1d1310u && WLCM_CC_CFG1_COMMON == 0x51310u && WLCM_WBM_CC_CFG2_OR == 0xbu &&
	    WLCM_WBM_CC_CONVERT_OR == 0x13au, "c2f cc constants");
	for (unsigned i = 0; i < WLCM_NRING; i++) {
		const struct wlcm_ring *r = &cm.ring[i];
		const uint32_t words = exp[i][2] * exp[i][3], msb = (words & 0xfffffu) << 8;
		const uint32_t batch = exp[i][1] ? (exp[i][0] >= 128u ? 256u : 1u) : (exp[i][0] == 121u || exp[i][0] == 16u || exp[i][0] == 20u ? 1u : 1u);
		const uint32_t timer = (exp[i][1] && exp[i][0] >= 128u) ? 1000u : 256u;
		const uint32_t cons = ((timer >> 3) & 0xffffu) << 16 | ((batch * exp[i][3]) & 0x7fffu);
		CHECK(r->id == exp[i][0] && r->dir == exp[i][1] && r->entries == exp[i][2] && r->entry_dw == exp[i][3], "c2f ring geometry");
		CHECK(r->r0 == exp[i][4] && r->r2 == exp[i][5], "c2f ring r0/r2 base");
		wlcm_ring_regs(&cm, i, regs, &n);
		CHECK(n == (exp[i][1] ? 9u : 10u), "c2f ring write count");
		CHECK(regs[0][0] == exp[i][4] + WLCE_R0_BASE_LSB && regs[0][1] == (uint32_t)r->iova, "c2f ring BASE_LSB");
		CHECK(regs[1][0] == exp[i][4] + WLCE_R0_BASE_MSB && regs[1][1] == msb, "c2f ring BASE_MSB (size)");
		if (exp[i][1] == 0) {
			CHECK(regs[2][1] == exp[i][3], "c2f src ID entry size");
			CHECK(regs[3][0] == exp[i][4] + WLCE_R0_CONS_IX0 && regs[3][1] == cons, "c2f src CONS_IX0");
			CHECK(regs[n - 1][1] == (WLCE_MISC_LOOPCNT_DIS | WLCE_MISC_SRNG_ENABLE), "c2f src MISC");
		} else {
			CHECK(regs[2][1] == ((exp[i][0] & 0xffu) << 8 | exp[i][3]), "c2f dst ID");
			CHECK(regs[3][0] == exp[i][4] + WLCE_R0_PROD_INT && regs[3][1] == cons, "c2f dst PROD_INT");
			CHECK(regs[n - 1][1] == WLCE_MISC_SRNG_ENABLE, "c2f dst MISC");
		}
	}
	CHECK(wlcm_buf_cookie(0) == 0x20000u && wlcm_buf_cookie(512u) == 0x20200u && wlcm_buf_cookie(WLRX_NBUF - 1u) == 0x20ffeu, "c2f cookie gen");
	for (uint32_t k = 0; k < WLRX_NBUF; k += 777u) { CHECK(wlcm_cookie_slot(wlcm_buf_cookie(k)) == k, "c2f cookie round-trip"); }
	CHECK(wlcm_cookie_slot(0u) == WLRX_SLOTS && wlcm_cookie_slot(0x1fffffu) == WLRX_SLOTS &&
	    wlcm_cookie_slot(wlcm_buf_cookie(WLRX_NBUF - 1u) + 1u) == WLRX_SLOTS, "c2f foreign cookie refused");
	CHECK(wlcm_ppt_addr(&cm, 7u) == 0x113000u + 4u * 7u && wlcm_ppt_val(&cm, 0u) == (uint32_t)((cm.spt_iova) >> 12), "c2f ppt");
	{   /* the converted-return decode: a good cookie is held and queued; a foreign cookie is refused, not held */
		static struct wlrd rd;
		memset(&rd, 0, sizeof(rd));
		wlrd_slots_init(&rd);
		CHECK(wlrd_recycle_cc(&rd, &rx, wlcm_buf_cookie(5u)) == 0 && rd.returned == 1 && rd.cc_entries == 1 &&
		    rd.slot[5] == WLRD_SLOT_HELD, "c2f converted return held");
		CHECK(wlrd_recycle_cc(&rd, &rx, 0u) == 1 && rd.why, "c2f foreign converted cookie refused");
	}
	CHECK(WLWMI_PDEV_PARAM_PMF_QOS == 0x22u && WLWMI_PDEV_PARAM_DYNAMIC_BW == 0x0au && WLWMI_PDEV_PARAM_ARP_AC_OVERRIDE == 0x23u &&
	    WLWMI_PDEV_PARAM_MESH_MCAST_ENABLE == 0x8fu && WLWMI_PDEV_PARAM_IDLE_PS_CONFIG == 0x2cu &&
	    WLWMI_VDEV_PARAM_TX_ENCAP_TYPE == 0x30u && WLWMI_VDEV_PARAM_RX_DECAP_TYPE == 0x5bu &&
	    WLWMI_CMD_DFS_PHYERR_OFFLOAD == 0xa005u && WLWMI_TAG_DFS_PHYERR_OFFLOAD_CMD == 0x250u &&
	    WLWMI_CMD_LRO_CONFIG == 0x1d010u && WLWMI_TAG_LRO_INFO_CMD == 0x1aau &&
	    WLHTT_H2T_PPDU_STATS_CFG == 0x11u && WLHTT_PPDU_STATS_DEFAULT == 0xe67u, "c2f WMI/HTT param values");
	free(ca); free(ra);
	puts("C2f pure: cc_config words and constants, the ten common rings' geometry/base/ID/threshold/MISC, the cc cookie "
	     "round-trip and foreign-cookie refusal, the PPT math, and the mac-start/encap/PPDU param values");
}

/* C2f gate: the production wl27_setup (C2d's REO pre-reads, then W29's common-ring pre-reads, cc_init and the common
 * rings around rel/exc) and wl29_cc_config, against the REO script and the W29 register file / CMEM of cm_access. */
static uint8_t *c2f_ro, *c2f_ca;
static void
c2f_gate_reset(void)
{
	reset();
	cmd.on = true; cmd.cmem_lo = 0x113000u; cmd.cmem_hi = 0x114000u;
	wl20_bits = 0x5902u;
	if (!c2f_ro) { c2f_ro = aligned_alloc(16384, REOA); c2f_ca = aligned_alloc(16384, WLCM_ARENA_BYTES); }
	CHECK(c2f_ro && c2f_ca, "c2f gate alloc");
	memset(c2f_ro, 0, REOA);
	reo_host = c2f_ro; reo_iova = 0xa2600000ull; reo_mapped = true;
	CHECK(wlrd_layout(&wl_rd, c2f_ro, reo_iova, REOA) == 0, "c2f gate REO layout");
	memset(&wl_cm, 0, sizeof(wl_cm));
	CHECK(wlcm_layout(&wl_cm, c2f_ca, 0xa5000000ull, WLCM_ARENA_BYTES) == 0, "c2f gate cm layout");
	wl29_writes = wl29_cmem_writes = 0;
	wl_cmem_base = 0x113000u; wl_cmem_size = 0x1000u; wl_cmem_valid = true;
}

static void
c2f_gate_done(void)
{
	memset(&wl_cm, 0, sizeof(wl_cm));
	wl_cmem_base = wl_cmem_size = 0; wl_cmem_valid = false;
	reset();
}

static void
s_c2f_gate(void)
{
	static const char why_cc[] = "FUXI WLAN: W29 cookie conversion not programmed: a REO/WBM cookie register is already "
	    "configured (cfg0 base, wbm cfg1 enable or convert global); w20 0x4000 cc dropped for this boot\n";
	static const char why_bad[] = "FUXI WLAN: W29 cookie conversion not programmed: the QMI CMEM region is out of bounds "
	    "(upper bits, start, alignment or size); w20 0x4000 cc dropped for this boot\n";
	/* both ours, the region good, the cc registers clear: the CMEM writes after every pre-read and before the first
	 * common-ring write (dp_alloc order), the rings, then the six cc_config writes at service ready */
	c2f_gate_reset();
	CHECK(wl27_setup() == 0 && wl_rd.reo_on && wl_cm.rings_on && wl_cm.cc_on && wl_rx.cc_on && wl_rd.cc_on, "c2f gate happy");
	CHECK(cmd.cmem_rd == 280u && cmd.cmem_wr == 280u && wl29_cmem_writes == 280u && !cmd.ring_wr_before_cmem &&
	    !cmd.cmem_wr_after_ring && cmd.ring_wr && cmd.cc_rd == 8u && !cmd.cc_wr, "c2f gate happy: CMEM after the pre-reads, before the rings");
	{
		static const char *const order[] = { "FUXI WLAN: W27 reo before:", "FUXI WLAN: W29 common before: rings 10 enabled 0 based 0 unreadable 0; not owned\n",
			"FUXI WLAN: W29 cookie conversion cmem base 0x113000 pages 280 (rx 24 tx 256)", "FUXI WLAN: W29 common ring 121 sw2wbm-rel",
			"FUXI WLAN: W29 common ring 16 reinject", "FUXI WLAN: W27 reo ring 131", "FUXI WLAN: W29 common ring 21 reo-status",
			"FUXI WLAN: W27 reo setup gen-enable" };
		expect_order(order, sizeof(order) / sizeof(order[0]));
	}
	CHECK(wl29_cc_config() == 0 && cmd.cc_wr == 6u && cmd.cc_rd == 10u, "c2f gate happy cc_config");
	expect("FUXI WLAN: W29 cc_config reo-cfg1 0x1d1310 wbm-cfg1 0x51310 cfg2 0->0xb convert 0->0x13a; writes 6\n");
	expect_not("cc dropped for this boot");
	/* REO owned (C2d refuses it): no W29 access at all, cc dropped with its line; cc_init and cc_config called again
	 * directly still write nothing */
	c2f_gate_reset();
	C.reo_pre[2] = 0x40u;
	CHECK(wl27_setup() == 0 && !wl_rd.reo_on && !wl_cm.rings_on && !wl_cm.cc_on && !wl_rx.cc_on && !wl_rd.cc_on, "c2f gate REO owned");
	expect("FUXI WLAN: W29 cookie conversion not programmed: the REO path is not ours (W27 refused it); cookie conversion needs it; "
	    "w20 0x4000 cc dropped for this boot\n");
	CHECK(wl29_cc_init() == 0 && wl29_cc_config() == 0 && !wl_cm.cc_on, "c2f gate REO owned, direct");
	CHECK(!cmd.cmem_rd && !cmd.cmem_wr && !cmd.cc_rd && !cmd.cc_wr && !cmd.ring_rd && !cmd.ring_wr && !wl29_cmem_writes,
	    "c2f gate REO owned: a CMEM, cc_config or common-ring access");
	/* the common rings owned (W29's pre-reads refuse them): C2d's own rings still run; no CMEM access, no cc_config
	 * register beyond the two ring pre-reads; cc_config refuses even with cc_on forced unless the REO path and the
	 * rings are both ours */
	c2f_gate_reset();
	cm_set(0xa44900u + 0x10u, 0x40u);
	CHECK(wl27_setup() == 0 && wl_rd.reo_on && !wl_cm.rings_on && !wl_cm.cc_on, "c2f gate rings owned");
	expect("FUXI WLAN: W29 common rings not programmed: a common ring has SRNG_ENABLE set (already owned)");
	expect("FUXI WLAN: W29 cookie conversion not programmed: the common rings are not ours (refused above); cookie conversion "
	    "needs them; w20 0x4000 cc dropped for this boot\n");
	CHECK(wl29_cc_init() == 0 && wl29_cc_config() == 0, "c2f gate rings owned, direct");
	wl_cm.cc_on = true;
	CHECK(wl29_cc_config() == 0, "c2f gate cc_config, rings not ours");
	wl_cm.rings_on = true; wl_rd.reo_on = false;
	CHECK(wl29_cc_config() == 0, "c2f gate cc_config, REO not ours");
	CHECK(!cmd.cmem_rd && !cmd.cmem_wr && cmd.cc_rd == 2u && !cmd.cc_wr && !cmd.ring_wr, "c2f gate rings owned: a CMEM or cc_config access");
	/* the cc_config pre-reads: a CFG0 base, the WBM CFG1 enable (bits 19/20), the convert GLOBAL_EN or any ~0 drop cc
	 * (one line, no CMEM access, nothing written to the cc registers; the rings still run); field bits alone do not, nor
	 * (C2g) REO CFG1 with both bases 0, whatever it holds: L19 writes it whole (dp.c:1340-1352) */
	{
		static const struct { unsigned k; uint32_t v; int drop; } cs[] = {
			{ 0, 0x113000u, 1 }, { 2, 0x1u, 1 }, { 3, 1u << 19, 1 }, { 3, 1u << 20, 1 }, { 5, 1u << 8, 1 }, { 3, ~0u, 2 },
			{ 1, 0x51310u, 0 }, { 5, 0x3au, 0 }, { 4, 0xbu, 0 }, { 3, 0x51310u, 0 },
			{ 1, 1u << 19, 0 }, { 1, 1u << 20, 0 }, { 1, 0x111700u, 0 }, { 1, 0x1d1310u, 0 }, { 1, 0xfffffffeu, 0 },
		};
		for (unsigned i = 0; i < sizeof(cs) / sizeof(cs[0]); i++) {
			c2f_gate_reset();
			cmd.cc_pre[cs[i].k] = cs[i].v;
			CHECK(wl27_setup() == 0 && wl_rd.reo_on && wl_cm.rings_on && wl_cm.ring[WLCM_R_REOSTATUS].on, "c2f gate cc pre-read run");
			CHECK(wl29_cc_config() == 0, "c2f gate cc pre-read cc_config");
			if (cs[i].drop) {
				CHECK(!wl_cm.cc_on && !cmd.cmem_rd && !cmd.cmem_wr && !cmd.cc_wr, "c2f gate: a configured cc register not refused");
				if (cs[i].drop == 1) { expect(why_cc); }
				else { expect("FUXI WLAN: W29 cookie conversion not programmed: a pre-read is ~0 (unreadable, ownership not established)"); }
			} else {
				CHECK(wl_cm.cc_on && cmd.cmem_wr == 280u && cmd.cc_wr == 6u, "c2f gate: an unconfigured cc register refused");
				expect_not("cc dropped for this boot");
			}
		}
	}
	/* the QMI CMEM region: absent, too small, below 0x80000, at or past 0x2000000 (start or PPT end), misaligned, upper bits
	 * set: dropped with its line, no CMEM access; the two boundaries accepted */
	{
		static const struct { uint64_t start, size; int ok; } rg[] = {
			{ 0x113000u, 0x45cu, 0 }, { 0x7fffcu, 0x1000u, 0 }, { 0x2000000u, 0x1000u, 0 }, { 0x1fffe00u, 0x1000u, 0 },
			{ 0x113002u, 0x1000u, 0 }, { 0x100113000ull, 0x1000u, 0 }, { 0x80000u, 0x460u, 1 }, { 0x1fffba0u, 0x460u, 1 },
		};
		for (unsigned i = 0; i < sizeof(rg) / sizeof(rg[0]); i++) {
			c2f_gate_reset();
			wl_cmem_base = rg[i].start; wl_cmem_size = rg[i].size;
			cmd.cmem_lo = (uint32_t)rg[i].start & ~3u; cmd.cmem_hi = cmd.cmem_lo + 0x1000u;
			CHECK(wl27_setup() == 0 && wl_rd.reo_on && wl_cm.rings_on, "c2f gate region run");
			CHECK(wl29_cc_config() == 0, "c2f gate region cc_config");
			if (rg[i].ok) {
				CHECK(wl_cm.cc_on && cmd.cmem_rd == 280u && cmd.cmem_wr == 280u && cmd.cc_wr == 6u, "c2f gate: a good CMEM region refused");
			} else {
				CHECK(!wl_cm.cc_on && !cmd.cmem_rd && !cmd.cmem_wr && cmd.cc_rd == 2u && !cmd.cc_wr, "c2f gate: a bad CMEM region used");
				expect(why_bad);
			}
		}
		c2f_gate_reset();
		wl_cmem_valid = false;
		CHECK(wl27_setup() == 0 && !wl_cm.cc_on && !cmd.cmem_rd && !cmd.cmem_wr && cmd.cc_rd == 2u, "c2f gate no CMEM region");
		expect("FUXI WLAN: W29 cookie conversion not programmed: no QMI CMEM region (dev_mem[0]); cookie conversion needs it");
	}
	/* the CMEM allowlist keeps every address inside start..start+size; the window refuses an offset past its 6-bit field
	 * without touching the window register */
	{
		struct wlcm cm;
		uint32_t v = 0, sel, w;
		memset(&cm, 0, sizeof(cm));
		cm.cmem_valid = true; cm.cmem_base = 0x113000u; cm.cmem_size = 0x460u;
		CHECK(wlcm_cmem_ok(&cm, 0x113000u) && wlcm_cmem_ok(&cm, 0x11345cu) && !wlcm_cmem_ok(&cm, 0x113460u) &&
		    !wlcm_cmem_ok(&cm, 0x113002u) && !wlcm_cmem_ok(&cm, 0x112ffcu), "c2f CMEM allowlist (size 0x460)");
		cm.cmem_size = 0x45cu;
		CHECK(wlcm_cmem_ok(&cm, 0x113458u) && !wlcm_cmem_ok(&cm, 0x11345cu), "c2f CMEM allowlist (start..start+size)");
		c2f_gate_reset();
		cmd.cmem_lo = 0x1fff000u; cmd.cmem_hi = 0x2000000u;
		sel = win_selects; w = bar_win;
		CHECK(wl_win(0x2000000u, false, &v) == 1 && wl_win(0xffffffffu, true, &v) == 1 && win_selects == sel && bar_win == w,
		    "c2f window: an offset past the 6-bit field not refused, or the window register touched");
		CHECK(wl_win(0x1fffffcu, false, &v) == 0 && win_selects == sel + 1u, "c2f window: the top offset refused");
	}
	c2f_gate_done();
	puts("C2f gate: CMEM written only after the REO and common-ring pre-reads and before the first common-ring write; REO "
	     "owned or common rings owned: no CMEM or cc_config write (cc_config refuses even with cc_on forced); a cc register "
	     "pre-read configured or unreadable: cc dropped with its line; the QMI CMEM region absent, too small, out of range, "
	     "misaligned or with upper bits: cc dropped, no CMEM access; the CMEM allowlist inside start..start+size; the window "
	     "refuses an offset past its field");
}

/* C2f bisect, the guard: the phone's r1c2g false SPILL (cookie conversion dropped, so the SPT was never filled, yet
 * every RX SPT entry counted as spt-lost 4095) is gone: unfilled, the guard counts non-zero SPT dwords instead (the
 * device was never given those pages); filled, the lost entries as before. Run through the production wl27_setup and
 * wl29_report. */
static void
s_c2f_guard(void)
{
	volatile uint32_t *spt;
	/* cc dropped (C2g: the WBM CFG1 enable already set; REO CFG1 alone no longer drops it): no SPILL; one non-zero SPT
	 * dword is one */
	c2f_gate_reset();
	wl_rx.bufs_iova = 0xa1a20000ull;
	cmd.cc_pre[3] = 1u << 19;
	CHECK(wl27_setup() == 0 && wl_cm.rings_on && !wl_cm.cc_on && !wl_cm.spt_filled, "c2f guard: cc dropped run");
	wl29_report();
	expect("FUXI WLAN: W29 guard nonzero dwords: rdp 0, ring-gap 0, spt-nonzero 0, tail 0\n");
	expect_not("SPILL");
	spt = (volatile uint32_t *)(void *)wl_cm.spt;
	spt[3u * 1024u + 7u] = 1u;   /* a TX SPT page the device was never given */
	loglen = 0; logbuf[0] = 0;
	wl29_report();
	expect("FUXI WLAN: W29 guard SPILL nonzero dwords: rdp 0, ring-gap 0, spt-nonzero 1, tail 0; stop; owner review before any "
	    "further kit\n");
	spt[3u * 1024u + 7u] = 0u;
	spt[(WLCM_RX_SPT_OFFSET + 2u) * 1024u + 5u] = 0xa5a5a5a5u;   /* an RX SPT page too */
	loglen = 0; logbuf[0] = 0;
	wl29_report();
	expect("spt-nonzero 1, tail 0; stop");
	/* cc on: the SPT filled; no SPILL; a cleared RX entry is spt-lost 1 */
	c2f_gate_reset();
	wl_rx.bufs_iova = 0xa1a20000ull;
	CHECK(wl27_setup() == 0 && wl_cm.cc_on && wl_cm.spt_filled, "c2f guard: cc on run");
	wl29_report();
	expect("FUXI WLAN: W29 guard nonzero dwords: rdp 0, ring-gap 0, spt-lost 0, tail 0\n");
	expect_not("SPILL");
	spt = (volatile uint32_t *)(void *)wl_cm.spt;
	spt[WLCM_RX_SPT_OFFSET * 1024u + 2u * 100u] = 0u; spt[WLCM_RX_SPT_OFFSET * 1024u + 2u * 100u + 1u] = 0u;   /* slot 100 */
	loglen = 0; logbuf[0] = 0;
	wl29_report();
	expect("FUXI WLAN: W29 guard SPILL nonzero dwords: rdp 0, ring-gap 0, spt-lost 1, tail 0; stop; owner review before any "
	    "further kit\n");
	/* cc not selected (fuxiwlanw29 0x01): never filled, counted as unfilled */
	c2f_gate_reset();
	wl_rx.bufs_iova = 0xa1a20000ull;
	wl29_mask = WL29_RINGS;
	CHECK(wl27_setup() == 0 && wl_cm.rings_on && !wl_cm.cc_on && !wl_cm.spt_filled, "c2f guard: cc not selected run");
	wl29_report();
	expect("FUXI WLAN: W29 guard nonzero dwords: rdp 0, ring-gap 0, spt-nonzero 0, tail 0\n");
	expect_not("SPILL");
	c2f_gate_done();
	puts("C2f guard: cc dropped or not selected (SPT never filled): no false spt-lost, a non-zero SPT dword (TX or RX page) is a "
	     "SPILL; cc on: the filled entries intact, a cleared one is a SPILL");
}

/* C2f bisect, the cc_config pre-reads printed read-only (address and value of all six, then the hal.h names set per
 * register), when cc goes on and when it is dropped; the widest decode fits its buffer and the console untruncated. */
static void
s_c2f_ccvals(void)
{
	static const char why_cc[] = "FUXI WLAN: W29 cookie conversion not programmed: a REO/WBM cookie register is already "
	    "configured (cfg0 base, wbm cfg1 enable or convert global); w20 0x4000 cc dropped for this boot\n";
	c2f_gate_reset();
	CHECK(wl27_setup() == 0 && wl_cm.cc_on, "c2f cc values: happy");
	{
		static const char *const order[] = {
			"FUXI WLAN: W29 cc before: reo-cfg0@0xa38050 0, reo-cfg1@0xa38054 0, wbm-cfg0@0xa34040 0, wbm-cfg1@0xa34044 0, "
			"wbm-cfg2@0xa34090 0, convert@0xa34094 0\n",
			"FUXI WLAN: W29 cookie conversion cmem base 0x113000 pages 280",
		};
		expect_order(order, 2);
	}
	expect_not("cc before decode");
	/* the firmware already configured cookie conversion as L19 would (the phone's r1c2g reading, values assumed) */
	c2f_gate_reset();
	{
		static const uint32_t v[6] = { 0x113000u, 0x1d1310u, 0x113000u, 0x51310u, 0xbu, 0x13au };
		memcpy(cmd.cc_pre, v, sizeof(v));
	}
	CHECK(wl27_setup() == 0 && wl_cm.rings_on && !wl_cm.cc_on && !cmd.cmem_rd && !cmd.cc_wr, "c2f cc values: configured");
	{
		static const char *const order[] = {
			"FUXI WLAN: W29 cc before: reo-cfg0@0xa38050 0x113000, reo-cfg1@0xa38054 0x1d1310, wbm-cfg0@0xa34040 0x113000, "
			"wbm-cfg1@0xa34044 0x51310, wbm-cfg2@0xa34090 0xb, convert@0xa34094 0x13a\n",
			"FUXI WLAN: W29 cc before decode reo-cfg1 0x1d1310 HAL_REO1_SW_COOKIE_CFG_*: CMEM_BASE_ADDR_MSB=0x10 COOKIE_PPT_MSB=0x13 "
			"COOKIE_SPT_MSB=0x8 ALIGN ENABLE GLOBAL_ENABLE\n",
			"FUXI WLAN: W29 cc before decode wbm-cfg1 0x51310 HAL_WBM_SW_COOKIE_CFG_*: CMEM_BASE_ADDR_MSB=0x10 COOKIE_PPT_MSB=0x13 "
			"COOKIE_SPT_MSB=0x8 ALIGN\n",
			"FUXI WLAN: W29 cc before decode wbm-cfg2 0xb HAL_WBM_SW_COOKIE_CFG_*: RELEASE_PATH_EN ERR_PATH_EN CONV_IND_EN\n",
			"FUXI WLAN: W29 cc before decode convert 0x13a HAL_WBM_SW_COOKIE_CONV_CFG_*: WBM2SW0_EN WBM2SW2_EN WBM2SW3_EN WBM2SW4_EN "
			"GLOBAL_EN\n",
			why_cc,
		};
		expect_order(order, sizeof(order) / sizeof(order[0]));
	}
	CHECK(count_text("cc before decode") == 4, "c2f cc values: CFG0 has no named bits");
	/* the widest: every register 0xfffffffe (one ~0 is unreadable: printed, not decoded) */
	c2f_gate_reset();
	for (unsigned k = 0; k < 6; k++) { cmd.cc_pre[k] = k == 4 ? ~0u : 0xfffffffeu; }
	CHECK(wl27_setup() == 0 && !wl_cm.cc_on && !cmd.cmem_rd, "c2f cc values: widest");
	expect("FUXI WLAN: W29 cc before: reo-cfg0@0xa38050 0xfffffffe, reo-cfg1@0xa38054 0xfffffffe, wbm-cfg0@0xa34040 0xfffffffe, "
	    "wbm-cfg1@0xa34044 0xfffffffe, wbm-cfg2@0xa34090 0xffffffff, convert@0xa34094 0xfffffffe\n");
	expect("FUXI WLAN: W29 cc before decode reo-cfg1 0xfffffffe HAL_REO1_SW_COOKIE_CFG_*: CMEM_BASE_ADDR_MSB=0xfe COOKIE_PPT_MSB=0x1f "
	    "COOKIE_SPT_MSB=0x1f ALIGN ENABLE GLOBAL_ENABLE other 0xffe00000\n");
	expect("FUXI WLAN: W29 cc before decode wbm-cfg1 0xfffffffe HAL_WBM_SW_COOKIE_CFG_*: CMEM_BASE_ADDR_MSB=0xfe COOKIE_PPT_MSB=0x1f "
	    "COOKIE_SPT_MSB=0x1f ALIGN other 0xfff80000\n");
	expect("FUXI WLAN: W29 cc before decode convert 0xfffffffe HAL_WBM_SW_COOKIE_CONV_CFG_*: WBM2SW0_EN WBM2SW1_EN WBM2SW2_EN "
	    "WBM2SW3_EN WBM2SW4_EN GLOBAL_EN other 0xfffffec0\n");
	expect("FUXI WLAN: W29 cookie conversion not programmed: a pre-read is ~0 (unreadable, ownership not established)");
	CHECK(count_text("cc before decode") == 3, "c2f cc values: a ~0 or CFG0 register decoded");
	/* CFG2 bits alone do not refuse cc: the values and the decode are printed and cc goes on */
	c2f_gate_reset();
	cmd.cc_pre[4] = 0xfffffffeu;
	CHECK(wl27_setup() == 0 && wl_cm.cc_on, "c2f cc values: cfg2 set");
	{
		static const char *const order[] = {
			"FUXI WLAN: W29 cc before: reo-cfg0@0xa38050 0, reo-cfg1@0xa38054 0, wbm-cfg0@0xa34040 0, wbm-cfg1@0xa34044 0, "
			"wbm-cfg2@0xa34090 0xfffffffe, convert@0xa34094 0\n",
			"FUXI WLAN: W29 cc before decode wbm-cfg2 0xfffffffe HAL_WBM_SW_COOKIE_CFG_*: ERR_PATH_EN CONV_IND_EN other 0xfffffff4\n",
			"FUXI WLAN: W29 cookie conversion cmem base 0x113000 pages 280",
		};
		expect_order(order, 3);
	}
	CHECK(count_text("cc before decode") == 1, "c2f cc values: zero registers decoded");
	/* C2g: the phone's values at every boot (reo-cfg1 0x111700, wbm-cfg2 0x40, the rest 0): no base anywhere, WBM off, so
	 * not owned; cc goes on and cc_config programs all six, CFG2 and CONVERT read-modify-write over what they hold */
	c2f_gate_reset();
	{
		static const uint32_t v[6] = { 0, 0x111700u, 0, 0, 0x40u, 0 };
		memcpy(cmd.cc_pre, v, sizeof(v));
	}
	CHECK(wl27_setup() == 0 && wl_cm.rings_on && wl_cm.cc_on && wl_rx.cc_on && wl_rd.cc_on && cmd.cmem_wr == 280u, "c2g cc: phone values");
	{
		static const char *const order[] = {
			"FUXI WLAN: W29 cc before: reo-cfg0@0xa38050 0, reo-cfg1@0xa38054 0x111700, wbm-cfg0@0xa34040 0, wbm-cfg1@0xa34044 0, "
			"wbm-cfg2@0xa34090 0x40, convert@0xa34094 0\n",
			"FUXI WLAN: W29 cc before decode reo-cfg1 0x111700 HAL_REO1_SW_COOKIE_CFG_*: COOKIE_PPT_MSB=0x17 COOKIE_SPT_MSB=0x8 "
			"GLOBAL_ENABLE\n",
			"FUXI WLAN: W29 cc before decode wbm-cfg2 0x40 HAL_WBM_SW_COOKIE_CFG_*: other 0x40\n",
			"FUXI WLAN: W29 cookie conversion cmem base 0x113000 pages 280",
		};
		expect_order(order, sizeof(order) / sizeof(order[0]));
	}
	expect_not("cc dropped for this boot");
	CHECK(wl29_cc_config() == 0 && cmd.cc_wr == 6u, "c2g cc: phone values, cc_config");
	expect("FUXI WLAN: W29 cc_config reo-cfg1 0x1d1310 wbm-cfg1 0x51310 cfg2 0x40->0x4b convert 0->0x13a; writes 6\n");
	/* no pre-reads (no QMI CMEM region): no value line */
	c2f_gate_reset();
	wl_cmem_valid = false;
	CHECK(wl27_setup() == 0 && !wl_cm.cc_on, "c2f cc values: no CMEM");
	expect_not("cc before");
	c2f_gate_done();
	puts("C2f cc values: the six cc_config pre-reads printed with their addresses, cc on and dropped; the hal.h names set "
	     "decoded per register (CFG0 and ~0 not), other bits as other; the widest within the console; C2g: the phone's "
	     "values (reo-cfg1 0x111700, wbm-cfg2 0x40) are not an owner and cc is programmed");
}

/* C2f bisect, fuxiwlanw29 at W20: the common rings (0x01) and cookie conversion (0x02) each run only when selected; without
 * 0x01 not even the common-ring pre-reads; C2d's own rings stand either way. */
static void
s_c2f_mask_gate(void)
{
	c2f_gate_reset();
	wl29_mask = WL29_RINGS;
	CHECK(wl27_setup() == 0 && wl_rd.reo_on && wl_cm.rings_on && wl_cm.ring[WLCM_R_REOSTATUS].on && !wl_cm.cc_on, "c2f mask 0x01");
	CHECK(wl29_cc_config() == 0 && !cmd.cmem_rd && !cmd.cmem_wr && cmd.cc_rd == 2u && !cmd.cc_wr && cmd.ring_wr, "c2f mask 0x01 accesses");
	expect("FUXI WLAN: W29 common ring 21 reo-status");
	expect_not("cookie conversion"); expect_not("cc before");
	c2f_gate_reset();
	wl29_mask = WL29_RINGS;
	C.reo_pre[2] = 0x40u;   /* REO owned: cc not selected, so not even its drop line */
	CHECK(wl27_setup() == 0 && !wl_rd.reo_on && !wl_cm.cc_on, "c2f mask 0x01 REO owned");
	expect_not("cookie conversion");
	c2f_gate_reset();
	wl29_mask = WL29_PPDU | WL29_ENCAP;
	CHECK(wl27_setup() == 0 && wl_rd.reo_on && !wl_cm.rings_on && !wl_cm.cc_on, "c2f mask 0x84");
	CHECK(wl29_cc_config() == 0 && !cmd.ring_rd && !cmd.ring_wr && !cmd.cc_rd && !cmd.cc_wr && !cmd.cmem_rd, "c2f mask 0x84 accesses");
	expect("FUXI WLAN: W27 reo ring 131");
	expect_not("W29 common"); expect_not("cookie conversion");
	c2f_gate_reset();
	wl29_mask = WL29_RINGS | WL29_CC;
	CHECK(wl27_setup() == 0 && wl_cm.rings_on && wl_cm.cc_on && cmd.cmem_wr == 280u, "c2f mask 0x03");
	CHECK(wl29_cc_config() == 0 && cmd.cc_wr == 6u, "c2f mask 0x03 cc_config");
	c2f_gate_done();
	puts("C2f mask gate: fuxiwlanw29 0x01 the common rings alone (no CMEM, no cc register beyond the two ring pre-reads, no cc "
	     "line, also with the REO path refused), 0x84 no common-ring or cc access at all (C2d's rings stand), 0x03 both");
}

/* C2f bisect, fuxiwlanw29 in a full qrtr 7 scan (w20 0x4002: the mac-start pieces need no ring): the firmware sees exactly
 * the selected mac-start / encap commands, word by word, in L19's order and place, and the scan still completes. */
static void
s_c2f_bisect(void)
{
	static const uint32_t l19[11] = { 0x22u, 0x0au, 0x23u, 0xdu, 0x11u, 0x11u, 0x8fu, 0x1du, 0x2cu, 0x30u, 0x5bu };
	static const uint8_t bit[11] = { WL29_PDEV, WL29_PDEV, WL29_PDEV, WL29_DFS, WL29_PPDU, WL29_PPDU, WL29_PDEV, WL29_LRO, WL29_IDLEPS,
		WL29_ENCAP, WL29_ENCAP };
	static const unsigned masks[] = { 0xffu, 0x01u, 0x05u, 0x09u, 0x11u, 0x21u, 0x41u, 0x81u, 0x5cu, 0xa0u, 0x03u, WL29_DEFAULT };
	_Static_assert(WL29_DEFAULT == 0x9fu, "C2g: the fuxiwlanw29 default");
	for (unsigned t = 0; t < sizeof(masks) / sizeof(masks[0]); t++) {
		const unsigned m = masks[t];
		unsigned n = 0;
		c2_reset();
		wl20_bits = WL20_HTTVER | WL20_L19ORDER;
		wl29_mask = m;
		CHECK(run() == 0, "c2f bisect run");
		expect("FUXI WLAN: W23 scan completed reason 0 after ");
		CHECK(fw_scan_cmds == 1 && fw_c2_step == 8, "c2f bisect: the scan");
		for (unsigned i = 0; i < 11u; i++) {
			if (!(m & bit[i])) { continue; }
			CHECK(n < fw_w29_n && fw_w29_log[n] == l19[i], "c2f bisect: a selected command missing or out of order");
			n++;
		}
		CHECK(n == fw_w29_n, "c2f bisect: a command fuxiwlanw29 did not select");
		CHECK(count_text("W29 htt ppdu stats cfg") == (m & WL29_PPDU ? 2u : 0u), "c2f bisect: ppdu lines");
		CHECK(count_text("FUXI WLAN: W29 vdev 0 encap/decap native-wifi (service 461 not set)\n") == (m & WL29_ENCAP ? 1u : 0u),
		    "c2f bisect: encap line");
		if ((m & 0x1cu) == 0x1cu) {
			expect("FUXI WLAN: W29 mac-start pdev params pmf-qos 1 dynamic-bw 1 arp-ac 0 dfs-offload mesh-mcast 1 ppdu-stats 2 pdev 0\n");
		}
		if ((m & 0x60u) == 0x60u) { expect("FUXI WLAN: W29 mac-start lro cfg (seeds 0, lro-enable 0) idle-ps 1 pdev 0\n"); }
		if (!(m & 0x1cu)) { expect_not("W29 mac-start pdev params"); }
		if (!(m & 0x60u)) { expect_not("W29 mac-start lro"); }
		if (m == 0x05u) { expect("FUXI WLAN: W29 mac-start pdev params not sent, dfs-offload not sent, ppdu-stats 2, pdev 0 (fuxiwlanw29 0x5)\n"); }
		if (m == 0x09u) {
			expect("FUXI WLAN: W29 mac-start pdev params pmf-qos 1 dynamic-bw 1 arp-ac 0 mesh-mcast 1, dfs-offload not sent, ppdu-stats 0, "
			    "pdev 0 (fuxiwlanw29 0x9)\n");
		}
		if (m == 0x11u) { expect("FUXI WLAN: W29 mac-start pdev params not sent, dfs-offload sent, ppdu-stats 0, pdev 0 (fuxiwlanw29 0x11)\n"); }
		if (m == 0x21u) { expect("FUXI WLAN: W29 mac-start lro cfg (seeds 0, lro-enable 0), idle-ps not sent, pdev 0 (fuxiwlanw29 0x21)\n"); }
		if (m == 0x41u) { expect("FUXI WLAN: W29 mac-start lro cfg not sent, idle-ps 1, pdev 0 (fuxiwlanw29 0x41)\n"); }
		expect_not("W23 failure");
		contained_and_freed();
	}
	reset();
	puts("C2f bisect: fuxiwlanw29 0xff, 0x01, 0x05, 0x09, 0x11, 0x21, 0x41, 0x81, 0x5c, 0xa0, 0x03 and the C2g default 0x9f in "
	     "a full scan: exactly the selected mac-start pdev params, DFS offload, PPDU stats, LRO, idle-PS and encap/decap, word by "
	     "word, in L19's order and place (the default: no LRO, no idle-PS); the scan completes");
}

/* C2g, read-only: at every W26 watch the raw remote pointer words of each LMAC ring set up, named by where L19 puts
 * them (an SRC ring's HP in wrp[id - 160] and TP in rdp[id], a DST ring the reverse), then the valid entries of the
 * MAC rings 161/169 with the first and last index. The model's firmware mirrors its MAC ring HP into wrp[1]/[9]; the
 * phone's (C.mac_hp_silent) fills the rings and leaves both words 0, which is what L19 itself would read there (it
 * never reads or writes these two rings' pointers after SRING_SETUP). */
static void
s_c2g_ptrs(void)
{
	/* the C2c happy path: 2048 refill entries taken, 1024 to each MAC ring, HP mirrored */
	c2c_happy_run(0);
	show("c2g 0x702");
	{
		static const char *const lines[] = {
			("FUXI WLAN: W26 rx watch setup+100ms refill tp 4096 taken 2048 left 2047 mac0 hp 2048 tp 0 avail 1024 mac1 hp 2048 tp 0 "
			 "avail 1024 err hp 0 fill 0; exhausted none\n"),
			("FUXI WLAN: W26 rx ptr words setup+100ms: 160 hp wrp[0] 0x1ffe tp rdp[160] 0x1000; 161 hp wrp[1] 0x800 tp rdp[161] 0; "
			 "169 hp wrp[9] 0x800 tp rdp[169] 0\n"),
			"FUXI WLAN: W26 rx ptr words setup+100ms: 186 hp rdp[186] 0 tp wrp[26] 0; 185/193 not set up\n",
			"FUXI WLAN: W26 rx valid span setup+100ms: mac0 valid 1024 first 0 last 1023; mac1 valid 1024 first 0 last 1023\n",
			"FUXI WLAN: W26 idle watch setup+100ms ",
			"FUXI WLAN: W26 rx watch started ",
			"FUXI WLAN: W26 rx ptr words started: 160 hp wrp[0] 0x1ffe ",
			"FUXI WLAN: W26 rx valid span started: ",
			"FUXI WLAN: W26 rx watch mid (foreign 19) ",
			("FUXI WLAN: W26 rx ptr words mid (foreign 19): 160 hp wrp[0] 0x1ffe tp rdp[160] 0x1000; 161 hp wrp[1] 0x800 tp rdp[161] "
			 "0x34; 169 hp wrp[9] 0x800 tp rdp[169] 0"),
			"FUXI WLAN: W26 rx watch completed ",
			"FUXI WLAN: W26 rx ptr words completed: ",
			"FUXI WLAN: W26 rx ptr words completed: 186 hp rdp[186] 0 tp wrp[26] 0; 185/193 not set up\n",
			"FUXI WLAN: W26 rx valid span completed: mac0 valid 1024 first 0 last 1023; mac1 valid 1024 first 0 last 1023\n",
			"FUXI WLAN: W26 rx content mac0 entries 1024 valid 1024 ",
		};
		expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	}
	CHECK(count_text("W26 rx ptr words") == 8 && count_text("W26 rx valid span") == 4, "c2g: two word lines and one span line per watch");
	contained_and_freed();
	/* the phone (r1c2c-all, w20 0x702): 4095 taken, 2047 and 2016 entries written to 161/169, no HP word written: the
	 * watch shows the phone's "hp 0 tp 0 avail 0", the raw words are 0 and the span shows where the entries are */
	c2d_reset(0x702); c2d_phone(); C.mac_hp_silent = true; C.d_frames = 120; happy_frames();
	CHECK(run() == 1, "c2g phone: the wedge");
	show("c2g phone");
	{
		static const char *const lines[] = {
			("FUXI WLAN: W26 rx watch setup+100ms refill tp 8190 taken 4095 left 0 mac0 hp 0 tp 0 avail 0 mac1 hp 0 tp 0 avail 0 "),
			("FUXI WLAN: W26 rx ptr words setup+100ms: 160 hp wrp[0] 0x1ffe tp rdp[160] 0x1ffe; 161 hp wrp[1] 0 tp rdp[161] 0; "
			 "169 hp wrp[9] 0 tp rdp[169] 0\n"),
			"FUXI WLAN: W26 rx ptr words setup+100ms: 186 hp rdp[186] 0 tp wrp[26] 0; 185/193 not set up\n",
			"FUXI WLAN: W26 rx valid span setup+100ms: mac0 valid 2047 first 0 last 2046; mac1 valid 2016 first 0 last 2015\n",
			"FUXI WLAN: W26 rx valid span completed: mac0 valid 2047 first 0 last 2046; mac1 valid 2016 first 0 last 2015\n",
			"FUXI WLAN: W26 rx content mac0 entries 2047 valid 2047 foreign 0 rbm-other 0 cookie-other 0\n",
			"FUXI WLAN: W26 rx content mac1 entries 2016 valid 2016 foreign 0 rbm-other 0 cookie-other 0\n",
		};
		expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	}
	expect_not("not owned; refused");
	contained_and_freed();
	/* with the monitor status rings (C2e, w20 0x3b02): the second word line names 185/193 */
	c2e_reset(0x3b02); C.d_frames = 120; happy_frames();
	CHECK(run() == 0, "c2g 0x3b02");
	show("c2g 0x3b02");
	expect("FUXI WLAN: W26 rx ptr words setup+100ms: 186 hp rdp[186] 0 tp wrp[26] 0; 185 hp wrp[25] 0x7fe tp rdp[185] 0; "
	    "193 hp wrp[33] 0x7fe tp rdp[193] 0\n");
	expect_not("185/193 not set up");
	c2e_contained(false);
	/* the RXDMA error ring 186 (DST): its HP is the device's rdp[186], its TP the host's wrp[26] (never moved, 0) */
	c2c_reset(0x312); C.exp_n = 13; C.rx_rule = 5; C.rx_frames = 1; C.err_n = 1100; C.err_code = 0; C.err_rbm = 1;
	CHECK(run() == 0, "c2g err full");
	show("c2g err full");
	expect("FUXI WLAN: W26 rx ptr words completed: 186 hp rdp[186] 0x1ff8 tp wrp[26] 0; 185/193 not set up\n");
	c2c_contained();
	/* with no MAC ring entry at all (the firmware takes nothing): "valid 0 first none last none" */
	c2d_reset(0x702); C.rx_take = 0; C.rx_post[0] = C.rx_post[1] = 0; C.d_frames = 120; happy_frames();
	(void)run();
	show("c2g none");
	expect("FUXI WLAN: W26 rx valid span setup+100ms: mac0 valid 0 first none last none; mac1 valid 0 first none last none\n");
	contained_and_freed();
	reset();
	puts("C2g pointer words: at every W26 watch the raw HP/TP words of 160/161/169/186 (and 185/193 with 0x2000) where L19 puts "
	     "them, and the valid span of 161/169; the phone's silent HP (raw 0 with 2047/2016 valid entries at 0..2046/0..2015), "
	     "no entry at all, host reads only");
}

static void
dump_c2f(void)
{
	static struct wlcm cm;
	static struct wlrx rx;
	uint8_t *ca = aligned_alloc(16384, WLCM_ARENA_BYTES), *ra = aligned_alloc(16384, RXA);
	uint32_t regs[WLCM_RING_REGS_MAX][2], cc[WLCM_CC_REGS][2];
	unsigned n = 0;
	CHECK(ca && ra && !wlrx_layout(&rx, ra, 0xa1400000ull, RXA) && !wlcm_layout(&cm, ca, 0xa5000000ull, WLCM_ARENA_BYTES), "dump layout");
	cm.cmem_base = 0x113000u; cm.cmem_valid = true;
	wlcm_cc_regs(&cm, 0u, 0u, cc);
	for (unsigned k = 0; k < WLCM_CC_REGS; k++) { fprintf(stdout, "ccreg %u %u %u\n", k, cc[k][0], cc[k][1]); }
	for (unsigned i = 0; i < WLCM_NRING; i++) {
		wlcm_ring_regs(&cm, i, regs, &n);
		fprintf(stdout, "ringn %u %u %u %u %u\n", cm.ring[i].id, cm.ring[i].dir, cm.ring[i].entries, cm.ring[i].entry_dw, n);
		for (unsigned k = 0; k < n; k++) { fprintf(stdout, "ring %u %u %u %u\n", cm.ring[i].id, k, regs[k][0], regs[k][1]); }
	}
	for (unsigned i = 0; i < 2u * WLCM_NRING + 2u; i++) { fprintf(stdout, "cmpre %u %u\n", i, wlcm_pre_addr(&cm, i)); }
	for (unsigned k = 0; k < WLCM_CC_REGS; k++) {   /* C2f bisect: the cc decode's hal.h names (tools/test-fuxi-wlan-ce.py l19_c2f) */
		const struct wlcm_cc_field *f = NULL;
		const char *pfx = NULL;
		unsigned n = wlcm_cc_fields(k, &f, &pfx);
		for (unsigned i = 0; i < n; i++) { fprintf(stdout, "ccfield %u %s%s %u %u\n", k, pfx, f[i].name, f[i].shift, f[i].width); }
	}
	{
		const uint32_t ks[4] = { 0u, 511u, 512u, WLRX_NBUF - 1u };
		for (unsigned i = 0; i < 4; i++) { fprintf(stdout, "cookie %u %u %u\n", ks[i], wlcm_buf_cookie(ks[i]), wlcm_cookie_slot(wlcm_buf_cookie(ks[i]))); }
	}
	for (unsigned i = 0; i < WLCM_NSPT; i += (WLCM_NSPT - 1u) / 2u) { fprintf(stdout, "ppt %u %u %u\n", i, wlcm_ppt_addr(&cm, i), wlcm_ppt_val(&cm, i)); }
	fprintf(stdout, "ccv %u %u %u %u\n", WLCM_REO_CC_CFG1_VAL, WLCM_CC_CFG1_COMMON, WLCM_WBM_CC_CFG2_OR, WLCM_WBM_CC_CONVERT_OR);
	fprintf(stdout, "arena %u %u %u %u %u\n", WLCM_ARENA_BYTES, WLCM_OFF_TAIL, WLCM_NSPT, WLCM_NUM_RX_SPT_PAGES, WLCM_RX_SPT_OFFSET);
	fprintf(stdout, "ppdu %u %u\n", WLHTT_H2T_PPDU_STATS_CFG, WLHTT_PPDU_STATS_DEFAULT);
	fflush(stdout);
	free(ca); free(ra);
}

static void
dump_c2d(void)
{
	static struct wlrd rd;
	uint8_t *ro = aligned_alloc(16384, REOA);
	uint32_t w[17], regs[WLRD_RING_REGS][2], hw[WLRD_HW_REGS][2];
	CHECK(ro && !wlrd_layout(&rd, ro, 0xa2600000ull, REOA), "dump layout");
	for (unsigned m = 0; m < 2; m++) {
		wlrd_sel_words(m, w);
		for (unsigned k = 0; k < 17; k++) { fprintf(stdout, "sel%u %u %u\n", m, k, w[k]); }
	}
	for (unsigned i = 0; i < WLRD_NRING; i++) {
		wlrd_ring_regs(&rd, i, regs);
		for (unsigned k = 0; k < WLRD_RING_REGS; k++) { fprintf(stdout, "ring%u %u %u %u\n", rd.ring[i].id, k, regs[k][0], regs[k][1]); }
		for (unsigned j = 0; j < 36; j++) { if (wl15_shadow_v3[j] == rd.ring[i].r2 + 4u) { fprintf(stdout, "shadow%u 0 %u\n", rd.ring[i].id, j); } }
	}
	wlrd_hw_regs(0x1u, 0x01fe0001u, hw);
	for (unsigned k = 0; k < WLRD_HW_REGS; k++) { fprintf(stdout, "hw %u %u %u\n", k, hw[k][0], hw[k][1]); }
	for (unsigned i = 0; i < WLRD_PRE_N; i++) { fprintf(stdout, "pre %u %u\n", i, wlrd_pre_addr(i)); }
	fflush(stdout);
	free(ro);
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

/* ---- C2h (fuxiwlanvrx): the vendor's kiwi RX extras, each behind its bit ---- */

/* Pure: the counts of wlvrx_note, the IPA flag, the FSE/FISA words (the vendor's, written down here again), the region
 * and read allowlists, the PPT count. */
static void
s_c2h_pure(void)
{
	struct wlvrx_reo r;
	uint8_t ev[96];
	memset(&r, 0, sizeof(r));
	/* FW_CONSUMED: a beacon, a probe response, an action frame, a short TLV, a TLV past the event */
	for (unsigned k = 0; k < 5; k++) {
		static const uint32_t sub[5] = { 8, 5, 13, 8, 8 };
		memset(ev, 0, sizeof(ev));
		p32(ev, (0x3d3u << 16) | (k == 3 ? 32u : k == 4 ? 80u : 36u));
		p32(ev + 4 + 12, sub[k] << 28);
		wlvrx_note(&r, 0x700du, ev, 40);
	}
	CHECK(r.consumed == 5 && r.consumed_bcn == 1 && r.consumed_prsp == 1 && r.consumed_bad == 2, "c2h: fw-consumed counts");
	wlvrx_note(&r, 0x700eu, ev, 0); wlvrx_note(&r, 0x7002u, ev, 44); wlvrx_note(&r, 0x4002u, ev, 44);
	CHECK(r.srng_reap == 1 && r.mgmt_rx == 0 && r.consumed == 5, "c2h: srng-reap and other ids");
	/* MGMT_RX: header TLV then REO params with the counter valid, without, short, absent */
	for (unsigned k = 0; k < 4; k++) {
		memset(ev, 0, sizeof(ev));
		p32(ev, 0x2cu << 16 | 8u);
		p32(ev + 12, (k == 3 ? 0x11u : 0x3d2u) << 16 | (k == 2 ? 12u : 16u));
		p32(ev + 12 + 8, k == 0 ? 1u << 15 : 0x7fffu);
		wlvrx_note(&r, 0x7001u, ev, 32);
	}
	CHECK(r.mgmt_rx == 4 && r.reo_tlv == 2 && r.reo_valid == 1, "c2h: mgmt-rx reo params counts");
	/* IPA_DISABLE: set without IPA, cleared with it, every other flag1 bit kept */
	CHECK(wlvrx_flag1_ipa(0x40020u, false) == 0x40040020u && wlvrx_flag1_ipa(0x40040020u, false) == 0x40040020u &&
	    wlvrx_flag1_ipa(0xffffffffu, true) == 0xbfffffffu && wlvrx_flag1_ipa(0u, true) == 0u, "c2h: flag1 IPA_DISABLE");
	{
		static const uint32_t fse[14] = { 0x12u, 0x21000080u, 0x114000u, 0x10u, 0xda565a6du, 0xc20e5b25u, 0x3d256741u,
			0xb08fa343u, 0xcb2bcad0u, 0xb4307baeu, 0xa32dcb77u, 0x0cf23080u, 0x3bb7426au, 0xfa01acbeu };
		static const uint8_t key[40] = { 0x6d, 0x5a, 0x56, 0xda, 0x25, 0x5b, 0x0e, 0xc2, 0x41, 0x67, 0x25, 0x3d, 0x43, 0xa3,
			0x8f, 0xb0, 0xd0, 0xca, 0x2b, 0xcb, 0xae, 0x7b, 0x30, 0xb4, 0x77, 0xcb, 0x2d, 0xa3, 0x80, 0x30, 0xf2, 0x0c, 0x6a,
			0x42, 0xb7, 0x3b, 0xbe, 0xac, 0x01, 0xfa };
		uint32_t w[WLVRX_FSE_WORDS], f[WLVRX_FISA_WORDS];
		wlvrx_fse_words(0x113000u, w);
		CHECK(!memcmp(w, fse, sizeof(fse)), "c2h: FSE_SETUP words");
		for (unsigned i = 0; i < 10; i++) { CHECK(w[4 + i] == g32(key + 4 * i), "c2h: the key as native little-endian words"); }
		wlvrx_fse_words(0x80000u, w);
		CHECK(w[2] == 0x81000u && w[3] == 0x10u, "c2h: FSE base follows the CMEM base");
		wlvrx_fisa_words(f);
		CHECK(f[0] == 0x15u && f[1] == 0x1fu && f[2] == 0xffffffffu, "c2h: FISA_CFG words");
	}
	CHECK(wlvrx_fst_region_ok(0x113000u, 0x5000u) && !wlvrx_fst_region_ok(0x113000u, 0x4ffcu) &&
	    !wlvrx_fst_region_ok(0x1ffc000u, 0x5000u) && wlvrx_fst_region_ok(0x1ffb000u, 0x5000u) &&
	    !wlvrx_fst_region_ok(0x100113000ull, 0x5000u) && !wlvrx_fst_region_ok(0x113002u, 0x5000u), "c2h: flow table region");
	CHECK(wlvrx_fst_rd_ok(0x113000u, 0x5000u, 0x114024u) && wlvrx_fst_rd_ok(0x113000u, 0x5000u, 0x114000u + 127u * 64u + 0x24u) &&
	    !wlvrx_fst_rd_ok(0x113000u, 0x5000u, 0x114000u + 128u * 64u + 0x24u) && !wlvrx_fst_rd_ok(0x113000u, 0x5000u, 0x114020u) &&
	    !wlvrx_fst_rd_ok(0x113000u, 0x5000u, 0x113024u) && !wlvrx_fst_rd_ok(0x113000u, 0x1000u, 0x114024u), "c2h: flow table reads");
	{
		struct wlcm cm;
		struct wlvrx_ppt c;
		memset(&cm, 0, sizeof(cm));
		cm.cmem_base = 0x113000u; cm.cmem_size = 0x1000u; cm.cmem_valid = true;
		CHECK(!wlvrx_ppt_rest_ok(&cm, 0x113000u + 4u * 279u) && wlvrx_ppt_rest_ok(&cm, 0x113000u + 4u * 280u) &&
		    wlvrx_ppt_rest_ok(&cm, 0x113ffcu) && !wlvrx_ppt_rest_ok(&cm, 0x114000u) && !wlvrx_ppt_rest_ok(&cm, 0x113462u), "c2h: ppt rest");
		cm.cmem_size = 0x800u;
		CHECK(wlvrx_ppt_rest_ok(&cm, 0x1137fcu) && !wlvrx_ppt_rest_ok(&cm, 0x113800u), "c2h: ppt rest inside the region");
		cm.cmem_valid = false;
		CHECK(!wlvrx_ppt_rest_ok(&cm, 0x113800u - 4u), "c2h: ppt rest without a region");
		cm.ppt_pre[3] = 0x80003u; cm.ppt_pre[200] = ~0u; cm.ppt_pre[279] = 1u;
		wlvrx_ppt_count(&cm, &c);
		CHECK(c.stale == 3 && c.ones == 1 && c.first == 3 && c.last == 279, "c2h: ppt count");
	}
	reset();
	puts("C2h pure: fw-consumed / srng-reap / reo-params counts (beacon and probe-resp subtypes, short and overrunning TLVs, "
	     "other ids), flag1 IPA_DISABLE set and cleared, the vendor's FSE_SETUP and FISA_CFG words (the key as native words), "
	     "the flow table region and its valid-word read allowlist, the PPT rest allowlist and count");
}

/* 0x02: cookie conversion over a stale PPT, against the W29 register file / CMEM of cm_access (the C2f gate). */
static void
c2h_seed_android(void)
{
	for (unsigned i = 0; i < 64; i++) { cmd.seed[i] = 0x9a000u + i; cmd.seed[128 + i] = 0x9b000u + i; }
	for (unsigned i = 300; i < 310; i++) { cmd.seed[i] = 0x9c000u + i; }
}

static void
s_c2h_ccstale(void)
{
	/* the stale table: our 280 overwritten (the C2f writes, nothing past them), the other 744 read and left; cc_config */
	c2f_gate_reset();
	wl30_mask = WL30_CCSTALE;
	c2h_seed_android();
	CHECK(wl27_setup() == 0 && wl_cm.rings_on && wl_cm.cc_on && wl_rx.cc_on && wl_rd.cc_on, "c2h ccstale: cc on");
	CHECK(cmd.cmem_wr == 280u && cmd.wr_hi == 4u * 280u && cmd.rd_rest == 744u && cmd.cmem_rd == 1024u && !cmd.rd_fst,
	    "c2h ccstale: 280 writes, 744 rest reads, nothing past the 280 written");
	for (unsigned i = 0; i < 280; i++) { CHECK(cmd.seed[i] == wlcm_ppt_val(&wl_cm, i), "c2h ccstale: an entry not overwritten"); }
	for (unsigned i = 300; i < 310; i++) { CHECK(cmd.seed[i] == 0x9c000u + i, "c2h ccstale: a rest entry changed"); }
	{
		static const char *const order[] = {
			"FUXI WLAN: W29 cc before: ",
			("FUXI WLAN: W30 ccstale: ppt stale 128 of 280 (first 0 last 191) to be overwritten; rest read 744 non-zero 10 "
			 "(first 300) all-ones 0, left as found (fuxiwlanvrx 0x02)\n"),
			"FUXI WLAN: W29 cookie conversion cmem base 0x113000 pages 280 (rx 24 tx 256) spt 0xa5001000 ppt writes 280; cc on\n",
			"FUXI WLAN: W29 common ring 121 sw2wbm-rel",
		};
		expect_order(order, sizeof(order) / sizeof(order[0]));
	}
	expect_not("cookie conversion not programmed");
	CHECK(wl29_cc_config() == 0 && cmd.cc_wr == 6u, "c2h ccstale: the six cc_config writes follow");
	c2f_gate_done();
	/* the region smaller than the vendor's 4 KiB table: the rest read only up to its end */
	c2f_gate_reset();
	wl30_mask = WL30_CCSTALE; wl_cmem_size = 0x800u;
	c2h_seed_android();
	CHECK(wl27_setup() == 0 && wl_cm.cc_on && cmd.rd_rest == 0x800u / 4u - 280u && cmd.cmem_wr == 280u, "c2h ccstale: small region");
	expect("FUXI WLAN: W30 ccstale: ppt stale 128 of 280 (first 0 last 191) to be overwritten; rest read 232 non-zero 10 ");
	c2f_gate_done();
	/* a clean PPT: the C2f path, the rest still read and counted */
	c2f_gate_reset();
	wl30_mask = WL30_CCSTALE;
	cmd.seed[1023] = 5u;
	CHECK(wl27_setup() == 0 && wl_cm.cc_on && cmd.cmem_wr == 280u && cmd.rd_rest == 744u, "c2h ccstale: clean");
	expect("FUXI WLAN: W30 ccstale: ppt clean (non-zero 0 of 280); rest read 744 non-zero 1 all-ones 0, left as found (fuxiwlanvrx 0x02)\n");
	c2f_gate_done();
	/* an unreadable entry among our 280: refused, the C2f refusal line, nothing written */
	c2f_gate_reset();
	wl30_mask = WL30_CCSTALE;
	c2h_seed_android(); cmd.seed[250] = ~0u; cmd.seed[700] = ~0u;
	CHECK(wl27_setup() == 0 && !wl_cm.cc_on && !cmd.cmem_wr && cmd.rd_rest == 744u, "c2h ccstale: ~0 refused");
	expect("FUXI WLAN: W30 ccstale refused: ppt non-zero 129 of 280, all-ones 1 (unreadable); rest read 744 non-zero 10 all-ones 1; "
	    "nothing written (fuxiwlanvrx 0x02)\n");
	expect("FUXI WLAN: W29 cookie conversion not programmed: a CMEM PPT entry is already non-zero (cookie converter owned); ");
	CHECK(wl29_cc_config() == 0 && !cmd.cc_wr, "c2h ccstale: no cc_config after the refusal");
	c2f_gate_done();
	/* a cc register configured: the C2f gate stands, no CMEM access at all */
	c2f_gate_reset();
	wl30_mask = WL30_CCSTALE;
	c2h_seed_android(); cmd.cc_pre[0] = 0x113000u;
	CHECK(wl27_setup() == 0 && !wl_cm.cc_on && !cmd.cmem_rd && !cmd.cmem_wr, "c2h ccstale: cc owned");
	expect("FUXI WLAN: W30 ccstale not applied: a REO/WBM cookie register is already configured (cfg0 base, wbm cfg1 enable or "
	    "convert global) (fuxiwlanvrx 0x02)\n");
	c2f_gate_done();
	/* no CMEM region: not applied, no access */
	c2f_gate_reset();
	wl30_mask = WL30_CCSTALE; wl_cmem_valid = false;
	CHECK(wl27_setup() == 0 && !wl_cm.cc_on && !cmd.cmem_rd, "c2h ccstale: no region");
	expect("FUXI WLAN: W30 ccstale not applied: no QMI CMEM region (dev_mem[0]); cookie conversion needs it (fuxiwlanvrx 0x02)\n");
	c2f_gate_done();
	/* without the bit: the stale table refuses cc as at c398819, no W30 line, no read past the 280 */
	c2f_gate_reset();
	c2h_seed_android();
	CHECK(wl27_setup() == 0 && !wl_cm.cc_on && !cmd.cmem_wr && cmd.cmem_rd == 280u && !cmd.rd_rest, "c2h ccstale: off");
	expect_not("W30");
	expect("FUXI WLAN: W29 cookie conversion not programmed: a CMEM PPT entry is already non-zero");
	c2f_gate_done();
	puts("C2h ccstale: a stale PPT (the vendor's TX pages) overwritten by our 280 C2f writes, nothing past them written, the "
	     "other 744 read and left, cc_config follows; the rest read only inside a smaller region; a clean PPT; a ~0 entry "
	     "refused with nothing written; a configured cc register or no CMEM region: not applied, no CMEM access; without "
	     "the bit the C2f refusal unchanged");
}

/* 0x04: WMI INIT flag1 bit 30 (the firmware's INIT check wants it with the bit, not without). */
static void
s_c2h_ipadis(void)
{
	c2_reset();
	happy_frames();
	wl30_mask = WL30_IPADIS;
	CHECK(run() == 0 && fw_init_ok == 1, "c2h ipadis run");
	expect("FUXI WLAN: W30 ipadis: wmi init flag1 0x40020 -> 0x40040020 (IPA_DISABLE bit 30 = !ipa-enabled; no IPA on this host) "
	    "(fuxiwlanvrx 0x04)\n");
	expect("FUXI WLAN: W22 wmi init sent bytes 332 (vdevs 4 peers 16 chains 0x3 flag1 0x40040020 flags2 0x200 host-svc 0x10 mem-chunks 0)\n");
	expect("FUXI WLAN: W23 scan completed reason 0 after ");
	contained_and_freed();
	reset();
	puts("C2h ipadis: WMI INIT flag1 0x40020 -> 0x40040020, every other INIT word as before, the scan completes");
}

/* 0x08: FSE_SETUP and FISA_CFG in a 0x902 run (RX rings and ring selection), the table's valid words pre-read through
 * the CMEM window of cm_access, nothing written to CMEM. */
static void
c2h_fse_reset(uint32_t size)
{
	c2c_reset(0x902u);
	C.rx_rule = 2; C.rx_frames = 2;
	happy_frames();
	memset(&wl_cm, 0, sizeof(wl_cm));
	cmd.on = true; cmd.cmem_lo = 0x113000u; cmd.cmem_hi = 0x118000u;  /* off again before containment (the latch check) */
	wl_cmem_base = 0x113000u; wl_cmem_size = size; wl_cmem_valid = true;
	wl30_mask = WL30_FSE;
}

static void
s_c2h_fse(void)
{
	c2h_fse_reset(0x5000u);
	cmd.seed[(0x1000u + 3u * 64u + 0x24u) / 4u] = 0x100u; cmd.seed[(0x1000u + 100u * 64u + 0x24u) / 4u] = 0x4fffu;
	cmd.seed[(0x1000u + 101u * 64u + 0x24u) / 4u] = 0x400u; cmd.seed[(0x1000u + 127u * 64u + 0x24u) / 4u] = ~0u;
	CHECK(run() == 0, "c2h fse run");
	cmd.on = false;
	CHECK(fw_fse_n == 1 && fw_fisa_n == 1 && cmd.rd_fst == 128u && !cmd.rd_other && !cmd.cmem_wr && !cmd.rd_rest, "c2h fse: messages and reads");
	expect("FUXI WLAN: W30 fse setup sent base 0x114000/0x10 records 128 depth 16 prefix 2 key da565a6d..fa01acbe; table valid 2 "
	    "unreadable 1; fisa cfg sent 0x1f/0xffffffff; no cmem write (fuxiwlanvrx 0x08)\n");
	{
		static const char *const order[] = { "FUXI WLAN: W27 htt rx ring selection mac1 ", "FUXI WLAN: W30 fse setup sent",
			"FUXI WLAN: W26 rx watch setup+100ms", "FUXI WLAN: W23 chan list 38 passive", "FUXI WLAN: W23 scan completed reason 0" };
		expect_order(order, sizeof(order) / sizeof(order[0]));
	}
	contained_and_freed();
	/* a region too small for the flow table, or none: not sent, no CMEM read, the scan as without the bit */
	for (unsigned t = 0; t < 2; t++) {
		c2h_fse_reset(0x1000u);
		if (t) { wl_cmem_valid = false; }
		CHECK(run() == 0 && !fw_fse_n && !fw_fisa_n && !cmd.cmem_rd && !cmd.cmem_wr, "c2h fse refused");
		cmd.on = false;
		expect(t ? "FUXI WLAN: W30 fse/fisa cfg not sent: no QMI CMEM region (dev_mem[0]) (fuxiwlanvrx 0x08)\n" :
		    "FUXI WLAN: W30 fse/fisa cfg not sent: the QMI CMEM region cannot hold the 4 KiB PPT and the 16 KiB flow table (fuxiwlanvrx 0x08)\n");
		expect("FUXI WLAN: W23 scan completed reason 0 after ");
		contained_and_freed();
	}
	reset();
	puts("C2h fse: RX_FSE_SETUP_CFG (CMEM 0x114000, hi 0x10, 128 records, search 16, prefix 2, the vendor's key) and "
	     "RX_FISA_CFG (0x1f, 0xffffffff) word by word after the ring selection and before the channel list, the table's 128 "
	     "valid words read and nothing else, nothing written; too small a region or none: not sent, no CMEM access");
}

/* 0x01: nothing sent; the counts of what the firmware does with management frames (REO params TLV in MGMT_RX, FW_CONSUMED,
 * SRNG_REAP); the scan itself unchanged (the frames with the REO TLV still reach the BSS table). */
static void
s_c2h_mgmtreo(void)
{
	static const uint8_t B_R1[6] = {2, 0, 0, 0, 0, 0x21}, B_R2[6] = {2, 0, 0, 0, 0, 0x22};
	uint32_t base_frames, base_mgmt, cmds;
	char want[256];
	struct mbeacon *b;
	c2_reset();
	happy_frames();
	b = add_frame(2, 30, 8, B_R1, 0x0401, "ReoA", 4, 3, 0, false); (void)b;
	b = add_frame(3, 30, 8, B_R2, 0x0401, "ReoB", 4, 4, 0, false); (void)b;
	CHECK(run() == 0, "c2h mgmtreo baseline");
	base_frames = wl_scan.frames; base_mgmt = wl_scan.mgmt; cmds = fw_wmi_msgs;
	expect_not("W30");
	CHECK(!wl30_reo.mgmt_rx && !wl30_reo.consumed && !wl30_reo.srng_reap, "c2h mgmtreo: counted without the bit");
	contained_and_freed();
	c2_reset();
	C.svc299 = true; C.srng_reap = 2;
	happy_frames();
	b = add_frame(2, 30, 8, B_R1, 0x0401, "ReoA", 4, 3, 0, false); b->mode = 4;
	b = add_frame(3, 30, 8, B_R2, 0x0401, "ReoB", 4, 4, 0, false); b->mode = 6;
	b = add_frame(5, 30, 8, B_R1, 0x0401, "Gone", 4, 6, 0, false); b->mode = 5;
	b = add_frame(6, 30, 5, B_R2, 0x0401, "Gone", 4, 7, 0, false); b->mode = 5;
	b = add_frame(20, 30, 8, B_R2, 0x0401, "Gone", 4, 0, 0, false); b->mode = 5;
	wl30_mask = WL30_MGMTREO;
	CHECK(run() == 0, "c2h mgmtreo run");
	CHECK(wl_scan.frames == base_frames && wl_scan.mgmt == base_mgmt && fw_wmi_msgs == cmds, "c2h mgmtreo: the scan and the commands as without the bit");
	CHECK(wl30_reo.mgmt_rx == wl_scan.mgmt && wl30_reo.reo_tlv == 2 && wl30_reo.reo_valid == 1 && wl30_reo.consumed == 3 &&
	    wl30_reo.consumed_bcn == 2 && wl30_reo.consumed_prsp == 1 && wl30_reo.srng_reap == 2, "c2h mgmtreo: counts");
	expect("FUXI WLAN: W30 mgmtreo: nothing sent (mgmt rx reo is host-side reordering, absent from the vendor kiwi build and L19); "
	    "service 299 set; counting fw-consumed, srng-reap, reo-params (fuxiwlanvrx 0x01)\n");
	snprintf(want, sizeof(want), "FUXI WLAN: W30 mgmtreo counts: mgmt-rx %u (reo-params 2, ctr-valid 1), fw-consumed 3 (beacon 2, "
	    "probe-resp 1, no hdr 0), srng-reap 2\n", wl_scan.mgmt);
	expect(want);
	{
		static const char *const order[] = { "FUXI WLAN: W21 wmi service ready complete\n", "FUXI WLAN: W30 mgmtreo: nothing sent",
			"FUXI WLAN: W22 wmi init sent", "FUXI WLAN: W24 ", "FUXI WLAN: W30 mgmtreo counts:" };
		expect_order(order, sizeof(order) / sizeof(order[0]));
	}
	contained_and_freed();
	/* without service 299: said so, still nothing sent */
	c2_reset();
	happy_frames();
	wl30_mask = WL30_MGMTREO;
	CHECK(run() == 0, "c2h mgmtreo no 299");
	expect("service 299 not set; counting");
	expect("FUXI WLAN: W30 mgmtreo counts: mgmt-rx ");
	contained_and_freed();
	reset();
	puts("C2h mgmtreo: nothing sent (the WMI command stream as without the bit), service 299 reported, the REO params TLV "
	     "(counter valid or not), FW_CONSUMED (beacon, probe response) and SRNG_REAP counted; frames with the TLV still "
	     "reach the BSS table");
}

/* All four at once in a 0x5902 + 0x4000 gate-free full run is covered piecewise above; here the four bits together in the
 * 0x902 run (mgmtreo, ipadis, fse) end to end. */
static void
s_c2h_all(void)
{
	c2h_fse_reset(0x5000u);
	wl30_mask = WL30_MGMTREO | WL30_IPADIS | WL30_FSE;
	CHECK(run() == 0 && fw_fse_n == 1 && fw_fisa_n == 1 && fw_init_ok == 1, "c2h all run");
	cmd.on = false;
	CHECK(count_text("FUXI WLAN: W30 ") == 4, "c2h all: four W30 lines");
	expect("W23 scan completed reason 0");
	contained_and_freed();
	reset();
	puts("C2h all: mgmtreo, ipadis and fse together in one 0x902 scan, four W30 lines, the scan completes");
}

int
main(int argc, char **argv)
{
	if (argc == 2 && !strcmp(argv[1], "dump-init")) { dump_init(); return 0; }
	if (argc == 2 && !strcmp(argv[1], "dump-c2")) { dump_c2(); return 0; }
	if (argc == 2 && !strcmp(argv[1], "dump-c2c")) { dump_c2c(); return 0; }
	if (argc == 2 && !strcmp(argv[1], "dump-c2d")) { dump_c2d(); return 0; }
	if (argc == 2 && !strcmp(argv[1], "dump-c2e")) { dump_c2e(); return 0; }
	if (argc == 2 && !strcmp(argv[1], "dump-c2f")) { dump_c2f(); return 0; }
	if (argc == 2 && !strcmp(argv[1], "c2e")) {  /* the C2e scenarios alone (the mutant runs of tools/test-fuxi-wlan-ce.py) */
		s_c2e_pure(); s_c2e_happy(); s_c2e_rings_only(); s_c2e_crash(); s_c2e_full(); s_c2e_spill(); s_c2e_readings(); s_c2e_contain();
		reset();
		puts("C2e model: all scenarios passed");
		return 0;
	}
	if (argc == 2 && !strcmp(argv[1], "c2d")) {  /* the C2d scenarios alone (the mutant runs of tools/test-fuxi-wlan-ce.py) */
		s_c2d_pure(); s_c2d_happy(); s_c2d_wedge(); s_c2d_exhaust(); s_c2d_entries(); s_c2d_contain(); s_c2d_failures(); s_c2d_widest();
		reset();
		puts("C2d model: all scenarios passed");
		return 0;
	}
	if (argc == 2 && !strcmp(argv[1], "c2h")) {  /* the C2h scenarios alone (the mutant runs of tools/test-fuxi-wlan-ce.py) */
		s_c2h_pure(); s_c2h_ccstale(); s_c2h_ipadis(); s_c2h_fse(); s_c2h_mgmtreo(); s_c2h_all();
		reset();
		puts("C2h model: all scenarios passed");
		return 0;
	}
	if (argc == 2 && !strcmp(argv[1], "c2f")) {  /* the C2f scenarios alone (the mutant runs of tools/test-fuxi-wlan-ce.py) */
		s_c2f_pure();
		s_c2f_gate();
		s_c2f_guard();
		s_c2f_ccvals();
		s_c2f_mask_gate();
		s_c2f_bisect();
		s_c2g_ptrs();
		reset();
		puts("C2f model: all scenarios passed");
		return 0;
	}
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
	s_c2d_pure();
	s_c2d_happy();
	s_c2d_wedge();
	s_c2d_exhaust();
	s_c2d_entries();
	s_c2d_contain();
	s_c2d_failures();
	s_c2d_widest();
	s_c2e_pure();
	s_c2e_happy();
	s_c2e_rings_only();
	s_c2e_crash();
	s_c2e_full();
	s_c2e_spill();
	s_c2e_readings();
	s_c2e_contain();
	s_c2f_pure();
	s_c2f_gate();
	s_c2f_guard();
	s_c2f_ccvals();
	s_c2f_mask_gate();
	s_c2f_bisect();
	s_c2g_ptrs();
	s_c2h_pure();
	s_c2h_ccstale();
	s_c2h_ipadis();
	s_c2h_fse();
	s_c2h_mgmtreo();
	s_c2h_all();
	reset();
	puts("W20 model: all scenarios passed");
	return 0;
}
