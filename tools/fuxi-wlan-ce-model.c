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
static void
test_printf(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(logbuf + loglen, sizeof(logbuf) - loglen, fmt, ap);
	va_end(ap);
	assert(n >= 0 && (size_t)n < sizeof(logbuf) - loglen);
	loglen += (size_t)n;
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
		uint32_t pipe_lo = r->ce == 0 ? 0x10000u : 0x18000u, pipe_hi = r->ce == 0 ? 0x18000u : 0x28000u;
		CHECK(r->ce == 0 || r->ce == 3, "TX on a pipe without TX slots");
		while (r->dev != v) {
			uint8_t *e = dev_mem(r->base + 4ull * r->dev, 16, 0x1000, 0x10000, "SRC descriptor outside the rings");
			uint32_t w0 = g32(e), w1 = g32(e + 4), w2 = g32(e + 8), len = w1 >> 16;
			uint64_t a = w0 | (uint64_t)(w1 & 0xff) << 32;
			CHECK((w1 & 0xff00u) == 0, "SRC descriptor byte swap/gather/hash bits set");
			CHECK(len && len <= WLCE_BUF, "SRC length 0 or above 2048");
			CHECK(a == arena_iova + pipe_lo + (uint64_t)(r->dev / 4 % ((pipe_hi - pipe_lo) / WLCE_BUF)) * WLCE_BUF,
			    "SRC buffer is not the TX slot of its entry");
			uint8_t *buf = dev_mem(a, len, pipe_lo, pipe_hi, "TX read outside the pipe's TX slots");
			uint8_t copy[WLCE_BUF];
			memcpy(copy, buf, len);
			r->dev = (r->dev + 4) % r->size;
			write_rdp(r, r->dev);
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
	m_reset();  /* SERVICE_READY_EXT: fixed, 2 hw modes, 2 hw mode caps, 1 MAC/PHY cap with pdev 1 */
	m_tlv(0x1ac, 76); m_zero(76);
	m_tlv(0x212, 8); m_u32(2); m_u32(0);
	m_tlv(0x12, 2 * 16);
	for (unsigned i = 0; i < 2; i++) { m_tlv(0x211, 12); m_u32(i); m_u32(1); m_u32(i); }
	m_tlv(0x12, C.ext_nested_overrun ? 40u : 4u + 108u);
	m_tlv(0x210, C.ext_nested_overrun ? 108u : 108u); m_u32(0); m_u32(1);
	m_zero(C.ext_nested_overrun ? 28u : 100u);
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

/* ---- scenarios ---- */
static void
reset(void)
{
	if (wl_ce_arena.va) { free((void *)wl_ce_arena.va); memset(&wl_ce_arena, 0, sizeof(wl_ce_arena)); }
	memset(&C, 0, sizeof(C));
	C.credits = 4; C.credit_size = 2048; C.ready_ce = 2; C.htt_eid = 1; C.wmi_eid = 2; C.max_msg = 2048; C.ready_stslen = -1;
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
happy_lines(void)
{
	static const char *const lines[] = {
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
		"FUXI WLAN: W22 complete; stop/contain",
	};
	expect_order(lines, sizeof(lines) / sizeof(lines[0]));
	CHECK(fw_init_ok == 1 && fw_hw_mode_ok == 1, "INIT / HW mode not seen by the firmware");
}

static void
s_happy_shadow(void)
{
	reset();
	CHECK(run() == 0, "happy path (shadow) failed");
	happy_lines();
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
	happy_lines();
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
	reset();
	puts("W20 model: all scenarios passed");
	return 0;
}
