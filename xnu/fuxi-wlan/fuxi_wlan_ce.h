/*
 * FUXI WLAN W20/W21/W22 (fuxiwlanqrtr=6, C1): WCN7850 copy engines on SRNG
 * rings, HTC and WMI init to WMI READY, polled. Pure C: no XNU or Linux
 * includes. The includer provides uint8_t..uint64_t, size_t, bool and memcpy
 * (bsd/dev/fuxi_wlan.c in the kernel, tools/fuxi-wlan-ce-model.c on the host).
 * Hardware is reached only through struct wlce_io; every length that comes
 * from the device is bounded by the buffer it arrived in; nothing allocates.
 * Per-field provenance and the log lines: docs-lanes/wlan-w20.md.
 *
 * Derived from Linux ath12k v6.19 (tag v6.19, 05f7e89ab), directory
 * drivers/net/wireless/ath/ath12k, files and their notices:
 *   ce.c, ce.h, hal_desc.h:
 *     Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) 2021-2022, 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 *   hal.c, hal.h, hw.c, hw.h:
 *     Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) 2021-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 *   htc.c:
 *     Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) 2021-2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *   htc.h:
 *     Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *   wmi.c, wmi.h, core.c:
 *     Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *   pci.c:
 *     Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * Every SPDX line there is BSD-3-Clause-Clear. This file is an independent
 * encoding of the wire and register facts; no Linux function, macro or table
 * is copied. File:line references below are into that tree ("L19").
 *
 * The Clear BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FUXI_WLAN_CE_H
#define FUXI_WLAN_CE_H

/* ---- copy engines (SRNG) ----
 * Adapted from Linux ath12k ce.c, ce.h, hal.c, hal.h, hal_desc.h, hw.c and
 * pci.c (BSD-3-Clause-Clear), v6.19. Host CE configuration, ath12k_host_ce_config_wcn7850 (ce.c:145-220):
 * CE0 SRC 16 x 2048 (HTC control), CE1 DST 512 (HTT + HTC control), CE2 DST
 * 64 (WMI), CE3 SRC 32 x 2048 (WMI), CE4 SRC 2048 x 256 (HTT, idle in C1);
 * CE5-8 have no host rings. Ring ids: SRC 64+n, DST 81+n, DST_STATUS 100+n
 * (hal.h:419-468). Entry sizes: SRC 4, DST 2, STATUS 4 dwords (hal_desc.h:
 * 1543-1547, 1631-1634, 1686-1691; hal.c:85-108 directions). */
#define WLCE_NPIPE 5u
#define WLCE_NRING 7u
#define WLCE_BUF 2048u              /* src_sz_max of CE0..CE3; DEST_CTRL max length */
#define WLCE_RX_POST 62u            /* per RX pipe; L19 posts nentries-2 (ce.c:948-949) */
#define WLCE_ID_SRC(n) (64u + (n))
#define WLCE_ID_DST(n) (81u + (n))
#define WLCE_ID_STS(n) (100u + (n))
#define WLCE_RDP_IDS 128u           /* rdp: one u32 per ring id (hal.c:1564-1576) */

/* Register blocks (SoC offsets, BAR0 window 0x37): hw.c:1273-1276 gives CE0
 * SRC 0x01b80000, CE0 DST 0x01b81000, CE1 SRC 0x01b82000; hal.c:1376-1402
 * steps by that 0x2000 for every CE. R0 offsets of host-produced rings are
 * wcn7850_regs hal_tcl1_ring_* minus 0x900 (hw.c:1196-1207, hal.h:68-113);
 * of the device-produced STATUS ring hal_reo1_ring_* minus 0x4e4
 * (hw.c:1244-1253, hal.c:252-295). STATUS R0 at DST block + 0x58, DEST_CTRL
 * at + 0xb0, R2 HP at + 0x400 (STATUS + 0x408), TP 4 above HP
 * (hal.h:116-122, 172-176, 203-210). */
#define WLCE_REG_BASE 0x01b80000u
#define WLCE_REG_END 0x01b8a000u    /* CE0..CE4 SRC and DST blocks */
#define WLCE_SRC_BLOCK(n) (0x01b80000u + 0x2000u * (n))
#define WLCE_DST_BLOCK(n) (0x01b81000u + 0x2000u * (n))
#define WLCE_R0_BASE_LSB 0x00u
#define WLCE_R0_BASE_MSB 0x04u      /* address 39:32 in 7:0, ring size in dwords 27:8 */
#define WLCE_R0_ID 0x08u            /* entry size 7:0; STATUS also ring id 15:8 */
#define WLCE_R0_MISC 0x10u
#define WLCE_R0_TP_ADDR_LSB 0x1cu   /* host-produced rings */
#define WLCE_R0_TP_ADDR_MSB 0x20u
#define WLCE_R0_CONS_IX0 0x30u
#define WLCE_R0_CONS_IX1 0x34u
#define WLCE_R0_HP_ADDR_LSB 0x14u   /* STATUS ring */
#define WLCE_R0_HP_ADDR_MSB 0x18u
#define WLCE_R0_PROD_INT 0x24u
#define WLCE_STS_R0 0x58u
#define WLCE_DEST_CTRL 0xb0u        /* max buffer length 15:0 (hal.h:339) */
#define WLCE_R2_HP 0x400u
#define WLCE_STS_R2_HP 0x408u
#define WLCE_MISC_LOOPCNT_DIS (1u << 1)  /* host-produced only (hal.h:291, hal.c:1798-1799) */
#define WLCE_MISC_SRNG_ENABLE (1u << 6)  /* hal.h:295, 320 */
#define WLCE_RING_SIZE_MAX 0xffffu       /* HAL_CE_*_RING_BASE_MSB_RING_SIZE (hal.h:368-370) */
/* Shadow register i is BAR0 + 0x8fc + 4i, 40 of them (hal.h:38-43): below
 * 0xFE0, so neither the window nor a wake is needed (pci.c:33-39). */
#define WLCE_SHADOW_BASE 0x8fcu
#define WLCE_SHADOW_REGS 40u

/* Arena, one IOVA range of 512 KiB, Normal-NC: rdp, the seven rings, the TX
 * slots of CE0/CE3 (slot i = SRC entry i) and the RX buffers of CE1/CE2.
 * Ring bases are 4 KiB aligned (L19 needs 8, CE_DESC_RING_ALIGN ce.h:108). */
#define WLCE_ARENA_BYTES 0x80000u
#define WLCE_OFF_RDP 0x00000u
#define WLCE_OFF_CE0_SRC 0x01000u   /*   16 x 16 B */
#define WLCE_OFF_CE1_DST 0x02000u   /*  512 x  8 B */
#define WLCE_OFF_CE1_STS 0x03000u   /*  512 x 16 B */
#define WLCE_OFF_CE2_DST 0x05000u   /*   64 x  8 B */
#define WLCE_OFF_CE2_STS 0x06000u   /*   64 x 16 B */
#define WLCE_OFF_CE3_SRC 0x07000u   /*   32 x 16 B */
#define WLCE_OFF_CE4_SRC 0x08000u   /* 2048 x 16 B */
#define WLCE_OFF_CE0_TX 0x10000u    /* 16 x 2 KiB */
#define WLCE_OFF_CE3_TX 0x18000u    /* 32 x 2 KiB */
#define WLCE_OFF_CE1_RX 0x28000u    /* 62 x 2 KiB */
#define WLCE_OFF_CE2_RX 0x47000u    /* 62 x 2 KiB */
#define WLCE_OFF_END 0x66000u
_Static_assert(WLCE_RDP_IDS * 4u <= WLCE_OFF_CE0_SRC && WLCE_OFF_CE0_SRC + 16u * 16u <= WLCE_OFF_CE1_DST &&
    WLCE_OFF_CE1_DST + 512u * 8u <= WLCE_OFF_CE1_STS && WLCE_OFF_CE1_STS + 512u * 16u <= WLCE_OFF_CE2_DST &&
    WLCE_OFF_CE2_DST + 64u * 8u <= WLCE_OFF_CE2_STS && WLCE_OFF_CE2_STS + 64u * 16u <= WLCE_OFF_CE3_SRC &&
    WLCE_OFF_CE3_SRC + 32u * 16u <= WLCE_OFF_CE4_SRC && WLCE_OFF_CE4_SRC + 2048u * 16u <= WLCE_OFF_CE0_TX &&
    WLCE_OFF_CE0_TX + 16u * WLCE_BUF <= WLCE_OFF_CE3_TX && WLCE_OFF_CE3_TX + 32u * WLCE_BUF <= WLCE_OFF_CE1_RX &&
    WLCE_OFF_CE1_RX + WLCE_RX_POST * WLCE_BUF <= WLCE_OFF_CE2_RX &&
    WLCE_OFF_CE2_RX + WLCE_RX_POST * WLCE_BUF == WLCE_OFF_END && WLCE_OFF_END <= WLCE_ARENA_BYTES,
    "W20 arena layout");

/* A device-written rdp word is one 32-bit load; the host model wraps it. */
#ifndef WLCE_RDP_LOAD
#define WLCE_RDP_LOAD(ce, id) ((ce)->rdp[(id)])
#endif

enum { WLCE_SRC, WLCE_DST, WLCE_STS };

struct wlce_ring {                  /* one SRNG */
	uint16_t id;
	uint8_t kind;                   /* WLCE_SRC/WLCE_DST host-produced, WLCE_STS device-produced */
	uint8_t entry_dw;
	uint32_t entries;               /* power of two */
	uint32_t r0, r2;                /* SoC offsets of the R0 base and the R2 HP register */
	uint32_t shadow;                /* BAR0 offset of its shadow register, 0 = R2 through the window */
	uint32_t shadow_idx;
	uint32_t hp, tp;                /* host copies in dwords: SRC/DST hp produced, tp reaped; STATUS tp consumed */
	uint32_t seen;                  /* last device pointer read from rdp */
	uint8_t *va;                    /* ring memory, Normal-NC */
	uint64_t iova;
};

struct wlce_pipe {
	struct wlce_ring src, dst, sts;
	bool has_src, has_dst;
	uint32_t buf_bytes;
	uint32_t slot0, nslots;         /* TX slot or RX buffer range: arena offset, count */
	uint32_t posted, reaped;        /* TX: sent/completed messages; RX: posted/returned buffers */
	uint32_t post_slot, reap_slot;
};

struct wlce_io {                    /* the only way the pure code touches hardware */
	int (*soc_wr)(void *ctx, uint32_t soc_off, uint32_t value);   /* windowed, quiet */
	int (*soc_rd)(void *ctx, uint32_t soc_off, uint32_t *value);
	int (*bar_wr)(void *ctx, uint32_t bar_off, uint32_t value);   /* shadow registers only */
	void (*wbarrier)(void *ctx);    /* before a pointer publish (descriptors and buffers visible) */
	void (*rbarrier)(void *ctx);    /* after a device pointer read (L19 dma_rmb) */
	void *ctx;
};

struct wlce {
	struct wlce_pipe pipe[WLCE_NPIPE];
	struct wlce_ring *ring[WLCE_NRING];   /* L19 init order (ce.c:921-968) */
	volatile uint32_t *rdp;
	uint64_t rdp_iova;
	uint8_t *base;
	uint64_t base_iova;
	uint32_t bytes;
	bool shadow, stopping;
	unsigned writes, errors;
	const char *why;                /* first failure, a string literal */
};

static inline uint32_t wlce_get16(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8); }
static inline uint32_t wlce_get32(const uint8_t *p) { return wlce_get16(p) | (wlce_get16(p + 2) << 16); }
static inline void
wlce_put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static inline int
wlce_fail(struct wlce *ce, const char *why)
{
	if (!ce->why) { ce->why = why; }
	ce->errors++;
	return 1;
}

static inline uint32_t wlce_size(const struct wlce_ring *r) { return r->entries * r->entry_dw; }
/* Dwords from A forward to B in a ring of SIZE dwords. */
static inline uint32_t wlce_dist(uint32_t a, uint32_t b, uint32_t size) { return (b + size - a) % size; }
static inline uint32_t wlce_word(const struct wlce_ring *r, uint32_t dw) { return ((const volatile uint32_t *)(const void *)r->va)[dw]; }
static inline void wlce_set_word(struct wlce_ring *r, uint32_t dw, uint32_t v) { ((volatile uint32_t *)(void *)r->va)[dw] = v; }

/* The only CE registers C1 writes: R0 setup and R2 HP/TP of the 7 rings and
 * DEST_CTRL of CE1/CE2. Anything else is refused before the window moves. */
static inline bool
wlce_reg_ok(uint32_t off)
{
	static const uint8_t host_r0[] = { WLCE_R0_BASE_LSB, WLCE_R0_BASE_MSB, WLCE_R0_ID, WLCE_R0_MISC,
		WLCE_R0_TP_ADDR_LSB, WLCE_R0_TP_ADDR_MSB, WLCE_R0_CONS_IX0, WLCE_R0_CONS_IX1 };
	static const uint8_t sts_r0[] = { WLCE_R0_BASE_LSB, WLCE_R0_BASE_MSB, WLCE_R0_ID, WLCE_R0_MISC,
		WLCE_R0_HP_ADDR_LSB, WLCE_R0_HP_ADDR_MSB, WLCE_R0_PROD_INT };
	if (off < WLCE_REG_BASE || off >= WLCE_REG_END || (off & 3)) { return false; }
	uint32_t rel = (off - WLCE_REG_BASE) % 0x2000u, ce = (off - WLCE_REG_BASE) / 0x2000u;
	bool dst = rel >= 0x1000u;
	if (dst) { rel -= 0x1000u; }
	if (dst ? ce != 1 && ce != 2 : ce != 0 && ce != 3 && ce != 4) { return false; }
	if (rel == WLCE_R2_HP || rel == WLCE_R2_HP + 4u) { return true; }
	for (unsigned i = 0; i < sizeof(host_r0); i++) { if (rel == host_r0[i]) { return true; } }
	if (!dst) { return false; }
	if (rel == WLCE_DEST_CTRL || rel == WLCE_STS_R2_HP || rel == WLCE_STS_R2_HP + 4u) { return true; }
	for (unsigned i = 0; i < sizeof(sts_r0); i++) { if (rel == WLCE_STS_R0 + sts_r0[i]) { return true; } }
	return false;
}

/* Place rdp, rings, TX slots and RX buffers in the arena and look every ring
 * pointer up in the shadow-v3 table WLAN_CFG sent: a SRC-direction ring
 * shadows its R2 HP, the STATUS ring its R2 TP (hal.c:2471-2507; order of
 * ath12k_ce_shadow_config ce.c:874-887). SHADOW false (fuxiwlanw15=1, no
 * table in CFG) publishes every pointer to R2 through the window. */
static inline int
wlce_layout(struct wlce *ce, uint8_t *base, uint64_t iova, uint32_t bytes,
    const uint32_t *shadow_tbl, unsigned shadow_n, bool shadow)
{
	static const struct { uint8_t ce, kind; uint32_t entries, ring, buf, nbuf; } plan[WLCE_NRING] = {
		{0, WLCE_SRC, 16, WLCE_OFF_CE0_SRC, WLCE_OFF_CE0_TX, 16},
		{1, WLCE_DST, 512, WLCE_OFF_CE1_DST, WLCE_OFF_CE1_RX, WLCE_RX_POST},
		{1, WLCE_STS, 512, WLCE_OFF_CE1_STS, 0, 0},
		{2, WLCE_DST, 64, WLCE_OFF_CE2_DST, WLCE_OFF_CE2_RX, WLCE_RX_POST},
		{2, WLCE_STS, 64, WLCE_OFF_CE2_STS, 0, 0},
		{3, WLCE_SRC, 32, WLCE_OFF_CE3_SRC, WLCE_OFF_CE3_TX, 32},
		{4, WLCE_SRC, 2048, WLCE_OFF_CE4_SRC, 0, 0},
	};
	for (size_t i = 0; i < sizeof(*ce); i++) { ((uint8_t *)ce)[i] = 0; }
	if (!base || (iova & 0xfffu) || bytes < WLCE_ARENA_BYTES || iova + WLCE_ARENA_BYTES > (1ull << 32)) {
		return wlce_fail(ce, "CE arena bounds/alignment");
	}
	ce->base = base; ce->base_iova = iova; ce->bytes = bytes; ce->shadow = shadow;
	ce->rdp = (volatile uint32_t *)(void *)(base + WLCE_OFF_RDP);
	ce->rdp_iova = iova + WLCE_OFF_RDP;
	for (unsigned i = 0; i < WLCE_NRING; i++) {
		struct wlce_pipe *p = &ce->pipe[plan[i].ce];
		uint8_t kind = plan[i].kind;
		struct wlce_ring *r = kind == WLCE_SRC ? &p->src : kind == WLCE_DST ? &p->dst : &p->sts;
		uint32_t block = kind == WLCE_SRC ? WLCE_SRC_BLOCK(plan[i].ce) : WLCE_DST_BLOCK(plan[i].ce);
		r->kind = kind;
		r->entry_dw = kind == WLCE_DST ? 2 : 4;
		r->entries = plan[i].entries;
		r->id = (uint16_t)(kind == WLCE_SRC ? WLCE_ID_SRC(plan[i].ce) : kind == WLCE_DST ?
		    WLCE_ID_DST(plan[i].ce) : WLCE_ID_STS(plan[i].ce));
		r->r0 = block + (kind == WLCE_STS ? WLCE_STS_R0 : 0u);
		r->r2 = block + (kind == WLCE_STS ? WLCE_STS_R2_HP : WLCE_R2_HP);
		r->va = base + plan[i].ring;
		r->iova = iova + plan[i].ring;
		if (plan[i].nbuf) { p->slot0 = plan[i].buf; p->nslots = plan[i].nbuf; p->buf_bytes = WLCE_BUF; }
		if (kind == WLCE_SRC) { p->has_src = true; }
		if (kind == WLCE_DST) { p->has_dst = true; }
		ce->ring[i] = r;
		uint32_t target = r->r2 + (kind == WLCE_STS ? 4u : 0u);
		unsigned found = 0;
		for (unsigned j = 0; j < shadow_n; j++) {
			if (shadow_tbl[j] != target) { continue; }
			if (found++ || j >= WLCE_SHADOW_REGS) { return wlce_fail(ce, "CE shadow-v3 entry repeated or past register 39"); }
			r->shadow_idx = j;
			r->shadow = WLCE_SHADOW_BASE + 4u * j;
		}
		if (!shadow) { r->shadow = 0; }
		else if (!found) { return wlce_fail(ce, "CE ring pointer missing from the WLAN_CFG shadow-v3 table"); }
	}
	return 0;
}

static inline int
wlce_wr(struct wlce *ce, const struct wlce_io *io, uint32_t off, uint32_t v)
{
	if (ce->stopping) { return wlce_fail(ce, "CE stopping: register write refused"); }
	if (!wlce_reg_ok(off) || io->soc_wr(io->ctx, off, v)) { return wlce_fail(ce, "CE register write refused or failed"); }
	ce->writes++;
	return 0;
}

static inline int
wlce_rd(struct wlce *ce, const struct wlce_io *io, uint32_t off, uint32_t *v)
{
	*v = ~0u;
	if (!wlce_reg_ok(off) || io->soc_rd(io->ctx, off, v)) { return wlce_fail(ce, "CE register read refused or failed"); }
	return 0;
}

/* Polled ring setup in the L19 order: memory and rdp word zeroed
 * (hal.c:2369-2370, 1699, 1787), then for host-produced rings BASE, size,
 * entry size, consumer thresholds 0, TP_ADDR = rdp + 4 * id, R2 HP = TP = 0,
 * MISC (hal.c:1714-1807); for the STATUS ring BASE, size, id << 8 | entry
 * size, producer thresholds 0, HP_ADDR, R2 HP = TP = 0, MISC (hal.c:
 * 1638-1712). No MSI address and no interrupt threshold: L19 writes them only
 * with HAL_SRNG_FLAGS_MSI_INTR / non-DIS_INTR CEs (ce.c:578-614). */
static inline int
wlce_ring_init(struct wlce *ce, const struct wlce_io *io, struct wlce_ring *r)
{
	uint32_t size = wlce_size(r);
	uint64_t ptr = ce->rdp_iova + 4ull * r->id;
	uint32_t lo = (uint32_t)r->iova, msb = (uint32_t)(r->iova >> 32) & 0xffu;
	if (!r->entries || (r->entries & (r->entries - 1)) || size > WLCE_RING_SIZE_MAX || r->id >= WLCE_RDP_IDS) {
		return wlce_fail(ce, "CE ring geometry");
	}
	for (uint32_t i = 0; i < size; i++) { wlce_set_word(r, i, 0); }
	ce->rdp[r->id] = 0;
	r->hp = r->tp = r->seen = 0;
	if (r->kind != WLCE_STS) {
		return wlce_wr(ce, io, r->r0 + WLCE_R0_BASE_LSB, lo) ||
		    wlce_wr(ce, io, r->r0 + WLCE_R0_BASE_MSB, msb | (size << 8)) ||
		    wlce_wr(ce, io, r->r0 + WLCE_R0_ID, r->entry_dw) ||
		    wlce_wr(ce, io, r->r0 + WLCE_R0_CONS_IX0, 0) ||
		    wlce_wr(ce, io, r->r0 + WLCE_R0_CONS_IX1, 0) ||
		    wlce_wr(ce, io, r->r0 + WLCE_R0_TP_ADDR_LSB, (uint32_t)ptr) ||
		    wlce_wr(ce, io, r->r0 + WLCE_R0_TP_ADDR_MSB, (uint32_t)(ptr >> 32)) ||
		    wlce_wr(ce, io, r->r2, 0) || wlce_wr(ce, io, r->r2 + 4u, 0) ||
		    wlce_wr(ce, io, r->r0 + WLCE_R0_MISC, WLCE_MISC_LOOPCNT_DIS | WLCE_MISC_SRNG_ENABLE);
	}
	return wlce_wr(ce, io, r->r0 + WLCE_R0_BASE_LSB, lo) ||
	    wlce_wr(ce, io, r->r0 + WLCE_R0_BASE_MSB, msb | (size << 8)) ||
	    wlce_wr(ce, io, r->r0 + WLCE_R0_ID, ((uint32_t)r->id << 8) | r->entry_dw) ||
	    wlce_wr(ce, io, r->r0 + WLCE_R0_PROD_INT, 0) ||
	    wlce_wr(ce, io, r->r0 + WLCE_R0_HP_ADDR_LSB, (uint32_t)ptr) ||
	    wlce_wr(ce, io, r->r0 + WLCE_R0_HP_ADDR_MSB, (uint32_t)(ptr >> 32)) ||
	    wlce_wr(ce, io, r->r2, 0) || wlce_wr(ce, io, r->r2 + 4u, 0) ||
	    wlce_wr(ce, io, r->r0 + WLCE_R0_MISC, WLCE_MISC_SRNG_ENABLE);
}

/* DEST_CTRL max length, read-modify-write of 15:0, after the DST ring setup
 * (hal.c:1620-1636, 2439-2442; max_buffer_len = src_sz_max ce.c:597-598),
 * then read back (L19 does not): it is the only bound on what the engine
 * writes into one RX buffer, so a value that did not take stops the run
 * before any buffer is posted. */
static inline int
wlce_dest_ctrl(struct wlce *ce, const struct wlce_io *io, const struct wlce_ring *r, uint32_t max_len, uint32_t *readback)
{
	uint32_t v = 0;
	*readback = ~0u;
	if (r->kind != WLCE_DST || !max_len || max_len > 0xffffu) { return wlce_fail(ce, "CE DEST_CTRL target"); }
	if (wlce_rd(ce, io, r->r0 + WLCE_DEST_CTRL, &v) || wlce_wr(ce, io, r->r0 + WLCE_DEST_CTRL, (v & ~0xffffu) | max_len) ||
	    wlce_rd(ce, io, r->r0 + WLCE_DEST_CTRL, readback)) { return 1; }
	return (*readback & 0xffffu) == max_len ? 0 : wlce_fail(ce, "CE DEST_CTRL readback");
}

/* Readback after setup: BASE_LSB exact, MISC with SRNG_ENABLE. */
static inline int
wlce_ring_check(struct wlce *ce, const struct wlce_io *io, const struct wlce_ring *r, uint32_t *base, uint32_t *misc)
{
	if (wlce_rd(ce, io, r->r0 + WLCE_R0_BASE_LSB, base) || wlce_rd(ce, io, r->r0 + WLCE_R0_MISC, misc)) { return 1; }
	if (*base != (uint32_t)r->iova || !(*misc & WLCE_MISC_SRNG_ENABLE)) { return wlce_fail(ce, "CE ring BASE_LSB/MISC readback"); }
	return 0;
}

/* Publish the host pointer (HP of host-produced rings, TP of STATUS) with a
 * write barrier first: L19 relies on the MMIO accessor's wmb for HP and uses
 * mb() before a STATUS TP (hal.c:2194-2215). Shadow register or R2. */
static inline int
wlce_publish(struct wlce *ce, const struct wlce_io *io, struct wlce_ring *r)
{
	uint32_t v = r->kind == WLCE_STS ? r->tp : r->hp;
	if (ce->stopping) { return wlce_fail(ce, "CE stopping: pointer publish refused"); }
	io->wbarrier(io->ctx);
	if (!r->shadow) { return wlce_wr(ce, io, r->r2 + (r->kind == WLCE_STS ? 4u : 0u), v); }
	if (io->bar_wr(io->ctx, r->shadow, v)) { return wlce_fail(ce, "CE shadow register write failed"); }
	ce->writes++;
	return 0;
}

/* Device pointer from rdp (TP of host-produced rings, HP of STATUS), then a
 * read barrier before any descriptor or payload it covers (hal.c:2144-2164). */
static inline int
wlce_device_ptr(struct wlce *ce, const struct wlce_io *io, const struct wlce_ring *r, uint32_t *out)
{
	uint32_t v = WLCE_RDP_LOAD(ce, r->id);
	io->rbarrier(io->ctx);
	if (v >= wlce_size(r) || v % r->entry_dw) { return wlce_fail(ce, "CE device ring pointer outside the ring"); }
	*out = v;
	return 0;
}

/* RX post: one DST descriptor (buffer address, hi 8 bits; hal.c:1941-1947)
 * per buffer at HP, then one HP publish. At most nslots buffers are out. */
static inline int
wlce_post_rx(struct wlce *ce, const struct wlce_io *io, struct wlce_pipe *p, uint32_t n)
{
	struct wlce_ring *d = &p->dst;
	uint32_t size = wlce_size(d);
	if (ce->stopping) { return wlce_fail(ce, "CE stopping: RX post refused"); }
	if (!p->has_dst || !p->nslots) { return wlce_fail(ce, "CE RX post on a pipe without buffers"); }
	for (uint32_t k = 0; k < n; k++) {
		if (p->posted - p->reaped >= p->nslots || (d->hp + d->entry_dw) % size == d->tp) {
			return wlce_fail(ce, "CE RX post beyond the buffers");
		}
		uint64_t a = ce->base_iova + p->slot0 + (uint64_t)p->post_slot * p->buf_bytes;
		wlce_set_word(d, d->hp, (uint32_t)a);
		wlce_set_word(d, d->hp + 1, (uint32_t)(a >> 32) & 0xffu);
		d->hp = (d->hp + d->entry_dw) % size;
		p->post_slot = (p->post_slot + 1) % p->nslots;
		p->posted++;
	}
	return n ? wlce_publish(ce, io, d) : 0;
}

typedef int (*wlce_rx_fn)(void *ctx, const uint8_t *buf, uint32_t len);

/* RX reap (ce.c:410-494): device STATUS HP from rdp; for every new STATUS
 * entry the length is w0 31:16 (0 or above the buffer is an error here; L19
 * warns and drops), the buffer is the next DST entry in posting order, the
 * length field is cleared (hal.c:1949-1957), the payload goes to FN in place,
 * the buffer is posted again; one TP publish at the end. The new entries
 * may not outnumber the buffers out. A STATUS HP that moved back k entries
 * reads as a forward move of (entries - k): that bound stops it only when
 * (entries - k) exceeds the buffers out (on CE2, k = 1); otherwise the length
 * of the first entry, already consumed (cleared) or never written (zero),
 * stops it as a STATUS length 0. */
static inline int
wlce_reap_rx(struct wlce *ce, const struct wlce_io *io, struct wlce_pipe *p, wlce_rx_fn fn, void *ctx)
{
	struct wlce_ring *s = &p->sts, *d = &p->dst;
	uint32_t ssize = wlce_size(s), dsize = wlce_size(d), hp = 0, n = 0;
	if (ce->stopping) { return 0; }
	if (!p->has_dst) { return wlce_fail(ce, "CE RX reap on a pipe without rings"); }
	if (wlce_device_ptr(ce, io, s, &hp)) { return 1; }
	uint32_t avail = wlce_dist(s->tp, hp, ssize);
	if (avail / s->entry_dw > p->posted - p->reaped) { return wlce_fail(ce, "CE STATUS entries beyond the posted buffers"); }
	s->seen = hp;
	while (s->tp != hp && n < s->entries) {
		uint32_t w0 = wlce_word(s, s->tp), len = w0 >> 16;
		uint64_t want = ce->base_iova + p->slot0 + (uint64_t)p->reap_slot * p->buf_bytes;
		const uint8_t *buf = ce->base + p->slot0 + p->reap_slot * p->buf_bytes;
		if (!len || len > p->buf_bytes) { return wlce_fail(ce, "CE STATUS length 0 or above the RX buffer"); }
		if (wlce_word(d, d->tp) != (uint32_t)want || (wlce_word(d, d->tp + 1) & 0xffu) != ((uint32_t)(want >> 32) & 0xffu)) {
			return wlce_fail(ce, "CE DST descriptor changed under the device");
		}
		wlce_set_word(s, s->tp, w0 & 0xffffu);
		s->tp = (s->tp + s->entry_dw) % ssize;
		d->tp = (d->tp + d->entry_dw) % dsize;
		p->reap_slot = (p->reap_slot + 1) % p->nslots;
		p->reaped++; n++;
		if (fn(ctx, buf, len)) { return wlce_fail(ce, "CE RX message rejected by HTC/WMI"); }
		if (wlce_post_rx(ce, io, p, 1)) { return 1; }
	}
	return n ? wlce_publish(ce, io, s) : 0;
}

/* TX completions (ce.c:496-550): the device TP (rdp) may only advance and
 * never pass the host HP. */
static inline int
wlce_reap_tx(struct wlce *ce, const struct wlce_io *io, struct wlce_pipe *p)
{
	struct wlce_ring *r = &p->src;
	uint32_t size = wlce_size(r), t = 0;
	if (ce->stopping) { return 0; }
	if (!p->has_src) { return wlce_fail(ce, "CE TX reap on a pipe without a SRC ring"); }
	if (wlce_device_ptr(ce, io, r, &t)) { return 1; }
	uint32_t done = wlce_dist(r->tp, t, size);
	if (done > wlce_dist(r->tp, r->hp, size)) { return wlce_fail(ce, "CE device TP passed the host HP or went backwards"); }
	r->seen = t;
	r->tp = t;
	p->reaped += done / r->entry_dw;
	return 0;
}

static inline bool wlce_tx_idle(const struct wlce_pipe *p) { return p->src.tp == p->src.hp; }

/* TX (ce.c:725-804, hal.c:1927-1939): copy into the slot of the SRC entry at
 * HP, descriptor = address, hi 8 bits, byte swap 0 (bit 9), gather 0
 * (bit 11), length 31:16, meta 15:0 = HTC endpoint id; w3 belongs to the
 * hardware. A free entry needs next HP != device TP (hal.c:2077-2080). */
static inline int
wlce_send(struct wlce *ce, const struct wlce_io *io, struct wlce_pipe *p, uint32_t eid, const uint8_t *msg, uint32_t len)
{
	struct wlce_ring *r = &p->src;
	uint32_t size = wlce_size(r);
	if (ce->stopping) { return wlce_fail(ce, "CE stopping: send refused"); }
	if (!p->has_src || !p->nslots || !len || len > p->buf_bytes || eid > 0xffffu) { return wlce_fail(ce, "CE TX pipe/length"); }
	if (wlce_reap_tx(ce, io, p)) { return 1; }
	uint32_t next = (r->hp + r->entry_dw) % size, slot = (r->hp / r->entry_dw) % p->nslots;
	if (next == r->tp) { return wlce_fail(ce, "CE SRC ring full"); }
	uint64_t a = ce->base_iova + p->slot0 + (uint64_t)slot * p->buf_bytes;
	memcpy(ce->base + p->slot0 + slot * p->buf_bytes, msg, len);
	wlce_set_word(r, r->hp, (uint32_t)a);
	wlce_set_word(r, r->hp + 1, ((uint32_t)(a >> 32) & 0xffu) | (len << 16));
	wlce_set_word(r, r->hp + 2, eid);
	r->hp = next;
	p->posted++;
	return wlce_publish(ce, io, r);
}

/* ---- HTC (htc.h, htc.c) ----
 * Adapted from Linux ath12k htc.c and htc.h (BSD-3-Clause-Clear), v6.19.
 * Header 8 bytes: endpoint 7:0, flags 15:8, payload length 31:16; then
 * control byte 0 (RX trailer length) 7:0 and control byte 1 (sequence) 15:8
 * (htc.h:18-23, 58-61; htc.c:53-72). */
#define WLHTC_HDR 8u
#define WLHTC_EPS 9u                    /* ATH12K_HTC_EP_COUNT (htc.h:219-231) */
#define WLHTC_MAX_LEN 4096u             /* htc.h:256 */
#define WLHTC_CTL_MAX 256u              /* htc.h:257 */
#define WLHTC_FLAG_NEED_CREDIT 0x01u    /* htc.h:48-51 */
#define WLHTC_FLAG_TRAILER 0x02u        /* htc.h:53-56 */
#define WLHTC_MSG_READY 1u              /* htc.h:63-72 */
#define WLHTC_MSG_CONNECT 2u
#define WLHTC_MSG_CONNECT_RESP 3u
#define WLHTC_MSG_SETUP_COMPLETE_EX 5u
#define WLHTC_READY_BYTES 8u            /* struct ath12k_htc_ready (htc.h:99-102) */
#define WLHTC_CONNECT_BYTES 8u          /* struct ath12k_htc_conn_svc (htc.h:109-112) */
#define WLHTC_CONNECT_RESP_BYTES 12u    /* struct ath12k_htc_conn_svc_resp (htc.h:114-118) */
#define WLHTC_SETUP_COMPLETE_BYTES 12u  /* struct ath12k_htc_setup_complete_extended (htc.h:120-124) */
#define WLHTC_CONN_DISABLE_FLOW (1u << 3)   /* htc.h:88 */
#define WLHTC_CONN_RECV_ALLOC_SHIFT 8u      /* htc.h:89, 15:8 */
#define WLHTC_SVC_RSVD_CTRL 0x0001u     /* htc.h:188-215 */
#define WLHTC_SVC_WMI 0x0100u
#define WLHTC_SVC_HTT 0x0300u
#define WLHTC_RECORD_CREDITS 1u         /* htc.h:131-148 */
#define WLHTC_PIPEDIR_IN 1u             /* ce.h:35-39 */
#define WLHTC_PIPEDIR_OUT 2u
#define WLHTC_PIPEDIR_INOUT 3u
_Static_assert(WLHTC_READY_BYTES == 8 && WLHTC_CONNECT_RESP_BYTES == 12, "HTC wire sizes");

struct wlhtc_ep {
	uint16_t svc, max_msg;
	uint8_t ul_pipe, dl_pipe, seq;
	bool used, flow;
	int credits;
};

struct wlhtc {
	uint16_t credit_size, credit_count;
	uint8_t max_ep;
	struct wlhtc_ep ep[WLHTC_EPS];
	uint8_t ctl[WLHTC_CTL_MAX];
	unsigned ctl_len;
	bool ctl_new, ready;
	uint8_t wmi_eid, htt_eid;
	unsigned bad_reports, ignored, htt_rx, credit_records;
	const uint32_t (*map)[3];       /* the WLAN_CFG service map: service, direction, pipe */
	unsigned map_n;
	const char *why;
};

typedef int (*wlhtc_msg_fn)(void *ctx, const uint8_t *msg, uint32_t len);

static inline int
wlhtc_fail(struct wlhtc *htc, const char *why)
{
	if (!htc->why) { htc->why = why; }
	return 1;
}

/* Pipes of a service from the service map (pci.c:1006-1046): IN = downlink,
 * OUT = uplink, INOUT both; each at most once. */
static inline int
wlhtc_pipes(const uint32_t (*map)[3], unsigned n, uint32_t svc, uint8_t *ul, uint8_t *dl)
{
	bool ul_set = false, dl_set = false;
	for (unsigned i = 0; i < n; i++) {
		if (map[i][0] != svc) { continue; }
		uint32_t dir = map[i][1], pipe = map[i][2];
		if (pipe >= WLCE_NPIPE) { return 1; }
		if (dir == WLHTC_PIPEDIR_IN || dir == WLHTC_PIPEDIR_INOUT) {
			if (dl_set) { return 1; }
			*dl = (uint8_t)pipe; dl_set = true;
		}
		if (dir == WLHTC_PIPEDIR_OUT || dir == WLHTC_PIPEDIR_INOUT) {
			if (ul_set) { return 1; }
			*ul = (uint8_t)pipe; ul_set = true;
		}
	}
	return ul_set && dl_set ? 0 : 1;
}

/* Endpoint 0 is the pseudo-connected RSVD_CTRL service: no request on the
 * wire, credit flow off, 256-byte messages (htc.c:430-444, 575-582, 750-795). */
static inline int
wlhtc_init(struct wlhtc *htc, const uint32_t (*map)[3], unsigned n)
{
	for (size_t i = 0; i < sizeof(*htc); i++) { ((uint8_t *)htc)[i] = 0; }
	htc->map = map; htc->map_n = n;
	struct wlhtc_ep *ep = &htc->ep[0];
	if (wlhtc_pipes(map, n, WLHTC_SVC_RSVD_CTRL, &ep->ul_pipe, &ep->dl_pipe)) {
		return wlhtc_fail(htc, "HTC control service missing from the service map");
	}
	ep->used = true; ep->svc = WLHTC_SVC_RSVD_CTRL; ep->max_msg = WLHTC_CTL_MAX; ep->flow = false;
	return 0;
}

/* Trailer records {id, len, pad, pad} + body; record 1 = credit reports
 * {eid, credits, pad, pad} (htc.c:144-233). A record longer than what is left
 * is an error (L19 compares against the length before its header); a report
 * for an endpoint >= 9 ends that record as in L19; credits for an endpoint
 * that is not a connected credit-flow endpoint are counted and dropped. */
static inline int
wlhtc_trailer(struct wlhtc *htc, const uint8_t *p, uint32_t len)
{
	while (len) {
		if (len < 4) { return wlhtc_fail(htc, "HTC trailer record header truncated"); }
		uint32_t id = p[0], rlen = p[1];
		if (rlen > len - 4) { return wlhtc_fail(htc, "HTC trailer record longer than the trailer"); }
		if (id == WLHTC_RECORD_CREDITS) {
			if (rlen < 4) { return wlhtc_fail(htc, "HTC credit record shorter than one report"); }
			htc->credit_records++;
			for (uint32_t i = 0; i + 4 <= rlen; i += 4) {
				uint32_t e = p[4 + i], c = p[5 + i];
				if (e >= WLHTC_EPS) { htc->bad_reports++; break; }
				if (htc->ep[e].used && htc->ep[e].flow) { htc->ep[e].credits += (int)c; }
				else { htc->bad_reports++; }
			}
		}
		p += 4 + rlen; len -= 4 + rlen;
	}
	return 0;
}

/* One received HTC message (htc.c:252-378): endpoint < 9, header + payload
 * within the CE transfer and 4096, trailer 4..payload; a message with no
 * payload left (trailer only, or empty) is dropped on every endpoint as in
 * L19 (htc.c:321-323). EP0 READY/CONNECT_RESP are copied (256 bytes at
 * most) for the waiting step; another unread control message is an error.
 * WMI messages go to FN; HTT messages are counted (no HTT parsing in C1). */
static inline int
wlhtc_rx(struct wlhtc *htc, const uint8_t *msg, uint32_t len, wlhtc_msg_fn fn, void *ctx)
{
	if (len < WLHTC_HDR) { return wlhtc_fail(htc, "HTC message shorter than its header"); }
	uint32_t w0 = wlce_get32(msg), w1 = wlce_get32(msg + 4);
	uint32_t eid = w0 & 0xffu, flags = (w0 >> 8) & 0xffu, plen = w0 >> 16;
	const uint8_t *pl = msg + WLHTC_HDR;
	if (eid >= WLHTC_EPS) { return wlhtc_fail(htc, "HTC endpoint id out of range"); }
	if (plen + WLHTC_HDR > WLHTC_MAX_LEN) { return wlhtc_fail(htc, "HTC payload above 4096 bytes"); }
	if (plen > len - WLHTC_HDR) { return wlhtc_fail(htc, "HTC payload longer than the CE transfer"); }
	if (flags & WLHTC_FLAG_TRAILER) {
		uint32_t tlen = w1 & 0xffu;
		if (tlen < 4 || tlen > plen) { return wlhtc_fail(htc, "HTC trailer length outside 4..payload"); }
		if (wlhtc_trailer(htc, pl + plen - tlen, tlen)) { return 1; }
		plen -= tlen;
	}
	if (!plen) { return 0; }
	if (!htc->ep[eid].used) { return wlhtc_fail(htc, "HTC message on an unconnected endpoint"); }
	if (eid == 0) {
		if (plen < 4) { return wlhtc_fail(htc, "HTC control message shorter than 4 bytes"); }
		uint32_t id = wlce_get16(pl);
		if (id != WLHTC_MSG_READY && id != WLHTC_MSG_CONNECT_RESP) { htc->ignored++; return 0; }
		if (htc->ctl_new) { return wlhtc_fail(htc, "HTC control message while the previous one is unread"); }
		htc->ctl_len = plen < WLHTC_CTL_MAX ? plen : WLHTC_CTL_MAX;
		memcpy(htc->ctl, pl, htc->ctl_len);
		htc->ctl_new = true;
		return 0;
	}
	if (eid == htc->wmi_eid) { return fn(ctx, pl, plen); }
	htc->htt_rx++;
	return 0;
}

/* READY on EP0: message id 15:0, credit count 31:16; credit size 15:0, max
 * endpoints 23:16; both counts non-zero (htc.c:491-555, htc.h:28-30). */
static inline int
wlhtc_parse_ready(struct wlhtc *htc)
{
	if (!htc->ctl_new) { return wlhtc_fail(htc, "HTC READY not received"); }
	htc->ctl_new = false;
	if (htc->ctl_len < WLHTC_READY_BYTES) { return wlhtc_fail(htc, "HTC READY shorter than 8 bytes"); }
	uint32_t w0 = wlce_get32(htc->ctl), w1 = wlce_get32(htc->ctl + 4);
	if ((w0 & 0xffffu) != WLHTC_MSG_READY) { return wlhtc_fail(htc, "HTC READY message id"); }
	htc->credit_count = (uint16_t)(w0 >> 16);
	htc->credit_size = (uint16_t)(w1 & 0xffffu);
	htc->max_ep = (uint8_t)(w1 >> 16);
	if (!htc->credit_count || !htc->credit_size) { return wlhtc_fail(htc, "HTC READY without credits"); }
	htc->ready = true;
	return 0;
}

/* CONNECT request: message 2 | service << 16, connection flags 15:0
 * (htc.c:597-617, htc.h:25-26). */
static inline void
wlhtc_connect_req(uint8_t *p, uint32_t svc, uint32_t flags)
{
	wlce_put32(p, WLHTC_MSG_CONNECT | (svc << 16));
	wlce_put32(p + 4, flags & 0xffffu);
}

/* CONNECT reply: message 3 | service << 16; status 7:0, endpoint 15:8, max
 * message 31:16 (htc.c:635-719, htc.h:35-39). The service must be the one
 * asked for (L19 does not check). The endpoint gets the credits we asked for
 * (htc.c:694) and the pipes of the service map. */
static inline int
wlhtc_parse_connect(struct wlhtc *htc, uint32_t svc, bool flow, uint32_t alloc, uint32_t *eid, uint32_t *status)
{
	*eid = WLHTC_EPS; *status = ~0u;
	if (!htc->ctl_new) { return wlhtc_fail(htc, "HTC connect reply not received"); }
	htc->ctl_new = false;
	if (htc->ctl_len < WLHTC_CONNECT_RESP_BYTES) { return wlhtc_fail(htc, "HTC connect reply shorter than 12 bytes"); }
	uint32_t w0 = wlce_get32(htc->ctl), w1 = wlce_get32(htc->ctl + 4);
	if ((w0 & 0xffffu) != WLHTC_MSG_CONNECT_RESP) { return wlhtc_fail(htc, "HTC connect reply message id"); }
	if ((w0 >> 16) != svc) { return wlhtc_fail(htc, "HTC connect reply for another service"); }
	*status = w1 & 0xffu; *eid = (w1 >> 8) & 0xffu;
	uint32_t max_msg = w1 >> 16;
	if (*status) { return wlhtc_fail(htc, "HTC service connect refused by the target"); }
	if (*eid >= WLHTC_EPS || !max_msg) { return wlhtc_fail(htc, "HTC connect reply endpoint/max message"); }
	struct wlhtc_ep *ep = &htc->ep[*eid];
	if (ep->used) { return wlhtc_fail(htc, "HTC endpoint already in use"); }
	if (wlhtc_pipes(htc->map, htc->map_n, svc, &ep->ul_pipe, &ep->dl_pipe)) {
		return wlhtc_fail(htc, "HTC service missing from the service map");
	}
	ep->used = true; ep->svc = (uint16_t)svc; ep->max_msg = (uint16_t)max_msg;
	ep->flow = flow; ep->credits = flow ? (int)alloc : 0;
	if (svc == WLHTC_SVC_WMI) { htc->wmi_eid = (uint8_t)*eid; }
	if (svc == WLHTC_SVC_HTT) { htc->htt_eid = (uint8_t)*eid; }
	return 0;
}

/* SETUP_COMPLETE_EX: message 5, flags 0, max bundled 0 (htc.c:721-748). */
static inline void
wlhtc_setup_complete(uint8_t *p)
{
	wlce_put32(p, WLHTC_MSG_SETUP_COMPLETE_EX); wlce_put32(p + 4, 0); wlce_put32(p + 8, 0);
}

/* Header + payload for endpoint EID; NEED_CREDIT_UPDATE on credit-flow
 * endpoints only, sequence per endpoint (htc.c:53-72). */
static inline int
wlhtc_frame(struct wlhtc *htc, uint32_t eid, const uint8_t *payload, uint32_t plen, uint8_t *out, uint32_t room, uint32_t *total)
{
	if (eid >= WLHTC_EPS || !htc->ep[eid].used) { return wlhtc_fail(htc, "HTC send on an unconnected endpoint"); }
	struct wlhtc_ep *ep = &htc->ep[eid];
	uint32_t t = WLHTC_HDR + plen;
	if (t > room || t > WLHTC_MAX_LEN || plen > 0xffffu) { return wlhtc_fail(htc, "HTC frame too long"); }
	wlce_put32(out, eid | ((ep->flow ? WLHTC_FLAG_NEED_CREDIT : 0u) << 8) | (plen << 16));
	wlce_put32(out + 4, (uint32_t)ep->seq++ << 8);
	memcpy(out + WLHTC_HDR, payload, plen);
	*total = t;
	return 0;
}

/* A message costs ceil(total length with header / credit size) (htc.c:90-103). */
static inline uint32_t
wlhtc_credits_needed(const struct wlhtc *htc, uint32_t total)
{
	return htc->credit_size ? (total + htc->credit_size - 1u) / htc->credit_size : ~0u;
}

static inline bool
wlhtc_has_credits(const struct wlhtc *htc, uint32_t eid, uint32_t total)
{
	const struct wlhtc_ep *ep = &htc->ep[eid];
	return eid < WLHTC_EPS && (!ep->flow || (ep->credits >= 0 && (uint32_t)ep->credits >= wlhtc_credits_needed(htc, total)));
}

static inline int
wlhtc_take_credits(struct wlhtc *htc, uint32_t eid, uint32_t total)
{
	if (!wlhtc_has_credits(htc, eid, total)) { return wlhtc_fail(htc, "HTC send without credits"); }
	if (htc->ep[eid].flow) { htc->ep[eid].credits -= (int)wlhtc_credits_needed(htc, total); }
	return 0;
}

/* ---- WMI (wmi.h, wmi.c) ----
 * Adapted from Linux ath12k wmi.c, wmi.h, hw.h and core.c
 * (BSD-3-Clause-Clear), v6.19. Command/event header u32, id 23:0 (wmi.h:69-82); TLV header u32, tag 31:16,
 * length 15:0 excluding the header (wmi.h:73-80, wmi.c:198-207). Ids and tags
 * resolved from the wmi.h enums (the host test re-derives them). */
#define WLWMI_CMD_INIT 0x1u
#define WLWMI_CMD_PDEV_SET_HW_MODE 0x401eu
#define WLWMI_EV_SERVICE_READY 0x1u
#define WLWMI_EV_READY 0x2u
#define WLWMI_EV_SERVICE_AVAILABLE 0x3u
#define WLWMI_EV_SERVICE_READY_EXT 0x4009u
#define WLWMI_EV_PDEV_SET_HW_MODE_RESP 0x4010u
#define WLWMI_EV_SERVICE_READY_EXT2 0x4022u
#define WLWMI_EV_REG_CHAN_LIST_CC_EXT 0x3a003u
#define WLWMI_TAG_ARRAY_UINT32 0x10u
#define WLWMI_TAG_ARRAY_STRUCT 0x12u
#define WLWMI_TAG_SERVICE_READY_EVENT 0x20u
#define WLWMI_TAG_READY_EVENT 0x23u
#define WLWMI_TAG_INIT_CMD 0x4au
#define WLWMI_TAG_RESOURCE_CONFIG 0x4bu
#define WLWMI_TAG_SERVICE_READY_EXT_EVENT 0x1acu
#define WLWMI_TAG_PDEV_SET_HW_MODE_CMD 0x203u
#define WLWMI_TAG_MAC_PHY_CAPABILITIES 0x210u
#define WLWMI_TAG_HW_MODE_CAPABILITIES 0x211u
#define WLWMI_TAG_SOC_MAC_PHY_HW_MODE_CAPS 0x212u
#define WLWMI_TAG_SERVICE_AVAILABLE_EVENT 0x22fu
#define WLWMI_TAG_SERVICE_READY_EXT2_EVENT 0x334u
#define WLWMI_SVC_EXT_MSG 77u           /* wmi.h:2103 */
#define WLWMI_SVC_EXT2_MSG 220u         /* wmi.h:2244 */
#define WLWMI_SVC_REG_CC_EXT 281u       /* wmi.h:2252 */
#define WLWMI_SVC_READY_BYTES 128u      /* struct wmi_service_ready_event (wmi.h:2607-2637) */
#define WLWMI_SVC_BM_WORDS 32u          /* WMI_SERVICE_BM_SIZE, 4 services per word (wmi.h:2639-2644) */
#define WLWMI_SVC_AVAIL_BYTES 20u       /* struct wmi_service_available_event (wmi.h:2855-2858) */
#define WLWMI_READY_MIN_BYTES 52u       /* struct ath12k_wmi_ready_event_min_params (wmi.h:2839-2847) */
#define WLWMI_INIT_BYTES 32u            /* struct wmi_init_cmd (wmi.h:2512-2525) */
#define WLWMI_RSRC_BYTES 292u           /* struct ath12k_wmi_resource_config_params (wmi.h:2534-2605) */
#define WLWMI_HW_MODE_BYTES 16u         /* struct ath12k_wmi_pdev_set_hw_mode_cmd (wmi.h:2497-2502) */
#define WLWMI_INIT_TOTAL (4u + WLWMI_INIT_BYTES + WLWMI_RSRC_BYTES + 4u)
#define WLWMI_HW_MODE_TOTAL (4u + WLWMI_HW_MODE_BYTES)
#define WLWMI_HW_MODE_DBS 1u            /* wmi.h:192-194 */
#define WLWMI_PDEV_ID_SOC 0u            /* wmi.h:3163 */
#define WLWMI_FLAG1_BSS_CHANNEL_INFO_64 (1u << 5)       /* wmi.h:2530 */
#define WLWMI_FLAG1_ACK_RSSI (1u << 18)                 /* wmi.h:2532 */
#define WLWMI_FLAGS2_CALC_NEXT_DTIM_COUNT_SET (1u << 9) /* wmi.h:2531 */
#define WLWMI_HOST_SVC_REG_CC_EXT_BIT 4u                /* wmi.h:2527 */
#define WLWMI_EXT2_WORDS 8u             /* services 256..511 */
_Static_assert(WLWMI_INIT_BYTES == 32 && WLWMI_RSRC_BYTES == 292 && WLWMI_INIT_TOTAL == 332, "WMI INIT wire sizes");

/* struct ath12k_wmi_resource_config_params word index (0 = TLV header). */
enum {
	WLWMI_RC_NUM_VDEVS = 1, WLWMI_RC_NUM_PEERS, WLWMI_RC_NUM_OFFLOAD_PEERS, WLWMI_RC_NUM_OFFLOAD_REORDER_BUFFS,
	WLWMI_RC_NUM_PEER_KEYS, WLWMI_RC_NUM_TIDS, WLWMI_RC_AST_SKID_LIMIT, WLWMI_RC_TX_CHAIN_MASK,
	WLWMI_RC_RX_CHAIN_MASK, WLWMI_RC_RX_TIMEOUT_PRI, WLWMI_RC_RX_DECAP_MODE = WLWMI_RC_RX_TIMEOUT_PRI + 4,
	WLWMI_RC_SCAN_MAX_PENDING_REQ, WLWMI_RC_BMISS_OFFLOAD_MAX_VDEV, WLWMI_RC_ROAM_OFFLOAD_MAX_VDEV,
	WLWMI_RC_ROAM_OFFLOAD_MAX_AP_PROFILES, WLWMI_RC_NUM_MCAST_GROUPS, WLWMI_RC_NUM_MCAST_TABLE_ELEMS,
	WLWMI_RC_MCAST2UCAST_MODE, WLWMI_RC_TX_DBG_LOG_SIZE, WLWMI_RC_NUM_WDS_ENTRIES, WLWMI_RC_DMA_BURST_SIZE,
	WLWMI_RC_MAC_AGGR_DELIM, WLWMI_RC_RX_SKIP_DEFRAG, WLWMI_RC_VOW_CONFIG, WLWMI_RC_GTK_OFFLOAD_MAX_VDEV,
	WLWMI_RC_NUM_MSDU_DESC, WLWMI_RC_MAX_FRAG_ENTRIES, WLWMI_RC_NUM_TDLS_VDEVS, WLWMI_RC_NUM_TDLS_CONN_TABLE_ENTRIES,
	WLWMI_RC_BEACON_TX_OFFLOAD_MAX_VDEV, WLWMI_RC_NUM_MULTICAST_FILTER_ENTRIES, WLWMI_RC_NUM_WOW_FILTERS,
	WLWMI_RC_NUM_KEEP_ALIVE_PATTERN, WLWMI_RC_KEEP_ALIVE_PATTERN_SIZE, WLWMI_RC_MAX_TDLS_SLEEP_STA,
	WLWMI_RC_MAX_TDLS_BUFFER_STA, WLWMI_RC_WMI_SEND_SEPARATE, WLWMI_RC_NUM_OCB_VDEVS, WLWMI_RC_NUM_OCB_CHANNELS,
	WLWMI_RC_NUM_OCB_SCHEDULES, WLWMI_RC_FLAG1, WLWMI_RC_SMART_ANT_CAP, WLWMI_RC_BK_MINFREE, WLWMI_RC_BE_MINFREE,
	WLWMI_RC_VI_MINFREE, WLWMI_RC_VO_MINFREE, WLWMI_RC_ALLOC_FRAG_DESC, WLWMI_RC_NUM_NS_EXT_TUPLES,
	WLWMI_RC_BPF_INSTRUCTION_SIZE, WLWMI_RC_MAX_BSSID_RX_FILTERS, WLWMI_RC_USE_PDEV_ID, WLWMI_RC_MAX_DBS_SCAN_DUTY,
	WLWMI_RC_MAX_GROUP_KEYS, WLWMI_RC_PEER_MAP_UNMAP_VERSION, WLWMI_RC_SCHED_PARAMS, WLWMI_RC_TWT_AP_PDEV_COUNT,
	WLWMI_RC_TWT_AP_STA_COUNT, WLWMI_RC_MAX_NLO_SSIDS, WLWMI_RC_NUM_PKT_FILTERS, WLWMI_RC_NUM_MAX_STA_VDEVS,
	WLWMI_RC_MAX_BSSID_INDICATOR, WLWMI_RC_UL_RESP_CONFIG, WLWMI_RC_MSDU_FLOW_OVERRIDE0, WLWMI_RC_MSDU_FLOW_OVERRIDE1,
	WLWMI_RC_FLAGS2, WLWMI_RC_HOST_SERVICE_FLAGS, WLWMI_RC_MAX_RNR_NEIGHBOURS, WLWMI_RC_EMA_MAX_VAP_CNT,
	WLWMI_RC_EMA_MAX_PROFILE_PERIOD, WLWMI_RC_WORDS
};
_Static_assert(WLWMI_RC_WORDS * 4 == WLWMI_RSRC_BYTES && WLWMI_RC_FLAG1 == 44 && WLWMI_RC_USE_PDEV_ID == 54 &&
    WLWMI_RC_PEER_MAP_UNMAP_VERSION == 57 && WLWMI_RC_FLAGS2 == 68 && WLWMI_RC_HOST_SERVICE_FLAGS == 69,
    "WMI resource config word order (wmi.h:2534-2605)");

struct wlwmi {
	uint32_t svc_base[WLWMI_SVC_BM_WORDS], svc_seg[4], svc_ext2[WLWMI_EXT2_WORDS];
	bool got_svc, got_avail, got_ext, got_ext2, got_ready, got_hw_resp;
	uint32_t fw_build, phy_capability, num_rf_chains, num_mem_reqs, max_scan_chans, txrx_chainmask;
	uint32_t num_hw_modes, hw_mode_caps, tot_phy, mac_phy_caps, pdev_id;
	bool pdev_valid;
	uint32_t ready_status, num_extra_mac, num_total_peers;
	uint8_t mac[6];
	uint32_t ext2_bytes, events, unknown_events, last_event, last_len;
	const char *why;
};

static inline int
wlwmi_fail(struct wlwmi *wmi, const char *why)
{
	if (!wmi->why) { wmi->why = why; }
	return 1;
}

struct wlwmi_it {
	const uint8_t *p, *v;
	uint32_t len, off, tag, tlen;
};

static inline void wlwmi_it_init(struct wlwmi_it *it, const uint8_t *p, uint32_t len) { it->p = p; it->len = len; it->off = 0; it->v = 0; it->tag = it->tlen = 0; }

/* Strict iterator (wmi.c:320-368): 1 = one TLV in it->tag/tlen/v, 0 = end,
 * -1 = a header or a value past the buffer. */
static inline int
wlwmi_next(struct wlwmi_it *it)
{
	if (it->off >= it->len) { return 0; }
	if (it->len - it->off < 4) { return -1; }
	uint32_t h = wlce_get32(it->p + it->off);
	it->tag = h >> 16; it->tlen = h & 0xffffu; it->off += 4;
	if (it->tlen > it->len - it->off) { return -1; }
	it->v = it->p + it->off;
	it->off += it->tlen;
	return 1;
}

static inline void wlwmi_put_tlv(uint8_t *p, uint32_t tag, uint32_t len) { wlce_put32(p, (tag << 16) | (len & 0xffffu)); }

static inline bool
wlwmi_svc(const struct wlwmi *wmi, uint32_t bit)
{
	if (bit < 128u) { return (wmi->svc_base[bit / 4u] >> (bit % 4u)) & 1u; }
	if (bit < 256u) { return (wmi->svc_seg[(bit - 128u) / 32u] >> ((bit - 128u) % 32u)) & 1u; }
	if (bit < 256u + 32u * WLWMI_EXT2_WORDS) { return (wmi->svc_ext2[(bit - 256u) / 32u] >> ((bit - 256u) % 32u)) & 1u; }
	return false;
}

/* SERVICE_READY (wmi.c:662-760): fixed TLV 0x20 (min length = the struct,
 * wmi.c:132-133), the first ARRAY_UINT32 of at least 32 words is the base
 * bitmap, 4 services per word (wmi.c:695-737). */
static inline int
wlwmi_parse_service_ready(struct wlwmi *wmi, const uint8_t *p, uint32_t len)
{
	struct wlwmi_it it; int r; bool fixed = false, bm = false;
	wlwmi_it_init(&it, p, len);
	while ((r = wlwmi_next(&it)) > 0) {
		if (it.tag == WLWMI_TAG_SERVICE_READY_EVENT) {
			if (it.tlen < WLWMI_SVC_READY_BYTES) { return wlwmi_fail(wmi, "WMI SERVICE_READY fixed TLV shorter than 128 bytes"); }
			wmi->fw_build = wlce_get32(it.v);
			wmi->phy_capability = wlce_get32(it.v + 7 * 4);
			wmi->num_rf_chains = wlce_get32(it.v + 9 * 4);
			wmi->num_mem_reqs = wlce_get32(it.v + 18 * 4);
			wmi->max_scan_chans = wlce_get32(it.v + 19 * 4);
			wmi->txrx_chainmask = wlce_get32(it.v + 29 * 4);
			fixed = true;
		} else if (it.tag == WLWMI_TAG_ARRAY_UINT32 && !bm) {
			if (it.tlen < WLWMI_SVC_BM_WORDS * 4u) { return wlwmi_fail(wmi, "WMI service bitmap shorter than 32 words"); }
			for (unsigned i = 0; i < WLWMI_SVC_BM_WORDS; i++) { wmi->svc_base[i] = wlce_get32(it.v + 4 * i) & 0xfu; }
			bm = true;
		}
	}
	if (r < 0) { return wlwmi_fail(wmi, "WMI SERVICE_READY TLV past the buffer"); }
	if (!fixed) { return wlwmi_fail(wmi, "WMI SERVICE_READY without its fixed TLV"); }
	wmi->got_svc = true;
	return 0;
}

/* SERVICE_AVAILABLE (wmi.c:7677-7744): every TLV at least 16 bytes as in
 * L19; fixed TLV = segment offset + 4 words for services 128..255 (32 per
 * word); an ARRAY_UINT32 carries services 256.. . */
static inline int
wlwmi_parse_available(struct wlwmi *wmi, const uint8_t *p, uint32_t len)
{
	struct wlwmi_it it; int r;
	wlwmi_it_init(&it, p, len);
	while ((r = wlwmi_next(&it)) > 0) {
		if (it.tlen < 16) { return wlwmi_fail(wmi, "WMI SERVICE_AVAILABLE TLV shorter than 16 bytes"); }
		if (it.tag == WLWMI_TAG_SERVICE_AVAILABLE_EVENT) {
			if (it.tlen < WLWMI_SVC_AVAIL_BYTES) { return wlwmi_fail(wmi, "WMI SERVICE_AVAILABLE fixed TLV shorter than 20 bytes"); }
			for (unsigned i = 0; i < 4; i++) { wmi->svc_seg[i] = wlce_get32(it.v + 4 + 4 * i); }
		} else if (it.tag == WLWMI_TAG_ARRAY_UINT32) {
			for (unsigned i = 0; i < WLWMI_EXT2_WORDS && 4 * i + 4 <= it.tlen; i++) { wmi->svc_ext2[i] = wlce_get32(it.v + 4 * i); }
		}
	}
	if (r < 0) { return wlwmi_fail(wmi, "WMI SERVICE_AVAILABLE TLV past the buffer"); }
	wmi->got_avail = true;
	return 0;
}

static inline uint32_t
wlwmi_fls(uint32_t v)
{
	uint32_t n = 0;
	while (v) { n++; v >>= 1; }
	return n;
}

/* SERVICE_READY_EXT (wmi.c:4847-4949): fixed TLV 0x1ac; 0x212 num_hw_modes
 * (8 bytes min, wmi.c:136-137); the first ARRAY_STRUCT holds only 0x211
 * entries, at most num_hw_modes (wmi.c:4494-4516), each adding fls(phy map)
 * PHYs; the second holds only 0x210 MAC/PHY caps, at most that many PHYs;
 * pdev id = 15:0 of word 1 of the first (wmi.h:2681-2691). Later arrays are
 * not needed in C1. Nested TLVs are bounded by their array. */
static inline int
wlwmi_parse_ext(struct wlwmi *wmi, const uint8_t *p, uint32_t len)
{
	struct wlwmi_it it, in; int r, q; bool fixed = false, caps = false; unsigned arrays = 0;
	wlwmi_it_init(&it, p, len);
	while ((r = wlwmi_next(&it)) > 0) {
		if (it.tag == WLWMI_TAG_SERVICE_READY_EXT_EVENT) { fixed = true; continue; }
		if (it.tag == WLWMI_TAG_SOC_MAC_PHY_HW_MODE_CAPS) {
			if (it.tlen < 8) { return wlwmi_fail(wmi, "WMI SOC_MAC_PHY_HW_MODE_CAPS shorter than 8 bytes"); }
			wmi->num_hw_modes = wlce_get32(it.v);
			caps = true;
			continue;
		}
		if (it.tag != WLWMI_TAG_ARRAY_STRUCT || ++arrays > 2) { continue; }
		wlwmi_it_init(&in, it.v, it.tlen);
		while ((q = wlwmi_next(&in)) > 0) {
			if (arrays == 1) {
				if (!caps || in.tag != WLWMI_TAG_HW_MODE_CAPABILITIES || wmi->hw_mode_caps >= wmi->num_hw_modes || in.tlen < 12) {
					return wlwmi_fail(wmi, "WMI EXT hw mode caps: tag, count or length");
				}
				wmi->hw_mode_caps++;
				wmi->tot_phy += wlwmi_fls(wlce_get32(in.v + 4));
			} else {
				if (in.tag != WLWMI_TAG_MAC_PHY_CAPABILITIES || wmi->mac_phy_caps >= wmi->tot_phy || in.tlen < 8) {
					return wlwmi_fail(wmi, "WMI EXT mac/phy caps: tag, count or length");
				}
				if (!wmi->mac_phy_caps++) { wmi->pdev_id = wlce_get32(in.v + 4) & 0xffffu; wmi->pdev_valid = true; }
			}
		}
		if (q < 0) { return wlwmi_fail(wmi, "WMI EXT nested TLV past its array"); }
	}
	if (r < 0) { return wlwmi_fail(wmi, "WMI SERVICE_READY_EXT TLV past the buffer"); }
	if (!fixed) { return wlwmi_fail(wmi, "WMI SERVICE_READY_EXT without its fixed TLV"); }
	wmi->got_ext = true;
	return 0;
}

/* SERVICE_READY_EXT2 (wmi.c:5560-5650): only well-formedness and the fixed
 * TLV 0x334 matter here; its arrays serve the data path. */
static inline int
wlwmi_parse_ext2(struct wlwmi *wmi, const uint8_t *p, uint32_t len)
{
	struct wlwmi_it it; int r; bool fixed = false;
	wlwmi_it_init(&it, p, len);
	while ((r = wlwmi_next(&it)) > 0) {
		if (it.tag == WLWMI_TAG_SERVICE_READY_EXT2_EVENT) { fixed = true; }
	}
	if (r < 0) { return wlwmi_fail(wmi, "WMI SERVICE_READY_EXT2 TLV past the buffer"); }
	if (!fixed) { return wlwmi_fail(wmi, "WMI SERVICE_READY_EXT2 without its fixed TLV"); }
	wmi->ext2_bytes = len;
	wmi->got_ext2 = true;
	return 0;
}

/* READY (wmi.c:6895-6953): fixed TLV 0x23, at least 52 bytes (wmi.c:166-167):
 * fw abi 0..23, MAC 24..29, status 32, num_dscp_table 36, num_extra_mac_addr
 * 40, num_total_peers 44 (wmi.h:2839-2847). L19 completes unified_ready even
 * without the TLV; here it is required. */
static inline int
wlwmi_parse_ready(struct wlwmi *wmi, const uint8_t *p, uint32_t len)
{
	struct wlwmi_it it; int r; bool fixed = false;
	wlwmi_it_init(&it, p, len);
	while ((r = wlwmi_next(&it)) > 0) {
		if (it.tag != WLWMI_TAG_READY_EVENT || fixed) { continue; }
		if (it.tlen < WLWMI_READY_MIN_BYTES) { return wlwmi_fail(wmi, "WMI READY fixed TLV shorter than 52 bytes"); }
		memcpy(wmi->mac, it.v + 24, 6);
		wmi->ready_status = wlce_get32(it.v + 32);
		wmi->num_extra_mac = wlce_get32(it.v + 40);
		wmi->num_total_peers = wlce_get32(it.v + 44);
		fixed = true;
	}
	if (r < 0) { return wlwmi_fail(wmi, "WMI READY TLV past the buffer"); }
	if (!fixed) { return wlwmi_fail(wmi, "WMI READY without its fixed TLV"); }
	wmi->got_ready = true;
	return 0;
}

/* Events handled in C1; everything else is counted and logged by id/length,
 * never parsed and never fatal (L19 ignores unknown ids, wmi.c:9953-9955). */
static inline bool
wlwmi_known(uint32_t id)
{
	return id == WLWMI_EV_SERVICE_READY || id == WLWMI_EV_SERVICE_AVAILABLE || id == WLWMI_EV_SERVICE_READY_EXT ||
	    id == WLWMI_EV_SERVICE_READY_EXT2 || id == WLWMI_EV_READY;
}

/* One WMI event (wmi.c:9798-9960). */
static inline int
wlwmi_rx(struct wlwmi *wmi, const uint8_t *msg, uint32_t len, uint32_t *id)
{
	*id = 0;
	if (len < 4) { return wlwmi_fail(wmi, "WMI event shorter than its header"); }
	*id = wlce_get32(msg) & 0xffffffu;
	wmi->events++; wmi->last_event = *id; wmi->last_len = len - 4;
	const uint8_t *p = msg + 4;
	switch (*id) {
	case WLWMI_EV_SERVICE_READY: return wlwmi_parse_service_ready(wmi, p, len - 4);
	case WLWMI_EV_SERVICE_AVAILABLE: return wlwmi_parse_available(wmi, p, len - 4);
	case WLWMI_EV_SERVICE_READY_EXT: return wlwmi_parse_ext(wmi, p, len - 4);
	case WLWMI_EV_SERVICE_READY_EXT2: return wlwmi_parse_ext2(wmi, p, len - 4);
	case WLWMI_EV_READY: return wlwmi_parse_ready(wmi, p, len - 4);
	case WLWMI_EV_PDEV_SET_HW_MODE_RESP: wmi->got_hw_resp = true; return 0;
	default: wmi->unknown_events++; return 0;
	}
}

static inline uint32_t
wlwmi_chain_mask(uint32_t chains)
{
	return chains >= 32u ? ~0u : (1u << chains) - 1u;
}

/* WMI_INIT for WCN7850 (wmi.c:4018-4124, 4218-4243): command id 1; TLV 0x4a
 * {host abi 6 x 0, num_host_mem_chunks 0}; TLV 0x4b resource config with the
 * ath12k_wmi_init_wcn7850 values (wmi.c:260-304, hw.h:33-65) as
 * ath12k_wmi_copy_resource_config puts them on the wire (wmi.c:3941-4016:
 * rx_batchmode is not copied; vow_config, atf, twt and ema are copied but
 * are 0 for WCN7850, wmi.c:3974, 3999-4005, 4013-4014, hw.h:58); then
 * ARRAY_STRUCT of length 0 (no host memory chunks, wmi.c:4080-4083). No
 * hw-mode TLV: a single-pdev chip sends hw_mode_id MAX (wmi.c:4034, 4234-4235).
 * flag1 = BSS_CHANNEL_INFO_64 | ACK_RSSI; flags2 = CALC_NEXT_DTIM_COUNT_SET
 * with RX_PEER_METADATA_VERSION 0; host_service_flags REG_CC_EXT when service
 * 281 is set (wmi.c:4223-4225, 4008-4009); no REO_QREF (reoq_lut_support
 * false, hw.c:1450). */
static inline int
wlwmi_encode_init(const struct wlwmi *wmi, uint8_t *out, uint32_t room, uint32_t *len)
{
	static const uint8_t fixed[][2] = {
		{WLWMI_RC_NUM_VDEVS, 4}, {WLWMI_RC_NUM_PEERS, 16}, {WLWMI_RC_NUM_OFFLOAD_PEERS, 3},
		{WLWMI_RC_NUM_OFFLOAD_REORDER_BUFFS, 3}, {WLWMI_RC_NUM_PEER_KEYS, 2}, {WLWMI_RC_NUM_TIDS, 32},
		{WLWMI_RC_AST_SKID_LIMIT, 16}, {WLWMI_RC_RX_TIMEOUT_PRI, 100}, {WLWMI_RC_RX_TIMEOUT_PRI + 1, 100},
		{WLWMI_RC_RX_TIMEOUT_PRI + 2, 100}, {WLWMI_RC_RX_TIMEOUT_PRI + 3, 40}, {WLWMI_RC_RX_DECAP_MODE, 1},
		{WLWMI_RC_SCAN_MAX_PENDING_REQ, 4}, {WLWMI_RC_BMISS_OFFLOAD_MAX_VDEV, 3},
		{WLWMI_RC_ROAM_OFFLOAD_MAX_VDEV, 3}, {WLWMI_RC_ROAM_OFFLOAD_MAX_AP_PROFILES, 8},
		{WLWMI_RC_GTK_OFFLOAD_MAX_VDEV, 2}, {WLWMI_RC_MAX_FRAG_ENTRIES, 0xa}, {WLWMI_RC_NUM_TDLS_VDEVS, 1},
		{WLWMI_RC_NUM_TDLS_CONN_TABLE_ENTRIES, 8}, {WLWMI_RC_BEACON_TX_OFFLOAD_MAX_VDEV, 2},
		{WLWMI_RC_NUM_MULTICAST_FILTER_ENTRIES, 0x20}, {WLWMI_RC_NUM_WOW_FILTERS, 0x16},
		{WLWMI_RC_USE_PDEV_ID, 1}, {WLWMI_RC_PEER_MAP_UNMAP_VERSION, 1},
	};
	if (room < WLWMI_INIT_TOTAL) { return 1; }
	for (uint32_t i = 0; i < WLWMI_INIT_TOTAL; i++) { out[i] = 0; }
	uint8_t *c = out + 4, *rc = c + WLWMI_INIT_BYTES;
	uint32_t mask = wlwmi_chain_mask(wmi->num_rf_chains);
	wlce_put32(out, WLWMI_CMD_INIT);
	wlwmi_put_tlv(c, WLWMI_TAG_INIT_CMD, WLWMI_INIT_BYTES - 4u);
	wlwmi_put_tlv(rc, WLWMI_TAG_RESOURCE_CONFIG, WLWMI_RSRC_BYTES - 4u);
	for (unsigned i = 0; i < sizeof(fixed) / sizeof(fixed[0]); i++) { wlce_put32(rc + 4u * fixed[i][0], fixed[i][1]); }
	wlce_put32(rc + 4u * WLWMI_RC_TX_CHAIN_MASK, mask);
	wlce_put32(rc + 4u * WLWMI_RC_RX_CHAIN_MASK, mask);
	wlce_put32(rc + 4u * WLWMI_RC_TX_DBG_LOG_SIZE, 1024);
	wlce_put32(rc + 4u * WLWMI_RC_NUM_MSDU_DESC, 0x400);
	wlce_put32(rc + 4u * WLWMI_RC_FLAG1, WLWMI_FLAG1_BSS_CHANNEL_INFO_64 | WLWMI_FLAG1_ACK_RSSI);
	wlce_put32(rc + 4u * WLWMI_RC_FLAGS2, WLWMI_FLAGS2_CALC_NEXT_DTIM_COUNT_SET);
	wlce_put32(rc + 4u * WLWMI_RC_HOST_SERVICE_FLAGS,
	    wlwmi_svc(wmi, WLWMI_SVC_REG_CC_EXT) ? 1u << WLWMI_HOST_SVC_REG_CC_EXT_BIT : 0u);
	wlwmi_put_tlv(rc + WLWMI_RSRC_BYTES, WLWMI_TAG_ARRAY_STRUCT, 0);
	*len = WLWMI_INIT_TOTAL;
	return 0;
}

/* PDEV_SET_HW_MODE (wmi.c:4186-4216): TLV 0x203 {pdev_id SOC 0, hw mode
 * index, num_band_to_mac 0}; core.c:946-953 asks DBS for single-pdev chips. */
static inline int
wlwmi_encode_hw_mode(uint8_t *out, uint32_t room, uint32_t mode, uint32_t *len)
{
	if (room < WLWMI_HW_MODE_TOTAL) { return 1; }
	wlce_put32(out, WLWMI_CMD_PDEV_SET_HW_MODE);
	wlwmi_put_tlv(out + 4, WLWMI_TAG_PDEV_SET_HW_MODE_CMD, WLWMI_HW_MODE_BYTES - 4u);
	wlce_put32(out + 8, WLWMI_PDEV_ID_SOC);
	wlce_put32(out + 12, mode);
	wlce_put32(out + 16, 0);
	*len = WLWMI_HW_MODE_TOTAL;
	return 0;
}

#endif /* FUXI_WLAN_CE_H */
