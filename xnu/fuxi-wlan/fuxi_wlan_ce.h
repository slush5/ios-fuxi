/*
 * FUXI WLAN W20/W21/W22 (fuxiwlanqrtr=6, C1): WCN7850 copy engines on SRNG
 * rings, HTC and WMI init to WMI READY, polled. W23/W24 (fuxiwlanqrtr=7, C2):
 * on top of that the passive channel list, a station vdev, one passive scan,
 * MGMT_RX parsing and a BSS table; nothing is transmitted on the air. C2b:
 * the CHAN_INFO words and the pdev stats answer after the scan. C2c (W26,
 * w20 0x100/0x200/0x400): host RX rings, the WBM idle link-descriptor list
 * and the pdev chain masks, with the watch, content and guard counts. W25
 * (fuxiwlanqrtr=8, C3): the same scan as a service for the iOS side, passive
 * scans of a request's channels (wlwmi_encode_start_scan_list), events of
 * other scan ids ignored, the MGMT_RX frames handed on (wlscan.heard).
 * Pure C: no XNU or Linux includes. The includer provides uint8_t..uint64_t,
 * size_t, bool and memcpy (bsd/dev/fuxi_wlan.c in the kernel,
 * tools/fuxi-wlan-ce-model.c on the host).
 * Hardware is reached only through struct wlce_io; every length that comes
 * from the device is bounded by the buffer it arrived in; nothing allocates.
 * Per-field provenance and the log lines: docs-lanes/wlan-w20.md.
 *
 * Derived from Linux ath12k v6.19 (tag v6.19, 05f7e89ab), directory
 * drivers/net/wireless/ath/ath12k, files and their notices:
 *   ce.c, ce.h, hal_desc.h:
 *     Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) 2021-2022, 2024-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 *   hal.c, hal.h, hw.c, hw.h, dp.c, dp.h, dp_tx.c, reg.c:
 *     Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) 2021-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 *   hal_rx.c:
 *     Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *   dp_mon.c:
 *     Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *   htc.c:
 *     Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) 2021-2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *   htc.h:
 *     Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 *     Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *   wmi.c, wmi.h, core.c, core.h, mac.c, dp_rx.c, debugfs.c:
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
 * slots of CE0/CE3/CE4 (slot i = SRC entry i mod slots) and the RX buffers of
 * CE1/CE2. Ring bases are 4 KiB aligned (L19 needs 8, CE_DESC_RING_ALIGN
 * ce.h:108). CE4 (HTT, src_sz_max 256, ce.c:145-220) gets 16 slots for the
 * one HTT message C2 may send (w20 0x02); C1 never sends on it. wlce_send
 * refuses a send while all slots of its pipe are in flight. */
#define WLCE_CE4_BUF 256u
#define WLCE_CE4_SLOTS 16u
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
#define WLCE_OFF_CE4_TX 0x66000u    /* 16 x 256 B */
#define WLCE_OFF_END 0x67000u
_Static_assert(WLCE_RDP_IDS * 4u <= WLCE_OFF_CE0_SRC && WLCE_OFF_CE0_SRC + 16u * 16u <= WLCE_OFF_CE1_DST &&
    WLCE_OFF_CE1_DST + 512u * 8u <= WLCE_OFF_CE1_STS && WLCE_OFF_CE1_STS + 512u * 16u <= WLCE_OFF_CE2_DST &&
    WLCE_OFF_CE2_DST + 64u * 8u <= WLCE_OFF_CE2_STS && WLCE_OFF_CE2_STS + 64u * 16u <= WLCE_OFF_CE3_SRC &&
    WLCE_OFF_CE3_SRC + 32u * 16u <= WLCE_OFF_CE4_SRC && WLCE_OFF_CE4_SRC + 2048u * 16u <= WLCE_OFF_CE0_TX &&
    WLCE_OFF_CE0_TX + 16u * WLCE_BUF <= WLCE_OFF_CE3_TX && WLCE_OFF_CE3_TX + 32u * WLCE_BUF <= WLCE_OFF_CE1_RX &&
    WLCE_OFF_CE1_RX + WLCE_RX_POST * WLCE_BUF <= WLCE_OFF_CE2_RX &&
    WLCE_OFF_CE2_RX + WLCE_RX_POST * WLCE_BUF == WLCE_OFF_CE4_TX &&
    WLCE_OFF_CE4_TX + WLCE_CE4_SLOTS * WLCE_CE4_BUF == WLCE_OFF_END && WLCE_OFF_END <= WLCE_ARENA_BYTES,
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
	static const struct { uint8_t ce, kind; uint32_t entries, ring, buf, nbuf, bsz; } plan[WLCE_NRING] = {
		{0, WLCE_SRC, 16, WLCE_OFF_CE0_SRC, WLCE_OFF_CE0_TX, 16, WLCE_BUF},
		{1, WLCE_DST, 512, WLCE_OFF_CE1_DST, WLCE_OFF_CE1_RX, WLCE_RX_POST, WLCE_BUF},
		{1, WLCE_STS, 512, WLCE_OFF_CE1_STS, 0, 0, 0},
		{2, WLCE_DST, 64, WLCE_OFF_CE2_DST, WLCE_OFF_CE2_RX, WLCE_RX_POST, WLCE_BUF},
		{2, WLCE_STS, 64, WLCE_OFF_CE2_STS, 0, 0, 0},
		{3, WLCE_SRC, 32, WLCE_OFF_CE3_SRC, WLCE_OFF_CE3_TX, 32, WLCE_BUF},
		{4, WLCE_SRC, 2048, WLCE_OFF_CE4_SRC, WLCE_OFF_CE4_TX, WLCE_CE4_SLOTS, WLCE_CE4_BUF},
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
		if (plan[i].nbuf) { p->slot0 = plan[i].buf; p->nslots = plan[i].nbuf; p->buf_bytes = plan[i].bsz; }
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
 * hardware. A free entry needs next HP != device TP (hal.c:2077-2080) and a
 * free slot: CE4 has 16 slots on 2048 entries (CE0/CE3 have a slot per
 * entry, so the ring bound comes first there). */
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
	if (p->posted - p->reaped >= p->nslots) { return wlce_fail(ce, "CE TX slots all in flight"); }
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
#define WLHTC_CREDITS_MOST 0xffff       /* C3: an endpoint never holds more (READY's count is a u16) */
_Static_assert(WLHTC_READY_BYTES == 8 && WLHTC_CONNECT_RESP_BYTES == 12, "HTC wire sizes");

struct wlhtc_ep {
	uint16_t svc, max_msg;
	uint8_t ul_pipe, dl_pipe, seq;
	bool used, flow;
	int credits;
};

typedef int (*wlhtc_msg_fn)(void *ctx, const uint8_t *msg, uint32_t len);

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
	wlhtc_msg_fn htt_fn;            /* C2 w20 0x02 only: HTT messages; NULL = counted only */
	void *htt_ctx;
};

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
 * that is not a connected credit-flow endpoint are counted and dropped. An
 * endpoint's credits stop at WLHTC_CREDITS_MOST (C3: a service runs for 30
 * minutes, and a target that over-reports must not wrap the int; L19 adds
 * without a bound, htc.c:165). */
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
				if (htc->ep[e].used && htc->ep[e].flow) {
					htc->ep[e].credits = htc->ep[e].credits > WLHTC_CREDITS_MOST - (int)c ? WLHTC_CREDITS_MOST :
					    htc->ep[e].credits + (int)c;
				} else { htc->bad_reports++; }
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
 * WMI messages go to FN; HTT messages are counted and, with a handler set
 * (C2 w20 0x02, the version reply), handed to it. */
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
	return htc->htt_fn ? htc->htt_fn(htc->htt_ctx, pl, plen) : 0;
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

struct wlscan;                      /* C2 scan state, below */
struct wlstats;                     /* C2b pdev stats answer, below */

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
	/* C2: bands of the preferred hw mode's PHYs, chain masks of the last of
	 * them (wmi.c:525-553, 4663-4686), and the scan the SCAN/CHAN_INFO/MGMT_RX
	 * events feed (NULL before the scan stage and in C1: those ids stay
	 * unknown events). */
	uint32_t phy_bands, phy_tx_mask, phy_rx_mask;
	struct wlscan *scan;
	/* C2b: the distinct pdev ids of the preferred hw mode's PHYs in PHY
	 * order, at most two, each with the supported_bands of its PHYs ORed
	 * (L19's fw_pdev[], one per PHY, wmi.c:528-532, 4660-4686: [0] is what
	 * ath12k_mac_get_target_pdev_id returns with no vif up, the other the
	 * one it picks by band once a vif is up there, mac.c:967-1009), and the
	 * stats answer the UPDATE_STATS events feed (NULL until a request goes
	 * out: the id stays an unknown event). */
	uint32_t phy_pdev[2], phy_pdev_bands[2], phy_pdevs;
	struct wlstats *stats;
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

/* L19's hw mode priority, indexed by WMI_HOST_HW_MODE_* (wmi.c:306-318,
 * wmi.h:185-217): DBS_SBS 0, DBS 1, DBS_OR_SBS 2, SBS 3, SBS_PASSIVE 4,
 * SINGLE 5; ids from 6 up are skipped (wmi.c:4542-4543). */
#define WLWMI_HW_MODE_PRI_SINGLE 5u
#define WLWMI_HW_MODE_PRI_NONE 6u
static inline uint32_t
wlwmi_hw_mode_pri(uint32_t mode)
{
	static const uint8_t pri[] = { 5, 1, 4, 3, 0, 2 };
	return mode < sizeof(pri) ? pri[mode] : WLWMI_HW_MODE_PRI_NONE;
}

/* SERVICE_READY_EXT (wmi.c:4847-4949): fixed TLV 0x1ac; 0x212 num_hw_modes
 * (8 bytes min, wmi.c:136-137); the first ARRAY_STRUCT holds only 0x211
 * entries, at most num_hw_modes (wmi.c:4494-4516), each adding fls(phy map)
 * PHYs; the second holds only 0x210 MAC/PHY caps, at most that many PHYs;
 * pdev id = 15:0 of word 1 of the first (wmi.h:2681-2691). Later arrays are
 * not needed in C1. Nested TLVs are bounded by their array.
 * C2 also keeps the bands and chain masks of L19's single pdev: the
 * preferred hw mode starts as SINGLE with no PHY for a single-pdev chip
 * (wmi.c:10207-10211) and a listed mode of strictly better priority takes
 * its place (wmi.c:4538-4551); its PHYs are the MAC/PHY caps from index
 * fls(phy map of the 0x211 entry listed just before it), 0 when it is
 * first (L19 assigns that index, it does not sum: wmi.c:505-517), one per
 * bit up to fls(its own map) (wmi.c:4663-4686). Of those, with at least 22
 * words: supported_bands (word 4) ORed, and the 2 GHz (12/13) or else
 * 5 GHz (20/21) tx/rx chain masks of the last one (wmi.c:525-553).
 * C2b also keeps the pdev id (15:0 of word 1, wmi.h:2686) of each of those
 * PHYs, distinct ids only, at most two, with its supported_bands (word 4,
 * when the entry has it): L19's fw_pdev[] in order, [0] being what
 * ath12k_mac_get_target_pdev_id returns on a single-pdev chip with no vif
 * up (wmi.c:528-532, mac.c:967-1009). No such PHY (nphy 0): none kept,
 * L19's fw_pdev[0] stays zeroed. */
static inline int
wlwmi_parse_ext(struct wlwmi *wmi, const uint8_t *p, uint32_t len)
{
	struct wlwmi_it it, in; int r, q; bool fixed = false, caps = false; unsigned arrays = 0;
	uint32_t pri = WLWMI_HW_MODE_PRI_SINGLE, first = 0, nphy = 0, prev_map = 0, nmode = 0, kphy = 0;
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
				uint32_t mode = wlce_get32(in.v), map = wlce_get32(in.v + 4);
				if (wlwmi_hw_mode_pri(mode) < pri) { pri = wlwmi_hw_mode_pri(mode); first = nmode ? wlwmi_fls(prev_map) : 0u; nphy = wlwmi_fls(map); }
				prev_map = map; nmode++;
				wmi->hw_mode_caps++;
				wmi->tot_phy += wlwmi_fls(map);
			} else {
				if (in.tag != WLWMI_TAG_MAC_PHY_CAPABILITIES || wmi->mac_phy_caps >= wmi->tot_phy || in.tlen < 8) {
					return wlwmi_fail(wmi, "WMI EXT mac/phy caps: tag, count or length");
				}
				uint32_t k = kphy++;
				if (!wmi->mac_phy_caps++) { wmi->pdev_id = wlce_get32(in.v + 4) & 0xffffu; wmi->pdev_valid = true; }
				if (k >= first && k - first < nphy) {
					uint32_t id = wlce_get32(in.v + 4) & 0xffffu, j = 0;
					while (j < wmi->phy_pdevs && wmi->phy_pdev[j] != id) { j++; }
					if (j < 2u) {
						if (j == wmi->phy_pdevs) { wmi->phy_pdev[j] = id; wmi->phy_pdevs++; }
						wmi->phy_pdev_bands[j] |= in.tlen >= 5u * 4u ? wlce_get32(in.v + 4 * 4) : 0u;
					}
				}
				if (in.tlen >= 22u * 4u && k >= first && k - first < nphy) {  /* 1/2 = WMI_HOST_WLAN_2GHZ_CAP/5GHZ_CAP (wmi.h:220-221) */
					uint32_t bands = wlce_get32(in.v + 4 * 4), m = bands & 1u ? 12u : bands & 2u ? 20u : 0u;
					wmi->phy_bands |= bands;
					if (m) { wmi->phy_tx_mask = wlce_get32(in.v + 4 * m); wmi->phy_rx_mask = wlce_get32(in.v + 4 * (m + 1)); }
				}
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

static inline bool wlscan_known(uint32_t id);
static inline int wlscan_rx(struct wlwmi *wmi, uint32_t id, const uint8_t *p, uint32_t len);
static inline bool wlstats_known(uint32_t id);
static inline void wlstats_parse(struct wlstats *st, const uint8_t *p, uint32_t len);

/* One WMI event (wmi.c:9798-9960). In the C2 scan stage SCAN, CHAN_INFO and
 * MGMT_RX go to the scan parser (wmi.c:9838-9856); once C2b asked for pdev
 * stats, UPDATE_STATS goes to the stats parser, which never fails
 * (wmi.c:9869-9870). */
static inline int
wlwmi_rx(struct wlwmi *wmi, const uint8_t *msg, uint32_t len, uint32_t *id)
{
	*id = 0;
	if (len < 4) { return wlwmi_fail(wmi, "WMI event shorter than its header"); }
	*id = wlce_get32(msg) & 0xffffffu;
	wmi->events++; wmi->last_event = *id; wmi->last_len = len - 4;
	const uint8_t *p = msg + 4;
	if (wmi->scan && wlscan_known(*id)) { return wlscan_rx(wmi, *id, p, len - 4); }
	if (wmi->stats && wlstats_known(*id)) { wlstats_parse(wmi->stats, p, len - 4); return 0; }
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

/* ---- HTT version request (C2, w20 0x02 only) ----
 * Adapted from Linux ath12k dp_tx.c, dp_rx.c and dp.h (BSD-3-Clause-Clear),
 * v6.19. H2T VERSION_REQ: message 0 in 7:0 of word 0, then the TCL metadata
 * version option {tag 5 in 7:0, length 4 in 15:8, value 2 in 31:16}
 * (dp_tx.c:1277-1305, dp.h:433, 471, 481-491; version 2 outside FTM mode).
 * T2H VERSION_CONF: message 0 in 7:0, minor 15:8, major 23:16; L19 waits 3 s
 * and accepts major 3 only (dp_tx.c:1275, 1312-1323; dp_rx.c:1931-1942;
 * dp.h:1338-1359). It travels on the HTT endpoint (CE4 up, CE1 down). */
#define WLHTT_H2T_VERSION_REQ 0u
#define WLHTT_T2H_VERSION_CONF 0u
#define WLHTT_TAG_TCL_METADATA_VERSION 5u
#define WLHTT_TCL_METADATA_VER_SZ 4u
#define WLHTT_TCL_METADATA_VER_V2 2u
#define WLHTT_VERSION_MAJOR 3u
#define WLHTT_VER_REQ_BYTES 8u          /* struct htt_ver_req_cmd (dp.h:488-491) */

static inline void
wlhtt_version_req(uint8_t *p)
{
	wlce_put32(p, WLHTT_H2T_VERSION_REQ);
	wlce_put32(p + 4, WLHTT_TAG_TCL_METADATA_VERSION | (WLHTT_TCL_METADATA_VER_SZ << 8) | (WLHTT_TCL_METADATA_VER_V2 << 16));
}

/* 1 = a VERSION_CONF (MAJOR/MINOR filled), 0 = any other HTT message. */
static inline int
wlhtt_version_conf(const uint8_t *msg, uint32_t len, uint32_t *major, uint32_t *minor)
{
	if (len < 4) { return 0; }
	uint32_t w = wlce_get32(msg);
	if ((w & 0xffu) != WLHTT_T2H_VERSION_CONF) { return 0; }
	*minor = (w >> 8) & 0xffu;
	*major = (w >> 16) & 0xffu;
	return 1;
}

/* ---- W23/W24 (fuxiwlanqrtr=7, C2): channel list, station vdev, passive scan ----
 * Adapted from Linux ath12k wmi.c, wmi.h, mac.c, reg.c and core.h
 * (BSD-3-Clause-Clear), v6.19. Ids, tags, sizes and flags come from the
 * wmi.h enums and structs (the host test re-derives them). Nothing here asks
 * the radio to transmit: every channel carries PASSIVE, the scan is PASSIVE
 * with no SSID, no extra IE and no probe count, and the one BSSID is the
 * broadcast placeholder L19 always sends (wmi.c:2565-2570). */
#define WLWMI_CMD_START_SCAN 0x3001u
#define WLWMI_CMD_STOP_SCAN 0x3002u
#define WLWMI_CMD_SCAN_CHAN_LIST 0x3003u
#define WLWMI_CMD_VDEV_CREATE 0x5001u
#define WLWMI_CMD_VDEV_SET_PARAM 0x5008u
#define WLWMI_CMD_STA_POWERSAVE_MODE 0x9001u
#define WLWMI_CMD_STA_POWERSAVE_PARAM 0x9002u
#define WLWMI_EV_SCAN 0x3001u
#define WLWMI_EV_CHAN_INFO 0x4002u
#define WLWMI_EV_MGMT_RX 0x7001u
#define WLWMI_EV_PDEV_RSSI_DBM_CONVERSION 0x402bu  /* L19 moves its noise floor with it (wmi.c:9740-9796); C2 counts it, not applied */
#define WLWMI_TAG_ARRAY_BYTE 0x11u
#define WLWMI_TAG_ARRAY_FIXED_STRUCT 0x13u
#define WLWMI_TAG_SCAN_EVENT 0x24u
#define WLWMI_TAG_CHAN_INFO_EVENT 0x26u
#define WLWMI_TAG_MGMT_RX_HDR 0x2cu
#define WLWMI_TAG_START_SCAN_CMD 0x4du
#define WLWMI_TAG_STOP_SCAN_CMD 0x4eu
#define WLWMI_TAG_SCAN_CHAN_LIST_CMD 0x4fu
#define WLWMI_TAG_CHANNEL 0x50u
#define WLWMI_TAG_VDEV_CREATE_CMD 0x56u
#define WLWMI_TAG_VDEV_SET_PARAM_CMD 0x5fu
#define WLWMI_TAG_STA_POWERSAVE_MODE_CMD 0x6cu
#define WLWMI_TAG_STA_POWERSAVE_PARAM_CMD 0x6du
#define WLWMI_TAG_VDEV_TXRX_STREAMS 0x19bu
/* Command structs with their TLV header; event structs are the TLV value. */
#define WLWMI_CHAN_LIST_BYTES 16u       /* struct wmi_scan_chan_list_cmd (wmi.h:4001-4006) */
#define WLWMI_CHANNEL_BYTES 28u         /* struct ath12k_wmi_channel_params (wmi.h:3636-3644) */
#define WLWMI_VDEV_CREATE_BYTES 48u     /* struct wmi_vdev_create_cmd (wmi.h:2878-2890) */
#define WLWMI_TXRX_STREAMS_BYTES 16u    /* struct ath12k_wmi_vdev_txrx_streams_params (wmi.h:2892-2897) */
#define WLWMI_VDEV_SET_PARAM_BYTES 16u  /* struct wmi_vdev_set_param_cmd (wmi.h:3677-3682) */
#define WLWMI_STA_PS_PARAM_BYTES 16u    /* struct wmi_sta_powersave_param_cmd (wmi.h:3318-3323) */
#define WLWMI_STA_PS_MODE_BYTES 12u     /* struct wmi_pdev_set_ps_mode_cmd (wmi.h:3286-3290) */
#define WLWMI_START_SCAN_BYTES 160u     /* struct wmi_start_scan_cmd (wmi.h:3432-3464) */
#define WLWMI_STOP_SCAN_BYTES 24u       /* struct wmi_stop_scan_cmd (wmi.h:3985-3992) */
#define WLWMI_MAC_ADDR_BYTES 8u         /* struct ath12k_wmi_mac_addr_params, 6 + 2 pad (wmi.h:2825-2828) */
#define WLWMI_SCAN_EVENT_BYTES 28u      /* struct wmi_scan_event (wmi.h:4525-4538) */
#define WLWMI_CHAN_INFO_BYTES 56u       /* struct wmi_chan_info_event (wmi.h:4586-4601) */
#define WLWMI_MGMT_RX_HDR_BYTES 68u     /* struct ath12k_wmi_mgmt_rx_params (wmi.h:4498-4513) */
#define WLWMI_CHAN_INFO_PASSIVE (1u << 7)        /* wmi.h:3613-3626 */
#define WLWMI_CHAN_INFO_DFS (1u << 10)
#define WLWMI_CHAN_INFO_ALLOW_HE (1u << 17)
#define WLWMI_MODE_11A 0u                        /* enum wmi_phy_mode (wmi.h:3031-3034) */
#define WLWMI_MODE_11G 1u
#define WLWMI_APPEND_CHAN_LIST 1u                /* WMI_APPEND_TO_EXISTING_CHAN_LIST_FLAG (wmi.h:170) */
/* reg_info power in half dB: the 20 dBm cfg80211's world domain allows on
 * every listed channel (mac.c:31 max_power 30 capped by the rule), antenna
 * gain 0; L19 doubles the channel values (reg.c:212-215) [I]. Nothing is
 * transmitted, so they bound nothing in C2. */
#define WLWMI_CHAN_POWER_HALF_DB 40u
#define WLWMI_VDEV_ID 0u                         /* the first free vdev (mac.c:10051) */
#define WLWMI_VDEV_TYPE_STA 2u                   /* wmi.h:4626 */
#define WLWMI_VDEV_SUBTYPE_NONE 0u               /* wmi.h:4631-4632 */
#define WLWMI_MBSSID_NON_MBSSID_AP 1u            /* wmi.h:2974; mac.c:9742 */
#define WLWMI_BAND_2G 0u                         /* WMI_TPC_CHAINMASK_CONFIG_BAND_2G/5G (wmi.h:2272-2274) */
#define WLWMI_BAND_5G 1u
#define WLWMI_NUM_BANDS 2u                       /* WMI_NUM_SUPPORTED_BAND_MAX */
#define WLWMI_HOST_2GHZ_CAP 1u                   /* wmi.h:220-221 */
#define WLWMI_HOST_5GHZ_CAP 2u
#define WLWMI_VDEV_PARAM_NSS 0x22u               /* enum wmi_tlv_vdev_param (wmi.h:1114) */
#define WLWMI_STA_PS_PARAM_RX_WAKE_POLICY 0u     /* wmi.h:4642-4644 */
#define WLWMI_STA_PS_PARAM_TX_WAKE_THRESHOLD 1u
#define WLWMI_STA_PS_PARAM_PSPOLL_COUNT 2u
#define WLWMI_STA_PS_RX_WAKE_POLICY_WAKE 0u      /* wmi.h:4828 */
#define WLWMI_STA_PS_TX_WAKE_THRESHOLD_ALWAYS 1u /* wmi.h:4662 */
#define WLWMI_STA_PS_PSPOLL_COUNT_NO_MAX 0u      /* wmi.h:4678 */
#define WLWMI_STA_PS_MODE_DISABLED 0u            /* wmi.h:3647 */
#define WLWMI_SCAN_ID 0xa000u                    /* ATH12K_SCAN_ID (wmi.h:3597) */
#define WLWMI_SCAN_REQ_ID 1u                     /* wmi.c:2545 */
#define WLWMI_SCAN_EV_STARTED (1u << 0)          /* enum wmi_scan_event_type (wmi.h:3407-3421) */
#define WLWMI_SCAN_EV_COMPLETED (1u << 1)
#define WLWMI_SCAN_EV_BSS_CHANNEL (1u << 2)
#define WLWMI_SCAN_EV_FOREIGN_CHAN (1u << 3)
#define WLWMI_SCAN_EV_DEQUEUED (1u << 4)
#define WLWMI_SCAN_EV_START_FAILED (1u << 6)
#define WLWMI_SCAN_REASON_CANCELLED 1u           /* enum wmi_scan_completion_reason (wmi.h:3423-3430) */
#define WLWMI_SCAN_FLAG_PASSIVE 0x1u             /* wmi.h:3466 */
#define WLWMI_SCAN_CHAN_STAT_EVENT 0x10u         /* wmi.h:3470 */
#define WLWMI_SCAN_STOP_ONE 0u                   /* wmi.h:3590 */
#define WLWMI_CHAN_INFO_START_RESP 0u            /* wmi.h:4583-4584 */
#define WLWMI_CHAN_INFO_END_RESP 1u
#define WLWMI_RX_ERR_CRC 0x01u                   /* wmi.h:4473-4477 */
#define WLWMI_RX_ERR_DECRYPT 0x08u
#define WLWMI_RX_ERR_KEY_CACHE_MISS 0x20u
#define WLWMI_NOISE_FLOOR (-95)                  /* ATH12K_DEFAULT_NOISE_FLOOR (core.h:634; mac.c:14226-14228) */
#define WLWMI_MAX_5GHZ_CHAN 173u                 /* ATH12K_MAX_5GHZ_CHAN (core.h:615) */
#define WLWMI_MIN_6GHZ_FREQ 5925u                /* ATH12K_MIN/MAX_6GHZ_FREQ (core.h:601-613) */
#define WLWMI_MAX_6GHZ_FREQ 7125u
/* ath12k_wmi_start_scan_init (wmi.c:2541-2571) and the passive request of
 * ath12k_mac_initiate_hw_scan (mac.c:5335-5366): no SSID = scan_f_passive. C2
 * adds START_FAILED to the subscribed events (L19 handles it unsubscribed,
 * wmi.c:7351-7354) and sets max_scan_time to n x dwell x 3/2 instead of
 * 20000 (overnight_plan.md 3.2). */
#define WLSCAN_DWELL_ACTIVE 50u
#define WLSCAN_DWELL_ACTIVE_2G 0u
#define WLSCAN_DWELL_PASSIVE 150u
#define WLSCAN_DWELL_ACTIVE_6G 70u
#define WLSCAN_DWELL_PASSIVE_6G 70u
#define WLSCAN_MIN_REST 50u
#define WLSCAN_MAX_REST 500u
#define WLSCAN_PROBE_DELAY 5u
#define WLSCAN_EVENTS (WLWMI_SCAN_EV_STARTED | WLWMI_SCAN_EV_COMPLETED | WLWMI_SCAN_EV_BSS_CHANNEL | \
	WLWMI_SCAN_EV_FOREIGN_CHAN | WLWMI_SCAN_EV_DEQUEUED | WLWMI_SCAN_EV_START_FAILED)
#define WLSCAN_FLAGS (WLWMI_SCAN_FLAG_PASSIVE | WLWMI_SCAN_CHAN_STAT_EVENT)
#define WLSCAN_NCHAN 38u
#define WLSCAN_NCHAN_24 13u
#define WLSCAN_BSS_MAX 64u
_Static_assert(WLWMI_CHANNEL_BYTES == 28 && WLWMI_VDEV_CREATE_BYTES == 48 && WLWMI_START_SCAN_BYTES == 160 &&
    WLWMI_STOP_SCAN_BYTES == 24 && WLWMI_SCAN_EVENT_BYTES == 28 && WLWMI_CHAN_INFO_BYTES == 56 &&
    WLWMI_MGMT_RX_HDR_BYTES == 68 && WLHTT_VER_REQ_BYTES == 8, "C2 WMI/HTT wire sizes");
_Static_assert(WLSCAN_FLAGS == 0x11u && !(WLSCAN_FLAGS & 0x2u), "C2 scans are passive: no broadcast probe request");

/* struct wmi_start_scan_cmd word index (0 = TLV header). */
enum {
	WLWMI_SS_SCAN_ID = 1, WLWMI_SS_SCAN_REQ_ID, WLWMI_SS_VDEV_ID, WLWMI_SS_PRIORITY, WLWMI_SS_NOTIFY,
	WLWMI_SS_DWELL_ACTIVE, WLWMI_SS_DWELL_PASSIVE, WLWMI_SS_MIN_REST, WLWMI_SS_MAX_REST, WLWMI_SS_REPEAT_PROBE,
	WLWMI_SS_PROBE_SPACING, WLWMI_SS_IDLE, WLWMI_SS_MAX_SCAN_TIME, WLWMI_SS_PROBE_DELAY, WLWMI_SS_CTRL_FLAGS,
	WLWMI_SS_BURST, WLWMI_SS_NUM_CHAN, WLWMI_SS_NUM_BSSID, WLWMI_SS_NUM_SSIDS, WLWMI_SS_IE_LEN, WLWMI_SS_N_PROBES,
	WLWMI_SS_MAC_ADDR, WLWMI_SS_MAC_MASK = WLWMI_SS_MAC_ADDR + 2, WLWMI_SS_IE_BITMAP = WLWMI_SS_MAC_MASK + 2,
	WLWMI_SS_NUM_VENDOR_OUI = WLWMI_SS_IE_BITMAP + 8, WLWMI_SS_CTRL_FLAGS_EXT, WLWMI_SS_DWELL_ACTIVE_2G,
	WLWMI_SS_DWELL_ACTIVE_6G, WLWMI_SS_DWELL_PASSIVE_6G, WLWMI_SS_START_OFFSET, WLWMI_SS_WORDS
};
_Static_assert(WLWMI_SS_WORDS * 4 == WLWMI_START_SCAN_BYTES && WLWMI_SS_MAX_SCAN_TIME == 13 &&
    WLWMI_SS_CTRL_FLAGS == 15 && WLWMI_SS_N_PROBES == 21, "WMI START_SCAN word order (wmi.h:3432-3464)");

/* C2b: struct wmi_chan_info_event word index (the TLV value, wmi.h:4586-4601);
 * C2 reads cmd_flags, noise_floor and vdev_id at bytes 8, 12 and 52. */
enum {
	WLWMI_CI_ERR_CODE, WLWMI_CI_FREQ, WLWMI_CI_CMD_FLAGS, WLWMI_CI_NOISE_FLOOR, WLWMI_CI_RX_CLEAR_COUNT,
	WLWMI_CI_CYCLE_COUNT, WLWMI_CI_CHAN_TX_PWR_RANGE, WLWMI_CI_CHAN_TX_PWR_TP, WLWMI_CI_RX_FRAME_COUNT,
	WLWMI_CI_MY_BSS_RX_CYCLE_COUNT, WLWMI_CI_RX_11B_MODE_DATA_DURATION, WLWMI_CI_TX_FRAME_CNT, WLWMI_CI_MAC_CLK_MHZ,
	WLWMI_CI_VDEV_ID, WLWMI_CI_WORDS
};
_Static_assert(WLWMI_CI_WORDS * 4 == WLWMI_CHAN_INFO_BYTES && WLWMI_CI_CMD_FLAGS == 2 && WLWMI_CI_NOISE_FLOOR == 3 &&
    WLWMI_CI_VDEV_ID == 13, "WMI CHAN_INFO word order (wmi.h:4586-4601)");

/* The C2 channel list, one for the radio and the iOS side (overnight_plan.md
 * 3.2): 2.4 GHz 1-13, 5 GHz 36-48, 52-64 (DFS), 100-144 (DFS), 149-165; no
 * 6 GHz. The 13 2.4 GHz channels come first: w20 0x10 scans those only. */
static inline uint32_t
wlscan_chan(uint32_t i)
{
	static const uint16_t mhz[WLSCAN_NCHAN] = {
		2412, 2417, 2422, 2427, 2432, 2437, 2442, 2447, 2452, 2457, 2462, 2467, 2472,
		5180, 5200, 5220, 5240, 5260, 5280, 5300, 5320,
		5500, 5520, 5540, 5560, 5580, 5600, 5620, 5640, 5660, 5680, 5700, 5720,
		5745, 5765, 5785, 5805, 5825,
	};
	return i < WLSCAN_NCHAN ? mhz[i] : 0u;
}

static inline bool wlscan_is_dfs(uint32_t mhz) { return (mhz >= 5260u && mhz <= 5320u) || (mhz >= 5500u && mhz <= 5720u); }
static inline uint32_t wlscan_max_time(uint32_t n) { return n * WLSCAN_DWELL_PASSIVE * 3u / 2u; }

/* C3: whether MHZ is one of the first N channels of the list (the ones the
 * channel list gave the firmware: 38, or 13 with w20 0x10). */
static inline bool
wlscan_listed(uint32_t mhz, uint32_t n)
{
	for (uint32_t i = 0; i < n && i < WLSCAN_NCHAN; i++) { if (wlscan_chan(i) == mhz) { return true; } }
	return false;
}

static inline uint32_t
wlwmi_popcount(uint32_t v)
{
	uint32_t n = 0;
	for (; v; v &= v - 1u) { n++; }
	return n;
}

/* Channel info as ath12k_reg_update_chan_list and the encoder set it
 * (reg.c:200-221, wmi.c:2942-2963): allow_ht/vht/he all true, of which the
 * else-if chain sends ALLOW_HE only; PASSIVE (C2: every channel; L19 for
 * NO_IR or radar); DFS on radar channels; MODE_11G on 2.4 GHz, else 11A. */
static inline uint32_t
wlwmi_chan_info(uint32_t mhz)
{
	return (mhz < 3000u ? WLWMI_MODE_11G : WLWMI_MODE_11A) | WLWMI_CHAN_INFO_PASSIVE | WLWMI_CHAN_INFO_ALLOW_HE |
	    (wlscan_is_dfs(mhz) ? WLWMI_CHAN_INFO_DFS : 0u);
}

/* Channels per SCAN_CHAN_LIST: L19 bounds the TLV payload (fixed TLV, array
 * header, entries) by the WMI endpoint's max message (wmi.c:2895-2903);
 * here also by one TX slot with the HTC and WMI headers. */
static inline uint32_t
wlwmi_chan_list_max(uint32_t max_msg)
{
	uint32_t room = WLCE_BUF - WLHTC_HDR - 4u;
	if (max_msg < room) { room = max_msg; }
	return room > WLWMI_CHAN_LIST_BYTES + 4u ? (room - WLWMI_CHAN_LIST_BYTES - 4u) / WLWMI_CHANNEL_BYTES : 0u;
}

/* SCAN_CHAN_LIST (wmi.c:2880-2997) for list entries FIRST .. FIRST + N - 1:
 * TLV 0x4f {num_scan_chans, flags (APPEND after the first command), pdev},
 * then ARRAY_STRUCT of TLV 0x50 {mhz, cfreq1 = mhz, cfreq2 0, info,
 * reg_info_1 = min 0 | max power 15:8 | max reg power 23:16 | class 0,
 * reg_info_2 = antenna 0 | max tx power 15:8 = max reg power}. L19 writes the
 * array header with ath12k_wmi_tlv_cmd_hdr, which takes the 4-byte header off
 * the length: it says 28 x N - 4 (wmi.c:204-207, 2923-2926); sent as L19 does. */
static inline int
wlwmi_encode_chan_list(uint32_t first, uint32_t n, bool append, uint32_t pdev, uint8_t *out, uint32_t room, uint32_t *len)
{
	uint32_t total = 4u + WLWMI_CHAN_LIST_BYTES + 4u + n * WLWMI_CHANNEL_BYTES;
	if (!n || n > WLSCAN_NCHAN || first > WLSCAN_NCHAN - n || total > room) { return 1; }
	for (uint32_t i = 0; i < total; i++) { out[i] = 0; }
	wlce_put32(out, WLWMI_CMD_SCAN_CHAN_LIST);
	wlwmi_put_tlv(out + 4, WLWMI_TAG_SCAN_CHAN_LIST_CMD, WLWMI_CHAN_LIST_BYTES - 4u);
	wlce_put32(out + 8, n);
	wlce_put32(out + 12, append ? WLWMI_APPEND_CHAN_LIST : 0u);
	wlce_put32(out + 16, pdev);
	wlwmi_put_tlv(out + 20, WLWMI_TAG_ARRAY_STRUCT, n * WLWMI_CHANNEL_BYTES - 4u);
	for (uint32_t i = 0; i < n; i++) {
		uint8_t *c = out + 24 + i * WLWMI_CHANNEL_BYTES;
		uint32_t mhz = wlscan_chan(first + i);
		wlwmi_put_tlv(c, WLWMI_TAG_CHANNEL, WLWMI_CHANNEL_BYTES - 4u);
		wlce_put32(c + 4, mhz);
		wlce_put32(c + 8, mhz);
		wlce_put32(c + 16, wlwmi_chan_info(mhz));
		wlce_put32(c + 20, (WLWMI_CHAN_POWER_HALF_DB << 8) | (WLWMI_CHAN_POWER_HALF_DB << 16));
		wlce_put32(c + 24, WLWMI_CHAN_POWER_HALF_DB << 8);
	}
	*len = total;
	return 0;
}

/* VDEV_CREATE (wmi.c:923-1010, mac.c:9727-9780, 10004-10087): TLV 0x56
 * {vdev 0, type STA, subtype NONE, MAC + 2 pad, num_cfg_txrx_streams 2, pdev,
 * mbssid flags NON_MBSSID_AP, tx vdev 0, stats id valid BIT(0), stats id 0
 * (the first free one, mac.c:9667-9687)}, then ARRAY_STRUCT (exact length,
 * ath12k_wmi_tlv_hdr) of two TLV 0x19b {band 2G / 5G, tx and rx streams =
 * the pdev's chains where the pdev has the band, else 0}. No MLO TLV: not an
 * MLD. The firmware sends no reply (L19 waits for none). */
#define WLWMI_VDEV_CREATE_TOTAL (4u + WLWMI_VDEV_CREATE_BYTES + 4u + WLWMI_NUM_BANDS * WLWMI_TXRX_STREAMS_BYTES)
static inline int
wlwmi_encode_vdev_create(const uint8_t *mac, uint32_t pdev, uint32_t bands, uint32_t tx, uint32_t rx,
    uint8_t *out, uint32_t room, uint32_t *len)
{
	if (room < WLWMI_VDEV_CREATE_TOTAL) { return 1; }
	for (uint32_t i = 0; i < WLWMI_VDEV_CREATE_TOTAL; i++) { out[i] = 0; }
	wlce_put32(out, WLWMI_CMD_VDEV_CREATE);
	wlwmi_put_tlv(out + 4, WLWMI_TAG_VDEV_CREATE_CMD, WLWMI_VDEV_CREATE_BYTES - 4u);
	wlce_put32(out + 8, WLWMI_VDEV_ID);
	wlce_put32(out + 12, WLWMI_VDEV_TYPE_STA);
	wlce_put32(out + 16, WLWMI_VDEV_SUBTYPE_NONE);
	memcpy(out + 20, mac, 6);
	wlce_put32(out + 28, WLWMI_NUM_BANDS);
	wlce_put32(out + 32, pdev);
	wlce_put32(out + 36, WLWMI_MBSSID_NON_MBSSID_AP);
	wlce_put32(out + 44, 1u);
	wlwmi_put_tlv(out + 52, WLWMI_TAG_ARRAY_STRUCT, WLWMI_NUM_BANDS * WLWMI_TXRX_STREAMS_BYTES);
	for (uint32_t b = 0; b < WLWMI_NUM_BANDS; b++) {
		uint8_t *s = out + 56 + b * WLWMI_TXRX_STREAMS_BYTES;
		bool has = bands & (b == WLWMI_BAND_2G ? WLWMI_HOST_2GHZ_CAP : WLWMI_HOST_5GHZ_CAP);
		wlwmi_put_tlv(s, WLWMI_TAG_VDEV_TXRX_STREAMS, WLWMI_TXRX_STREAMS_BYTES - 4u);
		wlce_put32(s + 4, b == 0 ? WLWMI_BAND_2G : WLWMI_BAND_5G);
		wlce_put32(s + 8, has ? tx : 0u);
		wlce_put32(s + 12, has ? rx : 0u);
	}
	*len = WLWMI_VDEV_CREATE_TOTAL;
	return 0;
}

/* A command that is one fixed TLV of NW words: VDEV_SET_PARAM {vdev, param,
 * value} (wmi.c:1877-1909), STA_POWERSAVE_PARAM {vdev, param, value}
 * (wmi.c:1815-1846), STA_POWERSAVE_MODE {vdev, mode} (wmi.c:1652-1680),
 * STOP_SCAN {requestor, scan id, req type, vdev, pdev} (wmi.c:2832-2878). */
static inline int
wlwmi_encode_words(uint32_t cmd, uint32_t tag, const uint32_t *w, uint32_t nw, uint8_t *out, uint32_t room, uint32_t *len)
{
	uint32_t total = 8u + 4u * nw;
	if (total > room) { return 1; }
	wlce_put32(out, cmd);
	wlwmi_put_tlv(out + 4, tag, 4u * nw);
	for (uint32_t i = 0; i < nw; i++) { wlce_put32(out + 8 + 4 * i, w[i]); }
	*len = total;
	return 0;
}

/* START_SCAN (wmi.c:2646-2830): the fixed TLV 0x4d with the
 * ath12k_wmi_start_scan_init values; scan_priority stays 0 on the wire (L19
 * fills arg->scan_priority, never cmd->scan_priority, wmi.c:2708-2712);
 * notify events and max_scan_time as above; flags PASSIVE | CHAN_STAT_EVENT,
 * adaptive dwell mode 0 (wmi.c:2600-2643). Then ARRAY_UINT32 of the N MHz
 * values, ARRAY_FIXED_STRUCT of 0 SSIDs, ARRAY_FIXED_STRUCT with the
 * broadcast BSSID + 2 pad, ARRAY_BYTE of 0 extra IE bytes; no hint arrays. */
static inline uint32_t
wlwmi_start_scan_total(uint32_t n)
{
	return 4u + WLWMI_START_SCAN_BYTES + 4u + 4u * n + 4u + 4u + WLWMI_MAC_ADDR_BYTES + 4u;
}

static inline int
wlwmi_encode_start_scan(uint32_t n, uint32_t max_time, uint8_t *out, uint32_t room, uint32_t *len)
{
	uint32_t total = wlwmi_start_scan_total(n);
	if (!n || n > WLSCAN_NCHAN || total > room) { return 1; }
	for (uint32_t i = 0; i < total; i++) { out[i] = 0; }
	uint8_t *f = out + 4, *p = f + WLWMI_START_SCAN_BYTES;
	wlce_put32(out, WLWMI_CMD_START_SCAN);
	wlwmi_put_tlv(f, WLWMI_TAG_START_SCAN_CMD, WLWMI_START_SCAN_BYTES - 4u);
	wlce_put32(f + 4 * WLWMI_SS_SCAN_ID, WLWMI_SCAN_ID);
	wlce_put32(f + 4 * WLWMI_SS_SCAN_REQ_ID, WLWMI_SCAN_REQ_ID);
	wlce_put32(f + 4 * WLWMI_SS_VDEV_ID, WLWMI_VDEV_ID);
	wlce_put32(f + 4 * WLWMI_SS_NOTIFY, WLSCAN_EVENTS);
	wlce_put32(f + 4 * WLWMI_SS_DWELL_ACTIVE, WLSCAN_DWELL_ACTIVE);
	wlce_put32(f + 4 * WLWMI_SS_DWELL_PASSIVE, WLSCAN_DWELL_PASSIVE);
	wlce_put32(f + 4 * WLWMI_SS_MIN_REST, WLSCAN_MIN_REST);
	wlce_put32(f + 4 * WLWMI_SS_MAX_REST, WLSCAN_MAX_REST);
	wlce_put32(f + 4 * WLWMI_SS_MAX_SCAN_TIME, max_time);
	wlce_put32(f + 4 * WLWMI_SS_PROBE_DELAY, WLSCAN_PROBE_DELAY);
	wlce_put32(f + 4 * WLWMI_SS_CTRL_FLAGS, WLSCAN_FLAGS);
	wlce_put32(f + 4 * WLWMI_SS_NUM_CHAN, n);
	wlce_put32(f + 4 * WLWMI_SS_NUM_BSSID, 1u);
	wlce_put32(f + 4 * WLWMI_SS_DWELL_ACTIVE_2G, WLSCAN_DWELL_ACTIVE_2G);
	wlce_put32(f + 4 * WLWMI_SS_DWELL_ACTIVE_6G, WLSCAN_DWELL_ACTIVE_6G);
	wlce_put32(f + 4 * WLWMI_SS_DWELL_PASSIVE_6G, WLSCAN_DWELL_PASSIVE_6G);
	wlwmi_put_tlv(p, WLWMI_TAG_ARRAY_UINT32, 4u * n);
	for (uint32_t i = 0; i < n; i++) { wlce_put32(p + 4 + 4 * i, wlscan_chan(i)); }
	p += 4u + 4u * n;
	wlwmi_put_tlv(p, WLWMI_TAG_ARRAY_FIXED_STRUCT, 0);
	p += 4;
	wlwmi_put_tlv(p, WLWMI_TAG_ARRAY_FIXED_STRUCT, WLWMI_MAC_ADDR_BYTES);
	for (uint32_t i = 0; i < 6; i++) { p[4 + i] = 0xff; }
	p += 4u + WLWMI_MAC_ADDR_BYTES;
	wlwmi_put_tlv(p, WLWMI_TAG_ARRAY_BYTE, 0);
	*len = total;
	return 0;
}

/* C3 (W25, one scan per iOS request): the START_SCAN above for the N
 * channels of MHZ, scan id ID and passive dwell DWELL ms; then only those
 * three things differ. PASSIVE and CHAN_STAT_EVENT stay forced (there is no
 * flags argument), still no SSID, IE or probe. Refused: N outside 1..38, a
 * channel not on the list, a dwell outside 50..200 ms (the iOS side's own
 * range), an id outside WLSCAN_ID_FIRST..WLSCAN_ID_LAST. Ids: ATH12K_SCAN_ID
 * (0xa000, the C2 scan) with a 12-bit counter in the low bits, so a late
 * event of an earlier scan never matches the scan in flight. */
#define WLSCAN_ID_FIRST 0xa001u
#define WLSCAN_ID_LAST 0xafffu
static inline int
wlwmi_encode_start_scan_list(const uint16_t *mhz, uint32_t n, uint32_t id, uint32_t dwell, uint32_t max_time,
    uint8_t *out, uint32_t room, uint32_t *len)
{
	if (!mhz || n > WLSCAN_NCHAN || dwell < 50u || dwell > 200u || id < WLSCAN_ID_FIRST || id > WLSCAN_ID_LAST) { return 1; }
	for (uint32_t i = 0; i < n; i++) { if (!wlscan_listed(mhz[i], WLSCAN_NCHAN)) { return 1; } }
	if (wlwmi_encode_start_scan(n, max_time, out, room, len)) { return 1; }
	uint8_t *f = out + 4, *p = f + WLWMI_START_SCAN_BYTES + 4u;
	wlce_put32(f + 4 * WLWMI_SS_SCAN_ID, id);
	wlce_put32(f + 4 * WLWMI_SS_DWELL_PASSIVE, dwell);
	for (uint32_t i = 0; i < n; i++) { wlce_put32(p + 4 * i, mhz[i]); }
	return 0;
}

/* What the scan heard. One entry per BSSID: the strongest reception's
 * frequency and dBm, every flag seen, the first usable SSID. */
enum { WLSCAN_SSID_NONE, WLSCAN_SSID_OK, WLSCAN_SSID_HIDDEN, WLSCAN_SSID_BAD };
struct wlscan_bss {
	uint8_t bssid[6], ssid[32];
	uint8_t ssid_len, ssid_state, chan;
	bool beacon, probe_resp, privacy, rsn;
	uint16_t cap;
	uint32_t freq, frames;
	int32_t dbm;
};

struct wlscan {
	uint32_t nchan;
	int32_t noise_floor;            /* dBm = SNR + this (wmi.c:7184-7187) */
	bool started, done, start_failed, dequeued;
	uint32_t reason;                /* of COMPLETED, DEQUEUED or START_FAILED */
	uint32_t ev_type, ev_reason, ev_freq, ev_scan_id, ev_vdev;   /* the last SCAN_EVENT */
	uint32_t events, foreign, bss_channel, other_types, ignored, wrong_vdev, bad_events;
	uint32_t ci_start, ci_end, ci_ignored;
	uint32_t ci_seen, ci[WLWMI_CI_WORDS];   /* C2b: well-formed CHAN_INFO events, the last one's words */
	int32_t nf_min, nf_max;
	bool nf_seen;
	uint32_t mgmt, frames, drop_status, drop_channel, drop_bad, other_mgmt, ie_truncated, table_full;
	uint32_t nbss;
	struct wlscan_bss bss[WLSCAN_BSS_MAX];
	const char *why;
	/* C3 (W25), 0 in C2 (wlscan_init): the scan id SCAN_EVENTs must carry (0 = any; others counted in
	 * wrong_id and ignored), and who gets every beacon or probe response the table took, during the
	 * call that has its RX buffer: the frame, its length, the RX MHz, dBm (SNR + noise floor), the floor. */
	uint32_t want_id, wrong_id;
	void (*heard)(void *ctx, const uint8_t *frame, uint32_t len, uint32_t mhz, int32_t dbm, int32_t nf);
	void *heard_ctx;
};

static inline void
wlscan_init(struct wlscan *s, uint32_t nchan)
{
	for (size_t i = 0; i < sizeof(*s); i++) { ((uint8_t *)s)[i] = 0; }
	s->nchan = nchan;
	s->noise_floor = WLWMI_NOISE_FLOOR;
}

static inline int
wlscan_fail(struct wlscan *s, const char *why)
{
	if (!s->why) { s->why = why; }
	return 1;
}

static inline bool
wlscan_known(uint32_t id)
{
	return id == WLWMI_EV_SCAN || id == WLWMI_EV_CHAN_INFO || id == WLWMI_EV_MGMT_RX;
}

/* The last TLV of TAG (L19's tlv_parse keeps the last, wmi.c:370-379) and
 * whether any TLV of TAG was below MIN (L19's policy then drops the event,
 * wmi.c:345-357). -1 = a TLV past the buffer (stops the run, as in C1). */
static inline int
wlscan_tlv(const uint8_t *p, uint32_t len, uint32_t tag, uint32_t min, const uint8_t **v, bool *short_seen)
{
	struct wlwmi_it it;
	int r;
	*v = 0; *short_seen = false;
	wlwmi_it_init(&it, p, len);
	while ((r = wlwmi_next(&it)) > 0) {
		if (it.tag != tag) { continue; }
		if (it.tlen < min) { *short_seen = true; } else { *v = it.v; }
	}
	return r;
}

/* SCAN_EVENT (wmi.c:6473-6504, 7289-7368), TLV 0x24, vdev 0 only: STARTED and
 * START_FAILED count while starting, FOREIGN_CHAN and COMPLETED while running
 * (L19 ignores COMPLETED before STARTED, wmi.c:6363-6387); DEQUEUED ends the
 * scan as L19's __ath12k_mac_scan_finish does (wmi.c:7355-7357). */
static inline int
wlscan_parse_event(struct wlscan *s, const uint8_t *p, uint32_t len)
{
	const uint8_t *v;
	bool bad;
	if (wlscan_tlv(p, len, WLWMI_TAG_SCAN_EVENT, WLWMI_SCAN_EVENT_BYTES, &v, &bad) < 0) {
		return wlscan_fail(s, "WMI SCAN_EVENT TLV past the buffer");
	}
	if (bad || !v) { s->bad_events++; return 0; }
	s->events++;
	s->ev_type = wlce_get32(v); s->ev_reason = wlce_get32(v + 4); s->ev_freq = wlce_get32(v + 8);
	s->ev_scan_id = wlce_get32(v + 16); s->ev_vdev = wlce_get32(v + 20);
	if (s->ev_vdev != WLWMI_VDEV_ID) { s->wrong_vdev++; return 0; }
	if (s->want_id && s->ev_scan_id != s->want_id) { s->wrong_id++; return 0; }   /* C3: another scan's */
	bool starting = !s->started && !s->start_failed && !s->done, running = s->started && !s->done;
	switch (s->ev_type) {
	case WLWMI_SCAN_EV_STARTED:
		if (starting) { s->started = true; } else { s->ignored++; }
		break;
	case WLWMI_SCAN_EV_START_FAILED:
		if (starting) { s->start_failed = true; s->reason = s->ev_reason; } else { s->ignored++; }
		break;
	case WLWMI_SCAN_EV_COMPLETED:
		if (running) { s->done = true; s->reason = s->ev_reason; } else { s->ignored++; }
		break;
	case WLWMI_SCAN_EV_DEQUEUED:
		if (!s->done && !s->start_failed) { s->done = s->dequeued = true; s->reason = s->ev_reason; } else { s->ignored++; }
		break;
	case WLWMI_SCAN_EV_FOREIGN_CHAN:
		if (running) { s->foreign++; } else { s->ignored++; }
		break;
	case WLWMI_SCAN_EV_BSS_CHANNEL:
		s->bss_channel++;
		break;
	default:
		s->other_types++;
		break;
	}
	return 0;
}

/* CHAN_INFO (wmi.c:6587-6623, 7493-7560), TLV 0x26: END_RESP ends the report;
 * START_RESP of vdev 0 while the scan runs carries the noise floor. C2b keeps
 * every word of each well-formed event for its detail line. */
static inline int
wlscan_parse_chan_info(struct wlscan *s, const uint8_t *p, uint32_t len)
{
	const uint8_t *v;
	bool bad;
	if (wlscan_tlv(p, len, WLWMI_TAG_CHAN_INFO_EVENT, WLWMI_CHAN_INFO_BYTES, &v, &bad) < 0) {
		return wlscan_fail(s, "WMI CHAN_INFO TLV past the buffer");
	}
	if (bad || !v) { s->bad_events++; return 0; }
	for (uint32_t i = 0; i < WLWMI_CI_WORDS; i++) { s->ci[i] = wlce_get32(v + 4 * i); }
	s->ci_seen++;
	uint32_t flags = wlce_get32(v + 8), vdev = wlce_get32(v + 52);
	int32_t nf = (int32_t)wlce_get32(v + 12);
	if (flags == WLWMI_CHAN_INFO_END_RESP) { s->ci_end++; return 0; }
	if (vdev != WLWMI_VDEV_ID || !s->started || s->done || flags != WLWMI_CHAN_INFO_START_RESP) { s->ci_ignored++; return 0; }
	s->ci_start++;
	if (!s->nf_seen || nf < s->nf_min) { s->nf_min = nf; }
	if (!s->nf_seen || nf > s->nf_max) { s->nf_max = nf; }
	s->nf_seen = true;
	return 0;
}

/* An 802.11 beacon (subtype 8) or probe response (subtype 5), management
 * type 0: 24-byte header with the BSSID as address 3 (bytes 16..21), then
 * timestamp 8, interval 2, capability 2 (Privacy = bit 4), then elements
 * {id, length, body} (IEEE 802.11; layout facts, not code). The walk stops
 * at the first element that does not fit, as cfg80211's element iterator
 * does; elements before it still count. SSID (0): length 0 or all zero bytes
 * = hidden, 1..32 = the name, above 32 = invalid; DS parameter set (3), else
 * HT operation (61), gives the channel; RSN (48) is noted.
 * 0 = a BSS, 1 = another frame type, 2 = too short for the fixed fields. */
static inline int
wl80211_parse_bss(const uint8_t *f, uint32_t len, struct wlscan_bss *b, uint32_t *truncated)
{
	if (len < 2) { return 2; }
	uint32_t fc = wlce_get16(f), sub = (fc >> 4) & 0xfu, off = 36, ht = 0;
	bool ssid_seen = false;
	if ((fc & 0x0cu) || (sub != 8u && sub != 5u)) { return 1; }
	if (len < 36u) { return 2; }
	memcpy(b->bssid, f + 16, 6);
	b->cap = (uint16_t)wlce_get16(f + 34);
	b->privacy = (b->cap >> 4) & 1u;
	b->beacon = sub == 8u;
	b->probe_resp = sub == 5u;
	while (off < len) {
		if (len - off < 2u || (uint32_t)f[off + 1] > len - off - 2u) { (*truncated)++; break; }
		uint32_t id = f[off], l = f[off + 1];
		const uint8_t *v = f + off + 2;
		if (id == 0 && !ssid_seen) {
			bool zero = true;
			ssid_seen = true;
			b->ssid_len = (uint8_t)l;
			if (l > 32u) { b->ssid_state = WLSCAN_SSID_BAD; }
			else {
				for (uint32_t i = 0; i < l; i++) { b->ssid[i] = v[i]; if (v[i]) { zero = false; } }
				b->ssid_state = zero ? WLSCAN_SSID_HIDDEN : WLSCAN_SSID_OK;
			}
		} else if (id == 3 && l >= 1 && !b->chan) {
			b->chan = v[0];
		} else if (id == 61 && l >= 1 && !ht) {
			ht = v[0];
		} else if (id == 48) {
			b->rsn = true;
		}
		off += 2u + l;
	}
	if (!b->chan) { b->chan = (uint8_t)ht; }
	return 0;
}

static inline bool
wlscan_same(const uint8_t *a, const uint8_t *b, uint32_t n)
{
	for (uint32_t i = 0; i < n; i++) { if (a[i] != b[i]) { return false; } }
	return true;
}

/* Deduplicated by BSSID, 64 entries; a frame of a BSSID past them is not
 * kept and counts once in table_full (frames, not BSSIDs). */
static inline void
wlscan_bss_add(struct wlscan *s, const struct wlscan_bss *n)
{
	for (uint32_t i = 0; i < s->nbss; i++) {
		struct wlscan_bss *e = &s->bss[i];
		if (!wlscan_same(e->bssid, n->bssid, 6)) { continue; }
		e->frames++;
		if (n->dbm > e->dbm) { e->dbm = n->dbm; e->freq = n->freq; }
		e->beacon = e->beacon || n->beacon;
		e->probe_resp = e->probe_resp || n->probe_resp;
		e->privacy = e->privacy || n->privacy;
		e->rsn = e->rsn || n->rsn;
		if (!e->chan) { e->chan = n->chan; }
		if ((e->ssid_state != WLSCAN_SSID_OK && n->ssid_state == WLSCAN_SSID_OK) ||
		    (e->ssid_state == WLSCAN_SSID_NONE && n->ssid_state != WLSCAN_SSID_NONE)) {
			e->ssid_state = n->ssid_state; e->ssid_len = n->ssid_len;
			memcpy(e->ssid, n->ssid, sizeof(e->ssid));
		}
		return;
	}
	if (s->nbss >= WLSCAN_BSS_MAX) { s->table_full++; return; }
	s->bss[s->nbss++] = *n;
}

/* MGMT_RX (wmi.c:6162-6234, 7113-7230): the last TLV 0x2c header of at least
 * 68 bytes and the first ARRAY_BYTE (the frame, buf_len bytes of it: L19
 * bounds buf_len by the event, here by the byte array itself). Dropped as
 * L19 drops them: no header or frame, status CRC/DECRYPT/KEY_CACHE_MISS, a
 * channel neither 1..14 nor 36..173; also 6 GHz (not on the C2 list). The
 * frequency follows from the channel and band (ieee80211_channel_to_frequency);
 * dBm = SNR + noise floor in u32 as L19 (wmi.c:7187). */
static inline int
wlscan_parse_mgmt_rx(struct wlscan *s, const uint8_t *p, uint32_t len)
{
	struct wlwmi_it it;
	const uint8_t *hdr = 0, *frame = 0;
	uint32_t flen = 0, freq;
	bool short_hdr = false;
	int r;
	wlwmi_it_init(&it, p, len);
	while ((r = wlwmi_next(&it)) > 0) {
		if (it.tag == WLWMI_TAG_MGMT_RX_HDR) {
			if (it.tlen < WLWMI_MGMT_RX_HDR_BYTES) { short_hdr = true; } else { hdr = it.v; }
		} else if (it.tag == WLWMI_TAG_ARRAY_BYTE && !frame) {
			frame = it.v; flen = it.tlen;
		}
	}
	if (r < 0) { return wlscan_fail(s, "WMI MGMT_RX TLV past the buffer"); }
	s->mgmt++;
	if (short_hdr || !hdr || !frame) { s->drop_bad++; return 0; }
	uint32_t channel = wlce_get32(hdr), snr = wlce_get32(hdr + 4), buf_len = wlce_get32(hdr + 16);
	uint32_t status = wlce_get32(hdr + 20), chan_freq = wlce_get32(hdr + 64);
	if (buf_len > flen) { s->drop_bad++; return 0; }
	if (status & (WLWMI_RX_ERR_CRC | WLWMI_RX_ERR_DECRYPT | WLWMI_RX_ERR_KEY_CACHE_MISS)) { s->drop_status++; return 0; }
	if (chan_freq >= WLWMI_MIN_6GHZ_FREQ && chan_freq <= WLWMI_MAX_6GHZ_FREQ) { s->drop_channel++; return 0; }
	if (channel >= 1u && channel <= 14u) { freq = channel == 14u ? 2484u : 2407u + 5u * channel; }
	else if (channel >= 36u && channel <= WLWMI_MAX_5GHZ_CHAN) { freq = 5000u + 5u * channel; }
	else { s->drop_channel++; return 0; }
	struct wlscan_bss b;
	for (size_t i = 0; i < sizeof(b); i++) { ((uint8_t *)&b)[i] = 0; }
	int k = wl80211_parse_bss(frame, buf_len, &b, &s->ie_truncated);
	if (k == 1) { s->other_mgmt++; return 0; }
	if (k) { s->drop_bad++; return 0; }
	b.freq = freq;
	b.dbm = (int32_t)(snr + (uint32_t)s->noise_floor);
	b.frames = 1;
	if (!b.chan) { b.chan = (uint8_t)channel; }
	s->frames++;
	wlscan_bss_add(s, &b);
	if (s->heard) { s->heard(s->heard_ctx, frame, buf_len, freq, b.dbm, s->noise_floor); }   /* C3: still in its RX buffer */
	return 0;
}

static inline int
wlscan_rx(struct wlwmi *wmi, uint32_t id, const uint8_t *p, uint32_t len)
{
	struct wlscan *s = wmi->scan;
	int r = id == WLWMI_EV_SCAN ? wlscan_parse_event(s, p, len) :
	    id == WLWMI_EV_CHAN_INFO ? wlscan_parse_chan_info(s, p, len) : wlscan_parse_mgmt_rx(s, p, len);
	return r ? wlwmi_fail(wmi, s->why) : 0;
}

/* Distinct visible SSIDs, and entries without one (hidden or absent). */
static inline uint32_t
wlscan_ssids(const struct wlscan *s)
{
	uint32_t k = 0;
	for (uint32_t i = 0; i < s->nbss; i++) {
		const struct wlscan_bss *a = &s->bss[i];
		bool dup = false;
		if (a->ssid_state != WLSCAN_SSID_OK) { continue; }
		for (uint32_t j = 0; j < i && !dup; j++) {
			const struct wlscan_bss *c = &s->bss[j];
			dup = c->ssid_state == WLSCAN_SSID_OK && c->ssid_len == a->ssid_len && wlscan_same(c->ssid, a->ssid, a->ssid_len);
		}
		if (!dup) { k++; }
	}
	return k;
}

static inline uint32_t
wlscan_hidden(const struct wlscan *s)
{
	uint32_t k = 0;
	for (uint32_t i = 0; i < s->nbss; i++) {
		if (s->bss[i].ssid_state == WLSCAN_SSID_HIDDEN || s->bss[i].ssid_state == WLSCAN_SSID_NONE) { k++; }
	}
	return k;
}

/* The SSID for the C2 proof log (owner decision: in clear): printable ASCII
 * as is, anything else and the quote and backslash as \xNN; or <hidden>,
 * <none>, <invalid length N>. OUT holds WLSCAN_SSID_TEXT bytes. */
#define WLSCAN_SSID_TEXT 140u
static inline void
wlscan_ssid_text(const struct wlscan_bss *b, char *out)
{
	static const char hex[] = "0123456789abcdef";
	const char *lit = b->ssid_state == WLSCAN_SSID_HIDDEN ? "ssid <hidden>" : b->ssid_state == WLSCAN_SSID_NONE ?
	    "ssid <none>" : b->ssid_state == WLSCAN_SSID_BAD ? "ssid <invalid length " : "ssid \"";
	uint32_t o = 0;
	while (*lit) { out[o++] = *lit++; }
	if (b->ssid_state == WLSCAN_SSID_BAD) {
		uint32_t l = b->ssid_len;
		if (l >= 100u) { out[o++] = (char)('0' + l / 100u); }
		if (l >= 10u) { out[o++] = (char)('0' + l / 10u % 10u); }
		out[o++] = (char)('0' + l % 10u);
		out[o++] = '>';
	} else if (b->ssid_state == WLSCAN_SSID_OK) {
		for (uint32_t i = 0; i < b->ssid_len && i < 32u; i++) {
			uint8_t c = b->ssid[i];
			if (c >= 0x20u && c < 0x7fu && c != '"' && c != '\\') { out[o++] = (char)c; continue; }
			out[o++] = '\\'; out[o++] = 'x'; out[o++] = hex[c >> 4]; out[o++] = hex[c & 15u];
		}
		out[o++] = '"';
	}
	out[o] = 0;
}

/* ---- C2b (fuxiwlanqrtr=7): the pdev stats requests after the scan ----
 * Adapted from Linux ath12k wmi.c, wmi.h, mac.c and debugfs.c
 * (BSD-3-Clause-Clear), v6.19. Ids, tags, sizes and word orders come from
 * the wmi.h enums and structs (the host test re-derives them). The request
 * is what ath12k_wmi_send_stats_request_cmd builds (wmi.c:890-920) for L19's
 * debugfs pdev_stats reader (debugfs.c:1388-1410): stats id PDEV_STAT, vdev
 * 0, peer MAC 0 + 2 pad (the zeroed skb, wmi.c:776-794), pdev =
 * ath12k_mac_get_target_pdev_id (wmi->phy_pdev[0], then the other PHY's
 * wmi->phy_pdev[1]). Host to firmware only. */
#define WLWMI_CMD_REQUEST_STATS 0x16001u
#define WLWMI_EV_UPDATE_STATS 0x1d004u
#define WLWMI_TAG_STATS_EVENT 0x46u
#define WLWMI_TAG_REQUEST_STATS_CMD 0x8fu
#define WLWMI_REQUEST_PDEV_STAT (1u << 2)        /* enum wmi_stats_id (wmi.h:5905-5910) */
#define WLWMI_REQUEST_STATS_BYTES 24u            /* struct wmi_request_stats_cmd (wmi.h:5912-5918) */
#define WLWMI_REQUEST_STATS_TOTAL (4u + WLWMI_REQUEST_STATS_BYTES)
#define WLWMI_STATS_EVENT_BYTES 44u              /* struct wmi_stats_event (wmi.h:5891-5903) */
#define WLWMI_VDEV_STATS_BYTES 164u              /* struct wmi_vdev_stats_params (wmi.h:5934-5949) */
#define WLWMI_BCN_STATS_BYTES 12u                /* struct ath12k_wmi_bcn_stats_params (wmi.h:5951-5955) */
#define WLWMI_PDEV_STATS_BYTES 176u              /* struct ath12k_wmi_pdev_stats_params, base 28 + tx 92 + rx 56 (wmi.h:5957-6014) */

/* struct wmi_stats_event word index (the TLV value). */
enum {
	WLWMI_SE_STATS_ID, WLWMI_SE_NUM_PDEV_STATS, WLWMI_SE_NUM_VDEV_STATS, WLWMI_SE_NUM_PEER_STATS, WLWMI_SE_NUM_BCNFLT_STATS,
	WLWMI_SE_NUM_CHAN_STATS, WLWMI_SE_NUM_MIB_STATS, WLWMI_SE_PDEV_ID, WLWMI_SE_NUM_BCN_STATS, WLWMI_SE_NUM_PEER_EXTD_STATS,
	WLWMI_SE_NUM_PEER_EXTD2_STATS, WLWMI_SE_WORDS
};
/* struct ath12k_wmi_pdev_stats_params word index: base (chan_nf .. chan_tx_pwr),
 * tx (comp_queued .. txop_ovf), rx (mid_ppdu_route_change .. mpdu_errs);
 * a_sle32 fields are signed (wmi.h:5957-6014, wmi.c:8134-8195). */
enum {
	WLWMI_PS_CHAN_NF, WLWMI_PS_TX_FRAME_COUNT, WLWMI_PS_RX_FRAME_COUNT, WLWMI_PS_RX_CLEAR_COUNT, WLWMI_PS_CYCLE_COUNT,
	WLWMI_PS_PHY_ERR_COUNT, WLWMI_PS_CHAN_TX_PWR,
	WLWMI_PS_COMP_QUEUED, WLWMI_PS_COMP_DELIVERED, WLWMI_PS_MSDU_ENQUED, WLWMI_PS_MPDU_ENQUED, WLWMI_PS_WMM_DROP,
	WLWMI_PS_LOCAL_ENQUED, WLWMI_PS_LOCAL_FREED, WLWMI_PS_HW_QUEUED, WLWMI_PS_HW_REAPED, WLWMI_PS_UNDERRUN,
	WLWMI_PS_TX_ABORT, WLWMI_PS_MPDUS_REQUED, WLWMI_PS_TX_KO, WLWMI_PS_DATA_RC, WLWMI_PS_SELF_TRIGGERS,
	WLWMI_PS_SW_RETRY_FAILURE, WLWMI_PS_ILLGL_RATE_PHY_ERR, WLWMI_PS_PDEV_CONT_XRETRY, WLWMI_PS_PDEV_TX_TIMEOUT,
	WLWMI_PS_PDEV_RESETS, WLWMI_PS_STATELESS_TID_ALLOC_FAILURE, WLWMI_PS_PHY_UNDERRUN, WLWMI_PS_TXOP_OVF,
	WLWMI_PS_MID_PPDU_ROUTE_CHANGE, WLWMI_PS_STATUS_RCVD, WLWMI_PS_R0_FRAGS, WLWMI_PS_R1_FRAGS, WLWMI_PS_R2_FRAGS,
	WLWMI_PS_R3_FRAGS, WLWMI_PS_HTT_MSDUS, WLWMI_PS_HTT_MPDUS, WLWMI_PS_LOC_MSDUS, WLWMI_PS_LOC_MPDUS,
	WLWMI_PS_OVERSIZE_AMSDU, WLWMI_PS_PHY_ERRS, WLWMI_PS_PHY_ERR_DROP, WLWMI_PS_MPDU_ERRS, WLWMI_PS_WORDS
};
_Static_assert(WLWMI_REQUEST_STATS_BYTES == 24 && WLWMI_SE_WORDS * 4 == WLWMI_STATS_EVENT_BYTES &&
    WLWMI_PS_WORDS * 4 == WLWMI_PDEV_STATS_BYTES && WLWMI_PS_COMP_QUEUED == 7 && WLWMI_PS_MID_PPDU_ROUTE_CHANGE == 30,
    "C2b WMI stats wire sizes and word order (wmi.h:5891-6014)");

/* REQUEST_STATS {stats id PDEV_STAT, vdev 0, peer MAC 0 + 2 pad, PDEV}. */
static inline int
wlwmi_encode_stats_req(uint32_t pdev, uint8_t *out, uint32_t room, uint32_t *len)
{
	const uint32_t w[5] = { WLWMI_REQUEST_PDEV_STAT, 0, 0, 0, pdev };
	return wlwmi_encode_words(WLWMI_CMD_REQUEST_STATS, WLWMI_TAG_REQUEST_STATS_CMD, w, 5, out, room, len);
}

/* What came back: the first usable event's fixed TLV and first pdev entry
 * (raw words) and the length of the byte array that held it, how many
 * events arrived, how many were unusable and why the last one was. */
struct wlstats {
	uint32_t events, unusable, bytes;
	const char *why;
	bool have;
	uint32_t ev[WLWMI_SE_WORDS], w[WLWMI_PS_WORDS];
};

static inline bool wlstats_known(uint32_t id) { return id == WLWMI_EV_UPDATE_STATS; }

/* UPDATE_STATS as ath12k_update_stats_event takes it for a PDEV_STAT request
 * (wmi.c:8197-8312, 8389-8434, 8488-8545): the TLVs as ath12k_wmi_tlv_iter
 * walks them (wmi.c:320-368: a TLV past the event ends it); the fixed TLV 0x46
 * must come before an ARRAY_BYTE (wmi.c:8210-8213; L19 has no length policy
 * for it, here it must hold the 44-byte struct); in each ARRAY_BYTE L19 reads
 * num_vdev_stats vdev entries, then num_bcn_stats beacon entries, then
 * num_pdev_stats pdev entries, each bounded by what is left
 * (wmi.c:8228-8312); the first pdev entry is the one its dump prints
 * (wmi.c:8041-8042). An event without one leaves L19 waiting
 * (fw_stats_done, wmi.c:8517-8522) and is unusable here too. Never fails: an
 * unusable event is counted and its reason kept; the first usable one stays. */
static inline void
wlstats_parse(struct wlstats *st, const uint8_t *p, uint32_t len)
{
	struct wlwmi_it it;
	const uint8_t *ev = 0, *pev = 0, *pdev = 0;
	const char *why = 0;
	uint32_t bytes = 0;
	int r = 0;
	st->events++;
	if (st->have) { return; }
	wlwmi_it_init(&it, p, len);
	while (!why && (r = wlwmi_next(&it)) > 0) {
		if (it.tag == WLWMI_TAG_STATS_EVENT) {
			if (it.tlen < WLWMI_STATS_EVENT_BYTES) { why = "stats fixed TLV shorter than 44 bytes"; } else { ev = it.v; }
			continue;
		}
		if (it.tag != WLWMI_TAG_ARRAY_BYTE) { continue; }
		if (!ev) { why = "stats byte array before the fixed TLV"; break; }
		const uint32_t n[3] = { wlce_get32(ev + 4 * WLWMI_SE_NUM_VDEV_STATS), wlce_get32(ev + 4 * WLWMI_SE_NUM_BCN_STATS),
		    wlce_get32(ev + 4 * WLWMI_SE_NUM_PDEV_STATS) };
		const uint32_t sz[3] = { WLWMI_VDEV_STATS_BYTES, WLWMI_BCN_STATS_BYTES, WLWMI_PDEV_STATS_BYTES };
		const uint8_t *d = it.v;
		uint32_t left = it.tlen;
		for (uint32_t k = 0; k < 3 && !why; k++) {
			for (uint32_t i = 0; i < n[k]; i++) {
				if (left < sz[k]) { why = "stats byte array shorter than its entries"; break; }
				if (k == 2 && !pdev) { pdev = d; pev = ev; bytes = it.tlen; }
				d += sz[k]; left -= sz[k];
			}
		}
	}
	if (!why && r < 0) { why = "stats TLV past the event"; }
	if (!why && !pdev) { why = ev ? "no pdev stats in the event" : "no stats fixed TLV"; }
	if (why) { st->unusable++; st->why = why; return; }
	for (uint32_t i = 0; i < WLWMI_SE_WORDS; i++) { st->ev[i] = wlce_get32(pev + 4 * i); }
	for (uint32_t i = 0; i < WLWMI_PS_WORDS; i++) { st->w[i] = wlce_get32(pdev + 4 * i); }
	st->bytes = bytes;
	st->have = true;
}

/* ---- C2c (fuxiwlanqrtr=7): W26, host RX rings, the WBM idle list and the chain masks ----
 * Adapted from Linux ath12k dp.c, dp.h, dp_rx.c, dp_tx.c, dp_mon.c, hal.c, hal.h,
 * hal_desc.h, hal_rx.c, hw.c, mac.c and wmi.c (BSD-3-Clause-Clear), v6.19. What
 * L19 gives the firmware before its first scan (dp_alloc core.c:1291 before
 * core_start core.c:1300; ring announcements from pdev_create core.c:1143, after
 * the HTT version exchange core.c:955), each behind its own w20 bit: 0x200 the
 * WBM idle list (ring 120, 32767 MSDU link descriptors; dp.c:759-876,
 * hal.c:1714-1806), 0x100 the refill ring with 4095 host buffers, the two
 * per-MAC rings and the RXDMA error ring, announced by HTT SRING_SETUP
 * (dp_rx.c:4610-4646, dp_tx.c:1159-1273), 0x400 PDEV_SET_PARAM TX/RX_CHAIN_MASK
 * (mac.c:8562-8590, 9418). No ring selection, REO, TCL, release or monitor ring,
 * no other pdev parameter. Ids, values and offsets are re-derived from L19 by the
 * host test. The glue allocates two arenas (Normal-NC, SMMU-mapped); only the
 * functions below lay them out and fill them, every device-written word is read
 * after a read barrier, every count is a host read of host memory, nothing
 * allocates and every index is bounded by its ring. */
#define WLRX_W20_BAND24 0x10u            /* the w20 bits of fuxi_wlan.c that W26 reads */
#define WLRX_W20_RXRING 0x100u
#define WLRX_W20_IDLELINK 0x200u
#define WLRX_W20_CHAINMASK 0x400u
#define WLRX_RING_IDLE 120u              /* HAL_SRNG_RING_ID_WBM_IDLE_LINK (hal.h:470) */
#define WLRX_RING_REFILL 160u            /* HAL_SRNG_SW2RXDMA_BUF0: RXDMA_BUF ring_num 0, mac 0 (dp_rx.c:4690-4693) */
#define WLRX_RINGS_PER_PMAC 8u           /* HAL_SRNG_RINGS_PER_PMAC = 192 - 184 (hal.h:497-523) */
#define WLRX_RING_MAC0 161u              /* RXDMA_BUF ring_num 1, mac 0 (dp_rx.c:4699-4711; hal.c:1421-1423, 1830-1832) */
#define WLRX_RING_MAC1 (WLRX_RING_MAC0 + WLRX_RINGS_PER_PMAC)
#define WLRX_RING_ERR 186u               /* HAL_SRNG_RING_ID_WMAC1_RXDMA2SW0: RXDMA_DST ring_num 0, mac 0 */
#define WLRX_DMAC_START 160u             /* HAL_SRNG_RING_ID_DMAC_CMN_ID_START: wrp index = id - 160 (hal.c:2393, 2427) */
#define WLRX_RDP_WORDS 207u              /* HAL_SRNG_RING_ID_MAX = 183 + 3 x 8 (hal.c:1564-1576) */
#define WLRX_WRP_WORDS 47u               /* 3 x 8 + (183 - 160) (hal.c:1592-1604) */
#define WLRX_REFILL_ENTRIES 4096u        /* DP_RXDMA_BUF_RING_SIZE (dp.h:189) */
#define WLRX_MAC_ENTRIES 2048u           /* DP_RX_MAC_BUF_RING_SIZE (dp.h:190) */
#define WLRX_ERR_ENTRIES 1024u           /* DP_RXDMA_ERR_DST_RING_SIZE (dp.h:192) */
#define WLRX_IDLE_ENTRIES 32768u         /* 18560 link descriptors rounded up (dp.c:766-784) */
#define WLRX_BUF_DW 2u                   /* sizeof(hal_wbm_buffer_ring) / 4 (hal_desc.h:2117-2119) */
#define WLRX_ERR_DW 8u                   /* sizeof(hal_reo_entrance_ring) / 4 (hal_desc.h:1051-1059) */
/* Error entry fields taken from the received frame, never stored or printed (counts only): w5
 * MPDU_BYTE_COUNT 21:8 (hal_desc.h:1033), w6 MPDU_FRAG_NUM 10:7 and MPDU_SEQ_NUM 30:19 (hal_desc.h:1039, 1044). */
#define WLRX_ERR_W5_FRAME 0x003fff00u
#define WLRX_ERR_W6_FRAME 0x7ff80780u
#define WLRX_IDLE_DW 2u                  /* sizeof(hal_wbm_link_desc) / 4 (hal_desc.h:1797-1799) */
#define WLRX_NBUF 4095u                  /* a fresh source ring takes entries - 1 (hal.c:2039-2042, dp_rx.c:332-336) */
#define WLRX_NDESC 32767u                /* get_next_entry stops at next hp == tp (hal.c:2077-2080) */
#define WLRX_BUF_BYTES 2048u             /* DP_RX_BUFFER_SIZE (dp.h:206): the posted half of a slot */
#define WLRX_SLOT_BYTES 4096u            /* slot stride: the upper 2048 bytes are a guard, never posted */
#define WLRX_SLOTS 4096u                 /* slots 0..4094 posted, slot 4095 never */
#define WLRX_SENTINEL 0xa5u
#define WLRX_SENTINEL_BYTES 64u
#define WLRX_LINK_DESC_BYTES 128u        /* HAL_LINK_DESC_SIZE and _ALIGN (hal.h:16-17) */
#define WLRX_RBM_IDLE_DESC 1u            /* HAL_RX_BUF_RBM_WBM_DEV0_IDLE_DESC_LIST (hal.h:809-816, dp.c:1717-1722) */
#define WLRX_RBM_SW1 6u                  /* HAL_RX_BUF_RBM_SW1_BM, WCN7850 rx_buf_rbm (hw.c:1291) */
#define WLRX_COOKIE_START 0x4000u        /* DP_LINK_DESC_START, _SHIFT 3: the link cookie's shape (dp.h:97-101) */
#define WLRX_COOKIE_SHIFT 3u
/* RX arena (0x100): rdp u32[207] and wrp u32[47] on the first page, the four
 * rings, 4096 slots of 4 KiB, a 128 KiB tail. Link arena (0x200): ring 120, then
 * 32768 link descriptor slots of 128 bytes (32767 listed). */
#define WLRX_ARENA_BYTES 0x1040000u
#define WLRX_OFF_RDP 0x0000000u
#define WLRX_OFF_WRP 0x0000400u
#define WLRX_OFF_REFILL 0x0004000u
#define WLRX_OFF_MAC0 0x000c000u
#define WLRX_OFF_MAC1 0x0010000u
#define WLRX_OFF_ERR 0x0014000u
#define WLRX_OFF_BUFS 0x0020000u
#define WLRX_OFF_TAIL 0x1020000u
#define WLRX_LINK_BYTES 0x440000u
#define WLRX_LINK_OFF_RING 0x000000u
#define WLRX_LINK_OFF_DESC 0x040000u
_Static_assert(WLRX_OFF_RDP + 4u * WLRX_RDP_WORDS <= WLRX_OFF_WRP && WLRX_OFF_WRP + 4u * WLRX_WRP_WORDS <= WLRX_OFF_REFILL &&
    WLRX_OFF_REFILL + 4u * WLRX_BUF_DW * WLRX_REFILL_ENTRIES <= WLRX_OFF_MAC0 &&
    WLRX_OFF_MAC0 + 4u * WLRX_BUF_DW * WLRX_MAC_ENTRIES <= WLRX_OFF_MAC1 &&
    WLRX_OFF_MAC1 + 4u * WLRX_BUF_DW * WLRX_MAC_ENTRIES <= WLRX_OFF_ERR &&
    WLRX_OFF_ERR + 4u * WLRX_ERR_DW * WLRX_ERR_ENTRIES <= WLRX_OFF_BUFS &&
    WLRX_OFF_BUFS + WLRX_SLOTS * WLRX_SLOT_BYTES == WLRX_OFF_TAIL && WLRX_OFF_TAIL + 0x20000u == WLRX_ARENA_BYTES &&
    WLRX_SLOT_BYTES == 0x1000u && 2u * WLRX_BUF_BYTES == WLRX_SLOT_BYTES && WLRX_NBUF == WLRX_SLOTS - 1u &&
    !(WLRX_OFF_BUFS & 0xfffu) && WLRX_RDP_WORDS == 183u + 3u * WLRX_RINGS_PER_PMAC &&
    WLRX_WRP_WORDS == 3u * WLRX_RINGS_PER_PMAC + 183u - WLRX_DMAC_START && WLRX_RING_ERR < WLRX_RDP_WORDS,
    "W26 RX arena layout");
_Static_assert(WLRX_LINK_OFF_RING + 4u * WLRX_IDLE_DW * WLRX_IDLE_ENTRIES <= WLRX_LINK_OFF_DESC &&
    WLRX_LINK_OFF_DESC + WLRX_IDLE_ENTRIES * WLRX_LINK_DESC_BYTES == WLRX_LINK_BYTES && WLRX_NDESC == WLRX_IDLE_ENTRIES - 1u &&
    (((WLRX_NDESC - 1u) + WLRX_COOKIE_START) << WLRX_COOKIE_SHIFT) <= 0xfffffu, "W26 link arena layout, 20-bit cookie");
/* HTT SRING_SETUP (msg 0xb, struct htt_srng_setup_cmd 13 words; dp.h:470-478,
 * 493-513, 642-676): ring ids HOST1_TO_FW_RXBUF 5, RXDMA_HOST_BUF 0,
 * NON_MONITOR_DEST 7; types HW_TO_SW 0, SW_TO_HW 1, SW_TO_SW 2. Thresholds of
 * ath12k_dp_srng_setup (dp.c:270-320, hal.h:762-769): RXDMA_BUF batch 0, timer
 * 500 us, low threshold entries >> 3 (kept in words, hal.c:2379-2380);
 * RXDMA_DST and WBM_IDLE_LINK batch 1, timer 256 us. */
#define WLRX_HTT_SRING_SETUP 0xbu
#define WLRX_HTT_RING_RXDMA_HOST_BUF 0u
#define WLRX_HTT_RING_HOST1_TO_FW_RXBUF 5u
#define WLRX_HTT_RING_NON_MONITOR_DEST 7u
#define WLRX_HTT_HW_TO_SW 0u
#define WLRX_HTT_SW_TO_HW 1u
#define WLRX_HTT_SW_TO_SW 2u
#define WLRX_HTT_LOOP_CNT_DIS (1u << 25)
#define WLRX_HTT_RESPONSE_REQUIRED (1u << 19)   /* never set: L19 defines no T2H answer for it (dp.h:637-639, 1338-1352) */
#define WLRX_SRING_WORDS 13u
#define WLRX_TIMER_RX_US 500u
#define WLRX_TIMER_OTHER_US 256u
#define WLRX_BATCH_OTHER 1u
/* WCN7850 WBM (hw.c:1194-1216, hal.h:59, 94-122, 222-225, 242, 287-298, 316,
 * 346): ring 120's R0 at the UMAC WBM block + hal_wbm_idle_ring_base_lsb, the
 * TCL1 R0 layout (BASE_MSB 4, ID 8, MISC 0x10, CONSUMER_INT_SETUP IX0 0x30, IX1
 * 0x34, as WLCE_R0_*), R2 HP at + 0x30b8, TP 4 above; the idle list control and
 * size registers and the scattered ring base (read only: scatter path). */
#define WLRX_WBM_REG 0x00a34000u
#define WLRX_WBM_IDLE_R0 (WLRX_WBM_REG + 0xd3cu)
#define WLRX_WBM_IDLE_HP (WLRX_WBM_REG + 0x30b8u)
#define WLRX_WBM_IDLE_TP (WLRX_WBM_IDLE_HP + 4u)
#define WLRX_WBM_LIST_CNTL (WLRX_WBM_REG + 0x240u)
#define WLRX_WBM_LIST_SIZE (WLRX_WBM_REG + 0x244u)
#define WLRX_WBM_SCAT_LSB (WLRX_WBM_REG + 0x250u)
#define WLRX_WBM_SCAT_MSB (WLRX_WBM_REG + 0x254u)
#define WLRX_WBM_LIST_MODE (1u << 1)     /* HAL_WBM_LINK_DESC_IDLE_LIST_MODE: scatter (hal.h:346) */
#define WLRX_MISC_RING_ID_DISABLE (1u << 0)
#define WLRX_WBM_MISC (WLRX_MISC_RING_ID_DISABLE | WLCE_MISC_LOOPCNT_DIS | WLCE_MISC_SRNG_ENABLE)
#define WLRX_IDLE_REGS 8u
/* PDEV_SET_PARAM (wmi.h:63, 245, 322-324, 934-936, 1296, 3279-3284; wmi.c:1620-1650). */
#define WLWMI_CMD_PDEV_SET_PARAM 0x4003u
#define WLWMI_TAG_PDEV_SET_PARAM_CMD 0x52u
#define WLWMI_PDEV_PARAM_TX_CHAIN_MASK 0x1u
#define WLWMI_PDEV_PARAM_RX_CHAIN_MASK 0x2u
#define WLWMI_PDEV_SET_PARAM_BYTES 16u
_Static_assert(WLRX_SRING_WORDS * 4u == 52u && WLWMI_PDEV_SET_PARAM_BYTES == 16u && WLRX_WBM_MISC == 0x43u &&
    WLRX_WBM_IDLE_R0 >> 19 == 0x14u && WLRX_WBM_IDLE_TP >> 19 == 0x14u && WLRX_WBM_LIST_CNTL >> 19 == 0x14u,
    "W26 wire sizes and one window for every WBM register");

/* A device-written word of the RX arena is one 32-bit load; the host model wraps it. A host
 * pointer publish into the arena (wrp[0]) likewise. */
#ifndef WLRX_LOAD
#define WLRX_LOAD(p) (*(p))
#endif
#ifndef WLRX_PUBLISH
#define WLRX_PUBLISH(p, v) (*(p) = (v))
#endif

enum { WLRX_R_REFILL, WLRX_R_MAC0, WLRX_R_MAC1, WLRX_R_ERR, WLRX_NRING };
enum { WLRX_P_SETUP, WLRX_P_STARTED, WLRX_P_MID, WLRX_P_COMPLETED, WLRX_NPOINT };
/* watch field bits (bad, unreadable, exhausted) */
#define WLRX_F_REFILL 0x01u
#define WLRX_F_MAC0 0x02u
#define WLRX_F_MAC1 0x04u
#define WLRX_F_ERR 0x08u
#define WLRX_F_IDLE 0x10u
/* the pre-reads, in this order, before any WBM write */
enum { WLRX_PRE_BASE_LSB, WLRX_PRE_BASE_MSB, WLRX_PRE_MISC, WLRX_PRE_R2_HP, WLRX_PRE_R2_TP, WLRX_PRE_LIST_CNTL,
    WLRX_PRE_LIST_SIZE, WLRX_PRE_SCAT_LSB, WLRX_PRE_SCAT_MSB, WLRX_PRE_N };
/* their reading: free, or why nothing is written (D1: scatter mode with base 0 and SRNG_ENABLE 0 is
 * ambiguous, not a firmware-owned list) */
enum { WLRX_IDLE_FREE, WLRX_IDLE_SRNG, WLRX_IDLE_BASE, WLRX_IDLE_AMBIGUOUS, WLRX_IDLE_UNREADABLE };

struct wlrx_ring {              /* one HAL SRNG the host lays out */
	uint16_t id;                /* HAL ring id: 160, 161, 169, 186, 120 */
	uint8_t src;                /* 1 L19 SRC ring (hp in wrp / R2), 0 DST (hp in rdp) */
	uint8_t entry_dw;           /* 2 RXDMA_BUF and WBM_IDLE_LINK, 8 RXDMA_DST */
	uint32_t entries, off;      /* entries; offset inside its arena */
	uint32_t mac, htt_ring, htt_type, batch, timer_us;   /* SRING_SETUP fields (L19 mac_id, not the pdev field) */
	bool low;                   /* LOW_THRESH_INTR_EN */
	volatile uint32_t *va;
	uint64_t iova, hp_iova, tp_iova;   /* remote pointer words; 0 for ring 120 */
};
struct wlrx_watch {             /* raw device-side words in dwords; ~0u = unreadable */
	uint32_t refill_tp, mac_hp[2], mac_tp[2], err_hp, idle_hp, idle_tp;
	uint32_t refill_taken, refill_left, mac_avail[2], err_fill, idle_avail;   /* derived, in entries */
	uint32_t foreign;           /* the scan's FOREIGN_CHAN count at this watch */
	int32_t kept;               /* refill taken - entries the firmware wrote to the MAC rings (hp / 2) */
	uint8_t bad, unreadable, exhausted;   /* WLRX_F_* */
	bool taken, kept_valid, refill_empty;
};
struct wlrx_content {
	uint32_t mac_entries[2], mac_valid[2], mac_foreign[2], refill_taken;
	int32_t kept;
	uint32_t mac_rbm_other[2], mac_cookie_other[2];            /* among valid: printed only */
	uint32_t bufs_written;                                     /* sentinel changed in bytes 0..63 */
	uint32_t guard_upper, guard_upper_slots, guard_last, guard_tail;   /* non-zero dwords */
	uint32_t guard_gap;         /* non-zero dwords where the device was given no address (2.7 "none") */
	uint32_t err_hp, err_scanned, err_entries, err_push[4], err_code[20], err_rbm_link, err_rbm_buf, err_rbm_other;
	uint32_t err_raw[4][4];     /* w1, w5, w6, w7 of the first four entries, frame fields masked */
	bool err_bad;
	uint32_t link_written, link_unlisted, idle_changed;
};
struct wlrx {
	uint8_t *base; uint64_t iova; uint32_t bytes;                /* RX arena, 0x1040000 */
	volatile uint32_t *rdp, *wrp; uint64_t rdp_iova, wrp_iova;   /* u32[207], u32[47] */
	struct wlrx_ring ring[WLRX_NRING];                          /* refill, mac0, mac1, err */
	uint8_t *bufs; uint64_t bufs_iova; uint32_t nbufs;          /* 4095 posted of 4096 slots, stride 4096 */
	uint8_t *link; uint64_t link_iova; uint32_t link_bytes;     /* link arena 0x440000 */
	struct wlrx_ring idle; uint32_t ndesc;                      /* 32767 */
	uint32_t wbm_pre[WLRX_PRE_N];                               /* D1: seven pre-reads + the scattered base */
	unsigned idle_own;                                          /* WLRX_IDLE_* of the pre-reads */
	uint32_t chain_tx, chain_rx;                                /* sent values; 0 = not sent */
	struct wlrx_watch w[WLRX_NPOINT];                           /* setup, started, mid, completed */
	struct wlrx_content c;
	uint32_t htt_t2h[64], htt_t2h_other, sring_sent;
	uint32_t tx_ci, ci_rx_frame_max, stats_answers, tx_stats[5];   /* D4 / D5 notes from the C2b readouts */
	bool rx_on, idle_on, idle_tried, chain_on, stopping;
};

static inline uint32_t
wlrx_cookie(uint32_t i)
{
	return ((WLRX_NDESC - 1u - i) + WLRX_COOKIE_START) << WLRX_COOKIE_SHIFT;  /* id 32766 - i, page 0 */
}

/* struct ath12k_buffer_addr (hal_desc.h:11-20): address 31:0; address 39:32 in
 * 7:0, return buffer manager 11:8, cookie 31:12 of word 1 (hal_rx.c:268-279,
 * hal.c:1959-1970). Both cookies are ours: the slot index for buffers, the
 * DP_LINK_DESC_COOKIE_SET shape for links. */
static inline void
wlrx_addr_entry(volatile uint32_t *e, uint64_t addr, uint32_t rbm, uint32_t cookie)
{
	e[0] = (uint32_t)addr;
	e[1] = ((uint32_t)(addr >> 32) & 0xffu) | ((rbm & 0xfu) << 8) | ((cookie & 0xfffffu) << 12);
}

/* Lay out the RX arena (2.7): rdp, wrp, the rings with their remote pointer
 * words (an L19 SRC ring has its HP in wrp[id - 160] and its TP in rdp[id], a
 * DST ring the reverse; hal.c:1881-1911, 2374-2431) and the slots. */
static inline int
wlrx_layout(struct wlrx *rx, uint8_t *base, uint64_t iova, uint32_t bytes)
{
	static const struct { uint16_t id; uint8_t src, dw; uint32_t entries, off, mac, ring, type, batch, timer; bool low; }
	    plan[WLRX_NRING] = {
		{ WLRX_RING_REFILL, 1, WLRX_BUF_DW, WLRX_REFILL_ENTRIES, WLRX_OFF_REFILL, 0, WLRX_HTT_RING_HOST1_TO_FW_RXBUF,
		  WLRX_HTT_SW_TO_SW, 0, WLRX_TIMER_RX_US, true },
		{ WLRX_RING_MAC0, 1, WLRX_BUF_DW, WLRX_MAC_ENTRIES, WLRX_OFF_MAC0, 0, WLRX_HTT_RING_RXDMA_HOST_BUF,
		  WLRX_HTT_SW_TO_HW, 0, WLRX_TIMER_RX_US, true },
		{ WLRX_RING_MAC1, 1, WLRX_BUF_DW, WLRX_MAC_ENTRIES, WLRX_OFF_MAC1, 1, WLRX_HTT_RING_RXDMA_HOST_BUF,
		  WLRX_HTT_SW_TO_HW, 0, WLRX_TIMER_RX_US, true },
		{ WLRX_RING_ERR, 0, WLRX_ERR_DW, WLRX_ERR_ENTRIES, WLRX_OFF_ERR, 0, WLRX_HTT_RING_NON_MONITOR_DEST,
		  WLRX_HTT_HW_TO_SW, WLRX_BATCH_OTHER, WLRX_TIMER_OTHER_US, false },
	};
	if (!base || (iova & 0xfffu) || bytes < WLRX_ARENA_BYTES || iova + WLRX_ARENA_BYTES > (1ull << 32)) { return 1; }
	rx->base = base; rx->iova = iova; rx->bytes = WLRX_ARENA_BYTES;
	rx->rdp = (volatile uint32_t *)(void *)(base + WLRX_OFF_RDP); rx->rdp_iova = iova + WLRX_OFF_RDP;
	rx->wrp = (volatile uint32_t *)(void *)(base + WLRX_OFF_WRP); rx->wrp_iova = iova + WLRX_OFF_WRP;
	for (unsigned i = 0; i < WLRX_NRING; i++) {
		struct wlrx_ring *r = &rx->ring[i];
		uint64_t rdp = rx->rdp_iova + 4ull * plan[i].id, wrp = rx->wrp_iova + 4ull * (plan[i].id - WLRX_DMAC_START);
		r->id = plan[i].id; r->src = plan[i].src; r->entry_dw = plan[i].dw; r->entries = plan[i].entries;
		r->off = plan[i].off; r->mac = plan[i].mac; r->htt_ring = plan[i].ring; r->htt_type = plan[i].type;
		r->batch = plan[i].batch; r->timer_us = plan[i].timer; r->low = plan[i].low;
		r->va = (volatile uint32_t *)(void *)(base + plan[i].off);
		r->iova = iova + plan[i].off;
		r->hp_iova = r->src ? wrp : rdp;
		r->tp_iova = r->src ? rdp : wrp;
	}
	rx->bufs = base + WLRX_OFF_BUFS; rx->bufs_iova = iova + WLRX_OFF_BUFS; rx->nbufs = WLRX_NBUF;
	return 0;
}

/* Host writes, all before the first SRING_SETUP: the whole arena zeroed, the
 * 0xa5 sentinel over bytes 0..63 of each posted slot, 4095 refill entries (slot
 * start, RBM 6, cookie = slot). The HP follows with wlrx_publish_refill. */
static inline int
wlrx_fill_refill(struct wlrx *rx)
{
	struct wlrx_ring *r = &rx->ring[WLRX_R_REFILL];
	if (rx->stopping || !rx->base) { return 1; }
	for (uint32_t i = 0; i < rx->bytes / 4u; i++) { ((volatile uint32_t *)(void *)rx->base)[i] = 0; }
	for (uint32_t k = 0; k < WLRX_NBUF; k++) {
		volatile uint8_t *s = rx->bufs + (size_t)k * WLRX_SLOT_BYTES;
		for (uint32_t j = 0; j < WLRX_SENTINEL_BYTES; j++) { s[j] = WLRX_SENTINEL; }
		wlrx_addr_entry(&r->va[WLRX_BUF_DW * k], rx->bufs_iova + (uint64_t)k * WLRX_SLOT_BYTES, WLRX_RBM_SW1, k);
	}
	return 0;
}

/* L19 access_end for an LMAC source ring: dma_wmb, then the HP into shared
 * memory (hal.c:2173-2184): wrp[0] = 4095 entries in words. */
static inline int
wlrx_publish_refill(struct wlrx *rx, const struct wlce_io *io)
{
	if (rx->stopping || !rx->base) { return 1; }
	io->wbarrier(io->ctx);
	WLRX_PUBLISH(&rx->wrp[WLRX_RING_REFILL - WLRX_DMAC_START], WLRX_NBUF * WLRX_BUF_DW);
	return 0;
}

/* HTT SRING_SETUP as ath12k_dp_tx_htt_srng_setup builds it (dp_tx.c:1159-1273):
 * pdev = mac for SW_TO_SW, DP_SW2HW_MACID(mac) = mac + 1 otherwise; ring size in
 * words, entry words, LOOP_CNT_DIS for SW_TO_HW only; MSI 0 (no ring of C2c is in
 * an MSI group, dp.c:179-223); batch x entry words, timer >> 3; the low threshold
 * only with LOW_THRESH_INTR_EN; RESPONSE_REQUIRED never. */
static inline void
wlrx_sring_words(const struct wlrx *rx, unsigned i, uint32_t *w)
{
	const struct wlrx_ring *r = &rx->ring[i < WLRX_NRING ? i : 0];
	uint32_t pdev = r->htt_type == WLRX_HTT_SW_TO_SW ? r->mac : r->mac + 1u;
	for (uint32_t k = 0; k < WLRX_SRING_WORDS; k++) { w[k] = 0; }
	w[0] = WLRX_HTT_SRING_SETUP | (pdev & 0xffu) << 8 | (r->htt_ring & 0xffu) << 16 | (r->htt_type & 0xffu) << 24;
	w[1] = (uint32_t)r->iova; w[2] = (uint32_t)(r->iova >> 32);
	w[3] = ((r->entries * r->entry_dw) & 0xffffu) | (uint32_t)r->entry_dw << 16 |
	    (r->htt_type == WLRX_HTT_SW_TO_HW ? WLRX_HTT_LOOP_CNT_DIS : 0u);
	w[4] = (uint32_t)r->hp_iova; w[5] = (uint32_t)(r->hp_iova >> 32);
	w[6] = (uint32_t)r->tp_iova; w[7] = (uint32_t)(r->tp_iova >> 32);
	w[11] = ((r->batch * r->entry_dw) & 0x7fffu) | ((r->timer_us >> 3) & 0xffffu) << 16;
	w[12] = r->low ? ((r->entries >> 3) * r->entry_dw) & 0xffffu : 0u;
}

/* Lay out and zero the link arena: ring 120 (a UMAC SRC ring, R2 pointers, no
 * TP_ADDR) and the 128-byte descriptor slots. */
static inline int
wlrx_link_layout(struct wlrx *rx, uint8_t *base, uint64_t iova, uint32_t bytes)
{
	struct wlrx_ring *r = &rx->idle;
	if (!base || (iova & 0xfffu) || bytes < WLRX_LINK_BYTES || iova + WLRX_LINK_BYTES > (1ull << 32)) { return 1; }
	rx->link = base; rx->link_iova = iova; rx->link_bytes = WLRX_LINK_BYTES; rx->ndesc = WLRX_NDESC;
	r->id = WLRX_RING_IDLE; r->src = 1; r->entry_dw = WLRX_IDLE_DW; r->entries = WLRX_IDLE_ENTRIES;
	r->off = WLRX_LINK_OFF_RING; r->batch = WLRX_BATCH_OTHER; r->timer_us = WLRX_TIMER_OTHER_US; r->low = false;
	r->va = (volatile uint32_t *)(void *)(base + WLRX_LINK_OFF_RING); r->iova = iova + WLRX_LINK_OFF_RING;
	r->hp_iova = r->tp_iova = 0;
	for (uint32_t i = 0; i < WLRX_LINK_BYTES / 4u; i++) { ((volatile uint32_t *)(void *)base)[i] = 0; }
	return 0;
}

/* The idle list entry I as the host writes it: descriptor slot I, RBM 1, our
 * cookie (2.6.3); entry 32767 stays 0. */
static inline void
wlrx_link_expect(const struct wlrx *rx, uint32_t i, uint32_t *e)
{
	volatile uint32_t v[2] = { 0, 0 };
	if (i < WLRX_NDESC) {
		wlrx_addr_entry(v, rx->link_iova + WLRX_LINK_OFF_DESC + (uint64_t)i * WLRX_LINK_DESC_BYTES, WLRX_RBM_IDLE_DESC,
		    wlrx_cookie(i));
	}
	e[0] = v[0]; e[1] = v[1];
}

static inline int
wlrx_link_fill(struct wlrx *rx)
{
	if (rx->stopping || !rx->link) { return 1; }
	for (uint32_t i = 0; i < WLRX_NDESC; i++) {
		uint32_t e[2];
		wlrx_link_expect(rx, i, e);
		rx->idle.va[WLRX_IDLE_DW * i] = e[0];
		rx->idle.va[WLRX_IDLE_DW * i + 1u] = e[1];
	}
	return 0;
}

static inline uint32_t
wlrx_pre_addr(unsigned i)
{
	static const uint32_t a[WLRX_PRE_N] = {
		WLRX_WBM_IDLE_R0 + WLCE_R0_BASE_LSB, WLRX_WBM_IDLE_R0 + WLCE_R0_BASE_MSB, WLRX_WBM_IDLE_R0 + WLCE_R0_MISC,
		WLRX_WBM_IDLE_HP, WLRX_WBM_IDLE_TP, WLRX_WBM_LIST_CNTL, WLRX_WBM_LIST_SIZE, WLRX_WBM_SCAT_LSB, WLRX_WBM_SCAT_MSB,
	};
	return i < WLRX_PRE_N ? a[i] : 0u;
}

/* Ring 120's eight writes in L19's order (hal.c:1714-1806): BASE_LSB, BASE_MSB
 * (address 39:32, ring size in words 27:8), ID (entry words), IX0 (timer 256 raw
 * 31:16, batch 1 x entry words 14:0), IX1 0 (no low threshold), R2 HP 0, R2 TP
 * 0, MISC RING_ID_DISABLE | LOOPCNT_DISABLE | SRNG_ENABLE. No MSI registers (no
 * MSI group), no TP_ADDR for ring 120 (hal.c:1724, 1771). */
static inline void
wlrx_idle_regs(const struct wlrx *rx, uint32_t regs[WLRX_IDLE_REGS][2])
{
	const struct wlrx_ring *r = &rx->idle;
	const uint32_t words = r->entries * r->entry_dw;
	const uint32_t v[WLRX_IDLE_REGS][2] = {
		{ WLRX_WBM_IDLE_R0 + WLCE_R0_BASE_LSB, (uint32_t)r->iova },
		{ WLRX_WBM_IDLE_R0 + WLCE_R0_BASE_MSB, ((uint32_t)(r->iova >> 32) & 0xffu) | (words & 0xfffffu) << 8 },
		{ WLRX_WBM_IDLE_R0 + WLCE_R0_ID, r->entry_dw },
		{ WLRX_WBM_IDLE_R0 + WLCE_R0_CONS_IX0, (r->timer_us & 0xffffu) << 16 | ((r->batch * r->entry_dw) & 0x7fffu) },
		{ WLRX_WBM_IDLE_R0 + WLCE_R0_CONS_IX1, 0u },
		{ WLRX_WBM_IDLE_HP, 0u },
		{ WLRX_WBM_IDLE_TP, 0u },
		{ WLRX_WBM_IDLE_R0 + WLCE_R0_MISC, WLRX_WBM_MISC },
	};
	for (unsigned i = 0; i < WLRX_IDLE_REGS; i++) { regs[i][0] = v[i][0]; regs[i][1] = v[i][1]; }
}

/* The WBM accessor's allowlist, nothing else in the WBM, REO or TCL blocks:
 * reads of the nine pre-read addresses (which hold the readback and watch
 * registers too), writes of ring 120's eight. */
static inline bool
wlrx_wbm_ok(uint32_t off, bool wr)
{
	static const uint32_t wa[WLRX_IDLE_REGS] = {
		WLRX_WBM_IDLE_R0 + WLCE_R0_BASE_LSB, WLRX_WBM_IDLE_R0 + WLCE_R0_BASE_MSB, WLRX_WBM_IDLE_R0 + WLCE_R0_ID,
		WLRX_WBM_IDLE_R0 + WLCE_R0_CONS_IX0, WLRX_WBM_IDLE_R0 + WLCE_R0_CONS_IX1, WLRX_WBM_IDLE_HP, WLRX_WBM_IDLE_TP,
		WLRX_WBM_IDLE_R0 + WLCE_R0_MISC,
	};
	for (unsigned i = 0; wr && i < WLRX_IDLE_REGS; i++) { if (off == wa[i]) { return true; } }
	for (unsigned i = 0; !wr && i < WLRX_PRE_N; i++) { if (off == wlrx_pre_addr(i)) { return true; } }
	return false;
}

/* The pre-reads (B5, D1): any ~0u = ownership not established; SRNG_ENABLE or a
 * non-zero ring base = the firmware owns the list; scatter mode with base 0 and
 * SRNG_ENABLE 0 = ambiguous. Anything but FREE: no WBM write at all. */
static inline unsigned
wlrx_idle_owned(const uint32_t *pre)
{
	for (unsigned i = 0; i < WLRX_PRE_N; i++) { if (pre[i] == ~0u) { return WLRX_IDLE_UNREADABLE; } }
	if (pre[WLRX_PRE_MISC] & WLCE_MISC_SRNG_ENABLE) { return WLRX_IDLE_SRNG; }
	if (pre[WLRX_PRE_BASE_LSB] || (pre[WLRX_PRE_BASE_MSB] & 0xffu)) { return WLRX_IDLE_BASE; }
	if (pre[WLRX_PRE_LIST_CNTL] & WLRX_WBM_LIST_MODE) { return WLRX_IDLE_AMBIGUOUS; }
	return WLRX_IDLE_FREE;
}

static inline const char *
wlrx_idle_why(unsigned own)
{
	switch (own) {
	case WLRX_IDLE_SRNG: return "SRNG_ENABLE set (firmware owns the list)";
	case WLRX_IDLE_BASE: return "base non-zero (firmware owns the list)";
	case WLRX_IDLE_AMBIGUOUS: return "scatter mode set with base 0 and SRNG_ENABLE 0 (AMBIGUOUS)";
	case WLRX_IDLE_UNREADABLE: return "a pre-read is ~0 (unreadable, ownership not established)";
	default: return "free";
	}
}

/* The device-side pointer words of the four rings (0x100), each followed by the
 * read barrier (L19's dma_rmb, hal.c:2144-2164). Ring 120's R2 words come
 * through the glue's WBM accessor. */
static inline void
wlrx_watch_read(const struct wlrx *rx, const struct wlce_io *io, struct wlrx_watch *w)
{
	w->refill_tp = WLRX_LOAD(&rx->rdp[WLRX_RING_REFILL]); io->rbarrier(io->ctx);
	for (unsigned m = 0; m < 2; m++) {
		const struct wlrx_ring *r = &rx->ring[WLRX_R_MAC0 + m];
		w->mac_hp[m] = WLRX_LOAD(&rx->wrp[r->id - WLRX_DMAC_START]); io->rbarrier(io->ctx);
		w->mac_tp[m] = WLRX_LOAD(&rx->rdp[r->id]); io->rbarrier(io->ctx);
	}
	w->err_hp = WLRX_LOAD(&rx->rdp[WLRX_RING_ERR]); io->rbarrier(io->ctx);
}

/* 0 = a pointer of a ring of SIZE words with entries of DW words; 1 = bad (not a
 * multiple of the entry or not below the ring); 2 = unreadable (~0u). */
static inline unsigned
wlrx_word(uint32_t v, uint32_t size, uint32_t dw)
{
	return v == ~0u ? 2u : (v >= size || v % dw) ? 1u : 0u;
}

/* Remaining entries and exhaustion (2.8, D2): refill taken tp / 2, left ((8190 -
 * tp) mod 8192) / 2; macN avail ((hp - tp) mod 4096) / 2, exhausted at 0 once
 * its tp moved; err fill hp / 8, exhausted from 1023; idle avail ((HP - TP) mod
 * 65536) / 2, exhausted at 0. Refill left 0 is exhaustion only with an exhausted
 * MAC ring (avail 0 once its tp moved: a ring the firmware never used is not one)
 * or kept above the setup watch's; else it is the refill-empty note. */
static inline void
wlrx_watch_derive(struct wlrx *rx, unsigned p)
{
	struct wlrx_watch *w = &rx->w[p < WLRX_NPOINT ? p : 0];
	const struct wlrx_watch *s = &rx->w[WLRX_P_SETUP];
	const uint32_t rsize = WLRX_REFILL_ENTRIES * WLRX_BUF_DW, msize = WLRX_MAC_ENTRIES * WLRX_BUF_DW;
	const uint32_t esize = WLRX_ERR_ENTRIES * WLRX_ERR_DW, isize = WLRX_IDLE_ENTRIES * WLRX_IDLE_DW;
	w->bad = w->unreadable = w->exhausted = 0;
	w->refill_taken = w->refill_left = w->err_fill = w->idle_avail = w->mac_avail[0] = w->mac_avail[1] = 0;
	w->kept = 0; w->kept_valid = w->refill_empty = false;
	if (rx->rx_on) {
		const uint32_t f[4] = { WLRX_F_REFILL, WLRX_F_MAC0, WLRX_F_MAC1, WLRX_F_ERR };
		unsigned k = wlrx_word(w->refill_tp, rsize, WLRX_BUF_DW);
		bool mac_ok[2];
		if (k) { w->bad |= k == 1 ? f[0] : 0u; w->unreadable |= k == 2 ? f[0] : 0u; }
		else { w->refill_taken = w->refill_tp / WLRX_BUF_DW; w->refill_left = (WLRX_NBUF * WLRX_BUF_DW + rsize - w->refill_tp) % rsize / WLRX_BUF_DW; }
		for (unsigned m = 0; m < 2; m++) {
			unsigned a = wlrx_word(w->mac_hp[m], msize, WLRX_BUF_DW), b = wlrx_word(w->mac_tp[m], msize, WLRX_BUF_DW);
			mac_ok[m] = !a && !b;
			if (a == 2 || b == 2) { w->unreadable |= f[1 + m]; } else if (a || b) { w->bad |= f[1 + m]; }
			if (!mac_ok[m]) { continue; }
			w->mac_avail[m] = (w->mac_hp[m] + msize - w->mac_tp[m]) % msize / WLRX_BUF_DW;
			if (!w->mac_avail[m] && w->mac_tp[m]) { w->exhausted |= f[1 + m]; }
		}
		k = wlrx_word(w->err_hp, esize, WLRX_ERR_DW);
		if (k) { w->bad |= k == 1 ? f[3] : 0u; w->unreadable |= k == 2 ? f[3] : 0u; }
		else { w->err_fill = w->err_hp / WLRX_ERR_DW; if (w->err_fill >= WLRX_ERR_ENTRIES - 1u) { w->exhausted |= f[3]; } }
		if (!(w->bad & f[0]) && !(w->unreadable & f[0]) && mac_ok[0] && mac_ok[1]) {
			w->kept = (int32_t)(w->refill_taken - w->mac_hp[0] / WLRX_BUF_DW - w->mac_hp[1] / WLRX_BUF_DW);
			w->kept_valid = true;
		}
		if (!((w->bad | w->unreadable) & f[0]) && !w->refill_left) {
			if ((w->exhausted & (f[1] | f[2])) || (w->kept_valid && s->kept_valid && w->kept > s->kept)) { w->exhausted |= f[0]; }
			else { w->refill_empty = true; }
		}
	}
	if (rx->idle_on) {
		unsigned a = wlrx_word(w->idle_hp, isize, WLRX_IDLE_DW), b = wlrx_word(w->idle_tp, isize, WLRX_IDLE_DW);
		if (a == 2 || b == 2) { w->unreadable |= WLRX_F_IDLE; }
		else if (a || b) { w->bad |= WLRX_F_IDLE; }
		else {
			w->idle_avail = (w->idle_hp + isize - w->idle_tp) % isize / WLRX_IDLE_DW;
			if (!w->idle_avail) { w->exhausted |= WLRX_F_IDLE; }
		}
	}
	w->taken = true;
}

/* Guard words (B1): non-zero dwords in bytes 2048..4095 of slots 0..4094 (and
 * in how many slots), in slot 4095 and in the 128 KiB tail; and the gaps the
 * device was given no address in (2.7 "none"): the first 16 KiB except the
 * pointer words SRING_SETUP named (rdp 160/161/169/186, wrp 0/1/9/26), and the
 * 16 KiB after the error ring. Counts only. */
static inline void
wlrx_guard_count(const struct wlrx *rx, uint32_t *upper, uint32_t *upper_slots, uint32_t *last, uint32_t *tail, uint32_t *gap)
{
	const volatile uint32_t *a = (const volatile uint32_t *)(const void *)rx->base;
	*upper = *upper_slots = *last = *tail = *gap = 0;
	for (uint32_t i = 0; i < WLRX_OFF_REFILL / 4u; i++) {
		const uint32_t k = i - WLRX_OFF_WRP / 4u;   /* the wrp index for i at or past wrp[0] */
		const bool named = i == WLRX_RING_REFILL || i == WLRX_RING_MAC0 || i == WLRX_RING_MAC1 || i == WLRX_RING_ERR ||
		    (i >= WLRX_OFF_WRP / 4u && (k == WLRX_RING_REFILL - WLRX_DMAC_START || k == WLRX_RING_MAC0 - WLRX_DMAC_START ||
		    k == WLRX_RING_MAC1 - WLRX_DMAC_START || k == WLRX_RING_ERR - WLRX_DMAC_START));
		*gap += !named && a[i] != 0;
	}
	for (uint32_t i = (WLRX_OFF_ERR + 4u * WLRX_ERR_DW * WLRX_ERR_ENTRIES) / 4u; i < WLRX_OFF_BUFS / 4u; i++) { *gap += a[i] != 0; }
	for (uint32_t k = 0; k < WLRX_SLOTS; k++) {
		const volatile uint32_t *s = (const volatile uint32_t *)(const void *)(rx->bufs + (size_t)k * WLRX_SLOT_BYTES);
		uint32_t from = k < WLRX_NBUF ? WLRX_BUF_BYTES / 4u : 0u, n = 0;
		for (uint32_t i = from; i < WLRX_SLOT_BYTES / 4u; i++) { n += s[i] != 0; }
		if (k < WLRX_NBUF) { *upper += n; *upper_slots += n != 0; } else { *last += n; }
	}
	for (uint32_t i = 0; i < (WLRX_ARENA_BYTES - WLRX_OFF_TAIL) / 4u; i++) {
		*tail += ((const volatile uint32_t *)(const void *)(rx->base + WLRX_OFF_TAIL))[i] != 0;
	}
}

/* The RX content after the scan (2.8): per-MAC entries split by address alone
 * into valid (the start of a posted slot) and foreign; among valid, RBM other
 * than 6 and cookie other than the slot, printed only; kept = refill taken -
 * valid; buffers whose sentinel changed; the guards; the RXDMA error ring
 * entries 0..min(fill, 1023) - 1 (all 1024 when its HP is bad, clamped to the
 * ring): push reason, error code and RBM histograms, the first four raw with
 * the frame's length, fragment and sequence numbers masked out. */
static inline void
wlrx_content(struct wlrx *rx, const struct wlce_io *io, uint32_t refill_taken)
{
	struct wlrx_content *c = &rx->c;
	const struct wlrx_ring *e = &rx->ring[WLRX_R_ERR];
	for (size_t i = 0; i < sizeof(*c); i++) { ((uint8_t *)c)[i] = 0; }
	io->rbarrier(io->ctx);
	for (unsigned m = 0; m < 2; m++) {
		const struct wlrx_ring *r = &rx->ring[WLRX_R_MAC0 + m];
		for (uint32_t i = 0; i < r->entries; i++) {
			uint32_t w0 = r->va[WLRX_BUF_DW * i], w1 = r->va[WLRX_BUF_DW * i + 1u];
			uint64_t a = w0 | (uint64_t)(w1 & 0xffu) << 32;
			if (!w0 && !w1) { continue; }
			c->mac_entries[m]++;
			if (a < rx->bufs_iova || (a - rx->bufs_iova) % WLRX_SLOT_BYTES || (a - rx->bufs_iova) / WLRX_SLOT_BYTES >= WLRX_NBUF) {
				c->mac_foreign[m]++;
				continue;
			}
			c->mac_valid[m]++;
			if (((w1 >> 8) & 0xfu) != WLRX_RBM_SW1) { c->mac_rbm_other[m]++; }
			if ((w1 >> 12) != (uint32_t)((a - rx->bufs_iova) / WLRX_SLOT_BYTES)) { c->mac_cookie_other[m]++; }
		}
	}
	c->refill_taken = refill_taken;
	c->kept = (int32_t)(refill_taken - c->mac_valid[0] - c->mac_valid[1]);
	for (uint32_t k = 0; k < WLRX_NBUF; k++) {
		const volatile uint8_t *s = rx->bufs + (size_t)k * WLRX_SLOT_BYTES;
		bool changed = false;
		for (uint32_t j = 0; j < WLRX_SENTINEL_BYTES && !changed; j++) { changed = s[j] != WLRX_SENTINEL; }
		c->bufs_written += changed;
	}
	wlrx_guard_count(rx, &c->guard_upper, &c->guard_upper_slots, &c->guard_last, &c->guard_tail, &c->guard_gap);
	c->err_hp = WLRX_LOAD(&rx->rdp[WLRX_RING_ERR]); io->rbarrier(io->ctx);
	c->err_bad = wlrx_word(c->err_hp, WLRX_ERR_ENTRIES * WLRX_ERR_DW, WLRX_ERR_DW) != 0;
	c->err_scanned = c->err_bad ? WLRX_ERR_ENTRIES : c->err_hp / WLRX_ERR_DW;  /* a usable HP is at most 1023 entries */
	for (uint32_t i = 0; i < c->err_scanned; i++) {
		const volatile uint32_t *d = &e->va[WLRX_ERR_DW * i];
		uint32_t w1 = d[1], w6 = d[6], rbm = (w1 >> 8) & 0xfu, code = (w6 >> 2) & 0x1fu, any = 0;
		for (uint32_t k = 0; k < WLRX_ERR_DW; k++) { any |= d[k]; }
		if (!any) { continue; }
		if (c->err_entries < 4u) {
			c->err_raw[c->err_entries][0] = w1; c->err_raw[c->err_entries][1] = d[5] & ~WLRX_ERR_W5_FRAME;
			c->err_raw[c->err_entries][2] = w6 & ~WLRX_ERR_W6_FRAME; c->err_raw[c->err_entries][3] = d[7];
		}
		c->err_entries++;
		c->err_push[w6 & 3u]++;
		c->err_code[code < 19u ? code : 19u]++;
		if (rbm == WLRX_RBM_IDLE_DESC) { c->err_rbm_link++; } else if (rbm == WLRX_RBM_SW1) { c->err_rbm_buf++; } else { c->err_rbm_other++; }
	}
}

/* Link content (B7): descriptors 0..32766 with any non-zero dword, the unlisted
 * slot 32767, ring 120 entries that differ from what the host wrote. */
static inline void
wlrx_link_content(struct wlrx *rx, const struct wlce_io *io)
{
	struct wlrx_content *c = &rx->c;
	c->link_written = c->link_unlisted = c->idle_changed = 0;
	io->rbarrier(io->ctx);
	for (uint32_t d = 0; d < WLRX_IDLE_ENTRIES; d++) {
		const volatile uint32_t *s = (const volatile uint32_t *)(const void *)(rx->link + WLRX_LINK_OFF_DESC +
		    (size_t)d * WLRX_LINK_DESC_BYTES);
		uint32_t e[2], any = 0;
		for (uint32_t k = 0; k < WLRX_LINK_DESC_BYTES / 4u; k++) { any |= s[k]; }
		if (d < WLRX_NDESC) { c->link_written += any != 0; } else { c->link_unlisted += any != 0; }
		wlrx_link_expect(rx, d, e);
		c->idle_changed += rx->idle.va[WLRX_IDLE_DW * d] != e[0] || rx->idle.va[WLRX_IDLE_DW * d + 1u] != e[1];
	}
}

/* HTT T2H messages by type (word 0 bits 7:0, dp_rx.c:1931, dp.h:1357). */
static inline void
wlrx_htt_count(struct wlrx *rx, const uint8_t *msg, uint32_t len)
{
	uint32_t t = len ? msg[0] : 0xffu;
	if (t < 64u) { rx->htt_t2h[t]++; } else { rx->htt_t2h_other++; }
}

/* D4/D5 notes from the C2b readouts: CHAN_INFO events with a non-zero
 * tx-frame and the largest rx-frame; pdev stats answers with a non-zero
 * local-enqued, hw-queued, mpdu-enqued, hw-reaped or base tx-frame (the pdev's
 * twin of the CHAN_INFO one). */
static inline void
wlrx_note_ci(struct wlrx *rx, const uint32_t *ci)
{
	if (ci[WLWMI_CI_TX_FRAME_CNT]) { rx->tx_ci++; }
	if (ci[WLWMI_CI_RX_FRAME_COUNT] > rx->ci_rx_frame_max) { rx->ci_rx_frame_max = ci[WLWMI_CI_RX_FRAME_COUNT]; }
}

static inline void
wlrx_note_stats(struct wlrx *rx, const uint32_t *s)
{
	static const uint8_t f[5] = { WLWMI_PS_LOCAL_ENQUED, WLWMI_PS_HW_QUEUED, WLWMI_PS_MPDU_ENQUED, WLWMI_PS_HW_REAPED,
		WLWMI_PS_TX_FRAME_COUNT };
	rx->stats_answers++;
	for (unsigned i = 0; i < 5; i++) { rx->tx_stats[i] += s[f[i]] != 0; }
}

/* D5: the next step after a boot that heard no beacon, the first matching
 * row of rx_path_plan.md 2.11 for the kit's bits (0x200 rows for the idle-list
 * kit without rings, 0x100 rows with rings, 0x400 rows for the chain-mask kit). */
static inline const char *
wlrx_next(const struct wlrx *rx, uint32_t bits)
{
	const struct wlrx_content *c = &rx->c;
	const struct wlrx_watch *mid = &rx->w[WLRX_P_MID], *end = &rx->w[WLRX_P_COMPLETED];
	const bool links = bits & WLRX_W20_IDLELINK, rings = bits & WLRX_W20_RXRING, chain = bits & WLRX_W20_CHAINMASK;
	const uint32_t valid = c->mac_valid[0] + c->mac_valid[1];
	if (rx->rx_on && (c->guard_upper || c->guard_last || c->guard_tail || c->guard_gap || c->mac_foreign[0] || c->mac_foreign[1])) {
		return "none: stop, owner review before any further kit (guard spill or foreign MAC entry)";
	}
	if ((mid->taken && mid->exhausted) || (end->taken && end->exhausted)) {
		return bits & WLRX_W20_BAND24 ? "none: owner review (a second inconclusive run, w20 0x10 already set)" :
		    "the same kit with w20 0x10 added (an exhaustion flag: inconclusive, nothing killed)";
	}
	if (links && rx->idle_tried && !rx->idle_on) {
		return rx->idle_own == WLRX_IDLE_SRNG || rx->idle_own == WLRX_IDLE_BASE ?
		    "fallback r1c2c-a w20 0x102 (258), then 0x502 (1282) (the firmware owns the WBM idle list)" :
		    "none: owner review (WBM idle pre-read ambiguous or unreadable)";
	}
	if (links && !rings && rx->idle_on && end->taken && !(end->bad | end->unreadable)) {
		if (end->idle_tp && c->link_written) { return "r1c2c w20 0x302 (770) (links used, nothing forwarded: H1b or H4)"; }
		if (!end->idle_tp && !c->link_written) { return "r1c2c w20 0x302 (770) (links untouched: RXDMA pushed nothing)"; }
	}
	if (rings && rx->rx_on && end->taken) {
		if (!(end->bad & WLRX_F_REFILL) && !(end->unreadable & WLRX_F_REFILL) && !end->refill_tp &&
		    !c->mac_entries[0] && !c->mac_entries[1]) {
			return "C2d step 1 after reading the W26 htt-t2h types (the firmware ignored the rings)";
		}
		if (c->refill_taken && valid && !c->bufs_written) {
			return "r1c2c-m w20 0x402 (1026) if not yet booted, else BDF/calibration (buffers placed, nothing written)";
		}
		if (c->bufs_written && (c->err_code[0] || c->err_rbm_link)) {
			return links ? "C2d step 1 (RXDMA could not complete pushes)" :
			    "r1c2c w20 0x302 (770) (RXDMA could not complete pushes)";
		}
		if (c->bufs_written && !c->err_entries) { return "C2d step 1, then the H8 pdev-parameter rung (frames landed, not forwarded)"; }
	}
	if (chain && !rx->chain_on) { return "none: kit void, owner review (chain mask not sent: no MAC/PHY caps)"; }
	if (chain && !rings && !links) {
		return rx->ci_rx_frame_max ? "r1c2c-all w20 0x702 (1794) if rx-frame rose above the C2b readout, else 1.2 rows L3/L4" :
		    "1.2 rows L3/L4 (rx-frame 0 on every channel: H3 dead)";
	}
	return "none matched: owner review (rx_path_plan.md 2.11)";
}

#endif /* FUXI_WLAN_CE_H */
