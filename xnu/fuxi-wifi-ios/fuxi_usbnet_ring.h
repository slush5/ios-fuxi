/*
 * Route U: en0's Ethernet frames over the loader's USB monitor (docs-lanes/wifi-ios.md).
 *
 * The loader (shim/fuxi-probe/usbnet.h) owns a 0x24000-byte region outside XNU RAM and the
 * GPU scratch, writes header line 0 before XNU starts and names the region in /chosen
 * "fuxi-usbnet" {u64 pa, u64 size}. fuxi_wifi_ios.c (fuxiwifiios=2) maps it Normal
 * non-cacheable; the loader runs with the MMU off, so neither side caches it and no cache
 * maintenance is needed, only ordering: a dsb before every index store, and one after
 * every index load before the slot it covers is read. No MMIO, no device: plain DRAM.
 *
 * This is the kernel's side of the ring, pure C, so tools/test-fuxi-usbnet.py compiles it
 * unchanged on the host and checks it byte for byte against tools/fuxi_usbnet_ring.py
 * KernelSim; the same test checks every value here against usbnet.h and gpu-handoff.h.
 * Line 1 (0x40..0x7f) is the kernel's alone: magic, kstate, rx_tail, tx_head, counters.
 * Line 2 (0x80..) and the RX slots are the loader's; the kernel only reads them.
 * Slot k: ring offset + (k % 32) * 2048 = {u16 len, u16 flags 0, u32 seq k, u32 fnv1a32,
 * u32 0} and the frame (14..1514 bytes) zero padded to 4. Counters run free (u32).
 * The region is an address (uintptr_t), every access an aligned 32-bit volatile one.
 */
#ifndef _SYS_FUXI_USBNET_RING_H_
#define _SYS_FUXI_USBNET_RING_H_
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FXNET_SIZE              0x24000u
#define FXNET_ALIGN             0x4000u
#define FXNET_SLOT_SIZE         2048u
#define FXNET_SLOTS             32u
#define FXNET_RX_OFF            0x4000u
#define FXNET_TX_OFF            0x14000u
#define FXNET_SLOT_HDR          16u
#define FXNET_FRAME_MIN         14u
#define FXNET_FRAME_MAX         1514u
#define FXNET_VERSION           1u
#define FXNET_MAGIC_LO          0x454e5846u     /* "FXNETL01", the loader's */
#define FXNET_MAGIC_HI          0x31304c54u
#define FXNET_KMAGIC_LO         0x454e5846u     /* "FXNETK01", the kernel's opt-in */
#define FXNET_KMAGIC_HI         0x31304b54u
#define FXNET_L0_MAGIC          0x00u
#define FXNET_L0_VERSION        0x08u
#define FXNET_L0_SIZE           0x0cu
#define FXNET_L0_SELF           0x10u
#define FXNET_L0_SLOT_SIZE      0x18u
#define FXNET_L0_SLOTS          0x1cu
#define FXNET_L0_RX_OFF         0x20u
#define FXNET_L0_TX_OFF         0x24u
#define FXNET_K_MAGIC           0x40u
#define FXNET_K_STATE           0x48u
#define FXNET_K_RX_TAIL         0x4cu
#define FXNET_K_TX_HEAD         0x50u
#define FXNET_K_RX_FRAMES       0x54u
#define FXNET_K_RX_BAD          0x58u
#define FXNET_K_TX_FRAMES       0x5cu
#define FXNET_K_TX_FULL         0x60u
#define FXNET_K_TX_SIZE         0x64u
#define FXNET_L_RX_HEAD         0x80u
#define FXNET_L_HOST_TAIL       0x84u
#define FXNET_KSTATE_UP         1u              /* kmagic written and the RX thread running */
#define FXNET_KSTATE_LINK       2u              /* en0's link is up */
/* Where the region must never be: XNU RAM at its largest (gpu-handoff.h FG_RAM_BASE,
 * FG_RAM_SIZE) and the GPU scratch (FG_SCRATCH, FG_SCRATCH_SIZE). */
#define FXNET_RAM_BASE          0x8c0000000ull
#define FXNET_RAM_SIZE          0x13e000000ull
#define FXNET_GPU_SCRATCH       0x9fe000000ull
#define FXNET_GPU_SCRATCH_SIZE  0x02000000ull

#if defined(KERNEL) && (defined(__arm64__) || defined(__aarch64__))
#define FXNET_DSB()             __builtin_arm_dsb(0xf)          /* dsb sy */
#else
#define FXNET_DSB()             __atomic_thread_fence(__ATOMIC_SEQ_CST)
#endif

enum { FXNET_TX_OK, FXNET_TX_FULL, FXNET_TX_SIZE };
enum { FXNET_RX_EMPTY, FXNET_RX_OK, FXNET_RX_BAD };

static inline uint32_t
fxnet_rd(uintptr_t region, uint32_t offset)
{
	return *(volatile const uint32_t *)(region + offset);
}

static inline void
fxnet_wr(uintptr_t region, uint32_t offset, uint32_t value)
{
	*(volatile uint32_t *)(region + offset) = value;
}

static inline void
fxnet_bump(uintptr_t region, uint32_t offset)
{
	fxnet_wr(region, offset, fxnet_rd(region, offset) + 1);
}

/* The /chosen "fuxi-usbnet" value: NULL when it names a usable region (*pa is set), else why not. */
static inline const char *
fxnet_handoff(const uint8_t *value, unsigned int size, uint64_t *pa)
{
	static const uint8_t placeholder[16] = { 'F', 'X', 'N', 'E', 'T', '-', 'P', 'L', 'A', 'C', 'E', 'H', 'O', 'L', 'D', 'R' };
	uint64_t at = 0, bytes = 0;
	unsigned int same = 0;

	if (size != 16) {
		return "/chosen fuxi-usbnet is not 16 bytes";
	}
	for (unsigned int i = 0; i < 16; i++) {
		same += value[i] == placeholder[i];
	}
	if (same == 16) {
		return "the placeholder is still there (no FUXI_USBNET loader, or it refused its region)";
	}
	for (unsigned int i = 8; i-- > 0;) {
		at = at << 8 | value[i];
		bytes = bytes << 8 | value[8 + i];
	}
	if (bytes != FXNET_SIZE) {
		return "the region is not 0x24000 bytes";
	}
	if (at == 0 || (at & (FXNET_ALIGN - 1)) != 0 || at + FXNET_SIZE < at) {
		return "the region is not 16 KiB aligned";
	}
	if (at < FXNET_RAM_BASE + FXNET_RAM_SIZE && FXNET_RAM_BASE < at + FXNET_SIZE) {
		return "the region is inside XNU RAM";
	}
	if (at < FXNET_GPU_SCRATCH + FXNET_GPU_SCRATCH_SIZE && FXNET_GPU_SCRATCH < at + FXNET_SIZE) {
		return "the region is inside the GPU scratch";
	}
	*pa = at;
	return NULL;
}

/* Line 0 of the mapped region: NULL when it is the loader layout this kernel speaks. */
static inline const char *
fxnet_header(uintptr_t region, uint64_t pa)
{
	uint64_t self = fxnet_rd(region, FXNET_L0_SELF) | (uint64_t)fxnet_rd(region, FXNET_L0_SELF + 4) << 32;

	if (fxnet_rd(region, FXNET_L0_MAGIC) != FXNET_MAGIC_LO || fxnet_rd(region, FXNET_L0_MAGIC + 4) != FXNET_MAGIC_HI) {
		return "no loader magic in the region";
	}
	if (fxnet_rd(region, FXNET_L0_VERSION) != FXNET_VERSION) {
		return "the region is not version 1";
	}
	if (fxnet_rd(region, FXNET_L0_SIZE) != FXNET_SIZE || self != pa) {
		return "the region header names another size or address";
	}
	if (fxnet_rd(region, FXNET_L0_SLOT_SIZE) != FXNET_SLOT_SIZE || fxnet_rd(region, FXNET_L0_SLOTS) != FXNET_SLOTS ||
	    fxnet_rd(region, FXNET_L0_RX_OFF) != FXNET_RX_OFF || fxnet_rd(region, FXNET_L0_TX_OFF) != FXNET_TX_OFF) {
		return "the region has another ring layout";
	}
	return NULL;
}

/* The kernel's opt-in: the loader takes 0x5d frames only with this magic and kstate bit0. */
static inline void
fxnet_attach(uintptr_t region)
{
	fxnet_wr(region, FXNET_K_MAGIC, FXNET_KMAGIC_LO);
	fxnet_wr(region, FXNET_K_MAGIC + 4, FXNET_KMAGIC_HI);
	FXNET_DSB();
}

static inline void
fxnet_kstate(uintptr_t region, uint32_t state)
{
	fxnet_wr(region, FXNET_K_STATE, state);
	FXNET_DSB();
}

/* One frame into TX slot tx_head % 32 (the caller serializes): FXNET_TX_FULL while
 * tx_head - host_tx_tail == 32 (tx_full_drops), FXNET_TX_SIZE outside 14..1514 (tx_size_drops). */
static inline int
fxnet_tx_put(uintptr_t region, const uint8_t *frame, size_t length)
{
	uint32_t head, tail, slot, hash = 2166136261u;

	if (length < FXNET_FRAME_MIN || length > FXNET_FRAME_MAX) {
		fxnet_bump(region, FXNET_K_TX_SIZE);
		return FXNET_TX_SIZE;
	}
	head = fxnet_rd(region, FXNET_K_TX_HEAD);
	tail = fxnet_rd(region, FXNET_L_HOST_TAIL);
	FXNET_DSB();
	if (head - tail >= FXNET_SLOTS) {
		fxnet_bump(region, FXNET_K_TX_FULL);
		return FXNET_TX_FULL;
	}
	slot = FXNET_TX_OFF + head % FXNET_SLOTS * FXNET_SLOT_SIZE;
	for (uint32_t i = 0; i < length; i += 4) {
		uint32_t word = 0;
		for (uint32_t k = 0; k < 4 && i + k < length; k++) {
			word |= (uint32_t)frame[i + k] << 8 * k;
			hash = (hash ^ frame[i + k]) * 16777619u;
		}
		fxnet_wr(region, slot + FXNET_SLOT_HDR + i, word);
	}
	fxnet_wr(region, slot, (uint32_t)length);       /* len, flags 0 */
	fxnet_wr(region, slot + 4, head);               /* seq */
	fxnet_wr(region, slot + 8, hash);
	fxnet_wr(region, slot + 12, 0);
	FXNET_DSB();
	fxnet_wr(region, FXNET_K_TX_HEAD, head + 1);
	fxnet_bump(region, FXNET_K_TX_FRAMES);
	return FXNET_TX_OK;
}

/* The oldest unread RX slot, copied into frame[FXNET_FRAME_MAX]: FXNET_RX_OK (*length set) or
 * FXNET_RX_BAD (len, seq or fnv wrong), each then consumed by fxnet_rx_done; or FXNET_RX_EMPTY. */
static inline int
fxnet_rx_take(uintptr_t region, uint8_t *frame, size_t *length)
{
	uint32_t head, tail, slot, len, hash, check = 2166136261u;

	head = fxnet_rd(region, FXNET_L_RX_HEAD);
	tail = fxnet_rd(region, FXNET_K_RX_TAIL);
	FXNET_DSB();
	if (head == tail) {
		return FXNET_RX_EMPTY;
	}
	slot = FXNET_RX_OFF + tail % FXNET_SLOTS * FXNET_SLOT_SIZE;
	len = fxnet_rd(region, slot) & 0xffff;
	hash = fxnet_rd(region, slot + 8);
	if (len < FXNET_FRAME_MIN || len > FXNET_FRAME_MAX || fxnet_rd(region, slot + 4) != tail) {
		return FXNET_RX_BAD;
	}
	for (uint32_t i = 0; i < len; i += 4) {
		uint32_t word = fxnet_rd(region, slot + FXNET_SLOT_HDR + i);
		for (uint32_t k = 0; k < 4 && i + k < len; k++) {
			frame[i + k] = (uint8_t)(word >> 8 * k);
			check = (check ^ frame[i + k]) * 16777619u;
		}
	}
	if (check != hash) {
		return FXNET_RX_BAD;
	}
	*length = len;
	return FXNET_RX_OK;
}

/* Consume the slot fxnet_rx_take looked at; after the dsb the loader may fill it again. */
static inline void
fxnet_rx_done(uintptr_t region, bool good)
{
	FXNET_DSB();
	fxnet_wr(region, FXNET_K_RX_TAIL, fxnet_rd(region, FXNET_K_RX_TAIL) + 1);
	fxnet_bump(region, good ? FXNET_K_RX_FRAMES : FXNET_K_RX_BAD);
}
#endif /* _SYS_FUXI_USBNET_RING_H_ */
