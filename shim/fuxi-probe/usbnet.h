/* Route U (docs-lanes/wifi-ios.md): en0's Ethernet frames over this USB monitor.
 * The loader owns one UN_SIZE-byte region inside its Image (build.sh usbnet.S: 16 KiB
 * aligned, zero in the file, before the payload), checks at the handoff that it lies
 * outside XNU RAM and the GPU scratch, writes header line 0, publishes it and names it
 * to the kernel in the ADT copy (/chosen "fuxi-usbnet", tools/mkadt-salami.py --usbnet).
 * Two monitor requests move frames: 0x5d writes one host frame into the RX ring, 0x5e
 * reads the header and the kernel's TX ring. Nothing else is read or written for the
 * host, and no request names an address. Pure C: no MMIO and no system registers. The
 * MMU is off, so every region access is an aligned 32- or 64-bit volatile load or store
 * (Device memory: frames move in 64-bit words where both sides are 8-byte aligned) and a
 * `dsb sy` comes before every index update.
 * A STALL drops the device from a WinUSB host until a replug (SPEC A1), so only a
 * malformed request STALLs (0x5d: wValue or wLength out of range; 0x5e the same). A
 * well-formed 0x5d always gets its data stage and the frame is kept or dropped and
 * counted; 0x5e before the launch or the handoff answers wLength zero bytes (magic 0).
 * tools/test-fuxi-usbnet.py compiles this file unchanged on the host and checks it
 * byte for byte against tools/fuxi_usbnet_ring.py LoaderRingSim.
 *
 * Layout, little endian; three 64-byte header lines, each with exactly one writer:
 *  line 0 (loader, once before XNU): 0x00 "FXNETL01", 0x08 version 1, 0x0c size,
 *    0x10 u64 self PA, 0x18 slot size 2048, 0x1c slots 32, 0x20 RX ring 0x4000,
 *    0x24 TX ring 0x14000, 0x28 adt_status (UN_ADT_*)
 *  line 1 (kernel, xnu/fuxi-wifi-ios/fuxi_usbnet_ring.h): 0x40 "FXNETK01" (its opt-in),
 *    0x48 kstate (bit0 backend up, bit1 en0 link), 0x4c rx_tail, 0x50 tx_head,
 *    0x54 rx_frames, 0x58 rx_bad, 0x5c tx_frames, 0x60 tx_full_drops, 0x64 tx_size_drops
 *  line 2 (loader, per request): 0x80 rx_head, 0x84 host_tx_tail, 0x88 out_ok,
 *    0x8c out_stall_full, 0x90 out_stall_other (frames dropped: ring full, anything
 *    else), 0x94 in_count, 0x98 tail_bad
 * Slot k of a ring is at the ring offset + (k % 32) * 2048: {u16 len, u16 flags 0,
 * u32 seq = k, u32 fnv1a32 of the frame, u32 0}, then the frame (14..1514 bytes, an
 * Ethernet frame without FCS) zero padded to 4. Indexes and counters are free running
 * u32. RX (host to kernel) is full at rx_head - rx_tail == 32, TX at
 * tx_head - host_tx_tail == 32.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "gpu-handoff.h"

/* FUXI_USBNET (0 or 1): default 1 in the salami XNU loader, 0 elsewhere (the standalone
 * probe has no handoff and no kernel to talk to). 0: no region, 0x5d/0x5e are unknown
 * requests and STALL like any other (no route U host talks to such an image). */
#ifndef FUXI_USBNET
#if defined(FUXI_SALAMI) && defined(FUXI_XNU_LOADER)
#define FUXI_USBNET 1
#else
#define FUXI_USBNET 0
#endif
#endif
#if FUXI_USBNET != 0 && FUXI_USBNET != 1
#error "FUXI_USBNET must be 0 or 1"
#endif
#if FUXI_USBNET && !(defined(FUXI_SALAMI) && defined(FUXI_XNU_LOADER))
#error "FUXI_USBNET is a salami XNU loader (FUXI_SALAMI, FUXI_XNU_LOADER) switch"
#endif

#define UN_SIZE         0x24000u
#define UN_ALIGN        0x4000u
#define UN_SLOT_SIZE    2048u
#define UN_SLOTS        32u
#define UN_RX_OFF       0x4000u
#define UN_TX_OFF       0x14000u
#define UN_SLOT_HDR     16u
#define UN_FRAME_MIN    14u
#define UN_FRAME_MAX    1514u
#define UN_IN_MIN       0xc0u           /* 0x5e: the three header lines at least */
#define UN_IN_MAX       4096u
#define UN_VERSION      1u
#define UN_MAGIC_LO     0x454e5846u     /* "FXNETL01" */
#define UN_MAGIC_HI     0x31304c54u
#define UN_KMAGIC_LO    0x454e5846u     /* "FXNETK01" */
#define UN_KMAGIC_HI    0x31304b54u
#define UN_REQ_OUT      0x5d
#define UN_REQ_IN       0x5e
#define UN_VALUE_OUT    0x4e52u         /* 'NR' */
#define UN_VALUE_IN     0x4e53u         /* 'NS' */
#define UN_FNV_BASIS    2166136261u
#define UN_FNV_PRIME    16777619u
#define UN_L0_MAGIC     0x00u
#define UN_L0_VERSION   0x08u
#define UN_L0_SIZE      0x0cu
#define UN_L0_SELF      0x10u
#define UN_L0_SLOT_SIZE 0x18u
#define UN_L0_SLOTS     0x1cu
#define UN_L0_RX_OFF    0x20u
#define UN_L0_TX_OFF    0x24u
#define UN_L0_ADT       0x28u
#define UN_K_MAGIC      0x40u
#define UN_K_STATE      0x48u
#define UN_K_RX_TAIL    0x4cu
#define UN_K_TX_HEAD    0x50u
#define UN_K_RX_FRAMES  0x54u
#define UN_K_RX_BAD     0x58u
#define UN_K_TX_FRAMES  0x5cu
#define UN_K_TX_FULL    0x60u
#define UN_K_TX_SIZE    0x64u
#define UN_L_RX_HEAD    0x80u
#define UN_L_HOST_TAIL  0x84u
#define UN_L_OUT_OK     0x88u
#define UN_L_OUT_FULL   0x8cu
#define UN_L_OUT_OTHER  0x90u
#define UN_L_IN_COUNT   0x94u
#define UN_L_TAIL_BAD   0x98u
#define UN_KSTATE_UP    1u
#define UN_KSTATE_LINK  2u
#define UN_ADT_PATCHED  1u              /* {pa, size} written over the one placeholder */
#define UN_ADT_NONE     2u              /* no placeholder: ADT made without --usbnet */
#define UN_ADT_SEVERAL  3u              /* two or more: nothing patched */
#define UN_ADT_REGION   4u              /* region check failed: not published, 0x5e stays all zero */
#define UN_ADT_WORDS    13u
#define UN_W(a, b, c, d) ((uint32_t)(a) | (uint32_t)(b) << 8 | (uint32_t)(c) << 16 | (uint32_t)(d) << 24)

/* cpu0 publishes (un_handoff) while cpu@100 serves USB from the same struct: base is
 * written last, after the header, and read volatile. */
struct un {
 volatile uintptr_t base;       /* the region (MMU off: its PA); 0 = not published */
 uint32_t adt_status;           /* UN_ADT_*; also +0x28 of a published region */
};

static inline uint32_t un_rd(uintptr_t b, uint32_t off) { return *(const volatile uint32_t *)(b + off); }
static inline void un_wr(uintptr_t b, uint32_t off, uint32_t v) { *(volatile uint32_t *)(b + off) = v; }
static inline void un_bump(uintptr_t b, uint32_t off) { un_wr(b, off, un_rd(b, off) + 1); }
static inline void un_dsb(void)
{
#if defined(__aarch64__)
 __asm__ volatile("dsb sy" ::: "memory");
#else
 __atomic_thread_fence(__ATOMIC_SEQ_CST);   /* host test */
#endif
}

/* The region is 16 KiB aligned and clear of the largest XNU RAM window the loader takes
 * (FG_RAM_BASE + FG_RAM_SIZE: XNU never manages it) and of the GPU scratch. */
static inline bool un_region_ok(uint64_t pa)
{
 const uint64_t end = pa + UN_SIZE;
 if (!pa || (pa & (UN_ALIGN - 1)) || end < pa) return false;
 if (pa < FG_RAM_BASE + FG_RAM_SIZE && FG_RAM_BASE < end) return false;
 if (pa < FG_SCRATCH + FG_SCRATCH_SIZE && FG_SCRATCH < end) return false;
 return true;
}

/* The /chosen property mkadt-salami.py --usbnet writes, as ADT words: the 32-byte name
 * "fuxi-usbnet", the u32 length 16 (no placeholder flag), the value "FXNET-PLACEHOLDR".
 * Returns how often it occurs at a 4-byte aligned offset of the ADT copy (2: two or
 * more); *at = the byte offset of the first. */
static inline unsigned un_adt_find(uintptr_t adt, uint64_t size, uint64_t *at)
{
 static const uint32_t want[UN_ADT_WORDS] = {
  UN_W('f', 'u', 'x', 'i'), UN_W('-', 'u', 's', 'b'), UN_W('n', 'e', 't', 0), 0, 0, 0, 0, 0, 16,
  UN_W('F', 'X', 'N', 'E'), UN_W('T', '-', 'P', 'L'), UN_W('A', 'C', 'E', 'H'), UN_W('O', 'L', 'D', 'R')};
 unsigned found = 0;
 for (uint64_t o = 0; o + 4 * UN_ADT_WORDS <= size && found < 2; o += 4) {
  unsigned i = 0;
  while (i < UN_ADT_WORDS && un_rd(adt, (uint32_t)o + 4 * i) == want[i]) i++;
  if (i == UN_ADT_WORDS && !found++) *at = o;
 }
 return found;
}

/* loader_boot, after the ADT copy, before XNU (cpu0; cpu@100 leaves the region alone
 * until base is set). A region that fails un_region_ok is never written: adt_status 4,
 * nothing published, 0x5e keeps answering zeros and 0x5d frames are dropped (uncounted:
 * there is nowhere to count). Otherwise: zero it, write line 0,
 * write {pa, size} over the placeholder when the ADT copy holds exactly one, record
 * adt_status at +0x28, then publish. Returns adt_status. */
static inline uint32_t un_handoff(struct un *u, uintptr_t base, uintptr_t adt, uint64_t adt_size)
{
 uint64_t at = 0;
 if (!un_region_ok(base)) return u->adt_status = UN_ADT_REGION;
 for (uint32_t off = 0; off < UN_SIZE; off += 4) un_wr(base, off, 0);
 un_wr(base, UN_L0_VERSION, UN_VERSION);
 un_wr(base, UN_L0_SIZE, UN_SIZE);
 un_wr(base, UN_L0_SELF, (uint32_t)base);
 un_wr(base, UN_L0_SELF + 4, (uint32_t)((uint64_t)base >> 32));
 un_wr(base, UN_L0_SLOT_SIZE, UN_SLOT_SIZE);
 un_wr(base, UN_L0_SLOTS, UN_SLOTS);
 un_wr(base, UN_L0_RX_OFF, UN_RX_OFF);
 un_wr(base, UN_L0_TX_OFF, UN_TX_OFF);
 un_wr(base, UN_L0_MAGIC, UN_MAGIC_LO);
 un_wr(base, UN_L0_MAGIC + 4, UN_MAGIC_HI);
 un_dsb();
 const unsigned found = un_adt_find(adt, adt_size, &at);
 if (found == 1) {
  un_wr(adt, (uint32_t)at + 36, (uint32_t)base);
  un_wr(adt, (uint32_t)at + 40, (uint32_t)((uint64_t)base >> 32));
  un_wr(adt, (uint32_t)at + 44, UN_SIZE);
  un_wr(adt, (uint32_t)at + 48, 0);
 }
 u->adt_status = found == 1 ? UN_ADT_PATCHED : found ? UN_ADT_SEVERAL : UN_ADT_NONE;
 un_wr(base, UN_L0_ADT, u->adt_status);
 un_dsb();
 u->base = base;
 un_dsb();
 return u->adt_status;
}

/* The kernel's opt-in: its magic in line 1 and kstate bit0 (RX thread running). */
static inline bool un_kernel_up(uintptr_t b)
{
 return un_rd(b, UN_K_MAGIC) == UN_KMAGIC_LO && un_rd(b, UN_K_MAGIC + 4) == UN_KMAGIC_HI &&
        (un_rd(b, UN_K_STATE) & UN_KSTATE_UP);
}

/* host_tx_tail from wIndex, the low 16 bits of the host's TX consumed count. That count
 * lies in [tx_head - 32, tx_head], so it is rebuilt from tx_head; a value outside that
 * window keeps the old host_tx_tail and counts tail_bad. */
static inline void un_host_tail(uintptr_t b, uint16_t index)
{
 const uint32_t head = un_rd(b, UN_K_TX_HEAD);
 uint32_t t = (head & ~0xffffu) | index;
 if (t > head) t -= 0x10000u;
 if (head - t <= UN_SLOTS) un_wr(b, UN_L_HOST_TAIL, t);
 else un_bump(b, UN_L_TAIL_BAD);
}

/* Request 0x5d setup (OUT 0x40, wValue 'NR', wIndex = the host's TX count, wLength =
 * one 14..1514-byte frame): true starts the data stage. Only a malformed request (a
 * correct host never sends one) is refused: false STALLs and, once the region is
 * published, counts out_stall_other. Whether the frame is kept is decided after the
 * data stage (un_out_data), so a busy phone never STALLs (SPEC A1). */
static inline bool un_out_ok(struct un *u, uint16_t value, uint16_t length)
{
 const uintptr_t b = u->base;
 if (value == UN_VALUE_OUT && length >= UN_FRAME_MIN && length <= UN_FRAME_MAX) return true;
 if (b) un_bump(b, UN_L_OUT_OTHER);
 return false;
}

/* Request 0x5d data stage completed; ok: it carried exactly wLength bytes (the audio
 * OUT residue rule). src is the ep0 buffer (8-byte aligned, readable to length rounded
 * up to 4). The frame is kept only after the launch, with the region published, the
 * kernel up (kmagic, kstate bit0) and a free RX slot: it and its header go into slot
 * rx_head % 32 (only this code advances rx_head), then after a dsb rx_head + 1,
 * host_tx_tail from wIndex, out_ok + 1. Anything else drops it (the status stage still
 * completes, host_tx_tail is left alone) and counts out_stall_full when a full ring was
 * the only reason, else out_stall_other; unpublished, nothing is counted. Returns true
 * when the frame was kept. */
static inline bool un_out_data(struct un *u, bool launched, bool ok, const volatile uint8_t *src,
                               uint16_t length, uint16_t index)
{
 const uintptr_t b = u->base;
 if (!b) return false;
 const bool other = !ok || !launched || !un_kernel_up(b) || length < UN_FRAME_MIN || length > UN_FRAME_MAX;
 const uint32_t head = un_rd(b, UN_L_RX_HEAD);
 const bool full = head - un_rd(b, UN_K_RX_TAIL) >= UN_SLOTS;
 if (other || full) {
  un_bump(b, other ? UN_L_OUT_OTHER : UN_L_OUT_FULL);
  return false;
 }
 const uint32_t slot = UN_RX_OFF + head % UN_SLOTS * UN_SLOT_SIZE;
 uint32_t hash = UN_FNV_BASIS, i = 0;
 for (; i + 8 <= length; i += 8) {      /* UD_DATA and slot + 16 are both 8-byte aligned */
  const uint64_t w = *(const volatile uint64_t *)(src + i);
  for (uint32_t k = 0; k < 8; k++) hash = (hash ^ (uint8_t)(w >> 8 * k)) * UN_FNV_PRIME;
  *(volatile uint64_t *)(b + slot + UN_SLOT_HDR + i) = w;
 }
 for (; i < length; i += 4) {           /* the last 1..7 bytes, zero padded to 4 */
  uint32_t w = *(const volatile uint32_t *)(src + i);
  if (length - i < 4) w &= (1u << 8 * (length - i)) - 1;
  for (uint32_t k = 0; k < 4 && i + k < length; k++) hash = (hash ^ (uint8_t)(w >> 8 * k)) * UN_FNV_PRIME;
  un_wr(b, slot + UN_SLOT_HDR + i, w);
 }
 un_wr(b, slot, length);                /* len, flags 0 */
 un_wr(b, slot + 4, head);              /* seq */
 un_wr(b, slot + 8, hash);
 un_wr(b, slot + 12, 0);
 un_dsb();
 un_wr(b, UN_L_RX_HEAD, head + 1);
 un_host_tail(b, index);
 un_bump(b, UN_L_OUT_OK);
 return true;
}

/* Zero out[pos, length rounded up to 4); pos is a multiple of 4, out 8-byte aligned. */
static inline void un_zero(volatile uint8_t *out, uint32_t pos, uint32_t length)
{
 for (; pos < length && (pos & 7); pos += 4) *(volatile uint32_t *)(out + pos) = 0;
 for (; pos + 8 <= length; pos += 8) *(volatile uint64_t *)(out + pos) = 0;
 for (; pos < length; pos += 4) *(volatile uint32_t *)(out + pos) = 0;
}

/* Request 0x5e (IN 0xc0, wValue 'NS', wIndex = the host's TX count, wLength 192..4096).
 * 0 STALLs: only wValue or wLength out of range. Otherwise out (8-byte aligned, room for
 * length rounded up to 4) gets exactly length bytes. Before the launch or the handoff
 * they are all zero (magic 0: not ready) and nothing else happens. Else host_tx_tail from
 * wIndex, in_count + 1, then header lines 0..2, then from 0xc0 whole TX slots from
 * host_tx_tail while below tx_head, each the 16-byte slot header and the frame zero
 * padded to 4, as many as fit, then zeros. The zero padding after the last one (a len
 * of 0) ends the list; a slot whose len or seq is wrong ends it too. */
static inline uint32_t un_in(struct un *u, bool launched, uint16_t value, uint16_t index, uint16_t length,
                             volatile uint8_t *out)
{
 const uintptr_t b = u->base;
 if (value != UN_VALUE_IN || length < UN_IN_MIN || length > UN_IN_MAX) return 0;
 if (!launched || !b) {
  un_zero(out, 0, length);
  return length;
 }
 un_host_tail(b, index);
 un_bump(b, UN_L_IN_COUNT);
 for (uint32_t o = 0; o < UN_IN_MIN; o += 8) *(volatile uint64_t *)(out + o) = *(const volatile uint64_t *)(b + o);
 const uint32_t head = un_rd(b, UN_K_TX_HEAD);
 un_dsb();
 const uint32_t tail = un_rd(b, UN_L_HOST_TAIL);
 uint32_t pos = UN_IN_MIN;
 for (uint32_t k = tail; k != head && k - tail < UN_SLOTS; k++) {
  const uint32_t slot = UN_TX_OFF + k % UN_SLOTS * UN_SLOT_SIZE;
  const uint32_t len = un_rd(b, slot) & 0xffffu;
  if (len < UN_FRAME_MIN || len > UN_FRAME_MAX || un_rd(b, slot + 4) != k) break;
  const uint32_t bytes = UN_SLOT_HDR + ((len + 3) & ~3u);
  if (bytes > length - pos) break;
  uint32_t i = 0;
  if (!(pos & 7))                       /* the slot is 8-byte aligned, out + pos maybe only 4 */
   for (; i + 8 < bytes; i += 8) *(volatile uint64_t *)(out + pos + i) = *(const volatile uint64_t *)(b + slot + i);
  for (; i < bytes; i += 4) {           /* the last word or two: the pad bytes masked */
   uint32_t w = un_rd(b, slot + i);
   if (i + 4 == bytes && (len & 3)) w &= (1u << 8 * (len & 3)) - 1;
   *(volatile uint32_t *)(out + pos + i) = w;
  }
  pos += bytes;
 }
 un_zero(out, pos, length);
 return length;
}
