/* M6 cold handoff, shared by the loader and XNU. No firmware bytes here.
 * Also the one source of the board's ABL splash geometry and its scanout guard. */
#ifndef FUXI_GPU_HANDOFF_H
#define FUXI_GPU_HANDOFF_H
#define FG_BANK_SIZE 0x140000000ULL
#define FG_RAM_BASE 0x8c0000000ULL
#define FG_RAM_SIZE 0x13e000000ULL
#define FG_SCRATCH  0x9fe000000ULL
#define FG_SCRATCH_SIZE 0x02000000u
#define FG_HANDOFF  (FG_SCRATCH + 0x1800000)
#define FG_FW_BASE  (FG_HANDOFF + 0x4000)
#define FG_FW_SLOT  0x100000u
#define FG_MAGIC    0x36555047u /* GPU6 */
struct fg_handoff {
 uint32_t magic, version, bytes, reserved;
 uint64_t base, size;
 uint32_t fw_size[3], fw_hash[3]; /* GMU, SQE, zap; FNV1a detects corruption */
};
static inline uint32_t fg_hash(const uint8_t *p, unsigned n)
{
 uint32_t h = 2166136261u;
 for (unsigned i = 0; i < n; i++) h = (h ^ p[i]) * 16777619u;
 return h;
}
/* ABL splash (live FDT /reserved-memory/splash_region, same on both boards):
 * one 1080-wide ARGB8888 buffer at 0xb8000000, stride 4320, SSPP format 0x237ff.
 * Fuxi panel 1080x2400, scanned by SSPP0 alone. OnePlus 11 "salami"
 * (FUXI_SALAMI) timing@fhd_sa_60 is 1080x2412 (panel-height 0x96c); a salami
 * probe boot measured the ABL handoff: the same buffer, stride and format,
 * unpack 0x03020001, scanned by TWO pipes, VIG0 0xae05000 and VIG1 0xae07000,
 * each 540x2412 (source and output size 0x096c021c), multirect off. The format
 * can still be overridden at build time (-DFG_ABL_SSPP_FORMAT=...). */
#define FG_SPLASH_PA   0xb8000000ULL
#define FG_SPLASH_SIZE 0x2b00000ULL
#define FG_SSPP0       0x0ae05000UL
#define FG_FB_W 1080
#if defined(FUXI_SALAMI)
#define FG_FB_H 2412
#define FG_ABL_PIPES   2u          /* VIG0, VIG1 */
#define FG_ABL_PIPE_W  540u        /* columns per pipe */
#define FG_SSPP_STRIDE 0x2000u     /* VIG1 = FG_SSPP0 + FG_SSPP_STRIDE */
#define FG_ABL_UNPACK  0x03020001u /* B, G, R, A: DRM ARGB8888 */
#else
#define FG_FB_H 2400
#endif
#define FG_FB_STRIDE (FG_FB_W * 4)
#ifndef FG_ABL_SSPP_FORMAT
#define FG_ABL_SSPP_FORMAT 0x237ffu
#endif
#if defined(FUXI_SALAMI)
/* The ABL-scanout guard (salami): nonzero only while VIG0 and then VIG1 both
 * still scan the splash exactly as measured. rd(ctx, off) returns the register
 * at FG_SSPP0 + off, so off + FG_SSPP_STRIDE is the same register of VIG1.
 * Per pipe, reads stop at the first mismatch: +0x14 address, +0x24 stride,
 * +0x00 source size, +0x0c output size, +0x30 format, +0x34 unpack, +0x170
 * multirect (0). */
static inline __attribute__((unused)) int
fg_abl_scanout_ok(uint32_t (*rd)(const void *ctx, uint32_t off), const void *ctx)
{
 const uint32_t size = (uint32_t)FG_FB_H << 16 | FG_ABL_PIPE_W;
 for (uint32_t p = 0; p < FG_ABL_PIPES; p++) {
  const uint32_t b = p * FG_SSPP_STRIDE;
  if (rd(ctx, b + 0x14) != (uint32_t)FG_SPLASH_PA || rd(ctx, b + 0x24) != (uint32_t)FG_FB_STRIDE ||
      rd(ctx, b + 0x00) != size || rd(ctx, b + 0x0c) != size ||
      rd(ctx, b + 0x30) != (uint32_t)FG_ABL_SSPP_FORMAT || rd(ctx, b + 0x34) != FG_ABL_UNPACK ||
      rd(ctx, b + 0x170) != 0)
   return 0;
 }
 return 1;
}
#else
/* The ABL-scanout guard: nonzero only while SSPP0 still scans the splash with
 * exactly this geometry. rd(ctx, off) returns SSPP0 register off; reads stop at
 * the first mismatch: +0x14 address, +0x24 stride, +0x00 source size, +0x30 format. */
static inline __attribute__((unused)) int
fg_abl_scanout_ok(uint32_t (*rd)(const void *ctx, uint32_t off), const void *ctx)
{
 return rd(ctx, 0x14) == (uint32_t)FG_SPLASH_PA && rd(ctx, 0x24) == (uint32_t)FG_FB_STRIDE &&
        rd(ctx, 0x00) == ((uint32_t)FG_FB_H << 16 | (uint32_t)FG_FB_W) &&
        rd(ctx, 0x30) == (uint32_t)FG_ABL_SSPP_FORMAT;
}
#endif
#endif
