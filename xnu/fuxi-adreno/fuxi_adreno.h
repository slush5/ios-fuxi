#ifndef FUXI_ADRENO_H
#define FUXI_ADRENO_H
#define FG_CACHE_SLOTS 4u
/* Panel rows the ABL splash pipe scans out of 0xb8000000 (and the double/
 * triple buffers 0xb8a00000/0xb9400000), 1080 wide, ARGB8888, stride 4320.
 * fuxi (Xiaomi 13): 2400. salami (OnePlus 11, FUXI_SALAMI, timing
 * fhd_sa_60): 2412. Either frame fits one 0xa00000 slot (4320 x 2412 =
 * 0x9efe80). FUXI_PANEL_MAP is one frame rounded up to 16 KiB pages.
 * The single source is FG_FB_H in gpu-handoff.h, which the C++ users of this
 * header (SweetSurface, SweetFramebuffer) never include; fuxi_adreno.c sees
 * both and refuses to compile if they differ. */
#ifndef FUXI_PANEL_H
#if defined(FUXI_SALAMI)
#define FUXI_PANEL_H 2412u
#else
#define FUXI_PANEL_H 2400u
#endif
#endif
#define FUXI_PANEL_MAP ((4320u*FUXI_PANEL_H+16383u)&~16383u)
/* Timings in mach absolute ticks, CPU uses scheduler runtime (excludes sleep).
 * Surface leases use a short lock separate from the copy/cache transaction.
 * The ABI shared mutex serializes ring and SMMU control. */
struct fg_times {
 uint64_t pin, lookup, map, clean, submit, wait, unmap, cpu;
 unsigned attempts, hits, misses, sleeps, blocks, groups;
};
/* Full-range lookup only; zero for unpublished, wrapping or unknown PAs. */
uintptr_t fuxi_gpu_nc_pointer(uint64_t pa, uint64_t bytes);
int fuxi_gpu_ready(void);
/* Cumulative ABI cache bytes (including initial map clean), plus live imports.
 * Snapshot under the GPU lock; no counter reset or ownership change. */
void fuxi_gpu_import_stats(uint64_t *cleaned, uint64_t *skipped, uint64_t *invalidated,
    uint64_t *mapped, unsigned *imports);
/* Default-off command-mode handoff. configure is called before GPU startup.
 * acquire: 1 GPU target writable, 0 deferred, -1 held. CPU writers must use
 * cpu_acquire, which drains/restores A until the next GPU frame.
 * publish: 1 transferred, 2 back ready/deferred, 0 old path (caller kicks), -1 held.
 * retry: same results, but 0 means no ready frame; never issue a generic kick.
 * reserve/release bracket a SweetSurface cache transaction; 0 means defer.
 */
void fuxi_gpu_scanout_configure(uintptr_t pixels, unsigned enabled, unsigned check);
int fuxi_gpu_scanout_owned(void);
/* One copy/ready frame at a time; timestamps are mach absolute units. */
struct fg_present_times { uint64_t submitted, fence, start, previous_start; };
void fuxi_gpu_scanout_stamps(struct fg_present_times *times);
/* Caller-owned, zero-initialized TE observer. Use from one serialized timer;
 * independent observers share no mutable estimator state. Healthy observation
 * takes no GPU lock or writes; double-mode MMIO failure stops/restores A.
 * On success frame (low16), edge and period describe the latest observed TE
 * in mach absolute units. On failure these outputs must not be used. */
struct fg_te_observer {
 uint64_t edge, period, poll, change, raw_edge;
 unsigned frame, raw_frame, samples, initialized, nominal_us;
};
int fuxi_gpu_scanout_te(struct fg_te_observer *te);
int fuxi_gpu_scanout_acquire(void);
int fuxi_gpu_scanout_publish(void);
int fuxi_gpu_scanout_retry(void);
int fuxi_gpu_scanout_pending(void);
int fuxi_gpu_scanout_double(void);
int fuxi_gpu_scanout_triple(void);
int fuxi_gpu_scanout_cpu_acquire(void);
int fuxi_gpu_scanout_reserve(void);
void fuxi_gpu_scanout_release(void);
void fuxi_gpu_start(uint64_t framebuffer, unsigned width, unsigned height, unsigned pitch);
int fuxi_gpu_map(unsigned slot, uint64_t generation, uintptr_t source,
    uint64_t length, struct fg_times *t);
/* False means failed DMA drain: keep backing prepared/retained until reset. */
int fuxi_gpu_unmap(unsigned slot, struct fg_times *t);
/* copy: 2 submitted, 0 fallback, -1 quarantine. poll: 0 busy, 1 done,
 * -1 failed/drained, -2 failed/quarantined. Serialized by the surface cache lock. */
int fuxi_gpu_poll(struct fg_times *t);
int fuxi_gpu_copy(unsigned slot, uint64_t generation, uintptr_t source, uint64_t length,
    unsigned pitch, unsigned sx, unsigned sy, unsigned dx, unsigned dy,
    unsigned width, unsigned height, struct fg_times *t);
#endif
