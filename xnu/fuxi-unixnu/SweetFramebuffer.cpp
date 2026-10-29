/*
 * Built-in IOMobileFramebuffer for SWEET: the ABL's continuous-splash
 * framebuffer (a8r8g8b8 in RAM, 1080x2400 on fuxi, 1080x2412 on salami;
 * the size comes from boot_args Video; the panel scans it out by
 * itself) as the main display, driven by CoreAnimation's CPU renderer.
 *
 * IOMobileFramebuffer.framework (iOS 16.7) counts /arm-io children with
 * device_type "display-subsystem", waits for that many services matching
 * IOServiceMatching("IOMobileFramebuffer"), opens each with type 0 and
 * reads canvas-width/canvas-height from the service's provider. The class
 * here (AppleCLCD, an IOMobileFramebuffer) matches the /arm-io/disp0 nub.
 *
 * User client selectors used by CoreAnimation:
 *   3  GetLayerDefaultSurface [layer, ?] -> [IOSurface port name]
 *   4  SwapBegin -> [swap id]
 *   5  SwapEnd, struct 0x46c: +0x50 swap id, +0x54 IOSurface id per layer
 *      (4 layers), +0x64 rect a / +0xc4 rect b per layer (int x,y,w,h),
 *      +0x104 enabled layer mask
 *   6  SwapWait, 7 GetID, 8 GetDisplaySize -> [w, h],
 *   12 RequestPowerChange [state]: 0 = display off (backboardd, power
 *      button / auto-lock), else on. Off fades the panel out over a few
 *      vsyncs (halving each pixel) and keeps it black until on again;
 *      the panel itself stays powered (AMOLED black pixels are off).
 *      Boot-arg sweetfb_bootoff=N (0..8, default 1 on salami, else 0)
 *      drops the first N offs since boot: backboardd turns the display off
 *      once while it brings up the GUI, the boot logo stays on screen.
 *   18 IsMainDisplay,
 *   28 GetDotPitch, 72 Enable/DisableNotifications [func, refcon, type, 0]
 * Everything else is accepted and logged once.
 *
 * SwapEnd copies the enabled layers' surfaces into the panel memory (no
 * scaling, CPU copy). Vsync (notification type 5) is a 60 Hz timer that
 * sends (count, mach_absolute_time, 0) as 32-bit halves to clients that
 * enabled it, through the port they registered (IOConnectSetNotificationPort).
 *
 * The panel memory itself is published as an IOSurface and returned as the
 * default framebuffer surface. The kernel text console keeps drawing over
 * the GUI by default: boot-arg sweetfb_noconsole=1 stops drawing it from
 * the first swap on (it still reaches the persistent log; panics draw).
 *
 * FUXI (0093): frames are shown at up to 60 Hz by default (7 fast cores;
 * sweetfb_hz still overrides), the tick runs on absolute deadlines so the
 * copy time does not stretch the period, every 600 presented frames log
 * "SWEET FB: stats" (copy time, swaps, dropped swaps), and display off is a
 * single black fill (write-only: the panel memory is write-combined, the
 * read-modify-write fade read it back uncached). sweet_fb_display_off tells
 * the touch driver (bsd/dev/fuxi_touch.c) that the display is off.
 * sweetfb_statsframes=N (30..6000) replaces the 600. IOLog cuts lines near
 * 256 bytes, so the stats line is followed by a short "SWEET FB: tegate hits"
 * line that repeats the gate and format counters from its tail.
 *
 * The plain CPU copy moves BGRA words only: a layer whose surface has another
 * format (w30r, b3a8 with wide colour) is not copied, logged once per format
 * and counted (fmt_refused in the stats line); sweetfb_fmtcheck=0 copies it
 * as before. FUXI opt-ins, both off by default: sweetfb_tegate=1 starts each
 * CPU copy into the panel buffer inside a window of INTF1 tear-check lines
 * (sweet_fb_tegate_configure); sweetfb_wakepresent=1 presents a CPU-path swap
 * from SwapEnd through the writer-wake call instead of at the next tick.
 */
// Ported from UniXNU drivers/NativeMobileDisplay/NativeLegacyDisplay.cpp (BSD-2-Clause-Patent)
// Optional demand-driven timer policy; see LICENSE.NativeMobileDisplay.
#include "SweetSurface.h"
#include "fuxi_unixnu_copy.h"
#include "fuxi_unixnu_display.h"
#if defined(FUXI)
extern "C" {
#include "../../../bsd/dev/fuxi_adreno.h"
void fuxi_perf_start(void);
void fuxi_perf_swap(void);
}
#endif

extern "C" int sweet_console_muted;     /* osfmk/console/serial_console.c */
extern "C" { int sweet_fb_display_off; }        /* read by bsd/dev/fuxi_touch.c */

#if defined(SWEET)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/system.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <sys/proc.h>

#if defined(FUXI)
extern bool sweet_surface_present_begin(SweetSurface **surfaces, unsigned count, uint64_t *ready, const uint64_t *epochs);
extern void sweet_surface_present_end(void);
extern void sweet_surface_present_notify(void (*wake)(void *), void *context);
extern void fuxi_backlight_publish(IOService *provider);
extern void fuxi_backlight_display_state(IOService *provider, bool on);
#endif

enum {
	kSweetFBGetLayerDefaultSurface  = 3,
	kSweetFBSwapBegin               = 4,
	kSweetFBSwapEnd                 = 5,
	kSweetFBSwapWait                = 6,
	kSweetFBGetID                   = 7,
	kSweetFBGetDisplaySize          = 8,
	kSweetFBRequestPowerChange      = 12,
	kSweetFBIsMainDisplay           = 18,
	kSweetFBGetDotPitch             = 28,
	kSweetFBNotifications           = 72,
};

#define SWEET_FB_SWAP_REC_SIZE          0x46c
#define SWEET_FB_LAYERS                 4
#define SWEET_FB_QUEUE_DEPTH            3
#define SWEET_FB_SWAP_HISTORY           64
#define SWEET_FB_NOTIFY_TYPES           8
#define SWEET_FB_NOTIFY_VSYNC           5
#define SWEET_FB_VSYNC_US               16667
#define SWEET_FB_DOT_PITCH              458     /* ppi, as the iPhone X panel */
/*
 * The iOS userland only has status bar / layout data for real iPhone
 * screens (UIKit: "No status bar visual provider for screen", the red
 * rdar://45025538 bar for 360x800 points). The display is therefore
 * presented as the iPhone X panel, 1125x2436 (375x812 points @3x), and the
 * centered panel-sized part of it is shown on the panel (boot framebuffer
 * size: 1080x2400 on fuxi, 1080x2412 on salami).
 * Boot-arg sweetfb_native=1 presents the real panel size instead.
 */
#define SWEET_FB_LOGICAL_W              1125
#define SWEET_FB_LOGICAL_H              2436
#define SWEET_FB_FORMAT_BGRA            0x42475241

class IOMobileFramebuffer : public IOService {
	OSDeclareDefaultStructors(IOMobileFramebuffer);
};

OSDefineMetaClassAndStructors(IOMobileFramebuffer, IOService);

class IOMobileFramebufferUserClient;

class AppleCLCD : public IOMobileFramebuffer {
	OSDeclareDefaultStructors(AppleCLCD);

	IOWorkLoop         *fWorkLoop;
	IOTimerEventSource *fVsync;
	uint64_t            fVsyncCount;

	static void vsyncFired(OSObject *owner, IOTimerEventSource *sender);

public:
	uint64_t      fPhys;
	uint32_t      fWidth, fHeight, fRowBytes;
	uint32_t      fLogicalW, fLogicalH;     /* size reported to userland */
	int32_t       fOffX, fOffY;             /* logical origin of the panel */
	uintptr_t     fPixels;          /* kernel mapping of the panel memory */
	IOMemoryMap  *fMap;
	SweetSurface *fSurface;         /* the panel memory as an IOSurface */
	IOLock       *fLock;            /* swaps and client list */
	OSArray      *fClients;
	uint32_t      fSwapID;
	bool          fConsolePaused;
 struct SwapRecord { uint8_t bytes[SWEET_FB_SWAP_REC_SIZE]; SweetSurface *surfaces[SWEET_FB_LAYERS]; uint64_t swapped, ready, writers[SWEET_FB_LAYERS], epochs[SWEET_FB_LAYERS]; bool shown, consumed, reused, stale; unsigned uses; };
 SwapRecord    fPending[SWEET_FB_QUEUE_DEPTH], fLast;
 SwapRecord   *fPresentRecord;
 uint64_t      fReuseBeforeShow;
 struct SwapStatus { uint32_t id; unsigned state; } fSwapStatus[SWEET_FB_SWAP_HISTORY];
 uint32_t      fLatestSwap;
 enum { SwapQueued = 1, SwapConsumed, SwapShown, SwapDropped };
 uint64_t      fWaitPoll[2], fWaitEarly, fWaitLatest;
 unsigned      fOutstandingMax, fStageBlocks[3];
 unsigned      fPendingCount;
 bool          fLastValid, fReplay;
 uint32_t      fQueuedDefers, fOverflowDrops, fReadySkips;
	bool          fDirty;
	uint32_t      fFlushTicks;    /* vsync ticks between panel copies */
	bool          fPowerOff;      /* RequestPowerChange(0) */
#if defined(FUXI)
	bool          fBacklightPowerNotifications;
#endif
	uint32_t      fFadeStep;      /* fade-out steps done while off */
	uint64_t      fTickAbs;       /* vsync period, absolute time units */
	uint64_t      fNextTick;      /* next vsync deadline */
	uint64_t      fStatCopy, fStatCopyMax, fStatStart;
	uint32_t      fStatFrames, fStatSwaps, fStatDropped;
	uint32_t      fGpuFrames, fCpuFrames;
	uint64_t      fGpuTime, fCpuTime;
	bool          fUnixnuDiff, fUnixnuIdle, fTickArmed;
	bool          fShadowValid;
	void         *fShadow;
	uint64_t      fShadowBytes, fDiffStored, fDiffFull;
	uint32_t      fDiffFrames, fIdleStops, fIdleWakes;
	bool          fScanout, fScanoutHeld;
	uint8_t       fScanoutLog;  /* 1 summary, 2 also per frame (fuxigpuscanoutlog=1) */
 uint32_t      fScanoutFrames, fScanoutGpu, fScanoutCpu, fScanoutDefers;
 uint64_t      fScanoutTime, fScanoutMax, fScanoutBytes, fScanoutPrepare;
#if defined(FUXI)
 bool          fVsyncTE, fTeValid, fTeSent;
 struct fg_te_observer fTe;
 uint64_t      fTeOffset, fTeNext, fTeLastNotify, fTePeriodSum;
 unsigned      fTeSentFrame, fTeLocked, fTeFallback;
 uint16_t      fTePhase[256];  /* 128 us bins, locked notifications only */
 bool teVsyncTick(uint64_t &now, bool fallback);
 void teVsyncLog(uint64_t now);
#endif
 void scanoutLog(bool gpu, uint64_t bytes, uint64_t copy, uint64_t prepare);
 bool          fGpuPending, fPublicationPending;
 uint64_t      fPublicationTime, fPublicationStage[6];
 uint32_t      fPublicationID;
 uint64_t      fStage[6], fStageSum[6], fStageMax[6], fLastStart;
 unsigned      fStageN[6], fStageCallbacks[3];
 uint64_t      fWriterWakeQueued, fWakeSum[2], fWakeMax[2];
 unsigned      fWakeN[2];
 bool          fStageGpu;
 bool presentStep(bool flush);
 void stageStats(uint64_t now);
	uint32_t      fGpuBusyTicks, fGpuFailed;
 bool          fFormatAny;     /* sweetfb_fmtcheck=0: copy non-BGRA layers as BGRA */
 uint32_t      fFormatRefused, fFormatSeenCount, fFormatSeen[4];
#if defined(FUXI)
 struct fuxi_unixnu_te fTeGate; /* sweetfb_tegate; read_line unset: off */
 bool          fWakePresent;   /* sweetfb_wakepresent: CPU-path present from SwapEnd */
#endif

	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn requestPowerChange(uint64_t state);
	static IOReturn powerChangeAction(OSObject *owner, void *state,
	    void *, void *, void *);
	void addClient(IOMobileFramebufferUserClient *client);
	void removeClient(IOMobileFramebufferUserClient *client);
	void wakePresentLocked(void);
	bool presentWantedLocked(void);
	void free(void) APPLE_KEXT_OVERRIDE;
	IOReturn swap(const uint8_t *rec);
 IOReturn swapWait(uint32_t id, uint32_t flags, uint64_t timeout);
 void recordDone(SwapRecord &record, bool shown);
 void consumeRecord(SwapRecord &record);
 void retireEarly(const SwapRecord &record);
 bool          fEarlyRetire;
 uint32_t      fEarlyRetires;
 void stopRecords(void);
	bool present(const SwapRecord &record);
 void presentNext(void);
 void observeRecord(SwapRecord &record, bool shown);
 void releaseRecord(SwapRecord &record);
 static void writerRetired(void *context);
#if defined(FUXI)
 thread_call_t fWriterWakeCall;
 uint32_t      fStatEvery;     /* sweetfb_statsframes (30..6000); 0: 600. Last member: none moves */
 static void writerWake(thread_call_param_t context, thread_call_param_t unused);
 static IOReturn writerWakeAction(OSObject *owner, void *, void *, void *, void *);
#endif
 void presentStats(uint64_t took);
 void completePresent(uint64_t took);
};

OSDefineMetaClassAndStructors(AppleCLCD, IOMobileFramebuffer);

class IOMobileFramebufferUserClient : public IOUserClient {
	OSDeclareDefaultStructors(IOMobileFramebufferUserClient);

public:
	AppleCLCD          *fFB;
	task_t              fTask;
	mach_port_t         fPort;
	io_user_reference_t fFunc[SWEET_FB_NOTIFY_TYPES];
	io_user_reference_t fRefcon[SWEET_FB_NOTIFY_TYPES];

	bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn registerNotificationPort(mach_port_t port, UInt32 type,
	    io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target,
	    void *reference) APPLE_KEXT_OVERRIDE;
	void notify(uint32_t type, const uint64_t *values, uint32_t count);
};

OSDefineMetaClassAndStructors(IOMobileFramebufferUserClient, IOUserClient);
#if defined(FUXI)
extern "C" void fuxi_display_kick(void);
extern "C" void fuxi_gpu_abi_stats_log(void);
extern "C" void fuxi_gpu_abi_sched_log(void);
extern "C" int fuxi_gpu_abi_pace(void);
extern "C" int fuxi_gpu_abi_start(void);
#endif

/* NativeMetal c6d29ca (MIT): prioritize the path carrying frames to screen.
 * Here IOTimerEventSource executes vsyncFired/present on its thread-call,
 * under the work-loop gate; raising only the IOWorkLoop thread misses it.
 * See bsd/dev/LICENSE.NativeMetal. Keep scheduling changes opt-in.
 * Default is KERNEL_HIGH, not user default.
 */
static uint32_t sweet_fb_timer_options(void)
{
	uint32_t high = 0;
#if defined(FUXI)
	PE_parse_boot_argn("fuxifbprio", &high, sizeof(high));
#endif
	return high == 1 ? kIOTimerEventSourceOptionsPriorityHigh : kIOTimerEventSourceOptionsDefault;
}

#if defined(FUXI)
/* IOMFBDisplayRefresh.displayMinRefreshInterval: seconds in 0.32 fixed point, a 32-bit OSNumber
 * (CA reads it with CFNumberGetValue as int32; NativeMobileDisplay getRefreshAttributes uses 2^32/hz).
 * 120 Hz -> 35791394. displayRefreshStepMachTime is deliberately not published (VRR-only). */
static uint32_t sweet_fb_refresh_interval(uint32_t hz)
{
 return hz == 0 ? 0 : (uint32_t)(((1ULL << 32) + hz / 2) / hz);
}

/* Opt-in: fuxipanel120=1 only; an explicit sweetfb_hz<=60 keeps CA's 1/60 s heartbeat. */
static bool sweet_fb_refresh_wanted(uint32_t panel120, bool hzPresent, uint32_t hz)
{
 return panel120 == 1 && !(hzPresent && hz <= 60);
}

static void sweet_fb_refresh_publish(AppleCLCD *fb)
{
 uint32_t panel120 = 0, hz = 0;
 PE_parse_boot_argn("fuxipanel120", &panel120, sizeof(panel120));
#if defined(FUXI_SALAMI)
 panel120 = 0;	/* refused on salami: the GPU scanout never switches its panel to 120 Hz */
#endif
 bool hzPresent = PE_parse_boot_argn("sweetfb_hz", &hz, sizeof(hz));
 if (!sweet_fb_refresh_wanted(panel120, hzPresent, hz)) return;
 OSDictionary *refresh = OSDictionary::withCapacity(1);
 OSNumber *interval = OSNumber::withNumber((uint64_t)sweet_fb_refresh_interval(120), 32);
 if (refresh && interval && refresh->setObject("displayMinRefreshInterval", interval) &&
     fb->setProperty("IOMFBDisplayRefresh", refresh))
  IOLog("SWEET FB: IOMFBDisplayRefresh displayMinRefreshInterval=%u (120 Hz)\n", sweet_fb_refresh_interval(120));
 OSSafeReleaseNULL(interval);
 OSSafeReleaseNULL(refresh);
}

#if defined(FUXI_SALAMI)
/* salami ABL scans the splash through VIG0 and VIG1; GPU scanout flips VIG0
 * only and owns INTF1/CTL0, unproven on this board. fuxigpuscanout=1 is refused
 * unless fuxiscansalami=1. Logs the scanout decision and the pe_serial display
 * kick mode (fuxikick, same clamp: above 3 is 0) once, from AppleCLCD::start. */
static uint32_t sweet_fb_salami_scanout(uint32_t scanout)
{
 uint32_t fuxiscansalami = 0, fuxikick = 0;
 PE_parse_boot_argn("fuxiscansalami", &fuxiscansalami, sizeof(fuxiscansalami));
 if (scanout == 1 && fuxiscansalami != 1) {
  IOLog("FUXI SCANOUT: refused on salami (VIG0-only flips, INTF1 ownership unproven); fuxiscansalami=1 overrides\n");
  scanout = 0;
 } else {
  IOLog("FUXI SCANOUT: salami %s (fuxigpuscanout=%u fuxiscansalami=%u)\n",
      scanout == 1 ? "override, GPU scanout allowed" : "off", scanout, fuxiscansalami);
 }
 PE_parse_boot_argn("fuxikick", &fuxikick, sizeof(fuxikick));
 IOLog("FUXI KICK: mode %u (0 none, 1 AR once, 2 AR every kick, 3 START only)\n", fuxikick <= 3 ? fuxikick : 0);
 return scanout;
}
#endif

static void sweet_fb_te_configure(AppleCLCD *fb)
{
 unsigned enabled = 0, offset = 7800; /* 7.3 ms transfer + 0.5 ms margin */
 PE_parse_boot_argn("fuxivsyncte", &enabled, sizeof(enabled));
 fb->fVsyncTE = enabled == 1;
 if (!fb->fVsyncTE) return;
 unsigned requested = offset;
 if (PE_parse_boot_argn("fuxivsynctoff", &requested, sizeof(requested)) && requested <= 16000)
  offset = requested;
 nanoseconds_to_absolutetime((uint64_t)offset*1000, &fb->fTeOffset);
 IOLog("SWEET FB: vsync TE requested offset_us=%u\n", offset);
}

bool AppleCLCD::teVsyncTick(uint64_t &now, bool fallback)
{
 bool wasValid = fTeValid;
 fTeValid = fScanout && fuxi_gpu_scanout_te(&fTe);
 now = mach_absolute_time();
 if (!fTeValid) {
  if (wasValid && fTeLastNotify) {
   /* Resume the old period without a second tick just after the last TE. */
   fNextTick = fTeLastNotify + fTickAbs;
   return now >= fNextTick;
  }
  return fallback;
 }
 /* Offset is a phase: wrap offsets exceeding a faster panel's period.
  * Keep an armed deadline if the next TE is observed just before it fires:
  * replacing it with that TE's deadline would silently drop a notification.
  * Require a confirmed counter for every tick; skip long gaps, never burst. */
 if (!wasValid) fTeSent = false;
 uint64_t offset = fTeOffset % fTe.period;
 unsigned target = fTeSent ? (fTeSentFrame + 1) & 0xffffu : fTe.frame;
 unsigned delta = (fTe.frame - target) & 0xffffu;
 if (fTeSent && delta > 0x7fffu) {
  fTeNext = fTe.edge + fTe.period + offset;
  return false; /* the next target TE has not been observed yet */
 }
 if (!fTeSent || delta == 0 || delta > 1) {
  target = fTe.frame;
  fTeNext = fTe.edge + offset;
 }
 if (now < fTeNext) return false;
 fTeSent = true;
 fTeSentFrame = target;
 fTeNext = delta == 1 ? fTe.edge + offset : fTeNext + fTe.period;
 /* A fallback tick may have occurred just before lock acquisition. */
 if (fTeLastNotify && now - fTeLastNotify < fTe.period/2) return false;
 return true;
}

void AppleCLCD::teVsyncLog(uint64_t now)
{
 fTeLastNotify = now;
 if (fTeValid) {
  fTeLocked++;
  fTePeriodSum += fTe.period;
  uint64_t ns;
  absolutetime_to_nanoseconds((now - fTe.edge) % fTe.period, &ns);
  unsigned bin = (unsigned)(ns/128000);
  fTePhase[bin < 256 ? bin : 255]++;
 } else fTeFallback++;
 if (fTeLocked + fTeFallback != 600) return;
 unsigned p50 = 0, p90 = 0, cumulative = 0;
 for (unsigned i = 0; i < 256; i++) {
  cumulative += fTePhase[i];
  if (fTeLocked && !p50 && cumulative >= (fTeLocked+1)/2) p50 = i*128+64;
  if (fTeLocked && !p90 && cumulative >= (fTeLocked*9+9)/10) p90 = i*128+64;
  fTePhase[i] = 0;
 }
 uint64_t ns = 0;
 if (fTeLocked) absolutetime_to_nanoseconds(fTePeriodSum/fTeLocked, &ns);
 IOLog("SWEET FB: vsync te-locked=%u fallback=%u phase_us %u/%u period_us %llu\n",
     fTeLocked, fTeFallback, p50, p90, (unsigned long long)(ns/1000));
 fTeLocked = fTeFallback = 0;
 fTePeriodSum = 0;
}
#endif

/*
 * sweetfb_bootoff=N (0..8; default 1 on salami, 0 otherwise): drop the first
 * N RequestPowerChange(0) since boot. backboardd sends one off after drawing
 * the Apple logo, before SpringBoard turns the display on again; honoring it
 * blacks the panel for that gap. A dropped off never reaches the power path:
 * fPowerOff, sweet_fb_display_off (touch, keys) and the backlight interests
 * stay on, the panel keeps the last frame, and the matching on finds the
 * display already on (no state change). Every later off (auto-lock, power
 * button) takes the unchanged path. Set once in start before registerService,
 * so before any user client can send a request.
 */
static uint32_t sweet_fb_bootoff_max, sweet_fb_bootoff_done;

static void sweet_fb_bootoff_configure(void)
{
#if defined(FUXI_SALAMI)
 uint32_t n = 1;
#else
 uint32_t n = 0;
#endif
 PE_parse_boot_argn("sweetfb_bootoff", &n, sizeof(n));
 if (n > 8) n = 8;
 sweet_fb_bootoff_max = n;
 if (n) IOLog("SWEET FB: boot display off hold %u (sweetfb_bootoff, 0..8)\n", n);
}

/* True: drop this display-off request. fLock orders it with concurrent requests. */
static bool sweet_fb_bootoff_ignored(AppleCLCD *fb)
{
 uint32_t n = 0;
 if (sweet_fb_bootoff_max == 0) return false;
 IOLockLock(fb->fLock);
 if (sweet_fb_bootoff_done < sweet_fb_bootoff_max) n = ++sweet_fb_bootoff_done;
 IOLockUnlock(fb->fLock);
 if (n) IOLog("SWEET FB: boot display off ignored (%u/%u)\n", n, sweet_fb_bootoff_max);
 return n != 0;
}

#if defined(FUXI)
/*
 * sweetfb_tegate=1 (default off; refused with GPU scanout): before the CPU
 * copy of a frame into the panel buffer, wait until the INTF1 tear-check line
 * count is inside a window at its wrap (fuxi_unixnu_te_gate), so the
 * top-to-bottom copy starts as the panel starts a new fetch and stays ahead of
 * it. Register: INTF1 0x0ae36000 + sweetfb_tereg, 0x2b0 (TEAR_LINE_COUNT,
 * default) or 0x298 (INT_COUNT_VAL): the low 16 bits of 32-bit reads, mapped
 * once here and never written; autorefresh (+0x2b4) is read once for the log.
 * The window is sweetfb_tewin lines (default 64) and opens sweetfb_telead
 * lines (default 32, so the window straddles the wrap and also holds a counter
 * parked at its last line between transfers; 0 opens at the first lines of the
 * frame) before the wrap. The wait
 * sleeps without timer coalescing until sweetfb_teguard us (default 300, at
 * most 2000) before the predicted opening, then polls; it never exceeds one
 * wrap period (17 ms cap), after which the copy goes ahead as a miss, and 120
 * misses in a row turn the gate off. While the gate is on, the software vsync
 * tick follows the counter (sweet_fb_rearm): each tick lands guard + 0.7 ms
 * before a window opening, so a tick-driven present waits about that long
 * instead of up to a frame. Calibration samples the counter every 10 us for up
 * to 70 ms (4 wraps) and logs one line, then its verdict again on a short line
 * ("SWEET FB: tegate on" / "off (reason)"): IOLog cuts lines near 256 bytes.
 */
#define SWEET_FB_INTF1_PA               0x0ae36000ULL
extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
static volatile uint32_t *sweet_fb_te_line;
static int sweet_fb_te_event;

static uint32_t
sweet_fb_te_read(void)
{
 return *sweet_fb_te_line;
}

static void
sweet_fb_te_sleep(uint64_t deadline)
{
 /* Critical urgency: a normal-urgency kernel wait may be coalesced up to 1 ms late. */
 if (assert_wait_deadline_with_leeway((event_t)&sweet_fb_te_event, THREAD_UNINT,
     TIMEOUT_URGENCY_SYS_CRITICAL, deadline, 0) == THREAD_WAITING)
  (void)thread_block(THREAD_CONTINUE_NULL);
}

static void
sweet_fb_tegate_configure(AppleCLCD *fb)
{
 uint32_t enabled = 0, reg = 0x2b0, window = 64, lead = 32, guard = 300;
 PE_parse_boot_argn("sweetfb_tegate", &enabled, sizeof(enabled));
 if (enabled != 1) return;
 PE_parse_boot_argn("sweetfb_tereg", &reg, sizeof(reg));
 PE_parse_boot_argn("sweetfb_tewin", &window, sizeof(window));
 PE_parse_boot_argn("sweetfb_telead", &lead, sizeof(lead));
 PE_parse_boot_argn("sweetfb_teguard", &guard, sizeof(guard));
 if (guard > 2000) guard = 2000;
 if (fb->fScanout || (reg != 0x2b0 && reg != 0x298)) {
  IOLog("SWEET FB: tegate refused: %s\n", fb->fScanout ? "GPU scanout owns the panel" :
      "sweetfb_tereg must be 0x2b0 or 0x298");
  return;
 }
 vm_offset_t intf1 = ml_io_map(SWEET_FB_INTF1_PA, 0x1000);
 if (intf1 == 0) {
  IOLog("SWEET FB: tegate refused: INTF1 not mapped\n");
  return;
 }
 sweet_fb_te_line = (volatile uint32_t *)(intf1 + reg);
 uint32_t autorefresh = *(volatile uint32_t *)(intf1 + 0x2b4);
 struct fuxi_unixnu_te_cal cal;
 bzero(&cal, sizeof(cal));
 uint64_t stop, minPeriod, maxPeriod, cap, ns = 0, periodNs = 0;
 nanoseconds_to_absolutetime(70000000ULL, &stop);
 nanoseconds_to_absolutetime(4000000ULL, &minPeriod);
 nanoseconds_to_absolutetime(40000000ULL, &maxPeriod);
 nanoseconds_to_absolutetime(17000000ULL, &cap);
 uint64_t now = mach_absolute_time();
 stop += now;
 while (cal.wraps < 4 && now < stop) {
  uint32_t line = sweet_fb_te_read() & 0xffffu;
  now = mach_absolute_time();
  fuxi_unixnu_te_sample(&cal, now, line);
  IODelay(10);
 }
 struct fuxi_unixnu_te *te = &fb->fTeGate;
 bool on = fuxi_unixnu_te_finish(&cal, te, minPeriod, maxPeriod) != 0;
 absolutetime_to_nanoseconds(cal.count_ticks, &ns);
 uint64_t perMs = ns ? cal.count_lines * 1000000000ULL / ns : 0;     /* lines per ms x 1000 */
 if (cal.wraps >= 2) absolutetime_to_nanoseconds((cal.wrap_last - cal.wrap_first) / (cal.wraps - 1), &periodNs);
 if (on) {
  fuxi_unixnu_te_window(te, window, lead);
  nanoseconds_to_absolutetime((uint64_t)guard * 1000, &te->guard);
  nanoseconds_to_absolutetime((uint64_t)guard * 1000 + 700000, &te->early);
  if (te->early > te->period / 4) te->early = te->period / 4;
  te->interval = fb->fTickAbs;
  te->budget = te->period < cap ? te->period : cap;
  te->now = mach_absolute_time;
  te->sleep_until = sweet_fb_te_sleep;
  /* Last (release): the armed vsync tick and present() test read_line (acquire) before any other field. */
  fuxi_unixnu_te_publish(te, sweet_fb_te_read);
 }
 uint64_t earlyNs = 0;
 absolutetime_to_nanoseconds(te->early, &earlyNs);
 const char *verdict = on ? "on" : "off (no periodic counter)";
 IOLog("SWEET FB: tegate calibration INTF1+0x%x autorefresh 0x%x: lines %u..%u wraps %u period_us %llu "
     "lines_per_ms %llu.%03llu advanced %llu lines, stalled %u of %u samples; tewin %u telead %u -> "
     "start %u window %u of %u, guard_us %u tick_early_us %llu -> %s\n",
     reg, autorefresh, cal.lo, cal.hi, cal.wraps, periodNs / 1000, perMs / 1000, perMs % 1000,
     cal.count_lines, cal.stalls, cal.samples, window, lead, te->start, te->window, te->span, guard,
     earlyNs / 1000, verdict);
 IOLog("SWEET FB: tegate %s\n", verdict);    /* the line above loses its tail */
}

/*
 * sweetfb_statsframes=N: presented frames per "SWEET FB: stats" line, clamped
 * to 30..6000 (a lightly used phone may not present 600 in a session).
 * Absent: 0, which presentStats() reads as the default 600, and no log.
 */
static uint32_t
sweet_fb_stats_frames(void)
{
 uint32_t frames = 0;
 if (!PE_parse_boot_argn("sweetfb_statsframes", &frames, sizeof(frames))) return 0;
 uint32_t every = frames < 30 ? 30 : frames > 6000 ? 6000 : frames;
 IOLog("SWEET FB: sweetfb_statsframes=%u: stats every %u presented frames\n", frames, every);
 return every;
}
#endif

bool
AppleCLCD::start(IOService *provider)
{
	PE_Video video;

	if (!IOService::start(provider)) {
		return false;
	}
	PE_current_console(&video);
	fPhys = video.v_baseAddr;
	fWidth = (uint32_t)video.v_width;
	fHeight = (uint32_t)video.v_height;
	fRowBytes = (uint32_t)video.v_rowBytes;
	if (fPhys == 0 || fWidth == 0 || fHeight == 0 || fRowBytes < fWidth * 4) {
		IOLog("SWEET FB: no boot framebuffer\n");
		return false;
	}
	int native = 0;
	PE_parse_boot_argn("sweetfb_native", &native, sizeof(native));
	fLogicalW = fWidth;
	fLogicalH = fHeight;
	if (!native && fWidth <= SWEET_FB_LOGICAL_W && fHeight <= SWEET_FB_LOGICAL_H) {
		fLogicalW = SWEET_FB_LOGICAL_W;
		fLogicalH = SWEET_FB_LOGICAL_H;
	}
	fOffX = (int32_t)(fLogicalW - fWidth) / 2;
	fOffY = (int32_t)(fLogicalH - fHeight) / 2;
	fLock = IOLockAlloc();
	fClients = OSArray::withCapacity(4);
	fSurface = sweet_surface_create_physical(fPhys, fWidth, fHeight, fRowBytes, SWEET_FB_FORMAT_BGRA);
	fPixels = fSurface != NULL ? fSurface->kernelAddress() : 0;
	fWorkLoop = IOWorkLoop::workLoop();
	if (fLock == NULL || fClients == NULL || fPixels == 0 || fWorkLoop == NULL) {
		IOLog("SWEET FB: setup failed\n");
		OSSafeReleaseNULL(fSurface);    /* stays published: also held by the table */
		OSSafeReleaseNULL(fWorkLoop);
		return false;
	}
#if defined(FUXI)
	uint32_t diff = 0, idle = 0;
	PE_parse_boot_argn("fuxi_unixnu_diff", &diff, sizeof(diff));
	PE_parse_boot_argn("fuxi_unixnu_idlefb", &idle, sizeof(idle));
	fUnixnuDiff = diff == 1;
	fUnixnuIdle = idle == 1;
	uint32_t early = 0;
	PE_parse_boot_argn("fuxiearly", &early, sizeof(early));
	fEarlyRetire = early == 1;
	if (fUnixnuDiff) {
		fShadowBytes = (uint64_t)fWidth * fHeight * 4;
		if (fWidth > 16384 || fHeight > 16384 || !fShadowBytes || fShadowBytes > (64ULL << 20) ||
		    (fWidth & 1) || (fPixels & 7) || (fRowBytes & 7)) {
			fUnixnuDiff = false;
		} else {
			fShadow = IOMallocAligned((vm_size_t)fShadowBytes, 64);
			if (!fShadow) fUnixnuDiff = false;
		}
		IOLog("FUXI UNIXNU FB: differential copy %s (CPU full-panel BGRA only)\n",
		    fUnixnuDiff ? "enabled" : "unavailable");
	}
	if (fUnixnuIdle) IOLog("FUXI UNIXNU FB: demand timer enabled, software 60 Hz, panel power inherited\n");
	sweet_fb_te_configure(this);
#endif
	uint32_t timerOptions = sweet_fb_timer_options();
	if (timerOptions == kIOTimerEventSourceOptionsPriorityHigh)
		IOLog("FUXI SCANOUT: present timer THREAD_CALL_PRIORITY_HIGH\n");	fVsync = IOTimerEventSource::timerEventSource(timerOptions, this, &AppleCLCD::vsyncFired);
	if (fVsync == NULL || fWorkLoop->addEventSource(fVsync) != kIOReturnSuccess) {
		IOLog("SWEET FB: no vsync timer\n");
		return false;
	}
#if defined(FUXI)
	uint32_t hz = 60;
#else
	uint32_t hz = 10;
#endif
	PE_parse_boot_argn("sweetfb_hz", &hz, sizeof(hz));
	fFlushTicks = (hz == 0 || hz >= 60) ? 1 : 60 / hz;
#if defined(FUXI)
	nanoseconds_to_absolutetime(SWEET_FB_VSYNC_US * 1000ULL, &fTickAbs);
	fNextTick = mach_absolute_time() + fTickAbs;
	fTickArmed = true;
	fStatStart = mach_absolute_time();
	fStatEvery = sweet_fb_stats_frames();
	fVsync->wakeAtTime(kIOTimeOptionsWithLeeway, fNextTick, 0);
	IOLog("SWEET FB: present up to %u Hz (every %u ticks)\n", 60 / fFlushTicks, fFlushTicks);
#else
	fVsync->setTimeoutUS(SWEET_FB_VSYNC_US);
#endif

	setProperty(kIOUserClientClassKey, "IOMobileFramebufferUserClient");
	setProperty("external", false);
	setProperty("DisplayPipePlaneBaseAlignment", 64ULL, 32);
	sweet_fb_bootoff_configure();
	uint32_t fmtcheck = 1;
	PE_parse_boot_argn("sweetfb_fmtcheck", &fmtcheck, sizeof(fmtcheck));
	fFormatAny = fmtcheck == 0;
	if (fFormatAny) IOLog("SWEET FB: format check off, non-BGRA layers copied as BGRA (sweetfb_fmtcheck=0)\n");
#if defined(FUXI)
	int backlight = 0;
	PE_parse_boot_argn("fuxibacklight", &backlight, sizeof(backlight));
	fBacklightPowerNotifications = backlight == 2 || backlight == 3;
	fuxi_backlight_publish(this);
	sweet_fb_refresh_publish(this);
#endif
	registerService();
	IOLog("SWEET FB: AppleCLCD %ux%u stride %u at 0x%llx, default surface %u, presented as %ux%u\n",
	    fWidth, fHeight, fRowBytes, fPhys, fSurface->id, fLogicalW, fLogicalH);

#if defined(FUXI)
 uint32_t scanout = 0, check = 0, log = 0;
 PE_parse_boot_argn("fuxigpuscanout", &scanout, sizeof(scanout));
#if defined(FUXI_SALAMI)
 scanout = sweet_fb_salami_scanout(scanout);
#endif
 PE_parse_boot_argn("fuxigpuscanoutcheck", &check, sizeof(check));
 PE_parse_boot_argn("fuxigpuscanoutlog", &log, sizeof(log));
 int quiet = 0;
 PE_parse_boot_argn("sweetfb_noconsole", &quiet, sizeof(quiet));
 fScanout = scanout == 1 && quiet == 1;
 fScanoutLog = log == 1 ? 2 : fScanout ? 1 : 0;
 if (scanout == 1 && !fScanout)
  IOLog("FUXI SCANOUT: unavailable, requires sweetfb_noconsole=1; existing present path\n");
 if (fScanout) { sweet_console_muted = 1; fConsolePaused = true; }
 fuxi_gpu_scanout_configure(fPixels, fScanout, check);
 if (fScanout) {
  fWriterWakeCall = thread_call_allocate(&AppleCLCD::writerWake, this);
  if (!fWriterWakeCall) return false;
  sweet_surface_present_notify(&AppleCLCD::writerRetired, this);
 }
 sweet_fb_tegate_configure(this);
 /* K2: the CPU path presents a swap from SwapEnd like fScanout's writer wake, not at the next tick. */
 uint32_t wakepresent = 0;
 PE_parse_boot_argn("sweetfb_wakepresent", &wakepresent, sizeof(wakepresent));
 if (wakepresent == 1 && (fScanout || fTeGate.read_line)) {
  /* With the gate a SwapEnd present would only sleep (up to a frame, gated) until the window the locked tick meets anyway. */
  IOLog("SWEET FB: sweetfb_wakepresent ignored, %s\n", fScanout ? "GPU scanout already presents from SwapEnd" :
      "sweetfb_tegate presents at ticks locked to the panel");
 } else if (wakepresent == 1) {
  fWriterWakeCall = thread_call_allocate(&AppleCLCD::writerWake, this);
  fWakePresent = fWriterWakeCall != NULL;
  IOLog("SWEET FB: CPU present from SwapEnd %s (sweetfb_wakepresent=1)\n", fWakePresent ? "on" : "unavailable");
 }
 fuxi_gpu_start(fPhys, fWidth, fHeight, fRowBytes);
	fuxi_perf_start();
#endif
	return true;
}

void
AppleCLCD::free(void)
{
#if defined(FUXI)
 if (fScanout) sweet_surface_present_notify(NULL, NULL);
 if (fWriterWakeCall) { thread_call_cancel_wait(fWriterWakeCall); thread_call_free(fWriterWakeCall); }
#endif
 if (fLock) { IOLockLock(fLock); stopRecords(); IOLockUnlock(fLock); }
	if (fShadow) IOFreeAligned(fShadow, (vm_size_t)fShadowBytes);
	IOMobileFramebuffer::free();
}

/* fLock serializes sleep/wake decisions with swaps, notifications and power requests. */
bool
AppleCLCD::presentWantedLocked(void)
{
	bool notify = false;
	for (unsigned int i = 0; i < fClients->getCount(); i++) {
		IOMobileFramebufferUserClient *client =
		    (IOMobileFramebufferUserClient *)fClients->getObject(i);
		if (client->fFunc[SWEET_FB_NOTIFY_VSYNC] && client->fPort != MACH_PORT_NULL) notify = true;
	}
#if defined(FUXI)
 if (fScanout && (fPublicationPending || fuxi_gpu_scanout_pending())) return true;
#endif
	return fuxi_unixnu_tick_wanted(fPowerOff, fFadeStep == 0, fDirty, fGpuPending, notify);
}

void
AppleCLCD::wakePresentLocked(void)
{
#if defined(FUXI)
	if (!fUnixnuIdle || fTickArmed || !presentWantedLocked()) return;
	fTickArmed = true;
	fNextTick = mach_absolute_time() + fTickAbs;
	fVsync->wakeAtTime(kIOTimeOptionsWithLeeway, fNextTick, 0);
	if (++fIdleWakes == 1) IOLog("FUXI UNIXNU FB: timer wake\n");
#endif
}

static void
sweet_fb_rearm(AppleCLCD *fb, IOTimerEventSource *sender)
{
#if defined(FUXI)
 if (fb->fScanout && fuxi_gpu_scanout_owned()) {
  /* Poll ownership/fences at ~1 ms while active, independently of 60 Hz
   * notification deadlines. No extra notification or vsync count per poll. */
  IOLockLock(fb->fLock);
  if (fb->fUnixnuIdle && !fb->presentWantedLocked()) {
   fb->fTickArmed = false;
   IOLockUnlock(fb->fLock);
   return;
  }
  uint64_t now = mach_absolute_time();
  if (now >= fb->fNextTick)
   fb->fNextTick = fuxi_unixnu_next_tick(fb->fNextTick, fb->fTickAbs, now);
  uint64_t deadline = fb->fNextTick;
  bool active = fb->fGpuPending || fb->fPublicationPending || fuxi_gpu_scanout_pending() || (!fb->fScanoutHeld &&
      (fb->fPowerOff ? fb->fFadeStep == 0 : fb->fDirty));
  /* fuxigpuncimport bit 2048: poll the in-flight scanout copy at 1/64 tick so its surface lease is released at retire. */
  uint64_t poll = now + fb->fTickAbs/((fb->fGpuPending && fuxi_gpu_abi_pace()) || ((fb->fPublicationPending || fuxi_gpu_scanout_pending()) && fuxi_gpu_abi_start()) ? 64 : 16);
  if (fb->fVsyncTE) {
   if (fb->fTeValid) {
    deadline = fb->fTeNext;
    poll = now + fb->fTe.period/16;
   }
   /* Keep observing TE even with subscribers but no dirty swap. */
   if (poll < deadline || deadline <= now) deadline = poll;
  } else if (active && poll < deadline) deadline = poll;
  fb->fTickArmed = true;
  sender->wakeAtTime(kIOTimeOptionsWithLeeway, deadline, 0);
  IOLockUnlock(fb->fLock);
  return;
 }
	if (!fb->fUnixnuIdle) {
		/* Preserve the existing continuous-tick path when the option is absent.
		 * A SwapEnd present (writer wake) keeps the armed tick: never advance it early.
		 * sweetfb_tegate: each tick lands just before a counter window (fuxi_unixnu_te_tick). */
		uint64_t now = mach_absolute_time();
  if (!(fb->fScanout || fb->fWakePresent) || now >= fb->fNextTick)
   fb->fNextTick = fuxi_unixnu_te_on(&fb->fTeGate) ? fuxi_unixnu_te_tick(&fb->fTeGate) :
       fuxi_unixnu_next_tick(fb->fNextTick, fb->fTickAbs, now);
		sender->wakeAtTime(kIOTimeOptionsWithLeeway, fb->fNextTick, 0);
		return;
	}
	/* Keep scheduling and wakeups under the same lock: no lost idle->swap wake. */
	IOLockLock(fb->fLock);
	if (!fb->presentWantedLocked()) {
		fb->fTickArmed = false;
		if (++fb->fIdleStops == 1) IOLog("FUXI UNIXNU FB: timer idle\n");
		IOLockUnlock(fb->fLock);
		return;
	}
	uint64_t now = mach_absolute_time();
 if (!(fb->fScanout || fb->fWakePresent) || now >= fb->fNextTick)
  fb->fNextTick = fuxi_unixnu_te_on(&fb->fTeGate) ? fuxi_unixnu_te_tick(&fb->fTeGate) :
      fuxi_unixnu_next_tick(fb->fNextTick, fb->fTickAbs, now);
	sender->wakeAtTime(kIOTimeOptionsWithLeeway, fb->fNextTick, 0);
	IOLockUnlock(fb->fLock);
#else
	(void)fb;
	sender->setTimeoutUS(SWEET_FB_VSYNC_US);
#endif
}

void
AppleCLCD::vsyncFired(OSObject *owner, IOTimerEventSource *sender)
{
	AppleCLCD *fb = (AppleCLCD *)owner;
	uint64_t now = mach_absolute_time();
 bool vsyncTick = true;
#if defined(FUXI)
 if (fb->fScanout || fb->fWakePresent) vsyncTick = now >= fb->fNextTick;
 if (fb->fVsyncTE) {
  vsyncTick = fb->teVsyncTick(now, vsyncTick);
  if (vsyncTick) fb->teVsyncLog(now);
 }
#endif
 uint64_t count = vsyncTick ? ++fb->fVsyncCount : fb->fVsyncCount;
	uint64_t values[6] = {
		(uint32_t)count, (uint32_t)(count >> 32),
		(uint32_t)now, (uint32_t)(now >> 32),
		0, 0,
	};

	IOLockLock(fb->fLock);
	for (unsigned int i = 0; i < fb->fClients->getCount(); i++) {
		IOMobileFramebufferUserClient *client =
		    (IOMobileFramebufferUserClient *)fb->fClients->getObject(i);
		if (vsyncTick) client->notify(SWEET_FB_NOTIFY_VSYNC, values, 6);
	}
 IOLockUnlock(fb->fLock);
#if defined(FUXI)
 fb->fStageCallbacks[vsyncTick ? 0 : 1]++;
 bool flush = (count % fb->fFlushTicks) == 0 && (vsyncTick || fb->fFlushTicks == 1);
 if (!fb->presentStep(flush)) { sweet_fb_rearm(fb, sender); return; }
#endif
 IOLockLock(fb->fLock);
	if (fb->fPowerOff) {
		/* Fade out: halve every channel each vsync, then black, then idle. */
		uint32_t step = fb->fFadeStep;
		if (step <= 6) {
			fb->fFadeStep = step + 1;
		}
		IOLockUnlock(fb->fLock);
#if defined(FUXI)
		if (step == 0) {
			fb->fShadowValid = false;
			if (fb->fUnixnuDiff) {
				fuxi_unixnu_fill_pixels((volatile void *)fb->fPixels,
				    (uint64_t)fb->fRowBytes * fb->fHeight, fb->fRowBytes,
				    fb->fWidth, fb->fHeight, 0);
			} else {
				/* Plain stores, no DC ZVA (bzero) on the write-combined mapping. */
				for (uint32_t y = 0; y < fb->fHeight; y++) {
					volatile uint64_t *row = (volatile uint64_t *)(fb->fPixels + (uint64_t)y * fb->fRowBytes);
					for (uint32_t x = 0; x < fb->fWidth / 2; x++) {
						row[x] = 0;
					}
				}
			}
			__asm__ volatile ("dsb sy" ::: "memory");
            if (fb->fScanout && fuxi_gpu_scanout_publish() == 0) fuxi_display_kick();
		}
#else
		if (step <= 6) {
			for (uint32_t y = 0; y < fb->fHeight; y++) {
				volatile uint32_t *row = (volatile uint32_t *)(fb->fPixels + (uint64_t)y * fb->fRowBytes);
				for (uint32_t x = 0; x < fb->fWidth; x++) {
					row[x] = step < 6 ? (row[x] >> 1) & 0x7f7f7f7f : 0;
				}
			}
		}
#endif
		sweet_fb_rearm(fb, sender);
		return;
	}
#if !defined(FUXI)
 bool flush = fb->fDirty && (count % fb->fFlushTicks) == 0;
#endif
 IOLockUnlock(fb->fLock);
#if !defined(FUXI)
 if (flush) fb->presentNext();
#endif
	sweet_fb_rearm(fb, sender);
}

void
AppleCLCD::addClient(IOMobileFramebufferUserClient *client)
{
	IOLockLock(fLock);
	fClients->setObject(client);
	IOLockUnlock(fLock);
}

void
AppleCLCD::removeClient(IOMobileFramebufferUserClient *client)
{
	IOLockLock(fLock);
	unsigned int i = fClients->getNextIndexOfObject(client, 0);
	if (i != (unsigned int)-1) {
		fClients->removeObject(i);
	}
	IOLockUnlock(fLock);
}

static uint32_t
get32(const uint8_t *p, uint32_t off)
{
	uint32_t v;
	memcpy(&v, p + off, 4);
	return v;
}

static uint32_t sweet_fb_swaps_logged;
static uint64_t sweet_fb_swap_count;

void AppleCLCD::observeRecord(SwapRecord &record, bool shown)
{
#if defined(FUXI)
 if (!record.shown && !record.consumed) for (unsigned i = 0; i < SWEET_FB_LAYERS; i++) if (record.surfaces[i]) {
  uint64_t epoch = 0, serial = sweet_surface_writer_serial(record.surfaces[i], &epoch);
  if (epoch != record.epochs[i]) record.stale = true;
  if (!record.reused && serial != record.writers[i]) {
   record.reused = true; if (fReuseBeforeShow != ~0ull) fReuseBeforeShow++;
  }
 }
#endif
 record.shown |= shown;
}

void AppleCLCD::consumeRecord(SwapRecord &record)
{
 observeRecord(record, false); record.consumed = true;
 for (unsigned i = 0; i < SWEET_FB_LAYERS; i++) if (record.uses & (1u << i)) sweet_surface_display_drop(record.surfaces[i]);
 record.uses = 0;
 uint32_t id = get32(record.bytes, 0x50);
 SwapStatus &status = fSwapStatus[id % SWEET_FB_SWAP_HISTORY];
 if (id && status.id == id && status.state == SwapQueued) { status.state = SwapConsumed; IOLockWakeup(fLock, fSwapStatus, false); }
}

/* NativeMobileDisplay (BSD-2-Clause-Patent) early retire: the GPU copy owns the frame, so SwapWait(flags 0)
 * may return now. displayUses stay held (released at the fence) and the writer lock still blocks CA on the
 * surface until the copy retires. */
void AppleCLCD::retireEarly(const SwapRecord &record)
{
 uint32_t id = get32(record.bytes, 0x50);
 IOLockLock(fLock);
 SwapStatus &status = fSwapStatus[id % SWEET_FB_SWAP_HISTORY];
 if (id && status.id == id && status.state == SwapQueued) { status.state = SwapConsumed; fEarlyRetires++; IOLockWakeup(fLock, fSwapStatus, false); }
 IOLockUnlock(fLock);
}

void AppleCLCD::recordDone(SwapRecord &record, bool shown)
{
 consumeRecord(record); record.shown |= shown;
 uint32_t id = get32(record.bytes, 0x50);
 SwapStatus &status = fSwapStatus[id % SWEET_FB_SWAP_HISTORY];
 if (id && status.id == id && (shown || !fPublicationPending || id != fPublicationID)) { status.state = record.shown || status.state == SwapShown ? SwapShown : SwapDropped; IOLockWakeup(fLock, fSwapStatus, false); }
}

void AppleCLCD::releaseRecord(SwapRecord &record)
{
 recordDone(record, false);
 for (unsigned i = 0; i < SWEET_FB_LAYERS; i++) OSSafeReleaseNULL(record.surfaces[i]);
}

void AppleCLCD::writerRetired(void *context)
{
 #if defined(FUXI)
 AppleCLCD *fb = (AppleCLCD *)context;
 uint64_t empty = 0, now = mach_absolute_time();
 (void)__atomic_compare_exchange_n(&fb->fWriterWakeQueued, &empty, now, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED);
 (void)thread_call_enter(fb->fWriterWakeCall);
#else
 (void)context;
#endif
}

#if defined(FUXI)
void AppleCLCD::writerWake(thread_call_param_t context, thread_call_param_t)
{
 AppleCLCD *fb = (AppleCLCD *)context;
 uint64_t stamps[2] = { __atomic_exchange_n(&fb->fWriterWakeQueued, 0, __ATOMIC_RELAXED), mach_absolute_time() };
 fb->fWorkLoop->runAction(&AppleCLCD::writerWakeAction, fb, stamps);
}
IOReturn AppleCLCD::writerWakeAction(OSObject *owner, void *arg, void *, void *, void *)
{
 AppleCLCD *fb = (AppleCLCD *)owner;
 if (arg) {
  uint64_t *stamps = (uint64_t *)arg, now = mach_absolute_time();
  for (unsigned i = 0; i < 2; i++) {
   uint64_t a = stamps[i], b = i ? now : stamps[1];
   if (!a || b < a) continue;
   uint64_t d = b-a; fb->fWakeSum[i] += d; if (d > fb->fWakeMax[i]) fb->fWakeMax[i] = d; fb->fWakeN[i]++;
  }
 }
 fb->fStageCallbacks[2]++;
 IOLockLock(fb->fLock);
 bool wanted = !fb->fPowerOff && !fb->fScanoutHeld && (fb->fDirty || fb->fGpuPending || fb->fPublicationPending);
 IOLockUnlock(fb->fLock);
 if (wanted) { (void)fb->presentStep(fb->fFlushTicks == 1); sweet_fb_rearm(fb, fb->fVsync); }
 return kIOReturnSuccess;
}
#endif

bool AppleCLCD::presentStep(bool flush)
{
#if defined(FUXI)
 uint64_t elapsed = 0;
 int completed = sweet_surface_gpu_poll(&elapsed);
 if (!completed) { fGpuBusyTicks++; fScanoutDefers++; return false; }
 if (fScanoutHeld) {
  fGpuPending = false;
  IOLockLock(fLock); stopRecords(); IOLockUnlock(fLock); return false;
 }
 if (fGpuPending) {
  fGpuPending = false;
  if (completed == 1) {
   struct fg_present_times times; fuxi_gpu_scanout_stamps(&times);
   fStage[4] = times.fence;
   fGpuFrames++; fGpuTime += elapsed;
   completePresent(elapsed + fScanoutPrepare);
   scanoutLog(true, fScanoutBytes, elapsed, fScanoutPrepare);
  } else {
   fGpuFailed++;
   IOLockLock(fLock); fReplay = fDirty = true; IOLockUnlock(fLock);
  }
 } else if (fScanout) {
  int published = fuxi_gpu_scanout_retry();
  if (published == 1 && fPublicationPending) presentStats(fPublicationTime);
  else if (published < 0) fScanoutHeld = true;
 }
 if (fScanoutHeld) { IOLockLock(fLock); stopRecords(); IOLockUnlock(fLock); return false; }
 if (fScanout && fuxi_gpu_scanout_pending() && (!fuxi_gpu_scanout_triple() || fPowerOff)) { fStageBlocks[0]++; fScanoutDefers++; return false; }
 IOLockLock(fLock);
 bool off = fPowerOff, wanted = !fScanoutHeld && (off ? fFadeStep == 0 : fDirty && flush);
 IOLockUnlock(fLock);
 if (!wanted) return off && !fScanoutHeld;
 if (fScanout) {
  int writable = off ? fuxi_gpu_scanout_cpu_acquire() : fuxi_gpu_scanout_acquire();
  if (writable != 1) { fStageBlocks[1]++; fScanoutHeld = writable < 0; fScanoutDefers++; if (fScanoutHeld) { IOLockLock(fLock); stopRecords(); IOLockUnlock(fLock); } return false; }
 }
 if (off) return true;
 presentNext();
#else
 (void)flush;
#endif
 return false;
}

void AppleCLCD::presentNext(void)
{
 SwapRecord record = {};
 IOLockLock(fLock);
 unsigned chosen = 0;
 bool replay = fReplay || !fPendingCount;
 for (;;) {
  if (replay && !fLastValid) { fReplay = false; fDirty = fPendingCount != 0; IOLockUnlock(fLock); return; }
  SwapRecord &candidate = replay ? fLast : fPending[chosen];
  observeRecord(candidate, false);
  if (candidate.stale && !candidate.shown) {
   if (replay) { releaseRecord(fLast); fLastValid = fReplay = false; }
   else {
    releaseRecord(candidate); fPendingCount--;
    for (unsigned i = chosen; i < fPendingCount; i++) fPending[i] = fPending[i+1];
   }
   fReadySkips++; fStatDropped++; chosen = 0; replay = fPendingCount == 0;
   if (replay) { fDirty = false; IOLockUnlock(fLock); return; }
   continue;
  }
#if defined(FUXI)
  SweetSurface *sources[SWEET_FB_LAYERS]; uint64_t epochs[SWEET_FB_LAYERS]; unsigned count = 0;
  uint64_t ready = candidate.swapped;
  for (unsigned i = 0; i < SWEET_FB_LAYERS; i++) if (candidate.surfaces[i]) { epochs[count] = candidate.epochs[i]; sources[count++] = candidate.surfaces[i]; }
  if (fScanout && !sweet_surface_present_begin(sources, count, &ready, replay && candidate.shown ? NULL : epochs)) {
   fQueuedDefers++;
   if (replay && fPendingCount) { replay = false; continue; }
   if (!replay && ++chosen < fPendingCount) continue;
   IOLockUnlock(fLock); return;
  }
  candidate.ready = ready;
#endif
  if (replay && candidate.shown) {
   candidate.consumed = false;
   bool held = true;
   for (unsigned i = 0; i < SWEET_FB_LAYERS; i++) if (candidate.surfaces[i]) {
    if (sweet_surface_display_hold(candidate.surfaces[i])) candidate.uses |= 1u << i; else held = false;
#if defined(FUXI)
    candidate.writers[i] = sweet_surface_writer_serial(candidate.surfaces[i], &candidate.epochs[i]);
#endif
   }
   if (!held) {
#if defined(FUXI)
    if (fScanout) sweet_surface_present_end();
#endif
    recordDone(candidate, true); IOLockUnlock(fLock); return;
   }
  }
  record = candidate;
  if (replay) {
   bzero(&fLast, sizeof(fLast)); fLastValid = false; fReplay = false;
  } else {
   for (unsigned i = 0; i < chosen; i++) releaseRecord(fPending[i]);
   if (fReplay) { fReplay = false; fReadySkips++; fStatDropped++; }
   fReadySkips += chosen; fStatDropped += chosen;
   fPendingCount -= chosen + 1;
   for (unsigned i = 0; i < fPendingCount; i++) fPending[i] = fPending[i + chosen + 1];
  }
  break;
 }
 fPresentRecord = &record;
 IOLockUnlock(fLock);
 fStage[0] = record.swapped; fStage[1] = record.ready; fStage[2] = mach_absolute_time();
 fStage[3] = fStage[4] = fStage[5] = 0; fStageGpu = false;
 bool accepted = present(record);
 IOLockLock(fLock);
 fPresentRecord = NULL;
 if (accepted) {
  releaseRecord(fLast); fLast = record; fLastValid = true;
 } else if (fPendingCount == SWEET_FB_QUEUE_DEPTH) {
  releaseRecord(record); fOverflowDrops++; fStatDropped++;
 } else {
  for (unsigned i = fPendingCount; i; i--) fPending[i] = fPending[i-1];
  fPending[0] = record; fPendingCount++;
 }
 fDirty = fPendingCount != 0 || fReplay;
 if (fScanoutHeld && !fGpuPending) stopRecords();
 IOLockUnlock(fLock);
}

IOReturn
AppleCLCD::swap(const uint8_t *rec)
{
#if defined(FUXI)
 fuxi_perf_swap();
#endif
 IOLockLock(fLock);
 uint32_t id = get32(rec, 0x50);
 SwapStatus &status = fSwapStatus[id % SWEET_FB_SWAP_HISTORY];
 if (!id || id > fSwapID || status.id == id || status.state == SwapQueued || status.state == SwapConsumed || fScanoutHeld) {
  IOLockUnlock(fLock); return kIOReturnNotReady;
 }
 SwapRecord record = {};
 memcpy(record.bytes, rec, SWEET_FB_SWAP_REC_SIZE); record.swapped = mach_absolute_time();
 bool held = true;
 for (unsigned i = 0; i < SWEET_FB_LAYERS; i++) {
  uint32_t surfaceID = get32(rec, 0x54 + 4*i);
  if (!(get32(rec, 0x104) & (1u << i)) || !surfaceID) continue;
  record.surfaces[i] = sweet_surface_copy(surfaceID);
  if (record.surfaces[i] == fSurface) OSSafeReleaseNULL(record.surfaces[i]);
  else if (!record.surfaces[i]) held = false;
  if (record.surfaces[i]) {
   if (sweet_surface_display_hold(record.surfaces[i])) record.uses |= 1u << i; else held = false;
#if defined(FUXI)
   record.writers[i] = sweet_surface_writer_serial(record.surfaces[i], &record.epochs[i]);
#endif
  }
 }
 if (!held) { releaseRecord(record); IOLockUnlock(fLock); return kIOReturnNoResources; }
 if (fPendingCount == SWEET_FB_QUEUE_DEPTH) {
  releaseRecord(fPending[0]);
  for (unsigned i = 1; i < fPendingCount; i++) fPending[i-1] = fPending[i];
  fPendingCount--; fOverflowDrops++; fStatDropped++;
 }
 status.id = id; status.state = SwapQueued; fLatestSwap = id;
 unsigned outstanding = 0;
 for (unsigned i = 0; i < SWEET_FB_SWAP_HISTORY; i++) if (fSwapStatus[i].state == SwapQueued || fSwapStatus[i].state == SwapConsumed) outstanding++;
 if (outstanding > fOutstandingMax) fOutstandingMax = outstanding;
 fPending[fPendingCount++] = record;
 fStatSwaps++; fDirty = true; wakePresentLocked();
 IOLockUnlock(fLock);
#if defined(FUXI)
 if ((fScanout || fWakePresent) && fWriterWakeCall && fFlushTicks == 1) writerRetired(this);
#endif
 if ((++sweet_fb_swap_count % 1000) == 0) IOLog("SWEET FB: %llu swaps\n", sweet_fb_swap_count);
 return kIOReturnSuccess;
}

IOReturn AppleCLCD::swapWait(uint32_t id, uint32_t flags, uint64_t timeout)
{
 if (flags & ~0x80000003u) return kIOReturnBadArgument;
 uint64_t duration = 0;
 nanoseconds_to_absolutetime(timeout && timeout < 100000000ull ? timeout : 100000000ull, &duration);
 uint64_t deadline = mach_absolute_time() + duration;
 IOLockLock(fLock);
 fWaitPoll[flags & 1]++;
 if (!id) fWaitLatest++;
 if (!id && !(flags & 2)) id = fLatestSwap;
 IOReturn result;
 for (;;) {
  result = kIOReturnBusy;
  if (fScanoutHeld) result = kIOReturnNotReady;
  else if (flags & 2) {
   bool pending = fPresentRecord != NULL;
   for (unsigned i = 0; i < SWEET_FB_SWAP_HISTORY; i++) if (fSwapStatus[i].state == SwapQueued || ((flags & 1) && fSwapStatus[i].state == SwapConsumed)) pending = true;
   if (!pending) result = kIOReturnSuccess;
  } else {
   SwapStatus &status = fSwapStatus[id % SWEET_FB_SWAP_HISTORY];
   if (!id || status.id != id) result = kIOReturnNotFound;
   else if (status.state == SwapShown) result = kIOReturnSuccess;
   else if (status.state == SwapConsumed && !(flags & 1)) { result = kIOReturnSuccess; fWaitEarly++; }
   else if (status.state == SwapDropped) result = flags & 1 ? kIOReturnAborted : kIOReturnSuccess;
  }
  if (result != kIOReturnBusy || (flags & 0x80000000u)) break;
  if (mach_absolute_time() >= deadline) { result = kIOReturnTimeout; break; }
  int slept = IOLockSleepDeadline(fLock, fSwapStatus, deadline, THREAD_ABORTSAFE);
  if (slept == THREAD_INTERRUPTED) { result = kIOReturnAborted; break; }
 }
 IOLockUnlock(fLock);
 return result;
}

void AppleCLCD::stopRecords(void)
{
#if defined(FUXI)
 if (fGpuPending) fScanoutHeld = true;
 if (fGpuPending) for (unsigned i = 0; i < SWEET_FB_LAYERS; i++) if (fLast.surfaces[i]) sweet_surface_display_poison(fLast.surfaces[i]);
#endif
 for (unsigned i = 0; i < fPendingCount; i++) releaseRecord(fPending[i]);
 if (fPublicationPending) {
  SwapStatus &status = fSwapStatus[fPublicationID % SWEET_FB_SWAP_HISTORY];
  if (status.id == fPublicationID && status.state != SwapShown) status.state = SwapDropped;
  fPublicationPending = false;
 }
 fPendingCount = 0; releaseRecord(fLast); fLastValid = fReplay = fDirty = false;
 IOLockWakeup(fLock, fSwapStatus, false);
}

bool
AppleCLCD::present(const SwapRecord &record)
{
 const uint8_t *rec = record.bytes;
	uint32_t mask = get32(rec, 0x104);
	uint64_t t0 = mach_absolute_time();
	bool gpuFrame = false, cpuFrame = false;
#if defined(FUXI)
 struct PresentRead { bool held; ~PresentRead() { if (held) sweet_surface_present_end(); } } read = { fScanout };
 bool teGated = false;
 uint64_t teWait = 0;
#endif
 uint64_t cpuBytes = 0, cpuTicks = 0;

	if (!fConsolePaused) {
		int quiet = 0;
		fConsolePaused = true;
		if (PE_parse_boot_argn("sweetfb_noconsole", &quiet, sizeof(quiet)) && quiet != 0) {
			/* Not PE_initialize_console(kPEDisableScreen): its console switch
			 * writes the read-only-late cons_ops_index and panics. */
			sweet_console_muted = 1;
		}
	}

	for (uint32_t layer = 0; layer < SWEET_FB_LAYERS; layer++) {
		uint32_t id = get32(rec, 0x54 + 4 * layer);
		const uint8_t *a = rec + 0x64 + 16 * layer;
		const uint8_t *b = rec + 0xc4 + 16 * layer;
		int32_t sx = (int32_t)get32(a, 0), sy = (int32_t)get32(a, 4);
		int32_t dx = (int32_t)get32(b, 0), dy = (int32_t)get32(b, 4);
		int32_t w = (int32_t)get32(b, 8), h = (int32_t)get32(b, 12);

		if (!(mask & (1u << layer)) || id == 0) {
			continue;
		}
		SweetSurface *s = record.surfaces[layer]; if (s) s->retain();
		if (sweet_fb_swaps_logged < 16) {
			sweet_fb_swaps_logged++;
			IOLog("SWEET FB: swap %u layer %u surface %u %ux%u fmt 0x%x bpr %u a %d,%d %ux%u b %d,%d %dx%d\n",
			    get32(rec, 0x50), layer, id, s ? s->width : 0, s ? s->height : 0,
			    s ? s->pixelFormat : 0, s ? s->bytesPerRow : 0, sx, sy, get32(a, 8),
			    get32(a, 12), dx, dy, w, h);
		}
		if (s == NULL || s == fSurface) {
			fShadowValid = false;
			OSSafeReleaseNULL(s);
			continue;
		}
		uintptr_t src = s->kernelAddress();
		uint32_t bpe = s->bytesPerElement ? s->bytesPerElement : 4;
		if (w <= 0 || h <= 0) {
			w = (int32_t)s->width;
			h = (int32_t)s->height;
		}
		if (sx < 0 || sy < 0) {
			sx = sy = 0;
		}
		if (dx < 0 || dy < 0) {
			dx = dy = 0;
		}
		/* Logical (presented) display coordinates -> panel coordinates. */
		dx -= fOffX;
		dy -= fOffY;
		if (dx < 0) {
			sx -= dx;
			w += dx;
			dx = 0;
		}
		if (dy < 0) {
			sy -= dy;
			h += dy;
			dy = 0;
		}
		if (w > (int32_t)s->width - sx) {
			w = (int32_t)s->width - sx;
		}
		if (h > (int32_t)s->height - sy) {
			h = (int32_t)s->height - sy;
		}
		if (w > (int32_t)fWidth - dx) {
			w = (int32_t)fWidth - dx;
		}
		if (h > (int32_t)fHeight - dy) {
			h = (int32_t)fHeight - dy;
		}
		uint64_t last = (uint64_t)s->offset + (uint64_t)(sy + h - 1) * s->bytesPerRow +
		    (uint64_t)(sx + w) * 4;
		if (w <= 0 || h <= 0 || s->elementWidth != 1 || s->elementHeight != 1 ||
		    s->memory == NULL || last > s->memory->getLength()) {
			src = 0;
		}
		if (src != 0 && bpe == 4) {
			int copied = 0;
#if defined(FUXI)
            /* Keep CPU replay complete too if a full composed crop is available. */
            if (fuxi_gpu_scanout_double() && (mask & (mask-1)) == 0 && sx >= dx && sy >= dy &&
                (uint64_t)(sx-dx)+fWidth <= s->width && (uint64_t)(sy-dy)+fHeight <= s->height &&
                ((uint64_t)(sx-dx)+fWidth)*4 <= s->bytesPerRow &&
                (uint64_t)s->offset + ((uint64_t)(sy-dy)+fHeight-1)*s->bytesPerRow +
                ((uint64_t)(sx-dx)+fWidth)*4 <= s->memory->getLength()) {
             sx -= dx; sy -= dy; dx = dy = 0; w = fWidth; h = fHeight;
            }
			uint64_t copyStart = mach_absolute_time();
			/* CA's composed frame has one layer. Preserve the CPU path for
			 * multi-layer records rather than submit multiple fences per frame. */
			if ((mask & (mask-1)) == 0)
				copied = sweet_surface_gpu_copy(s, src, sx, sy, dx, dy, w, h);
            if (copied == 3 || (copied < 0 && fScanout)) {
             fStageBlocks[2]++;
             fScanoutDefers++;
             s->release();
             return false; /* busy ring: keep the record for retry */
            }
			if (copied == 2) {
             fScanoutPrepare = mach_absolute_time() - copyStart;
             struct fg_present_times times; fuxi_gpu_scanout_stamps(&times);
             fStageGpu = true; fStage[3] = times.submitted;
             if (read.held) { sweet_surface_present_end(); read.held = false; }
             if (fEarlyRetire) retireEarly(record);
             fScanoutBytes = fuxi_gpu_scanout_double() ? (uint64_t)fWidth * fHeight * 4 : (uint64_t)w * h * 4;
				fShadowValid = false;
				gpuFrame = true;
				fGpuPending = true;
			}
            if (copied != 2 && fScanout) {
             int writable = fuxi_gpu_scanout_cpu_acquire();
             if (writable != 1) {
              fScanoutHeld = writable < 0;
              fScanoutDefers++;
              s->release();
              return false;
             }
            }
#endif
			/* The CPU copies move BGRA words: another 4-byte format would show wrong colours. */
			bool refused = copied != 2 && s->pixelFormat != SWEET_FB_FORMAT_BGRA && !fFormatAny;
			if (refused) {
				fFormatRefused++;
				if (fuxi_unixnu_format_new(fFormatSeen, &fFormatSeenCount, 4, s->pixelFormat))
					IOLog("SWEET FB: layer %u surface %u format 0x%x is not BGRA: not copied (sweetfb_fmtcheck=0 copies it)\n",
					    layer, id, s->pixelFormat);
			}
#if defined(FUXI)
			/* sweetfb_tegate: once per frame, before the first CPU store into the panel buffer. */
			if (copied != 2 && !refused && !teGated && fuxi_unixnu_te_on(&fTeGate)) {
				teGated = true;
				teWait = fuxi_unixnu_te_gate(&fTeGate);
				if (fTeGate.disabled)
					IOLog("SWEET FB: tegate off after %u misses in a row\n", fTeGate.streak);
			}
			copyStart = mach_absolute_time();
#endif
			bool diffCopied = false;
#if defined(FUXI)
			if (copied != 2 && fUnixnuDiff && sweet_console_muted &&
			    s->pixelFormat == SWEET_FB_FORMAT_BGRA && (mask & (mask - 1)) == 0 &&
			    dx == 0 && dy == 0 && w == (int32_t)fWidth && h == (int32_t)fHeight) {
				uint64_t offset = (uint64_t)s->offset + (uint64_t)sy * s->bytesPerRow + (uint64_t)sx * 4;
				uint64_t stored = fuxi_unixnu_copy_diff((volatile void *)fPixels,
				    (uint64_t)fRowBytes * fHeight, fRowBytes, (const void *)(src + offset),
				    s->memory->getLength() - offset, s->bytesPerRow, fWidth, fHeight,
				    fShadow, fShadowBytes, fShadowValid);
				if (stored != UINT64_MAX) {
					diffCopied = fShadowValid = true;
					fDiffStored += stored;
					fDiffFull += (uint64_t)fWidth * fHeight * 4;
					if (++fDiffFrames == 1 || fDiffFrames % 600 == 0)
						IOLog("FUXI UNIXNU FB: diff %u frames stored %llu of %llu bytes\n",
						    fDiffFrames, fDiffStored, fDiffFull);
				}
			}
#endif
			if (copied != 2 && !diffCopied && !refused) {
				fShadowValid = false;
#if defined(FUXI)
				if (fUnixnuDiff) {
					uint64_t offset = (uint64_t)s->offset + (uint64_t)sy * s->bytesPerRow + (uint64_t)sx * 4;
					uint64_t dest = (uint64_t)dy * fRowBytes + (uint64_t)dx * 4;
					fuxi_unixnu_copy_pixels((volatile void *)(fPixels + dest),
					    (uint64_t)fRowBytes * fHeight - dest, fRowBytes, (const void *)(src + offset),
					    s->memory->getLength() - offset, s->bytesPerRow, w, h);
				} else
#endif
				for (int32_t y = 0; y < h; y++) {
					memcpy((void *)(fPixels + (uint64_t)(dy + y) * fRowBytes + (uint64_t)dx * 4),
					    (const void *)(src + s->offset + (uint64_t)(sy + y) * s->bytesPerRow + (uint64_t)sx * 4),
					    (size_t)w * 4);
				}
			}
#if defined(FUXI)
			if (copied != 2 && !refused) {
				cpuFrame = true;
                uint64_t duration = mach_absolute_time() - copyStart;
				fCpuTime += duration;
                cpuTicks += duration;
                cpuBytes += (uint64_t)w * h * 4;
			}
#endif
		}
		s->release();
	}
#if defined(FUXI)
 fCpuFrames += cpuFrame;
 if (!gpuFrame) {
  fStage[3] = fStage[4] = mach_absolute_time();
  if (read.held) { sweet_surface_present_end(); read.held = false; }
  completePresent(mach_absolute_time() - t0 - teWait);   /* copy time without the tegate wait */
  if (cpuFrame) scanoutLog(false, cpuBytes, cpuTicks, 0);
 }
#endif
 return true;
}

void AppleCLCD::completePresent(uint64_t took)
{
#if defined(FUXI)
 IOLockLock(fLock);
 SwapRecord &record = fPresentRecord ? *fPresentRecord : fLast;
 consumeRecord(record);
 if (fPublicationPending) {
  SwapStatus &status = fSwapStatus[fPublicationID % SWEET_FB_SWAP_HISTORY];
  if (status.id == fPublicationID && status.state != SwapShown) status.state = SwapDropped;
  fPublicationPending = false; fStatDropped++;
  IOLockWakeup(fLock, fSwapStatus, false);
 }
 IOLockUnlock(fLock);
 int published = fuxi_gpu_scanout_publish();
 if (!published) fuxi_display_kick();
 if (published == 2) {
  IOLockLock(fLock);
  fPublicationPending = true; fPublicationTime = took; fPublicationID = get32(record.bytes, 0x50);
  memcpy(fPublicationStage, fStage, sizeof(fStage));
  IOLockUnlock(fLock);
 }
 else if (published >= 0) presentStats(took);
 else if (fScanout) fScanoutHeld = true;
#else
 (void)took;
#endif
}

void AppleCLCD::stageStats(uint64_t now)
{
#if defined(FUXI)
 struct fg_present_times times; fuxi_gpu_scanout_stamps(&times);
 bool owned = fScanout && fuxi_gpu_scanout_owned();
 fStage[5] = owned ? times.start : now;
 if (owned) fLastStart = times.previous_start;
 for (unsigned i = 0; i < 6; i++) {
  uint64_t a = i < 5 ? fStage[i] : fLastStart, b = i < 5 ? fStage[i+1] : fStage[5];
  if (!a || !b || b < a) continue;
  uint64_t d = b-a;
  fStageSum[i] += d; if (d > fStageMax[i]) fStageMax[i] = d; fStageN[i]++;
 }
 fLastStart = fStage[5];
 if (fStatFrames != 599) return;
 uint64_t avg[6], max[6];
 for (unsigned i = 0; i < 6; i++) {
  absolutetime_to_nanoseconds(fStageN[i] ? fStageSum[i]/fStageN[i] : 0, &avg[i]); avg[i] /= 1000;
  absolutetime_to_nanoseconds(fStageMax[i], &max[i]); max[i] /= 1000;
 }
 IOLog("SWEET FB: stages 600 shown swap_ready_us=%llu/%llu ready_present_us=%llu/%llu present_submit_us=%llu/%llu "
     "submit_fence_us=%llu/%llu fence_start_us=%llu/%llu start_start_us=%llu/%llu samples=%u/%u/%u/%u/%u/%u callbacks_vsync=%u poll=%u wake=%u callbacks_per_shown_x1000=%llu/%llu/%llu\n",
     avg[0], max[0], avg[1], max[1], avg[2], max[2], avg[3], max[3], avg[4], max[4], avg[5], max[5],
     fStageN[0], fStageN[1], fStageN[2], fStageN[3], fStageN[4], fStageN[5], fStageCallbacks[0], fStageCallbacks[1], fStageCallbacks[2],
     (uint64_t)fStageCallbacks[0]*1000/600, (uint64_t)fStageCallbacks[1]*1000/600, (uint64_t)fStageCallbacks[2]*1000/600);
 bzero(fStageSum, sizeof(fStageSum)); bzero(fStageMax, sizeof(fStageMax)); bzero(fStageN, sizeof(fStageN)); bzero(fStageCallbacks, sizeof(fStageCallbacks));
 uint64_t wakeAvg[2], wakeMax[2];
 for (unsigned i = 0; i < 2; i++) {
  absolutetime_to_nanoseconds(fWakeN[i] ? fWakeSum[i]/fWakeN[i] : 0, &wakeAvg[i]); wakeAvg[i] /= 1000;
  absolutetime_to_nanoseconds(fWakeMax[i], &wakeMax[i]); wakeMax[i] /= 1000;
 }
 IOLog("SWEET FB: pacing wake_dispatch_us=%llu/%llu wake_gate_us=%llu/%llu wake_samples=%u/%u blocked_publication=%u acquire=%u ring=%u\n",
     wakeAvg[0], wakeMax[0], wakeAvg[1], wakeMax[1], fWakeN[0], fWakeN[1], fStageBlocks[0], fStageBlocks[1], fStageBlocks[2]);
 bzero(fWakeSum, sizeof(fWakeSum)); bzero(fWakeMax, sizeof(fWakeMax)); bzero(fWakeN, sizeof(fWakeN)); bzero(fStageBlocks, sizeof(fStageBlocks));
#else
 (void)now;
#endif
}

void AppleCLCD::presentStats(uint64_t took)
{
#if defined(FUXI)
 uint64_t activeStage[6]; bool deferred = fPublicationPending;
 memcpy(activeStage, fStage, sizeof(fStage));
 IOLockLock(fLock);
 if (deferred) {
  SwapStatus &status = fSwapStatus[fPublicationID % SWEET_FB_SWAP_HISTORY];
  if (status.id == fPublicationID) status.state = SwapShown;
  if (fLastValid && get32(fLast.bytes, 0x50) == fPublicationID) fLast.shown = true;
  fPublicationPending = false; IOLockWakeup(fLock, fSwapStatus, false);
  memcpy(fStage, fPublicationStage, sizeof(fStage));
 } else recordDone(fPresentRecord ? *fPresentRecord : fLast, true);
 IOLockUnlock(fLock);
 uint64_t t1 = mach_absolute_time();
 stageStats(t1);
 if (deferred) memcpy(fStage, activeStage, sizeof(fStage));
	fStatCopy += took;
	fStatCopyMax = took > fStatCopyMax ? took : fStatCopyMax;
	if (++fStatFrames >= (fStatEvery ? fStatEvery : 600)) {
		uint64_t avg_ns, max_ns, span_ns;
		absolutetime_to_nanoseconds(fStatCopy / fStatFrames, &avg_ns);
		absolutetime_to_nanoseconds(fStatCopyMax, &max_ns);
		absolutetime_to_nanoseconds(t1 - fStatStart, &span_ns);
		IOLockLock(fLock);
		uint32_t swaps = fStatSwaps, dropped = fStatDropped;
  uint32_t pending = fQueuedDefers, overflow = fOverflowDrops, skips = fReadySkips;
  uint64_t reuse = fReuseBeforeShow;
  uint64_t waits[2] = { fWaitPoll[0], fWaitPoll[1] }, early = fWaitEarly, latest = fWaitLatest;
  unsigned outstanding = fOutstandingMax;
		fStatSwaps = fStatDropped = 0;
		IOLockUnlock(fLock);
  /* Present-path counters (serialized with present()): format refusals and the tegate window. */
  uint32_t teCalls = fTeGate.hits + fTeGate.misses;
  uint64_t teAvg = 0, teMax = 0;
  absolutetime_to_nanoseconds(teCalls ? fTeGate.wait_sum / teCalls : 0, &teAvg);
  absolutetime_to_nanoseconds(fTeGate.wait_max, &teMax);
		IOLog("SWEET FB: stats %u frames in %llu ms, copy avg %llu us max %llu us, "
		    "%u swaps, %u not shown queued_pending=%u fifo_overflow=%u ready_skips=%u ownership_defers=%u reuse_before_show=%llu wait_mode0=%llu wait_mode1=%llu wait_early=%llu wait_id0=%llu outstanding_max=%u early_retires=%u "
		    "fmt_refused=%u tegate_hits=%u tegate_misses=%u tegate_wait_avg_us=%llu tegate_wait_max_us=%llu\n", fStatFrames, span_ns / 1000000, avg_ns / 1000,
		    max_ns / 1000, swaps, dropped, pending, overflow, skips, fScanoutDefers, reuse, waits[0], waits[1], early, latest, outstanding, fEarlyRetires,
		    fFormatRefused, fTeGate.hits, fTeGate.misses, teAvg / 1000, teMax / 1000);
  /* IOLog cuts lines near 256 bytes: the tail of the line above again, short (zeros with the gate off). */
  IOLog("SWEET FB: tegate hits %u misses %u wait avg %llu us max %llu us, fmt_refused %u\n",
      fTeGate.hits, fTeGate.misses, teAvg / 1000, teMax / 1000, fFormatRefused);
  fFormatRefused = fTeGate.hits = fTeGate.misses = 0;
  fTeGate.wait_sum = fTeGate.wait_max = 0;
		uint64_t gpu_ns, cpu_ns;
		absolutetime_to_nanoseconds(fGpuTime, &gpu_ns);
		absolutetime_to_nanoseconds(fCpuTime, &cpu_ns);
		struct fg_times split;
		uint64_t cleaned, skipped, invalidated, mapped;
		unsigned imports;
		fuxi_gpu_import_stats(&cleaned, &skipped, &invalidated, &mapped, &imports);
		sweet_surface_gpu_stats(&split);
		uint64_t ticks[] = {split.pin, split.lookup, split.map, split.clean,
		    split.submit, split.wait, split.unmap, split.cpu};
		uint64_t us[8];
		unsigned attempts = split.attempts;
		for (unsigned i = 0; i < 8; i++) {
			uint64_t ns;
			absolutetime_to_nanoseconds(ticks[i], &ns);
			us[i] = attempts ? ns / 1000 / attempts : 0;
		}
		IOLog("FUXI GPU: presents GPU %u CPU %u, GPU retire avg %llu us CPU avg %llu us, "
		    "split pin %llu lookup %llu map+TLB %llu clean %llu submit %llu wait %llu unmap %llu us, "
		    "GPU CPU-time %llu us, cache hit %u miss %u, sleeps %u, 2M %u 64K %u, busy ticks %u failed %u, "
		    "imports %u mapped %llu cache cleaned %llu skipped %llu invalidated %llu bytes cumulative\n",
		    fGpuFrames, fCpuFrames, fGpuFrames ? gpu_ns / 1000 / fGpuFrames : 0ULL,
		    fCpuFrames ? cpu_ns / 1000 / fCpuFrames : 0ULL,
		    us[0], us[1], us[2], us[3], us[4], us[5], us[6], us[7],
		    split.hits, split.misses, split.sleeps, split.blocks, split.groups, fGpuBusyTicks, fGpuFailed,
		    imports, mapped, cleaned, skipped, invalidated);
		fuxi_gpu_abi_stats_log();
		fuxi_gpu_abi_sched_log();
		fGpuBusyTicks = fGpuFailed = 0;
		fGpuFrames = fCpuFrames = 0;
		fGpuTime = fCpuTime = 0;
		fStatFrames = 0;
		fStatCopy = fStatCopyMax = 0;
		fStatStart = t1;
	}
#else
 (void)took;
#endif
}

void
AppleCLCD::scanoutLog(bool gpu, uint64_t bytes, uint64_t copy, uint64_t prepare)
{
#if defined(FUXI)
 if (!fScanoutLog) return;
 uint64_t ns, prep;
 absolutetime_to_nanoseconds(copy, &ns);
 absolutetime_to_nanoseconds(prepare, &prep);
 ++fScanoutFrames;
 if (gpu) ++fScanoutGpu; else ++fScanoutCpu;
 fScanoutTime += ns;
 if (ns > fScanoutMax) fScanoutMax = ns;
 /* Per-frame console lines cost CPU on every swap; summary only by default. */
 if (fScanoutLog == 2)
  IOLog("FUXI SCANOUT: frame=%u path=%s bytes=%llu copy_us=%llu prepare_us=%llu\n",
      fScanoutFrames, gpu ? "GPU" : "CPU", bytes, ns/1000, prep/1000);
 if (fScanoutFrames % 120 == 0)
  IOLog("FUXI SCANOUT: summary frames=%u GPU=%u CPU=%u copy_avg_us=%llu copy_max_us=%llu deferred=%u\n",
      fScanoutFrames, fScanoutGpu, fScanoutCpu, fScanoutTime/1000/fScanoutFrames,
      fScanoutMax/1000, fScanoutDefers);
#else
 (void)gpu; (void)bytes; (void)copy; (void)prepare;
#endif
}

bool
IOMobileFramebufferUserClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	return true;
}

bool
IOMobileFramebufferUserClient::start(IOService *provider)
{
	fFB = OSDynamicCast(AppleCLCD, provider);
	if (fFB == NULL || !IOUserClient::start(provider)) {
		return false;
	}
	fFB->addClient(this);
	return true;
}

IOReturn
IOMobileFramebufferUserClient::clientClose(void)
{
	if (fFB != NULL) {
		fFB->removeClient(this);
		IOLockLock(fFB->fLock);
		mach_port_t old = fPort;
		fPort = MACH_PORT_NULL;
		IOLockUnlock(fFB->fLock);
		if (old != MACH_PORT_NULL) {
			releaseNotificationPort(old);
		}
	}
	terminate();
	return kIOReturnSuccess;
}

IOReturn
IOMobileFramebufferUserClient::registerNotificationPort(mach_port_t port, __unused UInt32 type,
    __unused io_user_reference_t refCon)
{
	mach_port_t old;
	IOLockLock(fFB->fLock);
	old = fPort;
	fPort = port;
	fFB->wakePresentLocked();
	IOLockUnlock(fFB->fLock);
	if (old != MACH_PORT_NULL) {
		releaseNotificationPort(old);
	}
	return kIOReturnSuccess;
}

/* Called with the framebuffer lock held. */
void
IOMobileFramebufferUserClient::notify(uint32_t type, const uint64_t *values, uint32_t count)
{
	OSAsyncReference64 ref;

	if (type >= SWEET_FB_NOTIFY_TYPES || fFunc[type] == 0 || fPort == MACH_PORT_NULL) {
		return;
	}
	bzero(ref, sizeof(ref));
	setAsyncReference64(ref, fPort, (mach_vm_address_t)fFunc[type], fRefcon[type], fTask);
	sendAsyncResult64WithOptions(ref, kIOReturnSuccess, (io_user_reference_t *)values, count,
	    kIOUserNotifyOptionCanDrop);
}

static uint64_t sweet_fb_logged;        /* selectors already logged (< 64) */

IOReturn
AppleCLCD::requestPowerChange(uint64_t state)
{
	void *argument = (void *)(uintptr_t)state;
#if defined(FUXI)
	/* Serialize modes 2/3 requests and interest delivery with each other.
	 * Never call arbitrary interest clients holding fLock. */
	if (fBacklightPowerNotifications)
		return fWorkLoop->runAction(&AppleCLCD::powerChangeAction, this, argument);
#endif
	return powerChangeAction(this, argument, NULL, NULL, NULL);
}

IOReturn
AppleCLCD::powerChangeAction(OSObject *owner, void *state, void *, void *, void *)
{
	AppleCLCD *fb = (AppleCLCD *)owner;
	bool off = state == NULL;
	IOLockLock(fb->fLock);
	if (off != fb->fPowerOff) {
		fb->fPowerOff = off;
		fb->fFadeStep = 0;
		fb->fDirty = true;     /* on: redraw the last accepted frame if the FIFO is empty */
	}
	sweet_fb_display_off = off ? 1 : 0;
	fb->wakePresentLocked();
	IOLockUnlock(fb->fLock);
	IOLog("SWEET FB: display %s\n", off ? "off" : "on");
#if defined(FUXI)
	if (fb->fBacklightPowerNotifications) fuxi_backlight_display_state(fb, !off);
#endif
	return kIOReturnSuccess;
}

IOReturn
IOMobileFramebufferUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target,
    __unused void *reference)
{
	IOReturn ret = kIOReturnSuccess;
	uint64_t out[2] = { 0, 0 };
	uint32_t outCount = 0;

	switch (selector) {
	case kSweetFBGetLayerDefaultSurface:
		out[0] = sweet_surface_port_in_task(fFB->fSurface, fTask);
		outCount = 1;
		break;
	case kSweetFBSwapBegin:
		IOLockLock(fFB->fLock);
		if (fFB->fSwapID == UINT32_MAX) ret = kIOReturnNoResources; else out[0] = ++fFB->fSwapID;
		IOLockUnlock(fFB->fLock);
		outCount = 1;
		break;
	case kSweetFBSwapEnd: {
		uint8_t rec[SWEET_FB_SWAP_REC_SIZE];
		if (args->structureInputDescriptor != NULL ||
		    args->structureInputSize < SWEET_FB_SWAP_REC_SIZE) {
			ret = kIOReturnBadArgument;
			break;
		}
		memcpy(rec, args->structureInput, sizeof(rec));
		ret = fFB->swap(rec);
		break;
	}
	case kSweetFBGetID:
		out[0] = 0;
		outCount = 1;
		break;
	case kSweetFBGetDisplaySize:
		out[0] = fFB->fLogicalW;
		out[1] = fFB->fLogicalH;
		outCount = 2;
		break;
	case kSweetFBIsMainDisplay:
		out[0] = 1;
		outCount = 1;
		break;
	case kSweetFBGetDotPitch:
		out[0] = SWEET_FB_DOT_PITCH;
		outCount = 1;
		break;
	case kSweetFBNotifications:
		if (args->scalarInputCount >= 3 && args->scalarInput[2] < SWEET_FB_NOTIFY_TYPES) {
			uint32_t type = (uint32_t)args->scalarInput[2];
			IOLockLock(fFB->fLock);
			fFunc[type] = args->scalarInput[0];
			fRefcon[type] = args->scalarInput[1];
			fFB->wakePresentLocked();
			IOLockUnlock(fFB->fLock);
			char name[32];
			proc_selfname(name, sizeof(name));
			IOLog("SWEET FB: %s notifications type %u %s\n", name, type,
			    args->scalarInput[0] ? "on" : "off");
		}
		break;
	case kSweetFBRequestPowerChange:
		/* A boot-held off (sweetfb_bootoff) returns success and changes nothing. */
		if (args->scalarInputCount >= 1 &&
		    (args->scalarInput[0] != 0 || !sweet_fb_bootoff_ignored(fFB))) {
			ret = fFB->requestPowerChange(args->scalarInput[0]);
		}
		break;
	case kSweetFBSwapWait:
  ret = args->scalarInputCount == 3 && args->scalarInput[0] <= UINT32_MAX && args->scalarInput[1] <= UINT32_MAX ?
      fFB->swapWait((uint32_t)args->scalarInput[0], (uint32_t)args->scalarInput[1], args->scalarInput[2]) : kIOReturnBadArgument;
  break;
	default:
		break;
	}

	for (uint32_t i = 0; i < args->scalarOutputCount; i++) {
		args->scalarOutput[i] = i < outCount ? out[i] : 0;
	}
	if (args->structureOutputDescriptor == NULL && args->structureOutput != NULL) {
		bzero(args->structureOutput, args->structureOutputSize);
	}
	if (selector >= 64 || !(sweet_fb_logged & (1ULL << selector))) {
		if (selector < 64) {
			sweet_fb_logged |= 1ULL << selector;
		}
		char name[32];
		proc_selfname(name, sizeof(name));
		IOLog("SWEET FB: %s sel %u in %u/%u out %u/%u -> 0x%x\n", name, selector,
		    args->scalarInputCount, args->structureInputSize,
		    args->scalarOutputCount, args->structureOutputSize, ret);
	}
	return ret;
}
#endif /* SWEET */
