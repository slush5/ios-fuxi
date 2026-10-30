/*
 * FUXI WLAN (lane-wlan milestones 4/5/6): WCN7850 power-up, PCIe0 link, BAR0
 * assignment and a read-only MHI/BHI register dump from XNU.
 * Gated by boot-arg fuxiwlan (default 0 = do nothing):
 *   fuxiwlan=1      run all 28 steps
 *   fuxiwlan=2..28  stop after that step (bisecting a hang or reset)
 *   fuxiwlanoldiova=1  use M6f's IOVA pool, retaining the M6g SMMU profile
 *   fuxiwlanclk=1  add missing PCIe fabric clock votes (default: observe)
 *   fuxiwlanicc=0  disable M6i BCM votes (default 1 = enabled)
 *   fuxiwlaniccbw=0..8  optional KIWI CNSS bandwidth row (default 0)
 *   fuxiwlaniboff=1  disable stale inbound iATU windows before BHI (opt-in)
 *   fuxiwlanprobe=unmapped|pattern|civac|nc  one BHI experiment, then contain
 *   fuxiwlanaspm=1  retain inherited ASPM (revert download safeguard)
 *   fuxiwlanpre=qfprom|warm|reset|wake|l1ss  pre-BHI experiment
 *   fuxiwlanfw=stock  separate caller-supplied stock image, no fallback
 *   fuxiwlanbhi=full  M6l whole-file BHI size experiment
 *   fuxiwlanvec=early|rddm  M6l TX staging or RDDM RX experiment
 *   fuxiwlancnss=pdc  M6n AOP WLAN PDC setup before rails (default off)
 *   fuxiwlanseq=cycle,parf,dbi,eq,hostirq,msi,por,prefix  M6o opt-ins
 *   fuxiwlanseq=cyclerails  also cycle shared rails (froze the SoC once)
 *   fuxiwlanrails=1  re-vote six rails ON before WLAN_EN (also on cycle resume)
 *   fuxiwlanrails=2  re-vote ON only immediately before WLAN_MODE; no rail OFF
 *   fuxiwlanstop22=1  retain the BHI-only diagnostic stop (default: continue)
 *   fuxiwlankeep=1  retain DMA/device only after complete selected milestone success
 *   fuxiwlanmlo=1  ath12k PHY_CAP/MLO negotiation; =2 omit MLO (W12+; default unchanged)
 *                 P17 bits ORed on top (all default off, see wlan-p17.md): 0x4 HOST_CAP TLV 0x33,
 *                 0x8 always send MLO TLVs, 0x10 log PHY_CAP TLV 0x17 (needs =1), 0x20 DMA IOVA
 *                 align to 2^n of size, 0x40 read-only PMK8550 gpio3 sleep clock log, 0x80 also
 *                 enable it, 0x100 decode BHI error words and scan RDDM text
 *   fuxiwlanqrtr=1  W11 capability-only diagnostic; =2 W12 setup; =3 W13 small-chunk negotiation; =4 W14 downloads; =5 W15 config/mode
 *   fuxiwlanqrtr=6  W20-W22: W15, then copy engines, HTC and WMI init to WMI READY (needs
 *                 fuxiwlan=28 or 1; not with keep/stop22); any other value outside 0..7 is refused
 *   fuxiwlanqrtr=7  W23/W24: all of 6, then the 38-channel passive list, a station vdev and one
 *                 passive scan; logs SSIDs and OUI-only BSSIDs; nothing is transmitted (same needs);
 *                 C2b: a line per CHAN_INFO and a pdev stats request per PHY pdev after the scan
 *   fuxiwlanw15=1  W15 only (qrtr 5..7): omit WLAN_CFG shadow-v3 TLV (default 0); W20 then
 *                 publishes CE pointers to R2 through the window
 *   fuxiwlanw16=N  qrtr=5..7 only: 1 larger IPCR, 2 doorbell redrive, 3 both,
 *                 4 diagnostics only; all include MODE context diagnostics
 *   fuxiwlanw20=N  qrtr=6/7: 0x01 no HTT connect, 0x04 endpoint MRRS 128, 0x40 no
 *                 PDEV_SET_HW_MODE; qrtr=7 also 0x02 HTT version request, 0x10 2.4 GHz only,
 *                 0x80 EXT MAC/PHY pdev id; C2c (W26) 0x100 host RX rings (needs 0x02),
 *                 0x200 WBM idle link-descriptor list, 0x400 pdev chain masks; 0x08 (active
 *                 scan), 0x20 (whole BSSIDs; the owner masks every BSSID) and every other bit
 *                 refused, as is 0x01 with 0x02 (docs-lanes/wlan-w20.md)
 * Salami (FUXI_SALAMI, OnePlus 11, K-W1): the driver runs only when
 * fuxiwlansalami=1 is present as well; its absence or any other value refuses
 * before the thread starts, as do fuxiwlanseq with cyclerails and any invalid
 * selector. Step 1 expects the salami cmd-db (ldob15 0x40b00), step 2 votes
 * S4E 950 mV (salami live DT), and rh_send_request checks every RPMh command
 * before the first TCS write: only a salami WLAN rail at its live-DT mV,
 * enable 1 or mode 7 (HPM), or a BCM address; never an OFF, another mode or
 * a headroom vote, never 0x40a00 (salami ldob12, panel vddio).
 * fuxiwlanrails=1/2 re-votes the six WCN rails ON (mV, enable 1, mode 7) from
 * the salami cmd-db table (K-W1b: sheng, same SoC and chip, needed the HPM
 * votes or two rails stayed off and the link never trained). Every fuxiwlan*
 * knob is read from the boot-args first, else from the ADT /defaults property
 * of the same name (wl_arg; tools/mkadt-salami.py --wlan-defaults). The
 * firmware names are boot-args (basenames under /usr/local/share/fuxi/wlan),
 * defaults = the files salami Android loads:
 *   fuxiwlanbdf=bdwlan.b0e  fuxiwlanregdb=regdb.bin  fuxiwlanm3=phy_ucode20.elf
 *   fuxiwlanamss=NAME  replaces the amss20.bin/amss.bin choice; not with fuxiwlanfw=stock
 * Steps 17..28 probe DMA safety and load rootfs firmware through BHI/BHIe.
 * Completion is polled; M6o can enable MSI to a RAM sink. W11 channels/QMI are opt-in;
 * see docs-lanes/wlan-bt.md M6.
 *
 * Steps 1..13 are the loader's milestone-3 sequence, compiled from an exact
 * copy of shim/fuxi-probe/wlan.c (bsd/dev/wlan.c, FUXI_WLAN_XNU). Every write
 * there is documented against Linux and the live DT in that file and in
 * docs-lanes/wlan-bt.md section 7; it passed on the phone (Gen2 x2 link,
 * endpoint 17cb:1107, BAR0 64-bit non-prefetchable 2 MiB).
 * This file supplies the loader helpers (usb_rd/ts_wr/ts_gpio/rh_send and
 * the fault-contained accessors) over kernel mappings, then adds:
 *  14  outbound iATU region 1, type MEM, for the live-DT ranges entry
 *      <0x02000000 0 0x60300000 0 0x60300000 0 0x3d00000>
 *      (dw_pcie_prog_outbound_atu; unrolled region n at ATU base + n*0x200);
 *      root-port memory window 0x60400000-0x605fffff, empty prefetchable
 *      window, command IO|MEM|MASTER|SERR (dw_pcie_setup_rc / pci bridge
 *      window setup).
 *  15  endpoint BAR0 = 0x60400000 (2 MiB aligned, inside the window),
 *      BAR1 (upper half) = 0, command |= MEMORY only (no bus master).
 *  16  read MHI registers at BAR0 (drivers/bus/mhi/common.h: MHIREGLEN 0x00,
 *      MHIVER 0x08, MHICFG 0x10, CHDBOFF 0x18, ERDBOFF 0x20, BHIOFF 0x28,
 *      BHIEOFF 0x2c, DEBUGOFF 0x30, MHICTRL 0x38, MHISTATUS 0x48) and the
 *      BHI block at BHIOFF (BHIVERSION 0x00/0x04, EXECENV 0x28, STATUS 0x2c,
 *      ERRCODE 0x30, ERRDBG1..3 0x34..0x3c). BHI_SERIALNU is never read.
 *      ath12k (mhi.c) points mhi_ctrl->regs at BAR0, so the MHI block is at
 *      offset 0.
 * All device accesses go through fuxi_gpu_access (synchronous aborts are
 * recovered); after a fault normal writes are refused. M6 cleanup permits
 * containment and undo of only our proven-owned SMMU/RC slots.
 */
#if defined(FUXI)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/fcntl.h>
#include <sys/vnode.h>
#include <sys/uio.h>
#include <sys/random.h>
#include <string.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <arm/caches_internal.h>
#include "sweet_spmi_bus.h"
#include "fuxi_wlan_ce.h" /* W20: pure CE/HTC/WMI code (Linux ath12k v6.19, Clear BSD) */
/* osfmk/vm/pmap.h declares these under MACH_KERNEL_PRIVATE only; bsd/ cannot
 * see them, but they are in the same kernel image (M6k fuxiwlanprobe=nc). */
/* pexpert exposes the string-only parser under XNU_KERNEL_PRIVATE. */
extern boolean_t PE_parse_boot_arg_str(const char *, char *, int);
extern unsigned int pmap_cache_attributes(ppnum_t pn);
extern void pmap_set_cache_attributes(ppnum_t pn, unsigned int cacheattr);

extern ppnum_t pmap_find_phys(pmap_t, addr64_t);

extern kern_return_t thread_terminate(thread_t);
extern kern_return_t pmap_enter_options_addr(pmap_t, vm_map_offset_t,
    uint64_t, vm_prot_t, vm_prot_t, unsigned, boolean_t, unsigned, void *);
/* Fault-contained aligned access, osfmk/arm64/machine_routines_asm.s. */
extern unsigned fuxi_gpu_access(uintptr_t, uint64_t *, unsigned, unsigned);
void fuxi_wlan_init(void);
#if defined(FUXI_SALAMI)
/* ---- K-W1b salami knob source ----
 * The salami cmdline has about 33 characters free, so every fuxiwlan* knob is
 * read from the boot-args first and, only when absent there, from the ADT
 * /defaults property of the same name (PE_get_default, the fuxi_touch.c
 * ft_knob pattern; tools/mkadt-salami.py --wlan-defaults writes them):
 * numbers 4-byte LE, strings NUL-terminated. PE_get_default copies at most
 * SIZE bytes and treats a larger property as absent; every caller keeps its
 * own truncation sentinel. Absent from both = absent, exactly as before. The
 * two macros route every parser call below (wlan.c has none) through here;
 * the parenthesized names call the real parsers. */
static bool
wl_arg_adt(const char *name, void *buf, int size)
{
	if (size <= 0 || !PE_get_default(name, buf, (unsigned int)size)) { return false; }
	printf("FUXI WLAN: salami %s from ADT /defaults\n", name);
	return true;
}
static bool
wl_arg(const char *name, void *buf, int size)
{
	if ((PE_parse_boot_argn)(name, buf, size)) { return true; }
	return wl_arg_adt(name, buf, size);
}
static bool
wl_arg_str(const char *name, char *buf, int size)
{
	if ((PE_parse_boot_arg_str)(name, buf, size)) { return true; }
	return wl_arg_adt(name, buf, size);
}
#define PE_parse_boot_argn(name, buf, size) wl_arg((name), (buf), (size))
#define PE_parse_boot_arg_str(name, buf, size) wl_arg_str((name), (buf), (size))
/* ---- end K-W1b salami knob source ---- */
#endif

#define rsr(r) ({ uint64_t rsr_v_; __asm__ volatile("mrs %0, " r : "=r"(rsr_v_)); rsr_v_; })
#define delay fw_delay
static void fw_delay(unsigned us) { delay_for_interval(us, NSEC_PER_USEC); }

/* ---- kernel mappings (16 KiB aligned; same scheme as fuxi_adreno.c:
 * kernel_map VA only, fallible page-table expansion, never the 8 MiB
 * io_map submap). Kept for the boot; M5 partial/failure paths assert PERST. */
struct fw_window { uint64_t pa, size; uintptr_t va; };
static struct fw_window fw_windows[] = {
	{0x00100000, 0x80000, 0},   /* GCC: votes, GDSCs, PCIe0 RCGs/CBCRs/BCRs */
	{0x00898000, 0x4000, 0},    /* QUP2 SE6 (BT UART), gated reads only */
	{0x008c0000, 0x4000, 0},    /* QUP2 wrapper */
	{0x01c00000, 0x8000, 0},    /* PCIe0 PARF 0x1c00000 + QMP PHY 0x1c06000 */
	{0x01fd4000, 0x4000, 0},    /* TCSR clkref 0x1fd5100 */
	{0x15000000, 0x100000, 0},  /* apps SMMU: GR0/GR1 + runtime-bounded CBs */
	{0x60480000, 0x80000, 0},   /* EP sliding 512-KiB window: version + bounded BL logs */
	{0x0f14c000, 0x18000, 0},   /* TLMM tiles gpio76..96 */
	{0x17a20000, 0x4000, 0},    /* apps RSC DRV2 + TCS */
	{0x60000000, 0x4000, 0},    /* PCIe0 DBI, ELBI 0xf20, iATU 0x1000 */
	{0x60100000, 0x4000, 0},    /* config window (iATU region 0, CFG0) */
	{0x60400000, 0x4000, 0},    /* endpoint BAR0, first 16 KiB (MHI/BHI) */
	{0x81c60000, 0x20000, 0},   /* cmd-db (reserved DDR) */
	{0x0c300000, 0x4000, 0},    /* opt-in M6n AOSS QMP; descriptor bounds 0x400 */
	{0x00408000, 0x4000, 0},    /* opt-in M6n IPCC MPROC SEND_ID */
};
#define FW_NWIN (sizeof(fw_windows) / sizeof(fw_windows[0]))
static uint64_t fw_mapped_bytes;

static kern_return_t
fw_map_window(struct fw_window *w)
{
	vm_offset_t va = 0;
	if (!w->size || (w->pa & PAGE_MASK) || (w->size & PAGE_MASK) || w->va) {
		return KERN_INVALID_ARGUMENT;
	}
	kern_return_t kr = kmem_alloc(kernel_map, &va, w->size,
	    KMA_VAONLY | KMA_KOBJECT | KMA_NOPAGEWAIT, VM_KERN_MEMORY_IOKIT);
	if (kr != KERN_SUCCESS) {
		return kr;
	}
	for (vm_size_t off = 0; off < w->size; off += PAGE_SIZE) {
		kr = pmap_enter_options_addr(kernel_pmap, va + off, w->pa + off,
		    VM_PROT_READ | VM_PROT_WRITE, VM_PROT_NONE, VM_WIMG_IO, TRUE,
		    PMAP_OPTIONS_NOWAIT, NULL);
		if (kr != KERN_SUCCESS) {
			pmap_remove(kernel_pmap, va, va + w->size);
			kmem_free(kernel_map, va, w->size);
			return kr;
		}
	}
	w->va = va;
	fw_mapped_bytes += w->size;
	return KERN_SUCCESS;
}

/* ---- fault-contained 32-bit access by physical address ---- */
static unsigned fw_faults, fw_unmapped, fw_refused, fw_logged;
static uint32_t fw_fault_esr;
static uint64_t fw_fault_far;

static uintptr_t
fw_va(uint64_t pa)
{
	for (unsigned i = 0; i < FW_NWIN; i++) {
		struct fw_window *w = &fw_windows[i];
		if (w->va && pa >= w->pa && pa - w->pa <= w->size - 4) {
			return w->va + (uintptr_t)(pa - w->pa);
		}
	}
	return 0;
}

/* M6o link-down guard, defined with the cycle below. */
static bool wl_link_refused(uint64_t pa);

static unsigned
fw_access(uint64_t pa, uint64_t *v, unsigned wr)
{
	if (wl_link_refused(pa)) {
		fw_faults++;
		return 1;
	}
	uintptr_t va = fw_va(pa);
	if (!va || (pa & 3)) {
		if (fw_unmapped++ < 4) {
			printf("FUXI WLAN: no window for PA %#llx\n", pa);
		}
		fw_faults++;
		return 1;
	}
	if (wr && fw_faults) {
		if (fw_refused++ < 4) {
			printf("FUXI WLAN: write PA %#llx refused after a fault\n", pa);
		}
		return 1;
	}
	unsigned bad = fuxi_gpu_access(va, v, wr, 0);
	if (bad) {
		fw_fault_esr = (uint32_t)rsr("esr_el1");
		fw_fault_far = pa;
		fw_faults++;
		if (fw_logged++ < 8) {
			printf("FUXI WLAN: access fault PA %#llx %s ESR %#x\n", pa,
			    wr ? "write" : "read", fw_fault_esr);
		}
	}
	return bad;
}

/* Loader-helper equivalents for bsd/dev/wlan.c. */
#define probe_fault_esr fw_fault_esr
#define probe_fault_far fw_fault_far
#define TS_TLMM 0x0f100000UL
#define RH_DRV_PA 0x17a20000UL
#define RH_TCS_PA 0x17a20d00UL

static uint32_t
usb_rd(uintptr_t base, uint32_t off)
{
	uint64_t v = 0;
	if (fw_access((uint64_t)base + off, &v, 0)) {
		return 0xffffffffu;
	}
	return (uint32_t)v;
}

static void
ts_wr(uintptr_t base, uint32_t off, uint32_t value)
{
	uint64_t v = value;
	__asm__ volatile("dmb oshst" ::: "memory");
	(void)fw_access((uint64_t)base + off, &v, 1);
}

static uint32_t
fw_safe_rd32(uint64_t pa, uint32_t *faulted)
{
	uint64_t v = 0;
	*faulted = fw_access(pa, &v, 0);
	return (uint32_t)v;
}

static unsigned
fw_safe_wr32(uint64_t pa, uint32_t value)
{
	uint64_t v = value;
	return fw_access(pa, &v, 1);
}
#define probe_safe_rd32 fw_safe_rd32
#define probe_safe_wr32 fw_safe_wr32

/* TLMM CTL: pull [1:0], func [5:2], drive [8:6] = mA/2-1, OE [9]; as probe.c. */
static void
ts_gpio(unsigned n, unsigned func, unsigned pull, unsigned ma, int out)
{
	uintptr_t c = TS_TLMM + 0x1000u * n;
	if (out >= 0) {
		ts_wr(c, 4, out ? 2u : 0u);
	}
	ts_wr(c, 0, (usb_rd(c, 0) & ~0x3ffu) | pull | (func << 2) | ((ma / 2 - 1) << 6) |
	    (out >= 0 ? 0x200u : 0u));
}

/* ACTIVE TCS transport shared by the existing WLAN regulator votes and M6i.
 * RSC v3: TCS stride 672, command stride 24 (rpmh-rsc.c). No other XNU
 * code drives the fuxi apps RSC. Never touch inherited busy TCS state; an
 * uncertain transaction is quarantined for the session. */
#define RH_STRIDE 0x2a0u
#define RH_WAIT   0x20u
#define RH_CTRL   0x24u
#define RH_IDLE   0x28u
#define RH_EN     0x2cu
#define RH_MSGID  0x34u
#define RH_ADDR   0x38u
#define RH_DATA   0x3cu
#define RH_CSTAT  0x40u
static uint32_t rh_rd(uint32_t off) { return usb_rd(RH_TCS_PA, off); }
static void
rh_wr(uint32_t off, uint32_t v)
{
	ts_wr(RH_TCS_PA, off, v);
	__asm__ volatile("dsb osh" ::: "memory");
}
struct wl_rh_cmd { uint32_t addr, data, wait; };
static bool rh_poisoned;
#if defined(FUXI_SALAMI)
/* K-W1 salami RPMh policy, defined after bsd/dev/wlan.c (wl_salami_rail). */
static bool wl_salami_vote_ok(const struct wl_rh_cmd *cmd, bool regulator);
#endif
static uint32_t
rh_send_request(unsigned t, const struct wl_rh_cmd *cmd, unsigned n, bool regulator)
{
	/* DRV2 has three ACTIVE TCSs; preserve all sleep/wake/other slots. */
	if (t >= 3 || !n || n > 16 || rh_poisoned || fw_faults) { return 1; }
#if defined(FUXI_SALAMI)
	/* Before the first TCS register access: a refused command is never sent. */
	for (unsigned i = 0; i < n; i++) {
		if (!wl_salami_vote_ok(&cmd[i], regulator)) { return 1; }
	}
#endif
	uint32_t b = t * RH_STRIDE, bit = 1u << t;
	uint32_t cfg = usb_rd(RH_DRV_PA, 0xc);
	if (((usb_rd(RH_DRV_PA, 0) >> 16) & 255) != 3 ||
	    ((cfg >> 12) & 63) < 3 || ((cfg >> 27) & 31) < n ||
	    rh_rd(b + RH_IDLE) == 0 || rh_rd(b + RH_CTRL) || rh_rd(b + RH_EN) ||
	    rh_rd(b + RH_WAIT) || (rh_rd(4) & bit) || fw_faults) { return 1; }
	uint32_t irq_en = rh_rd(0), wait = 0;
	if (fw_faults) { return 1; }
	rh_poisoned = true;
	rh_wr(0, irq_en & ~bit);
	for (unsigned i = 0; i < n; i++) {
		uint32_t c = b + i * 24;
		rh_wr(c + RH_MSGID, 8u | (1u << 16) | (!regulator || cmd[i].wait ? 1u << 8 : 0));
		rh_wr(c + RH_ADDR, cmd[i].addr);
		rh_wr(c + RH_DATA, cmd[i].data);
		if (cmd[i].wait) { wait |= 1u << i; }
	}
	rh_wr(b + RH_WAIT, wait);
	rh_wr(b + RH_EN, (1u << n) - 1);
	bool programmed = !(rh_rd(0) & bit) && rh_rd(b + RH_WAIT) == wait && rh_rd(b + RH_EN) == (1u << n) - 1;
	for (unsigned i = 0; i < n; i++) {
		uint32_t c = b + i * 24;
		if (rh_rd(c + RH_ADDR) != cmd[i].addr || rh_rd(c + RH_DATA) != cmd[i].data ||
		    rh_rd(c + RH_MSGID) != (8u | (1u << 16) | (!regulator || cmd[i].wait ? 1u << 8 : 0))) { programmed = false; }
	}
	if (!programmed || fw_faults) { return 2; }
	rh_wr(b + RH_CTRL, 0);
	rh_wr(b + RH_CTRL, 1u << 16);
	if (rh_rd(b + RH_CTRL) != (1u << 16) || fw_faults) { return 2; }
	rh_wr(b + RH_CTRL, (1u << 16) | (1u << 24));
	bool done = false;
	for (unsigned us = 0; us < 100000 && !fw_faults; us++) {
		/* Device (M6j): BCM commands reach ISSUED|COMPLETED with the TCS idle
		 * while IRQ_STATUS stays 0 (its IRQ is masked here), and after the
		 * first BCM request regulator requests stop raising it too. Every
		 * request completes on idle plus per-command status. */
		done = rh_rd(b + RH_IDLE) != 0;
		for (unsigned i = 0; i < n; i++) {
			uint32_t mask = 0x100u | (cmd[i].wait ? 0x10000u : 0);
			if ((rh_rd(b + i * 24 + RH_CSTAT) & mask) != mask) { done = false; }
		}
		if (done) { break; }
		fw_delay(1);
	}
	if (!done || fw_faults) {
		if (!regulator) {
			uint32_t irq = rh_rd(4), ctrl = rh_rd(b + RH_CTRL), idle = rh_rd(b + RH_IDLE);
			for (unsigned i = 0; i < n; i++) {
				uint32_t c = b + i * 24;
				uint32_t msgid = rh_rd(c + RH_MSGID), status = rh_rd(c + RH_CSTAT);
				printf("FUXI WLAN: M6j RPMh timeout TCS %u cmd %u addr %#x data %#x MSGID %#x CMD_STATUS %#x IRQ_STATUS %#x CONTROL %#x IDLE %#x faults %u\n",
				    t, i, cmd[i].addr, cmd[i].data, msgid, status, irq, ctrl, idle, fw_faults);
			}
		}
		return 2;
	}
	rh_wr(b + RH_CTRL, 1u << 16);
	rh_wr(b + RH_CTRL, 0);
	rh_wr(b + RH_EN, 0);
	rh_wr(b + RH_WAIT, 0);
	rh_wr(8, bit);
	rh_wr(0, (rh_rd(0) & ~bit) | (irq_en & bit));
	if (fw_faults || rh_rd(b + RH_CTRL) || rh_rd(b + RH_EN) ||
	    rh_rd(b + RH_WAIT) || (rh_rd(4) & bit) || !rh_rd(b + RH_IDLE) || fw_faults) { return 3; }
	rh_poisoned = false;
	return 0;
}
static uint32_t
rh_send_batch(unsigned t, const struct wl_rh_cmd *cmd, unsigned n)
{
	/* BCM requests wait for the TCS IRQ and request responses on all cmds. */
	return rh_send_request(t, cmd, n, false);
}
static uint32_t
rh_send(unsigned t, uint32_t addr, uint32_t data)
{
	const struct wl_rh_cmd cmd = {addr, data, 1};
	return rh_send_request(t, &cmd, 1, true);
}

static bool wl_m6o_parf(void);
static int wl_m6o_host(unsigned);
#define FUXI_WLAN_XNU 1
#include "wlan.c"

#if defined(FUXI_SALAMI)
/* ---- K-W1 salami RPMh policy ----
 * Every command rh_send_request would put in a TCS is checked first. A
 * regulator request may only set one of the eight salami WLAN rails
 * (wlan.c wl_salami_rail, cmd-db by name) to its salami live-DT voltage
 * (+0), enable it (+4 = 1) or put it in HPM (+8 = 7, K-W1b: the
 * fuxiwlanrails ON-only re-vote; sheng needed it for the link to train).
 * Refused: any OFF (+4 = 0), any other mode value, any headroom (+0xc)
 * write, any other address, and 0x40a00 (salami ldob12, the panel vddio)
 * whatever the register. ldob15 also feeds the eUSB2 repeater of the USB
 * monitor, the WCD938x codec, the TFA amps and NFC, so it is only ever voted
 * ON: exactly 1800 mV, enable 1, mode 7. A batch (M6i ICC) may only address
 * BCMs (cmd-db slave 5, 0x5xxxx). This table is independent of the step 2/4
 * literals in wlan.c and of wl_rails: a mismatch fails the step closed
 * instead of sending another voltage.
 * mV, wl_salami_rail order: salami live DT chip_cfg@1 vdd-wlan-io, -dig,
 * -aon, -rfa2, vdd-wlan, -rfa1, then pcie0 vreg-0p9 and vreg-1p2. */
static const uint16_t wl_salami_mv[8] = {1800, 950, 980, 1350, 1000, 1900, 880, 1200};
static bool
wl_salami_vote_ok(const struct wl_rh_cmd *cmd, bool regulator)
{
	uint32_t base = cmd->addr & ~0xffu, reg = cmd->addr & 0xffu;
	if (!regulator) {
		if ((cmd->addr >> 16) == 5) { return true; }
	} else if (base != WL_SALAMI_LDOB12) {
		for (unsigned i = 0; i < 8; i++) {
			if (base != wl_salami_rail[i]) { continue; }
			if ((reg == 0 && cmd->data == (uint32_t)wl_salami_mv[i]) || (reg == 4 && cmd->data == 1) ||
			    (reg == 8 && cmd->data == 7)) {
				return true;
			}
			break;
		}
	}
	printf("FUXI WLAN: salami refused RPMh %s addr %#x data %#x (only a salami WLAN rail at its DT mV, enable 1 or mode 7, or a BCM)\n",
	    regulator ? "regulator" : "batch", cmd->addr, cmd->data);
	return false;
}
/* ---- end K-W1 salami RPMh policy ---- */
#endif

/* ---- milestone 6i: fixed SM8550 PCIe0 ICC clients, ACTIVE session votes ----
 * Graph, aggregation and packet format: Linux v6.18 sm8550.c, bcm-voter.c,
 * icc-rpmh.c, cmd-db.c and tcs.h; derivation / CNSS audit: docs/wlan-m6i.md.
 * No BCM address is MMIO and no address/unit/width/VCD is hardcoded here.
 * This is the WLAN client's vote only, not an aggregate of other XNU clients.
 */
#define WL_ICC_N 6u
struct wl_bcm {
	const char *name; bool mask; /* Linux enable_mask BCM: no unit/width use */
	uint32_t addr, unit, width, vcd, version, x, y, data;
};
static struct wl_bcm wl_bcms[WL_ICC_N] = {
	{.name = "SN7"}, {.name = "SH1", .mask = true}, {.name = "SH0"},
	{.name = "MC0"}, {.name = "ACV", .mask = true}, {.name = "CN0", .mask = true}
};
static unsigned wl_icc_acks;
static bool wl_icc_ready, wl_icc_failed, wl_icc_configured;
static int wl_icc_enabled = 1, wl_icc_bw;

static bool
wl_cdb_range(uint32_t at, uint32_t n)
{
	return at <= WL_CMDDB_BYTES && n <= WL_CMDDB_BYTES - at;
}
static uint32_t
wl_cdb32(uint32_t off)
{
	return wl_cdb16(off) | wl_cdb16(off + 2) << 16;
}
static int
wl_bcm_lookup(struct wl_bcm *bcm)
{
	/* Same existing reserved-DDR mapping and byte accessors as WLAN rails.
	 * Header v1, 8 slave headers, 24-byte entries; all offsets relative to
	 * byte 144. Validate complete extents before fetching entry/aux bytes. */
	if (fw_faults || wl_cdb32(0) != 1 || wl_cdb32(4) != 0x0c0330db ||
	    (wl_cdb32(140) & 1)) { return 1; }
	unsigned matches = 0;
	for (unsigned h = 8; h < 136; h += 16) {
		unsigned slave = wl_cdb16(h);
		if (!slave) { break; }
		uint32_t at = 144 + wl_cdb16(h + 2), aux = 144 + wl_cdb16(h + 4);
		unsigned n = wl_cdb16(h + 6);
		if (slave < 3 || slave > 5 || !wl_cdb_range(at, n * 24) || !wl_cdb_range(aux, 0)) { return 2; }
		for (unsigned i = 0; i < n; i++) {
			uint32_t e = at + i * 24;
			unsigned j = 0;
			for (; j < 8 && bcm->name[j]; j++) {
				if (wl_cdb8(e + j) != (uint8_t)bcm->name[j]) { break; }
			}
			if (j < 8 && bcm->name[j]) { continue; }
			/* Compare the entire zero-padded key, not just a prefix. */
			for (; j < 8 && !wl_cdb8(e + j); j++) { }
			if (j != 8) { continue; }
			uint32_t addr = wl_cdb32(e + 16), len = wl_cdb16(e + 20);
			uint32_t data = aux + wl_cdb16(e + 22);
			if (slave != 5 || (addr >> 16) != 5 || (addr & 3) || len < 8 ||
			    !wl_cdb_range(data, len)) { return 3; }
			bcm->addr = addr; bcm->unit = wl_cdb32(data);
			bcm->width = wl_cdb16(data + 4); bcm->vcd = wl_cdb8(data + 6);
			bcm->version = wl_cdb16(h + 8);
			if (!bcm->mask && (!bcm->unit || !bcm->width)) { return 4; } /* device: ACV width 0, VCD 12 */
			matches++;
		}
	}
	return fw_faults || matches != 1 ? 5 : 0;
}

/* Linux bcm_div floors except a nonzero value below the divisor becomes 1.
 * Do not substitute DIV_ROUND_UP: both normalization stages matter. */
static uint64_t
wl_bcm_div(uint64_t value, uint32_t divisor)
{
	return value && value < divisor ? 1 : value / divisor;
}
static uint32_t
wl_bcm_encode(uint64_t x, uint64_t y, bool commit)
{
	uint32_t valid = !!(x || y);
	if (x > 0x3fff) { x = 0x3fff; }
	if (y > 0x3fff) { y = 0x3fff; }
	return (commit ? 1u << 30 : 0) | valid << 29 | (uint32_t)x << 14 | (uint32_t)y;
}
static void
wl_bcm_bandwidth(struct wl_bcm *b, uint32_t avg, uint32_t peak, unsigned buswidth, unsigned channels)
{
	uint64_t x = wl_bcm_div((uint64_t)avg * b->width, buswidth * channels);
	uint64_t y = wl_bcm_div((uint64_t)peak * b->width, buswidth);
	x = wl_bcm_div(x * 1000, b->unit); y = wl_bcm_div(y * 1000, b->unit);
	b->x = (uint32_t)MIN(x, 0x3fff); b->y = (uint32_t)MIN(y, 0x3fff);
}
static void
wl_icc_configure(void)
{
	if (wl_icc_configured) { return; }
	wl_icc_configured = true;
	(void)PE_parse_boot_argn("fuxiwlanicc", &wl_icc_enabled, sizeof(wl_icc_enabled));
	(void)PE_parse_boot_argn("fuxiwlaniccbw", &wl_icc_bw, sizeof(wl_icc_bw));
	if ((wl_icc_enabled != 0 && wl_icc_enabled != 1) || wl_icc_bw < 0 || wl_icc_bw > 8) {
		wl_icc_failed = true;
	}
	printf("FUXI WLAN: M6i ICC fuxiwlanicc=%d fuxiwlaniccbw=%d default=1 retained ACTIVE session votes; CNSS download auto-vote=none\n",
	    wl_icc_enabled, wl_icc_bw);
}
static int
wl_icc_vote(uint32_t peak, bool download)
{
	/* KIWI (0x1107), two paths / nine rows, kB/s: path1 PCIe->GEM,
	 * path2 GEM->EBI. Optional experiment; CNSS2 does not automatically
	 * select a nonzero row during PBL download in the audited revision. */
	static const uint32_t cnss[][3] = {
		{0, 0, 0}, {2250, 1600000, 2188800}, {7500, 1600000, 2188800},
		{30000, 1600000, 2188800}, {100000, 1600000, 2188800},
		{175000, 6448000, 6220800}, {312500, 6448000, 8368000},
		{587500, 8528000, 12800000}, {7500, 3200000, 2188800}
	};
	wl_icc_configure();
	if (wl_icc_failed || fw_faults) { return 1; }
	if (!wl_icc_enabled) { return 0; }
	/* Resolve every dependency before the first vote. Missing or malformed
	 * metadata never leads to guessed addresses or partial lookup votes. */
	if (!wl_icc_ready) {
		for (unsigned i = 0; i < WL_ICC_N; i++) {
			struct wl_bcm *b = &wl_bcms[i];
			int e = wl_bcm_lookup(b);
			printf("FUXI WLAN: M6i ICC %s cmd-db addr %#x unit %u width %u VCD %u aux-version %#x lookup %d\n",
			    b->name, b->addr, b->unit, b->width, b->vcd, b->version, e);
			if (e) { wl_icc_failed = true; return 2; }
			for (unsigned j = 0; j < i; j++) {
				if (b->addr == wl_bcms[j].addr) { wl_icc_failed = true; return 2; }
			}
		}
		wl_icc_ready = true;
	}
	unsigned row = download ? (unsigned)wl_icc_bw : 0;
	uint32_t avg = cnss[row][0], p1 = cnss[row][1], p2 = cnss[row][2];
	if (p1 < peak) { p1 = peak; }
	if (p2 < peak) { p2 = peak; }
	printf("FUXI WLAN: M6i ICC %s pcie-mem 0/%u cpu-pcie 0/1 CNSS row %u path1 %u/%u path2 %u/%u kBps\n",
	    download ? "link/download" : "pretrain", peak, row, avg, cnss[row][1], avg, cnss[row][2]);
	wl_bcm_bandwidth(&wl_bcms[0], avg, p1, 16, 1); /* SN7 qns_pcie_mem_noc */
	wl_bcms[1].x = 0; wl_bcms[1].y = 1;          /* SH1 mask, both paths */
	wl_bcm_bandwidth(&wl_bcms[2], avg, p2, 16, 4); /* SH0 qns_llcc */
	wl_bcm_bandwidth(&wl_bcms[3], avg, p2, 4, 4);  /* MC0 ebi */
	wl_bcms[4].x = 0; wl_bcms[4].y = 8;          /* ACV mask */
	wl_bcms[5].x = 1; wl_bcms[5].y = 1;          /* CN0 mask + keepalive */
	unsigned order[WL_ICC_N];
	for (unsigned i = 0; i < WL_ICC_N; i++) {
		unsigned j = i;
		while (j && wl_bcms[order[j - 1]].vcd > wl_bcms[i].vcd) {
			order[j] = order[j - 1]; j--;
		}
		order[j] = i;
	}
	/* M6j serializes one whole VCD per request. Linux v6.18 can pack
	 * multiple VCDs; see docs/wlan-m6i.md for this deliberate restriction.
	 * All commands request responses; only the last gets commit/wait. */
	struct wl_rh_cmd cmd[WL_ICC_N];
	for (unsigned i = 0; i < WL_ICC_N; i++) {
		struct wl_bcm *b = &wl_bcms[order[i]];
		bool commit = i + 1 == WL_ICC_N || b->vcd != wl_bcms[order[i + 1]].vcd;
		b->data = wl_bcm_encode(b->x, b->y, commit);
		cmd[i] = (struct wl_rh_cmd){b->addr, b->data, commit};
	}
	unsigned groups = 0;
	for (unsigned first = 0; first < WL_ICC_N;) {
		unsigned end = first + 1;
		while (end < WL_ICC_N && !cmd[end - 1].wait) { end++; }
		uint32_t e = rh_send_batch(0, cmd + first, end - first);
		for (unsigned i = first; i < end; i++) {
			struct wl_bcm *b = &wl_bcms[order[i]];
			printf("FUXI WLAN: M6i ICC %s addr %#x vote %u/%u data %#x VCD %u commit %u ack %s error %#x\n",
			    b->name, b->addr, b->x, b->y, b->data, b->vcd, cmd[i].wait, e ? "unconfirmed" : "yes", e);
		}
		unsigned vcd = wl_bcms[order[first]].vcd;
		if (!e) { groups++; }
		printf("FUXI WLAN: M6j ICC VCD %u cmds %u ack %s error %#x acked_groups %u\n",
		    vcd, end - first, e ? "unconfirmed" : "yes", e, groups);
		if (e) { wl_icc_failed = true; return 3; }
		first = end;
	}
	wl_icc_acks++;
	return 0;
}
static int
wl_icc_step(unsigned step)
{
	if (step == 2) { return wl_icc_vote(250000, false); }
	if (step != 12) { return 0; }
	wl_icc_configure();
	if (wl_icc_failed) { return 1; }
	if (!wl_icc_enabled) { return 0; }
	/* Step 11's capability walk / link read has succeeded. Validate it
	 * again before accepting a negotiated rate; never lower to zero. */
	uint32_t link = usb_rd(WL_DBI, wl3_report[67] + 0x10);
	unsigned speed = (link >> 16) & 15, width = (link >> 20) & 63;
	static const uint32_t rate[] = {0, 250000, 500000, 984500};
	if (fw_faults || !(link & (1u << 29)) || !width || width > 2 || speed < 1 || speed > 3) {
		wl_icc_failed = true; return 4;
	}
	return wl_icc_vote(rate[speed] * width, true);
}
/* ---- end milestone 6i ---- */

/* ---- milestone 4 ---- */
#define WL4_MAGIC   0x574c0004u
#define WL4_STEPS   16u
#define WL_ATU1     0x60001200ull  /* unrolled outbound region 1 */
#define WL_MEM_LO   0x60300000u    /* live DT ranges, 32-bit non-prefetchable */
#define WL_MEM_HI   0x63ffffffu
#define WL_BAR0     0x60400000u
#define WL_BAR0_END 0x605fffffu
#define WL_EPCFG    0x60100000ull
static uint32_t wl4_report[64];

static const char *
wl_mhi_state(uint32_t s)
{
	static const char *n[8] = {"RESET", "READY", "M0", "M1", "M2", "M3", "M3_FAST", "BHI"};
	return s < 8 ? n[s] : (s == 0xff ? "SYS_ERR" : "?");
}

static const char *
wl_mhi_ee(uint32_t e)
{
	static const char *n[8] = {"PBL", "SBL", "AMSS", "RDDM", "WFW", "PTHRU", "EDL", "FP"};
	return e < 8 ? n[e] : "?";
}

/* 0 on success or a nonzero reason. */
static int
wl4_step(unsigned step)
{
	uint32_t *r = wl4_report, *m3 = wl3_report;
	switch (step) {
	case 14: { /* MEM outbound + root-port window */
		wl_pwr(WL_ATU1 + 0x08, WL_MEM_LO); wl_pwr(WL_ATU1 + 0x0c, 0);
		wl_pwr(WL_ATU1 + 0x10, WL_MEM_HI); wl_pwr(WL_ATU1 + 0x20, 0);
		wl_pwr(WL_ATU1 + 0x14, WL_MEM_LO); wl_pwr(WL_ATU1 + 0x18, 0);
		wl_pwr(WL_ATU1 + 0x00, 0);         /* type MEM */
		wl_pwr(WL_ATU1 + 0x04, 1u << 31);  /* enable */
		unsigned i;
		for (i = 0; i < 5; i++) {
			if (wl_prd(WL_ATU1 + 0x04) & (1u << 31)) {
				break;
			}
			delay(9000);
		}
		r[4] = wl_prd(WL_ATU1 + 0x04);
		r[5] = wl_prd(WL_ATU1 + 0x08);
		r[6] = wl_prd(WL_ATU1 + 0x10);
		r[7] = wl_prd(WL_ATU1 + 0x14);
		/* type-1 header: memory base/limit [15:4]/[31:20] = addr[31:20] */
		wl_pwr(WL_DBI + 0x20, ((WL_BAR0_END >> 16) & 0xfff0u) << 16 | ((WL_BAR0 >> 16) & 0xfff0u));
		wl_pwr(WL_DBI + 0x24, 0x0000fff0u); /* prefetchable window empty */
		wl_pwr(WL_DBI + 0x28, 0);
		wl_pwr(WL_DBI + 0x2c, 0);
		wl_pwr(WL_DBI + 0x04, (wl_prd(WL_DBI + 0x04) & 0xffffu) | 0x107u);
		r[8] = wl_prd(WL_DBI + 0x20);
		r[9] = wl_prd(WL_DBI + 0x24);
		r[10] = wl_prd(WL_DBI + 0x04);
		printf("FUXI WLAN: iATU1 MEM %#x-%#x -> %#x ctrl2 %#x; RC window %#x pref %#x cmd/status %#x\n",
		    r[5], r[6], r[7], r[4], r[8], r[9], r[10]);
		if (wl3_faults) {
			return 1;
		}
		if (!(r[4] & (1u << 31)) || r[5] != WL_MEM_LO || r[6] != WL_MEM_HI) {
			return 2;
		}
		return (r[10] & 6u) == 6u ? 0 : 3;
	}
	case 15: { /* BAR0 assignment */
		uint32_t rb = m3[87], size = ~(rb & ~0xfu) + 1u;
		r[11] = rb;
		r[12] = size;
		/* step 13 must have found a 64-bit non-prefetchable memory BAR0 of
		 * at most 2 MiB with an all-ones upper half (sizes below 4 GiB). */
		if ((rb & 0xfu) != 0x4u || !size || size > 0x200000u || (size & (size - 1)) ||
		    m3[88] != 0xffffffffu) {
			printf("FUXI WLAN: BAR0 readback %#x/%#x unexpected; not assigning\n", rb, m3[88]);
			return 1;
		}
		uint32_t cmd = wl_prd(WL_EPCFG + 0x04);
		r[13] = cmd;
		wl_pwr(WL_EPCFG + 0x04, cmd & 0xffffu & ~7u);   /* decode off while moving */
		wl_pwr(WL_EPCFG + 0x10, WL_BAR0);
		wl_pwr(WL_EPCFG + 0x14, 0);
		r[14] = wl_prd(WL_EPCFG + 0x10);
		r[15] = wl_prd(WL_EPCFG + 0x14);
		wl_pwr(WL_EPCFG + 0x04, (cmd & 0xffffu & ~7u) | 2u); /* MEMORY only */
		r[16] = wl_prd(WL_EPCFG + 0x04);
		printf("FUXI WLAN: BAR0 size %#x assigned %#llx (readback %#x %#x) cmd/status %#x -> %#x\n",
		    size, (uint64_t)r[15] << 32 | (r[14] & ~0xfu), r[14], r[15], cmd, r[16]);
		if (wl3_faults) {
			return 2;
		}
		if ((r[14] & ~0xfu) != WL_BAR0 || r[15] != 0) {
			return 3;
		}
		return (r[16] & 7u) == 2u ? 0 : 4;
	}
	case 16: { /* MHI + BHI, read-only */
		static const uint16_t mo[10] = {0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x2c, 0x30, 0x38, 0x48};
		static const uint16_t bo[8] = {0x00, 0x04, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c};
		for (unsigned i = 0; i < 10; i++) {
			r[20 + i] = wl_prd((uint64_t)WL_BAR0 + mo[i]);
		}
		if (wl3_faults) {
			return 1;
		}
		uint32_t cfg = r[22], st = r[29], ctrl = r[28], bhi = r[25];
		printf("FUXI WLAN: MHI REGLEN %#x MHIVER %#x MHICFG %#x (nhwer %u ner %u nhwch %u nch %u)\n",
		    r[20], r[21], cfg, cfg >> 24, (cfg >> 16) & 0xffu, (cfg >> 8) & 0xffu, cfg & 0xffu);
		printf("FUXI WLAN: MHI CHDBOFF %#x ERDBOFF %#x BHIOFF %#x BHIEOFF %#x DEBUGOFF %#x\n",
		    r[23], r[24], r[25], r[26], r[27]);
		printf("FUXI WLAN: MHICTRL %#x (state %u, reset %u) MHISTATUS %#x (state %s, ready %u, syserr %u)\n",
		    ctrl, (ctrl >> 8) & 0xffu, (ctrl >> 1) & 1u, st, wl_mhi_state((st >> 8) & 0xffu),
		    st & 1u, (st >> 2) & 1u);
		if (r[20] == 0xffffffffu && r[21] == 0xffffffffu) {
			return 2;               /* BAR not decoded / link down */
		}
		if (bhi < 0x40 || bhi > 0x4000u - 0x40u || (bhi & 3)) {
			printf("FUXI WLAN: BHIOFF %#x outside the mapped 16 KiB; BHI not read\n", bhi);
			return 3;
		}
		for (unsigned i = 0; i < 8; i++) {
			r[30 + i] = wl_prd((uint64_t)WL_BAR0 + bhi + bo[i]);
		}
		printf("FUXI WLAN: BHI version %u.%u EXECENV %#x (%s) STATUS %#x (%u) ERRCODE %#x ERRDBG %#x %#x %#x\n",
		    r[31], r[30], r[32], wl_mhi_ee(r[32]), r[33], r[33] >> 30, r[34], r[35], r[36], r[37]);
		/* Root-port status after BAR traffic (received master/target abort). */
		r[38] = wl_prd(WL_DBI + 0x04);
		r[39] = wl_prd(WL_EPCFG + 0x04);
		printf("FUXI WLAN: RC cmd/status %#x EP cmd/status %#x\n", r[38], r[39]);
		return wl3_faults ? 4 : 0;
	}
	}
	return 0xff;
}

/* ---- milestone 5: polling only, no channel start or QMI ----
 * Register/layout references (independent implementation): Linux MHI common.h,
 * host/boot.c + pm.c, arm-smmu.h; OpenBSD if_qwz_pci.c READY/FBC sequence.
 * M6 uses owned stage-1 translation of WLAN IOVAs to wired XNU pages.
 */
#define WL5_STEPS 28u
#define WL_SMMU 0x15000000ull
#define WL_CHUNK 0x80000u
#define WL_FW_MAX 0xf00000u
#define WL_SEQ_MASK 0x3fffffffu
static uint32_t wl5_report[128];
static uint32_t wl_bhi, wl_bhie, wl_fw_size, wl_seq;
static const char *wl_fw_path;
static bool wl_mhi_valid;
struct wl_dma { vm_offset_t va; vm_size_t size; uint64_t pa, iova; bool nc; };
static struct wl_dma wl_image, wl_vectors, wl_contexts, wl_tables;
static struct wl_dma wl_amss; /* full file when BHI uses a separate prefix */
static bool wl_qrtr, wl_w12, wl_w13, wl_w14, wl_w15, wl_w11_attempted, wl_w11_ok;
static unsigned wl15_variant, wl16_variant, wl12_mlo_variant;
static unsigned wl_rails_variant;
static bool wl_rails_requested;
static bool wl16_large, wl16_redrive;
/* W20 (fuxiwlanqrtr=6, C1): after this boot's W15 MODE reply, copy engines,
 * HTC and WMI init to WMI READY, then containment. fuxiwlanw20 bits (0 =
 * default): 0x01 NOHTT no HTT connect; 0x04 MRRS128 endpoint Device Control
 * max read request 128 bytes before CE init; 0x40 NOHWMODE no
 * PDEV_SET_HW_MODE. W23 (fuxiwlanqrtr=7, C2) runs all of that, then the
 * passive scan; it also takes 0x02 HTTVER (HTT version request after HW
 * mode), 0x10 BAND24 (2.4 GHz channels only) and 0x80 PDEVCAP (the EXT
 * MAC/PHY pdev id instead of L19's 0 for a single-pdev chip; overnight_plan.md
 * calls this rung PDEV0). 0x08 ACTIVE would transmit probe requests and stays
 * refused; 0x20 FULLBSSID (whole BSSIDs in the log) stays refused too: the
 * owner's decision masks the last three octets of every BSSID. 0x01 with 0x02
 * is refused (no HTT endpoint to ask). qrtr 8 (C3) is refused until it exists.
 * C2c (W26, qrtr 7 only, all default off): 0x100 RXRING the RX arena, the
 * refill/per-MAC/RXDMA-error rings with 4095 host buffers and their four HTT
 * SRING_SETUP messages (refused without 0x02: L19 asks the HTT version before
 * any ring announcement, core.c:955 then 1143); 0x200 IDLELINK the WBM idle
 * link-descriptor list (ring 120, programmed only after read-only pre-reads
 * show no owner); 0x400 CHAINMASK PDEV_SET_PARAM TX then RX chain mask before
 * the channel list. 0x200 and 0x400 need no other bit.
 * C2d (W27, qrtr 7 only, default off, both need 0x100): 0x800 RXSEL one HTT
 * RX_RING_SELECTION_CFG per MAC ring right after the SRING_SETUPs; 0x1000 REORX
 * the UMAC RX path (WBM2SW3 release and REO2SW0 exception rings and the REO
 * setup at W20, REO2SW1..8 after service ready) with every returned buffer
 * given back to the refill ring.
 * C2e (W28, qrtr 7 only, default off, needs 0x100): 0x2000 MONSTAT the two
 * RXDMA monitor status rings 185/193 with 1023 host status buffers each, filled
 * at W20 after the refill ring and announced by HTT SRING_SETUP after the four
 * C2c ones and before the ring selection; watched, never reaped (L19 STA mode
 * reaps none); a read-only MHI/BHI crash line at the COMPLETED timeout and in
 * containment before the MHI reset. */
#define WL20_NOHTT 0x01u
#define WL20_HTTVER 0x02u
#define WL20_MRRS128 0x04u
#define WL20_ACTIVE 0x08u
#define WL20_BAND24 0x10u
#define WL20_FULLBSSID 0x20u
#define WL20_NOHWMODE 0x40u
#define WL20_PDEVCAP 0x80u
#define WL20_RXRING 0x100u
#define WL20_IDLELINK 0x200u
#define WL20_CHAINMASK 0x400u
#define WL20_RXSEL 0x800u
#define WL20_REORX 0x1000u
#define WL20_MONSTAT 0x2000u
#define WL20_L19ORDER 0x4000u
#define WL26_BITS (WL20_RXRING | WL20_IDLELINK | WL20_CHAINMASK)
#define WL27_BITS (WL20_RXSEL | WL20_REORX)
#define WL20_BITS (WL20_NOHTT | WL20_MRRS128 | WL20_NOHWMODE)
#define WL23_BITS (WL20_BITS | WL20_HTTVER | WL20_BAND24 | WL20_PDEVCAP | WL26_BITS | WL27_BITS | WL20_MONSTAT | WL20_L19ORDER)
#define WL20_QRTR_MAX 7
static bool wl_w20, wl_w23;
static unsigned wl20_bits;
static unsigned wl20_qrtr; /* 6 or 7 when wl_w20, for the init lines */
/* ---- P17: opt-in WCN7850 MODE-silence probes (fuxiwlanmlo bits >= 4) ----
 * Low two bits keep the W12 meaning (1 PHY_CAP, 2 omit MLO). Bits:
 *   0x004 HOSTMEM   HOST_CAP vendor TLV 0x33 (u8 1, "host memory on demand")
 *   0x008 MLOALL    send the HOST_CAP MLO TLVs whatever PHY_CAP said
 *   0x010 PHY17     decode and log PHY_CAP reply TLV 0x17 (needs mlo=1)
 *   0x020 DMAALIGN  align every DMA IOVA to its size rounded up to 2^n
 *   0x040 CLKCHECK  read-only log of PMK8550 gpio3 (32 kHz sleep clock)
 *   0x080 CLKON     as CLKCHECK, then enable the pin if it is not an output
 *   0x100 ERRDEC    decode BHI error words; scan RDDM text on a crash
 * The pin sequence, the power-of-two alignment rule and the text scan are
 * adapted from UniXNU-sheng package/WifiProbe.cpp (08cde20, SPDX
 * BSD-2-Clause-Patent, Copyright (c) 2026 UniXNU contributors): pmicPrepare,
 * arenaAlloc and rddmScan/looksLikeError. The sheng source has no WLFW code,
 * so TLV 0x33 (u8 1) and PHY TLV 0x17 come from the lane brief, unverified
 * against firmware; the MLO TLVs reuse this file's own HOST_CAP codec. */
#define WL17_HOSTMEM 0x004u
#define WL17_MLOALL 0x008u
#define WL17_PHY17 0x010u
#define WL17_DMAALIGN 0x020u
#define WL17_CLKCHECK 0x040u
#define WL17_CLKON 0x080u
#define WL17_ERRDEC 0x100u
#define WL17_MASK 0x1fcu
static unsigned wl17_bits;
static unsigned wl17_phy17_len;
static uint64_t wl17_phy17_val;

/* Links to put in HOST_CAP MLO TLVs: 0 omits them, -1 refuses (num_phy > 2). */
static int
wl17_mlo_links(unsigned variant, unsigned bits, bool enabled, unsigned num)
{
	if (variant == 2) { return 0; }
	if (enabled && num && num != 255) { return num > 2 && !(bits & WL17_MLOALL) ? -1 : (num > 2 ? 2 : (int)num); }
	return (bits & WL17_MLOALL) ? (num == 1 ? 1 : 2) : 0;
}

/* PHY_CAP TLV 0x17: width unknown, keep length and up to 8 LE bytes. */
static void
wl17_phy17_parse(const uint8_t *p, unsigned n)
{
	wl17_phy17_len = n; wl17_phy17_val = 0;
	for (unsigned i = 0; i < n && i < 8; i++) { wl17_phy17_val |= (uint64_t)p[i] << (8*i); }
}

static uint64_t
wl17_align(uint64_t size)
{
	uint64_t align = 4096;
	while (align < size && align < (1ull << 40)) { align <<= 1; }
	return align;
}

/* PMK8550 gpio3 = SID 0 page 0xba. regs 04 05 40 44 46 as read over SPMI. */
#define WL17_CLK_PPID 0x0bau
struct wl17_gpio { uint8_t type, sub, mode, func, en; };
struct wl17_wr { uint8_t reg, val; };
static const struct wl17_wr wl17_clk_writes[5] = {
	{0x42, 5}, {0x41, 0}, {0x44, 2}, {0x40, 1}, {0x46, 0x80}
};
/* 0xff already an enabled output of function index 2; 0 write needed;
 * 3 the pin belongs to another EE; 4 not an LV/MV pin (other layout). */
static unsigned
wl17_clk_state(const struct wl17_gpio *g, unsigned owner)
{
	if ((g->en & 0x80) && (g->mode & 3) == 1 && (g->func & 0xf) == 2) { return 0xff; }
	if (owner) { return 3; }
	if (g->sub < 0x10 || g->sub > 0x13) { return 4; }
	return 0;
}

static void
wl17_errword(uint32_t w, char *out, unsigned n)
{
	char a[5];
	for (unsigned i = 0; i < 4; i++) {
		unsigned c = (w >> (8*i)) & 255;
		a[i] = c >= 0x20 && c < 0x7f ? (char)c : '.';
	}
	a[4] = 0;
	const char *k = w == 0 ? "zero" : w == ~0u ? "all-ones (read fault or no link)" :
	    (w >> 16) == 0xcafe ? "0xcafe-prefixed sentinel word" :
	    (w >> 16) == 0xdead ? "0xdead-prefixed sentinel word" : "plain value";
	snprintf(out, n, "%#010x hi16 %#x lo16 %#x bytes(LE) '%s' class: %s", w, w >> 16, w & 0xffff, a, k);
}

/* Text runs of a RAM dump that look like a firmware assert/fatal message. */
static bool wl17_print(uint8_t c) { return c >= 0x20 && c < 0x7f; }
static bool
wl17_error_text(const uint8_t *t, unsigned n)
{
	for (unsigned i = 0; i < n; i++) { if (t[i] == '%') { return false; } }
	for (unsigned i = 0; i + 3 < n; i++) {
		if (t[i] == '.' && t[i+1] == 'c' && t[i+2] == ':' && t[i+3] >= '0' && t[i+3] <= '9') { return true; }
		if (i + 5 <= n && (!memcmp(t+i, "ssert", 5) || !memcmp(t+i, "SSERT", 5) || !memcmp(t+i, "FATAL", 5))) { return true; }
		if (i + 8 <= n && !memcmp(t+i, "rr_fatal", 8)) { return true; }
	}
	return false;
}

typedef void (*wl17_emit)(uint64_t off, const char *text);
static unsigned
wl17_scan(const uint8_t *seg, uint64_t size, unsigned max, wl17_emit emit)
{
	unsigned found = 0; uint32_t seen[40] = {0};
	for (uint64_t i = 0; i < size && found < max && found < 40;) {
		if (!wl17_print(seg[i])) { i++; continue; }
		uint64_t j = i;
		while (j < size && wl17_print(seg[j]) && j - i < 200) { j++; }
		unsigned n = (unsigned)(j - i);
		if (n >= 10 && wl17_error_text(seg+i, n)) {
			uint32_t h = 2166136261u; bool dup = false;
			for (unsigned k = 0; k < n; k++) { h = (h ^ seg[i+k]) * 16777619u; }
			for (unsigned k = 0; k < found; k++) { if (seen[k] == h) { dup = true; } }
			if (!dup) {
				char body[85]; unsigned len = n < 84 ? n : 84;
				memcpy(body, seg+i, len); body[len] = 0;
				seen[found++] = h; emit(i, body);
			}
		}
		i = j;
	}
	return found;
}
/* ---- end P17 pure helpers ---- */

/* W12/W13 allocate after an accepted REQUEST_MEM; retain partial allocations until
 * containment is proven. Limit includes XNU page rounding, not just wire sizes. */
#define WL13_CHUNK_LIMIT (2u * 1024u * 1024u)
#define WL_TARGET_MAX 52u
#define WL_TARGET_CAP (64u * 1024u * 1024u)
static struct wl_dma wl_target[WL_TARGET_MAX];
static uint32_t wl_target_size[WL_TARGET_MAX], wl_target_type[WL_TARGET_MAX];
static unsigned wl_target_n;
static uint64_t wl_target_total;
static struct wl_dma wl_ipcr;
static struct wl_dma wl_ce_arena; /* W20: rdp, CE rings, TX slots, RX buffers (fuxi_wlan_ce.h layout) */
/* C2c W26: the RX arena (w20 0x100) and the link arena (w20 0x200), fuxi_wlan_ce.h layouts. */
static struct wl_dma wl_rx_arena, wl_link_arena, wl_reo_arena, wl_mon_arena, wl_cm_arena;
/* W14 inputs are retained through keep/containment, including partial reads. */
static struct wl_dma wl14_regdb, wl14_bdf, wl14_m3;
static char wl14_bdf_name[65]; /* one extra byte detects XNU string truncation */
#if defined(FUXI_SALAMI)
/* K-W1: salami firmware names are boot-args (same 65-byte rule as the BDF). */
static char wl14_regdb_name[65], wl14_m3_name[65];
static char wl_amss_path[96]; /* empty = the SoC-major choice at step 18 */
#define WL14_REGDB_NAME wl14_regdb_name
#define WL14_M3_NAME wl14_m3_name
#else
#define WL14_REGDB_NAME "regdb_xiaomi.bin"
#define WL14_M3_NAME "m3.bin"
#endif
#define WL14_FILE_MAX (1024u * 1024u)
#define WL14_M3_MAX (2u * 1024u * 1024u)
#define WL14_SEG 6144u
static bool
wl14_basename(const char *s)
{
	if (!*s || !((*s >= 'A' && *s <= 'Z') || (*s >= 'a' && *s <= 'z') || (*s >= '0' && *s <= '9'))) { return false; }
	for (unsigned i = 0; i < sizeof(wl14_bdf_name)-1; i++) {
		char c = s[i];
		if (!c) { return true; }
		if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
		    (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')) { return false; }
		if (c == '.' && i && s[i-1] == '.') { return false; }
	}
	return false;
}
#if defined(FUXI_SALAMI)
/* K-W1: a 65-byte firmware basename from knob ARG (boot-arg, else ADT
 * /defaults: wl_arg_str), else FALLBACK (a literal, SIZE = sizeof it).
 * false = truncated or not a plain basename. */
static bool
wl_salami_name(const char *arg, char *name, const char *fallback, size_t size)
{
	bool given;
	bzero(name, 65);
	given = PE_parse_boot_arg_str(arg, name, 65);
	if (!given && size <= 65) { memcpy(name, fallback, size); }
	if (name[63] || !wl14_basename(name)) {
		printf("FUXI WLAN: salami invalid %s basename; refused\n", arg);
		return false;
	}
	printf("FUXI WLAN: salami %s %s (%s)\n", arg, name, given ? "boot-arg or ADT" : "salami default");
	return true;
}
#endif
static bool wl_stop22, wl_keep, wl_bhie_fired, wl_amss_reached;
static bool wl_sbl_ready, wl_contexts_published, wl_sbl_m0;
static uint32_t wl_bhie_seq;
/* Register-layout facts, not imported loader code: OpenBSD if_qwz_pci.c
 * MHI definitions (BSD/ISC), revision 1.13, and M4/W9 BAR observations.
 * Full provenance and the READY -> contexts -> M0 -> FBC plan: wlan-w10.md.
 * All BHIe offsets are relative to the discovered BHIEOFF, never absolute. */
#define WL_BHIE_TXADDR 0x2cu
#define WL_BHIE_TXSIZE 0x34u
#define WL_BHIE_TXDB 0x3cu
#define WL_BHIE_TXSTATUS 0x44u
static struct wl_dma wl_rddm, wl_rxvectors, wl_msi_sink;
#define WL_RDDM_SIZE 0x780000u /* ath12k WCN7850; crash sink only, never dumped */
static unsigned wl_pre, wl_full, wl_vec, wl_fw;
static unsigned wl_cnss;
static bool wl_pdc_attempted;
static unsigned wl_pdc_acks;
static const char *const wl_cnss_names[] = {"off", "pdc"};
static bool wl_variant_invalid;
static const char *const wl_pre_names[] = {"off", "qfprom", "warm", "reset", "wake", "l1ss"};
static const char *const wl_fw_names[] = {"off", "stock"};
static const char *const wl_vec_names[] = {"off", "early", "rddm"};

static unsigned
wl_variant_arg(const char *name, const char *const *names, unsigned count)
{
	char arg[16] = {0};
	if (!PE_parse_boot_argn(name, arg, sizeof(arg))) { return 0; }
	arg[sizeof(arg) - 1] = 0;
	for (unsigned i = 0; i < count; i++) {
		if (!strcmp(arg, names[i])) { return i; }
	}
	wl_variant_invalid = true;
	return 0;
}

/* Independent default-off sequence deltas; register facts: docs/wlan-m6o.md. */
enum { WL_O_CYCLE=1, WL_O_PARF=2, WL_O_DBI=4, WL_O_EQ=8,
    WL_O_HOSTIRQ=16, WL_O_MSI=32, WL_O_POR=64, WL_O_PREFIX=128,
    WL_O_CYCLERAILS=256 };
static unsigned wl_seq_flags;
static bool wl_cycle_attempted, wl_seq_requested;
static unsigned wl_msi_cap, wl_pm_cap;
static const char *const wl_seq_names[] = {"cycle", "parf", "dbi", "eq", "hostirq", "msi", "por", "prefix", "cyclerails"};
static bool wl_m6o_parf(void) { return !!(wl_seq_flags & WL_O_PARF); }
static bool wl_m6o_early(void) { return !!(wl_seq_flags & (WL_O_CYCLE | WL_O_MSI)); }
static void
wl_seq_configure(void)
{
	char arg[96] = {0};
	wl_seq_flags = 0; wl_seq_requested = false;
	if (!PE_parse_boot_argn("fuxiwlanseq", arg, sizeof(arg))) { return; }
	wl_seq_requested = true;
	/* A sentinel prevents a truncated comma-list from becoming valid. */
	if (arg[sizeof(arg)-1]) { wl_variant_invalid = true; return; }
	if (!strcmp(arg, "off")) { return; }
	char *part = arg;
	while (*part) {
		char *end = part;
		while (*end && *end != ',') { end++; }
		bool more = *end == ','; *end = 0;
		unsigned bit = 0;
		for (unsigned i = 0; i < sizeof(wl_seq_names)/sizeof(wl_seq_names[0]); i++) {
			if (!strcmp(part, wl_seq_names[i])) { bit = 1u << i; break; }
		}
		if (!bit || (wl_seq_flags & bit)) { wl_variant_invalid = true; break; }
		wl_seq_flags |= bit;
		if (!more) { break; }
		part = end + 1;
		if (!*part) { wl_variant_invalid = true; }
	}
	if (!*arg) { wl_variant_invalid = true; }
	/* Imply cycle only after duplicate detection, independent of list order. */
	if (wl_seq_flags & WL_O_CYCLERAILS) { wl_seq_flags |= WL_O_CYCLE; }
}

static bool wl_variant_active(void) { return wl_pre || wl_full || wl_vec || wl_fw || wl_cnss || wl_seq_flags; }
static unsigned wl_bhi_bytes(void) { return wl_full ? wl_fw_size : WL_CHUNK; }
enum wl_probe_mode { WL_PROBE_OFF, WL_PROBE_UNMAPPED, WL_PROBE_PATTERN,
    WL_PROBE_CIVAC, WL_PROBE_NC, WL_PROBE_INVALID };
static enum wl_probe_mode wl_probe;
static bool wl_probe_fired;
static uint64_t wl_perst_done, wl_link_done;
static const char *const wl_probe_names[] = {"off", "unmapped", "pattern", "civac", "nc", "invalid"};

static void
wl_probe_configure(void)
{
	char arg[16] = {0};
	const char *const sizes[] = {"off", "full"};
	wl_variant_invalid = false;
	wl_pre = wl_variant_arg("fuxiwlanpre", wl_pre_names, 6);
	wl_fw = wl_variant_arg("fuxiwlanfw", wl_fw_names, 2);
	wl_full = wl_variant_arg("fuxiwlanbhi", sizes, 2);
	wl_vec = wl_variant_arg("fuxiwlanvec", wl_vec_names, 3);
	wl_cnss = wl_variant_arg("fuxiwlancnss", wl_cnss_names, 2);
	wl_seq_configure();
	int rails = 0;
	wl_rails_requested = PE_parse_boot_argn("fuxiwlanrails", &rails, sizeof(rails));
	wl_rails_variant = 0;
	if (rails < 0 || rails > 2) { wl_variant_invalid = true; }
	else { wl_rails_variant = (unsigned)rails; }
	if (wl_rails_variant && (wl_seq_flags & WL_O_CYCLERAILS)) {
		wl_variant_invalid = true;
		printf("FUXI WLAN: rails ON-only conflicts with cyclerails; refused before power writes\n");
	}
	if (wl_rails_requested && rails) {
		printf("FUXI WLAN: rails variant %d (1 early before WLAN_EN, 2 late before WLAN_MODE) valid %u\n",
		    rails, !wl_variant_invalid);
	}
	int stop22 = 0, keep = 0, qrtr = 0, w20 = 0;
	(void)PE_parse_boot_argn("fuxiwlanstop22", &stop22, sizeof(stop22));
	(void)PE_parse_boot_argn("fuxiwlankeep", &keep, sizeof(keep));
	(void)PE_parse_boot_argn("fuxiwlanqrtr", &qrtr, sizeof(qrtr));
	wl_w23 = qrtr == 7; /* C2: the W23 passive scan runs on top of W20..W22 */
	wl_w20 = qrtr == 6 || wl_w23; /* W20 runs on top of the whole W11..W15 chain */
	wl20_qrtr = wl_w20 ? (unsigned)qrtr : 0;
	wl_w15 = qrtr == 5 || wl_w20; wl_w14 = qrtr == 4 || wl_w15; wl_w13 = qrtr == 3 || wl_w14; wl_w12 = qrtr == 2 || wl_w13; wl_qrtr = qrtr == 1 || wl_w12; wl_w11_attempted = wl_w11_ok = false;
	wl12_mlo_variant = 0; wl17_bits = 0;
	{
		int variant = 0;
		bool present = PE_parse_boot_argn("fuxiwlanmlo", &variant, sizeof(variant));
		bool bad = variant < 0 || variant > (int)(WL17_MASK | 3) || (variant & 3) == 3 ||
		    ((variant & WL17_MLOALL) && (variant & 3) == 2) || ((variant & WL17_PHY17) && (variant & 3) != 1);
		/* Before W12 only the non-QMI P17 bits apply; the legacy selector stays ignored. */
		if (present && (wl_w12 || (variant > 0 && (variant & WL17_MASK)))) {
			if (bad) { wl_variant_invalid = true; }
			else {
				wl17_bits = (unsigned)variant & WL17_MASK;
				if (wl_w12) { wl12_mlo_variant = (unsigned)variant & 3; }
			}
			if (!wl17_bits && !bad) {
				if (variant) { printf("FUXI WLAN: W12 mlo variant %d (1 PHY_CAP, 2 omit MLO) valid %u\n", variant, !wl_variant_invalid); }
			} else {
				printf("FUXI WLAN: W12 mlo variant %d (1 PHY_CAP, 2 omit MLO) P17 bits %#x (hostmem %u mloall %u phy17 %u dmaalign %u clkcheck %u clkon %u errdec %u) valid %u\n",
				    variant, wl17_bits, !!(wl17_bits & WL17_HOSTMEM), !!(wl17_bits & WL17_MLOALL), !!(wl17_bits & WL17_PHY17),
				    !!(wl17_bits & WL17_DMAALIGN), !!(wl17_bits & WL17_CLKCHECK), !!(wl17_bits & WL17_CLKON),
				    !!(wl17_bits & WL17_ERRDEC), !wl_variant_invalid);
			}
		}
	}
	wl15_variant = 0;
	wl16_variant = 0; wl16_large = wl16_redrive = false;
	if (wl_w15) {
		int variant = 0;
		(void)PE_parse_boot_argn("fuxiwlanw15", &variant, sizeof(variant));
		if (variant < 0 || variant > 1) { wl_variant_invalid = true; }
		else { wl15_variant = (unsigned)variant; }
		printf("FUXI WLAN: W15 variant %u (0 default, 1 omit shadow v3) valid %u\n", wl15_variant, !wl_variant_invalid);
		variant = 0;
		(void)PE_parse_boot_argn("fuxiwlanw16", &variant, sizeof(variant));
		if (variant < 0 || variant > 4) { wl_variant_invalid = true; }
		else { wl16_variant = (unsigned)variant; }
		wl16_large = wl16_variant == 1 || wl16_variant == 3;
		wl16_redrive = wl16_variant == 2 || wl16_variant == 3;
		if (variant) {
			printf("FUXI WLAN: W16 variant %d large-IPCR %u doorbell-redrive %u diagnostics %u valid %u\n",
			    variant, wl16_large, wl16_redrive, !!wl16_variant, !wl_variant_invalid);
		}
	}
	/* W20: qrtr outside 0..WL20_QRTR_MAX used to run silently without QRTR;
	 * it is refused now, like w20 without qrtr 6/7, bits outside the set of
	 * the selected stage (0x08 ACTIVE and 0x20 FULLBSSID are in neither),
	 * 0x01 with 0x02, and keep or stop22 with qrtr 6/7 (no keep path exists
	 * for live copy engines). C2c: 0x100 without 0x02 is refused (with 0x01
	 * too, through the 0x01 + 0x02 rule); qrtr 6 refuses every W26 bit. C2d:
	 * 0x800 or 0x1000 without 0x100 is refused; qrtr 6 refuses both. C2e:
	 * 0x2000 without 0x100 is refused; qrtr 6 refuses it. */
	wl20_bits = 0;
	bool w20_given = PE_parse_boot_argn("fuxiwlanw20", &w20, sizeof(w20));
	if (qrtr < 0 || qrtr > WL20_QRTR_MAX) {
		wl_variant_invalid = true;
		printf("FUXI WLAN: W20 fuxiwlanqrtr=%d is not 0..%d in this build; refused\n", qrtr, WL20_QRTR_MAX);
	}
	if (wl_w20) {
		unsigned allowed = wl_w23 ? WL23_BITS : WL20_BITS;
		if (w20 < 0 || ((unsigned)w20 & ~allowed) || ((unsigned)w20 & (WL20_NOHTT | WL20_HTTVER)) == (WL20_NOHTT | WL20_HTTVER) ||
		    ((unsigned)w20 & (WL20_RXRING | WL20_HTTVER)) == WL20_RXRING || stop22 == 1 || keep == 1 ||
		    (((unsigned)w20 & WL27_BITS) && !((unsigned)w20 & WL20_RXRING)) ||
		    (((unsigned)w20 & WL20_MONSTAT) && !((unsigned)w20 & WL20_RXRING)) ||
		    (((unsigned)w20 & WL20_L19ORDER) && !((unsigned)w20 & WL20_REORX))) { wl_variant_invalid = true; }
		else { wl20_bits = (unsigned)w20; }
		printf("FUXI WLAN: W20 selected qrtr=%d (%s) w20 %#x valid %u\n", qrtr,
		    wl_w23 ? "CE/HTC/WMI init, then W23 passive scan" : "CE/HTC/WMI init to READY", (unsigned)w20, !wl_variant_invalid);
		if (wl_w23) {
			printf("FUXI WLAN: W23 selected passive scan of %s, dwell 150 ms, nothing transmitted; BSSIDs OUI only, pdev %s, htt version %s\n",
			    (wl20_bits & WL20_BAND24) ? "13 channels (2.4 GHz only)" : "38 channels",
			    (wl20_bits & WL20_PDEVCAP) ? "from EXT caps" : "0", (wl20_bits & WL20_HTTVER) ? "asked" : "not asked");
		}
		if (wl20_bits & WL26_BITS) {
			printf("FUXI WLAN: W26 selected: rx rings %s; wbm idle list %s; chain mask %s; nothing selected to host rings\n",
			    (wl20_bits & WL20_RXRING) ? "4095 buffers, rings 160/161/169/186 (w20 0x100)" : "no",
			    (wl20_bits & WL20_IDLELINK) ? "32767 link descriptors (w20 0x200)" : "no",
			    (wl20_bits & WL20_CHAINMASK) ? "pdev tx/rx (w20 0x400)" : "no");
		}
		if (wl20_bits & WL27_BITS) {
			printf("FUXI WLAN: W27 selected: ring selection %s; reo rx path %s; nothing transmitted\n",
			    (wl20_bits & WL20_RXSEL) ? "mac0/mac1 2048 B, mpdu-start/packet/msdu-end tlvs, data filters (w20 0x800)" : "no",
			    (wl20_bits & WL20_REORX) ? "rings 131/0/1-8, buffers back to ring 160 (w20 0x1000)" : "no");
		}
		if (wl20_bits & WL20_MONSTAT) {
			printf("FUXI WLAN: W28 selected: monitor status rings 185/193, 1023 buffers each (w20 0x2000), watched, not reaped; "
			    "crash line at the COMPLETED timeout and before MHI reset; nothing transmitted\n");
		}
		if (wl20_bits & WL20_L19ORDER) {
			printf("FUXI WLAN: W29 selected: L19 dp_alloc order, cookie conversion (CMEM from QMI), common rings sw2wbm/tcl+comp x3/"
			    "reinject/reo-cmd/reo-status programmed never posted, mac-start pdev params and vdev encap/decap (w20 0x4000); nothing transmitted\n");
		}
	} else if (w20_given && w20) {
		wl_variant_invalid = true;
		printf("FUXI WLAN: W20 fuxiwlanw20 %#x needs fuxiwlanqrtr=6; refused\n", (unsigned)w20);
	}
	wl_stop22 = stop22 == 1; wl_keep = keep == 1;
	if (wl_w14) {
		bzero(wl14_bdf_name, sizeof(wl14_bdf_name));
		if (!PE_parse_boot_arg_str("fuxiwlanbdf", wl14_bdf_name, sizeof(wl14_bdf_name))) {
#if defined(FUXI_SALAMI)
			/* salami Android: /odm/etc/wifi/kiwi/bdwlan.b0e (board id 0xff, region 22) */
			memcpy(wl14_bdf_name, "bdwlan.b0e", sizeof("bdwlan.b0e"));
#else
			memcpy(wl14_bdf_name, "bd_m3gl.elf", sizeof("bd_m3gl.elf"));
#endif
		}
		if (wl14_bdf_name[sizeof(wl14_bdf_name)-2] || !wl14_basename(wl14_bdf_name)) {
			wl_variant_invalid = true;
			printf("FUXI WLAN: W14 invalid fuxiwlanbdf basename; refused\n");
		} else { printf("FUXI WLAN: W14 selected BDF %s (override fuxiwlanbdf; default unverified)\n", wl14_bdf_name); }
#if defined(FUXI_SALAMI)
		/* salami Android: /odm/etc/wifi/kiwi/regdb.bin and the M3 image
		 * /vendor/firmware_mnt/image/kiwi/phy_ucode20.elf */
		if (!wl_salami_name("fuxiwlanregdb", wl14_regdb_name, "regdb.bin", sizeof("regdb.bin")) ||
		    !wl_salami_name("fuxiwlanm3", wl14_m3_name, "phy_ucode20.elf", sizeof("phy_ucode20.elf"))) {
			wl_variant_invalid = true;
		}
#endif
	}
#if defined(FUXI_SALAMI)
	bzero(wl_amss_path, sizeof(wl_amss_path));
	{
		char amss[65] = {0};
		if (PE_parse_boot_arg_str("fuxiwlanamss", amss, sizeof(amss))) {
			if (amss[sizeof(amss)-2] || !wl14_basename(amss) || wl_fw) {
				wl_variant_invalid = true;
				printf("FUXI WLAN: salami invalid fuxiwlanamss basename (or with fuxiwlanfw=stock); refused\n");
			} else {
				snprintf(wl_amss_path, sizeof(wl_amss_path), "/usr/local/share/fuxi/wlan/%s", amss);
				printf("FUXI WLAN: salami fuxiwlanamss %s (replaces the SoC-major choice)\n", wl_amss_path);
			}
		}
	}
#endif
	wl_bhie_fired = wl_amss_reached = false;
	wl_sbl_ready = wl_contexts_published = wl_sbl_m0 = false;
	wl_bhie_seq = 0;
	if ((wl_seq_flags & WL_O_PREFIX) && (wl_full || wl_vec == 1)) { wl_variant_invalid = true; }
	wl_probe = WL_PROBE_OFF; wl_probe_fired = false;
	if (PE_parse_boot_argn("fuxiwlanprobe", arg, sizeof(arg))) {
		arg[sizeof(arg) - 1] = 0;
		wl_probe = WL_PROBE_INVALID;
		for (unsigned i = 0; i < WL_PROBE_INVALID; i++) {
			if (!strcmp(arg, wl_probe_names[i])) { wl_probe = (enum wl_probe_mode)i; break; }
		}
	}
	if (wl_variant_active() && (wl_probe == WL_PROBE_UNMAPPED || wl_probe == WL_PROBE_PATTERN)) {
		wl_variant_invalid = true;
	}
	int retain = 0;
	(void)PE_parse_boot_argn("fuxiwlanaspm", &retain, sizeof(retain));
	if (wl_pre == 5 && retain == 1) { wl_variant_invalid = true; }
	printf("FUXI WLAN: W10 continue=%u keep=%u (stop22=1 retains BHI diagnostics; no data channels)\n", !wl_stop22, wl_keep);
	printf("FUXI WLAN: W11 qrtr=%u keep=%u (one conversation after AMSS/M0)\n", wl_qrtr, wl_keep);
	if (wl_w12) { printf("FUXI WLAN: W12 selected qrtr=%u target memory cap %u bytes\n", wl_w20 ? wl20_qrtr : wl_w15 ? 5 : wl_w14 ? 4 : wl_w13 ? 3 : 2, WL_TARGET_CAP); }
	if (wl_w13) { printf("FUXI WLAN: W13 small-chunk policy limit %u bytes; one zero-segment negotiation, aligned IOVAs\n", WL13_CHUNK_LIMIT); }
	printf("FUXI WLAN: M6o seq mask %#x valid=%u (selectors default off)\n",
	    wl_seq_flags, !wl_variant_invalid);
	printf("FUXI WLAN: M6l pre=%s bhi=%s vec=%s valid=%u\n",
	    wl_pre_names[wl_pre], sizes[wl_full], wl_vec_names[wl_vec], !wl_variant_invalid);
	printf("FUXI WLAN: M6k probe=%s (one BHI doorbell; unmapped/pattern always stop at 22)\n", wl_probe_names[wl_probe]);
	printf("FUXI WLAN: M6m fw=%s l1ss=%u valid=%u (stock means supplied input, not verified provenance)\n",
	    wl_fw_names[wl_fw], wl_pre == 5, !wl_variant_invalid);
	printf("FUXI WLAN: M6n cnss=%s valid=%u (PDC before step 1)\n",
	    wl_cnss_names[wl_cnss], !wl_variant_invalid);
}
#define WL_CNSS_IOVA 0xa0000000ull
#define WL_CNSS_IOVA_END 0xb0000000ull
#define WL_LEGACY_IOVA 0x10000000ull
#define WL_LEGACY_IOVA_END 0x11000000ull
_Static_assert((WL_CNSS_IOVA >> 30) == ((WL_CNSS_IOVA_END - 1) >> 30) &&
    (WL_LEGACY_IOVA >> 30) == ((WL_LEGACY_IOVA_END - 1) >> 30), "one L2 per pool");
_Static_assert(!((WL_CNSS_IOVA | WL_CNSS_IOVA_END | WL_LEGACY_IOVA |
    WL_LEGACY_IOVA_END) & ((1ull << 21) - 1)), "2-MiB pool alignment");
static uint64_t wl_iova_base = WL_CNSS_IOVA, wl_iova_end = WL_CNSS_IOVA_END;
static uint64_t wl_iova_next = WL_CNSS_IOVA, wl_cb_base, wl_gr1;
/* Xiaomi SM8550 arm-smmu + io-pgtable-fast: coherent walk = ISH/WB-WA;
 * fastmap cached leaves = OSH + MAIR 0xff. See docs-lanes/wlan-bt.md, M6g.
 * Keep EPD1/T1SZ/TG1 legal, TTBR0 4-KiB granule and 32-bit input. */
#define WL_CB_TCR (0x80a00000u | (3u << 12) | (1u << 10) | (1u << 8) | 32u)
/* HUPCF=1, CFCFG=0 (stall-disable), CFRE=0 (no-CFRE), M=1.
 * ASIDPNE/AFE/TRE retained as optional Linux-style hints. CFIE stays off:
 * this lane polls FSR and has no context-fault IRQ handler. Coherent SCTLR
 * bypass overrides only apply with M=0, when we keep bus mastering off. */
#define WL_CB_SCTLR 0x1107u
#define WL_DMA_PTE 0x60000000000e63ull /* XN, nG, AF, OSH, RW, NS, AttrIdx 0 */
static unsigned wl_slot, wl_cb, wl_smr_count, wl_cb_count, wl_pgshift;
static uint32_t wl_old_smr, wl_old_s2cr, wl_old_sctlr;
#define WL_CB_PROBE_MAX 8u
static bool wl_cb_unusable[128];
static unsigned wl_smmu_mismatches;
static uint32_t wl_route_cfg;
static unsigned wl_route_hash;
static bool wl_owned, wl_enabled, wl_route_owned, wl_route_installed;
static int wl_contain_wr(uint64_t, uint32_t);
static int wl_smmu_fault(unsigned);
/* Packed MHI contexts are 44 bytes; the old research note's 40 was wrong. */
struct wl_context {
	uint32_t cfg, type, index;
	uint64_t base, len, rp, wp;
} __attribute__((packed));
_Static_assert(sizeof(struct wl_context) == 44, "MHI context stride");
_Static_assert(__builtin_offsetof(struct wl_context, base) == 12 &&
    __builtin_offsetof(struct wl_context, len) == 20 &&
    __builtin_offsetof(struct wl_context, rp) == 28 &&
    __builtin_offsetof(struct wl_context, wp) == 36, "MHI context wire offsets");
struct wl_vector { uint64_t addr, size; };
_Static_assert(sizeof(struct wl_vector) == 16, "BHIe vector stride");
/* Within a single 16 KiB page: 128 disabled channels, two event contexts,
 * command context, 512-byte control ring, 4 KiB data ring, 2 KiB command ring.
 * Ring bases are aligned to their lengths. W11 transfer rings are separate. */
#define WL_ECTX 0x1800u
#define WL_CCTX 0x1900u
#define WL_ER0  0x1a00u
#define WL_ER1  0x2000u
#define WL_CR   0x3000u
_Static_assert(128 * sizeof(struct wl_context) <= WL_ECTX, "channel array");
_Static_assert(WL_CR + 2048 <= 16384, "context page");

/* Select once before the WLAN thread allocates anything. The experiment
 * changes only IOVAs; both boots use the CNSS coherency/fault profile. */
static void
wl_iova_configure(void)
{
	wl_probe_configure();
	int legacy = 0;
	(void)PE_parse_boot_argn("fuxiwlanoldiova", &legacy, sizeof(legacy));
	wl_iova_base = legacy == 1 ? WL_LEGACY_IOVA : WL_CNSS_IOVA;
	wl_iova_end = legacy == 1 ? WL_LEGACY_IOVA_END : WL_CNSS_IOVA_END;
	wl_iova_next = wl_iova_base;
	printf("FUXI WLAN: M6g IOVA %s pool [%#llx,%#llx) fuxiwlanoldiova=%d\n",
	    legacy == 1 ? "legacy" : "cnss", wl_iova_base, wl_iova_end, legacy);
}

static void
wl_sleep(unsigned ms)
{
	/* Yield this dedicated thread; root mount and other drivers must run. */
	assert_wait_timeout((event_t)&wl5_report, THREAD_UNINT, ms, NSEC_PER_MSEC);
	thread_block(THREAD_CONTINUE_NULL);
}

/* Independent QMP polling implementation. The only kernel QMP writer here is
 * this single WLAN thread; shim audio.c runs in a separate boot environment.
 * Public protocol/DT facts and pinned source references: docs/wlan-m6n.md.
 * Never overwrite a busy slot, clear an unacknowledged packet, or retry a
 * partially applied PDC table. Acknowledgement means consumption, not a
 * voltage readback. No security/fuse state is written. */
#define WL_AOSS_QMP 0x0c300000ull
#define WL_AOSS_IPCC 0x00408000ull

static int
wl_qmp_wait(unsigned offset, uint32_t expected)
{
	for (unsigned ms = 0; ms <= 1000; ms++) {
		uint32_t value = wl_prd(WL_AOSS_QMP + offset);
		if (fw_faults) { return 1; }
		if (value == expected) { return 0; }
		if (ms < 1000) { wl_sleep(1); }
	}
	printf("FUXI WLAN: M6n QMP timeout offset %#x expected %#x\n", offset, expected);
	return 1;
}

static int
wl_qmp_kick(void)
{
	__asm__ volatile("dsb sy" ::: "memory");
	if (fw_faults) { return 1; }
	wl_pwr(WL_AOSS_IPCC + 0xc, 0); /* AOP client 0 / QMP signal 0 */
	return fw_faults ? 1 : 0;
}

static int
wl_cnss_pdc(void)
{
	if (!wl_cnss) { return 0; }
	if (wl_variant_invalid || fw_faults || wl_pdc_attempted) { return 1; }
	wl_pdc_attempted = true;
	wl_pdc_acks = 0;
	if (wl_prd(WL_AOSS_QMP) != 0x4d41494c || wl_prd(WL_AOSS_QMP + 4) != 1) { return 2; }
	unsigned offset = wl_prd(WL_AOSS_QMP + 0x38), bytes = wl_prd(WL_AOSS_QMP + 0x34);
	if (fw_faults || (offset & 3) || offset < 0x40 || offset >= 0x400 ||
	    bytes < 68 || bytes > 0x400 - offset || wl_prd(WL_AOSS_QMP + offset)) { return 3; }
	uint32_t link = wl_prd(WL_AOSS_QMP + 0x28), channel = wl_prd(WL_AOSS_QMP + 0x30);
	if (fw_faults) { return 3; }
	if (link != 0xffff || channel != 0xffff) {
		uint32_t remote = wl_prd(WL_AOSS_QMP + 0xc);
		if (fw_faults || (remote && remote != 0xffff && remote != 0xffff0000)) { return 4; }
		wl_pwr(WL_AOSS_QMP + 0x10, remote);
		wl_pwr(WL_AOSS_QMP + 0x24, 0xffff);
		if (wl_qmp_kick() || wl_qmp_wait(0x28, 0xffff)) { return 4; }
		wl_pwr(WL_AOSS_QMP + 0x2c, 0xffff);
		if (wl_qmp_kick() || wl_qmp_wait(0x14, 0xffff)) { return 4; }
		wl_pwr(WL_AOSS_QMP + 0x18, 0xffff);
		if (wl_qmp_kick() || wl_qmp_wait(0x30, 0xffff)) { return 4; }
	}
	if (wl_prd(WL_AOSS_QMP + 0xc) != 0xffff || wl_prd(WL_AOSS_QMP + 0x14) != 0xffff ||
	    fw_faults) { return 4; }
	/* KIWI autonomous power settings, encoded as protocol data, not driver
	 * code imported from CNSS. RF voltage pairs first, then route MX to BB,
	 * enable BB PDC and set its MX/CX up/down values. */
	const char *const rf[] = {"s4e", "s4g", "s6g"};
	const unsigned volts[][2] = {{916, 612}, {1316, 944}, {1864, 1820}, {976, 800}, {940, 420}};
	for (unsigned i = 0; i < 13; i++) {
		char msg[64] = {0};
		int n;
		if (i < 6 || i >= 9) {
			unsigned row = i < 6 ? i / 2 : 3 + (i - 9) / 2;
			unsigned down = i < 6 ? i & 1 : (i - 9) & 1;
			n = snprintf(msg, sizeof(msg), "{class: wlan_pdc, ss: %s, res: %s.v, %sval: %u}",
			    i < 6 ? "rf" : "bb", i < 6 ? rf[row] : row == 3 ? "s2g" : "s5g",
			    down ? "dwn" : "up", volts[row][down]);
		} else {
			n = snprintf(msg, sizeof(msg), "{class: wlan_pdc, ss: %s, res: %s, enable: %u}",
			    i == 8 ? "bb" : "rf", i == 6 ? "s2g.m" : i == 7 ? "s2g.v" : "pdc", i == 8 ? 1u : 0u);
		}
		if (n <= 0 || (unsigned)n >= sizeof(msg) || fw_faults ||
		    wl_prd(WL_AOSS_QMP + offset)) { return 5; }
		printf("FUXI WLAN: M6n PDC %u/13 %s\n", i + 1, msg);
		for (unsigned word = 0; word < 16; word++) {
			uint32_t value = 0;
			for (unsigned b = 0; b < 4; b++) { value |= (uint32_t)(uint8_t)msg[4 * word + b] << (8 * b); }
			if (fw_faults) { return 5; }
			wl_pwr(WL_AOSS_QMP + offset + 4 + 4 * word, value);
		}
		if (fw_faults) { return 5; }
		__asm__ volatile("dsb sy" ::: "memory");
		wl_pwr(WL_AOSS_QMP + offset, 64);
		uint32_t length = wl_prd(WL_AOSS_QMP + offset); /* drain posted writes */
		if (fw_faults || (length != 64 && length != 0) || wl_qmp_kick() || wl_qmp_wait(offset, 0)) { return 6; }
		wl_pdc_acks++;
	}
	printf("FUXI WLAN: M6n PDC consumed %u/13 before WLAN rails; AOP policy persists until reconfigured\n", wl_pdc_acks);
	return 0;
}

/* Wired allocations need neither physical contiguity nor low DDR. */
static uint64_t
wl_phys(vm_offset_t va)
{
	return ptoa_64(pmap_find_phys(kernel_pmap, va)) + (va & PAGE_MASK);
}

static int
wl_dma_alloc(struct wl_dma *d, vm_size_t bytes)
{
	d->size = round_page(bytes);
	if ((wl17_bits & WL17_DMAALIGN) && d != &wl_tables && bytes && d->size >= bytes) {
		uint64_t align = wl17_align(d->size), next = (wl_iova_next + align - 1) & ~(align - 1);
		if (next >= wl_iova_next && next < wl_iova_end) {
			printf("FUXI WLAN: P17 dma align size %#llx align %#llx IOVA %#llx -> %#llx\n",
			    (uint64_t)d->size, align, wl_iova_next, next);
			wl_iova_next = next;
		}
	}
	if (!bytes || d->size < bytes || wl_iova_next < wl_iova_base ||
	    wl_iova_next >= wl_iova_end || d->size > wl_iova_end - wl_iova_next) { return 1; }
	kern_return_t kr = kmem_alloc(kernel_map, &d->va, d->size,
	    KMA_DATA | KMA_ZERO | KMA_NOPAGEWAIT, VM_KERN_MEMORY_IOKIT);
	if (kr != KERN_SUCCESS) {
		printf("FUXI WLAN: wired DMA allocation failed %d\n", kr); return 1;
	}
	for (vm_size_t off = 0; off < d->size; off += PAGE_SIZE) {
		uint64_t pa = wl_phys(d->va + off);
		if (!pa || pa >= (1ull << 36)) { return 2; }
		if (d == &wl_image && wl_probe == WL_PROBE_NC &&
		    pmap_cache_attributes((ppnum_t)(pa >> PAGE_SHIFT)) != VM_WIMG_DEFAULT) { return 3; }
	}
	if (d == &wl_image && wl_probe == WL_PROBE_NC) {
		/* Change ALL CPU aliases through pmap, including the physical aperture.
		 * WCOMB is Normal NC (MAIR 0x44); Device mappings can fault memcpy.
		 * Do this before VFS writes the image, restore only after containment. */
		d->nc = true;
		for (vm_size_t off = 0; off < d->size; off += PAGE_SIZE) {
			ppnum_t pn = (ppnum_t)(wl_phys(d->va + off) >> PAGE_SHIFT);
			pmap_set_cache_attributes(pn, VM_WIMG_WCOMB);
			if (pmap_cache_attributes(pn) != VM_WIMG_WCOMB) { return 3; }
		}
		printf("FUXI WLAN: M6k image CPU mapping Normal-NC WCOMB all aliases bytes %#llx\n", (uint64_t)d->size);
	}
	d->pa = wl_phys(d->va);
	d->iova = wl_iova_next; wl_iova_next += d->size;
	return 0;
}

/* W11 live contexts share cache lines between host WP and device RP. Use
 * Normal-NC on every CPU alias before publishing them: flushing an active WB
 * context could otherwise overwrite a newer device RP with a stale line. */
static int
wl_dma_nc(struct wl_dma *d)
{
	for (vm_size_t off = 0; off < d->size; off += PAGE_SIZE) {
		if (pmap_cache_attributes((ppnum_t)(wl_phys(d->va + off) >> PAGE_SHIFT)) != VM_WIMG_DEFAULT) { return 1; }
	}
	d->nc = true; /* restore all aliases even if the conversion fails halfway */
	for (vm_size_t off = 0; off < d->size; off += PAGE_SIZE) {
		ppnum_t pn = (ppnum_t)(wl_phys(d->va + off) >> PAGE_SHIFT);
		pmap_set_cache_attributes(pn, VM_WIMG_WCOMB);
		if (pmap_cache_attributes(pn) != VM_WIMG_WCOMB) { return 1; }
	}
	__asm__ volatile("dsb sy" ::: "memory");
	return 0;
}

static void
wl_dma_clean(struct wl_dma *d)
{
	if (d->nc) { __asm__ volatile("dsb sy" ::: "memory"); return; }
	for (vm_size_t off = 0; off < d->size; off += PAGE_SIZE) {
		flush_dcache64(wl_phys(d->va + off), PAGE_SIZE, TRUE);
	}
	__asm__ volatile("dsb sy" ::: "memory");
}

/* Explicit architectural clean+invalidate to PoC, regardless of CB attributes.
 * CTR_EL0.DminLine is log2(words); never assume an Apple cache-line size. */
static void
wl_dma_civac(struct wl_dma *d)
{
	if (d->nc) { __asm__ volatile("dsb sy" ::: "memory"); return; }
	uint64_t ctr = 0;
	__asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
	vm_size_t line = (vm_size_t)4 << ((ctr >> 16) & 15);
	for (vm_size_t off = 0; off < d->size; off += line) {
		vm_offset_t va = d->va + off;
		__asm__ volatile("dc civac, %0" :: "r"(va) : "memory");
	}
	__asm__ volatile("dsb sy" ::: "memory");
}

static void
wl_probe_publish(void)
{
	if (wl_probe != WL_PROBE_CIVAC) { return; }
	wl_dma_civac(&wl_image); wl_dma_civac(&wl_vectors);
	wl_dma_civac(&wl_contexts); wl_dma_civac(&wl_tables);
	if (wl_amss.va) { wl_dma_civac(&wl_amss); }
	if (wl_rddm.va) { wl_dma_civac(&wl_rddm); wl_dma_civac(&wl_rxvectors); }
	printf("FUXI WLAN: M6k DC CIVAC + DSB SY image/vectors/contexts/tables before BHI doorbell\n");
}

static void
wl_dma_free(struct wl_dma *d)
{
	if (d->va) {
		if (d->nc) {
			for (vm_size_t off = 0; off < d->size; off += PAGE_SIZE) {
				pmap_set_cache_attributes((ppnum_t)(wl_phys(d->va + off) >> PAGE_SHIFT), VM_WIMG_DEFAULT);
			}
		}
		kmem_free(kernel_map, d->va, d->size);
		bzero(d, sizeof(*d));
	}
}

static uint32_t wl_mrd(uint32_t off) { return wl_prd((uint64_t)WL_BAR0 + off); }
static void wl_mwr(uint32_t off, uint32_t v) { wl_pwr((uint64_t)WL_BAR0 + off, v); }
static void
wl_mwr64(uint32_t off, uint64_t v)
{
	wl_mwr(off + 4, (uint32_t)(v >> 32));
	wl_mwr(off, (uint32_t)v);
}

/* The only caller is the dedicated bring-up thread. Save/restore the dynamic
 * window for every access, including fault paths; never leave a log window set.
 * CNSS pci.c/reg.h: WINDOW_SHIFT=19, mask=0x3f, start=0x80000. */
static int
wl_soc_read(uint32_t off, uint32_t *val)
{
	*val = ~0u;
	if (fw_faults || (off & 3) || off >= 0x02000000) { return 1; }
	uint32_t saved = wl_mrd(0x310c), win = (saved & ~0x3fu) | 0x40000000 | (off >> 19);
	if (saved == ~0u || fw_faults) { return 1; }
	wl_mwr(0x310c, win);
	int e = wl_mrd(0x310c) != win || fw_faults;
	if (!e) { *val = wl_prd(WL_BAR0 + 0x80000ull + (off & 0x7ffff)); }
	/* Containment writes remain possible after a latched synchronous fault. */
	if (wl_contain_wr(WL_BAR0 + 0x310c, saved) || wl_mrd(0x310c) != saved) { e = 1; }
	return e || fw_faults;
}

/* Independent register transactions; behavioral references and chip-specific
 * addresses are linked in docs/wlan-m6l.md. Preserve/verify the dynamic window
 * even on an exception. No raw arbitrary-address boot argument is accepted. */
static int
wl_soc_write(uint32_t off, uint32_t value, bool verify)
{
	if (fw_faults || (off & 3) || off >= 0x02000000) { return 1; }
	uint32_t saved = wl_mrd(0x310c), win = (saved & ~0x3fu) | 0x40000000 | (off >> 19);
	if (saved == ~0u || fw_faults) { return 1; }
	wl_mwr(0x310c, win);
	int e = wl_mrd(0x310c) != win || fw_faults;
	if (!e) {
		uint64_t pa = WL_BAR0 + 0x80000ull + (off & 0x7ffff);
		wl_pwr(pa, value);
		e = fw_faults || (verify && wl_prd(pa) != value);
	}
	if (wl_contain_wr(WL_BAR0 + 0x310c, saved) || wl_mrd(0x310c) != saved) { e = 1; }
	printf("FUXI WLAN: M6l SOC write [%#x] %#x %s window restore checked\n", off, value, e ? "FAILED" : "ok");
	return e || fw_faults;
}

static int
wl_pre_bhi(void)
{
	if (!wl_pre) { return 0; }
	if (wl_probe_fired || fw_faults || wl_prd(WL_EPCFG) != 0x110717cb ||
	    ((wl5_report[16] >> 8) & 15) != 2 || (wl_prd(WL_EPCFG + 4) & 4) ||
	    wl_mrd(wl_bhi + 0x28) != 0) { return 1; }
	uint32_t v, cookie, cause;
	printf("FUXI WLAN: M6l pre-BHI %s begin BME off EE PBL\n", wl_pre_names[wl_pre]);
	if (wl_pre == 4) {
		wl_mwr(0x3004, 1); wl_sleep(5);
		if (fw_faults || wl_mrd(0x3004) != 1) { return 1; }
	}
	if (wl_pre == 3) {
		if (wl_soc_read(0x01e081b0, &v) || v == ~0u) { return 1; }
		if (v != 0x111 && wl_soc_write(0x01e081b0, 0x111, true)) { return 1; }
		/* WCN7850 corrected hot-reset register, NOT v6.6's QCN9274 address. */
		if (wl_soc_read(0x01e40304, &v) || v == ~0u ||
		    wl_soc_write(0x01e40304, v | 0x10, true)) { return 1; }
		wl_sleep(5);
		if (wl_soc_write(0x01e08228, ~0u, false)) { return 1; } /* W1C */
	}
	if (wl_pre == 1 || wl_pre == 3) {
		if (wl_soc_read(0x01f8031c, &v) || v == ~0u) { return 1; }
		unsigned mask = wl_pre == 1 ? 5 : 4; /* CNSS vs ath12k */
		printf("FUXI WLAN: M6l QFPROM before %#x clear mask %#x\n", v, mask);
		if (wl_soc_write(0x01f8031c, v & ~mask, true)) { return 1; }
	}
	if (wl_pre == 3) {
		const unsigned clear[] = {0x3c, 0x44, 0x70, 0x78};
		for (unsigned i = 0; i < 4; i++) {
			wl_mwr(wl_bhie + clear[i], 0);
			if (wl_mrd(wl_bhie + clear[i]) || fw_faults) { return 1; }
		}
	}
	if (wl_pre == 2 || wl_pre == 3) {
		if (wl_soc_read(0x01f80500, &cookie) || cookie == ~0u ||
		    wl_soc_read(0x01f80504, &v) || v == ~0u) { return 1; }
		printf("FUXI WLAN: M6l warm cookie %#x entry %#x\n", cookie, v);
		wl_sleep(10);
		if (wl_soc_write(0x01f80504, 0, true)) { return 1; }
		wl_sleep(10);
		if (wl_soc_read(0x01f8060c, &cause) || cause == ~0u) { return 1; } /* read-clear */
		printf("FUXI WLAN: M6l warm entry cleared reset cause %#x (read-clear)\n", cause);
	}
	if (wl_pre == 3) {
		v = wl_mrd(0x3008);
		if (v == ~0u || fw_faults) { return 1; }
		wl_mwr(0x3008, v | 1); wl_sleep(10);
		/* Always try to release our reset even if the intervening read faults. */
		if (wl_contain_wr(WL_BAR0 + 0x3008, v & ~1u)) { return 1; }
		wl_sleep(10);
		if (wl_mrd(0x3008) == ~0u || (wl_mrd(0x3008) & 1) || fw_faults) { return 1; }
		wl_mwr(0x38, 2); wl_sleep(10);
		for (unsigned i = 0; i < 40 && (wl_mrd(0x38) & 2) && !fw_faults; i++) { wl_sleep(25); }
		if (fw_faults || (wl_mrd(0x38) & 2) || (wl_mrd(0x48) & 0xff04) ||
		    wl_mrd(0x28) != wl_bhi || wl_mrd(0x2c) != wl_bhie) { return 1; }
	}
	if (fw_faults || wl_mrd(wl_bhi + 0x28) != 0 || (wl_prd(WL_EPCFG + 4) & 4)) { return 1; }
	printf("FUXI WLAN: M6l pre-BHI %s complete BME off EE PBL\n", wl_pre_names[wl_pre]);
	return 0;
}

/* CNSS KIWI bootloader diagnostics only, <=27 words, no arbitrary SRAM dump.
 * In PBL the DBG tuple may still be the input DMA address/size, not an SBL log. */
static void
wl_boot_logs(void)
{
	uint32_t ee = wl_mrd(wl_bhi + 0x28);
	if (fw_faults || (ee != 0 && ee != 1)) {
		printf("FUXI WLAN: M6f boot logs skipped EE %#x faults %u\n", ee, fw_faults); return;
	}
	const uint32_t regs[] = {0x01b000f8, 0x01e22b34, 0x01a10008, 0x01e0e238, 0x01e0e23c};
	const char *names[] = {"TCSR_PBL_LOGGING", "PBL_WLAN_BOOT_CFG", "KIWI_PBL_BOOTSTRAP_STATUS",
	    "PCIE_BHI_ERRDBG2", "PCIE_BHI_ERRDBG3"};
	uint32_t v[5];
	for (unsigned i = 0; i < 5; i++) {
		if (wl_soc_read(regs[i], &v[i])) {
			printf("FUXI WLAN: M6f boot log access failed reg %#x; window restored/containment attempted\n", regs[i]); return;
		}
		printf("FUXI WLAN: M6f %s [%#x] %#x\n", names[i], regs[i], v[i]);
	}
	for (unsigned i = 0; i < 10; i++) {
		uint32_t value, addr = 0x01403d98 + 4 * i;
		if (wl_soc_read(addr, &value)) { return; }
		printf("FUXI WLAN: M6f PBL log [%#x] %#x\n", addr, value);
	}
	uint32_t start = v[3], size = v[4];
	/* Clamp like CNSS, but require word alignment and subtraction-safe bounds. */
	if (size > 48) { size = 48; }
	if (start < 0x01400000 || start >= 0x01800000 || (start & 3) ||
	    !size || (size & 3) || size > 0x01800000 - start) {
		printf("FUXI WLAN: M6f SBL log skipped invalid SRAM tuple %#x/%#x (runtime DBG %#x/%#x)\n",
		    start, v[4], wl_mrd(wl_bhi + 0x38), wl_mrd(wl_bhi + 0x3c)); return;
	}
	for (unsigned i = 0; i < size; i += 4) {
		uint32_t value;
		if (wl_soc_read(start + i, &value)) { return; }
		printf("FUXI WLAN: M6f SBL log [%#x] %#x\n", start + i, value);
	}
}

static void
wl5_status(unsigned step)
{
	if (!wl_mhi_valid) {
		return;
	}
	uint32_t *r = wl5_report;
	r[64] = wl_mrd(wl_bhi + 0x28); r[65] = wl_mrd(0x48);
	r[66] = wl_mrd(0x38); r[67] = wl_mrd(wl_bhi + 0x2c);
	r[68] = wl_mrd(wl_bhi + 0x30);
	for (unsigned i = 0; i < 3; i++) { r[69 + i] = wl_mrd(wl_bhi + 0x34 + 4 * i); }
	r[72] = wl_mrd(wl_bhie + 0x44); r[73] = wl_prd(WL_EPCFG + 4);
	printf("FUXI WLAN: M6 step %u EE %#x (%s) MHISTATUS %#x (%s ready %u) MHICTRL %#x BHI %#x ERRCODE %#x ERRDBG %#x/%#x/%#x BHIE %#x EPcmd %#x\n",
	    step, r[64], wl_mhi_ee(r[64]), r[65], wl_mhi_state((r[65] >> 8) & 255),
	    r[65] & 1, r[66], r[67], r[68], r[69], r[70], r[71], r[72], r[73]);
}

enum {
	WL_DIAG_STATUS_ONES = 1, WL_DIAG_BAR_LOST = 2, WL_DIAG_CONFIG_LOST = 4,
	WL_DIAG_LINK_LOST = 8, WL_DIAG_DECODE_CHANGED = 16, WL_DIAG_PERST = 32,
	WL_DIAG_NON_D0 = 64, WL_DIAG_PBL_ERROR = 128, WL_DIAG_MMIO_FAULT = 256,
	WL_DIAG_CAP_INVALID = 512,
	WL_DIAG_TRANSPORT = WL_DIAG_BAR_LOST | WL_DIAG_CONFIG_LOST | WL_DIAG_LINK_LOST |
	    WL_DIAG_DECODE_CHANGED | WL_DIAG_PERST | WL_DIAG_NON_D0 | WL_DIAG_MMIO_FAULT
};

/* M6e diagnostics are read-only. Missing capabilities use ~0, malformed
 * lists set a separate flag; neither is mistaken for a D3/AER indication. */
static unsigned
wl_diag_cap(uint64_t cfg, unsigned id, bool ext, uint32_t *flags)
{
	unsigned p = ext ? 0x100 : wl_prd(cfg + 0x34) & 255;
	for (unsigned n = 0; p && n < 48; n++) {
		if ((p & 3) || p < (ext ? 0x100u : 0x40u) || p > (ext ? 0xffcu : 0xfcu)) { break; }
		uint32_t v = wl_prd(cfg + p);
		if (v == ~0u) { break; }
		if (!v) { return 0; }
		if ((v & (ext ? 0xffffu : 255u)) == id) {
			unsigned bytes = ext ? 0x34 : id == 0x10 ? 0x14 : 8;
			if (p + bytes > (ext ? 0x1000u : 0x100u)) { break; }
			return p;
		}
		p = ext ? (v >> 20) & 0xfff : (v >> 8) & 255;
		if (!p) { return 0; }
	}
	if (p) { *flags |= WL_DIAG_CAP_INVALID; }
	return 0;
}

static uint32_t
wl_bhi_diag(unsigned sample)
{
	uint32_t *r = wl5_report, f = 0;
	wl5_status(22); /* raw STATUS, ERRCODE, ERRDBG1..3, EE, MHI, EP cmd */
	r[108] = 0x4d366500; /* M6e extension, r[109..123] is latest sample */
	r[110] = wl_prd(WL_PARF + 0x1b0); /* LTSSM */
	r[111] = wl_prd(WL_DBI + 0xf28); /* ELBI_SYS_STTS (ELBI + 8) */
	unsigned cap = wl_diag_cap(WL_DBI, 0x10, false, &f);
	r[112] = cap ? wl_prd(WL_DBI + cap + 0x10) : ~0u;
	r[113] = wl_prd(WL_DBI + 4);
	r[114] = wl_prd(WL_EPCFG);
	r[115] = wl_prd(WL_EPCFG + 0x10); r[116] = wl_prd(WL_EPCFG + 0x14);
	r[117] = r[118] = r[119] = r[120] = r[121] = r[122] = ~0u;
	if (r[114] != ~0u && r[114]) {
		cap = wl_diag_cap(WL_EPCFG, 1, false, &f);
		if (cap) { r[117] = wl_prd(WL_EPCFG + cap + 4); }
		cap = wl_diag_cap(WL_EPCFG, 1, true, &f);
		if (cap) {
			r[121] = wl_prd(WL_EPCFG + cap + 4);
			r[122] = wl_prd(WL_EPCFG + cap + 0x10);
		}
	}
	cap = wl_diag_cap(WL_DBI, 1, true, &f);
	if (cap) {
		r[118] = wl_prd(WL_DBI + cap + 4); r[119] = wl_prd(WL_DBI + cap + 0x10);
		r[120] = wl_prd(WL_DBI + cap + 0x30);
	}
	r[123] = wl_prd(TS_TLMM + 0x1000ull * 94 + 4);
	uint32_t len = wl_mrd(0), cfg = wl_mrd(0x10);
	if (r[67] == ~0u) { f |= WL_DIAG_STATUS_ONES; }
	if (len == ~0u || cfg == ~0u || r[64] == ~0u) { f |= WL_DIAG_BAR_LOST; }
	if (r[114] != 0x110717cb) { f |= WL_DIAG_CONFIG_LOST; }
	if (r[110] == ~0u || r[111] == ~0u || !(r[111] & 0x400) ||
	    r[112] == ~0u || !(r[112] & (1u << 29))) { f |= WL_DIAG_LINK_LOST; }
	if (r[73] == ~0u || !(r[73] & 2) || (r[115] & ~15u) != WL_BAR0 || r[116]) { f |= WL_DIAG_DECODE_CHANGED; }
	if (r[123] == ~0u || !(r[123] & 2)) { f |= WL_DIAG_PERST; }
	if (r[117] != ~0u && (r[117] & 3)) { f |= WL_DIAG_NON_D0; }
	if (fw_faults) { f |= WL_DIAG_MMIO_FAULT; }
	/* All-ones STATUS alone is ambiguous. Require independent transport and
	 * PBL/error evidence before calling it a device-reported error. */
	if (!(f & WL_DIAG_TRANSPORT) && r[64] == 0 && (r[67] >> 30) == 3 && r[68] && r[68] != ~0u) { f |= WL_DIAG_PBL_ERROR; }
	r[109] = f;
	printf("FUXI WLAN: M6e BHI sample %u STATUS raw %#x field %u ERRCODE %#x ERRDBG %#x/%#x/%#x EXECENV %#x REGLEN %#x MHICFG %#x flags %#x: %s\n",
	    sample, r[67], r[67] >> 30, r[68], r[69], r[70], r[71], r[64], len, cfg, f,
	    (f & WL_DIAG_TRANSPORT) ? "transport inaccessible/changed" : (f & WL_DIAG_PBL_ERROR) ? "PBL error evidence" : "transport readable; BHI result unclassified");
	printf("FUXI WLAN: M6e link sample %u PARF_LTSSM %#x ELBI_SYS_STTS %#x RC_LNKCTLSTA %#x RCcmd/status %#x EPid %#x EPcmd/status %#x BAR0 %#x/%#x PMCSR %#x PERST_IO %#x\n",
	    sample, r[110], r[111], r[112], r[113], r[114], r[73], r[115], r[116], r[117], r[123]);
	printf("FUXI WLAN: M6e AER sample %u RC uncor/cor/root %#x/%#x/%#x EP uncor/cor %#x/%#x (absent/unreadable = all-ones)\n",
	    sample, r[118], r[119], r[120], r[121], r[122]);
	return f;
}

/* ath12k disables ASPM during download. Preserve Link Status W1C bits by
 * writing only the low control half. M6m additionally tests disabling all
 * four L1SS enables (PCI-PM as well as ASPM), without PHY workarounds. */
static int
wl_download_aspm(void)
{
	int retain = 0;
	(void)PE_parse_boot_argn("fuxiwlanaspm", &retain, sizeof(retain));
	if (wl_pre == 5 && (retain == 1 || fw_faults ||
	    wl_prd(WL_EPCFG) != 0x110717cb || (wl_prd(WL_EPCFG + 4) & 4))) { return 3; }
	const uint64_t cfg[] = {WL_EPCFG, WL_DBI};
	for (unsigned i = 0; i < 2; i++) {
		uint32_t flags = 0;
		unsigned cap = wl_diag_cap(cfg[i], 0x10, false, &flags);
		unsigned l1ss = wl_diag_cap(cfg[i], 0x1e, true, &flags);
		if (flags || fw_faults || !cap) { return 1; }
		uint32_t old = wl_prd(cfg[i] + cap + 0x10) & 0xffff;
		uint32_t want = retain == 1 ? old : old & ~3u;
		if (old != want) { wl_pwr(cfg[i] + cap + 0x10, want); }
		uint32_t now = wl_prd(cfg[i] + cap + 0x10) & 0xffff;
		printf("FUXI WLAN: M6k %s ASPM LNKCTL %#x -> %#x L1SS_CTL1 %#x retained %u\n",
		    i ? "RC" : "EP", old, now, l1ss ? wl_prd(cfg[i] + l1ss + 8) : 0, retain == 1);
		if (fw_faults || now != want) { return 2; }
		if (wl_pre == 5) {
			if (!l1ss) {
				printf("FUXI WLAN: M6m %s L1SS absent\n", i ? "RC" : "EP");
				continue;
			}
			uint32_t before = wl_prd(cfg[i] + l1ss + 8);
			if (before == ~0u || fw_faults) { return 3; }
			uint32_t disabled = before & ~0xfu;
			if (before != disabled) { wl_pwr(cfg[i] + l1ss + 8, disabled); }
			uint32_t after = wl_prd(cfg[i] + l1ss + 8);
			printf("FUXI WLAN: M6m %s L1SS_CTL1 %#x -> %#x mask 0xf BME off\n",
			    i ? "RC" : "EP", before, after);
			if (after != disabled || fw_faults) { return 3; }
		}
	}
	return 0;
}

/* Linux qcom_pcie_config_sid_1_9_0: CRC8/MSB over big-endian BDF,
 * BDF[31:16] | (SID - first iommu-map SID)[15:8] | NEXT[7:0].
 * PCIe0's DT base is 0x1400, so offsets 0/1 mean SIDs 0x1400/0x1401.
 * RID 0 hashes to 0 and encodes as zero; RID 0x100 hashes to 0x15.
 * Own the whole table only while bypassed and with no other user evident. */
#define WL_RC_ENTRY 0x01000100u
static unsigned
wl_rc_hash(uint16_t rid)
{
	unsigned hash = 0;
	for (unsigned byte = 0; byte < 2; byte++) {
		hash ^= (rid >> (byte ? 0 : 8)) & 255;
		for (unsigned bit = 0; bit < 8; bit++) {
			hash = ((hash << 1) ^ ((hash & 0x80) ? 7 : 0)) & 255;
		}
	}
	return hash;
}

/* M6h upstream DMA audit. Sources and limits: docs/wlan-m6h.md.
 * The DT iATU aperture is 0x1000 bytes: eight unrolled 0x200-byte pairs,
 * inbound at +0x100. Read only within that aperture, including unused slots;
 * do not probe implementation by writing all-ones as Linux detection does.
 * Snapshots never clear PCI status/AER, nor alter route/ATU/clock state. */
#define WL_IB_SLOTS 8u
#define WL_IB(n) (WL_DBI + 0x1100ull + (uint64_t)(n) * 0x200)
#define WL_GCC_PA 0x00100000ull
static const uint32_t wl_fabric_cb[] = {
	0x1003c, 0x10030, 0x6b02c, 0x6b020, 0x6b01c, 0x1004c, 0x10028
};

static void
wl_host_diag(const char *phase)
{
	uint32_t flags = 0, cs = wl_prd(WL_DBI + 4), sec = wl_prd(WL_DBI + 0x1c) >> 16;
	unsigned aer = wl_diag_cap(WL_DBI, 1, true, &flags);
	printf("FUXI WLAN: M6h %s RC cmd/status %#x BME %u abort STA/RTA/RMA %u/%u/%u SEC %#x abort STA/RTA/RMA %u/%u/%u MEM %#x PREF %#x upper %#x/%#x AER unc/root %#x/%#x capflags %#x AXI_RESP %#x/%#x\n",
	    phase, cs, (cs >> 2) & 1, (cs >> 27) & 1, (cs >> 28) & 1, (cs >> 29) & 1,
	    sec, (sec >> 11) & 1, (sec >> 12) & 1, (sec >> 13) & 1,
	    wl_prd(WL_DBI + 0x20), wl_prd(WL_DBI + 0x24), wl_prd(WL_DBI + 0x28), wl_prd(WL_DBI + 0x2c),
	    aer ? wl_prd(WL_DBI + aer + 4) : ~0u, aer ? wl_prd(WL_DBI + aer + 0x30) : ~0u,
	    flags, wl_prd(WL_DBI + 0x818), wl_prd(WL_DBI + 0x81c));
	printf("FUXI WLAN: M6h %s PARF SYS %#x PM %#x PHY %#x MHI_CLK %#x BYPASS %u HALT v1/v2 %#x/%#x Q2A %#x LTSSM %#x TYPE %#x NOSNOOP %#x\n",
	    phase, wl_prd(WL_PARF), wl_prd(WL_PARF + 0x20), wl_prd(WL_PARF + 0x40),
	    wl_prd(WL_PARF + 0x174), (wl_prd(WL_PARF + 0x174) >> 4) & 1,
	    wl_prd(WL_PARF + 0x178), wl_prd(WL_PARF + 0x1a8), wl_prd(WL_PARF + 0x1ac),
	    wl_prd(WL_PARF + 0x1b0), wl_prd(WL_PARF + 0x1000), wl_prd(WL_PARF + 0x3d4));
	printf("FUXI WLAN: M6h %s PARF ADDR DBI/SLV v1 %#x/%#x DBI v2 %#x:%08x SLV v2 %#x:%08x ATU %#x:%08x BDF_TRANSLATE %#x\n",
	    phase, wl_prd(WL_PARF + 0x168), wl_prd(WL_PARF + 0x16c),
	    wl_prd(WL_PARF + 0x354), wl_prd(WL_PARF + 0x350),
	    wl_prd(WL_PARF + 0x35c), wl_prd(WL_PARF + 0x358),
	    wl_prd(WL_PARF + 0x638), wl_prd(WL_PARF + 0x634), wl_prd(WL_PARF + 0x24c));
	unsigned hash = wl_rc_hash(0x100), nonzero = 0, mismatch = 0;
	uint32_t entry = 0, root = 0, digest = 2166136261u;
	for (unsigned i = 0; i < 256; i++) {
		uint32_t v = wl_prd(WL_PARF + 0x2000 + 4 * i);
		if (i == hash) { entry = v; }
		if (!i) { root = v; }
		nonzero += !!v;
		mismatch += v != (i == hash ? WL_RC_ENTRY : 0);
		digest = (digest ^ v) * 16777619u;
	}
	printf("FUXI WLAN: M6h %s SID RID 0x100 DTbase 0x1400 cfg %#x offset %#x hash %#x entry %#x BDF %#x SIDoff %#x NEXT %#x root %#x nonzero %u mismatch %u digest %#x\n",
	    phase, wl_prd(WL_PARF + 0x2c00), wl_prd(WL_PARF + 0x234), hash, entry,
	    entry >> 16, (entry >> 8) & 255, entry & 255, root, nonzero, mismatch, digest);
	uint32_t ib[WL_IB_SLOTS], enabled = 0;
	for (unsigned i = 0; i < WL_IB_SLOTS; i++) {
		ib[i] = wl_prd(WL_IB(i) + 4);
		if (ib[i] & (1u << 31)) { enabled |= 1u << i; }
	}
	printf("FUXI WLAN: M6h %s iATU IB slots 0..7 enabled %#x ctrl2 %#x/%#x/%#x/%#x/%#x/%#x/%#x/%#x IOVA %#llx\n",
	    phase, enabled, ib[0], ib[1], ib[2], ib[3], ib[4], ib[5], ib[6], ib[7], wl_image.iova);
	for (unsigned i = 0; i < WL_IB_SLOTS; i++) {
		if (!(enabled & (1u << i))) { continue; }
		uint64_t p = WL_IB(i);
		printf("FUXI WLAN: M6h %s iATU IB%u ctrl1/2 %#x/%#x BARmatch %u base %#x:%08x limit %#x:%08x target %#x:%08x\n",
		    phase, i, wl_prd(p), ib[i], (ib[i] >> 30) & 1,
		    wl_prd(p + 0x0c), wl_prd(p + 8), wl_prd(p + 0x20), wl_prd(p + 0x10),
		    wl_prd(p + 0x18), wl_prd(p + 0x14));
	}
	uint32_t cb[7], off = 0, ungated = 0;
	for (unsigned i = 0; i < 7; i++) {
		cb[i] = wl_prd(WL_GCC_PA + wl_fabric_cb[i]);
		if (cb[i] & (1u << 31)) { off |= 1u << i; }
		if ((cb[i] & (1u << 31)) && !(cb[i] & 2)) { ungated |= 1u << i; }
	}
	printf("FUXI WLAN: M6h %s GCC vote0/8 %#x/%#x CB aggre/cnoc/master/slave/q2a/ddrss/cfg %#x/%#x/%#x/%#x/%#x/%#x/%#x OFF %#x OFF_no_HWCG %#x\n",
	    phase, wl_prd(WL_GCC_PA + 0x52000), wl_prd(WL_GCC_PA + 0x52008),
	    cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], off, ungated);
	unsigned cap = wl_diag_cap(WL_DBI, 0x10, false, &flags);
	uint32_t link = cap ? wl_prd(WL_DBI + cap + 0x10) : 0;
	unsigned speed = (link >> 16) & 15, width = (link >> 20) & 63;
	unsigned peak = speed == 1 ? 250000 : speed == 2 ? 500000 : speed == 3 ? 984500 : 0;
	/* No BCM register reads: cmd-db gives RPMh command addresses, not MMIO.
	 * Firmware/other clients' aggregated votes cannot be inferred from GCC. */
	printf("FUXI WLAN: M6h %s ICC XNU_batches=%u Linux pcie-mem init 0/250000 link 0/%u kBps cpu-pcie 0/1 kBps Gen%u x%u BCM SN7/SH1/SH0/MC0/ACV/CN0 firmware_votes=unknown faults %u\n",
	    phase, wl_icc_acks, peak * width, speed, width, fw_faults);
}

/* SM8550 gcc clock drivers: existing M3 votes cover master/slave/Q2A,
 * aggregation and DDRSS; missing CNOC SF is vote8 bit6, ANOC config is
 * vote0 bit20. Add only those shared votes; never remove another client's
 * vote. Exactly 1 opts in. No PARF recipe from a different IP revision. */
static int
wl_fabric_prepare(void)
{
	int clocks = 0, iboff = 0;
	(void)PE_parse_boot_argn("fuxiwlanclk", &clocks, sizeof(clocks));
	(void)PE_parse_boot_argn("fuxiwlaniboff", &iboff, sizeof(iboff));
	printf("FUXI WLAN: M6h fixes fuxiwlanclk=%d fuxiwlaniboff=%d (only 1 enables)\n", clocks, iboff);
	if (fw_faults || !wl_route_installed || (wl_prd(WL_EPCFG + 4) & 4)) { return 1; }
	if (clocks == 1) {
		uint32_t v0 = wl_prd(WL_GCC_PA + 0x52000), v8 = wl_prd(WL_GCC_PA + 0x52008);
		wl_pwr(WL_GCC_PA + 0x52000, v0 | (1u << 20));
		wl_pwr(WL_GCC_PA + 0x52008, v8 | (1u << 6));
		uint32_t a0 = wl_prd(WL_GCC_PA + 0x52000), a8 = wl_prd(WL_GCC_PA + 0x52008);
		printf("FUXI WLAN: M6h clock votes %#x/%#x -> %#x/%#x retained for boot\n", v0, v8, a0, a8);
		if (fw_faults || !(a0 & (1u << 20)) || !(a8 & (1u << 6))) { return 2; }
		for (unsigned i = 0; i < 2; i++) {
			uint32_t off = i ? 0x10030 : 0x10028, v = 0;
			unsigned n;
			for (n = 0; n < 500; n++) {
				v = wl_prd(WL_GCC_PA + off);
				if (fw_faults || !(v & (1u << 31)) || (v & 2)) { break; }
				fw_delay(1);
			}
			printf("FUXI WLAN: M6h clock CB %#x value %#x polls %u\n", off, v, n);
			if (fw_faults || n == 500) { return 3; }
		}
	}
	if (iboff == 1) {
		for (unsigned i = 0; i < WL_IB_SLOTS; i++) {
			uint64_t p = WL_IB(i) + 4;
			uint32_t v = wl_prd(p);
			if (fw_faults) { return 4; }
			if (!(v & (1u << 31))) { continue; }
			wl_pwr(p, v & ~(1u << 31));
			uint32_t after = wl_prd(p);
			printf("FUXI WLAN: M6h iATU disable IB%u ctrl2 %#x -> %#x\n", i, v, after);
			if (fw_faults || after != (v & ~(1u << 31))) { return 4; }
		}
	}
	return !!fw_faults;
}

/* Read-only capability walk: never disable an existing user to claim a table. */
static int
wl_rc_irqs_busy(uint64_t cfg)
{
	unsigned p = wl_prd(cfg + 0x34) & 255;
	for (unsigned i = 0; p && i < 48; i++) {
		if (p < 0x40 || p > 0xfc || (p & 3)) { return 1; }
		uint32_t cap = wl_prd(cfg + p);
		if (((cap & 255) == 5 && (cap & (1u << 16))) ||
		    ((cap & 255) == 0x11 && (cap & (1u << 31)))) { return 1; }
		p = (cap >> 8) & 255;
	}
	return p || fw_faults;
}

static int
wl_rc_fail(unsigned reason, const char *name)
{
	wl5_report[25] = reason;
	printf("FUXI WLAN: RC route refused: %s (route reason %u; step 17 reason 5)\n", name, reason);
	return (int)reason;
}

static int
wl_rc_check(void)
{
	if (!wl_route_installed || !wl_route_owned ||
	    wl_prd(WL_PARF + 0x2c00) != (wl_route_cfg & ~1u) || wl_prd(WL_PARF + 0x234)) { return 1; }
	for (unsigned i = 0; i < 256; i++) {
		if (wl_prd(WL_PARF + 0x2000 + 4 * i) != (i == wl_route_hash ? WL_RC_ENTRY : 0)) { return 1; }
	}
	return !!fw_faults;
}

static int
wl_rc_route(void)
{
	wl_route_cfg = wl_prd(WL_PARF + 0x2c00);
	uint32_t offset = wl_prd(WL_PARF + 0x234);
	wl_route_hash = wl_rc_hash(0x100);
	uint32_t entry = wl_prd(WL_PARF + 0x2000 + 4 * wl_route_hash);
	wl5_report[22] = wl_route_cfg; wl5_report[23] = offset; wl5_report[24] = entry;
	printf("FUXI WLAN: RC RID 0x100 SID cfg %#x offset %#x hash %#x entry %#x\n",
	    wl_route_cfg, offset, wl_route_hash, entry);
	if (fw_faults || offset || wl_route_cfg == ~0u) { return wl_rc_fail(1, "RC config/offset inaccessible or unsupported"); }
	if (!(wl_route_cfg & 1)) { return wl_rc_fail(2, "RC table in use (bypass clear)"); }
	if (wl_rc_irqs_busy(WL_DBI) || wl_rc_irqs_busy(WL_EPCFG) || (wl_prd(WL_EPCFG + 4) & 4)) {
		return wl_rc_fail(3, "MSI/MSI-X or endpoint bus master active, or capability chain unproven");
	}
	/* DWC supports up to eight blocks of 32 MSI vectors. All must be idle. */
	for (unsigned i = 0; i < 8; i++) {
		if (wl_prd(WL_DBI + 0x828 + 12 * i)) { return wl_rc_fail(3, "RC MSI vectors enabled"); }
	}
	if (fw_faults || wl_prd(WL_PARF + 0x2c00) != wl_route_cfg) { return wl_rc_fail(1, "RC config changed before claim"); }
	wl_route_owned = true; /* includes partial scrub/install; never restore garbage */
	for (unsigned i = 0; i < 256; i++) { wl_pwr(WL_PARF + 0x2000 + 4 * i, 0); }
	for (unsigned i = 0; i < 256; i++) {
		if (wl_prd(WL_PARF + 0x2000 + 4 * i) || fw_faults) { return wl_rc_fail(4, "RC table zero readback"); }
	}
	printf("FUXI WLAN: M6b RC bypassed table zeroed 256 entries, readback verified; MSI idle\n");
	/* Root RID 0 -> offset 0 is already installed by the verified zero at slot 0. */
	wl_pwr(WL_PARF + 0x2000 + 4 * wl_route_hash, WL_RC_ENTRY);
	if (wl_prd(WL_PARF + 0x2000 + 4 * wl_route_hash) != WL_RC_ENTRY ||
	    wl_prd(WL_PARF + 0x2000) || fw_faults) { return wl_rc_fail(5, "RC mapping readback"); }
	/* Enable lookup last, preserving every other CFG bit. */
	wl_pwr(WL_PARF + 0x2c00, wl_route_cfg & ~1u);
	if (wl_prd(WL_PARF + 0x2c00) != (wl_route_cfg & ~1u) || fw_faults) { return wl_rc_fail(5, "RC bypass clear readback"); }
	wl_route_installed = true;
	printf("FUXI WLAN: M6b RC route installed: RID 0 -> DT SID 0x1400 offset 0 slot 0; RID 0x100 -> DT SID 0x1401 offset 1 slot %#x entry %#x; bypass clear last\n", wl_route_hash, WL_RC_ENTRY);
	return 0;
}

static uint64_t
wl_cb_addr(unsigned cb)
{
	return wl_cb_base + ((uint64_t)cb << wl_pgshift);
}

/* Strictly read-only selection. Reserve CBs referenced by ANY S2CR, even an
 * invalid SMR or non-translating S2CR: conservative with firmware leftovers. */
static int
wl_smmu_probe(void)
{
	uint32_t *r = wl5_report;
	wl_smmu_mismatches = 0;
	r[4] = wl_prd(WL_SMMU); r[5] = wl_prd(WL_SMMU + 0x20);
	r[6] = wl_prd(WL_SMMU + 0x24); r[7] = wl_prd(WL_SMMU + 0x28);
	wl_smr_count = r[5] & 255; wl_cb_count = r[6] & 255;
	wl_pgshift = (r[6] & (1u << 31)) ? 16 : 12;
	unsigned pages = 1u << (((r[6] >> 28) & 7) + 1);
	wl_gr1 = WL_SMMU + (1u << wl_pgshift);
	wl_cb_base = WL_SMMU + ((uint64_t)pages << wl_pgshift);
	printf("FUXI WLAN: M6 apps_smmu sCR0 %#x IDR0 %#x IDR1 %#x IDR2 %#x NUMSMRG %u NUMCB %u CBbase %#llx\n",
	    r[4], r[5], r[6], r[7], wl_smr_count, wl_cb_count, wl_cb_base);
	/* No EXID, global reconfiguration or guesses about unsupported formats. */
	if (fw_faults || r[4] == ~0u || r[5] == ~0u || r[6] == ~0u || r[7] == ~0u ||
	    (r[4] & 9) || (r[5] & 0x48000000) != 0x48000000 ||
	    !(r[7] & 0x1000) || !(r[7] & 15) || (r[7] & 15) > 5 ||
	    !((r[7] >> 4) & 15) || ((r[7] >> 4) & 15) > 5 ||
	    !wl_smr_count || !wl_cb_count || wl_cb_count > 128 ||
	    ((r[6] >> 16) & 255) > wl_cb_count || ((r[6] >> 24) & 3) == 2 || ((r[7] >> 8) & 15) > 5 ||
	    wl_cb_base + ((uint64_t)wl_cb_count << wl_pgshift) > WL_SMMU + 0x100000) { return 1; }
	bool referenced[128] = {false};
	wl_slot = wl_smr_count; wl_cb = wl_cb_count;
	for (unsigned i = 0; i < wl_smr_count; i++) {
		uint32_t smr = wl_prd(WL_SMMU + 0x800 + 4 * i);
		uint32_t s2cr = wl_prd(WL_SMMU + 0xc00 + 4 * i);
		if (smr == ~0u || s2cr == ~0u || fw_faults) { return 1; }
		if ((s2cr & 255) < wl_cb_count) { referenced[s2cr & 255] = true; }
		if (smr & 0x80000000u) {
			if (((0x1401 ^ smr) & ~((smr >> 16) & 0x7fff) & 0xffff) == 0) {
				printf("FUXI WLAN: SMMU endpoint collision slot %u SMR %#x S2CR %#x\n", i, smr, s2cr); return 2;
			}
		} else if (wl_slot == wl_smr_count && ((s2cr >> 16) & 3) >= 1 && ((s2cr >> 16) & 3) <= 2) {
			wl_slot = i; wl_old_smr = smr; wl_old_s2cr = s2cr;
		}
	}
	unsigned s2only = (r[6] >> 16) & 255;
	for (unsigned i = 0; i < wl_cb_count; i++) {
		uint32_t ctl = wl_prd(wl_cb_addr(i));
		if (ctl == ~0u) { return 3; }
		if (i >= s2only && !referenced[i] && !(ctl & 1) && wl_cb == wl_cb_count) {
			wl_cb = i; wl_old_sctlr = ctl;
		}
	}
	if (fw_faults || wl_slot == wl_smr_count || wl_cb == wl_cb_count) { return 3; }
	/* CB index is also the unique ASID, as in the generic arm-smmu driver.
	 * Refuse firmware configurations reusing this ASID in any enabled bank. */
	for (unsigned i = 0; i < wl_cb_count; i++) {
		if (wl_prd(wl_cb_addr(i)) & 1) {
			if ((wl_prd(wl_cb_addr(i) + 0x24) >> 16) == wl_cb ||
			    (wl_prd(wl_cb_addr(i) + 0x2c) >> 16) == wl_cb) { return 3; }
		}
	}
	if (fw_faults || (wl_prd(WL_EPCFG + 4) & 4)) { return 4; }
	printf("FUXI WLAN: SMMU free SMR %u old %#x/%#x CB %u SCTLR %#x unreferenced; SID 0x1401 mask 0\n",
	    wl_slot, wl_old_smr, wl_old_s2cr, wl_cb, wl_old_sctlr);
	wl5_report[8] = wl_slot; wl5_report[9] = wl_cb;
	return wl_rc_route() ? 5 : 0;
}

/* SMMUv2 IHI0062D.c 10.2 / 16.5: masks for our Non-secure AArch64
 * stage-1 format. Ignore undefined/RO fields, not failures of writable bits.
 * MMU-500 integrations can hardwire TCR2.TBI[1:0] to one (0x60).
 * SCTLR.AFE/TRE are UNK/SBOP in long-descriptor formats. ASIDPNE is an
 * optional hint (MMU-500 integrations can read zero); unique ASIDs are checked
 * across all enabled banks, so correctness never relies on that hint.
 * ACTLR is untouched. */
#define WL_CB_SCTLR_MASK 0x7fdfc7f9u /* no reserved [31,21,13,11], ASIDPNE, AFE/TRE */
#define WL_CB_TCR_MASK   0xffffffbfu /* AArch64: only bit 6 reserved */
#define WL_CB_TTBR_LO_MASK 0xffffffe0u /* TnSZ=32, 4-KiB granule: x=5 */
#define WL_CB_MAIR_MASK  0xffffffffu /* all eight attribute bytes are writable */

static uint32_t
wl_tcr2_mask(void)
{
	uint32_t mask = 0x38017; /* SEP, AS, PASize[2:0]; no TBI or Secure NSCFG */
	if (wl5_report[7] & (1u << 28)) { mask |= 0x300; } /* HADS */
	unsigned hafdbs = (wl5_report[6] >> 24) & 3;
	if (hafdbs & 1) { mask |= 0x400; } /* HA */
	if (hafdbs == 3) { mask |= 0x800; } /* HD */
	return mask;
}

static uint32_t
wl_ttbr_hi_mask(void)
{
	static const unsigned bits[] = {32, 36, 40, 42, 44, 48};
	unsigned ias = wl5_report[7] & 15, oas = (wl5_report[7] >> 4) & 15;
	/* Probe already bounded both encodings; S1/S2-bypass uses the smaller. */
	return 0xffff0000u | (uint32_t)(((1ull << bits[MIN(ias, oas)]) - 1) >> 32);
}

/* Offsets include GR1/CB page selection. Log the complete raw read even on
 * success, so ignored bits remain visible. W1C uses its own expected value;
 * command registers use completion polling, never equality-to-write checks. */
static int
wl_smmu_verify(const char *name, uint64_t pa, uint32_t written,
    uint32_t readback, uint32_t expected, uint32_t mask)
{
	bool mismatch = fw_faults || (readback & mask) != (expected & mask);
	if (!mismatch || wl_smmu_mismatches++ < 16) {
		printf("FUXI WLAN: M6d SMMU %s CB %u %s offset %#x wrote %#x read %#x mask %#x faults %u\n",
		    mismatch ? "mismatch" : "readback", wl_cb, name,
		    (unsigned)(pa - WL_SMMU), written, readback, mask, fw_faults);
	}
	return mismatch;
}

static int
wl_smmu_wr(const char *name, uint64_t pa, uint32_t v, uint32_t mask)
{
	wl_pwr(pa, v);
	uint32_t readback = wl_prd(pa);
	return wl_smmu_verify(name, pa, v, readback, v, mask);
}

/* Return 0 only for a presently disabled, unreferenced stage-1 candidate with
 * an unused ASID. 1 skips that bank; -1 aborts on shared ownership/access change.
 * Re-read the entire stream table before EVERY probe, not just the saved slot.
 * This driver never performs Linux's destructive global reset/quirk probes. */
static int
wl_smmu_candidate(unsigned candidate, uint32_t *ctl)
{
	if (fw_faults || wl_prd(WL_SMMU) != wl5_report[4] || wl_rc_check() ||
	    (wl_prd(WL_EPCFG + 4) & 4) ||
	    wl_prd(WL_SMMU + 0x800 + 4 * wl_slot) != wl_old_smr ||
	    wl_prd(WL_SMMU + 0xc00 + 4 * wl_slot) != wl_old_s2cr) { return -1; }
	bool referenced = false, asid_used = false;
	for (unsigned i = 0; i < wl_smr_count; i++) {
		uint32_t smr = wl_prd(WL_SMMU + 0x800 + 4 * i);
		uint32_t s2cr = wl_prd(WL_SMMU + 0xc00 + 4 * i);
		if (fw_faults || smr == ~0u || s2cr == ~0u ||
		    ((smr & 0x80000000u) &&
		    (((smr ^ 0x1401) & ~((smr >> 16) & 0x7fff) & 0xffff) == 0))) { return -1; }
		if ((s2cr & 255) == candidate) { referenced = true; }
	}
	*ctl = wl_prd(wl_cb_addr(candidate));
	for (unsigned i = 0; i < wl_cb_count; i++) {
		if (wl_prd(wl_cb_addr(i)) & 1) {
			if ((wl_prd(wl_cb_addr(i) + 0x24) >> 16) == candidate ||
			    (wl_prd(wl_cb_addr(i) + 0x2c) >> 16) == candidate) { asid_used = true; }
		}
	}
	if (fw_faults) { return -1; }
	return referenced || asid_used || *ctl == ~0u || (*ctl & 1) ||
	       candidate < ((wl5_report[6] >> 16) & 255) || wl_cb_unusable[candidate];
}

/* Zero TCR2 is a legal, non-enabling write, independent of ASID/TTBR format.
 * CONTEXTIDR does not hold the ASID in long-descriptor formats; choose
 * the configuration register we must program anyway.
 * A masked-zero readback is only a filter (RAZ/WI could pass); all configuration and
 * GR1 readbacks remain mandatory. Never retry a failure AFTER claiming a CB. */
static int
wl_smmu_claim_candidate(void)
{
	unsigned attempts = 0;
	for (unsigned i = wl_cb; i < wl_cb_count && attempts < WL_CB_PROBE_MAX; i++) {
		uint32_t ctl;
		int state = wl_smmu_candidate(i, &ctl);
		if (state < 0) { return 3; }
		if (state) { continue; }
		wl_cb = i;
		printf("FUXI WLAN: M6d CB probe %u/%u CB %u SCTLR %#x M=0 unreferenced; TCR2 zero\n",
		    ++attempts, WL_CB_PROBE_MAX, i, ctl);
		if (wl_smmu_wr("CB_TCR2 probe", wl_cb_addr(i) + 0x10, 0, wl_tcr2_mask())) {
			wl_cb_unusable[i] = true;
			printf("FUXI WLAN: M6d CB %u unusable; probe failed%s\n", i, fw_faults ? "; MMIO fault, abort" : "; next candidate");
			if (fw_faults) { return 6; }
			continue;
		}
		uint32_t now;
		if (wl_smmu_candidate(i, &now) || now != ctl) { return 3; }
		wl_old_sctlr = ctl;
		wl5_report[9] = i;
		printf("FUXI WLAN: M6d CB %u probe accepted; claim pending full CB/GR1 readbacks\n", i);
		return 0;
	}
	printf("FUXI WLAN: M6d CB probe exhausted %u candidates (limit %u); no bank claimed\n", attempts, WL_CB_PROBE_MAX);
	return 6;
}

static int
wl_smmu_sync(bool cleanup)
{
	uint64_t cb = wl_cb_addr(wl_cb);
	if (cleanup) {
		if (wl_contain_wr(cb + 0x610, wl_cb) || wl_contain_wr(cb + 0x7f0, 0)) { return 1; }
	} else {
		wl_pwr(cb + 0x610, wl_cb); wl_pwr(cb + 0x7f0, 0);
	}
	__asm__ volatile("dsb sy" ::: "memory");
	for (unsigned i = 0; i <= 100; i++) {
		uint32_t v = wl_prd(cb + 0x7f4);
		if (v == ~0u || (!cleanup && fw_faults)) { return 1; }
		if (!(v & 1)) { return 0; }
		if (i < 100) { wl_sleep(1); }
	}
	printf("FUXI WLAN: SMMU CB %u TLB sync timeout 100 ms\n", wl_cb);
	return 1;
}

/* 4-KiB AArch64 stage-1 tables: L1 root, L2, one L3 per 2 MiB of the pool.
 * Only allocated DMA buffers get leaves; page tables never get DMA mappings.
 * Both selectable pools fit within one L1 entry and are 2-MiB aligned. */
static uint64_t
wl_pt_pa(unsigned table)
{
	return wl_phys(wl_tables.va + table * 4096);
}

/* Reverse PA lookup is restricted to our wired allocations. Never dereference
 * an untrusted table PA via phystokv or invent physical contiguity at 16 KiB. */
static vm_offset_t
wl_owned_alias(uint64_t pa, unsigned bytes, bool tables_only)
{
	struct wl_dma *bufs[] = { &wl_tables, &wl_image, &wl_vectors, &wl_contexts, &wl_rddm, &wl_rxvectors, &wl_msi_sink, &wl_amss, &wl_ipcr, &wl14_regdb, &wl14_bdf, &wl14_m3, &wl_ce_arena, &wl_rx_arena, &wl_link_arena, &wl_reo_arena, &wl_mon_arena, &wl_cm_arena };
	unsigned fixed = sizeof(bufs)/sizeof(bufs[0]);
	for (unsigned i = 0; i < (tables_only ? 1u : fixed + WL_TARGET_MAX); i++) {
		struct wl_dma *d = i < fixed ? bufs[i] : &wl_target[i-fixed];
		for (vm_size_t off = 0; d->va && off < d->size; off += PAGE_SIZE) {
			uint64_t base = wl_phys(d->va + off);
			if (pa >= base && pa - base < PAGE_SIZE && bytes <= PAGE_SIZE - (pa - base)) {
				return d->va + off + (vm_offset_t)(pa - base);
			}
		}
	}
	return 0;
}

static uint64_t
wl_hash_bytes(uint64_t h, const uint8_t *p, unsigned bytes)
{
	for (unsigned i = 0; i < bytes; i++) { h = (h ^ p[i]) * 0x100000001b3ull; }
	return h;
}

static int
wl_bhi_mapping_check(void)
{
	const uint64_t mask = 0x0000000ffffff000ull, seed = 0xcbf29ce484222325ull;
	unsigned bytes = wl_bhi_bytes();
	uint64_t source = wl_hash_bytes(seed, (const uint8_t *)wl_image.va, bytes), mapped = seed;
	/* flush_dcache64 is clean+invalidate to PoC on this ARM64 target.
	 * Force the physical-page hash to reload after the source hash. */
	wl_dma_clean(&wl_image); wl_dma_clean(&wl_tables);
	uint64_t root = ((uint64_t)wl_prd(wl_cb_addr(wl_cb) + 0x24) << 32) | wl_prd(wl_cb_addr(wl_cb) + 0x20);
	unsigned pages = 0, bad = 0;
	/* Walk the live CB TTBR, not fixed virtual indexes used by the builder. */
	if ((root & mask) != wl_pt_pa(0) || fw_faults) { bad = 1; }
	for (unsigned off = 0; !bad && off < bytes; off += 4096) {
		uint64_t iova = wl_image.iova + off, table = root & mask, leaf = 0;
		const unsigned shifts[] = {30, 21, 12};
		for (unsigned level = 0; level < 3; level++) {
			vm_offset_t alias = wl_owned_alias(table, 4096, true);
			if (!alias) { bad = 2; break; }
			uint64_t desc = ((const uint64_t *)alias)[(iova >> shifts[level]) & 511];
			if (level == 2 && wl_probe == WL_PROBE_UNMAPPED) {
				if (desc) { bad = 6; }
				break;
			}
			if ((desc & 3) != 3) { bad = 3; break; }
			table = desc & mask;
			if (level == 2) { leaf = table; }
		}
		if (bad) { break; }
		if (wl_probe == WL_PROBE_UNMAPPED) { pages++; continue; }
		vm_offset_t alias = wl_owned_alias(leaf, 4096, false);
		if (!alias) { bad = 4; break; }
		mapped = wl_hash_bytes(mapped, (const uint8_t *)alias, MIN(4096, bytes - off)); pages++;
		if (leaf != wl_phys(wl_image.va + off)) { bad = 5; }
	}
	int e = bad || pages != (bytes + 4095) / 4096 || (wl_probe != WL_PROBE_UNMAPPED && source != mapped);
	if (wl_probe == WL_PROBE_UNMAPPED) {
		printf("FUXI WLAN: M6k hole IOVA %#llx+%#x invalid leaves %u/128 %s reason %u (live CB walk)\n",
		    wl_image.iova, WL_CHUNK, pages, e ? "FAIL" : "VERIFIED", bad);
		return e;
	}
	printf("FUXI WLAN: M6f CB %u BHI IOVA %#llx+%#x FNV1a64 source %016llx walk %016llx pages %u/%u %s reason %u (CPU view)\n",
	    wl_cb, wl_image.iova, bytes, source, mapped, pages, (bytes + 4095) / 4096, e ? "FAIL" : "MATCH", bad);
	return e;
}

static int
wl_smmu_populate(void)
{
	unsigned l3_count = (unsigned)((wl_iova_end - wl_iova_base) >> 21);
	uint64_t *root = (uint64_t *)wl_tables.va;
	uint64_t *l2 = (uint64_t *)(wl_tables.va + 4096);
	root[wl_iova_base >> 30] = wl_pt_pa(1) | 3;
	for (unsigned i = 0; i < l3_count; i++) {
		l2[((wl_iova_base >> 21) & 511) + i] = wl_pt_pa(2 + i) | 3;
	}
	struct wl_dma *bufs[] = { &wl_image, &wl_vectors, &wl_contexts, &wl_rddm, &wl_rxvectors, &wl_msi_sink, &wl_amss, &wl_ipcr };
	for (unsigned i = 0; i < sizeof(bufs)/sizeof(bufs[0]); i++) {
		struct wl_dma *d = bufs[i];
		if (d == &wl_image && wl_probe == WL_PROBE_UNMAPPED) { continue; }
		for (vm_size_t off = 0; off < d->size; off += 4096) {
			uint64_t addr = d->iova + off;
			if (addr < wl_iova_base || addr >= wl_iova_end) { return 2; }
			unsigned n = (unsigned)((addr - wl_iova_base) >> 21);
			uint64_t *l3 = (uint64_t *)(wl_tables.va + (2 + n) * 4096);
			l3[(addr >> 12) & 511] = wl_phys(d->va + off) | WL_DMA_PTE;
		}
	}
	wl_dma_clean(&wl_tables);
	return fw_faults ? 1 : 0;
}

static int
wl_smmu_enable(void)
{
	if (wl_rc_check()) { return 3; }
	if (wl_enabled) {
		/* M6o installed a fault-only/sink mapping before MASTER. Add only
		 * owned buffers, publish leaves, invalidate cached faults, then prepare. */
		if (wl_smmu_fault(20) || fw_faults ||
		    wl_prd(WL_SMMU+0x800+4*wl_slot) != 0x80001401 ||
		    wl_prd(WL_SMMU+0xc00+4*wl_slot) != wl_cb ||
		    (wl_prd(wl_cb_addr(wl_cb))&WL_CB_SCTLR_MASK) != (WL_CB_SCTLR&WL_CB_SCTLR_MASK)) { return 5; }
		return wl_smmu_populate() || wl_smmu_sync(false) || wl_smmu_fault(20) ? 5 : 0;
	}
	unsigned l3_count = (unsigned)((wl_iova_end - wl_iova_base) >> 21);
	if (wl_dma_alloc(&wl_tables, (2 + l3_count) * 4096)) { return 1; }
	if (wl_smmu_populate()) { return 2; }
	/* Re-prove vacancy after allocations/VFS sleeps, immediately before writes. */
	if (wl_prd(WL_SMMU + 0x800 + 4 * wl_slot) != wl_old_smr ||
	    wl_prd(WL_SMMU + 0xc00 + 4 * wl_slot) != wl_old_s2cr ||
	    wl_prd(wl_cb_addr(wl_cb)) != wl_old_sctlr) { return 3; }
	for (unsigned i = 0; i < wl_smr_count; i++) {
		uint32_t smr = wl_prd(WL_SMMU + 0x800 + 4 * i), s2cr = wl_prd(WL_SMMU + 0xc00 + 4 * i);
		if ((s2cr & 255) == wl_cb || ((smr & 0x80000000u) &&
		    (((smr ^ 0x1401) & ~((smr >> 16) & 0x7fff) & 0xffff) == 0))) { return 3; }
	}
	if (fw_faults || (wl_prd(WL_EPCFG + 4) & 4)) { return 3; }
	for (unsigned i = 0; i < wl_cb_count; i++) {
		if (wl_prd(wl_cb_addr(i)) & 1) {
			if ((wl_prd(wl_cb_addr(i) + 0x24) >> 16) == wl_cb ||
			    (wl_prd(wl_cb_addr(i) + 0x2c) >> 16) == wl_cb) { return 3; }
		}
	}
	if (fw_faults || wl_prd(WL_SMMU) != wl5_report[4] || wl_rc_check()) { return 3; }
	int claim = wl_smmu_claim_candidate();
	if (claim) { return claim; }
	uint64_t cb = wl_cb_addr(wl_cb), ttbr = wl_pt_pa(0) | ((uint64_t)wl_cb << 48);
	wl_owned = true; /* includes partial setup: abort must disable and sync */
	/* IRPTNDX is not required storage in v2; VMID16 is RES0 when disabled. */
	bool vmid16 = (wl5_report[4] & (1u << 31)) && (wl5_report[7] & (1u << 15));
	uint32_t cbar_mask = vmid16 ? 0x00ffff00 : 0x00ffffff;
	uint32_t cba2r_mask = vmid16 ? 0xffff0001 : 1; /* MONC ignored Non-secure */
	uint32_t ttbr_hi_mask = wl_ttbr_hi_mask();
	if (wl_smmu_wr("CB_SCTLR disable", cb, 0, WL_CB_SCTLR_MASK) ||
	    wl_smmu_wr("GR1_CBA2R", wl_gr1 + 0x800 + 4 * wl_cb, 1, cba2r_mask) || /* VA64 */
	    wl_smmu_wr("GR1_CBAR", wl_gr1 + 4 * wl_cb, 0x1f300, cbar_mask) || /* S1, S2 bypass */
	    wl_smmu_wr("CB_TCR2", cb + 0x10, 0x38011, wl_tcr2_mask()) || /* SEP upstream, AS, 36-bit PA */
	    wl_smmu_wr("CB_TCR", cb + 0x30, WL_CB_TCR, WL_CB_TCR_MASK) ||
	    wl_smmu_wr("CB_TTBR0_HI", cb + 0x24, (uint32_t)(ttbr >> 32), ttbr_hi_mask) ||
	    wl_smmu_wr("CB_TTBR0_LO", cb + 0x20, (uint32_t)ttbr, WL_CB_TTBR_LO_MASK) ||
	    wl_smmu_wr("CB_TTBR1_HI", cb + 0x2c, wl_cb << 16, ttbr_hi_mask) ||
	    wl_smmu_wr("CB_TTBR1_LO", cb + 0x28, 0, WL_CB_TTBR_LO_MASK) ||
	    wl_smmu_wr("CB_MAIR0", cb + 0x38, 0xff, WL_CB_MAIR_MASK) ||
	    wl_smmu_wr("CB_MAIR1", cb + 0x3c, 0, WL_CB_MAIR_MASK)) { return 4; }
	/* FSR is W1C only for our newly owned bank; preserve later DMA faults. */
	wl_pwr(cb + 0x58, 0xc00001fe);
	if (wl_smmu_verify("CB_FSR W1C", cb + 0x58, 0xc00001fe,
	    wl_prd(cb + 0x58), 0, 0xc00001fe)) { return 5; }
	if (wl_smmu_fault(20) || wl_smmu_sync(false) ||
	    wl_smmu_wr("CB_SCTLR enable", cb, WL_CB_SCTLR, WL_CB_SCTLR_MASK) ||
	    wl_smmu_wr("GR0_S2CR", WL_SMMU + 0xc00 + 4 * wl_slot, wl_cb, ~0u) ||
	    wl_smmu_wr("GR0_SMR", WL_SMMU + 0x800 + 4 * wl_slot, 0x80001401, ~0u)) { return 5; }
	wl_enabled = true;
	printf("FUXI WLAN: SMMU owned stage1 SMR %u CB %u TTBR %#llx IOVA %#llx-%#llx readback verified\n",
	    wl_slot, wl_cb, wl_pt_pa(0), wl_iova_base,
	    wl_iova_next - 1);
	printf("FUXI WLAN: M6g CB %u coherent walk TCR %#x IRGN0=1 ORGN0=1 SH0=3; DMA MAIR0 %#x SH=2; SCTLR %#x HUPCF=1 CFCFG=0 CFRE=0 CFIE=0\n",
	    wl_cb, wl_prd(cb + 0x30), wl_prd(cb + 0x38), wl_prd(cb));
	printf("FUXI WLAN: DMA self-test: no standalone BHI host-write cookie; step 21/22 BHI transfer + CB faults is first endpoint proof\n");
	return 0;
}

static int
wl_smmu_fault(unsigned step)
{
	if (!wl_owned) { return 0; }
	uint64_t cb = wl_cb_addr(wl_cb);
	uint32_t fsr = wl_prd(cb + 0x58), lo = wl_prd(cb + 0x60), hi = wl_prd(cb + 0x64);
	uint32_t syn = wl_prd(cb + 0x68), syn1 = wl_prd(cb + 0x6c);
	uint32_t global = wl_prd(WL_SMMU + 0x48), sid = wl_prd(WL_SMMU + 0x54);
	uint32_t cb_sid = wl_prd(wl_gr1 + 0x400 + 4 * wl_cb); /* CBFRSYNRA */
	wl5_report[80] = fsr; wl5_report[81] = lo; wl5_report[82] = hi;
	wl5_report[83] = syn; wl5_report[84] = syn1;
	wl5_report[85] = global; wl5_report[86] = sid;
	if (step >= 17 && step <= 28) { wl5_report[96 + step - 17] = fsr; }
	int bad = fw_faults || !!(fsr & 0xc00001fe) || ((global & 2) && (sid & 0xffff) == 0x1401);
	/* W11 polls with step 0: retain fault snapshots, log only on a fault. */
	if (step || bad) {
		printf("FUXI WLAN: M6 step %u CB %u FSR %#x FAR %#x:%08x FSYNR %#x/%#x GFSR %#x GFSYNR1 %#x\n", step, wl_cb, fsr, hi, lo, syn, syn1, global, sid);
		printf("FUXI WLAN: M6k step %u CBFRSYNRA %#x SID %#x\n", step, cb_sid, cb_sid & 0xffff);
	}
	return bad;
}

/* Fault-contained cleanup is restricted to this driver's proven vacant slots.
 * BME off + PERST/drain are prerequisites in wl5_abort. Failure quarantines
 * tables and data; never free a table while a walker could still use it. */
static int
wl_smmu_undo(void)
{
	if (wl_owned) {
		uint64_t smr = WL_SMMU + 0x800 + 4 * wl_slot;
		uint64_t s2cr = WL_SMMU + 0xc00 + 4 * wl_slot;
		if (wl_contain_wr(smr, 0x1401) || (wl_prd(smr) & 0x80000000u)) { return 1; }
		if (wl_contain_wr(s2cr, wl_old_s2cr) || wl_prd(s2cr) != wl_old_s2cr) { return 1; }
		uint64_t cb = wl_cb_addr(wl_cb);
		if (wl_contain_wr(cb, 0)) { return 1; }
		unsigned before = fw_faults;
		uint32_t readback = wl_prd(cb);
		printf("FUXI WLAN: M6d SMMU undo CB %u CB_SCTLR offset %#x wrote 0 read %#x mask %#x faults %u\n",
		    wl_cb, (unsigned)(cb - WL_SMMU), readback, WL_CB_SCTLR_MASK, fw_faults);
		if ((readback & WL_CB_SCTLR_MASK) || fw_faults != before) { return 1; }
		if (wl_smmu_sync(true)) { return 1; }
		if (wl_contain_wr(smr, wl_old_smr) || wl_prd(smr) != wl_old_smr) { return 1; }
		wl_enabled = wl_owned = false;
		printf("FUXI WLAN: SMMU undo SMR invalid, S2CR restored, CB disabled, TLBI synced\n");
	}
	if (wl_route_owned) {
		/* Restore bypass before touching any owned entry, including partial setup. */
		if (wl_contain_wr(WL_PARF + 0x2c00, wl_route_cfg) || wl_prd(WL_PARF + 0x2c00) != wl_route_cfg) { return 1; }
		wl_route_installed = false;
		for (unsigned i = 0; i < 256; i++) {
			uint64_t pa = WL_PARF + 0x2000 + 4 * i;
			if (wl_contain_wr(pa, 0) || wl_prd(pa)) { return 1; }
		}
		wl_route_owned = false;
		printf("FUXI WLAN: M6b RC undo bypass restored first, 256 owned entries zeroed\n");
	}
	return 0;
}

/* Disable both PCI message interrupt mechanisms, leaving the endpoint in
 * polling mode even if a previous boot stage left a capability enabled. */
static int
wl_disable_irqs(void)
{
	unsigned p = wl_prd(WL_EPCFG + 0x34) & 0xff;
	for (unsigned i = 0; p && i < 48; i++) {
		if (p < 0x40 || p > 0xfc || (p & 3)) { return 1; }
		uint32_t cap = wl_prd(WL_EPCFG + p), v = cap;
		if ((cap & 255) == 5) { v &= ~(1u << 16); } /* MSI enable */
		if ((cap & 255) == 0x11) { v = (v & ~(1u << 31)) | (1u << 30); } /* MSI-X enable/mask */
		if (v != cap) {
			wl_pwr(WL_EPCFG + p, v);
			if (wl_prd(WL_EPCFG + p) != v) { return 2; }
		}
		p = (cap >> 8) & 255;
	}
	return p || fw_faults ? 3 : 0;
}

/* M6o is original polling glue, using only the cited register constants.
 * Journal only host configuration, never W1C/status, DMA doorbells or BME.
 * Banked equalization entries carry their selector for bounded reverse undo. */
struct wl_seq_undo { uint64_t pa; uint32_t old, mask; unsigned bank; };
#define WL_SEQ_UNDO_MAX 96u
static struct wl_seq_undo wl_seq_undo[WL_SEQ_UNDO_MAX];
static unsigned wl_seq_undos, wl_eq_bank;
static bool wl_cycle_powered;
static uint32_t wl_msi_saved[5];
static bool wl_msi_pvm, wl_msi_64;
/* MSI capability layout: KIWI reports 32-bit addressing with per-vector
 * masking (cap 0x010a7005 on device), so data/mask follow the low address. */
static unsigned wl_msi_data_off(void) { return wl_msi_64 ? 12 : 8; }
static unsigned wl_msi_mask_off(void) { return wl_msi_64 ? 16 : 12; }
static unsigned wl_msi_words(void) { return wl_msi_mask_off() / 4 + (wl_msi_pvm ? 1 : 0); }

static int
wl_m6o_write(const char *what, uint64_t pa, uint32_t value, uint32_t mask)
{
	printf("FUXI WLAN: M6o begin %s offset %#llx value %#x mask %#x\n", what, pa, value, mask);
	bool host = (pa >= WL_PARF && pa < WL_PARF + 0x4000) ||
	    (pa >= WL_DBI && pa < WL_DBI + 0x1000);
	/* Cycle link control is contained by PERST, not restored to running. */
	bool journal = host && pa != WL_PARF + 0x1b0 && pa != WL_DBI + 0xf24;
	uint32_t old = wl_prd(pa);
	/* W9: try8's zero-fault POR abort implies an all-ones old INTVEC.
	 * No old bits are used or journaled for this full write. Earlier BHI
	 * preflights read zero after writing it: keep strict readback below,
	 * including after SYS_ERR reset; this is not a write-only assumption. */
	bool intvec_zero = wl_mhi_valid && (wl_seq_flags & WL_O_POR) &&
	    pa == WL_BAR0+wl_bhi+0x20 && value == 0 && mask == ~0u;
	bool ones_bad = old == ~0u && !intvec_zero &&
	    !(wl_msi_pvm && pa == WL_EPCFG+wl_msi_cap+wl_msi_mask_off());
	bool full = journal && wl_seq_undos >= WL_SEQ_UNDO_MAX;
	if (fw_faults || ones_bad || full) {
		printf("FUXI WLAN: M6o refused %s offset %#llx old %#x faults %u undos %u/%u journal %u reason %s\n",
		    what, pa, old, fw_faults, wl_seq_undos, WL_SEQ_UNDO_MAX, journal,
		    fw_faults ? "access fault" : ones_bad ? "old all ones" : "undo full");
		return 1;
	}
	uint32_t v = (old & ~mask) | (value & mask);
	/* PCI command/control shares a dword with W1C status. */
	if ((pa >= WL_EPCFG && pa < WL_EPCFG + 0x1000 && mask == 0xffff) ||
	    pa == WL_EPCFG+4 || pa == WL_DBI+0x78 || (wl_pm_cap && pa == WL_EPCFG+wl_pm_cap+4)) { v &= 0xffff; }
	if (wl_pm_cap && pa == WL_EPCFG+wl_pm_cap+4) { v &= ~0x8000u; } /* PME status W1C */
	if (journal) {
		wl_seq_undo[wl_seq_undos++] = (struct wl_seq_undo){pa, old, mask,
		    pa == WL_DBI + 0x8a8 || pa == WL_DBI + 0x8ac || (pa == WL_DBI+0x890 && mask == 1) ? wl_eq_bank : 0};
	}
	wl_pwr(pa, v);
	uint32_t rb = wl_prd(pa);
	printf("FUXI WLAN: M6o %s offset %#llx write %#x read %#x mask %#x old %#x faults %u undos %u/%u\n",
	    what, pa, v, rb, mask, old, fw_faults, wl_seq_undos, WL_SEQ_UNDO_MAX);
	/* PME_TURNOFF self-clears once the message is sent; PM_STTS is checked next. */
	if (pa == WL_DBI + 0xf24) { return fw_faults ? 1 : 0; }
	return fw_faults || (rb & mask) != (v & mask);
}

static int
wl_m6o_host(unsigned step)
{
	if (!wl_seq_flags) { return 0; }
	if (wl_variant_invalid || fw_faults) { return 1; }
	if (step == 8) {
		if ((wl_seq_flags & WL_O_PARF) &&
		    (wl_m6o_write("PARF DBI_BASE_ADDR", WL_PARF+0x350, 0, ~0u) ||
		     wl_m6o_write("PARF SYS_CTRL", WL_PARF, 0x365e, ~0u) ||
		     wl_m6o_write("PARF SLV_ADDR_SPACE_SIZE", WL_PARF+0x358, 0x04000000, ~0u))) { return 1; }
		if ((wl_seq_flags & WL_O_HOSTIRQ) &&
		    (wl_m6o_write("PARF INT_ALL_MASK zero", WL_PARF+0x22c, 0, ~0u) ||
		     wl_m6o_write("PARF INT_ALL_MASK events", WL_PARF+0x22c, 0x3fc0d202, ~0u))) { return 1; }
	}
	if (step == 10 && (wl_seq_flags & WL_O_DBI)) {
		if (wl_m6o_write("PARF CFG_BITS_3", WL_PARF+0x2c4, 0x100, 0x101) ||
		    wl_m6o_write("DBI LANE_SKEW", WL_DBI+0x714, 0x20, 0x20)) { return 1; }
	}
	if (step == 12 && (wl_seq_flags & WL_O_DBI)) {
		if (wl_m6o_write("DBI N_FTS", WL_DBI+0x70c, 0x8000, 0x8000) ||
		    wl_m6o_write("DBI AUX_CLK_FREQ", WL_DBI+0xb40, 20, ~0u)) { return 1; }
	}
	if (step != 11) { return 0; }
	if (wl_seq_flags & WL_O_EQ) {
		unsigned speed = wl_prd(WL_DBI+0x7c) & 15;
		if (fw_faults || speed < 3 || speed > 4) { return 1; }
		if (wl_m6o_write("DBI GEN3_GEN2_CTRL", WL_DBI+0x80c, 0x100, 0x1f00)) { return 1; }
		for (unsigned gen = 3; gen <= speed; gen++) {
			wl_eq_bank = gen;
			if (wl_m6o_write("DBI RATE_SHADOW_SEL", WL_DBI+0x890, (gen-3)<<24, 0x03000000) ||
			    wl_m6o_write("DBI EQ preset vector", WL_DBI+0x8a8, 0, 0x00ffff00) ||
			    wl_m6o_write("DBI ZRXDC", WL_DBI+0x890, 0, 1) ||
			    wl_m6o_write("DBI EQ FMDC", WL_DBI+0x8ac, 0, 0x1f)) { return 1; }
		}
		wl_eq_bank = 0;
		if (wl_m6o_write("DBI RO write enable", WL_DBI+0x8bc, 1, 1) ||
		    wl_m6o_write("DBI Gen3 core preset", WL_DBI+0x154, 0x55555555, ~0u) ||
		    (speed >= 4 && wl_m6o_write("DBI Gen4 core preset", WL_DBI+0x188, 0x55555555, ~0u)) ||
		    wl_m6o_write("DBI RO write disable", WL_DBI+0x8bc, 0, 1)) { return 1; }
	}
	if ((wl_seq_flags & WL_O_DBI) &&
	    wl_m6o_write("DBI MRRS 4096", WL_DBI+0x78, 0x5000, 0x7000)) { return 1; }
	return 0;
}

/* Bounded standard PCI capability walk; reject malformed/cyclic lists. */
static unsigned
wl_m6o_cap(unsigned id)
{
	unsigned p = wl_prd(WL_EPCFG+0x34) & 255;
	uint64_t seen = 0;
	while (p) {
		if (p < 0x40 || p > 0xfc || (p & 3) || (seen & (1ull << (p/4)))) { return 0; }
		seen |= 1ull << (p/4);
		uint32_t v = wl_prd(WL_EPCFG+p);
		if (fw_faults || v == ~0u) { return 0; }
		if ((v & 255) == id) { return p; }
		p = (v >> 8) & 255;
	}
	return 0;
}

static int
wl_m6o_wait(uint64_t pa, uint32_t mask, uint32_t expected, unsigned ms)
{
	printf("FUXI WLAN: M6o begin wait %#llx mask %#x expected %#x limit %u ms\n", pa, mask, expected, ms);
	for (unsigned i = 0; i <= ms; i++) {
		uint32_t v = wl_prd(pa);
		if (fw_faults || v == ~0u) { return 1; }
		if ((v & mask) == expected) {
			printf("FUXI WLAN: M6o wait %#llx read %#x mask %#x after %u ms\n", pa, v, mask, i);
			return 0;
		}
		if (i < ms) { wl_sleep(1); }
	}
	printf("FUXI WLAN: M6o wait %#llx mask %#x expected %#x timeout %u ms\n", pa, mask, expected, ms);
	return 1;
}

/* ---- opt-in ON-only rail re-votes; audit: docs-lanes/wlan-rails.md ----
 * External PMU inputs, not the WCN PMU's internal WLAN output regulators.
 * Preserve M6o's downstream order and requested mV, including S4E=950.
 * VRM register facts: voltage +0 (mV), enable +4, mode +8; PMIC HPM/PWM=7.
 * rh_send already supports a waited single-register regulator request.
 * Salami (K-W1b): ldob15 is 0x40b00 in the salami cmd-db (0x40a00 there is
 * ldob12, the panel vddio); the other five addresses and all six mV are the
 * same as salami's (live DT chip_cfg@1, wl_salami_mv), so only that row
 * differs. wl_salami_vote_ok checks every vote again on its own.
 */
static const struct {
	unsigned idx, addr, mv;
	const char *name;
} wl_rails[] = {
#if defined(FUXI_SALAMI)
	{0, 0x40b00, 1800, "ldob15"},
#else
	{0, 0x40a00, 1800, "ldob15"},
#endif
	{4, 0x44000, 1000, "smpg5"},
	{2, 0x40400, 980, "smpg2"},
	{1, 0x40200, 950, "smpe4"},
	{5, 0x40600, 1900, "smpg6"},
	{3, 0x40500, 1350, "smpg4"},
};

static int
wl_rails_on(unsigned variant, const char *where)
{
	if (!wl_rails_variant || wl_rails_variant != variant) { return 0; }
	printf("FUXI WLAN: rails variant %u %s begin ON-only\n", variant, where);
	/* Validate the complete cmd-db table before the first vote. */
	for (unsigned i = 0; i < 6; i++) {
		unsigned addr = wl3_report[20+wl_rails[i].idx];
		if (addr != wl_rails[i].addr || fw_faults) {
			printf("FUXI WLAN: rails skip %s cmd-db addr %#x expected %#x faults %u; refused\n",
			    wl_rails[i].name, addr, wl_rails[i].addr, fw_faults);
			return 1;
		}
	}
	for (unsigned i = 0; i < 6; i++) {
		const unsigned values[] = {wl_rails[i].mv, 1, 7};
		const char *const kinds[] = {"voltage-mV", "enable", "mode-HPM/PWM"};
		for (unsigned reg = 0; reg < 3; reg++) {
			unsigned addr = wl_rails[i].addr + 4*reg;
			printf("FUXI WLAN: rails begin %s %s addr %#x value %u\n",
			    wl_rails[i].name, kinds[reg], addr, values[reg]);
			unsigned e = rh_send(0, addr, values[reg]);
			printf("FUXI WLAN: rails %s %s addr %#x value %u completion %#x\n",
			    wl_rails[i].name, kinds[reg], addr, values[reg], e);
			if (e) { return 1; } /* no retry, rollback or OFF vote */
		}
		wl_sleep(1); /* as in the existing M3 rail votes; no analog readback */
	}
	printf("FUXI WLAN: rails variant %u %s complete ON-only\n", variant, where);
	return 0;
}
/* ---- end opt-in ON-only rail re-votes ---- */

/* Explicit cyclerails only: the original rail cycle froze the SoC once.
 * These six votes may be shared; plain cycle and its undo never alter them. */
static int
wl_m6o_rails(bool on)
{
	if (!(wl_seq_flags & WL_O_CYCLERAILS)) { return 0; }
	if (wl_rails_variant) {
		printf("FUXI WLAN: rails ON-only refuses legacy cyclerails vote\n");
		return 1;
	}
#if defined(FUXI_SALAMI)
	/* Second fence behind the init refusal of cyclerails: never a rail OFF. */
	if (!on) {
		printf("FUXI WLAN: salami refuses the cyclerails rail OFF votes\n");
		return 1;
	}
#endif
	for (unsigned n = 0; n < 6; n++) {
		unsigned i = on ? n : 5-n, addr = wl3_report[20+wl_rails[i].idx];
		if (addr != wl_rails[i].addr || fw_faults) { return 1; }
		unsigned e = 0;
		if (on) {
			printf("FUXI WLAN: M6o begin rail voltage addr %#x mV %u\n", addr, wl_rails[i].mv);
			e = rh_send(0, addr, wl_rails[i].mv);
			if (e) { return 1; }
		}
		printf("FUXI WLAN: M6o begin rail enable addr %#x value %u\n", addr+4, on ? 1u : 0u);
		e = rh_send(0, addr+4, on ? 1 : 0);
		printf("FUXI WLAN: M6o WLAN rail %#x %s mV %u completion %#x\n", addr, on ? "on" : "off", wl_rails[i].mv, e);
		if (e) { return 1; }
	}
	return 0;
}

struct wl_seq_config { unsigned off[13]; uint32_t value[13]; };
static int
wl_m6o_config_save(struct wl_seq_config *cfg)
{
	unsigned pc = wl_m6o_cap(0x10);
	if (!pc || pc > 0xcc || ((wl_prd(WL_EPCFG+pc)>>16)&15) < 2) { return 1; }
	const unsigned off[] = {4, 0xc, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x3c,
	    pc+8, pc+0x10, pc+0x28, pc+0x30};
	for (unsigned i = 0; i < 13; i++) {
		cfg->off[i] = off[i]; cfg->value[i] = wl_prd(WL_EPCFG+off[i]);
		if (fw_faults || cfg->value[i] == ~0u) { return 1; }
	}
	return 0;
}

static int
wl_m6o_config_restore(const struct wl_seq_config *cfg)
{
	/* BARs and controls first, saved command last, always MASTER clear. */
	for (unsigned i = 1; i < 13; i++) {
		uint32_t mask = i >= 9 ? 0xffff : i == 8 ? 255 : i == 1 ? 0xffff : ~0u;
		if (wl_m6o_write("restore config", WL_EPCFG+cfg->off[i], cfg->value[i], mask)) { return 1; }
	}
	return wl_m6o_write("restore command", WL_EPCFG+4, (cfg->value[0]&~4u)|0x402, 0xffff);
}

/* A trained controller taken down without reset can hang the NoC on DBI.
 * Keep every PCIe0 aperture closed until core reset + PHY ready. Then permit
 * only local DBI (not ELBI/iATU/config/BAR) for the downstream pre-training writes.
 * Close that exception on every resume exit; L0 alone opens all apertures. */
static bool wl_link_off, wl_dbi_ready;

static bool
wl_link_refused(uint64_t pa)
{
	if (!wl_link_off || pa < 0x60000000ull || pa >= 0x64000000ull) { return false; }
	if (wl_dbi_ready && pa < WL_DBI + 0xf20) { return false; }
	printf("FUXI WLAN: PA %#llx refused while the PCIe link is down\n", pa);
	return true;
}

static int
wl_m6o_cycle_link(bool up)
{
	if (!up) {
		wl_link_off = true; wl_dbi_ready = false;
		printf("FUXI WLAN: M6o cycle link down dwell 1 ms, no DBI access\n");
		wl_sleep(1);
		printf("FUXI WLAN: M6o cycle PARF LTSSM %#x after link down\n", wl_prd(WL_PARF+0x1b0));
		return fw_faults ? 1 : 0;
	}
	uint32_t seen[2] = {0, 0}, lt = 0;
	unsigned ms;
	printf("FUXI WLAN: M6o cycle wait PARF LTSSM L0 before DBI\n");
	printf("FUXI WLAN: M6o begin wait %#llx mask 0x3f expected 0x11 limit 1000 ms\n", WL_PARF+0x1b0);
	for (ms = 0; ms <= 1000; ms++) {
		lt = wl_prd(WL_PARF+0x1b0);
		if (fw_faults || lt == ~0u) { break; }
		unsigned state = lt & 0x3f;
		seen[state / 32] |= 1u << (state % 32);
		if (state == 0x11) { break; }
		if (ms < 1000) { wl_sleep(1); }
	}
	bool l0 = !fw_faults && lt != ~0u && (lt & 0x3f) == 0x11;
	if (l0) {
		printf("FUXI WLAN: M6o wait %#llx read %#x mask 0x3f after %u ms\n", WL_PARF+0x1b0, lt, ms);
	} else if (!fw_faults) {
		printf("FUXI WLAN: M6o wait %#llx mask 0x3f expected 0x11 timeout 1000 ms\n", WL_PARF+0x1b0);
	}
	int bad = 1;
	if (l0) {
		wl_link_off = false;
		bad = wl_m6o_wait(WL_DBI+0x80, 1u<<29, 1u<<29, 100);
		if (bad) { wl_link_off = true; }
	}
	/* Diagnostics are PARF/PHY only, including timeout and access-fault exits. */
	uint32_t pm = wl_prd(WL_PARF+0x24), phy = wl_prd(WL_PHY+0x214);
	printf("FUXI WLAN: M6q cycle link %s after %u ms LTSSM %#x seen %#x/%#x PM_STTS %#x PHY_STATUS %#x\n",
	    !bad && !fw_faults ? "L0" : "FAILED", ms > 1000 ? 1000 : ms, lt, seen[0], seen[1], pm, phy);
	return bad || fw_faults;
}

/* Re-enter the enable path with PERST low and LTSSM off, not merely an
 * LTSSM toggle on a stale controller. No clock/GDSC/rail teardown is needed
 * for this reset/replay; all those votes remain owned by the initial path. */
static int
wl_m6q_resume(void)
{
	int bad = 1;
	printf("FUXI WLAN: M6q cycle begin PHY power-down and sw reset; no DBI access\n");
	wl_pwr(WL_PHY+0x200, 1); /* PCS SW_RESET */
	wl_pwr(WL_PHY+0x240, 0); /* downstream disable: power-down control */
	if (fw_faults) { goto out; }
	printf("FUXI WLAN: M6q cycle begin step 8 core reset/PARF, PERST held low\n");
	if (wl3_step(8) || fw_faults || wl3_faults) { goto out; }
	/* Reset may have changed LTSSM; keep its enable bit explicitly clear. */
	if (wl_m6o_write("cycle LTSSM off after core reset", WL_PARF+0x1b0, 0, 0x100)) { goto out; }
	printf("FUXI WLAN: M6q cycle begin step 9 PHY init/ready, reset hold 1 ms\n");
	int phy_bad = wl3_phy_init(1000);
	printf("FUXI WLAN: M6q cycle PHY init %s ready after %u us PHY_STATUS %#x\n",
	    phy_bad || fw_faults || wl3_faults ? "FAILED" : "done", wl3_report[62], wl3_report[63]);
	if (phy_bad || fw_faults || wl3_faults) { goto out; }
	/* Verify the boundary before allowing downstream's local DBI writes. */
	if ((wl_prd(WL_GCC_PA+0x6b000) & 1) || (wl_prd(WL_GCC_PA+0x6c01c) & 1) ||
	    (wl_prd(WL_PHY+0x214) & 0x40) || (wl_prd(WL_PHY+0x200) & 1) ||
	    (wl_prd(WL_PHY+0x240) & 3) != 3 || (wl_prd(WL_PHY+0x244) & 3) != 3 ||
	    (wl_prd(TS_TLMM+0x1000ull*94+4) & 2) || (wl_prd(WL_PARF+0x1b0) & 0x100) ||
	    fw_faults) { goto out; }
	printf("FUXI WLAN: M6q cycle core reset done + PHY ready; local DBI allowed, config/BAR blocked\n");
	wl_dbi_ready = true;
	printf("FUXI WLAN: M6q cycle begin PHY-ready EP latency 10 ms, PERST held low\n");
	wl_sleep(10); /* X ep-latency, K enable after PHY ready */
	if (wl_m6o_host(10) ||
	    wl_m6o_write("cycle PERST high", TS_TLMM+0x1000ull*94+4, 2, 2)) { goto out; }
	printf("FUXI WLAN: M6o cycle begin PERST high dwell 1 ms\n");
	wl_sleep(1); wl_perst_done = mach_absolute_time();
	if (wl_m6o_host(11)) { goto out; }
	wl_dbi_ready = false; /* pre-training exception ends before LTSSM enable */
	if (wl_m6o_write("cycle LTSSM resume", WL_PARF+0x1b0, 0x100, 0x100) ||
	    wl_m6o_cycle_link(true)) { goto out; }
	wl_link_done = mach_absolute_time();
	/* Core reset invalidates RC bus numbers/windows too. Reuse enumeration
	 * and MEM setup, without BAR sizing or changing the saved EP controls. */
	printf("FUXI WLAN: M6q cycle begin step 12 CFG0 and step 14 MEM/window restore\n");
	if (wl3_step(12) || wl4_step(14) || fw_faults || wl3_faults) { goto out; }
	bad = 0;
out:
	wl_dbi_ready = false;
	return bad;
}

static int
wl_m6o_cycle(void)
{
	if (wl_cycle_attempted || fw_faults || wl_prd(WL_EPCFG) != 0x110717cb) { return 1; }
	wl_cycle_attempted = true;
	unsigned pm = wl_m6o_cap(1);
	wl_pm_cap = pm;
	if (!pm || pm > 0xf8 || wl_disable_irqs()) { return 1; }
	struct wl_seq_config cfg;
	if (wl_m6o_write("cycle clear MASTER", WL_EPCFG+4, 0, 4) || wl_m6o_config_save(&cfg)) { return 1; }
	printf("FUXI WLAN: M6o cycle saved config after clear MASTER; disable function then D3hot\n");
	if (wl_m6o_write("cycle disable function", WL_EPCFG+4, 0x400, 0xffff) ||
	    wl_m6o_write("cycle D3hot", WL_EPCFG+pm+4, 3, 3)) { return 1; }
	wl_sleep(10); /* PCI D3hot transition, before PME_TURNOFF */
	if (wl_m6o_write("cycle PME_TURNOFF", WL_DBI+0xf24, 0x10, ~0u) ||
	    /* Downstream logs a missing L23_READY and powers off anyway. */
	    (wl_m6o_wait(WL_PARF+0x24, 0x20, 0x20, 100) && !fw_faults &&
	     (printf("FUXI WLAN: M6o cycle L23_READY not received, continuing as downstream\n"), 0))) { return 1; }
	/* Latch before attempting link-down writes, including partial/faulted ones. */
	wl_link_off = true; wl_dbi_ready = false;
	if (wl_m6o_write("cycle PERST low", TS_TLMM+0x1000ull*94+4, 0, 2) ||
	    wl_m6o_write("cycle LTSSM off", WL_PARF+0x1b0, 0, 0x100) ||
	    wl_m6o_cycle_link(false) ||
	    wl_m6o_write("cycle WLAN_EN low", TS_TLMM+0x1000ull*80+4, 0, 2)) { return 1; }
	wl_cycle_powered = true; /* WLAN_EN containment; partial rail votes only if selected */
	if (!(wl_seq_flags & WL_O_CYCLERAILS)) {
		printf("FUXI WLAN: M6o cycle retains all six rails; WLAN_EN only\n");
	}
	if ((wl_seq_flags & WL_O_CYCLERAILS) && wl_m6o_rails(false)) { return 1; }
	printf("FUXI WLAN: M6o cycle begin WLAN_EN low dwell 10 ms\n");
	wl_sleep(10);
	if (((wl_seq_flags & WL_O_CYCLERAILS) && wl_m6o_rails(true)) ||
	    wl_rails_on(1, "cycle before WLAN_EN high") ||
	    wl_m6o_write("cycle WLAN_EN high", TS_TLMM+0x1000ull*80+4, 2, 2)) { return 1; }
	printf("FUXI WLAN: M6o cycle begin WLAN_EN high dwell 10 ms\n");
	wl_sleep(10); /* retained M6o WLAN_EN dwell; EP latency follows PHY ready */
	if (wl_m6q_resume()) { return 1; }
	printf("FUXI WLAN: M6o cycle begin endpoint ID check\n");
	if (wl_prd(WL_EPCFG) != 0x110717cb ||
	    wl_m6o_write("cycle D0", WL_EPCFG+pm+4, 0, 3)) { return 1; }
	printf("FUXI WLAN: M6o cycle begin D0 dwell 10 ms\n");
	wl_sleep(10);
	if (wl_m6o_write("cycle enable function", WL_EPCFG+4, 0x402, 0xffff)) { return 1; }
	if (wl_m6o_config_restore(&cfg)) { return 1; }
	printf("FUXI WLAN: M6o cycle begin BHIOFF check offset 0x28\n");
	if (wl_mrd(0x28) != wl_bhi || fw_faults) { return 1; }
	printf("FUXI WLAN: M6o cycle begin BHIEOFF check offset 0x2c\n");
	if (wl_mrd(0x2c) != wl_bhie || fw_faults) { return 1; }
	printf("FUXI WLAN: M6o cycle begin PBL EE check offset %#x\n", wl_bhi+0x28);
	if (wl_mrd(wl_bhi+0x28) != 0 || fw_faults) { return 1; }
	printf("FUXI WLAN: M6o cycle resumed D0/config; MASTER follows owned SMMU install; WLAON skipped (KIWI DT gate absent)\n");
	return 0;
}

/* Step 18 failed silently on device right after a successful cycle; name the stage. */
static int
wl_m6o_master_fail(const char *what)
{
	printf("FUXI WLAN: M6o power-up failed at %s (faults %u)\n", what, fw_faults);
	return 1;
}

static int
wl_m6o_master(void)
{
	struct wl_seq_config cfg;
	if (wl_m6o_config_save(&cfg)) { return wl_m6o_master_fail("config save"); }
	if (wl_seq_flags & WL_O_MSI) {
		unsigned p = wl_m6o_cap(5);
		uint32_t cap = p ? wl_prd(WL_EPCFG+p) : 0;
		/* >=32 messages; 32- or 64-bit addressing. Require room for addr/data. */
		if (!p || p > 0xe8 || ((cap>>17)&7) < 5 ||
		    wl_disable_irqs() || wl_dma_alloc(&wl_msi_sink, PAGE_SIZE)) {
			printf("FUXI WLAN: M6o MSI cap %#x at %#x sink %p\n", cap, p, (void *)wl_msi_sink.va);
			return wl_m6o_master_fail("MSI capability/IRQ disable/sink");
		}
		wl_msi_pvm = !!(cap & (1u<<24));
		wl_msi_64 = !!(cap & (1u<<23));
		if (!wl_msi_64 && (wl_msi_sink.iova >> 32)) {
			printf("FUXI WLAN: M6o MSI 32-bit capability but sink IOVA %#llx above 4 GiB\n", wl_msi_sink.iova);
			return wl_m6o_master_fail("MSI 32-bit sink address");
		}
		printf("FUXI WLAN: M6o MSI cap %#x at %#x %u-bit%s\n", cap, p, wl_msi_64 ? 64 : 32, wl_msi_pvm ? " per-vector mask" : "");
		for (unsigned i = 0; i < wl_msi_words(); i++) { wl_msi_saved[i] = wl_prd(WL_EPCFG+p+4*i); }
		wl_msi_saved[0] &= ~(1u<<16); /* cleanup never reenables inherited IRQs */
		if (fw_faults) { return wl_m6o_master_fail("MSI save fault"); }
		wl_msi_cap = p;
		wl_dma_clean(&wl_msi_sink);
	}
	if (wl_smmu_enable()) { return wl_m6o_master_fail("SMMU enable"); }
	if (wl_fabric_prepare()) { return wl_m6o_master_fail("fabric prepare"); }
	unsigned pm = wl_m6o_cap(1);
	wl_pm_cap = pm;
	if (!pm || pm > 0xf8 || wl_m6o_write("power-up D0", WL_EPCFG+pm+4, 0, 3) ||
	    wl_m6o_write("power-up enable MEMORY", WL_EPCFG+4, 0x402, 0xffff) ||
	    wl_m6o_config_restore(&cfg) || wl_download_aspm()) { return wl_m6o_master_fail("D0/config restore/ASPM"); }
	if (wl_seq_flags & WL_O_MSI) {
		unsigned p = wl_msi_cap;
		if ((wl_msi_64 && wl_m6o_write("MSI address high", WL_EPCFG+p+8, (uint32_t)(wl_msi_sink.iova>>32), ~0u)) ||
		    wl_m6o_write("MSI address low", WL_EPCFG+p+4, (uint32_t)wl_msi_sink.iova, ~0u) ||
		    wl_m6o_write("MSI data base", WL_EPCFG+p+wl_msi_data_off(), 0, 0xffff) ||
		    (wl_msi_pvm && wl_m6o_write("MSI unmask sink vectors", WL_EPCFG+p+wl_msi_mask_off(), 0, ~0u)) ||
		    wl_m6o_write("MSI 32 vectors enabled", WL_EPCFG+p, (5u<<20)|(1u<<16), 0x00710000)) { return 1; }
		printf("FUXI WLAN: M6o MSI 32 vectors (MHI 0..2) sink IOVA %#llx, polling; no ITS/GIC changes\n", wl_msi_sink.iova);
	}
	if (wl_m6o_write("MASTER before MHI prepare", WL_EPCFG+4, 0x406, 0xffff)) { return 1; }
	printf("FUXI WLAN: M6o D0/config valid MASTER on before contexts/RX prepare; unmapped DMA faults contained\n");
	return 0;
}

static int
wl_m6o_rx_clear(void)
{
	for (unsigned o = 0x60; o <= 0x78; o += 4) {
		wl_mwr(wl_bhie+o, 0);
		uint32_t rb = wl_mrd(wl_bhie+o);
		printf("FUXI WLAN: M6o prepare RX zero offset %#x write 0 read %#x\n", o, rb);
		if (rb || fw_faults) { return 1; }
	}
	return 0;
}

static int
wl_m6o_prepare(void)
{
	/* Recheck discovery before RX writes; BHIE in wl5_status is TXSTATUS
	 * (+0x44), not BHIEOFF. Never infer a new base from that status word. */
	uint32_t bhi = wl_mrd(0x28), bhie = wl_mrd(0x2c);
	printf("FUXI WLAN: W9 prepare BHIOFF %#x/%#x BHIEOFF %#x/%#x (live/saved) INTVEC offset %#x undos %u/%u faults %u\n",
	    bhi, wl_bhi, bhie, wl_bhie, wl_bhi+0x20, wl_seq_undos, WL_SEQ_UNDO_MAX, fw_faults);
	if (fw_faults || bhi != wl_bhi || bhie != wl_bhie) { return 1; }
	uint64_t addr = wl_rxvectors.iova;
	unsigned bytes = (WL_RDDM_SIZE/WL_CHUNK)*sizeof(struct wl_vector);
	uint32_t seq; read_random(&seq, sizeof(seq)); seq &= WL_SEQ_MASK; if (!seq) { seq = 1; }
	if (wl_m6o_write("prepare RX high", WL_BAR0+wl_bhie+0x64, addr>>32, ~0u) ||
	    wl_m6o_write("prepare RX low", WL_BAR0+wl_bhie+0x60, (uint32_t)addr, ~0u) ||
	    wl_m6o_write("prepare RX bytes", WL_BAR0+wl_bhie+0x68, bytes, ~0u) ||
	    wl_m6o_write("prepare RX random DB", WL_BAR0+wl_bhie+0x70, seq, WL_SEQ_MASK) ||
	    wl_m6o_write("POR INTVEC zero", WL_BAR0+wl_bhi+0x20, 0, ~0u)) { return 1; }
	uint32_t ee = wl_mrd(wl_bhi+0x28), st = wl_mrd(0x48);
	if (fw_faults || ee != 0 || st == ~0u) { return 1; }
	if (((st>>8)&255) == 0xff) {
		/* SYS_ERR only; never an unconditional cold reset. */
		wl_mwr(0x38, 2);
		unsigned i;
		for (i = 0; i <= 40; i++) {
			uint32_t ctrl = wl_mrd(0x38);
			if (fw_faults || ctrl == ~0u) { return 1; }
			if (!(ctrl & 2)) { break; }
			if (i < 40) { wl_sleep(25); }
		}
		if (i > 40 || wl_m6o_write("POR INTVEC after reset", WL_BAR0+wl_bhi+0x20, 0, ~0u) ||
		    wl_mrd(wl_bhi+0x28) != 0 || (wl_mrd(0x48)&0xff04) || fw_faults) { return 1; }
	}
	printf("FUXI WLAN: M6o prepare RX zero +60..78, high/low/size/random DB %#x; POR EE %#x state %#x INTVEC 0\n", seq, ee, st);
	return 0;
}

/* Called only after BME clear, before PERST makes config inaccessible. */
static int
wl_m6o_msi_undo(void)
{
	if (!wl_msi_cap) { return 0; }
	unsigned faults0 = fw_faults;
	uint64_t pa = WL_EPCFG+wl_msi_cap;
	if (wl_contain_wr(pa, wl_msi_saved[0]) ||
	    (wl_prd(pa)&0x00710000) != (wl_msi_saved[0]&0x00710000)) { return 1; }
	for (unsigned i = 1; i < wl_msi_words(); i++) {
		uint32_t mask = i == wl_msi_data_off() / 4 ? 0xffff : ~0u;
		if (wl_contain_wr(pa+4*i, wl_msi_saved[i]&mask) ||
		    (wl_prd(pa+4*i)&mask) != (wl_msi_saved[i]&mask)) { return 1; }
	}
	if (fw_faults != faults0) { return 1; }
	wl_msi_cap = 0;
	printf("FUXI WLAN: M6o MSI disabled, saved address/data restored before PERST\n");
	return 0;
}

static int
wl_m6o_undo(bool contained)
{
	if (!contained) { return 1; }
	unsigned faults0 = fw_faults;
	int bad = 0;
	while (wl_seq_undos) {
		struct wl_seq_undo *u = &wl_seq_undo[wl_seq_undos-1];
		if (u->bank) {
			uint32_t v = wl_prd(WL_DBI+0x890);
			if (v == ~0u || wl_contain_wr(WL_DBI+0x890, (v&~0x03000000u)|((u->bank-3)<<24)) ||
			    (wl_prd(WL_DBI+0x890)&0x03000000) != (u->bank-3)<<24) { return 1; }
		}
		uint32_t v = wl_prd(u->pa);
		uint32_t restore = (v&~u->mask)|(u->old&u->mask);
		if (u->pa == WL_DBI+0x78) { restore &= 0xffff; }
		if (v == ~0u || wl_contain_wr(u->pa, restore) ||
		    (wl_prd(u->pa)&u->mask) != (u->old&u->mask)) { bad = 1; break; }
		wl_seq_undos--;
	}
	if (wl_cycle_powered) {
		printf("FUXI WLAN: M6o cycle undo begin WLAN_EN low\n");
		bad |= wl_contain_wr(TS_TLMM+0x1000ull*80+4, 0);
		bad |= !!(wl_prd(TS_TLMM+0x1000ull*80+4)&2);
		if (wl_seq_flags & WL_O_CYCLERAILS) { bad |= wl_m6o_rails(false); }
		if (!bad) { wl_cycle_powered = false; }
	}
	bad |= fw_faults != faults0;
	printf("FUXI WLAN: M6o undo host entries pending %u WLAN power %s result %s\n", wl_seq_undos,
	    wl_cycle_powered ? "unconfirmed" : "contained", bad ? "unconfirmed; quarantine" : "done");
	return bad;
}

static int
wl_read_firmware(void)
{
	vnode_t root = NULL, vp = NULL;
	vfs_context_t ctx = vfs_context_kernel();
	printf("FUXI WLAN: wait root <=120 s, firmware %s\n", wl_fw_path);
	for (unsigned i = 0; i <= 1200; i++) {
		root = vfs_rootvnode();
		if (root) { break; }
		if (i < 1200) { wl_sleep(100); }
	}
	if (!root) { return 1; }
	vnode_put(root);
	int e = vnode_open(wl_fw_path, FREAD | O_NOFOLLOW | O_NONBLOCK, 0, 0, &vp, ctx);
	if (e) {
		printf("FUXI WLAN: firmware open failed errno %d\n", e);
		return 2;
	}
	struct vnode_attr a;
	VATTR_INIT(&a); VATTR_WANTED(&a, va_data_size);
	e = vnode_getattr(vp, &a, ctx);
	if (e || vnode_vtype(vp) != VREG || !VATTR_IS_SUPPORTED(&a, va_data_size) ||
	    a.va_data_size < WL_CHUNK || a.va_data_size > WL_FW_MAX) {
		e = 3; goto close;
	}
	wl_fw_size = (uint32_t)a.va_data_size;
	unsigned read_bytes = (wl_seq_flags & WL_O_PREFIX) ? WL_CHUNK : wl_fw_size;
	if (wl_dma_alloc(&wl_image, read_bytes)) { e = 4; goto close; }
	/* Keep the BHI prefix allocation independent, but read the full file once
	 * from this vnode for W10. Allocate/map everything before the doorbells;
	 * never reopen a possibly changed path after SBL starts. */
	struct wl_dma *input = &wl_image;
	if ((wl_seq_flags & WL_O_PREFIX) && !wl_stop22) {
		if (wl_dma_alloc(&wl_amss, wl_fw_size)) { e = 4; goto close; }
		input = &wl_amss; read_bytes = wl_fw_size;
	}
	int resid = 0;
	e = vn_rdwr(UIO_READ, vp, (caddr_t)input->va, (int)read_bytes, 0,
	    UIO_SYSSPACE, 0,
	    vfs_context_ucred(ctx), &resid, vfs_context_proc(ctx));
	if (e || resid) {
		printf("FUXI WLAN: firmware read errno %d residual %d\n", e, resid);
		e = 5; goto close;
	}
	/* Qualcomm images need not be native ARM ELFs. Check ident, header size,
	 * version and bounded program table; do not reject vendor e_machine. */
	if (input == &wl_amss) {
		memcpy((void *)wl_image.va, (const void *)wl_amss.va, WL_CHUNK);
		wl_dma_clean(&wl_amss);
		printf("FUXI WLAN: W10 prefix %#x bytes; full AMSS %u bytes retained from same file read\n", WL_CHUNK, wl_fw_size);
	}
	const uint8_t *b = (const uint8_t *)wl_image.va;
	uint16_t ehsize, phsize, phnum;
	uint32_t version; memcpy(&version, b + 20, 4);
	uint64_t phoff = 0;
	if (version != 1 || memcmp(b, "\177ELF", 4) || (b[4] != 1 && b[4] != 2) || b[5] != 1 || b[6] != 1) { e = 6; goto close; }
	if (b[4] == 1) {
		uint32_t p; memcpy(&p, b + 28, 4); phoff = p;
		memcpy(&ehsize, b + 40, 2); memcpy(&phsize, b + 42, 2); memcpy(&phnum, b + 44, 2);
	} else {
		memcpy(&phoff, b + 32, 8);
		memcpy(&ehsize, b + 52, 2); memcpy(&phsize, b + 54, 2); memcpy(&phnum, b + 56, 2);
	}
	if (ehsize != (b[4] == 1 ? 52 : 64) || !phnum || phsize != (b[4] == 1 ? 32 : 56) ||
	    phoff < ehsize || phoff > WL_CHUNK || (uint64_t)phsize * phnum > WL_CHUNK - phoff) { e = 6; goto close; }
	/* FBC prefix must contain ELF metadata and any Qualcomm hash segment.
	 * Other PT_LOAD ranges may be in the later BHIe transfer. No extraction. */
	unsigned hashes = 0;
	uint64_t hash_off = 0, hash_size = 0;
	for (unsigned i = 0; i < phnum; i++) {
		const uint8_t *ph = b + phoff + (uint64_t)i * phsize;
		uint32_t flags;
		memcpy(&flags, ph + (b[4] == 1 ? 24 : 4), 4);
		if ((flags & 0x07000000) != 0x02000000) { continue; }
		if (b[4] == 1) {
			uint32_t off, size;
			memcpy(&off, ph + 4, 4); memcpy(&size, ph + 16, 4);
			hash_off = off; hash_size = size;
		} else {
			memcpy(&hash_off, ph + 8, 8); memcpy(&hash_size, ph + 32, 8);
		}
		if (++hashes > 1 || !hash_size || hash_off > WL_CHUNK || hash_size > WL_CHUNK - hash_off) { e = 6; goto close; }
	}
	printf("FUXI WLAN: M6f FBC metadata fits prefix phend %#llx hash segments %u offset %#llx size %#llx\n",
	    phoff + (uint64_t)phsize * phnum, hashes, hash_off, hash_size);
	/* MBN v5 fields are lengths, not proof of authenticity. Do not print
	 * certificate contents, keys or device identity, or interpret other versions. */
	if (hashes && hash_size >= 40) {
		uint32_t h[10]; memcpy(h, b + hash_off, sizeof(h));
		if (h[1] == 5) {
			uint64_t used = (uint64_t)h[2] + h[3] + h[5] + h[7] + h[9];
			printf("FUXI WLAN: M6m MBNv5 code %#x QTI sig/cert %#x/%#x OEM sig/cert %#x/%#x bounds %s; authentication unverified\n",
			    h[5], h[2], h[3], h[7], h[9], used <= hash_size - 40 ? "fit" : "INVALID");
			if (used > hash_size - 40) { e = 6; goto close; }
		} else {
			printf("FUXI WLAN: M6m hash header version %u unparsed; authentication unverified\n", h[1]);
		}
	}
	if ((wl_seq_flags & WL_O_PREFIX) && wl_stop22) {
		printf("FUXI WLAN: M6o prefix separate coherent allocation %#x bytes, file %u; full image/FBC vectors deferred beyond BHI\n", WL_CHUNK, wl_fw_size);
	}
	wl_dma_clean(&wl_image);
	wl5_report[17] = wl_fw_size; wl5_report[18] = (uint32_t)wl_image.iova;
	printf("FUXI WLAN: firmware %s bytes %u ELF%u DMA %#llx cache-clean\n", wl_fw_path, wl_fw_size, b[4] == 1 ? 32 : 64, wl_image.iova);
	printf("FUXI WLAN: M6e BHI layout file offset 0 ELF ident %02x%02x%02x%02x class %u phoff %#llx phnum %u first %#x bytes; no segment extraction\n",
	    b[0], b[1], b[2], b[3], b[4], phoff, phnum, WL_CHUNK);
	e = 0;
close:
	(void)vnode_close(vp, FREAD, ctx);
	return e;
}

/* Original bounded polling against public MHI register fields. BHI and BHIe
 * have distinct owned sequences. Never infer completion from EE ordering. */
static int
wl_poll(unsigned mode)
{
	uint32_t ee = 0, st = 0, bs = 0, es = 0, ctrl = 0;
	bool mismatch_logged = false, amss_logged = false;
	const char *why = "timeout";
	int error = 2;
	for (unsigned i = 0; i <= 200; i++) {
		if (wl_owned && ((wl_prd(wl_cb_addr(wl_cb) + 0x58) & 0xc00001fe) ||
		    ((wl_prd(WL_SMMU + 0x48) & 2) && (wl_prd(WL_SMMU + 0x54) & 0xffff) == 0x1401))) {
			(void)wl_smmu_fault(22 + mode);
			why = "SMMU fault"; error = 3; break;
		}
		ee = wl_mrd(wl_bhi + 0x28); st = wl_mrd(0x48);
		bs = wl_mrd(wl_bhi + 0x2c); es = wl_mrd(wl_bhie + WL_BHIE_TXSTATUS);
		ctrl = wl_mrd(0x38);
		uint32_t state = (st >> 8) & 255;
		if (fw_faults || ee == ~0u || st == ~0u || ctrl == ~0u) { why = "MMIO/access lost"; error = 1; break; }
		if (ee == 3) { why = "RDDM requested (RX sink only; no dump/retry)"; error = 1; break; }
		if (ee == 6) { why = "EDL error EE"; error = 1; break; }
		if (ee != 0 && ee != 1 && ee != 2) { why = "unsupported EE (including WFW/FP)"; error = 1; break; }
		if ((st & 4) || state == 255 || (ctrl & 2)) { why = "SYS_ERR/reset"; error = 1; break; }
		if ((bs >> 30) == 3) { why = "BHI error"; error = 1; break; }
		if (mode >= 3 && (es >> 30) == 3) { why = "BHIe TXSTATUS error"; error = 1; break; }
		uint32_t status = mode >= 3 ? es : bs;
		uint32_t seq = mode >= 3 ? wl_bhie_seq : wl_seq;
		bool matched = (status >> 30) == 2 && (status & WL_SEQ_MASK) == seq && seq;
		if ((status >> 30) == 2 && !matched && !mismatch_logged) {
			printf("FUXI WLAN: W10 %s sequence mismatch raw %#x expected %#x; waiting for owned completion\n",
			    mode >= 3 ? "BHIe TXSTATUS" : "BHI STATUS", status, seq);
			mismatch_logged = true;
		}
		if (mode == 4 && ee == 2 && !amss_logged) {
			printf("FUXI WLAN: W10 AMSS EE reached state %#x; waiting M0\n", state);
			amss_logged = true;
		}
		if ((mode == 0 && matched && ee == 1 && state == 1 && (st & 1)) ||
		    (mode == 1 && ee == 1 && state == 1 && (st & 1)) ||
		    (mode == 2 && ee == 1 && state == 2) ||
		    (mode == 3 && matched && (ee == 1 || ee == 2)) ||
		    (mode == 4 && matched && ee == 2 && state == 2)) {
			printf("FUXI WLAN: W10 poll mode %u success EE %#x MHISTATUS %#x %s %#x seq %#x after %u ms\n",
			    mode, ee, st, mode >= 3 ? "TXSTATUS" : "BHI STATUS", status, seq, i * 25);
			return 0;
		}
		if (i < 200) { wl_sleep(25); }
	}
	printf("FUXI WLAN: W10 poll mode %u %s EE %#x MHISTATUS %#x MHICTRL %#x BHI %#x TXSTATUS %#x expected seq %#x (limit 5000 ms)\n",
	    mode, why, ee, st, ctrl, bs, es, mode >= 3 ? wl_bhie_seq : wl_seq);
	return error;
}

static void
wl_probe_pattern(void)
{
	if (wl_probe != WL_PROBE_PATTERN) { return; }
	uint8_t *p = (uint8_t *)wl_image.va;
	for (vm_size_t i = 0; i < wl_image.size; i++) { p[i] = (uint8_t)(0xa5u ^ (i & 255)); }
	bzero(p, 64);
	memcpy(p, "\177ELF\377\377\0", 7); /* invalid class, endian and version */
	memcpy(p + 16, "FUXI-M6K-PATTERN", 16);
	printf("FUXI WLAN: M6k pattern first64 ELF class ff endian ff version 00 marker FUXI-M6K-PATTERN FNV1a64 %016llx\n",
	    wl_hash_bytes(0xcbf29ce484222325ull, p, WL_CHUNK));
}

/* Unlike normal wl_poll, continue observing after PBL's first error: a late
 * SMMU fault is the discriminant. Never clear FSR, retry, or proceed to READY.
 * The caller's existing abort contains DMA before any allocation is freed. */
static int
wl_probe_observe(void)
{
	uint32_t flags = wl_bhi_diag(0);
	for (unsigned i = 0; i < 200 && !fw_faults; i++) { wl_sleep(25); }
	flags |= wl_bhi_diag(1);
	(void)wl_smmu_fault(22);
	uint64_t cb = wl_cb_addr(wl_cb);
	uint32_t fsr = wl_prd(cb + 0x58);
	uint64_t far = ((uint64_t)wl_prd(cb + 0x64) << 32) | wl_prd(cb + 0x60);
	uint32_t sid = wl_prd(wl_gr1 + 0x400 + 4 * wl_cb) & 0xffff;
	const char *result = "no-CB-translation-fault";
	if (fsr & 2) {
		result = far >= wl_image.iova && far - wl_image.iova < wl_bhi_bytes() && sid == 0x1401 ?
		    "target-translation-fault" : "other-translation-fault";
	}
	if (fw_faults || (flags & WL_DIAG_TRANSPORT)) { result = "transport-inconclusive"; }
	printf("FUXI WLAN: M6k result probe=%s %s FSR %#x FAR %#llx SID %#x observation 5000 ms; stop/contain (not firmware success)\n",
	    wl_probe_names[wl_probe], result, fsr, far, sid);
	wl_host_diag("probe-completion");
	if (!fw_faults && !(flags & WL_DIAG_TRANSPORT)) { wl_boot_logs(); }
	if (wl_variant_active()) {
		if (wl_seq_flags) {
			printf("FUXI WLAN: M6o result seq mask %#x EE %#x STATUS %#x ERRCODE %#x; stop/contain at 22\n",
			    wl_seq_flags, wl_mrd(wl_bhi + 0x28), wl_mrd(wl_bhi + 0x2c), wl_mrd(wl_bhi + 0x30));
		}
		if (wl_cnss) {
			printf("FUXI WLAN: M6n result cnss=%s PDC consumed %u/13 EE %#x ERRCODE %#x; raw code, no security decode\n",
			    wl_cnss_names[wl_cnss], wl_pdc_acks, wl_mrd(wl_bhi + 0x28), wl_mrd(wl_bhi + 0x30));
		}
		printf("FUXI WLAN: M6m result fw=%s pre=%s bhi=%s vec=%s EE %#x ERRCODE %#x observation 5000 ms; stop/contain\n",
		    wl_fw_names[wl_fw], wl_pre_names[wl_pre], wl_full ? "full" : "off", wl_vec_names[wl_vec],
		    wl_mrd(wl_bhi + 0x28), wl_mrd(wl_bhi + 0x30));
		printf("FUXI WLAN: M6l result pre=%s bhi=%s vec=%s EE %#x ERRCODE %#x observation 5000 ms; stop/contain\n",
		    wl_pre_names[wl_pre], wl_full ? "full" : "off", wl_vec_names[wl_vec],
		    wl_mrd(wl_bhi + 0x28), wl_mrd(wl_bhi + 0x30));
		return 0x6c;
	}
	return 0x6b; /* deliberate diagnostic stop, including unexpected BHI success */
}

/* W11: original bounded polling transport. Protocol/ring semantics: public MHI
 * protocol documentation; register/packet layout facts: OpenBSD qwz definitions.
 * WLFW IDs/TLV layouts: ath12k qmi.h/qmi.c (BSD-3-Clause-Clear), Linux Foundation
 * (2018-2021), Qualcomm Innovation Center (2021-2022). No Linux core code used.
 * Sources, ownership and unsupported operations: docs-lanes/wlan-w11.md. */
/* W16 capacity facts: ath12k v6.12 mhi.c, WCN7850 (BSD-3-Clause-Clear,
 * The Linux Foundation 2020-2021; Qualcomm Innovation Center 2021-2024).
 * Selected before allocation/publication; never resize a live ring. */
#define WL_IPCR_N (wl16_large ? 64u : 16u)
#define WL_IPCR_MTU (wl16_large ? 8192u : 4096u)
#define WL_IPCR_TXR 0u
#define WL_IPCR_RXR (WL_IPCR_N * 16u)
#define WL_IPCR_TXBUF 0x1000u
#define WL_IPCR_TXMTU (wl_w14 ? 8192u : WL_IPCR_MTU)
#define WL_IPCR_RXBUF (WL_IPCR_TXBUF + WL_IPCR_N * WL_IPCR_TXMTU)
#define WL_IPCR_BYTES (WL_IPCR_RXBUF + WL_IPCR_N * WL_IPCR_MTU)
#define WL_QRTR_CTRL 0xfffffffeu
#define WL_QRTR_LOCAL 1u
#define WL_QRTR_PORT 0x4000u
struct wl_tre { uint64_t ptr; uint32_t d0, d1; };
_Static_assert(sizeof(struct wl_tre) == 16, "MHI ring element stride");
_Static_assert(2 * 64 * 16 <= WL_IPCR_TXBUF, "largest IPCR rings before buffers");
static unsigned wl_er_read[2], wl_tx_read, wl_tx_write, wl_rx_read, wl_rx_write;
static unsigned wl_cmd_write, wl_chan_attempts;
static uint64_t wl_cmd_pending;
static unsigned wl_cmd_channel, wl_cmd_opcode;
static uint32_t wl_chdb, wl_erdb, wl_server_node, wl_server_port, wl_peer_node;
static unsigned wl16_xfers[2], wl16_oob[2], wl16_dbmode[2];
static struct wl_tre wl16_last[2];
static bool wl_cmd_done, wl_hello, wl_cap_sent, wl_cap_done, wl_w11_stopping;
static unsigned wl_w11_error;
enum { WL12_DISCOVERY, WL12_IND, WL12_HOST, WL12_REQUEST, WL12_RESPOND,
    WL12_READY, WL12_CAP, WL12_DONE, WL14_REGDB, WL14_BDF, WL14_M3, WL14_READY,
    WL15_INI, WL15_CFG, WL15_MODE, WL15_OBSERVE, WL20_CE, WL21_HTC, WL22_WMI, WL23_SCAN };
static unsigned wl12_stage;
static uint16_t wl12_txn, wl12_msg;
static bool wl12_reply, wl12_requested, wl12_ready;
/* WCN7850 core_alloc initial MLO support and hw_params.def_num_link. */
static bool wl12_mlo_enabled;
static unsigned wl12_num_phy;
static bool wl13_delayed, wl13_negotiated;
static bool wl14_ready, wl14_init_done, wl14_board_valid;
static uint64_t wl15_ready_time;
static uint32_t wl14_board;
static uint32_t wl_cmem_base; static bool wl_cmem_valid;   /* C2f: QMI dev_mem[0] CMEM region, read in wl_qmi_cap */
static int wl_w12_run(void);
static int wl_qmi_w12(const uint8_t *, unsigned);

static void
wl15_elapsed(const char *step)
{
	if (!wl_w15 || !wl14_ready) { return; }
	uint64_t ns = 0;
	absolutetime_to_nanoseconds(mach_absolute_time() - wl15_ready_time, &ns);
	printf("FUXI WLAN: W15 elapsed %llu ms since FW_READY: %s\n", ns / 1000000ull, step);
}

static const char *
wl15_step(bool reply)
{
	return wl12_stage == WL15_INI ? (reply ? "INI reply" : "INI send") :
	       wl12_stage == WL15_CFG ? (reply ? "CFG reply" : "CFG send") :
	       (reply ? "MODE reply" : "MODE send");
}

static uint16_t wl_get16(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }
static uint32_t wl_get32(const uint8_t *p) { return wl_get16(p) | ((uint32_t)wl_get16(p+2) << 16); }
static void wl_put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void wl_put32(uint8_t *p, uint32_t v) { wl_put16(p, v); wl_put16(p+2, v >> 16); }
static unsigned wl_ring_next(unsigned i, unsigned n) { return (i + 1) % n; }
static struct wl_context *wl_channel(unsigned ch) { return (void *)(wl_contexts.va + ch * sizeof(struct wl_context)); }
static struct wl_tre *wl_ipcr_ring(unsigned off) { return (void *)(wl_ipcr.va + off); }
static void wl_w11_barrier(void) { __asm__ volatile("dsb sy" ::: "memory"); }

/* RAM-only snapshot: channel RP is diagnostic, never used to reclaim TREs.
 * Completion events remain authoritative. Do not repair a live device-owned
 * RP even when it resembles type/ER words (as in try15c/try15d). */
static void
wl16_snapshot(const char *phase)
{
	if (!wl16_variant || !wl_contexts.va || !wl_contexts.nc || !wl_ipcr.va) { return; }
	wl_w11_barrier();
	printf("FUXI WLAN: W16 %s variant %u txn %u msg %#x IPCR entries %u TX/RX bytes %u/%u\n",
	    phase, wl16_variant, wl12_txn, wl12_msg, WL_IPCR_N, WL_IPCR_TXMTU, WL_IPCR_MTU);
	for (unsigned i = 0; i < 2; i++) {
		struct wl_context c = *wl_channel(20+i);
		uint64_t base = wl_ipcr.iova + (i ? WL_IPCR_RXR : WL_IPCR_TXR);
		bool rp_valid = c.rp >= base && c.rp < base+WL_IPCR_N*16 && !((c.rp-base)&15);
		printf("FUXI WLAN: W16 ch %u cfg %#x type %u ER %u base %#llx len %#llx RP %#llx WP %#llx layout-ok %u rp-in-ring %u host-r/w %u/%u xfers %u OOB %u DB_MODE %u last %#llx/%#x/%#x\n",
		    20+i, c.cfg, c.type, c.index, c.base, c.len, c.rp, c.wp,
		    c.type == (i ? 2u : 1u) && c.index == 1 && c.base == base && c.len == WL_IPCR_N*16,
		    rp_valid, i ? wl_rx_read : wl_tx_read, i ? wl_rx_write : wl_tx_write,
		    wl16_xfers[i], wl16_oob[i], wl16_dbmode[i], wl16_last[i].ptr, wl16_last[i].d0, wl16_last[i].d1);
		/* Fixed eleven words, no buffer contents; exposes offset/stride errors. */
		const uint8_t *raw = (const uint8_t *)wl_channel(20+i);
		printf("FUXI WLAN: W16 ch %u context words", 20+i);
		for (unsigned j = 0; j < 11; j++) { printf(" %08x", wl_get32(raw+j*4)); }
		printf("\n");
	}
}

static int
wl_w11_fail(const char *reason)
{
	if (!wl_w11_error) { printf("FUXI WLAN: %s failure: %s; contain, no retry\n", wl12_stage == WL23_SCAN ? "W23" : wl12_stage == WL22_WMI ? "W22" :
	    wl12_stage == WL21_HTC ? "W21" : wl12_stage == WL20_CE ? "W20" : wl_w15 && wl12_stage >= WL15_INI ? "W15" : wl_w12 ? "W12" : "W11", reason); }
	wl_w11_error = 1; wl_w11_ok = false;
	return 1;
}

/* Check live TTBR and pre-mapped context/IPCR pages. W12 target leaves are
 * added separately by wl12_map_target after REQUEST_MEM. */
static int
wl_w11_mapping(void)
{
	const uint64_t mask = 0x0000000ffffff000ull;
	uint64_t root = ((uint64_t)wl_prd(wl_cb_addr(wl_cb)+0x24) << 32) | wl_prd(wl_cb_addr(wl_cb)+0x20);
	if ((root & mask) != wl_pt_pa(0)) { return 1; }
	struct wl_dma *bufs[] = { &wl_contexts, &wl_ipcr };
	for (unsigned b = 0; b < 2; b++) {
		struct wl_dma *d = bufs[b];
		if (!d->va || !d->nc || d->iova < wl_iova_base || d->iova >= wl_iova_end ||
		    d->size > wl_iova_end - d->iova) { return 1; }
		for (vm_size_t off = 0; off < d->size; off += 4096) {
			uint64_t table = root & mask, addr = d->iova + off;
			const unsigned shifts[] = {30, 21, 12};
			for (unsigned l = 0; l < 3; l++) {
				vm_offset_t alias = wl_owned_alias(table, 4096, true);
				if (!alias) { return 1; }
				uint64_t desc = ((uint64_t *)alias)[(addr >> shifts[l]) & 511];
				if ((desc & 3) != 3) { return 1; }
				table = desc & mask;
				if (l == 2 && (table != wl_phys(d->va + off) ||
				    (desc & ~mask) != (WL_DMA_PTE & ~mask))) { return 1; }
			}
		}
	}
	printf("FUXI WLAN: W11 DMA live walk verified, contexts/IPCR Normal-NC, all buffers in owned IOVA window\n");
	return fw_faults ? 1 : 0;
}

/* Doorbells are write-only notifications. Read back the shared context/base
 * registers, never require a doorbell to echo the write. High word first. */
static int
wl_w11_guard(void)
{
	uint32_t st = wl_mrd(0x48), ctrl = wl_mrd(0x38);
	if (!wl_enabled || !wl_owned || fw_faults || st == ~0u ||
	    ((st >> 8) & 255) != 2 || (st & 4) || (ctrl & 2) ||
	    wl_mrd(wl_bhi+0x28) != 2 || wl_mrd(0x18) != wl_chdb ||
	    wl_mrd(0x20) != wl_erdb || (wl_prd(WL_EPCFG+4) & 0x406) != 0x406 ||
	    wl_smmu_fault(0) || fw_faults) { return wl_w11_fail("AMSS/M0, ownership, discovery or access/SMMU fault"); }
	return 0;
}

/* Original opt-in recovery for queued IPCR TREs: re-publish only the already
 * committed WP. OOB/DB_MODE are notifications, not transfer completions.
 * Never derive a doorbell from the suspect shared channel RP. */
static int
wl16_kick(unsigned ch)
{
	unsigned read = ch == 20 ? wl_tx_read : wl_rx_read;
	unsigned write = ch == 20 ? wl_tx_write : wl_rx_write;
	if (!wl16_redrive || wl_w11_stopping || read == write) { return 0; }
	uint64_t wp = wl_ipcr.iova + (ch == 20 ? WL_IPCR_TXR : WL_IPCR_RXR) + write*16;
	if (wl_channel(ch)->wp != wp) { return wl_w11_fail("W16 doorbell WP changed"); }
	wl_w11_barrier(); wl_mwr64(wl_chdb+ch*8, wp);
	return fw_faults ? wl_w11_fail("W16 redrive doorbell access") : 0;
}

static int
wl_ipcr_send(const uint8_t *packet, unsigned bytes)
{
	unsigned i = wl_tx_write, next = wl_ring_next(i, WL_IPCR_N);
	if (!bytes || bytes > WL_IPCR_TXMTU || next == wl_tx_read || wl_w11_guard()) {
		return wl_w11_fail("TX length/full/transport guard");
	}
	uint64_t addr = wl_ipcr.iova + WL_IPCR_TXBUF + i * WL_IPCR_TXMTU;
	memcpy((void *)(wl_ipcr.va + WL_IPCR_TXBUF + i * WL_IPCR_TXMTU), packet, bytes);
	wl_ipcr_ring(WL_IPCR_TXR)[i] = (struct wl_tre){addr, bytes, (2u << 16) | (1u << 9)};
	wl_tx_write = next;
	wl_w11_barrier();
	wl_channel(20)->wp = wl_ipcr.iova + WL_IPCR_TXR + next * 16;
	wl_w11_barrier();
	wl_mwr64(wl_chdb + 20 * 8, wl_channel(20)->wp);
	return fw_faults ? wl_w11_fail("TX doorbell access") : 0;
}

static int
wl_qrtr_send(unsigned type, uint32_t node, uint32_t port, const uint8_t *data, unsigned len)
{
	static uint8_t packet[8192]; /* one boot thread; copied before publication */
	bzero(packet, sizeof(packet));
	unsigned padded = (len + 3) & ~3u;
	if (len > sizeof(packet)-32 || padded > sizeof(packet)-32) { return wl_w11_fail("QRTR TX bounds"); }
	/* QRTR v1, eight little-endian u32 words; size excludes header/padding. */
	wl_put32(packet, 1); wl_put32(packet+4, type);
	wl_put32(packet+8, WL_QRTR_LOCAL);
	wl_put32(packet+12, type == 1 ? WL_QRTR_PORT : WL_QRTR_CTRL);
	wl_put32(packet+20, len); wl_put32(packet+24, node); wl_put32(packet+28, port);
	if (len) { memcpy(packet+32, data, len); }
	return wl_ipcr_send(packet, 32+padded);
}

static int
wl_qrtr_control(unsigned type, uint32_t node, uint32_t a, uint32_t b)
{
	uint8_t data[20] = {0};
	wl_put32(data, type); wl_put32(data+4, a); wl_put32(data+8, b);
	return wl_qrtr_send(type, node, WL_QRTR_CTRL, data, sizeof(data));
}

/* QMI nested timestamp string has a one-byte length inside TLV 0x13;
 * top-level build-id TLV 0x14 uses the TLV length itself. Sanitize log text. */
static void
wl_qmi_text(char *out, const uint8_t *p, unsigned len)
{
	for (unsigned i = 0; i < len; i++) { out[i] = p[i] >= 32 && p[i] < 127 ? p[i] : '?'; }
	out[len] = 0;
}

static int
wl_qmi_cap(const uint8_t *p, unsigned len)
{
	if (len < 7 || wl_get16(p+5) != len-7) { return wl_w11_fail("QMI header length"); }
	if (p[0] != 2 || wl_get16(p+1) != (wl_w12 ? wl12_txn : 1) || wl_get16(p+3) != 0x24) {
		/* Unsolicited indications and unrelated transactions are not replies. */
		return 0;
	}
	uint32_t seen = 0, chip = 0, family = 0, board = 0, soc = 0, version = 0;
	char stamp[33] = "absent", build[129] = "absent";
	uint32_t macs = 0, mv = 0, hz = 0, otp = 0, cal_timeout = 0, chain = 0;
	uint64_t caps = 0, dev_start[4] = {0}, dev_size[4] = {0};
	for (unsigned off = 7; off < len;) {
		if (len-off < 3) { return wl_w11_fail("QMI truncated TLV header"); }
		unsigned type = p[off], n = wl_get16(p+off+1); off += 3;
		if (n > len-off) { return wl_w11_fail("QMI truncated TLV value"); }
		unsigned bit = type == 2 ? 1 : (type >= 0x10 && type <= 0x29 ? 2u << (type-0x10) : 0);
		if (bit && (seen & bit)) { return wl_w11_fail("QMI duplicate capability TLV"); }
		seen |= bit;
		const uint8_t *v = p+off;
		switch (type) {
		case 2:
			if (n != 4) { return wl_w11_fail("QMI result TLV size"); }
			printf("FUXI WLAN: %s WLFW reply txn %u msg 0x24 result %u error %u%s\n",
			    wl_w12 ? "W12" : "W11", wl_w12 ? wl12_txn : 1, wl_get16(v), wl_get16(v+2),
			    wl_get16(v+2) == 90 ? " (INCOMPATIBLE_STATE)" : "");
			if (wl_get16(v) || wl_get16(v+2)) { return wl_w11_fail("WLFW capability rejected"); }
			break;
		case 0x10:
			if (n != 8) { return wl_w11_fail("QMI chip TLV size"); }
			chip = wl_get32(v); family = wl_get32(v+4); break;
		case 0x11:
			if (n != 4) { return wl_w11_fail("QMI board TLV size"); }
			board = wl_get32(v); break;
		case 0x12:
			if (n != 4) { return wl_w11_fail("QMI soc TLV size"); }
			soc = wl_get32(v); break;
		case 0x13:
			if (n < 5 || v[4] > 32 || n != 5u+v[4]) { return wl_w11_fail("QMI version/timestamp bounds"); }
			version = wl_get32(v); wl_qmi_text(stamp, v+5, v[4]); break;
		case 0x14:
			if (n > 128) { return wl_w11_fail("QMI build-id bounds"); }
			wl_qmi_text(build, v, n); break;
		case 0x15:
			if (n != 1) { return wl_w11_fail("QMI num MACs TLV size"); }
			macs = v[0]; break;
		case 0x16: case 0x17: case 0x18: case 0x19: case 0x1b:
			if (n != 4) { return wl_w11_fail("QMI scalar capability TLV size"); }
			if (type == 0x16) { mv = wl_get32(v); }
			if (type == 0x17) { hz = wl_get32(v); }
			if (type == 0x18) { otp = wl_get32(v); }
			if (type == 0x19) { cal_timeout = wl_get32(v); }
			if (type == 0x1b) { chain = wl_get32(v); }
			break;
		case 0x1a:
			if (n != 8) { return wl_w11_fail("QMI fw caps TLV size"); }
			caps = wl_get32(v) | ((uint64_t)wl_get32(v+4) << 32); break;
		case 0x1c:
			/* ath12k declares a STATIC array of four u64 start/size pairs. */
			if (n != 64) { return wl_w11_fail("QMI device memory TLV size"); }
			for (unsigned i = 0; i < 4; i++) {
				dev_start[i] = wl_get32(v+16*i) | ((uint64_t)wl_get32(v+16*i+4) << 32);
				dev_size[i] = wl_get32(v+16*i+8) | ((uint64_t)wl_get32(v+16*i+12) << 32);
			}
			break;
		case 0x1e:
			if (n != 4) { return wl_w11_fail("hang_data_addr_offset size"); }
			printf("FUXI WLAN: W14 cap hang_data_addr_offset %#x (reported only)\n", wl_get32(v)); break;
		case 0x1f:
			if (n != 2) { return wl_w11_fail("hang_data_length size"); }
			printf("FUXI WLAN: W14 cap hang_data_length %u (reported only)\n", wl_get16(v)); break;
		case 0x22:
			/* Vendor protocol fact: u8 count, then 32-byte name + four u32s.
			 * Original decoder; no cnss2 implementation/EI tables imported. */
			if (!n || v[0] > 16 || n != 1u+48u*v[0]) { return wl_w11_fail("ol_cpr_cfg bounds"); }
			printf("FUXI WLAN: W14 cap ol_cpr_cfg pmu_param_len %u (reported only; no power writes)\n", v[0]);
			for (unsigned i = 0; i < v[0]; i++) {
				const uint8_t *q = v+1+48*i; char name[33]; unsigned k = 0;
				while (k < 32 && q[k]) { k++; }
				wl_qmi_text(name, q, k);
				printf("FUXI WLAN: W14 ol_cpr_cfg %u pin %s wake_volt_valid %u wake_volt %u sleep_volt_valid %u sleep_volt %u\n",
				    i, name, wl_get32(q+32), wl_get32(q+36), wl_get32(q+40), wl_get32(q+44));
			}
			break;
		case 0x26:
			if (n != 4) { return wl_w11_fail("he_channel_width_cap size"); }
			printf("FUXI WLAN: W14 cap he_channel_width_cap %#x\n", wl_get32(v)); break;
		default:
			if (wl_w12) { printf("FUXI WLAN: W12 cap unknown TLV %#x bytes %u\n", type, n); }
			break;
		}
		off += n;
	}
	if (!(seen & 1)) { return wl_w11_fail("QMI reply missing result"); }
	printf("FUXI WLAN: %s valid %#x chip id %#x chip family %#x board id %#x soc id %#x firmware %#x timestamp %s build %s\n",
	    wl_w12 ? "W12 cap:" : "W11 WLFW cap:", seen, chip, family, board, soc, version, stamp, build);
	if (wl_w12) {
		printf("FUXI WLAN: W12 cap: num macs %u voltage mv %u time freq hz %u otp %#x eeprom cal timeout %u fw caps %#llx rd_card_chain_cap %u (presence in valid mask)\n",
		    macs, mv, hz, otp, cal_timeout, caps, chain);
		if (seen & (2u << 12)) {
			for (unsigned i = 0; i < 4; i++) {
				printf("FUXI WLAN: W12 cap: device memory %u start %#llx size %#llx (reported only)\n", i, dev_start[i], dev_size[i]);
			}
		}
	}
	wl14_board = board; wl14_board_valid = !!(seen & (2u << 1));
	/* C2f: the CMEM region (dev_mem[0], ATH12K_QMI_DEVMEM_CMEM_INDEX) cookie
	 * conversion needs; low 32 bits as the window accessor takes. Kept for W29
	 * only; refused there if absent. */
	if ((seen & (2u << 12)) && dev_size[0]) { wl_cmem_base = (uint32_t)dev_start[0]; wl_cmem_valid = true; }
	wl_cap_done = true;
	return 0;
}

static int
wl_qrtr_receive(const uint8_t *p, unsigned bytes)
{
	unsigned hdr, type, confirm, len;
	uint32_t src, sport, dst, dport;
	if (bytes >= 32 && wl_get32(p) == 1) {
		hdr = 32; type = wl_get32(p+4); src = wl_get32(p+8); sport = wl_get32(p+12);
		confirm = wl_get32(p+16); len = wl_get32(p+20); dst = wl_get32(p+24); dport = wl_get32(p+28);
	} else if (bytes >= 16 && p[0] == 3) {
		hdr = 16; type = p[1]; confirm = p[2] & 1;
		if (p[3]) { return wl_w11_fail("QRTR optional header unsupported"); }
		if (p[2] & ~1u) { return wl_w11_fail("QRTR compact flags"); }
		len = wl_get32(p+4); src = wl_get16(p+8); sport = wl_get16(p+10);
		dst = wl_get16(p+12); dport = wl_get16(p+14);
		if (sport == 0xffff) { sport = WL_QRTR_CTRL; }
		if (dport == 0xffff) { dport = WL_QRTR_CTRL; }
		if (dst == 0xffff) { dst = ~0u; }
	} else { return wl_w11_fail("QRTR version/header"); }
	if (hdr > bytes || len > WL_IPCR_MTU-hdr || len > bytes-hdr ||
	    ((len+3)&~3u) != bytes-hdr || confirm > 1 || !src || src == WL_QRTR_LOCAL || src == ~0u) {
		return wl_w11_fail("QRTR length/source bounds");
	}
	if ((dst != WL_QRTR_LOCAL && dst != ~0u) || (dport != WL_QRTR_CTRL && dport != WL_QRTR_PORT)) { return 0; }
	const uint8_t *data = p+hdr;
	if (type != 1) {
		if (len != 20 || wl_get32(data) != type || dport != WL_QRTR_CTRL) { return wl_w11_fail("QRTR control length/type/port"); }
		if (type == 2) {
			if (!wl_hello) {
				wl_hello = true; wl_peer_node = src;
				printf("FUXI WLAN: W11 QRTR hello peer node %u\n", src);
				if (wl_qrtr_control(2, src, 0, 0)) { return 1; }
			}
		} else if (type == 3 || type == 8) {
			if (src == wl_peer_node || src == wl_server_node) { return wl_w11_fail("QRTR BYE/EXIT"); }
		} else if (type == 4 || type == 5) {
			uint32_t service = wl_get32(data+4), instance = wl_get32(data+8);
			uint32_t node = wl_get32(data+12), port = wl_get32(data+16);
			if (service == 0x45 && (instance & 255) == 1 && node && node != WL_QRTR_LOCAL &&
			    node != ~0u && port && port < WL_QRTR_CTRL) {
				if (type == 5 && node == wl_server_node && port == wl_server_port) { return wl_w11_fail("WLFW server removed"); }
				if (type == 4 && !wl_server_port) {
					wl_server_node = node; wl_server_port = port;
					printf("FUXI WLAN: W11 WLFW server node %u port %u instance %#x\n", node, port, instance);
				}
			}
		}
	} else if (dport == WL_QRTR_PORT && src == wl_server_node && sport == wl_server_port) {
		if (wl_w12) {
			if (wl_qmi_w12(data, len)) { return 1; }
		} else if (wl_cap_sent && !wl_cap_done && wl_qmi_cap(data, len)) { return 1; }
	}
	if (confirm) { return wl_qrtr_control(7, src, WL_QRTR_LOCAL, dport); }
	return 0;
}

/* Device controls ER context RP (next event to produce), host controls WP
 * (last free element). Keep a private consumer; return one credit per event.
 * Process at most the ring's capacity per pass, even under a hostile producer. */
static int
wl_w11_events(void)
{
	if (wl_w11_guard()) { return 1; }
	for (unsigned r = 0; r < 2; r++) {
		struct wl_context *ec = (void *)(wl_contexts.va + WL_ECTX + r*sizeof(*ec));
		unsigned off = r ? WL_ER1 : WL_ER0, n = r ? 256 : 32;
		uint64_t base = wl_contexts.iova + off;
		for (unsigned budget = 0; budget < n; budget++) {
			wl_w11_barrier();
			uint64_t producer = ec->rp;
			if (producer < base || producer >= base+n*16 || ((producer-base)&15)) { return wl_w11_fail("event RP outside ring/alignment"); }
			unsigned i = wl_er_read[r];
			if (producer == base+i*16) { break; }
			wl_w11_barrier();
			struct wl_tre ev = ((struct wl_tre *)(wl_contexts.va+off))[i];
			unsigned type = (ev.d1 >> 16) & 255, ch = ev.d1 >> 24, cc = ev.d0 >> 24;
			unsigned bytes = ev.d0 & 0xffff;
			if (type == 0x21) {
				if (!wl_cmd_pending || ev.ptr != wl_cmd_pending || cc != 1) { return wl_w11_fail("command completion pointer/code"); }
				struct wl_tre cmd = ((struct wl_tre *)(wl_contexts.va+WL_CR))[wl_cmd_write];
				if (cmd.d1 != ((wl_cmd_channel << 24) | (wl_cmd_opcode << 16))) { return wl_w11_fail("command TRE changed"); }
				wl_cmd_pending = 0; wl_cmd_done = true;
			} else if (type == 0x22) {
				if (ch != 20 && ch != 21) { return wl_w11_fail("transfer event channel"); }
				if (wl16_variant) {
					unsigned c = ch-20;
					wl16_last[c] = ev;
					if (cc == 5) { wl16_oob[c]++; }
					else if (cc == 6) { wl16_dbmode[c]++; }
					else { wl16_xfers[c]++; }
					if ((cc == 5 || cc == 6) && wl16_oob[c]+wl16_dbmode[c] <= 4) {
						printf("FUXI WLAN: W16 ch %u notification %u redrive %u host-r/w %u/%u\n",
						    ch, cc, wl16_redrive, ch == 20 ? wl_tx_read : wl_rx_read,
						    ch == 20 ? wl_tx_write : wl_rx_write);
					}
				}
				/* Legacy ignores OOB/DB_MODE; W16 optionally re-rings pending
				 * TREs without reclaiming/reposting or changing any channel context. */
				if ((cc == 5 || cc == 6) && wl16_kick(ch)) { return 1; }
				if (cc != 5 && cc != 6) {
					if (cc != 2) { return wl_w11_fail("transfer completion code (requires EOT)"); }
					unsigned ri = ch == 20 ? wl_tx_read : wl_rx_read;
					unsigned roff = ch == 20 ? WL_IPCR_TXR : WL_IPCR_RXR;
					if (ev.ptr != wl_ipcr.iova+roff+ri*16 || (ch == 20 && ri == wl_tx_write)) { return wl_w11_fail("transfer completion pointer/order"); }
					struct wl_tre tre = wl_ipcr_ring(roff)[ri];
					if (bytes > (ch == 20 ? WL_IPCR_TXMTU : WL_IPCR_MTU) || bytes > (tre.d0 & 0xffff)) { return wl_w11_fail("transfer completion length"); }
					if (ch == 20) { wl_tx_read = wl_ring_next(ri, WL_IPCR_N); }
					else {
						uint64_t lo = wl_ipcr.iova + WL_IPCR_RXBUF;
						if (tre.ptr < lo || tre.ptr >= lo+WL_IPCR_N*WL_IPCR_MTU ||
						    ((tre.ptr-lo) % WL_IPCR_MTU) || !bytes) { return wl_w11_fail("RX buffer ownership/length"); }
						if (!wl_w11_stopping && wl_qrtr_receive((const uint8_t *)(wl_ipcr.va+(tre.ptr-wl_ipcr.iova)), bytes)) { return 1; }
						wl_rx_read = wl_ring_next(ri, WL_IPCR_N);
						if (!wl_w11_stopping) {
							wl_ipcr_ring(WL_IPCR_RXR)[wl_rx_write] = (struct wl_tre){tre.ptr, WL_IPCR_MTU, (2u<<16)|(1u<<9)};
							wl_rx_write = wl_ring_next(wl_rx_write, WL_IPCR_N);
							wl_w11_barrier(); wl_channel(21)->wp = wl_ipcr.iova + WL_IPCR_RXR + wl_rx_write*16;
							wl_w11_barrier(); wl_mwr64(wl_chdb+21*8, wl_channel(21)->wp);
						}
					}
				}
			} else if (type == 0x20) {
				if (((ev.d0 >> 24) & 255) != 2) { return wl_w11_fail("MHI state event outside M0"); }
			} else if (type == 0x40) {
				/* Boot events may predate AMSS. Live EE is checked every pass. */
				unsigned ee = ev.d0 >> 24;
				if (ee != 0 && ee != 1 && ee != 2) { return wl_w11_fail("MHI error/unsupported EE event"); }
			} else { return wl_w11_fail("unsupported MHI event type"); }
			wl_er_read[r] = wl_ring_next(i, n);
			ec->wp = base + ((wl_er_read[r]+n-1)%n)*16;
			wl_w11_barrier(); wl_mwr64(wl_erdb+r*8, ec->wp);
			if (fw_faults) { return wl_w11_fail("event credit doorbell access"); }
		}
	}
	return 0;
}

/* All waits sleep 10 ms, have a 2-second deadline, and perform bounded work.
 * No command retries; a timed-out pending command is left for MHI reset. */
static int
wl_w11_wait(unsigned what, const char *stage)
{
	for (unsigned ms = 0; ms <= 2000; ms += 10) {
		if (wl_w11_events()) { return 1; }
		bool done = what == 0 ? wl_cmd_done : what == 1 ? (wl_hello && wl_server_port) :
		    what == 2 ? wl_cap_done : wl_tx_read == wl_tx_write;
		if (done) { return 0; }
		if (ms < 2000) { wl_sleep(10); }
	}
	printf("FUXI WLAN: W11 timeout %s (2000 ms); contain, no retry\n", stage);
	return 2;
}

static int
wl_w11_command(unsigned ch, unsigned opcode)
{
	if (wl_cmd_pending || wl_w11_guard()) { return wl_w11_fail("command pending/transport guard"); }
	struct wl_context *c = (void *)(wl_contexts.va+WL_CCTX);
	uint64_t ptr = wl_contexts.iova + WL_CR + wl_cmd_write*16;
	((struct wl_tre *)(wl_contexts.va+WL_CR))[wl_cmd_write] = (struct wl_tre){0, 0, (ch<<24)|(opcode<<16)};
	wl_cmd_channel = ch; wl_cmd_opcode = opcode; wl_cmd_pending = ptr; wl_cmd_done = false;
	wl_w11_barrier(); c->wp = wl_contexts.iova+WL_CR+wl_ring_next(wl_cmd_write, 128)*16;
	wl_w11_barrier(); wl_mwr64(0x70, c->wp);
	if (fw_faults) { return wl_w11_fail("command doorbell access"); }
	int e = wl_w11_wait(0, opcode == 18 ? "START_CHAN completion" : "STOP_CHAN completion");
	if (!e) {
		printf("FUXI WLAN: W11 command %#x chan %u completed\n", opcode, ch);
		wl_cmd_write = wl_ring_next(wl_cmd_write, 128);
	}
	return e;
}

static void
wl_w11_stop(void)
{
	if (!wl_chan_attempts) { return; }
	if (wl_w11_ok && !fw_faults && !wl_cmd_pending) {
		if (!wl_qrtr_control(3, ~0u, 0, 0)) {
			(void)wl_w11_wait(3, "QRTR BYE TX drain");
			printf("FUXI WLAN: W11 QRTR BYE sent before containment\n");
		}
	}
	wl_w11_ok = false; wl_w11_stopping = true;
	/* A pending/invalid command must not be overwritten to attempt STOP.
	 * Regardless of STOP success, caller continues reset/BME-off/PERST. */
	for (unsigned i = 0; i < 2; i++) {
		if (!(wl_chan_attempts & (1u << i))) { continue; }
		int e = wl_cmd_pending ? 1 : wl_w11_command(20+i, 17);
		printf("FUXI WLAN: W11 stop chan %u %s; MHI reset/PERST follows\n", 20+i, e ? "unconfirmed" : "completed");
	}
	wl_chan_attempts = 0;
}

static int
wl_w11_run(void)
{
	if (wl_w11_attempted) { return wl_w11_fail("one conversation per boot"); }
	wl_w11_attempted = true; wl_w11_ok = false; wl_w11_error = 0; wl_w11_stopping = false;
	wl_cmd_pending = 0; wl_cmd_write = wl_chan_attempts = 0;
	bzero(wl16_xfers, sizeof(wl16_xfers)); bzero(wl16_oob, sizeof(wl16_oob));
	bzero(wl16_dbmode, sizeof(wl16_dbmode)); bzero(wl16_last, sizeof(wl16_last));
	wl_tx_read = wl_tx_write = wl_rx_read = 0; wl_rx_write = WL_IPCR_N-1;
	wl_er_read[0] = wl_er_read[1] = 0;
	wl_server_node = wl_server_port = wl_peer_node = 0; wl_hello = wl_cap_sent = wl_cap_done = false;
	wl12_stage = WL12_DISCOVERY; wl12_txn = wl12_msg = 0;
	wl12_reply = wl12_requested = wl12_ready = false;
	wl13_delayed = wl13_negotiated = false;
	wl14_ready = wl14_init_done = wl14_board_valid = false; wl14_board = 0;
	wl_chdb = wl_mrd(0x18); wl_erdb = wl_mrd(0x20);
	if (!wl_qrtr || !wl_amss_reached || !wl_contexts_published || wl_w11_guard() ||
	    (wl_chdb & 7) || wl_chdb < 0x100 || wl_chdb > 0x3c00 ||
	    (wl_erdb & 7) || wl_erdb < 0x100 || wl_erdb > 0x3ff0 ||
	    (wl_erdb < wl_chdb+128*8 && wl_erdb+16 > wl_chdb) || wl_w11_mapping()) {
		return wl_w11_fail("preflight/mapping");
	}
	if (wl_mrd(0x58) != (uint32_t)wl_contexts.iova || wl_mrd(0x5c) ||
	    wl_mrd(0x60) != (uint32_t)(wl_contexts.iova+WL_ECTX) || wl_mrd(0x64) ||
	    wl_mrd(0x68) != (uint32_t)(wl_contexts.iova+WL_CCTX) || wl_mrd(0x6c) || fw_faults) {
		return wl_w11_fail("context base readback");
	}
	struct wl_context *cc = (void *)(wl_contexts.va+WL_CCTX);
	if (cc->base != wl_contexts.iova+WL_CR || cc->len != 2048 || cc->rp != cc->base || cc->wp != cc->base) {
		return wl_w11_fail("command context not empty");
	}
	for (unsigned i = 0; i < 2; i++) {
		struct wl_context *ec = (void *)(wl_contexts.va+WL_ECTX+i*sizeof(*ec));
		uint64_t base = wl_contexts.iova+(i ? WL_ER1 : WL_ER0), len = i ? 4096 : 512;
		if (ec->base != base || ec->len != len || ec->type != 1 || ec->wp != base+len-16) {
			return wl_w11_fail("event context layout/credit changed");
		}
	}
	if (wl_w11_events()) { return 1; } /* validate/drain queued boot events */
	wl_mwr64(wl_chdb+127*8, 1); /* hold device wake throughout conversation/keep */
	if (fw_faults) { return wl_w11_fail("device wake doorbell access"); }
	for (unsigned i = 0; i < 2; i++) {
		struct wl_context *c = wl_channel(20+i);
		if (c->cfg || c->type || c->index || c->base || c->len || c->rp || c->wp) { return wl_w11_fail("IPCR context not disabled/empty"); }
		c->cfg = 1 | (2u << 8); /* ENABLED, burst disabled */
		c->type = i ? 2 : 1; c->index = 1;
		c->base = c->rp = c->wp = wl_ipcr.iova + (i ? WL_IPCR_RXR : WL_IPCR_TXR);
		c->len = WL_IPCR_N*16;
	}
	wl_w11_barrier();
	for (unsigned i = 0; i < 2; i++) {
		wl_chan_attempts |= 1u << i; /* includes a fault on START doorbell */
		int e = wl_w11_command(20+i, 18);
		if (e) { return e; }
	}
	printf("FUXI WLAN: W11 chan 20/21 started\n");
	for (unsigned i = 0; i < WL_IPCR_N-1; i++) {
		wl_ipcr_ring(WL_IPCR_RXR)[i] = (struct wl_tre){wl_ipcr.iova+WL_IPCR_RXBUF+i*WL_IPCR_MTU, WL_IPCR_MTU, (2u<<16)|(1u<<9)};
	}
	wl_w11_barrier(); wl_channel(21)->wp = wl_ipcr.iova+WL_IPCR_RXR+wl_rx_write*16;
	wl_w11_barrier(); wl_mwr64(wl_chdb+21*8, wl_channel(21)->wp);
	if (fw_faults) { return wl_w11_fail("RX doorbell access"); }
	printf("FUXI WLAN: W11 RX posted %u buffers of %u bytes; polling RAM-sink MSI\n", WL_IPCR_N-1, WL_IPCR_MTU);
	wl16_snapshot("channels started/RX posted");
	if (wl_qrtr_control(2, ~0u, 0, 0) || wl_qrtr_control(10, ~0u, 0x45, 1)) { return 1; }
	printf("FUXI WLAN: W11 QRTR hello sent; lookup service 0x45 version 1\n");
	int e = wl_w11_wait(1, "QRTR HELLO/WLFW server");
	if (e) { return e; }
	if (wl_w12) { return wl_w12_run(); }
	uint8_t cap[7] = {0}; /* empty CAP_REQ TLV list; no QMUX over QRTR */
	wl_put16(cap+1, 1); wl_put16(cap+3, 0x24); wl_put16(cap+5, 0);
	printf("FUXI WLAN: W11 WLFW capability request txn 1 msg 0x24\n");
	wl_cap_sent = true;
	if (wl_qrtr_send(1, wl_server_node, wl_server_port, cap, sizeof(cap))) { return 1; }
	e = wl_w11_wait(2, "WLFW capability reply");
	if (!e) { e = wl_w11_wait(3, "TX completion drain"); }
	if (e || wl_w11_guard()) { return e ? e : 1; }
	wl_w11_ok = true;
	printf("FUXI WLAN: W11 conversation complete; %s\n", wl_keep ? "keep requested, channels/DMA/wake retained (polling ended)" : "stop/contain");
	return 0;
}

/* W12: independently encoded WLFW setup using wire facts and WCN7850 values
 * from ath12k qmi.h/qmi.c + hw.c, Linux v6.6 (BSD-3-Clause-Clear).
 * Credit: The Linux Foundation (2018-2021), Qualcomm Innovation Center, Inc.
 * (2021-2022). No qmi_helpers, Linux core routines or tables are imported.
 * Ordering, per-field provenance and deliberate stop: docs-lanes/wlan-w12.md. */
static uint64_t
wl_get64(const uint8_t *p)
{
	return wl_get32(p) | ((uint64_t)wl_get32(p+4) << 32);
}

static void
wl_put64(uint8_t *p, uint64_t v)
{
	wl_put32(p, (uint32_t)v); wl_put32(p+4, (uint32_t)(v >> 32));
}

/* W14 BDF requests need 6144 data bytes plus TLVs/header. Static encoders
 * avoid growing the boot thread stack; only one request is outstanding. */
#define WL12_QMI_MAX (WL14_SEG + 64u)
struct wl12_encoder { uint8_t data[WL12_QMI_MAX]; unsigned len; };
static int
wl12_tlv(struct wl12_encoder *e, unsigned type, uint64_t value, unsigned bytes)
{
	if (bytes > 8 || e->len > sizeof(e->data)-3-bytes) { return wl_w11_fail("QMI encode bounds"); }
	uint8_t *v = e->data+e->len;
	v[0] = type; wl_put16(v+1, bytes);
	for (unsigned i = 0; i < bytes; i++) { v[3+i] = (uint8_t)(value >> (8*i)); }
	e->len += 3+bytes;
	return 0;
}

static int
wl12_send(struct wl12_encoder *e, unsigned msg, unsigned stage)
{
	if (e->len < 7 || e->len > sizeof(e->data) || wl12_txn >= (unsigned)((wl_w14 ? 512 : wl_w13 ? 5 : 4) + (wl12_mlo_variant == 1)) || wl_w11_guard()) {
		return wl_w11_fail("QMI request bounds/one attempt");
	}
	wl12_stage = stage; wl12_msg = msg; wl12_txn++; wl12_reply = false;
	if (wl_w15 && stage >= WL15_INI) { wl15_elapsed(wl15_step(false)); }
	e->data[0] = 0; wl_put16(e->data+1, wl12_txn);
	wl_put16(e->data+3, msg); wl_put16(e->data+5, e->len-7);
	printf("FUXI WLAN: %s send txn %u msg %#x stage %u\n", wl_w15 && stage >= WL15_INI ? "W15" : "W12", wl12_txn, msg, stage);
	int error = wl_qrtr_send(1, wl_server_node, wl_server_port, e->data, e->len);
	if (!error && stage == WL15_MODE) { wl16_snapshot("MODE queued"); }
	return error;
}

/* Adapted wire facts/behavior from Linux v6.12 ath12k qmi.c/qmi.h:
 * ath12k_qmi_phy_cap_send(), ath12k_host_cap_parse_mlo(),
 * ath12k_qmi_host_cap_send(); core.c ath12k_core_alloc(), hw.c WCN7850.
 * BSD-3-Clause-Clear. Copyright (c) 2018-2021 The Linux Foundation;
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc.
 * Independent bounded codec; no Linux QMI helpers/native packing.
 * PHY board_id is reported only (ath12k does not use it for BDF selection). */
static int
wl12_phy_reply(const uint8_t *p, unsigned len)
{
	unsigned seen = 0, num = 0, support = 0, result = 0, error = 0;
	uint32_t board = 0;
	if (len < 7 || wl_get16(p+5) != len-7) { goto malformed; }
	for (unsigned off = 7; off < len;) {
		if (len-off < 3) { goto malformed; }
		unsigned type = p[off], n = wl_get16(p+off+1); off += 3;
		if (n > len-off) { goto malformed; }
		unsigned bit = type == 2 ? 1 : type == 0x10 ? 2 : type == 0x11 ? 4 : type == 0x13 ? 8 : 0;
		if (bit) {
			if ((seen & bit) || n != (bit == 1 || bit == 4 ? 4u : 1u)) { goto malformed; }
			seen |= bit;
			if (bit == 1) { result = wl_get16(p+off); error = wl_get16(p+off+2); }
			else if (bit == 2) { num = p[off]; }
			else if (bit == 4) { board = wl_get32(p+off); }
			else { support = p[off]; }
		} else if (type == 0x17 && (wl17_bits & WL17_PHY17)) {
			wl17_phy17_parse(p+off, n);
			printf("FUXI WLAN: P17 phy cap TLV 0x17 len %u value %#llx\n", wl17_phy17_len, wl17_phy17_val);
		} else { printf("FUXI WLAN: W12 phy cap unknown TLV %#x bytes %u\n", type, n); }
		off += n;
	}
	if (!(seen & 1)) { goto malformed; }
	printf("FUXI WLAN: W12 phy cap result %u error %u num_phy valid %u value %u board_id valid %u value %#x single_chip_mlo_support valid %u value %u\n",
	    result, error, !!(seen & 2), num, !!(seen & 4), board, !!(seen & 8), support);
	/* Match ath12k's result check and order: support applies before the
	 * missing-num_phy fallback. Absent support retains initial support. */
	if (!result) {
		if (seen & 8) { wl12_mlo_enabled = support != 0; }
		if (seen & 2) { wl12_num_phy = num; wl12_reply = true; return 0; }
	}
	printf("FUXI WLAN: W12 phy cap fallback num_phy 2 (%s)\n", result ? "rejected" : "absent num_phy");
	wl12_reply = true;
	return 0;
malformed:
	/* ath12k treats decode failure like an unavailable PHY response. Do
	 * not apply partially decoded fields; close this optional transaction. */
	printf("FUXI WLAN: W12 phy cap fallback num_phy 2 (malformed reply)\n");
	wl12_reply = true;
	return 0;
}

static int
wl12_host_cap(struct wl12_encoder *e)
{
	/* ath12k_qmi_host_cap_send: fresh boot cal_done=0, memory mode=0,
	 * internal clock bits 1/2/3, hw_params CNSS feature bits 3/4. */
	if (wl12_tlv(e, 0x10, 1, 4) || wl12_tlv(e, 0x13, 0x0e, 1) ||
	    wl12_tlv(e, 0x14, 1, 1) || wl12_tlv(e, 0x16, 1, 1) ||
	    wl12_tlv(e, 0x17, 1, 1) || wl12_tlv(e, 0x1a, 0, 1) ||
	    wl12_tlv(e, 0x1c, 0, 1)) { return 1; }
	int links = wl17_mlo_links(wl12_mlo_variant, wl17_bits, wl12_mlo_enabled, wl12_num_phy);
	bool mlo = links != 0;
	if (mlo) {
		/* Upstream's arrays hold TWO links. Refuse unsupported counts
		 * instead of reproducing its unchecked array writes for 3..254
		 * (P17 MLOALL clamps to two instead). */
		if (links < 0) { return wl_w11_fail("PHY_CAP num_phy exceeds two-link EI capacity"); }
		if (wl12_tlv(e, 0x21, 1, 1) || wl12_tlv(e, 0x22, 0, 2) ||
		    wl12_tlv(e, 0x23, 0, 1) || wl12_tlv(e, 0x24, 32, 2) ||
		    wl12_tlv(e, 0x25, 1, 1)) { return 1; }
		/* EI STATIC_ARRAY: three chips, each chip_id, num_local_links,
		 * hw_link_id[2], valid_mlo_link_id[2]. No count/valid prefixes. */
		if (e->len > sizeof(e->data)-21) { return wl_w11_fail("HOST_CAP MLO bounds"); }
		e->data[e->len++] = 0x26; wl_put16(e->data+e->len, 18); e->len += 2;
		uint8_t info[18] = {0}; info[1] = (uint8_t)links;
		for (int i = 0; i < links; i++) { info[2+i] = (uint8_t)i; info[4+i] = 1; }
		memcpy(e->data+e->len, info, sizeof(info)); e->len += sizeof(info);
	}
	if (wl12_mlo_variant || wl17_bits) {
		printf("FUXI WLAN: W12 host cap MLO TLVs %s num_phy %u\n", mlo ? "present" : "omitted", wl12_num_phy);
	}
	if (wl12_tlv(e, 0x27, 0x18, 8)) { return 1; }
	if (wl17_bits & WL17_HOSTMEM) {
		printf("FUXI WLAN: P17 host cap vendor TLV 0x33 host-memory-on-demand = 1 appended after 0x27\n");
		return wl12_tlv(e, 0x33, 1, 1);
	}
	return 0;
}

/* W15 variable arrays: one u8 count followed by explicitly LE u32 fields.
 * Native structure padding and optional-valid flags never go on the wire. */
static int
wl15_array(struct wl12_encoder *e, unsigned type, const void *words,
    unsigned count, unsigned fields)
{
	if (!fields || fields > 5 || count > 60 || e->len > sizeof(e->data) ||
	    4+count*fields*4 > sizeof(e->data)-e->len) { return wl_w11_fail("W15 array bounds"); }
	uint8_t *p = e->data+e->len;
	p[0] = type; wl_put16(p+1, 1+count*fields*4); p[3] = count;
	for (unsigned i = 0; i < count*fields; i++) {
		uint32_t word; memcpy(&word, (const uint8_t *)words+4*i, sizeof(word));
		wl_put32(p+4+4*i, word);
	}
	e->len += 4+count*fields*4;
	return 0;
}

/* The Clear BSD License (for the ath12k adaptations below):
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

/* Adapted from Linux ath12k hw.c (BSD-3-Clause-Clear), v6.12:
 * ath12k_target_ce_config_wlan_wcn7850 and
 * ath12k_target_service_to_ce_map_wlan_wcn7850; ce.h directions/flags,
 * htc.h service IDs. Copyright (c) 2018-2021 The Linux Foundation;
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc.
 * qmi.c qmi_wlanfw_ce_tgt_pipe_cfg_s_v01_ei has FIVE u32 fields:
 * pipe_num, pipe_dir (enum32), nentries, nbytes_max, flags (no reserved).
 * qmi_wlanfw_ce_svc_pipe_cfg_s_v01_ei: service_id, pipe_dir, pipe_num.
 * hw_params.target_ce_count=9, svc_to_ce_map_len=14 excludes sentinel. */
static const uint32_t wl15_target_ce[9][5] = {
	{0, 2, 32, 2048, 0}, {1, 1, 32, 2048, 0},
	{2, 1, 32, 2048, 0}, {3, 2, 32, 2048, 0},
	{4, 2, 256, 256, 8}, {5, 1, 32, 2048, 0},
	{6, 3, 32, 16384, 0}, {7, 4, 0, 0, 8}, {8, 3, 32, 16384, 0}
};
static const uint32_t wl15_service_ce[14][3] = {
	{0x104, 2, 3}, {0x104, 1, 2}, /* WMI_DATA_VO */
	{0x102, 2, 3}, {0x102, 1, 2}, /* WMI_DATA_BK */
	{0x101, 2, 3}, {0x101, 1, 2}, /* WMI_DATA_BE */
	{0x103, 2, 3}, {0x103, 1, 2}, /* WMI_DATA_VI */
	{0x100, 2, 3}, {0x100, 1, 2}, /* WMI_CONTROL */
	{1, 2, 0}, {1, 1, 2},         /* RSVD_CTRL (downlink CE2) */
	{0x300, 2, 4}, {0x300, 1, 1}  /* HTT_DATA_MSG */
};

/* Adapted from Linux ath12k hal.c/hal.h/ce.c (BSD-3-Clause-Clear), v6.12.
 * Copyright (c) 2018-2021 The Linux Foundation;
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc.
 * Result of ath12k_ce_get_shadow_config():
 * ath12k_hal_srng_create_config_wcn7850 + ath12k_hal_srng_shadow_config,
 * then ath12k_ce_shadow_config using ath12k_host_ce_config_wcn7850.
 * HAL ring enum order; skip DMAC/PMAC and CE initially; every UMAC ring
 * contributes reg_start[1]+ring*reg_size[1]+(DST ? 4 : 0).
 * CE source 0/3/4 and destination+status 1/2 only; CE5 has zero entries.
 * These are WCSS-relative address VALUES for QMI, not BAR accesses. */
static const uint32_t wl15_shadow_v3[36] = {
	0x00a3b04c, 0x00a3b054, 0x00a3b05c, 0x00a3b064, /* REO_DST 0..7 TP */
	0x00a3b06c, 0x00a3b074, 0x00a3b07c, 0x00a3b084,
	0x00a3b08c, /* REO_EXCEPTION TP */
	0x00a3b028, /* REO_REINJECT HP */
	0x00a3b020, /* REO_CMD HP */
	0x00a3b0ac, /* REO_STATUS TP */
	0x00a46000, 0x00a46008, 0x00a46010, 0x00a46018, 0x00a46020, /* TCL_DATA */
	0x00a46028, 0x00a4604c, /* TCL_CMD HP, TCL_STATUS TP */
	0x00a370b8, 0x00a37010, /* WBM_IDLE_LINK, SW2WBM_RELEASE HP */
	0x00a370cc, 0x00a370d4, 0x00a370dc, 0x00a370e4, /* WBM2SW_RELEASE TP */
	0x00a370ec, 0x00a370f4, 0x00a370fc, 0x00a37104,
	0x01b80400,                         /* CE0 SRC HP */
	0x01b83400, 0x01b8340c,             /* CE1 DST HP, STATUS TP */
	0x01b85400, 0x01b8540c,             /* CE2 DST HP, STATUS TP */
	0x01b86400, 0x01b88400              /* CE3/4 SRC HP */
};

static int
wl15_encode_cfg(struct wl12_encoder *e)
{
	/* Adapted from Linux ath12k qmi.c/qmi.h (BSD-3-Clause-Clear):
	 * qmi_wlanfw_wlan_cfg_req_msg_v01 / *_ei. host_version TLV 0x10
	 * QMI_STRING is raw "WIN" (ATH12K_HOST_VERSION_STRING), no count/NUL.
	 * TLVs 0x11/0x12/0x17 use QMI_DATA_LEN u8 and nested structs above;
	 * qmi_wlanfw_shadow_reg_v3_cfg_s_v01_ei is one u32 addr. No v1 0x13. */
	if (e->len != 7) { return wl_w11_fail("W15 CFG encoder state"); }
	return wl12_tlv(e, 0x10, 0x4e4957, 3) ||
	       wl15_array(e, 0x11, wl15_target_ce, 9, 5) ||
	       wl15_array(e, 0x12, wl15_service_ce, 14, 3) ||
	       (!wl15_variant && wl15_array(e, 0x17, wl15_shadow_v3, 36, 1));
}

/* Allocation policy facts: ath12k v6.6 qmi.c, BSD-3-Clause-Clear,
 * The Linux Foundation (2018-2021), Qualcomm Innovation Center (2021-2022).
 * Original implementation: no EI tables, Linux allocators or helpers copied.
 * 2 MiB is ath12k's failure/fallback threshold, NOT a firmware maximum.
 * =3 deliberately declines larger supported chunks before any allocation/map
 * to exercise that fallback without reclaiming live DMA. =2 is unchanged. */
static bool
wl13_supported(uint32_t type)
{
	return type == 1 || type == 3 || type == 4 || type == 9;
}

static uint64_t
wl13_alignment(uint32_t size)
{
	uint64_t align = PAGE_SIZE;
	while (align < size) { align <<= 1; }
	return align;
}

/* RESPOND_MEM EI: mandatory TLV 0x01; QMI_DATA_LEN sizeof(u8), then
 * nested addr u64 / size u32 / signed enum32 / restore u8. Wire stride 17,
 * no native struct padding or per-entry TLVs. Explicit little endian.
 * Count zero still emits 01 01 00 00 (not an absent TLV). */
static int
wl12_encode_respond(struct wl12_encoder *e, bool delayed)
{
	unsigned count = delayed ? 0 : wl_target_n;
	if (count > WL_TARGET_MAX || e->len != 7) { return wl_w11_fail("RESPOND_MEM encode bounds"); }
	unsigned bytes = 1+17*count;
	e->data[7] = 1; wl_put16(e->data+8, bytes); e->data[10] = count;
	for (unsigned i = 0; i < count; i++) {
		uint8_t *v = e->data+11+17*i;
		wl_put64(v, wl_target[i].iova); wl_put32(v+8, wl_target_size[i]);
		wl_put32(v+12, wl_target_type[i]); v[16] = 0;
	}
	e->len = 10+bytes;
	printf("FUXI WLAN: W13 RESPOND_MEM TLV 0x01 count-u8 %u entry-bytes 17 restore 0 LE%s\n",
	    count, delayed ? " delayed" : "");
	return 0;
}

/* Parse the variable array's u8 counts, NOT native u32 struct lengths.
 * Validate the entire request before allocating anything or accepting it. */
static int
wl12_request_mem(const uint8_t *v, unsigned n)
{
	if (!n || !v[0] || v[0] > WL_TARGET_MAX) { return wl_w11_fail("REQUEST_MEM segment count"); }
	unsigned count = v[0], off = 1;
	uint64_t total = 0, rounded = 0, cursor = wl_iova_next;
	bool large = false;
	for (unsigned i = 0; i < count; i++) {
		if (n-off < 9) { return wl_w11_fail("REQUEST_MEM truncated segment"); }
		uint32_t size = wl_get32(v+off), type = wl_get32(v+off+4);
		unsigned cfg = v[off+8]; off += 9;
		printf("FUXI WLAN: W12 request segment %u type %u size %u cfg %u\n", i, type, size, cfg);
		if (!size || size > WL_TARGET_CAP || cfg > 2 || n-off < cfg*13) {
			return wl_w11_fail("REQUEST_MEM size/config bounds (cap 64 MiB)");
		}
		/* Nonsecure subranges are metadata within this segment. This lane
		 * cannot furnish secure memory; never silently claim it did. */
		for (unsigned c = 0; c < cfg; c++, off += 13) {
			uint64_t offset = wl_get64(v+off); uint32_t bytes = wl_get32(v+off+8);
			unsigned secure = v[off+12];
			printf("FUXI WLAN: W12 request segment %u cfg %u offset %#llx size %u secure %u\n", i, c, offset, bytes, secure);
			if (offset > size || bytes > size-offset || secure) { return wl_w11_fail("REQUEST_MEM unsupported secure/range config"); }
		}
		total += size; rounded += ((uint64_t)size + PAGE_MASK) & ~(uint64_t)PAGE_MASK;
		if (rounded > WL_TARGET_CAP) {
			printf("FUXI WLAN: W12 request refused n=%u total=%llu rounded=%llu cap=%u\n", count, total, rounded, WL_TARGET_CAP);
			return wl_w11_fail("REQUEST_MEM total exceeds 64 MiB cap");
		}
		if (wl_w13 && wl13_supported(type)) {
			if (size > WL13_CHUNK_LIMIT) { large = true; }
			uint64_t align = wl13_alignment(size);
			cursor = ((cursor+align-1) & ~(align-1)) + round_page(size);
		}
		wl_target_size[i] = size; wl_target_type[i] = type;
	}
	if (off != n || wl_iova_next < wl_iova_base || wl_iova_next >= wl_iova_end ||
	    rounded > wl_iova_end-wl_iova_next) { return wl_w11_fail("REQUEST_MEM trailing bytes/IOVA exhaustion"); }
	if (wl_w13 && ((wl13_negotiated && large) || (!large && cursor > wl_iova_end))) {
		return wl_w11_fail("REQUEST_MEM small-chunk limit/aligned IOVA exhaustion; negotiation exhausted");
	}
	wl_target_n = count; wl_target_total = total; wl12_requested = true;
	printf("FUXI WLAN: W12 request mem n=%u total=%llu rounded=%llu cap=%u\n", count, total, rounded, WL_TARGET_CAP);
	return 0;
}

/* Selected endpoint only. Indications never complete a transaction. An
 * indication racing its triggering response is queued; premature ones are
 * logged and ignored, and cannot satisfy a later wait. Duplicate provisioning
 * fails rather than replacing DMA the device may already be using. */
static int
wl_qmi_w12(const uint8_t *p, unsigned len)
{
	const char *lane = wl_w15 && wl12_stage >= WL15_INI ? "W15" : "W12";
	if (len >= 7 && p[0] == 2 && wl12_mlo_variant == 1 && wl12_msg == 0x57 &&
	    !wl12_reply && wl_get16(p+1) == wl12_txn && wl_get16(p+3) == 0x57) {
		return wl12_phy_reply(p, len);
	}
	if (len < 7 || wl_get16(p+5) != len-7) { return wl_w11_fail("QMI header length"); }
	unsigned flag = p[0], txn = wl_get16(p+1), msg = wl_get16(p+3);
	bool indication = flag == 4;
	if (flag != 2 && !indication) { return wl_w11_fail("QMI unsupported flag"); }
	if (!indication && (!wl12_txn || wl12_reply || txn != wl12_txn || msg != wl12_msg)) {
		printf("FUXI WLAN: %s unrelated reply txn %u msg %#x stage %u\n", lane, txn, msg, wl12_stage);
		return 0;
	}
	if (!indication && msg == 0x24) {
		int e = wl_qmi_cap(p, len);
		if (!e && wl_cap_done) { wl12_reply = true; }
		return e;
	}
	const uint8_t *segments = NULL; unsigned segment_bytes = 0;
	bool result = false, status = false, shared = false; uint64_t fw_status = 0;
	for (unsigned off = 7; off < len;) {
		if (len-off < 3) { return wl_w11_fail("QMI truncated TLV header"); }
		unsigned type = p[off], n = wl_get16(p+off+1); off += 3;
		if (n > len-off) { return wl_w11_fail("QMI truncated TLV value"); }
		const uint8_t *v = p+off;
		if (indication && msg == 0x35 && type == 1) {
			if (segments) { return wl_w11_fail("REQUEST_MEM duplicate segment TLV"); }
			segments = v; segment_bytes = n;
		} else if (!indication && type == 2) {
			if (result || n != 4) { return wl_w11_fail("QMI result duplicate/size"); }
			result = true;
			printf("FUXI WLAN: %s reply txn %u msg %#x result %u error %u%s\n",
			    lane, txn, msg,
			    wl_get16(v), wl_get16(v+2), wl_get16(v+2) == 90 ? " (INCOMPATIBLE_STATE)" : "");
			if (wl_get16(v) || wl_get16(v+2)) {
				if (wl13_delayed && msg == 0x36 && !wl_get16(v+2)) {
					printf("FUXI WLAN: W13 delayed RESPOND_MEM result %u error 0 accepted (ath12k)\n", wl_get16(v));
				} else { return wl_w11_fail("WLFW setup rejected"); }
			}
		} else if (!indication && msg == 0x20 && type == 0x10) {
			if (status || n != 8) { return wl_w11_fail("IND_REGISTER status duplicate/size"); }
			status = true; fw_status = wl_get64(v);
		} else if (!indication && msg == 0x36 && type == 0x10) {
			if (shared || !n || v[0] > 8 || n != 1u+20u*v[0]) { return wl_w11_fail("RESPOND_MEM share_mem duplicate/bounds"); }
			shared = true;
			printf("FUXI WLAN: W14 RESPOND_MEM share_mem_len %u (reported only)\n", v[0]);
			for (unsigned i = 0; i < v[0]; i++) {
				const uint8_t *q = v+1+20*i;
				printf("FUXI WLAN: W14 share_mem %u type %u start %#llx size %#llx (never dereferenced)\n",
				    i, wl_get32(q), wl_get64(q+4), wl_get64(q+12));
			}
		} else {
			printf("FUXI WLAN: %s msg %#x unknown TLV %#x bytes %u\n", lane, msg, type, n);
		}
		off += n;
	}
	if (!indication) {
		if (!result) { return wl_w11_fail("QMI reply missing result"); }
		if (status) { printf("FUXI WLAN: W12 ind register fw status %#llx\n", fw_status); }
		wl12_reply = true;
		if (wl_w15 && wl12_stage >= WL15_INI) { wl15_elapsed(wl15_step(true)); }
		return 0;
	}
	if (wl_w15 && wl12_stage >= WL15_INI) {
		/* Observe only after provisioning: never act on a fresh memory or
		 * readiness indication, and never let it complete a transaction. */
		printf("FUXI WLAN: W15 indication msg %#x stage %u bytes %u (reported only)\n", msg, wl12_stage, len-7);
		return 0;
	}
	printf("FUXI WLAN: W12 indication msg %#x stage %u bytes %u\n", msg, wl12_stage, len-7);
	if (msg == 0x35) {
		if (wl12_requested) { return wl_w11_fail("duplicate REQUEST_MEM; no reprovision"); }
		if (wl12_stage == WL12_RESPOND && wl13_delayed) {
			printf("FUXI WLAN: W13 smaller REQUEST_MEM queued before delayed RESPOND_MEM reply\n");
		} else if (wl12_stage != WL12_REQUEST) {
			printf("FUXI WLAN: W12 out-of-order REQUEST_MEM stage %u %s\n", wl12_stage,
			    wl12_stage == WL12_HOST ? "queued before HOST_CAP reply" : "ignored before HOST_CAP");
			if (wl12_stage != WL12_HOST) { return 0; }
		}
		if (!segments) { return wl_w11_fail("REQUEST_MEM missing segments"); }
		return wl12_request_mem(segments, segment_bytes);
	}
	if (wl_w14 && (msg == 0x21 || msg == 0x38)) {
		/* Only an indication after M3 publication can satisfy W14. A race
		 * before the M3 reply is retained; the reply is still mandatory.
		 * ath12k's FW_READY_IND is 0x38 (ath11k calls 0x38 FW_INIT_DONE
		 * and uses 0x21 for ready); 0x21 is only reported here. */
		if (wl12_stage == WL14_M3 || wl12_stage == WL14_READY) {
			if (msg == 0x38) {
				if (!wl14_ready && wl_w15) { wl15_ready_time = mach_absolute_time(); }
				wl14_ready = true;
			}
			else { wl14_init_done = true; }
			printf("FUXI WLAN: W14 %s indication%s\n", msg == 0x38 ? "FW_READY (0x38)" : "ath11k FW_READY (0x21)",
			    wl12_stage == WL14_M3 ? " queued during M3_INFO reply wait" : "");
		} else { printf("FUXI WLAN: W14 premature ready indication %#x ignored stage %u\n", msg, wl12_stage); }
		return 0;
	}
	if (msg == 0x37) {
		if (wl13_delayed) {
			return wl_w11_fail("FW_MEM_READY during zero-segment negotiation");
		}
		if (wl12_stage == WL12_RESPOND || wl12_stage == WL12_READY) {
			if (wl12_ready) { printf("FUXI WLAN: W12 out-of-order duplicate FW_MEM_READY ignored\n"); return 0; }
			if (wl12_stage == WL12_RESPOND) { printf("FUXI WLAN: W12 out-of-order FW_MEM_READY queued before RESPOND_MEM reply\n"); }
			wl12_ready = true;
		} else { printf("FUXI WLAN: W12 out-of-order FW_MEM_READY stage %u ignored\n", wl12_stage); }
	} else {
		/* Includes FW_INIT_DONE (ath12k FW_READY 0x38), legacy
		 * FW_READY (0x21), calibration and future indications. */
		printf("FUXI WLAN: W12 %s indication %#x stage %u deferred; no BDF/M3/regdb in W12\n",
		    msg == 0x38 || msg == 0x21 ? "out-of-order ready" : "unknown/cal", msg, wl12_stage);
	}
	return 0;
}

/* P17 ERRDEC: classify the BHI error words (raw values stay in the line
 * above) and, when the device is in RDDM and the RX sink exists, scan the
 * uploaded dump for firmware assert text. Reads only; no dump is requested. */
static void
wl17_emit_hit(uint64_t off, const char *text)
{
	printf("FUXI WLAN: P17 rddm text +%#llx: %s\n", off, text);
}

static void
wl17_rddm_scan(void)
{
	if (!wl_rddm.va || !wl_bhie || wl_rddm.size < WL_RDDM_SIZE) {
		printf("FUXI WLAN: P17 rddm scan unavailable (no RX sink; needs fuxiwlanvec=rddm)\n"); return;
	}
	uint32_t st = 0; unsigned waited = 0, before = fw_faults;
	for (; waited < 100; waited++) {
		st = wl_mrd(wl_bhie+0x78);
		if (fw_faults != before || (st >> 30) >= 2) { break; }
		wl_sleep(10);
	}
	printf("FUXI WLAN: P17 rddm upload RXVECSTATUS %#x after %u ms faults %u\n", st, waited*10, fw_faults-before);
	if (fw_faults != before || (st >> 30) != 2) { return; }
	wl_dma_civac(&wl_rddm);
	const uint8_t *seg = (const uint8_t *)wl_rddm.va;
	unsigned found = wl17_scan(seg, WL_RDDM_SIZE, 40, wl17_emit_hit);
	printf("FUXI WLAN: P17 rddm scan %u bytes %u messages\n", WL_RDDM_SIZE, found);
	for (unsigned o = 0; o < 0x80; o += 32) {
		uint32_t w[8]; memcpy(w, seg+o, 32);
		printf("FUXI WLAN: P17 rddm-rec %03x: %08x %08x %08x %08x %08x %08x %08x %08x\n", o, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
	}
}

static void
wl17_errdec(const char *label, uint32_t ee, uint32_t code, uint32_t d1, uint32_t d2, uint32_t d3)
{
	const char *names[4] = {"ERRCODE", "ERRDBG1", "ERRDBG2", "ERRDBG3"};
	uint32_t w[4] = {code, d1, d2, d3};
	char text[160];
	for (unsigned i = 0; i < 4; i++) {
		wl17_errword(w[i], text, sizeof(text));
		printf("FUXI WLAN: P17 %s %s %s\n", label, names[i], text);
	}
	if (ee == 3 && !fw_faults) { wl17_rddm_scan(); }
}

/* Snapshot only: six fault-contained reads before STOP/reset can change the
 * evidence. RDDM EE=3; MHISTATUS SYSERR is bit 2, MHI state is bits 15:8
 * (SYS_ERR=0xff). Never arm/read a crash dump or initiate a large transfer. */
static void
wl_wlfw_diagnostics(const char *label)
{
	unsigned before = fw_faults;
	uint32_t ee = wl_mrd(wl_bhi+0x28), st = wl_mrd(0x48);
	uint32_t code = wl_mrd(wl_bhi+0x30), d1 = wl_mrd(wl_bhi+0x34);
	uint32_t d2 = wl_mrd(wl_bhi+0x38), d3 = wl_mrd(wl_bhi+0x3c);
	printf("FUXI WLAN: %s diagnostic BHI_EXECENV %#x (%s) MHISTATUS %#x SYSERR(bit2) %u MHI_STATE %#x BHI_ERRCODE %#x BHI_ERRDBG1 %#x BHI_ERRDBG2 %#x BHI_ERRDBG3 %#x access-faults %u valid %u\n",
	    label, ee, wl_mhi_ee(ee), st, (st >> 2) & 1, (st >> 8) & 255,
	    code, d1, d2, d3, fw_faults-before, !fw_faults && ee != ~0u && st != ~0u);
	if (ee == 3 && !fw_faults) {
		printf("FUXI WLAN: %s RDDM observed; dump not collected; contain\n", wl_w15 && wl12_stage >= WL15_INI ? "W15" : "W13");
	}
	if ((wl17_bits & WL17_ERRDEC) && !fw_faults) { wl17_errdec(label, ee, code, d1, d2, d3); }
}

static void
wl13_respond_diagnostics(void)
{
	wl_wlfw_diagnostics("W13 RESPOND_MEM");
}

/* MODE timeout only. Read shared Normal-NC contexts/events without consuming,
 * crediting, reposting, acknowledging QRTR or ringing any doorbell. BHI owns
 * ERRCODE/ERRDBG1..3; BHIe exposes vector status, not a second ERRDBG bank.
 * At most 16 events per ring and 16 QMI headers; never read payload/dump data. */
static void
wl15_timeout_snapshot(void)
{
	if (!wl_w15 || wl12_stage != WL15_MODE) { return; }
	wl16_snapshot("MODE timeout");
	wl15_elapsed("MODE timeout");
	wl_wlfw_diagnostics("W15 MODE timeout");
	unsigned faults = fw_faults;
	uint32_t ctrl = wl_mrd(0x38), tx = wl_mrd(wl_bhie+0x44), rx = wl_mrd(wl_bhie+0x78);
	printf("FUXI WLAN: W15 timeout MHICTRL %#x BHIE_TXVECSTATUS %#x BHIE_RXVECSTATUS %#x access-faults %u valid %u\n",
	    ctrl, tx, rx, fw_faults-faults, !fw_faults && ctrl != ~0u && tx != ~0u && rx != ~0u);
	printf("FUXI WLAN: W15 pending QMI txn %u msg %#x reply %u cmd_pending %u\n",
	    wl12_txn, wl12_msg, wl12_reply, !!wl_cmd_pending);
	if (!wl_contexts.va || !wl_ipcr.va || !wl_contexts.nc || !wl_ipcr.nc ||
	    wl_contexts.size < 16384 || wl_ipcr.size < WL_IPCR_BYTES) {
		printf("FUXI WLAN: W15 timeout ring snapshot unavailable\n"); return;
	}
	wl_w11_barrier();
	for (unsigned ch = 20; ch <= 21; ch++) {
		struct wl_context c = *wl_channel(ch);
		printf("FUXI WLAN: W15 IPCR ch %u state %u host read %u write %u RP %#llx WP %#llx\n",
		    ch, c.cfg & 255, ch == 20 ? wl_tx_read : wl_rx_read,
		    ch == 20 ? wl_tx_write : wl_rx_write, c.rp, c.wp);
	}
	unsigned pending = 0;
	for (unsigned r = 0; r < 2; r++) {
		struct wl_context ec = *(struct wl_context *)(wl_contexts.va+WL_ECTX+r*sizeof(ec));
		unsigned off = r ? WL_ER1 : WL_ER0, n = r ? 256 : 32;
		uint64_t base = wl_contexts.iova+off;
		bool valid = ec.rp >= base && ec.rp < base+n*16 && !((ec.rp-base)&15) && wl_er_read[r] < n;
		unsigned count = valid ? ((unsigned)((ec.rp-base)/16)+n-wl_er_read[r])%n : 0;
		printf("FUXI WLAN: W15 ER %u host read %u RP %#llx WP %#llx pending %u valid %u truncated %u\n",
		    r, wl_er_read[r], ec.rp, ec.wp, count, valid, count > 16);
		for (unsigned j = 0; j < count && j < 16 && pending < 16; j++) {
			struct wl_tre ev = ((struct wl_tre *)(wl_contexts.va+off))[(wl_er_read[r]+j)%n];
			if ((ev.d1 >> 24) != 21 || ((ev.d1 >> 16)&255) != 0x22 || (ev.d0 >> 24) != 2) { continue; }
			uint64_t rb = wl_ipcr.iova+WL_IPCR_RXR;
			if (ev.ptr < rb || ev.ptr >= rb+WL_IPCR_N*16 || ((ev.ptr-rb)&15)) { continue; }
			struct wl_tre tre = wl_ipcr_ring(WL_IPCR_RXR)[(ev.ptr-rb)/16];
			uint64_t lo = wl_ipcr.iova+WL_IPCR_RXBUF;
			unsigned bytes = ev.d0 & 0xffff;
			if (tre.ptr < lo || tre.ptr >= lo+WL_IPCR_N*WL_IPCR_MTU ||
			    (tre.ptr-lo)%WL_IPCR_MTU || bytes > WL_IPCR_MTU || bytes > (tre.d0&0xffff)) { continue; }
			const uint8_t *p = (const uint8_t *)(wl_ipcr.va+(tre.ptr-wl_ipcr.iova));
			unsigned hdr, len;
			if (bytes >= 32 && wl_get32(p) == 1 && wl_get32(p+4) == 1 &&
			    wl_get32(p+8) == wl_server_node && wl_get32(p+12) == wl_server_port &&
			    wl_get32(p+28) == WL_QRTR_PORT) { hdr = 32; len = wl_get32(p+20); }
			else if (bytes >= 16 && p[0] == 3 && p[1] == 1 && !p[3] &&
			    wl_get16(p+8) == wl_server_node && wl_get16(p+10) == wl_server_port &&
			    wl_get16(p+14) == WL_QRTR_PORT) { hdr = 16; len = wl_get32(p+4); }
			else { continue; }
			if (len < 7 || len > bytes-hdr || wl_get16(p+hdr+5) != len-7) { continue; }
			const uint8_t *q = p+hdr;
			printf("FUXI WLAN: W15 pending RX QMI type %#x txn %u msg %#x bytes %u\n",
			    q[0], wl_get16(q+1), wl_get16(q+3), len);
			pending++;
		}
	}
	printf("FUXI WLAN: W15 pending RX QMI headers %u (snapshot only)\n", pending);
}

static int
wl12_wait(unsigned what, const char *stage)
{
	/* Match ath12k's ten-second WLFW timeout, with bounded transport passes. */
	for (unsigned ms = 0; ms <= 10000; ms += 10) {
		if (wl_w11_events()) {
			if (wl12_stage == WL12_RESPOND) { wl13_respond_diagnostics(); }
			return 1;
		}
		if (what == 0 ? wl12_reply : what == 1 ? wl12_requested : what == 2 ? wl12_ready : wl14_ready) { return 0; }
		if (wl16_variant && wl12_stage == WL15_MODE && ms == 1000) { wl16_snapshot("MODE pending 1000 ms"); }
		if (ms < 10000) { wl_sleep(10); }
	}
	if (wl12_mlo_variant == 1 && wl12_msg == 0x57) {
		printf("FUXI WLAN: W12 phy cap fallback num_phy 2 (timeout 10000 ms)\n");
		return 0; /* ath12k_qmi_phy_cap_send is optional; transport faults are fatal above. */
	}
	printf("FUXI WLAN: %s timeout %s (10000 ms); contain, no retry\n", wl_w15 && wl12_stage >= WL15_INI ? "W15" : "W12", stage);
	if (wl_w15 && wl12_stage == WL15_MODE) { wl15_timeout_snapshot(); }
	if (wl12_stage == WL12_RESPOND) { wl13_respond_diagnostics(); }
	return wl_w11_fail(stage);
}

/* Extend only invalid leaves in the preallocated owned 256-MiB pool. Never
 * rebuild live tables (which would overwrite active mappings). Check every
 * parent through the live TTBR before writing any leaf; clean, TLBI + sync,
 * then walk each newly installed leaf and recheck live CB/route registers. */
static int
wl12_map_target(struct wl_dma *d)
{
	const uint64_t mask = 0x0000000ffffff000ull;
	uint64_t cb = wl_cb_addr(wl_cb), root = wl_pt_pa(0) | ((uint64_t)wl_cb << 48);
	if (!d->va || !d->size || d->iova < wl_iova_base || d->iova >= wl_iova_end ||
	    d->size > wl_iova_end-d->iova || wl_w11_guard()) { return wl_w11_fail("target DMA bounds/guard"); }
	for (unsigned pass = 0; pass < 3; pass++) {
		if (wl_prd(cb+0x20) != (uint32_t)root || wl_prd(cb+0x24) != (uint32_t)(root >> 32) ||
		    (wl_prd(cb) & WL_CB_SCTLR_MASK) != (WL_CB_SCTLR & WL_CB_SCTLR_MASK) ||
		    (wl_prd(cb+0x30) & WL_CB_TCR_MASK) != (WL_CB_TCR & WL_CB_TCR_MASK) ||
		    (wl_prd(cb+0x38) & WL_CB_MAIR_MASK) != 0xff ||
		    wl_prd(WL_SMMU+0x800+4*wl_slot) != 0x80001401 ||
		    wl_prd(WL_SMMU+0xc00+4*wl_slot) != wl_cb || wl_rc_check() || fw_faults) {
			return wl_w11_fail("target DMA live SMMU readback");
		}
		for (vm_size_t off = 0; off < d->size; off += 4096) {
			uint64_t addr = d->iova+off, pa = wl_phys(d->va+off);
			vm_offset_t alias = wl_owned_alias(root & mask, 4096, true);
			if (!alias || ((uint64_t *)alias)[addr >> 30] != (wl_pt_pa(1) | 3)) { return wl_w11_fail("target DMA root changed"); }
			alias = wl_owned_alias(wl_pt_pa(1), 4096, true);
			unsigned table = 2+(unsigned)((addr-wl_iova_base) >> 21);
			if (!alias || ((uint64_t *)alias)[(addr >> 21) & 511] != (wl_pt_pa(table) | 3)) { return wl_w11_fail("target DMA parent changed"); }
			alias = wl_owned_alias(wl_pt_pa(table), 4096, true);
			if (!alias || !pa || pa >= (1ull << 36)) { return wl_w11_fail("target DMA backing"); }
			uint64_t *leaf = &((uint64_t *)alias)[(addr >> 12) & 511];
			if (pass == 0 && *leaf) { return wl_w11_fail("target DMA occupied leaf"); }
			if (pass == 1) { *leaf = pa | WL_DMA_PTE; }
			if (pass == 2 && *leaf != (pa | WL_DMA_PTE)) { return wl_w11_fail("target DMA leaf readback"); }
		}
		if (pass == 1) {
			wl_dma_clean(&wl_tables);
			if (wl_probe == WL_PROBE_CIVAC) { wl_dma_civac(&wl_tables); }
			if (wl_smmu_sync(false)) { return wl_w11_fail("target DMA TLB sync"); }
		}
	}
	printf("FUXI WLAN: W12 DMA window extended IOVA %#llx-%#llx (pool end %#llx), live SMMU/leaves readback verified\n",
	    d->iova, d->iova+d->size-1, wl_iova_end);
	printf("FUXI WLAN: W13 target DMA SID 0x1401 CB %u AP[7:6]=1 RW AttrIdx=0 MAIR0=0xff OSH; CPU Normal-NC\n", wl_cb);
	return wl_w11_guard();
}


/* W14 uses ath12k v6.6 BSD-3-Clause-Clear EI field facts, credited above.
 * File reads and codecs are original. No firmware bytes are built into XNU. */
static int
wl14_read(const char *name, struct wl_dma *d, unsigned limit, unsigned *size)
{
	char path[128]; vnode_t vp = NULL; vfs_context_t ctx = vfs_context_kernel();
	if (!wl14_basename(name)) { return wl_w11_fail("W14 invalid firmware basename"); }
	snprintf(path, sizeof(path), "/usr/local/share/fuxi/wlan/%s", name);
	int e = vnode_open(path, FREAD | O_NOFOLLOW | O_NONBLOCK, 0, 0, &vp, ctx);
	if (e) {
		printf("FUXI WLAN: W14 missing/unreadable file %s errno %d; contain\n", path, e);
		return wl_w11_fail("W14 firmware open");
	}
	struct vnode_attr a; VATTR_INIT(&a); VATTR_WANTED(&a, va_data_size);
	e = vnode_getattr(vp, &a, ctx);
	if (e || vnode_vtype(vp) != VREG || !VATTR_IS_SUPPORTED(&a, va_data_size) ||
	    !a.va_data_size || a.va_data_size > limit) {
		printf("FUXI WLAN: W14 invalid file %s size/type (limit %u)\n", path, limit);
		e = 1; goto close;
	}
	*size = (unsigned)a.va_data_size;
	if (d == &wl14_m3) {
		uint64_t align = wl13_alignment(*size);
		wl_iova_next = (wl_iova_next+align-1) & ~(align-1);
	}
	if (d->va || wl_dma_alloc(d, *size)) { e = 1; goto close; }
	int resid = 0;
	e = vn_rdwr(UIO_READ, vp, (caddr_t)d->va, (int)*size, 0, UIO_SYSSPACE, 0,
	    vfs_context_ucred(ctx), &resid, vfs_context_proc(ctx));
	if (resid) { e = 1; }
	if (!e) { printf("FUXI WLAN: W14 file %s bytes %u\n", name, *size); }
close:
	/* No data is advertised on a failed/short read; partial allocations stay
	 * owned until the common containment/quarantine path proves safety. */
	if (vnode_close(vp, FREAD, ctx)) { e = 1; }
	if (e) { printf("FUXI WLAN: W14 file read/validation failed %s\n", path); return wl_w11_fail("W14 file read/validation"); }
	return wl_w11_guard();
}

static int
wl14_encode_bdf(struct wl12_encoder *e, const uint8_t *data,
    unsigned remaining, unsigned seg, unsigned type)
{
	unsigned n = remaining > WL14_SEG ? WL14_SEG : remaining;
	if (!n || e->len != 7 || type > 4 ||
	    wl12_tlv(e, 1, 1, 1) || wl12_tlv(e, 0x10, wl14_board, 4) ||
	    wl12_tlv(e, 0x11, remaining, 4) || wl12_tlv(e, 0x12, seg, 4)) { return wl_w11_fail("W14 segment bounds"); }
	/* EI data_len is u16 within TLV 0x13; valid flags are presence only.
	 * ath12k total_size is REMAINING on every segment, not original size. */
	e->data[e->len++] = 0x13; wl_put16(e->data+e->len, n+2); e->len += 2;
	wl_put16(e->data+e->len, n); e->len += 2;
	memcpy(e->data+e->len, data, n); e->len += n;
	return wl12_tlv(e, 0x14, remaining <= WL14_SEG, 1) || wl12_tlv(e, 0x15, type, 1);
}

static int
wl14_download(const char *name, struct wl_dma *d, unsigned stage)
{
	unsigned size = 0;
	if (wl14_read(name, d, WL14_FILE_MAX, &size)) { return 1; }
	const uint8_t *data = (const void *)d->va;
	unsigned type = stage == WL14_REGDB ? 4 : size >= 4 && !memcmp(data, "\177ELF", 4) ? 1 : 0;
	static struct wl12_encoder e;
	for (unsigned off = 0, seg = 0; off < size; seg++) {
		unsigned n = size-off > WL14_SEG ? WL14_SEG : size-off;
		bzero(&e, sizeof(e)); e.len = 7;
		if (wl14_encode_bdf(&e, data+off, size-off, seg, type) ||
		    wl12_send(&e, 0x25, stage) || wl12_wait(0, "W14 BDF_DOWNLOAD reply") ||
		    wl_w11_wait(3, "W14 segment TX drain")) { return 1; }
		printf("FUXI WLAN: W14 download %s type %u seg %u bytes %u remaining %u end %u ok\n",
		    name, type, seg, n, size-off, off+n == size);
		off += n;
	}
	if (stage == WL14_REGDB) { printf("FUXI WLAN: W14 regdb ok\n"); }
	else { printf("FUXI WLAN: W14 bdf %s ok\n", name); }
	return 0;
}

/* ---- W20: CE/HTC/WMI to WMI READY (fuxiwlanqrtr=6, C1) ----
 * Adapted from Linux ath12k core.c, ce.c, htc.c, wmi.c and pci.c (C1) and
 * mac.c, reg.c, wmi.c and dp_tx.c (C2, the W23/W24 part below)
 * (BSD-3-Clause-Clear), v6.19; the notice and the file list are in
 * bsd/dev/fuxi_wlan_ce.h, the pure ring/HTC/WMI code. This block is the
 * kernel glue: the quiet windowed register access, the arena, the poll
 * loop, logs, the failure snapshot and the containment hook. Reached only
 * from wl_w14_run
 * after this boot's WLAN_MODE reply. Register writes: R0/R2 of the 7 CE rings
 * and DEST_CTRL of CE1/CE2 (wlce_reg_ok; 0x310c saved, selected, verified and
 * restored around every access), the 7 shadow registers BAR0 0x8fc-0x998 and,
 * with w20 0x04 only, the endpoint Device Control register. No interrupt,
 * MSI, GIC, pdev parameter, vdev or RF activity; the W11 conversation and its
 * device-wake doorbell stay up. L19 order after MODE (core.c:1274-1300,
 * 868-960): CE init, HTC init, RX post, HTC READY, HTT and WMI connect,
 * SETUP_COMPLETE_EX, SERVICE_READY (+ EXT, EXT2), INIT, READY, HW mode DBS;
 * the data path, regulatory and the HTT version request are skipped.
 * With qrtr 7 (C2) the W23/W24 scan stage below follows W22, with the C2b
 * diagnostics (CHAN_INFO lines, pdev stats requests) inside it, and with a
 * C2c bit (w20 0x100/0x200/0x400) the W26 RX resources: the only other
 * registers are ring 120's of the WBM (wlrx_wbm_ok) and its shadow register.
 * Per-field provenance and every log line: docs-lanes/wlan-w20.md. */
static struct wlce wl_ce;
static struct wlhtc wl_htc;
static struct wlwmi wl_wmi;
static bool wl_ce_started;
static unsigned wl_ce_waited, wl_ce_need_eid, wl_ce_need_len, wl_ce_tx_pipe, wl_ce_logged;
/* W23 (C2): scan state, set up by wl_w23_run, read by the poll and the contain hook.
 * wl23_guard_bad: a poll pass saw the W11 events or guard fail (AMSS/M0, SYSERR,
 * MHI reset, discovery, MEM|MASTER, SMMU CB); containment then sends nothing.
 * wl23_rssi_conv: 0x402b events seen with qrtr 7 (W24 detail line). */
static struct wlscan wl_scan;
static bool wl23_scan_sent, wl23_htt_seen, wl23_guard_bad;
static unsigned wl23_lines, wl23_pdev, wl23_htt_major, wl23_htt_minor, wl23_rssi_conv;
/* C2b: the pdev stats answer (wl23_stats) and the CHAN_INFO detail lines,
 * WL23_CI_LINES at most, the rest counted (wl23_ci_line, wl23_ci_flush). */
#define WL23_CI_LINES 128u
static struct wlstats wl_stats;
static unsigned wl23_ci_lines, wl23_ci_drop;
/* C2c W26: the RX and link arena state (fuxi_wlan_ce.h), the ring 120 setup
 * writes, and the D5 hint held back until after the W26 lines. */
static struct wlrx wl_rx;
static unsigned wl26_wbm_writes;
static bool wl26_heard_none;
static void wl26_watch(unsigned p);
/* C2d W27: the REO arena state, ring selection and the buffer return path (fuxi_wlan_ce.h). */
static struct wlrd wl_rd;
static unsigned wl27_reo_writes;
static int wl27_poll(void);
static void wl27_watch(const char *point);
static int wl27_setup(void);
static int wl27_dest(void);
static int wl27_sel(unsigned mac);
/* C2e W28: the monitor status arena state (fuxi_wlan_ce.h) and the crash line. */
static struct wlms wl_ms;
static int wl28_sring(unsigned m);
static void wl28_watch(unsigned p, const char *point);
static void wl28_report(void);
static void wl28_crash(const char *where);
static void wl27_report(void);
/* C2f W29: L19's dp_alloc order, cookie conversion and the common SRNGs (fuxi_wlan_ce.h). */
static struct wlcm wl_cm;
static unsigned wl29_writes, wl29_cmem_writes;
static int wl29_cc_init(void);
static int wl29_common_pre(void);
static int wl29_common_post(void);
static int wl29_cc_config(void);
static int wl29_macstart_pre(void);
static int wl29_macstart_post(void);
static int wl29_encap_decap(void);
static void wl29_report(void);

/* One register access through the dynamic window: the wl_soc_write
 * discipline without its per-access log line (pci.c:163-182, 1201-1285;
 * WCN7850 has no static window map, pci.c:1631-1635). Reached only from the
 * two allowlisted accessors below (CE: wlce_reg_ok; C2c WBM: wlrx_wbm_ok). */
static int
wl_win(uint32_t off, bool wr, uint32_t *value)
{
	uint32_t saved = wl_mrd(0x310c), win = (saved & ~0x3fu) | 0x40000000 | (off >> 19);
	if (saved == ~0u || fw_faults) { return 1; }
	wl_mwr(0x310c, win);
	int e = wl_mrd(0x310c) != win || fw_faults;
	if (!e) {
		uint64_t pa = WL_BAR0 + 0x80000ull + (off & 0x7ffff);
		if (wr) { wl_pwr(pa, *value); } else { *value = wl_prd(pa); }
		e = fw_faults != 0;
	}
	/* Containment writes remain possible after a latched synchronous fault. */
	if (wl_contain_wr(WL_BAR0 + 0x310c, saved) || wl_mrd(0x310c) != saved) { e = 1; }
	return e || fw_faults;
}

static int
wl_ce_window(uint32_t off, bool wr, uint32_t *value)
{
	if (fw_faults || !wlce_reg_ok(off)) { return 1; }
	return wl_win(off, wr, value);
}

static int wl_ce_io_wr(void *ctx, uint32_t off, uint32_t value) { (void)ctx; return wl_ce_window(off, true, &value); }
static int wl_ce_io_rd(void *ctx, uint32_t off, uint32_t *value) { (void)ctx; *value = ~0u; return wl_ce_window(off, false, value); }

static int
wl_ce_io_bar_wr(void *ctx, uint32_t off, uint32_t value)
{
	(void)ctx;
	if (fw_faults || off < WLCE_SHADOW_BASE || off >= WLCE_SHADOW_BASE + 4u * WLCE_SHADOW_REGS || (off & 3)) { return 1; }
	wl_mwr(off, value);
	return fw_faults ? 1 : 0;
}

/* dsb sy on both sides: at least L19's dma_rmb after a device pointer and its
 * wmb/mb before a pointer publish (hal.c:2154-2162, 2194-2215). */
static void wl_ce_wbarrier(void *ctx) { (void)ctx; __asm__ volatile("dsb sy" ::: "memory"); }
static void wl_ce_rbarrier(void *ctx) { (void)ctx; __asm__ volatile("dsb sy" ::: "memory"); }
static const struct wlce_io wl_ce_io = { wl_ce_io_wr, wl_ce_io_rd, wl_ce_io_bar_wr, wl_ce_wbarrier, wl_ce_rbarrier, NULL };

static const char *
wl_ce_lane(void)
{
	return wl12_stage == WL23_SCAN ? "W23" : wl12_stage == WL22_WMI ? "W22" : wl12_stage == WL21_HTC ? "W21" : "W20";
}

/* Failure evidence, RAM only: host ring pointers next to their rdp words,
 * buffers, HTC credits and the last WMI event; then the six-read diagnostic. */
static void
wl_ce_snapshot(const char *stage)
{
	printf("FUXI WLAN: W20 snapshot %s: writes %u errors %u stopping %u why %s\n", stage, wl_ce.writes,
	    wl_ce.errors, wl_ce.stopping, wl_ce.why ? wl_ce.why : "-");
	for (unsigned i = 0; wl_ce.rdp && i < WLCE_NRING; i++) {
		const struct wlce_ring *r = wl_ce.ring[i];
		if (r) { printf("FUXI WLAN: W20 snapshot ring %u hp %u tp %u rdp %u seen %u\n", r->id, r->hp, r->tp, wl_ce.rdp[r->id], r->seen); }
	}
	printf("FUXI WLAN: W20 snapshot tx CE0 %u/%u CE3 %u/%u", wl_ce.pipe[0].posted, wl_ce.pipe[0].reaped,
	    wl_ce.pipe[3].posted, wl_ce.pipe[3].reaped);
	if (wl_ce.pipe[4].posted) { printf(" CE4 %u/%u", wl_ce.pipe[4].posted, wl_ce.pipe[4].reaped); } /* C2 w20 0x02 */
	printf(" rx CE1 %u/%u CE2 %u/%u (posted/reaped)\n", wl_ce.pipe[1].posted, wl_ce.pipe[1].reaped,
	    wl_ce.pipe[2].posted, wl_ce.pipe[2].reaped);
	printf("FUXI WLAN: W20 snapshot htc ready %u size %u count %u eid:svc/credits", wl_htc.ready, wl_htc.credit_size, wl_htc.credit_count);
	for (unsigned e = 0; e < WLHTC_EPS; e++) {
		if (wl_htc.ep[e].used) { printf(" %u:%#x/%d", e, wl_htc.ep[e].svc, wl_htc.ep[e].credits); }
	}
	printf(" bad-reports %u htt-rx %u why %s\n", wl_htc.bad_reports, wl_htc.htt_rx, wl_htc.why ? wl_htc.why : "-");
	printf("FUXI WLAN: W20 snapshot wmi events %u unknown %u last %#x bytes %u svc %u avail %u ext %u ext2 %u ready %u why %s\n",
	    wl_wmi.events, wl_wmi.unknown_events, wl_wmi.last_event, wl_wmi.last_len, wl_wmi.got_svc, wl_wmi.got_avail,
	    wl_wmi.got_ext, wl_wmi.got_ext2, wl_wmi.got_ready, wl_wmi.why ? wl_wmi.why : "-");
	if (wl_wmi.scan) {
		printf("FUXI WLAN: W20 snapshot scan started %u done %u start-failed %u events %u foreign %u chan-info %u mgmt-rx %u frames %u bss %u why %s\n",
		    wl_scan.started, wl_scan.done, wl_scan.start_failed, wl_scan.events, wl_scan.foreign, wl_scan.ci_start,
		    wl_scan.mgmt, wl_scan.frames, wl_scan.nbss, wl_scan.why ? wl_scan.why : "-");
	}
	wl_wlfw_diagnostics("W20 snapshot");
}

/* A pure-layer refusal: the most specific reason, the snapshot, contain. */
static int
wl_ce_fail(const char *what)
{
	const char *why = wl_wmi.why ? wl_wmi.why : wl_htc.why ? wl_htc.why : wl_ce.why ? wl_ce.why : what;
	printf("FUXI WLAN: %s %s FAILED: %s\n", wl_ce_lane(), what, why);
	wl_ce_snapshot(what);
	return wl_w11_fail(why);
}

/* w20 0x04: endpoint PCIe Device Control MRRS (14:12) = 0, 128 bytes, before
 * CE init (an observation from UniXNU-sheng 186abef, not Linux). Device
 * Status (31:16) is W1C, so only the low half is written. */
static int
wl_ce_mrrs128(void)
{
	unsigned cap = wl_m6o_cap(0x10);
	uint32_t old = cap ? wl_prd(WL_EPCFG + cap + 8) : ~0u;
	if (!cap || old == ~0u || fw_faults) { return wl_w11_fail("W20 MRRS128: no PCIe capability"); }
	wl_pwr(WL_EPCFG + cap + 8, old & 0xffffu & ~0x7000u);
	uint32_t rb = wl_prd(WL_EPCFG + cap + 8);
	printf("FUXI WLAN: W20 endpoint devctl cap %#x MRRS 128: old %#x read %#x\n", cap, old & 0xffffu, rb & 0xffffu);
	return fw_faults || rb == ~0u || (rb & 0x7000u) ? wl_w11_fail("W20 MRRS128 readback") : 0;
}

/* The 512 KiB arena, aligned to its size like W13/W14 align target segments,
 * Normal-NC, mapped leaf by leaf with live SMMU readback (wl12_map_target). */
static int
wl_ce_alloc(void)
{
	uint64_t align = WLCE_ARENA_BYTES;
	if (wl_ce_arena.va) { return wl_w11_fail("W20 CE arena already allocated"); }
	wl_iova_next = (wl_iova_next + align - 1) & ~(align - 1);
	if (wl_dma_alloc(&wl_ce_arena, WLCE_ARENA_BYTES) || wl_dma_nc(&wl_ce_arena) ||
	    wl12_map_target(&wl_ce_arena)) { return wl_w11_fail("W20 CE arena allocation/NC/map"); }
	if (wlce_layout(&wl_ce, (uint8_t *)wl_ce_arena.va, wl_ce_arena.iova, (uint32_t)wl_ce_arena.size,
	    wl15_shadow_v3, wl15_variant ? 0u : 36u, !wl15_variant)) { return wl_ce_fail("CE layout"); }
	printf("FUXI WLAN: W20 ce arena IOVA %#llx bytes %#x Normal-NC mapped; rdp %#llx\n",
	    wl_ce_arena.iova, (unsigned)wl_ce_arena.size, wl_ce.rdp_iova);
	if (wl15_variant) {
		printf("FUXI WLAN: W20 ce shadow off (fuxiwlanw15=1): pointers go to R2 through the window\n");
		return 0;
	}
	printf("FUXI WLAN: W20 ce shadow");
	for (unsigned i = 0; i < WLCE_NRING; i++) { printf(" %u", wl_ce.ring[i]->shadow_idx); }
	printf(" (from WLAN_CFG table)\n");
	return 0;
}

/* Seven rings in L19 order (ce.c:912-971): CE0 SRC, CE1 DST + DEST_CTRL, CE1
 * STATUS, CE2 DST + DEST_CTRL, CE2 STATUS, CE3 SRC, CE4 SRC; BASE_LSB and MISC
 * read back after each, DEST_CTRL after its write. */
static int
wl_ce_init_rings(void)
{
	for (unsigned i = 0; i < WLCE_NRING; i++) {
		struct wlce_ring *r = wl_ce.ring[i];
		uint32_t base = 0, misc = 0, dctl = 0;
		if (wlce_ring_init(&wl_ce, &wl_ce_io, r) ||
		    (r->kind == WLCE_DST && wlce_dest_ctrl(&wl_ce, &wl_ce_io, r, WLCE_BUF, &dctl)) ||
		    wlce_ring_check(&wl_ce, &wl_ce_io, r, &base, &misc)) {
			printf("FUXI WLAN: W20 ce ring %u setup after %u writes (base %#x misc %#x dest-ctrl %#x)\n", r->id,
			    wl_ce.writes, base, misc, dctl);
			return wl_ce_fail("CE ring setup");
		}
		printf("FUXI WLAN: W20 ce ring %u entries %u base %#x misc %#x", r->id, r->entries, base, misc);
		if (r->kind == WLCE_DST) { printf(" dest-ctrl %#x", dctl); }
		printf(" readback ok\n");
	}
	printf("FUXI WLAN: W20 ce init ok rings %u writes %u window restore checked\n", WLCE_NRING, wl_ce.writes);
	return wl_w11_guard();
}

enum { WL20_W_CTL, WL20_W_TX, WL20_W_CREDITS, WL20_W_SVC, WL20_W_AVAIL, WL20_W_EXT, WL20_W_EXT2, WL20_W_READY, WL20_W_NONE,
    WL23_W_HTT, WL23_W_STARTED, WL23_W_STATS };

static bool
wl_ce_done(unsigned what)
{
	switch (what) {
	case WL20_W_CTL: return wl_htc.ctl_new;
	case WL20_W_TX: return wlce_tx_idle(&wl_ce.pipe[wl_ce_tx_pipe]);
	case WL20_W_CREDITS: return wlhtc_has_credits(&wl_htc, wl_ce_need_eid, wl_ce_need_len);
	case WL20_W_SVC: return wl_wmi.got_svc;
	case WL20_W_AVAIL: return wl_wmi.got_avail;
	case WL20_W_EXT: return wl_wmi.got_ext;
	case WL20_W_EXT2: return wl_wmi.got_ext2;
	case WL20_W_READY: return wl_wmi.got_ready;
	case WL23_W_HTT: return wl23_htt_seen;
	case WL23_W_STARTED: return wl_scan.started || wl_scan.start_failed;
	case WL23_W_STATS: return wl_stats.have;
	default: return false;
	}
}

/* C2b: one line per CHAN_INFO the scan parser took, the words of struct
 * wmi_chan_info_event as received (wmi.h:4586-4601). L19 builds the survey
 * from a START_RESP's cycle_count and rx_clear_count alone (time = cycle /
 * (mac_clk_mhz x 1000) ms, busy likewise) and only notes an END_RESP
 * (wmi.c:7514-7561); what the firmware puts in each is for the log to show,
 * so START and END are printed raw, no delta. One printf per line (the
 * scan prints up to 128 of them); other flags values in hex. WL23_CI_LINES
 * lines at most, the rest counted for wl23_ci_flush. */
static void
wl23_ci_line(void)
{
	const uint32_t *c = wl_scan.ci;
	uint32_t fl = c[WLWMI_CI_CMD_FLAGS];
	char odd[12];
	const char *fs = fl == WLWMI_CHAN_INFO_END_RESP ? "END" : fl == WLWMI_CHAN_INFO_START_RESP ? "START" : odd;
	if (wl23_ci_lines >= WL23_CI_LINES) { wl23_ci_drop++; return; }
	wl23_ci_lines++;
	if (fs == odd) { snprintf(odd, sizeof(odd), "%#x", fl); }
	printf("FUXI WLAN: W23 chan-info freq %u flags %s nf %d rx-clear %u cycle %u rx-frame %u tx-frame %u my-bss-rx %u mac-clk %u\n",
	    c[WLWMI_CI_FREQ], fs, (int)c[WLWMI_CI_NOISE_FLOOR],
	    c[WLWMI_CI_RX_CLEAR_COUNT], c[WLWMI_CI_CYCLE_COUNT], c[WLWMI_CI_RX_FRAME_COUNT], c[WLWMI_CI_TX_FRAME_CNT],
	    c[WLWMI_CI_MY_BSS_RX_CYCLE_COUNT], c[WLWMI_CI_MAC_CLK_MHZ]);
}

/* WMI events: the handled ones fill wl_wmi; the rest are logged by id and
 * length (16 lines at most; with qrtr 7 the first 0x402b also past that)
 * and never fail the run. In the W23 scan stage
 * SCAN, CHAN_INFO and MGMT_RX feed wl_scan; each new foreign channel gets a
 * line (64 at most), each CHAN_INFO a C2b detail line. In the C2b stats wait
 * UPDATE_STATS feeds wl_stats and gets no line of its own here. */
static int
wl_ce_wmi_rx(void *ctx, const uint8_t *msg, uint32_t len)
{
	uint32_t id = 0, foreign = wl_scan.foreign, ci = wl_scan.ci_seen;
	(void)ctx;
	if (wlwmi_rx(&wl_wmi, msg, len, &id)) { return 1; }
	if (wl_wmi.scan && wlscan_known(id)) {
		if (wl_scan.foreign != foreign && wl23_lines++ < 64) { printf("FUXI WLAN: W23 scan foreign freq %u\n", wl_scan.ev_freq); }
		if (wl_scan.ci_seen != ci) { wlrx_note_ci(&wl_rx, wl_scan.ci); wl23_ci_line(); }  /* C2c: D4/D5 notes, no line */
		return 0;
	}
	if (wl_wmi.stats && wlstats_known(id)) { return 0; }
	if (!wlwmi_known(id)) {
		/* C2: L19 moves its MGMT_RX noise floor on 0x402b (wmi.c:9740-9796,
		 * 7183-7187); C2 keeps -95, counts it and logs the first past the cap. */
		bool conv = wl_w23 && id == WLWMI_EV_PDEV_RSSI_DBM_CONVERSION && !wl23_rssi_conv++;
		if (wl_ce_logged++ < 16 || conv) {
			printf("FUXI WLAN: %s wmi event %#x bytes %u (logged only)\n", wl_ce_lane(), id, len - 4);
		}
	}
	return 0;
}

static int wl_ce_rx(void *ctx, const uint8_t *buf, uint32_t len) { (void)ctx; return wlhtc_rx(&wl_htc, buf, len, wl_ce_wmi_rx, NULL); }

/* One poll pass: MHI events, QMI indications and the AMSS/M0/SMMU guard
 * (W11), RX on CE2 then CE1 (one HTC handler for both, ce.c:154-170), TX
 * completions on CE0/CE3, and on CE4 once something was sent there (C2 w20
 * 0x02 only; L19 polls CE4 completions too, ce.c:717-760). A W11 events or
 * guard failure also marks the transport unhealthy for wl23_stop_scan. C2d:
 * then the REO and release rings and the refill replenish (w20 0x1000 only). */
static int
wl_ce_poll(void)
{
	if (wl_w11_events()) { wl23_guard_bad = true; return 1; }
	if (wlce_reap_rx(&wl_ce, &wl_ce_io, &wl_ce.pipe[2], wl_ce_rx, NULL) ||
	    wlce_reap_rx(&wl_ce, &wl_ce_io, &wl_ce.pipe[1], wl_ce_rx, NULL) ||
	    wlce_reap_tx(&wl_ce, &wl_ce_io, &wl_ce.pipe[0]) ||
	    wlce_reap_tx(&wl_ce, &wl_ce_io, &wl_ce.pipe[3]) ||
	    (wl_ce.pipe[4].posted && wlce_reap_tx(&wl_ce, &wl_ce_io, &wl_ce.pipe[4]))) { return wl_ce_fail("CE poll"); }
	if (wl_rd.reo_on && wl27_poll()) { return 1; }  /* C2d: REO/release rings, buffers back to ring 160 */
	if (wl_w11_guard()) { wl23_guard_bad = true; return 1; }
	return 0;
}

/* A reply is due: poll every 1 ms, at most MS ms, then contain. */
static int
wl_ce_wait(unsigned what, unsigned ms, const char *stage)
{
	for (unsigned t = 0; t <= ms; t++) {
		if (wl_ce_poll()) { return 1; }
		if (wl_ce_done(what)) { wl_ce_waited = t; return 0; }
		if (t < ms) { wl_sleep(1); }
	}
	printf("FUXI WLAN: %s timeout %s (%u ms); contain, no retry\n", wl_ce_lane(), stage, ms);
	wl_ce_snapshot(stage);
	return wl_w11_fail(stage);
}

/* Nothing is due: poll every 10 ms for MS ms (early end when WHAT holds). */
static int
wl_ce_observe(unsigned what, unsigned ms)
{
	for (unsigned t = 0; t < ms; t += 10) {
		if (wl_ce_poll()) { return 1; }
		if (wl_ce_done(what)) { return 0; }
		wl_sleep(10);
	}
	return wl_ce_poll();
}

/* One HTC message at a time, as W11: credits first (3 s, wmi.c:435-456),
 * frame, slot copy and publish (wlce_send), then the TX drain. */
static int
wl_ce_send(uint32_t eid, const uint8_t *payload, uint32_t len, const char *what)
{
	static uint8_t frame[WLCE_BUF];
	uint32_t total = WLHTC_HDR + len;
	if (eid >= WLHTC_EPS || !wl_htc.ep[eid].used) { return wl_w11_fail("W21 send on an unconnected HTC endpoint"); }
	if (!wlhtc_has_credits(&wl_htc, eid, total)) {
		wl_ce_need_eid = eid; wl_ce_need_len = total;
		printf("FUXI WLAN: %s %s waits for %u HTC credits (has %d)\n", wl_ce_lane(), what,
		    wlhtc_credits_needed(&wl_htc, total), wl_htc.ep[eid].credits);
		if (wl_ce_wait(WL20_W_CREDITS, 3000, "HTC credits")) { return 1; }
	}
	wl_ce_tx_pipe = wl_htc.ep[eid].ul_pipe;
	if (wlhtc_frame(&wl_htc, eid, payload, len, frame, sizeof(frame), &total) ||
	    wlhtc_take_credits(&wl_htc, eid, total) ||
	    wlce_send(&wl_ce, &wl_ce_io, &wl_ce.pipe[wl_ce_tx_pipe], eid, frame, total)) { return wl_ce_fail(what); }
	return wl_ce_wait(WL20_W_TX, 1000, "TX drain");
}

/* HTC connect (htc.c:557-719): WMI control asks for credit flow with
 * RECV_ALLOC = all credits as a u8 (one WMI endpoint: single-pdev preferred
 * hw mode, wmi.c:10207-10211, htc.c:446-489, 763-777); every other service
 * disables credit flow (htc.c:607-613). */
static int
wl_ce_connect(uint32_t svc)
{
	uint8_t req[WLHTC_CONNECT_BYTES];
	bool flow = svc == WLHTC_SVC_WMI;
	uint32_t alloc = flow ? (uint32_t)(uint8_t)wl_htc.credit_count : 0u, eid = 0, status = 0;
	wlhtc_connect_req(req, svc, flow ? alloc << WLHTC_CONN_RECV_ALLOC_SHIFT : WLHTC_CONN_DISABLE_FLOW);
	if (wl_ce_send(0, req, sizeof(req), flow ? "HTC connect WMI" : "HTC connect HTT") ||
	    wl_ce_wait(WL20_W_CTL, 3000, "HTC connect reply")) { return 1; }
	if (wlhtc_parse_connect(&wl_htc, svc, flow, alloc, &eid, &status)) {
		printf("FUXI WLAN: W21 htc svc %#x status %u eid %u refused\n", svc, status, eid);
		return wl_ce_fail("HTC connect");
	}
	const struct wlhtc_ep *ep = &wl_htc.ep[eid];
	printf("FUXI WLAN: W21 htc svc %#x status %u eid %u maxmsg %u pipes %u/%u", svc, status, eid, ep->max_msg,
	    ep->ul_pipe, ep->dl_pipe);
	if (flow) { printf(" credits %d", ep->credits); }
	printf("\n");
	return 0;
}

/* ---- W23/W24: one passive scan (fuxiwlanqrtr=7, C2) ----
 * After W22: the HTT version request (w20 0x02 only, as core.c:955 right
 * after HW mode), the passive SCAN_CHAN_LIST (reg.c:183-250), a station vdev
 * with the fixed K-W2 address and what L19 sends right after creating one
 * (mac.c:10071-10156: NSS, three STA power-save parameters, power save off;
 * the encap/decap offload, TX power, RTS, 11d and DP attach steps are left
 * out: data path, transmit or regulatory only), one passive START_SCAN
 * (mac.c:5335-5388), then polling: SCAN_EVENT, CHAN_INFO and MGMT_RX feed a
 * 64-entry BSS table that W24 prints. Nothing is transmitted on the air: no
 * probe request, no ACTIVE bit, no data path, peer or key.
 * Adapted from Linux ath12k mac.c, reg.c, wmi.c and dp_tx.c
 * (BSD-3-Clause-Clear), v6.19; notice in bsd/dev/fuxi_wlan_ce.h. */
/* K-W2: the locally administered address the wifi-ios side gives en0
 * (fuxi_wifi_ios.c, both backends); never the phone's wlan_mac.bin. */
static const uint8_t wl23_mac[6] = { 0x02, 0x46, 0x55, 0x58, 0x49, 0x01 };

/* HTT messages (CE1): the VERSION_CONF the w20 0x02 exchange waits for; with a
 * W26 bit every message is also counted by type (installed at the HTT connect). */
static int
wl23_htt_rx(void *ctx, const uint8_t *msg, uint32_t len)
{
	(void)ctx;
	if (wl20_bits & WL26_BITS) { wlrx_htt_count(&wl_rx, msg, len); }
	if (wlhtt_version_conf(msg, len, &wl23_htt_major, &wl23_htt_minor)) { wl23_htt_seen = true; }
	return 0;
}

/* One fixed-TLV WMI command (wlwmi_encode_words) through wl_ce_send. */
static int
wl23_words(uint32_t cmd, uint32_t tag, const uint32_t *w, uint32_t nw, const char *what)
{
	uint8_t buf[8 + 4 * 5];
	uint32_t len = 0;
	if (nw > 5 || wlwmi_encode_words(cmd, tag, w, nw, buf, sizeof(buf), &len)) { return wl_w11_fail("W23 command encode bounds"); }
	return wl_ce_send(wl_htc.wmi_eid, buf, len, what);
}

/* The scan runs: poll every 1 ms until COMPLETED or DEQUEUED, at most MS ms.
 * 0 = over, 1 = the poll failed (already contained), 2 = timed out. C2c: the
 * W26 mid watch at the first pass with foreign >= (n + 1) / 2 (19 of 38, 7 of
 * 13); without a W26 ring it returns at once. */
static int
wl23_wait_done(unsigned ms, unsigned *took)
{
	for (unsigned t = 0; t <= ms; t++) {
		if (wl_ce_poll()) { return 1; }
		if (!wl_rx.w[WLRX_P_MID].taken && wl_scan.foreign >= (wl_scan.nchan + 1u) / 2u) { wl26_watch(WLRX_P_MID); }
		if (wl_scan.done) { *took = t; return 0; }
		if (t < ms) { wl_sleep(1); }
	}
	*took = ms;
	return 2;
}

/* W24: the BSS table, one line per BSSID (64 at most), SSIDs in clear and
 * every BSSID OUI-only, the last three octets masked (owner decision 6), then
 * the summary. table-full counts frames of BSSIDs past the 64th, not BSSIDs;
 * rssi-conv counts the 0x402b events whose noise floor L19 would apply. */
static void
wl24_report(void)
{
	static char ssid[WLSCAN_SSID_TEXT];
	for (uint32_t i = 0; i < wl_scan.nbss; i++) {
		const struct wlscan_bss *b = &wl_scan.bss[i];
		wlscan_ssid_text(b, ssid);
		printf("FUXI WLAN: W24 bss %02x:%02x:%02x:xx:xx:xx ch %u freq %u dBm %d privacy %u rsn %u %s frames %u %s\n",
		    b->bssid[0], b->bssid[1], b->bssid[2], b->chan, b->freq, (int)b->dbm, (unsigned)b->privacy,
		    (unsigned)b->rsn, b->beacon && b->probe_resp ? "beacon+probe-resp" : b->beacon ? "beacon" : "probe-resp",
		    b->frames, ssid);
	}
	printf("FUXI WLAN: W24 scan summary frames %u bss %u ssids %u\n", wl_scan.frames, wl_scan.nbss, wlscan_ssids(&wl_scan));
	printf("FUXI WLAN: W24 scan detail mgmt-rx %u hidden %u table-full frames %u dropped status %u channel %u malformed %u "
	    "other-mgmt %u ie-truncated %u noise-floor %d dBm (L19 default; rssi-conv events %u, not applied)\n", wl_scan.mgmt,
	    wlscan_hidden(&wl_scan), wl_scan.table_full, wl_scan.drop_status, wl_scan.drop_channel, wl_scan.drop_bad,
	    wl_scan.other_mgmt, wl_scan.ie_truncated, (int)wl_scan.noise_floor, wl23_rssi_conv);
}

/* ---- C2b (fuxiwlanqrtr=7): CHAN_INFO detail and the pdev stats requests ----
 * Diagnostics on top of C2, no knob of their own (docs-lanes/wlan-w20.md, C2b):
 * every CHAN_INFO gets a line (wl23_ci_line) and, after COMPLETED, one
 * WMI_REQUEST_STATS per pdev id of the preferred hw mode's PHYs (at most two,
 * one after the other) asks for pdev stats as L19's debugfs pdev_stats reader
 * does (debugfs.c:1388-1410, mac.c:5069-5115); with a vif up L19 would name
 * the pdev of that band (mac.c:967-1009), so with DBS each MAC is asked.
 * Host to firmware only, through the C1 rings and writers; nothing on the
 * air. Bounded: credits 3000 ms and the TX drain 1000 ms (wl_ce_send's
 * bounds), UPDATE_STATS 500 ms (L19: 1 s, then 3 s), per request. A refusal,
 * a timeout or an unusable event is logged and the run goes on to
 * containment; a failed poll or a CE/HTC fault in the send (the transport,
 * as in every W20..W23 wait and wl_ce_send) ends it. No vdev stats: L19
 * completes those only for started vdevs (wmi.c:8443-8470) and the C2 vdev
 * is never started.
 * Adapted from Linux ath12k wmi.c, mac.c and debugfs.c (BSD-3-Clause-Clear),
 * v6.19; notice in bsd/dev/fuxi_wlan_ce.h. */

/* Poll every 1 ms until WHAT holds, at most MS ms. Unlike wl_ce_wait a
 * timeout neither snapshots nor fails the run. 0 = done (after *TOOK ms),
 * 1 = the poll failed (already contained), 2 = timed out. */
static int
wl23_poll_for(unsigned what, unsigned ms, unsigned *took)
{
	for (unsigned t = 0; t <= ms; t++) {
		if (wl_ce_poll()) { return 1; }
		if (wl_ce_done(what)) { *took = t; return 0; }
		if (t < ms) { wl_sleep(1); }
	}
	*took = ms;
	return 2;
}

/* The messages the contain line counts: TX posted on CE0/CE3/CE4, RX reaped
 * on CE1/CE2 (each reaped buffer is posted again). */
static unsigned wl23_ce_tx(void) { return wl_ce.pipe[0].posted + wl_ce.pipe[3].posted + wl_ce.pipe[4].posted; }
static unsigned wl23_ce_rx(void) { return wl_ce.pipe[1].reaped + wl_ce.pipe[2].reaped; }

static void
wl23_ci_flush(void)
{
	if (wl23_ci_drop) { printf("FUXI WLAN: W23 chan-info lines suppressed %u (cap %u)\n", wl23_ci_drop, WL23_CI_LINES); }
	wl23_ci_drop = 0;
}

/* The pdev entry as L19's debugfs dump prints it (wmi.c:7893-8031, field
 * names of struct ath12k_wmi_pdev_stats_params): the event's counts, then
 * base, rx and tx, two lines each. The console keeps at most 255 bytes of a
 * line, newline included (serial_general.c:159-188 without drain_uart_sync),
 * so no line here can pass 254 bytes even with every field at its widest
 * (the host test computes it). */
static void
wl23_stats_report(unsigned took, unsigned tx0, unsigned rx0)
{
	const uint32_t *s = wl_stats.w, *e = wl_stats.ev;
	wlrx_note_stats(&wl_rx, s);  /* C2c D4: no line here */
	printf("FUXI WLAN: W23 stats pdev %u after %u ms (stats-id %#x pdevs %u vdevs %u bcns %u bytes %u events %u unusable %u; "
	    "ce tx %u rx %u)\n", e[WLWMI_SE_PDEV_ID], took, e[WLWMI_SE_STATS_ID], e[WLWMI_SE_NUM_PDEV_STATS],
	    e[WLWMI_SE_NUM_VDEV_STATS], e[WLWMI_SE_NUM_BCN_STATS], wl_stats.bytes, wl_stats.events, wl_stats.unusable,
	    wl23_ce_tx() - tx0, wl23_ce_rx() - rx0);
	printf("FUXI WLAN: W23 stats base chan-nf %d rx-frame %u rx-clear %u cycle %u tx-frame %u phy-err %u chan-tx-pwr %u\n",
	    (int)s[WLWMI_PS_CHAN_NF], s[WLWMI_PS_RX_FRAME_COUNT], s[WLWMI_PS_RX_CLEAR_COUNT], s[WLWMI_PS_CYCLE_COUNT],
	    s[WLWMI_PS_TX_FRAME_COUNT], s[WLWMI_PS_PHY_ERR_COUNT], s[WLWMI_PS_CHAN_TX_PWR]);
	printf("FUXI WLAN: W23 stats rx1 status-rcvd %d r0-frags %d r1-frags %d r2-frags %d r3-frags %d htt-msdus %d htt-mpdus %d\n",
	    (int)s[WLWMI_PS_STATUS_RCVD], (int)s[WLWMI_PS_R0_FRAGS], (int)s[WLWMI_PS_R1_FRAGS], (int)s[WLWMI_PS_R2_FRAGS],
	    (int)s[WLWMI_PS_R3_FRAGS], (int)s[WLWMI_PS_HTT_MSDUS], (int)s[WLWMI_PS_HTT_MPDUS]);
	printf("FUXI WLAN: W23 stats rx2 loc-msdus %d loc-mpdus %d oversize-amsdu %d phy-errs %d phy-err-drop %d mpdu-errs %d "
	    "mid-ppdu-route-change %d\n", (int)s[WLWMI_PS_LOC_MSDUS], (int)s[WLWMI_PS_LOC_MPDUS], (int)s[WLWMI_PS_OVERSIZE_AMSDU],
	    (int)s[WLWMI_PS_PHY_ERRS], (int)s[WLWMI_PS_PHY_ERR_DROP], (int)s[WLWMI_PS_MPDU_ERRS], (int)s[WLWMI_PS_MID_PPDU_ROUTE_CHANGE]);
	printf("FUXI WLAN: W23 stats tx1 comp-queued %d comp-delivered %d msdu-enqued %d mpdu-enqued %d local-enqued %d "
	    "local-freed %d hw-queued %d\n", (int)s[WLWMI_PS_COMP_QUEUED], (int)s[WLWMI_PS_COMP_DELIVERED],
	    (int)s[WLWMI_PS_MSDU_ENQUED], (int)s[WLWMI_PS_MPDU_ENQUED], (int)s[WLWMI_PS_LOCAL_ENQUED], (int)s[WLWMI_PS_LOCAL_FREED],
	    (int)s[WLWMI_PS_HW_QUEUED]);
	printf("FUXI WLAN: W23 stats tx2 hw-reaped %d underrun %d tx-abort %d mpdus-requed %d tx-ko %u self-triggers %u "
	    "pdev-resets %u\n", (int)s[WLWMI_PS_HW_REAPED], (int)s[WLWMI_PS_UNDERRUN], (int)s[WLWMI_PS_TX_ABORT],
	    (int)s[WLWMI_PS_MPDUS_REQUED], s[WLWMI_PS_TX_KO], s[WLWMI_PS_SELF_TRIGGERS], s[WLWMI_PS_PDEV_RESETS]);
}

/* One REQUEST_STATS (PDEV_STAT, vdev 0, PDEV) the way wl_ce_send sends
 * (credits, frame, slot, publish, drain), then up to 500 ms for a usable
 * UPDATE_STATS. The waits are bounded and their timeouts are not fatal; a
 * CE or HTC fault in the send is (wl_ce_fail, as wl_ce_send). Each ending
 * line carries the CE messages of the exchange (ce tx / rx), so the contain
 * line's counts reconcile with the C2 run's. 0 = over (answered or not),
 * 1 = a poll or the send failed (already contained), 2 = not sent or its TX
 * not drained (no further request). */
static int
wl23_stats_pdev(uint32_t pdev, uint32_t bands)
{
	static uint8_t cmd[WLWMI_REQUEST_STATS_TOTAL], frame[WLCE_BUF];
	const uint32_t eid = wl_htc.wmi_eid;
	const unsigned tx0 = wl23_ce_tx(), rx0 = wl23_ce_rx();
	const char *why = NULL;
	uint32_t len = 0, total = 0;
	unsigned took = 0;
	int r = 0;
	bzero(&wl_stats, sizeof(wl_stats));
	wl_wmi.stats = &wl_stats;
	if (wlwmi_encode_stats_req(pdev, cmd, sizeof(cmd), &len)) {
		why = "encode bounds";
	} else if (!wlhtc_has_credits(&wl_htc, eid, WLHTC_HDR + len)) {
		wl_ce_need_eid = eid; wl_ce_need_len = WLHTC_HDR + len;
		printf("FUXI WLAN: W23 stats request waits for %u HTC credits (has %d)\n",
		    wlhtc_credits_needed(&wl_htc, WLHTC_HDR + len), wl_htc.ep[eid].credits);
		r = wl23_poll_for(WL20_W_CREDITS, 3000, &took);  /* wl_ce_send's credit bound (wmi.c:435-456) */
		if (r == 1) { return 1; }
		if (r == 2) { why = "no HTC credits after 3000 ms"; }
	}
	if (why) {
		printf("FUXI WLAN: W23 stats request pdev-stat pdev %u bands %#x not sent: %s (ce tx %u rx %u); continue to containment\n",
		    pdev, bands, why, wl23_ce_tx() - tx0, wl23_ce_rx() - rx0);
		return 2;
	}
	wl_ce_tx_pipe = wl_htc.ep[eid].ul_pipe;
	if (wlhtc_frame(&wl_htc, eid, cmd, len, frame, sizeof(frame), &total) || wlhtc_take_credits(&wl_htc, eid, total) ||
	    wlce_send(&wl_ce, &wl_ce_io, &wl_ce.pipe[wl_ce_tx_pipe], eid, frame, total)) { return wl_ce_fail("WMI REQUEST_STATS"); }
	printf("FUXI WLAN: W23 stats request pdev-stat pdev %u bands %#x vdev 0 sent\n", pdev, bands);
	r = wl23_poll_for(WL20_W_TX, 1000, &took);
	if (r == 1) { return 1; }
	if (r == 2) {
		printf("FUXI WLAN: W23 stats request TX not drained after 1000 ms (ce tx %u rx %u); continue to containment\n",
		    wl23_ce_tx() - tx0, wl23_ce_rx() - rx0);
		if (wl_stats.have) { wl23_stats_report(took, tx0, rx0); }  /* answered meanwhile: already parsed and bounded */
		return 2;
	}
	r = wl23_poll_for(WL23_W_STATS, 500, &took);
	if (r == 1) { return 1; }
	if (r == 2) {
		printf("FUXI WLAN: W23 stats pdev %u no usable UPDATE_STATS in 500 ms (events %u unusable %u%s%s; ce tx %u rx %u)\n",
		    pdev, wl_stats.events, wl_stats.unusable, wl_stats.why ? ", last: " : "", wl_stats.why ? wl_stats.why : "",
		    wl23_ce_tx() - tx0, wl23_ce_rx() - rx0);
		return 0;
	}
	wl23_stats_report(took, tx0, rx0);
	return 0;
}

/* After COMPLETED: a request per pdev id of the preferred hw mode's PHYs
 * (none kept: L19's zeroed fw_pdev[0], pdev 0), the next only after an
 * exchange that went out and drained; then what the scan parser took
 * meanwhile (wl_wmi.scan stays attached: a late MGMT_RX, CHAN_INFO or scan
 * event is counted here, the W24 lines are not printed again). 1 = a poll
 * or the send failed (already contained). */
static int
wl23_stats(void)
{
	const uint32_t mgmt = wl_scan.mgmt, frames = wl_scan.frames, nbss = wl_scan.nbss, events = wl_scan.events,
	    ci = wl_scan.ci_seen, other = wl_wmi.unknown_events, n = wl_wmi.phy_pdevs ? wl_wmi.phy_pdevs : 1u;
	int r = 0;
	for (uint32_t i = 0; i < n && r == 0; i++) { r = wl23_stats_pdev(wl_wmi.phy_pdev[i], wl_wmi.phy_pdev_bands[i]); }
	if (r == 1) { return 1; }
	printf("FUXI WLAN: W23 stats after-scan wmi mgmt-rx +%u frames +%u bss +%u scan-events +%u chan-info +%u other +%u\n",
	    wl_scan.mgmt - mgmt, wl_scan.frames - frames, wl_scan.nbss - nbss, wl_scan.events - events, wl_scan.ci_seen - ci,
	    wl_wmi.unknown_events - other);
	return 0;
}

/* ---- C2c (fuxiwlanqrtr=7): W26, host RX rings, the WBM idle list and the chain masks ----
 * w20 0x100 RXRING, 0x200 IDLELINK, 0x400 CHAINMASK, all default off
 * (docs-lanes/wlan-w20.md, C2c; the pure layout, encoders and counts are in
 * fuxi_wlan_ce.h). In L19's order: at W20, after the seven CE rings and before
 * the CE RX buffers (dp_alloc between ce_init_pipes and core_start,
 * core.c:1285-1300), both arenas, then the idle list (pre-reads, ring 120,
 * entries, HP) as dp_alloc does before its RX alloc (dp.c:1755-1768, 1813),
 * then the refill ring (sentinels, 4095 entries, wrp[0]); at W23, right after
 * VERSION_CONF, the four SRING_SETUP (dp_rx.c:4616-4646), a 100 ms look, the
 * setup watch and the two chain masks (mac.c:9418, before the channel list at
 * 9422). Watches at setup, STARTED, the mid FOREIGN_CHAN and the end of the
 * COMPLETED wait; after the C2b lines the content, guard, error-ring, link and
 * counter lines, then (D5) the no-beacon hint with the next rung of
 * rx_path_plan.md 2.11. Register accesses: only wlrx_wbm_ok's (the nine
 * pre-reads, the eight ring 120 writes, two readbacks, R2 HP/TP per watch)
 * through wl_win, and one HP publish to the shadow register WLAN_CFG gave ring
 * 120 (wl15_shadow_v3 index 19, BAR0 0x948) or, with fuxiwlanw15=1, R2 HP;
 * nothing after wl26_contain. No interrupt, MSI, GIC, power or PMIC access; the
 * W11 device wake stays held as for every W20 access. Both arenas are held until
 * bus master off, PERST and the SMMU undo are proven, then counted, zeroed and
 * freed (wl26_free), else quarantined with the rest.
 * Adapted from Linux ath12k dp.c, dp_rx.c, dp_tx.c, hal.c, mac.c and wmi.c
 * (BSD-3-Clause-Clear), v6.19; notice in bsd/dev/fuxi_wlan_ce.h. */
static const char *const wl26_ring_name[WLRX_NRING] = { "refill", "mac0", "mac1", "rxdma-err" };
static const char *const wl26_err_code[20] = { "overflow", "mpdu-len", "fcs", "decrypt", "tkip-mic", "unencrypted",
	"msdu-len", "msdu-limit", "wifi-parse", "amsdu-parse", "sa-timeout", "da-timeout", "flow-timeout", "flush-request",
	"amsdu-frag", "mcast-echo", "amsdu-mismatch", "unauth-wds", "grpcast-amsdu-wds", "other" };

/* The WBM accessor: wlrx_wbm_ok's allowlist and the wl_win discipline;
 * refused after wl26_contain. */
static int
wl26_wbm(uint32_t off, bool wr, uint32_t *value)
{
	if (fw_faults || wl_rx.stopping || !wlrx_wbm_ok(off, wr)) { return 1; }
	return wl_win(off, wr, value);
}

/* A read that fails or returns ~0u is "unreadable" (~0u): never retried. */
static uint32_t
wl26_wbm_rd(uint32_t off)
{
	uint32_t v = ~0u;
	return wl26_wbm(off, false, &v) ? ~0u : v;
}

/* One wired arena, IOVA aligned to 2 MiB, Normal-NC, mapped leaf by leaf with
 * live SMMU readback (wl12_map_target, as wl_ce_alloc). */
static int
wl26_alloc(struct wl_dma *d, uint32_t bytes, const char *what)
{
	const uint64_t align = 0x200000ull;
	if (d->va) { return wl_w11_fail(what); }
	wl_iova_next = (wl_iova_next + align - 1) & ~(align - 1);
	if (wl_dma_alloc(d, bytes) || wl_dma_nc(d) || wl12_map_target(d)) { return wl_w11_fail(what); }
	return 0;
}

/* W26b: the pre-reads (refuse on an owner or an unreadable register: no WBM
 * write at all, 0x200 dropped for this boot), ring 120's eight writes in L19's
 * order, BASE_LSB and MISC read back, the 32767 entries, the write barrier and
 * the HP: the shadow register WLAN_CFG gave ring 120 (looked up, so CFG and W26
 * cannot disagree) or, with fuxiwlanw15=1, R2 HP through the window. */
static int
wl26_idle(void)
{
	uint32_t regs[WLRX_IDLE_REGS][2], rb = ~0u, hp = WLRX_NDESC * WLRX_IDLE_DW, idx = WLCE_SHADOW_REGS;
	const uint32_t *p = wl_rx.wbm_pre;
	char via[32];
	wl_rx.idle_tried = true;
	for (unsigned i = 0; i < WLRX_PRE_N; i++) { wl_rx.wbm_pre[i] = wl26_wbm_rd(wlrx_pre_addr(i)); }
	wl_rx.idle_own = wlrx_idle_owned(wl_rx.wbm_pre);
	printf("FUXI WLAN: W26 wbm idle before: base-lsb %#x base-msb %#x misc %#x r2-hp %#x r2-tp %#x idle-list-cntl %#x "
	    "idle-list-size %#x scatter-base %#x/%#x; %s\n", p[WLRX_PRE_BASE_LSB], p[WLRX_PRE_BASE_MSB], p[WLRX_PRE_MISC],
	    p[WLRX_PRE_R2_HP], p[WLRX_PRE_R2_TP], p[WLRX_PRE_LIST_CNTL], p[WLRX_PRE_LIST_SIZE], p[WLRX_PRE_SCAT_LSB],
	    p[WLRX_PRE_SCAT_MSB], wl_rx.idle_own == WLRX_IDLE_FREE ? "not owned" : "refused");
	if (wl_rx.idle_own != WLRX_IDLE_FREE) {
		printf("FUXI WLAN: W26 wbm idle list not programmed: %s; w20 0x200 dropped for this boot\n", wlrx_idle_why(wl_rx.idle_own));
		return 0;
	}
	wlrx_idle_regs(&wl_rx, regs);
	for (unsigned i = 0; i < WLRX_IDLE_REGS; i++) {
		if (wl26_wbm(regs[i][0], true, &regs[i][1])) {
			printf("FUXI WLAN: W26 wbm idle write %u of %u (%#x) failed; contain\n", i + 1u, WLRX_IDLE_REGS, regs[i][0]);
			return wl_w11_fail("W26 wbm idle register write");
		}
		wl26_wbm_writes++;
	}
	for (unsigned i = 0; i < 2; i++) {  /* BASE_LSB (write 1) exact, MISC (write 8) with its three bits */
		const unsigned k = i ? WLRX_IDLE_REGS - 1u : 0u;
		rb = wl26_wbm_rd(regs[k][0]);
		if (rb == ~0u || (i ? (rb & WLRX_WBM_MISC) != WLRX_WBM_MISC : rb != regs[k][1])) {
			printf("FUXI WLAN: W26 wbm idle readback mismatch %s wrote %#x read %#x\n", i ? "misc" : "base-lsb", regs[k][1], rb);
			return wl_w11_fail("W26 wbm idle readback mismatch");
		}
	}
	if (wlrx_link_fill(&wl_rx)) { return wl_w11_fail("W26 link fill"); }
	for (unsigned j = 0; j < sizeof(wl15_shadow_v3) / sizeof(wl15_shadow_v3[0]); j++) {
		if (wl15_shadow_v3[j] == WLRX_WBM_IDLE_HP) { idx = j; }
	}
	if (wl15_variant) { snprintf(via, sizeof(via), "R2 %#x (fuxiwlanw15=1)", WLRX_WBM_IDLE_HP); }
	else { snprintf(via, sizeof(via), "shadow %u (%#x)", idx, WLCE_SHADOW_BASE + 4u * idx); }
	wl_ce_wbarrier(NULL);  /* entries and descriptors visible before the HP (hal.c:2194-2215) */
	if (wl15_variant ? wl26_wbm(WLRX_WBM_IDLE_HP, true, &hp) :
	    idx >= WLCE_SHADOW_REGS || wl_rx.stopping || wl_ce_io_bar_wr(NULL, WLCE_SHADOW_BASE + 4u * idx, hp)) {
		return wl_w11_fail("W26 wbm idle HP publish");
	}
	wl_rx.idle_on = true;
	printf("FUXI WLAN: W26 wbm idle ring 120 base %#x %u words, %u link descriptors (rbm 1, our cookie) misc %#x readback ok, "
	    "hp %u via %s; writes %u window restore checked\n", regs[0][1], WLRX_IDLE_ENTRIES * WLRX_IDLE_DW, WLRX_NDESC, rb, hp,
	    via, wl26_wbm_writes);
	return 0;
}

/* W20 part: both arenas before any WBM write (a failed allocation leaves the
 * WBM untouched), the idle list, then the refill ring. Silent without 0x100 and
 * 0x200. */
static int
wl26_setup(void)
{
	bzero(&wl_rx, sizeof(wl_rx));
	bzero(&wl_rd, sizeof(wl_rd));  /* C2d */
	bzero(&wl_ms, sizeof(wl_ms));  /* C2e */
	bzero(&wl_cm, sizeof(wl_cm));  /* C2f */
	wl26_wbm_writes = wl27_reo_writes = 0;
	wl29_writes = wl29_cmem_writes = 0;
	wl26_heard_none = false;
	if (!(wl20_bits & (WL20_RXRING | WL20_IDLELINK))) { return 0; }
	if (wl20_bits & WL20_IDLELINK) {
		if (wl26_alloc(&wl_link_arena, WLRX_LINK_BYTES, "W26 link arena allocation/NC/map")) { return 1; }
		if (wlrx_link_layout(&wl_rx, (uint8_t *)wl_link_arena.va, wl_link_arena.iova, (uint32_t)wl_link_arena.size)) {
			return wl_w11_fail("W26 link arena layout");
		}
		printf("FUXI WLAN: W26 link arena IOVA %#llx bytes %#x Normal-NC mapped\n", wl_link_arena.iova, (unsigned)wl_link_arena.size);
	}
	if (wl20_bits & WL20_RXRING) {
		if (wl26_alloc(&wl_rx_arena, WLRX_ARENA_BYTES, "W26 rx arena allocation/NC/map")) { return 1; }
		if (wlrx_layout(&wl_rx, (uint8_t *)wl_rx_arena.va, wl_rx_arena.iova, (uint32_t)wl_rx_arena.size)) {
			return wl_w11_fail("W26 rx arena layout");
		}
		printf("FUXI WLAN: W26 rx arena IOVA %#llx bytes %#x Normal-NC mapped; rdp %#llx wrp %#llx\n", wl_rx_arena.iova,
		    (unsigned)wl_rx_arena.size, wl_rx.rdp_iova, wl_rx.wrp_iova);
	}
	if (wl20_bits & WL20_REORX) {  /* C2d: the third arena, still before any WBM or REO write */
		if (wl26_alloc(&wl_reo_arena, WLRD_ARENA_BYTES, "W27 reo arena allocation/NC/map")) { return 1; }
		if (wlrd_layout(&wl_rd, (uint8_t *)wl_reo_arena.va, wl_reo_arena.iova, (uint32_t)wl_reo_arena.size)) {
			return wl_w11_fail("W27 reo arena layout");
		}
		printf("FUXI WLAN: W27 reo arena IOVA %#llx bytes %#x Normal-NC mapped; rdp %#llx\n", wl_reo_arena.iova,
		    (unsigned)wl_reo_arena.size, wl_rd.rdp_iova);
	}
	if (wl20_bits & WL20_MONSTAT) {  /* C2e: the fourth arena, still before any WBM or REO write */
		if (wl26_alloc(&wl_mon_arena, WLMS_ARENA_BYTES, "W28 mon arena allocation/NC/map")) { return 1; }
		if (wlms_layout(&wl_ms, &wl_rx, (uint8_t *)wl_mon_arena.va, wl_mon_arena.iova, (uint32_t)wl_mon_arena.size)) {
			return wl_w11_fail("W28 mon arena layout");
		}
		printf("FUXI WLAN: W28 mon arena IOVA %#llx bytes %#x Normal-NC mapped; rings %#llx %#llx slots %#llx\n", wl_mon_arena.iova,
		    (unsigned)wl_mon_arena.size, wl_ms.ring[0].iova, wl_ms.ring[1].iova, wl_ms.bufs_iova);
	}
	if (wl20_bits & WL20_L19ORDER) {  /* C2f: the fifth arena (SPT pages + common rings), still before any WBM/REO write */
		if (wl26_alloc(&wl_cm_arena, WLCM_ARENA_BYTES, "W29 cm arena allocation/NC/map")) { return 1; }
		if (wlcm_layout(&wl_cm, (uint8_t *)wl_cm_arena.va, wl_cm_arena.iova, (uint32_t)wl_cm_arena.size)) {
			return wl_w11_fail("W29 cm arena layout");
		}
		printf("FUXI WLAN: W29 arena IOVA %#llx bytes %#x Normal-NC mapped; rdp %#llx spt %#llx\n", wl_cm_arena.iova,
		    (unsigned)wl_cm_arena.size, wl_cm.rdp_iova, wl_cm.spt_iova);
	}
	if ((wl20_bits & WL20_IDLELINK) && wl26_idle()) { return 1; }
	if (wl29_cc_init()) { return 1; }   /* C2f: cookie converter, after the idle list and before the common SRNGs (dp_alloc order) */
	if (wl27_setup()) { return 1; }  /* C2d: release and exception rings, REO setup (dp_alloc's common setup, before its RX alloc) */
	if (wl20_bits & WL20_RXRING) {
		if (wlrx_fill_refill(&wl_rx) || wlrx_publish_refill(&wl_rx, &wl_ce_io)) { return wl_w11_fail("W26 refill fill"); }
		wl_rx.rx_on = true;
		wlrd_slots_init(&wl_rd);  /* C2d: slots 0..4094 out with the device, the refill HP as published */
		printf("FUXI WLAN: W26 refill ring 160 filled %u buffers (rbm 6, 2048 B at a 4096 B stride, sentinel 0xa5 x64, "
		    "slots %#llx-%#llx) hp %u in wrp[0]\n", WLRX_NBUF, wl_rx.bufs_iova,
		    wl_rx.bufs_iova + (uint64_t)(WLRX_NBUF - 1u) * WLRX_SLOT_BYTES, WLRX_NBUF * WLRX_BUF_DW);
	}
	for (unsigned m = 0; (wl20_bits & WL20_MONSTAT) && m < 2u; m++) {  /* C2e: after the refill (ath12k_dp_rxdma_buf_setup) */
		if (wlms_fill(&wl_ms, &wl_rx, m) || wlms_publish(&wl_ms, &wl_rx, &wl_ce_io, m)) { return wl_w11_fail("W28 mon status fill"); }
		printf("FUXI WLAN: W28 mon status ring %u filled %u buffers (rbm 6, cookie 0-%u, 2048 B at a 4096 B stride, sentinel 0xa5 "
		    "x64, slots %#llx-%#llx) hp %u in wrp[%u]\n", wl_ms.ring[m].id, WLMS_NBUF, WLMS_NBUF - 1u, wlms_slot_iova(&wl_ms, m, 0),
		    wlms_slot_iova(&wl_ms, m, WLMS_NBUF - 1u), wl_ms.hp[m], wl_ms.ring[m].id - WLRX_DMAC_START);
	}
	return wl_w11_guard();
}

/* One W26c message through wl_ce_send on the HTT endpoint (CE4, one in
 * flight, TX drained), no reply asked. */
static int
wl26_sring(unsigned i)
{
	uint32_t w[WLRX_SRING_WORDS];
	uint8_t msg[4u * WLRX_SRING_WORDS];
	const uint32_t eid = wl_htc.htt_eid;
	const struct wlce_pipe *p = &wl_ce.pipe[wl_htc.ep[eid].ul_pipe];
	const char *why = !eid || !wl_htc.ep[eid].used ? "no HTT endpoint" : wl_rx.stopping ? "stopping" :
	    p->posted - p->reaped >= p->nslots ? "CE4 slots all in flight" : NULL;
	wlrx_sring_words(&wl_rx, i, w);
	if (why) {
		printf("FUXI WLAN: W26 htt sring setup %s %u not sent: %s\n", wl26_ring_name[i], wl_rx.ring[i].id, why);
		return wl_w11_fail("W26 htt sring setup not sent");
	}
	for (unsigned k = 0; k < WLRX_SRING_WORDS; k++) { wlce_put32(msg + 4u * k, w[k]); }
	if (wl_ce_send(eid, msg, sizeof(msg), "HTT SRING_SETUP")) { return 1; }
	wl_rx.sring_sent++;
	printf("FUXI WLAN: W26 htt sring setup %s %u info0 %#010x info1 %#010x intr %#010x low %u\n", wl26_ring_name[i],
	    wl_rx.ring[i].id, w[0], w[3], w[11], w[12] & 0xffffu);
	return 0;
}

/* W26m: PDEV_SET_PARAM TX_CHAIN_MASK, then RX_CHAIN_MASK, with the masks of the
 * preferred hw mode's PHYs that VDEV_CREATE's streams come from, for the pdev
 * VDEV_CREATE names (mac.c:8562-8590); no reply awaited (wmi.c:1620-1650).
 * Nothing when a mask is 0 (L19 would have no pdev to send for). */
static int
wl26_chainmask(void)
{
	const uint32_t tx = wl_wmi.phy_tx_mask, rx = wl_wmi.phy_rx_mask;
	const uint32_t t[3] = { wl23_pdev, WLWMI_PDEV_PARAM_TX_CHAIN_MASK, tx }, r[3] = { wl23_pdev, WLWMI_PDEV_PARAM_RX_CHAIN_MASK, rx };
	if (!tx || !rx) {
		printf("FUXI WLAN: W26 chain mask not sent: no MAC/PHY caps (tx %#x rx %#x)\n", tx, rx);
		return 0;
	}
	if (wl23_words(WLWMI_CMD_PDEV_SET_PARAM, WLWMI_TAG_PDEV_SET_PARAM_CMD, t, 3, "WMI PDEV_SET_PARAM TX_CHAIN_MASK") ||
	    wl23_words(WLWMI_CMD_PDEV_SET_PARAM, WLWMI_TAG_PDEV_SET_PARAM_CMD, r, 3, "WMI PDEV_SET_PARAM RX_CHAIN_MASK")) { return 1; }
	wl_rx.chain_tx = tx; wl_rx.chain_rx = rx; wl_rx.chain_on = true;
	printf("FUXI WLAN: W26 chain mask pdev %u tx %#x rx %#x sent (PDEV_SET_PARAM %#x ids %u, %u)\n", wl23_pdev, tx, rx,
	    WLWMI_CMD_PDEV_SET_PARAM, WLWMI_PDEV_PARAM_TX_CHAIN_MASK, WLWMI_PDEV_PARAM_RX_CHAIN_MASK);
	return 0;
}

/* W23 part, right after VERSION_CONF (or where it would be): the four
 * SRING_SETUP back to back and a 100 ms look (non-fatal), the setup watch (for
 * the idle list alone, without the wait), the chain masks. Silent without a
 * W26 bit. */
static int
wl26_rings(void)
{
	unsigned took = 0;
	if (wl_rx.rx_on) {
		for (unsigned i = 0; i < WLRX_NRING; i++) { if (wl26_sring(i)) { return 1; } }
		if ((wl20_bits & WL20_MONSTAT) && (wl28_sring(0) || wl28_sring(1))) { return 1; }  /* C2e: dp_rx.c:4657-4670 */
		if ((wl20_bits & WL20_RXSEL) && (wl27_sel(0) || wl27_sel(1))) { return 1; }  /* C2d: dp_rx.c:4672 */
		if (wl23_poll_for(WL20_W_NONE, 100, &took) == 1) { return 1; }
	}
	wl26_watch(WLRX_P_SETUP);
	if (wl29_macstart_pre()) { return 1; }   /* C2f: mac-start pdev params before the chain mask (mac.c:9372-9415) */
	return (wl20_bits & WL20_CHAINMASK) ? wl26_chainmask() : 0;
}

/* A watch (B4, D2): the four rings' device-side words and ring 120's R2 HP and
 * TP, raw and derived, two lines; host reads only. */
static void
wl26_watch(unsigned p)
{
	struct wlrx_watch *w = &wl_rx.w[p < WLRX_NPOINT ? p : 0];
	const uint8_t f[3] = { WLRX_F_MAC0, WLRX_F_MAC1, WLRX_F_IDLE };
	char lb[28], rf[40], mc[3][40], er[40], fl[40];
	if (p >= WLRX_NPOINT || w->taken || wl_rx.stopping || (!wl_rx.rx_on && !wl_rx.idle_on)) { return; }
	if (wl_rx.rx_on) { wlrx_watch_read(&wl_rx, &wl_ce_io, w); }
	if (wl_rx.idle_on) { w->idle_hp = wl26_wbm_rd(WLRX_WBM_IDLE_HP); w->idle_tp = wl26_wbm_rd(WLRX_WBM_IDLE_TP); }
	w->foreign = wl_scan.foreign;
	wl_rx.refill_head = wl_rd.reo_on ? wl_rd.refill_hp : 0u;   /* C2d re-posts move the head */
	wlrx_watch_derive(&wl_rx, p);
	if (p == WLRX_P_MID) { snprintf(lb, sizeof(lb), "mid (foreign %u)", w->foreign); }
	else {
		snprintf(lb, sizeof(lb), "%s", p == WLRX_P_STARTED ? "started" : p == WLRX_P_COMPLETED ? "completed" :
		    wl_rx.rx_on ? "setup+100ms" : "setup");
	}
	for (unsigned m = 0; m < 3; m++) {
		const uint32_t hp = m < 2 ? w->mac_hp[m] : w->idle_hp, tp = m < 2 ? w->mac_tp[m] : w->idle_tp;
		const uint32_t avail = m < 2 ? w->mac_avail[m] : w->idle_avail;
		if ((w->bad | w->unreadable) & f[m]) {
			snprintf(mc[m], sizeof(mc[m]), "hp %u tp %u%s", hp, tp, (w->unreadable & f[m]) ? " unreadable" : " bad");
		} else { snprintf(mc[m], sizeof(mc[m]), "hp %u tp %u avail %u", hp, tp, avail); }
	}
	if (wl_rx.rx_on) {
		if (w->unreadable & WLRX_F_REFILL) { snprintf(rf, sizeof(rf), "tp unreadable"); }
		else if (w->bad & WLRX_F_REFILL) { snprintf(rf, sizeof(rf), "tp %u bad", w->refill_tp); }
		else { snprintf(rf, sizeof(rf), "tp %u taken %u left %u", w->refill_tp, w->refill_taken, w->refill_left); }
		if (w->unreadable & WLRX_F_ERR) { snprintf(er, sizeof(er), "hp unreadable"); }
		else if (w->bad & WLRX_F_ERR) { snprintf(er, sizeof(er), "hp %u bad", w->err_hp); }
		else { snprintf(er, sizeof(er), "hp %u fill %u", w->err_hp, w->err_fill); }
		snprintf(fl, sizeof(fl), "%s%s%s%s", (w->exhausted & WLRX_F_REFILL) ? " refill" : "", (w->exhausted & WLRX_F_MAC0) ?
		    " mac0" : "", (w->exhausted & WLRX_F_MAC1) ? " mac1" : "", (w->exhausted & WLRX_F_ERR) ? " err" : "");
		printf("FUXI WLAN: W26 rx watch %s refill %s mac0 %s mac1 %s err %s; exhausted%s%s\n", lb, rf, mc[0], mc[1], er,
		    fl[0] ? fl : " none", w->refill_empty ? "; refill-empty" : "");
	}
	if (wl_rx.idle_on) {
		printf("FUXI WLAN: W26 idle watch %s %s; exhausted %s\n", lb, mc[2], (w->exhausted & WLRX_F_IDLE) ? "idle" : "none");
	}
	wl27_watch(lb);  /* C2d; silent without the REO path */
	wl28_watch(p, lb);  /* C2e; silent without the monitor status rings */
}

/* Non-zero counts as " NAME N" (or " 0xT:N" with no names, then " other N"), as
 * many "FUXI WLAN: HEAD" lines as the console's 254 bytes need. */
static void
wl26_list(const char *head, const char *const *name, const uint32_t *v, unsigned n, uint32_t other)
{
	char line[256], item[48];
	size_t len = 0;
	for (unsigned i = 0; i <= n; i++) {
		const uint32_t x = i < n ? v[i] : other;
		int k;
		if (!x) { continue; }
		if (i == n) { k = snprintf(item, sizeof(item), " other %u", x); }
		else if (name) { k = snprintf(item, sizeof(item), " %s %u", name[i], x); }
		else { k = snprintf(item, sizeof(item), " %#x:%u", i, x); }
		if (k <= 0 || (size_t)k >= sizeof(item)) { continue; }
		if (len && len + (size_t)k > 254u) { printf("%s\n", line); len = 0; }
		if (!len) { len = (size_t)snprintf(line, sizeof(line), "FUXI WLAN: %s", head); }
		memcpy(line + len, item, (size_t)k + 1u);
		len += (size_t)k;
	}
	if (len) { printf("%s\n", line); }
}

/* After the C2b lines (also when the stats exchange ended the run, and before
 * the COMPLETED-timeout snapshot): the RX content, guard, error ring, links,
 * counters, the D4 no-TX line and (D5) the no-beacon hint. Host reads only;
 * nothing read from a received frame is printed (counts, and error entries
 * with their frame fields masked). Silent without a W26 bit. */
static void
wl26_report(void)
{
	const struct wlrx_content *c = &wl_rx.c;
	const struct wlrx_watch *e = &wl_rx.w[WLRX_P_COMPLETED];
	uint32_t t2h = wl_rx.htt_t2h_other;
	if (!(wl20_bits & WL26_BITS)) { return; }
	if (wl_rx.rx_on && !wl_rx.stopping) {
		const bool taken = e->taken && !((e->bad | e->unreadable) & WLRX_F_REFILL);
		wlrx_content(&wl_rx, &wl_ce_io, taken ? e->refill_taken : 0u);
		for (unsigned m = 0; m < 2; m++) {
			printf("FUXI WLAN: W26 rx content mac%u entries %u valid %u foreign %u rbm-other %u cookie-other %u\n", m,
			    c->mac_entries[m], c->mac_valid[m], c->mac_foreign[m], c->mac_rbm_other[m], c->mac_cookie_other[m]);
		}
		if (taken) {
			printf("FUXI WLAN: W26 rx content refill taken %u kept %d; buffers written %u of %u\n", c->refill_taken, (int)c->kept,
			    c->bufs_written, WLRX_NBUF);
		} else {
			printf("FUXI WLAN: W26 rx content refill taken unknown (no usable completed watch); buffers written %u of %u\n",
			    c->bufs_written, WLRX_NBUF);
		}
		printf("FUXI WLAN: W26 rx guard %snonzero dwords: slot-upper %u (slots %u), slot 4095 %u, tail %u, gaps %u%s\n",
		    c->guard_upper || c->guard_last || c->guard_tail || c->guard_gap ? "SPILL " : "", c->guard_upper, c->guard_upper_slots,
		    c->guard_last, c->guard_tail, c->guard_gap,
		    c->guard_upper || c->guard_last || c->guard_tail || c->guard_gap ? "; stop; owner review before any further kit" : "");
		printf("FUXI WLAN: W26 rxdma err entries %u (hp %u%s) push err %u routing %u flush %u other %u; rbm link %u buf %u "
		    "other %u\n", c->err_entries, c->err_hp, c->err_bad ? " bad" : "", c->err_push[0], c->err_push[1], c->err_push[2],
		    c->err_push[3], c->err_rbm_link, c->err_rbm_buf, c->err_rbm_other);
		wl26_list("W26 rxdma err codes", wl26_err_code, c->err_code, 19, c->err_code[19]);
		for (unsigned i = 0; i < c->err_entries && i < 4u; i++) {
			printf("FUXI WLAN: W26 rxdma err entry %u w1 %#x w5 %#x w6 %#x w7 %#x\n", i, c->err_raw[i][0], c->err_raw[i][1],
			    c->err_raw[i][2], c->err_raw[i][3]);
		}
	}
	if (wl_rx.idle_on && !wl_rx.stopping) {
		wlrx_link_content(&wl_rx, &wl_ce_io);
		printf("FUXI WLAN: W26 link content descriptors written %u of %u, unlisted slot %s; idle ring entries changed %u\n",
		    c->link_written, WLRX_NDESC, c->link_unlisted ? "written" : "clean", c->idle_changed);
	}
	for (unsigned i = 0; i < 64u; i++) { t2h += wl_rx.htt_t2h[i]; }
	printf("FUXI WLAN: W26 counters htt-t2h %u htt-rx %u wmi-events %u unknown %u bad-reports %u sring-sent %u\n", t2h,
	    wl_htc.htt_rx, wl_wmi.events, wl_wmi.unknown_events, wl_htc.bad_reports, wl_rx.sring_sent);
	if (t2h) { wl26_list("W26 htt-t2h types", NULL, wl_rx.htt_t2h, 64u, wl_rx.htt_t2h_other); }
	else { printf("FUXI WLAN: W26 htt-t2h types none\n"); }
	if (wl_rx.tx_ci || wl_rx.tx_stats[0] || wl_rx.tx_stats[1] || wl_rx.tx_stats[2] || wl_rx.tx_stats[3] || wl_rx.tx_stats[4]) {
		printf("FUXI WLAN: W26 TX COUNTERS NON-ZERO: owner review (chan-info tx-frame events %u; stats answers %u with "
		    "local-enqued %u hw-queued %u mpdu-enqued %u hw-reaped %u base tx-frame %u)\n", wl_rx.tx_ci, wl_rx.stats_answers,
		    wl_rx.tx_stats[0], wl_rx.tx_stats[1], wl_rx.tx_stats[2], wl_rx.tx_stats[3], wl_rx.tx_stats[4]);
	}
	wl27_report();  /* C2d; silent without a W27 bit */
	wl28_report();  /* C2e; silent without 0x2000 */
	wl29_report();  /* C2f; silent without 0x4000 */
	if (wl26_heard_none) {
		printf("FUXI WLAN: W24 no beacon or probe response heard (chan-info %u); next rung %s\n", wl_scan.ci_start,
		    (wl20_bits & WL20_MONSTAT) ? wlms_next(&wl_ms, wl20_bits, (wl20_bits & WL27_BITS) ? wlrd_next(&wl_rd, wl20_bits,
		    wl_rx.c.bufs_written) : wlrx_next(&wl_rx, wl20_bits)) :
		    (wl20_bits & WL27_BITS) ? wlrd_next(&wl_rd, wl20_bits, wl_rx.c.bufs_written) : wlrx_next(&wl_rx, wl20_bits));
	}
}

/* First containment step (wl_ce_contain, before STOP_SCAN and the CE latch):
 * no host write to either arena and no WBM access from here on; reads for the
 * final lines stay allowed. Silent without a W26 arena. */
static void
wl26_contain(void)
{
	wl_rd.stopping = true;  /* C2d: no reap, return, replenish or REO access from here on */
	wl_ms.stopping = true;  /* C2e: no host write to the monitor status arena from here on */
	wl_cm.stopping = true;  /* C2f: no common-ring, cc_config or CMEM access from here on */
	if (!wl_rx_arena.va && !wl_link_arena.va && !wl_cm_arena.va) { return; }
	wl_rx.stopping = true;
	printf("FUXI WLAN: W26 rx contain: host writes off; arenas held until bus master off, PERST and SMMU undo\n");
}

/* wl5_abort's proven branch only (bus master off and PERST confirmed, the M6o
 * undo clean, the SMMU undo done): D3, the guards counted again (host reads),
 * then both arenas zeroed through their NC mappings and freed (wl_dma_free only
 * restores attributes and frees: neighbours' frame bytes must not reach the
 * kernel allocator). Silent without a W26 arena. */
static void
wl26_free(void)
{
	uint32_t up = 0, slots = 0, last = 0, tail = 0, gap = 0, bytes = 0;
	if (!wl_rx_arena.va && !wl_link_arena.va && !wl_cm_arena.va) { return; }
	if (wl_rx_arena.va && wl_rx.base) {
		wlrx_guard_count(&wl_rx, &up, &slots, &last, &tail, &gap);
		printf("FUXI WLAN: W26 rx guard after containment nonzero dwords: slot-upper %u (slots %u), slot 4095 %u, tail %u, "
		    "gaps %u%s\n", up, slots, last, tail, gap, up || last || tail || gap ? "; SPILL: owner review before any further kit" : "");
	}
	if (wl_reo_arena.va && wl_rd.base) {  /* C2d */
		wlrd_guard_count(&wl_rd, &gap, &tail);
		printf("FUXI WLAN: W27 reo guard after containment nonzero dwords: gaps %u, tail %u%s\n", gap, tail,
		    gap || tail ? "; SPILL: owner review before any further kit" : "");
	}
	if (wl_mon_arena.va && wl_ms.base) {  /* C2e */
		wlms_guard_count(&wl_ms, &up, &slots, &last, &tail, &gap);
		printf("FUXI WLAN: W28 mon guard after containment nonzero dwords: slot-upper %u (slots %u), unposted slots %u, tail %u, "
		    "gaps %u%s\n", up, slots, last, tail, gap, up || last || tail || gap ? "; SPILL: owner review before any further kit" : "");
	}
	if (wl_cm_arena.va && wl_cm.base) {  /* C2f */
		uint32_t rdp_gap = 0, ring_gap = 0, spt_lost = 0;
		wlcm_guard_count(&wl_cm, &rdp_gap, &ring_gap, &spt_lost, &tail);
		printf("FUXI WLAN: W29 guard after containment nonzero dwords: rdp %u, ring-gap %u, spt-lost %u, tail %u%s\n", rdp_gap,
		    ring_gap, spt_lost, tail, rdp_gap || ring_gap || spt_lost || tail ? "; SPILL: owner review before any further kit" : "");
	}
	if (wl_rx_arena.va) { bzero((void *)wl_rx_arena.va, wl_rx_arena.size); bytes += (uint32_t)wl_rx_arena.size; }
	if (wl_link_arena.va) { bzero((void *)wl_link_arena.va, wl_link_arena.size); bytes += (uint32_t)wl_link_arena.size; }
	if (wl_reo_arena.va) { bzero((void *)wl_reo_arena.va, wl_reo_arena.size); bytes += (uint32_t)wl_reo_arena.size; }
	if (wl_mon_arena.va) { bzero((void *)wl_mon_arena.va, wl_mon_arena.size); bytes += (uint32_t)wl_mon_arena.size; }
	if (wl_cm_arena.va) { bzero((void *)wl_cm_arena.va, wl_cm_arena.size); bytes += (uint32_t)wl_cm_arena.size; }
	wl_ce_wbarrier(NULL);
	printf("FUXI WLAN: W26 arenas zeroed %#x bytes before free\n", bytes);
	wl_dma_free(&wl_rx_arena);
	wl_dma_free(&wl_link_arena);
	wl_dma_free(&wl_reo_arena);
	wl_dma_free(&wl_mon_arena);
	wl_dma_free(&wl_cm_arena);
	bzero(&wl_rx, sizeof(wl_rx));
	bzero(&wl_rd, sizeof(wl_rd));
	bzero(&wl_ms, sizeof(wl_ms));
	bzero(&wl_cm, sizeof(wl_cm));
}

/* ---- C2d (fuxiwlanqrtr=7): W27, RX ring selection and the REO/WBM buffer return path ----
 * w20 0x800 RXSEL and 0x1000 REORX, default off, each needing 0x100
 * (docs-lanes/wlan-w20.md, C2d; the pure encoders, the ring table, the entry
 * checks and the replenish are in fuxi_wlan_ce.h). In L19's order: at W20, inside
 * wl26_setup, the REO arena with the other two, then after the idle list and
 * before the refill fill dp_alloc's common setup (dp.c:491-595): 38 read-only
 * pre-reads (refusal: no REO or release-ring write at all, 0x1000 dropped), the
 * WBM2SW3 release ring, the REO2SW0 exception ring, the REO setup; at W21 after
 * service ready the eight REO destination rings (core.c:925), each re-read first;
 * at W23 the two RX_RING_SELECTION_CFG right after the four SRING_SETUP
 * (dp_rx.c:4670). Every poll pass then reaps the ten rings (rdp page of the REO
 * arena), hands every returned buffer back to refill ring 160 with a fresh
 * sentinel and publishes each ring's TP. Register accesses: only wlrd_reg_ok's
 * through wl_win, and the TP publishes to the shadow registers WLAN_CFG gave the
 * ten rings (or R2 TP with fuxiwlanw15=1); nothing after wl26_contain. No MSI,
 * interrupt, GIC, power or PMIC access, nothing transmitted. The REO arena is
 * held, counted, zeroed and freed with the W26 arenas (wl26_free).
 * Adapted from Linux ath12k dp.c, dp_rx.c, dp_tx.c, hal.c and hal_rx.c
 * (BSD-3-Clause-Clear), v6.19; notice in bsd/dev/fuxi_wlan_ce.h. */
static const char *const wl27_ring_name[WLRD_NRING] = { "rel", "exc", "dst1", "dst2", "dst3", "dst4", "dst5", "dst6", "dst7",
	"dst8" };

/* The REO accessor: wlrd_reg_ok's allowlist and the wl_win discipline; refused
 * after wl26_contain. */
static int
wl27_reo(uint32_t off, bool wr, uint32_t *value)
{
	if (fw_faults || wl_rd.stopping || !wlrd_reg_ok(off, wr)) { return 1; }
	return wl_win(off, wr, value);
}

static uint32_t
wl27_reo_rd(uint32_t off)
{
	uint32_t v = ~0u;
	return wl27_reo(off, false, &v) ? ~0u : v;
}

/* The shadow register WLAN_CFG gave ring R's R2 TP (wl15_shadow_v3 indices 0..8
 * and 24), WLCE_SHADOW_REGS when none. */
static uint32_t
wl27_shadow(const struct wlrd_ring *r)
{
	uint32_t idx = WLCE_SHADOW_REGS;
	for (unsigned j = 0; j < sizeof(wl15_shadow_v3) / sizeof(wl15_shadow_v3[0]); j++) {
		if (wl15_shadow_v3[j] == r->r2 + 4u) { idx = j; }
	}
	return idx;
}

/* A destination ring's TP after its entries were taken: write barrier, then the
 * shadow register (or R2 TP through the window with fuxiwlanw15=1). */
static int
wl27_tp(const struct wlrd_ring *r)
{
	uint32_t tp = r->tp, idx = wl27_shadow(r);
	if (wl_rd.stopping) { return 1; }
	wl_ce_wbarrier(NULL);
	if (wl15_variant) { return wl27_reo(r->r2 + 4u, true, &tp); }
	return idx >= WLCE_SHADOW_REGS ? 1 : wl_ce_io_bar_wr(NULL, WLCE_SHADOW_BASE + 4u * idx, tp);
}

/* Ring I: its nine writes in L19's order, BASE_LSB and MISC read back. */
static int
wl27_ring(unsigned i)
{
	uint32_t regs[WLRD_RING_REGS][2], base, misc, idx;
	struct wlrd_ring *r = &wl_rd.ring[i];
	char via[32];
	wlrd_ring_regs(&wl_rd, i, regs);
	for (unsigned k = 0; k < WLRD_RING_REGS; k++) {
		if (wl27_reo(regs[k][0], true, &regs[k][1])) {
			printf("FUXI WLAN: W27 reo ring %u write %u of %u (%#x) failed; contain\n", r->id, k + 1u, WLRD_RING_REGS, regs[k][0]);
			return wl_w11_fail("W27 reo register write");
		}
		wl27_reo_writes++;
	}
	base = wl27_reo_rd(regs[0][0]);
	misc = wl27_reo_rd(regs[WLRD_RING_REGS - 1u][0]);
	if (base != regs[0][1] || misc == ~0u || !(misc & WLCE_MISC_SRNG_ENABLE)) {
		printf("FUXI WLAN: W27 reo ring %u readback mismatch base %#x misc %#x\n", r->id, base, misc);
		return wl_w11_fail("W27 reo ring readback mismatch");
	}
	r->on = true;
	idx = wl27_shadow(r);
	if (wl15_variant) { snprintf(via, sizeof(via), "R2 %#x (fuxiwlanw15=1)", r->r2 + 4u); }
	else { snprintf(via, sizeof(via), "shadow %u (%#x)", idx, WLCE_SHADOW_BASE + 4u * idx); }
	printf("FUXI WLAN: W27 reo ring %u %s entries %u base %#x misc %#x readback ok; tp via %s\n", r->id, wl27_ring_name[i],
	    r->entries, base, misc, via);
	return 0;
}

/* W20 part (dp_alloc's common setup, after the idle list and before the RX
 * alloc): the pre-reads, then (unless they refuse) the release ring, the
 * exception ring and ath12k_hal_reo_hw_setup (GEN_ENABLE and MISC_CTRL read
 * right before their writes, as L19). Silent without 0x1000. */
static int
wl27_setup(void)
{
	const uint32_t *p = wl_rd.pre;
	uint32_t en = 0, based = 0, unread = 0, hw[WLRD_HW_REGS][2], gen, misc;
	if (!(wl20_bits & WL20_REORX)) { return 0; }
	for (unsigned i = 0; i < WLRD_PRE_N; i++) { wl_rd.pre[i] = wl27_reo_rd(wlrd_pre_addr(i)); }
	wl_rd.own = wlrd_owned(wl_rd.pre);
	for (unsigned k = 0; k < WLRD_NRING; k++) {
		en += p[3u * k + 2u] != ~0u && (p[3u * k + 2u] & WLCE_MISC_SRNG_ENABLE);
		based += p[3u * k] != ~0u && (p[3u * k] || (p[3u * k + 1u] & 0xffu));
	}
	for (unsigned i = 0; i < WLRD_PRE_N; i++) { unread += p[i] == ~0u; }
	printf("FUXI WLAN: W27 reo before: rings %u enabled %u based %u unreadable %u gen-enable %#x misc-ctrl %#x aging %#x/%#x/%#x/%#x "
	    "dest-ctrl %#x/%#x; %s\n", WLRD_NRING, en, based, unread, p[3u * WLRD_NRING], p[3u * WLRD_NRING + 1u],
	    p[3u * WLRD_NRING + 2u], p[3u * WLRD_NRING + 3u], p[3u * WLRD_NRING + 4u], p[3u * WLRD_NRING + 5u], p[3u * WLRD_NRING + 6u],
	    p[3u * WLRD_NRING + 7u], wl_rd.own == WLRD_FREE ? "not owned" : "refused");
	if (wl_rd.own != WLRD_FREE) {
		printf("FUXI WLAN: W27 reo not programmed: %s; w20 0x1000 dropped for this boot\n", wlrd_why(wl_rd.own));
		return 0;
	}
	if (wl29_common_pre()) { return 1; }   /* C2f: SW2WBM release, TCL+comp x3, reinject (L19 common setup, before rx_rel) */
	if (wl27_ring(WLRD_R_REL) || wl27_ring(WLRD_R_EXC)) { return 1; }
	if (wl29_common_post()) { return 1; }  /* C2f: REO command (+init) and status rings, after the exception ring and before reo_hw_setup */
	gen = wl27_reo_rd(WLRD_REO_GEN_ENABLE);
	wlrd_hw_regs(gen, 0u, hw);
	if (gen == ~0u || wl27_reo(hw[0][0], true, &hw[0][1])) { return wl_w11_fail("W27 reo GEN_ENABLE read-modify-write"); }
	wl27_reo_writes++;
	misc = wl27_reo_rd(WLRD_REO_MISC_CTRL);
	wlrd_hw_regs(gen, misc, hw);
	if (misc == ~0u) { return wl_w11_fail("W27 reo MISC_CTRL read"); }
	for (unsigned k = 1; k < WLRD_HW_REGS; k++) {
		if (wl27_reo(hw[k][0], true, &hw[k][1])) {
			printf("FUXI WLAN: W27 reo setup write %u of %u (%#x) failed; contain\n", k + 1u, WLRD_HW_REGS, hw[k][0]);
			return wl_w11_fail("W27 reo register write");
		}
		wl27_reo_writes++;
	}
	wl_rd.reo_on = true;
	printf("FUXI WLAN: W27 reo setup gen-enable %#x -> %#x misc-ctrl %#x -> %#x aging %u/%u/%u/%u hash %#x; writes %u window restore "
	    "checked\n", gen, hw[0][1], misc, hw[1][1], hw[2][1], hw[3][1], hw[4][1], hw[5][1], hw[6][1], wl27_reo_writes);
	return 0;
}

/* W21 part, after service ready and before INIT (core.c:925): each destination
 * ring read again (a ring that turned enabled or based since W20 stops the run:
 * the release and exception rings are live by then), then its nine writes. */
static int
wl27_dest(void)
{
	if (!wl_rd.reo_on || wl_rd.dst_on) { return 0; }
	for (unsigned n = 0; n < WLRD_NDST; n++) {
		const unsigned i = WLRD_R_DST + n;
		const uint32_t b = wl27_reo_rd(wlrd_pre_addr(3u * i)), m = wl27_reo_rd(wlrd_pre_addr(3u * i + 1u)),
		    c = wl27_reo_rd(wlrd_pre_addr(3u * i + 2u));
		if (b == ~0u || m == ~0u || c == ~0u || b || (m & 0xffu) || (c & WLCE_MISC_SRNG_ENABLE)) {
			printf("FUXI WLAN: W27 reo %s not free at W21 (base %#x/%#x misc %#x); contain\n", wl27_ring_name[i], b, m, c);
			return wl_w11_fail("W27 reo destination ring not free at W21");
		}
		if (wl27_ring(i)) { return 1; }
	}
	wl_rd.dst_on = true;
	return wl_w11_guard();
}

/* One W27 message through wl_ce_send on the HTT endpoint (CE4, one in flight,
 * TX drained), no reply asked (L19 asks none). */
static int
wl27_sel(unsigned mac)
{
	uint32_t w[WLRD_SEL_WORDS];
	uint8_t msg[4u * WLRD_SEL_WORDS];
	const uint32_t eid = wl_htc.htt_eid;
	const struct wlce_pipe *p = &wl_ce.pipe[wl_htc.ep[eid].ul_pipe];
	const char *why = !eid || !wl_htc.ep[eid].used ? "no HTT endpoint" : wl_rx.stopping ? "stopping" :
	    p->posted - p->reaped >= p->nslots ? "CE4 slots all in flight" : NULL;
	if (why) {
		printf("FUXI WLAN: W27 htt rx ring selection mac%u not sent: %s\n", mac, why);
		return wl_w11_fail("W27 htt rx ring selection not sent");
	}
	wlrd_sel_words(mac, w);
	for (unsigned k = 0; k < WLRD_SEL_WORDS; k++) { wlce_put32(msg + 4u * k, w[k]); }
	if (wl_ce_send(eid, msg, sizeof(msg), "HTT RX_RING_SELECTION_CFG")) { return 1; }
	wl_rd.sel_sent++;
	wl_rd.sel_on = wl_rd.sel_sent == 2u;
	printf("FUXI WLAN: W27 htt rx ring selection mac%u %u info0 %#010x buf %u flags %#x/%#x/%#x/%#x tlv %#x offsets %#010x/%#010x/%#x "
	    "sent\n", mac, wl_rx.ring[WLRX_R_MAC0 + mac].id, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8], w[9]);
	return 0;
}

/* One poll pass (wl_ce_poll, REORX only): the ten rings in setup order, each
 * TP published after its entries, then the refill replenish. A refused entry
 * or a HP outside its ring stops the run (contain). */
static int
wl27_poll(void)
{
	uint32_t moved = 0;
	if (wl_rd.stopping) { return 0; }
	for (unsigned i = 0; i < WLRD_NRING; i++) {
		if (!wl_rd.ring[i].on) { continue; }
		if (wlrd_reap(&wl_rd, &wl_rx, &wl_ce_io, i, &moved)) {
			printf("FUXI WLAN: W27 reo %s entry refused: %s; contain\n", wl27_ring_name[i], wl_rd.why ? wl_rd.why : "-");
			return wl_w11_fail(wl_rd.why ? wl_rd.why : "W27 reo entry refused");
		}
		if (moved && wl27_tp(&wl_rd.ring[i])) { return wl_w11_fail("W27 reo TP publish"); }
	}
	if (wl_rx.rx_on) { (void)wlrd_replenish(&wl_rd, &wl_rx, &wl_ce_io); }
	return 0;
}

/* At each W26 watch, after its lines (host counts only). */
static void
wl27_watch(const char *point)
{
	uint32_t dst = 0;
	if (!wl_rd.reo_on) { return; }
	for (unsigned n = 0; n < WLRD_NDST; n++) { dst += wl_rd.ring[WLRD_R_DST + n].reaped; }
	printf("FUXI WLAN: W27 reo watch %s reaped rel %u exc %u dst %u; returned %u reposted %u pending %u (max %u) refill hp %u\n",
	    point, wl_rd.ring[WLRD_R_REL].reaped, wl_rd.ring[WLRD_R_EXC].reaped, dst, wl_rd.returned, wl_rd.reposted, wl_rd.pend_n,
	    wl_rd.pend_max, wl_rd.refill_hp);
}

/* In wl26_report, before the D5 hint: per-ring reap counts, the entry kinds, the
 * error codes, the release sources, the buffers returned and the REO arena's
 * guard. Counts only. Silent without a W27 bit. */
static void
wl27_report(void)
{
	const struct wlrd *d = &wl_rd;
	uint32_t gap = 0, tail = 0;
	if (!(wl20_bits & WL27_BITS)) { return; }
	if (wl20_bits & WL20_RXSEL) { printf("FUXI WLAN: W27 rx ring selection sent %u of 2\n", d->sel_sent); }
	if (!d->reo_on) { return; }
	printf("FUXI WLAN: W27 reo reaped rel %u exc %u dst %u %u %u %u %u %u %u %u\n", d->ring[0].reaped, d->ring[1].reaped,
	    d->ring[2].reaped, d->ring[3].reaped, d->ring[4].reaped, d->ring[5].reaped, d->ring[6].reaped, d->ring[7].reaped,
	    d->ring[8].reaped, d->ring[9].reaped);
	printf("FUXI WLAN: W27 reo entries msdu %u link %u (held %u, msdus %u, skipped %u, not ours %u) push routing %u error %u "
	    "continuation %u\n", d->msdu, d->link, d->links_held, d->link_msdus, d->link_skipped, d->link_unowned, d->routing,
	    d->reo_error, d->cont);
	wl26_list("W27 reo error codes", NULL, d->reo_code, 32u, 0u);
	printf("FUXI WLAN: W27 release src rxdma %u reo %u other %u desc-other %u rbm-other %u cookie-other %u\n", d->rel_src[0],
	    d->rel_src[1], d->rel_src[2], d->desc_other, d->rbm_other, d->cookie_other);
	wl26_list("W27 release rxdma codes", NULL, d->rxdma_code, 32u, 0u);
	printf("FUXI WLAN: W27 buffers returned %u reposted %u pending %u (max %u) publishes %u refill hp %u refill-bad %u\n",
	    d->returned, d->reposted, d->pend_n, d->pend_max, d->publishes, d->refill_hp, d->refill_bad);
	if (!d->stopping) {
		wlrd_guard_count(d, &gap, &tail);
		printf("FUXI WLAN: W27 reo guard %snonzero dwords: gaps %u, tail %u%s\n", gap || tail ? "SPILL " : "", gap, tail,
		    gap || tail ? "; stop; owner review before any further kit" : "");
	}
}

/* ---- C2e (fuxiwlanqrtr=7): W28, the RXDMA monitor status rings and the crash line ----
 * w20 0x2000 MONSTAT, default off, needing 0x100 (docs-lanes/wlan-w20.md, C2e;
 * the layout, the fill, the encoder, the watch and the counts are in
 * fuxi_wlan_ce.h). In L19's order: at W20, inside wl26_setup, the fourth arena
 * with the other three (before any WBM or REO write), then after the refill
 * ring the two rings' 1023 entries each and their HP publishes (wrp[25],
 * wrp[33]; ath12k_dp_rxdma_buf_setup, dp_rx.c:483-517); at W23 their two
 * SRING_SETUP after the four C2c ones and before the ring selection
 * (dp_rx.c:4657-4672). Never reaped (L19 STA mode reaps none): at every W26
 * watch the device TPs, buffers written, fill and exhaustion; after the scan
 * the content and guard counts. The crash line: the six MHI/BHI reads of the
 * W11/W15 diagnostic through wlms_diag_ok's allowlist, at the COMPLETED timeout
 * and in containment before the MHI reset. No register write, no MSI,
 * interrupt, GIC, power or PMIC access, nothing transmitted. The arena is held,
 * counted, zeroed and freed with the W26 arenas (wl26_free), else quarantined.
 * Adapted from Linux ath12k dp.c, dp_mon.c, dp_rx.c, dp_tx.c and hal.c
 * (BSD-3-Clause-Clear), v6.19; notice in bsd/dev/fuxi_wlan_ce.h. */

/* One crash-line read: wlms_diag_ok's six addresses only, ~0u otherwise. */
static uint32_t
wl28_rd(uint32_t off)
{
	if (!wlms_diag_ok(off, wl_bhi)) { return ~0u; }
	return wl_mrd(off);
}

/* The read-only crash line (no write, no retry; silent without 0x2000). */
static void
wl28_crash(const char *where)
{
	uint32_t v[WLMS_DIAG_N];
	const unsigned before = fw_faults;
	if (!(wl20_bits & WL20_MONSTAT)) { return; }
	if (!wl_mhi_valid || !wlms_diag_ok(WLMS_MHISTATUS, wl_bhi)) {
		printf("FUXI WLAN: W28 crash %s: not read (MHI/BHI offsets not established)\n", where);
		return;
	}
	for (unsigned i = 0; i < WLMS_DIAG_N; i++) { v[i] = wl28_rd(wlms_diag_addr(i, wl_bhi)); }
	printf("FUXI WLAN: W28 crash %s: mhistatus %#x (state %#x syserr %u) ee %#x bhi errcode %#x errdbg %#x/%#x/%#x; "
	    "access-faults %u; read-only\n", where, v[0], (v[0] >> 8) & 0xffu, (v[0] >> 2) & 1u, v[1], v[2], v[3], v[4], v[5],
	    fw_faults - before);
}

/* One monitor status SRING_SETUP through wl_ce_send on the HTT endpoint (CE4,
 * one in flight, TX drained), no reply asked, as wl26_sring. */
static int
wl28_sring(unsigned m)
{
	uint32_t w[WLRX_SRING_WORDS];
	uint8_t msg[4u * WLRX_SRING_WORDS];
	const uint32_t eid = wl_htc.htt_eid;
	const struct wlce_pipe *p = &wl_ce.pipe[wl_htc.ep[eid].ul_pipe];
	const char *why = !eid || !wl_htc.ep[eid].used ? "no HTT endpoint" : wl_ms.stopping ? "stopping" :
	    !wl_ms.hp[m & 1u] ? "ring not filled" : p->posted - p->reaped >= p->nslots ? "CE4 slots all in flight" : NULL;
	if (why) {
		printf("FUXI WLAN: W28 htt sring setup mon%u not sent: %s\n", m, why);
		return wl_w11_fail("W28 htt sring setup not sent");
	}
	wlms_sring_words(&wl_ms, m, w);
	for (unsigned k = 0; k < WLRX_SRING_WORDS; k++) { wlce_put32(msg + 4u * k, w[k]); }
	if (wl_ce_send(eid, msg, sizeof(msg), "HTT SRING_SETUP")) { return 1; }
	wl_ms.sring_sent++;
	wl_ms.on = wl_ms.sring_sent == 2u;
	printf("FUXI WLAN: W28 htt sring setup mon%u %u info0 %#010x info1 %#010x intr %#010x low %u\n", m, wl_ms.ring[m & 1u].id,
	    w[0], w[3], w[11], w[12] & 0xffffu);
	return 0;
}

/* At each W26 watch, after its lines (host reads only). */
static void
wl28_watch(unsigned p, const char *point)
{
	const struct wlms_watch *w = &wl_ms.w[p < WLRX_NPOINT ? p : 0];
	char mc[2][64];
	if (!wl_ms.on || wl_ms.stopping || p >= WLRX_NPOINT) { return; }
	wlms_watch(&wl_ms, &wl_rx, &wl_ce_io, p);
	for (unsigned m = 0; m < 2; m++) {
		const uint8_t f = m ? WLMS_F_MON1 : WLMS_F_MON0;
		if ((w->bad | w->unreadable) & f) {
			snprintf(mc[m], sizeof(mc[m]), "hp %u tp %u %s written %u", wl_ms.hp[m], w->tp[m], (w->unreadable & f) ? "unreadable" : "bad",
			    w->written[m]);
		} else {
			snprintf(mc[m], sizeof(mc[m]), "hp %u tp %u taken %u left %u written %u", wl_ms.hp[m], w->tp[m], w->taken[m], w->left[m],
			    w->written[m]);
		}
	}
	printf("FUXI WLAN: W28 mon watch %s mon0 %s mon1 %s; exhausted %s\n", point, mc[0], mc[1],
	    w->exhausted == (WLMS_F_MON0 | WLMS_F_MON1) ? "mon0 mon1" : w->exhausted & WLMS_F_MON0 ? "mon0" :
	    w->exhausted & WLMS_F_MON1 ? "mon1" : "none");
}

/* In wl26_report, before the D5 hint: the content and guard counts (host
 * reads; nothing from a buffer). Silent without 0x2000. */
static void
wl28_report(void)
{
	if (!(wl20_bits & WL20_MONSTAT)) { return; }
	printf("FUXI WLAN: W28 mon sring sent %u of 2\n", wl_ms.sring_sent);
	if (!wl_ms.on || wl_ms.stopping) { return; }
	wlms_content(&wl_ms, &wl_ce_io);
	for (unsigned m = 0; m < 2; m++) {
		printf("FUXI WLAN: W28 mon content mon%u written %u of %u; ring entries changed %u (foreign %u)\n", m, wl_ms.written[m],
		    WLMS_NBUF, wl_ms.changed[m], wl_ms.foreign[m]);
	}
	printf("FUXI WLAN: W28 mon guard %snonzero dwords: slot-upper %u (slots %u), unposted slots %u, tail %u, gaps %u%s\n",
	    wl_ms.guard_upper || wl_ms.guard_last || wl_ms.guard_tail || wl_ms.guard_gap ? "SPILL " : "", wl_ms.guard_upper,
	    wl_ms.guard_upper_slots, wl_ms.guard_last, wl_ms.guard_tail, wl_ms.guard_gap,
	    wl_ms.guard_upper || wl_ms.guard_last || wl_ms.guard_tail || wl_ms.guard_gap ? "; stop; owner review before any further kit" : "");
}

/* ---- C2f (fuxiwlanqrtr=7): W29, L19's dp_alloc order, cookie conversion and
 * the common SRNGs ---- w20 0x4000 L19ORDER, default off, needing 0x1000 (so
 * 0x100 and 0x02) (docs-lanes/wlan-w20.md, C2f; the layout, encoders, cc cookie
 * math, allowlists and guards are in fuxi_wlan_ce.h). In L19's order: at dp_alloc
 * (wl26_setup), after the idle list and before the common SRNGs, ath12k_dp_cc_init
 * (wl29_cc_init: the CMEM PPT table and the SPT pages, gated on the QMI CMEM
 * region, dropped for the boot without it); inside wl27_setup the common rings of
 * ath12k_dp_srng_common_setup around C2d's rel/exc (wl29_common_pre: SW2WBM
 * release, three TCL data rings each with its WBM completion ring, the REO
 * reinject ring; wl29_common_post: the REO command ring with its command-number
 * init and the REO status ring); at service ready before C2d's destination rings,
 * ath12k_dp_cc_config (wl29_cc_config: the REO and WBM cookie registers). The
 * common rings are programmed to L19's exact registers and backed to full size
 * but never posted and never reaped (a passive scan drives no TX and no REO
 * command). Cookie conversion makes the RX buffer cookies and the WBM2SW RX
 * release entries the converted shape; every returned cookie maps back through
 * the slot table (wlcm_cookie_slot) or is refused. Register accesses go through
 * wl29_reg (wlcm_reg_ok) and the CMEM writes through wl29_cmem (wlcm_cmem_ok),
 * both via wl_win and both refused after wl26_contain. No MSI, interrupt, GIC,
 * power or PMIC access; nothing transmitted. The arena is held, counted, zeroed
 * and freed with the W26 arenas (wl26_free).
 * Adapted from Linux ath12k dp.c, dp.h, hal.c, hal.h, hal_desc.h, hal_rx.c, hw.c,
 * mac.c and wmi.c (BSD-3-Clause-Clear), v6.19; notice in bsd/dev/fuxi_wlan_ce.h. */
static const char *const wl29_ring_name[WLCM_NRING] = { "sw2wbm-rel", "tcl0", "comp0", "tcl1", "comp1", "tcl2", "comp2",
	"reinject", "reo-cmd", "reo-status" };

/* The W29 register accessor (common-ring and cc_config registers): wlcm_reg_ok's
 * allowlist and the wl_win discipline; refused after wl26_contain. */
static int
wl29_reg(uint32_t off, bool wr, uint32_t *value)
{
	if (fw_faults || wl_cm.stopping || !wlcm_reg_ok(&wl_cm, off, wr)) { return 1; }
	return wl_win(off, wr, value);
}

static uint32_t
wl29_reg_rd(uint32_t off)
{
	uint32_t v = ~0u;
	return wl29_reg(off, false, &v) ? ~0u : v;
}

/* The W29 CMEM accessor (the 280 PPT entries only): wlcm_cmem_ok's allowlist and
 * the wl_win discipline; refused after wl26_contain. */
static int
wl29_cmem(uint32_t off, bool wr, uint32_t *value)
{
	if (fw_faults || wl_cm.stopping || !wlcm_cmem_ok(&wl_cm, off)) { return 1; }
	return wl_win(off, wr, value);
}

static uint32_t
wl29_cmem_rd(uint32_t off)
{
	uint32_t v = ~0u;
	return wl29_cmem(off, false, &v) ? ~0u : v;
}

/* One common ring: its SRC or DST writes in L19's order, BASE_LSB and MISC read
 * back (a mismatch stops the run). Never posted, never reaped afterward. */
static int
wl29_ring(unsigned i)
{
	uint32_t regs[WLCM_RING_REGS_MAX][2], base, misc;
	unsigned n = 0;
	struct wlcm_ring *r = &wl_cm.ring[i];
	wlcm_ring_regs(&wl_cm, i, regs, &n);
	for (unsigned k = 0; k < n; k++) {
		if (wl29_reg(regs[k][0], true, &regs[k][1])) {
			printf("FUXI WLAN: W29 common ring %u write %u of %u (%#x) failed; contain\n", r->id, k + 1u, n, regs[k][0]);
			return wl_w11_fail("W29 common ring register write");
		}
		wl29_writes++;
	}
	base = wl29_reg_rd(r->r0 + WLCE_R0_BASE_LSB);
	misc = wl29_reg_rd(r->r0 + WLCE_R0_MISC);
	if (base != (uint32_t)r->iova || misc == ~0u || !(misc & WLCE_MISC_SRNG_ENABLE)) {
		printf("FUXI WLAN: W29 common ring %u readback mismatch base %#x misc %#x\n", r->id, base, misc);
		return wl_w11_fail("W29 common ring readback mismatch");
	}
	r->on = true;
	printf("FUXI WLAN: W29 common ring %u %s entries %u base %#x misc %#x readback ok\n", r->id, wl29_ring_name[i], r->entries,
	    base, misc);
	return 0;
}

/* dp_alloc's cookie-converter init (ath12k_dp_cc_init, dp.c:1583-1667): the PPT
 * table in CMEM and the SPT pages. Gated on the QMI CMEM region; dropped for the
 * boot (cc_config and the converted-entry decode go with it) when it is absent or
 * already owned. On success the RX buffer cookies and the returned entries become
 * cookie-converted. */
static int
wl29_cc_init(void)
{
	unsigned own;
	if (!(wl20_bits & WL20_L19ORDER)) { return 0; }
	wl_cm.cmem_base = wl_cmem_base; wl_cm.cmem_valid = wl_cmem_valid;
	if (!wl_cmem_valid) {
		printf("FUXI WLAN: W29 cookie conversion not programmed: %s; w20 0x4000 cc dropped for this boot\n", wlcm_why(WLCM_NOCMEM));
		return 0;
	}
	for (unsigned i = 0; i < WLCM_NSPT; i++) { wl_cm.ppt_pre[i] = wl29_cmem_rd(wlcm_ppt_addr(&wl_cm, i)); }
	own = wlcm_cmem_owned(&wl_cm);
	if (own != WLCM_FREE) {
		printf("FUXI WLAN: W29 cookie conversion not programmed: %s; w20 0x4000 cc dropped for this boot\n", wlcm_why(own));
		return 0;
	}
	for (unsigned i = 0; i < WLCM_NSPT; i++) {
		uint32_t v = wlcm_ppt_val(&wl_cm, i);
		if (wl29_cmem(wlcm_ppt_addr(&wl_cm, i), true, &v)) {
			printf("FUXI WLAN: W29 cmem ppt write %u of %u (%#x) failed; contain\n", i + 1u, WLCM_NSPT, wlcm_ppt_addr(&wl_cm, i));
			return wl_w11_fail("W29 cmem ppt write");
		}
		wl29_cmem_writes++;
	}
	wlcm_spt_fill(&wl_cm, &wl_rx);
	wl_cm.cc_on = wl_rx.cc_on = wl_rd.cc_on = true;
	printf("FUXI WLAN: W29 cookie conversion cmem base %#x pages %u (rx %u tx %u) spt %#llx ppt writes %u; cc on\n",
	    wl_cm.cmem_base, WLCM_NSPT, WLCM_NUM_RX_SPT_PAGES, WLCM_NUM_TX_SPT_PAGES, wl_cm.spt_iova, wl29_cmem_writes);
	return 0;
}

/* The common-ring ownership pre-reads (wl27_setup calls this once, before the
 * pre group). Any owner: no common-ring write at all, W29 common rings dropped
 * for the boot (cc and C2d's own rings stand). */
static int
wl29_common_owned(void)
{
	uint32_t en = 0, based = 0, unread = 0;
	for (unsigned i = 0; i < 2u * WLCM_NRING + 2u; i++) { wl_cm.pre[i] = wl29_reg_rd(wlcm_pre_addr(&wl_cm, i)); }
	wl_cm.own = wlcm_rings_owned(&wl_cm);
	for (unsigned k = 0; k < WLCM_NRING; k++) {
		en += wl_cm.pre[2u * k + 1u] != ~0u && (wl_cm.pre[2u * k + 1u] & WLCE_MISC_SRNG_ENABLE);
		based += wl_cm.pre[2u * k] != ~0u && wl_cm.pre[2u * k] != 0u;
		unread += wl_cm.pre[2u * k] == ~0u || wl_cm.pre[2u * k + 1u] == ~0u;
	}
	printf("FUXI WLAN: W29 common before: rings %u enabled %u based %u unreadable %u; %s\n", WLCM_NRING, en, based, unread,
	    wl_cm.own == WLCM_FREE ? "not owned" : "refused");
	if (wl_cm.own != WLCM_FREE) {
		printf("FUXI WLAN: W29 common rings not programmed: %s; w20 0x4000 common rings dropped for this boot\n", wlcm_why(wl_cm.own));
		return 1;
	}
	return 0;
}

/* The pre group of ath12k_dp_srng_common_setup (dp.c:499-534), before C2d's
 * rel/exc: the SW2WBM release ring, then for each of three TX rings the TCL data
 * ring and its WBM completion ring, then the REO reinject ring. */
static int
wl29_common_pre(void)
{
	static const unsigned pre[] = { WLCM_R_SW2WBM, WLCM_R_TCL0, WLCM_R_COMP0, WLCM_R_TCL1, WLCM_R_COMP1, WLCM_R_TCL2,
		WLCM_R_COMP2, WLCM_R_REINJECT };
	if (!(wl20_bits & WL20_L19ORDER)) { return 0; }
	if (wl29_common_owned()) { return 0; }   /* dropped for the boot; C2d rings still run */
	wl_cm.rings_on = true;
	for (unsigned j = 0; j < sizeof(pre) / sizeof(pre[0]); j++) { if (wl29_ring(pre[j])) { return 1; } }
	return 0;
}

/* The post group (dp.c:548-568), after C2d's rel/exc and before reo_hw_setup:
 * the REO command ring (with ath12k_hal_reo_init_cmd_ring's per-entry command
 * numbers, hal_rx.c:871-894) and the REO status ring. */
static int
wl29_common_post(void)
{
	struct wlcm_ring *c = &wl_cm.ring[WLCM_R_REOCMD];
	if (!(wl20_bits & WL20_L19ORDER) || !wl_cm.rings_on) { return 0; }
	if (wl29_ring(WLCM_R_REOCMD)) { return 1; }
	for (uint32_t i = 0; i < c->entries; i++) { c->va[i * c->entry_dw + 2u] = (i + 1u) & 0xffffu; }   /* cmd number, word 2 (after the 8-byte tl) */
	printf("FUXI WLAN: W29 reo cmd ring initialised %u entries cmd-number 1..%u\n", c->entries, c->entries);
	if (wl29_ring(WLCM_R_REOSTATUS)) { return 1; }
	return 0;
}

/* ath12k_dp_cc_config (dp.c:1330-1380), at service ready before the REO
 * destination rings: the REO and WBM cookie configuration registers. CFG2 and
 * the convert-config are read-modify-write. Silent without cc. */
static int
wl29_cc_config(void)
{
	uint32_t regs[WLCM_CC_REGS][2], cfg2, conv;
	if (!(wl20_bits & WL20_L19ORDER) || !wl_cm.cc_on) { return 0; }
	cfg2 = wl29_reg_rd(WLCM_WBM_CC_CFG2);
	conv = wl29_reg_rd(WLCM_WBM_CC_CONVERT);
	if (cfg2 == ~0u || conv == ~0u) { return wl_w11_fail("W29 cc_config read"); }
	wlcm_cc_regs(&wl_cm, cfg2, conv, regs);
	for (unsigned k = 0; k < WLCM_CC_REGS; k++) {
		if (wl29_reg(regs[k][0], true, &regs[k][1])) {
			printf("FUXI WLAN: W29 cc_config write %u of %u (%#x) failed; contain\n", k + 1u, WLCM_CC_REGS, regs[k][0]);
			return wl_w11_fail("W29 cc_config register write");
		}
		wl29_writes++;
	}
	printf("FUXI WLAN: W29 cc_config reo-cfg1 %#x wbm-cfg1 %#x cfg2 %#x->%#x convert %#x->%#x; writes %u\n", WLCM_REO_CC_CFG1_VAL,
	    WLCM_CC_CFG1_COMMON, cfg2, regs[4][1], conv, regs[5][1], WLCM_CC_REGS);
	return 0;
}

/* One HTT PPDU_STATS_CFG (dp_tx.c:1328-1361): a single word msg =
 * type | pdev_mask << 9 | tlv_bitmask << 16, on the HTT endpoint, no reply. One
 * per num_rxdma_per_pdev (2 on WCN7850), pdev_mask = 1 << (i + pdev_idx 0). */
static int
wl29_ppdu_stats(void)
{
	for (unsigned i = 0; i < 2u; i++) {
		uint8_t msg[4];
		uint32_t w = WLHTT_H2T_PPDU_STATS_CFG | ((1u << i) & 0x7fu) << 9 | (WLHTT_PPDU_STATS_DEFAULT & 0xffffu) << 16;
		if (!wl_htc.htt_eid) { return wl_w11_fail("W29 PPDU stats without an HTT endpoint"); }
		wlce_put32(msg, w);
		if (wl_ce_send(wl_htc.htt_eid, msg, sizeof(msg), "HTT PPDU_STATS_CFG")) { return 1; }
		printf("FUXI WLAN: W29 htt ppdu stats cfg pdev-mask %#x msg %#010x sent\n", 1u << i, w);
	}
	return 0;
}

/* The mac-start pdev parameters before the chain mask and channel list
 * (ath12k_mac_start mac.c:9372-9415): PMF_QOS 1, DYNAMIC_BW 1, ARP_AC_OVERRIDE 0,
 * the DFS phyerr offload enable, the two PPDU stats configs, MESH_MCAST 1. All
 * host-to-firmware; no reply awaited. Silent without 0x4000. */
static int
wl29_macstart_pre(void)
{
	static const uint32_t pp[4][3] = {
		{ 0, WLWMI_PDEV_PARAM_PMF_QOS, 1u }, { 0, WLWMI_PDEV_PARAM_DYNAMIC_BW, 1u },
		{ 0, WLWMI_PDEV_PARAM_ARP_AC_OVERRIDE, 0u }, { 0, WLWMI_PDEV_PARAM_MESH_MCAST_ENABLE, 1u },
	};
	const uint32_t dfs[1] = { wl23_pdev };
	uint32_t a[3];
	if (!(wl20_bits & WL20_L19ORDER)) { return 0; }
	for (unsigned i = 0; i < 3u; i++) {   /* PMF_QOS, DYNAMIC_BW, ARP_AC_OVERRIDE */
		a[0] = wl23_pdev; a[1] = pp[i][1]; a[2] = pp[i][2];
		if (wl23_words(WLWMI_CMD_PDEV_SET_PARAM, WLWMI_TAG_PDEV_SET_PARAM_CMD, a, 3, "WMI PDEV_SET_PARAM (mac-start)")) { return 1; }
	}
	if (wl23_words(WLWMI_CMD_DFS_PHYERR_OFFLOAD, WLWMI_TAG_DFS_PHYERR_OFFLOAD_CMD, dfs, 1, "WMI DFS_PHYERR_OFFLOAD_ENABLE")) { return 1; }
	if (wl29_ppdu_stats()) { return 1; }
	a[0] = wl23_pdev; a[1] = pp[3][1]; a[2] = pp[3][2];
	if (wl23_words(WLWMI_CMD_PDEV_SET_PARAM, WLWMI_TAG_PDEV_SET_PARAM_CMD, a, 3, "WMI PDEV_SET_PARAM MESH_MCAST")) { return 1; }
	printf("FUXI WLAN: W29 mac-start pdev params pmf-qos 1 dynamic-bw 1 arp-ac 0 dfs-offload mesh-mcast 1 ppdu-stats 2 pdev %u\n",
	    wl23_pdev);
	return 0;
}

/* The mac-start LRO config and idle-PS after the channel list (mac.c:9452-9465).
 * LRO is not enabled (lro_enable stays 0; L19 randomises the seeds but never sets
 * lro_enable, so they shape nothing): the seeds are fixed at 0 here, documented.
 * Silent without 0x4000. */
static int
wl29_macstart_post(void)
{
	static uint8_t buf[8 + 4 * WLCM_LRO_WORDS];
	uint32_t w[WLCM_LRO_WORDS], len = 0;
	const uint32_t ips[3] = { wl23_pdev, WLWMI_PDEV_PARAM_IDLE_PS_CONFIG, 1u };
	if (!(wl20_bits & WL20_L19ORDER)) { return 0; }
	for (unsigned i = 0; i < WLCM_LRO_WORDS; i++) { w[i] = 0; }   /* lro_enable 0, res 0, th_4/th_6 fixed 0 seeds */
	w[WLCM_LRO_WORDS - 1u] = wl23_pdev;
	if (wlwmi_encode_words(WLWMI_CMD_LRO_CONFIG, WLWMI_TAG_LRO_INFO_CMD, w, WLCM_LRO_WORDS, buf, sizeof(buf), &len) ||
	    wl_ce_send(wl_htc.wmi_eid, buf, len, "WMI LRO_CONFIG")) { return 1; }
	if (wl23_words(WLWMI_CMD_PDEV_SET_PARAM, WLWMI_TAG_PDEV_SET_PARAM_CMD, ips, 3, "WMI PDEV_SET_PARAM IDLE_PS")) { return 1; }
	printf("FUXI WLAN: W29 mac-start lro cfg (seeds 0, lro-enable 0) idle-ps 1 pdev %u\n", wl23_pdev);
	return 0;
}

/* The vdev encap/decap of ath12k_mac_update_vif_offload (mac.c:9782-9827), right
 * after VDEV_CREATE and before NSS: TX_ENCAP_TYPE then RX_DECAP_TYPE, ETHERNET
 * when the firmware advertises WMI_TLV_SERVICE_ETH_OFFLOAD (461), NATIVE_WIFI
 * otherwise (no RAW mode here). Silent without 0x4000. */
static int
wl29_encap_decap(void)
{
	const uint32_t mode = wlwmi_svc(&wl_wmi, WLWMI_SVC_ETH_OFFLOAD) ? WLWMI_TXRX_ETHERNET : WLWMI_TXRX_NATIVE_WIFI;
	const uint32_t enc[3] = { WLWMI_VDEV_ID, WLWMI_VDEV_PARAM_TX_ENCAP_TYPE, mode };
	const uint32_t dec[3] = { WLWMI_VDEV_ID, WLWMI_VDEV_PARAM_RX_DECAP_TYPE, mode };
	if (!(wl20_bits & WL20_L19ORDER)) { return 0; }
	if (wl23_words(WLWMI_CMD_VDEV_SET_PARAM, WLWMI_TAG_VDEV_SET_PARAM_CMD, enc, 3, "WMI VDEV_SET_PARAM TX_ENCAP_TYPE") ||
	    wl23_words(WLWMI_CMD_VDEV_SET_PARAM, WLWMI_TAG_VDEV_SET_PARAM_CMD, dec, 3, "WMI VDEV_SET_PARAM RX_DECAP_TYPE")) { return 1; }
	printf("FUXI WLAN: W29 vdev 0 encap/decap %s (service 461 %s)\n", mode == WLWMI_TXRX_ETHERNET ? "ethernet" : "native-wifi",
	    wlwmi_svc(&wl_wmi, WLWMI_SVC_ETH_OFFLOAD) ? "set" : "not set");
	return 0;
}

/* In wl26_report, after the W27 lines and before the D5 hint: the W29 counts and
 * the arena guard (host reads only). Silent without 0x4000. */
static void
wl29_report(void)
{
	uint32_t rdp_gap = 0, ring_gap = 0, spt_lost = 0, tail = 0;
	unsigned on = 0;
	if (!(wl20_bits & WL20_L19ORDER)) { return; }
	for (unsigned k = 0; k < WLCM_NRING; k++) { on += wl_cm.ring[k].on; }
	printf("FUXI WLAN: W29 common rings %u of %u programmed, writes %u; cookie conversion %s (cmem writes %u, returns decoded %u)\n",
	    on, WLCM_NRING, wl29_writes, wl_cm.cc_on ? "on" : "off", wl29_cmem_writes, wl_rd.cc_entries);
	if (!wl_cm.stopping && wl_cm.base) {
		wlcm_guard_count(&wl_cm, &rdp_gap, &ring_gap, &spt_lost, &tail);
		printf("FUXI WLAN: W29 guard %snonzero dwords: rdp %u, ring-gap %u, spt-lost %u, tail %u%s\n",
		    rdp_gap || ring_gap || spt_lost || tail ? "SPILL " : "", rdp_gap, ring_gap, spt_lost, tail,
		    rdp_gap || ring_gap || spt_lost || tail ? "; stop; owner review before any further kit" : "");
	}
}

static int
wl_w23_run(void)
{
	static uint8_t cmd[WLCE_BUF];
	uint32_t len = 0, n = (wl20_bits & WL20_BAND24) ? WLSCAN_NCHAN_24 : WLSCAN_NCHAN, max_ms = wlscan_max_time(n);
	uint32_t eid = wl_htc.wmi_eid, sends = 0, bytes = 0, dfs = 0;
	unsigned took = 0;
	wl12_stage = WL23_SCAN;
	wlscan_init(&wl_scan, n);
	wl_wmi.scan = &wl_scan;
	wl23_scan_sent = wl23_htt_seen = false;
	wl23_lines = 0;
	wl23_ci_lines = wl23_ci_drop = 0;  /* C2b */
	/* L19 sends pdev 0 for a single-pdev chip such as WCN7850 (wmi.c:4688-4691;
	 * reg.c:178, mac.c:9740, 4894 read it); w20 0x80 takes the EXT MAC/PHY pdev. */
	wl23_pdev = (wl20_bits & WL20_PDEVCAP) ? wl_wmi.pdev_id : 0u;
	if (wl20_bits & WL20_HTTVER) {
		uint8_t req[WLHTT_VER_REQ_BYTES];
		if (!wl_htc.htt_eid) { return wl_w11_fail("W23 HTT version request without an HTT endpoint"); }
		wl_htc.htt_fn = wl23_htt_rx;
		wlhtt_version_req(req);
		/* HTT_TARGET_VERSION_TIMEOUT_HZ, 3 s (dp_tx.c:1275, 1312-1317) */
		if (wl_ce_send(wl_htc.htt_eid, req, sizeof(req), "HTT VERSION_REQ") || wl_ce_wait(WL23_W_HTT, 3000, "HTT VERSION_CONF")) { return 1; }
		printf("FUXI WLAN: W23 htt version %u.%u (w20 0x02)\n", wl23_htt_major, wl23_htt_minor);
		if (wl23_htt_major != WLHTT_VERSION_MAJOR) { return wl_w11_fail("W23 HTT major version is not 3 (L19 refuses it too, dp_tx.c:1319-1323)"); }
	}
	if (wl26_rings()) { return 1; }  /* C2c: SRING_SETUP, the setup watch, the chain masks; silent without W26 */
	uint32_t per = wlwmi_chan_list_max(wl_htc.ep[eid].max_msg);
	if (!per) { return wl_w11_fail("W23 WMI max message below one channel entry"); }
	for (uint32_t first = 0; first < n; first += per, sends++) {
		uint32_t k = n - first < per ? n - first : per;
		if (wlwmi_encode_chan_list(first, k, sends != 0, wl23_pdev, cmd, sizeof(cmd), &len)) { return wl_w11_fail("W23 SCAN_CHAN_LIST encode bounds"); }
		if (wl_ce_send(eid, cmd, len, "WMI SCAN_CHAN_LIST")) { return 1; }
		bytes += len;
	}
	for (uint32_t i = 0; i < n; i++) { dfs += wlscan_is_dfs(wlscan_chan(i)) ? 1u : 0u; }
	printf("FUXI WLAN: W23 chan list %u passive (2.4 GHz %u, 5 GHz %u, dfs %u) pdev %u commands %u bytes %u\n", n,
	    n < WLSCAN_NCHAN_24 ? n : WLSCAN_NCHAN_24, n > WLSCAN_NCHAN_24 ? n - WLSCAN_NCHAN_24 : 0u, dfs, wl23_pdev, sends, bytes);
	if (wl29_macstart_post()) { return 1; }   /* C2f: LRO config and idle-PS after the channel list (mac.c:9452-9465) */
	/* Streams = the pdev's chains on each band it has (mac.c:9753-9760,
	 * 14557-14560); without MAC/PHY caps, SERVICE_READY's rf chains on both. */
	uint32_t bands = wl_wmi.phy_bands & (WLWMI_HOST_2GHZ_CAP | WLWMI_HOST_5GHZ_CAP);
	uint32_t tx = wlwmi_popcount(wl_wmi.phy_tx_mask), rx = wlwmi_popcount(wl_wmi.phy_rx_mask);
	bool caps = bands && tx && rx;
	if (!caps) { bands = WLWMI_HOST_2GHZ_CAP | WLWMI_HOST_5GHZ_CAP; tx = rx = wl_wmi.num_rf_chains ? wl_wmi.num_rf_chains : 1u; }
	if (wlwmi_encode_vdev_create(wl23_mac, wl23_pdev, bands, tx, rx, cmd, sizeof(cmd), &len)) { return wl_w11_fail("W23 VDEV_CREATE encode bounds"); }
	if (wl_ce_send(eid, cmd, len, "WMI VDEV_CREATE")) { return 1; }
	printf("FUXI WLAN: W23 vdev 0 sta created mac %02x:%02x:%02x:%02x:%02x:%02x pdev %u bands %#x streams tx %u rx %u%s\n",
	    wl23_mac[0], wl23_mac[1], wl23_mac[2], wl23_mac[3], wl23_mac[4], wl23_mac[5], wl23_pdev, bands, tx, rx,
	    caps ? "" : " (rf-chains: no MAC/PHY caps)");
	if (wl29_encap_decap()) { return 1; }   /* C2f: vdev encap/decap before NSS (mac.c:9782-9827, called from vdev_create) */
	{
		uint32_t nss = tx ? tx : 1u; /* hweight32(cfg_tx_chainmask) ?: 1 (mac.c:10091) */
		const uint32_t vp[3] = { WLWMI_VDEV_ID, WLWMI_VDEV_PARAM_NSS, nss };
		const uint32_t ps[3][3] = {
			{ WLWMI_VDEV_ID, WLWMI_STA_PS_PARAM_RX_WAKE_POLICY, WLWMI_STA_PS_RX_WAKE_POLICY_WAKE },
			{ WLWMI_VDEV_ID, WLWMI_STA_PS_PARAM_TX_WAKE_THRESHOLD, WLWMI_STA_PS_TX_WAKE_THRESHOLD_ALWAYS },
			{ WLWMI_VDEV_ID, WLWMI_STA_PS_PARAM_PSPOLL_COUNT, WLWMI_STA_PS_PSPOLL_COUNT_NO_MAX },
		};
		const uint32_t mode[2] = { WLWMI_VDEV_ID, WLWMI_STA_PS_MODE_DISABLED };
		if (wl23_words(WLWMI_CMD_VDEV_SET_PARAM, WLWMI_TAG_VDEV_SET_PARAM_CMD, vp, 3, "WMI VDEV_SET_PARAM NSS")) { return 1; }
		for (unsigned i = 0; i < 3; i++) {
			if (wl23_words(WLWMI_CMD_STA_POWERSAVE_PARAM, WLWMI_TAG_STA_POWERSAVE_PARAM_CMD, ps[i], 3, "WMI STA_POWERSAVE_PARAM")) { return 1; }
		}
		if (wl23_words(WLWMI_CMD_STA_POWERSAVE_MODE, WLWMI_TAG_STA_POWERSAVE_MODE_CMD, mode, 2, "WMI STA_POWERSAVE_MODE")) { return 1; }
		printf("FUXI WLAN: W23 vdev 0 nss %u, sta ps rx-wake 0 tx-wake 1 pspoll 0, ps mode off\n", nss);
	}
	if (wlwmi_encode_start_scan(n, max_ms, cmd, sizeof(cmd), &len)) { return wl_w11_fail("W23 START_SCAN encode bounds"); }
	wl23_scan_sent = true; /* from here on containment asks for STOP_SCAN unless the scan is over */
	printf("FUXI WLAN: W23 scan start id %#x vdev 0 channels %u passive dwell %u ms max %u ms events %#x flags %#x\n",
	    WLWMI_SCAN_ID, n, WLSCAN_DWELL_PASSIVE, max_ms, WLSCAN_EVENTS, WLSCAN_FLAGS);
	/* L19 waits 1 s for STARTED (mac.c:5046-5053) */
	if (wl_ce_send(eid, cmd, len, "WMI START_SCAN") || wl_ce_wait(WL23_W_STARTED, 1000, "scan STARTED")) { wl23_ci_flush(); return 1; }
	if (wl_scan.start_failed) {
		printf("FUXI WLAN: W23 scan START_FAILED reason %u (ladder: w20 0x80, then 0x10, then 0x40)\n", wl_scan.reason);
		return wl_w11_fail("W23 scan START_FAILED");
	}
	printf("FUXI WLAN: W23 scan started after %u ms\n", wl_ce_waited);
	wl26_watch(WLRX_P_STARTED);  /* C2c */
	/* L19 arms its scan timeout at max_scan_time + 200 ms (mac.c:5385-5388) */
	int w = wl23_wait_done(max_ms + 200u, &took);
	if (w == 1) { wl23_ci_flush(); return 1; }
	if (w == 0) {
		printf("FUXI WLAN: W23 scan completed reason %u after %u ms (foreign %u chan-info %u%s)\n", wl_scan.reason, took,
		    wl_scan.foreign, wl_scan.ci_start, wl_scan.dequeued ? ", dequeued" : "");
	} else {
		printf("FUXI WLAN: W23 timeout scan COMPLETED (%u ms); report, contain, no retry\n", took);
		wl28_crash("scan COMPLETED timeout");  /* C2e: read-only, before anything else touches the device */
	}
	wl26_watch(WLRX_P_COMPLETED);  /* C2c: as soon as the wait returned, before the W24 lines */
	wl24_report();
	if (w == 2) {
		wl23_ci_flush();
		wl26_report();  /* C2c */
		wl_ce_snapshot("scan COMPLETED");
		return wl_w11_fail("scan COMPLETED");
	}
	if (!wl_scan.frames && (wl20_bits & WL26_BITS)) { wl26_heard_none = true; }  /* C2c D5: the hint follows the W26 lines */
	else if (!wl_scan.frames) {
		printf("FUXI WLAN: W24 no beacon or probe response heard (chan-info %u); next rung %s\n", wl_scan.ci_start,
		    (wl20_bits & WL20_HTTVER) ? "C2c = RX data path" : "W24b = w20 0x02");
	}
	if (wl23_stats()) { wl23_ci_flush(); wl26_report(); return 1; }  /* C2b: only the transport ends the run here; C2c: the W26 lines first */
	wl23_ci_flush();
	wl26_report();  /* C2c: content, guard, error ring, links, counters, D4 and D5 lines; silent without W26 */
	printf("FUXI WLAN: W23 complete; stop/contain\n");
	return 0;
}

/* Containment, before the stopping latch (wl_ce_contain): one STOP_SCAN
 * when a START_SCAN may have gone out and no COMPLETED, DEQUEUED or
 * START_FAILED came back, with L19's values (mac.c:4883-4896, wmi.c:
 * 2832-2878: requester 0, scan id 0xA000, STOP_ONE, vdev 0 as L19 leaves it,
 * pdev). Best effort: only on a healthy transport (no access fault, no
 * CE-layer error, HTC ready, and no W11 events/guard failure in any poll:
 * scan_plan.md 4 and 6.4), only with HTC credits in hand (no wait), a TX
 * drain of at most 100 ms, no RX consumption, through the same rings and
 * writers; nothing here reads the guard again or fails the abort. */
static void
wl23_stop_scan(void)
{
	static uint8_t cmd[4 + WLWMI_STOP_SCAN_BYTES], frame[WLCE_BUF];
	const uint32_t w[5] = { 0, WLWMI_SCAN_ID, WLWMI_SCAN_STOP_ONE, WLWMI_VDEV_ID, wl23_pdev };
	uint32_t len = 0, total = 0;
	unsigned t = 0;
	if (!wl23_scan_sent) { return; }
	if (wl_scan.done || wl_scan.start_failed) {
		printf("FUXI WLAN: W23 contain: scan %s, no STOP_SCAN\n", wl_scan.done ? "over" : "start failed");
		return;
	}
	if (fw_faults || wl_ce.errors || wl23_guard_bad || !wl_htc.ready || !wl_htc.wmi_eid) {
		printf("FUXI WLAN: W23 contain: STOP_SCAN not sent (transport not healthy)\n");
		return;
	}
	struct wlce_pipe *p = &wl_ce.pipe[wl_htc.ep[wl_htc.wmi_eid].ul_pipe];
	if (wlwmi_encode_words(WLWMI_CMD_STOP_SCAN, WLWMI_TAG_STOP_SCAN_CMD, w, 5, cmd, sizeof(cmd), &len) ||
	    !wlhtc_has_credits(&wl_htc, wl_htc.wmi_eid, WLHTC_HDR + len)) {
		printf("FUXI WLAN: W23 contain: STOP_SCAN not sent (no HTC credits)\n");
		return;
	}
	if (wlhtc_frame(&wl_htc, wl_htc.wmi_eid, cmd, len, frame, sizeof(frame), &total) ||
	    wlhtc_take_credits(&wl_htc, wl_htc.wmi_eid, total) || wlce_send(&wl_ce, &wl_ce_io, p, wl_htc.wmi_eid, frame, total)) {
		printf("FUXI WLAN: W23 contain: STOP_SCAN not sent (%s)\n", wl_ce.why ? wl_ce.why : wl_htc.why ? wl_htc.why : "-");
		return;
	}
	while (!wlce_tx_idle(p) && t < 100) {
		wl_sleep(1);
		t++;
		if (wlce_reap_tx(&wl_ce, &wl_ce_io, p)) { break; }
	}
	printf("FUXI WLAN: W23 contain: STOP_SCAN sent (best effort), tx drained %u after %u ms\n", wlce_tx_idle(p), t);
}

/* W22: INIT, READY, HW mode DBS (core.c:933-953), a 500 ms look for the HW
 * mode response and unprompted events, then stop (C1) or the scan (C2). */
static int
wl_w22_run(void)
{
	static uint8_t cmd[WLWMI_INIT_TOTAL];
	const uint8_t *rc = cmd + 4 + WLWMI_INIT_BYTES;
	uint32_t len = 0;
	wl12_stage = WL22_WMI;
	if (wlwmi_encode_init(&wl_wmi, cmd, sizeof(cmd), &len)) { return wl_w11_fail("W22 INIT encode bounds"); }
	if (wl_ce_send(wl_htc.wmi_eid, cmd, len, "WMI INIT")) { return 1; }
	printf("FUXI WLAN: W22 wmi init sent bytes %u (vdevs %u peers %u chains %#x flag1 %#x flags2 %#x host-svc %#x mem-chunks 0)\n",
	    len, wlce_get32(rc + 4 * WLWMI_RC_NUM_VDEVS), wlce_get32(rc + 4 * WLWMI_RC_NUM_PEERS),
	    wlce_get32(rc + 4 * WLWMI_RC_TX_CHAIN_MASK), wlce_get32(rc + 4 * WLWMI_RC_FLAG1),
	    wlce_get32(rc + 4 * WLWMI_RC_FLAGS2), wlce_get32(rc + 4 * WLWMI_RC_HOST_SERVICE_FLAGS));
	if (wl_ce_wait(WL20_W_READY, 5000, "WMI READY")) { return 1; }
	unsigned mac_zero = !(wl_wmi.mac[0] | wl_wmi.mac[1] | wl_wmi.mac[2] | wl_wmi.mac[3] | wl_wmi.mac[4] | wl_wmi.mac[5]);
	/* L19 stores the status and never acts on it (wmi.c:6910): logged only. */
	printf("FUXI WLAN: W22 wmi ready status %u mac-zero %u peers %u\n", wl_wmi.ready_status, mac_zero, wl_wmi.num_total_peers);
	if (wl20_bits & WL20_NOHWMODE) {
		printf("FUXI WLAN: W22 hw mode skipped (w20 0x40)\n");
	} else {
		static uint8_t hw[WLWMI_HW_MODE_TOTAL];
		if (wlwmi_encode_hw_mode(hw, sizeof(hw), WLWMI_HW_MODE_DBS, &len)) { return wl_w11_fail("W22 HW mode encode bounds"); }
		if (wl_ce_send(wl_htc.wmi_eid, hw, len, "WMI PDEV_SET_HW_MODE")) { return 1; }
		printf("FUXI WLAN: W22 hw mode dbs sent\n");
	}
	/* L19 does not wait for the response (wmi.c:4186-4216). */
	if (wl_ce_observe(WL20_W_NONE, 500)) { return 1; }
	printf("FUXI WLAN: W22 hw mode response %s; wmi events %u unknown %u\n", wl_wmi.got_hw_resp ? "seen" : "not seen",
	    wl_wmi.events, wl_wmi.unknown_events);
	printf("FUXI WLAN: W22 complete; %s\n", wl_w23 ? "W23 scan next" : "stop/contain");
	return wl_w23 ? wl_w23_run() : 0;
}

/* W21: connect HTT (unless w20 0x01) and WMI, SETUP_COMPLETE_EX, then the
 * service-ready set. L19 completes service ready from EXT when the target has
 * no EXT2 service, else from EXT2 (wmi.c:4940-4941, 5645), so EXT is always
 * awaited; AVAILABLE (services 128..) is optional and comes first. */
static int
wl_w21_run(void)
{
	uint8_t msg[WLHTC_SETUP_COMPLETE_BYTES];
	wl12_stage = WL21_HTC;
	if (wl20_bits & WL20_NOHTT) { printf("FUXI WLAN: W21 htc HTT connect skipped (w20 0x01)\n"); }
	else if (wl_ce_connect(WLHTC_SVC_HTT)) { return 1; }
	else if (wl20_bits & WL26_BITS) { wl_htc.htt_fn = wl23_htt_rx; }  /* C2c: every HTT message counted by type */
	if (wl_ce_connect(WLHTC_SVC_WMI)) { return 1; }
	wlhtc_setup_complete(msg);
	if (wl_ce_send(0, msg, sizeof(msg), "HTC SETUP_COMPLETE_EX")) { return 1; }
	printf("FUXI WLAN: W21 htc setup complete sent\n");
	if (wl_ce_wait(WL20_W_SVC, 5000, "WMI SERVICE_READY")) { return 1; }
	printf("FUXI WLAN: W21 wmi service ready fw %#x rf-chains %u mem-reqs %u scan-chans %u\n",
	    wl_wmi.fw_build, wl_wmi.num_rf_chains, wl_wmi.num_mem_reqs, wl_wmi.max_scan_chans);
	/* L19 never reads num_mem_reqs and never hands host memory chunks to
	 * WMI_INIT (wmi.c:4230, 4077-4083); stopping here is overnight_plan.md 4.4. */
	if (wl_wmi.num_mem_reqs) { return wl_w11_fail("W21 SERVICE_READY asks for host memory chunks (plan rule, not L19)"); }
	if (wl_ce_observe(WL20_W_AVAIL, 1000)) { return 1; }
	printf("FUXI WLAN: W21 wmi services ext %u ext2 %u reg-cc-ext %u (available event %u)\n",
	    wlwmi_svc(&wl_wmi, WLWMI_SVC_EXT_MSG), wlwmi_svc(&wl_wmi, WLWMI_SVC_EXT2_MSG),
	    wlwmi_svc(&wl_wmi, WLWMI_SVC_REG_CC_EXT), wl_wmi.got_avail);
	if (wl_ce_wait(WL20_W_EXT, 5000, "WMI SERVICE_READY_EXT")) { return 1; }
	printf("FUXI WLAN: W21 wmi service ready ext hw-modes %u caps %u pdev %u%s\n", wl_wmi.num_hw_modes,
	    wl_wmi.hw_mode_caps, wl_wmi.pdev_id, wl_wmi.pdev_valid ? "" : " (no MAC/PHY caps)");
	if (wlwmi_svc(&wl_wmi, WLWMI_SVC_EXT2_MSG)) {
		if (wl_ce_wait(WL20_W_EXT2, 5000, "WMI SERVICE_READY_EXT2")) { return 1; }
		printf("FUXI WLAN: W21 wmi service ready ext2 bytes %u\n", wl_wmi.ext2_bytes);
	}
	printf("FUXI WLAN: W21 wmi service ready complete\n");
	if (wl29_cc_config()) { return 1; }  /* C2f: ath12k_dp_cc_config at core.c:923, before the REO destination rings; silent without cc */
	if (wl27_dest()) { return 1; }  /* C2d: REO2SW1..8 after service ready, before INIT (core.c:925); silent without REO */
	return wl_w22_run();
}

/* W20: optional MRRS, arena, seven rings, HTC init, 62 + 62 RX buffers, HTC
 * READY (3 s; L19 waits 1 s, services the CEs, waits 1 s more: htc.c:491-515). */
static int
wl_w20_run(void)
{
	wl12_stage = WL20_CE;
	wl_ce_started = true; wl_ce_waited = wl_ce_logged = 0;
	bzero(&wl_htc, sizeof(wl_htc)); bzero(&wl_wmi, sizeof(wl_wmi));
	wl23_scan_sent = wl23_guard_bad = false; /* C2: no STOP_SCAN in containment unless W23 sent START_SCAN */
	wl23_rssi_conv = 0;
	if ((wl20_bits & WL20_MRRS128) && wl_ce_mrrs128()) { return 1; }
	if (wl_ce_alloc() || wl_ce_init_rings()) { return 1; }
	if (wl26_setup()) { return 1; }  /* C2c: W26 arenas, idle list, refill ring; silent without 0x100/0x200 */
	if (wlhtc_init(&wl_htc, wl15_service_ce, 14) ||
	    wlce_post_rx(&wl_ce, &wl_ce_io, &wl_ce.pipe[1], WLCE_RX_POST) ||
	    wlce_post_rx(&wl_ce, &wl_ce_io, &wl_ce.pipe[2], WLCE_RX_POST)) { return wl_ce_fail("HTC init/RX post"); }
	printf("FUXI WLAN: W20 ce rx posted CE1 %u CE2 %u\n", wl_ce.pipe[1].posted, wl_ce.pipe[2].posted);
	if (wl_ce_wait(WL20_W_CTL, 3000, "HTC READY")) { return 1; }
	if (wlhtc_parse_ready(&wl_htc)) { return wl_ce_fail("HTC READY"); }
	printf("FUXI WLAN: W20 htc ready credits %u size %u max-ep %u after %u ms\n", wl_htc.credit_count,
	    wl_htc.credit_size, wl_htc.max_ep, wl_ce_waited);
	return wl_w21_run();
}

/* First step of wl5_abort: latch "stopping" so the pure layer posts, sends,
 * consumes and publishes nothing more. The arena is freed with the other
 * buffers only when MHI reset, bus master off, PERST and the SMMU undo are
 * proven, else it stays quarantined (wl5_abort). C2's best-effort STOP_SCAN
 * goes in this function before the latch (the latch refuses every send): one
 * bounded send and a short TX drain whose failure never fails the abort
 * (wl23_stop_scan). */
static void
wl_ce_contain(unsigned step)
{
	(void)step;
	if (!wl_ce_started) { return; }
	wl_ce_started = false;
	wl26_contain();  /* C2c: W26 host writes and WBM access off first; silent without a W26 arena */
	if (wl_w23) { wl23_stop_scan(); } /* before the latch: it refuses every send */
	wl_ce.stopping = true;
	/* tx: CE0 + CE3, and CE4 (C2 w20 0x02 only; always 0 with qrtr 6) */
	printf("FUXI WLAN: W20 ce contain posted %u reaped %u tx %u\n", wl_ce.pipe[1].posted + wl_ce.pipe[2].posted,
	    wl_ce.pipe[1].reaped + wl_ce.pipe[2].reaped, wl_ce.pipe[0].posted + wl_ce.pipe[3].posted + wl_ce.pipe[4].posted);
}
/* ---- end W20 ---- */

/* Firmware-start QMI only: no CE allocation, register programming, HTC or WMI.
 * Adapted from Linux ath12k qmi.c (BSD-3-Clause-Clear), v6.12,
 * ath12k_qmi_firmware_start: INI -> CFG -> MODE, one reply at a time. */
static int
wl_w15_run(void)
{
	static struct wl12_encoder e;
	bzero(&e, sizeof(e)); e.len = 7;
	/* qmi_wlanfw_wlan_ini_req_msg_v01 / *_ei: optional 0x10 u8.
	 * Explicit lane policy: disable fw log (0). Upstream v6.12 sends 1. */
	printf("FUXI WLAN: W15 wlan ini enable_fwlog 0\n");
	if (wl12_tlv(&e, 0x10, 0, 1) || wl12_send(&e, 0x2f, WL15_INI) ||
	    wl12_wait(0, "WLAN_INI reply") || wl_w11_wait(3, "W15 INI TX drain")) { return 1; }
	printf("FUXI WLAN: W15 wlan ini ok\n");
	bzero(&e, sizeof(e)); e.len = 7;
	printf("FUXI WLAN: W15 wlan cfg host WIN target CE 9 service map 14 shadow v3 %u\n", wl15_variant ? 0 : 36);
	if (wl15_encode_cfg(&e) || wl12_send(&e, 0x23, WL15_CFG) ||
	    wl12_wait(0, "WLAN_CFG reply") || wl_w11_wait(3, "W15 CFG TX drain")) { return 1; }
	printf("FUXI WLAN: W15 wlan cfg ok\n");
	wl16_snapshot("CFG reply/before MODE");
	if (wl16_redrive) {
		printf("FUXI WLAN: W16 before MODE re-ring posted RX; no CE init before MODE (ath12k v6.12 order)\n");
		if (wl_w11_guard() || wl16_kick(21)) { return 1; }
	}
	bzero(&e, sizeof(e)); e.len = 7;
	/* qmi_wlanfw_wlan_mode_req_msg_v01 / *_ei: mandatory 0x01 u32
	 * ATH12K_FIRMWARE_MODE_NORMAL=0; optional 0x10 hw_debug u8=0. */
	printf("FUXI WLAN: W15 wlan mode normal hw_debug 0\n");
	if (wl12_tlv(&e, 1, 0, 4) || wl12_tlv(&e, 0x10, 0, 1) ||
	    wl_rails_on(2, "late before WLAN_MODE") ||
	    wl12_send(&e, 0x22, WL15_MODE) || wl12_wait(0, "WLAN_MODE reply") ||
	    wl_w11_wait(3, "W15 MODE TX drain") || wl_w11_guard()) { return 1; }
	printf("FUXI WLAN: W15 wlan mode normal ok; %s\n", wl_w20 ? "CE/HTC next" : "stop before CE/HTC");
	wl16_snapshot("MODE reply");
	wl12_stage = WL15_OBSERVE;
	unsigned observe = wl_w20 ? 200 : 2000; /* W20 moves on to the copy engines */
	printf("FUXI WLAN: W15 observing indications %u ms (reported only)\n", observe);
	/* Transport consumes/reposts IPCR RX buffers as usual, but indication
	 * contents cannot provision memory or trigger further QMI requests. */
	for (unsigned ms = 0; ms < observe; ms += 10) {
		if (wl_w11_events()) { return 1; }
		wl_sleep(10);
	}
	return wl_w11_events() || wl_w11_guard();
}

static int
wl_w14_run(void)
{
	// Unprogrammed OTP omits board_info; ath12k then uses board id 0xff. The
	// BDF here comes from fuxiwlanbdf (or its default), not from the id.
	if (!wl14_board_valid) { printf("FUXI WLAN: W14 CAP has no board id (OTP unset); ath12k default 0xff, BDF by name\n"); }
	if (wl14_download(WL14_REGDB_NAME, &wl14_regdb, WL14_REGDB) ||
	    wl14_download(wl14_bdf_name, &wl14_bdf, WL14_BDF)) { return 1; }
	unsigned size = 0;
	if (wl14_read(WL14_M3_NAME, &wl14_m3, WL14_M3_MAX, &size)) { return 1; }
	/* Clean after reading, before NC conversion and mapping/publication.
	 * Mapping checks live SMMU and every leaf.
	 * Firmware receives the IOVA and exact file size, never VA/backing PA. */
	wl_dma_clean(&wl14_m3);
	if (wl_probe == WL_PROBE_CIVAC) { wl_dma_civac(&wl14_m3); }
	if (wl_dma_nc(&wl14_m3) || wl12_map_target(&wl14_m3)) { return wl_w11_fail("W14 M3 DMA NC/map"); }
	static struct wl12_encoder e;
	bzero(&e, sizeof(e)); e.len = 7;
	wl14_ready = wl14_init_done = false;
	printf("FUXI WLAN: W14 M3 DMA IOVA %#llx bytes %u Normal-NC mapped\n", wl14_m3.iova, size);
	if (wl12_tlv(&e, 1, wl14_m3.iova, 8) || wl12_tlv(&e, 2, size, 4) ||
	    wl12_send(&e, 0x3c, WL14_M3) || wl12_wait(0, "W14 M3_INFO reply") ||
	    wl_w11_wait(3, "W14 M3 TX drain")) { return 1; }
	printf("FUXI WLAN: W14 m3 ok\n");
	wl12_stage = WL14_READY;
	if (wl12_wait(3, "W14 FW_READY indication") || wl_w11_guard()) { return 1; }
	if (wl_w15) {
		printf("FUXI WLAN: W15 fw ready; starting WLAN INI/CFG/MODE\n");
		wl12_stage = WL15_INI;
		int error = wl_w15_run();
		if (!error && wl_w20) { error = wl_w20_run(); } /* only after this boot's MODE reply */
		/* Preserve post-MODE or failure evidence before containment/reset. */
		wl_wlfw_diagnostics("W15 WLAN start");
		return error || wl_w11_guard();
	}
	printf("FUXI WLAN: W14 fw ready (ath11k 0x21 also seen %u); stop before WLAN config/mode\n", wl14_init_done);
	return 0;
}

static int
wl_w12_run(void)
{
	static struct wl12_encoder e;
	bzero(&e, sizeof(e)); e.len = 7;
	wl_target_n = 0; wl_target_total = 0;
	wl12_num_phy = 2; wl12_mlo_enabled = true;
	/* ath12k_qmi_event_server_arrive: PHY_CAP before IND_REGISTER/HOST_CAP.
	 * Its request EI is EOTI only: zero TLVs, seven-byte QMI header.
	 * Discovery stage stays zero until IND_REGISTER; old stage IDs unchanged. */
	if (wl12_mlo_variant == 1) {
		if (wl12_send(&e, 0x57, WL12_DISCOVERY) || wl12_wait(0, "PHY_CAP reply")) { return 1; }
		bzero(&e, sizeof(e)); e.len = 7;
	}
	/* IND_REGISTER: exactly the five enables + client ID set by ath12k.
	 * Pin/cal-download/cal-update/MSA/XO/rejuvenate TLVs remain absent. */
	if (wl12_tlv(&e, 0x10, 1, 1) || wl12_tlv(&e, 0x15, 0x4b4e454c, 4) ||
	    wl12_tlv(&e, 0x16, 1, 1) || wl12_tlv(&e, 0x17, 1, 1) ||
	    wl12_tlv(&e, 0x18, 1, 1) || wl12_tlv(&e, 0x1b, 1, 1) ||
	    wl12_send(&e, 0x20, WL12_IND) || wl12_wait(0, "IND_REGISTER reply")) { return 1; }
	printf("FUXI WLAN: W12 ind register ok\n");
	bzero(&e, sizeof(e)); e.len = 7;
	if (wl12_host_cap(&e) || wl12_send(&e, 0x34, WL12_HOST) ||
	    wl12_wait(0, "HOST_CAP reply")) { return 1; }
	printf("FUXI WLAN: W12 host cap ok\n");
	wl12_stage = WL12_REQUEST;
	if (wl12_wait(1, "REQUEST_MEM indication")) { return 1; }
	for (unsigned round = 0; round < 2; round++) {
		wl13_delayed = false;
		if (wl_w13 && !wl13_negotiated) {
			for (unsigned i = 0; i < wl_target_n; i++) {
				if (wl13_supported(wl_target_type[i]) && wl_target_size[i] > WL13_CHUNK_LIMIT) {
					wl13_delayed = true;
				}
			}
		}
		if (wl13_delayed) {
			wl13_negotiated = true;
			printf("FUXI WLAN: W13 decline large chunks n=%u limit=%u; RESPOND_MEM zero segments before any target allocation\n",
			    wl_target_n, WL13_CHUNK_LIMIT);
			wl12_requested = false; /* accept exactly one replacement, including a reply race */
		} else {
			for (unsigned i = 0; i < wl_target_n; i++) {
				if (wl_w13 && !wl13_supported(wl_target_type[i])) {
					printf("FUXI WLAN: W13 unsupported target type %u size %u; addr 0 (ath12k)\n",
					    wl_target_type[i], wl_target_size[i]);
					continue;
				}
				if (wl_w13) {
					uint64_t align = wl13_alignment(wl_target_size[i]);
					wl_iova_next = (wl_iova_next+align-1) & ~(align-1);
				}
				if (wl_target[i].va || wl_dma_alloc(&wl_target[i], wl_target_size[i]) ||
				    wl_dma_nc(&wl_target[i]) || wl12_map_target(&wl_target[i])) {
					return wl_w11_fail("target allocation/NC/map failed");
				}
				printf("FUXI WLAN: W12 target segment %u type %u size %u IOVA %#llx allocated %llu\n",
				    i, wl_target_type[i], wl_target_size[i], wl_target[i].iova, (uint64_t)wl_target[i].size);
			}
		}
		bzero(&e, sizeof(e)); e.len = 7;
		if (wl12_encode_respond(&e, wl13_delayed) ||
		    wl12_send(&e, 0x36, WL12_RESPOND) || wl12_wait(0, "RESPOND_MEM reply")) { return 1; }
		printf("FUXI WLAN: W12 respond mem ok\n");
		if (!wl13_delayed) { break; }
		printf("FUXI WLAN: W13 delayed response complete; waiting smaller REQUEST_MEM (10000 ms)\n");
		wl12_stage = WL12_REQUEST;
		if (wl12_wait(1, "smaller REQUEST_MEM")) { return 1; }
	}
	wl13_delayed = false;
	wl12_stage = WL12_READY;
	if (wl12_wait(2, "FW_MEM_READY indication")) { return 1; }
	printf("FUXI WLAN: W12 fw mem ready\n");
	bzero(&e, sizeof(e)); e.len = 7;
	if (wl12_send(&e, 0x24, WL12_CAP) || wl12_wait(0, "CAP_REQ reply") ||
	    wl_w11_wait(3, "W12 TX completion drain")) { return 1; }
	if (wl_w14) {
		if (wl_w14_run()) { return 1; }
		if (wl_w11_events() || wl_w11_wait(3, "W14 final TX drain") || wl_w11_guard()) { return 1; }
		wl_w11_ok = true;
		if (!wl_w20) { /* W22 printed its own completion line */
			printf("FUXI WLAN: %s complete; %s\n", wl_w15 ? "W15" : "W14", wl_keep ? "keep requested, DMA/channels/wake retained (polling ended)" : "stop/contain");
		}
		return 0;
	}
	wl12_stage = WL12_DONE;
	printf("FUXI WLAN: W12 capability complete; next ath12k host steps regdb/BDF/M3 deferred; observing indications 200 ms\n");
	for (unsigned ms = 0; ms < 200; ms += 10) {
		if (wl_w11_events()) { return 1; }
		wl_sleep(10);
	}
	if (wl_w11_events() || wl_w11_guard() || wl_w11_wait(3, "W12 observation TX drain")) { return 1; }
	wl_w11_ok = true;
	printf("FUXI WLAN: W12 handshake complete; %s\n", wl_keep ? "keep requested, DMA/channels/wake retained (polling ended)" : "stop/contain");
	return 0;
}

static int
wl_w10_tx_refuse(unsigned code, const char *reason)
{
	printf("FUXI WLAN: W10 BHIe TX refused: %s (reason %u); contain, no retry\n", reason, code);
	return (int)code;
}

static int
wl5_step(unsigned s)
{
	switch (s) {
	case 17:
		if (wl_variant_invalid) { return 0x6c; }
		if (wl_cnss && wl_pdc_acks != 13) { return 0x6e; }
		if (wl_probe == WL_PROBE_INVALID) { return 0x6b; }
		/* M4 validated BHIOFF; independently validate all M5 block extents. */
		wl_bhi = wl4_report[25]; wl_bhie = wl4_report[26];
		if (wl_bhi < 0x40 || wl_bhi > 0x3fc0 || (wl_bhi & 3) ||
		    wl_bhie < 0x40 || wl_bhie > 0x3f80 || (wl_bhie & 3)) { return 4; }
		wl_mhi_valid = true;
		if ((wl_seq_flags & WL_O_CYCLE) && wl_m6o_cycle()) { return 0x6f; }
		return wl_smmu_probe();
	case 18: {
		uint32_t saved = wl_mrd(0x310c);
		wl_mwr(0x310c, 0x40000036); /* EP internal TCSR, NOT host TCSR */
		if (wl_mrd(0x310c) != 0x40000036) { return 1; }
		uint32_t hw = wl_prd((uint64_t)WL_BAR0 + 0x80000);
		wl_mwr(0x310c, saved);
		wl5_report[16] = hw;
		if (!hw || hw == ~0u || fw_faults) { return 2; }
		wl_fw_path = ((hw >> 8) & 15) == 2 ?
		    "/usr/local/share/fuxi/wlan/amss20.bin" : "/usr/local/share/fuxi/wlan/amss.bin";
#if defined(FUXI_SALAMI)
		if (wl_amss_path[0]) { wl_fw_path = wl_amss_path; } /* fuxiwlanamss; never with wl_fw */
#endif
		if (wl_fw) {
			if (wl_prd(WL_EPCFG) != 0x110717cb || ((hw >> 8) & 15) != 2) { return 0x6c; }
			wl_fw_path = "/usr/local/share/fuxi/wlan/stock-amss20.bin";
			printf("FUXI WLAN: M6m stock input selected major 2; no fallback or provenance claim\n");
		}
		printf("FUXI WLAN: TCSR_SOC_HW_VERSION %#x major %u minor %u firmware %s (CNSS kiwi major-version rule)\n", hw, (hw >> 8) & 15, hw & 255, wl_fw_path);
		if (wl_pre_bhi()) { return 0x6c; }
		return wl_m6o_early() && wl_m6o_master() ? 0x6f : 0;
	}
	case 19: return wl_read_firmware();
	case 20: {
		if (!(wl_seq_flags & WL_O_MSI) && wl_disable_irqs()) { return 4; }
		if (wl_mrd(wl_bhi + 0x28) != 0 || (wl_mrd(0x38) & 2) ||
		    ((wl_mrd(0x48) & 0xff04) && !((wl_seq_flags & WL_O_POR) &&
		    ((wl_mrd(0x48)>>8)&255) == 0xff)) || ((wl_prd(WL_EPCFG + 4) & 4) != (wl_m6o_early() ? 4u : 0u))) { return 1; }
		struct wl_dma *amss = wl_amss.va ? &wl_amss : &wl_image;
		unsigned n = ((wl_seq_flags & WL_O_PREFIX) && wl_stop22) ? 0 : (wl_fw_size + WL_CHUNK - 1) / WL_CHUNK;
		if ((n && wl_dma_alloc(&wl_vectors, n * sizeof(struct wl_vector))) ||
		    wl_dma_alloc(&wl_contexts, 16384)) { return 2; }
		if (wl_qrtr && (wl_dma_alloc(&wl_ipcr, WL_IPCR_BYTES) ||
		    wl_dma_nc(&wl_contexts) || wl_dma_nc(&wl_ipcr))) { return 2; }
		struct wl_vector *v = (struct wl_vector *)wl_vectors.va;
		for (unsigned i = 0; i < n; i++) {
			unsigned off = i * WL_CHUNK;
			v[i].addr = amss->iova + off;
			v[i].size = MIN(WL_CHUNK, wl_fw_size - off);
		}
		struct wl_context *e = (struct wl_context *)(wl_contexts.va + WL_ECTX);
		for (unsigned i = 0; i < 2; i++) {
			e[i].type = 1; e[i].base = e[i].rp = wl_contexts.iova + (i ? WL_ER1 : WL_ER0);
			e[i].len = i ? 4096 : 512; e[i].wp = e[i].base + e[i].len - 16;
		}
		struct wl_context *c = (struct wl_context *)(wl_contexts.va + WL_CCTX);
		c->base = c->rp = c->wp = wl_contexts.iova + WL_CR; c->len = 2048;
		wl_dma_clean(&wl_contexts); wl_dma_clean(&wl_vectors);
		wl5_report[19] = n * sizeof(struct wl_vector);
		if ((wl_seq_flags & WL_O_POR) && wl_m6o_rx_clear()) { return 0x6f; }
		if (wl_vec == 2 || (wl_seq_flags & WL_O_POR)) {
			unsigned rxn = WL_RDDM_SIZE / WL_CHUNK;
			if (wl_dma_alloc(&wl_rddm, WL_RDDM_SIZE) ||
			    wl_dma_alloc(&wl_rxvectors, rxn * sizeof(struct wl_vector))) { return 2; }
			struct wl_vector *rx = (struct wl_vector *)wl_rxvectors.va;
			for (unsigned i = 0; i < rxn; i++) {
				rx[i].addr = wl_rddm.iova + i * WL_CHUNK; rx[i].size = WL_CHUNK;
			}
			wl_dma_clean(&wl_rddm); wl_dma_clean(&wl_rxvectors);
		}
		wl5_report[20] = (uint32_t)wl_vectors.iova; wl5_report[21] = (uint32_t)wl_contexts.iova;
		printf("FUXI WLAN: prepared %u vectors (%u bytes), 44-byte contexts, channels disabled; BME follows selected sequence\n", n, wl5_report[19]);
		if (fw_faults) { return 3; }
		int setup = wl_smmu_enable();
		if (!setup && (wl_seq_flags & WL_O_POR)) { setup = wl_m6o_prepare(); }
		return setup;
	}
	case 21: {
		if (wl_probe_fired) { return 0x6b; } /* never a second doorbell this boot */
		if (!wl_enabled || wl_smmu_fault(21) || wl_rc_check() ||
		    wl_prd(WL_SMMU) != wl5_report[4] ||
		    wl_prd(WL_SMMU + 0x800 + 4 * wl_slot) != 0x80001401 ||
		    wl_prd(WL_SMMU + 0xc00 + 4 * wl_slot) != wl_cb ||
		    wl_smmu_verify("CB_SCTLR pre-DMA", wl_cb_addr(wl_cb), WL_CB_SCTLR,
		    wl_prd(wl_cb_addr(wl_cb)), WL_CB_SCTLR, WL_CB_SCTLR_MASK) ||
		    (wl_prd(WL_PARF + 0x2c00) & 1) || wl_prd(WL_PARF + 0x234)) { return 6; }
		if (!wl_m6o_early() && wl_fabric_prepare()) {
			wl_host_diag("preflight-blocked");
			return 10;
		}
		if (!wl_m6o_early() && wl_download_aspm()) { return 11; }
		/* ath12k_mhi_clear_vector: undefined reset status is not a result.
		 * Only doorbells/status, no RDDM buffers and no retry. */
		const unsigned clear[] = {0x3c, 0x44, 0x70, 0x78};
		unsigned clear_count = (wl_seq_flags & WL_O_POR) ? 2 : 4;
		for (unsigned i = 0; i < clear_count; i++) { wl_mwr(wl_bhie + clear[i], 0); }
		for (unsigned i = 0; i < clear_count; i++) {
			if (wl_mrd(wl_bhie + clear[i]) || fw_faults) { return 9; }
		}
		printf("FUXI WLAN: M6f BHIe %s doorbells/status cleared; %s\n",
		    (wl_seq_flags & WL_O_POR) ? "TX (prepared RX retained)" : "TX/RX",
		    (wl_vec == 2 || (wl_seq_flags & WL_O_POR)) ? "RX buffers prepared" : "no RX buffers");
		if (wl_vec && !(wl_seq_flags & WL_O_POR && wl_vec == 2)) {
			bool rx = wl_vec == 2;
			uint32_t off = rx ? 0x60 : 0x2c;
			uint64_t addr = rx ? wl_rxvectors.iova : wl_vectors.iova;
			unsigned bytes = rx ? (WL_RDDM_SIZE / WL_CHUNK) * sizeof(struct wl_vector) : wl5_report[19];
			if (rx) {
				/* Clear complete RX setup, not only DB/status, as MHI prepare does. */
				for (unsigned o = 0x60; o <= 0x78; o += 4) { wl_mwr(wl_bhie + o, 0); }
			}
			wl_mwr64(wl_bhie + off, addr); wl_mwr(wl_bhie + off + 8, bytes);
			if (wl_mrd(wl_bhie + off) != (uint32_t)addr ||
			    wl_mrd(wl_bhie + off + 4) != (uint32_t)(addr >> 32) ||
			    wl_mrd(wl_bhie + off + 8) != bytes || fw_faults) { return 12; }
			if (rx) {
				uint32_t seq; read_random(&seq, sizeof(seq));
				seq &= WL_SEQ_MASK; if (!seq) { seq = 1; }
				wl_mwr(wl_bhie + 0x70, seq);
				if (wl_mrd(wl_bhie + 0x70) != seq || fw_faults) { return 12; }
			}
			printf("FUXI WLAN: M6l BHIe %s staged bytes %u before BHI; %s\n",
			    rx ? "RDDM RX" : "AMSS TX", bytes, rx ? "RX doorbell armed; sink only" : "TX doorbell remains zero");
		}
		wl_probe_pattern();
		if (!(wl_seq_flags & WL_O_POR)) { wl_mwr(wl_bhi + 0x20, 0); }
		wl_mwr(wl_bhi + 0x2c, 0);
		wl_mwr64(wl_bhi + 8, wl_image.iova); wl_mwr(wl_bhi + 0x10, wl_bhi_bytes());
		wl_dma_clean(&wl_image);
		if (wl_bhi_mapping_check()) { return 8; }
		/* Flush posted setup writes and verify the exact Linux BHI tuple before
		 * enabling bus mastering. No MMIO write to MHICTRL before BHI. */
		uint32_t hi = wl_mrd(wl_bhi + 0x0c), lo = wl_mrd(wl_bhi + 8);
		uint32_t size = wl_mrd(wl_bhi + 0x10), ivec = wl_mrd(wl_bhi + 0x20);
		uint32_t status = wl_mrd(wl_bhi + 0x2c), ee = wl_mrd(wl_bhi + 0x28);
		printf("FUXI WLAN: M6e BHI preflight addr %#x:%08x size %#x INTVEC %#x STATUS %#x EE %#x image/tables cache-clean + dsb sy\n",
		    hi, lo, size, ivec, status, ee);
		if (fw_faults) { return 1; }
		if (hi != (uint32_t)(wl_image.iova >> 32) || lo != (uint32_t)wl_image.iova ||
		    size != wl_bhi_bytes() || ivec || status || ee != 0 || (wl_image.iova & 4095)) { return 7; }
		/* Default enables BME here; M6o early-MASTER modes must retain it.
		 * INTx stays disabled; MSI either disabled or targets the owned sink. */
		if (wl_m6o_early() && !(wl_prd(WL_EPCFG+4)&4)) { return 0x6f; }
		if (wl_seq_flags & WL_O_MSI) {
			unsigned p = wl_msi_cap;
			if (!p || (wl_prd(WL_EPCFG+p)&0x00710000) != 0x00510000 ||
			    wl_prd(WL_EPCFG+p+4) != (uint32_t)wl_msi_sink.iova ||
			    (wl_msi_64 && wl_prd(WL_EPCFG+p+8) != (uint32_t)(wl_msi_sink.iova>>32)) ||
			    (wl_prd(WL_EPCFG+p+wl_msi_data_off())&0xffff) ||
			    (wl_msi_pvm && wl_prd(WL_EPCFG+p+wl_msi_mask_off()))) { return 0x6f; }
		}
		if (!wl_m6o_early()) { wl_pwr(WL_EPCFG + 4, (wl_prd(WL_EPCFG + 4) & 0xffff) | 0x404); }
		if ((wl_prd(WL_EPCFG + 4) & 0x406) != 0x406 || fw_faults) { return 2; }
		wl_host_diag("preflight");
		/* Step 14 sets RC BME already. Prove it survived; do not silently
		 * repair changed command state or emit the BHI doorbell after a fault. */
		if (fw_faults || !(wl_prd(WL_DBI + 4) & 4)) { return 10; }
		read_random(&wl_seq, sizeof(wl_seq));
		wl_seq &= WL_SEQ_MASK;
		if (!wl_seq) { wl_seq = 1; } /* bounded nonzero fallback */
		wl_probe_publish(); /* last CPU buffer operation before the doorbell */
		uint64_t now = mach_absolute_time(), perst_ns = 0, link_ns = 0;
		absolutetime_to_nanoseconds(now - wl_perst_done, &perst_ns);
		absolutetime_to_nanoseconds(now - wl_link_done, &link_ns);
		printf("FUXI WLAN: M6k BHI timing since PERST-step completion %llu ms since link-step completion %llu ms (PERST sample is 1 ms after deassert)\n",
		    perst_ns / 1000000ull, link_ns / 1000000ull);
		wl_probe_fired = true;
		wl_mwr(wl_bhi + 0x18, wl_seq);
		printf("FUXI WLAN: M6f BHI doorbell random seq %#x mask %#x bytes %#x bus master on\n", wl_seq, WL_SEQ_MASK, wl_bhi_bytes());
		return fw_faults ? 3 : 0;
	}
	case 22: {
		if (wl_stop22 || wl_probe == WL_PROBE_UNMAPPED || wl_probe == WL_PROBE_PATTERN) {
			printf("FUXI WLAN: W10 diagnostic stop at 22; no AMSS TX\n");
			return wl_probe_observe();
		}
		int e = wl_poll(0);
		uint32_t first = wl_bhi_diag(0);
		wl5_report[124] = wl5_report[67]; wl5_report[125] = wl5_report[68];
		wl5_report[126] = first; wl5_report[127] = (uint32_t)e;
		wl_sleep(10); /* diagnostic repeat only; never retry a failed transfer */
		uint32_t again = wl_bhi_diag(1);
		bool failed = e || ((first | again) & (WL_DIAG_PBL_ERROR | WL_DIAG_TRANSPORT)) ||
		    (wl5_report[67] >> 30) == 3 || wl5_report[64] == 3 || wl5_report[64] == 6 ||
		    (wl5_report[65] & 4) || ((wl5_report[65] >> 8) & 255) == 255 || (wl5_report[66] & 2);
		wl_host_diag(failed ? "failure" : "completion");
		if (e || ((first | again) & WL_DIAG_PBL_ERROR)) {
			if (!((first | again) & WL_DIAG_TRANSPORT)) { wl_boot_logs(); }
			else { printf("FUXI WLAN: M6f boot logs skipped transport flags %#x\n", first | again); }
		}
		if (fw_faults) { return e == 3 ? e : 5; }
		if ((first | again) & WL_DIAG_TRANSPORT) { return e == 3 ? e : 5; }
		/* A late error/EE loss in the repeat must not turn into step-22 OK. */
		if (!e && ((wl5_report[67] >> 30) == 3 || wl5_report[64] == 3 || wl5_report[64] == 6 ||
		    (wl5_report[65] & 4) || ((wl5_report[65] >> 8) & 255) == 255 || (wl5_report[66] & 2))) { return 1; }
		if (!e && ((wl5_report[67] >> 30) != 2 || (wl5_report[67] & WL_SEQ_MASK) != wl_seq ||
		    wl5_report[64] != 1 || (wl5_report[65] & 0xff01) != 0x101)) { return 1; }
		if (!e) {
			wl_sbl_ready = true;
			printf("FUXI WLAN: W10 SBL READY success BHI STATUS %#x seq %#x; continue to AMSS\n", wl5_report[67], wl_seq);
		}
		return e;
	}
	case 23: return wl_sbl_ready ? wl_poll(1) : 1;
	case 24: {
		if (!wl_sbl_ready || wl_contexts_published || wl_poll(1)) { return 3; }
		uint32_t erdb = wl_mrd(0x20);
		uint32_t chdb = wl_mrd(0x18);
		if ((erdb & 7) || erdb < 0x100 || erdb > 0x3ff0 ||
		    (chdb & 7) || chdb < 0x100 || chdb > 0x3c00 ||
		    (erdb < chdb + 128 * 8 && erdb + 16 > chdb) ||
		    (wl_mrd(0x10) & 255) != 128) { return 1; }
		wl_mwr64(0x58, wl_contexts.iova); wl_mwr64(0x60, wl_contexts.iova + WL_ECTX);
		wl_mwr64(0x68, wl_contexts.iova + WL_CCTX);
		wl_mwr64(0x80, 0); wl_mwr64(0x88, 0xffffffffull);
		wl_mwr64(0x98, 0); wl_mwr64(0xa0, 0xffffffffull);
		wl_mwr(0x10, (wl_mrd(0x10) & 0xffff) | (2u << 16));
		const unsigned off[] = {0x58, 0x60, 0x68, 0x80, 0x88, 0x98, 0xa0};
		const uint64_t val[] = {wl_contexts.iova, wl_contexts.iova + WL_ECTX,
		    wl_contexts.iova + WL_CCTX, 0, 0xffffffffull, 0, 0xffffffffull};
		for (unsigned i = 0; i < sizeof(off)/sizeof(off[0]); i++) {
			if (wl_mrd(off[i]) != (uint32_t)val[i] ||
			    wl_mrd(off[i]+4) != (uint32_t)(val[i]>>32) || fw_faults) { return 2; }
		}
		if (((wl_mrd(0x10)>>16)&255) != 2 || fw_faults) { return 2; }
		/* Event ring doorbells only: channel contexts remain disabled and the
		 * command ring empty. No channel (including wake) doorbell is rung. */
		wl_mwr64(erdb, wl_contexts.iova + WL_ER0 + 512 - 16);
		wl_mwr64(erdb + 8, wl_contexts.iova + WL_ER1 + 4096 - 16);
		wl_contexts_published = !fw_faults;
		printf("FUXI WLAN: W10 contexts published; event rings only, all 128 channels disabled\n");
		return fw_faults ? 2 : 0;
	}
	case 25: {
		if (!wl_contexts_published || wl_poll(1)) { return 1; }
		wl_mwr(0x38, (wl_mrd(0x38) & ~0xff00u) | 0x200);
		int e = wl_poll(2);
		wl_sbl_m0 = !e;
		return e;
	}
	case 26: {
		if (wl_bhie_fired || !wl_probe_fired || !wl_sbl_m0 || wl_stop22 ||
		    !wl_enabled || wl_smmu_fault(26) || wl_rc_check() || fw_faults) {
			return wl_w10_tx_refuse(1, "one-shot/ownership/SBL M0 guard");
		}
		if (wl_poll(2) || wl_mrd(0x28) != wl_bhi || wl_mrd(0x2c) != wl_bhie ||
		    (wl_prd(WL_EPCFG+4)&0x406) != 0x406 || fw_faults) {
			return wl_w10_tx_refuse(1, "SBL/discovery/bus-master state changed");
		}
		struct wl_dma *amss = wl_amss.va ? &wl_amss : &wl_image;
		unsigned n = (wl_fw_size + WL_CHUNK - 1) / WL_CHUNK;
		if (!wl_vectors.va || amss->size < wl_fw_size || !wl_fw_size ||
		    wl5_report[19] != n * sizeof(struct wl_vector) || wl_vectors.size < wl5_report[19]) {
			return wl_w10_tx_refuse(2, "missing/truncated full AMSS or vector table");
		}
		struct wl_vector *v = (struct wl_vector *)wl_vectors.va;
		for (unsigned i = 0; i < n; i++) {
			if (v[i].addr != amss->iova + i * WL_CHUNK ||
			    v[i].size != MIN(WL_CHUNK, wl_fw_size - i * WL_CHUNK)) {
				return wl_w10_tx_refuse(2, "vector does not cover owned full image");
			}
		}
		wl_dma_clean(amss); wl_dma_clean(&wl_vectors); wl_dma_clean(&wl_tables);
		if (wl_probe == WL_PROBE_CIVAC) { wl_dma_civac(amss); wl_dma_civac(&wl_vectors); wl_dma_civac(&wl_tables); }
		wl_mwr(wl_bhie + WL_BHIE_TXSTATUS, 0);
		wl_mwr64(wl_bhie + WL_BHIE_TXADDR, wl_vectors.iova);
		wl_mwr(wl_bhie + WL_BHIE_TXSIZE, wl5_report[19]);
		if (wl_mrd(wl_bhie + WL_BHIE_TXADDR) != (uint32_t)wl_vectors.iova ||
		    wl_mrd(wl_bhie + WL_BHIE_TXADDR + 4) != (uint32_t)(wl_vectors.iova >> 32) ||
		    wl_mrd(wl_bhie + WL_BHIE_TXSIZE) != wl5_report[19] ||
		    wl_mrd(wl_bhie + WL_BHIE_TXSTATUS) || fw_faults) {
			return wl_w10_tx_refuse(3, "TX tuple/status readback failed");
		}
		read_random(&wl_bhie_seq, sizeof(wl_bhie_seq));
		wl_bhie_seq &= WL_SEQ_MASK;
		if (!wl_bhie_seq || wl_bhie_seq == wl_seq) {
			wl_bhie_seq = (wl_seq + 1) & WL_SEQ_MASK;
			if (!wl_bhie_seq) { wl_bhie_seq = 1; }
		}
		uint32_t db = wl_mrd(wl_bhie + WL_BHIE_TXDB);
		if (fw_faults || db == ~0u) { return wl_w10_tx_refuse(3, "TX doorbell unreadable"); }
		printf("FUXI WLAN: W10 BHIe TX preflight table %#llx bytes %u vectors %u full AMSS %u; cache-clean + dsb sy\n",
		    wl_vectors.iova, wl5_report[19], n, wl_fw_size);
		if (wl_poll(2) || wl_smmu_fault(26) || fw_faults) {
			return wl_w10_tx_refuse(1, "SBL/SMMU changed during TX prepare");
		}
		wl_bhie_fired = true; /* latch before the only TX attempt, even on fault */
		wl_mwr(wl_bhie + WL_BHIE_TXDB, (db & ~WL_SEQ_MASK) | wl_bhie_seq);
		printf("FUXI WLAN: W10 BHIe TX doorbell seq %#x (one shot)\n", wl_bhie_seq);
		return fw_faults ? 4 : 0;
	}
	case 27: return wl_bhie_fired ? wl_poll(3) : 1;
	case 28: {
		if (!wl_bhie_fired) { return 1; }
		int e = wl_poll(4);
		wl_amss_reached = !e;
		if (!e && wl_qrtr) { return wl_w11_run(); }
		if (!e) { printf("FUXI WLAN: W10 AMSS EE reached; M0 confirmed; no channels/QMI; %s\n",
		    wl_keep ? "keep-running requested, DMA buffers retained" : "stop/contain"); }
		return e;
	}
	}
	return 0xff;
}

/* Cleanup is the only exception to the normal write-after-fault latch. These
 * containment/owned undo destinations were mapped before bring-up. Keep each access
 * fault-contained, and never clear the original fault counters. */
static int
wl_contain_wr(uint64_t pa, uint32_t value)
{
	uintptr_t va = fw_va(pa); uint64_t v = value;
	if (!va || wl_link_refused(pa)) { return 1; }
	__asm__ volatile("dsb sy" ::: "memory");
	return (int)fuxi_gpu_access(va, &v, 1, 0);
}

static void
wl5_abort(unsigned step)
{
	wl_ce_contain(step); /* W20: latch CE stop first; returns at once when W20 never ran */
	if (wl_w11_attempted) { wl_w11_stop(); }
	wl5_status(step);
	(void)wl_smmu_fault(step);
	wl28_crash("contain before MHI reset");  /* C2e: read-only; silent without w20 0x2000 */
	int reset_bad = 0;
	if (wl_mhi_valid) {
		reset_bad = wl_contain_wr((uint64_t)WL_BAR0 + 0x38, 2);
		for (unsigned i = 0; !reset_bad && i <= 40; i++) {
			uint32_t ctrl = wl_mrd(0x38);
			if (ctrl == ~0u) { reset_bad = 1; break; }
			if (!(ctrl & 2)) { break; }
			if (i == 40) { reset_bad = 1; break; }
			wl_sleep(25);
		}
	}
	/* PCI command upper word is W1C status: always write only low 16 bits. */
	uint32_t cmd = wl_prd(WL_EPCFG + 4);
	int bm_bad = wl_contain_wr(WL_EPCFG + 4, cmd & 0xffff & ~4u);
	cmd = wl_prd(WL_EPCFG + 4); bm_bad |= cmd == ~0u || !!(cmd & 4);
	if (!bm_bad && wl_seq_flags && wl_m6o_msi_undo()) { bm_bad = 1; }
	/* gpio94 was configured as an output at step 5; clear only its OUT bit. */
	uint64_t perst = TS_TLMM + 0x1000ull * 94 + 4;
	uint32_t io = wl_prd(perst);
	int perst_bad = wl_contain_wr(perst, io & ~2u);
	io = wl_prd(perst); perst_bad |= io == ~0u || !!(io & 2);
	__asm__ volatile("dsb sy" ::: "memory");
	wl_sleep(10); /* PERST assertion + outstanding transaction drain */
	printf("FUXI WLAN: abort step %u MHI reset %s, bus master off %s, PERST asserted %s\n", step,
	    reset_bad ? "unconfirmed" : "done", bm_bad ? "unconfirmed" : "yes", perst_bad ? "unconfirmed" : "yes");
	/* No DMA buffer is ever returned to the allocator while containment is
	 * unproven. This is a bounded leak on an MMIO fault, never a panic/UAF. */
	int seq_bad = wl_seq_flags ? wl_m6o_undo(!perst_bad) : 0;
	if (!bm_bad && !perst_bad && !seq_bad && !wl_smmu_undo()) {
		wl26_free();  /* C2c: guards counted again, both W26 arenas zeroed, then freed; silent without them */
		wl_dma_free(&wl_msi_sink); wl_dma_free(&wl_tables);
		wl_dma_free(&wl_rxvectors); wl_dma_free(&wl_rddm);
		for (unsigned i = 0; i < WL_TARGET_MAX; i++) { wl_dma_free(&wl_target[i]); }
		wl_dma_free(&wl14_regdb); wl_dma_free(&wl14_bdf); wl_dma_free(&wl14_m3);
		wl_dma_free(&wl_ce_arena);
		wl_dma_free(&wl_ipcr);
		wl_dma_free(&wl_contexts); wl_dma_free(&wl_vectors); wl_dma_free(&wl_amss); wl_dma_free(&wl_image);
		printf("FUXI WLAN: abort buffers freed\n");
	} else {
		printf("FUXI WLAN: abort buffers quarantined until reboot (containment or SMMU undo unproven)\n");
	}
}

static void
wl_finish(unsigned limit)
{
	/* Keep requires a complete, fault-free AMSS/M0 run. Earlier stops always
	 * contain, including fuxiwlan=22 with keep=1. Errors call abort directly. */
	if ((limit >= 17 || (wl_seq_flags && limit >= 8)) &&
	    !(limit == WL5_STEPS && wl_keep && wl_amss_reached && (!wl_qrtr || wl_w11_ok) && !fw_faults)) { wl5_abort(limit); }
}

static void
fw_dump(const char *tag, const uint32_t *r, unsigned words)
{
	for (unsigned i = 0; i < words; i += 8) {
		unsigned any = 0;
		for (unsigned j = 0; j < 8; j++) {
			any |= r[i + j];
		}
		if (any) {
			printf("FUXI WLAN: %s r[%3u] %08x %08x %08x %08x %08x %08x %08x %08x\n", tag, i,
			    r[i], r[i + 1], r[i + 2], r[i + 3], r[i + 4], r[i + 5], r[i + 6], r[i + 7]);
		}
	}
}

static void
fw_m3_summary(unsigned s)
{
	const uint32_t *r = wl3_report;
	switch (s) {
	case 1:
		printf("FUXI WLAN: cmd-db %#x %#x %#x %#x %#x %#x %#x %#x GPLL0 %#x\n",
		    r[20], r[21], r[22], r[23], r[24], r[25], r[26], r[27], r[28]);
		break;
	case 9:
		printf("FUXI WLAN: PHY ready after %u us pcs_status %#x\n", r[62], r[63]);
		break;
	case 11:
		printf("FUXI WLAN: link LTSSM %#x LNKCTL/STA %#x (gen%u x%u DLLLA %u) %u ms\n",
		    r[69], r[70], (r[70] >> 16) & 0xfu, (r[70] >> 20) & 0x3fu, (r[70] >> 29) & 1u, r[68]);
		break;
	case 12:
		printf("FUXI WLAN: endpoint id %#x class/rev %#x cmd/status %#x tries %u\n",
		    r[75], r[77], r[80], r[76]);
		break;
	case 13:
		printf("FUXI WLAN: BAR0 orig %#x/%#x sizing readback %#x/%#x\n", r[81], r[82], r[87], r[88]);
		break;
	}
}

/* P17 CLKCHECK / CLKON: PMK8550 gpio3 (32 kHz sleep clock) over SPMI arbiter
 * v7, EE 0 (map as fuxi_keys.c; layout as docs-lanes/buttons.md). CLKCHECK is
 * observer reads only. CLKON writes the five registers of wl17_clk_writes on
 * the pin's own channel, only if EE 0 owns the APID and the pin is not an
 * enabled function-2 output. Volatile PMIC state; failures never stop the run. */
extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
#define WL17_SP_CORE 0x0c400000u
#define WL17_SP_CNFG 0x0c42d000u
#define WL17_SP_OBS 0x0c440000u
#define WL17_SP_CHN 0x0c500000u

static uint32_t
wl17_sp_rd(vm_offset_t base, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static bool
wl17_sp_read(vm_offset_t obs, uint32_t apid, uint8_t reg, uint8_t *out)
{
	uint32_t off = 0x20u * apid;
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)(obs + off) = (1u << 27) | ((uint32_t)reg << 4); /* EXT_READL, 1 byte */
	__asm__ volatile("dsb osh" ::: "memory");
	for (unsigned i = 0; i < 1000; i++) {
		uint32_t st = wl17_sp_rd(obs, off + 8);
		if (st & 1u) {
			if (st & 0x0eu) { return false; }
			*out = (uint8_t)wl17_sp_rd(obs, off + 0x18);
			return true;
		}
		delay(1);
	}
	return false;
}

static bool
wl17_sp_write(vm_offset_t chn, uint8_t reg, uint8_t val)
{
	*(volatile uint32_t *)(chn + 0x10) = val; /* WDATA0 */
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)(chn + 0) = (uint32_t)reg << 4; /* EXT_WRITEL */
	__asm__ volatile("dsb osh" ::: "memory");
	for (unsigned i = 0; i < 1000; i++) {
		uint32_t st = wl17_sp_rd(chn, 8);
		if (st & 1u) { return (st & 0x0eu) == 0; }
		delay(1);
	}
	return false;
}

static bool
wl17_clk_read(vm_offset_t obs, uint32_t apid, struct wl17_gpio *g)
{
	return wl17_sp_read(obs, apid, 0x04, &g->type) && wl17_sp_read(obs, apid, 0x05, &g->sub) &&
	    wl17_sp_read(obs, apid, 0x40, &g->mode) && wl17_sp_read(obs, apid, 0x44, &g->func) &&
	    wl17_sp_read(obs, apid, 0x46, &g->en);
}

static void
wl17_clk_step(void)
{
	if (!(wl17_bits & (WL17_CLKCHECK | WL17_CLKON))) { return; }
	vm_offset_t core = ml_io_map(WL17_SP_CORE, 0x3000), cnfg = ml_io_map(WL17_SP_CNFG, 0x1000);
	vm_offset_t obs = ml_io_map(WL17_SP_OBS, 0x8000);
	if (!core || !cnfg || !obs) { printf("FUXI WLAN: P17 sleepclk SPMI mapping failed\n"); return; }
	uint32_t ver = wl17_sp_rd(core, 0), count = wl17_sp_rd(core, 4) & 0x7ffu;
	if (ver < 0x70000000u || ver >= 0x80000000u || !count || count > 1024) {
		printf("FUXI WLAN: P17 sleepclk unsupported SPMI arb=%#x APIDs=%u; skipped\n", ver, count); return;
	}
	uint32_t apid = ~0u; unsigned owner = 7;
	sweet_spmi_bus_lock();
	for (uint32_t i = 0; i < count; i++) {
		uint32_t map = wl17_sp_rd(core, 0x2000u + 4u*i);
		if (!map || ((map >> 8) & 0xfffu) != WL17_CLK_PPID) { continue; }
		unsigned o = wl17_sp_rd(cnfg, 4u*i) & 7u;
		if (apid == ~0u || o == 0) { apid = i; owner = o; }
	}
	struct wl17_gpio g = {0};
	bool ok = apid != ~0u && wl17_clk_read(obs, apid, &g);
	unsigned state = ok ? wl17_clk_state(&g, owner) : 0xfe;
	printf("FUXI WLAN: P17 sleepclk pmk8550 gpio3 ppid %#x apid %d owner ee%u read %u type %#x sub %#x mode %#x func %#x en %#x state %#x (0xff on, 0 off, 3 not ours, 4 layout, 0xfe unreadable)\n",
	    WL17_CLK_PPID, apid == ~0u ? -1 : (int)apid, owner, ok, g.type, g.sub, g.mode, g.func, g.en, state);
	if (ok && state == 0 && (wl17_bits & WL17_CLKON)) {
		vm_offset_t chn = ml_io_map(WL17_SP_CHN + 0x1000u*apid, 0x1000);
		bool wok = chn != 0;
		for (unsigned i = 0; wok && i < 5; i++) { wok = wl17_sp_write(chn, wl17_clk_writes[i].reg, wl17_clk_writes[i].val); }
		struct wl17_gpio a = {0};
		bool aok = wl17_clk_read(obs, apid, &a);
		printf("FUXI WLAN: P17 sleepclk enable writes ok %u after: read %u mode %#x func %#x en %#x state %#x\n",
		    wok, aok, a.mode, a.func, a.en, aok ? wl17_clk_state(&a, owner) : 0xfe);
	} else if (wl17_bits & WL17_CLKON) {
		printf("FUXI WLAN: P17 sleepclk enable skipped (state %#x)\n", state);
	}
	sweet_spmi_bus_unlock();
}

static void
fw_run(unsigned limit)
{
	uint32_t *r4 = wl4_report;
	r4[0] = WL4_MAGIC;
	r4[1] = limit;
	wl5_report[0] = 0x574c0006; wl5_report[1] = limit;
	for (unsigned s = 1; s <= limit; s++) {
		uint64_t t0 = mach_absolute_time(), ns = 0;
		unsigned faults0 = fw_faults;
		int e;
		if (s <= WL3_STEPS) {
			if (s == 1) { wl17_clk_step(); } /* P17 opt-in sleep clock, before any WLAN power writes */
			e = s == 1 ? wl_cnss_pdc() : 0; /* before any WLAN power writes */
			if (s == 1 && e) {
				printf("FUXI WLAN: M6n PDC blocked before WLAN power, consumed %u/13 reason %u; reboot to retry\n",
				    wl_pdc_acks, (unsigned)e);
			}
			if (!e && s == 1 && wl_variant_invalid && wl_seq_requested) { e = 0x6f; }
			if (!e && s == 1 && wl_variant_invalid && wl_rails_requested) { e = 0x6f; }
			if (!e) { e = wl_icc_step(s); } /* failure blocks link training / BHI */
			if (!e && s == 3) { e = wl_rails_on(1, "early before WLAN_EN high"); }
			if (!e) {
				wlan_m3(s); /* runs only step s: earlier ones are in wl3_done */
				e = (int)(wl3_report[3] & 0xffu);
				if (wl3_report[3]) { e = e ? e : 0xfe; }
			}
		} else {
			wl_mark(s);
			wl3_faults = 0;
			e = s <= WL4_STEPS ? wl4_step(s) : wl5_step(s);
			if (s > WL4_STEPS) {
				wl5_status(s);
				if (wl_smmu_fault(s) && !e) { e = 0xfc; }
			}
			wl3_report[124] += wl3_faults;
		}
		absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
		/* A fault in a plain (usb_rd/ts_wr) access also ends the run. */
		if (!e && fw_faults != faults0) {
			e = 0xfd;
		}
		r4[2] = s;
		uint32_t code = e ? (s << 8 | (unsigned)e) : 0x1000u | s;
		if (s <= WL4_STEPS) { r4[40 + s] = code; }
		wl5_report[2] = s; wl5_report[32 + s] = code;
		printf("FUXI WLAN: step %u %s code %#x (%llu ms, faults %u)\n", s, e ? "FAILED" : "ok",
		    code, ns / 1000000ull, fw_faults);
		if (!e && s <= WL3_STEPS) {
			if (s == 10) { wl_perst_done = mach_absolute_time(); }
			if (s == 11) { wl_link_done = mach_absolute_time(); }
			fw_m3_summary(s);
		}
		if (e) {
			r4[3] = s << 8 | (unsigned)e;
			wl5_report[3] = r4[3];
			if (s >= 17 || (wl_seq_flags && s >= 8)) { wl5_abort(s); }
			printf("FUXI WLAN: stopped at step %u reason %u ESR %#x FAR %#llx; reboot to retry\n",
			    s, e, fw_fault_esr, fw_fault_far);
			goto out;
		}
	}
	printf("FUXI WLAN: %u steps done\n", limit);
	/* W10 contains successful full runs as well as partial runs. Only explicit
	 * keep after verified AMSS/M0 retains the device and all DMA ownership. */
	wl_finish(limit);
out:
	/* Shared GCC vote registers (touch and GPU also read-modify-write
	 * 0x52008/0x52000 without a common lock): show the final values. */
	printf("FUXI WLAN: GCC votes 0x52000 %#x 0x52008 %#x gdsc-collapse 0x52020 %#x\n",
	    usb_rd(WL_GCC, 0x52000), usb_rd(WL_GCC, 0x52008), usb_rd(WL_GCC, 0x52020));
	fw_dump("m3", wl3_report, 128);
	fw_dump("m4", wl4_report, 64);
	if (limit >= 17) { fw_dump("m6", wl5_report, 128); }
}

static void
fw_thread(void *arg, wait_result_t result)
{
	unsigned limit = (unsigned)(uintptr_t)arg;
	unsigned mapped = 0;
	(void)result;
	for (unsigned i = 0; i < FW_NWIN; i++) {
		if ((!wl_cnss || wl_variant_invalid) &&
		    (fw_windows[i].pa == WL_AOSS_QMP || fw_windows[i].pa == WL_AOSS_IPCC)) { continue; }
		kern_return_t kr = fw_map_window(&fw_windows[i]);
		if (kr != KERN_SUCCESS) {
			printf("FUXI WLAN: map PA %#llx bytes %#llx failed %d\n",
			    fw_windows[i].pa, fw_windows[i].size, kr);
			goto out;
		}
		mapped++;
	}
	printf("FUXI WLAN: mapped %llu bytes in %u windows; running steps 1..%u\n",
	    fw_mapped_bytes, mapped, limit);
	fw_run(limit);
out:
	printf("FUXI WLAN: thread exit\n");
	(void)thread_terminate(current_thread());
	thread_block(THREAD_CONTINUE_NULL);
}

void
fuxi_wlan_init(void)
{
	static bool started;
	int arg = 0;
	bool requested;
	thread_t thread;
	if (started) {
		return;
	}
	started = true;
	requested = PE_parse_boot_argn("fuxiwlan", &arg, sizeof(arg)) && arg > 0;
#if defined(FUXI_SALAMI)
	/*
	 * K-W1, salami (OnePlus 11): compiled in, refused unless fuxiwlansalami=1
	 * is present as well (read at run time, so the driver stays in the
	 * binary). With it, step 1 expects the salami cmd-db (ldob15 0x40b00,
	 * never 0x40a00 = salami ldob12, the panel vddio), the firmware names
	 * default to the salami Android files, and rh_send_request refuses any
	 * OFF vote, any mode but 7 or other address (wl_salami_vote_ok). Both
	 * knobs, like every fuxiwlan* knob, may come from ADT /defaults (wl_arg).
	 */
	if (requested) {
		int salami = 0;
		if (!PE_parse_boot_argn("fuxiwlansalami", &salami, sizeof(salami)) || salami != 1) {
			printf("FUXI WLAN: refused on salami (fuxiwlan=%d needs fuxiwlansalami=1)\n", arg);
			requested = false;
		}
	}
#endif
	if (!requested) {
		return;
	}
	unsigned limit = (arg == 1 || arg > (int)WL5_STEPS) ? WL5_STEPS : (unsigned)arg;
	wl_iova_configure();
	if (wl_w20 && limit != WL5_STEPS) {
		wl_variant_invalid = true;
		printf("FUXI WLAN: W20 fuxiwlanqrtr=%u needs all %u steps (fuxiwlan=1 or 28), limit %u; refused\n", wl20_qrtr, WL5_STEPS, limit);
	}
#if defined(FUXI_SALAMI)
	/* Before the thread, any mapping or register access: cyclerails votes
	 * the shared rails OFF, and a selector that did not parse is refused
	 * outright. fuxiwlanrails=1/2 (K-W1b) re-votes the salami table ON only
	 * (voltage, enable 1, mode 7); its other values are invalid selectors. */
	{
		bool invalid = wl_variant_invalid || wl_probe == WL_PROBE_INVALID;
		if (invalid || (wl_seq_flags & WL_O_CYCLERAILS)) {
			printf("FUXI WLAN: refused on salami before any access (invalid selector %u, cyclerails %u)\n",
			    invalid, !!(wl_seq_flags & WL_O_CYCLERAILS));
			return;
		}
	}
	printf("FUXI WLAN: salami K-W1 gate open (fuxiwlansalami=1, fuxiwlanrails %u)\n", wl_rails_variant);
#endif
	printf("FUXI WLAN: enabled (fuxiwlan=%d), WCN7850 power-up + PCIe0 + MHI firmware, limit %u\n",
	    arg, limit);
	if (kernel_thread_start(fw_thread, (void *)(uintptr_t)limit, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("FUXI WLAN: kernel_thread_start failed\n");
	}
}
#endif /* FUXI */
