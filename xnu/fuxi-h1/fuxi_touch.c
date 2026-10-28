/*
 * FUXI touch: ST FTS (FTM5) on QUP1 SE4, SPI through QUP1 GPI DMA (polled).
 * Hardware facts and protocol: research/touch/hw-report.md; first proven in
 * the loader (shim/fuxi-probe/probe.c request 0x55). The loader votes the
 * RPMh rails (ldob12 vdd 1.8 V, ldom6 avdd 3.2 V) before the XNU handoff;
 * this driver does not talk to RPMh. SE4 is GSI-only (GENI_IF_DISABLE_RO),
 * so transfers use GPII 0 (TX channel 0, RX channel 1, one event ring), with
 * apps-smmu SMRs for streams 0xb6 (GPI) and 0xa3 (QUP1) routed to context
 * bank 4 (stage 1 off), as the loader does (research/touch/gpi-recipe.md,
 * ts-12). It takes the IC through its reset,
 * waits for the controller-ready event, enables multi-touch scanning and
 * polls the event FIFO while GPIO25 (IRQ, active low) is asserted.
 * No firmware upload: the FTS keeps its firmware in its own flash.
 *
 * Polling (0093): every 4 ms while a finger is down or the IRQ line was
 * seen recently, every 8 ms when idle and every 32 ms while the display is
 * off (AppleCLCD RequestPowerChange(0)); deadlines are absolute and use the
 * critical timer urgency, so no coalescing leeway is added. Touches are not
 * reported while the display is off (boot-arg fuxitouch_offsuppress=0 keeps
 * reporting). Boot-arg fuxitouch_poll_us sets the active period (default
 * 4000), fuxitouch_idle_us the idle one (default 8000).
 *
 * Tap-to-wake (H1, boot-arg fuxitaptowake=1, default off): a new contact
 * while the display is off asks bsd/dev/fuxi_keys.c for a power press,
 * which it sends only while backboardd's TouchDetectionMode is 0xfe. After
 * such a press the contacts of that off period stay hidden from iOS until
 * they lift. Without a wake press nothing changes.
 * Ported from HackiOS hackios-xnu-userland userland/src/inject.c (MIT):
 * the waking touch is kept from iOS. MIT notice in fuxi_key_state.h.
 *
 * Report batching/counters: adapted from UniXNU/NativeHidInput,
 * 75efe1c, e43c254, 22f2dd4 and ba03e5d (BSD-2-Clause-Patent).
 * Notice: iokit/Kernel/arm/hid/LICENSE.NativeHidInput. fuxitbatch=1
 * sends the complete ID-indexed contact set in one digitizer collection.
 *
 * salami (OnePlus 11, FUXI_SALAMI): Synaptics S3908, TouchComm v1 over I2C
 * at 0x4b on the same SE4 (design: salami-port/touch_s3908_design.md). The
 * backend follows the SE4 protocol (GENI_FW_REVISION_RO: 1 SPI is the FTS
 * above, 3 I2C is TouchComm) unless fuxitic forces one, and FTS is refused
 * on salami. I2C runs over the same GPI DMA when the SE is GSI-only and over
 * the SE FIFO otherwise (fuxitmode). The S3908 runs its application
 * firmware from flash: nothing is uploaded and no flash, bootloader,
 * config-commit or SPI-master command (0x11, 0x12, 0x13, 0x15, 0x16, 0x1f,
 * 0x28, 0x2b, 0x43; tcm_flash_cmd) is ever sent. XNU never votes RPMh
 * (VDD ldob4 belongs to the loader); it drives AVDD (GPIO10), the reset
 * pulse (GPIO24) and the I2C pin mux (GPIO44/45) only as fuxitpwr allows.
 * Every value the device has not confirmed is a boot-arg or an ADT /defaults
 * property of the same name (boot-arg first), and every fact the device
 * must answer is printed once as a "salami-touch:" line. On salami the
 * thread is off unless boot-arg fuxitouch=1: a default boot makes no SMMU,
 * GPII, QUP, GCC or TLMM access from this file (fuxi keeps default on).
 */
#if defined(FUXI)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <arm/caches_internal.h>
#include "fuxi_key_state.h"

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
extern kern_return_t thread_terminate(thread_t);
extern void sweet_hid_touch(int finger, int down, int x, int y);
extern int sweet_hid_touch_report(uint32_t mask, const int *x, const int *y);
extern ppnum_t pmap_find_phys(pmap_t pmap, addr64_t va);
extern kern_return_t kernel_thread_start_priority(thread_continue_t continuation,
    void *parameter, integer_t priority, thread_t *new_thread);
extern int sweet_fb_display_off;       /* iokit/Kernel/arm/SweetFramebuffer.cpp */
extern bool fuxi_keys_touch_wake(void); /* bsd/dev/fuxi_keys.c */
void fuxi_touch_init(void);

#define FT_SE_PA     0x00a90000u
#define FT_GPI_PA    0x00a10000u   /* GPII 0: ee base 0x9f0000 + 0x20000 */
#define FT_SMMU_PA   0x15000000u
#define FT_GCC_PA    0x00100000u
#define FT_TLMM_PA   0x0f100000u   /* upstream tlmm@f100000; 0x0f0xxxxx aborts */
#define FT_GPIO(n)   ((uint32_t)(n) * 0x1000u)
#define FT_RESET     24u
#define FT_IRQ       25u
#define FT_MAX       10u
#define FT_EV_READY  0x03u
#define FT_EV_ENTER  0x13u
#define FT_EV_MOTION 0x23u
#define FT_EV_LEAVE  0x33u
#define FT_TYPE_HOVER 0x05u        /* ftsSoftware.h TOUCH_TYPE_HOVER: not touching */
#define FT_PRIORITY  92            /* BASEPRI_PREEMPT */

/* salami S3908 (TouchComm v1 over I2C, design sections 3-5). */
#define FT_SDA       44u           /* qup1_se4_l0, function 1 */
#define FT_SCL       45u           /* qup1_se4_l1, function 1 */
#define TC_PINS      0x19180au     /* fuxitgpio 0: ATTN 25, reset 24, AVDD 10 */
#define TC_XFER_MAX  1020u         /* one I2C transfer (DMA buffers are 1 KiB) */
#define TC_MSG_MAX   1024u         /* one message: header, payload and padding */
#define TC_CFG_MAX   256u
#define TC_BLOB_MAX  256u
#define TC_ENG_FIFO  1u
#define TC_ENG_GPI   2u
#define TC_SPAN_X    5760u         /* DT panel-coords, chip span fallback */
#define TC_SPAN_Y    12864u
#define TC_OUT_W     1080u         /* boot framebuffer fallback */
#if defined(FUXI_SALAMI)
#define TC_OUT_H     2412u
#else
#define TC_OUT_H     2400u
#endif
#define TCM_MARKER   0xa5u
#define TCM_PAD      0x5au
#define TCM_IDLE     0x00u
#define TCM_OK       0x01u
#define TCM_BUSY     0x02u
#define TCM_CONT     0x03u
#define TCM_IDENTIFY 0x10u
#define TCM_TOUCH    0x11u
#define TCM_C_CONT   0x01u         /* CMD_CONTINUE_WRITE */
#define TCM_C_IDENTIFY 0x02u
#define TCM_C_RESET  0x04u
#define TCM_C_BOOT_INFO 0x10u
#define TCM_C_RUN_APP 0x14u
#define TCM_C_APP_INFO 0x20u
#define TCM_C_SET_DC 0x24u
#define TCM_C_GET_CFG 0x25u
#define TCM_C_SET_CFG 0x26u
#define TCM_C_SLEEP  0x2cu
#define TCM_C_WAKE   0x2du
#define TCM_MODE_APP 0x01u
#define TC_ERR_NONE  0u
#define TC_ERR_ANACK 1u
#define TC_ERR_DNACK 2u
#define TC_ERR_PROTO 3u
#define TC_ERR_ARB   4u
#define TC_ERR_TIME  5u
#define TC_ERR_OTHER 6u
#define TC_ERR_LEN   7u
/* GENI M_IRQ_STATUS (SE+0x610) */
#define FT_M_DONE    (1u << 0)
#define FT_M_CANCEL  (1u << 4)
#define FT_M_ABORT   (1u << 5)
#define FT_M_NACK    (1u << 10)
#define FT_M_PROTO   (1u << 12)
#define FT_M_ARB     (1u << 13)
#define FT_M_RX_WM   (1u << 26)
#define FT_M_RX_LAST (1u << 27)
#define FT_M_TX_WM   (1u << 30)
#define FT_M_ERRORS  (0x3eu | (1u << 9) | FT_M_NACK | FT_M_PROTO | FT_M_ARB | (3u << 24) | (3u << 28))
#define FT_M_EVENTS  (FT_M_DONE | FT_M_ERRORS | FT_M_RX_WM | FT_M_RX_LAST | FT_M_TX_WM)
/* geni_se_config_packing(8, 4): MSB first on the wire, FIFO bytes LSB first. */
#define FT_PACK0     0x0007f8feu
#define FT_PACK1     0x000ffefeu
#define TC_LOG(...)  printf("salami-touch: " __VA_ARGS__)
#define TC_FACT(...) do { if (fk.dbg & 1u) { TC_LOG(__VA_ARGS__); } } while (0)

static struct {
	vm_offset_t se, gcc, tlmm, gpi, smmu;
	vm_offset_t dma;
	uint64_t dma_pa;
	unsigned depth, level, wp[2], evrp;
	bool started;
	unsigned batch;              /* opt-in report-wide HID collection */
	bool retry;                  /* latest set not yet constructed by HID */
	uint32_t active;
	int x[FT_MAX], y[FT_MAX];
	bool ic_reset;                 /* controller-ready event while sensing */
	/* Per-contact statistics (first finger down -> last finger up). */
	uint64_t c_start, c_last, c_poll_sum, c_poll_max;
	unsigned c_reports, c_polls, contacts;
	unsigned c_multi, c_max, c_lifts;
	struct fuxi_wake_touch wake;   /* tap-to-wake contact filter */
} ft;
static int ft_chan;

/* TouchComm/I2C backend state (salami). */
static struct {
	bool on;                     /* TouchComm backend selected (SE4 protocol 3) */
	bool sensing;                /* bring-up done: 0x11 reports reach tcm_dispatch */
	bool deliver;                /* display on at the last tcm_poll */
	bool sleeping;               /* fuxitoff=1 deep sleep */
	bool cmd_reset;              /* an identify report arrived instead of an answer */
	bool c0_sent;                /* CONFIG0 sent since the last GPI init */
	bool motion;                 /* coalesced movement not reported yet */
	unsigned engine, proto, depth;
	unsigned pin_avdd, pin_rst, pin_attn;
	unsigned div, t_high, t_low, t_cycle, khz;
	uint32_t c0_d0, c0_d2;
	unsigned last_err;           /* TC_ERR_* of the last failed transfer */
	uint32_t ev_a, ev_b, gp_len;
	unsigned rdlen;              /* predictive first-read length */
	unsigned code, len;          /* last message: code, payload length */
	uint8_t msg[TC_MSG_MAX];
	uint8_t tmp[TC_XFER_MAX];
	uint8_t out[TC_XFER_MAX];
	unsigned mode, maxwr, id_ver, id_len, build;
	char part[17];
	unsigned app_status, cfg_max, max_x, max_y, sx, sy, ow, oh;
	uint8_t cfg[TC_CFG_MAX], fw_cfg[TC_CFG_MAX];
	unsigned cfg_len, fw_cfg_len;
	uint32_t px[FT_MAX], py[FT_MAX], pcls[FT_MAX], pseen;   /* tcm_parse output */
	uint32_t raw_prev;
	/* Facts and statistics. */
	bool pins_logged, app_logged, fwcfg_logged, idle_logged, notif_logged;
	bool touch_seen, quiet_logged, gpi_ok_logged, bus_logged;
	unsigned reads, attn_logs, pad_errs, bus_errs, parse_errs, resets, set_logs;
	unsigned contact_logs, marker_logs, rx_logs, dbg_reports;
	uint32_t codes_seen[8], rx_codes[8];
	uint64_t sense_abs, last_touch, cycle_at, dbg_win;
	uint64_t rd_bytes, rd_us, rd_count;
	bool cs_on;
	unsigned cs_n, cs_reports, cs_gap;
	uint32_t cs_rx, cs_ry, cs_cls;
	uint64_t cs_start, cs_last;
} tc;

/* Knobs: boot-arg, else ADT /defaults, else default (design 5.2). */
static struct {
	unsigned ic, mode, addr, clk, i2ct, rgo, c0, to, pinctl, gpio, pwr, pwrw, rst, rstw, run;
	unsigned rcfg, rdlen, drain, cls, palm, stale, dc, spanx, spany, xy, ow, oh;
	unsigned irq, sweep, recent, off, dbg;
	int lvl, cb, ox, oy;
	uint64_t cmd;
	uint8_t rcfgb[TC_BLOB_MAX], scr0[TC_BLOB_MAX], scr1[TC_BLOB_MAX];
	bool has_rcfgb, has_scr0, has_scr1;
} fk;

static uint32_t
ft_rd(vm_offset_t base, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static void
ft_wr(vm_offset_t base, uint32_t off, uint32_t v)
{
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)(base + off) = v;
	__asm__ volatile("dsb osh" ::: "memory");
}

static void
ft_sleep_until(uint64_t deadline)
{
	(void)assert_wait_deadline_with_leeway(&ft_chan, THREAD_UNINT,
	    TIMEOUT_URGENCY_SYS_CRITICAL, deadline, 0);
	thread_block(THREAD_CONTINUE_NULL);
}

static void
ft_sleep(unsigned ms)
{
	uint64_t deadline;
	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	(void)assert_wait_deadline(&ft_chan, THREAD_UNINT, deadline);
	thread_block(THREAD_CONTINUE_NULL);
}

/* TLMM CTL: pull [1:0] (1 down, 3 up), func [5:2], drive [8:6] = mA/2-1, OE [9]. */
static void
ft_gpio(unsigned n, unsigned func, unsigned pull, unsigned ma, int out)
{
	if (out >= 0) {
		ft_wr(ft.tlmm, FT_GPIO(n) + 4, out ? 2u : 0u);
	}
	ft_wr(ft.tlmm, FT_GPIO(n), (ft_rd(ft.tlmm, FT_GPIO(n)) & ~0x3ffu) | pull |
	    (func << 2) | ((ma / 2 - 1) << 6) | (out >= 0 ? 0x200u : 0u));
}

/* Raw TLMM CTL write including i2c_pull (bit 13), which ft_gpio cannot set. */
static void
ft_gpio_ctl(unsigned n, uint32_t ctl)
{
	ft_wr(ft.tlmm, FT_GPIO(n), (ft_rd(ft.tlmm, FT_GPIO(n)) & ~0x23ffu) | (ctl & 0x23ffu));
}

static bool
ft_on(uint32_t cbcr)
{
	return !(ft_rd(ft.gcc, cbcr) & 0x80000000u);
}

/* Before any SE register access: SE registers hang the bus without it. */
static int
ft_ahb(void)
{
	/* Vote wrapper1 core/core2x/m_ahb/s_ahb and SE4 only. */
	ft_wr(ft.gcc, 0x52008, ft_rd(ft.gcc, 0x52008) | (0xfu << 18) | (1u << 26));
	for (unsigned i = 0; i < 10000 && !ft_on(0x2327c); i++) {
		delay(1);
	}
	if (!ft_on(0x2327c)) {
		/* SE registers hang the bus without the wrapper AHB clock. */
		printf("FUXI TOUCH: wrapper1 AHB off (vote %#x)\n", ft_rd(ft.gcc, 0x52008));
		return EIO;
	}
	return 0;
}

/* SE clock: div is the SE divider (FTS 2); lvl < 0 searches the XO level (fuxitlvl). */
static int
ft_clock(unsigned div, int lvl)
{
	/* S4 is a DFS RCG under hardware control: the SE's level request brings
	 * it up. Level from XO, undivided (19.2 MHz); SE divider 2: 9.6 MHz. */
	unsigned level = 0;
	if (lvl >= 0) {
		level = (unsigned)lvl;
	} else if (ft_rd(ft.gcc, 0x18504) & 1u) {
		while (level < 8 && (ft_rd(ft.gcc, 0x1850c + 4 * level) & 0x71fu) > 1u) {
			level++;
		}
	} else if ((ft_rd(ft.gcc, 0x184f4) & 0x71fu) > 1u) {
		level = 8;
	}
	if (level == 8) {
		printf("FUXI TOUCH: no XO clock level\n");
		return EIO;
	}
	ft.level = level;
	ft_wr(ft.se, 0x07c, level);
	ft_wr(ft.se, 0x048, (div << 4) | 1u);
	for (unsigned i = 0; i < 10000 && !ft_on(0x184e4); i++) {
		delay(1);
	}
	if (!ft_on(0x184e4)) {
		printf("FUXI TOUCH: SE4 clock off (cbcr %#x)\n", ft_rd(ft.gcc, 0x184e4));
		return EIO;
	}
	return 0;
}

/* DMA page: event ring (4 KiB, 256 events), TX and RX rings (1 KiB, 64 TREs
 * each), then the TX and RX data buffers. Cacheable: clean before a doorbell,
 * invalidate before reading what the hardware wrote. */
#define FT_EV_OFF    0x0000u
#define FT_RING_OFF(k) (0x1000u + 0x400u * (k))
#define FT_TXB_OFF   0x1800u
#define FT_RXB_OFF   0x1c00u
#define FT_RING      64u
#define FT_EVRING    256u
#define GPI_CH(k)    (0x0000u + 0x80u * (k))
#define GPI_EV       0x1000u
#define GPI_CHDB(k)  (0x2000u + 8u * (k))
#define GPI_EVDB     0x2100u
#define GPI_GPII     0x3000u

static void
ft_dma_sync(uint32_t off, uint32_t len)
{
	flush_dcache64((addr64_t)(ft.dma_pa + off), len, TRUE);
}

static uint32_t
ft_state(uint32_t ctx)
{
	return (ft_rd(ft.gpi, ctx) >> 20) & 0xf;
}

static bool
ft_gpi_cmd(uint32_t reg, uint32_t v, uint32_t ctx, uint32_t want)
{
	ft_wr(ft.gpi, GPI_GPII + reg, v);
	for (unsigned i = 0; i < 50000; i++) {
		uint32_t st = ft_state(ctx);
		if (st == want) {
			ft_wr(ft.gpi, GPI_GPII + 0xa0, ft_rd(ft.gpi, GPI_GPII + 0x90));
			ft_wr(ft.gpi, GPI_GPII + 0xa4, ft_rd(ft.gpi, GPI_GPII + 0x94));
			return true;
		}
		if (st == 0xf) {
			break;
		}
		delay(1);
	}
	printf("FUXI TOUCH: GPI cmd %#x -> state %#x (want %u), error log %#x\n",
	    v, ft_state(ctx), want, ft_rd(ft.gpi, GPI_GPII + 0x200));
	return false;
}

/* Bus-masters without an SMR fault (sCR0.USFCFG) and the hypervisor turns an
 * S2CR bypass into fault: route GPI and QUP1 to s2cr (FTS: CB4 like DWC3).
 * Idempotent: an SMR already present keeps its S2CR. */
static int
ft_smmu(uint32_t s2cr)
{
	static const uint16_t sid[] = {0xb6, 0xa3};
	uint32_t nsmr = ft_rd(ft.smmu, 0x20) & 0xff;
	for (unsigned j = 0; j < 2; j++) {
		unsigned i, freei = ~0u;
		for (i = 0; i < nsmr; i++) {
			uint32_t smr = ft_rd(ft.smmu, 0x800 + 4 * i);
			if (!(smr & 0x80000000u)) {
				if (freei == ~0u) {
					freei = i;
				}
				continue;
			}
			if (((sid[j] ^ smr) & ~(smr >> 16) & 0x7fff) == 0) {
				break;
			}
		}
		if (i < nsmr) {
			continue;
		}
		if (freei == ~0u) {
			return ENOSPC;
		}
		ft_wr(ft.smmu, 0xc00 + 4 * freei, s2cr);
		ft_wr(ft.smmu, 0x800 + 4 * freei, 0x80000000u | sid[j]);
		printf("FUXI TOUCH: SMR %u = sid %#x -> S2CR %#x\n", freei, sid[j],
		    ft_rd(ft.smmu, 0xc00 + 4 * freei));
	}
	return 0;
}

static int
ft_gpi_init(void)
{
	uint32_t lo = (uint32_t)ft.dma_pa, hi = (uint32_t)(ft.dma_pa >> 32);
	tc.c0_sent = false;
	/* The loader leaves both channels started on its own rings: stop, reset, free. */
	for (unsigned k = 0; k < 2; k++) {
		if (ft_state(GPI_CH(k)) == 2) {
			(void)ft_gpi_cmd(0x08, (2u << 24) | k, GPI_CH(k), 3);
		}
		if (ft_state(GPI_CH(k))) {
			(void)ft_gpi_cmd(0x08, (9u << 24) | k, GPI_CH(k), 1);
			ft_wr(ft.gpi, GPI_GPII + 0x08, (10u << 24) | k);
			delay(1000);
		}
	}
	if (ft_state(GPI_EV)) {
		(void)ft_gpi_cmd(0x10, 9u << 24, GPI_EV, 1);
		ft_wr(ft.gpi, GPI_GPII + 0x10, 10u << 24);
		delay(1000);
	}
	bzero((void *)ft.dma, PAGE_SIZE);
	ft_dma_sync(0, PAGE_SIZE);
	ft_wr(ft.gpi, GPI_GPII + 0x88, 0x4f);
	ft_wr(ft.gpi, GPI_GPII + 0xb8, 1);
	ft_wr(ft.gpi, GPI_GPII + 0x98, 3);
	ft_wr(ft.gpi, GPI_GPII + 0x9c, 1);
	ft_wr(ft.gpi, GPI_GPII + 0x108, 1);
	ft_wr(ft.gpi, GPI_GPII + 0x120, 0xf);
	ft_wr(ft.gpi, GPI_GPII + 0x188, 0);
	ft_wr(ft.gpi, GPI_GPII + 0x18c, 0);
	ft_wr(ft.gpi, GPI_GPII + 0x400, 0);
	ft_wr(ft.gpi, GPI_GPII + 0x404, 0);
	ft_wr(ft.gpi, GPI_GPII + 0x180, 1);
	ft_wr(ft.gpi, GPI_GPII + 0x200, 0);
	if (!ft_gpi_cmd(0x10, 0, GPI_EV, 1)) {
		return EIO;
	}
	ft_wr(ft.gpi, GPI_EV + 0x00, 0x10010002u);
	ft_wr(ft.gpi, GPI_EV + 0x04, FT_EVRING * 16);
	ft_wr(ft.gpi, GPI_EV + 0x08, lo + FT_EV_OFF);
	ft_wr(ft.gpi, GPI_EV + 0x0c, hi);
	ft_wr(ft.gpi, GPI_EVDB + 4, hi);
	for (uint32_t o = 0x20; o <= 0x34; o += 4) {
		ft_wr(ft.gpi, GPI_EV + o, 0);
	}
	ft.evrp = 0;
	ft_wr(ft.gpi, GPI_EVDB, lo + FT_EV_OFF + FT_EVRING * 16 - 16);
	for (unsigned k = 0; k < 2; k++) {
		if (!ft_gpi_cmd(0x08, k, GPI_CH(k), 1)) {
			return EIO;
		}
		ft_wr(ft.gpi, GPI_CH(k) + 0x00, k ? 0x10000002u : 0x1000000au);
		ft_wr(ft.gpi, GPI_CH(k) + 0x04, FT_RING * 16);
		ft_wr(ft.gpi, GPI_CH(k) + 0x08, lo + FT_RING_OFF(k));
		ft_wr(ft.gpi, GPI_CH(k) + 0x0c, hi);
		ft_wr(ft.gpi, GPI_CHDB(k) + 4, hi);
		/* SCRATCH_0: pair, protocol (1 SPI, 3 I2C), SE 4. */
		ft_wr(ft.gpi, GPI_CH(k) + 0x60, (k ? 0u : 0x00010000u) | (tc.on ? 0x34u : 0x14u));
		ft_wr(ft.gpi, GPI_CH(k) + 0x64, 0);
		ft_wr(ft.gpi, GPI_CH(k) + 0x68, 0);
		ft_wr(ft.gpi, GPI_CH(k) + 0x6c, 0);
		ft_wr(ft.gpi, GPI_CH(k) + 0x5c, 1);
		ft.wp[k] = 0;
	}
	for (unsigned k = 0; k < 2; k++) {
		if (!ft_gpi_cmd(0x08, (1u << 24) | k, GPI_CH(k), 2)) {
			return EIO;
		}
	}
	return 0;
}

/* geni_se_select_mode(GENI_GPI_DMA); the protocol check is in ft_detect. */
static int
ft_se_gsi(void)
{
	/* The CONFIG0 TRE sets mode, word size, packing and clock. */
	ft_wr(ft.se, 0xe18, 0);
	ft_wr(ft.se, 0x618, 0xffffffffu);
	ft_wr(ft.se, 0x648, 0xffffffffu);
	ft_wr(ft.se, 0xc44, 0xffffffffu);
	ft_wr(ft.se, 0xd44, 0xffffffffu);
	ft_wr(ft.se, 0x028, ft_rd(ft.se, 0x028) | 0x7f);
	ft_wr(ft.se, 0xe30, ft_rd(ft.se, 0xe30) | 0xf);
	ft_wr(ft.se, 0x024, 0x7f);
	ft_wr(ft.se, 0x020, 1);
	ft_wr(ft.se, 0xe1c, 0);
	ft_wr(ft.se, 0x614, ft_rd(ft.se, 0x614) & ~((1u << 0) | (1u << 26) | (1u << 27) | (1u << 30)));
	ft_wr(ft.se, 0x644, ft_rd(ft.se, 0x644) & ~1u);
	ft_wr(ft.se, 0x258, 1);
	ft_wr(ft.se, 0xe18, ft_rd(ft.se, 0xe18) | 0xf);
	return 0;
}

static void
ft_tre(unsigned k, uint32_t d0, uint32_t d1, uint32_t d2, uint32_t d3)
{
	uint32_t off = FT_RING_OFF(k) + 16 * ft.wp[k];
	volatile uint32_t *t = (volatile uint32_t *)(ft.dma + off);
	t[0] = d0;
	t[1] = d1;
	t[2] = d2;
	t[3] = d3;
	ft_dma_sync(off, 16);
	ft.wp[k] = (ft.wp[k] + 1) % FT_RING;
}

static void
ft_ring_db(unsigned k)
{
	__asm__ volatile("dsb sy" ::: "memory");
	ft_wr(ft.gpi, GPI_CHDB(k), (uint32_t)ft.dma_pa + FT_RING_OFF(k) + 16 * ft.wp[k]);
}

/*
 * I2C GSI event (MI2C:826-887, MGPI:155-201): true for an error, whose kind
 * is kept in tc.last_err. d3 = status[15:0] type[23:16] chid[31:24]; a
 * transfer event carries DM_I2C_CB_ERR (0x340) in its status, a QUP
 * notification (type 0x31) carries M_IRQ_STATUS in d0.
 */
static bool
ft_i2c_event(uint32_t d0, uint32_t d2, uint32_t d3)
{
	unsigned code = d2 >> 24, type = (d3 >> 16) & 0xffu, ch = d3 >> 24;
	uint32_t irq = 0;
	if (type == 0x31u) {
		irq = d0;
		if (!(irq & (0x3eu | FT_M_NACK | FT_M_PROTO | FT_M_ARB))) {
			if (!tc.notif_logged) {
				tc.notif_logged = true;
				TC_FACT("gpi notify without error bits d0=%#x d3=%#x\n", d0, d3);
			}
			return false;
		}
	} else if (type == 0x22u && (code == 16u || (d3 & 0x340u))) {
		irq = ((d3 & 0x40u) ? FT_M_NACK : 0u) | ((d3 & 0x100u) ? FT_M_PROTO : 0u) |
		    ((d3 & 0x200u) ? FT_M_ARB : 0u);
	} else if (type != 0xffu && ch < 2u) {
		return false;
	}
	tc.ev_a = d2;
	tc.ev_b = d3;
	tc.gp_len = ft_rd(ft.se, 0x910);
	tc.last_err = (irq & FT_M_NACK) ? (tc.gp_len ? TC_ERR_DNACK : TC_ERR_ANACK) :
	    (irq & FT_M_PROTO) ? TC_ERR_PROTO : (irq & FT_M_ARB) ? TC_ERR_ARB : TC_ERR_OTHER;
	return true;
}

/* Drain events until an end-of-transfer (code 2) from chid arrives. */
static int
ft_gpi_wait(unsigned chid, uint32_t *len, unsigned timeout_us)
{
	for (unsigned us = 0; us < timeout_us; us++) {
		uint32_t hw = ft_rd(ft.gpi, GPI_EV + 0x10);
		bool found = false, bad = false;
		while (hw != (uint32_t)ft.dma_pa + FT_EV_OFF + 16 * ft.evrp) {
			uint32_t off = FT_EV_OFF + 16 * ft.evrp;
			ft_dma_sync(off, 16);
			volatile uint32_t *e = (volatile uint32_t *)(ft.dma + off);
			uint32_t d2 = e[2], d3 = e[3];
			ft.evrp = (ft.evrp + 1) % FT_EVRING;
			unsigned code = d2 >> 24, type = (d3 >> 16) & 0xff, ch = d3 >> 24;
			if (tc.on) {
				if (ft_i2c_event(e[0], d2, d3)) {
					bad = true;
				} else if (type == 0x22 && ch == chid && code == 2) {
					found = true;
					*len = d2 & 0xffffff;
				}
			} else if (code == 16 || type == 0xff || ch >= 2) {
				bad = true;
				printf("FUXI TOUCH: GPI event %#x %#x\n", d2, d3);
			} else if (type == 0x22 && ch == chid && code == 2) {
				found = true;
				*len = d2 & 0xffffff;
			}
		}
		ft_wr(ft.gpi, GPI_EVDB, (uint32_t)ft.dma_pa + FT_EV_OFF +
		    16 * ((ft.evrp + FT_EVRING - 1) % FT_EVRING));
		ft_wr(ft.gpi, GPI_GPII + 0xc0, 1);
		if (bad) {
			return EIO;
		}
		if (found) {
			return 0;
		}
		delay(1);
	}
	return ETIMEDOUT;
}

/* CONFIG0: FTS SPI as before; I2C per fuxitc0 (every transfer, or once per GPI init). */
static void
ft_c0(void)
{
	if (!tc.on) {
		ft_tre(0, 0x03000004u, 0, (ft.level << 16) | 2u, 0x00220001u);
	} else if (!fk.c0 || !tc.c0_sent) {
		ft_tre(0, tc.c0_d0, 0, tc.c0_d2, 0x00220001u);
		tc.c0_sent = true;
	}
}

static int
ft_tx(const uint8_t *tx, unsigned nt, bool frag)
{
	uint32_t len = 0;
	int err;
	memcpy((void *)(ft.dma + FT_TXB_OFF), tx, nt);
	ft_dma_sync(FT_TXB_OFF, nt);
	ft_c0();
	ft_tre(0, (frag ? (1u << 26) : 0) | 1u, 0, 0, 0x00200001u);
	ft_tre(0, (uint32_t)ft.dma_pa + FT_TXB_OFF, (uint32_t)(ft.dma_pa >> 32), nt, 0x00100200u);
	ft_ring_db(0);
	if ((err = ft_gpi_wait(0, &len, 5000))) {
		return err;
	}
	return len == nt ? 0 : EIO;
}

/* FTS read: opcode + address (CS held), then one dummy byte and n data bytes. */
static int
ft_read(const uint8_t *cmd, unsigned nc, uint8_t *out, unsigned n)
{
	uint32_t len = 0;
	int err;
	if (nc > 8 || n > 8 * 32) {
		return EINVAL;
	}
	if ((err = ft_tx(cmd, nc, true))) {
		return err;
	}
	ft_dma_sync(FT_RXB_OFF, n + 1);
	ft_tre(1, (uint32_t)ft.dma_pa + FT_RXB_OFF, (uint32_t)(ft.dma_pa >> 32), n + 1, 0x00100200u);
	ft_tre(0, 2u, 0, n + 1, 0x00200900u);
	ft_ring_db(1);
	ft_ring_db(0);
	if ((err = ft_gpi_wait(1, &len, 5000))) {
		return err;
	}
	ft_dma_sync(FT_RXB_OFF, n + 1);
	memcpy(out, (const void *)(ft.dma + FT_RXB_OFF + 1), n);
	return 0;
}

static uint64_t
ft_us(uint64_t abs)
{
	uint64_t ns;
	absolutetime_to_nanoseconds(abs, &ns);
	return ns / 1000;
}

/* One line per contact: how often iOS got a report and what a FIFO read costs. */
static void
ft_contact_end(void)
{
	uint64_t dur = ft_us(ft.c_last - ft.c_start);
	ft.contacts++;
	printf("FUXI TOUCH: contact %u: %u reports in %llu ms (%llu us apart), %u polls, "
	    "FIFO read avg %llu us max %llu us, multi %u max %u lifts %u\n",
	    ft.contacts, ft.c_reports, dur / 1000,
	    ft.c_reports > 1 ? dur / (ft.c_reports - 1) : 0ULL, ft.c_polls,
	    ft.c_polls ? ft_us(ft.c_poll_sum) / ft.c_polls : 0ULL, ft_us(ft.c_poll_max),
	    ft.c_multi, ft.c_max, ft.c_lifts);
}

extern void fuxi_perf_input(void);
static void
ft_report(uint32_t mask)
{
	fuxi_perf_input();
	uint64_t now = mach_absolute_time();
	if (mask && !ft.active) {
		ft.c_start = now;
		ft.c_reports = ft.c_polls = 0;
		ft.c_poll_sum = ft.c_poll_max = 0;
		ft.c_multi = ft.c_max = ft.c_lifts = 0;
	}
	unsigned down_count = 0;
	for (unsigned id = 0; id < FT_MAX; id++) {
		uint32_t bit = 1u << id;
		if (mask & bit) {
			down_count++;
			if (!ft.batch) {
				sweet_hid_touch((int)id, 1, ft.x[id], ft.y[id]);
			}
		} else if (ft.active & bit) {
			ft.c_lifts++;
			if (!ft.batch) {
				sweet_hid_touch((int)id, 0, ft.x[id], ft.y[id]);
			}
		}
	}
	if (ft.batch && (mask || ft.active)) {
		ft.retry = !sweet_hid_touch_report(mask, ft.x, ft.y);
	}
	ft.c_multi += down_count >= 2;
	ft.c_max = down_count > ft.c_max ? down_count : ft.c_max;
	if (mask || ft.active) {
		ft.c_reports++;
		ft.c_last = now;
	}
	bool ended = ft.active && !mask;
	ft.active = mask;
	if (ended) {
		ft_contact_end();
	}
}

/* Retry the latest set, including final all-up, without counting a new sample. */
static void
ft_retry_report(void)
{
	if (ft.retry) {
		ft.retry = !sweet_hid_touch_report(ft.active, ft.x, ft.y);
	}
}

/*
 * Drain the FIFO in two reads (as fts_event_handler): one event, whose
 * byte 7 holds the number of events still queued, then all of those at
 * once. With deliver false (display off) the events are consumed and
 * dropped.
 */
static int
ft_poll(bool deliver)
{
	static const uint8_t fifo[] = {0x87};
	uint8_t e[8 * 32];
	uint32_t mask = ft.active;
	bool changed = false;
	int err = ft_read(fifo, 1, e, 8);
	if (err) {
		return err;
	}
	if (e[0] == 0) {
		return 0;
	}
	unsigned n = 1 + (e[7] & 0x1f);
	if (n > 32) {
		n = 32;
	}
	if (n > 1 && (err = ft_read(fifo, 1, e + 8, 8 * (n - 1)))) {
		return err;
	}
	for (unsigned i = 0; i < n; i++) {
		const uint8_t *ev = e + 8 * i;
		if (ev[0] == 0) {
			break;
		}
		if (ev[0] == FT_EV_READY) {
			ft.ic_reset = true;
			continue;
		}
		unsigned id = ev[1] >> 4, type = ev[1] & 0xf;
		if ((ev[0] != FT_EV_ENTER && ev[0] != FT_EV_MOTION && ev[0] != FT_EV_LEAVE) ||
		    id >= FT_MAX || (type == 0 && ev[0] != FT_EV_LEAVE)) {
			continue;
		}
		bool touching = ev[0] != FT_EV_LEAVE && type != FT_TYPE_HOVER, fresh;
		if (fuxi_wake_touch_event(&ft.wake, !deliver, id, touching, &fresh)) {
			if (fresh && fuxi_keys_touch_wake()) {
				fuxi_wake_touch_woke(&ft.wake);
			}
			if (deliver) {
				continue;      /* a contact that woke the display: not for iOS */
			}
		}
		/* Super-resolution coordinates, 10x the 1080x2400 panel. */
		int x = ((ev[3] << 8) | ev[2]) / 10, y = ((ev[5] << 8) | ev[4]) / 10;
		ft.x[id] = x > 1079 ? 1079 : x;
		ft.y[id] = y > 2399 ? 2399 : y;
		if (ev[0] == FT_EV_LEAVE || type == FT_TYPE_HOVER) {
			mask &= ~(1u << id);
		} else {
			mask |= 1u << id;
		}
		changed = true;
	}
	if (changed && deliver) {
		ft_report(mask);
	}
	return 0;
}

/*
 * Knob lookup (design 5.1): boot-arg (PE_parse_boot_argn, 64-bit), else the
 * ADT /defaults property of the same name (4-byte LE, sign-extended, or
 * 8-byte), else def. Out of range (zero allows 0 as well) logs a reject and
 * keeps def; a value other than def is logged once.
 */
static int64_t
ft_knob(const char *name, int64_t def, int64_t lo, int64_t hi, bool zero)
{
	int64_t v = 0;
	uint32_t v32 = 0;
	uint64_t v64 = 0;
	const char *src = "bootarg";
	if (!PE_parse_boot_argn(name, &v, sizeof(v))) {
		src = "adt";
		if (PE_get_default(name, &v32, sizeof(v32))) {
			v = (int64_t)(int32_t)v32;
		} else if (PE_get_default(name, &v64, sizeof(v64))) {
			v = (int64_t)v64;
		} else {
			return def;
		}
	}
	if ((v < lo || v > hi) && !(zero && v == 0)) {
		TC_LOG("cfg reject %s=%lld (%s)\n", name, v, src);
		return def;
	}
	if (v != def) {
		TC_LOG("cfg %s=%lld (%s)\n", name, v, src);
	}
	return v;
}

/* ADT-only byte blob (scripts, report config), zero-padded to max. */
static bool
ft_knob_blob(const char *name, uint8_t *buf, unsigned max)
{
	bzero(buf, max);
	if (!PE_get_default(name, buf, max)) {
		return false;
	}
	TC_LOG("cfg %s blob (adt)\n", name);
	return true;
}

static void
ft_config(void)
{
	uint32_t pins;
	fk.ic = (unsigned)ft_knob("fuxitic", 0, 0, 2, false);
	fk.mode = (unsigned)ft_knob("fuxitmode", 0, 0, 2, false);
	fk.addr = (unsigned)ft_knob("fuxitaddr", 0x4b, 0x08, 0x77, false);
	fk.clk = (unsigned)ft_knob("fuxitclk", 400, 100, 1000, false);
	if (fk.clk != 100 && fk.clk != 400 && fk.clk != 1000) {
		TC_LOG("cfg reject fuxitclk=%u (100, 400 or 1000)\n", fk.clk);
		fk.clk = 400;
	}
	fk.i2ct = (unsigned)ft_knob("fuxiti2ct", 0, 0x01010101, 0x0fffffff, true);
	if (fk.i2ct && (!(fk.i2ct & 0xffu) || !(fk.i2ct & 0xff00u) || !(fk.i2ct & 0xff0000u))) {
		TC_LOG("cfg reject fuxiti2ct=%#x (a zero field)\n", fk.i2ct);
		fk.i2ct = 0;
	}
	fk.lvl = (int)ft_knob("fuxitlvl", -1, -1, 7, false);
	fk.rgo = (unsigned)ft_knob("fuxitrgo", 0, 0, 1, false);
	fk.c0 = (unsigned)ft_knob("fuxitc0", 0, 0, 1, false);
	fk.to = (unsigned)ft_knob("fuxitto", 3000, 500, 100000, false);
	fk.cb = (int)ft_knob("fuxitcb", -1, -1, 255, false);
	fk.pinctl = (unsigned)ft_knob("fuxitpinctl", 0x2007, 0, 0x23ff, false);
	fk.gpio = (unsigned)ft_knob("fuxitgpio", 0, 0, 0x2f2f2f, true);
	if ((fk.gpio & 0xffu) > 47u || ((fk.gpio >> 8) & 0xffu) > 47u || ((fk.gpio >> 16) & 0xffu) > 47u) {
		TC_LOG("cfg reject fuxitgpio=%#x (pins 0..47)\n", fk.gpio);
		fk.gpio = 0;
	}
	fk.pwr = (unsigned)ft_knob("fuxitpwr", 0x7, 0, 0xf, false);
	fk.pwrw = (unsigned)ft_knob("fuxitpwrw", 10, 0, 200, false);
	fk.rst = (unsigned)ft_knob("fuxitrst", 10, 1, 200, false);
	fk.rstw = (unsigned)ft_knob("fuxitrstw", 300, 20, 3000, false);
	fk.run = (unsigned)ft_knob("fuxitrun", 2, 0, 5, false);
	fk.rcfg = (unsigned)ft_knob("fuxitrcfg", 2, 0, 3, false);
	fk.rdlen = (unsigned)ft_knob("fuxitrdlen", 0, 9, TC_XFER_MAX, true);
	fk.drain = (unsigned)ft_knob("fuxitdrain", 4, 1, 16, false);
	fk.cls = (unsigned)ft_knob("fuxitcls", 0x12e, 0, 0x3ff, false);
	fk.palm = (unsigned)ft_knob("fuxitpalm", 0, 0, 1, false);
	fk.stale = (unsigned)ft_knob("fuxitstale", 0, 50, 5000, true);
	fk.dc = (unsigned)ft_knob("fuxitdc", 0, 0x10000, 0xffffff, true);
	fk.cmd = (uint64_t)ft_knob("fuxitcmd", 0, -0x7fffffffffffffffLL - 1, 0x7fffffffffffffffLL, false);
	if (fk.cmd && (!(fk.cmd & 0xffu) || ((fk.cmd >> 8) & 0xffu) > 6u)) {
		TC_LOG("cfg reject fuxitcmd=%#llx (bytes cmd, n <= 6, payload)\n", fk.cmd);
		fk.cmd = 0;
	}
	fk.spanx = (unsigned)ft_knob("fuxitspanx", 0, 16, 65535, true);
	fk.spany = (unsigned)ft_knob("fuxitspany", 0, 16, 65535, true);
	fk.xy = (unsigned)ft_knob("fuxitxy", 0, 0, 7, false);
	fk.ow = (unsigned)ft_knob("fuxitow", 0, 320, 4096, true);
	fk.oh = (unsigned)ft_knob("fuxitoh", 0, 320, 4096, true);
	fk.ox = (int)ft_knob("fuxitox", 0, -200, 200, false);
	fk.oy = (int)ft_knob("fuxitoy", 0, -200, 200, false);
	fk.irq = (unsigned)ft_knob("fuxitirq", 1, 0, 2, false);
	fk.sweep = (unsigned)ft_knob("fuxitsweep", 32, 8, 1000, false);
	fk.recent = (unsigned)ft_knob("fuxitrecent", 150, 0, 2000, false);
	fk.off = (unsigned)ft_knob("fuxitoff", 0, 0, 1, false);
	fk.dbg = (unsigned)ft_knob("fuxitdbg", 1, 0, 0xff, false);
	fk.has_rcfgb = ft_knob_blob("fuxitrcfgb", fk.rcfgb, TC_BLOB_MAX);
	fk.has_scr0 = ft_knob_blob("fuxitscr0", fk.scr0, TC_BLOB_MAX);
	fk.has_scr1 = ft_knob_blob("fuxitscr1", fk.scr1, TC_BLOB_MAX);
	pins = fk.gpio ? fk.gpio : TC_PINS;
	tc.pin_avdd = pins & 0xffu;
	tc.pin_rst = (pins >> 8) & 0xffu;
	tc.pin_attn = (pins >> 16) & 0xffu;
}

/*
 * SE4 identity, read once the SE clock is on: protocol (1 SPI, 3 I2C),
 * GSI-only (GENI_IF_DISABLE_RO bit 0) and FIFO depth. Picks the backend
 * (fuxitic) and, for TouchComm, the engine (fuxitmode). No SE write.
 */
static int
ft_detect(void)
{
	uint32_t fw = ft_rd(ft.se, 0x068), ifd = ft_rd(ft.se, 0x064), st = ft_rd(ft.se, 0x040);
	uint32_t hw0 = ft_rd(ft.se, 0xe24), hw2 = ft_rd(ft.se, 0xe2c);
	unsigned want;
	tc.proto = (fw >> 8) & 0xffu;
	tc.depth = (hw0 >> 16) & 0xffu;   /* SE_HW_PARAM_0 [23:16], Linux GENMASK(23, 16) */
	TC_FACT("se4 fw=%#x proto=%u if_disable=%#x status=%#x hwp0=%#x fifo_depth=%u hwp2=%#x rtl=%u\n",
	    fw, tc.proto, ifd, st, hw0, tc.depth, hw2, (hw2 >> 15) & 1u);
	want = fk.ic ? fk.ic : (tc.proto == 1u ? 1u : (tc.proto == 3u ? 2u : 0u));
	if (!want || (want == 1u && tc.proto != 1u) || (want == 2u && tc.proto != 3u) || (st & 1u)) {
		printf("FUXI TOUCH: SE4 fw %#x status %#x: no touch backend for protocol %u (fuxitic=%u)\n",
		    fw, st, tc.proto, fk.ic);
		return ENODEV;
	}
	tc.on = want == 2u;
	if (!tc.on) {
		return 0;
	}
	tc.engine = fk.mode ? fk.mode : ((ifd & 1u) ? TC_ENG_GPI : TC_ENG_FIFO);
	if (tc.engine == TC_ENG_FIFO && (ifd & 1u)) {
		TC_LOG("fuxitmode=1 (FIFO) but SE4 is GSI-only (if_disable %#x): touch stopped\n", ifd);
		return ENODEV;
	}
	TC_LOG("backend TouchComm/I2C, engine %s (%s)\n", tc.engine == TC_ENG_GPI ? "GPI" : "FIFO",
	    fk.mode ? "fuxitmode" : "auto");
	return 0;
}

/* S2CR for new GPI/QUP SMRs: fuxitcb, else the DWC3 (SID 0x40) S2CR, else CB4. */
static uint32_t
ft_smmu_cb(void)
{
	uint32_t nsmr = ft_rd(ft.smmu, 0x20) & 0xffu, dwc3 = ~0u, used;
	int gpi = -1, qup = -1;
	for (uint32_t i = 0; i < nsmr; i++) {
		uint32_t smr = ft_rd(ft.smmu, 0x800 + 4 * i), m = ~(smr >> 16) & 0x7fffu;
		if (!(smr & 0x80000000u)) {
			continue;
		}
		if (dwc3 == ~0u && ((0x40u ^ smr) & m) == 0) {
			dwc3 = ft_rd(ft.smmu, 0xc00 + 4 * i);
		}
		if (gpi < 0 && ((0xb6u ^ smr) & m) == 0) {
			gpi = (int)i;
		}
		if (qup < 0 && ((0xa3u ^ smr) & m) == 0) {
			qup = (int)i;
		}
	}
	used = fk.cb >= 0 ? (uint32_t)fk.cb : (dwc3 != ~0u ? dwc3 : 4u);
	TC_FACT("smmu nsmr=%u dwc3_s2cr=%#x gpi_smr=%d qup_smr=%d s2cr_used=%#x\n",
	    nsmr, dwc3, gpi, qup, used);
	return used;
}

/* fuxitclk / fuxiti2ct -> SE divider and SCL counts (MI2C:250-254, 19.2 MHz). */
static void
ft_i2c_timing(void)
{
	unsigned div = 2, th = 7, tl = 10, cyc = 24;     /* 400 kHz */
	if (fk.clk == 100) {
		div = 7;
		th = 10;
		tl = 11;
		cyc = 26;
	} else if (fk.clk == 1000) {
		div = 1;
		th = 3;
		tl = 9;
		cyc = 18;
	}
	if (fk.i2ct) {
		div = (fk.i2ct >> 24) & 0xffu;
		th = (fk.i2ct >> 16) & 0xffu;
		tl = (fk.i2ct >> 8) & 0xffu;
		cyc = fk.i2ct & 0xffu;
	}
	tc.div = div;
	tc.t_high = th;
	tc.t_low = tl;
	tc.t_cycle = cyc;
	tc.khz = 19200u / (div * cyc);
	if (!tc.khz) {
		tc.khz = 1;
	}
	tc.c0_d0 = (3u << 24) | (cyc << 16) | (th << 8) | tl;
}

/* Busy-wait budget of one transfer: fuxitto plus four times the wire time. */
static unsigned
ft_i2c_budget(unsigned n)
{
	return fk.to + 4u * ((n + 1u) * 9u * 1000u / tc.khz);
}

/* FIFO-mode SE init (geni_se_init + FIFO select), from bsd/dev/sweet_touch.c. */
static int
ft_se_fifo(void)
{
	if (tc.depth < 2u) {
		TC_LOG("fifo depth %u: FIFO engine refused\n", tc.depth);
		return ENODEV;
	}
	ft_wr(ft.se, 0xe18, 0);              /* no GSI events */
	ft_wr(ft.se, 0xe1c, 0);              /* no IRQ routing: polled */
	ft_wr(ft.se, 0x644, 0);
	ft_wr(ft.se, 0x618, 0xffffffffu);
	ft_wr(ft.se, 0x648, 0xffffffffu);
	ft_wr(ft.se, 0xc44, 0xffffffffu);
	ft_wr(ft.se, 0xd44, 0xffffffffu);
	ft_wr(ft.se, 0x258, 0);              /* GENI_DMA_MODE_EN off: FIFO */
	ft_wr(ft.se, 0x028, ft_rd(ft.se, 0x028) | 0x7fu);
	ft_wr(ft.se, 0xe30, ft_rd(ft.se, 0xe30) | 0xfu);
	ft_wr(ft.se, 0x024, 0x7f);
	ft_wr(ft.se, 0x020, 1);
	ft_wr(ft.se, 0x07c, ft.level);
	ft_wr(ft.se, 0x048, (tc.div << 4) | 1u);
	ft_wr(ft.se, 0x278, (tc.t_high << 20) | (tc.t_low << 10) | tc.t_cycle);
	ft_wr(ft.se, 0x254, 0);
	ft_wr(ft.se, 0x260, FT_PACK0);
	ft_wr(ft.se, 0x264, FT_PACK1);
	ft_wr(ft.se, 0x284, FT_PACK0);
	ft_wr(ft.se, 0x288, FT_PACK1);
	ft_wr(ft.se, 0x80c, 0);
	ft_wr(ft.se, 0x810, tc.depth - 1u);
	ft_wr(ft.se, 0x814, tc.depth);
	ft_wr(ft.se, 0x614, ft_rd(ft.se, 0x614) | FT_M_EVENTS);
	if (ft_rd(ft.se, 0x260) != FT_PACK0 || ft_rd(ft.se, 0x264) != FT_PACK1 ||
	    ft_rd(ft.se, 0x284) != FT_PACK0 || ft_rd(ft.se, 0x288) != FT_PACK1) {
		TC_LOG("fifo packing readback %#x/%#x %#x/%#x: FIFO engine refused\n",
		    ft_rd(ft.se, 0x260), ft_rd(ft.se, 0x264), ft_rd(ft.se, 0x284), ft_rd(ft.se, 0x288));
		return EIO;
	}
	for (unsigned i = 0; i < 256u && (ft_rd(ft.se, 0x804) & 0x1ffffffu); i++) {
		(void)ft_rd(ft.se, 0x780);   /* stale RX words: up to the 8-bit FIFO depth */
	}
	return 0;
}

/* Pop the RX FIFO words now present (byte p of a word sits at bits 8p). */
static int
ft_fifo_pop(uint8_t *r, unsigned n, unsigned *got)
{
	uint32_t cnt = ft_rd(ft.se, 0x804) & 0x1ffffffu;
	if (cnt > (n - *got + 3u) / 4u) {
		return EIO;
	}
	while (cnt--) {
		uint32_t word = ft_rd(ft.se, 0x780);
		for (unsigned j = 0; j < 4u && *got < n; j++) {
			r[(*got)++] = (uint8_t)(word >> (8u * j));
		}
	}
	return 0;
}

/* Cancel, then abort, the current M command (LSE:171-186; MI2C:714-747). */
static void
ft_fifo_stop(void)
{
	bool done = false;
	ft_wr(ft.se, 0x604, 4u);
	for (unsigned i = 0; i < 1000u && !done; i++) {
		done = (ft_rd(ft.se, 0x610) & FT_M_CANCEL) != 0;
		if (!done) {
			delay(1);
		}
	}
	if (!done) {
		ft_wr(ft.se, 0x604, 2u);
		for (unsigned i = 0; i < 1000u && !done; i++) {
			done = (ft_rd(ft.se, 0x610) & FT_M_ABORT) != 0;
			if (!done) {
				delay(1);
			}
		}
	}
	for (unsigned i = 0; i < 256u && (ft_rd(ft.se, 0x804) & 0x1ffffffu); i++) {
		(void)ft_rd(ft.se, 0x780);
	}
	ft_wr(ft.se, 0x80c, 0);
	ft_wr(ft.se, 0x618, 0xffffffffu);
	if (!done) {
		TC_LOG("fifo cancel/abort not acknowledged, status %#x\n", ft_rd(ft.se, 0x040));
	}
}

/* One polled FIFO-mode message: write w or read r, n bytes, START..STOP. */
static int
ft_i2c_fifo(const uint8_t *w, uint8_t *r, unsigned n)
{
	unsigned got = 0, budget = ft_i2c_budget(n);
	uint32_t irq = 0;
	int err = ETIMEDOUT;
	if (ft_rd(ft.se, 0x040) & 1u) {
		tc.last_err = TC_ERR_OTHER;
		tc.ev_a = ft_rd(ft.se, 0x040);
		ft_fifo_stop();
		return EIO;
	}
	ft_wr(ft.se, 0x618, 0xffffffffu);
	if (w) {
		ft_wr(ft.se, 0x26c, n);
		ft_wr(ft.se, 0x600, (1u << 27) | (fk.addr << 9));
		ft_wr(ft.se, 0x80c, 1);
	} else {
		ft_wr(ft.se, 0x270, n);
		ft_wr(ft.se, 0x600, (2u << 27) | (fk.addr << 9));
	}
	for (unsigned us = 0; us < budget; us++) {
		irq = ft_rd(ft.se, 0x610);
		if (irq & FT_M_ERRORS) {
			err = EIO;
			break;
		}
		if (w && got < n && (irq & FT_M_TX_WM)) {
			for (unsigned k = 0; k + 1u < tc.depth && got < n; k++) {
				uint32_t word = 0;
				for (unsigned j = 0; j < 4u && got < n; j++) {
					word |= (uint32_t)w[got++] << (8u * j);
				}
				ft_wr(ft.se, 0x700, word);
			}
			if (got == n) {
				ft_wr(ft.se, 0x80c, 0);
			}
		}
		if (r && ft_fifo_pop(r, n, &got)) {
			err = EIO;
			tc.last_err = TC_ERR_LEN;
			break;
		}
		if (irq & FT_M_DONE) {
			/* Re-sample after the last drain: a late error beats DONE. */
			irq = ft_rd(ft.se, 0x610);
			if (irq & FT_M_ERRORS) {
				err = EIO;
				break;
			}
			if (r && ft_fifo_pop(r, n, &got)) {
				err = EIO;
				tc.last_err = TC_ERR_LEN;
				break;
			}
			ft_wr(ft.se, 0x618, irq);
			if (got == n) {
				return 0;
			}
			tc.last_err = TC_ERR_LEN;
			tc.ev_a = irq;
			return EIO;
		}
		ft_wr(ft.se, 0x618, irq & (FT_M_TX_WM | FT_M_RX_WM | FT_M_RX_LAST));
		delay(1);
	}
	tc.ev_a = irq;
	tc.ev_b = ft_rd(ft.se, 0x040);
	if (err == EIO && tc.last_err == TC_ERR_NONE) {
		tc.gp_len = ft_rd(ft.se, 0x910);
		tc.last_err = (irq & FT_M_NACK) ? (tc.gp_len ? TC_ERR_DNACK : TC_ERR_ANACK) :
		    (irq & FT_M_PROTO) ? TC_ERR_PROTO : (irq & FT_M_ARB) ? TC_ERR_ARB : TC_ERR_OTHER;
		/* A NACK completes the command: wait for DONE, skip the abort if idle. */
		for (unsigned i = 0; i < 1000u && !(irq & FT_M_DONE); i++) {
			delay(1);
			irq = ft_rd(ft.se, 0x610);
		}
		if ((irq & FT_M_DONE) && !(ft_rd(ft.se, 0x040) & 1u) &&
		    !(ft_rd(ft.se, 0x800) & 0xfffffffu) && !(ft_rd(ft.se, 0x804) & 0x1ffffffu)) {
			ft_wr(ft.se, 0x80c, 0);
			ft_wr(ft.se, 0x618, 0xffffffffu);
			return err;
		}
	}
	ft_fifo_stop();
	return err;
}

/* GPI write: [CONFIG0] GO write + TX DMA on channel 0 (design 3.4.3). */
static int
ft_i2c_gpi_write(const uint8_t *buf, unsigned n)
{
	uint32_t len = 0;
	int err;
	memcpy((void *)(ft.dma + FT_TXB_OFF), buf, n);
	ft_dma_sync(FT_TXB_OFF, n);
	ft_c0();
	ft_tre(0, (fk.addr << 8) | 1u, 0, 0, 0x00200001u);
	ft_tre(0, (uint32_t)ft.dma_pa + FT_TXB_OFF, (uint32_t)(ft.dma_pa >> 32), n, 0x00100200u);
	ft_ring_db(0);
	if ((err = ft_gpi_wait(0, &len, ft_i2c_budget(n)))) {
		return err;
	}
	if (len != n) {
		tc.last_err = TC_ERR_LEN;
		tc.ev_a = len;
		return EIO;
	}
	return 0;
}

/* GPI read: RX DMA on channel 1, then [CONFIG0] GO read on channel 0; no dummy byte. */
static int
ft_i2c_gpi_read(uint8_t *buf, unsigned n)
{
	uint32_t len = 0;
	int err;
	ft_dma_sync(FT_RXB_OFF, n);
	ft_tre(1, (uint32_t)ft.dma_pa + FT_RXB_OFF, (uint32_t)(ft.dma_pa >> 32), n, 0x00100200u);
	ft_c0();
	ft_tre(0, (fk.addr << 8) | 2u, 0, n, fk.rgo ? 0x00200800u : 0x00200900u);
	ft_ring_db(1);
	ft_ring_db(0);
	if ((err = ft_gpi_wait(1, &len, ft_i2c_budget(n)))) {
		return err;
	}
	if (len != n) {
		tc.last_err = TC_ERR_LEN;
		tc.ev_a = len;
		return EIO;
	}
	ft_dma_sync(FT_RXB_OFF, n);
	memcpy(buf, (const void *)(ft.dma + FT_RXB_OFF), n);
	return 0;
}

/* Lower-case hex of p[0..n), cut to fit cap (NUL included). */
static void
tcm_hex(const uint8_t *p, unsigned n, char *out, unsigned cap)
{
	static const char digits[] = "0123456789abcdef";
	unsigned o = 0;
	for (unsigned i = 0; i < n && o + 2u < cap; i++) {
		out[o++] = digits[p[i] >> 4];
		out[o++] = digits[p[i] & 15u];
	}
	out[o] = '\0';
}

/*
 * Engine dispatch for one I2C message (write w or read r). A GPI error or
 * timeout re-initialises the GPII at once (the channels are in a bad state
 * after an error, MGPI); the FIFO engine has cancelled/aborted already.
 */
static int
ft_i2c(const uint8_t *w, uint8_t *r, unsigned n)
{
	static const char *const kinds[] = {"none", "addr_nack", "data_nack", "bus_proto",
		                            "arb_lost", "timeout", "other", "length"};
	uint64_t t0 = mach_absolute_time(), us;
	int err;
	if (!n || n > TC_XFER_MAX || (!w && !r)) {
		return EINVAL;
	}
	tc.last_err = TC_ERR_NONE;
	tc.ev_a = tc.ev_b = tc.gp_len = 0;
	if (tc.engine == TC_ENG_GPI) {
		err = w ? ft_i2c_gpi_write(w, n) : ft_i2c_gpi_read(r, n);
	} else {
		err = ft_i2c_fifo(w, r, n);
	}
	us = ft_us(mach_absolute_time() - t0);
	if (fk.dbg & 4u) {
		TC_LOG("xfer %s %u B -> %d in %llu us\n", w ? "write" : "read", n, err, us);
	}
	if (err) {
		if (tc.last_err == TC_ERR_NONE) {
			tc.last_err = err == ETIMEDOUT ? TC_ERR_TIME : TC_ERR_OTHER;
		}
		tc.bus_errs++;
		if (tc.bus_errs <= 8u || (tc.bus_errs % 100u) == 0) {
			bool gpi = tc.engine == TC_ENG_GPI;
			TC_FACT("bus err kind=%s ev=%#x %#x gp_len=%u ch_state=%#x/%#x\n",
			    kinds[tc.last_err < 8u ? tc.last_err : TC_ERR_OTHER], tc.ev_a, tc.ev_b, tc.gp_len,
			    gpi ? ft_state(GPI_CH(0)) : ft_rd(ft.se, 0x040),
			    gpi ? ft_state(GPI_CH(1)) : ft_rd(ft.se, 0x610));
		}
		if (tc.engine == TC_ENG_GPI) {
			(void)ft_gpi_init();
		}
		return err;
	}
	if (r) {
		tc.rd_bytes += n + 1u;
		tc.rd_us += us;
		if (++tc.rd_count == 32u && tc.rd_us) {
			TC_FACT("bus est_khz=%llu from %llu B in %llu us\n",
			    tc.rd_bytes * 9000u / tc.rd_us, tc.rd_bytes, tc.rd_us);
		}
		if (tc.engine == TC_ENG_GPI && !tc.gpi_ok_logged) {
			tc.gpi_ok_logged = true;
			TC_FACT("gpi read ok flags=%#x\n", fk.rgo ? 0x00200800u : 0x00200900u);
		}
	}
	return 0;
}

static int
ft_i2c_write(const uint8_t *buf, unsigned n)
{
	return ft_i2c(buf, NULL, n);
}

static int
ft_i2c_read(uint8_t *buf, unsigned n)
{
	return ft_i2c(NULL, buf, n);
}

/* Shared-loop recovery (3 and every 50 errors): GPI re-init, or FIFO stop + SE re-init. */
static int
ft_bus_reinit(void)
{
	int err;
	if (!tc.on || tc.engine == TC_ENG_GPI) {
		err = ft_gpi_init();
	} else {
		ft_fifo_stop();
		err = ft_se_fifo();
	}
	if (tc.on) {
		TC_LOG("bus re-init ios=%#x -> %d\n", ft_rd(ft.se, 0x908), err);
	}
	return err;
}

/* The IRQ/ATTN line per fuxitirq: 0 never asserted, 1 active low, 2 active high. */
static bool
ft_irq_line(void)
{
	uint32_t in = ft_rd(ft.tlmm, FT_GPIO(tc.pin_attn) + 4) & 1u;
	return fk.irq == 1u ? in == 0u : (fk.irq == 2u && in != 0u);
}

/*
 * Never sent (fuxitcmd and ADT scripts included): commands that touch flash
 * or leave the application firmware (0x11 ERASE_FLASH, 0x12 WRITE_FLASH,
 * 0x13 READ_FLASH, 0x16 REBOOT_TO_ROM_BOOTLOADER, 0x1f RUN_BOOTLOADER_FIRMWARE),
 * that write the config to flash (0x28 COMMIT_CONFIG, 0x2b SET_CONFIG_ID, the
 * customer config id) or pass bytes to the IC's SPI master, where a flash
 * part can sit (0x15, 0x43 SPI_MASTER_WRITE_THEN_READ[_EXTENDED]).
 */
static bool
tcm_flash_cmd(unsigned cmd)
{
	return cmd == 0x11u || cmd == 0x12u || cmd == 0x13u || cmd == 0x15u || cmd == 0x16u ||
	       cmd == 0x1fu || cmd == 0x28u || cmd == 0x2bu || cmd == 0x43u;
}

static unsigned
tcm_le16(const uint8_t *p, unsigned n, unsigned off)
{
	return off + 2u <= n ? (unsigned)p[off] | ((unsigned)p[off + 1u] << 8) : 0u;
}

/* Report field: w bits at bit offset off, LSB first; past the end reads 0. */
static uint32_t
tcm_bits(const uint8_t *p, unsigned len, unsigned off, unsigned w)
{
	uint32_t v = 0;
	if (w == 0 || w > 32u || off + w > len * 8u) {
		return 0;
	}
	for (unsigned i = 0; i < w; i++) {
		unsigned b = off + i;
		v |= (uint32_t)((p[b >> 3] >> (b & 7u)) & 1u) << i;
	}
	return v;
}

/* Length of a report config up to and including END (0); codes above 4 carry a width byte. */
static unsigned
tcm_cfg_len(const uint8_t *cfg, unsigned max)
{
	unsigned i = 0;
	while (i < max) {
		unsigned code = cfg[i++];
		if (code == 0) {
			return i;
		}
		if (code > 4u) {
			i++;
		}
	}
	return max;
}

/* Index just past the FOREACH_END (3) at or after idx, or n. */
static unsigned
tcm_cfg_skip(const uint8_t *cfg, unsigned n, unsigned idx)
{
	while (idx < n) {
		unsigned code = cfg[idx++];
		if (code == 3u) {
			return idx;
		}
		if (code == 0) {
			return n;
		}
		if (code > 4u) {
			idx++;
		}
	}
	return n;
}

/* Bits of one object block between idx and its FOREACH_END (pads count 0). */
static unsigned
tcm_cfg_body(const uint8_t *cfg, unsigned n, unsigned idx)
{
	unsigned bits = 0;
	while (idx < n) {
		unsigned code = cfg[idx++];
		if (code == 0 || code == 3u) {
			break;
		}
		if (code > 4u && idx < n) {
			bits += cfg[idx++];
		}
	}
	return bits;
}

/*
 * Walk the active report config (tc.cfg) over one 0x11 payload (design
 * 4.6, the Synaptics reference walker: unknown entities are skipped by
 * their width). Objects land in tc.px/py/pcls; tc.pseen marks the objects
 * whose classification lies inside the report. An object block that would
 * run past the end of the report is not entered. EINVAL drops the report;
 * every object index stays below FT_MAX.
 */
static int
tcm_parse(const uint8_t *p, unsigned len)
{
	const uint8_t *cfg = tc.cfg;
	unsigned n = tc.cfg_len, idx = 0, loop = 0, off = 0, obj = 0, seen = 0, body = 0;
	unsigned bits = len * 8u, steps = 0;
	int active = -1;
	bool only = false;
	tc.pseen = 0;
	for (unsigned i = 0; i < FT_MAX; i++) {
		tc.px[i] = tc.py[i] = tc.pcls[i] = 0;
	}
	while (idx < n) {
		unsigned code = cfg[idx++], w;
		uint32_t v;
		if (++steps > 65536u) {
			return EINVAL;
		}
		if (code == 0) {
			break;
		}
		if (code == 1u || code == 2u) {
			only = code == 1u;
			obj = 0;
			seen = 0;
			loop = idx;
			body = tcm_cfg_body(cfg, n, idx);
			if (only && (active == 0 || (active < 0 && off + body > bits))) {
				idx = tcm_cfg_skip(cfg, n, idx);
			}
			continue;
		}
		if (code == 3u) {
			if (only) {
				if (active >= 0 ? ++seen < (unsigned)active : (body && off + body <= bits)) {
					idx = loop;
				}
			} else if (++obj < FT_MAX) {
				idx = loop;
			}
			continue;
		}
		if (code == 4u) {
			off = (off + 7u) & ~7u;
			continue;
		}
		if (idx >= n) {
			return EINVAL;
		}
		w = cfg[idx++];
		if (obj >= FT_MAX && code >= 7u && code <= 9u) {
			return EINVAL;   /* object field after a completed FOREACH_OBJECT loop */
		}
		v = tcm_bits(p, len, off, w);
		switch (code) {
		case 6:
			if (v >= FT_MAX) {
				return EINVAL;
			}
			obj = v;
			break;
		case 7:
			tc.pcls[obj] = v;
			if (off + w <= bits) {
				tc.pseen |= 1u << obj;
			}
			break;
		case 8:
			tc.px[obj] = v;
			break;
		case 9:
			tc.py[obj] = v;
			break;
		case 24:
			active = (int)v;
			if (!v) {
				idx = tcm_cfg_skip(cfg, n, idx);
			}
			break;
		default:
			break;
		}
		off += w;
	}
	return 0;
}

/* Chip coordinates to the output space (design 4.8): span, swap/flip, scale, offset, clamp. */
static void
tcm_map(uint32_t rx, uint32_t ry, int *ox, int *oy)
{
	uint32_t x = rx, y = ry, sx = tc.sx ? tc.sx : TC_SPAN_X, sy = tc.sy ? tc.sy : TC_SPAN_Y, t;
	uint32_t ow = tc.ow ? tc.ow : TC_OUT_W, oh = tc.oh ? tc.oh : TC_OUT_H;
	int64_t px, py;
	if (fk.xy & 1u) {
		t = x;
		x = y;
		y = t;
		t = sx;
		sx = sy;
		sy = t;
	}
	x = x < sx ? x : sx - 1u;
	y = y < sy ? y : sy - 1u;
	if (fk.xy & 2u) {
		x = sx - 1u - x;
	}
	if (fk.xy & 4u) {
		y = sy - 1u - y;
	}
	px = (int64_t)((uint64_t)x * ow / sx) + fk.ox;
	py = (int64_t)((uint64_t)y * oh / sy) + fk.oy;
	px = px < 0 ? 0 : (px > (int64_t)ow - 1 ? (int64_t)ow - 1 : px);
	py = py < 0 ? 0 : (py > (int64_t)oh - 1 ? (int64_t)oh - 1 : py);
	*ox = (int)px;
	*oy = (int)py;
}

/* Unexpected message codes: logged once each (D22). */
static void
tcm_other(unsigned code)
{
	uint32_t bit = 1u << (code & 31u);
	if (!(tc.codes_seen[(code >> 5) & 7u] & bit)) {
		tc.codes_seen[(code >> 5) & 7u] |= bit;
		TC_FACT("code %#x len %u (first)\n", code, tc.len);
	}
}

/*
 * Read one TouchComm v1 message into tc.msg (design 4.1): a predictive read
 * of R bytes, then continued reads ("a5 03" + the rest) until the whole
 * message "a5 <code> <len LE16> <payload> 5a" is in. IDLE, BUSY and
 * CONTINUED_READ carry nothing. ENXIO: no marker; EIO: framing error.
 */
static int
tcm_read_msg(void)
{
	char hex[2 * 32 + 1];
	unsigned r = fk.rdlen ? fk.rdlen : tc.rdlen, total;
	int err;
	r = r < 9u ? 9u : (r > TC_XFER_MAX ? TC_XFER_MAX : r);
	tc.code = 0xffu;
	tc.len = 0;
	if ((err = ft_i2c_read(tc.msg, r))) {
		return err;
	}
	if (++tc.reads == 1u) {
		tcm_hex(tc.msg, r, hex, sizeof(hex));
		TC_FACT("first read %u B: %s\n", r, hex);
	}
	if (tc.msg[0] != TCM_MARKER) {
		if (tc.marker_logs++ < 4u) {
			tcm_hex(tc.msg, r < 8u ? r : 8u, hex, sizeof(hex));
			TC_LOG("no marker: %s\n", hex);
		}
		return ENXIO;
	}
	tc.code = tc.msg[1];
	if (tc.code == TCM_IDLE || tc.code == TCM_BUSY || tc.code == TCM_CONT) {
		if (!tc.idle_logged && tc.code == TCM_IDLE) {
			tc.idle_logged = true;
			tcm_hex(tc.msg, r, hex, sizeof(hex));
			TC_FACT("idle read %s\n", hex);
		}
		return 0;
	}
	tc.len = tcm_le16(tc.msg, 4, 2);
	total = tc.len + 5u;
	if (total > TC_MSG_MAX) {
		TC_LOG("code %#x len %u too long\n", tc.code, tc.len);
		tc.code = 0xffu;
		return EIO;
	}
	for (unsigned have = r; have < total;) {
		unsigned rem = total - have, k = rem > TC_XFER_MAX - 2u ? TC_XFER_MAX - 2u : rem;
		if (rem == 1u) {
			tc.msg[have] = TCM_PAD;   /* only the padding is left */
			break;
		}
		if ((err = ft_i2c_read(tc.tmp, k + 2u))) {
			tc.code = 0xffu;
			return err;
		}
		if (tc.tmp[0] != TCM_MARKER || tc.tmp[1] != TCM_CONT) {
			TC_LOG("continued read %02x %02x\n", tc.tmp[0], tc.tmp[1]);
			tc.code = 0xffu;
			return EIO;
		}
		memcpy(tc.msg + have, tc.tmp + 2, k);
		have += k;
	}
	if (tc.msg[total - 1u] != TCM_PAD) {
		tc.pad_errs++;
		TC_FACT("pad err #%u last=%#x\n", tc.pad_errs, tc.msg[total - 1u]);
		tc.code = 0xffu;
		return EIO;
	}
	tc.rdlen = total < 9u ? 9u : (total > TC_XFER_MAX ? TC_XFER_MAX : total);
	if (tc.attn_logs < 4u) {
		tc.attn_logs++;
		TC_FACT("attn after read high=%u\n", ft_irq_line() ? 0u : 1u);
	}
	if ((fk.dbg & 2u) && (tc.rx_logs < 16u || !(tc.rx_codes[(tc.code >> 5) & 7u] & (1u << (tc.code & 31u))))) {
		tc.rx_logs++;
		tc.rx_codes[(tc.code >> 5) & 7u] |= 1u << (tc.code & 31u);
		tcm_hex(tc.msg, total, hex, sizeof(hex));
		TC_LOG("rx %02x %u %s\n", tc.code, tc.len, hex);
	}
	return 0;
}

/*
 * Host-to-device command "<cmd> <len LE16> <payload>" (design 4.1); above
 * the identify max_write_size the rest goes in "01 <bytes>" chunks. The one
 * place a command leaves: flash and bootloader commands are refused here.
 */
static int
tcm_write_cmd(unsigned cmd, const uint8_t *pl, unsigned n)
{
	unsigned chunk = tc.maxwr >= 8u && tc.maxwr < TC_XFER_MAX ? tc.maxwr : TC_XFER_MAX;
	unsigned first = n + 3u > chunk ? chunk - 3u : n, sent;
	int err;
	if (tcm_flash_cmd(cmd)) {
		TC_LOG("cmd %02x refused (flash/bootloader command)\n", cmd);
		return EPERM;
	}
	if (n > TC_XFER_MAX - 3u || (n && !pl)) {
		return EINVAL;
	}
	tc.out[0] = (uint8_t)cmd;
	tc.out[1] = (uint8_t)n;
	tc.out[2] = (uint8_t)(n >> 8);
	if (first) {
		memcpy(tc.out + 3, pl, first);
	}
	if ((err = ft_i2c_write(tc.out, first + 3u))) {
		return err;
	}
	for (sent = first; sent < n;) {
		unsigned k = n - sent > chunk - 1u ? chunk - 1u : n - sent;
		tc.out[0] = TCM_C_CONT;
		memcpy(tc.out + 1, pl + sent, k);
		if ((err = ft_i2c_write(tc.out, k + 1u))) {
			return err;
		}
		sent += k;
	}
	return 0;
}

/* Identify payload (tc.msg): version, mode, part number, build id, max write size. */
static void
tcm_take_identify(void)
{
	const uint8_t *p = tc.msg + 4;
	unsigned n = tc.len, i;
	tc.id_len = n;
	tc.id_ver = n ? p[0] : 0u;
	tc.mode = n > 1u ? p[1] : 0u;
	for (i = 0; i < 16u && 2u + i < n && p[2u + i]; i++) {
		tc.part[i] = p[2u + i] >= 0x20u && p[2u + i] < 0x7fu ? (char)p[2u + i] : '?';
	}
	tc.part[i] = '\0';
	tc.build = tcm_le16(p, n, 18) | (tcm_le16(p, n, 20) << 16);
	tc.maxwr = tcm_le16(p, n, 22);
}

static void
tcm_log_identify(void)
{
	TC_FACT("identify len=%u ver=%u mode=%#x part=\"%s\" build=%u maxwr=%u\n",
	    tc.id_len, tc.id_ver, tc.mode, tc.part, tc.build, tc.maxwr);
}

static int tcm_dispatch(bool deliver);

/*
 * Send one command and poll for its answer (design 4.2). want is TCM_OK, or
 * TCM_IDENTIFY for reset and run-application. A read happens when ATTN is
 * asserted or 10 ms passed. An identify report (always parsed) instead of
 * the answer means the IC reset under the command (tc.cmd_reset); 0x11
 * reports read meanwhile go to tcm_dispatch once sensing.
 */
static int
tcm_cmd(unsigned cmd, const uint8_t *pl, unsigned n, unsigned want, unsigned timeout_ms)
{
	unsigned waited = 0, since = 0, bad = 0;
	int err = tcm_write_cmd(cmd, pl, n);
	if (err) {
		TC_LOG("cmd %02x write failed %d\n", cmd, err);
		return err;
	}
	while (waited < timeout_ms) {
		ft_sleep(1);
		waited++;
		if (!ft_irq_line() && ++since < 10u) {
			continue;
		}
		since = 0;
		if ((err = tcm_read_msg())) {
			if (++bad >= 3u) {
				TC_LOG("cmd %02x: read failed %d\n", cmd, err);
				return err;
			}
			continue;
		}
		if (tc.code == TCM_IDENTIFY) {
			tcm_take_identify();
		}
		if (tc.code == want) {
			return 0;
		}
		if (tc.code == TCM_IDLE || tc.code == TCM_BUSY || tc.code == TCM_CONT || tc.code == TCM_OK) {
			continue;   /* OK before an identify: keep waiting for the report */
		}
		if (tc.code == TCM_IDENTIFY) {
			tc.cmd_reset = true;
			TC_LOG("cmd %02x interrupted by identify (mode %#x)\n", cmd, tc.mode);
			return EIO;
		}
		if (tc.code < 0x10u || tc.code == 0xffu) {
			TC_LOG("cmd %02x -> status %02x err %02x\n", cmd, tc.code, tc.len ? tc.msg[4] : 0u);
			return EIO;
		}
		if (tc.code == TCM_TOUCH && tc.sensing) {
			(void)tcm_dispatch(tc.deliver);
			continue;
		}
		tcm_other(tc.code);
	}
	TC_LOG("cmd %02x: no answer in %u ms\n", cmd, timeout_ms);
	return ETIMEDOUT;
}

/* B2: application firmware or stop (flash is never touched, design section 1). */
static int
tcm_app_mode(void)
{
	for (unsigned i = 0; i < fk.run && tc.mode != TCM_MODE_APP && tc.mode != 0x02u; i++) {
		tc.cmd_reset = false;
		if (tcm_cmd(TCM_C_RUN_APP, NULL, 0, TCM_IDENTIFY, 1000) && !tc.cmd_reset &&
		    !tcm_cmd(TCM_C_IDENTIFY, NULL, 0, TCM_OK, 1000)) {
			tcm_take_identify();
		}
		tcm_log_identify();
	}
	if (tc.mode == TCM_MODE_APP) {
		return 0;
	}
	TC_LOG("firmware mode %#x (%s): touch stopped\n", tc.mode, tc.mode == 0x02u ?
	    "host download" : (tc.mode == 0x0bu || tc.mode == 0x0cu) ? "bootloader" : "not application");
	if (!tcm_cmd(TCM_C_BOOT_INFO, NULL, 0, TCM_OK, 1000)) {
		TC_FACT("boot status=%#x reset_reason=%#x\n", tc.len > 1u ? tc.msg[5] : 0u,
		    tc.len > 9u ? tc.msg[13] : 0u);
	}
	return ENODEV;
}

/* B3: application info, status gate, coordinate span. */
static int
tcm_app_info(void)
{
	const uint8_t *p = tc.msg + 4;
	char id[17];
	unsigned i, n;
	int err = EIO;
	for (unsigned t = 0; t <= 10u; t++) {
		if (t) {
			ft_sleep(100);
		}
		if ((err = tcm_cmd(TCM_C_APP_INFO, NULL, 0, TCM_OK, 1000))) {
			return err;
		}
		tc.app_status = tcm_le16(p, tc.len, 2);
		if (tc.app_status != 1u && tc.app_status != 2u) {
			break;
		}
	}
	n = tc.len;
	tc.cfg_max = tcm_le16(p, n, 12);
	tc.max_x = tcm_le16(p, n, 32);
	tc.max_y = tcm_le16(p, n, 34);
	for (i = 0; i < 16u && 16u + i < n && p[16u + i]; i++) {
		id[i] = p[16u + i] >= 0x20u && p[16u + i] < 0x7fu ? (char)p[16u + i] : '?';
	}
	id[i] = '\0';
	if (!tc.app_logged) {
		tc.app_logged = true;
		TC_FACT("app len=%u ver=%u status=%u cfgmax=%u paymax=%u id=\"%s\" maxx=%u maxy=%u "
		    "objs=%u rows=%u cols=%u hybrid=%u\n", n, tcm_le16(p, n, 0), tc.app_status,
		    tc.cfg_max, tcm_le16(p, n, 14), id, tc.max_x, tc.max_y, tcm_le16(p, n, 36),
		    tcm_le16(p, n, 40), tcm_le16(p, n, 42), tcm_le16(p, n, 44));
		tc.sx = fk.spanx ? fk.spanx : (tc.max_x + 1u >= 16u ? tc.max_x + 1u : TC_SPAN_X);
		tc.sy = fk.spany ? fk.spany : (tc.max_y + 1u >= 16u ? tc.max_y + 1u : TC_SPAN_Y);
		TC_FACT("span %ux%u (%s)\n", tc.sx, tc.sy, (fk.spanx || fk.spany) ? "knob" :
		    (tc.max_x + 1u >= 16u && tc.max_y + 1u >= 16u) ? "app" : "fallback");
	}
	if (tc.app_status) {
		TC_LOG("app status %#x (%s): touch stopped\n", tc.app_status,
		    tc.app_status == 0xffu ? "bad app config" : "not ok");
		return ENODEV;
	}
	return 0;
}

/* B4-B6: firmware config, the chosen config (fuxitrcfg) and the active one. */
static int
tcm_set_config(void)
{
	static const uint8_t cfg_min[] = {0x01, 0x06, 0x04, 0x07, 0x04, 0x08, 0x10, 0x09, 0x10, 0x03, 0x00};
	static const uint8_t cfg_oplus[] = {0x10, 0x08, 0xc6, 0x30, 0x01, 0x06, 0x04, 0x07, 0x04, 0x08,
		                            0x10, 0x09, 0x10, 0x0b, 0x0c, 0x0c, 0x0c, 0xcb, 0x20, 0x03, 0x00};
	char hex[2 * 64 + 1];
	const uint8_t *want = NULL;
	unsigned wlen = 0, n;
	int serr = 0, gerr;
	if (!tcm_cmd(TCM_C_GET_CFG, NULL, 0, TCM_OK, 1000)) {
		n = tc.len < TC_CFG_MAX ? tc.len : TC_CFG_MAX;
		memcpy(tc.fw_cfg, tc.msg + 4, n);
		tc.fw_cfg_len = n;
		if (!tc.fwcfg_logged) {
			tc.fwcfg_logged = true;
			tcm_hex(tc.fw_cfg, tcm_cfg_len(tc.fw_cfg, n), hex, sizeof(hex));
			TC_FACT("fw cfg len=%u %s\n", n, hex);
		}
	}
	if (fk.rcfg == 1u) {
		want = cfg_min;
		wlen = (unsigned)sizeof(cfg_min);
	} else if (fk.rcfg == 2u) {
		want = cfg_oplus;
		wlen = (unsigned)sizeof(cfg_oplus);
	} else if (fk.rcfg == 3u && fk.has_rcfgb) {
		want = fk.rcfgb;
		wlen = tcm_cfg_len(fk.rcfgb, TC_BLOB_MAX);
	}
	if (want) {
		if (tc.cfg_max < wlen || tc.cfg_max > TC_XFER_MAX - 3u) {
			TC_LOG("set cfg: max config size %u, config %u B: not sent\n", tc.cfg_max, wlen);
			serr = EINVAL;
		} else {
			bzero(tc.tmp, tc.cfg_max);
			memcpy(tc.tmp, want, wlen);
			serr = tcm_cmd(TCM_C_SET_CFG, tc.tmp, tc.cfg_max, TCM_OK, 1000);
		}
	}
	/* B6: the parser uses exactly what the firmware reports now. */
	if (!(gerr = tcm_cmd(TCM_C_GET_CFG, NULL, 0, TCM_OK, 1000))) {
		n = tc.len < TC_CFG_MAX ? tc.len : TC_CFG_MAX;
		memcpy(tc.cfg, tc.msg + 4, n);
		tc.cfg_len = n;
	} else if (want && !serr) {
		memcpy(tc.cfg, want, wlen);
		tc.cfg_len = wlen;
	} else if (tc.fw_cfg_len) {
		memcpy(tc.cfg, tc.fw_cfg, tc.fw_cfg_len);
		tc.cfg_len = tc.fw_cfg_len;
	} else {
		TC_LOG("no report config (set %d get %d): touch stopped\n", serr, gerr);
		return EIO;
	}
	if (tc.set_logs++ < 4u) {
		tcm_hex(tc.cfg, tcm_cfg_len(tc.cfg, tc.cfg_len), hex, sizeof(hex));
		TC_FACT("set cfg mode=%u len=%u -> %#x; active len=%u %s\n", fk.rcfg,
		    want ? tc.cfg_max : 0u, (unsigned)serr, tc.cfg_len, hex);
	}
	return 0;
}

/* GPIO record of a script: only the touch pins (avdd, reset, attn, sda, scl). */
static int
tcm_script_gpio(const uint8_t *d)
{
	unsigned pin = d[0], func = d[1], pull = d[2], ma = d[3], out = d[4];
	if ((pin != tc.pin_avdd && pin != tc.pin_rst && pin != tc.pin_attn && pin != FT_SDA &&
	    pin != FT_SCL) || func > 15u || pull > 3u || ma < 2u || ma > 16u || (ma & 1u) ||
	    (out > 1u && out != 0xffu)) {
		return EINVAL;
	}
	ft_gpio(pin, func, pull, ma, out == 0xffu ? -1 : (int)out);
	return 0;
}

/*
 * ADT script fuxitscr<k> (design 4.11): records "<op> <n> <n bytes>" up to
 * op 0. 1 TouchComm command, 2 raw write, 3 raw read (logged), 4 sleep ms
 * LE16, 5 GPIO. Flash and bootloader commands are refused, raw or not.
 */
static void
tcm_script(unsigned k, const uint8_t *s)
{
	char hex[2 * 32 + 1];
	unsigned i = 0;
	while (i + 2u <= TC_BLOB_MAX && s[i]) {
		unsigned op = s[i], n = s[i + 1u];
		const uint8_t *d = s + i + 2u;
		int err = EINVAL;
		if (i + 2u + n > TC_BLOB_MAX) {
			TC_LOG("scr%u truncated record at %u\n", k, i);
			return;
		}
		if (op == 1u && n >= 1u) {
			err = tcm_cmd(d[0], d + 1, n - 1u,
			    d[0] == TCM_C_RESET || d[0] == TCM_C_RUN_APP ? TCM_IDENTIFY : TCM_OK, 1000);
		} else if (op == 2u && n >= 1u) {
			err = tcm_flash_cmd(d[0]) ? EPERM : ft_i2c_write(d, n);
		} else if (op == 3u && n >= 1u && d[0]) {
			if (!(err = ft_i2c_read(tc.tmp, d[0]))) {
				tcm_hex(tc.tmp, d[0], hex, sizeof(hex));
				TC_LOG("scr%u read %u B: %s\n", k, d[0], hex);
			}
		} else if (op == 4u && n >= 2u) {
			ft_sleep((unsigned)d[0] | ((unsigned)d[1] << 8));
			err = 0;
		} else if (op == 5u && n >= 5u) {
			err = tcm_script_gpio(d);
		} else if (op > 5u) {
			TC_LOG("scr%u op%u unknown: script stopped\n", k, op);
			return;
		}
		TC_LOG("scr%u op%u -> %d\n", k, op, err);
		i += 2u + n;
	}
}

/* B7: optional dynamic config, raw command and script 1 (failures are not fatal). */
static void
tcm_extra(void)
{
	uint8_t pl[6];
	int err;
	if (fk.dc) {
		pl[0] = (uint8_t)(fk.dc >> 16);
		pl[1] = (uint8_t)fk.dc;
		pl[2] = (uint8_t)(fk.dc >> 8);
		err = tcm_cmd(TCM_C_SET_DC, pl, 3, TCM_OK, 300);
		TC_LOG("dynamic config id %#x value %#x -> %d\n", fk.dc >> 16, fk.dc & 0xffffu, err);
	}
	if (fk.cmd) {
		unsigned c = (unsigned)(fk.cmd & 0xffu), n = (unsigned)((fk.cmd >> 8) & 0xffu);
		for (unsigned i = 0; i < n; i++) {
			pl[i] = (uint8_t)(fk.cmd >> (16u + 8u * i));
		}
		err = tcm_cmd(c, pl, n, c == TCM_C_RESET || c == TCM_C_RUN_APP ? TCM_IDENTIFY : TCM_OK, 1000);
		TC_LOG("fuxitcmd %02x len %u -> %d\n", c, n, err);
	}
	if (fk.has_scr1) {
		tcm_script(1, fk.scr1);
	}
}

/* B3-B7, after bring-up and after every IC self-reset. */
static int
tcm_setup(void)
{
	int err = tcm_app_info();
	if (!err) {
		err = tcm_set_config();
	}
	if (!err) {
		tcm_extra();
	}
	return err;
}

/* X3-X8 per fuxitpwr: ATTN input, I2C mux (bit 2), reset (bit 1), AVDD (bit 0). */
static void
tcm_power(void)
{
	unsigned pins[5] = {tc.pin_avdd, tc.pin_rst, tc.pin_attn, FT_SDA, FT_SCL};
	uint32_t ios = ft_rd(ft.se, 0x908);
	if (!tc.pins_logged) {
		for (unsigned i = 0; i < 5u; i++) {
			TC_FACT("pins gpio%u ctl=%#x io=%#x\n", pins[i], ft_rd(ft.tlmm, FT_GPIO(pins[i])),
			    ft_rd(ft.tlmm, FT_GPIO(pins[i]) + 4));
		}
	}
	ft_gpio(tc.pin_attn, 0, 3, 8, -1);
	if (fk.pwr & 4u) {
		ft_gpio_ctl(FT_SDA, fk.pinctl);
		ft_gpio_ctl(FT_SCL, fk.pinctl);
	}
	if (!tc.pins_logged) {
		tc.pins_logged = true;
		TC_FACT("ios before=%#x after=%#x\n", ios, ft_rd(ft.se, 0x908));
	}
	if (fk.pwr & 2u) {
		ft_gpio(tc.pin_rst, 0, 0, 8, 0);
	}
	if (fk.pwr & 1u) {
		ft_gpio(tc.pin_avdd, 0, 1, 2, 1);
	}
	if ((fk.pwr & 3u) && fk.pwrw) {
		ft_sleep(fk.pwrw);
	}
	if (fk.pwr & 2u) {
		ft_sleep(fk.rst);
		ft_gpio(tc.pin_rst, 0, 0, 8, 1);
	}
}

/* X9: sleep-poll ATTN every 2 ms up to max_ms; returns the time waited. */
static unsigned
tcm_wait_attn(unsigned max_ms)
{
	unsigned ms = 0;
	while (ms < max_ms && !ft_irq_line()) {
		ft_sleep(2);
		ms += 2u;
	}
	return ms;
}

/* Power/reset steps, then the identify (B1), application mode (B2) and B3-B7. */
static int
tcm_bringup(void)
{
	unsigned ms;
	int err;
	tc.sensing = false;
	tc.sleeping = false;
	tc.mode = 0;
	tc.maxwr = 0;
	tc.rdlen = 9;
	tc.raw_prev = 0;
	tc.motion = false;
	tcm_power();
	if (fk.has_scr0) {
		tcm_script(0, fk.scr0);
	}
	ms = tcm_wait_attn(fk.rstw);
	if (ft_irq_line()) {
		TC_FACT("attn after reset %u ms\n", ms);
	} else {
		TC_FACT("attn not seen in %u ms\n", ms);
	}
	/* B1: the identify report the reset left pending, else ask (V1:1331-1362). */
	err = tcm_read_msg();
	if (!err && tc.code == TCM_IDENTIFY) {
		tcm_take_identify();
	} else {
		TC_LOG("no identify pending (err %d code %#x): CMD_IDENTIFY\n", err, tc.code);
		tc.cmd_reset = false;
		err = tcm_cmd(TCM_C_IDENTIFY, NULL, 0, TCM_OK, 1000);
		if (!err) {
			tcm_take_identify();
		} else if (!tc.cmd_reset) {
			err = tcm_cmd(TCM_C_RESET, NULL, 0, TCM_IDENTIFY, 1000);
		} else {
			err = 0;
		}
		if (err) {
			TC_LOG("no identify (err %d): touch stopped\n", err);
			return err;
		}
	}
	tcm_log_identify();
	if (!(err = tcm_app_mode())) {
		err = tcm_setup();
	}
	if (!err) {
		tc.sensing = true;
		tc.sense_abs = mach_absolute_time();
		TC_LOG("sensing (config %u B, span %ux%u, out %ux%u)\n", tc.cfg_len, tc.sx, tc.sy,
		    tc.ow, tc.oh);
	}
	return err;
}

/* Bring-up, twice at most; a power cycle in between when fuxitpwr bit 3 allows. */
static int
tcm_start(void)
{
	int err = EIO;
	for (unsigned a = 0; a < 2u; a++) {
		if (a) {
			if (fk.pwr & 8u) {
				tc.cycle_at = mach_absolute_time();
				ft_gpio(tc.pin_rst, 0, 0, 8, 0);
				ft_gpio(tc.pin_avdd, 0, 1, 2, 0);
			}
			ft_sleep(100);
		}
		if (!(err = tcm_bringup())) {
			break;
		}
		TC_LOG("bring-up attempt %u failed %d\n", a + 1u, err);
		if (err == ENODEV || err == EPERM) {
			break;
		}
	}
	return err;
}

/*
 * After 50 errors in a row (fuxitpwr bit 1 or 3): bring-up again, with a
 * power cycle first when bit 3 allows; at most once a minute (EBUSY).
 */
static int
tcm_recover(void)
{
	uint64_t now = mach_absolute_time();
	if (tc.cycle_at && ft_us(now - tc.cycle_at) < 60000000ull) {
		return EBUSY;
	}
	tc.cycle_at = now;
	if (fk.pwr & 8u) {
		ft_gpio(tc.pin_rst, 0, 0, 8, 0);
		ft_gpio(tc.pin_avdd, 0, 1, 2, 0);
		ft_sleep(100);
	}
	return tcm_bringup();
}

/* Unsolicited identify while sensing: B2-B7 again (the report config is volatile). */
static int
tcm_resume(void)
{
	int err = EIO;
	tc.resets++;
	if (tc.resets <= 8u || (tc.resets % 100u) == 0) {
		TC_FACT("ic reset #%u mode=%#x\n", tc.resets, tc.mode);
	}
	tc.sensing = false;
	tc.sleeping = false;
	tc.raw_prev = 0;
	tc.motion = false;
	for (unsigned a = 0; a < 3u && err; a++) {
		if (a) {
			ft_sleep(100);
		}
		tc.cmd_reset = false;
		if (!(err = tcm_app_mode())) {
			err = tcm_setup();
		}
	}
	tc.sensing = !err;
	return err;
}

/* Per-contact report statistics (D18): first finger down to last finger up. */
static void
tcm_stats(uint32_t raw, uint64_t now)
{
	unsigned gap;
	if (raw && !tc.cs_on) {
		tc.cs_on = true;
		tc.cs_start = tc.cs_last = now;
		tc.cs_reports = tc.cs_gap = 0;
		tc.cs_rx = tc.cs_ry = tc.cs_cls = 0;
	}
	if (!tc.cs_on) {
		return;
	}
	gap = (unsigned)(ft_us(now - tc.cs_last) / 1000u);
	tc.cs_gap = gap > tc.cs_gap ? gap : tc.cs_gap;
	tc.cs_last = now;
	tc.cs_reports++;
	for (unsigned id = 0; id < FT_MAX; id++) {
		if (tc.pseen & (1u << id)) {
			tc.cs_rx = tc.px[id] > tc.cs_rx ? tc.px[id] : tc.cs_rx;
			tc.cs_ry = tc.py[id] > tc.cs_ry ? tc.py[id] : tc.cs_ry;
			tc.cs_cls |= 1u << (tc.pcls[id] & 31u);
		}
	}
	if (!raw) {
		tc.cs_on = false;
		tc.cs_n++;
		if (tc.cs_n <= 8u || (tc.cs_n % 100u) == 0 || (fk.dbg & 16u)) {
			TC_FACT("contact stats reports=%u in %llu ms gap_max=%u ms raw_max=%u,%u classes=%#x\n",
			    tc.cs_reports, ft_us(now - tc.cs_start) / 1000u, tc.cs_gap, tc.cs_rx, tc.cs_ry,
			    tc.cs_cls);
		}
	}
}

/*
 * One 0x11 report (design 4.7): a full snapshot. A down-set change reaches
 * HID at once (a tap shorter than a poll is not lost); movement is
 * coalesced until the end of the drain (tc.motion). Tap-to-wake: the same
 * filter as FTS, fed with every contact of this and the previous snapshot.
 */
static int
tcm_dispatch(bool deliver)
{
	uint64_t now = mach_absolute_time();
	uint32_t raw = 0, mask = 0, all;
	bool palm = false, moved = false;
	if (tcm_parse(tc.msg + 4, tc.len)) {
		if (++tc.parse_errs <= 4u) {
			char hex[2 * 48 + 1];
			tcm_hex(tc.msg + 4, tc.len, hex, sizeof(hex));
			TC_LOG("report parse error #%u len %u: %s\n", tc.parse_errs, tc.len, hex);
		}
		return EINVAL;
	}
	for (unsigned id = 0; id < FT_MAX; id++) {
		uint32_t bit = 1u << id;
		if (!(tc.pseen & bit)) {
			continue;
		}
		palm = palm || tc.pcls[id] == 6u;
		if (tc.pcls[id] < 32u && ((fk.cls >> tc.pcls[id]) & 1u)) {
			raw |= bit;
		}
	}
	if (palm && fk.palm) {
		raw = 0;
	}
	if (!tc.touch_seen) {
		tc.touch_seen = true;
		TC_FACT("first touch report after %llu ms sensing\n", ft_us(now - tc.sense_abs) / 1000u);
	}
	tc.last_touch = now;
	tcm_stats(raw, now);
	all = raw | tc.raw_prev;
	for (unsigned id = 0; id < FT_MAX; id++) {
		uint32_t bit = 1u << id;
		bool down = (raw & bit) != 0, fresh;
		if (!(all & bit)) {
			continue;
		}
		if (fuxi_wake_touch_event(&ft.wake, !deliver, id, down, &fresh)) {
			if (fresh && fuxi_keys_touch_wake()) {
				fuxi_wake_touch_woke(&ft.wake);
			}
			continue;          /* display off, or a contact that woke it: not for iOS */
		}
		mask |= down ? bit : 0u;
	}
	tc.raw_prev = raw;
	if (!deliver) {
		return 0;
	}
	for (unsigned id = 0; id < FT_MAX; id++) {
		uint32_t bit = 1u << id;
		int x, y;
		if (!(mask & bit)) {
			continue;
		}
		tcm_map(tc.px[id], tc.py[id], &x, &y);
		if (!(ft.active & bit) && tc.contact_logs < 8u) {
			tc.contact_logs++;
			TC_FACT("contact %u raw=%u,%u cls=%u -> %d,%d\n", id, tc.px[id], tc.py[id],
			    tc.pcls[id], x, y);
		}
		if (x != ft.x[id] || y != ft.y[id]) {
			ft.x[id] = x;
			ft.y[id] = y;
			moved = true;
		}
	}
	if (fk.dbg & 8u) {
		if (ft_us(now - tc.dbg_win) >= 1000000u) {
			tc.dbg_win = now;
			tc.dbg_reports = 0;
		}
		if (tc.dbg_reports++ < 10u) {
			TC_LOG("report raw=%#x mask=%#x palm=%u\n", raw, mask, palm ? 1u : 0u);
		}
	}
	if (mask != ft.active) {
		tc.motion = false;
		ft_report(mask);
	} else if (moved) {
		tc.motion = true;
	}
	return 0;
}

/*
 * The shared loop's read hook: messages while ATTN stays asserted (at most
 * fuxitdrain), one when the read was forced. An identify while sensing is
 * an IC self-reset (ft.ic_reset). Coalesced movement is reported last.
 */
static int
tcm_poll(bool deliver)
{
	uint64_t now;
	int err = 0;
	tc.deliver = deliver;
	for (unsigned i = 0; i < fk.drain; i++) {
		if (i && !ft_irq_line()) {
			break;
		}
		if ((err = tcm_read_msg())) {
			break;
		}
		if (tc.code == TCM_IDLE || tc.code == TCM_BUSY || tc.code == TCM_CONT) {
			break;
		}
		if (tc.code == TCM_TOUCH) {
			if ((err = tcm_dispatch(deliver))) {
				break;
			}
		} else if (tc.code == TCM_IDENTIFY) {
			tcm_take_identify();
			ft.ic_reset = true;
			break;
		} else {
			tcm_other(tc.code);
		}
	}
	if (tc.motion) {
		tc.motion = false;
		if (deliver) {
			ft_report(ft.active);
		}
	}
	now = mach_absolute_time();
	if (fk.stale && ft.active && ft_us(now - tc.last_touch) >= fk.stale * 1000ull) {
		ft_report(0);
	}
	if (!tc.touch_seen && !tc.quiet_logged && ft_us(now - tc.sense_abs) >= 10000000ull) {
		tc.quiet_logged = true;
		TC_FACT("no touch reports yet after 10 s sensing\n");
	}
	return err;
}

/* fuxitoff=1: deep sleep while the display is off (no tap-to-wake then). */
static void
tcm_display(bool off)
{
	int err;
	if (!tc.sensing) {
		return;
	}
	tc.cmd_reset = false;
	if (off) {
		err = tcm_cmd(TCM_C_SLEEP, NULL, 0, TCM_OK, 1000);
		tc.sleeping = !err;
	} else {
		tc.sleeping = false;
		err = tcm_cmd(TCM_C_WAKE, NULL, 0, TCM_OK, 1000);
	}
	if (tc.cmd_reset) {
		ft.ic_reset = true;   /* IC reset under the command: tcm_resume re-applies the config */
	}
	TC_LOG("deep sleep %s -> %d\n", off ? "enter" : "exit", err);
}

/* Output space (design 4.8): fuxitow/oh, else the boot console, else 1080 x panel height. */
static void
tcm_out_space(void)
{
	PE_Video video;
	const char *src = "knob";
	unsigned w = TC_OUT_W, h = TC_OUT_H;
	bzero(&video, sizeof(video));
	if (!fk.ow || !fk.oh) {
		src = "fallback";
		if (PE_current_console(&video) == 0 && video.v_width >= 320 && video.v_width <= 4096 &&
		    video.v_height >= 320 && video.v_height <= 4096) {
			w = (unsigned)video.v_width;
			h = (unsigned)video.v_height;
			src = "console";
		}
	}
	tc.ow = fk.ow ? fk.ow : w;
	tc.oh = fk.oh ? fk.oh : h;
	TC_FACT("out %ux%u (%s) off %d,%d\n", tc.ow, tc.oh, src, fk.ox, fk.oy);
}

/* X1-X2 for TouchComm: SE clock at the I2C divider, then GPI (SMMU + GPII) or FIFO engine. */
static int
tcm_init_bus(void)
{
	int err;
	ft_i2c_timing();
	if ((err = ft_clock(tc.div, fk.lvl))) {
		return err;
	}
	tc.c0_d2 = (ft.level << 16) | tc.div;
	TC_FACT("clk level=%u dfs_en=%u div=%u t=%u/%u/%u nominal_khz=%u\n", ft.level,
	    ft_rd(ft.gcc, 0x18504) & 1u, tc.div, tc.t_high, tc.t_low, tc.t_cycle, tc.khz);
	if (tc.engine == TC_ENG_GPI) {
		if ((err = ft_se_gsi()) || (err = ft_smmu(ft_smmu_cb())) || (err = ft_gpi_init())) {
			return err;
		}
	} else if ((err = ft_se_fifo())) {
		return err;
	}
	tcm_out_space();
	TC_LOG("config engine=%s addr=%#x clk=%u kHz (div %u, %u/%u/%u) rcfg=%u pwr=%#x irq=%u drain=%u\n",
	    tc.engine == TC_ENG_GPI ? "GPI" : "FIFO", fk.addr, tc.khz, tc.div, tc.t_high, tc.t_low,
	    tc.t_cycle, fk.rcfg, fk.pwr, fk.irq, fk.drain);
	return 0;
}

static void
ft_thread(void *arg, wait_result_t result)
{
	static const uint8_t id_cmd[] = {0xfb, 0x20, 0x00, 0x00, 0x00}, fifo[] = {0x87};
	static const uint8_t sense[] = {0xa0, 0x00, 0x01};
	uint8_t ev[8] = {0}, chip[2] = {0};
	unsigned delay_s = 30, errors = 0;
	unsigned poll_us = 4000, idle_us = 8000, offsup = 1;
	bool irq_seen = false, was_off = false, salami = false;
	(void)arg;
	(void)result;
#if defined(FUXI_SALAMI)
	salami = true;
#endif
	(void)PE_parse_boot_argn("fuxitouchdelay", &delay_s, sizeof(delay_s));
	(void)PE_parse_boot_argn("fuxitouch_poll_us", &poll_us, sizeof(poll_us));
	(void)PE_parse_boot_argn("fuxitouch_idle_us", &idle_us, sizeof(idle_us));
	(void)PE_parse_boot_argn("fuxitouch_offsuppress", &offsup, sizeof(offsup));
	(void)PE_parse_boot_argn("fuxitbatch", &ft.batch, sizeof(ft.batch));
	poll_us = poll_us < 1000 ? 1000 : poll_us;
	idle_us = idle_us < poll_us ? poll_us : idle_us;
	ft_config();
	printf("FUXI TOUCH: starting in %u s (poll %u us active, %u us idle)\n", delay_s,
	    poll_us, idle_us);
	if (ft.batch) {
		printf("FUXI TOUCH: report collections enabled (fuxitbatch=1)\n");
	}
	if (salami && fk.ic == 1u) {
		/* salami has no ST FTS: refused before any register write. */
		printf("FUXI TOUCH: FTS/SPI driver refused on salami (fuxitic=1)\n");
		goto out;
	}
	ft_sleep(delay_s * 1000u);
	ft.gcc = ml_io_map(FT_GCC_PA, 0x1f0000);
	ft.tlmm = ml_io_map(FT_TLMM_PA, 0x30000);
	ft.se = ml_io_map(FT_SE_PA, 0x4000);
	ft.gpi = ml_io_map(FT_GPI_PA, 0x4000);
	ft.smmu = ml_io_map(FT_SMMU_PA, 0x1000);
	if (!ft.gcc || !ft.tlmm || !ft.se || !ft.gpi || !ft.smmu) {
		goto out;
	}
	if (kmem_alloc_contig(kernel_map, &ft.dma, PAGE_SIZE, PAGE_MASK, 0, 0,
	    KMA_DATA | KMA_ZERO | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("FUXI TOUCH: no DMA memory\n");
		goto out;
	}
	ppnum_t pn = pmap_find_phys(kernel_pmap, (addr64_t)ft.dma);
	if (!pn) {
		goto out;
	}
	ft.dma_pa = ptoa_64(pn);
	int ierr;
	/* As before: AHB vote and SE clock (CBCR on) ahead of the first SE read. */
	if ((ierr = ft_ahb()) || (ierr = ft_clock(2, fk.lvl)) || (ierr = ft_detect())) {
		printf("FUXI TOUCH: init failed %d\n", ierr);
		goto out;
	}
	if (salami && !tc.on) {
		/* Only reachable with a protocol-1 SE4: never the FTS path on salami. */
		printf("FUXI TOUCH: FTS/SPI driver refused on salami (SE4 protocol %u)\n", tc.proto);
		goto out;
	}
	if (tc.on) {
		if ((ierr = tcm_init_bus()) || (ierr = tcm_start())) {
			printf("FUXI TOUCH: S3908 init failed %d\n", ierr);
			goto out;
		}
		printf("FUXI TOUCH: sensing, polling GPIO%u\n", tc.pin_attn);
	} else {
		if ((ierr = ft_se_gsi()) || (ierr = ft_smmu(4)) || (ierr = ft_gpi_init())) {
			printf("FUXI TOUCH: init failed %d\n", ierr);
			goto out;
		}
		tc.pin_attn = FT_IRQ;
		printf("FUXI TOUCH: GPI up, DMA page %#llx\n", ft.dma_pa);
		ft_gpio(FT_IRQ, 0, 3, 8, -1);
		ft_gpio(44, 1, 1, 6, -1);
		ft_gpio(45, 1, 1, 6, -1);
		ft_gpio(46, 1, 1, 6, -1);
		ft_gpio(47, 1, 3, 6, -1);
		/* fts_system_reset: reset low 10 ms, high, then poll for controller ready. */
		ft_gpio(FT_RESET, 0, 3, 8, 0);
		ft_sleep(10);
		ft_gpio(FT_RESET, 0, 3, 8, 1);
		int err = EIO;
		for (unsigned t = 0; t < 100; t++) {
			ft_sleep(2);
			if ((err = ft_read(fifo, 1, ev, 8)) || ev[0] == FT_EV_READY) {
				break;
			}
		}
		int cerr = ft_read(id_cmd, 5, chip, 2);
		printf("FUXI TOUCH: reset err=%d event %02x %02x %02x %02x; chip id err=%d %02x%02x\n",
		    err, ev[0], ev[1], ev[2], ev[3], cerr, chip[0], chip[1]);
		if (err || cerr || ev[0] != FT_EV_READY) {
			goto out;
		}
		if ((err = ft_tx(sense, 3, false))) {
			printf("FUXI TOUCH: sense on failed %d\n", err);
			goto out;
		}
		printf("FUXI TOUCH: sensing, polling GPIO%u\n", FT_IRQ);
	}
	/*
	 * IRQ is level low while the FIFO holds events. Check it every poll
	 * period (absolute deadlines, no timer leeway) and read the FIFO every
	 * 32 ms (fuxitsweep) regardless in case the IC does not drive the line.
	 */
	uint64_t poll_abs, idle_abs, off_abs, recent_abs, sweep_abs;
	nanoseconds_to_absolutetime(poll_us * 1000ULL, &poll_abs);
	nanoseconds_to_absolutetime(idle_us * 1000ULL, &idle_abs);
	nanoseconds_to_absolutetime(fk.sweep * NSEC_PER_MSEC, &off_abs);
	nanoseconds_to_absolutetime(fk.sweep * NSEC_PER_MSEC, &sweep_abs);
	nanoseconds_to_absolutetime(fk.recent * NSEC_PER_MSEC, &recent_abs);
	uint64_t next = mach_absolute_time(), last_irq = 0, last_read = 0;
	for (;;) {
		uint64_t now = mach_absolute_time();
		bool off = offsup && sweet_fb_display_off;
		if (off != was_off) {
			was_off = off;
			fuxi_wake_touch_display(&ft.wake, off);
			if (off && ft.active) {
				ft_report(0);   /* release fingers held when the display went off */
			}
			if (tc.on && fk.off) {
				tc.deliver = !off;   /* reports read by the sleep/wake command */
				tcm_display(off);
			}
		}
		bool irq = ft_irq_line();
		if (irq) {
			last_irq = now;
			if (!irq_seen) {
				irq_seen = true;
				printf("FUXI TOUCH: IRQ line asserted\n");
			}
		}
		if (!tc.sleeping && (irq || ft.active || now - last_read >= sweep_abs)) {
			last_read = now;
			int perr = tc.on ? tcm_poll(!off) : ft_poll(!off);
			uint64_t took = mach_absolute_time() - now;
			if (ft.active) {
				ft.c_polls++;
				ft.c_poll_sum += took;
				ft.c_poll_max = took > ft.c_poll_max ? took : ft.c_poll_max;
			}
			if (perr) {
				errors++;
				if (errors == 1 || (errors % 100u) == 0) {
					printf("FUXI TOUCH: FIFO read error %d (%u in a row)\n", perr, errors);
				}
				/* Do not leave a finger stuck down while reads fail. */
				if (errors >= 2 && ft.active) {
					ft_report(0);
				}
				if (errors >= 2) {
					fuxi_wake_touch_reset(&ft.wake);
				}
				/* A timed-out transfer leaves TREs queued: reset the GPII rings. */
				if (errors == 3 || (errors % 50u) == 0) {
					int rerr = ft_bus_reinit();
					printf("FUXI TOUCH: GPI re-init after %u errors -> %d\n", errors, rerr);
				}
				if (tc.on && (errors % 50u) == 0 && (fk.pwr & 0xau)) {
					int berr = tcm_recover();
					if (berr != EBUSY) {
						printf("FUXI TOUCH: S3908 recovery after %u errors -> %d\n", errors, berr);
					}
					errors = berr ? errors : 0;
				}
			} else {
				if (errors >= 3) {
					printf("FUXI TOUCH: FIFO reads recovered after %u errors\n", errors);
				}
				errors = 0;
			}
			if (ft.ic_reset) {
				/* Controller-ready event / identify while sensing: the IC reset itself. */
				ft.ic_reset = false;
				if (ft.active) {
					ft_report(0);
				}
				fuxi_wake_touch_reset(&ft.wake);
				int serr = tc.on ? tcm_resume() : ft_tx(sense, 3, false);
				printf("FUXI TOUCH: controller reset event, sense on again -> %d\n", serr);
				if (serr && tc.on) {
					goto out;
				}
			}
		}
		ft_retry_report();
		uint64_t period = off ? off_abs :
		    (ft.active || now - last_irq < recent_abs) ? poll_abs : idle_abs;
		next += period;
		now = mach_absolute_time();
		if (next <= now) {
			next = now + period;
		}
		ft_sleep_until(next);
	}
out:
	printf("FUXI TOUCH: stopped\n");
	(void)thread_terminate(current_thread());
	thread_block(THREAD_CONTINUE_NULL);
}

void
fuxi_touch_init(void)
{
	int enabled = 1;
	bool salami = false;
	thread_t thread;
	if (ft.started) {
		return;
	}
	ft.started = true;
	/*
	 * salami (FUXI_SALAMI) has a Synaptics S3908 on I2C: the same thread
	 * runs the TouchComm backend. ft_thread refuses the FTS/SPI backend
	 * there before any FTS register write (fuxitic=1 before the start
	 * delay, a protocol-1 SE4 right after detection). No salami image has
	 * made the thread's SMMU/GPII/QUP/GCC/TLMM accesses on hardware yet, so
	 * salami starts it only with boot-arg fuxitouch=1 (default 0). A flag,
	 * not an early return, so nothing becomes unreachable under -Werror.
	 */
#if defined(FUXI_SALAMI)
	salami = true;
	enabled = 0;
#endif
	(void)PE_parse_boot_argn("fuxitouch", &enabled, sizeof(enabled));
	if (salami && !enabled) {
		TC_LOG("touch thread off (fuxitouch=0 is the salami default): no SMMU, GPII, QUP, "
		    "GCC or TLMM access; boot-arg fuxitouch=1 enables it\n");
	}
	if (!enabled) {
		return;
	}
	if (kernel_thread_start_priority(ft_thread, NULL, FT_PRIORITY, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
#endif /* FUXI */
