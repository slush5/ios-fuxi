/* Fuxi Wi-Fi iOS shim entry and synthetic Ethernet link. No radio access.
 * fuxiwifiios=2 (route U) carries that link's frames over the loader's USB monitor.
 * fuxiwifiios=3/4 (K2) list real scan results from the handoff of sys/fuxi_wifi_radio.h, scan only. */
#if defined(FUXI)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/fuxi_wifi_ios.h>
#include <sys/fuxi_wifi_net.h>
#include <sys/fuxi_wifi_radio.h>
#include <sys/fuxi_usbnet_ring.h>
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <pexpert/pexpert.h>
#include <pexpert/device_tree.h>

extern void FuxiNetworkStackStart(void);
extern void thread_set_thread_name(thread_t thread, const char *name);
/* arm/machine_routines.h declares it for osfmk and pexpert only (other bsd/dev units do the same for ml_io_map) */
extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
static const struct fuxi_wifi_backend *backend;
static bool enabled;

static bool
transmit(mbuf_t packet, size_t length)
{
	uint8_t frame[FUXI_WIFI_FRAME_MAX];
	return length >= 14 && length <= sizeof(frame) &&
	       mbuf_copydata(packet, 0, length, frame) == 0 && backend->tx_frame(frame, length);
}

static struct fuxi_wifi_net_link link = {
	.name = "synthetic-loopback", .wireless = true, .transmit = transmit,
};
static const struct fuxi_wifi_input input = {
	.rx_frame = fuxi_wifi_net_input, .link_state = fuxi_wifi_net_link_state,
};

bool
fuxi_wifi_ios_attach(const struct fuxi_wifi_backend *ops)
{
	/* Init-only, before publication. Backend/storage must live until shutdown. */
	if (!enabled || backend != NULL || ops == NULL || ops->tx_frame == NULL || ops->start == NULL ||
	    ops->station.power == NULL || ops->station.scan == NULL || ops->station.heard == NULL ||
	    ops->station.join == NULL || ops->station.leave == NULL || ops->station.state == NULL ||
	    ops->station.channels == NULL) {
		return false;
	}
	backend = ops;
	memcpy(link.address, ops->station.address, sizeof(link.address));
	ops->start(&input);
	fuxi_wifi_sta_attach(&ops->station);
	if (!fuxi_wifi_sta_present() || !fuxi_wifi_net_attach(&link)) {
		printf("FUXI WIFI IOS: attach failed; no retry this boot\n");
		return false;
	}
	FuxiNetworkStackStart();
	return true;
}

/* The synthetic station has one open BSS. Never accept real credentials or
 * claim WPA2 success: there is no access point or firmware key installation.
 */
static LCK_GRP_DECLARE(fake_group, "fuxi-wifi-fake");
static LCK_MTX_DECLARE(fake_lock, &fake_group);
static const struct fuxi_wifi_input *fake_input;
static struct fuxi_wifi_sta_state fake_state;
static bool fake_heard;
static const struct fuxi_wifi_sta_bss fake_bss = {
	.bssid = { 0x02, 0x46, 0x55, 0x58, 0x49, 0x02 },
	.ssid = "Fuxi Loopback", .ssid_length = 13, .frequency = 2412,
	.rssi = -42, .beacon_interval = 100, .capability = 1,
	.elements_length = 4, .elements = { 1, 2, 0x82, 0x84 },
};

static void
fake_start(const struct fuxi_wifi_input *in)
{
	fake_input = in;
	fake_state.phase = FUXI_WIFI_STA_IDLE;
}

static int
fake_power(bool on)
{
	lck_mtx_lock(&fake_lock);
	bzero(&fake_state, sizeof(fake_state));
	fake_state.phase = on ? FUXI_WIFI_STA_IDLE : FUXI_WIFI_STA_OFF;
	fake_heard = false;
	lck_mtx_unlock(&fake_lock);
	fake_input->link_state(false);
	return 0;
}

static int
fake_scan(const struct fuxi_wifi_sta_scan *scan)
{
	bool match = scan->ssid_length == 0 || (scan->ssid_length == fake_bss.ssid_length &&
	    memcmp(scan->ssid, fake_bss.ssid, scan->ssid_length) == 0);
	bool channel = scan->frequency_count == 0;
	for (unsigned i = 0; i < scan->frequency_count; i++) {
		channel |= scan->frequencies[i] == fake_bss.frequency;
	}
	lck_mtx_lock(&fake_lock);
	fake_heard = match && channel;
	lck_mtx_unlock(&fake_lock);
	printf("FUXI WIFI IOS: fake scan complete\n");
	return 0;
}

static void
fake_results(void (*each)(const struct fuxi_wifi_sta_bss *, void *), void *context)
{
	bool have;
	lck_mtx_lock(&fake_lock);
	have = fake_heard;
	lck_mtx_unlock(&fake_lock);
	if (have) {
		each(&fake_bss, context);
	}
}

static int
fake_leave(void)
{
	lck_mtx_lock(&fake_lock);
	bzero(&fake_state, sizeof(fake_state));
	fake_state.phase = FUXI_WIFI_STA_IDLE;
	lck_mtx_unlock(&fake_lock);
	fake_input->link_state(false);
	return 0;
}

static int
fake_join(const struct fuxi_wifi_sta_join *join)
{
	static const uint8_t any[6];
	int error = 0;
	if (join->security != FUXI_WIFI_STA_OPEN) {
		error = EOPNOTSUPP;
	} else if (join->ssid_length != fake_bss.ssid_length ||
	    memcmp(join->ssid, fake_bss.ssid, fake_bss.ssid_length) != 0 ||
	    (memcmp(join->bssid, any, 6) != 0 && memcmp(join->bssid, fake_bss.bssid, 6) != 0)) {
		error = ENOENT;
	}
	lck_mtx_lock(&fake_lock);
	bzero(&fake_state, sizeof(fake_state));
	fake_state.phase = error == 0 ? FUXI_WIFI_STA_JOINED : FUXI_WIFI_STA_IDLE;
	fake_state.last_error = error;
	if (error == 0) {
		fake_state.bss = fake_bss;
	}
	lck_mtx_unlock(&fake_lock);
	fake_input->link_state(error == 0);
	printf("FUXI WIFI IOS: fake connect result %d (no radio)\n", error);
	return error;
}

static void
fake_snapshot(struct fuxi_wifi_sta_state *state)
{
	lck_mtx_lock(&fake_lock);
	*state = fake_state;
	lck_mtx_unlock(&fake_lock);
}

static unsigned
fake_channels(struct fuxi_wifi_sta_channel *out, unsigned most)
{
	if (most == 0) {
		return 0;
	}
	out[0] = (struct fuxi_wifi_sta_channel) { .frequency = 2412 };
	return 1;
}

static bool
fake_tx(const uint8_t *frame, size_t length)
{
	bool up;
	lck_mtx_lock(&fake_lock);
	up = fake_state.phase == FUXI_WIFI_STA_JOINED;
	lck_mtx_unlock(&fake_lock);
	/* Echo only local experimental frames. IP/ARP/EAPOL have no synthetic peer. */
	if (!up || length < 14 || length > FUXI_WIFI_FRAME_MAX || frame[12] != 0x88 || frame[13] != 0xb5) {
		return false;
	}
	fake_input->rx_frame(frame, length);
	printf("FUXI WIFI IOS: fake loopback frame %zu bytes\n", length);
	return true;
}

static const struct fuxi_wifi_backend fake_backend = {
	.station = { .power = fake_power, .scan = fake_scan, .heard = fake_results,
		.join = fake_join, .leave = fake_leave, .state = fake_snapshot, .channels = fake_channels,
		.address = { 0x02, 0x46, 0x55, 0x58, 0x49, 0x01 } },
	.tx_frame = fake_tx, .start = fake_start,
};

/* Route U (fuxiwifiios=2, docs-lanes/wifi-ios.md): the same synthetic station ("Fuxi
 * Loopback" is scanned and joined as above), but en0's frames cross the USB cable. The
 * loader keeps a ring region outside XNU RAM (shim/fuxi-probe/usbnet.h) that its USB
 * monitor shares with a bridge on the host (tools/fuxi_usbnet_ring.py): tx_frame fills the
 * TX ring, one thread drains the RX ring. Only tx_frame and start differ from the synthetic
 * backend; a handoff or region header failure keeps the synthetic loopback. No MMIO: the
 * region is DRAM the loader owns, mapped Normal non-cacheable (sys/fuxi_usbnet_ring.h). */
static LCK_MTX_DECLARE(usbnet_lock, &fake_group);
static uintptr_t usbnet;                        /* the region; 0 = synthetic loopback */
static const struct fuxi_wifi_input *usbnet_up; /* en0's input */
static struct fuxi_wifi_input usbnet_input;     /* the station's: link changes pass usbnet_link */
static struct fuxi_wifi_backend usbnet_backend;
static uint32_t usbnet_kstate;
static uint64_t usbnet_tx_until;                /* a frame went out: poll fast until then */

static bool
usbnet_open(void)
{
	DTEntry chosen;
	const void *value = NULL;
	unsigned int size = 0;
	uint64_t pa = 0;
	const char *why;

	if (SecureDTLookupEntry(NULL, "/chosen", &chosen) != kSuccess ||
	    SecureDTGetProperty(chosen, "fuxi-usbnet", &value, &size) != kSuccess) {
		why = "no /chosen fuxi-usbnet (mkadt-salami.py --usbnet)";
	} else {
		why = fxnet_handoff(value, size, &pa);
	}
	if (why == NULL) {
		/* No soft failure to fall back from: io_map allocates its VA with KMA_NOFAIL and
		 * pmap_map panics on a failed pmap_enter (0096 notes the same); 144 KiB of the
		 * 8 MiB io submap. */
		usbnet = ml_io_map_wcomb((vm_offset_t)pa, FXNET_SIZE);
		why = fxnet_header(usbnet, pa);
	}
	if (why != NULL) {
		printf("FUXI WIFI IOS: usbnet: %s; synthetic loopback instead\n", why);
		usbnet = 0;
		return false;
	}
	fxnet_attach(usbnet);
	printf("FUXI WIFI IOS: usbnet: loader region pa %#llx size %#x, en0 frames over the USB monitor\n",
	    (unsigned long long)pa, FXNET_SIZE);
	return true;
}

/* kstate: bit0 once the RX thread runs, bit1 = en0's link (the loader and the host read it). */
static void
usbnet_set(uint32_t bit, bool on)
{
	lck_mtx_lock(&usbnet_lock);
	usbnet_kstate = on ? usbnet_kstate | bit : usbnet_kstate & ~bit;
	fxnet_kstate(usbnet, usbnet_kstate);
	lck_mtx_unlock(&usbnet_lock);
}

static void
usbnet_link(bool up)
{
	usbnet_set(FXNET_KSTATE_LINK, up);
	usbnet_up->link_state(up);
}

static void
usbnet_start(const struct fuxi_wifi_input *in)
{
	usbnet_up = in;
	usbnet_input.rx_frame = in->rx_frame;
	usbnet_input.link_state = usbnet_link;
	fake_start(&usbnet_input);
}

static bool
usbnet_tx(const uint8_t *frame, size_t length)
{
	uint64_t now, second;
	int put;

	lck_mtx_lock(&usbnet_lock);
	put = fxnet_tx_put(usbnet, frame, length);
	lck_mtx_unlock(&usbnet_lock);
	clock_get_uptime(&now);
	clock_interval_to_absolutetime_interval(1, NSEC_PER_SEC, &second);
	__atomic_store_n(&usbnet_tx_until, now + second, __ATOMIC_RELAXED);
	return put == FXNET_TX_OK;
}

/* The RX thread polls rx_head every 1 ms while frames moved either way in the last second,
 * else every 20 ms: no busy loop and no timer storm when idle (tight polls drain the
 * battery). A frame is copied out of the mapping, handed to en0, then its slot consumed;
 * a bad slot is consumed too and counted (rx_bad). */
static void
usbnet_rx(__unused void *parameter, __unused wait_result_t result)
{
	uint8_t frame[FXNET_FRAME_MAX];
	uint64_t now, second, busy_until = 0;

	clock_interval_to_absolutetime_interval(1, NSEC_PER_SEC, &second);
	usbnet_set(FXNET_KSTATE_UP, true);
	for (;;) {
		size_t length = 0;
		bool moved = false;
		int got;

		for (unsigned int n = 0; n < FXNET_SLOTS; n++) {
			got = fxnet_rx_take(usbnet, frame, &length);
			if (got == FXNET_RX_EMPTY) {
				break;
			}
			if (got == FXNET_RX_OK) {
				usbnet_up->rx_frame(frame, length);
			}
			fxnet_rx_done(usbnet, got == FXNET_RX_OK);
			moved = true;
		}
		clock_get_uptime(&now);
		if (moved) {
			busy_until = now + second;
		}
		(void)assert_wait_timeout((event_t)&usbnet_kstate, THREAD_UNINT,
		    now < busy_until || now < __atomic_load_n(&usbnet_tx_until, __ATOMIC_RELAXED) ? 1 : 20, NSEC_PER_MSEC);
		(void)thread_block(THREAD_CONTINUE_NULL);
	}
}

/* fuxiwifiios=3 (radio) and =4 (canned producer, no radio), K2: what a scan hears comes through the
 * handoff of sys/fuxi_wifi_radio.h, which copies every request and record; this backend never sees MMIO
 * or DMA and never calls the radio lane. Every channel is passive, a join is refused without touching
 * anything (never EAUTH: Settings would blame the password), no frame passes en0 and its link stays
 * down. A radio that stops, or that never reports, turns Wi-Fi off for the boot. */
static LCK_MTX_DECLARE(rscan_backend_lock, &fake_group);
static const struct fuxi_wifi_input *rscan_input;       /* kept, never called: nothing is received */
static int rscan_refused;                               /* EOPNOTSUPP once a join was refused (state()) */
static unsigned rscan_joins;

static void
rscan_start(const struct fuxi_wifi_input *in)
{
	rscan_input = in;
}

static int
rscan_power(bool on)
{
	if (!on) {
		fuxi_wifi_radio_cancel();
		return 0;
	}
	return fuxi_wifi_radio_state(NULL) == FUXI_WIFI_RADIO_STOPPED ? EIO : 0;
}

/* The listed channels wifid asked for (all of them for "any"), passive, 150 ms each unless it asked for
 * 50 to 200; waited for at most 1 s plus one and a half dwells a channel (15 s at most), so that the radio
 * lane, which gives its own scan n * dwell * 3 / 2, always finishes or aborts first. */
static int
rscan_scan(const struct fuxi_wifi_sta_scan *what)
{
	struct fuxi_wifi_radio_request request = { .passive = what->passive };
	struct fuxi_wifi_sta_channel channels[FUXI_WIFI_RADIO_CHANNELS];
	unsigned count = fuxi_wifi_radio_channels(channels, FUXI_WIFI_RADIO_CHANNELS), dwell, timeout;

	for (unsigned i = 0; i < count; i++) {
		bool asked = what->frequency_count == 0;

		for (unsigned j = 0; j < what->frequency_count && j < FUXI_WIFI_RADIO_FREQS_MOST; j++) {
			asked |= what->frequencies[j] == channels[i].frequency;
		}
		if (asked) {
			request.freqs[request.freq_count++] = channels[i].frequency;
		}
	}
	request.ssid_length = what->ssid_length <= sizeof(request.ssid) ? what->ssid_length : 0;
	memcpy(request.ssid, what->ssid, request.ssid_length);
	dwell = what->dwell_ms >= 50 && what->dwell_ms <= 200 ? what->dwell_ms : FUXI_WIFI_RADIO_DWELL_MS;
	request.dwell_ms = (uint16_t)dwell;
	timeout = 1000 + request.freq_count * dwell * 3 / 2;
	return fuxi_wifi_radio_scan(&request, timeout < FUXI_WIFI_RADIO_WAIT_MOST_MS ? timeout : FUXI_WIFI_RADIO_WAIT_MOST_MS,
	           fuxi_wifi_sta_power_pending);
}

static void
rscan_heard(void (*each)(const struct fuxi_wifi_sta_bss *, void *), void *context)
{
	(void)fuxi_wifi_radio_results(each, context);
}

static int
rscan_join(__unused const struct fuxi_wifi_sta_join *what)
{
	unsigned times;

	lck_mtx_lock(&rscan_backend_lock);
	rscan_refused = EOPNOTSUPP;
	times = ++rscan_joins;
	lck_mtx_unlock(&rscan_backend_lock);
	if (times <= 24 || times % 100 == 0) {
		printf("FUXI WIFI IOS: radio: join refused (scan-only backend), no radio access\n");
	}
	return EOPNOTSUPP;
}

static int
rscan_leave(void)
{
	return 0;
}

static void
rscan_snapshot(struct fuxi_wifi_sta_state *state)
{
	bool scanning;
	enum fuxi_wifi_radio_state radio = fuxi_wifi_radio_state(&scanning);

	bzero(state, sizeof(*state));
	state->phase = radio == FUXI_WIFI_RADIO_STOPPED ? FUXI_WIFI_STA_OFF : scanning ? FUXI_WIFI_STA_SCANNING :
	    FUXI_WIFI_STA_IDLE;
	lck_mtx_lock(&rscan_backend_lock);
	state->last_error = rscan_refused;
	lck_mtx_unlock(&rscan_backend_lock);
}

static bool
rscan_tx(__unused const uint8_t *frame, __unused size_t length)
{
	return false;
}

/* thread call context (fuxi_wifi_radio.c's notify), after attach */
static void
rscan_changed(enum fuxi_wifi_radio_state state)
{
	if (state == FUXI_WIFI_RADIO_STOPPED) {
		fuxi_wifi_sta_radio_off();
	}
}

static const struct fuxi_wifi_backend rscan_backend = {
	.station = { .power = rscan_power, .scan = rscan_scan, .heard = rscan_heard, .join = rscan_join,
		.leave = rscan_leave, .state = rscan_snapshot, .channels = fuxi_wifi_radio_channels,
		.address = { 0x02, 0x46, 0x55, 0x58, 0x49, 0x01 } },
	.tx_frame = rscan_tx, .start = rscan_start,
};

static void
rscan_init(bool canned)
{
	printf("FUXI WIFI IOS: enabled, %s; Apple80211 16.7 ABI unverified\n", canned ? "canned scan backend (no radio)" :
	    "radio scan backend (scan only, no join)");
	link.name = "radio-scan";
	fuxi_wifi_apple80211_backend(canned ? "Fuxi Wi-Fi iOS shim (canned scan backend)" :
	    "Fuxi Wi-Fi iOS shim (radio backend, scan only)", true);
	if (!fuxi_wifi_radio_use(canned, rscan_changed)) {
		printf("FUXI WIFI IOS: radio: no memory or thread call for the scan handoff; no retry this boot\n");
		return;
	}
	if (fuxi_wifi_ios_attach(&rscan_backend)) {
		fuxi_wifi_radio_attached();
	}
}

void
fuxi_wifi_ios_init(void)
{
	const struct fuxi_wifi_backend *ops = &fake_backend;
	thread_t thread;
	int on = 0;
	PE_parse_boot_argn("fuxiwifiios", &on, sizeof(on));
	if (on < 1 || on > 4 || enabled) {
		return;
	}
	enabled = true;
	if (on >= 3) {
		rscan_init(on == 4);
		return;
	}
	if (on == 2 && usbnet_open()) {
		usbnet_backend = fake_backend;
		usbnet_backend.tx_frame = usbnet_tx;
		usbnet_backend.start = usbnet_start;
		ops = &usbnet_backend;
	}
	printf("FUXI WIFI IOS: enabled, %s backend; Apple80211 16.7 ABI unverified\n",
	    ops == &fake_backend ? "synthetic" : "usbnet");
	if (!fuxi_wifi_ios_attach(ops) || ops == &fake_backend) {
		return;
	}
	if (kernel_thread_start(usbnet_rx, NULL, &thread) != KERN_SUCCESS) {
		printf("FUXI WIFI IOS: usbnet: no RX thread; kstate stays 0 and the loader drops host frames\n");
		return;
	}
	thread_set_thread_name(thread, "wifi-usbnet-rx");
	thread_deallocate(thread);
}
#endif /* FUXI */
