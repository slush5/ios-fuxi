/*
 * Fuxi Wi-Fi iOS shim: the scan handoff (sys/fuxi_wifi_radio.h). Original Fuxi code, 2026-10-30 (K2).
 *
 * What stands between a radio that scans and the iOS side that shows what it heard:
 *   - the state word, the request mailbox and the two tables' pointers, under radio_lock. It is a leaf
 *     lock: while it is held only plain loads and stores and memcpy/bzero/memcmp happen, a wait is only
 *     asserted (assert_wait_timeout, then unlock, then thread_block) or slept through lck_mtx_sleep, which
 *     drops it, and helpers called under it end in _locked (tools/test-fuxi-wifi-radio.py T6 checks);
 *   - the back table, the producer's alone from take() to done(); done() swaps it to the front;
 *   - the front table, read by the consumer one record at a time, each copied under the lock; a reader
 *     stops when the generation moves (a swap, a stop, a failed scan);
 *   - one thread call (radio_notify) that tells the consumer READY and STOPPED, entered after unlock;
 *   - the canned producer of fuxiwifiios=4: four made-up beacons through the very same path.
 * The tables are allocated by use() only (fuxiwifiios=3/4) and never freed. They never point at radio
 * DMA, so the radio lane's containment and quarantine (wl5_abort) can leave nothing dangling here.
 *
 * 802.11 frame and element layouts are facts of the standard. The passive dwell (150 ms) and the noise
 * floor used when there is none (-95 dBm) are Linux ath12k's values (BSD-3-Clause-Clear, v6.19 wmi.c
 * and core.h), cited, not copied. The log carries counts, bands, flags and errno only: never an SSID,
 * a BSSID or an element byte.
 */
#if defined(FUXI)
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/errno.h>
#include <sys/fuxi_wifi_radio.h>
#include <sys/systm.h>

extern void thread_set_thread_name(thread_t thread, const char *name);

/* ---- pure: the channel list, a frame into a record, elements into a room ------------------------------ */

#define RADIO_P         FUXI_WIFI_STA_CHANNEL_PASSIVE
#define RADIO_PR        (FUXI_WIFI_STA_CHANNEL_PASSIVE | FUXI_WIFI_STA_CHANNEL_RADAR)
/* Every channel passive (nothing is transmitted, so no regulatory domain is needed for it); no 6 GHz. */
static const struct fuxi_wifi_sta_channel radio_list[] = {
	{ 2412, RADIO_P }, { 2417, RADIO_P }, { 2422, RADIO_P }, { 2427, RADIO_P }, { 2432, RADIO_P },
	{ 2437, RADIO_P }, { 2442, RADIO_P }, { 2447, RADIO_P }, { 2452, RADIO_P }, { 2457, RADIO_P },
	{ 2462, RADIO_P }, { 2467, RADIO_P }, { 2472, RADIO_P },
	{ 5180, RADIO_P }, { 5200, RADIO_P }, { 5220, RADIO_P }, { 5240, RADIO_P },
	{ 5260, RADIO_PR }, { 5280, RADIO_PR }, { 5300, RADIO_PR }, { 5320, RADIO_PR },
	{ 5500, RADIO_PR }, { 5520, RADIO_PR }, { 5540, RADIO_PR }, { 5560, RADIO_PR }, { 5580, RADIO_PR },
	{ 5600, RADIO_PR }, { 5620, RADIO_PR }, { 5640, RADIO_PR }, { 5660, RADIO_PR }, { 5680, RADIO_PR },
	{ 5700, RADIO_PR }, { 5720, RADIO_PR },
	{ 5745, RADIO_P }, { 5765, RADIO_P }, { 5785, RADIO_P }, { 5805, RADIO_P }, { 5825, RADIO_P },
};
#undef RADIO_P
#undef RADIO_PR
_Static_assert(sizeof(radio_list) / sizeof(radio_list[0]) == FUXI_WIFI_RADIO_CHANNELS, "the 38 passive channels");

unsigned
fuxi_wifi_radio_channels(struct fuxi_wifi_sta_channel *out, unsigned most)
{
	unsigned count = most < FUXI_WIFI_RADIO_CHANNELS ? most : FUXI_WIFI_RADIO_CHANNELS;

	if (out == NULL) {
		return 0;
	}
	memcpy(out, radio_list, count * sizeof(*out));
	return count;
}

static int16_t
radio_dbm(int16_t dbm)
{
	return dbm < -127 ? -127 : dbm > 0 ? 0 : dbm;
}

static bool
radio_listed(unsigned frequency, const uint16_t *allowed, unsigned count)
{
	for (unsigned i = 0; allowed != NULL && i < count; i++) {
		if (allowed[i] == frequency) {
			return true;
		}
	}
	return false;
}

/*
 * Which elements are kept first when not all fit: the priority set of ios_scan_plan 1.3, with what shows a
 * network and its lock ahead of the rest of it. 0: the SSID, RSN and WPA (OUI 00:50:f2 type 1); 1: rates,
 * DS channel, country, BSS load, HT, extended rates, mobility domain, HT operation, extended capabilities,
 * VHT, WMM (00:50:f2 type 2), HE capabilities and operation (extension 35, 36); 2: any other.
 */
static unsigned
element_rank(const uint8_t *element)
{
	bool microsoft = element[1] >= 4 && element[2] == 0x00 && element[3] == 0x50 && element[4] == 0xf2;

	switch (element[0]) {
	case 0: case 48:
		return 0;
	case 1: case 3: case 7: case 11: case 45: case 50: case 54: case 61: case 127: case 191: case 192:
		return 1;
	case 221:
		return microsoft && element[5] == 1 ? 0 : microsoft && element[5] == 2 ? 1 : 2;
	case 255:
		return element[1] >= 1 && (element[2] == 35 || element[2] == 36) ? 1 : 2;
	default:
		return 2;
	}
}

unsigned
fuxi_wifi_elements_fit(const uint8_t *in, unsigned length, uint8_t *out, unsigned room)
{
	unsigned at, size, whole = 0, put = 0, kept[2] = { 0, 0 }, used[3] = { 0, 0, 0 };

	if (in == NULL || out == NULL) {
		return 0;
	}
	/* the whole elements: one that runs past the end ends the walk */
	for (at = 0; at + 2 <= length && at + 2 + in[at + 1] <= length; at += 2u + in[at + 1]) {
		whole = at + 2u + in[at + 1];
	}
	if (whole <= room) {
		memcpy(out, in, whole);
		return whole;
	}
	/* rank 0, then rank 1 in what rank 0 left: in their order, each that still fits */
	for (unsigned rank = 0; rank < 2; rank++) {
		for (at = 0; at < whole; at += size) {
			size = 2u + in[at + 1];
			if (element_rank(in + at) == rank && (rank == 0 ? 0 : kept[0]) + kept[rank] + size <= room) {
				kept[rank] += size;
			}
		}
	}
	/* in their order: ranks 0 and 1 as chosen above, then any other that still fits; none is ever cut */
	for (at = 0; at < whole; at += size) {
		unsigned rank = element_rank(in + at), before = rank == 0 ? 0 : rank == 1 ? kept[0] : kept[0] + kept[1];

		size = 2u + in[at + 1];
		if (before + used[rank] + size <= room) {
			used[rank] += size;
			memcpy(out + put, in + at, size);
			put += size;
		}
	}
	return put;
}

int
fuxi_wifi_radio_parse(const uint8_t *frame, size_t length, uint16_t rx_mhz, int16_t rssi_dbm, int16_t noise_dbm,
    const uint16_t *allowed, unsigned allowed_count, struct fuxi_wifi_sta_bss *out)
{
	static const uint8_t zero[6];
	const uint8_t *ssid = NULL;
	unsigned header, end, at, ssid_length = 0, frequency = rx_mhz;
	int ds = -1, primary = -1;
	bool low, named = false;

	if (frame == NULL || out == NULL || length < 24 + 12 || length > 0xffff) {
		return FUXI_WIFI_RADIO_DROP_LENGTH;
	}
	/* a management frame, beacon (0x80) or probe response (0x50); no DS bits, not protected */
	if ((frame[0] != 0x80 && frame[0] != 0x50) || (frame[1] & 0x43) != 0) {
		return FUXI_WIFI_RADIO_DROP_KIND;
	}
	header = frame[1] & 0x80 ? 28 : 24;             /* (the order bit: an HT control field follows) */
	end = (unsigned)length;
	if (end < header + 12) {
		return FUXI_WIFI_RADIO_DROP_LENGTH;
	}
	if ((frame[16] & 1) != 0 || memcmp(frame + 16, zero, sizeof(zero)) == 0) {         /* address 3 */
		return FUXI_WIFI_RADIO_DROP_BSSID;
	}
	if ((frame[header + 10] & 1) == 0) {            /* capability: ESS */
		return FUXI_WIFI_RADIO_DROP_IBSS;
	}
	for (at = header + 12; at + 2 <= end && at + 2 + frame[at + 1] <= end; at += 2u + frame[at + 1]) {
		if (frame[at] == 0 && ssid == NULL) {
			ssid = frame + at + 2;
			ssid_length = frame[at + 1];
		} else if (frame[at] == 3 && frame[at + 1] >= 1 && ds < 0) {
			ds = frame[at + 2];
		} else if (frame[at] == 61 && frame[at + 1] >= 1 && primary < 0) {
			primary = frame[at + 2];
		}
	}
	if (ssid == NULL || ssid_length > sizeof(out->ssid)) {
		return FUXI_WIFI_RADIO_DROP_SSID;
	}
	low = rx_mhz >= 2400 && rx_mhz < 2500;
	if (!low && !(rx_mhz >= 4900 && rx_mhz <= 5895)) {
		return FUXI_WIFI_RADIO_DROP_BAND;
	}
	/* the BSS's own channel: a 2.4 GHz beacon is often heard a channel or two away from it */
	if (low && ds >= 0) {
		if (ds < 1 || ds > 14) {
			return FUXI_WIFI_RADIO_DROP_CHANNEL;
		}
		frequency = ds == 14 ? 2484 : 2407 + 5 * (unsigned)ds;
	} else if (!low && primary >= 0) {
		if (primary < 1 || primary > 179) {     /* (179: 5895 MHz, the top of 5 GHz) */
			return FUXI_WIFI_RADIO_DROP_CHANNEL;
		}
		frequency = 5000 + 5 * (unsigned)primary;
	}
	if (!radio_listed(frequency, allowed, allowed_count)) {
		return FUXI_WIFI_RADIO_DROP_UNLISTED;
	}
	bzero(out, sizeof(*out));
	memcpy(out->bssid, frame + 16, sizeof(out->bssid));
	for (unsigned i = 0; i < ssid_length; i++) {
		named |= ssid[i] != 0;
	}
	if (named) {                                    /* (all zero bytes: a hidden network, as an empty name) */
		out->ssid_length = (uint8_t)ssid_length;
		memcpy(out->ssid, ssid, ssid_length);
	}
	out->frequency = (uint16_t)frequency;
	out->rssi = radio_dbm(rssi_dbm);
	out->noise = radio_dbm(noise_dbm);
	out->beacon_interval = (uint16_t)(frame[header + 8] | frame[header + 9] << 8);
	out->capability = (uint16_t)(frame[header + 10] | frame[header + 11] << 8);
	out->elements_length = (uint16_t)fuxi_wifi_elements_fit(frame + header + 12, end - header - 12, out->elements,
	    sizeof(out->elements));
	return FUXI_WIFI_RADIO_KEPT;
}

/* ---- the shared state --------------------------------------------------------------------------------- */

#define RADIO_POLL_MS           10      /* a waiting scan looks at itself (and at a power-off asked for) this often */
#define RADIO_SILENT_S          30      /* fuxiwifiios=3: a producer still silent this long after attach is not there */
#define RADIO_TIMEOUTS_MOST     3       /* scans timed out in a row before the iOS side gives up */
#define RADIO_SAID_MOST         24      /* scans said in the log one by one; after that every hundredth */

static LCK_GRP_DECLARE(radio_lock_group, "fuxi-wifi-radio");
static LCK_MTX_DECLARE(radio_lock, &radio_lock_group);

struct radio_table {
	unsigned count;
	unsigned frames, dropped[FUXI_WIFI_RADIO_DROP_REASONS];  /* handed over for this scan, and why some were not kept */
	unsigned at_24, at_5, rsn, hidden;                       /* made before done() publishes it (the scan's line) */
	uint64_t heard_at[FUXI_WIFI_RADIO_BSS_MOST];             /* when each record's last frame came (absolute time) */
	struct fuxi_wifi_sta_bss bss[FUXI_WIFI_RADIO_BSS_MOST];
};

/* the state word: the one thing a producer call stores before use() */
static enum fuxi_wifi_radio_state radio_state;
static unsigned radio_code;                                     /* the first stop code */
static uint16_t radio_freqs[FUXI_WIFI_RADIO_FREQS_MOST];        /* the listed channels ready() named: fixed from READY on */
static unsigned radio_freq_count, radio_freq_low;               /* (how many of them at 2.4 GHz) */
/* the consumer, from use() on */
static bool radio_used, radio_canned, radio_attached;
static void (*radio_changed)(enum fuxi_wifi_radio_state state);
static thread_call_t radio_call;                                /* radio_notify */
static enum fuxi_wifi_radio_state radio_told;                   /* the last state radio_notify passed on */
static uint64_t radio_silent_at;                                /* the backstop's deadline (fuxiwifiios=3) */
/* one request at a time */
static struct fuxi_wifi_radio_request radio_request;
static uint32_t radio_seq;                                      /* the last one posted */
static uint32_t radio_live;                                     /* the one in flight, 0: none (cancelled(seq): seq != this) */
static bool radio_taken;                                        /* the producer has it */
static uint32_t radio_done_seq;                                 /* the last one the producer finished, and how */
static int radio_done_status;
static unsigned radio_timeouts, radio_scans;
/* the tables, and one record of scratch for each side */
static struct radio_table *radio_front, *radio_back;
static uint32_t radio_generation;
static struct fuxi_wifi_sta_bss *radio_parsed;                  /* the producer's */
static struct fuxi_wifi_sta_bss *radio_copy;                    /* the consumer's: the record each() is given */

/* STOPPED, once: the request in flight ends (its waiter sees the state), nothing heard stays; the call to enter. */
static thread_call_t
radio_stop_locked(unsigned code)
{
	if (radio_state == FUXI_WIFI_RADIO_STOPPED) {
		return NULL;
	}
	radio_state = FUXI_WIFI_RADIO_STOPPED;
	radio_code = code;
	radio_live = 0;
	if (radio_front != NULL) {
		radio_front->count = 0;
	}
	radio_generation++;
	return radio_used ? radio_call : NULL;
}

/* Nothing heard (a scan that failed, or that asked the radio for nothing). */
static void
radio_forget_locked(void)
{
	if (radio_front != NULL) {
		radio_front->count = 0;
	}
	radio_generation++;
}

/* The producer's table for the request `seq`, if that is the one in flight and taken; else NULL. */
static struct radio_table *
radio_open_locked(uint32_t seq, bool lane)
{
	if ((lane && radio_canned) || seq == 0 || seq != radio_live || !radio_taken ||
	    radio_state != FUXI_WIFI_RADIO_READY) {
		return NULL;
	}
	return radio_back;
}

/*
 * fuxiwifiios=3: iOS asks the station something (a scan, its state) and no producer has said anything.
 * Patch 0099 calls fuxi_wlan_init from the root device probe, before launchd starts whatever asks, and
 * (C3) that reports STARTING or STOPPED on every boot, so a radio lane still silent by then is not in
 * this kernel.
 */
static thread_call_t
radio_asked_locked(void)
{
	if (radio_attached && !radio_canned && radio_state == FUXI_WIFI_RADIO_ABSENT) {
		return radio_stop_locked(FUXI_WIFI_RADIO_NO_PRODUCER | 1);
	}
	return NULL;
}

static const char *
radio_why(unsigned code, bool canned_producer)
{
	switch (code >> 16) {
	case FUXI_WIFI_RADIO_NO_PRODUCER >> 16:
		return (code & 0xffff) == 2 ? "no radio lane reported within 30 s of attach" :
		       "no radio lane reported before iOS first asked";
	case FUXI_WIFI_RADIO_GAVE_UP >> 16:
		return "scans timed out 3 times in a row";
	case FUXI_WIFI_RADIO_NO_CHANNEL >> 16:
		return "the radio lane named no usable channel";
	default:
		return canned_producer ? "the canned producer stopped" : "the radio lane stopped";
	}
}

/*
 * The consumer's side of READY and STOPPED, once each, after attach (thread call context: it may block).
 * With fuxiwifiios=3 it is also the backstop: a producer still ABSENT RADIO_SILENT_S after attach is none.
 */
static void
radio_notify(__unused thread_call_param_t zero, __unused thread_call_param_t one)
{
	void (*changed)(enum fuxi_wifi_radio_state state);
	enum fuxi_wifi_radio_state state;
	thread_call_t call;
	uint64_t now, silent_at = 0;
	unsigned code, count, low;
	bool canned, tell, again;

	clock_get_uptime(&now);
	lck_mtx_lock(&radio_lock);
	if (radio_attached && !radio_canned && radio_state == FUXI_WIFI_RADIO_ABSENT) {
		if (now >= radio_silent_at) {
			(void)radio_stop_locked(FUXI_WIFI_RADIO_NO_PRODUCER | 2);
		} else {
			silent_at = radio_silent_at;
		}
	}
	state = radio_state;
	tell = radio_attached && state != radio_told && (state == FUXI_WIFI_RADIO_READY || state == FUXI_WIFI_RADIO_STOPPED);
	if (tell) {
		radio_told = state;
	}
	code = radio_code;
	count = radio_freq_count;
	low = radio_freq_low;
	canned = radio_canned;
	changed = radio_changed;
	call = radio_call;
	lck_mtx_unlock(&radio_lock);
	if (silent_at != 0) {
		(void)thread_call_enter_delayed(call, silent_at);
		/* a READY or STOPPED since the look above entered this call while it ran, and on XNU this re-arm replaced
		 * that entry (the last one wins): enter it again, so that it is told now and not at the deadline */
		lck_mtx_lock(&radio_lock);
		again = radio_state != FUXI_WIFI_RADIO_ABSENT;
		lck_mtx_unlock(&radio_lock);
		if (again) {
			(void)thread_call_enter(call);
		}
	}
	if (!tell) {
		return;
	}
	if (state == FUXI_WIFI_RADIO_READY) {
		printf("FUXI WIFI IOS: radio: %s producer ready, %u channels (%u at 2.4 GHz, %u at 5 GHz), passive only\n",
		    canned ? "canned" : "radio", count, low, count - low);
	} else {
		printf("FUXI WIFI IOS: radio: %s (code %#x); Wi-Fi reported off, no retry this boot\n", radio_why(code, canned),
		    code);
	}
	changed(state);
}

/* ---- the producer --------------------------------------------------------------------------------------
 * The radio lane calls the public functions (lane true: ignored with fuxiwifiios=4); the canned producer
 * calls the same code with lane false. One producer thread at a time. */

static void
radio_starting(bool lane)
{
	lck_mtx_lock(&radio_lock);
	if (!(lane && radio_canned) && radio_state == FUXI_WIFI_RADIO_ABSENT) {
		radio_state = FUXI_WIFI_RADIO_STARTING;
	}
	lck_mtx_unlock(&radio_lock);
}

static void
radio_ready(const uint16_t *freqs, unsigned count, bool lane)
{
	uint16_t usable[FUXI_WIFI_RADIO_FREQS_MOST];
	unsigned n = 0, low = 0;
	thread_call_t call = NULL;

	/* only the listed channels, each once */
	for (unsigned i = 0; freqs != NULL && i < count && n < FUXI_WIFI_RADIO_FREQS_MOST; i++) {
		bool listed = false, again = false;

		for (unsigned j = 0; j < FUXI_WIFI_RADIO_CHANNELS; j++) {
			listed |= radio_list[j].frequency == freqs[i];
		}
		for (unsigned j = 0; j < n; j++) {
			again |= usable[j] == freqs[i];
		}
		if (listed && !again) {
			low += freqs[i] < 4000;
			usable[n++] = freqs[i];
		}
	}
	lck_mtx_lock(&radio_lock);
	if (!(lane && radio_canned) && (radio_state == FUXI_WIFI_RADIO_ABSENT || radio_state == FUXI_WIFI_RADIO_STARTING)) {
		if (n == 0) {
			/* nothing a scan could ask for (channel numbers for MHz, 6 GHz only): off with the cause, not on and empty */
			call = radio_stop_locked(FUXI_WIFI_RADIO_NO_CHANNEL | (count < 0xffff ? count : 0xffff));
		} else {
			memcpy(radio_freqs, usable, n * sizeof(usable[0]));
			radio_freq_count = n;
			radio_freq_low = low;
			radio_state = FUXI_WIFI_RADIO_READY;
			call = radio_used ? radio_call : NULL;
		}
	}
	lck_mtx_unlock(&radio_lock);
	if (call != NULL) {
		(void)thread_call_enter(call);
	}
}

static struct radio_table *
radio_take_locked(struct fuxi_wifi_radio_request *out, bool lane)
{
	if ((lane && radio_canned) || radio_state != FUXI_WIFI_RADIO_READY || radio_live == 0 || radio_taken) {
		return NULL;
	}
	memcpy(out, &radio_request, sizeof(*out));
	radio_taken = true;
	return radio_back;
}

/* The producer's own table, emptied for the scan it has just taken (no lock: nobody else touches it). */
static void
radio_reset(struct radio_table *table)
{
	table->count = table->frames = 0;
	table->at_24 = table->at_5 = table->rsn = table->hidden = 0;
	bzero(table->dropped, sizeof(table->dropped));
}

static bool
radio_cancelled(uint32_t seq, bool lane)
{
	bool cancelled;

	lck_mtx_lock(&radio_lock);
	cancelled = radio_open_locked(seq, lane) == NULL;
	lck_mtx_unlock(&radio_lock);
	return cancelled;
}

static int
radio_count(struct radio_table *table, int why)
{
	table->frames++;
	if (why > 0 && why < FUXI_WIFI_RADIO_DROP_REASONS) {
		table->dropped[why]++;
	}
	return why;
}

/* Into the producer's table, keyed by BSSID. Full: a stronger one replaces the weakest, a weaker one is dropped. */
static int
radio_keep(struct radio_table *table, const struct fuxi_wifi_sta_bss *bss)
{
	unsigned i, weakest = 0;
	uint64_t now;

	clock_get_uptime(&now);
	for (i = 0; i < table->count && memcmp(table->bss[i].bssid, bss->bssid, sizeof(bss->bssid)) != 0; i++) {
	}
	if (i < table->count) {
		/* heard again: the newer frame wins, but a hidden beacon keeps the name a probe response gave */
		struct fuxi_wifi_sta_bss *old = &table->bss[i];
		uint8_t name[sizeof(old->ssid)], name_length = old->ssid_length;

		memcpy(name, old->ssid, sizeof(name));
		*old = *bss;
		if (bss->ssid_length == 0 && name_length != 0) {
			memcpy(old->ssid, name, sizeof(name));
			old->ssid_length = name_length;
		}
	} else if (table->count < FUXI_WIFI_RADIO_BSS_MOST) {
		table->bss[table->count++] = *bss;
	} else {
		for (unsigned j = 1; j < table->count; j++) {
			weakest = table->bss[j].rssi < table->bss[weakest].rssi ? j : weakest;
		}
		if (bss->rssi <= table->bss[weakest].rssi) {
			return FUXI_WIFI_RADIO_DROP_FULL;
		}
		i = weakest;
		table->bss[i] = *bss;
	}
	table->heard_at[i] = now;
	return FUXI_WIFI_RADIO_KEPT;
}

static int
radio_frame(uint32_t seq, const uint8_t *frame, size_t length, uint16_t rx_mhz, int16_t rssi_dbm, int16_t noise_dbm,
    bool lane)
{
	struct radio_table *table;
	int why;

	lck_mtx_lock(&radio_lock);
	table = radio_open_locked(seq, lane);
	lck_mtx_unlock(&radio_lock);
	if (table == NULL) {
		return FUXI_WIFI_RADIO_DROP_STALE;
	}
	/* (radio_freqs is fixed once READY was seen under the lock; radio_parsed is the producer's) */
	why = fuxi_wifi_radio_parse(frame, length, rx_mhz, rssi_dbm, noise_dbm, radio_freqs, radio_freq_count, radio_parsed);
	return radio_count(table, why == FUXI_WIFI_RADIO_KEPT ? radio_keep(table, radio_parsed) : why);
}

static int
radio_bss(uint32_t seq, const struct fuxi_wifi_sta_bss *bss, bool lane)
{
	static const uint8_t zero[6];
	struct fuxi_wifi_sta_bss *out = radio_parsed;
	struct radio_table *table;
	bool named = false;

	lck_mtx_lock(&radio_lock);
	table = radio_open_locked(seq, lane);
	lck_mtx_unlock(&radio_lock);
	if (table == NULL) {
		return FUXI_WIFI_RADIO_DROP_STALE;
	}
	/* the same rules as a parsed frame's */
	if (bss == NULL || bss->elements_length > sizeof(bss->elements)) {
		return radio_count(table, FUXI_WIFI_RADIO_DROP_LENGTH);
	}
	if ((bss->bssid[0] & 1) != 0 || memcmp(bss->bssid, zero, sizeof(zero)) == 0) {
		return radio_count(table, FUXI_WIFI_RADIO_DROP_BSSID);
	}
	if ((bss->capability & 1) == 0) {
		return radio_count(table, FUXI_WIFI_RADIO_DROP_IBSS);
	}
	if (bss->ssid_length > sizeof(bss->ssid)) {
		return radio_count(table, FUXI_WIFI_RADIO_DROP_SSID);
	}
	if (!((bss->frequency >= 2400 && bss->frequency < 2500) || (bss->frequency >= 4900 && bss->frequency <= 5895))) {
		return radio_count(table, FUXI_WIFI_RADIO_DROP_BAND);
	}
	if (!radio_listed(bss->frequency, radio_freqs, radio_freq_count)) {
		return radio_count(table, FUXI_WIFI_RADIO_DROP_UNLISTED);
	}
	bzero(out, sizeof(*out));
	memcpy(out->bssid, bss->bssid, sizeof(out->bssid));
	for (unsigned i = 0; i < bss->ssid_length; i++) {
		named |= bss->ssid[i] != 0;
	}
	if (named) {
		out->ssid_length = bss->ssid_length;
		memcpy(out->ssid, bss->ssid, bss->ssid_length);
	}
	out->frequency = bss->frequency;
	out->rssi = radio_dbm(bss->rssi);
	out->noise = radio_dbm(bss->noise);
	out->beacon_interval = bss->beacon_interval;
	out->capability = bss->capability;
	out->elements_length = (uint16_t)fuxi_wifi_elements_fit(bss->elements, bss->elements_length, out->elements,
	    sizeof(out->elements));
	return radio_count(table, radio_keep(table, out));
}

/* The scan line's counts, made on the producer's own table before done() publishes it. */
static void
radio_summarize(struct radio_table *table)
{
	table->at_24 = table->at_5 = table->rsn = table->hidden = 0;
	for (unsigned i = 0; i < table->count; i++) {
		const struct fuxi_wifi_sta_bss *bss = &table->bss[i];
		bool rsn = false;

		for (unsigned at = 0; at + 2 <= bss->elements_length && at + 2 + bss->elements[at + 1] <= bss->elements_length;
		    at += 2u + bss->elements[at + 1]) {
			rsn |= bss->elements[at] == 48;
		}
		table->at_24 += bss->frequency < 4000;
		table->at_5 += bss->frequency >= 4000;
		table->rsn += rsn;
		table->hidden += bss->ssid_length == 0;
	}
}

/* The request in flight is over: what was heard becomes the front (status 0), and its waiter is told. */
static void
radio_published_locked(uint32_t seq, int status, struct radio_table *table)
{
	if (status == 0) {
		radio_back = radio_front;
		radio_front = table;
		radio_generation++;
	}
	radio_done_seq = seq;
	radio_done_status = status;
	radio_live = 0;
}

static void
radio_done(uint32_t seq, int status, bool lane)
{
	struct radio_table *table, *still;

	lck_mtx_lock(&radio_lock);
	table = radio_open_locked(seq, lane);
	lck_mtx_unlock(&radio_lock);
	if (table == NULL) {
		return;
	}
	radio_summarize(table);
	lck_mtx_lock(&radio_lock);
	still = radio_open_locked(seq, lane);
	if (still == table) {
		radio_published_locked(seq, status < 0 ? EIO : status, table);
	}
	lck_mtx_unlock(&radio_lock);
	if (still == table) {
		thread_wakeup((event_t)&radio_done_seq);
	}
}

void
fuxi_wifi_radio_starting(void)
{
	radio_starting(true);
}

void
fuxi_wifi_radio_ready(const uint16_t *freqs, unsigned count)
{
	radio_ready(freqs, count, true);
}

bool
fuxi_wifi_radio_take(struct fuxi_wifi_radio_request *out)
{
	struct radio_table *table;

	if (out == NULL) {
		return false;
	}
	lck_mtx_lock(&radio_lock);
	table = radio_take_locked(out, true);
	lck_mtx_unlock(&radio_lock);
	if (table != NULL) {
		radio_reset(table);
	}
	return table != NULL;
}

bool
fuxi_wifi_radio_cancelled(uint32_t seq)
{
	return radio_cancelled(seq, true);
}

/*
 * Whether a radio lane that serves requests should go on: false once STOPPED, which is terminal (the iOS side
 * gave up after 3 timeouts, ready() named no usable channel, or a stop was reported). Always true with
 * fuxiwifiios=4, where the lane's calls are ignored anyway.
 */
bool
fuxi_wifi_radio_wanted(void)
{
	bool wanted;

	lck_mtx_lock(&radio_lock);
	wanted = radio_canned || radio_state != FUXI_WIFI_RADIO_STOPPED;
	lck_mtx_unlock(&radio_lock);
	return wanted;
}

int
fuxi_wifi_radio_frame(uint32_t seq, const uint8_t *frame, size_t length, uint16_t rx_mhz, int16_t rssi_dbm,
    int16_t noise_dbm)
{
	return radio_frame(seq, frame, length, rx_mhz, rssi_dbm, noise_dbm, true);
}

int
fuxi_wifi_radio_bss(uint32_t seq, const struct fuxi_wifi_sta_bss *bss)
{
	return radio_bss(seq, bss, true);
}

void
fuxi_wifi_radio_done(uint32_t seq, int status)
{
	radio_done(seq, status, true);
}

void
fuxi_wifi_radio_stopped(unsigned code)
{
	thread_call_t call = NULL;

	lck_mtx_lock(&radio_lock);
	if (!radio_canned) {
		call = radio_stop_locked(code);
	}
	lck_mtx_unlock(&radio_lock);
	thread_wakeup((event_t)&radio_done_seq);
	if (call != NULL) {
		(void)thread_call_enter(call);
	}
}

/* ---- fuxiwifiios=4: the canned producer ----------------------------------------------------------------
 * Four made-up networks (locally administered BSSIDs 02:46:55:58:4a:01..04) as beacons built here and
 * handed to the very path a radio's frames take, one channel every CANNED_DWELL_MS. No radio, no hardware. */

#define CANNED_DWELL_MS         40
#define CANNED_FRAME_MOST       256

static char canned_dwell;               /* (its address only: what a dwell waits on, and nobody wakes) */

static const struct canned_bss {
	uint8_t last;                   /* the BSSID's last byte */
	uint16_t heard_mhz;             /* the channel its beacon is heard on */
	uint8_t channel;                /* the one it says it is on (DS at 2.4 GHz, HT operation at 5 GHz); 0: none */
	int8_t rssi;
	bool rsn;
	uint8_t ssid_length;
	const char *ssid;               /* NULL: hidden (as many zero bytes) */
} canned_list[] = {
	{ 0x01, 2432, 6, -48, true, 16, "Fuxi Canned WPA2" },   /* heard on channel 5, on 6: the DS correction */
	{ 0x02, 5180, 0, -61, false, 16, "Fuxi Canned Open" },
	{ 0x03, 2462, 11, -70, true, 9, NULL },
	{ 0x04, 5260, 52, -74, true, 15, "Fuxi Canned DFS" },
};

static size_t
canned_beacon(const struct canned_bss *bss, uint8_t *frame)
{
	static const uint8_t rates_low[] = { 1, 8, 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24 };
	static const uint8_t more_low[] = { 50, 4, 0x30, 0x48, 0x60, 0x6c };
	static const uint8_t rates_high[] = { 1, 8, 0x8c, 0x12, 0x98, 0x24, 0xb0, 0x48, 0x60, 0x6c };
	/* RSN: version 1, group CCMP, one pairwise CCMP, one AKM PSK, capabilities 0x000c */
	static const uint8_t rsn[] = { 48, 20, 1, 0, 0x00, 0x0f, 0xac, 4, 1, 0, 0x00, 0x0f, 0xac, 4, 1, 0, 0x00, 0x0f, 0xac, 2,
		                       0x0c, 0 };
	static const uint8_t bssid[6] = { 0x02, 0x46, 0x55, 0x58, 0x4a, 0 };
	bool low = bss->heard_mhz < 4000;
	size_t at;

	bzero(frame, CANNED_FRAME_MOST);
	frame[0] = 0x80;                                        /* a beacon */
	for (at = 4; at < 10; at++) {
		frame[at] = 0xff;                               /* to everyone */
	}
	memcpy(frame + 10, bssid, sizeof(bssid));
	memcpy(frame + 16, bssid, sizeof(bssid));
	frame[15] = frame[21] = bss->last;
	at = 24 + 8;                                            /* the header and the timestamp */
	frame[at] = 100;                                        /* every 100 TU */
	frame[at + 2] = bss->rsn ? 0x11 : 0x01;                 /* ESS, and privacy when protected */
	at += 4;
	frame[at] = 0;
	frame[at + 1] = bss->ssid_length;
	if (bss->ssid != NULL) {
		memcpy(frame + at + 2, bss->ssid, bss->ssid_length);
	}
	at += 2u + bss->ssid_length;
	memcpy(frame + at, low ? rates_low : rates_high, sizeof(rates_low));
	at += sizeof(rates_low);
	if (low) {
		frame[at] = 3;
		frame[at + 1] = 1;
		frame[at + 2] = bss->channel;
		at += 3;
		memcpy(frame + at, more_low, sizeof(more_low));
		at += sizeof(more_low);
	}
	if (bss->rsn) {
		memcpy(frame + at, rsn, sizeof(rsn));
		at += sizeof(rsn);
	}
	if (!low && bss->channel != 0) {
		frame[at] = 61;                                 /* HT operation: the primary channel, the rest zero */
		frame[at + 1] = 22;
		frame[at + 2] = bss->channel;
		at += 24;
	}
	return at;
}

static void
radio_canned_run(__unused void *parameter, __unused wait_result_t result)
{
	struct fuxi_wifi_radio_request request;
	struct radio_table *table;
	uint8_t frame[CANNED_FRAME_MOST];
	uint16_t freqs[FUXI_WIFI_RADIO_CHANNELS];

	for (unsigned i = 0; i < FUXI_WIFI_RADIO_CHANNELS; i++) {
		freqs[i] = radio_list[i].frequency;
	}
	radio_starting(false);
	radio_ready(freqs, FUXI_WIFI_RADIO_CHANNELS, false);
	for (;;) {
		bool cancelled = false;

		lck_mtx_lock(&radio_lock);
		while ((table = radio_take_locked(&request, false)) == NULL) {
			(void)lck_mtx_sleep(&radio_lock, LCK_SLEEP_DEFAULT, (event_t)&radio_request, THREAD_UNINT);
		}
		lck_mtx_unlock(&radio_lock);
		radio_reset(table);
		for (unsigned i = 0; i < request.freq_count && !cancelled; i++) {
			(void)assert_wait_timeout((event_t)&canned_dwell, THREAD_UNINT, CANNED_DWELL_MS, NSEC_PER_MSEC);
			(void)thread_block(THREAD_CONTINUE_NULL);
			cancelled = radio_cancelled(request.seq, false);
			for (unsigned j = 0; j < sizeof(canned_list) / sizeof(canned_list[0]) && !cancelled; j++) {
				if (canned_list[j].heard_mhz == request.freqs[i]) {
					(void)radio_frame(request.seq, frame, canned_beacon(&canned_list[j], frame),
					    canned_list[j].heard_mhz, canned_list[j].rssi, FUXI_WIFI_RADIO_NOISE_DBM, false);
				}
			}
		}
		radio_done(request.seq, cancelled ? ECANCELED : 0, false);
	}
}

/* ---- the consumer ------------------------------------------------------------------------------------- */

bool
fuxi_wifi_radio_use(bool canned_producer, void (*changed)(enum fuxi_wifi_radio_state state))
{
	struct radio_table *front, *back;
	struct fuxi_wifi_sta_bss *scratch;
	thread_call_t call = NULL;
	bool used;

	lck_mtx_lock(&radio_lock);
	used = radio_used;
	lck_mtx_unlock(&radio_lock);
	if (used || changed == NULL) {
		return false;
	}
	front = kalloc_data(sizeof(*front), Z_WAITOK | Z_ZERO);
	back = kalloc_data(sizeof(*back), Z_WAITOK | Z_ZERO);
	scratch = kalloc_data(2 * sizeof(*scratch), Z_WAITOK | Z_ZERO);
	if (front != NULL && back != NULL && scratch != NULL) {
		call = thread_call_allocate_with_options(radio_notify, NULL, THREAD_CALL_PRIORITY_KERNEL, THREAD_CALL_OPTIONS_ONCE);
	}
	if (call == NULL) {
		if (front != NULL) {
			kfree_data(front, sizeof(*front));
		}
		if (back != NULL) {
			kfree_data(back, sizeof(*back));
		}
		if (scratch != NULL) {
			kfree_data(scratch, 2 * sizeof(*scratch));
		}
		return false;
	}
	lck_mtx_lock(&radio_lock);
	radio_front = front;
	radio_back = back;
	radio_parsed = scratch;
	radio_copy = scratch + 1;
	radio_call = call;
	radio_changed = changed;
	radio_canned = canned_producer;
	radio_used = true;
	if (canned_producer) {
		/* the canned producer stands in for the radio lane: what the lane said before is forgotten */
		radio_state = FUXI_WIFI_RADIO_ABSENT;
		radio_code = 0;
		radio_freq_count = radio_freq_low = 0;
	}
	lck_mtx_unlock(&radio_lock);
	return true;
}

void
fuxi_wifi_radio_attached(void)
{
	thread_call_t call;
	thread_t thread;
	uint64_t silent_at;
	bool canned_producer;

	clock_interval_to_deadline(RADIO_SILENT_S, NSEC_PER_SEC, &silent_at);
	lck_mtx_lock(&radio_lock);
	radio_attached = radio_used;
	radio_silent_at = silent_at;
	call = radio_used ? radio_call : NULL;
	canned_producer = radio_canned;
	lck_mtx_unlock(&radio_lock);
	if (call == NULL) {
		return;
	}
	if (canned_producer) {
		if (kernel_thread_start(radio_canned_run, NULL, &thread) == KERN_SUCCESS) {
			thread_set_thread_name(thread, "wifi-radio-canned");
			thread_deallocate(thread);
		} else {
			lck_mtx_lock(&radio_lock);
			(void)radio_stop_locked(FUXI_WIFI_RADIO_NO_THREAD);
			lck_mtx_unlock(&radio_lock);
		}
	}
	(void)thread_call_enter(call);          /* what the producer said before attach; the backstop (=3) */
}

/* Under the lock: how the scan `seq` stands (-1: still running; `late`: its time is up). */
static int
radio_waited_locked(uint32_t seq, bool late)
{
	if (radio_done_seq == seq) {
		return radio_done_status;
	}
	if (radio_state != FUXI_WIFI_RADIO_READY) {
		return ENETDOWN;
	}
	if (radio_live != seq) {
		return ECANCELED;
	}
	return late ? ETIMEDOUT : -1;
}

static uint32_t
radio_post_locked(const struct fuxi_wifi_radio_request *what, const uint16_t *freqs, unsigned count)
{
	memcpy(&radio_request, what, sizeof(radio_request));
	bzero(radio_request.freqs, sizeof(radio_request.freqs));
	memcpy(radio_request.freqs, freqs, count * sizeof(freqs[0]));
	radio_request.freq_count = (uint16_t)count;
	radio_request.passive = true;                   /* whatever was asked: nothing is transmitted in this milestone */
	if (++radio_seq == 0) {
		radio_seq = 1;
	}
	radio_request.seq = radio_live = radio_seq;
	radio_taken = false;
	return radio_seq;
}

/* What one scan came to, for its line. */
struct radio_said {
	unsigned number, channels, heard, at_24, at_5, rsn, hidden, frames, dropped[FUXI_WIFI_RADIO_DROP_REASONS];
	enum fuxi_wifi_radio_state state;
	bool asked;                                     /* a request was posted */
};

static thread_call_t
radio_finish_locked(uint32_t seq, int status, struct radio_said *said)
{
	thread_call_t call = NULL;

	if (radio_live == seq) {
		radio_live = 0;                         /* a timeout or a cancel: the producer sees cancelled(seq) */
	}
	if (status != 0) {
		radio_forget_locked();
	}
	if (status == ETIMEDOUT && ++radio_timeouts >= RADIO_TIMEOUTS_MOST) {
		call = radio_stop_locked(FUXI_WIFI_RADIO_GAVE_UP | RADIO_TIMEOUTS_MOST);
	} else if (status == 0) {
		radio_timeouts = 0;
	}
	if (status == 0 && radio_front != NULL) {
		said->heard = radio_front->count;
		said->at_24 = radio_front->at_24;
		said->at_5 = radio_front->at_5;
		said->rsn = radio_front->rsn;
		said->hidden = radio_front->hidden;
		said->frames = radio_front->frames;
		memcpy(said->dropped, radio_front->dropped, sizeof(said->dropped));
	}
	return call;
}

static void
radio_say(const struct radio_said *said, int status, uint64_t elapsed)
{
	static const char *const states[] = { "absent", "starting", "ready", "stopped" };
	unsigned dropped = 0;
	uint64_t ns;

	if (said->number > RADIO_SAID_MOST && said->number % 100 != 0) {
		return;
	}
	if (!said->asked && said->state == FUXI_WIFI_RADIO_READY) {
		printf("FUXI WIFI IOS: radio: scan %u: none of the channels asked for is the radio's, nothing heard, status %d\n",
		    said->number, status);
		return;
	}
	if (!said->asked) {
		printf("FUXI WIFI IOS: radio: scan %u: the radio is %s, nothing asked of it, status %d\n", said->number,
		    states[said->state], status);
		return;
	}
	absolutetime_to_nanoseconds(elapsed, &ns);
	printf("FUXI WIFI IOS: radio: scan %u: %u channels passive, %u heard (%u at 2.4 GHz, %u at 5 GHz, %u RSN, %u hidden), "
	    "%llu ms, status %d\n", said->number, said->channels, said->heard, said->at_24, said->at_5, said->rsn, said->hidden,
	    (unsigned long long)(ns / 1000000), status);
	for (unsigned i = 1; i < FUXI_WIFI_RADIO_DROP_REASONS; i++) {
		dropped += said->dropped[i];
	}
	if (dropped != 0) {
		printf("FUXI WIFI IOS: radio: scan %u: %u frames, %u not kept (length %u, kind %u, BSSID %u, IBSS %u, SSID %u, "
		    "band %u, channel %u, unlisted %u, full %u)\n", said->number, said->frames, dropped,
		    said->dropped[FUXI_WIFI_RADIO_DROP_LENGTH], said->dropped[FUXI_WIFI_RADIO_DROP_KIND],
		    said->dropped[FUXI_WIFI_RADIO_DROP_BSSID], said->dropped[FUXI_WIFI_RADIO_DROP_IBSS],
		    said->dropped[FUXI_WIFI_RADIO_DROP_SSID], said->dropped[FUXI_WIFI_RADIO_DROP_BAND],
		    said->dropped[FUXI_WIFI_RADIO_DROP_CHANNEL], said->dropped[FUXI_WIFI_RADIO_DROP_UNLISTED],
		    said->dropped[FUXI_WIFI_RADIO_DROP_FULL]);
	}
}

/*
 * One scan, on the station thread: the channels asked for that the producer may use, posted as one request,
 * waited for at most `timeout_ms` (and while `give_up` says no). 0, ENETDOWN (no radio ready, or it stopped),
 * ETIMEDOUT (then cancelled), ECANCELED, or the producer's errno. Nothing is heard after anything but 0.
 */
int
fuxi_wifi_radio_scan(const struct fuxi_wifi_radio_request *what, unsigned timeout_ms, bool (*give_up)(void))
{
	struct radio_said said;
	uint16_t freqs[FUXI_WIFI_RADIO_FREQS_MOST];
	uint64_t start, now, limit;
	thread_call_t call;
	unsigned count = 0;
	uint32_t seq = 0;
	int status;

	if (what == NULL || what->freq_count > FUXI_WIFI_RADIO_FREQS_MOST || what->ssid_length > sizeof(what->ssid)) {
		return EINVAL;
	}
	bzero(&said, sizeof(said));
	if (timeout_ms == 0 || timeout_ms > FUXI_WIFI_RADIO_WAIT_MOST_MS) {
		timeout_ms = FUXI_WIFI_RADIO_WAIT_MOST_MS;
	}
	clock_get_uptime(&start);
	clock_interval_to_absolutetime_interval(timeout_ms, NSEC_PER_MSEC, &limit);
	lck_mtx_lock(&radio_lock);
	call = radio_asked_locked();
	said.state = radio_state;
	lck_mtx_unlock(&radio_lock);
	if (call != NULL) {
		(void)thread_call_enter(call);
	}
	/* (radio_freqs is fixed once READY was seen under the lock) */
	for (unsigned i = 0; said.state == FUXI_WIFI_RADIO_READY && i < what->freq_count; i++) {
		bool again = false;

		for (unsigned j = 0; j < count; j++) {
			again |= freqs[j] == what->freqs[i];
		}
		if (!again && radio_listed(what->freqs[i], radio_freqs, radio_freq_count)) {
			freqs[count++] = what->freqs[i];
		}
	}
	lck_mtx_lock(&radio_lock);
	said.number = ++radio_scans;
	said.state = radio_state;
	if (radio_state == FUXI_WIFI_RADIO_READY && count != 0) {
		seq = radio_post_locked(what, freqs, count);
	} else {
		radio_forget_locked();
	}
	lck_mtx_unlock(&radio_lock);
	if (seq == 0) {
		status = said.state == FUXI_WIFI_RADIO_READY ? 0 : ENETDOWN;
		radio_say(&said, status, 0);
		return status;
	}
	said.asked = true;
	said.channels = count;
	thread_wakeup((event_t)&radio_request);         /* (the canned producer sleeps on it; a radio lane polls take()) */
	for (status = -1; status < 0;) {
		clock_get_uptime(&now);
		lck_mtx_lock(&radio_lock);
		status = radio_waited_locked(seq, now - start >= limit);
		if (status < 0) {
			(void)assert_wait_timeout((event_t)&radio_done_seq, THREAD_UNINT, RADIO_POLL_MS, NSEC_PER_MSEC);
		}
		lck_mtx_unlock(&radio_lock);
		if (status < 0) {
			(void)thread_block(THREAD_CONTINUE_NULL);
			if (give_up != NULL && give_up()) {
				fuxi_wifi_radio_cancel();
			}
		}
	}
	lck_mtx_lock(&radio_lock);
	call = radio_finish_locked(seq, status, &said);
	lck_mtx_unlock(&radio_lock);
	if (call != NULL) {
		(void)thread_call_enter(call);
	}
	clock_get_uptime(&now);
	radio_say(&said, status, now - start);
	return status;
}

void
fuxi_wifi_radio_cancel(void)
{
	lck_mtx_lock(&radio_lock);
	radio_live = 0;
	lck_mtx_unlock(&radio_lock);
	thread_wakeup((event_t)&radio_done_seq);
}

/*
 * The last scan's records, one each() at a time, on the station thread: each is copied under the lock into
 * the consumer's scratch and handed over without it (each() takes sta_lock). A swap or a stop ends the walk.
 */
unsigned
fuxi_wifi_radio_results(void (*each)(const struct fuxi_wifi_sta_bss *bss, void *context), void *context)
{
	uint32_t generation;
	uint64_t now, ns;
	unsigned given = 0;

	if (each == NULL) {
		return 0;
	}
	clock_get_uptime(&now);
	lck_mtx_lock(&radio_lock);
	generation = radio_generation;
	lck_mtx_unlock(&radio_lock);
	for (unsigned i = 0; i < FUXI_WIFI_RADIO_BSS_MOST; i++) {
		uint64_t heard_at = 0;
		bool there;

		lck_mtx_lock(&radio_lock);
		there = radio_front != NULL && radio_copy != NULL && radio_generation == generation && i < radio_front->count;
		if (there) {
			memcpy(radio_copy, &radio_front->bss[i], sizeof(*radio_copy));
			heard_at = radio_front->heard_at[i];
		}
		lck_mtx_unlock(&radio_lock);
		if (!there) {
			break;
		}
		absolutetime_to_nanoseconds(now > heard_at ? now - heard_at : 0, &ns);
		radio_copy->age_ms = ns / 1000000 > 0xffffffffull ? 0xffffffffu : (uint32_t)(ns / 1000000);
		each(radio_copy, context);
		given++;
	}
	return given;
}

enum fuxi_wifi_radio_state
fuxi_wifi_radio_state(bool *scanning)
{
	enum fuxi_wifi_radio_state state;
	thread_call_t call;
	bool live;

	lck_mtx_lock(&radio_lock);
	call = radio_asked_locked();
	state = radio_state;
	live = radio_live != 0;
	lck_mtx_unlock(&radio_lock);
	if (call != NULL) {
		(void)thread_call_enter(call);
	}
	if (scanning != NULL) {
		*scanning = live;
	}
	return state;
}

#endif /* FUXI */
