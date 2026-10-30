/*
 * Ported from HackiOS xnu/files/bsd/dev/redwood_apple80211.c (APSL-2.0)
 * Fuxi adaptations, 2026-10-23: opt-in iOS shim; see docs-lanes/wifi-ios.md.
 * Fuxi, 2026-10-29: the requests wifid of 20H392 was seen to make and the shim refused are answered
 * or refused on purpose (each says which below); a summary of the run is said now and then.
 * Fuxi, 2026-10-30 (K2): a scan's result carries the noise figure when there is one and its channel's
 * own flags (passive, radar); with the radio backends (fuxiwifiios=3/4) the elements are kept whole
 * within the room wifid gives, and GET 43/44 say which backend answers.
 * Original file commentary is retained below. See LICENSE.HackiOS.
 */
/*
 * Darwin-on-redwood: Apple80211, answered in the kernel.
 *
 * iOS's Wi-Fi side -- wifid, and through it Settings and the status bar -- talks to a Wi-Fi
 * driver in requests that were once ioctls and are, where the driver offers no user client,
 * ioctls still: SIOCGA80211 and SIOCSA80211 on any socket, with a struct apple80211req that
 * names the interface, a request type, a value, and a buffer in the caller's memory
 * (IO80211.framework: Apple80211Get/Set and their kin all end there). XNU hands an interface
 * ioctl it does not know to the interface, so they arrive in bsd/dev/fuxi_wifi_net.c, which
 * passes them here when its link is a radio.
 *
 * What is asked is answered from the station boundary (sys/fuxi_wifi_sta.h): the radio's power,
 * its channels, a scan and what it heard, the network joined. What happens there by itself is
 * told to iOS the way a driver tells it: kernel events of class 6 (IEEE 802.11), subclass 1,
 * whose code is Apple80211's own event number and whose data begins with the interface's name
 * in 16 bytes (IO80211.framework reads them from a PF_SYSTEM socket and compares the name).
 *
 * Every request is still written to the log -- who asks, get or set, the type, the value, the
 * length -- because which of them wifid needs is found that way. Never what a request carries:
 * a network's name or its key may be in there.
 *
 * The request types and the layouts are Apple's. The old ones are as the headers of its open
 * IO80211 sources once described them (apple80211_ioctl.h, apple80211_var.h); where iOS 16.5
 * differs the layout was read out of IO80211.framework of 16.5, which fills and reads the same
 * structures on the other side (Apple80211GetWithIOCTL, _getScanData, _addScanResultToList,
 * _performAssociation, _dispatchAsyncAssocEvent, _dispatchLinkChangedEvent).
 *
 * A join: ASSOCIATE (20) carries the network's name, perhaps its access point, and the
 * passphrase as typed (the framework does not make the key of it), and is answered at once;
 * when the station has joined or failed, event 9 says so, and ASSOCIATE_RESULT (21) gives the
 * outcome as one of Apple80211's result numbers, which the framework turns into its errors:
 * 1 success, 21 "invalid PMK" (a wrong passphrase), 9 a timeout, 2 anything else.
 *
 * Fuxi, 20H392: wifid starts, opens the interface and polls it, and scans only when somebody
 * asks it to (Settings' Wi-Fi page) or when its auto-join knows a network to look for; none of
 * the answers below decides that. A refusal (EOPNOTSUPP) and "not joined" (ENXIO) both reach wifid
 * as -3903, which it only logs. The user client that IO80211 asks FuxiWLAN for (type 0) stays
 * unmade on purpose: without it the framework uses these ioctls and the kernel-event socket for
 * everything, and a user client would have to speak IO80211's own protocol (15408-byte requests,
 * a mapped event pipe) to be of any use.
 */
#if defined(FUXI)
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <sys/errno.h>
#include <sys/kern_event.h>
#include <sys/kernel.h>
#include <sys/proc.h>
#include <sys/fuxi_wifi_net.h>
#include <sys/fuxi_wifi_radio.h>
#include <sys/fuxi_wifi_sta.h>
#include <sys/sockio.h>
#include <sys/systm.h>

#include <sys/fuxi_wifi_apple80211.h>

/* (as far as they have come by: the names help reading the log; 96 and up as IO80211 of 20H392 names them) */
static const char *
type_name(int32_t type)
{
	static const struct {
		int32_t type;
		const char *name;
	} names[] = {
		{ 1, "SSID" }, { 2, "AUTH_TYPE" }, { 3, "CIPHER_KEY" }, { 4, "CHANNEL" }, { 5, "POWERSAVE" }, { 6, "PROTMODE" },
		{ 7, "TXPOWER" }, { 8, "RATE" }, { 9, "BSSID" }, { 10, "SCAN_REQ" }, { 11, "SCAN_RESULT" }, { 12, "CARD_CAPABILITIES" },
		{ 13, "STATE" }, { 14, "PHY_MODE" }, { 15, "OP_MODE" }, { 16, "RSSI" }, { 17, "NOISE" }, { 18, "INT_MIT" }, { 19, "POWER" },
		{ 20, "ASSOCIATE" }, { 21, "ASSOCIATE_RESULT" }, { 22, "DISASSOCIATE" }, { 23, "STATUS_DEV_NAME" }, { 24, "IBSS_MODE" },
		{ 25, "HOST_AP_MODE" }, { 26, "AP_MODE" }, { 27, "SUPPORTED_CHANNELS" }, { 28, "LOCALE" }, { 29, "DEAUTH" },
		{ 30, "COUNTERMEASURES" }, { 32, "RATE_SET" }, { 42, "STATION_LIST" }, { 43, "DRIVER_VERSION" },
		{ 44, "HARDWARE_VERSION" }, { 46, "RSN_IE" }, { 47, "BACKGROUND_SCAN" }, { 48, "AP_IE_LIST" }, { 49, "STATS" },
		{ 50, "ASSOCIATION_STATUS" }, { 51, "COUNTRY_CODE" }, { 54, "RADIO_INFO" }, { 57, "MCS" }, { 66, "MCS_INDEX_SET" },
		{ 79, "STA_STATS" }, { 80, "ROAM_THRESH" }, { 85, "IE" }, { 86, "SCAN_REQ_MULTIPLE" }, { 87, "BTCOEX_MODE" },
		{ 90, "SCANCACHE_CLEAR" }, { 94, "VIRTUAL_IF_CREATE" }, { 95, "VIRTUAL_IF_DELETE" }, { 96, "VIRTUAL_IF_ROLE" },
		{ 97, "VIRTUAL_IF_PARENT" }, { 103, "CURRENT_NETWORK" }, { 106, "RSSI_BOUNDS" }, { 107, "ROAM" },
		{ 115, "POWER_DEBUG_INFO" }, { 207, "CHANNELS_INFO" }, { 254, "HW_SUPPORTED_CHANNELS" }, { 365, "DISPLAY_STATE" },
		{ 383, "SET_PROPERTY" },
	};

	for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
		if (names[i].type == type) {
			return names[i].name;
		}
	}
	return "?";
}

/* Apple80211's events (APPLE80211_M_...) */
enum { EVENT_POWER_CHANGED = 1, EVENT_SSID_CHANGED = 2, EVENT_BSSID_CHANGED = 3, EVENT_LINK_CHANGED = 4, EVENT_ASSOC_DONE = 9,
       EVENT_SCAN_DONE = 10 };

static const char interface_name[16] = "en0";   /* (bsd/dev/fuxi_wifi_net.c makes one interface, and that is its name) */

static LCK_GRP_DECLARE(apple80211_lock_group, "fuxi-wifi-apple80211");
static LCK_MTX_DECLARE(apple80211_lock, &apple80211_lock_group);
static unsigned result_next;                    /* which of what the last scan heard a SCAN_RESULT request gets next */
static uint32_t join_result;                    /* of the last join asked for: 0 while it runs, then Apple80211's number */
/*
 * The country, as iOS last set it (SET COUNTRY_CODE). Until it does, a placeholder that every table
 * of iOS's knows: nothing here transmits, and the synthetic radio's one channel is allowed everywhere.
 * A real radio's backend will have to say its own.
 */
static char country[3] = "US";

/*
 * (Fuxi, K2) Who answers, for GET 43/44, and whether a scan's records carry real elements: the radio
 * backends (fuxiwifiios=3/4) say so once at init (fuxi_wifi_apple80211_backend); the synthetic and route U
 * backends never call it, and what they are answered stays as it was.
 */
static char driver_name[256] = "Fuxi Wi-Fi iOS shim (synthetic backend)";
static bool whole_elements;
static bool room_said;

void
fuxi_wifi_apple80211_backend(const char *name, bool whole)
{
	bzero(driver_name, sizeof(driver_name));
	snprintf(driver_name, sizeof(driver_name), "%s", name);
	whole_elements = whole;
}

/* ---- channels ---------------------------------------------------------------------------------------- */

static unsigned
channel_number(unsigned frequency)
{
	return frequency == 2484 ? 14 : frequency < 4000 ? (frequency - 2407) / 5 : (frequency - 5000) / 5;
}

static unsigned
channel_frequency(const struct apple80211_channel *channel)
{
	bool low = channel->flags & CHANNEL_2GHZ ? true : channel->flags & CHANNEL_5GHZ ? false : channel->channel <= 14;

	if (channel->channel == 0 || channel->channel > 196 || (channel->flags & 0x2000)) {     /* (0x2000: 6 GHz) */
		return 0;
	}
	return low ? (channel->channel == 14 ? 2484 : 2407 + 5 * channel->channel) : 5000 + 5 * channel->channel;
}

static void
channel_fill(struct apple80211_channel *channel, unsigned frequency, unsigned flags)
{
	channel->version = 1;
	channel->channel = channel_number(frequency);
	channel->flags = CHANNEL_20MHZ | (frequency < 4000 ? CHANNEL_2GHZ : CHANNEL_5GHZ) | (flags & FUXI_WIFI_STA_CHANNEL_PASSIVE ?
	    0 : CHANNEL_ACTIVE) | (flags & FUXI_WIFI_STA_CHANNEL_RADAR ? CHANNEL_DFS : 0);
}

/* (Fuxi, K2) What the station's list says of a channel (passive, radar); 0 when it is not on the list. */
static unsigned
channel_flags(unsigned frequency)
{
	struct fuxi_wifi_sta_channel channels[96];
	unsigned count = fuxi_wifi_sta_channels(channels, 96);

	for (unsigned i = 0; i < count; i++) {
		if (channels[i].frequency == frequency) {
			return channels[i].flags;
		}
	}
	return 0;
}

/* ---- an access point's rates --------------------------------------------------------------------------- */

/*
 * The rates it says it has, in Mbit/s, at most `most` of them; how many. (Elements 1 and 50: units of
 * 500 kbit/s, the top bit marks the ones everybody must have.)
 */
static unsigned
bss_rates(const struct fuxi_wifi_sta_bss *bss, uint32_t *rates, unsigned most)
{
	unsigned count = 0;

	for (unsigned at = 0; at + 2 <= bss->elements_length && at + 2 + bss->elements[at + 1] <= bss->elements_length;
	    at += 2 + bss->elements[at + 1]) {
		for (unsigned i = 0; (bss->elements[at] == 1 || bss->elements[at] == 50) && i < bss->elements[at + 1]; i++) {
			unsigned rate = (bss->elements[at + 2 + i] & 0x7f) / 2;

			if (rate != 0 && rate <= 54 && count < most) {
				rates[count++] = rate;
			}
		}
	}
	return count;
}

/* The best of them (0: it says none). */
static unsigned
bss_best_rate(const struct fuxi_wifi_sta_bss *bss)
{
	uint32_t rates[64];
	unsigned count = bss_rates(bss, rates, 64), best = 0;

	for (unsigned i = 0; i < count; i++) {
		best = rates[i] > best ? rates[i] : best;
	}
	return best;
}

/* ---- what the caller is given back ------------------------------------------------------------------- */

/* At most as many bytes as it has room for. */
static int
give(const struct apple80211req *request, const void *data, size_t length)
{
	return request->req_data == USER_ADDR_NULL ? EFAULT : copyout(data, request->req_data, length < request->req_len ? length :
	       request->req_len);
}

/*
 * An access point in the form a scan's result has; the caller has said how much room for elements there is.
 * (Fuxi, K2) With whole elements (the radio backends) a room of 0 or over 1024 counts as 1024, a short
 * buffer's room is what follows the fixed fields, and the elements that fit are kept whole, those that show a
 * network and its protection first (fuxi_wifi_elements_fit). Otherwise all of them or none, as before.
 */
static int
give_network(const struct apple80211req *request, const struct fuxi_wifi_sta_bss *bss)
{
	struct apple80211_scan_result *result = kalloc_data(sizeof(*result), Z_WAITOK | Z_ZERO);
	unsigned room = sizeof(result->elements), asked = 0, head = (unsigned)offsetof(struct apple80211_scan_result, elements);
	int error;

	if (bss->ssid_length > sizeof(bss->ssid) || bss->elements_length > sizeof(bss->elements)) {
		if (result != NULL) {
			kfree_data(result, sizeof(*result));
		}
		return EINVAL;
	}
	if (result == NULL) {
		return ENOMEM;
	}
	if (request->req_len >= sizeof(*result)) {
		if (copyin(request->req_data, result, head) == 0) {
			asked = result->elements_length;
			room = asked < room && (asked != 0 || !whole_elements) ? asked : room;
		}
	} else if (whole_elements) {
		room = request->req_len > head ? request->req_len - head : 0;
	}
	bzero(result, sizeof(*result));
	result->version = 1;
	channel_fill(&result->channel, bss->frequency, channel_flags(bss->frequency));
	if (bss->noise != 0) {                  /* (the synthetic BSS has none: its result is as before) */
		int snr = bss->rssi - bss->noise;

		result->noise_known = 1;
		result->noise = bss->noise;
		result->snr = (int16_t)(snr < 0 ? 0 : snr > 100 ? 100 : snr);
	}
	result->rssi = bss->rssi;
	result->beacon_interval = bss->beacon_interval;
	result->capability = bss->capability;
	memcpy(result->bssid, bss->bssid, sizeof(result->bssid));
	result->ssid_length = bss->ssid_length;
	memcpy(result->ssid, bss->ssid, bss->ssid_length);
	result->age_ms = bss->age_ms;
	if (whole_elements) {
		result->elements_length = (uint16_t)fuxi_wifi_elements_fit(bss->elements, bss->elements_length, result->elements,
		    room);
		if (!__atomic_exchange_n(&room_said, true, __ATOMIC_RELAXED)) {
			printf("FUXI WIFI IOS: apple80211: first SCAN_RESULT room %u bytes (wifid's %u in %u bytes)\n", room, asked,
			    request->req_len);
		}
	} else {
		result->elements_length = (uint16_t)(bss->elements_length <= room ? bss->elements_length : 0);
		memcpy(result->elements, bss->elements, result->elements_length);
	}
	result->rate_count = (uint8_t)bss_rates(bss, result->rates, (unsigned)(sizeof(result->rates) / sizeof(result->rates[0])));
	error = give(request, result, sizeof(*result));
	kfree_data(result, sizeof(*result));
	return error;
}

/* ---- get ---------------------------------------------------------------------------------------------- */

static int
get(struct apple80211req *request, char *said, size_t said_size)
{
	struct fuxi_wifi_sta_state *state;
	int error = 0;

	switch (request->req_type) {
	case 43:
	case 44:                                /* the driver's and the hardware's name: the text alone, in 256 bytes */
		return give(request, driver_name, sizeof(driver_name));
	case 12: {
		/*
		 * What the card can do, a bit each (apple80211_var.h: APPLE80211_CAP_*): the 13 bytes are the
		 * bits alone, from the first (the framework hands in a buffer that begins after the old
		 * header's version word). AES, AES-CCM, power management, short slot and preamble, WPA2, WME,
		 * short guard interval at 20 MHz. Read wrong, the framework sees no WPA2 and builds no join
		 * for a WPA2 network ("Unable to join").
		 */
		uint8_t bits[13] = { 0 };
		static const uint8_t has[] = { 2, 3, 6, 9, 10, 14, 17, 19 };

		for (unsigned i = 0; i < sizeof(has); i++) {
			bits[has[i] / 8] |= (uint8_t)(1u << (has[i] % 8));
		}
		return give(request, bits, sizeof(bits));
	}
	case 19: {                              /* the radio's power: one radio. Until the driver is up it counts as on: it */
		struct {                        /* is being brought up, and iOS asks this once, early */
			uint32_t version, radios, state[4];
		} answer = { .version = 1, .radios = 1, .state = { !fuxi_wifi_sta_present() || fuxi_wifi_sta_powered() } };

		return give(request, &answer, sizeof(answer));
	}
	case 15:                                /* the mode: a station (APPLE80211_M_STA) */
		request->req_val = 1;
		return 0;
	case 27:
	case 254: {                             /* the channels the radio may tune to */
		struct apple80211_channels *answer;
		struct fuxi_wifi_sta_channel channels[96];
		unsigned count = fuxi_wifi_sta_channels(channels, 96);

		if (count == 0 || (answer = kalloc_data(sizeof(*answer), Z_WAITOK | Z_ZERO)) == NULL) {
			return count == 0 ? ENXIO : ENOMEM;
		}
		answer->version = 1;
		answer->count = count;
		for (unsigned i = 0; i < count; i++) {
			channel_fill(&answer->channels[i], channels[i].frequency, channels[i].flags);
		}
		error = give(request, answer, sizeof(*answer));
		kfree_data(answer, sizeof(*answer));
		return error;
	}
	case 207: {                             /* ... and more of each */
		struct apple80211_channels_info *answer;
		struct fuxi_wifi_sta_channel channels[96];
		unsigned count = fuxi_wifi_sta_channels(channels, 96);

		if (count == 0 || (answer = kalloc_data(sizeof(*answer), Z_WAITOK | Z_ZERO)) == NULL) {
			return count == 0 ? ENXIO : ENOMEM;
		}
		answer->count = (uint16_t)count;
		for (unsigned i = 0; i < count; i++) {
			struct apple80211_channel channel;

			channel_fill(&channel, channels[i].frequency, channels[i].flags);
			answer->number[i] = (uint8_t)channel.channel;
			answer->dfs[i] = (channels[i].flags & FUXI_WIFI_STA_CHANNEL_RADAR) != 0;
			answer->passive[i] = (channels[i].flags & FUXI_WIFI_STA_CHANNEL_PASSIVE) != 0;
			answer->flags[i] = channel.flags;
		}
		error = give(request, answer, sizeof(*answer));
		kfree_data(answer, sizeof(*answer));
		return error;
	}
	case 21:                                /* how the last join went (0: not over yet) */
		lck_mtx_lock(&apple80211_lock);
		request->req_val = (int32_t)join_result;
		lck_mtx_unlock(&apple80211_lock);
		return 0;
	case 11: {                              /* what the last scan heard, one access point a request; EIO: that was all */
		struct fuxi_wifi_sta_bss *bss = kalloc_data(sizeof(*bss), Z_WAITOK);
		unsigned index;
		bool there;

		if (bss == NULL) {
			return ENOMEM;
		}
		lck_mtx_lock(&apple80211_lock);
		index = result_next++;
		lck_mtx_unlock(&apple80211_lock);
		there = fuxi_wifi_sta_heard(index, bss);
		error = there ? give_network(request, bss) : EIO;
		kfree_data(bss, sizeof(*bss));
		return error;
	}
	case 51: {                              /* the country: its two letters and a NUL (wifid reads it, then sets its own) */
		char code[3];

		lck_mtx_lock(&apple80211_lock);
		memcpy(code, country, sizeof(code));
		lck_mtx_unlock(&apple80211_lock);
		return give(request, code, sizeof(code));
	}
	case 106: {                             /* what to make of a signal: the weakest, the strongest, and below what it is weak */
		struct apple80211_rssi_bounds answer = { .version = 1, .rssi_min = -100, .rssi_max = 0, .rssi_threshold = -80 };

		return give(request, &answer, sizeof(answer));
	}
	case 5:                                 /* power saving: off (the shim saves nothing) */
	case 18:                                /* interference mitigation: off */
		request->req_val = 0;
		return 0;
	case 6: {                               /* protection (RTS/CTS): off, no threshold */
		struct apple80211_protmode answer = { .version = 1 };

		return give(request, &answer, sizeof(answer));
	}
	/*
	 * Refused on purpose (wifid and CoreWiFi take a refusal as "no such value"): the station boundary
	 * knows no noise figure and no MCS (an access point's legacy rates only); en0 is the infrastructure
	 * interface itself, not a virtual one with a role or a parent; there is no power to account for.
	 */
	case 17:
	case 57:
	case 96:
	case 97:
	case 115:
		snprintf(said, said_size, " (on purpose: %s)", request->req_type == 17 ? "no noise figure" : request->req_type == 57 ?
		    "no MCS" : request->req_type == 115 ? "no power accounting" : "not a virtual interface");
		return EOPNOTSUPP;
	default:
		break;
	}

	/* the rest is about the network joined, and there is none unless the station says so */
	if ((state = kalloc_data(sizeof(*state), Z_WAITOK)) == NULL) {
		return ENOMEM;
	}
	fuxi_wifi_sta_state(state);
	switch (request->req_type) {
	case 13:                                /* the state: APPLE80211_S_INIT 0, _SCAN 1, _AUTH 2, _ASSOC 3, _RUN 4 */
		request->req_val = state->phase == FUXI_WIFI_STA_JOINED ? 4 : state->phase == FUXI_WIFI_STA_JOINING ? 2 :
		    state->phase == FUXI_WIFI_STA_SCANNING ? 1 : 0;
		break;
	case 1:                                 /* its name; the length goes back in the request */
		if (state->phase != FUXI_WIFI_STA_JOINED) {
			error = ENXIO;
		} else if ((error = give(request, state->bss.ssid, state->bss.ssid_length)) == 0) {
			request->req_len = state->bss.ssid_length;
		}
		break;
	case 9:                                 /* its access point's address */
		error = state->phase != FUXI_WIFI_STA_JOINED ? ENXIO : give(request, state->bss.bssid, 6);
		break;
	case 4: {                               /* its channel */
		struct {
			uint32_t version;
			struct apple80211_channel channel;
		} answer = { .version = 1 };

		if (state->phase != FUXI_WIFI_STA_JOINED) {
			error = ENXIO;
		} else {
			channel_fill(&answer.channel, state->bss.frequency, 0);
			error = give(request, &answer, sizeof(answer));
		}
		break;
	}
	case 16: {                              /* how strongly it is heard: dBm, per radio and all together, twice over */
		struct {
			uint32_t version, radios, unit;
			int32_t each[4], all, each_ext[4], all_ext;
		} answer = { .version = 1, .radios = 1, .each = { state->bss.rssi }, .all = state->bss.rssi,
			     .each_ext = { state->bss.rssi }, .all_ext = state->bss.rssi };

		error = state->phase != FUXI_WIFI_STA_JOINED ? ENXIO : give(request, &answer, sizeof(answer));
		break;
	}
	case 103:                               /* all of it, as a scan's result */
		error = state->phase != FUXI_WIFI_STA_JOINED ? ENXIO : give_network(request, &state->bss);
		break;
	case 2: {                               /* how it was joined: open system, then WPA2 with a passphrase */
		struct {
			uint32_t version, lower, upper, extended;
		} answer = { .version = 1, .lower = 1, .upper = state->security == FUXI_WIFI_STA_WPA2_PSK ? 8 : 0 };

		error = state->phase != FUXI_WIFI_STA_JOINED ? ENXIO : give(request, &answer, sizeof(answer));
		break;
	}
	case 8: {                               /* the rate: Mbit/s, the best its access point has (no rate control here) */
		unsigned rate = state->phase == FUXI_WIFI_STA_JOINED ? bss_best_rate(&state->bss) : 0;

		if (rate == 0) {
			error = ENXIO;
		} else {
			request->req_val = (int32_t)rate;
		}
		break;
	}
	case 14: {                              /* the kinds of 802.11 the radio has (by its bands) and the one in use */
		struct apple80211_phy_mode answer = { .version = 1, .supported = PHY_MODE_AUTO, .active = PHY_MODE_AUTO };
		struct fuxi_wifi_sta_channel channels[96];
		unsigned count = fuxi_wifi_sta_channels(channels, 96);

		for (unsigned i = 0; i < count; i++) {
			answer.supported |= channels[i].frequency < 4000 ? PHY_MODE_11B | PHY_MODE_11G : PHY_MODE_11A;
		}
		if (state->phase == FUXI_WIFI_STA_JOINED) {
			answer.active = state->bss.frequency >= 4000 ? PHY_MODE_11A : bss_best_rate(&state->bss) > 11 ? PHY_MODE_11G :
			    PHY_MODE_11B;
		}
		error = give(request, &answer, sizeof(answer));
		break;
	}
	default:
		error = EOPNOTSUPP;
		break;
	}
	kfree_data(state, sizeof(*state));
	return error;
}

/* ---- set ---------------------------------------------------------------------------------------------- */

static int
set_scan(const struct apple80211req *request, char *said, size_t said_size)
{
	struct apple80211_scan_request *asked;
	struct fuxi_wifi_sta_scan *what;
	unsigned unknown = 0, any = 0;
	int error;

	if (request->req_len != sizeof(*asked)) {
		return EINVAL;
	}
	if (!fuxi_wifi_sta_present()) {
		return EBUSY;                   /* (the radio is still being brought up: the framework tries again) */
	}
	asked = kalloc_data(sizeof(*asked), Z_WAITOK);
	what = kalloc_data(sizeof(*what), Z_WAITOK | Z_ZERO);
	if (asked == NULL || what == NULL || (error = copyin(request->req_data, asked, sizeof(*asked))) != 0) {
		error = asked == NULL || what == NULL ? ENOMEM : error;
		goto out;
	}
	if (asked->ssid_length > sizeof(what->ssid) || asked->channel_count > 400 ||
	    asked->name_count > 10) {
		error = EINVAL;
		goto out;
	}
	if (asked->ssid_length <= sizeof(what->ssid)) {
		what->ssid_length = (uint8_t)asked->ssid_length;
		memcpy(what->ssid, asked->ssid, asked->ssid_length);
	}
	what->passive = asked->scan_type == 2;
	what->dwell_ms = asked->dwell_ms;
	/*
	 * (More channels than there is room to name: every channel. So is one numbered 0: IO80211 of 20H392
	 * sends a scan of type 3 with exactly one, {1, 0, 0}, when it was given no channel of its own.)
	 */
	for (unsigned i = 0; i < asked->channel_count && i < 400 && asked->channel_count <= 64; i++) {
		unsigned frequency = channel_frequency(&asked->channels[i]);

		if (asked->channels[i].channel == 0) {
			any++;
		} else if (frequency != 0) {
			what->frequencies[what->frequency_count++] = (uint16_t)frequency;
		} else {
			unknown++;
		}
	}
	if (any != 0) {
		what->frequency_count = 0;
	}
	snprintf(said, said_size, " (a scan: type %u, %u channels, %u not the radio's, %u any; %u ms on each, a name of %u bytes, "
	    "%u more names, flags %#llx)", asked->scan_type, asked->channel_count, unknown, any, asked->dwell_ms, asked->ssid_length,
	    asked->name_count, (unsigned long long)asked->flags);
	if (asked->channel_count != 0 && asked->channel_count <= 64 && what->frequency_count == 0 && any == 0) {
		error = EINVAL;                 /* (nothing of what is asked for is a channel of this radio's) */
	} else {
		lck_mtx_lock(&apple80211_lock);
		result_next = 0;
		lck_mtx_unlock(&apple80211_lock);
		error = fuxi_wifi_sta_ask_scan(what);
	}
out:
	if (asked != NULL) {
		kfree_data(asked, sizeof(*asked));
	}
	if (what != NULL) {
		kfree_data(what, sizeof(*what));
	}
	return error;
}

static int
set_join(const struct apple80211req *request, char *said, size_t said_size)
{
	struct apple80211_assoc *asked;
	struct fuxi_wifi_sta_join *what;
	int error;

	if (request->req_len != sizeof(struct apple80211_assoc)) {
		return EINVAL;
	}
	asked = kalloc_data(sizeof(*asked), Z_WAITOK | Z_ZERO);
	what = kalloc_data(sizeof(*what), Z_WAITOK | Z_ZERO);
	if (asked == NULL || what == NULL || (error = copyin(request->req_data, asked, offsetof(struct apple80211_assoc, rest))) != 0) {
		error = asked == NULL || what == NULL ? ENOMEM : error;
		goto out;
	}
	/* (lengths and kinds only: the name, the access point and the key stay out of the log) */
	snprintf(said, said_size, " (a join: a name of %u bytes, %s access point, authentication %#x/%#x, a key of %u bytes of kind %u)",
	    asked->ssid_length, asked->bssid[0] | asked->bssid[1] | asked->bssid[2] | asked->bssid[3] | asked->bssid[4] |
	    asked->bssid[5] ? "a given" : "any", asked->auth_lower, asked->auth_upper, asked->key_length, asked->key_cipher);
	if (asked->ssid_length == 0 || asked->ssid_length > sizeof(what->ssid)) {
		error = EINVAL;
		goto out;
	}
	what->ssid_length = (uint8_t)asked->ssid_length;
	memcpy(what->ssid, asked->ssid, asked->ssid_length);
	memcpy(what->bssid, asked->bssid, sizeof(what->bssid));
	if (asked->key_length == 0) {
		what->security = FUXI_WIFI_STA_OPEN;
	} else if (asked->key_cipher == CIPHER_PASSPHRASE && asked->key_length >= 8 && asked->key_length <= 63) {
		what->security = FUXI_WIFI_STA_WPA2_PSK;
		what->passphrase_length = (uint8_t)asked->key_length;
		memcpy(what->passphrase, asked->key, asked->key_length);
	} else if (asked->key_cipher == CIPHER_PMK && asked->key_length == sizeof(what->pmk)) {
		what->security = FUXI_WIFI_STA_WPA2_PSK;
		memcpy(what->pmk, asked->key, sizeof(what->pmk));
	} else {
		error = EINVAL;
		goto out;
	}
	lck_mtx_lock(&apple80211_lock);
	join_result = 0;
	lck_mtx_unlock(&apple80211_lock);
	error = fuxi_wifi_sta_ask_join(what);
out:
	if (asked != NULL) {
		memset_s(asked, sizeof(*asked), 0, sizeof(*asked));
		kfree_data(asked, sizeof(*asked));
	}
	if (what != NULL) {
		memset_s(what, sizeof(*what), 0, sizeof(*what));
		kfree_data(what, sizeof(*what));
	}
	return error;
}

/* The country iOS has settled on: two letters (or a letter and a digit), kept and given back by GET. */
static int
set_country(const struct apple80211req *request, char *said, size_t said_size)
{
	struct apple80211_country asked = { 0 };
	int error;

	if (request->req_len < offsetof(struct apple80211_country, code) + 2 || request->req_len > 64) {
		return EINVAL;
	}
	error = copyin(request->req_data, &asked, request->req_len < sizeof(asked) ? request->req_len : sizeof(asked));
	if (error != 0) {
		return error;
	}
	for (unsigned i = 0; i < 2; i++) {
		char c = asked.code[i] >= 'a' && asked.code[i] <= 'z' ? (char)(asked.code[i] - 'a' + 'A') : asked.code[i];

		if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
			return EINVAL;
		}
		asked.code[i] = c;
	}
	lck_mtx_lock(&apple80211_lock);
	country[0] = asked.code[0];
	country[1] = asked.code[1];
	country[2] = '\0';
	lck_mtx_unlock(&apple80211_lock);
	snprintf(said, said_size, " (the country: %c%c)", asked.code[0], asked.code[1]);
	return 0;
}

/*
 * One of wifid's properties of the interface (its IP configuration, wake schedule and the like), as an
 * XML property list of one key: taken and dropped; the key's name goes to the log, never the value.
 */
static int
set_property(const struct apple80211req *request, char *said, size_t said_size)
{
	struct apple80211_property *asked;
	char key[64];
	unsigned length, k = 0;
	int error;

	if (request->req_len < offsetof(struct apple80211_property, xml) || request->req_len > sizeof(*asked)) {
		return EINVAL;
	}
	if ((asked = kalloc_data(sizeof(*asked), Z_WAITOK | Z_ZERO)) == NULL) {
		return ENOMEM;
	}
	if ((error = copyin(request->req_data, asked, request->req_len)) == 0) {
		length = request->req_len - (unsigned)offsetof(struct apple80211_property, xml);
		length = asked->length < length ? asked->length : length;
		for (unsigned at = 0; at + 5 <= length; at++) {
			if (memcmp(asked->xml + at, "<key>", 5) == 0) {
				for (at += 5; at < length && asked->xml[at] != '<' && k + 1 < sizeof(key); at++) {
					key[k++] = (char)(asked->xml[at] > ' ' && asked->xml[at] < 0x7f ? asked->xml[at] : '?');
				}
				break;
			}
		}
		key[k] = '\0';
		snprintf(said, said_size, " (a property: %s, %u bytes of XML; taken, not used)", k != 0 ? key : "no key", asked->length);
	}
	kfree_data(asked, sizeof(*asked));
	return error;
}

static int
set(const struct apple80211req *request, char *said, size_t said_size)
{
	switch (request->req_type) {
	case 20:
		return set_join(request, said, said_size);
	case 22:                                /* leave the network */
		return fuxi_wifi_sta_ask_leave();
	case 19: {                              /* the radio's power */
		struct {
			uint32_t version, radios, state[4];
		} asked;
		int error;

		if (request->req_len < sizeof(asked) || (error = copyin(request->req_data, &asked, sizeof(asked))) != 0) {
			return request->req_len < sizeof(asked) ? EINVAL : error;
		}
		snprintf(said, said_size, " (power: %s)", asked.state[0] ? "on" : "off");
		return fuxi_wifi_sta_ask_power(asked.state[0] != 0);
	}
	case 10:
		return set_scan(request, said, said_size);
	case 51:
		return set_country(request, said, said_size);
	case 383:
		return set_property(request, said, said_size);
	case 365:                               /* the display went on or off: nothing here depends on it */
		snprintf(said, said_size, " (the display: %d; taken, not used)", request->req_val);
		return 0;
	case 25:                                /* host access point: 2 (no data) stops it, and none runs; none is started */
		if (request->req_val == 2 && request->req_len == 0) {
			snprintf(said, said_size, " (stop the access point: none runs)");
			return 0;
		}
		snprintf(said, said_size, " (on purpose: no access point is made)");
		return EOPNOTSUPP;
	default:
		return EOPNOTSUPP;
	}
}

/* ---- events -------------------------------------------------------------------------------------------- */

static unsigned posted[EVENT_SCAN_DONE + 1];   /* how many of each were posted (for the summary) */

static void
post(uint32_t code, const void *data, uint32_t length)
{
	struct kev_msg message = { .vendor_code = KEV_VENDOR_APPLE, .kev_class = KEV_IEEE80211_CLASS, .kev_subclass = 1,
		                   .event_code = code };
	char name[16];

	memcpy(name, interface_name, sizeof(name));
	message.dv[0].data_length = sizeof(name);
	message.dv[0].data_ptr = name;
	if (length != 0) {
		message.dv[1].data_length = length;
		message.dv[1].data_ptr = __DECONST(void *, data);
	}
	if (code < sizeof(posted) / sizeof(posted[0])) {
		__atomic_fetch_add(&posted[code], 1, __ATOMIC_RELAXED);
	}
	kev_post_msg(&message);
}

/* What the station side says has happened (bsd/dev/fuxi_wifi_sta.c calls this, on its thread or the station's). */
void
fuxi_wifi_apple80211_event(enum fuxi_wifi_sta_event event, int status)
{
	static unsigned said;

	if (said < 200) {
		said++;
		printf("FUXI WIFI IOS: apple80211: the station says: %s, %d\n", event == FUXI_WIFI_STA_POWER ? "power" : event == FUXI_WIFI_STA_SCAN_DONE ?
		    "a scan is over" : event == FUXI_WIFI_STA_JOIN_DONE ? "a join is over" : "the link", status);
	}
	switch (event) {
	case FUXI_WIFI_STA_POWER:
		post(EVENT_POWER_CHANGED, NULL, 0);
		break;
	case FUXI_WIFI_STA_SCAN_DONE:
		lck_mtx_lock(&apple80211_lock);
		result_next = 0;
		lck_mtx_unlock(&apple80211_lock);
		post(EVENT_SCAN_DONE, NULL, 0);
		break;
	case FUXI_WIFI_STA_JOIN_DONE:
		lck_mtx_lock(&apple80211_lock);
		join_result = status == 0 ? 1 : status == EAUTH ? 21 : status == ETIMEDOUT ? 9 : 2;
		lck_mtx_unlock(&apple80211_lock);
		post(EVENT_ASSOC_DONE, NULL, 0);
		break;
	case FUXI_WIFI_STA_LINK: {
		struct apple80211_link_changed link = { .down = status != 1, .involuntary = status != 1 && status != 0,
			                                .reason = status != 1 && status != 0 ? 4 : 0 };      /* (4: gone, not asked) */
		struct fuxi_wifi_sta_state *state = kalloc_data(sizeof(*state), Z_WAITOK | Z_ZERO);

		if (state != NULL) {
			fuxi_wifi_sta_state(state);
			link.rssi = status == 1 ? state->bss.rssi : 0;
			kfree_data(state, sizeof(*state));
		}
		post(EVENT_SSID_CHANGED, NULL, 0);
		post(EVENT_BSSID_CHANGED, NULL, 0);
		post(EVENT_LINK_CHANGED, &link, sizeof(link));
		break;
	}
	}
}

/* ---- the requests as they arrive, and a summary now and then ------------------------------------------ */

/*
 * How often each kind of request came and how it fared. Said as a summary some seconds after the first
 * request and then every minute in which there was one (salami backs off: summary_tick), because the
 * lines said along the way are thinned out (a poll goes on for ever) and can be lost in a long log.
 */
#define SUMMARY_FIRST_S         20
#define SUMMARY_EVERY_S         60
#define SUMMARY_KINDS_A_LINE    5

struct kind {
	int32_t type;
	bool set;
	unsigned times, answered;
	int last_error;
};
static struct kind kinds[96];
static unsigned kind_count;
static unsigned requests, answered_all, refused_all, failed_all;   /* all of them */
static unsigned summaries, summarized_at;                           /* how many said; `requests` at the last */
static uint64_t first_request;                                      /* (absolute time) */
static thread_call_t summary_call;
static bool summary_started;

/* Count it in; the how-manieth of its kind it is (0: there are too many kinds to count it alone). */
static unsigned
note(const struct apple80211req *request, bool is_set, int error)
{
	unsigned i, times = 0;

	lck_mtx_lock(&apple80211_lock);
	if (requests++ == 0) {
		clock_get_uptime(&first_request);
	}
	answered_all += error == 0;
	refused_all += error == EOPNOTSUPP;
	failed_all += error != 0 && error != EOPNOTSUPP;
	for (i = 0; i < kind_count && !(kinds[i].type == request->req_type && kinds[i].set == is_set); i++) {
	}
	if (i < sizeof(kinds) / sizeof(kinds[0])) {
		if (i == kind_count) {
			kinds[kind_count++] = (struct kind) { .type = request->req_type, .set = is_set };
		}
		times = ++kinds[i].times;
		kinds[i].answered += error == 0;
		kinds[i].last_error = error != 0 ? error : kinds[i].last_error;
	}
	lck_mtx_unlock(&apple80211_lock);
	return times;
}

/* What was asked in this run so far, to the log; nothing if nothing was asked since it was last said. */
void
fuxi_wifi_apple80211_summary(void)
{
	struct kind *copy = kalloc_data(sizeof(kinds), Z_WAITOK);
	unsigned count, asked, answered, refused, failed, number, event[sizeof(posted) / sizeof(posted[0])];
	uint64_t start, now, elapsed_ns;
	char line[320];

	if (copy == NULL) {
		return;
	}
	lck_mtx_lock(&apple80211_lock);
	if (requests == summarized_at) {
		lck_mtx_unlock(&apple80211_lock);
		kfree_data(copy, sizeof(kinds));
		return;
	}
	summarized_at = asked = requests;
	number = ++summaries;
	count = kind_count;
	memcpy(copy, kinds, sizeof(kinds));
	answered = answered_all;
	refused = refused_all;
	failed = failed_all;
	start = first_request;
	lck_mtx_unlock(&apple80211_lock);
	for (unsigned i = 0; i < sizeof(event) / sizeof(event[0]); i++) {
		event[i] = __atomic_load_n(&posted[i], __ATOMIC_RELAXED);
	}
	clock_get_uptime(&now);
	absolutetime_to_nanoseconds(now - start, &elapsed_ns);

	printf("FUXI WIFI IOS: apple80211: summary %u, %llu s after the first request: %u requests of %u kinds, %u answered, "
	    "%u refused, %u failed; events posted: power %u, link %u, join %u, scan %u\n", number,
	    (unsigned long long)(elapsed_ns / 1000000000ull), asked, count, answered, refused, failed, event[EVENT_POWER_CHANGED],
	    event[EVENT_LINK_CHANGED], event[EVENT_ASSOC_DONE], event[EVENT_SCAN_DONE]);
	/* (each kind: get or set, the type, its name, how often/how often answered, the last error if there was one) */
	for (unsigned i = 0; i < count; i += SUMMARY_KINDS_A_LINE) {
		size_t at = 0;

		line[0] = '\0';
		for (unsigned j = i; j < count && j < i + SUMMARY_KINDS_A_LINE && at < sizeof(line) - 1; j++) {
			int n;

			if (copy[j].last_error != 0) {
				n = snprintf(line + at, sizeof(line) - at, "%s %s %d %s %u/%u (%d)", j == i ? "" : ",", copy[j].set ? "set" : "get",
				    copy[j].type, type_name(copy[j].type), copy[j].times, copy[j].answered, copy[j].last_error);
			} else {
				n = snprintf(line + at, sizeof(line) - at, "%s %s %d %s %u/%u", j == i ? "" : ",", copy[j].set ? "set" : "get",
				    copy[j].type, type_name(copy[j].type), copy[j].times, copy[j].answered);
			}
			if (n > 0) {
				at = at + (size_t)n < sizeof(line) - 1 ? at + (size_t)n : sizeof(line) - 1;
			}
		}
		printf("FUXI WIFI IOS: apple80211: summary %u:%s\n", number, line);
	}
	kfree_data(copy, sizeof(kinds));
}

#if defined(FUXI_SALAMI)
/*
 * Salami (K-W1b): the timer backs off, 20 s after the first request, 60 s after that and then every
 * 10 minutes, so that a boot left running for days is not woken every minute for it; a tenth of each
 * wait is leeway the kernel may use to fold it into another wakeup. (The Fuxi build is unchanged.)
 */
#define SUMMARY_LATER_S         600
#define SUMMARY_LEEWAY_PART     10

/* The summary's timer, `seconds` from now, with a tenth of that as leeway (a summary is never urgent). */
static void
summary_arm(thread_call_t call, uint32_t seconds)
{
	uint64_t deadline, leeway;

	clock_interval_to_deadline(seconds, NSEC_PER_SEC, &deadline);
	clock_interval_to_absolutetime_interval(seconds / SUMMARY_LEEWAY_PART, NSEC_PER_SEC, &leeway);
	(void)thread_call_enter_delayed_with_leeway(call, NULL, deadline, leeway, THREAD_CALL_DELAY_LEEWAY);
}

static void
summary_tick(__unused thread_call_param_t zero, __unused thread_call_param_t one)
{
	static bool second_said;   /* (only this callback touches it; a THREAD_CALL_OPTIONS_ONCE call never runs twice at once) */
	uint32_t next = second_said ? SUMMARY_LATER_S : SUMMARY_EVERY_S;

	second_said = true;
	fuxi_wifi_apple80211_summary();
	summary_arm(summary_call, next);
}
#else
static void
summary_tick(__unused thread_call_param_t zero, __unused thread_call_param_t one)
{
	uint64_t deadline;

	fuxi_wifi_apple80211_summary();
	clock_interval_to_deadline(SUMMARY_EVERY_S, NSEC_PER_SEC, &deadline);
	(void)thread_call_enter_delayed(summary_call, deadline);
}
#endif

/* At the first request: the summary's timer, which runs for good after that. */
static void
summary_start(void)
{
	thread_call_t call;
#if !defined(FUXI_SALAMI)
	uint64_t deadline;
#endif

	if (__atomic_load_n(&summary_started, __ATOMIC_ACQUIRE) || __atomic_exchange_n(&summary_started, true, __ATOMIC_ACQ_REL)) {
		return;
	}
	call = thread_call_allocate_with_options(summary_tick, NULL, THREAD_CALL_PRIORITY_LOW, THREAD_CALL_OPTIONS_ONCE);
	if (call == NULL) {
		printf("FUXI WIFI IOS: apple80211: no timer for the summary: it is not said\n");
		return;
	}
	summary_call = call;
#if defined(FUXI_SALAMI)
	summary_arm(call, SUMMARY_FIRST_S);
#else
	clock_interval_to_deadline(SUMMARY_FIRST_S, NSEC_PER_SEC, &deadline);
	(void)thread_call_enter_delayed(call, deadline);
#endif
}

/* An Apple80211 request for the radio's interface, from the process that is running now. */
errno_t
fuxi_wifi_apple80211_ioctl(u_long command, void *data)
{
	static unsigned said;
	struct apple80211req *request = data;
	char who[32] = "?", what[200] = "", outcome[24];
	bool is_set = command == SIOCSA80211;
	unsigned times;
	int error;

	if (command != SIOCSA80211 && command != SIOCGA80211) {
		return EOPNOTSUPP;
	}
	if (strncmp(request->req_if_name, interface_name, sizeof(interface_name)) != 0) {
		return ENXIO;
	}
	if (is_set && proc_suser(current_proc()) != 0) {
		return EPERM;
	}
	proc_selfname(who, sizeof(who));
	error = is_set ? set(request, what, sizeof(what)) : get(request, what, sizeof(what));
	times = note(request, is_set, error);
	summary_start();
	/* (each kind is said the first few times and then every hundredth: a request that is repeated for ever is not said for ever) */
	if ((times <= 24 || times % 100 == 0) && said < 4000) {
		said++;
		if (error == 0 || error == EOPNOTSUPP) {
			snprintf(outcome, sizeof(outcome), "%s", error == 0 ? "answered" : "REFUSED");
		} else {
			snprintf(outcome, sizeof(outcome), "error %d", error);
		}
		printf("FUXI WIFI IOS: apple80211: %s %s %d (%s), value %d, %u bytes%s: %s\n", who, is_set ? "sets" : "gets", request->req_type,
		    type_name(request->req_type), request->req_val, request->req_len, what, outcome);
	}
	return error;
}

#endif /* FUXI */
