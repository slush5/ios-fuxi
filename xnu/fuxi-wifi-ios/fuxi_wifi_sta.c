/*
 * Ported from HackiOS xnu/files/bsd/dev/redwood_sta.c (APSL-2.0)
 * Fuxi adaptations, 2026-10-23: opt-in iOS shim; see docs-lanes/wifi-ios.md.
 * Fuxi, 2026-10-30 (K2): a radio that stops by itself turns the station off for the boot
 * (fuxi_wifi_sta_radio_off), and a scan can ask whether "off" waits (fuxi_wifi_sta_power_pending).
 * Original file commentary is retained below. See LICENSE.HackiOS.
 */
/*
 * Darwin-on-redwood: what stands between the two sides of the station boundary
 * (sys/fuxi_wifi_sta.h).
 *
 * Whoever wants a network asks from wherever it happens to be: an ioctl of wifid's, the thread
 * that brings the radio up. The station side wants to be called by one thread, one call at a
 * time, and a scan or a join takes seconds. So what is asked for is noted here and done by a
 * thread of this file's; the asker returns at once and hears the outcome as an event. One
 * request of each kind can wait: a second scan asked for while the first has not begun
 * replaces it, as does a second join.
 *
 * What the last scan heard is kept here, so that it can be read without entering the station
 * side, and so is the list of channels.
 *
 * Fuxi omits the root-file standing/fallback network: wifid owns rejoin policy.
 */
#if defined(FUXI)
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/errno.h>
#include <sys/fuxi_wifi_sta.h>
#include <sys/systm.h>

extern void thread_set_thread_name(thread_t thread, const char *name);

#define HEARD_MOST      128
#define CHANNELS_MOST   96

static LCK_GRP_DECLARE(sta_lock_group, "fuxi-wifi-sta");
static LCK_MTX_DECLARE(sta_lock, &sta_lock_group);

static const struct fuxi_wifi_sta_ops *sta_ops;   /* the station side, while there is one */
static bool sta_thread_made;
static bool sta_calls;                          /* the thread is in a call of the station side's */
static bool sta_on;                             /* the radio's power, as last asked for */
static bool sta_gone;                           /* the radio stopped by itself: it stays off until reboot */

/* what has been asked for and not begun */
static bool asked_power, asked_power_on, asked_scan, asked_join, asked_leave;
static struct fuxi_wifi_sta_scan asked_scan_what;
static struct fuxi_wifi_sta_join asked_join_what;

static struct fuxi_wifi_sta_bss *heard;           /* by the last scan that came to an end */
static unsigned heard_count;
static struct fuxi_wifi_sta_channel channels[CHANNELS_MOST];
static unsigned channel_count;

static void
heard_one(const struct fuxi_wifi_sta_bss *bss, __unused void *context)
{
	/* (only the thread writes the list, and it reads the count without the lock) */
	if (bss->ssid_length <= sizeof(bss->ssid) &&
	    bss->elements_length <= sizeof(bss->elements) && heard_count < HEARD_MOST) {
		lck_mtx_lock(&sta_lock);
		heard[heard_count++] = *bss;
		lck_mtx_unlock(&sta_lock);
	}
}

static void
sta_runs(__unused void *parameter, __unused wait_result_t result)
{
	static struct fuxi_wifi_sta_scan scan;
	static struct fuxi_wifi_sta_join join;

	for (;;) {
		const struct fuxi_wifi_sta_ops *ops;
		bool power = false, power_on = false, leave = false, do_join = false, do_scan = false;
		int error = 0;

		lck_mtx_lock(&sta_lock);
		while (sta_ops == NULL || !(asked_power || asked_leave || asked_join || asked_scan)) {
			lck_mtx_sleep(&sta_lock, LCK_SLEEP_DEFAULT, (event_t)&sta_ops, THREAD_UNINT);
		}
		ops = sta_ops;
		/* one thing at a time, in the order that makes sense when several wait */
		if (asked_power) {
			power = true;
			power_on = asked_power_on;
			asked_power = false;
		} else if (asked_leave) {
			leave = true;
			asked_leave = false;
		} else if (asked_join) {
			do_join = true;
			join = asked_join_what;
			bzero(&asked_join_what, sizeof(asked_join_what));
			asked_join = false;
		} else {
			do_scan = true;
			scan = asked_scan_what;
			asked_scan = false;
		}
		sta_calls = true;
		lck_mtx_unlock(&sta_lock);

		if (power) {
			struct fuxi_wifi_sta_state *state = kalloc_data(sizeof(*state), Z_WAITOK | Z_ZERO);
			bool was_joined = false, on;

			if (state != NULL) {
				ops->state(state);
				was_joined = state->phase == FUXI_WIFI_STA_JOINED;
				kfree_data(state, sizeof(*state));
			}
			error = ops->power(power_on);
			lck_mtx_lock(&sta_lock);
			on = (error == 0 ? power_on : !power_on) && !sta_gone;
			if (!asked_power) {
				sta_on = on;
			}
			lck_mtx_unlock(&sta_lock);
			if (!power_on && was_joined) {
				fuxi_wifi_apple80211_event(FUXI_WIFI_STA_LINK, 0);
			}
			fuxi_wifi_apple80211_event(FUXI_WIFI_STA_POWER, on);
		} else if (leave) {
			ops->leave();
			fuxi_wifi_apple80211_event(FUXI_WIFI_STA_LINK, 0);
		} else if (do_join) {
			error = ops->join(&join);
			bzero(&join, sizeof(join));
			fuxi_wifi_apple80211_event(FUXI_WIFI_STA_JOIN_DONE, error);
			if (error == 0) {
				fuxi_wifi_apple80211_event(FUXI_WIFI_STA_LINK, 1);
			}
		} else if (do_scan) {
			error = ops->scan(&scan);
			lck_mtx_lock(&sta_lock);
			heard_count = 0;
			lck_mtx_unlock(&sta_lock);
			ops->heard(heard_one, NULL);
			fuxi_wifi_apple80211_event(FUXI_WIFI_STA_SCAN_DONE, error);
		}

		lck_mtx_lock(&sta_lock);
		sta_calls = false;
		lck_mtx_unlock(&sta_lock);
		thread_wakeup((event_t)&sta_calls);
	}
}

/* ---- the station side ---------------------------------------------------------------------------------- */

void
fuxi_wifi_sta_attach(const struct fuxi_wifi_sta_ops *ops)
{
	thread_t thread;

	if (ops == NULL || sta_ops != NULL) {
		return;
	}
	if (heard == NULL) {
		heard = kalloc_data(HEARD_MOST * sizeof(*heard), Z_WAITOK | Z_ZERO);
	}
	if (heard == NULL) {
		printf("wifi: no memory for what a scan hears: nobody can ask the radio for a network\n");
		return;
	}
	if (!sta_thread_made) {
		if (kernel_thread_start(sta_runs, NULL, &thread) != KERN_SUCCESS) {
			printf("wifi: no thread for the station: nobody can ask the radio for a network\n");
			return;
		}
		thread_set_thread_name(thread, "wifi-station");
		thread_deallocate(thread);
		sta_thread_made = true;
	}
	channel_count = ops->channels(channels, CHANNELS_MOST);
	if (channel_count > CHANNELS_MOST) {
		channel_count = CHANNELS_MOST;
	}
	lck_mtx_lock(&sta_lock);
	sta_ops = ops;
	sta_on = true;
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	fuxi_wifi_apple80211_event(FUXI_WIFI_STA_POWER, 1);
}

void
fuxi_wifi_sta_lost(int error)
{
	fuxi_wifi_apple80211_event(FUXI_WIFI_STA_LINK, error != 0 ? error : ECONNRESET);
}

/*
 * (Fuxi, K2) The radio behind the station stopped by itself (sys/fuxi_wifi_radio.h): the station is off,
 * what waited is dropped, iOS hears "power off", and asking for "on" again leaves it off until reboot.
 */
void
fuxi_wifi_sta_radio_off(void)
{
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL) {
		lck_mtx_unlock(&sta_lock);
		return;
	}
	sta_gone = true;
	sta_on = false;
	asked_scan = asked_join = false;
	bzero(&asked_join_what, sizeof(asked_join_what));
	lck_mtx_unlock(&sta_lock);
	fuxi_wifi_apple80211_event(FUXI_WIFI_STA_POWER, 0);
}

/* (Fuxi, K2) "Off" is asked for and the thread has not begun it: a scan that waits gives up. */
bool
fuxi_wifi_sta_power_pending(void)
{
	bool off;

	lck_mtx_lock(&sta_lock);
	off = asked_power && !asked_power_on;
	lck_mtx_unlock(&sta_lock);
	return off;
}

/* ---- who wants a network ------------------------------------------------------------------------------- */

bool
fuxi_wifi_sta_present(void)
{
	bool present;
	lck_mtx_lock(&sta_lock);
	present = sta_ops != NULL;
	lck_mtx_unlock(&sta_lock);
	return present;
}

bool
fuxi_wifi_sta_powered(void)
{
	bool on;
	lck_mtx_lock(&sta_lock);
	on = sta_ops != NULL && sta_on;
	lck_mtx_unlock(&sta_lock);
	return on;
}

bool
fuxi_wifi_sta_busy(void)
{
	bool busy;

	lck_mtx_lock(&sta_lock);
	busy = sta_ops != NULL && (sta_calls || asked_power || asked_leave || asked_join || asked_scan);
	lck_mtx_unlock(&sta_lock);
	return busy;
}

int
fuxi_wifi_sta_ask_power(bool on)
{
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL) {
		lck_mtx_unlock(&sta_lock);
		return ENXIO;
	}
	sta_on = on && !sta_gone;
	asked_power = true;
	asked_power_on = on;
	if (!on) {
		asked_scan = asked_join = asked_leave = false;
		bzero(&asked_join_what, sizeof(asked_join_what));
	}
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

int
fuxi_wifi_sta_ask_scan(const struct fuxi_wifi_sta_scan *what)
{
	if (what == NULL || what->ssid_length > sizeof(what->ssid) ||
	    what->frequency_count > sizeof(what->frequencies) / sizeof(what->frequencies[0])) {
		return EINVAL;
	}
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL || !sta_on) {
		lck_mtx_unlock(&sta_lock);
		return sta_ops == NULL ? ENXIO : ENETDOWN;
	}
	asked_scan_what = *what;
	asked_scan = true;
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

int
fuxi_wifi_sta_ask_join(const struct fuxi_wifi_sta_join *what)
{
	if (what == NULL || what->ssid_length == 0 || what->ssid_length > sizeof(what->ssid) ||
	    what->passphrase_length > 63 || (what->security != FUXI_WIFI_STA_OPEN &&
	    what->security != FUXI_WIFI_STA_WPA2_PSK)) {
		return EINVAL;
	}
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL || !sta_on) {
		lck_mtx_unlock(&sta_lock);
		return sta_ops == NULL ? ENXIO : ENETDOWN;
	}
	asked_join_what = *what;
	asked_join = true;
	asked_leave = false;
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

int
fuxi_wifi_sta_ask_leave(void)
{
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL) {
		lck_mtx_unlock(&sta_lock);
		return ENXIO;
	}
	asked_leave = true;
	asked_join = false;
	bzero(&asked_join_what, sizeof(asked_join_what));
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

bool
fuxi_wifi_sta_heard(unsigned index, struct fuxi_wifi_sta_bss *bss)
{
	bool there;

	lck_mtx_lock(&sta_lock);
	if ((there = index < heard_count)) {
		*bss = heard[index];
	}
	lck_mtx_unlock(&sta_lock);
	return there;
}

void
fuxi_wifi_sta_state(struct fuxi_wifi_sta_state *state)
{
	const struct fuxi_wifi_sta_ops *ops;

	bzero(state, sizeof(*state));
	lck_mtx_lock(&sta_lock);
	ops = sta_ops;
	lck_mtx_unlock(&sta_lock);
	if (ops != NULL) {
		ops->state(state);
		if (state->bss.ssid_length > sizeof(state->bss.ssid) ||
		    state->bss.elements_length > sizeof(state->bss.elements)) {
			bzero(state, sizeof(*state));
		}
	}
}

unsigned
fuxi_wifi_sta_channels(struct fuxi_wifi_sta_channel *out, unsigned most)
{
	unsigned count;

	lck_mtx_lock(&sta_lock);
	count = channel_count < most ? channel_count : most;
	memcpy(out, channels, count * sizeof(*out));
	lck_mtx_unlock(&sta_lock);
	return count;
}

bool
fuxi_wifi_sta_address(uint8_t *address)
{
	const struct fuxi_wifi_sta_ops *ops = sta_ops;

	if (ops != NULL) {
		memcpy(address, ops->address, 6);
	}
	return ops != NULL;
}

#endif /* FUXI */
