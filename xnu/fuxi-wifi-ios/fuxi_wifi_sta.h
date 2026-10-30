/*
 * Ported from HackiOS xnu/files/bsd/sys/redwood_sta.h (APSL-2.0)
 * Fuxi adaptations, 2026-10-23: opt-in iOS shim; see docs-lanes/wifi-ios.md.
 * Fuxi, 2026-10-30 (K2): a record's noise figure; the radio backends' Wi-Fi off and power-off check,
 * and the name they give Apple80211 (sys/fuxi_wifi_radio.h).
 * Original file commentary is retained below. See LICENSE.HackiOS.
 */
/*
 * Darwin-on-redwood: the station boundary.
 *
 * On one side whatever does the scanning and the joining with a radio: the station logic over
 * ath11k on the phone (wifi/lx in the repository). On the other whoever wants a network: the
 * Apple80211 requests of iOS's Wi-Fi side, answered in bsd/dev/fuxi_wifi_apple80211.c, and the
 * kernel's own join of the network it was given in a file (bsd/dev/redwood_wifi.c). Frames do
 * not pass here, and neither does "the link is up": those go through bsd/sys/fuxi_wifi_net.h,
 * which the station side calls itself.
 *
 * Between the two stands bsd/dev/fuxi_wifi_sta.c: one thread, on which every call into the station
 * side is made, one at a time, and what the last scan heard, kept for whoever asks.
 *
 * Nothing but C's own types here: the station side is not compiled as XNU's code.
 */
#ifndef _SYS_FUXI_WIFI_STA_H_
#define _SYS_FUXI_WIFI_STA_H_

#include <stdbool.h>
#include <stdint.h>

#define FUXI_WIFI_STA_ELEMENTS_MOST       1024

/* One access point, as a scan heard it. */
struct fuxi_wifi_sta_bss {
	uint8_t bssid[6];
	uint8_t ssid[32], ssid_length;
	uint16_t frequency;                     /* MHz */
	int16_t rssi;                           /* dBm */
	int16_t noise;                          /* dBm; 0: not known */
	uint16_t beacon_interval;               /* in units of 1024 microseconds */
	uint16_t capability;                    /* the beacon's capability field */
	uint32_t age_ms;                        /* how long ago it was last heard */
	uint16_t elements_length;               /* the beacon's information elements as received: who asks */
	uint8_t elements[FUXI_WIFI_STA_ELEMENTS_MOST];    /* reads the network's protection out of them itself */
};

/* What to scan for. All zero: every channel, the radio's own way. */
struct fuxi_wifi_sta_scan {
	uint8_t ssid[32], ssid_length;          /* ask for this network by name (a hidden one answers only then) */
	uint16_t frequencies[64], frequency_count;      /* only these channels */
	bool passive;                           /* only listen */
	uint16_t dwell_ms;                      /* on each channel; 0: the radio's own */
};

enum { FUXI_WIFI_STA_OPEN, FUXI_WIFI_STA_WPA2_PSK };

struct fuxi_wifi_sta_join {
	uint8_t ssid[32], ssid_length;
	uint8_t bssid[6];                       /* all zero: the access point of that name heard best */
	int security;
	uint8_t pmk[32];                        /* WPA2 with a passphrase: the key made of the passphrase and the name ... */
	uint8_t passphrase[64], passphrase_length;      /* ... or, if this is not empty, the passphrase itself */
};

enum fuxi_wifi_sta_phase { FUXI_WIFI_STA_OFF, FUXI_WIFI_STA_IDLE, FUXI_WIFI_STA_SCANNING, FUXI_WIFI_STA_JOINING, FUXI_WIFI_STA_JOINED };

struct fuxi_wifi_sta_state {
	enum fuxi_wifi_sta_phase phase;
	struct fuxi_wifi_sta_bss bss;             /* the network joined or being joined */
	int security;                          /* FUXI_WIFI_STA_OPEN or FUXI_WIFI_STA_WPA2_PSK */
	int last_error;                         /* why the last join failed or the network was lost (an errno); 0: it was not */
};

#define FUXI_WIFI_STA_CHANNEL_PASSIVE     1u      /* the radio may only listen there */
#define FUXI_WIFI_STA_CHANNEL_RADAR       2u      /* ... and has to look out for radar */

struct fuxi_wifi_sta_channel {
	uint16_t frequency;                     /* MHz */
	uint16_t flags;
};

/*
 * What the station side supplies. All but state() are called on one thread, one at a time, and
 * may take as long as the thing takes: scan() returns when the scan is over, join() when frames
 * can pass or it is known why not. Each gives 0 or an errno.
 */
struct fuxi_wifi_sta_ops {
	int (*power)(bool on);                  /* off: leave the network, stop the radio; on: start it */
	int (*scan)(const struct fuxi_wifi_sta_scan *what);
	void (*heard)(void (*each)(const struct fuxi_wifi_sta_bss *bss, void *context), void *context);   /* by the last scan */
	int (*join)(const struct fuxi_wifi_sta_join *what);
	int (*leave)(void);
	void (*state)(struct fuxi_wifi_sta_state *state);         /* on any thread; does not sleep */
	unsigned (*channels)(struct fuxi_wifi_sta_channel *channels, unsigned most);      /* what the radio may tune to */
	uint8_t address[6];
};

/* What happens, said to whoever listens (the Apple80211 side). */
enum fuxi_wifi_sta_event {
	FUXI_WIFI_STA_POWER,                      /* status: 1 on, 0 off */
	FUXI_WIFI_STA_SCAN_DONE,                  /* status: 0, or the scan's errno */
	FUXI_WIFI_STA_JOIN_DONE,                  /* status: 0, or the join's errno */
	FUXI_WIFI_STA_LINK,                       /* status: 1 a network is joined, 0 it has been left when asked,
	                                         * an errno: it was lost (the access point showed us the door) */
};

#ifdef KERNEL
/* ---- the station side ---- */
void     fuxi_wifi_sta_attach(const struct fuxi_wifi_sta_ops *ops);         /* init only; immutable boot-lifetime backend */
void     fuxi_wifi_sta_lost(int error);                                   /* the network joined is gone by itself */
void     fuxi_wifi_sta_radio_off(void);                                   /* the radio is gone by itself: off until reboot */
bool     fuxi_wifi_sta_power_pending(void);                               /* "off" is asked for and not begun (a scan gives up) */

/* ---- who wants a network: each asks and returns; the outcome is an event. 0, or an errno ---- */
bool     fuxi_wifi_sta_present(void);
int      fuxi_wifi_sta_ask_power(bool on);
int      fuxi_wifi_sta_ask_scan(const struct fuxi_wifi_sta_scan *what);
int      fuxi_wifi_sta_ask_join(const struct fuxi_wifi_sta_join *what);
int      fuxi_wifi_sta_ask_leave(void);
bool     fuxi_wifi_sta_busy(void);                                        /* something asked for is not over yet */
bool     fuxi_wifi_sta_powered(void);
/* the `index`th access point the last scan heard: true, or false when there are not that many */
bool     fuxi_wifi_sta_heard(unsigned index, struct fuxi_wifi_sta_bss *bss);
void     fuxi_wifi_sta_state(struct fuxi_wifi_sta_state *state);
unsigned fuxi_wifi_sta_channels(struct fuxi_wifi_sta_channel *channels, unsigned most);
bool     fuxi_wifi_sta_address(uint8_t *address);

/* ---- the listener: bsd/dev/fuxi_wifi_apple80211.c ---- */
void     fuxi_wifi_apple80211_event(enum fuxi_wifi_sta_event event, int status);
void     fuxi_wifi_apple80211_summary(void);                              /* what was asked in this run so far, to the log
                                                                         * (a timer says it every minute while requests come) */
/* init only, the radio backends: their name (GET 43/44), and a scan's records carry real elements, kept whole */
void     fuxi_wifi_apple80211_backend(const char *name, bool whole_elements);
#endif

#endif
