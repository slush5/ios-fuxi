/*
 * Fuxi Wi-Fi iOS shim: the scan handoff between the radio lane and the iOS side.
 * Original Fuxi code, 2026-10-30 (K2: wlan/r1 overnight_plan 3.2, ios_scan_plan 1.2-2.5).
 *
 * The producer is the radio lane (bsd/dev/fuxi_wlan.c, patch 0099), or the canned producer of
 * bsd/dev/fuxi_wifi_radio.c with fuxiwifiios=4. The consumer is the radio backend of
 * bsd/dev/fuxi_wifi_ios.c (fuxiwifiios=3 and =4). Neither calls the other: both call
 * bsd/dev/fuxi_wifi_radio.c, which owns every shared byte and the one shared lock. No pointer
 * crosses: a request is copied to the producer, a frame is parsed and copied during the call that
 * hands it over, and the records are copied to the consumer one at a time.
 *
 * The state, one word for the boot:
 *
 *   ABSENT --starting()--> STARTING --ready()--> READY
 *   any ------stopped(code)------> STOPPED (terminal: Wi-Fi is off until the next boot)
 *
 * Producer calls never sleep, never touch hardware and never call into the iOS side; before the
 * consumer exists (or without fuxiwifiios=3) they only store the state word. With fuxiwifiios=4
 * the radio lane's calls are ignored. The consumer never sees MMIO or DMA, and every wait it
 * makes is bounded.
 *
 * Nothing but C's own types here, like sys/fuxi_wifi_sta.h.
 */
#ifndef _SYS_FUXI_WIFI_RADIO_H_
#define _SYS_FUXI_WIFI_RADIO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/fuxi_wifi_sta.h>

#define FUXI_WIFI_RADIO_BSS_MOST        64      /* records one scan keeps (a stronger one replaces the weakest) */
#define FUXI_WIFI_RADIO_FREQS_MOST      64      /* = fuxi_wifi_sta_scan.frequencies */
#define FUXI_WIFI_RADIO_CHANNELS        38      /* the one passive list of both sides: fuxi_wifi_radio_channels() */
#define FUXI_WIFI_RADIO_DWELL_MS        150     /* passive dwell per channel (Linux ath12k wmi.c, BSD-3-Clause-Clear) */
#define FUXI_WIFI_RADIO_NOISE_DBM       (-95)   /* the noise floor when the radio has none (Linux ath12k core.h) */
#define FUXI_WIFI_RADIO_WAIT_MOST_MS    15000   /* the longest a scan is waited for */

enum fuxi_wifi_radio_state {
	FUXI_WIFI_RADIO_ABSENT,                 /* the producer has said nothing yet */
	FUXI_WIFI_RADIO_STARTING,               /* it runs; no scan yet */
	FUXI_WIFI_RADIO_READY,                  /* it takes scan requests */
	FUXI_WIFI_RADIO_STOPPED,                /* it stopped, or the iOS side gave up on it */
};

/* One scan, as take() copies it to the producer. */
struct fuxi_wifi_radio_request {
	uint32_t seq;                                                   /* never 0 */
	uint16_t freqs[FUXI_WIFI_RADIO_FREQS_MOST], freq_count;         /* MHz, never 0 of them */
	uint8_t ssid[32], ssid_length;                                  /* wifid's; unused while passive */
	bool passive;                                                   /* always true until the active milestone */
	uint16_t dwell_ms;                                              /* on each channel */
};

/*
 * Stop codes: class << 16 | detail. Classes 1 to 4 are the radio lane's (EXITED: fw_run's step << 8 |
 * reason in the detail); 5 to 7 are the iOS side's own (detail: 1 iOS asked first, 2 the 30 s backstop;
 * the number of timeouts in a row; how many channels ready() named).
 */
#define FUXI_WIFI_RADIO_NOT_REQUESTED   (1u << 16)      /* no fuxiwlan on this boot */
#define FUXI_WIFI_RADIO_REFUSED         (2u << 16)      /* the radio lane refused its knobs at init */
#define FUXI_WIFI_RADIO_NO_THREAD       (3u << 16)      /* no thread for the radio lane (or for the canned producer) */
#define FUXI_WIFI_RADIO_EXITED          (4u << 16)      /* the radio lane's thread ended, the device contained */
#define FUXI_WIFI_RADIO_NO_PRODUCER     (5u << 16)      /* fuxiwifiios=3 and nothing said by the producer */
#define FUXI_WIFI_RADIO_GAVE_UP         (6u << 16)      /* scans timed out 3 times in a row */
#define FUXI_WIFI_RADIO_NO_CHANNEL      (7u << 16)      /* ready() named none of the listed channels */

/* Why parse(), frame() or bss() kept no record (0: one was kept). */
enum {
	FUXI_WIFI_RADIO_KEPT,
	FUXI_WIFI_RADIO_DROP_LENGTH,            /* shorter than the header and the fixed fields, or over 64 KiB */
	FUXI_WIFI_RADIO_DROP_KIND,              /* not a beacon or a probe response, or DS bits or protected */
	FUXI_WIFI_RADIO_DROP_BSSID,             /* multicast or zero */
	FUXI_WIFI_RADIO_DROP_IBSS,              /* the ESS bit is clear */
	FUXI_WIFI_RADIO_DROP_SSID,              /* no whole SSID element, or one over 32 bytes */
	FUXI_WIFI_RADIO_DROP_BAND,              /* not heard at 2.4 or 5 GHz (6 GHz is not reported) */
	FUXI_WIFI_RADIO_DROP_CHANNEL,           /* a DS or HT operation channel out of range */
	FUXI_WIFI_RADIO_DROP_UNLISTED,          /* not a channel the producer said it may use */
	FUXI_WIFI_RADIO_DROP_STALE,             /* no request in flight with that seq (or the radio lane in canned mode) */
	FUXI_WIFI_RADIO_DROP_FULL,              /* 64 records, all at least as strong */
	FUXI_WIFI_RADIO_DROP_REASONS
};

/* ---- pure: no kernel call, no state (host tested) ---- */

/*
 * A beacon or probe response as received (802.11 header first, no FCS), heard at rx_mhz: into `out`
 * when it is kept (0), else why not. `allowed` lists the frequencies a record may have.
 */
int      fuxi_wifi_radio_parse(const uint8_t *frame, size_t length, uint16_t rx_mhz, int16_t rssi_dbm,
    int16_t noise_dbm, const uint16_t *allowed, unsigned allowed_count, struct fuxi_wifi_sta_bss *out);
/* Whole elements of `in` into at most `room` bytes of `out`, the priority set first, in their order; how many bytes. */
unsigned fuxi_wifi_elements_fit(const uint8_t *in, unsigned length, uint8_t *out, unsigned room);
/* The 38 passive channels (2.4 GHz 1-13, 5 GHz 36-165, 52-144 with radar); how many were given. */
unsigned fuxi_wifi_radio_channels(struct fuxi_wifi_sta_channel *out, unsigned most);

#ifdef KERNEL
/* ---- the producer: one thread (the radio lane's, or the canned producer). Never sleeps, no hardware ---- */
void     fuxi_wifi_radio_starting(void);
void     fuxi_wifi_radio_ready(const uint16_t *freqs, unsigned count);         /* the channels the firmware may use */
bool     fuxi_wifi_radio_take(struct fuxi_wifi_radio_request *out);            /* a new request, copied */
bool     fuxi_wifi_radio_cancelled(uint32_t seq);                              /* stop scanning for it */
int      fuxi_wifi_radio_frame(uint32_t seq, const uint8_t *frame, size_t length, uint16_t rx_mhz, int16_t rssi_dbm,
    int16_t noise_dbm);                                                        /* parsed and copied during the call */
int      fuxi_wifi_radio_bss(uint32_t seq, const struct fuxi_wifi_sta_bss *bss);       /* a record parsed elsewhere */
void     fuxi_wifi_radio_done(uint32_t seq, int status);                       /* 0, or an errno */
void     fuxi_wifi_radio_stopped(unsigned code);                               /* terminal, idempotent */

/* ---- the consumer: the fuxiwifiios=3/4 backend (bsd/dev/fuxi_wifi_ios.c) ---- */
bool     fuxi_wifi_radio_use(bool canned, void (*changed)(enum fuxi_wifi_radio_state state));   /* init, once */
void     fuxi_wifi_radio_attached(void);                                       /* after fuxi_wifi_ios_attach() */
int      fuxi_wifi_radio_scan(const struct fuxi_wifi_radio_request *what, unsigned timeout_ms, bool (*give_up)(void));
void     fuxi_wifi_radio_cancel(void);
unsigned fuxi_wifi_radio_results(void (*each)(const struct fuxi_wifi_sta_bss *bss, void *context), void *context);
enum fuxi_wifi_radio_state fuxi_wifi_radio_state(bool *scanning);
#endif

#endif
