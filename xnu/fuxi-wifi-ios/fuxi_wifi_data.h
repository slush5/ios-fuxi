/*
 * Fuxi Wi-Fi iOS shim: the data handoff between the radio lane and en0 (fuxiwifiios=5).
 * Original Fuxi code, 2026-10-30 (docs-lanes/wlan-join-plan.md 4, "a radio data backend for en0").
 *
 * The producer is the radio lane (bsd/dev/fuxi_wlan.c, patch 0099, a later change): it joins the network
 * asked for, says when the link is up or down, hands over the data frames it received (decrypted, one
 * 802.11 MPDU each) and takes the frames to send. The consumer is the fuxiwifiios=5 backend at the end of
 * bsd/dev/fuxi_wifi_data.c, which stands behind en0. Neither calls the other: both call the handoff in
 * bsd/dev/fuxi_wifi_data.c, which owns every shared byte and its one leaf lock. No pointer crosses: a
 * request is copied to the producer, a received frame is converted and copied during the call that hands
 * it over, a frame to send is copied in by en0 and out (converted) by the producer.
 *
 * The link, as the handoff sees it:
 *
 *   IDLE --join() posts a request--> JOINING --link_up(seq, bssid, MHz)--> UP
 *   JOINING --link_down(seq, errno), or the wait runs out, or leave()--> IDLE (the join fails)
 *   UP --link_down(seq, errno), or leave(), or ready(false)--> IDLE (the link is lost, or left)
 *
 * Every change of the link empties the queues and moves the epoch: nothing of one link reaches the next.
 * Producer calls never sleep, never touch hardware and never call into the iOS side; before use() (any
 * boot but fuxiwifiios=5) they drop everything and store only ready()'s word. A join request also means
 * "leave whatever else was joined". Every queue is bounded; when one is full
 * the newest frame is dropped and counted. The consumer never sees MMIO or DMA, and every wait it makes
 * is bounded.
 *
 * 802.11 frame layouts and the address table are IEEE 802.11-2020 9.2.4.1 and 9.3.2.1; the SNAP headers
 * RFC 1042 and IEEE 802.1H. Only the station's case: To DS 0 / From DS 1 received, To DS 1 sent.
 *
 * Nothing but C's own types here, like sys/fuxi_wifi_radio.h.
 */
#ifndef _SYS_FUXI_WIFI_DATA_H_
#define _SYS_FUXI_WIFI_DATA_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FUXI_WIFI_DATA_FRAME_MAX        1514    /* an 802.3 frame: DA, SA, type, at most 1500 bytes of payload */
#define FUXI_WIFI_DATA_MPDU_MOST        1532    /* what tx_take() makes of the longest: 24 + 8 (SNAP) + 1500 */
#define FUXI_WIFI_DATA_RX_SLOTS         32      /* received frames waiting for en0 */
#define FUXI_WIFI_DATA_EAPOL_SLOTS      8       /* received EAPOL frames waiting for a supplicant (none yet) */
#define FUXI_WIFI_DATA_TX_SLOTS         32      /* en0's frames waiting for the producer */
#define FUXI_WIFI_DATA_TX_INFLIGHT      32      /* taken by the producer and not yet done() */
#define FUXI_WIFI_DATA_JOIN_WAIT_MS     10000   /* the longest a join waits for link_up() */
#define FUXI_WIFI_DATA_ETHERTYPE_EAPOL  0x888e

enum fuxi_wifi_data_state {
	FUXI_WIFI_DATA_IDLE,                    /* no network */
	FUXI_WIFI_DATA_JOINING,                 /* a join request is posted; EAPOL from its BSSID is queued already */
	FUXI_WIFI_DATA_UP,                      /* frames pass both ways */
};

/* One join (or leave) request, as take() copies it to the producer. */
struct fuxi_wifi_data_request {
	uint32_t seq;                           /* never 0; link_up() and link_down() name it */
	bool leave;                             /* leave whatever was joined; the rest is zero */
	uint8_t ssid[32], ssid_length;
	uint8_t bssid[6];                       /* the one to join: never zero, never a group address */
	uint16_t mhz;                           /* where a scan heard it */
	int security;                           /* FUXI_WIFI_STA_OPEN (the only one until the supplicant) */
};

/* The link, as the consumer reads it. */
struct fuxi_wifi_data_link {
	enum fuxi_wifi_data_state state;
	uint32_t seq;                           /* the request it is for (0: none) */
	uint32_t epoch;                         /* moves on every change */
	uint8_t bssid[6];
	uint16_t mhz;
	int16_t rssi;                           /* of the last data frame kept; 0: none yet */
	int status;                             /* why the last link or join ended (an errno); 0: it did not */
};

/* Why a frame was not kept (0: it was). RX: fuxi_wifi_data_rx(), to_8023(). TX: send(), from_8023(). */
enum {
	FUXI_WIFI_DATA_KEPT,
	FUXI_WIFI_DATA_DROP_LENGTH,             /* RX: shorter than its header and SNAP, or over 1514 bytes as 802.3;
	                                         * TX: under 14 or over 1514 bytes, or no room for the MPDU */
	FUXI_WIFI_DATA_DROP_KIND,               /* RX: not Data or QoS Data (Null, management, control, version) */
	FUXI_WIFI_DATA_DROP_PROTECTED,          /* RX: the Protected bit set: the radio lane hands over plaintext */
	FUXI_WIFI_DATA_DROP_FRAGMENT,           /* RX: More Fragments, or a fragment number other than 0 */
	FUXI_WIFI_DATA_DROP_AMSDU,              /* RX: an A-MSDU: the radio lane hands over one MSDU at a time */
	FUXI_WIFI_DATA_DROP_DS,                 /* RX: not To DS 0 / From DS 1 (to an AP, IBSS, four addresses) */
	FUXI_WIFI_DATA_DROP_BSSID,              /* RX: Address 2 is not the BSS joined */
	FUXI_WIFI_DATA_DROP_DEST,               /* RX: neither to this station nor to a group */
	FUXI_WIFI_DATA_DROP_SOURCE,             /* RX: from a group address, or our own frame reflected by the AP;
	                                         * TX: not from this station's address */
	FUXI_WIFI_DATA_DROP_LLC,                /* RX: no RFC 1042 or bridge-tunnel SNAP header (or RFC 1042 with a type
	                                         * 802.1H tunnels); TX: an 802.3 length field, not a type */
	FUXI_WIFI_DATA_DROP_EAPOL,              /* TX: EAPOL from en0 (only a supplicant sends it) */
	FUXI_WIFI_DATA_DROP_DOWN,               /* no link (data while joining included), or no consumer */
	FUXI_WIFI_DATA_DROP_FULL,               /* the queue is full */
	FUXI_WIFI_DATA_DROP_REASONS
};

/* What passed, since use(). RX counts the producer's calls, TX en0's. */
struct fuxi_wifi_data_counts {
	uint32_t rx_frames, rx_kept, rx_eapol, rx_dropped[FUXI_WIFI_DATA_DROP_REASONS], rx_pulled, eapol_pulled, rx_flushed;
	uint32_t tx_frames, tx_queued, tx_dropped[FUXI_WIFI_DATA_DROP_REASONS], tx_taken, tx_ok, tx_failed, tx_stale,
	    tx_flushed;
	uint32_t joins, links_up, links_lost, links_left, link_stale;
};

/* ---- pure: no kernel call, no state (host tested) ---- */

/*
 * A received data MPDU (802.11 header first, no FCS) from the BSS `bssid` to the station `self`, as an
 * 802.3 frame of at most FUXI_WIFI_DATA_FRAME_MAX bytes into `out` (0 and *out_length set), else why not.
 * *eapol says whether it is EAPOL (ethertype 0x888e).
 */
int      fuxi_wifi_data_to_8023(const uint8_t *mpdu, size_t length, const uint8_t *bssid, const uint8_t *self,
    uint8_t *out, size_t *out_length, bool *eapol);
/*
 * An 802.3 frame from `self` as an MPDU to the BSS `bssid` (Data, To DS, sequence left to the firmware)
 * into at most `room` bytes of `out`. 0 and *out_length set, else why not. `out` NULL: only checked.
 */
int      fuxi_wifi_data_from_8023(const uint8_t *frame, size_t length, const uint8_t *bssid, const uint8_t *self,
    uint8_t *out, size_t room, size_t *out_length);

#ifdef KERNEL
/* ---- the producer: the radio lane's thread, one at a time. Never sleeps, no hardware ---- */
void     fuxi_wifi_data_ready(bool ready);                                     /* it takes joins (true), or no longer */
bool     fuxi_wifi_data_take(struct fuxi_wifi_data_request *out);              /* a join or leave request, copied */
void     fuxi_wifi_data_link_up(uint32_t seq, const uint8_t *bssid, uint16_t mhz);      /* joined: frames may pass */
void     fuxi_wifi_data_link_down(uint32_t seq, int status);                   /* the join failed, or the link is lost */
int      fuxi_wifi_data_rx(const uint8_t *mpdu, size_t length, int16_t rssi_dbm);       /* converted and copied */
size_t   fuxi_wifi_data_tx_take(uint8_t *mpdu, size_t room, uint32_t *id);     /* 0: nothing to send */
void     fuxi_wifi_data_tx_done(uint32_t id, int status);                      /* 0, or an errno */

/* ---- the consumer: the fuxiwifiios=5 backend ---- */
bool     fuxi_wifi_data_use(const uint8_t *self);                              /* init, once */
bool     fuxi_wifi_data_present(void);                                         /* a producer takes joins */
int      fuxi_wifi_data_join(const struct fuxi_wifi_data_request *what, unsigned timeout_ms, bool (*give_up)(void),
    struct fuxi_wifi_data_link *link);
void     fuxi_wifi_data_leave(void);
int      fuxi_wifi_data_send(const uint8_t *frame, size_t length);             /* en0's frame, copied; 0 or why not */
bool     fuxi_wifi_data_pull(uint8_t *frame, size_t room, size_t *length);     /* a received frame for en0 */
bool     fuxi_wifi_data_eapol_pull(uint8_t *frame, size_t room, size_t *length);       /* for a supplicant */
void     fuxi_wifi_data_link(struct fuxi_wifi_data_link *out);
void     fuxi_wifi_data_counts(struct fuxi_wifi_data_counts *out);

/* ---- fuxiwifiios=5: the radio scan backend of fuxi_wifi_ios.c, with joins and en0's frames from here ---- */
struct fuxi_wifi_backend;
void     fuxi_wifi_data_init(const struct fuxi_wifi_backend *scan);
#endif

#endif
