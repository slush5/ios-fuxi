/*
 * Fuxi Wi-Fi iOS shim: the data handoff (sys/fuxi_wifi_data.h) and the fuxiwifiios=5 backend.
 * Original Fuxi code, 2026-10-30 (docs-lanes/wlan-join-plan.md 4).
 *
 * What stands between a radio that has joined a network and en0:
 *   - the link (state, the request it is for, its epoch, the BSSID and channel), the request mailbox, the
 *     three queues' indices, the frames in flight and the counters, under data_lock. It is a leaf lock: while
 *     it is held only plain loads and stores and memcpy/bzero/memcmp happen, a wait is only asserted
 *     (assert_wait_timeout, then unlock, then thread_block), and helpers called under it end in _locked
 *     (tools/test-fuxi-wifi-data.py checks);
 *   - the store: three rings of 802.3 frames (received for en0, received EAPOL for a supplicant, en0's to
 *     send) and the producer's two frames of scratch, allocated by use() only (fuxiwifiios=5), never freed;
 *   - the conversion both ways, pure, run outside the lock on the producer's scratch: a received MPDU is
 *     converted before it is queued, a frame to send when the producer takes it.
 * Nothing here points at radio DMA, so the radio lane's containment can leave nothing dangling here.
 *
 * Frame layouts are facts of IEEE 802.11-2020 (9.2.4.1 Frame Control, 9.2.4.4 Sequence Control, 9.2.4.5 QoS
 * Control, 9.3.2.1 the data frame and its address table), RFC 1042 and IEEE 802.1H (the bridge-tunnel
 * encapsulation for AppleTalk ARP and IPX); no other code was used. The log carries counts, MHz and errno
 * only: never an SSID, a MAC address or a frame byte.
 */
#if defined(FUXI)
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/errno.h>
#include <sys/fuxi_wifi_data.h>
#include <sys/fuxi_wifi_ios.h>
#include <sys/fuxi_wifi_radio.h>
#include <sys/fuxi_wifi_sta.h>
#include <sys/systm.h>

extern void thread_set_thread_name(thread_t thread, const char *name);

/* ---- pure: 802.11 <-> 802.3 ------------------------------------------------------------------------------ */

static const uint8_t data_rfc1042[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };
static const uint8_t data_tunnel[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0xf8 };

/* IEEE 802.1H: the two types sent with the bridge-tunnel header (AppleTalk ARP, IPX), every other with RFC 1042 */
static bool
data_tunnelled(unsigned type)
{
	return type == 0x80f3 || type == 0x8137;
}

int
fuxi_wifi_data_to_8023(const uint8_t *mpdu, size_t length, const uint8_t *bssid, const uint8_t *self, uint8_t *out,
    size_t *out_length, bool *eapol)
{
	unsigned header = 24, subtype, type;
	const uint8_t *body;
	size_t payload;

	if (out_length != NULL) {
		*out_length = 0;
	}
	if (eapol != NULL) {
		*eapol = false;
	}
	if (mpdu == NULL || bssid == NULL || self == NULL || out == NULL || out_length == NULL || eapol == NULL ||
	    length < 24) {
		return FUXI_WIFI_DATA_DROP_LENGTH;
	}
	/* Frame Control: version 0, type Data, subtype Data (0) or QoS Data (8); Null and the CF subtypes carry nothing */
	subtype = mpdu[0] >> 4;
	if ((mpdu[0] & 0x0f) != 0x08 || (subtype != 0 && subtype != 8)) {
		return FUXI_WIFI_DATA_DROP_KIND;
	}
	if ((mpdu[1] & 0x40) != 0) {
		return FUXI_WIFI_DATA_DROP_PROTECTED;
	}
	if ((mpdu[1] & 0x04) != 0 || (mpdu[22] & 0x0f) != 0) {
		return FUXI_WIFI_DATA_DROP_FRAGMENT;
	}
	/* a station hears its AP: To DS 0, From DS 1. DA = Address 1, BSSID = Address 2, SA = Address 3 */
	if ((mpdu[1] & 0x03) != 0x02) {
		return FUXI_WIFI_DATA_DROP_DS;
	}
	if (subtype == 8) {
		header += 2;                            /* QoS Control */
		if ((mpdu[1] & 0x80) != 0) {
			header += 4;                    /* the Order bit of a QoS Data frame: HT Control follows */
		}
	}
	if (length < header + 8) {
		return FUXI_WIFI_DATA_DROP_LENGTH;
	}
	if (subtype == 8 && (mpdu[24] & 0x80) != 0) {
		return FUXI_WIFI_DATA_DROP_AMSDU;
	}
	if (memcmp(mpdu + 10, bssid, 6) != 0) {
		return FUXI_WIFI_DATA_DROP_BSSID;
	}
	if ((mpdu[4] & 1) == 0 && memcmp(mpdu + 4, self, 6) != 0) {
		return FUXI_WIFI_DATA_DROP_DEST;
	}
	if ((mpdu[16] & 1) != 0 || memcmp(mpdu + 16, self, 6) == 0) {
		return FUXI_WIFI_DATA_DROP_SOURCE;
	}
	body = mpdu + header;
	type = (unsigned)body[6] << 8 | body[7];
	if (type < 0x0600 || !((memcmp(body, data_rfc1042, 6) == 0 && !data_tunnelled(type)) ||
	    memcmp(body, data_tunnel, 6) == 0)) {
		return FUXI_WIFI_DATA_DROP_LLC;
	}
	payload = length - header - 8;
	if (payload > FUXI_WIFI_DATA_FRAME_MAX - 14) {
		return FUXI_WIFI_DATA_DROP_LENGTH;
	}
	memcpy(out, mpdu + 4, 6);
	memcpy(out + 6, mpdu + 16, 6);
	out[12] = (uint8_t)(type >> 8);
	out[13] = (uint8_t)type;
	memcpy(out + 14, body + 8, payload);
	*out_length = 14 + payload;
	*eapol = type == FUXI_WIFI_DATA_ETHERTYPE_EAPOL;
	return FUXI_WIFI_DATA_KEPT;
}

int
fuxi_wifi_data_from_8023(const uint8_t *frame, size_t length, const uint8_t *bssid, const uint8_t *self, uint8_t *out,
    size_t room, size_t *out_length)
{
	unsigned type;
	size_t made;

	if (out_length != NULL) {
		*out_length = 0;
	}
	if (frame == NULL || bssid == NULL || self == NULL || out_length == NULL || length < 14 ||
	    length > FUXI_WIFI_DATA_FRAME_MAX) {
		return FUXI_WIFI_DATA_DROP_LENGTH;
	}
	type = (unsigned)frame[12] << 8 | frame[13];
	if (type < 0x0600) {
		return FUXI_WIFI_DATA_DROP_LLC;
	}
	if (memcmp(frame + 6, self, 6) != 0) {
		return FUXI_WIFI_DATA_DROP_SOURCE;
	}
	made = 24 + 8 + (length - 14);
	if (out == NULL) {
		*out_length = made;
		return FUXI_WIFI_DATA_KEPT;
	}
	if (room < made) {
		return FUXI_WIFI_DATA_DROP_LENGTH;
	}
	/* Data, To DS; duration and sequence 0, the firmware's. Address 1 = BSSID, 2 = SA (us), 3 = DA */
	bzero(out, 24);
	out[0] = 0x08;
	out[1] = 0x01;
	memcpy(out + 4, bssid, 6);
	memcpy(out + 10, self, 6);
	memcpy(out + 16, frame, 6);
	memcpy(out + 24, data_tunnelled(type) ? data_tunnel : data_rfc1042, 6);
	out[30] = frame[12];
	out[31] = frame[13];
	memcpy(out + 32, frame + 14, length - 14);
	*out_length = made;
	return FUXI_WIFI_DATA_KEPT;
}

/* ---- the shared state --------------------------------------------------------------------------------- */

#define DATA_POLL_MS    10      /* a waiting join looks at itself (and at a power-off asked for) this often */

static LCK_GRP_DECLARE(data_lock_group, "fuxi-wifi-data");
static LCK_MTX_DECLARE(data_lock, &data_lock_group);

struct data_slot {
	uint16_t length;
	uint8_t frame[FUXI_WIFI_DATA_FRAME_MAX];
};

struct data_store {
	struct data_slot rx[FUXI_WIFI_DATA_RX_SLOTS], eapol[FUXI_WIFI_DATA_EAPOL_SLOTS], tx[FUXI_WIFI_DATA_TX_SLOTS];
	uint8_t rx_scratch[FUXI_WIFI_DATA_FRAME_MAX];   /* the producer's: a received frame being converted */
	uint8_t tx_scratch[FUXI_WIFI_DATA_FRAME_MAX];   /* the producer's: a frame to send, before it is converted */
};

/* head - tail frames wait; head % slots is written next, tail % slots read next (the counts wrap with 2^32) */
struct data_ring {
	unsigned head, tail;
};

_Static_assert((FUXI_WIFI_DATA_RX_SLOTS & (FUXI_WIFI_DATA_RX_SLOTS - 1)) == 0 &&
    (FUXI_WIFI_DATA_EAPOL_SLOTS & (FUXI_WIFI_DATA_EAPOL_SLOTS - 1)) == 0 &&
    (FUXI_WIFI_DATA_TX_SLOTS & (FUXI_WIFI_DATA_TX_SLOTS - 1)) == 0, "slots a power of two: the index wraps with the count");

static bool data_ready_word;                            /* the one thing a producer call stores before use() */
/* the consumer, from use() on */
static bool data_used;
static struct data_store *data_store;
static uint8_t data_self[6];
static struct data_ring data_rx, data_eapol, data_tx;
static uint32_t data_inflight[FUXI_WIFI_DATA_TX_INFLIGHT];     /* the ids taken and not done */
static unsigned data_inflight_count;
static uint32_t data_tx_id;                             /* the last id given */
/* the link and the mailbox */
static enum fuxi_wifi_data_state data_state;
static struct fuxi_wifi_data_request data_request;
static bool data_posted;                                /* data_request waits for take() */
static uint32_t data_seq;                               /* the last request posted */
static uint32_t data_live;                              /* the join the link is for, 0: none */
static uint32_t data_epoch;
static uint8_t data_bssid[6];
static uint16_t data_mhz;
static int16_t data_rssi;
static int data_status;
static struct fuxi_wifi_data_counts data_counted;

static bool
data_put_locked(struct data_ring *ring, struct data_slot *slots, unsigned most, const uint8_t *frame, size_t length)
{
	struct data_slot *slot;

	if (ring->head - ring->tail >= most) {
		return false;
	}
	slot = &slots[ring->head % most];
	memcpy(slot->frame, frame, length);
	slot->length = (uint16_t)length;
	ring->head++;
	return true;
}

/* The oldest frame into `frame` (room for FUXI_WIFI_DATA_FRAME_MAX), consumed; its length, 0: none. */
static size_t
data_get_locked(struct data_ring *ring, struct data_slot *slots, unsigned most, uint8_t *frame)
{
	struct data_slot *slot;

	if (ring->head == ring->tail) {
		return 0;
	}
	slot = &slots[ring->tail % most];
	memcpy(frame, slot->frame, slot->length);
	ring->tail++;
	return slot->length;
}

/* Every queue emptied, every frame in flight forgotten (its done() is stale). */
static void
data_flush_locked(void)
{
	data_counted.rx_flushed += (data_rx.head - data_rx.tail) + (data_eapol.head - data_eapol.tail);
	data_counted.tx_flushed += (data_tx.head - data_tx.tail) + data_inflight_count;
	data_rx.tail = data_rx.head;
	data_eapol.tail = data_eapol.head;
	data_tx.tail = data_tx.head;
	data_inflight_count = 0;
}

/*
 * The link becomes `state`: a new one, or none (then `status` says why). Nothing of the last one stays; from
 * JOINING to UP it is the same link, and the EAPOL received while joining (a handshake) stays queued.
 */
static void
data_change_locked(enum fuxi_wifi_data_state state, int status)
{
	if (state != FUXI_WIFI_DATA_UP) {
		data_flush_locked();
	}
	data_state = state;
	data_status = status;
	if (state == FUXI_WIFI_DATA_IDLE) {
		data_live = 0;
		data_rssi = 0;
	}
	data_epoch++;
}

/* A join (`what`) or a leave (NULL) for take(); an untaken one is replaced. Its seq. */
static uint32_t
data_post_locked(const struct fuxi_wifi_data_request *what)
{
	if (what != NULL) {
		memcpy(&data_request, what, sizeof(data_request));
	} else {
		bzero(&data_request, sizeof(data_request));
		data_request.leave = true;
	}
	if (++data_seq == 0) {
		data_seq = 1;
	}
	data_request.seq = data_seq;
	data_posted = true;
	return data_seq;
}

/* The link (or the join) the consumer ends: the producer is told to leave, unless it never took the join. */
static void
data_end_locked(int status)
{
	if (data_posted && !data_request.leave) {
		data_posted = false;
	} else {
		(void)data_post_locked(NULL);
	}
	if (data_state == FUXI_WIFI_DATA_UP) {
		data_counted.links_left++;
	}
	data_change_locked(FUXI_WIFI_DATA_IDLE, status);
}

static void
data_link_locked(struct fuxi_wifi_data_link *out)
{
	out->state = data_state;
	out->seq = data_live;
	out->epoch = data_epoch;
	memcpy(out->bssid, data_bssid, sizeof(out->bssid));
	out->mhz = data_mhz;
	out->rssi = data_rssi;
	out->status = data_status;
}

static bool
data_forget_locked(uint32_t id)
{
	for (unsigned i = 0; i < data_inflight_count; i++) {
		if (data_inflight[i] == id) {
			data_inflight[i] = data_inflight[--data_inflight_count];
			return true;
		}
	}
	return false;
}

/* ---- the producer --------------------------------------------------------------------------------------- */

void
fuxi_wifi_data_ready(bool ready)
{
	lck_mtx_lock(&data_lock);
	data_ready_word = ready;
	if (!ready && data_used) {
		/* nobody takes requests any more: one that waits is dropped, the link (or the join) ends */
		data_posted = false;
		if (data_state == FUXI_WIFI_DATA_UP) {
			data_counted.links_lost++;
		}
		if (data_state != FUXI_WIFI_DATA_IDLE) {
			data_change_locked(FUXI_WIFI_DATA_IDLE, ENETDOWN);
		}
	}
	lck_mtx_unlock(&data_lock);
	thread_wakeup((event_t)&data_state);
}

bool
fuxi_wifi_data_take(struct fuxi_wifi_data_request *out)
{
	bool taken;

	if (out == NULL) {
		return false;
	}
	lck_mtx_lock(&data_lock);
	taken = data_used && data_posted;
	if (taken) {
		memcpy(out, &data_request, sizeof(*out));
		data_posted = false;
	}
	lck_mtx_unlock(&data_lock);
	return taken;
}

void
fuxi_wifi_data_link_up(uint32_t seq, const uint8_t *bssid, uint16_t mhz)
{
	bool up = false;

	if (bssid == NULL) {
		return;
	}
	lck_mtx_lock(&data_lock);
	if (data_used && data_state == FUXI_WIFI_DATA_JOINING && seq != 0 && seq == data_live && !data_posted &&
	    memcmp(bssid, data_bssid, sizeof(data_bssid)) == 0) {
		data_change_locked(FUXI_WIFI_DATA_UP, 0);
		data_mhz = mhz;
		data_counted.links_up++;
		up = true;
	} else if (data_used) {
		data_counted.link_stale++;
	}
	lck_mtx_unlock(&data_lock);
	if (up) {
		thread_wakeup((event_t)&data_state);
	}
}

void
fuxi_wifi_data_link_down(uint32_t seq, int status)
{
	bool down = false;

	lck_mtx_lock(&data_lock);
	if (data_used && seq != 0 && seq == data_live && !data_posted && data_state != FUXI_WIFI_DATA_IDLE) {
		if (data_state == FUXI_WIFI_DATA_UP) {
			data_counted.links_lost++;
		}
		data_change_locked(FUXI_WIFI_DATA_IDLE, status > 0 ? status :
		    data_state == FUXI_WIFI_DATA_UP ? ECONNRESET : ECONNREFUSED);
		down = true;
	} else if (data_used) {
		data_counted.link_stale++;
	}
	lck_mtx_unlock(&data_lock);
	if (down) {
		thread_wakeup((event_t)&data_state);
	}
}

int
fuxi_wifi_data_rx(const uint8_t *mpdu, size_t length, int16_t rssi_dbm)
{
	enum fuxi_wifi_data_state state;
	uint8_t bssid[6], self[6];
	uint32_t epoch;
	size_t made = 0;
	bool eapol = false, used;
	int why = FUXI_WIFI_DATA_DROP_DOWN;

	lck_mtx_lock(&data_lock);
	used = data_used;
	state = data_state;
	epoch = data_epoch;
	memcpy(bssid, data_bssid, sizeof(bssid));
	memcpy(self, data_self, sizeof(self));
	lck_mtx_unlock(&data_lock);
	if (!used) {
		return FUXI_WIFI_DATA_DROP_DOWN;
	}
	/* (data_store->rx_scratch is the producer's) */
	if (state != FUXI_WIFI_DATA_IDLE) {
		why = fuxi_wifi_data_to_8023(mpdu, length, bssid, self, data_store->rx_scratch, &made, &eapol);
	}
	lck_mtx_lock(&data_lock);
	data_counted.rx_frames++;
	if (data_epoch != epoch || data_state == FUXI_WIFI_DATA_IDLE ||
	    (why == FUXI_WIFI_DATA_KEPT && !eapol && data_state != FUXI_WIFI_DATA_UP)) {
		why = FUXI_WIFI_DATA_DROP_DOWN;             /* the link changed, or data before it is up */
	}
	if (why == FUXI_WIFI_DATA_KEPT && !(eapol ?
	    data_put_locked(&data_eapol, data_store->eapol, FUXI_WIFI_DATA_EAPOL_SLOTS, data_store->rx_scratch, made) :
	    data_put_locked(&data_rx, data_store->rx, FUXI_WIFI_DATA_RX_SLOTS, data_store->rx_scratch, made))) {
		why = FUXI_WIFI_DATA_DROP_FULL;
	}
	if (why == FUXI_WIFI_DATA_KEPT) {
		data_counted.rx_kept++;
		data_counted.rx_eapol += eapol;
		data_rssi = rssi_dbm < -127 ? -127 : rssi_dbm > 0 ? 0 : rssi_dbm;
	} else if (why > 0 && why < FUXI_WIFI_DATA_DROP_REASONS) {
		data_counted.rx_dropped[why]++;
	}
	lck_mtx_unlock(&data_lock);
	return why;
}

size_t
fuxi_wifi_data_tx_take(uint8_t *mpdu, size_t room, uint32_t *id)
{
	uint8_t bssid[6], self[6];
	size_t length = 0, made = 0;
	uint32_t taken = 0;
	int why;

	if (id != NULL) {
		*id = 0;
	}
	if (mpdu == NULL || id == NULL) {
		return 0;
	}
	lck_mtx_lock(&data_lock);
	if (data_used && data_state == FUXI_WIFI_DATA_UP && data_inflight_count < FUXI_WIFI_DATA_TX_INFLIGHT) {
		length = data_get_locked(&data_tx, data_store->tx, FUXI_WIFI_DATA_TX_SLOTS, data_store->tx_scratch);
	}
	if (length != 0) {
		if (++data_tx_id == 0) {
			data_tx_id = 1;
		}
		taken = data_tx_id;
		data_inflight[data_inflight_count++] = taken;
		data_counted.tx_taken++;
		memcpy(bssid, data_bssid, sizeof(bssid));
		memcpy(self, data_self, sizeof(self));
	}
	lck_mtx_unlock(&data_lock);
	if (length == 0) {
		return 0;
	}
	/* (data_store->tx_scratch is the producer's; send() checked the frame, so only the room can fail here) */
	why = fuxi_wifi_data_from_8023(data_store->tx_scratch, length, bssid, self, mpdu, room, &made);
	if (why != FUXI_WIFI_DATA_KEPT) {
		lck_mtx_lock(&data_lock);
		(void)data_forget_locked(taken);
		data_counted.tx_dropped[why]++;
		lck_mtx_unlock(&data_lock);
		return 0;
	}
	*id = taken;
	return made;
}

void
fuxi_wifi_data_tx_done(uint32_t id, int status)
{
	lck_mtx_lock(&data_lock);
	if (!data_used) {
		/* nothing */
	} else if (id == 0 || !data_forget_locked(id)) {
		data_counted.tx_stale++;
	} else if (status == 0) {
		data_counted.tx_ok++;
	} else {
		data_counted.tx_failed++;
	}
	lck_mtx_unlock(&data_lock);
}

/* ---- the consumer --------------------------------------------------------------------------------------- */

bool
fuxi_wifi_data_use(const uint8_t *self)
{
	struct data_store *store;
	bool used;

	if (self == NULL) {
		return false;
	}
	lck_mtx_lock(&data_lock);
	used = data_used;
	lck_mtx_unlock(&data_lock);
	if (used) {
		return false;
	}
	store = kalloc_data(sizeof(*store), Z_WAITOK | Z_ZERO);
	if (store == NULL) {
		return false;
	}
	lck_mtx_lock(&data_lock);
	data_store = store;
	memcpy(data_self, self, sizeof(data_self));
	data_used = true;
	lck_mtx_unlock(&data_lock);
	return true;
}

bool
fuxi_wifi_data_present(void)
{
	bool present;

	lck_mtx_lock(&data_lock);
	present = data_used && data_ready_word;
	lck_mtx_unlock(&data_lock);
	return present;
}

/* Under the lock: how the join `seq` stands (-1: still waiting; `late`: its time is up). */
static int
data_waited_locked(uint32_t seq, bool late)
{
	if (data_live != seq) {
		return data_status > 0 ? data_status : ECANCELED;
	}
	if (data_state == FUXI_WIFI_DATA_UP) {
		return 0;
	}
	return late ? ETIMEDOUT : -1;
}

/*
 * One join, on the station thread: posted for take(), waited for at most `timeout_ms` (and while `give_up`
 * says no). 0 (the link is up; `link` says which), EOPNOTSUPP (no producer), EINVAL, ETIMEDOUT, ECANCELED,
 * or the producer's errno. Whatever was joined before is left first; nothing passes until the link is up.
 */
int
fuxi_wifi_data_join(const struct fuxi_wifi_data_request *what, unsigned timeout_ms, bool (*give_up)(void),
    struct fuxi_wifi_data_link *link)
{
	static const uint8_t zero[6];
	struct fuxi_wifi_data_request request;
	uint64_t start, now, limit;
	uint32_t seq = 0;
	int status;

	if (what == NULL || what->leave || what->ssid_length > sizeof(what->ssid) || (what->bssid[0] & 1) != 0 ||
	    memcmp(what->bssid, zero, sizeof(zero)) == 0) {
		return EINVAL;
	}
	if (timeout_ms == 0 || timeout_ms > FUXI_WIFI_DATA_JOIN_WAIT_MS) {
		timeout_ms = FUXI_WIFI_DATA_JOIN_WAIT_MS;
	}
	memcpy(&request, what, sizeof(request));
	clock_get_uptime(&start);
	clock_interval_to_absolutetime_interval(timeout_ms, NSEC_PER_MSEC, &limit);
	lck_mtx_lock(&data_lock);
	if (data_used && data_ready_word) {
		if (data_state == FUXI_WIFI_DATA_UP) {
			data_counted.links_left++;
		}
		data_change_locked(FUXI_WIFI_DATA_JOINING, 0);
		seq = data_live = data_post_locked(&request);
		memcpy(data_bssid, request.bssid, sizeof(data_bssid));
		data_mhz = request.mhz;
		data_counted.joins++;
	}
	lck_mtx_unlock(&data_lock);
	if (seq == 0) {
		return EOPNOTSUPP;
	}
	thread_wakeup((event_t)&data_request);
	for (status = -1; status < 0;) {
		clock_get_uptime(&now);
		lck_mtx_lock(&data_lock);
		status = data_waited_locked(seq, now - start >= limit);
		if (status < 0) {
			(void)assert_wait_timeout((event_t)&data_state, THREAD_UNINT, DATA_POLL_MS, NSEC_PER_MSEC);
		}
		lck_mtx_unlock(&data_lock);
		if (status < 0) {
			(void)thread_block(THREAD_CONTINUE_NULL);
			if (give_up != NULL && give_up()) {
				lck_mtx_lock(&data_lock);
				if (data_live == seq && data_state == FUXI_WIFI_DATA_JOINING) {
					data_end_locked(ECANCELED);
				}
				lck_mtx_unlock(&data_lock);
			}
		}
	}
	lck_mtx_lock(&data_lock);
	if (status != 0 && data_live == seq && data_state == FUXI_WIFI_DATA_JOINING) {
		data_end_locked(status);                /* timed out: the producer is told to give up */
	}
	if (link != NULL) {
		data_link_locked(link);
	}
	lck_mtx_unlock(&data_lock);
	thread_wakeup((event_t)&data_request);
	return status;
}

void
fuxi_wifi_data_leave(void)
{
	lck_mtx_lock(&data_lock);
	if (data_used && data_state != FUXI_WIFI_DATA_IDLE) {
		data_end_locked(0);
	}
	lck_mtx_unlock(&data_lock);
	thread_wakeup((event_t)&data_state);
	thread_wakeup((event_t)&data_request);
}

int
fuxi_wifi_data_send(const uint8_t *frame, size_t length)
{
	uint8_t self[6];
	size_t made;
	bool used;
	int why;

	lck_mtx_lock(&data_lock);
	used = data_used;
	memcpy(self, data_self, sizeof(self));
	lck_mtx_unlock(&data_lock);
	if (!used) {
		return FUXI_WIFI_DATA_DROP_DOWN;
	}
	why = fuxi_wifi_data_from_8023(frame, length, self, self, NULL, 0, &made);
	if (why == FUXI_WIFI_DATA_KEPT && ((unsigned)frame[12] << 8 | frame[13]) == FUXI_WIFI_DATA_ETHERTYPE_EAPOL) {
		why = FUXI_WIFI_DATA_DROP_EAPOL;
	}
	lck_mtx_lock(&data_lock);
	data_counted.tx_frames++;
	if (why == FUXI_WIFI_DATA_KEPT && data_state != FUXI_WIFI_DATA_UP) {
		why = FUXI_WIFI_DATA_DROP_DOWN;
	}
	if (why == FUXI_WIFI_DATA_KEPT && !data_put_locked(&data_tx, data_store->tx, FUXI_WIFI_DATA_TX_SLOTS, frame, length)) {
		why = FUXI_WIFI_DATA_DROP_FULL;
	}
	if (why == FUXI_WIFI_DATA_KEPT) {
		data_counted.tx_queued++;
	} else if (why > 0 && why < FUXI_WIFI_DATA_DROP_REASONS) {
		data_counted.tx_dropped[why]++;
	}
	lck_mtx_unlock(&data_lock);
	return why;
}

static bool
data_pull(bool eapol, uint8_t *frame, size_t room, size_t *length)
{
	size_t got = 0;

	if (length != NULL) {
		*length = 0;
	}
	if (frame == NULL || length == NULL || room < FUXI_WIFI_DATA_FRAME_MAX) {
		return false;
	}
	lck_mtx_lock(&data_lock);
	if (data_used && eapol) {
		got = data_get_locked(&data_eapol, data_store->eapol, FUXI_WIFI_DATA_EAPOL_SLOTS, frame);
		data_counted.eapol_pulled += got != 0;
	} else if (data_used) {
		got = data_get_locked(&data_rx, data_store->rx, FUXI_WIFI_DATA_RX_SLOTS, frame);
		data_counted.rx_pulled += got != 0;
	}
	lck_mtx_unlock(&data_lock);
	*length = got;
	return got != 0;
}

bool
fuxi_wifi_data_pull(uint8_t *frame, size_t room, size_t *length)
{
	return data_pull(false, frame, room, length);
}

bool
fuxi_wifi_data_eapol_pull(uint8_t *frame, size_t room, size_t *length)
{
	return data_pull(true, frame, room, length);
}

void
fuxi_wifi_data_link(struct fuxi_wifi_data_link *out)
{
	if (out == NULL) {
		return;
	}
	lck_mtx_lock(&data_lock);
	data_link_locked(out);
	lck_mtx_unlock(&data_lock);
}

void
fuxi_wifi_data_counts(struct fuxi_wifi_data_counts *out)
{
	if (out == NULL) {
		return;
	}
	lck_mtx_lock(&data_lock);
	memcpy(out, &data_counted, sizeof(*out));
	lck_mtx_unlock(&data_lock);
}

/* ---- fuxiwifiios=5: the radio data backend --------------------------------------------------------------
 * The radio scan backend of fuxi_wifi_ios.c (fuxiwifiios=3) as it is for power, scans, results and channels;
 * joins, leave, the state, en0's frames and en0's link are this part's. It calls the handoff above only
 * through its public functions, like any other consumer, and never the radio lane. Route U's RX thread
 * (usbnet_rx) is the model of its own: 1 ms polls while frames moved in the last second, else 20 ms. Open
 * networks only: a WPA2 join is refused (EOPNOTSUPP) until a supplicant drains the EAPOL queue. */

#define RDATA_SAID_MOST 24      /* refusals said one by one; after that every hundredth */

static LCK_MTX_DECLARE(rdata_lock, &data_lock_group);          /* a leaf: the phase, the record, the error */
static LCK_MTX_DECLARE(rdata_tell_lock, &data_lock_group);     /* one caller of en0's link_state at a time */
static struct fuxi_wifi_backend rdata_backend;
static struct fuxi_wifi_sta_ops rdata_scan;                     /* the radio scan backend's station */
static const struct fuxi_wifi_input *rdata_input;
static enum fuxi_wifi_sta_phase rdata_phase = FUXI_WIFI_STA_IDLE;       /* IDLE, JOINING or JOINED */
static uint32_t rdata_epoch;                                    /* the handoff's epoch of the link joined */
static struct fuxi_wifi_sta_bss rdata_bss;                      /* the network joined or being joined */
static struct fuxi_wifi_sta_bss rdata_found;                    /* the station thread's: the best match */
static int rdata_error;
static unsigned rdata_refusals;
static bool rdata_told;                                         /* what en0 was last told */
static uint64_t rdata_tx_until;                                 /* a frame went out: poll fast until then */

/* en0's link follows the phase: JOINED is up, anything else down. Never under rdata_lock. */
static void
rdata_tell(void)
{
	bool up, change;

	lck_mtx_lock(&rdata_tell_lock);
	lck_mtx_lock(&rdata_lock);
	up = rdata_phase == FUXI_WIFI_STA_JOINED;
	change = up != rdata_told;
	rdata_told = up;
	lck_mtx_unlock(&rdata_lock);
	if (change) {
		rdata_input->link_state(up);
	}
	lck_mtx_unlock(&rdata_tell_lock);
}

static void
rdata_say_counts(const char *what, int status)
{
	struct fuxi_wifi_data_counts counts;
	unsigned rx_dropped = 0, tx_dropped = 0;

	fuxi_wifi_data_counts(&counts);
	for (unsigned i = 1; i < FUXI_WIFI_DATA_DROP_REASONS; i++) {
		rx_dropped += counts.rx_dropped[i];
		tx_dropped += counts.tx_dropped[i];
	}
	printf("FUXI WIFI IOS: data: %s, status %d; rx %u frames, %u kept (%u EAPOL), %u dropped; tx %u frames, %u queued, "
	    "%u dropped, %u ok, %u failed\n", what, status, counts.rx_frames, counts.rx_kept, counts.rx_eapol, rx_dropped,
	    counts.tx_frames, counts.tx_queued, tx_dropped, counts.tx_ok, counts.tx_failed);
}

/* Whatever was joined is left (the producer told, en0 down); whether something was joined. */
static bool
rdata_drop(void)
{
	bool was;

	lck_mtx_lock(&rdata_lock);
	was = rdata_phase == FUXI_WIFI_STA_JOINED;
	rdata_phase = FUXI_WIFI_STA_IDLE;
	lck_mtx_unlock(&rdata_lock);
	fuxi_wifi_data_leave();
	rdata_tell();
	return was;
}

static void
rdata_start(const struct fuxi_wifi_input *in)
{
	rdata_input = in;
}

static int
rdata_power(bool on)
{
	if (!on) {
		(void)rdata_drop();
	}
	return rdata_scan.power(on);
}

struct rdata_pick {
	const struct fuxi_wifi_sta_join *what;
	bool found;
};

/* The strongest record of the last scan with the name (and the BSSID, if wifid named one). */
static void
rdata_pick_one(const struct fuxi_wifi_sta_bss *bss, void *context)
{
	static const uint8_t any[6];
	struct rdata_pick *pick = context;

	if (bss->ssid_length == 0 || bss->ssid_length != pick->what->ssid_length ||
	    memcmp(bss->ssid, pick->what->ssid, bss->ssid_length) != 0 ||
	    (memcmp(pick->what->bssid, any, sizeof(any)) != 0 && memcmp(pick->what->bssid, bss->bssid, 6) != 0)) {
		return;
	}
	if (!pick->found || bss->rssi > rdata_found.rssi) {
		rdata_found = *bss;
		pick->found = true;
	}
}

static int
rdata_join(const struct fuxi_wifi_sta_join *what)
{
	struct fuxi_wifi_data_request request;
	struct fuxi_wifi_data_link link;
	struct rdata_pick pick = { .what = what };
	const char *why = NULL;
	uint64_t start, end, ns;
	unsigned times;
	int error;

	(void)rdata_drop();                     /* a join replaces the network joined: en0 down, no loss said */
	if (!fuxi_wifi_data_present()) {
		why = "no radio data producer";
	} else if (what->security != FUXI_WIFI_STA_OPEN) {
		why = "WPA2 needs a supplicant, not in this build";
	} else {
		(void)fuxi_wifi_radio_results(rdata_pick_one, &pick);
		if (pick.found && (rdata_found.capability & 0x10) != 0) {
			why = "the network is protected, open joins only";
		}
	}
	if (why != NULL || !pick.found) {
		error = why != NULL ? EOPNOTSUPP : ENOENT;
		lck_mtx_lock(&rdata_lock);
		rdata_error = error;
		times = ++rdata_refusals;
		lck_mtx_unlock(&rdata_lock);
		if (times <= RDATA_SAID_MOST || times % 100 == 0) {
			printf("FUXI WIFI IOS: data: join refused (%s), no radio access, status %d\n",
			    why != NULL ? why : "not heard by the last scan", error);
		}
		return error;
	}
	bzero(&request, sizeof(request));
	memcpy(request.ssid, what->ssid, what->ssid_length);
	request.ssid_length = what->ssid_length;
	memcpy(request.bssid, rdata_found.bssid, sizeof(request.bssid));
	request.mhz = rdata_found.frequency;
	request.security = FUXI_WIFI_STA_OPEN;
	lck_mtx_lock(&rdata_lock);
	rdata_phase = FUXI_WIFI_STA_JOINING;
	rdata_bss = rdata_found;
	lck_mtx_unlock(&rdata_lock);
	clock_get_uptime(&start);
	error = fuxi_wifi_data_join(&request, FUXI_WIFI_DATA_JOIN_WAIT_MS, fuxi_wifi_sta_power_pending, &link);
	lck_mtx_lock(&rdata_lock);
	if (error == 0 && rdata_phase == FUXI_WIFI_STA_JOINING) {
		rdata_phase = FUXI_WIFI_STA_JOINED;
		rdata_epoch = link.epoch;
		rdata_bss.frequency = link.mhz;
	} else {
		rdata_phase = FUXI_WIFI_STA_IDLE;
		error = error != 0 ? error : ECANCELED;
	}
	rdata_error = error;
	lck_mtx_unlock(&rdata_lock);
	rdata_tell();
	/* (lost already, between the wait and the phase above: a failed join, not a join and a loss) */
	fuxi_wifi_data_link(&link);
	lck_mtx_lock(&rdata_lock);
	if (error == 0 && (link.state != FUXI_WIFI_DATA_UP || link.epoch != rdata_epoch)) {
		rdata_phase = FUXI_WIFI_STA_IDLE;
		error = rdata_error = link.status > 0 ? link.status : ECONNRESET;
	}
	lck_mtx_unlock(&rdata_lock);
	if (error != 0) {
		rdata_tell();
	}
	clock_get_uptime(&end);
	absolutetime_to_nanoseconds(end - start, &ns);
	printf("FUXI WIFI IOS: data: join %s, %u MHz, open, %llu ms, status %d\n", error == 0 ? "done" : "failed",
	    request.mhz, (unsigned long long)(ns / 1000000), error);
	return error;
}

static int
rdata_leave(void)
{
	if (rdata_drop()) {
		rdata_say_counts("left", 0);
	}
	return 0;
}

static void
rdata_snapshot(struct fuxi_wifi_sta_state *state)
{
	struct fuxi_wifi_data_link link;

	rdata_scan.state(state);                /* the radio scan backend's: off, scanning or idle */
	fuxi_wifi_data_link(&link);
	lck_mtx_lock(&rdata_lock);
	if (rdata_phase != FUXI_WIFI_STA_IDLE) {
		state->phase = rdata_phase;
		memcpy(&state->bss, &rdata_bss, sizeof(state->bss));
		state->security = FUXI_WIFI_STA_OPEN;
		if (rdata_phase == FUXI_WIFI_STA_JOINED && link.rssi != 0) {
			state->bss.rssi = link.rssi;
		}
	}
	state->last_error = rdata_error;
	lck_mtx_unlock(&rdata_lock);
}

static bool
rdata_tx(const uint8_t *frame, size_t length)
{
	uint64_t now, second;

	clock_get_uptime(&now);
	clock_interval_to_absolutetime_interval(1, NSEC_PER_SEC, &second);
	__atomic_store_n(&rdata_tx_until, now + second, __ATOMIC_RELAXED);
	return fuxi_wifi_data_send(frame, length) == FUXI_WIFI_DATA_KEPT;
}

/* thread call context (fuxi_wifi_radio.c's notify), after attach: a radio that stopped takes the link with it */
static void
rdata_changed(enum fuxi_wifi_radio_state state)
{
	if (state != FUXI_WIFI_RADIO_STOPPED) {
		return;
	}
	if (rdata_drop()) {
		rdata_say_counts("link lost with the radio", ENETDOWN);
		fuxi_wifi_sta_lost(ENETDOWN);
	}
	fuxi_wifi_sta_radio_off();
}

/* Received frames to en0, and the link's loss to wifid. */
static void
rdata_rx(__unused void *parameter, __unused wait_result_t result)
{
	uint8_t frame[FUXI_WIFI_DATA_FRAME_MAX];
	struct fuxi_wifi_data_link link;
	uint64_t now, second, busy_until = 0;

	clock_interval_to_absolutetime_interval(1, NSEC_PER_SEC, &second);
	for (;;) {
		size_t length = 0;
		bool moved = false, lost = false;
		int error = 0;

		for (unsigned n = 0; n < FUXI_WIFI_DATA_RX_SLOTS && fuxi_wifi_data_pull(frame, sizeof(frame), &length); n++) {
			rdata_input->rx_frame(frame, length);
			moved = true;
		}
		fuxi_wifi_data_link(&link);
		lck_mtx_lock(&rdata_lock);
		if (rdata_phase == FUXI_WIFI_STA_JOINED && (link.state != FUXI_WIFI_DATA_UP || link.epoch != rdata_epoch)) {
			rdata_phase = FUXI_WIFI_STA_IDLE;
			error = rdata_error = link.status > 0 ? link.status : ECONNRESET;
			lost = true;
		}
		lck_mtx_unlock(&rdata_lock);
		if (lost) {
			rdata_say_counts("link lost", error);
			rdata_tell();
			fuxi_wifi_sta_lost(error);
		}
		clock_get_uptime(&now);
		if (moved) {
			busy_until = now + second;
		}
		(void)assert_wait_timeout((event_t)&rdata_input, THREAD_UNINT,
		    now < busy_until || now < __atomic_load_n(&rdata_tx_until, __ATOMIC_RELAXED) ? 1 : 20, NSEC_PER_MSEC);
		(void)thread_block(THREAD_CONTINUE_NULL);
	}
}

void
fuxi_wifi_data_init(const struct fuxi_wifi_backend *scan)
{
	thread_t thread;

	printf("FUXI WIFI IOS: enabled, radio data backend (radio scans, open joins, en0 frames through the data handoff); "
	    "Apple80211 16.7 ABI unverified\n");
	if (scan == NULL) {
		return;
	}
	rdata_backend = *scan;
	rdata_scan = scan->station;
	rdata_backend.station.power = rdata_power;
	rdata_backend.station.join = rdata_join;
	rdata_backend.station.leave = rdata_leave;
	rdata_backend.station.state = rdata_snapshot;
	rdata_backend.tx_frame = rdata_tx;
	rdata_backend.start = rdata_start;
	fuxi_wifi_apple80211_backend("Fuxi Wi-Fi iOS shim (radio data backend)", true);
	if (!fuxi_wifi_radio_use(false, rdata_changed) || !fuxi_wifi_data_use(rdata_backend.station.address)) {
		printf("FUXI WIFI IOS: data: no memory or thread call for the handoffs; no retry this boot\n");
		return;
	}
	if (!fuxi_wifi_ios_attach(&rdata_backend)) {
		return;
	}
	fuxi_wifi_radio_attached();
	if (kernel_thread_start(rdata_rx, NULL, &thread) != KERN_SUCCESS) {
		printf("FUXI WIFI IOS: data: no RX thread; nothing received reaches en0\n");
		return;
	}
	thread_set_thread_name(thread, "wifi-data-rx");
	thread_deallocate(thread);
}

#endif /* FUXI */
