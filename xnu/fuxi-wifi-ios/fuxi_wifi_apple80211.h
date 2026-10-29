/*
 * Ported from HackiOS xnu/files/bsd/dev/redwood_apple80211.c (APSL-2.0)
 * Fuxi, 2026-10-23: shared diagnostic ABI; 16.7 payloads UNVERIFIED.
 * See LICENSE.HackiOS and docs-lanes/wifi-ios.md for provenance/confidence.
 */
#ifndef FUXI_WIFI_APPLE80211_H
#define FUXI_WIFI_APPLE80211_H
#include <stdint.h>
#include <stddef.h>
#define SIOCSA80211             0x802869c8u     /* _IOW('i', 200, struct apple80211req) */
#define SIOCGA80211             0xc02869c9u     /* _IOWR('i', 201, struct apple80211req) */

struct apple80211req {
	char req_if_name[16];
	int32_t req_type;
	int32_t req_val;
	uint32_t req_len;
	uint64_t req_data __attribute__((aligned(8)));
};

_Static_assert(sizeof(struct apple80211req) == 40 && offsetof(struct apple80211req, req_data) == 32,
    "arm64 Apple80211 request envelope (16.7 consumer unverified)");

/* ---- the layouts ------------------------------------------------------------------------------------------ */

#define CHANNEL_20MHZ           0x2u
#define CHANNEL_2GHZ            0x8u
#define CHANNEL_5GHZ            0x10u
#define CHANNEL_ACTIVE          0x80u           /* the radio may send there */
#define CHANNEL_DFS             0x100u

struct apple80211_channel {
	uint32_t version, channel, flags;
};

/* SUPPORTED_CHANNELS (27) and HW_SUPPORTED_CHANNELS (254): 4824 bytes */
struct apple80211_channels {
	uint32_t version, count;
	struct apple80211_channel channels[401];
	uint32_t unused;
};

/* CHANNELS_INFO (207): 10016 bytes; of each channel a byte in each of several rows, and a word of flags */
struct apple80211_channels_info {
	uint8_t unknown_0[12];
	uint16_t count;
	uint8_t unknown_1[800];
	uint8_t number[400], indoor[400], dfs[400], passive[400], wide_40[400], wide_80[400];
	uint8_t unknown_2[1602];
	uint32_t flags[400];
	uint8_t unknown_3[3600];
} __attribute__((packed));

/* SCAN_REQ (10): 5456 bytes */
struct apple80211_scan_request {
	uint32_t version, bss_type;
	uint8_t bssid[6], unused_0[2];
	uint32_t ssid_length;
	uint8_t ssid[32];
	uint32_t scan_type;                     /* 1: asking, 2: listening only */
	uint32_t phy_mode;
	uint16_t dwell_ms, unused_1;
	uint32_t rest_ms;
	uint32_t channel_count;
	struct apple80211_channel channels[400];
	uint64_t flags;
	uint32_t unused_2;
	uint32_t name_count;                    /* networks asked for by name, each in 56 bytes */
	uint8_t names[10][56];
	uint32_t short_ssid, unused_3;
};

/* SCAN_RESULT (11) and CURRENT_NETWORK (103): 1164 bytes */
struct apple80211_scan_result {
	uint32_t version;
	struct apple80211_channel channel;
	uint16_t noise_known;
	int16_t noise, snr, rssi;
	uint16_t beacon_interval, capability;
	uint8_t bssid[6];
	uint8_t rate_count, unused_0;
	uint32_t rates[15];                     /* Mbit/s */
	uint8_t ssid_length, ssid[32], unused_1[3];
	uint32_t age_ms;
	uint8_t flags, unused_2;
	uint16_t elements_length;               /* comes in as the room there is */
	uint8_t elements[1024];
};

/* ASSOCIATE (20): 592 bytes; of what follows the key (the RSN element wifid built, flags) nothing is used */
struct apple80211_assoc {
	uint32_t version;
	uint16_t mode, auth_lower, auth_upper, unused_0;
	uint32_t ssid_length;
	uint8_t ssid[32];
	uint8_t bssid[6], unused_1[2];
	uint32_t key_version, key_length, key_cipher;   /* cipher 10: a passphrase as typed; 6: the key (from 64 hex digits) */
	uint16_t key_flags, key_index;
	uint8_t key[64];
	uint8_t rest[0x250 - 0x88];
};

#define CIPHER_PMK              6
#define CIPHER_PASSPHRASE       10

/* the link event's data (at least 32 bytes, or the framework drops the event) */
struct apple80211_link_changed {
	uint8_t down, unused_0[3];
	int32_t rssi;
	uint16_t snr, noise;
	uint8_t unused_1[4];
	uint8_t involuntary, unused_2[3];
	uint32_t reason, subreason;
	uint8_t unused_3[4];
};

/*
 * Fuxi, 2026-10-29: the layouts from here on were read out of IO80211.framework of 20H392 (16.7):
 * Apple80211GetWithIOCTL and Apple80211SetWithIOCTL, Apple80211SetProperty. Against the same framework
 * the scan request (__getScanData), a scan's result (__addScanResultToList), the channel list (254)
 * and the channels' description (207) above were checked and match; the association request and the
 * link event were not.
 */

/* PHY_MODE (14): 12 bytes. The kinds of 802.11 as apple80211_var.h numbers them (APPLE80211_MODE_*). */
#define PHY_MODE_AUTO           0x1u
#define PHY_MODE_11A            0x2u
#define PHY_MODE_11B            0x4u
#define PHY_MODE_11G            0x8u

struct apple80211_phy_mode {
	uint32_t version, supported, active;
};

/* RSSI_BOUNDS (106): 16 bytes, dBm */
struct apple80211_rssi_bounds {
	int32_t version, rssi_min, rssi_max, rssi_threshold;
};

/* PROTMODE (6): 12 bytes */
struct apple80211_protmode {
	uint32_t version, mode, threshold;
};

/* COUNTRY_CODE (51) as it is set: 8 bytes. (As it is got: the two letters and a NUL alone, in 3 bytes.) */
struct apple80211_country {
	uint32_t version;
	char code[3];
	uint8_t unused;
};

/* SET_PROPERTY (383): 4104 bytes; an XML property list of one key, `length` bytes with its NUL */
struct apple80211_property {
	uint32_t version, length;
	char xml[4096];
};

_Static_assert(sizeof(struct apple80211_phy_mode) == 12 && sizeof(struct apple80211_rssi_bounds) == 16 &&
    sizeof(struct apple80211_protmode) == 12, "PHY_MODE, RSSI_BOUNDS and PROTMODE as 20H392 has them");
_Static_assert(sizeof(struct apple80211_country) == 8 && offsetof(struct apple80211_country, code) == 4,
    "COUNTRY_CODE as 20H392 sets it");
_Static_assert(sizeof(struct apple80211_property) == 4104 && offsetof(struct apple80211_property, xml) == 8,
    "SET_PROPERTY as 20H392 has it");

_Static_assert(sizeof(struct apple80211_assoc) == 592 && offsetof(struct apple80211_assoc, key) == 0x48,
    "the association request as iOS 16.5 has it");
_Static_assert(sizeof(struct apple80211_link_changed) == 32 && offsetof(struct apple80211_link_changed, reason) == 0x14,
    "the link event as iOS 16.5 reads it");
_Static_assert(sizeof(struct apple80211_channels) == 4824, "the channel list as iOS 16.5 has it");
_Static_assert(sizeof(struct apple80211_channels_info) == 10016 && offsetof(struct apple80211_channels_info, number) == 0x32e &&
    offsetof(struct apple80211_channels_info, flags) == 0x12d0, "the channels' description as iOS 16.5 has it");
_Static_assert(sizeof(struct apple80211_scan_request) == 5456 && offsetof(struct apple80211_scan_request, channels) == 0x48 &&
    offsetof(struct apple80211_scan_request, flags) == 0x1308 && offsetof(struct apple80211_scan_request, names) == 0x1318,
    "the scan request as iOS 16.5 has it");
_Static_assert(sizeof(struct apple80211_scan_result) == 1164 && offsetof(struct apple80211_scan_result, ssid_length) == 0x60 &&
    offsetof(struct apple80211_scan_result, age_ms) == 0x84 && offsetof(struct apple80211_scan_result, elements) == 0x8c,
    "a scan's result as iOS 16.5 has it");


#endif
