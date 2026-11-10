/*
 * FUXI WLAN EAPOL: the RSN 4-way and group-key handshakes, done in the
 * kernel because the WCN7850 firmware does not run the supplicant (unlike a
 * phone's own radio, the Linux ath12k path drives EAPOL from user space). The
 * radio lane hands us an ASSOCIATE with the 32-byte PMK (key kind 6) and then
 * relays EAPOL-Key frames to and from the AP; this file turns those frames
 * into an installed pairwise key (TK) and group key (GTK), with no air access
 * of its own.
 *
 * Pure C: no XNU or Linux includes. The includer provides uint8_t..uint64_t,
 * size_t, bool and memcpy/memset/memcmp (bsd/dev/fuxi_wlan.c in the kernel,
 * the host harness in tools/test-fuxi-wlan-eapol.py). Nothing allocates;
 * every buffer is caller-owned or a fixed field of struct fxeapol. Every
 * length that comes off the air is bounded by the frame it arrived in before
 * it is used. The SNonce is drawn from a caller-supplied random function, so
 * the file keeps no entropy source of its own. Design and the state machine:
 * docs-lanes/wlan-eapol.md.
 *
 * Provenance. The primitives are written from the public specifications, not
 * copied: SHA-1 from FIPS 180-4; HMAC from RFC 2104; AES-128 from FIPS 197;
 * the AES key wrap and unwrap from RFC 3394; the PRF, the key hierarchy, the
 * EAPOL-Key frame and the handshake state machine from IEEE Std 802.11-2020,
 * clause 12.7 and Annex J. hostap/wpa_supplicant (BSD-licensed) was read to
 * cross-check field layout and the MIC/key-wrap wiring; no function, table or
 * line of it is reproduced here. The test vectors are the published ones
 * (FIPS 180-4, RFC 2202, FIPS 197, RFC 3394, IEEE Annex J); any station or
 * AP names are invented.
 *
 * This file is placed in the public domain by its author (slush5). It carries
 * no third-party code and imposes no third-party license terms.
 */
#ifndef FUXI_WLAN_EAPOL_H
#define FUXI_WLAN_EAPOL_H

/* ======================================================================
 * SHA-1 (FIPS 180-4, section 6.1). 160-bit digest, 512-bit blocks,
 * big-endian length. Streaming so HMAC and the PRF never copy the message.
 * ====================================================================== */
struct fxsha1 {
	uint32_t h[5];
	uint64_t bits;      /* message length in bits, for the padding */
	uint8_t  buf[64];
	uint32_t n;         /* bytes buffered in buf */
};

static inline uint32_t fxrol32(uint32_t v, unsigned c) { return (v << c) | (v >> (32u - c)); }

static inline void
fxsha1_init(struct fxsha1 *s)
{
	s->h[0] = 0x67452301u; s->h[1] = 0xefcdab89u; s->h[2] = 0x98badcfeu;
	s->h[3] = 0x10325476u; s->h[4] = 0xc3d2e1f0u;
	s->bits = 0; s->n = 0;
}

static inline void
fxsha1_block(struct fxsha1 *s, const uint8_t *p)
{
	uint32_t w[80], a, b, c, d, e, f, k, t;
	unsigned i;
	for (i = 0; i < 16; i++) {
		w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) |
		    ((uint32_t)p[4 * i + 2] << 8) | (uint32_t)p[4 * i + 3];
	}
	for (i = 16; i < 80; i++) { w[i] = fxrol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1); }
	a = s->h[0]; b = s->h[1]; c = s->h[2]; d = s->h[3]; e = s->h[4];
	for (i = 0; i < 80; i++) {
		if (i < 20) { f = (b & c) | (~b & d); k = 0x5a827999u; }
		else if (i < 40) { f = b ^ c ^ d; k = 0x6ed9eba1u; }
		else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8f1bbcdcu; }
		else { f = b ^ c ^ d; k = 0xca62c1d6u; }
		t = fxrol32(a, 5) + f + e + k + w[i];
		e = d; d = c; c = fxrol32(b, 30); b = a; a = t;
	}
	s->h[0] += a; s->h[1] += b; s->h[2] += c; s->h[3] += d; s->h[4] += e;
}

static inline void
fxsha1_update(struct fxsha1 *s, const uint8_t *p, size_t len)
{
	s->bits += (uint64_t)len * 8u;
	while (len > 0) {
		uint32_t take = 64u - s->n;
		if (take > len) { take = (uint32_t)len; }
		memcpy(s->buf + s->n, p, take);
		s->n += take; p += take; len -= take;
		if (s->n == 64u) { fxsha1_block(s, s->buf); s->n = 0; }
	}
}

static inline void
fxsha1_final(struct fxsha1 *s, uint8_t out[20])
{
	uint8_t pad = 0x80; uint8_t lenbe[8]; unsigned i;
	uint64_t bits = s->bits;
	for (i = 0; i < 8; i++) { lenbe[7 - i] = (uint8_t)(bits >> (8u * i)); }
	fxsha1_update(s, &pad, 1);
	while (s->n != 56u) { uint8_t z = 0; fxsha1_update(s, &z, 1); }
	fxsha1_update(s, lenbe, 8);
	for (i = 0; i < 5; i++) {
		out[4 * i] = (uint8_t)(s->h[i] >> 24); out[4 * i + 1] = (uint8_t)(s->h[i] >> 16);
		out[4 * i + 2] = (uint8_t)(s->h[i] >> 8); out[4 * i + 3] = (uint8_t)s->h[i];
	}
}

static inline void
fxsha1(const uint8_t *p, size_t len, uint8_t out[20])
{
	struct fxsha1 s; fxsha1_init(&s); fxsha1_update(&s, p, len); fxsha1_final(&s, out);
}

/* ======================================================================
 * HMAC-SHA-1 (RFC 2104), with a vector form so the PRF can feed the label,
 * the data and the one-byte counter without a scratch concatenation buffer.
 * ====================================================================== */
#define FXE_HMAC_SEGS 4u

static inline void
fxhmac_sha1_v(const uint8_t *key, size_t keylen,
    const uint8_t *seg[], const size_t seglen[], unsigned nseg, uint8_t out[20])
{
	uint8_t k0[64], ipad[64], opad[64], inner[20];
	struct fxsha1 s; unsigned i;
	memset(k0, 0, sizeof(k0));
	if (keylen > 64u) { fxsha1(key, keylen, k0); }
	else { memcpy(k0, key, keylen); }
	for (i = 0; i < 64u; i++) { ipad[i] = k0[i] ^ 0x36u; opad[i] = k0[i] ^ 0x5cu; }
	fxsha1_init(&s);
	fxsha1_update(&s, ipad, 64u);
	for (i = 0; i < nseg; i++) { fxsha1_update(&s, seg[i], seglen[i]); }
	fxsha1_final(&s, inner);
	fxsha1_init(&s);
	fxsha1_update(&s, opad, 64u);
	fxsha1_update(&s, inner, 20u);
	fxsha1_final(&s, out);
}

static inline void
fxhmac_sha1(const uint8_t *key, size_t keylen, const uint8_t *msg, size_t msglen, uint8_t out[20])
{
	const uint8_t *seg[1]; size_t seglen[1];
	seg[0] = msg; seglen[0] = msglen;
	fxhmac_sha1_v(key, keylen, seg, seglen, 1u, out);
}

/* ======================================================================
 * PRF (IEEE 802.11-2020, 12.7.1.2). R = HMAC-SHA-1(K, A || 0 || B || i) for
 * i = 0, 1, ... truncated to the requested length; A is a null-terminated
 * label, so the single 0 octet that 12.7.1.2 appends is the label's own NUL.
 * ====================================================================== */
static inline void
fxprf(const uint8_t *key, size_t keylen, const char *label,
    const uint8_t *data, size_t datalen, uint8_t *out, size_t outlen)
{
	const uint8_t *seg[3]; size_t seglen[3];
	uint8_t hash[20], counter = 0; size_t pos = 0, labellen = 0;
	while (label[labellen] != '\0') { labellen++; }
	labellen += 1u;     /* include the terminating NUL: the "|| 0" of 12.7.1.2 */
	seg[0] = (const uint8_t *)label; seglen[0] = labellen;
	seg[1] = data; seglen[1] = datalen;
	seg[2] = &counter; seglen[2] = 1u;
	while (pos < outlen) {
		size_t take = outlen - pos;
		fxhmac_sha1_v(key, keylen, seg, seglen, 3u, hash);
		if (take > 20u) { take = 20u; }
		memcpy(out + pos, hash, take);
		pos += take; counter++;
	}
}

/* PTK = PRF-384(PMK, "Pairwise key expansion",
 *   min(AA,SPA) || max(AA,SPA) || min(ANonce,SNonce) || max(ANonce,SNonce)).
 * The result is KCK[0:16] || KEK[16:32] || TK[32:48] for CCMP (12.7.1.3). */
static inline void
fxeapol_ptk(const uint8_t pmk[32], const uint8_t aa[6], const uint8_t spa[6],
    const uint8_t anonce[32], const uint8_t snonce[32], uint8_t ptk[48])
{
	uint8_t b[76]; const uint8_t *lo, *hi;
	lo = (memcmp(aa, spa, 6) < 0) ? aa : spa;
	hi = (lo == aa) ? spa : aa;
	memcpy(b, lo, 6); memcpy(b + 6, hi, 6);
	lo = (memcmp(anonce, snonce, 32) < 0) ? anonce : snonce;
	hi = (lo == anonce) ? snonce : anonce;
	memcpy(b + 12, lo, 32); memcpy(b + 44, hi, 32);
	fxprf(pmk, 32u, "Pairwise key expansion", b, sizeof(b), ptk, 48u);
}

/* ======================================================================
 * AES-128 (FIPS 197) block encrypt and decrypt, and the NIST AES key wrap
 * and unwrap (RFC 3394). The 4-way message 3 and the group message 1 carry
 * their key data wrapped under the KEK; the supplicant only ever needs
 * unwrap, but encrypt and wrap are here so the same file drives both sides
 * of the host test. Byte-wise, column-major state; no tables beyond the two
 * S-boxes and Rcon.
 * ====================================================================== */
static const uint8_t fxaes_sbox[256] = {
	0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
	0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
	0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
	0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
	0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
	0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
	0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
	0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
	0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
	0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
	0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
	0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
	0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
	0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
	0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
	0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};
static const uint8_t fxaes_isbox[256] = {
	0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
	0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
	0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
	0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
	0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
	0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
	0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
	0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
	0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
	0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
	0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
	0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
	0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
	0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
	0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
	0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d
};
static const uint8_t fxaes_rcon[10] = { 0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36 };

static inline uint8_t
fxaes_mul(uint8_t a, uint8_t b)
{
	uint8_t r = 0; unsigned i;
	for (i = 0; i < 8u; i++) {
		if (b & 1u) { r ^= a; }
		{ uint8_t hi = (uint8_t)(a & 0x80u); a = (uint8_t)(a << 1); if (hi) { a ^= 0x1bu; } }
		b = (uint8_t)(b >> 1);
	}
	return r;
}

/* 11 round keys (176 bytes), word i at rk[4*i..]. Column-major, so the round
 * key for round r at AddRoundKey is rk[16*r + 4*c + b]. */
static inline void
fxaes128_expand(const uint8_t key[16], uint8_t rk[176])
{
	unsigned i; uint8_t t[4];
	memcpy(rk, key, 16u);
	for (i = 4; i < 44u; i++) {
		t[0] = rk[4 * (i - 1)]; t[1] = rk[4 * (i - 1) + 1];
		t[2] = rk[4 * (i - 1) + 2]; t[3] = rk[4 * (i - 1) + 3];
		if ((i & 3u) == 0u) {
			uint8_t r = t[0];
			t[0] = (uint8_t)(fxaes_sbox[t[1]] ^ fxaes_rcon[i / 4u - 1u]);
			t[1] = fxaes_sbox[t[2]]; t[2] = fxaes_sbox[t[3]]; t[3] = fxaes_sbox[r];
		}
		rk[4 * i] = (uint8_t)(rk[4 * (i - 4)] ^ t[0]);
		rk[4 * i + 1] = (uint8_t)(rk[4 * (i - 4) + 1] ^ t[1]);
		rk[4 * i + 2] = (uint8_t)(rk[4 * (i - 4) + 2] ^ t[2]);
		rk[4 * i + 3] = (uint8_t)(rk[4 * (i - 4) + 3] ^ t[3]);
	}
}

static inline void
fxaes_addkey(uint8_t s[16], const uint8_t *rk)
{
	unsigned i;
	for (i = 0; i < 16u; i++) { s[i] ^= rk[i]; }
}

static inline void
fxaes128_encrypt(const uint8_t rk[176], const uint8_t in[16], uint8_t out[16])
{
	uint8_t s[16]; unsigned round, c; uint8_t a0, a1, a2, a3, tmp;
	memcpy(s, in, 16u);
	fxaes_addkey(s, rk);
	for (round = 1; round <= 10u; round++) {
		unsigned i;
		for (i = 0; i < 16u; i++) { s[i] = fxaes_sbox[s[i]]; }
		/* ShiftRows (rows 1..3 rotate left by the row index), column-major. */
		tmp = s[1]; s[1] = s[5]; s[5] = s[9]; s[9] = s[13]; s[13] = tmp;
		tmp = s[2]; s[2] = s[10]; s[10] = tmp; tmp = s[6]; s[6] = s[14]; s[14] = tmp;
		tmp = s[15]; s[15] = s[11]; s[11] = s[7]; s[7] = s[3]; s[3] = tmp;
		if (round != 10u) {
			for (c = 0; c < 4u; c++) {
				a0 = s[4 * c]; a1 = s[4 * c + 1]; a2 = s[4 * c + 2]; a3 = s[4 * c + 3];
				s[4 * c] = (uint8_t)(fxaes_mul(a0, 2) ^ fxaes_mul(a1, 3) ^ a2 ^ a3);
				s[4 * c + 1] = (uint8_t)(a0 ^ fxaes_mul(a1, 2) ^ fxaes_mul(a2, 3) ^ a3);
				s[4 * c + 2] = (uint8_t)(a0 ^ a1 ^ fxaes_mul(a2, 2) ^ fxaes_mul(a3, 3));
				s[4 * c + 3] = (uint8_t)(fxaes_mul(a0, 3) ^ a1 ^ a2 ^ fxaes_mul(a3, 2));
			}
		}
		fxaes_addkey(s, rk + 16u * round);
	}
	memcpy(out, s, 16u);
}

static inline void
fxaes128_decrypt(const uint8_t rk[176], const uint8_t in[16], uint8_t out[16])
{
	uint8_t s[16]; unsigned round, c; uint8_t a0, a1, a2, a3, tmp;
	memcpy(s, in, 16u);
	fxaes_addkey(s, rk + 16u * 10u);
	for (round = 9; ; round--) {
		unsigned i;
		/* InvShiftRows (rotate right by the row index). */
		tmp = s[13]; s[13] = s[9]; s[9] = s[5]; s[5] = s[1]; s[1] = tmp;
		tmp = s[2]; s[2] = s[10]; s[10] = tmp; tmp = s[6]; s[6] = s[14]; s[14] = tmp;
		tmp = s[3]; s[3] = s[7]; s[7] = s[11]; s[11] = s[15]; s[15] = tmp;
		for (i = 0; i < 16u; i++) { s[i] = fxaes_isbox[s[i]]; }
		fxaes_addkey(s, rk + 16u * round);
		if (round != 0u) {
			for (c = 0; c < 4u; c++) {
				a0 = s[4 * c]; a1 = s[4 * c + 1]; a2 = s[4 * c + 2]; a3 = s[4 * c + 3];
				s[4 * c] = (uint8_t)(fxaes_mul(a0, 14) ^ fxaes_mul(a1, 11) ^ fxaes_mul(a2, 13) ^ fxaes_mul(a3, 9));
				s[4 * c + 1] = (uint8_t)(fxaes_mul(a0, 9) ^ fxaes_mul(a1, 14) ^ fxaes_mul(a2, 11) ^ fxaes_mul(a3, 13));
				s[4 * c + 2] = (uint8_t)(fxaes_mul(a0, 13) ^ fxaes_mul(a1, 9) ^ fxaes_mul(a2, 14) ^ fxaes_mul(a3, 11));
				s[4 * c + 3] = (uint8_t)(fxaes_mul(a0, 11) ^ fxaes_mul(a1, 13) ^ fxaes_mul(a2, 9) ^ fxaes_mul(a3, 14));
			}
		}
		if (round == 0u) { break; }
	}
	memcpy(out, s, 16u);
}

/* RFC 3394. The wrap produces inlen+8 bytes; the unwrap consumes a multiple
 * of 8 that is at least 16 and returns inlen-8 bytes, 1 on an integrity
 * failure (the recovered A is not the A6A6... default IV). */
#define FXE_KW_IV 0xa6u

static inline void
fxaes_wrap(const uint8_t kek[16], const uint8_t *in, size_t inlen, uint8_t *out)
{
	uint8_t rk[176], a[8], b[16]; size_t n = inlen / 8u, i, j;
	fxaes128_expand(kek, rk);
	memset(a, FXE_KW_IV, 8u);
	memcpy(out + 8u, in, inlen);
	for (j = 0; j < 6u; j++) {
		for (i = 1; i <= n; i++) {
			uint64_t t = n * j + i; unsigned k;
			memcpy(b, a, 8u); memcpy(b + 8u, out + 8u * i, 8u);
			fxaes128_encrypt(rk, b, b);
			memcpy(a, b, 8u);
			for (k = 0; k < 8u; k++) { a[7 - k] ^= (uint8_t)(t >> (8u * k)); }
			memcpy(out + 8u * i, b + 8u, 8u);
		}
	}
	memcpy(out, a, 8u);
}

static inline int
fxaes_unwrap(const uint8_t kek[16], const uint8_t *in, size_t inlen, uint8_t *out)
{
	uint8_t rk[176], a[8], b[16]; size_t n, i, j;
	if (inlen < 16u || (inlen % 8u) != 0u) { return 1; }
	n = inlen / 8u - 1u;
	fxaes128_expand(kek, rk);
	memcpy(a, in, 8u);
	memcpy(out, in + 8u, inlen - 8u);
	for (j = 6; j-- > 0; ) {
		for (i = n; i >= 1u; i--) {
			uint64_t t = n * j + i; unsigned k;
			for (k = 0; k < 8u; k++) { a[7 - k] ^= (uint8_t)(t >> (8u * k)); }
			memcpy(b, a, 8u); memcpy(b + 8u, out + 8u * (i - 1u), 8u);
			fxaes128_decrypt(rk, b, b);
			memcpy(a, b, 8u);
			memcpy(out + 8u * (i - 1u), b + 8u, 8u);
		}
	}
	for (i = 0; i < 8u; i++) { if (a[i] != FXE_KW_IV) { return 1; } }
	return 0;
}

/* ======================================================================
 * EAPOL-Key frame (IEEE 802.1X-2020 11.9 header, 802.11-2020 12.7.2 body).
 * Byte offsets from the start of the 802.1X PDU; no packed struct is laid
 * over the frame. Version 2 descriptor only: HMAC-SHA-1-128 MIC and the AES
 * key wrap. The MIC field is 16 bytes.
 * ====================================================================== */
#define FXE_O_VER        0u    /* 802.1X protocol version */
#define FXE_O_PKTTYPE    1u    /* 3 = EAPOL-Key */
#define FXE_O_PKTLEN     2u    /* 2 bytes: body length after this 4-byte header */
#define FXE_O_DESC       4u    /* 2 = RSN key descriptor */
#define FXE_O_INFO       5u    /* 2 bytes: key information */
#define FXE_O_KEYLEN     7u    /* 2 bytes */
#define FXE_O_REPLAY     9u    /* 8 bytes */
#define FXE_O_NONCE      17u   /* 32 bytes */
#define FXE_O_IV         49u   /* 16 bytes */
#define FXE_O_RSC        65u   /* 8 bytes */
#define FXE_O_RSVD       73u   /* 8 bytes */
#define FXE_O_MIC        81u   /* 16 bytes */
#define FXE_O_DATALEN    97u   /* 2 bytes */
#define FXE_O_DATA       99u
#define FXE_HDR          99u   /* fixed bytes before key data */
#define FXE_MICLEN       16u

#define FXE_PKTTYPE_KEY  3u
#define FXE_DESC_RSN     2u

#define FXE_KI_VERSION   0x0007u
#define FXE_KI_V2        0x0002u   /* HMAC-SHA-1-128 + AES key wrap */
#define FXE_KI_PAIRWISE  0x0008u   /* key type: 1 pairwise, 0 group */
#define FXE_KI_INSTALL   0x0040u
#define FXE_KI_ACK       0x0080u
#define FXE_KI_MIC       0x0100u
#define FXE_KI_SECURE    0x0200u
#define FXE_KI_ERROR     0x0400u
#define FXE_KI_REQUEST   0x0800u
#define FXE_KI_ENCRYPTED 0x1000u

static inline uint32_t fxe_get16(const uint8_t *p) { return ((uint32_t)p[0] << 8) | (uint32_t)p[1]; }
static inline void fxe_put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

/* KDE and RSN element (802.11-2020 9.4.2.24, 12.7.2 Table 12-9). The GTK KDE
 * is 00-0F-AC data type 1: one key-info byte (bits 0..1 key id, bit 2 Tx),
 * one reserved byte, then the GTK. The RSN element id is 48. */
#define FXE_EID_RSN      48u
#define FXE_EID_VENDOR   0xddu
static const uint8_t fxe_kde_oui[3] = { 0x00, 0x0f, 0xac };
#define FXE_KDE_GTK      1u

/* ======================================================================
 * Supplicant state machine (IEEE 802.11-2020 12.7.6 the 4-way handshake,
 * 12.7.7 the group-key handshake). The supplicant is purely reactive: the
 * authenticator drives the exchange and we answer. A timeout retransmits the
 * last answer a bounded number of times before the handshake is failed, so a
 * lost message 3 or group message 1 does not wedge the lane.
 * ====================================================================== */
enum {
	FXST_INIT = 0,     /* PMK and SNonce ready, waiting for message 1 */
	FXST_WAIT_M3,      /* message 2 sent, waiting for message 3 */
	FXST_PTK_DONE,     /* 4-way complete; group rekeys handled from here */
	FXST_FAILED        /* handshake aborted */
};

enum fxeapol_evt {
	FXE_NONE = 0,      /* frame consumed, nothing to transmit */
	FXE_TX,            /* out holds a frame to transmit now */
	FXE_DROP,          /* frame ignored (replay, or not an expected message) */
	FXE_FAIL           /* protocol violation; the handshake is aborted */
};

#define FXE_MAXFRAME 512u
#define FXE_RSN_MAX  64u
#define FXE_GTK_MAX  32u

typedef void (*fxeapol_rng)(void *ctx, uint8_t *out, size_t n);

struct fxeapol {
	uint8_t pmk[32];
	uint8_t spa[6];           /* our MAC (supplicant) */
	uint8_t aa[6];            /* AP MAC (authenticator) */
	uint8_t own_rsn[FXE_RSN_MAX]; uint8_t own_rsn_len;   /* the RSN element we put in the assoc request */
	uint8_t ap_rsn[FXE_RSN_MAX];  uint8_t ap_rsn_len;    /* the RSN element the beacon advertised */

	uint8_t snonce[32];
	uint8_t anonce[32];
	uint8_t ptk[48];          /* KCK | KEK | TK */
	bool    ptk_valid;

	uint8_t gtk[FXE_GTK_MAX]; uint8_t gtk_len; uint8_t gtk_id; bool gtk_tx; bool gtk_valid;

	uint8_t  replay[8];       /* last accepted replay counter */
	bool     replay_valid;

	int      state;
	bool     tk_installed;    /* set when the pairwise key should be programmed */
	bool     gtk_installed;   /* set when the group key should be programmed */

	unsigned retries;
	unsigned max_retries;
	uint32_t timer_ms;
	uint32_t timeout_ms;
	bool     waiting;         /* a response is outstanding and may be retransmitted */

	uint8_t  last_tx[FXE_MAXFRAME]; uint32_t last_tx_len;

	fxeapol_rng rng; void *rng_ctx;
};

/* Set up the supplicant. own_rsn is the RSN element sent in the assoc
 * request (placed verbatim in message 2); ap_rsn is the one from the beacon
 * (checked against message 3). The SNonce is drawn now from rng. */
static inline void
fxeapol_init(struct fxeapol *st, const uint8_t pmk[32], const uint8_t spa[6], const uint8_t aa[6],
    const uint8_t *own_rsn, uint8_t own_rsn_len, const uint8_t *ap_rsn, uint8_t ap_rsn_len,
    fxeapol_rng rng, void *rng_ctx)
{
	memset(st, 0, sizeof(*st));
	memcpy(st->pmk, pmk, 32u);
	memcpy(st->spa, spa, 6u);
	memcpy(st->aa, aa, 6u);
	if (own_rsn_len > FXE_RSN_MAX) { own_rsn_len = FXE_RSN_MAX; }
	if (ap_rsn_len > FXE_RSN_MAX) { ap_rsn_len = FXE_RSN_MAX; }
	memcpy(st->own_rsn, own_rsn, own_rsn_len); st->own_rsn_len = own_rsn_len;
	memcpy(st->ap_rsn, ap_rsn, ap_rsn_len); st->ap_rsn_len = ap_rsn_len;
	st->rng = rng; st->rng_ctx = rng_ctx;
	st->rng(st->rng_ctx, st->snonce, 32u);
	st->state = FXST_INIT;
	st->max_retries = 3u;
	st->timeout_ms = 1000u;
}

/* MIC over the whole PDU with the MIC field zeroed (12.7.6.3). Version 2 is
 * HMAC-SHA-1-128: the first 16 bytes of the HMAC. */
static inline void
fxeapol_mic(const uint8_t kck[16], const uint8_t *frame, uint32_t len, uint8_t mic[16])
{
	uint8_t copy[FXE_MAXFRAME], full[20];
	if (len > FXE_MAXFRAME) { len = FXE_MAXFRAME; }
	memcpy(copy, frame, len);
	memset(copy + FXE_O_MIC, 0, FXE_MICLEN);
	fxhmac_sha1(kck, 16u, copy, len, full);
	memcpy(mic, full, 16u);
}

/* Lay down the fixed header and key data of an outgoing frame, returning its
 * length. The MIC is left zero; the caller fills it after. */
static inline uint32_t
fxeapol_frame(uint8_t *out, uint32_t info, uint32_t keylen, const uint8_t replay[8],
    const uint8_t nonce[32], const uint8_t *data, uint32_t data_len)
{
	memset(out, 0, FXE_HDR);
	out[FXE_O_VER] = 2u;                 /* 802.1X-2010 and later */
	out[FXE_O_PKTTYPE] = (uint8_t)FXE_PKTTYPE_KEY;
	fxe_put16(out + FXE_O_PKTLEN, FXE_HDR - 4u + data_len);
	out[FXE_O_DESC] = (uint8_t)FXE_DESC_RSN;
	fxe_put16(out + FXE_O_INFO, info);
	fxe_put16(out + FXE_O_KEYLEN, keylen);
	memcpy(out + FXE_O_REPLAY, replay, 8u);
	if (nonce != 0) { memcpy(out + FXE_O_NONCE, nonce, 32u); }
	fxe_put16(out + FXE_O_DATALEN, data_len);
	if (data_len > 0) { memcpy(out + FXE_O_DATA, data, data_len); }
	return FXE_HDR + data_len;
}

/* Find a KDE or element in (already unwrapped) key data. For an RSN element
 * (id 48) the whole element including id and length is returned; for a GTK
 * KDE the body after the 00-0F-AC type byte is returned. Returns 0 if not
 * found. A length that overruns the buffer stops the walk. */
static inline const uint8_t *
fxeapol_find_rsn(const uint8_t *kd, uint32_t len, uint32_t *out_len)
{
	uint32_t i = 0;
	while (i + 2u <= len) {
		uint8_t id = kd[i], l = kd[i + 1];
		if (id == 0u || (id == FXE_EID_VENDOR && l == 0u)) { break; }  /* 12.7.2 padding (0xDD 0x00..) */
		if ((uint32_t)i + 2u + l > len) { break; }
		if (id == FXE_EID_RSN) { *out_len = (uint32_t)l + 2u; return kd + i; }
		i += 2u + l;
	}
	return 0;
}

static inline const uint8_t *
fxeapol_find_gtk(const uint8_t *kd, uint32_t len, uint32_t *out_len)
{
	uint32_t i = 0;
	while (i + 2u <= len) {
		uint8_t id = kd[i], l = kd[i + 1];
		if (id == 0u || (id == FXE_EID_VENDOR && l == 0u)) { break; }  /* padding */
		if ((uint32_t)i + 2u + l > len) { break; }
		if (id == FXE_EID_VENDOR && l >= 6u &&
		    memcmp(kd + i + 2u, fxe_kde_oui, 3u) == 0 && kd[i + 5u] == FXE_KDE_GTK) {
			*out_len = (uint32_t)l - 4u;     /* body after OUI + type */
			return kd + i + 6u;
		}
		i += 2u + l;
	}
	return 0;
}

/* Common validation shared by every received frame: the 802.1X header, the
 * descriptor type and version, the declared key-data length, and the MIC
 * when the MIC bit is set. Returns 0 on success, with *info and *data_len
 * filled; nonzero means the frame is malformed. kck may be 0 when no MIC is
 * expected (message 1). */
static inline int
fxeapol_check(const uint8_t *f, uint32_t len, const uint8_t *kck, uint32_t *info, uint32_t *data_len)
{
	uint32_t ki, dl;
	if (len < FXE_HDR) { return 1; }                                 /* truncation */
	if (f[FXE_O_PKTTYPE] != FXE_PKTTYPE_KEY || f[FXE_O_DESC] != FXE_DESC_RSN) { return 1; }
	ki = fxe_get16(f + FXE_O_INFO);
	if ((ki & FXE_KI_VERSION) != FXE_KI_V2) { return 1; }            /* v2 only */
	dl = fxe_get16(f + FXE_O_DATALEN);
	if ((uint64_t)FXE_HDR + dl > len) { return 1; }                  /* data overruns the frame */
	if (fxe_get16(f + FXE_O_PKTLEN) != FXE_HDR - 4u + dl) { return 1; }
	if (ki & FXE_KI_MIC) {
		uint8_t mic[16];
		if (kck == 0) { return 1; }
		fxeapol_mic(kck, f, FXE_HDR + dl, mic);
		if (memcmp(mic, f + FXE_O_MIC, FXE_MICLEN) != 0) { return 1; }   /* bad MIC */
	}
	*info = ki; *data_len = dl;
	return 0;
}

static inline int
fxeapol_replay_ok(struct fxeapol *st, const uint8_t rc[8])
{
	if (!st->replay_valid) { return 1; }
	return memcmp(rc, st->replay, 8u) > 0;     /* strictly newer */
}

/* Install the GTK carried (unwrapped) in body/body_len. Returns 0 on success,
 * 1 if the KDE is missing or too large. */
static inline int
fxeapol_take_gtk(struct fxeapol *st, const uint8_t *kd, uint32_t kdlen)
{
	uint32_t glen; const uint8_t *g = fxeapol_find_gtk(kd, kdlen, &glen);
	if (g == 0 || glen < 2u || glen - 2u > FXE_GTK_MAX) { return 1; }
	st->gtk_id = (uint8_t)(g[0] & 0x03u);
	st->gtk_tx = (g[0] & 0x04u) != 0u;
	st->gtk_len = (uint8_t)(glen - 2u);
	memcpy(st->gtk, g + 2u, st->gtk_len);
	st->gtk_valid = true;
	return 0;
}

/* Feed one received EAPOL-Key frame. On FXE_TX, out/outlen hold the response
 * to send. Key installation is signalled through st->tk_installed and
 * st->gtk_installed (with st->ptk / st->gtk holding the material). */
static inline enum fxeapol_evt
fxeapol_input(struct fxeapol *st, const uint8_t *f, uint32_t len, uint8_t *out, uint32_t outcap, uint32_t *outlen)
{
	uint32_t info, dl;
	const uint8_t *kck = st->ptk_valid ? st->ptk : 0;

	if (st->state == FXST_FAILED) { return FXE_DROP; }
	if (fxeapol_check(f, len, kck, &info, &dl) != 0) { st->state = FXST_FAILED; return FXE_FAIL; }

	/* Message 1 of the 4-way: pairwise, ACK, no MIC, not secure. */
	if ((info & FXE_KI_PAIRWISE) && (info & FXE_KI_ACK) && !(info & FXE_KI_MIC)) {
		uint32_t n; uint8_t mic[16];
		memcpy(st->anonce, f + FXE_O_NONCE, 32u);
		memcpy(st->replay, f + FXE_O_REPLAY, 8u); st->replay_valid = true;
		fxeapol_ptk(st->pmk, st->aa, st->spa, st->anonce, st->snonce, st->ptk);
		st->ptk_valid = true;
		/* Build message 2: our SNonce, our RSN element, MIC under the new KCK. */
		n = fxeapol_frame(out, FXE_KI_V2 | FXE_KI_PAIRWISE | FXE_KI_MIC,
		    fxe_get16(f + FXE_O_KEYLEN), st->replay, st->snonce, st->own_rsn, st->own_rsn_len);
		if (n > outcap) { st->state = FXST_FAILED; return FXE_FAIL; }
		fxeapol_mic(st->ptk, out, n, mic);
		memcpy(out + FXE_O_MIC, mic, FXE_MICLEN);
		*outlen = n;
		memcpy(st->last_tx, out, n); st->last_tx_len = n;
		st->state = FXST_WAIT_M3; st->waiting = true; st->timer_ms = 0; st->retries = 0;
		return FXE_TX;
	}

	/* From here every frame must carry a MIC (already verified) and a fresh
	 * replay counter, and we must already hold a PTK. */
	if (!(info & FXE_KI_MIC) || !st->ptk_valid) { return FXE_DROP; }
	if (!fxeapol_replay_ok(st, f + FXE_O_REPLAY)) { return FXE_DROP; }

	/* Message 3 of the 4-way: pairwise, ACK, MIC, secure, encrypted key data. */
	if ((info & FXE_KI_PAIRWISE) && (info & FXE_KI_ACK)) {
		uint8_t kd[FXE_MAXFRAME]; uint32_t kdlen; const uint8_t *rsn; uint32_t rsnlen; uint32_t n; uint8_t mic[16];
		if (memcmp(f + FXE_O_NONCE, st->anonce, 32u) != 0) { st->state = FXST_FAILED; return FXE_FAIL; }
		if (!(info & FXE_KI_ENCRYPTED)) { st->state = FXST_FAILED; return FXE_FAIL; }
		if (dl < 16u || (dl % 8u) != 0u) { st->state = FXST_FAILED; return FXE_FAIL; }  /* not a wrap block */
		if (fxaes_unwrap(st->ptk + 16u, f + FXE_O_DATA, dl, kd) != 0) { st->state = FXST_FAILED; return FXE_FAIL; }
		kdlen = dl - 8u;
		rsn = fxeapol_find_rsn(kd, kdlen, &rsnlen);
		if (rsn == 0 || rsnlen != st->ap_rsn_len || memcmp(rsn, st->ap_rsn, rsnlen) != 0) {
			st->state = FXST_FAILED; return FXE_FAIL;                   /* RSN element must equal the beacon's */
		}
		if (fxeapol_take_gtk(st, kd, kdlen) != 0) { st->state = FXST_FAILED; return FXE_FAIL; }
		memcpy(st->replay, f + FXE_O_REPLAY, 8u);
		/* Build message 4: MIC, secure, no key data, nonce zero. */
		{ uint8_t z[32]; memset(z, 0, 32u);
		  n = fxeapol_frame(out, FXE_KI_V2 | FXE_KI_PAIRWISE | FXE_KI_MIC | FXE_KI_SECURE,
		      fxe_get16(f + FXE_O_KEYLEN), st->replay, z, 0, 0); }
		if (n > outcap) { st->state = FXST_FAILED; return FXE_FAIL; }
		fxeapol_mic(st->ptk, out, n, mic);
		memcpy(out + FXE_O_MIC, mic, FXE_MICLEN);
		*outlen = n;
		memcpy(st->last_tx, out, n); st->last_tx_len = n;
		st->tk_installed = true;              /* Install bit directs the pairwise key (TK = ptk[32:48]) */
		st->gtk_installed = true;             /* and the GTK that message 3 carried */
		st->state = FXST_PTK_DONE; st->waiting = false;
		return FXE_TX;
	}

	/* Group-key message 1: group (not pairwise), ACK, MIC, secure, encrypted. */
	if (!(info & FXE_KI_PAIRWISE) && (info & FXE_KI_ACK)) {
		uint8_t kd[FXE_MAXFRAME]; uint32_t n; uint8_t mic[16]; uint8_t z[32];
		if (st->state != FXST_PTK_DONE) { return FXE_DROP; }
		if (!(info & FXE_KI_ENCRYPTED)) { st->state = FXST_FAILED; return FXE_FAIL; }
		if (dl < 16u || (dl % 8u) != 0u) { st->state = FXST_FAILED; return FXE_FAIL; }
		if (fxaes_unwrap(st->ptk + 16u, f + FXE_O_DATA, dl, kd) != 0) { st->state = FXST_FAILED; return FXE_FAIL; }
		if (fxeapol_take_gtk(st, kd, dl - 8u) != 0) { st->state = FXST_FAILED; return FXE_FAIL; }
		memcpy(st->replay, f + FXE_O_REPLAY, 8u);
		memset(z, 0, 32u);
		n = fxeapol_frame(out, FXE_KI_V2 | FXE_KI_MIC | FXE_KI_SECURE,
		    fxe_get16(f + FXE_O_KEYLEN), st->replay, z, 0, 0);
		if (n > outcap) { st->state = FXST_FAILED; return FXE_FAIL; }
		fxeapol_mic(st->ptk, out, n, mic);
		memcpy(out + FXE_O_MIC, mic, FXE_MICLEN);
		*outlen = n;
		memcpy(st->last_tx, out, n); st->last_tx_len = n;
		st->gtk_installed = true;
		return FXE_TX;
	}

	return FXE_DROP;
}

/* Advance the retransmission timer. If a response is outstanding and the
 * timeout elapses, the last response is retransmitted, up to max_retries;
 * beyond that the handshake fails. Returns FXE_TX (out holds the resend),
 * FXE_NONE (nothing due) or FXE_FAIL (retries exhausted). */
static inline enum fxeapol_evt
fxeapol_tick(struct fxeapol *st, uint32_t elapsed_ms, uint8_t *out, uint32_t outcap, uint32_t *outlen)
{
	if (!st->waiting || st->state == FXST_FAILED) { return FXE_NONE; }
	st->timer_ms += elapsed_ms;
	if (st->timer_ms < st->timeout_ms) { return FXE_NONE; }
	st->timer_ms = 0;
	if (st->retries >= st->max_retries) { st->state = FXST_FAILED; st->waiting = false; return FXE_FAIL; }
	st->retries++;
	if (st->last_tx_len > outcap) { st->state = FXST_FAILED; return FXE_FAIL; }
	memcpy(out, st->last_tx, st->last_tx_len);
	*outlen = st->last_tx_len;
	return FXE_TX;
}

#endif /* FUXI_WLAN_EAPOL_H */
