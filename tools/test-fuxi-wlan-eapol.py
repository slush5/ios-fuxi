#!/usr/bin/env python3
"""EAPOL supplicant (RSN 4-way and group-key handshakes), host-tested.

Compiles a C harness that includes the pure xnu/fuxi-wlan/fuxi_wlan_eapol.h
with ASan/UBSan and -Werror (the way tools/test-fuxi-wlan-ce.py builds its
model), then exercises it two ways:

  1. The harness runs the published standard vectors against its own
     from-scratch crypto: FIPS 180-4 SHA-1, RFC 2202 HMAC-SHA-1, FIPS 197
     AES-128 (encrypt and decrypt), RFC 3394 key wrap and unwrap (with an
     integrity-failure case), and the IEEE 802.11-2020 Annex J PRF and PTK
     construction. Every vector is also recomputed here, independently, from
     hashlib and a pure-Python AES/key-wrap, and the two are compared, so the
     C primitives are pinned both to the printed constants and to a second
     implementation.

  2. A command loop ('drive') lets this script play the authenticator: it
     builds message 1 and message 3 of the 4-way handshake and group message
     1, hands them to the supplicant, and checks the message 2 / message 4 /
     group message 2 it gets back (bit for bit, MIC and all), the installed
     PTK and GTK, the replay counter, and the install flags. On top of the
     clean run it drives the negatives the standard requires a supplicant to
     reject (bad MIC, replay, the wrong RSN element in message 3, a truncated
     frame, key data that is not a multiple of 8, an AES-unwrap integrity
     failure) and a mutation sweep: every single-byte flip of a valid,
     MIC-protected message 3 must be refused.

No device, no network. Run: CC=cc python3 -B tools/test-fuxi-wlan-eapol.py
"""
from pathlib import Path
import hashlib
import hmac
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / 'xnu/fuxi-wlan/fuxi_wlan_eapol.h'
CC = [os.environ.get('CC', 'cc'), '-std=c11', '-D_DEFAULT_SOURCE', '-g', '-Wall', '-Wextra', '-Werror',
      '-Wno-unused-function', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']

# ---------------------------------------------------------------------------
# An independent reference implementation of the same primitives, used only to
# cross-check the C. SHA-1 and HMAC come from the standard library; AES-128 and
# the RFC 3394 wrap are written out here so nothing but the Python runtime is
# assumed.
# ---------------------------------------------------------------------------
SBOX = bytes.fromhex(
    '637c777bf26b6fc53001672bfed7ab76ca82c97dfa5947f0add4a2af9ca472c0'
    'b7fd9326363ff7cc34a5e5f171d8311504c723c31896059a071280e2eb27b275'
    '09832c1a1b6e5aa0523bd6b329e32f8453d100ed20fcb15b6acbbe394a4c58cf'
    'd0efaafb434d338545f9027f503c9fa851a3408f929d38f5bcb6da2110fff3d2'
    'cd0c13ec5f974417c4a77e3d645d197360814fdc222a908846eeb814de5e0bdb'
    'e0323a0a4906245cc2d3ac629195e479e7c8376d8dd54ea96c56f4ea657aae08'
    'ba78252e1ca6b4c6e8dd741f4bbd8b8a703eb5664803f60e613557b986c11d9e'
    'e1f8981169d98e949b1e87e9ce5528df8ca1890dbfe6426841992d0fb054bb16')
ISBOX = [0] * 256
for _i, _v in enumerate(SBOX):
    ISBOX[_v] = _i
ISBOX = bytes(ISBOX)
RCON = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36]


def gmul(a, b):
    r = 0
    for _ in range(8):
        if b & 1:
            r ^= a
        hi = a & 0x80
        a = (a << 1) & 0xff
        if hi:
            a ^= 0x1b
        b >>= 1
    return r


def aes_expand(key):
    w = [list(key[4 * i:4 * i + 4]) for i in range(4)]
    for i in range(4, 44):
        t = list(w[i - 1])
        if i % 4 == 0:
            t = t[1:] + t[:1]
            t = [SBOX[x] for x in t]
            t[0] ^= RCON[i // 4 - 1]
        w.append([w[i - 4][j] ^ t[j] for j in range(4)])
    return [b for word in w for b in word]


def _rounds(state, rk, enc):
    s = list(state)

    def addkey(r):
        for i in range(16):
            s[i] ^= rk[16 * r + i]

    def sub(box):
        for i in range(16):
            s[i] = box[s[i]]

    def shift(inv):
        for row in range(1, 4):
            col = [s[4 * c + row] for c in range(4)]
            k = -row if inv else row
            col = col[k % 4:] + col[:k % 4]
            for c in range(4):
                s[4 * c + row] = col[c]

    def mix(inv):
        m = (14, 11, 13, 9) if inv else (2, 3, 1, 1)
        for c in range(4):
            a = s[4 * c:4 * c + 4]
            for r in range(4):
                s[4 * c + r] = gmul(a[0], m[(0 - r) % 4]) ^ gmul(a[1], m[(1 - r) % 4]) ^ \
                    gmul(a[2], m[(2 - r) % 4]) ^ gmul(a[3], m[(3 - r) % 4])

    if enc:
        addkey(0)
        for rnd in range(1, 10):
            sub(SBOX)
            shift(False)
            mix(False)
            addkey(rnd)
        sub(SBOX)
        shift(False)
        addkey(10)
    else:
        addkey(10)
        for rnd in range(9, 0, -1):
            shift(True)
            sub(ISBOX)
            addkey(rnd)
            mix(True)
        shift(True)
        sub(ISBOX)
        addkey(0)
    return bytes(s)


def aes_encrypt(key, block):
    return _rounds(block, aes_expand(key), True)


def aes_decrypt(key, block):
    return _rounds(block, aes_expand(key), False)


def aes_wrap(kek, data):
    n = len(data) // 8
    a = b'\xa6' * 8
    r = [data[8 * i:8 * i + 8] for i in range(n)]
    for j in range(6):
        for i in range(1, n + 1):
            b = aes_encrypt(kek, a + r[i - 1])
            t = n * j + i
            a = bytes(x ^ ((t >> (8 * (7 - k))) & 0xff) for k, x in enumerate(b[:8]))
            r[i - 1] = b[8:]
    return a + b''.join(r)


def aes_unwrap(kek, data):
    n = len(data) // 8 - 1
    a = bytearray(data[:8])
    r = [bytearray(data[8 * (i + 1):8 * (i + 1) + 8]) for i in range(n)]
    for j in range(5, -1, -1):
        for i in range(n, 0, -1):
            t = n * j + i
            a = bytes(x ^ ((t >> (8 * (7 - k))) & 0xff) for k, x in enumerate(a))
            b = aes_decrypt(kek, bytes(a) + bytes(r[i - 1]))
            a = bytearray(b[:8])
            r[i - 1] = bytearray(b[8:])
    if bytes(a) != b'\xa6' * 8:
        raise ValueError('key unwrap integrity failure')
    return b''.join(bytes(x) for x in r)


def prf(key, label, data, outbytes):
    out, i = b'', 0
    while len(out) < outbytes:
        out += hmac.new(key, label + b'\x00' + data + bytes([i]), hashlib.sha1).digest()
        i += 1
    return out[:outbytes]


def derive_ptk(pmk, aa, spa, anonce, snonce):
    b = min(aa, spa) + max(aa, spa) + min(anonce, snonce) + max(anonce, snonce)
    return prf(pmk, b'Pairwise key expansion', b, 48)


def eapol_mic(kck, frame):
    z = bytearray(frame)
    z[81:97] = b'\x00' * 16
    return hmac.new(kck, bytes(z), hashlib.sha1).digest()[:16]


# EAPOL-Key field offsets, matching the header.
KI_V2, KI_PAIRWISE, KI_INSTALL, KI_ACK = 0x0002, 0x0008, 0x0040, 0x0080
KI_MIC, KI_SECURE, KI_ENCRYPTED = 0x0100, 0x0200, 0x1000


def build_eapol(info, keylen, replay, nonce, data=b'', kck=None):
    f = bytearray(99 + len(data))
    f[0] = 2
    f[1] = 3
    f[2:4] = (95 + len(data)).to_bytes(2, 'big')
    f[4] = 2
    f[5:7] = info.to_bytes(2, 'big')
    f[7:9] = keylen.to_bytes(2, 'big')
    f[9:17] = replay
    f[17:49] = nonce
    f[97:99] = len(data).to_bytes(2, 'big')
    f[99:] = data
    if kck is not None:
        f[81:97] = eapol_mic(kck, bytes(f))
    return bytes(f)


def gtk_kde(gtk, keyid, tx):
    body = bytes([(keyid & 3) | (0x04 if tx else 0), 0]) + gtk
    inner = bytes([0x00, 0x0f, 0xac, 0x01]) + body
    return bytes([0xdd, len(inner)]) + inner


def pad_key_data(kd):
    kd = bytearray(kd)
    if len(kd) < 16 or len(kd) % 8:
        kd.append(0xdd)
        while len(kd) % 8:
            kd.append(0x00)
    return bytes(kd)


# A made-up but well-formed WPA2-PSK/CCMP RSN element (one pairwise cipher,
# one AKM), used both as the station's assoc-request element and, echoed, as
# the AP's beacon element.
RSN_IE = bytes.fromhex('30140100000fac040100000fac040100000fac020000')


# ---------------------------------------------------------------------------
# The C harness. Written to a temp file, it includes the header and offers a
# self-test mode and a 'drive' command loop.
# ---------------------------------------------------------------------------
HARNESS = r'''
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "fuxi_wlan_eapol.h"

static int fails;
static int checks;

static void expect(int cond, const char *name) {
    checks++;
    if (!cond) { printf("FAIL %s\n", name); fails++; }
}

static int hexval(int c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static size_t unhex(const char *s, uint8_t *out, size_t cap) {
    size_t n = 0;
    while (s[0] && s[1] && hexval((unsigned char)s[0]) >= 0) {
        if (n >= cap) break;
        out[n++] = (uint8_t)(hexval((unsigned char)s[0]) * 16 + hexval((unsigned char)s[1]));
        s += 2;
    }
    return n;
}

static void puthex(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) printf("%02x", p[i]);
}

/* Fixed SNonce fed to the supplicant in drive mode. */
static uint8_t g_snonce[32];
static void drive_rng(void *ctx, uint8_t *out, size_t n) { (void)ctx; memcpy(out, g_snonce, n < 32u ? n : 32u); }

/* ---- standard vectors ---- */
static int selftest(void) {
    uint8_t out[64], d[20];

    /* FIPS 180-4 SHA-1. */
    fxsha1((const uint8_t *)"abc", 3, d);
    expect(memcmp(d, (uint8_t[]){0xa9,0x99,0x3e,0x36,0x47,0x06,0x81,0x6a,0xba,0x3e,0x25,0x71,0x78,0x50,0xc2,0x6c,0x9c,0xd0,0xd8,0x9d}, 20) == 0, "sha1 abc");
    fxsha1((const uint8_t *)"", 0, d);
    expect(memcmp(d, (uint8_t[]){0xda,0x39,0xa3,0xee,0x5e,0x6b,0x4b,0x0d,0x32,0x55,0xbf,0xef,0x95,0x60,0x18,0x90,0xaf,0xd8,0x07,0x09}, 20) == 0, "sha1 empty");
    fxsha1((const uint8_t *)"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq", 56, d);
    expect(memcmp(d, (uint8_t[]){0x84,0x98,0x3e,0x44,0x1c,0x3b,0xd2,0x6e,0xba,0xae,0x4a,0xa1,0xf9,0x51,0x29,0xe5,0xe5,0x46,0x70,0xf1}, 20) == 0, "sha1 56");

    /* RFC 2202 HMAC-SHA-1. */
    { uint8_t k[20]; memset(k, 0x0b, 20);
      fxhmac_sha1(k, 20, (const uint8_t *)"Hi There", 8, d);
      expect(memcmp(d, (uint8_t[]){0xb6,0x17,0x31,0x86,0x55,0x05,0x72,0x64,0xe2,0x8b,0xc0,0xb6,0xfb,0x37,0x8c,0x8e,0xf1,0x46,0xbe,0x00}, 20) == 0, "hmac rfc2202-1"); }
    fxhmac_sha1((const uint8_t *)"Jefe", 4, (const uint8_t *)"what do ya want for nothing?", 28, d);
    expect(memcmp(d, (uint8_t[]){0xef,0xfc,0xdf,0x6a,0xe5,0xeb,0x2f,0xa2,0xd2,0x74,0x16,0xd5,0xf1,0x84,0xdf,0x9c,0x25,0x9a,0x7c,0x79}, 20) == 0, "hmac rfc2202-2");

    /* FIPS 197 AES-128 (C.1). */
    { uint8_t key[16], pt[16], ct[16], rk[176], o[16];
      unhex("000102030405060708090a0b0c0d0e0f", key, 16);
      unhex("00112233445566778899aabbccddeeff", pt, 16);
      unhex("69c4e0d86a7b0430d8cdb78070b4c55a", ct, 16);
      fxaes128_expand(key, rk);
      fxaes128_encrypt(rk, pt, o); expect(memcmp(o, ct, 16) == 0, "aes128 encrypt");
      fxaes128_decrypt(rk, ct, o); expect(memcmp(o, pt, 16) == 0, "aes128 decrypt"); }

    /* RFC 3394 4.1: wrap 128-bit key with 128-bit KEK, and the round trip. */
    { uint8_t kek[16], key[16], want[24], w[24], u[16];
      unhex("000102030405060708090a0b0c0d0e0f", kek, 16);
      unhex("00112233445566778899aabbccddeeff", key, 16);
      unhex("1fa68b0a8112b447aef34bd8fb5a7b829d3e862371d2cfe5", want, 24);
      fxaes_wrap(kek, key, 16, w); expect(memcmp(w, want, 24) == 0, "rfc3394 wrap");
      expect(fxaes_unwrap(kek, want, 24, u) == 0 && memcmp(u, key, 16) == 0, "rfc3394 unwrap");
      w[0] ^= 0x01; expect(fxaes_unwrap(kek, w, 24, u) != 0, "rfc3394 unwrap integrity"); }

    /* IEEE 802.11-2020 Annex J style PRF and PTK: printed for the Python side
     * to re-derive from hashlib. */
    { uint8_t pmk[32], aa[6], spa[6], an[32], sn[32], ptk[48];
      memset(pmk, 0x11, 32); memset(an, 0x22, 32); memset(sn, 0x33, 32);
      unhex("0203040506aa", aa, 6); unhex("0203040506bb", spa, 6);
      fxprf(pmk, 32, "Pairwise key expansion", (const uint8_t *)"test", 4, out, 48);
      printf("PRF "); puthex(out, 48); printf("\n");
      fxeapol_ptk(pmk, aa, spa, an, sn, ptk);
      printf("PTK "); puthex(ptk, 48); printf("\n"); }

    printf("crypto vectors: %d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}

/* ---- command loop: this program is the supplicant; stdin is the AP ---- */
static struct fxeapol st;

static void print_state(enum fxeapol_evt ev, const uint8_t *tx, uint32_t txn) {
    printf("evt %d state %d ptkv %d tk %d gtk %d gtkv %d gid %d gtx %d ",
        (int)ev, st.state, st.ptk_valid, st.tk_installed, st.gtk_installed, st.gtk_valid, st.gtk_id, st.gtk_tx);
    printf("ptk "); if (st.ptk_valid) puthex(st.ptk, 48); else printf("-");
    printf(" gk "); if (st.gtk_valid) puthex(st.gtk, st.gtk_len); else printf("-");
    printf(" tx "); if (ev == FXE_TX) puthex(tx, txn); else printf("-");
    printf("\n");
}

static int drive(void) {
    char line[8192];
    uint8_t buf[4096], out[FXE_MAXFRAME];
    uint32_t outn;
    while (fgets(line, sizeof(line), stdin)) {
        char *cmd = strtok(line, " \t\r\n");
        if (!cmd) continue;
        if (strcmp(cmd, "init") == 0) {
            uint8_t pmk[32], spa[6], aa[6], own[64], apr[64]; uint8_t ownl, aprl;
            char *a[6]; int i;
            for (i = 0; i < 6; i++) { a[i] = strtok(NULL, " \t\r\n"); if (!a[i]) return 2; }
            unhex(a[0], pmk, 32); unhex(a[1], spa, 6); unhex(a[2], aa, 6);
            ownl = (uint8_t)unhex(a[3], own, 64); aprl = (uint8_t)unhex(a[4], apr, 64);
            unhex(a[5], g_snonce, 32);
            fxeapol_init(&st, pmk, spa, aa, own, ownl, apr, aprl, drive_rng, NULL);
            printf("init ok\n");
        } else if (strcmp(cmd, "rx") == 0) {
            char *h = strtok(NULL, " \t\r\n"); size_t n = h ? unhex(h, buf, sizeof(buf)) : 0;
            enum fxeapol_evt ev = fxeapol_input(&st, buf, (uint32_t)n, out, sizeof(out), &outn);
            print_state(ev, out, outn);
        } else if (strcmp(cmd, "tick") == 0) {
            char *m = strtok(NULL, " \t\r\n"); uint32_t ms = m ? (uint32_t)strtoul(m, NULL, 10) : 0;
            enum fxeapol_evt ev = fxeapol_tick(&st, ms, out, sizeof(out), &outn);
            print_state(ev, out, outn);
        } else {
            printf("err unknown %s\n", cmd);
        }
        fflush(stdout);
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc > 1 && strcmp(argv[1], "drive") == 0) return drive();
    return selftest();
}
'''


class Driver:
    """Talks to the C harness in drive mode, one command per line."""

    def __init__(self, exe):
        self.p = subprocess.Popen([str(exe), 'drive'], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, text=True)

    def cmd(self, line):
        self.p.stdin.write(line + '\n')
        self.p.stdin.flush()
        return self.p.stdout.readline().strip()

    def rx(self, frame):
        return self.parse(self.cmd('rx ' + frame.hex()))

    def tick(self, ms):
        return self.parse(self.cmd('tick %d' % ms))

    @staticmethod
    def parse(line):
        t = line.split()
        d = {}
        i = 0
        while i < len(t):
            k = t[i]
            v = t[i + 1]
            d[k] = v
            i += 2
        d['evt'] = int(d['evt'])
        for b in ('state', 'ptkv', 'tk', 'gtk', 'gtkv', 'gid', 'gtx'):
            d[b] = int(d[b])
        d['ptk'] = b'' if d['ptk'] == '-' else bytes.fromhex(d['ptk'])
        d['gk'] = b'' if d['gk'] == '-' else bytes.fromhex(d['gk'])
        d['tx'] = b'' if d['tx'] == '-' else bytes.fromhex(d['tx'])
        return d

    def close(self):
        self.p.stdin.close()
        self.p.wait()


# Event codes, matching enum fxeapol_evt.
FXE_NONE, FXE_TX, FXE_DROP, FXE_FAIL = 0, 1, 2, 3


class Count:
    def __init__(self):
        self.n = 0

    def ok(self, cond, msg):
        self.n += 1
        if not cond:
            raise AssertionError(msg)


def check_crosscheck(exe, cnt):
    """Re-derive the PRF and PTK the harness printed, from hashlib."""
    out = subprocess.run([str(exe)], capture_output=True, text=True)
    print(out.stdout.strip())
    if out.returncode:
        print(out.stderr[-4000:])
        raise SystemExit('EAPOL self-test (standard vectors) failed')
    vals = {}
    for ln in out.stdout.splitlines():
        if ln.startswith('PRF ') or ln.startswith('PTK '):
            vals[ln[:3]] = bytes.fromhex(ln[4:])
    pmk = b'\x11' * 32
    want_prf = prf(pmk, b'Pairwise key expansion', b'test', 48)
    cnt.ok(vals['PRF'] == want_prf, 'PRF cross-check vs hashlib')
    want_ptk = derive_ptk(pmk, bytes.fromhex('0203040506aa'), bytes.fromhex('0203040506bb'),
                          b'\x22' * 32, b'\x33' * 32)
    cnt.ok(vals['PTK'] == want_ptk, 'PTK cross-check vs hashlib')
    # And the Python AES / key wrap against the same published constants.
    cnt.ok(aes_encrypt(bytes.fromhex('000102030405060708090a0b0c0d0e0f'),
                       bytes.fromhex('00112233445566778899aabbccddeeff')).hex()
           == '69c4e0d86a7b0430d8cdb78070b4c55a', 'python aes fips197')
    cnt.ok(aes_wrap(bytes.fromhex('000102030405060708090a0b0c0d0e0f'),
                    bytes.fromhex('00112233445566778899aabbccddeeff')).hex()
           == '1fa68b0a8112b447aef34bd8fb5a7b829d3e862371d2cfe5', 'python wrap rfc3394')


def full_handshake(exe, cnt):
    """A complete authenticator-side 4-way + group-key run against the C
    supplicant, every returned frame verified."""
    pmk = bytes(range(32))
    spa = bytes.fromhex('02112233aa01')
    aa = bytes.fromhex('02ffeedd0001')
    snonce = bytes.fromhex('5a') * 32
    anonce = bytes.fromhex('a5') * 32
    gtk = bytes.fromhex('000102030405060708090a0b0c0d0e0f')
    ptk = derive_ptk(pmk, aa, spa, anonce, snonce)
    kck, kek = ptk[:16], ptk[16:32]

    d = Driver(exe)
    try:
        r = d.cmd('init %s %s %s %s %s %s' % (pmk.hex(), spa.hex(), aa.hex(),
                                              RSN_IE.hex(), RSN_IE.hex(), snonce.hex()))
        cnt.ok(r == 'init ok', 'init')

        # Message 1 -> message 2.
        rc1 = (1).to_bytes(8, 'big')
        m1 = build_eapol(KI_V2 | KI_PAIRWISE | KI_ACK, 16, rc1, anonce)
        s = d.rx(m1)
        cnt.ok(s['evt'] == FXE_TX, 'M1 yields a TX')
        cnt.ok(s['ptkv'] == 1 and s['ptk'] == ptk, 'PTK derived from M1 matches the AP')
        m2 = s['tx']
        info2 = int.from_bytes(m2[5:7], 'big')
        cnt.ok(info2 & KI_MIC and info2 & KI_PAIRWISE and not (info2 & KI_ACK)
               and not (info2 & KI_SECURE) and not (info2 & KI_ENCRYPTED), 'M2 key-info bits')
        cnt.ok(m2[9:17] == rc1, 'M2 echoes the replay counter')
        cnt.ok(m2[17:49] == snonce, 'M2 carries our SNonce')
        dl2 = int.from_bytes(m2[97:99], 'big')
        cnt.ok(m2[99:99 + dl2] == RSN_IE, 'M2 carries our RSN element')
        cnt.ok(eapol_mic(kck, m2) == m2[81:97], 'M2 MIC verifies under KCK')
        cnt.ok(s['tk'] == 0 and s['gtk'] == 0, 'no keys installed yet after M2')

        # Message 3 -> message 4.
        kd = pad_key_data(RSN_IE + gtk_kde(gtk, 2, False))
        rc2 = (2).to_bytes(8, 'big')
        m3 = build_eapol(KI_V2 | KI_PAIRWISE | KI_ACK | KI_MIC | KI_INSTALL | KI_SECURE | KI_ENCRYPTED,
                         16, rc2, anonce, aes_wrap(kek, kd), kck=kck)
        s = d.rx(m3)
        cnt.ok(s['evt'] == FXE_TX, 'M3 yields a TX')
        cnt.ok(s['tk'] == 1 and s['gtk'] == 1, 'M3 installs the pairwise and group keys')
        cnt.ok(s['gtkv'] == 1 and s['gk'] == gtk and s['gid'] == 2 and s['gtx'] == 0, 'GTK, key id and Tx bit')
        m4 = s['tx']
        info4 = int.from_bytes(m4[5:7], 'big')
        cnt.ok(info4 & KI_MIC and info4 & KI_SECURE and info4 & KI_PAIRWISE and not (info4 & KI_ACK)
               and not (info4 & KI_ENCRYPTED), 'M4 key-info bits')
        cnt.ok(m4[9:17] == rc2, 'M4 echoes the message-3 replay counter')
        cnt.ok(m4[17:49] == b'\x00' * 32, 'M4 nonce is zero')
        cnt.ok(int.from_bytes(m4[97:99], 'big') == 0, 'M4 has no key data')
        cnt.ok(eapol_mic(kck, m4) == m4[81:97], 'M4 MIC verifies under KCK')

        # Group-key message 1 -> message 2, with a fresh GTK and a higher counter.
        gtk2 = bytes.fromhex('f0e1d2c3b4a5968778695a4b3c2d1e0f')
        kd_g = pad_key_data(gtk_kde(gtk2, 3, True))
        rc3 = (3).to_bytes(8, 'big')
        g1 = build_eapol(KI_V2 | KI_ACK | KI_MIC | KI_SECURE | KI_ENCRYPTED,
                         16, rc3, b'\x00' * 32, aes_wrap(kek, kd_g), kck=kck)
        s = d.rx(g1)
        cnt.ok(s['evt'] == FXE_TX, 'group M1 yields a TX')
        cnt.ok(s['gtkv'] == 1 and s['gk'] == gtk2 and s['gid'] == 3 and s['gtx'] == 1, 'group rekey installs the new GTK')
        g2 = s['tx']
        infog = int.from_bytes(g2[5:7], 'big')
        cnt.ok(infog & KI_MIC and infog & KI_SECURE and not (infog & KI_PAIRWISE)
               and not (infog & KI_ACK), 'group M2 key-info bits')
        cnt.ok(g2[9:17] == rc3, 'group M2 echoes the replay counter')
        cnt.ok(eapol_mic(kck, g2) == g2[81:97], 'group M2 MIC verifies')
    finally:
        d.close()
    return (pmk, spa, aa, snonce, anonce, ptk, kck, kek, m3)


def arm_to_m3(exe, ctx):
    """A fresh supplicant driven through M1/M2 so it is waiting for M3."""
    pmk, spa, aa, snonce, anonce = ctx[0], ctx[1], ctx[2], ctx[3], ctx[4]
    d = Driver(exe)
    d.cmd('init %s %s %s %s %s %s' % (pmk.hex(), spa.hex(), aa.hex(), RSN_IE.hex(), RSN_IE.hex(), snonce.hex()))
    rc1 = (1).to_bytes(8, 'big')
    d.rx(build_eapol(KI_V2 | KI_PAIRWISE | KI_ACK, 16, rc1, anonce))
    return d


def negatives(exe, ctx, cnt):
    pmk, spa, aa, snonce, anonce, ptk, kck, kek, good_m3 = ctx
    rc2 = (2).to_bytes(8, 'big')
    base_flags = KI_V2 | KI_PAIRWISE | KI_ACK | KI_MIC | KI_INSTALL | KI_SECURE | KI_ENCRYPTED
    kd = pad_key_data(RSN_IE + gtk_kde(bytes(16), 1, False))

    # Bad MIC: a valid M3 with one MIC byte flipped.
    d = arm_to_m3(exe, ctx)
    bad = bytearray(good_m3)
    bad[81] ^= 0x01
    s = d.rx(bytes(bad))
    cnt.ok(s['evt'] == FXE_FAIL and s['tk'] == 0, 'bad MIC in M3 is rejected')
    d.close()

    # Replay: a correctly-MIC'd M3 whose counter is not newer than M1's.
    d = arm_to_m3(exe, ctx)
    rc_old = (1).to_bytes(8, 'big')
    m3_replay = build_eapol(base_flags, 16, rc_old, anonce, aes_wrap(kek, kd), kck=kck)
    s = d.rx(m3_replay)
    cnt.ok(s['evt'] == FXE_DROP and s['tk'] == 0, 'replayed M3 (stale counter) is dropped')
    d.close()

    # Wrong RSN element in M3 (does not match the beacon's).
    d = arm_to_m3(exe, ctx)
    other_rsn = bytes.fromhex('30140100000fac040100000fac040100000fac060000')
    kd_bad = pad_key_data(other_rsn + gtk_kde(bytes(16), 1, False))
    m3_rsn = build_eapol(base_flags, 16, rc2, anonce, aes_wrap(kek, kd_bad), kck=kck)
    s = d.rx(m3_rsn)
    cnt.ok(s['evt'] == FXE_FAIL and s['tk'] == 0, 'wrong RSN element in M3 is rejected')
    d.close()

    # Truncation: a frame shorter than the fixed header.
    d = arm_to_m3(exe, ctx)
    s = d.rx(good_m3[:80])
    cnt.ok(s['evt'] == FXE_FAIL, 'truncated frame is rejected')
    d.close()

    # Key data not a multiple of 8 (declared length 20, still MIC'd correctly).
    d = arm_to_m3(exe, ctx)
    bad_len = aes_wrap(kek, kd)[:20]
    m3_len = build_eapol(base_flags, 16, rc2, anonce, bad_len, kck=kck)
    s = d.rx(m3_len)
    cnt.ok(s['evt'] == FXE_FAIL and s['tk'] == 0, 'key data not a multiple of 8 is rejected')
    d.close()

    # Unwrap integrity failure: MIC is valid, but a ciphertext byte is wrong.
    d = arm_to_m3(exe, ctx)
    wrapped = bytearray(aes_wrap(kek, kd))
    wrapped[9] ^= 0x01
    m3_unw = build_eapol(base_flags, 16, rc2, anonce, bytes(wrapped), kck=kck)
    s = d.rx(m3_unw)
    cnt.ok(s['evt'] == FXE_FAIL and s['tk'] == 0, 'AES-unwrap integrity failure in M3 is rejected')
    d.close()

    # Retransmission and the retry limit: no M3 arrives, so M2 is resent up to
    # the limit, then the handshake fails.
    d = arm_to_m3(exe, ctx)
    resend = 0
    failed = False
    for _ in range(10):
        s = d.tick(1000)
        if s['evt'] == FXE_TX:
            resend += 1
        elif s['evt'] == FXE_FAIL:
            failed = True
            break
    cnt.ok(resend == 3 and failed, 'M2 is retransmitted 3 times then the handshake fails')
    d.close()


def mutants(exe, ctx, cnt):
    """Every single-byte flip of a valid message 3 must fail to install a key:
    the MIC covers the whole frame, so nothing short of the genuine frame is
    processed as message 3. (A flip that clears the MIC bit turns the frame
    into something parsed as a MIC-less message 1, which merely re-emits
    message 2 and installs nothing; that too is a refusal to trust the forged
    key data.)"""
    good_m3 = ctx[8]
    bad = []
    for i in range(len(good_m3)):
        for delta in (0x01, 0x80, 0xff):
            d = arm_to_m3(exe, ctx)
            mut = bytearray(good_m3)
            mut[i] ^= delta
            if bytes(mut) == good_m3:
                d.close()
                continue
            s = d.rx(bytes(mut))
            if s['tk'] != 0 or s['gtk'] != 0:
                bad.append((i, delta, s['evt']))
            d.close()
    cnt.ok(not bad, 'a mutated M3 installed a key at %r' % bad[:4])
    return len(good_m3) * 3


def dash_check(cnt):
    files = ['xnu/fuxi-wlan/fuxi_wlan_eapol.h', 'tools/test-fuxi-wlan-eapol.py', 'docs-lanes/wlan-eapol.md']
    for f in files:
        text = (ROOT / f).read_text()
        cnt.ok(chr(0x2013) not in text and chr(0x2014) not in text, 'en/em dash in ' + f)
    print('no en or em dash in the %d EAPOL files' % len(files))


def purity_check(cnt):
    h = HEADER.read_text()
    for bad in ('#include', 'malloc', 'kalloc', 'printf', 'calloc', 'free('):
        cnt.ok(bad not in h, 'header is impure: ' + bad)
    print('pure header: no includes, no allocation, no stdio')


def main():
    cnt = Count()
    with tempfile.TemporaryDirectory(prefix='fuxi-wlan-eapol-') as tmp:
        (Path(tmp) / 'fuxi_wlan_eapol.h').write_text(HEADER.read_text())
        src = Path(tmp) / 'harness.c'
        src.write_text(HARNESS)
        exe = Path(tmp) / 'eapol'
        subprocess.run(CC + ['-I', tmp, str(src), '-o', str(exe)], check=True)
        check_crosscheck(exe, cnt)
        ctx = full_handshake(exe, cnt)
        negatives(exe, ctx, cnt)
        nmut = mutants(exe, ctx, cnt)
    purity_check(cnt)
    dash_check(cnt)
    print('EAPOL host tests passed: %d assertions, %d M3 mutants refused' % (cnt.n, nmut))


if __name__ == '__main__':
    main()
