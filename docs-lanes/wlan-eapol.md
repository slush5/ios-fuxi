# WLAN lane: the EAPOL supplicant (RSN 4-way and group-key handshakes)

## Why this lives in the kernel

On the WCN7850 the firmware does not run the supplicant. On Linux, ath12k
passes EAPOL-Key frames up to `wpa_supplicant` in user space, which owns the
PMK, derives the keys, and pushes the pairwise and group keys back down. On a
stock phone the radio's own firmware hides all of this. Our port has neither:
wifid hands the kernel an `ASSOCIATE` carrying a 32-byte PMK (key kind 6) when
the user joins a WPA2 network, the firmware will relay EAPOL-Key frames to and
from the AP, and nothing in between runs the four-way handshake. So the kernel
has to be its own supplicant.

`xnu/fuxi-wlan/fuxi_wlan_eapol.h` is that supplicant, as a pure,
allocation-free header in the same shape as `fuxi_wlan_ce.h`: byte-wise
get/put, no packed structs, `memcpy`/`memset`/`memcmp` only, every off-the-air
length bounded by the frame it arrived in before it is used, and no entropy
source of its own (the SNonce comes from a caller-supplied random function).
It compiles on the host with nothing but the fixed-width integer types, and it
compiles in the kernel against the same glue that `fuxi_wlan.c` already
provides for the CE lane.

## What it contains

The file is built bottom-up so the protocol layer never reaches outside it.

1. **Crypto primitives**, written from the public specifications:
   - SHA-1 (FIPS 180-4), streaming, so HMAC and the PRF never need a scratch
     concatenation buffer.
   - HMAC-SHA-1 (RFC 2104), with a vector form that takes several message
     segments.
   - AES-128 block encrypt and decrypt (FIPS 197), byte-wise, column-major
     state, only the two S-boxes and Rcon as tables.
   - AES key wrap and unwrap (RFC 3394). The supplicant only ever unwraps
     (message 3 and group message 1 carry wrapped key data); wrap is present
     so the host test can build the authenticator side with the same code.

2. **Key hierarchy** (IEEE Std 802.11-2020, 12.7.1):
   - `fxprf`: the 802.11 PRF, `HMAC-SHA-1(K, A || 0 || B || i)` concatenated
     and truncated. The label `A` is passed null-terminated, so the single 0
     octet 12.7.1.2 appends is the label's own terminator.
   - `fxeapol_ptk`: PTK = PRF-384(PMK, "Pairwise key expansion",
     min(AA,SPA) || max(AA,SPA) || min(ANonce,SNonce) || max(ANonce,SNonce)),
     split as KCK[0:16] | KEK[16:32] | TK[32:48] for CCMP.

3. **EAPOL-Key frame** (802.1X-2020 header, 802.11-2020 12.7.2 body) as byte
   offsets, version 2 only (HMAC-SHA-1-128 MIC, AES key wrap), with the
   key-information bit masks and the GTK KDE / RSN element layout.

4. **The state machine** (12.7.6 and 12.7.7), described below.

## The state machine

The supplicant is purely reactive. The authenticator drives the exchange; we
answer and, on a timeout, retransmit the last answer a bounded number of times
before failing the handshake, so a lost message does not wedge the lane.

```
                       +-----------+
   init (PMK, SNonce)  |   INIT    |
   ------------------> |           |
                       +-----------+
                             | rx message 1 (pairwise, ACK, no MIC)
                             |   derive PTK; build message 2 (SNonce + our
                             |   RSN element, MIC under the new KCK)
                             v
                       +-----------+     tick timeout, retries < limit:
                       | WAIT_M3   | --> resend message 2
                       |           |     tick timeout, retries == limit: FAIL
                       +-----------+
                             | rx message 3 (pairwise, ACK, MIC, secure,
                             |   encrypted key data)
                             |   verify MIC, ANonce == message 1, replay newer;
                             |   unwrap key data; RSN element == beacon;
                             |   take GTK KDE; build message 4; install TK+GTK
                             v
                       +-----------+
                       | PTK_DONE  | <--------------------+
                       |           |                      | rx group message 1
                       +-----------+ ---------------------+   (group, ACK, MIC,
                             |         build group message 2;   secure, encrypted)
                             |         install the new GTK
                             | any MIC failure, bad RSN element, unwrap
                             | integrity failure, or malformed frame
                             v
                       +-----------+
                       |  FAILED   |
                       +-----------+
```

`fxeapol_input` returns `FXE_TX` (a frame to send is in the caller's buffer),
`FXE_NONE`, `FXE_DROP` (a replay, or a frame not expected in the current
state), or `FXE_FAIL` (a protocol violation; the handshake is aborted). Key
installation is signalled through `st->tk_installed` and `st->gtk_installed`,
with the material in `st->ptk` (TK = `ptk[32:48]`) and `st->gtk`.
`fxeapol_tick` advances the retransmit timer.

Validation common to every received frame lives in `fxeapol_check`: the
802.1X header and declared lengths, the descriptor type and version, and,
when the MIC bit is set, the HMAC-SHA-1-128 MIC over the whole PDU with the
MIC field zeroed. Message 3 and group message 1 additionally require a replay
counter strictly newer than the last accepted one, a key-data length that is
a non-empty multiple of 8, a successful AES unwrap (the recovered A must be
the RFC 3394 default IV), and, for message 3, an RSN element in the key data
byte-for-byte equal to the one the beacon advertised.

## How it plugs into a radio lane

The radio lane owns the PMK, the MAC addresses, the RSN elements and the
transport; the supplicant owns only the handshake.

1. On `ASSOCIATE`, the lane calls `fxeapol_init` with the PMK (key kind 6),
   our MAC (the en0 address the wifi-ios side assigns, the same locally
   administered address the scan lane uses), the AP MAC, the RSN element we
   put in the assoc request, the RSN element from the beacon, and a random
   function. `fxeapol_init` draws the SNonce immediately.
2. For every EAPOL-Key frame the firmware relays up, the lane calls
   `fxeapol_input`. On `FXE_TX` it sends the returned frame back down the same
   management path. On `st->tk_installed` / `st->gtk_installed` it programs the
   keys through the firmware's key-install command (pairwise TK keyed to the
   AP, GTK at `st->gtk_id`).
3. On its periodic poll the lane calls `fxeapol_tick` with the elapsed time;
   `FXE_TX` means retransmit, `FXE_FAIL` means tear the association down.

The header allocates nothing and keeps no global state: one `struct fxeapol`
per association, owned by the lane. All buffers are fixed and caller-owned.

## Provenance

The primitives are written from the public specifications, not copied from any
implementation:

- SHA-1 from FIPS 180-4.
- HMAC from RFC 2104.
- AES-128 from FIPS 197.
- AES key wrap and unwrap from RFC 3394.
- The PRF, the key hierarchy, the EAPOL-Key frame, the KDEs and the handshake
  state machine from IEEE Std 802.11-2020, clause 12.7 and Annex J.

`hostap`/`wpa_supplicant` (BSD-licensed) was read to cross-check field layout
and the MIC / key-wrap wiring, but no function, table or line of it is
reproduced here, and no GPL source was consulted. The header carries no
third-party code and imposes no third-party license terms; it is placed in the
public domain by its author.

## Testing

`tools/test-fuxi-wlan-eapol.py` compiles a C harness that includes the header
with ASan/UBSan and `-Werror` (the way `tools/test-fuxi-wlan-ce.py` builds its
model) and checks it two ways:

- The harness runs the published standard vectors against its own from-scratch
  crypto: FIPS 180-4 SHA-1, RFC 2202 HMAC-SHA-1, FIPS 197 AES-128 (encrypt and
  decrypt), RFC 3394 key wrap and unwrap including an integrity-failure case,
  and the Annex J PRF and PTK construction. Each is also recomputed in Python,
  independently, from `hashlib` and a pure-Python AES / key wrap, and the two
  are compared, so the C primitives are pinned both to the printed constants
  and to a second implementation.
- A `drive` command loop lets the test act as the authenticator: it builds
  message 1, message 3 and group message 1, hands them to the supplicant, and
  checks the message 2 / message 4 / group message 2 that come back bit for
  bit, including the MIC, the installed PTK and GTK, the replay counter and the
  install flags. On top of the clean run it drives the negatives the standard
  requires a supplicant to reject (bad MIC, replay, the wrong RSN element in
  message 3, a truncated frame, key data that is not a multiple of 8, an
  AES-unwrap integrity failure), the retransmit-and-give-up path, and a
  mutation sweep in which every single-byte flip of a valid message 3 must
  fail to install a key.

Run it with `CC=cc python3 -B tools/test-fuxi-wlan-eapol.py`. No device, no
network.
