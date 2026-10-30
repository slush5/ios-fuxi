# P-WIFI: iOS Wi-Fi interface shim

Source-only, opt-in bring-up. Patch **0104-fuxi-wifi-ios.patch** publishes a
synthetic Wi-Fi `en0` to iOS 16.7 on Fuxi. The WCN7850 driver in patch 0099 is
unchanged: QMI setup does not yet provide Ethernet TX/RX, association, scanning
or key installation. This lane performs **no radio, PCIe, firmware or DMA access**.
Host tests and local arm64 syntax checks pass; no device result is claimed.

## Enablement and backend boundary

`fuxiwifiios=1` enables the shim; `fuxiwifiios=2` enables it with route U
(below); `fuxiwifiios=3` and `=4` enable the scan-only radio backends (K2,
below). Missing, zero or other values do nothing:
no interface, station thread or discovery services are created. Preserve all
other caller boot arguments. No `fuxiwlan` argument is required for the fake
backend. The existing USB Ethernet interface is `ue0`; it is not renamed.

The entry runs after `net_init_run()` in BSD initialization, before launchd.
`fuxi_wifi_ios.h` defines one immutable, boot-lifetime backend:

- `station`: power, scan request, cached results, connect (`join`), disconnect
  (`leave`), state snapshot, allowed channels and a locally administered MAC.
- `tx_frame`: borrowed Ethernet bytes, 14-1514 bytes without FCS; copy before
  returning. TX is serialized, but may overlap station/state calls.
- `start`: receives the `rx_frame` and `link_state` input callbacks. RX is
  copied before return; invoke both in thread context, never an interrupt.

Station requests are queued to one worker and answered asynchronously by
Apple80211 events. State snapshots and cached scans have separate locks.
Only init may attach the backend; replacement and detach are unsupported.
`start` must not emit events before attachment finishes. `join` returns only
after association and keys are ready; the worker then sends association/link
events. The backend changes the BSD carrier with `link_state`; unrequested
loss also calls `fuxi_wifi_sta_lost(errno)` after updating its state snapshot.
The shim bounds cached SSIDs, information elements and channel counts.

For a future `fuxi_wlan` adapter, select its backend at the init call, as K2's
radio backend does; readiness is the backend's own state (init runs before
launchd, long before a radio is up), not a reason to attach later. Supply Ethernet
encapsulation/decapsulation and a bounded worker for real RX. The ported
`fuxi_wifi_wpa.h` offers PMK derivation, four-way/group EAPOL processing and
key outputs. The adapter must generate fresh nonces, own EAPOL routing and
retransmission/timeouts, program pairwise/group firmware keys, and assert
carrier only after successful installation. Nothing binds QMI success to a
usable link today. The crypto library is compiled and tested but is not used
by the open synthetic BSS.

The fake backend reports **Fuxi Loopback**, an open BSS on channel 1, RSSI
−42 dBm. It honors SSID/channel scan filters, accepts only that BSS, rejects
WPA2 requests with `EOPNOTSUPP`, and supports disconnect/power changes.
Only Ethernet type `0x88b5` is echoed to RX after joining. IP, ARP and EAPOL
are dropped: no DHCP server, gateway, DNS, Internet or real authentication
is simulated. A Settings association checkmark/routability is consequently
**not** a success criterion. No passphrase/network file is read or retained
as a standing/fallback connection; wifid owns rejoin policy.

## iOS discovery and private ioctl ABI

The BSD interface uses the Ethernet ifnet KPI, Wi-Fi subfamily and 802.11
media reporting. Its discovery tree is a property-only `IO80211Controller`
with an `IONetworkInterface` subclass, `FuxiWLAN`. It supplies
`IO80211InterfaceRole=Infrastructure`, `IOInterfaceName=en0`, MAC, type/unit
and built-in properties. `newUserClient` returns unsupported so the framework
can fall back to `SIOCGA80211`/`SIOCSA80211`. This does not implement Skywalk,
IO80211Family's private vtables or a user client ABI.

`FuxiNetworkStack.cpp` publishes `IOResources/IONetworkStack` and its minimal
`IONetworkStackUserClient`. HackiOS found that configd's InterfaceNamer waits
about five minutes without it, blocking interface enumeration. The user
client opens and refuses naming commands because the ifnet is already named.
This must be validated against this root's configd; other drivers keeping the
registry busy can still delay quiet-state detection.

The existing [iOS 16.7 lane investigation](wlan-bt.md#3-what-ios-16-expects-iphone-x-userland)
records wifid's DT gate, interface discovery and ioctl usage. HackiOS's source
comments report private layouts reverse-read from **16.5** IO80211.framework.
No matching 16.7 private header or independently decoded 20H392 framework
layout was available in the inspected notes/headers. **All private payload
layouts below remain unverified for 16.7**; same-generation XNU alone proves
no compatibility. Static assertions and tests verify our encoding, not Apple's
16.7 consumer. No Apple binaries or extracted private headers are committed.

| Request / data | HackiOS 16.5 bytes / offsets retained | 16.7 evidence |
| --- | --- | --- |
| Outer request | 40; type 16, value 20, length 24, arm64 user pointer 32 | arm64 C layout/ioctl size checked; consumer unverified |
| GET / SET ioctl | `0xc02869c9` / `0x802869c8` | use recorded in lane notes; layout unverified |
| ASSOCIATE (20) | 592; SSID length `0x0c`, key `0x48` | unverified |
| SCAN_REQ (10) | 5456; channels `0x48`, flags `0x1308`, names `0x1318` | checked against 20H392 IO80211 (`__getScanData`), 2026-10-29 |
| SCAN_RESULT (11), CURRENT_NETWORK (103) | 1164; SSID length `0x60`, age `0x84`, IEs `0x8c` | checked against 20H392 (`__addScanResultToList`); 103 unverified |
| SUPPORTED_CHANNELS (27/254) | 4824 | 254 checked against 20H392 |
| CHANNELS_INFO (207) | 10016; number row `0x32e`, flags `0x12d0` | checked against 20H392 |
| Link event 4 | 32 after the 16-byte interface name; reason `0x14` | unverified |
| POWER (19) | 24 | unverified |
| CHANNEL (4), AUTH_TYPE (2) | 16 each | unverified |
| RSSI (16) | 52 | unverified |
| CARD_CAPABILITIES (12) | 13 bare bytes (no version prefix) | unverified |
| DRIVER/HARDWARE_VERSION (43/44) | 256 text bytes | unverified |
| SSID / BSSID (1/9) | raw SSID bytes / 6 bytes | unverified |
| STATE, OP_MODE, ASSOCIATE_RESULT (13/15/21) | outer `req_val` | unverified |

Known SET sizes are checked (scan/association require the complete known
layout). Unsupported types return `EOPNOTSUPP`; malformed lengths return
`EINVAL`; bad user buffers propagate `EFAULT`. SET requires a superuser
caller. GET copies at most the caller's buffer size. Logs contain request
types, lengths and status, never SSIDs, BSSIDs, passphrases or keys. The
shared `fuxi_wifi_apple80211.h` is the single ABI definition for kernel,
host fixtures and the optional probe. Record actual 16.7 request sizes and
fallback logs before broadening any layout; do not silently treat new bytes
as a known structure.

## Explicit mounted-root edit

The root tool is independent of the boot argument and is **off by default**:

```sh
python3 tools/enable-wifi-ios.py "$IOS_ROOT"                  # inspect only
python3 tools/enable-wifi-ios.py "$IOS_ROOT" --enable-wifid   # explicit edit
python3 tools/enable-wifi-ios.py "$IOS_ROOT" --restore-wifid  # undo job keys
```

It finds `/usr/sbin/wifid` in `System/Library/xpc/launchd.plist` and, if
present, `System/Library/LaunchDaemons/com.apple.wifid.plist`. It removes only
`_LimitLoadToDeviceTree` and sets `Disabled=false`. Use the existing
`launchd_unsecure_cache=1` boot option for the edited cache. This does not
create a `wlan` DT node, enable other jobs, change sandbox/entitlements or
modify the root staging defaults. It refuses symlinked paths, validates all
inputs before writing, preserves the edited files' owner/mode/inode, and
saves the original two keys beside each plist in a mode-0600
`.fuxi-wifi-ios-backup.plist`. Repeat enable preserves the first backup;
restore reinstates only those keys and preserves unrelated later edits.
Keep backups with the edited offline root until undo is no longer needed.
No root image was mounted or edited in this task.

## Maintainer device test plan (not executed)

1. First boot the newly built patch stack with `fuxiwifiios` omitted and the
   unedited root. Require no `FUXI WIFI IOS:` lines, no new `en0` or
   FuxiWLAN/IONetworkStack service, and the existing UI/`ue0` behavior.
2. Prepare a separate offline root with the explicit tool above. Build/sign
   `tools/fuxi-wifi-ios-test.c` with the normal iOS arm64 SDK and existing
   local signing/staging policy; stage it as `/usr/local/bin/fuxi-wifi-ios-test`.
   The source header gives the compile command. Do not replace any system
   daemon. Boot that root with the previous working command line plus
   `fuxiwifiios=1` and `launchd_unsecure_cache=1`, without enabling WCN7850.
3. Expected kernel startup lines (ordering of asynchronous events may vary):

   ```text
   FUXI WIFI IOS: enabled, synthetic backend; Apple80211 16.7 ABI unverified
   FUXI WIFI IOS: en0 attached on synthetic-loopback (a radio), link down
   ```

   Require an exec/start of wifid. Inspect the registry for the controller,
   FuxiWLAN and `IOResources/IONetworkStack`, and the service's `en0` and
   Infrastructure properties. Expected client-attempt line:

   ```text
   FuxiWLAN: a user client of type <type> is asked for: there is none
   ```

   Look for framework `Falling back to IOCTL path` if its logging is enabled.
   Require requests logged as `FUXI WIFI IOS: apple80211: wifid ...` with the
   table's sizes, and `MiniIONetworkStack: opened by configd; ...`. Observe
   InterfaceNamer reaching `*QUIET*` without the roughly five-minute
   `*TIMEOUT*` stall. A fallback failure or changed payload size is an ABI
   investigation result, not a passed shim test.
4. Run `/usr/local/bin/fuxi-wifi-ios-test --loopback` as root using an existing
   local test launch mechanism. It verifies the synthetic driver identity
   before sending SETs, powers on, scans, joins, sends a 60-byte experimental
   frame through BPF with outgoing capture disabled, checks RX, then queues
   disconnect. Expected probe lines:

   ```text
   P-WIFI TEST: scan PASS (synthetic BSS)
   P-WIFI TEST: connect PASS (no radio/DHCP)
   P-WIFI TEST: RX loopback PASS (outgoing BPF capture disabled)
   P-WIFI TEST: disconnect queued; PASS
   ```

   Corresponding kernel lines include:

   ```text
   FUXI WIFI IOS: fake scan complete
   FUXI WIFI IOS: fake connect result 0 (no radio)
   FUXI WIFI IOS: the link (synthetic-loopback) is up
   FUXI WIFI IOS: fake loopback frame 60 bytes
   FUXI WIFI IOS: the link (synthetic-loopback) is down
   ```

   `--loopback` needs the root's BPF device support; without it, run the probe
   without that option and report the RX test as untested. The queued
   disconnect is complete only when the kernel logs link down. No DHCP or
   routability is expected. Retain errors and ioctl lengths if a step fails.
5. Open Settings → Wi-Fi, toggle off/on and request a scan. Expect POWER SET
   (19), power events 0/1 and SCAN_REQ/SCAN_RESULT plus scan-done event 10.
   The synthetic BSS should appear if 16.7 discovery/ABI agrees. Attempting
   to join it must give association result 1, link-on and assoc event 9;
   IP configuration may remain pending. Confirm no panic, wifid respawn loop
   or configd five-minute enumeration delay. Do not enter a real password.
6. Restore the root job keys, omit `fuxiwifiios`, and repeat the baseline.
   Hardware driver validation remains a separate WLAN lane test.

## Route U: internet over the USB cable (fuxiwifiios=2, salami)

Route U keeps the synthetic station (scan and join of **Fuxi Loopback** are
exactly as above) and carries `en0`'s Ethernet frames over the USB cable that
already links the phone to the host: the loader's DWC3 ep0 monitor on cpu@100.
A bridge on the host (DHCP, ARP, DNS, UDP NAT, TCP termination with ordinary
sockets) answers them. Only the backend's `tx_frame` and `start` differ.

    iOS en0 -> fuxi_wifi_ios.c usbnet_tx -> TX ring --0x5e--> host bridge
    host bridge --0x5d--> RX ring -> RX thread -> fuxi_wifi_net_input -> iOS en0

Enable it with all three: a loader built with `FUXI_USBNET` (the default for
`FUXI_BOARD=salami`; `FUXI_USBNET=0` builds the old image, byte for byte, named
`-un0`), an ADT from `tools/mkadt-salami.py --wlan-node --usbnet` (adds the
16-byte /chosen `fuxi-usbnet` placeholder) and `fuxiwifiios=2` instead of `=1`
(same length; no other boot-arg). The host side is `tools/fuxi_usbnet_ring.py`:
`UsbnetTransport(ctrl, lock)` over pyusb `ctrl_transfer` (share `lock` with
the console loop), and `LoaderRingSim`/`KernelSim`/`sim_pair()` to run a
bridge end to end in WSL with no phone.

**Region.** One 0x24000-byte block inside the loader Image (`build.sh`
`usbnet.S`: zero bytes in the file, 16 KiB aligned, between `.data` and
`.bss`, before the payload; `build.sh` checks it against the Image). At the
handoff, after the ADT copy, the loader checks it is outside `FG_RAM_BASE` +
`FG_RAM_SIZE` and the GPU scratch (else it writes and publishes nothing,
`adt_status` 4: 0x5e keeps answering zeros, 0x5d frames are dropped), zeroes
it, writes line 0, writes `{u64 pa, u64 0x24000}` over
the ADT copy's placeholder when there is exactly one, and publishes it. XNU
never manages it; the kernel maps it with `ml_io_map_wcomb` (Normal
non-cacheable), the loader runs with the MMU off (Device), so no cache upkeep,
only a `dsb sy` before every index store and after every index load.

| Offset | Writer | Fields (u32 unless noted) |
| --- | --- | --- |
| 0x00 | loader, once | u64 "FXNETL01", version 1, size 0x24000, u64 self_pa, slot_size 2048, slots 32, rx_off 0x4000, tx_off 0x14000, adt_status (1 patched, 2 no placeholder, 3 several) |
| 0x40 | kernel | u64 "FXNETK01" (its opt-in), kstate (bit0 up, bit1 en0 link), rx_tail, tx_head, rx_frames, rx_bad, tx_frames, tx_full_drops, tx_size_drops (0x64) |
| 0x80 | loader | rx_head, host_tx_tail, out_ok, out_stall_full and out_stall_other (frames dropped: ring full, anything else), in_count, tail_bad (0x98) |
| 0x4000 | loader | RX ring (host to phone): 32 slots of 2048 |
| 0x14000 | kernel | TX ring (phone to host): 32 slots of 2048 |

Slot k of a ring is at its offset + (k % 32) * 2048: {u16 len, u16 flags 0,
u32 seq = k, u32 fnv1a32(frame), u32 0}, then the frame (14..1514 bytes, no
FCS) zero padded to 4. Counters run free (u32, wraparound). RX is full at
rx_head - rx_tail == 32, TX at tx_head - host_tx_tail == 32.

**Requests** (`shim/fuxi-probe/usbnet.h`, under `FUXI_USBNET`, outside the
salami pre-filter, no per-request log). A STALL makes the device vanish from a
WinUSB host until the cable is replugged (measured at home, SPEC amendment
A1), taking the console monitor with it, so only a malformed request that a
correct host never sends is STALLed:

- `0x5d` OUT (0x40), wValue `0x4e52`, wIndex = the low 16 bits of the host's
  TX consumed count, wLength 14..1514 = one frame. STALL only for another
  wValue or wLength (counted `out_stall_other` once published). Every
  well-formed request gets its data stage (the audio OUT residue rule), then
  the frame is kept only after the launch, with the region published, kmagic
  and kstate bit0 set and a free RX slot: slot, `dsb`, rx_head + 1,
  host_tx_tail, out_ok + 1. Otherwise it is dropped and counted
  (`out_stall_full` when the full ring was the only reason, else
  `out_stall_other`; nothing is counted before the region is published) and
  host_tx_tail is left alone.
- `0x5e` IN (0xc0), wValue `0x4e53`, same wIndex, wLength 192..4096. STALL
  only for another wValue or wLength. Before the launch or the handoff the
  reply is wLength zero bytes (magic 0: not ready) and nothing is touched.
  Otherwise it is always exactly wLength bytes (no zero-length packet): header
  lines 0..2, then whole TX slots from host_tx_tail below tx_head, each header
  + frame padded to 4, as many as fit; zero padding (a len of 0) or a slot with
  a bad len or seq ends the list.
- host_tx_tail is rebuilt from wIndex and tx_head (the host's count lies in
  [tx_head - 32, tx_head]); anything else keeps the old value (`tail_bad`).
- The host avoids drops: `UsbnetTransport` keeps (rx_head + frames sent since
  the last header) - rx_tail below 32, refreshes the header with one 192-byte
  0x5e when that looks full, and sends nothing until a header shows the loader
  magic and the kernel up. `send_frame` then returns False (busy, nothing
  sent); `stats['phone_drops']` counts what the phone dropped anyway.
- The loader copies frames in 64-bit Device accesses wherever the ep0 buffer
  and the slot are both 8-byte aligned, 32-bit ones (pad bytes masked) for the
  last word or two.

**Kernel.** `fuxiwifiios=2` reads /chosen `fuxi-usbnet` (16 bytes, not the
placeholder, size 0x24000, 16 KiB aligned, outside XNU RAM and the GPU
scratch), maps it, checks line 0 (magic, version, size, self_pa, ring layout),
writes kmagic and logs one line with pa and size; any handoff or header failure
logs one `FUXI WIFI IOS: usbnet: ...` line and keeps the synthetic loopback
(`ml_io_map_wcomb` has no soft failure: io_map uses `KMA_NOFAIL` and panics if
`pmap_map` fails; the 144 KiB fit its 8 MiB submap). TX writes a
slot under the backend's lock (full: `tx_full_drops`). One thread drains RX
every 1 ms while frames moved either way in the last second, else every 20 ms,
checks len, seq and fnv, copies the frame out and hands it to `en0`, then
consumes the slot (`rx_bad` for a bad one). kstate bit0 is set when that thread
runs, bit1 follows `en0`'s link (join, leave, power).

**Safety.** No request names an address: the region is the loader's own
linker symbol, checked at build time (inside the loaded Image, zero) and at
boot (aligned, outside XNU RAM and the GPU scratch); every slot index is
reduced mod 32 and every length bounded to 14..1514, so 0x5d writes only one RX
slot and line 2 of that region, and 0x5e only reads the region into the ep0
buffer (4096 bytes, inside the 16 KiB `dma_area`). No well-formed request
STALLs, so a busy or booting phone never knocks the monitor off the host.
Nothing is kept until the kernel opts in with kmagic and kstate bit0, a dropped
frame changes only line 2's counters, and a host count outside the window
only makes the kernel see fewer free slots, never overwrite unread ones. No
MMIO, rails, clocks, RPMh or GPIO: plain DRAM plus the ep0 path every monitor
request already uses. The kernel treats the RX ring as untrusted (len, seq,
fnv) and frames enter iOS through the normal `en0` input path, as from any
Ethernet; what they can reach is the host bridge's decision, as with USB
tethering. Requests 0x4f, 0x55 and 0x57 are unchanged.

**Throughput (estimate, not measured).** ep0 runs 64-byte packets; a 1514-byte
frame is 24 of them, about 50 us on a high-speed wire. The host's synchronous
control transfers (WinUSB through pyusb) cost roughly 0.25 to 1 ms each, so
about 1000 to 4000 requests per second shared by both directions and the
console loop: one frame per 0x5d, and per 4096-byte 0x5e two full-size frames
or up to 32 small ones. Expect a few Mbit/s each way, up to roughly 10 to 20
Mbit/s in the best case, and 1 to 3 ms more round-trip time (host poll plus the
kernel's 1 ms poll): enough for DNS, HTTPS, messaging and App Store metadata,
not for video. A full-speed (12 Mbit/s) link caps it lower.

Host validation: `CC=cc python3 -B tools/test-fuxi-usbnet.py` (layout and
switches; usbnet.h and fuxi_usbnet_ring.h compiled unchanged and checked byte
for byte against the Python models over one scenario with every STALL and drop
rule, bound and wraparound, and no STALL or drop for a host that keeps fewer
than 32 frames in flight; probe.c's own ep0 glue; the ADT patch; the kernel
backend on the wifi-ios model; the transport, including zeros before the
launch and a phone reboot under a running bridge; 57 mutants of the rules and
bounds), and
`python3 tools/fuxi_usbnet_ring.py --selftest`.

## Radio backends: real scan results, scan only (fuxiwifiios=3/4, K2)

Plan: `op11-tools/salami-port/wlan/r1/overnight_plan.md` 3.2 and `ios_scan_plan.md`.
`bsd/dev/fuxi_wifi_radio.c` (`bsd/sys/fuxi_wifi_radio.h`) is the handoff between
a producer that scans and the iOS side. It owns every shared byte and one leaf
lock; requests and records are copied, no pointer crosses, and the iOS side
never sees MMIO or DMA. The tables (about 140 KB) are allocated only with
`fuxiwifiios=3/4`.

- `fuxiwifiios=3`: the producer is the radio lane (patch 0099). Since C3,
  `fuxi_wlan_init` reports on every boot, before launchd: STARTING just before
  its thread starts, or STOPPED with why not (`0x10000` no `fuxiwlan`,
  `0x20001`/`0x20002` salami refusals, `0x30000` no thread); its thread reports
  STOPPED `0x4XXXX` after its containment, on every exit (XXXX = the failing
  step << 8 | reason, 0 when every step passed). With `fuxiwlanqrtr=8` the
  thread's W25 service says READY with its channel list and serves the scans
  (docs-lanes/wlan-w20.md, C3), for 30 minutes, then contains and stops. While
  STARTING (minutes, from power-up through WMI READY) a scan is over at once
  with ENETDOWN and Wi-Fi stays on with an empty list. A kernel whose radio lane
  never reports (no 0099 hooks) still gets Wi-Fi off when iOS first asks for a
  scan or the state, or 30 s after attach: `radio: no radio lane reported before
  iOS first asked (code 0x50001); ...` (or `within 30 s of attach`, `0x50002`).
- `fuxiwifiios=4`: the canned producer, no radio: four networks with locally
  administered BSSIDs `02:46:55:58:4a:01..04`, one channel every 40 ms: "Fuxi
  Canned WPA2" (channel 6 heard on 5, the DS correction, RSN, -48 dBm), "Fuxi
  Canned Open" (36, -61), a hidden one (11, RSN, -70) and "Fuxi Canned DFS" (52,
  radar, RSN, -74). The radio lane's calls are ignored.
- Channels: 38, all passive, no 6 GHz: 1-13, 36-48, 52-64 and 100-144 (radar),
  149-165. Every scan is passive whatever wifid asks, 150 ms a channel (50-200
  if asked); the iOS side waits 1000 + n * dwell * 3/2 ms, at most 15 s.
- Results: SSID, BSSID, channel (the DS or HT operation channel), RSSI, noise
  (-95 dBm when the radio has none) and SNR, beacon interval, capability, and
  the elements kept whole within wifid's room (0 or over 1024 counts as 1024),
  the name, RSN and WPA first, then rates, channel, country, HT/VHT/HE; at most
  64 networks, the strongest. GET 43/44 name the backend.
- A join is refused at once without any radio access (`EOPNOTSUPP`:
  ASSOCIATE_RESULT 2 and event 9, never "wrong password"); the link stays down
  and no frame passes en0.
- Wi-Fi goes off for the boot when the producer stops, when 3 scans in a row
  time out (`0x60003`), when there is no producer, or when `ready()` names no
  listed channel (`0x7000n`, n channels named: MHz expected, no 6 GHz); SET 19
  "on" is answered and Wi-Fi stays off. A radio lane serving scans (C3) then
  ends its service at its next idle poll (`W25 complete: the iOS side stopped`)
  and contains; the user's own Wi-Fi switch only cancels and ends nothing.
- Producer API, for the radio lane: `starting`, `ready(freqs)`, `take`,
  `cancelled(seq)`, `wanted()` (C3: false once STOPPED, so the radio lane's
  service ends when this side has given up instead of idling to its lifetime;
  true with `=4`), `frame(seq, frame, length, MHz, dBm, noise)` or
  `bss(seq, record)`, `done(seq, status)`, `stopped(code)`. It never sleeps and
  never calls the iOS side; before the consumer exists only the state word is
  stored. Stop codes are class << 16 | detail: 1 not requested, 2 refused,
  3 no thread, 4 exited (detail: fw_run's step << 8 | reason), 5 no producer,
  6 gave up, 7 no usable channel.
- Logs carry counts, bands, flags and errno; never an SSID, BSSID or element
  byte. M1 (`=4`) lines: `enabled, canned scan backend (no radio)`, `radio: canned
  producer ready, 38 channels (13 at 2.4 GHz, 25 at 5 GHz), passive only`,
  `radio: scan 1: 38 channels passive, 4 heard (2 at 2.4 GHz, 2 at 5 GHz, 3 RSN,
  1 hidden), <T> ms, status 0`, `apple80211: first SCAN_RESULT room <N> bytes
  (...)`, four `gets 11 ... answered` and one `error 5`, and on a join
  `radio: join refused (scan-only backend), no radio access`.
- `fuxiwifiios=1` and `=2` are unchanged: the same lines and the same GET 11
  bytes (no noise figure, an active channel `0x8a`, a room of 0 still gives no
  elements).
- M2 (C3: `fuxiwifiios=3` with `fuxiwlan=28 fuxiwlanqrtr=8` and ADT
  `fuxiwlansalami=2`, `--wlan-node`): `radio: radio producer ready, 38 channels
  (13 at 2.4 GHz, 25 at 5 GHz), passive only`, then per scan `radio: scan K: 38
  channels passive, R heard (...), T ms, status 0` (T about 5.7 s for 38
  channels), and at the end `radio: the radio lane stopped (code 0x40000); Wi-Fi
  reported off, no retry this boot`. tools/test-fuxi-wlan-ce.py links this
  unit's fuxi_wifi_radio.c with the radio lane model end to end (a request of
  {2437, 5180} out as one passive START_SCAN of those channels, the beaconed
  networks back through `fuxi_wifi_radio_results` intact).
- M2c (C3 rebased onto the C2c..C2e RX rungs): M2 with the rung's w20 bits in
  ADT /defaults (`--wlan-defaults fuxiwlanw20=<n>`, 15106 for r1c2e) and
  `fuxiwlansalami=2` on the cmdline instead of `fuxicpuload=10` (the ADT has no
  room for both numbers; owner decision 4, recommended yes, pending owner;
  docs-lanes/wlan-w20.md, C3, Kit (M2c)). This side's lines are those of M2;
  only the radio lane adds its W26..W28 lines around the first scan.
- `tools/fuxi-wifi-ios-test.c --scan-only` lists what one scan heard (band,
  channel, RSSI, privacy, RSN, the name's length) and refuses any join on these
  backends. `scan PASS` (exit 0) only when a network was heard; none is
  `scan EMPTY` (exit 1); a scan that never began (radio not ready, Wi-Fi off)
  or never ended is `FAIL`.

## Provenance and exclusions

HackiOS kernel checkout `8e61187098f2c26c5daee8f9f72b6416aa64b9ab`,
**APSL-2.0**:

| Upstream source | Canonical Fuxi source |
| --- | --- |
| `xnu/files/bsd/dev/redwood_net.c`, `bsd/sys/redwood_net.h` (same `xnu/files/` prefix) | `fuxi_wifi_net.c/.h` |
| `xnu/files/bsd/dev/redwood_sta.c`, `bsd/sys/redwood_sta.h` (same prefix) | `fuxi_wifi_sta.c/.h` |
| `xnu/files/bsd/dev/redwood_apple80211.c` | `fuxi_wifi_apple80211.c/.h` |
| `xnu/files/iokit/Kernel/RedwoodWLAN.cpp` | `FuxiWLAN.cpp` |
| `xnu/files/iokit/Kernel/MiniIONetworkStack.cpp` | `FuxiNetworkStack.cpp` |
| `wifi/lx/src/lx_wpa.c/.h` | `fuxi_wifi_wpa.c/.h` |

Ported files retain their original commentary plus provenance/adaptation
headers. `LICENSE.HackiOS` retains the upstream licence and is also carried
inside 0104. Fuxi changes include names/gating, backend lifetime, removal of
QEMU/poll/packet-name tracing and credential-file rejoin policy, request bounds,
root SET checks, safe mbuf header adjustment, open-auth reporting and WPA reply
capacity/completed-handshake replay checks.

HackiOS userland checkout `906425b75e02b4f99eb5d7bb1ef7c9583472d772`,
**MIT**, was read for `userland/build.sh` wifid job notes and
`userland/src/net_test.m` diagnostic/16.5 association layout. No implementation
was copied from it. The root tool and diagnostic probe are original Fuxi code.
No UniXNU code was needed. Both reference projects were read only; none of
their tools, scripts, imports or builds was executed.

Excluded: Linux/ath11k compatibility layer and GPL sources; WCN6750/SM7325
hardware, virtio, firmware, root credential files, Apple binaries, private
framework patches, Skywalk, real radio scanning/association/key installation,
DHCP emulation, Mesa, metal2vulkan and NativeMetal. They are unnecessary for
this iOS shim or belong to later hardware/userland work. WPA arithmetic alone
does not supply retransmissions or a working radio supplicant.

## Host validation

```sh
python3 tools/sync-fuxi-wifi-ios.py --check
python3 tools/test-fuxi-wifi-ios.py
CC=cc python3 -B tools/test-fuxi-wifi-radio.py
CC=cc python3 -B tools/test-fuxi-usbnet.py
python3 tools/check-fuxi-wifi-ios.py "$XNU_BUILDTREE"
```

`test-fuxi-wifi-radio.py` (K2) runs the handoff and the `fuxiwifiios=3/4`
backends on the same model: the frame parser and element fit (every drop
reason, 20000 fuzzed frames), the table, a fake radio lane (passive forced,
GET 11 field by field, timeouts, a stop or a power-off mid-scan, no producer,
READY while the backstop re-arms, a `ready()` with no usable channel), the
canned producer, a default boot that allocates nothing, static rules for the
leaf lock and the producer (and that patch 0099's copy of the producer API,
its request struct and stop codes equals sys/fuxi_wifi_radio.h), and 17
mutants. The model's radio thread call
follows XNU's ONCE rule: entered while it runs, it runs once more, when the
last entry says.

The tests compile production C with ASan/UBSan and a pthread station adapter;
cover missing/invalid/enabled gates, one-time attachment, malformed requests,
privilege/user-copy failures, synthetic scan/connect/disconnect/power, event
class/name/payload, truncation limits, Ethernet TX/RX ownership and mbuf header
adjustment, allocation/copy failures and fake frame filtering. An independent
Python hashlib/HMAC/AES-wrap peer checks PMK, PTK, message 1/3, group rekey,
MIC failure, replay and truncated EAPOL frames. Tests require `cryptography`.
Mounted-root fixtures cover dry-run, enable/idempotence, restore preserving
unrelated edits, malformed input and symlink refusal. The probe's portable
ioctl portion is also host syntax-checked; its Apple BPF portion needs an iOS
SDK build/device validation.

All seven kernel C/C++ units passed a local `arm64-apple-macos13` syntax check
using existing xnu-8796 build headers (older SWEET artifact selected with
`--config DEVELOPMENT_ARM64_SWEET --header-machine SWEET`). Apple type-signature
builtins are substituted as in `xnu-cc.py`; header/macro initializer warnings
remain. This is not a linked kernel or an iOS SDK build. Integration deltas
through 0104 were applied on the available local base; already present
0083/0084 deltas were reverse-checked. The sync check covers byte equality,
hunk counts and application of every generated source/integration hunk.
Existing `tools/test-fuxi-h1.py` and `tools/test-fuxi-wlan-w14.py` also pass.
No phone tools, CI/workflow, push or firmware/blob copy was used.
