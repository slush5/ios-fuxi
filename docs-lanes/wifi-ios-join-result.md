# Wi-Fi iOS: the join result study (silent refusal, pre-join stall)

Study lane on top of P-WIFI (docs-lanes/wifi-ios.md, patch 0104). Code, host tests
and this note only; no device result is claimed here. Two observations from the
phone with the canned scan backend (`fuxiwifiios=4`):

1. A refused join fails **silently**: no "Unable to join the network" alert,
   the row just returns.
2. Tapping a locked network shows the password sheet only after **a few
   seconds** of stall.

Marking: **[I]** is a guess about wifid, IO80211.framework or Settings internals
(not read from any Apple binary; only the public Apple80211 headers, public
behaviour and our own kernel log). Everything else is read from our own code
(`xnu/fuxi-wifi-ios/`) or from the host traces of `tools/test-fuxi-wifi-radio.py`
(T10).

## 1. The phone log, read against the code

Per attempt (canned backend):

```
sets 20 (ASSOCIATE) ... 592 bytes (name 16 bytes, any AP, auth 0x1/0x8, key 32 bytes kind 6): answered
radio: join refused (scan-only backend), no radio access
the station says: a join is over, 102
gets 21 (ASSOCIATE_RESULT) value 2 answered
gets 103 (CURRENT_NETWORK) error 6
sets 22 (DISASSOCIATE)
```

- `102` is Darwin's EOPNOTSUPP, returned by `rscan_join` (fuxi_wifi_ios.c).
- `error 6` is ENXIO: "not joined" (`get()`, second switch).
- `kind 6` with 32 bytes is `CIPHER_PMK`: on 20H392 wifid hands over the PMK, not
  the passphrase as typed (`set_join` accepts both; the comment at the top of
  fuxi_wifi_apple80211.c that says the framework never derives the key is not
  what the phone did). This matters for a future real join, not for this study.
- wifid **did** process the failure: right after event 9 it reads GET 21 (2),
  checks GET 103 (not joined) and leaves (SET 22). Whatever drops the alert sits
  after that, in wifid or Settings [I].

## 2. Every join-related path in the shim

All in `fuxi_wifi_apple80211.c` unless named. The station thread
(`fuxi_wifi_sta.c`) runs one queued request at a time and reports by
`fuxi_wifi_apple80211_event`.

### SET 20 ASSOCIATE (`set_join`)

1. `req_len` must be 592 (`sizeof(struct apple80211_assoc)`), else EINVAL.
2. Copy in; log lengths and kinds only (never name, BSSID or key).
3. Name length 1..32, else EINVAL. Key: none (open), kind 10 with 8..63 bytes
   (passphrase), or kind 6 with 32 bytes (PMK), else EINVAL.
4. `join_result = 0` (running), then `fuxi_wifi_sta_ask_join`: ENXIO without a
   station, ENETDOWN when Wi-Fi is off, else 0 and the join is queued (a second
   join replaces one not yet begun).
5. Answered at once. The outcome is asynchronous: station thread calls the
   backend's `join`, then event JOIN_DONE(status), and on status 0 also
   LINK(1).

### Join over (event `FUXI_WIFI_STA_JOIN_DONE`)

`join_result` = 1 if status 0; 21 if EAUTH; 9 if ETIMEDOUT; 2 for any other errno
(Apple80211 result numbers: 1 success, 2 unspecified failure, 9 timeout, 21
invalid PMK). Then **event 9** (APPLE80211_M_ASSOC_DONE) is posted with **no
data** (only the 16-byte interface name every event carries).

### GET 21 ASSOCIATE_RESULT

`req_val = join_result`: 0 while running (or never asked), else the number above.
Never an error.

### SET 22 DISASSOCIATE

`fuxi_wifi_sta_ask_leave`: ENXIO without a station, else 0 and a leave is
queued (it cancels a join not yet begun). The station thread calls the
backend's `leave` and **always** posts LINK(0), joined or not.

### GET 103 CURRENT_NETWORK

ENXIO unless the station's phase is JOINED; then the joined BSS in the 1164-byte
scan-result form (`give_network`).

### Link and power events

| Event | Code | When | Data |
|---|---|---|---|
| POWER_CHANGED | 1 | attach (on), SET 19 done, radio stopped by itself (off) | none |
| SSID_CHANGED, BSSID_CHANGED, LINK_CHANGED | 2, 3, 4 | every LINK: after a join that succeeded (1), after any SET 22 (0), power off while joined (0), lost (an errno) | 4 carries 32 bytes: `down` = status != 1, `involuntary` = status is an errno, `reason` 4 if involuntary else 0, `rssi` of the joined BSS when up |
| ASSOC_DONE | 9 | after every join the station ran | none |
| SCAN_DONE | 10 | after every scan | none |

### The pre-join GETs

| GET | Not joined | Joined |
|---|---|---|
| 16 RSSI | ENXIO (6) | 52 bytes, the BSS's dBm |
| 4 CHANNEL | ENXIO | version + channel (flags: active, band, 20 MHz) |
| 2 AUTH_TYPE | ENXIO | lower 1 (open system), upper 8 (WPA2 PSK) or 0 |
| 9 BSSID | ENXIO | 6 bytes |
| 1 SSID | ENXIO | the name, its length in `req_len` |
| 8 RATE | ENXIO | best legacy rate of the BSS |
| 17 NOISE | EOPNOTSUPP (on purpose) | EOPNOTSUPP |
| 57 MCS | EOPNOTSUPP (on purpose) | EOPNOTSUPP |
| 13 STATE | 0 (or 1 while scanning) | 4 (RUN) |
| 14 PHY_MODE | answered, `active` AUTO | answered, the mode in use |

Both ENXIO and EOPNOTSUPP reach wifid as -3903 (comment in
fuxi_wifi_apple80211.c), which it only logs.

## 3. What each backend returns today

| Path | `=1` synthetic loopback | `=3` radio | `=4` canned |
|---|---|---|---|
| SET 20 | 0 (queued) | 0 | 0 |
| backend `join` | open + "Fuxi Loopback" + any or its BSSID: 0, en0 link up; WPA2: EOPNOTSUPP; other name/BSSID: ENOENT | EOPNOTSUPP, "join refused (scan-only backend)" | same as `=3` |
| join-over status | 0 / 102 / 2 | 102 | 102 |
| GET 21 | 1 / 2 / 2 | 2 | 2 |
| events after the join | 9, then 2, 3, 4 (up) on success; 9 alone on failure | 9 alone | 9 alone |
| GET 103 | the loopback BSS when joined, else ENXIO | ENXIO always | ENXIO always |
| SET 22 | 0; link down; events 2, 3, 4 (down, voluntary) | 0; events 2, 3, 4 (down, voluntary, though nothing was up) | same as `=3` |
| pre-join 16/4/2/9/1/8 | ENXIO | ENXIO | ENXIO |
| pre-join 17/57 | EOPNOTSUPP | EOPNOTSUPP | EOPNOTSUPP |
| GET 13 | 0, 4 when joined | 0 / 1 scanning / 0 off | same as `=3` |

`=3` and `=4` share `rscan_*` in fuxi_wifi_ios.c; only the scan producer differs.

## 4. Success (`=1`) and refusal (`=3/=4`), event by event

From the T10 trace (`trace1`, `trace4`), which drives the production C on the
host model; the order is the order the kernel posts.

**`=1`, open join that the phone completed:**

1. SET 20 answered (`authentication 0/0, a key of 0 bytes`).
2. Station thread: `fake_join` returns 0, sets en0's link up, says `fake connect result 0`.
3. `the station says: a join is over, 0` -> `join_result = 1` -> **event 9** (no data).
4. `the station says: the link, 1` -> **events 2, 3, 4** (4: down 0, rssi -42).
5. wifid: GET 21 = **1**; GET 103 answered (the loopback BSS); GET 13 = 4;
   then the IP side.

**`=3/=4`, refused join:**

1. SET 20 answered (`authentication 0x1/0x8`, a key).
2. Station thread: `rscan_join` returns EOPNOTSUPP, says `join refused (scan-only backend)`.
3. `the station says: a join is over, 102` -> `join_result = 2` -> **event 9** (no data).
4. No link event (nothing came up, nothing went down).
5. wifid: GET 21 = **2**; GET 103 = ENXIO; SET 22.
6. SET 22 -> `the link, 0` -> **events 2, 3, 4** (4: down 1, involuntary 0,
   reason 0, rssi 0): "left because you asked".

The two sequences differ only in the status, GET 21 and the link events; the
failure is delivered on the same path and in the same shape as the success
that worked.

## 5. Why no alert: hypotheses, ranked

1. **Result 2 is a failure class wifid does not show [I].** Evidence: wifid
   clearly consumes the failure (GET 21, GET 103, SET 22 follow event 9), so the
   delivery works; what remains is how 2 ("unspecified") is classified. On a real
   iPhone, "Unable to join" is the common outcome of a timeout or an AP that
   rejects association, and a wrong password gives "Incorrect password" [I]. Our
   code sends 9 only for ETIMEDOUT and 21 only for EAUTH, so a refused join can
   never look like either. Candidates C1, C6.
2. **The only link event says the user left [I].** After the failure the sole
   LINK_CHANGED is the one SET 22 causes, `involuntary 0, reason 0`. If wifid
   ties the alert to an involuntary link-down (a real driver usually has one when
   an association fails [I]), it sees a clean user leave instead. Candidates
   C2, C3.
3. **Event 9 has no payload [I].** The framework handles event 9 and then asks
   GET 21 (it did so on success too, so an empty event 9 is not fatal). If its
   user-facing error is built from a status carried in the event, an empty one
   may default to a code without alert text. Candidate C4.
4. **Synchronous error path [I].** An ioctl error on SET 20 may be surfaced to
   Settings differently from an asynchronous result. Counter-evidence: the shim
   notes that EOPNOTSUPP/ENXIO reach wifid as -3903, which it only logs.
   Candidate C5.
5. **Too fast [I].** The canned refusal completes in microseconds, possibly
   before Settings has armed its observer for this join. Candidate C7 (no result
   at all, wifid's own timeout decides). A delayed event 9 is not implemented
   (it would need a sleep on the station thread); it is the next candidate if C7
   alone shows an alert.
6. **Other requests [I]**: an ASSOCIATION_STATUS (50) or a reason/status code
   wifid reads after the failure. The phone log should show any new request type
   in the summary (`get 50 ...`); none was reported so far.

### Candidate table

C0 is today. The last three columns are for a **real iPhone X** capture of a
comparable situation (wifid's log lines around the failure, the association
result it logs, and whether the alert appeared), and the first data columns
are what salami shows with the knob; fill both in.

| ID | `fuxiwifijoin` | SET 20 | Events after the failure (order) | GET 21 | Comparable real situation [I] | salami: alert? | iPhone X: wifid log lines | iPhone X: result | iPhone X: alert? |
|---|---|---|---|---|---|---|---|---|---|
| C0 | 0 (default) | 0 | 9 | 2 | AP refuses for an unspecified reason | no (observed) | | | |
| C1 | 1 | 0 | 9 | 9 | AP out of range after the scan | | | | |
| C2 | 2 | 0 | 9, then 2, 3, 4 (down, involuntary, reason 4) | 2 | AP rejects the association | | | | |
| C3 | 3 | 0 | 2, 3, 4 (down, involuntary, reason 4), then 9 | 2 | AP deauthenticates during the join | | | | |
| C4 | 4 | 0 | 9 carrying 4 bytes: 2 | 2 | n/a (shape test) | | | | |
| C5 | 5 | EOPNOTSUPP (radio backends only) | none | 2 | driver refuses the request | | | | |
| C6 | 6 | 0 | 9 | 21 | wrong password (control: blames the password) | | | | |
| C7 | 7 | 0 | none; GET 21 stays 0 | 0 | AP silent, no answer at all | | | | |

## 6. The knobs (default off)

Both are boot-args read in `fuxi_wifi_ios_init` only when `fuxiwifiios` is
1..4, and handed to `fuxi_wifi_apple80211_study`. With both absent or 0 nothing
changes and nothing is said: T10 proves every request, value, byte, event and log
line of a scripted session is identical to the sources before the study.

**`fuxiwifijoin=N`**, how a join that **failed** is told (a join that succeeded
is never touched):

| N | Effect |
|---|---|
| 0 | as today |
| 1..7 | candidate C1..C7 for every failed join |
| 8 | C1, C2, ... C7, C1, ... one per SET 20, so all seven are seen in one boot |
| other | off, said at boot |

C5 applies only on the radio backends (`=3/=4`), where every join is refused;
with `=1` that attempt goes on as with 0. The SET 20 line names the candidate
(`[study C3: result 2, link down before event 9 if it fails]`) and a failure says
`apple80211: study C3: the join failed (102), told as: ...`.

**`fuxiwifiprejoin=N`**, the pre-join GETs:

| N | Effect |
|---|---|
| 0 | as today |
| 1 | not joined: GET 16 RSSI, 4 CHANNEL, 9 BSSID answered from the last scan |
| 2 | as 1, plus GET 2 AUTH_TYPE (WPA2 if the BSS has an RSN element, else open) |
| 3 | no answer changes; timing only |
| other | off, said at boot |

"From the last scan": the network the last scan asked for **by name** if heard
(the strongest of that name), else the strongest heard; nothing heard keeps
ENXIO. The name is kept from the last named scan and never logged. GET 103, 1,
8, 13 stay "not joined": the answers say "this is what you are about to join",
not "joined" [I: whether wifid reads it that way is what the experiment shows].
These lines end `(study: not joined, from the last scan)`.

**Timing**: with either knob nonzero every request line ends with
`[+N ms]`, the time since the request before it. That places the stall: a long
gap before the first pre-join GET points at Settings or wifid, a gap between
GETs at a retry, a gap after them at something else (a scan, an XPC call) [I].

### Stall hypotheses (ranked)

1. **wifid retries the "not joined" GETs [I].** Each GET answers at once, so the
   GETs cannot block; a back-off between retries of ENXIO could. Test:
   `fuxiwifiprejoin=3`, look for repeated GET 16/4/2/9 lines with gaps; then
   `=1`/`=2`: if the gaps go, this was it.
2. **A scan before the sheet [I].** If a SET 10 appears between the tap and the
   sheet, the canned scan takes 38 x 40 ms, about 1.5 s, and the radio one about
   5.7 s (docs-lanes/wifi-ios.md). The timing shows it directly.
3. **Outside the shim [I]** (keychain lookup, Settings UI): no request lines in
   the gap at all.

## 7. Phone plan (one session)

Boot-args are added to the usual `fuxiwifiios=4` line (keep the whole command
line within the payload's 608-byte cmdline field):

1. `fuxiwifiios=4 fuxiwifijoin=8 fuxiwifiprejoin=3`: tap the locked canned
   network and enter any password 8 times; after each, note whether an alert
   came and its text, and the `[study Cn ...]` line. Capture the full log.
2. `fuxiwifiios=4 fuxiwifiprejoin=1`, then `=2`: tap the locked network, time the
   sheet, compare the `[+N ms]` gaps. (`fuxiwifijoin=0` here: C0 alone.)
3. Fill the salami column of the table; once an iPhone X capture exists, fill the
   rest and pick the candidate that matches what a real driver does for "AP
   refused" (likely C1, C2 or C3 [I]).

Whatever wins becomes the default for the scan-only backends in a separate
change; C6 never does (it would blame the user's password).

## 8. Tests

`CC=cc python3 -B tools/test-fuxi-wifi-radio.py` (T10, in both the Fuxi and the
salami build):

- `trace1`, `trace4`: a scripted Settings session (scan, pre-join GETs, a named
  scan, refused and open joins, DISASSOCIATE), every request's error, value and
  bytes (buffers prefilled with 0xa5) and every event (code, length, first 4
  bytes) printed. Built from the files as of salami `7caf3ed` (`STUDY_BASE`) and
  from these, without the knobs: identical output, and identical again on a
  second run. The log lines within one step are compared as a sorted block (the
  station's thread and the ioctl's may print in either order); each line is
  still compared byte for byte. `trace4 0 0` equals the default; `trace4 8 2`
  differs.
- `study`: `fuxiwifiios=4 fuxiwifijoin=8 fuxiwifiprejoin=2`, C1..C7 then C1 again,
  each with its SET 20 error, GET 21 and event order; the pre-join answers from
  the strongest and from a named scan; ENXIO before any scan.
- `study-loopback`: `=1` with C5 and prejoin 1: C5 is not applied, the open
  join still succeeds (GET 21 1, events 9 2 3 4), AUTH_TYPE stays ENXIO.
- `study-range`, `study-timing`: out-of-range values are off; `=3` changes no
  answer and stamps `[+N ms]` from the model clock.
- 6 study mutants (a candidate or pre-join answer without the knob, timing
  without the knob, timing-only answering, C5 where joins succeed, the strongest
  over the named network), each caught.

If one of `STUDY_FILES` is later changed on purpose, `STUDY_BASE` moves to that
change once its trace diff is understood.
