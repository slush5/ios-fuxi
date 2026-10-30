#!/usr/bin/env python3
"""W20 (fuxiwlanqrtr=6, C1) and W23 (fuxiwlanqrtr=7, C2), host-tested.

C1: copy engines, HTC and WMI init to WMI READY. C2: on top of that the
passive channel list, a station vdev, one passive scan, MGMT_RX parsing, the
BSS table and the STOP_SCAN of containment.

Builds tools/fuxi-wlan-ce-model.c with ASan/UBSan and -Werror against the pure
xnu/fuxi-wlan/fuxi_wlan_ce.h and the production glue cut out of
xnu/fuxi-wlan/fuxi_wlan.c (written to fuxi-wlan-ce-prod.inc in a temporary
directory), then runs every model scenario (docs-lanes/wlan-w20.md). Static
checks: the W20 call is gated, containment runs before the W11 stop, the C2
STOP_SCAN runs before the stopping latch, the arena is freed only on the
proven path, the W20 block writes only through its two writers, the scan is
passive, the vdev MAC is the address the wifi-ios side gives en0. When the
Linux ath12k v6.19 tree is on disk ($ATH12K_SRC or
~/op11-ios/src/linux-ath12k/drivers/net/wireless/ath/ath12k) the WMI ids and
tags, the wire struct sizes, all 73 INIT resource-config words, the HTC
constants, the CE ring geometry, ids and register offsets, and for C2 every
command word (channel list, VDEV_CREATE, NSS, START_SCAN, STOP_SCAN, HTT
version request), the event field offsets, the flags and the L19 code facts
C2 relies on are derived from it again and compared with the header and the
model's encoder dump; without the tree that part is skipped with a message.
C2b (same qrtr 7 run): a line per CHAN_INFO and a pdev stats request per PHY
pdev after COMPLETED. Static: the requests sit only on the COMPLETED path,
every wait is bounded, only the transport (a failed poll, a CE/HTC fault in
the send) can fail the run, PDEV_STAT only, the CHAN_INFO lines are capped,
every new line fits the console (254 bytes) and carries a new prefix; L19:
the ids, tags, sizes, every word order of the CHAN_INFO, stats event and
pdev stats structs, the signedness each line prints, the request words, the
target pdev rule (fw_pdev per PHY, by band) and the parse order.
C2c (W26, w20 0x100 RXRING, 0x200 IDLELINK, 0x400 CHAINMASK): the model adds
the RX and link arenas with per-region device permissions, the WBM ring 120
register file, RXDMA and the firmware's ring use, and the scenarios of
rx_path_plan.md 2.12. Static: the gate, where W26 sits in L19's order, the WBM
allowlist and the one window/shadow path, no WBM write before the pre-reads or
after a refusal, no host write to the arenas after the first SRING_SETUP or
after the latch, no HTT message but SRING_SETUP and no pdev parameter but the
two chain masks, free only in the proven branch after the D3 recount and the
zeroing, D4/D5 rules, every W26 line within 254 bytes. L19: ring ids, rdp/wrp,
HTT enums and fields, thresholds, sizes (struct replicas compiled), RBMs, the
WCN7850 WBM registers and L19's ring 120 write order, the PDEV_SET_PARAM ids,
compared word by word with the model's dump.
C2d (W27, w20 0x800 RXSEL, 0x1000 REORX): the model adds the REO arena, the REO
register conversation in L19's order, the REO/WBM device that returns every
received buffer on a destination, exception or release ring, the host's
re-posts checked at each publish, and the phone's wedge without them. Static:
the gate, W27's place in L19's order, the REO allowlist and its one window
path, the TP publish path, no REO write before the pre-reads or after a
refusal, the latch, the free, counts-only lines within 254 bytes. L19: the
ring selection words (rx_desc.h compiled), ring ids, sizes, entry structs,
registers, write orders, thresholds, the REO setup, compared word by word with
the model's dump. Mutants: every C2d rule broken once in the production code
is caught by the static checks or the model's C2d scenarios.
C2e (W28, w20 0x2000 MONSTAT): the model adds the monitor status arena with
per-region device permissions, the two HP publishes checked when they happen,
the 5th and 6th SRING_SETUP, a fake RXDMA writing one status TLV per PPDU into
the next status buffer (never given back), the six MHI/BHI registers of the
crash line and, in the C2e scenarios only, the phone's crash (RX live without
the monitor status rings: the scan stops after 15 channels, 3 with ring
selection, MHI SYS_ERR with the phone's ERRDBG words). Static: the gate, W28's
place in L19's order, the crash line's allowlist and its one read path (no
register write), the fills and publishes only at W20 and refused after the
latch, SRING_SETUP the only message, the crash line at the COMPLETED timeout
and before the MHI reset, the free, counts-only lines within 254 bytes. L19:
ring ids, sizes, entry struct (compiled), buffer budget and alignment, fill
count, RBM and cookie, the HTT ring id and type, thresholds, the announcement
and fill order, the MSI and reap facts the deviations rest on, compared word by
word with the model's dump. Mutants: every C2e rule broken once in the
production code is caught by the static checks or the model's C2e scenarios.
No device, no network.
"""
from pathlib import Path
import contextlib
import io
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
DRIVER = (ROOT / 'xnu/fuxi-wlan/fuxi_wlan.c').read_text()
HEADER = (ROOT / 'xnu/fuxi-wlan/fuxi_wlan_ce.h').read_text()
MODEL = ROOT / 'tools/fuxi-wlan-ce-model.c'
CC = [os.environ.get('CC', 'cc'), '-std=c11', '-D_DEFAULT_SOURCE', '-g', '-Wall', '-Wextra', '-Werror',
      '-Wno-unused-function', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
L19 = Path(os.environ.get('ATH12K_SRC', Path.home() / 'op11-ios/src/linux-ath12k/drivers/net/wireless/ath/ath12k'))


def between(text, start, end, keep_end=False):
    assert text.count(start) == 1, start
    body = text.split(start, 1)[1]
    assert end in body, end
    body = body.split(end, 1)[0]
    return start + body + (end if keep_end else '')


def production():
    """The fuxi_wlan.c excerpts the model links against, barriers routed to the model."""
    enum = re.search(r'enum \{ WL12_DISCOVERY,.*?\};', DRIVER, re.S)[0]
    knobs = between(DRIVER, '/* W20 (fuxiwlanqrtr=6, C1): after this boot', 'static unsigned wl20_bits;\n', True)
    tables = between(DRIVER, '/* Adapted from Linux ath12k hw.c (BSD-3-Clause-Clear), v6.12:', 'static int\nwl15_encode_cfg(')
    fail = between(DRIVER, 'static int\nwl_w11_fail(const char *reason)\n{', '\n}\n', True)
    block = between(DRIVER, '/* ---- W20: CE/HTC/WMI to WMI READY (fuxiwlanqrtr=6, C1) ----', '/* ---- end W20 ---- */')
    abort = between(DRIVER, 'static void\nwl5_abort(unsigned step)\n{', '\n}\n', True)
    for side, kind in (('w', 'W'), ('r', 'R')):
        line = f'static void wl_ce_{side}barrier(void *ctx) {{ (void)ctx; __asm__ volatile("dsb sy" ::: "memory"); }}'
        assert block.count(line) == 1, line
        block = block.replace(line, f"static void wl_ce_{side}barrier(void *ctx) {{ (void)ctx; model_barrier('{kind}'); }}")
    assert '__asm__' not in block, 'W20 block has a barrier the model does not see'
    abort = re.sub(r'__asm__ volatile\([^;]+;', '/* host: no barrier */;', abort)
    return '\n'.join([enum, knobs, tables, fail, block, abort]) + '\n'


def static_checks():
    w14 = between(DRIVER, 'static int\nwl_w14_run(void)\n{', '\n}\n')
    assert 'int error = wl_w15_run();\n\t\tif (!error && wl_w20) { error = wl_w20_run(); }' in w14
    assert DRIVER.count('wl_w20_run()') == 1 and DRIVER.count('wl_w20 = qrtr == 6 || wl_w23;') == 1
    assert DRIVER.count('wl_w23 = qrtr == 7;') == 1
    # C2: the scan stage only after W22 and only with qrtr 7; STOP_SCAN before the stopping latch.
    w22 = between(DRIVER, 'static int\nwl_w22_run(void)\n{', '\n}\n')
    assert w22.rstrip().endswith('return wl_w23 ? wl_w23_run() : 0;') and DRIVER.count('wl_w23_run()') == 1
    contain = between(DRIVER, 'static void\nwl_ce_contain(unsigned step)\n{', '\n}\n')
    assert contain.index('if (wl_w23) { wl23_stop_scan(); }') < contain.index('wl_ce.stopping = true;')
    stop = between(DRIVER, 'static void\nwl23_stop_scan(void)\n{', '\n}\n')
    assert 'wl_ce_wait' not in stop and 'wl_ce_poll' not in stop and 'wl_w11_' not in stop and 'wlce_reap_rx' not in stop
    # A W11 events/guard failure in any poll pass marks the transport unhealthy: no STOP_SCAN after it.
    assert 'if (fw_faults || wl_ce.errors || wl23_guard_bad || !wl_htc.ready || !wl_htc.wmi_eid) {' in stop
    poll = between(DRIVER, 'static int\nwl_ce_poll(void)\n{', '\n}\n')
    assert 'if (wl_w11_events()) { wl23_guard_bad = true; return 1; }' in poll
    assert 'if (wl_w11_guard()) { wl23_guard_bad = true; return 1; }' in poll
    assert '#define WL23_BITS (WL20_BITS | WL20_HTTVER | WL20_BAND24 | WL20_PDEVCAP | WL26_BITS | WL27_BITS | WL20_MONSTAT | WL20_L19ORDER)' in DRIVER
    assert '#define WL26_BITS (WL20_RXRING | WL20_IDLELINK | WL20_CHAINMASK)' in DRIVER
    assert '#define WL27_BITS (WL20_RXSEL | WL20_REORX)' in DRIVER
    bits23 = DRIVER.split('#define WL23_BITS', 1)[1].split('\n', 1)[0]
    bits26 = DRIVER.split('#define WL26_BITS', 1)[1].split('\n', 1)[0] + DRIVER.split('#define WL27_BITS', 1)[1].split('\n', 1)[0]
    assert '#define WL20_ACTIVE 0x08u' in DRIVER and 'ACTIVE' not in bits23 + bits26 and 'FULLBSSID' not in bits23 + bits26
    # Owner decision 6: every BSSID in the proof log has its last three octets masked; no path prints them.
    w24 = between(DRIVER, 'static void\nwl24_report(void)\n{', '\n}\n')
    assert 'W24 bss %02x:%02x:%02x:xx:xx:xx ch' in w24 and 'bssid[3]' not in w24 and 'FULLBSSID' not in w24
    assert DRIVER.count('WL20_FULLBSSID') == 1  # the define only: refused at every qrtr
    # Passive only: the flags, no SSID/IE/probe words, the broadcast placeholder BSSID only.
    assert '#define WLSCAN_FLAGS (WLWMI_SCAN_FLAG_PASSIVE | WLWMI_SCAN_CHAN_STAT_EVENT)' in HEADER
    enc = between(HEADER, 'static inline int\nwlwmi_encode_start_scan(', '\n}\n')
    for word in ('WLWMI_SS_NUM_SSIDS', 'WLWMI_SS_IE_LEN', 'WLWMI_SS_N_PROBES', 'WLWMI_SS_PRIORITY', 'WLWMI_SS_IE_BITMAP',
                 'WLWMI_SS_MAC_ADDR', 'WLWMI_SS_CTRL_FLAGS_EXT'):
        assert word not in enc, word
    assert 'WLWMI_CHAN_INFO_PASSIVE' in between(HEADER, 'static inline uint32_t\nwlwmi_chan_info(', '\n}\n')
    # The vdev MAC is the one the wifi-ios side gives en0 (K-W2), in both backends.
    ios = (ROOT / 'xnu/fuxi-wifi-ios/fuxi_wifi_ios.c').read_text()
    addrs = set(re.findall(r'\.address = \{ ([0-9a-fx, ]+) \}', ios))
    mac = re.search(r'static const uint8_t wl23_mac\[6\] = \{ ([0-9a-fx, ]+) \};', DRIVER)[1]
    assert addrs == {mac} and mac == '0x02, 0x46, 0x55, 0x58, 0x49, 0x01', (addrs, mac)
    assert int(mac.split(',')[0], 16) & 0x02 and not int(mac.split(',')[0], 16) & 0x01  # locally administered, unicast
    abort = between(DRIVER, 'static void\nwl5_abort(unsigned step)\n{', '\n}\n')
    assert abort.index('wl_ce_contain(step);') < abort.index('wl_w11_stop();')
    freed = abort.split('if (!bm_bad && !perst_bad && !seq_bad && !wl_smmu_undo()) {', 1)[1].split('} else {', 1)[0]
    assert 'wl_dma_free(&wl_ce_arena);' in freed and DRIVER.count('wl_dma_free(&wl_ce_arena)') == 1
    owned = between(DRIVER, 'wl_owned_alias(uint64_t pa, unsigned bytes, bool tables_only)\n{', '\n}\n')
    assert '&wl14_m3, &wl_ce_arena, &wl_rx_arena, &wl_link_arena, &wl_reo_arena, &wl_mon_arena, &wl_cm_arena };' in owned
    block = between(DRIVER, '/* ---- W20: CE/HTC/WMI to WMI READY (fuxiwlanqrtr=6, C1) ----', '/* ---- end W20 ---- */')
    # Hardware writes: the window helper (CE registers via wlce_reg_ok), the shadow writer, the
    # window restore and, with w20 0x04, the endpoint Device Control register. Nothing else.
    assert re.findall(r'\bwl_pwr\(([^,]+),', block) == ['pa', 'WL_EPCFG + cap + 8'], re.findall(r'\bwl_pwr\(([^,]+),', block)
    assert re.findall(r'\bwl_mwr\(([^,]+),', block) == ['0x310c', 'off'], re.findall(r'\bwl_mwr\(([^,]+),', block)
    assert re.findall(r'\bwl_contain_wr\(([^,]+),', block) == ['WL_BAR0 + 0x310c']
    code = re.sub(r'/\*.*?\*/', '', block, flags=re.S)
    for forbidden in ('wl_soc_write', 'rh_send', 'wl_m6o_write', 'wl_rails_on', 'wl17_', 'wl_mwr64', 'msi', 'MSI', 'wl_pwr64'):
        assert forbidden not in code, forbidden
    assert 'if (fw_faults || !wlce_reg_ok(off)) { return 1; }' in block
    assert 'off < WLCE_SHADOW_BASE || off >= WLCE_SHADOW_BASE + 4u * WLCE_SHADOW_REGS' in block
    # The pure header stays pure: no includes, no allocation, no XNU calls, the Clear BSD text.
    assert '#include' not in HEADER and 'kalloc' not in HEADER and 'printf' not in HEADER and 'malloc' not in HEADER
    assert 'The Clear BSD License' in HEADER and 'NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY' in HEADER
    assert all(f'#define {k}' in DRIVER for k in ('WL20_NOHTT 0x01u', 'WL20_MRRS128 0x04u', 'WL20_NOHWMODE 0x40u',
                                                  'WL20_ACTIVE 0x08u', 'WL20_QRTR_MAX 7', 'WL20_HTTVER 0x02u',
                                                  'WL20_BAND24 0x10u', 'WL20_FULLBSSID 0x20u', 'WL20_PDEVCAP 0x80u'))
    assert '#define WL20_BITS (WL20_NOHTT | WL20_MRRS128 | WL20_NOHWMODE)' in DRIVER
    assert 'python3 tools/test-fuxi-wlan-ce.py' in (ROOT / 'tools/ci/build.sh').read_text()
    print('W20 static: gated call, containment before the W11 stop, arena in the owned/free lists, '
          'only the window/shadow/devctl writers, pure header with the Clear BSD text')
    print('W23 static: scan only after W22 with qrtr 7, STOP_SCAN before the latch with no wait/poll/RX and not after a '
          'W11 guard failure, passive flags, no SSID/IE/probe words, ACTIVE and FULLBSSID outside every accepted set, '
          'BSSIDs always masked, vdev MAC = en0 K-W2 address (locally administered)')
    static_c2b()
    static_c2c()
    static_c2d()
    static_c2e()
    static_c2f()


def c_calls(body, name):
    """The argument text of every call NAME(...) in BODY, up to its own closing parenthesis (literals respected: a
    format may hold '); ')."""
    out = []
    for m in re.finditer(r'(?<![A-Za-z0-9_])' + name + r'\(', body):
        i, depth = m.end(), 1
        while depth:
            c = body[i]
            if c == '"':
                i += 1
                while body[i] != '"':
                    i += 2 if body[i] == '\\' else 1
            elif c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
            i += 1
        out.append(body[m.end():i - 1])
    return out


def c_args(call):
    """The top-level comma-separated arguments of a C call's argument text (string literals and brackets respected)."""
    out, cur, depth, i = [], '', 0, 0
    while i < len(call):
        c = call[i]
        if c == '"':
            j = i + 1
            while call[j] != '"':
                j += 2 if call[j] == '\\' else 1
            cur += call[i:j + 1]
            i = j + 1
            continue
        if c in '([':
            depth += 1
        elif c in ')]':
            depth -= 1
        elif c == ',' and depth == 0:
            out.append(cur.strip())
            cur = ''
            i += 1
            continue
        cur += c
        i += 1
    out.append(cur.strip())
    return out


def static_c2b():
    """C2b: where the diagnostics sit and what they may not do."""
    w23 = between(DRIVER, 'static int\nwl_w23_run(void)\n{', '\n}\n')
    # The stats exchanges only on the COMPLETED path: after the W24 report and the no-frames line, before W23 complete;
    # the COMPLETED timeout only flushes the CHAN_INFO count; every failed wait from START_SCAN on flushes it too.
    timeout_path = w23.split('if (w == 2) {', 1)[1].split('\n\t}\n', 1)[0]
    assert 'wl23_stats' not in timeout_path and 'wl23_ci_flush();' in timeout_path, timeout_path
    # (C2c: the W26 report also on that failure path, after the flush; silent without a W26 bit)
    call = 'if (wl23_stats()) { wl23_ci_flush(); wl26_report(); return 1; }'
    assert DRIVER.count('wl23_stats()') == 1 and w23.count(call) == 1
    # the hint names W24b only while w20 0x02 is off; r1c2b ran 0x02 and still heard nothing
    assert '(wl20_bits & WL20_HTTVER) ? "C2c = RX data path" : "W24b = w20 0x02"' in w23
    assert w23.index('next rung %s') < w23.index(call) < \
        w23.index('wl23_ci_flush();\n\twl26_report();  /* C2c: content, guard, error ring, links, counters, D4 and D5 lines; '
                  'silent without W26 */\n\tprintf("FUXI WLAN: W23 complete; stop/contain\\n");')
    assert 'wl_ce_wait(WL23_W_STARTED, 1000, "scan STARTED")) { wl23_ci_flush(); return 1; }' in w23
    assert 'if (w == 1) { wl23_ci_flush(); return 1; }' in w23
    strip = lambda s: re.sub(r'/\*.*?\*/', '', s, flags=re.S)  # noqa: E731
    one = strip(between(DRIVER, 'static int\nwl23_stats_pdev(uint32_t pdev, uint32_t bands)\n{', '\n}\n'))
    loop = strip(between(DRIVER, 'static int\nwl23_stats(void)\n{', '\n}\n'))
    # Only the transport fails the run: 1 right after a failed poll, or wl_ce_fail when the CE/HTC send itself fails,
    # exactly as wl_ce_send; no failing wait, other send, fail or snapshot helper.
    for bad in ('wl_ce_wait', 'wl_ce_send', 'wl_w11_fail', 'wl23_words', 'wl_ce_snapshot', 'wl_w11_'):
        assert bad not in one and bad not in loop, bad
    assert 'wl_ce_fail' not in loop and one.count('wl_ce_fail(') == 1
    assert re.search(r'if \(wlhtc_frame\(&wl_htc, eid, cmd, len, frame, sizeof\(frame\), &total\) \|\| '
                     r'wlhtc_take_credits\(&wl_htc, eid, total\) \|\|\s+wlce_send\(&wl_ce, &wl_ce_io, '
                     r'&wl_ce\.pipe\[wl_ce_tx_pipe\], eid, frame, total\)\) \{ return wl_ce_fail\("WMI REQUEST_STATS"\); \}',
                     one)
    send = between(DRIVER, 'static int\nwl_ce_send(uint32_t eid, const uint8_t *payload, uint32_t len, const char *what)\n{',
                   '\n}\n')
    assert 'wlce_send(&wl_ce, &wl_ce_io, &wl_ce.pipe[wl_ce_tx_pipe], eid, frame, total)) { return wl_ce_fail(what); }' in send
    assert one.count('return 1;') == 3 and one.count('if (r == 1) { return 1; }') == 3
    assert loop.count('return 1;') == 1 and 'if (r == 1) { return 1; }' in loop
    # Bounded: wl_ce_send's credit and TX drain bounds, 500 ms for UPDATE_STATS, all through the 1 ms poll loop, per
    # request; at most two requests (the pdev ids kept), the next only after one that went out and drained (0).
    assert 'wl23_poll_for(WL20_W_CREDITS, 3000, &took)' in one and 'wl23_poll_for(WL20_W_TX, 1000, &took)' in one
    assert 'wl23_poll_for(WL23_W_STATS, 500, &took)' in one and one.count('wl23_poll_for(') == 3
    assert 'wl23_poll_for(' not in loop
    assert 'for (uint32_t i = 0; i < n && r == 0; i++) { r = wl23_stats_pdev(wl_wmi.phy_pdev[i], wl_wmi.phy_pdev_bands[i]); }' \
        in loop and 'n = wl_wmi.phy_pdevs ? wl_wmi.phy_pdevs : 1u' in loop
    assert one.count('return 2;') == 2 and one.count('return 0;') == 2
    assert 'uint32_t phy_pdev[2], phy_pdev_bands[2], phy_pdevs;' in HEADER
    ext = between(HEADER, 'static inline int\nwlwmi_parse_ext(', '\n}\n')
    assert 'if (k >= first && k - first < nphy) {' in ext and 'if (j < 2u) {' in ext
    poll = between(DRIVER, 'static int\nwl23_poll_for(unsigned what, unsigned ms, unsigned *took)\n{', '\n}\n')
    assert 'for (unsigned t = 0; t <= ms; t++) {' in poll and 'if (t < ms) { wl_sleep(1); }' in poll and 'wl_ce_poll()' in poll
    assert 'printf' not in poll and 'wl_w11_' not in poll
    # PDEV_STAT only (no vdev stats), the target pdevs of L19's debugfs reader, through the C1 send path.
    assert 'wlwmi_encode_stats_req(pdev, cmd, sizeof(cmd), &len)' in one
    enc = between(HEADER, 'static inline int\nwlwmi_encode_stats_req(', '\n}\n')
    assert '{ WLWMI_REQUEST_PDEV_STAT, 0, 0, 0, pdev }' in enc and 'VDEV' not in enc
    # The parser cannot fail WMI: void, and wlwmi_rx returns 0 after it.
    assert 'static inline void\nwlstats_parse(' in HEADER
    assert 'if (wmi->stats && wlstats_known(*id)) { wlstats_parse(wmi->stats, p, len - 4); return 0; }' in HEADER
    # CHAN_INFO lines: one per well-formed event, 2 x 64 at most, the rest counted; UPDATE_STATS gets no unknown line.
    assert '#define WL23_CI_LINES 128u' in DRIVER
    ci = between(DRIVER, 'static void\nwl23_ci_line(void)\n{', '\n}\n')
    assert ci.index('if (wl23_ci_lines >= WL23_CI_LINES) { wl23_ci_drop++; return; }') < ci.index('printf')
    rx = between(DRIVER, 'static int\nwl_ce_wmi_rx(void *ctx, const uint8_t *msg, uint32_t len)\n{', '\n}\n')
    assert 'if (wl_scan.ci_seen != ci) { wlrx_note_ci(&wl_rx, wl_scan.ci); wl23_ci_line(); }' in rx
    assert rx.index('if (wl_wmi.stats && wlstats_known(id)) { return 0; }') < rx.index('if (!wlwmi_known(id)) {')
    longest = c2b_line_bounds()
    print('C2b static: stats requests only on the COMPLETED path, waits bounded (credits 3000, TX 1000, UPDATE_STATS 500 ms '
          'per request, at most two), only the transport fails the run (a failed poll; a CE/HTC send fault as wl_ce_send), '
          'PDEV_STAT with the PHY pdevs through the C1 send path, a void parser, CHAN_INFO lines capped at 128 and counted '
          f'(also after a failed wait), every new line with a new prefix and at most {longest} of 254 bytes')


def c2b_line_bounds():
    """Every C2b printf fits the console line: XNU keeps at most 255 bytes of a printf line, newline included, and
    drops the rest with the newline when drain_uart_sync is off (osfmk/console/serial_general.c:159-188,
    SERIAL_CONS_BUF_SIZE 256), so each line's worst case (10 bytes per %u or %#x, 11 per %d, the longest string a %s
    can get) must be 254 or less; and every line starts with one of the two new prefixes (the qrtr 7 rule)."""
    fns = ('static void\nwl23_ci_line(void)\n{', 'static void\nwl23_ci_flush(void)\n{',
           'static void\nwl23_stats_report(unsigned took, unsigned tx0, unsigned rx0)\n{',
           'static int\nwl23_stats_pdev(uint32_t pdev, uint32_t bands)\n{', 'static int\nwl23_stats(void)\n{')
    parse = between(HEADER, 'static inline void\nwlstats_parse(', '\n}\n')
    one = between(DRIVER, fns[3], '\n}\n')
    reasons = re.findall(r'why = (?:ev \? )?"([^"]*)"(?: : "([^"]*)")?;', parse)
    reasons = [r for pair in reasons for r in pair if r]
    assert len(reasons) == 6, reasons
    widths = {
        'fs': max(len('START'), len('END'), len('0xffffffff')),  # or the %#x of the 32-bit flags word
        'why': max(len(r) for r in re.findall(r'why = "([^"]*)";', one)),
        'wl_stats.why ? ", last: " : ""': len(', last: '),
        'wl_stats.why ? wl_stats.why : ""': max(len(r) for r in reasons),
    }
    longest = n = 0
    for fn in fns:
        body = between(DRIVER, fn, '\n}\n')
        for call in c_calls(body, 'printf'):
            args = c_args(call)
            fmt = ''.join(re.findall(r'"((?:[^"\\]|\\.)*)"', args[0])).replace('\\n', '\n')
            assert fmt.startswith(('FUXI WLAN: W23 chan-info ', 'FUXI WLAN: W23 stats ')) and fmt.endswith('\n'), fmt
            specs = re.findall(r'%#?[a-z]', fmt)
            assert len(specs) == len(args) - 1, (fmt, args)
            worst = len(fmt) - 1 - sum(len(s) for s in specs)
            for spec, arg in zip(specs, args[1:]):
                worst += {'%u': 10, '%#x': 10, '%d': 11}[spec] if spec != '%s' else widths[arg]
            assert worst <= 254, (worst, fmt)
            longest = max(longest, worst)
            n += 1
    assert n == 14, n
    return longest


W26_START = '/* ---- C2c (fuxiwlanqrtr=7): W26, host RX rings, the WBM idle list and the chain masks ----'
H26_START = '/* ---- C2c (fuxiwlanqrtr=7): W26, host RX rings, the WBM idle list and the chain masks ----'
W27_START = '/* ---- C2d (fuxiwlanqrtr=7): W27, RX ring selection and the REO/WBM buffer return path ----'
H27_START = W27_START
W28_START = '/* ---- C2e (fuxiwlanqrtr=7): W28, the RXDMA monitor status rings and the crash line ----'
H28_START = W28_START


def strip_comments(s):
    return re.sub(r'/\*.*?\*/', '', s, flags=re.S)


def c_functions(region):
    """{name: body} of the functions defined at column 0 in REGION (static [inline] TYPE\\nNAME(...)\\n{ ... \\n})."""
    out = {}
    for m in re.finditer(r'\n(?:static (?:inline )?)?[a-z_][\w ]*\*?\n(\w+)\(([^)]*)\)\n\{', region):
        out[m[1]] = region[m.end():region.index('\n}\n', m.end())]
    return out


def static_c2c():
    """C2c: the gate, the place of each W26 step in L19's order, the register paths, the host-write and free rules."""
    code = strip_comments(DRIVER)
    for k, v in (('WL20_RXRING', '0x100u'), ('WL20_IDLELINK', '0x200u'), ('WL20_CHAINMASK', '0x400u')):
        assert f'#define {k} {v}' in DRIVER, k
    # The gate: 0x100 needs 0x02 (with 0x01 refused by the 0x01 + 0x02 rule); qrtr 6 keeps WL20_BITS (no W26 bit).
    cfg = between(DRIVER, 'static void\nwl_probe_configure(void)\n{', '\n}\n')
    assert '((unsigned)w20 & (WL20_RXRING | WL20_HTTVER)) == WL20_RXRING || stop22 == 1 || keep == 1 ||\n' in cfg
    assert 'unsigned allowed = wl_w23 ? WL23_BITS : WL20_BITS;' in cfg and 'WL26' not in DRIVER.split('#define WL20_BITS', 1)[1].split('\n')[0]
    assert cfg.index('if (wl20_bits & WL26_BITS) {') > cfg.index('htt version %s')  # the W26 selected line after the W23 one
    region = between(DRIVER, W26_START, W27_START)
    fns = c_functions(region)
    for name in ('wl26_wbm', 'wl26_wbm_rd', 'wl26_alloc', 'wl26_idle', 'wl26_setup', 'wl26_sring', 'wl26_chainmask', 'wl26_rings',
                 'wl26_watch', 'wl26_list', 'wl26_report', 'wl26_contain', 'wl26_free'):
        assert name in fns, name
    # L19's order: W20 after the CE rings and before the RX post (dp_alloc between ce_init_pipes and core_start); in W23
    # right after the HTT version block and before the channel list; the watches at STARTED, in the COMPLETED wait, and
    # as soon as it returns; the report after the C2b lines (and in the COMPLETED-timeout path, before the snapshot).
    w20 = between(DRIVER, 'static int\nwl_w20_run(void)\n{', '\n}\n')
    assert w20.index('if (wl_ce_alloc() || wl_ce_init_rings()) { return 1; }') < w20.index('if (wl26_setup()) { return 1; }') < \
        w20.index('wlhtc_init(&wl_htc')
    w23 = between(DRIVER, 'static int\nwl_w23_run(void)\n{', '\n}\n')
    assert w23.index('W23 htt version %u.%u (w20 0x02)') < w23.index('if (wl26_rings()) { return 1; }') < \
        w23.index('wlwmi_chan_list_max(') < w23.index('wl26_watch(WLRX_P_STARTED);') < w23.index('wl23_wait_done(') < \
        w23.index('wl26_watch(WLRX_P_COMPLETED);') < w23.index('wl24_report();')
    timeout_path = w23.split('if (w == 2) {', 1)[1].split('\n\t}\n', 1)[0]
    assert timeout_path.index('wl23_ci_flush();') < timeout_path.index('wl26_report();') < timeout_path.index('wl_ce_snapshot(')
    wait = between(DRIVER, 'static int\nwl23_wait_done(unsigned ms, unsigned *took)\n{', '\n}\n')
    assert 'if (!wl_rx.w[WLRX_P_MID].taken && wl_scan.foreign >= (wl_scan.nchan + 1u) / 2u) { wl26_watch(WLRX_P_MID); }' in wait
    assert wait.index('wl_ce_poll()') < wait.index('wl26_watch(WLRX_P_MID)') < wait.index('if (wl_scan.done)')
    # D5: boots without a W26 bit keep C2b's exact hint; with one, the hint comes after the W26 lines.
    assert 'if (!wl_scan.frames && (wl20_bits & WL26_BITS)) { wl26_heard_none = true; }' in w23
    assert '\telse if (!wl_scan.frames) {\n\t\tprintf("FUXI WLAN: W24 no beacon or probe response heard (chan-info %u); next rung %s\\n", ' \
        'wl_scan.ci_start,\n\t\t    (wl20_bits & WL20_HTTVER) ? "C2c = RX data path" : "W24b = w20 0x02");' in w23
    rep = fns['wl26_report']
    assert rep.index('wl26_heard_none') > rep.index('TX COUNTERS NON-ZERO') > rep.index('W26 counters')
    assert 'if (!(wl20_bits & WL26_BITS)) { return; }' in rep
    # D4: the four stats tx counters, the base tx-frame and the CHAN_INFO tx-frame, as notes with no line of their own in
    # C2b's code; the D4 line tests all five.
    assert 'static const uint8_t f[5] = { WLWMI_PS_LOCAL_ENQUED, WLWMI_PS_HW_QUEUED, WLWMI_PS_MPDU_ENQUED, WLWMI_PS_HW_REAPED,\n' \
        '\t\tWLWMI_PS_TX_FRAME_COUNT };' in HEADER
    assert 'if (wl_rx.tx_ci || wl_rx.tx_stats[0] || wl_rx.tx_stats[1] || wl_rx.tx_stats[2] || wl_rx.tx_stats[3] || ' \
        'wl_rx.tx_stats[4]) {' in rep
    # The W26 report also when the stats exchange ended the run (after the CHAN_INFO flush, before the return).
    assert w23.count('wl26_report();') == 3 and 'if (wl23_stats()) { wl23_ci_flush(); wl26_report(); return 1; }' in w23
    assert 'wlrx_note_stats(&wl_rx, s);  /* C2c D4: no line here */' in DRIVER
    # The HTT handler counts types whenever a W26 bit is set (installed at the HTT connect), VERSION_CONF unchanged.
    assert 'else if (wl20_bits & WL26_BITS) { wl_htc.htt_fn = wl23_htt_rx; }' in between(DRIVER, 'static int\nwl_w21_run(void)\n{', '\n}\n')
    # Registers: one window implementation (wl_win), reached from exactly two allowlisted accessors; the WBM accessor is
    # wlrx_wbm_ok's and refuses after the latch; W26 never calls a raw accessor; the one BAR write is the HP publish to
    # the shadow register looked up for ring 120's R2 HP (index 19).
    block = between(DRIVER, '/* ---- W20: CE/HTC/WMI to WMI READY (fuxiwlanqrtr=6, C1) ----', '/* ---- end W20 ---- */')
    assert strip_comments(block).count('wl_win(') == 6, 'wl_win: the definition and the five accessors only (CE, WBM, C2d REO, C2f reg, C2f cmem)'
    assert 'if (fw_faults || wl_rx.stopping || !wlrx_wbm_ok(off, wr)) { return 1; }\n\treturn wl_win(off, wr, value);' in fns['wl26_wbm']
    w26code = strip_comments(region)
    for raw in ('wl_prd(', 'wl_pwr(', 'wl_mrd(', 'wl_mwr(', 'wl_contain_wr(', 'wl_ce_window(', 'wlce_wr(', 'wl_soc_write', 'rh_send',
                'wl_m6o_', 'wl17_', 'msi', 'MSI', 'gic', 'GIC', 'pmic', 'PMIC', 'wl_rails', 'irq', 'IRQ'):
        assert raw not in w26code, raw
    assert w26code.count('wl_ce_io_bar_wr(') == 1 and \
        'wl_ce_io_bar_wr(NULL, WLCE_SHADOW_BASE + 4u * idx, hp)' in fns['wl26_idle']
    assert 'if (wl15_shadow_v3[j] == WLRX_WBM_IDLE_HP) { idx = j; }' in fns['wl26_idle']
    # No WBM write before the nine pre-reads or after a refusal: the pre-read loop, the refusal return, then the writes.
    idle = fns['wl26_idle']
    assert idle.index('for (unsigned i = 0; i < WLRX_PRE_N; i++) { wl_rx.wbm_pre[i] = wl26_wbm_rd(wlrx_pre_addr(i)); }') < \
        idle.index('if (wl_rx.idle_own != WLRX_IDLE_FREE) {') < idle.index('return 0;') < idle.index('wl26_wbm(regs[i][0], true,')
    assert idle.index('wl26_wbm(regs[i][0], true,') < idle.index('rb = wl26_wbm_rd(regs[k][0]);') < \
        idle.index('wlrx_link_fill(&wl_rx)') < idle.index('wl_ce_wbarrier(NULL);') < idle.index('wl_ce_io_bar_wr(')
    # Both arenas before any WBM write; the host writes into the arenas only through the three H fills, called only from
    # W20 (wl26_setup / wl26_idle), so nothing after the first SRING_SETUP; each refuses after the latch.
    setup = fns['wl26_setup']
    assert setup.index('wl26_alloc(&wl_link_arena') < setup.index('wl26_alloc(&wl_rx_arena') < setup.index('wl26_idle()') < \
        setup.index('wlrx_fill_refill(&wl_rx)')
    for fill in ('wlrx_fill_refill(', 'wlrx_publish_refill(', 'wlrx_link_fill('):
        assert code.count(fill) == 1 and (fill in fns['wl26_setup'] or fill in fns['wl26_idle']), fill
        assert 'if (rx->stopping' in between(HEADER, 'static inline int\n' + fill, '\n}\n'), fill
    assert code.count('wl26_setup()') == 1 and code.count('wl26_rings()') == 1  # one call each
    # No store into either arena from the glue: no rdp/wrp/ring/slot/link word assigned, no memcpy/bzero into them but
    # wl26_free's zeroing; the arena pointers are only handed to the H layout functions.
    assert not re.search(r'wl_rx\.(?:rdp|wrp|base|bufs|link)\s*\[[^\]]*\]\s*[-+|&^]?=', code)
    assert not re.search(r'wl_rx\.(?:ring\[[^\]]*\]|idle)\.va\s*\[[^\]]*\]\s*[-+|&^]?=', code)
    assert not re.search(r'(?:memcpy|bzero|memset)\(\s*\(?[^,]*wl_rx\.', code)
    assert re.findall(r'bzero\(\(void \*\)(wl_\w+_arena)\.va', code) == ['wl_rx_arena', 'wl_link_arena', 'wl_reo_arena', 'wl_mon_arena', 'wl_cm_arena'] and \
        all(f'bzero((void *)wl_{a}_arena.va' in fns['wl26_free'] for a in ('rx', 'link'))
    assert 'WLRX_PUBLISH(' in between(HEADER, 'static inline int\nwlrx_publish_refill(', '\n}\n') and \
        HEADER.count('WLRX_PUBLISH(') == 2  # the default macro and the one use
    # HTT: only SRING_SETUP (0xb), RESPONSE_REQUIRED never set, no ring selection / PPDU / EXT stats encoder.
    h26 = strip_comments(HEADER.split(H26_START, 1)[1].split(H27_START, 1)[0])
    assert 'WLRX_HTT_SRING_SETUP |' in h26 and h26.count('WLRX_HTT_RESPONSE_REQUIRED') == 1  # the define only
    for word in ('RING_SELECTION', 'PPDU_STATS', 'EXT_STATS', '0xcu', '0x11u'):
        assert word not in w26code and word not in h26, word
    # PDEV_SET_PARAM: params 1 and 2 only, TX then RX, through the C2 one-TLV helper.
    cm = fns['wl26_chainmask']
    assert cm.index('WLWMI_PDEV_PARAM_TX_CHAIN_MASK, tx }') < cm.index('WLWMI_PDEV_PARAM_RX_CHAIN_MASK, rx }')
    assert cm.index('wl23_words(WLWMI_CMD_PDEV_SET_PARAM, WLWMI_TAG_PDEV_SET_PARAM_CMD, t,') < \
        cm.index('wl23_words(WLWMI_CMD_PDEV_SET_PARAM, WLWMI_TAG_PDEV_SET_PARAM_CMD, r,')
    assert re.findall(r'#define WLWMI_PDEV_PARAM_\w+', HEADER.split('---- C2f', 1)[0]) == ['#define WLWMI_PDEV_PARAM_TX_CHAIN_MASK',
                                                                   '#define WLWMI_PDEV_PARAM_RX_CHAIN_MASK']
    assert 'if (!tx || !rx) {' in cm and cm.index('if (!tx || !rx) {') < cm.index('wl23_words(')
    # Containment: wl26_contain first in wl_ce_contain; wl26_free only in wl5_abort's proven branch, before the other
    # frees: D3 recount, then zeroing, then the frees; both arenas free nowhere else.
    contain = between(DRIVER, 'static void\nwl_ce_contain(unsigned step)\n{', '\n}\n')
    assert contain.index('wl26_contain();') < contain.index('if (wl_w23) { wl23_stop_scan(); }') < contain.index('wl_ce.stopping = true;')
    assert 'wl_rx.stopping = true;' in fns['wl26_contain']
    abort = between(DRIVER, 'static void\nwl5_abort(unsigned step)\n{', '\n}\n')
    proven = abort.split('if (!bm_bad && !perst_bad && !seq_bad && !wl_smmu_undo()) {', 1)[1].split('} else {', 1)[0]
    assert proven.strip().startswith('wl26_free();') and code.count('wl26_free()') == 1 and 'wl26_free' not in abort.split('} else {', 1)[1]
    fr = fns['wl26_free']
    assert fr.index('wlrx_guard_count(') < fr.index('bzero((void *)wl_rx_arena.va') < fr.index('bzero((void *)wl_link_arena.va') < \
        fr.index('wl_ce_wbarrier(NULL);') < fr.index('wl_dma_free(&wl_rx_arena);') < fr.index('wl_dma_free(&wl_link_arena);')
    assert DRIVER.count('wl_dma_free(&wl_rx_arena)') == 1 and DRIVER.count('wl_dma_free(&wl_link_arena)') == 1
    # D2 in the pure derive; D1 refusal order (unreadable, SRNG_ENABLE, base, then scatter = ambiguous).
    der = between(HEADER, 'static inline void\nwlrx_watch_derive(', '\n}\n')
    assert 'if ((w->exhausted & (f[1] | f[2])) || (w->kept_valid && s->kept_valid && w->kept > s->kept)) { w->exhausted |= f[0]; }' \
        in der and der.index('if (!w->mac_avail[m] && w->mac_tp[m]) { w->exhausted |= f[1 + m]; }') < der.index('f[1] | f[2]')
    # The owner rule (counts only): no printf in the W26 glue takes an argument that reads an arena (a buffer, a link
    # descriptor, a ring entry or a pointer word) or WLRX_LOAD; error entries reach a line only as wlrx_content's
    # masked err_raw, whose two frame masks are L19's fields.
    arena = re.compile(r'\bwl_rx\.(?:bufs|link|base|rdp|wrp)\b|\.va\b|\bWLRX_LOAD\b|\bwl_(?:rx|link)_arena\b')
    nprint = 0
    for name, body in fns.items():
        for fn in ('printf', 'snprintf', 'wl26_list'):
            for call in c_calls(strip_comments(body), fn):
                for arg in c_args(call)[1:]:
                    assert not arena.search(arg) or arg in ('wl_rx_arena.iova', '(unsigned)wl_rx_arena.size', 'wl_link_arena.iova',
                                                            '(unsigned)wl_link_arena.size'), (name, arg)
                nprint += 1
    assert nprint >= 40, nprint
    con = between(HEADER, 'static inline void\nwlrx_content(', '\n}\n')
    assert 'c->err_raw[c->err_entries][1] = d[5] & ~WLRX_ERR_W5_FRAME;' in con and \
        'c->err_raw[c->err_entries][2] = w6 & ~WLRX_ERR_W6_FRAME;' in con and con.count('err_raw[') == 4
    assert header_define('WLRX_ERR_W5_FRAME') == 0x003fff00 and header_define('WLRX_ERR_W6_FRAME') == 0x7ff80780
    own = between(HEADER, 'static inline unsigned\nwlrx_idle_owned(', '\n}\n')
    assert own.index('WLRX_IDLE_UNREADABLE') < own.index('WLRX_IDLE_SRNG') < own.index('WLRX_IDLE_BASE') < own.index('WLRX_IDLE_AMBIGUOUS')
    longest = c2c_line_bounds(fns, cfg)
    print('C2c static: gate (0x100 needs 0x02, qrtr 6 refuses W26), W26 in L19\'s order (W20 between the CE rings and the RX '
          'post, W23 between VERSION_CONF and the channel list, watches at STARTED/mid/COMPLETED), one window path for the CE '
          'and WBM allowlists, one shadow-19 HP publish, no WBM write before the pre-reads or after a refusal, arena fills only '
          'in W20 and refused after the latch, SRING_SETUP the only HTT message, PDEV_SET_PARAM ids 1 then 2 only, D2/D4/D5 '
          f'rules, wl26_contain first, free only in the proven branch after the D3 recount and the zeroing, no arena word in '
          f'any of the {nprint} W26 printf/snprintf/list calls (error entries only masked); W26 lines at most {longest} of 254 bytes')


def c2c_line_bounds(fns, cfg, tag='W26', min_lines=28, extra=None):
    """Every W26 line within the console's 254 bytes (C2b's rule): 10 bytes per %u, %x with 0x or %#010x, 8 per %x, 11
    per %d, 18 per %#llx; a %s takes its longest literal, the longest snprintf into a local buffer (each of which must
    fit its buffer), the longest literal a pure function returns or a table holds. Bounded derived fields of the
    watch segments (in the branch where the words are inside their ring: tp < 8192 with taken/left <= 4095, MAC hp/tp <
    4096 and idle < 65536 with avail <= 32767, err hp < 8192 with fill <= 1023) are counted at those widths; the model's
    s_c2c_widest drives every watch state at its widest and fails any longer line. wl26_list is checked by its flush rule."""
    tables = {}
    for name, body in re.findall(r'static const char \*const (\w+)\[\w*\] = \{(.*?)\};', DRIVER, re.S):
        tables[name] = max(len(s) for s in re.findall(r'"([^"]*)"', body))
    pure = {}
    for name in ('wlrx_idle_why', 'wlrx_next', 'wlrd_why', 'wlrd_next', 'wlms_next', 'wlcm_why'):
        body = between(HEADER, f'static inline const char *\n{name}(', '\n}\n')
        pure[name] = max(len(''.join(x)) for x in re.findall(r'return [^;]*?"([^"]*)"(?:\s*"([^"]*)")?', body) +
                         re.findall(r'[:?]\s*"([^"]*)"(?:\s*"([^"]*)")?', body))
    bounded = {'tp %u taken %u left %u': [4, 4, 4], 'hp %u tp %u avail %u': [5, 5, 5], 'hp %u fill %u': [4, 4],
               'hp %u tp %u taken %u left %u written %u': [4, 4, 4, 4, 4], 'hp %u tp %u %s written %u': [4, 10, 0, 4]}

    def fmt_of(arg):
        return ''.join(re.findall(r'"((?:[^"\\]|\\.)*)"', arg)).replace('\\n', '\n')

    def worst(fmt, args, local, assigned):
        specs = re.findall(r'%[#0-9]*(?:ll)?[udxs]', fmt)
        assert len(specs) == len(args), (fmt, args)
        w = len(fmt) - sum(len(s) for s in specs) - fmt.endswith('\n')
        for i, (spec, arg) in enumerate(zip(specs, args)):
            if spec != '%s':
                if fmt in bounded:
                    w += bounded[fmt][i]
                else:
                    w += {'%u': 10, '%#x': 10, '%#010x': 10, '%x': 8, '%d': 11, '%#llx': 18}[spec]
                continue
            cand = [len(s.encode().decode('unicode_escape')) for s in re.findall(r'"((?:[^"\\]|\\.)*)"', arg)]
            cand += [local[n] for n in re.findall(r'\b(\w+)(?:\[\w+\])?', arg) if n in local]
            cand += [tables[n] for n in re.findall(r'\b(\w+)\[', arg) if n in tables]
            cand += [pure[n] for n in re.findall(r'\b(\w+)\(', arg) if n in pure]
            cand += [assigned[n] for n in re.findall(r'\b(\w+)\b', arg) if n in assigned]
            assert cand, (fmt, arg)
            w += max(cand)
        return w
    longest = lines = 0
    for name, body in list(fns.items()) + [('wl_probe_configure', cfg)]:
        local, assigned, sizes = {}, dict(extra or {}), {}
        for decl in re.findall(r'\bchar ([^;=]*);', body):  # char a[N], b[M][N], ...: the last dimension
            for d in decl.split(','):
                m = re.match(r'\s*(\w+)((?:\[\d+\])+)\s*$', d)
                if m:
                    sizes[m[1]] = int(re.findall(r'\[(\d+)\]', m[2])[-1])
        for m in re.finditer(r'const char \*(\w+) = (.*?);', body, re.S):
            assigned[m[1]] = max(len(s) for s in re.findall(r'"([^"]*)"', m[2]))
        for call in c_calls(body, 'snprintf') if name != 'wl_probe_configure' else []:
            a = c_args(call)
            buf = re.match(r'(\w+)', a[0])[1]
            if buf in ('item', 'line'):
                continue  # wl26_list's own buffers: checked below
            w = worst(fmt_of(a[2]), a[3:], local, assigned)
            assert w <= sizes[buf] - 1, (name, buf, w, sizes[buf])
            local[buf] = max(local.get(buf, 0), w)
        for call in c_calls(body, 'printf'):
            a = c_args(call)
            fmt = fmt_of(a[0])
            if name == 'wl_probe_configure' and tag not in fmt:
                continue
            if name == 'wl26_list':
                assert fmt == '%s\n', fmt
                continue
            assert fmt.startswith(('FUXI WLAN: W26 ', 'FUXI WLAN: W27 ', 'FUXI WLAN: W28 ', 'FUXI WLAN: W29 ', 'FUXI WLAN: W24 no beacon')) and \
                fmt.endswith('\n'), fmt
            w = worst(fmt, a[1:], local, assigned)
            assert w <= 254, (name, w, fmt)
            longest = max(longest, w)
            lines += 1
    if tag != 'W26':
        assert lines >= min_lines, lines
        return longest
    lst = fns['wl26_list']
    assert 'char line[256], item[48];' in lst and 'if (len && len + (size_t)k > 254u) { printf("%s\\n", line); len = 0; }' in lst
    assert 'if (k <= 0 || (size_t)k >= sizeof(item)) { continue; }' in lst
    assert len(' grpcast-amsdu-wds 4294967295') < 48 and len('FUXI WLAN: W26 rxdma err codes') + len(' other 4294967295') < 254
    assert lines >= 28, lines
    return longest


def static_c2d():
    """C2d: the gate, the place of each W27 step in L19's order, the REO allowlist and its one window path, the TP
    publish path, no REO write before the pre-reads or after a refusal, the latch, the free, counts-only lines."""
    drv, hdr = DRIVER, HEADER
    code = strip_comments(drv)
    for k, v in (('WL20_RXSEL', '0x800u'), ('WL20_REORX', '0x1000u')):
        assert f'#define {k} {v}' in drv, k
    cfg = between(drv, 'static void\nwl_probe_configure(void)\n{', '\n}\n')
    assert '(((unsigned)w20 & WL27_BITS) && !((unsigned)w20 & WL20_RXRING)) ||\n' in cfg
    assert 'WL27' not in drv.split('#define WL20_BITS', 1)[1].split('\n')[0]  # qrtr 6 refuses both
    assert cfg.index('if (wl20_bits & WL27_BITS) {') > cfg.index('if (wl20_bits & WL26_BITS) {')
    region = between(drv, W27_START, W28_START)
    fns = c_functions(region)
    for name in ('wl27_reo', 'wl27_reo_rd', 'wl27_shadow', 'wl27_tp', 'wl27_ring', 'wl27_setup', 'wl27_dest', 'wl27_sel', 'wl27_poll',
                 'wl27_watch', 'wl27_report'):
        assert name in fns, name
    w26 = c_functions(between(drv, W26_START, W27_START))
    # L19's order: the REO arena after the RX arena and before any WBM write; the common setup after the idle list and
    # before the refill (dp_alloc: idle ring, common setup, RX alloc); the destination rings after service ready, before
    # INIT; ring selection right after the SRING_SETUPs (dp_rx.c:4670); the poll after the CE reaps, before the guard.
    st = w26['wl26_setup']
    assert st.index('wl26_alloc(&wl_rx_arena') < st.index('wl26_alloc(&wl_reo_arena') < st.index('wl26_idle()') < \
        st.index('if (wl27_setup()) { return 1; }') < st.index('wlrx_fill_refill(&wl_rx)') < st.index('wlrd_slots_init(&wl_rd);')
    w21 = between(drv, 'static int\nwl_w21_run(void)\n{', '\n}\n')
    assert w21.index('W21 wmi service ready complete') < w21.index('if (wl27_dest()) { return 1; }') < w21.index('return wl_w22_run();')
    rings = w26['wl26_rings']
    assert rings.index('if (wl26_sring(i)) { return 1; }') < rings.index('(wl27_sel(0) || wl27_sel(1))') < \
        rings.index('wl23_poll_for(WL20_W_NONE, 100, &took)')
    poll = between(drv, 'static int\nwl_ce_poll(void)\n{', '\n}\n')
    assert poll.index('wlce_reap_tx(&wl_ce, &wl_ce_io, &wl_ce.pipe[3])') < poll.index('if (wl_rd.reo_on && wl27_poll()) { return 1; }') < \
        poll.index('if (wl_w11_guard())')
    assert code.count('wl27_setup()') == 1 and code.count('wl27_dest()') == 1 and code.count('wl27_poll()') == 1
    # Registers: the REO accessor is wlrd_reg_ok's, refused after the latch, through wl_win; the W27 code has no raw
    # accessor, no MSI/IRQ/PMIC; the only BAR write is the TP publish to the shadow register of the ring's R2 TP.
    assert 'if (fw_faults || wl_rd.stopping || !wlrd_reg_ok(off, wr)) { return 1; }\n\treturn wl_win(off, wr, value);' in fns['wl27_reo']
    w27code = strip_comments(region)
    for raw in ('wl_prd(', 'wl_pwr(', 'wl_mrd(', 'wl_mwr(', 'wl_contain_wr(', 'wl_ce_window(', 'wlce_wr(', 'wl_soc_write', 'rh_send',
                'wl_m6o_', 'wl17_', 'msi', 'MSI', 'gic', 'GIC', 'pmic', 'PMIC', 'wl_rails', 'irq', 'IRQ', 'wl26_wbm(', 'wl23_words(',
                'WLWMI_CMD', 'wlwmi_encode'):
        assert raw not in w27code, raw
    assert w27code.count('wl_win(') == 1 and w27code.count('wl_ce_io_bar_wr(') == 1 and \
        'wl_ce_io_bar_wr(NULL, WLCE_SHADOW_BASE + 4u * idx, tp)' in fns['wl27_tp']
    assert 'if (wl15_shadow_v3[j] == r->r2 + 4u) { idx = j; }' in fns['wl27_shadow']
    tp = fns['wl27_tp']
    assert tp.index('if (wl_rd.stopping) { return 1; }') < tp.index('wl_ce_wbarrier(NULL);') < tp.index('wl27_reo(r->r2 + 4u, true, &tp)')
    # No REO write before the 38 pre-reads or after a refusal; the destination rings read again at W21.
    s = fns['wl27_setup']
    assert s.index('for (unsigned i = 0; i < WLRD_PRE_N; i++) { wl_rd.pre[i] = wl27_reo_rd(wlrd_pre_addr(i)); }') < \
        s.index('if (wl_rd.own != WLRD_FREE) {') < s.index('w20 0x1000 dropped for this boot') < s.index('wl27_ring(WLRD_R_REL)') < \
        s.index('wl27_ring(WLRD_R_EXC)') < s.index('gen = wl27_reo_rd(WLRD_REO_GEN_ENABLE);') < s.index('misc = wl27_reo_rd(WLRD_REO_MISC_CTRL);')
    d = fns['wl27_dest']
    assert d.index('wl27_reo_rd(wlrd_pre_addr(3u * i))') < d.index('return wl_w11_fail("W27 reo destination ring not free at W21");') < \
        d.index('if (wl27_ring(i)) { return 1; }')
    r = fns['wl27_ring']
    assert r.index('wl27_reo(regs[k][0], true, &regs[k][1])') < r.index('base = wl27_reo_rd(regs[0][0]);') < r.index('r->on = true;')
    # HTT: the one new message is RX_RING_SELECTION_CFG through wl_ce_send on the HTT endpoint; no reply asked.
    assert w27code.count('wl_ce_send(') == 1 and 'wl_ce_send(eid, msg, sizeof(msg), "HTT RX_RING_SELECTION_CFG")' in fns['wl27_sel']
    h27 = strip_comments(hdr.split(H27_START, 1)[1])
    assert 'WLRX_HTT_RESPONSE_REQUIRED' not in h27 and 'WLRD_HTT_RX_RING_SEL |' in h27
    # Host writes into the arenas only through the H functions (recycle: zero then sentinel; replenish: barrier then one
    # publish), each refused after the latch; the glue stores into neither arena.
    assert not re.search(r'wl_rd\.(?:rdp|base)\s*\[[^\]]*\]\s*[-+|&^]?=', code)
    assert not re.search(r'wl_rd\.ring\[[^\]]*\]\.va\s*\[[^\]]*\]\s*[-+|&^]?=', code)
    rec = between(hdr, 'static inline int\nwlrd_recycle(', '\n}\n')
    assert rec.index('if (rd->stopping || rx->stopping)') < rec.index('return wlrd_stop(rd, "W27 returned entry is not one of our posted buffers");') < \
        rec.index('if (rd->slot[k] != WLRD_SLOT_OUT)') < rec.index('rd->slot[k] = WLRD_SLOT_HELD;') < \
        rec.index('s[j] = 0; }') < rec.index('s[j] = 0xa5a5a5a5u; }')
    repl = between(hdr, 'static inline uint32_t\nwlrd_replenish(', '\n}\n')
    assert repl.index('if (rd->stopping || rx->stopping || !rd->pend_n) { return 0; }') < repl.index('io->rbarrier(io->ctx);') < \
        repl.index('(tp + size - rd->refill_hp - WLRX_BUF_DW) % size >= WLRX_BUF_DW') < repl.index('io->wbarrier(io->ctx);') < \
        repl.index('WLRD_PUBLISH(')
    assert hdr.count('WLRD_PUBLISH(') == 2 and hdr.count('WLRD_LOAD(') == 2
    reap = between(hdr, 'static inline int\nwlrd_reap(', '\n}\n')
    assert reap.index('hp = WLRD_LOAD(&rd->rdp[r->id]); io->rbarrier(io->ctx);') < reap.index('if (hp >= size || hp % WLRD_ENTRY_DW)')
    # Containment: wl_rd latched first in wl26_contain; the REO arena counted, zeroed and freed only in wl26_free.
    assert w26['wl26_contain'].strip().startswith('wl_rd.stopping = true;')
    fr = w26['wl26_free']
    assert fr.index('wlrd_guard_count(&wl_rd') < fr.index('bzero((void *)wl_reo_arena.va') < fr.index('wl_ce_wbarrier(NULL);') < \
        fr.index('wl_dma_free(&wl_reo_arena);')
    assert drv.count('wl_dma_free(&wl_reo_arena)') == 1
    # Counts only: no printf in the W27 glue takes an argument that reads an arena word.
    arena = re.compile(r'\bwl_rd\.(?:base|rdp)\b|\.va\b|\bWLRD_LOAD\b|\bWLRX_LOAD\b|\bwl_(?:rx|link|reo)_arena\b|\bwl_rx\.(?:bufs|link|base|rdp|wrp)\b')
    nprint = 0
    for name, body in fns.items():
        for fn in ('printf', 'snprintf', 'wl26_list'):
            for call in c_calls(strip_comments(body), fn):
                for arg in c_args(call)[1:]:
                    assert not arena.search(arg), (name, arg)
                nprint += 1
    setup_line = [c for c in c_calls(st, 'printf') if 'W27 reo arena' in c][0]
    assert all(a in ('wl_reo_arena.iova', '(unsigned)wl_reo_arena.size', 'wl_rd.rdp_iova') for a in c_args(setup_line)[1:])
    assert 'char lb[28]' in w26['wl26_watch'] and 'wl27_watch(lb);' in w26['wl26_watch']
    longest = c2c_line_bounds(fns, cfg, tag='W27', min_lines=16, extra={'point': 27})
    print('C2d static: gate (0x800/0x1000 need 0x100, qrtr 6 refuses both), W27 in L19\'s order (REO arena before any WBM '
          'write, common setup between the idle list and the refill, destination rings after service ready and before INIT, '
          'ring selection after the SRING_SETUPs, the poll after the CE reaps), the REO allowlist through the one window path, '
          'TP publishes only to the looked-up shadow register after the write barrier, no REO write before the 38 pre-reads '
          'or after a refusal, RX_RING_SELECTION_CFG the only new message, returns refused after the latch and zeroed before '
          f'their sentinel, one barriered publish, the REO arena freed only after its recount and zeroing, no arena word in '
          f'any of the {nprint} W27 print calls; W27 lines at most {longest} of 254 bytes')


def static_c2e():
    """C2e: the gate, W28's place in L19's order, the crash line's allowlist and its one read path, no register write, the
    fills and publishes only at W20 and refused after the latch, SRING_SETUP the only message, the free, counts-only lines."""
    drv, hdr = DRIVER, HEADER
    code = strip_comments(drv)
    assert '#define WL20_MONSTAT 0x2000u' in drv and header_define('WLRX_W20_MONSTAT') == 0x2000
    cfg = between(drv, 'static void\nwl_probe_configure(void)\n{', '\n}\n')
    assert '(((unsigned)w20 & WL20_MONSTAT) && !((unsigned)w20 & WL20_RXRING)) ||' in cfg
    assert 'MONSTAT' not in drv.split('#define WL20_BITS', 1)[1].split('\n')[0]  # qrtr 6 refuses it
    assert 'WL20_MONSTAT' in drv.split('#define WL23_BITS', 1)[1].split('\n')[0]
    assert cfg.index('if (wl20_bits & WL20_MONSTAT) {') > cfg.index('if (wl20_bits & WL27_BITS) {')
    region = between(drv, W28_START, '/* ---- C2f (fuxiwlanqrtr=7)')
    fns = c_functions(region)
    for name in ('wl28_rd', 'wl28_crash', 'wl28_sring', 'wl28_watch', 'wl28_report'):
        assert name in fns, name
    w26 = c_functions(between(drv, W26_START, W27_START))
    # L19's order: the fourth arena after the REO arena and before any WBM write; its fill and HP publishes after the
    # refill ring (ath12k_dp_rxdma_buf_setup); its two SRING_SETUP after the four C2c ones and before the ring selection
    # (ath12k_dp_rx_htt_setup); its watch after W27's, its report after W27's and before the hint.
    st = w26['wl26_setup']
    assert st.index('wl26_alloc(&wl_reo_arena') < st.index('wl26_alloc(&wl_mon_arena') < st.index('wl26_idle()') < \
        st.index('wlrx_fill_refill(&wl_rx)') < st.index('wlrx_publish_refill(') < st.index('wlms_fill(&wl_ms, &wl_rx, m)') < \
        st.index('wlms_publish(&wl_ms, &wl_rx, &wl_ce_io, m)') < st.index('return wl_w11_guard();')
    rings = w26['wl26_rings']
    assert rings.index('if (wl26_sring(i)) { return 1; }') < rings.index('(wl28_sring(0) || wl28_sring(1))') < \
        rings.index('(wl27_sel(0) || wl27_sel(1))') < rings.index('wl23_poll_for(WL20_W_NONE, 100, &took)')
    assert w26['wl26_watch'].index('wl27_watch(lb);') < w26['wl26_watch'].index('wl28_watch(p, lb);')
    rep = w26['wl26_report']
    assert rep.index('wl27_report();') < rep.index('wl28_report();') < rep.index('if (wl26_heard_none) {') < rep.index('wlms_next(')
    assert code.count('wl28_sring(') == 4 and code.count('wl28_watch(') == 3 and code.count('wl28_report();') == 1  # decl, def, uses; one report call
    # The crash line: one read path through the allowlist (the six reads of the W11/W15 diagnostic), no write; silent
    # without 0x2000; exactly two places: the COMPLETED timeout and wl5_abort right before the MHI reset write.
    assert 'if (!wlms_diag_ok(off, wl_bhi)) { return ~0u; }\n\treturn wl_mrd(off);' in fns['wl28_rd']
    w28code = strip_comments(region)
    assert w28code.count('wl_mrd(') == 1
    for raw in ('wl_prd(', 'wl_pwr(', 'wl_mwr(', 'wl_contain_wr(', 'wl_win(', 'wl_ce_window(', 'wl_ce_io_bar_wr(', 'wlce_wr(',
                'wl_soc_', 'rh_send', 'wl_m6o_', 'wl17_', 'wl26_wbm(', 'wl27_reo(', 'msi', 'MSI', 'gic', 'GIC', 'pmic', 'PMIC',
                'wl_rails', 'irq', 'IRQ', 'wl23_words(', 'WLWMI_CMD', 'wlwmi_encode'):
        assert raw not in w28code, raw
    cr = fns['wl28_crash']
    assert cr.index('if (!(wl20_bits & WL20_MONSTAT)) { return; }') < cr.index('wl28_rd(wlms_diag_addr(i, wl_bhi))')
    diag = [hdr.split('static const uint8_t o[WLMS_DIAG_N] = {', 1)[1].split('};', 1)[0]]
    assert ' '.join(diag[0].split()) == '0, WLMS_BHI_EXECENV, WLMS_BHI_ERRCODE, WLMS_BHI_ERRDBG1, WLMS_BHI_ERRDBG1 + 4u, WLMS_BHI_ERRDBG1 + 8u'
    assert (header_define('WLMS_MHISTATUS'), header_define('WLMS_BHI_EXECENV'), header_define('WLMS_BHI_ERRCODE'),
            header_define('WLMS_BHI_ERRDBG1'), header_define('WLMS_DIAG_N')) == (0x48, 0x28, 0x30, 0x34, 6)
    # the same six offsets the existing W11/W15 diagnostic reads (wl_wlfw_diagnostics)
    wd = between(drv, 'static void\nwl_wlfw_diagnostics(const char *label)\n{', '\n}\n')
    assert sorted(re.findall(r'wl_mrd\(([^)]*)\)', wd)) == sorted(['wl_bhi+0x28', '0x48', 'wl_bhi+0x30', 'wl_bhi+0x34', 'wl_bhi+0x38',
                                                                    'wl_bhi+0x3c'])
    assert code.count('wl28_crash(') == 4  # the declaration, the definition, the timeout, wl5_abort
    w23 = between(drv, 'static int\nwl_w23_run(void)\n{', '\n}\n')
    assert 'printf("FUXI WLAN: W23 timeout scan COMPLETED (%u ms); report, contain, no retry\\n", took);\n\t\twl28_crash("scan COMPLETED timeout");' in w23
    abort = between(drv, 'static void\nwl5_abort(unsigned step)\n{', '\n}\n')
    assert abort.index('wl_ce_contain(step);') < abort.index('wl28_crash("contain before MHI reset");') < \
        abort.index('reset_bad = wl_contain_wr((uint64_t)WL_BAR0 + 0x38, 2);')
    # HTT: the one message is SRING_SETUP through wl_ce_send on the HTT endpoint (the C2c encoder), no reply asked.
    assert w28code.count('wl_ce_send(') == 1 and 'wl_ce_send(eid, msg, sizeof(msg), "HTT SRING_SETUP")' in fns['wl28_sring']
    assert 'wlrx_sring_ring(&ms->ring[m < 2u ? m : 0u], w);' in hdr and 'wlrx_sring_ring(&rx->ring[i < WLRX_NRING ? i : 0], w);' in hdr
    # Host writes into the arena only through wlms_layout (zeroing), wlms_fill and wlms_publish, each called once from
    # wl26_setup and refused after the latch; the glue stores into neither the arena nor rdp/wrp.
    for fn in ('wlms_fill(', 'wlms_publish(', 'wlms_layout('):
        assert code.count(fn) == 1 and fn in st, fn
    for fn in ('wlms_fill(', 'wlms_publish('):
        assert 'if (ms->stopping || rx->stopping' in between(hdr, 'static inline int\n' + fn, '\n}\n'), fn
    assert not re.search(r'wl_ms\.(?:base|bufs)\s*\[[^\]]*\]\s*[-+|&^]?=', code)
    assert not re.search(r'wl_ms\.ring\[[^\]]*\]\.va\s*\[[^\]]*\]\s*[-+|&^]?=', code)
    pub = between(hdr, 'static inline int\nwlms_publish(', '\n}\n')
    assert pub.index('io->wbarrier(io->ctx);') < pub.index('WLMS_PUBLISH(&rx->wrp[ms->ring[m].id - WLRX_DMAC_START], WLMS_NBUF * WLRX_BUF_DW);')
    assert hdr.count('WLMS_PUBLISH(') == 2 and hdr.count('WLMS_LOAD(') == 2
    wa = between(hdr, 'static inline void\nwlms_watch(', '\n}\n')
    assert 'w->tp[m] = WLMS_LOAD(&rx->rdp[ms->ring[m].id]); io->rbarrier(io->ctx);' in wa
    # Never reaped: no TP or HP write after W20 (no WLMS_PUBLISH outside wlms_publish, no wrp/rdp store in the glue).
    h28 = strip_comments(between(hdr, H28_START, '#endif /* FUXI_WLAN_CE_H */'))
    assert 'wlms_publish' not in strip_comments(region) and not re.search(r'(?:rdp|wrp)\[[^\]]*\]\s*[-+|&^]?=', h28) and \
        h28.count('ms->hp[m] =') == 2 and 'WLRX_PUBLISH' not in h28 and 'WLRD_PUBLISH' not in h28
    # Containment: wl_ms latched in wl26_contain before its early return; the arena recounted, zeroed and freed only in
    # wl26_free (the proven branch), owned for the SMMU walk.
    con = w26['wl26_contain']
    assert con.index('wl_ms.stopping = true;') < con.index('if (!wl_rx_arena.va && !wl_link_arena.va && !wl_cm_arena.va) { return; }')
    fr = w26['wl26_free']
    assert fr.index('wlms_guard_count(&wl_ms') < fr.index('bzero((void *)wl_mon_arena.va') < fr.index('wl_ce_wbarrier(NULL);') < \
        fr.index('wl_dma_free(&wl_mon_arena);')
    assert drv.count('wl_dma_free(&wl_mon_arena)') == 1 and '&wl_reo_arena, &wl_mon_arena, &wl_cm_arena };' in drv
    # Counts only: no print in the W28 glue (or W28's lines in wl26_setup / wl26_free) takes an arena word.
    arena = re.compile(r'\bwl_ms\.(?:base|bufs)\b|\.va\b|\bWLMS_LOAD\b|\bWLRX_LOAD\b|\bwl_rx\.(?:bufs|link|base|rdp|wrp)\b')
    nprint = 0
    for name, body in list(fns.items()) + [('wl26_setup', st), ('wl26_free', fr)]:
        for fn in ('printf', 'snprintf'):
            for call in c_calls(strip_comments(body), fn):
                if name in ('wl26_setup', 'wl26_free') and 'W28' not in call:
                    continue
                for arg in c_args(call)[1:]:
                    assert not arena.search(arg), (name, arg)
                nprint += 1
    assert nprint >= 10, nprint
    longest = c2c_line_bounds(fns, cfg, tag='W28', min_lines=8, extra={'point': 27, 'where': len('contain before MHI reset')})
    print('C2e static: gate (0x2000 needs 0x100, qrtr 6 refuses it), W28 in L19\'s order (the arena after the REO arena and '
          'before any WBM write, fill and HP publishes after the refill ring, the two SRING_SETUP after the four C2c ones and '
          'before the ring selection, watch and report after W27\'s), the crash line\'s six reads through its allowlist (the '
          'W11/W15 diagnostic\'s offsets) and one read path, no register write, at the COMPLETED timeout and before the MHI '
          'reset only, SRING_SETUP the only message, arena writes only at W20 and refused after the latch, never reaped, '
          f'freed only after its recount and zeroing, no arena word in any of the {nprint} W28 print calls; W28 lines at most '
          f'{longest} of 254 bytes')


def l19_c2e(dump):
    """C2e: the monitor status rings (hal.h ids, hal.c config, dp.h sizes, dp_rx.c alloc / fill / announcement order,
    dp_mon.c replenish: count, cookie, RBM and buffer budget, dp_tx.c HTT mapping, dp.c thresholds), the MSI and reap facts
    the deviations rest on (hw.c, dp_mon.c, dp_tx.c, mac.c), against the header, then every word of the model's dump."""
    if not (L19 / 'wmi.h').exists():
        print(f'L19 re-derivation (C2e) SKIPPED: no Linux ath12k v6.19 tree at {L19} (set ATH12K_SRC)')
        return
    hal_h, hal_c, hw_c, dp_h, dp_c, dp_tx_c, dp_rx_c, dp_mon_c, desc, mac_c = (l19_text(n) for n in (
        'hal.h', 'hal.c', 'hw.c', 'dp.h', 'dp.c', 'dp_tx.c', 'dp_rx.c', 'dp_mon.c', 'hal_desc.h', 'mac.c'))
    H = hdefs()
    flat = lambda t: ' '.join(t.replace('\\\n', ' ').split())  # noqa: E731

    def enum(text, name):
        body = re.search(r'enum\s+' + name + r'\s*\{(.*?)\};', text, re.S)[1]
        out, cur = {}, -1
        for item in body.split(','):
            item = item.strip()
            if not item:
                continue
            if '=' in item:
                k, v = (x.strip() for x in item.split('=', 1))
                cur = out[v] if v in out else int(v, 0)
            else:
                k, cur = item, cur + 1
            out[k] = cur
        return out

    def num(name, text):
        return int(re.search(r'#define ' + name + r'\s+(\d+)\s*$', text, re.M)[1])

    def field(name, text):
        v = re.search(r'#define ' + name + r'\s+(.+?)\s*$', text, re.M)[1]
        m = re.fullmatch(r'GENMASK\((\d+), (\d+)\)', v)
        if m:
            return int(m[2]), ((1 << (int(m[1]) - int(m[2]) + 1)) - 1)
        return int(re.fullmatch(r'BIT\((\d+)\)', v)[1]), 1
    # Ring ids: the STATBUF ring of each PMAC, ring_num 0 (hal.h enum, hal.c config and ring-id arithmetic).
    ring = enum(hal_h, 'hal_srng_ring_id')
    pp = ring['HAL_SRNG_RING_ID_PMAC1_ID_END'] - ring['HAL_SRNG_RING_ID_PMAC1_ID_START']
    d0 = ring['HAL_SRNG_RING_ID_DMAC_CMN_ID_START']
    fc = flat(hal_c)
    cfgs = {t: dict(re.findall(r'\.(\w+) = ([^,]+)', b)) for t, b in re.findall(r'\[(HAL_\w+)\] = \{(.*?)\},', fc)}
    ms = cfgs['HAL_RXDMA_MONITOR_STATUS']
    assert (ms['start_ring_id'], ms['entry_size'], ms['mac_type'], ms['ring_dir'], ms['max_rings']) == \
        ('HAL_SRNG_RING_ID_WMAC1_SW2RXDMA1_STATBUF', 'sizeof(struct hal_wbm_buffer_ring) >> 2', 'ATH12K_HAL_SRNG_PMAC',
         'HAL_SRNG_DIR_SRC', '1'), ms
    assert 'HAL_RXDMA_MONITOR_STATUS' not in fn_body(hal_c, 'static int ath12k_hal_srng_create_config_wcn7850(')
    ids = [ring['HAL_SRNG_RING_ID_WMAC1_SW2RXDMA1_STATBUF'] + 0 + m * pp for m in (0, 1)]
    assert ids == [H['WLRX_RING_MON0'], H['WLRX_RING_MON1']] == [185, 193], ids
    # WCN7850: no rxdma1 (so the status rings, not the monitor buffer ring), two per pdev, RBM SW1, MSI for them.
    wcn = hw_c.split('.name = "wcn7850 hw2.0"', 1)[1].split('.name =', 1)[0]
    for fact in ('.rxdma1_enable = false', '.num_rxdma_per_pdev = 2', '.ring_mask = &ath12k_hw_ring_mask_wcn7850',
                 '.hal_params = &ath12k_hw_hal_params_wcn7850'):
        assert fact in wcn, fact
    rbm = enum(hal_h, 'hal_rx_buf_return_buf_manager')
    assert re.search(r'ath12k_hw_hal_params_wcn7850 = \{\s+\.rx_buf_rbm = HAL_RX_BUF_RBM_SW1_BM,', hw_c) and \
        rbm['HAL_RX_BUF_RBM_SW1_BM'] == H['WLRX_RBM_SW1'] == 6
    mask = hw_c.split('ath12k_hw_ring_mask_wcn7850 = {', 1)[1].split('};', 1)[0]
    assert 'ATH12K_RX_MON_STATUS_RING_MASK_0' in mask.split('.rx_mon_status = {', 1)[1].split('}', 1)[0]  # L19: MSI; ours: none
    # Allocation (dp_rx.c ath12k_dp_rx_alloc): after the error ring, before the buffer setup, in the !rxdma1 branch.
    al = flat(fn_body(dp_rx_c, 'int ath12k_dp_rx_alloc('))
    st = 'ath12k_dp_srng_setup(ab, srng, HAL_RXDMA_MONITOR_STATUS, 0, i, DP_RXDMA_MON_STATUS_RING_SIZE);'
    assert al.index('HAL_RXDMA_DST, 0, i, DP_RXDMA_ERR_DST_RING_SIZE);') < al.index('} else {') < al.index(st) < \
        al.index('ret = ath12k_dp_rxdma_buf_setup(ab);')
    assert num('DP_RXDMA_MON_STATUS_RING_SIZE', dp_h) == H['WLMS_ENTRIES'] == 1024
    # Fill (ath12k_dp_rxdma_buf_setup): the refill ring first, then each status ring with size / entry size = 1024 asked
    # (dp.c pads the ring by HAL_RING_BASE_ALIGN - 1 bytes, which the division drops); the replenish takes min(asked,
    # bufs_max, num_free) and a fresh source ring has entries - 1 free: 1023.
    bs = flat(fn_body(dp_rx_c, 'static int ath12k_dp_rxdma_buf_setup('))
    assert bs.index('ath12k_dp_rxdma_ring_buf_setup(ab, &dp->rx_refill_buf_ring);') < \
        bs.index('ret = ath12k_dp_rxdma_mon_ring_buf_setup(ab, mon_ring, HAL_RXDMA_MONITOR_STATUS);')
    mb = flat(fn_body(dp_rx_c, 'static int ath12k_dp_rxdma_mon_ring_buf_setup('))
    assert 'num_entries = rx_ring->refill_buf_ring.size / ath12k_hal_srng_get_entrysize(ab, ringtype); rx_ring->bufs_max = num_entries;' \
        in mb and 'ath12k_dp_mon_status_bufs_replenish(ab, rx_ring, num_entries);' in mb
    assert 'ring->size = (num_entries * entry_sz) + HAL_RING_BASE_ALIGN - 1;' in dp_c and num('HAL_RING_BASE_ALIGN', hal_h) == 8
    rp = flat(fn_body(dp_mon_c, 'int ath12k_dp_mon_status_bufs_replenish('))
    for fact in ('enum hal_rx_buf_return_buf_manager mgr = ab->hw_params->hal_params->rx_buf_rbm;',
                 'req_entries = min(req_entries, rx_ring->bufs_max);', 'num_free = ath12k_hal_srng_src_num_free(ab, srng, true);',
                 'req_entries = min(num_free, req_entries);', 'skb = dev_alloc_skb(RX_MON_STATUS_BUF_SIZE);',
                 'if (!IS_ALIGNED((unsigned long)skb->data, RX_MON_STATUS_BUF_ALIGN)) {',
                 'buf_id = idr_alloc(&rx_ring->bufs_idr, skb, 0, rx_ring->bufs_max * 3, GFP_ATOMIC);',
                 'cookie = u32_encode_bits(buf_id, DP_RXDMA_BUF_COOKIE_BUF_ID);',
                 'ath12k_hal_rx_buf_addr_info_set(desc, paddr, cookie, mgr);', 'ath12k_hal_srng_access_end(ab, srng);'):
        assert fact in rp, fact
    assert 'return ((srng->ring_size - hp + tp) / srng->entry_size) - 1;' in hal_c
    entry_dw = 2  # hal_wbm_buffer_ring: one ath12k_buffer_addr (two __le32, compiled in l19_c2c)
    assert re.search(r'struct hal_wbm_buffer_ring \{\s+struct ath12k_buffer_addr buf_addr_info;\s+\}', desc) and \
        re.search(r'struct ath12k_buffer_addr \{\s+__le32 info0;\s+__le32 info1;\s+\}', desc)
    asked = (1024 * entry_dw * 4 + num('HAL_RING_BASE_ALIGN', hal_h) - 1) // (entry_dw * 4)
    bufs_max, num_free = asked, (1024 * entry_dw) // entry_dw - 1  # a fresh ring: hp = tp = 0, ring_size in words
    nbuf = min(min(asked, bufs_max), num_free)
    assert (asked, nbuf) == (1024, H['WLMS_NBUF']) == (1024, 1023), (asked, nbuf)
    ck = field('DP_RXDMA_BUF_COOKIE_BUF_ID', dp_h)
    assert ck == (0, (1 << 18) - 1) and nbuf - 1 <= ck[1] and field('DP_RXDMA_BUF_COOKIE_PDEV_ID', dp_h) == (18, 3)
    # The buffer: dev_alloc_skb(RX_MON_STATUS_BUF_SIZE), 128-aligned, inside the RX_MON_STATUS_BASE_BUF_SIZE budget of
    # 2048 bytes (what is left after the reservation, the alignment and the skb tail); we post the whole budget.
    assert re.search(r'#define RX_MON_STATUS_BUF_SIZE\s+\(RX_MON_STATUS_BASE_BUF_SIZE - \\\s+\(RX_MON_STATUS_BUF_RESERVATION \+ \\\s+'
                     r'RX_MON_STATUS_BUF_ALIGN \+ \\\s+SKB_DATA_ALIGN\(sizeof\(struct skb_shared_info\)\)\)\)', dp_h)
    assert num('RX_MON_STATUS_BASE_BUF_SIZE', dp_h) == H['WLMS_BUF_BYTES'] == 2048 and num('RX_MON_STATUS_BUF_ALIGN', dp_h) == 128
    assert H['WLMS_SLOT_BYTES'] % 128 == 0 and H['WLMS_OFF_BUFS'] % 128 == 0 and H['WLMS_SLOT_BYTES'] == 2 * H['WLMS_BUF_BYTES']
    # HTT: RXDMA_MONITOR_STATUS ring id and type, the announcement order (ath12k_dp_rx_htt_setup).
    rid, rtype = enum(dp_h, 'htt_srng_ring_id'), enum(dp_h, 'htt_srng_ring_type')
    assert rid['HTT_RXDMA_MONITOR_STATUS_RING'] == H['WLMS_HTT_RING_MON_STATUS'] == 1
    typ = flat(fn_body(dp_tx_c, 'ath12k_dp_tx_get_ring_id_type('))
    assert 'case HAL_RXDMA_MONITOR_STATUS: *htt_ring_id = HTT_RXDMA_MONITOR_STATUS_RING; *htt_ring_type = HTT_SW_TO_HW_RING; break;' in typ
    hs = flat(fn_body(dp_rx_c, 'int ath12k_dp_rx_htt_setup('))
    mon = 'ret = ath12k_dp_tx_htt_srng_setup(ab, ring_id, i, HAL_RXDMA_MONITOR_STATUS);'
    assert hs.index('ath12k_dp_tx_htt_srng_setup(ab, ring_id, i, HAL_RXDMA_DST);') < hs.index('if (ab->hw_params->rxdma1_enable) {') < \
        hs.index('} else {') < hs.index(mon) < hs.index('rxdma_ring_sel_config(ab)')
    assert 'ring_id = dp->rx_mon_status_refill_ring[i].refill_buf_ring.ring_id;' in hs
    # Thresholds (dp.c ath12k_dp_srng_setup): low = entries >> 3 with LOW_THRESH_INTR_EN, batch 1, timer RX (500 us);
    # the low threshold in words (hal.c).
    th = flat(fn_body(dp_c, 'int ath12k_dp_srng_setup('))
    assert 'case HAL_RXDMA_MONITOR_STATUS: params.low_threshold = num_entries >> 3; params.flags |= HAL_SRNG_FLAGS_LOW_THRESH_INTR_EN; ' \
        'params.intr_batch_cntr_thres_entries = 1; params.intr_timer_thres_us = HAL_SRNG_INT_TIMER_THRESHOLD_RX; break;' in th
    timer = num('HAL_SRNG_INT_TIMER_THRESHOLD_RX', hal_h)
    assert (timer, 1) == (H['WLMS_TIMER_US'], H['WLMS_BATCH']) == (500, 1)
    assert 'srng->u.src_ring.low_threshold = params->low_threshold * srng->entry_size;' in fc
    # L19 STA mode: no reap without a monitor vdev, no ring selection for the status rings outside monitor mode.
    pr = flat(fn_body(dp_mon_c, 'int ath12k_dp_mon_process_ring('))
    assert '} else { if (ar->monitor_started) num_buffs_reaped = __ath12k_dp_mon_process_ring(ar, mac_id, napi, &budget); }' in pr
    assert flat(dp_tx_c).count('HAL_RXDMA_MONITOR_STATUS, RX_MON_STATUS_BUF_SIZE,') == 1 and \
        'HAL_RXDMA_MONITOR_STATUS, RX_MON_STATUS_BUF_SIZE,' in flat(fn_body(dp_tx_c, 'int ath12k_dp_tx_htt_rx_monitor_mode_ring_config('))
    assert 'if (!ab->hw_params->rxdma1_enable) return ret;' in flat(fn_body(mac_c, 'static int ath12k_mac_config_mon_status_default('))
    # Every word of the model's dump (RX arena at A = 0xa1400000, monitor status arena at M = 0xa2e00000).
    words = {}
    for line in dump.splitlines():
        k, *v = line.split()
        words.setdefault(k, []).append([int(x) for x in v])
    A, M = 0xa1400000, 0xa2e00000
    assert '#define WLMS_OFF_RING(m) (0x4000u * (m))' in HEADER
    f = {k: field('HTT_SRNG_SETUP_CMD_' + k, dp_h) for k in ('INFO0_MSG_TYPE', 'INFO0_PDEV_ID', 'INFO0_RING_ID', 'INFO0_RING_TYPE',
                                                           'INFO1_RING_SIZE', 'INFO1_RING_ENTRY_SIZE', 'INFO1_RING_LOOP_CNT_DIS',
                                                           'INTR_INFO_BATCH_COUNTER_THRESH', 'INTR_INFO_INTR_TIMER_THRESH',
                                                           'INFO2_INTR_LOW_THRESH')}
    enc = lambda k, v: (v & f[k][1]) << f[k][0]  # noqa: E731
    sw1 = rbm['HAL_RX_BUF_RBM_SW1_BM']
    for m in (0, 1):
        rid_ = ids[m]
        want = [enc('INFO0_MSG_TYPE', enum(dp_h, 'htt_h2t_msg_type')['HTT_H2T_MSG_TYPE_SRING_SETUP']) |
                enc('INFO0_PDEV_ID', m + 1) | enc('INFO0_RING_ID', rid['HTT_RXDMA_MONITOR_STATUS_RING']) |
                enc('INFO0_RING_TYPE', rtype['HTT_SW_TO_HW_RING']),
                M + 0x4000 * m, 0,
                enc('INFO1_RING_ENTRY_SIZE', entry_dw) | enc('INFO1_RING_SIZE', 1024 * entry_dw) | enc('INFO1_RING_LOOP_CNT_DIS', 1),
                A + H['WLRX_OFF_WRP'] + 4 * (rid_ - d0), 0, A + H['WLRX_OFF_RDP'] + 4 * rid_, 0,
                0, 0, 0,  # MSI address and data: L19's MSI, none here
                enc('INTR_INFO_BATCH_COUNTER_THRESH', 1 * entry_dw) | enc('INTR_INFO_INTR_TIMER_THRESH', timer >> 3),
                enc('INFO2_INTR_LOW_THRESH', (1024 >> 3) * entry_dw)]
        got = [v for _, v in sorted((k, v) for k, v in words[f'msring{m}'])]
        assert got == want, (m, [(i, hex(a), hex(b)) for i, (a, b) in enumerate(zip(got, want)) if a != b])
        b = M + H['WLMS_OFF_BUFS'] + 1024 * 4096 * m
        assert words[f'mentry{m}'] == [[0, b, sw1 << 8], [1022, b + 1022 * 4096, sw1 << 8 | 1022 << 12]], words[f'mentry{m}']
        assert words[f'mring{m}'] == [[rid_, 1024, entry_dw]]
    assert words['mnbuf'] == [[nbuf]] and words['mbufbytes'] == [[2048]] and words['marena'] == [[H['WLMS_ARENA_BYTES']]]
    print('L19 re-derivation (C2e): monitor status ring ids 185/193 (STATBUF + PMAC stride), 1024 entries of '
          'hal_wbm_buffer_ring (2 words), source rings, allocated after the error ring and filled after the refill ring with '
          '1023 buffers each (size / entry size, min with num_free), RBM SW1 6, cookie = idr id from 0 in 17:0, the 2048-byte '
          'RX_MON_STATUS_BASE_BUF_SIZE budget at 128 alignment, HTT RXDMA_MONITOR_STATUS 1 / SW_TO_HW, announced after the '
          'error ring and before the ring selection, batch 1, 500 us, low 128 entries in words, L19\'s MSI for them (ours 0), '
          'no reap and no status-ring selection in STA mode; the model\'s dump word by word (2 x 13 SRING_SETUP words: '
          '0x0101010b / 0x0101020b, 0x02020800, 0x003e0002, low 256; entries 0 and 1022)')


def l19_c2f(dump):
    """C2f: cookie conversion (dp.c cc_config/cc_init, dp.h CMEM/PPT/SPT constants, hal.h cookie cfg masks, hw.c wbm2sw
    convert), the common SRNGs of ath12k_dp_srng_common_setup (hal.h/hw.c reg bases, dp.h ring sizes, hal_desc.h entry
    sizes, dp.c thresholds, src/dst hw-init word encodings) and the mac-start / encap-decap enum values, re-derived from
    L19 and compared with the model's dump word by word."""
    if not (L19 / 'wmi.h').exists():
        print(f'L19 re-derivation (C2f) SKIPPED: no Linux ath12k v6.19 tree at {L19} (set ATH12K_SRC)')
        return
    hal_h, hal_c, hw_c, dp_h, dp_c, desc, wmi_h, mac_c, dp_tx_c, core_c = (l19_text(n) for n in (
        'hal.h', 'hal.c', 'hw.c', 'dp.h', 'dp.c', 'hal_desc.h', 'wmi.h', 'mac.c', 'dp_tx.c', 'core.c'))
    H = hdefs()

    def enum(text, name):
        body = re.search(r'enum\s+' + name + r'\s*\{(.*?)\};', text, re.S)[1]
        out, cur = {}, -1
        for item in body.split(','):
            item = re.sub(r'/\*.*?\*/', '', item, flags=re.S).strip()
            if not item:
                continue
            if '=' in item:
                k, v = (x.strip() for x in item.split('=', 1))
                cur = out[v] if v in out else int(v, 0)
            else:
                k, cur = item, cur + 1
            out[k] = cur
        return out

    def num(name, text):
        return int(re.search(r'#define ' + name + r'\s+(0x[0-9a-fA-F]+|\d+)', text)[1], 0)

    def field(name, text):
        v = re.search(r'#define ' + name + r'\s+(.+?)\s*$', text, re.M)[1]
        m = re.fullmatch(r'GENMASK\((\d+), (\d+)\)', v)
        if m:
            return int(m[2]), ((1 << (int(m[1]) - int(m[2]) + 1)) - 1)
        return int(re.fullmatch(r'BIT\((\d+)\)', v)[1]), 1

    def regval(name):   # a wcn7850_regs field, e.g. hal_reo1_ring_base_lsb
        reg = hw_c.split('wcn7850_regs = {', 1)[1].split('\n};', 1)[0]
        return int(re.search(r'\.' + name + r'\s*=\s*(0x[0-9a-fA-F]+)', reg)[1], 0)

    def struct_dwords(name):
        """sizeof(struct NAME) >> 2, by walking its fields (le32 1, le64 2, nested structs recurse, arrays multiply)."""
        body = re.search(r'struct ' + name + r' \{(.*?)\n\} __packed;', desc, re.S)[1]
        base = {'__le32': 1, '__le64': 2, 'u8': None}
        total = 0
        for line in body.split('\n'):
            line = re.sub(r'/\*.*?\*/', '', line).strip().rstrip(';')
            if not line:
                continue
            if re.match(r'\w+\s+\w+\[\]$', line):
                continue  # flexible array member (TLV payload): size 0
            mt = re.match(r'struct (\w+)\s+(\w+)(?:\[(\d+)\])?$', line)
            if mt:
                total += struct_dwords(mt[1]) * (int(mt[3]) if mt[3] else 1)
                continue
            m = re.match(r'(\w+)\s+(\w+)(?:\[(\d+)\])?$', line)
            assert m and m[1] in base and base[m[1]], line
            total += base[m[1]] * (int(m[3]) if m[3] else 1)
        return total

    words = {}
    for line in dump.splitlines():
        k, *v = line.split()
        words.setdefault(k, []).append([int(x) for x in v])

    REO, WBM, TCL = 0xa38000, 0xa34000, 0xa44000
    assert num('HAL_SEQ_WCSS_UMAC_REO_REG', hal_h) == REO and num('HAL_SEQ_WCSS_UMAC_WBM_REG', hal_h) == WBM and \
        num('HAL_SEQ_WCSS_UMAC_TCL_REG', hal_h) == TCL
    # Cookie conversion cc_config values (dp.c:1330-1380; hal.h masks; dp.h constants; hw.c wbm2sw_cc_enable).
    cmem_msb, ppt_msb, spt_msb = num('ATH12K_CMEM_ADDR_MSB', dp_h), num('ATH12K_CC_PPT_MSB', dp_h), num('ATH12K_CC_SPT_MSB', dp_h)
    def enc(text, pfx, pairs):
        v = 0
        for name, x in pairs:
            sh, mask = field(pfx + name, text)
            v |= (x & mask) << sh
        return v
    reo_cfg1 = enc(hal_h, 'HAL_REO1_SW_COOKIE_CFG_', [('CMEM_BASE_ADDR_MSB', cmem_msb), ('COOKIE_PPT_MSB', ppt_msb),
                   ('COOKIE_SPT_MSB', spt_msb), ('ALIGN', 1), ('ENABLE', 1), ('GLOBAL_ENABLE', 1)])
    wbm_cfg1 = enc(hal_h, 'HAL_WBM_SW_COOKIE_CFG_', [('CMEM_BASE_ADDR_MSB', cmem_msb), ('COOKIE_PPT_MSB', ppt_msb),
                   ('COOKIE_SPT_MSB', spt_msb), ('ALIGN', 1)])
    cfg2_or = enc(hal_h, 'HAL_WBM_SW_COOKIE_CFG_', [('RELEASE_PATH_EN', 1), ('ERR_PATH_EN', 1), ('CONV_IND_EN', 1)])
    conv_wcn = hw_c.split('ath12k_hw_hal_params_wcn7850 = {', 1)[1].split('};', 1)[0]
    conv_bits = ['HAL_WBM_SW_COOKIE_CONV_CFG_GLOBAL_EN'] + re.findall(r'HAL_WBM_SW_COOKIE_CONV_CFG_WBM2SW\d_EN', conv_wcn)
    conv_or = 0
    for b in conv_bits:
        sh, _ = field(b, hal_h)
        conv_or |= 1 << sh
    assert words['ccv'][0] == [reo_cfg1, wbm_cfg1, cfg2_or, conv_or] == [0x1d1310, 0x51310, 0xb, 0x13a], words['ccv'][0]
    # cc_config register addresses: REO cookie cfg0/1 from wcn7850_regs, WBM ones from hal.h offsets.
    reo_cfg0 = int(re.search(r'hal_reo1_sw_cookie_cfg0 = (0x[0-9a-f]+)', hw_c)[1], 0)
    reo_cfg1a = int(re.search(r'hal_reo1_sw_cookie_cfg1 = (0x[0-9a-f]+)', hw_c)[1], 0)
    cc_exp = [[REO + reo_cfg0, 0x113000], [REO + reo_cfg1a, reo_cfg1],
              [WBM + num('HAL_WBM_SW_COOKIE_CFG0', hal_h), 0x113000], [WBM + num('HAL_WBM_SW_COOKIE_CFG1', hal_h), wbm_cfg1],
              [WBM + num('HAL_WBM_SW_COOKIE_CFG2', hal_h), cfg2_or], [WBM + num('HAL_WBM_SW_COOKIE_CONVERT_CFG', hal_h), conv_or]]
    assert [c[1:] for c in words['ccreg']] == cc_exp, (words['ccreg'], cc_exp)
    # SPT / PPT / cookie math (dp.h); DEFAULT profile rx_desc 12288, pool_tx 32768 (core.c).
    prof = core_c.split('ATH12K_QMI_MEMORY_MODE_DEFAULT] = {', 1)[1].split('},', 1)[0]
    rx_desc = int(re.search(r'rx_desc_count = (\d+)', prof)[1])
    pool_tx = int(re.search(r'num_pool_tx_desc = (\d+)', prof)[1])
    spt_ent = num('ATH12K_MAX_SPT_ENTRIES', dp_h)
    nrx, ntx = rx_desc // spt_ent, (pool_tx // spt_ent) * num('ATH12K_HW_MAX_QUEUES', l19_text('hw.h'))
    assert words['arena'][0][2:] == [nrx + ntx, nrx, ntx] == [280, 24, 256], words['arena'][0]
    ppt_shift = num('ATH12K_CC_PPT_SHIFT', dp_h)
    for k, cookie, slot in words['cookie']:
        page, spt = k // spt_ent, k % spt_ent
        assert cookie == ((ntx + page) << ppt_shift | spt) and slot == k, (k, cookie, slot)
    # PPT entry: cmem_base + 4*i, value = spt page paddr >> 12.
    spt_iova = 0xa5000000 + H['WLCM_PAGE_BYTES']
    for i, addr, v in words['ppt']:
        assert addr == 0x113000 + 4 * i and v == (spt_iova + i * H['WLCM_PAGE_BYTES']) >> num('ATH12K_SPT_4K_ALIGN_OFFSET', dp_h)
    # Common rings: ids, R0/R2 bases, sizes, entry dwords, thresholds, and the src/dst hw-init word encodings.
    reo1_lsb, reo2_lsb = regval('hal_reo1_ring_base_lsb'), regval('hal_reo2_ring_base')
    tcl1_lsb, tcl2_lsb = regval('hal_tcl1_ring_base_lsb'), regval('hal_tcl2_ring_base_lsb')
    wbm0_lsb, wbm1_lsb = regval('hal_wbm0_release_ring_base_lsb'), regval('hal_wbm1_release_ring_base_lsb')
    tcl_hp, tcl_hp2 = num('HAL_TCL1_RING_HP', hal_h), num('HAL_TCL2_RING_HP', hal_h)
    wbm0_hp = num('HAL_WBM0_RELEASE_RING_HP', hal_h)
    comp_nums = [int(x) for x in re.findall(r'\.wbm_ring_num = (\d+)', hal_c.split('ath12k_hal_wcn7850_tcl_to_wbm_rbm_map', 1)[1].split('};', 1)[0])]
    assert comp_nums == [0, 2, 4], comp_nums
    tx_comp = int(re.search(r'tx_comp_ring_size = (\d+)', prof)[1])
    batch_tx, timer_tx = num('HAL_SRNG_INT_BATCH_THRESHOLD_TX', hal_h), num('HAL_SRNG_INT_TIMER_THRESHOLD_TX', hal_h)
    batch_o, timer_o = num('HAL_SRNG_INT_BATCH_THRESHOLD_OTHER', hal_h), num('HAL_SRNG_INT_TIMER_THRESHOLD_OTHER', hal_h)
    dw_tcl, dw_wbm = struct_dwords('hal_tcl_data_cmd'), struct_dwords('hal_wbm_release_ring')
    dw_reinj = struct_dwords('hal_reo_entrance_ring')
    dw_cmd = struct_dwords('hal_tlv_64_hdr') + struct_dwords('hal_reo_get_queue_stats')
    dw_stat = struct_dwords('hal_tlv_64_hdr') + struct_dwords('hal_reo_get_queue_stats_status')
    exp = {  # id: (dir, entries, entry_dw, r0, r2, batch, timer)
        121: (0, num('DP_WBM_RELEASE_RING_SIZE', dp_h), dw_wbm, WBM + regval('hal_wbm_sw_release_ring_base_lsb'),
              WBM + num('HAL_WBM_SW_RELEASE_RING_HP', hal_h), batch_o, timer_o),
        16: (0, num('DP_REO_REINJECT_RING_SIZE', dp_h), dw_reinj, REO + regval('hal_sw2reo_ring_base'),
             REO + num('HAL_SW2REO_RING_HP', hal_h), batch_o, timer_o),
        20: (0, num('DP_REO_CMD_RING_SIZE', dp_h), dw_cmd, REO + regval('hal_reo_cmd_ring_base'),
             REO + num('HAL_REO_CMD_HP', hal_h), batch_o, timer_o),
        21: (1, num('DP_REO_STATUS_RING_SIZE', dp_h), dw_stat, REO + regval('hal_reo_status_ring_base'),
             REO + num('HAL_REO_STATUS_HP', hal_h), batch_o, timer_o),
    }
    for i in range(3):
        exp[24 + i] = (0, num('DP_TCL_DATA_RING_SIZE', dp_h), dw_tcl, TCL + tcl1_lsb + (tcl2_lsb - tcl1_lsb) * i,
                       TCL + tcl_hp + (tcl_hp2 - tcl_hp) * i, batch_o, timer_o)
        cn = comp_nums[i]
        exp[128 + cn] = (1, tx_comp, dw_wbm, WBM + wbm0_lsb + (wbm1_lsb - wbm0_lsb) * cn,
                         WBM + wbm0_hp + 8 * cn, batch_tx, timer_tx)
    FL = {'BASE_LSB': 0x0, 'BASE_MSB': 0x4, 'ID': 0x8, 'MISC': 0x10, 'IX0': 0x30, 'IX1': 0x34, 'TP_LSB': 0x1c,
          'TP_MSB': 0x20, 'PROD_INT': 0x24, 'HP_LSB': 0x14, 'HP_MSB': 0x18}
    rdp = 0xa5000000
    ringn = {r[0]: r[1:] for r in words['ringn']}
    for rid, (d, ent, dw, r0, r2, batch, timer) in exp.items():
        assert ringn[rid] == [d, ent, dw, 10 if d == 0 else 9], (rid, ringn[rid])
        regs = {x[2]: x[3] for x in words['ring'] if x[0] == rid}
        iova = regs[r0 + FL['BASE_LSB']]
        msb = (iova >> 32 & 0xff) | (ent * dw & 0xfffff) << 8
        cons = ((timer >> 3) & 0xffff) << 16 | (batch * dw & 0x7fff)
        exp_regs = {r0 + FL['BASE_LSB']: iova, r0 + FL['BASE_MSB']: msb, r2: 0, r2 + 4: 0}
        if d == 0:
            exp_regs[r0 + FL['ID']] = dw
            exp_regs[r0 + FL['IX0']] = cons
            exp_regs[r0 + FL['IX1']] = 0
            exp_regs[r0 + FL['TP_LSB']] = (rdp + 4 * rid) & 0xffffffff
            exp_regs[r0 + FL['TP_MSB']] = (rdp + 4 * rid) >> 32
            exp_regs[r0 + FL['MISC']] = (1 << 1) | (1 << 6)
        else:
            exp_regs[r0 + FL['ID']] = (rid & 0xff) << 8 | dw
            exp_regs[r0 + FL['PROD_INT']] = cons
            exp_regs[r0 + FL['HP_LSB']] = (rdp + 4 * rid) & 0xffffffff
            exp_regs[r0 + FL['HP_MSB']] = (rdp + 4 * rid) >> 32
            exp_regs[r0 + FL['MISC']] = 1 << 6
        assert regs == exp_regs, (rid, regs, exp_regs)
    # mac-start pdev params, encap/decap and PPDU stats enum values.
    pdev = enum(wmi_h, 'wmi_tlv_pdev_param')
    vdev = enum(wmi_h, 'wmi_tlv_vdev_param')
    assert (pdev['WMI_PDEV_PARAM_PMF_QOS'], pdev['WMI_PDEV_PARAM_DYNAMIC_BW'], pdev['WMI_PDEV_PARAM_ARP_AC_OVERRIDE'],
            pdev['WMI_PDEV_PARAM_MESH_MCAST_ENABLE'], pdev['WMI_PDEV_PARAM_IDLE_PS_CONFIG']) == \
        (H['WLWMI_PDEV_PARAM_PMF_QOS'], H['WLWMI_PDEV_PARAM_DYNAMIC_BW'], H['WLWMI_PDEV_PARAM_ARP_AC_OVERRIDE'],
         H['WLWMI_PDEV_PARAM_MESH_MCAST_ENABLE'], H['WLWMI_PDEV_PARAM_IDLE_PS_CONFIG'])
    assert (vdev['WMI_VDEV_PARAM_TX_ENCAP_TYPE'], vdev['WMI_VDEV_PARAM_RX_DECAP_TYPE']) == \
        (H['WLWMI_VDEV_PARAM_TX_ENCAP_TYPE'], H['WLWMI_VDEV_PARAM_RX_DECAP_TYPE'])
    svc = enum(wmi_h, 'wmi_tlv_service')
    assert svc['WMI_TLV_SERVICE_ETH_OFFLOAD'] == H['WLWMI_SVC_ETH_OFFLOAD'] == 461
    txrx = enum(wmi_h, 'ath12k_hw_txrx_mode')
    assert txrx['ATH12K_HW_TXRX_NATIVE_WIFI'] == H['WLWMI_TXRX_NATIVE_WIFI'] and txrx['ATH12K_HW_TXRX_ETHERNET'] == H['WLWMI_TXRX_ETHERNET']
    ppdu_type = num('HTT_H2T_MSG_TYPE_PPDU_STATS_CFG', dp_h) if re.search(r'#define HTT_H2T_MSG_TYPE_PPDU_STATS_CFG', dp_h) else \
        enum(dp_h, 'htt_h2t_msg_type')['HTT_H2T_MSG_TYPE_PPDU_STATS_CFG']
    tag = enum(dp_h, 'htt_ppdu_stats_tag_type')
    default_bits = 0
    for b in re.findall(r'BIT\((HTT_PPDU_STATS_TAG_\w+)\)', dp_h.split('#define HTT_PPDU_STATS_TAG_DEFAULT', 1)[1].split('#define', 1)[0]):
        default_bits |= 1 << tag[b]
    assert words['ppdu'][0] == [ppdu_type, default_bits] == [0x11, 0xe67], words['ppdu'][0]
    print('L19 re-derivation (C2f): cc_config addresses and values (REO/WBM cookie cfg0/1/2, convert; CMEM_ADDR_MSB, PPT/SPT '
          'MSB, align/enable/global, wbm2sw0/2/3/4), the CMEM/PPT/SPT and cookie math (DEFAULT profile 24 rx + 256 tx pages, '
          'ppt << 9 | spt, page paddr >> 12), the ten common rings (ids 121/24-26/128-132/16/20/21, hal.h+hw.c R0/R2 bases, '
          'dp.h sizes with tx_comp 32768, hal_desc.h entry dwords 8/8/8/12/28, dp.c TX vs other thresholds, src/dst hw-init '
          'word encodings), and the mac-start/encap/PPDU enum values; the model\'s dump word by word')


def l19_c2d(define, dump):
    """C2d: RX_RING_SELECTION_CFG (dp.h enums, masks, struct, encoder, the WCN7850 filter, rx_desc.h offsets compiled),
    the REO/release rings (hal.h ids, dp.h sizes, hw.c registers, hal.c dst init order, dp.c thresholds and setup order,
    core.c placement), the REO setup (hal_rx.c, dp.c hash map), the entry fields and RBMs (hal_desc.h, hal.h, hal_rx.c,
    dp_rx.c): against the header, then every word of the model's dump."""
    hal_h, hal_c, hw_c, dp_h, dp_c, dp_tx_c, dp_rx_c, desc, hal_rx_c, core_c = (l19_text(n) for n in (
        'hal.h', 'hal.c', 'hw.c', 'dp.h', 'dp.c', 'dp_tx.c', 'dp_rx.c', 'hal_desc.h', 'hal_rx.c', 'core.c'))
    H = hdefs()
    flat = lambda t: ' '.join(t.replace('\\\n', ' ').split())  # noqa: E731

    def struct_fields(f, name):
        body = re.search(r'struct ' + name + r' \{(.*?)\}', re.sub(r'/\*.*?\*/', '', (L19 / f).read_text(), flags=re.S), re.S)[1]
        return [d.split()[-1] for d in (' '.join(x.split()) for x in body.split(';')) if d]

    def num(name, text):
        v = re.search(r'#define ' + name + r'\s+\(?([0-9x \*]+)\)?\s*$', text, re.M)[1]
        assert re.fullmatch(r'[0-9x \*]+', v), v
        return eval(v, {}, {})

    def bit(name, text):
        return 1 << int(re.search(r'\b' + name + r'\s*=?\s*BIT\((\d+)\)', text)[1])

    def mask(name, text):
        hi, lo = map(int, re.search(r'#define ' + name + r'\s+GENMASK\((\d+), (\d+)\)', text).groups())
        return ((1 << (hi + 1)) - 1) & ~((1 << lo) - 1)

    def enum(text, name):
        body = re.search(r'enum\s+' + name + r'\s*\{(.*?)\};', text, re.S)[1]
        out, cur = {}, -1
        for item in body.split(','):
            item = item.strip()
            if not item:
                continue
            if '=' in item:
                k, v = (x.strip() for x in item.split('=', 1))
                cur = out[v] if v in out else int(v, 0)
            else:
                k, cur = item, cur + 1
            out[k] = cur
        return out
    # -- ring selection
    assert enum(dp_h, 'htt_h2t_msg_type')['HTT_H2T_MSG_TYPE_RX_RING_SELECTION_CFG'] == H['WLRD_HTT_RX_RING_SEL'] == 0xc
    assert mask('HTT_RX_RING_SELECTION_CFG_CMD_INFO0_MSG_TYPE', dp_h) == 0xff and mask('HTT_RX_RING_SELECTION_CFG_CMD_INFO0_PDEV_ID', dp_h) == 0xff00
    assert mask('HTT_RX_RING_SELECTION_CFG_CMD_INFO0_RING_ID', dp_h) == 0xff0000
    assert bit('HTT_RX_RING_SELECTION_CFG_CMD_INFO0_OFFSET_VALID', dp_h) == H['WLRD_SEL_OFFSET_VALID']
    assert bit('HTT_RX_RING_SELECTION_CFG_CMD_INFO0_EN_RXMON', dp_h) == H['WLRD_SEL_EN_RXMON']
    assert mask('HTT_RX_RING_SELECTION_CFG_CMD_INFO1_BUF_SIZE', dp_h) == 0xffff
    assert mask('HTT_RX_RING_SELECTION_CFG_RX_PACKET_OFFSET', dp_h) == 0xffff and mask('HTT_RX_RING_SELECTION_CFG_RX_HEADER_OFFSET', dp_h) == 0xffff0000
    assert mask('HTT_RX_RING_SELECTION_CFG_RX_MPDU_START_OFFSET', dp_h) == 0xffff0000 and mask('HTT_RX_RING_SELECTION_CFG_RX_MSDU_END_OFFSET', dp_h) == 0xffff
    assert bit('HTT_RX_FP_CTRL_PKT_FILTER_TLV_FLAGS2_BAR', dp_h) == H['WLRD_SEL_FLAGS2']
    assert bit('HTT_RX_FP_DATA_PKT_FILTER_TLV_FLASG3_MCAST', dp_h) | bit('HTT_RX_FP_DATA_PKT_FILTER_TLV_FLASG3_UCAST', dp_h) | \
        bit('HTT_RX_FP_DATA_PKT_FILTER_TLV_FLASG3_NULL_DATA', dp_h) == H['WLRD_SEL_FLAGS3']
    fd = flat(dp_h)
    assert '#define HTT_RX_TLV_FLAGS_RXDMA_RING (HTT_RX_FILTER_TLV_FLAGS_MPDU_START | HTT_RX_FILTER_TLV_FLAGS_RX_PACKET | ' \
        'HTT_RX_FILTER_TLV_FLAGS_MSDU_END)' in fd
    assert bit('HTT_RX_FILTER_TLV_FLAGS_MPDU_START', dp_h) | bit('HTT_RX_FILTER_TLV_FLAGS_RX_PACKET', dp_h) | \
        bit('HTT_RX_FILTER_TLV_FLAGS_MSDU_END', dp_h) == H['WLRD_SEL_TLV']
    assert define('DP_RXDMA_REFILL_RING_SIZE', dp_h) == H['WLRD_SEL_BUF_SIZE']
    assert struct_fields('dp.h', 'htt_rx_ring_selection_cfg_cmd') == [
        'info0', 'info1', 'pkt_type_en_flags0', 'pkt_type_en_flags1', 'pkt_type_en_flags2', 'pkt_type_en_flags3', 'rx_filter_tlv',
        'rx_packet_offset', 'rx_mpdu_offset', 'rx_msdu_offset', 'rx_attn_offset', 'info2', 'reserved[2]', 'rx_mpdu_start_end_mask',
        'rx_msdu_end_word_mask', 'info3']
    sel = flat(fn_body(dp_rx_c, 'int ath12k_dp_rxdma_ring_sel_config_wcn7850('))
    for fact in ('tlv_filter.rx_filter = HTT_RX_TLV_FLAGS_RXDMA_RING;', 'tlv_filter.pkt_filter_flags2 = HTT_RX_FP_CTRL_PKT_FILTER_TLV_FLAGS2_BAR;',
                 'tlv_filter.pkt_filter_flags3 = HTT_RX_FP_DATA_PKT_FILTER_TLV_FLASG3_MCAST | HTT_RX_FP_DATA_PKT_FILTER_TLV_FLASG3_UCAST | '
                 'HTT_RX_FP_DATA_PKT_FILTER_TLV_FLASG3_NULL_DATA;', 'tlv_filter.offset_valid = true;',
                 'tlv_filter.rx_packet_offset = hal_rx_desc_sz;',
                 'tlv_filter.rx_header_offset = offsetof(struct hal_rx_desc_wcn7850, pkt_hdr_tlv);',
                 'for (i = 0; i < ab->hw_params->num_rxdma_per_pdev; i++) { ring_id = dp->rx_mac_buf_ring[i].ring_id; '
                 'ret = ath12k_dp_tx_htt_rx_filter_setup(ab, ring_id, i, HAL_RXDMA_BUF, DP_RXDMA_REFILL_RING_SIZE, &tlv_filter); }'):
        assert fact in sel, fact
    assert 'pkt_filter_flags0' not in sel and 'pkt_filter_flags1' not in sel and 'rxmon_disable' not in sel and 'wmask' not in sel
    assert 'return offsetof(struct hal_rx_desc_wcn7850, mpdu_start_tag);' in flat(hal_c) and \
        'return offsetof(struct hal_rx_desc_wcn7850, msdu_end_tag);' in flat(hal_c)
    assert 'return sizeof(struct hal_rx_desc_wcn7850);' in flat(hal_c)
    enc = flat(fn_body(dp_tx_c, 'int ath12k_dp_tx_htt_rx_filter_setup('))
    for fact in ('if (htt_ring_type == HTT_SW_TO_HW_RING || htt_ring_type == HTT_HW_TO_SW_RING) cmd->info0 |= '
                 'le32_encode_bits(DP_SW2HW_MACID(mac_id), HTT_RX_RING_SELECTION_CFG_CMD_INFO0_PDEV_ID);',
                 'cmd->info0 |= le32_encode_bits(!tlv_filter->rxmon_disable, HTT_RX_RING_SELECTION_CFG_CMD_INFO0_EN_RXMON);',
                 'cmd->info1 = le32_encode_bits(rx_buf_size, HTT_RX_RING_SELECTION_CFG_CMD_INFO1_BUF_SIZE);',
                 'if (tlv_filter->rx_mpdu_start_wmask > 0 && tlv_filter->rx_msdu_end_wmask > 0) {'):
        assert fact in enc, fact
    rt = flat(fn_body(dp_tx_c, 'ath12k_dp_tx_get_ring_id_type('))
    assert 'if (ring_id == HAL_SRNG_SW2RXDMA_BUF0) { *htt_ring_id = HTT_HOST1_TO_FW_RXBUF_RING; *htt_ring_type = HTT_SW_TO_SW_RING; } ' \
        'else { *htt_ring_id = HTT_RXDMA_HOST_BUF_RING; *htt_ring_type = HTT_SW_TO_HW_RING; }' in rt
    assert enum(dp_h, 'htt_srng_ring_id')['HTT_RXDMA_HOST_BUF_RING'] == 0 and '#define DP_SW2HW_MACID(mac_id) ((mac_id) + 1)' in dp_h
    hs = flat(fn_body(dp_rx_c, 'int ath12k_dp_rx_htt_setup('))
    assert hs.index('HAL_RXDMA_DST);') < hs.index('rxdma_ring_sel_config(ab)')
    src = ('#include <stdio.h>\n#include <stddef.h>\n#include <stdint.h>\ntypedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32;\n'
           'typedef uint16_t __le16; typedef uint32_t __le32; typedef uint64_t __le64;\n#define __packed __attribute__((packed))\n'
           '#define BIT(n) (1u << (n))\n#define GENMASK(h, l) (((~0u) << (l)) & (~0u >> (31 - (h))))\n#define ETH_ALEN 6\n'
           '#include "rx_desc.h"\n'
           'int main(void) { printf("%zu %zu %zu %zu\\n", sizeof(struct hal_rx_desc_wcn7850), offsetof(struct hal_rx_desc_wcn7850, '
           'pkt_hdr_tlv), offsetof(struct hal_rx_desc_wcn7850, mpdu_start_tag), offsetof(struct hal_rx_desc_wcn7850, msdu_end_tag)); '
           'return 0; }\n')
    with tempfile.TemporaryDirectory(prefix='fuxi-c2d-rxdesc-') as tmp:
        (Path(tmp) / 'o.c').write_text(src)
        subprocess.run([CC[0], '-std=gnu11', '-I', str(L19), '-o', str(Path(tmp) / 'o'), str(Path(tmp) / 'o.c')], check=True)
        got = list(map(int, subprocess.run([str(Path(tmp) / 'o')], check=True, capture_output=True, text=True).stdout.split()))
    assert got == [H['WLRD_DESC_BYTES'], H['WLRD_OFF_PKT_HDR'], H['WLRD_OFF_MPDU_START'], H['WLRD_OFF_MSDU_END']], got
    want = {}
    for m in range(2):
        w = [0] * 17
        w[0] = 0xc | (m + 1) << 8 | bit('HTT_RX_RING_SELECTION_CFG_CMD_INFO0_OFFSET_VALID', dp_h) | \
            bit('HTT_RX_RING_SELECTION_CFG_CMD_INFO0_EN_RXMON', dp_h)
        w[1], w[4], w[5], w[6] = define('DP_RXDMA_REFILL_RING_SIZE', dp_h), H['WLRD_SEL_FLAGS2'], H['WLRD_SEL_FLAGS3'], H['WLRD_SEL_TLV']
        w[7], w[8], w[9] = got[0] | got[1] << 16, got[2] << 16, got[3]
        for k in range(17):
            want[f'sel{m} {k}'] = w[k]
    assert (want['sel0 0'], want['sel1 0'], want['sel0 7'], want['sel0 8']) == (0x1400010c, 0x1400020c, 0x01100190, 0x00900000)
    # -- the REO and release rings
    ring = enum(hal_h, 'hal_srng_ring_id')
    rel_num = define('HAL_WBM2SW_REL_ERR_RING_NUM', hal_h)
    ids = [ring['HAL_SRNG_RING_ID_WBM2SW0_RELEASE'] + rel_num, ring['HAL_SRNG_RING_ID_REO2SW0']] + \
        [ring['HAL_SRNG_RING_ID_REO2SW1'] + n for n in range(8)]
    assert ids == [H['WLRD_RING_REL'], H['WLRD_RING_EXC']] + [H['WLRD_RING_DST0'] + n for n in range(8)] and ids[0] == 131, ids
    fc = flat(hal_c)
    for t, start in (('HAL_REO_DST', 'HAL_SRNG_RING_ID_REO2SW1'), ('HAL_REO_EXCEPTION', 'HAL_SRNG_RING_ID_REO2SW0'),
                     ('HAL_WBM2SW_RELEASE', 'HAL_SRNG_RING_ID_WBM2SW0_RELEASE')):
        body = re.search(r'\[' + t + r'\] = \{(.*?)\},', fc)[1]
        assert f'.start_ring_id = {start},' in body + ',' and '.ring_dir = HAL_SRNG_DIR_DST' in body and \
            '.mac_type = ATH12K_HAL_SRNG_UMAC' in body, t
    for ours, theirs in (('WLRD_DST_ENTRIES', 'DP_REO_DST_RING_SIZE'), ('WLRD_EXC_ENTRIES', 'DP_REO_EXCEPTION_RING_SIZE'),
                         ('WLRD_REL_ENTRIES', 'DP_RX_RELEASE_RING_SIZE'), ('WLRD_NDST', 'DP_REO_DST_RING_MAX')):
        assert H[ours] == define(theirs, dp_h), ours
    raw = re.sub(r'/\*.*?\*/', '', (L19 / 'hal_desc.h').read_text(), flags=re.S)
    structs = ['ath12k_buffer_addr', 'rx_mpdu_desc', 'rx_msdu_desc', 'rx_msdu_ext_desc', 'hal_reo_dest_ring', 'hal_wbm_release_ring',
               'hal_desc_header', 'hal_rx_msdu_details', 'hal_rx_msdu_link']
    src = '#include <stdio.h>\n#include <stdint.h>\ntypedef uint32_t __le32;\n#define __packed __attribute__((packed))\n'
    for s_ in structs:
        src += re.search(r'struct ' + s_ + r' \{.*?\}(?: __packed)?;', raw, re.S)[0] + '\n'
    src += 'int main(void) { printf("%zu %zu %zu\\n", sizeof(struct hal_reo_dest_ring), sizeof(struct hal_wbm_release_ring), ' \
        'sizeof(struct hal_rx_msdu_link)); return 0; }\n'
    with tempfile.TemporaryDirectory(prefix='fuxi-c2d-sizes-') as tmp:
        (Path(tmp) / 's.c').write_text(src)
        subprocess.run([CC[0], '-std=gnu11', '-o', str(Path(tmp) / 's'), str(Path(tmp) / 's.c')], check=True)
        sz = list(map(int, subprocess.run([str(Path(tmp) / 's')], check=True, capture_output=True, text=True).stdout.split()))
    assert sz == [4 * H['WLRD_ENTRY_DW'], 4 * H['WLRD_ENTRY_DW'], H['WLRX_LINK_DESC_BYTES']], sz
    assert struct_fields('hal_desc.h', 'hal_reo_dest_ring') == ['buf_addr_info', 'rx_mpdu_info', 'rx_msdu_info', 'buf_va_lo', 'buf_va_hi', 'info0']
    assert struct_fields('hal_desc.h', 'hal_wbm_release_ring') == ['buf_addr_info', 'info0', 'info1', 'info2', 'info3', 'info4', 'info5']
    assert struct_fields('hal_desc.h', 'hal_rx_msdu_link') == ['desc_hdr', 'buf_addr_info', 'info0', 'pn[4]', 'msdu_link[6]']
    assert 1 + 2 + 1 + 4 == H['WLRD_LINK_MSDU_W0'] and define('HAL_NUM_RX_MSDUS_PER_LINK_DESC', hal_h) == H['WLRD_LINK_MSDUS']
    assert bit('HAL_REO_DEST_RING_INFO0_BUFFER_TYPE', desc) == 1 and mask('HAL_REO_DEST_RING_INFO0_PUSH_REASON', desc) == 0x6 and \
        mask('HAL_REO_DEST_RING_INFO0_ERROR_CODE', desc) == 0xf8
    assert enum(desc, 'hal_reo_dest_ring_push_reason')['HAL_REO_DEST_RING_PUSH_REASON_ROUTING_INSTRUCTION'] == H['WLRD_PUSH_ROUTING']
    assert enum(desc, 'hal_reo_dest_ring_buffer_type')['HAL_REO_DEST_RING_BUFFER_TYPE_LINK_DESC'] == 1
    assert bit('RX_MSDU_DESC_INFO0_MSDU_CONTINUATION', desc) == 4
    assert mask('HAL_WBM_RELEASE_INFO0_REL_SRC_MODULE', desc) == 0x7 and mask('HAL_WBM_RELEASE_INFO0_DESC_TYPE', desc) == 0x1c0
    assert mask('HAL_WBM_RELEASE_INFO0_RXDMA_ERROR_CODE', desc) == 0x1f << 19 and mask('HAL_WBM_RELEASE_INFO0_REO_ERROR_CODE', desc) == 0x1f << 26
    src_mod = enum(desc, 'hal_wbm_rel_src_module')
    assert (src_mod['HAL_WBM_REL_SRC_MODULE_RXDMA'], src_mod['HAL_WBM_REL_SRC_MODULE_REO']) == (1, 2)
    assert enum(desc, 'hal_wbm_rel_desc_type')['HAL_WBM_REL_DESC_TYPE_REL_MSDU'] == 0
    rbm = enum(hal_h, 'hal_rx_buf_return_buf_manager')
    assert rbm['HAL_RX_BUF_RBM_SW3_BM'] == H['WLRD_RBM_SW3'] and rbm['HAL_RX_BUF_RBM_SW1_BM'] == H['WLRX_RBM_SW1']
    pe = flat(fn_body(hal_rx_c, 'int ath12k_hal_wbm_desc_parse_err('))
    assert 'if (type != HAL_WBM_REL_DESC_TYPE_REL_MSDU) {' in pe and 'if (rel_src != HAL_WBM_REL_SRC_MODULE_RXDMA && rel_src != ' \
        'HAL_WBM_REL_SRC_MODULE_REO) return -EINVAL;' in pe and 'if (val != HAL_RX_BUF_RBM_SW3_BM) {' in pe
    pr = flat(fn_body(dp_rx_c, 'int ath12k_dp_rx_process_err('))
    assert 'if (rbm != partner_ab->dp.idle_link_rbm && rbm != HAL_RX_BUF_RBM_SW3_BM && rbm != ' \
        'partner_ab->hw_params->hal_params->rx_buf_rbm) {' in pr
    li = flat(fn_body(hal_rx_c, 'void ath12k_hal_rx_msdu_link_info_get('))
    assert 'msdu = &link->msdu_link[0]; *rbm = le32_get_bits(msdu->buf_addr_info.info1, BUFFER_ADDR_INFO1_RET_BUF_MGR);' in li and \
        'if (val == 0) { *num_msdus = i; break; }' in li
    # WCN7850 registers (hw.c wcn7850_regs) and the R2 groups (hal.h)
    wcn = hw_c.split('static const struct ath12k_hw_regs wcn7850_regs = {', 1)[1].split('};', 1)[0]
    reg = {k: int(v, 16) for k, v in re.findall(r'\.(\w+) = (0x[0-9a-fA-F]+)', wcn)}
    reo, wbm = define('HAL_SEQ_WCSS_UMAC_REO_REG', hal_h), define('HAL_SEQ_WCSS_UMAC_WBM_REG', hal_h)
    r1 = reg['hal_reo1_ring_base_lsb']
    stride = reg['hal_reo2_ring_base'] - r1
    wstride = reg['hal_wbm1_release_ring_base_lsb'] - reg['hal_wbm0_release_ring_base_lsb']
    r2 = {'HAL_REO1_RING_HP': define('HAL_REO1_RING_HP', hal_h), 'HAL_REO2_RING_HP': define('HAL_REO2_RING_HP', hal_h)}
    tpoff = define('HAL_REO1_RING_TP', hal_h) - r2['HAL_REO1_RING_HP']
    r0s = [wbm + reg['hal_wbm0_release_ring_base_lsb'] + rel_num * wstride, reo + reg['hal_reo2_sw0_ring_base']] + \
        [reo + r1 + n * stride for n in range(8)]
    r2s = [wbm + define('HAL_WBM0_RELEASE_RING_HP', hal_h) + rel_num * (define('HAL_WBM1_RELEASE_RING_HP', hal_h) -
           define('HAL_WBM0_RELEASE_RING_HP', hal_h)), reo + define('HAL_REO_SW0_RING_HP', hal_h)] + \
        [reo + r2['HAL_REO1_RING_HP'] + n * (r2['HAL_REO2_RING_HP'] - r2['HAL_REO1_RING_HP']) for n in range(8)]
    off = {'base_lsb': 0, 'base_msb': reg['hal_reo1_ring_base_msb'] - r1, 'id': reg['hal_reo1_ring_id'] - r1,
           'misc': reg['hal_reo1_ring_misc'] - r1, 'hp_lsb': reg['hal_reo1_ring_hp_addr_lsb'] - r1,
           'hp_msb': reg['hal_reo1_ring_hp_addr_msb'] - r1, 'prod': reg['hal_reo1_ring_producer_int_setup'] - r1}
    assert (off['base_msb'], off['id'], off['misc'], off['hp_lsb'], off['hp_msb'], off['prod']) == \
        (H['WLCE_R0_BASE_MSB'], H['WLCE_R0_ID'], H['WLCE_R0_MISC'], H['WLCE_R0_HP_ADDR_LSB'], H['WLCE_R0_HP_ADDR_MSB'], H['WLCE_R0_PROD_INT'])
    wc = flat(fn_body(hal_c, 'static int ath12k_hal_srng_create_config_wcn7850('))
    for fact in ('s = &hal->srng_config[HAL_REO_DST]; s->reg_start[0] = HAL_SEQ_WCSS_UMAC_REO_REG + HAL_REO1_RING_BASE_LSB(ab); '
                 's->reg_start[1] = HAL_SEQ_WCSS_UMAC_REO_REG + HAL_REO1_RING_HP; s->reg_size[0] = HAL_REO2_RING_BASE_LSB(ab) - '
                 'HAL_REO1_RING_BASE_LSB(ab); s->reg_size[1] = HAL_REO2_RING_HP - HAL_REO1_RING_HP;',
                 's = &hal->srng_config[HAL_REO_EXCEPTION]; s->reg_start[0] = HAL_SEQ_WCSS_UMAC_REO_REG + HAL_REO_SW0_RING_BASE_LSB(ab); '
                 's->reg_start[1] = HAL_SEQ_WCSS_UMAC_REO_REG + HAL_REO_SW0_RING_HP;',
                 's = &hal->srng_config[HAL_WBM2SW_RELEASE]; s->reg_start[0] = HAL_SEQ_WCSS_UMAC_WBM_REG + HAL_WBM0_RELEASE_RING_BASE_LSB(ab); '
                 's->reg_start[1] = HAL_SEQ_WCSS_UMAC_WBM_REG + HAL_WBM0_RELEASE_RING_HP; s->reg_size[0] = HAL_WBM1_RELEASE_RING_BASE_LSB(ab) '
                 '- HAL_WBM0_RELEASE_RING_BASE_LSB(ab); s->reg_size[1] = HAL_WBM1_RELEASE_RING_HP - HAL_WBM0_RELEASE_RING_HP;'):
        assert fact in wc, fact
    # ath12k_hal_srng_dst_hw_init's order (MSI writes only with HAL_SRNG_FLAGS_MSI_INTR, left out)
    di = flat(fn_body(hal_c, 'static void ath12k_hal_srng_dst_hw_init('))
    seq = [di.index(s_) for s_ in ('if (srng->flags & HAL_SRNG_FLAGS_MSI_INTR) {', 'ath12k_hif_write32(ab, reg_base, srng->ring_base_paddr);',
                                   'reg_base + ath12k_hal_reo1_ring_base_msb_offset(ab), val);', 'reg_base + ath12k_hal_reo1_ring_id_offset(ab), val);',
                                   'reg_base + ath12k_hal_reo1_ring_producer_int_setup_offset(ab), val);',
                                   'reg_base + ath12k_hal_reo1_ring_hp_addr_lsb_offset(ab), hp_addr & HAL_ADDR_LSB_REG_MASK);',
                                   'reg_base + ath12k_hal_reo1_ring_hp_addr_msb_offset(ab), hp_addr >> HAL_ADDR_MSB_REG_SHIFT);',
                                   'ath12k_hif_write32(ab, reg_base, 0);', 'ath12k_hif_write32(ab, reg_base + HAL_REO1_RING_TP_OFFSET, 0);',
                                   'val |= HAL_REO1_RING_MISC_SRNG_ENABLE;', 'reg_base + ath12k_hal_reo1_ring_misc_offset(ab), val);')]
    assert seq == sorted(seq), seq
    assert mask('HAL_REO1_RING_BASE_MSB_RING_SIZE', hal_h) == 0xfffff00 and mask('HAL_REO1_RING_ID_RING_ID', hal_h) == 0xff00 and \
        mask('HAL_REO1_RING_PRDR_INT_SETUP_INTR_TMR_THOLD', hal_h) == 0xffff0000 and \
        mask('HAL_REO1_RING_PRDR_INT_SETUP_BATCH_COUNTER_THOLD', hal_h) == 0x7fff and bit('HAL_REO1_RING_MISC_SRNG_ENABLE', hal_h) == 0x40
    # thresholds (dp.c ath12k_dp_srng_setup), the WCN7850 TX completion rings (hw.c), the setup order (dp.c, core.c)
    ss = flat(fn_body(dp_c, 'int ath12k_dp_srng_setup('))
    assert 'case HAL_REO_DST: params.intr_batch_cntr_thres_entries = HAL_SRNG_INT_BATCH_THRESHOLD_RX; params.intr_timer_thres_us = ' \
        'HAL_SRNG_INT_TIMER_THRESHOLD_RX; break;' in ss
    other = ss.split('fallthrough; ', 1)[1].split('break;', 1)[0]
    assert 'case HAL_REO_EXCEPTION:' in other and 'HAL_SRNG_INT_BATCH_THRESHOLD_OTHER' in other and 'HAL_SRNG_INT_TIMER_THRESHOLD_OTHER' in other
    assert 'if (ring_num == 0 || ring_num == 2 || ring_num == 4) return true;' in flat(fn_body(hw_c, 'static bool ath12k_dp_srng_is_comp_ring_wcn7850('))
    thr = {'dst': define('HAL_SRNG_INT_TIMER_THRESHOLD_RX', hal_h) >> 3 << 16 | define('HAL_SRNG_INT_BATCH_THRESHOLD_RX', hal_h) * 8,
           'other': define('HAL_SRNG_INT_TIMER_THRESHOLD_OTHER', hal_h) >> 3 << 16 | define('HAL_SRNG_INT_BATCH_THRESHOLD_OTHER', hal_h) * 8}
    assert (thr['dst'], thr['other']) == (0x003e0400, 0x00200008)
    cs = flat(fn_body(dp_c, 'static int ath12k_dp_srng_common_setup('))
    order = [cs.index(s_) for s_ in ('&dp->rx_rel_ring, HAL_WBM2SW_RELEASE, HAL_WBM2SW_REL_ERR_RING_NUM, 0, DP_RX_RELEASE_RING_SIZE);',
                                     '&dp->reo_except_ring, HAL_REO_EXCEPTION, 0, 0, DP_REO_EXCEPTION_RING_SIZE);',
                                     'ath12k_hal_reo_hw_setup(ab, ring_hash_map);')]
    assert order == sorted(order)
    assert 'ring_hash_map = HAL_HASH_ROUTING_RING_SW1 | HAL_HASH_ROUTING_RING_SW2 << 4 | HAL_HASH_ROUTING_RING_SW3 << 8 | ' \
        'HAL_HASH_ROUTING_RING_SW4 << 12 | HAL_HASH_ROUTING_RING_SW1 << 16 | HAL_HASH_ROUTING_RING_SW2 << 20 | ' \
        'HAL_HASH_ROUTING_RING_SW3 << 24 | HAL_HASH_ROUTING_RING_SW4 << 28;' in cs
    hmap = 0
    for i, n in enumerate((1, 2, 3, 4, 1, 2, 3, 4)):
        hmap |= define(f'HAL_HASH_ROUTING_RING_SW{n}', hal_h) << (4 * i)
    assert hmap == H['WLRD_HASH']
    da = flat(fn_body(dp_c, 'int ath12k_dp_alloc('))
    assert da.index('ath12k_wbm_idle_ring_setup(ab, &n_link_desc);') < da.index('ath12k_dp_srng_common_setup(ab);') < da.index('ath12k_dp_rx_alloc(ab);')
    assert 'if (!ab->hw_params->reoq_lut_support) return 0;' in flat(fn_body(dp_c, 'static int ath12k_dp_reoq_lut_setup('))
    assert '.reoq_lut_support = false' in hw_c.split('.name = "wcn7850 hw2.0"', 1)[1].split('.name =', 1)[0]
    cc = flat(core_c)
    assert cc.index('ret = ath12k_wmi_wait_for_service_ready(ab);') < cc.index('ret = ath12k_dp_rx_pdev_reo_setup(ab);') < \
        cc.index('ret = ath12k_wmi_cmd_init(ab);')
    assert cc.index('ret = ath12k_ce_init_pipes(ab);') < cc.index('ret = ath12k_dp_alloc(ab);') < cc.index('ret = ath12k_core_start(ab);')
    rs = flat(fn_body(dp_rx_c, 'int ath12k_dp_rx_pdev_reo_setup('))
    assert 'for (i = 0; i < DP_REO_DST_RING_MAX; i++) { ret = ath12k_dp_srng_setup(ab, &dp->reo_dst_ring[i], HAL_REO_DST, i, 0, ' \
        'DP_REO_DST_RING_SIZE);' in rs
    # ath12k_hal_reo_hw_setup's order and values
    hw = flat(fn_body(hal_rx_c, 'void ath12k_hal_reo_hw_setup('))
    seq = [hw.index(s_) for s_ in ('val = ath12k_hif_read32(ab, reo_base + HAL_REO1_GEN_ENABLE);',
                                   'ath12k_hif_write32(ab, reo_base + HAL_REO1_GEN_ENABLE, val);',
                                   'val = ath12k_hif_read32(ab, reo_base + HAL_REO1_MISC_CTRL_ADDR(ab));',
                                   'ath12k_hif_write32(ab, reo_base + HAL_REO1_MISC_CTRL_ADDR(ab), val);',
                                   'HAL_REO1_AGING_THRESH_IX_0(ab), HAL_DEFAULT_BE_BK_VI_REO_TIMEOUT_USEC);',
                                   'HAL_REO1_AGING_THRESH_IX_1(ab), HAL_DEFAULT_BE_BK_VI_REO_TIMEOUT_USEC);',
                                   'HAL_REO1_AGING_THRESH_IX_2(ab), HAL_DEFAULT_BE_BK_VI_REO_TIMEOUT_USEC);',
                                   'HAL_REO1_AGING_THRESH_IX_3(ab), HAL_DEFAULT_VO_REO_TIMEOUT_USEC);',
                                   'reo_base + HAL_REO1_DEST_RING_CTRL_IX_2, ring_hash_map);', 'reo_base + HAL_REO1_DEST_RING_CTRL_IX_3, ring_hash_map);')]
    assert seq == sorted(seq), seq
    assert 'u32_encode_bits(HAL_SRNG_RING_ID_REO2SW0, HAL_REO1_MISC_CTL_FRAG_DST_RING)' in hw and \
        'u32_encode_bits(HAL_SRNG_RING_ID_REO2SW0, HAL_REO1_MISC_CTL_BAR_DST_RING)' in hw
    gen = bit('HAL_REO1_GEN_ENABLE_AGING_LIST_ENABLE', hal_h) | bit('HAL_REO1_GEN_ENABLE_AGING_FLUSH_ENABLE', hal_h)
    mmask = mask('HAL_REO1_MISC_CTL_FRAG_DST_RING', hal_h) | mask('HAL_REO1_MISC_CTL_BAR_DST_RING', hal_h)
    assert gen == H['WLRD_GEN_AGING'] and mmask == H['WLRD_MISC_DST_MASK']
    hwregs = [(reo + define('HAL_REO1_GEN_ENABLE', hal_h), 0x1 | gen), (reo + reg['hal_reo1_misc_ctrl_addr'], 0x01fe0001 & ~mmask),
              (reo + reg['hal_reo1_aging_thres_ix0'], num('HAL_DEFAULT_BE_BK_VI_REO_TIMEOUT_USEC', hal_h)),
              (reo + reg['hal_reo1_aging_thres_ix1'], num('HAL_DEFAULT_BE_BK_VI_REO_TIMEOUT_USEC', hal_h)),
              (reo + reg['hal_reo1_aging_thres_ix2'], num('HAL_DEFAULT_BE_BK_VI_REO_TIMEOUT_USEC', hal_h)),
              (reo + reg['hal_reo1_aging_thres_ix3'], num('HAL_DEFAULT_VO_REO_TIMEOUT_USEC', hal_h)),
              (reo + define('HAL_REO1_DEST_RING_CTRL_IX_2', hal_h), hmap), (reo + define('HAL_REO1_DEST_RING_CTRL_IX_3', hal_h), hmap)]
    # every word of the model's dump
    d = {}
    for line in dump.splitlines():
        p = line.split()
        d[' '.join(p[:-1])] = int(p[-1])
    for k, v in want.items():
        assert d[k] == v, (k, d[k], v)
    base, sizes = 0xa2600000, [H['WLRD_REL_ENTRIES'], H['WLRD_EXC_ENTRIES']] + [H['WLRD_DST_ENTRIES']] * 8
    offs = [H['WLRD_OFF_REL'], H['WLRD_OFF_EXC']] + [0x10000 + 0x10000 * n for n in range(8)]
    sh = re.search(r'static const uint32_t wl15_shadow_v3\[36\] = \{(.*?)\};', DRIVER, re.S)[1]
    shadow = [int(x, 16) for x in re.findall(r'0x[0-9a-fA-F]+', sh)]
    for i, rid in enumerate(ids):
        words = sizes[i] * sz[0] // 4
        regs = [(r0s[i], base + offs[i]), (r0s[i] + off['base_msb'], words << 8), (r0s[i] + off['id'], rid << 8 | sz[0] // 4),
                (r0s[i] + off['prod'], thr['dst' if i >= 2 else 'other']), (r0s[i] + off['hp_lsb'], base + 4 * rid),
                (r0s[i] + off['hp_msb'], 0), (r2s[i], 0), (r2s[i] + tpoff, 0), (r0s[i] + off['misc'], 0x40)]
        for k, (a, v) in enumerate(regs):
            assert d[f'ring{rid} {k} {a}'] == v, (rid, k, hex(a), hex(v))
        assert shadow[d[f'shadow{rid} 0']] == r2s[i] + tpoff and d[f'shadow{rid} 0'] == (24 if rid == 131 else 8 if rid == 0 else rid - 1)
    for k, (a, v) in enumerate(hwregs):
        assert d[f'hw {k} {a}'] == v, (k, hex(a), hex(v))
    pre = []
    for i in range(10):
        pre += [r0s[i], r0s[i] + off['base_msb'], r0s[i] + off['misc']]
    pre += [a for a, _ in hwregs]
    assert [d[f'pre {i}'] for i in range(len(pre))] == pre and len(pre) == 38 and 'pre 38' not in d
    assert '#define WLRD_PRE_N (3u * WLRD_NRING + 8u)' in HEADER and \
        'enum { WLRD_R_REL, WLRD_R_EXC, WLRD_R_DST, WLRD_NRING = WLRD_R_DST + WLRD_NDST };' in HEADER
    print('L19 re-derivation (C2d): RX_RING_SELECTION_CFG (msg 0xc, info0/info1 fields, OFFSET_VALID, EN_RXMON, BAR, FP data '
          'flags, RXDMA_RING TLVs, the 17-word struct, the WCN7850 filter and its loop over the MAC rings with mac + 1 and buffer '
          'size 2048, rx_desc.h compiled: descriptor 400, header 272, mpdu_start 144, msdu_end 0), the REO/release ring ids, '
          'sizes, entry structs (compiled), entry fields and RBMs, the WCN7850 R0/R2 registers, dst_hw_init\'s order, '
          'thresholds, dp_alloc and core placement, reo_hw_setup\'s order and values with the hash map 0x43214321; the '
          'model\'s dump word by word (2 x 17 selection words, 10 x 9 ring writes, 8 setup writes, 38 pre-reads, 10 TP shadow '
          'indices)')


# Each C2d rule broken in the production code (driver or header) must be caught by the C2d static checks or the
# model's C2d scenarios.
C2D_MUTANTS = [
    ('h', '	    WLRD_SEL_OFFSET_VALID | WLRD_SEL_EN_RXMON;', '	    WLRD_SEL_EN_RXMON;', 'selection without OFFSET_VALID'),
    ('h', 'w[0] = WLRD_HTT_RX_RING_SEL | ((mac + 1u) & 0xffu) << 8', 'w[0] = WLRD_HTT_RX_RING_SEL | (mac & 0xffu) << 8', 'selection pdev = mac'),
    ('h', '#define WLRD_SEL_BUF_SIZE 2048u', '#define WLRD_SEL_BUF_SIZE 2176u', 'selection buffer size'),
    ('h', '#define WLRD_SEL_TLV ((1u << 0) | (1u << 2) | (1u << 3))', '#define WLRD_SEL_TLV 0xfu', 'selection TLVs'),
    ('h', '#define WLRD_OFF_PKT_HDR 272u', '#define WLRD_OFF_PKT_HDR 256u', 'selection header offset'),
    ('h', '		{ r->r2, 0u },\n		{ r->r2 + 4u, 0u },\n		{ r->r0 + WLCE_R0_MISC, WLCE_MISC_SRNG_ENABLE },',
     '		{ r->r0 + WLCE_R0_MISC, WLCE_MISC_SRNG_ENABLE },\n		{ r->r2, 0u },\n		{ r->r2 + 4u, 0u },', 'MISC before R2'),
    ('h', '#define WLRD_BATCH_DST 128u', '#define WLRD_BATCH_DST 64u', 'destination batch threshold'),
    ('h', '#define WLRD_AGING_VO 40000u', '#define WLRD_AGING_VO 100000u', 'VO aging threshold'),
    ('h', '| 3u << 24 | 4u << 28)', '| 3u << 24 | 1u << 28)', 'hash map'),
    ('h', '(misc & ~WLRD_MISC_DST_MASK) |', 'misc |', 'MISC_CTRL destination fields not cleared'),
    ('h', 'r->hp_iova = rd->rdp_iova + 4ull * r->id;', 'r->hp_iova = rd->rdp_iova + 4ull * i;', 'HP address by index'),
    ('h', '	if (pre[3u * WLRD_NRING + 6u] == WLRD_HASH || pre[3u * WLRD_NRING + 7u] == WLRD_HASH) { return WLRD_APPLIED; }\n', '',
     'hash map already applied not refused'),
    ('h', '	for (unsigned k = 0; k < WLRD_NRING; k++) { if (pre[3u * k + 2u] & WLCE_MISC_SRNG_ENABLE) { return WLRD_SRNG; } }\n', '',
     'an enabled ring not refused'),
    ('h', '		return wlrd_stop(rd, "W27 returned entry is not one of our posted buffers");\n	}', '	}', 'foreign return accepted'),
    ('h', '	if (rd->slot[k] != WLRD_SLOT_OUT) { return wlrd_stop(rd, "W27 buffer returned twice (not out with the device)"); }\n', '',
     'double return accepted'),
    ('h', '		for (uint32_t j = 0; j < WLRX_SENTINEL_BYTES / 4u; j++) { s[j] = 0xa5a5a5a5u; }\n', '', 'no fresh sentinel'),
    ('h', '		for (uint32_t j = 0; j < WLRX_BUF_BYTES / 4u; j++) { s[j] = 0; }\n', '', 'returned buffer not zeroed'),
    ('h', '		io->wbarrier(io->ctx);\n		WLRD_PUBLISH(', '		WLRD_PUBLISH(', 'refill republish without the barrier'),
    ('h', '(tp + size - rd->refill_hp - WLRX_BUF_DW) % size >= WLRX_BUF_DW', '(tp + size - rd->refill_hp) % size >= WLRX_BUF_DW',
     'replenish past the TP'),
    ('h', '	hp = WLRD_LOAD(&rd->rdp[r->id]); io->rbarrier(io->ctx);', '	hp = WLRD_LOAD(&rd->rdp[r->id]);', 'HP without the read barrier'),
    ('h', '	if (hp >= size || hp % WLRD_ENTRY_DW) { return wlrd_stop(rd, "W27 REO/release ring HP outside the ring"); }\n', '',
     'HP not bounded'),
    ('h', '		if (((e[1] >> 8) & 0xfu) != WLRD_RBM_SW3) { rd->rbm_other++; return 0; }\n', '', 'release RBM not checked'),
    ('h', '		if (wlrd_recycle(rd, rx, m0, m1)) { return 1; }', '		if (0 && wlrd_recycle(rd, rx, m0, m1)) { return 1; }',
     'link MSDUs not returned'),
    ('d', '	wl_ce_wbarrier(NULL);\n	if (wl15_variant) { return wl27_reo(', '	if (wl15_variant) { return wl27_reo(', 'TP without the barrier'),
    ('d', 'if (wl15_shadow_v3[j] == r->r2 + 4u) { idx = j; }', 'if (wl15_shadow_v3[j] == r->r2) { idx = j; }', 'shadow of the HP'),
    ('d', '	if (wl_rd.own != WLRD_FREE) {\n		printf("FUXI WLAN: W27 reo not programmed', '	if (0) {\n		printf("FUXI WLAN: W27 reo not programmed',
     'write after a refusal'),
    ('d', '		if (b == ~0u || m == ~0u || c == ~0u || b || (m & 0xffu) || (c & WLCE_MISC_SRNG_ENABLE)) {', '		if (b == ~0u) {',
     'W21 re-read ignored'),
    ('d', '	if (wl27_dest()) { return 1; }  /* C2d', '	if (0) { return 1; }  /* C2d', 'destination rings never set up'),
    ('d', '	wl_rd.stopping = true;  /* C2d: no reap', '	(void)0;  /* C2d: no reap', 'no latch'),
    ('d', '		if ((wl20_bits & WL20_RXSEL) && (wl27_sel(0) || wl27_sel(1))) { return 1; }', '', 'no ring selection'),
    ('d', '	if (wl_rd.reo_on && wl27_poll()) { return 1; }  /* C2d', '	/* C2d', 'never polled'),
    ('d', '(((unsigned)w20 & WL27_BITS) && !((unsigned)w20 & WL20_RXRING)) ||', '0 ||',
     'gate without 0x100'),
    ('d', '	if (wl_reo_arena.va) { bzero((void *)wl_reo_arena.va, wl_reo_arena.size); bytes += (uint32_t)wl_reo_arena.size; }\n', '',
     'REO arena not zeroed'),
    ('d', '		if (moved && wl27_tp(&wl_rd.ring[i])) {', '		if (0 && wl27_tp(&wl_rd.ring[i])) {', 'TP never published'),
]

# Each C2e rule broken in the production code (driver or header) must be caught by the C2e static checks or the
# model's C2e scenarios.
C2E_MUTANTS = [
    ('h', '#define WLMS_HTT_RING_MON_STATUS 1u', '#define WLMS_HTT_RING_MON_STATUS 2u', 'monitor status HTT ring id'),
    ('h', 'r->htt_ring = WLMS_HTT_RING_MON_STATUS;\n		r->htt_type = WLRX_HTT_SW_TO_HW;',
     'r->htt_ring = WLMS_HTT_RING_MON_STATUS;\n		r->htt_type = WLRX_HTT_SW_TO_SW;', 'monitor status ring type SW_TO_SW'),
    ('h', 'r->batch = WLMS_BATCH; r->timer_us = WLMS_TIMER_US; r->low = true;', 'r->batch = WLMS_BATCH; r->timer_us = WLMS_TIMER_US; r->low = false;',
     'no low threshold'),
    ('h', '#define WLMS_TIMER_US 500u', '#define WLMS_TIMER_US 256u', 'monitor status timer threshold'),
    ('h', '#define WLMS_BATCH 1u', '#define WLMS_BATCH 0u', 'monitor status batch threshold'),
    ('h', 'r->id = (uint16_t)(m ? WLRX_RING_MON1 : WLRX_RING_MON0);', 'r->id = (uint16_t)(m ? WLRX_RING_MON0 : WLRX_RING_MON1);',
     'ring ids swapped'),
    ('h', '		r->hp_iova = rx->wrp_iova + 4ull * (r->id - WLRX_DMAC_START);\n		r->tp_iova = rx->rdp_iova + 4ull * r->id;',
     '		r->tp_iova = rx->wrp_iova + 4ull * (r->id - WLRX_DMAC_START);\n		r->hp_iova = rx->rdp_iova + 4ull * r->id;', 'HP/TP words swapped'),
    ('h', 'wlrx_addr_entry(v, wlms_slot_iova(ms, m, k), WLRX_RBM_SW1, k);', 'wlrx_addr_entry(v, wlms_slot_iova(ms, m, k), WLRX_RBM_SW1, k + 1u);',
     'cookie off by one'),
    ('h', 'wlrx_addr_entry(v, wlms_slot_iova(ms, m, k), WLRX_RBM_SW1, k);', 'wlrx_addr_entry(v, wlms_slot_iova(ms, m, k), WLRX_RBM_IDLE_DESC, k);',
     'wrong RBM'),
    ('h', '	if (k < WLMS_NBUF) { wlrx_addr_entry(v,', '	if (k <= WLMS_NBUF) { wlrx_addr_entry(v,', 'entry 1023 listed'),
    ('h', '	io->wbarrier(io->ctx);\n	WLMS_PUBLISH(', '	WLMS_PUBLISH(', 'HP publish without the barrier'),
    ('h', '		w->tp[m] = WLMS_LOAD(&rx->rdp[ms->ring[m].id]); io->rbarrier(io->ctx);', '		w->tp[m] = WLMS_LOAD(&rx->rdp[ms->ring[m].id]);',
     'TP without the read barrier'),
    ('h', '	if (ms->stopping || rx->stopping || !ms->base || m >= 2u) { return 1; }\n	for (uint32_t k = 0;',
     '	if (!ms->base || m >= 2u) { return 1; }\n	for (uint32_t k = 0;', 'fill after the latch'),
    ('h', '		*gap += !ring && a[i] != 0;', '		(void)ring;', 'head gaps not counted'),
    ('h', '		if (posted) { *upper += n; *upper_slots += n != 0; } else { *last += n; }', '		if (posted) { *upper += n; *upper_slots += n != 0; }',
     'unposted slots not counted'),
    ('h', '	rx->mon_named = true;\n', '', 'W26 gap count does not name the monitor status words'),
    ('h', '	return i >= WLMS_DIAG_N ? 0u : i ? bhi + o[i] : WLMS_MHISTATUS;', '	return i >= WLMS_DIAG_N ? 0u : i ? bhi + o[i] : 0x38u;',
     'crash line reads MHICTRL'),
    ('h', '			if (!w->left[m]) { w->exhausted |= f; }', '', 'exhaustion never flagged'),
    ('h', '			if (a < s0 || (a - s0) % WLMS_SLOT_BYTES', '			if (a < s0 && (a - s0) % WLMS_SLOT_BYTES', 'foreign entries not counted'),
    ('d', '		if ((wl20_bits & WL20_MONSTAT) && (wl28_sring(0) || wl28_sring(1))) { return 1; }  /* C2e: dp_rx.c:4657-4670 */\n', '',
     'monitor status rings never announced'),
    ('d', '(((unsigned)w20 & WL20_MONSTAT) && !((unsigned)w20 & WL20_RXRING)) ||', '(0) ||',
     'gate: 0x2000 without 0x100'),
    ('d', '	wl28_crash("contain before MHI reset");  /* C2e: read-only; silent without w20 0x2000 */\n', '', 'no crash line before the MHI reset'),
    ('d', '		wl28_crash("scan COMPLETED timeout");  /* C2e: read-only, before anything else touches the device */\n', '',
     'no crash line at the timeout'),
    ('d', '	if (!wlms_diag_ok(off, wl_bhi)) { return ~0u; }\n', '', 'crash read without the allowlist'),
    ('d', '	wl_ms.stopping = true;  /* C2e: no host write', '	(void)0;  /* C2e: no host write', 'no latch'),
    ('d', '	if (wl_mon_arena.va) { bzero((void *)wl_mon_arena.va, wl_mon_arena.size); bytes += (uint32_t)wl_mon_arena.size; }\n', '',
     'arena not zeroed'),
    ('d', '	wl_dma_free(&wl_mon_arena);\n', '', 'arena never freed'),
    ('d', '	for (unsigned m = 0; (wl20_bits & WL20_MONSTAT) && m < 2u; m++) {', '	for (unsigned m = 0; (wl20_bits & WL20_MONSTAT) && m < 1u; m++) {',
     'ring 193 never filled'),
    ('d', '	if (!wl_ms.on || wl_ms.stopping || p >= WLRX_NPOINT) { return; }\n	wlms_watch(', '	if (1) { return; }\n	wlms_watch(',
     'never watched'),
    ('d', '	wl28_report();  /* C2e; silent without 0x2000 */\n', '', 'never reported'),
    ('d', 'if (wlms_fill(&wl_ms, &wl_rx, m) || wlms_publish(&wl_ms, &wl_rx, &wl_ce_io, m))',
     'if (wlms_publish(&wl_ms, &wl_rx, &wl_ce_io, m) || wlms_fill(&wl_ms, &wl_rx, m))', 'HP published before the entries'),
]


def static_c2f():
    """C2f: the gate (0x4000 needs 0x1000, qrtr 6 refuses it), L19's dp_alloc order (cc_init after the idle list and before
    the common rings; the pre group before rel/exc and the post group after them; cc_config at service ready before the
    destination rings; mac-start params around the chain mask and channel list; encap/decap before NSS; the W29 report after
    W27's), the two allowlisted accessors through wl_win, no transmit, the latch and the free."""
    drv, hdr = DRIVER, HEADER
    code = strip_comments(drv)
    cfg = between(drv, 'static void\nwl_probe_configure(void)\n{', '\n}\n')
    assert '#define WL20_L19ORDER 0x4000u' in drv and header_define('WLRX_W20_L19ORDER') == 0x4000
    assert 'WL20_L19ORDER' in drv.split('#define WL23_BITS', 1)[1].split('\n')[0]
    assert 'L19ORDER' not in drv.split('#define WL20_BITS', 1)[1].split('\n')[0]  # qrtr 6 refuses it
    assert '(((unsigned)w20 & WL20_L19ORDER) && !((unsigned)w20 & WL20_REORX))) { wl_variant_invalid = true; }' in cfg
    region = between(drv, '/* ---- C2f (fuxiwlanqrtr=7)', '/* ---- end W20 ---- */')
    fns = c_functions(region)
    for name in ('wl29_reg', 'wl29_cmem', 'wl29_ring', 'wl29_cc_init', 'wl29_common_pre', 'wl29_common_post', 'wl29_cc_config',
                 'wl29_macstart_pre', 'wl29_macstart_post', 'wl29_encap_decap', 'wl29_report', 'wl29_ppdu_stats'):
        assert name in fns, name
    w26 = c_functions(between(drv, W26_START, W27_START))
    # dp_alloc order: the idle list, then the common setup (wl27_setup) with cc_init inside it, before the refill.
    st = w26['wl26_setup']
    assert st.index('wl26_idle()') < st.index('wl27_setup()') < st.index('wlrx_fill_refill(&wl_rx)')
    assert 'wl29_cc_init' not in st
    assert 'wl26_alloc(&wl_cm_arena, WLCM_ARENA_BYTES' in st
    # common setup: the pre group before rel/exc, the post group after exc and before reo_hw_setup (the GEN_ENABLE read).
    su = between(drv, 'static int\nwl27_setup(void)\n{', '\n}\n')
    assert su.index('wl29_common_pre()') < su.index('wl27_ring(WLRD_R_REL) || wl27_ring(WLRD_R_EXC)') < \
        su.index('wl29_common_post()') < su.index('wl27_reo_rd(WLRD_REO_GEN_ENABLE)')
    # cc_init only after the ownership checks: C2d's REO pre-reads (a refusal reaches cc_init only for its drop line) and
    # W29's common-ring pre-reads (likewise), its CMEM writes before the first common-ring write.
    assert su.index('if (wl_rd.own != WLRD_FREE) {') < su.index('wl29_cc_init()') < su.index('wl29_common_pre()')
    cp = fns['wl29_common_pre']
    assert cp.index('if (wl29_common_owned()) { return wl29_cc_init(); }') < cp.index('wl_cm.rings_on = true;') < \
        cp.index('if (wl29_cc_init()) { return 1; }   /* C2f: cookie converter') < cp.index('wl29_ring(pre[j])')
    # The QMI CMEM region: cleared at the start of every capability parse, set only from that reply's dev_mem[0].
    qmi = between(drv, 'static int\nwl_qmi_cap(const uint8_t *p, unsigned len)\n{', '\n}\n')
    assert qmi.index('wl_cmem_valid = false;') < qmi.index('for (unsigned off = 7; off < len;)') < \
        qmi.index('wl_cmem_valid = true;')
    assert code.count('wl_cmem_valid = true;') == 1
    # cc_config at service ready, before the destination rings (core.c:923 < 925).
    w21 = between(drv, 'static int\nwl_w21_run(void)\n{', '\n}\n')
    assert w21.index('wl29_cc_config()') < w21.index('wl27_dest()')
    # mac-start params: pre group before the chain mask; post group (lro, idle_ps) after the channel list and before vdev
    # create; encap/decap after vdev create and before NSS.
    assert w26['wl26_rings'].index('wl29_macstart_pre()') < w26['wl26_rings'].index('wl26_chainmask()')
    w23 = between(drv, 'static int\nwl_w23_run(void)\n{', '\n}\n')
    assert w23.index('W23 chan list %u passive') < w23.index('wl29_macstart_post()') < w23.index('WMI VDEV_CREATE') < \
        w23.index('wl29_encap_decap()') < w23.index('WMI VDEV_SET_PARAM NSS')
    assert w26['wl26_report'].index('wl27_report();') < w26['wl26_report'].index('wl29_report();')
    # Containment latch and the free (zeroed, in the owned alias, freed once).
    assert 'wl_cm.stopping = true;' in w26['wl26_contain']
    fr = w26['wl26_free']
    assert fr.index('wlcm_guard_count(&wl_cm') < fr.index('bzero((void *)wl_cm_arena.va') < fr.index('wl_dma_free(&wl_cm_arena);')
    assert drv.count('wl_dma_free(&wl_cm_arena)') == 1
    # Two allowlisted accessors only (wl29_reg via wlcm_reg_ok, wl29_cmem via wlcm_cmem_ok); both refused after the latch.
    assert 'if (fw_faults || wl_cm.stopping || !wlcm_reg_ok(&wl_cm, off, wr)) { return 1; }' in fns['wl29_reg']
    assert 'if (fw_faults || wl_cm.stopping || !wlcm_cmem_ok(&wl_cm, off)) { return 1; }' in fns['wl29_cmem']
    w29 = strip_comments(region)
    # No raw hardware access in the W29 glue except through wl_win (the two accessors) and wl_ce_send (HTT/WMI commands);
    # nothing transmits (no TCL/completion post, no HP/TP publish of a common ring).
    for raw in ('wl_pwr(', 'wl_mwr(', 'wl_prd(', 'wl_contain_wr(', 'wl_ce_window(', 'wl_ce_io_bar_wr(', 'wl26_wbm(', 'wl27_reo(',
                'msi', 'MSI', 'gic', 'GIC', 'pmic', 'PMIC', 'irq', 'IRQ', 'WLRD_PUBLISH', 'WLRX_PUBLISH', 'WLMS_PUBLISH'):
        assert raw not in w29, raw
    assert w29.count('wl_win(') == 2  # wl29_reg, wl29_cmem
    # cc is gated on the QMI CMEM region (dropped for the boot without it); on success all three decode flags are set so the
    # refill cookies and the returned entries are decoded as cookie-converted.
    assert 'own = !wl_cmem_valid ? WLCM_NOCMEM : !wl_cm.cmem_valid ? WLCM_CMEM_BAD : WLCM_FREE;' in fns['wl29_cc_init'] and \
        'cc dropped for this boot' in region
    assert 'wl_cm.cc_on = wl_rx.cc_on = wl_rd.cc_on = true;' in fns['wl29_cc_init']
    # The REO command ring's per-entry command numbers; never a TX descriptor posted.
    assert 'c->va[i * c->entry_dw + 2u] = (i + 1u) & 0xffffu;' in fns['wl29_common_post']
    print('C2f static: gate (0x4000 needs 0x1000, qrtr 6 refuses it), L19 dp_alloc order (cc_init after the idle list, the '
          'REO and common-ring ownership checks and before the common rings, the pre group before rel/exc and the post '
          'group after them, the QMI CMEM region cleared per capability parse, cc_config before the '
          'destination rings, mac-start params around the chain mask and channel list, encap/decap before NSS, W29 report '
          'after W27), the two wl_win accessors (wlcm_reg_ok, wlcm_cmem_ok) refused after the latch, no transmit and no ring '
          'post, cc gated on the CMEM region, the latch and the single zeroed free')


# Each C2f rule broken in the production code must be caught by the C2f static checks or the model's C2f scenarios
# (s_c2f_pure, s_c2f_gate).
C2F_MUTANTS = [
    ('h', '#define WLCM_REO_CC_CFG1_VAL (WLCM_CC_CFG1_COMMON | (1u << 19) | (1u << 20))',
     '#define WLCM_REO_CC_CFG1_VAL (WLCM_CC_CFG1_COMMON | (1u << 20))', 'cc cfg1 without ENABLE'),
    ('h', '#define WLCM_WBM_CC_CONVERT_OR ((1u << 8) | (1u << 1) | (1u << 3) | (1u << 4) | (1u << 5))',
     '#define WLCM_WBM_CC_CONVERT_OR ((1u << 8) | (1u << 1) | (1u << 3) | (1u << 5))', 'wbm2sw3 conversion dropped'),
    ('h', '#define WLCM_COMP_ENTRIES 32768u         /* tx_comp_ring_size, DEFAULT profile */',
     '#define WLCM_COMP_ENTRIES 16384u         /* tx_comp_ring_size, DEFAULT profile */', 'completion ring size'),
    ('h', '#define WLCM_RX_SPT_OFFSET WLCM_NUM_TX_SPT_PAGES   /* ATH12K_RX_SPT_PAGE_OFFSET (dp.h:256) */',
     '#define WLCM_RX_SPT_OFFSET 0u   /* ATH12K_RX_SPT_PAGE_OFFSET (dp.h:256) */', 'rx spt offset'),
    ('h', '#define WLCM_CC_SPT_SHIFT 9u             /* ATH12K_CC_PPT_SHIFT (dp.h:279) */',
     '#define WLCM_CC_SPT_SHIFT 8u             /* ATH12K_CC_PPT_SHIFT (dp.h:279) */', 'cookie ppt shift'),
    ('h', '#define WLCM_SW2WBM_R0 (WLCM_WBM_REG + 0x37cu)', '#define WLCM_SW2WBM_R0 (WLCM_WBM_REG + 0x380u)', 'sw2wbm base'),
    ('h', '#define WLCM_REOCMD_DW 12u', '#define WLCM_REOCMD_DW 11u', 'reo cmd entry size'),
    ('h', '#define WLCM_REOSTATUS_DW 28u            /* (sizeof(hal_tlv_64_hdr) 8 + sizeof(hal_reo_get_queue_stats_status) 104) >> 2 */',
     '#define WLCM_REOSTATUS_DW 27u            /* (sizeof(hal_tlv_64_hdr) 8 + sizeof(hal_reo_get_queue_stats_status) 104) >> 2 */',
     'reo status entry size'),
    ('h', 'regs[k][0] = r->r0 + WLCE_R0_MISC; regs[k++][1] = WLCE_MISC_LOOPCNT_DIS | WLCE_MISC_SRNG_ENABLE;',
     'regs[k][0] = r->r0 + WLCE_R0_MISC; regs[k++][1] = WLCE_MISC_SRNG_ENABLE;', 'src ring loopcnt bit'),
    ('h', '#define WLWMI_PDEV_PARAM_PMF_QOS 0x22u', '#define WLWMI_PDEV_PARAM_PMF_QOS 0x23u', 'pmf-qos param id'),
    ('h', '#define WLWMI_VDEV_PARAM_RX_DECAP_TYPE 0x5bu', '#define WLWMI_VDEV_PARAM_RX_DECAP_TYPE 0x5cu', 'rx decap param id'),
    ('h', '#define WLHTT_PPDU_STATS_DEFAULT 0xe67u         /* HTT_PPDU_STATS_TAG_DEFAULT bits */',
     '#define WLHTT_PPDU_STATS_DEFAULT 0xe66u         /* HTT_PPDU_STATS_TAG_DEFAULT bits */', 'ppdu stats default bits'),
    ('h', 'if (slot >= WLRX_NBUF) { return wlrd_stop(rd, "W29 returned cookie is not one of our converted cookies"); }\n', '',
     'foreign converted cookie accepted'),
    ('h', 'page = ppt - WLCM_RX_SPT_OFFSET;\n\tk = page * WLCM_SPT_ENTRIES + spt;', 'page = ppt;\n\tk = page * WLCM_SPT_ENTRIES + spt;',
     'cookie slot decode offset'),
    ('d', 'if (wl29_cc_init()) { return 1; }   /* C2f: cookie converter', '(void)0; /* C2f: cookie converter', 'cc_init skipped'),
    ('d', 'if (wl29_common_pre()) { return 1; }   /* C2f: SW2WBM release', '(void)0;   /* C2f: SW2WBM release', 'common pre skipped'),
    ('d', 'if (wl29_common_post()) { return 1; }  /* C2f: REO command', '(void)0;  /* C2f: REO command', 'common post skipped'),
    ('d', 'if (wl29_cc_config()) { return 1; }  /* C2f: ath12k_dp_cc_config', '(void)0;  /* C2f: ath12k_dp_cc_config', 'cc_config skipped'),
    ('d', 'if (wl29_encap_decap()) { return 1; }   /* C2f: vdev encap/decap', '(void)0;   /* C2f: vdev encap/decap', 'encap/decap skipped'),
    ('d', '(((unsigned)w20 & WL20_L19ORDER) && !((unsigned)w20 & WL20_REORX))) { wl_variant_invalid = true; }',
     '(0)) { wl_variant_invalid = true; }', 'gate: 0x4000 without 0x1000'),
    ('d', 'wl_cm.stopping = true;  /* C2f: no common-ring', '(void)0;  /* C2f: no common-ring', 'no latch'),
    ('d', '	if (wl_cm_arena.va) { bzero((void *)wl_cm_arena.va, wl_cm_arena.size); bytes += (uint32_t)wl_cm_arena.size; }\n', '',
     'cm arena not zeroed'),
    ('d', '	wl_dma_free(&wl_cm_arena);\n', '', 'cm arena never freed'),
    ('d', 'wl_cm.cc_on = wl_rx.cc_on = wl_rd.cc_on = true;', 'wl_cm.cc_on = wl_rx.cc_on = true;', 'reo decode cc flag not set'),
    # C2f review fixes: the cc ownership gate (H1) and the CMEM bounds (M1).
    ('d', 'own = wl_rd.own != WLRD_FREE ? WLCM_REO_OWNED : !wl_cm.rings_on ? WLCM_RINGS_OWNED : WLCM_FREE;', 'own = WLCM_FREE;',
     'cc_init without the REO/common-ring gate'),
    ('d', ' || !wl_rd.reo_on || !wl_cm.rings_on) { return 0; }', ') { return 0; }', 'cc_config without the REO/common-ring gate'),
    ('d', 'for (unsigned k = 0; k < WLCM_CC_REGS; k++) { wl_cm.cc_pre[k] = wl29_reg_rd(regs[k][0]); }', '(void)regs;',
     'cc registers not pre-read'),
    ('d', 'wl_cm.cmem_valid = wl_cmem_valid && wlcm_cmem_region_ok(wl_cmem_base, wl_cmem_size);', 'wl_cm.cmem_valid = wl_cmem_valid;',
     'CMEM region not validated'),
    ('d', 'wl_cm.cmem_base = (uint32_t)wl_cmem_base; wl_cm.cmem_size = wl_cmem_size;', 'wl_cm.cmem_base = (uint32_t)wl_cmem_base;',
     'CMEM size not kept'),
    ('d', '\tif ((off >> 19) > 0x3fu) { return 1; }   /* the 6-bit window field: an offset beyond it is refused, never folded into '
     'the register */\n', '', 'window offset past its field not refused'),
    ('d', '\twl_cmem_valid = false; wl_cmem_base = wl_cmem_size = 0;   /* C2f: set again only from this reply\'s dev_mem[0] */\n', '',
     'QMI parse keeps a stale CMEM region'),
    ('h', '\t\tif (off == WLCM_REO_CC_CFG0 || off == WLCM_REO_CC_CFG1 || off == WLCM_WBM_CC_CFG0 || off == WLCM_WBM_CC_CFG1) '
     '{ return true; }\n', '', 'cc registers not in the read allowlist'),
    ('h', 'if (pre[0] || pre[2] || ', 'if (', 'a configured cc CFG0 base not refused'),
    ('h', '(pre[1] & WLCM_REO_CC_CFG1_EN) || ', '', 'REO cc CFG1 enable not refused'),
    ('h', '#define WLCM_REO_CC_CFG1_EN ((1u << 19) | (1u << 20))', '#define WLCM_REO_CC_CFG1_EN (1u << 19)',
     'REO cc CFG1 GLOBAL_ENABLE not refused'),
    ('h', ' || (pre[5] & WLCM_WBM_CC_CONVERT_GLOBAL)', '', 'convert GLOBAL_EN not refused'),
    ('h', '!(start & 3u) && ', '', 'CMEM alignment not checked'),
    ('h', 'start >= WLCM_CMEM_LO && ', '', 'CMEM lower bound not checked'),
    ('h', 'start + WLCM_CMEM_MIN <= WLCM_CMEM_HI && ', '', 'CMEM PPT end past the window'),
    ('h', '&& size >= WLCM_CMEM_MIN;', '&& size >= 4u;', 'CMEM size not checked'),
    ('h', '\tif ((uint64_t)off + 4u > (uint64_t)cm->cmem_base + cm->cmem_size) { return false; }\n', '',
     'CMEM allowlist ignores the region size'),
]


def mutants(table=None, mode='c2d'):
    """Every C2d (C2e) mutant is caught (static check or the model's C2d (C2e) scenarios); run in parallel."""
    table = C2D_MUTANTS if table is None else table
    import concurrent.futures
    global DRIVER, HEADER
    orig = (DRIVER, HEADER)

    def one(m):
        where, old, new, what = m
        drv, hdr = orig
        if where == 'd':
            assert drv.count(old) == 1, what
            drv = drv.replace(old, new)
        else:
            assert hdr.count(old) == 1, what
            hdr = hdr.replace(old, new)
        return what, drv, hdr
    jobs = [one(m) for m in table]
    built = []
    with tempfile.TemporaryDirectory(prefix=f'fuxi-{mode}-mut-') as tmp:
        for n, (what, drv, hdr) in enumerate(jobs):
            DRIVER, HEADER = drv, hdr
            caught = False
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    static_checks()
            except (AssertionError, ValueError, KeyError, IndexError, TypeError):
                caught = True
            if not caught:
                d = Path(tmp) / str(n)
                d.mkdir()
                (d / 'fuxi-wlan-ce-prod.inc').write_text(production())
                (d / 'fuxi_wlan_ce.h').write_text(hdr)
                built.append((what, d))
            DRIVER, HEADER = orig

        def run(item):
            what, d = item
            b = subprocess.run(CC + ['-I', str(d), str(MODEL), '-o', str(d / 'model')], capture_output=True, text=True)
            if b.returncode:
                return what, True
            r = subprocess.run([str(d / 'model'), mode], capture_output=True, text=True, timeout=600)
            return what, r.returncode != 0
        with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count() or 2) as ex:
            res = list(ex.map(run, built))
    missed = [w for w, c in res if not c]
    tag = 'C2' + mode[2:]
    assert not missed, f'{tag} mutants not caught: {missed}'
    print(f'{tag} mutants: all {len(table)} caught ({len(table) - len(built)} by the static checks, {len(built)} by the '
          f'model\'s {tag} scenarios)')


def header_define(name):
    m = re.search(r'^#define ' + name + r' (.+?)(?:\s*/\*.*)?$', HEADER, re.M)
    assert m, name
    return eval(re.sub(r'\b(0x[0-9a-fA-F]+|\d+)u\b', r'\1', m[1]), {}, {})


def l19_text(name):
    return re.sub(r'//[^\n]*', '', re.sub(r'/\*.*?\*/', '', (L19 / name).read_text(), flags=re.S))


def l19_checks(model, c2dump, c2cdump='', c2ddump=''):
    if not (L19 / 'wmi.h').exists():
        print(f'L19 re-derivation SKIPPED: no Linux ath12k v6.19 tree at {L19} (set ATH12K_SRC)')
        return
    wmi_h, htc_h, hal_h, hw_c, hw_h, ce_c, wmi_c = (l19_text(n) for n in
                                                   ('wmi.h', 'htc.h', 'hal.h', 'hw.c', 'hw.h', 'ce.c', 'wmi.c'))
    # WMI enums (the method of wf/b21.py): command/event groups, ids, tags, services.
    vals = {}

    def ev(expr):
        expr = expr.strip()
        m = re.match(r'(WMI_TLV_CMD|WMI_TLV_EV|WMI_EVT_GRP_START_ID|WMI_CMD_GRP)\((\w+)\)', expr)
        if m:
            return (vals[m[2]] << 12) | 1
        return eval(re.sub(r'\bBIT\((\d+)\)', r'(1 << \1)', expr), {}, vals)

    def run(name):
        body = re.search(r'enum\s+' + name + r'\s*\{(.*?)\};', wmi_h, re.S)[1]
        cur = -1
        for item in body.split(','):
            item = item.strip()
            if not item:
                continue
            if '=' in item:
                k, v = item.split('=', 1)
                cur = ev(v)
                k = k.strip()
            else:
                k, cur = item, cur + 1
            vals[k] = cur
    for e in ('wmi_cmd_group', 'wmi_tlv_cmd_id', 'wmi_tlv_event_id', 'wmi_tlv_tag', 'wmi_tlv_service'):
        run(e)
    pairs = {'WLWMI_CMD_INIT': 'WMI_INIT_CMDID', 'WLWMI_CMD_PDEV_SET_HW_MODE': 'WMI_PDEV_SET_HW_MODE_CMDID',
             'WLWMI_EV_SERVICE_READY': 'WMI_SERVICE_READY_EVENTID', 'WLWMI_EV_READY': 'WMI_READY_EVENTID',
             'WLWMI_EV_SERVICE_AVAILABLE': 'WMI_SERVICE_AVAILABLE_EVENTID',
             'WLWMI_EV_SERVICE_READY_EXT': 'WMI_SERVICE_READY_EXT_EVENTID',
             'WLWMI_EV_PDEV_SET_HW_MODE_RESP': 'WMI_PDEV_SET_HW_MODE_RESP_EVENTID',
             'WLWMI_EV_SERVICE_READY_EXT2': 'WMI_SERVICE_READY_EXT2_EVENTID',
             'WLWMI_EV_REG_CHAN_LIST_CC_EXT': 'WMI_REG_CHAN_LIST_CC_EXT_EVENTID',
             'WLWMI_SVC_EXT_MSG': 'WMI_TLV_SERVICE_EXT_MSG', 'WLWMI_SVC_EXT2_MSG': 'WMI_TLV_SERVICE_EXT2_MSG',
             'WLWMI_SVC_REG_CC_EXT': 'WMI_TLV_SERVICE_REG_CC_EXT_EVENT_SUPPORT'}
    for tag in ('ARRAY_UINT32', 'ARRAY_STRUCT', 'SERVICE_READY_EVENT', 'READY_EVENT', 'INIT_CMD', 'RESOURCE_CONFIG',
                'SERVICE_READY_EXT_EVENT', 'PDEV_SET_HW_MODE_CMD', 'MAC_PHY_CAPABILITIES', 'HW_MODE_CAPABILITIES',
                'SOC_MAC_PHY_HW_MODE_CAPS', 'SERVICE_AVAILABLE_EVENT', 'SERVICE_READY_EXT2_EVENT'):
        pairs['WLWMI_TAG_' + tag] = 'WMI_TAG_' + tag
    for ours, theirs in pairs.items():
        assert header_define(ours) == vals[theirs], (ours, hex(header_define(ours)), theirs, hex(vals[theirs]))
    # Wire struct sizes, nested structs and array sizes resolved from the headers (ETH_ALEN is 6).
    consts = {'ETH_ALEN': 6}
    for k in ('HW_BD_INFO_SIZE', 'WMI_SERVICE_SEGMENT_BM_SIZE32', 'WMI_IE_BITMAP_SIZE', 'ATH_MAX_ANTENNA',
              'ATH12K_WMI_SSID_LEN', 'WLAN_MAX_AC', 'MAX_TX_RATE_VALUES'):
        consts[k] = int(re.search(r'#define ' + k + r'\s+(\d+)', wmi_h)[1])

    def struct_fields(name, text=wmi_h):
        body = re.search(r'struct\s+' + name + r'\s*\{(.*?)\}\s*__packed;', text, re.S)[1]
        out = []
        for decl in body.split(';'):
            decl = ' '.join(decl.split())
            if not decl:
                continue
            m = re.fullmatch(r'(?:__le32|u32|a_sle32|u8|struct (\w+)) (\w+)(?:\[(\w+)\])?', decl)
            assert m, (name, decl)
            n = int(m[3]) if m[3] and m[3].isdigit() else consts[m[3]] if m[3] else 1
            out.append((m[1], m[2], n, decl.startswith('u8 ')))
        return out

    def size(name):
        return sum(n * (size(sub) if sub else 1 if byte else 4) for sub, _, n, byte in struct_fields(name))
    sizes = {'WLWMI_INIT_BYTES': 'wmi_init_cmd', 'WLWMI_RSRC_BYTES': 'ath12k_wmi_resource_config_params',
             'WLWMI_SVC_READY_BYTES': 'wmi_service_ready_event', 'WLWMI_READY_MIN_BYTES': 'ath12k_wmi_ready_event_min_params',
             'WLWMI_SVC_AVAIL_BYTES': 'wmi_service_available_event', 'WLWMI_HW_MODE_BYTES': 'ath12k_wmi_pdev_set_hw_mode_cmd'}
    for ours, theirs in sizes.items():
        assert header_define(ours) == size(theirs), (ours, header_define(ours), theirs, size(theirs))

    def word_names(name):
        out = []
        for sub, field, n, _ in struct_fields(name):
            out += [field] * (size(sub) // 4 if sub else 1) if n == 1 else [f'{field}[{i}]' for i in range(n)]
        return out
    names = word_names('wmi_service_ready_event')
    for field, idx in (('fw_build_vers', 0), ('phy_capability', 7), ('num_rf_chains', 9), ('num_mem_reqs', 18),
                       ('max_num_scan_channels', 19), ('txrx_chainmask', 29)):
        assert names.index(field) == idx, (field, names.index(field), idx)
    ready = word_names('ath12k_wmi_ready_event_min_params')
    assert (ready.index('mac_addr'), ready.index('status'), ready.index('num_extra_mac_addr'),
            ready.index('num_total_peers')) == (6, 8, 10, 11), ready
    # The 73 INIT resource-config words: ath12k_wmi_init_wcn7850 values, through
    # ath12k_wmi_copy_resource_config, at the wire offsets of the struct (2 RF chains, service 281 set).
    target = {k: int(v, 0) for k, v in re.findall(r'#define (TARGET_\w+)\s+\(?(\d+|0x[0-9a-f]+)\)?', hw_h)}
    init = re.search(r'void ath12k_wmi_init_wcn7850\(.*?\n\{(.*?)\n\}', wmi_c, re.S)[1]
    arg = {}
    for k, v in re.findall(r'config->(\w+(?:\[\d\])?) = (.*?);', init):
        v = v.strip()
        arg[k] = 3 if 'num_rf_chains' in v else (target[v] if v in target else int(v, 0))
    copy = re.search(r'ath12k_wmi_copy_resource_config\(.*?\n\{(.*?)\n\}', wmi_c, re.S)[1]
    bits = {k: 1 << int(v) for k, v in re.findall(r'#define (WMI_RSRC_CFG_\w+)\s+BIT\((\d+)\)', wmi_h)}
    shifts = {k: int(v) for k, v in re.findall(r'#define (WMI_RSRC_CFG_HOST_SVC_FLAG_\w+_BIT)\s+(\d+)', wmi_h)}
    wire = {}
    for field, op, expr in re.findall(r'wmi_cfg->(\w+(?:\[\d\])?)\s*(\|?=)\s*(.*?);', copy, re.S):
        expr = ' '.join(expr.split())
        m = re.fullmatch(r'cpu_to_le32\(tg_cfg->(\w+(?:\[\d\])?)\)', expr)
        if m and op == '=':
            wire[field] = arg.get(m[1], 0)
        elif field == 'flag1' and op == '=':
            assert 'tg_cfg->atf_config' in expr and 'BSS_CHANNEL_INFO_64' in expr and 'ACK_RSSI' in expr, expr
            wire[field] = arg.get('atf_config', 0) | bits['WMI_RSRC_CFG_FLAG1_BSS_CHANNEL_INFO_64'] | bits['WMI_RSRC_CFG_FLAG1_ACK_RSSI']
        elif field == 'flags2' and op == '=':
            assert 'peer_metadata_ver' in expr and 'RX_PEER_METADATA_VERSION' in expr, expr
            wire[field] = arg.get('peer_metadata_ver', 0) << 4
        elif field == 'flags2':
            assert expr == 'cpu_to_le32(WMI_RSRC_CFG_FLAGS2_CALC_NEXT_DTIM_COUNT_SET)', expr
            wire[field] |= bits['WMI_RSRC_CFG_FLAGS2_CALC_NEXT_DTIM_COUNT_SET']
        elif field == 'host_service_flags' and op == '=':
            assert 'is_reg_cc_ext_event_supported << WMI_RSRC_CFG_HOST_SVC_FLAG_REG_CC_EXT_SUPPORT_BIT' in expr, expr
            wire[field] = 1 << shifts['WMI_RSRC_CFG_HOST_SVC_FLAG_REG_CC_EXT_SUPPORT_BIT']
        elif field == 'host_service_flags':
            assert 'REO_QREF' in expr, expr  # only with reoq_lut_support, false for WCN7850
        else:
            raise AssertionError((field, op, expr))
    assert 'reoq_lut_support = false' in hw_c.split('.name = "wcn7850 hw2.0"', 1)[1].split('.name =', 1)[0]
    order = word_names('ath12k_wmi_resource_config_params')
    assert len(order) == 73 and order[0] == 'tlv_header'
    expected = [wire.get(f, 0) for f in order]
    expected[0] = (vals['WMI_TAG_RESOURCE_CONFIG'] << 16) | 288
    got = [int(line.split()[1]) for line in model.splitlines()]
    assert got == expected, [(order[i], got[i], expected[i]) for i in range(73) if got[i] != expected[i]]
    # HTC (htc.h): flags, message ids, connection flags, limits, service ids, endpoint count.
    def hdef(name, text=htc_h):
        m = re.search(r'\b' + name + r'\s*=?\s*(BIT\(\d+\)|0x[0-9A-Fa-f]+|\d+)', text)
        assert m, name
        v = m[1]
        return 1 << int(v[4:-1]) if v.startswith('BIT') else int(v, 0)
    for ours, theirs in (('WLHTC_FLAG_NEED_CREDIT', 'ATH12K_HTC_FLAG_NEED_CREDIT_UPDATE'),
                         ('WLHTC_FLAG_TRAILER', 'ATH12K_HTC_FLAG_TRAILER_PRESENT'),
                         ('WLHTC_MSG_READY', 'ATH12K_HTC_MSG_READY_ID'), ('WLHTC_MSG_CONNECT', 'ATH12K_HTC_MSG_CONNECT_SERVICE_ID'),
                         ('WLHTC_MSG_CONNECT_RESP', 'ATH12K_HTC_MSG_CONNECT_SERVICE_RESP_ID'),
                         ('WLHTC_MSG_SETUP_COMPLETE_EX', 'ATH12K_HTC_MSG_SETUP_COMPLETE_EX_ID'),
                         ('WLHTC_CONN_DISABLE_FLOW', 'ATH12K_HTC_CONN_FLAGS_DISABLE_CREDIT_FLOW_CTRL'),
                         ('WLHTC_MAX_LEN', 'ATH12K_HTC_MAX_LEN'), ('WLHTC_CTL_MAX', 'ATH12K_HTC_MAX_CTRL_MSG_LEN'),
                         ('WLHTC_RECORD_CREDITS', 'ATH12K_HTC_RECORD_CREDITS')):
        assert header_define(ours) == hdef(theirs), (ours, theirs)
    assert re.search(r'#define ATH12K_HTC_CONN_FLAGS_RECV_ALLOC\s+GENMASK\(15, 8\)', htc_h)
    eps = re.search(r'enum ath12k_htc_ep_id \{(.*?)\};', htc_h, re.S)[1]
    assert eps.split('ATH12K_HTC_EP_COUNT')[0].count('ATH12K_HTC_EP_') == 1 + 9 and header_define('WLHTC_EPS') == 9
    for ours, group, idx in (('WLHTC_SVC_RSVD_CTRL', 0, 1), ('WLHTC_SVC_WMI', 1, 0), ('WLHTC_SVC_HTT', 3, 0)):
        assert header_define(ours) == (group << 8 | idx), ours
    assert re.search(r'ATH12K_HTC_SVC_ID_HTT_DATA_MSG\s*= SVC\(ATH12K_HTC_SVC_GRP_HTT, 0\)', htc_h)
    assert re.search(r'ATH12K_HTC_SVC_GRP_HTT = 3', htc_h) and re.search(r'ATH12K_HTC_SVC_GRP_WMI = 1', htc_h)
    for struct, ours in (('ath12k_htc_ready', 'WLHTC_READY_BYTES'), ('ath12k_htc_conn_svc_resp', 'WLHTC_CONNECT_RESP_BYTES'),
                         ('ath12k_htc_conn_svc', 'WLHTC_CONNECT_BYTES'),
                         ('ath12k_htc_setup_complete_extended', 'WLHTC_SETUP_COMPLETE_BYTES')):
        body = re.search(r'struct ' + struct + r' \{(.*?)\}', htc_h, re.S)[1]
        assert 4 * body.count('__le32') == header_define(ours), struct
    # CE: WCN7850 host CE config (ce.c), ring ids and offsets (hal.h, hw.c), shadow base.
    cfg = re.search(r'ath12k_host_ce_config_wcn7850\[\] = \{(.*?)\n\};', ce_c, re.S)[1]
    ces = [dict(re.findall(r'\.(\w+) = (\d+)', e)) for e in re.findall(r'\{(.*?)\}', cfg, re.S)]
    assert [(int(c['src_nentries']), int(c['dest_nentries'])) for c in ces[:5]] == [(16, 0), (0, 512), (0, 64), (32, 0), (2048, 0)]
    assert [int(c['src_sz_max']) for c in ces[:4]] == [header_define('WLCE_BUF')] * 4 and int(ces[4]['src_sz_max']) == 256
    assert all(int(c['src_nentries']) == 0 and int(c['dest_nentries']) == 0 for c in ces[5:]) and len(ces) == 9
    plan = re.search(r'plan\[WLCE_NRING\] = \{(.*?)\n\t\};', HEADER, re.S)[1]
    rows = re.findall(r'\{(\d), WLCE_(SRC|DST|STS), (\d+),', plan)
    assert [(int(c), k, int(n)) for c, k, n in rows] == [(0, 'SRC', 16), (1, 'DST', 512), (1, 'STS', 512), (2, 'DST', 64),
                                                      (2, 'STS', 64), (3, 'SRC', 32), (4, 'SRC', 2048)], rows
    ids = {k: int(v) for k, v in re.findall(r'(HAL_SRNG_RING_ID_CE0_(?:SRC|DST|DST_STATUS)) = (\d+)', hal_h)}
    assert ids == {'HAL_SRNG_RING_ID_CE0_SRC': 64, 'HAL_SRNG_RING_ID_CE0_DST': 81, 'HAL_SRNG_RING_ID_CE0_DST_STATUS': 100}
    assert '#define WLCE_ID_SRC(n) (64u + (n))' in HEADER and '#define WLCE_ID_DST(n) (81u + (n))' in HEADER
    assert '#define WLCE_ID_STS(n) (100u + (n))' in HEADER
    for ours, theirs in (('WLCE_SHADOW_BASE', 'HAL_SHADOW_BASE_ADDR'), ('WLCE_SHADOW_REGS', 'HAL_SHADOW_NUM_REGS'),
                         ('WLCE_STS_R0', 'HAL_CE_DST_STATUS_RING_BASE_LSB'), ('WLCE_DEST_CTRL', 'HAL_CE_DST_RING_CTRL'),
                         ('WLCE_R2_HP', 'HAL_CE_DST_RING_HP'), ('WLCE_STS_R2_HP', 'HAL_CE_DST_STATUS_RING_HP'),
                         ('WLCE_REG_BASE', 'HAL_CE_WFSS_CE_REG_BASE'), ('WLCE_RING_SIZE_MAX', 'HAL_CE_SRC_RING_BASE_MSB_RING_SIZE')):
        assert header_define(ours) == int(re.search(r'#define ' + theirs + r'\s+(0x[0-9a-fA-F]+|\d+)', hal_h)[1], 0), ours
    for ours, theirs in (('WLCE_MISC_LOOPCNT_DIS', 'HAL_TCL1_RING_MISC_MSI_LOOPCNT_DISABLE'),
                         ('WLCE_MISC_SRNG_ENABLE', 'HAL_TCL1_RING_MISC_SRNG_ENABLE')):
        assert header_define(ours) == 1 << int(re.search(r'#define ' + theirs + r'\s+BIT\((\d+)\)', hal_h)[1]), ours
    regs = dict(re.findall(r'\.(\w+) = (0x[0-9a-fA-F]+),', hw_c.split('wcn7850_regs = {', 1)[1].split('};', 1)[0]))
    regs = {k: int(v, 16) for k, v in regs.items()}
    assert (regs['hal_umac_ce0_src_reg_base'], regs['hal_umac_ce0_dest_reg_base'], regs['hal_umac_ce1_src_reg_base']) == \
        (header_define('WLCE_REG_BASE'), 0x01b81000, 0x01b82000)
    tcl = regs['hal_tcl1_ring_base_lsb']
    for ours, field in (('WLCE_R0_BASE_MSB', 'hal_tcl1_ring_base_msb'), ('WLCE_R0_ID', 'hal_tcl1_ring_id'),
                        ('WLCE_R0_MISC', 'hal_tcl1_ring_misc'), ('WLCE_R0_TP_ADDR_LSB', 'hal_tcl1_ring_tp_addr_lsb'),
                        ('WLCE_R0_TP_ADDR_MSB', 'hal_tcl1_ring_tp_addr_msb'),
                        ('WLCE_R0_CONS_IX0', 'hal_tcl1_ring_consumer_int_setup_ix0'),
                        ('WLCE_R0_CONS_IX1', 'hal_tcl1_ring_consumer_int_setup_ix1')):
        assert header_define(ours) == regs[field] - tcl, ours
    reo = regs['hal_reo1_ring_base_lsb']
    for ours, field in (('WLCE_R0_BASE_MSB', 'hal_reo1_ring_base_msb'), ('WLCE_R0_ID', 'hal_reo1_ring_id'),
                        ('WLCE_R0_MISC', 'hal_reo1_ring_misc'), ('WLCE_R0_HP_ADDR_LSB', 'hal_reo1_ring_hp_addr_lsb'),
                        ('WLCE_R0_HP_ADDR_MSB', 'hal_reo1_ring_hp_addr_msb'),
                        ('WLCE_R0_PROD_INT', 'hal_reo1_ring_producer_int_setup')):
        assert header_define(ours) == regs[field] - reo, ours
    desc = l19_text('hal_desc.h')
    for struct, dw in (('hal_ce_srng_src_desc', 4), ('hal_ce_srng_dest_desc', 2), ('hal_ce_srng_dst_status_desc', 4)):
        assert re.search(r'struct ' + struct + r' \{(.*?)\}', desc, re.S)[1].count('__le32') == dw, struct
    assert re.search(r'#define WMI_PDEV_ID_SOC\s+0', wmi_h) and header_define('WLWMI_PDEV_ID_SOC') == 0
    assert re.search(r'WMI_HOST_HW_MODE_DBS\s+= 1', wmi_h) and header_define('WLWMI_HW_MODE_DBS') == 1
    print('L19 re-derivation: WMI ids/tags/services, wire sizes, SERVICE_READY words, all 73 INIT words '
          '(init_wcn7850 through copy_resource_config), HTC constants, CE geometry/ids/offsets/shadow equal the header')
    assert int(re.findall(r'\.src_sz_max = (\d+)', cfg)[4]) == header_define('WLCE_CE4_BUF')
    l19_c2(vals, ev, struct_fields, size, word_names, wmi_h, wmi_c, c2dump, c2cdump, c2ddump)


def fn_body(text, head):
    """The C function whose definition starts with HEAD at column 0, up to its closing brace at column 0."""
    i = text.index('\n' + head) + 1
    return text[i:text.index('\n}\n', i)]


def l19_c2(vals, ev, struct_fields, size, word_names, wmi_h, wmi_c, dump, c2cdump='', c2ddump=''):
    """C2: ids, tags, sizes, field offsets, flags and every encoded word, re-derived from L19."""
    mac_c, reg_c, core_h, dp_h, dp_tx_c, dp_rx_c, hw_c = (l19_text(n) for n in
                                                         ('mac.c', 'reg.c', 'core.h', 'dp.h', 'dp_tx.c', 'dp_rx.c', 'hw.c'))
    # Every enum of wmi.h (scan event types, phy modes, vdev types, STA PS values, ...), item by item.
    for body in re.findall(r'enum\s+\w+\s*\{(.*?)\};', wmi_h, re.S):
        cur = -1
        for item in body.split(','):
            item = item.strip()
            if not item:
                continue
            try:
                if '=' in item:
                    k, v = item.split('=', 1)
                    cur = ev(v)
                    k = k.strip()
                else:
                    k, cur = item, cur + 1
            except Exception:
                break
            vals.setdefault(k, cur)

    def define(name, text=wmi_h):
        v = re.search(r'#define ' + name + r'\s+(.+?)\s*$', text, re.M)[1].strip()
        if v.startswith('(') and v.endswith(')') and v.count('(') == 1:
            v = v[1:-1].strip()
        m = re.fullmatch(r'BIT\((\d+)\)', v)
        if m:
            return 1 << int(m[1])
        m = re.fullmatch(r'GENMASK\((\d+), (\d+)\)', v)
        if m:
            return (int(m[1]), int(m[2]))
        return int(v, 0)
    ids = {'WLWMI_CMD_START_SCAN': 'WMI_START_SCAN_CMDID', 'WLWMI_CMD_STOP_SCAN': 'WMI_STOP_SCAN_CMDID',
           'WLWMI_CMD_SCAN_CHAN_LIST': 'WMI_SCAN_CHAN_LIST_CMDID', 'WLWMI_CMD_VDEV_CREATE': 'WMI_VDEV_CREATE_CMDID',
           'WLWMI_CMD_VDEV_SET_PARAM': 'WMI_VDEV_SET_PARAM_CMDID', 'WLWMI_CMD_STA_POWERSAVE_MODE': 'WMI_STA_POWERSAVE_MODE_CMDID',
           'WLWMI_CMD_STA_POWERSAVE_PARAM': 'WMI_STA_POWERSAVE_PARAM_CMDID', 'WLWMI_EV_SCAN': 'WMI_SCAN_EVENTID',
           'WLWMI_EV_CHAN_INFO': 'WMI_CHAN_INFO_EVENTID', 'WLWMI_EV_MGMT_RX': 'WMI_MGMT_RX_EVENTID',
           'WLWMI_EV_PDEV_RSSI_DBM_CONVERSION': 'WMI_PDEV_RSSI_DBM_CONVERSION_PARAMS_INFO_EVENTID',
           'WLWMI_VDEV_PARAM_NSS': 'WMI_VDEV_PARAM_NSS', 'WLWMI_MODE_11A': 'MODE_11A', 'WLWMI_MODE_11G': 'MODE_11G',
           'WLWMI_VDEV_TYPE_STA': 'WMI_VDEV_TYPE_STA', 'WLWMI_VDEV_SUBTYPE_NONE': 'WMI_VDEV_SUBTYPE_NONE',
           'WLWMI_MBSSID_NON_MBSSID_AP': 'WMI_VDEV_MBSSID_FLAGS_NON_MBSSID_AP',
           'WLWMI_BAND_2G': 'WMI_TPC_CHAINMASK_CONFIG_BAND_2G', 'WLWMI_BAND_5G': 'WMI_TPC_CHAINMASK_CONFIG_BAND_5G',
           'WLWMI_NUM_BANDS': 'WMI_NUM_SUPPORTED_BAND_MAX', 'WLWMI_HOST_2GHZ_CAP': 'WMI_HOST_WLAN_2GHZ_CAP',
           'WLWMI_HOST_5GHZ_CAP': 'WMI_HOST_WLAN_5GHZ_CAP',
           'WLWMI_STA_PS_PARAM_RX_WAKE_POLICY': 'WMI_STA_PS_PARAM_RX_WAKE_POLICY',
           'WLWMI_STA_PS_PARAM_TX_WAKE_THRESHOLD': 'WMI_STA_PS_PARAM_TX_WAKE_THRESHOLD',
           'WLWMI_STA_PS_PARAM_PSPOLL_COUNT': 'WMI_STA_PS_PARAM_PSPOLL_COUNT',
           'WLWMI_STA_PS_RX_WAKE_POLICY_WAKE': 'WMI_STA_PS_RX_WAKE_POLICY_WAKE',
           'WLWMI_STA_PS_TX_WAKE_THRESHOLD_ALWAYS': 'WMI_STA_PS_TX_WAKE_THRESHOLD_ALWAYS',
           'WLWMI_STA_PS_PSPOLL_COUNT_NO_MAX': 'WMI_STA_PS_PSPOLL_COUNT_NO_MAX',
           'WLWMI_STA_PS_MODE_DISABLED': 'WMI_STA_PS_MODE_DISABLED', 'WLWMI_SCAN_EV_STARTED': 'WMI_SCAN_EVENT_STARTED',
           'WLWMI_SCAN_EV_COMPLETED': 'WMI_SCAN_EVENT_COMPLETED', 'WLWMI_SCAN_EV_BSS_CHANNEL': 'WMI_SCAN_EVENT_BSS_CHANNEL',
           'WLWMI_SCAN_EV_FOREIGN_CHAN': 'WMI_SCAN_EVENT_FOREIGN_CHAN', 'WLWMI_SCAN_EV_DEQUEUED': 'WMI_SCAN_EVENT_DEQUEUED',
           'WLWMI_SCAN_EV_START_FAILED': 'WMI_SCAN_EVENT_START_FAILED', 'WLWMI_SCAN_REASON_CANCELLED': 'WMI_SCAN_REASON_CANCELLED'}
    for tag in ('ARRAY_BYTE', 'ARRAY_FIXED_STRUCT', 'SCAN_EVENT', 'CHAN_INFO_EVENT', 'MGMT_RX_HDR', 'START_SCAN_CMD',
                'STOP_SCAN_CMD', 'SCAN_CHAN_LIST_CMD', 'CHANNEL', 'VDEV_CREATE_CMD', 'VDEV_SET_PARAM_CMD',
                'STA_POWERSAVE_MODE_CMD', 'STA_POWERSAVE_PARAM_CMD', 'VDEV_TXRX_STREAMS'):
        ids['WLWMI_TAG_' + tag] = 'WMI_TAG_' + tag
    for ours, theirs in ids.items():
        assert header_define(ours) == vals[theirs], (ours, header_define(ours), theirs, vals[theirs])
    for ours, theirs in (('WLWMI_CHAN_INFO_PASSIVE', 'WMI_CHAN_INFO_PASSIVE'), ('WLWMI_CHAN_INFO_DFS', 'WMI_CHAN_INFO_DFS'),
                         ('WLWMI_CHAN_INFO_ALLOW_HE', 'WMI_CHAN_INFO_ALLOW_HE'), ('WLWMI_SCAN_FLAG_PASSIVE', 'WMI_SCAN_FLAG_PASSIVE'),
                         ('WLWMI_SCAN_CHAN_STAT_EVENT', 'WMI_SCAN_CHAN_STAT_EVENT'), ('WLWMI_SCAN_STOP_ONE', 'WMI_SCAN_STOP_ONE'),
                         ('WLWMI_APPEND_CHAN_LIST', 'WMI_APPEND_TO_EXISTING_CHAN_LIST_FLAG'),
                         ('WLWMI_CHAN_INFO_START_RESP', 'WMI_CHAN_INFO_START_RESP'), ('WLWMI_CHAN_INFO_END_RESP', 'WMI_CHAN_INFO_END_RESP'),
                         ('WLWMI_RX_ERR_CRC', 'WMI_RX_STATUS_ERR_CRC'), ('WLWMI_RX_ERR_DECRYPT', 'WMI_RX_STATUS_ERR_DECRYPT'),
                         ('WLWMI_RX_ERR_KEY_CACHE_MISS', 'WMI_RX_STATUS_ERR_KEY_CACHE_MISS'), ('WLWMI_SCAN_ID', 'ATH12K_SCAN_ID')):
        assert header_define(ours) == define(theirs), (ours, theirs)
    assert define('WMI_CHAN_INFO_MODE') == (5, 0) and define('WMI_SCAN_DWELL_MODE_MASK') == (23, 21)
    assert [define('WMI_CHAN_REG_INFO1_' + f) for f in ('MIN_PWR', 'MAX_PWR', 'MAX_REG_PWR', 'REG_CLS')] == \
        [(7, 0), (15, 8), (23, 16), (31, 24)]
    assert [define('WMI_CHAN_REG_INFO2_' + f) for f in ('ANT_MAX', 'MAX_TX_PWR')] == [(7, 0), (15, 8)]
    assert define('ATH12K_DEFAULT_NOISE_FLOOR', core_h) == header_define('WLWMI_NOISE_FLOOR')
    assert define('ATH12K_MAX_5GHZ_CHAN', core_h) == header_define('WLWMI_MAX_5GHZ_CHAN')
    half = define('ATH12K_HALF_20MHZ_BW', core_h)
    assert (define('ATH12K_6GHZ_MIN_CENTER', core_h) - half, define('ATH12K_6GHZ_MAX_CENTER', core_h) + half) == \
        (header_define('WLWMI_MIN_6GHZ_FREQ'), header_define('WLWMI_MAX_6GHZ_FREQ'))
    # Sizes (commands with their TLV header, events as the TLV value) and the event fields C2 reads.
    for ours, theirs in (('WLWMI_CHAN_LIST_BYTES', 'wmi_scan_chan_list_cmd'), ('WLWMI_CHANNEL_BYTES', 'ath12k_wmi_channel_params'),
                         ('WLWMI_VDEV_CREATE_BYTES', 'wmi_vdev_create_cmd'),
                         ('WLWMI_TXRX_STREAMS_BYTES', 'ath12k_wmi_vdev_txrx_streams_params'),
                         ('WLWMI_VDEV_SET_PARAM_BYTES', 'wmi_vdev_set_param_cmd'),
                         ('WLWMI_STA_PS_PARAM_BYTES', 'wmi_sta_powersave_param_cmd'), ('WLWMI_STA_PS_MODE_BYTES', 'wmi_pdev_set_ps_mode_cmd'),
                         ('WLWMI_START_SCAN_BYTES', 'wmi_start_scan_cmd'), ('WLWMI_STOP_SCAN_BYTES', 'wmi_stop_scan_cmd'),
                         ('WLWMI_MAC_ADDR_BYTES', 'ath12k_wmi_mac_addr_params'), ('WLWMI_SCAN_EVENT_BYTES', 'wmi_scan_event'),
                         ('WLWMI_CHAN_INFO_BYTES', 'wmi_chan_info_event'), ('WLWMI_MGMT_RX_HDR_BYTES', 'ath12k_wmi_mgmt_rx_params')):
        assert header_define(ours) == size(theirs), (ours, header_define(ours), theirs, size(theirs))
    se, ci, mr = word_names('wmi_scan_event'), word_names('wmi_chan_info_event'), word_names('ath12k_wmi_mgmt_rx_params')
    assert [se.index(f) for f in ('event_type', 'reason', 'channel_freq', 'scan_id', 'vdev_id')] == [0, 1, 2, 4, 5]
    assert [ci.index(f) for f in ('freq', 'cmd_flags', 'noise_floor', 'vdev_id')] == [1, 2, 3, 13]
    assert [mr.index(f) for f in ('channel', 'snr', 'buf_len', 'status', 'pdev_id', 'chan_freq')] == [0, 1, 4, 5, 15, 16]
    body = re.search(r'struct ath12k_wmi_mac_phy_caps_params \{(.*?)\} __packed;', wmi_h, re.S)[1]
    caps = [d.split()[-1].split('[')[0] for d in body.split(';') if d.strip()]
    assert [caps.index(f) for f in ('pdev_and_hw_link_ids', 'supported_bands', 'tx_chain_mask_2g', 'rx_chain_mask_2g',
                                    'tx_chain_mask_5g', 'rx_chain_mask_5g')] == [1, 4, 12, 13, 20, 21]
    # The preferred hw mode whose PHYs give the streams and NSS: the priority table by mode id, the single-pdev
    # start (SINGLE, no PHY), strictly better wins, the PHY index L19 assigns (fls of the entry before), the PHYs
    # one per bit of its map, 2 GHz masks else 5 GHz.
    prim = re.search(r'ath12k_hw_mode_pri_map\[\] = \{(.*?)\};', wmi_c, re.S)[1]
    table = {vals[m]: vals[m + '_PRI'] for m in re.findall(r'PRIMAP\((\w+)\)', prim) if m != 'WMI_HOST_HW_MODE_MAX'}
    ours = [int(x) for x in re.search(r'static const uint8_t pri\[\] = \{ ([0-9, ]+) \};', HEADER)[1].split(',')]
    assert table == dict(enumerate(ours)) and len(ours) == vals['WMI_HOST_HW_MODE_MAX'], (table, ours)
    assert header_define('WLWMI_HW_MODE_PRI_SINGLE') == vals['WMI_HOST_HW_MODE_SINGLE_PRI']
    assert header_define('WLWMI_HW_MODE_PRI_NONE') == vals['WMI_HOST_HW_MODE_MAX_PRI']
    assert re.search(r'preferred_hw_mode = WMI_HOST_HW_MODE_MAX;\s+if \(ab->hw_params->single_pdev_only\)\s+'
                     r'ab->wmi_ab\.preferred_hw_mode = WMI_HOST_HW_MODE_SINGLE;', fn_body(wmi_c, 'int ath12k_wmi_attach('))
    modes = fn_body(wmi_c, 'static int ath12k_wmi_hw_mode_caps(')
    assert 'if (mode >= WMI_HOST_HW_MODE_MAX)\n\t\t\tcontinue;' in modes
    assert 'if (ath12k_hw_mode_pri_map[mode] < ath12k_hw_mode_pri_map[pref]) {' in modes
    pull = fn_body(wmi_c, 'ath12k_pull_mac_phy_cap_svc_ready_ext(')
    assert re.search(r'phy_map = le32_to_cpu\(wmi_hw_mode_caps\[hw_idx\]\.phy_id_map\);\s+phy_idx = fls\(phy_map\);', pull)
    assert 'phy_idx += phy_id;' in pull and 'pdev_cap->supported_bands |= le32_to_cpu(mac_caps->supported_bands);' in pull
    assert re.search(r'supported_bands\) & WMI_HOST_WLAN_2GHZ_CAP\) \{\s+pdev_cap->tx_chain_mask = le32_to_cpu\(mac_caps->'
                     r'tx_chain_mask_2g\);.*?\} else if \(le32_to_cpu\(mac_caps->supported_bands\) & WMI_HOST_WLAN_5GHZ_CAP\)'
                     r' \{.*?pdev_cap->tx_chain_mask = le32_to_cpu\(mac_caps->tx_chain_mask_5g\);', pull, re.S)
    reg = fn_body(wmi_c, 'static int ath12k_wmi_ext_soc_hal_reg_caps_parse(')
    assert 'phy_id_map = le32_to_cpu(svc_rdy_ext->pref_hw_mode_caps.phy_id_map);' in reg
    assert re.search(r'while \(phy_id_map && soc->num_radios < MAX_RADIOS\) \{.*?hw_mode_id, soc->num_radios,.*?'
                     r'phy_id_map >>= 1;', reg, re.S)
    # L19 code facts C2 relies on.
    tlvhdr = fn_body(wmi_c, 'static __le32 ath12k_wmi_tlv_cmd_hdr(')
    assert 'len - TLV_HDR_SIZE' in tlvhdr
    chan = fn_body(wmi_c, 'int ath12k_wmi_send_scan_chan_list_cmd(')
    assert 'tlv->header = ath12k_wmi_tlv_cmd_hdr(WMI_TAG_ARRAY_STRUCT,' in chan  # the array length is 4 short
    assert re.search(r'if \(channel_arg->allow_he\)\s+chan_info->info \|= cpu_to_le32\(WMI_CHAN_INFO_ALLOW_HE\);\s+else if', chan)
    vdev = fn_body(wmi_c, 'int ath12k_wmi_vdev_create(')
    assert 'tlv->header = ath12k_wmi_tlv_hdr(WMI_TAG_ARRAY_STRUCT, len);' in vdev  # exact length here
    assert 'cmd->num_cfg_txrx_streams = cpu_to_le32(WMI_NUM_SUPPORTED_BAND_MAX);' in vdev
    assert 'cmd->vdev_stats_id_valid = cpu_to_le32(BIT(0));' in vdev
    upd = fn_body(reg_c, 'int ath12k_reg_update_chan_list(')
    for fact in ('ch->allow_he = true;', 'ch->is_chan_passive |= ch->dfs_set;', 'ch->phy_mode = MODE_11G;',
                 'ch->phy_mode = MODE_11A;', 'ch->cfreq1 = channel->center_freq;', 'ch->maxpower = channel->max_power * 2;'):
        assert fact in upd, fact
    assert 'arg->pdev_id = ar->pdev->pdev_id;' in upd
    soc = fn_body(wmi_c, 'static int ath12k_wmi_ext_soc_hal_reg_caps_parse(')
    assert re.search(r'if \(soc->hw_params->single_pdev_only\) \{\s+soc->num_radios = 1;\s+soc->pdevs\[0\]\.pdev_id = 0;', soc)
    assert 'single_pdev_only = true' in hw_c.split('.name = "wcn7850 hw2.0"', 1)[1].split('.name =', 1)[0]
    init = fn_body(wmi_c, 'void ath12k_wmi_start_scan_init(')
    arg = {k: v for k, v in re.findall(r'arg->(\w+) = (\w+);', init)}
    notify = sum(vals[n] for n in re.findall(r'WMI_SCAN_EVENT_\w+', init.split('notify_scan_events =', 1)[1].split(';', 1)[0]))
    start = fn_body(wmi_c, 'int ath12k_wmi_send_scan_start_cmd(')
    assert 'cmd->scan_priority' not in start  # filled into arg only: 0 on the wire
    copied = dict(re.findall(r'cmd->(\w+) = cpu_to_le32\(arg->(\w+)\);', start))
    hw_scan = fn_body(mac_c, 'static int ath12k_mac_initiate_hw_scan(')
    assert 'arg->scan_f_passive = 1;' in hw_scan and 'arg->scan_id = ATH12K_SCAN_ID;' in hw_scan
    assert 'wait_for_completion_timeout(&ar->scan.started, 1 * HZ)' in fn_body(mac_c, 'static int ath12k_start_scan(')
    assert re.search(r'#define ATH12K_MAC_SCAN_TIMEOUT_MSECS 200', mac_c) and 'arg->max_scan_time +' in mac_c
    stop = fn_body(mac_c, 'static int ath12k_scan_stop(')
    assert re.search(r'struct ath12k_wmi_scan_cancel_arg arg = \{\s+\.req_type = WLAN_SCAN_CANCEL_SINGLE,\s+'
                     r'\.scan_id = ATH12K_SCAN_ID,\s+\};', stop) and 'arg.pdev_id = ar->pdev->pdev_id;' in stop
    assert 'arg.vdev_id' not in stop and 'arg.requester' not in stop  # 0 on the wire
    create = fn_body(mac_c, 'int ath12k_mac_vdev_create(')
    order = [create.index(s) for s in ('ath12k_wmi_vdev_create(ar, arvif->bssid, &vdev_arg)', 'WMI_VDEV_PARAM_NSS, nss)',
                                       'WMI_STA_PS_PARAM_RX_WAKE_POLICY;', 'WMI_STA_PS_PARAM_TX_WAKE_THRESHOLD;',
                                       'WMI_STA_PS_PARAM_PSPOLL_COUNT;', 'ath12k_wmi_pdev_set_ps_mode(ar, arvif->vdev_id, false)')]
    assert order == sorted(order), order
    for p, v in (('RX_WAKE_POLICY', 'WMI_STA_PS_RX_WAKE_POLICY_WAKE'), ('TX_WAKE_THRESHOLD', 'WMI_STA_PS_TX_WAKE_THRESHOLD_ALWAYS'),
                 ('PSPOLL_COUNT', 'WMI_STA_PS_PSPOLL_COUNT_NO_MAX')):
        assert re.search(r'param_id = WMI_STA_PS_PARAM_' + p + r';\s+param_value = ' + v + ';', create), p
    assert 'nss = hweight32(ar->cfg_tx_chainmask) ? : 1;' in create
    mgmt = fn_body(wmi_c, 'static void ath12k_mgmt_rx_event(')
    assert re.search(r'WMI_RX_STATUS_ERR_DECRYPT \|\s+WMI_RX_STATUS_ERR_KEY_CACHE_MISS \|\s+WMI_RX_STATUS_ERR_CRC', mgmt)
    assert 'rx_ev.channel >= 1 && rx_ev.channel <= 14' in mgmt and 'rx_ev.channel <= ATH12K_MAX_5GHZ_CHAN' in mgmt
    assert 'status->signal = rx_ev.snr + noise_floor;' in mgmt
    assert 'ar->rssi_info.min_nf_dbm = ATH12K_DEFAULT_NOISE_FLOOR;' in mac_c and 'ar->rssi_info.temp_offset = 0;' in mac_c
    assert 'if (!parse->frame_buf_done) {' in fn_body(wmi_c, 'static int ath12k_wmi_tlv_mgmt_rx_parse(')
    # HTT version request (dp.h, dp_tx.c, dp_rx.c).
    htt_req = fn_body(dp_tx_c, 'int ath12k_dp_tx_htt_h2t_ver_req_msg(')
    assert 'HTT_OPTION_TCL_METADATA_VER_V2' in htt_req and 'ath12k_ftm_mode ?' in htt_req
    assert define('HTT_TAG_TCL_METADATA_VERSION', dp_h) == header_define('WLHTT_TAG_TCL_METADATA_VERSION')
    assert define('HTT_TCL_METADATA_VER_SZ', dp_h) == header_define('WLHTT_TCL_METADATA_VER_SZ')
    assert define('HTT_OPTION_TCL_METADATA_VER_V2', dp_h) == header_define('WLHTT_TCL_METADATA_VER_V2')
    assert [define('HTT_OPTION_' + f, dp_h) for f in ('TAG', 'LEN', 'VALUE')] == [(7, 0), (15, 8), (31, 16)]
    assert define('HTT_TARGET_VERSION_MAJOR', dp_h) == header_define('WLHTT_VERSION_MAJOR')
    assert [define('HTT_T2H_' + f, dp_h) for f in ('MSG_TYPE', 'VERSION_CONF_MINOR', 'VERSION_CONF_MAJOR')] == [(7, 0), (15, 8), (23, 16)]
    assert re.search(r'enum htt_t2h_msg_type \{\s+HTT_T2H_MSG_TYPE_VERSION_CONF,', dp_h)
    assert re.search(r'HTT_H2T_MSG_TYPE_VERSION_REQ\s+= 0,', dp_h) and 'major != HTT_TARGET_VERSION_MAJOR' in htt_req
    assert re.search(r'#define HTT_TARGET_VERSION_TIMEOUT_HZ \(3 \* HZ\)', dp_tx_c)
    assert 4 * re.search(r'struct htt_ver_req_cmd \{(.*?)\}', dp_h, re.S)[1].count('__le32') == header_define('WLHTT_VER_REQ_BYTES')
    assert 'case HTT_T2H_MSG_TYPE_VERSION_CONF:' in dp_rx_c
    # Every encoded word against L19's struct order and values.
    words = {}
    for line in dump.splitlines():
        k, i, v = line.split()
        words.setdefault(k, []).append(int(v))
    tlv = lambda tag, n: (vals['WMI_TAG_' + tag] << 16) | n  # noqa: E731

    def chan_info(mhz):
        dfs = 5260 <= mhz <= 5320 or 5500 <= mhz <= 5720
        return (vals['MODE_11G'] if mhz < 3000 else vals['MODE_11A']) | define('WMI_CHAN_INFO_PASSIVE') | \
            define('WMI_CHAN_INFO_ALLOW_HE') | (define('WMI_CHAN_INFO_DFS') if dfs else 0)
    mhz = [2412 + 5 * i for i in range(13)] + [5180 + 20 * i for i in range(8)] + [5500 + 20 * i for i in range(12)] + \
        [5745 + 20 * i for i in range(5)]
    assert len(mhz) == 38 and sum(5260 <= f <= 5320 or 5500 <= f <= 5720 for f in mhz) == 16
    power = header_define('WLWMI_CHAN_POWER_HALF_DB')
    want = [vals['WMI_SCAN_CHAN_LIST_CMDID'], tlv('SCAN_CHAN_LIST_CMD', size('wmi_scan_chan_list_cmd') - 4), 38, 0, 0,
            tlv('ARRAY_STRUCT', 38 * size('ath12k_wmi_channel_params') - 4)]
    for f in mhz:
        want += [tlv('CHANNEL', size('ath12k_wmi_channel_params') - 4), f, f, 0, chan_info(f), power << 8 | power << 16, power << 8]
    assert words['chanlist'] == want, [(i, a, b) for i, (a, b) in enumerate(zip(words['chanlist'], want)) if a != b][:5]
    order = word_names('wmi_vdev_create_cmd')
    v = dict.fromkeys(order, 0)
    v.update(tlv_header=tlv('VDEV_CREATE_CMD', size('wmi_vdev_create_cmd') - 4), vdev_type=vals['WMI_VDEV_TYPE_STA'],
             vdev_subtype=vals['WMI_VDEV_SUBTYPE_NONE'], num_cfg_txrx_streams=vals['WMI_NUM_SUPPORTED_BAND_MAX'],
             mbssid_flags=vals['WMI_VDEV_MBSSID_FLAGS_NON_MBSSID_AP'], vdev_stats_id_valid=1)
    mac_words = [0x58554602, 0x0149]  # 02:46:55:58:49:01 + 2 pad, little endian
    want = [vals['WMI_VDEV_CREATE_CMDID']] + [mac_words[order[:i].count('vdev_macaddr')] if f == 'vdev_macaddr' else v[f]
                                              for i, f in enumerate(order)]
    want += [tlv('ARRAY_STRUCT', 2 * size('ath12k_wmi_vdev_txrx_streams_params'))]
    for band in ('WMI_TPC_CHAINMASK_CONFIG_BAND_2G', 'WMI_TPC_CHAINMASK_CONFIG_BAND_5G'):
        want += [tlv('VDEV_TXRX_STREAMS', size('ath12k_wmi_vdev_txrx_streams_params') - 4), vals[band], 2, 2]
    assert words['vdev'] == want, (words['vdev'], want)
    order = word_names('wmi_start_scan_cmd')
    fixed = dict.fromkeys(order, 0)
    for field, src in copied.items():
        if src in arg:
            fixed[field] = int(arg[src], 0) if arg[src].isdigit() else 0
    fixed.update(tlv_header=tlv('START_SCAN_CMD', size('wmi_start_scan_cmd') - 4), scan_id=define('ATH12K_SCAN_ID'),
                 notify_scan_events=notify | vals['WMI_SCAN_EVENT_START_FAILED'],  # C2 adds START_FAILED
                 scan_ctrl_flags=define('WMI_SCAN_FLAG_PASSIVE') | define('WMI_SCAN_CHAN_STAT_EVENT'),
                 max_scan_time=38 * int(arg['dwell_time_passive']) * 3 // 2,  # C2: n x dwell x 3/2, not 20000
                 num_chan=38)
    assert int(arg['max_scan_time']) == 20000 and fixed['num_bssid'] == 1 and fixed['dwell_time_passive'] == 150
    assert fixed['scan_priority'] == 0 and fixed['num_ssids'] == fixed['ie_len'] == fixed['n_probes'] == 0
    want = [vals['WMI_START_SCAN_CMDID']] + [fixed[f] for f in order]
    want += [tlv('ARRAY_UINT32', 4 * 38)] + mhz + [tlv('ARRAY_FIXED_STRUCT', 0), tlv('ARRAY_FIXED_STRUCT', 8), 0xffffffff, 0xffff,
                                                   tlv('ARRAY_BYTE', 0)]
    assert words['scan'] == want, [(i, a, b) for i, (a, b) in enumerate(zip(words['scan'], want)) if a != b][:5]
    order = word_names('wmi_stop_scan_cmd')
    stopv = dict.fromkeys(order, 0)
    stopv.update(tlv_header=tlv('STOP_SCAN_CMD', size('wmi_stop_scan_cmd') - 4), scan_id=define('ATH12K_SCAN_ID'),
                 req_type=define('WMI_SCAN_STOP_ONE'))
    assert words['stop'] == [vals['WMI_STOP_SCAN_CMDID']] + [stopv[f] for f in order], words['stop']
    order = word_names('wmi_vdev_set_param_cmd')
    nss = dict(tlv_header=tlv('VDEV_SET_PARAM_CMD', size('wmi_vdev_set_param_cmd') - 4), vdev_id=0,
               param_id=vals['WMI_VDEV_PARAM_NSS'], param_value=2)
    assert words['nss'] == [vals['WMI_VDEV_SET_PARAM_CMDID']] + [nss[f] for f in order], words['nss']
    assert words['htt'] == [vals.get('HTT_H2T_MSG_TYPE_VERSION_REQ', 0),
                            define('HTT_TAG_TCL_METADATA_VERSION', dp_h) | define('HTT_TCL_METADATA_VER_SZ', dp_h) << 8 |
                            define('HTT_OPTION_TCL_METADATA_VER_V2', dp_h) << 16], words['htt']
    print('L19 re-derivation (C2): scan/vdev/STA PS ids, tags, flags, sizes and event field offsets; every word of the '
          'channel list (array header 4 short as L19), VDEV_CREATE, NSS, START_SCAN (init values, priority 0, passive), '
          'STOP_SCAN and the HTT version request; single-pdev pdev 0, preferred hw mode and its PHYs, STA follow-up '
          'order, MGMT_RX drop rules, noise floor')
    l19_c2b(vals, define, struct_fields, size, word_names, tlv, wmi_h, wmi_c, mac_c, words)
    l19_c2c(vals, define, struct_fields, size, word_names, tlv, wmi_h, wmi_c, mac_c, c2cdump)
    l19_c2d(define, c2ddump)


def l19_c2b(vals, define, struct_fields, size, word_names, tlv, wmi_h, wmi_c, mac_c, words):
    """C2b: stats ids, tags, sizes, the three word orders, signedness, request words and the L19 facts it relies on."""
    debugfs_c = l19_text('debugfs.c')
    for ours, theirs in (('WLWMI_CMD_REQUEST_STATS', 'WMI_REQUEST_STATS_CMDID'), ('WLWMI_EV_UPDATE_STATS', 'WMI_UPDATE_STATS_EVENTID'),
                         ('WLWMI_TAG_STATS_EVENT', 'WMI_TAG_STATS_EVENT'), ('WLWMI_TAG_REQUEST_STATS_CMD', 'WMI_TAG_REQUEST_STATS_CMD'),
                         ('WLWMI_REQUEST_PDEV_STAT', 'WMI_REQUEST_PDEV_STAT'), ('WLWMI_TAG_ARRAY_BYTE', 'WMI_TAG_ARRAY_BYTE')):
        assert header_define(ours) == vals[theirs], (ours, header_define(ours), theirs, vals[theirs])
    for ours, theirs in (('WLWMI_REQUEST_STATS_BYTES', 'wmi_request_stats_cmd'), ('WLWMI_STATS_EVENT_BYTES', 'wmi_stats_event'),
                         ('WLWMI_VDEV_STATS_BYTES', 'wmi_vdev_stats_params'), ('WLWMI_BCN_STATS_BYTES', 'ath12k_wmi_bcn_stats_params'),
                         ('WLWMI_PDEV_STATS_BYTES', 'ath12k_wmi_pdev_stats_params'), ('WLWMI_CHAN_INFO_BYTES', 'wmi_chan_info_event')):
        assert header_define(ours) == size(theirs), (ours, header_define(ours), theirs, size(theirs))

    def flat(name):  # word names, nested structs flattened
        out = []
        for sub, field, n, _ in struct_fields(name):
            out += flat(sub) if sub else [field] if n == 1 else [f'{field}[{i}]' for i in range(n)]
        return out
    # The header's word-index enums are L19's struct fields in L19's order, name by name.
    for prefix, struct in (('WLWMI_CI_', 'wmi_chan_info_event'), ('WLWMI_SE_', 'wmi_stats_event'),
                           ('WLWMI_PS_', 'ath12k_wmi_pdev_stats_params')):
        want = [prefix + f.upper() for f in flat(struct)] + [prefix + 'WORDS']
        got = [x.strip() for x in re.search(r'enum \{\s*(' + want[0] + r',.*?)\};', HEADER, re.S)[1].split(',') if x.strip()]
        assert got == want, (struct, [(a, b) for a, b in zip(got, want) if a != b][:3], len(got), len(want))
    assert [sub for sub, _, _, _ in struct_fields('ath12k_wmi_pdev_stats_params')] == \
        ['ath12k_wmi_pdev_base_stats_params', 'ath12k_wmi_pdev_tx_stats_params', 'ath12k_wmi_pdev_rx_stats_params']
    # Signedness: every a_sle32 field printed through (int) with %d, every __le32 one as is with %u or %#x.
    types = {}
    for sub in ('ath12k_wmi_pdev_base_stats_params', 'ath12k_wmi_pdev_tx_stats_params', 'ath12k_wmi_pdev_rx_stats_params'):
        body = re.search(r'struct\s+' + sub + r'\s*\{(.*?)\}\s*__packed;', wmi_h, re.S)[1]
        types.update({f.upper(): t for t, f in re.findall(r'(a_sle32|__le32) (\w+);', body)})
    rep = between(DRIVER, 'static void\nwl23_stats_report(unsigned took, unsigned tx0, unsigned rx0)\n{', '\n}\n')
    printed = 0
    for call in re.findall(r'printf\((.*?)\);', rep, re.S):
        fmt = ''.join(re.findall(r'"((?:[^"\\]|\\.)*)"', call.split('",', 1)[0] + '"'))
        specs = re.findall(r'%#?[a-z]', fmt)
        args = [a.strip() for a in call.rsplit('"', 1)[1].split(',')[1:]]
        assert len(specs) == len(args), (specs, args)
        for spec, arg in zip(specs, args):
            m = re.fullmatch(r'(\(int\))?s\[WLWMI_PS_(\w+)\]', arg)
            if m:
                printed += 1
                assert (m[1] == '(int)') == (types[m[2]] == 'a_sle32') and (spec == '%d') == (types[m[2]] == 'a_sle32'), (spec, arg)
    assert printed == 35, printed
    ci = between(DRIVER, 'static void\nwl23_ci_line(void)\n{', '\n}\n')
    assert '(int)c[WLWMI_CI_NOISE_FLOOR]' in ci  # L19 keeps it in an s8 survey field (wmi.c:7556)
    # The request: ath12k_wmi_send_stats_request_cmd into a zeroed skb, asked as L19's debugfs pdev_stats reader asks.
    send = fn_body(wmi_c, 'int ath12k_wmi_send_stats_request_cmd(')
    for fact in ('cmd->tlv_header = ath12k_wmi_tlv_cmd_hdr(WMI_TAG_REQUEST_STATS_CMD,', 'cmd->stats_id = cpu_to_le32(stats_id);',
                 'cmd->vdev_id = cpu_to_le32(vdev_id);', 'cmd->pdev_id = cpu_to_le32(pdev_id);',
                 'ath12k_wmi_cmd_send(wmi, skb, WMI_REQUEST_STATS_CMDID)'):
        assert fact in send, fact
    assert 'peer_macaddr' not in send and 'memset(skb->data, 0, round_len);' in fn_body(wmi_c, 'struct sk_buff *ath12k_wmi_alloc_skb(')
    assert re.search(r'param\.pdev_id = ath12k_mac_get_target_pdev_id\(ar\);\s+param\.vdev_id = 0;\s+'
                     r'param\.stats_id = WMI_REQUEST_PDEV_STAT;', debugfs_c)
    target = fn_body(mac_c, 'u8 ath12k_mac_get_target_pdev_id(')
    assert re.search(r'if \(!ab->hw_params->single_pdev_only\)\s+return ar->pdev->pdev_id;', target)
    assert re.search(r'if \(!arvif\)\s+return ar->ab->fw_pdev\[0\]\.pdev_id;', target)
    # With a vif up L19 names the fw_pdev whose band matches (one per PHY, its own supported_bands): C2b asks each.
    assert re.search(r'for \(i = 0; i < ab->fw_pdev_count; i\+\+\) \{\s+if \(ath12k_mac_band_match\(band, '
                     r'ab->fw_pdev\[i\]\.supported_bands\)\)\s+return ab->fw_pdev\[i\]\.pdev_id;', mac_c)
    pull = fn_body(wmi_c, 'ath12k_pull_mac_phy_cap_svc_ready_ext(')
    assert 'fw_pdev = &ab->fw_pdev[ab->fw_pdev_count];' in pull and \
        'fw_pdev->pdev_id = ath12k_wmi_mac_phy_get_pdev_id(mac_caps);' in pull and \
        'fw_pdev->supported_bands = le32_to_cpu(mac_caps->supported_bands);' in pull
    reg = fn_body(wmi_c, 'static int ath12k_wmi_ext_soc_hal_reg_caps_parse(')
    assert reg.index('soc->num_radios = 0;') < reg.index('soc->fw_pdev_count = 0;') < reg.index('while (phy_id_map')
    assert define('WMI_CAPS_PARAMS_PDEV_ID') == (15, 0)
    # Waits: L19 1 s for the first event, 3 s for done (C2b 500 ms); WMI send credits 3 s (WMI_SEND_TIMEOUT_HZ).
    fw = fn_body(mac_c, 'int ath12k_mac_get_fw_stats(')
    assert 'wait_for_completion_timeout(&ar->fw_stats_complete, 1 * HZ)' in fw and \
        'wait_for_completion_timeout(&ar->fw_stats_done, 3 * HZ)' in fw
    assert re.search(r'#define WMI_SEND_TIMEOUT_HZ \(3 \* HZ\)', wmi_h)
    # The parse: ev first (else -EPROTO), then vdev, beacon and pdev entries each bounded by what is left; no length
    # policy for the fixed TLV (C2b requires the whole struct); single-pdev chips accept any event pdev id; the dump
    # prints the first pdev entry; a PDEV_STAT answer completes fw_stats_done, empty vdev stats never do.
    data = fn_body(wmi_c, 'static int ath12k_wmi_tlv_fw_stats_data_parse(')
    assert 'if (!ev) {' in data and data.count('if (len < sizeof(*src)) {') == 3
    assert data.index('ev->num_vdev_stats') < data.index('ev->num_bcn_stats') < data.index('ev->num_pdev_stats')
    parse = fn_body(wmi_c, 'static int ath12k_wmi_tlv_fw_stats_parse(')
    assert re.search(r'case WMI_TAG_STATS_EVENT:\s+parse->ev = ptr;', parse)
    assert re.search(r'case WMI_TAG_ARRAY_BYTE:\s+ret = ath12k_wmi_tlv_fw_stats_data_parse\(ab, parse, ptr, len\);', parse)
    policies = wmi_c.split('ath12k_wmi_tlv_policies[] = {', 1)[1].split('};', 1)[0]
    assert '[WMI_TAG_STATS_EVENT]' not in policies and '[WMI_TAG_ARRAY_BYTE] = { .min_len = 0 }' in policies
    assert re.search(r'if \(ab->hw_params->single_pdev_only\) \{\s+pdev = rcu_dereference\(ab->pdevs_active\[0\]\);',
                     fn_body(mac_c, 'struct ath12k *ath12k_mac_get_ar_by_pdev_id('))
    assert 'list_first_entry_or_null(&fw_stats->pdevs,' in fn_body(wmi_c, 'ath12k_wmi_fw_pdev_stats_dump(')
    upd = fn_body(wmi_c, 'static void ath12k_update_stats_event(')
    assert re.search(r'if \(stats\.stats_id == WMI_REQUEST_PDEV_STAT\) \{\s+list_splice_tail_init\(&stats\.pdevs, '
                     r'&ar->fw_stats\.pdevs\);\s+complete\(&ar->fw_stats_done\);', upd)
    proc = fn_body(wmi_c, 'static void ath12k_wmi_fw_stats_process(')
    assert 'total_vdevs_started += pdev->ar->num_started_vdevs;' in proc and \
        re.search(r'if \(list_empty\(&stats->vdevs\)\) \{\s+ath12k_warn\(ab, "empty vdev stats"\);\s+return;', proc)
    # CHAN_INFO as L19 uses it: END only noted, START's counters taken whole (no delta, nothing kept), MAC clock in MHz.
    ch = fn_body(wmi_c, 'static void ath12k_chan_info_event(')
    assert re.search(r'if \(le32_to_cpu\(ch_info_ev\.cmd_flags\) == WMI_CHAN_INFO_END_RESP\) \{.*?return;', ch, re.S)
    for fact in ('cc_freq_hz = (le32_to_cpu(ch_info_ev.mac_clk_mhz) * 1000);', 'memset(survey, 0, sizeof(*survey));',
                 'survey->time = div_u64(le32_to_cpu(ch_info_ev.cycle_count), cc_freq_hz);'):
        assert fact in ch, fact
    assert 'last' not in ch and 'prev' not in ch
    # The request words against L19's struct order and values (the model dumps pdev 1; the peer MAC words stay 0).
    order = word_names('wmi_request_stats_cmd')
    assert order == ['tlv_header', 'stats_id', 'vdev_id', 'peer_macaddr', 'peer_macaddr', 'pdev_id'], order
    req = dict.fromkeys(order, 0)
    req.update({'tlv_header': tlv('REQUEST_STATS_CMD', size('wmi_request_stats_cmd') - 4),
                'stats_id': vals['WMI_REQUEST_PDEV_STAT'], 'pdev_id': 1})
    want = [vals['WMI_REQUEST_STATS_CMDID']] + [req[f] for f in order]
    assert words['stats'] == want, (words['stats'], want)
    print('L19 re-derivation (C2b): REQUEST_STATS/UPDATE_STATS ids, tags, PDEV_STAT, five struct sizes, the CHAN_INFO, '
          'stats event and pdev stats word orders name by name, signedness of all 35 printed pdev fields, the request '
          'words, target pdev = fw_pdev[0] (then the band-matched fw_pdev), L19 waits, vdev/beacon/pdev parse order, no '
          'fixed-TLV policy, CHAN_INFO '
          'without deltas')


def hdefs():
    """Every #define of the header whose value is an integer expression of integers and other such defines."""
    raw = dict(re.findall(r'^#define (\w+) (.+?)(?:\s*/\*.*)?$', HEADER, re.M))
    out = {}

    def val(name, depth=0):
        if name in out:
            return out[name]
        assert depth < 20, name
        expr = re.sub(r'\b(0x[0-9a-fA-F]+|\d+)u\b', r'\1', raw[name])
        for ref in set(re.findall(r'\b([A-Z][A-Z0-9_]+)\b', expr)):
            expr = re.sub(r'\b' + ref + r'\b', str(val(ref, depth + 1)), expr)
        out[name] = eval(expr, {}, {})
        return out[name]
    for name in raw:
        try:
            val(name)
        except Exception:
            pass
    return out


def l19_c2c(vals, define, struct_fields, size, word_names, tlv, wmi_h, wmi_c, mac_c, dump):
    """C2c: ring ids and rdp/wrp from hal.h/hal.c, HTT enums, fields and the encoder rules (dp.h, dp_tx.c), thresholds
    (hal.h, dp.c), sizes from compiled replicas of L19's structs, RBMs (hal.h, hw.c, dp.c), the WCN7850 WBM registers
    (hw.c, hal.h) and L19's ring 120 write order and values (hal.c), the PDEV_SET_PARAM ids and order (wmi.h, wmi.c,
    mac.c): against the header, then every word of the model's dump (four SRING_SETUP, two PDEV_SET_PARAM, first and last
    refill and idle entries, the ring 120 register list, the pre-read addresses, shadow index and HP)."""
    hal_h, hal_c, hw_c, dp_h, dp_c, dp_tx_c, dp_rx_c, desc, hal_rx_c = (l19_text(n) for n in (
        'hal.h', 'hal.c', 'hw.c', 'dp.h', 'dp.c', 'dp_tx.c', 'dp_rx.c', 'hal_desc.h', 'hal_rx.c'))
    H = hdefs()
    flat = lambda t: ' '.join(t.replace('\\\n', ' ').split())  # noqa: E731

    def enum_values(text, name):
        body = re.search(r'enum\s+' + name + r'\s*\{(.*?)\};', text, re.S)[1]
        out, cur = {}, -1
        for item in body.split(','):
            item = item.strip()
            if not item:
                continue
            if '=' in item:
                k, v = (x.strip() for x in item.split('=', 1))
                cur = out[v] if v in out else int(v, 0)
            else:
                k, cur = item, cur + 1
            out[k] = cur
        return out
    # Ring ids (hal.h enum, hal.c configs and ring-id arithmetic, dp_rx.c/dp.c setups) and the rdp/wrp sizes.
    ring = enum_values(hal_h, 'hal_srng_ring_id')
    pp = ring['HAL_SRNG_RING_ID_PMAC1_ID_END'] - ring['HAL_SRNG_RING_ID_PMAC1_ID_START']
    d0, dend, npmac = ring['HAL_SRNG_RING_ID_DMAC_CMN_ID_START'], ring['HAL_SRNG_RING_ID_DMAC_CMN_ID_END'], define('HAL_SRNG_NUM_PMACS', hal_h)
    fh = flat(hal_h)
    assert '#define HAL_SRNG_RINGS_PER_PMAC (HAL_SRNG_RING_ID_PMAC1_ID_END - HAL_SRNG_RING_ID_PMAC1_ID_START)' in fh
    assert '#define HAL_SRNG_RING_ID_MAX (HAL_SRNG_RING_ID_DMAC_CMN_ID_END + HAL_SRNG_NUM_PMAC_RINGS)' in fh
    assert '#define HAL_SRNG_NUM_PMAC_RINGS (HAL_SRNG_NUM_PMACS * HAL_SRNG_RINGS_PER_PMAC)' in fh
    assert '#define HAL_SRNG_NUM_DMAC_RINGS (HAL_SRNG_RING_ID_DMAC_CMN_ID_END - HAL_SRNG_RING_ID_DMAC_CMN_ID_START)' in fh
    fc = flat(hal_c)
    assert 'size = sizeof(u32) * HAL_SRNG_RING_ID_MAX;' in fc and 'size = sizeof(u32) * (HAL_SRNG_NUM_PMAC_RINGS + HAL_SRNG_NUM_DMAC_RINGS);' in fc
    rdp_words, wrp_words = dend + npmac * pp, npmac * pp + (dend - d0)
    cfgs = {}
    for t, body in re.findall(r'\[(HAL_\w+)\] = \{(.*?)\},', fc):
        cfgs[t] = dict(re.findall(r'\.(\w+) = ([^,]+)', body))
    assert cfgs['HAL_RXDMA_BUF']['start_ring_id'] == 'HAL_SRNG_SW2RXDMA_BUF0' and cfgs['HAL_RXDMA_BUF']['ring_dir'] == 'HAL_SRNG_DIR_SRC'
    assert cfgs['HAL_RXDMA_DST']['start_ring_id'] == 'HAL_SRNG_RING_ID_WMAC1_RXDMA2SW0' and cfgs['HAL_RXDMA_DST']['ring_dir'] == 'HAL_SRNG_DIR_DST'
    assert cfgs['HAL_WBM_IDLE_LINK']['start_ring_id'] == 'HAL_SRNG_RING_ID_WBM_IDLE_LINK' and \
        cfgs['HAL_WBM_IDLE_LINK']['ring_dir'] == 'HAL_SRNG_DIR_SRC' and cfgs['HAL_WBM_IDLE_LINK']['mac_type'] == 'ATH12K_HAL_SRNG_UMAC'
    wcfg = flat(fn_body(hal_c, 'static int ath12k_hal_srng_create_config_wcn7850('))
    assert 's = &hal->srng_config[HAL_RXDMA_BUF]; s->max_rings = 2; s->mac_type = ATH12K_HAL_SRNG_PMAC;' in wcfg
    assert 's = &hal->srng_config[HAL_RXDMA_DST]; s->max_rings = 1; s->entry_size = sizeof(struct hal_reo_entrance_ring) >> 2;' in wcfg
    assert 's = &hal->srng_config[HAL_WBM_IDLE_LINK]; s->reg_start[0] = HAL_SEQ_WCSS_UMAC_WBM_REG + HAL_WBM_IDLE_LINK_RING_BASE_LSB(ab); ' \
        's->reg_start[1] = HAL_SEQ_WCSS_UMAC_WBM_REG + HAL_WBM_IDLE_LINK_RING_HP;' in wcfg
    gid = flat(fn_body(hal_c, 'static int ath12k_hal_srng_get_ring_id('))
    assert 'ring_id = srng_config->start_ring_id + ring_num; if (srng_config->mac_type == ATH12K_HAL_SRNG_PMAC) ring_id += ' \
        'mac_id * HAL_SRNG_RINGS_PER_PMAC;' in gid
    alloc = flat(fn_body(dp_rx_c, 'int ath12k_dp_rx_alloc('))
    assert 'ath12k_dp_srng_setup(ab, &dp->rx_refill_buf_ring.refill_buf_ring, HAL_RXDMA_BUF, 0, 0, DP_RXDMA_BUF_RING_SIZE);' in alloc
    assert 'ath12k_dp_srng_setup(ab, &dp->rx_mac_buf_ring[i], HAL_RXDMA_BUF, 1, i, DP_RX_MAC_BUF_RING_SIZE);' in alloc
    assert 'ath12k_dp_srng_setup(ab, &dp->rxdma_err_dst_ring[i], HAL_RXDMA_DST, 0, i, DP_RXDMA_ERR_DST_RING_SIZE);' in alloc
    buf0, dst0 = ring['HAL_SRNG_SW2RXDMA_BUF0'], ring['HAL_SRNG_RING_ID_WMAC1_RXDMA2SW0']
    ids = [buf0 + 0 + 0 * pp, buf0 + 1 + 0 * pp, buf0 + 1 + 1 * pp, dst0 + 0 + 0 * pp]
    assert [ring['HAL_SRNG_RING_ID_WBM_IDLE_LINK']] + ids == [H['WLRX_RING_IDLE'], H['WLRX_RING_REFILL'], H['WLRX_RING_MAC0'],
                                                              H['WLRX_RING_MAC1'], H['WLRX_RING_ERR']], ids
    assert (pp, d0, rdp_words, wrp_words) == (H['WLRX_RINGS_PER_PMAC'], H['WLRX_DMAC_START'], H['WLRX_RDP_WORDS'], H['WLRX_WRP_WORDS'])
    wcn = hw_c.split('.name = "wcn7850 hw2.0"', 1)[1].split('.name =', 1)[0]
    for fact in ('.rx_mac_buf_ring = true', '.num_rxdma_per_pdev = 2', '.num_rxdma_dst_ring = 1', '.rxdma1_enable = false',
                 '.hal_params = &ath12k_hw_hal_params_wcn7850'):
        assert fact in wcn, fact
    # L19's announcement order (dp_rx.c ath12k_dp_rx_htt_setup): refill, the per-MAC rings, the error ring; then monitor
    # status rings and ring selection, which C2c leaves out.
    hs = flat(fn_body(dp_rx_c, 'int ath12k_dp_rx_htt_setup('))
    order = [hs.index(s) for s in ('ath12k_dp_tx_htt_srng_setup(ab, ring_id, 0, HAL_RXDMA_BUF);',
                                   'ath12k_dp_tx_htt_srng_setup(ab, ring_id, i, HAL_RXDMA_BUF);',
                                   'ath12k_dp_tx_htt_srng_setup(ab, ring_id, i, HAL_RXDMA_DST);', 'rxdma_ring_sel_config(ab)')]
    assert order == sorted(order), order
    # Sizes: dp.h ring sizes, buffer size and alignment, link descriptor size; struct sizes from compiled replicas.
    for ours, theirs in (('WLRX_REFILL_ENTRIES', 'DP_RXDMA_BUF_RING_SIZE'), ('WLRX_MAC_ENTRIES', 'DP_RX_MAC_BUF_RING_SIZE'),
                         ('WLRX_ERR_ENTRIES', 'DP_RXDMA_ERR_DST_RING_SIZE'), ('WLRX_BUF_BYTES', 'DP_RX_BUFFER_SIZE'),
                         ('WLRX_COOKIE_START', 'DP_LINK_DESC_START'), ('WLRX_COOKIE_SHIFT', 'DP_LINK_DESC_SHIFT')):
        assert H[ours] == define(theirs, dp_h), (ours, theirs)
    assert define('DP_RX_BUFFER_ALIGN_SIZE', dp_h) == 128 and H['WLRX_SLOT_BYTES'] % 128 == 0
    assert re.search(r'#define DP_LINK_DESC_COOKIE_SET\(id, page\) \\\s+\(\(\(\(id\) \+ DP_LINK_DESC_START\) << DP_LINK_DESC_SHIFT\) \| \(page\)\)', dp_h)
    assert re.search(r'#define HAL_LINK_DESC_SIZE\s+\(32 << 2\)', hal_h) and define('HAL_LINK_DESC_ALIGN', hal_h) == 128 == H['WLRX_LINK_DESC_BYTES']
    raw = {n: re.sub(r'/\*.*?\*/', '', (L19 / n).read_text(), flags=re.S) for n in ('hal_desc.h', 'dp.h', 'wmi.h')}
    structs = [('hal_desc.h', 'ath12k_buffer_addr'), ('hal_desc.h', 'rx_mpdu_desc'), ('hal_desc.h', 'hal_reo_entrance_ring'),
               ('hal_desc.h', 'hal_wbm_link_desc'), ('hal_desc.h', 'hal_wbm_buffer_ring'), ('dp.h', 'htt_srng_setup_cmd'),
               ('wmi.h', 'wmi_pdev_set_param_cmd')]
    src = '#include <stdio.h>\n#include <stdint.h>\ntypedef uint32_t __le32;\ntypedef uint32_t u32;\n#define __packed __attribute__((packed))\n'
    for f, s in structs:
        src += re.search(r'struct ' + s + r' \{.*?\}(?: __packed)?;', raw[f], re.S)[0] + '\n'
    src += 'int main(void) { printf("' + ' '.join(['%zu'] * len(structs)) + '\\n", ' + \
        ', '.join(f'sizeof(struct {s})' for _, s in structs) + '); return 0; }\n'
    with tempfile.TemporaryDirectory(prefix='fuxi-c2c-sizes-') as tmp:
        (Path(tmp) / 's.c').write_text(src)
        subprocess.run([CC[0], '-std=gnu11', '-o', str(Path(tmp) / 's'), str(Path(tmp) / 's.c')], check=True)
        got = dict(zip((s for _, s in structs), map(int, subprocess.run([str(Path(tmp) / 's')], check=True, capture_output=True,
                                                                         text=True).stdout.split())))
    assert got['hal_wbm_buffer_ring'] // 4 == H['WLRX_BUF_DW'] and got['hal_reo_entrance_ring'] // 4 == H['WLRX_ERR_DW'] and \
        got['hal_wbm_link_desc'] // 4 == H['WLRX_IDLE_DW'] and got['htt_srng_setup_cmd'] == 4 * H['WLRX_SRING_WORDS'] and \
        got['wmi_pdev_set_param_cmd'] == H['WLWMI_PDEV_SET_PARAM_BYTES'] and got['rx_mpdu_desc'] == 8, got

    # The error entry's frame fields the report masks out (counts only): info0 (w5) MPDU_BYTE_COUNT, info1 (w6)
    # MPDU_FRAG_NUM and MPDU_SEQ_NUM; the struct's word order puts info0/info1 at w5/w6.
    def genmask(name):
        hi, lo = map(int, re.search(r'#define ' + name + r'\s+GENMASK\((\d+), (\d+)\)', desc).groups())
        return ((1 << (hi + 1)) - 1) & ~((1 << lo) - 1)
    assert genmask('HAL_REO_ENTR_RING_INFO0_MPDU_BYTE_COUNT') == H['WLRX_ERR_W5_FRAME']
    assert genmask('HAL_REO_ENTR_RING_INFO1_MPDU_FRAG_NUM') | genmask('HAL_REO_ENTR_RING_INFO1_MPDU_SEQ_NUM') == H['WLRX_ERR_W6_FRAME']
    ent = re.search(r'struct hal_reo_entrance_ring \{(.*?)\}', raw['hal_desc.h'], re.S)[1]
    assert re.findall(r'(\w+);', ent) == ['buf_addr_info', 'rx_mpdu_info', 'queue_addr_lo', 'info0', 'info1', 'info2'], ent
    # The idle list: n_link_desc rounded to a power of two (dp.c), on the ring path (below the scatter threshold).
    ws = fn_body(dp_c, 'static int ath12k_wbm_idle_ring_setup(')

    def dpdef(name):  # an integer #define of dp.h, other defines resolved
        expr = re.search(r'#define ' + name + r'\s+(.+?)\s*$', dp_h, re.M)[1]
        for ref in set(re.findall(r'\b([A-Z][A-Z0-9_]+)\b', expr)):
            expr = re.sub(r'\b' + ref + r'\b', str(dpdef(ref)), expr)
        return eval(expr, {}, {})
    dpn = {k: dpdef(k) for k in ('DP_NUM_TIDS_MAX', 'DP_AVG_MPDUS_PER_TID_MAX', 'DP_AVG_FLOWS_PER_TID', 'DP_AVG_MSDUS_PER_FLOW',
                                 'DP_AVG_MSDUS_PER_MPDU')}
    haln = {k: define(k, hal_h) for k in ('HAL_NUM_MPDUS_PER_LINK_DESC', 'HAL_NUM_MPDU_LINKS_PER_QUEUE_DESC',
                                          'HAL_NUM_TX_MSDUS_PER_LINK_DESC', 'HAL_NUM_RX_MSDUS_PER_LINK_DESC')}
    assert 'if (*n_link_desc & (*n_link_desc - 1))\n\t\t*n_link_desc = 1 << fls(*n_link_desc);' in ws
    mpdu = dpn['DP_NUM_TIDS_MAX'] * dpn['DP_AVG_MPDUS_PER_TID_MAX'] // haln['HAL_NUM_MPDUS_PER_LINK_DESC']
    n = mpdu + mpdu // haln['HAL_NUM_MPDU_LINKS_PER_QUEUE_DESC'] + \
        dpn['DP_NUM_TIDS_MAX'] * dpn['DP_AVG_FLOWS_PER_TID'] * dpn['DP_AVG_MSDUS_PER_FLOW'] // haln['HAL_NUM_TX_MSDUS_PER_LINK_DESC'] + \
        dpn['DP_NUM_TIDS_MAX'] * dpn['DP_AVG_MPDUS_PER_TID_MAX'] * dpn['DP_AVG_MSDUS_PER_MPDU'] // haln['HAL_NUM_RX_MSDUS_PER_LINK_DESC']
    n = n if not n & (n - 1) else 1 << n.bit_length()
    assert n == H['WLRX_IDLE_ENTRIES'] and n * got['hal_wbm_link_desc'] <= define('DP_LINK_DESC_ALLOC_SIZE_THRESH', dp_h), n
    assert 'ath12k_dp_srng_setup(ab, &dp->wbm_idle_ring, HAL_WBM_IDLE_LINK, 0, 0, *n_link_desc);' in flat(ws)
    # Fill counts: a source ring holds entries - 1 (hal.c num_free; the get_next_entry full rule).
    assert H['WLRX_NBUF'] == H['WLRX_REFILL_ENTRIES'] - 1 and H['WLRX_NDESC'] == n - 1
    assert 'return ((srng->ring_size - hp + tp) / srng->entry_size) - 1;' in hal_c
    # HTT: enums, the field masks and the encoder rules.
    h2t, rtype, rid = enum_values(dp_h, 'htt_h2t_msg_type'), enum_values(dp_h, 'htt_srng_ring_type'), enum_values(dp_h, 'htt_srng_ring_id')
    for ours, v in (('WLRX_HTT_SRING_SETUP', h2t['HTT_H2T_MSG_TYPE_SRING_SETUP']), ('WLRX_HTT_HW_TO_SW', rtype['HTT_HW_TO_SW_RING']),
                    ('WLRX_HTT_SW_TO_HW', rtype['HTT_SW_TO_HW_RING']), ('WLRX_HTT_SW_TO_SW', rtype['HTT_SW_TO_SW_RING']),
                    ('WLRX_HTT_RING_RXDMA_HOST_BUF', rid['HTT_RXDMA_HOST_BUF_RING']),
                    ('WLRX_HTT_RING_HOST1_TO_FW_RXBUF', rid['HTT_HOST1_TO_FW_RXBUF_RING']),
                    ('WLRX_HTT_RING_NON_MONITOR_DEST', rid['HTT_RXDMA_NON_MONITOR_DEST_RING'])):
        assert H[ours] == v, ours
    assert H['WLRX_HTT_SRING_SETUP'] not in (h2t['HTT_H2T_MSG_TYPE_RX_RING_SELECTION_CFG'], h2t['HTT_H2T_MSG_TYPE_PPDU_STATS_CFG'],
                                             h2t['HTT_H2T_MSG_TYPE_EXT_STATS_CFG'])
    fld = {k: define('HTT_SRNG_SETUP_CMD_' + k, dp_h) for k in ('INFO0_MSG_TYPE', 'INFO0_PDEV_ID', 'INFO0_RING_ID', 'INFO0_RING_TYPE',
                                                             'INFO1_RING_SIZE', 'INFO1_RING_ENTRY_SIZE', 'INFO1_RING_LOOP_CNT_DIS',
                                                             'INTR_INFO_BATCH_COUNTER_THRESH', 'INTR_INFO_INTR_TIMER_THRESH',
                                                             'INFO2_INTR_LOW_THRESH', 'INFO2_RESPONSE_REQUIRED')}
    assert (fld['INFO0_MSG_TYPE'], fld['INFO0_PDEV_ID'], fld['INFO0_RING_ID'], fld['INFO0_RING_TYPE']) == ((7, 0), (15, 8), (23, 16), (31, 24))
    assert (fld['INFO1_RING_SIZE'], fld['INFO1_RING_ENTRY_SIZE'], fld['INTR_INFO_BATCH_COUNTER_THRESH'], fld['INTR_INFO_INTR_TIMER_THRESH'],
            fld['INFO2_INTR_LOW_THRESH']) == ((15, 0), (23, 16), (14, 0), (31, 16), (15, 0))
    assert fld['INFO1_RING_LOOP_CNT_DIS'] == H['WLRX_HTT_LOOP_CNT_DIS'] and fld['INFO2_RESPONSE_REQUIRED'] == H['WLRX_HTT_RESPONSE_REQUIRED']
    order = word_names_dp = [f for _, f, _, _ in struct_fields('htt_srng_setup_cmd', dp_h)]
    assert order == ['info0', 'ring_base_addr_lo', 'ring_base_addr_hi', 'info1', 'ring_head_off32_remote_addr_lo',
                     'ring_head_off32_remote_addr_hi', 'ring_tail_off32_remote_addr_lo', 'ring_tail_off32_remote_addr_hi',
                     'ring_msi_addr_lo', 'ring_msi_addr_hi', 'msi_data', 'intr_info', 'info2'], word_names_dp
    typ = flat(fn_body(dp_tx_c, 'ath12k_dp_tx_get_ring_id_type('))
    assert 'if (ring_id == HAL_SRNG_SW2RXDMA_BUF0) { *htt_ring_id = HTT_HOST1_TO_FW_RXBUF_RING; *htt_ring_type = HTT_SW_TO_SW_RING; } ' \
        'else { *htt_ring_id = HTT_RXDMA_HOST_BUF_RING; *htt_ring_type = HTT_SW_TO_HW_RING; }' in typ
    assert 'case HAL_RXDMA_DST: *htt_ring_id = HTT_RXDMA_NON_MONITOR_DEST_RING; *htt_ring_type = HTT_HW_TO_SW_RING;' in typ
    enc = flat(fn_body(dp_tx_c, 'int ath12k_dp_tx_htt_srng_setup('))
    for fact in ('if (htt_ring_type == HTT_SW_TO_HW_RING || htt_ring_type == HTT_HW_TO_SW_RING) cmd->info0 |= '
                 'le32_encode_bits(DP_SW2HW_MACID(mac_id), HTT_SRNG_SETUP_CMD_INFO0_PDEV_ID); else cmd->info0 |= '
                 'le32_encode_bits(mac_id, HTT_SRNG_SETUP_CMD_INFO0_PDEV_ID);',
                 'ring_entry_sz >>= 2;', 'le32_encode_bits(params.num_entries * ring_entry_sz, HTT_SRNG_SETUP_CMD_INFO1_RING_SIZE);',
                 'if (htt_ring_type == HTT_SW_TO_HW_RING) cmd->info1 |= cpu_to_le32(HTT_SRNG_SETUP_CMD_INFO1_RING_LOOP_CNT_DIS);',
                 'le32_encode_bits(params.intr_batch_cntr_thres_entries * ring_entry_sz, HTT_SRNG_SETUP_CMD_INTR_INFO_BATCH_COUNTER_THRESH);',
                 'le32_encode_bits(params.intr_timer_thres_us >> 3, HTT_SRNG_SETUP_CMD_INTR_INFO_INTR_TIMER_THRESH);',
                 'cmd->info2 = 0; if (params.flags & HAL_SRNG_FLAGS_LOW_THRESH_INTR_EN) { cmd->info2 = le32_encode_bits(params.low_threshold, '
                 'HTT_SRNG_SETUP_CMD_INFO2_INTR_LOW_THRESH); }', 'ath12k_htc_send(&ab->htc, ab->dp.eid, skb);'):
        assert fact in enc, fact
    assert 'RESPONSE_REQUIRED' not in enc
    assert re.search(r'#define DP_SW2HW_MACID\(mac_id\) \(\(mac_id\) \+ 1\)', dp_h)
    # Thresholds (dp.c ath12k_dp_srng_setup, hal.h) and the source ring's low threshold in words (hal.c).
    th = flat(fn_body(dp_c, 'int ath12k_dp_srng_setup('))
    assert 'case HAL_RXDMA_BUF: case HAL_RXDMA_MONITOR_BUF: params.low_threshold = num_entries >> 3; params.flags |= ' \
        'HAL_SRNG_FLAGS_LOW_THRESH_INTR_EN; params.intr_batch_cntr_thres_entries = 0; params.intr_timer_thres_us = ' \
        'HAL_SRNG_INT_TIMER_THRESHOLD_RX; break;' in th
    other = th.split('case HAL_WBM_IDLE_LINK:', 1)[1].split('break;', 1)[0]
    assert 'case HAL_RXDMA_DST:' in other and 'params.intr_batch_cntr_thres_entries = HAL_SRNG_INT_BATCH_THRESHOLD_OTHER; ' \
        'params.intr_timer_thres_us = HAL_SRNG_INT_TIMER_THRESHOLD_OTHER;' in other
    assert (define('HAL_SRNG_INT_TIMER_THRESHOLD_RX', hal_h), define('HAL_SRNG_INT_TIMER_THRESHOLD_OTHER', hal_h),
            define('HAL_SRNG_INT_BATCH_THRESHOLD_OTHER', hal_h)) == (H['WLRX_TIMER_RX_US'], H['WLRX_TIMER_OTHER_US'], H['WLRX_BATCH_OTHER'])
    assert 'srng->u.src_ring.low_threshold = params->low_threshold * srng->entry_size;' in fc
    # Remote pointer words: SRC tp in rdp[id], LMAC hp in wrp[id - 160]; DST hp in rdp[id], LMAC tp in wrp[id - 160].
    for fact in ('srng->u.src_ring.tp_addr = (void *)(hal->rdp.vaddr + ring_id);', 'srng->u.dst_ring.hp_addr = (void *)(hal->rdp.vaddr + ring_id);',
                 'idx = ring_id - HAL_SRNG_RING_ID_DMAC_CMN_ID_START; srng->u.src_ring.hp_addr = (void *)(hal->wrp.vaddr + idx);',
                 'idx = ring_id - HAL_SRNG_RING_ID_DMAC_CMN_ID_START; srng->u.dst_ring.tp_addr = (void *)(hal->wrp.vaddr + idx);'):
        assert fact in fc, fact
    # RBMs and the entry fields; our cookies in L19's shapes.
    rbm = enum_values(hal_h, 'hal_rx_buf_return_buf_manager')
    assert rbm['HAL_RX_BUF_RBM_SW1_BM'] == H['WLRX_RBM_SW1'] and rbm['HAL_RX_BUF_RBM_WBM_DEV0_IDLE_DESC_LIST'] == H['WLRX_RBM_IDLE_DESC']
    assert re.search(r'ath12k_hw_hal_params_wcn7850 = \{\s+\.rx_buf_rbm = HAL_RX_BUF_RBM_SW1_BM,', hw_c)
    assert re.search(r'case 0:\s+return HAL_RX_BUF_RBM_WBM_DEV0_IDLE_DESC_LIST;', fn_body(dp_c, 'ath12k_dp_get_idle_link_rbm('))
    addr = [define('BUFFER_ADDR_INFO' + k, desc) for k in ('0_ADDR', '1_ADDR', '1_RET_BUF_MGR', '1_SW_COOKIE')]
    assert addr == [(31, 0), (7, 0), (11, 8), (31, 12)], addr
    sl = flat(fn_body(hal_c, 'void ath12k_hal_set_link_desc_addr('))
    assert 'le32_encode_bits(rbm, BUFFER_ADDR_INFO1_RET_BUF_MGR) | le32_encode_bits(cookie, BUFFER_ADDR_INFO1_SW_COOKIE);' in sl
    assert 'BUFFER_ADDR_INFO1_RET_BUF_MGR' in hal_rx_c and 'BUFFER_ADDR_INFO1_SW_COOKIE' in hal_rx_c
    # The WCN7850 WBM registers (hw.c), the UMAC block, R2 HP/TP (hal.h), the MISC bits, the scatter mode bit.
    regs = {k: int(v, 16) for k, v in re.findall(r'\.(\w+) = (0x[0-9a-fA-F]+),', hw_c.split('wcn7850_regs = {', 1)[1].split('};', 1)[0])}
    wbm, hp = define('HAL_SEQ_WCSS_UMAC_WBM_REG', hal_h), define('HAL_WBM_IDLE_LINK_RING_HP', hal_h)
    tpoff = define('HAL_TCL1_RING_TP', hal_h) - define('HAL_TCL1_RING_HP', hal_h)
    r0 = wbm + regs['hal_wbm_idle_ring_base_lsb']
    tcl = regs['hal_tcl1_ring_base_lsb']
    off = {k: regs['hal_tcl1_ring_' + k] - tcl for k in ('base_msb', 'id', 'misc', 'consumer_int_setup_ix0', 'consumer_int_setup_ix1')}
    assert regs['hal_wbm_idle_ring_misc_addr'] - regs['hal_wbm_idle_ring_base_lsb'] == off['misc']
    pre = [r0, r0 + off['base_msb'], wbm + regs['hal_wbm_idle_ring_misc_addr'], wbm + hp, wbm + hp + tpoff,
           wbm + regs['hal_wbm_r0_idle_list_cntl_addr'], wbm + regs['hal_wbm_r0_idle_list_size_addr'],
           wbm + regs['hal_wbm_scattered_ring_base_lsb'], wbm + regs['hal_wbm_scattered_ring_base_msb']]
    assert (r0, wbm + hp, H['WLRX_WBM_LIST_CNTL'], H['WLRX_WBM_SCAT_LSB'], H['WLRX_WBM_SCAT_MSB']) == \
        (H['WLRX_WBM_IDLE_R0'], H['WLRX_WBM_IDLE_HP'], pre[5], pre[7], pre[8])
    bit = lambda n: define(n, hal_h)  # noqa: E731
    misc = bit('HAL_TCL1_RING_MISC_MSI_RING_ID_DISABLE') | bit('HAL_TCL1_RING_MISC_MSI_LOOPCNT_DISABLE') | bit('HAL_TCL1_RING_MISC_SRNG_ENABLE')
    assert misc == H['WLRX_WBM_MISC'] and bit('HAL_WBM_LINK_DESC_IDLE_LIST_MODE') == H['WLRX_WBM_LIST_MODE']
    assert (bit('HAL_TCL1_RING_BASE_MSB_RING_SIZE'), bit('HAL_TCL1_RING_CONSR_INT_SETUP_IX0_INTR_TMR_THOLD'),
            bit('HAL_TCL1_RING_CONSR_INT_SETUP_IX0_BATCH_COUNTER_THOLD')) == ((27, 8), (31, 16), (14, 0))
    # L19's write order for ring 120 (ath12k_hal_srng_src_hw_init without the MSI block, which needs an MSI group, and
    # without TP_ADDR, which it skips for this ring) and the values for an idle ring at 0xa1400000.
    hw = fn_body(hal_c, 'static void ath12k_hal_srng_src_hw_init(')
    assert 'if (srng->flags & HAL_SRNG_FLAGS_MSI_INTR) {' in hw and 'if (srng->ring_id != HAL_SRNG_RING_ID_WBM_IDLE_LINK) {' in hw
    hw = re.sub(r'if \(srng->flags & HAL_SRNG_FLAGS_MSI_INTR\) \{.*?\n\t\}\n', '', hw, flags=re.S)
    hw = re.sub(r'if \(srng->ring_id != HAL_SRNG_RING_ID_WBM_IDLE_LINK\) \{.*?\n\t\}\n', '', hw, flags=re.S)
    seq, grp = [], 'R0'
    for m in re.finditer(r'reg_base = srng->hwreg_base\[HAL_SRNG_REG_GRP_(R\d)\];|ath12k_hif_write32\(ab,\s*(reg_base(?:\s*\+\s*[\w()]+)?)\s*,', hw):
        if m[1]:
            grp = m[1]
            continue
        seq.append((grp, ' '.join(m[2].split())))
    names = {'reg_base': None, 'reg_base + HAL_TCL1_RING_BASE_MSB_OFFSET(ab)': 'base_msb', 'reg_base + HAL_TCL1_RING_ID_OFFSET(ab)': 'id',
             'reg_base + HAL_TCL1_RING_CONSR_INT_SETUP_IX0_OFFSET(ab)': 'consumer_int_setup_ix0',
             'reg_base + HAL_TCL1_RING_CONSR_INT_SETUP_IX1_OFFSET(ab)': 'consumer_int_setup_ix1',
             'reg_base + HAL_TCL1_RING_MISC_OFFSET(ab)': 'misc', 'reg_base + HAL_TCL1_RING_TP_OFFSET': 'tp'}
    L, dw, ent = 0xa1400000, got['hal_wbm_link_desc'] // 4, n
    want_regs = []
    for g, a in seq:
        k = names[a]
        if g == 'R2':
            want_regs.append((wbm + hp + (tpoff if k == 'tp' else 0), 0))
        elif k is None:
            want_regs.append((r0, L))
        else:
            v = {'base_msb': (L >> 32) | (dw * ent) << 8, 'id': dw,
                 'consumer_int_setup_ix0': define('HAL_SRNG_INT_TIMER_THRESHOLD_OTHER', hal_h) << 16 |
                 define('HAL_SRNG_INT_BATCH_THRESHOLD_OTHER', hal_h) * dw, 'consumer_int_setup_ix1': 0, 'misc': misc}[k]
            want_regs.append((r0 + off[k], v))
    assert len(want_regs) == 8 and want_regs[-1][0] == r0 + off['misc'], want_regs
    assert 'if (srng->ring_id == HAL_SRNG_RING_ID_WBM_IDLE_LINK)\n\t\tval |= HAL_TCL1_RING_MISC_MSI_RING_ID_DISABLE;' in hw
    assert 'case HAL_WBM_IDLE_LINK:' not in th.split('case HAL_RXDMA_BUF:', 1)[1].split('break;', 1)[0]  # no LOW flag for ring 120
    # The shadow register WLAN_CFG gave ring 120's R2 HP, and the PDEV_SET_PARAM facts.
    shadow = [int(x, 16) for x in re.findall(r'0x[0-9a-f]{8}', between(DRIVER, 'static const uint32_t wl15_shadow_v3[36] = {', '};'))]
    six = shadow.index(wbm + hp)
    assert six == 19 and define('HAL_SHADOW_BASE_ADDR', hal_h) + 4 * six == 0x948
    for ours, theirs in (('WLWMI_CMD_PDEV_SET_PARAM', 'WMI_PDEV_SET_PARAM_CMDID'), ('WLWMI_TAG_PDEV_SET_PARAM_CMD', 'WMI_TAG_PDEV_SET_PARAM_CMD'),
                         ('WLWMI_PDEV_PARAM_TX_CHAIN_MASK', 'WMI_PDEV_PARAM_TX_CHAIN_MASK'),
                         ('WLWMI_PDEV_PARAM_RX_CHAIN_MASK', 'WMI_PDEV_PARAM_RX_CHAIN_MASK')):
        assert H[ours] == vals[theirs], (ours, H[ours], vals[theirs])
    assert word_names('wmi_pdev_set_param_cmd') == ['tlv_header', 'pdev_id', 'param_id', 'param_value']
    psp = flat(fn_body(wmi_c, 'int ath12k_wmi_pdev_set_param('))
    assert 'cmd->tlv_header = ath12k_wmi_tlv_cmd_hdr(WMI_TAG_PDEV_SET_PARAM_CMD, sizeof(*cmd)); cmd->pdev_id = cpu_to_le32(pdev_id); ' \
        'cmd->param_id = cpu_to_le32(param_id); cmd->param_value = cpu_to_le32(param_value);' in psp
    ant = fn_body(mac_c, 'static int __ath12k_set_antenna(')
    assert ant.index('WMI_PDEV_PARAM_TX_CHAIN_MASK,\n\t\t\t\t\ttx_ant, ar->pdev->pdev_id);') < \
        ant.index('WMI_PDEV_PARAM_RX_CHAIN_MASK,\n\t\t\t\t\trx_ant, ar->pdev->pdev_id);')
    ms = fn_body(mac_c, 'static int ath12k_mac_start(')
    assert ms.index('__ath12k_set_antenna(ar, ar->cfg_tx_chainmask, ar->cfg_rx_chainmask);') < ms.index('ath12k_reg_update_chan_list(ar, false);')
    # Every word of the model's dump against these facts (RX arena at A = 0xa1a00000, our layout offsets).
    words = {}
    for line in dump.splitlines():
        k, i, v = line.split()
        words.setdefault(k, []).append(int(v))
    A = 0xa1a00000
    plan = [(ids[0], 'HAL_RXDMA_BUF', 0, H['WLRX_REFILL_ENTRIES'], H['WLRX_OFF_REFILL']),
            (ids[1], 'HAL_RXDMA_BUF', 0, H['WLRX_MAC_ENTRIES'], H['WLRX_OFF_MAC0']),
            (ids[2], 'HAL_RXDMA_BUF', 1, H['WLRX_MAC_ENTRIES'], H['WLRX_OFF_MAC1']),
            (ids[3], 'HAL_RXDMA_DST', 0, H['WLRX_ERR_ENTRIES'], H['WLRX_OFF_ERR'])]
    for i, (rid_, t, mac, entries, roff) in enumerate(plan):
        buf = t == 'HAL_RXDMA_BUF'
        if buf and rid_ == buf0:
            hring, htype = rid['HTT_HOST1_TO_FW_RXBUF_RING'], rtype['HTT_SW_TO_SW_RING']
        elif buf:
            hring, htype = rid['HTT_RXDMA_HOST_BUF_RING'], rtype['HTT_SW_TO_HW_RING']
        else:
            hring, htype = rid['HTT_RXDMA_NON_MONITOR_DEST_RING'], rtype['HTT_HW_TO_SW_RING']
        edw = (got['hal_wbm_buffer_ring'] if buf else got['hal_reo_entrance_ring']) // 4
        pdev = mac + 1 if htype in (rtype['HTT_SW_TO_HW_RING'], rtype['HTT_HW_TO_SW_RING']) else mac
        src = cfgs[t]['ring_dir'] == 'HAL_SRNG_DIR_SRC'
        rdp, wrp = A + H['WLRX_OFF_RDP'] + 4 * rid_, A + H['WLRX_OFF_WRP'] + 4 * (rid_ - d0)
        batch = 0 if buf else define('HAL_SRNG_INT_BATCH_THRESHOLD_OTHER', hal_h)
        timer = define('HAL_SRNG_INT_TIMER_THRESHOLD_RX' if buf else 'HAL_SRNG_INT_TIMER_THRESHOLD_OTHER', hal_h)
        want = [h2t['HTT_H2T_MSG_TYPE_SRING_SETUP'] | pdev << 8 | hring << 16 | htype << 24, A + roff, 0,
                entries * edw | edw << 16 | (fld['INFO1_RING_LOOP_CNT_DIS'] if htype == rtype['HTT_SW_TO_HW_RING'] else 0),
                wrp if src else rdp, 0, rdp if src else wrp, 0, 0, 0, 0, batch * edw | (timer >> 3) << 16,
                (entries >> 3) * edw if buf else 0]
        assert words[f'sring{i}'] == want, (i, [(k, hex(a), hex(b)) for k, (a, b) in enumerate(zip(words[f'sring{i}'], want)) if a != b])
    for i, (param, mask) in enumerate((('WMI_PDEV_PARAM_TX_CHAIN_MASK', 3), ('WMI_PDEV_PARAM_RX_CHAIN_MASK', 3))):
        assert words[f'pset{i}'] == [vals['WMI_PDEV_SET_PARAM_CMDID'], tlv('PDEV_SET_PARAM_CMD', size('wmi_pdev_set_param_cmd') - 4), 0,
                                     vals[param], mask], words[f'pset{i}']
    B, D, sw1, dev0 = A + H['WLRX_OFF_BUFS'], L + H['WLRX_LINK_OFF_DESC'], rbm['HAL_RX_BUF_RBM_SW1_BM'], rbm['HAL_RX_BUF_RBM_WBM_DEV0_IDLE_DESC_LIST']
    cookie = lambda d_id: ((d_id + define('DP_LINK_DESC_START', dp_h)) << define('DP_LINK_DESC_SHIFT', dp_h)) | 0  # noqa: E731
    assert words['refill'] == [B, sw1 << 8 | 0 << 12, B + 4094 * H['WLRX_SLOT_BYTES'], sw1 << 8 | 4094 << 12], words['refill']
    assert words['idle'] == [D, dev0 << 8 | cookie(n - 2) << 12, D + (n - 2) * 128, dev0 << 8 | cookie(0) << 12], words['idle']
    assert list(zip(words['wbm'], words['wbmv'])) == want_regs, (words['wbm'], words['wbmv'], want_regs)
    assert words['pre'] == pre and words['shadow'] == [six] and words['hp'] == [(n - 1) * dw], (words['pre'], pre)
    print('L19 re-derivation (C2c): ring ids 120/160/161/169/186 (enum, PMAC stride, setups), rdp 207 and wrp 47 words, the RX '
          'alloc and HTT announcement order, ring sizes, buffer and link descriptor sizes, struct sizes from compiled replicas '
          '(buffer 8, entrance 32, link 8, SRING_SETUP 52, PDEV_SET_PARAM 16), 32767 links on the ring path, HTT enums, fields '
          'and encoder rules, thresholds, remote pointer words, RBMs 6 and 1, entry fields, the WCN7850 WBM registers incl. the '
          'idle-list and scattered-base ones, ring 120\'s 8 writes in L19\'s order with L19\'s values, shadow 19 = 0x948, '
          'PDEV_SET_PARAM 0x4003/tag 0x52/ids 1, 2 TX then RX before the channel list, the error entry\'s frame-field masks '
          '(w5 byte count, w6 fragment and sequence); the model\'s dump word by word')


def dash_check():
    files = ['xnu/fuxi-wlan/fuxi_wlan.c', 'xnu/fuxi-wlan/fuxi_wlan_ce.h', 'xnu/patches/0099-fuxi-wlan.patch',
             'tools/fuxi-wlan-ce-model.c', 'tools/test-fuxi-wlan-ce.py', 'tools/test-fuxi-wlan-salami.py',
             'tools/mkadt-salami.py', 'tools/ci/build.sh', 'docs-lanes/wlan-w20.md', 'tools/test-fuxi-wlan-m6.py',
             'tools/test-fuxi-wlan-w11.py', 'tools/test-fuxi-wlan-w15.py', 'tools/test-fuxi-wlan-w16.py']
    for f in files:
        text = (ROOT / f).read_text()
        assert chr(0x2013) not in text and chr(0x2014) not in text, f
    print(f'no en or em dash in the {len(files)} W20 files')


def main():
    static_checks()
    with tempfile.TemporaryDirectory(prefix='fuxi-wlan-ce-') as tmp:
        (Path(tmp) / 'fuxi-wlan-ce-prod.inc').write_text(production())
        exe = Path(tmp) / 'model'
        subprocess.run(CC + ['-I', tmp, '-I', str(ROOT / 'xnu/fuxi-wlan'), str(MODEL), '-o', str(exe)], check=True)
        res = subprocess.run([str(exe)], capture_output=True, text=True)
        out = res.stdout
        print(out.strip())
        if res.returncode:
            print(res.stderr[-6000:])
            raise SystemExit(f'W20 model failed with status {res.returncode}')
        assert out.strip().endswith('W20 model: all scenarios passed')
        dump = subprocess.run([str(exe), 'dump-init'], check=True, capture_output=True, text=True).stdout
        c2dump = subprocess.run([str(exe), 'dump-c2'], check=True, capture_output=True, text=True).stdout
        c2cdump = subprocess.run([str(exe), 'dump-c2c'], check=True, capture_output=True, text=True).stdout
        c2ddump = subprocess.run([str(exe), 'dump-c2d'], check=True, capture_output=True, text=True).stdout
        c2edump = subprocess.run([str(exe), 'dump-c2e'], check=True, capture_output=True, text=True).stdout
        c2fdump = subprocess.run([str(exe), 'dump-c2f'], check=True, capture_output=True, text=True).stdout
    l19_checks(dump, c2dump, c2cdump, c2ddump)
    l19_c2e(c2edump)
    l19_c2f(c2fdump)
    mutants()
    mutants(C2E_MUTANTS, 'c2e')
    mutants(C2F_MUTANTS, 'c2f')
    dash_check()
    print('W20 (fuxiwlanqrtr=6) CE/HTC/WMI, W23 (fuxiwlanqrtr=7) passive scan, C2c (W26), C2d (W27), C2e (W28) and C2f (W29) '
          'host tests passed')


if __name__ == '__main__':
    main()
