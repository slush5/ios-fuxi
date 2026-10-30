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
No device, no network.
"""
from pathlib import Path
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
    assert '#define WL23_BITS (WL20_BITS | WL20_HTTVER | WL20_BAND24 | WL20_PDEVCAP | WL26_BITS)' in DRIVER
    assert '#define WL26_BITS (WL20_RXRING | WL20_IDLELINK | WL20_CHAINMASK)' in DRIVER
    bits23 = DRIVER.split('#define WL23_BITS', 1)[1].split('\n', 1)[0]
    bits26 = DRIVER.split('#define WL26_BITS', 1)[1].split('\n', 1)[0]
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
    assert '&wl14_m3, &wl_ce_arena, &wl_rx_arena, &wl_link_arena };' in owned
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
    assert '((unsigned)w20 & (WL20_RXRING | WL20_HTTVER)) == WL20_RXRING || stop22 == 1 || keep == 1) { wl_variant_invalid = true; }' in cfg
    assert 'unsigned allowed = wl_w23 ? WL23_BITS : WL20_BITS;' in cfg and 'WL26' not in DRIVER.split('#define WL20_BITS', 1)[1].split('\n')[0]
    assert cfg.index('if (wl20_bits & WL26_BITS) {') > cfg.index('htt version %s')  # the W26 selected line after the W23 one
    region = between(DRIVER, W26_START, 'static int\nwl_w23_run(void)')
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
    assert strip_comments(block).count('wl_win(') == 3, 'wl_win: the definition and the two accessors only'
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
    assert re.findall(r'bzero\(\(void \*\)(wl_\w+_arena)\.va', code) == ['wl_rx_arena', 'wl_link_arena'] and \
        all(f'bzero((void *)wl_{a}_arena.va' in fns['wl26_free'] for a in ('rx', 'link'))
    assert 'WLRX_PUBLISH(' in between(HEADER, 'static inline int\nwlrx_publish_refill(', '\n}\n') and \
        HEADER.count('WLRX_PUBLISH(') == 2  # the default macro and the one use
    # HTT: only SRING_SETUP (0xb), RESPONSE_REQUIRED never set, no ring selection / PPDU / EXT stats encoder.
    h26 = strip_comments(HEADER.split(H26_START, 1)[1])
    assert 'WLRX_HTT_SRING_SETUP |' in h26 and h26.count('WLRX_HTT_RESPONSE_REQUIRED') == 1  # the define only
    for word in ('RING_SELECTION', 'PPDU_STATS', 'EXT_STATS', '0xcu', '0x11u'):
        assert word not in w26code and word not in h26, word
    # PDEV_SET_PARAM: params 1 and 2 only, TX then RX, through the C2 one-TLV helper.
    cm = fns['wl26_chainmask']
    assert cm.index('WLWMI_PDEV_PARAM_TX_CHAIN_MASK, tx }') < cm.index('WLWMI_PDEV_PARAM_RX_CHAIN_MASK, rx }')
    assert cm.index('wl23_words(WLWMI_CMD_PDEV_SET_PARAM, WLWMI_TAG_PDEV_SET_PARAM_CMD, t,') < \
        cm.index('wl23_words(WLWMI_CMD_PDEV_SET_PARAM, WLWMI_TAG_PDEV_SET_PARAM_CMD, r,')
    assert re.findall(r'#define WLWMI_PDEV_PARAM_\w+', HEADER) == ['#define WLWMI_PDEV_PARAM_TX_CHAIN_MASK',
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


def c2c_line_bounds(fns, cfg):
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
    for name in ('wlrx_idle_why', 'wlrx_next'):
        body = between(HEADER, f'static inline const char *\n{name}(', '\n}\n')
        pure[name] = max(len(''.join(x)) for x in re.findall(r'return [^;]*?"([^"]*)"(?:\s*"([^"]*)")?', body) +
                         re.findall(r'[:?]\s*"([^"]*)"(?:\s*"([^"]*)")?', body))
    bounded = {'tp %u taken %u left %u': [4, 4, 4], 'hp %u tp %u avail %u': [5, 5, 5], 'hp %u fill %u': [4, 4]}

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
        local, assigned, sizes = {}, {}, {}
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
            if name == 'wl_probe_configure' and 'W26' not in fmt:
                continue
            if name == 'wl26_list':
                assert fmt == '%s\n', fmt
                continue
            assert fmt.startswith(('FUXI WLAN: W26 ', 'FUXI WLAN: W24 no beacon')) and fmt.endswith('\n'), fmt
            w = worst(fmt, a[1:], local, assigned)
            assert w <= 254, (name, w, fmt)
            longest = max(longest, w)
            lines += 1
    lst = fns['wl26_list']
    assert 'char line[256], item[48];' in lst and 'if (len && len + (size_t)k > 254u) { printf("%s\\n", line); len = 0; }' in lst
    assert 'if (k <= 0 || (size_t)k >= sizeof(item)) { continue; }' in lst
    assert len(' grpcast-amsdu-wds 4294967295') < 48 and len('FUXI WLAN: W26 rxdma err codes') + len(' other 4294967295') < 254
    assert lines >= 28, lines
    return longest


def header_define(name):
    m = re.search(r'^#define ' + name + r' (.+?)(?:\s*/\*.*)?$', HEADER, re.M)
    assert m, name
    return eval(re.sub(r'\b(0x[0-9a-fA-F]+|\d+)u\b', r'\1', m[1]), {}, {})


def l19_text(name):
    return re.sub(r'//[^\n]*', '', re.sub(r'/\*.*?\*/', '', (L19 / name).read_text(), flags=re.S))


def l19_checks(model, c2dump, c2cdump=''):
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
    l19_c2(vals, ev, struct_fields, size, word_names, wmi_h, wmi_c, c2dump, c2cdump)


def fn_body(text, head):
    """The C function whose definition starts with HEAD at column 0, up to its closing brace at column 0."""
    i = text.index('\n' + head) + 1
    return text[i:text.index('\n}\n', i)]


def l19_c2(vals, ev, struct_fields, size, word_names, wmi_h, wmi_c, dump, c2cdump=''):
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
    l19_checks(dump, c2dump, c2cdump)
    dash_check()
    print('W20 (fuxiwlanqrtr=6) CE/HTC/WMI, W23 (fuxiwlanqrtr=7) passive scan and C2c (W26) host tests passed')


if __name__ == '__main__':
    main()
