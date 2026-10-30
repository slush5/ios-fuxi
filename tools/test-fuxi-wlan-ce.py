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
    assert '#define WL23_BITS (WL20_BITS | WL20_HTTVER | WL20_BAND24 | WL20_PDEVCAP)' in DRIVER
    bits23 = DRIVER.split('#define WL23_BITS', 1)[1].split('\n', 1)[0]
    assert '#define WL20_ACTIVE 0x08u' in DRIVER and 'ACTIVE' not in bits23 and 'FULLBSSID' not in bits23
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
    assert '&wl14_m3, &wl_ce_arena };' in owned
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
    call = 'if (wl23_stats()) { wl23_ci_flush(); return 1; }'
    assert DRIVER.count('wl23_stats()') == 1 and w23.count(call) == 1
    # the hint names W24b only while w20 0x02 is off; r1c2b ran 0x02 and still heard nothing
    assert '(wl20_bits & WL20_HTTVER) ? "C2c = RX data path" : "W24b = w20 0x02"' in w23
    assert w23.index('next rung %s') < w23.index(call) < \
        w23.index('wl23_ci_flush();\n\tprintf("FUXI WLAN: W23 complete; stop/contain\\n");')
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
    assert 'if (wl_scan.ci_seen != ci) { wl23_ci_line(); }' in rx
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


def header_define(name):
    m = re.search(r'^#define ' + name + r' (.+?)(?:\s*/\*.*)?$', HEADER, re.M)
    assert m, name
    return eval(re.sub(r'\b(0x[0-9a-fA-F]+|\d+)u\b', r'\1', m[1]), {}, {})


def l19_text(name):
    return re.sub(r'//[^\n]*', '', re.sub(r'/\*.*?\*/', '', (L19 / name).read_text(), flags=re.S))


def l19_checks(model, c2dump):
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
    l19_c2(vals, ev, struct_fields, size, word_names, wmi_h, wmi_c, c2dump)


def fn_body(text, head):
    """The C function whose definition starts with HEAD at column 0, up to its closing brace at column 0."""
    i = text.index('\n' + head) + 1
    return text[i:text.index('\n}\n', i)]


def l19_c2(vals, ev, struct_fields, size, word_names, wmi_h, wmi_c, dump):
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
    l19_checks(dump, c2dump)
    dash_check()
    print('W20 (fuxiwlanqrtr=6) CE/HTC/WMI and W23 (fuxiwlanqrtr=7) passive scan host tests passed')


if __name__ == '__main__':
    main()
