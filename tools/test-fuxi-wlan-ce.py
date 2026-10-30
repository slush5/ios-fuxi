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
No device, no network.
"""
from pathlib import Path
import importlib.util
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
    assert DRIVER.count('wl_w23 = qrtr == 7 || wl_w25;') == 1 and DRIVER.count('wl_w25 = qrtr == 8;') == 1
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
                                                  'WL20_ACTIVE 0x08u', 'WL20_QRTR_MAX 8', 'WL20_HTTVER 0x02u',
                                                  'WL20_BAND24 0x10u', 'WL20_FULLBSSID 0x20u', 'WL20_PDEVCAP 0x80u'))
    assert '#define WL20_BITS (WL20_NOHTT | WL20_MRRS128 | WL20_NOHWMODE)' in DRIVER
    assert 'python3 tools/test-fuxi-wlan-ce.py' in (ROOT / 'tools/ci/build.sh').read_text()
    print('W20 static: gated call, containment before the W11 stop, arena in the owned/free lists, '
          'only the window/shadow/devctl writers, pure header with the Clear BSD text')
    print('W23 static: scan only after W22 with qrtr 7, STOP_SCAN before the latch with no wait/poll/RX and not after a '
          'W11 guard failure, passive flags, no SSID/IE/probe words, ACTIVE and FULLBSSID outside every accepted set, '
          'BSSIDs always masked, vdev MAC = en0 K-W2 address (locally administered)')
    c3_static_checks()


def c3_static_checks():
    """C3 (fuxiwlanqrtr=8): where the W25 service runs, what it may send and say, the 0099 hooks, the handoff copy."""
    w23 = between(DRIVER, 'static int\nwl_w23_run(void)\n{', '\n}\n')
    hook = 'if (wl_w25) { return wl_w25_run(n); }'
    assert w23.count(hook) == 1 and w23.index(hook) > w23.index('WMI STA_POWERSAVE_MODE') and \
        w23.index(hook) < w23.index('wlwmi_encode_start_scan(') and DRIVER.count('wl_w25_run(') == 2
    svc = between(DRIVER, '/* ---- W25: the scan service for iOS (fuxiwlanqrtr=8, C3) ----', 'static int\nwl_w23_run(void)')
    code = re.sub(r'/\*.*?\*/', '', svc, flags=re.S)
    # it sends START_SCAN (the passive list encoder) and STOP_SCAN, nothing else; no register writer of its own
    assert re.findall(r'\bWLWMI_CMD_\w+', code) == ['WLWMI_CMD_STOP_SCAN'], re.findall(r'\bWLWMI_CMD_\w+', code)
    assert re.findall(r'wl_ce_send\(([^)]*)\)', code) == ['wl_htc.wmi_eid, cmd, len, "WMI START_SCAN"']
    assert code.count('wlwmi_encode_start_scan_list(') == 1 and 'wlwmi_encode_start_scan(' not in code
    assert code.count('wl23_words(') == 1 and 'wlce_send(' not in code and 'wlhtc_' not in code
    for forbidden in ('wl_pwr', 'wl_mwr', 'wl_contain_wr', 'wl_soc_write', 'wlce_wr', 'wlce_publish', 'WL20_ACTIVE',
                      'printf("FUXI WLAN: W24'):
        assert forbidden not in code, forbidden
    # passive refused before anything is sent; the request's channels only through the list check
    scan = between(DRIVER, 'static int\nwl25_scan(const struct fuxi_wifi_radio_request *rq, uint32_t nlist)\n{', '\n}\n')
    assert scan.index('if (!rq->passive || !n) {') < scan.index('wlwmi_encode_start_scan_list(') < scan.index('wl_ce_send(')
    assert 'wlscan_listed(rq->freqs[i], nlist)' in scan and scan.count('fuxi_wifi_radio_done(') == 2
    # every pass of both loops polls (W11 events/guard, RX, TX, credits); 1 ms while scanning, 10 ms idle
    run = between(DRIVER, 'static int\nwl_w25_run(uint32_t n)\n{', '\n}\n')
    assert run.index('if (wl_ce_poll()) { return 1; }') < run.index('if (wl25_ms() - wl25_t0 >= WL25_LIFETIME_MS) {') < \
        run.index('fuxi_wifi_radio_take(')
    assert 'wl_sleep(10)' in run and 'wl_sleep(1)' not in run and 'wl_sleep(1);' in scan and 'wl_sleep(10)' not in scan
    assert scan.count('if (wl_ce_poll()) { return 1; }') == 1
    wait_end = between(DRIVER, 'static int\nwl25_wait_end(unsigned ms)\n{', '\n}\n')
    assert wait_end.index('if (wl_ce_poll()) { return -1; }') < wait_end.index('if (wl_scan.done) { return 1; }') < \
        wait_end.index('wl_sleep(1);') and 'wl_sleep(10)' not in wait_end
    # the iOS side's stop ends the service only while idle (nothing in flight), never mid-scan
    idle = run.split('if (!fuxi_wifi_radio_take(&rq)) {', 1)[1].split('\n\t\t}\n', 1)[0]
    assert 'if (!fuxi_wifi_radio_wanted()) {' in idle and 'return 0;' in idle and DRIVER.count('fuxi_wifi_radio_wanted()') == 1
    # an unconfirmed stop: done() first (the iOS side's margin), then the wait for the end, before the next take()
    assert scan.index('fuxi_wifi_radio_done(rq->seq, status);\n\tif (!ended) {') < \
        scan.index('wl25_wait_end(WL25_ABORT_MS - WL25_STOP_MS)')
    assert '#define WL25_STOP_MS 300u' in DRIVER and '#define WL25_ABORT_MS 3000u' in DRIVER
    # what W25 says: never an SSID or a BSSID (the W24 proof lines come only from wl24_report, once)
    for fmt in re.findall(r'printf\("((?:[^"\\]|\\.)*)"', code):
        assert 'ssid' not in fmt.lower() and '%02x' not in fmt, fmt
    assert code.count('wl24_report()') == 1 and 'if (status == 0 && !wl25_proved) { wl25_proved = true; wl24_report(); }' in code
    # the list encoder: C2's START_SCAN with id, dwell and channels replaced; no flags, SSID, IE or probe word
    enc = between(HEADER, 'static inline int\nwlwmi_encode_start_scan_list(', '\n}\n')
    assert 'wlwmi_encode_start_scan(n, max_time, out, room, len)' in enc
    for word in ('CTRL_FLAGS', 'NUM_SSIDS', 'IE_LEN', 'N_PROBES', 'PRIORITY', 'IE_BITMAP', 'MAC_ADDR', 'NOTIFY'):
        assert word not in enc, word
    assert re.findall(r'WLWMI_SS_\w+', enc) == ['WLWMI_SS_SCAN_ID', 'WLWMI_SS_DWELL_PASSIVE']
    # the 0099 hooks: STARTING just before the thread, STOPPED once at every other exit of fuxi_wlan_init and
    # after the thread's containment ("thread exit"), none of them printing
    init = between(DRIVER, 'void\nfuxi_wlan_init(void)\n{', '\n}\n')
    assert init.index('fuxi_wifi_radio_starting();\n\tif (kernel_thread_start(') > 0
    assert [m for m in re.findall(r'fuxi_wifi_radio_stopped\(([^)]*)\)', init)] == [
        'FUXI_WIFI_RADIO_REFUSED | 1', 'FUXI_WIFI_RADIO_NOT_REQUESTED', 'FUXI_WIFI_RADIO_REFUSED | 2', 'FUXI_WIFI_RADIO_NO_THREAD']
    thread = between(DRIVER, 'static void\nfw_thread(void *arg, wait_result_t result)\n{', '\n}\n')
    assert 'fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | detail);' in thread, 'fw_thread does not report EXITED'
    assert thread.index('printf("FUXI WLAN: thread exit\\n");') < thread.index(
        'fuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | detail);') < thread.index('thread_terminate(')
    assert thread.index('fw_run(limit);\n\tdetail = wl4_report[3] & 0xffffu;') > 0 and thread.count('fuxi_wifi_radio_') == 1
    assert DRIVER.count('fuxi_wifi_radio_stopped(') == 6 and DRIVER.count('fuxi_wifi_radio_starting(') == 2
    # the handoff copied from the 0104 header, word for word
    hdr = (ROOT / 'xnu/fuxi-wifi-ios/fuxi_wifi_radio.h').read_text()

    def norm(text):
        return re.sub(r'\s+', ' ', re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)).strip()
    nh = norm(hdr)
    mirror = between(DRIVER, '/* ---- the scan handoff of patch 0104 (bsd/sys/fuxi_wifi_radio.h), producer half ----',
                     '/* ---- end scan handoff ---- */')
    externs = re.findall(r'extern ([^;]+;)', norm(mirror))
    assert len(externs) == 8 and all(e in nh for e in externs), externs
    struct = re.search(r'struct fuxi_wifi_radio_request \{.*?\};', norm(mirror))[0]
    assert struct in nh, struct
    defines = re.findall(r'#define (FUXI_WIFI_RADIO_\w+) ([^#]+?) (?=#|struct|$)', norm(mirror))
    assert len(defines) == 5 and all(f'#define {k} {v} ' in nh + ' ' for k, v in defines), defines
    print('W25 static: the service only from W23 after the vdev with qrtr 8; START_SCAN (passive list encoder) and '
          'STOP_SCAN only, no register writer; non-passive and unlisted refused before any encode/send; a poll every '
          'pass, 1 ms scanning / 10 ms idle; no SSID/BSSID in W25 lines, W24 once; 0099 hooks: STARTING before the '
          'thread, STOPPED at each exit and after "thread exit"; the handoff copy (8 prototypes, the struct, 5 '
          'defines) equals sys/fuxi_wifi_radio.h; the iOS side\'s stop ends the service only while idle; an unconfirmed '
          'stop answers iOS first, then waits up to 3 s for the end')


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
              'ATH12K_WMI_SSID_LEN'):
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


# ---- C3 end to end: the real iOS-side handoff (fuxi_wifi_radio.c, built as tools/test-fuxi-wifi-radio.py builds
# it) in one unit, the radio lane model (-DFUXI_WLAN_E2E: no fake handoff) in another, linked together. ----
E2E_IOS = r'''
/* ---- C3 end to end: the consumer (fuxiwifiios=3) of the scan handoff; the radio lane model is the producer ---- */
bool fuxi_wifi_net_attach(const struct fuxi_wifi_net_link *p) { (void)p; attached++; return false; }   /* (en0: not here) */
static atomic_int e2e_phase;           /* 0 waits for READY, 1 asking, 2 collecting, 3 done */
static int e2e_status = -1;
static unsigned e2e_changed_calls;
static enum fuxi_wifi_radio_state e2e_changed_last;
static struct fuxi_wifi_sta_bss e2e_got[8];
static unsigned e2e_n;
static pthread_t e2e_thread;
static void e2e_changed(enum fuxi_wifi_radio_state s) { e2e_changed_last = s; e2e_changed_calls++; }
static void e2e_each(const struct fuxi_wifi_sta_bss *b, void *ctx) { (void)ctx; if (e2e_n < 8) e2e_got[e2e_n++] = *b; }
static bool e2e_said(const char *what) {
 pthread_mutex_lock(&log_lock); bool s = strstr(logbuf, what) != NULL; pthread_mutex_unlock(&log_lock); return s;
}
/* the station thread's part: one scan of {2437, 5180}, passive, 150 ms, waited for as rscan_scan waits */
static void *e2e_consumer(void *a) {
 struct timespec pause = { .tv_nsec = 200000 };
 (void)a;
 for (unsigned i = 0; i < 100000 && fuxi_wifi_radio_state(NULL) != FUXI_WIFI_RADIO_READY; i++) nanosleep(&pause, NULL);
 assert(fuxi_wifi_radio_state(NULL) == FUXI_WIFI_RADIO_READY);
 struct fuxi_wifi_radio_request rq = { .passive = true, .dwell_ms = 150, .freq_count = 2 };
 rq.freqs[0] = 2437; rq.freqs[1] = 5180;
 atomic_store(&e2e_phase, 1);
 e2e_status = fuxi_wifi_radio_scan(&rq, 1000 + 2 * 150 * 3 / 2, NULL);
 atomic_store(&e2e_phase, 2);
 (void)fuxi_wifi_radio_results(e2e_each, NULL);
 atomic_store(&e2e_phase, 3);
 return NULL;
}
void e2e_ios_start(void) {
 fuxi_wifi_radio_starting();                            /* fuxi_wlan_init, before launchd */
 assert(fuxi_wifi_radio_use(false, e2e_changed));       /* fuxiwifiios=3 */
 fuxi_wifi_radio_attached();
 other_settle();
 assert(!pthread_create(&e2e_thread, NULL, e2e_consumer, NULL));
}
/* every radio-lane sleep: the iOS side's clock follows the model's; while READY and nothing is asked, or a
   finished scan is being collected, the lane waits (real time) for the consumer */
void e2e_tick(uint64_t ms) {
 struct timespec pause = { .tv_nsec = 100000 };
 now_abs = ms * NSEC_PER_MSEC;
 for (unsigned i = 0; i < 200000; i++) {
  bool scanning = false;
  if (atomic_load(&e2e_phase) == 3 || fuxi_wifi_radio_state(&scanning) != FUXI_WIFI_RADIO_READY || scanning) return;
  nanosleep(&pause, NULL);
 }
 assert(!"e2e: the consumer never asked or never collected");
}
int e2e_ios_finish(void) {
 static const uint8_t home[6] = {0x3c, 0x84, 0x6a, 0x11, 0x22, 0x33}, office[6] = {0x74, 0x83, 0xc2, 0xaa, 0xbb, 0xcc};
 struct timespec pause = { .tv_nsec = 1000000 };
 pthread_join(e2e_thread, NULL);
 for (unsigned i = 0; i < 5000 && !e2e_said("Wi-Fi reported off"); i++) nanosleep(&pause, NULL);
 other_settle();
 assert(e2e_status == 0 && e2e_n == 2);
 for (unsigned i = 0; i < 2; i++) {
  const struct fuxi_wifi_sta_bss *b = &e2e_got[i];
  bool is_home = !memcmp(b->bssid, home, 6);
  assert(is_home || !memcmp(b->bssid, office, 6));
  if (is_home) assert(b->ssid_length == 7 && !memcmp(b->ssid, "HomeNet", 7) && b->frequency == 2437 && b->rssi == -48);
  else assert(b->ssid_length == 9 && !memcmp(b->ssid, "Office-5G", 9) && b->frequency == 5180 && b->rssi == -45);
  assert(b->noise == -95 && b->beacon_interval == 100 && (b->capability & 0x11) == 0x11 && b->elements_length > 20);
 }
 assert(memcmp(e2e_got[0].bssid, e2e_got[1].bssid, 6));
 assert(e2e_said("FUXI WIFI IOS: radio: radio producer ready, 38 channels (13 at 2.4 GHz, 25 at 5 GHz), passive only\n"));
 assert(e2e_said("FUXI WIFI IOS: radio: scan 1: 2 channels passive, 2 heard (1 at 2.4 GHz, 1 at 5 GHz, 2 RSN, 0 hidden), "));
 assert(e2e_said("FUXI WIFI IOS: radio: the radio lane stopped (code 0x40000); Wi-Fi reported off, no retry this boot\n"));
 assert(e2e_changed_calls == 2 && e2e_changed_last == FUXI_WIFI_RADIO_STOPPED);
 assert(fuxi_wifi_radio_state(NULL) == FUXI_WIFI_RADIO_STOPPED && fuxi_wifi_radio_results(e2e_each, NULL) == 0);
 /* the iOS side never says a name or a BSSID */
 assert(!e2e_said("HomeNet") && !e2e_said("Office") && !e2e_said("Cafe") && !e2e_said("3c:84") && !e2e_said("74:83"));
 return 0;
}
'''


def e2e(tmp):
    import sys
    import types
    try:
        import cryptography.hazmat.primitives.keywrap  # noqa: F401  (test-fuxi-wifi-ios.py's WPA peer needs it)
    except ImportError:
        # Not for this test: only test-fuxi-wifi-ios.py's MODEL and unit_source are used; its WPA peer never runs.
        for name in ('cryptography', 'cryptography.hazmat', 'cryptography.hazmat.primitives',
                     'cryptography.hazmat.primitives.keywrap'):
            sys.modules.setdefault(name, types.ModuleType(name))
        sys.modules['cryptography.hazmat.primitives.keywrap'].aes_key_wrap = None
    spec = importlib.util.spec_from_file_location('test_wifi_ios_e2e', ROOT / 'tools/test-fuxi-wifi-ios.py')
    w = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(w)
    ios = Path(tmp) / 'e2e-ios.c'
    ios.write_text(w.unit_source(w.MODEL, ROOT / 'xnu/fuxi-wifi-ios') + E2E_IOS)
    cc = os.environ.get('CC', 'cc')
    san = ['-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
    subprocess.run([cc, '-std=gnu11', '-g', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                    '-Wno-unused-parameter', '-pthread', *san, '-c', str(ios), '-o', str(Path(tmp) / 'e2e-ios.o')], check=True)
    subprocess.run(CC + ['-DFUXI_WLAN_E2E=1', '-I', tmp, '-I', str(ROOT / 'xnu/fuxi-wlan'), '-c', str(MODEL), '-o',
                         str(Path(tmp) / 'e2e-lane.o')], check=True)
    exe = Path(tmp) / 'e2e'
    subprocess.run([cc, *san, '-pthread', str(Path(tmp) / 'e2e-lane.o'), str(Path(tmp) / 'e2e-ios.o'), '-o', str(exe)], check=True)
    res = subprocess.run([str(exe), 'e2e'], capture_output=True, text=True, timeout=600)
    line = [l for l in res.stdout.splitlines() if l.startswith('W25 end to end')]
    if res.returncode or not line:
        print(res.stdout[-3000:], res.stderr[-6000:])
        raise SystemExit(f'W25 end-to-end test failed with status {res.returncode}')
    print(line[0])


# ---- C3 mutants: each must fail a test for its own reason ----
MUTANTS = [
    ('the passive check dropped', 'xnu/fuxi-wlan/fuxi_wlan.c', '\tif (!rq->passive || !n) {', '\tif (!n) {',
     'model', 'a request that was not passive reached the firmware as a START_SCAN'),
    ('the NOT_REQUESTED stop dropped', 'xnu/fuxi-wlan/fuxi_wlan.c',
     '\tif (!requested) {\n\t\tfuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_NOT_REQUESTED);\n\t\treturn;',
     '\tif (!requested) {\n\t\treturn;', 'gate', "AssertionError: ('', '', "),
    ('the EXITED stop dropped', 'xnu/fuxi-wlan/fuxi_wlan.c',
     '\tfuxi_wifi_radio_stopped(FUXI_WIFI_RADIO_EXITED | detail);\n', '', 'static', 'fw_thread does not report EXITED'),
    ('no RX re-post', 'xnu/fuxi-wlan/fuxi_wlan_ce.h', '\t\tif (wlce_post_rx(ce, io, p, 1)) { return 1; }\n', '',
     'model', 'RX ring starved: a due message found no posted RX buffer for 1000 ms'),
    ('the lifetime bound broken', 'xnu/fuxi-wlan/fuxi_wlan.c', 'if (wl25_ms() - wl25_t0 >= WL25_LIFETIME_MS) {',
     'if (wl25_ms() - wl25_t0 >= 2u * WL25_LIFETIME_MS) {', 'model', 'the W25 service ran past its 1800 s lifetime'),
    ('the scan id filter dropped', 'xnu/fuxi-wlan/fuxi_wlan_ce.h',
     '\tif (s->want_id && s->ev_scan_id != s->want_id) { s->wrong_id++; return 0; }   /* C3: another scan\'s */\n', '',
     'model', "another id's STARTED ignored"),
    ('no STOP_SCAN on a cancel', 'xnu/fuxi-wlan/fuxi_wlan.c', '\tif (stop) {\n\t\tint s = wl25_stop();',
     '\tif (false && stop) {\n\t\tint s = wl25_stop();', 'model', 'a cancel: STOP_SCAN of that scan, ECANCELED, the next request served'),
    ('3 failures never contain', 'xnu/fuxi-wlan/fuxi_wlan.c', 'if (wl25_fails >= WL25_FAILS_MOST) {',
     'if (wl25_fails >= 100u * WL25_FAILS_MOST) {', 'model', '3 failed scans in a row did not contain'),
    ('containment STOP_SCAN names the C2 id 0xa000', 'xnu/fuxi-wlan/fuxi_wlan.c',
     'const uint32_t w[5] = { 0, wl23_scan_id, WLWMI_SCAN_STOP_ONE, WLWMI_VDEV_ID, wl23_pdev };\n\tuint32_t len = 0, total = 0;',
     'const uint32_t w[5] = { 0, WLWMI_SCAN_ID, WLWMI_SCAN_STOP_ONE, WLWMI_VDEV_ID, wl23_pdev };\n\tuint32_t len = 0, total = 0;',
     'model', 'W25 STOP_SCAN requester/scan id of the scan in flight/STOP_ONE/vdev'),
    ('overrun deadline from the 150 ms dwell', 'xnu/fuxi-wlan/fuxi_wlan.c', 'if (t >= max_ms + WL25_OVERRUN_MS ||',
     'if (t >= wlscan_max_time(n) + WL25_OVERRUN_MS ||', 'model',
     "STOP_SCAN at max_scan_time + 200 ms with the request's dwell (600 + 200)"),
    ('an unconfirmed cancel is not a failure', 'xnu/fuxi-wlan/fuxi_wlan.c',
     'failed = status != 0 && !(status == ECANCELED && ended);', 'failed = status != 0 && status != ECANCELED;',
     'model', 'missing log text: scans 2 (completed 1, failed 1, cancelled 0, refused 0)'),
    ('no wait for the end of an unconfirmed stop', 'xnu/fuxi-wlan/fuxi_wlan.c',
     '\t\tint s = wl25_wait_end(WL25_ABORT_MS - WL25_STOP_MS);', '\t\tint s = 0;',
     'model', 'unanswered cancel: ECANCELED, then the next START_SCAN 3 s after the STOP_SCAN'),
    ("the iOS side's stop never ends the service", 'xnu/fuxi-wlan/fuxi_wlan.c', 'if (!fuxi_wifi_radio_wanted()) {',
     'if (false && !fuxi_wifi_radio_wanted()) {', 'model', 'the service did not end when the iOS side stopped'),
    ('ignored scan ids never said', 'xnu/fuxi-wlan/fuxi_wlan.c', 'if (wl25_id_lines++ < 8u) {',
     'if (false && wl25_id_lines++ < 8u) {', 'model', 'missing or out of order: FUXI WLAN: W25 ignored scan event'),
    ('W25 credit-wait line not capped', 'xnu/fuxi-wlan/fuxi_wlan.c',
     'if (wl12_stage != WL25_SERVICE || ++wl25_waits <= 16u || wl25_waits % 100u == 0) {',
     'if (wl12_stage != WL25_SERVICE || ++wl25_waits > 0u) {', 'model', 'W25 credit-wait lines past the cap'),
    ('failed scans past 24 not said', 'xnu/fuxi-wlan/fuxi_wlan.c', '!(failed && wl25_failed <= WL25_SAID_MOST)',
     '!(failed && wl25_failed > 1000000u)', 'model', 'W25 lines: 1..24, every 100th and the failed 30th'),
]


def mutants(tmp):
    """Each mutant of fuxi_wlan.c / fuxi_wlan_ce.h must fail its test with the named reason (the model's C3
    scenarios, the salami gate suite, or this file's static checks)."""
    global DRIVER, HEADER
    killed = []
    keep = (DRIVER, HEADER)
    for name, file, old, new, how, why in MUTANTS:
        text = (ROOT / file).read_text()
        assert text.count(old) == 1, (name, text.count(old))
        mutated = text.replace(old, new)
        DRIVER, HEADER = (mutated, keep[1]) if file.endswith('.c') else (keep[0], mutated)
        out = ''
        try:
            if how == 'static':
                try:
                    c3_static_checks()
                except AssertionError as e:
                    out = 'AssertionError: ' + str(e)
            elif how == 'gate':
                spec = importlib.util.spec_from_file_location('sal_mut', ROOT / 'tools/test-fuxi-wlan-salami.py')
                sal = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(sal)
                sal.DRIVER = DRIVER
                try:
                    with tempfile.TemporaryDirectory(prefix='mut-gate-', dir=tmp) as t:
                        sal.gate_suite(t, sal.load_mkadt())
                except AssertionError as e:
                    out = 'AssertionError: ' + repr(e.args[0])[:200] if e.args else 'AssertionError'
            else:
                d = Path(tmp) / ('mut-' + re.sub(r'\W+', '-', name))
                d.mkdir()
                (d / 'fuxi-wlan-ce-prod.inc').write_text(production())
                if file.endswith('.h'):
                    (d / 'fuxi_wlan_ce.h').write_text(mutated)
                exe = d / 'model'
                subprocess.run(CC + ['-I', str(d), '-I', str(ROOT / 'xnu/fuxi-wlan'), str(MODEL), '-o', str(exe)], check=True)
                r = subprocess.run([str(exe), 'c3'], capture_output=True, text=True, timeout=900)
                out = r.stderr[-400:] if r.returncode else ''
        finally:
            DRIVER, HEADER = keep
        caught = why in out
        print(f'    mutant "{name}": ' + ('caught: ' + why if caught else 'SURVIVED or caught for another reason: ' +
                                         out.strip().splitlines()[-1][:200] if out.strip() else 'SURVIVED'), flush=True)
        if caught:
            killed.append(name)
    assert len(killed) == len(MUTANTS), f'{len(MUTANTS) - len(killed)} C3 mutants survived'
    print(f'C3 mutants: {len(killed)} of {len(MUTANTS)} caught, each for its own reason')


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
        e2e(tmp)
        if os.environ.get('FUXI_C3_MUTANTS', '1') != '0':
            mutants(tmp)
    l19_checks(dump, c2dump)
    dash_check()
    print('W20 (fuxiwlanqrtr=6) CE/HTC/WMI, W23 (fuxiwlanqrtr=7) passive scan and W25 (fuxiwlanqrtr=8) scan service '
          'host tests passed')


if __name__ == '__main__':
    main()
