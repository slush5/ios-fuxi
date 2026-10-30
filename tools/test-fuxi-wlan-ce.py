#!/usr/bin/env python3
"""W20 (fuxiwlanqrtr=6, C1): copy engines, HTC and WMI init to WMI READY, host-tested.

Builds tools/fuxi-wlan-ce-model.c with ASan/UBSan and -Werror against the pure
xnu/fuxi-wlan/fuxi_wlan_ce.h and the production glue cut out of
xnu/fuxi-wlan/fuxi_wlan.c (written to fuxi-wlan-ce-prod.inc in a temporary
directory), then runs every model scenario (docs-lanes/wlan-w20.md). Static
checks: the W20 call is gated, containment runs before the W11 stop, the arena
is freed only on the proven path, the W20 block writes only through its two
writers. When the Linux ath12k v6.19 tree is on disk ($ATH12K_SRC or
~/op11-ios/src/linux-ath12k/drivers/net/wireless/ath/ath12k) the WMI ids and
tags, the wire struct sizes, all 73 INIT resource-config words, the HTC
constants and the CE ring geometry, ids and register offsets are derived from
it again and compared with the header; without the tree that part is skipped
with a message. No device, no network.
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
    assert DRIVER.count('wl_w20_run()') == 1 and DRIVER.count('wl_w20 = qrtr == 6;') == 1
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
                                                  'WL20_ACTIVE 0x08u', 'WL20_QRTR_MAX 6'))
    assert '#define WL20_BITS (WL20_NOHTT | WL20_MRRS128 | WL20_NOHWMODE)' in DRIVER
    assert 'python3 tools/test-fuxi-wlan-ce.py' in (ROOT / 'tools/ci/build.sh').read_text()
    print('W20 static: gated call, containment before the W11 stop, arena in the owned/free lists, '
          'only the window/shadow/devctl writers, pure header with the Clear BSD text')


def header_define(name):
    m = re.search(r'^#define ' + name + r' (.+?)(?:\s*/\*.*)?$', HEADER, re.M)
    assert m, name
    return eval(re.sub(r'\b(0x[0-9a-fA-F]+|\d+)u\b', r'\1', m[1]), {}, {})


def l19_text(name):
    return re.sub(r'//[^\n]*', '', re.sub(r'/\*.*?\*/', '', (L19 / name).read_text(), flags=re.S))


def l19_checks(model):
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
        return eval(expr, {}, vals)

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
    for k in ('HW_BD_INFO_SIZE', 'WMI_SERVICE_SEGMENT_BM_SIZE32'):
        consts[k] = int(re.search(r'#define ' + k + r'\s+(\d+)', wmi_h)[1])

    def struct_fields(name):
        body = re.search(r'struct ' + name + r' \{(.*?)\} __packed;', wmi_h, re.S)[1]
        out = []
        for decl in body.split(';'):
            decl = ' '.join(decl.split())
            if not decl:
                continue
            m = re.fullmatch(r'(?:__le32|u8|struct (\w+)) (\w+)(?:\[(\w+)\])?', decl)
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
    l19_checks(dump)
    dash_check()
    print('W20 (fuxiwlanqrtr=6) CE/HTC/WMI host tests passed')


if __name__ == '__main__':
    main()
