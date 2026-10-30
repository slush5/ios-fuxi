#!/usr/bin/env python3
"""Build the OnePlus 11 (salami, SM8550) ADT with the Fuxi recipe.

Wraps tools/mkadt-fuxi.py unchanged: every hardware value it hardcodes was
re-checked against the live salami FDT (same RAM bank 0x8c0000000+0x140000000,
GIC 0x17100000/0x17180000, UFS host 0x1d84000, splash 0xb8000000+0x2b00000,
timer INTID 27, MPIDRs 0x0-0x700). Only its board check (mkadt-fuxi.py:46-47)
refuses salami. Differences:
  * board check: /model must contain "salami" and qcom,msm-id must list SoC 519;
    the other live-FDT checks are mkadt-fuxi.py:48-74 with the same constants;
  * identity: root compatible 'D22AP iPhone10,3 oneplus,salami xiaomi,fuxi
    qcom,sm8550 AppleARM'. xiaomi,fuxi STAYS (SweetPlatformExpert IONameMatch,
    0082-fuxi-ram-bringup.patch:70), as does qcom,sm8550-cpu on the cpus; own
    synthetic ECID and firmware-version strings;
  * framebuffer: sweet-fbcon width/height/stride from --fb (default 1080x2412,
    the fhd_sa_60 timing ABL is believed to scan out; stride 4*W unless
    --fb-stride). It must equal the loader's FB_W/FB_H/FB_STRIDE and is only
    confirmed by the first loader probe boot (SSPP0 +0x00/+0x24/+0x30);
  * --no-fbcon omits sweet-fbcon altogether: patch 0003 fbcon_probe then registers
    no framebuffer text console, so XNU draws no kernel text before the GUI (the
    log still reaches the USB monitor through sweet_log_ring). Pair it with a
    loader built with FUXI_BOOT_LOGO (shim/fuxi-loader/build.sh), which leaves the
    Apple logo on the splash (logo only); tools/run-ios-salami.py accepts a
    missing sweet-fbcon only with such a loader. The default ADT (WITH
    sweet-fbcon), such a loader and a kernel with xnu/patches/0115 make the
    verbose then logo boot instead: kernel text for sweetfb_verbose_secs, then
    the loader's Apple logo until iOS draws (logo-boot/s30_build.sh);
  * /product comes from --d22ap-adt (mkadt.build otherwise reads only its
    hardcoded research/ipsw-x path); a missing tree is refused, because without
    /product MobileGestalt loses the D22AP identity;
  * refuses an ADT above the loader's 16 KiB cap (loader.c:117), more than 512
    extents (FX_MAX_EXTENTS, sweet_ufs.c), and extents outside, at block 0 of,
    or overlapping in userdata (sda14, 57097632 4 KiB blocks: GPT 4234843..61332474,
    dm-default-key length 456781056 x 512 B, measure/dm_userdata.txt);
  * --wlan-defaults "KEY=VALUE ..." writes fuxiwlan* knobs into /defaults (K-W1b): the
    salami kernel reads every fuxiwlan* knob from the boot-args first and, only when it
    is absent there, from the /defaults property of the same name (fuxi_wlan.c wl_arg),
    so the WLAN lane fits after the cmdline is full. Numbers become 4-byte LE int32,
    strings NUL-terminated and no longer than the kernel buffer; unknown keys, other
    types and repeats are refused. Each property costs 36 bytes plus its value rounded
    up to 4 (a number: 40 bytes) inside the same 16 KiB cap; the bytes are printed;
  * --usbnet (route U, docs-lanes/wifi-ios.md) adds /chosen "fuxi-usbnet": exactly 16 bytes,
    the ASCII placeholder "FXNET-PLACEHOLDR" (52 bytes of ADT). A FUXI_USBNET loader writes
    {u64 pa, u64 size} of its usbnet ring region over it in its ADT copy, only when the
    copy holds exactly one (shim/fuxi-probe/usbnet.h); fuxiwifiios=2 reads it.

Root extents: --root-extents takes the extents.json that tools/salami-install-root.sh
writes (salami-pin output; bytes, userdata-relative). The pinned file is the image
rounded up to whole 2 MiB f2fs sections; fuxi-root-extents lists all of it and
fuxi-root-blocks is the HFS+ image size (XNU maps only those blocks). That size is
extents.json root_bytes; --root-bytes may repeat it (it must then be equal) and is
required only for an older extents.json without root_bytes. Either way it must be a
4 KiB multiple no larger than the pinned file, the rule tools/run-ios-salami.py applies.
"""
import argparse
import importlib.util
import json
import re
import struct
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import adt  # noqa: E402
import mkadt  # noqa: E402

_spec = importlib.util.spec_from_file_location('mkadt_fuxi', TOOLS / 'mkadt-fuxi.py')
fx = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fx)

SALAMI_ECID = 0x53414C414D000001        # 'SALAM' + 1, synthetic (not the phone's)
SOC_ID_SM8550 = 519                      # qcom,msm-id 0x207 (rev 2.0 = 0x20000 on this unit)
ADT_MAX = 0x4000                         # shim/fuxi-loader/loader.c:117
FX_MAX_EXTENTS = 512                     # xnu/fuxi-unixnu/sweet_ufs.c:135
USERDATA_BLOCKS = 61332474 - 4234843 + 1 # sda14, 4 KiB blocks (= 57097632)
SCANOUT_SLOT = 0xa00000                  # one GPU scanout slot (0xb8000000/0xb8a00000/0xb9400000)
COMPATIBLE = b'D22AP\0iPhone10,3\0oneplus,salami\0xiaomi,fuxi\0qcom,sm8550\0AppleARM\0'


# Touch baseline for the ADT /defaults node (touch_s3908_design.md 5.2-5.3): TCM backend
# and the salami SweetHID surface size in mm. Integer knobs are 4-byte LE u32.
TOUCH_DEFAULTS = (('fuxitic', 2), ('sweethid_mmw', 70), ('sweethid_mmh', 156))
# K-W1b: every fuxiwlan* knob of xnu/fuxi-wlan/fuxi_wlan.c, as the salami kernel reads it from
# /defaults when the boot-arg is absent (wl_arg, PE_get_default). 'i' = an int: 4-byte LE int32.
# A number = a string of at most that many characters, written NUL-terminated: the kernel buffer
# less its truncation sentinel (char[16] selectors 15, fuxiwlanseq char[96] 95, firmware
# basenames char[65] 63). PE_get_default reads a larger property as absent, so it is refused here.
WLAN_KNOBS = {
    'fuxiwlan': 'i', 'fuxiwlansalami': 'i', 'fuxiwlanrails': 'i', 'fuxiwlanqrtr': 'i',
    'fuxiwlanmlo': 'i', 'fuxiwlanw15': 'i', 'fuxiwlanw16': 'i', 'fuxiwlanw20': 'i', 'fuxiwlanw29': 'i', 'fuxiwlanvrx': 'i',
    'fuxiwlanstop22': 'i', 'fuxiwlankeep': 'i', 'fuxiwlanicc': 'i', 'fuxiwlaniccbw': 'i', 'fuxiwlanaspm': 'i',
    'fuxiwlanoldiova': 'i', 'fuxiwlanclk': 'i', 'fuxiwlaniboff': 'i',
    'fuxiwlanseq': 95, 'fuxiwlanpre': 15, 'fuxiwlanfw': 15, 'fuxiwlanbhi': 15,
    'fuxiwlanvec': 15, 'fuxiwlancnss': 15, 'fuxiwlanprobe': 15,
    'fuxiwlanbdf': 63, 'fuxiwlanregdb': 63, 'fuxiwlanm3': 63, 'fuxiwlanamss': 63,
}
WLAN_NUMBER = re.compile(r'-?(?:0[xX][0-9a-fA-F]+|0|[1-9][0-9]*)')
# Route U: the /chosen property a FUXI_USBNET loader patches (shim/fuxi-probe/usbnet.h un_adt_find).
USBNET_PROP = 'fuxi-usbnet'
USBNET_PLACEHOLDER = b'FXNET-PLACEHOLDR'
# The proven salami payload cmdline (also in tools/run-ios-salami.py): ufsrw=1 rootrw=1
# (booted iOS to the lock screen 2026-10-28), fuxikick=0. The cmdline itself is written
# by tools/mkpayload.py.
SALAMI_FIRST_BOOT_CMDLINE = (
    'debug=0xc serial=3 cpus=7 keepsyms=1 -enable_kprintf_spam -noprogress '
    'rd=ufsroot0 fuxi_ufs_probe=1 ufsroot=userdata ufsrw=1 rootrw=1 '
    'unentitled_ios_sim_launch=1 ignition_level=1 wdt_test=no_user_checkin '
    'launchd_unsecure_cache=1 wdt=-1 sweetlog=0 sweetnoexec=bluetoothd,fairplayd.H2 '
    'sweetfb_noconsole=1 fuxikick=0')


class PrintCmdline(argparse.Action):
    def __init__(self, option_strings, dest, **kw):
        super().__init__(option_strings, dest, nargs=0, **kw)

    def __call__(self, parser, namespace, values, option_string=None):
        print(SALAMI_FIRST_BOOT_CMDLINE)
        parser.exit()


def validate_salami(path, ufs_readonly=False):
    """mkadt-fuxi.validate_live() with the board check swapped for salami."""
    spec = importlib.util.spec_from_file_location('fuxi_fdt', TOOLS / 'fuxi-inspect-dtb.py')
    fdt = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fdt)
    nodes, reservations, _ = fdt.parse(path.read_bytes())
    root = nodes['/']
    if b'salami' not in root.get('model', b'').lower():
        raise ValueError('live FDT does not identify a OnePlus 11 (salami) board')
    ids = root.get('qcom,msm-id', b'')
    if not ids or len(ids) % 8 or SOC_ID_SM8550 not in [s for s, _ in struct.iter_unpack('>II', ids)]:
        raise ValueError('live FDT is not an SM8550 (soc id 519) tree')
    # From here on mkadt-fuxi.py:48-74, same constants and the same order.
    memory = nodes.get('/memory', {}).get('reg', b'')
    if len(memory) % 16:
        raise ValueError('unexpected /memory cell layout')
    ranges = list(struct.iter_unpack('>QQ', memory))
    if not any(base <= fx.PHYS_BASE and fx.PHYS_BASE + fx.BANK_SIZE <= base + size for base, size in ranges):
        raise ValueError('requested XNU RAM is outside advertised memory')
    inventory = fdt.describe(nodes, reservations)['nodes']
    for name, props in inventory.items():
        if name.startswith('/reserved-memory/'):
            for region in props.get('decoded-reg', []):
                base, size = int(region['address'], 16), int(region['size'], 16)
                if size and base < fx.PHYS_BASE + fx.BANK_SIZE and fx.PHYS_BASE < base + size:
                    raise ValueError(f'XNU RAM overlaps {name}')
    for region in reservations:
        base, size = int(region['address'], 16), int(region['size'], 16)
        if size and base < fx.PHYS_BASE + fx.BANK_SIZE and fx.PHYS_BASE < base + size:
            raise ValueError('XNU RAM overlaps FDT reserve map')
    gic = nodes.get('/soc/interrupt-controller@17100000', {})
    if gic.get('reg') != struct.pack('>4I', fx.GICD, 0x10000, fx.GICR, fx.GICR_SIZE):
        raise ValueError('GIC layout differs from the validated Fuxi map')
    if ufs_readonly:
        ufs = nodes.get('/soc/ufshc@1d84000', {})
        if ufs.get('reg', b'')[:8] != struct.pack('>II', 0x1d84000, 0x3000):
            raise ValueError('UFS host layout differs from the measured Fuxi map')
    splash = nodes.get('/reserved-memory/splash_region', {}).get('reg')
    if splash != struct.pack('>QQ', fx.FB, fx.FB_SIZE):
        raise ValueError('splash reservation differs from the measured Fuxi map')


def check_extents(words):
    """words = fuxi-root-extents u64s {file, partition, count} in 4 KiB blocks."""
    n = len(words) // 3
    if not 0 < n <= FX_MAX_EXTENTS:
        raise ValueError(f'{n} extents; the kernel takes 1..{FX_MAX_EXTENTS}')
    spans = []
    for i in range(n):
        part, count = words[3 * i + 1], words[3 * i + 2]
        if part == 0 or count == 0:
            raise ValueError(f'extent {i} is empty or starts at userdata block 0')
        if part + count > USERDATA_BLOCKS:
            raise ValueError(f'extent {i} ends past userdata (sda14, {USERDATA_BLOCKS} blocks)')
        spans.append((part, part + count, i))
    spans.sort()
    for (_, end, i), (start, _, j) in zip(spans, spans[1:]):
        if start < end:
            raise ValueError(f'extents {i} and {j} overlap in userdata')
    return n


def root_size(path, root_bytes):
    """HFS+ image size for fuxi-root-blocks: extents.json root_bytes and/or --root-bytes."""
    table = json.loads(path.read_text())
    size, recorded = table.get('size'), table.get('root_bytes')
    if recorded is not None and root_bytes is not None and recorded != root_bytes:
        raise ValueError(f'--root-bytes {root_bytes} differs from root_bytes {recorded} in {path}')
    rb = recorded if recorded is not None else root_bytes
    if rb is None:
        raise ValueError(f'{path} records no root_bytes: give --root-bytes (the HFS+ image size)')
    if type(rb) is not int or type(size) is not int or not 0 < rb <= size or rb % 4096:
        raise ValueError(f'root size {rb!r} must be a 4 KiB multiple inside the {size!r}-byte pinned file')
    return rb


def adt_prop_bytes(value):
    """Bytes one property adds to the serialized ADT (adt.serialize): 32-byte name, u32 size,
    the value padded to 4."""
    return 32 + 4 + len(value) + (-len(value) & 3)


def wlan_defaults(texts):
    """--wlan-defaults: whitespace-separated KEY=VALUE tokens (cmdline syntax, the option may repeat)
    -> [(knob, encoded value)] in the given order, typed by WLAN_KNOBS."""
    props, seen = [], set()
    for token in ' '.join(texts).split():
        key, sep, value = token.partition('=')
        if not sep:
            raise ValueError(f'--wlan-defaults wants KEY=VALUE tokens, got {token!r}')
        kind = WLAN_KNOBS.get(key)
        if kind is None:
            raise ValueError(f'--wlan-defaults: {key!r} is not a fuxiwlan* knob of fuxi_wlan.c '
                             f'({", ".join(sorted(WLAN_KNOBS))})')
        if key in seen:
            raise ValueError(f'--wlan-defaults: {key} given twice')
        seen.add(key)
        if kind == 'i':
            if not WLAN_NUMBER.fullmatch(value):
                raise ValueError(f'--wlan-defaults: {key}={value!r} must be a decimal or 0x number')
            number = int(value, 0)
            if not -2 ** 31 <= number < 2 ** 31:
                raise ValueError(f'--wlan-defaults: {key}={value} does not fit the kernel int')
            props.append((key, struct.pack('<i', number)))
        else:
            if not 0 < len(value) <= kind or not all(32 < ord(c) < 127 for c in value):
                raise ValueError(f'--wlan-defaults: {key}={value!r} must be 1..{kind} printable characters '
                                 'without spaces (the kernel buffer)')
            props.append((key, value.encode('ascii') + b'\0'))
    if not props:
        raise ValueError('--wlan-defaults is empty')
    return props


def add_wlan_defaults(root, props):
    """Append the knobs to /defaults; returns [(knob, bytes added)]. A knob already there is refused."""
    defaults = next((n for n in root.children if n.name == 'defaults'), None)
    if defaults is None:
        raise ValueError('no /defaults node to carry the fuxiwlan* knobs')
    added = []
    for name, value in props:
        if defaults.prop(name) is not None:
            raise ValueError(f'/defaults already has {name}')
        defaults.props.append((name, 0, value))
        added.append((name, adt_prop_bytes(value)))
    return added


def add_usbnet(root):
    """--usbnet: /chosen "fuxi-usbnet" = the 16-byte placeholder; returns the bytes added."""
    chosen = next((n for n in root.children if n.name == 'chosen'), None)
    if chosen is None:
        raise ValueError('no /chosen node to carry fuxi-usbnet')
    if chosen.prop(USBNET_PROP) is not None:
        raise ValueError(f'/chosen already has {USBNET_PROP}')
    assert len(USBNET_PLACEHOLDER) == 16
    chosen.props.append((USBNET_PROP, 0, USBNET_PLACEHOLDER))
    return adt_prop_bytes(USBNET_PLACEHOLDER)


def parse_fb(text, stride_text):
    try:
        w, h = (int(v) for v in text.lower().split('x'))
        stride = int(stride_text, 0) if stride_text else w * 4
    except ValueError:
        raise ValueError(f'--fb wants WxH (e.g. 1080x2412) and --fb-stride an integer, got {text!r}') from None
    if w <= 0 or h <= 0 or stride < w * 4 or stride % 4:
        raise ValueError(f'bad framebuffer geometry {w}x{h} stride {stride}')
    if stride * h > SCANOUT_SLOT:
        raise ValueError(f'{w}x{h} stride {stride} = {stride * h:#x} bytes does not fit one '
                         f'{SCANOUT_SLOT:#x} scanout slot of the {fx.FB_SIZE:#x} splash')
    return w, h, stride


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', type=Path)
    ap.add_argument('--live-fdt', type=Path, required=True, help='salami live FDT (salami-live.dtb)')
    ap.add_argument('--ufs-readonly', action='store_true', help='include the UFS node without a root table (read-only probe)')
    ap.add_argument('--root-extents', type=Path, help='extents.json from tools/salami-install-root.sh; needs --root-bytes')
    ap.add_argument('--root-bytes', type=int, help='HFS+ root image size in bytes (4 KiB aligned; '
                    'default and must-equal: extents.json root_bytes)')
    ap.add_argument('--smp', action='store_true', help='add cores 2-7 (cpu@100 stays with the USB monitor)')
    ap.add_argument('--gpu-reserve', action='store_true', help='declare the top-of-bank 32 MiB GPU scratch')
    ap.add_argument('--ram-layout', choices=('full', '1g'), default='full', help='full = 5088 MiB; 1g = 1 GiB XNU window')
    ap.add_argument('--product-haptics', action='store_true', help='add /product/haptics copied from the D22AP tree')
    ap.add_argument('--product-camera', action='store_true', help='add an empty /product/camera node (no flash)')
    ap.add_argument('--display-gamut', choices=('P3', 'sRGB'), help='set /product artwork-display-gamut')
    ap.add_argument('--wlan-node', action='store_true', help='add an empty /arm-io/wlan node so launchd starts wifid (its job is limited to the wlan / marconi-wifi device-tree nodes); pairs with fuxiwifiios=1')
    ap.add_argument('--d22ap-adt', type=Path, default=Path(mkadt.D22AP_ADT), help='real D22AP device tree (not in git)')
    ap.add_argument('--fb', help='ABL splash WxH (default 1080x2412); must match the loader FB_W/FB_H')
    ap.add_argument('--fb-stride', help='bytes per line (default 4*W); must match the loader FB_STRIDE')
    ap.add_argument('--no-fbcon', action='store_true',
                    help='omit the sweet-fbcon node: no XNU framebuffer text console (logo-only boot with a '
                         'boot-logo loader; keep the node for verbose then logo)')
    ap.add_argument('--touch-defaults', action='store_true',
                    help='write the S3908 touch baseline into /defaults (fuxitic=2, sweethid_mmw/h)')
    ap.add_argument('--wlan-defaults', action='append', metavar='"KEY=VALUE ..."',
                    help='write these fuxiwlan* knobs into /defaults (may repeat); the salami kernel reads '
                         'each from the boot-args first, else from here. Numbers 4-byte LE, strings '
                         'NUL-terminated; each property costs 36 bytes + its value rounded up to 4')
    ap.add_argument('--usbnet', action='store_true',
                    help='add /chosen fuxi-usbnet, a 16-byte placeholder a FUXI_USBNET loader overwrites with '
                         'its usbnet ring region {pa, size} (route U); pairs with fuxiwifiios=2')
    ap.add_argument('--print-cmdline', action=PrintCmdline,
                    help='print the recommended salami first-boot payload cmdline and exit')
    args = ap.parse_args()
    mem_size = fx.MEM_SIZE if args.ram_layout == 'full' else fx.LEGACY_SIZE
    try:
        if args.no_fbcon and (args.fb or args.fb_stride):
            raise ValueError('--no-fbcon writes no sweet-fbcon node: --fb/--fb-stride have nothing to set')
        w, h, stride = parse_fb(args.fb or '1080x2412', args.fb_stride)
        mkadt.FB_W, mkadt.FB_H, mkadt.FB_STRIDE = w, h, stride
        product_path = str(args.d22ap_adt)
        if not args.d22ap_adt.is_file():
            raise ValueError(f'{product_path}: D22AP device tree missing (no /product, MobileGestalt breaks)')
        orig_product = mkadt.d22ap_product
        mkadt.d22ap_product = lambda path=product_path: orig_product(path)
        root_props, n_extents = (), 0
        if args.root_extents:
            root_bytes = root_size(args.root_extents, args.root_bytes)
            root_props = fx.root_extent_props(args.root_extents, root_bytes)
            words = struct.unpack(f'<{len(root_props[0][1]) // 8}Q', root_props[0][1])
            n_extents = check_extents(words)
        elif args.root_bytes:
            raise ValueError('--root-bytes without --root-extents')
        root = fx.build(args.ufs_readonly or bool(args.root_extents), root_props, args.smp, args.gpu_reserve, mem_size)
        if args.no_fbcon:
            # fbcon_probe (patch 0003) returns before register_serial_functions when
            # SecureDTFindEntry("name", "sweet-fbcon") fails: no kernel text on the panel.
            if sum(n.name == 'sweet-fbcon' for n in root.children) != 1:
                raise ValueError('expected exactly one sweet-fbcon node to omit')
            root.children = [n for n in root.children if n.name != 'sweet-fbcon']
        fx.setprop(root, 'compatible', COMPATIBLE)
        chosen = next(n for n in root.children if n.name == 'chosen')
        fx.setprop(chosen, 'firmware-version', b'salami-shim\0')
        fx.setprop(chosen, 'system-firmware-version', b'salami-shim\0')
        fx.setprop(chosen, 'unique-chip-id', struct.pack('<Q', SALAMI_ECID))
        if not any(n.name == 'product' for n in root.children):
            raise ValueError(f'{product_path}: no /product node was built')
        root = fx.add_product_children(root, args.product_haptics, args.product_camera, product_path)
        if args.display_gamut:
            root = fx.set_display_gamut(root, args.display_gamut)
        if args.wlan_node:
            arm_io = next(n for n in root.children if n.name == 'arm-io')
            if any(n.name == 'wlan' for n in arm_io.children):
                raise ValueError('/arm-io/wlan already exists')
            arm_io.children.append(mkadt.node('wlan', [
                ('AAPL,phandle', struct.pack('<I', fx.max_phandle(root) + 1)),
                ('device_type', mkadt.cstr('wlan'))]))
        if args.touch_defaults:
            defaults = next((n for n in root.children if n.name == 'defaults'), None)
            if defaults is None:
                raise ValueError('no /defaults node to carry the touch baseline')
            for name, value in TOUCH_DEFAULTS:
                defaults.props = [(k, fl, v) for k, fl, v in defaults.props if k != name]
                defaults.props.append((name, 0, struct.pack('<I', value)))
        wlan_added = add_wlan_defaults(root, wlan_defaults(args.wlan_defaults)) if args.wlan_defaults else []
        wlan_bytes = sum(n for _, n in wlan_added)
        usbnet_bytes = add_usbnet(root) if args.usbnet else 0
        has_ufs = any(n.name == 'fuxi-ufs' for n in root.children)
        validate_salami(args.live_fdt, has_ufs)
        blob = adt.serialize(root)
        if len(blob) > ADT_MAX:
            raise ValueError(f'ADT is {len(blob)} bytes; the loader accepts at most {ADT_MAX} '
                             f'({n_extents} extents at 24 bytes each'
                             + (f'; --wlan-defaults {wlan_bytes} of them' if wlan_added else '') + ')')
    except (ValueError, KeyError, StopIteration, struct.error, OSError) as error:
        ap.exit(1, f'{ap.prog}: {error}\n')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(blob)
    cpus = len(next(n for n in root.children if n.name == 'cpus').children)
    product = next(n for n in root.children if n.name == 'product')
    extra = sorted(c.name for c in product.children)
    print(f'{args.out}: {len(blob)} bytes (cap {ADT_MAX}, {ADT_MAX - len(blob)} free); '
          f'RAM {fx.PHYS_BASE:#x}+{mem_size:#x}; CPUs {cpus}; UFS={"present" if has_ufs else "absent"}; '
          f'root extents {n_extents}; '
          + ('no sweet-fbcon (no XNU text console)' if args.no_fbcon else f'fb {w}x{h} stride {stride}')
          + (f'; /product children {",".join(extra)}' if extra else '')
          + ('; touch defaults' if args.touch_defaults else '')
          + (f'; /chosen fuxi-usbnet placeholder {usbnet_bytes} bytes' if usbnet_bytes else '')
          + (f'; /defaults wlan knobs {wlan_bytes} bytes (' + ', '.join(f'{k} {n}' for k, n in wlan_added) + ')'
             if wlan_added else ''))


if __name__ == '__main__':
    main()
