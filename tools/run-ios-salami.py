#!/usr/bin/env python3
"""OnePlus 11 (salami) pre-boot runner, modelled on tools/run-ios-thermal.py.

From Android with Magisk (salami has no OrangeFox), checks that the pinned root
is still exactly where the payload's ADT says, then reboots to the bootloader,
fastboot-boots the loader image and (unless --no-monitor) runs
tools/fuxi-xnu-monitor.py. The serial comes from --serial or SALAMI_SERIAL (kept
out of git). Run from anywhere; monitor arguments that this runner does not know
are passed through.

  python3 tools/run-ios-salami.py OUT --boot-img IMG --kernel K --payload P \\
      [--extents research/salami/pinfile-01/extents.json] [monitor args]
  python3 tools/run-ios-salami.py OUT --boot-img IMG --kernel K --payload P --loader-only
      Android -> loader only (no root checks); the monitor gets --no-handoff.
  --check-only    run every check, do not reboot.

Checks (all fail closed, before the phone leaves Android):
  * --boot-img is what fastboot will boot: an Android boot image (header v4, page
    4096) whose kernel Image is a FUXI_SALAMI loader (it carries the salami
    splash-overlap guard, shim/fuxi-probe/probe.c) and ends with exactly the
    --payload bytes (shim/fuxi-loader/build.sh incbins the payload last), so the
    checks below are about the payload that really boots;
  * adb get-state is 'device' (Android), not recovery/sideload/bootloader;
  * ro.product.device == OP594DL1 and Magisk su gives uid 0;
  * by-name/userdata is sda14 and `dmctl table userdata` is ONE default-key
    target over sda14 at offset 0, so FIEMAP physical offsets are sda14 offsets;
  * `salami-pin verify` (through su) finds the file pinned, uncompressed, and its
    extents, size and digest equal extents.json. Android runs between sessions:
    f2fs GC clears a pin after gc_pin_file_thresh failures and may then move the
    file, and XNU writes would land on live Android data (the Magisk boot check
    from tools/salami-install-root.sh raises the threshold at every boot);
  * the payload kernel is a salami build (FUXI_SALAMI volume map, 0114) and its
    ADT is a salami ADT (oneplus,salami) whose /chosen fuxi-root-extents equal
    extents.json and whose fuxi-root-blocks equal the recorded root_bytes (or fit
    the pinned file for an older extents.json); rd=ufsroot0 on userdata (the
    kernel's default ufsroot) without fuxi-root-extents is refused;
  * the ADT sweet-fbcon geometry is the loader's 1080x2412 stride 4320, unless
    --fb-geometry WxH:S names another one measured by a probe boot. A boot-logo
    loader (shim/fuxi-loader/build.sh FUXI_BOOT_LOGO; --logo requires one) is
    accepted with either ADT:
      - without sweet-fbcon (tools/mkadt-salami.py --no-fbcon, no XNU text
        console): logo only. Such an ADT is accepted ONLY with a boot-logo
        loader, whose marker then stands in for the geometry check;
      - with sweet-fbcon: verbose then logo. A kernel with xnu/patches/0115
        (string sweetlogo_pa) draws kernel text for sweetfb_verbose_secs, then
        the loader's Apple logo until iOS draws. A kernel without 0115 only
        gets a warning: its text covers the loader logo for the whole boot;
  * the payload cmdline has no fuxipanel120=1 (Xiaomi M3 panel DCS sequence), and
    ufsrw / rootrw with any nonzero, bare or unparsable value (PE_parse_boot_argn
    semantics) need --allow-rw (only after a read-only boot logged
    'FUXI UFS: root file in userdata: N extents' with the expected N).
Verbose then logo boot-args (never refused; logo-boot/s30_build.sh builds this boot):
  sweetfb_verbose_secs=N  0115 kernels: seconds of sweet-fbcon kernel text before
      the kernel stops the text and draws the boot-logo loader's Apple logo, which
      then stays until iOS (backboardd) draws its own at the first swaps. s30 uses
      30; without it the kernel's own default applies. It times the text console,
      so it only matters with a sweet-fbcon ADT and a boot-logo loader; otherwise,
      or when PE_parse_boot_argn would not read it as an integer, a warning says so.
  sweetlogo_pa=0x<PA> sweetlogo_w=W sweetlogo_h=H  where the loader's logo mask
      lives (the loader image stays in DRAM, outside XNU RAM, because the cpu@100
      USB monitor runs from it). The boot-logo loader appends them at the handoff
      (shim/fuxi-loader/loader.c boot_logo_args, at most 66 characters), so the
      payload normally has none. It appends nothing when the payload cmdline
      already contains sweetlogo_ or the line would pass 607 characters (a
      warning when the payload cmdline leaves less than 66); monitor request 0x48
      then says why. Payload sweetlogo_* are accepted with a note: XNU uses those
      payload values (PE_parse_boot_argn takes the first occurrence).
Touch: unlike run-ios-thermal.py this never passes --touch-power, and any --touch*
monitor argument (--touch-power, --touch-test, --touch-stage, abbreviations) is
refused. The Fuxi loader touch_power sends Fuxi's literal VRM addresses; on salami
0x40900 is ldob10 and 0x41b00 is ldon7 (pm8010n_l7, camera PMIC, max 2960 mV, so
its 3200 mV vote would over-volt it). Never send them; only a salami touch_power
that resolves ldob4 (0x40800) by name may change that.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import time

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import adt  # noqa: E402

DEVICE = 'OP594DL1'
PART = 'sda14'
PART_SECTORS = 456781056          # 57097632 x 4 KiB (GPT 4234843..61332474)
HELPER = '/data/local/tmp/salami-pin'
STATUS = '/data/local/tmp/salami-root-status.txt'   # tools/salami-magisk-service.sh
GC_PIN_THRESH = 65535             # what the Magisk boot check raises gc_pin_file_thresh to
BLK = 4096
BOOT_PAGE = 4096                  # Android boot image header v3/v4: fixed 4 KiB pages
SALAMI_COMPATIBLE = b'oneplus,salami'
LOADER_MARKER = b'overlaps ABL splash'            # probe.c probe_splash_overlap(), FUXI_SALAMI only
LOGO_MARKER = b'FUXI BOOT LOGO:'                  # loader.c boot_logo_marker, FUXI_BOOT_LOGO builds only
LOGO_HANDOVER = b'sweetlogo_'                     # a boot-logo loader that appends sweetlogo_* to the cmdline
# Longest loader append (boot_logo_args.h): ' sweetlogo_pa=0x' + 16 hex digits + ' sweetlogo_w='
# + 4 digits (W <= 1080) + ' sweetlogo_h=' + 4 digits (H <= 2412). It must fit in 607 characters.
LOGO_APPEND_MAX = 16 + 16 + 13 + 4 + 13 + 4
BOOT_LINE_CHARS = 607                             # BOOT_LINE_LENGTH 608 minus the NUL
VERBOSE_LOGO_KERNEL = b'sweetlogo_pa'             # xnu/patches/0115 (verbose then logo) boot-arg name
VERBOSE_SECS = 'sweetfb_verbose_secs'             # 0115: seconds of kernel text before the logo
KERNEL_MARKER = b'FUXI KEYS: salami volume map'   # xnu/fuxi-h1/fuxi_keys.c, FUXI_SALAMI only
LOADER_FB = (1080, 2412, 4320)    # gpu-handoff.h FG_FB_W/FG_FB_H/FG_FB_STRIDE under FUXI_SALAMI
DEFAULT_EXTENTS = TOOLS.parent / 'research/salami/pinfile-01/extents.json'
MONITOR = TOOLS / 'fuxi-xnu-monitor.py'
FULL_BOOT_ARGS = ['--seconds', '14400', '--framebuffer', '--reset-minutes', '60']
# GPU scanout family refused on salami unless fuxiscansalami=1 (VIG0-only flips, INTF1
# ownership unproven). fuxiscan* (except fuxiscansalami) is caught by prefix below.
SCANOUT_ARGS = {'fuxigpuscanout', 'fuxivsyncte', 'fuxigpuncimport'}
# Proven salami boot (iOS 16.7.16 to the lock screen, 2026-10-28): ufsrw=1 rootrw=1 as
# Sweet/Fuxi boot (XNU initialises the NeedInit HFS journal; launchd needs a writable
# /private/var), fuxikick=0 (no MDSS access). Under the 608-byte payload limit
# (tools/mkpayload.py). Pass --allow-rw for it.
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


def fail(message):
    raise SystemExit(f'run-ios-salami: {message}')


def fnv(size, extents):
    h = 0xcbf29ce484222325
    for v in [size] + [x for e in extents for x in e]:
        for b in struct.pack('<Q', v):
            h = ((h ^ b) * 0x100000001b3) & 0xffffffffffffffff
    return f'{h:016x}'


def fb_geometry(text):
    m = re.fullmatch(r'(\d+)x(\d+):(\d+)', text)
    if not m:
        raise argparse.ArgumentTypeError(f'{text!r}: want WxH:S, e.g. 1080x2412:4320')
    w, h, s = (int(v) for v in m.groups())
    if not (w and h) or s < 4 * w or s % 4:
        raise argparse.ArgumentTypeError(f'{text!r}: stride must be a multiple of 4 and at least 4*W')
    return w, h, s


class Phone:
    def __init__(self, serial, adb='adb'):
        self.adb = [adb, '-s', serial]

    def run(self, args, timeout=120):
        p = subprocess.run(self.adb + args, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout.replace('\r', ''), p.stderr.replace('\r', '')

    def shell(self, command):
        code, out, err = self.run(['shell', command])
        if code:
            fail(f'adb shell {command!r} failed ({code}): {err.strip() or out.strip()}')
        return out.strip()

    def su(self, command, check=True):
        if "'" in command:
            fail('internal: quote in su command')
        code, out, err = self.run(['shell', f"su -c '{command}'"])
        if check and code:
            fail(f'su -c {command!r} failed ({code}): {err.strip() or out.strip()}')
        return code, out.strip(), err.strip()


def is_display_poll(name):
    return name.startswith('--display') and '--display-poll'.startswith(name)


def split_monitor_args(extra):
    i_know = any(a.split('=', 1)[0] == '--i-know-s5-s6' for a in extra)
    poll = False
    for a in extra:
        name = a.split('=', 1)[0]
        if name.startswith('--touc'):
            fail(f'{a}: touch power/test arguments are refused on salami (the Fuxi touch_power '
                 f'VRM addresses are ldob10 and ldon7 (camera PMIC) here)')
        if name == '--no-handoff':
            fail('--no-handoff: use --loader-only')
        if is_display_poll(name):
            poll = True
    if poll and not i_know:
        fail('--display-poll samples INTF1 (+0x2b4/+0x2b0) and DSI0, unproven on salami until '
             'probe stages S5 (INTF) and S6 (DSI) pass; pass --i-know-s5-s6 to override')
    return extra


def load_extents(path):
    t = json.loads(path.read_text())
    size, ext = t['size'], t['extents']
    pos = 0
    for i, (lo, ph, ln) in enumerate(ext):
        if lo != pos or min(lo, ph, ln) < 0 or ph == 0 or ln == 0 or (lo | ph | ln) % BLK:
            fail(f'{path}: extent {i} is not a contiguous 4 KiB extent')
        if ph + ln > PART_SECTORS * 512:
            fail(f'{path}: extent {i} ends past {PART}')
        pos += ln
    if pos != size:
        fail(f'{path}: extents cover {pos} of {size} bytes')
    if not re.fullmatch(r'/data/[A-Za-z0-9_][A-Za-z0-9._-]*', str(t.get('file', ''))):
        fail(f'{path}: file {t.get("file")!r} is not a plain /data/NAME path')
    if t.get('digest') and t['digest'] != fnv(size, ext):
        fail(f'{path}: digest does not match its extents')
    # root_bytes (salami-install-root.sh): the HFS+ image size inside the 2 MiB-rounded pinned file.
    rb = t.get('root_bytes')
    if rb is not None and (type(rb) is not int or not 0 < rb <= size or rb % BLK):
        fail(f'{path}: root_bytes {rb!r} is not a 4 KiB multiple inside the {size}-byte pinned file')
    return t


def boot_arg_on(value):
    """PE_parse_boot_argn into an integer: a bare name is 1; only a plain zero is off.

    Anything else (nonzero, octal/hex/binary forms, empty, junk that XNU copies
    as a string) counts as on, so the guard errs towards refusing."""
    if not value:
        return True
    return not re.fullmatch(r'-?(0+|0[xX]0+|0[bB]0+)', value)


def boot_arg_int(value):
    """PE_parse_boot_argn into an integer, or None when XNU would copy it as a string.

    A bare name is 1; a leading 0 is octal, 0x hex, 0b binary; a minus is allowed."""
    if value is None or value == '':
        return 1
    m = re.fullmatch(r'(-?)(0[xX][0-9a-fA-F]+|0[bB][01]+|0[0-7]*|[1-9][0-9]*)', value)
    if not m:
        return None
    sign, body = m.groups()
    if body[:2] in ('0x', '0X'):
        n = int(body, 16)
    elif body[:2] in ('0b', '0B'):
        n = int(body, 2)
    elif len(body) > 1 and body[0] == '0':
        n = int(body, 8)
    else:
        n = int(body)
    return -n if sign else n


def check_cmdline(cmdline, allow_rw):
    """Returns the cmdline as [(name, value or None)], after the display/rw guards."""
    args = [(n, v if sep else None) for n, sep, v in (w.partition('=') for w in cmdline.split())]
    if any(n == 'fuxipanel120' and boot_arg_on(v) for n, v in args):
        fail('payload cmdline has fuxipanel120 (Xiaomi panel DCS); never on salami')
    salami_scan = any(n == 'fuxiscansalami' and boot_arg_on(v) for n, v in args)
    scan = sorted({n for n, v in args if boot_arg_on(v)
                   and (n in SCANOUT_ARGS or (n.startswith('fuxiscan') and n != 'fuxiscansalami'))})
    if scan and not salami_scan:
        fail(f'payload cmdline has {" ".join(scan)} (GPU scanout); on salami these flip VIG0 '
             f'only and take unproven INTF1 ownership, so add fuxiscansalami=1 to accept them')
    kicks = [v for n, v in args if n == 'fuxikick']
    if any(boot_arg_int(v) == 2 for v in kicks):
        fail('payload cmdline has fuxikick=2 (INTF1 read every kick); never on salami '
             '(use 0 for no MDSS access, or 1/3 after probe stage S5)')
    if not kicks:
        print('warning: payload cmdline has no fuxikick; the salami kernel default 0 (no MDSS '
              'access) applies. Add fuxikick=0 to document it.', flush=True)
    if any(n == 'rootrw' and boot_arg_on(v) for n, v in args) and not allow_rw:
        fail('payload cmdline has rootrw (root mounted read-write); pass --allow-rw (the proven '
             'salami boot uses ufsrw=1 rootrw=1, writes bounded to the pinned extents)')
    rw = [n if v is None else f'{n}={v}' for n, v in args if n == 'ufsrw' and boot_arg_on(v)]
    if rw and not allow_rw:
        fail(f'payload cmdline has {" ".join(rw)} (read-write); pass --allow-rw only after a '
             f'read-only boot logged the expected extent count')
    # Verbose then logo (0115): accepted, only checked for readability.
    secs = [v for n, v in args if n == VERBOSE_SECS]
    if secs and (boot_arg_int(secs[0]) is None or boot_arg_int(secs[0]) < 0):
        print(f'warning: {VERBOSE_SECS}={secs[0]} is not a non-negative integer as PE_parse_boot_argn '
              f'reads it, so the kernel gets no usable text time', flush=True)
    if len(secs) > 1:
        print(f'note: the payload cmdline has {VERBOSE_SECS} {len(secs)} times; XNU takes the first', flush=True)
    if 'sweetlogo_' in cmdline:
        named = sorted({n for n, _ in args if n.startswith('sweetlogo_')})
        print(f'note: the payload cmdline already contains sweetlogo_ ({" ".join(named) or "inside a value"}), '
              f'so the boot-logo loader appends nothing and '
              + ('XNU uses the payload values (PE_parse_boot_argn takes the first occurrence)' if named else
                 'XNU gets no sweetlogo_* at all'), flush=True)
    return args


def userdata_ufsroot(args):
    """rd=ufsroot0 (or rootdev=) with ufsroot on userdata, the sweet_ufs.c default."""
    rd = any(n in ('rd', 'rootdev') and 'ufsroot' in (v or '') for n, v in args)
    roots = [v for n, v in args if n == 'ufsroot']
    return rd and (not roots or any(v in (None, '', 'userdata') for v in roots))


def payload_kernel_has(b, needle):
    """needle in a SWEETXNU payload outside its cmdline and ADT, so in the kernel itself
    (a cmdline that names a boot-arg must not pass for a kernel that parses it)."""
    masked = bytearray(b)
    masked[0x48:0x48 + 608] = bytes(len(masked[0x48:0x48 + 608]))
    if len(b) >= 0x30:
        adt_off, adt_size = struct.unpack_from('<2Q', b, 0x20)
        if adt_off + adt_size <= len(b):
            masked[adt_off:adt_off + adt_size] = bytes(adt_size)
    return needle in masked


def adt_has_fbcon(b):
    """True when the payload ADT has a sweet-fbcon node (False when it cannot be parsed)."""
    try:
        adt_off, adt_size = struct.unpack_from('<2Q', b, 0x20)
        if not 0 < adt_size <= 0x4000 or adt_off + adt_size > len(b):
            return False
        root, _ = adt.parse(b[adt_off:adt_off + adt_size])
        return any(c.name == 'sweet-fbcon' for c in root.children)
    except (struct.error, AssertionError, IndexError, ValueError, RecursionError):
        return False


def check_payload(path, b, table, loader_only, allow_rw, want_fb, logo_loader=False, fb_explicit=False):
    """logo_loader: the boot image's loader carries LOGO_MARKER (check_boot_img).
    fb_explicit: want_fb came from --fb-geometry, not the LOADER_FB default."""
    if b[:8] != b'SWEETXNU' or len(b) < 0x48 + 608:
        fail(f'{path}: not a SWEETXNU payload')
    if KERNEL_MARKER not in b:
        fail(f'{path}: payload kernel is not a salami build (no FUXI_SALAMI volume map, '
             f'xnu/patches/0114-salami-board.patch)')
    adt_off, adt_size = struct.unpack_from('<2Q', b, 0x20)
    cmdline = b[0x48:0x48 + 608].split(b'\0')[0].decode(errors='replace')
    args = check_cmdline(cmdline, allow_rw)
    secs = [v for n, v in args if n == VERBOSE_SECS]
    if not 0 < adt_size <= 0x4000 or adt_off + adt_size > len(b):
        fail(f'{path}: ADT {adt_size} bytes at {adt_off:#x} is not a loader-acceptable ADT')
    root, _ = adt.parse(b[adt_off:adt_off + adt_size])
    if SALAMI_COMPATIBLE not in (root.prop('compatible') or b'').split(b'\0'):
        fail(f'{path}: ADT is not a salami ADT (build it with tools/mkadt-salami.py)')
    chosen = next(c for c in root.children if c.name == 'chosen')
    fbcons = [c for c in root.children if c.name == 'sweet-fbcon']
    if len(fbcons) > 1:
        fail(f'{path}: ADT has {len(fbcons)} sweet-fbcon nodes')
    if not fbcons:
        # No XNU text console (mkadt-salami.py --no-fbcon). There is no ADT geometry to
        # compare: the loader's boot-logo marker is the check instead.
        if not logo_loader:
            fail(f'{path}: ADT has no sweet-fbcon node; that is only accepted with a boot-logo '
                 f'loader (shim/fuxi-loader/build.sh FUXI_BOOT_LOGO), which the boot image is not')
        if fb_explicit:
            fail('--fb-geometry checks the ADT sweet-fbcon node, and this payload ADT has none')
        fb_text = 'no sweet-fbcon (no XNU text console; the loader draws the boot logo)'
        if secs:
            print(f'warning: {VERBOSE_SECS} times the XNU text console, and this ADT has no sweet-fbcon '
                  f'(mkadt-salami.py --no-fbcon), so it has nothing to time', flush=True)
    else:
        try:
            fb = tuple(struct.unpack('<I', fbcons[0].prop(k))[0] for k in ('width', 'height', 'stride'))
        except (TypeError, struct.error):
            fail(f'{path}: ADT sweet-fbcon width/height/stride are not u32 properties')
        if fb != want_fb:
            fail(f'payload ADT sweet-fbcon {fb[0]}x{fb[1]} stride {fb[2]} differs from the loader '
                 f'{want_fb[0]}x{want_fb[1]} stride {want_fb[2]} (mkadt-salami.py --fb/--fb-stride; '
                 f'--fb-geometry only for a geometry a probe boot measured)')
        fb_text = f'fb {fb[0]}x{fb[1]} stride {fb[2]}'
        if logo_loader and payload_kernel_has(b, VERBOSE_LOGO_KERNEL):
            n = boot_arg_int(secs[0]) if secs else None
            when = (f'the kernel default time (no {VERBOSE_SECS})' if not secs else
                    f'{n} s ({VERBOSE_SECS})' if n is not None and n >= 0 else
                    f'{VERBOSE_SECS}={secs[0]} (unreadable, see the warning)')
            fb_text += f'; verbose then logo (0115): kernel text for {when}, then the loader Apple logo'
        elif logo_loader:
            print(f'warning: the ADT has sweet-fbcon but the payload kernel has no xnu/patches/0115 '
                  f'({VERBOSE_LOGO_KERNEL.decode()}): its text console covers the loader boot logo until '
                  f'the GUI. Use a 0115 kernel for verbose then logo, or mkadt-salami.py --no-fbcon for '
                  f'logo only', flush=True)
        elif secs:
            print(f'warning: {VERBOSE_SECS} is set but the loader is not a boot-logo loader '
                  f'(FUXI_BOOT_LOGO): there is no loader logo for the kernel to draw after the text', flush=True)
    ext, blocks = chosen.prop('fuxi-root-extents'), chosen.prop('fuxi-root-blocks')
    print(f'payload: {path.name}, salami kernel, ADT {adt_size} bytes, {fb_text}', flush=True)
    print(f'cmdline: {cmdline}', flush=True)
    if loader_only:
        print('loader-only: no handoff, ADT root extents not checked', flush=True)
        return None
    if ext is None or blocks is None:
        if userdata_ufsroot(args):
            fail('payload boots rd=ufsroot0 from userdata (the default ufsroot) but its ADT '
                 'has no fuxi-root-extents/-blocks')
        return None
    if len(ext) % 24 or len(blocks) != 8:
        fail('payload ADT fuxi-root-extents/-blocks are malformed')
    got = list(struct.unpack(f'<{len(ext) // 8}Q', ext))
    want = [v // BLK for e in table['extents'] for v in e]
    if got != want:
        fail('payload ADT fuxi-root-extents differ from extents.json (rebuild the ADT and payload)')
    root_blocks, = struct.unpack('<Q', blocks)
    if 'root_bytes' in table:
        if root_blocks * BLK != table['root_bytes']:
            fail(f'payload ADT fuxi-root-blocks {root_blocks} != root_bytes {table["root_bytes"]} '
                 f'in extents.json (mkadt-salami.py --root-bytes)')
    elif not 0 < root_blocks * BLK <= table['size']:
        fail(f'payload ADT fuxi-root-blocks {root_blocks} does not fit the {table["size"]}-byte file')
    print(f'payload ADT root extents match extents.json ({len(table["extents"])} extents, '
          f'{root_blocks} blocks)', flush=True)
    return root_blocks


def check_boot_img(path, payload, want_logo=False):
    """The image fastboot boots must be a salami loader carrying exactly this payload.

    Returns True when the loader is a boot-logo build (LOGO_MARKER); want_logo
    (--logo) refuses any other loader."""
    b = path.read_bytes()
    if len(b) < BOOT_PAGE or b[:8] != b'ANDROID!':
        fail(f'{path}: not an Android boot image')
    kernel_size, = struct.unpack_from('<I', b, 8)
    version, = struct.unpack_from('<I', b, 40)
    if version != 4:
        fail(f'{path}: boot header version {version}, expected 4 (shim/fuxi-loader/build.sh)')
    if not 64 <= kernel_size <= len(b) - BOOT_PAGE:
        fail(f'{path}: kernel size {kernel_size} does not fit the image')
    k = b[BOOT_PAGE:BOOT_PAGE + kernel_size]
    if k[56:60] != b'ARM\x64' or struct.unpack_from('<Q', k, 16)[0] != kernel_size:
        fail(f'{path}: kernel is not a loader Image (arm64 header whose image_size is the whole Image)')
    loader = k
    if payload is not None:
        if not k.endswith(payload):
            fail(f'{path} does not embed --payload (rebuild it with shim/fuxi-loader/build.sh from '
                 f'this payload, or pass the payload it was built from)')
        loader = k[:len(k) - len(payload)]
    if LOADER_MARKER not in loader:
        fail(f'{path}: not a salami loader (no FUXI_SALAMI splash-overlap guard; build it with '
             f'FUXI_BOARD=salami)')
    logo = LOGO_MARKER in loader
    if want_logo and not logo:
        fail(f'{path}: --logo, but the loader has no boot logo (build it with FUXI_BOOT_LOGO, '
             f'shim/fuxi-loader/build.sh)')
    handover = logo and LOGO_HANDOVER in loader
    print(f'boot image: {path.name}, salami loader'
          + (' with boot logo' + (' (appends sweetlogo_* for the kernel)' if handover else '') if logo else '')
          + f', kernel {kernel_size} bytes'
          + (f', ends with the {len(payload)}-byte payload' if payload is not None else ''), flush=True)
    # Verbose then logo needs both halves: a 0115 kernel with nothing handed over has no
    # logo to draw after the text (only matters when the ADT gives it a text console).
    if (logo and not handover and payload is not None and adt_has_fbcon(payload)
            and payload_kernel_has(payload, VERBOSE_LOGO_KERNEL)):
        print('warning: the payload kernel has xnu/patches/0115 and the ADT has sweet-fbcon, but this '
              'boot-logo loader appends no sweetlogo_* (built before the loader handover): after the '
              'text there is no logo for the kernel to draw', flush=True)
    if handover and payload is not None and len(payload) >= 0x48 + 608:
        used = len(payload[0x48:0x48 + 608].split(b'\0')[0])
        if used + LOGO_APPEND_MAX > BOOT_LINE_CHARS:
            print(f'warning: the payload cmdline is {used} characters; the loader appends up to '
                  f'{LOGO_APPEND_MAX} (sweetlogo_pa/_w/_h) and appends nothing past {BOOT_LINE_CHARS}, '
                  f'so the logo may not reach the kernel (monitor request 0x48 would say so)', flush=True)
    return logo


def check_phone(phone, table, helper_local):
    code, out, err = phone.run(['get-state'])
    if out.strip() != 'device':
        fail(f"adb get-state is {out.strip() or err.strip()!r}, expected 'device' (boot Android with Magisk)")
    if phone.shell('getprop ro.product.device') != DEVICE:
        fail(f'ro.product.device is not {DEVICE}')
    if phone.su('id -u')[1] != '0':
        fail('su did not give uid 0 (grant Magisk root to the shell)')
    print(f'phone: {DEVICE}, adb device state, su ok', flush=True)
    if table is None:
        return
    if phone.su('readlink -f /dev/block/by-name/userdata')[1] != f'/dev/block/{PART}':
        fail(f'by-name/userdata is not {PART}')
    if phone.su(f'cat /sys/class/block/{PART}/size')[1] != str(PART_SECTORS):
        fail(f'{PART} size changed')
    part_dev = phone.su(f'cat /sys/class/block/{PART}/dev')[1]
    targets = [l for l in phone.su('dmctl table userdata')[1].splitlines() if re.match(r'^\s*\d+-\d+:', l)]
    m = len(targets) == 1 and re.match(r'^\s*(\d+)-(\d+):\s*default-key,\s*(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)', targets[0])
    if not m or m[1] != '0' or m[2] != str(PART_SECTORS) or m[6] not in (part_dev, f'/dev/block/{PART}') or m[7] != '0':
        fail(f'userdata is not one default-key target over {PART} at offset 0: {targets}')
    print(f'userdata: one default-key target over {PART} ({part_dev}) at offset 0', flush=True)
    if helper_local:
        code, _, err = phone.run(['push', str(helper_local), HELPER])
        if code:
            fail(f'adb push {helper_local}: {err.strip()}')
        phone.shell(f'chmod 755 {HELPER}')
    code, out, err = phone.su(f'{HELPER} verify {table["file"]} {table["size"]}', check=False)
    if code:
        fail(f'salami-pin verify refused ({code}): {err or out} '
             f'(missing helper: pass --push-helper tools/out/salami-pin)')
    if err:
        print(err, file=sys.stderr, flush=True)
    try:
        live = json.loads(out)
    except ValueError:
        fail(f'salami-pin verify printed no JSON: {out[:200]!r}')
    for key in ('file', 'size', 'extents'):
        if live.get(key) != table.get(key):
            fail(f'FIEMAP {key} differs from extents.json: the pinned root moved or changed; do not boot')
    if live.get('digest') != fnv(table['size'], table['extents']):
        fail('live extent digest differs from extents.json')
    if live.get('pinned') is not True or live.get('compressed') is not False:
        fail('pinned root lost its pin or became compressed')
    if live.get('partition') != PART or live.get('dm_name') != 'userdata':
        fail(f'pinned root is on {live.get("dm_name")}/{live.get("partition")}')
    thresh, gcf = live.get('gc_pin_file_thresh', -1), live.get('gc_failures', 0)
    print(f'pinned root extents verified: {len(live["extents"])} extents, digest {live["digest"]}, '
          f'GC failures {gcf} of {thresh}', flush=True)
    if thresh and thresh > 0 and gcf >= thresh // 2:
        print(f'warning: {gcf} GC failures on the pin (threshold {thresh}); recreate the file soon', flush=True)
    # Information only: the live check above is what gates the boot.
    code, status, _ = phone.su(f'cat {STATUS}', check=False)
    status = status if code == 0 and status else ''
    print(f'boot check: {status or "no status (Magisk service not installed or not run)"}', flush=True)
    if status and 'OK' not in status.split()[:2]:
        print('warning: the Magisk boot check did not report OK at the last Android boot', flush=True)
    if isinstance(thresh, int) and 0 <= thresh < GC_PIN_THRESH:
        print(f'warning: gc_pin_file_thresh is {thresh}, not {GC_PIN_THRESH}: the Magisk boot check '
              f'(/data/adb/service.d/salami-ios-root.sh) did not raise it this boot', flush=True)


def wait_fastboot(serial, seconds=60):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        p = subprocess.run(['fastboot', 'devices'], capture_output=True, text=True)
        if any(line.split()[:1] == [serial] for line in p.stdout.splitlines()):
            return
        time.sleep(0.5)
    fail('phone did not reach fastboot')


def wait_monitor(seconds=10):
    def find():
        try:
            import usb.core
            return bool(list(usb.core.find(find_all=True, idVendor=0x1209, idProduct=0x0001)))
        except Exception:
            p = subprocess.run(['lsusb', '-d', '1209:0001'], capture_output=True, text=True)
            return bool(p.stdout.strip())
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if find():
            return
        time.sleep(0.1)
    fail('native monitor 1209:0001 did not enumerate (OnePlus ABL may have torn USB down)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
                                 allow_abbrev=False)
    ap.add_argument('out', type=Path, help='monitor output directory (must not exist); give it first')
    ap.add_argument('--boot-img', type=Path, required=True, help='salami loader image for fastboot boot')
    ap.add_argument('--kernel', type=Path, help='unstripped kernel for the monitor')
    ap.add_argument('--payload', type=Path, help='SWEETXNU payload embedded in --boot-img (its ADT is checked)')
    ap.add_argument('--extents', type=Path, default=DEFAULT_EXTENTS, help='extents.json from salami-install-root.sh')
    ap.add_argument('--serial', default=os.environ.get('SALAMI_SERIAL'), help='adb/fastboot serial (or SALAMI_SERIAL)')
    ap.add_argument('--adb', default='adb')
    ap.add_argument('--loader-only', action='store_true', help='no root checks; the monitor gets --no-handoff')
    ap.add_argument('--no-monitor', action='store_true', help='stop after fastboot boot')
    ap.add_argument('--check-only', action='store_true', help='run the checks, do not reboot')
    ap.add_argument('--allow-rw', action='store_true', help='accept ufsrw (read-write) in the payload cmdline; needed for the first boot')
    ap.add_argument('--print-cmdline', action=PrintCmdline,
                    help='print the recommended salami first-boot payload cmdline and exit')
    ap.add_argument('--fb-geometry', type=fb_geometry, metavar='WxH:S',
                    help='expected ADT sweet-fbcon geometry (default 1080x2412:4320, the salami loader)')
    ap.add_argument('--logo', action='store_true',
                    help='require a boot-logo loader (FUXI_BOOT_LOGO); an ADT without sweet-fbcon '
                         '(mkadt-salami.py --no-fbcon, logo only) is accepted only with one, and with '
                         'sweet-fbcon plus a 0115 kernel the boot is verbose then logo')
    ap.add_argument('--push-helper', type=Path, help='push this local salami-pin to the phone first')
    ap.add_argument('--monitor-python', default=sys.executable)
    args, extra = ap.parse_known_args()
    extra = split_monitor_args(extra)
    if not args.serial:
        ap.error('give --serial or SALAMI_SERIAL')
    if not args.boot_img.is_file():
        ap.error(f'{args.boot_img}: no such loader image')
    monitor = not args.no_monitor
    if monitor and not (args.kernel and args.payload):
        ap.error('the monitor needs --kernel and --payload (or pass --no-monitor)')
    if not args.loader_only and not args.payload:
        ap.error('an iOS boot needs --payload, so its ADT extents can be checked')
    if args.payload and not args.payload.is_file():
        ap.error(f'{args.payload}: no such payload')
    if args.out.exists() and monitor:
        ap.error(f'{args.out} exists')
    for tool in (args.adb, 'fastboot'):
        if not shutil.which(tool):
            ap.error(f'{tool} not found (checked before the phone is rebooted)')
    if monitor and subprocess.run([args.monitor_python, '-c', 'import usb.core'], capture_output=True).returncode:
        ap.error(f'{args.monitor_python} cannot import pyusb (usb.core); the monitor needs it')

    table = None
    if not args.loader_only:
        if not args.extents.is_file():
            ap.error(f'{args.extents}: no extents.json (run tools/salami-install-root.sh)')
        table = load_extents(args.extents)
    payload = args.payload.read_bytes() if args.payload else None
    # The loader first: whether it draws the boot logo decides if the ADT may lack sweet-fbcon.
    logo = check_boot_img(args.boot_img, payload, args.logo)
    if payload is not None:
        check_payload(args.payload, payload, table, args.loader_only, args.allow_rw,
                      args.fb_geometry or LOADER_FB, logo, args.fb_geometry is not None)
    payload = None
    phone = Phone(args.serial, args.adb)
    check_phone(phone, table, args.push_helper)
    if args.check_only:
        print('all checks passed; --check-only, not rebooting', flush=True)
        return 0

    code, _, err = phone.run(['reboot', 'bootloader'])
    if code:
        fail(f'adb reboot bootloader: {err.strip()}')
    wait_fastboot(args.serial)
    try:
        subprocess.run(['fastboot', '-s', args.serial, 'boot', str(args.boot_img)], check=True, timeout=30)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        fail(f'fastboot boot: {error}')
    if not monitor:
        print('fastboot boot sent; no monitor (the loader resets at its 90 s deadline without a host)', flush=True)
        return 0
    wait_monitor()
    cmd = [args.monitor_python, str(MONITOR), '--out', str(args.out), '--kernel', str(args.kernel),
           '--payload', str(args.payload), '--skip-clocks']
    cmd += ['--no-handoff'] if args.loader_only else FULL_BOOT_ARGS
    cmd += extra
    if any(a.split('=', 1)[0].startswith('--touc') for a in cmd):
        fail('internal: touch argument in the monitor command')
    print('monitor args', cmd[2:], flush=True)
    return subprocess.run(cmd).returncode


if __name__ == '__main__':
    raise SystemExit(main())
