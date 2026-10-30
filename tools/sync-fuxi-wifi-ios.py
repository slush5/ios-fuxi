#!/usr/bin/env python3
"""Generate/check P-WIFI's source overlay; no XNU checkout or reference execution."""
import argparse
import difflib
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'xnu/fuxi-wifi-ios'
PATCH = ROOT / 'xnu/patches/0104-fuxi-wifi-ios.patch'
INTEGRATION = '''--- a/bsd/conf/files
+++ b/bsd/conf/files
@@ -113,3 +113,10 @@
 bsd/dev/fuxi_wlan.c\t\t\tstandard
+bsd/dev/fuxi_wifi_ios.c\t\t\tstandard
+bsd/dev/fuxi_wifi_net.c\t\t\tstandard
+bsd/dev/fuxi_wifi_sta.c\t\t\tstandard
+bsd/dev/fuxi_wifi_apple80211.c\t\tstandard
+bsd/dev/fuxi_wifi_radio.c\t\tstandard
+bsd/dev/fuxi_wifi_data.c\t\tstandard
+bsd/dev/fuxi_wifi_wpa.c\t\t\tstandard
 bsd/dev/sweet_rpmh.c\t\t\tstandard
 bsd/dev/sweet_spmi.c\t\t\tstandard
--- a/iokit/conf/files.arm64
+++ b/iokit/conf/files.arm64
@@ -18,3 +18,5 @@
 iokit/Kernel/arm/FuxiAdrenoUserClient.cpp                         optional iokitcpp
+iokit/Kernel/arm/FuxiWLAN.cpp                                    optional iokitcpp
+iokit/Kernel/arm/FuxiNetworkStack.cpp                            optional iokitcpp
 iokit/Kernel/arm/SweetBattery.cpp                                optional iokitcpp
 iokit/Kernel/arm/hid/SweetHID.cpp                                optional iokitcpp
--- a/bsd/kern/bsd_init.c
+++ b/bsd/kern/bsd_init.c
@@ -873,3 +873,10 @@
 \tnet_init_run();
+#if defined(FUXI)
+\t/* P-WIFI: must run after Ethernet/protocol initialization, before launchd. */
+\t{
+\t\textern void fuxi_wifi_ios_init(void);
+\t\tfuxi_wifi_ios_init();
+\t}
+#endif
 #endif /* NETWORKING */
\x20
'''


def destination(path):
    if path.suffix == '.cpp':
        return 'iokit/Kernel/arm/' + path.name
    if path.suffix == '.h':
        return 'bsd/sys/' + path.name
    if path.name == 'LICENSE.HackiOS':
        return 'bsd/dev/FuxiWiFi.LICENSE.HackiOS'
    return 'bsd/dev/' + path.name


def expected():
    parts = [INTEGRATION]
    for path in sorted(SOURCE.iterdir()):
        if path.suffix not in ('.h', '.c', '.cpp') and path.name != 'LICENSE.HackiOS':
            continue
        parts.extend(difflib.unified_diff([], path.read_text().splitlines(True),
                                         fromfile='/dev/null', tofile='b/' + destination(path)))
    return ''.join(parts)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()
    patch = expected()
    if args.check:
        assert PATCH.read_text() == patch, 'P-WIFI sources differ; run tools/sync-fuxi-wifi-ios.py'
    else:
        PATCH.write_text(patch)
    # Minimal context fixtures exercise all integration hunks; full-stack application
    # is additionally checked against a local XNU tree when available (see lane docs).
    with tempfile.TemporaryDirectory(prefix='fuxi-wifi-overlay-') as tmp:
        fixtures = {
            'bsd/conf/files': 'bsd/dev/fuxi_wlan.c\t\t\tstandard\nbsd/dev/sweet_rpmh.c\t\t\tstandard\nbsd/dev/sweet_spmi.c\t\t\tstandard\n',
            'iokit/conf/files.arm64': 'iokit/Kernel/arm/FuxiAdrenoUserClient.cpp                         optional iokitcpp\niokit/Kernel/arm/SweetBattery.cpp                                optional iokitcpp\niokit/Kernel/arm/hid/SweetHID.cpp                                optional iokitcpp\n',
            'bsd/kern/bsd_init.c': '\tnet_init_run();\n#endif /* NETWORKING */\n\n',
        }
        for name, value in fixtures.items():
            dest = Path(tmp) / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_text(value)
        subprocess.run(['git', 'apply', '--check', '-'], input=patch, text=True, cwd=tmp, check=True)
    print('PASS: P-WIFI canonical sources match overlay; source and integration hunks apply')


if __name__ == '__main__':
    main()
