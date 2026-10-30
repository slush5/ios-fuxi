#!/usr/bin/env python3
"""xnu fixer: refresh (or --check) the new-file blocks of 0099-fuxi-wlan.patch.

0099 = bsd/conf/files hunk, bsd/dev/sweet_ufs.c hunk (both kept byte for byte),
then the new files bsd/dev/fuxi_wlan.c (exact copy of xnu/fuxi-wlan/fuxi_wlan.c),
bsd/dev/fuxi_wlan_ce.h (exact copy of xnu/fuxi-wlan/fuxi_wlan_ce.h, W20/C1; it
must sit before wlan.c, whose block the m6 tests read to the end of the patch)
and bsd/dev/wlan.c (exact copy of shim/fuxi-probe/wlan.c), in that order, in
the '--- a/X' / '+++ b/X' / '@@ -0,0 +1,N @@' form the m6 tests reconstruct.
A patch from before W20 (no header block) is accepted as input and refreshed.
Usage: xnufix-sync-0099.py REPO [--check]
"""
import re
import sys
from pathlib import Path

repo = Path(sys.argv[1])
check = '--check' in sys.argv[2:]
patch_path = repo / 'xnu/patches/0099-fuxi-wlan.patch'
text = patch_path.read_text()
blocks = re.split(r'(?=^--- a/)', text, flags=re.M)
assert blocks[0] == '', 'unexpected text before the first block'
blocks = blocks[1:]
names = [re.match(r'--- a/(\S+)\n', b)[1] for b in blocks]
head = ['bsd/conf/files', 'bsd/dev/sweet_ufs.c', 'bsd/dev/fuxi_wlan.c']
assert names in (head + ['bsd/dev/wlan.c'], head + ['bsd/dev/fuxi_wlan_ce.h', 'bsd/dev/wlan.c']), names


def new_file(target, source):
    body = source.read_text()
    assert body.endswith('\n') and '\r' not in body, source
    lines = body.splitlines(True)
    return f'--- a/{target}\n+++ b/{target}\n@@ -0,0 +1,{len(lines)} @@\n' + ''.join('+' + l for l in lines)


expected = (blocks[0] + blocks[1] +
            new_file('bsd/dev/fuxi_wlan.c', repo / 'xnu/fuxi-wlan/fuxi_wlan.c') +
            new_file('bsd/dev/fuxi_wlan_ce.h', repo / 'xnu/fuxi-wlan/fuxi_wlan_ce.h') +
            new_file('bsd/dev/wlan.c', repo / 'shim/fuxi-probe/wlan.c'))
if check:
    assert text == expected, '0099 differs from fuxi_wlan.c / fuxi_wlan_ce.h / wlan.c; run without --check'
    print('PASS: 0099 fuxi_wlan.c, fuxi_wlan_ce.h and wlan.c blocks match their canonical sources')
else:
    changed = text != expected
    patch_path.write_text(expected)
    print('0099 refreshed' if changed else '0099 already current')
