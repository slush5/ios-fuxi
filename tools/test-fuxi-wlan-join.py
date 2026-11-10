#!/usr/bin/env python3
"""J knobs (fuxiwlanjoin / fuxiwlantx / fuxiwlancc), host-tested.

The join work starts with three boot-arg knobs and one pure gate over them in
xnu/fuxi-wlan/fuxi_wlan.c (the region between the two "J knobs" markers). This
test cuts exactly that region out of the driver, compiles it with ASan/UBSan
and -Werror, and checks it three ways:

  1. Against an independent model written here, over a grid of every knob
     combination worth naming (out-of-range rungs, every transmit value, good
     and malformed countries, every scan path selector, fuxiwlankeep).
  2. Named cases: all three absent is accepted and inert; "IL" on the scan
     path is accepted; any nonzero fuxiwlantx is refused whatever else is set;
     a refusal always leaves both outputs at 0.
  3. Static checks on the driver source: the gate is the only writer of the
     two knob variables, no variable exists that could hold an accepted
     transmit knob, the three names are parsed once each, the passive-scan
     guard is still in the CE header, and the ADT tool mirrors the knobs.

No device, no network. Run: CC=cc python3 -B tools/test-fuxi-wlan-join.py
"""
from pathlib import Path
import itertools
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / 'xnu/fuxi-wlan/fuxi_wlan.c'
CE = ROOT / 'xnu/fuxi-wlan/fuxi_wlan_ce.h'
MKADT = ROOT / 'tools/mkadt-salami.py'
CC = [os.environ.get('CC', 'cc'), '-std=c11', '-g', '-Wall', '-Wextra', '-Werror',
      '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
BEGIN = '/* J knobs (begin):'
END = '/* J knobs (end) */'
JOIN_MAX = 1

HARNESS = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
/* J knobs (begin): %s
int
main(void)
{
	int join, tx, cc, qrtr, keep;
	while (scanf("%%d %%d %%d %%d %%d", &join, &tx, &cc, &qrtr, &keep) == 5) {
		unsigned jo = 0xdeadu;
		uint32_t co = 0xdeadu;
		bool ok = wl_join_knobs(join, tx, cc, qrtr, keep, &jo, &co);
		printf("%%d %%u %%u\n", ok ? 1 : 0, jo, (unsigned)co);
	}
	printf("max %%d\n", WL_JOIN_MAX);
	return 0;
}
'''


def cc_ok(cc):
    if cc <= 0 or cc >> 16:
        return False
    a, b = cc & 0xff, (cc >> 8) & 0xff
    return ord('A') <= a <= ord('Z') and ord('A') <= b <= ord('Z')


def model(join, tx, cc, qrtr, keep):
    """The gate, restated independently of the C."""
    refused = (0, 0, 0)
    if tx != 0:
        return refused
    if join < 0 or join > JOIN_MAX:
        return refused
    if join == 0:
        return (1, 0, 0) if cc == 0 else refused
    if qrtr not in (7, 8) or keep != 0 or not cc_ok(cc):
        return refused
    return (1, join, cc)


def alpha2(s):
    return ord(s[0]) | (ord(s[1]) << 8)


def main():
    src = DRIVER.read_text()
    assert src.count(BEGIN) == 1 and src.count(END) == 1, 'J knobs markers'
    region = src.split(BEGIN, 1)[1].split(END, 1)[0]
    # The region is pure: nothing in it may reach the device, the log or a global.
    for word in ('printf', 'wl_wmi', 'wl_ce', 'IODelay', 'wl_variant_invalid', 'PE_parse_boot_argn'):
        assert word not in region, 'J knobs region is not pure: ' + word

    INT_MAX, INT_MIN = 2**31 - 1, -2**31
    joins = [INT_MIN, -1, 0, 1, 2, 3, 4, 5, INT_MAX]
    txs = [INT_MIN, -1, 0, 1, 2, 3, 4, INT_MAX]
    ccs = [INT_MIN, -1, 0, alpha2('IL'), alpha2('US'), alpha2('AA'), alpha2('ZZ'),
           alpha2('il'), alpha2('Il'), alpha2('iL'), alpha2('@A'), alpha2('A@'), alpha2('[A'), alpha2('A['),
           ord('I'), ord('L') << 8, alpha2('IL') | 0x10000, alpha2('IL') | 0x1000000, 0x3030, INT_MAX]
    qrtrs = [-1, 0, 1, 6, 7, 8, 9]
    keeps = [-1, 0, 1, 2]
    grid = list(itertools.product(joins, txs, ccs, qrtrs, keeps))

    with tempfile.TemporaryDirectory() as d:
        c = Path(d) / 'join.c'
        exe = Path(d) / 'join'
        c.write_text(HARNESS % region)
        subprocess.run(CC + ['-o', str(exe), str(c)], check=True)
        feed = ''.join('%d %d %d %d %d\n' % g for g in grid)
        out = subprocess.run([str(exe)], input=feed, capture_output=True, text=True, check=True).stdout.split('\n')
    assert out[len(grid)] == 'max %d' % JOIN_MAX, 'WL_JOIN_MAX moved: this test and its model must move with it'
    got = {}
    accepted = 0
    for g, line in zip(grid, out):
        r = tuple(int(x) for x in line.split())
        assert r == model(*g), 'gate differs from the model at %r: %r' % (g, r)
        got[g] = r
        accepted += r[0]
        if not r[0]:
            assert r[1:] == (0, 0), 'a refusal left an output set at %r' % (g,)
        if g[1] != 0:
            assert r == (0, 0, 0), 'a nonzero fuxiwlantx was accepted at %r' % (g,)

    il = alpha2('IL')
    assert il == 0x4c49 == 19529
    # all three absent: accepted and inert, on any scan path, with or without keep
    for q, k in itertools.product(qrtrs, keeps):
        assert got[(0, 0, 0, q, k)] == (1, 0, 0)
    # the first rung with a country on either scan path
    assert got[(1, 0, il, 7, 0)] == (1, 1, il)
    assert got[(1, 0, il, 8, 0)] == (1, 1, il)
    # and its refusals: no country, no scan path, keep, a rung this build lacks, a country alone
    assert got[(1, 0, 0, 8, 0)] == (0, 0, 0)
    assert got[(1, 0, il, 6, 0)] == (0, 0, 0)
    assert got[(1, 0, il, 0, 0)] == (0, 0, 0)
    assert got[(1, 0, il, 8, 1)] == (0, 0, 0)
    assert got[(2, 0, il, 8, 0)] == (0, 0, 0)
    assert got[(0, 0, il, 8, 0)] == (0, 0, 0)
    assert got[(1, 0, alpha2('il'), 8, 0)] == (0, 0, 0)
    assert got[(1, 0, il | 0x10000, 8, 0)] == (0, 0, 0)

    # --- static checks on the driver ---
    for name in ('fuxiwlanjoin', 'fuxiwlantx', 'fuxiwlancc'):
        assert len(re.findall(r'PE_parse_boot_argn\("%s",' % name, src)) == 1, name + ' is not parsed exactly once'
    # one call of the gate, and it alone writes the two knob variables
    assert len(re.findall(r'\bwl_join_knobs\(', src)) == 2, 'the gate is defined once and called once'
    assert 'if (!wl_join_knobs(jjn, jtx, jcc, qrtr, keep, &wl_join, &wl_cc)) { wl_variant_invalid = true; }' in src
    for var in ('wl_join', 'wl_cc'):
        assert not re.search(r'\b%s\s*(=[^=]|\+\+|--|[-+|&^]=)' % var, src), var + ' is written outside the gate'
        assert len(re.findall(r'&%s\b' % var, src)) == 1, var + ' has its address taken outside the gate call'
    # nothing can hold an accepted transmit knob: the parsed value lives in one local and feeds the gate only
    ce = CE.read_text()
    for text in (src, ce):
        assert not re.search(r'\bwl_tx\b', text), 'a transmit knob variable exists'
    assert len(re.findall(r'\bjtx\b', src.split('int jjn = 0, jtx = 0, jcc = 0;', 1)[1].split('wl_w25 = qrtr == 8;', 1)[0])) == 5, \
        'the parsed fuxiwlantx value is used outside its parse, the gate call and the print'
    # the passive-scan guard of the scan code is still there
    assert ce.count('_Static_assert(WLSCAN_FLAGS == 0x11u && !(WLSCAN_FLAGS & 0x2u)') == 1, 'passive scan guard'
    # the ADT tool mirrors the three knobs as integers
    mk = MKADT.read_text()
    for name in ('fuxiwlanjoin', 'fuxiwlantx', 'fuxiwlancc'):
        assert "'%s': 'i'" % name in mk, name + ' is missing from WLAN_KNOBS'

    print('J knobs host tests passed: %d combinations against the model (%d accepted), named cases and static checks' % (len(grid), accepted))


if __name__ == '__main__':
    main()
