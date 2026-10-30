#!/usr/bin/env python3
"""Tests for tools/check-hygiene.py: every rule against a throwaway git repo built here.

Fixture data is made up: SSIDs are invented names, MACs are locally administered
02:00:00:xx addresses, and the one globally administered MAC is assembled at run time
from the RFC 7042 documentation block (00:00:5E:00:53:xx), so this file stays clean
under the checker it tests.
"""
import importlib.util
import os
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

TOOL = Path(__file__).resolve().parent / 'check-hygiene.py'
GOOD = ('slush5', '83555836+slush5@users.noreply.github.com')
OTHER = ('Someone Else', 'someone@example.invalid')
EM, EN = '\u2014', '\u2013'
DOC_MAC = ':'.join(['00', '00', '5e', '00', '53', '01'])
LA_MAC = '02:00:00:00:00:2a'
NET = 'MadeUpNet7'
SSID_Q = 'ss' + 'id "'
SSID_EQ = 'SS' + 'ID='

spec = importlib.util.spec_from_file_location('check_hygiene', TOOL)
hyg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hyg)

SYNC_TOOL = '''\
#!/usr/bin/env python3
import argparse
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
PATCH = ROOT / 'xnu/patches/0001-demo.patch'
ap = argparse.ArgumentParser(); ap.add_argument('--check', action='store_true')
args = ap.parse_args()
want = 'demo: ' + (ROOT / 'src/demo.txt').read_text()
if args.check:
    assert PATCH.read_text() == want, 'demo differs; run tools/sync-demo.py'
else:
    PATCH.write_text(want)
'''

LEGACY_SYNC_TOOL = '''\
#!/usr/bin/env python3
"""Usage: sync-legacy.py REPO [--check]"""
import sys
from pathlib import Path
repo = Path(sys.argv[1])
check = '--check' in sys.argv[2:]
patch = repo / 'xnu/patches/0002-legacy.patch'
want = 'legacy: ' + (repo / 'src/legacy.txt').read_text()
if check:
    assert patch.read_text() == want, 'legacy differs'
'''


class Repo:
    def __init__(self, tmp, base_ref='salami'):
        self.root = Path(tmp) / 'repo'
        self.root.mkdir()
        self.git('init', '-q', '-b', 'main')
        self.write('README.md', 'base\n')
        self.write('old.md', f'pre-existing {EM} line\n')
        self.commit('base')
        if base_ref == 'salami':
            self.git('branch', 'salami')
        else:
            self.git('update-ref', base_ref, 'HEAD')

    def git(self, *args, ident=GOOD, committer=None):
        cn, ce = committer or ident
        env = dict(os.environ, GIT_AUTHOR_NAME=ident[0], GIT_AUTHOR_EMAIL=ident[1],
                   GIT_COMMITTER_NAME=cn, GIT_COMMITTER_EMAIL=ce,
                   GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM='1')
        return subprocess.run(['git', *args], cwd=self.root, env=env, check=True,
                              capture_output=True, text=True).stdout

    def write(self, path, content):
        p = self.root / path
        p.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(content, bytes):
            p.write_bytes(content)
        else:
            p.write_text(content)

    def commit(self, msg, ident=GOOD, committer=None, **files):
        for path, content in files.items():
            self.write(path.replace('__', '/'), content)
        self.git('add', '-A')
        self.git('commit', '-q', '--allow-empty', '-m', msg, ident=ident, committer=committer)

    def check(self, *args):
        proc = subprocess.run([sys.executable, str(TOOL), *args], cwd=self.root,
                              capture_output=True, text=True)
        return proc.returncode, proc.stdout.splitlines()


class HygieneTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.repo = Repo(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def assertLines(self, lines, *needles):
        for n in needles:
            self.assertTrue(any(n in ln for ln in lines), f'{n!r} not in {lines}')

    # clean
    def test_clean_range_passes(self):
        self.repo.commit('add notes', notes__a__txt='plain ascii - hyphen only\n')
        self.assertEqual(self.repo.check(), (0, []))

    # rule 1: dashes
    def test_em_dash_in_changed_line(self):
        self.repo.commit('add', **{'src__x.c': 'int a;\n/* one ' + EM + ' two */\n'})
        rc, out = self.repo.check()
        self.assertEqual(rc, 1)
        self.assertEqual(out, ['error: src/x.c:2: U+2014 em dash'])

    def test_en_dash_in_changed_line(self):
        self.repo.commit('add', **{'doc.md': 'range 1' + EN + '9\n'})
        rc, out = self.repo.check()
        self.assertEqual((rc, out), (1, ['error: doc.md:1: U+2013 en dash']))

    def test_unchanged_dash_line_not_reported(self):
        self.repo.commit('touch old', **{'old.md': f'pre-existing {EM} line\nnew line\n'})
        self.assertEqual(self.repo.check(), (0, []))

    def test_dash_in_commit_message(self):
        self.repo.commit('fix ' + EM + ' thing', **{'a.txt': 'x\n'})
        rc, out = self.repo.check()
        self.assertEqual(rc, 1)
        self.assertLines(out, 'U+2014 em dash in commit message')

    def test_dash_in_binary_ignored(self):
        self.repo.commit('blob', **{'fw.bin': b'\x00\xff' + EM.encode() + b'\x00' * 64})
        self.assertEqual(self.repo.check(), (0, []))

    # rule 2: identity
    def test_bad_author(self):
        self.repo.commit('by other', ident=OTHER, committer=GOOD, **{'a.txt': 'x\n'})
        rc, out = self.repo.check()
        self.assertEqual(rc, 1)
        self.assertEqual(len(out), 1)
        self.assertLines(out, 'author is Someone Else <someone@example.invalid>')

    def test_bad_committer(self):
        self.repo.commit('committed by other', committer=OTHER, **{'a.txt': 'x\n'})
        rc, out = self.repo.check()
        self.assertEqual(rc, 1)
        self.assertEqual(len(out), 1)
        self.assertLines(out, 'committer is Someone Else')

    def test_commits_before_base_ignored(self):
        repo = self.repo
        repo.git('checkout', '-q', 'salami')
        repo.commit('old other', ident=OTHER, **{'b.txt': 'y\n'})
        repo.git('checkout', '-q', 'main')
        repo.commit('mine', **{'c.txt': 'z\n'})
        repo.git('merge', '-q', '--no-ff', '--no-edit', 'salami')
        # salami's own history (and its b.txt) is not part of salami..HEAD
        self.assertEqual(repo.check(), (0, []))
        rc, out = repo.check('HEAD~2..HEAD')
        self.assertEqual(rc, 1)
        self.assertLines(out, 'author is Someone Else')

    # rule 3: NUL
    def test_nul_in_text_file(self):
        self.repo.commit('nul', **{'src__y.c': b'int a;\nint\x00b;\n'})
        rc, out = self.repo.check()
        self.assertEqual((rc, out), (1, ['error: src/y.c:2: NUL byte in text file']))

    def test_nul_in_unknown_ext_text(self):
        self.repo.commit('nul', **{'notes': b'hello\x00world\nline two\n' * 20})
        rc, out = self.repo.check()
        self.assertEqual(rc, 1)
        self.assertLines(out, 'notes:1: NUL byte in text file (20 lines)')

    def test_nul_in_binary_ok(self):
        self.repo.commit('blob', **{'tbl.dat': b'\x00' * 32 + b'abc', 'pad': b'ab' + b'\x00' * 400})
        self.assertEqual(self.repo.check(), (0, []))

    # rule 4: SSID/BSSID warnings
    def test_global_mac_warns(self):
        self.repo.commit('log', **{'logs__scan.txt': f'bssid {DOC_MAC} rssi -40\n'})
        rc, out = self.repo.check()
        self.assertEqual(rc, 0)
        self.assertEqual(len(out), 1)
        self.assertTrue(out[0].startswith('warning: logs/scan.txt:1: MAC ' + DOC_MAC))
        self.assertEqual(self.repo.check('--strict')[0], 1)

    def test_global_mac_dash_form_warns(self):
        self.repo.commit('log', **{'a.txt': DOC_MAC.replace(':', '-').upper() + '\n'})
        rc, out = self.repo.check()
        self.assertEqual(rc, 0)
        self.assertLines(out, 'warning: a.txt:1: MAC')

    def test_benign_macs_quiet(self):
        body = (f'local {LA_MAC}\nmcast 01:00:5e:00:00:fb\nzero 00:00:00:00:00:00\n'
                f'ipv6 fe80::1:2:3:4:5:6\nlong 00:00:5e:00:53:01:02\n')
        self.repo.commit('macs', **{'a.txt': body})
        self.assertEqual(self.repo.check(), (0, []))

    def test_mac_in_fixture_dir_quiet(self):
        self.repo.commit('fx', **{'tools__fixtures__scan.txt': f'{DOC_MAC}\n{SSID_EQ}{NET}\n'})
        self.assertEqual(self.repo.check(), (0, []))

    def test_allowlist_comment(self):
        self.repo.commit('allow', **{'a.c': f'/* {DOC_MAC} hygiene: allow */\n'
                                            f'const char *n = "{SSID_EQ}{NET}"; // hygiene: allow\n'})
        self.assertEqual(self.repo.check(), (0, []))

    def test_ssid_quoted_warns(self):
        self.repo.commit('s', **{'a.txt': f'joined {SSID_Q}{NET}" ok\n'})
        rc, out = self.repo.check()
        self.assertEqual(rc, 0)
        self.assertEqual(out, [f'warning: a.txt:1: ssid "{NET}" looks like a real network name'])

    def test_ssid_assign_warns(self):
        self.repo.commit('s', **{'a.sh': f'export {SSID_EQ}"{NET}"\n'})
        rc, out = self.repo.check()
        self.assertEqual(rc, 0)
        self.assertEqual(out, [f'warning: a.sh:1: SSID={NET} looks like a real network name'])

    def test_ssid_placeholders_quiet(self):
        body = ''.join(f'{line}\n' for line in [
            f'{SSID_Q}%s"', f'{SSID_Q}<ssid>"', f'{SSID_Q}"', f'{SSID_EQ}$1', f'{SSID_EQ}${{NAME}}',
            f'{SSID_EQ}{{ssid}}', f'{SSID_EQ}<your-ssid>', f'{SSID_EQ}XXXX', f'{SSID_EQ}example-net',
            f'{SSID_EQ}test-ap', f'B{SSID_EQ}{LA_MAC}',
        ])
        self.repo.commit('s', **{'a.txt': body})
        self.assertEqual(self.repo.check(), (0, []))

    # rule 5: patch sync
    def _sync_repo(self):
        r = self.repo
        r.write('tools/sync-demo.py', SYNC_TOOL)
        r.write('src/demo.txt', 'one\n')
        r.write('xnu/patches/0001-demo.patch', 'demo: one\n')
        r.commit('add sync tool')
        r.git('branch', '-f', 'salami', 'HEAD')
        return r

    def test_patch_in_sync(self):
        r = self._sync_repo()
        r.commit('bump', **{'src__demo.txt': 'two\n', 'xnu__patches__0001-demo.patch': 'demo: two\n'})
        self.assertEqual(r.check(), (0, []))

    def test_patch_out_of_sync(self):
        r = self._sync_repo()
        r.commit('edit patch only', **{'xnu__patches__0001-demo.patch': 'demo: hand edit\n'})
        rc, out = r.check()
        self.assertEqual(rc, 1)
        self.assertEqual(len(out), 1)
        self.assertLines(out, 'error: xnu/patches/0001-demo.patch: out of sync: tools/sync-demo.py --check exited 1',
                         'demo differs')

    def test_patch_without_tool_skipped(self):
        r = self._sync_repo()
        r.commit('other patch', **{'xnu__patches__0009-other.patch': 'anything\n'})
        self.assertEqual(r.check(), (0, []))

    def test_untouched_patch_not_checked(self):
        r = self._sync_repo()
        r.commit('unrelated', **{'a.txt': 'x\n'})
        r.write('xnu/patches/0001-demo.patch', 'demo: stale\n')  # drifted, but not in the range
        self.assertEqual(r.check(), (0, []))

    def test_legacy_positional_tool(self):
        r = self.repo
        r.write('tools/sync-legacy.py', LEGACY_SYNC_TOOL)
        r.write('src/legacy.txt', 'a\n')
        r.commit('legacy', **{'xnu__patches__0002-legacy.patch': 'legacy: a\n'})
        self.assertEqual(r.check(), (0, []))
        r.commit('drift', **{'xnu__patches__0002-legacy.patch': 'legacy: b\n'})
        rc, out = r.check()
        self.assertEqual(rc, 1)
        self.assertLines(out, 'tools/sync-legacy.py --check exited 1')

    # CLI modes
    def test_files_mode_checks_whole_file_only(self):
        r = self.repo
        r.commit('by other', ident=OTHER, **{'a.txt': 'x\n'})
        rc, out = r.check('--files', 'old.md')
        self.assertEqual((rc, out), (1, ['error: old.md:1: U+2014 em dash']))
        r.write('clean.md', 'fine\n')
        self.assertEqual(r.check('--files', 'clean.md'), (0, []))

    def test_files_mode_patch_sync(self):
        r = self._sync_repo()
        r.write('xnu/patches/0001-demo.patch', 'demo: drift\n')
        rc, out = r.check('--files', 'xnu/patches/0001-demo.patch')
        self.assertEqual(rc, 1)
        self.assertLines(out, 'out of sync: tools/sync-demo.py')

    def test_range_and_files(self):
        r = self.repo
        r.commit('by other', ident=OTHER, committer=GOOD, **{'a.txt': 'x\n'})
        rc, out = r.check('salami..HEAD', '--files', 'old.md')
        self.assertEqual(rc, 1)
        self.assertEqual(len(out), 2)
        self.assertLines(out, 'author is Someone Else', 'old.md:1: U+2014')

    def test_origin_salami_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = Repo(tmp, base_ref='refs/remotes/origin/salami')
            r.commit('bad', ident=OTHER, **{'a.txt': 'x\n'})
            rc, out = r.check()
            self.assertEqual(rc, 1)
            self.assertLines(out, 'author is Someone Else')

    def test_bad_range_exits_2(self):
        self.assertEqual(self.repo.check('nosuchref..HEAD')[0], 2)


class HelperTest(unittest.TestCase):
    def test_placeholders(self):
        for v in ['', 'SSID', '<ssid>', '$X', '%s', '{n}', 'xxxx', '...', 'example', 'test-1', 'fake_ap']:
            self.assertTrue(hyg.is_placeholder(v), v)
        for v in [NET, 'CoffeeShop', 'Home5G']:
            self.assertFalse(hyg.is_placeholder(v), v)

    def test_mac_classes(self):
        self.assertTrue(hyg.mac_is_suspect(DOC_MAC))
        self.assertFalse(hyg.mac_is_suspect(LA_MAC))
        self.assertFalse(hyg.mac_is_suspect('01:00:5e:00:00:01'))
        self.assertFalse(hyg.mac_is_suspect('00:00:00:00:00:00'))

    def test_text_detection(self):
        self.assertTrue(hyg.is_text('a.c', b'a\x00b'))
        self.assertFalse(hyg.is_text('a.bin', b'abc'))
        self.assertFalse(hyg.is_text('x', b'\xff\xfe\x00'))
        self.assertFalse(hyg.is_text('x', b'a' + b'\x00' * 100))

    def test_tool_and_tests_are_clean(self):
        here = TOOL.parent
        proc = subprocess.run([sys.executable, str(TOOL), '--strict', '--files', str(TOOL),
                               str(here / 'test-check-hygiene.py')], cwd=here, capture_output=True, text=True)
        self.assertEqual((proc.returncode, proc.stdout), (0, ''))

    def test_fixture_paths(self):
        self.assertTrue(hyg.is_fixture('tools/fixtures/a.txt'))
        self.assertFalse(hyg.is_fixture('tools/fixtures.py'))


if __name__ == '__main__':
    result = unittest.main(exit=False, verbosity=1).result
    run, bad = result.testsRun, len(result.failures) + len(result.errors)
    print(f'test-check-hygiene: {run - bad}/{run} passed')
    sys.exit(1 if bad else 0)
