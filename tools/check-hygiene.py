#!/usr/bin/env python3
"""Pre-push hygiene guard for the public repo (python3 stdlib only).

Checks a git range (default salami..HEAD, falling back to origin/salami) and/or
a list of files, and prints one line per problem:

  error    U+2013 / U+2014 in a changed line of a changed text file (or in a
           commit message of the range)
  error    a commit whose author or committer is not
           slush5 <83555836+slush5@users.noreply.github.com>
  error    a NUL byte in a changed text file
  warning  a line that looks like a printed SSID/BSSID outside test fixtures:
           a full 6-octet MAC that is not locally administered (multicast and
           all-zero are ignored), or `ssid "X"` / `SSID=X` with a value that is
           not a placeholder. Put `hygiene: allow` in a comment on the line to
           accept it.
  error    an xnu/patches/*.patch the range touches whose generator
           tools/sync-*.py --check fails (the sync tool reads the working tree)

Exit status: 0 clean (warnings allowed unless --strict), 1 problems, 2 usage or
git failure.

  tools/check-hygiene.py                    # salami..HEAD
  tools/check-hygiene.py origin/salami..HEAD
  tools/check-hygiene.py --files a.c b.md   # whole files, no commit checks
  tools/check-hygiene.py RANGE --files x.py # both
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path

IDENTITY = ('slush5', '83555836+slush5@users.noreply.github.com')
DASHES = {'\u2013': 'U+2013 en dash', '\u2014': 'U+2014 em dash'}
ALLOW = 'hygiene: allow'
FIXTURE_DIRS = {'fixtures', 'fixture', 'testdata', 'test-fixtures'}
TEXT_EXT = {
    '.c', '.h', '.cc', '.cpp', '.hpp', '.m', '.mm', '.s', '.S', '.rs', '.py', '.sh', '.md',
    '.txt', '.patch', '.diff', '.json', '.yml', '.yaml', '.toml', '.conf', '.cfg', '.ini',
    '.env', '.entitlements', '.exports', '.iig', '.def', '.ld', '.lds', '.mk', '.cmake',
    '.html', '.css', '.js', '.ts', '.xml', '.csv',
}
BINARY_EXT = {
    '.bin', '.img', '.fw', '.mbn', '.dtb', '.dtbo', '.o', '.a', '.so', '.dylib', '.elf',
    '.png', '.jpg', '.jpeg', '.gif', '.ico', '.pdf', '.gz', '.xz', '.zst', '.bz2', '.zip',
    '.tar', '.dmg', '.ipsw', '.im4p', '.car', '.metallib', '.spv', '.dat', '.raw', '.plist',
}
MAC = re.compile(r'(?<![0-9A-Fa-f])(?<![0-9A-Fa-f][:-])'
                 r'([0-9A-Fa-f]{2})([:-])(?:[0-9A-Fa-f]{2}\2){4}[0-9A-Fa-f]{2}'
                 r'(?![0-9A-Fa-f])(?![:-][0-9A-Fa-f])')
SSID_QUOTED = re.compile(r'\bssid "([^"]*)"?', re.I)
SSID_ASSIGN = re.compile(r'\bSSID=([\'"]?)([^\s\'"`;,)]*)', re.I)
PLACEHOLDERS = {
    'ssid', 'your_ssid', 'yourssid', 'your-ssid', 'myssid', 'my_ssid', 'example', 'placeholder',
    'redacted', 'dummy', 'none', 'null', 'nil', 'test', 'testnet', 'name', 'network', 'value',
}
PLACEHOLDER_RE = re.compile(r'^(?:[<${%\\]|example|test[-_]|fake|dummy|placeholder|[xX*._-]+$)')


class GitError(Exception):
    pass


def git(root, *args, binary=False):
    proc = subprocess.run(['git', '-c', 'core.quotePath=false', *args], cwd=root,
                          capture_output=True)
    if proc.returncode:
        raise GitError(f"git {' '.join(args)}: {proc.stderr.decode(errors='replace').strip()}")
    return proc.stdout if binary else proc.stdout.decode('utf-8', errors='replace')


def is_text(path, data):
    ext = Path(path).suffix
    if ext in BINARY_EXT:
        return False
    if ext in TEXT_EXT:
        return True
    stripped = data.replace(b'\0', b'')
    try:
        stripped.decode('utf-8')
    except UnicodeDecodeError:
        return False
    # Mostly NULs is a binary blob that happens to be ASCII, not text.
    return (len(data) - len(stripped)) * 10 <= len(data)


def is_fixture(path):
    return bool(FIXTURE_DIRS.intersection(Path(path).parts[:-1]))


def is_placeholder(value):
    v = value.strip()
    return not v or v.lower() in PLACEHOLDERS or bool(PLACEHOLDER_RE.match(v))


def mac_is_suspect(text):
    first = int(text[:2], 16)
    if first & 0x02 or first & 0x01:  # locally administered, or multicast
        return False
    return int(re.sub(r'[:-]', '', text), 16) != 0


class Report:
    def __init__(self):
        self.errors, self.warnings = [], []

    def error(self, where, msg):
        self.errors.append(f'error: {where}: {msg}')

    def warn(self, where, msg):
        self.warnings.append(f'warning: {where}: {msg}')


def check_line(rep, path, lineno, line):
    where = f'{path}:{lineno}'
    for ch, name in DASHES.items():
        if ch in line:
            rep.error(where, f'{name}')
    if is_fixture(path) or ALLOW in line:
        return
    for m in MAC.finditer(line):
        if mac_is_suspect(m.group(0)):
            rep.warn(where, f'MAC {m.group(0)} is not locally administered (looks like a real BSSID)')
    for m in SSID_QUOTED.finditer(line):
        if not is_placeholder(m.group(1)):
            rep.warn(where, f'ssid "{m.group(1)}" looks like a real network name')
    for m in SSID_ASSIGN.finditer(line):
        if not is_placeholder(m.group(2)):
            rep.warn(where, f'SSID={m.group(2)} looks like a real network name')


def check_nul(rep, path, data):
    if b'\0' not in data:
        return
    bad = [i + 1 for i, ln in enumerate(data.split(b'\n')) if b'\0' in ln]
    extra = f' ({len(bad)} lines)' if len(bad) > 1 else ''
    rep.error(f'{path}:{bad[0]}', f'NUL byte in text file{extra}')


def parse_range(rng):
    if '...' in rng:
        base, head = rng.split('...', 1)
    elif '..' in rng:
        base, head = rng.split('..', 1)
    else:
        base, head = rng, ''
    return base or 'HEAD', head or 'HEAD'


def resolve_base(root, base, explicit):
    try:
        git(root, 'rev-parse', '--verify', '--quiet', base + '^{commit}')
        return base
    except GitError:
        if not explicit and '/' not in base:
            alt = 'origin/' + base
            git(root, 'rev-parse', '--verify', '--quiet', alt + '^{commit}')
            return alt
        raise


def check_commits(rep, root, base, head):
    log = git(root, 'log', '--format=%H%x00%an%x00%ae%x00%cn%x00%ce%x00%B%x01', f'{base}..{head}')
    for rec in log.split('\x01'):
        rec = rec.strip('\n')
        if not rec:
            continue
        sha, an, ae, cn, ce, msg = rec.split('\0', 5)
        short = sha[:12]
        if (an, ae) != IDENTITY:
            rep.error(f'commit {short}', f'author is {an} <{ae}>, expected {IDENTITY[0]} <{IDENTITY[1]}>')
        if (cn, ce) != IDENTITY:
            rep.error(f'commit {short}', f'committer is {cn} <{ce}>, expected {IDENTITY[0]} <{IDENTITY[1]}>')
        for ch, name in DASHES.items():
            if ch in msg:
                rep.error(f'commit {short}', f'{name} in commit message')


HUNK = re.compile(r'^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@')


def check_range(rep, root, base, head):
    check_commits(rep, root, base, head)
    names = git(root, 'diff', '-z', '--name-only', '--diff-filter=d', '-M', f'{base}...{head}')
    text_paths = []
    for path in filter(None, names.split('\0')):
        data = git(root, 'cat-file', 'blob', f'{head}:{path}', binary=True)
        if is_text(path, data):
            text_paths.append(path)
            check_nul(rep, path, data)
    if text_paths:
        diff = git(root, 'diff', '-a', '-U0', '--no-color', '--no-ext-diff', '-M',
                   '--diff-filter=d', f'{base}...{head}', '--', *text_paths, binary=True)
        path, lineno, header = None, 0, False
        for raw in diff.split(b'\n'):
            line = raw.decode('utf-8', errors='replace')
            if line.startswith('diff --git '):
                path, header = None, True
            elif header and line.startswith('+++ '):
                path = line[6:] if line.startswith('+++ b/') else None
            elif line.startswith('@@'):
                header = False
                m = HUNK.match(line)
                lineno = int(m.group(1)) if m else 0
            elif path and not header and line.startswith('+'):
                check_line(rep, path, lineno, line[1:])
                lineno += 1
    return names.split('\0')


def check_files(rep, root, files):
    touched = []
    for f in files:
        p = Path(f)
        abspath = p if p.is_absolute() else Path.cwd() / p
        try:
            rel = abspath.resolve().relative_to(root).as_posix()
        except ValueError:
            rel = f
        try:
            data = abspath.read_bytes()
        except OSError as e:
            rep.error(f, f'cannot read: {e.strerror}')
            continue
        touched.append(rel)
        if not is_text(rel, data):
            continue
        check_nul(rep, rel, data)
        for i, line in enumerate(data.decode('utf-8', errors='replace').split('\n'), 1):
            check_line(rep, rel, i, line)
    return touched


def sync_command(root, tool):
    src = tool.read_text(errors='replace')
    if 'argparse' not in src and 'sys.argv[1]' in src:
        return [sys.executable, str(tool), str(root), '--check']
    return [sys.executable, str(tool), '--check']


def check_patches(rep, root, touched):
    patches = sorted({p for p in touched
                      if re.fullmatch(r'xnu/patches/[^/]+\.patch', p) and (root / p).exists()})
    if not patches:
        return
    tools = sorted((root / 'tools').glob('sync-*.py'))
    sources = {t: t.read_text(errors='replace') for t in tools}
    ran = {}
    for patch in patches:
        name = Path(patch).name
        for tool in [t for t in tools if name in sources[t]]:
            if tool not in ran:
                proc = subprocess.run(sync_command(root, tool), cwd=root, capture_output=True,
                                      text=True, timeout=600)
                tail = (proc.stderr.strip() or proc.stdout.strip()).splitlines()
                ran[tool] = (proc.returncode, tail[-1] if tail else '')
            rc, last = ran[tool]
            if rc:
                rel = tool.relative_to(root).as_posix()
                rep.error(patch, f'out of sync: {rel} --check exited {rc}' + (f' ({last})' if last else ''))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('range', nargs='?', help='git range (default salami..HEAD)')
    ap.add_argument('--files', nargs='+', default=[], metavar='FILE',
                    help='check these whole files (no range unless one is given)')
    ap.add_argument('--repo', help='repository root (default: the one containing the cwd)')
    ap.add_argument('--strict', action='store_true', help='warnings also fail')
    args = ap.parse_args(argv)

    rep = Report()
    try:
        root = Path(args.repo or git('.', 'rev-parse', '--show-toplevel').strip()).resolve()
        touched = []
        if args.range or not args.files:
            base, head = parse_range(args.range or 'salami..HEAD')
            base = resolve_base(root, base, explicit=args.range is not None)
            touched += check_range(rep, root, base, head)
        touched += check_files(rep, root, args.files)
        check_patches(rep, root, touched)
    except (GitError, subprocess.TimeoutExpired) as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    for line in rep.errors + rep.warnings:
        print(line)
    failed = rep.errors or (args.strict and rep.warnings)
    if failed or rep.warnings:
        print(f'check-hygiene: {len(rep.errors)} error(s), {len(rep.warnings)} warning(s)',
              file=sys.stderr)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
