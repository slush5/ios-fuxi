#!/usr/bin/env bash
# Install the HFS+ iOS root image into a pinned file on OnePlus 11 (salami) userdata.
# Run from a PC with adb; the phone runs Android with Magisk (salami has no
# OrangeFox). Replaces the unpublished Fuxi recovery step (README.md:70).
#
#   tools/salami-install-root.sh --serial SER [options] IMAGE
#     IMAGE            HFS+ (HFSX, "HX" at byte 1024) root image, 4 KiB multiple
#     --serial SER     adb serial (or ANDROID_SERIAL / SALAMI_SERIAL)
#     --helper PATH    static aarch64 salami-pin (default tools/out/salami-pin)
#     --out DIR        extents.json + install.json (default research/salami/pinfile-01)
#     --file PATH      pinned file on the phone (default /data/ios-root.img)
#     --reuse          the pinned file exists (same pinned size): verify it, rewrite the image
#     --keep-staging   keep the staged copy in /data/local/tmp after success
#     --yes            do not ask before the raw write
#   Env: ADB (default adb), PYTHON (default python3).
#
# WHY RAW dd: salami userdata is f2fs on dm-default-key (metadata encryption,
# aes-256-xts, wrappedkey_v0) as ONE target over sda14 at offset 0 with 4096-byte
# sectors (measure/dm_userdata.txt). Anything written THROUGH /data/ios-root.img,
# FBE or not, reaches UFS as ciphertext under the metadata key, but XNU (patch
# 0088 extent root) reads and writes sda14 with plain READ(10)/WRITE(10) and no
# ICE. So the file only reserves pinned blocks (salami-pin: pin, fallocate,
# FIEMAP), and the image is written RAW into /dev/block/sda14 at the file's
# extents, below dm-default-key. Reading the file through /data afterwards shows
# garbage; that is expected. Never write the file through /data, and never run
# sweet-log-pull.sh (the tail of sda14 is live encrypted f2fs space).
#
# PINNED SIZE: f2fs fallocates a pinned file in whole sections (2 MiB here), and a
# section tail past EOF would leave FIEMAP's last in-range extent without
# FIEMAP_EXTENT_LAST (salami-pin then refuses). So the pinned file is the image size
# rounded up to 2 MiB; extents.json records that size plus root_bytes = the image
# size, which mkadt-salami.py puts in fuxi-root-blocks (XNU maps only those blocks)
# and tools/run-ios-salami.py checks. Only the image's blocks are written.
#
# Steps: checks (device OP594DL1, su, by-name/userdata = sda14, dmctl table =
# one default-key target over sda14 at offset 0, free space) -> push salami-pin
# -> salami-pin create (pin, fallocate, FIEMAP -> extents.json) -> push the image
# to /data/local/tmp and compare sha256 -> re-check the pin by digest -> raw dd per
# extent (bs=4096 skip/seek/count, conv=notrunc,fsync; every extent checked
# against sda14 on the PC and again on the phone) -> sync + drop_caches -> verify
# the first block (HX at 1024) and a sha256 of all raw image blocks against the
# image -> salami-pin verify again -> install the Magisk boot check -> remove the
# staged copy.
# BOOT CHECK: tools/salami-magisk-service.sh goes to /data/adb/service.d/salami-ios-root.sh
# (0755) with salami-pin and extents.json in /data/adb/salami-ios-root/ (root only).
# At every Android boot it raises gc_pin_file_thresh to 65535 (f2fs unpins a file
# after that many GC failures and may then move it), re-verifies the pin and extents,
# and writes /data/local/tmp/salami-root-status.txt. tools/run-ios-salami.py still
# re-checks the extents itself before every boot.
set -euo pipefail
export MSYS_NO_PATHCONV=1 MSYS2_ARG_CONV_EXCL='*'   # Git Bash: keep /data/... paths intact

REPO=$(cd "$(dirname "$0")/.." && pwd)
ADB=${ADB:-adb}
PY=${PYTHON:-python3}
SERIAL=${ANDROID_SERIAL:-${SALAMI_SERIAL:-}}
HELPER_LOCAL=$REPO/tools/out/salami-pin
SERVICE_LOCAL=$REPO/tools/salami-magisk-service.sh
OUT=$REPO/research/salami/pinfile-01
FILE=/data/ios-root.img
REUSE=0
KEEP_STAGING=0
YES=0
IMAGE=

EXPECT_DEVICE=OP594DL1
PART=sda14
PART_SECTORS=456781056          # 512 B sectors = 57097632 x 4 KiB (GPT 4234843..61332474)
SECTION=2097152                 # f2fs section (measured): pinned fallocate granularity
MAX_EXTENTS=512                 # FX_MAX_EXTENTS, xnu/fuxi-unixnu/sweet_ufs.c
ADT_EXTENT_BUDGET=190           # ~24 B each inside the 16 KiB ADT next to /product
FREE_MARGIN_GIB=16              # keep f2fs roomy so GC does not fight the pin
PHONE_TMP=/data/local/tmp
PHONE_HELPER=$PHONE_TMP/salami-pin
STAGE=$PHONE_TMP/ios-root.stage.img
WRITER=$PHONE_TMP/salami-root-write.sh
HASHER=$PHONE_TMP/salami-root-hash.sh
BOOT_DIR=/data/adb/salami-ios-root
BOOT_SERVICE=/data/adb/service.d/salami-ios-root.sh
BOOT_STATUS=$PHONE_TMP/salami-root-status.txt
SERVICE_STAGE=$PHONE_TMP/salami-ios-root.sh.stage
TABLE_STAGE=$PHONE_TMP/salami-extents.json.stage

die() { printf 'salami-install-root: %s\n' "$*" >&2; exit 1; }
say() { printf '== %s\n' "$*"; }
usage() { sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; }
need() { [ "$2" -ge 2 ] || die "$1 needs a value"; }

while [ $# -gt 0 ]; do
    case $1 in
        --serial) need "$1" $#; SERIAL=$2; shift 2 ;;
        --helper) need "$1" $#; HELPER_LOCAL=$2; shift 2 ;;
        --out) need "$1" $#; OUT=$2; shift 2 ;;
        --file) need "$1" $#; FILE=$2; shift 2 ;;
        --reuse) REUSE=1; shift ;;
        --keep-staging) KEEP_STAGING=1; shift ;;
        --yes) YES=1; shift ;;
        -h|--help) usage; exit 0 ;;
        -*) die "unknown option $1" ;;
        *) [ -z "$IMAGE" ] || die "only one IMAGE"; IMAGE=$1; shift ;;
    esac
done
[ -n "$IMAGE" ] || { usage >&2; exit 1; }
[ -n "$SERIAL" ] || die "give --serial (or ANDROID_SERIAL); the serial stays out of git"
[ -f "$IMAGE" ] || die "$IMAGE: no such image"
[ -f "$HELPER_LOCAL" ] || die "$HELPER_LOCAL missing; build it:
  source ~/op11-ios/toolchain/env.sh && \$ZIG cc -target aarch64-linux-musl -static -O2 -s -o tools/out/salami-pin tools/salami-pin.c"
[ -f "$SERVICE_LOCAL" ] || die "$SERVICE_LOCAL missing (the Magisk boot check)"
# Directly under /data/: unencrypted by FBE, plain characters (it goes into phone shell lines).
[[ $FILE =~ ^/data/[A-Za-z0-9_][A-Za-z0-9._-]*$ ]] || die "--file must be /data/NAME with plain characters"
command -v "$PY" >/dev/null || die "$PY not found"
command -v "$ADB" >/dev/null || die "$ADB not found"

TMPD=$(mktemp -d)
trap 'rm -rf "$TMPD"' EXIT
PYTOOL=$TMPD/salami_root.py

# Host-side parsing and validation, independent of the phone helper's own checks.
cat > "$PYTOOL" <<'PYEOF'
import hashlib, json, os, re, struct, sys

BLK = 4096


def fnv(size, extents):
    h = 0xcbf29ce484222325
    for v in [size] + [x for e in extents for x in e]:
        for b in struct.pack('<Q', v):
            h = ((h ^ b) * 0x100000001b3) & 0xffffffffffffffff
    return f'{h:016x}'


def die(msg):
    sys.exit('salami-install-root: ' + msg)


def image(path, section):
    size, section = os.path.getsize(path), int(section)
    if size <= 0 or size % BLK:
        die(f'{path}: size {size} is not a nonzero multiple of 4096')
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        head = f.read(BLK)
        if head[1024:1026] != b'HX':
            die(f'{path}: no HFSX signature "HX" at byte 1024 (got {head[1024:1026]!r})')
        h.update(head)
        for chunk in iter(lambda: f.read(1 << 22), b''):
            h.update(chunk)
    # The pinned file is whole f2fs sections; the image is its first `size` bytes.
    print(size, h.hexdigest(), -(-size // section) * section)


def elf(path):
    b = open(path, 'rb').read()
    if b[:4] != b'\x7fELF' or b[4] != 2 or b[5] != 1:
        die(f'{path}: not a little-endian ELF64')
    machine, = struct.unpack_from('<H', b, 18)
    if machine != 183:
        die(f'{path}: e_machine {machine}, expected 183 (AArch64)')
    phoff, = struct.unpack_from('<Q', b, 32)
    phentsize, phnum = struct.unpack_from('<HH', b, 54)
    for i in range(phnum):
        if struct.unpack_from('<I', b, phoff + i * phentsize)[0] == 3:
            die(f'{path}: dynamically linked (PT_INTERP); build with -static')


def dm(table_path, part_dev, part_sectors):
    """dmctl table userdata: exactly one default-key target over the partition at offset 0."""
    lines = [l.strip() for l in open(table_path) if re.match(r'^\s*\d+-\d+:', l)]
    if len(lines) != 1:
        die(f'userdata has {len(lines)} dm targets, expected 1: {lines}')
    m = re.match(r'^(\d+)-(\d+):\s*(\S+?),\s*(.*)$', lines[0])
    if not m:
        die(f'cannot parse dm target: {lines[0]}')
    start, end, kind, params = int(m[1]), int(m[2]), m[3], m[4].split()
    # default-key: <cipher> <key> <iv_offset> <device> <start> [<#opt> <opts>...]
    if kind != 'default-key' or len(params) < 5:
        die(f'userdata target is {kind!r}, expected default-key: {lines[0]}')
    if start != 0 or end != int(part_sectors):
        die(f'dm target covers {start}-{end}, the partition has {part_sectors} sectors')
    if params[3] not in (part_dev, '/dev/block/sda14') or params[4] != '0':
        die(f'dm target maps {params[3]} at offset {params[4]}, expected {part_dev} at 0')
    if 'sector_size:4096' not in params:
        print('warning: no sector_size:4096 in the dm table', file=sys.stderr)
    print('ok', ' '.join(params[:5]))


def df_avail(path):
    rows = [l.split() for l in open(path) if l.strip()]
    row = rows[-1]
    print(int(row[-3]) * 1024)   # df -k: ... Available Use% Mounted-on


def accept(helper_json, out_json, file, size, root_bytes, section, part_bytes, max_ext, budget):
    """Validate salami-pin output on the PC and write extents.json (atomic).

    size = pinned file (whole sections), root_bytes = image (<= size), recorded so
    mkadt-salami.py and run-ios-salami.py agree on fuxi-root-blocks."""
    t = json.load(open(helper_json))
    size, root_bytes, part_bytes = int(size), int(root_bytes), int(part_bytes)
    if size % int(section) or not 0 < root_bytes <= size or root_bytes % BLK:
        die(f'pinned size {size} / image size {root_bytes}: not whole sections holding a 4 KiB-aligned image')
    if t.get('tool') != 'salami-pin' or t.get('file') != file or t.get('size') != size:
        die(f'helper output is for {t.get("file")} / {t.get("size")}, expected {file} / {size}')
    if t.get('pinned') is not True or t.get('compressed') is not False:
        die('helper says the file is not pinned or is compressed')
    if t.get('dm_name') != 'userdata' or t.get('partition') != 'sda14' or t.get('partition_bytes') != part_bytes:
        die(f'helper saw {t.get("dm_name")}/{t.get("partition")}/{t.get("partition_bytes")}, expected userdata/sda14/{part_bytes}')
    ext, pos, spans = t['extents'], 0, []
    if not 0 < len(ext) <= int(max_ext):
        die(f'{len(ext)} extents (limit {max_ext}); free space on /data, delete the file and retry')
    for i, (lo, ph, ln) in enumerate(ext):
        if lo != pos or ln <= 0 or lo % BLK or ph % BLK or ln % BLK or ph <= 0:
            die(f'extent {i} {lo},{ph},{ln} is not a contiguous 4 KiB extent')
        if ph + ln > part_bytes:
            die(f'extent {i} ends at {ph + ln}, past sda14 ({part_bytes} bytes)')
        spans.append((ph, ph + ln, i))
        pos += ln
    if pos != size:
        die(f'extents cover {pos} bytes, the pinned file is {size}')
    spans.sort()
    for (_, e0, i), (s1, _, j) in zip(spans, spans[1:]):
        if s1 < e0:
            die(f'extents {i} and {j} overlap')
    if fnv(size, ext) != t.get('digest'):
        die('extent digest mismatch between helper and PC')
    if len(ext) > int(budget):
        print(f'warning: {len(ext)} extents may not fit the 16 KiB ADT next to /product '
              f'(mkadt-salami.py refuses if not)', file=sys.stderr)
    t['root_bytes'] = root_bytes
    tmp = out_json + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(t, f, indent=1)
        f.write('\n')
    os.replace(tmp, out_json)
    print(len(ext), t['digest'], t.get('gc_failures'), t.get('gc_pin_file_thresh'))


def same(helper_json, extents_json):
    a, b = json.load(open(helper_json)), json.load(open(extents_json))
    for k in ('file', 'size', 'extents', 'digest'):
        if a.get(k) != b.get(k):
            die(f'pinned file changed since extents.json was written ({k} differs)')
    if a.get('pinned') is not True or a.get('compressed') is not False:
        die('pinned file lost its pin or became compressed')
    print('same', a['digest'], 'gc_failures', a.get('gc_failures'))


def scripts(extents_json, writer, hasher, file, helper, stage, part_sectors):
    t = json.load(open(extents_json))
    part_blocks = int(part_sectors) // 8
    root_blocks = t['root_bytes'] // BLK
    # Phone sh (mksh) arithmetic is 32-bit: only 4 KiB block numbers (< 2^31) go there.
    rows = [(lo // BLK, ph // BLK, ln // BLK) for lo, ph, ln in t['extents']]
    for s, p, c in rows:
        if not (0 < p and 0 < c and p + c <= part_blocks < 2**31):
            die(f'extent at block {p}+{c} is outside sda14 ({part_blocks} blocks)')
    # Only the image's blocks: the pinned tail past root_bytes is never written or hashed.
    rows = [(s, p, min(c, root_blocks - s)) for s, p, c in rows if s < root_blocks]
    if sum(c for _, _, c in rows) != root_blocks:
        die('extents do not cover the image')
    head = ['#!/system/bin/sh', '# generated by tools/salami-install-root.sh', 'set -e',
            'DEV=/dev/block/sda14',
            '[ "$(readlink -f /dev/block/by-name/userdata)" = "$DEV" ] || { echo "userdata is not $DEV"; exit 1; }',
            f'[ "$(cat /sys/class/block/sda14/size)" = "{part_sectors}" ] || {{ echo "sda14 size changed"; exit 1; }}',
            f'{helper} check {file} {t["size"]} {t["digest"]} || {{ echo "pinned file moved: refusing"; exit 1; }}',
            'ok() { [ "$2" -gt 0 ] && [ "$3" -gt 0 ] && [ $(( $2 + $3 )) -le ' + str(part_blocks) + ' ] || { echo "extent $2+$3 outside sda14"; exit 1; }; }']
    w = head + [f'SRC={stage}',
                'w() {', '  ok "$@"',
                '  out=$(dd if="$SRC" of="$DEV" bs=4096 skip=$1 seek=$2 count=$3 conv=notrunc,fsync 2>&1) || { echo "$out"; exit 1; }',
                '}']
    w += [f'w {s} {p} {c}' for s, p, c in rows]
    w += ['sync', 'echo 3 > /proc/sys/vm/drop_caches', 'echo SALAMI-WRITE-OK']
    h = head + ['set -o pipefail', 'sync', 'echo 3 > /proc/sys/vm/drop_caches',
                'r() { ok "$@"; dd if="$DEV" bs=4096 skip=$2 count=$3 2>/dev/null; }',
                '{']
    h += [f'  r {s} {p} {c}' for s, p, c in rows]
    h += ['} | sha256sum']
    for path, lines in ((writer, w), (hasher, h)):
        with open(path, 'w', newline='\n') as f:
            f.write('\n'.join(lines) + '\n')
    print(rows[0][1])


def first(raw_path, image_path):
    raw = open(raw_path, 'rb').read()
    with open(image_path, 'rb') as f:
        img = f.read(BLK)
    if len(raw) != BLK or raw[1024:1026] != b'HX':
        die(f'first extent block on sda14 has no HX at 1024 (got {raw[1024:1026]!r}, {len(raw)} bytes)')
    if raw != img:
        die('first extent block on sda14 differs from the image')
    print('HX ok')


def manifest(out_json, image_path, size, sha, extents_json):
    t = json.load(open(extents_json))
    json.dump({'image': os.path.basename(image_path), 'size': int(size), 'sha256': sha,
               'file': t['file'], 'pinned_bytes': t['size'], 'root_bytes': t['root_bytes'],
               'extents': len(t['extents']), 'digest': t['digest'],
               'partition': 'sda14', 'written': 'raw dd below dm-default-key',
               'boot_check': '/data/adb/service.d/salami-ios-root.sh'},
              open(out_json, 'w'), indent=1)


getattr(sys.modules[__name__], sys.argv[1])(*sys.argv[2:])
PYEOF

pyt() { "$PY" "$PYTOOL" "$@"; }
adbc() { "$ADB" -s "$SERIAL" "$@"; }
pshell() { adbc shell "$1" | tr -d '\r'; }
psu() {
    case $1 in *"'"*) die "internal: quote in su command" ;; esac
    adbc shell "su -c '$1'" | tr -d '\r'
}
sha_local() { "$PY" -c 'import hashlib,sys; print(hashlib.sha256(open(sys.argv[1],"rb").read()).hexdigest())' "$1"; }

say "host checks"
read -r SIZE SHA PIN_SIZE < <(pyt image "$IMAGE" "$SECTION") || die "image check failed"
[ -n "${PIN_SIZE:-}" ] || die "image check failed"
pyt elf "$HELPER_LOCAL"
bash -n "$SERVICE_LOCAL" || die "$SERVICE_LOCAL: syntax error"
echo "image $IMAGE: $SIZE bytes, sha256 $SHA; pinned file $PIN_SIZE bytes (whole 2 MiB f2fs sections)"

say "phone checks"
[ "$(adbc get-state 2>/dev/null | tr -d '\r')" = device ] || die "adb state is not 'device' (boot Android with Magisk)"
[ "$(pshell 'getprop ro.product.device')" = "$EXPECT_DEVICE" ] || die "ro.product.device is not $EXPECT_DEVICE"
[ "$(psu 'id -u')" = 0 ] || die "su did not give uid 0 (grant Magisk root to the shell)"
[ "$(psu 'readlink -f /dev/block/by-name/userdata')" = /dev/block/$PART ] || die "by-name/userdata is not $PART"
SECTORS=$(psu "cat /sys/class/block/$PART/size") || die "cannot read $PART size"
[ "$SECTORS" = "$PART_SECTORS" ] || die "$PART has $SECTORS sectors, measured $PART_SECTORS"
PART_DEV=$(psu "cat /sys/class/block/$PART/dev") || die "cannot read $PART dev"
psu 'dmctl table userdata' > "$TMPD/dm.txt" || die "dmctl table userdata failed"
pyt dm "$TMPD/dm.txt" "$PART_DEV" "$PART_SECTORS"
PART_BYTES=$("$PY" -c "print($PART_SECTORS * 512)")
pshell 'df -k /data' > "$TMPD/df.txt" || die "df /data failed"
AVAIL=$(pyt df_avail "$TMPD/df.txt")
NEED=$("$PY" -c "print((0 if $REUSE else $PIN_SIZE) + $SIZE + $FREE_MARGIN_GIB * 2**30)")
[ "$AVAIL" -ge "$NEED" ] || die "/data has $AVAIL bytes free, need $NEED (pinned file + staging + margin)"
EXISTS=$(psu "if [ -e $FILE ]; then echo yes; else echo no; fi") || die "su failed"
if [ "$REUSE" = 1 ]; then
    [ "$EXISTS" = yes ] || die "--reuse but $FILE does not exist"
else
    [ "$EXISTS" = no ] || die "$FILE exists; use --reuse to rewrite it, or remove it on the phone first"
fi

say "push salami-pin"
adbc push "$HELPER_LOCAL" "$PHONE_HELPER" >/dev/null
pshell "chmod 755 $PHONE_HELPER"

mkdir -p "$OUT"
EXT=$OUT/extents.json
[ ! -e "$EXT" ] || cp "$EXT" "$EXT.prev.$(date +%Y%m%d-%H%M%S)"
if [ "$REUSE" = 1 ]; then
    say "verify existing pinned file"
    psu "$PHONE_HELPER verify $FILE $PIN_SIZE" > "$TMPD/pin.json" \
        || die "salami-pin verify refused (the pinned file must be $PIN_SIZE bytes; see above)"
else
    say "create pinned file ($PIN_SIZE bytes for the $SIZE-byte image)"
    psu "$PHONE_HELPER create $FILE $PIN_SIZE" > "$TMPD/pin.json" || die "salami-pin create refused (see above)"
fi
read -r NEXT DIGEST GCF THRESH < <(pyt accept "$TMPD/pin.json" "$EXT" "$FILE" "$PIN_SIZE" "$SIZE" "$SECTION" \
    "$PART_BYTES" "$MAX_EXTENTS" "$ADT_EXTENT_BUDGET") || die "extent table refused"
[ -n "${DIGEST:-}" ] || die "extent table refused"
echo "$NEXT extents, digest $DIGEST, gc failures $GCF, gc_pin_file_thresh $THRESH, root_bytes $SIZE -> $EXT"

say "stage the image on the phone"
adbc push "$IMAGE" "$STAGE"
STAGED=$(psu "sha256sum $STAGE" | awk '{print $1}') || die "sha256sum of the staged copy failed"
[ "$STAGED" = "$SHA" ] || die "staged copy sha256 $STAGED != $SHA"

say "re-check the pin"
psu "$PHONE_HELPER verify $FILE $PIN_SIZE" > "$TMPD/pin2.json" || die "salami-pin verify refused"
pyt same "$TMPD/pin2.json" "$EXT"

FIRST=$(pyt scripts "$EXT" "$TMPD/write.sh" "$TMPD/hash.sh" "$FILE" "$PHONE_HELPER" "$STAGE" "$PART_SECTORS") \
    || die "phone script generation refused"
adbc push "$TMPD/write.sh" "$WRITER" >/dev/null
adbc push "$TMPD/hash.sh" "$HASHER" >/dev/null

echo "About to write $SIZE bytes RAW into /dev/block/$PART at the first $SIZE bytes of $NEXT extents (first block $FIRST)."
echo "Only blocks of the pinned $FILE are touched; each extent is checked against $PART."
if [ "$YES" != 1 ]; then
    [ -t 0 ] || die "not interactive: pass --yes to write"
    read -r -p "Type WRITE to continue: " ANSWER
    [ "$ANSWER" = WRITE ] || die "aborted, nothing written"
fi

say "raw write"
RESULT=$(psu "sh $WRITER") || { printf '%s\n' "$RESULT" >&2; die "raw write failed"; }
printf '%s\n' "$RESULT" | grep -qx SALAMI-WRITE-OK || { printf '%s\n' "$RESULT" >&2; die "raw write did not finish"; }

say "verify"
adbc exec-out "su -c 'dd if=/dev/block/$PART bs=4096 skip=$FIRST count=1 2>/dev/null'" > "$TMPD/first.bin"
pyt first "$TMPD/first.bin" "$IMAGE"
RAWSHA=$(psu "sh $HASHER" | awk 'END {print $1}') || die "raw read-back failed"
[ "$RAWSHA" = "$SHA" ] || die "raw extents sha256 $RAWSHA != image $SHA"
echo "raw extents sha256 matches the image"
psu "$PHONE_HELPER verify $FILE $PIN_SIZE" > "$TMPD/pin3.json" || die "salami-pin verify refused after the write"
pyt same "$TMPD/pin3.json" "$EXT"

say "install the Magisk boot check"
# Root-only copies: the boot script must not run a helper the shell user can replace.
adbc push "$SERVICE_LOCAL" "$SERVICE_STAGE" >/dev/null
adbc push "$EXT" "$TABLE_STAGE" >/dev/null
psu "mkdir -p $BOOT_DIR /data/adb/service.d && chmod 700 $BOOT_DIR && \
cp $PHONE_HELPER $BOOT_DIR/salami-pin.new && cp $TABLE_STAGE $BOOT_DIR/extents.json.new && \
cp $SERVICE_STAGE $BOOT_SERVICE.new && \
chown 0:0 $BOOT_DIR/salami-pin.new $BOOT_DIR/extents.json.new $BOOT_SERVICE.new && \
chmod 700 $BOOT_DIR/salami-pin.new && chmod 600 $BOOT_DIR/extents.json.new && chmod 755 $BOOT_SERVICE.new && \
mv -f $BOOT_DIR/salami-pin.new $BOOT_DIR/salami-pin && mv -f $BOOT_DIR/extents.json.new $BOOT_DIR/extents.json && \
mv -f $BOOT_SERVICE.new $BOOT_SERVICE && rm -f $SERVICE_STAGE $TABLE_STAGE" || die "boot check install failed"
WANT_SUMS="$(sha_local "$HELPER_LOCAL") $(sha_local "$EXT") $(sha_local "$SERVICE_LOCAL")"
GOT_SUMS=$(psu "sha256sum $BOOT_DIR/salami-pin $BOOT_DIR/extents.json $BOOT_SERVICE" | awk '{printf "%s ", $1}')
[ "${GOT_SUMS% }" = "$WANT_SUMS" ] || die "boot check files on the phone differ from the PC copies"
psu "rm -f $BOOT_STATUS; sh $BOOT_SERVICE" >/dev/null || die "boot check did not run"
BOOT_LINE=$(psu "cat $BOOT_STATUS") || die "boot check wrote no $BOOT_STATUS"
echo "boot check: $BOOT_LINE"
# "<time> OK|WARN|FAIL <message>"
BOOT_KIND=$(printf '%s\n' "$BOOT_LINE" | awk '{for (i = 1; i <= 2; i++) if ($i ~ /^(OK|WARN|FAIL)$/) { print $i; exit }}')
case $BOOT_KIND in
    OK) ;;
    WARN) echo "warning: the boot check warns (see above)" >&2 ;;
    *) die "the boot check fails on this phone (the root itself was written and verified): $BOOT_LINE" ;;
esac

say "clean up"
if [ "$KEEP_STAGING" = 1 ]; then
    psu "rm -f $WRITER $HASHER"
    echo "staged copy kept at $STAGE"
else
    psu "rm -f $STAGE $WRITER $HASHER"
fi
pyt manifest "$OUT/install.json" "$IMAGE" "$SIZE" "$SHA" "$EXT"

say "done"
echo "extents: $EXT (pinned $PIN_SIZE bytes, root_bytes $SIZE)"
echo "boot check: $BOOT_SERVICE -> $BOOT_STATUS at every Android boot"
echo "next: python3 tools/mkadt-salami.py OUT.adt --live-fdt LIVE.dtb --smp --gpu-reserve \\"
echo "        --root-extents $EXT --root-bytes $SIZE"
echo "before every boot: python3 tools/run-ios-salami.py ... --extents $EXT"
