#!/system/bin/sh
# salami-ios-root: Magisk service.d boot check for the pinned iOS root on OnePlus 11 (salami).
#
# Installed by tools/salami-install-root.sh as /data/adb/service.d/salami-ios-root.sh
# (root, 0755), with its data in /data/adb/salami-ios-root/ (root only, 0700):
#   salami-pin     the static aarch64 helper (tools/salami-pin.c), run in verify mode
#   extents.json   the table the ADT and tools/run-ios-salami.py were made from
# Magisk runs it at late_start service on every Android boot, in parallel with the
# rest of the boot. Run it by hand with: su -c sh /data/adb/service.d/salami-ios-root.sh
#
# Steps:
#  1. find the dm device named "userdata" whose only slave is sda14 (by-name/userdata
#     must be sda14) and its f2fs sysfs directory /sys/fs/f2fs/<dm-N>;
#  2. raise gc_pin_file_thresh there to 65535, never lower it. f2fs clears a file's pin
#     once its GC failure count passes this threshold (default 2048; sysfs, so reset at
#     every boot) and may then move the file, after which the raw HFS+ root on sda14 is
#     no longer where the ADT says. 65535 is USHRT_MAX, the f2fs limit (the on-disk GC
#     failure counter is 16 bits);
#  3. salami-pin verify FILE SIZE (read-only open, FIEMAP): the file must still be pinned,
#     uncompressed, on userdata/sda14, with the size, digest and every extent of
#     extents.json;
#  4. write one status line to /data/local/tmp/salami-root-status.txt (0644):
#       <time> OK|WARN|FAIL <message>
#     and the same line to logcat (tag salami-ios-root).
# Safety: no block device is ever opened (nothing touches /dev/block/sda14); the only
# writes are the sysfs threshold and the status file, which goes through /data like any
# app file. Nothing is repaired here: after a FAIL do not boot iOS; recreate the root
# with tools/salami-install-root.sh.

DIR=/data/adb/salami-ios-root
HELPER=$DIR/salami-pin
TABLE=$DIR/extents.json
STATUS=/data/local/tmp/salami-root-status.txt
PART=sda14
WANT_THRESH=65535
WAIT_SECONDS=120

# The status file is built in the root-only $DIR and renamed into /data/local/tmp
# (same f2fs), so a link planted in the shell-writable directory is replaced, not followed.
finish() {
    line="$(date '+%Y-%m-%dT%H:%M:%S%z' 2>/dev/null) $1 $2"
    tmp=$DIR/status.tmp
    if mkdir -p -m 700 "$DIR" 2>/dev/null && printf '%s\n' "$line" > "$tmp" 2>/dev/null; then
        chmod 0644 "$tmp" 2>/dev/null
        chcon u:object_r:shell_data_file:s0 "$tmp" 2>/dev/null
        if [ -d "$STATUS" ] || ! mv -f "$tmp" "$STATUS" 2>/dev/null; then
            rm -f "$tmp"
        fi
    fi
    if [ "$1" = OK ]; then prio=i; else prio=w; fi
    log -t salami-ios-root -p "$prio" "$line" 2>/dev/null
    exit 0
}

# One-line JSON (whitespace removed; every value here is a number, a bool or a plain
# string without spaces) -> the value of one key.
jstr() { printf '%s' "$2" | sed -n "s/.*\"$1\":\"\([^\"]*\)\".*/\1/p"; }
jnum() { printf '%s' "$2" | sed -n "s/.*\"$1\":\(-\{0,1\}[0-9][0-9]*\).*/\1/p"; }
jext() { printf '%s' "$1" | sed -n 's/.*"extents":\(\[\[[0-9,]*\]\(,\[[0-9,]*\]\)*\]\).*/\1/p'; }
oneline() { tr -d ' \t\r\n'; }

find_dm() {
    for d in /sys/block/dm-*; do
        [ "$(cat "$d/dm/name" 2>/dev/null)" = userdata ] || continue
        set -- "$d"/slaves/*
        [ $# -eq 1 ] && [ "${1##*/}" = "$PART" ] || return 2
        echo "${d##*/}"
        return 0
    done
    return 1
}

# 1. userdata dm device and its f2fs sysfs directory
n=0
while :; do
    DM=$(find_dm)
    rc=$?
    [ $rc -eq 2 ] && finish FAIL "dm device userdata is not directly on $PART: do not boot iOS"
    [ $rc -eq 0 ] && [ -d "/sys/fs/f2fs/$DM" ] && break
    n=$((n + 2))
    [ $n -le $WAIT_SECONDS ] || finish FAIL "no f2fs userdata dm device after $WAIT_SECONDS s"
    sleep 2
done
[ "$(readlink -f /dev/block/by-name/userdata 2>/dev/null)" = "/dev/block/$PART" ] ||
    finish FAIL "by-name/userdata is not $PART: do not boot iOS"

# 2. GC pin threshold (raise only)
T=/sys/fs/f2fs/$DM/gc_pin_file_thresh
OLD=$(cat "$T" 2>/dev/null)
case $OLD in ''|*[!0-9]*) finish FAIL "cannot read $T" ;; esac
if [ "$OLD" -lt "$WANT_THRESH" ]; then
    echo "$WANT_THRESH" > "$T" 2>/dev/null
fi
NEW=$(cat "$T" 2>/dev/null)
case $NEW in ''|*[!0-9]*) NEW=$OLD ;; esac
THRESH="gc_pin_file_thresh $OLD->$NEW on $DM"

# 3. the pinned file against extents.json
[ -x "$HELPER" ] || finish FAIL "$HELPER missing (rerun tools/salami-install-root.sh); $THRESH"
[ -r "$TABLE" ] || finish FAIL "$TABLE missing (rerun tools/salami-install-root.sh); $THRESH"
WANT=$(oneline < "$TABLE")
FILE=$(jstr file "$WANT")
SIZE=$(jnum size "$WANT")
DIGEST=$(jstr digest "$WANT")
WANT_EXT=$(jext "$WANT")
case $FILE in /data/*) ;; *) finish FAIL "$TABLE names no /data file; $THRESH" ;; esac
case $FILE in *[!A-Za-z0-9/._-]*|*/.*|*//*) finish FAIL "$TABLE: unsafe file name; $THRESH" ;; esac
case $SIZE in ''|0|*[!0-9]*) finish FAIL "$TABLE has no size; $THRESH" ;; esac
case $DIGEST in ????????????????) ;; *) finish FAIL "$TABLE has no digest; $THRESH" ;; esac
[ -n "$WANT_EXT" ] || finish FAIL "$TABLE has no extents; $THRESH"

ERR=$DIR/verify.err
OUT=$("$HELPER" verify "$FILE" "$SIZE" 2>"$ERR")
rc=$?
if [ $rc -ne 0 ]; then
    finish FAIL "salami-pin verify $FILE refused ($rc): $(tr '\n' ' ' < "$ERR" | cut -c1-300); do not boot iOS; $THRESH"
fi
LIVE=$(printf '%s' "$OUT" | oneline)
[ "$(jstr file "$LIVE")" = "$FILE" ] && [ "$(jnum size "$LIVE")" = "$SIZE" ] ||
    finish FAIL "salami-pin reported another file or size; $THRESH"
case $LIVE in *'"pinned":true,'*) ;; *) finish FAIL "$FILE is no longer pinned: do not boot iOS; $THRESH" ;; esac
case $LIVE in *'"compressed":false,'*) ;; *) finish FAIL "$FILE is compressed: do not boot iOS; $THRESH" ;; esac
[ "$(jstr dm_name "$LIVE")" = userdata ] && [ "$(jstr partition "$LIVE")" = "$PART" ] &&
    [ "$(jstr dm "$LIVE")" = "$DM" ] || finish FAIL "$FILE is not on userdata/$PART ($DM); $THRESH"
LIVE_DIGEST=$(jstr digest "$LIVE")
[ "$LIVE_DIGEST" = "$DIGEST" ] && [ "$(jext "$LIVE")" = "$WANT_EXT" ] ||
    finish FAIL "$FILE extents differ from extents.json (digest $LIVE_DIGEST, recorded $DIGEST): the root moved, do not boot iOS; $THRESH"

# 4. status
COUNT=$(jnum extent_count "$LIVE")
GCF=$(jnum gc_failures "$LIVE")
case $GCF in ''|*[!0-9]*) GCF=0 ;; esac
MSG="$FILE pinned at its $COUNT recorded extents (digest $DIGEST), gc_failures $GCF, $THRESH"
[ "$NEW" -ge "$WANT_THRESH" ] || finish WARN "$MSG: could not raise the threshold to $WANT_THRESH"
[ "$GCF" -lt $((NEW / 2)) ] || finish WARN "$MSG: GC failures past half the threshold, recreate the file soon"
finish OK "$MSG"
