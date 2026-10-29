#!/usr/bin/env python3
"""Replay the journal of a journaled HFS+/HFSX volume image, as macOS does at mount.

The iOS root is never cleanly unmounted (the phone is reset), so committed
transactions can still sit in the journal. Linux's hfsplus driver ignores the
journal, so they must be written to their home blocks before a Linux mount.

Format and replay rules: Apple hfs project, tag hfs-683.120.3 (the iOS 16 era
hfs-627.100.6 differs only in the bsize == 0 check), core/hfs_journal.h
(journal_header, block_list_header, block_info) and core/hfs_journal.c
(calc_checksum, do_journal_io, journal_open, replay_journal). Line numbers below
refer to hfs-683.120.3. The journal header and block lists may be in either byte
order; the journal info block is always big-endian.

This follows replay_journal() for every journal XNU replays cleanly, including
the transactions XNU finds past the header's end (they were written to the
journal but the reset came before the header update). Where XNU itself would drop
committed transactions or replay part of a transaction, this refuses instead.

  hfs-journal-replay.py IMAGE          # check and list only (default)
  hfs-journal-replay.py IMAGE --apply  # write blocks home, mark journal empty

Volume-header helpers for the Linux edit (tools/root-edit-linux.sh), used only
with an empty journal and a clean fsck:
  --mark-unmounted   set kHFSVolumeUnmountedBit (Linux hfsplus mounts read-only
                     without it), as a clean macOS unmount leaves it
  --mark-journaled   after Linux unmounts: put lastMountedVersion back to 'HFSJ'
                     in the primary and alternate headers, so XNU sees a volume
                     last unmounted by a journaling implementation
"""
import argparse, os, struct, sys

JH_MAGIC, ENDIAN_MAGIC = 0x4A4E4C78, 0x12345678
JIB_JOURNAL_IN_FS, JIB_NEED_INIT = 0x1, 0x4            # hfs_format.h:814-816
JH_CKSUM_SIZE, BLH_CKSUM_SIZE, BINFO_SIZE = 44, 32, 16  # hfs_journal.h:135, hfs_journal.c:130
BLHDR_CHECK_CHECKSUMS, BLHDR_FIRST_HEADER = 0x1, 0x2    # hfs_journal.h:77-78


def checksum(buf):  # calc_checksum, hfs_journal.c:227-239
    c = 0
    for b in buf:
        c = ((c << 8) ^ (c + b)) & 0xFFFFFFFF
    return (~c) & 0xFFFFFFFF


class BadTxn(Exception):  # goto bad_txn_handling
    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--mark-unmounted", action="store_true")
    ap.add_argument("--mark-journaled", action="store_true")
    a = ap.parse_args()
    if a.mark_unmounted or a.mark_journaled:
        return mark(a)
    fd = os.open(a.image, os.O_RDWR if a.apply else os.O_RDONLY)
    rd = lambda off, n: os.pread(fd, n, off)

    vh = rd(1024, 512)
    sig, _, attr, _, jib = struct.unpack(">2sHI4sI", vh[:16])
    bsize = struct.unpack(">I", vh[40:44])[0]
    if sig not in (b"H+", b"HX") or not attr & 0x2000:
        sys.exit(f"not a journaled HFS+ volume (sig {sig}, attributes {attr:#x})")
    ji = rd(jib * bsize, 52)
    jflags = struct.unpack(">I", ji[:4])[0]
    joff, jsize = struct.unpack(">QQ", ji[36:52])
    if not jflags & JIB_JOURNAL_IN_FS:
        sys.exit("journal is not inside the volume")
    if jflags & JIB_NEED_INIT:
        # hfs_vfsutils.c:2599-2618 (and 2866): XNU calls journal_create, which writes a
        # fresh header; whatever the journal holds is never replayed.
        print("journal info block has kJIJournalNeedInitMask: XNU re-creates the journal at mount")
        print("JOURNAL_EMPTY")
        return

    raw = rd(joff, 512)
    for e in "<>":  # hfs_journal.c:1969-1977: either byte order, checksum over the raw bytes
        if struct.unpack(e + "I", raw[:4])[0] == JH_MAGIC:
            break
    else:
        sys.exit("bad journal magic")
    magic, endian, start, end, size, blhdr_size, cks, jhdr_size = struct.unpack(e + "IIqqqiIi", raw[:44])
    if endian != ENDIAN_MAGIC or size != jsize:
        sys.exit(f"bad journal header (endian {endian:#x}, size {size} vs {jsize})")
    zeroed = raw[:36] + b"\0" * 4 + raw[40:JH_CKSUM_SIZE]
    if checksum(zeroed) != cks:  # XNU only logs this (hfs_journal.c:1988-1993); refuse
        sys.exit("journal header checksum mismatch")
    print(f"journal {'little' if e == '<' else 'big'}-endian start {start} end {end} size {size} "
          f"blhdr {blhdr_size} jhdr {jhdr_size}")
    # journal_open sanity checks, hfs_journal.c:1961, 2019-2064: XNU fails the mount
    if (jhdr_size <= 0 or not 0 < start <= size or not 0 < end <= size or start % 512 or end % 512
            or not 256 * 1024 <= size <= 1024 ** 3 or not BLH_CKSUM_SIZE <= blhdr_size <= size):
        sys.exit("journal header fails the journal_open checks")
    # hfs_journal.c:1177-1186: wrap a pointer at the very end, empty when start == end
    if start == size:
        start = jhdr_size
    if end == size:
        end = jhdr_size
    if start == end:
        print("JOURNAL_EMPTY")
        return

    def jread(pos, n):  # do_journal_io, hfs_journal.c:341-347, 423-430: wrap to jhdr_size
        out = b""
        while n:
            if pos >= size:
                pos = jhdr_size
            k = min(n, size - pos)
            out += rd(joff + pos, k)
            pos, n = pos + k, n - k
        return out, (jhdr_size if pos >= size else pos)

    writes, final, lists, extra = replay(jread, e, start, end, size, jhdr_size, blhdr_size)
    img_size = os.fstat(fd).st_size
    for off, data in writes:
        if off + len(data) > img_size:
            sys.exit(f"block outside the image at {off}")
    print(f"{lists} block lists, {len(writes)} blocks, {sum(len(d) for _, d in writes)} bytes to replay")
    if extra:
        print(f"{extra} of them past the header end {end} (written before the reset, XNU replays them); "
              f"journal ends at {final}")
    if not a.apply:
        print("DRY_RUN (use --apply)")
        return
    for off, data in writes:  # in journal order, so later copies win (same result as XNU's co_buf)
        os.pwrite(fd, data, off)
    os.fsync(fd)
    hdr = bytearray(raw[:jhdr_size])
    struct.pack_into(e + "qq", hdr, 8, final, final)  # start = end: hfs_journal.c:1453-1456, 1510
    struct.pack_into(e + "I", hdr, 36, 0)
    struct.pack_into(e + "I", hdr, 36, checksum(bytes(hdr[:JH_CKSUM_SIZE])))
    os.pwrite(fd, bytes(hdr), joff)
    os.fsync(fd)
    print("JOURNAL_REPLAYED")


def replay(jread, e, start, end, size, jhdr_size, blhdr_size):
    """replay_journal(), hfs_journal.c:1161-1456. Returns the (byte offset, data) writes in
    journal order, the final start (= end), the number of block lists and how many of them
    lie past the header's end."""
    orig_start, orig_end = start, end
    check_past_jnl_end, in_uncharted, check_block_checksums = True, False, False
    txn_start, txn_start_extra, retries = 0, False, 0
    while True:  # restart_replay
        writes, lists, extra, last_seq, cont, past = [], 0, 0, 0, False, False
        start, visits = orig_start, 0  # past: at or after the original header end
        try:
            while check_past_jnl_end or start != end:  # :1211
                visits += 1
                if visits > 2 * (size // blhdr_size) + 2:
                    sys.exit("journal scan does not terminate")
                blhdr_off = start
                hdr, offset = jread(start, blhdr_size)  # :1215
                # :1225-1235: the block list checksum is always checked, flags or not
                if checksum(hdr[:8] + b"\0" * 4 + hdr[12:BLH_CKSUM_SIZE]) != struct.unpack_from(e + "I", hdr, 8)[0]:
                    if check_past_jnl_end and in_uncharted:  # :1245-1254: end of the extra txns
                        if cont:
                            sys.exit(f"invalid block list at {blhdr_off} where the transaction past the header "
                                     "end continues: XNU would replay part of it; refusing")
                        check_past_jnl_end, end = False, blhdr_off
                        continue
                    if blhdr_off == orig_start:  # :1259-1262
                        sys.exit(f"block list checksum mismatch at {blhdr_off}: XNU aborts the replay")
                    raise BadTxn(f"block list checksum mismatch at {blhdr_off}")
                # hfs_journal.h:63-70 + 47-61: flags at 12, binfo[0] at 16 (next pointer,
                # bsize, sequence_num at 28)
                max_blocks, num_blocks, bytes_used, _, flags = struct.unpack_from(e + "HHiIi", hdr, 0)
                nxt, _, seq = struct.unpack_from(e + "qiI", hdr, 16)
                if (last_seq and seq and seq != last_seq and seq != (last_seq + 1) & 0xFFFFFFFF):  # :1268-1286
                    txn_start = end = blhdr_off
                    txn_start_extra = past
                    if check_past_jnl_end:
                        if not in_uncharted:
                            sys.exit(f"sequence {seq} after {last_seq} at {blhdr_off} inside the committed "
                                     "journal: XNU would drop the rest of it; refusing")
                        if cont:
                            sys.exit(f"sequence break at {blhdr_off} cuts a transaction past the header end: "
                                     "XNU would replay part of it; refusing")
                        check_past_jnl_end = False
                        continue
                    raise BadTxn(f"sequence {seq} after {last_seq} at {blhdr_off}")
                last_seq = seq
                if blhdr_off >= end and start <= end and last_seq == 0:  # :1289-1298
                    check_past_jnl_end = False
                    continue
                if (max_blocks <= 0 or max_blocks > size // jhdr_size
                        or num_blocks <= 0 or num_blocks > max_blocks):  # :1302-1308
                    raise BadTxn(f"bad block list at {blhdr_off}: {num_blocks}/{max_blocks}")
                if not blhdr_size <= bytes_used <= size:  # not checked by XNU; refuse
                    sys.exit(f"bad block list at {blhdr_off}: used {bytes_used}")
                binfo = [struct.unpack_from(e + "qiI", hdr, 16 + i * BINFO_SIZE) for i in range(num_blocks)]
                for bnum, _, _ in binfo[1:]:  # :1311-1316
                    if bnum < 0 and bnum != -1:
                        raise BadTxn(f"bogus block number {bnum} at {blhdr_off}")
                if flags & BLHDR_CHECK_CHECKSUMS:  # :1323-1328 (never cleared again)
                    check_block_checksums = True
                if flags & BLHDR_FIRST_HEADER:  # :1330-1332
                    txn_start, txn_start_extra = blhdr_off, past
                for bnum, bsz, bck in binfo[1:]:  # :1337-1406
                    if bsz <= 0:  # :1344-1348 (hfs-683 only); negative is refused there too
                        raise BadTxn(f"invalid bsize {bsz} at {blhdr_off}")
                    if bnum != -1:  # killed blocks are skipped, :1350-1352
                        data, _ = jread(offset, bsz)
                        if check_block_checksums and bck != 0 and checksum(data) != bck:  # :1355-1382
                            raise BadTxn(f"block {bnum} data checksum mismatch in the block list at {blhdr_off}")
                        writes.append((bnum * jhdr_size, data))  # bnum in jhdr_size units, :1022, :2001-2017
                    offset += bsz
                    if offset >= size:
                        offset = jhdr_size + (offset - size)
                lists += 1
                extra += past
                cont = nxt != 0  # binfo[0].bnum links the next block list of the txn (:2911, :4134)
                start += bytes_used  # :1442-1450
                if start >= size:
                    start = (start % size) + jhdr_size
                if start == end:
                    in_uncharted = True
                if start == orig_end:
                    past = True
        except BadTxn as bad:  # bad_txn_handling, :1413-1440
            if txn_start == 0:
                sys.exit(f"{bad}: no good transaction before it, XNU aborts the replay")
            if not txn_start_extra:
                sys.exit(f"{bad}: XNU would replay only up to {txn_start} and drop committed transactions; refusing")
            if retries == 3:
                sys.exit(f"{bad}: repeated errors, XNU aborts the replay")
            retries += 1
            print(f"{bad}: torn transaction past the header end, replay stops at {txn_start} as in XNU")
            end, check_past_jnl_end = txn_start, False
            continue
        return writes, start, lists, extra  # :1453-1456: start == end here


def mark(a):
    fd = os.open(a.image, os.O_RDWR)
    vh = os.pread(fd, 512, 1024)
    bsize, total = struct.unpack(">II", vh[40:48])
    heads = [1024, bsize * total - 1024]  # primary, alternate (end of the volume)
    for off in heads:
        h = bytearray(os.pread(fd, 512, off))
        if h[:2] not in (b"H+", b"HX"):
            sys.exit(f"no volume header at {off}")
        if a.mark_unmounted:
            attr = struct.unpack_from(">I", h, 4)[0]
            struct.pack_into(">I", h, 4, attr | 0x100)
        if a.mark_journaled:
            if not struct.unpack_from(">I", h, 4)[0] & 0x2000:
                sys.exit("volume is not journaled")
            h[8:12] = b"HFSJ"
        os.pwrite(fd, bytes(h), off)
    os.fsync(fd)
    print("VOLUME_HEADERS", "unmounted" if a.mark_unmounted else "", "HFSJ" if a.mark_journaled else "")


if __name__ == "__main__":
    main()
