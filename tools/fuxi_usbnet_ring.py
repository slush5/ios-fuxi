#!/usr/bin/env python3
"""Route U, host side: en0's Ethernet frames over the loader's USB monitor (docs-lanes/wifi-ios.md).

UsbnetTransport moves frames through the loader's two route U requests with any pyusb-style
ctrl_transfer callable: 0x5d sends one frame to the phone, 0x5e returns the ring header and
the frames the phone sent. LoaderRingSim is a byte-exact model of the loader side
(shim/fuxi-probe/usbnet.h: region bytes, both requests, the STALL and drop rules, replies of
exactly wLength bytes) and KernelSim of the kernel side (xnu/fuxi-wifi-ios/fuxi_usbnet_ring.h),
so a bridge runs end to end in WSL with no phone:

    transport, loader, kernel = sim_pair()
    kernel.tx(frame)                 # the phone sends a frame
    transport.poll()                 # -> [frame]
    transport.send_frame(reply)      # -> True (False: busy, nothing sent; retry after a poll)
    kernel.rx()                      # -> [reply]

With a phone: transport = UsbnetTransport(lambda *a: dev.ctrl_transfer(*a, timeout=500), lock)
where lock is shared with anything else using the monitor (the console loop). A STALL drops
the device from a WinUSB host until a replug (SPEC A1), so the loader STALLs only malformed
requests, which this transport never sends: a frame the phone cannot take is dropped there
and counted (out_stall_full, out_stall_other), and the transport avoids that by never having
32 frames in flight (rx_head + frames sent since the last header - rx_tail < 32) and by
sending nothing until the header shows the loader and the kernel up. Before the launch and
the handoff 0x5e answers zeros (not ready): poll() returns no frames and send_frame False.
tools/test-fuxi-usbnet.py checks both models against the C byte for byte.
`python3 tools/fuxi_usbnet_ring.py --selftest` runs an exchange over the simulator.
"""
import argparse
import errno
import struct
import threading

SIZE, ALIGN = 0x24000, 0x4000
SLOT_SIZE, SLOTS, SLOT_HDR = 2048, 32, 16
RX_OFF, TX_OFF = 0x4000, 0x14000
FRAME_MIN, FRAME_MAX = 14, 1514
IN_MIN, IN_MAX = 0xc0, 4096
VERSION = 1
MAGIC, KMAGIC = b'FXNETL01', b'FXNETK01'
TYPE_OUT, TYPE_IN = 0x40, 0xc0
REQ_OUT, REQ_IN = 0x5d, 0x5e
VALUE_OUT, VALUE_IN = 0x4e52, 0x4e53       # 'NR', 'NS'
KSTATE_UP, KSTATE_LINK = 1, 2
ADT_PATCHED, ADT_NONE, ADT_SEVERAL, ADT_REGION = 1, 2, 3, 4
ADT_NAME, PLACEHOLDER = 'fuxi-usbnet', b'FXNET-PLACEHOLDR'
# gpu-handoff.h: the largest XNU RAM window and the GPU scratch; the region avoids both.
RAM_BASE, RAM_SIZE = 0x8c0000000, 0x13e000000
SCRATCH, SCRATCH_SIZE = 0x9fe000000, 0x02000000
M32 = 0xffffffff
# u32 header fields (the magics at 0x00/0x40 and self_pa at 0x10 are 8 bytes).
FIELDS = {
    'version': 0x08, 'size': 0x0c, 'slot_size': 0x18, 'slots': 0x1c, 'rx_off': 0x20, 'tx_off': 0x24,
    'adt_status': 0x28,
    'kstate': 0x48, 'rx_tail': 0x4c, 'tx_head': 0x50, 'rx_frames': 0x54, 'rx_bad': 0x58,
    'tx_frames': 0x5c, 'tx_full_drops': 0x60, 'tx_size_drops': 0x64,
    'rx_head': 0x80, 'host_tx_tail': 0x84, 'out_ok': 0x88, 'out_stall_full': 0x8c,
    'out_stall_other': 0x90, 'in_count': 0x94, 'tail_bad': 0x98,
}
LIBUSB_ERROR_PIPE = -9


def fnv1a32(data):
    h = 2166136261
    for b in data:
        h = ((h ^ b) * 16777619) & M32
    return h


def parse_header(buf):
    """Header lines 0..2 (the first 0xc0 bytes of a 0x5e reply or of the region) as a dict."""
    if len(buf) < IN_MIN:
        raise ValueError(f'route U header of {len(buf)} bytes, want {IN_MIN}')
    h = {name: struct.unpack_from('<I', buf, off)[0] for name, off in FIELDS.items()}
    h['magic'], h['kmagic'] = bytes(buf[0:8]), bytes(buf[0x40:0x48])
    h['self_pa'] = struct.unpack_from('<Q', buf, 0x10)[0]
    h['loader_ok'] = h['magic'] == MAGIC and h['version'] == VERSION and h['size'] == SIZE
    h['kernel_up'] = h['kmagic'] == KMAGIC and bool(h['kstate'] & KSTATE_UP)
    h['link_up'] = bool(h['kstate'] & KSTATE_LINK)
    return h


def region_ok(pa):
    """usbnet.h un_region_ok: 16 KiB aligned, clear of XNU RAM at its largest and the GPU scratch."""
    end = pa + SIZE
    if not pa or pa % ALIGN or end > 1 << 64:
        return False
    if pa < RAM_BASE + RAM_SIZE and RAM_BASE < end:
        return False
    if pa < SCRATCH + SCRATCH_SIZE and SCRATCH < end:
        return False
    return True


def adt_property():
    """The 52 bytes mkadt-salami.py --usbnet adds: 32-byte name, u32 length 16, the placeholder."""
    return ADT_NAME.encode().ljust(32, b'\0') + struct.pack('<I', len(PLACEHOLDER)) + PLACEHOLDER


class Stall(OSError):
    """What pyusb raises for a STALLed control request (libusb LIBUSB_ERROR_PIPE, errno EPIPE)."""

    def __init__(self, what='STALL'):
        super().__init__(errno.EPIPE, f'Pipe error ({what})')
        self.backend_error_code = LIBUSB_ERROR_PIPE


def is_stall(error):
    return getattr(error, 'backend_error_code', None) == LIBUSB_ERROR_PIPE or getattr(error, 'errno', None) == errno.EPIPE


class Region:
    """The 0x24000 shared bytes, u32 little endian; write() is every store either side makes."""

    def __init__(self):
        self.bytes = bytearray(SIZE)

    def u32(self, off):
        return struct.unpack_from('<I', self.bytes, off)[0]

    def write(self, off, value):
        struct.pack_into('<I', self.bytes, off, value & M32)

    def bump(self, off):
        self.write(off, self.u32(off) + 1)


class LoaderRingSim:
    """The loader side of route U, byte for byte as shim/fuxi-probe/usbnet.h does it.

    pa is the region's physical address (what line 0 and the ADT get). launched is the monitor's
    MON_LAUNCH; handoff() is loader_boot's un_handoff and publishes the region."""

    def __init__(self, pa=0xa0000000, region=None):
        self.pa = pa
        self.region = region or Region()
        self.launched = False
        self.published = False
        self.adt_status = 0

    # ---- loader_boot ----
    @staticmethod
    def adt_find(adt):
        want, found, at = adt_property(), 0, 0
        o = 0
        while o + len(want) <= len(adt) and found < 2:
            if adt[o:o + len(want)] == want:
                if not found:
                    at = o
                found += 1
            o += 4
        return found, at

    def handoff(self, adt=None):
        """un_handoff: returns adt_status; adt (a bytearray, the loader's ADT copy) is patched in place."""
        if not region_ok(self.pa):
            self.adt_status = ADT_REGION
            return ADT_REGION
        r = self.region
        r.bytes[:] = bytes(SIZE)
        for off, value in ((0x08, VERSION), (0x0c, SIZE), (0x10, self.pa & M32), (0x14, self.pa >> 32),
                           (0x18, SLOT_SIZE), (0x1c, SLOTS), (0x20, RX_OFF), (0x24, TX_OFF)):
            r.write(off, value)
        r.bytes[0:8] = MAGIC
        found, at = self.adt_find(adt if adt is not None else b'')
        if found == 1:
            struct.pack_into('<QQ', adt, at + 36, self.pa, SIZE)
        self.adt_status = ADT_PATCHED if found == 1 else ADT_SEVERAL if found else ADT_NONE
        r.write(0x28, self.adt_status)
        self.published = True
        return self.adt_status

    # ---- the two requests ----
    def kernel_up(self):
        r = self.region
        return bytes(r.bytes[0x40:0x48]) == KMAGIC and bool(r.u32(0x48) & KSTATE_UP)

    def host_tail(self, index):
        r = self.region
        head = r.u32(0x50)
        t = (head & ~0xffff & M32) | index
        if t > head:
            t = (t - 0x10000) & M32
        if (head - t) & M32 <= SLOTS:
            r.write(0x84, t)
        else:
            r.bump(0x98)

    def out_ok(self, value, length):
        """0x5d setup: True starts the data stage; False is a STALL, only for a malformed
        request (counted out_stall_other once published)."""
        if value == VALUE_OUT and FRAME_MIN <= length <= FRAME_MAX:
            return True
        if self.published:
            self.region.bump(0x90)
        return False

    def out_data(self, ok, data, length, index):
        """0x5d data stage done; ok: exactly wLength bytes arrived. data[:length] is the frame.
        True: kept in the RX ring; False: dropped (counted once published)."""
        if not self.published:
            return False
        r = self.region
        other = not ok or not self.launched or not self.kernel_up() or length < FRAME_MIN or length > FRAME_MAX
        head = r.u32(0x80)
        full = (head - r.u32(0x4c)) & M32 >= SLOTS
        if other or full:
            r.bump(0x90 if other else 0x8c)
            return False
        slot = RX_OFF + head % SLOTS * SLOT_SIZE
        frame = bytes(data[:length])
        padded = frame + bytes(-length & 3)
        r.bytes[slot + SLOT_HDR:slot + SLOT_HDR + len(padded)] = padded
        struct.pack_into('<IIII', r.bytes, slot, length, head, fnv1a32(frame), 0)
        r.write(0x80, head + 1)
        self.host_tail(index)
        r.bump(0x88)
        return True

    def in_reply(self, value, index, length):
        """0x5e: exactly length bytes (all zero before the launch or the handoff), or None for
        a STALL (only wValue or wLength out of range)."""
        if value != VALUE_IN or length < IN_MIN or length > IN_MAX:
            return None
        if not self.launched or not self.published:
            return bytes(length)
        r = self.region
        self.host_tail(index)
        r.bump(0x94)
        out = bytearray(length)
        out[:IN_MIN] = r.bytes[:IN_MIN]
        head, tail = r.u32(0x50), r.u32(0x84)
        pos, k = IN_MIN, tail
        while k != head and (k - tail) & M32 < SLOTS:
            slot = TX_OFF + k % SLOTS * SLOT_SIZE
            n = r.u32(slot) & 0xffff
            if n < FRAME_MIN or n > FRAME_MAX or r.u32(slot + 4) != k:
                break
            size = SLOT_HDR + ((n + 3) & ~3)
            if size > length - pos:
                break
            entry = bytearray(r.bytes[slot:slot + size])
            entry[SLOT_HDR + n:] = bytes(size - SLOT_HDR - n)
            out[pos:pos + size] = entry
            pos += size
            k = (k + 1) & M32
        return bytes(out)

    def ctrl(self, bmRequestType, bRequest, wValue=0, wIndex=0, data_or_wLength=None, timeout=None):
        """pyusb ctrl_transfer for the two route U requests; anything else STALLs as on the phone.
        A 0x5d frame the loader drops still completes (its data stage is taken)."""
        if bmRequestType == TYPE_OUT and bRequest == REQ_OUT:
            data = bytes(data_or_wLength or b'')
            if not self.out_ok(wValue, len(data)):
                raise Stall('0x5d')
            self.out_data(True, data, len(data), wIndex)
            return len(data)
        if bmRequestType == TYPE_IN and bRequest == REQ_IN:
            reply = self.in_reply(wValue, wIndex, int(data_or_wLength or 0))
            if reply is None:
                raise Stall('0x5e')
            return reply
        raise Stall(f'request {bmRequestType:#04x}/{bRequest:#04x}')


class KernelSim:
    """The kernel side (fuxi_usbnet_ring.h, fuxi_wifi_ios.c usbnet backend) on a loader's region."""

    def __init__(self, loader):
        self.loader = loader
        self.region = loader.region
        self.kstate = 0

    def attach(self):
        self.region.bytes[0x40:0x48] = KMAGIC

    def set_state(self, bit, on):
        self.kstate = self.kstate | bit if on else self.kstate & ~bit
        self.region.write(0x48, self.kstate)

    def up(self):
        """kmagic and the RX thread: what the loader needs before it takes 0x5d frames."""
        self.attach()
        self.set_state(KSTATE_UP, True)

    def link(self, up):
        self.set_state(KSTATE_LINK, up)

    def tx(self, frame):
        """One frame to the host: 'ok', 'full' (tx_full_drops) or 'size' (tx_size_drops)."""
        r, n = self.region, len(frame)
        if n < FRAME_MIN or n > FRAME_MAX:
            r.bump(0x64)
            return 'size'
        head, tail = r.u32(0x50), r.u32(0x84)
        if (head - tail) & M32 >= SLOTS:
            r.bump(0x60)
            return 'full'
        slot = TX_OFF + head % SLOTS * SLOT_SIZE
        for i in range(0, n, 4):
            r.write(slot + SLOT_HDR + i, int.from_bytes(frame[i:i + 4].ljust(4, b'\0'), 'little'))
        for off, value in ((0, n), (4, head), (8, fnv1a32(frame)), (12, 0)):
            r.write(slot + off, value)
        r.write(0x50, head + 1)
        r.bump(0x5c)
        return 'ok'

    def rx_take(self):
        """('empty', None), ('ok', frame) or ('bad', None); rx_done consumes the last two."""
        r = self.region
        head, tail = r.u32(0x80), r.u32(0x4c)
        if head == tail:
            return 'empty', None
        slot = RX_OFF + tail % SLOTS * SLOT_SIZE
        n, want = r.u32(slot) & 0xffff, r.u32(slot + 8)
        if n < FRAME_MIN or n > FRAME_MAX or r.u32(slot + 4) != tail:
            return 'bad', None
        frame = bytes(r.bytes[slot + SLOT_HDR:slot + SLOT_HDR + n])
        if fnv1a32(frame) != want:
            return 'bad', None
        return 'ok', frame

    def rx_done(self, good):
        r = self.region
        r.write(0x4c, r.u32(0x4c) + 1)
        r.bump(0x54 if good else 0x58)

    def rx(self, most=SLOTS):
        """One pass of the RX thread: the frames for en0 (bad slots consumed and counted)."""
        frames = []
        for _ in range(most):
            got, frame = self.rx_take()
            if got == 'empty':
                break
            if got == 'ok':
                frames.append(frame)
            self.rx_done(got == 'ok')
        return frames


class UsbnetTransport:
    """Frames to and from the phone over the loader monitor's requests 0x5d and 0x5e.

    ctrl(bmRequestType, bRequest, wValue, wIndex, data_or_length) is a pyusb-style
    ctrl_transfer; lock serializes it with other users of the device (one is made if None).
    poll() -> frames the phone sent, in order ([] while the loader is not ready);
    send_frame(frame) -> True when the frame went to the loader, False when the phone is
    busy and nothing was sent (not ready, kernel not up, or 32 frames in flight: retry after
    the next poll); header() -> the decoded header lines; stats -> counters. One thread may
    poll() while another calls send_frame().

    Flow control (SPEC A1): the loader never STALLs a well-formed request, it drops (and
    counts) a frame it cannot keep. So send_frame keeps (rx_head + frames sent since the
    last header) - rx_tail below 32, refreshing the header with one 192-byte 0x5e when that
    looks full, and sends nothing until a header shows the loader magic and the kernel up
    (kmagic, kstate bit0). stats['phone_drops'] is the rise of the loader's out_stall_full +
    out_stall_other between headers (frames the phone dropped anyway, e.g. its kernel went
    down between two headers); stats['stall'] counts EPIPE on 0x5d, which a correct host
    never sees (still answered as busy, never a lost link).

    The host's TX consumed count (self.consumed) goes in wIndex of both requests, so the
    loader learns it with the next request: call header() before handing the device to
    another transport, or that one sees up to 32 frames again. A new transport, or one
    that saw the loader not ready (a phone reboot), adopts host_tx_tail from the first
    ready header. Until then it sends SYNC_INDEX, which the loader refuses (tail_bad + 1)
    unless tx_head's low 16 bits happen to lie in [SYNC_INDEX, SYNC_INDEX + 32] (33 in
    65536): then the listing starts up to 32 frames away from where the old bridge
    stopped, once, so up to 32 frames repeat or are skipped (seq and fnv are still
    checked; TCP/IP above recovers)."""

    SYNC_INDEX = 0x8000

    def __init__(self, ctrl, lock=None, in_length=IN_MAX):
        if not IN_MIN <= in_length <= IN_MAX:
            raise ValueError(f'in_length must be {IN_MIN}..{IN_MAX}')
        self.ctrl = ctrl
        self.lock = lock if lock is not None else threading.Lock()
        self.in_length = in_length
        self.consumed = None
        self.last = None
        self.unacked = 0                # frames sent since self.last was read
        self.stats = {'polls': 0, 'frames_in': 0, 'frames_out': 0, 'busy': 0, 'stall': 0, 'not_ready': 0,
                      'phone_drops': 0, 'resync': 0, 'bad_seq': 0, 'bad_fnv': 0, 'bad_entry': 0}

    def _in_locked(self, length):
        # Under self.lock: the request, the header and the bookkeeping change together.
        index = self.SYNC_INDEX if self.consumed is None else self.consumed & 0xffff
        raw = bytes(self.ctrl(TYPE_IN, REQ_IN, VALUE_IN, index, length))
        if len(raw) != length:
            raise ValueError(f'0x5e gave {len(raw)} bytes, asked {length}: not a route U loader')
        h = parse_header(raw)
        if not h['loader_ok']:
            # Zeros: before the launch or the handoff (or the phone rebooted). Start over
            # from the next ready header; the loader touched nothing.
            self.consumed, self.last, self.unacked = None, None, 0
            self.stats['not_ready'] += 1
            return raw, h
        if self.last is not None:
            dropped = h['out_stall_full'] + h['out_stall_other'] - self.last['out_stall_full'] - self.last['out_stall_other']
            self.stats['phone_drops'] += dropped & M32
        self.last, self.unacked = h, 0
        if self.consumed is None:
            self.consumed = h['host_tx_tail']   # where the loader lists from: a restarted bridge picks up there
        return raw, h

    def _in(self, length):
        with self.lock:
            return self._in_locked(length)

    def header(self):
        return dict(self._in(IN_MIN)[1])

    def _resync(self, h):
        # The header's tx_head is read before the list: up to 32 frames past it may already be
        # ours. Further off, our count is wrong (the loader refused it): take tx_head.
        if not 0 < (self.consumed - h['tx_head']) & M32 <= SLOTS:
            self.consumed = h['tx_head']
        self.stats['resync'] += 1

    def poll(self):
        with self.lock:                 # a concurrent send_frame may see the loader not ready
            return self._poll_locked()

    def _poll_locked(self):
        raw, h = self._in_locked(self.in_length)
        self.stats['polls'] += 1
        if not h['loader_ok']:
            return []
        frames, pos, bad = [], IN_MIN, None
        while pos + SLOT_HDR <= len(raw):
            n, _flags, seq, want, _zero = struct.unpack_from('<HHIII', raw, pos)
            if n == 0:
                break
            if not FRAME_MIN <= n <= FRAME_MAX or pos + SLOT_HDR + n > len(raw):
                bad = 'bad_entry'
                break
            frame = raw[pos + SLOT_HDR:pos + SLOT_HDR + n]
            if seq != self.consumed:
                bad = 'bad_seq'
                break
            if fnv1a32(frame) != want:
                bad = 'bad_fnv'
                break
            frames.append(frame)
            self.consumed = (self.consumed + 1) & M32
            pos += SLOT_HDR + ((n + 3) & ~3)
        # Nothing listed although the kernel is ahead, with room for a whole frame: the loader
        # lists from another tail (ours was refused) or its next slot is broken. Skip to tx_head.
        stuck = not frames and h['tx_head'] != self.consumed and len(raw) >= IN_MIN + SLOT_HDR + FRAME_MAX + 2
        if bad or stuck:
            self.stats[bad or 'bad_seq'] += 1
            self._resync(h)
        self.stats['frames_in'] += len(frames)
        return frames

    def _room(self):
        """The last header shows the kernel up and fewer than 32 frames in the RX ring,
        counting the frames sent since that header."""
        h = self.last
        return h is not None and h['kernel_up'] and (h['rx_head'] + self.unacked - h['rx_tail']) & M32 < SLOTS

    def send_frame(self, frame):
        frame = bytes(frame)
        if not FRAME_MIN <= len(frame) <= FRAME_MAX:
            raise ValueError(f'a frame is {FRAME_MIN}..{FRAME_MAX} bytes, not {len(frame)}')
        for fresh in (False, True):
            if fresh:
                self._in(IN_MIN)        # one fresh header: the kernel may have drained the ring
            with self.lock:
                if not self._room():
                    continue
                try:
                    self.ctrl(TYPE_OUT, REQ_OUT, VALUE_OUT, self.consumed & 0xffff, frame)
                except Exception as error:      # pyusb's USBError, or this module's Stall
                    if is_stall(error):         # never from a correct loader (SPEC A1)
                        self.stats['stall'] += 1
                        self.stats['busy'] += 1
                        return False
                    raise
                self.unacked += 1
                self.stats['frames_out'] += 1
                return True
        self.stats['busy'] += 1
        return False


def sim_pair(pa=0xa0000000, adt=None, link=True):
    """A published, launched LoaderRingSim with a KernelSim that is up (and its link), plus a
    transport on loader.ctrl: the whole route U path without a phone."""
    loader = LoaderRingSim(pa)
    loader.handoff(adt)
    loader.launched = True
    kernel = KernelSim(loader)
    kernel.up()
    kernel.link(link)
    return UsbnetTransport(loader.ctrl), loader, kernel


def selftest():
    # A bridge started early: zeros before the launch and the handoff, nothing sent, no STALL.
    loader = LoaderRingSim()
    transport = UsbnetTransport(loader.ctrl)
    assert transport.poll() == [] and not transport.send_frame(bytes(60)), 'selftest: not ready'
    loader.launched = True
    assert transport.poll() == [] and not transport.header()['loader_ok'], 'selftest: unpublished'
    loader.handoff()
    kernel = KernelSim(loader)
    assert not transport.send_frame(bytes(60)), 'selftest: kernel not up'
    kernel.up()
    kernel.link(True)
    to_host = [bytes([0xff] * 6) + bytes.fromhex('024655584901') + b'\x08\x06' + bytes([i]) * (28 + i) for i in range(40)]
    to_phone = [bytes.fromhex('024655584901') + bytes.fromhex('02aabbccdd01') + b'\x08\x00' + bytes([i]) * (46 + 30 * i)
                for i in range(40)]
    got_host, got_phone, put, sent = [], [], 0, 0
    while len(got_host) < len(to_host) or len(got_phone) < len(to_phone):
        while put < len(to_host) and kernel.tx(to_host[put]) == 'ok':
            put += 1
        got_host += transport.poll()
        while sent < len(to_phone) and transport.send_frame(to_phone[sent]):
            sent += 1
        got_phone += kernel.rx()
    assert got_host == to_host and got_phone == to_phone, 'selftest: frames differ'
    h = transport.header()
    assert transport.stats['stall'] == 0 and transport.stats['phone_drops'] == 0, 'selftest: a STALL or a drop'
    assert h['out_stall_full'] == 0 and h['out_stall_other'] == 0, 'selftest: the loader dropped frames'
    print(f'route U selftest: {len(got_host)} frames phone -> host, {len(got_phone)} host -> phone; '
          f'transport {transport.stats}; out_stall_full {h["out_stall_full"]}, '
          f'out_stall_other {h["out_stall_other"]}, in_count {h["in_count"]}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--selftest', action='store_true', help='exchange frames over LoaderRingSim/KernelSim')
    args = ap.parse_args()
    if args.selftest:
        selftest()
    else:
        ap.print_help()


if __name__ == '__main__':
    main()
