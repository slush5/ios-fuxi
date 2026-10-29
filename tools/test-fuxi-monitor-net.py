#!/usr/bin/env python3
"""fuxi-xnu-monitor.py --net without a phone (no pyusb: fake usb.core/usb.util modules).

The fake device plays the loader monitor: the pre-handoff reports, 0x46 status, 0x47 RAM
reads (early status, console counter and ring), 0x48, 0x54, 0x59 (STALL), and route U
0x5d/0x5e through fuxi_usbnet_ring.LoaderRingSim with a KernelSim as the phone's kernel.

1. Without --net the monitor sends byte for byte the same requests as the committed
   tools/fuxi-xnu-monitor.py (git HEAD), in a deterministic run that ends at stage 99.
2. With --net: the bridge thread answers ARP, DHCP and pings from the KernelSim; the
   device is then unplugged (every transfer on the old handle fails, nothing enumerates
   for a while): the bridge pauses, the monitor's reconnect() finds the new handle, the
   bridge resumes on it with its state (the pings sent during the outage are answered);
   the bridge thread sends only 0x5d/0x5e (never 0x49/0x4e/0x4f/0x55/0x57), the monitor
   thread never sends them; the console log is complete across the reconnect; the 0x54
   renewal runs again after the reconnect; periodic 'net:' stats reach the output.
3. A loader without FUXI_USBNET (0x5d/0x5e STALL): one STALLed 0x5e, then the bridge is
   off for good (no retry after console polls or a reconnect); the console run goes on.
4. Ctrl+C in the console loop: the bridge is paused and stopped and its summary recorded
   before the end-of-session 0x59; no bridge request after the interrupt.
5. RingTransport alone: a 0x5d STALL (which the ring module reports as busy) disables the
   transport instead of a retry loop; with frames queued and the phone's kernel down the
   transport polls slowly and drops the stale frames, then works again once it is back.

    python tools/test-fuxi-monitor-net.py        (Windows venv or Linux, Python 3.10+)
"""
import contextlib
import errno
import importlib.util
import io
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import fuxi_usbnet_ring as R  # noqa: E402

RESULTS = []
VIRT = 0xfffffe0007004000
OFF_EARLY, OFF_TOTAL, OFF_RING = 0x10000, 0x11000, 0x100000
PHONE_MAC = bytes.fromhex('02aabbccdd01')
GW_MAC = bytes.fromhex('0246555849fe')
PHONE_IP, GW_IP = bytes([192, 168, 88, 2]), bytes([192, 168, 88, 1])


def check(name, ok, detail=''):
    RESULTS.append((name, bool(ok)))
    print(('PASS  ' if ok else 'FAIL  ') + name + ('' if ok or not detail else f'\n      {detail}'), flush=True)


# ---------------------------------------------------------------- fake pyusb
class USBError(IOError):
    def __init__(self, strerror, error_code=None, errno=None):
        IOError.__init__(self, errno, strerror)
        self.backend_error_code = error_code


class FakePhone:
    """The loader monitor and the phone behind every fake device handle."""

    def __init__(self, end_after_polls=None):
        self.mem = threading.Lock()          # the region: loader requests vs the kernel side
        self.loader = R.LoaderRingSim()
        self.kernel = R.KernelSim(self.loader)
        self.stage = 0
        self.gen = 0
        self.unplugged = False
        self.log = []                        # (thread, gen, bmRequestType, bRequest, wValue, wIndex, data/len, timeout)
        self.console = bytearray(b'FUXI console line 0\n')
        self.polls_after_handoff = 0
        self.end_after_polls = end_after_polls
        self.dead_handle_requests = 0
        self.fail_bridge = 0                 # fail this many next bridge-thread requests (a timeout)
        self.no_usbnet = False               # a loader built without FUXI_USBNET: 0x5d/0x5e STALL
        self.interrupt_main = False          # the main thread's next 0x46 raises KeyboardInterrupt
        self.interrupt_at = None             # len(log) when it did

    def device(self):
        return FakeDev(self, self.gen)

    def unplug(self):
        self.unplugged = True
        self.gen += 1

    def replug(self):
        self.unplugged = False

    def status(self):
        fields = [0, 1, 0x100, 1, self.stage, 0x1234000, 0x5678000, 0, 0x9000000, 0x9100000]
        return b'FXIMON01' + struct.pack('<10Q', *fields)

    def ram(self, off, length):
        if OFF_EARLY <= off < OFF_EARLY + 128:
            b = struct.pack('<Q', 42) + b'kernel up'.ljust(120, b'\0')
            return b[off - OFF_EARLY:off - OFF_EARLY + length]
        if off == OFF_TOTAL:
            return struct.pack('<Q', len(self.console))[:length]
        if off >= OFF_RING:
            i = off - OFF_RING
            return bytes(self.console[i:i + length]).ljust(length, b'\0')
        return bytes(length)

    def answer(self, bm, req, value, index, data_or_length):
        if bm == 0xc0:
            n = int(data_or_length)
            if req == 0x40:
                return (b'FXIPROBE' + bytes(144))[:n]
            if req == 0x45:
                return struct.pack('<10Q', 0, 1, 0, 1, 0, 0x100, 0, 0, 0, 0)[:n]
            if req == 0x46:
                if self.interrupt_main and threading.current_thread() is threading.main_thread():
                    self.interrupt_main = False
                    self.interrupt_at = len(self.log)
                    raise KeyboardInterrupt
                if self.stage >= 8 and self.stage != 99:
                    self.polls_after_handoff += 1
                    if self.end_after_polls and self.polls_after_handoff > self.end_after_polls:
                        self.stage = 99
                return self.status()[:n]
            if req == 0x47:
                return self.ram(value | index << 16, n)
            if req == 0x48:
                return (b'test loader note' if self.stage != 99 else b'test end').ljust(n, b'\0')[:n]
            if req == 0x49:
                self.stage = 8
                with self.mem:
                    self.loader.handoff()
                    self.loader.launched = True
                    self.kernel.up()
                    self.kernel.link(True)
                return bytes(n)
            if req in (0x42, 0x4b, 0x4c, 0x54):
                return bytes(n)
            if req == 0x59:
                raise USBError('Pipe error', -9, errno.EPIPE)
        if (bm, req) in ((0x40, 0x5d), (0xc0, 0x5e)) and not self.no_usbnet:
            with self.mem:
                return self.loader.ctrl(bm, req, value, index, data_or_length)
        raise USBError(f'Pipe error (request {bm:#x}/{req:#x})', -9, errno.EPIPE)


class FakeDev:
    iProduct = 2

    def __init__(self, phone, gen):
        self.phone = phone
        self.gen = gen

    def ctrl_transfer(self, bm, req, value=0, index=0, data_or_length=None, timeout=None):
        p = self.phone
        what = bytes(data_or_length) if bm == 0x40 else int(data_or_length)
        p.log.append((threading.current_thread().name, self.gen, bm, req, value, index, what, timeout))
        if p.unplugged or self.gen != p.gen:
            if self.gen != p.gen:
                p.dead_handle_requests += 1
            raise USBError('No such device (it may have been disconnected)', -4, errno.ENODEV)
        if p.fail_bridge and threading.current_thread().name == 'fuxi-net-bridge':
            p.fail_bridge -= 1
            raise USBError('Operation timed out', -7, errno.ETIMEDOUT)
        return p.answer(bm, req, value, index, data_or_length)


def install_fake_usb(phone):
    usb = types.ModuleType('usb')
    core = types.ModuleType('usb.core')
    util = types.ModuleType('usb.util')
    core.USBError = USBError
    core.find = lambda find_all=True, idVendor=None, idProduct=None: [] if phone.unplugged else [phone.device()]
    util.get_string = lambda d, i: 'Fuxi hardware probe'
    util.claim_interface = lambda d, i: None
    util.dispose_resources = lambda d: None
    usb.core, usb.util = core, util
    sys.modules.update({'usb': usb, 'usb.core': core, 'usb.util': util})


def make_inputs(tmp):
    payload = bytearray(4096)
    payload[:8] = b'SWEETXNU'
    struct.pack_into('<7Q', payload, 16, VIRT, 0, 0, 0, 0, 0x8c0000000, 0x40000000)
    payload[72:72 + 13] = b'serial=3 -v\0\0'
    names = [b'_fuxi_early_status', b'_sweet_log_total', b'_sweet_log_ring']
    values = [VIRT + OFF_EARLY, VIRT + OFF_TOTAL, VIRT + OFF_RING]
    strtab = b'\0' + b'\0'.join(names) + b'\0'
    stroffs, pos = [], 1
    for n in names:
        stroffs.append(pos)
        pos += len(n) + 1
    symoff = 32 + 24
    nlist = b''.join(struct.pack('<IBBHQ', o, 0x0f, 1, 0, v) for o, v in zip(stroffs, values))
    stroff = symoff + len(nlist)
    macho = struct.pack('<8I', 0xfeedfacf, 0x100000c, 0, 2, 1, 24, 0, 0)
    macho += struct.pack('<6I', 2, 24, symoff, len(names), stroff, len(strtab)) + nlist + strtab
    (tmp / 'payload.bin').write_bytes(bytes(payload))
    (tmp / 'kernel.macho').write_bytes(macho)
    return tmp / 'kernel.macho', tmp / 'payload.bin'


def load_monitor(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def run_monitor(mod, argv):
    old = sys.argv
    sys.argv = ['fuxi-xnu-monitor.py', *argv]
    try:
        mod.main()
        return None
    except SystemExit as e:
        return e
    finally:
        sys.argv = old


# ---------------------------------------------------------------- frames
def csum(b):
    if len(b) & 1:
        b += b'\0'
    s = sum(struct.unpack(f'!{len(b) // 2}H', b))
    while s >> 16:
        s = (s & 0xffff) + (s >> 16)
    return ~s & 0xffff


def ipv4(src, dst, proto, payload, ident=1):
    h = struct.pack('!BBHHHBBH4s4s', 0x45, 0, 20 + len(payload), ident, 0, 64, proto, 0, src, dst)
    return h[:10] + struct.pack('!H', csum(h)) + h[12:] + payload


def ping(seq):
    icmp = struct.pack('!BBHHH', 8, 0, 0, 0x4655, seq) + b'route U ping %05d' % seq
    icmp = icmp[:2] + struct.pack('!H', csum(icmp)) + icmp[4:]
    return GW_MAC + PHONE_MAC + b'\x08\x00' + ipv4(PHONE_IP, GW_IP, 1, icmp, seq)


def arp_who_has_gw():
    a = struct.pack('!HHBBH', 1, 0x0800, 6, 4, 1) + PHONE_MAC + PHONE_IP + bytes(6) + GW_IP
    return b'\xff' * 6 + PHONE_MAC + b'\x08\x06' + a


def dhcp_discover():
    opts = bytes((53, 1, 1, 255))
    bootp = struct.pack('!BBBBIHH4s4s4s4s16s64s128s', 1, 1, 6, 0, 0x1234, 0, 0x8000, bytes(4), bytes(4),
                        bytes(4), bytes(4), PHONE_MAC + bytes(10), b'', b'') + b'\x63\x82\x53\x63' + opts
    udp = struct.pack('!HHHH', 68, 67, 8 + len(bootp), 0) + bootp
    return b'\xff' * 6 + PHONE_MAC + b'\x08\x00' + ipv4(bytes(4), b'\xff' * 4, 17, udp)


def classify(fr):
    """('echo', seq) / ('arp', None) / ('dhcp', msgtype) / ('other', None)"""
    if fr[12:14] == b'\x08\x06' and fr[20:22] == b'\x00\x02' and fr[28:32] == GW_IP:
        return 'arp', None
    if fr[12:14] == b'\x08\x00':
        ip = fr[14:]
        ihl = (ip[0] & 15) * 4
        if ip[9] == 1 and ip[ihl] == 0:
            return 'echo', struct.unpack_from('!H', ip, ihl + 6)[0]
        if ip[9] == 17 and struct.unpack_from('!H', ip, ihl + 2)[0] == 68:
            b = ip[ihl + 8:]
            i = 240
            while i + 2 <= len(b) and b[i] != 255:
                if b[i] == 53:
                    return 'dhcp', b[i + 2]
                i += 2 + b[i + 1] if b[i] else 1
            return 'dhcp', None
    return 'other', None


# ---------------------------------------------------------------- test 1
def test_no_net_identical(tmp, kernel, payload):
    try:
        old_src = subprocess.run(['git', 'show', 'HEAD:tools/fuxi-xnu-monitor.py'], cwd=HERE, capture_output=True,
                                 check=True).stdout
    except (OSError, subprocess.CalledProcessError) as e:
        check('without --net: committed monitor from git HEAD', False, repr(e))
        return
    (tmp / 'old_monitor.py').write_bytes(old_src)
    if b'--net' in old_src:
        print('note: git HEAD already has --net; comparing against it anyway', flush=True)
    logs = {}
    for label, path in (('HEAD', tmp / 'old_monitor.py'), ('working tree', HERE / 'fuxi-xnu-monitor.py')):
        phone = FakePhone(end_after_polls=4)
        install_fake_usb(phone)
        mod = load_monitor(path, 'monitor_' + label.replace(' ', '_'))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            ended = run_monitor(mod, ['--kernel', str(kernel), '--payload', str(payload),
                                      '--out', str(tmp / f'out-{label.replace(" ", "_")}'), '--seconds', '30',
                                      '--reset-minutes', '5'])
        logs[label] = [entry[2:] for entry in phone.log]
        check(f'without --net ({label}): run ends at stage 99 after {len(phone.log)} requests',
              ended is None and phone.stage == 99, f'{ended!r} stage {phone.stage}\n{buf.getvalue()[-800:]}')
    same = logs['HEAD'] == logs['working tree']
    detail = ''
    if not same:
        for i, (a, b) in enumerate(zip(logs['HEAD'], logs['working tree'])):
            if a != b:
                detail = f'first difference at request {i}: {a} vs {b}'
                break
        else:
            detail = f'lengths {len(logs["HEAD"])} vs {len(logs["working tree"])}'
    check(f'without --net: byte-for-byte the same {len(logs["HEAD"])} requests as git HEAD', same, detail)
    reqs = sorted({hex(e[1]) for e in logs['working tree']})
    check(f'without --net: no route U request ({", ".join(reqs)})',
          not any(e[1] in (0x5d, 0x5e) for e in logs['working tree']))


# ---------------------------------------------------------------- test 2
class PhoneSide:
    """The phone's kernel side: frames out via KernelSim.tx, replies via KernelSim.rx."""

    def __init__(self, phone):
        self.phone = phone
        self.got = []

    def tx(self, frame):
        with self.phone.mem:
            return self.phone.kernel.tx(frame)

    def drain(self):
        with self.phone.mem:
            frames = self.phone.kernel.rx()
        self.got += [classify(f) for f in frames]

    def wait(self, pred, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.drain()
            if pred():
                return True
            time.sleep(0.005)
        self.drain()
        return pred()

    def echoes(self):
        return {seq for kind, seq in self.got if kind == 'echo'}


def test_net(tmp, kernel, payload):
    phone = FakePhone()
    install_fake_usb(phone)
    mod = load_monitor(HERE / 'fuxi-xnu-monitor.py', 'monitor_net')
    side = PhoneSide(phone)
    notes = {}

    def driver():
        try:
            if not side.wait(lambda: phone.stage == 8, 20):
                notes['error'] = 'no handoff'
                return
            for i in range(1, 4):
                phone.console += b'FUXI console line %d\n' % i
            side.tx(arp_who_has_gw())
            notes['arp'] = side.wait(lambda: ('arp', None) in side.got, 15)
            side.tx(dhcp_discover())
            notes['dhcp_offer'] = side.wait(lambda: ('dhcp', 2) in side.got, 5)
            for seq in range(1, 21):
                side.tx(ping(seq))
            notes['before'] = side.wait(lambda: set(range(1, 21)) <= side.echoes(), 5)
            time.sleep(1.2)                       # a 'net:' stats line or two
            # ---- unplug: every transfer on the old handle fails, nothing enumerates for 1.5 s
            requests_before = len(phone.log)
            phone.unplug()
            notes['unplug_at'] = requests_before
            for seq in range(100, 106):           # sent while the link is down: kept in the TX ring
                side.tx(ping(seq))
            phone.console += b'FUXI console line during the outage\n'
            time.sleep(1.5)
            phone.replug()
            notes['during'] = side.wait(lambda: set(range(100, 106)) <= side.echoes(), 15)
            for seq in range(200, 230):
                side.tx(ping(seq))
                if seq % 8 == 0:
                    side.drain()
            notes['after'] = side.wait(lambda: set(range(200, 230)) <= side.echoes(), 10)
            # ---- one bridge request times out while the device is fine: the bridge pauses
            # and resumes on the same handle after the next console poll
            phone.fail_bridge = 1
            time.sleep(0.6)
            for seq in range(300, 310):
                side.tx(ping(seq))
            notes['transient'] = side.wait(lambda: set(range(300, 310)) <= side.echoes(), 10)
            phone.console += b'FUXI console line last\n'
            time.sleep(1.5)                       # the console loop reads it; the 0x54 renewal runs
        except Exception as e:
            notes['error'] = repr(e)
        finally:
            phone.stage = 99                      # the console loop ends

    th = threading.Thread(target=driver, name='phone-driver', daemon=True)
    th.start()
    out_dir = tmp / 'out-net'
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ended = run_monitor(mod, ['--kernel', str(kernel), '--payload', str(payload), '--out', str(out_dir),
                                  '--seconds', '60', '--reset-minutes', '5', '--net', '--net-upstream-dns',
                                  '127.0.0.1', '--net-stats-seconds', '1', '--reconnect-seconds', '30'])
    th.join(5)
    out = buf.getvalue()
    (tmp / 'monitor-net-stdout.txt').write_text(out)
    print('---- monitor --net output (net and USB lines) ----')
    for line in out.splitlines():
        if line.startswith('USB') or 'net:' in line:
            print('  ' + line[:230])
    print('----')
    check('--net run ends normally at stage 99', ended is None and phone.stage == 99 and 'error' not in notes,
          f'{ended!r} {notes.get("error")}')
    check('bridge answers ARP for 192.168.88.1', notes.get('arp'))
    check('bridge answers DHCP DISCOVER with an OFFER', notes.get('dhcp_offer'))
    check('20 pings to 192.168.88.1 answered before the unplug', notes.get('before'))
    check('6 pings sent during the outage answered after the reconnect (state kept)', notes.get('during'))
    check('30 pings answered after the reconnect', notes.get('after'))
    check('a bridge-only timeout: paused, resumed on the same handle, 10 more pings answered',
          notes.get('transient'))
    bridge_reqs = [e for e in phone.log if e[0] == 'fuxi-net-bridge']
    other_reqs = [e for e in phone.log if e[0] != 'fuxi-net-bridge']
    kinds = sorted({(hex(e[2]), hex(e[3])) for e in bridge_reqs})
    check(f'bridge thread sent only 0x5d/0x5e ({len(bridge_reqs)} requests: {kinds})',
          bridge_reqs and all((e[2], e[3]) in ((0x40, 0x5d), (0xc0, 0x5e)) for e in bridge_reqs))
    check('bridge thread never sent 0x49/0x4e/0x4f/0x55/0x57',
          not any(e[3] in (0x49, 0x4e, 0x4f, 0x55, 0x57) for e in bridge_reqs))
    check('monitor thread never sent 0x5d/0x5e', not any(e[3] in (0x5d, 0x5e) for e in other_reqs))
    check('bridge requests use a 500 ms timeout, on the first handle and then on the reconnected one',
          all(e[7] == 500 for e in bridge_reqs) and any(e[1] == 1 for e in bridge_reqs)
          and any(e[1] == 0 for e in bridge_reqs))
    dead = [e for e in phone.log[notes.get('unplug_at', 0):] if e[0] == 'fuxi-net-bridge' and e[1] == 0]
    check(f'bridge paused at once: {len(dead)} request(s) on the unplugged handle (<= 2)', len(dead) <= 2)
    progress = [json.loads(line) for line in (out_dir / 'progress.jsonl').read_text().splitlines()]
    recon = [p for p in progress if 'usb_reconnect' in p]
    check(f'monitor reconnect() found the monitor again ({len(recon)} reconnect)', len(recon) >= 1)
    net = [p['net'] for p in progress if 'net' in p]
    s = net[-1] if net else {}
    ring = s.get('ring', {})
    check(f'net summary (unplug, timeout, session end): pauses {s.get("pauses")}, resumes {s.get("resumes")}, device swaps '
          f'{s.get("device_swaps")}, usb errors {s.get("errors")}',
          s.get('pauses', 0) == 3 and s.get('device_swaps', 0) == 1 and s.get('resumes', 0) == 2
          and s.get('errors') in (1, 2) and not s.get('failed'))  # 1: the monitor paused it before its next request
    check(f'ring: in {ring.get("frames_in")} out {ring.get("frames_out")}, stall {ring.get("stall")}, phone_drops '
          f'{ring.get("phone_drops")}, bad_seq {ring.get("bad_seq")}, bad_fnv {ring.get("bad_fnv")}',
          ring and ring['stall'] == 0 and ring['phone_drops'] == 0 and ring['bad_fnv'] == 0 and ring['bad_seq'] == 0)
    renewals = [e for e in phone.log if e[3] == 0x54]
    check(f'0x54 reset-deadline renewal still runs after the reconnect ({len(renewals)} sent)', len(renewals) >= 2)
    log = (out_dir / 'boot.log').read_bytes()
    check('console log complete across the reconnect', log == bytes(phone.console),
          f'{len(log)} vs {len(phone.console)} bytes')
    check('periodic net: stats lines in the monitor output',
          out.count('net: ') >= 3 and 'transport: ring in=' in out and 'stats up=' in out)


# ---------------------------------------------------------------- test 3
def test_net_stall_loader(tmp, kernel, payload):
    phone = FakePhone(end_after_polls=14)        # about 3.5 s of console polls
    phone.no_usbnet = True
    install_fake_usb(phone)
    mod = load_monitor(HERE / 'fuxi-xnu-monitor.py', 'monitor_net_stall')
    out_dir = tmp / 'out-net-stall'
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ended = run_monitor(mod, ['--kernel', str(kernel), '--payload', str(payload), '--out', str(out_dir),
                                  '--seconds', '30', '--net', '--net-upstream-dns', '127.0.0.1',
                                  '--net-stats-seconds', '0'])
    out = buf.getvalue()
    for line in out.splitlines():
        if 'net:' in line:
            print('  ' + line[:200])
    bridge_reqs = [e for e in phone.log if e[0] == 'fuxi-net-bridge']
    check('loader without route U: the run ends normally at stage 99', ended is None and phone.stage == 99,
          f'{ended!r}\n{out[-600:]}')
    check(f'loader without route U: exactly 1 STALLed bridge request ({len(bridge_reqs)} sent, '
          f'{phone.polls_after_handoff} console polls)', len(bridge_reqs) == 1)
    check('loader without route U: "bridge disabled" message',
          'net: route U STALLed (loader without FUXI_USBNET?), bridge disabled' in out)
    progress = [json.loads(line) for line in (out_dir / 'progress.jsonl').read_text().splitlines()]
    net = [p['net'] for p in progress if 'net' in p]
    s = net[-1] if net else {}
    check(f'loader without route U: summary disabled={s.get("disabled")!r} resumes={s.get("resumes")}',
          s.get('disabled') and s.get('resumes') == 0)
    check('loader without route U: console log complete', (out_dir / 'boot.log').read_bytes() == bytes(phone.console))
    # The same NetLink after a reconnect: set_dev() and resume() never turn it back on.
    link = mod.NetLink(phone.device(), threading.Lock(), out=lambda m: None)
    link.started = True
    link.on_error(R.Stall('0x5e'))
    link.resume()
    link.set_dev(phone.device())
    try:
        link.ctrl(0xc0, 0x5e, 0, 0, 192)
        refused = False
    except OSError:
        refused = True
    check('a disabled NetLink stays off after resume() and set_dev(), ctrl() refuses',
          not link.up and link.disabled and refused)


# ---------------------------------------------------------------- test 4
def test_net_interrupt(tmp, kernel, payload):
    phone = FakePhone()
    install_fake_usb(phone)
    mod = load_monitor(HERE / 'fuxi-xnu-monitor.py', 'monitor_net_int')
    side = PhoneSide(phone)
    notes = {}

    def driver():
        if not side.wait(lambda: phone.stage == 8, 20):
            return
        side.tx(arp_who_has_gw())
        notes['arp'] = side.wait(lambda: ('arp', None) in side.got, 15)
        for seq in range(1, 6):
            side.tx(ping(seq))
        notes['ping'] = side.wait(lambda: set(range(1, 6)) <= side.echoes(), 5)
        phone.interrupt_main = True
        for seq in range(10, 400):            # keep the bridge busy until the end
            side.tx(ping(seq))
            side.drain()
            time.sleep(0.002)
            if phone.interrupt_at is not None and seq > 20:
                break

    th = threading.Thread(target=driver, name='phone-driver', daemon=True)
    th.start()
    out_dir = tmp / 'out-net-int'
    buf = io.StringIO()
    interrupted = False
    with contextlib.redirect_stdout(buf):
        try:
            run_monitor(mod, ['--kernel', str(kernel), '--payload', str(payload), '--out', str(out_dir),
                              '--seconds', '60', '--net', '--net-upstream-dns', '127.0.0.1',
                              '--net-stats-seconds', '0'])
        except KeyboardInterrupt:
            interrupted = True
    th.join(5)
    time.sleep(0.3)
    at = phone.interrupt_at
    check('Ctrl+C: KeyboardInterrupt reaches the caller after the bridge carried traffic',
          interrupted and at is not None and notes.get('arp') and notes.get('ping'), f'{notes} {at}')
    after = phone.log[at:] if at is not None else []
    bridge_after = [e for e in after if e[0] == 'fuxi-net-bridge']
    end_diag = [i for i, e in enumerate(after) if e[3] == 0x59]
    last_bridge = max((i for i, e in enumerate(after) if e[0] == 'fuxi-net-bridge'), default=-1)
    check(f'Ctrl+C: no bridge request after the interrupt ({len(bridge_after)}), the end-of-session '
          f'0x59 went out ({len(end_diag)})', len(bridge_after) == 0 and end_diag and last_bridge < end_diag[0])
    alive = [t.name for t in threading.enumerate() if t.name == 'fuxi-net-bridge']
    check(f'Ctrl+C: bridge thread stopped (alive: {alive})', not alive)
    progress = [json.loads(line) for line in (out_dir / 'progress.jsonl').read_text().splitlines()]
    net = [p['net'] for p in progress if 'net' in p]
    ring = (net[-1] if net else {}).get('ring', {})
    check(f'Ctrl+C: net summary recorded in progress.jsonl (ring in {ring.get("frames_in")} '
          f'out {ring.get("frames_out")})', net and ring.get('frames_out', 0) > 0)


# ---------------------------------------------------------------- test 5
def run_for(t, seconds, counter):
    """The bridge's polled-transport schedule: recv_frames, then wait poll_interval."""
    end = time.monotonic() + seconds
    calls = 0
    while time.monotonic() < end:
        t.recv_frames()
        calls += 1
        time.sleep(max(t.poll_interval, 0.0005))
    return calls


def test_ring_transport_unit():
    bridge = load_monitor(HERE / 'fuxi-usb-net-bridge.py', 'bridge_unit')
    frame = GW_MAC + PHONE_MAC + b'\x08\x00' + bytes(60)

    # 0x5d STALL: one try, then the transport is off; no retry loop
    _, loader, kernel = R.sim_pair()
    reqs = []
    errors = []

    def ctrl(bm, req, value, index, data):
        reqs.append(req)
        if req == 0x5d:
            raise R.Stall('0x5d')
        return loader.ctrl(bm, req, value, index, data)
    t = bridge.RingTransport(ctrl=ctrl, on_error=errors.append)
    t.recv_frames()
    t.send_frame(frame)
    t.send_frame(frame)
    n0 = len(reqs)
    calls = run_for(t, 0.5, reqs)
    stalls = reqs.count(0x5d)
    check(f'0x5d STALL: {stalls} STALLed 0x5d, {len(reqs) - n0} requests in the next 0.5 s ({calls} '
          f'recv_frames), transport disabled, on_error told ({len(errors)})',
          stalls == 1 and len(reqs) == n0 and t.stalled and not t.pending and len(errors) == 1
          and R.is_stall(errors[0]) and 'STALLED' in t.describe())

    # 0x5e STALL (loader without route U): the first poll, then nothing
    reqs2 = []

    def ctrl2(bm, req, value, index, data):
        reqs2.append(req)
        raise R.Stall('0x5e')
    t2 = bridge.RingTransport(ctrl=ctrl2)
    run_for(t2, 0.5, reqs2)
    check(f'0x5e STALL without on_error: {len(reqs2)} request(s) in 0.5 s, transport disabled',
          len(reqs2) == 1 and t2.stalled and t2.poll_interval == t2.SLOW_POLL)

    # Kernel down with frames queued: slow polls, stale frames dropped, back to work after
    _, loader, kernel = R.sim_pair()
    reqs3 = []

    def ctrl3(bm, req, value, index, data):
        reqs3.append(req)
        return loader.ctrl(bm, req, value, index, data)
    t3 = bridge.RingTransport(ctrl=ctrl3)
    t3.recv_frames()
    for _ in range(40):                       # 32 in flight, 8 queued
        t3.send_frame(frame)
    queued = len(t3.pending)
    kernel.set_state(R.KSTATE_UP, False)
    t3.recv_frames()
    t3.send_frame(frame)
    t3.send_frame(frame)
    n0 = len(reqs3)
    calls = run_for(t3, 1.0, reqs3)
    per_s = len(reqs3) - n0
    c = t3.c
    check(f'kernel down with {queued} frames queued: {per_s} requests/s ({calls} recv_frames), interval '
          f'{t3.poll_interval}, queue {len(t3.pending)}, dropped {c["tx_flushed_not_ready"]} queued + '
          f'{c["tx_dropped_not_ready"]} new',
          queued == 8 and per_s <= 80 and not t3.pending and t3.poll_interval == t3.SLOW_POLL
          and c['tx_flushed_not_ready'] == 8 and c['tx_dropped_not_ready'] == 2)
    kernel.rx()                               # the 32 frames that got through before
    kernel.set_state(R.KSTATE_UP, True)
    t3.recv_frames()
    ok = t3.send_frame(frame)
    t3.recv_frames()
    got = kernel.rx()
    check(f'kernel back up: frames flow again ({len(got)} delivered)', ok and got == [frame])


def main():
    with tempfile.TemporaryDirectory(prefix='fuxi-monitor-net-') as t:
        tmp = Path(t)
        kernel, payload = make_inputs(tmp)
        test_no_net_identical(tmp, kernel, payload)
        test_net(tmp, kernel, payload)
        test_net_stall_loader(tmp, kernel, payload)
        test_net_interrupt(tmp, kernel, payload)
        test_ring_transport_unit()
    bad = [n for n, ok in RESULTS if not ok]
    print(f'monitor --net tests: {len(RESULTS) - len(bad)} PASS, {len(bad)} FAIL', flush=True)
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
