#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fuxi-usb-net-bridge.py  --  user-mode IPv4 gateway for the OnePlus 11 iOS port.

The iOS port exposes a synthetic Ethernet interface (en0).  Its raw Ethernet II
frames reach the PC over a frame transport (today: UDP datagrams for testing,
later: the USB debug-link RAM ring).  This bridge pretends to be the phone's
whole LAN + router:

    gateway   192.168.88.1   MAC 02:46:55:58:49:fe
    phone     192.168.88.2   /24, MTU 1500

and terminates every IPv4 flow in Python, re-originating it with ordinary
outbound sockets.  It never touches Windows networking: no TAP driver, no
ICS/NAT, no routes, no firewall rules, no admin rights.  The only socket that
listens is the UDP frame transport itself.

Implemented: ARP, DHCP server, ICMP echo (gateway), DNS relay (UDP and TCP to
.1:53), UDP NAT, a user-mode TCP terminator (per-flow seq/ack state machine
with retransmission, window/backpressure, half-close), IP fragmentation of
oversized replies, stats.

Python 3.10+, standard library only, Windows and Linux.

Example:
    py fuxi-usb-net-bridge.py --transport udp --listen 127.0.0.1:47001 --peer 127.0.0.1:47002
"""
from __future__ import annotations

import argparse
import errno
import os
import random
import re
import select
import socket
import struct
import subprocess
import sys
import time
from collections import Counter, OrderedDict, deque

IS_WIN = sys.platform.startswith("win")

# ----------------------------------------------------------------------------
# Fixed addressing
# ----------------------------------------------------------------------------
GW_MAC = bytes.fromhex("0246555849fe")
BCAST_MAC = b"\xff" * 6
GW_IP = socket.inet_aton("192.168.88.1")
PHONE_IP = socket.inet_aton("192.168.88.2")
NETMASK = socket.inet_aton("255.255.255.0")
SUBNET_BCAST = socket.inet_aton("192.168.88.255")
ALL_ONES = b"\xff\xff\xff\xff"
ZERO_IP = b"\x00\x00\x00\x00"
MTU = 1500
MSS = 1460
M32 = 0xFFFFFFFF

ETH_IP, ETH_ARP, ETH_IP6 = 0x0800, 0x0806, 0x86DD
PROTO_ICMP, PROTO_TCP, PROTO_UDP = 1, 6, 17
FIN, SYN, RST, PSH, ACK = 0x01, 0x02, 0x04, 0x08, 0x10

# ----------------------------------------------------------------------------
# Limits / timers
# ----------------------------------------------------------------------------
TCP_MAX_FLOWS = 512
TCP_CONNECT_TIMEOUT = 10.0
TCP_RTO_INIT = 0.5
TCP_RTO_MAX = 16.0
TCP_MAX_RETRIES = 8
TCP_TIME_WAIT = 2.0
TCP_DEAD_PEER = 120.0          # phone silent while we have unacked data
TCP_KEEPALIVE_IDLE = 300.0     # phone silent this long with nothing outstanding -> probe
TCP_KEEPALIVE_INTVL = 15.0
TCP_KEEPALIVE_PROBES = 4       # unanswered probes before abort
TCP_HALFCLOSE_IDLE = 120.0     # phone FIN'd, we shut_wr, server silent this long -> abort
TCP_IDLE_MAX = 2 * 3600.0      # no data either way this long -> abort
TCP_IDLE_CROWDED = 15 * 60.0   # same, while the flow table is >75% full
TCP_WINDOW = 65535
FLOW_BUF_CAP = 1 << 20         # per direction, per flow
GLOBAL_BUF_CAP = 64 << 20
OOO_MAX_BYTES = 256 * 1024
OOO_MAX_SEGS = 64
SOCK_READ_CHUNK = 256 * 1024

UDP_MAX_FLOWS = 256
UDP_IDLE = 60.0
DNS_TIMEOUT = 10.0
DNS_MAX_PENDING = 1024
DHCP_LEASE = 86400

DHCP_MAGIC = b"\x63\x82\x53\x63"


LOG_PREFIX = ""                # e.g. "net: " when the bridge runs inside the monitor


def log(msg: str) -> None:
    sys.stdout.write(time.strftime("%H:%M:%S ") + LOG_PREFIX + msg + "\n")
    sys.stdout.flush()


# ----------------------------------------------------------------------------
# Checksums and packet builders
# ----------------------------------------------------------------------------
def csum(data) -> int:
    """RFC 1071 Internet checksum (value to store in the field).

    Uses the identity sum16(words) == int(data) mod 0xFFFF (since 2^16 == 1
    mod 0xFFFF), which is very fast in CPython.  Verifying a block that already
    contains its checksum yields 0.
    """
    if len(data) & 1:
        data = bytes(data) + b"\x00"
    n = int.from_bytes(data, "big")
    r = n % 0xFFFF
    if r == 0 and n:
        r = 0xFFFF
    return (~r) & 0xFFFF


def pseudo(src: bytes, dst: bytes, proto: int, length: int) -> bytes:
    return src + dst + struct.pack("!BBH", 0, proto, length)


def ip_to_int(ip: bytes) -> int:
    return int.from_bytes(ip, "big")


def _net(cidr: str):
    a, b = cidr.split("/")
    b = int(b)
    mask = (M32 << (32 - b)) & M32 if b else 0
    return ip_to_int(socket.inet_aton(a)) & mask, mask


ALWAYS_BLOCKED = [_net(c) for c in ("0.0.0.0/8", "127.0.0.0/8", "224.0.0.0/4",
                                    "240.0.0.0/4", "192.168.88.0/24")]
LAN_BLOCKED = [_net(c) for c in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16",
                                 "169.254.0.0/16")]


def parse_hostport(s: str, default_host: str = "127.0.0.1"):
    if ":" not in s:
        return default_host, int(s)
    h, p = s.rsplit(":", 1)
    return (h or default_host), int(p)


# ----------------------------------------------------------------------------
# Frame transports
# ----------------------------------------------------------------------------
class FrameTransport:
    """Moves whole Ethernet II frames between the phone and the bridge.

    recv_frames() must not block and returns every frame currently available
    (possibly an empty list).  send_frame() must not block; it returns False
    if the frame had to be dropped (TCP will retransmit).  fileno() returns a
    selectable socket/fd, or None, in which case the bridge polls every
    poll_interval seconds.
    """

    poll_interval = 0.005

    def recv_frames(self) -> list:
        raise NotImplementedError

    def send_frame(self, frame: bytes) -> bool:
        raise NotImplementedError

    def fileno(self):
        return None

    def close(self) -> None:
        pass

    def describe(self) -> str:
        return self.__class__.__name__


class UdpFrameTransport(FrameTransport):
    """One Ethernet frame per UDP datagram, to/from a fixed peer address.

    Used by the WSL test harness (TAP <-> UDP relay) and handy for any other
    out-of-process frame pump.  Datagrams from any other source are ignored.
    If the peer disappears and comes back nothing is reset: the bridge keeps
    all flow state and TCP simply retransmits.
    """

    def __init__(self, listen, peer, max_batch: int = 512):
        self.listen = listen
        self.peer = (socket.gethostbyname(peer[0]), int(peer[1]))
        self.max_batch = max_batch
        self.foreign = 0
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if IS_WIN and hasattr(socket, "SIO_UDP_CONNRESET"):
            try:  # stop ICMP port-unreachable from poisoning recvfrom()
                s.ioctl(socket.SIO_UDP_CONNRESET, False)
            except OSError:
                pass
        for opt in (socket.SO_RCVBUF, socket.SO_SNDBUF):
            try:
                s.setsockopt(socket.SOL_SOCKET, opt, 4 << 20)
            except OSError:
                pass
        s.bind(listen)
        s.setblocking(False)
        self.sock = s

    def fileno(self):
        return self.sock.fileno()

    def recv_frames(self) -> list:
        out = []
        for _ in range(self.max_batch):
            try:
                data, addr = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except ConnectionResetError:
                continue
            except OSError:
                break
            if addr[0] != self.peer[0] or addr[1] != self.peer[1]:
                self.foreign += 1
                continue
            out.append(data)
        return out

    def send_frame(self, frame: bytes) -> bool:
        try:
            self.sock.sendto(frame, self.peer)
            return True
        except (BlockingIOError, InterruptedError):
            return False
        except OSError:
            return False

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def describe(self) -> str:
        return "udp listen=%s:%d peer=%s:%d" % (self.listen + self.peer)


def load_ring_module():
    """tools/fuxi_usbnet_ring.py, from this file's own directory (whatever the cwd is)."""
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    import fuxi_usbnet_ring
    return fuxi_usbnet_ring


class RingTransport(FrameTransport):
    """Route U: Ethernet frames over the loader's USB monitor (fuxi_usbnet_ring.py).

    Wraps fuxi_usbnet_ring.UsbnetTransport: give it either ctrl (a pyusb-style
    ctrl_transfer callable) and lock (shared with every other user of the USB
    device), or an existing UsbnetTransport as transport=.  fileno() is None:
    the bridge polls.  poll_interval is adaptive: FAST_POLL while frames came
    from the phone in the last ACTIVE_WINDOW seconds or frames wait to go to
    it, else SLOW_POLL.  After a poll that returned frames the bridge polls
    again at once (up to BURST polls in a row), so a full phone ring drains
    quickly while other users of the lock still get their turn.

    send_frame() never blocks on the phone: a frame the phone cannot take now
    (UsbnetTransport.send_frame False: not ready or 32 frames in flight) waits
    in a bounded FIFO (tx_cap, oldest dropped and counted) and goes out, in
    order, after the next poll; while the last header shows the phone not
    ready or its kernel down, frames are dropped (counted) instead, so a
    phone that went away is polled slowly, not hammered.  A USB error pauses
    the transport, keeping every state: with ready= (a callable) it resumes
    once ready() is true, otherwise after RETRY_SECONDS.  A STALL on either
    request (also the 0x5d STALL the ring module reports as busy) disables it
    for good: a correct loader never STALLs them.  on_error(e), if given, is
    told about each error (a STALL as fuxi_usbnet_ring.Stall or the USB
    error itself).  The only requests made are the ring module's 0x5d and 0x5e.
    """

    FAST_POLL = 0.0015
    SLOW_POLL = 0.015
    ACTIVE_WINDOW = 0.1
    RETRY_SECONDS = 0.5
    BURST = 8

    def __init__(self, ctrl=None, lock=None, transport=None, in_length=None,
                 tx_cap: int = 256, ready=None, on_error=None):
        ring = load_ring_module()
        if transport is None:
            if ctrl is None:
                raise ValueError("RingTransport needs ctrl or transport")
            transport = ring.UsbnetTransport(ctrl, lock, in_length or ring.IN_MAX)
        self.ring_mod = ring
        self.ring = transport
        self.pending = deque()
        self.tx_cap = tx_cap
        self.ready = ready
        self.on_error = on_error
        self.paused = False
        self.stalled = False            # a STALL on 0x5d/0x5e: off for good (see _fail)
        self.retry_at = 0.0
        self.last_rx = -1e9
        self.burst = 0
        self.last_error = None
        self.c = Counter()

    def _phone_can_take(self) -> bool:
        """The last header shows the loader ready and the phone's kernel up."""
        h = self.ring.last
        return bool(h is not None and h.get("kernel_up"))

    @property
    def poll_interval(self) -> float:
        if self.paused or self.stalled:
            return self.SLOW_POLL
        if self.burst:
            return 0.0
        if time.monotonic() - self.last_rx < self.ACTIVE_WINDOW:
            return self.FAST_POLL
        if self.pending and self._phone_can_take():
            return self.FAST_POLL
        return self.SLOW_POLL

    def _fail(self, where: str, e) -> None:
        """A USB error: pause, keeping every state.  A STALL (EPIPE) is different: a
        correct route U loader never STALLs a well-formed 0x5d/0x5e (SPEC A1), so it means
        a loader without FUXI_USBNET, or a request this host gets wrong, and on WinUSB
        the device is gone until a replug.  Retrying only repeats it: the transport
        stops for good (no further request), and on_error still hears about it."""
        self.c["usb_errors"] += 1
        if not self.paused:
            self.c["pauses"] += 1
        self.paused = True
        self.burst = 0
        self.retry_at = time.monotonic() + self.RETRY_SECONDS
        self.last_error = "%s: %r" % (where, e)
        if self.ring_mod.is_stall(e) and not self.stalled:
            self.stalled = True
            self.c["tx_dropped_stall"] += len(self.pending)
            self.pending.clear()
            log("route U %s STALLed (loader without FUXI_USBNET?): ring transport disabled" % where)
        if self.on_error is not None:
            try:
                self.on_error(e)
            except Exception:
                pass

    def _resume_ok(self) -> bool:
        if self.stalled:
            return False
        if not self.paused:
            return True
        if self.ready is not None:
            try:
                ok = bool(self.ready())
            except Exception:
                ok = False
        else:
            ok = time.monotonic() >= self.retry_at
        if ok:
            self.paused = False
            self.c["resumes"] += 1
        return ok

    def _send_one(self, frame: bytes, where: str):
        """One UsbnetTransport.send_frame.  True: sent; False: busy, keep the frame;
        None: the frame is gone (a STALL, which the ring module reports as busy, or a
        USB error: the transport is paused or stopped then)."""
        stalls = self.ring.stats["stall"]
        try:
            sent = self.ring.send_frame(frame)
        except Exception as e:
            self._fail(where, e)
            return False if not self.stalled else None
        if self.ring.stats["stall"] != stalls:
            self.c["tx_dropped_stall"] += 1
            self._fail(where, self.ring_mod.Stall("0x5d"))
            return None
        return sent

    def _flush(self) -> None:
        while self.pending and not self.paused and not self.stalled:
            sent = self._send_one(self.pending[0], "send")
            if sent is None:            # stalled: _fail emptied the queue
                return
            if not sent:
                return
            self.pending.popleft()

    def _drop_pending_not_ready(self) -> None:
        # The phone cannot take frames (loader not ready or rebooted, kernel down): what
        # waits is stale by the time it comes back, and keeping it would poll fast for ever.
        if self.pending:
            self.c["tx_flushed_not_ready"] += len(self.pending)
            self.pending.clear()

    def recv_frames(self) -> list:
        if not self._resume_ok():
            return []
        try:
            frames = self.ring.poll()
        except Exception as e:
            self._fail("poll", e)
            return []
        if frames:
            self.last_rx = time.monotonic()
            self.burst = self.burst + 1 if self.burst < self.BURST else 0
        else:
            self.burst = 0
        if self._phone_can_take():
            self._flush()
        else:
            self._drop_pending_not_ready()
        return frames

    def send_frame(self, frame: bytes) -> bool:
        n = len(frame)
        if not self.ring_mod.FRAME_MIN <= n <= self.ring_mod.FRAME_MAX:
            self.c["tx_bad_size"] += 1
            return False
        if self.stalled:
            self.c["tx_dropped_stall"] += 1
            return False
        if not self._phone_can_take():
            # Not ready / kernel down at the last poll: nothing to send to (and no
            # extra 0x5e per frame); the bridge's TCP retransmits once it is back.
            self.c["tx_dropped_not_ready"] += 1
            return False
        frame = bytes(frame)
        if not self.pending and not self.paused:
            sent = self._send_one(frame, "send")
            if sent:
                return True
            if sent is None:
                return False
        self.pending.append(frame)
        if len(self.pending) > self.tx_cap:
            self.pending.popleft()
            self.c["tx_queue_drops"] += 1
        return True

    def describe(self) -> str:
        s = self.ring.stats
        return ("ring in=%d out=%d polls=%d busy=%d stall=%d not_ready=%d phone_drops=%d "
                "resync=%d bad_seq=%d bad_fnv=%d bad_entry=%d | txq=%d qdrops=%d usb_err=%d "
                "resumes=%d not_ready_drops=%d%s"
                % (s["frames_in"], s["frames_out"], s["polls"], s["busy"], s["stall"],
                   s["not_ready"], s["phone_drops"], s["resync"], s["bad_seq"], s["bad_fnv"],
                   s["bad_entry"], len(self.pending), self.c["tx_queue_drops"],
                   self.c["usb_errors"], self.c["resumes"],
                   self.c["tx_flushed_not_ready"] + self.c["tx_dropped_not_ready"],
                   " STALLED (disabled)" if self.stalled else " PAUSED" if self.paused else ""))


class SimPhone:
    """--transport sim test hook: the phone's kernel side (KernelSim) of a simulated
    route U ring, fed from a UDP frame socket (the netns TAP relay of
    tools/usbnet-bridge-test).  Datagrams from the socket are what en0 sends
    (KernelSim.tx); frames the bridge delivered to the phone (KernelSim.rx) go back
    to the socket.  Runs in a daemon thread and takes the ring's lock around every
    kernel-side access, as the loader's requests do.  A frame the kernel TX ring has
    no room for waits in a bounded backlog (en0's send queue)."""

    def __init__(self, kernel, lock, listen, peer, backlog: int = 512, interval: float = 0.001):
        self.kernel = kernel
        self.lock = lock
        self.listen = listen
        self.peer = (socket.gethostbyname(peer[0]), int(peer[1]))
        self.backlog = deque()
        self.backlog_cap = backlog
        self.interval = interval
        self.c = Counter()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for opt in (socket.SO_RCVBUF, socket.SO_SNDBUF):
            try:
                s.setsockopt(socket.SOL_SOCKET, opt, 4 << 20)
            except OSError:
                pass
        s.bind(listen)
        s.setblocking(False)
        self.sock = s

    def step(self, wait: float) -> None:
        r, _, _ = select.select([self.sock], [], [], wait)
        if r:
            for _ in range(512):
                try:
                    data, addr = self.sock.recvfrom(65535)
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    break
                if addr[0] != self.peer[0] or addr[1] != self.peer[1]:
                    self.c["foreign"] += 1
                    continue
                self.backlog.append(data)
                if len(self.backlog) > self.backlog_cap:
                    self.backlog.popleft()
                    self.c["backlog_drops"] += 1
        with self.lock:
            while self.backlog:
                got = self.kernel.tx(self.backlog[0])
                if got == "full":
                    break
                self.backlog.popleft()
                self.c["tx_" + got] += 1
            frames = self.kernel.rx()
        for fr in frames:
            try:
                self.sock.sendto(fr, self.peer)
                self.c["rx"] += 1
            except OSError:
                self.c["rx_send_err"] += 1

    def run(self) -> None:
        while True:
            self.step(0 if self.backlog else self.interval)

    def start(self):
        import threading
        th = threading.Thread(target=self.run, name="sim-phone", daemon=True)
        th.start()
        return th

    def describe(self) -> str:
        return ("sim phone udp listen=%s:%d peer=%s:%d " % (self.listen + self.peer)
                + " ".join("%s=%d" % kv for kv in sorted(self.c.items())))


# ----------------------------------------------------------------------------
# TCP flow state
# ----------------------------------------------------------------------------
CONNECTING, SYNACK, EST, TIME_WAIT = "CONNECTING", "SYNACK", "EST", "TIME_WAIT"


class Flow:
    __slots__ = ("key", "sport", "dip", "dport", "rhost", "rport", "sock", "state",
                 "irs", "iss", "rcv_nxt", "snd_una", "sent", "sndbuf", "rcvbuf",
                 "ooo", "ooo_bytes", "wnd", "mss", "sock_eof", "fin_sent", "fin_ever",
                 "fin_acked", "phone_fin", "shut_wr", "rto", "rto_at", "retries",
                 "dupacks", "need_ack", "deadline", "last_rx", "adv_wnd", "up", "down",
                 "created", "last_act", "rcv_adv", "ka_probes", "ka_at")

    def __init__(self):
        self.sock = None
        self.sndbuf = bytearray()      # socket -> phone, starts at seq snd_una
        self.rcvbuf = bytearray()      # phone -> socket, not yet written
        self.ooo = {}
        self.ooo_bytes = 0
        self.sent = 0
        self.sock_eof = False
        self.fin_sent = False
        self.fin_ever = False
        self.fin_acked = False
        self.phone_fin = False
        self.shut_wr = False
        self.rto = TCP_RTO_INIT
        self.rto_at = None
        self.retries = 0
        self.dupacks = 0
        self.need_ack = False
        self.deadline = None
        self.adv_wnd = TCP_WINDOW
        self.rcv_adv = None            # right edge of the last window we advertised
        self.ka_probes = 0
        self.ka_at = None
        self.up = 0
        self.down = 0

    def snd_nxt(self) -> int:
        return (self.snd_una + self.sent + (1 if self.fin_sent else 0)) & M32

    def outstanding(self) -> bool:
        return self.sent > 0 or (self.fin_sent and not self.fin_acked)

    def name(self) -> str:
        return "%d->%s:%d" % (self.sport, socket.inet_ntoa(self.dip), self.dport)


class UdpFlow:
    __slots__ = ("key", "sock", "last", "dip", "dport", "sport")


# ----------------------------------------------------------------------------
# The bridge
# ----------------------------------------------------------------------------
class Bridge:
    def __init__(self, transport: FrameTransport, upstream_dns: str, allow_lan=False,
                 fake_ping_external=False, verbose=False, stats_every=30.0):
        self.t = transport
        self.upstream_dns = upstream_dns
        self.allow_lan = allow_lan
        self.fake_ping_external = fake_ping_external
        self.verbose = verbose
        self.stats_every = stats_every
        self.c = Counter()
        self.phone_mac = None
        self.ip_id = random.randint(0, 0xFFFF)
        self.flows = {}
        self.ack_set = set()
        self.udp_flows = OrderedDict()
        self.sockmap = {}
        self.global_buf = 0            # bytes in all flows' sndbuf+rcvbuf (kept current)
        self.wnd_starved = False       # some flow got a sub-MSS window due to the global cap
        self.t_err_until = 0.0         # transport recv backoff after an exception
        self.t_err_logged = 0.0
        self.dns_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # connected, never bound to a listening address: a plain outbound client socket
        self.dns_sock.connect((self.upstream_dns, 53))
        self.dns_sock.setblocking(False)
        if IS_WIN and hasattr(socket, "SIO_UDP_CONNRESET"):
            try:
                self.dns_sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            except OSError:
                pass
        self.dns_pending = {}
        self.next_stats = time.monotonic() + stats_every if stats_every > 0 else None
        self.next_timer_scan = 0.0
        self.next_t_poll = 0.0         # polled transports (fileno None): next recv_frames
        self.stats_hook = None         # called after each periodic stats line
        self.running = True
        self.started = time.monotonic()

    # ---------------------------------------------------------------- output
    def vlog(self, msg):
        if self.verbose:
            log(msg)

    def drop(self, why: str):
        self.c["drop_" + why] += 1

    def send_eth(self, dst_mac: bytes, ethertype: int, payload: bytes):
        frame = dst_mac + GW_MAC + struct.pack("!H", ethertype) + payload
        if len(frame) < 60:
            frame += b"\x00" * (60 - len(frame))
        try:
            ok = self.t.send_frame(frame)
        except Exception as e:     # transport down/unplugged: keep all state
            ok = False
            self.transport_error("send", e)
        if ok:
            self.c["frames_out"] += 1
            self.c["bytes_out"] += len(frame)
        else:
            self.c["tx_fail"] += 1

    def transport_error(self, where: str, e):
        self.c["transport_err"] += 1
        now = time.monotonic()
        if now - self.t_err_logged > 5.0:
            self.t_err_logged = now
            log("transport %s error (%d so far): %r" % (where, self.c["transport_err"], e))

    def send_ip(self, src: bytes, dst: bytes, proto: int, payload: bytes,
                dst_mac: bytes = None, df: bool = True):
        if dst_mac is None:
            dst_mac = self.phone_mac or BCAST_MAC
        self.ip_id = (self.ip_id + 1) & 0xFFFF
        if 20 + len(payload) <= MTU:
            hdr = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20 + len(payload), self.ip_id,
                              0x4000 if df else 0, 64, proto, 0, src, dst)
            hdr = hdr[:10] + struct.pack("!H", csum(hdr)) + hdr[12:]
            self.send_eth(dst_mac, ETH_IP, hdr + payload)
            return
        # Fragment (DF cleared) - only happens for large UDP/DNS replies.
        chunk = (MTU - 20) & ~7
        off = 0
        self.c["ip_frag_out"] += 1
        while off < len(payload):
            part = payload[off:off + chunk]
            more = 0x2000 if off + len(part) < len(payload) else 0
            hdr = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20 + len(part), self.ip_id,
                              more | (off >> 3), 64, proto, 0, src, dst)
            hdr = hdr[:10] + struct.pack("!H", csum(hdr)) + hdr[12:]
            self.send_eth(dst_mac, ETH_IP, hdr + part)
            off += len(part)

    def send_udp(self, src: bytes, sport: int, dst: bytes, dport: int, payload: bytes,
                 dst_mac: bytes = None):
        ulen = 8 + len(payload)
        if ulen > 0xFFFF:
            self.drop("udp_too_big")
            return
        hdr = struct.pack("!HHHH", sport, dport, ulen, 0)
        c = csum(pseudo(src, dst, PROTO_UDP, ulen) + hdr + payload)
        if c == 0:
            c = 0xFFFF
        hdr = hdr[:6] + struct.pack("!H", c)
        self.send_ip(src, dst, PROTO_UDP, hdr + payload, dst_mac=dst_mac)

    def send_tcp_raw(self, src: bytes, sport: int, dport: int, seq: int, ack: int,
                     flags: int, wnd: int, payload: bytes = b"", opts: bytes = b""):
        hl = 20 + len(opts)
        hdr = struct.pack("!HHIIBBHHH", sport, dport, seq & M32, ack & M32,
                          (hl // 4) << 4, flags, wnd, 0, 0) + opts
        seg = hdr + payload
        c = csum(pseudo(src, PHONE_IP, PROTO_TCP, len(seg)) + seg)
        seg = seg[:16] + struct.pack("!H", c) + seg[18:]
        self.send_ip(src, PHONE_IP, PROTO_TCP, seg)

    # ------------------------------------------------------------- frame in
    def handle_frame(self, frame: bytes):
        self.c["frames_in"] += 1
        self.c["bytes_in"] += len(frame)
        n = len(frame)
        if n < 14:
            return self.drop("runt")
        if n > 1518:
            return self.drop("giant")
        dst = frame[0:6]
        src = frame[6:12]
        et = (frame[12] << 8) | frame[13]
        if src[0] & 1:
            return self.drop("bad_src_mac")
        if not (dst[0] & 1) and dst != GW_MAC:
            return self.drop("not_for_us")
        if et == ETH_IP:
            self.handle_ip(frame, src)
        elif et == ETH_ARP:
            self.handle_arp(frame[14:], src)
        elif et == ETH_IP6:
            self.c["ignored_ipv6"] += 1
        else:
            self.c["ignored_ethertype"] += 1

    def learn_mac(self, mac: bytes):
        if mac != self.phone_mac:
            self.phone_mac = mac
            log("phone MAC is %s" % mac.hex(":"))

    # ----------------------------------------------------------------- ARP
    def handle_arp(self, a: bytes, src_mac: bytes):
        if len(a) < 28:
            return self.drop("arp_short")
        htype, ptype, hlen, plen, op = struct.unpack("!HHBBH", a[:8])
        if htype != 1 or ptype != 0x0800 or hlen != 6 or plen != 4:
            return self.drop("arp_bad")
        sha, spa, tpa = a[8:14], a[14:18], a[24:28]
        if op != 1:
            self.c["arp_other"] += 1
            return
        if tpa == PHONE_IP or tpa == spa:
            # gratuitous / duplicate-address probe for the phone: never answer
            self.c["arp_probe_ignored"] += 1
            return
        if tpa[:3] != GW_IP[:3] or tpa[3] in (0, 255):
            self.c["arp_offnet"] += 1
            return
        if sha[0] & 1:
            return self.drop("arp_bad_sha")
        if spa == PHONE_IP:
            self.learn_mac(sha)
        reply = struct.pack("!HHBBH", 1, 0x0800, 6, 4, 2) + GW_MAC + tpa + sha + spa
        self.c["arp_reply"] += 1
        self.send_eth(sha, ETH_ARP, reply)

    # ------------------------------------------------------------------ IP
    def handle_ip(self, frame: bytes, src_mac: bytes):
        p = frame[14:]
        if len(p) < 20:
            return self.drop("ip_short")
        vihl = p[0]
        ihl = (vihl & 15) * 4
        if vihl >> 4 != 4 or ihl < 20 or len(p) < ihl:
            return self.drop("ip_bad_hdr")
        tot = (p[2] << 8) | p[3]
        if tot < ihl or tot > len(p):
            return self.drop("ip_bad_len")
        p = p[:tot]
        if csum(p[:ihl]) != 0:
            return self.drop("ip_csum")
        if ((p[6] << 8) | p[7]) & 0x3FFF:
            return self.drop("ip_fragment")
        proto = p[9]
        src = p[12:16]
        dst = p[16:20]
        data = p[ihl:]
        if proto == PROTO_UDP and len(data) >= 8 and data[2] == 0 and data[3] == 67:
            return self.handle_dhcp(src, dst, data, src_mac)
        if src != PHONE_IP:
            return self.drop("ip_src_not_phone")
        self.learn_mac(src_mac)
        if dst == GW_IP:
            return self.handle_local(proto, data)
        if dst == PHONE_IP:
            return self.drop("ip_to_self")
        if dst == SUBNET_BCAST or dst == ALL_ONES or dst[0] >= 224:
            self.c["ignored_bcast_mcast"] += 1
            return
        if not self.dst_allowed(dst):
            self.c["blocked_dst"] += 1
            if proto == PROTO_TCP:
                self.tcp_reject(dst, data)
            return
        if proto == PROTO_TCP:
            self.tcp_input(dst, data)
        elif proto == PROTO_UDP:
            self.udp_input(dst, data)
        elif proto == PROTO_ICMP:
            self.icmp_input(dst, data, external=True)
        else:
            self.drop("ip_proto")

    def dst_allowed(self, ip: bytes) -> bool:
        v = ip_to_int(ip)
        for net, mask in ALWAYS_BLOCKED:
            if v & mask == net:
                return False
        if not self.allow_lan:
            for net, mask in LAN_BLOCKED:
                if v & mask == net:
                    return False
        return True

    def handle_local(self, proto: int, data: bytes):
        if proto == PROTO_ICMP:
            return self.icmp_input(GW_IP, data, external=False)
        if proto == PROTO_UDP:
            u = self.parse_udp(PHONE_IP, GW_IP, data)
            if u is None:
                return
            sport, dport, payload = u
            if dport == 53:
                return self.dns_query(sport, payload)
            return self.drop("udp_local_port")
        if proto == PROTO_TCP:
            if len(data) >= 4 and data[2] == 0 and data[3] == 53:
                return self.tcp_input(GW_IP, data)
            return self.tcp_reject(GW_IP, data)
        self.drop("ip_proto_local")

    def parse_udp(self, src: bytes, dst: bytes, data: bytes):
        if len(data) < 8:
            self.drop("udp_short")
            return None
        sport, dport, ulen, uc = struct.unpack("!HHHH", data[:8])
        if ulen < 8 or ulen > len(data):
            self.drop("udp_bad_len")
            return None
        data = data[:ulen]
        if uc != 0 and csum(pseudo(src, dst, PROTO_UDP, ulen) + data) != 0:
            self.drop("udp_csum")
            return None
        return sport, dport, data[8:]

    # ---------------------------------------------------------------- ICMP
    def icmp_input(self, dst: bytes, data: bytes, external: bool):
        if len(data) < 8:
            return self.drop("icmp_short")
        if csum(data) != 0:
            return self.drop("icmp_csum")
        if data[0] != 8 or data[1] != 0:
            self.c["icmp_ignored"] += 1
            return
        if external and not self.fake_ping_external:
            self.c["icmp_external_dropped"] += 1
            return
        rep = b"\x00\x00\x00\x00" + data[4:]
        rep = rep[:2] + struct.pack("!H", csum(rep)) + rep[4:]
        self.c["icmp_echo_reply"] += 1
        self.send_ip(dst, PHONE_IP, PROTO_ICMP, rep)

    # ---------------------------------------------------------------- DHCP
    def handle_dhcp(self, src: bytes, dst: bytes, data: bytes, src_mac: bytes):
        u = self.parse_udp(src, dst, data)
        if u is None:
            return
        sport, dport, b = u
        if sport != 68 or len(b) < 240 or b[0] != 1 or b[236:240] != DHCP_MAGIC:
            return self.drop("dhcp_bad")
        if b[1] != 1 or b[2] != 6:
            return self.drop("dhcp_htype")
        xid = b[4:8]
        flags = (b[10] << 8) | b[11]
        ciaddr = b[12:16]
        chaddr = b[28:34]
        if chaddr[0] & 1:
            return self.drop("dhcp_bad_chaddr")
        opts = {}
        i = 240
        while i < len(b):
            k = b[i]
            if k == 255:
                break
            if k == 0:
                i += 1
                continue
            if i + 1 >= len(b):
                break
            ln = b[i + 1]
            if i + 2 + ln > len(b):
                break
            opts[k] = b[i + 2:i + 2 + ln]
            i += 2 + ln
        mt = opts.get(53, b"\x00")[0] if opts.get(53) else 0
        self.learn_mac(chaddr)
        reqip = opts.get(50) or (ciaddr if ciaddr != ZERO_IP else None)
        if mt == 1:      # DISCOVER
            self.c["dhcp_discover"] += 1
            self.dhcp_reply(2, xid, flags, ciaddr, PHONE_IP, chaddr)
        elif mt == 3:    # REQUEST
            sid = opts.get(54)
            if sid is not None and sid != GW_IP:
                self.c["dhcp_other_server"] += 1
                return
            if reqip == PHONE_IP:
                self.c["dhcp_ack"] += 1
                self.dhcp_reply(5, xid, flags, ciaddr, PHONE_IP, chaddr)
                log("DHCP ACK 192.168.88.2 to %s" % chaddr.hex(":"))
            else:
                self.c["dhcp_nak"] += 1
                self.dhcp_reply(6, xid, flags | 0x8000, ZERO_IP, ZERO_IP, chaddr)
        elif mt == 8:    # INFORM
            self.c["dhcp_inform"] += 1
            self.dhcp_reply(5, xid, flags, ciaddr, ZERO_IP, chaddr, inform=True)
        else:
            self.c["dhcp_ignored"] += 1

    def dhcp_reply(self, mtype, xid, flags, ciaddr, yiaddr, chaddr, inform=False):
        o = bytearray()

        def opt(k, v):
            o.extend(bytes((k, len(v))) + v)

        opt(53, bytes((mtype,)))
        opt(54, GW_IP)
        if mtype != 6:
            if not inform:
                opt(51, struct.pack("!I", DHCP_LEASE))
                opt(58, struct.pack("!I", DHCP_LEASE // 2))
                opt(59, struct.pack("!I", DHCP_LEASE * 7 // 8))
            opt(1, NETMASK)
            opt(3, GW_IP)
            opt(6, GW_IP)
            opt(28, SUBNET_BCAST)
            opt(26, struct.pack("!H", MTU))
        o.append(255)
        bootp = struct.pack("!BBBBIHH4s4s4s4s16s64s128s", 2, 1, 6, 0,
                            int.from_bytes(xid, "big"), 0, flags & 0x8000, ciaddr,
                            yiaddr, GW_IP, ZERO_IP, chaddr + b"\x00" * 10, b"", b"")
        msg = bootp + DHCP_MAGIC + bytes(o)
        if len(msg) < 300:
            msg += b"\x00" * (300 - len(msg))
        if mtype == 6:
            dip, dmac = ALL_ONES, BCAST_MAC
        elif ciaddr != ZERO_IP:
            dip, dmac = ciaddr, chaddr
        elif flags & 0x8000:
            dip, dmac = ALL_ONES, BCAST_MAC
        else:
            dip, dmac = yiaddr, chaddr
        self.send_udp(GW_IP, 67, dip, 68, msg, dst_mac=dmac)

    # ----------------------------------------------------------------- DNS
    def dns_query(self, sport: int, payload: bytes):
        if len(payload) < 12:
            return self.drop("dns_short")
        if len(self.dns_pending) >= DNS_MAX_PENDING:
            return self.drop("dns_pending_full")
        oid = payload[:2]
        for _ in range(16):
            nid = random.randint(0, 0xFFFF)
            if nid not in self.dns_pending:
                break
        else:
            return self.drop("dns_no_id")
        self.dns_pending[nid] = (sport, oid, time.monotonic())
        try:
            self.dns_sock.send(struct.pack("!H", nid) + payload[2:])
            self.c["dns_query"] += 1
        except OSError:
            self.dns_pending.pop(nid, None)
            self.drop("dns_send")

    def dns_readable(self):
        for _ in range(256):
            try:
                data, addr = self.dns_sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except ConnectionResetError:
                continue
            except OSError:
                break
            if len(data) < 12 or addr[1] != 53 or addr[0] != self.upstream_dns:
                self.drop("dns_foreign")
                continue
            nid = (data[0] << 8) | data[1]
            ent = self.dns_pending.pop(nid, None)
            if ent is None:
                self.drop("dns_unknown_id")
                continue
            sport, oid, _ = ent
            self.c["dns_answer"] += 1
            self.send_udp(GW_IP, 53, PHONE_IP, sport, oid + data[2:])

    # ------------------------------------------------------------- UDP NAT
    def udp_input(self, dst: bytes, data: bytes):
        u = self.parse_udp(PHONE_IP, dst, data)
        if u is None:
            return
        sport, dport, payload = u
        if dport == 0:
            return self.drop("udp_port0")
        key = (sport, dst, dport)
        uf = self.udp_flows.get(key)
        if uf is None:
            while len(self.udp_flows) >= UDP_MAX_FLOWS:
                _, old = self.udp_flows.popitem(last=False)
                self.udp_close(old, pop=False)
                self.c["udp_evicted"] += 1
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                if IS_WIN and hasattr(socket, "SIO_UDP_CONNRESET"):
                    try:
                        s.ioctl(socket.SIO_UDP_CONNRESET, False)
                    except OSError:
                        pass
                s.setblocking(False)
                s.connect((socket.inet_ntoa(dst), dport))
            except OSError:
                s.close()
                return self.drop("udp_socket")
            uf = UdpFlow()
            uf.key, uf.sock, uf.dip, uf.dport, uf.sport = key, s, dst, dport, sport
            self.udp_flows[key] = uf
            self.sockmap[s] = uf
            self.c["udp_flows_opened"] += 1
            self.vlog("udp open %d->%s:%d" % (sport, socket.inet_ntoa(dst), dport))
        else:
            self.udp_flows.move_to_end(key)
        uf.last = time.monotonic()
        try:
            uf.sock.send(payload)
            self.c["udp_up"] += 1
        except OSError:
            self.drop("udp_send")

    def udp_readable(self, uf: UdpFlow):
        for _ in range(64):
            try:
                data = uf.sock.recv(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break  # ICMP errors etc.
            uf.last = time.monotonic()
            self.c["udp_down"] += 1
            self.send_udp(uf.dip, uf.dport, PHONE_IP, uf.sport, data)

    def udp_close(self, uf: UdpFlow, pop=True):
        if pop:
            self.udp_flows.pop(uf.key, None)
        self.sockmap.pop(uf.sock, None)
        try:
            uf.sock.close()
        except OSError:
            pass

    # ----------------------------------------------------------------- TCP
    def tcp_reject(self, dst: bytes, data: bytes):
        """RST a segment that has no flow (RFC 793 reset generation)."""
        if len(data) < 20:
            return
        sport, dport, seq, ack, off, flags = struct.unpack("!HHIIBB", data[:14])
        if flags & RST:
            return
        plen = max(0, len(data) - (off >> 4) * 4)
        self.c["tcp_rst_sent"] += 1
        if flags & ACK:
            self.send_tcp_raw(dst, dport, sport, ack, 0, RST, 0)
        else:
            self.send_tcp_raw(dst, dport, sport, 0,
                              seq + plen + (1 if flags & SYN else 0) + (1 if flags & FIN else 0),
                              RST | ACK, 0)

    def our_wnd(self, f: Flow) -> int:
        g = GLOBAL_BUF_CAP - self.global_buf
        w = max(0, min(TCP_WINDOW, FLOW_BUF_CAP - len(f.rcvbuf), g))
        if g < MSS:
            self.wnd_starved = True
        if f.rcv_adv is not None:
            # never shrink: the right edge we already advertised stays valid
            edge = (f.rcv_adv - f.rcv_nxt) & M32
            if edge < 0x80000000 and edge > w:
                w = edge
        return w

    def tcp_send(self, f: Flow, flags: int, seq: int, payload: bytes = b"", opts: bytes = b""):
        wnd = self.our_wnd(f)
        f.adv_wnd = wnd
        if flags & ACK:
            f.rcv_adv = (f.rcv_nxt + wnd) & M32
        self.send_tcp_raw(f.dip, f.dport, f.sport, seq, f.rcv_nxt if flags & ACK else 0,
                          flags, wnd, payload, opts)
        if flags & ACK:
            f.need_ack = False

    def tcp_input(self, dst: bytes, data: bytes):
        if len(data) < 20:
            return self.drop("tcp_short")
        sport, dport, seq, ack, off, flags, wnd = struct.unpack("!HHIIBBH", data[:16])
        hl = (off >> 4) * 4
        if hl < 20 or hl > len(data):
            return self.drop("tcp_bad_off")
        if csum(pseudo(PHONE_IP, dst, PROTO_TCP, len(data)) + data) != 0:
            return self.drop("tcp_csum")
        if sport == 0 or dport == 0:
            return self.drop("tcp_port0")
        opts = data[20:hl]
        payload = data[hl:]
        key = (sport, dst, dport)
        f = self.flows.get(key)
        now = time.monotonic()

        if flags & RST:
            if f is not None:
                d = (seq - f.rcv_nxt) & M32
                if f.state in (CONNECTING, SYNACK, TIME_WAIT) or d <= TCP_WINDOW:
                    self.vlog("tcp %s RST from phone" % f.name())
                    self.c["tcp_rst_from_phone"] += 1
                    self.tcp_close(f)
            return

        if f is None:
            if flags & SYN and not flags & ACK:
                return self.tcp_new(key, dst, seq, wnd, opts)
            return self.tcp_reject(dst, data)

        f.last_rx = now
        if flags & SYN:
            if f.state == CONNECTING and seq == f.irs:
                return
            if f.state == SYNACK and seq == f.irs and not flags & ACK:
                self.tcp_send_synack(f)
                return
            if f.state == TIME_WAIT or seq != f.irs:
                self.tcp_close(f)
                if not flags & ACK:
                    return self.tcp_new(key, dst, seq, wnd, opts)
                return self.tcp_reject(dst, data)
            f.need_ack = True
            self.ack_set.add(f)
            return

        if not flags & ACK:
            return self.drop("tcp_no_ack")

        if f.state == CONNECTING:
            return self.drop("tcp_early")
        if f.state == TIME_WAIT:
            if flags & FIN or payload:
                self.tcp_send(f, ACK, f.snd_nxt())
            return
        if f.state == SYNACK:
            if ack != f.snd_una:
                return self.tcp_reject(dst, data)
            f.state = EST
            f.rto_at = None
            f.retries = 0
            f.rto = TCP_RTO_INIT
            self.vlog("tcp %s established" % f.name())

        self.tcp_ack(f, ack, wnd, bool(payload) or bool(flags & FIN))
        if payload or flags & FIN:
            self.tcp_data(f, seq, payload, bool(flags & FIN))
        if f.key in self.flows:
            self.tcp_output(f)
            self.tcp_check_close(f)

    def tcp_new(self, key, dst, seq, wnd, opts):
        if len(self.flows) >= TCP_MAX_FLOWS:
            for k, of in list(self.flows.items()):
                if of.state == TIME_WAIT:
                    self.tcp_close(of)
            if len(self.flows) >= TCP_MAX_FLOWS:
                # evict the least recently active established flow, preferring
                # ones with nothing outstanding toward the phone
                victim, vk = None, None
                for of in self.flows.values():
                    if of.state != EST:
                        continue
                    k2 = (of.outstanding(), of.last_act)
                    if victim is None or k2 < vk:
                        victim, vk = of, k2
                if victim is not None:
                    self.c["tcp_evicted"] += 1
                    self.vlog("tcp %s evicted (flow table full)" % victim.name())
                    self.tcp_abort(victim)
            if len(self.flows) >= TCP_MAX_FLOWS:
                self.c["tcp_flow_cap"] += 1
                return self.send_tcp_raw(dst, key[2], key[0], 0, seq + 1, RST | ACK, 0)
        f = Flow()
        f.key = key
        f.sport, f.dip, f.dport = key
        if dst == GW_IP:
            f.rhost, f.rport = self.upstream_dns, 53
        else:
            f.rhost, f.rport = socket.inet_ntoa(dst), f.dport
        f.irs = seq
        f.rcv_nxt = (seq + 1) & M32
        f.iss = random.getrandbits(32)
        f.snd_una = (f.iss + 1) & M32
        f.wnd = wnd
        f.mss = max(64, min(MSS, self.parse_mss(opts)))
        now = time.monotonic()
        f.created = f.last_rx = f.last_act = now
        f.deadline = now + TCP_CONNECT_TIMEOUT
        f.state = CONNECTING
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setblocking(False)
        try:
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            pass
        try:
            err = s.connect_ex((f.rhost, f.rport))
        except OSError as e:
            err = e.errno or -1
        if err not in (0, errno.EINPROGRESS, errno.EWOULDBLOCK, errno.EAGAIN,
                       errno.EALREADY, 10035, 10036):
            s.close()
            self.c["tcp_connect_fail"] += 1
            self.vlog("tcp %s connect failed immediately (%s)" % (f.name(), err))
            return self.send_tcp_raw(dst, f.dport, f.sport, 0, f.rcv_nxt, RST | ACK, 0)
        f.sock = s
        self.flows[key] = f
        self.sockmap[s] = f
        self.c["tcp_flows_opened"] += 1
        self.vlog("tcp %s connecting to %s:%d" % (f.name(), f.rhost, f.rport))

    @staticmethod
    def parse_mss(opts: bytes) -> int:
        i = 0
        while i < len(opts):
            k = opts[i]
            if k == 0:
                break
            if k == 1:
                i += 1
                continue
            if i + 1 >= len(opts):
                break
            ln = opts[i + 1]
            if ln < 2 or i + ln > len(opts):
                break
            if k == 2 and ln == 4:
                return (opts[i + 2] << 8) | opts[i + 3]
            i += ln
        return 536

    def tcp_send_synack(self, f: Flow):
        self.tcp_send(f, SYN | ACK, f.iss, opts=struct.pack("!BBH", 2, 4, MSS))

    def tcp_connected(self, f: Flow):
        err = f.sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
        if err != 0:
            return self.tcp_connect_failed(f, err)
        f.state = SYNACK
        f.deadline = None
        f.rto = TCP_RTO_INIT
        f.rto_at = time.monotonic() + f.rto
        self.tcp_send_synack(f)
        self.vlog("tcp %s connected, SYN-ACK sent" % f.name())

    def tcp_connect_failed(self, f: Flow, why):
        self.c["tcp_connect_fail"] += 1
        self.vlog("tcp %s connect failed (%s)" % (f.name(), why))
        self.tcp_send(f, RST | ACK, 0)
        self.tcp_close(f)

    def tcp_ack(self, f: Flow, ack: int, wnd: int, has_data: bool):
        acked = (ack - f.snd_una) & M32
        limit = len(f.sndbuf) + (1 if f.fin_ever else 0)
        if acked > limit:
            if acked < 0x80000000:   # acks data we never sent
                f.need_ack = True
                self.ack_set.add(f)
            f.wnd = wnd
            return
        if acked == 0:
            if not has_data and f.sent > 0 and wnd == f.wnd:
                f.dupacks += 1
                if f.dupacks == 3:
                    n = min(f.mss, f.sent, len(f.sndbuf))
                    if n > 0:
                        self.c["tcp_fast_retx"] += 1
                        self.tcp_send(f, ACK | PSH, f.snd_una, bytes(f.sndbuf[:n]))
            f.wnd = wnd
            return
        if f.fin_ever and acked == len(f.sndbuf) + 1:
            f.fin_acked = True
            f.fin_sent = True
            acked -= 1
        if acked:
            del f.sndbuf[:acked]
            self.global_buf -= acked
            f.snd_una = (f.snd_una + acked) & M32
            f.sent = max(0, f.sent - acked)
        f.wnd = wnd
        f.retries = 0
        f.dupacks = 0
        f.rto = TCP_RTO_INIT
        f.rto_at = None

    def tcp_data(self, f: Flow, seq: int, payload: bytes, fin: bool):
        if f.phone_fin:
            f.need_ack = True
            self.ack_set.add(f)
            return
        d = (seq - f.rcv_nxt) & M32
        if d >= 0x80000000:                     # starts before rcv_nxt
            back = (f.rcv_nxt - seq) & M32
            if back >= len(payload) + (1 if fin else 0):
                f.need_ack = True               # pure duplicate
                self.ack_set.add(f)
                return
            payload = payload[back:]
            d = 0
        if d == 0:
            self.tcp_accept(f, payload, fin)
            # drain out-of-order queue
            while f.ooo:
                hit = None
                for s in f.ooo:
                    if ((s - f.rcv_nxt) & M32) == 0 or ((s - f.rcv_nxt) & M32) >= 0x80000000:
                        hit = s
                        break
                if hit is None:
                    break
                p, pf = f.ooo.pop(hit)
                f.ooo_bytes -= len(p)
                back = (f.rcv_nxt - hit) & M32
                if back < len(p) + (1 if pf else 0) and not f.phone_fin:
                    self.tcp_accept(f, p[back:], pf)
            self.flush_rcv(f)
        else:
            if (d < self.our_wnd(f) and len(f.ooo) < OOO_MAX_SEGS
                    and f.ooo_bytes + len(payload) <= OOO_MAX_BYTES and seq not in f.ooo):
                f.ooo[seq] = (bytes(payload), fin)
                f.ooo_bytes += len(payload)
                self.c["tcp_ooo_queued"] += 1
            else:
                self.c["tcp_ooo_dropped"] += 1
        f.need_ack = True
        self.ack_set.add(f)

    def tcp_accept(self, f: Flow, payload: bytes, fin: bool):
        room = self.our_wnd(f)
        if len(payload) > room:
            payload = payload[:room]
            fin = False
            self.c["tcp_rx_trimmed"] += 1
        if payload:
            f.rcvbuf += payload
            self.global_buf += len(payload)
            f.last_act = time.monotonic()
            f.rcv_nxt = (f.rcv_nxt + len(payload)) & M32
            f.up += len(payload)
            self.c["tcp_bytes_up"] += len(payload)
        if fin:
            f.rcv_nxt = (f.rcv_nxt + 1) & M32
            f.phone_fin = True
            self.vlog("tcp %s FIN from phone" % f.name())

    def flush_rcv(self, f: Flow):
        if f.sock is None or not f.rcvbuf or f.state != EST:
            return
        try:
            n = f.sock.send(f.rcvbuf[:SOCK_READ_CHUNK])
        except (BlockingIOError, InterruptedError):
            return
        except OSError as e:
            self.vlog("tcp %s socket send error %s" % (f.name(), e))
            return self.tcp_abort(f)
        if n:
            del f.rcvbuf[:n]
            self.global_buf -= n
            f.last_act = time.monotonic()
            new = self.our_wnd(f)
            if new >= f.adv_wnd + 2 * MSS or (f.adv_wnd < MSS <= new):
                f.need_ack = True       # window update
                self.ack_set.add(f)

    def sock_readable(self, f: Flow):
        if f.state != EST or f.sock_eof:
            return
        room = min(FLOW_BUF_CAP - len(f.sndbuf), GLOBAL_BUF_CAP - self.global_buf,
                   SOCK_READ_CHUNK)
        if room <= 0:
            return
        try:
            data = f.sock.recv(room)
        except (BlockingIOError, InterruptedError):
            return
        except OSError as e:
            self.vlog("tcp %s socket recv error %s" % (f.name(), e))
            return self.tcp_abort(f)
        if not data:
            f.sock_eof = True
            self.vlog("tcp %s remote EOF" % f.name())
        else:
            f.sndbuf += data
            self.global_buf += len(data)
            f.last_act = time.monotonic()
            f.down += len(data)
            self.c["tcp_bytes_down"] += len(data)
        self.tcp_output(f)
        self.tcp_check_close(f)

    def tcp_output(self, f: Flow):
        if f.state != EST:
            return
        while True:
            unsent = len(f.sndbuf) - f.sent
            if unsent <= 0:
                break
            room = f.wnd - f.sent
            if room <= 0:
                break
            n = min(f.mss, unsent, room)
            seq = (f.snd_una + f.sent) & M32
            fl = ACK | (PSH if n == unsent else 0)
            self.tcp_send(f, fl, seq, bytes(f.sndbuf[f.sent:f.sent + n]))
            f.sent += n
        if f.sock_eof and not f.fin_sent and f.sent == len(f.sndbuf):
            self.tcp_send(f, FIN | ACK, (f.snd_una + f.sent) & M32)
            f.fin_sent = True
            f.fin_ever = True
            self.vlog("tcp %s FIN to phone" % f.name())
        if f.rto_at is None and (f.outstanding() or len(f.sndbuf) > f.sent):
            f.rto_at = time.monotonic() + f.rto

    def tcp_check_close(self, f: Flow):
        if f.state != EST:
            return
        if f.phone_fin and not f.rcvbuf and not f.shut_wr and f.sock is not None:
            try:
                f.sock.shutdown(socket.SHUT_WR)
            except OSError:
                pass
            f.shut_wr = True
        if f.phone_fin and f.fin_acked and f.shut_wr:
            self.vlog("tcp %s closed cleanly (up %d down %d)" % (f.name(), f.up, f.down))
            self.c["tcp_closed_clean"] += 1
            self.tcp_release_sock(f)
            f.state = TIME_WAIT
            f.rto_at = None
            f.deadline = time.monotonic() + TCP_TIME_WAIT

    def tcp_abort(self, f: Flow):
        if f.state in (SYNACK, EST):
            self.tcp_send(f, RST | ACK, f.snd_nxt())
        self.c["tcp_aborted"] += 1
        self.tcp_close(f)

    def tcp_release_sock(self, f: Flow):
        if f.sock is not None:
            self.sockmap.pop(f.sock, None)
            try:
                f.sock.close()
            except OSError:
                pass
            f.sock = None
        self.global_buf -= len(f.sndbuf) + len(f.rcvbuf)
        if self.global_buf < 0:
            self.global_buf = 0
        f.sndbuf = bytearray()
        f.rcvbuf = bytearray()
        f.ooo = {}
        f.ooo_bytes = 0

    def tcp_close(self, f: Flow):
        self.tcp_release_sock(f)
        self.flows.pop(f.key, None)
        self.ack_set.discard(f)

    def tcp_timer(self, f: Flow, now: float):
        if f.state == CONNECTING:
            if now >= f.deadline:
                self.tcp_connect_failed(f, "timeout")
            return
        if f.state == TIME_WAIT:
            if now >= f.deadline:
                self.tcp_close(f)
            return
        if f.state == EST and self.tcp_idle(f, now):
            return
        if f.rto_at is None or now < f.rto_at:
            return
        if f.state == SYNACK:
            f.retries += 1
            if f.retries > TCP_MAX_RETRIES:
                return self.tcp_abort(f)
            self.c["tcp_retx"] += 1
            self.tcp_send_synack(f)
            f.rto = min(f.rto * 2, TCP_RTO_MAX)
            f.rto_at = now + f.rto
            return
        f.rto_at = None
        if f.outstanding():
            if now - f.last_rx > TCP_DEAD_PEER:
                return self.tcp_abort(f)
            f.retries += 1
            if f.retries > TCP_MAX_RETRIES:
                self.vlog("tcp %s too many retransmissions" % f.name())
                return self.tcp_abort(f)
            self.c["tcp_retx"] += 1
            f.sent = 0                          # go-back-N from snd_una
            if f.fin_sent and not f.fin_acked:
                f.fin_sent = False
            f.rto = min(f.rto * 2, TCP_RTO_MAX)
            if f.wnd == 0 and f.sndbuf:
                self.tcp_send(f, ACK, f.snd_una, bytes(f.sndbuf[:1]))
            self.tcp_output(f)
            if f.rto_at is None:
                f.rto_at = now + f.rto
        elif len(f.sndbuf) > f.sent and f.wnd - f.sent <= 0:
            # zero-window persist probe; does not count against retries
            if now - f.last_rx > TCP_DEAD_PEER:
                return self.tcp_abort(f)
            self.c["tcp_window_probe"] += 1
            self.tcp_send(f, ACK, (f.snd_una + f.sent) & M32,
                          bytes(f.sndbuf[f.sent:f.sent + 1]))
            f.rto = min(f.rto * 2, 5.0)
            f.rto_at = now + f.rto
        else:
            self.tcp_output(f)

    def tcp_idle(self, f: Flow, now: float) -> bool:
        """Idle ageing + keepalive for EST flows. Returns True if f was aborted."""
        idle = now - f.last_act
        crowded = len(self.flows) > TCP_MAX_FLOWS * 3 // 4
        if idle > (TCP_IDLE_CROWDED if crowded else TCP_IDLE_MAX):
            self.c["tcp_idle_abort"] += 1
            self.vlog("tcp %s idle %ds, aborting" % (f.name(), idle))
            self.tcp_abort(f)
            return True
        if f.phone_fin and f.shut_wr and idle > TCP_HALFCLOSE_IDLE:
            self.c["tcp_halfclose_abort"] += 1
            self.vlog("tcp %s half-closed and silent, aborting" % f.name())
            self.tcp_abort(f)
            return True
        if f.outstanding() or f.rto_at is not None:
            f.ka_probes = 0          # RTO / persist / TCP_DEAD_PEER cover this case
            f.ka_at = None
            return False
        if now - f.last_rx < TCP_KEEPALIVE_IDLE:
            f.ka_probes = 0
            f.ka_at = None
            return False
        if f.ka_at is not None and now < f.ka_at:
            return False
        if f.ka_probes >= TCP_KEEPALIVE_PROBES:
            self.c["tcp_keepalive_abort"] += 1
            self.vlog("tcp %s phone did not answer keepalives, aborting" % f.name())
            self.tcp_abort(f)
            return True
        # keepalive: one byte below snd_una; a live phone answers with an ACK, a
        # rebooted one with RST seq == rcv_nxt (accepted by the RST check)
        self.c["tcp_keepalive"] += 1
        self.tcp_send(f, ACK, (f.snd_una - 1) & M32)
        f.ka_probes += 1
        f.ka_at = now + TCP_KEEPALIVE_INTVL
        return False

    # ---------------------------------------------------------- main loop
    def poll(self, r, w, x, timeout):
        """select() abstraction: poll() on POSIX, chunked select() on Windows."""
        if not IS_WIN:
            p = select.poll()
            objs = {}
            masks = {}
            for lst, m in ((r, select.POLLIN), (w, select.POLLOUT), (x, select.POLLPRI)):
                for s in lst:
                    fd = s.fileno()
                    if fd < 0:
                        continue
                    objs[fd] = s
                    masks[fd] = masks.get(fd, 0) | m
            for fd, m in masks.items():
                p.register(fd, m)
            try:
                ev = p.poll(max(0, timeout) * 1000)
            except InterruptedError:
                return [], [], []
            rr, ww, xx = [], [], []
            for fd, e in ev:
                s = objs[fd]
                m = masks[fd]
                err = e & (select.POLLERR | select.POLLHUP | select.POLLNVAL)
                if m & select.POLLIN and (e & select.POLLIN or err):
                    rr.append(s)
                if m & select.POLLOUT and (e & select.POLLOUT or err):
                    ww.append(s)
                if m & select.POLLPRI and err:
                    xx.append(s)
            return rr, ww, xx
        total = len(r) + len(w) + len(x)
        if total == 0:
            time.sleep(timeout)
            return [], [], []
        if total <= 500:
            try:
                if timeout >= 0.016:
                    return select.select(r, w, x, timeout)
                # Winsock select() rounds short timeouts up to the ~15.6 ms timer
                # tick; time.sleep() is high resolution (3.11+): poll, then sleep.
                res = select.select(r, w, x, 0)
                if timeout > 0 and not (res[0] or res[1] or res[2]):
                    time.sleep(timeout)
                    res = select.select(r, w, x, 0)
                return res
            except (OSError, ValueError):
                time.sleep(0.001)
                return [], [], []
        # Windows FD_SETSIZE is 512: poll in chunks without blocking.
        rr, ww, xx = [], [], []
        for lst, out, kind in ((r, rr, 0), (w, ww, 1), (x, xx, 2)):
            for i in range(0, len(lst), 400):
                part = lst[i:i + 400]
                try:
                    res = select.select(part if kind == 0 else [], part if kind == 1 else [],
                                        part if kind == 2 else [], 0)
                except (OSError, ValueError):
                    continue
                out.extend(res[kind])
        if not (rr or ww or xx):
            time.sleep(min(timeout, 0.002))
        return rr, ww, xx

    def run(self):
        log("bridge up: %s, gateway 192.168.88.1 (%s), phone 192.168.88.2, upstream DNS %s%s"
            % (self.t.describe(), GW_MAC.hex(":"), self.upstream_dns,
               ", LAN allowed" if self.allow_lan else ""))
        tfd = self.t.fileno()
        while self.running:
            now = time.monotonic()
            r, w, x = [], [], []
            if tfd is not None:
                r.append(self.t)
            r.append(self.dns_sock)
            for uf in self.udp_flows.values():
                r.append(uf.sock)
            gbuf = 0
            for f in self.flows.values():
                gbuf += len(f.sndbuf) + len(f.rcvbuf)
            self.global_buf = gbuf
            can_read = gbuf < GLOBAL_BUF_CAP
            for f in self.flows.values():
                if f.sock is None:
                    continue
                if f.state == CONNECTING:
                    w.append(f.sock)
                    x.append(f.sock)
                elif f.state == EST:
                    if can_read and not f.sock_eof and len(f.sndbuf) < FLOW_BUF_CAP:
                        r.append(f.sock)
                    if f.rcvbuf:
                        w.append(f.sock)
            if tfd is None:
                # Polled transport (route U): recv_frames at most every poll_interval
                # (the transport adapts it to traffic), however often sockets wake us.
                timeout = max(0.0, min(self.next_t_poll - now, 0.1))
            else:
                timeout = 0.01 if self.flows or self.dns_pending else 0.1
            rr, ww, xx = self.poll(r, w, x, timeout)
            frames = []
            due = self.t in rr
            if tfd is None:
                due = time.monotonic() >= self.next_t_poll
            if due and time.monotonic() >= self.t_err_until:
                try:
                    frames = self.t.recv_frames()
                except Exception as e:  # transport gone: back off, keep state
                    self.transport_error("recv", e)
                    self.t_err_until = time.monotonic() + 0.2
                    frames = []
                if tfd is None:
                    self.next_t_poll = time.monotonic() + self.t.poll_interval
            for fr in frames:
                try:
                    self.handle_frame(fr)
                except Exception as e:  # never die on a frame
                    self.c["frame_exception"] += 1
                    if self.verbose:
                        log("frame handler exception: %r" % (e,))
            for s in xx:
                f = self.sockmap.get(s)
                if isinstance(f, Flow) and f.state == CONNECTING:
                    self.guard(f, self.tcp_connect_failed, f, "connect error")
            for s in rr:
                if s is self.t:
                    continue
                if s is self.dns_sock:
                    self.guard(None, self.dns_readable)
                    continue
                o = self.sockmap.get(s)
                if isinstance(o, Flow):
                    self.guard(o, self.sock_readable, o)
                elif isinstance(o, UdpFlow):
                    self.guard(o, self.udp_readable, o)
            for s in ww:
                f = self.sockmap.get(s)
                if not isinstance(f, Flow):
                    continue
                if f.state == CONNECTING:
                    self.guard(f, self.tcp_connected, f)
                elif f.state == EST:
                    self.guard(f, self.flush_rcv, f)
                    if f.key in self.flows:
                        self.guard(f, self.tcp_check_close, f)
            if self.wnd_starved and self.global_buf + MSS <= GLOBAL_BUF_CAP:
                # global budget freed up: reopen windows we had closed
                self.wnd_starved = False
                for f in self.flows.values():
                    if f.state == EST and f.adv_wnd < MSS <= self.our_wnd(f):
                        f.need_ack = True
                        self.ack_set.add(f)
            if self.ack_set:
                for f in list(self.ack_set):
                    if f.need_ack and f.key in self.flows and f.state in (EST, TIME_WAIT):
                        self.tcp_send(f, ACK, f.snd_nxt())
                    f.need_ack = False
                self.ack_set.clear()
            now = time.monotonic()
            if now >= self.next_timer_scan:
                self.next_timer_scan = now + 0.01
                for f in list(self.flows.values()):
                    if f.key in self.flows:
                        self.guard(f, self.tcp_timer, f, now)
                if self.dns_pending:
                    for k in [k for k, v in self.dns_pending.items() if now - v[2] > DNS_TIMEOUT]:
                        del self.dns_pending[k]
                        self.c["dns_timeout"] += 1
                while self.udp_flows:
                    k, uf = next(iter(self.udp_flows.items()))
                    if now - uf.last < UDP_IDLE:
                        break
                    self.udp_flows.popitem(last=False)
                    self.udp_close(uf, pop=False)
                    self.c["udp_expired"] += 1
            if self.next_stats is not None and now >= self.next_stats:
                self.next_stats = now + self.stats_every
                self.print_stats()
                if self.stats_hook is not None:
                    try:
                        self.stats_hook()
                    except Exception:
                        pass

    def stop(self):
        """Make run() return after its current iteration (any thread may call it)."""
        self.running = False

    def guard(self, o, fn, *args):
        """Run one per-socket/per-flow handler; a bug there kills only that flow."""
        try:
            fn(*args)
        except Exception as e:
            self.c["handler_exception"] += 1
            log("handler %s exception: %r" % (getattr(fn, "__name__", "?"), e))
            try:
                if isinstance(o, Flow):
                    if o.key in self.flows:
                        self.tcp_abort(o)
                elif isinstance(o, UdpFlow):
                    self.udp_close(o)
            except Exception:
                pass

    def print_stats(self):
        c = self.c
        drops = sum(v for k, v in c.items() if k.startswith("drop_"))
        top = ",".join("%s=%d" % (k[5:], v) for k, v in c.most_common() if k.startswith("drop_"))
        states = Counter(f.state for f in self.flows.values())
        log("stats up=%ds frames in=%d out=%d txfail=%d | tcp flows=%d %s opened=%d "
            "up=%dB down=%dB retx=%d rst=%d | udp flows=%d up=%d down=%d | dns q=%d a=%d "
            "pend=%d | arp=%d dhcp_ack=%d icmp=%d | ipv6=%d other=%d | buf=%dB | terr=%d hexc=%d "
            "ka=%d idle_abort=%d evict=%d | drops=%d %s"
            % (time.monotonic() - self.started, c["frames_in"], c["frames_out"], c["tx_fail"],
               len(self.flows), dict(states), c["tcp_flows_opened"], c["tcp_bytes_up"],
               c["tcp_bytes_down"], c["tcp_retx"], c["tcp_rst_sent"], len(self.udp_flows),
               c["udp_up"], c["udp_down"], c["dns_query"], c["dns_answer"],
               len(self.dns_pending), c["arp_reply"], c["dhcp_ack"], c["icmp_echo_reply"],
               c["ignored_ipv6"], c["ignored_ethertype"], self.global_buf, c["transport_err"],
               c["handler_exception"], c["tcp_keepalive"],
               c["tcp_idle_abort"] + c["tcp_halfclose_abort"] + c["tcp_keepalive_abort"],
               c["tcp_evicted"], drops, top))
        if isinstance(self.t, RingTransport) or self.verbose:
            log("transport: " + self.t.describe())


# ----------------------------------------------------------------------------
# Upstream DNS auto-detection
# ----------------------------------------------------------------------------
_IPV4_RE = re.compile(r"\b(\d{1,3}(?:\.\d{1,3}){3})\b")


def _usable_dns(ip: str) -> bool:
    try:
        b = socket.inet_aton(ip)
    except OSError:
        return False
    return b[0] not in (0, 255) and b[0] < 224 and ip.count(".") == 3


def detect_upstream_dns() -> str:
    cands = []
    if IS_WIN:
        cmds = [
            ["powershell", "-NoProfile", "-NonInteractive", "-Command",
             "Get-NetIPConfiguration | Where-Object { $_.IPv4DefaultGateway } | "
             "ForEach-Object { $_.DNSServer } | Where-Object { $_.AddressFamily -eq 2 } | "
             "ForEach-Object { $_.ServerAddresses }"],
            ["powershell", "-NoProfile", "-NonInteractive", "-Command",
             "Get-DnsClientServerAddress -AddressFamily IPv4 | "
             "ForEach-Object { $_.ServerAddresses }"],
        ]
        for cmd in cmds:
            try:
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=15,
                                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)
                                     ).stdout
            except (OSError, subprocess.SubprocessError):
                continue
            cands += [ip for ip in _IPV4_RE.findall(out or "")]
            if cands:
                break
    else:
        try:
            with open("/etc/resolv.conf", encoding="utf-8", errors="replace") as fh:
                for line in fh:
                    parts = line.split()
                    if len(parts) >= 2 and parts[0] == "nameserver":
                        cands.append(parts[1])
        except OSError:
            pass
    for ip in cands:
        if _usable_dns(ip) and (not ip.startswith("127.") or not IS_WIN):
            return ip
    return "1.1.1.1"


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="User-mode IPv4 gateway for the iOS-on-OnePlus-11 en0 interface.")
    ap.add_argument("--transport", choices=("udp", "sim"), default="udp",
                    help="udp: frames as UDP datagrams; sim: route U over the ring "
                         "simulator (fuxi_usbnet_ring.sim_pair), whose phone side "
                         "exchanges frames as UDP datagrams on --listen/--peer. With a "
                         "phone, route U runs inside fuxi-xnu-monitor.py --net.")
    ap.add_argument("--listen", default="127.0.0.1:47001",
                    help="UDP/sim transport: local HOST:PORT to receive frames on")
    ap.add_argument("--peer", default="127.0.0.1:47002",
                    help="UDP/sim transport: HOST:PORT that frames are sent to")
    ap.add_argument("--upstream-dns", default=None,
                    help="resolver for queries sent to 192.168.88.1 (default: OS, else 1.1.1.1)")
    ap.add_argument("--allow-lan", action="store_true",
                    help="permit RFC1918 / link-local destinations")
    ap.add_argument("--fake-ping-external", action="store_true",
                    help="answer ICMP echo to any external IP locally")
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("--stats-every", type=float, default=30.0, metavar="SECONDS",
                    help="periodic stats line interval (0 = off)")
    a = ap.parse_args(argv)

    if not a.upstream_dns:
        log("detecting upstream DNS from the OS (use --upstream-dns to skip)...")
    dns = a.upstream_dns or detect_upstream_dns()
    try:
        socket.inet_aton(dns)
    except OSError:
        ap.error("--upstream-dns must be an IPv4 address")
    if a.transport == "udp":
        t = UdpFrameTransport(parse_hostport(a.listen), parse_hostport(a.peer))
    else:
        ring = load_ring_module()
        usbnet, _loader, kernel = ring.sim_pair()
        t = RingTransport(transport=usbnet)
        phone = SimPhone(kernel, usbnet.lock, parse_hostport(a.listen), parse_hostport(a.peer))
        phone.start()
        log("sim: route U ring simulator, " + phone.describe())
    b = Bridge(t, dns, allow_lan=a.allow_lan, fake_ping_external=a.fake_ping_external,
               verbose=a.verbose, stats_every=a.stats_every)
    try:
        b.run()
    except KeyboardInterrupt:
        log("interrupted")
        b.print_stats()
        if a.transport == "sim":
            log(phone.describe())
    finally:
        t.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
