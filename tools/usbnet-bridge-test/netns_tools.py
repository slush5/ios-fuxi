#!/usr/bin/env python3
"""Helpers for run_netns_test.sh (Linux/WSL only, run as root).

  relay   TAP (inside a netns) <-> UDP frame relay to the bridge transport
  dhcp    minimal DHCP client over AF_PACKET (fallback when no dhclient/udhcpc)
  dnsq    send one DNS A query (UDP or TCP) and print the answers
  fuzz    blast malformed frames out of an interface (bridge must survive)
"""
import argparse
import fcntl
import os
import random
import select
import socket
import struct
import sys
import time

TUNSETIFF = 0x400454CA
IFF_TAP = 0x0002
IFF_NO_PI = 0x1000


def setns_net(path):
    fd = os.open(path, os.O_RDONLY)
    try:
        if hasattr(os, "setns"):
            os.setns(fd, getattr(os, "CLONE_NEWNET", 0x40000000))
        else:
            import ctypes
            libc = ctypes.CDLL(None, use_errno=True)
            if libc.setns(fd, 0x40000000) != 0:
                raise OSError(ctypes.get_errno(), "setns failed")
    finally:
        os.close(fd)


def hostport(s):
    h, p = s.rsplit(":", 1)
    return h, int(p)


def cmd_relay(a):
    # UDP socket is created in the ROOT namespace, then we enter the netns
    # to attach to the TAP device; the socket stays in the root namespace.
    us = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    us.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    us.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)
    us.bind(hostport(a.listen))
    peer = hostport(a.peer)
    setns_net(a.netns)
    tfd = os.open("/dev/net/tun", os.O_RDWR)
    pi = False
    try:
        fcntl.ioctl(tfd, TUNSETIFF, struct.pack("16sH", a.tap.encode(), IFF_TAP | IFF_NO_PI))
    except OSError:
        fcntl.ioctl(tfd, TUNSETIFF, struct.pack("16sH", a.tap.encode(), IFF_TAP))
        pi = True
    print("relay: attached to %s (pi=%s), udp %s <-> %s" % (a.tap, pi, a.listen, a.peer), flush=True)
    os.set_blocking(tfd, False)
    us.setblocking(False)
    n_up = n_down = 0
    last = time.time()
    while True:
        r, _, _ = select.select([tfd, us], [], [], 1.0)
        if tfd in r:
            for _ in range(256):
                try:
                    fr = os.read(tfd, 65536)
                except BlockingIOError:
                    break
                if pi:
                    fr = fr[4:]
                try:
                    us.sendto(fr, peer)
                    n_up += 1
                except OSError:
                    pass
        if us in r:
            for _ in range(256):
                try:
                    fr, addr = us.recvfrom(65536)
                except BlockingIOError:
                    break
                except OSError:
                    break
                if pi:
                    fr = b"\x00\x00\x08\x00" + fr
                try:
                    os.write(tfd, fr)
                    n_down += 1
                except OSError:
                    pass
        if time.time() - last > 10:
            last = time.time()
            print("relay: frames phone->bridge %d bridge->phone %d" % (n_up, n_down), flush=True)


# ------------------------------------------------------------------ DHCP
def csum(b):
    if len(b) & 1:
        b += b"\0"
    s = sum(struct.unpack("!%dH" % (len(b) // 2), b))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return ~s & 0xFFFF


def mac_of(ifname):
    with open("/sys/class/net/%s/address" % ifname) as fh:
        return bytes.fromhex(fh.read().strip().replace(":", ""))


def dhcp_frame(mac, xid, mtype, extra=b""):
    opts = bytes((53, 1, mtype)) + extra + bytes((55, 4, 1, 3, 6, 26)) + b"\xff"
    bootp = struct.pack("!BBBBIHH4s4s4s4s16s64s128s", 1, 1, 6, 0, xid, 0, 0x8000,
                        b"\0" * 4, b"\0" * 4, b"\0" * 4, b"\0" * 4, mac + b"\0" * 10, b"", b"")
    payload = bootp + b"\x63\x82\x53\x63" + opts
    udp = struct.pack("!HHHH", 68, 67, 8 + len(payload), 0) + payload
    src, dst = b"\0" * 4, b"\xff" * 4
    c = csum(src + dst + struct.pack("!BBH", 0, 17, len(udp)) + udp) or 0xFFFF
    udp = udp[:6] + struct.pack("!H", c) + udp[8:]
    ip = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20 + len(udp), random.randint(0, 65535), 0, 64, 17, 0, src, dst)
    ip = ip[:10] + struct.pack("!H", csum(ip)) + ip[12:]
    return b"\xff" * 6 + mac + b"\x08\x00" + ip + udp


def dhcp_parse(fr, xid):
    if len(fr) < 14 + 20 + 8 + 240 or fr[12:14] != b"\x08\x00":
        return None
    ihl = (fr[14] & 15) * 4
    if fr[14 + 9] != 17:
        return None
    u = fr[14 + ihl:]
    if struct.unpack("!H", u[2:4])[0] != 68:
        return None
    b = u[8:]
    if len(b) < 240 or b[0] != 2 or struct.unpack("!I", b[4:8])[0] != xid:
        return None
    opts = {}
    i = 240
    while i < len(b) and b[i] != 255:
        if b[i] == 0:
            i += 1
            continue
        opts[b[i]] = b[i + 2:i + 2 + b[i + 1]]
        i += 2 + b[i + 1]
    return b[16:20], opts


def cmd_dhcp(a):
    mac = mac_of(a.iface)
    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
    s.bind((a.iface, 0))
    xid = random.getrandbits(32)

    def exchange(frame, want, timeout=3.0):
        for _ in range(3):
            s.send(frame)
            end = time.time() + timeout
            while time.time() < end:
                r, _, _ = select.select([s], [], [], end - time.time())
                if not r:
                    break
                res = dhcp_parse(s.recv(4096), xid)
                if res and res[1].get(53, b"\0")[0] in want:
                    return res
        return None

    off = exchange(dhcp_frame(mac, xid, 1), (2,))
    if not off:
        print("DHCP: no OFFER", file=sys.stderr)
        return 1
    yi, o = off
    ack = exchange(dhcp_frame(mac, xid, 3, bytes((50, 4)) + yi + bytes((54, 4)) + o.get(54, b"")), (5, 6))
    if not ack or ack[1][53][0] != 5:
        print("DHCP: no ACK", file=sys.stderr)
        return 1
    yi, o = ack
    ntoa = socket.inet_ntoa
    print("ip=%s" % ntoa(yi))
    print("mask=%s" % ntoa(o.get(1, b"\xff\xff\xff\x00")))
    print("router=%s" % ntoa(o[3][:4]) if 3 in o else "router=")
    print("dns=%s" % ntoa(o[6][:4]) if 6 in o else "dns=")
    print("mtu=%d" % struct.unpack("!H", o[26])[0] if 26 in o else "mtu=")
    print("lease=%d" % struct.unpack("!I", o[51])[0] if 51 in o else "lease=")
    return 0


# ------------------------------------------------------------------ DNS
def cmd_dnsq(a):
    tid = random.getrandbits(16)
    q = struct.pack("!HHHHHH", tid, 0x0100, 1, 0, 0, 0)
    for part in a.name.split("."):
        q += bytes((len(part),)) + part.encode()
    q += b"\0" + struct.pack("!HH", 1, 1)
    if a.tcp:
        s = socket.create_connection((a.server, 53), timeout=5)
        s.sendall(struct.pack("!H", len(q)) + q)
        buf = b""
        while len(buf) < 2 or len(buf) < 2 + struct.unpack("!H", buf[:2])[0]:
            d = s.recv(4096)
            if not d:
                break
            buf += d
        r = buf[2:]
    else:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(3)
        r = None
        for _ in range(3):
            s.sendto(q, (a.server, 53))
            try:
                r, _ = s.recvfrom(4096)
                break
            except socket.timeout:
                continue
        if r is None:
            print("no answer", file=sys.stderr)
            return 1
    if len(r) < 12 or struct.unpack("!H", r[:2])[0] != tid:
        print("bad answer", file=sys.stderr)
        return 1
    an = struct.unpack("!H", r[6:8])[0]
    i = 12

    def skip_name(i):
        while True:
            ln = r[i]
            if ln == 0:
                return i + 1
            if ln & 0xC0 == 0xC0:
                return i + 2
            i += 1 + ln

    i = skip_name(i) + 4
    ips = []
    for _ in range(an):
        i = skip_name(i)
        typ, cls, ttl, rdl = struct.unpack("!HHIH", r[i:i + 10])
        i += 10
        if typ == 1 and rdl == 4:
            ips.append(socket.inet_ntoa(r[i:i + 4]))
        i += rdl
    print(" ".join(ips))
    return 0 if ips else 1


# ------------------------------------------------------------------ fuzz
def cmd_fuzz(a):
    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
    s.bind((a.iface, 0))
    mac = mac_of(a.iface)
    gw = bytes.fromhex("0246555849fe")
    rnd = random.Random(1234)
    ip = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 40, 1, 0x4000, 64, 6, 0,
                     socket.inet_aton("192.168.88.2"), socket.inet_aton("93.184.216.34"))
    ip = ip[:10] + struct.pack("!H", csum(ip)) + ip[12:]
    good_tcp = ip + struct.pack("!HHIIBBHHH", 40000, 80, 1, 0, 0x50, 0x02, 65535, 0, 0)
    sent = 0
    for n in range(a.count):
        kind = n % 6
        if kind == 0:
            body = bytes(rnd.getrandbits(8) for _ in range(rnd.randint(0, 80)))
            et = rnd.choice([b"\x08\x00", b"\x08\x06", b"\x86\xdd", b"\x12\x34"])
        elif kind == 1:
            body = good_tcp[:rnd.randint(0, len(good_tcp))]
            et = b"\x08\x00"
        elif kind == 2:
            b = bytearray(good_tcp)
            for _ in range(3):
                b[rnd.randrange(len(b))] = rnd.getrandbits(8)
            body, et = bytes(b), b"\x08\x00"
        elif kind == 3:
            body = bytes([0x4F]) + bytes(rnd.getrandbits(8) for _ in range(40))
            et = b"\x08\x00"
        elif kind == 4:
            body = bytes(rnd.getrandbits(8) for _ in range(rnd.randint(0, 40)))
            et = b"\x08\x06"
        else:
            body = bytes(rnd.getrandbits(8) for _ in range(rnd.randint(0, 1400)))
            et = b"\x08\x00"
        fr = gw + mac + et + body
        if len(fr) < 14:
            continue
        try:
            s.send(fr)
            sent += 1
        except OSError:
            pass
    print("fuzz: sent %d frames" % sent)
    return 0


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)
    p = sp.add_parser("relay")
    p.add_argument("--netns", required=True)
    p.add_argument("--tap", required=True)
    p.add_argument("--listen", required=True)
    p.add_argument("--peer", required=True)
    p = sp.add_parser("dhcp")
    p.add_argument("iface")
    p = sp.add_parser("dnsq")
    p.add_argument("--server", required=True)
    p.add_argument("--tcp", action="store_true")
    p.add_argument("name")
    p = sp.add_parser("fuzz")
    p.add_argument("iface")
    p.add_argument("--count", type=int, default=3000)
    a = ap.parse_args()
    return {"relay": cmd_relay, "dhcp": cmd_dhcp, "dnsq": cmd_dnsq, "fuzz": cmd_fuzz}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main() or 0)
