#!/usr/bin/env bash
# End-to-end test of fuxi-usb-net-bridge.py inside WSL Ubuntu (run as root).
#
#   wsl -d Ubuntu -u root -- bash "/mnt/c/Users/Jordan/ios oneplus 11/op11_home_kit_14711/OnePlus11_iOS_home_kit/windows-tools/bridge-test/run_netns_test.sh"
#
# Layout:
#   netns "phone":  tap0 (plays the iPhone's en0)
#        |  TAP fd
#   netns_tools.py relay (UDP socket in root ns)  <-- UDP frames -->  bridge (root ns)
#                                                                       |
#                                                       ordinary sockets -> internet
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE="$HERE/../fuxi-usb-net-bridge.py"
TOOLS="$HERE/netns_tools.py"
HOOK_SRC="$HERE/dhcp_hook.sh"
WORK=/tmp/fuxi-bridge-test
NS=phone
TAP=tap0
BR_ADDR=127.0.0.1:47001
RELAY_ADDR=127.0.0.1:47002
PIDS=()
RESULTS=()
FAILS=0

say() { echo "[test] $*"; }
pass() { RESULTS+=("PASS  $1"); say "PASS: $1"; }
fail() { RESULTS+=("FAIL  $1"); FAILS=$((FAILS+1)); say "FAIL: $1"; }
nsx() { ip netns exec "$NS" "$@"; }

cleanup() {
  set +e
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  [ -f "$WORK/dhclient.pid" ] && kill "$(cat "$WORK/dhclient.pid")" 2>/dev/null
  pkill -f "netns_tools.py relay" 2>/dev/null
  sleep 0.3
  for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done
  ip netns del "$NS" 2>/dev/null
  rm -rf "/etc/netns/$NS"
}
trap cleanup EXIT INT TERM

if [ "$(id -u)" != 0 ]; then echo "must run as root"; exit 2; fi
rm -rf "$WORK"; mkdir -p "$WORK"
# CRLF-safe copy of the hook
tr -d '\r' < "$HOOK_SRC" > "$WORK/dhcp_hook.sh"; chmod +x "$WORK/dhcp_hook.sh"
export LEASE_FILE="$WORK/lease.env"

say "kernel $(uname -r), $(python3 --version)"
[ -c /dev/net/tun ] || { modprobe tun 2>/dev/null; }
[ -c /dev/net/tun ] || { echo "no /dev/net/tun"; exit 2; }

# ---------------------------------------------------------------- namespace
ip netns del "$NS" 2>/dev/null
ip netns add "$NS"
ip -n "$NS" link set lo up
ip -n "$NS" tuntap add dev "$TAP" mode tap
mkdir -p "/etc/netns/$NS"
echo "nameserver 192.168.88.1" > "/etc/netns/$NS/resolv.conf"
printf 'passwd: files\ngroup: files\nhosts: files dns\n' > "/etc/netns/$NS/nsswitch.conf"

# ---------------------------------------------------------------- bridge + relay
python3 -u "$BRIDGE" --transport udp --listen "$BR_ADDR" --peer "$RELAY_ADDR" \
  --stats-every 5 --verbose > "$WORK/bridge.log" 2>&1 &
BRIDGE_PID=$!; PIDS+=("$BRIDGE_PID")
python3 -u "$TOOLS" relay --netns "/run/netns/$NS" --tap "$TAP" \
  --listen "$RELAY_ADDR" --peer "$BR_ADDR" > "$WORK/relay.log" 2>&1 &
PIDS+=("$!")
sleep 1.5
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then cat "$WORK/bridge.log"; fail "bridge start"; fi
cat "$WORK/relay.log"
ip -n "$NS" link set "$TAP" up
sleep 0.5

# ---------------------------------------------------------------- DHCP
DHCP_TOOL=""
if command -v dhclient >/dev/null; then DHCP_TOOL=dhclient
elif command -v udhcpc >/dev/null; then DHCP_TOOL=udhcpc
elif command -v busybox >/dev/null && busybox --list 2>/dev/null | grep -qx udhcpc; then DHCP_TOOL=busybox
else
  say "no DHCP client found; trying apt-get -y install busybox"
  if timeout 120 apt-get -y -q install busybox >"$WORK/apt.log" 2>&1 \
     || { timeout 180 apt-get -q update >>"$WORK/apt.log" 2>&1 && timeout 120 apt-get -y -q install busybox >>"$WORK/apt.log" 2>&1; }; then
    busybox --list 2>/dev/null | grep -qx udhcpc && DHCP_TOOL=busybox
  fi
  [ -z "$DHCP_TOOL" ] && DHCP_TOOL=python
fi
say "DHCP client: $DHCP_TOOL"
rm -f "$LEASE_FILE"
case "$DHCP_TOOL" in
  dhclient) nsx timeout 30 dhclient -1 -v -sf "$WORK/dhcp_hook.sh" -pf "$WORK/dhclient.pid" \
              -lf "$WORK/dhclient.leases" "$TAP" > "$WORK/dhcp.log" 2>&1 ;;
  udhcpc)   nsx timeout 30 udhcpc -i "$TAP" -f -q -n -t 5 -T 2 -s "$WORK/dhcp_hook.sh" > "$WORK/dhcp.log" 2>&1 ;;
  busybox)  nsx timeout 30 busybox udhcpc -i "$TAP" -f -q -n -t 5 -T 2 -s "$WORK/dhcp_hook.sh" > "$WORK/dhcp.log" 2>&1 ;;
  python)   nsx timeout 30 python3 "$TOOLS" dhcp "$TAP" > "$LEASE_FILE" 2>"$WORK/dhcp.log" ;;
esac
if [ ! -s "$LEASE_FILE" ] && [ "$DHCP_TOOL" != python ]; then
  say "system client gave no lease, falling back to python client"
  sed -n '1,20p' "$WORK/dhcp.log"
  nsx timeout 30 python3 "$TOOLS" dhcp "$TAP" > "$LEASE_FILE" 2>>"$WORK/dhcp.log"
fi
cat "$LEASE_FILE" 2>/dev/null | sed 's/^/[lease] /'
LEASE_IP=$(sed -n 's/^ip=//p' "$LEASE_FILE" 2>/dev/null)
LEASE_GW=$(sed -n 's/^router=//p' "$LEASE_FILE" 2>/dev/null)
LEASE_DNS=$(sed -n 's/^dns=//p' "$LEASE_FILE" 2>/dev/null)
if [ "$LEASE_IP" = 192.168.88.2 ] && [ "$LEASE_GW" = 192.168.88.1 ] && [ "$LEASE_DNS" = 192.168.88.1 ]; then
  pass "DHCP lease ($DHCP_TOOL): ip=$LEASE_IP router=$LEASE_GW dns=$LEASE_DNS"
else
  fail "DHCP lease ($DHCP_TOOL): got ip='$LEASE_IP' router='$LEASE_GW' dns='$LEASE_DNS'"
  LEASE_IP=192.168.88.2; LEASE_GW=192.168.88.1
fi
[ -f "$WORK/dhclient.pid" ] && kill "$(cat "$WORK/dhclient.pid")" 2>/dev/null
nsx ip addr flush dev "$TAP"
nsx ip addr add "$LEASE_IP/24" dev "$TAP"
nsx ip link set "$TAP" mtu 1500 up
nsx ip route replace default via "$LEASE_GW" dev "$TAP"

# ---------------------------------------------------------------- tests
if nsx ping -c2 -W2 192.168.88.1 > "$WORK/ping.log" 2>&1; then
  pass "ping 192.168.88.1 ($(grep -o 'rtt.*' "$WORK/ping.log"))"
else
  cat "$WORK/ping.log"; fail "ping 192.168.88.1"
fi

if nsx ping -c1 -W2 1.1.1.1 >/dev/null 2>&1; then
  fail "external ping should be dropped without --fake-ping-external"
else
  pass "external ping dropped by default"
fi

A=$(nsx python3 "$TOOLS" dnsq --server 192.168.88.1 example.com 2>&1)
[ $? -eq 0 ] && pass "DNS relay UDP via 192.168.88.1: example.com -> $A" || fail "DNS relay UDP: $A"
A=$(nsx python3 "$TOOLS" dnsq --tcp --server 192.168.88.1 example.com 2>&1)
[ $? -eq 0 ] && pass "DNS over TCP via 192.168.88.1: example.com -> $A" || fail "DNS TCP: $A"
A=$(nsx getent ahostsv4 example.com 2>&1 | awk '{print $1}' | sort -u | tr '\n' ' ')
[ -n "$A" ] && pass "getent (netns resolv.conf -> 192.168.88.1): example.com -> $A" || fail "getent example.com"
A=$(nsx python3 "$TOOLS" dnsq --server 1.1.1.1 apple.com 2>&1)
[ $? -eq 0 ] && pass "UDP NAT (direct query to 1.1.1.1:53): apple.com -> $A" || fail "UDP NAT to 1.1.1.1: $A"

CODE=$(nsx curl -4 -sS -m 20 -o /dev/null -w '%{http_code}' http://captive.apple.com 2>&1)
[ "$CODE" = 200 ] && pass "HTTP captive.apple.com -> $CODE" || fail "HTTP captive.apple.com -> $CODE"
BODY=$(nsx curl -4 -sS -m 20 http://captive.apple.com 2>&1)
echo "$BODY" | grep -q Success && pass "captive.apple.com body contains Success" || fail "captive body: $BODY"
CODE=$(nsx curl -4 -sS -m 20 -o /dev/null -w '%{http_code}' https://example.com 2>&1)
[ "$CODE" = 200 ] && pass "HTTPS example.com -> $CODE" || fail "HTTPS example.com -> $CODE"

OUT=$(nsx curl -4 -sS -m 180 -o /dev/null -w '%{http_code} %{size_download} %{speed_download} %{time_total}' \
      'https://speed.cloudflare.com/__down?bytes=10000000' 2>&1)
read -r C S SP T <<<"$OUT"
if [ "$C" = 200 ] && [ "${S%.*}" -ge 10000000 ] 2>/dev/null; then
  MBPS=$(awk -v s="$SP" 'BEGIN{printf "%.2f MB/s (%.1f Mbit/s)", s/1e6, s*8/1e6}')
  pass "10 MB download: $S bytes in ${T}s = $MBPS"
else
  fail "10 MB download: $OUT"
fi

# bulk upload (phone -> internet, exercises the TCP receive path / window)
head -c 4000000 /dev/urandom > "$WORK/up.bin"
OUT=$(nsx curl -4 -sS -m 120 -o /dev/null -w '%{http_code} %{size_upload} %{speed_upload}' \
      --data-binary @"$WORK/up.bin" -H 'Content-Type: application/octet-stream' \
      'https://speed.cloudflare.com/__up' 2>&1)
read -r C S SP <<<"$OUT"
if [ "$C" = 200 ] && [ "${S%.*}" = 4000000 ]; then
  pass "4 MB upload: $S bytes at $(awk -v s="$SP" 'BEGIN{printf "%.2f MB/s", s/1e6}')"
else
  fail "4 MB upload: $OUT"
fi

# 8 concurrent transfers
PAR_PIDS=()
for i in 1 2 3 4 5 6 7 8; do
  ( nsx curl -4 -sS -m 120 -o /dev/null -w '%{http_code} %{size_download}\n' \
      "https://speed.cloudflare.com/__down?bytes=$((i*300000))" > "$WORK/par$i.out" 2>&1 ) &
  PAR_PIDS+=("$!")
done
wait "${PAR_PIDS[@]}"
OK=0; DETAIL=""
for i in 1 2 3 4 5 6 7 8; do
  read -r C S < "$WORK/par$i.out"
  DETAIL="$DETAIL $C/$S"
  [ "$C" = 200 ] && [ "${S%.*}" = $((i*300000)) ] && OK=$((OK+1))
done
[ $OK = 8 ] && pass "8 concurrent HTTPS downloads all OK:$DETAIL" || fail "8 concurrent downloads ($OK/8):$DETAIL"

# blocked destinations get a fast RST
CODE=$(nsx curl -4 -sS -m 5 -o /dev/null -w '%{http_code}' http://127.0.0.1:80 2>&1)
case "$CODE" in *"refused"*|*"Couldn't connect"*|*000*) pass "loopback destination refused";; *) fail "loopback destination: $CODE";; esac

# malformed-frame fuzz, then make sure everything still works
nsx python3 "$TOOLS" fuzz "$TAP" --count 3000 | sed 's/^/[fuzz] /'
sleep 1
if kill -0 "$BRIDGE_PID" 2>/dev/null && nsx ping -c1 -W2 192.168.88.1 >/dev/null 2>&1 \
   && [ "$(nsx curl -4 -sS -m 20 -o /dev/null -w '%{http_code}' http://captive.apple.com 2>&1)" = 200 ]; then
  pass "bridge survived 3000 malformed frames and still forwards"
else
  fail "bridge after fuzz"
fi

sleep 5.5
say "---- bridge log (last stats + any exceptions) ----"
grep -E 'Traceback|exception|Error' "$WORK/bridge.log" | head -20
grep 'stats' "$WORK/bridge.log" | tail -1
kill -0 "$BRIDGE_PID" 2>/dev/null || { fail "bridge process died"; tail -30 "$WORK/bridge.log"; }

echo
echo "================ SUMMARY ================"
for r in "${RESULTS[@]}"; do echo "$r"; done
echo "========================================="
if [ $FAILS -eq 0 ]; then echo "OVERALL: PASS (${#RESULTS[@]} checks)"; exit 0
else echo "OVERALL: FAIL ($FAILS of ${#RESULTS[@]} checks failed)"; exit 1; fi
