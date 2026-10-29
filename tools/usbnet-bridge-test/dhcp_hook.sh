#!/bin/sh
# Lease hook for dhclient (-sf) and udhcpc (-s). It only records the lease;
# run_netns_test.sh applies the address itself. Output: $LEASE_FILE
LEASE_FILE="${LEASE_FILE:-/tmp/fuxi-bridge-test/lease.env}"
case "${reason:-}" in
  PREINIT) ip link set "${interface}" up ;;
  BOUND|RENEW|REBIND|REBOOT)
    {
      echo "ip=${new_ip_address}"
      echo "mask=${new_subnet_mask}"
      echo "router=$(echo ${new_routers} | cut -d' ' -f1)"
      echo "dns=$(echo ${new_domain_name_servers} | cut -d' ' -f1)"
      echo "mtu=${new_interface_mtu}"
      echo "lease=${new_dhcp_lease_time}"
    } > "$LEASE_FILE" ;;
esac
case "${1:-}" in
  bound|renew)
    {
      echo "ip=${ip}"
      echo "mask=${subnet}"
      echo "router=$(echo ${router} | cut -d' ' -f1)"
      echo "dns=$(echo ${dns} | cut -d' ' -f1)"
      echo "mtu=${mtu}"
      echo "lease=${lease}"
    } > "$LEASE_FILE" ;;
esac
exit 0
