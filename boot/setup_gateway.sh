#!/bin/bash
# Gives this board a permanent default route (internet) via the robot's
# gateway, run once with sudo:   sudo ~/projects/fetch/boot/setup_gateway.sh
# Adds "gateway 192.168.123.165" to the eth0 stanza of /etc/network/interfaces
# (applied at every boot; the eth0:0 alias is untouched) and adds the route
# now. Original kept as /etc/network/interfaces.fetch-orig.
# Undo: sudo cp /etc/network/interfaces.fetch-orig /etc/network/interfaces
set -e
GW=${GATEWAY:-192.168.123.165}
F=/etc/network/interfaces
ME=$(hostname -I | tr ' ' '\n' | grep -m1 '^192\.168\.123\.')
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
grep -q "^address $ME\$" $F || { echo "no 'address $ME' line in $F, not touching it"; exit 1; }
if grep -q "^gateway " $F; then
  echo "$F already has a gateway: $(grep '^gateway ' $F)"
else
  [ -f $F.fetch-orig ] || cp $F $F.fetch-orig
  sed -i "/^address $ME\$/a gateway $GW" $F
  echo "added 'gateway $GW' to eth0 in $F"
fi
ip route replace default via $GW dev eth0
ip route | grep default
timeout 5 curl -sI http://google.com >/dev/null && echo "internet OK" || echo "no internet through $GW right now"
