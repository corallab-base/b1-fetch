#!/bin/bash
# One-time clock sync setup, run once on each board (needs sudo):
#   sudo ~/projects/fetch/boot/setup_timesync.sh
#
# The boards have no clock battery and boot in 2018. ROS needs their clocks
# to agree (TF, move_base, SLAM, bags all compare stamps across boards), so:
#   - the time server (.23, the head; internet via .165, made permanent by
#     boot/setup_gateway.sh) runs chrony against
#     internet NTP, steps its clock whenever it is off by > 1 s (not just in
#     the first 3 updates, so a late network still fixes a 2018 boot), and
#     serves time to 192.168.123.0/24 -- from its own clock if it has no
#     internet, so the boards at least agree with each other;
#   - every other board syncs from it with systemd-timesyncd, polling at
#     least every 64 s so a step on the server is followed within a minute.
# Originals are kept as *.fetch-orig. Undo: see NOTES.md. Re-running it moves
# the server: the new server installs chrony and stops its timesyncd client,
# a former server stops chrony (config restored) and becomes a client.
set -e
SERVER=${TIME_SERVER:-192.168.123.23}
ME=$(hostname -I | tr ' ' '\n' | grep -m1 '^192\.168\.123\.')
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }

if [ "$ME" = "$SERVER" ]; then
  conf=/etc/chrony/chrony.conf
  if [ ! -f $conf ]; then
    echo "installing chrony (needs internet)"
    apt-get install -y chrony >/dev/null
  fi
  # no longer a client of another board
  rm -f /etc/systemd/timesyncd.conf.d/fetch.conf
  systemctl disable --now systemd-timesyncd 2>/dev/null || true
  [ -f $conf.fetch-orig ] || cp $conf $conf.fetch-orig
  sed -i 's/^makestep .*/makestep 1 -1/' $conf
  if ! grep -q "^# fetch: serve time" $conf; then
    cat >>$conf <<'EOF'

# fetch: serve time to the robot network, even before/without internet
allow 192.168.123.0/24
local stratum 10
EOF
  fi
  systemctl restart chrony
  echo "time server ($ME): chrony restarted"
  sleep 5; chronyc tracking | grep -E "Reference|System time"
else
  # a former time server: stop its chrony and restore the original config
  if systemctl is-active --quiet chrony; then
    systemctl disable --now chrony
    [ ! -f /etc/chrony/chrony.conf.fetch-orig ] || cp /etc/chrony/chrony.conf.fetch-orig /etc/chrony/chrony.conf
    echo "chrony stopped (this board is no longer the time server)"
  fi
  mkdir -p /etc/systemd/timesyncd.conf.d
  cat >/etc/systemd/timesyncd.conf.d/fetch.conf <<EOF
# fetch: sync from the robot's time server (boot/setup_timesync.sh)
[Time]
NTP=$SERVER
FallbackNTP=
PollIntervalMaxSec=64
EOF
  timedatectl set-ntp true
  systemctl restart systemd-timesyncd
  echo "client ($ME): syncing from $SERVER"
  sleep 5; timedatectl | grep -E "Local time|synchronized"
fi
