#!/bin/bash
# Boot-time USB reset for the RealSense cameras: the part of Unitree's
# ~/Unitree/autostart/b1camera_start.sh we still need after disabling its
# video-streaming containers. Without it a camera can come up stuck
# ("Set SEL ... failed" in dmesg, missing from rs-enumerate-devices).
#
# Installed on each NX as ~/projects/fetch/boot/usb_reset.sh and started at
# login by ~/.config/autostart/fetch_usb_reset.desktop. Safe to run by hand
# (with no camera drivers running). Log: ~/projects/fetch/boot/usb_reset.log
export SUDO_ASKPASS=/home/unitree/Unitree/autostart/passwd.sh
exec >> ~/projects/fetch/boot/usb_reset.log 2>&1
echo "== $(date) usb reset"
sudo -A rmmod uvcvideo
sudo -A /sbin/hub_power_reset
sleep 5
sudo -A modprobe uvcvideo
sleep 5
rs-enumerate-devices -s 2>&1 | tail -n +2
