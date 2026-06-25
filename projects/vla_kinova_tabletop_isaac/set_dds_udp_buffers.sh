#!/bin/bash

# This script increases kernel UDP buffer limits so that DDS can deliver large messages such as camera images
# The previous limits caused <30 fps framerates for the realsense camera for example
# Re-run this after every reboot as these changes are not persistent unless we modify /etc_sysctl.d

set -e

# Higher read and write buffer sizes
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=2147483647
sudo sysctl -w net.core.wmem_max=2147483647
sudo sysctl -w net.core.wmem_default=2147483647

# Shortens the window where a reassembling IP-fragmented datagram holds kernel
# memory, and raises the memory ceiling for in-flight reassembly, so a burst of
# fragmented camera frames doesn't get dropped under reassembly memory pressure.
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728

echo ""
echo "==> DDS UDP buffer limits applied for this boot session."
echo "    Re-run this script after every reboot, before launching cameras."
