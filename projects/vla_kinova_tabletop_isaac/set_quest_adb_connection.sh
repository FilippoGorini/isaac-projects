#!/bin/bash

# Enables to connect the quest to the pc via usb c cable instead of over wifi
# Wifi connection resulted in lags and dropouts of the data stream from the quest which resulted in poor motion of the arm
# Re-run this any time the USB cable is unplugged/replugged, since the port forward doesn't survive a disconnect
#
# Usage: ./set_quest_adb_connection.sh [ros_tcp_port]   (defaults to 10000, matching quest_bringup.launch.py's default)

set -e

ROS_TCP_PORT="${1:-10000}"

echo "==> Restarting adb server as root..."
sudo adb kill-server
sudo adb start-server

echo ""
echo "==> Waiting for the Quest to be authorized..."
echo "    If it doesn't show up, put the headset on and accept the"
echo "    'Allow USB debugging?' prompt (only needed once per computer)."
while true; do
    DEVICE_LINE="$(sudo adb devices | grep -v "^List" | grep -v "^$" || true)"
    if echo "$DEVICE_LINE" | grep -qE "[[:space:]]device$"; then
        break
    fi
    echo "    still waiting (current state: ${DEVICE_LINE:-no device detected})..."
    sleep 2
done
echo "==> Quest authorized."

echo ""
echo "==> Forwarding tcp:${ROS_TCP_PORT} over USB..."
sudo adb reverse "tcp:${ROS_TCP_PORT}" "tcp:${ROS_TCP_PORT}"

echo ""
echo "==> Done. On the Quest, set the Quest2ROS app's server IP to 127.0.0.1"
echo "    (port ${ROS_TCP_PORT}) if you haven't already, then launch:"
echo "    ros2 launch vla_kinova_teleop quest_bringup.launch.py"
