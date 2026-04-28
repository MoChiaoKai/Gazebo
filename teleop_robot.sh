#!/usr/bin/env bash
set -eo pipefail

ROBOT_ID="${1:-1}"
SPEED="${2:-0.20}"
TURN="${3:-1.00}"
REPEAT_RATE="${4:-30.0}"
KEY_TIMEOUT="${5:-0.30}"

if [[ "$ROBOT_ID" != "1" && "$ROBOT_ID" != "2" && "$ROBOT_ID" != "3" ]]; then
  echo "Usage: $0 <robot_id: 1|2|3> [speed] [turn] [repeat_rate] [key_timeout]"
  exit 1
fi

WS="/home/mo/Gazebo"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

CMD_TOPIC="/rosmaster_x3_${ROBOT_ID}/cmd_vel"

echo "[INFO] Teleop robot ${ROBOT_ID}"
echo "[INFO] topic=${CMD_TOPIC}"
echo "[INFO] speed=${SPEED}, turn=${TURN}, repeat_rate=${REPEAT_RATE}, key_timeout=${KEY_TIMEOUT}"

# Prevent duplicate keyboard teleop publishers for this robot.
pkill -f "teleop_twist_keyboard.*${CMD_TOPIC}" 2>/dev/null || true
sleep 0.1

PUB_COUNT=$(ros2 topic info "$CMD_TOPIC" 2>/dev/null | awk '/Publisher count/ {print $3}')
if [[ -n "${PUB_COUNT:-}" && "$PUB_COUNT" != "0" ]]; then
  echo "[WARN] ${CMD_TOPIC} already has ${PUB_COUNT} publisher(s). If control feels odd, close extra teleop terminals."
fi

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:="${SPEED}" \
  -p turn:="${TURN}" \
  -p repeat_rate:="${REPEAT_RATE}" \
  -p key_timeout:="${KEY_TIMEOUT}" \
  -r cmd_vel:="${CMD_TOPIC}"
