#!/usr/bin/env bash
set -eo pipefail

WS="/home/mo/Gazebo"
HEADLESS="${1:-False}"
TELEOP_SPEED="${2:-0.18}"
TELEOP_TURN="${3:-1.00}"
ROBOT_ID="${4:-1}"
ENABLE_RGBD="${5:-false}"

if [[ "$ROBOT_ID" != "1" && "$ROBOT_ID" != "2" && "$ROBOT_ID" != "3" ]]; then
  echo "Usage: $0 [headless:true|false] [speed] [turn] [robot_id: 1|2|3] [enable_rgbd:true|false]"
  exit 1
fi

HEADLESS_LC="$(echo "$HEADLESS" | tr '[:upper:]' '[:lower:]')"
ENABLE_RGBD_LC="$(echo "$ENABLE_RGBD" | tr '[:upper:]' '[:lower:]')"

if [[ "$HEADLESS_LC" != "true" && "$HEADLESS_LC" != "false" ]]; then
  echo "[ERROR] headless must be true or false"
  exit 1
fi

if [[ "$ENABLE_RGBD_LC" != "true" && "$ENABLE_RGBD_LC" != "false" ]]; then
  echo "[ERROR] enable_rgbd must be true or false"
  exit 1
fi

CMD_TOPIC="/rosmaster_x3_${ROBOT_ID}/cmd_vel"
CM="/rosmaster_x3_${ROBOT_ID}/controller_manager"
STALE_PROC_PATTERN="ros2|ign gazebo|gz sim|controller_manager|spawner|ros_gz|cmd_vel_relay|robot_state_publisher|joint_state_publisher|rviz2"

terminate_stale_processes() {
  local pattern="$1"
  # Try graceful termination first to reduce middleware stale-state artifacts.
  pkill -f "$pattern" 2>/dev/null || true
  sleep 1
  pkill -9 -f "$pattern" 2>/dev/null || true
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
cd "$WS"

if [[ ! -f install/setup.bash ]]; then
  echo "[ERROR] install/setup.bash not found. Please build workspace first."
  exit 1
fi

source install/setup.bash

# Work around FastDDS shared-memory lock issues (RTPS_TRANSPORT_SHM Error).
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
echo "[INFO] FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}"

# Clear stale ROS/Gazebo processes to avoid topic/service conflicts and TF collisions.
terminate_stale_processes "$STALE_PROC_PATTERN"

echo "[INFO] Launching Gazebo (headless: ${HEADLESS_LC}, enable_rgbd: ${ENABLE_RGBD_LC})..."
ros2 launch yahboom_rosmaster_gazebo yahboom_rosmaster_three.gazebo.launch.py \
  "headless:=${HEADLESS_LC}" \
  "enable_rgbd:=${ENABLE_RGBD_LC}" &
LAUNCH_PID=$!

echo "[INFO] Waiting for all robot controllers to become active..."
READY_SELECTED=0
for ID in 1 2 3; do
  ROBOT_CM="/rosmaster_x3_${ID}/controller_manager"
  ROBOT_READY=0
  for _ in $(seq 1 80); do
    if ros2 control list_controllers -c "$ROBOT_CM" 2>/dev/null | grep -q "mecanum_drive_controller.*active"; then
      ROBOT_READY=1
      [[ "$ID" == "$ROBOT_ID" ]] && READY_SELECTED=1
      break
    fi
    sleep 1
  done

  if [[ "$ROBOT_READY" -eq 1 ]]; then
    echo "[INFO] robot ${ID}: mecanum_drive_controller active"
  else
    echo "[WARN] robot ${ID}: controller not active yet"
  fi
  ros2 control list_controllers -c "$ROBOT_CM" || true
done

if [[ "$READY_SELECTED" -ne 1 ]]; then
  echo "[WARN] selected robot ${ROBOT_ID} controller not active yet. Launch will keep running."
fi

echo "[INFO] Teleop autostart disabled."
echo "[INFO] Start teleop manually when needed: bash teleop_robot.sh ${ROBOT_ID} ${TELEOP_SPEED} ${TELEOP_TURN}"
echo "[INFO] Gazebo is running. Press Ctrl+C to stop."

wait "$LAUNCH_PID"
