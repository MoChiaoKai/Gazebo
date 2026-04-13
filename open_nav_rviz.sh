#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${SCRIPT_DIR}"
WAIT_TIMEOUT="${1:-60}"
PROFILE="${2:-default}"
ALLOW_MULTI_RVIZ="${ALLOW_MULTI_RVIZ:-false}"
RVIZ_SOFTWARE_RENDERING="${RVIZ_SOFTWARE_RENDERING:-false}"

RVIZ_CONFIG_INSTALL_DEFAULT="${WS}/install/yahboom_rosmaster_navigation/share/yahboom_rosmaster_navigation/rviz/nav2_default_view.rviz"
RVIZ_CONFIG_SRC_DEFAULT="${WS}/src/yahboom_rosmaster/yahboom_rosmaster_navigation/rviz/nav2_default_view.rviz"
RVIZ_CONFIG_INSTALL_SLAM="${WS}/install/yahboom_rosmaster_navigation/share/yahboom_rosmaster_navigation/rviz/slam_live_view.rviz"
RVIZ_CONFIG_SRC_SLAM="${WS}/src/yahboom_rosmaster/yahboom_rosmaster_navigation/rviz/slam_live_view.rviz"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "[ERROR] /opt/ros/humble/setup.bash not found"
  exit 1
fi

if [[ -f "${WS}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
  set -u
fi

if [[ "${PROFILE}" == "slam" ]]; then
  RVIZ_CONFIG_INSTALL="${RVIZ_CONFIG_INSTALL_SLAM}"
  RVIZ_CONFIG_SRC="${RVIZ_CONFIG_SRC_SLAM}"
else
  RVIZ_CONFIG_INSTALL="${RVIZ_CONFIG_INSTALL_DEFAULT}"
  RVIZ_CONFIG_SRC="${RVIZ_CONFIG_SRC_DEFAULT}"
fi

if [[ -f "${RVIZ_CONFIG_SRC}" ]]; then
  RVIZ_CONFIG="${RVIZ_CONFIG_SRC}"
elif [[ -f "${RVIZ_CONFIG_INSTALL}" ]]; then
  RVIZ_CONFIG="${RVIZ_CONFIG_INSTALL}"
else
  echo "[ERROR] RViz config not found for profile '${PROFILE}' in src/ or install/"
  echo "[INFO] Available profiles: default, slam"
  exit 1
fi

if [[ "${RVIZ_CONFIG}" == "${RVIZ_CONFIG_SRC}" ]]; then
  echo "[INFO] Using RViz config from src: ${RVIZ_CONFIG}"
else
  echo "[INFO] Using RViz config from install: ${RVIZ_CONFIG}"
fi

echo "[INFO] Waiting up to ${WAIT_TIMEOUT}s for /tf and /map..."
READY="false"
for ((i=1; i<=WAIT_TIMEOUT; i++)); do
  TOPICS="$(ros2 topic list 2>/dev/null || true)"
  if grep -q '^/tf$' <<<"${TOPICS}" && grep -q '^/map$' <<<"${TOPICS}"; then
    READY="true"
    break
  fi
  sleep 1
done

if [[ "${READY}" != "true" ]]; then
  echo "[ERROR] /tf or /map not ready after ${WAIT_TIMEOUT}s."
  echo "[INFO] Start SLAM/NAV stack first, then run this script again."
  exit 1
else
  echo "[INFO] Topics ready. Opening RViz."
fi

if [[ "${ALLOW_MULTI_RVIZ}" != "true" ]]; then
  NODES="$(ros2 node list 2>/dev/null || true)"
  if grep -Eq '^/rviz$|^/rviz2$' <<<"${NODES}"; then
    echo "[WARN] RViz is already running. Skip launching a second RViz to avoid lag."
    echo "[INFO] Set ALLOW_MULTI_RVIZ=true if you intentionally want multiple RViz instances."
    exit 0
  fi
fi

if [[ "${PROFILE}" == "default" ]]; then
  AMCL_WAIT_SEC="${AMCL_WAIT_SEC:-30}"
  echo "[INFO] Waiting up to ${AMCL_WAIT_SEC}s for /amcl_pose (map localization)..."
  if timeout "${AMCL_WAIT_SEC}" ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
    echo "[INFO] AMCL pose is available."
  else
    echo "[WARN] /amcl_pose is not ready yet, so map->odom may still be missing."
    echo "[WARN] If Gazebo and RViz look misaligned, use 2D Pose Estimate once in RViz."
  fi
fi

if [[ "${RVIZ_SOFTWARE_RENDERING}" == "true" ]]; then
  exec env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 LIBGL_ALWAYS_SOFTWARE=1 QT_QUICK_BACKEND=software rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true
fi

exec env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true
