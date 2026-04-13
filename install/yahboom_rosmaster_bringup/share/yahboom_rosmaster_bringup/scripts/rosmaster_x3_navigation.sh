#!/usr/bin/env bash
set -euo pipefail

# Single script to launch the Yahboom ROSMASTER X3 with Gazebo + Nav2.
# Usage:
#   bash rosmaster_x3_navigation.sh [slam|nav|auto] [headless:true|false] [world_file] [map_yaml] [spawn_x] [spawn_y] [spawn_yaw]
# Example:
#   bash rosmaster_x3_navigation.sh slam false temp.world

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
MODE="${1:-nav}"
HEADLESS="${2:-false}"
WORLD_FILE="${3:-temp.world}"
DEFAULT_MAP_PATH="${WS}/src/yahboom_rosmaster/yahboom_rosmaster_navigation/maps/temp_world_map.yaml"
MAP_PATH="${4:-${DEFAULT_MAP_PATH}}"
SPAWN_X="${5:-0.0}"
SPAWN_Y="${6:-0.0}"
SPAWN_YAW="${7:-0.0}"
CLEAN_STALE="${CLEAN_STALE:-true}"
AUTO_INITIALPOSE="${AUTO_INITIALPOSE:-true}"
AUTO_EXPLORE="false"
SIM_PROC_PATTERN="ign gazebo|gz sim|ignition-gazebo|gzserver|gzclient"
STACK_PROC_PATTERN="ros_gz_bridge|parameter_bridge|controller_manager|spawner|component_container_isolated|robot_state_publisher|joint_state_publisher|assisted_teleoperation.py|nav_to_pose.py|go_to_point.py|go_to_goal_pose|basic_navigator|slam_toolbox|lifecycle_manager|rviz2|rosmaster_x3_navigation.launch.py|frontier_explorer.py"

terminate_matching_processes() {
    local pattern="$1"
    pkill -f "${pattern}" 2>/dev/null || true
}

wait_for_no_processes() {
    local pattern="$1"
    local timeout_sec="${2:-12}"
    local i
    for i in $(seq 1 "${timeout_sec}"); do
        if ! pgrep -f "${pattern}" >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    return 1
}

publish_initial_pose_from_spawn() {
    local spawn_x="$1"
    local spawn_y="$2"
    local spawn_yaw="$3"

    local qz qw
    read -r qz qw < <(python3 - "$spawn_yaw" <<'PY'
import math
import sys
yaw = float(sys.argv[1])
print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))
PY
)

    # Wait for AMCL to subscribe to /initialpose before publishing.
    local ready=0
    for _ in $(seq 1 30); do
        if ros2 topic info /initialpose 2>/dev/null | grep -Eq 'Subscription count:\s*[1-9]'; then
            ready=1
            break
        fi
        sleep 1
    done

    if [[ "$ready" -ne 1 ]]; then
        echo "[WARN] /initialpose has no subscribers yet. Please use 2D Pose Estimate in RViz."
        return 1
    fi

    # Ensure wheel odometry is flowing before setting initial pose.
    if ! timeout 8 ros2 topic echo /mecanum_drive_controller/odom --once >/dev/null 2>&1; then
        echo "[WARN] /mecanum_drive_controller/odom not ready yet. Delaying automatic /initialpose."
        return 1
    fi

    # Use non-zero covariance so AMCL can still converge when map and spawn are not perfectly aligned.
    if python3 - "$spawn_x" "$spawn_y" "$qz" "$qw" <<'PY'
import sys
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

x = float(sys.argv[1])
y = float(sys.argv[2])
qz = float(sys.argv[3])
qw = float(sys.argv[4])

rclpy.init()
node = rclpy.create_node('auto_initialpose_publisher')
qos = QoSProfile(depth=1)
qos.reliability = ReliabilityPolicy.BEST_EFFORT
qos.durability = DurabilityPolicy.VOLATILE
pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)

msg = PoseWithCovarianceStamped()
msg.header.frame_id = 'map'
msg.pose.pose.position.x = x
msg.pose.pose.position.y = y
msg.pose.pose.orientation.z = qz
msg.pose.pose.orientation.w = qw
msg.pose.covariance[0] = 0.25
msg.pose.covariance[7] = 0.25
msg.pose.covariance[35] = 0.0685389191

for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() > 0:
        break

for _ in range(5):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
PY
    then
        echo "[INFO] Published /initialpose from spawn (${spawn_x}, ${spawn_y}, yaw=${spawn_yaw})."
        return 0
    fi

    echo "[WARN] Failed to publish /initialpose automatically. Please use 2D Pose Estimate in RViz."
    return 1
}

if [[ "${MODE}" == "-h" || "${MODE}" == "--help" ]]; then
    echo "Usage: bash rosmaster_x3_navigation.sh [slam|nav|auto] [headless:true|false] [world_file] [map_yaml] [spawn_x] [spawn_y] [spawn_yaw]"
    echo "Examples:"
    echo "  bash rosmaster_x3_navigation.sh slam false temp.world"
    echo "  bash rosmaster_x3_navigation.sh nav true temp.world"
    echo "  bash rosmaster_x3_navigation.sh auto false temp.world"
    echo "  bash rosmaster_x3_navigation.sh nav false temp.world ${WS}/src/yahboom_rosmaster/yahboom_rosmaster_navigation/maps/temp_world_map.yaml 0.0 0.0 0.0"
    echo "  bash rosmaster_x3_navigation.sh nav true temp.world ${WS}/src/yahboom_rosmaster/yahboom_rosmaster_navigation/maps/temp_world_map.yaml"
    echo "Optional env: CLEAN_STALE=true|false (default: true), AUTO_INITIALPOSE=true|false (default: true)"
    exit 0
fi

HEADLESS_LC="$(echo "${HEADLESS}" | tr '[:upper:]' '[:lower:]')"

if [[ "${MODE}" == "slam" ]]; then
    SLAM_VALUE="True"
elif [[ "${MODE}" == "nav" ]]; then
    SLAM_VALUE="False"
elif [[ "${MODE}" == "auto" ]]; then
    SLAM_VALUE="True"
    AUTO_EXPLORE="true"
else
    echo "[ERROR] First argument must be 'slam', 'nav', or 'auto'."
    echo "Usage: bash rosmaster_x3_navigation.sh [slam|nav|auto] [headless:true|false] [world_file] [map_yaml]"
    exit 1
fi

if [[ "${HEADLESS_LC}" != "true" && "${HEADLESS_LC}" != "false" ]]; then
    echo "[ERROR] Second argument must be true or false (headless)."
    exit 1
fi

if [[ "${MAP_PATH}" != /* ]]; then
    MAP_PATH="${WS}/${MAP_PATH}"
fi

if [[ "${WORLD_FILE}" == */* ]]; then
    echo "[ERROR] world_file must be a filename under ${WS}/src/yahboom_rosmaster/yahboom_rosmaster_gazebo/worlds"
    exit 1
fi

WORLD_PATH="${WS}/src/yahboom_rosmaster/yahboom_rosmaster_gazebo/worlds/${WORLD_FILE}"

if [[ ! -f "${WORLD_PATH}" ]]; then
    echo "[ERROR] World file not found: ${WORLD_PATH}"
    echo "[INFO] Available worlds:"
    ls -1 "${WS}/src/yahboom_rosmaster/yahboom_rosmaster_gazebo/worlds" || true
    exit 1
fi

SPAWN_Z="0.20"
if [[ "${WORLD_FILE}" == "house.world" || "${WORLD_FILE}" == "temp.world" ]]; then
    SPAWN_Z="0.05"
fi

if [[ "${MODE}" == "nav" && ! -f "${MAP_PATH}" ]]; then
    echo "[ERROR] Map file not found for nav mode: ${MAP_PATH}"
    echo "[INFO] Please run SLAM first and save the map, for example:"
    echo "  ros2 run nav2_map_server map_saver_cli -f ${MAP_PATH%.yaml}"
    exit 1
fi

if [[ "${MODE}" == "slam" ]]; then
    mkdir -p "$(dirname "${MAP_PATH}")"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[ERROR] ROS 2 Humble setup not found at /opt/ros/humble/setup.bash"
    exit 1
fi

if [[ ! -f "${WS}/install/setup.bash" ]]; then
    echo "[ERROR] Workspace overlay missing: ${WS}/install/setup.bash"
    echo "[INFO] Build first: cd ${WS} && colcon build"
    exit 1
fi

# ROS setup files may reference unset vars internally; temporarily relax nounset.
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

# Work around FastDDS shared-memory lock issues (RTPS_TRANSPORT_SHM Error).
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
echo "[INFO] FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}"

if [[ "${CLEAN_STALE}" == "true" ]]; then
    echo "[INFO] Cleaning stale Gazebo/Nav2 processes..."
    terminate_matching_processes "${STACK_PROC_PATTERN}"
    terminate_matching_processes "${SIM_PROC_PATTERN}"

    if ! wait_for_no_processes "${SIM_PROC_PATTERN}" 10; then
        echo "[WARN] Some Gazebo processes are still alive, sending SIGKILL..."
        pkill -9 -f "${SIM_PROC_PATTERN}" 2>/dev/null || true
        sleep 1
    fi

    if ! wait_for_no_processes "${SIM_PROC_PATTERN}" 8; then
        echo "[ERROR] Stale Gazebo process still detected. Please kill it manually before retrying."
        pgrep -af "${SIM_PROC_PATTERN}" || true
        exit 1
    fi

    sleep 1
fi

USE_RVIZ="true"
if [[ "${HEADLESS_LC}" == "true" ]]; then
    # In headless mode, skip RViz autostart to reduce crashes on low-resource setups.
    USE_RVIZ="false"
fi

cleanup() {
    echo "Cleaning up..."
    if [[ -n "${AUTO_PID:-}" ]] && kill -0 "${AUTO_PID}" 2>/dev/null; then
        kill "${AUTO_PID}" 2>/dev/null || true
        wait "${AUTO_PID}" 2>/dev/null || true
    fi
    if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        kill "${LAUNCH_PID}" 2>/dev/null || true
        wait "${LAUNCH_PID}" 2>/dev/null || true
    fi
}

trap cleanup SIGINT SIGTERM EXIT

echo "[INFO] Workspace: ${WS}"
echo "[INFO] Mode: ${MODE} (slam:=${SLAM_VALUE})"
echo "[INFO] Auto frontier exploration: ${AUTO_EXPLORE}"
echo "[INFO] Auto initial pose publish: ${AUTO_INITIALPOSE}"
echo "[INFO] Headless: ${HEADLESS_LC}, use_rviz: ${USE_RVIZ}, world: ${WORLD_PATH}"
echo "[INFO] Spawn XY yaw: ${SPAWN_X}, ${SPAWN_Y}, ${SPAWN_YAW}"
echo "[INFO] Spawn Z: ${SPAWN_Z}"
echo "[INFO] Map YAML: ${MAP_PATH}"
echo "[INFO] Launching Gazebo simulation with Nav2..."

ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py \
    enable_odom_tf:=false \
    enable_rgbd:=false \
    headless:="${HEADLESS_LC}" \
    load_controllers:=true \
    world_file:="${WORLD_FILE}" \
    use_rviz:="${USE_RVIZ}" \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:="${SPAWN_X}" \
    y:="${SPAWN_Y}" \
    z:="${SPAWN_Z}" \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:="${SPAWN_YAW}" \
    slam:="${SLAM_VALUE}" \
    map:="${MAP_PATH}" &

LAUNCH_PID=$!

echo "[INFO] Waiting 25 seconds for simulation to initialize..."
sleep 25

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[ERROR] Launch process exited early. Check terminal logs above."
    exit 1
fi

CM_READY=0
for _ in $(seq 1 30); do
    if ros2 service list 2>/dev/null | grep -q '^/controller_manager/list_controllers$'; then
        CM_READY=1
        break
    fi
    sleep 1
done

if [[ "${CM_READY}" -ne 1 ]]; then
    echo "[ERROR] /controller_manager/list_controllers is still unavailable."
    echo "[INFO] This usually means robot was spawned into a stale Gazebo instance or ros2_control did not load."
    echo "[INFO] Running Gazebo processes:"
    pgrep -af "${SIM_PROC_PATTERN}" || true
    echo "[INFO] Controller manager services:"
    ros2 service list 2>/dev/null | grep controller_manager || true
    exit 1
fi

if [[ "${MODE}" == "nav" && "${AUTO_INITIALPOSE}" == "true" ]]; then
    publish_initial_pose_from_spawn "${SPAWN_X}" "${SPAWN_Y}" "${SPAWN_YAW}" || true
fi

if [[ "${HEADLESS_LC}" == "false" ]]; then
    if command -v gz >/dev/null 2>&1; then
        echo "[INFO] Adjusting Gazebo GUI camera position..."
        gz service -s /gui/move_to/pose \
            --reqtype gz.msgs.GUICamera \
            --reptype gz.msgs.Boolean \
            --timeout 2000 \
            --req "pose: {position: {x: 0.0, y: -2.0, z: 2.0} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}" || true
    else
        echo "[WARN] 'gz' command not found, skipping camera adjustment."
    fi
fi

if [[ "${AUTO_EXPLORE}" == "true" ]]; then
    echo "[INFO] Starting frontier auto-exploration node..."
    ros2 run yahboom_rosmaster_navigation frontier_explorer.py --ros-args \
        -p map_topic:=/map \
                    -p base_frame:=base_footprint \
        -p plan_period:=3.0 \
        -p sample_step:=3 \
        -p frontier_standoff:=1.0 \
        -p boundary_margin_m:=0.45 \
        -p goal_clearance_cells:=3 \
        -p occupied_threshold:=50 \
        -p min_goal_distance:=0.5 \
        -p max_goal_distance:=2.5 \
        -p blacklist_radius:=1.2 \
        -p visited_radius:=1.2 \
        -p cluster_bin_size:=0.6 \
        -p cluster_weight:=0.8 \
        -p distance_penalty:=1.6 \
        -p direction_weight:=0.8 \
        -p max_jump_from_last:=1.0 \
        -p goal_timeout:=120.0 &
    AUTO_PID=$!
    sleep 1
    if ! kill -0 "${AUTO_PID}" 2>/dev/null; then
        echo "[ERROR] frontier_explorer failed to start."
        exit 1
    fi
fi

if [[ "${MODE}" == "slam" ]]; then
    MAP_PREFIX="${MAP_PATH}"
    if [[ "${MAP_PREFIX}" == *.yaml ]]; then
        MAP_PREFIX="${MAP_PREFIX%.yaml}"
    fi
    echo "[INFO] SLAM mode is running. After mapping, save with:"
    echo "  source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ${MAP_PREFIX}"
elif [[ "${MODE}" == "auto" ]]; then
    MAP_PREFIX="${MAP_PATH}"
    if [[ "${MAP_PREFIX}" == *.yaml ]]; then
        MAP_PREFIX="${MAP_PREFIX%.yaml}"
    fi
    echo "[INFO] AUTO mode is running (SLAM + frontier exploration)."
    echo "[INFO] After exploration converges, save with:"
    echo "  source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ${MAP_PREFIX}"
fi

echo "[INFO] Launch is running. Press Ctrl+C to stop."
wait "${LAUNCH_PID}"
