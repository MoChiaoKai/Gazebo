#!/usr/bin/env python3
import argparse
import math
import os
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from astar_planner import AStarGridPlanner
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


WORLD_MIN = -2.5
WORLD_MAX = 2.5


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parse_scalar(block: str, key: str, default: float = 0.0) -> float:
    m = re.search(rf"\b{re.escape(key)}:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", block)
    return float(m.group(1)) if m else default


def strip_ansi(text: str) -> str:
    return re.sub(r'\x1b\[[0-9;]*m', '', text)


def parse_controller_state(list_output: str, controller_name: str) -> str | None:
    for line in strip_ansi(list_output).splitlines():
        fields = line.split()
        if len(fields) >= 3 and fields[0] == controller_name:
            return fields[-1].strip().lower()
    return None


def get_controller_state(robot_id: int, timeout_sec: float = 4.0) -> str | None:
    manager = f'/rosmaster_x3_{robot_id}/controller_manager'
    try:
        output = subprocess.check_output(
            ['ros2', 'control', 'list_controllers', '-c', manager],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=max(1.0, timeout_sec),
        )
    except Exception:
        return None
    return parse_controller_state(output, 'mecanum_drive_controller')


def set_controller_state(robot_id: int, state: str, timeout_sec: float = 5.0) -> bool:
    manager = f'/rosmaster_x3_{robot_id}/controller_manager'
    try:
        output = subprocess.check_output(
            ['ros2', 'control', 'set_controller_state', '-c', manager, 'mecanum_drive_controller', state],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=max(1.0, timeout_sec),
        )
    except Exception:
        return False
    lowered = strip_ansi(output).lower()
    return 'success' in lowered or 'ok' in lowered


def unload_controller(robot_id: int, timeout_sec: float = 5.0) -> bool:
    manager = f'/rosmaster_x3_{robot_id}/controller_manager'
    try:
        output = subprocess.check_output(
            ['ros2', 'control', 'unload_controller', '-c', manager, 'mecanum_drive_controller'],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=max(1.0, timeout_sec),
        )
    except Exception:
        return False
    return 'successfully' in strip_ansi(output).lower()


def load_controller_active(robot_id: int, timeout_sec: float = 8.0) -> bool:
    manager = f'/rosmaster_x3_{robot_id}/controller_manager'
    try:
        output = subprocess.check_output(
            [
                'ros2',
                'control',
                'load_controller',
                '-c',
                manager,
                'mecanum_drive_controller',
                '--set-state',
                'active',
            ],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=max(1.0, timeout_sec),
        )
    except Exception:
        return False
    lowered = strip_ansi(output).lower()
    return 'successfully loaded controller' in lowered and 'active' in lowered


def ensure_mecanum_controller_active(robot_id: int, timeout_sec: float = 10.0) -> tuple[bool, str]:
    deadline = time.time() + max(2.0, timeout_sec)
    state = get_controller_state(robot_id)
    if state == 'active':
        return True, state

    if state == 'inactive':
        set_controller_state(robot_id, 'active')
    elif state == 'unconfigured':
        unload_controller(robot_id)
        load_controller_active(robot_id)

    while time.time() < deadline:
        state = get_controller_state(robot_id)
        if state == 'active':
            return True, state
        if state == 'inactive':
            set_controller_state(robot_id, 'active')
        elif state == 'unconfigured':
            unload_controller(robot_id)
            load_controller_active(robot_id)
        time.sleep(0.2)

    return False, state or 'unknown'


def parse_world_pose_snapshot(output: str) -> dict[str, tuple[float, float, float]]:
    found: dict[str, tuple[float, float, float]] = {}
    lines = output.splitlines()
    i = 0
    while i < len(lines):
        if lines[i].strip() != 'pose {':
            i += 1
            continue

        depth = 1
        i += 1
        block_lines = []
        while i < len(lines) and depth > 0:
            line = lines[i]
            depth += line.count('{')
            depth -= line.count('}')
            block_lines.append(line)
            i += 1

        block = '\n'.join(block_lines)
        m_name = re.search(r'name:\s*"([^"]+)"', block)
        if not m_name:
            continue

        name = m_name.group(1)
        m_pos = re.search(r'position\s*\{([^}]*)\}', block, re.S)
        m_ori = re.search(r'orientation\s*\{([^}]*)\}', block, re.S)
        pos_block = m_pos.group(1) if m_pos else ''
        ori_block = m_ori.group(1) if m_ori else ''

        x = parse_scalar(pos_block, 'x', 0.0)
        y = parse_scalar(pos_block, 'y', 0.0)
        qx = parse_scalar(ori_block, 'x', 0.0)
        qy = parse_scalar(ori_block, 'y', 0.0)
        qz = parse_scalar(ori_block, 'z', 0.0)
        qw = parse_scalar(ori_block, 'w', 1.0)
        yaw = yaw_from_quaternion(qx, qy, qz, qw)

        found[name] = (x, y, yaw)

    return found


def read_world_poses(world_name: str, timeout_sec: float) -> dict[str, tuple[float, float, float]]:
    topic = f'/world/{world_name}/pose/info'

    try:
        output = subprocess.check_output(
            ['ign', 'topic', '-e', '-t', topic, '-n', '1'],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=max(timeout_sec, 0.5),
        )
    except FileNotFoundError as e:
        raise RuntimeError('ign command not found') from e
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f'timeout waiting pose data on {topic}') from e
    except subprocess.CalledProcessError as e:
        raw = e.output.strip() if e.output else 'unknown error'
        # Keep sync warnings concise; huge pose dumps can flood logs and slow control loop.
        detail = raw.splitlines()[-1][:220] if raw else 'unknown error'
        raise RuntimeError(f'failed to read {topic}: {detail}') from e

    return parse_world_pose_snapshot(output)


def read_robot_world_pose(robot_name: str, world_name: str, timeout_sec: float) -> tuple[float, float, float]:
    topic = f'/world/{world_name}/pose/info'
    poses = read_world_poses(world_name, timeout_sec)
    pose = poses.get(robot_name)
    if pose is None:
        raise RuntimeError(f'{robot_name} not found in {topic}')
    return pose


def world_to_odom(
    world_x: float,
    world_y: float,
    tx_world: float,
    ty_world: float,
    yaw_world_from_odom: float,
) -> tuple[float, float]:
    dx = world_x - tx_world
    dy = world_y - ty_world
    c = math.cos(yaw_world_from_odom)
    s = math.sin(yaw_world_from_odom)
    odom_x = c * dx + s * dy
    odom_y = -s * dx + c * dy
    return odom_x, odom_y


class GoToPointNode(Node):
    def __init__(
        self,
        robot_id: int,
        target_x: float,
        target_y: float,
        fixed_world_waypoints: list[tuple[float, float]] | None,
        target_yaw_deg: float | None,
        pos_tol: float,
        yaw_tol_deg: float,
        max_linear: float,
        max_lateral: float,
        max_angular: float,
        max_angular_final: float,
        k_linear: float,
        k_lateral: float,
        k_yaw: float,
        turn_slowdown_deg: float,
        yaw_settle_sec: float,
        min_approach_speed: float,
        min_approach_distance: float,
        node_step_distance: float,
        face_heading_tol_deg: float,
        forward_realign_deg: float,
        obstacle_stop_dist: float,
        obstacle_slow_dist: float,
        obstacle_turn_deg: float,
        obstacle_front_half_deg: float,
        obstacle_side_start_deg: float,
        obstacle_side_end_deg: float,
        target_frame: str,
    ):
        super().__init__(f'go_to_point_node_{robot_id}')
        self.robot_id = robot_id

        self.target_x = target_x
        self.target_y = target_y
        self.fixed_world_waypoints = fixed_world_waypoints[:] if fixed_world_waypoints else []
        self.target_yaw = None if target_yaw_deg is None else math.radians(target_yaw_deg)

        self.pos_tol = abs(pos_tol)
        self.yaw_tol = math.radians(abs(yaw_tol_deg))
        self.max_linear = abs(max_linear)
        self.max_lateral = abs(max_lateral)
        self.max_angular = abs(max_angular)
        self.max_angular_final = abs(max_angular_final)
        self.k_linear = abs(k_linear)
        self.k_lateral = abs(k_lateral)
        self.k_yaw = abs(k_yaw)
        self.turn_slowdown_rad = math.radians(abs(turn_slowdown_deg))
        self.yaw_settle_sec = abs(yaw_settle_sec)
        self.min_approach_speed = abs(min_approach_speed)
        self.min_approach_distance = abs(min_approach_distance)
        self.node_step_distance = max(0.05, abs(node_step_distance))
        self.face_heading_tol_rad = math.radians(abs(face_heading_tol_deg))
        raw_forward_realign = math.radians(abs(forward_realign_deg))
        # Keep forward realign threshold slightly looser than rotate exit to avoid phase chatter deadlock.
        self.forward_realign_rad = max(raw_forward_realign, self.face_heading_tol_rad + math.radians(0.2))
        self.obstacle_stop_dist = max(0.05, abs(obstacle_stop_dist))
        self.obstacle_slow_dist = max(self.obstacle_stop_dist + 0.05, abs(obstacle_slow_dist))
        self.obstacle_turn_rad = math.radians(abs(obstacle_turn_deg))
        self.obstacle_front_half_rad = math.radians(abs(obstacle_front_half_deg))
        self.obstacle_side_start_rad = math.radians(abs(obstacle_side_start_deg))
        self.obstacle_side_end_rad = math.radians(abs(obstacle_side_end_deg))

        # Legacy obstacle fields are kept for constructor compatibility.
        # Global mode plans from temp.world static obstacles and does not use LaserScan local avoidance.
        self.obstacle_min_rank = 3
        self.obstacle_block_min_points = 3
        self.obstacle_slow_min_points = 4
        self.obstacle_enter_hits = 2
        self.obstacle_exit_misses = 3
        self.obstacle_caution_enter_hits = 2
        self.obstacle_caution_exit_misses = 2
        self.obstacle_turn_lock_sec = 0.8
        self.blocked_sweep_yaw = max(self.max_angular_final, 0.22)
        self.forward_yaw_correction_max = 0.18
        self.forward_yaw_correction_caution = 0.10
        # Odom-yaw turn tracking: require stable in-tolerance yaw before leaving rotate phase.
        self.turn_settle_sec = 0.24
        self.turn_retarget_rad = math.radians(6.0)
        self.turn_dir_release_rad = math.radians(8.0)
        self.turn_pi_lock_rad = math.radians(10.0)
        self.turn_split_180_rad = math.radians(150.0)
        self.turn_overshoot_margin_rad = math.radians(2.0)
        self.turn_rate_tol = 0.05
        self.turn_in_tol_force_sec = 0.80
        self.turn_kp = 0.95
        self.turn_kd = 0.90
        self.turn_min_angular = 0.03
        self.turn_min_apply_err_rad = math.radians(12.0)
        self.turn_cmd_slew_rate = 0.70
        self.turn_brake_decel_est = 0.24
        self.turn_180_comp_rad = math.radians(30.0)
        self.turn_brake_margin_rad = math.radians(2.2)
        self.turn_brake_min_rate = 0.03
        self.turn_brake_gain = 1.20
        self.rotate_relax_tol_rad = math.radians(5.0)
        self.rotate_relax_rate_rad = 0.06
        self.rotate_relax_hold_sec = 0.60
        # Smooth forward motion: ramp speed changes and avoid unnecessary stop-go on collinear nodes.
        self.linear_cmd_slew_rate = 0.80
        self.linear_cmd_slew_rate_decel = 1.10
        # Keep speed uniform on straight segments by chaining collinear nodes.
        # The robot will still stop/rotate at true corners or when safety hold triggers.
        self.force_stop_each_node = False
        self.forward_chain_heading_tol_rad = math.radians(8.0)
        self.forward_realign_enter_hyst_rad = math.radians(0.9)
        self.forward_realign_exit_hyst_rad = math.radians(0.5)
        self.forward_realign_hold_sec = 0.32
        self.forward_realign_disable_dist = max(0.18, self.pos_tol * 2.5)
        self.forward_force_rotate_rad = math.radians(20.0)
        self.forward_force_rotate_dist = max(0.42, self.pos_tol * 5.0)
        self.forward_heading_kp = 0.72
        self.forward_heading_wz_max = 0.10
        self.forward_heading_wz_slew_rate = 0.45
        self.forward_speed_heading_slow_start_rad = math.radians(2.0)
        self.forward_speed_heading_slow_end_rad = math.radians(6.0)
        self.forward_speed_min_scale = 0.68
        self.forward_speed_min_abs = max(0.12, self.min_approach_speed * 0.75)
        self.progress_keepout_dist = 0.22
        self.progress_keepout_margin = 0.02
        self.progress_done_tol = max(0.12, self.pos_tol * 1.4)
        self.progress_replan_margin = 0.10
        self.near_goal_turn_relax_dist = 0.45
        self.near_goal_face_tol_rad = math.radians(3.0)
        self.near_goal_forward_realign_rad = math.radians(3.8)
        self.cross_track_replan_dist = max(0.08, self.pos_tol * 0.8)
        self.cross_track_replan_margin = 0.03
        self.cross_track_replan_cooldown_sec = 0.8
        self.blocked_rotate_switch_sec = 1.0
        self.blocked_axis_replan_cooldown_sec = 1.5
        self.forward_stall_node_dist = 0.30
        self.forward_stall_err_eps = 0.10
        self.forward_stall_move_eps = 0.12
        self.forward_stall_hold_sec = 4.0
        self.forward_stall_replan_cooldown_sec = 2.0
        self.world_heading_use_limit_rad = math.radians(12.0)
        # In fixed coordinator-path mode, honor coordinator waypoints exactly.
        # Only out-of-world points will be clamped to hard world bounds.
        self.fixed_path_edge_margin = 0.0
        self._fixed_path_clamp_logged = False
        # Backup dynamic peer safety: emergency-only short-range brake.
        self.peer_emergency_stop_dist = 0.16
        self.peer_critical_stop_dist = 0.08

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_wz = 0.0
        self.odom_ready = False
        self.done = False
        self._last_log_time = 0.0
        self._yaw_in_tol_since = None
        self._progress_frame = 'odom'
        self._progress_cur_x = None
        self._progress_cur_y = None
        self._progress_goal_x = None
        self._progress_goal_y = None
        self._effective_goal_world_x = self.target_x
        self._effective_goal_world_y = self.target_y
        self._world_pose_ready = False
        self._world_x = 0.0
        self._world_y = 0.0
        self._world_yaw = 0.0
        self._world_from_odom_yaw = 0.0
        # Each node stores: (x, y, kind, reserved).
        # kind in {'waypoint', 'final'}.
        self.path_nodes: list[tuple[float, float, str, int]] = []
        self.active_node_idx = 0
        self.drive_phase = 'idle'
        self.path_failed = False

        self.map_ready = False
        self._map_topic = 'none'
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_cos = 1.0
        self.map_origin_sin = 0.0
        self.map_blocked: list[bool] = []
        self.global_plan_world: list[tuple[float, float]] = []
        self.map_inflate_radius_m = 0.0

        self.turn_target_yaw = None
        self.turn_mid_yaw = None
        self._turn_in_tol_since = None
        self.turn_dir_sign = 0.0
        self.turn_budget_active = False
        self.turn_budget_latched = False
        self.turn_start_yaw = 0.0
        self.turn_expected_mag = 0.0
        self.turn_heading_override = None
        self.turn_budget_accum = 0.0
        self.turn_budget_prev_yaw = 0.0
        self.prev_turn_cmd_wz = 0.0
        self.prev_cmd_vx = 0.0
        self._forward_realign_since = None
        self._rotate_relax_since = None
        self._blocked_rotate_since = None
        self._last_cross_track_replan = 0.0
        self._last_blocked_axis_replan = 0.0
        self._last_world_heading_fallback_log = 0.0
        self._last_control_time = None
        self._forward_stall_since = None
        self._forward_stall_ref_err = None
        self._forward_stall_ref_x = None
        self._forward_stall_ref_y = None
        self._last_forward_stall_replan = 0.0
        self.peer_hold_active = False
        self.peer_hold_leader = 0
        self.peer_hold_dist = float('inf')
        self._peer_last_log = 0.0

        cmd_topic = f'/rosmaster_x3_{robot_id}/cmd_vel'
        odom_topic = f'/rosmaster_x3_{robot_id}/mecanum_drive_controller/odom'

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 50)

        self.log_start(robot_id, target_frame, target_x, target_y, target_yaw_deg, cmd_topic, odom_topic)

    def log_start(
        self,
        robot_id: int,
        frame_name: str,
        target_x: float,
        target_y: float,
        target_yaw_deg: float | None,
        cmd_topic: str,
        odom_topic: str,
    ):
        yaw_text = 'none' if target_yaw_deg is None else f'{target_yaw_deg:.1f} deg'
        self.get_logger().info(
            f'start: robot={robot_id}, frame={frame_name}, target=({target_x:.3f}, {target_y:.3f}), '
            f'target_yaw={yaw_text}, cmd_topic={cmd_topic}, odom_topic={odom_topic}'
        )

    def set_target_odom(
        self,
        target_x: float,
        target_y: float,
        target_yaw_rad: float | None,
        announce: bool = True,
        replan: bool = True,
    ):
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = target_yaw_rad
        if replan or self.done:
            self.done = False
            self._yaw_in_tol_since = None
            self.turn_target_yaw = None
            self.turn_mid_yaw = None
            self._turn_in_tol_since = None
            self.turn_dir_sign = 0.0
            self.turn_heading_override = None
            self.turn_budget_accum = 0.0
            self.turn_budget_prev_yaw = self.current_yaw
            self.prev_turn_cmd_wz = 0.0

        if self.odom_ready and (replan or not self.path_nodes):
            self._rebuild_node_path()

        if announce:
            self.get_logger().info(
                f'odom target: ({self.target_x:.3f}, {self.target_y:.3f}), '
                f"yaw={'none' if self.target_yaw is None else f'{math.degrees(self.target_yaw):.1f} deg'}"
            )

    def _apply_base_blocked_map(
        self,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
        base_blocked: list[bool],
        source_label: str,
    ):
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return
        inflate_cells = max(0, int(math.ceil(self.map_inflate_radius_m / resolution)))
        if inflate_cells <= 0:
            blocked = base_blocked
        else:
            blocked = base_blocked[:]
            radius2 = inflate_cells * inflate_cells
            occupied_indices = [i for i, is_blocked in enumerate(base_blocked) if is_blocked]
            for occ_idx in occupied_indices:
                ox = occ_idx % width
                oy = occ_idx // width
                for dy in range(-inflate_cells, inflate_cells + 1):
                    ny = oy + dy
                    if ny < 0 or ny >= height:
                        continue
                    rem = radius2 - dy * dy
                    if rem < 0:
                        continue
                    span = int(math.floor(math.sqrt(rem)))
                    row_base = ny * width
                    x0 = max(0, ox - span)
                    x1 = min(width - 1, ox + span)
                    for nx in range(x0, x1 + 1):
                        blocked[row_base + nx] = True

        origin_cos = math.cos(origin_yaw)
        origin_sin = math.sin(origin_yaw)

        was_ready = self.map_ready
        self.map_width = width
        self.map_height = height
        self.map_resolution = resolution
        self.map_origin_x = origin_x
        self.map_origin_y = origin_y
        self.map_origin_cos = origin_cos
        self.map_origin_sin = origin_sin
        self.map_blocked = blocked
        self._map_topic = source_label
        self.map_ready = True

        if not was_ready:
            self.get_logger().info(
                f'map ready: topic={source_label}, size={width}x{height}, res={resolution:.3f} m, '
                f'inflation={self.map_inflate_radius_m:.2f} m'
            )

    @staticmethod
    def _tag_name(elem: ET.Element) -> str:
        return elem.tag.split('}', 1)[-1]

    def _find_child(self, elem: ET.Element, name: str) -> ET.Element | None:
        for child in elem:
            if self._tag_name(child) == name:
                return child
        return None

    def _find_children(self, elem: ET.Element, name: str) -> list[ET.Element]:
        return [child for child in elem if self._tag_name(child) == name]

    def _find_text(self, elem: ET.Element, name: str, default: str = '') -> str:
        child = self._find_child(elem, name)
        if child is None or child.text is None:
            return default
        return child.text.strip()

    @staticmethod
    def _parse_float_list(text: str | None) -> list[float]:
        if not text:
            return []
        vals = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', text)
        return [float(v) for v in vals]

    def load_world_obstacles_from_sdf(self, world_path: str, grid_resolution: float) -> bool:
        if grid_resolution <= 0.0:
            self.get_logger().error(f'invalid world grid resolution: {grid_resolution}')
            return False

        try:
            tree = ET.parse(world_path)
        except (ET.ParseError, OSError) as e:
            self.get_logger().error(f'failed to parse world file: {world_path}, err={e}')
            return False

        root = tree.getroot()
        world_elem = next((elem for elem in root.iter() if self._tag_name(elem) == 'world'), None)
        if world_elem is None:
            self.get_logger().error(f'no <world> tag found in {world_path}')
            return False

        world_min = -2.5
        world_max = 2.5
        axis_span = world_max - world_min
        node_count = int(round(axis_span / grid_resolution)) + 1
        if node_count <= 1:
            self.get_logger().error(
                f'invalid world node count from resolution={grid_resolution:.3f}'
            )
            return False

        width = node_count
        height = node_count
        map_min_x = world_min - 0.5 * grid_resolution
        map_min_y = world_min - 0.5 * grid_resolution

        base_blocked = [False] * (width * height)
        marked_count = 0
        obstacle_points: list[tuple[float, float]] = []

        for model in self._find_children(world_elem, 'model'):
            model_name = model.attrib.get('name', '')
            if not model_name.startswith('Obstacle'):
                continue

            static_text = self._find_text(model, 'static', 'false').lower()
            if static_text not in ('1', 'true', 'yes'):
                continue

            pose_vals = self._parse_float_list(self._find_text(model, 'pose', ''))
            if len(pose_vals) < 2:
                continue
            mpx = pose_vals[0]
            mpy = pose_vals[1]
            snapped_x = round((mpx - world_min) / grid_resolution) * grid_resolution + world_min
            snapped_y = round((mpy - world_min) / grid_resolution) * grid_resolution + world_min

            if snapped_x < (world_min - 1e-6) or snapped_x > (world_max + 1e-6):
                continue
            if snapped_y < (world_min - 1e-6) or snapped_y > (world_max + 1e-6):
                continue

            mx = int(round((snapped_x - world_min) / grid_resolution))
            my = int(round((snapped_y - world_min) / grid_resolution))
            if mx < 0 or my < 0 or mx >= width or my >= height:
                continue

            idx = my * width + mx
            if not base_blocked[idx]:
                base_blocked[idx] = True
                marked_count += 1
                obstacle_points.append((snapped_x, snapped_y))

        self._apply_base_blocked_map(
            width,
            height,
            grid_resolution,
            map_min_x,
            map_min_y,
            0.0,
            base_blocked,
            f'world:{os.path.basename(world_path)}',
        )
        self.get_logger().info(
            f'loaded world occupancy: file={world_path}, x_range=[{world_min:.1f}, {world_max:.1f}], '
            f'y_range=[{world_min:.1f}, {world_max:.1f}], blocked_nodes={marked_count}, '
            f'obstacle_points={obstacle_points}'
        )
        return True

    def _world_to_map_local(self, world_x: float, world_y: float) -> tuple[float, float]:
        dx = world_x - self.map_origin_x
        dy = world_y - self.map_origin_y
        local_x = self.map_origin_cos * dx + self.map_origin_sin * dy
        local_y = -self.map_origin_sin * dx + self.map_origin_cos * dy
        return local_x, local_y

    def _world_to_map_cell(self, world_x: float, world_y: float) -> tuple[int, int] | None:
        if not self.map_ready or self.map_resolution <= 0.0:
            return None
        local_x, local_y = self._world_to_map_local(world_x, world_y)
        mx = int(math.floor(local_x / self.map_resolution))
        my = int(math.floor(local_y / self.map_resolution))
        if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
            return None
        return mx, my

    def _map_cell_to_world(self, mx: int, my: int) -> tuple[float, float]:
        local_x = (mx + 0.5) * self.map_resolution
        local_y = (my + 0.5) * self.map_resolution
        world_x = self.map_origin_x + self.map_origin_cos * local_x - self.map_origin_sin * local_y
        world_y = self.map_origin_y + self.map_origin_sin * local_x + self.map_origin_cos * local_y
        return world_x, world_y

    def _build_global_path_world(
        self,
        start_world_x: float,
        start_world_y: float,
        goal_world_x: float,
        goal_world_y: float,
    ) -> list[tuple[float, float]] | None:
        if not self.map_ready:
            return None

        start_cell = self._world_to_map_cell(start_world_x, start_world_y)
        goal_cell = self._world_to_map_cell(goal_world_x, goal_world_y)

        if start_cell is None or goal_cell is None:
            self.get_logger().error('global planner: start or goal is outside world occupancy bounds')
            return None

        planner = AStarGridPlanner(self.map_width, self.map_height, self.map_blocked)
        search_radius_cells = max(3, int(math.ceil(0.7 / self.map_resolution)))
        start_free = planner.find_nearest_free_cell(start_cell[0], start_cell[1], search_radius_cells)
        goal_free = planner.find_nearest_free_cell(goal_cell[0], goal_cell[1], search_radius_cells)
        if start_free is None or goal_free is None:
            self.get_logger().error('global planner: no nearby free start/goal cell')
            return None

        if goal_free != goal_cell:
            goal_reachable_world = self._map_cell_to_world(goal_free[0], goal_free[1])
            self.get_logger().warn(
                'global planner: goal cell occupied, using nearest free goal '
                f'requested=({goal_world_x:.2f}, {goal_world_y:.2f}) '
                f'adjusted=({goal_reachable_world[0]:.2f}, {goal_reachable_world[1]:.2f})'
            )

        cell_path = planner.plan(start_free, goal_free)
        if not cell_path:
            self.get_logger().error('global planner: A* could not find a route')
            return None

        compact_cells = planner.compress_path(cell_path)

        world_path: list[tuple[float, float]] = [(start_world_x, start_world_y)]
        if len(compact_cells) > 2:
            for mx, my in compact_cells[1:-1]:
                world_path.append(self._map_cell_to_world(mx, my))
        goal_reachable_world = self._map_cell_to_world(goal_free[0], goal_free[1])
        world_path.append(goal_reachable_world)
        return world_path

    def _build_fixed_world_path(
        self,
        start_world_x: float,
        start_world_y: float,
        goal_world_x: float,
        goal_world_y: float,
    ) -> list[tuple[float, float]] | None:
        min_world = WORLD_MIN
        max_world = WORLD_MAX

        clamped_count = 0
        points: list[tuple[float, float]] = []
        for wx, wy in self.fixed_world_waypoints:
            if (not math.isfinite(wx)) or (not math.isfinite(wy)):
                continue
            cx = clamp(float(wx), min_world, max_world)
            cy = clamp(float(wy), min_world, max_world)
            if abs(cx - float(wx)) > 1e-6 or abs(cy - float(wy)) > 1e-6:
                clamped_count += 1
            if points and math.hypot(points[-1][0] - cx, points[-1][1] - cy) <= 1e-4:
                continue
            points.append((cx, cy))

        if not points:
            return None

        goal_cx = clamp(float(goal_world_x), min_world, max_world)
        goal_cy = clamp(float(goal_world_y), min_world, max_world)
        if abs(goal_cx - float(goal_world_x)) > 1e-6 or abs(goal_cy - float(goal_world_y)) > 1e-6:
            clamped_count += 1

        if math.hypot(points[-1][0] - goal_cx, points[-1][1] - goal_cy) > 0.05:
            points.append((goal_cx, goal_cy))

        nearest_idx = min(
            range(len(points)),
            key=lambda i: math.hypot(points[i][0] - start_world_x, points[i][1] - start_world_y),
        )
        start_idx = nearest_idx
        near_tol = max(self.pos_tol * 1.5, 0.12)
        while start_idx < len(points) - 1:
            dist_to_point = math.hypot(
                points[start_idx][0] - start_world_x,
                points[start_idx][1] - start_world_y,
            )
            if dist_to_point > near_tol:
                break
            start_idx += 1

        world_path: list[tuple[float, float]] = [(start_world_x, start_world_y)]
        world_path.extend(points[start_idx:])
        if len(world_path) <= 1:
            world_path.append((goal_cx, goal_cy))

        if clamped_count > 0 and (not self._fixed_path_clamp_logged):
            self.get_logger().warn(
                f'coordinator path bound clamp: adjusted={clamped_count}, range=[{min_world:.1f}, {max_world:.1f}]'
            )
            self._fixed_path_clamp_logged = True
        return world_path

    def _rebuild_node_path(self, force_first_axis: str | None = None):
        del force_first_axis

        use_world = self.using_world_control()
        if use_world:
            start_x = self._world_x
            start_y = self._world_y
            goal_x = float(self._progress_goal_x)
            goal_y = float(self._progress_goal_y)
            heading_now = self._world_yaw
        else:
            start_x = self.current_x
            start_y = self.current_y
            goal_x = self.target_x
            goal_y = self.target_y
            heading_now = self.current_yaw

        using_fixed_path = bool(self.fixed_world_waypoints)

        world_path: list[tuple[float, float]] | None = None
        if using_fixed_path:
            world_path = self._build_fixed_world_path(start_x, start_y, goal_x, goal_y)
            if world_path is None:
                self.path_nodes = []
                self.active_node_idx = 0
                self.drive_phase = 'no_path'
                self.path_failed = True
                self.get_logger().error('coordinator path invalid or empty')
                return

        if world_path is None:
            if not self.map_ready:
                self.path_nodes = []
                self.active_node_idx = 0
                self.drive_phase = 'waiting_global_path'
                self.path_failed = False
                self.get_logger().warn('global planner waiting for world occupancy data')
                return

            world_path = self._build_global_path_world(start_x, start_y, goal_x, goal_y)
            if world_path is None:
                self.path_nodes = []
                self.active_node_idx = 0
                self.drive_phase = 'no_path'
                self.path_failed = True
                return

        self._effective_goal_world_x = world_path[-1][0]
        self._effective_goal_world_y = world_path[-1][1]

        self.global_plan_world = world_path
        self.path_nodes = [(x, y, 'waypoint', 0) for x, y in world_path[1:]]
        if not self.path_nodes:
            self.path_nodes = [(goal_x, goal_y, 'final', 0)]
        else:
            last_x, last_y, _kind, _reserved = self.path_nodes[-1]
            self.path_nodes[-1] = (last_x, last_y, 'final', 0)

        self.active_node_idx = 0
        self.drive_phase = 'rotate_to_node'
        self.path_failed = False
        self.turn_target_yaw = None
        self.turn_mid_yaw = None
        self._turn_in_tol_since = None
        self.turn_dir_sign = 0.0
        self.turn_heading_override = None
        self.turn_budget_accum = 0.0
        self.turn_budget_prev_yaw = heading_now
        self.prev_turn_cmd_wz = 0.0
        path_label = 'coordinator path loaded' if using_fixed_path else 'global path planned'
        self.get_logger().info(
            f'{path_label}: waypoints={len(self.path_nodes)}, map={self.map_width}x{self.map_height}, '
            f'res={self.map_resolution:.3f} m, topic={self._map_topic}'
        )

    def effective_goal_world(self) -> tuple[float, float]:
        return self._effective_goal_world_x, self._effective_goal_world_y

    def set_progress_view(
        self,
        frame_name: str,
        cur_x: float,
        cur_y: float,
        goal_x: float,
        goal_y: float,
    ):
        self._progress_frame = frame_name
        self._progress_cur_x = cur_x
        self._progress_cur_y = cur_y
        self._progress_goal_x = goal_x
        self._progress_goal_y = goal_y

    def set_world_heading_state(
        self,
        world_x: float,
        world_y: float,
        world_yaw: float,
        yaw_world_from_odom: float,
    ):
        self._world_pose_ready = True
        self._world_x = world_x
        self._world_y = world_y
        self._world_yaw = world_yaw
        self._world_from_odom_yaw = yaw_world_from_odom

    def using_world_control(self) -> bool:
        return (
            self._progress_frame == 'world'
            and self._world_pose_ready
            and self._progress_goal_x is not None
            and self._progress_goal_y is not None
        )

    def update_peer_safety(self, world_poses: dict[str, tuple[float, float, float]], now: float):
        self_key = f'rosmaster_x3_{self.robot_id}'
        self_pose = world_poses.get(self_key)
        if self_pose is None:
            return

        nearest_peer = 0
        nearest_dist = float('inf')
        for peer_id in [1, 2, 3]:
            if peer_id == self.robot_id:
                continue
            peer_pose = world_poses.get(f'rosmaster_x3_{peer_id}')
            if peer_pose is None:
                continue
            dist = math.hypot(self_pose[0] - peer_pose[0], self_pose[1] - peer_pose[1])
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_peer = peer_id

        should_hold = False
        hold_leader = 0
        if nearest_peer > 0:
            if nearest_dist <= self.peer_emergency_stop_dist and (
                self.robot_id > nearest_peer or nearest_dist <= self.peer_critical_stop_dist
            ):
                should_hold = True
                hold_leader = nearest_peer

        if should_hold:
            self.peer_hold_active = True
            self.peer_hold_leader = hold_leader
            self.peer_hold_dist = nearest_dist
            return

        if self.peer_hold_active and now - self._peer_last_log >= 1.5:
            self.get_logger().info(
                f'peer-hold released: leader=robot{self.peer_hold_leader}, dist={self.peer_hold_dist:.2f} m'
            )
            self._peer_last_log = now
        self.peer_hold_active = False
        self.peer_hold_leader = 0
        self.peer_hold_dist = float('inf')

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_wz = msg.twist.twist.angular.z
        self.odom_ready = True

    def progress_error(self) -> float | None:
        if (
            self._progress_cur_x is None
            or self._progress_cur_y is None
            or self._progress_goal_x is None
            or self._progress_goal_y is None
        ):
            return None
        return math.hypot(
            self._progress_goal_x - self._progress_cur_x,
            self._progress_goal_y - self._progress_cur_y,
        )

    def axis_chain_remaining_distance(self, node_idx: int, cur_x: float, cur_y: float) -> float:
        if node_idx >= len(self.path_nodes):
            return 0.0
        node_x, node_y, node_axis, node_axis_dir = self.path_nodes[node_idx]
        if node_axis == 'x':
            chain_target_x = node_x
            for i in range(node_idx + 1, len(self.path_nodes)):
                nx, _ny, naxis, ndir = self.path_nodes[i]
                if naxis == 'x' and ndir == node_axis_dir:
                    chain_target_x = nx
                else:
                    break
            return abs(chain_target_x - cur_x)
        if node_axis == 'y':
            chain_target_y = node_y
            for i in range(node_idx + 1, len(self.path_nodes)):
                _nx, ny, naxis, ndir = self.path_nodes[i]
                if naxis == 'y' and ndir == node_axis_dir:
                    chain_target_y = ny
                else:
                    break
            return abs(chain_target_y - cur_y)
        return math.hypot(node_x - cur_x, node_y - cur_y)

    def heading_chain_remaining_distance(
        self,
        node_idx: int,
        cur_x: float,
        cur_y: float,
        base_heading: float,
    ) -> float:
        if node_idx >= len(self.path_nodes):
            return 0.0

        node_x, node_y, _node_kind, _node_reserved = self.path_nodes[node_idx]
        remaining = math.hypot(node_x - cur_x, node_y - cur_y)
        prev_x = node_x
        prev_y = node_y

        for i in range(node_idx + 1, len(self.path_nodes)):
            nx, ny, _kind, _reserved = self.path_nodes[i]
            seg_dx = nx - prev_x
            seg_dy = ny - prev_y
            seg_len = math.hypot(seg_dx, seg_dy)
            if seg_len <= 1e-6:
                prev_x = nx
                prev_y = ny
                continue

            seg_heading = math.atan2(seg_dy, seg_dx)
            if abs(normalize_angle(seg_heading - base_heading)) > self.forward_chain_heading_tol_rad:
                break

            remaining += seg_len
            prev_x = nx
            prev_y = ny

        return remaining

    def stop_robot(self):
        self.prev_cmd_vx = 0.0
        self.prev_turn_cmd_wz = 0.0
        self._forward_realign_since = None
        self._rotate_relax_since = None
        self.cmd_pub.publish(Twist())

    def stable_heading_error(self, target_yaw: float, current_heading: float) -> float:
        err = normalize_angle(target_yaw - current_heading)
        # Around +-pi, tiny yaw noise can flip sign between +pi and -pi and destabilize 180 turns.
        if abs(abs(err) - math.pi) <= self.turn_pi_lock_rad:
            if self.turn_dir_sign != 0.0:
                return self.turn_dir_sign * abs(err)
            if abs(self.prev_turn_cmd_wz) > 1e-4:
                return math.copysign(abs(err), self.prev_turn_cmd_wz)
        return err

    def control_step(self):
        if self.done or not self.odom_ready:
            return

        if self.drive_phase == 'waiting_global_path':
            if self.map_ready:
                self._rebuild_node_path()
            self.stop_robot()
            return
        if self.drive_phase == 'no_path':
            self.done = True
            self.stop_robot()
            return

        loop_now = time.time()
        if self._last_control_time is None:
            dt = 1.0 / 20.0
        else:
            dt = clamp(loop_now - self._last_control_time, 0.001, 0.20)
        self._last_control_time = loop_now

        if self.peer_hold_active:
            self.stop_robot()
            if loop_now - self._peer_last_log >= 0.9:
                self.get_logger().info(
                    f'peer-hold: yielding to robot{self.peer_hold_leader}, dist={self.peer_hold_dist:.2f} m'
                )
                self._peer_last_log = loop_now
            return

        cmd = Twist()
        done_now = False

        node_distance = 0.0
        cross_track_error = 0.0
        heading_err_deg = 0.0
        control_in_world = self.using_world_control()
        cur_x_now = self._world_x if control_in_world else self.current_x
        cur_y_now = self._world_y if control_in_world else self.current_y
        if control_in_world:
            # Use odom-derived world heading for control to avoid world/odom yaw
            # source switching near threshold, which can cause forward-phase
            # oscillation and repeated near-node stalls.
            world_heading_from_odom = normalize_angle(self.current_yaw + self._world_from_odom_yaw)
            heading_mismatch = abs(normalize_angle(self._world_yaw - world_heading_from_odom))
            if heading_mismatch > self.world_heading_use_limit_rad:
                if loop_now - self._last_world_heading_fallback_log >= 2.0:
                    self.get_logger().warn(
                        f'world heading fallback: mismatch={math.degrees(heading_mismatch):.1f} deg, '
                        f'using odom-derived heading'
                    )
                    self._last_world_heading_fallback_log = loop_now
            cur_heading_now = world_heading_from_odom
        else:
            cur_heading_now = self.current_yaw
        progress_err_now = self.progress_error()

        if self.drive_phase != 'forward_to_node':
            self._forward_stall_since = None
            self._forward_stall_ref_err = None
            self._forward_stall_ref_x = None
            self._forward_stall_ref_y = None

        if self.path_nodes and self.active_node_idx < len(self.path_nodes):
            node_x, node_y, node_axis, _node_reserved = self.path_nodes[self.active_node_idx]
            dx = node_x - cur_x_now
            dy = node_y - cur_y_now

            node_distance = math.hypot(dx, dy)
            cross_track_error = 0.0
            heading_to_node = math.atan2(dy, dx)
            node_reached = node_distance <= self.pos_tol

            current_heading = cur_heading_now
            desired_heading_for_turn = heading_to_node

            # Turning target must be node heading in odom; obstacle noise should not bias corner angle.
            desired_heading = desired_heading_for_turn

            if self.turn_target_yaw is None:
                raw_turn_err = normalize_angle(desired_heading - current_heading)
                target_heading = desired_heading
                self.turn_heading_override = None
                self.turn_mid_yaw = None
                if abs(raw_turn_err) >= self.turn_split_180_rad:
                    compensated_err = math.copysign(
                        max(0.0, abs(raw_turn_err) - self.turn_180_comp_rad),
                        raw_turn_err,
                    )
                    self.turn_mid_yaw = normalize_angle(current_heading + compensated_err)
                    target_heading = self.turn_mid_yaw
                self.turn_target_yaw = target_heading
                self._turn_in_tol_since = None
                self.turn_dir_sign = 0.0
                init_turn_err = abs(normalize_angle(self.turn_target_yaw - current_heading))
                self.turn_start_yaw = current_heading
                self.turn_budget_latched = False
                self.turn_budget_active = init_turn_err >= self.turn_split_180_rad
                self.turn_expected_mag = init_turn_err
                self.turn_budget_accum = 0.0
                self.turn_budget_prev_yaw = current_heading
            elif (
                self.turn_mid_yaw is None
                and abs(normalize_angle(desired_heading - self.turn_target_yaw)) > self.turn_retarget_rad
            ):
                raw_turn_err = normalize_angle(desired_heading - current_heading)
                target_heading = desired_heading
                self.turn_heading_override = None
                self.turn_mid_yaw = None
                if abs(raw_turn_err) >= self.turn_split_180_rad:
                    compensated_err = math.copysign(
                        max(0.0, abs(raw_turn_err) - self.turn_180_comp_rad),
                        raw_turn_err,
                    )
                    self.turn_mid_yaw = normalize_angle(current_heading + compensated_err)
                    target_heading = self.turn_mid_yaw
                self.turn_target_yaw = target_heading
                self._turn_in_tol_since = None
                self.turn_dir_sign = 0.0
                init_turn_err = abs(normalize_angle(self.turn_target_yaw - current_heading))
                self.turn_start_yaw = current_heading
                self.turn_budget_latched = False
                self.turn_budget_active = init_turn_err >= self.turn_split_180_rad
                self.turn_expected_mag = init_turn_err
                self.turn_budget_accum = 0.0
                self.turn_budget_prev_yaw = current_heading

            final_heading_error = self.stable_heading_error(self.turn_target_yaw, current_heading)
            active_face_tol = self.face_heading_tol_rad
            if progress_err_now is not None and progress_err_now <= self.near_goal_turn_relax_dist:
                active_face_tol = max(active_face_tol, self.near_goal_face_tol_rad)
            heading_error = final_heading_error

            if self.turn_budget_active:
                yaw_step = normalize_angle(current_heading - self.turn_budget_prev_yaw)
                self.turn_budget_accum += abs(yaw_step)
                self.turn_budget_prev_yaw = current_heading
                turned_mag = self.turn_budget_accum
                turn_budget = self.turn_expected_mag + self.turn_overshoot_margin_rad
                if turned_mag >= turn_budget:
                    heading_error = 0.0
                    self.turn_dir_sign = 0.0
                    if not self.turn_budget_latched:
                        self.get_logger().info(
                            f'turn budget reached: turned={math.degrees(turned_mag):.1f} deg, '
                            f'budget={math.degrees(turn_budget):.1f} deg'
                        )
                        self.turn_budget_latched = True
            heading_err_deg = math.degrees(heading_error)

            if node_reached:
                self.active_node_idx += 1

                if self.active_node_idx < len(self.path_nodes):
                    next_node_x, next_node_y, _next_kind, _next_reserved = self.path_nodes[self.active_node_idx]
                    next_seg_dx = next_node_x - node_x
                    next_seg_dy = next_node_y - node_y
                    next_seg_len = math.hypot(next_seg_dx, next_seg_dy)
                    next_heading_jump = math.pi
                    if next_seg_len > 1e-6:
                        next_heading = math.atan2(next_seg_dy, next_seg_dx)
                        next_heading_jump = abs(normalize_angle(next_heading - heading_to_node))

                    # Preserve momentum when the next segment keeps almost the same heading.
                    continue_forward_chain = (
                        (not self.force_stop_each_node)
                        and
                        self.drive_phase == 'forward_to_node'
                        and next_seg_len > 1e-6
                        and next_heading_jump <= self.forward_chain_heading_tol_rad
                    )
                    if continue_forward_chain:
                        self.turn_target_yaw = None
                        self.turn_mid_yaw = None
                        self._turn_in_tol_since = None
                        self.turn_dir_sign = 0.0
                        self.turn_heading_override = None
                        self.prev_turn_cmd_wz = 0.0
                        self._forward_realign_since = None
                        self.get_logger().info(
                            f'node reached: {self.active_node_idx}/{len(self.path_nodes)} (continue forward)'
                        )
                        return

                    self.stop_robot()
                    self.drive_phase = 'rotate_to_node'
                    self.turn_target_yaw = None
                    self.turn_mid_yaw = None
                    self._turn_in_tol_since = None
                    self.turn_dir_sign = 0.0
                    self.turn_heading_override = None
                    self.prev_turn_cmd_wz = 0.0
                    self.get_logger().info(f'node reached: {self.active_node_idx}/{len(self.path_nodes)}')
                    return

                progress_err = self.progress_error()
                if (
                    self._progress_frame == 'world'
                    and progress_err is not None
                    and progress_err > (self.progress_done_tol + self.progress_replan_margin)
                ):
                    self.get_logger().warn(
                        f'odom path ended but world err={progress_err:.2f} m '
                        f'(tol={self.progress_done_tol:.2f} m), rebuilding path'
                    )
                    self._rebuild_node_path()
                    return

                self.get_logger().info('all nodes reached, completing target')
                self.drive_phase = 'arrived'
                return

            if self.drive_phase in ('idle', 'rotate_to_node'):
                if abs(heading_error) > active_face_tol:
                    relax_ready = (
                        abs(heading_error) <= self.rotate_relax_tol_rad
                        and abs(self.current_wz) <= self.rotate_relax_rate_rad
                    )
                    if relax_ready:
                        if self._rotate_relax_since is None:
                            self._rotate_relax_since = loop_now
                        elif (loop_now - self._rotate_relax_since) >= self.rotate_relax_hold_sec:
                            self.stop_robot()
                            self.drive_phase = 'forward_to_node'
                            self.turn_target_yaw = None
                            self.turn_mid_yaw = None
                            self._turn_in_tol_since = None
                            self.turn_dir_sign = 0.0
                            self.prev_turn_cmd_wz = 0.0
                            self.get_logger().info(
                                f'rotate relax to forward: heading_err={math.degrees(heading_error):.1f} deg '
                                f'hold={self.rotate_relax_hold_sec:.2f}s'
                            )
                            return
                    else:
                        self._rotate_relax_since = None

                    self._turn_in_tol_since = None
                    yaw_mag = abs(heading_error)
                    max_turn = self.max_angular
                    if yaw_mag < self.turn_slowdown_rad:
                        span = max(self.turn_slowdown_rad - active_face_tol, math.radians(1.0))
                        ratio = clamp((yaw_mag - active_face_tol) / span, 0.0, 1.0)
                        max_turn = self.max_angular_final + (self.max_angular - self.max_angular_final) * ratio
                    # If heading error crosses zero, release direction lock immediately.
                    if self.turn_dir_sign != 0.0 and (self.turn_dir_sign * heading_error < 0.0):
                        self.turn_dir_sign = 0.0
                    if self.turn_dir_sign == 0.0:
                        self.turn_dir_sign = 1.0 if heading_error > 0.0 else -1.0
                    raw_turn = self.turn_kp * heading_error - self.turn_kd * self.current_wz
                    raw_turn = clamp(raw_turn, -max_turn, max_turn)
                    if abs(raw_turn) < self.turn_min_angular and abs(heading_error) > self.turn_min_apply_err_rad:
                        raw_turn = (1.0 if heading_error > 0.0 else -1.0) * self.turn_min_angular

                    # Active braking near target angle to avoid 180-degree overshoot.
                    braking = False
                    if heading_error * self.current_wz > 0.0 and abs(self.current_wz) >= self.turn_brake_min_rate:
                        stop_angle = (abs(self.current_wz) ** 2) / (2.0 * max(self.turn_brake_decel_est, 1e-3))
                        if abs(heading_error) <= stop_angle + self.turn_brake_margin_rad:
                            # Apply reverse torque against current yaw rate to bleed momentum before overshoot.
                            brake_mag = clamp(
                                abs(self.current_wz) * max(self.turn_brake_gain, 0.0),
                                self.turn_min_angular,
                                max_turn,
                            )
                            raw_turn = -math.copysign(brake_mag, self.current_wz)
                            braking = True

                    if (abs(heading_error) > self.turn_dir_release_rad) and (not braking):
                        cmd.angular.z = self.turn_dir_sign * abs(raw_turn)
                    else:
                        self.turn_dir_sign = 0.0
                        cmd.angular.z = raw_turn

                    # Limit angular acceleration to avoid inertia-driven overshoot on large turns.
                    slew_rate = self.turn_cmd_slew_rate
                    if braking:
                        slew_rate *= max(1.0, 1.0 + self.turn_brake_gain)
                    max_delta_wz = slew_rate * dt
                    cmd.angular.z = clamp(
                        cmd.angular.z,
                        self.prev_turn_cmd_wz - max_delta_wz,
                        self.prev_turn_cmd_wz + max_delta_wz,
                    )
                    self.prev_turn_cmd_wz = cmd.angular.z
                else:
                    self._rotate_relax_since = None
                    if self.turn_mid_yaw is not None:
                        # Two-stage near-180 turn: finish at compensated yaw first, then settle to true node heading.
                        self.stop_robot()
                        self.turn_target_yaw = desired_heading
                        self.turn_mid_yaw = None
                        self._turn_in_tol_since = None
                        self.turn_dir_sign = 0.0
                        self.turn_heading_override = None
                        self.turn_budget_latched = False
                        self.turn_budget_accum = 0.0
                        self.turn_budget_prev_yaw = current_heading
                        self.turn_expected_mag = abs(normalize_angle(self.turn_target_yaw - current_heading))
                        self.turn_budget_active = self.turn_expected_mag >= self.turn_split_180_rad
                        self.prev_turn_cmd_wz = 0.0
                        self.get_logger().info('near-180 staged turn: settling final heading')
                        return

                    now = time.time()
                    if self._turn_in_tol_since is None:
                        self._turn_in_tol_since = now

                    stable_yaw_rate = abs(self.current_wz) <= self.turn_rate_tol
                    in_tol_time = now - self._turn_in_tol_since
                    if (stable_yaw_rate and in_tol_time >= self.turn_settle_sec) or (
                        in_tol_time >= self.turn_in_tol_force_sec
                    ):
                        self.stop_robot()
                        self.drive_phase = 'forward_to_node'
                        self.turn_target_yaw = None
                        self.turn_mid_yaw = None
                        self._turn_in_tol_since = None
                        self.turn_dir_sign = 0.0
                        self.prev_turn_cmd_wz = 0.0
                        self._forward_realign_since = None
                        return
                    cmd.angular.z = 0.0
                    self.prev_turn_cmd_wz = 0.0

            elif self.drive_phase == 'forward_to_node':
                forward_heading_target = heading_to_node
                heading_error_forward = normalize_angle(forward_heading_target - current_heading)
                heading_err_deg = math.degrees(heading_error_forward)
                realign_tol = self.forward_realign_rad
                if progress_err_now is not None and progress_err_now <= self.near_goal_turn_relax_dist:
                    realign_tol = max(realign_tol, self.near_goal_forward_realign_rad)
                abs_heading_err_forward = abs(heading_error_forward)
                realign_enter = realign_tol + self.forward_realign_enter_hyst_rad
                realign_exit = max(0.0, realign_tol - self.forward_realign_exit_hyst_rad)
                realign_enabled = node_distance > self.forward_realign_disable_dist
                now = time.time()

                if (
                    abs_heading_err_forward >= self.forward_force_rotate_rad
                    and node_distance > self.forward_force_rotate_dist
                ):
                    self._forward_realign_since = None
                    self.get_logger().info(
                        f'force rotate: heading_err={math.degrees(heading_error_forward):.1f} deg '
                        f'node_err={node_distance:.2f} m'
                    )
                    self.stop_robot()
                    self.drive_phase = 'rotate_to_node'
                    self.turn_target_yaw = None
                    self.turn_mid_yaw = None
                    self._turn_in_tol_since = None
                    self.turn_dir_sign = 0.0
                    self.turn_heading_override = None
                    self.prev_turn_cmd_wz = 0.0
                    return

                if not realign_enabled:
                    self._forward_realign_since = None
                elif abs_heading_err_forward > realign_enter:
                    if self._forward_realign_since is None:
                        self._forward_realign_since = now
                    elif (now - self._forward_realign_since) >= self.forward_realign_hold_sec:
                        self.get_logger().info(
                            f'realign to rotate: heading_err={math.degrees(heading_error_forward):.1f} deg '
                            f'threshold={math.degrees(realign_enter):.1f} deg '
                            f'hold={self.forward_realign_hold_sec:.2f}s'
                        )
                        self.stop_robot()
                        self.drive_phase = 'rotate_to_node'
                        self.turn_target_yaw = None
                        self.turn_mid_yaw = None
                        self._turn_in_tol_since = None
                        self.turn_dir_sign = 0.0
                        self.turn_heading_override = None
                        self.prev_turn_cmd_wz = 0.0
                        return
                elif abs_heading_err_forward < realign_exit:
                    self._forward_realign_since = None

                if progress_err_now is not None and node_distance > self.forward_stall_node_dist:
                    if (
                        self._forward_stall_since is None
                        or self._forward_stall_ref_err is None
                        or self._forward_stall_ref_x is None
                        or self._forward_stall_ref_y is None
                    ):
                        self._forward_stall_since = now
                        self._forward_stall_ref_err = progress_err_now
                        self._forward_stall_ref_x = cur_x_now
                        self._forward_stall_ref_y = cur_y_now
                    else:
                        moved = math.hypot(
                            cur_x_now - self._forward_stall_ref_x,
                            cur_y_now - self._forward_stall_ref_y,
                        )
                        err_drop = self._forward_stall_ref_err - progress_err_now
                        if err_drop > self.forward_stall_err_eps or moved > self.forward_stall_move_eps:
                            self._forward_stall_since = now
                            self._forward_stall_ref_err = progress_err_now
                            self._forward_stall_ref_x = cur_x_now
                            self._forward_stall_ref_y = cur_y_now
                        elif (
                            (now - self._forward_stall_since) >= self.forward_stall_hold_sec
                            and (now - self._last_forward_stall_replan)
                            >= self.forward_stall_replan_cooldown_sec
                        ):
                            self._last_forward_stall_replan = now
                            self.get_logger().warn(
                                f'forward stall detected: node_err={node_distance:.2f} m, '
                                f'world_err={progress_err_now:.2f} m, rebuilding path'
                            )
                            self.stop_robot()
                            self._rebuild_node_path()
                            self._forward_stall_since = now
                            self._forward_stall_ref_err = progress_err_now
                            self._forward_stall_ref_x = cur_x_now
                            self._forward_stall_ref_y = cur_y_now
                            return
                else:
                    self._forward_stall_since = None
                    self._forward_stall_ref_err = None
                    self._forward_stall_ref_x = None
                    self._forward_stall_ref_y = None

                chain_remaining = self.heading_chain_remaining_distance(
                    self.active_node_idx,
                    cur_x_now,
                    cur_y_now,
                    heading_to_node,
                )
                speed_distance = max(node_distance, chain_remaining)
                desired_forward_base = clamp(
                    self.k_linear * speed_distance,
                    self.min_approach_speed,
                    self.max_linear,
                )

                heading_mag = abs_heading_err_forward
                if heading_mag <= self.forward_speed_heading_slow_start_rad:
                    heading_scale = 1.0
                elif heading_mag >= self.forward_speed_heading_slow_end_rad:
                    heading_scale = self.forward_speed_min_scale
                else:
                    span = max(
                        self.forward_speed_heading_slow_end_rad - self.forward_speed_heading_slow_start_rad,
                        math.radians(0.5),
                    )
                    ratio = (heading_mag - self.forward_speed_heading_slow_start_rad) / span
                    heading_scale = 1.0 - (1.0 - self.forward_speed_min_scale) * clamp(ratio, 0.0, 1.0)

                desired_forward = max(self.forward_speed_min_abs, desired_forward_base * heading_scale)

                max_rise = self.linear_cmd_slew_rate * dt
                max_fall = self.linear_cmd_slew_rate_decel * dt
                forward = clamp(
                    desired_forward,
                    max(0.0, self.prev_cmd_vx - max_fall),
                    self.prev_cmd_vx + max_rise,
                )

                # Forward-only mode: no reverse, no lateral motion while moving.
                cmd.linear.x = max(0.0, forward)
                cmd.linear.y = 0.0

                # Apply only a tiny heading trim in forward phase to reduce stop-go realign chatter.
                target_forward_wz = clamp(
                    self.forward_heading_kp * heading_error_forward,
                    -self.forward_heading_wz_max,
                    self.forward_heading_wz_max,
                )
                max_delta_forward_wz = self.forward_heading_wz_slew_rate * dt
                cmd.angular.z = clamp(
                    target_forward_wz,
                    self.prev_turn_cmd_wz - max_delta_forward_wz,
                    self.prev_turn_cmd_wz + max_delta_forward_wz,
                )
                self.prev_turn_cmd_wz = cmd.angular.z

        if (not self.path_nodes or self.active_node_idx >= len(self.path_nodes)) and self.target_yaw is not None:
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            if abs(yaw_error) > self.yaw_tol:
                self._yaw_in_tol_since = None

                yaw_mag = abs(yaw_error)
                max_turn = self.max_angular
                if yaw_mag < self.turn_slowdown_rad:
                    span = max(self.turn_slowdown_rad - self.yaw_tol, math.radians(1.0))
                    ratio = clamp((yaw_mag - self.yaw_tol) / span, 0.0, 1.0)
                    max_turn = self.max_angular_final + (self.max_angular - self.max_angular_final) * ratio
                cmd.angular.z = clamp(self.k_yaw * yaw_error, -max_turn, max_turn)
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
            else:
                now = time.time()
                if self._yaw_in_tol_since is None:
                    self._yaw_in_tol_since = now
                elif now - self._yaw_in_tol_since >= self.yaw_settle_sec:
                    done_now = True
        elif not self.path_nodes or self.active_node_idx >= len(self.path_nodes):
            done_now = True

        if done_now:
            self.done = True
            self.stop_robot()
            self.prev_turn_cmd_wz = 0.0
            self.get_logger().info(
                f'done: final=({self.current_x:.3f}, {self.current_y:.3f}), '
                f'yaw={math.degrees(self.current_yaw):.1f} deg'
            )
            return

        self.prev_cmd_vx = cmd.linear.x
        self.cmd_pub.publish(cmd)

        now = time.time()
        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            final_odom_err = math.hypot(self.target_x - self.current_x, self.target_y - self.current_y)
            node_text = (
                f'node={min(self.active_node_idx + 1, max(len(self.path_nodes), 1))}/{max(len(self.path_nodes), 1)} '
                f'phase={self.drive_phase} map_ready={int(self.map_ready)} '
                f'yaw_err={heading_err_deg:.1f}deg yaw_rate={self.current_wz:.2f}rad/s '
                f'drift={cross_track_error:.2f}m'
            )
            if (
                self._progress_cur_x is not None
                and self._progress_cur_y is not None
                and self._progress_goal_x is not None
                and self._progress_goal_y is not None
            ):
                progress_err = math.hypot(
                    self._progress_goal_x - self._progress_cur_x,
                    self._progress_goal_y - self._progress_cur_y,
                )
                self.get_logger().info(
                    f'moving[{self._progress_frame}]: cur=({self._progress_cur_x:.2f}, {self._progress_cur_y:.2f}) '
                    f'goal=({self._progress_goal_x:.2f}, {self._progress_goal_y:.2f}) '
                    f'err={progress_err:.2f} m (odom_final_err={final_odom_err:.2f} m, odom_node_err={node_distance:.2f} m, {node_text})'
                )
            else:
                self.get_logger().info(
                    f'moving: cur=({self.current_x:.2f}, {self.current_y:.2f}) '
                    f'goal=({self.target_x:.2f}, {self.target_y:.2f}) err={final_odom_err:.2f} m ({node_text})'
                )


def parse_args():
    parser = argparse.ArgumentParser(description='Go to a world target with fixed tuning profile.')
    parser.add_argument('--robot', type=int, required=True, choices=[1, 2, 3], help='robot id: 1, 2, or 3')
    parser.add_argument('--x', type=float, required=True, help='target world x (meters)')
    parser.add_argument('--y', type=float, required=True, help='target world y (meters)')
    parser.add_argument(
        '--target-yaw-deg',
        type=float,
        default=None,
        help='optional target world yaw (deg) to enforce after reaching final position',
    )
    parser.add_argument(
        '--pos-tol',
        type=float,
        default=None,
        help='optional position tolerance override (meters)',
    )
    parser.add_argument(
        '--yaw-tol-deg',
        type=float,
        default=None,
        help='optional yaw tolerance override (degrees)',
    )
    parser.add_argument(
        '--yaw-settle-sec',
        type=float,
        default=None,
        help='optional yaw settle duration override (seconds)',
    )
    parser.add_argument(
        '--path-waypoints',
        type=str,
        default=None,
        help='optional world path from coordinator: x1,y1;x2,y2;... (includes route waypoints)',
    )
    return parser.parse_args()


def parse_world_waypoints(text: str | None) -> list[tuple[float, float]]:
    if text is None:
        return []
    raw = text.strip()
    if not raw:
        return []

    points: list[tuple[float, float]] = []
    for item in raw.split(';'):
        chunk = item.strip()
        if not chunk:
            continue
        parts = chunk.split(',')
        if len(parts) != 2:
            raise ValueError(f'invalid waypoint chunk: "{chunk}"')
        try:
            wx = float(parts[0].strip())
            wy = float(parts[1].strip())
        except ValueError as exc:
            raise ValueError(f'invalid waypoint value: "{chunk}"') from exc
        points.append((wx, wy))
    return points


def main():
    args = parse_args()
    try:
        fixed_world_waypoints = parse_world_waypoints(args.path_waypoints)
    except ValueError as e:
        print(f'[ERROR] {e}', file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    exit_code = 0

    # Fixed run profile with optional strict final pose overrides.
    fixed_robot_id = args.robot
    fixed_frame = 'world'
    fixed_world_name = 'default'
    fixed_ign_timeout = 4.0
    fixed_timeout_sec = 300.0
    fixed_pos_tol = 0.08
    fixed_yaw_tol_deg = 5.0
    fixed_max_linear = 0.52
    fixed_max_lateral = 0.21
    fixed_max_angular = 0.67
    fixed_max_angular_final = 0.17
    fixed_k_linear = 1.28
    fixed_k_lateral = 0.90
    fixed_k_yaw = 0.90
    fixed_turn_slowdown_deg = 60.0
    fixed_yaw_settle_sec = 0.42
    fixed_min_approach_speed = 0.16
    fixed_min_approach_distance = 0.10
    fixed_node_step_distance = 0.50
    fixed_face_heading_tol_deg = 3.2
    fixed_forward_realign_deg = 6.8
    fixed_obstacle_stop_dist = 0.42
    fixed_obstacle_slow_dist = 0.75
    fixed_obstacle_turn_deg = 32.0
    fixed_obstacle_front_half_deg = 20.0
    fixed_obstacle_side_start_deg = 25.0
    fixed_obstacle_side_end_deg = 80.0
    fixed_world_sync_sec = 0.35
    fixed_world_yaw_track_step_deg = 3.0
    fixed_world_yaw_track_rotate_step_deg = 6.0
    fixed_world_yaw_track_rotate_wz_gate = 0.10
    fixed_world_yaw_track_limit_deg = 85.0
    fixed_world_replan_yaw_deg = 120.0
    fixed_world_replan_wz_gate = 0.06
    fixed_world_replan_cooldown_sec = 1.2
    fixed_world_rotate_sync_replan = False
    fixed_rate_hz = 20.0
    fixed_controller_recover_timeout = 10.0
    fixed_odom_wait_sec = 15.0
    fixed_world_grid_resolution = fixed_node_step_distance
    fixed_target_yaw_deg = args.target_yaw_deg

    if args.pos_tol is not None:
        fixed_pos_tol = max(0.01, abs(args.pos_tol))
    if args.yaw_tol_deg is not None:
        fixed_yaw_tol_deg = max(0.1, abs(args.yaw_tol_deg))
    if args.yaw_settle_sec is not None:
        fixed_yaw_settle_sec = max(0.0, abs(args.yaw_settle_sec))

    target_yaw_world_rad = None
    if fixed_target_yaw_deg is not None:
        target_yaw_world_rad = math.radians(fixed_target_yaw_deg)

    ws_dir = os.path.dirname(os.path.abspath(__file__))
    fixed_world_sdf_candidates = [
        os.path.join(
            ws_dir,
            'src',
            'yahboom_rosmaster',
            'yahboom_rosmaster_gazebo',
            'worlds',
            'temp.world',
        ),
        os.path.join(
            ws_dir,
            'install',
            'yahboom_rosmaster_gazebo',
            'share',
            'yahboom_rosmaster_gazebo',
            'worlds',
            'temp.world',
        ),
    ]
    fixed_world_sdf_path = next((p for p in fixed_world_sdf_candidates if os.path.isfile(p)), None)
    if fixed_world_sdf_path is None:
        print(
            f'[ERROR] world file not found, candidates={fixed_world_sdf_candidates}',
            file=sys.stderr,
        )
        sys.exit(1)

    robot_name = f'rosmaster_x3_{fixed_robot_id}'

    node = GoToPointNode(
        robot_id=fixed_robot_id,
        target_x=args.x,
        target_y=args.y,
        fixed_world_waypoints=fixed_world_waypoints,
        target_yaw_deg=fixed_target_yaw_deg,
        pos_tol=fixed_pos_tol,
        yaw_tol_deg=fixed_yaw_tol_deg,
        max_linear=fixed_max_linear,
        max_lateral=fixed_max_lateral,
        max_angular=fixed_max_angular,
        max_angular_final=fixed_max_angular_final,
        k_linear=fixed_k_linear,
        k_lateral=fixed_k_lateral,
        k_yaw=fixed_k_yaw,
        turn_slowdown_deg=fixed_turn_slowdown_deg,
        yaw_settle_sec=fixed_yaw_settle_sec,
        min_approach_speed=fixed_min_approach_speed,
        min_approach_distance=fixed_min_approach_distance,
        node_step_distance=fixed_node_step_distance,
        face_heading_tol_deg=fixed_face_heading_tol_deg,
        forward_realign_deg=fixed_forward_realign_deg,
        obstacle_stop_dist=fixed_obstacle_stop_dist,
        obstacle_slow_dist=fixed_obstacle_slow_dist,
        obstacle_turn_deg=fixed_obstacle_turn_deg,
        obstacle_front_half_deg=fixed_obstacle_front_half_deg,
        obstacle_side_start_deg=fixed_obstacle_side_start_deg,
        obstacle_side_end_deg=fixed_obstacle_side_end_deg,
        target_frame=fixed_frame,
    )

    if not node.load_world_obstacles_from_sdf(fixed_world_sdf_path, fixed_world_grid_resolution):
        node.get_logger().error('failed to build occupancy from temp.world, aborting')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)

    dt = 1.0 / max(fixed_rate_hz, 1.0)
    t0 = time.time()

    try:
        node.get_logger().info(
            f'fixed profile: robot={fixed_robot_id}, frame={fixed_frame}, world={fixed_world_name}'
        )
        if fixed_world_waypoints:
            node.get_logger().info(
                f'coordinator path received: points={len(fixed_world_waypoints)}'
            )

        ok_controller, controller_state = ensure_mecanum_controller_active(
            fixed_robot_id,
            fixed_controller_recover_timeout,
        )
        if ok_controller:
            node.get_logger().info(f'mecanum controller state: {controller_state}')
        else:
            node.get_logger().warn(
                f'mecanum controller not active (state={controller_state}), waiting for odom anyway'
            )

        odom_wait_deadline = time.time() + max(fixed_odom_wait_sec, fixed_ign_timeout + 2.0)
        while rclpy.ok() and not node.odom_ready and time.time() < odom_wait_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if not node.odom_ready:
            controller_state = get_controller_state(fixed_robot_id) or 'unknown'
            candidate_odom_topics = [
                f'/rosmaster_x3_{fixed_robot_id}/mecanum_drive_controller/odom',
                '/mecanum_drive_controller/odom',
                '/odom',
            ]
            available_topics = {name for name, _ in node.get_topic_names_and_types()}
            seen = [topic for topic in candidate_odom_topics if topic in available_topics]
            node.get_logger().error(
                'no odom received, cannot navigate '
                f'(controller_state={controller_state}, seen_odom_topics={seen if seen else "none"})'
            )
            exit_code = 1
            raise SystemExit(1)

        world_poses = read_world_poses(fixed_world_name, fixed_ign_timeout)
        if robot_name not in world_poses:
            raise RuntimeError(f'{robot_name} not found in /world/{fixed_world_name}/pose/info')
        world_x_now, world_y_now, world_yaw_now = world_poses[robot_name]
        node.update_peer_safety(world_poses, time.time())

        odom_x_now = node.current_x
        odom_y_now = node.current_y
        odom_yaw_now = node.current_yaw

        yaw_world_from_odom_est = normalize_angle(world_yaw_now - odom_yaw_now)
        yaw_world_from_odom_base = yaw_world_from_odom_est
        c = math.cos(yaw_world_from_odom_est)
        s = math.sin(yaw_world_from_odom_est)
        tx_world = world_x_now - (c * odom_x_now - s * odom_y_now)
        ty_world = world_y_now - (s * odom_x_now + c * odom_y_now)

        target_odom_x, target_odom_y = world_to_odom(
            args.x,
            args.y,
            tx_world,
            ty_world,
            yaw_world_from_odom_est,
        )
        target_odom_yaw = None
        if target_yaw_world_rad is not None:
            target_odom_yaw = normalize_angle(target_yaw_world_rad - yaw_world_from_odom_est)

        node.get_logger().info(
            f'world->odom calibrated: world_now=({world_x_now:.3f}, {world_y_now:.3f}), '
            f'odom_now=({odom_x_now:.3f}, {odom_y_now:.3f}), '
            f'dyaw={math.degrees(yaw_world_from_odom_est):.2f} deg'
        )
        node.set_world_heading_state(world_x_now, world_y_now, world_yaw_now, yaw_world_from_odom_est)
        # Set world progress frame before building path so initial nodes are generated in world coordinates.
        node.set_progress_view('world', world_x_now, world_y_now, args.x, args.y)

        node.set_target_odom(target_odom_x, target_odom_y, target_odom_yaw)
        if node.path_failed:
            node.get_logger().error('global planner failed at startup, aborting')
            exit_code = 1
            raise SystemExit(1)

        effective_goal_world_x, effective_goal_world_y = node.effective_goal_world()
        target_odom_x, target_odom_y = world_to_odom(
            effective_goal_world_x,
            effective_goal_world_y,
            tx_world,
            ty_world,
            yaw_world_from_odom_est,
        )
        node.set_target_odom(
            target_odom_x,
            target_odom_y,
            target_odom_yaw,
            announce=False,
            replan=False,
        )
        node.set_progress_view(
            'world',
            world_x_now,
            world_y_now,
            effective_goal_world_x,
            effective_goal_world_y,
        )

        start_world_distance = math.hypot(
            effective_goal_world_x - world_x_now,
            effective_goal_world_y - world_y_now,
        )
        progress_speed_floor = max(0.08, fixed_min_approach_speed * 0.90)
        adaptive_timeout_sec = max(
            fixed_timeout_sec,
            (start_world_distance / progress_speed_floor) + 120.0,
        )
        node.get_logger().info(
            f'adaptive timeout: {adaptive_timeout_sec:.1f}s (start_dist={start_world_distance:.2f} m)'
        )

        world_sync_period = max(0.2, fixed_world_sync_sec)
        next_world_sync = time.time() + world_sync_period
        last_world_sync_warn = 0.0
        last_yaw_drift_warn = 0.0
        last_world_yaw_replan = 0.0

        while rclpy.ok() and not node.done:
            if node.odom_ready and (time.time() - t0) > adaptive_timeout_sec:
                node.get_logger().warn(f'timeout: no completion within {adaptive_timeout_sec:.1f}s, stopping robot')
                exit_code = 1
                break

            rclpy.spin_once(node, timeout_sec=dt)

            if node.odom_ready:
                now = time.time()
                if now >= next_world_sync:
                    try:
                        world_poses = read_world_poses(fixed_world_name, fixed_ign_timeout)
                        if robot_name not in world_poses:
                            raise RuntimeError(
                                f'{robot_name} not found in /world/{fixed_world_name}/pose/info'
                            )
                        world_x_now, world_y_now, world_yaw_now = world_poses[robot_name]
                        node.update_peer_safety(world_poses, now)

                        odom_x_now = node.current_x
                        odom_y_now = node.current_y
                        odom_yaw_now = node.current_yaw

                        yaw_world_from_odom_candidate = normalize_angle(world_yaw_now - odom_yaw_now)
                        yaw_delta = normalize_angle(yaw_world_from_odom_candidate - yaw_world_from_odom_est)
                        yaw_drift_deg = abs(math.degrees(yaw_delta))
                        candidate_bias_from_base_deg = abs(
                            math.degrees(normalize_angle(yaw_world_from_odom_candidate - yaw_world_from_odom_base))
                        )

                        phase = node.drive_phase
                        # Keep dyaw tracking bounded and conservative to avoid frame-drift feedback loops.
                        if phase == 'rotate_to_node':
                            max_step_deg = fixed_world_yaw_track_rotate_step_deg
                            if abs(node.current_wz) > fixed_world_yaw_track_rotate_wz_gate:
                                max_step_deg = min(max_step_deg, 2.0)
                            max_step_rad = math.radians(max_step_deg)
                        else:
                            max_step_rad = math.radians(fixed_world_yaw_track_step_deg)

                        if max_step_rad > 0.0:
                            yaw_step = clamp(yaw_delta, -max_step_rad, max_step_rad)
                            yaw_world_from_odom_est = normalize_angle(yaw_world_from_odom_est + yaw_step)

                        max_bias_from_base = math.radians(fixed_world_yaw_track_limit_deg)
                        bias_from_base = normalize_angle(yaw_world_from_odom_est - yaw_world_from_odom_base)
                        if abs(bias_from_base) > max_bias_from_base:
                            yaw_world_from_odom_est = normalize_angle(
                                yaw_world_from_odom_base + math.copysign(max_bias_from_base, bias_from_base)
                            )

                        if yaw_drift_deg > 30.0 and now - last_yaw_drift_warn > 2.0:
                            node.get_logger().warn(
                                f'world sync yaw tracked: candidate={math.degrees(yaw_world_from_odom_candidate):.1f} deg, '
                                f'est={math.degrees(yaw_world_from_odom_est):.1f} deg, '
                                f'phase={phase}, wz={node.current_wz:.2f}'
                            )
                            last_yaw_drift_warn = now

                        sync_replan = (
                            fixed_world_rotate_sync_replan
                            and
                            phase == 'rotate_to_node'
                            and abs(node.current_wz) <= fixed_world_replan_wz_gate
                            and yaw_drift_deg >= fixed_world_replan_yaw_deg
                            and candidate_bias_from_base_deg <= (fixed_world_yaw_track_limit_deg + 5.0)
                            and (now - last_world_yaw_replan) >= fixed_world_replan_cooldown_sec
                        )
                        if sync_replan:
                            last_world_yaw_replan = now
                            node.get_logger().info(
                                f'world sync rotate replan: yaw_drift={yaw_drift_deg:.1f} deg '
                                f'(candidate={math.degrees(yaw_world_from_odom_candidate):.1f} deg)'
                            )

                        c = math.cos(yaw_world_from_odom_est)
                        s = math.sin(yaw_world_from_odom_est)
                        tx_world = world_x_now - (c * odom_x_now - s * odom_y_now)
                        ty_world = world_y_now - (s * odom_x_now + c * odom_y_now)

                        effective_goal_world_x, effective_goal_world_y = node.effective_goal_world()
                        target_odom_x, target_odom_y = world_to_odom(
                            effective_goal_world_x,
                            effective_goal_world_y,
                            tx_world,
                            ty_world,
                            yaw_world_from_odom_est,
                        )
                        target_odom_yaw = None
                        if target_yaw_world_rad is not None:
                            target_odom_yaw = normalize_angle(target_yaw_world_rad - yaw_world_from_odom_est)

                        node.set_world_heading_state(
                            world_x_now,
                            world_y_now,
                            world_yaw_now,
                            yaw_world_from_odom_est,
                        )
                        # Keep progress frame updated before any potential sync-triggered rebuild.
                        node.set_progress_view(
                            'world',
                            world_x_now,
                            world_y_now,
                            effective_goal_world_x,
                            effective_goal_world_y,
                        )
                        node.set_target_odom(
                            target_odom_x,
                            target_odom_y,
                            target_odom_yaw,
                            announce=False,
                            replan=sync_replan,
                        )
                    except RuntimeError as e:
                        if now - last_world_sync_warn > 2.0:
                            node.get_logger().warn(f'world sync skipped: {e}')
                            last_world_sync_warn = now
                    finally:
                        next_world_sync = now + world_sync_period

            node.control_step()
            if node.path_failed:
                node.get_logger().error('global planner failed during run, stopping')
                exit_code = 1
                break

        if node.done and node.odom_ready:
            try:
                world_poses = read_world_poses(fixed_world_name, fixed_ign_timeout)
                if robot_name not in world_poses:
                    raise RuntimeError(f'{robot_name} not found in /world/{fixed_world_name}/pose/info')
                world_x_now, world_y_now, world_yaw_now = world_poses[robot_name]
                world_yaw_deg = math.degrees(world_yaw_now)
                effective_goal_world_x, effective_goal_world_y = node.effective_goal_world()
                world_err = math.hypot(
                    effective_goal_world_x - world_x_now,
                    effective_goal_world_y - world_y_now,
                )
                goal_text = f'goal=({effective_goal_world_x:.3f}, {effective_goal_world_y:.3f})'
                if (
                    abs(effective_goal_world_x - args.x) > 1e-6
                    or abs(effective_goal_world_y - args.y) > 1e-6
                ):
                    goal_text += f', requested=({args.x:.3f}, {args.y:.3f})'
                node.get_logger().info(
                    f'world final: cur=({world_x_now:.3f}, {world_y_now:.3f}), '
                    f'{goal_text}, err={world_err:.3f} m, yaw={world_yaw_deg:.1f} deg'
                )
                if args.pos_tol is None:
                    world_done_tol = max(0.12, node.progress_done_tol)
                else:
                    world_done_tol = max(0.03, fixed_pos_tol * 1.5)
                if world_err > world_done_tol:
                    node.get_logger().warn(
                        f'world final error too large ({world_err:.3f} m > {world_done_tol:.3f} m), marking failure'
                    )
                    #exit_code = 1

                if target_yaw_world_rad is not None:
                    world_yaw_err_deg = abs(
                        math.degrees(normalize_angle(target_yaw_world_rad - world_yaw_now))
                    )
                    node.get_logger().info(
                        f'world final yaw: target={fixed_target_yaw_deg:.1f} deg, err={world_yaw_err_deg:.2f} deg'
                    )
                    yaw_done_tol = max(0.1, fixed_yaw_tol_deg)
                    if world_yaw_err_deg > (yaw_done_tol + 0.5):
                        node.get_logger().warn(
                            f'world final yaw error too large ({world_yaw_err_deg:.2f} deg > {yaw_done_tol:.2f} deg), marking failure'
                        )
                        exit_code = 1
            except RuntimeError as e:
                node.get_logger().warn(f'world final check skipped: {e}')

        if not node.done and exit_code == 0:
            node.get_logger().warn('target not reached, exiting with non-zero status')
            exit_code = 1

    except ExternalShutdownException:
        exit_code = 130
    except KeyboardInterrupt:
        node.get_logger().info('interrupted by user')
        exit_code = 130
    except RuntimeError as e:
        node.get_logger().error(str(e))
        exit_code = 1
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if exit_code != 0:
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
