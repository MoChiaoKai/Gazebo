#!/usr/bin/env python3
import argparse
from dataclasses import dataclass
import itertools
import math
import os
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from astar_planner import AStarGridPlanner


WORLD_MIN = -2.5
WORLD_MAX = 2.5
DEFAULT_RESOLUTION = 0.5


@dataclass
class ExecState:
    proc: subprocess.Popen[str]
    robot_id: int
    cmd: list[str]
    start_time: float
    paused: bool = False
    paused_since: float | None = None
    paused_total: float = 0.0
    last_stop_sent: float = 0.0


@dataclass
class PauseLock:
    higher_robot: int
    # Timestamp of the last significant distance change for this paused pair.
    since: float
    # Last observed pair distance used for deadlock stability detection.
    ref_dist: float
    last_deadlock_break: float = 0.0
    last_guard_warn: float = 0.0
    last_escape_cmd: float = 0.0
    last_blocker_escape_cmd: float = 0.0


@dataclass
class PairMetric:
    dist: float
    closing_speed: float


def normalize_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parse_scalar(block: str, key: str, default: float = 0.0) -> float:
    m = re.search(rf"\b{re.escape(key)}:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", block)
    return float(m.group(1)) if m else default


def find_world_file(cli_path: str | None) -> str:
    if cli_path:
        if not os.path.isfile(cli_path):
            raise RuntimeError(f"world file does not exist: {cli_path}")
        return cli_path

    ws_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(
            ws_dir,
            "src",
            "yahboom_rosmaster",
            "yahboom_rosmaster_gazebo",
            "worlds",
            "temp.world",
        ),
        os.path.join(
            ws_dir,
            "install",
            "yahboom_rosmaster_gazebo",
            "share",
            "yahboom_rosmaster_gazebo",
            "worlds",
            "temp.world",
        ),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path

    raise RuntimeError(f"temp.world not found, checked: {candidates}")


def find_go_to_point_script(cli_path: str | None) -> str:
    if cli_path:
        if not os.path.isfile(cli_path):
            raise RuntimeError(f"go_to_point script does not exist: {cli_path}")
        return cli_path

    ws_dir = os.path.dirname(os.path.abspath(__file__))
    default_path = os.path.join(ws_dir, "go_to_point.py")
    if os.path.isfile(default_path):
        return default_path

    raise RuntimeError(f"go_to_point.py not found, checked: {default_path}")


def world_to_cell(world_x: float, world_y: float, resolution: float) -> tuple[int, int] | None:
    axis_span = WORLD_MAX - WORLD_MIN
    node_count = int(round(axis_span / resolution)) + 1
    width = node_count
    height = node_count
    map_min_x = WORLD_MIN - 0.5 * resolution
    map_min_y = WORLD_MIN - 0.5 * resolution

    mx = int(math.floor((world_x - map_min_x) / resolution))
    my = int(math.floor((world_y - map_min_y) / resolution))
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return None
    return mx, my


def cell_to_world(mx: int, my: int, resolution: float) -> tuple[float, float]:
    map_min_x = WORLD_MIN - 0.5 * resolution
    map_min_y = WORLD_MIN - 0.5 * resolution
    return map_min_x + (mx + 0.5) * resolution, map_min_y + (my + 0.5) * resolution


def clamp_world_point(x: float, y: float) -> tuple[float, float]:
    eps = 1e-6
    cx = min(WORLD_MAX - eps, max(WORLD_MIN + eps, x))
    cy = min(WORLD_MAX - eps, max(WORLD_MIN + eps, y))
    return cx, cy


def load_world_blocked_nodes(world_path: str, resolution: float) -> tuple[int, int, list[bool]]:
    tree = ET.parse(world_path)
    root = tree.getroot()
    world_elem = next((elem for elem in root.iter() if elem.tag.split("}", 1)[-1] == "world"), None)
    if world_elem is None:
        raise RuntimeError(f"no <world> in {world_path}")

    axis_span = WORLD_MAX - WORLD_MIN
    node_count = int(round(axis_span / resolution)) + 1
    if node_count <= 1:
        raise RuntimeError(f"invalid resolution {resolution}")

    width = node_count
    height = node_count
    blocked = [False] * (width * height)

    for model in world_elem:
        if model.tag.split("}", 1)[-1] != "model":
            continue
        model_name = model.attrib.get("name", "")
        if not model_name.startswith("Obstacle"):
            continue

        static_text = ""
        pose_text = ""
        for child in model:
            tag = child.tag.split("}", 1)[-1]
            if tag == "static" and child.text:
                static_text = child.text.strip().lower()
            if tag == "pose" and child.text:
                pose_text = child.text.strip()

        if static_text not in ("1", "true", "yes"):
            continue

        vals = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", pose_text)
        if len(vals) < 2:
            continue

        x = float(vals[0])
        y = float(vals[1])

        snapped_x = round((x - WORLD_MIN) / resolution) * resolution + WORLD_MIN
        snapped_y = round((y - WORLD_MIN) / resolution) * resolution + WORLD_MIN
        cell = world_to_cell(snapped_x, snapped_y, resolution)
        if cell is None:
            continue

        mx, my = cell
        blocked[my * width + mx] = True

    return width, height, blocked


def read_robot_world_poses(world_name: str, robot_names: list[str], timeout_sec: float) -> dict[str, tuple[float, float, float]]:
    topic = f"/world/{world_name}/pose/info"
    try:
        output = subprocess.check_output(
            ["ign", "topic", "-e", "-t", topic, "-n", "1"],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=max(timeout_sec, 0.5),
        )
    except Exception as e:
        raise RuntimeError(f"failed to read {topic}: {e}") from e

    found: dict[str, tuple[float, float, float]] = {}
    lines = output.splitlines()
    i = 0
    while i < len(lines):
        if lines[i].strip() != "pose {":
            i += 1
            continue

        depth = 1
        i += 1
        block_lines = []
        while i < len(lines) and depth > 0:
            line = lines[i]
            depth += line.count("{")
            depth -= line.count("}")
            block_lines.append(line)
            i += 1

        block = "\n".join(block_lines)
        m_name = re.search(r'name:\s*"([^"]+)"', block)
        if not m_name:
            continue

        name = m_name.group(1)
        if name not in robot_names:
            continue

        m_pos = re.search(r"position\s*\{([^}]*)\}", block, re.S)
        m_ori = re.search(r"orientation\s*\{([^}]*)\}", block, re.S)
        pos_block = m_pos.group(1) if m_pos else ""
        ori_block = m_ori.group(1) if m_ori else ""

        px = parse_scalar(pos_block, "x", 0.0)
        py = parse_scalar(pos_block, "y", 0.0)
        qx = parse_scalar(ori_block, "x", 0.0)
        qy = parse_scalar(ori_block, "y", 0.0)
        qz = parse_scalar(ori_block, "z", 0.0)
        qw = parse_scalar(ori_block, "w", 1.0)
        yaw = yaw_from_quaternion(qx, qy, qz, qw)

        found[name] = (px, py, yaw)

    missing = [name for name in robot_names if name not in found]
    if missing:
        raise RuntimeError(f"missing robot poses in world topic: {missing}")

    return found


def plan_paths_allow_overlap(
    planner: AStarGridPlanner,
    requests: list[tuple[tuple[int, int], tuple[int, int]]],
    search_radius_cells: int,
) -> tuple[list[list[tuple[int, int]]] | None, list[int] | None]:
    paths: list[list[tuple[int, int]]] = []
    for start_cell, goal_cell in requests:
        start_free = planner.find_nearest_free_cell(start_cell[0], start_cell[1], search_radius_cells)
        goal_free = planner.find_nearest_free_cell(goal_cell[0], goal_cell[1], search_radius_cells)
        if start_free is None or goal_free is None:
            return None, None

        path = planner.plan(start_free, goal_free)
        if not path:
            return None, None
        paths.append(path)

    return paths, [0, 1, 2]


def find_overlap_cells(paths: list[list[tuple[int, int]]]) -> set[tuple[int, int]]:
    used: set[tuple[int, int]] = set()
    overlaps: set[tuple[int, int]] = set()
    for path in paths:
        for cell in path:
            if cell in used:
                overlaps.add(cell)
            else:
                used.add(cell)
    return overlaps


def inflate_cells(
    cells: list[tuple[int, int]],
    clearance_cells: int,
    width: int,
    height: int,
) -> set[tuple[int, int]]:
    if clearance_cells <= 0:
        return set(cells)

    inflated: set[tuple[int, int]] = set()
    radius = int(clearance_cells)
    for mx, my in cells:
        for dx in range(-radius, radius + 1):
            nx = mx + dx
            if nx < 0 or nx >= width:
                continue
            for dy in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) > radius:
                    continue
                ny = my + dy
                if ny < 0 or ny >= height:
                    continue
                inflated.add((nx, ny))
    return inflated


def plan_disjoint_paths_with_clearance(
    planner: AStarGridPlanner,
    requests: list[tuple[tuple[int, int], tuple[int, int]]],
    search_radius_cells: int,
    clearance_cells: int,
) -> tuple[list[list[tuple[int, int]]] | None, list[int] | None]:
    if clearance_cells <= 0:
        return planner.plan_disjoint_paths(requests, search_radius_cells)

    req_count = len(requests)
    req_indices = list(range(req_count))
    best_paths: list[list[tuple[int, int]]] | None = None
    best_order: list[int] | None = None
    best_score: tuple[int, int] | None = None

    for order in itertools.permutations(req_indices):
        planned_paths: list[list[tuple[int, int]] | None] = [None] * req_count

        def dfs(depth: int, reserved: set[tuple[int, int]], total_turns: int, total_steps: int):
            nonlocal best_paths, best_order, best_score

            if best_score is not None:
                if total_turns > best_score[0]:
                    return
                if total_turns == best_score[0] and total_steps >= best_score[1]:
                    return

            if depth >= req_count:
                score = (total_turns, total_steps)
                if best_score is None or score < best_score:
                    picked: list[list[tuple[int, int]]] = []
                    for idx in req_indices:
                        path = planned_paths[idx]
                        if path is None:
                            return
                        picked.append(path)
                    best_score = score
                    best_order = list(order)
                    best_paths = picked
                return

            req_idx = order[depth]
            start_cell, goal_cell = requests[req_idx]

            candidates = planner._candidate_paths_for_request(
                start_cell,
                goal_cell,
                reserved,
                search_radius_cells,
            )
            if not candidates:
                return

            for path in candidates:
                if any(cell in reserved for cell in path):
                    continue

                planned_paths[req_idx] = path
                turns = AStarGridPlanner.path_turn_count(path)
                steps = max(0, len(path) - 1)

                next_reserved = set(reserved)
                next_reserved.update(
                    inflate_cells(path, clearance_cells, planner.width, planner.height)
                )
                dfs(depth + 1, next_reserved, total_turns + turns, total_steps + steps)
                planned_paths[req_idx] = None

        dfs(0, set(), 0, 0)

    return best_paths, best_order


def find_path_conflict_pairs(
    paths: list[list[tuple[int, int]]],
    clearance_cells: int,
) -> set[tuple[int, int]]:
    pairs: set[tuple[int, int]] = set()
    min_clearance = max(0, clearance_cells)
    for ia in range(len(paths)):
        for ib in range(ia + 1, len(paths)):
            conflict = False
            for ax, ay in paths[ia]:
                if conflict:
                    break
                for bx, by in paths[ib]:
                    if max(abs(ax - bx), abs(ay - by)) <= min_clearance:
                        conflict = True
                        break
            if conflict:
                pairs.add((ia, ib))
    return pairs


def first_conflict_index(
    path_a: list[tuple[int, int]],
    path_b: list[tuple[int, int]],
    clearance_cells: int,
) -> int:
    threshold = max(0, clearance_cells)
    for idx_a, (ax, ay) in enumerate(path_a):
        for bx, by in path_b:
            if max(abs(ax - bx), abs(ay - by)) <= threshold:
                return idx_a
    return len(path_a)


def last_conflict_index(
    path_a: list[tuple[int, int]],
    path_b: list[tuple[int, int]],
    clearance_cells: int,
) -> int:
    threshold = max(0, clearance_cells)
    last_idx = -1
    for idx_a, (ax, ay) in enumerate(path_a):
        for bx, by in path_b:
            if max(abs(ax - bx), abs(ay - by)) <= threshold:
                last_idx = idx_a
                break
    return last_idx


def choose_conflict_leader(
    robot_a: int,
    robot_b: int,
    path_a: list[tuple[int, int]],
    path_b: list[tuple[int, int]],
    clearance_cells: int,
) -> int:
    def min_cell_path_clearance(
        cell: tuple[int, int],
        path: list[tuple[int, int]],
    ) -> int:
        if not path:
            return 10**6
        cx, cy = cell
        best = 10**6
        for px, py in path:
            d = max(abs(cx - px), abs(cy - py))
            if d < best:
                best = d
                if best == 0:
                    break
        return best

    a_entry = first_conflict_index(path_a, path_b, clearance_cells)
    b_entry = first_conflict_index(path_b, path_a, clearance_cells)
    a_exit = last_conflict_index(path_a, path_b, clearance_cells)
    b_exit = last_conflict_index(path_b, path_a, clearance_cells)

    # If one robot starts ahead on the other robot's route, let the blocking
    # front robot clear first to avoid leader-behind deadlocks.
    a_hits_b_start = first_conflict_index(path_a, [path_b[0]], clearance_cells)
    b_hits_a_start = first_conflict_index(path_b, [path_a[0]], clearance_cells)
    a_blocked_by_b_ahead = 0 < a_hits_b_start < len(path_a)
    b_blocked_by_a_ahead = 0 < b_hits_a_start < len(path_b)
    if a_blocked_by_b_ahead and (not b_blocked_by_a_ahead):
        return robot_b
    if b_blocked_by_a_ahead and (not a_blocked_by_b_ahead):
        return robot_a

    # Avoid selecting a leader that will stop at a goal cell located on the
    # other robot's path corridor; this can trap the follower indefinitely.
    threshold = max(0, clearance_cells)
    a_goal_on_b_path = min_cell_path_clearance(path_a[-1], path_b) <= threshold
    b_goal_on_a_path = min_cell_path_clearance(path_b[-1], path_a) <= threshold
    if a_goal_on_b_path and (not b_goal_on_a_path):
        return robot_b
    if b_goal_on_a_path and (not a_goal_on_b_path):
        return robot_a

    # Prefer the robot that can clear the shared/close-conflict region sooner.
    if a_exit >= 0 and b_exit >= 0 and a_exit != b_exit:
        return robot_a if a_exit < b_exit else robot_b

    if a_entry < b_entry:
        return robot_a
    if b_entry < a_entry:
        return robot_b

    if len(path_a) != len(path_b):
        return robot_a if len(path_a) < len(path_b) else robot_b

    # If entry order ties, prefer the robot that is currently closer to the other
    # robot's goal area so it can vacate that region first.
    a_start = path_a[0]
    b_start = path_b[0]
    a_goal = path_a[-1]
    b_goal = path_b[-1]
    a_blocks_b_goal = max(abs(a_start[0] - b_goal[0]), abs(a_start[1] - b_goal[1]))
    b_blocks_a_goal = max(abs(b_start[0] - a_goal[0]), abs(b_start[1] - a_goal[1]))
    if a_blocks_b_goal < b_blocks_a_goal:
        return robot_a
    if b_blocks_a_goal < a_blocks_b_goal:
        return robot_b

    return min(robot_a, robot_b)


def build_startup_wait_rules(
    poses: dict[str, tuple[float, float, float]],
    active_robot_ids: list[int],
    start_yield_distance: float,
    preferred_wait_for: dict[int, set[int]] | None = None,
    path_steps_by_robot: dict[int, int] | None = None,
) -> dict[int, set[int]]:
    startup_wait_for: dict[int, set[int]] = {}
    ids = sorted(active_robot_ids)
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            ra = ids[i]
            rb = ids[j]
            dist = get_pose_distance(poses, ra, rb)
            if dist is None or dist >= start_yield_distance:
                continue

            leader = min(ra, rb)
            follower = max(ra, rb)
            preferred_decided = False

            if preferred_wait_for is not None:
                if rb in preferred_wait_for.get(ra, set()):
                    leader = rb
                    follower = ra
                    preferred_decided = True
                elif ra in preferred_wait_for.get(rb, set()):
                    leader = ra
                    follower = rb
                    preferred_decided = True

            if path_steps_by_robot is not None and (not preferred_decided):
                sa = max(0, path_steps_by_robot.get(ra, 0))
                sb = max(0, path_steps_by_robot.get(rb, 0))
                if sa == 0 and sb > 0:
                    leader = rb
                    follower = ra
                elif sb == 0 and sa > 0:
                    leader = ra
                    follower = rb
                elif sa > 0 and sb > 0 and sa != sb:
                    if sa < sb:
                        leader = ra
                        follower = rb
                    else:
                        leader = rb
                        follower = ra

            startup_wait_for.setdefault(follower, set()).add(leader)
    return startup_wait_for


def pair_has_wait_rule(wait_for: dict[int, set[int]], ra: int, rb: int) -> bool:
    return (rb in wait_for.get(ra, set())) or (ra in wait_for.get(rb, set()))


def wait_path_exists(wait_for: dict[int, set[int]], src: int, dst: int) -> bool:
    if src == dst:
        return True
    seen: set[int] = set()
    stack: list[int] = [src]
    while stack:
        cur = stack.pop()
        if cur == dst:
            return True
        if cur in seen:
            continue
        seen.add(cur)
        for nxt in wait_for.get(cur, set()):
            if nxt not in seen:
                stack.append(nxt)
    return False


def edge_creates_wait_cycle(wait_for: dict[int, set[int]], lower: int, higher: int) -> bool:
    if lower == higher:
        return True
    # Adding lower -> higher creates a cycle iff higher can already reach lower.
    return wait_path_exists(wait_for, higher, lower)


def active_elapsed_sec(state: ExecState, now: float) -> float:
    elapsed = now - state.start_time - state.paused_total
    if state.paused and state.paused_since is not None:
        elapsed -= (now - state.paused_since)
    return elapsed


def pause_state(state: ExecState, now: float) -> bool:
    if state.paused or state.proc.poll() is not None:
        return False
    try:
        state.proc.send_signal(signal.SIGSTOP)
    except Exception:
        return False
    state.paused = True
    state.paused_since = now
    publish_zero_cmd(state.robot_id)
    state.last_stop_sent = now
    return True


def resume_state(state: ExecState, now: float) -> bool:
    if (not state.paused) or state.proc.poll() is not None:
        return False
    try:
        state.proc.send_signal(signal.SIGCONT)
    except Exception:
        return False
    if state.paused_since is not None:
        state.paused_total += max(0.0, now - state.paused_since)
    state.paused_since = None
    state.paused = False
    state.last_stop_sent = now
    return True


def terminate_state(state: ExecState):
    if state.proc.poll() is not None:
        return
    if state.paused:
        try:
            state.proc.send_signal(signal.SIGCONT)
        except Exception:
            pass
    publish_zero_cmd(state.robot_id, timeout_sec=0.25)
    try:
        state.proc.terminate()
        state.proc.wait(timeout=0.8)
    except subprocess.TimeoutExpired:
        try:
            state.proc.kill()
        except Exception:
            pass
    except Exception:
        pass


def refresh_paused_stop(state: ExecState, now: float, refresh_sec: float):
    if (not state.paused) or state.proc.poll() is not None:
        return
    if (now - state.last_stop_sent) < refresh_sec:
        return
    if publish_zero_cmd(state.robot_id):
        state.last_stop_sent = now


def format_pose_line(poses: dict[str, tuple[float, float, float]], robot_ids: list[int]) -> str:
    parts: list[str] = []
    for robot_id in robot_ids:
        key = f"rosmaster_x3_{robot_id}"
        pose = poses.get(key)
        if pose is None:
            parts.append(f"r{robot_id}=(n/a)")
            continue
        parts.append(f"r{robot_id}=({pose[0]:.2f},{pose[1]:.2f})")
    return " ".join(parts)


def build_pair_metrics(
    poses: dict[str, tuple[float, float, float]],
    active_robot_ids: list[int],
    now: float,
    pair_history: dict[tuple[int, int], tuple[float, float]],
) -> dict[tuple[int, int], PairMetric]:
    metrics: dict[tuple[int, int], PairMetric] = {}
    active_pairs: set[tuple[int, int]] = set()

    ids = sorted(active_robot_ids)
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            ra = ids[i]
            rb = ids[j]
            pa = poses.get(f"rosmaster_x3_{ra}")
            pb = poses.get(f"rosmaster_x3_{rb}")
            if pa is None or pb is None:
                continue

            dist = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            key = (ra, rb)
            active_pairs.add(key)

            closing_speed = 0.0
            prev = pair_history.get(key)
            if prev is not None:
                prev_dist, prev_time = prev
                dt = now - prev_time
                if dt > 1e-3:
                    closing_speed = max(0.0, (prev_dist - dist) / dt)

            pair_history[key] = (dist, now)
            metrics[key] = PairMetric(dist=dist, closing_speed=closing_speed)

    for stale_key in list(pair_history.keys()):
        if stale_key not in active_pairs:
            pair_history.pop(stale_key, None)

    return metrics


def compute_pause_reasons(
    active_robot_ids: list[int],
    pair_metrics: dict[tuple[int, int], PairMetric],
    yield_distance: float,
    predict_time: float,
    predict_margin: float,
) -> dict[int, tuple[int, float]]:
    reasons: dict[int, tuple[int, float]] = {}
    ids = sorted(active_robot_ids)
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            ra = ids[i]
            rb = ids[j]
            metric = pair_metrics.get((ra, rb))
            if metric is None:
                continue

            # Keep concurrent motion when pairs are not actively closing.
            base_yield = yield_distance
            if metric.closing_speed <= 0.05:
                base_yield = max(0.25, yield_distance * 0.60)

            effective_yield = base_yield + min(
                predict_margin,
                metric.closing_speed * predict_time,
            )
            if metric.dist > effective_yield:
                continue

            higher = min(ra, rb)
            lower = max(ra, rb)
            prev = reasons.get(lower)
            if prev is None or higher < prev[0] or (higher == prev[0] and metric.dist < prev[1]):
                reasons[lower] = (higher, metric.dist)
    return reasons


def get_pose_distance(
    poses: dict[str, tuple[float, float, float]],
    robot_a: int,
    robot_b: int,
) -> float | None:
    pa = poses.get(f"rosmaster_x3_{robot_a}")
    pb = poses.get(f"rosmaster_x3_{robot_b}")
    if pa is None or pb is None:
        return None
    return math.hypot(pa[0] - pb[0], pa[1] - pb[1])


def publish_twist_cmd(
    robot_id: int,
    vx: float,
    vy: float,
    wz: float,
    timeout_sec: float = 0.6,
) -> bool:
    topic = f"/rosmaster_x3_{robot_id}/cmd_vel"
    msg = (
        "{linear: {x: "
        f"{vx:.3f}"
        ", y: "
        f"{vy:.3f}"
        ", z: 0.0}, angular: {x: 0.0, y: 0.0, z: "
        f"{wz:.3f}"
        "}}"
    )
    try:
        completed = subprocess.run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                topic,
                "geometry_msgs/msg/Twist",
                msg,
            ],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=max(0.2, timeout_sec),
            text=True,
        )
    except KeyboardInterrupt:
        return False
    except Exception:
        return False
    return completed.returncode == 0


def publish_zero_cmd(robot_id: int, timeout_sec: float = 0.6) -> bool:
    return publish_twist_cmd(robot_id, 0.0, 0.0, 0.0, timeout_sec=timeout_sec)


def publish_escape_cmd(
    poses: dict[str, tuple[float, float, float]],
    yielder_robot: int,
    blocker_robot: int,
    speed_mps: float,
    prefer_lateral: bool = False,
    timeout_sec: float = 0.35,
) -> tuple[bool, float, float]:
    y_key = f"rosmaster_x3_{yielder_robot}"
    b_key = f"rosmaster_x3_{blocker_robot}"
    y_pose = poses.get(y_key)
    b_pose = poses.get(b_key)
    if y_pose is None or b_pose is None:
        return False, 0.0, 0.0

    away_world_x = y_pose[0] - b_pose[0]
    away_world_y = y_pose[1] - b_pose[1]
    norm = math.hypot(away_world_x, away_world_y)
    if norm < 1e-4:
        return False, 0.0, 0.0

    away_ux = away_world_x / norm
    away_uy = away_world_y / norm

    if prefer_lateral:
        # Move out of the current lane first, while keeping a small separation component.
        side1_x = -away_uy
        side1_y = away_ux
        side2_x = -side1_x
        side2_y = -side1_y

        proj1_x = y_pose[0] + side1_x * 0.35
        proj2_x = y_pose[0] + side2_x * 0.35
        margin1 = min(proj1_x - WORLD_MIN, WORLD_MAX - proj1_x)
        margin2 = min(proj2_x - WORLD_MIN, WORLD_MAX - proj2_x)
        if margin1 >= margin2:
            side_x, side_y = side1_x, side1_y
        else:
            side_x, side_y = side2_x, side2_y

        mix_x = 0.85 * side_x + 0.15 * away_ux
        mix_y = 0.85 * side_y + 0.15 * away_uy
        mix_norm = math.hypot(mix_x, mix_y)
        if mix_norm > 1e-4:
            world_vx = speed_mps * (mix_x / mix_norm)
            world_vy = speed_mps * (mix_y / mix_norm)
        else:
            world_vx = speed_mps * away_ux
            world_vy = speed_mps * away_uy
    else:
        world_vx = speed_mps * away_ux
        world_vy = speed_mps * away_uy

    yaw = y_pose[2]
    body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
    body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy

    ok = publish_twist_cmd(
        yielder_robot,
        body_vx,
        body_vy,
        0.0,
        timeout_sec=timeout_sec,
    )
    return ok, body_vx, body_vy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plan and execute safe 3-robot routes from current poses to 3 goals."
    )
    parser.add_argument("--x1", type=float, required=True, help="robot1 goal x (world)")
    parser.add_argument("--y1", type=float, required=True, help="robot1 goal y (world)")
    parser.add_argument("--x2", type=float, required=True, help="robot2 goal x (world)")
    parser.add_argument("--y2", type=float, required=True, help="robot2 goal y (world)")
    parser.add_argument("--x3", type=float, required=True, help="robot3 goal x (world)")
    parser.add_argument("--y3", type=float, required=True, help="robot3 goal y (world)")

    parser.add_argument("--s1x", type=float, help="robot1 start x override (world)")
    parser.add_argument("--s1y", type=float, help="robot1 start y override (world)")
    parser.add_argument("--s2x", type=float, help="robot2 start x override (world)")
    parser.add_argument("--s2y", type=float, help="robot2 start y override (world)")
    parser.add_argument("--s3x", type=float, help="robot3 start x override (world)")
    parser.add_argument("--s3y", type=float, help="robot3 start y override (world)")

    parser.add_argument("--world-name", type=str, default="default", help="Gazebo world name")
    parser.add_argument("--world-file", type=str, default=None, help="path to temp.world")
    parser.add_argument("--ign-timeout", type=float, default=4.0, help="ign topic read timeout")
    parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION, help="grid resolution")
    parser.add_argument(
        "--search-radius-m",
        type=float,
        default=0.0,
        help="nearest-free relocation radius for blocked starts/goals (0 means strict exact goals)",
    )
    parser.add_argument(
        "--allow-path-overlap",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="allow path overlap in planning stage; runtime traffic control still prevents collision",
    )
    parser.add_argument(
        "--path-clearance-distance",
        type=float,
        default=0.60,
        help="preferred minimum path spacing (m) between robots in non-overlap planning",
    )
    parser.add_argument(
        "--prefer-non-overlap-when-allow-overlap",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="when --allow-path-overlap is set, try non-overlapping planning first and only fall back to overlap if needed",
    )
    parser.add_argument(
        "--enforce-overlap-wait",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="apply lower-priority waiting for planned overlap/close-conflict path pairs",
    )
    parser.add_argument(
        "--overlap-prep-time",
        type=float,
        default=0.30,
        help="seconds to pre-evacuate waiting robots laterally before releasing overlap traffic",
    )
    parser.add_argument(
        "--overlap-release-distance",
        type=float,
        default=1.35,
        help="release overlap wait when lower-higher distance is above this value (m), even if higher is still active",
    )
    parser.add_argument(
        "--show-world-path",
        action="store_true",
        help="print full world-coordinate path points for each robot",
    )
    parser.add_argument(
        "--execute",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="after planning, execute go_to_point.py using --exec-mode (default: enabled)",
    )
    parser.add_argument(
        "--go-to-script",
        type=str,
        default=None,
        help="path to go_to_point.py (default: sibling go_to_point.py)",
    )
    parser.add_argument(
        "--exec-timeout",
        type=float,
        default=300.0,
        help="per-robot timeout in seconds when --execute is enabled",
    )
    parser.add_argument(
        "--exec-mode",
        type=str,
        choices=["sequential", "parallel"],
        default="parallel",
        help="execution mode when --execute is enabled (default: parallel)",
    )
    parser.add_argument(
        "--yield-distance",
        type=float,
        default=1.30,
        help="distance threshold (m) to start yielding in parallel mode",
    )
    parser.add_argument(
        "--yield-release-distance",
        type=float,
        default=1.70,
        help="distance threshold (m) to release a yielded robot (should be > yield-distance)",
    )
    parser.add_argument(
        "--yield-stop-refresh",
        type=float,
        default=0.12,
        help="while yielding, publish zero cmd every N seconds to hold stop",
    )
    parser.add_argument(
        "--yield-predict-time",
        type=float,
        default=0.85,
        help="seconds of closing-speed lookahead used to trigger yielding earlier",
    )
    parser.add_argument(
        "--yield-predict-margin",
        type=float,
        default=0.45,
        help="maximum extra yield distance (m) added by predictive lookahead",
    )
    parser.add_argument(
        "--yield-deadlock-wait",
        type=float,
        default=3.0,
        help="if a yielded pair stays near the same distance for this long, trigger deadlock break",
    )
    parser.add_argument(
        "--yield-deadlock-dist-eps",
        type=float,
        default=0.06,
        help="distance change threshold (m) used to detect deadlock while yielding",
    )
    parser.add_argument(
        "--yield-deadlock-break",
        type=float,
        default=2.0,
        help="seconds to temporarily pause higher-priority robot during deadlock break",
    )
    parser.add_argument(
        "--pose-check-period",
        type=float,
        default=0.10,
        help="live pose sampling period in seconds for priority yielding",
    )
    parser.add_argument(
        "--pose-log-period",
        type=float,
        default=0.8,
        help="position print period in seconds during parallel execution",
    )
    parser.add_argument(
        "--show-live-poses",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="print other robots world positions during parallel execution",
    )
    parser.add_argument(
        "--priority-yield",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="enable fixed priority yielding (1 > 2 > 3) in parallel execution",
    )
    parser.add_argument(
        "--start-yield-distance",
        type=float,
        default=1.05,
        help="startup spacing threshold (m): if any pair starts closer, lower priority robot waits",
    )
    parser.add_argument(
        "--start-release-distance",
        type=float,
        default=1.35,
        help="startup wait release distance (m), should be > --start-yield-distance",
    )
    parser.add_argument(
        "--pose-failsafe-timeout",
        type=float,
        default=0.90,
        help="if no fresh world pose for this long, pause all robots until pose feed recovers",
    )
    parser.add_argument(
        "--yield-hard-guard-distance",
        type=float,
        default=0.72,
        help="if paused-lower pair distance falls below this, force close-hold (pause higher briefly, keep lower paused)",
    )
    parser.add_argument(
        "--yield-escape-speed",
        type=float,
        default=0.30,
        help="escape speed (m/s) used to actively move yielding robot away during close-hold",
    )
    parser.add_argument(
        "--yield-escape-period",
        type=float,
        default=0.12,
        help="minimum seconds between active escape commands during close-hold",
    )
    parser.add_argument(
        "--emergency-stop-distance",
        type=float,
        default=0.22,
        help="emergency-stop all robots if any pair distance <= this value (0 disables)",
    )
    parser.add_argument(
        "--emergency-brake-time",
        type=float,
        default=0.80,
        help="extra emergency distance = closing_speed * this time, capped internally",
    )
    parser.add_argument(
        "--emergency-predict-cap",
        type=float,
        default=0.18,
        help="maximum predictive extra distance (m) added to emergency-stop threshold",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    resolution = max(0.05, abs(args.resolution))
    search_radius_cells = max(0, int(math.ceil(max(0.0, args.search_radius_m) / resolution)))
    path_clearance_cells = max(
        0,
        int(math.floor(max(0.0, args.path_clearance_distance) / resolution + 1e-9)),
    )

    world_file = find_world_file(args.world_file)
    width, height, blocked = load_world_blocked_nodes(world_file, resolution)
    planner = AStarGridPlanner(width, height, blocked)

    robot_names = ["rosmaster_x3_1", "rosmaster_x3_2", "rosmaster_x3_3"]

    starts_override = [
        (args.s1x, args.s1y),
        (args.s2x, args.s2y),
        (args.s3x, args.s3y),
    ]

    if any(vx is not None or vy is not None for vx, vy in starts_override):
        starts_world: list[tuple[float, float]] = []
        for i, (vx, vy) in enumerate(starts_override, start=1):
            if vx is None or vy is None:
                print(
                    f"[ERROR] start override for robot{i} is incomplete, both x and y are required",
                    file=sys.stderr,
                )
                return 1
            starts_world.append((vx, vy))
    else:
        poses = read_robot_world_poses(args.world_name, robot_names, args.ign_timeout)
        starts_world = [
            (poses[robot_names[0]][0], poses[robot_names[0]][1]),
            (poses[robot_names[1]][0], poses[robot_names[1]][1]),
            (poses[robot_names[2]][0], poses[robot_names[2]][1]),
        ]

    goals_world = [
        (args.x1, args.y1),
        (args.x2, args.y2),
        (args.x3, args.y3),
    ]

    requests: list[tuple[tuple[int, int], tuple[int, int]]] = []
    for idx in range(3):
        start_x = starts_world[idx][0]
        start_y = starts_world[idx][1]
        s_cell = world_to_cell(start_x, start_y, resolution)
        if s_cell is None:
            clamp_x, clamp_y = clamp_world_point(start_x, start_y)
            s_cell = world_to_cell(clamp_x, clamp_y, resolution)
            if s_cell is not None:
                print(
                    f"[WARN] robot{idx + 1} start out of bounds, clamped "
                    f"({start_x:.2f}, {start_y:.2f}) -> ({clamp_x:.2f}, {clamp_y:.2f})"
                )

        g_cell = world_to_cell(goals_world[idx][0], goals_world[idx][1], resolution)
        if s_cell is None:
            print(f"[ERROR] robot{idx + 1} start out of bounds: {starts_world[idx]}", file=sys.stderr)
            return 1
        if g_cell is None:
            print(f"[ERROR] robot{idx + 1} goal out of bounds: {goals_world[idx]}", file=sys.stderr)
            return 1
        requests.append((s_cell, g_cell))

    planned_with_overlap = False
    planned_with_clearance = False
    paths: list[list[tuple[int, int]]] | None = None
    plan_order: list[int] | None = None

    if args.allow_path_overlap:
        if args.prefer_non_overlap_when_allow_overlap:
            if path_clearance_cells > 0:
                paths, plan_order = plan_disjoint_paths_with_clearance(
                    planner,
                    requests,
                    search_radius_cells,
                    path_clearance_cells,
                )
                planned_with_clearance = paths is not None and plan_order is not None
            if paths is None or plan_order is None:
                paths, plan_order = planner.plan_disjoint_paths(requests, search_radius_cells)
        if paths is None or plan_order is None:
            paths, plan_order = plan_paths_allow_overlap(planner, requests, search_radius_cells)
            planned_with_overlap = True
    else:
        if path_clearance_cells > 0:
            paths, plan_order = plan_disjoint_paths_with_clearance(
                planner,
                requests,
                search_radius_cells,
                path_clearance_cells,
            )
            planned_with_clearance = paths is not None and plan_order is not None
        if paths is None or plan_order is None:
            paths, plan_order = planner.plan_disjoint_paths(requests, search_radius_cells)

    if paths is None or plan_order is None:
        if args.allow_path_overlap:
            print("[FAIL] no feasible paths found for all 3 robots")
        else:
            print("[FAIL] no non-overlapping paths found for all 3 robots")
        return 2

    overlap_cells = find_overlap_cells(paths)
    if overlap_cells and not args.allow_path_overlap:
        first_cell = next(iter(overlap_cells))
        print(f"[FAIL] overlap detected unexpectedly at cell={first_cell}")
        return 3

    conflict_pairs = find_path_conflict_pairs(paths, max(1, path_clearance_cells))

    if overlap_cells:
        print(f"[OK] paths found with overlap, overlap_cells={len(overlap_cells)}")
    else:
        if args.allow_path_overlap and (not planned_with_overlap):
            if planned_with_clearance:
                print("[OK] non-overlapping paths found (preferred with safety clearance)")
            else:
                print("[OK] non-overlapping paths found (preferred)")
        else:
            print("[OK] non-overlapping paths found")

    if conflict_pairs:
        pair_text = ", ".join(
            f"r{ia + 1}-r{ib + 1}" for ia, ib in sorted(conflict_pairs)
        )
        print(
            f"[WARN] path close-conflicts={len(conflict_pairs)} within clearance {max(1, path_clearance_cells)} cells: {pair_text}"
        )
    else:
        print(f"[OK] planned path spacing clear (>= {max(1, path_clearance_cells)} cells)")

    print(f"planning_order={plan_order} (request indices 0-based)")

    planned_goal_worlds: list[tuple[float, float]] = [(0.0, 0.0)] * 3
    planned_world_paths: dict[int, list[tuple[float, float]]] = {}
    path_steps_by_robot: dict[int, int] = {}
    overlap_wait_for_plan: dict[int, set[int]] = {}
    strict_wait_for_plan: dict[int, set[int]] = {}
    if args.enforce_overlap_wait and conflict_pairs:
        for ia, ib in sorted(conflict_pairs):
            ra = ia + 1
            rb = ib + 1
            leader = choose_conflict_leader(
                ra,
                rb,
                paths[ia],
                paths[ib],
                max(1, path_clearance_cells),
            )
            follower = rb if leader == ra else ra
            overlap_wait_for_plan.setdefault(follower, set()).add(leader)
            strict_wait_for_plan.setdefault(follower, set()).add(leader)

    if overlap_wait_for_plan:
        rule_items: list[str] = []
        for low, highs in sorted(overlap_wait_for_plan.items()):
            for high in sorted(highs):
                rule_items.append(f"robot{low}->robot{high}")
        rules = ", ".join(rule_items)
        print(f"[YIELD] path wait rules: {rules}")

    for idx, path in enumerate(paths, start=1):
        turns = AStarGridPlanner.path_turn_count(path)
        steps = max(0, len(path) - 1)
        path_steps_by_robot[idx] = steps
        req_goal_world = goals_world[idx - 1]
        req_goal_cell = requests[idx - 1][1]
        goal_cell = path[-1]
        goal_world = cell_to_world(goal_cell[0], goal_cell[1], resolution)
        world_points = [cell_to_world(mx, my, resolution) for mx, my in path]
        planned_goal_worlds[idx - 1] = goal_world
        planned_world_paths[idx] = world_points
        print(
            f"robot{idx}: steps={steps}, turns={turns}, "
            f"start_cell={path[0]}, end_cell={goal_cell}, end_world=({goal_world[0]:.2f}, {goal_world[1]:.2f}), "
            f"requested_goal=({req_goal_world[0]:.2f}, {req_goal_world[1]:.2f})"
        )
        if goal_cell != req_goal_cell:
            print(
                f"  adjusted_goal: requested_cell={req_goal_cell} -> planned_cell={goal_cell}"
            )
        if args.show_world_path:
            print(f"  world_path={world_points}")

    if args.execute:
        try:
            go_to_script = find_go_to_point_script(args.go_to_script)
        except RuntimeError as e:
            print(f"[ERROR] {e}", file=sys.stderr)
            return 1

        exec_timeout = max(10.0, abs(args.exec_timeout))
        print(
            f"[EXEC] mode={args.exec_mode}, timeout={exec_timeout:.1f}s per robot, "
            f"script={go_to_script}"
        )

        jobs: list[tuple[int, list[str]]] = []
        for robot_id in [1, 2, 3]:
            req_idx = robot_id - 1
            gx, gy = planned_goal_worlds[req_idx]
            path_points = planned_world_paths.get(robot_id, [])
            cmd = [
                sys.executable,
                go_to_script,
                "--robot",
                str(robot_id),
                "--x",
                f"{gx:.3f}",
                "--y",
                f"{gy:.3f}",
            ]
            if path_points:
                path_arg = ";".join(f"{px:.3f},{py:.3f}" for px, py in path_points)
                cmd.append(f"--path-waypoints={path_arg}")
            jobs.append((robot_id, cmd))

        if args.exec_mode == "sequential":
            for robot_id, cmd in jobs:
                print(f"[EXEC] robot{robot_id}: cmd={' '.join(cmd)}")
                try:
                    completed = subprocess.run(cmd, check=False, timeout=exec_timeout)
                except subprocess.TimeoutExpired:
                    print(
                        f"[FAIL] robot{robot_id} execution timeout after {exec_timeout:.1f}s",
                        file=sys.stderr,
                    )
                    return 4

                if completed.returncode != 0:
                    if completed.returncode in (130, -signal.SIGINT):
                        print(
                            f"[WARN] robot{robot_id} interrupted (returncode={completed.returncode})"
                        )
                        return 130
                    print(
                        f"[FAIL] robot{robot_id} execution failed, returncode={completed.returncode}",
                        file=sys.stderr,
                    )
                    return 4

                print(f"[OK] robot{robot_id} reached planned goal")
        else:
            running: dict[int, ExecState] = {}
            for robot_id, cmd in jobs:
                print(f"[EXEC] launch robot{robot_id}: cmd={' '.join(cmd)}")
                proc = subprocess.Popen(cmd, text=True)
                running[robot_id] = ExecState(
                    proc=proc,
                    robot_id=robot_id,
                    cmd=cmd,
                    start_time=time.time(),
                )

            pose_sample_period = max(0.1, abs(args.pose_check_period))
            pose_log_period = max(0.2, abs(args.pose_log_period))
            yield_distance = max(0.2, abs(args.yield_distance))
            yield_release_distance = max(yield_distance + 0.05, abs(args.yield_release_distance))
            stop_refresh_sec = max(0.05, abs(args.yield_stop_refresh))
            deadlock_wait_sec = max(0.8, abs(args.yield_deadlock_wait))
            deadlock_dist_eps = max(0.01, abs(args.yield_deadlock_dist_eps))
            deadlock_break_sec = max(0.5, abs(args.yield_deadlock_break))
            yield_predict_time = max(0.0, abs(args.yield_predict_time))
            yield_predict_margin = max(0.0, abs(args.yield_predict_margin))
            pose_failsafe_timeout = max(0.3, abs(args.pose_failsafe_timeout))
            hard_guard_distance = max(0.2, abs(args.yield_hard_guard_distance))
            yield_escape_speed = max(0.05, abs(args.yield_escape_speed))
            yield_escape_period = max(0.05, abs(args.yield_escape_period))
            overlap_prep_time = max(0.0, abs(args.overlap_prep_time))
            overlap_release_distance = max(0.2, abs(args.overlap_release_distance))
            start_yield_distance = max(0.2, abs(args.start_yield_distance))
            start_release_distance = max(start_yield_distance + 0.05, abs(args.start_release_distance))
            overlap_release_distance = max(overlap_release_distance, start_release_distance)
            emergency_stop_distance = max(0.0, abs(args.emergency_stop_distance))
            emergency_brake_time = max(0.0, abs(args.emergency_brake_time))
            emergency_predict_cap = max(0.0, abs(args.emergency_predict_cap))
            strict_prep_release_distance = max(0.55, emergency_stop_distance + 0.10)
            # Keep hard-guard in a narrow near-emergency band to avoid medium-distance lockups.
            if emergency_stop_distance > 0.0:
                guard_floor = emergency_stop_distance + 0.04
                guard_ceil = emergency_stop_distance + 0.08
            else:
                guard_floor = 0.24
                guard_ceil = max(0.30, yield_distance * 0.55)
            hard_guard_distance = min(hard_guard_distance, guard_ceil)
            hard_guard_distance = max(hard_guard_distance, guard_floor)
            pose_cache: dict[str, tuple[float, float, float]] | None = None
            next_pose_sample = 0.0
            next_pose_log = 0.0
            last_pose_warn = 0.0
            pause_locks: dict[int, PauseLock] = {}
            manual_pause_until: dict[int, float] = {}
            deadlock_pair_cooldown_until: dict[tuple[int, int], float] = {}
            pair_history: dict[tuple[int, int], tuple[float, float]] = {}
            overlap_wait_for = {
                lower: set(highers)
                for lower, highers in overlap_wait_for_plan.items()
            }
            strict_wait_for = {
                lower: set(highers)
                for lower, highers in strict_wait_for_plan.items()
            }
            overlap_forced_paused: set[int] = set()
            overlap_last_escape_cmd: dict[int, float] = {}
            overlap_prep_until: dict[int, float] = {}
            overlap_prep_deadline: dict[int, float] = {}
            last_pose_ok = 0.0
            pose_safety_hold = False
            last_pose_hold_warn = 0.0

            startup_wait_for: dict[int, set[int]] = {}
            startup_now = time.time()
            try:
                startup_pose = read_robot_world_poses(args.world_name, robot_names, args.ign_timeout)
                pose_cache = startup_pose
                last_pose_ok = startup_now
                next_pose_sample = startup_now + pose_sample_period
                startup_wait_for = build_startup_wait_rules(
                    startup_pose,
                    sorted(running.keys()),
                    start_yield_distance,
                    preferred_wait_for=overlap_wait_for,
                    path_steps_by_robot=path_steps_by_robot,
                )
            except RuntimeError as e:
                print(f"[WARN] startup pose snapshot unavailable: {e}")

            if startup_wait_for:
                startup_items: list[str] = []
                combined_wait_for: dict[int, set[int]] = {}
                for low, highs in strict_wait_for.items():
                    if highs:
                        combined_wait_for.setdefault(low, set()).update(highs)
                for low, highs in overlap_wait_for.items():
                    if highs:
                        combined_wait_for.setdefault(low, set()).update(highs)
                for low, highs in sorted(startup_wait_for.items()):
                    for high in sorted(highs):
                        # Keep path-conflict priority direction authoritative.
                        if low in strict_wait_for.get(high, set()):
                            continue
                        # Never introduce startup rules that close a wait cycle.
                        if edge_creates_wait_cycle(combined_wait_for, low, high):
                            continue
                        startup_items.append(f"robot{low}->robot{high}")
                        overlap_wait_for.setdefault(low, set()).add(high)
                        combined_wait_for.setdefault(low, set()).add(high)
                if startup_items:
                    print(
                        f"[YIELD] startup wait rules (close starts): {', '.join(startup_items)} "
                        f"(enter={start_yield_distance:.2f}m, release={start_release_distance:.2f}m)"
                    )

            if overlap_wait_for:
                for lower_robot, higher_set in sorted(overlap_wait_for.items()):
                    if not higher_set:
                        continue
                    higher_robot = min(higher_set)
                    lower_state = running.get(lower_robot)
                    if lower_state is None or lower_state.proc.poll() is not None:
                        continue

                    if overlap_prep_time <= 0.0:
                        continue

                    if pause_state(lower_state, startup_now):
                        print(
                            f"[YIELD] robot{lower_robot} pre-wait-overlap for robot{higher_robot}"
                        )
                    overlap_forced_paused.add(lower_robot)
                    overlap_last_escape_cmd[lower_robot] = 0.0

                    prep_until = startup_now + overlap_prep_time
                    overlap_prep_until[lower_robot] = prep_until
                    overlap_prep_deadline[lower_robot] = startup_now + max(
                        0.8,
                        overlap_prep_time + 0.5,
                    )
                    higher_state = running.get(higher_robot)
                    if higher_state is None or higher_state.proc.poll() is not None:
                        continue
                    if (not higher_state.paused) and pause_state(higher_state, startup_now):
                        print(
                            f"[YIELD] robot{higher_robot} prep-hold {overlap_prep_time:.1f}s for overlap evac "
                            f"(robot{lower_robot})"
                        )
                    manual_pause_until[higher_robot] = max(
                        manual_pause_until.get(higher_robot, 0.0),
                        prep_until,
                    )

            try:
                while running:
                    now = time.time()
                    finished_ids: list[int] = []
                    for robot_id, state in running.items():
                        rc = state.proc.poll()
                        if rc is not None:
                            if state.paused and state.paused_since is not None:
                                state.paused_total += max(0.0, now - state.paused_since)
                                state.paused_since = None
                                state.paused = False
                            if rc != 0:
                                if rc in (130, -signal.SIGINT):
                                    print(
                                        f"[WARN] robot{robot_id} interrupted (returncode={rc})"
                                    )
                                    for other_id, other_state in running.items():
                                        if other_id != robot_id:
                                            terminate_state(other_state)
                                    return 130
                                print(
                                    f"[FAIL] robot{robot_id} execution failed, returncode={rc}",
                                    file=sys.stderr,
                                )
                                for other_id, other_state in running.items():
                                    if other_id != robot_id:
                                        terminate_state(other_state)
                                return 4
                            print(f"[OK] robot{robot_id} reached planned goal")
                            finished_ids.append(robot_id)

                    for rid in finished_ids:
                        running.pop(rid, None)
                        pause_locks.pop(rid, None)
                        manual_pause_until.pop(rid, None)
                        overlap_wait_for.pop(rid, None)
                        strict_wait_for.pop(rid, None)
                        overlap_forced_paused.discard(rid)
                        overlap_last_escape_cmd.pop(rid, None)
                        overlap_prep_until.pop(rid, None)
                        overlap_prep_deadline.pop(rid, None)
                        for lower_robot in list(overlap_wait_for.keys()):
                            blockers = overlap_wait_for.get(lower_robot)
                            if blockers is None:
                                continue
                            blockers.discard(rid)
                            if blockers:
                                continue
                            overlap_wait_for.pop(lower_robot, None)
                            if lower_robot != rid:
                                overlap_forced_paused.discard(lower_robot)
                                overlap_last_escape_cmd.pop(lower_robot, None)
                                overlap_prep_until.pop(lower_robot, None)
                                overlap_prep_deadline.pop(lower_robot, None)
                        for lower_robot in list(strict_wait_for.keys()):
                            blockers = strict_wait_for.get(lower_robot)
                            if blockers is None:
                                continue
                            blockers.discard(rid)
                            if blockers:
                                continue
                            strict_wait_for.pop(lower_robot, None)

                    if not running:
                        break

                    if now >= next_pose_sample:
                        try:
                            pose_cache = read_robot_world_poses(args.world_name, robot_names, args.ign_timeout)
                            last_pose_ok = now
                        except RuntimeError as e:
                            if now - last_pose_warn >= 2.0:
                                print(f"[WARN] live pose read failed: {e}")
                                last_pose_warn = now
                        finally:
                            next_pose_sample = now + pose_sample_period

                    if args.show_live_poses and pose_cache is not None and now >= next_pose_log:
                        print(f"[POSE] {format_pose_line(pose_cache, [1, 2, 3])}")
                        next_pose_log = now + pose_log_period

                    stale_age = (now - last_pose_ok) if last_pose_ok > 0.0 else float("inf")
                    pose_stale = (pose_cache is None) or (stale_age > pose_failsafe_timeout)
                    if pose_stale:
                        if (not pose_safety_hold) or (now - last_pose_hold_warn) >= 2.0:
                            age_text = f"{stale_age:.2f}s" if math.isfinite(stale_age) else "n/a"
                            print(
                                f"[WARN] pose safety hold: stale world pose ({age_text}), pausing all robots"
                            )
                            last_pose_hold_warn = now
                        pose_safety_hold = True
                        for state in running.values():
                            if not state.paused:
                                pause_state(state, now)
                            refresh_paused_stop(state, now, stop_refresh_sec)
                    else:
                        if pose_safety_hold:
                            print("[YIELD] pose feed recovered, resuming traffic control")
                        pose_safety_hold = False

                    if (not pose_safety_hold) and overlap_wait_for:
                        for lower_robot, higher_set in list(overlap_wait_for.items()):
                            if not higher_set:
                                overlap_wait_for.pop(lower_robot, None)
                                continue

                            lower_state = running.get(lower_robot)
                            if lower_state is None:
                                continue

                            active_highers: list[int] = []
                            for higher_robot in sorted(higher_set):
                                higher_state = running.get(higher_robot)
                                if higher_state is None or higher_state.proc.poll() is not None:
                                    continue
                                active_highers.append(higher_robot)

                            primary_higher = active_highers[0] if active_highers else min(higher_set)

                            if active_highers:
                                newly_forced = lower_robot not in overlap_forced_paused
                                prep_until = overlap_prep_until.get(lower_robot, 0.0)
                                prep_deadline = overlap_prep_deadline.get(lower_robot, now)
                                if newly_forced and overlap_prep_time > 0.0:
                                    prep_until = now + overlap_prep_time
                                    overlap_prep_until[lower_robot] = prep_until
                                    prep_deadline = now + max(
                                        0.8,
                                        overlap_prep_time + 0.5,
                                    )
                                    overlap_prep_deadline[lower_robot] = prep_deadline

                                pair_dist = None
                                nearest_higher = primary_higher
                                if pose_cache is not None:
                                    nearest_dist = float("inf")
                                    for higher_robot in active_highers:
                                        cand_dist = get_pose_distance(
                                            pose_cache,
                                            lower_robot,
                                            higher_robot,
                                        )
                                        if cand_dist is None:
                                            continue
                                        if cand_dist < nearest_dist:
                                            nearest_dist = cand_dist
                                            nearest_higher = higher_robot
                                    if nearest_dist < float("inf"):
                                        pair_dist = nearest_dist

                                strict_blockers = strict_wait_for.get(lower_robot, set())
                                active_strict_blockers = [
                                    higher_robot
                                    for higher_robot in active_highers
                                    if higher_robot in strict_blockers
                                ]

                                # If a strict-follower still sits very close to its blocker,
                                # extend prep-hold briefly so the follower can side-step away
                                # before the blocker starts moving. Keep this strictly bounded
                                # so prep-hold cannot grow into an indefinite deadlock.
                                if (
                                    active_strict_blockers
                                    and pair_dist is not None
                                    and pair_dist < strict_prep_release_distance
                                    and prep_until < prep_deadline
                                ):
                                    prep_until = min(
                                        prep_deadline,
                                        max(prep_until, now + 0.20),
                                    )
                                    overlap_prep_until[lower_robot] = prep_until

                                # For planned path conflicts, keep lower-priority robot waiting
                                # until blocking higher-priority robots have fully finished.
                                needs_overlap_hold = bool(active_strict_blockers)
                                if (not needs_overlap_hold):
                                    needs_overlap_hold = prep_until > now
                                if (not needs_overlap_hold) and pair_dist is not None:
                                    needs_overlap_hold = pair_dist < overlap_release_distance

                                if not needs_overlap_hold:
                                    overlap_forced_paused.discard(lower_robot)
                                    overlap_last_escape_cmd.pop(lower_robot, None)
                                    overlap_prep_until.pop(lower_robot, None)
                                    overlap_prep_deadline.pop(lower_robot, None)
                                    overlap_wait_for.pop(lower_robot, None)
                                    strict_wait_for.pop(lower_robot, None)
                                    can_resume_overlap_wait = (
                                        lower_state.paused
                                        and lower_robot not in pause_locks
                                        and manual_pause_until.get(lower_robot, 0.0) <= now
                                    )
                                    if can_resume_overlap_wait and resume_state(lower_state, now):
                                        dist_text = f", dist={pair_dist:.2f} m" if pair_dist is not None else ""
                                        print(
                                            f"[YIELD] robot{lower_robot} resume (path clear from robot{nearest_higher}{dist_text})"
                                        )
                                    continue

                                sent_overlap_escape = False
                                if not lower_state.paused:
                                    if pause_state(lower_state, now):
                                        if active_strict_blockers:
                                            blockers_text = ",".join(
                                                f"robot{rid}" for rid in sorted(active_strict_blockers)
                                            )
                                            print(
                                                f"[YIELD] robot{lower_robot} wait-path-conflict for {blockers_text}"
                                            )
                                        else:
                                            print(
                                                f"[YIELD] robot{lower_robot} wait-overlap for robot{nearest_higher}"
                                            )
                                overlap_forced_paused.add(lower_robot)

                                higher_state = running.get(primary_higher)
                                if prep_until > now and higher_state is not None:
                                    if (not higher_state.paused) and pause_state(higher_state, now):
                                        print(
                                            f"[YIELD] robot{primary_higher} prep-hold {overlap_prep_time:.1f}s for overlap evac "
                                            f"(robot{lower_robot})"
                                        )
                                    manual_pause_until[primary_higher] = max(
                                        manual_pause_until.get(primary_higher, 0.0),
                                        prep_until,
                                    )
                                    refresh_paused_stop(higher_state, now, stop_refresh_sec)

                                # For strict path-conflict waits, keep escape nudges only while
                                # the follower is still too close to its blocker. This clears
                                # startup nose-to-tail traps without continuously drifting away.
                                should_overlap_escape = True
                                if active_strict_blockers and now >= prep_until:
                                    close_to_blocker = (
                                        pair_dist is not None
                                        and pair_dist < strict_prep_release_distance
                                    )
                                    should_overlap_escape = close_to_blocker

                                if pose_cache is not None and should_overlap_escape:
                                    if newly_forced:
                                        overlap_last_escape_cmd[lower_robot] = 0.0
                                    escape_period = yield_escape_period
                                    if active_strict_blockers and now >= prep_until:
                                        escape_period = max(0.25, yield_escape_period * 1.8)
                                    if (now - overlap_last_escape_cmd.get(lower_robot, 0.0)) >= escape_period:
                                        overlap_escape_speed = max(0.60, yield_escape_speed * 3.0)
                                        if active_strict_blockers:
                                            # In strict waits, prioritize clearing the primary leader path.
                                            # Escaping from another forced follower can push in the wrong direction.
                                            escape_from_robot = primary_higher
                                        else:
                                            escape_from_robot = nearest_higher
                                        nearest_forced = None
                                        nearest_forced_dist = float("inf")
                                        for other_robot in overlap_forced_paused:
                                            if other_robot == lower_robot:
                                                continue
                                            if other_robot not in running:
                                                continue
                                            other_dist = get_pose_distance(
                                                pose_cache,
                                                lower_robot,
                                                other_robot,
                                            )
                                            if other_dist is None:
                                                continue
                                            if other_dist < nearest_forced_dist:
                                                nearest_forced_dist = other_dist
                                                nearest_forced = other_robot

                                        if (
                                            (not active_strict_blockers)
                                            and nearest_forced is not None
                                            and nearest_forced_dist < overlap_release_distance
                                        ):
                                            escape_from_robot = nearest_forced

                                        overlap_escape_ok, _, _ = publish_escape_cmd(
                                            pose_cache,
                                            lower_robot,
                                            escape_from_robot,
                                            overlap_escape_speed,
                                            prefer_lateral=True,
                                            timeout_sec=0.30,
                                        )
                                        if overlap_escape_ok:
                                            overlap_last_escape_cmd[lower_robot] = now
                                            lower_state.last_stop_sent = now
                                            sent_overlap_escape = True
                                if not sent_overlap_escape:
                                    refresh_paused_stop(lower_state, now, stop_refresh_sec)
                            else:
                                release_blocked = False
                                nearest_forced = None
                                nearest_forced_dist = float("inf")
                                for other_robot in overlap_forced_paused:
                                    if other_robot == lower_robot:
                                        continue
                                    if other_robot not in running:
                                        continue
                                    other_dist = get_pose_distance(
                                        pose_cache,
                                        lower_robot,
                                        other_robot,
                                    )
                                    if other_dist is None:
                                        release_blocked = True
                                        nearest_forced = other_robot
                                        break
                                    if other_dist < nearest_forced_dist:
                                        nearest_forced_dist = other_dist
                                        nearest_forced = other_robot

                                if nearest_forced is not None and nearest_forced_dist < overlap_release_distance:
                                    release_blocked = True

                                if release_blocked:
                                    sent_overlap_escape = False
                                    if (
                                        pose_cache is not None
                                        and nearest_forced is not None
                                        and (now - overlap_last_escape_cmd.get(lower_robot, 0.0)) >= yield_escape_period
                                    ):
                                        overlap_escape_speed = max(0.60, yield_escape_speed * 3.0)
                                        overlap_escape_ok, _, _ = publish_escape_cmd(
                                            pose_cache,
                                            lower_robot,
                                            nearest_forced,
                                            overlap_escape_speed,
                                            prefer_lateral=True,
                                            timeout_sec=0.30,
                                        )
                                        if overlap_escape_ok:
                                            overlap_last_escape_cmd[lower_robot] = now
                                            lower_state.last_stop_sent = now
                                            sent_overlap_escape = True
                                    if not sent_overlap_escape:
                                        refresh_paused_stop(lower_state, now, stop_refresh_sec)
                                    continue

                                overlap_forced_paused.discard(lower_robot)
                                overlap_last_escape_cmd.pop(lower_robot, None)
                                overlap_prep_until.pop(lower_robot, None)
                                overlap_prep_deadline.pop(lower_robot, None)
                                overlap_wait_for.pop(lower_robot, None)
                                strict_wait_for.pop(lower_robot, None)
                                can_resume_overlap_wait = (
                                    lower_state.paused
                                    and lower_robot not in pause_locks
                                    and manual_pause_until.get(lower_robot, 0.0) <= now
                                )
                                if can_resume_overlap_wait and resume_state(lower_state, now):
                                    print(
                                        f"[YIELD] robot{lower_robot} resume (path clear from robot{primary_higher})"
                                    )

                    if (not pose_safety_hold) and args.priority_yield and pose_cache is not None:
                        active_robot_ids = sorted(running.keys())
                        pair_metrics = build_pair_metrics(
                            pose_cache,
                            active_robot_ids,
                            now,
                            pair_history,
                        )

                        pause_reasons = compute_pause_reasons(
                            active_robot_ids,
                            pair_metrics,
                            yield_distance,
                            yield_predict_time,
                            yield_predict_margin,
                        )

                        for lower_robot, (higher_robot, dist) in pause_reasons.items():
                            if lower_robot in overlap_forced_paused:
                                continue
                            if pair_has_wait_rule(overlap_wait_for, lower_robot, higher_robot):
                                continue
                            if manual_pause_until.get(higher_robot, 0.0) > now:
                                continue
                            state = running.get(lower_robot)
                            if state is None:
                                continue
                            lock = pause_locks.get(lower_robot)
                            if lock is None or lock.higher_robot != higher_robot:
                                pause_locks[lower_robot] = PauseLock(
                                    higher_robot=higher_robot,
                                    since=now,
                                    ref_dist=dist,
                                )

                            if pause_state(state, now):
                                print(
                                    f"[YIELD] robot{lower_robot} pause for robot{higher_robot}, dist={dist:.2f} m"
                                )

                        for robot_id, state in running.items():
                            manual_until = manual_pause_until.get(robot_id)
                            if manual_until is not None:
                                if now < manual_until:
                                    if not state.paused:
                                        pause_state(state, now)
                                    refresh_paused_stop(state, now, stop_refresh_sec)
                                    continue
                                manual_pause_until.pop(robot_id, None)
                                if state.paused and robot_id not in pause_locks:
                                    hold_due_to_active_pair = False
                                    for lower_id, lower_lock in pause_locks.items():
                                        if lower_lock.higher_robot != robot_id:
                                            continue
                                        if lower_id not in running:
                                            continue
                                        pair_dist = get_pose_distance(pose_cache, robot_id, lower_id)
                                        if pair_dist is None or pair_dist < yield_release_distance:
                                            hold_due_to_active_pair = True
                                            break

                                    if hold_due_to_active_pair:
                                        manual_pause_until[robot_id] = max(
                                            manual_pause_until.get(robot_id, 0.0),
                                            now + max(0.4, deadlock_break_sec * 0.5),
                                        )
                                        refresh_paused_stop(state, now, stop_refresh_sec)
                                        continue

                                    if resume_state(state, now):
                                        print(
                                            f"[YIELD] robot{robot_id} resume (pause window done)"
                                        )

                            lock = pause_locks.get(robot_id)
                            if state.paused and lock is None:
                                refresh_paused_stop(state, now, stop_refresh_sec)

                            if not state.paused:
                                continue

                            if robot_id in overlap_forced_paused:
                                last_escape = overlap_last_escape_cmd.get(robot_id, 0.0)
                                if pose_cache is None or (now - last_escape) > (yield_escape_period * 1.5):
                                    refresh_paused_stop(state, now, stop_refresh_sec)
                                continue

                            if lock is None:
                                if robot_id in overlap_forced_paused:
                                    refresh_paused_stop(state, now, stop_refresh_sec)
                                    continue
                                if resume_state(state, now):
                                    print(f"[YIELD] robot{robot_id} resume")
                                continue

                            lock_higher = lock.higher_robot
                            if lock_higher not in running:
                                pause_locks.pop(robot_id, None)
                                if resume_state(state, now):
                                    print(f"[YIELD] robot{robot_id} resume")
                                continue

                            dist = get_pose_distance(pose_cache, robot_id, lock_higher)
                            if dist is None:
                                continue
                            pair_key = (robot_id, lock_higher)

                            if dist <= hard_guard_distance:
                                higher_state = running.get(lock_higher)
                                blocker_escape_vx = 0.0
                                blocker_escape_vy = 0.0
                                if higher_state is not None and higher_state.proc.poll() is None:
                                    if not higher_state.paused:
                                        pause_state(higher_state, now)
                                    manual_pause_until[lock_higher] = max(
                                        manual_pause_until.get(lock_higher, 0.0),
                                        now + deadlock_break_sec,
                                    )
                                    if (now - lock.last_blocker_escape_cmd) >= yield_escape_period:
                                        blocker_escape_speed = max(0.25, yield_escape_speed * 1.8)
                                        blocker_escape_ok, blocker_escape_vx, blocker_escape_vy = publish_escape_cmd(
                                            pose_cache,
                                            lock_higher,
                                            robot_id,
                                            blocker_escape_speed,
                                            timeout_sec=0.30,
                                        )
                                        if blocker_escape_ok:
                                            lock.last_blocker_escape_cmd = now
                                            higher_state.last_stop_sent = now
                                deadlock_pair_cooldown_until[pair_key] = now + deadlock_wait_sec
                                lock.ref_dist = dist
                                lock.since = now
                                escape_ok = False
                                escape_vx = 0.0
                                escape_vy = 0.0
                                if (now - lock.last_escape_cmd) >= yield_escape_period:
                                    escape_ok, escape_vx, escape_vy = publish_escape_cmd(
                                        pose_cache,
                                        robot_id,
                                        lock_higher,
                                        yield_escape_speed,
                                        prefer_lateral=True,
                                    )
                                    if escape_ok:
                                        lock.last_escape_cmd = now
                                        # Avoid zero-stop refresh immediately overriding escape cmd.
                                        state.last_stop_sent = now
                                if not escape_ok:
                                    refresh_paused_stop(state, now, stop_refresh_sec)
                                if (now - lock.last_guard_warn) >= 1.0:
                                    lock.last_guard_warn = now
                                    print(
                                        f"[YIELD] close-hold: pause robot{lock_higher} {deadlock_break_sec:.1f}s, "
                                        f"robot{robot_id} yield-away (dist={dist:.2f} m, "
                                        f"cmd=({escape_vx:.2f},{escape_vy:.2f}), "
                                        f"blocker_cmd=({blocker_escape_vx:.2f},{blocker_escape_vy:.2f}))"
                                    )
                                continue

                            active_reason = pause_reasons.get(robot_id)
                            reason_still_active = (
                                active_reason is not None and active_reason[0] == lock_higher
                            )
                            relaxed_release_distance = max(
                                hard_guard_distance + 0.08,
                                yield_distance * 0.65,
                            )

                            if dist >= yield_release_distance or (
                                (not reason_still_active) and dist >= relaxed_release_distance
                            ):
                                pause_locks.pop(robot_id, None)
                                if resume_state(state, now):
                                    print(
                                        f"[YIELD] robot{robot_id} resume (clear from robot{lock_higher}, dist={dist:.2f} m)"
                                    )
                                continue

                            refresh_paused_stop(state, now, stop_refresh_sec)

                            if abs(dist - lock.ref_dist) > deadlock_dist_eps:
                                lock.ref_dist = dist
                                lock.since = now
                                continue

                            if now < deadlock_pair_cooldown_until.get(pair_key, 0.0):
                                continue
                            if (
                                (now - lock.since) >= deadlock_wait_sec
                                and (now - lock.last_deadlock_break) >= deadlock_wait_sec
                            ):
                                higher_state = running.get(lock_higher)
                                if higher_state is None:
                                    continue
                                lock.last_deadlock_break = now
                                if (not higher_state.paused) and higher_state.proc.poll() is None:
                                    pause_state(higher_state, now)
                                manual_pause_until[lock_higher] = max(
                                    manual_pause_until.get(lock_higher, 0.0),
                                    now + deadlock_break_sec,
                                )
                                deadlock_pair_cooldown_until[pair_key] = now + deadlock_wait_sec
                                pause_locks.pop(robot_id, None)
                                if resume_state(state, now):
                                    print(
                                        f"[YIELD] deadlock-break: pause robot{lock_higher} {deadlock_break_sec:.1f}s, "
                                        f"resume robot{robot_id}"
                                    )

                        if emergency_stop_distance > 0.0:
                            emergency_pair: tuple[int, int, float] | None = None
                            for (ra, rb), metric in pair_metrics.items():
                                predictive_extra = min(
                                    emergency_predict_cap,
                                    metric.closing_speed * emergency_brake_time,
                                )
                                effective_stop = emergency_stop_distance + predictive_extra

                                higher = min(ra, rb)
                                lower = max(ra, rb)
                                lower_lock = pause_locks.get(lower)
                                managed_pair = (
                                    lower_lock is not None and lower_lock.higher_robot == higher
                                )
                                overlap_managed_pair = (
                                    (
                                        rb in overlap_wait_for.get(ra, set())
                                        and ra in overlap_forced_paused
                                    )
                                    or (
                                        ra in overlap_wait_for.get(rb, set())
                                        and rb in overlap_forced_paused
                                    )
                                )
                                overlap_group_pair = (
                                    ra in overlap_forced_paused and rb in overlap_forced_paused
                                )

                                if managed_pair:
                                    effective_stop = min(
                                        effective_stop,
                                        max(0.22, emergency_stop_distance * 0.50),
                                    )

                                if overlap_managed_pair:
                                    effective_stop = min(
                                        effective_stop,
                                        max(0.14, emergency_stop_distance * 0.25),
                                    )
                                    prep_until = overlap_prep_until.get(lower, 0.0)
                                    if prep_until > now:
                                        effective_stop = min(effective_stop, 0.12)

                                if overlap_group_pair:
                                    effective_stop = min(effective_stop, 0.12)

                                if metric.dist <= effective_stop:
                                    emergency_pair = (ra, rb, metric.dist)
                                    break
                            if emergency_pair is not None:
                                ra, rb, dist = emergency_pair
                                print(
                                    f"[FAIL] emergency stop: robot{ra} and robot{rb} too close (dist={dist:.2f} m)"
                                )
                                for state in running.values():
                                    terminate_state(state)
                                return 5

                    for robot_id, state in running.items():
                        if active_elapsed_sec(state, now) > exec_timeout:
                            print(
                                f"[FAIL] robot{robot_id} execution timeout after {exec_timeout:.1f}s",
                                file=sys.stderr,
                            )
                            terminate_state(state)
                            for other_id, other_state in running.items():
                                if other_id != robot_id:
                                    terminate_state(other_state)
                            return 4

                    if running:
                        time.sleep(0.1)
            except KeyboardInterrupt:
                print("[WARN] interrupted by user, terminating all robot tasks")
                for state in running.values():
                    terminate_state(state)
                return 130

        print("[OK] execution finished for all 3 robots")

    return 0


if __name__ == "__main__":
    sys.exit(main())
