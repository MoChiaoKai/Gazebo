#!/usr/bin/env python3
import argparse
from dataclasses import dataclass
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ws_dir = os.path.dirname(os.path.abspath(__file__))
if ws_dir not in sys.path:
    sys.path.insert(0, ws_dir)
from astar_planner import AStarGridPlanner

MATERIAL_POINTS = {
    'A': (-2.5, 1.5),
    'B': (-2.5, 0.0),
    'C': (-2.5, -1.5),
}

STATION_POINTS = {
    'station1': (2.0, 2.0),
    'station2': (2.0, 1.0),
    'station3': (2.0, 0.0),
    'station4': (2.0, -1.0),
    'station5': (2.0, -2.0),
}

START_POINTS = {
    1: (-2.5, -1.5),
    2: (-2.5, 0.0),
    3: (-2.5, 1.5),
}

SUPPLY_DWELL_SEC = 3.0
SUPPLY_APPROACH_X = -2.0
STATION_APPROACH_X = 1.5
ARRIVE_EPS = 0.06

A_STAR_RESOLUTION = 0.5

FACE_YAW_DEG = {
    '+X': 0.0,
    '-X': 180.0,
    '+Y': 90.0,
    '-Y': -90.0,
}


@dataclass
class Stop:
    kind: str
    resource_key: str
    target: tuple[float, float]
    approach: tuple[float, float]
    wait: tuple[float, float]
    dwell_sec: float
    face_label: str
    job_idx: int


@dataclass
class StageAction:
    robot_id: int
    move: bool
    phase: str
    target: tuple[float, float]
    stop: Stop | None
    reason: str


@dataclass
class RobotState:
    robot_id: int
    amr_name: str
    stops: list[Stop]
    current: tuple[float, float]
    stop_index: int = 0
    phase: int = 0  # 0: approach, 1: final target
    ready_time: float = 0.0

    def done(self) -> bool:
        return self.stop_index >= len(self.stops)


@dataclass
class ActiveMotion:
    robot_id: int
    phase: str
    goal: tuple[float, float]
    stop: Stop | None
    route: list[tuple[float, float]]
    proc: subprocess.Popen[str]
    start_time: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Execute AMR schedule using fixed static routes (no online planner waiting rules).'
    )
    parser.add_argument(
        '--schedule',
        type=str,
        default='amr_schedule.jsonl',
        help='jsonl schedule path',
    )
    return parser.parse_args()


def normalize_station_name(raw: str) -> str:
    txt = raw.strip().lower()
    txt = txt.replace('_', '')
    m = re.fullmatch(r'station(\d+)', txt)
    if not m:
        raise ValueError(f'invalid station name: {raw}')
    key = f'station{int(m.group(1))}'
    if key not in STATION_POINTS:
        raise ValueError(f'unsupported station: {raw}')
    return key


def parse_robot_id(amr_name: str) -> int:
    m = re.search(r'(\d+)$', amr_name.strip())
    if not m:
        raise ValueError(f'invalid amr name: {amr_name}')
    rid = int(m.group(1))
    if rid not in (1, 2, 3):
        raise ValueError(f'unsupported robot id in name {amr_name}')
    return rid


def dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def get_wait_pos(target: tuple[float, float], kind: str) -> tuple[float, float]:
    tx, ty = target
    offset = -0.5 if ty > 0 else 0.5
    if abs(ty) < 0.1: offset = 0.5
    wy = ty + offset
    wx = 1.5 if kind == 'station' else -2.0
    return (wx, wy)


def build_stop_list(jobs: list[dict]) -> list[Stop]:
    stops: list[Stop] = []
    for job in jobs:
        if not isinstance(job, dict):
            raise ValueError(f'invalid job entry: {job}')
        job_idx = int(job.get('idx', -1))
        duration = float(job.get('duration', 0.0))
        station_name = normalize_station_name(str(job.get('station', '')))
        station_xy = STATION_POINTS[station_name]

        supply = job.get('supply')
        if isinstance(supply, dict):
            material = str(supply.get('material', '')).strip().upper()
            if material not in MATERIAL_POINTS:
                raise ValueError(f'unsupported supply material: {material}')
            supply_xy = MATERIAL_POINTS[material]
            stops.append(
                Stop(
                    kind='supply',
                    resource_key=f'supply_{material}',
                    target=supply_xy,
                    approach=(SUPPLY_APPROACH_X, supply_xy[1]),
                    wait=get_wait_pos(supply_xy, 'supply'),
                    dwell_sec=SUPPLY_DWELL_SEC,
                    face_label='-X',
                    job_idx=job_idx,
                )
            )

        stops.append(
            Stop(
                kind='station',
                resource_key=station_name,
                target=station_xy,
                approach=(STATION_APPROACH_X, station_xy[1]),
                wait=get_wait_pos(station_xy, 'station'),
                dwell_sec=max(0.0, duration),
                face_label='+X',
                job_idx=job_idx,
            )
        )
    return stops


def load_schedule(path: str) -> dict[int, RobotState]:
    if not os.path.isfile(path):
        raise FileNotFoundError(f'schedule file not found: {path}')

    states: dict[int, RobotState] = {}
    with open(path, 'r', encoding='utf-8') as f:
        for lineno, line in enumerate(f, start=1):
            raw = line.strip()
            if not raw:
                continue
            obj = json.loads(raw)
            amr_name = str(obj.get('amr', '')).strip()
            jobs = obj.get('jobs', [])
            rid = parse_robot_id(amr_name)
            if rid in states:
                raise ValueError(f'duplicate AMR definition at line {lineno}: {amr_name}')
            if rid not in START_POINTS:
                raise ValueError(f'missing start point for AMR {amr_name}')
            stops = build_stop_list(list(jobs))
            states[rid] = RobotState(
                robot_id=rid,
                amr_name=amr_name,
                stops=stops,
                current=START_POINTS[rid],
            )

    for rid in (1, 2, 3):
        if rid not in states:
            raise ValueError(f'missing AMR{rid} in schedule file')
    return states


def release_expired_locks(resource_locks: dict[str, tuple[int, float]], now: float):
    for key in list(resource_locks.keys()):
        owner, until = resource_locks[key]
        if now >= until:
            del resource_locks[key]


def lock_available(
    resource_locks: dict[str, tuple[int, float]],
    key: str,
    robot_id: int,
    now: float,
) -> bool:
    item = resource_locks.get(key)
    if item is None:
        return True
    owner, until = item
    if owner == robot_id:
        return True
    return now >= until


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parse_scalar(block: str, key: str, default: float = 0.0) -> float:
    m = re.search(rf"\b{re.escape(key)}:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", block)
    return float(m.group(1)) if m else default


def parse_world_pose_snapshot(output: str) -> dict[str, tuple[float, float, float]]:
    found = {}
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
        return parse_world_pose_snapshot(output)
    except Exception:
        return {}


def get_wait_pos_candidates(current: tuple[float, float], target: tuple[float, float], kind: str) -> list[tuple[float, float]]:
    tx, ty = target
    cy = current[1]
    wx = 1.5 if kind == 'station' else -2.0
    
    # 將待命點限制在 2.0 到 -2.0 之間，避免選到 2.5 或 -2.5 被上下邊緣的虛擬牆壁擋住
    valid_y = [y / 10.0 for y in range(20, -21, -5)]
    if kind == 'station':
        doors = [2.0, 1.0, 0.0, -1.0, -2.0]
    else:
        doors = [1.5, 0.0, -1.5]
        
    def score(wy: float):
        dist_to_target = abs(wy - ty)
        dist_to_current = abs(wy - cy)
        passed_target = ((wy - cy) * (ty - cy) > 0) and (abs(wy - cy) > abs(ty - cy))
        penalty = 10.0 if passed_target else 0.0
        return penalty + dist_to_target + dist_to_current * 0.01
        
    candidates = []
    for wy in valid_y:
        if any(abs(wy - dy) < 0.1 for dy in doors):
            continue
        candidates.append((wx, wy))
        
    candidates.sort(key=lambda p: score(p[1]))
    return candidates


def advance_zero_move(state: RobotState, now: float, resource_locks: dict[str, tuple[int, float]]):
    while True:
        if state.done():
            return
        if now < state.ready_time:
            return

        stop = state.stops[state.stop_index]
        if state.phase == 0:
            if dist(state.current, stop.approach) <= ARRIVE_EPS:
                state.phase = 1
                continue
            return

        if dist(state.current, stop.target) <= ARRIVE_EPS:
            done_time = now + stop.dwell_sec
            if stop.dwell_sec > 0.0:
                resource_locks[stop.resource_key] = (state.robot_id, done_time)
            state.ready_time = done_time
            state.phase = 0
            state.stop_index += 1
            print(
                f'[AUTO] {state.amr_name} already at {stop.resource_key}, '
                f'face {stop.face_label}, dwell {stop.dwell_sec:.1f}s'
            )
            continue
        return


def build_next_action(
    state: RobotState,
    states: dict[int, RobotState],
    active: dict[int, ActiveMotion],
    now: float,
    resource_locks: dict[str, tuple[int, float]],
    active_target_owners: dict[str, int],
    proposed_moving: set[int],
    stage_idx: int,
    live_poses: dict[str, tuple[float, float, float]],
) -> StageAction:
    advance_zero_move(state, now, resource_locks)

    if state.done():
        return StageAction(
            robot_id=state.robot_id,
            move=False,
            phase='hold',
            target=state.current,
            stop=None,
            reason='done',
        )

    if now < state.ready_time:
        return StageAction(
            robot_id=state.robot_id,
            move=False,
            phase='hold',
            target=state.current,
            stop=None,
            reason=f'dwell until {state.ready_time:.1f}',
        )

    stop = state.stops[state.stop_index]

    def is_locked() -> bool:
        # Special case for the first round to allow swapping positions at supply points.
        # This prevents deadlock when one robot needs to move to a supply spot just vacated by another.
        if stage_idx == 0 and stop.kind == 'supply':
            owner = active_target_owners.get(stop.resource_key)
            if owner is not None and owner != state.robot_id:
                return True
            if not lock_available(resource_locks, stop.resource_key, state.robot_id, now):
                return True
            # In round 1, for supply points, ignore physical presence of other robots.
            return False

        owner = active_target_owners.get(stop.resource_key)
        if owner is not None and owner != state.robot_id:
            return True
        if not lock_available(resource_locks, stop.resource_key, state.robot_id, now):
            return True
            
        for other_rid, other_state in states.items():
            if other_rid == state.robot_id:
                continue
            
            ox, oy = other_state.current
            if other_state.amr_name in live_poses:
                ox, oy = live_poses[other_state.amr_name][:2]
                
            if dist((ox, oy), stop.target) <= 0.45 or dist((ox, oy), stop.approach) <= 0.45:
                return True
        return False

    # If already on exact target, do not backtrack. Dispatch target directly to enforce heading.
    if dist(state.current, stop.target) <= ARRIVE_EPS:
        if is_locked():
            return StageAction(
                robot_id=state.robot_id,
                move=False,
                phase='hold',
                target=state.current,
                stop=None,
                reason=f'waiting in-flight or locked {stop.resource_key}',
            )

        return StageAction(
            robot_id=state.robot_id,
            move=True,
            phase='target',
            target=stop.target,
            stop=stop,
            reason='already at station target, enforce heading',
        )

    on_direct_lane = False
    if stop.kind == 'station' and state.current[0] >= (stop.target[0] - ARRIVE_EPS):
        on_direct_lane = True
    elif stop.kind == 'supply' and state.current[0] <= (stop.target[0] + ARRIVE_EPS):
        on_direct_lane = True

    if is_locked():
        if on_direct_lane:
            return StageAction(
                robot_id=state.robot_id,
                move=False,
                phase='hold',
                target=state.current,
                stop=None,
                reason=f'holding in lane for {stop.resource_key}',
            )

        candidates = get_wait_pos_candidates(state.current, stop.target, stop.kind)
        selected_wait = None
        
        for dynamic_wait in candidates:
            wait_key = f"wait_{dynamic_wait[0]}_{dynamic_wait[1]}"
            
            wait_owner = active_target_owners.get(wait_key)
            if wait_owner is not None and wait_owner != state.robot_id:
                continue
                
            physically_occupied = False
            for other_rid, other_state in states.items():
                if other_rid == state.robot_id:
                    continue
                
                ox, oy = other_state.current
                if other_state.amr_name in live_poses:
                    ox, oy = live_poses[other_state.amr_name][:2]
                    
                if dist((ox, oy), dynamic_wait) <= 0.45:
                    physically_occupied = True
                    break
                    
            if physically_occupied:
                continue
                
            selected_wait = dynamic_wait
            break
            
        if selected_wait is None:
            return StageAction(
                robot_id=state.robot_id,
                move=False,
                phase='hold',
                target=state.current,
                stop=None,
                reason='all wait points occupied, holding at current',
            )
            
        if dist(state.current, selected_wait) > ARRIVE_EPS:
            return StageAction(
                robot_id=state.robot_id,
                move=True,
                phase='wait',
                target=selected_wait,
                stop=stop,
                reason=f'moving to wait point for {stop.resource_key}',
            )
        else:
            return StageAction(
                robot_id=state.robot_id,
                move=False,
                phase='hold',
                target=state.current,
                stop=None,
                reason=f'holding at wait point for {stop.resource_key}',
            )

    if state.phase == 0:
        if on_direct_lane:
            return StageAction(
                robot_id=state.robot_id,
                move=True,
                phase='target',
                target=stop.target,
                stop=stop,
                reason=f'{stop.kind} direct target from lane',
            )

        return StageAction(
            robot_id=state.robot_id,
            move=True,
            phase='approach',
            target=stop.approach,
            stop=stop,
            reason='approach for heading',
        )

    return StageAction(
        robot_id=state.robot_id,
        move=True,
        phase='target',
        target=stop.target,
        stop=stop,
        reason='execute stop',
    )


def next_ready_time(states: dict[int, RobotState], now: float) -> float | None:
    waits: list[float] = []
    for rid in (1, 2, 3):
        st = states[rid]
        if st.done():
            continue
        if st.ready_time > now:
            waits.append(st.ready_time)
    if not waits:
        return None
    return min(waits)


def next_lock_release_time(resource_locks: dict[str, tuple[int, float]], now: float) -> float | None:
    waits: list[float] = []
    for _, until in resource_locks.values():
        if until > now:
            waits.append(until)
    if not waits:
        return None
    return min(waits)


def load_world_map(world_file: str, resolution: float) -> tuple[int, int, list[bool]]:
    tree = ET.parse(world_file)
    root = tree.getroot()
    world_elem = next((elem for elem in root.iter() if elem.tag.split('}')[-1] == 'world'), None)
    width = int(round(5.0 / resolution)) + 1
    height = width
    blocked = [False] * (width * height)

    if world_elem is not None:
        for model in world_elem:
            if model.tag.split('}')[-1] != 'model': continue
            if not model.attrib.get('name', '').startswith('Obstacle'): continue
            static_text = ''
            pose_text = ''
            for child in model:
                tag = child.tag.split('}')[-1]
                if tag == 'static' and child.text: static_text = child.text.strip().lower()
                if tag == 'pose' and child.text: pose_text = child.text.strip()
            if static_text not in ('1', 'true', 'yes'): continue
            vals = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', pose_text)
            if len(vals) < 2: continue
            x, y = float(vals[0]), float(vals[1])
            snapped_x = round((x - (-2.5)) / resolution) * resolution + (-2.5)
            snapped_y = round((y - (-2.5)) / resolution) * resolution + (-2.5)
            mx = int(math.floor((snapped_x - (-2.5 - 0.5 * resolution)) / resolution))
            my = int(math.floor((snapped_y - (-2.5 - 0.5 * resolution)) / resolution))
            if 0 <= mx < width and 0 <= my < height:
                blocked[my * width + mx] = True

    # 加上「虛擬牆壁」來強制 A* 遵守原本的專用道規則 (避免撞牆與誤闖站點)
    station_y = [2.0, 1.0, 0.0, -1.0, -2.0]
    supply_y = [1.5, 0.0, -1.5]
    
    for cx in range(width):
        for cy in range(height):
            wx = -2.5 - 0.5 * resolution + (cx + 0.5) * resolution
            wy = -2.5 - 0.5 * resolution + (cy + 0.5) * resolution
            idx = cy * width + cx
            
            # 1. 封鎖地圖最上方與最下方的牆壁邊緣
            if wy > 2.15 or wy < -2.15:
                blocked[idx] = True
                
            # 2. 封鎖右側 (工作站區域 x >= 1.75)，只留下站點正前方的通道
            if wx >= 1.75:
                is_door = any(abs(wy - sy) < 0.2 for sy in station_y)
                if not is_door:
                    blocked[idx] = True
                    
            # 3. 封鎖左側 (供料區 x <= -2.25)，只留下供料區正前方的通道
            if wx <= -2.25:
                is_door = any(abs(wy - sy) < 0.2 for sy in supply_y)
                if not is_door:
                    blocked[idx] = True

    return width, height, blocked


def get_route_cells(world_path: list[tuple[float, float]], resolution: float) -> set[tuple[int, int]]:
    cells = set()
    if not world_path: return cells
    
    if len(world_path) == 1:
        wx, wy = world_path[0]
        mx = int(math.floor((wx - (-2.5 - 0.5 * resolution)) / resolution))
        my = int(math.floor((wy - (-2.5 - 0.5 * resolution)) / resolution))
        cells.add((mx, my))
        return cells
        
    for i in range(1, len(world_path)):
        x0, y0 = world_path[i-1]
        x1, y1 = world_path[i]
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(math.ceil(dist / (resolution * 0.5))))
        for k in range(steps + 1):
            t = k / steps
            wx = x0 + (x1 - x0) * t
            wy = y0 + (y1 - y0) * t
            mx = int(math.floor((wx - (-2.5 - 0.5 * resolution)) / resolution))
            my = int(math.floor((wy - (-2.5 - 0.5 * resolution)) / resolution))
            cells.add((mx, my))
    return cells


def build_smart_astar_route(
    planner_base: AStarGridPlanner,
    resolution: float,
    start: tuple[float, float],
    goal: tuple[float, float],
    blocked_cells: set[tuple[int, int]],
) -> list[tuple[float, float]] | None:
    width = planner_base.width
    height = planner_base.height
    grid = planner_base.blocked.copy()

    for mx, my in blocked_cells:
        if 0 <= mx < width and 0 <= my < height:
            grid[my * width + mx] = True

    temp_planner = AStarGridPlanner(width, height, grid)
    smx = int(math.floor((start[0] - (-2.5 - 0.5 * resolution)) / resolution))
    smy = int(math.floor((start[1] - (-2.5 - 0.5 * resolution)) / resolution))
    gmx = int(math.floor((goal[0] - (-2.5 - 0.5 * resolution)) / resolution))
    gmy = int(math.floor((goal[1] - (-2.5 - 0.5 * resolution)) / resolution))

    start_cell = temp_planner.find_nearest_free_cell(smx, smy, max_radius_cells=2)
    goal_cell = temp_planner.find_nearest_free_cell(gmx, gmy, max_radius_cells=2)

    if not start_cell or not goal_cell:
        return None

    path_cells = temp_planner.plan(start_cell, goal_cell)
    if not path_cells:
        return None

    compressed = temp_planner.compress_path(path_cells)
    world_path = [start]
    
    for cx, cy in compressed:
        wx = -2.5 - 0.5 * resolution + (cx + 0.5) * resolution
        wy = -2.5 - 0.5 * resolution + (cy + 0.5) * resolution
        wx_round, wy_round = round(wx, 3), round(wy, 3)
        
        last_wx, last_wy = world_path[-1]
        if abs(last_wx - wx_round) > 1e-3 and abs(last_wy - wy_round) > 1e-3:
            world_path.append((wx_round, last_wy))
            
        if math.hypot(world_path[-1][0] - wx_round, world_path[-1][1] - wy_round) > 1e-3:
            world_path.append((wx_round, wy_round))

    last_wx, last_wy = world_path[-1]
    if abs(last_wx - goal[0]) > 1e-3 and abs(last_wy - goal[1]) > 1e-3:
        world_path.append((goal[0], last_wy))

    if math.hypot(world_path[-1][0] - goal[0], world_path[-1][1] - goal[1]) > 1e-3:
        world_path.append(goal)
        
    return world_path


def serialize_waypoints(points: list[tuple[float, float]]) -> str:
    return ';'.join(f'{x:.3f},{y:.3f}' for x, y in points)


def terminate_process(proc: subprocess.Popen[str]):
    if proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        try:
            proc.kill()
        except Exception:
            pass
    except Exception:
        pass


def apply_completed_motion(
    state: RobotState,
    phase: str,
    goal: tuple[float, float],
    stop: Stop | None,
    done_time: float,
    resource_locks: dict[str, tuple[int, float]],
):
    state.current = goal
    if phase == 'wait':
        return

    if phase == 'approach':
        state.phase = 1
        return

    if stop is None:
        return

    state.phase = 0
    state.stop_index += 1
    state.ready_time = done_time + stop.dwell_sec
    if stop.dwell_sec > 0.0:
        resource_locks[stop.resource_key] = (state.robot_id, state.ready_time)

    print(
        f'[ARRIVE] AMR{state.robot_id}: {stop.resource_key} '
        f'face {stop.face_label}, dwell {stop.dwell_sec:.1f}s'
    )


def build_go_to_cmd(
    go_to_script: str,
    rid: int,
    phase: str,
    goal: tuple[float, float],
    route: list[tuple[float, float]],
    stop: Stop | None,
    stop_pos_tol: float,
    stop_yaw_tol_deg: float,
    stop_yaw_settle_sec: float,
    station_yaw_deg: float,
) -> list[str]:
    waypoint_str = serialize_waypoints(route)
    cmd = [
        sys.executable,
        go_to_script,
        '--robot', str(rid),
        '--x', f'{goal[0]:.3f}',
        '--y', f'{goal[1]:.3f}',
        f'--path-waypoints={waypoint_str}',
    ]

    if phase == 'target' and stop is not None:
        if stop.kind == 'station':
            yaw_deg = float(station_yaw_deg)
        else:
            yaw_deg = FACE_YAW_DEG.get(stop.face_label)
            if yaw_deg is None:
                raise ValueError(f'unsupported face_label for yaw mapping: {stop.face_label}')
        cmd.extend(
            [
                '--target-yaw-deg', f'{yaw_deg:.1f}',
                '--pos-tol', f'{max(0.01, abs(stop_pos_tol)):.3f}',
                '--yaw-tol-deg', f'{max(0.1, abs(stop_yaw_tol_deg)):.2f}',
                '--yaw-settle-sec', f'{max(0.0, abs(stop_yaw_settle_sec)):.2f}',
            ]
        )

    return cmd


def active_target_owner_map(active: dict[int, ActiveMotion]) -> dict[str, int]:
    owners: dict[str, int] = {}
    for rid, motion in active.items():
        if motion.phase in ('approach', 'target') and motion.stop is not None:
            owners[motion.stop.resource_key] = rid
        elif motion.phase == 'wait':
            owners[f"wait_{motion.goal[0]}_{motion.goal[1]}"] = rid
    return owners


def main() -> int:
    args = parse_args()

    # Default parameters moved from CLI args
    go_to_script_arg = 'go_to_point.py'
    exec_timeout_arg = 240.0
    dry_run_arg = False
    show_waypoints_arg = False
    max_stages_arg = 0
    stop_pos_tol_arg = 0.04
    stop_yaw_tol_deg_arg = 2.5
    stop_yaw_settle_sec_arg = 0.55
    station_yaw_deg_arg = 0.0

    ws_dir = os.path.dirname(os.path.abspath(__file__))
    schedule_path = args.schedule
    if os.path.isabs(schedule_path) or os.path.dirname(schedule_path):
        schedule_path = os.path.abspath(schedule_path)
    else:
        schedule_path = os.path.join(ws_dir, schedule_path)

    go_to_script = go_to_script_arg
    if not os.path.isabs(go_to_script):
        go_to_script = os.path.join(ws_dir, go_to_script)
    if not os.path.isfile(go_to_script):
        print(f'[ERROR] go_to script not found: {go_to_script}', file=sys.stderr)
        return 1

    try:
        states = load_schedule(schedule_path)
    except Exception as e:
        print(f'[ERROR] failed to parse schedule: {e}', file=sys.stderr)
        return 1

    print(f'[INFO] loaded schedule: {schedule_path}')
    print('[INFO] route mode: A* dynamic planning with virtual corridors')
    for rid in (1, 2, 3):
        st = states[rid]
        print(f'[INFO] {st.amr_name}: stops={len(st.stops)}, start={st.current}')

    world_file = os.path.join(ws_dir, 'src', 'yahboom_rosmaster', 'yahboom_rosmaster_gazebo', 'worlds', 'temp.world')
    if not os.path.exists(world_file):
        world_file = os.path.join(ws_dir, 'install', 'yahboom_rosmaster_gazebo', 'share', 'yahboom_rosmaster_gazebo', 'worlds', 'temp.world')
        
    if not os.path.exists(world_file):
        print(f'[ERROR] world file not found: {world_file}')
        return 1

    map_width, map_height, map_blocked = load_world_map(world_file, A_STAR_RESOLUTION)
    base_planner = AStarGridPlanner(map_width, map_height, map_blocked)

    stage_idx = 0
    resource_locks: dict[str, tuple[int, float]] = {}
    active: dict[int, ActiveMotion] = {}
    max_dispatch_rounds = max(0, int(max_stages_arg))
    allow_new_rounds = True
    per_motion_timeout = max(10.0, float(exec_timeout_arg))
    last_yield_reason: dict[int, str] = {}
    last_replan_time = 0.0
    live_poses: dict[str, tuple[float, float, float]] = {}

    try:
        while True:
            now = time.time()
            release_expired_locks(resource_locks, now)
            progressed = False

            # 獲取即時座標，用於中斷檢查與路徑放行
            if now - last_replan_time > 0.5:
                last_replan_time = now
                new_poses = read_world_poses('default', 0.5)
                if new_poses:
                    live_poses = new_poses

                for rid, motion in list(active.items()):
                    amr_name = states[rid].amr_name
                    if amr_name not in live_poses:
                        continue
                    px, py, _ = live_poses[amr_name]

                    # 如果已經快抵達目標 (< 0.3m)，不中斷以確保對位進站順利
                    if dist((px, py), motion.goal) < 0.3:
                        continue

                    # 建立臨時狀態機模擬
                    temp_state = RobotState(
                        robot_id=states[rid].robot_id,
                        amr_name=states[rid].amr_name,
                        stops=states[rid].stops,
                        current=(px, py),
                        stop_index=states[rid].stop_index,
                        phase=states[rid].phase,
                        ready_time=states[rid].ready_time,
                    )
                    
                    temp_active = {k: v for k, v in active.items() if k != rid}
                    temp_owner_map = active_target_owner_map(temp_active)

                    new_action = build_next_action(
                        state=temp_state,
                        states=states,
                        active=temp_active,
                        now=now,
                        resource_locks=resource_locks,
                        active_target_owners=temp_owner_map,
                        proposed_moving=set(),
                        stage_idx=stage_idx,
                        live_poses=live_poses,
                    )

                    should_preempt = False
                    preempt_reason = ""

                    # 狀況 A：本來要去排隊，結果目標空出來了！直接去目標
                    if new_action.phase == 'target' and motion.phase == 'wait':
                        should_preempt = True
                        preempt_reason = "target became available"
                    elif new_action.phase == 'approach' and motion.phase == 'wait':
                        should_preempt = True
                        preempt_reason = "approach became available"

                    # 狀況 B：本來的行駛路線上，有其他的車剛好「停下來了」擋住路
                    if not should_preempt and motion.route:
                        current_blocked = set()
                        for other_rid, other_state in states.items():
                            if other_rid == rid: continue
                            if other_rid not in temp_active:
                                ox, oy = other_state.current
                                if states[other_rid].amr_name in live_poses:
                                    ox, oy = live_poses[states[other_rid].amr_name][:2]
                                current_blocked.update(get_route_cells([(ox, oy)], A_STAR_RESOLUTION))
                        
                        # 找出機器人當下在路線上的最近點，只檢查「未來的路徑」是否有被擋住
                        nearest_idx = 0
                        min_d = float('inf')
                        for i, pt in enumerate(motion.route):
                            d = dist((px, py), pt)
                            if d < min_d:
                                min_d = d
                                nearest_idx = i
                        future_route = [(px, py)] + motion.route[nearest_idx:]
                        
                        my_route_cells = get_route_cells(future_route, A_STAR_RESOLUTION)
                        if my_route_cells.intersection(current_blocked):
                            should_preempt = True
                            preempt_reason = "future route blocked by physical robot"
                            
                    # 觸發中斷重算
                    if should_preempt:
                        print(f"[REPLAN] AMR{rid} preempting mid-flight ({motion.phase}): {preempt_reason}")
                        terminate_process(motion.proc)
                        del active[rid]
                        states[rid].current = (px, py)

            for rid in list(active.keys()):
                motion = active[rid]
                rc = motion.proc.poll()
                if rc is None:
                    if now - motion.start_time > per_motion_timeout:
                        for other in active.values():
                            terminate_process(other.proc)
                        print(
                            f'[FAIL] robot{rid} execution timeout after {per_motion_timeout:.1f}s',
                            file=sys.stderr,
                        )
                        return 124
                    continue

                progressed = True
                del active[rid]

                if rc != 0:
                    for other in active.values():
                        terminate_process(other.proc)
                    if rc in (130, -signal.SIGINT):
                        print(f'[WARN] robot{rid} interrupted (returncode={rc})')
                        return 130
                    print(f'[FAIL] robot{rid} execution failed, returncode={rc}', file=sys.stderr)
                    return rc if rc != 0 else 1

                print(f'[OK] robot{rid} reached planned goal')
                state = states[rid]
                apply_completed_motion(
                    state=state,
                    phase=motion.phase,
                    goal=motion.goal,
                    stop=motion.stop,
                    done_time=now,
                    resource_locks=resource_locks,
                )

            if all(states[rid].done() for rid in (1, 2, 3)) and not active:
                print('[OK] all AMR jobs completed')
                return 0

            if (not allow_new_rounds) and not active:
                print(f'[INFO] reached max dispatch rounds: {max_dispatch_rounds}, stopping early')
                return 0

            if allow_new_rounds:
                owner_map = active_target_owner_map(active)
                round_plans: list[tuple[int, StageAction, list[tuple[float, float]]]] = []

                proposed_moving: set[int] = set()

                # 只封鎖所有機器人的「當前實體位置」，不再封鎖「未來整段路線」
                # 消除幻影牆壁，讓機器人可以駛入空曠區域動態禮讓
                physical_blocked_cells = set()
                for r_id, r_state in states.items():
                    ox, oy = r_state.current
                    if r_state.amr_name in live_poses:
                        ox, oy = live_poses[r_state.amr_name][:2]
                    physical_blocked_cells.update(get_route_cells([(ox, oy)], A_STAR_RESOLUTION))

                for rid in (1, 2, 3):
                    if rid in active:
                        continue

                    state = states[rid]
                    action = build_next_action(
                        state=state,
                        states=states,
                        active=active,
                        now=now,
                        resource_locks=resource_locks,
                        active_target_owners=owner_map,
                        proposed_moving=proposed_moving,
                        stage_idx=stage_idx,
                        live_poses=live_poses,
                    )
                    if not action.move:
                        continue

                    proposed_moving.add(rid)

                    current_blocked = set(physical_blocked_cells)
                    
                    # 移除自己的實體位置，讓自己可以順利規劃出發路徑
                    my_ox, my_oy = state.current
                    if state.amr_name in live_poses:
                        my_ox, my_oy = live_poses[state.amr_name][:2]
                    my_cells = get_route_cells([(my_ox, my_oy)], A_STAR_RESOLUTION)
                    current_blocked.difference_update(my_cells)

                    # 第一回合 (stage_idx == 0) 去供料區時，把目標與進場點從 A* 障礙物中強制移除
                    # 避免 A* 把停在那裡的另一台車當成死路而卡住
                    if stage_idx == 0 and action.stop is not None and action.stop.kind == 'supply':
                        dest_cells = get_route_cells([action.stop.target, action.stop.approach], A_STAR_RESOLUTION)
                        current_blocked.difference_update(dest_cells)

                    route = build_smart_astar_route(
                        base_planner, A_STAR_RESOLUTION, state.current, action.target, current_blocked
                    )

                    if route is None:
                        if last_yield_reason.get(rid) != 'blocked by A* path traffic':
                            print(f'[YIELD] AMR{rid} holds at {state.current}: waiting for clear A* path')
                            last_yield_reason[rid] = 'blocked by A* path traffic'
                        proposed_moving.discard(rid)
                        continue

                    last_yield_reason.pop(rid, None)
                    # 封鎖新路線的「終點」，避免同回合的其他車規劃疊加在你未來的停車格上
                    if route:
                        physical_blocked_cells.update(get_route_cells([route[-1]], A_STAR_RESOLUTION))
                    
                    if action.phase in ('approach', 'target') and action.stop is not None:
                        owner_map[action.stop.resource_key] = rid
                    elif action.phase == 'wait':
                        owner_map[f"wait_{action.target[0]}_{action.target[1]}"] = rid
                    
                    round_plans.append((rid, action, route))

                if round_plans:
                    progressed = True
                    stage_idx += 1
                    print(f'\n===== DISPATCH ROUND {stage_idx} =====')

                    round_routes = {rid: route for rid, _, route in round_plans if route}
                    for rid, action, route in round_plans:
                        if action.stop is not None:
                            print(
                                f'[MOVE] AMR{rid}: {action.phase} -> {action.stop.resource_key} '
                                f'target=({action.target[0]:.2f}, {action.target[1]:.2f})'
                            )
                        if route:
                            print(f'[ROUTE] AMR{rid}: points={len(route)} start={route[0]} end={route[-1]}')
                            if show_waypoints_arg:
                                print(f'[ROUTE-WP] AMR{rid}: {serialize_waypoints(route)}')

                    for rid, action, route in round_plans:
                        cmd = build_go_to_cmd(
                            go_to_script=go_to_script,
                            rid=rid,
                            phase=action.phase,
                            goal=action.target,
                            route=route,
                            stop=action.stop,
                            stop_pos_tol=stop_pos_tol_arg,
                            stop_yaw_tol_deg=stop_yaw_tol_deg_arg,
                            stop_yaw_settle_sec=stop_yaw_settle_sec_arg,
                            station_yaw_deg=station_yaw_deg_arg,
                        )
                        print(f"[EXEC] launch robot{rid}: cmd={' '.join(cmd)}")

                        if dry_run_arg:
                            print(f'[OK] robot{rid} reached planned goal (dry-run)')
                            apply_completed_motion(
                                state=states[rid],
                                phase=action.phase,
                                goal=action.target,
                                stop=action.stop,
                                done_time=now,
                                resource_locks=resource_locks,
                            )
                            continue

                        proc = subprocess.Popen(cmd, text=True)
                        active[rid] = ActiveMotion(
                            robot_id=rid,
                            phase=action.phase,
                            goal=action.target,
                            stop=action.stop,
                            route=route,
                            proc=proc,
                            start_time=now,
                        )

                    if max_dispatch_rounds > 0 and stage_idx >= max_dispatch_rounds:
                        allow_new_rounds = False

            if progressed:
                continue

            wake_candidates: list[float] = []
            wake_ready = next_ready_time(states, now)
            if wake_ready is not None:
                wake_candidates.append(wake_ready)
            wake_lock = next_lock_release_time(resource_locks, now)
            if wake_lock is not None:
                wake_candidates.append(wake_lock)

            if wake_candidates:
                wake = min(wake_candidates)
                sleep_sec = max(0.05, min(0.50, wake - now))
                time.sleep(sleep_sec)
                continue

            if active:
                time.sleep(0.05)
                continue

            print('[WARN] no movable AMR and no wake time, exiting')
            return 2

    except KeyboardInterrupt:
        for motion in active.values():
            terminate_process(motion.proc)
        print('\n[WARN] interrupted by user')
        return 130


if __name__ == '__main__':
    sys.exit(main())
