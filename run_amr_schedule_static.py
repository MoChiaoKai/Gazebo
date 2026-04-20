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

GRID_STEP = 0.5
LEFT_GATE_X = -2.0
RIGHT_GATE_X = 2.0
RIGHT_GATE_X_BY_ROBOT = {
    1: 2.0,
    2: 1.5,
    3: 2.0,
}
TRANSIT_Y = {
    1: -2.0,
    2: 0.0,
    3: 1.5,
}
MIDLINE_BLOCK_X = 0.5
MIDLINE_DETOUR_Y = -0.5

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
        default='amr_schedule_1.jsonl',
        help='jsonl schedule path',
    )
    parser.add_argument(
        '--go-to-script',
        type=str,
        default='go_to_point.py',
        help='path to go_to_point.py',
    )
    parser.add_argument(
        '--exec-timeout',
        type=float,
        default=240.0,
        help='per-motion timeout in seconds',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='print dispatch plan only, do not execute motion',
    )
    parser.add_argument(
        '--show-waypoints',
        action='store_true',
        help='print full waypoint string per moving robot',
    )
    parser.add_argument(
        '--max-stages',
        type=int,
        default=0,
        help='stop after N dispatch rounds (0 means run full schedule)',
    )
    parser.add_argument(
        '--stop-pos-tol',
        type=float,
        default=0.04,
        help='strict position tolerance (m) for station/supply target phase',
    )
    parser.add_argument(
        '--stop-yaw-tol-deg',
        type=float,
        default=2.5,
        help='strict yaw tolerance (deg) for station/supply target phase',
    )
    parser.add_argument(
        '--stop-yaw-settle-sec',
        type=float,
        default=0.55,
        help='required yaw-in-tolerance settle time (s) for station/supply target phase',
    )
    parser.add_argument(
        '--station-yaw-deg',
        type=float,
        default=0.0,
        help='final heading (deg) enforced for station target phase',
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
    now: float,
    resource_locks: dict[str, tuple[int, float]],
    active_target_owners: dict[str, int],
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
    if state.phase == 0:
        # If already on station target, do not backtrack to approach x.
        # Dispatch target phase directly so go_to_point enforces final heading.
        if stop.kind == 'station' and dist(state.current, stop.target) <= ARRIVE_EPS:
            owner = active_target_owners.get(stop.resource_key)
            if owner is not None and owner != state.robot_id:
                return StageAction(
                    robot_id=state.robot_id,
                    move=False,
                    phase='hold',
                    target=state.current,
                    stop=None,
                    reason=f'waiting in-flight {stop.resource_key} by AMR{owner}',
                )

            if lock_available(resource_locks, stop.resource_key, state.robot_id, now):
                return StageAction(
                    robot_id=state.robot_id,
                    move=True,
                    phase='target',
                    target=stop.target,
                    stop=stop,
                    reason='already at station target, enforce heading',
                )

            return StageAction(
                robot_id=state.robot_id,
                move=False,
                phase='hold',
                target=state.current,
                stop=None,
                reason=f'waiting resource {stop.resource_key}',
            )

        # When already on the right station lane (x~=2.0), go directly to station target.
        # This prevents x=2.0 -> 1.5 -> 2.0 pullback between station-to-station jobs.
        if stop.kind == 'station' and state.current[0] >= (stop.target[0] - ARRIVE_EPS):
            owner = active_target_owners.get(stop.resource_key)
            if owner is not None and owner != state.robot_id:
                return StageAction(
                    robot_id=state.robot_id,
                    move=False,
                    phase='hold',
                    target=state.current,
                    stop=None,
                    reason=f'waiting in-flight {stop.resource_key} by AMR{owner}',
                )

            if lock_available(resource_locks, stop.resource_key, state.robot_id, now):
                return StageAction(
                    robot_id=state.robot_id,
                    move=True,
                    phase='target',
                    target=stop.target,
                    stop=stop,
                    reason='station direct target from right lane',
                )

            return StageAction(
                robot_id=state.robot_id,
                move=False,
                phase='hold',
                target=state.current,
                stop=None,
                reason=f'waiting resource {stop.resource_key}',
            )

        return StageAction(
            robot_id=state.robot_id,
            move=True,
            phase='approach',
            target=stop.approach,
            stop=stop,
            reason='approach for heading',
        )

    owner = active_target_owners.get(stop.resource_key)
    if owner is not None and owner != state.robot_id:
        return StageAction(
            robot_id=state.robot_id,
            move=False,
            phase='hold',
            target=state.current,
            stop=None,
            reason=f'waiting in-flight {stop.resource_key} by AMR{owner}',
        )

    if lock_available(resource_locks, stop.resource_key, state.robot_id, now):
        return StageAction(
            robot_id=state.robot_id,
            move=True,
            phase='target',
            target=stop.target,
            stop=stop,
            reason='execute stop',
        )

    return StageAction(
        robot_id=state.robot_id,
        move=False,
        phase='hold',
        target=state.current,
        stop=None,
        reason=f'waiting resource {stop.resource_key}',
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


def append_point(route: list[tuple[float, float]], x: float, y: float):
    p = (round(float(x), 3), round(float(y), 3))
    if route and dist(route[-1], p) <= 1e-6:
        return
    route.append(p)


def append_horizontal(route: list[tuple[float, float]], target_x: float):
    cur_x, cur_y = route[-1]
    target_x = float(target_x)
    if abs(target_x - cur_x) <= 1e-6:
        return

    step = GRID_STEP if target_x > cur_x else -GRID_STEP
    x = cur_x
    for _ in range(200):
        remain = target_x - x
        if remain * step <= 1e-6:
            break
        x_next = x + step
        if (target_x - x_next) * step < 0.0:
            x_next = target_x
        append_point(route, x_next, cur_y)
        x = x_next
    else:
        raise RuntimeError('append_horizontal iteration overflow')


def append_vertical(route: list[tuple[float, float]], target_y: float):
    cur_x, cur_y = route[-1]
    target_y = float(target_y)
    if abs(target_y - cur_y) <= 1e-6:
        return

    step = GRID_STEP if target_y > cur_y else -GRID_STEP
    y = cur_y
    for _ in range(200):
        remain = target_y - y
        if remain * step <= 1e-6:
            break
        y_next = y + step
        if (target_y - y_next) * step < 0.0:
            y_next = target_y
        append_point(route, cur_x, y_next)
        y = y_next
    else:
        raise RuntimeError('append_vertical iteration overflow')


def append_cross_corridor(
    route: list[tuple[float, float]],
    target_x: float,
    transit_y: float,
    prefer_fewer_turns: bool = False,
):
    append_vertical(route, transit_y)
    cur_x, cur_y = route[-1]

    if abs(cur_y) > 1e-6:
        append_horizontal(route, target_x)
        return

    crossing_to_right = cur_x < MIDLINE_BLOCK_X < target_x
    crossing_to_left = cur_x > MIDLINE_BLOCK_X > target_x

    if not crossing_to_right and not crossing_to_left:
        append_horizontal(route, target_x)
        return

    if crossing_to_right:
        append_horizontal(route, 0.0)
        append_vertical(route, MIDLINE_DETOUR_Y)
        append_horizontal(route, 1.0)
        if prefer_fewer_turns:
            # Keep rightward crossing on detour lane to avoid an extra up/down turn.
            append_horizontal(route, target_x)
            return
        append_vertical(route, 0.0)
        append_horizontal(route, target_x)
        return

    append_horizontal(route, 1.0)
    append_vertical(route, MIDLINE_DETOUR_Y)
    append_horizontal(route, 0.0)
    append_vertical(route, 0.0)
    append_horizontal(route, target_x)


def route_turn_count(route: list[tuple[float, float]]) -> int:
    if len(route) <= 2:
        return 0

    dirs: list[tuple[int, int]] = []
    for i in range(1, len(route)):
        dx = route[i][0] - route[i - 1][0]
        dy = route[i][1] - route[i - 1][1]
        if abs(dx) <= 1e-6 and abs(dy) <= 1e-6:
            continue
        if abs(dx) >= abs(dy):
            dirs.append((1 if dx > 0.0 else -1, 0))
        else:
            dirs.append((0, 1 if dy > 0.0 else -1))

    if len(dirs) <= 1:
        return 0

    turns = 0
    prev = dirs[0]
    for d in dirs[1:]:
        if d != prev:
            turns += 1
        prev = d
    return turns


def route_axis_length(route: list[tuple[float, float]]) -> float:
    total = 0.0
    for i in range(1, len(route)):
        dx = abs(route[i][0] - route[i - 1][0])
        dy = abs(route[i][1] - route[i - 1][1])
        total += (dx + dy)
    return total


def choose_best_route(candidates: list[list[tuple[float, float]]]) -> list[tuple[float, float]]:
    if not candidates:
        return []
    return min(
        candidates,
        key=lambda r: (route_turn_count(r), route_axis_length(r), len(r)),
    )


def classify_side(x: float) -> str:
    if x <= -1.2:
        return 'left'
    if x >= 1.2:
        return 'right'
    return 'center'


def build_static_route(
    robot_id: int,
    start: tuple[float, float],
    goal: tuple[float, float],
) -> list[tuple[float, float]]:
    sx, sy = float(start[0]), float(start[1])
    gx, gy = float(goal[0]), float(goal[1])

    def new_route() -> list[tuple[float, float]]:
        r: list[tuple[float, float]] = []
        append_point(r, sx, sy)
        return r

    def finalize(r: list[tuple[float, float]]) -> list[tuple[float, float]]:
        append_point(r, gx, gy)
        return r

    s_side = classify_side(sx)
    g_side = classify_side(gx)
    transit_y = TRANSIT_Y[robot_id]
    right_gate_x = RIGHT_GATE_X_BY_ROBOT.get(robot_id, RIGHT_GATE_X)
    right_gate_to_goal = min(right_gate_x, gx)
    right_gate_from_start = min(right_gate_x, sx)

    if s_side == 'left' and g_side == 'left':
        candidates: list[list[tuple[float, float]]] = []

        # Legacy corridor style: keep away from world edge.
        r_gate = new_route()
        append_horizontal(r_gate, LEFT_GATE_X)
        append_vertical(r_gate, gy)
        append_horizontal(r_gate, gx)
        candidates.append(finalize(r_gate))

        # Lower-turn variants on the same side.
        r_yx = new_route()
        append_vertical(r_yx, gy)
        append_horizontal(r_yx, gx)
        candidates.append(finalize(r_yx))

        r_xy = new_route()
        append_horizontal(r_xy, gx)
        append_vertical(r_xy, gy)
        candidates.append(finalize(r_xy))

        return choose_best_route(candidates)
    elif s_side == 'right' and g_side == 'right':
        candidates = []

        r_yx = new_route()
        append_vertical(r_yx, gy)
        append_horizontal(r_yx, gx)
        candidates.append(finalize(r_yx))

        r_xy = new_route()
        append_horizontal(r_xy, gx)
        append_vertical(r_xy, gy)
        candidates.append(finalize(r_xy))

        return choose_best_route(candidates)
    elif s_side == 'left' and g_side == 'right':
        route = new_route()
        append_horizontal(route, LEFT_GATE_X)
        append_cross_corridor(route, right_gate_to_goal, transit_y, prefer_fewer_turns=True)
        append_vertical(route, gy)
        append_horizontal(route, gx)
        return finalize(route)
    elif s_side == 'right' and g_side == 'left':
        route = new_route()
        append_horizontal(route, right_gate_from_start)
        append_cross_corridor(route, LEFT_GATE_X, transit_y)
        append_vertical(route, gy)
        append_horizontal(route, gx)
        return finalize(route)
    else:
        # Fallback for center-side starts/goals.
        candidates = []

        mid_gate = LEFT_GATE_X if gx < 0.0 else min(RIGHT_GATE_X, gx)
        r_gate = new_route()
        append_horizontal(r_gate, mid_gate)
        append_vertical(r_gate, gy)
        append_horizontal(r_gate, gx)
        candidates.append(finalize(r_gate))

        r_yx = new_route()
        append_vertical(r_yx, gy)
        append_horizontal(r_yx, gx)
        candidates.append(finalize(r_yx))

        r_xy = new_route()
        append_horizontal(r_xy, gx)
        append_vertical(r_xy, gy)
        candidates.append(finalize(r_xy))

        return choose_best_route(candidates)


def route_cells(route: list[tuple[float, float]]) -> set[tuple[float, float]]:
    cells: set[tuple[float, float]] = set()
    if not route:
        return cells

    def cell_snap(v: float) -> float:
        return round(v / GRID_STEP) * GRID_STEP

    cells.add((cell_snap(route[0][0]), cell_snap(route[0][1])))
    for i in range(1, len(route)):
        x0, y0 = route[i - 1]
        x1, y1 = route[i]
        seg_len = max(abs(x1 - x0), abs(y1 - y0))
        steps = max(1, int(math.ceil(seg_len / GRID_STEP)))
        for k in range(steps + 1):
            t = k / steps
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            cells.add((cell_snap(x), cell_snap(y)))
    return cells


def overlap_summary(routes: dict[int, list[tuple[float, float]]]) -> str:
    if len(routes) < 2:
        return 'n/a'

    cell_map = {rid: route_cells(path) for rid, path in routes.items()}
    parts: list[str] = []
    ids = sorted(routes.keys())
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            ra = ids[i]
            rb = ids[j]
            overlap = len(cell_map[ra].intersection(cell_map[rb]))
            parts.append(f'r{ra}-r{rb}:{overlap}')
    return ', '.join(parts)


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
        if motion.phase == 'target' and motion.stop is not None:
            owners[motion.stop.resource_key] = rid
    return owners


def main() -> int:
    args = parse_args()

    ws_dir = os.path.dirname(os.path.abspath(__file__))
    schedule_path = args.schedule
    if not os.path.isabs(schedule_path):
        schedule_path = os.path.join(ws_dir, schedule_path)

    go_to_script = args.go_to_script
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
    print('[INFO] route mode: static corridors (predefined, no online overlap waits)')
    for rid in (1, 2, 3):
        st = states[rid]
        print(f'[INFO] {st.amr_name}: stops={len(st.stops)}, start={st.current}, transit_y={TRANSIT_Y[rid]:.1f}')

    stage_idx = 0
    resource_locks: dict[str, tuple[int, float]] = {}
    active: dict[int, ActiveMotion] = {}
    max_dispatch_rounds = max(0, int(args.max_stages))
    allow_new_rounds = True
    per_motion_timeout = max(10.0, float(args.exec_timeout))

    try:
        while True:
            now = time.time()
            release_expired_locks(resource_locks, now)
            progressed = False

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

                for rid in (1, 2, 3):
                    if rid in active:
                        continue

                    state = states[rid]
                    action = build_next_action(
                        state=state,
                        now=now,
                        resource_locks=resource_locks,
                        active_target_owners=owner_map,
                    )
                    if not action.move:
                        continue

                    route = build_static_route(rid, state.current, action.target)
                    round_plans.append((rid, action, route))

                    if action.phase == 'target' and action.stop is not None:
                        owner_map[action.stop.resource_key] = rid

                if round_plans:
                    progressed = True
                    stage_idx += 1
                    print(f'\n===== DISPATCH ROUND {stage_idx} =====')

                    round_routes = {rid: route for rid, _, route in round_plans}
                    for rid, action, route in round_plans:
                        if action.stop is not None:
                            print(
                                f'[MOVE] AMR{rid}: {action.phase} -> {action.stop.resource_key} '
                                f'target=({action.target[0]:.2f}, {action.target[1]:.2f})'
                            )
                        print(f'[ROUTE] AMR{rid}: points={len(route)} start={route[0]} end={route[-1]}')
                        if args.show_waypoints:
                            print(f'[ROUTE-WP] AMR{rid}: {serialize_waypoints(route)}')

                    if round_routes:
                        print(f'[ROUTE] overlap-cells: {overlap_summary(round_routes)}')

                    for rid, action, route in round_plans:
                        cmd = build_go_to_cmd(
                            go_to_script=go_to_script,
                            rid=rid,
                            phase=action.phase,
                            goal=action.target,
                            route=route,
                            stop=action.stop,
                            stop_pos_tol=args.stop_pos_tol,
                            stop_yaw_tol_deg=args.stop_yaw_tol_deg,
                            stop_yaw_settle_sec=args.stop_yaw_settle_sec,
                            station_yaw_deg=args.station_yaw_deg,
                        )
                        print(f"[EXEC] launch robot{rid}: cmd={' '.join(cmd)}")

                        if args.dry_run:
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
