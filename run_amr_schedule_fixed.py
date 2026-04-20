#!/usr/bin/env python3
import argparse
from dataclasses import dataclass
import json
import math
import os
import re
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Execute AMR schedule with fixed approach routes and synchronized non-collision stages.'
    )
    parser.add_argument(
        '--schedule',
        type=str,
        default='amr_schedule_1.jsonl',
        help='jsonl schedule path',
    )
    parser.add_argument(
        '--planner-script',
        type=str,
        default='plan_three_non_overlap.py',
        help='path to plan_three_non_overlap.py',
    )
    parser.add_argument(
        '--exec-timeout',
        type=float,
        default=240.0,
        help='per-stage per-robot timeout passed to planner execution',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='print stage plan only, do not execute motion',
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


def build_stage_actions(
    states: dict[int, RobotState],
    now: float,
    resource_locks: dict[str, tuple[int, float]],
) -> dict[int, StageAction]:
    actions: dict[int, StageAction] = {}

    for rid in (1, 2, 3):
        state = states[rid]
        advance_zero_move(state, now, resource_locks)

        if state.done():
            actions[rid] = StageAction(
                robot_id=rid,
                move=False,
                phase='hold',
                target=state.current,
                stop=None,
                reason='done',
            )
            continue

        if now < state.ready_time:
            actions[rid] = StageAction(
                robot_id=rid,
                move=False,
                phase='hold',
                target=state.current,
                stop=None,
                reason=f'dwell until {state.ready_time:.1f}',
            )
            continue

        stop = state.stops[state.stop_index]
        if state.phase == 0:
            actions[rid] = StageAction(
                robot_id=rid,
                move=True,
                phase='approach',
                target=stop.approach,
                stop=stop,
                reason='approach for heading',
            )
        else:
            if lock_available(resource_locks, stop.resource_key, rid, now):
                actions[rid] = StageAction(
                    robot_id=rid,
                    move=True,
                    phase='target',
                    target=stop.target,
                    stop=stop,
                    reason='execute stop',
                )
            else:
                actions[rid] = StageAction(
                    robot_id=rid,
                    move=False,
                    phase='hold',
                    target=state.current,
                    stop=None,
                    reason=f'waiting resource {stop.resource_key}',
                )

    # If more than one robot tries to enter the same target resource in this stage,
    # keep only one mover to avoid simultaneous occupation.
    by_resource: dict[str, list[int]] = {}
    for rid, act in actions.items():
        if act.move and act.phase == 'target' and act.stop is not None:
            by_resource.setdefault(act.stop.resource_key, []).append(rid)

    for resource_key, contenders in by_resource.items():
        if len(contenders) <= 1:
            continue
        contenders.sort()
        winner = contenders[0]
        for rid in contenders[1:]:
            st = states[rid]
            actions[rid] = StageAction(
                robot_id=rid,
                move=False,
                phase='hold',
                target=st.current,
                stop=None,
                reason=f'waiting same-stage contention {resource_key}',
            )
        print(f'[LOCK] resource {resource_key}: allow AMR{winner} first, others hold')

    return actions


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


def run_plan_three_stage(
    planner_script: str,
    starts: dict[int, tuple[float, float]],
    goals: dict[int, tuple[float, float]],
    exec_timeout: float,
    dry_run: bool,
) -> int:
    cmd = [
        sys.executable,
        planner_script,
        '--x1', f'{goals[1][0]:.3f}', '--y1', f'{goals[1][1]:.3f}',
        '--x2', f'{goals[2][0]:.3f}', '--y2', f'{goals[2][1]:.3f}',
        '--x3', f'{goals[3][0]:.3f}', '--y3', f'{goals[3][1]:.3f}',
        '--s1x', f'{starts[1][0]:.3f}', '--s1y', f'{starts[1][1]:.3f}',
        '--s2x', f'{starts[2][0]:.3f}', '--s2y', f'{starts[2][1]:.3f}',
        '--s3x', f'{starts[3][0]:.3f}', '--s3y', f'{starts[3][1]:.3f}',
        '--exec-timeout', f'{exec_timeout:.1f}',
        '--overlap-prep-time', '0.0',
        '--no-priority-yield',
    ]
    print('[STAGE CMD] ' + ' '.join(cmd))
    if dry_run:
        return 0
    completed = subprocess.run(cmd, check=False)
    return completed.returncode


def main() -> int:
    args = parse_args()

    ws_dir = os.path.dirname(os.path.abspath(__file__))
    schedule_path = args.schedule
    if not os.path.isabs(schedule_path):
        schedule_path = os.path.join(ws_dir, schedule_path)

    planner_script = args.planner_script
    if not os.path.isabs(planner_script):
        planner_script = os.path.join(ws_dir, planner_script)
    if not os.path.isfile(planner_script):
        print(f'[ERROR] planner script not found: {planner_script}', file=sys.stderr)
        return 1

    try:
        states = load_schedule(schedule_path)
    except Exception as e:
        print(f'[ERROR] failed to parse schedule: {e}', file=sys.stderr)
        return 1

    print(f'[INFO] loaded schedule: {schedule_path}')
    for rid in (1, 2, 3):
        st = states[rid]
        print(f'[INFO] {st.amr_name}: stops={len(st.stops)}, start={st.current}')

    stage_idx = 0
    resource_locks: dict[str, tuple[int, float]] = {}

    try:
        while True:
            now = time.time()
            release_expired_locks(resource_locks, now)

            if all(states[rid].done() for rid in (1, 2, 3)):
                print('[OK] all AMR jobs completed')
                return 0

            actions = build_stage_actions(states, now, resource_locks)
            moving = [rid for rid in (1, 2, 3) if actions[rid].move]

            if not moving:
                wake = next_ready_time(states, now)
                if wake is None:
                    print('[WARN] no movable AMR and no wake time, exiting')
                    return 2
                sleep_sec = max(0.05, min(0.50, wake - now))
                time.sleep(sleep_sec)
                continue

            stage_idx += 1
            print(f'\n===== STAGE {stage_idx} =====')
            goals = {}
            for rid in (1, 2, 3):
                act = actions[rid]
                goals[rid] = act.target
                if act.move and act.stop is not None:
                    print(
                        f'[MOVE] AMR{rid}: {act.phase} -> {act.stop.resource_key} '
                        f'target=({act.target[0]:.2f}, {act.target[1]:.2f})'
                    )
                else:
                    print(f'[HOLD] AMR{rid}: {act.reason}')

            rc = run_plan_three_stage(
                planner_script=planner_script,
                starts={rid: states[rid].current for rid in (1, 2, 3)},
                goals=goals,
                exec_timeout=max(10.0, float(args.exec_timeout)),
                dry_run=bool(args.dry_run),
            )
            if rc != 0:
                print(f'[ERROR] stage {stage_idx} failed, returncode={rc}', file=sys.stderr)
                return rc

            stage_end = time.time()
            for rid in moving:
                state = states[rid]
                act = actions[rid]
                state.current = act.target
                stop = act.stop
                if act.phase == 'approach':
                    state.phase = 1
                    continue

                if stop is None:
                    continue

                state.phase = 0
                state.stop_index += 1
                state.ready_time = stage_end + stop.dwell_sec
                if stop.dwell_sec > 0.0:
                    resource_locks[stop.resource_key] = (rid, state.ready_time)

                print(
                    f'[ARRIVE] AMR{rid}: {stop.resource_key} '
                    f'face {stop.face_label}, dwell {stop.dwell_sec:.1f}s'
                )

    except KeyboardInterrupt:
        print('\n[WARN] interrupted by user')
        return 130


if __name__ == '__main__':
    sys.exit(main())
