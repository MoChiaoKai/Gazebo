#!/usr/bin/env bash
set -eo pipefail

WORLD_NAME="${1:-default}"

python3 - "$WORLD_NAME" <<'PY'
import math
import re
import subprocess
import sys


def parse_scalar(block: str, key: str, default: float = 0.0) -> float:
    m = re.search(rf"\b{re.escape(key)}:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", block)
    return float(m.group(1)) if m else default


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


world_name = sys.argv[1]
topic = f"/world/{world_name}/pose/info"

try:
    output = subprocess.check_output(
        ["ign", "topic", "-e", "-t", topic, "-n", "1"],
        stderr=subprocess.STDOUT,
        text=True,
        timeout=6.0,
    )
except FileNotFoundError:
    print("[ERROR] ign command not found")
    sys.exit(1)
except subprocess.TimeoutExpired:
    print(f"[ERROR] timeout waiting pose data on {topic}")
    sys.exit(1)
except subprocess.CalledProcessError as e:
    print(f"[ERROR] failed to read {topic}")
    print(e.output.strip())
    sys.exit(1)

poses = {}
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
    if name not in {"rosmaster_x3_1", "rosmaster_x3_2", "rosmaster_x3_3"}:
        continue

    m_pos = re.search(r"position\s*\{([^}]*)\}", block, re.S)
    m_ori = re.search(r"orientation\s*\{([^}]*)\}", block, re.S)
    pos_block = m_pos.group(1) if m_pos else ""
    ori_block = m_ori.group(1) if m_ori else ""

    x = parse_scalar(pos_block, "x", 0.0)
    y = parse_scalar(pos_block, "y", 0.0)
    z = parse_scalar(pos_block, "z", 0.0)

    qx = parse_scalar(ori_block, "x", 0.0)
    qy = parse_scalar(ori_block, "y", 0.0)
    qz = parse_scalar(ori_block, "z", 0.0)
    qw = parse_scalar(ori_block, "w", 1.0)
    yaw = yaw_from_quaternion(qx, qy, qz, qw)

    poses[name] = (x, y, z, yaw)

for robot in ["rosmaster_x3_1", "rosmaster_x3_2", "rosmaster_x3_3"]:
    if robot not in poses:
        print(f"{robot}: not found in {topic}")
        continue
    x, y, z, yaw = poses[robot]
    print(
        f"{robot}: x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
        f"yaw={yaw:.3f} rad ({math.degrees(yaw):.1f} deg)"
    )
PY
