#!/usr/bin/env python3
import argparse
import math
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


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


def read_robot_world_pose(robot_name: str, world_name: str, timeout_sec: float) -> tuple[float, float, float]:
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
        detail = e.output.strip() if e.output else 'unknown error'
        raise RuntimeError(f'failed to read {topic}: {detail}') from e

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
        if not m_name or m_name.group(1) != robot_name:
            continue

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

        return x, y, yaw

    raise RuntimeError(f'{robot_name} not found in {topic}')


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
        target_yaw_deg: float | None,
        pos_tol: float,
        yaw_tol_deg: float,
        max_linear: float,
        max_angular: float,
        k_linear: float,
        k_angular: float,
        k_yaw: float,
        target_frame: str,
    ):
        super().__init__(f'go_to_point_node_{robot_id}')

        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = None if target_yaw_deg is None else math.radians(target_yaw_deg)

        self.pos_tol = abs(pos_tol)
        self.yaw_tol = math.radians(abs(yaw_tol_deg))
        self.max_linear = abs(max_linear)
        self.max_angular = abs(max_angular)
        self.k_linear = abs(k_linear)
        self.k_angular = abs(k_angular)
        self.k_yaw = abs(k_yaw)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False
        self.done = False
        self._last_log_time = 0.0

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

    def set_target_odom(self, target_x: float, target_y: float, target_yaw_rad: float | None):
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = target_yaw_rad
        self.done = False
        self.get_logger().info(
            f'odom target: ({self.target_x:.3f}, {self.target_y:.3f}), '
            f"yaw={'none' if self.target_yaw is None else f'{math.degrees(self.target_yaw):.1f} deg'}"
        )

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_ready = True

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_step(self):
        if self.done or not self.odom_ready:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.hypot(dx, dy)

        cmd = Twist()
        done_now = False

        if distance > self.pos_tol:
            heading = math.atan2(dy, dx)
            heading_error = normalize_angle(heading - self.current_yaw)

            # If heading error is large, rotate first; otherwise drive and correct simultaneously.
            if abs(heading_error) > 0.60:
                linear = 0.0
            else:
                linear = self.k_linear * distance * max(0.0, math.cos(heading_error))

            angular = self.k_angular * heading_error

            cmd.linear.x = clamp(linear, -self.max_linear, self.max_linear)
            cmd.angular.z = clamp(angular, -self.max_angular, self.max_angular)
        elif self.target_yaw is not None:
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            if abs(yaw_error) > self.yaw_tol:
                cmd.angular.z = clamp(self.k_yaw * yaw_error, -self.max_angular, self.max_angular)
            else:
                done_now = True
        else:
            done_now = True

        if done_now:
            self.done = True
            self.stop_robot()
            self.get_logger().info(
                f'done: final=({self.current_x:.3f}, {self.current_y:.3f}), '
                f'yaw={math.degrees(self.current_yaw):.1f} deg'
            )
            return

        self.cmd_pub.publish(cmd)

        now = time.time()
        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self.get_logger().info(
                f'moving: cur=({self.current_x:.2f}, {self.current_y:.2f}) '
                f'goal=({self.target_x:.2f}, {self.target_y:.2f}) err={distance:.2f} m'
            )


def parse_args():
    parser = argparse.ArgumentParser(description='Go to a target in odom or Gazebo world frame without keyboard.')
    parser.add_argument('--robot', type=int, default=1, choices=[1, 2, 3], help='robot id: 1, 2, or 3')
    parser.add_argument('--frame', choices=['odom', 'world'], default='odom', help='input target frame')
    parser.add_argument('--x', type=float, required=True, help='target x in selected frame (meters)')
    parser.add_argument('--y', type=float, required=True, help='target y in selected frame (meters)')
    parser.add_argument('--yaw', type=float, default=None, help='target yaw in selected frame (degrees, optional)')
    parser.add_argument('--world-name', type=str, default='default', help='Gazebo world name for world frame')
    parser.add_argument('--ign-timeout', type=float, default=4.0, help='timeout for ign pose query (seconds)')
    parser.add_argument('--timeout', type=float, default=60.0, help='max run time in seconds')
    parser.add_argument('--pos-tol', type=float, default=0.05, help='position tolerance in meters')
    parser.add_argument('--yaw-tol', type=float, default=5.0, help='yaw tolerance in degrees')
    parser.add_argument('--max-linear', type=float, default=0.20, help='max linear speed m/s')
    parser.add_argument('--max-angular', type=float, default=1.00, help='max angular speed rad/s')
    parser.add_argument('--k-linear', type=float, default=0.9, help='linear proportional gain')
    parser.add_argument('--k-angular', type=float, default=2.4, help='heading proportional gain')
    parser.add_argument('--k-yaw', type=float, default=2.0, help='final yaw proportional gain')
    parser.add_argument('--rate', type=float, default=20.0, help='control loop rate in Hz')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    exit_code = 0
    robot_name = f'rosmaster_x3_{args.robot}'

    node = GoToPointNode(
        robot_id=args.robot,
        target_x=args.x,
        target_y=args.y,
        target_yaw_deg=args.yaw,
        pos_tol=args.pos_tol,
        yaw_tol_deg=args.yaw_tol,
        max_linear=args.max_linear,
        max_angular=args.max_angular,
        k_linear=args.k_linear,
        k_angular=args.k_angular,
        k_yaw=args.k_yaw,
        target_frame=args.frame,
    )

    dt = 1.0 / max(args.rate, 1.0)
    t0 = time.time()

    try:
        odom_wait_deadline = time.time() + max(3.0, args.ign_timeout + 2.0)
        while rclpy.ok() and not node.odom_ready and time.time() < odom_wait_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if not node.odom_ready:
            node.get_logger().error('no odom received, cannot navigate')
            exit_code = 1
            return

        if args.frame == 'world':
            world_x_now, world_y_now, world_yaw_now = read_robot_world_pose(robot_name, args.world_name, args.ign_timeout)

            odom_x_now = node.current_x
            odom_y_now = node.current_y
            odom_yaw_now = node.current_yaw

            yaw_world_from_odom = normalize_angle(world_yaw_now - odom_yaw_now)
            c = math.cos(yaw_world_from_odom)
            s = math.sin(yaw_world_from_odom)
            tx_world = world_x_now - (c * odom_x_now - s * odom_y_now)
            ty_world = world_y_now - (s * odom_x_now + c * odom_y_now)

            target_odom_x, target_odom_y = world_to_odom(
                args.x,
                args.y,
                tx_world,
                ty_world,
                yaw_world_from_odom,
            )

            target_odom_yaw = None
            if args.yaw is not None:
                target_odom_yaw = normalize_angle(math.radians(args.yaw) - yaw_world_from_odom)

            node.get_logger().info(
                f'world->odom calibrated: world_now=({world_x_now:.3f}, {world_y_now:.3f}), '
                f'odom_now=({odom_x_now:.3f}, {odom_y_now:.3f}), '
                f'dyaw={math.degrees(yaw_world_from_odom):.2f} deg'
            )
            node.set_target_odom(target_odom_x, target_odom_y, target_odom_yaw)

        while rclpy.ok() and not node.done:
            if node.odom_ready and (time.time() - t0) > args.timeout:
                node.get_logger().warn(f'timeout: no completion within {args.timeout:.1f}s, stopping robot')
                break

            rclpy.spin_once(node, timeout_sec=dt)
            node.control_step()

    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('interrupted by user')
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