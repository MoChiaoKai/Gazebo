#!/usr/bin/env python3
"""
Frontier-based autonomous exploration for SLAM mapping.

This node subscribes to /map, extracts frontier cells (free cells adjacent to
unknown cells), and continuously sends navigation goals to expand map coverage.
"""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class FrontierExplorer(Node):
    """Autonomous frontier exploration node."""

    def __init__(self):
        super().__init__('frontier_explorer')

        self.map_topic = self.declare_parameter('map_topic', '/map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.plan_period = float(self.declare_parameter('plan_period', 2.0).value)
        self.sample_step = int(self.declare_parameter('sample_step', 3).value)
        self.min_goal_distance = float(self.declare_parameter('min_goal_distance', 0.8).value)
        self.max_goal_distance = float(self.declare_parameter('max_goal_distance', 5.0).value)
        self.frontier_standoff = float(self.declare_parameter('frontier_standoff', 0.8).value)
        self.boundary_margin_m = float(self.declare_parameter('boundary_margin_m', 0.35).value)
        self.goal_clearance_cells = int(self.declare_parameter('goal_clearance_cells', 2).value)
        self.occupied_threshold = int(self.declare_parameter('occupied_threshold', 50).value)
        self.blacklist_radius = float(self.declare_parameter('blacklist_radius', 0.6).value)
        self.visited_radius = float(self.declare_parameter('visited_radius', 0.8).value)
        self.goal_timeout = float(self.declare_parameter('goal_timeout', 75.0).value)
        self.no_frontier_cycles_to_stop = int(self.declare_parameter('no_frontier_cycles_to_stop', 8).value)
        self.cluster_bin_size = float(self.declare_parameter('cluster_bin_size', 0.8).value)
        self.cluster_weight = float(self.declare_parameter('cluster_weight', 0.4).value)
        self.distance_penalty = float(self.declare_parameter('distance_penalty', 1.3).value)
        self.direction_weight = float(self.declare_parameter('direction_weight', 0.6).value)
        self.max_jump_from_last = float(self.declare_parameter('max_jump_from_last', 1.5).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.navigator = BasicNavigator()
        self._wait_for_navigation_server()
        self.get_logger().info('Navigation action server is ready. Frontier exploration started.')

        self.map_msg: Optional[OccupancyGrid] = None
        self.goal_sent_time_ns: Optional[int] = None
        self.active_goal: Optional[Tuple[float, float]] = None
        self.blacklist: List[Tuple[float, float]] = []
        self.visited: List[Tuple[float, float]] = []
        self.last_reached_goal: Optional[Tuple[float, float]] = None
        self.prev_reached_goal: Optional[Tuple[float, float]] = None
        self.no_frontier_cycles = 0

        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 10)
        self.create_timer(self.plan_period, self._tick)

    def _wait_for_navigation_server(self) -> None:
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        while rclpy.ok():
            if self.navigator.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                return
            self.get_logger().info('/navigate_to_pose not available yet, waiting...')

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _tick(self) -> None:
        if self.map_msg is None:
            return

        if self.active_goal is not None:
            self._monitor_goal()
            return

        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            return

        goal_xy = self._pick_frontier_goal(robot_xy)
        if goal_xy is None:
            self.no_frontier_cycles += 1
            if self.no_frontier_cycles >= self.no_frontier_cycles_to_stop:
                self.get_logger().info('No frontier found for several cycles. Exploration appears complete.')
            else:
                self.get_logger().info('No frontier found this cycle. Waiting for map updates...')
            return

        self.no_frontier_cycles = 0
        goal_pose = self._to_pose_stamped(goal_xy, robot_xy)
        self.navigator.goToPose(goal_pose)
        self.active_goal = goal_xy
        self.goal_sent_time_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f'Sent frontier goal: x={goal_xy[0]:.2f}, y={goal_xy[1]:.2f}')

    def _monitor_goal(self) -> None:
        if self.active_goal is None:
            return

        if self.goal_sent_time_ns is not None:
            elapsed = (self.get_clock().now().nanoseconds - self.goal_sent_time_ns) * 1e-9
            if elapsed > self.goal_timeout:
                self.get_logger().warn('Goal timeout reached. Cancelling and blacklisting this frontier.')
                self.navigator.cancelTask()
                self.blacklist.append(self.active_goal)
                self.active_goal = None
                self.goal_sent_time_ns = None
                return

        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Frontier goal reached.')
            if self.last_reached_goal is not None:
                self.prev_reached_goal = self.last_reached_goal
            self.last_reached_goal = self.active_goal
            # Mark reached frontiers so we don't keep selecting the same edge.
            self.visited.append(self.active_goal)
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Frontier goal canceled. Blacklisting this point.')
            self.blacklist.append(self.active_goal)
        elif result == TaskResult.FAILED:
            self.get_logger().warn('Frontier goal failed. Blacklisting this point.')
            self.blacklist.append(self.active_goal)
        else:
            self.get_logger().warn('Frontier goal returned invalid status. Blacklisting this point.')
            self.blacklist.append(self.active_goal)

        self.active_goal = None
        self.goal_sent_time_ns = None

    def _get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform('map', self.base_frame, Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException as exc:
            self.get_logger().warn(f'Waiting for map->{self.base_frame} transform: {exc}')
            return None

    def _pick_frontier_goal(self, robot_xy: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        msg = self.map_msg
        if msg is None:
            return None

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = msg.data

        candidate_goals: List[Tuple[Tuple[float, float], float]] = []

        # Skip boundaries because frontier checks look at neighbors.
        for y in range(1, height - 1, self.sample_step):
            for x in range(1, width - 1, self.sample_step):
                idx = y * width + x
                if data[idx] != 0:
                    continue

                if not self._is_frontier(data, width, idx):
                    continue

                wx = ox + (x + 0.5) * resolution
                wy = oy + (y + 0.5) * resolution

                goal_xy = self._make_safe_goal((wx, wy), robot_xy, msg, data)
                if goal_xy is None:
                    continue

                if self._is_blacklisted(goal_xy):
                    continue

                if self._is_visited(goal_xy):
                    continue

                dist = math.hypot(goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1])
                if dist < self.min_goal_distance or dist > self.max_goal_distance:
                    continue

                candidate_goals.append((goal_xy, dist))

        if not candidate_goals:
            return None

        bin_size = max(0.2, self.cluster_bin_size)
        clusters: Dict[Tuple[int, int], List[Tuple[Tuple[float, float], float]]] = {}
        for goal_xy, dist in candidate_goals:
            bx = int(math.floor(goal_xy[0] / bin_size))
            by = int(math.floor(goal_xy[1] / bin_size))
            clusters.setdefault((bx, by), []).append((goal_xy, dist))

        # Two-pass selection:
        # 1) Prefer local continuity around last reached goal.
        # 2) Relax continuity if local region is exhausted.
        passes = [True, False] if self.last_reached_goal is not None else [False]
        for require_local_jump in passes:
            best_goal = None
            best_score = -float('inf')

            for cluster_entries in clusters.values():
                centroid_x = sum(entry[0][0] for entry in cluster_entries) / len(cluster_entries)
                centroid_y = sum(entry[0][1] for entry in cluster_entries) / len(cluster_entries)
                cluster_goal = self._make_safe_goal((centroid_x, centroid_y), robot_xy, msg, data)
                if cluster_goal is None:
                    continue

                if self._is_blacklisted(cluster_goal):
                    continue

                if self._is_visited(cluster_goal):
                    continue

                dist = math.hypot(cluster_goal[0] - robot_xy[0], cluster_goal[1] - robot_xy[1])
                if dist < self.min_goal_distance or dist > self.max_goal_distance:
                    continue

                if require_local_jump and self.last_reached_goal is not None:
                    jump = math.hypot(
                        cluster_goal[0] - self.last_reached_goal[0],
                        cluster_goal[1] - self.last_reached_goal[1],
                    )
                    if jump > self.max_jump_from_last:
                        continue

                score = len(cluster_entries) * self.cluster_weight - self.distance_penalty * dist

                if self.prev_reached_goal is not None and self.last_reached_goal is not None:
                    last_vec_x = self.last_reached_goal[0] - self.prev_reached_goal[0]
                    last_vec_y = self.last_reached_goal[1] - self.prev_reached_goal[1]
                    cand_vec_x = cluster_goal[0] - self.last_reached_goal[0]
                    cand_vec_y = cluster_goal[1] - self.last_reached_goal[1]

                    last_norm = math.hypot(last_vec_x, last_vec_y)
                    cand_norm = math.hypot(cand_vec_x, cand_vec_y)
                    if last_norm > 1e-3 and cand_norm > 1e-3:
                        cos_angle = (last_vec_x * cand_vec_x + last_vec_y * cand_vec_y) / (last_norm * cand_norm)
                        cos_angle = max(-1.0, min(1.0, cos_angle))
                        score += self.direction_weight * cos_angle

                if score > best_score:
                    best_score = score
                    best_goal = cluster_goal

            if best_goal is not None:
                return best_goal

        # Fallback to nearest valid candidate if cluster centroids were filtered out.
        return min(candidate_goals, key=lambda item: item[1])[0]

    def _make_safe_goal(
        self,
        frontier_xy: Tuple[float, float],
        robot_xy: Tuple[float, float],
        msg: OccupancyGrid,
        data,
    ) -> Optional[Tuple[float, float]]:
        min_x, max_x, min_y, max_y = self._map_bounds(msg)

        vx = robot_xy[0] - frontier_xy[0]
        vy = robot_xy[1] - frontier_xy[1]
        norm = math.hypot(vx, vy)

        # Try larger standoff first, then smaller offsets as fallback.
        standoffs = [self.frontier_standoff, max(0.3, self.frontier_standoff * 0.5), 0.0]
        for standoff in standoffs:
            if norm > 1e-6:
                gx = frontier_xy[0] + (vx / norm) * standoff
                gy = frontier_xy[1] + (vy / norm) * standoff
            else:
                gx, gy = frontier_xy

            # Keep goals away from map borders to avoid off-grid recoveries.
            gx = min(max(gx, min_x + self.boundary_margin_m), max_x - self.boundary_margin_m)
            gy = min(max(gy, min_y + self.boundary_margin_m), max_y - self.boundary_margin_m)

            if self._is_free_goal_cell(gx, gy, msg, data):
                return (gx, gy)

        return None

    @staticmethod
    def _map_bounds(msg: OccupancyGrid) -> Tuple[float, float, float, float]:
        min_x = msg.info.origin.position.x
        min_y = msg.info.origin.position.y
        max_x = min_x + msg.info.width * msg.info.resolution
        max_y = min_y + msg.info.height * msg.info.resolution
        return (min_x, max_x, min_y, max_y)

    @staticmethod
    def _world_to_cell(x: float, y: float, msg: OccupancyGrid) -> Tuple[int, int]:
        cx = int((x - msg.info.origin.position.x) / msg.info.resolution)
        cy = int((y - msg.info.origin.position.y) / msg.info.resolution)
        return (cx, cy)

    def _is_free_goal_cell(self, x: float, y: float, msg: OccupancyGrid, data) -> bool:
        width = msg.info.width
        height = msg.info.height
        cx, cy = self._world_to_cell(x, y, msg)

        if cx < 0 or cy < 0 or cx >= width or cy >= height:
            return False

        # Require local clearance around the goal.
        for ny in range(cy - self.goal_clearance_cells, cy + self.goal_clearance_cells + 1):
            for nx in range(cx - self.goal_clearance_cells, cx + self.goal_clearance_cells + 1):
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    return False
                value = data[ny * width + nx]
                if value == -1:
                    return False
                if value >= self.occupied_threshold:
                    return False
        return True

    @staticmethod
    def _is_frontier(data, width: int, idx: int) -> bool:
        neighbors = (
            idx - 1,
            idx + 1,
            idx - width,
            idx + width,
            idx - width - 1,
            idx - width + 1,
            idx + width - 1,
            idx + width + 1,
        )
        return any(data[n] == -1 for n in neighbors)

    def _is_blacklisted(self, xy: Tuple[float, float]) -> bool:
        for bx, by in self.blacklist:
            if math.hypot(xy[0] - bx, xy[1] - by) <= self.blacklist_radius:
                return True
        return False

    def _is_visited(self, xy: Tuple[float, float]) -> bool:
        for vx, vy in self.visited:
            if math.hypot(xy[0] - vx, xy[1] - vy) <= self.visited_radius:
                return True
        return False

    @staticmethod
    def _to_pose_stamped(goal_xy: Tuple[float, float], robot_xy: Tuple[float, float]) -> PoseStamped:
        yaw = math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(goal_xy[0])
        pose.pose.position.y = float(goal_xy[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
