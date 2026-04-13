import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MoveDistanceNode(Node):
    def __init__(self, robot_id: int, target_distance: float, speed: float):
        super().__init__(f'move_distance_node_{robot_id}')

        cmd_topic = f'/rosmaster_x3_{robot_id}/cmd_vel'
        odom_topic = f'/rosmaster_x3_{robot_id}/mecanum_drive_controller/odom'

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 50)

        self.target_distance = abs(target_distance)
        self.speed = abs(speed)
        self.direction = 1.0 if target_distance >= 0.0 else -1.0

        self.start_x = None
        self.start_y = None
        self.reached = False
        self.last_print_distance = -1.0

        self.get_logger().info(
            f'start: robot={robot_id}, target={target_distance:.3f} m, speed={self.speed:.3f} m/s, '
            f'cmd_topic={cmd_topic}, odom_topic={odom_topic}'
        )

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        if self.reached:
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = current_x
            self.start_y = current_y
            self.get_logger().info(f'origin: x={self.start_x:.3f}, y={self.start_y:.3f}')
            return

        distance = math.sqrt((current_x - self.start_x) ** 2 + (current_y - self.start_y) ** 2)

        if distance < self.target_distance:
            cmd = Twist()
            cmd.linear.x = self.direction * self.speed
            self.cmd_pub.publish(cmd)

            if (distance - self.last_print_distance) >= 0.10:
                self.last_print_distance = distance
                self.get_logger().info(f'moving: {distance:.2f}/{self.target_distance:.2f} m')
        else:
            self.stop_robot()
            self.reached = True
            self.get_logger().info(f'done: reached {distance:.2f} m, robot stopped')


def parse_args():
    parser = argparse.ArgumentParser(description='Move a robot forward/backward by distance without keyboard.')
    parser.add_argument('--robot', type=int, default=1, choices=[1, 2, 3], help='robot id: 1, 2, or 3')
    parser.add_argument('--distance', type=float, default=1.0, help='target distance in meters (negative = backward)')
    parser.add_argument('--speed', type=float, default=0.2, help='linear speed in m/s (positive value)')
    parser.add_argument('--timeout', type=float, default=30.0, help='max run time in seconds before auto-stop')
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = MoveDistanceNode(args.robot, args.distance, args.speed)

    try:
        t0 = time.time()
        while rclpy.ok() and not node.reached:
            if (time.time() - t0) > args.timeout:
                node.get_logger().warn(f'timeout: no completion within {args.timeout:.1f}s, stopping robot')
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('interrupted by user')
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()