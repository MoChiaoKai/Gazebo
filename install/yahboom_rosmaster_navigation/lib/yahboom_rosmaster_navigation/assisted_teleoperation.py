#!/usr/bin/env python3
"""
Assisted Teleop Node for ROS 2 Navigation

This script implements an Assisted Teleop node that interfaces with the Nav2 stack.
It manages the lifecycle of the AssistedTeleop action, handles cancellation requests,
and periodically clears costmaps to remove temporary obstacles.

Subscription Topics:
    /cmd_vel_teleop (geometry_msgs/Twist): Velocity commands for assisted teleop
    /cancel_assisted_teleop (std_msgs/Bool): Cancellation requests for assisted teleop

Parameters:
    ~/costmap_clear_frequency (double): Frequency in Hz for costmap clearing. Default: 2.0

:author: Addison Sears-Collins
:date: December 5, 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator
from rcl_interfaces.msg import ParameterDescriptor


class AssistedTeleopNode(Node):
    """
    A ROS 2 node for managing Assisted Teleop functionality.
    """

    def __init__(self):
        """Initialize the AssistedTeleopNode."""
        super().__init__('assisted_teleop_node')

        # Declare and get parameters
        self.declare_parameter(
            'costmap_clear_frequency',
            2.0,
            ParameterDescriptor(description='Frequency in Hz for costmap clearing')
        )
        clear_frequency = self.get_parameter('costmap_clear_frequency').value

        # Initialize the BasicNavigator for interfacing with Nav2
        self.navigator = BasicNavigator('assisted_teleop_navigator')
        self.nav_ready = False

        # Create subscribers for velocity commands and cancellation requests
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.cmd_vel_callback, 10)
        self.cancel_sub = self.create_subscription(
            Bool, '/cancel_assisted_teleop', self.cancel_callback, 10)

        # Initialize state variables
        self.assisted_teleop_active = False
        self.cancellation_requested = False

        # Create a timer for periodic costmap clearing with configurable frequency
        period = 1.0 / clear_frequency
        self.clear_costmaps_timer = self.create_timer(period, self.clear_costmaps_callback)

        self.get_logger().info(f'Assisted Teleop Node initialized with costmap clearing frequency: {clear_frequency} Hz')

    def ensure_nav2_active(self) -> bool:
        """Wait for Nav2 only when assisted teleop is first requested."""
        if self.nav_ready:
            return True
        if not rclpy.ok():
            return False

        self.get_logger().info('Waiting for Nav2 to become active...')
        try:
            self.navigator.waitUntilNav2Active()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f'Nav2 activation skipped: {exc}')
            return False

        self.nav_ready = True
        return True

    def cmd_vel_callback(self, twist_msg: Twist) -> None:
        """Process incoming velocity commands and activate assisted teleop if needed."""
        # Reset cancellation flag when new velocity commands are received
        if (abs(twist_msg.linear.x) > 0.0 or
            abs(twist_msg.linear.y) > 0.0 or
                abs(twist_msg.angular.z) > 0.0):
            self.cancellation_requested = False

        if not self.assisted_teleop_active and not self.cancellation_requested:
            if (abs(twist_msg.linear.x) > 0.0 or
                abs(twist_msg.linear.y) > 0.0 or
                    abs(twist_msg.angular.z) > 0.0):
                self.start_assisted_teleop()

    def start_assisted_teleop(self) -> None:
        """Start the Assisted Teleop action with indefinite duration."""
        if not self.ensure_nav2_active():
            self.get_logger().warning('Nav2 is not active yet, assisted teleop request ignored')
            return

        self.assisted_teleop_active = True
        self.cancellation_requested = False
        self.navigator.assistedTeleop(time_allowance=0)  # 0 means indefinite duration
        self.get_logger().info('AssistedTeleop activated with indefinite duration')

    def cancel_callback(self, msg: Bool) -> None:
        """Handle cancellation requests for assisted teleop."""
        if msg.data and self.assisted_teleop_active and not self.cancellation_requested:
            self.cancel_assisted_teleop()

    def cancel_assisted_teleop(self) -> None:
        """Cancel the currently running Assisted Teleop action."""
        if self.assisted_teleop_active:
            self.navigator.cancelTask()
            self.assisted_teleop_active = False
            self.cancellation_requested = True
            self.get_logger().info('AssistedTeleop cancelled')

    def clear_costmaps_callback(self) -> None:
        """Periodically clear all costmaps to remove temporary obstacles."""
        if not self.assisted_teleop_active:
            return

        self.navigator.clearAllCostmaps()
        self.get_logger().debug('Costmaps cleared')


def main():
    """Initialize and run the AssistedTeleopNode."""
    rclpy.init()
    node = None

    try:
        node = AssistedTeleopNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Node shutting down due to keyboard interrupt')
    except ROSInterruptException:
        if node:
            node.get_logger().info('Node shutting down due to ROS interrupt')
    finally:
        if node:
            try:
                node.cancel_assisted_teleop()
            except Exception:  # pylint: disable=broad-except
                pass
            try:
                if node.nav_ready:
                    node.navigator.lifecycleShutdown()
            except Exception:  # pylint: disable=broad-except
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # pylint: disable=broad-except
            pass


if __name__ == '__main__':
    main()
