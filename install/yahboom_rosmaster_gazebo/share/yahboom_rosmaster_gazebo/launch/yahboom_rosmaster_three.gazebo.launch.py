#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def make_robot_group(pkg_share_description, robot_name, x, y, z='0.05', controller_delay=8.0, enable_rgbd='false'):
    controller_params = os.path.join(
        pkg_share_description, 'config', robot_name, 'my_ros2_controllers.yaml')

    joint_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        namespace=robot_name,
        arguments=[
            'joint_state_broadcaster',
            '-c', f'/{robot_name}/controller_manager',
            '--param-file', controller_params,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '120',
            '--switch-timeout', '30'
        ]
    )

    mecanum_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        namespace=robot_name,
        arguments=[
            'mecanum_drive_controller',
            '-c', f'/{robot_name}/controller_manager',
            '--param-file', controller_params,
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '120',
            '--switch-timeout', '30'
        ]
    )

    cmd_vel_relay = Node(
        package='yahboom_rosmaster_navigation',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen'
    )

    return GroupAction([
        PushRosNamespace(robot_name),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')
            ]),
            launch_arguments={
                'robot_name': robot_name,
                'use_sim_time': 'true',
                'use_gazebo': 'true',
                'use_namespace': 'true',
                'enable_rgbd': enable_rgbd,
                'jsp_gui': 'false',
                'use_jsp': 'false',
                'use_rviz': 'false'
            }.items()
        ),
        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', robot_name,
                '-allow_renaming', 'false',
                '-x', str(x),
                '-y', str(y),
                '-z', str(z)
            ]
        ),
        cmd_vel_relay,
        # Delay controller loading until robot model and ros2_control are fully ready.
        # A longer stagger helps avoid transient service-response loss under multi-robot load.
        TimerAction(
            period=controller_delay,
            actions=[
                joint_spawner,
                TimerAction(period=3.0, actions=[mecanum_spawner])
            ]
        )
    ])


def generate_launch_description():
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share_gazebo = FindPackageShare(package='yahboom_rosmaster_gazebo').find('yahboom_rosmaster_gazebo')
    pkg_share_description = FindPackageShare(package='yahboom_rosmaster_description').find('yahboom_rosmaster_description')

    world_path = os.path.join(pkg_share_gazebo, 'worlds', 'temp.world')
    bridge_config = os.path.join(pkg_share_gazebo, 'config', 'ros_gz_bridge.yaml')
    models_path = os.path.join(pkg_share_gazebo, 'models')
    headless = LaunchConfiguration('headless')
    enable_rgbd = LaunchConfiguration('enable_rgbd')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo server only when true (no GUI)'))
    ld.add_action(DeclareLaunchArgument(
        'enable_rgbd',
        default_value='false',
        choices=['true', 'false'],
        description='Enable RGBD camera sensors for each robot'))

    ld.add_action(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v 4 {world_path}'}.items()
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g '}.items(),
        condition=UnlessCondition(headless)
    ))

    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'))

    ld.add_action(make_robot_group(
        pkg_share_description, 'rosmaster_x3_1', -2.5, -1.5,
        controller_delay=8.0, enable_rgbd=enable_rgbd))
    ld.add_action(TimerAction(period=9.0, actions=[make_robot_group(
        pkg_share_description, 'rosmaster_x3_2', -2.5, 0.0,
        controller_delay=9.0, enable_rgbd=enable_rgbd)]))
    ld.add_action(TimerAction(period=18.0, actions=[make_robot_group(
        pkg_share_description, 'rosmaster_x3_3', -2.5, 1.5,
        controller_delay=10.0, enable_rgbd=enable_rgbd)]))

    return ld
