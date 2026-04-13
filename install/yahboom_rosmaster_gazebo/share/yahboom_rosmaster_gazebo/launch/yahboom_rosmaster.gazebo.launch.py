#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share_gazebo = FindPackageShare(package='yahboom_rosmaster_gazebo').find('yahboom_rosmaster_gazebo')
    pkg_share_description = FindPackageShare(package='yahboom_rosmaster_description').find('yahboom_rosmaster_description')

    bridge_config = os.path.join(pkg_share_gazebo, 'config', 'ros_gz_bridge.yaml')
    models_path = os.path.join(pkg_share_gazebo, 'models')

    headless = LaunchConfiguration('headless')
    enable_rgbd = LaunchConfiguration('enable_rgbd')
    world_file = LaunchConfiguration('world_file')
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    world_path = PathJoinSubstitution([pkg_share_gazebo, 'worlds', world_file])


    robot_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')]),
            launch_arguments={
                'robot_name': robot_name,
                'use_sim_time': use_sim_time,
                'use_gazebo': 'true',
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
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', roll,
                '-P', pitch,
                '-Y', yaw,
            ]
        ),
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['joint_state_broadcaster', '-c', 'controller_manager']
        ),
        Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['mecanum_drive_controller', '-c', 'controller_manager']
        )
    ])

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo server only when true (no GUI)'))
    ld.add_action(DeclareLaunchArgument(
        'enable_rgbd',
        default_value='false',
        choices=['true', 'false'],
        description='Enable RGBD camera in the spawned robot model'))
    ld.add_action(DeclareLaunchArgument(
        'world_file',
        default_value='temp.world',
        description='World file name in yahboom_rosmaster_gazebo/worlds'))
    ld.add_action(DeclareLaunchArgument(
        'robot_name',
        default_value='rosmaster_x3',
        description='Robot entity name in Gazebo'))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='x component of initial position, meters'))
    ld.add_action(DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='y component of initial position, meters'))
    ld.add_action(DeclareLaunchArgument(
        'z',
        default_value='0.05',
        description='z component of initial position, meters'))
    ld.add_action(DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians'))
    ld.add_action(DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians'))
    ld.add_action(DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='yaw angle of initial orientation, radians'))

    ld.add_action(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v 4 ', world_path]}.items()
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g '}.items(),
        condition=UnlessCondition(headless)
    ))
    ld.add_action(Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'config_file': bridge_config}], output='screen'))
    ld.add_action(robot_group)

    return ld
