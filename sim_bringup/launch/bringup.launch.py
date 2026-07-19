"""Stage 1 bringup: Gazebo + robot + bridge + raw->clean relays + EKF.

Topic flow:
    gz sensors --> /imu/raw, /wheel/odometry/raw   (RAW: Stage-2 injection point)
                       |
                    relay (identity passthrough, Stage 1 only)
                       v
                   /imu, /wheel/odometry           (CLEAN: what the EKF consumes)
                       v
                   EKF --> /odometry/filtered  +  odom->base_link TF

/ground_truth/odometry is published for scoring ONLY. Nothing in the estimation
path subscribes to it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sim_bringup')

    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '3', world_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': _read_xacro(xacro_file),
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'diff_robot',
            '-topic', 'robot_description',
            '-z', '0.1',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': bridge_config,
        }],
    )

    # Stage 1 only: identity passthrough. Stage 2 deletes these two relays and
    # drops the fault_injector into the same slot. The EKF config never changes.
    relay_imu = Node(
        package='topic_tools',
        executable='relay',
        name='relay_imu',
        output='screen',
        arguments=['/imu/raw', '/imu'],
        parameters=[{'use_sim_time': True}],
    )

    relay_wheel = Node(
        package='topic_tools',
        executable='relay',
        name='relay_wheel',
        output='screen',
        arguments=['/wheel/odometry/raw', '/wheel/odometry'],
        parameters=[{'use_sim_time': True}],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        relay_imu,
        relay_wheel,
        ekf,
    ])


def _read_xacro(path):
    import subprocess
    return subprocess.check_output(['xacro', path]).decode()