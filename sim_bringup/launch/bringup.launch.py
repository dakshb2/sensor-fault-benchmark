"""Bringup: Gazebo + robot + bridge + fault injector + EKF.

Topic flow:
    gz sensors --> /imu/raw, /wheel/odometry/raw   (RAW: injection point)
                       |
                 [fault_injector]                  (always present)
                       v
                   /imu, /wheel/odometry           (what the EKF consumes)
                       v
                   EKF --> /odometry/filtered  +  odom->base_link TF

The fault injector sits in the path for BOTH clean and faulted runs. With no
fault arguments it is a pure passthrough, so clean and faulted trials differ
only by the fault itself and not by which program handles the data.

    ros2 launch sim_bringup bringup.launch.py                         # clean
    ros2 launch sim_bringup bringup.launch.py imu_fault:=yaw_bias:0.2 # faulted

Fault timing is anchored to /trial/started, published by the trajectory driver
when it begins driving, so faults land at the same point in the path every run.

/ground_truth/odometry is published for scoring ONLY. Nothing in the estimation
path -- including the fault injector -- subscribes to it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sim_bringup')
    experiments_share = get_package_share_directory('experiments')

    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    faults_file = os.path.join(experiments_share, 'faults', 'faults.yaml')

    imu_fault = LaunchConfiguration('imu_fault')
    wheel_fault = LaunchConfiguration('wheel_fault')

    declare_imu_fault = DeclareLaunchArgument(
        'imu_fault', default_value='',
        description="IMU fault, e.g. 'yaw_bias:0.2'. Empty means clean.")
    declare_wheel_fault = DeclareLaunchArgument(
        'wheel_fault', default_value='',
        description="Wheel fault, e.g. 'dropout'. Empty means clean.")

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

    # Replaces the Stage 1 identity relays. Passes both sensor streams through
    # untouched unless a fault is specified.
    fault_injector = Node(
        package='fault_injector',
        executable='injector',
        name='fault_injector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'faults_file': faults_file,
            'imu_fault': imu_fault,
            'wheel_fault': wheel_fault,
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    return LaunchDescription([
        declare_imu_fault,
        declare_wheel_fault,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        fault_injector,
        ekf,
    ])


def _read_xacro(path):
    import subprocess
    return subprocess.check_output(['xacro', path]).decode()