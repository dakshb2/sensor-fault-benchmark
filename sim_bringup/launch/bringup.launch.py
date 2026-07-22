"""Bringup: Gazebo + robot + bridge + IMU path (relay or fault injector) + EKF.

Topic flow:
    gz sensors --> /imu/raw, /wheel/odometry/raw   (RAW: injection point)
                       |
             clean run: identity relay  /  faulted run: fault_injector
                       v
                   /imu, /wheel/odometry           (what the EKF consumes)
                       v
                   EKF --> /odometry/filtered  +  odom->base_link TF

The IMU path is selected by the 'fault_type' launch argument:
    fault_type:=none      -> relay_imu passes /imu/raw through unchanged (default)
    fault_type:=imu_bias  -> imu_bias_injector adds a yaw-rate bias
Exactly one runs; the EKF always reads /imu and cannot tell the difference.

/ground_truth/odometry is published for scoring ONLY. Nothing in the estimation
path — including the fault injector — subscribes to it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    pkg_share = get_package_share_directory('sim_bringup')

    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    fault_type = LaunchConfiguration('fault_type')
    imu_yaw_bias = LaunchConfiguration('imu_yaw_bias')

    declare_fault_type = DeclareLaunchArgument(
        'fault_type', default_value='none',
        description="'none' for clean run, 'imu_bias' to inject IMU yaw-rate bias")
    declare_imu_yaw_bias = DeclareLaunchArgument(
        'imu_yaw_bias', default_value='0.0',
        description='yaw-rate bias in rad/s when fault_type=imu_bias')

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

    imu_is_clean = PythonExpression(["'", fault_type, "' != 'imu_bias'"])
    imu_is_faulted = PythonExpression(["'", fault_type, "' == 'imu_bias'"])

    relay_imu = Node(
        package='topic_tools',
        executable='relay',
        name='relay_imu',
        output='screen',
        arguments=['/imu/raw', '/imu'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(imu_is_clean),
    )

    imu_bias_injector = Node(
        package='fault_injector', executable='imu_bias_injector',
        name='imu_bias_injector', output='screen',
        parameters=[{
            'use_sim_time': True,
            'in_topic': '/imu/raw',
            'out_topic': '/imu',
            'yaw_rate_bias': imu_yaw_bias,
        }],
        condition=IfCondition(imu_is_faulted),
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
        declare_fault_type,
        declare_imu_yaw_bias,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        relay_imu,
        imu_bias_injector,
        relay_wheel,
        ekf,
    ])


def _read_xacro(path):
    import subprocess
    return subprocess.check_output(['xacro', path]).decode()