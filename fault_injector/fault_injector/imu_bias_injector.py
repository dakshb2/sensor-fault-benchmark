#!/usr/bin/env python3
"""Constant IMU yaw-rate bias fault.

Reads the raw IMU stream, adds a fixed offset to angular_velocity.z, and
republishes on the clean topic the estimator consumes. This node replaces the
Stage 1 identity relay for /imu.

INTEGRITY: this node reads only the raw sensor stream and a fixed bias value. It
never reads ground truth. The corruption depends only on the incoming message,
not on where the robot actually is.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuBiasInjector(Node):
    def __init__(self):
        super().__init__('imu_bias_injector')
        self.declare_parameter('in_topic', '/imu/raw')
        self.declare_parameter('out_topic', '/imu')
        self.declare_parameter('yaw_rate_bias', 0.0)  # rad/s added to gyro z

        self._bias = self.get_parameter('yaw_rate_bias').value
        out_topic = self.get_parameter('out_topic').value
        in_topic = self.get_parameter('in_topic').value

        self._publisher = self.create_publisher(Imu, out_topic, 50)
        self.create_subscription(Imu, in_topic, self._on_imu, 50)

        self.get_logger().info(
            f'IMU yaw-rate bias fault: {self._bias} rad/s '
            f'({in_topic} -> {out_topic})')

    def _on_imu(self, msg):
        msg.angular_velocity.z = msg.angular_velocity.z + self._bias
        self._publisher.publish(msg)


def main():
    rclpy.init()
    node = ImuBiasInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()