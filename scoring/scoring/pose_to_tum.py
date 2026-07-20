#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PoseToTum(Node):
    def __init__(self):
        super().__init__('pose_to_tum')
        self.declare_parameter('est_topic', '/odometry/filtered')
        self.declare_parameter('ref_topic', '/ground_truth/odometry')
        self.declare_parameter('est_file', 'est.tum')
        self.declare_parameter('ref_file', 'ref.tum')
        self.declare_parameter('duration', 0.0)

        self._duration = self.get_parameter('duration').value
        self._start_time = None

        self._est_file = open(self.get_parameter('est_file').value, 'w')
        self._ref_file = open(self.get_parameter('ref_file').value, 'w')

        self.create_subscription(
            Odometry, self.get_parameter('est_topic').value,
            self._writer(self._est_file), 50)
        self.create_subscription(
            Odometry, self.get_parameter('ref_topic').value,
            self._writer(self._ref_file), 50)

        if self._duration > 0.0:
            self.create_timer(0.1, self._check_duration)
            self.get_logger().info(
                f'Recording for {self._duration} simulation seconds.')
        else:
            self.get_logger().info('Recording until interrupted.')

    def _check_duration(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if now <= 0.0:
            return

        if self._start_time is None:
            self._start_time = now
            return

        if now - self._start_time >= self._duration:
            self.get_logger().info('Recording duration reached.')
            raise SystemExit

    def _writer(self, handle):
        def callback(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            handle.write(
                f"{stamp:.9f} {position.x:.6f} {position.y:.6f} {position.z:.6f} "
                f"{orientation.x:.6f} {orientation.y:.6f} "
                f"{orientation.z:.6f} {orientation.w:.6f}\n")
        return callback

    def destroy_node(self):
        self._est_file.close()
        self._ref_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PoseToTum()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()