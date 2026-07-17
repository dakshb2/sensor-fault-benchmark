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

        self._est_file = open(self.get_parameter('est_file').value, 'w')
        self._ref_file = open(self.get_parameter('ref_file').value, 'w')

        self.create_subscription(
            Odometry, self.get_parameter('est_topic').value,
            self._writer(self._est_file), 50)
        self.create_subscription(
            Odometry, self.get_parameter('ref_topic').value,
            self._writer(self._ref_file), 50)

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()