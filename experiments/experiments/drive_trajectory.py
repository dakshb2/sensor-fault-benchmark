#!/usr/bin/env python3
"""Fixed trajectory driver, loaded from a YAML file.

Durations are SIMULATION seconds. With use_sim_time the node's clock is
Gazebo's, so the robot covers an identical path on any hardware regardless of
real-time factor. That is what makes runs comparable.

A trajectory file lists segments:
  segments:
    - {linear_x: 0.25, angular_z: 0.0, duration: 6.0}
"""

import sys

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PUBLISH_PERIOD = 0.05
STOP_HOLD = 1.0


class TrajectoryDriver(Node):
    def __init__(self):
        super().__init__('trajectory_driver')
        self.declare_parameter('trajectory', '')
        trajectory_path = self.get_parameter('trajectory').value

        if not trajectory_path:
            self.get_logger().error('No trajectory file given (-p trajectory:=<path>).')
            raise SystemExit(1)

        self._segments = self._load(trajectory_path)
        self._publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._index = 0
        self._segment_start = None
        self._finished = False
        self._stop_start = None
        self.create_timer(PUBLISH_PERIOD, self._tick)

    def _load(self, path):
        with open(path, 'r') as handle:
            data = yaml.safe_load(handle)
        segments = []
        for entry in data['segments']:
            segments.append((
                float(entry['linear_x']),
                float(entry['angular_z']),
                float(entry['duration']),
            ))
        total = sum(seg[2] for seg in segments)
        self.get_logger().info(
            f"Loaded '{data.get('name', path)}': "
            f"{len(segments)} segments, {total:.1f} s total.")
        return segments

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, linear_x, angular_z):
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self._publisher.publish(message)

    def _tick(self):
        now = self._now()
        if now <= 0.0:
            return

        if self._finished:
            self._publish(0.0, 0.0)
            if now - self._stop_start > STOP_HOLD:
                self.get_logger().info('Trajectory complete.')
                raise SystemExit
            return

        if self._segment_start is None:
            self._segment_start = now
            self.get_logger().info('Starting trajectory (simulation time).')

        linear_x, angular_z, duration = self._segments[self._index]

        if now - self._segment_start >= duration:
            self._index = self._index + 1
            self._segment_start = now
            if self._index >= len(self._segments):
                self._finished = True
                self._stop_start = now
                return
            linear_x, angular_z, duration = self._segments[self._index]

        self._publish(linear_x, angular_z)


def main():
    rclpy.init()
    try:
        node = TrajectoryDriver()
    except SystemExit as exit_signal:
        rclpy.shutdown()
        sys.exit(exit_signal.code if exit_signal.code else 0)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()