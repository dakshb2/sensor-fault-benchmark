#!/usr/bin/env python3
"""Unified sensor fault injector.

Sits between the raw sensor streams and the clean topics the estimator reads:

    /imu/raw            --> [this node] --> /imu
    /wheel/odometry/raw --> [this node] --> /wheel/odometry

With no faults specified it is a pure passthrough, so it is present in the graph
for BOTH clean and faulted runs. Clean and faulted trials therefore differ only
by the fault itself, not by which program sits in the data path.

Fault timing is anchored to /trial/started, published once by the trajectory
driver when it begins driving. Times in faults.yaml are simulation seconds
measured from that instant, so a fault lands at the same point in the path on
every run regardless of how long the simulator took to come up.

INTEGRITY: this node reads only the raw sensor streams, its fault
specification, and the trial-start signal. It never subscribes to ground truth.
Corruption depends only on the incoming message and elapsed trial time, never on
where the robot actually is.
"""

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


class FaultSpec:
    """One parsed fault: what it does, how strong, and when it is active."""

    def __init__(self, sensor, fault_type, magnitude, start, duration):
        self.sensor = sensor
        self.fault_type = fault_type
        self.magnitude = magnitude
        self.start = start
        self.duration = duration
        self._announced_active = False
        self._announced_done = False

    def is_active(self, elapsed):
        """True if the fault should be applied at this elapsed trial time."""
        if elapsed is None or elapsed < self.start:
            return False
        if self.duration is None:
            return True
        return elapsed < (self.start + self.duration)

    def window_text(self):
        if self.duration is None:
            return f'from {self.start}s to end of run'
        return f'from {self.start}s for {self.duration}s'

    def __repr__(self):
        return (f'{self.sensor}:{self.fault_type} '
                f'magnitude={self.magnitude} {self.window_text()}')


class FaultInjector(Node):
    def __init__(self):
        super().__init__('fault_injector')

        self.declare_parameter('faults_file', '')
        self.declare_parameter('imu_fault', '')     # e.g. 'yaw_bias:0.2'
        self.declare_parameter('wheel_fault', '')   # e.g. 'dropout'

        faults_file = self.get_parameter('faults_file').value
        if not faults_file:
            self.get_logger().error(
                'No faults_file given (-p faults_file:=<path to faults.yaml>).')
            raise SystemExit(1)

        with open(faults_file, 'r') as handle:
            self._definitions = yaml.safe_load(handle)

        self._trial_start = None      # sim time when /trial/started arrived
        self._last_wheel_msg = None   # for wheel freeze fault
        self._warned_no_start = False

        self._imu_fault = self._parse_fault(
            'imu', self.get_parameter('imu_fault').value)
        self._wheel_fault = self._parse_fault(
            'wheel', self.get_parameter('wheel_fault').value)

        imu_def = self._definitions['imu']
        wheel_def = self._definitions['wheel']

        self._imu_publisher = self.create_publisher(
            Imu, imu_def['topic_out'], 50)
        self._wheel_publisher = self.create_publisher(
            Odometry, wheel_def['topic_out'], 50)

        self.create_subscription(
            Imu, imu_def['topic_raw'], self._on_imu, 50)
        self.create_subscription(
            Odometry, wheel_def['topic_raw'], self._on_wheel, 50)

        # Latched so a late-starting injector still receives a signal that was
        # already published.
        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Empty, '/trial/started', self._on_trial_started, latched)

        active = [f for f in (self._imu_fault, self._wheel_fault) if f]
        if active:
            for fault in active:
                self.get_logger().info(f'Fault armed: {fault}')
        else:
            self.get_logger().info('No faults: pure passthrough (clean run).')

    # ---------- setup helpers ----------

    def _parse_fault(self, sensor, spec):
        """Parse a fault spec string into a FaultSpec, or None if empty.

        Grammar:  <type>[:<magnitude>][@<start>[:<duration>]]
        Examples:
            dropout                 type only; magnitude + timing from file
            yaw_bias:0.2            magnitude 0.2; timing from file
            freeze@12:15            timing start 12s, duration 15s
            yaw_bias:0.2@8:10       magnitude 0.2, start 8s, duration 10s
            drift:0.01@4            magnitude 0.01, start 4s, duration from file

        Splitting on '@' first keeps the magnitude colon and the duration
        colon on opposite sides, so they never collide.
        """
        spec = (spec or '').strip()
        if not spec:
            return None

        fault_part, _, timing_part = spec.partition('@')

        if ':' in fault_part:
            fault_type, _, raw_magnitude = fault_part.partition(':')
            try:
                magnitude = float(raw_magnitude)
            except ValueError:
                self.get_logger().error(
                    f"Bad magnitude '{raw_magnitude}' in {sensor} fault "
                    f"'{spec}'. Expected a number.")
                raise SystemExit(1)
        else:
            fault_type, magnitude = fault_part, None

        sensor_definitions = self._definitions.get(sensor, {})
        if fault_type not in sensor_definitions:
            available = [k for k in sensor_definitions
                         if k not in ('topic_raw', 'topic_out')]
            self.get_logger().error(
                f"Unknown {sensor} fault '{fault_type}'. "
                f"Defined: {available or 'none'}")
            raise SystemExit(1)

        definition = sensor_definitions[fault_type]

        if magnitude is None:
            magnitude = definition.get('default_magnitude')
            if magnitude is not None:
                self.get_logger().info(
                    f'{sensor}:{fault_type} using default magnitude '
                    f'{magnitude}')

        start = float(definition.get('start', 0.0))
        duration = (None if definition.get('duration') is None
                    else float(definition['duration']))

        if timing_part:
            raw_start, sep, raw_duration = timing_part.partition(':')
            try:
                start = float(raw_start)
            except ValueError:
                self.get_logger().error(
                    f"Bad start time '{raw_start}' in {sensor} fault "
                    f"'{spec}'. Expected a number.")
                raise SystemExit(1)
            if sep:
                try:
                    duration = float(raw_duration)
                except ValueError:
                    self.get_logger().error(
                        f"Bad duration '{raw_duration}' in {sensor} fault "
                        f"'{spec}'. Expected a number.")
                    raise SystemExit(1)
            self.get_logger().info(
                f'{sensor}:{fault_type} timing overridden: '
                f'start={start}s duration={duration}s')

        return FaultSpec(
            sensor=sensor,
            fault_type=fault_type,
            magnitude=magnitude,
            start=start,
            duration=duration,
        )

    # ---------- timing ----------

    def _on_trial_started(self, _msg):
        if self._trial_start is None:
            self._trial_start = self._now()
            self.get_logger().info(
                f'Trial start signal received at sim t={self._trial_start:.2f}s. '
                f'Fault clocks running.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self):
        """Seconds since the trajectory began, or None if it has not."""
        if self._trial_start is None:
            return None
        return self._now() - self._trial_start

    def _should_apply(self, fault):
        """Whether to corrupt right now, with one-time activation logging."""
        if fault is None:
            return False

        elapsed = self._elapsed()
        if elapsed is None:
            # Faults are armed but the trial has not started. Fail safe: pass
            # data through untouched, and say so once.
            if not self._warned_no_start:
                self.get_logger().warn(
                    'Faults armed but /trial/started not yet received; '
                    'passing data through unmodified.')
                self._warned_no_start = True
            return False

        active = fault.is_active(elapsed)

        if active and not fault._announced_active:
            self.get_logger().info(
                f'FAULT ACTIVE at t={elapsed:.2f}s: {fault}')
            fault._announced_active = True
        elif (not active and fault._announced_active
              and not fault._announced_done):
            self.get_logger().info(
                f'FAULT ENDED at t={elapsed:.2f}s: '
                f'{fault.sensor}:{fault.fault_type}')
            fault._announced_done = True

        return active

    # ---------- message handling ----------

    def _on_imu(self, msg):
        if self._should_apply(self._imu_fault):
            fault = self._imu_fault
            if fault.fault_type == 'yaw_bias':
                msg.angular_velocity.z = (
                    msg.angular_velocity.z + fault.magnitude)
        self._imu_publisher.publish(msg)

    def _on_wheel(self, msg):
        fault = self._wheel_fault
        if self._should_apply(fault):
            if fault.fault_type == 'dropout':
                return  # withhold entirely
            if fault.fault_type == 'freeze':
                if self._last_wheel_msg is not None:
                    frozen = self._last_wheel_msg
                    # re-stamp so the message looks current: a hung sensor
                    # keeps emitting fresh timestamps carrying stale data.
                    frozen.header.stamp = self.get_clock().now().to_msg()
                    self._wheel_publisher.publish(frozen)
                return
        # normal passthrough — also remember this as the last good message
        self._last_wheel_msg = msg
        self._wheel_publisher.publish(msg)


def main():
    rclpy.init()
    try:
        node = FaultInjector()
    except SystemExit as exit_signal:
        rclpy.shutdown()
        raise SystemExit(exit_signal.code if exit_signal.code else 1)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
       