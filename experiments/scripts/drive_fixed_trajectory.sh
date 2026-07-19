#!/usr/bin/env bash
# Fixed trajectory for the benchmark. Deterministic: same commands, same
# durations, every run. Drives a closed-ish loop inside the 6x6 arena.
set -e

pub() {  # pub <linear_x> <angular_z> <seconds>
  ros2 topic pub --rate 10 --times $(( $3 * 10 )) /cmd_vel \
    geometry_msgs/msg/Twist "{linear: {x: $1}, angular: {z: $2}}" > /dev/null
}

stop() {
  ros2 topic pub --rate 10 --times 10 /cmd_vel \
    geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" > /dev/null
}

echo "Driving fixed trajectory..."
pub 0.25 0.0  6      # straight
pub 0.0  0.5  3      # turn
pub 0.25 0.0  6      # straight
pub 0.0  0.5  3      # turn
pub 0.25 0.0  6      # straight
pub 0.0  0.5  3      # turn
pub 0.25 0.0  6      # straight
stop
echo "Trajectory complete."