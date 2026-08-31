#!/usr/bin/env bash
# Container entrypoint: source ROS, the ros_gz_sim overlay, and the built
# workspace, then hand off to whatever command was given.
#
# Sourcing happens here rather than in the Dockerfile because each `docker run`
# starts a fresh process that does not inherit a sourced shell.

set -e

source /opt/ros/humble/setup.bash
source /deps/install/setup.bash
source /workspace/install/setup.bash

cd /workspace
exec "$@"
