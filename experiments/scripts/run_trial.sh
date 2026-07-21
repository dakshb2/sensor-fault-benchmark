#!/usr/bin/env bash
# One-command benchmark trial.
#   ./run_trial.sh <trajectory.yaml> <output_prefix>
# Runs teardown -> launch -> origin check -> record -> drive -> score -> teardown,
# refusing to proceed if any integrity gate fails.

WS=~/sensor-fault-benchmark

# Source ROS with nounset OFF — its setup scripts reference unset variables.
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

# Now enable the rest of strict mode for our own logic.
set -eo pipefail

TRAJ="${1:?usage: run_trial.sh <trajectory.yaml> <output_prefix>}"
PREFIX="${2:?usage: run_trial.sh <trajectory.yaml> <output_prefix>}"

EST="$WS/results/${PREFIX}_est.tum"
REF="$WS/results/${PREFIX}_ref.tum"
mkdir -p "$WS/results"

teardown() {
  pkill -9 -f "ign gazebo"        2>/dev/null || true
  pkill -9 -f ign-gazebo-server   2>/dev/null || true
  pkill -9 -f ign-gazebo-gui      2>/dev/null || true
  pkill -9 -f parameter_bridge    2>/dev/null || true
  pkill -9 -f robot_state_publisher 2>/dev/null || true
  pkill -9 -f ekf_node            2>/dev/null || true
  pkill -9 -f "topic_tools/relay" 2>/dev/null || true
  pkill -9 -f pose_to_tum         2>/dev/null || true
  pkill -9 -f drive_trajectory    2>/dev/null || true
  sleep 3
}

# --- 1. clean slate ---
echo "[1/7] Teardown."
set +e   # teardown commands (pkill, grep -v, daemon) return non-zero as normal operation
for attempt in 1 2 3 4 5; do
  teardown
  ros2 daemon stop  >/dev/null 2>&1
  ros2 daemon start >/dev/null 2>&1
  n=$(ros2 node list 2>/dev/null | grep -v "transform_listener" | wc -l)
  if [ "$n" -eq 0 ]; then break; fi
  echo "       teardown attempt $attempt: $n node(s) remain, retrying..."
  sleep 3
  if [ "$attempt" -eq 5 ]; then
    echo "FAIL: graph not empty after 5 teardown attempts:"; ros2 node list
    set -e
    exit 1
  fi
done
set -e   # restore strict mode

# ensure we always clean up, even on error or Ctrl+C
trap teardown EXIT

# --- 2. duration from the trajectory file ---
DURATION=$(python3 -c "
import yaml
d = yaml.safe_load(open('$TRAJ'))
print(sum(float(s['duration']) for s in d['segments']) + 4.0)
")
echo "[2/7] Trajectory '$TRAJ' -> record window ${DURATION}s (segments + 4s margin)."

# --- 3. launch bringup in background ---
echo "[3/7] Launching sim."
ros2 launch sim_bringup bringup.launch.py > /tmp/bringup.log 2>&1 &

# --- 4. wait for the graph to come up ---
echo "[4/7] Waiting for /ground_truth/odometry..."
for i in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -q "/ground_truth/odometry"; then break; fi
  sleep 1
  if [ "$i" -eq 60 ]; then echo "FAIL: sim did not come up (see /tmp/bringup.log)"; exit 1; fi
done
sleep 3  # let the robot settle

# --- 5. origin gate ---
echo "[5/7] Verifying robot at origin."
X=$(ros2 topic echo /ground_truth/odometry --once --field pose.pose.position.x 2>/dev/null | head -1)
if python3 -c "import sys; sys.exit(0 if abs(float('$X')) < 0.01 else 1)"; then
  echo "       origin OK (x=$X)"
else
  echo "FAIL: robot not at origin (x=$X). Aborting."; exit 1
fi

# --- 6. record + drive ---
echo "[6/7] Recording (${DURATION}s) and driving."
ros2 run scoring pose_to_tum --ros-args \
  -p use_sim_time:=true -p duration:="$DURATION" \
  -p est_file:="$EST" -p ref_file:="$REF" > /tmp/recorder.log 2>&1 &
RECORDER_PID=$!
sleep 1
ros2 run experiments drive_trajectory --ros-args \
  -p use_sim_time:=true -p trajectory:="$TRAJ" > /tmp/driver.log 2>&1
wait "$RECORDER_PID"

# --- 7. score ---
echo "[7/7] Scoring."
evo_ape tum "$REF" "$EST" --t_max_diff 0.02 | grep -E "rmse|min|mean"
echo "Files: $EST"
echo "       $REF"