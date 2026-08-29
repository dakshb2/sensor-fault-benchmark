# Adding your own estimator

The benchmark ships with a `robot_localization` EKF as its reference estimator,
but nothing about the harness is specific to it. Any node that satisfies the
interface below can be scored the same way, against the same trajectories and
the same faults.

## The contract

Your estimator must:

**1. Subscribe to the clean sensor topics.**

| topic | type | rate |
|-------|------|------|
| `/imu` | `sensor_msgs/msg/Imu` | ~100 Hz |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | ~50 Hz |

These are the outputs of the fault injector. On a clean run they are byte-identical
to the simulator's raw streams; on a faulted run they are corrupted. Your
estimator cannot tell which, and should not try to -- that is the point.

**2. Publish its pose estimate.**

| topic | type |
|-------|------|
| `/odometry/filtered` | `nav_msgs/msg/Odometry` |

The scoring node records this topic. Only the position (`pose.pose.position`)
is used for ATE; the full message is recorded so orientation-aware metrics can
be added later.

**3. Broadcast the `odom -> base_link` transform.**

Nothing else in the graph publishes this transform -- the simulator's own
odometry TF is deliberately redirected to an unbridged topic so the estimator
owns it alone. If your estimator does not broadcast it, the TF tree will be
broken, though scoring itself will still work.

**4. Use simulation time.**

Run with `use_sim_time: true` and stamp your output with the sim clock. Scoring
matches estimate and ground-truth poses by their message timestamps, so a node
running on wall-clock will not associate correctly.

**5. Never subscribe to ground truth.**

`/ground_truth/odometry` exists for scoring only. An estimator that reads it is
not being measured -- it is being handed the answer. The `scoring` package is the
only package in the workspace that subscribes to it, and keeping it that way is
what makes results between different estimators comparable.

## Swapping it in

The reference EKF is launched from `sim_bringup/launch/bringup.launch.py`:

```python
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )
```

Replace that node with your own, keeping it in the same position in the returned
`LaunchDescription`. Everything else -- the simulator, the bridge, the fault
injector, the trajectory driver, the scorer -- is unchanged.

## Running

Once your node is in the launch file, the normal commands apply:

```bash
colcon build --symlink-install
source install/setup.bash

# clean baseline
./experiments/scripts/run_trial.sh experiments/trajectories/box.yaml mine_clean

# under a fault
./experiments/scripts/run_trial.sh experiments/trajectories/figure_eight.yaml \
    mine_yawbias --imu yaw_bias:0.2

# the full matrix
./experiments/scripts/scorecard.sh
```

## Comparing fairly

Two things are worth doing before comparing your estimator's numbers to the
reference EKF's:

**Re-measure the noise floor.** The floors quoted in the README (box 25%,
straight 22%, figure_eight 1.0%) are properties of the reference EKF, not of the
simulator. A different estimator will have a different run-to-run spread, and the
significance thresholds in the scorecard depend on it. Run
`experiments/scripts/noise_floor.sh` after five or more clean runs per
trajectory.

**Re-pin the baselines.** `experiments/baselines/` holds one clean reference run
per trajectory for the reference EKF. Those are not meaningful for a different
estimator; record your own.

## A note on what is being measured

This benchmark scores an estimator in dead-reckoning mode: there is no map frame
and no global corrector, so error accumulates without bound. That is deliberate
-- a corrector would absorb the drift the benchmark exists to measure. If your
estimator normally runs with a global correction step, expect its numbers here to
look worse than they would in its usual configuration, and interpret them as a
measure of the dead-reckoning core rather than of the full system.