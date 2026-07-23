# sensor-fault-benchmark

A reproducible ROS 2 benchmark for evaluating how a localization estimator
responds to sensor faults. It runs a fixed simulated trajectory, records the
estimated and ground-truth poses, and computes standard trajectory-error metrics
(ATE/RPE via [evo](https://github.com/MichaelGrupp/evo)).

The Stage 1 clean-run harness is complete: simulation, EKF baseline, trajectory
runner, and scoring pipeline. Stage 2 is in progress -- the scenario-driven fault
injector works with scheduled IMU faults; additional fault types, a scorecard
sweep, and Docker packaging are still to come.

The goal is not a new fault-injection or fault-detection technique — both are
well established. The goal is to make estimator comparisons easier to reproduce
by fixing the simulation, trajectory, fault configuration, timing, and scoring
in one place.

## Integrity model

The package structure and topic wiring keep fault injection, estimation, and
scoring separate, so a future detection method cannot draw on information it
should not have:

- **Faults are injected into raw sensor streams** (`/imu/raw`,
  `/wheel/odometry/raw`). The injector republishes on the clean topics (`/imu`,
  `/wheel/odometry`) that the estimator reads. It is present in the path for both
  clean and faulted runs — a clean run is simply the passthrough case — so the
  two conditions differ only by the fault, not by which program handles the data.
  This was verified: a clean run through the injector scores within the noise
  floor of the earlier relay-based baseline.
- **Fault timing is anchored to `/trial/started`**, published by the trajectory
  driver when it begins driving, not to simulator startup. The gap between
  simulator start and trial start varies by several seconds between runs, so
  anchoring to the trial keeps a fault landing at the same point in the path
  every time.
- **Ground truth is consumed only by the `scoring` package.** The estimator, the
  fault injector, and any future detector never subscribe to it;
  `/ground_truth/*` appears in none of their configs. Corruption depends only on
  the incoming message and elapsed trial time, never on where the robot is.
- **The estimator owns `odom -> base_link` alone.** The simulator's odometry TF
  broadcast is redirected to an unbridged topic so nothing competes with, or
  masks, the estimator's transform.
- **A detection method (Stage 3) lives in a separate package** and is validated
  by running through this benchmark. It cannot access ground truth or the fault
  injector's state.

## Architecture

| package          | role                                          | ground truth access |
|------------------|-----------------------------------------------|---------------------|
| `sim_bringup`    | robot, world, sensor bridge, EKF              | none                |
| `scoring`        | ground-truth tap, ATE/RPE (TUM export + evo)  | yes — only here     |
| `experiments`    | trajectory library, one-command trial runner  | none                |
| `fault_injector` | corrupts raw sensor streams (scheduled)         | none                |
| `fdir`           | detect / isolate / recover — Stage 3 stub     | none                |

The estimator is a `robot_localization` EKF fusing wheel-odometry planar
velocities (vx, vyaw) with IMU yaw rate. It runs in dead-reckoning mode
(`world_frame: odom`, no map frame), which lets localization error accumulate
without correction. A global corrector would absorb the drift the benchmark is
meant to measure, so there deliberately is not one.

## Environment

- Ubuntu 22.04, ROS 2 Humble
- Gazebo Fortress (Ignition), CycloneDDS
- Python: `evo`, `numpy<1.25` (see Known quirks)

Developed and tested on Ubuntu 22.04 ARM64 in a UTM VM on Apple Silicon, at a
real-time factor of about 0.7. Scoring is anchored to simulation time rather than
wall-clock, which should make results less sensitive to host-machine speed;
this has not yet been verified across multiple machines.

## Quickstart

```bash
# build
cd ~/sensor-fault-benchmark
colcon build --symlink-install
source install/setup.bash

# clean trial: teardown -> launch -> record -> drive -> score
./experiments/scripts/run_trial.sh experiments/trajectories/box.yaml trial01

# faulted trial
./experiments/scripts/run_trial.sh experiments/trajectories/figure_eight.yaml \
    trial02 --imu yaw_bias:0.2
```

`run_trial.sh` runs the whole pipeline as one command and stops if an integrity
gate fails — the ROS graph not being empty after teardown, or the robot not being
at the origin at spawn. It prints the ATE at the end.

Trajectories live in `experiments/trajectories/` (`box`, `straight`,
`figure_eight`); each declares its own duration, which the runner reads to size
the recording window. Fault types and their timing are defined in
`experiments/faults/faults.yaml`; the command line selects which fault applies to
which sensor and at what magnitude. A sensor with no flag runs clean.

## Preliminary results

### Clean baselines

No faults injected, using the Stage 1 EKF configuration in the UTM environment
described above, with the fault injector present in the data path (passthrough
mode). ATE is absolute trajectory error, translation part, unaligned; all values
in metres. These figures characterise total run-to-run variance in one
environment; they are not a decomposed error budget and have not been validated
across machines.

| trajectory     | runs | ATE RMSE (mean ± std) | spread |
|----------------|------|-----------------------|--------|
| straight       | 5    | 0.0245 ± 0.0020       | 22%    |
| box            | 10   | 0.0408 ± 0.0029       | 25%    |
| figure_eight   | 5    | 0.1962 ± 0.0009       | 1.0%   |

Per-frame relative error (RPE) on the box baseline is about 0.0023 m/frame.

**Trajectory sensitivity.** The run-to-run standard deviation is roughly constant
in absolute terms across trajectories (~0.002-0.003 m), set by physics-solver
nondeterminism and sensor-noise seeds. Because that noise floor is roughly fixed
while accumulated error is not, trajectories with larger error have
proportionally *lower* relative noise. figure_eight's 1.0% spread makes it about
25x more sensitive than box's 25%: a fault must shift box ATE by roughly a
quarter to be distinguishable from noise, but only by a percent or so on
figure_eight. Sensitive experiments should therefore lead with figure_eight.

### Fault injection

A first fault type is implemented: a constant bias added to the IMU yaw-rate
channel, active over a scheduled window. Example, on figure_eight with the fault
starting 8 s into a 28 s run:

| condition                     | ATE RMSE | vs clean |
|-------------------------------|----------|----------|
| clean                         | 0.196    | —        |
| `--imu yaw_bias:0.2` from 8 s | 0.705    | 3.6x     |

The degradation is far outside the 1.0% noise floor for this trajectory. An
earlier whole-run version of the same fault produced 1.50 (7.6x), consistent
with the scheduled fault having less time to accumulate error.

## Methodology

A few decisions that affect how the numbers should be read.

**Alignment is disabled.** Both trajectories start at the same origin (0,0,0), so
pre-scoring alignment has nothing legitimate to correct. When full SE(3)
(Umeyama) alignment was enabled it reported about 14% lower ATE by rotating the
estimate to reduce accumulated drift — which is part of what the benchmark is
trying to measure. Origin alignment, by contrast, produced a near-identity
transform and changed ATE by under 0.01 mm. Reported values are therefore
unaligned.

**Scoring is anchored to simulation time.** The recorder timestamps poses with
the message sim-time header, and the trajectory driver advances on sim time, not
wall-clock. The path driven and the resulting score depend on simulation time
only, so the machine's real-time factor changes how long a run takes in real
seconds but not the trajectory or the number.

**Accelerometer fusion (ax) was tested and excluded.** Fusing IMU linear
acceleration was expected to hurt, since chassis pitch during acceleration can
leak gravity into the forward-acceleration channel. Measured both ways on both
trajectories, the on/off difference stayed within the noise floor — no measurable
effect — so it is excluded for simplicity rather than because it is harmful. The
robot was verified level at rest (orientation flat to ~1e-7); the pitch it shows
is a brief transient during acceleration, not a static tilt.

**Trajectories exercise different error sources.** Straight-line ATE (~0.025) is
lower than box ATE (~0.043), which is lower than figure_eight (~0.197). The
ordering is consistent with the additional heading error introduced by turning,
and by reversing curvature in the figure_eight case.

## Known quirks

- **numpy pinning.** `evo` may pull numpy 2.x, which is binary-incompatible with
  the system SciPy and with ROS 2 Humble's Python packages. Pin `numpy<1.25`.
- **Gazebo Fortress `JointStatePublisher` ignores `<update_rate>`** and publishes
  every physics step (~1000 sim-Hz). `/joint_states` is only used for wheel
  visualization here, so this is left as-is.
- **`Ctrl+C` does not reliably kill a ROS 2 launch.** Nodes with threads waiting
  on external resources — the gz bridge in particular — can survive and corrupt
  the next run. `run_trial.sh` kills by pattern and verifies the graph is empty
  before proceeding.
- **Trajectories must stay inside the 6x6 m arena.** The origin gate catches bad
  starts but not wall collisions; a trajectory that drives into a wall produces a
  large ATE that looks like data but is a crash.

## Roadmap

- **Stage 1 (complete):** clean-run simulation, EKF, scoring, one-command trials
  with integrity gates, characterised noise floors.
- **Stage 2 (in progress):** scenario-driven fault injector with scheduled
  windows — done, with IMU yaw-rate bias as the first fault type. Remaining:
  further fault types (dropout, freeze, drift, wheel slip), a scorecard sweeping
  the fault × magnitude × trajectory matrix, and Docker packaging for
  one-command reproduction.
- **Stage 3:** innovation-based detection (NIS / chi-squared gating),
  cross-consistency isolation, and drop-and-readmit recovery, validated by
  running through the Stage 2 benchmark and kept separate from it.

## License

Apache-2.0. See `LICENSE`.
