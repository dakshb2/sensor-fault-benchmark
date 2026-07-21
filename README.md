# sensor-fault-benchmark

A reproducible ROS 2 benchmark for evaluating how a localization estimator
responds to sensor faults. It runs a fixed simulated trajectory, records the
estimated and ground-truth poses, and computes standard trajectory-error metrics
(ATE/RPE via [evo](https://github.com/MichaelGrupp/evo)).

The current Stage 1 implementation provides the simulation, an EKF baseline, the
trajectory runner, and the scoring pipeline — the clean-run half of the
benchmark. Stage 2 will add YAML-configured faults so the same estimator can be
evaluated under repeatable IMU, wheel-encoder, and camera failures.

The goal is not a new fault-injection or fault-detection technique — both are
well established. The goal is to make estimator comparisons easier to reproduce
by fixing the simulation, trajectory, fault configuration, timing, and scoring
in one place.

## Integrity model

The package structure and topic wiring keep fault injection, estimation, and
scoring separate, so a future detection method cannot draw on information it
should not have:

- **Faults are injected into raw sensor streams** (`/imu/raw`,
  `/wheel/odometry/raw`). The estimator subscribes to the clean-named topics
  (`/imu`, `/wheel/odometry`), which in Stage 1 are fed by identity relays and in
  Stage 2 by the fault injector. The estimator sees no difference between the two.
- **Ground truth is consumed only by the `scoring` package.** The estimator and
  any future detector never subscribe to it; `/ground_truth/*` appears in neither
  the EKF config nor their manifests.
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
| `fault_injector` | corrupts raw sensor streams — Stage 2 stub    | none                |
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

# run one full trial: teardown -> launch -> record -> drive -> score
./experiments/scripts/run_trial.sh experiments/trajectories/box.yaml trial01
```

`run_trial.sh` runs the whole pipeline as one command and stops if an integrity
gate fails — the ROS graph not being empty after teardown, or the robot not being
at the origin at spawn. It prints the ATE at the end.

Available trajectories: `box.yaml` (default), `straight.yaml`,
`figure_eight.yaml`. Each declares its own duration, which the runner reads to
size the recording window automatically.

## Preliminary results (Stage 1)

Clean-run baseline, differential-drive EKF, no faults injected, using the Stage 1
EKF configuration in the UTM environment described above. ATE is absolute
trajectory error (translation part, unaligned); all values in metres, from five
runs per trajectory. These figures characterise total run-to-run variance in one
environment; they are not a decomposed error budget and have not been validated
across machines.

| trajectory     | runs | ATE RMSE (mean ± std) | spread | notes                    |
|----------------|------|-----------------------|--------|--------------------------|
| straight       | 5    | 0.025 ± 0.002         | 19%    | pure translation, ~2.4 m |
| box            | 5    | 0.043 ± 0.002         | 13%    | four 90°-ish turns       |
| figure_eight   | 5    | 0.197 ± 0.001         | 1.3%   | reversing curvature      |

Per-frame relative error (RPE) on the box baseline is about 0.0023 m/frame.

**Noise floor and trajectory sensitivity.** The run-to-run standard deviation is
roughly constant across trajectories (~0.002 m), set by physics-solver
nondeterminism and sensor-noise seeds. Because that noise is roughly fixed in
absolute terms, trajectories with larger accumulated error have proportionally
*lower* relative noise: figure_eight's ~1.3% spread makes it the most sensitive
of the three, while a fault on the straight-line run must overcome ~19% variance
to register. This suggests figure_eight is the trajectory to lead with when
injecting faults in Stage 2. With five runs each these spreads are estimates, but
the ordering is consistent.

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

- **Stage 1 (current):** clean-run simulation, EKF, scoring, one-command trials.
- **Stage 2:** YAML-driven fault injector (IMU drift, encoder dropout, camera
  loss) between the raw and clean topics; a scorecard sweeping a scenario matrix;
  Docker packaging for one-command reproduction.
- **Stage 3:** innovation-based detection (NIS / chi-squared gating),
  cross-consistency isolation, and drop-and-readmit recovery, validated by
  running through the Stage 2 benchmark and kept separate from it.

## License

Apache-2.0. See `LICENSE`.
