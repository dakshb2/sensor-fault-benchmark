## Running in Docker

The container packages the whole environment -- ROS 2 Humble, Gazebo Fortress,
the pinned Python tooling, and the built workspace -- so a trial can be run
without installing any of it locally.

```bash
docker build -t sensor-fault-benchmark .

# default: a clean box trial
docker run --rm sensor-fault-benchmark

# keep the .tum output on the host
docker run --rm -v $(pwd)/results:/workspace/results sensor-fault-benchmark

# any trial the local runner accepts
docker run --rm -v $(pwd)/results:/workspace/results sensor-fault-benchmark \
    ./experiments/scripts/run_trial.sh experiments/trajectories/figure_eight.yaml \
    trial01 --imu yaw_bias:0.2

# the full scorecard
docker run --rm -v $(pwd)/results:/workspace/results sensor-fault-benchmark \
    ./experiments/scripts/scorecard.sh
```

Gazebo runs server-only inside the container (`ign gazebo -s`), selected by the
`IGN_GAZEBO_HEADLESS` environment variable that the Dockerfile sets. Everything
the benchmark measures comes from topic data, so nothing is lost by having no
GUI; on a desktop the same launch file still opens the window.

Files written through a mounted volume are owned by root, since the container
runs as root.

### Does it reproduce?

Container results fall inside the noise floors measured on the reference
machine, which is the check that matters for a benchmark that claims to be
reproducible:

| run | reference machine | container |
|-----|-------------------|-----------|
| box, clean | 0.041 (n=10, 25% spread) | 0.044, 0.046 |
| figure_eight, `--imu yaw_bias:0.2` | 0.654 | 0.699 |

Exact agreement is not expected: physics-solver nondeterminism and sensor-noise
seeds give the clean box run a 25% spread on the reference machine alone. What
matters is that the container's numbers sit inside that spread rather than
outside it.