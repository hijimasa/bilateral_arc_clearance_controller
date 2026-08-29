# Gazebo appearing-obstacle evidence

This directory reproduces the public simulation video with Gazebo Classic 11
and ROS 2 Humble. It uses Gazebo's differential-drive, 2D ray-sensor, odometry,
contact-sensor, and camera plugins. The BAC filter receives the simulated
`/scan`, `/odom`, and an upstream path-following velocity command; its output
drives the simulated robot through `/cmd_vel`.

The obstacle is spawned five seconds after the run starts. This is deliberately
simulation evidence, not a physical-robot validation or a full Nav2 controller
comparison.

The script builds the distribution-independent core and filter with
`BAC_BUILD_NAV2_PLUGIN=OFF`. The option only excludes the Jazzy Nav2 adapter
from this Humble evidence environment; normal package builds keep it enabled.

## Reproduce in Docker

From the package root:

```bash
docker build -t bac-gazebo-evidence -f examples/gazebo/Dockerfile .
docker run --rm --shm-size=1g \
  -v "$PWD:/source" -w /source bac-gazebo-evidence \
  "bash examples/gazebo/run_demo.sh"
```

The run writes the MP4, thumbnail, synchronized telemetry, and evidence JSON to
`docs/media/`. `run_demo.sh` returns nonzero unless all of the following hold:

- at least 180 camera frames were recorded;
- the robot progressed at least 4.5 m;
- BAC entered `AVOIDING` for at least five recorded frames;
- the lateral detour reached at least 0.25 m; and
- the robot stayed within 2.4 m laterally and returned to within 1.0 m of the
  upstream centerline; and
- Gazebo's body contact sensor reported no collision.

The JSON sidecar records those checks, the ROS/Gazebo versions, the source Git
state at capture time, and SHA-256 hashes for the algorithm and scenario inputs.
