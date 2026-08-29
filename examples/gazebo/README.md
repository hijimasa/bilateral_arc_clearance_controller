# Gazebo adaptive-clearance evidence

This directory reproduces the public simulation video with Gazebo Classic 11
and ROS 2 Humble. It uses Gazebo's differential-drive, 2D ray-sensor, odometry,
contact-sensor, and camera plugins. The BAC filter receives the simulated
`/scan`, `/odom`, and an upstream path-following velocity command; its output
drives the simulated robot through `/cmd_vel`.

The run combines three observable phases in one continuous take: an obstacle is
spawned partly across the centerline when odometry first reaches x = 1.0 m, the
robot returns to that centerline, and it then passes through a 1.0 m gate. The
0.50 m body and 0.12 m configured side margin require 0.74 m of that opening.
This is deliberately simulation evidence, not physical-robot validation or a
full Nav2 controller comparison. The core input is capped at 2.5 m, comparable
to a finite local-costmap horizon; this is a demo condition, not a default recommendation.

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
- the robot progressed beyond the gate to at least x = 11.0 m;
- BAC entered `AVOIDING` for at least five recorded frames;
- the open-space lateral detour reached at least 0.25 m;
- the oriented physical body stayed at least 0.28 m from the appearing obstacle;
- the robot stayed within 2.4 m laterally;
- its center stayed within 0.30 m of the centerline immediately before the gate;
- its center stayed within 0.24 m of the centerline while crossing the gate; and
- Gazebo's body contact sensor reported no collision.

The JSON sidecar records those checks, the ROS/Gazebo versions, the source Git
state at capture time, and SHA-256 hashes for the algorithm and scenario inputs.
Gazebo uses seed 42, the LiDAR noise is disabled, and the spawn event is
position-triggered so host load does not change its initial condition.
