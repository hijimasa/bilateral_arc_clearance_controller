# bilateral_arc_clearance_controller

English | [日本語](README.ja.md)

Bilateral Arc Clearance (BAC) is a Nav2 local controller for differential-drive robots. It evaluates the
free space remaining on the left and right sides of candidate arcs. Building on DWA-style velocity candidates
and stopping admissibility, BAC balances observed bilateral clearance in narrow openings and gives global-path
tracking priority in open space.

![BAC candidate-arc and bilateral-clearance geometry](docs/images/bac_geometry.svg)

The package provides three components:

- `bac::BacCore`: a ROS-independent C++17 algorithm
- `bac::BacController`: a Nav2 `nav2_core::Controller` plugin, tested with ROS 2 Jazzy
- `bac_filter_node`: an evaluation and legacy-integration node that reshapes an existing `cmd_vel` using raw scans

The package is licensed under MIT and is currently version 0.1.0.

## Purpose and scope of claims

BAC evaluates robot-frame observations on every control cycle instead of leaving obstacle response entirely to
a path represented in the map frame. Compared with an architecture that periodically updates a global path from
a single pose estimate, BAC is intended to **reduce sensitivity to lateral map–odom error and replanning delay**.

This is not a safety guarantee independent of upstream state. The following assumptions must hold:

- obstacle-sensor coverage, freshness, and extrinsics are adequate for the application;
- TF can transform the plan into the base frame, and current velocity is available;
- the footprint, control period, and braking capability conservatively represent the physical robot;
- the downstream velocity controller enforces velocity and acceleration limits; and
- obstacles can be treated as static points within one control cycle.

While a raw scan is valid, obstacle geometry and bilateral clearance do not directly depend on map–odom error.
However, path-tracking terms still depend on the transformed plan, and the costmap fallback used after a scan
failure again depends on the costmap and TF. The claim is therefore **reduced sensitivity within the stated
assumptions and evaluation range**, not invariance to arbitrary upstream failures. See
[Algorithm and claim boundaries](docs/en/algorithm.md) for the distinction between algorithmic rules,
observations, and non-guarantees.

## Evaluation results

The canonical benchmark used ROS 2 Jazzy with a common robot footprint, NavFn, 1 Hz replanning, worlds, and a
2D LiDAR simulator. It contains 18 scenarios × 3 runs × 4 controllers = 216 episodes. This is a system-level
configuration comparison, not a controller-algorithm-only experiment: BAC consumed the 20 Hz raw scan while the
comparison controllers primarily consumed the 10 Hz local costmap, and reversing policies also differed.

| Controller | Successes | Collisions | Mean on successful runs | Median | Worst minimum clearance |
|---|---:|---:|---:|---:|---:|
| BAC | 54/54 | 0 | 27.6 s | 28.4 s | 0.139 m |
| DWB | 49/54 | 1 | 27.9 s | 25.2 s | 0.000 m |
| MPPI | 51/54 | 0 | 29.0 s | 27.2 s | 0.091 m |
| RPP | 48/54 | 0 | 24.2 s | 24.8 s | 0.017 m |

Within this 18-scenario set, BAC had no collision and did not approach an obstacle closer than 0.13 m. In a
1.5 m corridor sweep with 0.10–0.25 m lateral path offset, its traversal time was 28.8 s and clearance was
0.22–0.23 m, with no degradation observed within that range. These are results from a limited simulation and do
not causally isolate bilateral clearance. They are not general success probabilities or a physical-robot safety
guarantee. See
[Method comparison and evaluation](docs/en/method_comparison.md) for conditions, raw-derived tables, and design
differences from existing controllers.

## Position in Nav2

BAC selects the next `(v,w)` from the plan, local obstacles, and current velocity, including locally departing
from the path when needed. Its primary Nav2 integration point is therefore a Controller Server plugin.

```text
Planner → BAC / DWB / MPPI → Velocity Smoother → Collision Monitor → base
```

Collision Monitor is an independent final-stage stop/slowdown layer, not a replacement for BAC. To use BAC only
in narrow passages, register multiple controllers and select among them with a behavior-tree ControllerSelector
or equivalent application logic. See the [Nav2 integration guide](docs/en/nav2_integration.md) for the distinction
between BAC, a costmap layer, a DWB critic, Collision Monitor, and `bac_filter_node`.

## Quick start

Resolve the ROS 2 and Nav2 dependencies, then build the package in a colcon workspace:

```bash
cd /path/to/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select bilateral_arc_clearance_controller
colcon test --packages-select bilateral_arc_clearance_controller
colcon test-result --verbose
```

Minimal configuration follows. Adjust the footprint, braking capability, and rear sensor coverage for the robot.
An installable, fuller example is provided in [`config/bac_controller.yaml`](config/bac_controller.yaml).

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "bac::BacController"
      scan_topic: /scan
      footprint.front: 0.5
      footprint.rear: -0.5
      footprint.width: 0.95
      limits.v_max: 0.4
      limits.v_min: 0.0  # Use a negative value only with adequate rear sensing
      stop_decel: 0.8
```

The core and closed-loop scenarios can also be tested without ROS:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The 13 closed-loop scenarios include LiDAR ray casting, an acceleration-limited actuator, and unicycle
kinematics.

```bash
./build/bac_scenario_harness --strict --csv-dir traces
python3 test/plot_traces.py --dir traces
```

## Limitations

- BAC assumes a 2D differential-drive robot and constant-curvature arcs; it has no holonomic or Ackermann model.
- It does not estimate obstacle velocity or future obstacle positions.
- The angular command is limited to a value reachable after one control cycle, but swept motion during angular
  acceleration transients and jerk have not been evaluated.
- Enable reverse motion only when rear sensor coverage is adequate.
- A raw-scan failure falls back to the costmap. Configure an independent layer such as Collision Monitor when
  fail-stop behavior is required.
- Version 0.1.0 is evaluated primarily in simulation. Physical-robot latency, slip, outliers, and control-period
  overruns require separate validation.

## Documentation

- [Documentation index](docs/en/README.md)
- [Algorithm and claim boundaries](docs/en/algorithm.md)
- [Nav2 integration guide](docs/en/nav2_integration.md)
- [Parameter reference](docs/en/parameters.md)
- [Method comparison and evaluation](docs/en/method_comparison.md)
- [Release review history](docs/en/release_review_history.md)
- [Public-release readiness checklist](docs/en/public_release_checklist.md)
- [Japanese documentation / 日本語ドキュメント](docs/README.md)

## License

[MIT License](LICENSE)
