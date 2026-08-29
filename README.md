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

A matched-condition benchmark used ROS 2 Jazzy with a common robot footprint, NavFn, 1 Hz replanning, worlds,
10 Hz local-costmap input, forward-only controller candidates, a common 0.4 m/s forward cap, and common
actuator acceleration limits. The shared Nav2 recovery tree still includes `BackUp`. It contains
18 scenarios × 3 runs × 4 controllers = 216 episodes. Controller-specific trajectory generation, horizons,
critics, and tuning necessarily remain different.

| Controller | Successes | Collisions | Mean on successful runs | Median | Worst minimum clearance |
|---|---:|---:|---:|---:|---:|
| BAC (matched) | 54/54 | 0 | 29.7 s | 28.4 s | 0.078 m |
| DWB | 48/54 | 2 | 24.6 s | 25.2 s | 0.000 m |
| MPPI (matched) | 51/54 | 0 | 27.9 s | 27.3 s | 0.049 m |
| RPP | 48/54 | 0 | 24.2 s | 24.8 s | 0.016 m |

BAC completed all matched episodes, including 3/3 in an appearing-obstacle case where the other configurations
did not complete, and 3/3 in a 1.5 m corridor with a synthetic 0.25 m lateral localization offset. However, BAC
was markedly slower and had less clearance in the zigzag cases, and one clutter run stalled for 82.8 s before
eventually completing. A separate 216-episode BAC ablation found the clearest effect from removing the bilateral
balance term: in the extreme offset corridor, mean time changed from 29.2 to 42.4 s, clearance from 0.227 to
0.174 m, and mean lateral error from 0.028 to 0.053 m. All ablation variants still completed 54/54, and the
reverse-escape contribution was not identified because baseline BAC never selected reverse in that set.

These are limited deterministic simulation observations, not independent success-probability estimates,
physical-robot safety evidence, or proof of general superiority. See the
[matched comparison and ablation report](docs/en/ablation_and_matched_evaluation.md) and
[method comparison](docs/en/method_comparison.md) for conditions, limitations, earlier feature-enabled results,
and design differences from existing controllers.

### Visual evidence

[![BAC and DWB replayed side by side](docs/media/bac_vs_dwb_matched_appearing_obstacle_thumbnail.jpg)](docs/media/bac_vs_dwb_matched_appearing_obstacle.mp4)

[Watch the 25.5 s side-by-side MP4](docs/media/bac_vs_dwb_matched_appearing_obstacle.mp4). It synchronously
replays run 1 of the matched `appearing_obstacle` benchmark at 2x speed: BAC completed the run, while DWB
eventually aborted. The annotation also reports the three-repeat result (BAC 3/3, DWB 0/3). This is a replay
of stored deterministic 2D ray-cast traces—not Gazebo footage—and its
[input/output hashes](docs/media/bac_vs_dwb_matched_appearing_obstacle_evidence.json) accompany it.

[![BAC demonstrating adaptive clearance in Gazebo](docs/media/bac_gazebo_adaptive_clearance_thumbnail.jpg)](docs/media/bac_gazebo_adaptive_clearance.mp4)

[Watch the 33.1 s Gazebo MP4](docs/media/bac_gazebo_adaptive_clearance.mp4). In one continuous left-to-right
take, an offset obstacle appears after motion starts, BAC detours by 0.37 m, returns to within 0.07 m of the
centerline, and passes through a 1.0 m gate with its center within 0.03 m of the gate center. The 0.50 m body
plus configured side margins requires 0.74 m. It reached x = 11.08 m with no body contact. Synchronized
[telemetry](docs/media/bac_gazebo_adaptive_clearance_telemetry.csv),
[machine-readable checks and input hashes](docs/media/bac_gazebo_adaptive_clearance_evidence.json), and the
[reproduction harness](examples/gazebo/README.md) accompany the video.

The replay supports only its selected matched run and aggregate annotation. The single BAC Gazebo run is
qualitative integration evidence, not an independent trial, physical-robot evidence, or safety validation.

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
- [BAC ablation and matched-condition evaluation](docs/en/ablation_and_matched_evaluation.md)
- [Reproducible Gazebo evidence](examples/gazebo/README.md)
- [Release review history](docs/en/release_review_history.md)
- [Public-release readiness checklist](docs/en/public_release_checklist.md)
- [Japanese documentation / 日本語ドキュメント](docs/README.md)

## License

[MIT License](LICENSE)
