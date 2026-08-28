# Nav2 integration guide

English | [日本語](../nav2_integration.md)

## Recommended integration point

BAC is primarily intended for the Controller Server. It uses the global plan as local intent while selecting the
next velocity command from obstacle points and current velocity, and it may locally depart from the path when
needed. This matches the Nav2 controller responsibility of generating control effort that is feasible in the
local environment.

- [Nav2 Controller Server](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/controller_server/)
- [Writing a Controller Plugin](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html)

```text
Planner → BAC / DWB / MPPI → Velocity Smoother → Collision Monitor → base
```

## Relation to other extension points

| Objective | Appropriate extension point | Relation to BAC |
|---|---|---|
| Generate `cmd_vel` while tracking a path and avoiding locally | Controller plugin | Primary BAC use |
| Retain DWB trajectory generation and add bilateral-clearance cost | DWB critic | Alternative when only the bilateral term is required |
| Integrate and retain sensor data as an obstacle representation | Local costmap layer | Does not select candidate arcs |
| Restrict speed in fixed map regions | Speed Filter | Separate from steering around live obstacles |
| Stop or slow down at the final command stage | Collision Monitor | Independent protective layer used with BAC |
| Select controllers and compose recoveries | BT / ControllerSelector | Useful when BAC is selected only in narrow passages |

BAC combines candidate generation, rectangular swept contact, stopping admissibility, bilateral centering, and
reverse escape. Moving the current design into a DWB critic would therefore change its meaning. If only the
bilateral-clearance score is desired, a custom DWB critic is a smaller change because DWB exposes both critic
plugins and trajectory generators.

## Controller plugin configuration

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "bac::BacController"
      scan_topic: /scan       # Empty: use LETHAL cells from the costmap
      scan_timeout: 0.5
      scan_downsample: 1
      scan_min_points: 10
      footprint.front: 0.5
      footprint.rear: -0.5
      footprint.width: 0.95
      safety_margin.front: 0.2
      safety_margin.rear: 0.2
      safety_margin.side: 0.2
      avoid_margin.side: 0.6
      limits.v_max: 0.4
      limits.v_min: 0.0       # Change to a negative value only after verifying rear sensing
      limits.w_max: 1.0
      limits.acc_v: 0.8
      limits.acc_w: 2.5
      control_period: 0.05
      stop_decel: 0.8
      weights.balance: 4.0
      sim_time: 2.5
```

The plugin transforms the plan into the base frame using TF. A missing plan `frame_id` or a TF failure is a
controller error. It also transforms raw scans into the base frame. If no scan has arrived, the scan is stale,
too few valid measurements remain, or TF fails, it falls back to lethal costmap cells. For costmap input,
`costmap_margin_compensation` accounts for cell-center quantization.

This fallback favors availability but changes the observation source. If low sensitivity to map–odom error from
raw scans is an operational requirement, monitor the standard `diagnostics` topic or use an upstream supervisor
or Collision Monitor to implement fail-stop behavior. BAC reports `raw_scan`, configured `costmap`, or
`costmap_fallback`; a fallback is WARN-level and includes its reason. `diagnostics_publish_period` controls the
publication interval and a non-positive value disables it.

## Using Collision Monitor

[Collision Monitor](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/)
reads sensors independently of the costmap and trajectory planner, then limits or stops velocity at the final
stage. It is not a controller that steers around obstacles. Following the official Nav2 ordering, place Collision
Monitor last in the `cmd_vel` post-processing chain, including when Velocity Smoother is used.

BAC selects a candidate with larger modeled clearance; Collision Monitor independently checks the resulting
command. Combining them does not automatically establish compliance with a physical-robot safety standard.
Validate zones, source timeouts, stopping distance, speed, and sensor coverage on the actual robot.

## Selecting among controllers

To use RPP or MPPI in open space and BAC in narrow openings, register multiple plugins with Controller Server and
select one using
[ControllerSelector](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/bt_plugins/actions/ControllerSelector/)
or the `FollowPath.controller_id` supplied by the application. Keep selection conditions observable—such as task,
location, passage class, or perception confidence—instead of embedding an implicit switch inside BAC.

## `bac_filter_node`

`bac_filter_node` converts an existing controller's `cmd_vel` into a virtual arc and reshapes it using a raw scan.

```bash
ros2 run bilateral_arc_clearance_controller bac_filter_node \
  --ros-args -r scan:=/scan -r odom:=/odom \
             -r cmd_vel_in:=/nav_cmd_vel -r cmd_vel_out:=/cmd_vel
```

The installed example can instead be started with:

```bash
ros2 launch bilateral_arc_clearance_controller bac_filter.launch.py
```

It passes the command through when obstacles are outside `influence_range`, uses core output while `AVOIDING`,
and stops if scan or odometry input times out. `avoid_status` is `0=CLEAR`, `1=AVOIDING`, or `2=STOP`.

Because it reconstructs a trajectory from a command selected by an upstream controller, it does not fully
preserve pure rotation, reverse motion, or the upstream acceleration model. Use the Controller plugin for primary
Nav2 integration and Collision Monitor for simple final-stage stopping or slowdown. Limit the filter node to
evaluation, staged integration into an existing system, or a non-Nav2 command source.

The filter node does not use TF. If LaserScan is not expressed in the base frame, set `sensor.x/y/yaw` to the
fixed 2D extrinsics.

## Physical-robot checklist

- Measure the footprint and sensor extrinsics.
- Set `stop_decel` no higher than the deceleration the robot can achieve under all relevant conditions.
- Match `control_period` to the Controller Server frequency.
- Use `limits.v_min=0.0` when rear coverage is insufficient.
- Define scan, odometry, and TF timeouts and the fallback policy.
- Align Velocity Smoother and downstream acceleration limits with BAC assumptions.
- Place Collision Monitor last and verify that it can stop the robot without BAC.
- Re-run core tests, the Nav2 benchmark, and latency, dropout, and outlier tests for the physical configuration.
