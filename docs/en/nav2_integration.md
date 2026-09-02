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
      motion_model.type: diff_drive
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

## Ackermann command contract

Set `motion_model.type: ackermann` together with the measured minimum turning radius `turn_radius_min`. Those two
parameters are the whole vehicle model, at the same granularity as the `AckermannConstraints` of Nav2 MPPI. The
installable [Ackermann example](../../config/bac_controller_ackermann.yaml) shows the full parameter block.
Configuration fails if `turn_radius_min` is not positive.

The Nav2 output remains `TwistStamped`: `linear.x` is body-forward speed and `angular.z` is yaw rate. BAC samples
candidates in body curvature and keeps every command within
`|angular.z| <= min(limits.w_max, |linear.x| / turn_radius_min)`, so the commanded arc is always
geometrically drivable by the vehicle. It never emits an in-place yaw command, and no in-place rotation
candidate exists in the lattice.

The downstream Ackermann controller must map this body twist to its steering interface — for a bicycle model of
wheelbase `L`, `delta = atan(L * angular.z / linear.x)`. BAC does not read measured steering-joint state and does
not model road-wheel rate, so the steering rate limit, steering tracking error, and any centering behavior while
stopped are the downstream controller's responsibility. Choose `turn_radius_min` to cover the vehicle's real
turning circle including that tracking error, rather than its geometric minimum alone.

A forward-only Ackermann configuration (`limits.v_min = 0`) cannot reach a goal behind the vehicle. BAC brakes
rather than fabricating a spin, so configure a Nav2 recovery — the standard behavior tree's `BackUp` — for that
case, or enable reverse with `limits.v_min < 0` and adequate rear sensor coverage.

## Holonomic command contract

Set `motion_model.type: omni` together with a positive `limits.vy_max`; configuration fails with zero lateral
authority. The installed [holonomic example](../../config/bac_controller_omni.yaml) shows the whole block.

**This model publishes `cmd_vel.linear.y`.** A downstream base controller that ignores it will drive straight
into the obstacle BAC believed it had avoided. `diff_drive` and `ackermann` always publish zero there, so
existing users are unaffected. The current-velocity input reads `linear.y` as well: on a holonomic platform whose
odometry twist leaves it unfilled, the acceleration window always starts from zero and the lateral response is
under-estimated.

Lateral velocity does the avoiding, not yaw. The yaw rate is a pose regulator, decided before candidate
generation and shared by every candidate.

**A goal orientation can be commanded.** Nav2 carries the requested goal pose on the last plan pose, and the
adapter transforms its orientation into the base frame and passes it to the core. The pose reference fades from
the path tangent to the goal orientation over the last 1.5 m and is fully governed by it within 0.5 m. Because
yaw is not the steering input, the vehicle **arrives holding the requested orientation while lateral velocity
closes the remaining position error**. Measured yaw error for goal orientations of 0.0, -1.2, 1.5, 2.5, -2.8 and
3.0 rad is 0.006-0.060 rad, inside the 0.25 rad `yaw_goal_tolerance` that Nav2's `SimpleGoalChecker` defaults to.
The conditions matter and are stated: `heading_gain` 1.5, the goal at (4, 2), open space. A lower gain gives a
larger error. The differential-drive reference spans 0.541-2.943 rad over the same set.

The orientation is passed on only when the goal itself survives pruning: pruning stops at `max_range`, and the
orientation of an intermediate waypoint is a path tangent, not a goal. `heading_gain: 0.0` holds the heading
fixed while translating, which suits a platform with 360-degree sensing.

`diff_drive` and `ackermann` steer with yaw and cannot choose their orientation independently of their direction
of travel, so they ignore a commanded goal orientation and follow the path tangent as before. A specific goal yaw
for those models needs a Nav2 recovery or a controller switch.

**`bac_filter_node` carries a holonomic command only partly.** The evaluation node synthesises its virtual path
and applies its speed cap on the forward component alone, so lateral velocity survives when there is a forward
component but a purely lateral command does not. Measured on the running node inside the container:
`cmd_vel_in(linear.x = 0.30, linear.y = 0.30)` gives `cmd_vel_out(linear.x = 0.234, linear.y = 0.187)`, while
`cmd_vel_in(linear.x = 0, linear.y = 0.30)` gives zero. The holonomic model's intended home is the Nav2
controller plugin, `bac::BacController`.

**Sideways motion needs sensor coverage abeam the body** — the same caveat `limits.v_min < 0` carries for the
rear. A front-only lidar cannot see where a crabbing robot is going, so set `limits.vy_max` from the observed
field of view, not from the drivetrain.

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
- For Ackermann, measure the real turning circle and verify the downstream twist-to-steering conversion.
- Confirm how the vehicle behaves when a rear goal makes the forward-only configuration stop, and that the
  Nav2 recovery fires.
- Use `limits.v_min=0.0` when rear coverage is insufficient.
- Define scan, odometry, and TF timeouts and the fallback policy.
- Align Velocity Smoother and downstream acceleration limits with BAC assumptions.
- Place Collision Monitor last and verify that it can stop the robot without BAC.
- Re-run core tests, the Nav2 benchmark, and latency, dropout, and outlier tests for the physical configuration.
