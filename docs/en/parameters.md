# Parameter reference

English | [日本語](../parameters.md)

`bac_filter_node` and `bac::BacController` expose the core parameters below with the same names. In the Nav2
plugin, prefix them with the plugin namespace, for example `FollowPath.`. All units are SI.

## Footprint and margins

| Parameter | Default | Description |
|---|---:|---|
| `footprint.front` | 0.5 | Distance from the base origin to the front edge [m] |
| `footprint.rear` | -0.5 | Distance from the base origin to the rear edge [m]; negative |
| `footprint.width` | 0.95 | Body width [m] |
| `safety_margin.front` | 0.2 | Forward margin of the emergency-stop region [m] |
| `safety_margin.rear` | 0.2 | Rear margin of the emergency-stop region [m] |
| `safety_margin.side` | 0.2 | Lateral margin of the emergency-stop region [m] |
| `avoid_margin.side` | 0.6 | Distance from the body side at which clearance reward saturates [m] |
| `ignore_box.front` | 0.0 | Forward extent of the self-reflection exclusion box [m] |
| `ignore_box.back` | 0.0 | Rear extent of the self-reflection exclusion box [m] |
| `ignore_box.width` | 0.0 | Width of the self-reflection exclusion box [m] |

`costmap_margin_compensation` is specific to the Nav2 adapter. It subtracts costmap cell-center quantization from
the safety margin. The default is zero with raw scans and half the cell resolution with costmap-only input.

## Velocity and candidate generation

| Parameter | Default | Description |
|---|---:|---|
| `limits.v_max` | 0.4 | Maximum forward speed [m/s] |
| `limits.v_min` | -0.1 | Minimum reverse escape speed [m/s]; use 0 with forward-only sensing |
| `limits.w_max` | 1.0 | Maximum absolute angular velocity [rad/s] |
| `limits.acc_v` | 0.8 | Linear acceleration used by the dynamic window [m/s²] |
| `limits.acc_w` | 2.5 | Physical angular acceleration. Limits output `w` to the value reachable from measured angular velocity in one cycle and rechecks the clamped arc; 0 disables it [rad/s²] |
| `control_period` | 0.05 | Control cycle assumed by the angular output limit [s] |
| `window_time` | 0.25 | Time span of the linear dynamic window [s] |
| `v_samples` | 5 | Number of linear-velocity samples; stop/rotation rows are added separately |
| `w_samples` | 25 | Number of angular samples across `[-w_max,w_max]` |
| `w_refine_steps` | 3 | Fine samples added on each side of the best coarse angular candidate; 0 disables refinement |
| `turn_radius_min` | 0.25 | Minimum turn radius for translating candidates [m] |
| `velocity_min` | 0.005 | Output linear speeds below this value are rounded to zero [m/s] |
| `angvel_min` | 0.01 | Output angular speeds below this value are rounded to zero [rad/s] |

Angular candidates cover the full range rather than an acceleration window around current angular velocity. This
intentional departure from the original DWA keeps corrective arcs available in narrow passages. At the output,
`limits.acc_w` clamps the selected command to a value reachable from the measured angular velocity and rechecks
stopping admissibility on the clamped constant-curvature arc. The only enforced property is that the **target
angular velocity is reachable after one control cycle**. Swept motion during the angular-acceleration transient
and jerk remain unevaluated. The downstream controller is responsible for executing angular acceleration and
jerk limits.

## Evaluation and weights

| Parameter | Default | Description |
|---|---:|---|
| `sim_time` | 2.5 | Candidate rollout duration [s] |
| `station_lateral_weight` | 0.3 | Lateral path-offset weight relative to `weights.path_dist`; zero on blocked path segments and full Euclidean weight outside the path's longitudinal range |
| `min_eval_distance` | 1.6 | Minimum evaluation distance retained at low speed [m] |
| `eval_lateral_max` | 0.5 | Maximum lateral displacement of a curved candidate [m] |
| `cap_adapt_rate` | 0.05 | EMA update rate for the density-adaptive clearance cap; 0 fixes the cap |
| `weights.clearance` | 2.0 | Reward for the smaller of left and right clearance |
| `weights.balance` | 4.0 | Left–right difference penalty in tight spaces |
| `weights.path_dist` | 1.0 | Weight for path cost: remaining projected arc length plus lateral offset |
| `weights.heading` | 0.15 | Endpoint heading-error weight |
| `weights.hysteresis` | 0.6 | Weight for change from the previously selected angular velocity |
| `weights.squeeze` | 0.5 | Speed penalty under small lateral clearance |

Always re-run `bac_scenario_harness --strict` after changing weights. In particular, `weights.balance` and
`weights.hysteresis` trade narrow-passage centering against steering oscillation.

## Safety, computation, and state

| Parameter | Default | Description |
|---|---:|---|
| `stop_decel` | 0.8 | Braking capability used by admissibility [m/s²]; **must not exceed the physical robot's braking limit** |
| `brake_reaction_time` | 0.1 | Reaction delay included in braking distance [s] |
| `margin_scale_floor` | 0.6 | Lower safety-margin scale at standstill |
| `margin_scale_speed` | 0.3 | Speed at which 100% of the safety margin applies [m/s] |
| `creep_fraction` | 0.3 | Minimum speed fraction used by the proximity governor |
| `side_envelope_lookahead` | 1.0 | Forward governor lookahead [m], shared by linear slowdown for collision-course points and the speed envelope for planned close passes |
| `tight_cruise_fraction` | 0.5 | Cruise-speed fraction allowed in a bilaterally constrained passage, linear in tightness; 1.0 disables moderation |
| `max_range` | 10.0 | Maximum range of obstacle points [m] |
| `max_points` | 1000 | Point-count cap with uniform subsampling above it; non-positive means unlimited |
| `influence_range` | 1.2 | Distance from the body beyond which the filter reports `CLEAR` [m] |
| `avoiding_latch_ticks` | 30 | Ticks for which the filter retains `AVOIDING` after an obstacle leaves |

## ROS-adapter-specific parameters

| Parameter | Component | Default | Description |
|---|---|---:|---|
| `scan_topic` | Nav2 | empty | Raw scan; empty uses lethal costmap cells |
| `scan_timeout` | both | 0.5 | Scan freshness [s]; Nav2 falls back to the costmap when stale |
| `scan_downsample` | Nav2 | 1 | Angular LaserScan downsampling stride |
| `scan_min_points` | both | 10 (int) | Scans with fewer valid measurements—finite hits plus `+Inf` no-return rays—are rejected as sensor faults |
| `scan_inf_is_valid` | both | true | Treat `+Inf` and values beyond `range_max` as valid no-obstacle measurements, equivalent to costmap `inf_is_valid` |
| `cmd_timeout` | filter | 0.5 | Upstream command timeout [s]; output becomes zero |
| `odom_timeout` | filter | 0.5 | Velocity-feedback timeout [s]; output stops |
| `costmap_margin_compensation` | Nav2 | automatic | Compensation for cell-center quantization [m] |
| `sensor.x/y/yaw` | filter | 0 | Fixed 2D extrinsics of the LaserScan frame |
| `virtual_path_length` | filter | 3.0 | Length of the virtual path created from input `cmd_vel` [m] |

The filter node does not use TF, so set `sensor.*` for the physical installation. The Nav2 plugin transforms
LaserScan data into the base frame using TF.

## Internal constants not exposed as ROS parameters

The following method-specific shape constants are not expected to vary by robot or environment and use compile-
time defaults in `bac_core.hpp`: `eval_angle_max` 1.05 rad, the maximum curved-candidate evaluation angle;
`blocked_near`/`blocked_far` 0.4/1.2 m, the fade range for contact penalties beyond the clearance window; and
`side_envelope_headroom` 0.1, the lateral-envelope cushion ratio. They can still be overridden from C++ through
`bac::Params`.
