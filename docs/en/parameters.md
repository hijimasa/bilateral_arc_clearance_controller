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

## Motion model

| Parameter | Default | Description |
|---|---:|---|
| `motion_model.type` | `diff_drive` | Kinematic policy: `diff_drive` or `ackermann` |

Both models expose the Nav2-standard body command `(linear.x, angular.z)`, interpreted here as forward speed and
yaw rate. The vehicle model is described at the same granularity as the `AckermannConstraints` of Nav2 MPPI and
takes no parameters of its own: the minimum turning radius `turn_radius_min` is the entire Ackermann
specification. Road-wheel kinematics — wheelbase, steering angle, steering rate — belong to the downstream
vehicle controller.

With `ackermann`, candidates are generated in body curvature `kappa = angular.z / linear.x`. For each sampled
speed, reverse included, the range `|kappa| <= min(1 / turn_radius_min, limits.w_max / |linear.x|)` is sampled
and refined, then converted through `angular.z = linear.x * kappa`. So `limits.w_max` does not merely bound the
resulting yaw rate: above `|linear.x| > limits.w_max * turn_radius_min` it narrows the candidate curvature range
itself. The effective yaw-rate limit is `min(limits.w_max, |linear.x| / turn_radius_min)`. No nonzero-yaw command
is produced at zero speed, and no
in-place rotation candidate exists in the lattice at all. `ackermann` requires a positive `turn_radius_min`; the
controller throws during configuration otherwise.

The downstream vehicle controller must map this body twist to its own steering interface — for a bicycle model
of wheelbase `L`, `delta = atan(L * angular.z / linear.x)`. BAC does not read measured steering-joint state, so
physical steering tracking and steering-rate enforcement are the downstream controller's responsibility and
require separate validation.

## Velocity and candidate generation

| Parameter | Default | Description |
|---|---:|---|
| `limits.v_max` | 0.4 | Maximum forward speed [m/s] |
| `limits.v_min` | -0.1 | Minimum reverse escape speed [m/s]; use 0 with forward-only sensing |
| `limits.w_max` | 1.0 | Maximum absolute angular velocity [rad/s] |
| `limits.acc_v` | 0.8 | Linear acceleration used by the dynamic window [m/s²] |
| `limits.acc_w` | 2.5 | Physical body yaw acceleration used to limit the output yaw rate; 0 disables it [rad/s²] |
| `control_period` | 0.05 | Control cycle assumed by the yaw-rate output limit [s] |
| `window_time` | 0.25 | Time span of the linear dynamic window [s] |
| `v_samples` | 5 | Number of linear-velocity samples; the stop row is added separately, the rotation row only for `diff_drive` |
| `w_samples` | 25 | Number of coarse yaw-rate samples for differential drive or body-curvature samples for Ackermann |
| `w_refine_steps` | 3 | Fine samples added on each side of the best coarse yaw-rate/curvature candidate; 0 disables refinement |
| `turn_radius_min` | 0.25 | Minimum turn radius [m]. For `diff_drive` it is the lower bound that keeps clearance scoring of translating candidates from degenerating; for `ackermann` it is the kinematic constraint that bounds candidate curvature itself and must be positive |
| `velocity_min` | 0.005 | Output linear speeds below this value are rounded to zero [m/s]. Ackermann zeroes the whole command, since a yaw rate without speed is not realizable |
| `angvel_min` | 0.01 | Differential drive only: output angular speeds below this value are rounded to zero [rad/s]. Ackermann does not apply it, because a small yaw rate at low speed can still represent material curvature; it zeroes `angular.z` only when the curvature itself is negligible |

Differential-drive yaw-rate candidates and Ackermann curvature candidates cover their full configured range
rather than an acceleration window around the current state. This intentional departure from the original DWA
keeps corrective arcs available in narrow passages. At the output, both models clamp the chosen command to the
yaw rate reachable in one cycle from `limits.acc_w` and `control_period`; Ackermann then reapplies its
`turn_radius_min` curvature bound. BAC rechecks stopping admissibility on the clamped constant-curvature arc. If
it must reduce speed, both `v` and `w` are scaled to preserve curvature. The yaw-rate reachability bound is
reapplied and the resulting arc rechecked if that scaling demands excessive angular deceleration. `limits.acc_w`
is a body yaw acceleration and is not a guarantee about Ackermann road-wheel steering rate. Swept motion during
the acceleration or steering transient and jerk remain unevaluated. The downstream controller is responsible for
executing those limits.

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
| `weights.hysteresis` | 0.6 | Weight for change from the previous output command, after the reachability clamp and deadband: yaw rate for differential drive [score per rad/s], curvature for Ackermann [score per 1/m] |
| `weights.squeeze` | 0.5 | Speed penalty under small lateral clearance |

`weights.hysteresis` carries different units per model: [score per rad/s] against yaw-rate change for
differential drive, [score per 1/m] against curvature change for Ackermann. The same weight value therefore has a
different effective strength. The Ackermann term scales roughly with `2 / turn_radius_min` and the
differential-drive one with `2 * w_max`, which for the shipped Ackermann example (`turn_radius_min` 1.0,
`w_max` 0.8) is 2.0 against 1.6 - they do not coincide. The Ackermann curvature term also does not shrink with
speed the way the differential-drive yaw-rate term does, so **this weight does not transfer between models**.
Measured: putting the differential-drive default of 0.6 into the shipped Ackermann configuration overpowers path
following and the vehicle orbits instead of closing on the goal (`bac_ackermann_scenarios` fails). The shipped
Ackermann example ships 0.3. A small `turn_radius_min` can let `weights.hysteresis` dominate
`weights.clearance` for the same reason, so always re-tune the weights after changing the motion model.

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
| `diagnostics_publish_period` | Nav2 | 1.0 | Period for standard `diagnostics` messages; non-positive disables publication [s] |
| `sensor.x/y/yaw` | filter | 0 | Fixed 2D extrinsics of the LaserScan frame |
| `virtual_path_length` | filter | 3.0 | Length of the virtual path created from input `cmd_vel` [m] |

The filter node does not use TF, so set `sensor.*` for the physical installation. The Nav2 plugin transforms
LaserScan data into the base frame using TF. Its standard `diagnostics` output reports `raw_scan`, `costmap`, or
`costmap_fallback`, the scan fallback reason, BAC status, candidate counts, and selected-candidate clearance.

## Internal constants not exposed as ROS parameters

The following method-specific shape constants are not expected to vary by robot or environment and use compile-
time defaults in `bac_core.hpp`: `eval_angle_max` 1.05 rad, the maximum curved-candidate evaluation angle;
`blocked_near`/`blocked_far` 0.4/1.2 m, the fade range for contact penalties beyond the clearance window; and
`side_envelope_headroom` 0.1, the lateral-envelope cushion ratio. They can still be overridden from C++ through
`bac::Params`.
