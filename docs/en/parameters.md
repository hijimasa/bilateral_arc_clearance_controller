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
| `motion_model.type` | `diff_drive` | Kinematic policy: `diff_drive`, `ackermann`, or `omni` |

`diff_drive` and `ackermann` expose the Nav2-standard body command `(linear.x, angular.z)`, interpreted here as
forward speed and yaw rate, and always publish `linear.y` as zero. `omni` also publishes `linear.y`, which the
downstream base controller must honour. The vehicle model is described at the same granularity as the `AckermannConstraints` of Nav2 MPPI and
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

With `omni`, **lateral velocity is the avoidance dimension, not yaw rate**. A holonomic body can side-step an
obstacle without changing where it points, so searching over yaw would spend the candidate budget on a degree of
freedom that avoids nothing. The candidate lattice is therefore forward speed × lateral speed — the **same two
dimensions** as the differential-drive forward speed × yaw rate lattice, not a three-dimensional one. It is not
the same candidate COUNT: measured with the shipped holonomic values (`limits.v_max` 0.4, `limits.vy_max` 0.3,
`limits.w_max` 1.0, `v_samples` 5, `vy_samples` 15, `w_samples` 25) from a current velocity of
(0.20 m/s, 0.0 rad/s), 96 holonomic candidates against 130 differential-drive ones (R18 M5, H2).

The yaw rate is not a searched dimension but a regulator output: proportional on the heading error to the local
path tangent, with gain `heading_gain`, **decided before candidate generation and shared by every candidate**.
That ordering is the point. Because yaw is fixed first, the trajectory that is scored and contact-checked is the
trajectory that is driven.

In a passage the regulator also takes the bilateral clearance imbalance into account, pointing the body INTO the
gap. A crabbing rectangle sweeps wider than a straight one — 0.86 m at 55 degrees for a 0.7 × 0.5 m body against
0.5 m going straight — so exactly where lateral room is scarce, crabbing is the expensive way to use it. Turning
reaches the same place through the width the body already occupies. The term is scaled by `tightness` and gated
to passages: bounded on both sides **and** open straight ahead. Around an isolated obstacle the two clearances
are the obstacle's own edges, so balancing them would steer into it.

The speed limit applies to the velocity **vector**, not per axis; a per-axis cap would admit
`hypot(limits.v_max, limits.vy_max)`. The proximity speed governor's cap reaches the lateral axis too; bounding only the forward axis would slow the
vehicle in front of an obstacle while it still slid sideways at full `limits.vy_max`, leaving the direction of
largest swept width unmoderated. The norm cap on the velocity vector does relax to `|limits.v_min|`, so that a
reverse candidate survives the governor as it does for differential drive (the default `v_min` is -0.1).

No value of `heading_gain` is uniformly best: the more orientation the goal demands, the more path is spent
achieving it. Measured in open space driving to a goal at (4, 2) with an orientation requested (4.47 m in a
straight line):

| `heading_gain` | goal yaw -2.8 rad: distance / final yaw error | goal yaw +3.0 rad: distance / final yaw error |
|---:|---|---|
| 0.5 | 4.43 m / 0.602 rad | 4.40 m / 0.517 rad |
| 1.0 | 4.47 m / 0.289 rad | 4.44 m / 0.141 rad |
| 1.5 | 8.63 m / 0.006 rad | 4.47 m / 0.060 rad |
| 3.0 | 8.06 m / 0.003 rad | 4.50 m / 0.005 rad |
| 5.0 | 7.72 m / 0.067 rad | 4.53 m / 0.002 rad |

At 1.0 and below the final yaw can miss Nav2's default `yaw_goal_tolerance` of 0.25 rad. At 1.5 and above the
orientation is met, but an extreme goal yaw nearly doubles the distance travelled and widens the final position
error from 0.29 m to 0.37-0.49 m. The shipped 1.5 is the smallest value that meets the tolerance.

`omni` requires a positive `limits.vy_max`; selecting the model with zero lateral authority would silently
degrade it to a drive that cannot steer, so the controller throws during configuration. **Sideways motion needs
sensor coverage abeam the body** — the same caveat `limits.v_min` carries for reverse.

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
| `limits.vy_max` | 0.0 | `omni` only. Lateral speed authority [m/s]; must be positive, and presumes sensor coverage abeam the body |
| `heading_gain` | 1.5 | `omni` only. Proportional gain of the pose regulator [1/s]. 0 holds the heading fixed, which suits a body with 360-degree sensing. No upper bound is validated; see the measured trade-off below |
| `vy_samples` | 15 | `omni` only. Lateral-velocity samples per forward-speed row, the counterpart of `w_samples`; at least 3 |
| `velocity_min` | 0.005 | Output linear speeds below this value are rounded to zero [m/s]. Ackermann zeroes the whole command, since a yaw rate without speed is not realizable. R19 L10: the skip is a property of `applyCommandDeadband` as a whole, so as with `angvel_min`, **the rounding is skipped when it would leave a twist that can no longer stop before contact** - which only the `angvel_min` row used to say |
| `angvel_min` | 0.01 | Differential drive AND the holonomic model: output angular speeds below this value are rounded to zero [rad/s] (R18 M7 - this entry previously said differential drive only, which is false). Ackermann does not apply it, because a small yaw rate at low speed can still represent material curvature; it zeroes `angular.z` only when the curvature itself is negligible. For every model the rounding is skipped when it would leave a twist that can no longer stop before contact. R19 L12: there are **four** routes here, not three - (1) the rounding does not change the command, so the rounded value (= the command as selected) is published; (2) the rounding changes the command **and the rounded command can still stop**, so **the rounded value is published** - the majority route among the ticks where the rounding changes anything; (3) the rounding changes the command, the rounded command cannot stop, but the command AS SELECTED can, so **the command as selected is published**; (4) neither the rounded command nor the command as selected can stop, so `(0, 0, 0)` is published. The first pass of the R19 response wrote this as three branches, dropping (2) and also dropping "the rounded command cannot stop" from the condition of (3). Measured on randomised differential-drive ticks at the shipped `angvel_min` of 0.01, same inputs fed to `main`: a corridor-plus-close-frontal-point generator drawing `current.w` from ±[0.085, 0.124] reaches the deadband decision on **216715 of 400000 ticks**, and those split (1) 197156, (2) 19377, **(3) 182**, (4) 0 - **all 183285 remaining ticks return earlier from the emergency layer**: `BacCore::process()` has exactly two returns ahead of this decision, the emergency layer and `path.empty()`, and `path.empty()` was measured at 0 (183285 + 0 + 216715 = 400000). A generator placing one obstacle cluster at a random bearing and drawing `current.w` uniformly over ±1 reaches it on 30712 of 40000 ticks, splitting (1) 30415, (2) 297, (3) 0, (4) 0. (The second pass of the R19 response computed (1) as "total ticks minus (2)", which counted ticks that never reached the decision; found by the R19 verification.) In the shipped regression `testDeadbandIsSkippedRatherThanBreakingAdmissibility`, (3) accounts for 255 of 21342 moving ticks - **measured under different conditions from the two rows above**: 30000 randomised single ticks of the holonomic model (a fresh `BacCore` each trial, not a closed-loop run), with `angvel_min` drawn from **[0.05, 0.35] rad/s rather than the shipped 0.01**, so that the branch is reached often enough for the count to be a band (the test's own doc comment says so). The "at the shipped `angvel_min` of 0.01" above does not apply to this 255 of 21342. Route (4) is defence in depth: 0 times over those two conditions (440k ticks) this round, and R18 M8 reported 0 of 4112 events over 360k ticks that also swept `velocity_min` |

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
| `weights.hysteresis` | 0.6 | Weight for change from the previous output command - after the reachability clamp, and after the deadband where the deadband was applied; on a tick where it was skipped because it would have broken stop-before-contact, the command as selected (R18 L2): yaw rate for differential drive [score per rad/s], curvature for Ackermann [score per 1/m], lateral velocity for `omni` [score per m/s] |
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
