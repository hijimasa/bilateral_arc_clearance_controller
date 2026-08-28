# Comparison with existing methods and the role of BAC

English | [日本語](../method_comparison.md)

- Literature reviewed: 2026-08-27
- Benchmark verified: 2026-08-28

## Scope of comparison

BAC addresses local path following and obstacle response for a differential-drive robot with a rectangular
footprint using 2D LiDAR or costmap points. This document compares it with velocity-space sampling, trajectory
optimization, strict path tracking, and reactive navigation methods designed for constrained spaces.

The method-level comparison below concerns design properties. It does not place performance values from
different papers and datasets in a single quantitative ranking.

## Method lineage

### Dynamic Window Approach (DWA) and DWB

The DWA of Fox, Burgard, and Thrun restricts the velocity search using robot kinematics and dynamics, then searches
linear and angular velocity space for reactive collision avoidance. BAC inherits velocity-candidate generation
and the admissibility requirement that the robot must be able to stop before collision. Nav2 DWB extends this
family with pluggable trajectory generators and critics whose weighted sum covers obstacles, path distance, goal
distance, oscillation, and other objectives.

BAC differs by retaining separate free-space measurements on the left and right of each candidate arc instead of
reducing obstacle cost to a single minimum distance or costmap value. In a tight passage,
`min(left, right)` and the left–right difference create a geometric centering direction. In open space, clearance
reward saturates and path terms regain priority. BAC is less extensible than DWB's critic architecture and does not
provide DWB's standard holonomic trajectory generator.

### Model Predictive Path Integral (MPPI)

Nav2 MPPI rolls out many control sequences from sampled control noise and updates the sequence using a motion
model and multiple critics. It supports differential-drive, holonomic, and Ackermann models, and can represent
smooth multi-step avoidance using a longer control sequence.

BAC selects one constant-curvature arc per control tick. Its computation and tuning surface are smaller, and
bilateral-clearance terms are easier to inspect in logs. In exchange, it does not represent an S-shaped control
sequence, predict dynamic-obstacle motion, or support non-differential-drive models.

### Regulated Pure Pursuit (RPP)

RPP augments pure pursuit with speed regulation based on curvature, obstacle proximity, and time to collision. It
is compact and fast when the path is valid and clear, but its primary purpose is to track that path, not to create
a lateral local detour around an unmodeled obstacle that remains on it.

BAC may depart from the path when bilateral clearance requires it while retaining distance to the local goal.
This can reduce waiting for replanning, but candidate evaluation is more expensive than RPP in open space and may
produce a longer traversal time.

### Vector Field Histogram (VFH) and Nearness Diagram (ND)

VFH compresses local occupancy into a polar histogram and selects a steering direction from the target direction
and obstacle-free valleys. ND classifies dense or complex local situations and explicitly includes modes such as
High Safety Narrow Region.

These methods resemble BAC in using the left–right or opening structure of nearby obstacles. BAC instead evaluates
body width and braking distance along dynamically executable constant-curvature arcs and retains the Nav2 local
path as continuous intent. It does not implement ND-style discrete situation classes or VFH histogram memory.

## Summary

| Method | Search unit | Obstacle representation | Narrow-space centering | Main strengths | Main limitations |
|---|---|---|---|---|---|
| DWB | Reachable local trajectories | Costmap and critics | Depends on critic design | Standard Nav2 plugin, extensible, holonomic support | Substantial critic-weight tuning |
| MPPI | Time sequence of controls | Costmap and critics | Depends on trajectory cost | Multiple motion models and smooth multi-step trajectories | Higher computation and configuration complexity |
| RPP | Path-tracking curvature | Collision time and proximity regulation | Depends on the path | Fast, strict path tracking | Local path departure is not its primary avoidance mechanism |
| VFH / ND | Steering direction or situation-specific strategy | Polar histogram or nearness diagram | Explicit openings and narrow regions | Reactivity and established narrow-space strategies | Integration with paths and body dynamics depends on the implementation |
| BAC | Constant-curvature `(v,w)` arc | Bilateral arc clearance | Directly penalizes the left–right difference | Geometric passage centering and an inspectable small core | Differential drive and static points; angular transient and jerk remain unevaluated |

BAC is not intended as a general replacement for DWA or DWB. It is specialized for narrow openings, lateral path
offset, and obstacles not yet reflected in the path. This implementation alone does not establish academic
novelty or performance superiority over prior research.

## Nav2 system benchmark

The workspace `nav2_benchmark` used ROS 2 Jazzy with a common rectangular footprint, NavFn, 1 Hz replanning,
worlds, and a 2D LiDAR simulator. On 2026-08-28, all controllers were regenerated after the release-review changes
to plan TF handling and swept-contact geometry, using the same 18 scenarios × 3 runs × 4 controllers = 216
episodes.

This dataset is not a controller-algorithm-only experiment. The controller-specific integration paths retained
the following differences:

| Factor | BAC | Comparison controllers | Consequence |
|---|---|---|---|
| Obstacle input | Direct 20 Hz raw scan; costmap fallback | Primarily 10 Hz local costmap | Observation latency and preprocessing differ |
| Reversing | `limits.v_min=-0.1` | DWB/RPP disabled; MPPI allowed to -0.15 | Escape action sets are not matched |
| Controller tuning | BAC-specific limits and weights | Native DWB/MPPI/RPP settings | The result compares configured systems, not one isolated scoring term |

The results below are useful integration evidence and a source of hypotheses, but they do not causally attribute
the differences to bilateral clearance. A matched-input benchmark and BAC ablations remain publication-readiness
work.

The runner prevents simultaneous episodes from sharing a `ROS_DOMAIN_ID`. `results/domain_manifest.csv` verifies
zero overlap across 216 retained intervals: all 90 domain IDs were reused, with 126 assignments after the initial
allocation. `results/provenance.json` records the BAC commit, zero dirty files, Nav2 version, container image
digest, world and configuration hashes, and parallel-job count.

| Controller | Successful episodes | Collisions | Mean time on successes | Median | Observed tendency |
|---|---:|---:|---:|---:|---|
| BAC | 54/54 (100%) | 0 | 27.6 s | 28.4 s | Completed every tested condition; limited slowdown in narrow and degraded cases |
| DWB | 49/54 (91%) | 1 | 27.9 s | 25.2 s | Fast in nominal cases; failed all appearing-obstacle runs and collided at 0.25 m offset |
| MPPI | 51/54 (94%) | 0 | 29.0 s | 27.2 s | Strong in nominal and crossing-obstacle cases; failed appearing-obstacle runs |
| RPP | 48/54 (89%) | 0 | 24.2 s | 24.8 s | Fastest successful runs; aborted on appearing obstacles and 0.25 m localization offset |

Mean traversal time is the arithmetic mean over successful episodes only; it is not a total score that penalizes
failures. The successful scenario set also differs by controller, so timing comparisons are descriptive. DWB's
mean exceeds its median because one `corridor_extreme_offset` recovery took 168 s; that does not negate its speed
under nominal conditions. BAC's longest result was 32.8 s. Because the simulator is deterministic and run-level
independence is limited, do not interpret the ratios as general success probabilities.

The largest observed outcome differences appeared in the following conditions:

- `appearing_obstacle`: BAC 3/3; DWB, MPPI, and RPP each 0/3. BAC locally detoured around an obstacle introduced
  onto the path.
- `corridor_locdrift_15x`, a 1.5 m corridor with **0.25 m** localization offset: BAC 3/3; DWB 2/3 with one
  collision; MPPI 3/3 with clearance reduced to 0.06–0.08 m; RPP 0/3. The next section reports the offset sweep.
- `corridor_extreme_offset`: DWB 2/3, including one 168 s recovery and one abort; the others were 3/3.
- All controllers succeeded in nominal open-space, wide-corridor, and narrow-corridor cases. BAC was generally
  1.05–1.1 times slower: open space 16.0 s versus RPP 14.8 s, wide corridor 26.0 versus 24.8, and offset narrow
  corridor 30.9 versus 28.8.
- All controllers succeeded in the opposite-turn, two-corner 1.7 m zigzag. RPP took 20.9 s, DWB 21.2 s, MPPI
  27.2 s, and BAC 31.5 s. BAC remained conservative after exact rectangular corner-sweep handling. It also
  completed `corridor_zigzag_locdrift`; BAC's worst minimum clearance over all 54 episodes, 0.139 m, occurred in
  this family.

### Lateral localization-offset sweep in a 1.5 m corridor

`results_driftsweep/` swept lateral map–odom offset from 0.10 to 0.25 m with two episodes per value.

| Offset [m] | BAC | DWB | MPPI | RPP |
|---:|---|---|---|---|
| 0.10 | 28.8 s / clr 0.23 | 25.2 s / 0.14 | 27.2 s / 0.17 | 24.8 s / 0.17 |
| 0.15 | 28.8 s / **0.23** | 25.2 s / 0.09 | 27.6 s / 0.11 | 25.2 s / 0.11 |
| 0.20 | 28.8 s / **0.23** | 25.2 s / 0.07 | 29.5 s / 0.06 | 27.8 s / 0.06 |
| 0.25 | 28.8 s / **0.22** | 1/2, one abort | 51.8 s / 0.08 | 0/2, both aborted |

In this sweep, BAC's displayed traversal time did not change and clearance remained 0.22–0.23 m. The comparison
controllers lost clearance as offset increased; DWB completed 1/2 and RPP 0/2 at 0.25 m. MPPI completed 2/2 but
with 0.08 m clearance and 51.8 s traversal time. These results apply to the tested settings and deterministic
simulator; they do not establish general invariance to offset or probabilistic superiority among controllers.

The parent repository tracks the simulator, evaluator, launch files, worlds, settings, scripts, and container
definition under `nav2_benchmark/`. Each result set records the BAC commit, benchmark tree hash, container image
digest, and configuration hashes in `provenance.json`. Release artifacts should include the raw episode archive.
Physical-robot evaluation still needs sensor latency, slip, point outliers, dynamic objects, and control-period
overruns.

The current runner structurally separates episodes by coupling a domain free-list to process reaping. Each run's
`domain_manifest.csv` verifies the parent-recorded launch and actual reap intervals. The canonical 18-scenario
numbers above were regenerated with this runner. The 32-episode offset sweep and 24-episode opening-width sweep
used fewer than the 90 available domains and therefore required no domain reuse; those sets remain from the older
runner.

## Sensitivity reduced in the execution layer

This section addresses whether the global planner alone can provide the same property. In an architecture where a
global path is updated at finite rate from a single pose estimate, the global planner does not perform the
following per-control-cycle operations. BAC evaluates robot-frame observations every cycle to locally reduce
sensitivity to lateral map–odom error and replanning delay. This does not deny alternatives or complements such
as belief-space or uncertainty-aware planning, other local controllers, and independent protective layers.

1. **Information asymmetry:** A planner operates on the map and pose estimate. If the pose estimate itself is
   wrong, the planner has no direct observation of the true relative geometry. Robot-frame obstacle geometry,
   emergency checks, and bilateral clearance do not directly depend on map–odom error. Path terms still depend on
   the transformed plan, but no degradation was observed in the tested corridor range under weak lateral-path
   weighting and bilateral clearance. Sensor-extrinsic error and latency remain.
2. **The replanning window:** Replanning occurs at a finite rate. Velocity response to an obstacle before the next
   path update belongs to a local layer. In this experiment, the path-following baselines still failed the
   appearing-obstacle condition with 1 Hz replanning and 5 Hz costmap updates.
3. **Location of velocity response:** Slowdown or entry rejection as a function of local collision-course
   geometry is not encoded in a global path alone. Nav2 offers other local mechanisms—including DWB, MPPI, RPP,
   Collision Monitor, and Speed Filter—that may provide alternative responses.

### Tested upstream mitigation: planner-side padding

As a direct upstream mitigation, global-costmap `footprint_padding` was increased for
`corridor_locdrift_15x`, the 1.5 m corridor with **0.25 m** localization offset, using two episodes per setting.

| Upstream setting | DWB | MPPI | RPP | BAC without mitigation |
|---|---|---|---|---|
| Baseline | Collision | Success, clr 0.06 | Aborted | **Success**, 28.8 s, clr 0.23 |
| Padding 0.15 m | Still collided | Aborted | Aborted | — |
| Padding 0.30 m | No plan | No plan | No plan | — |

The path was already at the corridor center; the coordinate relationship was wrong. In this evaluation, padding
smaller than the drift left tracking error, while body plus padding exceeded the corridor width at 0.30 m:
`0.95 + 2 × 0.30 = 1.55 > 1.5`, so planning failed. The tested padding settings did not combine narrow-passage
traversal with offset absorption. This is not proof that every planner, local controller, or uncertainty-aware
method is incapable of doing so.

### Observed proximity and collision statistics

Minimum obstacle distance across the canonical 18 scenarios × 3 runs, 54 episodes per controller:

| Controller | Collisions | Distance <0.05 m | Distance <0.10 m | Worst [m] | Median [m] |
|---|---:|---:|---:|---:|---:|
| BAC | 0 | 0 | 0 | 0.139 | 0.303 |
| DWB | 1 | 11 | 16 | 0.000 | 0.205 |
| MPPI | 0 | 0 | 3 | 0.091 | 0.281 |
| RPP | 0 | 2 | 5 | 0.017 | 0.300 |

In an opening-width sweep with a 0.95 m body, DWB and RPP speed was nearly unchanged, including 0.40 m/s through
a 1.15 m opening with 0.05 m clearance per side. MPPI changed from 0.39 to 0.38 m/s. BAC changed from 0.39 to
0.28 m/s at 1.35 m, 0.10 m/s at 1.25 m, and rejected forward entry at 1.15 m without collision. Minimum distance
during rejection was about 0.11 m, so the canonical table's “no entry below 0.10 m” statistic does not by itself
describe behavior at the rejection boundary.

The package is therefore positioned as a specialized controller that uses robot-frame observations to **reduce
sensitivity to specific coordinate offset and replanning delay**, not as the fastest controller. Within the tested
range, it showed: (a) zero collisions and 0.139 m worst minimum clearance over 54 episodes; (b) staged slowdown for
collision-course obstacles; (c) rejection of a 1.15 m opening; and (d) reverse escape within observed space.
These are simulation observations conditional on sensor data, TF, odometry, footprint, braking model, and
downstream velocity tracking. They do not imply independence from arbitrary upstream failure or physical-robot
safety.

## Relation to Nav2 Collision Monitor

When execution-layer protection is part of the package positioning, comparison and combination with
[Nav2 Collision Monitor](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/)
are essential. Collision Monitor reads sensors independently of the costmap and planner and provides stop,
slowdown, limit, velocity-dependent approach, and source timeout actions.

Collision Monitor **limits** `cmd_vel` by slowing or stopping but does not **steer**. BAC combines local path
departure, passage centering, and reverse escape and selects a candidate with larger modeled clearance. The two
roles are complementary: Collision Monitor can serve as the final protective layer, while BAC acts earlier to
reduce how often it activates. A benchmark baseline combining BAC and Collision Monitor remains future work.

## References and primary sources

1. D. Fox, W. Burgard, S. Thrun, “The Dynamic Window Approach to Collision Avoidance,” *IEEE Robotics & Automation Magazine*, 4(1), 1997. [CMU publication page](https://publications.ri.cmu.edu/the-dynamic-window-approach-to-collision-avoidance)
2. Navigation2, “DWB Controller.” [Official documentation](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)
3. Navigation2, “Model Predictive Path Integral Controller.” [Official source documentation](https://github.com/ros-navigation/navigation2/blob/main/nav2_mppi_controller/README.md)
4. S. Macenski et al., “Regulated Pure Pursuit for Robot Path Tracking,” *Autonomous Robots*, 2023. [arXiv](https://arxiv.org/abs/2305.20026)
5. J. Borenstein, Y. Koren, “The Vector Field Histogram—Fast Obstacle Avoidance for Mobile Robots,” *IEEE Transactions on Robotics and Automation*, 7(3), 1991. [Author-hosted PDF](https://public.websites.umich.edu/~ykoren/uploads/The_Vector_Field_HistogramuFast_Obstacle_Avoidance.pdf)
6. J. Minguez, L. Montano, “Nearness Diagram (ND) Navigation: Collision Avoidance in Troublesome Scenarios,” *IEEE Transactions on Robotics and Automation*, 20(1), 2004. [Author-hosted PDF](https://webdiis.unizar.es/~jminguez/TRAND.pdf)
