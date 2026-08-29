# BAC ablation and matched-condition evaluation

English | [日本語](../ablation_and_matched_evaluation.md)

- Evaluation date: 2026-08-29
- BAC: `25f12be18ab8256e0862134e0dee955966b60e2c` (dirty 0)
- Benchmark: `4fed3d27e601d595d16552920797b76d8d57882c` (dirty 0)
- Environment: ROS 2 Jazzy / Nav2 1.3.12 / 2D LiDAR ray-cast simulator
- Container image: `sha256:d58fe8c8f5790cd000cf7bdc1b46395ac2567c231cd592ae6d29426ba9eb2737`

This evaluation reduces the input-source, reverse-policy, and motion-limit differences left in the earlier
feature-enabled system comparison, and separately decomposes three BAC factors. The 216 matched-comparison
episodes and 216 ablation episodes use separate result roots; the 432 episodes are never aggregated together.

## Matched-condition comparison

### Conditions held common

| Condition | BAC | DWB | MPPI | RPP |
|---|---|---|---|---|
| Obstacle input | 10 Hz local costmap | same | same | same |
| Forward velocity | 0–0.4 m/s | same | same | same |
| Controller reverse candidates | disabled | disabled | disabled | disabled |
| Simulator linear acceleration/deceleration | ±0.8 m/s² | same | same | same |
| Simulator maximum angular acceleration | 2.5 rad/s² | same | same | same |
| Controller frequency | 20 Hz | same | same | same |
| Local-costmap update rate | 10 Hz | same | same | same |
| Footprint | 1.0 × 0.95 m rectangle | same | same | same |
| Planner | NavFn, 1 Hz replanning | same | same | same |

BAC used `scan_topic: ""` and `limits.v_min: 0.0`; MPPI used `vx_min: 0.0`. The remaining comparison settings
were retained from the canonical configurations. The common Nav2 behavior tree still includes `BackUp`.
After controller failure, four DWB episodes and six RPP episodes emitted up to `-0.15 m/s`; therefore the
**controller candidate policies are forward-only, but the complete Nav2 systems are not**.

### Conditions that necessarily differ

Matching does not turn the controllers into the same algorithm. Candidate generation, critics, lookahead, and
trajectory horizons remain controller-specific. Representative horizons are BAC 2.5 s, DWB 1.7 s, MPPI 2.8 s
(56 × 0.05 s), and RPP's 1.0 s collision-time limit. Angular settings also differ: BAC/DWB/MPPI use a 1.0 rad/s
bound while RPP's rotate-to-heading target is 0.6 rad/s (observed `|cmd_w|` remained at or below 1.0 rad/s for
every controller). Scores and tuning also differ. The result is therefore a
comparison of configured controllers under common input and motion constraints, not causal isolation of
bilateral clearance alone.

### Overall result

The run covered 18 scenarios × 3 runs × 4 controllers = 216 episodes. Mean and median times include successful
episodes only and do not penalize failures. Minimum-clearance statistics include finite values from failed
episodes as well.

| Controller | Success | Failure breakdown | Collisions | Successful mean | Successful median | Worst clearance | Clearance median |
|---|---:|---|---:|---:|---:|---:|---:|
| BAC (matched) | 54/54 | none | 0 | 29.7 s | 28.4 s | 0.078 m | 0.253 m |
| DWB | 48/54 | 2 collisions, 2 aborts, 2 timeouts | 2 | 24.6 s | 25.2 s | 0.000 m | 0.208 m |
| MPPI (matched) | 51/54 | 3 timeouts | 0 | 27.9 s | 27.3 s | 0.049 m | 0.285 m |
| RPP | 48/54 | 6 aborts | 0 | 24.2 s | 24.8 s | 0.016 m | 0.288 m |

The three deterministic repeats check repeatability; they are not independent statistical samples or general
success probabilities.

### Conditions that separated the controllers

| Scenario | BAC | DWB | MPPI | RPP | Bounded interpretation |
|---|---|---|---|---|---|
| `appearing_obstacle` | 3/3, 25.5 s, clr 0.302 m | 0/3 | 0/3 | 0/3 | Difference for this abrupt-occupancy setup, not all dynamic obstacles |
| `corridor_locdrift_15x` | 3/3, 28.8 s, clr 0.213 m | 1/3, 2 collisions | 3/3, 34.7 s, clr 0.069 m | 0/3 | Limited to the synthetic 1.5 m corridor and 0.25 m lateral offset |
| `corridor_extreme_offset` | 3/3 | 2/3, 1 timeout | 3/3 | 3/3 | DWB failed once, so this is not treated as a stable separation |
| `corridor_zigzag` | 3/3, 29.5 s, clr 0.154 m | 3/3, 21.3 s, 0.256 m | 3/3, 27.1 s, 0.286 m | 3/3, 20.8 s, 0.261 m | BAC was slower and had less clearance |
| `corridor_zigzag_locdrift` | 3/3, 38.8 s, clr 0.123 m | 3/3, 21.2 s, 0.203 m | 3/3, 27.3 s, 0.211 m | 3/3, 20.8 s, 0.204 m | The drifted zigzag is a clear BAC weakness |

![Appearing-obstacle trajectories under matched conditions](../images/matched_appearing_obstacle.png)

Trajectory overlay for the appearing obstacle. The three deterministic repeats nearly overlap. The legend shows
a representative outcome; the table and raw `summary.csv` are authoritative for every repeat.

BAC also completed `clutter_field` 3/3, but one run took 118.1 s, stopped for 82.8 s, and reached 0.078 m
clearance. The other two runs took 29.7–30.0 s. Success counts hide this stall, so traces and continuous metrics
must accompany the public result.

The data support stable completion by BAC in the tested appearing-obstacle and large-offset cases. Conversely,
RPP/DWB were faster in ordinary path following and zigzags, while MPPI maintained larger clearance in many
conditions. The results do not support calling BAC universally fastest, safest, or a replacement for every
other controller.

## BAC ablation

Each variant changes one intended factor from feature-enabled BAC.

| Variant | Change from baseline | Purpose |
|---|---|---|
| `bac` | none: 20 Hz raw scan, `v_min=-0.1` | feature-enabled baseline |
| `bac_no_balance` | `weights.balance=0` | bilateral-balance contribution |
| `bac_no_escape` | `limits.v_min=0` | remove reverse candidates |
| `bac_costmap` | `scan_topic=""` | use the 10 Hz local costmap while retaining reverse |

Every variant completed 54/54 episodes without collision. No factor was therefore necessary for completion in
this scenario set. Differences appeared in trajectories and continuous metrics.

### Bilateral-balance term

| Scenario | BAC time / clr / mean\|lat\| | Without balance time / clr / mean\|lat\| |
|---|---:|---:|
| `corridor_narrow_walled` | 28.5 s / 0.328 / 0.033 m | 29.2 s / 0.301 / 0.054 m |
| `corridor_narrow_walled_aligned` | 28.4 / 0.323 / 0.041 | 28.8 / 0.239 / 0.100 |
| `corridor_extreme_aligned` | 28.8 / 0.230 / 0.028 | 30.4 / 0.205 / 0.035 |
| `corridor_extreme_offset` | 29.2 / 0.227 / 0.028 | 42.4 / 0.174 / 0.053 |
| `corridor_locdrift_17` | 28.4 / 0.323 / 0.022 | 28.8 / 0.222 / 0.124 |

![BAC ablation trajectories in the extreme offset corridor](../images/ablation_extreme_offset.png)

Every variant completed, while the no-balance trajectory differs in centering and traversal time. The figure is
a visual aid for why continuous metrics must accompany binary outcomes.

Values are arithmetic means over three runs. All three repeats moved in the same direction in these cases,
consistent with the design intent that the balance term contributes to corridor centering, clearance, or
traversal time. This implementation-specific score ablation does not by itself establish academic novelty or
superiority over other methods.

### Reverse candidates

`bac_no_escape` also completed 54/54. Its mean drifted-zigzag time rose from 30.2 s to 60.7 s, but the increase
was concentrated in one 118.9 s run with 87.3 s stopped. Moreover, feature-enabled BAC never selected a negative
`cmd_v` in any of its 54 episodes. The difference therefore cannot be interpreted causally as escape recovery.
Escape contribution remains unidentified until a scenario actually requiring reverse is designed.

### Raw scan versus costmap

`bac_costmap` completed 54/54, so raw scans were not necessary for completion in this set. Most static corridors
showed small differences or parity. In the drifted zigzag, however, raw input produced 30.2 s / 0.155 m compared
with 37.2 s / 0.118 m for costmap input; the costmap variant commanded reverse in two repeats. Both variants
completed `appearing_obstacle` 3/3, with mean clearance 0.304 m for raw input and 0.264 m for costmap input. This
does not show that raw input is always better: it includes the combined effects of update rate, costmap
quantization and inflation, and candidate selection.

## Completeness and reproducibility

Both datasets passed the following checks:

- 216 expected / 216 observed / 0 missing / 0 corrupt / 0 unexpected
- 90 domains, 126 reassignments after first use, zero overlap between domain holding intervals
- all three provenance-v2 digests reproduced: `bench_tree_sha`, Git tree object, and `worlds_sha`
- clean BAC and benchmark worktrees, with configurations, worlds, and container digest recorded

The result roots are `results_matched_release_25f12be/` and `results_ablation_release_25f12be/`.

## Video evidence

### BAC-versus-DWB side-by-side replay

[![Synchronized BAC-versus-DWB matched benchmark replay](../media/bac_vs_dwb_matched_appearing_obstacle_thumbnail.jpg)](../media/bac_vs_dwb_matched_appearing_obstacle.mp4)

The 25.5 s video synchronizes `appearing_obstacle/run1` from the matched dataset and plays simulation time at 2x.
With the same world, obstacle appearance time, and initial state, BAC completed in 25.6 s and DWB eventually
ended as `aborted_6`. The footer's BAC 3/3 versus DWB 0/3 (two aborts, one timeout) is the three-repeat aggregate,
not a claim derived from run 1 alone. The [evidence JSON](../media/bac_vs_dwb_matched_appearing_obstacle_evidence.json)
stores SHA-256 hashes for the world, both traces and episodes, renderer, and outputs. This is a replay of stored
deterministic 2D ray-cast traces, not Gazebo or physical-robot footage.

### One-series Gazebo adaptive-clearance video

[![Gazebo adaptive-clearance demo](../media/bac_gazebo_adaptive_clearance_thumbnail.jpg)](../media/bac_gazebo_adaptive_clearance.mp4)

A BAC-only series, independent of the numerical benchmark, was captured with Gazebo Classic 11.10.2 and ROS 2
Humble. It connects a 20 Hz ray sensor, odometry, body contact, and differential drive. An upstream centerline
follower is capped at 0.35 m/s. The continuous left-to-right take shows avoidance of a static offset obstacle,
centerline recovery, and passage through a 1.0 m gate. The local-obstacle input horizon is 2.5 m, a disclosed
finite-local-costmap-like demo condition rather than a default recommendation. The 0.50 m body plus two configured
0.12 m margins requires 0.74 m. The Humble run excludes the Jazzy Nav2 adapter and connects `bac_filter_node`,
which uses the same `bac_core`.

| Check | Result |
|---|---:|
| Video | 34.9 s, 960 × 540, 12 fps, 419 frames |
| Minimum physical-body clearance from obstacle | 0.309 m (2.6x configured side safety margin) |
| Maximum lateral detour | 0.798 m |
| Maximum detour to `abs(y) <= 0.30 m` | 8.3 s |
| Maximum detour to `abs(y) <= 0.10 m` | 13.1 s |
| Maximum `abs(y)` immediately before gate | 0.042 m |
| Maximum `abs(y)` inside gate | 0.014 m |
| Final x progress | 11.91 m |
| `STOP` | 0 frames |
| Body contacts | 0 |

The video overlays time, state, pose, output command, and an explicit no-physical-validation label. It is
accompanied by frame-synchronized [telemetry CSV](../media/bac_gazebo_adaptive_clearance_telemetry.csv), an
nine-gate [evidence JSON](../media/bac_gazebo_adaptive_clearance_evidence.json), and a
[reproduction harness](../../examples/gazebo/README.md). The JSON records the capture commit and SHA-256 hashes
for the principal inputs.

This is qualitative evidence that the sensor-to-actuator chain worked and combined open-space clearance with
narrow-passage traversal in this one Gazebo condition. It uses a different simulator and dataset from the
side-by-side replay. Neither video establishes an independent success probability, physical latency/slip/outlier
performance, or safety validation.
