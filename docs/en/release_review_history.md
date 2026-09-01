# Release review history

English | [日本語](../release_review_history.md)

## Current state

- Checked: 2026-09-01 (through R14)
- Evaluated package: `13b3d16`. The working tree R11 reviewed was committed as `8fd0d7e` (implementation),
  `5951e0b` (documentation) and `24466ae` (review record); R12 reviewed those three commits and its response
  landed in `3521226`; R13 reviewed through `3bd88cb` (the scenario expansion closing R11 L5) and its response
  landed in `112eb35`; R14 reviewed the whole branch through `112eb35`, recorded in `0586253` with its response
  in `13b3d16`
- Benchmark: `026a17a` (canonical) / `4fed3d2` (matched comparison and ablation); neither R11 nor R12 re-ran the benchmark
- Decision: code review **Go** and public-release P0 **complete**; changing visibility remains an owner decision
- R14 (reviewing `112eb35` on `feature/ackermann-motion-model`) has had its 1 High and 10 Medium findings
  addressed. The verdict is **Conditional Go** and merging to main was concluded to be permissible. The 9 Low
  findings move to the next cycle (L5, L7 and L9 were partly closed)
- R15 (reviewing `616746b` on `feature/omni-motion-model`) has had its four High findings addressed, along with
  the Medium ones that blocked the merge and those needed to defend the fixes; the verdict before the response
  was **Hold**. Six Medium findings and all Low ones move to the next cycle. The `Evaluated package` named in
  this section is `main` (`2488248`); the holonomic model is not in it yet

All Critical, High, and Medium findings across thirteen release-review rounds have been addressed, as have the
three Low findings from R10, the five from R11 (L1 was closed by documentation with no behaviour change, and L5
by the 2026-09-01 scenario expansion), the three from R12, and five of the six from R13. R13 L7 - a data race on
`speed_limit_` that predates this branch - is carried to the next cycle. No release blocker remains. R12 re-verified R11's own work and found that the
abandoned-design wording R11 considered fixed still survived in the public header, that the Nav2 adapter's
differential-drive coverage had been replaced rather than added to, and that the shipped Ackermann example
configuration failed the package's own Ackermann regression suite - R11 L4 had been closed on a false premise. The 2026-08-29 follow-up added ROS adapter tests, input-source diagnostics, a
216-episode matched-condition comparison, and a 216-episode BAC ablation. On 2026-09-01 the Ackermann motion
model was added, and R11 fixed the exception safety of `setParams` (one High finding). Remaining work includes
rollout through angular-acceleration transients, physical disturbance evaluation, a Collision Monitor combined
baseline, decomposition of `BacCore::process()`, and clamping the filter node's virtual path to the turning
circle (R12 L5, deprioritised after R13 confirmed it is benign under Ackermann), and a data race on
`speed_limit_` (R13 L7, predating this branch). The broader Ackermann scenario suite (R11 L5) was completed on
2026-09-01, adding safety stop (forward-only and reverse escape), narrow-corridor centering, and a clutter
field. Ackermann
coverage consists of deterministic unit checks and closed-loop regressions only; there is no vehicle evidence.

Individual records preserve the decision made at each point and are not rewritten when a later review withdraws
a conclusion. In particular, R09 withdrew R08's post-hoc audit based on artifact modification time, and the
canonical 216 episodes were regenerated with the corrected runner. See the
[review-record policy](../reviews/README.en.md) for naming and update rules.

## Review summary

The linked findings and responses are preserved in Japanese as the original audit records.

| ID | Main issue | Final resolution | Original record |
|---:|---|---|---|
| R01 | Mixed plan/pose frames, mixed result generations, rectangular swept footprint, adapter fail-safe, comparison scope | TF transformation, result separation, geometry fix, input timeouts, bounded claims | [Findings, ja](../reviews/r01-2026-08-28-findings.md) / [Response, ja](../reviews/r01-2026-08-28-response.md) |
| R02 | Exact curved rectangular contact, `+Inf` scan semantics, coordinate-error claim, angular reachability, provenance | Closed-form contact, scan semantics, claim limitation, output angular limit, tracked benchmark | [Findings, ja](../reviews/r02-2026-08-28-findings.md) / [Response, ja](../reviews/r02-2026-08-28-response.md) |
| R03 | Contact-angle periodicity, angular-acceleration transient, performance evidence | Periodicity fix and property tests, bounded guarantee, raw microbenchmark | [Findings, ja](../reviews/r03-2026-08-28-findings.md) / [Response, ja](../reviews/r03-2026-08-28-response.md) |
| R04 | Container paths, missing episodes, performance revision | Completeness gate, corrected examples, measurement from a clean revision | [Findings, ja](../reviews/r04-2026-08-28-findings.md) / [Response, ja](../reviews/r04-2026-08-28-response.md) |
| R05 | Gate status, stale episode reuse, performance source, raw columns | Status propagation, preflight, clean provenance, `eval_pts` | [Findings, ja](../reviews/r05-2026-08-28-findings.md) / [Response, ja](../reviews/r05-2026-08-28-response.md) |
| R06 | Writes before preflight, overwrite propagation and deletion scope, file-open errors | Preflight before writes, bounded deletion, host environment propagation, I/O fail-fast | [Findings, ja](../reviews/r06-2026-08-28-findings.md) / [Response, ja](../reviews/r06-2026-08-28-response.md) |
| R07 | Unexpected artifacts, aggregate/provenance failures, input validation | Whole-root set checks, finalization gate, name/count/RTF validation | [Findings, ja](../reviews/r07-2026-08-28-findings.md) / [Response, ja](../reviews/r07-2026-08-28-response.md) |
| R08 | Reuse of active ROS domains, leading zeroes, trace schema, runner tests | PID/domain free-list, decimal normalization, trace semantics, shell integration test | [Findings, ja](../reviews/r08-2026-08-28-findings.md) / [Response, ja](../reviews/r08-2026-08-28-response.md) |
| R09 | Insufficient proof in the legacy-data audit, launch status, fail-open pool | Audit withdrawal, canonical regeneration, parent manifest, status propagation, fail-closed pool | [Findings, ja](../reviews/r09-2026-08-28-findings.md) / [Response, ja](../reviews/r09-2026-08-28-response.md) |
| R10 | Reproducible tree hash, manifest identity, reuse terminology, raw archive | Tracked-source hash, set/schema checks, terminology separation, archive tool | [Findings, ja](../reviews/r10-2026-08-28-findings.md) / [Response, ja](../reviews/r10-2026-08-28-response.md) |
| R11 | Ackermann support: half-applied rejected configuration, documentation diverged from code, vacuous test assertions | Validate-before-commit `setParams`, corrected the abandoned-design documentation, assertions that kill nine mutants | [Findings, ja](../reviews/r11-2026-09-01-findings.md) / [Response, ja](../reviews/r11-2026-09-01-response.md) |
| R12 | Shipped Ackermann config failed the regression suite, abandoned-design wording in the public header, replaced differential-drive adapter coverage | Retuned the example weight and added a shipped-configuration scenario, corrected the header, restored the default-configuration test and split the Ackermann one out | [Findings, ja](../reviews/r12-2026-09-01-findings.md) / [Response, ja](../reviews/r12-2026-09-01-response.md) |
| R13 | Shipped-configuration guard not tied to the yaml, thresholds fitted to one trajectory, speed governor uncovered closed-loop, wrong scenario count | The scenario now reads the yaml, thresholds re-derived from a perturbation band (the one with no separating value was dropped), clearance assertions added, count corrected to 11 | [Findings, ja](../reviews/r13-2026-09-01-findings.md) / [Response, ja](../reviews/r13-2026-09-01-response.md) |
| R14 | Merge-readiness for main. Unpinned behaviour change in the differential-drive output stage, shipped-configuration guard bound per key rather than per file, bands behind the two new thresholds, the `limits.w_max` term never exercised, documentation that misstates the tests | Output-stage semantics pinned by a unit regression, guard bound to the file, two non-separating thresholds removed with their coverage moved to unit tests, a `w_max`-binding fixture added, documentation corrected | [Findings, ja](../reviews/r14-2026-09-01-findings.md) / [Response, ja](../reviews/r14-2026-09-01-response.md) |
| R15 | Merge-readiness for the holonomic model. `vy` never reached the four places that decide the direction of travel, so contact checking, the stop-before-contact test and the emergency layer do not hold for a purely lateral command; one corridor-entry contact; six measured claims did not reproduce | The four direction-of-travel decisions generalised to the velocity vector, the emergency fallback gated on its combined twist, a property test asserting the invariant, and the measured claims and test counts corrected | [Findings, ja](../reviews/r15-2026-09-02-findings.md) / [Response, ja](../reviews/r15-2026-09-02-response.md) |

## Current validation contract

- Plain CMake Release build and ten CTest entries; eleven CTest entries in ROS 2 Jazzy/Nav2 with the adapter
  tests, which run the default (differential-drive) configuration while a separate test covers the Ackermann
  parameter plumbing. The two assert opposite halves of the same tick, so a misresolved `motion_model.type`
  always fails one of them. The Jazzy container additionally checks that the `ackermann`-labelled tests exist and pass, that the
  installed Ackermann configuration selects the model, and that a running node rejects an unsupported
  `motion_model.type` and a non-positive `turn_radius_min`
- Ten holonomic unit tests and twelve closed-loop scenarios. Five of the scenarios run the same world under a
  differential-drive reference and assert on the difference, one runs the shipped holonomic configuration
  itself, and one is a property test over 6000 randomised worlds AND randomised parameters that checks, on every
  tick, that the emitted twist can stop before contact along its own direction of travel - the form in which
  R15 H1, H2 and H3 were found and the only one that could have found them
- Core unit/property tests, 17 closed-loop scenarios, and 13 Ackermann closed-loop scenarios. The Ackermann set
  includes a run of the shipped example configuration; its narrow-corridor scenario bounds lateral error,
  curvature sign changes and stop ticks, its clutter scenario bounds clearance and stop ticks, and per-cycle
  curvature change is bounded in the offset-corridor and shipped-configuration runs. Thresholds are derived by
  sweeping **both** the correct and the broken band over the same perturbation grid; where no separating value
  exists no threshold is asserted, and the coverage moves to a threshold-free unit test instead (R14 M3, M4)
- A unit regression pinning the differential-drive output reachability stage (`test/output_stage_unit.cpp`):
  curvature-preserving deceleration, repeated correction, and an exact `(0,0)` with `STOP` when no rotation is
  admissible (R14 M1)
- Motion-model unit tests for differential drive and Ackermann; the Ackermann set covers the candidate lattice,
  the turning-radius bound, refinement, clearance probes, the deadband, a runtime model switch, and that the core
  stays usable after a rejected configuration
- Unit tests for scan projection and plan transform/pruning, plus integration checks for plugin lifecycle, TF
  errors, scan fallback, speed limits, and diagnostics
- 31 tests for the benchmark completeness checker
- 63 checks for runner orchestration
- Fail-fast expected-set, episode/trace schema, aggregate, and provenance promotion checks
- Domain manifest containing PID, domain, expected label, launch/reap interval, and exit status
- Provenance v2 reproducible from Git-tracked sources and its verification script
- Release archive script that creates the raw dataset and `SHA256SUMS` only after validation succeeds

## Canonical release evidence

- 18 scenarios × 3 runs × 4 controllers = 216 episodes
- BAC commit `1f9911e`; benchmark commit `026a17a`; zero dirty files in both worktrees; provenance v2
- All 90 domain IDs reused; 126 assignments after initial allocation; zero retained-interval overlap
- BAC 54/54 successful episodes, zero collisions, worst minimum clearance 0.136 m; DWB 50/54 with four
  collisions, MPPI 51/54, and RPP 47/54
- The same revisions produced the 32-episode offset and 24-episode opening sweeps; all 272 episodes have zero
  missing or corrupt records
- `release_archive_25f12be/` contains the earlier 272 episodes, the additional 432 episodes, both benchmark
  source generations, BAC source snapshots, and `SHA256SUMS`

All three datasets use provenance v2. `verify_provenance.py` reproduced 3/3 recorded digests: `bench_tree_sha`,
the Git tree object, and `worlds_sha`. The container image is
`sha256:d58fe8c8f5790cd000cf7bdc1b46395ac2567c231cd592ae6d29426ba9eb2737`.

## Additional P1 evaluation

- BAC `25f12be`, benchmark `4fed3d2`, and zero dirty files in both worktrees.
- Matched comparison: 18 scenarios × 3 runs × 4 controllers = 216 episodes. BAC 54/54, DWB 48/54 with two
  collisions, MPPI 51/54, and RPP 48/54.
- BAC ablation: 18 scenarios × 3 runs × 4 variants = 216 episodes. Every variant completed 54/54 without
  collision.
- Each dataset has 216 expected / 216 observed / zero missing, corrupt, or unexpected records, zero domain
  overlap, and 3/3 reproduced provenance digests.
- Removing the balance term degraded centering, clearance, or traversal time in narrow corridors. Raw scans
  were not necessary for completion, and escape contribution remains unidentified because baseline BAC did not
  select reverse.
- Details: [BAC ablation and matched-condition evaluation](ablation_and_matched_evaluation.md).
- Added a synchronized BAC/DWB replay of matched `appearing_obstacle/run1`, with the three-repeat aggregate and
  input/output hashes.
- Replaced the Gazebo evidence with one continuous avoidance, recovery, and 1.0 m-gate series. Minimum physical
  body clearance was 0.309 m, maximum lateral detour 0.799 m, in-gate center offset 0.013 m, body contacts zero,
  and final x 12.03 m; video, synchronized telemetry, nine-gate JSON, and harness are retained.

## Publication checklist

1. After publication, attach the archive, `SHA256SUMS`, source tag, and container image digest to the same GitHub
   Release.
2. State in the release notes that 0.1.0 is evaluated primarily in simulation and is not a physical-robot safety
   certification.
3. After tagging, verify source and archive accessibility from an external clone.
