# Release review history

English | [日本語](../release_review_history.md)

## Current state

- Checked: 2026-08-29
- Evaluated package: `25f12be` (implementation identical to `1f9911e`; intervening commits are documentation and release preparation)
- Benchmark: `026a17a` (canonical) / `4fed3d2` (matched comparison and ablation)
- Decision: code review **Go** and public-release P0 **complete**; changing visibility remains an owner decision

All Critical, High, and Medium findings across ten release-review rounds, plus the three Low findings from R10,
have been addressed. The 2026-08-29 follow-up added ROS adapter tests, input-source diagnostics, a 216-episode
matched-condition comparison, and a 216-episode BAC ablation. Remaining work includes rollout through angular-
acceleration transients, physical disturbance evaluation, a Collision Monitor combined baseline,
and decomposition of `BacCore::process()`.

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

## Current validation contract

- Plain CMake Release build and three CTest entries; four CTest entries in ROS 2 Jazzy/Nav2 with the adapter test
- Core unit/property tests and 13 closed-loop scenarios
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
