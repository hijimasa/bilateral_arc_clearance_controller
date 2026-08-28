# Release review history

English | [日本語](../release_review_history.md)

## Current state

- Checked: 2026-08-28
- Package: `dcba783`, documentation baseline; controller implementation unchanged from `59a78d3`
- Benchmark: `604780e`
- Decision: **Go**

All Critical, High, and Medium findings across ten release-review rounds, plus the three Low findings from R10,
have been addressed. No current release blocker remains. Follow-up work for the next cycle includes ROS adapter
tests, rollout that integrates the angular-acceleration transient, physical disturbance evaluation, a Collision
Monitor combined baseline, and decomposition of `BacCore::process()`.

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

- Plain CMake Release build and two CTest entries
- Core unit/property tests and 13 closed-loop scenarios
- 31 tests for the benchmark completeness checker
- 63 checks for runner orchestration
- Fail-fast expected-set, episode/trace schema, aggregate, and provenance promotion checks
- Domain manifest containing PID, domain, expected label, launch/reap interval, and exit status
- Provenance v2 reproducible from Git-tracked sources and its verification script
- Release archive script that creates the raw dataset and `SHA256SUMS` only after validation succeeds

## Canonical release evidence

- 18 scenarios × 3 runs × 4 controllers = 216 episodes
- BAC commit `f1e2a90`; benchmark commit `13becc0`; zero dirty files in both worktrees
- All 90 domain IDs reused; 126 assignments after initial allocation; zero retained-interval overlap
- BAC 54/54 successful episodes, zero collisions, worst minimum clearance 0.139 m

The canonical dataset uses provenance v1. Its `bench_tree_sha` included ignored generated files and therefore
cannot be reconstructed from the commit alone. Source identity is instead tracked by `bench_commit=13becc0`,
`bench_dirty=0`, and the matching `worlds_sha`. New runs use provenance v2, which hashes Git-tracked files only.
See the R10 response for details.

## Publication checklist

1. Validate and package the canonical, offset-sweep, and opening-sweep datasets with
   `nav2_benchmark/scripts/make_release_archive.sh`.
2. Attach the archive, `SHA256SUMS`, source tag, and container image digest to the same release.
3. Compare README and method-comparison values with the archived summary.
4. State in the release notes that 0.1.0 is evaluated primarily in simulation and is not a physical-robot safety
   certification.
