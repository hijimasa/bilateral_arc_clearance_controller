# Public-release readiness checklist

English | [日本語](../public_release_checklist.md)

- Last updated: 2026-08-29
- Target: `bilateral_arc_clearance_controller` 0.1.0
- Policy: keep the repository private until the private-preparation gates below are complete

This checklist separates technical publication value from the ability of a ROS user to install, inspect, and
reproduce the package. “Implemented; confirmation pending” means that the file exists but its first hosted CI run
or release-candidate exercise is still required.

## P0: required before changing visibility

| State | Item | Completion condition |
|---|---|---|
| Done | Bounded claims | README and theory documents distinguish simulation results, assumptions, and non-guarantees |
| Done | License and authorship | MIT LICENSE, source headers, and personal metadata in `package.xml` agree |
| Done | Minimum integration material | Installed Nav2 config, filter config, and filter launch examples exist |
| Done | Framework-free regression tests | Core, scenarios, and scan/path adapter CTest entries pass |
| Done | GitHub Actions | First green GCC, Clang, ASan/UBSan, and ROS 2 Jazzy/Nav2 jobs confirmed on 2026-08-29 |
| Done | Nav2 adapter integration tests | Exercise lifecycle, TF failure, stale/invalid scan, costmap fallback, and speed limits |
| Done | Confidentiality and rights audit | Tracked history contains no secret, internal path, third-party data, or non-redistributable artifact |
| Done | Benchmark release archive | `release_archive_1f9911e/` fixes source, image digest, 272 raw/summary episodes, and SHA256 |
| Done | Release-candidate dry run | A clean-clone-equivalent Jazzy/Nav2 environment passes build, test, install, and launch-file load |

Do not change repository visibility until every P0 item is complete. Physical-robot evidence is not an absolute
blocker for a 0.1.0 source release, but without it the release notes must label the version a simulation-first
preview.
All P0 items were complete on 2026-08-29, so the repository is technically ready to make public. The hosted CI
jobs were all green after push. The benchmark archive records BAC `1f9911e`, benchmark `026a17a`, ROS 2 Jazzy /
Nav2 1.3.12, and the container-image digest, with zero missing or corrupt episodes across the canonical 216,
offset-sweep 32, and opening-sweep 24 episodes. Visibility, tagging, and GitHub Release creation remain explicit
owner actions.
The history audit found no secret, private key, local home path, or large binary. Git commit metadata retains
`hijimasa@gmail.com`; revisit the history policy before publication only if that address is not intended as part
of the public identity.

## P1: strongly recommended for impact and trust

| State | Item | Purpose |
|---|---|---|
| Done | ROS-boundary isolation | Scan projection and plan transform/pruning are shared and lifecycle/fallback behavior is integration-tested |
| Done | Input-source diagnostics | Report raw scan use, costmap fallback and reason, and candidate/admissible counts |
| Open | RViz debug output | Optionally show the selected arc, preview goal, and stop/avoid state |
| Open | Matched comparison | Match scan/costmap update rates, reversing, speed, and acceleration conditions |
| Open | BAC ablation | Isolate `weights.balance=0`, escape disabled, and raw-scan versus costmap behavior |
| Open | Current baselines | Include current RPP, MPPI, DWB, and a Collision Monitor combined condition where relevant |
| Open | Physical-robot evidence | Record localization offset, appearing obstacle, and impassable-gap stop/escape cases with logs |
| Open | Performance budget in CI | Set a core regression threshold that accounts for runner noise |

For every comparison, retain a table of what is matched and what necessarily differs by controller. Three
deterministic repetitions demonstrate reproducibility; they are not independent statistical samples.

## P2: continuing project operation

- Add `QUALITY_DECLARATION.md` with the actual REP-2004 level and unmet requirements.
- Define versioning, parameter deprecation, and supported ROS distribution policies.
- Add issue and release-note templates and a maintenance-response policy.
- Produce coverage and static-analysis artifacts before advertising their values.
- Treat decomposition of `BacCore::process()` and rollout through angular-acceleration transients as next-cycle work.

## Actions that require public visibility

1. Change repository visibility to Public.
2. Enable branch protection and required CI checks.
3. Create a signed or annotated `v0.1.0` tag and GitHub Release with the verified archive.
4. Submit the ROS Index/rosdistro release.
5. Propose the package for Nav2's known-plugins list with its public URL, example, and video.
6. Verify repository, issue-tracker, and release-artifact links from an external clone.

## Decision rule

- **Ready to publish**: every P0 item is complete and unfinished P1 work is explicit in README and release notes.
- **Delay publication**: CI, adapter fail-safe behavior, benchmark provenance, or the rights/confidentiality audit is unverified.
- **Ready for an impact-oriented launch**: the publication gate plus a matched comparison, ablation, and at least one physical-robot video series.

The package has sufficient technical value to publish. Its strongest launch will pair the source with a
reproducible comparison, observable input-state behavior, and physical evidence whose limitations are explicit.
