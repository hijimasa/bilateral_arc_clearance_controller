^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bilateral_arc_clearance_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Extract differential-drive candidate generation, constant-command rollout,
  and in-place rotation policy from ``BacCore`` as a motion-model boundary.
* Extract constant-curvature bilateral-clearance and exact swept-footprint
  evaluation so it can be reused by future non-differential-drive policies.
* Add focused regression tests for the differential-drive motion-model seam.
* Add a reproducible ROS 2 Jazzy/Nav2 Docker build that verifies the complete
  controller plugin and ROS adapter tests without writing into the checkout.

0.1.0 (2026-08-27)
------------------
* Add the framework-independent BAC core, Nav2 controller plugin, and ROS 2
  velocity-filter evaluation node.
* Use a local path as intent and bilateral arc clearance for narrow-passage
  centering, with emergency stopping and DWA admissibility.
* Add closed-loop scenario and core geometry tests.
* Add algorithm, parameter, method-comparison, and benchmark documentation.
* Scope localization-drift and replanning-delay claims to the tested conditions,
  with explicit assumptions and non-guarantees.
* Split the concise README from the algorithm, Nav2 integration, comparison,
  and release-review history documents.
* Archive individual review records under stable IDs and document their
  naming, metadata, and update rules.
* Add an English release README and English user documentation while retaining
  the original Japanese documents and audit records.
* Add installable Nav2/filter configurations, a filter launch file, hosted CI,
  contribution and security policies, and a bilingual public-release checklist.
* Share and test scan projection and plan transformation geometry across ROS
  adapters, and publish obstacle-source/fallback state through diagnostics.
* Validate the release candidate with green hosted CI and archive 272 Nav2
  benchmark episodes with clean source provenance and checksums. Document the
  observed 1.15 m opening timeout as a configuration limit.
