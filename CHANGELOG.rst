^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bilateral_arc_clearance_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add an Ackermann motion model that samples body curvature within
  ``turn_radius_min``, never offers in-place rotation, and preserves the Nav2
  forward-speed/yaw-rate command contract. The vehicle model is described at
  the granularity of the Nav2 MPPI ``AckermannConstraints``; road-wheel
  kinematics belong to the downstream vehicle controller.
* Add deterministic Ackermann unit and closed-loop regression tests plus an
  installable Ackermann Nav2 configuration.
* Preserve constant-command curvature when contact rechecking lowers speed,
  then reapply yaw reachability and contact checks when needed.
* Bind the motion model once per configuration instead of per control tick, so
  an unusable kinematic configuration is rejected by ``setParams`` and
  ``process`` neither allocates nor throws.
* Validate a motion-model configuration before committing it. A rejected
  ``setParams`` previously left the surviving model reading the rejected
  parameters, where a non-positive ``turn_radius_min`` turned the Ackermann
  steering clamp into a full-lock command instead of a clean failure.
* Differential drive no longer emits an in-place rotation that was never
  checked for admissibility when the output stage brakes to zero speed. Such a
  tick now reports ``STOP`` rather than ``AVOIDING``; subscribers of
  ``avoid_status`` may observe the changed value. The 17-scenario harness
  output is unchanged.
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
