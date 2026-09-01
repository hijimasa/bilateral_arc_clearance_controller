^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bilateral_arc_clearance_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add a holonomic (omnidirectional) motion model, ``motion_model.type: omni``.
  Lateral velocity is the avoidance dimension, not yaw rate: the candidate
  lattice is forward speed x lateral speed, the same size as the
  differential-drive forward speed x yaw rate lattice. The yaw rate regulates
  the body onto the local path tangent and is fixed before candidate
  generation, so the trajectory that is scored and contact-checked is the one
  that is driven. In a passage the regulator also points the body into the gap
  rather than crabbing towards it, because a crabbing rectangle sweeps wider
  than a straight one. Requires a positive ``limits.vy_max`` and sensor
  coverage abeam the body, and it publishes ``cmd_vel.linear.y``, which the
  downstream base controller must honour.
* Honour the goal ORIENTATION Nav2 carries on the last plan pose. The adapter
  transforms it into the base frame and ``BacCore::process`` takes it as an
  optional argument; the pose reference fades from the path tangent to the goal
  orientation over the last 1.5 m and is fully governed by it within 0.5 m.
  Only the holonomic model can act on it - a model that steers with yaw cannot
  choose its orientation independently of its direction of travel - so
  differential drive and Ackermann ignore it and follow the path tangent
  exactly as before. Measured yaw error over goal orientations 0.0, -1.2, 1.5,
  2.5, -2.8 and 3.0 rad is 0.009-0.102 rad, inside the 0.25 rad
  ``yaw_goal_tolerance`` that Nav2's ``SimpleGoalChecker`` defaults to; the
  differential-drive reference spans 0.959-1.741 rad in the same runs.
* Generalise the swept-trajectory evaluator from "velocity is along body +x" to
  an arbitrary constant body twist. The centre of rotation moves from
  ``(0, v / w)`` to ``(-vy / w, v / w)`` and the footprint's leading, trailing
  and lateral extents become support functions of the direction of travel.
  Substituting ``vy = 0`` reproduces the previous closed forms exactly, so
  differential drive and Ackermann run the generalised code with byte-identical
  output rather than a preserved special case.
* ``Twist2D`` gains a ``vy`` field defaulting to zero, and the scorer and output
  stage carry a full body twist instead of a ``(v, w)`` scalar pair. Every
  non-holonomic model produces and consumes ``vy == 0``.
* Add deterministic holonomic unit and closed-loop regression tests plus an
  installable holonomic Nav2 configuration.
* Add an Ackermann motion model that samples body curvature within
  ``turn_radius_min``, never offers in-place rotation, and preserves the Nav2
  forward-speed/yaw-rate command contract. The vehicle model is described at
  the granularity of the Nav2 MPPI ``AckermannConstraints``; road-wheel
  kinematics belong to the downstream vehicle controller.
* Add deterministic Ackermann unit and closed-loop regression tests plus an
  installable Ackermann Nav2 configuration.
* Change the differential-drive output reachability stage, which every
  existing differential-drive user receives. When the one-cycle yaw limit
  changes the selected command and the clamped arc can then no longer stop
  before contact, the command is decelerated along its own curvature and the
  yaw limit and contact test are reapplied (up to eight times), instead of
  holding the yaw rate and lowering the speed once. A command that never
  becomes admissible brakes translation and retains only a reachable in-place
  rotation that is itself admissible. Measured per tick against the previous
  implementation with synchronised state on the shipped ``diff_drive``
  configuration, 1014 of 200000 sampled ticks differ (0.5070%); every observed
  difference is conservative (stop, drive slower, or give up the rotation). In
  closed loop, 9 of 10 worlds are bit-identical and the tenth deviates by at
  most 2 mm. ``test/output_stage_unit.cpp`` pins the new semantics.
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
  ``avoid_status`` may observe the changed value. This is the largest class of
  the output-stage difference measured above (977 of the 1014 differing
  ticks). The 17-scenario harness output is unchanged.
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
