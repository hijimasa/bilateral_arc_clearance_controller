^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bilateral_arc_clearance_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
