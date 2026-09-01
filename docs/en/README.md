# Documentation

English | [日本語](../README.md)

This index separates current user documentation from pre-release review records.

## User documentation

- [Algorithm and claim boundaries](algorithm.md): inputs, candidate evaluation, algorithmic properties,
  assumptions, and non-guarantees
- [Nav2 integration guide](nav2_integration.md): division of responsibilities among the Controller Server,
  Collision Monitor, costmaps, and related extension points
- [Parameter reference](parameters.md): all settings for the core, Nav2 plugin, and filter node
- [Method comparison and evaluation](method_comparison.md): comparison with DWB, MPPI, RPP, VFH/ND, and
  raw-derived benchmark results
- [BAC ablation and matched-condition evaluation](ablation_and_matched_evaluation.md): common conditions,
  432-episode results, causal limitations, and synchronized Gazebo evidence
- [Gazebo reproduction harness](../../examples/gazebo/README.md): Docker, world, URDF, capture, and automated checks

## Development and release documentation

- [Public-release readiness checklist](public_release_checklist.md): private preparation, publication gates, and
  post-publication actions
- [Release review history](release_review_history.md): summary of every review round, their resolutions,
  and remaining follow-up work
- [Review-record policy](../reviews/README.en.md): naming, metadata, and update rules
- `docs/reviews/rNN-YYYY-MM-DD-{findings,response}.md`: Japanese audit records preserving the evidence and
  reasoning at each review point

Individual review records may contain conclusions that were later corrected or withdrawn. For current behavior
and claims, use the root README, the user documentation above, and the latest state in the release review history.

## Figure

- [BAC geometry](../images/bac_geometry.svg): candidate arc, path projection, and bilateral clearance
- [Matched appearing-obstacle trajectories](../images/matched_appearing_obstacle.png): BAC/DWB/MPPI/RPP overlay
- [Extreme-offset ablation trajectories](../images/ablation_extreme_offset.png): four BAC variants
- [BAC-versus-DWB side-by-side GIF](../media/bac_vs_dwb_matched_appearing_obstacle_preview.gif)
  ([higher-quality MP4](../media/bac_vs_dwb_matched_appearing_obstacle.mp4)): synchronized replay of matched benchmark run 1
- [Gazebo adaptive-clearance GIF](../media/bac_gazebo_adaptive_clearance_preview.gif)
  ([higher-quality MP4](../media/bac_gazebo_adaptive_clearance.mp4)): one continuous avoidance, recovery, and 1.0 m-gate run
