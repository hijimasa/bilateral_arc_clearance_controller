# Contributing

Thank you for helping improve Bilateral Arc Clearance. Changes should keep the framework-free core independently
buildable and should not broaden safety or robustness claims beyond the available evidence.

## Development checks

Run the core tests without ROS:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

In a ROS 2 Jazzy workspace, also run:

```bash
colcon build --packages-select bilateral_arc_clearance_controller
colcon test --packages-select bilateral_arc_clearance_controller
colcon test-result --verbose
```

Changes to scan projection, TF/path adaptation, collision geometry, admissibility, candidate selection, or state
latching require a focused regression test. Changes to default parameters also require the strict scenario harness.

## Pull requests

- Keep planning logic in the ROS-independent library; keep ROS classes as adapters.
- Update English and Japanese user documents together when behavior or parameters change.
- State the test environment and exact commands in the pull request.
- Separate benchmark observations from algorithmic properties and physical-robot guarantees.
- Do not commit generated build trees, trace output, credentials, or private robot/site data.

See the [review-record policy](docs/reviews/README.en.md) for formal release reviews.
