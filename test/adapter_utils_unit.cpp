/**
 * @file adapter_utils_unit.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "bilateral_arc_clearance_controller/adapter_utils.hpp"
#include "test_expect.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace
{

using bac_test::expect;
using bac_test::failures;
using bac_test::near;

void testScanValidityAndProjection()
{
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<float> ranges{ 1.0f, inf, 12.0f, nan, 0.05f, -inf };

  const bac::ScanProjection projected =
      bac::projectScan(ranges, 0.0f, 0.5f, 0.1f, 10.0f, 5.0f, 1, true);
  expect(projected.valid_ray_count == 3U,
         "finite hit, positive infinity, and above-range sample are valid");
  expect(projected.points.size() == 1U, "only an in-range obstacle creates a point");
  expect(near(projected.points[0].x, 1.0f) && near(projected.points[0].y, 0.0f),
         "scan point uses the sample angle");

  const bac::ScanProjection strict =
      bac::projectScan(ranges, 0.0f, 0.5f, 0.1f, 10.0f, 5.0f, 1, false);
  expect(strict.valid_ray_count == 1U,
         "clear no-return measurements are invalid when inf_is_valid is disabled");
}

void testScanDownsampleAndExtrinsics()
{
  const std::vector<float> ranges{ 1.0f, 1.0f, 2.0f, 1.0f };
  const bac::ScanProjection projected =
      bac::projectScan(ranges, 0.0f, 0.5f, 0.1f, 10.0f, 1.5f, 2, true,
                       1.0f, -1.0f, 0.5f);
  expect(projected.valid_ray_count == 2U, "validity count follows the downsample stride");
  expect(projected.points.size() == 1U, "obstacles beyond the BAC range are omitted");
  expect(near(projected.points[0].x, 1.0f + std::cos(0.5f)) &&
             near(projected.points[0].y, -1.0f + std::sin(0.5f)),
         "fixed sensor extrinsics are applied in the robot frame");
}

void testTransformAndPrunePath()
{
  const std::vector<bac::Point2D> path{
    { -3.0f, 0.0f }, { -1.0f, 0.0f }, { 0.0f, 0.0f },
    { 1.0f, 0.0f }, { 3.0f, 0.0f }
  };
  const std::vector<bac::Point2D> local =
      bac::transformAndPrunePath(path, 0.0f, 0.0f, 0.0f, 1.5f);
  expect(local.size() == 2U, "path starts at its robot-nearest point and stops at range");
  expect(near(local[0].x, 0.0f) && near(local[1].x, 1.0f),
         "the stale prefix is pruned without dropping the local suffix");

  constexpr float half_pi = 1.57079632679f;
  const std::vector<bac::Point2D> rotated =
      bac::transformAndPrunePath({ { 1.0f, 0.0f }, { 2.0f, 0.0f } },
                                 1.0f, 2.0f, half_pi, 5.0f);
  expect(rotated.size() == 2U, "transformed path remains within the local range");
  expect(near(rotated[0].x, 1.0f) && near(rotated[0].y, 3.0f),
         "plan-to-base rigid transform is applied");
}

/// The orientations a plan carries have to survive pruning ATTACHED to their
/// own points. Both ends of the window move - the stale prefix is dropped and
/// the far end is cut at max_range - so an orientation sequence sliced by any
/// rule but the pruner's own would silently pair a point with another point's
/// orientation, and the body would be steered by it.
void
testTransformAndPrunePathCarriesOrientations()
{
  const std::vector<bac::Point2D> path{
    { -3.0f, 0.0f }, { -1.0f, 0.0f }, { 0.0f, 0.0f },
    { 1.0f, 0.0f }, { 3.0f, 0.0f }
  };
  const std::vector<float> plan_yaw{ 0.1f, 0.2f, 0.3f, 0.4f, 0.5f };

  {
    std::vector<float> pruned;
    const std::vector<bac::Point2D> local =
        bac::transformAndPrunePath(path, 0.0f, 0.0f, 0.0f, 1.5f, &plan_yaw, &pruned);
    expect(local.size() == 2U && pruned.size() == 2U,
           "orientations are retained in the same count as the points");
    // Indices 2 and 3 survive; 0.3 and 0.4 are THEIR orientations, and no
    // other pair of adjacent values would be wrong by an amount a closed-loop
    // run could distinguish from tracking lag.
    expect(near(pruned[0], 0.3f) && near(pruned[1], 0.4f),
           "each retained point keeps its own orientation across both cuts");
  }

  {
    // The transform rotates the plan frame into the base frame, so every
    // orientation gains that rotation - the same relation goalHeadingInBase
    // applies. A sign error here aims the body at a mirrored orientation.
    // A quarter turn puts the path on the base frame's y axis, so index 2 is
    // still the nearest and 2..4 all fall inside 5 m: three points, and the
    // orientations that come back are theirs.
    constexpr float half_pi = 1.57079632679f;
    std::vector<float> pruned;
    const std::vector<bac::Point2D> local =
        bac::transformAndPrunePath(path, 0.0f, 0.0f, half_pi, 5.0f, &plan_yaw, &pruned);
    expect(local.size() == 3U && pruned.size() == 3U,
           "the rotated prune keeps three points and three orientations");
    expect(near(pruned[0], 0.3f + half_pi) && near(pruned[2], 0.5f + half_pi),
           "plan-frame orientations are rotated into the base frame");
  }

  {
    // A mismatched count cannot be repaired - which end is missing decides
    // which point every remaining orientation belongs to.
    std::vector<float> pruned{ 9.0f, 9.0f };
    const std::vector<float> short_yaw{ 0.1f, 0.2f };
    bac::transformAndPrunePath(path, 0.0f, 0.0f, 0.0f, 1.5f, &short_yaw, &pruned);
    expect(pruned.empty(), "a mismatched orientation count yields none at all");
  }

  {
    // An empty plan returns early. The buffer is a caller's, reused every
    // tick: left untouched it would hand back the previous tick's sequence.
    std::vector<float> pruned{ 9.0f };
    const std::vector<float> none;
    bac::transformAndPrunePath({}, 0.0f, 0.0f, 0.0f, 1.5f, &none, &pruned);
    expect(pruned.empty(), "an empty plan clears the caller's orientation buffer");
  }
}


/// Nav2 carries the goal orientation on the last plan pose. It is a goal
/// orientation only while that pose is still on the pruned path, and it has to
/// be rotated into the base frame - a sign error here would aim the vehicle at
/// a mirrored heading, which no closed-loop scenario run in the base frame can
/// see, because the scenarios never exercise a plan frame offset from it.
void
testGoalHeadingInBase()
{
  const bac::Point2D plan_end(3.0f, 1.0f);

  {
    const std::vector<bac::Point2D> local_path = { { 1.0f, 0.0f }, { 3.0f, 1.0f } };
    const auto heading = bac::goalHeadingInBase(plan_end, local_path, 0.0f, 0.0f, 0.0f, 0.7f);
    expect(heading.has_value(), "an unpruned goal yields a goal heading");
    expect(heading && near(*heading, 0.7f),
           "an identity transform passes the plan orientation through");
  }

  {
    const float yaw = 0.5f;
    const float cs = std::cos(yaw), sn = std::sin(yaw);
    const bac::Point2D end_in_base(2.0f + cs * plan_end.x - sn * plan_end.y,
                                   -1.0f + sn * plan_end.x + cs * plan_end.y);
    const std::vector<bac::Point2D> local_path = { { 0.0f, 0.0f }, end_in_base };
    const auto heading = bac::goalHeadingInBase(plan_end, local_path, 2.0f, -1.0f, yaw, 0.7f);
    expect(heading.has_value(), "a rotated frame still yields a goal heading");
    expect(heading && near(*heading, 0.7f + yaw),
           "the plan orientation gains the transform rotation (got " +
               std::to_string(heading ? *heading : 0.0f) + ", expected " +
               std::to_string(0.7f + yaw) + ")");
  }

  {
    const std::vector<bac::Point2D> local_path = { { 1.0f, 0.0f }, { 2.0f, 0.5f } };
    const auto heading = bac::goalHeadingInBase(plan_end, local_path, 0.0f, 0.0f, 0.0f, 0.7f);
    expect(!heading.has_value(), "a pruned plan end is a waypoint and yields no goal heading");
  }

  {
    const auto heading = bac::goalHeadingInBase(plan_end, {}, 0.0f, 0.0f, 0.0f, 0.7f);
    expect(!heading.has_value(), "an empty path yields no goal heading");
  }
}

}  // namespace

int main()
{
  testScanValidityAndProjection();
  testScanDownsampleAndExtrinsics();
  testTransformAndPrunePath();
  testTransformAndPrunePathCarriesOrientations();
  testGoalHeadingInBase();

  if (failures != 0)
  {
    std::cerr << failures << " adapter utility check(s) failed\n";
    return 1;
  }
  std::cout << "All BAC adapter utility checks passed\n";
  return 0;
}
