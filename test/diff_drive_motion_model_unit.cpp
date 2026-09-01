/**
 * @file diff_drive_motion_model_unit.cpp
 * @brief Unit checks for the internal differential-drive policy seam
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "diff_drive_motion_model.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace
{

int failures = 0;

void
expect(bool condition, const std::string &message)
{
  if (!condition)
  {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

bool
near(float actual, float expected, float tolerance = 1e-5f)
{
  return std::fabs(actual - expected) <= tolerance;
}

void
testCandidateOrderingAndReachability()
{
  bac::Params params;
  bac::detail::DiffDriveMotionModel model(params);

  const bac::detail::CandidateBatch moving =
      model.sampleCandidates(bac::Twist2D(0.2f, 0.1f), 0.4f, 0.0f);
  expect(moving.commands.size() == 5 * 26,
         "moving batch keeps five linear rows by 26 angular samples");
  expect(near(moving.commands.front().v, 0.0f) && near(moving.commands.front().w, -1.0f),
         "stop/rotation row remains first");
  expect(near(moving.commands.back().v, 0.4f) && near(moving.commands.back().w, 0.0f),
         "dynamic-window cap and explicit straight command remain last");
  const std::vector<bac::Twist2D> refined =
      model.refinementCandidates(bac::Twist2D(0.2f, 0.0f));
  expect(refined.size() == 6U, "three refinement steps remain on each side");
  expect(near(refined.front().w, -(2.0f / 24.0f) / 4.0f),
         "refinement pitch matches the former in-core grid");

  const std::vector<bac::Twist2D> probes = model.clearanceProbeCommands(0.3f);
  expect(probes.size() == 3U && near(probes[0].w, -0.4f) &&
             near(probes[1].w, 0.0f) && near(probes[2].w, 0.4f),
         "tightness probes preserve their former order and yaw rates");

  const bac::detail::CandidateBatch standstill =
      model.sampleCandidates(bac::Twist2D(), 0.4f, 0.0f);
  expect(standstill.commands.size() == 6 * 26,
         "standstill batch keeps positive samples and two reverse escape rows");
  expect(near(standstill.commands[4 * 26].v, -0.1f) &&
             near(standstill.commands[5 * 26].v, -0.05f),
         "reverse escape rows remain last and ordered");
}

void
testConstantCommandProjection()
{
  const bac::Params params;
  const bac::detail::DiffDriveMotionModel model(params);

  const bac::detail::ProjectedPose2D straight =
      model.projectConstantCommand(bac::Twist2D(0.4f, 0.0f), 2.5f);
  expect(near(straight.x, 1.0f) && near(straight.y, 0.0f) && near(straight.theta, 0.0f),
         "straight projection preserves unicycle integration");

  const bac::detail::ProjectedPose2D curve =
      model.projectConstantCommand(bac::Twist2D(0.4f, 0.5f), 2.0f);
  expect(near(curve.theta, 1.0f), "curve projection integrates heading");
  expect(near(curve.x, 0.8f * std::sin(1.0f)) &&
             near(curve.y, 0.8f * (1.0f - std::cos(1.0f))),
         "curve projection preserves the constant-curvature endpoint");
}

void
testInPlaceRotationSweep()
{
  const bac::Params params;
  const bac::detail::DiffDriveMotionModel model(params);
  expect(model.isInPlaceRotationAdmissible({ { 2.0f, 0.0f } }),
         "far obstacle permits in-place rotation");
  expect(!model.isInPlaceRotationAdmissible({ { 0.55f, 0.0f } }),
         "point inside circumscribed sweep blocks in-place rotation");
  expect(model.supportsInPlaceRotation(), "differential drive supports in-place rotation");

  const bac::Twist2D limited = model.limitReachableCommand(
      bac::Twist2D(0.2f, 0.1f), bac::Twist2D(0.2f, 0.8f));
  expect(near(limited.w, 0.225f), "angular output reachability remains acceleration limited");
  const bac::Twist2D slowed = model.withLinearSpeed(limited, 0.1f);
  expect(near(slowed.v, 0.1f) && near(slowed.w, limited.w / 2.0f),
         "post-selection speed limit preserves differential-drive curvature");
  const bac::Twist2D stopped = model.withLinearSpeed(limited, 0.0f);
  expect(near(stopped.v, 0.0f) && near(stopped.w, 0.0f),
         "curvature-preserving stop does not create an in-place command");
}

}  // namespace

void
testTranslatingTurnRadiusGuard()
{
  const bac::Params params;
  const bac::detail::DiffDriveMotionModel model(params);

  // Documented in docs/parameters.md: for diff_drive, turn_radius_min bounds
  // TRANSLATING candidates (forward and reverse), and only the in-place
  // rotation row is exempt.
  expect(!model.isCommandKinematicallyValid({ 0.1f, 0.1f / (params.turn_radius_min * 0.5f) }),
         "a forward arc tighter than turn_radius_min is rejected");
  expect(!model.isCommandKinematicallyValid({ -0.1f, -0.1f / (params.turn_radius_min * 0.5f) }),
         "a reverse arc tighter than turn_radius_min is rejected");
  expect(model.isCommandKinematicallyValid({ 0.1f, 0.1f / (params.turn_radius_min * 2.0f) }),
         "a forward arc wider than turn_radius_min is accepted");
  expect(model.isCommandKinematicallyValid({ 0.0f, 0.8f }),
         "in-place rotation is exempt from the turning-radius guard");
}

int
main()
{
  testTranslatingTurnRadiusGuard();
  testCandidateOrderingAndReachability();
  testConstantCommandProjection();
  testInPlaceRotationSweep();

  if (failures != 0)
  {
    std::cerr << failures << " diff-drive motion-model check(s) failed\n";
    return 1;
  }
  std::cout << "All diff-drive motion-model checks passed\n";
  return 0;
}
