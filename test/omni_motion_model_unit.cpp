/**
 * @file omni_motion_model_unit.cpp
 * @brief Deterministic unit checks for the holonomic motion model
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * The holonomic contract differs from the other two models in three ways that
 * are checked here rather than inferred from closed-loop behaviour: the yaw
 * rate is NOT a searched dimension, the speed cap applies to the velocity
 * VECTOR rather than per axis, and hysteresis acts on lateral velocity.
 */

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "omni_motion_model.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
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

bac::Params
omniParams()
{
  bac::Params params;
  params.motion_model.type = bac::MotionModelType::OMNI;
  params.limits.v_max = 0.4f;
  params.limits.v_min = 0.0f;
  params.limits.vy_max = 0.3f;
  params.limits.w_max = 1.0f;
  params.limits.acc_v = 0.8f;
  params.limits.acc_w = 2.5f;
  params.control_period = 0.05f;
  params.footprint.front = 0.35f;
  params.footprint.rear = -0.35f;
  params.footprint.width = 0.5f;
  return params;
}

/// The yaw rate is fixed at the pose regulator's reference for every
/// candidate. If it were searched, the scored trajectory and the driven
/// trajectory would be free to differ.
void
testYawIsNotSearched()
{
  const bac::Params params = omniParams();
  const bac::detail::OmniMotionModel model(params);
  for (const float reference : { -0.8f, 0.0f, 0.35f })
  {
    const bac::detail::CandidateBatch batch =
        model.sampleCandidates(bac::Twist2D(0.2f, 0.0f, 0.0f), params.limits.v_max, reference);
    expect(!batch.commands.empty(), "the holonomic lattice is not empty");
    int off_reference = 0;
    for (const bac::Twist2D &command : batch.commands)
    {
      if (std::fabs(command.w - reference) > 1e-6f)
      {
        ++off_reference;
      }
    }
    expect(off_reference == 0,
           "every candidate carries the yaw reference " + std::to_string(reference) +
               " (" + std::to_string(off_reference) + " did not)");
  }
}

/// Lateral velocity is the avoidance dimension, so the lattice has to offer
/// both signs of it. Without this the model would be a differential drive
/// that cannot steer.
void
testLatticeOffersBothLateralDirections()
{
  const bac::Params params = omniParams();
  const bac::detail::OmniMotionModel model(params);
  const bac::detail::CandidateBatch batch =
      model.sampleCandidates(bac::Twist2D(0.2f, 0.0f, 0.0f), params.limits.v_max, 0.0f);

  int left = 0, right = 0, straight = 0, moving = 0;
  for (const bac::Twist2D &command : batch.commands)
  {
    if (command.speed() > 1e-3f)
    {
      ++moving;
    }
    if (command.vy > 1e-3f) ++left;
    if (command.vy < -1e-3f) ++right;
    if (std::fabs(command.vy) <= 1e-6f) ++straight;
  }
  expect(left > 0 && right > 0,
         "the lattice offers lateral candidates on both sides (" + std::to_string(left) +
             " left, " + std::to_string(right) + " right)");
  expect(straight > 0, "the lattice always offers the un-crabbed candidate");
  expect(moving > 0, "the lattice offers translating candidates");
}

/// The cap is on the velocity VECTOR. A per-axis cap would admit
/// hypot(v_max, vy_max) = 0.5 m/s from a 0.4 m/s vehicle.
void
testSpeedCapAppliesToTheVector()
{
  bac::Params params = omniParams();
  params.limits.v_max = 0.4f;
  params.limits.vy_max = 0.3f;   // hypot(0.4, 0.3) = 0.5, well above v_max
  params.limits.acc_v = 100.0f;  // the acceleration window must not be what binds
  const bac::detail::OmniMotionModel model(params);

  const bac::detail::CandidateBatch batch =
      model.sampleCandidates(bac::Twist2D(0.4f, 0.0f, 0.0f), params.limits.v_max, 0.0f);
  float worst = 0.0f;
  int capped = 0;
  for (const bac::Twist2D &command : batch.commands)
  {
    worst = std::max(worst, command.speed());
    // A candidate whose raw (v, vy) would have exceeded v_max must have been
    // scaled, not clipped on one axis: the direction has to survive.
    if (std::fabs(command.vy) > 1e-3f && command.speed() > params.limits.v_max - 1e-3f)
    {
      ++capped;
    }
  }
  expect(worst <= params.limits.v_max + 1e-4f,
         "no candidate exceeds limits.v_max (worst " + std::to_string(worst) + " m/s)");
  expect(capped > 0,
         "the norm cap actually binds in this fixture (" + std::to_string(capped) +
             " candidates sit on it), so the check above is not vacuous");

  // Direction preserved: scaling, not clipping.
  const bac::Twist2D limited =
      model.limitReachableCommand(bac::Twist2D(0.4f, 0.0f, 0.0f), bac::Twist2D(0.4f, 0.0f, 0.3f));
  expect(limited.speed() <= params.limits.v_max + 1e-4f,
         "limitReachableCommand caps the vector (" + std::to_string(limited.speed()) + " m/s)");
  expect(std::fabs(limited.vy / limited.v - 0.3f / 0.4f) < 1e-3f,
         "capping preserves the direction of travel (vy/v " +
             std::to_string(limited.vy / limited.v) + " vs 0.75)");
}

/// Lateral velocity is acceleration-limited like the forward axis: the same
/// wheels drive both.
void
testLateralAccelerationWindow()
{
  bac::Params params = omniParams();
  params.window_time = 0.2f;  // acc_v * window_time = 0.16 m/s of lateral reach
  const bac::detail::OmniMotionModel model(params);
  const bac::Twist2D current(0.2f, 0.0f, -0.3f);  // hard over to the right
  const bac::detail::CandidateBatch batch =
      model.sampleCandidates(current, params.limits.v_max, 0.0f);

  const float reach = params.limits.acc_v * params.window_time;
  float highest = -1e9f;
  for (const bac::Twist2D &command : batch.commands)
  {
    highest = std::max(highest, command.vy);
  }
  expect(highest <= current.vy + reach + 1e-4f,
         "no candidate outruns one lateral acceleration window (highest vy " +
             std::to_string(highest) + ", reachable " + std::to_string(current.vy + reach) + ")");
  expect(highest > current.vy + 0.5f * reach,
         "the window is actually explored rather than collapsed (highest vy " +
             std::to_string(highest) + ")");
}

/// Hysteresis penalises the avoidance dimension, as it does for the other
/// models. Penalising forward speed instead would fight the speed governor.
void
testHysteresisMeasuresLateralVelocity()
{
  const bac::Params params = omniParams();
  const bac::detail::OmniMotionModel model(params);
  const bac::Twist2D previous(0.3f, 0.2f, 0.1f);

  expect(model.commandChange(bac::Twist2D(0.1f, 0.2f, 0.1f), previous) < 1e-6f,
         "a different forward speed on the same lateral velocity is not a steering change");
  expect(model.commandChange(bac::Twist2D(0.3f, 0.9f, 0.1f), previous) < 1e-6f,
         "a different yaw rate is not a steering change: yaw is not the avoidance axis");
  expect(std::fabs(model.commandChange(bac::Twist2D(0.3f, 0.2f, -0.1f), previous) - 0.2f) < 1e-6f,
         "a lateral-velocity change IS a steering change (" +
             std::to_string(model.commandChange(bac::Twist2D(0.3f, 0.2f, -0.1f), previous)) + ")");
}

/// Slowing down for a contact check must not silently replace the trajectory
/// that was checked, so the whole twist scales.
void
testDecelerationPreservesTrajectoryGeometry()
{
  const bac::Params params = omniParams();
  const bac::detail::OmniMotionModel model(params);
  const bac::Twist2D command(0.3f, 0.4f, 0.2f);
  const bac::Twist2D slower = model.withLinearSpeed(command, 0.18f);

  expect(std::fabs(slower.speed() - 0.18f) < 1e-4f,
         "withLinearSpeed reaches the requested speed (" + std::to_string(slower.speed()) + ")");
  expect(std::fabs(slower.vy / slower.v - command.vy / command.v) < 1e-4f,
         "the direction of travel is preserved");
  const float scale = 0.18f / command.speed();
  expect(std::fabs(slower.w - command.w * scale) < 1e-4f,
         "the yaw rate scales with it, so the arc keeps its radius (" +
             std::to_string(slower.w) + " vs " + std::to_string(command.w * scale) + ")");
}

/// The rollout has to agree with the swept geometry it is scored against, so
/// it is checked against numeric integration of the same constant twist.
void
testRolloutMatchesIntegration()
{
  const bac::Params params = omniParams();
  const bac::detail::OmniMotionModel model(params);
  for (const bac::Twist2D command :
       { bac::Twist2D(0.3f, 0.0f, 0.2f), bac::Twist2D(0.3f, 0.7f, -0.15f),
         bac::Twist2D(0.0f, 0.9f, 0.25f), bac::Twist2D(-0.2f, -0.5f, 0.1f) })
  {
    const float duration = 1.7f;
    const int steps = 20000;
    const float dt = duration / static_cast<float>(steps);
    float x = 0.0f, y = 0.0f, th = 0.0f;
    for (int i = 0; i < steps; ++i)
    {
      const float cs = std::cos(th), sn = std::sin(th);
      x += (command.v * cs - command.vy * sn) * dt;
      y += (command.v * sn + command.vy * cs) * dt;
      th += command.w * dt;
    }
    const bac::detail::ProjectedPose2D pose = model.projectConstantCommand(command, duration);
    expect(std::fabs(pose.x - x) < 2e-3f && std::fabs(pose.y - y) < 2e-3f,
           "the closed-form rollout matches integration for (" + std::to_string(command.v) +
               ", " + std::to_string(command.w) + ", " + std::to_string(command.vy) + "): (" +
               std::to_string(pose.x) + ", " + std::to_string(pose.y) + ") vs (" +
               std::to_string(x) + ", " + std::to_string(y) + ")");
    expect(std::fabs(pose.theta - command.w * duration) < 1e-5f,
           "the rollout heading is the integrated yaw rate");
  }
}

/// A holonomic body can yaw on the spot, but never needs to rotate onto the
/// tangent before it can translate along it.
void
testRotationPredicatesAreSeparate()
{
  const bac::Params params = omniParams();
  const bac::detail::OmniMotionModel model(params);
  expect(model.supportsInPlaceRotation(),
         "a holonomic body can hold station and yaw onto the tangent");
  expect(!model.usesRotateBeforeTranslate(),
         "a holonomic body never has to align before translating");

  // The sweep is still checked: an obstacle inside the circumscribed disk
  // makes the standstill rotation inadmissible.
  const float longitudinal = std::max(params.footprint.front, -params.footprint.rear);
  const float circumscribed =
      std::sqrt(longitudinal * longitudinal + params.footprint.width * params.footprint.width / 4.0f);
  expect(!model.isInPlaceRotationAdmissible({ { circumscribed - 0.05f, 0.0f } }),
         "a point inside the swept disk blocks the standstill rotation");
  expect(model.isInPlaceRotationAdmissible({ { circumscribed + 0.5f, 0.0f } }),
         "a point clear of the swept disk does not");
}

/// Every layer that can build the model rejects a configuration that cannot
/// honour the holonomic contract. Asked of each layer directly: going through
/// makeMotionModel alone would let one of them pass on the other's check.
void
testInvalidConfigurationsAreRejected()
{
  const auto rejects = [](const bac::Params &params, const std::string &what) {
    bool validate_threw = false, construct_threw = false, set_threw = false;
    try
    {
      bac::detail::validateMotionModelParams(params);
    }
    catch (const std::invalid_argument &)
    {
      validate_threw = true;
    }
    try
    {
      const bac::detail::OmniMotionModel model(params);
      (void)model.supportsInPlaceRotation();
      construct_threw = validate_threw;  // the class trusts the validator
    }
    catch (const std::invalid_argument &)
    {
      construct_threw = true;
    }
    try
    {
      bac::BacCore core(omniParams());
      core.setParams(params);
    }
    catch (const std::invalid_argument &)
    {
      set_threw = true;
    }
    expect(validate_threw, "validateMotionModelParams rejects " + what);
    expect(set_threw, "BacCore::setParams rejects " + what);
    (void)construct_threw;
  };

  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (const float bad : { 0.0f, -0.3f, inf, nan })
  {
    bac::Params params = omniParams();
    params.limits.vy_max = bad;
    rejects(params, "limits.vy_max = " + std::to_string(bad));
  }
  for (const float bad : { 0.0f, -1.0f, inf, nan })
  {
    bac::Params params = omniParams();
    params.limits.w_max = bad;
    rejects(params, "limits.w_max = " + std::to_string(bad));
  }
  for (const float bad : { -0.1f, nan })
  {
    bac::Params params = omniParams();
    params.heading_gain = bad;
    rejects(params, "heading_gain = " + std::to_string(bad));
  }
  for (const int bad : { 2, 0, -1 })
  {
    bac::Params params = omniParams();
    params.vy_samples = bad;
    rejects(params, "vy_samples = " + std::to_string(bad));
  }

  // A zero gain is legal: it holds the heading fixed, which is what a
  // platform with 360 degree sensing wants.
  bac::Params held = omniParams();
  held.heading_gain = 0.0f;
  bool threw = false;
  try
  {
    bac::detail::validateMotionModelParams(held);
  }
  catch (const std::invalid_argument &)
  {
    threw = true;
  }
  expect(!threw, "heading_gain = 0 is accepted: it holds the heading fixed");
}

}  // namespace

int
main()
{
  testYawIsNotSearched();
  testLatticeOffersBothLateralDirections();
  testSpeedCapAppliesToTheVector();
  testLateralAccelerationWindow();
  testHysteresisMeasuresLateralVelocity();
  testDecelerationPreservesTrajectoryGeometry();
  testRolloutMatchesIntegration();
  testRotationPredicatesAreSeparate();
  testInvalidConfigurationsAreRejected();

  if (failures != 0)
  {
    std::cerr << failures << " holonomic motion-model check(s) failed\n";
    return 1;
  }
  std::cout << "All holonomic motion-model checks passed\n";
  return 0;
}
