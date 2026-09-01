/**
 * @file ackermann_motion_model_unit.cpp
 * @brief Unit checks for Ackermann body-curvature constraints
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "ackermann_motion_model.hpp"
#include "motion_model.hpp"

#include "bilateral_arc_clearance_controller/bac_core.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

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

bac::Params
ackermannParams()
{
  bac::Params params;
  params.motion_model.type = bac::MotionModelType::ACKERMANN;
  params.turn_radius_min = 1.2f;
  params.limits.v_min = 0.0f;
  params.v_samples = 3;
  params.w_samples = 5;
  return params;
}

/// A vehicle whose SPEED limit, not its turning circle, is what makes the
/// yaw-rate limit bite: v_max (1.2) > w_max (0.4) * turn_radius_min (0.5), so
/// |v| / turn_radius_min reaches 2.4 rad/s while limits.w_max allows 0.4. In
/// the fixture above - and in the shipped yaml, and in every scenario fixture
/// before R14 - v_max <= w_max * turn_radius_min, so min(w_max, |v| / R)
/// always picks the radius term and the w_max term is structurally inactive
/// (R14 M2: `w_max` appeared zero times in this file).
bac::Params
yawRateBoundParams()
{
  bac::Params params;
  params.motion_model.type = bac::MotionModelType::ACKERMANN;
  params.turn_radius_min = 0.5f;
  params.limits.v_min = 0.0f;
  params.limits.v_max = 1.2f;
  params.limits.w_max = 0.4f;
  params.v_samples = 3;
  params.w_samples = 5;
  return params;
}

/// Every path through the Ackermann curvature bound, exercised where the
/// `limits.w_max` term is the one that binds.
void
testYawRateLimitBinds()
{
  const bac::Params params = yawRateBoundParams();
  const bac::detail::AckermannMotionModel model(params);
  const float w_max = params.limits.w_max;
  const float radius_bound_at_v_max = params.limits.v_max / params.turn_radius_min;
  expect(radius_bound_at_v_max > w_max + 1e-3f,
         "the fixture is one where limits.w_max, not the turning circle, binds "
         "(circle allows " + std::to_string(radius_bound_at_v_max) + " rad/s, "
         "limits.w_max " + std::to_string(w_max) + ")");

  // 1. Candidate generation. Without the w_max term the fastest rows would be
  //    sampled out to |w| = 2.4 rad/s.
  const bac::detail::CandidateBatch batch =
      model.sampleCandidates(bac::Twist2D(1.0f, 0.0f), params.limits.v_max, 0.0f);
  bool sampled_a_binding_speed = false;
  for (const bac::Twist2D &command : batch.commands)
  {
    expect(std::fabs(command.w) <= w_max + 1e-5f,
           "no sampled candidate exceeds limits.w_max (v " +
               std::to_string(command.v) + ", w " + std::to_string(command.w) + ")");
    if (std::fabs(command.v) / params.turn_radius_min > w_max + 1e-5f)
    {
      sampled_a_binding_speed = true;
    }
  }
  expect(sampled_a_binding_speed,
         "the lattice actually reaches speeds where limits.w_max is the binding "
         "term, so the bound above is not vacuous");

  // 2. Validity. The turning circle alone would accept this arc.
  expect(!model.isCommandKinematicallyValid({ 1.2f, 0.6f }),
         "a yaw rate above limits.w_max is rejected even though the turning "
         "circle would allow it");
  expect(model.isCommandKinematicallyValid({ 1.2f, w_max }),
         "a yaw rate exactly at limits.w_max is accepted");

  // 3. Reachability limiting. The yaw-acceleration window here is
  //    [0.275, 0.525] rad/s (acc_w * control_period around the current w), so
  //    w_max = 0.4 sits strictly inside it and the clamp that produces 0.4 can
  //    only be the curvature bound.
  const bac::Twist2D limited =
      model.limitReachableCommand({ 1.2f, w_max }, { 1.2f, radius_bound_at_v_max });
  expect(near(limited.w, w_max),
         "limitReachableCommand clamps the yaw rate to limits.w_max (" +
             std::to_string(limited.w) + ")");

  // 4. Refinement and probes are built from the same bound.
  for (const bac::Twist2D &command : model.refinementCandidates({ 1.2f, w_max }))
  {
    expect(std::fabs(command.w) <= w_max + 1e-5f,
           "refinement never leaves limits.w_max (w " + std::to_string(command.w) + ")");
  }
  for (const bac::Twist2D &probe : model.clearanceProbeCommands(1.2f))
  {
    expect(std::fabs(probe.w) <= w_max + 1e-5f,
           "probe arcs never exceed limits.w_max (w " + std::to_string(probe.w) + ")");
  }
}

void
testCurvatureCandidateLattice()
{
  const bac::Params params = ackermannParams();
  const bac::detail::AckermannMotionModel model(params);
  const bac::detail::CandidateBatch batch =
      model.sampleCandidates(bac::Twist2D(0.2f, 0.0f), 0.4f, 0.0f);

  expect(batch.commands.size() == 1U + 2U * 6U,
         "stop plus two speed rows by five curvature samples and straight");
  expect(near(batch.commands.front().v, 0.0f) && near(batch.commands.front().w, 0.0f),
         "Ackermann lattice starts with one true stop");

  bool found_in_place_rotation = false;
  for (const bac::Twist2D &command : batch.commands)
  {
    if (std::fabs(command.v) <= 1e-4f && std::fabs(command.w) > 1e-4f)
    {
      found_in_place_rotation = true;
    }
    if (std::fabs(command.v) > 1e-4f)
    {
      const float radius = std::fabs(command.w) > 1e-5f
                               ? std::fabs(command.v / command.w)
                               : 1e9f;
      expect(radius + 1e-5f >= params.turn_radius_min,
             "every moving candidate respects minimum turning radius");
      expect(model.isCommandKinematicallyValid(command),
             "every sampled moving candidate is kinematically valid");
    }
  }
  expect(!found_in_place_rotation, "Ackermann lattice never offers in-place rotation");
  expect(!model.supportsInPlaceRotation(), "Ackermann reports no in-place rotation support");
  expect(!model.isCommandKinematicallyValid({ 0.0f, 0.2f }),
         "zero-speed yaw command is rejected");

  // The loop above only feeds the validator commands the sampler built from the
  // same bound, which it accepts by construction. Assert the negative side too.
  expect(!model.isCommandKinematicallyValid({ 0.3f, 0.3f / 0.5f }),
         "an arc tighter than turn_radius_min is rejected");
  expect(model.isCommandKinematicallyValid({ -0.2f, -0.2f / params.turn_radius_min }),
         "reverse arcs are validated on |v|, not on the sign of v");
  expect(!model.isCommandKinematicallyValid({ -0.2f, -0.2f / 0.5f }),
         "a reverse arc tighter than turn_radius_min is rejected");

  // The caller's speed cap is the DWA speed governor; exceeding it would offer
  // arcs the governor already ruled out.
  bool exceeded_cap = false;
  for (const bac::Twist2D &command : batch.commands)
  {
    if (command.v > 0.4f + 1e-5f)
    {
      exceeded_cap = true;
    }
  }
  expect(!exceeded_cap, "the lattice never exceeds the caller's linear speed cap");

  // The cap is the proximity speed governor's output, which is BELOW
  // limits.v_max exactly when an obstacle is near; a lattice that used v_max
  // here would round obstacles at 0.18 m instead of 0.36 m. This is the
  // coverage the closed-loop min_clearance bound in testObstacleDetour used to
  // provide before R14 M3 measured it as a tripwire - it is deterministic and
  // needs no threshold, and it fails 12 times on that mutation.
  const bac::detail::CandidateBatch capped =
      model.sampleCandidates(bac::Twist2D(0.2f, 0.0f), 0.1f, 0.0f);
  for (const bac::Twist2D &command : capped.commands)
  {
    expect(command.v <= 0.1f + 1e-5f, "a lowered speed cap lowers the lattice");
  }
  // ... including when the acceleration window alone would allow much more:
  // current 0.4 m/s + acc_v * window_time (0.2) is 0.6, and limits.v_max is
  // 0.4, but the caller only allows 0.15.
  const bac::detail::CandidateBatch hard_capped =
      model.sampleCandidates(bac::Twist2D(0.4f, 0.0f), 0.15f, 0.0f);
  for (const bac::Twist2D &command : hard_capped.commands)
  {
    expect(command.v <= 0.15f + 1e-5f,
           "the caller's cap, not limits.v_max, bounds the lattice (v " +
               std::to_string(command.v) + ")");
  }

  // An even w_samples puts no sample at kappa = 0, so the explicit straight
  // candidate is the only source of a true straight arc.
  bac::Params even_params = ackermannParams();
  even_params.w_samples = 4;
  const bac::detail::AckermannMotionModel even_model(even_params);
  const bac::detail::CandidateBatch even_batch =
      even_model.sampleCandidates(bac::Twist2D(0.2f, 0.0f), 0.4f, 0.0f);
  int straight_rows = 0;
  for (const bac::Twist2D &command : even_batch.commands)
  {
    if (command.v > 1e-4f && std::fabs(command.w) <= 1e-6f)
    {
      ++straight_rows;
    }
  }
  expect(straight_rows == 2,
         "each moving speed row offers a true straight arc even without a zero sample");
}

void
testProjectionAndRefinement()
{
  const bac::Params params = ackermannParams();
  const bac::detail::AckermannMotionModel model(params);
  const float v = 0.35f;
  const float command_curvature = 0.4f;
  const float w = v * command_curvature;

  const bac::detail::ProjectedPose2D pose =
      model.projectConstantCommand({ v, w }, 2.0f);
  const float radius = v / w;
  expect(near(pose.theta, 2.0f * w), "Ackermann projection integrates yaw rate");
  expect(near(pose.x, radius * std::sin(pose.theta)) &&
             near(pose.y, radius * (1.0f - std::cos(pose.theta))),
         "Ackermann projection follows its constant-curvature body arc");

  const std::vector<bac::Twist2D> refined =
      model.refinementCandidates({ v, 0.0f });
  expect(refined.size() == 6U, "Ackermann refinement adds three curvature offsets per side");
  const float bound = 1.0f / params.turn_radius_min;
  const float coarse_step = 2.0f * bound / static_cast<float>(params.w_samples - 1);
  bool below = false;
  bool above = false;
  for (const bac::Twist2D &command : refined)
  {
    expect(model.isCommandKinematicallyValid(command),
           "refined Ackermann candidate remains kinematically valid");
    const float offset = command.w / command.v;
    expect(std::fabs(offset) < coarse_step,
           "a refinement offset stays finer than the coarse curvature pitch");
    below = below || offset < 0.0f;
    above = above || offset > 0.0f;
  }
  expect(below && above, "refinement brackets the coarse winner on both sides");

  // Centred on the bound, half of the offsets fall outside it and must be
  // dropped rather than emitted as an infeasible arc.
  const std::vector<bac::Twist2D> at_bound =
      model.refinementCandidates({ v, v * bound });
  expect(!at_bound.empty(), "refinement still offers candidates at the curvature bound");
  for (const bac::Twist2D &command : at_bound)
  {
    expect(std::fabs(command.w / command.v) <= bound + 1e-5f,
           "refinement at the bound never leaves the turning circle");
    expect(model.isCommandKinematicallyValid(command),
           "refinement at the bound stays kinematically valid");
  }
}

void
testBodyReachabilityAndFactory()
{
  bac::Params params = ackermannParams();
  params.limits.acc_w = 0.4f;
  const bac::detail::AckermannMotionModel model(params);
  const float v = 0.3f;
  const bac::Twist2D desired(v, v / params.turn_radius_min);
  const bac::Twist2D limited =
      model.limitReachableCommand({ v, 0.0f }, desired);

  expect(near(limited.w, params.limits.acc_w * params.control_period),
         "body yaw-rate target is limited to one control-cycle reachability");
  // `desired` and `limited` share a speed, so curvature and yaw rate differ only
  // by the constant 1/v and any ordering between them holds under either
  // definition. Compare across DIFFERENT speeds, where the two disagree.
  expect(near(model.commandChange({ 0.4f, 0.4f / params.turn_radius_min },
                                  { 0.2f, 0.2f / params.turn_radius_min }),
              0.0f),
         "the same arc at a different speed is not a steering change");
  expect(model.commandChange({ 0.4f, 0.1f }, { 0.2f, 0.1f }) > 1e-3f,
         "the same yaw rate at a different speed IS a steering change");

  const bac::Twist2D slowed = model.withLinearSpeed(limited, v / 2.0f);
  expect(near(slowed.w / slowed.v, limited.w / limited.v) &&
             near(slowed.w, limited.w / 2.0f),
         "post-selection speed limit preserves Ackermann curvature");
  const bac::Twist2D stopped = model.withLinearSpeed(limited, 0.0f);
  expect(near(stopped.v, 0.0f) && near(stopped.w, 0.0f),
         "Ackermann speed limit to zero cannot leave an in-place yaw command");

  const bac::Twist2D radius_limited =
      model.limitReachableCommand({ v, desired.w }, { 0.05f, 1.0f });
  expect(near(radius_limited.w, 0.05f / params.turn_radius_min),
         "minimum turning radius is re-applied after body yaw-rate limiting");

  const std::unique_ptr<bac::detail::MotionModel> from_factory =
      bac::detail::makeMotionModel(params);
  expect(!from_factory->supportsInPlaceRotation(),
         "factory selects the configured Ackermann policy");
}

/// `clearanceProbeCommands` feeds the tightness probe in BacCore; a probe arc
/// the vehicle cannot drive would misreport how tight the surroundings are.
void
testClearanceProbes()
{
  const bac::Params params = ackermannParams();
  const bac::detail::AckermannMotionModel model(params);

  // At 0.3 m/s the 1.2 m turning circle allows only 0.25 rad/s.
  const std::vector<bac::Twist2D> slow = model.clearanceProbeCommands(0.3f);
  expect(slow.size() == 3U, "three probe arcs are offered");
  expect(near(slow[0].w, -0.25f) && near(slow[1].w, 0.0f) && near(slow[2].w, 0.25f),
         "probe yaw rates are clipped to the turning-radius bound at low speed");
  for (const bac::Twist2D &probe : slow)
  {
    expect(model.isCommandKinematicallyValid(probe), "probe arcs are drivable");
  }

  // At 1.0 m/s the circle would allow 0.833 rad/s, so the probe pitch caps it.
  const std::vector<bac::Twist2D> fast = model.clearanceProbeCommands(1.0f);
  expect(near(fast[2].w, 0.4f) && near(fast[0].w, -0.4f),
         "probe yaw rate saturates at the probe pitch when the circle is not binding");
}

/// The Ackermann deadband deliberately differs from the differential-drive one.
void
testDeadband()
{
  const bac::Params params = ackermannParams();
  const bac::detail::AckermannMotionModel model(params);

  const bac::Twist2D small_yaw = model.applyCommandDeadband({ 0.3f, 0.005f });
  expect(near(small_yaw.w, 0.005f),
         "a yaw rate below angvel_min survives, because at speed it is a real arc");

  const bac::Twist2D creeping = model.applyCommandDeadband({ 0.001f, 0.5f });
  expect(near(creeping.v, 0.0f) && near(creeping.w, 0.0f),
         "below velocity_min the whole command is zeroed, never leaving a yaw rate");

  const bac::Twist2D straight = model.applyCommandDeadband({ 0.3f, 1e-8f });
  expect(near(straight.w, 0.0f), "a negligible curvature is snapped to straight");
}

/// A motion-model switch must take effect even when no other parameter moves.
void
testRuntimeModelSwitch()
{
  bac::Params diff = ackermannParams();
  diff.motion_model.type = bac::MotionModelType::DIFF_DRIVE;
  bac::BacCore core(diff);

  const bac::Twist2D unconstrained =
      core.limitReachableCommand({ 0.4f, 1.0f }, { 0.4f, 1.0f });
  expect(std::fabs(unconstrained.w) > 0.4f / diff.turn_radius_min + 1e-3f,
         "differential drive is not bound by the Ackermann turning circle");

  bac::Params ackermann = diff;  // turn_radius_min deliberately unchanged
  ackermann.motion_model.type = bac::MotionModelType::ACKERMANN;
  core.setParams(ackermann);
  expect(near(core.limitReachableCommand({ 0.4f, 1.0f }, { 0.4f, 1.0f }).w,
              0.4f / ackermann.turn_radius_min, 1e-4f),
         "setParams switches the model even when turn_radius_min is unchanged");

  core.setParams(diff);
  expect(std::fabs(core.limitReachableCommand({ 0.4f, 1.0f }, { 0.4f, 1.0f }).w) >
             0.4f / diff.turn_radius_min + 1e-3f,
         "switching back to differential drive drops the turning circle again");
}

void
testInvalidConfiguration()
{
  bac::Params params = ackermannParams();
  params.turn_radius_min = 0.0f;
  bool rejected = false;
  try
  {
    (void)bac::detail::makeMotionModel(params);
  }
  catch (const std::invalid_argument &)
  {
    rejected = true;
  }
  expect(rejected, "factory rejects a non-positive Ackermann turning radius");

  // The model is bound at configuration time, so an unusable kinematic
  // configuration is reported there rather than from inside a control tick.
  bac::BacCore core;
  bool rejected_by_core = false;
  try
  {
    core.setParams(params);
  }
  catch (const std::invalid_argument &)
  {
    rejected_by_core = true;
  }
  expect(rejected_by_core, "setParams rejects the configuration, not process()");

  // Rejection must not half-apply. The model holds a reference to the core's
  // params, so committing them before a failed rebuild would leave the old
  // model reading a negative turning radius, whose clamp collapses a straight
  // command into a full-lock turn.
  expect(core.params().turn_radius_min == bac::Params{}.turn_radius_min,
         "a rejected setParams leaves the previous parameters in place");
  const bac::Twist2D straight = core.limitReachableCommand({ 0.3f, 0.0f }, { 0.3f, 0.0f });
  expect(near(straight.v, 0.3f) && near(straight.w, 0.0f),
         "a rejected setParams leaves the core driving straight, not full lock");

  // The `limits.w_max` half of the same guard (R14 M2: S2 and S3). Both
  // layers are checked directly, because routing through makeMotionModel would
  // let either one alone satisfy the test: the factory validates first, and the
  // constructor re-checks. `bad` outlives the model, which holds params by
  // reference.
  const float infinity = std::numeric_limits<float>::infinity();
  const float not_a_number = std::numeric_limits<float>::quiet_NaN();
  const float bad_w_max_values[] = { 0.0f, -0.5f, infinity, not_a_number };
  for (float bad_w_max : bad_w_max_values)
  {
    bac::Params bad = ackermannParams();
    bad.limits.w_max = bad_w_max;
    const std::string shown = std::to_string(bad_w_max);

    bool validator_rejected = false;
    try
    {
      bac::detail::validateMotionModelParams(bad);
    }
    catch (const std::invalid_argument &)
    {
      validator_rejected = true;
    }
    expect(validator_rejected,
           "validateMotionModelParams rejects limits.w_max = " + shown);

    bool constructor_rejected = false;
    try
    {
      const bac::detail::AckermannMotionModel model(bad);
      (void)model;
    }
    catch (const std::invalid_argument &)
    {
      constructor_rejected = true;
    }
    expect(constructor_rejected,
           "direct construction rejects limits.w_max = " + shown);

    bool core_rejected = false;
    try
    {
      bac::BacCore rejecting;
      rejecting.setParams(bad);
    }
    catch (const std::invalid_argument &)
    {
      core_rejected = true;
    }
    expect(core_rejected, "setParams rejects limits.w_max = " + shown);
  }

  bac::Params zero_radius = ackermannParams();
  zero_radius.turn_radius_min = 0.0f;
  bac::BacCore ackermann_core(ackermannParams());
  bool rejected_zero = false;
  try
  {
    ackermann_core.setParams(zero_radius);
  }
  catch (const std::invalid_argument &)
  {
    rejected_zero = true;
  }
  expect(rejected_zero, "a zero turning radius is rejected rather than made infinite");
  const bac::Twist2D still_bounded =
      ackermann_core.limitReachableCommand({ 0.3f, 1.0f }, { 0.3f, 1.0f });
  expect(near(still_bounded.w, 0.3f / ackermannParams().turn_radius_min, 1e-4f),
         "the turning circle survives a rejected reconfiguration");
}

/// BacCore owns a motion model bound to its own parameters, so a copy must
/// keep behaving like the Ackermann vehicle it was configured as.
void
testCoreValueSemantics()
{
  const bac::Params params = ackermannParams();
  bac::BacCore configured(params);

  bac::BacCore copy = configured;
  expect(copy.params().motion_model.type == bac::MotionModelType::ACKERMANN,
         "a copied core keeps the Ackermann policy");

  // The current yaw rate matches the request, so the yaw-acceleration window
  // does not bind and the turning-radius clamp is what is observed.
  const bac::Twist2D over_curved =
      copy.limitReachableCommand({ 0.4f, 1.0f }, { 0.4f, 1.0f });
  expect(near(over_curved.w, 0.4f / params.turn_radius_min, 1e-4f),
         "a copied core applies the Ackermann turning radius through its own model");

  bac::BacCore assigned;
  assigned = configured;
  const bac::Twist2D assigned_limited =
      assigned.limitReachableCommand({ 0.4f, 1.0f }, { 0.4f, 1.0f });
  expect(near(assigned_limited.w, 0.4f / params.turn_radius_min, 1e-4f),
         "an assigned core applies the Ackermann turning radius through its own model");

  // The source must be unaffected by either operation.
  const bac::Twist2D source_limited =
      configured.limitReachableCommand({ 0.4f, 1.0f }, { 0.4f, 1.0f });
  expect(near(source_limited.w, 0.4f / params.turn_radius_min, 1e-4f),
         "the copied-from core still owns a usable model");
}

}  // namespace

int
main()
{
  testYawRateLimitBinds();
  testCurvatureCandidateLattice();
  testProjectionAndRefinement();
  testBodyReachabilityAndFactory();
  testClearanceProbes();
  testDeadband();
  testRuntimeModelSwitch();
  testInvalidConfiguration();
  testCoreValueSemantics();

  if (failures != 0)
  {
    std::cerr << failures << " Ackermann motion-model check(s) failed\n";
    return 1;
  }
  std::cout << "All Ackermann motion-model checks passed\n";
  return 0;
}
