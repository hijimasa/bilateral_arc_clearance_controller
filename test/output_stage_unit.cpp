/**
 * @file output_stage_unit.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 * @brief Differential-drive OUTPUT REACHABILITY STAGE semantics.
 *
 * The stage that turns the winning candidate into the emitted command was
 * rewritten with the motion-model boundary. It now decelerates along the
 * SELECTED CURVATURE (instead of holding the yaw rate and lowering v alone),
 * reapplies the one-cycle yaw limit and the contact test up to
 * kReachabilityIterations times, and - when no translating command survives
 * all three constraints - emits zero speed plus the braking yaw rate ONLY
 * where an in-place rotation is admissible.
 *
 * Every differential-drive user receives that change, so the witnesses below
 * pin it: curvature-preserving deceleration, a loop that really iterates, and
 * the rotation guard on the fallback in both of its directions. They are
 * single-tick BacCore::process() calls with the shipped defaults, each
 * reduced from a randomized state-synchronised sweep against the pre-refactor
 * implementation to the smallest world that still exercises the property.
 * Wherever possible the assertion is on a property computed at run time (the
 * unconstrained winner, and the public limitReachableCommand) rather than on
 * a golden output.
 */

#include "bilateral_arc_clearance_controller/bac_core.hpp"

#include <cmath>
#include <iostream>
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

std::string
num(float value)
{
  return std::to_string(value);
}

/// Straight local path leaving the body at a fixed bearing.
std::vector<bac::Point2D>
straightPath(float bearing)
{
  std::vector<bac::Point2D> path;
  for (float s = 0.1f; s <= 5.0f; s += 0.2f)
  {
    path.emplace_back(s * std::cos(bearing), s * std::sin(bearing));
  }
  return path;
}

/**
 * @brief One deceleration witness: a single obstacle point ahead, a path
 * leaving to the side, and a current yaw rate low enough that the winning
 * candidate's yaw rate lies outside the one-cycle window
 * (acc_w * control_period = 0.125 rad/s). The clamped arc then cannot stop
 * before contact, so the output stage must slow down - and the arc it slows
 * down ON is the geometric decision that was contact-checked, so the yaw rate
 * has to come down with the speed.
 */
void
checkDecelerationWitness(const std::string &label, const bac::Twist2D &current,
                         const bac::Point2D &obstacle, float path_bearing)
{
  const std::vector<bac::Point2D> points{ obstacle };
  const std::vector<bac::Point2D> path = straightPath(path_bearing);

  // Control run: acc_w = 0 disables the output reachability limit
  // (limitReachableCommand is then the identity). Nothing else in process()
  // reads acc_w - candidate sampling and scoring do not - so this run reports
  // exactly the unconstrained winner the main run below starts from.
  bac::Params open_params;
  open_params.limits.acc_w = 0.0f;
  bac::BacCore open_core(open_params);
  const bac::Twist2D best = open_core.process(points, path, current).output;

  bac::BacCore core;
  const bac::Twist2D clamped = core.limitReachableCommand(current, best);
  expect(best.v > 1e-3f,
         label + " witness: the unconstrained winner translates (v = " + num(best.v) + ")");
  expect(std::fabs(clamped.w - best.w) > 1e-3f,
         label + " witness: the one-cycle yaw window clamps the winner (w " + num(best.w) +
             " -> " + num(clamped.w) + ")");

  const bac::Result result = core.process(points, path, current);

  expect(result.output.v > 1e-3f,
         label + ": the reachability stage keeps translating instead of braking to zero (v = " +
             num(result.output.v) + ")");
  expect(result.output.v < best.v - 1e-3f,
         label + ": the reachability stage lowers the speed of the clamped arc (v " +
             num(best.v) + " -> " + num(result.output.v) + ")");
  if (result.output.v > 1e-3f)
  {
    // The property: the emitted arc is the CLAMPED arc at a lower speed.
    const float selected_curvature = clamped.w / best.v;
    const float output_curvature   = result.output.w / result.output.v;
    expect(near(output_curvature, selected_curvature, 5e-3f),
           label + ": deceleration preserves the curvature of the contact-checked arc (" +
               num(output_curvature) + " vs " + num(selected_curvature) + " 1/m)");
    expect(std::fabs(result.output.w) < std::fabs(clamped.w) - 1e-3f,
           label +
               ": the yaw rate follows the speed down instead of being held at the window edge "
               "(|w| = " +
               num(std::fabs(result.output.w)) + " vs " + num(std::fabs(clamped.w)) + ")");
  }
}

void
testReachabilityDecelerationPreservesCurvature()
{
  // Two witnesses, because holding the yaw rate (the pre-refactor rule) fails
  // them in two different ways and each assertion above must be able to fail.
  //
  // (a) winner (0.120000, 0.479167), clamped (0.120000, 0.300000), emitted
  //     (0.093790, 0.234475) on the same 2.500000 1/m curvature. Holding the
  //     yaw rate ratchets this arc down to an in-place rotation (0, 0.300000);
  //     a single reachability iteration instead of eight leaves the loop
  //     non-admissible and brakes to (0, 0.050000).
  checkDecelerationWitness("decel-a", bac::Twist2D(0.24f, 0.175f), { 0.85f, -0.05f }, 1.2f);

  // (b) winner (0.120000, 0.479167), clamped (0.120000, 0.301853), emitted
  //     (0.086448, 0.217490) on 2.515 1/m. Here holding the yaw rate keeps the
  //     SAME speed 0.086448 at w = 0.301853, i.e. a 3.492 1/m arc that was
  //     never contact-checked: only the curvature assertion separates it.
  checkDecelerationWitness("decel-b", bac::Twist2D(0.2522f, 0.1769f), { 0.848f, -0.05f }, 1.1923f);
}

void
testReachabilityIterationDepth()
{
  // A straight corridor of half-width 0.80 m entered at a high yaw rate. The
  // yaw window clamps the winner, the clamped arc reaches a wall, and the
  // curvature-preserving deceleration has to be applied SEVERAL times before a
  // command is simultaneously admissible and reachable: measured here, the
  // loop needs its 5th iteration. With four iterations or fewer this tick
  // brakes to (0, -0.875000) instead of emitting (0.303044, -0.875000).
  //
  // This pins the loop as an iteration rather than a single correction. It
  // deliberately does not pin the exact bound of eight: over a 200000-tick
  // sweep only 32 ticks need a 5th iteration and a single tick distinguishes
  // eight from seven, so an assertion on the exact bound would be a tripwire
  // rather than a separation.
  std::vector<bac::Point2D> corridor;
  for (float x = -0.5f; x <= 3.0f + 1e-4f; x += 0.25f)
  {
    corridor.emplace_back(x, 0.80f);
    corridor.emplace_back(x, -0.80f);
  }
  const bac::Twist2D current(0.11f, -1.0f);

  bac::BacCore core;
  const bac::Result result = core.process(corridor, straightPath(0.0f), current);
  expect(result.output.v > 1e-3f,
         "repeated reachability correction finds a translating command in the corridor (v = " +
             num(result.output.v) + ")");
  expect(std::fabs(result.output.w) < std::fabs(current.w),
         "the corridor command stays inside the one-cycle yaw window (w " + num(current.w) +
             " -> " + num(result.output.w) + ")");
}

void
testUnreachableOutputStopsWithoutAdmissibleRotation()
{
  // Same world and same current twist twice; only the distance of the near
  // point changes. In both, no translating command is simultaneously
  // curvature-preserving, contact-admissible and reachable, so the output
  // stage brakes translation and offers the reachable braking yaw rate.
  // Whether that residual rotation may be emitted is decided by in-place
  // rotation admissibility, which is the conservative circumscribed disk
  // (sqrt(front^2 + (width/2)^2) = 0.6897 m, plus 2 cm).
  const bac::Point2D ahead(0.827f, 0.172f);
  const std::vector<bac::Point2D> path = straightPath(1.2f);
  const bac::Twist2D current(0.14f, -0.37f);

  bac::BacCore reference;
  const bac::Twist2D braking = reference.limitReachableCommand(current, bac::Twist2D(0.0f, 0.0f));
  expect(std::fabs(braking.w) > 1e-3f,
         "witness: braking from this yaw rate leaves a residual yaw rate (" + num(braking.w) + ")");

  // (a) A point at 0.683 m sits INSIDE the rotation disk: rotating is not
  // admissible, so the tick must emit exactly (0, 0) and report STOP.
  {
    const std::vector<bac::Point2D> points{ { 0.126f, 0.671f }, ahead };
    bac::BacCore core;
    const bac::Result result = core.process(points, path, current);
    expect(result.min_proximity_norm > 1.0f,
           "witness: the near point is outside the emergency zone, so the stop comes from the "
           "output stage (proximity norm " +
               num(result.min_proximity_norm) + ")");
    expect(near(result.output.v, 0.0f) && near(result.output.w, 0.0f),
           "an unreachable output with no admissible rotation emits exactly zero (" +
               num(result.output.v) + ", " + num(result.output.w) + ")");
    expect(result.status == bac::Status::STOP,
           "that tick reports STOP rather than AVOIDING");
  }

  // (b) The same point moved to 0.810 m clears the disk: the residual
  // rotation is admissible and IS emitted. Without this half, "always zero"
  // would satisfy (a) as well.
  {
    const std::vector<bac::Point2D> points{ { 0.126f, 0.80f }, ahead };
    bac::BacCore core;
    const bac::Result result = core.process(points, path, current);
    expect(near(result.output.v, 0.0f),
           "the unreachable output still brakes translation (v = " + num(result.output.v) + ")");
    expect(near(result.output.w, braking.w, 1e-4f),
           "an admissible rotation keeps the reachable braking yaw rate (" + num(result.output.w) +
               " vs " + num(braking.w) + ")");
    expect(result.status != bac::Status::STOP,
           "a tick that still rotates does not report STOP");
  }
}

}  // namespace

int
main()
{
  testReachabilityDecelerationPreservesCurvature();
  testReachabilityIterationDepth();
  testUnreachableOutputStopsWithoutAdmissibleRotation();

  if (failures != 0)
  {
    std::cerr << failures << " output stage check(s) failed\n";
    return 1;
  }
  std::cout << "All BAC output stage checks passed\n";
  return 0;
}
