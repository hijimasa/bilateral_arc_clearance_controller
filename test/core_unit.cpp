/**
 * @file core_unit.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "test_expect.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace
{

using bac_test::expect;
using bac_test::failures;
using bac_test::near;

void
testStraightArcGeometry()
{
  bac::BacCore core;
  const std::vector<bac::Point2D> points{ { 1.0f, 0.8f }, { 1.2f, -0.7f } };
  const bac::ArcEvaluation eval = core.evaluateArc(points, 0.4f, 0.0f, 4.0f);

  expect(near(eval.clearance_left, 0.8f), "straight arc left clearance");
  expect(near(eval.clearance_right, 0.7f), "straight arc right clearance");
  expect(eval.blocking_s == std::numeric_limits<float>::max(), "body-free arc has no blocking point");
}

void
testBlockingDistance()
{
  bac::BacCore core;
  const bac::ArcEvaluation eval = core.evaluateArc({ { 1.0f, 0.1f } }, 0.4f, 0.0f, 4.0f);

  // blocking_s is the body-origin travel at FIRST CONTACT: the leading edge
  // (front = 0.5) reaches the point at x = 1.0 after 0.5 m of travel.
  expect(near(eval.blocking_s, 0.5f), "first contact is reported as body-origin travel");
}

void
testEmergencyStop()
{
  // Moving into the emergency zone: braking has priority, output is zero.
  {
    bac::BacCore core;
    const bac::Result result =
        core.process({ { 0.59f, 0.0f } }, { { 1.0f, 0.0f } }, bac::Twist2D{ 0.3f, 0.0f });
    expect(result.status == bac::Status::STOP, "point inside emergency zone stops the robot");
    expect(near(result.output.v, 0.0f) && near(result.output.w, 0.0f),
           "emergency output is zero while moving");
  }

  // At standstill the emergency state must not freeze: with rear coverage
  // (limits.v_min < 0) the robot backs away from the frontal point instead.
  {
    bac::BacCore core;
    const bac::Result result =
        core.process({ { 0.59f, 0.0f } }, { { 1.0f, 0.0f } }, bac::Twist2D{});
    expect(result.output.v < 1e-4f, "standstill emergency never advances");
    expect(result.output.v < -1e-3f, "standstill emergency backs away when reverse is allowed");
  }

  // Without rear coverage (v_min = 0) the standstill emergency stays a stop.
  {
    bac::Params params;
    params.limits.v_min = 0.0f;
    bac::BacCore core(params);
    const bac::Result result =
        core.process({ { 0.59f, 0.0f } }, { { 1.0f, 0.0f } }, bac::Twist2D{});
    expect(result.status == bac::Status::STOP, "front-only sensing emergency stays a stop");
    expect(near(result.output.v, 0.0f) && near(result.output.w, 0.0f),
           "front-only emergency output is zero");
  }
}

void
testEmptyPathAndReset()
{
  bac::BacCore core;
  core.forceStop();
  expect(core.status() == bac::Status::STOP, "forceStop changes state");
  core.reset();
  expect(core.status() == bac::Status::CLEAR, "reset clears state");

  const bac::Result result = core.process({}, {}, bac::Twist2D{});
  expect(result.status == bac::Status::CLEAR, "empty path in open space is clear");
  expect(near(result.output.v, 0.0f) && near(result.output.w, 0.0f), "empty path holds position");
}

void
testSweptCornerOnArc()
{
  // Review finding: on a curved arc the rectangle's outer front corner
  // sweeps wider than the width/2 tube. Place a point just inside the
  // corner's swept position at t = 0.5 s on the (v = 0.4, w = 1.0) arc:
  // the body physically clips it, so blocking_s must be finite.
  bac::BacCore core;
  const float v = 0.4f, w = 1.0f, t = 0.5f;
  const float R = v / w, th = w * t;
  const float ox = R * std::sin(th), oy = R * (1.0f - std::cos(th));
  const float c = std::cos(th), s2 = std::sin(th);
  // outer front corner (right side on a left turn), pulled 2 cm inward
  const float lx = 0.5f - 0.02f, ly = -(0.475f - 0.02f);
  const bac::Point2D corner(ox + c * lx - s2 * ly, oy + s2 * lx + c * ly);

  const bac::ArcEvaluation eval = core.evaluateArc({ corner }, v, w, 2.0f);
  expect(eval.blocking_s < 1e6f, "outer corner sweep on an arc is detected as a body hit");

  // The same lateral band on a STRAIGHT run must not over-trigger: a point
  // at |y| = 0.55 beside a straight path is clearance, not a hit.
  const bac::ArcEvaluation straight = core.evaluateArc({ { 1.0f, 0.55f } }, v, 0.0f, 2.0f);
  expect(straight.blocking_s >= 1e6f, "straight motion keeps the exact half-width band");
}

void
testReverseRightTurnCounterexample()
{
  // Fixed counterexample from the re-re-review: reverse + right turn, the
  // point contacts the body at t ~ 0.48 s (body-origin travel ~ 0.19 m) but
  // the un-normalized advance angle rejected it as a next-revolution
  // crossing.
  bac::BacCore core;
  const bac::ArcEvaluation eval =
      core.evaluateArc({ { -0.55f, 0.60f } }, -0.40f, -0.50f, 2.0f);
  expect(eval.blocking_s < 1e6f, "reverse/right-turn contact is detected");
  expect(eval.blocking_s < 0.25f,
         "reverse/right-turn contact distance is not overestimated (" +
             std::to_string(eval.blocking_s) + ")");
}

void
testSweptFootprintProperty()
{
  // Property test against a brute-force ground truth (review re-finding):
  // for random (v, w, point) over forward/backward x left/right turns, the
  // reported blocking_s must (a) never miss a physical contact and (b) never
  // exceed the true first-contact arc length by more than a tolerance.
  // Ground truth: fine time-stepping of the exact rigid motion (rotation
  // about the turn center) with rectangle containment.
  bac::BacCore core;
  const bac::Params params;
  const float half = params.footprint.width / 2.0f;
  unsigned int lcg = 12345u;
  auto frand = [&](float lo, float hi) {
    lcg = lcg * 1664525u + 1013904223u;
    return lo + (hi - lo) * static_cast<float>((lcg >> 8) & 0xFFFF) / 65535.0f;
  };

  int checked = 0, missed = 0, over = 0;
  for (int it = 0; it < 400; ++it)
  {
    float v = frand(-0.3f, 0.4f);
    float w = frand(-1.0f, 1.0f);
    if (std::fabs(v) < 0.05f || std::fabs(w) < 0.05f)
    {
      continue;
    }
    const bac::Point2D p(frand(-1.0f, 2.0f), frand(-1.5f, 1.5f));
    if (p.x >= params.footprint.rear && p.x <= params.footprint.front && std::fabs(p.y) <= half)
    {
      continue;  // starts inside the body: the emergency layer's business
    }

    const float horizon = 2.0f;
    // The contact search is angle-capped (see eval_angle_max) - the ground
    // truth uses the same window.
    const float t_cap  = std::min(horizon, params.eval_angle_max / std::fabs(w));
    const float s_win  = std::fabs(v) * t_cap;
    // ground truth via the rotation about the turn center
    const float R = v / w;
    float s_true  = std::numeric_limits<float>::max();
    for (float t = 0.0f; t <= t_cap; t += 0.002f)
    {
      const float rho = w * t;
      const float ux = p.x, uy = p.y - R;
      const float qx = ux * std::cos(rho) + uy * std::sin(rho);
      const float qy = -ux * std::sin(rho) + uy * std::cos(rho) + R;
      if (qx >= params.footprint.rear && qx <= params.footprint.front && std::fabs(qy) <= half)
      {
        s_true = std::fabs(v) * t;
        break;
      }
    }

    const bac::ArcEvaluation eval = core.evaluateArc({ p }, v, w, horizon);
    ++checked;
    if (s_true < s_win - 0.05f && eval.blocking_s > s_true + 0.03f)
    {
      ++missed;
    }
    if (eval.blocking_s < 1e6f && eval.blocking_s + 0.03f < s_true &&
        s_true > eval.blocking_s + 0.05f && s_true < 1e6f)
    {
      // blocking earlier than truth is conservative; only flag if the truth
      // says NO contact at all yet blocking fired well inside the window
      ;
    }
    if (eval.blocking_s < s_win - 0.05f && s_true == std::numeric_limits<float>::max())
    {
      ++over;  // phantom contact where the body never touches the point
    }
  }
  // A COVERAGE GUARD, and a ONE-SIDED one: only the passing side of it has ever
  // been observed. `checked` is incremented unconditionally after every call
  // that survives the two skip filters above, and both filters read only the
  // LCG and the default footprint - nothing in src/ or bac_core.hpp can move
  // the count. Measured: 291 unmutated (threshold 200), and 291 again under
  // geo_param_eval_angle_half (which halves the very `eval_angle_max` the
  // ground truth below uses for its own window), geo_rho_max_drop_absw and
  // emerg_normalized_lat_removed. Over the 52 mutations this suite is scored
  // against, this assertion fails under none of them.
  //
  // So it defends against an EDIT TO THIS LOOP that quietly stops sampling -
  // shrinking the 400, tightening the filters - and against nothing else. It is
  // not a mutation detector and no destructive band for it has been measured.
  expect(checked > 200, "swept property test exercised enough samples");
  expect(missed == 0, "no physical contact is missed by blocking_s (" +
                          std::to_string(missed) + " missed)");
  expect(over == 0, "no phantom contact is reported (" + std::to_string(over) + " phantom)");

  // Deterministic coverage: EVERY forward/backward x left/right-turn
  // quadrant with a body-surrounding point grid (the random pass alone can
  // skip a quadrant; re-re-review Critical 1 recommendation).
  int g_checked = 0, g_missed = 0, g_over = 0;
  for (float v : { -0.4f, -0.15f, 0.15f, 0.4f })
  {
    for (float w : { -1.0f, -0.4f, 0.4f, 1.0f })
    {
      for (float px = -1.0f; px <= 2.0f; px += 0.15f)
      {
        for (float py = -1.5f; py <= 1.5f; py += 0.15f)
        {
          if (px >= params.footprint.rear - 0.02f && px <= params.footprint.front + 0.02f &&
              std::fabs(py) <= half + 0.02f)
          {
            continue;  // starting on/inside the body boundary
          }
          const float horizon = 2.0f;
          const float t_cap   = std::min(horizon, params.eval_angle_max / std::fabs(w));
          const float s_wing  = std::fabs(v) * t_cap;
          const float R = v / w;
          float s_true  = std::numeric_limits<float>::max();
          for (float t = 0.0f; t <= t_cap; t += 0.002f)
          {
            const float rho = w * t;
            const float ux = px, uy = py - R;
            const float qx = ux * std::cos(rho) + uy * std::sin(rho);
            const float qy = -ux * std::sin(rho) + uy * std::cos(rho) + R;
            if (qx >= params.footprint.rear && qx <= params.footprint.front &&
                std::fabs(qy) <= half)
            {
              s_true = std::fabs(v) * t;
              break;
            }
          }
          const bac::ArcEvaluation ev = core.evaluateArc({ { px, py } }, v, w, horizon);
          ++g_checked;
          if (s_true < s_wing - 0.05f && ev.blocking_s > s_true + 0.03f)
          {
            ++g_missed;
          }
          if (ev.blocking_s < s_wing - 0.05f && s_true == std::numeric_limits<float>::max())
          {
            ++g_over;
          }
        }
      }
    }
  }
  // A COVERAGE GUARD, one-sided in the same way and more strongly: this grid is
  // a fixed nest of four loops with a fixed skip test, so `g_checked` is a
  // compile-time-determined constant that no product code participates in.
  // Measured: 6272 unmutated (threshold 5000) and 6272 under the same three
  // mutations, and it fails under none of the 52. Its destructive band has
  // never been observed, and can only be reached by editing this loop.
  expect(g_checked > 5000, "quadrant grid exercised enough samples");
  expect(g_missed == 0, "quadrant grid: no missed contact (" + std::to_string(g_missed) + ")");
  expect(g_over == 0, "quadrant grid: no phantom contact (" + std::to_string(g_over) + ")");
}

void
testFaceAwayRecovery()
{
  // Standstill facing AWAY from the goal, open space ahead, a passable gap
  // in clutter behind toward the goal. The stop candidate must not win
  // forever (phantom clearance of the un-made straight run + the hysteresis
  // barrier against starting to turn used to freeze this state permanently).
  std::vector<bac::Point2D> world_points;
  for (float s = -0.6f; s <= 0.6f; s += 0.05f)
  {
    world_points.emplace_back(1.5f + s * 0.3f, 1.3f + s);
    world_points.emplace_back(1.5f + s * 0.3f, -1.5f + s);
  }

  float x = 0.0f, y = 0.0f, th = 2.0f;
  bac::BacCore core;
  bac::Twist2D current;
  int  stop_ticks = 0;
  // 35 s budget: the recovery route is mode-dependent (an in-place turn
  // into the gap, or a wide moving turnaround) - the guarded property is
  // "never freezes and reaches the far side", not the specific route.
  for (int i = 0; i < 700; i++)
  {
    const float cs = std::cos(-th), sn = std::sin(-th);
    std::vector<bac::Point2D> points;
    for (const bac::Point2D &p : world_points)
    {
      const float dx = p.x - x, dy = p.y - y;
      points.emplace_back(cs * dx - sn * dy, sn * dx + cs * dy);
    }
    std::vector<bac::Point2D> path;
    for (float gx = 0.0f; gx <= 6.0f; gx += 0.25f)
    {
      const float dx = gx - x, dy = 0.0f - y;
      path.emplace_back(cs * dx - sn * dy, sn * dx + cs * dy);
    }
    const bac::Result result = core.process(points, path, current);
    current = result.output;
    if (std::fabs(current.v) < 1e-3f && std::fabs(current.w) < 1e-3f)
    {
      stop_ticks++;
    }
    th += current.w * 0.05f;
    x += current.v * std::cos(th) * 0.05f;
    y += current.v * std::sin(th) * 0.05f;
    if (x > 5.0f)
    {
      break;  // reached the goal region: standing there is success, not a freeze
    }
  }
  expect(x > 2.0f, "faces-away start turns around and passes the obstacles towards the goal");
  expect(stop_ticks < 200, "faces-away recovery keeps moving (no freeze)");
}

void
testEmergencyEscapePreemptsAlignment()
{
  // The rearward path requests alignment, but this point is already inside
  // the standstill emergency margin. Escape translation must take precedence;
  // otherwise alignment removes translation while escape removes rotation and
  // only the permanent-stop candidate remains.
  bac::Params params;
  params.limits.v_min = -0.1f;
  bac::BacCore core(params);
  const std::vector<bac::Point2D> points{ { 0.58f, 0.53f } };
  const std::vector<bac::Point2D> rear_path{ { -0.1f, 0.0f }, { -4.0f, 0.0f } };

  const bac::Result result = core.process(points, rear_path, {});
  expect(result.output.v < -1e-3f,
         "emergency escape preempts rear-path alignment and reverses away");
  expect(std::fabs(result.output.w) < 1e-3f,
         "emergency escape does not rotate inside the emergency margin");
}

/// Parameters for the plan-orientation tests. `heading_gain` is deliberately
/// small and `acc_w` deliberately huge so that the commanded yaw rate is
/// exactly `heading_gain * pose_reference`: neither the +-w_max saturation nor
/// the one-cycle acceleration bound may mask what the interpolation produced.
bac::Params
planYawParams()
{
  bac::Params params;
  params.motion_model.type = bac::MotionModelType::OMNI;
  params.limits.vy_max     = 0.3f;
  params.limits.acc_w      = 1000.0f;
  params.heading_gain      = 0.1f;
  return params;
}

void
testPlanYawHoldsOrientation()
{
  // A path drawn straight to the robot's LEFT. Tangent following turns onto
  // it; a supplied orientation of "stay as you are" must not.
  std::vector<bac::Point2D> path;
  for (int i = 1; i <= 6; ++i)
  {
    path.emplace_back(0.0f, 0.5f * static_cast<float>(i));
  }

  bac::BacCore tangent(planYawParams());
  const bac::Twist2D turned = tangent.process({}, path, bac::Twist2D{}).output;
  // pose_reference = atan2(0.5, 0) = pi/2, times heading_gain 0.1.
  expect(near(turned.w, 0.15708f, 1e-4f), "tangent following turns onto a sideways path");

  bac::BacCore planned(planYawParams());
  const std::vector<float> hold(path.size(), 0.0f);
  const bac::Twist2D crab =
      planned.process({}, path, bac::Twist2D{}, std::nullopt, hold).output;
  expect(crab.w == 0.0f, "a supplied orientation of zero holds the heading");
  // Not a tuned bound: the whole journey goes into lateral velocity (v is
  // exactly zero - forward motion buys no progress towards a path abeam the
  // body), pushed as hard as ONE cycle allows, acc_v 0.8 * control_period
  // 0.05 = 0.04 m/s. A single process() call cannot show more than that.
  expect(crab.v == 0.0f, "holding the heading, no part of the journey goes forward (v " +
                             std::to_string(crab.v) + ")");
  expect(near(crab.vy, 0.04f, 1e-6f),
         "holding the heading, the body crabs towards the path at the one-cycle "
         "acceleration bound (vy " +
             std::to_string(crab.vy) + ")");
}

void
testPlanYawInterpolatesTheShortWayRound()
{
  // The robot's projection lands 80% along the segment whose two ends sit on
  // OPPOSITE sides of the +-pi branch cut. Interpolating the raw values takes
  // the long way: 3.10 + 0.8 * (-6.20) = -1.86 rad, an entire turn away from
  // the -3.1166 rad the plan actually asks for.
  std::vector<bac::Point2D> path;
  std::vector<float>        yaw;
  for (int i = 0; i < 9; ++i)
  {
    path.emplace_back(-0.9f + 0.5f * static_cast<float>(i), 0.0f);
    yaw.push_back(0.0f);
  }
  yaw[1] = 3.10f;
  yaw[2] = -3.10f;

  bac::BacCore core(planYawParams());
  const bac::Twist2D out = core.process({}, path, bac::Twist2D{}, std::nullopt, yaw).output;
  expect(near(out.w, -0.31166f, 1e-4f), "yaw interpolation crosses the branch cut the short way");
  expect(out.w < -0.25f, "the long way round (-0.186 rad/s) is not what is commanded");
}

void
testPlanYawIgnoredWhenItCannotBeTrusted()
{
  std::vector<bac::Point2D> path;
  for (int i = 1; i <= 6; ++i)
  {
    path.emplace_back(0.0f, 0.5f * static_cast<float>(i));
  }

  bac::BacCore reference(planYawParams());
  const bac::Twist2D tangent = reference.process({}, path, bac::Twist2D{}).output;

  // One entry short: which end is missing decides which point every remaining
  // orientation belongs to, so the sequence is dropped, not realigned.
  bac::BacCore mismatched(planYawParams());
  const std::vector<float> short_yaw(path.size() - 1U, 0.0f);
  const bac::Twist2D from_mismatch =
      mismatched.process({}, path, bac::Twist2D{}, std::nullopt, short_yaw).output;
  expect(from_mismatch.w == tangent.w, "a mismatched orientation count falls back to the tangent");

  // A model that steers with yaw cannot hold a commanded orientation, so it
  // must ignore one rather than fight its own kinematics.
  bac::Params diff = planYawParams();
  diff.motion_model.type = bac::MotionModelType::DIFF_DRIVE;
  diff.limits.vy_max     = 0.0f;
  bac::BacCore steered(diff);
  bac::BacCore steered_plain(diff);
  const std::vector<float> hold(path.size(), 0.0f);
  const bac::Twist2D steered_yaw =
      steered.process({}, path, bac::Twist2D{}, std::nullopt, hold).output;
  const bac::Twist2D steered_ref = steered_plain.process({}, path, bac::Twist2D{}).output;
  expect(steered_yaw.v == steered_ref.v && steered_yaw.w == steered_ref.w,
         "a differential-drive model ignores a commanded orientation");
}

}  // namespace

int
main()
{
  testStraightArcGeometry();
  testBlockingDistance();
  testEmergencyStop();
  testEmptyPathAndReset();
  testSweptCornerOnArc();
  testReverseRightTurnCounterexample();
  testSweptFootprintProperty();
  testFaceAwayRecovery();
  testEmergencyEscapePreemptsAlignment();
  testPlanYawHoldsOrientation();
  testPlanYawInterpolatesTheShortWayRound();
  testPlanYawIgnoredWhenItCannotBeTrusted();

  if (failures != 0)
  {
    std::cerr << failures << " core unit check(s) failed\n";
    return 1;
  }
  std::cout << "All BAC core unit checks passed\n";
  return 0;
}
