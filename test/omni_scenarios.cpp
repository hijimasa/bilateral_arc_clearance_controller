/**
 * @file omni_scenarios.cpp
 * @brief Closed-loop regressions for the holonomic BAC policy
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * The holonomic contract is that AVOIDANCE happens in lateral velocity while
 * the yaw rate regulates the body onto the local path tangent. The scenarios
 * that carry the weight here are therefore the ones that can tell the two
 * apart: they run the same world with a differential-drive reference that
 * shares the tuning, and assert on the DIFFERENCE. An assertion that a
 * holonomic robot reaches its goal would pass for either model.
 *
 * The plant is acceleration-limited on all three axes and integrated in the
 * world frame, so the commands are checked against a vehicle that cannot
 * change any velocity component instantly.
 */

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "shipped_config.hpp"
#include "sim_runner.hpp"
#include "sim_world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
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

float
wrapAngle(float angle)
{
  while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
  while (angle <= -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
  return angle;
}

/// Optional corridor-centering window: lateral error is aggregated only while
/// the vehicle is between x_from and x_to, i.e. inside the corridor proper.
struct LateralWindow
{
  bool enabled = false;
  float center_y = 0.0f;
  float x_from = 0.0f;
  float x_to = 0.0f;
};

struct OmniRun
{
  bac_sim::Pose final_pose;
  bool collided = false;
  bool reached_goal = false;
  bool exceeded_speed_cap = false;
  bool exceeded_lateral_cap = false;
  bool exceeded_yaw_cap = false;
  float min_clearance = 1e9f;
  float min_goal_distance = 1e9f;
  float travelled_distance = 0.0f;
  float max_speed = 0.0f;
  float max_abs_vy = 0.0f;
  float max_abs_w = 0.0f;
  float final_heading_error = 0.0f;  // vs the straight-line goal bearing
  int   stop_ticks = 0;
  int   total_ticks = 0;
  int   translating_ticks_first_second = 0;  // motion during the first 20 ticks
  float mean_abs_lateral = 0.0f;
  float max_abs_lateral = 0.0f;
  int   lateral_samples = 0;
};

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
  params.avoid_margin.side = 0.9f;  // see config/bac_controller_omni.yaml
  params.weights.hysteresis = 0.4f;
  return params;
}

/// The same vehicle under differential drive, used to show that the holonomic
/// assertions below test a real difference and not a property both models
/// happen to satisfy. Two settings differ, and both differ for a measured
/// reason rather than to make a test pass:
///
///  - weights.hysteresis: the term measures yaw-rate change here and lateral-
///    velocity change under the holonomic model, so the value does not carry
///    over (the same is true between differential drive and Ackermann).
///  - avoid_margin.side: with 0.9 the differential drive stops 3.9 m short of
///    the goal in the detour world below (measured at 0.5/0.6/0.7/0.9: final x
///    5.73, 5.72, 2.12, 2.11), because demanding that much side clearance
///    around an isolated obstacle keeps it from committing to either side. The
///    holonomic run is unaffected by the value (final x 5.72 at all four).
bac::Params
diffDriveReferenceParams()
{
  bac::Params params = omniParams();
  params.motion_model.type = bac::MotionModelType::DIFF_DRIVE;
  params.limits.vy_max = 0.0f;
  params.avoid_margin.side = 0.6f;
  params.weights.hysteresis = 0.6f;
  return params;
}

OmniRun
runOmni(bac::BacCore &core, const bac_sim::World &world, const bac_sim::Pose &start,
        const bac_sim::PathSource &path_source, float goal_x, float goal_y,
        float simulation_time, const LateralWindow &window = LateralWindow{},
        std::optional<float> goal_heading_world = std::nullopt)
{
  constexpr float dt = 0.05f;
  OmniRun run;
  bac_sim::Pose pose = start;
  float vx = 0.0f, vy = 0.0f, w = 0.0f;
  const bac::Params &params = core.params();
  const float acc_v = params.limits.acc_v;
  const float acc_w = params.limits.acc_w;
  double lateral_sum = 0.0;

  const int steps = static_cast<int>(simulation_time / dt);
  for (int step = 0; step < steps; ++step)
  {
    const float t = static_cast<float>(step) * dt;
    const std::vector<bac::Point2D> points =
        bac_sim::simulateLidar(world, pose, 720, params.max_range);
    const std::vector<bac::Point2D> path = path_source(pose, t);
    // Nav2 carries the goal orientation on the last plan pose; the adapter
    // hands it to the core in the CURRENT body frame, which is what this
    // reproduces.
    std::optional<float> goal_heading;
    if (goal_heading_world)
    {
      goal_heading = wrapAngle(*goal_heading_world - pose.th);
    }
    const bac::Result result =
        core.process(points, path, bac::Twist2D(vx, w, vy), goal_heading);
    const bac::Twist2D command = result.output;

    run.total_ticks++;
    const float commanded_speed = command.speed();
    if (!path.empty() && commanded_speed <= 1e-4f)
    {
      run.stop_ticks++;
    }
    if (step < 20 && commanded_speed > 1e-3f)
    {
      run.translating_ticks_first_second++;
    }
    run.max_speed = std::max(run.max_speed, commanded_speed);
    run.max_abs_vy = std::max(run.max_abs_vy, std::fabs(command.vy));
    run.max_abs_w = std::max(run.max_abs_w, std::fabs(command.w));
    if (commanded_speed > params.limits.v_max + 1e-3f) run.exceeded_speed_cap = true;
    if (std::fabs(command.vy) > params.limits.vy_max + 1e-3f) run.exceeded_lateral_cap = true;
    if (std::fabs(command.w) > params.limits.w_max + 1e-3f) run.exceeded_yaw_cap = true;

    // Acceleration-limited holonomic plant, integrated in the world frame.
    const float dv = acc_v * dt;
    const float dw = acc_w * dt;
    vx += std::max(-dv, std::min(dv, command.v - vx));
    vy += std::max(-dv, std::min(dv, command.vy - vy));
    w += std::max(-dw, std::min(dw, command.w - w));
    const float cs = std::cos(pose.th), sn = std::sin(pose.th);
    const float step_x = (vx * cs - vy * sn) * dt;
    const float step_y = (vx * sn + vy * cs) * dt;
    pose.x += step_x;
    pose.y += step_y;
    pose.th = wrapAngle(pose.th + w * dt);
    run.travelled_distance += std::sqrt(step_x * step_x + step_y * step_y);

    for (const bac::Point2D &point : points)
    {
      run.min_clearance = std::min(run.min_clearance,
                                   std::sqrt(point.x * point.x + point.y * point.y));
      // Body contact: the point is inside the footprint rectangle.
      if (point.x >= params.footprint.rear && point.x <= params.footprint.front &&
          std::fabs(point.y) <= params.footprint.width / 2.0f)
      {
        run.collided = true;
      }
    }

    const float goal_distance =
        std::sqrt((pose.x - goal_x) * (pose.x - goal_x) + (pose.y - goal_y) * (pose.y - goal_y));
    run.min_goal_distance = std::min(run.min_goal_distance, goal_distance);
    if (goal_distance < 0.35f)
    {
      run.reached_goal = true;
    }

    if (window.enabled && pose.x >= window.x_from && pose.x <= window.x_to)
    {
      const float lateral = std::fabs(pose.y - window.center_y);
      lateral_sum += lateral;
      run.lateral_samples++;
      run.max_abs_lateral = std::max(run.max_abs_lateral, lateral);
    }
  }

  run.final_pose = pose;
  if (run.lateral_samples > 0)
  {
    run.mean_abs_lateral =
        static_cast<float>(lateral_sum / static_cast<double>(run.lateral_samples));
  }
  run.final_heading_error =
      std::fabs(wrapAngle(std::atan2(goal_y - start.y, goal_x - start.x) - pose.th));
  return run;
}

/// Contracts every holonomic run must satisfy. Not thresholds: these are the
/// limits the configuration declares, so there is no band to measure.
void
expectLimitsRespected(const OmniRun &run, const std::string &where)
{
  expect(!run.exceeded_speed_cap, where + ": no commanded twist exceeds limits.v_max (max " +
                                      std::to_string(run.max_speed) + " m/s)");
  expect(!run.exceeded_lateral_cap, where + ": no commanded twist exceeds limits.vy_max (max " +
                                        std::to_string(run.max_abs_vy) + " m/s)");
  expect(!run.exceeded_yaw_cap, where + ": no commanded twist exceeds limits.w_max (max " +
                                    std::to_string(run.max_abs_w) + " rad/s)");
}

/// Baseline: an open field with a goal straight ahead.
void
testStraightGoalReached()
{
  bac_sim::World world;
  bac::BacCore core(omniParams());
  const OmniRun run = runOmni(core, world, { 0.0f, 0.0f, 0.0f },
                              bac_sim::gotoPointPath(6.0f, 0.0f), 6.0f, 0.0f, 60.0f);

  expect(!run.collided, "the open-field run has no body contact");
  expect(run.reached_goal, "the holonomic vehicle reaches a goal straight ahead (closest " +
                               std::to_string(run.min_goal_distance) + " m)");
  expectLimitsRespected(run, "open field");
}

/// THE discriminating scenario. A blocking obstacle is rounded by translating
/// sideways, not by turning: the same world under a differential-drive
/// reference has to yaw to get past it. Asserted as a comparison between the
/// two runs rather than against a hand-picked yaw threshold.
void
testSidestepsInsteadOfYawing()
{
  bac_sim::World world;
  world.addBox(3.0f, 0.0f, 0.8f, 0.8f);
  const bac_sim::PathSource path = bac_sim::gotoPointPath(6.0f, 0.0f);

  bac::BacCore reference(diffDriveReferenceParams());
  const OmniRun diff_run =
      runOmni(reference, world, { 0.0f, 0.0f, 0.0f }, path, 6.0f, 0.0f, 60.0f);
  bac::BacCore core(omniParams());
  const OmniRun omni_run = runOmni(core, world, { 0.0f, 0.0f, 0.0f }, path, 6.0f, 0.0f, 60.0f);

  expect(!omni_run.collided, "the holonomic detour has no body contact");
  expect(omni_run.reached_goal, "the holonomic vehicle rounds a blocking obstacle (closest " +
                                    std::to_string(omni_run.min_goal_distance) + " m)");
  expect(diff_run.reached_goal,
         "the differential-drive reference also gets past it, so the comparison below is "
         "between two solutions and not between success and failure");

  expect(omni_run.max_abs_vy > 0.05f,
         "the holonomic detour actually uses lateral velocity (max |vy| " +
             std::to_string(omni_run.max_abs_vy) + " m/s)");
  expect(diff_run.max_abs_vy <= 1e-6f,
         "the differential-drive reference commands no lateral velocity at all (max |vy| " +
             std::to_string(diff_run.max_abs_vy) + ")");
  // A comparison, not a threshold: the differential drive must steer around
  // the obstacle, the holonomic body does not have to.
  expect(omni_run.max_abs_w < diff_run.max_abs_w,
         "the holonomic detour yaws less than the differential-drive one (" +
             std::to_string(omni_run.max_abs_w) + " vs " + std::to_string(diff_run.max_abs_w) +
             " rad/s)");
  expectLimitsRespected(omni_run, "detour");
}

/// A goal behind the robot. The differential drive must rotate BEFORE it can
/// translate; the holonomic body translates from the first tick while it
/// turns. This is the scenario that shows usesRotateBeforeTranslate() is
/// doing something observable.
void
testRearGoalWithoutAligningFirst()
{
  bac_sim::World world;
  const bac_sim::PathSource path = bac_sim::gotoPointPath(-3.0f, 0.0f);

  bac::BacCore reference(diffDriveReferenceParams());
  const OmniRun diff_run =
      runOmni(reference, world, { 0.0f, 0.0f, 0.0f }, path, -3.0f, 0.0f, 90.0f);
  bac::BacCore core(omniParams());
  const OmniRun omni_run = runOmni(core, world, { 0.0f, 0.0f, 0.0f }, path, -3.0f, 0.0f, 90.0f);

  expect(diff_run.translating_ticks_first_second == 0,
         "the differential-drive reference rotates before it translates, so the assertion "
         "below is discriminating (" +
             std::to_string(diff_run.translating_ticks_first_second) + " moving ticks)");
  expect(omni_run.translating_ticks_first_second > 0,
         "the holonomic vehicle starts moving immediately instead of aligning first (" +
             std::to_string(omni_run.translating_ticks_first_second) + " of the first 20 ticks)");
  expect(omni_run.reached_goal,
         "the holonomic vehicle reaches a goal behind it with limits.v_min = 0 (closest " +
             std::to_string(omni_run.min_goal_distance) + " m)");
  expect(!omni_run.collided, "the rear-goal manoeuvre has no body contact");
  expectLimitsRespected(omni_run, "rear goal");
}

/// The yaw rate is a regulator, so it must actually regulate: an off-axis goal
/// has to leave the body pointing along the path it drove.
void
testHeadingRegulatorTracksTheTangent()
{
  bac_sim::World world;
  bac::BacCore core(omniParams());
  const OmniRun run = runOmni(core, world, { 0.0f, 0.0f, 0.0f },
                              bac_sim::gotoPointPath(4.0f, 3.0f), 4.0f, 3.0f, 90.0f);

  expect(run.reached_goal, "the holonomic vehicle reaches an off-axis goal (closest " +
                               std::to_string(run.min_goal_distance) + " m)");
  // Measured band over heading_gain 0.5-3.0 x v_max 0.3-0.5 x goal bearing
  // 0.3-1.2 rad: the regulator settles within 0.02-0.19 rad of the bearing,
  // while a disabled regulator (heading_gain = 0) sits at the full bearing,
  // 0.30-1.20 rad. 0.25 separates the two bands.
  expect(run.final_heading_error < 0.25f,
         "the body ends up pointing along the path it drove (heading error " +
             std::to_string(run.final_heading_error) + " rad)");
  expect(run.max_abs_w > 0.05f,
         "the regulator actually commanded yaw, so the check above is not vacuous (max |w| " +
             std::to_string(run.max_abs_w) + " rad/s)");
  expectLimitsRespected(run, "off-axis goal");
}

/// THE post-condition of the whole controller: the twist it emits must be able
/// to stop, along its own direction of travel, before it touches anything.
/// Asserted on the OUTPUT rather than on the lattice, over randomised worlds
/// AND randomised parameters, so it holds however the command was arrived at.
/// R15 H1/H2 were both found this way - the v = 0 row of the lattice never
/// reached the contact test, and the stopping distance was computed from the
/// forward component - and no per-scenario threshold could have seen either.
/// It then found a third: the emergency fallback kept a residual lateral
/// velocity and a residual yaw whose COMBINED arc could no longer stop, which
/// gating the lateral component alone did not catch.
///
/// The margin is re-derived here independently of the core's own support
/// function, so a mistake in that function shows up as a violation rather than
/// cancelling out.
///
/// Bodies already INSIDE their own margin are excluded: there the documented
/// behaviour is escape - motion away from the offending point - and no speed
/// could satisfy a stopping test. That regime is not covered by any assertion
/// here; see the R15 response for why no separating criterion was found.
void
testEmittedCommandCanAlwaysStop()
{
  unsigned seed = 20260902u;
  const auto next = [&seed]() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<float>((seed >> 8) & 0xFFFFu) / 65535.0f;
  };

  int violations = 0;
  int moving = 0;
  std::string worst;
  for (int trial = 0; trial < 6000; ++trial)
  {
    bac::Params params;
    params.motion_model.type = bac::MotionModelType::OMNI;
    params.limits.v_max = 0.2f + next() * 0.6f;
    params.limits.vy_max = 0.1f + next() * 0.6f;
    params.limits.v_min = 0.0f;
    params.limits.w_max = 0.5f + next() * 1.5f;
    // Acceleration limits far above anything one control period can need, so
    // limitReachableCommand is a no-op and the emitted twist IS the candidate
    // the scorer admitted. That isolates the candidate stage, which is where
    // R15 H1 and H2 lived. The output stage pulls a command back TOWARDS the
    // current velocity and re-checks it against a window capped by the
    // remaining path, so mixing the two regimes here would test a different
    // contract than the one this assertion states.
    params.limits.acc_v = 1000.0f;
    params.limits.acc_w = 1000.0f;
    params.control_period = 0.05f;
    params.stop_decel = 0.4f + next() * 1.6f;
    params.brake_reaction_time = next() * 0.25f;
    params.footprint.front = 0.2f + next() * 0.3f;
    params.footprint.rear = -(0.2f + next() * 0.3f);
    params.footprint.width = 0.3f + next() * 0.5f;
    params.safety_margin.front = 0.05f + next() * 0.3f;
    params.safety_margin.rear = params.safety_margin.front;
    params.safety_margin.side = 0.05f + next() * 0.3f;
    params.avoid_margin.side = 0.4f + next() * 0.6f;
    params.sim_time = 1.5f + next() * 2.0f;
    params.heading_gain = next() * 3.0f;
    bac::BacCore core(params);

    std::vector<bac::Point2D> points;
    const int clusters = 1 + static_cast<int>(next() * 3.0f);
    for (int c = 0; c < clusters; ++c)
    {
      const float bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
      const float distance = 0.35f + next() * 1.3f;
      for (int k = -14; k <= 14; ++k)
      {
        const float spread = 0.05f * static_cast<float>(k);
        points.push_back({ distance * std::cos(bearing) - spread * std::sin(bearing),
                           distance * std::sin(bearing) + spread * std::cos(bearing) });
      }
    }
    const float path_bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
    std::vector<bac::Point2D> path;
    for (int i = 1; i <= 20; ++i)
    {
      const float r = 0.25f * static_cast<float>(i);
      path.push_back({ r * std::cos(path_bearing), r * std::sin(path_bearing) });
    }
    // Drawn from the set the controller itself can command, so the first tick
    // is not asked to recover from a state it would never have produced
    // (limits.v_min is 0 here, so there is no reverse to start from).
    bac::Twist2D current(next() * params.limits.v_max,
                         (next() * 2.0f - 1.0f) * params.limits.w_max,
                         (next() * 2.0f - 1.0f) * params.limits.vy_max);

    const int ticks = 1 + static_cast<int>(next() * 3.0f);
    for (int tick = 0; tick < ticks; ++tick)
    {
      const bac::Result result = core.process(points, path, current);
      const bac::Twist2D out = result.output;
      current = out;
      const float speed = out.speed();
      if (speed <= 1e-3f)
      {
        continue;
      }
      ++moving;

      const float horizon = std::max(speed * params.sim_time, params.min_eval_distance);
      const bac::ArcEvaluation eval =
          core.evaluateArcWindows(points, out, horizon, horizon);
      if (eval.blocking_s >= 1e9f)
      {
        continue;
      }
      const float ux = out.v / speed;
      const float uy = out.vy / speed;
      const float margin =
          ((ux >= 0.0f) ? params.safety_margin.front * ux : params.safety_margin.rear * -ux) +
          params.safety_margin.side * std::fabs(uy);
      const float free_run = eval.blocking_s - margin;
      if (free_run <= 0.0f)
      {
        continue;  // already inside the margin: the escape regime, see above
      }
      const float decel = std::max(params.stop_decel, 0.1f);
      const float needed =
          speed * speed / (2.0f * decel) + speed * params.brake_reaction_time;
      if (free_run <= needed - 1e-4f)
      {
        if (violations == 0)
        {
          worst = "trial " + std::to_string(trial) + " out (" + std::to_string(out.v) + ", " +
                  std::to_string(out.w) + ", " + std::to_string(out.vy) + ") speed " +
                  std::to_string(speed) + ", free run " + std::to_string(free_run) +
                  ", needs " + std::to_string(needed);
        }
        ++violations;
      }
    }
  }

  expect(moving > 3000,
         "the sweep actually produced moving commands, so the check is not vacuous (" +
             std::to_string(moving) + ")");
  expect(violations == 0,
         "every emitted twist can stop before contact along its own direction of travel (" +
             std::to_string(violations) + " violations; first: " + worst + ")");
}

/// The same invariant as above, but with acceleration limits that BIND, so the
/// output reachability stage and the emergency fallback are the code deciding
/// what gets published. R16 H4 found that the R15 fix to the fallback ladder
/// was never executed by any test, because the sweep above deliberately raises
/// the acceleration limits to isolate the candidate stage. Measured over this
/// grid the fallback is entered 318 times and its gate actually rejects the
/// combined twist 67 times, so the ladder is exercised rather than merely
/// reached.
///
/// This pass is also what caught R16 H2: before the deadband was re-checked,
/// it left 7 published twists that could not stop, every one of them a crab
/// whose yaw rate had been zeroed after the arc was checked.
void
testEmittedCommandCanStopWhenAccelerationBinds()
{
  unsigned seed = 7u;
  const auto next = [&seed]() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<float>((seed >> 8) & 0xFFFFu) / 65535.0f;
  };

  int checked = 0;
  int violations = 0;
  std::string worst;
  for (int trial = 0; trial < 40000; ++trial)
  {
    bac::Params params;
    params.motion_model.type = bac::MotionModelType::OMNI;
    params.limits.v_max = 0.2f + next() * 0.6f;
    params.limits.vy_max = 0.1f + next() * 0.6f;
    params.limits.v_min = 0.0f;
    params.limits.w_max = 0.5f + next() * 1.5f;
    // Low enough that one control period cannot cancel the current velocity.
    params.limits.acc_v = 0.1f + next() * 0.5f;
    params.limits.acc_w = 0.2f + next() * 1.0f;
    params.control_period = 0.05f;
    params.stop_decel = 0.4f + next() * 1.6f;
    params.brake_reaction_time = next() * 0.25f;
    params.footprint.front = 0.2f + next() * 0.3f;
    params.footprint.rear = -(0.2f + next() * 0.3f);
    params.footprint.width = 0.3f + next() * 0.5f;
    params.safety_margin.front = 0.05f + next() * 0.3f;
    params.safety_margin.rear = params.safety_margin.front;
    params.safety_margin.side = 0.05f + next() * 0.3f;
    params.avoid_margin.side = 0.4f + next() * 0.6f;
    params.sim_time = 1.5f + next() * 2.0f;
    params.heading_gain = next() * 3.0f;
    bac::BacCore core(params);

    std::vector<bac::Point2D> points;
    const float bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
    const float distance = 0.35f + next() * 1.0f;
    for (int k = -14; k <= 14; ++k)
    {
      const float spread = 0.05f * static_cast<float>(k);
      points.push_back({ distance * std::cos(bearing) - spread * std::sin(bearing),
                         distance * std::sin(bearing) + spread * std::cos(bearing) });
    }
    const float path_bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
    std::vector<bac::Point2D> path;
    for (int i = 1; i <= 20; ++i)
    {
      const float r = 0.25f * static_cast<float>(i);
      path.push_back({ r * std::cos(path_bearing), r * std::sin(path_bearing) });
    }
    const bac::Twist2D current(next() * params.limits.v_max,
                               (next() * 2.0f - 1.0f) * params.limits.w_max,
                               (next() * 2.0f - 1.0f) * params.limits.vy_max);

    const bac::Result result = core.process(points, path, current);
    const bac::Twist2D out = result.output;
    const float speed = out.speed();
    if (speed <= 1e-3f)
    {
      continue;
    }
    const float horizon = std::max(speed * params.sim_time, params.min_eval_distance);
    const bac::ArcEvaluation eval = core.evaluateArcWindows(points, out, horizon, horizon);
    if (eval.blocking_s >= 1e9f)
    {
      continue;
    }
    const float ux = out.v / speed;
    const float uy = out.vy / speed;
    const float margin =
        ((ux >= 0.0f) ? params.safety_margin.front * ux : params.safety_margin.rear * -ux) +
        params.safety_margin.side * std::fabs(uy);
    const float free_run = eval.blocking_s - margin;
    if (free_run <= 0.0f)
    {
      continue;  // the escape regime, as above
    }
    ++checked;
    const float decel = std::max(params.stop_decel, 0.1f);
    const float needed = speed * speed / (2.0f * decel) + speed * params.brake_reaction_time;
    if (free_run <= needed - 1e-4f)
    {
      if (violations == 0)
      {
        worst = "trial " + std::to_string(trial) + " out (" + std::to_string(out.v) + ", " +
                std::to_string(out.w) + ", " + std::to_string(out.vy) + ") free run " +
                std::to_string(free_run) + ", needs " + std::to_string(needed);
      }
      ++violations;
    }
  }

  expect(checked > 2000,
         "the binding-acceleration sweep reached the invariant often enough to matter (" +
             std::to_string(checked) + ")");
  // Measured: 0 here, against 32 with the fallback gate removed and 7 with the
  // deadband left unchecked. Both bands are far from the bound.
  expect(violations == 0,
         "what the output stage and the fallback publish can stop before contact (" +
             std::to_string(violations) + " violations; first: " + worst + ")");
}

/// The v = 0 ROW of the lattice still translates - sideways - so it has to be
/// contact-checked like any other candidate. R15 H1 found it scored as a
/// turn-then-go rotation instead, which routed every pure-crab candidate around
/// the contact test and the stopping test.
///
/// Here forward motion is blocked by a wall exactly at footprint.front plus the
/// front margin, and the path leads sideways into a second wall. Every command
/// that progresses along that path either advances into the wall ahead or
/// slides into the one abeam, so the correct answer is to hold station.
/// Measured: with the row contact-checked the vehicle commands 0.0000 m/s at
/// abeam distances 0.50-0.95 m; scoring it as a rotation instead commands
/// 0.1800 m/s at every one of them.
void
testLateralRowIsContactChecked()
{
  bac::Params params = omniParams();
  params.limits.v_max = 0.6f;
  params.limits.vy_max = 0.6f;
  params.limits.acc_v = 1000.0f;  // isolate the candidate stage from reachability
  params.limits.acc_w = 1000.0f;
  params.heading_gain = 0.0f;
  bac::BacCore core(params);

  std::vector<bac::Point2D> points;
  for (int k = -40; k <= 40; ++k)
  {
    points.push_back({ 0.05f * static_cast<float>(k), 0.70f });  // abeam
    points.push_back({ 0.55f, 0.05f * static_cast<float>(k) });  // ahead, at the margin
  }
  std::vector<bac::Point2D> path;
  for (int i = 1; i <= 20; ++i)
  {
    path.push_back({ 0.0f, 0.25f * static_cast<float>(i) });
  }

  const bac::Result result = core.process(points, path, bac::Twist2D(0.0f, 0.0f, 0.3f));
  expect(result.output.speed() <= 1e-3f,
         "a sideways candidate that cannot clear the wall ahead of it is rejected, not "
         "emitted (speed " + std::to_string(result.output.speed()) + " m/s, twist " +
             std::to_string(result.output.v) + ", " + std::to_string(result.output.w) + ", " +
             std::to_string(result.output.vy) + ")");
}

/// The emergency layer decides on the direction of travel too. R15 H3 found it
/// reading `current.v` only in three places at once: the zone extended along
/// +x alone, a body crabbing at limits.vy_max counted as stationary and skipped
/// the hard brake, and the escape gate projected candidates onto emergency_x
/// while ignoring emergency_y.
///
/// A wall abeam at 0.52 m with the body sliding towards it at 0.3 m/s is inside
/// the zone once the zone follows the direction of travel, and outside it when
/// the zone only grows forwards. Measured over abeam distances 0.28-0.60 m,
/// the generalised layer stops at every distance up to 0.52 m and the
/// forward-axis one is already moving at 0.52 m (vy 0.2600), with
/// min_proximity_norm 0.9187 against 1.3500 at that distance.
void
testEmergencyLayerSeesLateralMotion()
{
  bac::Params params = omniParams();
  params.limits.v_max = 0.4f;
  params.limits.vy_max = 0.4f;
  params.heading_gain = 0.0f;  // hold the heading so the geometry stays fixed
  bac::BacCore core(params);

  std::vector<bac::Point2D> points;
  for (int k = -6; k <= 6; ++k)
  {
    points.push_back({ 0.05f * static_cast<float>(k), 0.52f });
  }
  std::vector<bac::Point2D> path;
  for (int i = 1; i <= 20; ++i)
  {
    path.push_back({ 0.0f, 0.25f * static_cast<float>(i) });  // straight at the wall
  }

  const bac::Result result = core.process(points, path, bac::Twist2D(0.0f, 0.0f, 0.3f));
  expect(result.min_proximity_norm < 1.0f,
         "a wall the body is sliding into is inside the emergency zone (normalised " +
             std::to_string(result.min_proximity_norm) + ")");
  expect(result.output.speed() <= 1e-3f,
         "and the sideways motion is stopped rather than continued (speed " +
             std::to_string(result.output.speed()) + " m/s, vy " +
             std::to_string(result.output.vy) + ")");
}

/// The proximity speed governor moderates speed in front of what the vehicle is
/// about to run into. R16 H3 found it reading the forward axis only: the same
/// wall, approached sideways, produced a command 32% faster than approaching it
/// head-on, because the wall fell into the side-envelope branch rather than the
/// head-on one.
///
/// This is a metamorphic check, not a threshold. With a SQUARE footprint and
/// isotropic margins the problem is invariant under a 90 degree rotation, so
/// driving at a wall and crabbing at the same wall rotated with it must be
/// governed the same way. Measured before the fix: 0.2600 head-on against
/// 0.3157-0.3423 sideways over wall distances 0.60-1.30 m.
void
testGovernorTreatsSidewaysLikeForward()
{
  bac::Params params = omniParams();
  params.limits.v_max = 0.6f;
  params.limits.vy_max = 0.6f;
  // Square body and isotropic margins: a 90 degree rotation is an exact
  // symmetry of the geometry, so any difference is the controller's.
  params.footprint.front = 0.25f;
  params.footprint.rear = -0.25f;
  params.footprint.width = 0.5f;
  params.safety_margin.front = 0.2f;
  params.safety_margin.rear = 0.2f;
  params.safety_margin.side = 0.2f;
  params.heading_gain = 0.0f;  // hold the heading so the rotation stays exact

  // Four rotations of the same problem: driving at the wall, crabbing left at
  // it, reversing at it, crabbing right at it. A square body under isotropic
  // margins makes all four the same problem, so any spread is the controller's.
  float worst_gap = 0.0f;
  std::string worst;
  for (const float wall : { 0.60f, 0.80f, 1.00f, 1.10f, 1.30f })
  {
    float reference = 0.0f;
    for (int rotation = 0; rotation < 4; ++rotation)
    {
      const float angle = 1.57079633f * static_cast<float>(rotation);
      const float ca = std::cos(angle), sa = std::sin(angle);
      const auto rotate = [&](float x, float y) {
        return bac::Point2D(x * ca - y * sa, x * sa + y * ca);
      };

      std::vector<bac::Point2D> points, path;
      for (int k = -30; k <= 30; ++k)
      {
        const float t = 0.05f * static_cast<float>(k);
        points.push_back(rotate(wall, t));
      }
      for (int i = 1; i <= 20; ++i)
      {
        path.push_back(rotate(0.25f * static_cast<float>(i), 0.0f));
      }
      const bac::Point2D velocity = rotate(0.5f, 0.0f);

      bac::BacCore core(params);
      const bac::Result result =
          core.process(points, path, bac::Twist2D(velocity.x, 0.0f, velocity.y));
      if (rotation == 0)
      {
        reference = result.output.speed();
        continue;
      }
      const float gap = result.output.speed() - reference;
      if (gap > worst_gap)
      {
        worst_gap = gap;
        worst = "wall " + std::to_string(wall) + " m, rotation " +
                std::to_string(rotation * 90) + " deg: " +
                std::to_string(result.output.speed()) + " against " +
                std::to_string(reference) + " head-on";
      }
    }
  }

  // The two axes are sampled at different resolutions, so exact equality is not
  // available; what must not happen is the sideways case being governed LESS.
  expect(worst_gap < 0.02f,
         "the governor moderates a wall approached sideways as it does one approached "
         "head-on (worst excess " + std::to_string(worst_gap) + " m/s; " + worst + ")");
}

/// Nav2 asks for a goal POSE, not a goal position. A model that steers with yaw
/// cannot honour the orientation - its heading is whatever direction it had to
/// travel in - but a holonomic body can hold the requested orientation while
/// lateral velocity closes the remaining position error. Asserted against the
/// differential-drive reference in the same world, which ignores it.
void
testGoalOrientationIsHeld()
{
  bac_sim::World world;
  const bac_sim::PathSource path = bac_sim::gotoPointPath(4.0f, 2.0f);
  const float goal_yaw = -1.2f;  // deliberately unlike the approach direction

  bac::BacCore reference(diffDriveReferenceParams());
  const OmniRun diff_run = runOmni(reference, world, { 0.0f, 0.0f, 0.0f }, path, 4.0f, 2.0f,
                                   90.0f, LateralWindow{}, goal_yaw);
  bac::BacCore core(omniParams());
  const OmniRun run =
      runOmni(core, world, { 0.0f, 0.0f, 0.0f }, path, 4.0f, 2.0f, 90.0f, LateralWindow{}, goal_yaw);

  expect(run.reached_goal, "the holonomic vehicle still reaches the goal position (closest " +
                               std::to_string(run.min_goal_distance) + " m)");
  const float error = std::fabs(wrapAngle(run.final_pose.th - goal_yaw));
  const float diff_error = std::fabs(wrapAngle(diff_run.final_pose.th - goal_yaw));
  // The bound is Nav2's own SimpleGoalChecker default yaw_goal_tolerance of
  // 0.25 rad - the tolerance the goal actually has to pass - not a number
  // picked here. Measured over goal orientations 0.0, -1.2, 1.5, 2.5, -2.8 and
  // 3.0 rad, the holonomic error spans 0.009-0.102 rad while the
  // differential-drive reference spans 0.959-1.741 rad: it ends at whatever
  // heading the approach left it with, 0.541 rad, whatever was asked for.
  expect(error < 0.25f,
         "the holonomic vehicle arrives within Nav2's default yaw_goal_tolerance of the "
         "requested orientation (error " + std::to_string(error) + " rad)");
  expect(diff_error > error,
         "the differential-drive reference does not hold it, so the check above is "
         "discriminating (its error " + std::to_string(diff_error) + " rad)");
  expectLimitsRespected(run, "goal orientation");
}

/// heading_gain = 0 holds the heading fixed. A platform with 360 degree
/// sensing may prefer that; it must still reach the goal by translating.
void
testZeroGainHoldsHeadingAndStillArrives()
{
  bac_sim::World world;
  bac::Params params = omniParams();
  params.heading_gain = 0.0f;
  bac::BacCore core(params);
  const OmniRun run = runOmni(core, world, { 0.0f, 0.0f, 0.0f },
                              bac_sim::gotoPointPath(3.0f, 3.0f), 3.0f, 3.0f, 90.0f);

  expect(run.reached_goal,
         "a heading-locked holonomic vehicle still reaches an off-axis goal (closest " +
             std::to_string(run.min_goal_distance) + " m)");
  expect(std::fabs(run.final_pose.th) < 0.05f,
         "heading_gain = 0 holds the heading fixed (final heading " +
             std::to_string(run.final_pose.th) + " rad)");
  expect(run.max_abs_vy > 0.05f,
         "it arrives by translating sideways (max |vy| " + std::to_string(run.max_abs_vy) + ")");
  expectLimitsRespected(run, "heading locked");
}

/// A corridor barely wider than the body. This is where pointing INTO the gap
/// beats crabbing towards it: a crabbing rectangle sweeps wider than a straight
/// one, so at this width a purely lateral correction does not fit.
///
/// Swept over the ENTRY OFFSET rather than run at one. R15 M3 found that a
/// single offset made the kill for the centering term a tripwire on one grid
/// cell, and R15 H4 found a body contact at offset 0.40 that the single-offset
/// fixture could not see. The grid is recorded here so a later round can tell
/// what was and was not covered:
///   entry offset 0.30, 0.36, 0.40 m x corridor width 1.2 m
/// Measured after the R15 fixes, holonomic mean lateral error over that grid is
/// 0.0117-0.0143 m with zero contacts, against 0.0349-0.0357 m for the
/// differential-drive reference. Removing the centering bias fails every cell,
/// not just one.
///
/// The grid stops at 0.40 because that is where the capability stops. At 0.45 m
/// of entry offset in this fixture the vehicle does not get in at all (it holds
/// at x = 1.544), and in a 1.1 m corridor it routes around the outside from
/// 0.36 m. Neither is a contact - the R15 H4 contact is gone - but both are
/// real limits, and docs/algorithm.md states them rather than letting the grid
/// imply they do not exist.
void
testNarrowCorridorCentering()
{
  bac_sim::World world;
  world.addCorridorX(2.0f, 9.5f, 0.0f, 1.2f);
  LateralWindow window;
  window.enabled = true;
  window.center_y = 0.0f;
  window.x_from = 3.0f;
  window.x_to = 9.0f;
  // Inside the corridor at all: half-width 0.60 less the body half-width 0.25.
  // Without this a run that goes AROUND the corridor satisfies both the
  // traverse and the sample-count checks (R15 M6).
  constexpr float kInsideCorridor = 0.35f;

  for (const float offset : { 0.30f, 0.36f, 0.40f })
  {
    const std::string at = "entry offset " + std::to_string(offset) + " m";
    bac::BacCore reference(diffDriveReferenceParams());
    const OmniRun diff_run =
        runOmni(reference, world, { 0.0f, offset, 0.0f },
                bac_sim::gotoPointPath(10.5f, 0.0f), 10.5f, 0.0f, 120.0f, window);
    bac::BacCore core(omniParams());
    const OmniRun run = runOmni(core, world, { 0.0f, offset, 0.0f },
                                bac_sim::gotoPointPath(10.5f, 0.0f), 10.5f, 0.0f, 120.0f, window);

    expect(!run.collided, at + ": corridor centering has no body contact");
    expect(run.lateral_samples > 0,
           at + ": the vehicle reached the measurement window (" +
               std::to_string(run.lateral_samples) + " samples)");
    expect(run.max_abs_lateral < kInsideCorridor,
           at + ": the vehicle went THROUGH the corridor rather than around it (max |y| " +
               std::to_string(run.max_abs_lateral) + " m)");
    expect(run.final_pose.x > 9.0f,
           at + ": the holonomic vehicle traverses a corridor 1.2 m wide (final x " +
               std::to_string(run.final_pose.x) + ")");
    expect(diff_run.final_pose.x > 9.0f,
           at + ": the differential-drive reference traverses it too, so the comparison "
                "below is between two solutions (final x " +
               std::to_string(diff_run.final_pose.x) + ")");
    // A comparison, not a threshold: the two bands measured over the grid above
    // do not touch (0.0117-0.0143 against 0.0349-0.0357).
    expect(run.mean_abs_lateral < diff_run.mean_abs_lateral,
           at + ": the holonomic vehicle holds the centerline better than the "
                "differential-drive reference (mean |y| " +
               std::to_string(run.mean_abs_lateral) + " vs " +
               std::to_string(diff_run.mean_abs_lateral) + " m)");
    expectLimitsRespected(run, "corridor at " + at);
  }
}

/// An obstacle inside the safety polygon. The correct behaviour is to hold
/// position; the assertions are the facts that are observable in a full
/// standstill, not an invariant that a standstill can never violate.
void
testSafetyStopHoldsPosition()
{
  bac_sim::World world;
  world.addWall(0.5f, -1.0f, 0.5f, 1.0f);  // inside the 0.55 m front safety edge
  bac::BacCore core(omniParams());
  const OmniRun run = runOmni(core, world, { 0.0f, 0.0f, 0.0f },
                              bac_sim::gotoPointPath(10.0f, 0.0f), 10.0f, 0.0f, 8.0f);

  expect(!run.collided, "the safety stop does not touch the wall (min clearance " +
                            std::to_string(run.min_clearance) + " m)");
  expect(run.final_pose.x < 0.05f, "the vehicle never advances towards the wall (final x " +
                                       std::to_string(run.final_pose.x) + ")");
  expect(run.travelled_distance < 0.15f,
         "the vehicle holds position (travelled " + std::to_string(run.travelled_distance) + " m)");
  expect(run.stop_ticks > run.total_ticks / 2,
         "most ticks command a standstill (" + std::to_string(run.stop_ticks) + " of " +
             std::to_string(run.total_ticks) + ")");
  expectLimitsRespected(run, "safety stop");
}

/// Keys config/bac_controller_omni.yaml may carry without this scenario
/// consuming them. See shipped_config.hpp for what the guard enforces.
const std::vector<std::string> kAllowedUnconsumedKeys = {
    "controller_server",           // block header
    "ros__parameters",             // block header
    "FollowPath",                  // block header: the plugin block itself
    "controller_frequency",        // Nav2 control loop rate, not a BAC parameter
    "controller_plugins",          // Nav2 plugin list
    "plugin",                      // plugin class name (checked by the docker gate)
    "scan_topic",                  // BacController scan adapter, not BacCore
    "scan_timeout",                //   "
    "scan_downsample",             //   "
    "scan_min_points",             //   "
    "scan_inf_is_valid",           //   "
    "diagnostics_publish_period",  // diagnostics only
};

bool g_shipped_config_loaded = false;

bac::Params
shippedExampleParams()
{
  const bac_sim::ConfigFile config = bac_sim::readConfigFile(BAC_OMNI_CONFIG_PATH);
  bac::Params params;
  if (!config.readable || config.entries.empty())
  {
    expect(false, "the shipped holonomic configuration is readable at " BAC_OMNI_CONFIG_PATH);
    return params;
  }

  bool complete = true;
  std::vector<std::string> consumed;
  const auto value_of = [&](const char *key) -> const bac_sim::ConfigEntry * {
    consumed.push_back(key);
    const auto found = config.entries.find(key);
    if (found == config.entries.end())
    {
      expect(false, std::string("the shipped configuration declares ") + key);
      complete = false;
      return nullptr;
    }
    if (found->second.section != "FollowPath")
    {
      expect(false, std::string("the shipped configuration keeps ") + key +
                        " inside the FollowPath block users copy (found under '" +
                        found->second.section + "' on line " +
                        std::to_string(found->second.line) + ")");
      complete = false;
      return nullptr;
    }
    return &found->second;
  };
  const auto number = [&](const char *key, float &field) {
    const bac_sim::ConfigEntry *entry = value_of(key);
    if (entry == nullptr)
    {
      return;
    }
    char *end = nullptr;
    const float parsed = std::strtof(entry->value.c_str(), &end);
    if (end == entry->value.c_str() || *end != '\0' || !std::isfinite(parsed))
    {
      expect(false, std::string("the shipped configuration gives ") + key +
                        " a plain finite number (got '" + entry->value + "' on line " +
                        std::to_string(entry->line) + ")");
      complete = false;
      return;
    }
    field = parsed;
  };
  const auto integer = [&](const char *key, int &field) {
    float value = 0.0f;
    number(key, value);
    field = static_cast<int>(value);
  };

  const bac_sim::ConfigEntry *model = value_of("motion_model.type");
  if (model == nullptr || model->value != "omni")
  {
    expect(false, std::string("the shipped configuration selects the holonomic model (got '") +
                      (model == nullptr ? std::string("<missing>") : model->value) + "')");
    complete = false;
  }
  params.motion_model.type = bac::MotionModelType::OMNI;

  number("footprint.front", params.footprint.front);
  number("footprint.rear", params.footprint.rear);
  number("footprint.width", params.footprint.width);
  number("safety_margin.front", params.safety_margin.front);
  number("safety_margin.rear", params.safety_margin.rear);
  number("safety_margin.side", params.safety_margin.side);
  number("avoid_margin.side", params.avoid_margin.side);
  number("limits.v_max", params.limits.v_max);
  number("limits.v_min", params.limits.v_min);
  number("limits.vy_max", params.limits.vy_max);
  number("limits.w_max", params.limits.w_max);
  number("limits.acc_v", params.limits.acc_v);
  number("limits.acc_w", params.limits.acc_w);
  number("control_period", params.control_period);
  number("stop_decel", params.stop_decel);
  number("brake_reaction_time", params.brake_reaction_time);
  number("heading_gain", params.heading_gain);
  integer("vy_samples", params.vy_samples);
  number("weights.clearance", params.weights.clearance);
  number("weights.path_dist", params.weights.path_dist);
  number("weights.balance", params.weights.balance);
  number("weights.heading", params.weights.heading);
  number("weights.hysteresis", params.weights.hysteresis);
  number("weights.squeeze", params.weights.squeeze);
  number("sim_time", params.sim_time);

  for (const std::string &duplicate : config.duplicates)
  {
    expect(false, "the shipped configuration declares " + duplicate +
                      " only once; a repeated key means a second block is "
                      "masking the one users copy");
  }
  for (const std::string &section : config.sections)
  {
    expect(bac_sim::isAllowedUnconsumed(section, kAllowedUnconsumedKeys),
           "the shipped configuration block '" + section + "' is one this suite knows about");
  }
  for (const auto &entry : config.entries)
  {
    if (std::find(consumed.begin(), consumed.end(), entry.first) != consumed.end())
    {
      continue;
    }
    expect(bac_sim::isAllowedUnconsumed(entry.first, kAllowedUnconsumedKeys),
           "the shipped configuration key " + entry.first + " (line " +
               std::to_string(entry.second.line) +
               ") is exercised by this suite or listed in kAllowedUnconsumedKeys on purpose");
  }

  g_shipped_config_loaded = complete;
  return params;
}

/// Runs the shipped example end to end, so a yaml edit cannot ship a
/// configuration that fails this suite while every test stays green.
void
testShippedExampleConfiguration()
{
  bac_sim::World world;
  world.addBox(3.0f, 0.0f, 0.8f, 0.8f);
  bac::BacCore core(shippedExampleParams());
  expect(g_shipped_config_loaded,
         "every value the scenario needs was read from the shipped configuration");
  const OmniRun run = runOmni(core, world, { 0.0f, 0.0f, 0.0f },
                              bac_sim::gotoPointPath(6.0f, 0.0f), 6.0f, 0.0f, 60.0f);

  expect(!run.collided, "the shipped holonomic example has no body contact");
  expect(run.reached_goal, "the shipped holonomic example reaches its goal (closest " +
                               std::to_string(run.min_goal_distance) + " m)");
  expect(run.max_abs_vy > 0.05f,
         "the shipped configuration grants usable lateral authority (max |vy| " +
             std::to_string(run.max_abs_vy) + " m/s)");
  expectLimitsRespected(run, "shipped configuration");
}

}  // namespace

int
main()
{
  testStraightGoalReached();
  testSidestepsInsteadOfYawing();
  testRearGoalWithoutAligningFirst();
  testHeadingRegulatorTracksTheTangent();
  testEmittedCommandCanAlwaysStop();
  testEmittedCommandCanStopWhenAccelerationBinds();
  testLateralRowIsContactChecked();
  testEmergencyLayerSeesLateralMotion();
  testGovernorTreatsSidewaysLikeForward();
  testGoalOrientationIsHeld();
  testZeroGainHoldsHeadingAndStillArrives();
  testNarrowCorridorCentering();
  testSafetyStopHoldsPosition();
  testShippedExampleConfiguration();

  if (failures != 0)
  {
    std::cerr << failures << " holonomic scenario check(s) failed\n";
    return 1;
  }
  std::cout << "All holonomic scenario checks passed\n";
  return 0;
}
