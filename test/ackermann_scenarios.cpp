/**
 * @file ackermann_scenarios.cpp
 * @brief Closed-loop body-curvature regressions for the Ackermann BAC policy
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * The Nav2-level Ackermann contract ends at body Twist, as it does in MPPI:
 * BAC emits (forward speed, yaw rate) on a curvature that respects
 * `turn_radius_min`, and the downstream vehicle controller maps it to
 * road-wheel steering. These regressions therefore assert on the COMMANDED
 * curvature, and drive a plant that a real steered vehicle can stand in for:
 * speed is acceleration-limited and curvature slews at a bounded rate, so the
 * commands are checked against a vehicle that cannot change its steering
 * instantly.
 */

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "sim_runner.hpp"
#include "sim_world.hpp"

#include <algorithm>
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

/// Acceleration- and steering-rate-limited plant driven by a body Twist.
struct AckermannPlant
{
  float acc_v = 0.8f;           // [m/s^2] forward-speed tracking
  float curvature_rate = 1.0f;  // [1/(m*s)] steering slew at the body level
};

struct AckermannRun
{
  bac_sim::Pose final_pose;
  bool collided = false;
  bool reached_goal = false;
  bool offered_in_place_rotation = false;
  bool commanded_reverse = false;
  bool violated_turning_radius = false;
  bool stopped_at_end = false;
  float min_clearance = 1e9f;
  float max_abs_curvature = 0.0f;
  float min_goal_distance = 1e9f;
  float travelled_distance = 0.0f;   // path length actually driven
  float max_curvature_step = 0.0f;   // largest per-tick commanded curvature jump
};

AckermannRun
runAckermann(bac::BacCore &core, const bac_sim::World &world,
             const bac_sim::Pose &start, const bac_sim::PathSource &path_source,
             float goal_x, float goal_y, float simulation_time,
             const AckermannPlant &plant = AckermannPlant{})
{
  constexpr float dt = 0.05f;
  AckermannRun run;
  bac_sim::Pose pose = start;
  float speed = 0.0f;
  float body_curvature = 0.0f;
  float yaw_rate = 0.0f;
  float previous_curvature = 0.0f;
  bool had_previous_curvature = false;
  const bac::Params &params = core.params();
  const float curvature_max = 1.0f / params.turn_radius_min;

  const int steps = static_cast<int>(simulation_time / dt);
  for (int step = 0; step < steps; ++step)
  {
    const float t = static_cast<float>(step) * dt;
    const bac::Twist2D current(speed, yaw_rate);
    const std::vector<bac::Point2D> points =
        bac_sim::simulateLidar(world, pose, 720, params.max_range);
    const std::vector<bac::Point2D> path = path_source(pose, t);
    const bac::Result result = core.process(points, path, current);
    const bac::Twist2D command = result.output;

    if (std::fabs(command.v) <= 1e-4f && std::fabs(command.w) > 1e-4f)
    {
      run.offered_in_place_rotation = true;
    }
    if (command.v < -1e-4f)
    {
      run.commanded_reverse = true;
    }

    float commanded_curvature = 0.0f;
    if (std::fabs(command.v) > 1e-4f)
    {
      commanded_curvature = command.w / command.v;
      if (had_previous_curvature)
      {
        run.max_curvature_step = std::max(
            run.max_curvature_step, std::fabs(commanded_curvature - previous_curvature));
      }
      previous_curvature = commanded_curvature;
      had_previous_curvature = true;
      run.max_abs_curvature =
          std::max(run.max_abs_curvature, std::fabs(commanded_curvature));
      if (std::fabs(commanded_curvature) > curvature_max + 1e-4f)
      {
        run.violated_turning_radius = true;
      }
    }
    else
    {
      commanded_curvature = body_curvature;  // hold the wheels while stopped
    }

    // --- plant: accel-limited speed, rate-limited body curvature ---
    speed += bac_sim::clampf(command.v - speed, -plant.acc_v * dt, plant.acc_v * dt);
    const float dk = plant.curvature_rate * dt;
    body_curvature += bac_sim::clampf(commanded_curvature - body_curvature, -dk, dk);
    body_curvature = bac_sim::clampf(body_curvature, -curvature_max, curvature_max);
    yaw_rate = speed * body_curvature;

    const float heading_mid = pose.th + yaw_rate * dt / 2.0f;
    pose.x += speed * std::cos(heading_mid) * dt;
    pose.y += speed * std::sin(heading_mid) * dt;
    pose.th += yaw_rate * dt;
    run.travelled_distance += std::fabs(speed) * dt;

    const float clearance = bac_sim::robotClearance(pose, params.footprint, world);
    run.min_clearance = std::min(run.min_clearance, clearance);
    if (clearance <= 0.0f)
    {
      run.collided = true;
      break;
    }
    const float goal_distance = std::hypot(goal_x - pose.x, goal_y - pose.y);
    run.min_goal_distance = std::min(run.min_goal_distance, goal_distance);
    if (goal_distance < 0.45f)
    {
      run.reached_goal = true;
    }
    run.stopped_at_end = std::fabs(speed) < 0.02f;
  }
  run.final_pose = pose;
  return run;
}

bac::Params
ackermannParams(float turn_radius_min = 1.0f)
{
  bac::Params params;
  params.motion_model.type = bac::MotionModelType::ACKERMANN;
  params.turn_radius_min = turn_radius_min;
  params.limits.v_min = 0.0f;
  params.limits.v_max = 0.4f;
  params.weights.hysteresis = 0.3f;
  return params;
}

bac::BacCore
makeAckermannCore(float turn_radius_min = 1.0f)
{
  return bac::BacCore(ackermannParams(turn_radius_min));
}

/// The same tuning under the differential-drive policy, so a scenario can show
/// that the Ackermann restriction is what changed the behavior.
bac::BacCore
makeDiffDriveReference()
{
  bac::Params params = ackermannParams();
  params.motion_model.type = bac::MotionModelType::DIFF_DRIVE;
  params.turn_radius_min = 0.25f;
  return bac::BacCore(params);
}

void
testGentleGoalTurn()
{
  bac_sim::World world;
  bac::BacCore core = makeAckermannCore();
  const AckermannRun run = runAckermann(
      core, world, { 0.0f, 0.0f, 0.0f }, bac_sim::gotoPointPath(8.0f, 2.0f),
      8.0f, 2.0f, 70.0f);

  expect(run.reached_goal,
         "Ackermann plant reaches a forward lateral goal (final " +
             std::to_string(run.final_pose.x) + ", " +
             std::to_string(run.final_pose.y) + ", closest " +
             std::to_string(run.min_goal_distance) + ")");
  expect(!run.offered_in_place_rotation, "goal turn never uses in-place rotation");
  expect(!run.violated_turning_radius,
         "goal turn respects minimum turning radius (max curvature " +
             std::to_string(run.max_abs_curvature) + ")");
}

void
testOffsetCorridorEntry()
{
  bac_sim::World world;
  world.addCorridorX(2.0f, 8.0f, 0.0f, 1.8f);
  bac::BacCore core = makeAckermannCore();
  const AckermannRun run = runAckermann(
      core, world, { 0.0f, 0.25f, 0.0f }, bac_sim::gotoPointPath(9.0f, 0.0f),
      9.0f, 0.0f, 90.0f);

  expect(!run.collided, "Ackermann corridor entry has no body contact");
  expect(run.reached_goal,
         "Ackermann vehicle traverses the offset corridor (final " +
             std::to_string(run.final_pose.x) + ", " +
             std::to_string(run.final_pose.y) + ")");
  expect(run.min_clearance > 0.1f,
         "Ackermann corridor entry retains physical clearance (" +
             std::to_string(run.min_clearance) + " m)");
  expect(!run.offered_in_place_rotation, "corridor correction never uses in-place rotation");
  expect(!run.violated_turning_radius,
         "corridor correction respects minimum turning radius (max curvature " +
             std::to_string(run.max_abs_curvature) + ")");
}

/// A steered vehicle cannot pivot around an obstacle; it has to commit to an
/// arc early enough that the minimum radius still clears the body.
void
testObstacleDetour()
{
  bac_sim::World world;
  world.addBox(4.0f, 0.0f, 1.0f, 1.0f);
  bac::BacCore core = makeAckermannCore();
  const AckermannRun run = runAckermann(
      core, world, { 0.0f, 0.0f, 0.0f }, bac_sim::gotoPointPath(9.0f, 0.0f),
      9.0f, 0.0f, 90.0f);

  expect(!run.collided, "Ackermann detour has no body contact");
  expect(run.reached_goal,
         "Ackermann vehicle rounds a blocking obstacle (final " +
             std::to_string(run.final_pose.x) + ", " +
             std::to_string(run.final_pose.y) + ", closest " +
             std::to_string(run.min_goal_distance) + ")");
  expect(!run.offered_in_place_rotation, "detour never uses in-place rotation");
  expect(!run.violated_turning_radius,
         "detour respects minimum turning radius (max curvature " +
             std::to_string(run.max_abs_curvature) + ")");
}

/// With `limits.v_min = 0`, a dead end has no Ackermann escape: the correct
/// behavior is to brake, not to spin in place or invent a reverse command.
void
testDeadEndStopsWithoutRotating()
{
  bac_sim::World world;
  world.addCorridorX(1.0f, 5.0f, 0.0f, 1.6f);
  world.addWall(5.0f, -0.8f, 5.0f, 0.8f);
  bac::BacCore core = makeAckermannCore();
  const AckermannRun run = runAckermann(
      core, world, { 0.0f, 0.0f, 0.0f }, bac_sim::gotoPointPath(9.0f, 0.0f),
      9.0f, 0.0f, 40.0f);

  expect(!run.collided,
         "Ackermann vehicle stops before the dead-end wall (min clearance " +
             std::to_string(run.min_clearance) + " m)");
  expect(run.stopped_at_end,
         "Ackermann vehicle is at rest in the dead end (final x " +
             std::to_string(run.final_pose.x) + ")");
  expect(!run.offered_in_place_rotation,
         "dead end never falls back to in-place rotation");
  expect(!run.commanded_reverse,
         "dead end respects limits.v_min = 0 and never commands reverse");
}

/// The discriminating scenario for the no-in-place-rotation property. A goal
/// directly behind the robot is the one case where the differential-drive
/// policy turns on the spot, so it shows that the Ackermann assertions in the
/// other scenarios are testing a real restriction and not an unreachable
/// candidate. A forward-only steered vehicle has no escape here and must
/// brake, leaving the recovery to Nav2; with reverse enabled it backs up.
void
testRearGoal()
{
  bac_sim::World world;
  const bac_sim::PathSource path = bac_sim::gotoPointPath(-3.0f, 0.0f);

  bac::BacCore reference = makeDiffDriveReference();
  const AckermannRun diff_run =
      runAckermann(reference, world, { 0.0f, 0.0f, 0.0f }, path, -3.0f, 0.0f, 60.0f);
  expect(diff_run.offered_in_place_rotation,
         "the differential-drive reference does turn on the spot for a rear goal, "
         "so the Ackermann restriction below is observable");

  bac::BacCore forward_only = makeAckermannCore();
  const AckermannRun forward_run =
      runAckermann(forward_only, world, { 0.0f, 0.0f, 0.0f }, path, -3.0f, 0.0f, 60.0f);
  expect(!forward_run.offered_in_place_rotation,
         "forward-only Ackermann never turns on the spot for a rear goal");
  expect(!forward_run.commanded_reverse,
         "forward-only Ackermann respects limits.v_min = 0 for a rear goal");
  // Not "did not approach the goal": that is satisfied by driving away from it.
  // The vehicle must actually hold station.
  expect(forward_run.travelled_distance < 0.2f,
         "forward-only Ackermann holds station rather than driving off (travelled " +
             std::to_string(forward_run.travelled_distance) + " m)");
  expect(std::hypot(forward_run.final_pose.x, forward_run.final_pose.y) < 0.15f,
         "forward-only Ackermann ends where it started (final " +
             std::to_string(forward_run.final_pose.x) + ", " +
             std::to_string(forward_run.final_pose.y) + ")");
  expect(forward_run.stopped_at_end, "forward-only Ackermann is at rest at the end");

  bac::Params reversing_params = ackermannParams();
  reversing_params.limits.v_min = -0.15f;
  bac::BacCore reversing(reversing_params);
  const AckermannRun reverse_run =
      runAckermann(reversing, world, { 0.0f, 0.0f, 0.0f }, path, -3.0f, 0.0f, 90.0f);
  expect(reverse_run.reached_goal,
         "Ackermann reaches a rear goal by reversing when limits.v_min allows it "
         "(closest " + std::to_string(reverse_run.min_goal_distance) + " m)");
  expect(!reverse_run.offered_in_place_rotation,
         "the reversing maneuver still never turns on the spot");
  expect(!reverse_run.violated_turning_radius,
         "the reversing maneuver respects minimum turning radius (max curvature " +
             std::to_string(reverse_run.max_abs_curvature) + ")");
}

/// The turning-radius constraint has to bind: a tighter vehicle in the same
/// world must be allowed more curvature than a wider one.
void
testTurningRadiusBinds()
{
  bac_sim::World world;
  bac::BacCore tight = makeAckermannCore(0.8f);
  bac::BacCore wide = makeAckermannCore(3.0f);
  const bac_sim::PathSource path = bac_sim::gotoPointPath(4.0f, 3.0f);

  const AckermannRun tight_run =
      runAckermann(tight, world, { 0.0f, 0.0f, 0.0f }, path, 4.0f, 3.0f, 60.0f);
  const AckermannRun wide_run =
      runAckermann(wide, world, { 0.0f, 0.0f, 0.0f }, path, 4.0f, 3.0f, 60.0f);

  expect(!tight_run.violated_turning_radius && !wide_run.violated_turning_radius,
         "both vehicles respect their own minimum turning radius");
  expect(wide_run.max_abs_curvature < tight_run.max_abs_curvature,
         "a larger turn_radius_min actually restricts commanded curvature (wide " +
             std::to_string(wide_run.max_abs_curvature) + " vs tight " +
             std::to_string(tight_run.max_abs_curvature) + ")");
  // `violated_turning_radius` above already compares against 1/turn_radius_min,
  // so assert the independent fact instead: the tight vehicle actually uses
  // curvature the wide one is forbidden from reaching.
  expect(tight_run.max_abs_curvature > 1.0f / 3.0f + 1e-3f,
         "the tight vehicle uses curvature beyond the wide vehicle's limit (tight " +
             std::to_string(tight_run.max_abs_curvature) + ")");
}

}  // namespace

int
main()
{
  testGentleGoalTurn();
  testOffsetCorridorEntry();
  testObstacleDetour();
  testDeadEndStopsWithoutRotating();
  testRearGoal();
  testTurningRadiusBinds();

  if (failures != 0)
  {
    std::cerr << failures << " Ackermann scenario check(s) failed\n";
    return 1;
  }
  std::cout << "All Ackermann closed-loop scenarios passed\n";
  return 0;
}
