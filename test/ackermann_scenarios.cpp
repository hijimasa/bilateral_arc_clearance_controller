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
#include "shipped_config.hpp"
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

/// Optional corridor-centering window: lateral error is aggregated only while
/// the vehicle is between x_from and x_to, i.e. inside the corridor proper.
struct LateralWindow
{
  bool enabled = false;
  float center_y = 0.0f;
  float x_from = 0.0f;
  float x_to = 0.0f;
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
  float max_abs_yaw_rate = 0.0f;   // largest commanded |w| (vs limits.w_max)
  int yaw_bound_ticks = 0;         // ticks where limits.w_max, not the turning
                                   // circle, is the binding term of the bound
  float min_goal_distance = 1e9f;
  float travelled_distance = 0.0f;   // path length actually driven
  float max_curvature_step = 0.0f;   // largest per-tick commanded curvature jump
  int stop_ticks = 0;                // commanded standstill while a path existed
  int curvature_sign_changes = 0;    // steering oscillation
  float mean_abs_lateral = 0.0f;     // vs the corridor centerline, in-window
  float max_abs_lateral = 0.0f;
  int lateral_samples = 0;           // 0 means the centering metrics are vacuous
};

AckermannRun
runAckermann(bac::BacCore &core, const bac_sim::World &world,
             const bac_sim::Pose &start, const bac_sim::PathSource &path_source,
             float goal_x, float goal_y, float simulation_time,
             const AckermannPlant &plant = AckermannPlant{},
             const LateralWindow &window = LateralWindow{})
{
  constexpr float dt = 0.05f;
  AckermannRun run;
  bac_sim::Pose pose = start;
  float speed = 0.0f;
  float body_curvature = 0.0f;
  float yaw_rate = 0.0f;
  float previous_curvature = 0.0f;
  bool had_previous_curvature = false;
  float previous_sign = 0.0f;
  double lateral_sum = 0.0;
  int lateral_samples = 0;
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

    run.max_abs_yaw_rate = std::max(run.max_abs_yaw_rate, std::fabs(command.w));
    if (std::fabs(command.v) / params.turn_radius_min > params.limits.w_max)
    {
      ++run.yaw_bound_ticks;
    }
    if (std::fabs(command.v) <= 1e-4f && std::fabs(command.w) > 1e-4f)
    {
      run.offered_in_place_rotation = true;
    }
    if (command.v < -1e-4f)
    {
      run.commanded_reverse = true;
    }
    if (!path.empty() && std::fabs(command.v) <= 1e-4f)
    {
      ++run.stop_ticks;
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
      const float sign = commanded_curvature > 1e-3f
                             ? 1.0f
                             : (commanded_curvature < -1e-3f ? -1.0f : 0.0f);
      if (sign != 0.0f)
      {
        if (previous_sign != 0.0f && sign != previous_sign)
        {
          ++run.curvature_sign_changes;
        }
        previous_sign = sign;
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
      // Hold the wheels while stopped, and break the per-tick chain: a step
      // measured across a standstill would not be a per-tick step.
      commanded_curvature = body_curvature;
      had_previous_curvature = false;
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

    if (window.enabled && pose.x >= window.x_from && pose.x <= window.x_to)
    {
      const float lateral = std::fabs(pose.y - window.center_y);
      lateral_sum += lateral;
      ++lateral_samples;
      run.max_abs_lateral = std::max(run.max_abs_lateral, lateral);
    }
  }
  run.lateral_samples = lateral_samples;
  if (lateral_samples > 0)
  {
    run.mean_abs_lateral = static_cast<float>(lateral_sum / lateral_samples);
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
  // These are the values config/bac_controller_ackermann.yaml ships, so the
  // suite defends the configuration users actually copy. The Weights default
  // (0.6) is tuned for differential drive and makes the vehicle orbit here.
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
  // Steering smoothness: no single 50 ms tick may swing more than half of the
  // full lock-to-lock curvature range (2 / turn_radius_min).
  expect(run.max_curvature_step < 1.0f / core.params().turn_radius_min,
         "corridor correction does not swing the steering half a lock in one tick (" +
             std::to_string(run.max_curvature_step) + " 1/m)");
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
  // Deliberately no min_clearance bound here (R14 M3). Both sides were swept
  // over the SAME grid - v_max 0.35/0.40/0.45/0.50/0.55/0.60 x sim_time
  // 2.0/2.5/3.0/3.5 x footprint.width 0.50/0.60/0.70 x plant curvature_rate
  // 0.7/1.0/1.5/2.0, 288 runs each, every run contact-free:
  //   correct controller                        0.144282 - 0.524244 m
  //   speed governor ignoring the caller's cap  0.117452 - 0.524440 m
  // The bands overlap completely, so no threshold on this metric separates
  // correct from broken. The 0.25 m bound R13 M2 placed here was a tripwire:
  // it failed on the UNMODIFIED controller at v_max 0.45/0.50/0.55/0.60 and
  // sim_time 3.0/3.5 while the mutation it targeted sits at 0.182466.
  // That mutation (`v_hi = std::min(params_.limits.v_max, current.v +
  // window_dv)`, i.e. sampling past the proximity governor's cap) stays dead:
  // "a lowered speed cap lowers the lattice" in ackermann_motion_model_unit.cpp
  // fails 12 times on it, deterministically and with no threshold at all.
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
  expect(!run.violated_turning_radius,
         "braking into a dead end respects minimum turning radius (max curvature " +
             std::to_string(run.max_abs_curvature) + ")");
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

/// An obstacle inside the safety polygon. A forward-only Ackermann vehicle has
/// no escape - it cannot reverse and cannot turn on the spot - so the only
/// correct behaviour is to hold position. The differential-drive counterpart
/// (scenarios.cpp, safety_stop) backs off instead.
void
testSafetyStopHoldsPosition()
{
  bac_sim::World world;
  world.addWall(0.65f, -1.0f, 0.65f, 1.0f);  // inside the 0.7 m front safety edge
  bac::BacCore core = makeAckermannCore();
  const AckermannRun run = runAckermann(
      core, world, { 0.0f, 0.0f, 0.0f }, bac_sim::gotoPointPath(10.0f, 0.0f),
      10.0f, 0.0f, 8.0f);

  expect(!run.collided,
         "forward-only Ackermann does not touch a wall inside the safety polygon "
         "(min clearance " + std::to_string(run.min_clearance) + " m)");
  expect(run.final_pose.x < 0.01f,
         "forward-only Ackermann never advances towards the wall (final x " +
             std::to_string(run.final_pose.x) + ")");
  expect(run.travelled_distance < 0.05f,
         "forward-only Ackermann holds position (travelled " +
             std::to_string(run.travelled_distance) + " m)");
  // Measured (R14 M6): all 160 ticks of the 8.0 s / 0.05 s run are stop ticks
  // (stop_ticks = 160, max |curvature| = 0.000000, max |w| = 0.000000), which
  // is the property this scenario is about. `violated_turning_radius` is only
  // ever set on the |command.v| > 1e-4 branch, so on the correct controller it
  // is UNREACHABLE here; asserting it would read as a curvature check that
  // cannot fail. The two assertions below state what is actually observable
  // instead - the run is a complete standstill - and they are what a change
  // that starts moving this vehicle would have to falsify.
  expect(run.stop_ticks == 160,
         "the safety stop commands a standstill on every tick (" +
             std::to_string(run.stop_ticks) + " of 160)");
  expect(run.max_abs_curvature <= 0.0f,  // a max of absolute values: <= 0 is == 0
         "the safety stop never commands any curvature at all (max |curvature| " +
             std::to_string(run.max_abs_curvature) + " 1/m, max |w| " +
             std::to_string(run.max_abs_yaw_rate) + " rad/s)");
  expect(!run.offered_in_place_rotation, "the safety stop never turns on the spot");
  expect(!run.commanded_reverse, "the safety stop respects limits.v_min = 0");
}

/// The same wall with reverse enabled. The vehicle must back off, and it must
/// do so smoothly: a hysteresis term measured in the wrong quantity shows up
/// here as steering chatter rather than as a wrong position.
void
testSafetyStopReverseEscape()
{
  bac_sim::World world;
  world.addWall(0.65f, -1.0f, 0.65f, 1.0f);
  bac::Params params = ackermannParams();
  params.limits.v_min = -0.15f;
  bac::BacCore core(params);
  const AckermannRun run = runAckermann(
      core, world, { 0.0f, 0.0f, 0.0f }, bac_sim::gotoPointPath(10.0f, 0.0f),
      10.0f, 0.0f, 8.0f);

  expect(!run.collided,
         "the reverse escape does not touch the wall (min clearance " +
             std::to_string(run.min_clearance) + " m)");
  expect(run.final_pose.x < 0.01f,
         "the reverse escape never advances towards the wall (final x " +
             std::to_string(run.final_pose.x) + ")");
  expect(run.commanded_reverse, "the reverse escape actually reverses");
  expect(!run.offered_in_place_rotation, "the reverse escape never turns on the spot");
  expect(!run.violated_turning_radius,
         "the reverse escape respects minimum turning radius (max curvature " +
             std::to_string(run.max_abs_curvature) + ")");
  // Deliberately no curvature_sign_changes bound here (R14 M4). R13 swept only
  // the correct side; sweeping BOTH sides over the same grid - v_max
  // 0.30/0.40/0.50/0.55/0.60 x plant curvature_rate 0.7/1.0/1.3/1.6/2.0 x
  // horizon 8/12/20/40 s, 100 runs each - the bands overlap:
  //          horizon 8 s   12 s     20 s     40 s
  //   correct   0 - 17    0 - 27   0 - 47   0 - 97
  //   broken    0 - 51    3 - 86   6 - 156 15 - 331
  // The broken side reaches 0 (v_max 0.50, curvature_rate >= 1.3) and 11-12
  // (v_max 0.55) - below the correct side's own maximum - so no bound on the
  // count separates. Neither does a rate: per second the correct side spans
  // 0.00-2.42 and the broken side 0.00-8.28; per metre 0.0-59.7 against
  // 0.0-360.1. Both are also horizon-dependent cumulative counts. The
  // mutation this bound targeted (hysteresis measured in yaw rate instead of
  // curvature) stays dead in ackermann_motion_model_unit.cpp, where "the same
  // arc at a different speed is not a steering change" and "the same yaw rate
  // at a different speed IS a steering change" both fail on it by
  // construction.
}

/// Offset entry into a long narrow corridor. This is where steering quality is
/// measurable: the vehicle must converge onto the centerline and hold it
/// without stopping, which coarse-only candidate generation cannot do.
void
testNarrowCorridorCentering()
{
  bac_sim::World world;
  world.addCorridorX(3.0f, 11.0f, 0.0f, 1.8f);
  bac::BacCore core = makeAckermannCore();
  LateralWindow window;
  window.enabled = true;
  window.center_y = 0.0f;
  window.x_from = 4.0f;
  window.x_to = 10.5f;
  const AckermannRun run = runAckermann(
      core, world, { 0.0f, 0.4f, 0.0f }, bac_sim::gotoPointPath(12.0f, 0.0f),
      12.0f, 0.0f, 120.0f, AckermannPlant{}, window);

  expect(!run.collided, "narrow corridor traverse has no body contact");
  expect(run.final_pose.x > 11.0f,
         "the Ackermann vehicle traverses the narrow corridor (final x " +
             std::to_string(run.final_pose.x) + ")");
  expect(run.min_clearance > 0.25f,
         "narrow corridor traverse keeps physical clearance (" +
             std::to_string(run.min_clearance) + " m)");
  expect(run.stop_ticks == 0,
         "the Ackermann vehicle never stops inside the corridor (" +
             std::to_string(run.stop_ticks) + " stop ticks)");
  // Without this the centering bounds below would pass on their initialisers
  // if the vehicle never reached the corridor.
  expect(run.lateral_samples > 0,
         "the centering metrics were actually sampled inside the corridor");
  expect(run.mean_abs_lateral < 0.04f,
         "the Ackermann vehicle converges onto the corridor centerline (mean |y| " +
             std::to_string(run.mean_abs_lateral) + ", max " +
             std::to_string(run.max_abs_lateral) + ")");
  expect(run.max_abs_lateral < 0.15f,
         "the Ackermann vehicle holds the centerline (max |y| " +
             std::to_string(run.max_abs_lateral) + ")");
  expect(run.curvature_sign_changes <= 12,
         "narrow corridor traverse does not oscillate (" +
             std::to_string(run.curvature_sign_changes) + " curvature sign changes)");
  // Note (R14 L7): `stop_ticks == 0` above already asserts that every tick with
  // a path commands |v| > 1e-4, and this scenario's path source is never empty,
  // so the next assertion can only fail together with that one. It is kept for
  // uniformity with the other ten scenarios, not because it adds a failure mode
  // here; the scenario that makes the no-in-place-rotation property observable
  // on its own is testRearGoal.
  expect(!run.offered_in_place_rotation, "narrow corridor traverse never turns on the spot");
  expect(!run.violated_turning_radius,
         "narrow corridor traverse respects minimum turning radius (max curvature " +
             std::to_string(run.max_abs_curvature) + ")");
}

/// Scattered obstacles that force repeated re-steering, including at the
/// turning-radius bound. Exercises the speed governor and steering continuity
/// together over a long run.
void
testClutterField()
{
  bac_sim::World world;
  world.addBox(2.5f, 0.0f, 0.3f, 0.3f);
  world.addBox(4.5f, 1.0f, 0.3f, 0.3f);
  world.addBox(4.5f, -1.2f, 0.3f, 0.3f);
  world.addBox(6.5f, 0.4f, 0.3f, 0.3f);
  world.addBox(8.0f, -0.9f, 0.3f, 0.3f);
  bac::BacCore core = makeAckermannCore();
  const AckermannRun run = runAckermann(
      core, world, { 0.0f, 0.0f, 0.0f }, bac_sim::gotoPointPath(10.0f, 0.0f),
      10.0f, 0.0f, 120.0f);

  expect(!run.collided, "clutter traverse has no body contact");
  expect(run.reached_goal,
         "the Ackermann vehicle weaves through clutter to its goal (closest " +
             std::to_string(run.min_goal_distance) + " m, final x " +
             std::to_string(run.final_pose.x) + ")");
  expect(run.min_clearance > 0.15f,
         "clutter traverse keeps physical clearance (" +
             std::to_string(run.min_clearance) + " m)");
  expect(run.stop_ticks < 20,
         "the speed governor slows for clutter without stalling (" +
             std::to_string(run.stop_ticks) + " stop ticks)");
  expect(!run.offered_in_place_rotation, "clutter traverse never turns on the spot");
  expect(!run.violated_turning_radius,
         "clutter traverse respects minimum turning radius (max curvature " +
             std::to_string(run.max_abs_curvature) + ")");
  // Deliberately no max_curvature_step bound here. Measured over benign
  // perturbations (plant curvature_rate, v_max, footprint width, sim_time) the
  // correct trajectory reaches 0.81 1/m per tick while broken variants sit at
  // 0.51 and 0.83, so no bound separates them: it would be a tripwire on one
  // trajectory, not a property of correctness. Steering continuity is bounded
  // where it is robust - the corridor and shipped-configuration runs.
}

/// `limits.w_max` is the other half of the Ackermann curvature bound
/// min(w_max, |v| / turn_radius_min). In every other fixture here, in the unit
/// fixture, and in the shipped yaml, v_max <= w_max * turn_radius_min, so the
/// radius term always wins and the w_max term is structurally inactive (R14
/// M2): deleting it from yawRateBound changed nothing anywhere. This vehicle
/// is fast and tight - v_max 1.2, turn_radius_min 0.5, w_max 0.4 - so the
/// turning circle would allow 2.4 rad/s and w_max is what actually limits the
/// commanded yaw rate.
void
testYawRateLimitBinds()
{
  bac_sim::World world;
  bac::Params params = ackermannParams(0.5f);
  params.limits.v_max = 1.2f;
  params.limits.w_max = 0.4f;
  bac::BacCore core(params);
  const AckermannRun run = runAckermann(
      core, world, { 0.0f, 0.0f, 0.0f }, bac_sim::gotoPointPath(4.0f, 3.0f),
      4.0f, 3.0f, 60.0f);

  expect(!run.collided, "the fast tight vehicle has no body contact");
  expect(run.reached_goal,
         "the fast tight vehicle reaches its lateral goal (final " +
             std::to_string(run.final_pose.x) + ", " +
             std::to_string(run.final_pose.y) + ", closest " +
             std::to_string(run.min_goal_distance) + ")");
  expect(!run.offered_in_place_rotation,
         "the fast tight vehicle never turns on the spot");
  expect(!run.violated_turning_radius,
         "the fast tight vehicle respects minimum turning radius (max curvature " +
             std::to_string(run.max_abs_curvature) + ")");
  // The bound itself, which is a contract rather than a tuned level: no
  // configuration may command more yaw rate than limits.w_max. Measured over
  // v_max 1.0-1.4 x sim_time 2.0-3.0 x plant acc_v 0.65-1.0 x footprint.width
  // 0.6/0.7 x plant curvature_rate 0.7-2.0 (360 runs, all goal-reaching,
  // contact-free, spin-free): max |w| spans 0.366667 - 0.400000 and never once
  // exceeds limits.w_max. Removing the w_max term from yawRateBound spans
  // 0.359792 - 0.527083 over the same grid and breaks the limit in 240 of the
  // 360 runs; at this scenario's own parameters it is 0.421667 - 0.440000
  // against exactly 0.400000 here.
  expect(run.max_abs_yaw_rate <= params.limits.w_max + 1e-4f,
         "the commanded yaw rate never exceeds limits.w_max (max |w| " +
             std::to_string(run.max_abs_yaw_rate) + " vs " +
             std::to_string(params.limits.w_max) + " rad/s)");
  // ... and the structural witness that keeps the bound above from being
  // vacuous: ticks where |v| / turn_radius_min exceeds w_max are ticks where
  // w_max is the term that limits the yaw rate. Measured 127-157 of 1200 over
  // the same 360-run grid (0 in every pre-R14 fixture and in the shipped yaml,
  // which is what made the term untested). A count, not a level: the assertion
  // is > 0 against a measured minimum of 127.
  expect(run.yaw_bound_ticks > 0,
         "limits.w_max is the binding term on at least one tick, so the bound "
         "above is not vacuous (" + std::to_string(run.yaw_bound_ticks) +
             " of 1200 ticks)");
}

/// The tightness probe must reach at least `min_eval_distance` regardless of
/// candidate speed: `probe_dist = max(v_probe * sim_time, min_eval_distance)`
/// in BacCore. Dropping that floor makes the controller myopic at low speed -
/// the source comment says it "creeps into them until the emergency stop" -
/// and until R13 M1 removed the clutter `max_curvature_step` bound, that bound
/// was the only thing killing it (R14 M5).
///
/// This is a single deterministic tick rather than another closed-loop
/// threshold: the gate sits in the annulus that the floor reaches and a purely
/// time-based horizon does not (v_probe * sim_time = 0.4 * 2.5 = 1.0 m,
/// min_eval_distance = 1.6 m, gate at 1.7 m), and the reference is the same
/// tick in an empty world rather than an absolute speed.
void
testClearanceProbeReach()
{
  const bac::Params params = ackermannParams();
  std::vector<bac::Point2D> path;
  for (int i = 1; i <= 40; ++i)
  {
    path.push_back({ 0.25f * static_cast<float>(i), 0.0f });
  }
  const bac_sim::Pose origin{ 0.0f, 0.0f, 0.0f };
  const bac::Twist2D current(0.2f, 0.0f);

  bac_sim::World gated;
  gated.addWall(1.7f, 0.6f, 2.0f, 0.6f);
  gated.addWall(1.7f, -0.6f, 2.0f, -0.6f);
  bac::BacCore gated_core(params);
  const bac::Result gated_result = gated_core.process(
      bac_sim::simulateLidar(gated, origin, 720, params.max_range), path, current);

  bac_sim::World open;
  bac::BacCore open_core(params);
  const bac::Result open_result = open_core.process(
      bac_sim::simulateLidar(open, origin, 720, params.max_range), path, current);

  expect(open_result.output.v > params.limits.v_max - 1e-4f,
         "the open-field reference tick cruises at limits.v_max (" +
             std::to_string(open_result.output.v) + " m/s)");
  // Measured as gated / open commanded speed over gate distance 1.7/1.8 m x
  // half-gap 0.50/0.55/0.60/0.65/0.70/0.80 m x current speed 0.2/0.3/0.4 m/s
  // (36 cases per side):
  //   correct controller                   0.801 - 0.859
  //   probe horizon without the floor      1.000 in all 36 (the gate is
  //                                        invisible; full cruise is commanded)
  // 0.95 separates the two measured bands with margin on both sides.
  expect(gated_result.output.v < 0.95f * open_result.output.v,
         "a gate 1.7 m ahead - beyond the speed-scaled horizon but inside "
         "min_eval_distance - still moderates the cruise (gated " +
             std::to_string(gated_result.output.v) + " vs open " +
             std::to_string(open_result.output.v) + " m/s)");
}

/// Keys config/bac_controller_ackermann.yaml may carry without this scenario
/// consuming them. Deliberately small and deliberately explicit: three block
/// headers plus the Nav2- and adapter-level settings that have no effect on the
/// closed-loop geometry measured here. Everything else in the file must be read
/// by shippedExampleParams(), so a new parameter cannot enter the shipped
/// configuration without either being exercised by this suite or being added to
/// this list on purpose. See shipped_config.hpp for what the guard enforces.
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
  bac::Params params;
  bac_sim::ShippedConfigSpec spec;
  spec.path       = BAC_ACKERMANN_CONFIG_PATH;
  spec.label      = "Ackermann";
  spec.model_name = "ackermann";
  spec.allowed    = kAllowedUnconsumedKeys;
  spec.bindings   = {
      bac_sim::bindNumber("turn_radius_min", params.turn_radius_min),
      bac_sim::bindNumber("footprint.front", params.footprint.front),
      bac_sim::bindNumber("footprint.rear", params.footprint.rear),
      bac_sim::bindNumber("footprint.width", params.footprint.width),
      bac_sim::bindNumber("safety_margin.front", params.safety_margin.front),
      bac_sim::bindNumber("safety_margin.rear", params.safety_margin.rear),
      bac_sim::bindNumber("safety_margin.side", params.safety_margin.side),
      bac_sim::bindNumber("avoid_margin.side", params.avoid_margin.side),
      bac_sim::bindNumber("limits.v_max", params.limits.v_max),
      bac_sim::bindNumber("limits.v_min", params.limits.v_min),
      bac_sim::bindNumber("limits.w_max", params.limits.w_max),
      bac_sim::bindNumber("limits.acc_v", params.limits.acc_v),
      bac_sim::bindNumber("limits.acc_w", params.limits.acc_w),
      bac_sim::bindNumber("control_period", params.control_period),
      bac_sim::bindNumber("stop_decel", params.stop_decel),
      bac_sim::bindNumber("brake_reaction_time", params.brake_reaction_time),
      bac_sim::bindNumber("weights.clearance", params.weights.clearance),
      bac_sim::bindNumber("weights.path_dist", params.weights.path_dist),
      bac_sim::bindNumber("weights.balance", params.weights.balance),
      bac_sim::bindNumber("weights.heading", params.weights.heading),
      bac_sim::bindNumber("weights.hysteresis", params.weights.hysteresis),
      bac_sim::bindNumber("weights.squeeze", params.weights.squeeze),
      bac_sim::bindNumber("sim_time", params.sim_time),
  };

  const bac_sim::ShippedConfigVerdict verdict = bac_sim::checkShippedConfig(spec, expect);
  if (!verdict.readable)
  {
    return params;
  }
  params.motion_model.type = bac::MotionModelType::ACKERMANN;
  g_shipped_config_loaded = verdict.complete;
  return params;
}

/// Runs the shipped example end to end. The Weights default (hysteresis 0.6)
/// is tuned for differential drive; under Ackermann the curvature penalty does
/// not shrink with speed and the vehicle orbits instead of reaching the goal,
/// so a suite that quietly used a different weight would not defend the file
/// users install.
void
testShippedExampleConfiguration()
{
  bac_sim::World world;
  world.addBox(4.0f, 0.0f, 1.0f, 1.0f);
  bac::BacCore core(shippedExampleParams());
  expect(g_shipped_config_loaded,
         "every value the scenario needs was read from the shipped configuration");
  const AckermannRun run = runAckermann(
      core, world, { 0.0f, 0.0f, 0.0f }, bac_sim::gotoPointPath(9.0f, 0.0f),
      9.0f, 0.0f, 90.0f);

  expect(!run.collided, "the shipped Ackermann example has no body contact");
  // Deliberately no min_clearance bound here either (R14 M3/M9). Same world,
  // same 288-run grid, this time on the shipped geometry and weights:
  //   correct controller                        0.175166 - 0.478241 m
  //   speed governor ignoring the caller's cap  0.117090 - 0.478012 m
  // The bands overlap, and the 0.20 m bound R13 M2 added here fails on benign
  // yaml edits the file itself invites (`limits.v_max: 1.4` -> 0.171 m), which
  // is exactly the contradiction R14 M9 records. What defends this file is the
  // guard above plus the goal/contact/turning-circle assertions below.
  expect(run.reached_goal,
         "the shipped Ackermann example reaches its goal rather than orbiting "
         "(closest " + std::to_string(run.min_goal_distance) + " m, travelled " +
             std::to_string(run.travelled_distance) + " m)");
  expect(!run.offered_in_place_rotation,
         "the shipped Ackermann example never turns on the spot");
  expect(!run.violated_turning_radius,
         "the shipped Ackermann example respects its own turning circle");
  expect(run.max_curvature_step < 1.0f / core.params().turn_radius_min,
         "the shipped Ackermann example steers smoothly (" +
             std::to_string(run.max_curvature_step) + " 1/m per tick)");
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
  // R14 L7: the eleventh scenario was the only one without this assertion.
  expect(!tight_run.offered_in_place_rotation && !wide_run.offered_in_place_rotation,
         "neither vehicle turns on the spot to reach the lateral goal");
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
  testYawRateLimitBinds();
  testClearanceProbeReach();
  testShippedExampleConfiguration();
  testSafetyStopHoldsPosition();
  testSafetyStopReverseEscape();
  testNarrowCorridorCentering();
  testClutterField();

  if (failures != 0)
  {
    std::cerr << failures << " Ackermann scenario check(s) failed\n";
    return 1;
  }
  std::cout << "All Ackermann closed-loop scenarios passed\n";
  return 0;
}
