/**
 * @file scenarios.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Scenario-based verification harness for the Bilateral Arc Clearance (BAC) algorithm
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Runs bac::BacCore in closed loop against synthetic worlds and
 * evaluates trajectory quality. Two tiers:
 *  - REGRESSION: must pass with the current algorithm; guards against breakage.
 *  - TARGET:     express the improvement goal ("avoid when there is room,
 *    smoothly enter narrow corridors when there is not"); allowed to fail
 *    until the algorithm is improved, reported but not fatal unless --strict.
 *
 * Usage: bac_scenario_harness [--strict] [--csv-dir DIR] [--filter SUBSTRING]
 */

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "sim_world.hpp"
#include "sim_runner.hpp"
#include "metrics.hpp"

using namespace bac_sim;

namespace
{

enum class Tier
{
  REGRESSION,
  TARGET
};

struct Check
{
  std::string name;
  bool        pass;
  std::string detail;
};

struct ScenarioResult
{
  std::string        name;
  Tier               tier;
  std::vector<Check> checks;
  Metrics            metrics;

  bool passed() const
  {
    for (const Check &c : checks)
    {
      if (!c.pass) return false;
    }
    return true;
  }
};

// Weight overrides from the command line (for parameter sweeps)
struct WeightOverrides
{
  float clearance  = -1.0f;
  float path_dist  = -1.0f;
  float balance    = -1.0f;
  float hysteresis = -1.0f;
};
WeightOverrides g_weight_overrides;

bac::BacCore
makeCore(float v_max = 0.4f)
{
  // bac::Params defaults; apply CLI overrides
  bac::Params params;
  params.limits.v_max = v_max;
  if (g_weight_overrides.clearance >= 0.0f) params.weights.clearance = g_weight_overrides.clearance;
  if (g_weight_overrides.path_dist >= 0.0f) params.weights.path_dist = g_weight_overrides.path_dist;
  if (g_weight_overrides.balance >= 0.0f) params.weights.balance = g_weight_overrides.balance;
  if (g_weight_overrides.hysteresis >= 0.0f) params.weights.hysteresis = g_weight_overrides.hysteresis;
  return bac::BacCore(params);
}

void
writeTraceCsv(const std::string &dir, const std::string &name, const SimResult &result, const World &world)
{
  std::filesystem::create_directories(dir);

  std::ofstream trace_file(dir + "/" + name + ".csv");
  trace_file << "t,x,y,th,cmd_v,cmd_w,out_v,out_w,act_v,act_w,status,clearance,speed_fraction,cmd_clearance\n";
  for (const TraceRow &row : result.trace)
  {
    trace_file << row.t << ',' << row.pose.x << ',' << row.pose.y << ',' << row.pose.th << ',' << row.command.v << ','
               << row.command.w << ',' << row.output.v << ',' << row.output.w << ',' << row.actual.v << ','
               << row.actual.w << ',' << row.status << ',' << row.clearance << ',' << row.speed_fraction << ','
               << row.command_clearance << '\n';
  }

  std::ofstream world_file(dir + "/" + name + "_world.csv");
  world_file << "x1,y1,x2,y2\n";
  for (const Segment &seg : world.walls)
  {
    world_file << seg.x1 << ',' << seg.y1 << ',' << seg.x2 << ',' << seg.y2 << '\n';
  }
}

std::string
fmt(float v)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3) << v;
  return oss.str();
}

// ---- individual scenarios ----

// Open space, no obstacles: the avoid layer must be fully transparent.
ScenarioResult
scenarioOpenPassthrough(const std::string &csv_dir)
{
  ScenarioResult result{ "open_passthrough", Tier::REGRESSION, {}, {} };

  World world;  // empty
  bac::BacCore core = makeCore(0.3f);
  SimConfig config;
  config.sim_time = 10.0f;

  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.0f, 0.0f }, gotoPointPath(10.0f, 0.0f), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  result.metrics = computeMetrics(sim, options);
  const Metrics &m = result.metrics;

  int clear_ticks = m.total_ticks - m.status_stop - m.status_avoid;
  result.checks.push_back({ "status always CLEAR", clear_ticks == m.total_ticks,
                            "CLEAR " + std::to_string(clear_ticks) + "/" +
                                std::to_string(m.total_ticks) });
  result.checks.push_back({ "no stop ticks", m.stop_ticks == 0, std::to_string(m.stop_ticks) + " stop ticks" });
  result.checks.push_back({ "travelled straight", m.final_x > 2.7f && std::fabs(m.final_y) < 0.01f,
                            "final (" + fmt(m.final_x) + ", " + fmt(m.final_y) + ")" });
  return result;
}

// Obstacle inside the safety polygon: must stop immediately and stay stopped.
ScenarioResult
scenarioSafetyStop(const std::string &csv_dir)
{
  ScenarioResult result{ "safety_stop", Tier::REGRESSION, {}, {} };

  World world;
  world.addWall(0.65f, -1.0f, 0.65f, 1.0f);  // inside safety polygon (front edge at 0.7 when stationary)
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 3.0f;

  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.0f, 0.0f }, gotoPointPath(10.0f, 0.0f), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  result.metrics = computeMetrics(sim, options);
  const Metrics &m = result.metrics;

  // With the escape-reverse row, a wall dead ahead makes the robot back off
  // to a safe distance and hold (instead of freezing in place). The safety
  // contract is: never advance towards the obstacle, never collide, settle.
  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "never advances", m.final_x < 0.01f, "final x " + fmt(m.final_x) });
  result.checks.push_back({ "retreats no further than needed", m.final_x > -0.6f,
                            "final x " + fmt(m.final_x) });
  return result;
}

// Single small obstacle ahead with plenty of room: should divert around it and keep going.
ScenarioResult
scenarioAvoidSingleObstacle(const std::string &csv_dir)
{
  ScenarioResult result{ "avoid_single_obstacle", Tier::TARGET, {}, {} };

  World world;
  world.addBox(4.0f, 0.0f, 0.2f, 0.2f);
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 60.0f;

  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.0f, 0.0f }, gotoPointPath(9.0f, 0.0f), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 9.0f;
  options.goal_y         = 0.0f;
  options.goal_tolerance = 0.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "passes the obstacle", m.final_x > 6.0f, "final x " + fmt(m.final_x) });
  result.checks.push_back({ "reaches goal", m.time_to_goal >= 0.0f,
                            m.time_to_goal >= 0.0f ? "t=" + fmt(m.time_to_goal) + "s" : "not reached" });
  result.checks.push_back({ "no full stop", m.stop_ticks < 20, std::to_string(m.stop_ticks) + " stop ticks" });
  return result;
}

// Wide corridor (plenty of side clearance): should pass through without drama.
ScenarioResult
scenarioCorridorWide(const std::string &csv_dir)
{
  ScenarioResult result{ "corridor_wide", Tier::TARGET, {}, {} };

  World world;
  world.addCorridorX(3.0f, 9.0f, 0.0f, 2.5f);
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 60.0f;

  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.0f, 0.0f }, gotoPointPath(10.0f, 0.0f), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 10.0f;
  options.goal_y         = 0.0f;
  options.goal_tolerance = 0.5f;
  options.eval_lateral   = true;
  options.center_y       = 0.0f;
  options.x_from         = 3.5f;
  options.x_to           = 8.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "traverses corridor", m.final_x > 9.0f, "final x " + fmt(m.final_x) });
  result.checks.push_back({ "no full stop", m.stop_ticks == 0, std::to_string(m.stop_ticks) + " stop ticks" });
  result.checks.push_back({ "stays near center", m.mean_abs_lateral < 0.2f,
                            "mean |y| " + fmt(m.mean_abs_lateral) + ", max " + fmt(m.max_abs_lateral) });
  return result;
}

// Narrow corridor, aligned entry. Walls are inside the avoid margin on both
// sides, but there is physical room: the robot should drive straight through
// at the commanded speed instead of stopping or oscillating.
ScenarioResult
scenarioCorridorNarrowAligned(const std::string &csv_dir)
{
  ScenarioResult result{ "corridor_narrow_aligned", Tier::TARGET, {}, {} };

  // Robot width 0.95 + safety side 0.2*2 = 1.35 hard-stop width.
  // Corridor 1.7 leaves ~0.175m true clearance per side but is fully inside
  // the avoid polygon width (0.95 + 0.3*2 = 1.55).
  World world;
  world.addCorridorX(3.0f, 9.0f, 0.0f, 1.7f);
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 90.0f;

  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.0f, 0.0f }, gotoPointPath(10.0f, 0.0f), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 10.0f;
  options.goal_y         = 0.0f;
  options.goal_tolerance = 0.5f;
  options.eval_lateral   = true;
  options.center_y       = 0.0f;
  options.x_from         = 3.5f;
  options.x_to           = 8.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "traverses corridor", m.final_x > 9.0f, "final x " + fmt(m.final_x) });
  result.checks.push_back({ "no full stop", m.stop_ticks == 0, std::to_string(m.stop_ticks) + " stop ticks" });
  result.checks.push_back({ "stays near center", m.mean_abs_lateral < 0.10f,
                            "mean |y| " + fmt(m.mean_abs_lateral) + ", max " + fmt(m.max_abs_lateral) });
  result.checks.push_back({ "low oscillation", m.w_sign_changes <= 8,
                            std::to_string(m.w_sign_changes) + " w sign changes" });
  return result;
}

// Narrow corridor, laterally offset approach. The improvement goal: funnel
// smoothly into the corridor center following the upper-level command, even
// though the avoid margins touch both walls.
ScenarioResult
scenarioCorridorNarrowOffset(const std::string &csv_dir)
{
  ScenarioResult result{ "corridor_narrow_offset", Tier::TARGET, {}, {} };

  World world;
  world.addCorridorX(3.0f, 9.0f, 0.0f, 1.7f);
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 90.0f;

  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.4f, 0.0f }, gotoPointPath(10.0f, 0.0f), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 10.0f;
  options.goal_y         = 0.0f;
  options.goal_tolerance = 0.5f;
  options.eval_lateral   = true;
  options.center_y       = 0.0f;
  options.x_from         = 4.0f;
  options.x_to           = 8.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "traverses corridor", m.final_x > 9.0f, "final x " + fmt(m.final_x) });
  result.checks.push_back({ "no full stop", m.stop_ticks == 0, std::to_string(m.stop_ticks) + " stop ticks" });
  result.checks.push_back({ "converges to center", m.mean_abs_lateral < 0.15f,
                            "mean |y| " + fmt(m.mean_abs_lateral) + ", max " + fmt(m.max_abs_lateral) });
  result.checks.push_back({ "low oscillation", m.w_sign_changes <= 10,
                            std::to_string(m.w_sign_changes) + " w sign changes" });
  return result;
}

// Narrow corridor whose mouth is the only opening in a long wall: detouring
// around is impossible, the robot MUST funnel into the corridor. This is the
// core case of the improvement goal.
ScenarioResult
scenarioCorridorNarrowWalled(const std::string &csv_dir)
{
  ScenarioResult result{ "corridor_narrow_walled", Tier::TARGET, {}, {} };

  World world;
  world.addCorridorX(3.0f, 9.0f, 0.0f, 1.7f);
  world.addWall(3.0f, 0.85f, 3.0f, 6.0f);    // wall above the corridor mouth
  world.addWall(3.0f, -0.85f, 3.0f, -6.0f);  // wall below the corridor mouth
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 120.0f;

  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.4f, 0.0f }, gotoPointPath(10.0f, 0.0f), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 10.0f;
  options.goal_y         = 0.0f;
  options.goal_tolerance = 0.5f;
  options.eval_lateral   = true;
  options.center_y       = 0.0f;
  options.x_from         = 4.0f;
  options.x_to           = 8.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "enters and traverses corridor", m.final_x > 9.0f && m.lateral_samples > 0,
                            "final (" + fmt(m.final_x) + ", " + fmt(m.final_y) + ")" });
  result.checks.push_back({ "no full stop", m.stop_ticks < 20, std::to_string(m.stop_ticks) + " stop ticks" });
  result.checks.push_back({ "converges to center", m.lateral_samples > 0 && m.mean_abs_lateral < 0.15f,
                            "mean |y| " + fmt(m.mean_abs_lateral) + ", max " + fmt(m.max_abs_lateral) });
  result.checks.push_back({ "low oscillation", m.w_sign_changes <= 10,
                            std::to_string(m.w_sign_changes) + " w sign changes" });
  return result;
}

// Physical-limit corridor: 1.5 m leaves only 0.275 m of slack per side -
// close to the 0.2 m safety margin. Requires geometry-driven centering (the
// balance term): the residual drift that 1.7 m tolerates is fatal here.
ScenarioResult
scenarioCorridorExtreme(const std::string &csv_dir)
{
  ScenarioResult result{ "corridor_extreme", Tier::TARGET, {}, {} };

  World world;
  world.addCorridorX(3.0f, 9.0f, 0.0f, 1.5f);
  world.addWall(3.0f, 0.75f, 3.0f, 6.0f);
  world.addWall(3.0f, -0.75f, 3.0f, -6.0f);
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 120.0f;

  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.4f, 0.0f }, gotoPointPath(10.0f, 0.0f), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 10.0f;
  options.goal_y         = 0.0f;
  options.goal_tolerance = 0.5f;
  options.eval_lateral   = true;
  options.center_y       = 0.0f;
  options.x_from         = 4.0f;
  options.x_to           = 8.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "enters and traverses corridor", m.final_x > 9.0f,
                            "final (" + fmt(m.final_x) + ", " + fmt(m.final_y) + ")" });
  result.checks.push_back({ "no full stop", m.stop_ticks < 40, std::to_string(m.stop_ticks) + " stop ticks" });
  result.checks.push_back({ "converges to center", m.lateral_samples > 0 && m.mean_abs_lateral < 0.10f,
                            "mean |y| " + fmt(m.mean_abs_lateral) + ", max " + fmt(m.max_abs_lateral) });
  return result;
}

// L-shaped entry: a wide approach leg ends in a 90-degree turn into a narrow
// (1.7 m) corridor. Tests cornering into a passage whose entry requires a
// swing, with a planner-like waypoint path around the corner.
ScenarioResult
scenarioCorridorLshape(const std::string &csv_dir)
{
  ScenarioResult result{ "corridor_lshape", Tier::TARGET, {}, {} };

  World world;
  world.addWall(-1.0f, -1.25f, 5.0f, -1.25f);   // approach, bottom
  world.addWall(-1.0f, 1.25f, 2.8f, 1.25f);     // approach, top (until the branch)
  world.addWall(4.5f, 1.25f, 5.0f, 1.25f);      // top, right of the branch
  world.addWall(5.0f, -1.25f, 5.0f, 1.25f);     // dead end ahead
  world.addWall(2.8f, 1.25f, 2.8f, 7.0f);       // narrow corridor, left
  world.addWall(4.5f, 1.25f, 4.5f, 7.0f);       // narrow corridor, right
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 120.0f;

  std::vector<Point2D> waypoints{ { 0.0f, 0.0f }, { 3.65f, 0.0f }, { 3.65f, 6.0f } };
  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.0f, 0.0f }, waypointsPath(waypoints), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 3.65f;
  options.goal_y         = 6.0f;
  options.goal_tolerance = 0.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "turns into and traverses corridor", m.final_y > 5.0f,
                            "final (" + fmt(m.final_x) + ", " + fmt(m.final_y) + ")" });
  result.checks.push_back({ "no full stop", m.stop_ticks < 40, std::to_string(m.stop_ticks) + " stop ticks" });
  return result;
}

// Z path (DWPP-style verification shape): two opposite 90-degree corners in
// close succession, 1.7 m legs, 2.3 m between corner centers. A single
// carrot point cannot represent the double corner - the path SHAPE must
// steer the entry and exit.
ScenarioResult
scenarioCorridorZigzag(const std::string &csv_dir)
{
  ScenarioResult result{ "corridor_zigzag", Tier::TARGET, {}, {} };

  World world;
  world.addWall(-1.0f, -0.85f, 3.0f, -0.85f);  // leg1 bottom
  world.addWall(-1.0f, 0.85f, 1.3f, 0.85f);    // leg1 top (until the upturn)
  world.addWall(1.3f, 0.85f, 1.3f, 3.15f);     // leg2 left
  world.addWall(3.0f, -0.85f, 3.0f, 1.45f);    // leg2 right (dead end of leg1)
  world.addWall(3.0f, 1.45f, 8.0f, 1.45f);     // leg3 bottom
  world.addWall(1.3f, 3.15f, 8.0f, 3.15f);     // leg3 top
  world.addWall(8.0f, 1.45f, 8.0f, 3.15f);     // end cap behind the goal
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 120.0f;

  std::vector<Point2D> waypoints{
    { 0.0f, 0.0f }, { 2.15f, 0.0f }, { 2.15f, 2.3f }, { 7.0f, 2.3f }
  };
  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.0f, 0.0f }, waypointsPath(waypoints), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 7.0f;
  options.goal_y         = 2.3f;
  options.goal_tolerance = 0.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "traverses both corners", m.final_x > 6.0f && m.final_y > 1.8f,
                            "final (" + fmt(m.final_x) + ", " + fmt(m.final_y) + ")" });
  result.checks.push_back({ "keeps clearance", m.min_clearance > 0.1f,
                            "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "no full stop", m.stop_ticks < 40, std::to_string(m.stop_ticks) + " stop ticks" });
  return result;
}

// REGRESSION guard for the localization-robustness selling point: the
// waypoint path rides 0.15 m off the true corridor center (as after
// localization drift with a correct map). Geometric centering must ignore
// the path's lateral error and ride the TRUE center.
ScenarioResult
scenarioPathOffsetNarrow(const std::string &csv_dir)
{
  ScenarioResult result{ "path_offset_narrow", Tier::REGRESSION, {}, {} };

  World world;
  world.addCorridorX(3.0f, 9.0f, 0.0f, 1.7f);
  world.addWall(3.0f, 0.85f, 3.0f, 6.0f);
  world.addWall(3.0f, -0.85f, 3.0f, -6.0f);
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 120.0f;

  std::vector<Point2D> waypoints{ { 0.0f, 0.15f }, { 10.0f, 0.15f } };
  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.0f, 0.0f }, waypointsPath(waypoints), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 10.0f;
  options.goal_y         = 0.15f;
  options.goal_tolerance = 0.5f;
  options.eval_lateral   = true;
  options.center_y       = 0.0f;
  options.x_from         = 4.0f;
  options.x_to           = 8.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "traverses corridor", m.final_x > 9.0f, "final x " + fmt(m.final_x) });
  result.checks.push_back({ "rides the TRUE center, not the offset path",
                            m.lateral_samples > 0 && m.mean_abs_lateral < 0.08f,
                            "mean |y| " + fmt(m.mean_abs_lateral) + ", max " + fmt(m.max_abs_lateral) });
  result.checks.push_back({ "keeps clearance", m.min_clearance > 0.2f,
                            "min clearance " + fmt(m.min_clearance) });
  return result;
}

// REGRESSION guard for the degraded-plan selling point: an obstacle sits ON
// the path (the planner does not know about it) and the robot must swerve
// around it while the path keeps pointing through it.
ScenarioResult
scenarioBlockedPathObstacle(const std::string &csv_dir)
{
  ScenarioResult result{ "blocked_path_obstacle", Tier::REGRESSION, {}, {} };

  World world;
  world.addBox(4.5f, 0.0f, 0.4f, 0.4f);
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 90.0f;

  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.0f, 0.0f }, gotoPointPath(9.0f, 0.0f), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 9.0f;
  options.goal_y         = 0.0f;
  options.goal_tolerance = 0.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "reaches goal", m.time_to_goal >= 0.0f,
                            m.time_to_goal >= 0.0f ? "t=" + fmt(m.time_to_goal) + "s" : "not reached" });
  result.checks.push_back({ "keeps clearance", m.min_clearance > 0.15f,
                            "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "no full stop", m.stop_ticks < 40, std::to_string(m.stop_ticks) + " stop ticks" });
  return result;
}

// Dense clutter: five unknown boxes with ~2 m spacing - an environment where
// the configured avoid margin (0.6 m from everything) is unattainable. The
// density adaptation must keep this passable instead of treating the whole
// field as a crisis.
ScenarioResult
scenarioClutterField(const std::string &csv_dir)
{
  ScenarioResult result{ "clutter_field", Tier::TARGET, {}, {} };

  World world;
  world.addBox(2.5f, 0.0f, 0.3f, 0.3f);
  world.addBox(4.5f, 1.0f, 0.3f, 0.3f);
  world.addBox(4.5f, -1.2f, 0.3f, 0.3f);
  world.addBox(6.5f, 0.4f, 0.3f, 0.3f);
  world.addBox(8.0f, -0.9f, 0.3f, 0.3f);
  bac::BacCore core = makeCore();
  SimConfig config;
  config.sim_time = 90.0f;

  SimResult sim = runClosedLoop(core, world, { 0.0f, 0.0f, 0.0f }, gotoPointPath(9.0f, 0.0f), config);
  writeTraceCsv(csv_dir, result.name, sim, world);

  MetricsOptions options;
  options.goal_x         = 9.0f;
  options.goal_y         = 0.0f;
  options.goal_tolerance = 0.5f;
  result.metrics         = computeMetrics(sim, options);
  const Metrics &m       = result.metrics;

  result.checks.push_back({ "no collision", !m.collided, "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "reaches goal", m.time_to_goal >= 0.0f,
                            m.time_to_goal >= 0.0f ? "t=" + fmt(m.time_to_goal) + "s" : "not reached" });
  result.checks.push_back({ "keeps clearance", m.min_clearance > 0.1f,
                            "min clearance " + fmt(m.min_clearance) });
  result.checks.push_back({ "no full stop", m.stop_ticks < 40, std::to_string(m.stop_ticks) + " stop ticks" });
  return result;
}

}  // namespace

int
main(int argc, char *argv[])
{
  bool        strict  = false;
  std::string csv_dir = "traces";
  std::string filter;

  for (int i = 1; i < argc; i++)
  {
    if (std::strcmp(argv[i], "--strict") == 0)
    {
      strict = true;
    }
    else if (std::strcmp(argv[i], "--csv-dir") == 0 && i + 1 < argc)
    {
      csv_dir = argv[++i];
    }
    else if (std::strcmp(argv[i], "--filter") == 0 && i + 1 < argc)
    {
      filter = argv[++i];
    }
    else if (std::strcmp(argv[i], "--w-clearance") == 0 && i + 1 < argc)
    {
      g_weight_overrides.clearance = std::atof(argv[++i]);
    }
    else if (std::strcmp(argv[i], "--w-path-dist") == 0 && i + 1 < argc)
    {
      g_weight_overrides.path_dist = std::atof(argv[++i]);
    }
    else if (std::strcmp(argv[i], "--w-balance") == 0 && i + 1 < argc)
    {
      g_weight_overrides.balance = std::atof(argv[++i]);
    }
    else if (std::strcmp(argv[i], "--w-hysteresis") == 0 && i + 1 < argc)
    {
      g_weight_overrides.hysteresis = std::atof(argv[++i]);
    }
    else
    {
      std::cerr << "Usage: " << argv[0]
                << " [--strict] [--csv-dir DIR] [--filter SUBSTRING]"
                << " [--w-clearance X] [--w-path-dist X] [--w-hysteresis X]" << std::endl;
      return 2;
    }
  }

  using ScenarioFunc = ScenarioResult (*)(const std::string &);
  const ScenarioFunc scenarios[] = {
    scenarioOpenPassthrough, scenarioSafetyStop,           scenarioAvoidSingleObstacle,
    scenarioCorridorWide,    scenarioCorridorNarrowAligned, scenarioCorridorNarrowOffset,
    scenarioCorridorNarrowWalled, scenarioCorridorExtreme, scenarioCorridorLshape,
    scenarioCorridorZigzag,
    scenarioPathOffsetNarrow, scenarioBlockedPathObstacle, scenarioClutterField,
  };

  std::vector<ScenarioResult> results;
  for (ScenarioFunc scenario : scenarios)
  {
    ScenarioResult result = scenario(csv_dir);
    if (!filter.empty() && result.name.find(filter) == std::string::npos)
    {
      continue;
    }
    results.push_back(std::move(result));
  }

  int regression_failures = 0;
  int target_failures     = 0;

  std::cout << "\n==== BAC scenario harness ====\n" << std::endl;
  for (const ScenarioResult &result : results)
  {
    const char *tier_name = (result.tier == Tier::REGRESSION) ? "REGRESSION" : "TARGET";
    bool        pass      = result.passed();
    std::cout << (pass ? "[PASS] " : "[FAIL] ") << result.name << " (" << tier_name << ")" << std::endl;
    for (const Check &check : result.checks)
    {
      std::cout << "    " << (check.pass ? "ok   " : "NG   ") << check.name << " : " << check.detail << std::endl;
    }
    const Metrics &m = result.metrics;
    std::cout << "    ---- ticks " << m.total_ticks << " (stop " << m.status_stop << " / avoid " << m.status_avoid
              << "), min clearance " << fmt(m.min_clearance) << ", max dw " << fmt(m.max_dw) << std::endl;
    if (!pass)
    {
      if (result.tier == Tier::REGRESSION)
      {
        regression_failures++;
      }
      else
      {
        target_failures++;
      }
    }
    std::cout << std::endl;
  }

  std::cout << "Summary: " << results.size() << " scenarios, " << regression_failures << " regression failure(s), "
            << target_failures << " target-not-met" << std::endl;
  std::cout << "Traces written to '" << csv_dir << "/' (plot with plot_traces.py)" << std::endl;

  if (regression_failures > 0)
  {
    return 1;
  }
  if (strict && target_failures > 0)
  {
    return 1;
  }
  return 0;
}
