/**
 * @file sim_runner.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Closed-loop simulation runner for the BAC scenario harness
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Replicates the integration loop the core is designed for: a local path in
 * the robot frame (recomputed every tick, like a 1 Hz-replanned nav2 plan
 * followed at 20 Hz), the core as the whole local planner, and a unicycle
 * plant with an accel-limited actuator and a simulated LiDAR.
 */

#pragma once
#ifndef BAC_SIM_SIM_RUNNER_HPP__
#define BAC_SIM_SIM_RUNNER_HPP__

#include <cmath>
#include <functional>
#include <limits>
#include <vector>
#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "sim_world.hpp"

namespace bac_sim
{

using bac::Point2D;
using bac::Status;
using bac::Twist2D;

struct SimConfig
{
  float dt              = 0.05f;  // 20Hz control loop
  float sim_time        = 30.0f;
  float acc_v           = 0.8f;  // actuator tracking accel limit [m/s^2]
  float acc_w           = 2.5f;  // actuator tracking accel limit [rad/s^2]
  int   lidar_beams     = 720;
  float lidar_max_range = 10.0f;
  bool  has_goal        = false;  // optional fixed world-frame marker for traces
  float goal_x          = 0.0f;
  float goal_y          = 0.0f;
};

struct TraceRow
{
  float   t;
  Pose    pose;
  Twist2D command;  // nominal upper intent (v_max while a path exists) for stall metrics
  Twist2D output;   // core output
  Twist2D actual;   // actuator state applied to kinematics
  int     status;   // Status returned this tick
  float   clearance;
  float   speed_fraction;     // admissible candidate fraction (debug)
  float   command_clearance;  // bilateral clearance of the selected arc (debug)
  float   goal_x;              // fixed world-frame evaluation goal, NaN if unspecified
  float   goal_y;
};

/// Local path source: (pose, time) -> path in the ROBOT frame, near-to-far.
/// Empty path = no intent (goal reached).
using PathSource = std::function<std::vector<Point2D>(const Pose &, float)>;

struct SimResult
{
  std::vector<TraceRow> trace;
  bool                  collided = false;
};

inline float
clampf(float v, float lo, float hi)
{
  return std::max(lo, std::min(hi, v));
}

inline SimResult
runClosedLoop(bac::BacCore &core, const World &world, const Pose &start,
              const PathSource &path_source, const SimConfig &config)
{
  SimResult result;
  Pose      pose = start;
  Twist2D   actual(0.0f, 0.0f);

  const bac::Footprint &footprint = core.params().footprint;
  int num_steps = static_cast<int>(config.sim_time / config.dt);
  result.trace.reserve(num_steps);

  for (int step = 0; step < num_steps; step++)
  {
    float t = step * config.dt;

    std::vector<Point2D> points = simulateLidar(world, pose, config.lidar_beams, config.lidar_max_range);
    std::vector<Point2D> path   = path_source(pose, t);

    bac::Result avoid   = core.process(points, path, actual);
    Twist2D     applied = avoid.output;

    // --- actuator model: accel-limited tracking ---
    actual.v += clampf(applied.v - actual.v, -config.acc_v * config.dt, config.acc_v * config.dt);
    actual.w += clampf(applied.w - actual.w, -config.acc_w * config.dt, config.acc_w * config.dt);

    // --- unicycle kinematics (midpoint integration) ---
    float th_mid = pose.th + actual.w * config.dt / 2.0f;
    pose.x += actual.v * std::cos(th_mid) * config.dt;
    pose.y += actual.v * std::sin(th_mid) * config.dt;
    pose.th += actual.w * config.dt;

    float clearance = robotClearance(pose, footprint, world);

    TraceRow row;
    row.t              = t;
    row.pose           = pose;
    row.command        = Twist2D(path.empty() ? 0.0f : core.params().limits.v_max, 0.0f);
    row.output         = applied;
    row.actual         = actual;
    row.status         = static_cast<int>(avoid.status);
    row.clearance      = clearance;
    row.speed_fraction = (avoid.candidate_count > 0)
                             ? static_cast<float>(avoid.admissible_count) / avoid.candidate_count
                             : 1.0f;
    row.command_clearance = avoid.best_clearance;
    row.goal_x = config.has_goal ? config.goal_x : std::numeric_limits<float>::quiet_NaN();
    row.goal_y = config.has_goal ? config.goal_y : std::numeric_limits<float>::quiet_NaN();
    result.trace.push_back(row);

    if (clearance <= 0.0f)
    {
      result.collided = true;
      break;  // physical collision: scenario failed, no point continuing
    }
  }

  return result;
}

// ---- common path sources ----

/**
 * @brief Straight-line local path from the robot to a goal point, recomputed
 *        every tick in the robot frame (the harness stand-in for a replanned
 *        global plan on a free map). Empty once within goal_tolerance.
 */
inline PathSource
gotoPointPath(float goal_x, float goal_y, float goal_tolerance = 0.3f, float path_length = 4.0f,
              float spacing = 0.1f)
{
  return [=](const Pose &pose, float) {
    std::vector<Point2D> path;
    float dx   = goal_x - pose.x;
    float dy   = goal_y - pose.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    if (dist < goal_tolerance)
    {
      return path;
    }
    // Goal direction in the robot frame
    float cs = std::cos(-pose.th), sn = std::sin(-pose.th);
    float gx = cs * dx - sn * dy;
    float gy = sn * dx + cs * dy;
    float len = std::min(dist, path_length);
    int   n   = std::max(2, static_cast<int>(len / spacing));
    for (int i = 1; i <= n; i++)
    {
      float s = len * static_cast<float>(i) / static_cast<float>(n);
      path.emplace_back(gx / dist * s, gy / dist * s);
    }
    return path;
  };
}

/**
 * @brief Piecewise-linear path through waypoints (a stand-in for a planner
 *        path with corners). Projects the robot onto the polyline and emits
 *        the next path_length meters; empty once within goal_tolerance of
 *        the final waypoint.
 */
inline PathSource
waypointsPath(std::vector<Point2D> waypoints, float goal_tolerance = 0.4f, float path_length = 4.0f,
              float spacing = 0.1f)
{
  return [=](const Pose &pose, float) {
    std::vector<Point2D> path;
    if (waypoints.size() < 2)
    {
      return path;
    }
    float gdx = waypoints.back().x - pose.x, gdy = waypoints.back().y - pose.y;
    if (std::sqrt(gdx * gdx + gdy * gdy) < goal_tolerance)
    {
      return path;
    }
    // Cumulative arclength and projection of the robot onto the polyline
    std::vector<float> cum(waypoints.size(), 0.0f);
    for (size_t i = 1; i < waypoints.size(); i++)
    {
      cum[i] = cum[i - 1] + std::hypot(waypoints[i].x - waypoints[i - 1].x,
                                       waypoints[i].y - waypoints[i - 1].y);
    }
    float best_d = 1e9f, s0 = 0.0f;
    for (size_t i = 0; i + 1 < waypoints.size(); i++)
    {
      float ax = waypoints[i].x, ay = waypoints[i].y;
      float bx = waypoints[i + 1].x, by = waypoints[i + 1].y;
      float abx = bx - ax, aby = by - ay;
      float ll = abx * abx + aby * aby;
      float t  = (ll > 1e-9f)
                     ? clampf(((pose.x - ax) * abx + (pose.y - ay) * aby) / ll, 0.0f, 1.0f)
                     : 0.0f;
      float d = std::hypot(pose.x - (ax + t * abx), pose.y - (ay + t * aby));
      if (d < best_d)
      {
        best_d = d;
        s0     = cum[i] + t * std::sqrt(ll);
      }
    }
    // Emit world points along the polyline from s0, converted to the robot frame
    float cs = std::cos(-pose.th), sn = std::sin(-pose.th);
    for (float s = s0 + spacing; s <= std::min(cum.back(), s0 + path_length); s += spacing)
    {
      size_t i = 1;
      while (i < cum.size() - 1 && cum[i] < s) i++;
      float seg = std::max(cum[i] - cum[i - 1], 1e-6f);
      float t   = (s - cum[i - 1]) / seg;
      float wx  = waypoints[i - 1].x + (waypoints[i].x - waypoints[i - 1].x) * t;
      float wy  = waypoints[i - 1].y + (waypoints[i].y - waypoints[i - 1].y) * t;
      float dx = wx - pose.x, dy = wy - pose.y;
      path.emplace_back(cs * dx - sn * dy, sn * dx + cs * dy);
    }
    return path;
  };
}

}  // namespace bac_sim

#endif  // BAC_SIM_SIM_RUNNER_HPP__
