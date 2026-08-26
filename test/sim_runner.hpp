/**
 * @file sim_runner.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Closed-loop simulation runner for the BAC scenario harness
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 REACT Co., Ltd.
 *
 * Replicates the integration loop the core is designed for:
 *  - v sign mixing with the current velocity before process()
 *  - radius-preserving clamp of the output back to the commanded speed
 *  - arbitration: the avoid output is applied only while status != CLEAR
 * and closes the loop with a unicycle kinematics model and a simulated LiDAR.
 */

#pragma once
#ifndef BAC_SIM_SIM_RUNNER_HPP__
#define BAC_SIM_SIM_RUNNER_HPP__

#include <cmath>
#include <functional>
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
};

struct TraceRow
{
  float   t;
  Pose    pose;
  Twist2D command;  // upper-level command (object_vec)
  Twist2D output;   // avoid output after main-loop clamp
  Twist2D actual;   // actuator state applied to kinematics
  int     status;   // Status returned this tick
  float   clearance;
  float   speed_fraction;     // Result extras, for evaluation plots
  float   command_clearance;
};

// Upper-level command source: (pose, current velocities, time) -> command
using CommandSource = std::function<Twist2D(const Pose &, const Twist2D &, float)>;

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
              const CommandSource &command_source, const SimConfig &config)
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

    Twist2D object_vec  = command_source(pose, actual, t);
    Twist2D current_vec = actual;
    Twist2D command_vec = object_vec;

    // --- upstream pre-processing: v sign mixing with the current velocity.
    // (Never mix w: overwriting the commanded w with the current w replaces
    // the upper-level intent with our own avoidance state and self-locks the
    // avoidance into its current turn.)
    if (command_vec.v * current_vec.v < 0.0f && std::fabs(current_vec.v) > 0.1f)
    {
      command_vec.v = current_vec.v;
    }

    bac::Result avoid = core.process(points, command_vec, current_vec);
    Twist2D           output_vec = avoid.output;

    // --- post-processing: clamp back to the upper command speed, keep radius ---
    if (std::fabs(output_vec.v) > std::fabs(object_vec.v))
    {
      if (output_vec.v != 0.0f && output_vec.w != 0.0f)
      {
        float r      = output_vec.v / output_vec.w;
        output_vec.w = object_vec.v / r;
      }
      output_vec.v = object_vec.v;
    }

    // --- arbitration: the avoid output only overrides while status != CLEAR ---
    Twist2D applied = (avoid.status == Status::CLEAR) ? object_vec : output_vec;

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
    row.t                 = t;
    row.pose              = pose;
    row.command           = object_vec;
    row.output            = output_vec;
    row.actual            = actual;
    row.status            = static_cast<int>(avoid.status);
    row.clearance         = clearance;
    row.speed_fraction    = avoid.speed_fraction;
    row.command_clearance = avoid.command_clearance;
    result.trace.push_back(row);

    if (clearance <= 0.0f)
    {
      result.collided = true;
      break;  // physical collision: scenario failed, no point continuing
    }
  }

  return result;
}

// ---- common command sources ----

inline CommandSource
constantCommand(float v, float w)
{
  return [v, w](const Pose &, const Twist2D &, float) { return Twist2D(v, w); };
}

/**
 * @brief Simple upper-level navigator: P-control on heading towards a goal point.
 *        Stops (0,0) once within goal_tolerance.
 */
inline CommandSource
gotoPointCommand(float goal_x, float goal_y, float cruise_v = 0.4f, float k_heading = 1.5f, float w_max = 0.6f,
                 float goal_tolerance = 0.3f)
{
  return [=](const Pose &pose, const Twist2D &, float) {
    float dx   = goal_x - pose.x;
    float dy   = goal_y - pose.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    if (dist < goal_tolerance)
    {
      return Twist2D(0.0f, 0.0f);
    }
    float heading_error = std::atan2(dy, dx) - pose.th;
    while (heading_error > M_PI) heading_error -= 2.0f * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0f * M_PI;
    float w = clampf(k_heading * heading_error, -w_max, w_max);
    float v = cruise_v * clampf(dist / 1.0f, 0.2f, 1.0f);  // slow down near goal
    return Twist2D(v, w);
  };
}

}  // namespace bac_sim

#endif  // BAC_SIM_SIM_RUNNER_HPP__
