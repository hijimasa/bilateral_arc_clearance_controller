/**
 * @file metrics.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Trajectory quality metrics for the BAC scenario harness
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#pragma once
#ifndef BAC_SIM_METRICS_HPP__
#define BAC_SIM_METRICS_HPP__

#include <cmath>
#include <vector>
#include "sim_runner.hpp"

namespace bac_sim
{

constexpr float kAngvelMin = 0.01f;

struct MetricsOptions
{
  // Goal check (enabled when goal_tolerance > 0)
  float goal_x = 0.0f, goal_y = 0.0f;
  float goal_tolerance = 0.0f;

  // Lateral deviation vs a corridor centerline y = center_y, evaluated for x in [x_from, x_to]
  bool  eval_lateral = false;
  float center_y = 0.0f, x_from = 0.0f, x_to = 0.0f;
};

struct Metrics
{
  bool  collided       = false;
  float min_clearance  = 1e9f;
  float time_to_goal   = -1.0f;  // <0: goal not reached
  float final_x        = 0.0f;
  float final_y        = 0.0f;

  int stop_ticks   = 0;  // actual speed ~0 while a nonzero command was given
  int status_stop  = 0;  // ticks with AVOID_STATUS_STOP
  int status_avoid = 0;  // ticks with AVOID_STATUS_AVOIDING
  int total_ticks  = 0;

  int   w_sign_changes = 0;    // oscillation of the applied angular velocity
  float max_dw         = 0.0f; // max |Δw| between consecutive ticks on the output [rad/s]

  float mean_abs_lateral = 0.0f;  // vs corridor centerline (if eval_lateral)
  float max_abs_lateral  = 0.0f;
  int   lateral_samples  = 0;
};

inline Metrics
computeMetrics(const SimResult &result, const MetricsOptions &options)
{
  Metrics m;
  m.collided    = result.collided;
  m.total_ticks = static_cast<int>(result.trace.size());

  float prev_w      = 0.0f;
  int   prev_w_sign = 0;
  float lateral_sum = 0.0f;

  for (const TraceRow &row : result.trace)
  {
    m.min_clearance = std::min(m.min_clearance, row.clearance);
    m.final_x       = row.pose.x;
    m.final_y       = row.pose.y;

    if (row.status == static_cast<int>(Status::STOP))
    {
      m.status_stop++;
    }
    else if (row.status == static_cast<int>(Status::AVOIDING))
    {
      m.status_avoid++;
    }

    if (std::fabs(row.command.v) > 0.05f && std::fabs(row.actual.v) < 0.01f)
    {
      m.stop_ticks++;
    }

    m.max_dw = std::max(m.max_dw, std::fabs(row.output.w - prev_w));
    prev_w   = row.output.w;

    int w_sign = (row.actual.w > kAngvelMin) ? 1 : (row.actual.w < -kAngvelMin ? -1 : 0);
    if (w_sign != 0 && prev_w_sign != 0 && w_sign != prev_w_sign)
    {
      m.w_sign_changes++;
    }
    if (w_sign != 0)
    {
      prev_w_sign = w_sign;
    }

    if (options.goal_tolerance > 0.0f && m.time_to_goal < 0.0f)
    {
      float dx = options.goal_x - row.pose.x;
      float dy = options.goal_y - row.pose.y;
      if (std::sqrt(dx * dx + dy * dy) < options.goal_tolerance)
      {
        m.time_to_goal = row.t;
      }
    }

    if (options.eval_lateral && row.pose.x >= options.x_from && row.pose.x <= options.x_to)
    {
      float lateral = std::fabs(row.pose.y - options.center_y);
      lateral_sum += lateral;
      m.max_abs_lateral = std::max(m.max_abs_lateral, lateral);
      m.lateral_samples++;
    }
  }

  if (m.lateral_samples > 0)
  {
    m.mean_abs_lateral = lateral_sum / m.lateral_samples;
  }

  return m;
}

}  // namespace bac_sim

#endif  // BAC_SIM_METRICS_HPP__
