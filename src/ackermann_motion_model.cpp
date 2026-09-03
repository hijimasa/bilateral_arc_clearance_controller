/**
 * @file ackermann_motion_model.cpp
 * @brief Ackermann candidate generation in body-curvature space
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "ackermann_motion_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bac::detail
{

AckermannMotionModel::AckermannMotionModel(const Params &params)
  : params_(params)
{
  // Also checked by the factory; repeated here so direct construction cannot
  // produce a model whose curvature bound is negative or infinite. The bound
  // is min(w_max, |v| / turn_radius_min), so both terms must be checked.
  if (!(params.turn_radius_min > 0.0f) || !std::isfinite(params.turn_radius_min))
  {
    throw std::invalid_argument(
        "bac: Ackermann turn_radius_min must be positive and finite");
  }
  if (!(params.limits.w_max > 0.0f) || !std::isfinite(params.limits.w_max))
  {
    throw std::invalid_argument(
        "bac: Ackermann limits.w_max must be positive and finite");
  }
}

float
AckermannMotionModel::yawRateBound(float linear_speed) const
{
  if (std::fabs(linear_speed) <= 1e-4f)
  {
    return 0.0f;
  }
  return std::min(params_.limits.w_max,
                  std::fabs(linear_speed) / params_.turn_radius_min);
}

float
AckermannMotionModel::curvatureBound(float linear_speed) const
{
  if (std::fabs(linear_speed) <= 1e-4f)
  {
    return 0.0f;
  }
  return yawRateBound(linear_speed) / std::fabs(linear_speed);
}

float
AckermannMotionModel::curvature(const Twist2D &command) const
{
  return std::fabs(command.v) > 1e-4f ? command.w / command.v : 0.0f;
}

Twist2D
AckermannMotionModel::commandFromCurvature(float linear_speed, float command_curvature) const
{
  return { linear_speed, linear_speed * command_curvature };
}

CandidateBatch
AckermannMotionModel::sampleCandidates(const Twist2D &current, float linear_speed_cap,
                                       float /*yaw_reference*/) const
{
  CandidateBatch batch;
  batch.commands.emplace_back(0.0f, 0.0f);  // Ackermann stop; never a rotation row

  const float window_dv = params_.limits.acc_v * params_.window_time;
  float v_lo = std::max(params_.limits.v_min, current.v - window_dv);
  float v_hi = std::min(linear_speed_cap, current.v + window_dv);
  if (v_hi < v_lo)
  {
    v_lo = v_hi = std::max(params_.limits.v_min, v_hi);
  }

  std::vector<float> linear_velocities;
  const int v_samples = std::max(2, params_.v_samples);
  for (int i = 0; i < v_samples; ++i)
  {
    const float v = v_lo + (v_hi - v_lo) * static_cast<float>(i) /
                                static_cast<float>(v_samples - 1);
    if (v > 1e-3f)
    {
      linear_velocities.push_back(v);
    }
  }

  // Keep the same near-standstill reverse escape policy as the differential
  // model, but every reverse candidate still obeys the steering geometry.
  const float v_rev = std::max(params_.limits.v_min, current.v - window_dv);
  if (v_rev < -1e-3f)
  {
    linear_velocities.push_back(v_rev);
    // Re-check the half-speed row: isCommandKinematicallyValid treats anything
    // slower than 1e-3 as a standstill and would reject it.
    if (v_rev / 2.0f < -1e-3f)
    {
      linear_velocities.push_back(v_rev / 2.0f);
    }
  }

  const int curvature_samples = std::max(3, params_.w_samples);
  batch.commands.reserve(
      1U + linear_velocities.size() * static_cast<std::size_t>(curvature_samples + 1));
  for (float v : linear_velocities)
  {
    const float bound = curvatureBound(v);
    for (int i = 0; i < curvature_samples; ++i)
    {
      const float command_curvature =
          -bound + 2.0f * bound * static_cast<float>(i) /
                       static_cast<float>(curvature_samples - 1);
      batch.commands.push_back(commandFromCurvature(v, command_curvature));
    }
    if (bound > 0.0f)
    {
      batch.commands.emplace_back(v, 0.0f);  // preserve an explicit straight candidate
    }
  }
  return batch;
}

std::vector<Twist2D>
AckermannMotionModel::refinementCandidates(const Twist2D &coarse_best) const
{
  std::vector<Twist2D> commands;
  if (params_.w_refine_steps <= 0 || std::fabs(coarse_best.v) <= 1e-3f)
  {
    return commands;
  }

  const float bound = curvatureBound(coarse_best.v);
  if (!(bound > 0.0f))
  {
    return commands;  // every offset would collapse onto the coarse winner
  }
  const int curvature_samples = std::max(3, params_.w_samples);
  const float coarse_step =
      2.0f * bound / static_cast<float>(std::max(curvature_samples - 1, 1));
  const float center = curvature(coarse_best);
  commands.reserve(static_cast<std::size_t>(2 * params_.w_refine_steps));
  for (int i = 1; i <= params_.w_refine_steps; ++i)
  {
    const float offset = coarse_step * static_cast<float>(i) /
                         static_cast<float>(params_.w_refine_steps + 1);
    for (float command_curvature : { center - offset, center + offset })
    {
      if (command_curvature >= -bound && command_curvature <= bound)
      {
        commands.push_back(commandFromCurvature(coarse_best.v, command_curvature));
      }
    }
  }
  return commands;
}

std::vector<Twist2D>
AckermannMotionModel::clearanceProbeCommands(float linear_speed) const
{
  const float probe_yaw = std::min(0.4f, yawRateBound(linear_speed));
  return { { linear_speed, -probe_yaw },
           { linear_speed, 0.0f },
           { linear_speed, probe_yaw } };
}

ProjectedPose2D
AckermannMotionModel::projectConstantCommand(const Twist2D &command, float duration) const
{
  ProjectedPose2D pose{ 0.0f, 0.0f, command.w * duration };
  if (std::fabs(command.w) < 1e-4f)
  {
    pose.x = command.v * duration;
    return pose;
  }

  const float radius = command.v / command.w;
  pose.x = radius * std::sin(pose.theta);
  pose.y = radius * (1.0f - std::cos(pose.theta));
  return pose;
}

bool
AckermannMotionModel::isCommandKinematicallyValid(const Twist2D &command) const
{
  if (std::fabs(command.v) <= 1e-3f)
  {
    return std::fabs(command.w) <= 1e-4f;
  }
  return std::fabs(command.w) <= yawRateBound(command.v) + 1e-5f;
}

bool
AckermannMotionModel::acceptsGoalHeading() const
{
  // This model steers with yaw: its orientation is not free to be chosen,
  // so a commanded goal orientation cannot be honoured.
  return false;
}

bool
AckermannMotionModel::supportsInPlaceRotation() const
{
  return false;
}

bool
AckermannMotionModel::usesRotateBeforeTranslate() const
{
  // A steered vehicle cannot rotate on the spot at all, so it never aligns
  // before translating; it steers onto the tangent while moving.
  return false;
}

bool
AckermannMotionModel::isInPlaceRotationAdmissible(
    const std::vector<Point2D> & /*points*/) const
{
  // Unconditional, and load-bearing: a steered vehicle cannot rotate on the
  // spot at any clearance, so no point cloud can make a standstill rotation
  // admissible. BacCore relies on this to gate rotation on this predicate
  // alone, without a separate `supportsInPlaceRotation()` conjunct.
  return false;
}

float
AckermannMotionModel::commandChange(const Twist2D &command,
                                    const Twist2D &previous) const
{
  return std::fabs(curvature(command) - curvature(previous));
}

Twist2D
AckermannMotionModel::limitReachableCommand(const Twist2D &current,
                                            const Twist2D &desired) const
{
  if (std::fabs(desired.v) <= 1e-3f)
  {
    return { 0.0f, 0.0f };
  }

  Twist2D limited = desired;
  if (params_.limits.acc_w > 1e-3f && params_.control_period > 1e-4f)
  {
    const float dw = params_.limits.acc_w * params_.control_period;
    limited.w = std::min(std::max(desired.w, current.w - dw), current.w + dw);
  }
  const float bound = yawRateBound(limited.v);
  limited.w = std::min(std::max(limited.w, -bound), bound);
  return limited;
}

Twist2D
AckermannMotionModel::withLinearSpeed(const Twist2D &command, float speed) const
{
  if (std::fabs(speed) <= 1e-3f)
  {
    return { 0.0f, 0.0f };
  }
  // Preserve the selected curvature across collision-driven speed reduction.
  // `speed` is a magnitude; a reversing command keeps reversing.
  const float signed_speed = (command.v >= 0.0f ? 1.0f : -1.0f) * std::fabs(speed);
  return commandFromCurvature(signed_speed, curvature(command));
}

Twist2D
AckermannMotionModel::applyCommandDeadband(const Twist2D &command) const
{
  if (std::fabs(command.v) < params_.velocity_min)
  {
    return { 0.0f, 0.0f };
  }

  // A small yaw rate at low speed can still represent material curvature.
  // Applying an angular deadband independently would alter the selected arc.
  Twist2D result = command;
  if (std::fabs(curvature(result)) < 1e-5f)
  {
    result.w = 0.0f;
  }
  return result;
}

}  // namespace bac::detail
