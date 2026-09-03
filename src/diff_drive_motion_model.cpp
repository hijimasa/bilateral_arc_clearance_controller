/**
 * @file diff_drive_motion_model.cpp
 * @brief Internal differential-drive candidate generation and rollout geometry
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "diff_drive_motion_model.hpp"

#include <algorithm>
#include <cmath>

namespace bac::detail
{

DiffDriveMotionModel::DiffDriveMotionModel(const Params &params)
  : params_(params)
{
}

CandidateBatch
DiffDriveMotionModel::sampleCandidates(const Twist2D &current, float linear_speed_cap,
                                        float /*yaw_reference*/) const
{
  const float window_dv = params_.limits.acc_v * params_.window_time;
  float v_lo = std::max(params_.limits.v_min, current.v - window_dv);
  float v_hi = std::min(linear_speed_cap, current.v + window_dv);
  if (v_hi < v_lo)
  {
    v_lo = v_hi = std::max(params_.limits.v_min, v_hi);
  }

  std::vector<float> linear_velocities;
  std::vector<float> angular_velocities;
  CandidateBatch batch;
  const float angular_min = -params_.limits.w_max;
  const float angular_max = params_.limits.w_max;

  linear_velocities.push_back(0.0f);  // rotation / stop row
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

  // Reverse is offered only when accel-reachable (near standstill). BacCore
  // later keeps it only when no safe forward candidate advances the path.
  const float v_rev = std::max(params_.limits.v_min, current.v - window_dv);
  if (v_rev < -1e-3f)
  {
    linear_velocities.push_back(v_rev);
    linear_velocities.push_back(v_rev / 2.0f);
  }

  const int w_samples = std::max(3, params_.w_samples);
  for (int i = 0; i < w_samples; ++i)
  {
    angular_velocities.push_back(
        angular_min + (angular_max - angular_min) * static_cast<float>(i) /
                                static_cast<float>(w_samples - 1));
  }
  if (angular_min < 0.0f && angular_max > 0.0f)
  {
    angular_velocities.push_back(0.0f);  // always offer the straight arc
  }

  batch.commands.reserve(linear_velocities.size() * angular_velocities.size());
  for (float v : linear_velocities)
  {
    for (float w : angular_velocities)
    {
      batch.commands.emplace_back(v, w);
    }
  }
  return batch;
}

std::vector<Twist2D>
DiffDriveMotionModel::refinementCandidates(const Twist2D &coarse_best) const
{
  std::vector<Twist2D> commands;
  if (params_.w_refine_steps <= 0 || std::fabs(coarse_best.v) <= 1e-3f)
  {
    return commands;
  }

  const float angular_min = -params_.limits.w_max;
  const float angular_max = params_.limits.w_max;
  const int w_samples = std::max(3, params_.w_samples);
  const float coarse_step =
      (angular_max - angular_min) / static_cast<float>(std::max(w_samples - 1, 1));
  commands.reserve(static_cast<std::size_t>(2 * params_.w_refine_steps));
  for (int i = 1; i <= params_.w_refine_steps; ++i)
  {
    const float dw = coarse_step * static_cast<float>(i) /
                     static_cast<float>(params_.w_refine_steps + 1);
    for (float w : { coarse_best.w - dw, coarse_best.w + dw })
    {
      if (w >= angular_min && w <= angular_max)
      {
        commands.emplace_back(coarse_best.v, w);
      }
    }
  }
  return commands;
}

std::vector<Twist2D>
DiffDriveMotionModel::clearanceProbeCommands(float linear_speed) const
{
  return { { linear_speed, -0.4f }, { linear_speed, 0.0f }, { linear_speed, 0.4f } };
}

ProjectedPose2D
DiffDriveMotionModel::projectConstantCommand(const Twist2D &command, float duration) const
{
  // Identical to the Ackermann rollout; see projectNonHolonomic.
  return projectNonHolonomic(command, duration);
}

bool
DiffDriveMotionModel::isCommandKinematicallyValid(const Twist2D &command) const
{
  return std::fabs(command.v) <= 1e-3f || std::fabs(command.w) <= 1e-4f ||
         std::fabs(command.v) / std::fabs(command.w) >= params_.turn_radius_min;
}

bool
DiffDriveMotionModel::acceptsGoalHeading() const
{
  // This model steers with yaw: its orientation is not free to be chosen,
  // so a commanded goal orientation cannot be honoured.
  return false;
}

bool
DiffDriveMotionModel::supportsInPlaceRotation() const
{
  return true;
}

bool
DiffDriveMotionModel::usesRotateBeforeTranslate() const
{
  // A differential-drive body cannot translate sideways, so a path tangent
  // far off the current heading is reached by rotating first.
  return true;
}

bool
DiffDriveMotionModel::isInPlaceRotationAdmissible(const std::vector<Point2D> &points) const
{
  // A full in-place rotation sweeps the disk of the circumscribed radius.
  // This deliberately preserves the former conservative BacCore test.
  const float longitudinal = std::max(params_.footprint.front, -params_.footprint.rear);
  const float half_width = params_.footprint.width / 2.0f;
  const float circumscribed =
      std::sqrt(longitudinal * longitudinal + half_width * half_width);
  for (const Point2D &point : points)
  {
    if (std::sqrt(point.x * point.x + point.y * point.y) < circumscribed + 0.02f)
    {
      return false;
    }
  }
  return true;
}

float
DiffDriveMotionModel::commandChange(const Twist2D &command, const Twist2D &previous) const
{
  return std::fabs(command.w - previous.w);
}

Twist2D
DiffDriveMotionModel::limitReachableCommand(const Twist2D &current,
                                            const Twist2D &desired) const
{
  Twist2D limited = desired;
  if (params_.limits.acc_w > 1e-3f && params_.control_period > 1e-4f)
  {
    const float dw = params_.limits.acc_w * params_.control_period;
    limited.w = std::min(std::max(desired.w, current.w - dw), current.w + dw);
  }
  return limited;
}

Twist2D
DiffDriveMotionModel::withLinearSpeed(const Twist2D &command, float speed) const
{
  // The direction of travel is the sign of the forward component; `speed` is a
  // magnitude.
  const float signed_speed = (command.v >= 0.0f ? 1.0f : -1.0f) * std::fabs(speed);
  if (std::fabs(command.v) <= 1e-3f || std::fabs(speed) <= 1e-3f)
  {
    return { signed_speed, 0.0f };
  }
  // Preserve the curvature of the reachable arc that was checked for
  // contact. Keeping w fixed here would tighten w/v as speed is reduced and
  // silently replace the selected geometric path.
  return { signed_speed, command.w * signed_speed / command.v };
}

Twist2D
DiffDriveMotionModel::applyCommandDeadband(const Twist2D &command) const
{
  Twist2D result = command;
  if (std::fabs(result.v) < params_.velocity_min)
  {
    result.v = 0.0f;
  }
  if (std::fabs(result.w) < params_.angvel_min)
  {
    result.w = 0.0f;
  }
  return result;
}

}  // namespace bac::detail
