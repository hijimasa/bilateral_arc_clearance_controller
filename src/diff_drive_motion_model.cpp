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
DiffDriveMotionModel::sampleCandidates(const Twist2D &current, float linear_speed_cap) const
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
  batch.angular_min = -params_.limits.w_max;
  batch.angular_max = params_.limits.w_max;

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
        batch.angular_min + (batch.angular_max - batch.angular_min) * static_cast<float>(i) /
                                static_cast<float>(w_samples - 1));
  }
  if (batch.angular_min < 0.0f && batch.angular_max > 0.0f)
  {
    angular_velocities.push_back(0.0f);  // always offer the straight arc
  }
  batch.coarse_angular_step =
      (batch.angular_max - batch.angular_min) / static_cast<float>(std::max(w_samples - 1, 1));

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

ProjectedPose2D
DiffDriveMotionModel::projectConstantCommand(const Twist2D &command, float duration) const
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

}  // namespace bac::detail
