/**
 * @file motion_model.cpp
 * @brief Motion-model factory
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "motion_model.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "ackermann_motion_model.hpp"
#include "diff_drive_motion_model.hpp"
#include "omni_motion_model.hpp"

namespace bac::detail
{

ProjectedPose2D
projectNonHolonomic(const Twist2D &command, float duration)
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
circumscribedDiskFree(const Footprint &body, const std::vector<Point2D> &points)
{
  const float longitudinal = std::max(body.front, -body.rear);
  const float half_width = body.width / 2.0f;
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

void
validateMotionModelParams(const Params &params)
{
  switch (params.motion_model.type)
  {
    case MotionModelType::DIFF_DRIVE:
      return;
    case MotionModelType::ACKERMANN:
      // The curvature bound is min(w_max, |v| / turn_radius_min). A
      // non-positive value of either makes it negative or infinite, and
      // std::min(std::max(w, -bound), bound) with a negative bound collapses
      // to -|bound| - silently turning a straight command into full lock.
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
      return;
    case MotionModelType::OMNI:
      // Lateral velocity IS the avoidance dimension for a holonomic model.
      // Selecting the model without lateral authority would silently degrade
      // it to a differential drive that cannot steer, so it is refused rather
      // than accepted and quietly crippled.
      if (!(params.limits.vy_max > 0.0f) || !std::isfinite(params.limits.vy_max))
      {
        throw std::invalid_argument(
            "bac: holonomic limits.vy_max must be positive and finite");
      }
      if (!(params.limits.w_max > 0.0f) || !std::isfinite(params.limits.w_max))
      {
        throw std::invalid_argument(
            "bac: holonomic limits.w_max must be positive and finite");
      }
      // The heading regulator multiplies this by a wrapped angle; a negative
      // gain drives the body AWAY from the tangent, and a non-finite one
      // poisons every candidate's yaw rate.
      if (!(params.heading_gain >= 0.0f) || !std::isfinite(params.heading_gain))
      {
        throw std::invalid_argument(
            "bac: holonomic heading_gain must be non-negative and finite");
      }
      if (params.vy_samples < 3)
      {
        throw std::invalid_argument("bac: holonomic vy_samples must be at least 3");
      }
      return;
  }
  throw std::invalid_argument("bac: unsupported motion model");
}

std::unique_ptr<MotionModel>
makeMotionModel(const Params &params)
{
  validateMotionModelParams(params);
  switch (params.motion_model.type)
  {
    case MotionModelType::DIFF_DRIVE:
      return std::make_unique<DiffDriveMotionModel>(params);
    case MotionModelType::ACKERMANN:
      return std::make_unique<AckermannMotionModel>(params);
    case MotionModelType::OMNI:
      return std::make_unique<OmniMotionModel>(params);
  }
  throw std::invalid_argument("bac: unsupported motion model");
}

}  // namespace bac::detail
