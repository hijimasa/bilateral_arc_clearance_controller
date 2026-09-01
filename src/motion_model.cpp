/**
 * @file motion_model.cpp
 * @brief Motion-model factory
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "motion_model.hpp"

#include <memory>
#include <stdexcept>

#include "ackermann_motion_model.hpp"
#include "diff_drive_motion_model.hpp"

namespace bac::detail
{

void
validateMotionModelParams(const Params &params)
{
  switch (params.motion_model.type)
  {
    case MotionModelType::DIFF_DRIVE:
      return;
    case MotionModelType::ACKERMANN:
      // A non-positive radius makes the curvature bound negative or infinite,
      // which silently turns the steering clamp into a full-lock command.
      if (!(params.turn_radius_min > 0.0f))
      {
        throw std::invalid_argument("bac: Ackermann turn_radius_min must be positive");
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
  }
  throw std::invalid_argument("bac: unsupported motion model");
}

}  // namespace bac::detail
