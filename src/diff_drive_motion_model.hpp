/**
 * @file diff_drive_motion_model.hpp
 * @brief Internal differential-drive candidate generation and rollout geometry
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#pragma once

#include <vector>

#include "bilateral_arc_clearance_controller/bac_core.hpp"

namespace bac::detail
{

struct ProjectedPose2D
{
  float x;
  float y;
  float theta;
};

struct CandidateBatch
{
  std::vector<Twist2D> commands;
  float angular_min;
  float angular_max;
  float coarse_angular_step;
};

/**
 * Differential-drive policy used by BacCore.
 *
 * This class intentionally owns no controller state. It isolates the parts
 * that differ by vehicle kinematics while preserving BacCore's public API and
 * current candidate ordering. Ackermann and holonomic policies can therefore
 * be introduced without teaching the clearance scorer how commands are made.
 */
class DiffDriveMotionModel
{
public:
  explicit DiffDriveMotionModel(const Params &params);

  CandidateBatch sampleCandidates(const Twist2D &current, float linear_speed_cap) const;

  ProjectedPose2D projectConstantCommand(const Twist2D &command, float duration) const;

  bool isInPlaceRotationAdmissible(const std::vector<Point2D> &points) const;

private:
  const Params &params_;
};

}  // namespace bac::detail
