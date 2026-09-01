/**
 * @file diff_drive_motion_model.hpp
 * @brief Internal differential-drive candidate generation and rollout geometry
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#pragma once

#include "motion_model.hpp"

namespace bac::detail
{

/**
 * Differential-drive policy used by BacCore.
 *
 * This class intentionally owns no controller state. It isolates the parts
 * that differ by vehicle kinematics while preserving BacCore's public API and
 * current candidate ordering. Ackermann and holonomic policies can therefore
 * be introduced without teaching the clearance scorer how commands are made.
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  explicit DiffDriveMotionModel(const Params &params);

  CandidateBatch sampleCandidates(const Twist2D &current, float linear_speed_cap,
                                  float yaw_reference) const override;
  std::vector<Twist2D> refinementCandidates(const Twist2D &coarse_best) const override;
  std::vector<Twist2D> clearanceProbeCommands(float linear_speed) const override;

  ProjectedPose2D projectConstantCommand(const Twist2D &command,
                                         float duration) const override;
  bool isCommandKinematicallyValid(const Twist2D &command) const override;

  bool acceptsGoalHeading() const override;
  bool supportsInPlaceRotation() const override;
  bool usesRotateBeforeTranslate() const override;
  bool isInPlaceRotationAdmissible(const std::vector<Point2D> &points) const override;

  float commandChange(const Twist2D &command, const Twist2D &previous) const override;
  Twist2D limitReachableCommand(const Twist2D &current,
                                const Twist2D &desired) const override;
  Twist2D withLinearSpeed(const Twist2D &command, float speed) const override;
  Twist2D applyCommandDeadband(const Twist2D &command) const override;
private:
  const Params &params_;
};

}  // namespace bac::detail
