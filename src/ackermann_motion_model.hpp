/**
 * @file ackermann_motion_model.hpp
 * @brief Ackermann candidate generation in body-curvature space
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#pragma once

#include "motion_model.hpp"

namespace bac::detail
{

/**
 * Ackermann policy expressed through BacCore's (forward speed, yaw rate)
 * command. Candidates are sampled in curvature and constrained by the minimum
 * turning radius. Road-wheel kinematics belong to the downstream controller.
 * In-place rotation is never offered.
 */
class AckermannMotionModel : public MotionModel
{
public:
  explicit AckermannMotionModel(const Params &params);

  CandidateBatch sampleCandidates(const Twist2D &current,
                                  float linear_speed_cap) const override;
  std::vector<Twist2D> refinementCandidates(const Twist2D &coarse_best) const override;
  std::vector<Twist2D> clearanceProbeCommands(float linear_speed) const override;

  ProjectedPose2D projectConstantCommand(const Twist2D &command,
                                         float duration) const override;
  bool isCommandKinematicallyValid(const Twist2D &command) const override;

  bool supportsInPlaceRotation() const override;
  bool isInPlaceRotationAdmissible(const std::vector<Point2D> &points) const override;

  float commandChange(const Twist2D &command, const Twist2D &previous) const override;
  Twist2D limitReachableCommand(const Twist2D &current,
                                const Twist2D &desired) const override;
  Twist2D withLinearSpeed(const Twist2D &command, float linear_speed) const override;
  Twist2D applyCommandDeadband(const Twist2D &command) const override;
private:
  float curvatureBound(float linear_speed) const;
  float curvature(const Twist2D &command) const;
  Twist2D commandFromCurvature(float linear_speed, float curvature) const;
  float yawRateBound(float linear_speed) const;

  const Params &params_;
};

}  // namespace bac::detail
