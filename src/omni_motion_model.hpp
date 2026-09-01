/**
 * @file omni_motion_model.hpp
 * @brief Internal holonomic candidate generation and rollout geometry
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#pragma once

#include "motion_model.hpp"

namespace bac::detail
{

/**
 * Holonomic (omnidirectional) vehicle model.
 *
 * The avoidance dimension is LATERAL VELOCITY, not yaw rate. A holonomic body
 * can side-step an obstacle without changing where it points, so searching
 * over yaw would spend the candidate budget on a degree of freedom that does
 * not avoid anything. The yaw rate instead regulates the body onto the local
 * path tangent, and is fixed across the candidate set for one tick, so the
 * trajectory that is scored is the trajectory that is driven.
 *
 * The candidate set is therefore (forward speed x lateral speed) at a single
 * yaw rate - the same lattice size as the differential-drive (v x w) set, not
 * a three-dimensional one.
 */
class OmniMotionModel : public MotionModel
{
public:
  explicit OmniMotionModel(const Params &params);

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
  Twist2D withLinearSpeed(const Twist2D &command, float requested) const override;
  Twist2D applyCommandDeadband(const Twist2D &command) const override;

private:
  const Params &params_;
};

}  // namespace bac::detail
