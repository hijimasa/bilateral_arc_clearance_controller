/**
 * @file motion_model.hpp
 * @brief Internal kinematic-policy boundary for candidate generation
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#pragma once

#include <memory>
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
};

/**
 * Kinematic operations needed by the common BAC scorer.
 *
 * Implementations are stateless across control ticks. BacCore owns the
 * previous scoring state and passes it explicitly.
 */
class MotionModel
{
public:
  virtual ~MotionModel() = default;

  virtual CandidateBatch sampleCandidates(const Twist2D &current,
                                          float linear_speed_cap) const = 0;
  virtual std::vector<Twist2D> refinementCandidates(const Twist2D &coarse_best) const = 0;
  virtual std::vector<Twist2D> clearanceProbeCommands(float linear_speed) const = 0;

  virtual ProjectedPose2D projectConstantCommand(const Twist2D &command,
                                                  float duration) const = 0;
  virtual bool isCommandKinematicallyValid(const Twist2D &command) const = 0;

  virtual bool supportsInPlaceRotation() const = 0;
  virtual bool isInPlaceRotationAdmissible(const std::vector<Point2D> &points) const = 0;

  virtual float commandChange(const Twist2D &command,
                              const Twist2D &previous) const = 0;
  virtual Twist2D limitReachableCommand(const Twist2D &current,
                                        const Twist2D &desired) const = 0;
  virtual Twist2D withLinearSpeed(const Twist2D &command,
                                  float linear_speed) const = 0;
  virtual Twist2D applyCommandDeadband(const Twist2D &command) const = 0;
};

/**
 * Rejects a kinematic configuration no model can honour.
 *
 * Separated from makeMotionModel so a caller can validate BEFORE mutating the
 * parameters its model is bound to by reference.
 */
void validateMotionModelParams(const Params &params);

std::unique_ptr<MotionModel> makeMotionModel(const Params &params);

}  // namespace bac::detail
