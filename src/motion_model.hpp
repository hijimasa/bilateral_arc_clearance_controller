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

  /**
   * @param yaw_reference Yaw rate the pose regulator wants this tick [rad/s],
   *   derived from the local path tangent. Non-holonomic models ignore it:
   *   for them the yaw rate IS the searched avoidance dimension. A holonomic
   *   model avoids with lateral velocity instead, so it fixes the yaw rate at
   *   this reference and searches (v, vy) - which keeps the trajectory that is
   *   scored and contact-checked the one that is actually driven.
   */
  virtual CandidateBatch sampleCandidates(const Twist2D &current,
                                          float linear_speed_cap,
                                          float yaw_reference) const = 0;
  virtual std::vector<Twist2D> refinementCandidates(const Twist2D &coarse_best) const = 0;
  virtual std::vector<Twist2D> clearanceProbeCommands(float linear_speed) const = 0;

  virtual ProjectedPose2D projectConstantCommand(const Twist2D &command,
                                                  float duration) const = 0;
  virtual bool isCommandKinematicallyValid(const Twist2D &command) const = 0;

  /// The model can hold a commanded orientation independently of its direction
  /// of travel. False for anything that steers with yaw: for those the final
  /// heading is whatever the path tangent leaves behind.
  virtual bool acceptsGoalHeading() const = 0;

  /// The model can emit a standstill rotation `(0, w)` at all.
  virtual bool supportsInPlaceRotation() const = 0;

  /// The model needs to rotate onto the path tangent BEFORE it can translate
  /// along it. True for differential drive; false for Ackermann, which cannot
  /// rotate on the spot, and false for holonomic models, which can translate
  /// in any direction and would only add pointless yaw motion.
  virtual bool usesRotateBeforeTranslate() const = 0;
  virtual bool isInPlaceRotationAdmissible(const std::vector<Point2D> &points) const = 0;

  virtual float commandChange(const Twist2D &command,
                              const Twist2D &previous) const = 0;
  virtual Twist2D limitReachableCommand(const Twist2D &current,
                                        const Twist2D &desired) const = 0;
  /**
   * @param speed Non-negative SPEED along the direction of travel. The
   *   direction itself is the model's business: a non-holonomic model recovers
   *   it from the sign of `command.v`, and a holonomic one from the whole
   *   velocity vector. Handing the caller's sign convention down here reversed
   *   a holonomic command whose forward component happened to be zero.
   */
  virtual Twist2D withLinearSpeed(const Twist2D &command, float speed) const = 0;
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
