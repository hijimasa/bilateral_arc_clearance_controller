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
 * Pose reached from the origin by holding a non-holonomic `(v, w)` for
 * `duration`, in the body frame.
 *
 * Shared by the differential-drive and Ackermann models, whose constant-twist
 * rollouts are the same closed form: a straight run below the yaw-rate
 * threshold, otherwise a circular arc of radius `v / w`.
 *
 * Deliberately NOT used by the holonomic model. Its general form is written
 * about the centre of rotation `(-vy/w, v/w)` and does not reduce to this one
 * numerically at `vy == 0`: `pose.y` is `cy - cy*cos(theta)` there against
 * `cy*(1 - cos(theta))` here, and the two cancellations lose different bits.
 *
 * Measured over 1,999,936 samples - `v` uniform on [-1.5, 1.5], `|w|` uniform
 * on [1e-4, 3] with a random sign, `duration` uniform on [0.01, 4], excluding
 * the `|w| < 1e-4` straight branch where both forms agree trivially.
 * `pose.x` agrees bit for bit in every sample. `pose.y` differs in 999,447 of
 * them (50.0%), and the difference is NOT confined to the last place: only
 * 54.5% of the differing samples are exactly 1 ULP apart, 22.6% are more than
 * 8 ULP apart, and the tail reaches 8,359,055 ULP - 2.4e-4 m in absolute
 * terms, at v = -1.24, w = 3.0e-4, duration = 1.14, where `cy = v/w` is large
 * and the cancellation is catastrophic on one side only. Restricting `|w|` to
 * a well-conditioned [0.05, 2] does not remove it (52.3% exactly 1 ULP, worst
 * case 9.5e-7 m).
 *
 * So folding the holonomic model into this helper would move its output by up
 * to a sub-millimetre, not by a rounding step. Do not.
 */
ProjectedPose2D projectNonHolonomic(const Twist2D &command, float duration);

/**
 * True when no point lies inside the disk that a full in-place rotation of
 * `body` sweeps, plus a 2 cm allowance.
 *
 * The conservative standstill-rotation test, shared by the differential-drive
 * and holonomic models: both sweep the same circumscribed disk, so both ask
 * the same question. Preserves the former BacCore test exactly.
 */
bool circumscribedDiskFree(const Footprint &body, const std::vector<Point2D> &points);

/**
 * Rejects a kinematic configuration no model can honour.
 *
 * Separated from makeMotionModel so a caller can validate BEFORE mutating the
 * parameters its model is bound to by reference.
 */
void validateMotionModelParams(const Params &params);

std::unique_ptr<MotionModel> makeMotionModel(const Params &params);

}  // namespace bac::detail
