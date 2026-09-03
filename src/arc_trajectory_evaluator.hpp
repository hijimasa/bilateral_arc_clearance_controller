/**
 * @file arc_trajectory_evaluator.hpp
 * @brief Internal bilateral clearance and swept-footprint evaluation for arcs
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#pragma once

#include <vector>

#include "bilateral_arc_clearance_controller/bac_core.hpp"

namespace bac::detail
{

/**
 * Largest projection of the footprint rectangle onto the direction `(dx, dy)`
 * - the rectangle's support function. For `(1, 0)` this is `front`, for
 * `(-1, 0)` it is `-rear`, and for `(0, 1)` it is `width / 2`, so a
 * non-holonomic command reproduces the front/rear/half-width extents exactly.
 *
 * Shared by the swept-arc frame and by the emergency governor's slab test:
 * both need the body's extent along an arbitrary direction of travel, and
 * they are required to agree.
 */
float supportExtent(const Footprint &body, float dx, float dy);

/**
 * Evaluates one constant-curvature trajectory independently from candidate
 * generation and scoring. The implementation is the extracted BacCore arc
 * geometry; keeping it separate lets the differential-drive and Ackermann
 * policies share it, while an omnidirectional model could provide a different
 * swept-trajectory evaluator.
 */
class ArcTrajectoryEvaluator
{
public:
  explicit ArcTrajectoryEvaluator(const Params &params);

  ArcEvaluation evaluate(const std::vector<Point2D> &points, const Twist2D &command,
                         float clearance_distance, float blocking_distance) const;

private:
  const Params &params_;
};

}  // namespace bac::detail
