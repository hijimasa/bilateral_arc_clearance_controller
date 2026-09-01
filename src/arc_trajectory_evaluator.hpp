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
 * Evaluates one constant-curvature trajectory independently from candidate
 * generation and scoring. The implementation is the extracted BacCore arc
 * geometry; keeping it separate allows Ackermann to reuse it while an
 * omnidirectional model can provide a different swept-trajectory evaluator.
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
