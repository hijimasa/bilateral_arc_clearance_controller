/**
 * @file adapter_utils.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief ROS-independent geometry shared by the BAC ROS adapters
 * @date 2026-08-29
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#pragma once
#ifndef BILATERAL_ARC_CLEARANCE_CONTROLLER__ADAPTER_UTILS_HPP_
#define BILATERAL_ARC_CLEARANCE_CONTROLLER__ADAPTER_UTILS_HPP_

#include <cstddef>
#include <optional>
#include <vector>

#include "bilateral_arc_clearance_controller/bac_core.hpp"

namespace bac
{

struct ScanProjection
{
  std::vector<Point2D> points;
  std::size_t valid_ray_count = 0;
};

/**
 * @brief Convert planar range samples into robot-frame obstacle points.
 *
 * Positive infinity and finite samples above range_max may count as valid
 * clear-space measurements, but never create obstacle points. NaN, negative
 * infinity, and samples below range_min are invalid. sensor_* is the fixed 2D
 * transform from the scan frame to the robot frame.
 */
ScanProjection projectScan(const std::vector<float> &ranges, float angle_min,
                           float angle_increment, float range_min, float range_max,
                           float obstacle_range_max, int downsample, bool inf_is_valid,
                           float sensor_x = 0.0f, float sensor_y = 0.0f,
                           float sensor_yaw = 0.0f);

/**
 * @brief Goal orientation in the base frame, when the goal is still on the plan.
 *
 * Nav2 carries the requested goal orientation on the LAST plan pose. It is a
 * goal orientation only while that pose is still in the pruned path: pruning
 * stops at max_range, and the orientation of an intermediate waypoint is a path
 * tangent. Returns nullopt when the plan end did not survive pruning, so a
 * caller cannot mistake a waypoint heading for a goal heading.
 *
 * @param plan_end       Last plan pose position, in the PLAN frame.
 * @param local_path     Pruned path, already in the BASE frame.
 * @param plan_goal_yaw  Orientation of the last plan pose, in the PLAN frame.
 */
std::optional<float> goalHeadingInBase(const Point2D &plan_end,
                                       const std::vector<Point2D> &local_path,
                                       float transform_x, float transform_y,
                                       float transform_yaw, float plan_goal_yaw);

/**
 * @brief Apply a 2D rigid transform, prune before the robot-nearest point,
 * and retain the following contiguous path window within max_range.
 *
 * @param plan_yaw   Optional per-pose orientation of `path`, in the PLAN
 *   frame, same length as `path`. Anything else - a null pointer, a null
 *   `pruned_yaw`, a length that does not match - means no orientations.
 * @param pruned_yaw Receives the retained window's orientations, rotated into
 *   the BASE frame by this same transform. It is the SAME window from the SAME
 *   decision: the pruning rule is stated once, so an orientation cannot end up
 *   paired with a point the pruner did not keep. Always cleared first, so a
 *   caller reusing one buffer cannot mistake last tick's sequence for this
 *   tick's.
 */
/**
 * `progress` / `window_ahead`: where on the plan the robot was last found.
 * When both are given (window_ahead > 0), the nearest plan point is searched
 * only from `*progress` (less a short back-step) forward along `window_ahead`
 * metres of plan, and `*progress` is advanced to the point found. Without
 * them the search is global, which on a plan that comes back past itself
 * (the indoor3 lap's outbound and return legs run 0.5-0.9 m apart) lets a
 * robot beside the later leg be taken for one that is nearly finished: on
 * 2026-10-12 a NavigateThroughPoses lap was cut straight to its last pose.
 * A caller resets `*progress` to 0 whenever it is handed a new plan.
 */
std::vector<Point2D> transformAndPrunePath(const std::vector<Point2D> &path,
                                           float transform_x, float transform_y,
                                           float transform_yaw, float max_range,
                                           const std::vector<float> *plan_yaw = nullptr,
                                           std::vector<float> *pruned_yaw = nullptr,
                                           std::size_t *progress = nullptr,
                                           float window_ahead = 0.0f);

}  // namespace bac

#endif  // BILATERAL_ARC_CLEARANCE_CONTROLLER__ADAPTER_UTILS_HPP_
