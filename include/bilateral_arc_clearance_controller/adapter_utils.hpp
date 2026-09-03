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
std::vector<Point2D> transformAndPrunePath(const std::vector<Point2D> &path,
                                           float transform_x, float transform_y,
                                           float transform_yaw, float max_range,
                                           const std::vector<float> *plan_yaw = nullptr,
                                           std::vector<float> *pruned_yaw = nullptr);

}  // namespace bac

#endif  // BILATERAL_ARC_CLEARANCE_CONTROLLER__ADAPTER_UTILS_HPP_
