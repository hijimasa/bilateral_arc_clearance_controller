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
 * @brief Apply a 2D rigid transform, prune before the robot-nearest point,
 * and retain the following contiguous path window within max_range.
 */
std::vector<Point2D> transformAndPrunePath(const std::vector<Point2D> &path,
                                           float transform_x, float transform_y,
                                           float transform_yaw, float max_range);

}  // namespace bac

#endif  // BILATERAL_ARC_CLEARANCE_CONTROLLER__ADAPTER_UTILS_HPP_
