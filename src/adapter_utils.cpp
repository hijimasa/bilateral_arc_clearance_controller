/**
 * @file adapter_utils.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief ROS-independent geometry shared by the BAC ROS adapters
 * @date 2026-08-29
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "bilateral_arc_clearance_controller/adapter_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bac
{

ScanProjection
projectScan(const std::vector<float> &ranges, float angle_min, float angle_increment,
            float range_min, float range_max, float obstacle_range_max, int downsample,
            bool inf_is_valid, float sensor_x, float sensor_y, float sensor_yaw)
{
  ScanProjection result;
  const std::size_t stride = static_cast<std::size_t>(std::max(1, downsample));
  result.points.reserve((ranges.size() + stride - 1U) / stride);

  const float cs = std::cos(sensor_yaw);
  const float sn = std::sin(sensor_yaw);
  for (std::size_t i = 0; i < ranges.size(); i += stride)
  {
    const float range = ranges[i];
    const bool clear_ray =
        inf_is_valid && ((std::isinf(range) && range > 0.0f) ||
                         (std::isfinite(range) && range > range_max));
    const bool hit_ray =
        std::isfinite(range) && range >= range_min && range <= range_max;
    if (clear_ray || hit_ray)
    {
      ++result.valid_ray_count;
    }
    if (!hit_ray || range > obstacle_range_max)
    {
      continue;
    }

    const float angle = angle_min + static_cast<float>(i) * angle_increment;
    const float lx = range * std::cos(angle);
    const float ly = range * std::sin(angle);
    result.points.emplace_back(sensor_x + cs * lx - sn * ly,
                               sensor_y + sn * lx + cs * ly);
  }
  return result;
}

std::vector<Point2D>
transformAndPrunePath(const std::vector<Point2D> &path, float transform_x,
                      float transform_y, float transform_yaw, float max_range)
{
  if (path.empty() || !(max_range > 0.0f))
  {
    return {};
  }

  const float cs = std::cos(transform_yaw);
  const float sn = std::sin(transform_yaw);
  std::vector<Point2D> transformed;
  transformed.reserve(path.size());
  for (const Point2D &point : path)
  {
    transformed.emplace_back(transform_x + cs * point.x - sn * point.y,
                             transform_y + sn * point.x + cs * point.y);
  }

  std::size_t nearest = 0;
  float best_distance_squared = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < transformed.size(); ++i)
  {
    const float distance_squared =
        transformed[i].x * transformed[i].x + transformed[i].y * transformed[i].y;
    if (distance_squared < best_distance_squared)
    {
      best_distance_squared = distance_squared;
      nearest = i;
    }
  }

  std::vector<Point2D> local_path;
  local_path.reserve(transformed.size() - nearest);
  const float max_distance_squared = max_range * max_range;
  for (std::size_t i = nearest; i < transformed.size(); ++i)
  {
    const float distance_squared =
        transformed[i].x * transformed[i].x + transformed[i].y * transformed[i].y;
    if (distance_squared > max_distance_squared)
    {
      break;
    }
    local_path.push_back(transformed[i]);
  }
  return local_path;
}

}  // namespace bac
