/**
 * @file sim_world.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief 2D world model and LiDAR simulator for the BAC scenario harness
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#pragma once
#ifndef BAC_SIM_SIM_WORLD_HPP__
#define BAC_SIM_SIM_WORLD_HPP__

#include <cmath>
#include <vector>
#include <algorithm>
#include "bilateral_arc_clearance_controller/bac_core.hpp"

namespace bac_sim
{

constexpr float kPi = 3.14159265358979323846f;

struct Pose
{
  float x  = 0.0f;
  float y  = 0.0f;
  float th = 0.0f;
};

struct Segment
{
  float x1, y1, x2, y2;
};

/// Optional corridor-centering window: lateral error is aggregated only while
/// the vehicle is between x_from and x_to, i.e. inside the corridor proper.
/// Shared because the holonomic and Ackermann rigs aggregate lateral error the
/// same way even though their kinematics differ; the rigs themselves are NOT
/// shared and are not meant to be.
struct LateralWindow
{
  bool enabled = false;
  float center_y = 0.0f;
  float x_from = 0.0f;
  float x_to = 0.0f;
};

struct World
{
  std::vector<Segment> walls;

  void addWall(float x1, float y1, float x2, float y2)
  {
    walls.push_back({ x1, y1, x2, y2 });
  }

  // Corridor along +x: two parallel walls from x_start to x_end, centered on y_center
  void addCorridorX(float x_start, float x_end, float y_center, float width)
  {
    addWall(x_start, y_center + width / 2.0f, x_end, y_center + width / 2.0f);
    addWall(x_start, y_center - width / 2.0f, x_end, y_center - width / 2.0f);
  }

  /// A corridor whose walls have THICKNESS, built from closed boxes rather than
  /// zero-thickness segments. `gap` is the free space between the inner faces.
  ///
  /// The difference matters at the mouth. A ray cast almost along a
  /// zero-thickness segment can miss it entirely between beams, so a body
  /// entering nearly parallel to such a wall may not see the very edge it is
  /// about to touch - not a controller failure but an artefact of modelling a
  /// wall as a line. Real walls have thickness and present a face. Scenarios
  /// that drive along a wall should use this.
  void addCorridorXWalls(float x_start, float x_end, float y_center, float gap,
                         float thickness)
  {
    const float half = gap / 2.0f;
    const float length = x_end - x_start;
    const float centre = (x_start + x_end) / 2.0f;
    addBox(centre, y_center + half + thickness / 2.0f, length, thickness);
    addBox(centre, y_center - half - thickness / 2.0f, length, thickness);
  }

  // Axis-aligned box obstacle
  void addBox(float cx, float cy, float w, float h)
  {
    float hw = w / 2.0f, hh = h / 2.0f;
    addWall(cx - hw, cy - hh, cx + hw, cy - hh);
    addWall(cx + hw, cy - hh, cx + hw, cy + hh);
    addWall(cx + hw, cy + hh, cx - hw, cy + hh);
    addWall(cx - hw, cy + hh, cx - hw, cy - hh);
  }
};

// Ray (origin, unit dir) vs segment intersection. Returns distance along ray, or -1 if no hit.
inline float
raySegmentDistance(float ox, float oy, float dx, float dy, const Segment &seg)
{
  float ex = seg.x2 - seg.x1;
  float ey = seg.y2 - seg.y1;
  float denom = dx * ey - dy * ex;
  if (std::fabs(denom) < 1e-9f)
  {
    return -1.0f;  // parallel
  }
  float sx = seg.x1 - ox;
  float sy = seg.y1 - oy;
  float t = (sx * ey - sy * ex) / denom;          // distance along ray
  float u = (sx * dy - sy * dx) / denom;          // position on segment
  if (t >= 0.0f && u >= -1e-6f && u <= 1.0f + 1e-6f)
  {
    return t;
  }
  return -1.0f;
}

/**
 * @brief Simulate a 2D LiDAR scan; returns hit points in the ROBOT frame.
 */
inline std::vector<bac::Point2D>
simulateLidar(const World &world, const Pose &pose, int num_beams = 720, float max_range = 10.0f)
{
  std::vector<bac::Point2D> points;
  points.reserve(num_beams);
  for (int i = 0; i < num_beams; i++)
  {
    float a_local  = -kPi + 2.0f * kPi * i / num_beams;
    float a_global = pose.th + a_local;
    float dx = std::cos(a_global);
    float dy = std::sin(a_global);

    float nearest = -1.0f;
    for (const Segment &seg : world.walls)
    {
      float t = raySegmentDistance(pose.x, pose.y, dx, dy, seg);
      if (t >= 0.0f && (nearest < 0.0f || t < nearest))
      {
        nearest = t;
      }
    }
    if (nearest >= 0.0f && nearest <= max_range)
    {
      points.emplace_back(nearest * std::cos(a_local), nearest * std::sin(a_local));
    }
  }
  return points;
}

// ---- geometry helpers for clearance / collision metrics ----

inline float
pointSegmentDistance(float px, float py, const Segment &seg)
{
  float ex = seg.x2 - seg.x1;
  float ey = seg.y2 - seg.y1;
  float len2 = ex * ex + ey * ey;
  float t = 0.0f;
  if (len2 > 1e-12f)
  {
    t = ((px - seg.x1) * ex + (py - seg.y1) * ey) / len2;
    t = std::max(0.0f, std::min(1.0f, t));
  }
  float cx = seg.x1 + t * ex;
  float cy = seg.y1 + t * ey;
  return std::sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
}

inline bool
segmentsIntersect(const Segment &a, const Segment &b)
{
  auto cross = [](float ax, float ay, float bx, float by) { return ax * by - ay * bx; };
  float d1x = a.x2 - a.x1, d1y = a.y2 - a.y1;
  float d2x = b.x2 - b.x1, d2y = b.y2 - b.y1;
  float denom = cross(d1x, d1y, d2x, d2y);
  float sx = b.x1 - a.x1, sy = b.y1 - a.y1;
  if (std::fabs(denom) < 1e-12f)
  {
    return false;  // parallel: treated as non-intersecting (distance check covers touching)
  }
  float t = cross(sx, sy, d2x, d2y) / denom;
  float u = cross(sx, sy, d1x, d1y) / denom;
  return t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f;
}

inline float
segmentSegmentDistance(const Segment &a, const Segment &b)
{
  if (segmentsIntersect(a, b))
  {
    return 0.0f;
  }
  float d = pointSegmentDistance(a.x1, a.y1, b);
  d = std::min(d, pointSegmentDistance(a.x2, a.y2, b));
  d = std::min(d, pointSegmentDistance(b.x1, b.y1, a));
  d = std::min(d, pointSegmentDistance(b.x2, b.y2, a));
  return d;
}

/**
 * @brief Robot body outline (physical size, no margins) as 4 segments in the world frame.
 */
inline std::vector<Segment>
robotOutline(const Pose &pose, const bac::Footprint &size)
{
  float hw = size.width / 2.0f;
  float c = std::cos(pose.th), s = std::sin(pose.th);
  auto tf = [&](float lx, float ly, float &wx, float &wy) {
    wx = pose.x + c * lx - s * ly;
    wy = pose.y + s * lx + c * ly;
  };
  float x[4], y[4];
  tf(size.front, -hw, x[0], y[0]);
  tf(size.front, hw, x[1], y[1]);
  tf(size.rear, hw, x[2], y[2]);
  tf(size.rear, -hw, x[3], y[3]);
  std::vector<Segment> outline;
  for (int i = 0; i < 4; i++)
  {
    int j = (i + 1) % 4;
    outline.push_back({ x[i], y[i], x[j], y[j] });
  }
  return outline;
}

/**
 * @brief Minimum distance between the robot body and any wall. 0 means contact/collision.
 */
inline float
robotClearance(const Pose &pose, const bac::Footprint &size, const World &world)
{
  auto outline = robotOutline(pose, size);
  float min_d  = 1e9f;
  for (const Segment &wall : world.walls)
  {
    for (const Segment &edge : outline)
    {
      min_d = std::min(min_d, segmentSegmentDistance(edge, wall));
    }
  }
  return min_d;
}

}  // namespace bac_sim

#endif  // BAC_SIM_SIM_WORLD_HPP__
