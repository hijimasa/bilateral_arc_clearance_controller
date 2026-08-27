/**
 * @file bac_core.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Framework-free core of the arc-clearance local planner (DWA-based)
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Algorithm summary (details and rationale in the package README):
 *  1. Emergency stop if any point is inside the body + braking distance zone,
 *     inflated by the safety margins with rounded (elliptical) corners.
 *  2. Sample v inside the acceleration-limited translational window and w
 *     over the configured angular range (plus a v=0 rotation row and the stop
 *     candidate), then roll each out as a constant-curvature arc.
 *  3. Admissibility: discard candidates that cannot stop before the first
 *     body hit on their arc (and rotations that sweep into an obstacle).
 *  4. Score = saturated bilateral clearance + local-goal following (distance /
 *     heading) - hysteresis - lateral squeeze; output the best.
 */

#include "bilateral_arc_clearance_controller/bac_core.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#ifdef BAC_DEBUG_CANDIDATES
#include <cstdio>
#endif

namespace bac
{

namespace
{

constexpr float kPi = 3.14159265358979323846f;

float
wrapAngle(float a)
{
  while (a > kPi) a -= 2.0f * kPi;
  while (a < -kPi) a += 2.0f * kPi;
  return a;
}

std::vector<Point2D>
filterObstaclePoints(const std::vector<Point2D> &points, const Params &params)
{
  std::vector<Point2D> filtered;
  filtered.reserve(points.size());
  for (const Point2D &point : points)
  {
    if (std::sqrt(point.x * point.x + point.y * point.y) > params.max_range)
    {
      continue;
    }
    if (point.x < params.ignore_box.front && point.x > -params.ignore_box.back &&
        std::fabs(point.y) < params.ignore_box.width / 2.0f)
    {
      continue;
    }
    filtered.push_back(point);
  }

  if (params.max_points <= 0 || static_cast<int>(filtered.size()) <= params.max_points)
  {
    return filtered;
  }

  // Uniform stride subsampling preserves the angular coverage of an ordered
  // laser scan while placing a hard bound on point x candidate work.
  std::vector<Point2D> reduced;
  reduced.reserve(static_cast<size_t>(params.max_points));
  const float stride = static_cast<float>(filtered.size()) / params.max_points;
  for (int i = 0; i < params.max_points; ++i)
  {
    reduced.push_back(filtered[static_cast<size_t>(i * stride)]);
  }
  return reduced;
}

struct ProximityResult
{
  float normalized = FLT_MAX;  // 1.0 = emergency margin boundary
  float distance   = FLT_MAX;  // Euclidean distance to physical footprint
  bool  emergency  = false;
};

ProximityResult
evaluateProximity(const std::vector<Point2D> &points, const Params &params, const Twist2D &current)
{
  ProximityResult result;

  const float stop_decel = std::max(params.stop_decel, 0.1f);
  const float brake_distance = current.v * current.v / (2.0f * stop_decel) +
                               std::fabs(current.v) * params.brake_reaction_time;
  const float body_x_min = params.footprint.rear;
  const float body_x_max = params.footprint.front;
  float       zone_x_min = body_x_min;
  float       zone_x_max = body_x_max;
  if (current.v >= 0.0f)
  {
    zone_x_max += brake_distance;
  }
  else
  {
    zone_x_min -= brake_distance;
  }

  const float body_y_half = params.footprint.width / 2.0f;
  const float speed_scale = std::min(
      1.0f, params.margin_scale_floor +
                (1.0f - params.margin_scale_floor) * std::fabs(current.v) /
                    std::max(params.margin_scale_speed, 1e-3f));
  const float margin_front = std::max(params.safety_margin.front * speed_scale, 1e-3f);
  const float margin_rear  = std::max(params.safety_margin.rear * speed_scale, 1e-3f);
  const float margin_side  = std::max(params.safety_margin.side * speed_scale, 1e-3f);

  for (const Point2D &point : points)
  {
    float dx_normalized = 0.0f;
    if (point.x > zone_x_max)
    {
      dx_normalized = (point.x - zone_x_max) / margin_front;
    }
    else if (point.x < zone_x_min)
    {
      dx_normalized = (zone_x_min - point.x) / margin_rear;
    }

    float dy_normalized = 0.0f;
    if (std::fabs(point.y) > body_y_half)
    {
      dy_normalized = (std::fabs(point.y) - body_y_half) / margin_side;
    }
    const float normalized =
        std::sqrt(dx_normalized * dx_normalized + dy_normalized * dy_normalized);
    result.normalized      = std::min(result.normalized, normalized);
    result.emergency       = result.emergency || normalized < 1.0f;

    const float dx = std::max(std::max(point.x - body_x_max, body_x_min - point.x), 0.0f);
    const float dy = std::max(std::fabs(point.y) - body_y_half, 0.0f);
    result.distance = std::min(result.distance, std::sqrt(dx * dx + dy * dy));
  }
  return result;
}

Point2D
selectLocalGoal(const std::vector<Point2D> &path, const std::vector<Point2D> &obstacles,
                const Params &params)
{
  size_t start_index       = 0;
  float  nearest_path_dist = FLT_MAX;
  for (size_t i = 0; i < path.size(); ++i)
  {
    const float distance_squared = path[i].x * path[i].x + path[i].y * path[i].y;
    if (distance_squared < nearest_path_dist)
    {
      nearest_path_dist = distance_squared;
      start_index       = i;
    }
  }

  const auto first = path.begin() + static_cast<std::vector<Point2D>::difference_type>(start_index);
  const std::vector<Point2D> local_path(first, path.end());
  std::vector<float> cumulative(local_path.size(), 0.0f);
  for (size_t i = 1; i < local_path.size(); ++i)
  {
    const float dx = local_path[i].x - local_path[i - 1].x;
    const float dy = local_path[i].y - local_path[i - 1].y;
    cumulative[i]  = cumulative[i - 1] + std::sqrt(dx * dx + dy * dy);
  }

  size_t goal_index = local_path.size() - 1;
  for (size_t i = 0; i < local_path.size(); ++i)
  {
    if (cumulative[i] >= params.score_lookahead)
    {
      goal_index = i;
      break;
    }
  }

  // Do not attract the robot to a path pose occupied by an obstacle. Search
  // farther along the plan so that bilateral clearance can swerve around it.
  const float half_width          = params.footprint.width / 2.0f;
  const float body_radius_squared = half_width * half_width;
  for (size_t i = goal_index; i < local_path.size(); ++i)
  {
    bool blocked = false;
    for (const Point2D &point : obstacles)
    {
      const float dx = point.x - local_path[i].x;
      const float dy = point.y - local_path[i].y;
      if (dx * dx + dy * dy < body_radius_squared)
      {
        blocked = true;
        break;
      }
    }
    goal_index = i;
    if (!blocked)
    {
      break;
    }
  }

  if (params.goal_los_radius <= 1e-3f)
  {
    return local_path[goal_index];
  }

  // Points on the plan represent a degraded-plan obstacle handled by the
  // swerve score. Only off-path geometry (for example a corner wall) trims
  // the line-of-sight goal.
  std::vector<bool> on_path(obstacles.size(), false);
  const float on_path_radius_squared = params.los_onpath_radius * params.los_onpath_radius;
  for (size_t obstacle_index = 0; obstacle_index < obstacles.size(); ++obstacle_index)
  {
    for (const Point2D &path_point : local_path)
    {
      const float dx = obstacles[obstacle_index].x - path_point.x;
      const float dy = obstacles[obstacle_index].y - path_point.y;
      if (dx * dx + dy * dy < on_path_radius_squared)
      {
        on_path[obstacle_index] = true;
        break;
      }
    }
  }

  const float los_radius_squared = params.goal_los_radius * params.goal_los_radius;
  auto has_line_of_sight = [&](const Point2D &goal) {
    const float goal_distance_squared = goal.x * goal.x + goal.y * goal.y;
    for (size_t i = 0; i < obstacles.size(); ++i)
    {
      if (on_path[i])
      {
        continue;
      }
      float projection = 0.0f;
      if (goal_distance_squared > 1e-9f)
      {
        projection = std::max(
            0.0f, std::min(1.0f, (obstacles[i].x * goal.x + obstacles[i].y * goal.y) /
                                     goal_distance_squared));
      }
      const float dx = obstacles[i].x - projection * goal.x;
      const float dy = obstacles[i].y - projection * goal.y;
      if (dx * dx + dy * dy < los_radius_squared)
      {
        return false;
      }
    }
    return true;
  };

  while (goal_index > 0 && !has_line_of_sight(local_path[goal_index]))
  {
    --goal_index;
  }
  return local_path[goal_index];
}

}  // namespace

BacCore::BacCore()
  : BacCore(Params{})
{
}

BacCore::BacCore(const Params &params)
  : params_(params)
  , current_status_(Status::CLEAR)
  , avoiding_counter_(0)
  , prev_selected_w_(0.0f)
  , cap_ema_(-1.0f)
{
}

void
BacCore::setParams(const Params &params)
{
  params_ = params;
}

const Params &
BacCore::params() const
{
  return params_;
}

Status
BacCore::status() const
{
  return current_status_;
}

void
BacCore::forceStop()
{
  current_status_ = Status::STOP;
}

void
BacCore::reset()
{
  current_status_   = Status::CLEAR;
  avoiding_counter_ = 0;
  prev_selected_w_  = 0.0f;
  cap_ema_          = -1.0f;
}

ArcEvaluation
BacCore::evaluateArc(const std::vector<Point2D> &points, float v, float w, float horizon) const
{
  float dist = std::fabs(v) * horizon;
  return evaluateArcWindows(points, v, w, dist, dist);
}

ArcEvaluation
BacCore::evaluateArcWindows(const std::vector<Point2D> &points, float v, float w, float dist_clear,
                            float dist_block) const
{
  ArcEvaluation eval{ FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, 1.0f };

  const Footprint &body = params_.footprint;
  // Body extent in the direction of travel (rear is negative)
  float lead_length  = (v >= 0.0f) ? body.front : -body.rear;
  float trail_length = (v >= 0.0f) ? -body.rear : body.front;
  float half_width   = body.width / 2.0f;
  float side_margin  = std::max(params_.safety_margin.side, 1e-3f);
  float s_max_clear  = dist_clear + lead_length;
  float s_max        = std::max(dist_block, dist_clear) + lead_length;

  for (const Point2D &point : points)
  {
    float s;            // longitudinal distance along the path, in travel direction
    float left_offset;  // signed lateral offset from the path centerline (+ = left of travel)

    if (std::fabs(w) < 1e-4f)
    {
      float direction = (v >= 0.0f) ? 1.0f : -1.0f;
      s           = point.x * direction;
      left_offset = point.y * direction;
    }
    else
    {
      // Arc around center C = (0, R), R = v / w. Outside/inside of the circle
      // maps to travel-left/right via sgn(w) (holds for backward motion too).
      float turn_radius     = v / w;
      float abs_turn_radius = std::fabs(turn_radius);
      float radial = std::sqrt(point.x * point.x + (point.y - turn_radius) * (point.y - turn_radius));
      left_offset  = (abs_turn_radius - radial) * ((w > 0.0f) ? 1.0f : -1.0f);

      float alpha  = std::atan2(point.y - turn_radius, point.x);
      float alpha0 = (turn_radius >= 0.0f) ? -kPi / 2.0f : kPi / 2.0f;
      float delta  = alpha - alpha0;
      while (delta > kPi) delta -= 2.0f * kPi;
      while (delta <= -kPi) delta += 2.0f * kPi;
      s = std::fabs(v) * (delta / w);
    }

    if (s < -trail_length || s > s_max)
    {
      continue;  // outside the swept longitudinal window
    }

    float abs_offset = std::fabs(left_offset);
    if (abs_offset < half_width)
    {
      // The body would hit this point: longitudinal distance to the first hit
      // (searched over the full blocking window)
      if (s < eval.blocking_s)
      {
        eval.blocking_s = s;
      }
    }

    if (s > s_max_clear)
    {
      // Beyond the clearance window: a body-hit point still poisons the
      // clearance when it is euclidean-near (see Params::blocked_near/far).
      if (abs_offset < half_width)
      {
        float d_e  = std::sqrt(point.x * point.x + point.y * point.y);
        float fade = (d_e - params_.blocked_near) /
                     std::max(params_.blocked_far - params_.blocked_near, 1e-3f);
        fade = std::max(0.0f, std::min(1.0f, fade));
        if (fade < 1.0f)
        {
          float cap_ref = half_width + params_.avoid_margin.side;
          float pseudo  = abs_offset + fade * (cap_ref - abs_offset);
          if (left_offset >= 0.0f)
          {
            eval.clearance_left = std::min(eval.clearance_left, pseudo);
          }
          else
          {
            eval.clearance_right = std::min(eval.clearance_right, pseudo);
          }
        }
      }
      continue;  // clearance/squeeze aggregation uses the (angle-capped) window
    }

    if (left_offset >= 0.0f)
    {
      eval.clearance_left = std::min(eval.clearance_left, left_offset);
      if (s > lead_length)
      {
        eval.far_left = std::min(eval.far_left, left_offset);
      }
    }
    else
    {
      eval.clearance_right = std::min(eval.clearance_right, -left_offset);
      if (s > lead_length)
      {
        eval.far_right = std::min(eval.far_right, -left_offset);
      }
    }

    if (abs_offset >= half_width)
    {
      // Non-hitting squeeze: intrusion into the safety side margin
      float fraction = (abs_offset - half_width) / side_margin;
      if (fraction < eval.lateral_fraction)
      {
        eval.lateral_fraction = std::min(1.0f, fraction);
      }
    }
  }

  return eval;
}

Result
BacCore::process(const std::vector<Point2D> &points, const std::vector<Point2D> &path,
                 const Twist2D &current)
{
  Result result;

  const std::vector<Point2D> filtered_points = filterObstaclePoints(points, params_);

  // Immediate-stop check: emergency zone = body rectangle (extended in the
  // CURRENT direction of travel by the braking distance at the current speed)
  // inflated by the safety margins with ROUNDED corners (elliptical, since the
  // margins are anisotropic). A plain margin rectangle would over-cover the
  // corners by sqrt(2) and dead-lock the robot on diagonal clearances that are
  // actually larger than the margin.
  // The same pass computes the plain-footprint distance used for status.
  const ProximityResult proximity = evaluateProximity(filtered_points, params_, current);
  result.min_proximity_norm       = proximity.normalized;
  result.nearest_distance         = proximity.distance;

  if (proximity.emergency)
  {
    current_status_ = Status::STOP;
    result.output   = Twist2D(0.0f, 0.0f);
    result.status   = Status::STOP;
    return result;
  }

  // No intent: hold still. Influence-based status so the filter node's
  // pass-through arbitration still works while idling.
  if (path.empty())
  {
    result.output   = Twist2D(0.0f, 0.0f);
    current_status_ =
        (proximity.distance > params_.influence_range) ? Status::CLEAR : Status::AVOIDING;
    result.status   = current_status_;
    return result;
  }

  const Point2D local_goal = selectLocalGoal(path, filtered_points, params_);
  const float   goal_x     = local_goal.x;
  const float   goal_y     = local_goal.y;
  result.goal_x            = goal_x;
  result.goal_y            = goal_y;

  // Proximity speed governor: cap the sampled speed near obstacles (with a
  // creep floor so an escape stays possible right at the boundary).
  float v_cap = params_.limits.v_max;
  if (proximity.normalized < params_.proximity_governor_range &&
      params_.proximity_governor_range > 1.0f)
  {
    float fraction =
        (proximity.normalized - 1.0f) / (params_.proximity_governor_range - 1.0f);
    fraction       = std::max(params_.creep_fraction, std::min(1.0f, fraction));
    v_cap *= fraction;
  }

  // Dynamic window (accel-limited around the current velocity)
  float window_dv = params_.limits.acc_v * params_.window_time;
  float v_lo      = std::max(params_.limits.v_min, current.v - window_dv);
  float v_hi      = std::min(v_cap, current.v + window_dv);
  if (v_hi < v_lo)
  {
    v_lo = v_hi = std::max(params_.limits.v_min, v_hi);
  }
  float w_lo = -params_.limits.w_max;
  float w_hi = params_.limits.w_max;

  std::vector<float> v_values;
  v_values.push_back(0.0f);  // rotation / stop row
  int v_samples = std::max(2, params_.v_samples);
  for (int i = 0; i < v_samples; i++)
  {
    float v = v_lo + (v_hi - v_lo) * static_cast<float>(i) / static_cast<float>(v_samples - 1);
    if (v > 1e-3f)
    {
      v_values.push_back(v);
    }
  }
  // Escape reverse: offered only when reverse is accel-reachable (near
  // standstill), so it never competes with normal forward driving.
  float v_rev = std::max(params_.limits.v_min, current.v - window_dv);
  if (v_rev < -1e-3f)
  {
    v_values.push_back(v_rev);
    v_values.push_back(v_rev / 2.0f);
  }

  std::vector<float> w_values;
  int w_samples = std::max(3, params_.w_samples);
  for (int i = 0; i < w_samples; i++)
  {
    w_values.push_back(w_lo + (w_hi - w_lo) * static_cast<float>(i) / static_cast<float>(w_samples - 1));
  }
  if (w_lo < 0.0f && w_hi > 0.0f)
  {
    w_values.push_back(0.0f);  // always offer the straight arc
  }

  // Rotation admissibility: a full in-place rotation sweeps the disk of the
  // circumscribed radius. Conservative: forbid rotation candidates when any
  // point lies inside that disk (they are still allowed to stop).
  float circumscribed = std::sqrt(std::max(params_.footprint.front, -params_.footprint.rear) *
                                      std::max(params_.footprint.front, -params_.footprint.rear) +
                                  (params_.footprint.width / 2.0f) * (params_.footprint.width / 2.0f));
  bool rotation_admissible = true;
  for (const Point2D &point : filtered_points)
  {
    if (std::sqrt(point.x * point.x + point.y * point.y) < circumscribed + 0.02f)
    {
      rotation_admissible = false;
      break;
    }
  }

  // Reusable buffer for turn-then-go evaluation of the v=0 row
  std::vector<Point2D> rotated_points;

  float clearance_cap = params_.footprint.width / 2.0f + params_.avoid_margin.side;

  // Tightness probe: best RAW bilateral clearance over three probe arcs.
  // Near 0 in the open (balance term off, pure path following); towards 1 in
  // passages tighter than cap + safety side margin (geometry takes over the
  // lateral authority).
  float tightness  = 0.0f;
  float probe_best = 0.0f;
  {
    float v_probe    = std::max(v_cap, 0.1f);
    float probe_dist = std::max(v_probe * params_.sim_time, params_.min_eval_distance);
    for (float wp : { -0.4f, 0.0f, 0.4f })
    {
      float dist_clear = probe_dist;
      if (std::fabs(wp) > 1e-4f)
      {
        dist_clear = std::min(dist_clear, (v_probe / std::fabs(wp)) * params_.eval_angle_max);
      }
      ArcEvaluation eval = evaluateArcWindows(filtered_points, v_probe, wp, dist_clear, probe_dist);
      probe_best = std::max(probe_best, std::min(eval.clearance_left, eval.clearance_right));
    }
    float cap_floor_probe = params_.footprint.width / 2.0f + params_.safety_margin.side;
    float probe_clamped   = std::max(cap_floor_probe, std::min(probe_best, clearance_cap));
    if (params_.cap_adapt_rate > 1e-6f)
    {
      if (cap_ema_ < 0.0f)
      {
        cap_ema_ = clearance_cap;
      }
      cap_ema_ += params_.cap_adapt_rate * (probe_clamped - cap_ema_);
    }
    else
    {
      cap_ema_ = clearance_cap;
    }
    float reference = cap_ema_ + params_.safety_margin.side;
    tightness = 1.0f - std::max(0.0f, std::min(1.0f, probe_best / std::max(reference, 1e-3f)));
  }

  // Adaptive saturation: aim for the configured avoid margin, but never DEMAND
  // more clearance than the passage towards the goal physically affords -
  // otherwise a corridor narrower than the cap scores permanently worse than
  // hesitating outside it (entry barrier).
  float cap_floor = params_.footprint.width / 2.0f + std::max(params_.safety_margin.side, 0.05f);
  float cap_eff   = std::min(clearance_cap, std::max(probe_best, cap_floor));

  float lead_length   = params_.footprint.front;
  float lead_margin   = params_.safety_margin.front;
  float stop_decel    = std::max(params_.stop_decel, 0.1f);

  float best_score = -FLT_MAX;
  Twist2D best_cmd(0.0f, 0.0f);
  float best_clearance = 0.0f, best_goal_dist = 0.0f;
  int   admissible_count = 0, candidate_count = 0;
  int   forward_admissible = 0;  // gates the escape-reverse row

  for (float v : v_values)
  {
    for (float w : w_values)
    {
      candidate_count++;
      if (v < -1e-3f && forward_admissible > 0)
      {
        continue;  // reverse is an escape move: only when forward is hopeless
      }

      // Endpoint pose and the point-free score parts first: they give an
      // upper bound (clearance <= cap_eff, balance/squeeze >= 0) that lets us
      // skip the expensive arc evaluation for candidates that cannot win.
      float end_th_pre = w * params_.sim_time;
      float end_x_pre, end_y_pre;
      if (std::fabs(w) < 1e-4f)
      {
        end_x_pre = v * params_.sim_time;
        end_y_pre = 0.0f;
      }
      else
      {
        float radius = v / w;
        end_x_pre    = radius * std::sin(end_th_pre);
        end_y_pre    = radius * (1.0f - std::cos(end_th_pre));
      }
      float gdx_pre = goal_x - end_x_pre, gdy_pre = goal_y - end_y_pre;
      float gd_pre  = std::sqrt(gdx_pre * gdx_pre + gdy_pre * gdy_pre);
      float fixed_penalties = params_.weights.goal_dist * gd_pre +
                              params_.weights.hysteresis * std::fabs(w - prev_selected_w_);
      // (Pruning waits until one admissible forward candidate is on record:
      // the escape-reverse gate depends on that count, and admissibility is
      // only known after evaluation.)
      if (forward_admissible > 0 &&
          params_.weights.clearance * cap_eff - fixed_penalties <= best_score)
      {
        continue;
      }

      float clearance, lateral_fraction, clear_left, clear_right;
      if (std::fabs(v) <= 1e-3f)
      {
        if (std::fabs(w) > 1e-3f && !rotation_admissible)
        {
          continue;
        }
        // Turn-then-go: score the rotation (or stop) by the straight run the
        // robot could make AFTER turning by w * sim_time. A myopic "clearance
        // beside the body" would make stopping/rotating look artificially
        // clean right in front of a blocked passage.
        float dth = w * params_.sim_time;
        float cs = std::cos(-dth), sn = std::sin(-dth);
        rotated_points.clear();
        rotated_points.reserve(filtered_points.size());
        for (const Point2D &p : filtered_points)
        {
          rotated_points.emplace_back(cs * p.x - sn * p.y, sn * p.x + cs * p.y);
        }
        float v_ref  = std::max(v_cap, 0.05f);
        float dist   = std::max(v_ref * params_.sim_time, params_.min_eval_distance);
        ArcEvaluation eval = evaluateArcWindows(rotated_points, v_ref, 0.0f, dist, dist);
        clearance        = std::min(eval.clearance_left, eval.clearance_right);
        lateral_fraction = eval.lateral_fraction;
        clear_left       = eval.far_left;
        clear_right      = eval.far_right;
      }
      else
      {
        if (std::fabs(w) > 1e-4f && std::fabs(v) / std::fabs(w) < params_.turn_radius_min)
        {
          continue;  // near-spin arc: degenerate clearance geometry
        }
        float dist_block = std::max(std::fabs(v) * params_.sim_time, params_.min_eval_distance);
        float dist_clear = dist_block;
        if (std::fabs(w) > 1e-4f)
        {
          float radius = std::fabs(v) / std::fabs(w);
          dist_clear   = std::min(dist_clear, radius * params_.eval_angle_max);
          if (params_.eval_lateral_max < radius)
          {
            dist_clear = std::min(
                dist_clear, radius * std::acos(1.0f - params_.eval_lateral_max / radius));
          }
        }
        ArcEvaluation eval = evaluateArcWindows(filtered_points, v, w, dist_clear, dist_block);
        if (eval.blocking_s < FLT_MAX)
        {
          // DWA admissibility: able to stop (plus the leading margin in the
          // direction of travel) before the hit
          float lead     = (v >= 0.0f) ? lead_length : -params_.footprint.rear;
          float margin   = (v >= 0.0f) ? lead_margin : params_.safety_margin.rear;
          float free_run = eval.blocking_s - lead - margin;
          float needed   = v * v / (2.0f * stop_decel) + std::fabs(v) * params_.brake_reaction_time;
          if (free_run <= needed)
          {
            continue;
          }
        }
        clearance        = std::min(eval.clearance_left, eval.clearance_right);
        lateral_fraction = eval.lateral_fraction;
        clear_left       = eval.far_left;
        clear_right      = eval.far_right;
      }
      admissible_count++;
      if (v > 1e-3f)
      {
        forward_admissible++;
      }

      // Bilateral balance over the FORWARD part of the arc (capped so wide
      // spaces zero out): first-order centering gradient towards equal
      // clearance on both sides of where the arc leads.
      // Balance applies to PASSAGES only (both sides bounded below the cap):
      // with one side open it would act as a plain wall-repulsion field and
      // over-steer around isolated obstacles. It keeps the PARAMETER cap -
      // the adaptive cap would saturate both sides at the worse side's level
      // and zero the centering out.
      float balance = (clear_left < clearance_cap && clear_right < clearance_cap)
                          ? std::fabs(clear_left - clear_right)
                          : 0.0f;

      // Rollout endpoint after sim_time (precomputed above)
      float end_th     = end_th_pre;
      float goal_dist  = gd_pre;
      float bearing    = std::atan2(gdy_pre, gdx_pre);
      float heading_err = wrapAngle(bearing - end_th);

      float score = params_.weights.clearance * std::min(clearance, cap_eff) -
                    params_.weights.balance * tightness * balance -
                    params_.weights.goal_dist * goal_dist -
                    params_.weights.heading * std::fabs(heading_err) -
                    params_.weights.hysteresis * std::fabs(w - prev_selected_w_) -
                    params_.weights.squeeze * std::fabs(v) * (1.0f - lateral_fraction);
#ifdef BAC_DEBUG_CANDIDATES
      std::printf("cand v=%.3f w=%6.3f clr=%6.3f gd=%6.3f he=%6.3f lat=%.2f score=%7.3f\n", v, w,
                  std::min(clearance, clearance_cap), goal_dist, heading_err, lateral_fraction, score);
#endif
      if (score > best_score)
      {
        best_score     = score;
        best_cmd       = Twist2D(v, w);
        best_clearance = std::min(clearance, cap_eff);
        best_goal_dist = goal_dist;
      }
    }
  }

  float out_v = best_cmd.v;
  float out_w = best_cmd.w;
  if (std::fabs(out_v) < params_.velocity_min)
  {
    out_v = 0.0f;
  }
  if (std::fabs(out_w) < params_.angvel_min)
  {
    out_w = 0.0f;
  }
  prev_selected_w_      = out_w;
  result.output         = Twist2D(out_v, out_w);
  result.best_clearance = best_clearance;
  result.best_goal_dist = best_goal_dist;
  result.admissible_count = admissible_count;
  result.candidate_count  = candidate_count;

  if (out_v == 0.0f && out_w == 0.0f)
  {
    // Intent exists but the best move is to hold still: blocked.
    current_status_ = Status::STOP;
    result.status   = Status::STOP;
    return result;
  }

  if (proximity.distance > params_.influence_range)
  {
    if (avoiding_counter_ > 0)
    {
      avoiding_counter_--;
      current_status_ = Status::AVOIDING;
    }
    else
    {
      current_status_ = Status::CLEAR;
    }
  }
  else
  {
    current_status_   = Status::AVOIDING;
    avoiding_counter_ = params_.avoiding_latch_ticks;
  }
  result.status = current_status_;
  return result;
}

}  // namespace bac
