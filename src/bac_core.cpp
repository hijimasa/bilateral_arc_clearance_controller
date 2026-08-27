/**
 * @file bac_core.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Framework-free core of the arc-clearance local planner (DWA-based)
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 REACT Co., Ltd.
 *
 * Algorithm summary (details and rationale in the package README):
 *  1. Emergency stop if any point is inside the body + braking distance zone,
 *     inflated by the safety margins with rounded (elliptical) corners.
 *  2. Sample (v, w) candidates inside the accel-limited dynamic window
 *     (plus a v=0 rotation row and the stop candidate), roll each out as a
 *     constant-curvature arc over sim_time.
 *  3. Admissibility: discard candidates that cannot stop before the first
 *     body hit on their arc (and rotations that sweep into an obstacle).
 *  4. Score = saturated bilateral clearance + path following (distance /
 *     progress / heading) - hysteresis - lateral squeeze; output the best.
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

float
wrapAngle(float a)
{
  while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
  while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
  return a;
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
      float alpha0 = (turn_radius >= 0.0f) ? -static_cast<float>(M_PI) / 2.0f : static_cast<float>(M_PI) / 2.0f;
      float delta  = alpha - alpha0;
      while (delta > static_cast<float>(M_PI)) delta -= 2.0f * static_cast<float>(M_PI);
      while (delta <= -static_cast<float>(M_PI)) delta += 2.0f * static_cast<float>(M_PI);
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

  // Pre-filter points once (range and ignore-box filters)
  std::vector<Point2D> filtered_points;
  filtered_points.reserve(points.size());
  for (const Point2D &point : points)
  {
    if (std::sqrt(point.x * point.x + point.y * point.y) > params_.max_range)
    {
      continue;  // ignore points too far
    }
    if (point.x < params_.ignore_box.front && point.x > -params_.ignore_box.back &&
        std::fabs(point.y) < params_.ignore_box.width / 2.0f)
    {
      continue;  // ignore area
    }
    filtered_points.push_back(point);
  }
  if (params_.max_points > 0 && static_cast<int>(filtered_points.size()) > params_.max_points)
  {
    // Uniform stride subsampling keeps the angular coverage of a scan
    std::vector<Point2D> reduced;
    reduced.reserve(params_.max_points);
    float stride = static_cast<float>(filtered_points.size()) / params_.max_points;
    for (int i = 0; i < params_.max_points; i++)
    {
      reduced.push_back(filtered_points[static_cast<size_t>(i * stride)]);
    }
    filtered_points.swap(reduced);
  }

  // Immediate-stop check: emergency zone = body rectangle (extended in the
  // CURRENT direction of travel by the braking distance at the current speed)
  // inflated by the safety margins with ROUNDED corners (elliptical, since the
  // margins are anisotropic). A plain margin rectangle would over-cover the
  // corners by sqrt(2) and dead-lock the robot on diagonal clearances that are
  // actually larger than the margin.
  // The same loop computes the plain-footprint nearest distance (influence).
  float min_proximity_norm = FLT_MAX;  // margin-normalized (1.0 = at the boundary)
  float nearest_distance   = FLT_MAX;  // Euclidean distance to the footprint rectangle
  bool  emergency          = false;
  {
    float stop_decel = std::max(params_.stop_decel, 0.1f);
    float brake_dist =
        current.v * current.v / (2.0f * stop_decel) + std::fabs(current.v) * params_.brake_reaction_time;
    float body_x_min = params_.footprint.rear;
    float body_x_max = params_.footprint.front;
    float zone_x_min = body_x_min;
    float zone_x_max = body_x_max;
    if (current.v >= 0.0f)
    {
      zone_x_max += brake_dist;
    }
    else
    {
      zone_x_min -= brake_dist;
    }
    float body_y_half  = params_.footprint.width / 2.0f;
    float speed_scale  = std::min(
        1.0f, params_.margin_scale_floor +
                  (1.0f - params_.margin_scale_floor) *
                      std::fabs(current.v) / std::max(params_.margin_scale_speed, 1e-3f));
    float margin_front = std::max(params_.safety_margin.front * speed_scale, 1e-3f);
    float margin_rear  = std::max(params_.safety_margin.rear * speed_scale, 1e-3f);
    float margin_side  = std::max(params_.safety_margin.side * speed_scale, 1e-3f);

    for (const Point2D &point : filtered_points)
    {
      float dx_norm = 0.0f;
      if (point.x > zone_x_max)
      {
        dx_norm = (point.x - zone_x_max) / margin_front;
      }
      else if (point.x < zone_x_min)
      {
        dx_norm = (zone_x_min - point.x) / margin_rear;
      }
      float dy_norm = 0.0f;
      if (std::fabs(point.y) > body_y_half)
      {
        dy_norm = (std::fabs(point.y) - body_y_half) / margin_side;
      }
      float norm         = std::sqrt(dx_norm * dx_norm + dy_norm * dy_norm);
      min_proximity_norm = std::min(min_proximity_norm, norm);
      if (norm < 1.0f)
      {
        emergency = true;
      }

      float ex = std::max(std::max(point.x - body_x_max, body_x_min - point.x), 0.0f);
      float ey = std::max(std::fabs(point.y) - body_y_half, 0.0f);
      nearest_distance = std::min(nearest_distance, std::sqrt(ex * ex + ey * ey));
    }
  }
  result.min_proximity_norm = min_proximity_norm;
  result.nearest_distance   = nearest_distance;

  if (emergency)
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
    current_status_ = (nearest_distance > params_.influence_range) ? Status::CLEAR : Status::AVOIDING;
    result.status   = current_status_;
    return result;
  }

  // Prune the path behind the robot: start at the point nearest to the origin.
  size_t start_index = 0;
  {
    float best = FLT_MAX;
    for (size_t i = 0; i < path.size(); i++)
    {
      float d = path[i].x * path[i].x + path[i].y * path[i].y;
      if (d < best)
      {
        best        = d;
        start_index = i;
      }
    }
  }
  std::vector<Point2D> local_path(path.begin() + static_cast<long>(start_index), path.end());
  std::vector<float>   cumulative(local_path.size(), 0.0f);
  for (size_t i = 1; i < local_path.size(); i++)
  {
    float dx      = local_path[i].x - local_path[i - 1].x;
    float dy      = local_path[i].y - local_path[i - 1].y;
    cumulative[i] = cumulative[i - 1] + std::sqrt(dx * dx + dy * dy);
  }

  // Local goal: the path point score_lookahead along the path, advanced past
  // points an obstacle sits on (the body cannot be centered there; pulling
  // towards them walks the robot into the obstacle).
  float half_width_goal = params_.footprint.width / 2.0f;
  size_t goal_index = local_path.size() - 1;
  for (size_t i = 0; i < local_path.size(); i++)
  {
    if (cumulative[i] >= params_.score_lookahead)
    {
      goal_index = i;
      break;
    }
  }
  for (size_t i = goal_index; i < local_path.size(); i++)
  {
    bool blocked = false;
    for (const Point2D &point : filtered_points)
    {
      float dx = point.x - local_path[i].x;
      float dy = point.y - local_path[i].y;
      if (dx * dx + dy * dy < half_width_goal * half_width_goal)
      {
        blocked = true;
        break;
      }
    }
    if (!blocked)
    {
      goal_index = i;
      break;
    }
    goal_index = i;  // all blocked to the end: use the last examined
  }
  // Line-of-sight trim: pull the goal back to the farthest path point whose
  // straight segment from the robot stays goal_los_radius away from every
  // obstacle point (see Params::goal_los_radius).
  if (params_.goal_los_radius > 1e-3f)
  {
    // Obstacle points sitting ON the path (degraded plan) are the swerve
    // logic's business and must not block the line of sight - otherwise the
    // goal collapses to the obstacle's face and propulsion dies. Only
    // off-path blockers (walls, corners) trim the goal.
    std::vector<bool> on_path(filtered_points.size(), false);
    float near2 = params_.los_onpath_radius * params_.los_onpath_radius;
    for (size_t pi = 0; pi < filtered_points.size(); pi++)
    {
      const Point2D &p = filtered_points[pi];
      for (const Point2D &q : local_path)
      {
        float dx = p.x - q.x, dy = p.y - q.y;
        if (dx * dx + dy * dy < near2)
        {
          on_path[pi] = true;
          break;
        }
      }
    }
    auto los_clear = [&](const Point2D &g) {
      float len2 = g.x * g.x + g.y * g.y;
      for (size_t pi = 0; pi < filtered_points.size(); pi++)
      {
        if (on_path[pi])
        {
          continue;
        }
        const Point2D &p = filtered_points[pi];
        float t = 0.0f;
        if (len2 > 1e-9f)
        {
          t = std::max(0.0f, std::min(1.0f, (p.x * g.x + p.y * g.y) / len2));
        }
        float dx = p.x - t * g.x, dy = p.y - t * g.y;
        if (dx * dx + dy * dy < params_.goal_los_radius * params_.goal_los_radius)
        {
          return false;
        }
      }
      return true;
    };
    while (goal_index > 0 && !los_clear(local_path[goal_index]))
    {
      goal_index--;
    }
  }

  float goal_x = local_path[goal_index].x;
  float goal_y = local_path[goal_index].y;
  result.goal_x = goal_x;
  result.goal_y = goal_y;

  // Proximity speed governor: cap the sampled speed near obstacles (with a
  // creep floor so an escape stays possible right at the boundary).
  float v_cap = params_.limits.v_max;
  if (min_proximity_norm < params_.proximity_governor_range && params_.proximity_governor_range > 1.0f)
  {
    float fraction = (min_proximity_norm - 1.0f) / (params_.proximity_governor_range - 1.0f);
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

  if (nearest_distance > params_.influence_range)
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
