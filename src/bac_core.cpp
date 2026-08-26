/**
 * @file bac_core.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Framework-free core of the arc-clearance obstacle avoidance algorithm
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 REACT Co., Ltd.
 *
 * Algorithm summary (details and rationale in the package README):
 *  1. Emergency stop if any point is inside the body + braking distance zone,
 *     inflated by the safety margins with rounded (elliptical) corners.
 *  2. Evaluate num_candidates+1 arcs around a window center (the command, or
 *     the previous selection while avoiding). Per arc: bilateral clearance =
 *     min(left, right) smallest lateral offset from the arc centerline.
 *  3. Score = clearance (saturated at width/2 + avoid side margin)
 *             - fidelity * viability * |w - command w|
 *             - hysteresis * |w - previous selection|
 *     where viability = command arc's own clearance / body half width.
 *  4. Longitudinal-only speed scaling (steering is never scaled away):
 *     distance to first body-hit, lateral squeeze, proximity governor.
 */

#include "bilateral_arc_clearance_controller/bac_core.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace bac
{

BacCore::BacCore()
  : BacCore(Params{})
{
}

BacCore::BacCore(const Params &params)
  : params_(params)
  , current_status_(Status::CLEAR)
  , command_sign_(1)
  , avoiding_counter_(0)
  , prev_selected_w_(0.0f)
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
  command_sign_     = 1;
  avoiding_counter_ = 0;
  prev_selected_w_  = 0.0f;
}

ArcEvaluation
BacCore::evaluateArc(const std::vector<Point2D> &points, float v, float w, float horizon) const
{
  ArcEvaluation eval{ FLT_MAX, FLT_MAX, FLT_MAX, 1.0f };

  const Footprint &body = params_.footprint;
  // Body extent in the direction of travel (rear is negative)
  float lead_length  = (v >= 0.0f) ? body.front : -body.rear;
  float trail_length = (v >= 0.0f) ? -body.rear : body.front;
  float half_width   = body.width / 2.0f;
  float side_margin  = std::max(params_.safety_margin.side, 1e-3f);
  float s_max        = std::fabs(v) * horizon + lead_length;

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

    if (left_offset >= 0.0f)
    {
      eval.clearance_left = std::min(eval.clearance_left, left_offset);
    }
    else
    {
      eval.clearance_right = std::min(eval.clearance_right, -left_offset);
    }

    float abs_offset = std::fabs(left_offset);
    if (abs_offset < half_width)
    {
      // The body would hit this point: longitudinal distance governs the speed
      if (s < eval.blocking_s)
      {
        eval.blocking_s = s;
      }
    }
    else
    {
      // Non-hitting squeeze: intruding into the safety side margin caps the
      // speed proportionally instead of forbidding motion outright
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
BacCore::process(const std::vector<Point2D> &points, const Twist2D &command, const Twist2D &current)
{
  Result result;

  Twist2D target_vec = command;
  if (std::fabs(command.v) < std::fabs(current.v))
  {
    target_vec.v = current.v;
  }
  if (target_vec.v != 0.0f)
  {
    command_sign_ = (target_vec.v > 0.0f) ? 1 : -1;
  }

  // Legacy creep: while avoiding, a pure-turn command is given a matching
  // forward speed (capped at creep_speed) so the turn makes spatial progress.
  if (avoiding_counter_ > 0)
  {
    float radius   = params_.footprint.width / 2.0f + params_.safety_margin.side;
    float velocity = radius * std::fabs(target_vec.w) * command_sign_;
    float w_ratio  = 1.0f;
    if (std::fabs(velocity) > params_.creep_speed)
    {
      w_ratio  = params_.creep_speed / std::fabs(velocity);
      velocity = params_.creep_speed * command_sign_;
    }
    if (std::fabs(target_vec.v) < std::fabs(velocity))
    {
      target_vec.v = velocity;
      target_vec.w *= w_ratio;
    }
  }

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

  // Immediate-stop check: emergency zone = body rectangle (extended in the
  // CURRENT direction of travel by the braking distance at the current speed)
  // inflated by the safety margins with ROUNDED corners (elliptical, since the
  // margins are anisotropic). A plain margin rectangle would over-cover the
  // corners by sqrt(2) and dead-lock the robot on diagonal clearances that are
  // actually larger than the margin. State-based, continuous in speed; whether
  // the COMMANDED motion is safe is decided by the arc evaluation below.
  float min_proximity_norm = FLT_MAX;  // nearest point in margin-normalized units (1.0 = at the boundary)
  {
    float stop_decel = std::max(params_.stop_decel, 0.1f);
    float brake_dist =
        current.v * current.v / (2.0f * stop_decel) + std::fabs(current.v) * params_.brake_reaction_time;
    float body_x_min = params_.footprint.rear;
    float body_x_max = params_.footprint.front;
    if (current.v >= 0.0f)
    {
      body_x_max += brake_dist;
    }
    else
    {
      body_x_min -= brake_dist;
    }
    float body_y_half  = params_.footprint.width / 2.0f;
    float margin_front = std::max(params_.safety_margin.front, 1e-3f);
    float margin_rear  = std::max(params_.safety_margin.rear, 1e-3f);
    float margin_side  = std::max(params_.safety_margin.side, 1e-3f);

    for (const Point2D &point : filtered_points)
    {
      float dx_norm = 0.0f;
      if (point.x > body_x_max)
      {
        dx_norm = (point.x - body_x_max) / margin_front;
      }
      else if (point.x < body_x_min)
      {
        dx_norm = (body_x_min - point.x) / margin_rear;
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
        current_status_           = Status::STOP;
        result.output             = Twist2D(0.0f, 0.0f);
        result.status             = Status::STOP;
        result.min_proximity_norm = min_proximity_norm;
        return result;
      }
    }
  }
  result.min_proximity_norm = min_proximity_norm;

  // Candidate window: centered on the command normally, but on the PREVIOUS
  // selection while avoiding. Otherwise, when the goal lies behind the
  // obstacle, the upper command's sign flips with the heading, the window
  // snaps to the other side, and the avoidance direction flip-flops through
  // the obstacle forever. Committing the window to the previous choice keeps
  // one side reachable; the fidelity term still walks the window back to the
  // command (up to w_range per tick) as soon as clearance allows.
  float window_center = (avoiding_counter_ > 0) ? prev_selected_w_ : target_vec.w;

  // Bilateral clearance per candidate arc: min(left, right) is the effective
  // free half-width of the passage along that arc.
  float horizon        = params_.reaction_time + params_.estimation_time_margin;
  int   num_candidates = std::max(2, params_.num_candidates) & ~1;  // even, >= 2
  int   half_num       = num_candidates / 2;

  std::vector<float> candidate_clearances(num_candidates + 1);
  std::vector<float> candidate_blocking_s(num_candidates + 1);
  std::vector<float> candidate_lateral_fraction(num_candidates + 1);
  for (int index = -half_num; index <= half_num; index++)
  {
    float rotation_speed = window_center + (params_.w_range * index / half_num);
    ArcEvaluation eval   = evaluateArc(filtered_points, target_vec.v, rotation_speed, horizon);
    candidate_clearances[index + half_num]       = std::min(eval.clearance_left, eval.clearance_right);
    candidate_blocking_s[index + half_num]       = eval.blocking_s;
    candidate_lateral_fraction[index + half_num] = eval.lateral_fraction;
  }

  // Fidelity viability scaling: measure how passable the commanded arc itself
  // is (it may lie outside the committed candidate window). The reference is
  // the body half-width: as long as the commanded arc physically fits, the
  // intent keeps full weight (a tight but viable corridor entry must not be
  // discounted); only a command the body cannot follow loses its pull.
  ArcEvaluation command_eval = evaluateArc(filtered_points, target_vec.v, target_vec.w, horizon);
  float command_clearance    = std::min(command_eval.clearance_left, command_eval.clearance_right);
  float body_half_width      = std::max(params_.footprint.width / 2.0f, 1e-3f);
  float fidelity_scale       = std::max(params_.weights.fidelity_viability_floor,
                                        std::min(1.0f, command_clearance / body_half_width));
  result.command_clearance   = command_clearance;
  result.fidelity_scale      = fidelity_scale;

  // Score = bilateral clearance saturated at the avoid half-width, minus small
  // penalties for deviating from the commanded w and from the previous choice.
  // With room, many candidates saturate and the fidelity term picks the one
  // closest to the command (passthrough / light avoidance). In tight spaces the
  // max-min clearance dominates, which steers through the middle of the opening.
  float clearance_cap         = params_.footprint.width / 2.0f + params_.avoid_margin.side;
  float min_rotation_speed    = target_vec.w;
  float best_score            = -FLT_MAX;
  float best_blocking_s       = FLT_MAX;
  float best_lateral_fraction = 1.0f;
  // Visit candidates from the window center outwards so ties resolve towards
  // the smaller deviation (strict > comparison keeps the first best).
  for (int i = 0; i < num_candidates + 1; i++)
  {
    int evaluation_index;
    if (i == 0)
    {
      evaluation_index = 0;
    }
    else if (i % 2 == 0)
    {
      evaluation_index = (i + 1) / 2;
    }
    else
    {
      evaluation_index = -(i + 1) / 2;
    }

    float candidate_w = window_center + (params_.w_range * static_cast<float>(evaluation_index) / half_num);
    float saturated_clearance = std::min(candidate_clearances[evaluation_index + half_num], clearance_cap);
    float score = params_.weights.clearance * saturated_clearance -
                  params_.weights.fidelity * fidelity_scale * std::fabs(candidate_w - target_vec.w) -
                  params_.weights.hysteresis * std::fabs(candidate_w - prev_selected_w_);
    if (score > best_score)
    {
      best_score            = score;
      min_rotation_speed    = candidate_w;
      best_blocking_s       = candidate_blocking_s[evaluation_index + half_num];
      best_lateral_fraction = candidate_lateral_fraction[evaluation_index + half_num];
    }
  }
  prev_selected_w_   = min_rotation_speed;
  result.selected_w  = min_rotation_speed;

  // Longitudinal speed scaling on the selected arc, capped by two factors:
  //  - distance: fraction of the horizon that is free before the first point
  //    the body would hit (1.0 = no hit within the horizon)
  //  - squeeze: lateral margin fraction of the tightest non-hitting point
  //    (grazing the body -> 0, outside the safety side margin -> 1)
  float min_distance = best_lateral_fraction;
  if (best_blocking_s < FLT_MAX)
  {
    float lead_length      = (target_vec.v >= 0.0f) ? params_.footprint.front : -params_.footprint.rear;
    float lead_margin      = (target_vec.v >= 0.0f) ? params_.safety_margin.front : params_.safety_margin.rear;
    float free_run         = best_blocking_s - lead_length - lead_margin;
    float horizon_distance = std::fabs(target_vec.v) * horizon;
    float distance_fraction;
    if (horizon_distance > 1e-6f)
    {
      distance_fraction = std::max(0.0f, std::min(1.0f, free_run / horizon_distance));
    }
    else
    {
      distance_fraction = (free_run > 0.0f) ? 1.0f : 0.0f;
    }
    min_distance = std::min(min_distance, distance_fraction);
  }

  // Proximity speed governor: near anything (in margin-normalized distance,
  // 1.0 = at the emergency boundary) the speed is capped so the actual
  // trajectory tracks the evaluated arc closely (actuator lag makes the real
  // path straighter than the ideal arc; at full speed that eats into the
  // passing clearance). A small creep floor keeps an escape possible right at
  // the boundary instead of freezing there.
  if (min_proximity_norm < params_.proximity_governor_range)
  {
    float proximity_fraction = std::max(min_proximity_norm - 1.0f, params_.creep_fraction);
    min_distance             = std::min(min_distance, proximity_fraction);
  }
  result.speed_fraction = min_distance;

  // Longitudinal deceleration only: the selected steering rate is kept even
  // when the arc is blocked ahead, so a blocked robot turns towards the free
  // direction (pure rotation at v=0) instead of freezing. Scaling w together
  // with v would zero the very steering that resolves the blockage.
  float target_v = command.v;
  float target_w = min_rotation_speed;
  if (min_distance < 1.0f)
  {
    target_v = target_vec.v * min_distance;
  }
  if (std::fabs(target_v) < params_.velocity_min)
  {
    target_v = 0.0f;
  }
  if (std::fabs(target_w) < params_.angvel_min)
  {
    target_w = 0.0f;
  }

  result.output = Twist2D(target_v, target_w);

  if (std::fabs(target_v - command.v) < 1e-3f && std::fabs(target_w - command.w) < 1e-3f)
  {
    avoiding_counter_--;
    if (avoiding_counter_ < 0)
    {
      avoiding_counter_ = 0;
    }
    // Latch: while the avoidance was recently active, keep reporting AVOIDING
    // so a downstream arbitration keeps applying our output. Otherwise a
    // single converged tick lets the raw upper command through and the
    // avoidance turn is undone every other tick (status chattering).
    if (avoiding_counter_ > 0)
    {
      current_status_ = Status::AVOIDING;
      result.status   = Status::AVOIDING;
      return result;
    }
    current_status_ = Status::CLEAR;
    result.status   = Status::CLEAR;
    return result;
  }

  if (target_v == 0.0f && target_w == 0.0f)
  {
    current_status_ = Status::STOP;
    result.status   = Status::STOP;
    return result;
  }

  current_status_   = Status::AVOIDING;
  avoiding_counter_ = params_.avoiding_latch_ticks;
  result.status     = Status::AVOIDING;
  return result;
}

}  // namespace bac
