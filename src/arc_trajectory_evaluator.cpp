/**
 * @file arc_trajectory_evaluator.cpp
 * @brief Internal bilateral clearance and swept-footprint evaluation for arcs
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "arc_trajectory_evaluator.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace bac::detail
{

namespace
{

constexpr float kPi = 3.14159265358979323846f;

/**
 * Exact first contact between the rectangular body moving on a constant-
 * curvature arc and a static point. See the public algorithm documentation
 * and the property tests in core_unit.cpp for the geometric derivation.
 */
float
firstContactArcLength(const Point2D &point, float v, float w, const Footprint &body,
                      float s_limit)
{
  const float R = v / w;
  const float sigma = (w > 0.0f) ? 1.0f : -1.0f;
  const float ux = point.x, uy = point.y - R;
  const float r = std::sqrt(ux * ux + uy * uy);
  const float half = body.width / 2.0f;

  if (r < 1e-6f)
  {
    const bool inside =
        std::fabs(R) <= half && body.rear <= 0.0f && body.front >= 0.0f;
    return inside ? 0.0f : FLT_MAX;
  }

  const float psi0 = std::atan2(uy, ux);
  const float rho_max = std::fabs(w) * s_limit / std::max(std::fabs(v), 1e-6f);

  float best_rho = FLT_MAX;
  auto consider = [&](float theta) {
    float rho = std::fmod(sigma * (psi0 - theta), 2.0f * kPi);
    if (rho < 0.0f)
    {
      rho += 2.0f * kPi;
    }
    if (rho > rho_max + 1e-6f)
    {
      return;
    }
    const float qx = r * std::cos(theta);
    const float qy = R + r * std::sin(theta);
    const float tol = 1e-4f;
    if (qx >= body.rear - tol && qx <= body.front + tol &&
        std::fabs(qy) <= half + tol)
    {
      best_rho = std::min(best_rho, rho);
    }
  };

  if (point.x >= body.rear && point.x <= body.front && std::fabs(point.y) <= half)
  {
    return 0.0f;
  }
  for (float y_e : { half, -half })
  {
    const float sv = (y_e - R) / r;
    if (std::fabs(sv) <= 1.0f)
    {
      const float a = std::asin(sv);
      consider(a);
      consider(kPi - a);
    }
  }
  for (float x_e : { body.front, body.rear })
  {
    const float cv = x_e / r;
    if (std::fabs(cv) <= 1.0f)
    {
      const float a = std::acos(cv);
      consider(a);
      consider(-a);
    }
  }
  if (best_rho == FLT_MAX)
  {
    return FLT_MAX;
  }
  return std::fabs(v) * best_rho / std::max(std::fabs(w), 1e-6f);
}

}  // namespace

ArcTrajectoryEvaluator::ArcTrajectoryEvaluator(const Params &params)
  : params_(params)
{
}

ArcEvaluation
ArcTrajectoryEvaluator::evaluate(const std::vector<Point2D> &points, const Twist2D &command,
                                 float clearance_distance, float blocking_distance) const
{
  const float v = command.v;
  const float w = command.w;
  ArcEvaluation eval{ FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, 1.0f };

  const Footprint &body = params_.footprint;
  const float lead_length = (v >= 0.0f) ? body.front : -body.rear;
  const float trail_length = (v >= 0.0f) ? -body.rear : body.front;
  const float half_width = body.width / 2.0f;
  const float side_margin = std::max(params_.safety_margin.side, 1e-3f);
  const float s_max_clear = clearance_distance + lead_length;
  const float s_max = std::max(blocking_distance, clearance_distance) + lead_length;

  float outer_excess = 0.0f;
  float sweep_r_min = 0.0f, sweep_r_max = FLT_MAX;
  if (std::fabs(w) > 1e-4f && std::fabs(v) > 1e-3f)
  {
    const float turn_r = std::fabs(v / w);
    const float corner_l = std::max(body.front, -body.rear);
    const float outer_r = turn_r + half_width;
    outer_excess = std::sqrt(outer_r * outer_r + corner_l * corner_l) - outer_r;
    sweep_r_min = std::max(turn_r - half_width, 0.0f);
    sweep_r_max = std::sqrt(outer_r * outer_r + corner_l * corner_l);
  }

  for (const Point2D &point : points)
  {
    float s;
    float left_offset;

    if (std::fabs(w) < 1e-4f)
    {
      const float direction = (v >= 0.0f) ? 1.0f : -1.0f;
      s = point.x * direction;
      left_offset = point.y * direction;
    }
    else
    {
      const float turn_radius = v / w;
      const float abs_turn_radius = std::fabs(turn_radius);
      const float radial = std::sqrt(
          point.x * point.x + (point.y - turn_radius) * (point.y - turn_radius));
      left_offset = (abs_turn_radius - radial) * ((w > 0.0f) ? 1.0f : -1.0f);

      const float alpha = std::atan2(point.y - turn_radius, point.x);
      const float alpha0 = (turn_radius >= 0.0f) ? -kPi / 2.0f : kPi / 2.0f;
      float delta = alpha - alpha0;
      while (delta > kPi) delta -= 2.0f * kPi;
      while (delta <= -kPi) delta += 2.0f * kPi;
      s = std::fabs(v) * (delta / w);

      const float rux = point.x, ruy = point.y - turn_radius;
      const float r_p = std::sqrt(rux * rux + ruy * ruy);
      if (r_p >= sweep_r_min - 1e-3f && r_p <= sweep_r_max + 1e-3f)
      {
        const float s_angle_cap = std::fabs(v / w) * params_.eval_angle_max;
        const float s_hit = firstContactArcLength(
            point, v, w, body, std::min(s_max, s_angle_cap));
        if (s_hit < eval.blocking_s)
        {
          eval.blocking_s = s_hit;
        }
      }
    }

    if (s < -trail_length || s > s_max)
    {
      continue;
    }

    const float abs_offset = std::fabs(left_offset);
    bool body_hit;
    if (std::fabs(w) < 1e-4f)
    {
      body_hit = abs_offset < half_width;
      const float s_contact = s - lead_length;
      if (body_hit && s_contact < eval.blocking_s)
      {
        eval.blocking_s = std::max(s_contact, 0.0f);
      }
    }
    else
    {
      const float inward = left_offset * ((w > 0.0f) ? 1.0f : -1.0f);
      body_hit = (inward < half_width) && (inward > -(half_width + outer_excess));
    }

    if (s > s_max_clear)
    {
      if (body_hit)
      {
        const float d_e = std::sqrt(point.x * point.x + point.y * point.y);
        float fade = (d_e - params_.blocked_near) /
                     std::max(params_.blocked_far - params_.blocked_near, 1e-3f);
        fade = std::max(0.0f, std::min(1.0f, fade));
        if (fade < 1.0f)
        {
          const float cap_ref = half_width + params_.avoid_margin.side;
          const float pseudo = abs_offset + fade * (cap_ref - abs_offset);
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
      continue;
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
      const float fraction = (abs_offset - half_width) / side_margin;
      if (fraction < eval.lateral_fraction)
      {
        eval.lateral_fraction = std::min(1.0f, fraction);
      }
    }
  }

  return eval;
}

}  // namespace bac::detail
