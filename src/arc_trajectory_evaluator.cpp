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

// Declared in the header: the emergency governor in bac_core.cpp needs the
// same support function for its slab test.
float
supportExtent(const Footprint &body, float dx, float dy)
{
  const float along = (dx >= 0.0f) ? body.front * dx : body.rear * dx;
  return along + (body.width / 2.0f) * std::fabs(dy);
}

namespace
{

constexpr float kPi = 3.14159265358979323846f;

/**
 * Geometry of one constant body twist, in the body frame.
 *
 * A constant `(v, vy, w)` moves the body origin along a circle about the
 * instantaneous centre of rotation - the body-frame point whose velocity is
 * zero. Solving `(v - w*p_y, vy + w*p_x) = 0` gives
 *
 *     centre = (-vy / w, v / w)
 *
 * which is the familiar `(0, v / w)` exactly when `vy == 0`. The heading
 * therefore keeps a CONSTANT offset from the direction of travel (the crab
 * angle `atan2(vy, v)`), so the swept-rectangle geometry below is the
 * non-holonomic geometry with the centre moved off the body y axis, not a
 * different construction.
 */
struct SweptFrame
{
  float speed = 0.0f;     // |V|, distance travelled per unit time
  float ux = 1.0f;        // direction of travel, unit, body frame
  float uy = 0.0f;
  float nx = 0.0f;        // left normal of the direction of travel, unit
  float ny = 1.0f;
  float lead = 0.0f;      // body extent ahead of the origin along +u
  float trail = 0.0f;     // body extent behind the origin along -u
  float perp_left = 0.0f;   // body extent to the left of the travel line
  float perp_right = 0.0f;  // body extent to the right of it
  bool  turning = false;  // the twist has a usable centre of rotation
  float cx = 0.0f;        // centre of rotation, body frame
  float cy = 0.0f;
  float turn_radius = 0.0f;  // |centre|, the radius the body origin travels on
  float sweep_r_min = 0.0f;  // nearest body point to the centre
  float sweep_r_max = 0.0f;  // farthest body point from it
};

SweptFrame
makeSweptFrame(const Footprint &body, const Twist2D &command)
{
  SweptFrame frame;
  frame.speed = command.speed();

  // Reversing on a straight line keeps the legacy convention: the direction of
  // travel is -x, so the rear edge leads.
  if (frame.speed > 1e-6f)
  {
    frame.ux = command.v / frame.speed;
    frame.uy = command.vy / frame.speed;
  }
  else
  {
    frame.ux = (command.v >= 0.0f) ? 1.0f : -1.0f;
    frame.uy = 0.0f;
  }
  frame.nx = -frame.uy;
  frame.ny = frame.ux;

  frame.lead = supportExtent(body, frame.ux, frame.uy);
  frame.trail = supportExtent(body, -frame.ux, -frame.uy);
  frame.perp_left = supportExtent(body, frame.nx, frame.ny);
  frame.perp_right = supportExtent(body, -frame.nx, -frame.ny);

  frame.turning = std::fabs(command.w) > 1e-4f && frame.speed > 1e-3f;
  if (!frame.turning)
  {
    return frame;
  }

  frame.cx = -command.vy / command.w;
  frame.cy = command.v / command.w;
  frame.turn_radius = frame.speed / std::fabs(command.w);

  // Nearest body point to the centre: the distance from the centre to the
  // footprint rectangle. Computed component-wise so that a centre directly
  // abeam the body - every non-holonomic command - yields the exact
  // `turn_radius - width/2` the closed-form used to produce.
  const float half = body.width / 2.0f;
  const float gap_x =
      std::max(std::max(body.rear - frame.cx, 0.0f), frame.cx - body.front);
  const float gap_y = std::max(std::max(-half - frame.cy, 0.0f), frame.cy - half);
  if (gap_x == 0.0f)
  {
    frame.sweep_r_min = gap_y;
  }
  else if (gap_y == 0.0f)
  {
    frame.sweep_r_min = gap_x;
  }
  else
  {
    frame.sweep_r_min = std::sqrt(gap_x * gap_x + gap_y * gap_y);
  }

  // Farthest body point from the centre is always a corner.
  frame.sweep_r_max = 0.0f;
  for (const float corner_x : { body.front, body.rear })
  {
    for (const float corner_y : { half, -half })
    {
      const float dx = corner_x - frame.cx;
      const float dy = corner_y - frame.cy;
      frame.sweep_r_max = std::max(frame.sweep_r_max, std::sqrt(dy * dy + dx * dx));
    }
  }
  return frame;
}

/**
 * Exact first contact between the rectangular body moving on a constant-
 * curvature arc and a static point. See the public algorithm documentation
 * and the property tests in core_unit.cpp for the geometric derivation.
 *
 * The body-frame construction is independent of where the centre of rotation
 * lies, so the same solution serves holonomic and non-holonomic commands: the
 * point orbits the centre, and contact is the first orbit angle at which it
 * enters the rectangle.
 */
float
firstContactArcLength(const Point2D &point, const SweptFrame &frame, float w,
                      const Footprint &body, float s_limit)
{
  const float sigma = (w > 0.0f) ? 1.0f : -1.0f;
  const float ux = point.x - frame.cx, uy = point.y - frame.cy;
  const float r = std::sqrt(ux * ux + uy * uy);
  const float half = body.width / 2.0f;

  if (r < 1e-6f)
  {
    const bool inside = frame.cx >= body.rear && frame.cx <= body.front &&
                        std::fabs(frame.cy) <= half;
    return inside ? 0.0f : FLT_MAX;
  }

  const float psi0 = std::atan2(uy, ux);
  const float rho_max = std::fabs(w) * s_limit / std::max(frame.speed, 1e-6f);

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
    const float qx = frame.cx + r * std::cos(theta);
    const float qy = frame.cy + r * std::sin(theta);
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
    const float sv = (y_e - frame.cy) / r;
    if (std::fabs(sv) <= 1.0f)
    {
      const float a = std::asin(sv);
      consider(a);
      consider(kPi - a);
    }
  }
  for (float x_e : { body.front, body.rear })
  {
    const float cv = (x_e - frame.cx) / r;
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
  return frame.speed * best_rho / std::max(std::fabs(w), 1e-6f);
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
  const float w = command.w;
  ArcEvaluation eval{ FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, 1.0f };

  const Footprint &body = params_.footprint;
  const SweptFrame frame = makeSweptFrame(body, command);
  const float lead_length = frame.lead;
  const float trail_length = frame.trail;
  const float side_margin = std::max(params_.safety_margin.side, 1e-3f);
  const float s_max_clear = clearance_distance + lead_length;
  const float s_max = std::max(blocking_distance, clearance_distance) + lead_length;

  for (const Point2D &point : points)
  {
    float s;
    float left_offset;

    if (!frame.turning)
    {
      s = point.x * frame.ux + point.y * frame.uy;
      left_offset = point.x * frame.nx + point.y * frame.ny;
    }
    else
    {
      const float rux = point.x - frame.cx, ruy = point.y - frame.cy;
      const float radial = std::sqrt(rux * rux + ruy * ruy);
      left_offset = (frame.turn_radius - radial) * ((w > 0.0f) ? 1.0f : -1.0f);

      const float alpha = std::atan2(ruy, rux);
      const float alpha0 = std::atan2(-frame.cy, -frame.cx);
      float delta = alpha - alpha0;
      while (delta > kPi) delta -= 2.0f * kPi;
      while (delta <= -kPi) delta += 2.0f * kPi;
      s = frame.speed * (delta / w);

      if (radial >= frame.sweep_r_min - 1e-3f && radial <= frame.sweep_r_max + 1e-3f)
      {
        const float s_angle_cap = frame.turn_radius * params_.eval_angle_max;
        const float s_hit = firstContactArcLength(
            point, frame, w, body, std::min(s_max, s_angle_cap));
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
    const float side_extent = (left_offset >= 0.0f) ? frame.perp_left : frame.perp_right;
    bool body_hit;
    if (!frame.turning)
    {
      body_hit = abs_offset < side_extent;
      const float s_contact = s - lead_length;
      if (body_hit && s_contact < eval.blocking_s)
      {
        eval.blocking_s = std::max(s_contact, 0.0f);
      }
    }
    else
    {
      const float radial = frame.turn_radius - left_offset * ((w > 0.0f) ? 1.0f : -1.0f);
      body_hit = radial > frame.sweep_r_min && radial < frame.sweep_r_max;
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
          const float cap_ref = side_extent + params_.avoid_margin.side;
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

    if (abs_offset >= side_extent)
    {
      const float fraction = (abs_offset - side_extent) / side_margin;
      if (fraction < eval.lateral_fraction)
      {
        eval.lateral_fraction = std::min(1.0f, fraction);
      }
    }
  }

  return eval;
}

}  // namespace bac::detail
