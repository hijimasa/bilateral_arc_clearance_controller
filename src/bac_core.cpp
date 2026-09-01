/**
 * @file bac_core.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Framework-free core of the arc-clearance local planner
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Algorithm summary (details and rationale in the package README):
 *  1. Emergency stop if any point is inside the body + braking distance zone,
 *     inflated by the safety margins with rounded (elliptical) corners.
 *  2. Ask the configured motion model for candidates: v inside the
 *     acceleration-limited translational window crossed with yaw rate
 *     (differential drive, plus a v=0 rotation row) or with body curvature
 *     bounded by turn_radius_min (Ackermann, never a rotation row), plus the
 *     stop candidate. Roll each out as a constant-curvature arc.
 *  3. Admissibility: discard candidates that cannot stop before the first
 *     body hit on their arc (and rotations that sweep into an obstacle).
 *  4. Score = saturated bilateral clearance + local-goal following (distance /
 *     heading) - hysteresis - lateral squeeze; output the best.
 */

#include "bilateral_arc_clearance_controller/bac_core.hpp"

#include "arc_trajectory_evaluator.hpp"
#include "motion_model.hpp"

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
  /// Speed fraction demanded by points on a COLLISION course (straight-line
  /// miss distance overlaps the body): longitudinal gap over the governor
  /// look-ahead, so 0 at the zone edge and 1 at the look-ahead. Points that
  /// would pass beside the body do not enter this ramp - a corridor wall
  /// parallel to the travel direction leaves the cruise speed alone. Used by
  /// the speed governor only; the emergency stop stays isotropic.
  float governor   = FLT_MAX;
  /// Tightest predicted side gap (points ahead on a GRAZING course, or
  /// already beside the body), as a ratio of the UNSCALED side margin. The
  /// governor keeps the sampled speed below the speed whose own scaled margin
  /// would swallow that gap - otherwise passing close to an obstacle
  /// Zeno-chatters: accelerate -> margin grows over the point -> emergency
  /// stop -> repeat.
  float side_ratio = FLT_MAX;
  float distance   = FLT_MAX;  // Euclidean distance to physical footprint
  bool  emergency  = false;
  /// Nearest emergency-zone point (valid when emergency): the standstill
  /// escape must move AWAY from it, whichever direction that is.
  float emergency_x = 0.0f;
  float emergency_y = 0.0f;
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
    if (normalized < result.normalized)
    {
      result.normalized  = normalized;
      result.emergency_x = point.x;
      result.emergency_y = point.y;
    }
    result.emergency = result.emergency || normalized < 1.0f;

    // Governor hazard (straight-line slab test in the current direction of
    // travel, with UNSCALED margins so the cap does not feed back on itself):
    // a point either hits the body head-on (|y| inside the body: brake on the
    // longitudinal ramp), grazes the side margin band (predictive side
    // envelope), or misses entirely (a corridor wall: no cap at all).
    const bool  forward   = current.v >= 0.0f;
    const float ahead     = forward ? point.x - zone_x_max : zone_x_min - point.x;
    const float lookahead = std::max(params.side_envelope_lookahead, 1e-3f);
    const float side_gap  = std::fabs(point.y) - body_y_half;
    if (side_gap < 0.0f && ahead > 0.0f)
    {
      // Dead-ahead collision course: linear speed ramp over the governor
      // look-ahead (full cruise at lookahead, creep floor at the zone edge).
      float hazard = ahead / lookahead;
      // Arc refinement, RELEASE-ONLY: while the robot is turning, the
      // straight-line prediction keeps reading the outer wall of the corner
      // as a frontal threat for the whole turn (the nose points at it until
      // the heading has swung), pinning the corner speed at creep. If the
      // CURRENT (v, w) arc carries the point past the body, it is no
      // hazard. Straight driving (small w) keeps the straight slab exactly,
      // so corridors are untouched, and a transient centering correction
      // cannot create a new hazard either - braking requires BOTH
      // predictions to hit. Admissibility and the emergency layer are
      // unaffected.
      if (current.v > 0.05f && std::fabs(current.w) > 1e-2f)
      {
        const float turn_radius = current.v / current.w;
        const float radial      = std::sqrt(point.x * point.x +
                                            (point.y - turn_radius) * (point.y - turn_radius));
        const float arc_offset  = std::fabs(turn_radius) - radial;  // |.| < half = arc hits
        if (std::fabs(arc_offset) >= body_y_half)
        {
          hazard = 1.0f;  // the current turn carries the point past the body
        }
        else
        {
          float alpha  = std::atan2(point.y - turn_radius, point.x);
          float alpha0 = (turn_radius >= 0.0f) ? -kPi / 2.0f : kPi / 2.0f;
          float delta  = alpha - alpha0;
          while (delta > kPi) delta -= 2.0f * kPi;
          while (delta <= -kPi) delta += 2.0f * kPi;
          const float s_arc = current.v * (delta / current.w) - zone_x_max;
          if (s_arc > 0.0f)
          {
            hazard = std::min(hazard, s_arc / lookahead);
          }
        }
      }
      result.governor = std::min(result.governor, hazard);
    }
    else if (side_gap >= 0.0f)
    {
      // Predicted close pass: between the trailing zone edge and the
      // look-ahead the pass gap feeds the side envelope. The look-ahead must
      // cover the swerve-carving phase - releasing the cap the moment the
      // predicted miss turns positive would let the robot accelerate
      // mid-carve and flatten its own berth.
      const float behind = forward ? zone_x_min - point.x : point.x - zone_x_max;
      if (behind < 0.0f && ahead < lookahead)
      {
        // Arc refinement, RELEASE-ONLY: mid-turn, the straight prediction
        // reads the wall the nose sweeps across as a grazing pass and pins
        // the corner speed at creep. When the CURRENT (v, w) arc carries the
        // point clear by at least 1.5x the side margin, it is no close pass
        // (1.5: swept over 1.1-2.5 - smaller re-tightens corners, larger
        // trades Z-corner clearance for nothing). A genuine close pass being
        // carved right now stays capped: at the moment of closest approach
        // the arc offset IS the (sub-band) pass gap, so the release
        // threshold is never met there.
        bool released = false;
        if (current.v > 0.05f && std::fabs(current.w) > 1e-2f)
        {
          const float turn_radius = current.v / current.w;
          const float radial      = std::sqrt(point.x * point.x +
                                              (point.y - turn_radius) * (point.y - turn_radius));
          const float arc_gap = std::fabs(std::fabs(turn_radius) - radial) - body_y_half;
          released = arc_gap >= params.safety_margin.side * 1.5f;
        }
        if (!released)
        {
          result.side_ratio = std::min(
              result.side_ratio, side_gap / std::max(params.safety_margin.side, 1e-3f));
        }
      }
    }

    const float dx = std::max(std::max(point.x - body_x_max, body_x_min - point.x), 0.0f);
    const float dy = std::max(std::fabs(point.y) - body_y_half, 0.0f);
    result.distance = std::min(result.distance, std::sqrt(dx * dx + dy * dy));
  }
  return result;
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
  , prev_selected_command_(0.0f, 0.0f)
  , cap_ema_(-1.0f)
  , alignment_mode_(false)
{
  rebuildMotionModel();
}

BacCore::~BacCore() = default;

BacCore::BacCore(const BacCore &other)
  : params_(other.params_)
  , current_status_(other.current_status_)
  , avoiding_counter_(other.avoiding_counter_)
  , prev_selected_command_(other.prev_selected_command_)
  , cap_ema_(other.cap_ema_)
  , alignment_mode_(other.alignment_mode_)
{
  rebuildMotionModel();
}

BacCore &
BacCore::operator=(const BacCore &other)
{
  if (this != &other)
  {
    const Params previous = params_;
    params_ = other.params_;
    try
    {
      rebuildMotionModel();
    }
    catch (...)
    {
      // The surviving model reads params_ by reference, so it must never be
      // left describing a configuration this object does not hold.
      params_ = previous;
      throw;
    }
    current_status_        = other.current_status_;
    avoiding_counter_      = other.avoiding_counter_;
    prev_selected_command_ = other.prev_selected_command_;
    cap_ema_               = other.cap_ema_;
    alignment_mode_        = other.alignment_mode_;
  }
  return *this;
}

void
BacCore::rebuildMotionModel()
{
  motion_model_ = detail::makeMotionModel(params_);
}

void
BacCore::setParams(const Params &params)
{
  // Validate FIRST. The motion model holds a reference to params_, so mutating
  // params_ before a failed rebuild would leave the surviving model reading a
  // rejected configuration - a negative turn_radius_min then turns the steering
  // clamp into a full-lock command instead of a clean failure.
  detail::validateMotionModelParams(params);

  const Params previous = params_;
  params_ = params;
  try
  {
    rebuildMotionModel();
  }
  catch (...)
  {
    // Validation above rejects an unusable configuration, so reaching here
    // means allocation failed. Restore params_ so the surviving model still
    // describes the configuration this object reports.
    params_ = previous;
    throw;
  }

  if (previous.motion_model.type != params.motion_model.type ||
      previous.turn_radius_min != params.turn_radius_min)
  {
    prev_selected_command_ = Twist2D{};
    alignment_mode_ = false;
  }
}

const Params &
BacCore::params() const
{
  return params_;
}

Twist2D
BacCore::limitReachableCommand(const Twist2D &current, const Twist2D &desired) const
{
  return motion_model_->limitReachableCommand(current, desired);
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
  prev_selected_command_ = Twist2D{};
  cap_ema_          = -1.0f;
  alignment_mode_   = false;
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
  const detail::ArcTrajectoryEvaluator evaluator(params_);
  return evaluator.evaluate(points, Twist2D(v, w), dist_clear, dist_block);
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

  bool escape_only = false;
  if (proximity.emergency)
  {
    if (std::fabs(current.v) > 0.05f || std::fabs(current.w) > 0.1f)
    {
      // Braking has priority while moving.
      current_status_ = Status::STOP;
      result.output   = Twist2D(0.0f, 0.0f);
      result.status   = Status::STOP;
      return result;
    }
    // Standstill with a point still inside the floor-scaled margins: a plain
    // latch would freeze forever. Per the safety contract (backing away and
    // waiting beats freezing), let the ESCAPE candidates run. The escape
    // must move AWAY from the offending point, whichever direction that is:
    // a frontal point permits only reverse, a point beside the rear permits
    // forward (categorically forbidding forward here froze exactly that
    // case). Rotation stays forbidden, and if every escape is blocked the
    // stop candidate wins and this remains a stop.
    escape_only = true;
  }

  // No intent: hold still. Influence-based status so the filter node's
  // pass-through arbitration still works while idling.
  if (path.empty())
  {
    alignment_mode_ = false;
    result.output   = Twist2D(0.0f, 0.0f);
    current_status_ =
        (proximity.distance > params_.influence_range) ? Status::CLEAR : Status::AVOIDING;
    result.status   = current_status_;
    return result;
  }

  // Station goal: decimated projection polyline with cumulative arc length.
  // Candidates are scored by projection station (progress) and path tangent.
  std::vector<Point2D> station_pts;
  std::vector<float>   station_s;
  std::vector<bool>    station_blocked;
  size_t               station_i0 = 0;   // robot's own projection segment
  float                station_s0 = 0.0f;  // robot's own projection station
  if (path.size() >= 2)
  {
    station_pts.reserve(path.size());
    station_pts.push_back(path.front());
    for (const Point2D &p : path)
    {
      const Point2D &last = station_pts.back();
      const float dx = p.x - last.x, dy = p.y - last.y;
      if (dx * dx + dy * dy >= 0.04f)  // resample at >= 0.2 m spacing
      {
        station_pts.push_back(p);
      }
    }
    {
      const Point2D &last = station_pts.back();
      const float dx = path.back().x - last.x, dy = path.back().y - last.y;
      if (dx * dx + dy * dy > 1e-6f)
      {
        station_pts.push_back(path.back());
      }
    }
    station_s.assign(station_pts.size(), 0.0f);
    for (size_t i = 1; i < station_pts.size(); ++i)
    {
      const float dx = station_pts[i].x - station_pts[i - 1].x;
      const float dy = station_pts[i].y - station_pts[i - 1].y;
      station_s[i]   = station_s[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    // Segments blocked by an obstacle (degraded plan) exert no LATERAL
    // attraction: pulling back onto a blocked line fights the swerve the
    // clearance terms are trying to make. Progress and tangent still apply.
    station_blocked.assign(station_pts.size(), false);
    const float block_radius   = params_.footprint.width / 2.0f;
    const float block_r2       = block_radius * block_radius;
    for (size_t i = 0; i + 1 < station_pts.size(); ++i)
    {
      const float ax = station_pts[i].x, ay = station_pts[i].y;
      const float vx = station_pts[i + 1].x - ax, vy = station_pts[i + 1].y - ay;
      const float len2 = std::max(vx * vx + vy * vy, 1e-9f);
      for (const Point2D &o : filtered_points)
      {
        float t = std::max(0.0f, std::min(1.0f, ((o.x - ax) * vx + (o.y - ay) * vy) / len2));
        const float qx = ax + t * vx, qy = ay + t * vy;
        if ((o.x - qx) * (o.x - qx) + (o.y - qy) * (o.y - qy) < block_r2)
        {
          station_blocked[i] = true;
          break;
        }
      }
    }
    // Robot projection (origin). Candidate projections search only from this
    // segment forward, which keeps the station monotone on paths that pass
    // near themselves.
    float best_d2 = FLT_MAX;
    for (size_t i = 0; i + 1 < station_pts.size(); ++i)
    {
      const float ax = station_pts[i].x, ay = station_pts[i].y;
      const float vx = station_pts[i + 1].x - ax, vy = station_pts[i + 1].y - ay;
      const float len2 = vx * vx + vy * vy;
      if (len2 < 1e-9f) continue;
      float t = std::max(0.0f, std::min(1.0f, (-ax * vx - ay * vy) / len2));
      const float qx = ax + t * vx, qy = ay + t * vy;
      const float d2 = qx * qx + qy * qy;
      if (d2 < best_d2)
      {
        best_d2    = d2;
        station_i0 = i;
        station_s0 = station_s[i] + t * std::sqrt(len2);
      }
    }
  }
  // A single-point (or sub-resolution) path degenerates to a plain point
  // goal: the cost lambda then returns the Euclidean distance to it.
  const bool  station_degenerate = station_pts.size() < 2;
  const float station_total      = station_degenerate ? 0.0f : station_s.back();

  // Candidate evaluation never looks meaningfully past the END of the path:
  // the run stops at the goal, so clearance differences beyond it (the wall
  // behind a docking point, the arena boundary) are irrelevant to the
  // remaining task - left uncapped they dominate the score on approach and
  // curl the robot away from the goal (the terminal whip). Safety is
  // unaffected: the emergency layer and the governor act on raw points.
  const float remaining_path =
      (station_degenerate ? std::sqrt(path.back().x * path.back().x +
                                      path.back().y * path.back().y)
                          : station_total - station_s0) +
      0.5f;

  // Diagnostics: the reported local goal is the path point one preview
  // length ahead of the robot's own projection (display only - scoring uses
  // the projection itself).
  {
    constexpr float kGoalPreview = 2.5f;  // [m]
    Point2D goal = path.back();
    if (!station_degenerate)
    {
      for (size_t i = 0; i < station_pts.size(); ++i)
      {
        goal = station_pts[i];
        if (station_s[i] >= station_s0 + kGoalPreview)
        {
          break;
        }
      }
    }
    result.goal_x = goal.x;
    result.goal_y = goal.y;
  }

  // Project a point onto the polyline (from the robot's segment forward).
  // Returns the goal COST (remaining station + weighted lateral offset; full
  // Euclidean distance once past the path end) and the reference bearing
  // (path tangent, or the direction to the final point at the terminal).
  // progress_out is signed progress from the robot's current projection;
  // it prevents collision-free motion AWAY from the ordered path from being
  // mistaken for a useful candidate.
  const float local_goal_distance = std::hypot(path.back().x, path.back().y);
  auto station_goal_cost = [&](float px, float py, float &bearing_out, float &heading_scale,
                               float &progress_out) {
    heading_scale = 1.0f;
    if (station_degenerate)
    {
      const float dx = path.back().x - px, dy = path.back().y - py;
      const float d  = std::sqrt(dx * dx + dy * dy);
      bearing_out    = (d > 1e-3f) ? std::atan2(dy, dx) : 0.0f;
      heading_scale  = std::min(1.0f, d / 0.5f);
      progress_out   = local_goal_distance - d;
      return d;
    }
    float best_d2 = FLT_MAX, s_best = 0.0f, tan_best = 0.0f;
    float best_qx = 0.0f, best_qy = 0.0f;
    bool  clamped = false, clamped_end = false, blocked = false;
    for (size_t i = station_i0; i + 1 < station_pts.size(); ++i)
    {
      const float ax = station_pts[i].x, ay = station_pts[i].y;
      const float vx = station_pts[i + 1].x - ax, vy = station_pts[i + 1].y - ay;
      const float len2 = vx * vx + vy * vy;
      if (len2 < 1e-9f) continue;
      float t = std::max(0.0f, std::min(1.0f, ((px - ax) * vx + (py - ay) * vy) / len2));
      const float qx = ax + t * vx, qy = ay + t * vy;
      const float d2 = (px - qx) * (px - qx) + (py - qy) * (py - qy);
      if (d2 < best_d2)
      {
        best_d2    = d2;
        s_best     = station_s[i] + t * std::sqrt(len2);
        tan_best   = std::atan2(vy, vx);
        best_qx    = qx;
        best_qy    = qy;
        // Clamped past either END of the projected span: beyond the last
        // vertex, or before the first considered segment. In the clamp
        // region the progress term has no gradient, so the full Euclidean
        // distance to the clamp point must take over - otherwise a robot
        // outside the path's longitudinal span (facing away at the path
        // start, or overshooting the end) can wander at only the weak
        // lateral cost.
        clamped_end = (i + 2 == station_pts.size()) && t >= 1.0f - 1e-4f;
        clamped     = clamped_end || (i == station_i0 && t <= 1e-4f);
        blocked     = station_blocked[i];
      }
    }
    const float d = std::sqrt(best_d2);
    progress_out = s_best - station_s0;
    if (clamped && d > 1e-3f)
    {
      bearing_out = std::atan2(best_qy - py, best_qx - px);
      if (clamped_end)
      {
        // Endpoint past the path END: the bearing to the goal point
        // degenerates as the distance shrinks (it flips to "behind" for any
        // overshoot), and a full-weight heading term then REWARDS arcs that
        // curl in beside the goal - the terminal whip. Fade the heading
        // authority out with the remaining distance; a far overshoot
        // (recovery) keeps it. The distance cost alone already prefers the
        // minimal-overshoot slow straight approach.
        heading_scale = std::min(1.0f, d / 0.5f);
      }
      return (station_total - s_best) + d;
    }
    bearing_out = tan_best;
    const float lateral = blocked ? 0.0f : params_.station_lateral_weight * d;
    return (station_total - s_best) + lateral;
  };

  // Alignment follows the ordered path TANGENT, not the bearing to its first
  // emitted point. A replanned/decimated path commonly starts 0.1 m ahead;
  // using that point bearing would rotate towards a harmless lateral path
  // offset near the goal and fight BAC's geometric centering.
  float relative_path_heading;
  if (station_degenerate)
  {
    relative_path_heading = std::atan2(path.back().y, path.back().x);
  }
  else
  {
    const Point2D &a = station_pts[station_i0];
    const Point2D &b = station_pts[station_i0 + 1];
    relative_path_heading = std::atan2(b.y - a.y, b.x - a.x);
  }
  relative_path_heading = wrapAngle(relative_path_heading);

  // Proximity speed governor: cap the sampled speed in front of points the
  // current motion would actually run into (slab test in evaluateProximity -
  // a wall parallel to the travel direction misses the body and leaves the
  // cruise speed alone), with a creep floor so an escape stays possible right
  // at the boundary.
  float v_cap    = params_.limits.v_max;
  float fraction = std::min(1.0f, proximity.governor);
  // Side envelope: while a point sits beside the body (or is about to be
  // passed) inside the unscaled side margin plus cushion, never sample a
  // speed whose own speed-scaled margin would reach within cushion of that
  // point - the emergency boundary must stay ahead of the sampled window,
  // not inside it.
  if (proximity.side_ratio < 1.0f + params_.side_envelope_headroom)
  {
    // Highest margin scale whose side margin still clears the gap, minus the
    // headroom cushion; inverted through the speed scaling. A gap the FULL
    // margin already clears (ratio >= 1 + headroom) never caps the speed.
    float scale_allowed = proximity.side_ratio - params_.side_envelope_headroom;
    float v_side = params_.margin_scale_speed * (scale_allowed - params_.margin_scale_floor) /
                   std::max(1.0f - params_.margin_scale_floor, 1e-3f);
    fraction = std::min(fraction, v_side / std::max(params_.limits.v_max, 1e-3f));
  }
  if (fraction < 1.0f)
  {
    fraction = std::max(params_.creep_fraction, std::min(1.0f, fraction));
    v_cap *= fraction;
  }

  float clearance_cap = params_.footprint.width / 2.0f + params_.avoid_margin.side;
  const detail::MotionModel *motion_model = motion_model_.get();

  // Tightness probe: best RAW bilateral clearance over three probe arcs.
  // Near 0 in the open (balance term off, pure path following); towards 1 in
  // passages tighter than cap + safety side margin (geometry takes over the
  // lateral authority).
  float tightness  = 0.0f;
  float probe_best = 0.0f;
  {
    float v_probe    = std::max(v_cap, 0.1f);
    float probe_dist = std::max(v_probe * params_.sim_time, params_.min_eval_distance);
    probe_dist       = std::min(probe_dist, remaining_path);
    for (const Twist2D &probe : motion_model->clearanceProbeCommands(v_probe))
    {
      const float wp = probe.w;
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

  // Cruise moderation: precise centering between close walls needs steering
  // authority (~w/v), so a fully tight passage caps the cruise at
  // tight_cruise_fraction of the limit (never below the creep floor).
  if (params_.tight_cruise_fraction < 1.0f)
  {
    float moderation = 1.0f - (1.0f - params_.tight_cruise_fraction) * tightness;
    v_cap = std::min(v_cap, params_.limits.v_max *
                                std::max(moderation, params_.creep_fraction));
  }

  const detail::CandidateBatch candidate_batch =
      motion_model->sampleCandidates(current, v_cap);

  // Rotation admissibility remains deliberately conservative: a full
  // in-place rotation sweeps the disk of the circumscribed radius.
  const bool rotation_admissible = motion_model->supportsInPlaceRotation() &&
                                   motion_model->isInPlaceRotationAdmissible(filtered_points);

  // A geometric path alone does not say that any collision-free forward
  // motion is useful. When the ordered path initially points almost behind
  // the body, or a nearby goal requires a compact side-step, first align the
  // body with the local path tangent. Hysteretic enter/exit thresholds avoid
  // alternating between rotate and translate at the boundary. If rotation is
  // obstructed, leave this mode and let a goal-progressing reverse candidate
  // (when configured and rear-observed) compete below.
  constexpr float kAlignExitAngle      = 0.25f;
  constexpr float kAlignNearEnterAngle = 0.70f;
  constexpr float kAlignRearEnterAngle = 2.60f;
  constexpr float kAlignNearDistance   = 1.00f;
  const float abs_path_heading = std::fabs(relative_path_heading);
  // Emergency escape has absolute priority: alignment removes every
  // translational candidate, while escape_only intentionally permits only a
  // translation away from the offending point. Combining the two would leave
  // the stop candidate as the sole survivor and latch forever.
  if (escape_only)
  {
    alignment_mode_ = false;
  }
  else if (alignment_mode_)
  {
    if (abs_path_heading < kAlignExitAngle || !rotation_admissible)
    {
      alignment_mode_ = false;
    }
  }
  else if (rotation_admissible &&
           ((local_goal_distance < kAlignNearDistance &&
             abs_path_heading > kAlignNearEnterAngle) ||
            abs_path_heading > kAlignRearEnterAngle))
  {
    alignment_mode_ = true;
  }
  const bool alignment_required = alignment_mode_ && rotation_admissible;

  // Reusable buffer for turn-then-go evaluation of the v=0 row
  std::vector<Point2D> rotated_points;

  // Adaptive saturation: aim for the configured avoid margin, but never DEMAND
  // more clearance than the passage towards the goal physically affords -
  // otherwise a corridor narrower than the cap scores permanently worse than
  // hesitating outside it (entry barrier).
  float cap_floor = params_.footprint.width / 2.0f + std::max(params_.safety_margin.side, 0.05f);
  float cap_eff   = std::min(clearance_cap, std::max(probe_best, cap_floor));

  float lead_margin   = params_.safety_margin.front;
  float stop_decel    = std::max(params_.stop_decel, 0.1f);

  float best_score = -FLT_MAX;
  Twist2D best_cmd(0.0f, 0.0f);
  float best_clearance = 0.0f, best_path_cost = 0.0f;
  int   admissible_count = 0, candidate_count = 0;
  int   forward_progressing = 0;  // gates reverse only when forward advances the ordered path

  auto evaluate_candidate = [&](float v, float w) {
      candidate_count++;
      if (alignment_required && std::fabs(v) > 1e-3f)
      {
        return;  // explicit rotate-before-translate mode
      }
      if (escape_only)
      {
        const bool rotation = std::fabs(v) <= 1e-3f && std::fabs(w) > 1e-3f;
        const bool towards  = v * proximity.emergency_x > 1e-6f;  // closing on the point
        if (rotation || towards)
        {
          return;  // emergency standstill: only motion away from the point, or stop
        }
      }

      // Endpoint pose and the point-free score parts first: they give an
      // upper bound (clearance <= cap_eff, balance/squeeze >= 0) that lets us
      // skip the expensive arc evaluation for candidates that cannot win.
      const detail::ProjectedPose2D projected =
          motion_model->projectConstantCommand(Twist2D(v, w), params_.sim_time);
      float end_th_pre = projected.theta;
      float end_x_pre = projected.x;
      float end_y_pre = projected.y;
      float gd_pre, bearing_pre, heading_scale_pre, candidate_progress;
      gd_pre = station_goal_cost(
          end_x_pre, end_y_pre, bearing_pre, heading_scale_pre, candidate_progress);
      // Collision-free forward motion that does not advance the ordered path
      // is not a navigation solution. In particular, this rejects the open-
      // space straight-ahead candidate when the complete path is behind.
      if (v > 1e-3f && candidate_progress <= 1e-4f && !escape_only)
      {
        return;
      }
      float fixed_penalties = params_.weights.path_dist * gd_pre +
                              params_.weights.hysteresis *
                                  motion_model->commandChange(Twist2D(v, w),
                                                              prev_selected_command_);
      // (Pruning waits until one admissible forward candidate is on record:
      // the reverse gate depends on GOAL PROGRESS, and admissibility is
      // only known after evaluation. The v=0 row is exempt - its goal
      // distance improves after evaluation via the turn-then-go advance, so
      // the pre-evaluation bound would not be an upper bound.)
      if (v > 1e-3f && forward_progressing > 0 &&
          params_.weights.clearance * cap_eff - fixed_penalties <= best_score)
      {
        return;
      }

      float clearance, lateral_fraction, clear_left, clear_right;
      if (std::fabs(v) <= 1e-3f)
      {
        if (std::fabs(w) > 1e-3f && !rotation_admissible)
        {
          return;
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
        dist         = std::min(dist, remaining_path);
        ArcEvaluation eval = evaluateArcWindows(rotated_points, v_ref, 0.0f, dist, dist);
        clearance        = std::min(eval.clearance_left, eval.clearance_right);
        lateral_fraction = eval.lateral_fraction;
        clear_left       = eval.far_left;
        clear_right      = eval.far_right;

        // The maneuver is turn-THEN-GO: its goal distance is scored at the
        // post-rotation ADVANCE endpoint (capped by the first body hit on
        // that run) PLUS the advance itself. A run straight at the goal then
        // scores exactly the in-place gd (the family baseline against real
        // moving candidates is unchanged), while a run pointing away pays up
        // to twice the advance. Without this the v=0 family is scored fully
        // in place, all sharing one goal distance, and a robot facing open
        // space away from the goal freezes: the phantom clearance of the run
        // it never makes rewards holding still, and the heading reward for
        // turning back is smaller than the hysteresis of starting to turn.
        float advance = std::min(v_ref * params_.sim_time, local_goal_distance);
        if (eval.blocking_s < FLT_MAX)
        {
          advance = std::max(0.0f, std::min(advance, eval.blocking_s - lead_margin));
        }
        end_x_pre = advance * std::cos(end_th_pre);
        end_y_pre = advance * std::sin(end_th_pre);
        gd_pre = station_goal_cost(
                     end_x_pre, end_y_pre, bearing_pre, heading_scale_pre,
                     candidate_progress) +
                 advance;
      }
      else
      {
        if (!motion_model->isCommandKinematicallyValid(Twist2D(v, w)))
        {
          return;
        }
        float dist_block = std::max(std::fabs(v) * params_.sim_time, params_.min_eval_distance);
        dist_block       = std::min(dist_block, remaining_path);
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
          // DWA admissibility: able to stop (keeping the directional safety
          // margin) before first contact. blocking_s already accounts for
          // the body extent (it is the body-origin travel at contact).
          float margin   = (v >= 0.0f) ? lead_margin : params_.safety_margin.rear;
          float free_run = eval.blocking_s - margin;
          float needed   = v * v / (2.0f * stop_decel) + std::fabs(v) * params_.brake_reaction_time;
          if (free_run <= needed)
          {
            return;
          }
        }
        clearance        = std::min(eval.clearance_left, eval.clearance_right);
        lateral_fraction = eval.lateral_fraction;
        clear_left       = eval.far_left;
        clear_right      = eval.far_right;
      }
      // Reverse candidates are evaluated through geometry/admissibility even
      // in ordinary tracking, but remain an escape alternative when a safe
      // forward candidate actually advances the ordered path.
      if (v < -1e-3f && forward_progressing > 0 && !escape_only)
      {
        return;
      }
      admissible_count++;
      if (v > 1e-3f && candidate_progress > 1e-4f)
      {
        forward_progressing++;
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
      float end_th      = end_th_pre;
      float path_cost   = gd_pre;
      const float motion_heading = end_th + (v < -1e-3f ? kPi : 0.0f);
      float heading_err = wrapAngle(bearing_pre - motion_heading);

      float score;
      if (alignment_required)
      {
        // While braking translational motion, do not start sweeping the body.
        // Once nearly stationary, choose the sampled in-place rotation that
        // most reduces the relative path heading over the rollout horizon.
        const float alignment_error = (std::fabs(current.v) > 0.05f)
                                          ? std::fabs(w)
                                          : std::fabs(wrapAngle(relative_path_heading - end_th));
        score = -alignment_error -
                0.05f * motion_model->commandChange(Twist2D(v, w),
                                                     prev_selected_command_);
      }
      else
      {
        score = params_.weights.clearance * std::min(clearance, cap_eff) -
                params_.weights.balance * tightness * balance -
                params_.weights.path_dist * path_cost -
                params_.weights.heading * heading_scale_pre * std::fabs(heading_err) -
                params_.weights.hysteresis *
                    motion_model->commandChange(Twist2D(v, w), prev_selected_command_) -
                params_.weights.squeeze * std::fabs(v) * (1.0f - lateral_fraction);
      }
#ifdef BAC_DEBUG_CANDIDATES
      std::printf("cand v=%.3f w=%6.3f clr=%6.3f gd=%6.3f he=%6.3f lat=%.2f score=%7.3f\n", v, w,
                  std::min(clearance, clearance_cap), path_cost, heading_err, lateral_fraction, score);
#endif
      if (score > best_score)
      {
        best_score     = score;
        best_cmd       = Twist2D(v, w);
        best_clearance = std::min(clearance, cap_eff);
        best_path_cost = path_cost;
      }
  };

  for (const Twist2D &command : candidate_batch.commands)
  {
    evaluate_candidate(command.v, command.w);
  }

  // Coarse-to-fine steering: re-sample w around the coarse winner at a finer
  // pitch (moving candidates only). The uniform grid quantizes w to
  // 2 * w_max / (w_samples - 1); the hysteresis term then makes the coarse
  // pitch itself the smallest applicable correction, which shows up as
  // tick-scale zigzag in narrow passages.
  for (const Twist2D &command : motion_model->refinementCandidates(best_cmd))
  {
    evaluate_candidate(command.v, command.w);
  }

  float out_v = best_cmd.v;
  float out_w = best_cmd.w;

  // Output reachability: the plant cannot jump to arbitrary yaw rate or road-
  // wheel angle in one cycle. If the reachable arc needs a lower speed to
  // stop before contact, reduce v and w together to preserve its curvature.
  // For differential drive at a large current yaw rate, that proportional w
  // can itself lie outside the one-cycle deceleration interval. Reapply the
  // reachability limit and recheck the resulting arc until it is admissible.
  const Twist2D limited =
      motion_model->limitReachableCommand(current, Twist2D(out_v, out_w));
  if (std::fabs(limited.v - out_v) > 1e-4f || std::fabs(limited.w - out_w) > 1e-4f)
  {
    out_v = limited.v;
    out_w = limited.w;
    bool output_admissible = true;
    constexpr int kReachabilityIterations = 8;
    for (int iteration = 0; iteration < kReachabilityIterations; ++iteration)
    {
      output_admissible = true;
      if (std::fabs(out_v) <= 1e-3f)
      {
        if (std::fabs(out_w) > 1e-4f && !rotation_admissible)
        {
          out_w = 0.0f;
        }
        break;
      }

      // The exact contact test is valid for any radius, including below
      // turn_radius_min (that guard exists for scoring, not for contact).
      float dist_block = std::max(std::fabs(out_v) * params_.sim_time, params_.min_eval_distance);
      dist_block       = std::min(dist_block, remaining_path);
      const ArcEvaluation ev = evaluateArcWindows(filtered_points, out_v, out_w, 0.0f, dist_block);
      if (ev.blocking_s >= FLT_MAX)
      {
        break;
      }

      const float margin = (out_v >= 0.0f) ? lead_margin : params_.safety_margin.rear;
      const float free_run = ev.blocking_s - margin;
      const float a = std::max(params_.stop_decel, 0.1f);
      const float tr = params_.brake_reaction_time;
      const float v_safe = free_run > 0.0f
                               ? a * (std::sqrt(tr * tr + 2.0f * free_run / a) - tr)
                               : 0.0f;
      if (std::fabs(out_v) <= v_safe + 1e-4f)
      {
        break;
      }

      output_admissible = false;
      const float safe_speed = (out_v > 0.0f ? 1.0f : -1.0f) * v_safe;
      const Twist2D curvature_preserving =
          motion_model->withLinearSpeed(Twist2D(out_v, out_w), safe_speed);
      const Twist2D next = motion_model->limitReachableCommand(current, curvature_preserving);
      if (std::fabs(next.v - out_v) <= 1e-5f && std::fabs(next.w - out_w) <= 1e-5f)
      {
        break;
      }
      out_v = next.v;
      out_w = next.w;
    }

    if (!output_admissible)
    {
      // No simultaneously curvature-preserving, contact-admissible, and
      // one-cycle-reachable translating command was found. Brake translation;
      // differential drive may retain only a safe reachable rotation.
      const Twist2D braking =
          motion_model->limitReachableCommand(current, Twist2D(0.0f, 0.0f));
      out_v = 0.0f;
      out_w = (std::fabs(braking.w) <= 1e-4f || rotation_admissible) ? braking.w : 0.0f;
    }
  }
  const Twist2D finalized = motion_model->applyCommandDeadband(Twist2D(out_v, out_w));
  out_v = finalized.v;
  out_w = finalized.w;
  prev_selected_command_ = Twist2D(out_v, out_w);
  result.output         = Twist2D(out_v, out_w);
  result.best_clearance = best_clearance;
  result.best_path_cost = best_path_cost;
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
