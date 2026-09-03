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

/// Distance the body still covers before a full stop from `speed`: one
/// deceleration ramp plus the run made during the brake reaction time. The
/// floor on stop_decel is part of the definition - a zero or negative
/// configured deceleration would otherwise make the distance infinite or
/// negative. `speed` is a SPEED along the direction of travel, never a signed
/// forward velocity: a holonomic body crabbing sideways brakes over the same
/// distance as one running forward (R15 H3).
float
brakingDistance(float speed, const Params &params)
{
  const float stop_decel = std::max(params.stop_decel, 0.1f);
  return speed * speed / (2.0f * stop_decel) + speed * params.brake_reaction_time;
}

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

  // The zone extends along the DIRECTION OF TRAVEL. A holonomic body moving
  // sideways has the same braking distance as one moving forward, and both
  // the zone and the speed-dependent margin scaling have to see it (R15 H3):
  // computing either from `current.v` alone classifies a body crabbing at
  // vy_max as stationary.
  const float current_speed = current.speed();
  const float brake_distance = brakingDistance(current_speed, params);
  const float body_x_min = params.footprint.rear;
  const float body_x_max = params.footprint.front;
  float       zone_x_min = body_x_min;
  float       zone_x_max = body_x_max;
  // The body's physical half-width. The emergency zone's inflation along the
  // direction of travel lives in zone_y_min / zone_y_max below, never in this;
  // it stays the bare footprint everywhere it is read.
  const float body_y_half = params.footprint.width / 2.0f;
  float       zone_y_min = -body_y_half;
  float       zone_y_max = body_y_half;
  if (current_speed > 1e-6f)
  {
    const float ux = current.v / current_speed;
    const float uy = current.vy / current_speed;
    if (ux >= 0.0f)
    {
      zone_x_max += brake_distance * ux;
    }
    else
    {
      zone_x_min += brake_distance * ux;
    }
    if (uy >= 0.0f)
    {
      zone_y_max += brake_distance * uy;
    }
    else
    {
      zone_y_min += brake_distance * uy;
    }
  }

  // Unit direction of travel and the body's extents along it, used by the
  // governor slab test below. Reduces to (+-1, 0) with lead = front (or -rear)
  // plus the braking distance, and half-extent = width / 2, whenever vy == 0.
  float travel_ux = (current.v >= 0.0f) ? 1.0f : -1.0f;
  float travel_uy = 0.0f;
  if (current_speed > 1e-6f)
  {
    travel_ux = current.v / current_speed;
    travel_uy = current.vy / current_speed;
  }
  const float travel_nx = -travel_uy;
  const float travel_ny = travel_ux;
  // The footprint's support function, shared with the swept-arc frame in
  // arc_trajectory_evaluator.cpp. The governor and the arc evaluator are
  // required to measure the body the same way; a second copy of the formula
  // here let them drift apart silently.
  const auto support = [&](float dx, float dy) {
    return detail::supportExtent(params.footprint, dx, dy);
  };
  const float travel_lead = support(travel_ux, travel_uy) + brake_distance;
  // The swept box's lateral interval is [-travel_right, travel_left] and is NOT
  // symmetric about the travel axis: the two differ by (front + rear) * uy.
  // Using one half-width for both symmetrised it, which left the under-braking
  // this generalisation exists to remove on one side and over-capped the other
  // (R17 H1). The arc evaluator has always kept these apart as perp_left and
  // perp_right; the governor now does too. Both equal width / 2 when vy == 0.
  const float travel_left = support(travel_nx, travel_ny);
  const float travel_right = support(-travel_nx, -travel_ny);

  const float speed_scale = std::min(
      1.0f, params.margin_scale_floor +
                (1.0f - params.margin_scale_floor) * current_speed /
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

    // The lateral half of the emergency zone extends the same way the
    // longitudinal half does, so a point directly abeam of a crabbing body is
    // as urgent as one ahead of a body driving forward. Identical to the
    // previous fixed half-width whenever vy == 0.
    float dy_normalized = 0.0f;
    if (point.y > zone_y_max)
    {
      dy_normalized = (point.y - zone_y_max) / margin_side;
    }
    else if (point.y < zone_y_min)
    {
      dy_normalized = (zone_y_min - point.y) / margin_side;
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
    // In the DIRECTION OF TRAVEL, not along +x. Reading the forward axis alone
    // put a wall the body was crabbing into on the side-envelope branch rather
    // than the head-on one, so the same wall approached sideways was governed
    // 32% less than approached head-on (R16 H3). For vy == 0 these reduce to
    // the previous expressions exactly: travel_ux is +-1, travel_uy is 0.
    const float travel_s   = point.x * travel_ux + point.y * travel_uy;
    const float travel_lat = point.x * travel_nx + point.y * travel_ny;
    const float ahead      = travel_s - travel_lead;
    const float lookahead  = std::max(params.side_envelope_lookahead, 1e-3f);
    const float side_gap =
        (travel_lat >= 0.0f) ? travel_lat - travel_left : -travel_lat - travel_right;
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
      // Behind the trailing edge, again in the direction of travel.
      const float behind = -travel_s - support(-travel_ux, -travel_uy);
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
  return evaluateArcWindows(points, Twist2D(v, w), dist_clear, dist_block);
}

ArcEvaluation
BacCore::evaluateArcWindows(const std::vector<Point2D> &points, const Twist2D &command,
                            float dist_clear, float dist_block) const
{
  const detail::ArcTrajectoryEvaluator evaluator(params_);
  return evaluator.evaluate(points, command, dist_clear, dist_block);
}

Result
BacCore::process(const std::vector<Point2D> &points, const std::vector<Point2D> &path,
                 const Twist2D &current, std::optional<float> goal_heading)
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
    if (current.speed() > 0.05f || std::fabs(current.w) > 0.1f)
    {
      // Braking has priority while moving - in ANY direction. Testing
      // `current.v` alone classified a body crabbing at limits.vy_max as
      // stationary and skipped the hard emergency brake (R15 H3).
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

  // How far ahead any arc is evaluated for a given speed: one sim_time of
  // travel, never shorter than min_eval_distance, never past the end of the
  // path. One definition, used by the tightness probe, the turn-then-go row,
  // every moving candidate, and the stopping test.
  const auto eval_window = [&](float speed) {
    return std::min(std::max(speed * params_.sim_time, params_.min_eval_distance),
                    remaining_path);
  };

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
  // Bilateral imbalance seen by the straight probe when it is threading a
  // passage. Used below to bias the pose regulator of a model that does not
  // search over yaw.
  float probe_center_bias = 0.0f;
  {
    float v_probe    = std::max(v_cap, 0.1f);
    const float probe_dist = eval_window(v_probe);
    for (const Twist2D &probe : motion_model->clearanceProbeCommands(v_probe))
    {
      const float wp = probe.w;
      float dist_clear = probe_dist;
      if (std::fabs(wp) > 1e-4f)
      {
        dist_clear = std::min(dist_clear, (v_probe / std::fabs(wp)) * params_.eval_angle_max);
      }
      // The probe command carries the model's own shape - a holonomic model
      // probes crabbed directions - so it is evaluated whole, not as (v, w).
      const Twist2D probe_command(v_probe, wp, probe.vy);
      ArcEvaluation eval =
          evaluateArcWindows(filtered_points, probe_command, dist_clear, probe_dist);
      probe_best = std::max(probe_best, std::min(eval.clearance_left, eval.clearance_right));
      // A PASSAGE, not an obstacle: bounded on both sides within the cap, and
      // open straight ahead. The second half is what separates a corridor from
      // a box in the road - for a box, the two "sides" are the obstacle's own
      // edges, so balancing them steers into it. Measured without this gate:
      // the detour run stops 3.8 m short of the goal and yaws MORE than the
      // differential-drive reference (0.443 vs 0.313 rad/s).
      if (std::fabs(wp) <= 1e-4f && std::fabs(probe.vy) <= 1e-4f &&
          eval.blocking_s >= FLT_MAX && eval.clearance_left < clearance_cap &&
          eval.clearance_right < clearance_cap)
      {
        const float total = eval.clearance_left + eval.clearance_right;
        probe_center_bias = (total > 1e-3f)
                                ? (eval.clearance_left - eval.clearance_right) / total
                                : 0.0f;
      }
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

  // Pose regulation. A holonomic model does not search over yaw - lateral
  // velocity does the avoiding - so it needs the yaw rate the body should hold
  // this tick. Proportional on the heading error to the local path tangent,
  // saturated at the configured yaw limit; the output stage still applies the
  // one-cycle acceleration bound. Non-holonomic models ignore it.
  // In a passage, point the body INTO the gap rather than crabbing towards it.
  // Both reach the same place, but a crabbing rectangle sweeps wider than a
  // straight one - 0.86 m at 55 degrees for a 0.7 x 0.5 m body against 0.5 m
  // going straight - so exactly where lateral room is scarce, crabbing is the
  // expensive way to use it. Scaled by tightness, and gated to passages at the
  // probe, this is inactive in the open and inactive in front of an obstacle,
  // where the candidate search does the avoiding.
  constexpr float kCenteringHeading = 0.6f;  // [rad] at full imbalance
  // Wrapped: past +-pi the proportional error stops being the short way round,
  // and the body would take the long way while inside a narrow passage - the
  // opposite of what this term exists for (R15 M9).
  float pose_reference =
      wrapAngle(relative_path_heading + tightness * kCenteringHeading * probe_center_bias);

  // Arriving in the orientation the goal asks for. A model that steers with
  // yaw cannot choose its orientation independently of where it is going, so
  // this is the one thing a holonomic body can do that the others cannot: hold
  // the goal heading while lateral velocity closes the remaining position
  // error. Faded in over the last kGoalHeadingFade metres so the tangent still
  // governs the approach; blending the two ERRORS is well defined because both
  // are expressed in the current body frame.
  if (goal_heading && motion_model->acceptsGoalHeading())
  {
    // Full authority over the last kGoalHeadingFull metres, faded in from
    // kGoalHeadingFade. The vehicle has to ARRIVE in the goal orientation, not
    // reach the goal and then turn: a controller that still had to rotate on
    // arrival would sit outside Nav2's yaw_goal_tolerance while the goal
    // checker waited for it.
    constexpr float kGoalHeadingFade = 1.5f;  // [m] where the goal starts to matter
    constexpr float kGoalHeadingFull = 0.5f;  // [m] where it fully governs
    const float blend =
        1.0f - std::max(0.0f, std::min(1.0f, (local_goal_distance - kGoalHeadingFull) /
                                                 (kGoalHeadingFade - kGoalHeadingFull)));
    pose_reference =
        wrapAngle(pose_reference + blend * wrapAngle(*goal_heading - pose_reference));
  }
  const float heading_error = pose_reference;
  const float yaw_reference =
      std::min(std::max(params_.heading_gain * heading_error, -params_.limits.w_max),
               params_.limits.w_max);

  const detail::CandidateBatch candidate_batch =
      motion_model->sampleCandidates(current, v_cap, yaw_reference);

  // Rotation admissibility remains deliberately conservative: a full
  // in-place rotation sweeps the disk of the circumscribed radius.
  // `isInPlaceRotationAdmissible` alone: a model that cannot rotate on the
  // spot answers false unconditionally (see AckermannMotionModel), so the
  // capability predicate this used to be conjoined with could never differ
  // from its right-hand side. It saved nothing and has since been removed
  // from MotionModel entirely.
  const bool rotation_admissible =
      motion_model->isInPlaceRotationAdmissible(filtered_points);
  // Rotating onto the tangent before translating is a differential-drive
  // manoeuvre. A holonomic body can rotate on the spot but never needs to
  // align first, so the two predicates are asked separately.
  const bool alignment_available =
      motion_model->usesRotateBeforeTranslate() && rotation_admissible;

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
    if (abs_path_heading < kAlignExitAngle || !alignment_available)
    {
      alignment_mode_ = false;
    }
  }
  else if (alignment_available &&
           ((local_goal_distance < kAlignNearDistance &&
             abs_path_heading > kAlignNearEnterAngle) ||
            abs_path_heading > kAlignRearEnterAngle))
  {
    alignment_mode_ = true;
  }
  const bool alignment_required = alignment_mode_ && alignment_available;

  // Reusable buffer for turn-then-go evaluation of the v=0 row
  std::vector<Point2D> rotated_points;

  // Adaptive saturation: aim for the configured avoid margin, but never DEMAND
  // more clearance than the passage towards the goal physically affords -
  // otherwise a corridor narrower than the cap scores permanently worse than
  // hesitating outside it (entry barrier).
  float cap_floor = params_.footprint.width / 2.0f + std::max(params_.safety_margin.side, 0.05f);
  float cap_eff   = std::min(clearance_cap, std::max(probe_best, cap_floor));

  float lead_margin   = params_.safety_margin.front;

  // Margin facing the direction of travel. This is the support function of the
  // margin box in that direction, so a forward command sees safety_margin.front
  // exactly, a reversing one safety_margin.rear, and a purely lateral one
  // safety_margin.side. Only a holonomic model can produce the blended case.
  const auto travel_margin = [&](const Twist2D &command) {
    const float speed = command.speed();
    if (speed <= 1e-6f)
    {
      return lead_margin;
    }
    const float ux = command.v / speed;
    const float uy = command.vy / speed;
    const float along = (ux >= 0.0f) ? lead_margin * ux : params_.safety_margin.rear * -ux;
    return along + params_.safety_margin.side * std::fabs(uy);
  };

  float best_score = -FLT_MAX;
  Twist2D best_cmd(0.0f, 0.0f);
  float best_clearance = 0.0f, best_path_cost = 0.0f;
  int   admissible_count = 0, candidate_count = 0;
  int   forward_progressing = 0;  // gates reverse only when forward advances the ordered path

  auto evaluate_candidate = [&](const Twist2D &command) {
      // The scalars stay so the scoring body reads as it did; `command` is
      // what reaches the motion model and the swept-geometry evaluator, so a
      // holonomic candidate is scored on the trajectory it actually drives.
      const float v = command.v;
      const float w = command.w;
      candidate_count++;
      if (alignment_required && std::fabs(v) > 1e-3f)
      {
        return;  // explicit rotate-before-translate mode
      }
      if (escape_only)
      {
        const bool rotation = command.speed() <= 1e-3f && std::fabs(w) > 1e-3f;
        // Closing on the offending point, measured on the whole velocity
        // vector. Projecting only `v` onto `emergency_x` let a pure crab move
        // straight at a point directly abeam of the body (R15 H3).
        const bool towards =
            v * proximity.emergency_x + command.vy * proximity.emergency_y > 1e-6f;
        if (rotation || towards)
        {
          return;  // emergency standstill: only motion away from the point, or stop
        }
      }

      // Endpoint pose and the point-free score parts first: they give an
      // upper bound (clearance <= cap_eff, balance/squeeze >= 0) that lets us
      // skip the expensive arc evaluation for candidates that cannot win.
      const detail::ProjectedPose2D projected =
          motion_model->projectConstantCommand(command, params_.sim_time);
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
                                  motion_model->commandChange(command, prev_selected_command_);
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
      // Not translating at all. `command.v` alone is the wrong test for a
      // holonomic candidate: the v = 0 ROW of the lattice still translates,
      // sideways, and scoring it as a turn-then-go rotation would route the
      // whole family of pure-crab candidates around the contact test and the
      // stopping test below (R15 H1).
      if (command.speed() <= 1e-3f)
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
        const float dist = eval_window(v_ref);
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
        if (!motion_model->isCommandKinematicallyValid(command))
        {
          return;
        }
        const float candidate_speed = command.speed();
        const float dist_block = eval_window(candidate_speed);
        float dist_clear = dist_block;
        if (std::fabs(w) > 1e-4f)
        {
          float radius = candidate_speed / std::fabs(w);
          dist_clear   = std::min(dist_clear, radius * params_.eval_angle_max);
          if (params_.eval_lateral_max < radius)
          {
            dist_clear = std::min(
                dist_clear, radius * std::acos(1.0f - params_.eval_lateral_max / radius));
          }
        }
        ArcEvaluation eval = evaluateArcWindows(filtered_points, command, dist_clear, dist_block);
        if (eval.blocking_s < FLT_MAX)
        {
          // DWA admissibility: able to stop (keeping the directional safety
          // margin) before first contact. blocking_s already accounts for
          // the body extent (it is the body-origin travel at contact).
          // Along the DIRECTION OF TRAVEL, not the forward axis. Computing
          // either from `v` alone admits a pure crab with zero braking
          // distance and the front margin instead of the side one (R15 H2).
          float margin   = travel_margin(command);
          float free_run = eval.blocking_s - margin;
          float needed   = brakingDistance(candidate_speed, params_);
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
                0.05f * motion_model->commandChange(command,
                                                     prev_selected_command_);
      }
      else
      {
        score = params_.weights.clearance * std::min(clearance, cap_eff) -
                params_.weights.balance * tightness * balance -
                params_.weights.path_dist * path_cost -
                params_.weights.heading * heading_scale_pre * std::fabs(heading_err) -
                params_.weights.hysteresis *
                    motion_model->commandChange(command, prev_selected_command_) -
                params_.weights.squeeze * std::fabs(v) * (1.0f - lateral_fraction);
      }
#ifdef BAC_DEBUG_CANDIDATES
      std::printf("cand v=%.3f w=%6.3f clr=%6.3f gd=%6.3f he=%6.3f lat=%.2f score=%7.3f\n", v, w,
                  std::min(clearance, clearance_cap), path_cost, heading_err, lateral_fraction, score);
#endif
      if (score > best_score)
      {
        best_score     = score;
        best_cmd       = command;
        best_clearance = std::min(clearance, cap_eff);
        best_path_cost = path_cost;
      }
  };

  for (const Twist2D &command : candidate_batch.commands)
  {
    evaluate_candidate(command);
  }

  // Coarse-to-fine steering: re-sample w around the coarse winner at a finer
  // pitch (moving candidates only). The uniform grid quantizes w to
  // 2 * w_max / (w_samples - 1); the hysteresis term then makes the coarse
  // pitch itself the smallest applicable correction, which shows up as
  // tick-scale zigzag in narrow passages.
  for (const Twist2D &command : motion_model->refinementCandidates(best_cmd))
  {
    evaluate_candidate(command);
  }

  float out_v = best_cmd.v;
  float out_w = best_cmd.w;
  float out_vy = best_cmd.vy;
  const auto out_command = [&]() { return Twist2D(out_v, out_w, out_vy); };

  // DWA admissibility of one emitted twist: can it stop, along its own
  // direction of travel and keeping the directional margin, before it touches
  // anything? One definition, used by the emergency fallback below.
  // Largest speed this twist could carry and still stop before contact, or
  // FLT_MAX when its arc is contact-free within the window.
  const auto safe_speed_for = [&](const Twist2D &command) {
    const float speed = command.speed();
    if (speed <= 1e-3f)
    {
      return FLT_MAX;
    }
    const float dist_block = eval_window(speed);
    const ArcEvaluation ev =
        evaluateArcWindows(filtered_points, command, 0.0f, dist_block);
    if (ev.blocking_s >= FLT_MAX)
    {
      return FLT_MAX;
    }
    const float free_run = ev.blocking_s - travel_margin(command);
    const float a = std::max(params_.stop_decel, 0.1f);
    const float tr = params_.brake_reaction_time;
    return free_run > 0.0f ? a * (std::sqrt(tr * tr + 2.0f * free_run / a) - tr) : 0.0f;
  };
  const auto stoppable = [&](const Twist2D &command) {
    return command.speed() <= safe_speed_for(command) + 1e-4f;
  };


  // Output reachability: the plant cannot jump to arbitrary yaw rate or road-
  // wheel angle in one cycle. If the reachable arc needs a lower speed to
  // stop before contact, reduce v and w together to preserve its curvature.
  // For differential drive at a large current yaw rate, that proportional w
  // can itself lie outside the one-cycle deceleration interval. Reapply the
  // reachability limit and recheck the resulting arc until it is admissible.
  const Twist2D limited =
      motion_model->limitReachableCommand(current, out_command());
  if (std::fabs(limited.v - out_v) > 1e-4f || std::fabs(limited.w - out_w) > 1e-4f ||
      std::fabs(limited.vy - out_vy) > 1e-4f)
  {
    out_v = limited.v;
    out_w = limited.w;
    out_vy = limited.vy;
    bool output_admissible = true;
    constexpr int kReachabilityIterations = 8;
    for (int iteration = 0; iteration < kReachabilityIterations; ++iteration)
    {
      output_admissible = true;
      if (out_command().speed() <= 1e-3f)
      {
        if (std::fabs(out_w) > 1e-4f && !rotation_admissible)
        {
          out_w = 0.0f;
        }
        break;
      }

      // The exact contact test is valid for any radius, including below
      // turn_radius_min (that guard exists for scoring, not for contact).
      // ONE definition of stoppability, shared with the emergency fallback
      // below: `safe_speed_for` returns FLT_MAX for a contact-free arc, so the
      // comparison accepts it and the loop breaks, exactly as the former
      // inline copy of this computation did.
      const float v_safe = safe_speed_for(out_command());
      if (out_command().speed() <= v_safe + 1e-4f)
      {
        break;
      }

      output_admissible = false;
      // A SPEED, not a signed forward speed. The direction of travel is the
      // model's business: deriving a sign from out_v alone flipped both the
      // lateral velocity and the yaw rate for a v = 0 winner, turning the
      // vehicle around instead of slowing it down (R15 H2 derivative).
      const Twist2D curvature_preserving =
          motion_model->withLinearSpeed(out_command(), v_safe);
      const Twist2D next = motion_model->limitReachableCommand(current, curvature_preserving);
      if (std::fabs(next.v - out_v) <= 1e-5f && std::fabs(next.w - out_w) <= 1e-5f &&
          std::fabs(next.vy - out_vy) <= 1e-5f)
      {
        break;
      }
      out_v = next.v;
      out_w = next.w;
      out_vy = next.vy;
    }

    if (!output_admissible)
    {
      // No simultaneously curvature-preserving, contact-admissible, and
      // one-cycle-reachable translating command was found. Brake translation;
      // differential drive may retain only a safe reachable rotation.
      const Twist2D braking =
          motion_model->limitReachableCommand(current, Twist2D(0.0f, 0.0f, 0.0f));
      out_v = 0.0f;
      // What one deceleration window leaves of the previous motion: a residual
      // rotation and a residual sideways slide. Both are commands like any
      // other, so the pair is kept only while the COMBINED twist passes the
      // same stopping test. Testing the lateral component on its own is not
      // enough - the retained yaw bends it onto an arc, and it was that arc,
      // not the straight slide, that could no longer stop (R15 H3).
      const float braking_w =
          (std::fabs(braking.w) <= 1e-4f || rotation_admissible) ? braking.w : 0.0f;
      // Two outcomes, not three. A standstill rotation has zero SPEED, so
      // `stoppable` returns true for it immediately and a third rung below
      // this one could never be reached (R16 M3). The rotation itself is
      // already gated on `rotation_admissible` above.
      if (stoppable(Twist2D(0.0f, braking_w, braking.vy)))
      {
        out_vy = braking.vy;
        out_w = braking_w;
      }
      else
      {
        out_vy = 0.0f;
        out_w = braking_w;
      }
    }
  }
  // The deadband runs AFTER every admissibility check, so left alone it can
  // publish a twist nobody checked: zeroing a yaw rate below angvel_min
  // straightens a crabbing arc into a different trajectory, and for a
  // holonomic body that rewrites 30% of ticks (R16 H2). Re-check what is
  // actually published and do not apply the deadband if it changed the command
  // into something that can no longer stop.
  //
  // R18 M7: this reaches differential drive too. An earlier revision of this
  // comment said non-holonomic commands were unaffected, on the grounds that
  // zeroing a sub-deadband yaw rate does not change a straight-line arc. It
  // does change the arc whenever the yaw rate is what makes the command
  // admissible.
  //
  // R19 M9: the branch below IS reached at the shipped angvel_min of
  // 0.01 rad/s, and how often depends entirely on the tick generator. Measured
  // against main (2488248) with identical randomised differential-drive tick
  // streams fed to both revisions, 400000 ticks each at the shipped
  // angvel_min, seed 12345 unless the row says a second seed (987654321),
  // branch reaches counted from a scratch copy of this file. The generators
  // are described by shape rather than shipped, so the COUNTS are not
  // reproducible verbatim from this comment; the qualitative result is.
  //
  //   generator                                   deadband  branch  rows
  //                                               changed   reached differ
  //   corridor + close frontal point, current w
  //     drawn from +-[0.085, 0.124] rad/s            19559     182     193
  //   the same, second seed                          19624     190     192
  //   the same corridor, current w uniform +-1        2151      11      11
  //   frontal wall with a gap, current w +-0.13      10476      13      13
  //   one cluster at a random bearing, current w
  //     uniform +-1 rad/s                             3033       1       1
  //
  // Reaching it needs the yaw rate ALONE to be rounded away while the speed
  // survives, so the CURRENT yaw rate has to sit just below one control
  // period of yaw authority (acc_w * control_period = 0.125 rad/s), where
  // limitReachableCommand leaves a residual of a few thousandths, or where the
  // curvature-preserving slowdown produces one. Over the 397 reaches above,
  // |current.w| was in [0.0863, 0.1301] rad/s without exception. The same
  // corridor generator over 100000 ticks reaches the branch 43 times drawing
  // the current yaw rate from that band, 3 times drawing it uniformly over
  // +-1 rad/s, and 0 times over +-0.03 rad/s.
  //
  // So a measured "0 rows differ" means the generator under-sampled that band,
  // NOT that the behaviour is unchanged. An earlier revision of this comment
  // reported 0 rows over 40000 ticks of the last generator above and read it
  // as a property of the change; that generator does give 0 at 40000 ticks
  // (the deadband still changes the command 297 times) and reaches the branch
  // once at 400000. See CHANGELOG.rst.
  const Twist2D finalized = motion_model->applyCommandDeadband(out_command());
  const bool deadband_changed = std::fabs(finalized.v - out_v) > 1e-6f ||
                                std::fabs(finalized.w - out_w) > 1e-6f ||
                                std::fabs(finalized.vy - out_vy) > 1e-6f;
  if (deadband_changed && !stoppable(finalized))
  {
    // The deadband is a quantization that suppresses jitter, not a safety
    // requirement, so when it would break admissibility it simply does not
    // apply. Every observed case is a yaw rate below angvel_min being removed:
    // over the evaluation window that straightens the arc enough to clip
    // something, while the command as selected was admissible.
    //
    // Braking to a standstill instead repeated the same decision every tick
    // and made an absorbing state (R17 H2). Slowing to a speed the STRAIGHTENED
    // arc could stop at is not available either, but not for the reason an
    // earlier revision of this comment gave: measured over the holonomic suite,
    // 171 of 267 events have safe_speed_for(finalized) == 0, so for those there
    // is no such speed - but the other 96 do have one, from 0.0015 to 0.671 m/s
    // (R18 H3 corrected an earlier claim of 14 of 14 here). What rules it out
    // is that it would slow the vehicle to satisfy a quantization the vehicle
    // does not have to satisfy, when the command as selected is already
    // admissible and is what the scorer chose.
    // R18 M8: this inner brake is defence in depth and is not reached in any
    // measured configuration - 0 of 4112 events over 360k randomised ticks that
    // also swept angvel_min and velocity_min. It is kept because the outer
    // condition does not by itself establish that the command as selected is
    // admissible; no assertion covers it.
    if (!stoppable(out_command()))
    {
      out_v = 0.0f;
      out_w = 0.0f;
      out_vy = 0.0f;
    }
    // else: keep the command as selected, deadband not applied.
  }
  else
  {
    out_v = finalized.v;
    out_w = finalized.w;
    out_vy = finalized.vy;
  }
  prev_selected_command_ = out_command();
  result.output         = out_command();
  result.best_clearance = best_clearance;
  result.best_path_cost = best_path_cost;
  result.admissible_count = admissible_count;
  result.candidate_count  = candidate_count;

  // A holding-still report has to mean holding still in EVERY axis. Testing
  // `out_v` and `out_w` alone reported STOP while the body was still sliding
  // sideways, which `avoid_status` subscribers and the filter node's
  // arbitration both act on (R16 M4).
  if (out_command().speed() == 0.0f && out_w == 0.0f)
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
