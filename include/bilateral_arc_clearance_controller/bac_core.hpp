/**
 * @file bac_core.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Framework-free core of the arc-clearance local planner (DWA-based)
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Self-contained C++17 library: no ROS, no shm, no project-specific headers.
 * Inputs are obstacle points and a LOCAL PATH, both in the robot frame, plus
 * the current velocity; output is the next (v, w) command.
 *
 * The method is a Dynamic Window Approach whose traversability evaluation is
 * replaced by the bilateral arc clearance measure: candidate velocities are
 * sampled from an acceleration-limited translational window and the configured
 * angular range, rolled out as constant-curvature
 * arcs, and scored by
 *   saturated bilateral clearance + local-goal following (distance / heading)
 *   - hysteresis - lateral squeeze,
 * with DWA admissibility (can the robot stop before the first body hit on the
 * arc) and an emergency-stop layer in front of everything.
 *
 * v1 of this core took an upper-level velocity command (v, w) as the intent.
 * That representation broke down under nav2: a feedback-type steering command
 * (pure pursuit) extrapolated as a persistent arc reads as infeasible in
 * corridors, collapsing the intent's weight. The intent is therefore a path.
 *
 * Wrapped by the nav2 controller plugin / cmd_vel filter node of this
 * package, or embeddable in any 20Hz-ish control loop. Behavior is verified
 * by the closed-loop scenario harness in test/.
 */

#pragma once
#ifndef BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CORE_HPP_
#define BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CORE_HPP_

#include <vector>

namespace bac
{

struct Point2D
{
  float x = 0.0f;
  float y = 0.0f;

  Point2D() = default;
  Point2D(float x_val, float y_val)
    : x(x_val)
    , y(y_val)
  {
  }
};

struct Twist2D
{
  float v = 0.0f;  // forward velocity [m/s]
  float w = 0.0f;  // angular velocity [rad/s], CCW positive

  Twist2D() = default;
  Twist2D(float v_val, float w_val)
    : v(v_val)
    , w(w_val)
  {
  }
};

enum class Status : int
{
  CLEAR    = 0,  // nothing within influence: a pass-through arbitration may apply
  AVOIDING = 1,  // obstacle-shaped output
  STOP     = 2   // emergency stop (obstacle inside the stop zone, or no way out)
};

/// Robot body extent in the robot frame. rear is NEGATIVE (x of the rear edge).
struct Footprint
{
  float front = 0.5f;
  float rear  = -0.5f;
  float width = 0.95f;
};

struct Margins
{
  float front = 0.2f;
  float rear  = 0.2f;
  float side  = 0.2f;
};

/// Lateral clearance at which the arc-clearance reward saturates.
struct AvoidMargin
{
  float side = 0.6f;
};

/// Axis-aligned box of points to ignore (sensor returns from the robot itself)
struct IgnoreBox
{
  float front = 0.0f;  // ignore x < front ...
  float back  = 0.0f;  // ... and x > -back ...
  float width = 0.0f;  // ... and |y| < width/2
};

/// Velocity and acceleration limits used for candidate generation.
struct Limits
{
  float v_max = 0.4f;  // [m/s]
  /// Reverse speed floor. Small reverse candidates are offered only when the
  /// robot is near standstill (escape from a wedge without an external
  /// recovery). Requires rear sensor coverage - set 0 for front-only sensors.
  float v_min = -0.1f;  // [m/s]
  float w_max = 1.0f;  // [rad/s]
  float acc_v = 0.8f;  // [m/s^2]
};

/// Scoring weights. Balance rationale (see the harness scenarios):
///  - clearance is the traversability measure (saturates at width/2 +
///    avoid_margin.side): in the open many candidates saturate and the path
///    terms decide; in tight spaces its max-min structure funnels through the
///    middle of the opening.
///  - goal_dist supplies the restoring force towards the lookahead point and
///    rewards path-advancing candidates (speed selection).
///  - hysteresis damps tick-to-tick steering chatter; keep it well below the
///    path terms or the previous choice can pin the robot off-path.
struct Weights
{
  float clearance  = 2.0f;  // [score per m] saturated bilateral clearance
  float goal_dist  = 1.0f;  // [score per m] endpoint distance to the local goal
  /// Bilateral balance: penalty on |left - right| clearance (both capped),
  /// active only in tight spaces (scaled by the probed tightness). Provides a
  /// FIRST-ORDER centering gradient from the geometry itself - the local-goal
  /// distance is only second-order in the lateral error, and the goal (a
  /// grid-quantized, replanned global path) can itself sit off-center.
  float balance    = 4.0f;  // [score per m]
  float heading    = 0.15f; // [score per rad] endpoint heading vs local-goal bearing
  float hysteresis = 0.6f;  // [score per rad/s] change from the previous w
  float squeeze    = 0.5f;  // [score per m/s] v scaled by (1 - lateral_fraction)
};

struct Params
{
  Footprint footprint{};
  Margins   safety_margin{};                   // emergency stop margins
  AvoidMargin avoid_margin{};  // clearance saturation cap
  IgnoreBox ignore_box{};
  Limits    limits{};
  Weights   weights{};

  float sim_time    = 2.5f;   // [s] candidate arc rollout horizon
  /// The scoring attractor is the path point this far along the local path
  /// (advanced past path points blocked by obstacles). Scoring against a
  /// LOCAL GOAL instead of the nearest path segment keeps the robot from
  /// being pulled into an obstacle that sits on the path itself.
  float score_lookahead = 2.5f;  // [m]
  /// Station goal (default): score candidates by arc-length PROGRESS along
  /// the local path (projection station) and heading vs the path TANGENT at
  /// the projection, instead of the fixed-lookahead goal point. Progress and
  /// tangent are both first-order immune to a laterally drifted path; the
  /// lateral offset to the path enters with the (deliberately weak) weight
  /// below so that clearance/balance keep the lateral authority, and exerts
  /// NO pull on path segments blocked by an obstacle (the swerve must not
  /// fight the path attraction). At the path end the remaining Euclidean
  /// distance takes over. false restores the fixed-lookahead point goal.
  bool  station_goal = true;
  /// Lateral-offset weight of the station cost, as a fraction of
  /// weights.goal_dist per meter of offset.
  float station_lateral_weight = 0.3f;
  /// Line-of-sight goal: while the straight segment from the robot to the
  /// local goal passes within this radius of an obstacle point, the goal is
  /// pulled back to the farthest VISIBLE path point. Around a corner the
  /// attractor then sits at the corner instead of beyond it, so the robot
  /// stops hugging the inner wall. 0 disables.
  float goal_los_radius = 0.45f;  // [m]
  /// Density adaptation: the tightness reference tracks an EMA (this rate
  /// per tick) of the probed achievable clearance, clamped to
  /// [body/2 + safety.side, the configured cap]. Where the configured avoid
  /// margin is unattainable everywhere (dense clutter) the "tight" judgment
  /// relaxes towards what the environment affords instead of treating the
  /// whole field as a crisis. 0 disables (fixed reference).
  float cap_adapt_rate = 0.05f;

  /// Obstacle points within this distance of the path are treated as
  /// on-path blockers (degraded plan - the swerve logic's business) and do
  /// not trim the line of sight. Smaller than half the body width so that
  /// walls a real planner path legitimately skirts still count as walls.
  float los_onpath_radius = 0.5f;  // [m]
  /// Clearance/admissibility evaluation reaches at least this far along the
  /// arc regardless of the candidate speed (a purely time-based horizon turns
  /// myopic at low speed: obstacles drop out of range as the robot slows, and
  /// it creeps into them until the emergency stop).
  float min_eval_distance = 1.6f;  // [m]
  /// Forward candidates with a turn radius below this are excluded: near-spin
  /// arcs degenerate the clearance measure (every point is "far" from a
  /// tiny circle). Sharp turning is covered by the v=0 turn-then-go row.
  float turn_radius_min = 0.25f;  // [m]
  /// Curved candidates are never evaluated beyond this arc angle: the planner
  /// re-decides every tick, so extrapolating a turn much past ~60 degrees
  /// mis-scores recovery arcs as "hitting the far wall eventually".
  float eval_angle_max = 1.05f;  // [rad]
  /// ... and never beyond this LATERAL displacement of the arc: a small-radius
  /// correction extrapolated deep across the passage would self-punish via
  /// the clearance min (the planner re-decides long before that).
  float eval_lateral_max = 0.5f;  // [m]
  /// A body-hit beyond the clearance window still poisons the clearance when
  /// the hit point is euclidean-NEAR (a tight arc reaches a nearby obstacle
  /// at a long arc length; arc length alone would hide the threat). The
  /// poison fades from full at blocked_near to none at blocked_far.
  float blocked_near = 0.4f;  // [m]
  float blocked_far  = 1.2f;  // [m]
  /// The emergency margins scale with speed (braking distance shrinks with
  /// speed; full margins at margin_scale_speed and above, margin_scale_floor
  /// of them at standstill). Without this the robot Zeno-chatters on the
  /// boundary: stop -> tiny margin violation -> stop, never rebuilding speed.
  float margin_scale_floor = 0.6f;
  float margin_scale_speed = 0.3f;  // [m/s]
  float window_time = 0.25f;  // [s] accel authority defining the dynamic window
  int   v_samples   = 5;      // forward-speed samples in the window (v=0 row is added)
  /// w candidates are sampled UNIFORMLY over [-w_max, w_max] (not an
  /// accel-window around the current w): corrective arcs must always be on
  /// the table, independent of the current state and the planner's target.
  int   w_samples   = 25;
  /// Coarse-to-fine steering: this many finer w offsets are re-sampled on
  /// each side of the coarse winner (at its v). Removes the steering
  /// quantization of the uniform grid, whose pitch otherwise becomes the
  /// smallest applicable correction (tick-scale zigzag in narrow passages).
  /// 0 disables.
  int w_refine_steps = 3;

  float stop_decel          = 1.0f;   // [m/s^2] braking capability
  float brake_reaction_time = 0.1f;   // [s] latency covered by braking distances
  float max_range           = 10.0f;  // [m] points beyond this are ignored
  /// Defensive cap on the point count (uniform stride subsampling above it):
  /// process() is linear in points x candidates; a dense sensor should be
  /// decimated upstream, this cap bounds the worst case.
  int max_points = 1000;

  float velocity_min = 0.005f;  // [m/s] outputs below are clamped to 0
  float angvel_min   = 0.01f;   // [rad/s] outputs below are clamped to 0

  /// Proximity speed governor: the sampled speed window is capped in front of
  /// points the CURRENT motion would actually run into (straight-line slab
  /// test: a point whose miss distance overlaps the body brakes on a linear
  /// ramp over side_envelope_lookahead; a wall parallel to the travel
  /// direction misses and never caps the cruise speed - safety beside the
  /// body rests on the admissibility test and the emergency layer). The
  /// creep floor keeps an escape possible right at the boundary instead of
  /// freezing.
  float creep_fraction = 0.3f;
  /// Arc refinement of the governor, RELEASE-ONLY: a point the straight
  /// prediction calls a collision course is released when the CURRENT (v, w)
  /// arc carries it past the body (mid-turn, the nose points at the corner's
  /// outer wall until the heading has swung, which would otherwise pin the
  /// corner speed at creep). Braking is never ADDED by the arc, so straight
  /// narrow-corridor behavior is untouched. false restores the pure straight
  /// slab.
  bool governor_arc_prediction = true;
  /// Cruise moderation in bilateral tightness: the sampled speed is scaled by
  /// (1 - (1 - tight_cruise_fraction) * tightness), i.e. this fraction of
  /// v_max remains available in a fully tight passage. Steering authority
  /// (lateral correction per meter, ~w/v) shrinks with speed, so precise
  /// centering between close walls needs a moderated cruise; 1.0 disables
  /// (full speed everywhere, coarser centering).
  float tight_cruise_fraction = 0.5f;
  /// Side envelope of the governor: while a point sits beside the body (or is
  /// about to be passed) inside the unscaled side margin plus this cushion
  /// (in margin units), the sampled speed stays below the speed whose own
  /// speed-scaled margin would reach within cushion of that point. Prevents
  /// the pass-accelerate-stop chatter next to obstacles by construction while
  /// leaving gaps the full margin already clears at full cruise speed.
  float side_envelope_headroom = 0.1f;
  /// How far ahead of the stop zone a predicted-close-pass point engages the
  /// side envelope [m]. Covers the swerve-carving phase around an obstacle
  /// (the cap must not release mid-carve); has no effect on gaps wider than
  /// the side margin plus cushion, so corridors stay at full cruise speed.
  float side_envelope_lookahead = 1.0f;

  /// Status is CLEAR when no obstacle point is within this distance of the
  /// footprint rectangle (pass-through arbitration hint for the filter node).
  float influence_range = 1.2f;  // [m]

  /// Ticks the AVOIDING status stays latched after the influence clears
  /// (prevents tick-level arbitration chattering downstream).
  int avoiding_latch_ticks = 30;
};

/**
 * @brief Result of one process() call, with evaluation extras for logging.
 */
struct Result
{
  Twist2D output{};
  Status  status = Status::CLEAR;

  // Evaluation / debug extras (for the selected candidate)
  float best_clearance     = 0.0f;  // bilateral clearance of the chosen arc [m]
  float best_goal_dist     = 0.0f;  // endpoint distance to the local goal [m]
  float goal_x             = 0.0f;  // local goal used for scoring (robot frame)
  float goal_y             = 0.0f;
  float min_proximity_norm = 0.0f;  // nearest point, margin-normalized (1.0 = at boundary)
  float nearest_distance   = 0.0f;  // nearest point to the footprint rectangle [m]
  int   admissible_count   = 0;     // candidates that passed the stopping test
  int   candidate_count    = 0;
};

/**
 * @brief Bilateral clearance of one candidate arc (v, w).
 *
 * clearance_left/right: smallest lateral offset from the arc centerline per
 * side of the travel direction (free half-width of the passage along the arc).
 * blocking_s: distance along the arc to the first point the BODY would hit
 * (|offset| < width/2); FLT_MAX if the path is body-free.
 * lateral_fraction in [0,1]: tightest squeeze among non-hitting points
 * (0 = grazing the body, 1 = everything outside the safety side margin).
 */
struct ArcEvaluation
{
  float clearance_left;
  float clearance_right;
  /// Same mins restricted to the FORWARD part of the arc (s >= body front):
  /// the near-field min is anchored to the current pose and carries no
  /// gradient for departing arcs; balance/centering must look ahead.
  float far_left;
  float far_right;
  float blocking_s;
  float lateral_fraction;
};

/**
 * @brief Arc-clearance local planner: follows a local path, avoids obstacles
 *        when there is room, and funnels smoothly through the middle of
 *        narrow openings when there is not.
 *
 * Stateful across ticks (hysteresis, status latch); call process() at a fixed
 * rate (nominally 20Hz) and reset() on discontinuities (teleport, long pause).
 */
class BacCore
{
public:
  BacCore();
  explicit BacCore(const Params &params);

  void          setParams(const Params &params);
  const Params &params() const;

  /**
   * @brief Process one control tick.
   * @param points  Obstacle points in the ROBOT frame [m]
   * @param path    Local path in the ROBOT frame, ordered near-to-far. An
   *                empty path means "no intent": the output is (0, 0).
   * @param current Current (measured) robot velocity
   */
  Result process(const std::vector<Point2D> &points, const std::vector<Point2D> &path,
                 const Twist2D &current);

  /// Evaluate a single candidate arc (exposed for tests and tuning tools)
  ArcEvaluation evaluateArc(const std::vector<Point2D> &points, float v, float w, float horizon) const;

  /// Arc evaluation with separate windows: clearance/squeeze aggregate within
  /// dist_clear along the arc, blocking is searched within dist_block.
  ArcEvaluation evaluateArcWindows(const std::vector<Point2D> &points, float v, float w,
                                   float dist_clear, float dist_block) const;

  Status status() const;
  /// Force the STOP status (e.g. sensor data lost); cleared by the next process()
  void forceStop();
  /// Clear all cross-tick state (latch, hysteresis reference)
  void reset();

private:
  Params params_;

  Status current_status_;
  int    avoiding_counter_;  // AVOIDING latch countdown
  float  prev_selected_w_;   // previously selected steering rate (hysteresis)
  float  cap_ema_;           // density-adapted clearance reference (-1 = uninitialized)
};

}  // namespace bac

#endif  // BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CORE_HPP_
