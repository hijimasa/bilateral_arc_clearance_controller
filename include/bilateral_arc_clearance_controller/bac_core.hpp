/**
 * @file bac_core.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Framework-free core of the arc-clearance local planner
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Self-contained C++17 library: no ROS, no shm, no project-specific headers.
 * Inputs are obstacle points and a LOCAL PATH, both in the robot frame, plus
 * the current velocity; output is the next (v, w) command.
 *
 * The differential-drive policy derives from Dynamic Window Approach candidate
 * sampling; the Ackermann policy samples body curvature bounded by
 * turn_radius_min, at the granularity of the Nav2 MPPI AckermannConstraints.
 * In both cases, traversability evaluation uses the bilateral arc clearance
 * measure: candidate velocities are sampled from an acceleration-limited
 * translational window and the configured steering range, rolled out as
 * constant-curvature arcs, and scored by
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

#include <cmath>
#include <memory>
#include <vector>

namespace bac
{

namespace detail
{
class MotionModel;
}

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
  float v = 0.0f;   // forward velocity, body +x [m/s]
  float w = 0.0f;   // angular velocity [rad/s], CCW positive
  float vy = 0.0f;  // lateral velocity, body +y [m/s]; non-zero only for
                    // holonomic models. Every non-holonomic model produces
                    // and consumes vy == 0, so the field is invisible to
                    // differential-drive and Ackermann users.

  Twist2D() = default;
  Twist2D(float v_val, float w_val)
    : v(v_val)
    , w(w_val)
  {
  }
  Twist2D(float v_val, float w_val, float vy_val)
    : v(v_val)
    , w(w_val)
    , vy(vy_val)
  {
  }

  /// Speed along the direction of travel. Equals |v| exactly when vy == 0.
  float speed() const { return (vy == 0.0f) ? std::fabs(v) : std::hypot(v, vy); }
};

enum class MotionModelType : int
{
  DIFF_DRIVE = 0,
  ACKERMANN  = 1,
  OMNI       = 2
};

/// Vehicle-kinematic policy. Every model consumes and produces body Twist.
struct MotionModelParameters
{
  MotionModelType type = MotionModelType::DIFF_DRIVE;
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
  /// Body yaw acceleration the plant can actually deliver. Candidate steering
  /// remains fully sampled so corrective arcs stay available, but the output
  /// is limited from the measured yaw rate and re-checked. This is not an
  /// Ackermann road-wheel steering-rate guarantee. 0 disables.
  float acc_w = 2.5f;  // [rad/s^2]
  /// Lateral speed authority, holonomic models only [m/s]. 0 for every
  /// non-holonomic model, and a holonomic configuration with 0 here is
  /// rejected: it would select a model that cannot use its own avoidance
  /// dimension. Sideways motion needs sensor coverage abeam the body, the
  /// same caveat `v_min` carries for reverse.
  float vy_max = 0.0f;  // [m/s]
};

/// Scoring weights. Balance rationale (see the harness scenarios):
///  - clearance is the traversability measure (saturates at width/2 +
///    avoid_margin.side): in the open many candidates saturate and the path
///    terms decide; in tight spaces its max-min structure funnels through the
///    middle of the opening.
///  - path_dist weighs the path-following cost (remaining projection
///    arc-length plus the weak lateral offset): it rewards progress along
///    the path and supplies the speed selection.
///  - hysteresis damps tick-to-tick steering chatter; keep it well below the
///    path terms or the previous choice can pin the robot off-path.
struct Weights
{
  float clearance  = 2.0f;  // [score per m] saturated bilateral clearance
  float path_dist  = 1.0f;  // [score per m] path cost: remaining station + weighted lateral offset
  /// Bilateral balance: penalty on |left - right| clearance (both capped),
  /// active only in tight spaces (scaled by the probed tightness). Provides a
  /// FIRST-ORDER centering gradient from the geometry itself - the local-goal
  /// distance is only second-order in the lateral error, and the goal (a
  /// grid-quantized, replanned global path) can itself sit off-center.
  float balance    = 4.0f;  // [score per m]
  float heading    = 0.15f; // [score per rad] endpoint heading vs local-goal bearing
  /// Change in yaw rate [score per rad/s] for differential drive, or in body
  /// curvature [score per 1/m] for Ackermann. The units differ, so this value
  /// does NOT transfer between models: it is tuned for differential drive, and
  /// the curvature term does not shrink with speed the way the yaw-rate one
  /// does. Ackermann needs a lower value; see config/bac_controller_ackermann.yaml.
  float hysteresis = 0.6f;
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
  MotionModelParameters motion_model{};

  float sim_time    = 2.5f;   // [s] candidate arc rollout horizon
  /// Path following scores candidates by arc-length PROGRESS along the local
  /// path (projection station) and heading vs the path TANGENT at the
  /// projection - both first-order immune to a laterally drifted path. The
  /// lateral offset to the path enters with this (deliberately weak) weight,
  /// as a fraction of weights.path_dist per meter, so that clearance/balance
  /// keep the lateral authority; it exerts NO pull on path segments blocked
  /// by an obstacle, and the full Euclidean distance takes over outside the
  /// path's longitudinal span.
  float station_lateral_weight = 0.3f;
  /// Density adaptation: the tightness reference tracks an EMA (this rate
  /// per tick) of the probed achievable clearance, clamped to
  /// [body/2 + safety.side, the configured cap]. Where the configured avoid
  /// margin is unattainable everywhere (dense clutter) the "tight" judgment
  /// relaxes towards what the environment affords instead of treating the
  /// whole field as a crisis. 0 disables (fixed reference).
  float cap_adapt_rate = 0.05f;

  /// Clearance/admissibility evaluation reaches at least this far along the
  /// arc regardless of the candidate speed (a purely time-based horizon turns
  /// myopic at low speed: obstacles drop out of range as the robot slows, and
  /// it creeps into them until the emergency stop).
  float min_eval_distance = 1.6f;  // [m]
  /// Differential drive: translating candidates (forward or reverse) with a
  /// turn radius below this are excluded, because near-spin arcs degenerate
  /// the clearance measure (every point is "far" from a tiny circle). Sharp
  /// turning is covered by the v=0 turn-then-go row.
  /// Ackermann: the kinematic minimum turning radius, which bounds candidate
  /// curvature itself. Must be positive.
  float turn_radius_min = 0.25f;  // [m]
  /// Holonomic models only: proportional gain from body heading error to the
  /// commanded yaw rate [1/s]. A holonomic model does not search over yaw -
  /// lateral velocity does the avoiding - so the yaw rate regulates the body
  /// onto the local path tangent while (v, vy) is searched. Must be finite
  /// and non-negative; 0 holds the heading fixed.
  float heading_gain = 1.5f;  // [1/s]
  /// Holonomic models only: lateral-velocity samples per forward-speed row,
  /// the counterpart of `w_samples`. Must be at least 3.
  int vy_samples = 15;
  /// Curved candidates are never evaluated beyond this arc angle: the planner
  /// re-decides every tick, so extrapolating a turn much past ~60 degrees
  /// mis-scores recovery arcs as "hitting the far wall eventually".
  /// (Method constant - not exposed as a ROS parameter.)
  float eval_angle_max = 1.05f;  // [rad]
  /// ... and never beyond this LATERAL displacement of the arc: a small-radius
  /// correction extrapolated deep across the passage would self-punish via
  /// the clearance min (the planner re-decides long before that).
  float eval_lateral_max = 0.5f;  // [m]
  /// A body-hit beyond the clearance window still poisons the clearance when
  /// the hit point is euclidean-NEAR (a tight arc reaches a nearby obstacle
  /// at a long arc length; arc length alone would hide the threat). The
  /// poison fades from full at blocked_near to none at blocked_far.
  /// (Method constants - not exposed as ROS parameters.)
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
  /// Differential-drive yaw rates or Ackermann curvatures are sampled
  /// uniformly over their configured range (not an acceleration window):
  /// corrective arcs must always be on the table.
  int   w_samples   = 25;
  /// Coarse-to-fine steering: this many finer yaw-rate or curvature offsets
  /// are re-sampled on each side of the coarse winner (at its v). Removes the steering
  /// quantization of the uniform grid, whose pitch otherwise becomes the
  /// smallest applicable correction (tick-scale zigzag in narrow passages).
  /// 0 disables.
  int w_refine_steps = 3;

  /// Braking capability assumed by the admissibility test. MUST be set at or
  /// below the real plant's braking limit (assuming a stronger brake than the
  /// robot has voids the stop-before-hit guarantee).
  float stop_decel          = 0.8f;   // [m/s^2]
  float brake_reaction_time = 0.1f;   // [s] latency covered by braking distances
  float max_range           = 10.0f;  // [m] points beyond this are ignored
  /// Defensive cap on the point count (uniform stride subsampling above it):
  /// process() is linear in points x candidates; a dense sensor should be
  /// decimated upstream, this cap bounds the worst case.
  int max_points = 1000;

  /// Control period assumed for the body yaw-acceleration output limit [s].
  float control_period = 0.05f;

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
  /// (Method constant - not exposed as a ROS parameter.)
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
  float best_path_cost     = 0.0f;  // path cost of the chosen candidate [m]
  float goal_x             = 0.0f;  // preview point on the path (diagnostics only)
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
 * blocking_s: BODY-ORIGIN travel distance along the arc at the FIRST physical
 * contact with any point (exact swept-rectangle contact on curved arcs,
 * capped at eval_angle_max of body rotation); FLT_MAX if contact-free.
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
  ~BacCore();

  /// Value semantics are preserved; each instance owns a motion model bound
  /// to its own params_. The model cannot be stolen from another instance,
  /// so moves deliberately fall back to these copies.
  BacCore(const BacCore &other);
  BacCore &operator=(const BacCore &other);

  /// Rebuilds the motion model, so an invalid kinematic configuration is
  /// rejected here rather than inside a control tick.
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

  /// Same, for a full body twist. Holonomic candidates carry a lateral
  /// component that the scalar overload cannot express.
  ArcEvaluation evaluateArcWindows(const std::vector<Point2D> &points, const Twist2D &command,
                                   float dist_clear, float dist_block) const;

  /// Limit a command to what the plant can reach within one control period
  /// under the configured motion model. Exposed for adapters that pass a
  /// command through unchanged: the reachability contract still applies to
  /// the passed-through command.
  Twist2D limitReachableCommand(const Twist2D &current, const Twist2D &desired) const;

  Status status() const;
  /// Force the STOP status (e.g. sensor data lost); cleared by the next process()
  void forceStop();
  /// Clear all cross-tick state (latch, hysteresis reference)
  void reset();

private:
  /// Bound to this instance's params_. Built once per configuration so that
  /// process() neither allocates nor throws on a bad configuration.
  void rebuildMotionModel();

  Params params_;
  std::unique_ptr<detail::MotionModel> motion_model_;

  Status current_status_;
  int    avoiding_counter_;  // AVOIDING latch countdown
  Twist2D prev_selected_command_;  // previous model command (steering hysteresis)
  float  cap_ema_;           // density-adapted clearance reference (-1 = uninitialized)
  bool   alignment_mode_;    // reduce a large local-path tangent error before translating
};

}  // namespace bac

#endif  // BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CORE_HPP_
