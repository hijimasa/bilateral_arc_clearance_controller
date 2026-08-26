/**
 * @file bac_core.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief Framework-free core of the arc-clearance obstacle avoidance algorithm
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 REACT Co., Ltd.
 *
 * Self-contained C++17 library: no ROS, no shm, no project-specific headers.
 * Inputs are obstacle points in the robot frame plus an upper-level velocity
 * command (v, w); output is a collision-shaped velocity command.
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
  CLEAR    = 0,  // output == command, avoidance transparent
  AVOIDING = 1,  // output overrides the command
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

/// Axis-aligned box of points to ignore (sensor returns from the robot itself)
struct IgnoreBox
{
  float front = 0.0f;  // ignore x < front ...
  float back  = 0.0f;  // ... and x > -back ...
  float width = 0.0f;  // ... and |y| < width/2
};

/// Scoring weights. Invariants that must hold (see the harness scenarios):
///  - hysteresis < fidelity, or the previous choice permanently beats the
///    return to the command once every candidate saturates (open space) and
///    the avoidance never releases;
///  - fidelity small enough that a few cm of bilateral clearance outweigh the
///    full w_range (clearance * cap_difference > fidelity * w_range).
struct Weights
{
  /// weight on the saturated bilateral clearance [score per m]
  float clearance = 1.0f;
  /// penalty per rad/s of deviation from the commanded w [m per rad/s]
  float fidelity = 0.05f;
  /// penalty per rad/s of deviation from the previously selected w
  float hysteresis = 0.03f;
  /// lower bound of the fidelity viability scale: a command whose own arc is
  /// blocked keeps this fraction of its pull (0 would orphan the intent)
  float fidelity_viability_floor = 0.1f;
};

struct Params
{
  Footprint footprint{};
  Margins   safety_margin{};                   // emergency stop / speed governor margins
  Margins   avoid_margin{ 0.3f, 0.3f, 0.3f };  // clearance saturation cap (how far avoidance aims)
  IgnoreBox ignore_box{};
  Weights   weights{};

  float reaction_time          = 3.0f;   // [s] planning horizon base
  float estimation_time_margin = 1.0f;   // [s] added to reaction_time for the horizon
  float stop_decel             = 1.0f;   // [m/s^2] braking capability (emergency zone extent)
  float brake_reaction_time    = 0.1f;   // [s] latency covered by the emergency zone
  float w_range                = 0.75f;  // [rad/s] candidate half-window
  int   num_candidates         = 20;     // even; num_candidates+1 arcs are evaluated
  float max_range              = 10.0f;  // [m] points beyond this are ignored

  float velocity_min = 0.005f;  // [m/s] outputs below are clamped to 0
  float angvel_min   = 0.01f;   // [rad/s] outputs below are clamped to 0

  /// While avoiding and turning, keep at least this forward speed so the turn
  /// makes progress (legacy creep behavior; also the speed cap of that creep).
  float creep_speed = 0.1f;
  /// Proximity governor: speed fraction floor right at the emergency boundary,
  /// keeping an escape possible instead of freezing there.
  float creep_fraction = 0.15f;
  /// Proximity governor engages below this margin-normalized distance
  /// (1.0 = at the emergency boundary; 2.0 = one extra margin away).
  float proximity_governor_range = 2.0f;

  /// Ticks the AVOIDING status stays latched after the output converged back
  /// to the command (prevents tick-level arbitration chattering downstream).
  int avoiding_latch_ticks = 30;
};

/**
 * @brief Result of one process() call, with evaluation extras for logging.
 */
struct Result
{
  Twist2D output{};
  Status  status = Status::CLEAR;

  // Evaluation / debug extras
  float selected_w         = 0.0f;  // steering rate chosen by the scorer (before clamps)
  float speed_fraction     = 1.0f;  // longitudinal speed scale applied to the command
  float command_clearance  = 0.0f;  // bilateral clearance of the commanded arc itself
  float fidelity_scale     = 1.0f;  // viability scale applied to the fidelity weight
  float min_proximity_norm = 0.0f;  // nearest point, margin-normalized (1.0 = at boundary)
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
  float blocking_s;
  float lateral_fraction;
};

/**
 * @brief Arc-clearance avoidance: shapes an upper-level velocity command so
 *        the robot avoids obstacles when there is room, and funnels smoothly
 *        through the middle of narrow openings when there is not.
 *
 * Stateful across ticks (candidate-window commitment, status latch); call
 * process() at a fixed rate (nominally 20Hz) and reset() on discontinuities
 * (teleport, long pause).
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
   * @param command Upper-level velocity command (the INTENT; do not pre-mix
   *                it with the current velocity, see the harness notes)
   * @param current Current (measured) robot velocity
   */
  Result process(const std::vector<Point2D> &points, const Twist2D &command, const Twist2D &current);

  /// Evaluate a single candidate arc (exposed for tests and tuning tools)
  ArcEvaluation evaluateArc(const std::vector<Point2D> &points, float v, float w, float horizon) const;

  Status status() const;
  /// Force the STOP status (e.g. sensor data lost); cleared by the next process()
  void forceStop();
  /// Clear all cross-tick state (latch, committed window)
  void reset();

private:
  Params params_;

  Status current_status_;
  int    command_sign_;      // sign of the last nonzero commanded forward velocity
  int    avoiding_counter_;  // AVOIDING latch countdown
  float  prev_selected_w_;   // previously selected steering rate (window commitment)
};

}  // namespace bac

#endif  // BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CORE_HPP_
