/**
 * @file omni_scenarios.cpp
 * @brief Closed-loop regressions for the holonomic BAC policy
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * The holonomic contract is that AVOIDANCE happens in lateral velocity while
 * the yaw rate regulates the body onto the local path tangent. The scenarios
 * that carry the weight here are therefore the ones that can tell the two
 * apart: they run the same world with a differential-drive reference that
 * shares the tuning, and assert on the DIFFERENCE. An assertion that a
 * holonomic robot reaches its goal would pass for either model.
 *
 * The plant is acceleration-limited on all three axes and integrated in the
 * world frame, so the commands are checked against a vehicle that cannot
 * change any velocity component instantly.
 */

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "shipped_config.hpp"
#include "test_expect.hpp"
#include "sim_runner.hpp"
#include "sim_world.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace
{

using bac_test::expect;
using bac_test::failures;
using bac_sim::LateralWindow;

float
wrapAngle(float angle)
{
  while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
  while (angle <= -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
  return angle;
}

/// Arc length at which the footprint rectangle first touches one of `points`,
/// derived here from the rectangle and the rigid-body motion alone.
///
/// R18 H1: the sweeps below assert that the emitted twist can stop before
/// contact, but they took that contact distance from `evaluateArcWindows` -
/// the code under test. A defect that shrinks the swept region shrinks the
/// requirement by exactly as much, so the assertion cancels out: seeding
/// `body_hit = abs_offset < side_extent * 0.5f` into the evaluator leaves both
/// sweeps printing "0 violations" while an independent probe finds 38. What
/// follows shares no code with the evaluator and is derived differently.
///
/// In the BODY frame a static obstacle point satisfies
/// `pdot = (-v + w * py, -vy - w * px)`, so it circles the instantaneous
/// centre `c = (-vy / w, v / w)` at angular rate `-w`: a point at radius `r`
/// and initial bearing `phi0` is at `phi0 - sign(w) * u` after travelling
/// `u` radians, i.e. after `s = speed * u / |w|`. Contact is therefore the
/// smallest `u >= 0` whose bearing puts the point inside the rectangle.
///
/// The circle crosses each of the four edge lines at most twice, so at most
/// eight bearings can change the inside/outside answer. Between consecutive
/// crossings the answer is constant, which makes the midpoint of each gap a
/// sufficient test and the whole thing exact rather than sampled.
float
independentContactDistance(const std::vector<bac::Point2D> &points,
                           const bac::Footprint &body, const bac::Twist2D &command)
{
  const float half_width = body.width * 0.5f;
  const float speed = command.speed();
  if (speed <= 1e-6f)
  {
    return std::numeric_limits<float>::max();
  }
  const auto inside = [&](float x, float y) {
    return x >= body.rear && x <= body.front && y >= -half_width && y <= half_width;
  };

  // Straight motion has no centre to circle about; the rectangle simply
  // translates, and the point is inside for an interval in s on each axis.
  //
  // R19 M1: this threshold is the exact complement of the evaluator's
  // `turning = std::fabs(command.w) > 1e-4f`, so the two agree on which branch
  // a command belongs to. It was 1e-6, two decades lower, which left a band
  // `1e-6 <= |w| <= 1e-4` where this function took the ARC branch while the
  // evaluator took the STRAIGHT one. In that band the centre `(-vy/w, v/w)` is
  // 1e4-1e6 m out and `per_radian = speed / |w|` amplifies the rounding of
  // `phi0` into metres of distance error.
  //
  // Measured against a third derivation - integrate the body pose in the world
  // frame and transform the static point back into the body frame, which uses
  // no centre of rotation at all - over 4000 draws per band inside a 3 m
  // window. At the old 1e-6 threshold, band 1e-6..1e-5: 98 of 3971 report
  // contact LATER than it happens (worst 3.00 m, i.e. no contact reported at
  // all where one exists) and 70 report it earlier; band 1e-5..1e-4: 4 and 2 of
  // 3957. At 1e-4 every band measured - 1e-6..1e-5, 1e-5..1e-4, 1e-4..1e-3,
  // 1e-2..1e-1, 1e-1..1e0 - is 0 and 0.
  //
  // No caller reaches the band either way, so this closes a trap rather than a
  // defect: of the 75338 calls this suite makes, 39206 have `w` exactly zero,
  // 0 have `0 < |w| < 1e-4`, 12 are in [1e-4, 1e-2) and 36120 at or above
  // 1e-2.
  if (std::fabs(command.w) <= 1e-4f)
  {
    const float ux = command.v / speed;
    const float uy = command.vy / speed;
    const auto axis = [](float coordinate, float lo, float hi, float direction, float &s_lo,
                         float &s_hi) {
      if (std::fabs(direction) < 1e-9f)
      {
        const bool within = coordinate >= lo && coordinate <= hi;
        s_lo = within ? 0.0f : 1.0f;
        s_hi = within ? std::numeric_limits<float>::max() : -1.0f;
        return;
      }
      const float a = (coordinate - hi) / direction;
      const float b = (coordinate - lo) / direction;
      s_lo = std::min(a, b);
      s_hi = std::max(a, b);
    };
    float best = std::numeric_limits<float>::max();
    for (const bac::Point2D &p : points)
    {
      float x_lo = 0.0f, x_hi = 0.0f, y_lo = 0.0f, y_hi = 0.0f;
      axis(p.x, body.rear, body.front, ux, x_lo, x_hi);
      axis(p.y, -half_width, half_width, uy, y_lo, y_hi);
      const float lo = std::max(std::max(x_lo, y_lo), 0.0f);
      if (lo <= std::min(x_hi, y_hi) && lo < best)
      {
        best = lo;
      }
    }
    return best;
  }

  const float two_pi = 2.0f * static_cast<float>(M_PI);
  const float turn_sign = command.w >= 0.0f ? 1.0f : -1.0f;
  const float centre_x = -command.vy / command.w;
  const float centre_y = command.v / command.w;
  const float per_radian = speed / std::fabs(command.w);

  float best = std::numeric_limits<float>::max();
  for (const bac::Point2D &p : points)
  {
    const float qx = p.x - centre_x;
    const float qy = p.y - centre_y;
    const float radius = std::hypot(qx, qy);
    if (radius < 1e-9f)
    {
      // The point sits on the centre and never moves relative to the body.
      if (inside(centre_x, centre_y))
      {
        return 0.0f;
      }
      continue;
    }
    const float phi0 = std::atan2(qy, qx);

    float bearings[9];
    int count = 0;
    const auto push_cosine = [&](float target) {
      const float value = (target - centre_x) / radius;
      if (value >= -1.0f && value <= 1.0f)
      {
        const float a = std::acos(value);
        bearings[count++] = a;
        bearings[count++] = -a;
      }
    };
    const auto push_sine = [&](float target) {
      const float value = (target - centre_y) / radius;
      if (value >= -1.0f && value <= 1.0f)
      {
        const float a = std::asin(value);
        bearings[count++] = a;
        bearings[count++] = static_cast<float>(M_PI) - a;
      }
    };
    push_cosine(body.rear);
    push_cosine(body.front);
    push_sine(-half_width);
    push_sine(half_width);
    if (count == 0)
    {
      // The circle clears every edge line, so it is entirely inside or
      // entirely outside; one probe settles it.
      if (inside(centre_x + radius * std::cos(phi0), centre_y + radius * std::sin(phi0)))
      {
        return 0.0f;
      }
      continue;
    }

    // Re-express each crossing as the travel `u` that reaches it.
    float travel[9];
    for (int i = 0; i < count; ++i)
    {
      float u = turn_sign * (phi0 - bearings[i]);
      u = std::fmod(u, two_pi);
      if (u < 0.0f)
      {
        u += two_pi;
      }
      travel[i] = u;
    }
    travel[count++] = 0.0f;
    std::sort(travel, travel + count);

    for (int i = 0; i < count; ++i)
    {
      const float lo = travel[i];
      const float hi = (i + 1 < count) ? travel[i + 1] : two_pi;
      if (hi <= lo)
      {
        continue;
      }
      const float mid = 0.5f * (lo + hi);
      const float phi = phi0 - turn_sign * mid;
      if (inside(centre_x + radius * std::cos(phi), centre_y + radius * std::sin(phi)))
      {
        const float s = lo * per_radian;
        if (s < best)
        {
          best = s;
        }
        break;
      }
    }
  }
  return best;
}

/// How far along its own arc the controller is REQUIRED to have looked, for the
/// twist `command` at the evaluation horizon `horizon`.
///
/// R19 H1: the sweeps used to run the independent contact test only after the
/// evaluator had already reported a contact (`blocking_s < 1e9`) and a positive
/// free run. That gate made the independent test unable to see the one defect
/// class it exists for - the evaluator MISSING a contact - because a miss took
/// the tick out of the sample instead of failing it. Measured on the first
/// sweep, 8197 of 8314 moving ticks (98.6%) were removed by that gate.
///
/// Lifting the gate needs a stopping rule of its own, or the sweeps demand a
/// stop before contacts the controller never promised to look at. The rule is
/// the evaluator's own window, rebuilt here from `Params` and the twist:
///
///   * `s_max = horizon + lead`, where `lead` is the footprint's support
///     extent along the direction of travel - `supportExtent(body, ux, uy)` in
///     the evaluator, `(dx>=0 ? front*dx : rear*dx) + width/2 * |dy|` written
///     out. `evaluateArcWindows(points, out, horizon, horizon)` passes the same
///     `horizon` as both clearance and blocking distance, so
///     `s_max = max(blocking, clearance) + lead` is `horizon + lead`.
///   * for a turning command the evaluator additionally caps the search at
///     `s_angle_cap = turn_radius * eval_angle_max`, with
///     `turn_radius = speed / |w|`, and passes `min(s_max, s_angle_cap)` to
///     its contact solver.
///
/// Measured cost of omitting the cap: 7 false violations in the first sweep and
/// 53 in the second, with no mutation applied - all of them contacts beyond
/// `eval_angle_max` (1.05 rad) of arc, which the controller is not asked to
/// brake for.
float
independentSearchWindow(const bac::Params &params, const bac::Twist2D &command, float horizon)
{
  const float speed = command.speed();
  const float ux = (speed > 1e-6f) ? command.v / speed : ((command.v >= 0.0f) ? 1.0f : -1.0f);
  const float uy = (speed > 1e-6f) ? command.vy / speed : 0.0f;
  const float lead = ((ux >= 0.0f) ? params.footprint.front * ux : params.footprint.rear * ux) +
                     (params.footprint.width * 0.5f) * std::fabs(uy);
  float window = horizon + lead;
  if (std::fabs(command.w) > 1e-4f && speed > 1e-3f)
  {
    window = std::min(window, (speed / std::fabs(command.w)) * params.eval_angle_max);
  }
  return window;
}

struct OmniRun
{
  bac_sim::Pose final_pose;
  bool collided = false;
  bool reached_goal = false;
  bool exceeded_speed_cap = false;
  bool exceeded_lateral_cap = false;
  bool exceeded_yaw_cap = false;
  /// Smallest distance between the body RECTANGLE and any wall segment over
  /// the run, from bac_sim::robotClearance - the same quantity, from the same
  /// helper, that runClosedLoop and runAckermann report under this name.
  /// Stays at the 1e9 sentinel in a world with no walls.
  float min_clearance = 1e9f;
  float min_goal_distance = 1e9f;
  float travelled_distance = 0.0f;
  float max_speed = 0.0f;
  float max_abs_vy = 0.0f;
  float max_abs_w = 0.0f;
  float final_heading_error = 0.0f;  // vs the straight-line goal bearing
  int   stop_ticks = 0;
  int   total_ticks = 0;
  int   translating_ticks_first_second = 0;  // motion during the first 20 ticks
  float mean_abs_lateral = 0.0f;
  float max_abs_lateral = 0.0f;
  int   lateral_samples = 0;
};

/// Keys config/bac_controller_omni.yaml may carry without this scenario
/// consuming them. See shipped_config.hpp for what the guard enforces.
const std::vector<std::string> kAllowedUnconsumedKeys = {
    "controller_server",           // block header
    "ros__parameters",             // block header
    "FollowPath",                  // block header: the plugin block itself
    "controller_frequency",        // Nav2 control loop rate, not a BAC parameter
    "controller_plugins",          // Nav2 plugin list
    "plugin",                      // plugin class name (checked by the docker gate)
    "scan_topic",                  // BacController scan adapter, not BacCore
    "scan_timeout",                //   "
    "scan_downsample",             //   "
    "scan_min_points",             //   "
    "scan_inf_is_valid",           //   "
    "diagnostics_publish_period",  // diagnostics only
};

bool g_shipped_config_loaded = false;

bac::Params
shippedExampleParams()
{
  bac::Params params;
  bac_sim::ShippedConfigSpec spec;
  spec.path       = BAC_OMNI_CONFIG_PATH;
  spec.label      = "holonomic";
  spec.model_name = "omni";
  spec.allowed    = kAllowedUnconsumedKeys;
  spec.bindings   = {
      bac_sim::bindNumber("footprint.front", params.footprint.front),
      bac_sim::bindNumber("footprint.rear", params.footprint.rear),
      bac_sim::bindNumber("footprint.width", params.footprint.width),
      bac_sim::bindNumber("safety_margin.front", params.safety_margin.front),
      bac_sim::bindNumber("safety_margin.rear", params.safety_margin.rear),
      bac_sim::bindNumber("safety_margin.side", params.safety_margin.side),
      bac_sim::bindNumber("avoid_margin.side", params.avoid_margin.side),
      bac_sim::bindNumber("limits.v_max", params.limits.v_max),
      bac_sim::bindNumber("limits.v_min", params.limits.v_min),
      bac_sim::bindNumber("limits.vy_max", params.limits.vy_max),
      bac_sim::bindNumber("limits.w_max", params.limits.w_max),
      bac_sim::bindNumber("limits.acc_v", params.limits.acc_v),
      bac_sim::bindNumber("limits.acc_w", params.limits.acc_w),
      bac_sim::bindNumber("control_period", params.control_period),
      bac_sim::bindNumber("stop_decel", params.stop_decel),
      bac_sim::bindNumber("brake_reaction_time", params.brake_reaction_time),
      bac_sim::bindNumber("heading_gain", params.heading_gain),
      bac_sim::bindInteger("vy_samples", params.vy_samples),
      bac_sim::bindNumber("weights.clearance", params.weights.clearance),
      bac_sim::bindNumber("weights.path_dist", params.weights.path_dist),
      bac_sim::bindNumber("weights.balance", params.weights.balance),
      bac_sim::bindNumber("weights.heading", params.weights.heading),
      bac_sim::bindNumber("weights.hysteresis", params.weights.hysteresis),
      bac_sim::bindNumber("weights.squeeze", params.weights.squeeze),
      bac_sim::bindNumber("sim_time", params.sim_time),
  };

  const bac_sim::ShippedConfigVerdict verdict = bac_sim::checkShippedConfig(spec, expect);
  if (!verdict.readable)
  {
    return params;
  }
  params.motion_model.type = bac::MotionModelType::OMNI;
  g_shipped_config_loaded = verdict.complete;
  return params;
}

/// The parameters every holonomic scenario in this file runs - READ from the
/// shipped yaml, not written to match it.
///
/// Until this change the two were separate: this function hand-listed the
/// values and `shippedExampleParams()` parsed the file, which is the exact
/// state test/shipped_config.hpp opens by forbidding ("a hand-copied duplicate
/// would let a yaml edit ship a configuration that fails the very suite meant
/// to protect it"). It had already collapsed: the two builders produced
/// byte-identical `bac::Params` - measured, 0 of 212 bytes differ - so the two
/// runs `testShippedExampleConfiguration()` used to make were bit-identical to
/// runs the file already had (see that function). Deriving in this direction
/// is what makes the yaml load-bearing: an edit to the file now moves the
/// scenarios themselves. Counted at the call sites rather than asserted: of the
/// 17 scenario functions main() runs, 13 take their parameters from here
/// (directly or through `diffDriveReferenceParams()`), and 12 of those 13 drive
/// the 26 closed-loop runs - the thirteenth,
/// `testShippedExampleConfiguration()`, only asks for the verdict. The other
/// four are the three sweeps and the deadband regression, which draw randomised
/// parameters on purpose and are not testing the shipped configuration.
///
/// The divergence set is currently EMPTY. A holonomic scenario that needs a
/// value the shipped file does not carry overrides it here, one field at a
/// time and with the measurement that justifies it - the way
/// `diffDriveReferenceParams()` below does - never by re-listing the file.
///
/// Read once. The yaml guard's assertions are about the FILE, not about each
/// of the calls that ask for its values, and the first caller pays for them.
/// Measured on this file as it stands: `omniParams()` is called 33 times and
/// drives 26 closed-loop runs, so the guard would otherwise assert 33 times
/// over.
bac::Params
omniParams()
{
  static const bac::Params params = shippedExampleParams();
  return params;
}

/// The same vehicle under differential drive, used to show that the holonomic
/// assertions below test a real difference and not a property both models
/// happen to satisfy. Two settings differ, and both differ for a measured
/// reason rather than to make a test pass:
///
///  - weights.hysteresis: the term measures yaw-rate change here and lateral-
///    velocity change under the holonomic model, so the value does not carry
///    over (the same is true between differential drive and Ackermann).
///  - avoid_margin.side: with 0.9 the differential drive stops 3.9 m short of
///    the goal in the detour world below (measured at 0.5/0.6/0.7/0.9: final x
///    5.73, 5.72, 2.12, 2.11), because demanding that much side clearance
///    around an isolated obstacle keeps it from committing to either side. The
///    holonomic run is unaffected by the value (final x 5.72 at all four).
bac::Params
diffDriveReferenceParams()
{
  bac::Params params = omniParams();
  params.motion_model.type = bac::MotionModelType::DIFF_DRIVE;
  params.limits.vy_max = 0.0f;
  params.avoid_margin.side = 0.6f;
  params.weights.hysteresis = 0.6f;
  return params;
}

OmniRun
runOmni(bac::BacCore &core, const bac_sim::World &world, const bac_sim::Pose &start,
        const bac_sim::PathSource &path_source, float goal_x, float goal_y,
        float simulation_time, const LateralWindow &window = LateralWindow{},
        std::optional<float> goal_heading_world = std::nullopt)
{
  constexpr float dt = 0.05f;
  OmniRun run;
  bac_sim::Pose pose = start;
  float vx = 0.0f, vy = 0.0f, w = 0.0f;
  const bac::Params &params = core.params();
  const float acc_v = params.limits.acc_v;
  const float acc_w = params.limits.acc_w;
  double lateral_sum = 0.0;

  const int steps = static_cast<int>(simulation_time / dt);
  for (int step = 0; step < steps; ++step)
  {
    const float t = static_cast<float>(step) * dt;
    const std::vector<bac::Point2D> points =
        bac_sim::simulateLidar(world, pose, 720, params.max_range);
    const std::vector<bac::Point2D> path = path_source(pose, t);
    // Nav2 carries the goal orientation on the last plan pose; the adapter
    // hands it to the core in the CURRENT body frame, which is what this
    // reproduces.
    std::optional<float> goal_heading;
    if (goal_heading_world)
    {
      goal_heading = wrapAngle(*goal_heading_world - pose.th);
    }
    const bac::Result result =
        core.process(points, path, bac::Twist2D(vx, w, vy), goal_heading);
    const bac::Twist2D command = result.output;

    run.total_ticks++;
    const float commanded_speed = command.speed();
    if (!path.empty() && commanded_speed <= 1e-4f)
    {
      run.stop_ticks++;
    }
    if (step < 20 && commanded_speed > 1e-3f)
    {
      run.translating_ticks_first_second++;
    }
    run.max_speed = std::max(run.max_speed, commanded_speed);
    run.max_abs_vy = std::max(run.max_abs_vy, std::fabs(command.vy));
    run.max_abs_w = std::max(run.max_abs_w, std::fabs(command.w));
    if (commanded_speed > params.limits.v_max + 1e-3f) run.exceeded_speed_cap = true;
    if (std::fabs(command.vy) > params.limits.vy_max + 1e-3f) run.exceeded_lateral_cap = true;
    if (std::fabs(command.w) > params.limits.w_max + 1e-3f) run.exceeded_yaw_cap = true;

    // Acceleration-limited holonomic plant, integrated in the world frame.
    const float dv = acc_v * dt;
    const float dw = acc_w * dt;
    vx += std::max(-dv, std::min(dv, command.v - vx));
    vy += std::max(-dv, std::min(dv, command.vy - vy));
    w += std::max(-dw, std::min(dw, command.w - w));
    const float cs = std::cos(pose.th), sn = std::sin(pose.th);
    const float step_x = (vx * cs - vy * sn) * dt;
    const float step_y = (vx * sn + vy * cs) * dt;
    pose.x += step_x;
    pose.y += step_y;
    pose.th = wrapAngle(pose.th + w * dt);
    run.travelled_distance += std::sqrt(step_x * step_x + step_y * step_y);

    // Body contact, decided the way runClosedLoop (test/sim_runner.hpp) and
    // runAckermann decide it: the exact minimum distance between the four
    // body edges and every wall segment.
    //
    // This loop used to ask instead whether any LiDAR RETURN landed inside the
    // footprint rectangle, and reported the shortest return RANGE FROM THE
    // BODY ORIGIN under the name `min_clearance`. Two reasons for the change,
    // and NEITHER of them is "the exact test catches contacts the old one
    // missed" - see the blind spot below, which cuts the other way:
    //
    //  (a) One quantity, one name. All three closed-loop runners now report
    //      the same thing, from the same helper, under the same field.
    //  (b) `min_clearance` is now a clearance. Over the 26 runOmni calls this
    //      file makes, the 9 runs with walls reported 0.2501 to
    //      0.3500 m MORE than the true clearance; the 1.2 m corridor runs
    //      reported 0.5213-0.5449 m where the body was actually 0.2600-0.2886 m
    //      from a wall. testSafetyStopHoldsPosition prints the value: forced to
    //      fire, its message now reads 0.150000 m where it read 0.500000 m.
    //
    //      The two counts were 28 and 11 until the shipped-configuration
    //      scenario stopped driving two runs that duplicated runs this file
    //      already had. Both re-counted at a counter in this function: 26 and
    //      9. The three RANGES are unchanged and were re-measured, not
    //      inherited - the two runs that went away were bit-identical to runs
    //      that stayed, so they contributed no endpoint.
    //
    // Switching changed no verdict here: all 26 runs are contact-free under
    // BOTH tests, measured, so the pass/fail set and the stdout are unchanged.
    //
    // THE NEW TEST HAS ITS OWN BLIND SPOT, and it is the larger one. Under this
    // loop's tick sampling robotClearance misses contacts the old point test
    // caught. Measured with this runner's own numbers - footprint 0.70 x 0.50 m,
    // dt 0.05 s at limits.v_max 0.4 m/s, so 0.02 m of travel per tick, the
    // largest step this loop can take; 720 beams, max_range 10 m; a trial is one
    // of 400 sub-tick start phases x 3 lateral offsets (0.00 / 0.10 / 0.20 m),
    // the body marched straight past a fixed zero-thickness wall segment; ground
    // truth is the CONTINUOUS pass sampled at 1e-5 m, and a test "misses" when
    // it never fires at any tick of a trial that truly overlaps:
    //
    //   segment ALONG travel     old point test     robotClearance
    //      1 mm                       31.7%              95.0%
    //     10 mm                       16.7%              50.0%
    //     20 mm and longer             0.0%               0.0%
    //
    //   segment ACROSS travel    old point test     robotClearance
    //      1 mm .. 10 mm               0.0%             100.0%
    //    100 mm                        0.0%              66.7%
    //    300 mm                        0.0%              33.3%
    //    500 mm and longer             0.0%               0.0%
    //
    // The reason is geometric, not statistical. robotClearance is a distance
    // between the body's four EDGES and the wall, so a wall that fits strictly
    // inside the rectangle never touches an edge and reads POSITIVE: a segment
    // at the body origin returns +0.2500 m, the body's inscribed radius
    // min(front, width/2) = min(0.35, 0.25). A zero-thickness segment across
    // the travel direction only touches an edge over a measure-zero set of
    // positions, which a 0.02 m tick steps over - hence 100%.
    //
    // These 17 scenarios cannot reach it. Every obstacle in all three worlds
    // they build is a connected component at least 0.800 m across (measured:
    // the 0.8 x 0.8 m detour box, a 2.0 m wall, and 7.5 x 0.10 m corridor wall
    // boxes - the 0.10 m segments are the end caps of the 7.5 m boxes, not
    // isolated features), and 0.800 m exceeds the 0.700 m footprint length, so
    // nothing can hide inside the body.
    //
    // THIS IS A CONSTRAINT ON NEW SCENARIOS. An obstacle small enough to fit
    // inside the footprint - a post, a table leg, thin debris - or any bare
    // zero-thickness segment shorter than the body width, will be driven
    // through without this runner noticing. Model such things as closed boxes
    // (World::addBox) whose extent exceeds the footprint, as the existing
    // worlds do, or give this loop a swept-volume test instead of a per-tick
    // one.
    const float clearance = bac_sim::robotClearance(pose, params.footprint, world);
    run.min_clearance = std::min(run.min_clearance, clearance);
    if (clearance <= 0.0f)
    {
      run.collided = true;
      break;  // physical contact: the scenario has failed, as in the other two runners
    }

    const float goal_distance =
        std::sqrt((pose.x - goal_x) * (pose.x - goal_x) + (pose.y - goal_y) * (pose.y - goal_y));
    run.min_goal_distance = std::min(run.min_goal_distance, goal_distance);
    if (goal_distance < 0.35f)
    {
      run.reached_goal = true;
    }

    if (window.enabled && pose.x >= window.x_from && pose.x <= window.x_to)
    {
      const float lateral = std::fabs(pose.y - window.center_y);
      lateral_sum += lateral;
      run.lateral_samples++;
      run.max_abs_lateral = std::max(run.max_abs_lateral, lateral);
    }
  }

  run.final_pose = pose;
  if (run.lateral_samples > 0)
  {
    run.mean_abs_lateral =
        static_cast<float>(lateral_sum / static_cast<double>(run.lateral_samples));
  }
  run.final_heading_error =
      std::fabs(wrapAngle(std::atan2(goal_y - start.y, goal_x - start.x) - pose.th));
  return run;
}

/// Contracts every holonomic run must satisfy. Not thresholds: these are the
/// limits the configuration declares, so there is no band to measure.
void
expectLimitsRespected(const OmniRun &run, const std::string &where)
{
  expect(!run.exceeded_speed_cap, where + ": no commanded twist exceeds limits.v_max (max " +
                                      std::to_string(run.max_speed) + " m/s)");
  expect(!run.exceeded_lateral_cap, where + ": no commanded twist exceeds limits.vy_max (max " +
                                        std::to_string(run.max_abs_vy) + " m/s)");
  expect(!run.exceeded_yaw_cap, where + ": no commanded twist exceeds limits.w_max (max " +
                                    std::to_string(run.max_abs_w) + " rad/s)");
}

/// Baseline: an open field with a goal straight ahead.
void
testStraightGoalReached()
{
  bac_sim::World world;
  bac::BacCore core(omniParams());
  const OmniRun run = runOmni(core, world, { 0.0f, 0.0f, 0.0f },
                              bac_sim::gotoPointPath(6.0f, 0.0f), 6.0f, 0.0f, 60.0f);

  expect(!run.collided, "the open-field run has no body contact");
  expect(run.reached_goal, "the holonomic vehicle reaches a goal straight ahead (closest " +
                               std::to_string(run.min_goal_distance) + " m)");
  expectLimitsRespected(run, "open field");
}

/// THE discriminating scenario. A blocking obstacle is rounded by translating
/// sideways, not by turning: the same world under a differential-drive
/// reference has to yaw to get past it. Asserted as a comparison between the
/// two runs rather than against a hand-picked yaw threshold.
void
testSidestepsInsteadOfYawing()
{
  bac_sim::World world;
  world.addBox(3.0f, 0.0f, 0.8f, 0.8f);
  const bac_sim::PathSource path = bac_sim::gotoPointPath(6.0f, 0.0f);

  bac::BacCore reference(diffDriveReferenceParams());
  const OmniRun diff_run =
      runOmni(reference, world, { 0.0f, 0.0f, 0.0f }, path, 6.0f, 0.0f, 60.0f);
  bac::BacCore core(omniParams());
  const OmniRun omni_run = runOmni(core, world, { 0.0f, 0.0f, 0.0f }, path, 6.0f, 0.0f, 60.0f);

  expect(!omni_run.collided, "the holonomic detour has no body contact");
  expect(omni_run.reached_goal, "the holonomic vehicle rounds a blocking obstacle (closest " +
                                    std::to_string(omni_run.min_goal_distance) + " m)");
  expect(diff_run.reached_goal,
         "the differential-drive reference also gets past it, so the comparison below is "
         "between two solutions and not between success and failure");

  expect(omni_run.max_abs_vy > 0.05f,
         "the holonomic detour actually uses lateral velocity (max |vy| " +
             std::to_string(omni_run.max_abs_vy) + " m/s)");
  expect(diff_run.max_abs_vy <= 1e-6f,
         "the differential-drive reference commands no lateral velocity at all (max |vy| " +
             std::to_string(diff_run.max_abs_vy) + ")");
  // A comparison, not a threshold: the differential drive must steer around
  // the obstacle, the holonomic body does not have to.
  expect(omni_run.max_abs_w < diff_run.max_abs_w,
         "the holonomic detour yaws less than the differential-drive one (" +
             std::to_string(omni_run.max_abs_w) + " vs " + std::to_string(diff_run.max_abs_w) +
             " rad/s)");
  expectLimitsRespected(omni_run, "detour");
}

/// A goal behind the robot. The differential drive must rotate BEFORE it can
/// translate; the holonomic body translates from the first tick while it
/// turns. This is the scenario that shows usesRotateBeforeTranslate() is
/// doing something observable.
void
testRearGoalWithoutAligningFirst()
{
  bac_sim::World world;
  const bac_sim::PathSource path = bac_sim::gotoPointPath(-3.0f, 0.0f);

  bac::BacCore reference(diffDriveReferenceParams());
  const OmniRun diff_run =
      runOmni(reference, world, { 0.0f, 0.0f, 0.0f }, path, -3.0f, 0.0f, 90.0f);
  bac::BacCore core(omniParams());
  const OmniRun omni_run = runOmni(core, world, { 0.0f, 0.0f, 0.0f }, path, -3.0f, 0.0f, 90.0f);

  expect(diff_run.translating_ticks_first_second == 0,
         "the differential-drive reference rotates before it translates, so the assertion "
         "below is discriminating (" +
             std::to_string(diff_run.translating_ticks_first_second) + " moving ticks)");
  expect(omni_run.translating_ticks_first_second > 0,
         "the holonomic vehicle starts moving immediately instead of aligning first (" +
             std::to_string(omni_run.translating_ticks_first_second) + " of the first 20 ticks)");
  expect(omni_run.reached_goal,
         "the holonomic vehicle reaches a goal behind it with limits.v_min = 0 (closest " +
             std::to_string(omni_run.min_goal_distance) + " m)");
  expect(!omni_run.collided, "the rear-goal manoeuvre has no body contact");
  expectLimitsRespected(omni_run, "rear goal");
}

/// The yaw rate is a regulator, so it must actually regulate: an off-axis goal
/// has to leave the body pointing along the path it drove.
void
testHeadingRegulatorTracksTheTangent()
{
  bac_sim::World world;
  bac::BacCore core(omniParams());
  const OmniRun run = runOmni(core, world, { 0.0f, 0.0f, 0.0f },
                              bac_sim::gotoPointPath(4.0f, 3.0f), 4.0f, 3.0f, 90.0f);

  expect(run.reached_goal, "the holonomic vehicle reaches an off-axis goal (closest " +
                               std::to_string(run.min_goal_distance) + " m)");
  // Measured band over heading_gain 0.5-3.0 x v_max 0.3-0.5 x goal bearing
  // 0.3-1.2 rad: the regulator settles within 0.02-0.19 rad of the bearing,
  // while a disabled regulator (heading_gain = 0) sits at the full bearing,
  // 0.30-1.20 rad. 0.25 separates the two bands.
  expect(run.final_heading_error < 0.25f,
         "the body ends up pointing along the path it drove (heading error " +
             std::to_string(run.final_heading_error) + " rad)");
  expect(run.max_abs_w > 0.05f,
         "the regulator actually commanded yaw, so the check above is not vacuous (max |w| " +
             std::to_string(run.max_abs_w) + " rad/s)");
  expectLimitsRespected(run, "off-axis goal");
}

/// THE post-condition of the whole controller: the twist it emits must be able
/// to stop, along its own direction of travel, before it touches anything.
/// Asserted on the OUTPUT rather than on the lattice, over randomised worlds
/// AND randomised parameters, so it holds however the command was arrived at.
/// R15 H1/H2 were both found this way - the v = 0 row of the lattice never
/// reached the contact test, and the stopping distance was computed from the
/// forward component - and no per-scenario threshold could have seen either.
/// It then found a third: the emergency fallback kept a residual lateral
/// velocity and a residual yaw whose COMBINED arc could no longer stop, which
/// gating the lateral component alone did not catch.
///
/// The margin is re-derived here independently of the core's own support
/// function, so a mistake in that function shows up as a violation rather than
/// cancelling out.
///
/// Bodies already INSIDE their own margin are excluded: there the documented
/// behaviour is escape - motion away from the offending point - and no speed
/// could satisfy a stopping test. That regime is not covered by any assertion
/// here; see the R15 response for why no separating criterion was found.
void
testEmittedCommandCanAlwaysStop()
{
  unsigned seed = 20260902u;
  const auto next = [&seed]() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<float>((seed >> 8) & 0xFFFFu) / 65535.0f;
  };

  int violations = 0;
  int stop_while_moving = 0;
  std::string stop_while_moving_worst;
  int moving = 0;
  int evaluated = 0;
  std::string worst;
  int independent_violations = 0;
  int independent_checks = 0;
  std::string independent_worst;
  for (int trial = 0; trial < 6000; ++trial)
  {
    bac::Params params;
    params.motion_model.type = bac::MotionModelType::OMNI;
    params.limits.v_max = 0.2f + next() * 0.6f;
    params.limits.vy_max = 0.1f + next() * 0.6f;
    params.limits.v_min = 0.0f;
    params.limits.w_max = 0.5f + next() * 1.5f;
    // Acceleration limits far above anything one control period can need, so
    // limitReachableCommand is a no-op and the emitted twist IS the candidate
    // the scorer admitted. That isolates the candidate stage, which is where
    // R15 H1 and H2 lived. The output stage pulls a command back TOWARDS the
    // current velocity and re-checks it against a window capped by the
    // remaining path, so mixing the two regimes here would test a different
    // contract than the one this assertion states.
    params.limits.acc_v = 1000.0f;
    params.limits.acc_w = 1000.0f;
    params.control_period = 0.05f;
    params.stop_decel = 0.4f + next() * 1.6f;
    params.brake_reaction_time = next() * 0.25f;
    params.footprint.front = 0.2f + next() * 0.3f;
    params.footprint.rear = -(0.2f + next() * 0.3f);
    params.footprint.width = 0.3f + next() * 0.5f;
    params.safety_margin.front = 0.05f + next() * 0.3f;
    params.safety_margin.rear = params.safety_margin.front;
    params.safety_margin.side = 0.05f + next() * 0.3f;
    params.avoid_margin.side = 0.4f + next() * 0.6f;
    params.sim_time = 1.5f + next() * 2.0f;
    params.heading_gain = next() * 3.0f;
    bac::BacCore core(params);

    std::vector<bac::Point2D> points;
    const int clusters = 1 + static_cast<int>(next() * 3.0f);
    for (int c = 0; c < clusters; ++c)
    {
      const float bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
      const float distance = 0.35f + next() * 1.3f;
      for (int k = -14; k <= 14; ++k)
      {
        const float spread = 0.05f * static_cast<float>(k);
        points.push_back({ distance * std::cos(bearing) - spread * std::sin(bearing),
                           distance * std::sin(bearing) + spread * std::cos(bearing) });
      }
    }
    const float path_bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
    std::vector<bac::Point2D> path;
    for (int i = 1; i <= 20; ++i)
    {
      const float r = 0.25f * static_cast<float>(i);
      path.push_back({ r * std::cos(path_bearing), r * std::sin(path_bearing) });
    }
    // Drawn from the set the controller itself can command, so the first tick
    // is not asked to recover from a state it would never have produced
    // (limits.v_min is 0 here, so there is no reverse to start from).
    bac::Twist2D current(next() * params.limits.v_max,
                         (next() * 2.0f - 1.0f) * params.limits.w_max,
                         (next() * 2.0f - 1.0f) * params.limits.vy_max);

    const int ticks = 1 + static_cast<int>(next() * 3.0f);
    for (int tick = 0; tick < ticks; ++tick)
    {
      const bac::Result result = core.process(points, path, current);
      const bac::Twist2D out = result.output;
      current = out;
      const float speed = out.speed();
      // Status::STOP means holding still. R17 H3: this increment existed only
      // in the second sweep, so this sweep's assertion on it could never fire.
      if (result.status == bac::Status::STOP && speed > 1e-3f)
      {
        if (stop_while_moving == 0)
        {
          stop_while_moving_worst =
              "speed " + std::to_string(speed) + " m/s, vy " + std::to_string(out.vy);
        }
        ++stop_while_moving;
      }
      if (speed <= 1e-3f)
      {
        continue;
      }
      ++moving;

      const float horizon = std::max(speed * params.sim_time, params.min_eval_distance);
      const float ux = out.v / speed;
      const float uy = out.vy / speed;
      const float margin =
          ((ux >= 0.0f) ? params.safety_margin.front * ux : params.safety_margin.rear * -ux) +
          params.safety_margin.side * std::fabs(uy);
      const float decel = std::max(params.stop_decel, 0.1f);
      const float needed =
          speed * speed / (2.0f * decel) + speed * params.brake_reaction_time;

      // R18 H1 / R19 H1: the same invariant, against a contact distance derived
      // here from the rectangle instead of read back from the evaluator - and
      // evaluated ABOVE the evaluator's own two gates, so that the evaluator
      // failing to see a contact cannot excuse the tick from being checked.
      // See independentSearchWindow for why the window is needed.
      {
        const float window = independentSearchWindow(params, out, horizon);
        const float contact = independentContactDistance(points, params.footprint, out);
        const float own_free_run = contact - margin;
        if (contact <= window && own_free_run > 0.0f)
        {
          ++independent_checks;
          if (own_free_run <= needed - 1e-4f)
          {
            if (independent_violations == 0)
            {
              independent_worst = "out (" + std::to_string(out.v) + ", " +
                                  std::to_string(out.w) + ", " + std::to_string(out.vy) +
                                  ") speed " + std::to_string(speed) + ", free run " +
                                  std::to_string(own_free_run) + ", needs " +
                                  std::to_string(needed);
            }
            ++independent_violations;
          }
        }
      }

      const bac::ArcEvaluation eval =
          core.evaluateArcWindows(points, out, horizon, horizon);
      if (eval.blocking_s >= 1e9f)
      {
        continue;
      }
      const float free_run = eval.blocking_s - margin;
      if (free_run <= 0.0f)
      {
        continue;  // already inside the margin: the escape regime, see above
      }
      ++evaluated;
      if (free_run <= needed - 1e-4f)
      {
        if (violations == 0)
        {
          worst = "trial " + std::to_string(trial) + " out (" + std::to_string(out.v) + ", " +
                  std::to_string(out.w) + ", " + std::to_string(out.vy) + ") speed " +
                  std::to_string(speed) + ", free run " + std::to_string(free_run) +
                  ", needs " + std::to_string(needed);
        }
        ++violations;
      }
    }
  }

  // Counts the ticks where the invariant was actually EVALUATED, not the ticks
  // where the vehicle merely moved. R16 M5 found the previous guard
  // anti-correlated with coverage: pushing the obstacles away raised `moving`
  // and lowered the number of checks to 16.
  // A COVERAGE GUARD, not a mutation detector - see the note on
  // `independent_checks` below, which measures that for all five of this
  // file's coverage counts at once.
  expect(evaluated > 60,
         "the sweep reached the invariant often enough to mean something (" +
             std::to_string(evaluated) + " of " + std::to_string(moving) + " moving ticks)");
  expect(stop_while_moving == 0,
         "no tick reports STOP while it is still translating (" +
             std::to_string(stop_while_moving) + "; first: " + stop_while_moving_worst + ")");
  // NOT redundant with the independent invariant below, although it now kills
  // nothing that the independent one does not. Measured over the 52 mutations
  // of src/ and bac_core.hpp this suite is scored against: this assertion kills
  // {adm_free_run_half_needed, adm_travel_margin_front_only}, and the union of
  // the three independent-contact assertions (this sweep's, the second sweep's
  // and the straight-command sweep's) kills 23, including both of those. The
  // set this one ADDS is EMPTY.
  //
  // It stays because it is a different question, not a weaker one. This asks
  // whether the evaluator's OWN answer and the published twist agree; the
  // independent version never looks at the evaluator at all. A defect that
  // makes `evaluateArcWindows` disagree with the output stage while the true
  // geometry stays reachable is invisible to the independent check by
  // construction, and this is the only assertion in the suite that would see
  // it. The 52 mutations happen not to contain one - that is a property of the
  // mutation set, not evidence that the failure mode cannot occur.
  //
  // R18 H1 introduced the independent derivation and R19 H1 (187c053) lifted it
  // above the evaluator's gates, which is what made the inclusion total here.
  expect(violations == 0,
         "every emitted twist can stop before contact along its own direction of travel (" +
             std::to_string(violations) + " violations; first: " + worst + ")");
  // R18 H1: separate evidence rather than a restatement - this requirement
  // does not come from the code that computes the swept region, so a defect
  // that shrinks that region shows up here instead of cancelling out.
  //
  // R19 M6 asked whether this threshold separates a normal band from a
  // destructive one. It does not, and after the R19 H1 move it cannot: the
  // count no longer depends on the evaluator at all, so evaluator mutations
  // barely move it. Measured over 52 mutations of src/ and of bac_core.hpp
  // (unmutated 117): 27, 49, 79, then 85, 91, 95, 96, 112, 112, 112, ... up to
  // 533. Three fall under 80:
  //
  //   27  eval_angle_max halved in bac_core.hpp   (8339 moving ticks)
  //   49  emergency layer's lateral term dropped  (4471 moving ticks)
  //   79  vy dropped from projectConstantCommand  (7266 moving ticks)
  //
  // R19 V2-M1: an earlier revision of this comment said only "the one mutation
  // that stops the vehicle moving" falls under 80, and that no mutation of the
  // evaluator's geometry ever trips it. Both are false. The LOWEST of the
  // three moves the body MORE than the unmutated run does (8339 against 8314),
  // and it is a purely optimistic change to the evaluator's geometry. It is
  // invisible to the invariant below for a specific reason: `eval_angle_max`
  // is a constant SHARED by the product and by `independentSearchWindow` in
  // this file, so halving it shrinks the test's own window in step and the
  // independent violation count stays at 0 (see the R19 response, H1). Halving
  // the SAME cap inside the evaluator only - so that the test's window does not
  // follow - gives 1 and 89 violations instead. Four assertions in this
  // file react to the shared constant being halved, and all four are coverage
  // guards rather than safety ones: `evaluated > 60` and this threshold in the
  // first sweep (117 -> 27), and `checked > 2000` and its counterpart in the
  // second (3150 -> 1723). No safety invariant here reacts at all.
  //
  // It is still kept as a VACUITY guard - the sweep still drives at obstacles -
  // and not as a mutation detector: over those 52 mutations no mutation is
  // killed by this assertion, by its counterpart in the second sweep, or by the
  // one in the third sweep alone. Every mutation that trips one of the three
  // also fails at least one assertion that is not a coverage count (the
  // eval_angle_max one fails only coverage counts HERE, but is killed by
  // BacScenarioHarness, BacAckermannScenarios, BacOutputStageUnit and
  // BacCoreUnit). Before the H1 move one mutation did die on a coverage count
  // alone (`rho_max * 0.90`, at 71 checks); it now dies on the invariant below.
  //
  // R19 M6 also noted that this count used to be numerically identical to
  // `evaluated`. Unmutated it still is - 117 == 117 here, and 3150 == 3150 in
  // the second sweep - but under mutation it is not (`rho_max * 0.95` gives 118
  // here against 90 there, the sigma flip 115 against 884).
  //
  // WORDING. This message and the one in the second sweep were character-for-
  // character identical until this change, so a CI log could not say which of
  // the two had failed - and they do not fail together. Measured over the 52
  // mutations by position rather than by message text, this one fails under
  // {emerg_normalized_lat_removed, geo_param_eval_angle_half,
  // omni_project_no_vy} and the second sweep's under
  // {emerg_normalized_lat_removed, geo_param_eval_angle_half,
  // omni_limit_reachable_bypass}. Each names its own sweep now.
  //
  // THIS IS A COVERAGE GUARD, NOT A MUTATION DETECTOR. Over those 52 mutations
  // no mutation dies on this assertion, or on any of the other four coverage
  // counts in this file, alone: every mutation that trips one also fails at
  // least one assertion that is not a coverage count. Read a failure here as
  // "the sweep stopped reaching its own invariant", and go find what made it
  // stop; do not read it as a safety violation.
  expect(independent_checks > 80,
         "the first sweep's independently derived contact test ran often enough to mean "
         "something (" +
             std::to_string(independent_checks) + " checks)");
  expect(independent_violations == 0,
         "every emitted twist can stop before a contact distance derived independently "
         "of the evaluator (" +
             std::to_string(independent_violations) + " violations; first: " +
             independent_worst + ")");
}

/// The same invariant as above, but with acceleration limits that BIND, so the
/// output reachability stage and the emergency fallback are the code deciding
/// what gets published. R16 H4 found that the R15 fix to the fallback ladder
/// was never executed by any test, because the sweep above deliberately raises
/// the acceleration limits to isolate the candidate stage. Measured over this
/// grid the fallback is entered 386 times and its gate actually rejects the
/// combined twist 67 times, so the ladder is exercised rather than merely
/// reached.
///
/// This pass is also what caught R16 H2: before the deadband was re-checked,
/// it left 7 published twists that could not stop, every one of them a crab
/// whose yaw rate had been zeroed after the arc was checked.
void
testEmittedCommandCanStopWhenAccelerationBinds()
{
  unsigned seed = 7u;
  const auto next = [&seed]() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<float>((seed >> 8) & 0xFFFFu) / 65535.0f;
  };

  int checked = 0;
  int violations = 0;
  int stop_while_moving = 0;
  std::string stop_while_moving_worst;
  std::string worst;
  int independent_violations = 0;
  int independent_checks = 0;
  std::string independent_worst;
  // DO NOT REDUCE THIS TRIAL COUNT without re-measuring. The separation this
  // sweep buys is ONE DRAW wide.
  //
  // `emerg_zone_no_brake` (the emergency zone's brake-distance extension
  // removed, `zone_x_max += brake_distance * ux` -> `+= 0.0f * ...`) is killed
  // here and NOWHERE ELSE: under that mutation a full ctest run fails 1 of 10
  // tests, BacOmniScenarios, and inside this binary only the
  // `independent_violations == 0` assertion below fires. It fires with exactly
  // 1 violation, and that violation is drawn at TRIAL 22557 of the 40000
  // (independent_checks was 1890 at that point). Truncate this loop at 22557
  // trials or fewer and the count is 0: the mutation then survives the entire
  // suite. At 10000 trials it is 0 violations and both vacuity guards below
  // fail as well, unmutated too (773 checks unmutated, 845 mutated) - so 10000
  // does not merely lose the kill, it stops being a valid sweep.
  //
  // The vacuity floor and the kill nearly coincide: `independent_checks > 2000`
  // is first satisfied at trial 25861 unmutated (23765 under the mutation), so
  // the smallest non-vacuous sweep is already past the one violating draw. The
  // count is not padding.
  //
  // NOT MEASURED: what a different `seed` does to any of this. Every number
  // above is for seed 7u. Do not assume the draw survives a reseed - measure it.
  for (int trial = 0; trial < 40000; ++trial)
  {
    bac::Params params;
    params.motion_model.type = bac::MotionModelType::OMNI;
    params.limits.v_max = 0.2f + next() * 0.6f;
    params.limits.vy_max = 0.1f + next() * 0.6f;
    params.limits.v_min = 0.0f;
    params.limits.w_max = 0.5f + next() * 1.5f;
    // Low enough that one control period cannot cancel the current velocity.
    params.limits.acc_v = 0.1f + next() * 0.5f;
    params.limits.acc_w = 0.2f + next() * 1.0f;
    params.control_period = 0.05f;
    params.stop_decel = 0.4f + next() * 1.6f;
    params.brake_reaction_time = next() * 0.25f;
    params.footprint.front = 0.2f + next() * 0.3f;
    params.footprint.rear = -(0.2f + next() * 0.3f);
    params.footprint.width = 0.3f + next() * 0.5f;
    params.safety_margin.front = 0.05f + next() * 0.3f;
    params.safety_margin.rear = params.safety_margin.front;
    params.safety_margin.side = 0.05f + next() * 0.3f;
    params.avoid_margin.side = 0.4f + next() * 0.6f;
    params.sim_time = 1.5f + next() * 2.0f;
    params.heading_gain = next() * 3.0f;
    bac::BacCore core(params);

    std::vector<bac::Point2D> points;
    const float bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
    const float distance = 0.35f + next() * 1.0f;
    for (int k = -14; k <= 14; ++k)
    {
      const float spread = 0.05f * static_cast<float>(k);
      points.push_back({ distance * std::cos(bearing) - spread * std::sin(bearing),
                         distance * std::sin(bearing) + spread * std::cos(bearing) });
    }
    const float path_bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
    std::vector<bac::Point2D> path;
    for (int i = 1; i <= 20; ++i)
    {
      const float r = 0.25f * static_cast<float>(i);
      path.push_back({ r * std::cos(path_bearing), r * std::sin(path_bearing) });
    }
    const bac::Twist2D current(next() * params.limits.v_max,
                               (next() * 2.0f - 1.0f) * params.limits.w_max,
                               (next() * 2.0f - 1.0f) * params.limits.vy_max);

    const bac::Result result = core.process(points, path, current);
    const bac::Twist2D out = result.output;
    const float speed = out.speed();
    // Status::STOP means the vehicle is holding still. R16 M4 found it reported
    // while still translating sideways, because the test read v and w only.
    if (result.status == bac::Status::STOP && speed > 1e-3f)
    {
      if (stop_while_moving == 0)
      {
        stop_while_moving_worst = "speed " + std::to_string(speed) + " m/s, vy " +
                                  std::to_string(out.vy);
      }
      ++stop_while_moving;
    }
    if (speed <= 1e-3f)
    {
      continue;
    }
    const float horizon = std::max(speed * params.sim_time, params.min_eval_distance);
    const float ux = out.v / speed;
    const float uy = out.vy / speed;
    const float margin =
        ((ux >= 0.0f) ? params.safety_margin.front * ux : params.safety_margin.rear * -ux) +
        params.safety_margin.side * std::fabs(uy);
    const float decel = std::max(params.stop_decel, 0.1f);
    const float needed = speed * speed / (2.0f * decel) + speed * params.brake_reaction_time;

    // R18 H1 / R19 H1: as in the sweep above - derived here rather than read
    // back from the evaluator, and placed ABOVE the evaluator's own two gates.
    {
      const float window = independentSearchWindow(params, out, horizon);
      const float contact = independentContactDistance(points, params.footprint, out);
      const float own_free_run = contact - margin;
      if (contact <= window && own_free_run > 0.0f)
      {
        ++independent_checks;
        if (own_free_run <= needed - 1e-4f)
        {
          if (independent_violations == 0)
          {
            independent_worst = "out (" + std::to_string(out.v) + ", " +
                                std::to_string(out.w) + ", " + std::to_string(out.vy) +
                                ") speed " + std::to_string(speed) + ", free run " +
                                std::to_string(own_free_run) + ", needs " +
                                std::to_string(needed);
          }
          ++independent_violations;
        }
      }
    }

    const bac::ArcEvaluation eval = core.evaluateArcWindows(points, out, horizon, horizon);
    if (eval.blocking_s >= 1e9f)
    {
      continue;
    }
    const float free_run = eval.blocking_s - margin;
    if (free_run <= 0.0f)
    {
      continue;  // the escape regime, as above
    }
    ++checked;
    if (free_run <= needed - 1e-4f)
    {
      if (violations == 0)
      {
        worst = "trial " + std::to_string(trial) + " out (" + std::to_string(out.v) + ", " +
                std::to_string(out.w) + ", " + std::to_string(out.vy) + ") free run " +
                std::to_string(free_run) + ", needs " + std::to_string(needed);
      }
      ++violations;
    }
  }

  // A COVERAGE GUARD, not a mutation detector; see the note in the first sweep.
  expect(checked > 2000,
         "the binding-acceleration sweep reached the invariant often enough to matter (" +
             std::to_string(checked) + ")");
  // R18 L4: measured 0 here, against 101 with `vy` dropped from the Status::STOP
  // condition. The 25 / 32 / 7 this comment used to quote are the bands of the
  // NEXT assertion, not of this one.
  expect(stop_while_moving == 0,
         "no tick reports STOP while it is still translating (" +
             std::to_string(stop_while_moving) + "; first: " + stop_while_moving_worst + ")");
  // Measured: 0 here, against 25 with the fallback gate removed, 32 with
  // stoppable() forced true, and 7 with the deadband left unchecked.
  //
  // As in the first sweep, this is NOT a restatement of the independent
  // invariant below, and here the inclusion is nearly - but not quite - total.
  // Measured over the same 52 mutations: this assertion kills
  // {adm_travel_margin_front_only, db_nobrake, geo_cx_sign, geo_sigma_flip};
  // the union of the three independent-contact assertions kills 23 of the 52
  // and contains all of those except `db_nobrake`. So the set this one adds is
  // exactly {db_nobrake} - the deadband re-check disabled - and that mutation
  // is caught in this same file anyway, by
  // testDeadbandIsSkippedRatherThanBreakingAdmissibility's `kept > 120`
  // (measured: 0 such ticks under the mutation, 255 unmutated). Those two are
  // the ONLY two assertions in the whole suite that fail under it.
  //
  // Do not read that as "the independent version replaces this one". This one
  // is the only place that compares the evaluator's answer with what the OUTPUT
  // STAGE and the emergency fallback actually publish; the independent version
  // reads the rectangle and the point cloud and never asks the evaluator
  // anything. Deleting it would leave the evaluator-versus-output disagreement
  // untested, which is the failure mode R16 H4 and R16 H2 were both found in.
  expect(violations == 0,
         "what the output stage and the fallback publish can stop before contact (" +
             std::to_string(violations) + " violations; first: " + worst + ")");
  // R18 H1: separate evidence rather than a restatement - this requirement
  // does not come from the code that computes the swept region, so a defect
  // that shrinks that region shows up here instead of cancelling out.
  //
  // R19 M6, as above: a vacuity guard, not a separating threshold. Measured
  // over the same 52 mutations (unmutated 3150): 425 (output reachability stage
  // bypassed), 817 (emergency lateral term dropped), 1723 (eval_angle_max
  // halved in bac_core.hpp), then 2931, 3131, ... upwards to 3407.
  //
  // R19 V2-M1: an earlier revision said the ones below 2000 are all mutations
  // that stop the vehicle moving. They are not - the lowest, the output
  // reachability bypass, leaves the moving-tick count at the unmutated 8314 in
  // the first sweep, and the eval_angle_max one raises it. What all three do is
  // change how far the sweep gets before the shared window closes. All three
  // are killed by assertions that are not coverage counts (the reachability
  // bypass by the governor's sideways-versus-head-on comparison here, the other
  // two by other test binaries), so nothing is killed by this assertion alone.
  // Unmutated the count is still numerically identical to `checked`
  // (3150 == 3150); under mutation it is not (`rho_max` without |w| gives 3164
  // here against 5614 there).
  //
  // WORDING, and A COVERAGE GUARD rather than a mutation detector: see the
  // first sweep's note. This message used to be identical to that one; the two
  // now name their sweep, because the mutations they fail under differ
  // (measured - this one adds omni_limit_reachable_bypass and drops
  // omni_project_no_vy relative to the first).
  expect(independent_checks > 2000,
         "the binding-acceleration sweep's independently derived contact test ran often "
         "enough to mean something (" +
             std::to_string(independent_checks) + " checks)");
  expect(independent_violations == 0,
         "every emitted twist can stop before a contact distance derived independently "
         "of the evaluator (" +
             std::to_string(independent_violations) + " violations; first: " +
             independent_worst + ")");
}

/// The output deadband is not applied when applying it would break
/// admissibility (R17 H2), and that is observable from outside.
///
/// R18 H4: the R17 response claimed no regression could distinguish the fix,
/// on the grounds that both behaviours are safe. They are, but they are not
/// the same output. Before the fix the stage published either the deadbanded
/// command - whose yaw is zero once below `angvel_min` - or an exact
/// (0, 0, 0), so a published yaw rate strictly between zero and `angvel_min`
/// was structurally impossible. Keeping the command as selected is the only
/// thing in the stage that produces one.
///
/// `angvel_min` is drawn wider than the shipped 0.01 rad/s so the branch is
/// reached often enough for the count to be a band rather than a handful.
/// R19 M4: the moving-tick denominators below were transposed here before -
/// 21342 - 21087 = 255 is exactly the number of ticks the FIRST rejected
/// behaviour stops, not a coincidence. Re-measured on this fixture:
///
///   current                                 255 kept of 21342 moving
///   brake to a standstill instead             0 kept of 21087 moving
///   apply the deadband regardless             0 kept of 21342 moving
///
/// and the last of the three also fails the binding sweep above with 7
/// violations.
void
testDeadbandIsSkippedRatherThanBreakingAdmissibility()
{
  unsigned seed = 913u;
  const auto next = [&seed]() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<float>((seed >> 8) & 0xFFFFu) / 65535.0f;
  };

  int kept = 0;
  int moving = 0;
  for (int trial = 0; trial < 30000; ++trial)
  {
    bac::Params params;
    params.motion_model.type = bac::MotionModelType::OMNI;
    params.limits.v_max = 0.2f + next() * 0.6f;
    params.limits.vy_max = 0.1f + next() * 0.6f;
    params.limits.v_min = 0.0f;
    params.limits.w_max = 0.5f + next() * 1.5f;
    params.limits.acc_v = 0.1f + next() * 0.5f;
    params.limits.acc_w = 0.2f + next() * 1.0f;
    params.control_period = 0.05f;
    params.stop_decel = 0.4f + next() * 1.6f;
    params.brake_reaction_time = next() * 0.25f;
    params.footprint.front = 0.2f + next() * 0.3f;
    params.footprint.rear = -(0.2f + next() * 0.3f);
    params.footprint.width = 0.3f + next() * 0.5f;
    params.safety_margin.front = 0.05f + next() * 0.3f;
    params.safety_margin.rear = params.safety_margin.front;
    params.safety_margin.side = 0.05f + next() * 0.3f;
    params.avoid_margin.side = 0.4f + next() * 0.6f;
    params.sim_time = 1.5f + next() * 2.0f;
    params.heading_gain = next() * 3.0f;
    params.angvel_min = 0.05f + next() * 0.30f;
    bac::BacCore core(params);

    std::vector<bac::Point2D> points;
    const float bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
    const float distance = 0.35f + next() * 1.0f;
    for (int k = -14; k <= 14; ++k)
    {
      const float spread = 0.05f * static_cast<float>(k);
      points.push_back({ distance * std::cos(bearing) - spread * std::sin(bearing),
                         distance * std::sin(bearing) + spread * std::cos(bearing) });
    }
    const float path_bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
    std::vector<bac::Point2D> path;
    for (int i = 1; i <= 20; ++i)
    {
      const float r = 0.25f * static_cast<float>(i);
      path.push_back({ r * std::cos(path_bearing), r * std::sin(path_bearing) });
    }
    const bac::Twist2D current(next() * params.limits.v_max,
                               (next() * 2.0f - 1.0f) * params.limits.w_max,
                               (next() * 2.0f - 1.0f) * params.limits.vy_max);

    const bac::Twist2D out = core.process(points, path, current).output;
    if (out.speed() > 1e-3f)
    {
      ++moving;
    }
    if (out.w != 0.0f && std::fabs(out.w) < params.angvel_min)
    {
      ++kept;
    }
  }

  expect(kept > 120,
         "the output stage publishes the command as selected where the deadband would "
         "have broken admissibility (" +
             std::to_string(kept) + " such ticks of " + std::to_string(moving) + " moving)");
}

/// The same invariant once more, on commands that do NOT rotate.
///
/// R18 H1: the two sweeps above almost never emit a straight twist - the yaw
/// regulator is active in both - so the evaluator's straight branch, which is
/// where `lead` and the left/right support extents decide contact, was reached
/// by neither. Measured among the ticks that REACH their invariant: 0 straight
/// ticks in the first sweep and 106 in the second, against 885 checks here.
/// (Counted over all moving ticks instead, rather than over the checked ones,
/// the two are 70 of 8314 and 358 of 28246 - still far below this sweep, and
/// the denominator is stated because the two counts differ.)
/// `frame.lead = body.front` survives both of them and fails here with 2
/// violations. R19 L2: this - not the footprint distribution - is why.
///
/// `heading_gain` is zero so the regulator asks for no yaw, which is what puts
/// the evaluator on its straight branch; the lattice is still the full
/// (v, vy) grid, so the direction of travel covers every bearing.
void
testStraightCommandsClearTheirOwnFootprint()
{
  unsigned seed = 411u;
  const auto next = [&seed]() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<float>((seed >> 8) & 0xFFFFu) / 65535.0f;
  };

  int checks = 0;
  int violations = 0;
  int rotating = 0;
  std::string worst;
  // DO NOT REDUCE THIS TRIAL COUNT without re-measuring. As in the sweep above,
  // the separation is TWO DRAWS wide.
  //
  // `geo_perp_swap` (the evaluator's left/right support extents exchanged) is
  // killed here and NOWHERE ELSE: under it a full ctest run fails 1 of 10
  // tests, BacOmniScenarios, and inside this binary only the
  // `violations == 0` assertion below fires. It fires with exactly 2
  // violations, drawn at TRIAL 27784 and TRIAL 36108 of the 60000 (checks 466
  // and 599 at those points). Truncate this loop at 27784 trials or fewer and
  // the count is 0: the mutation then survives the entire suite. At 15000
  // trials it is 0 violations and the `checks > 600` guard below fails as well,
  // unmutated too (222 checks unmutated, 246 mutated).
  //
  // Here too the vacuity floor sits past the kills: `checks > 600` is first
  // satisfied at trial 40474 unmutated (36149 under the mutation), so the
  // smallest non-vacuous sweep already contains both violating draws.
  //
  // NOT MEASURED: the effect of a different `seed`. Every number above is for
  // this loop's own seed. Two draws in 60000 is not a margin that should be
  // assumed to be seed-independent; measure it before relying on it.
  for (int trial = 0; trial < 60000; ++trial)
  {
    bac::Params params;
    params.motion_model.type = bac::MotionModelType::OMNI;
    params.limits.v_max = 0.2f + next() * 0.6f;
    params.limits.vy_max = 0.1f + next() * 0.6f;
    params.limits.v_min = 0.0f;
    params.limits.w_max = 0.5f + next() * 1.5f;
    params.limits.acc_v = 1000.0f;
    params.limits.acc_w = 1000.0f;
    params.control_period = 0.05f;
    params.stop_decel = 0.4f + next() * 1.6f;
    params.brake_reaction_time = next() * 0.25f;
    // Deliberately asymmetric, and wide enough that `width / 2` can exceed
    // `front`. Replacing the support extent along the direction of travel with
    // `front` is only OPTIMISTIC where the true extent is larger, which for a
    // sideways command means exactly `width / 2 > front`.
    //
    // R19 L2: that condition is NOT what the other sweeps lack. Counted over
    // their own draws, `width / 2 > front` holds in 1586 of 6000 trials (26.4%)
    // in the first sweep and 10782 of 40000 (27.0%) in the second, against
    // 23312 of 60000 (38.9%) here. Widening the footprint distribution raises
    // the rate but does not create the condition. What the other two sweeps
    // lack is straight ticks, which is the other half of this comment; see
    // below.
    params.footprint.front = 0.15f + next() * 0.45f;
    params.footprint.rear = -(0.10f + next() * 0.20f);
    params.footprint.width = 0.3f + next() * 0.7f;
    params.safety_margin.front = 0.05f + next() * 0.3f;
    params.safety_margin.rear = params.safety_margin.front;
    params.safety_margin.side = 0.05f + next() * 0.3f;
    params.avoid_margin.side = 0.4f + next() * 0.6f;
    params.sim_time = 1.5f + next() * 2.0f;
    params.heading_gain = 0.0f;  // no yaw asked for, so the command is straight
    bac::BacCore core(params);

    std::vector<bac::Point2D> points;
    const float bearing = (next() * 2.0f - 1.0f) * static_cast<float>(M_PI);
    const float distance = 0.35f + next() * 1.1f;
    for (int k = -14; k <= 14; ++k)
    {
      const float spread = 0.05f * static_cast<float>(k);
      points.push_back({ distance * std::cos(bearing) - spread * std::sin(bearing),
                         distance * std::sin(bearing) + spread * std::cos(bearing) });
    }
    // Aimed close to the cluster, so the vehicle is actually driven at the
    // obstacle and the invariant is reached rather than trivially satisfied by
    // an empty direction of travel. R19 L1: drawing the bearing uniformly
    // instead leaves 56 checks in 12000 trials, against 177 aimed over the same
    // 12000 and 885 aimed over the 60000 this loop runs; 303 uniform over
    // 60000.
    const float path_bearing = bearing + (next() * 2.0f - 1.0f) * 0.7f;
    std::vector<bac::Point2D> path;
    for (int i = 1; i <= 20; ++i)
    {
      const float r = 0.25f * static_cast<float>(i);
      path.push_back({ r * std::cos(path_bearing), r * std::sin(path_bearing) });
    }
    const bac::Twist2D current(next() * params.limits.v_max, 0.0f,
                               (next() * 2.0f - 1.0f) * params.limits.vy_max);

    const bac::Twist2D out = core.process(points, path, current).output;
    const float speed = out.speed();
    if (speed <= 1e-3f)
    {
      continue;
    }
    if (std::fabs(out.w) > 1e-4f)
    {
      ++rotating;
      continue;
    }
    const float contact = independentContactDistance(points, params.footprint, out);
    if (contact >= 1e9f)
    {
      continue;
    }
    const float ux = out.v / speed;
    const float uy = out.vy / speed;
    const float margin =
        ((ux >= 0.0f) ? params.safety_margin.front * ux : params.safety_margin.rear * -ux) +
        params.safety_margin.side * std::fabs(uy);
    const float free_run = contact - margin;
    if (free_run <= 0.0f)
    {
      continue;  // the escape regime, as in the sweeps above
    }
    ++checks;
    const float decel = std::max(params.stop_decel, 0.1f);
    const float needed = speed * speed / (2.0f * decel) + speed * params.brake_reaction_time;
    if (free_run <= needed - 1e-4f)
    {
      if (violations == 0)
      {
        worst = "trial " + std::to_string(trial) + " out (" + std::to_string(out.v) + ", " +
                std::to_string(out.vy) + ") speed " + std::to_string(speed) + ", free run " +
                std::to_string(free_run) + ", needs " + std::to_string(needed);
      }
      ++violations;
    }
  }

  // R19 M7 recorded that the destructive side of this threshold had never been
  // measured (the lowest over 61 mutations was 670, all above 600). Measured
  // here over 52 mutations of src/ and of bac_core.hpp (unmutated 885): 499
  // with the emergency layer's lateral term dropped, then 679, 867, 868, 871,
  // 874, 885 and upwards to 1037. So one mutation does land below, and 600
  // separates the set - but on one point only, and that mutation fails
  // assertions in five other test binaries as well. Like the two above, read
  // this as a vacuity guard, not as a kill.
  // A COVERAGE GUARD, not a mutation detector; see the note in the first sweep.
  expect(checks > 600,
         "the straight-command sweep reached the invariant often enough to matter (" +
             std::to_string(checks) + " checks, " + std::to_string(rotating) +
             " rotating ticks skipped)");
  expect(violations == 0,
         "every emitted straight twist can stop before a contact distance derived "
         "independently of the evaluator (" +
             std::to_string(violations) + " violations; first: " + worst + ")");
}

/// The v = 0 ROW of the lattice still translates - sideways - so it has to be
/// contact-checked like any other candidate. R15 H1 found it scored as a
/// turn-then-go rotation instead, which routed every pure-crab candidate around
/// the contact test and the stopping test.
///
/// Here forward motion is blocked by a wall exactly at footprint.front plus the
/// front margin, and the path leads sideways into a second wall. Every command
/// that progresses along that path either advances into the wall ahead or
/// slides into the one abeam, so the correct answer is to hold station.
/// Measured at an abeam distance of 0.70 m: with the row contact-checked the
/// vehicle commands 0.0000 m/s; scoring it as a rotation instead commands
/// 0.1800 m/s. R16 M7 found the previously recorded band ("0.0000 at every
/// abeam distance from 0.50 to 0.95") to be wrong - the mutation survives at
/// 0.50 through 0.65 - so the claim is now limited to the distance this
/// scenario actually runs.
void
testLateralRowIsContactChecked()
{
  bac::Params params = omniParams();
  params.limits.v_max = 0.6f;
  params.limits.vy_max = 0.6f;
  params.limits.acc_v = 1000.0f;  // isolate the candidate stage from reachability
  params.limits.acc_w = 1000.0f;
  params.heading_gain = 0.0f;
  bac::BacCore core(params);

  std::vector<bac::Point2D> points;
  for (int k = -40; k <= 40; ++k)
  {
    points.push_back({ 0.05f * static_cast<float>(k), 0.70f });  // abeam
    points.push_back({ 0.55f, 0.05f * static_cast<float>(k) });  // ahead, at the margin
  }
  std::vector<bac::Point2D> path;
  for (int i = 1; i <= 20; ++i)
  {
    path.push_back({ 0.0f, 0.25f * static_cast<float>(i) });
  }

  const bac::Result result = core.process(points, path, bac::Twist2D(0.0f, 0.0f, 0.3f));
  expect(result.output.speed() <= 1e-3f,
         "a sideways candidate that cannot clear the wall ahead of it is rejected, not "
         "emitted (speed " + std::to_string(result.output.speed()) + " m/s, twist " +
             std::to_string(result.output.v) + ", " + std::to_string(result.output.w) + ", " +
             std::to_string(result.output.vy) + ")");
}

/// The emergency layer decides on the direction of travel too. R15 H3 found it
/// reading `current.v` only in three places at once: the zone extended along
/// +x alone, a body crabbing at limits.vy_max counted as stationary and skipped
/// the hard brake, and the escape gate projected candidates onto emergency_x
/// while ignoring emergency_y.
///
/// A wall abeam at 0.52 m with the body sliding towards it at 0.3 m/s is inside
/// the zone once the zone follows the direction of travel, and outside it when
/// the zone only grows forwards. Measured over abeam distances 0.28-0.60 m,
/// the generalised layer stops at every distance up to 0.52 m and the
/// forward-axis one is already moving at 0.52 m (vy 0.2600), with
/// min_proximity_norm 0.9187 against 1.3500 at that distance.
void
testEmergencyLayerSeesLateralMotion()
{
  bac::Params params = omniParams();
  params.limits.v_max = 0.4f;
  params.limits.vy_max = 0.4f;
  params.heading_gain = 0.0f;  // hold the heading so the geometry stays fixed
  bac::BacCore core(params);

  std::vector<bac::Point2D> points;
  for (int k = -6; k <= 6; ++k)
  {
    points.push_back({ 0.05f * static_cast<float>(k), 0.52f });
  }
  std::vector<bac::Point2D> path;
  for (int i = 1; i <= 20; ++i)
  {
    path.push_back({ 0.0f, 0.25f * static_cast<float>(i) });  // straight at the wall
  }

  const bac::Result result = core.process(points, path, bac::Twist2D(0.0f, 0.0f, 0.3f));
  expect(result.min_proximity_norm < 1.0f,
         "a wall the body is sliding into is inside the emergency zone (normalised " +
             std::to_string(result.min_proximity_norm) + ")");
  expect(result.output.speed() <= 1e-3f,
         "and the sideways motion is stopped rather than continued (speed " +
             std::to_string(result.output.speed()) + " m/s, vy " +
             std::to_string(result.output.vy) + ")");
}

/// `Status::STOP` means the vehicle is holding still. R16 M4 found it reported
/// while the vehicle was still translating sideways at up to 0.49 m/s, because
/// the test that decides it read `v` and `w` and not `vy`. Subscribers of
/// `avoid_status` and the filter node's arbitration both act on this.
///
/// R16 M12: the same scenario exercises the heading wrap. The path tangent here
/// sits near -pi, so the centering bias pushes the pose reference across the
/// branch cut; unwrapped, the regulator takes the long way round - 3.70 rad
/// instead of 2.58 - and does it inside the passage, which is the opposite of
/// what the bias exists for.
void
testStopMeansStoppedAndHeadingTakesTheShortWay()
{
  bac::Params params = omniParams();
  bac::BacCore core(params);

  // Boxed in on three sides with the goal behind: the only honest answer is to
  // hold still, and the path tangent is near -pi.
  std::vector<bac::Point2D> points;
  for (int k = -20; k <= 20; ++k)
  {
    const float t = 0.05f * static_cast<float>(k);
    points.push_back({ 0.45f, t });
    points.push_back({ t, 0.42f });
    points.push_back({ t, -0.42f });
  }
  std::vector<bac::Point2D> path;
  for (int i = 1; i <= 20; ++i)
  {
    path.push_back({ -0.25f * static_cast<float>(i), 0.0f });
  }

  const bac::Result result = core.process(points, path, bac::Twist2D(0.0f, 0.0f, 0.2f));
  if (result.status == bac::Status::STOP)
  {
    expect(result.output.speed() <= 1e-3f,
           "a tick that reports STOP is not still translating (speed " +
               std::to_string(result.output.speed()) + " m/s, vy " +
               std::to_string(result.output.vy) + ")");
  }
  expect(std::fabs(result.output.w) <= params.limits.w_max + 1e-4f,
         "the yaw reference stays inside limits.w_max even with the path tangent near -pi (" +
             std::to_string(result.output.w) + ")");

  // The wrap itself. Inside a passage the centering bias is added to the path
  // heading error, and near the branch cut the sum can leave (-pi, pi]. Here
  // the tangent is at 3.05 rad and the bias is positive, so unwrapped the sum
  // is 3.24 and the regulator turns the LONG way: measured, the commanded yaw
  // is -0.1250 rad/s wrapped and +0.1250 unwrapped, a reversal.
  std::vector<bac::Point2D> corridor;
  for (int k = -60; k <= 60; ++k)
  {
    const float t = 0.05f * static_cast<float>(k);
    corridor.push_back({ t, 0.85f });   // asymmetric: more room to the left
    corridor.push_back({ t, -0.35f });
  }
  std::vector<bac::Point2D> rear_path;
  for (int i = 1; i <= 20; ++i)
  {
    const float r = 0.25f * static_cast<float>(i);
    rear_path.push_back({ r * std::cos(3.05f), r * std::sin(3.05f) });
  }
  bac::BacCore wrap_core(params);
  const bac::Result wrap = wrap_core.process(corridor, rear_path, bac::Twist2D(0.0f, 0.0f, 0.0f));
  expect(wrap.output.w < 0.0f,
         "a heading error past the branch cut turns the short way round (yaw " +
             std::to_string(wrap.output.w) + " rad/s)");
}

/// The proximity speed governor moderates speed in front of what the vehicle is
/// about to run into. R16 H3 found it reading the forward axis only: the same
/// wall, approached sideways, produced a command 32% faster than approaching it
/// head-on, because the wall fell into the side-envelope branch rather than the
/// head-on one.
///
/// This is a metamorphic check, not a threshold. With a SQUARE footprint and
/// isotropic margins the problem is invariant under a 90 degree rotation, so
/// driving at a wall and crabbing at the same wall rotated with it must be
/// governed the same way. Measured before the fix: 0.2600 head-on against
/// 0.3157-0.3423 sideways over wall distances 0.60-1.30 m.
void
testGovernorTreatsSidewaysLikeForward()
{
  bac::Params params = omniParams();
  params.limits.v_max = 0.6f;
  params.limits.vy_max = 0.6f;
  // Square body and isotropic margins: a 90 degree rotation is an exact
  // symmetry of the geometry, so any difference is the controller's.
  params.footprint.front = 0.25f;
  params.footprint.rear = -0.25f;
  params.footprint.width = 0.5f;
  params.safety_margin.front = 0.2f;
  params.safety_margin.rear = 0.2f;
  params.safety_margin.side = 0.2f;
  params.heading_gain = 0.0f;  // hold the heading so the rotation stays exact

  // Four rotations of the same problem: driving at the wall, crabbing left at
  // it, reversing at it, crabbing right at it. A square body under isotropic
  // margins makes all four the same problem, so any spread is the controller's.
  float worst_gap = 0.0f;
  std::string worst;
  for (const float wall : { 0.60f, 0.80f, 1.00f, 1.10f, 1.30f })
  {
    float reference = 0.0f;
    for (int rotation = 0; rotation < 4; ++rotation)
    {
      const float angle = 1.57079633f * static_cast<float>(rotation);
      const float ca = std::cos(angle), sa = std::sin(angle);
      const auto rotate = [&](float x, float y) {
        return bac::Point2D(x * ca - y * sa, x * sa + y * ca);
      };

      std::vector<bac::Point2D> points, path;
      for (int k = -30; k <= 30; ++k)
      {
        const float t = 0.05f * static_cast<float>(k);
        points.push_back(rotate(wall, t));
      }
      for (int i = 1; i <= 20; ++i)
      {
        path.push_back(rotate(0.25f * static_cast<float>(i), 0.0f));
      }
      const bac::Point2D velocity = rotate(0.5f, 0.0f);

      bac::BacCore core(params);
      const bac::Result result =
          core.process(points, path, bac::Twist2D(velocity.x, 0.0f, velocity.y));
      if (rotation == 0)
      {
        reference = result.output.speed();
        continue;
      }
      const float gap = result.output.speed() - reference;
      if (gap > worst_gap)
      {
        worst_gap = gap;
        worst = "wall " + std::to_string(wall) + " m, rotation " +
                std::to_string(rotation * 90) + " deg: " +
                std::to_string(result.output.speed()) + " against " +
                std::to_string(reference) + " head-on";
      }
    }
  }

  // The two axes are sampled at different resolutions, so exact equality is not
  // available; what must not happen is the sideways case being governed LESS.
  expect(worst_gap < 0.02f,
         "the governor moderates a wall approached sideways as it does one approached "
         "head-on (worst excess " + std::to_string(worst_gap) + " m/s; " + worst + ")");

  // MIRROR invariance, with an ASYMMETRIC footprint. The rotation check above
  // uses a square body, which is exactly why it could not see R17 H1: the
  // governor's lateral slab had been symmetrised about the travel axis, and the
  // true interval is asymmetric by (front + rear) * uy. A body whose front and
  // rear overhangs differ makes that term non-zero.
  //
  // The problem IS mirror-symmetric whatever the overhangs: the footprint is
  // symmetric about y = 0 and the margins are isotropic, so reflecting the
  // world, vy and w must reflect the command. Measured with front 0.70 and
  // rear -0.30 over the 144 cells this loop visits, the symmetrised form breaks
  // it as follows, all measured on this 144-cell grid (unmutated: worst
  // 2.98e-08 at either `heading_gain`, 0 cells broken):
  //
  //   mutation                                     gain 0        gain 1.5
  //   governor side_gap uses travel_left both      6 / 0.080000  8 / 0.080000
  //   governor side_gap uses travel_right both     6 / 0.080000  8 / 0.080000
  //   evaluator side_extent = perp_left           18 / 0.360000  0 / 2.98e-08
  //   evaluator side_extent = perp_right          18 / 0.360000  0 / 2.98e-08
  //   governor support along = front * dx          2 / 0.016828  0 / 2.98e-08
  //
  // The "8 of 144, worst 0.080000" this comment used to quote without naming
  // its mutation is ONE OF the first two rows and this check cannot tell them
  // apart: the travel_left form and its mirror image, the travel_right form,
  // produce the identical signature at both gains (R19 V2-M2 - an earlier
  // revision of this comment identified it as the travel_left form alone,
  // which the measurement does not support). Both survive the gain change. The
  // three rows below them are the reason R19 H2 sent the gain back to zero.
  //
  // A form which is genuinely SYMMETRISED is invisible here, measured: setting
  // travel_left and travel_right both to width / 2 leaves 0 / 144 at worst
  // 2.98e-08 at both gains, and so does making the along-travel term even in
  // dx (front * |dx|). Setting the evaluator's perp_left and perp_right both to
  // width / 2 leaves 0 / 144 at worst EXACTLY 0.000000 at gain 0 (and
  // 2.98e-08 at gain 1.5) - symmetrising that pair removes the last
  // asymmetric term, so the two mirrored runs agree bit for bit. Symmetrising
  // is itself mirror-symmetric, so what mirror invariance sees is the two
  // extents being used in the wrong ORDER, not their being made equal.
  // R18 M4: the numbers quoted here before all of this (186 of 4000, worst
  // 0.3496) came from a wider exploratory grid, not from this check.
  {
    bac::Params mirror = omniParams();
    mirror.limits.v_max = 0.6f;
    mirror.limits.vy_max = 0.6f;
    mirror.footprint.front = 0.70f;
    mirror.footprint.rear = -0.30f;
    mirror.footprint.width = 0.5f;
    mirror.safety_margin.front = 0.2f;
    mirror.safety_margin.rear = 0.2f;
    mirror.safety_margin.side = 0.2f;
    // R18 M3: the "w must reflect the command" half of this check used to be
    // vacuous - every output yaw rate was identically zero, because the grid
    // held no yaw at all (48 cells, cur_w fixed at 0, heading_gain 0).
    //
    // R19 H2: what fixed that is the `cur_w` dimension below, NOT the
    // heading_gain of 1.5 that was raised at the same time. Raising the gain
    // cost two kills that this check was the ONLY detector for:
    //   * the evaluator's
    //     `side_extent = (left_offset >= 0) ? perp_left : perp_right`
    //     reduced to `perp_left`;
    //   * the governor's `support()` along-travel term
    //     `(dx >= 0) ? front * dx : rear * dx` reduced to `front * dx`, which
    //     is a partial revert of the asymmetric support R17 H1 introduced.
    // With the gain at 1.5 both PASS this check. With the gain back at zero and
    // the cur_w grid kept they fail it again, at 18 of 144 cells (worst
    // 0.360000 m/s) and 2 of 144 (worst 0.016828 m/s). The yaw half stays
    // non-vacuous either way: 87 of 144 cells command a yaw rate at gain 0,
    // against 129 at gain 1.5.
    mirror.heading_gain = 0.0f;

    float worst_break = 0.0f;
    int nonzero_w = 0;
    int cells = 0;
    int breaks = 0;
    std::string worst_state;
    for (const float wall : { 0.60f, 0.80f, 1.00f, 1.20f })
      for (const float cur_v : { 0.0f, 0.2f, 0.4f })
        for (const float cur_w : { -0.4f, 0.0f, 0.4f })
        for (const float cur_vy : { -0.4f, -0.2f, 0.2f, 0.4f })
        {
          std::vector<bac::Point2D> points, mirrored, path, mirrored_path;
          for (int k = -30; k <= 30; ++k)
          {
            const float t = 0.05f * static_cast<float>(k);
            points.push_back({ t, wall });
            mirrored.push_back({ t, -wall });
          }
          for (int i = 1; i <= 20; ++i)
          {
            const float r = 0.25f * static_cast<float>(i);
            path.push_back({ r, 0.3f * r });
            mirrored_path.push_back({ r, -0.3f * r });
          }

          bac::BacCore a(mirror);
          const bac::Result ra = a.process(points, path, bac::Twist2D(cur_v, cur_w, cur_vy));
          bac::BacCore b(mirror);
          const bac::Result rb =
              b.process(mirrored, mirrored_path, bac::Twist2D(cur_v, -cur_w, -cur_vy));
          ++cells;
          if (ra.output.w != 0.0f)
          {
            ++nonzero_w;
          }

          const float dv = std::fabs(ra.output.v - rb.output.v);
          const float dvy = std::fabs(ra.output.vy + rb.output.vy);
          const float dw = std::fabs(ra.output.w + rb.output.w);
          const float break_size = std::max(dv, std::max(dvy, dw));
          if (break_size > 1e-3f)
          {
            ++breaks;
          }
          if (break_size > worst_break)
          {
            worst_break = break_size;
            worst_state = "wall " + std::to_string(wall) + " current (" +
                          std::to_string(cur_v) + ", " + std::to_string(cur_vy) + "): out (" +
                          std::to_string(ra.output.v) + ", " + std::to_string(ra.output.vy) +
                          ") against mirrored (" + std::to_string(rb.output.v) + ", " +
                          std::to_string(rb.output.vy) + ")";
          }
        }
    expect(worst_break < 1e-3f,
           "an asymmetric footprint does not break the mirror symmetry of the problem (" +
               std::to_string(breaks) + " of " + std::to_string(cells) + " cells, worst " +
               std::to_string(worst_break) + " m/s; " + worst_state + ")");
    // Measured 87 of 144 at heading_gain 0; the threshold is a quarter of the grid.
    expect(nonzero_w * 4 > cells,
           "the yaw half of that symmetry is actually exercised (" +
               std::to_string(nonzero_w) + " of " + std::to_string(cells) +
               " cells command a yaw rate)");
  }

  // R18 M2: mirror symmetry alone cannot see the two lateral extents being
  // SWAPPED, because swapping them is itself mirror-symmetric. Pin their
  // orientation directly.
  //
  // Crabbing left, the direction of travel is +y, so the left of the travel
  // line is -x - the REAR overhang - and its right is +x, the front. With front
  // 0.70 and rear -0.30 the swept box therefore reaches 0.30 m to the left of
  // the travel line and 0.70 m to the right. Swapping the two extents inverts
  // which side of the body the obstacle is treated as lying on, and the +0.70
  // case then runs at v_max.
  //
  // R19 M5: what these two pins actually measure, from a sweep of the obstacle
  // offset at 0.05 m pitch over -2.0..+2.0 and 0.5 m over -8.0..+8.0:
  //
  //   x <= -1.25          0.600000024 = v_max, not governed
  //   x = -1.00           0.583458
  //   x = -0.70           0.567726   <- `behind`
  //   x = -0.60 .. +0.20  0.561427
  //   x = +0.25 .. +1.00  0.560000   <- `ahead`
  //   x >= +1.05          0.600000024 = v_max, not governed
  //
  // So `behind` at -0.70 IS governed (0.5677 < 0.6000); an earlier revision of
  // this comment said it was "more than twice the rear reach away and must
  // not", which is false - the ungoverned region starts at x <= -1.25 on this
  // grid. And `ahead < 0.58` is not a statement about the front overhang: it
  // holds over the whole interval x in [-0.90, +1.00], every cell of it. What
  // the first pin can actually detect is a change that pushes the value up to
  // v_max, which is what swapping the extents does; the assertion text below
  // says only that. The normal side is one point (0.560000), not a band.
  {
    bac::Params sided = omniParams();
    sided.limits.v_max = 0.6f;
    sided.limits.vy_max = 0.6f;
    sided.footprint.front = 0.70f;
    sided.footprint.rear = -0.30f;
    sided.footprint.width = 0.5f;
    sided.safety_margin.front = 0.2f;
    sided.safety_margin.rear = 0.2f;
    sided.safety_margin.side = 0.2f;
    sided.heading_gain = 0.0f;

    const auto crabLeftSpeed = [&](float obstacle_x) {
      std::vector<bac::Point2D> points, path;
      for (int k = -6; k <= 6; ++k)
      {
        points.push_back({ obstacle_x + 0.02f * static_cast<float>(k), 1.45f });
      }
      for (int i = 1; i <= 20; ++i)
      {
        path.push_back({ 0.0f, 0.25f * static_cast<float>(i) });
      }
      bac::BacCore core(sided);
      return core.process(points, path, bac::Twist2D(0.0f, 0.0f, 0.6f)).output.speed();
    };

    const float ahead = crabLeftSpeed(0.70f);
    const float behind = crabLeftSpeed(-0.70f);
    // 0.58 sits between 0.560000 and v_max = 0.600000 with 0.02 m/s either
    // side. It does NOT separate `ahead` from `behind` - see the sweep above -
    // so the two assertions say only what they measure.
    expect(ahead < 0.58f,
           "an obstacle at +0.70 m still governs a left crab rather than leaving it at "
           "v_max (" + std::to_string(ahead) + " m/s, expected below 0.58; the swapped "
           "extents give 0.600000, which is v_max)");
    expect(behind > ahead,
           "the two extents are used in the right ORDER: an obstacle at -0.70 m is "
           "governed less hard than one at +0.70 m (" +
               std::to_string(behind) + " behind against " + std::to_string(ahead) +
               " ahead; the swapped extents invert this to 0.561427 against 0.600000)");
  }
}

/// Nav2 asks for a goal POSE, not a goal position. A model that steers with yaw
/// cannot honour the orientation - its heading is whatever direction it had to
/// travel in - but a holonomic body can hold the requested orientation while
/// lateral velocity closes the remaining position error. Asserted against the
/// differential-drive reference in the same world, which ignores it.
void
testGoalOrientationIsHeld()
{
  bac_sim::World world;
  const bac_sim::PathSource path = bac_sim::gotoPointPath(4.0f, 2.0f);

  // Swept over the goal orientation rather than run at one. R15 M2 found the
  // single-orientation form did not separate: with the fade disabled, 33 of 90
  // grid cells still landed inside the tolerance. Over the SET, the worst case
  // does separate - 0.0603 correct against 0.7701 broken.
  float error = 0.0f;
  float diff_error = 1e9f;
  float worst_yaw = 0.0f;
  float worst_position = 0.0f;
  for (const float goal_yaw : { 0.0f, -1.2f, 1.5f, 2.5f, -2.8f, 3.0f })
  {
    bac::BacCore reference(diffDriveReferenceParams());
    const OmniRun diff_run = runOmni(reference, world, { 0.0f, 0.0f, 0.0f }, path, 4.0f, 2.0f,
                                     90.0f, LateralWindow{}, goal_yaw);
    bac::BacCore core(omniParams());
    const OmniRun run = runOmni(core, world, { 0.0f, 0.0f, 0.0f }, path, 4.0f, 2.0f, 90.0f,
                                LateralWindow{}, goal_yaw);
    worst_position = std::max(worst_position, run.min_goal_distance);
    expectLimitsRespected(run, "goal orientation " + std::to_string(goal_yaw));
    const float e = std::fabs(wrapAngle(run.final_pose.th - goal_yaw));
    if (e > error)
    {
      error = e;
      worst_yaw = goal_yaw;
    }
    diff_error = std::min(diff_error, std::fabs(wrapAngle(diff_run.final_pose.th - goal_yaw)));
  }
  (void)worst_yaw;

  // Position, stated as the measured distance rather than the harness's
  // reached_goal flag. The path source stops emitting within 0.3 m of the goal,
  // so 0.27 m is the floor; holding the most extreme orientation (-2.8 rad)
  // costs a little on top of it. Measured over the set: 0.2876-0.3706 m.
  expect(worst_position < 0.40f,
         "holding the commanded orientation does not cost the goal position (worst "
         "closest approach " + std::to_string(worst_position) + " m)");
  // The bound is Nav2's own SimpleGoalChecker default yaw_goal_tolerance of
  // 0.25 rad - the tolerance the goal actually has to pass - not a number
  // picked here. Measured over goal orientations 0.0, -1.2, 1.5, 2.5, -2.8 and
  // 3.0 rad at heading_gain 1.5, the holonomic error spans 0.0065-0.0603 rad
  // while the differential-drive reference spans 0.5406-2.9426 rad: it ends at
  // whatever heading its approach left it with, whatever was asked for.
  expect(error < 0.25f,
         "the holonomic vehicle arrives within Nav2's default yaw_goal_tolerance of the "
         "requested orientation (error " + std::to_string(error) + " rad)");
  expect(diff_error > error,
         "the differential-drive reference does not hold it at any of them, so the check "
         "above is discriminating (its best error " + std::to_string(diff_error) + " rad)");
}

/// heading_gain = 0 holds the heading fixed. A platform with 360 degree
/// sensing may prefer that; it must still reach the goal by translating.
void
testZeroGainHoldsHeadingAndStillArrives()
{
  bac_sim::World world;
  bac::Params params = omniParams();
  params.heading_gain = 0.0f;
  bac::BacCore core(params);
  const OmniRun run = runOmni(core, world, { 0.0f, 0.0f, 0.0f },
                              bac_sim::gotoPointPath(3.0f, 3.0f), 3.0f, 3.0f, 90.0f);

  expect(run.reached_goal,
         "a heading-locked holonomic vehicle still reaches an off-axis goal (closest " +
             std::to_string(run.min_goal_distance) + " m)");
  expect(std::fabs(run.final_pose.th) < 0.05f,
         "heading_gain = 0 holds the heading fixed (final heading " +
             std::to_string(run.final_pose.th) + " rad)");
  expect(run.max_abs_vy > 0.05f,
         "it arrives by translating sideways (max |vy| " + std::to_string(run.max_abs_vy) + ")");
  expectLimitsRespected(run, "heading locked");
}

/// A corridor barely wider than the body. This is where pointing INTO the gap
/// beats crabbing towards it: a crabbing rectangle sweeps wider than a straight
/// one, so at this width a purely lateral correction does not fit.
///
/// Swept over the ENTRY OFFSET rather than run at one. R15 M3 found that a
/// single offset made the kill for the centering term a tripwire on one grid
/// cell, and R15 H4 found a body contact at offset 0.40 that the single-offset
/// fixture could not see. The grid is recorded here so a later round can tell
/// what was and was not covered:
///   entry offset 0.30, 0.36, 0.40 m x corridor width 1.2 m
/// Measured after the R15 fixes, holonomic mean lateral error over that grid is
/// 0.0126-0.0151 m with zero contacts, against 0.0291-0.0361 m for the
/// differential-drive reference. Removing the centering bias fails every cell,
/// not just one.
///
/// The grid is swept rather than sampled at three points. R16 H1 found that the
/// R15 fix had only been checked on the offsets it happened to try: a 0.002 m
/// sweep of the same fixture reproduced contact between them. After the R16
/// fixes to the deadband and the speed governor, that sweep is clean - 61 cells
/// from 0.30 to 0.42 m, zero contacts, every one traversed - with walls of
/// either 0.05, 0.10 or 0.20 m thickness and with zero-thickness ones.
///
/// R17 H4 found those recorded limits stale after the R16 fixes. Re-measured
/// over 0.30-0.50 m in 0.002 m steps, counting a contact, a failure to traverse
/// or a route around the outside as a failure: this 1.2 m corridor with 0.10 m
/// walls passes all 101 cells, and a 1.1 m one starts failing at 0.418 m.
/// Zero-thickness walls are worse - 0.476 m and 0.372 m respectively -
/// which is why the fixture has thickness.
void
testNarrowCorridorCentering()
{
  bac_sim::World world;
  // Walls with thickness: see addCorridorXWalls for why a corridor driven
  // ALONG must not be modelled as zero-thickness lines.
  world.addCorridorXWalls(2.0f, 9.5f, 0.0f, 1.2f, 0.10f);
  LateralWindow window;
  window.enabled = true;
  window.center_y = 0.0f;
  window.x_from = 3.0f;
  window.x_to = 9.0f;
  // Inside the corridor at all: half-width 0.60 less the body half-width 0.25.
  // Without this a run that goes AROUND the corridor satisfies both the
  // traverse and the sample-count checks (R15 M6).
  constexpr float kInsideCorridor = 0.35f;

  for (const float offset : { 0.30f, 0.36f, 0.40f })
  {
    const std::string at = "entry offset " + std::to_string(offset) + " m";
    bac::BacCore reference(diffDriveReferenceParams());
    const OmniRun diff_run =
        runOmni(reference, world, { 0.0f, offset, 0.0f },
                bac_sim::gotoPointPath(10.5f, 0.0f), 10.5f, 0.0f, 120.0f, window);
    bac::BacCore core(omniParams());
    const OmniRun run = runOmni(core, world, { 0.0f, offset, 0.0f },
                                bac_sim::gotoPointPath(10.5f, 0.0f), 10.5f, 0.0f, 120.0f, window);

    expect(!run.collided, at + ": corridor centering has no body contact");
    expect(run.lateral_samples > 0,
           at + ": the vehicle reached the measurement window (" +
               std::to_string(run.lateral_samples) + " samples)");
    expect(run.max_abs_lateral < kInsideCorridor,
           at + ": the vehicle went THROUGH the corridor rather than around it (max |y| " +
               std::to_string(run.max_abs_lateral) + " m)");
    expect(run.final_pose.x > 9.0f,
           at + ": the holonomic vehicle traverses a corridor 1.2 m wide (final x " +
               std::to_string(run.final_pose.x) + ")");
    expect(diff_run.final_pose.x > 9.0f,
           at + ": the differential-drive reference traverses it too, so the comparison "
                "below is between two solutions (final x " +
               std::to_string(diff_run.final_pose.x) + ")");
    // A comparison, not a threshold: the two bands measured over the grid above
    // do not touch (0.0126-0.0151 against 0.0291-0.0361), measured on this
    // fixture with 0.10 m thick walls.
    expect(run.mean_abs_lateral < diff_run.mean_abs_lateral,
           at + ": the holonomic vehicle holds the centerline better than the "
                "differential-drive reference (mean |y| " +
               std::to_string(run.mean_abs_lateral) + " vs " +
               std::to_string(diff_run.mean_abs_lateral) + " m)");
    expectLimitsRespected(run, "corridor at " + at);
  }
}

/// An obstacle inside the safety polygon. The correct behaviour is to hold
/// position; the assertions are the facts that are observable in a full
/// standstill, not an invariant that a standstill can never violate.
void
testSafetyStopHoldsPosition()
{
  bac_sim::World world;
  world.addWall(0.5f, -1.0f, 0.5f, 1.0f);  // inside the 0.55 m front safety edge
  bac::BacCore core(omniParams());
  const OmniRun run = runOmni(core, world, { 0.0f, 0.0f, 0.0f },
                              bac_sim::gotoPointPath(10.0f, 0.0f), 10.0f, 0.0f, 8.0f);

  expect(!run.collided, "the safety stop does not touch the wall (min clearance " +
                            std::to_string(run.min_clearance) + " m)");
  expect(run.final_pose.x < 0.05f, "the vehicle never advances towards the wall (final x " +
                                       std::to_string(run.final_pose.x) + ")");
  expect(run.travelled_distance < 0.15f,
         "the vehicle holds position (travelled " + std::to_string(run.travelled_distance) + " m)");
  expect(run.stop_ticks > run.total_ticks / 2,
         "most ticks command a standstill (" + std::to_string(run.stop_ticks) + " of " +
             std::to_string(run.total_ticks) + ")");
  expectLimitsRespected(run, "safety stop");
}

/// The shipped yaml, checked as a FILE.
///
/// This function used to drive the shipped configuration through two worlds as
/// well. It no longer does, because `omniParams()` now IS the shipped
/// configuration (see there), so every scenario in this file drives it and the
/// two worlds were already among them:
///
///   - the detour world (`addBox(3, 0, 0.8, 0.8)`, `gotoPointPath(6, 0)`, 60 s)
///     was bit-identical to the holonomic run of `testSidestepsInsteadOfYawing`;
///   - the corridor (`addCorridorXWalls(2, 9.5, 0, 1.2, 0.10)`, entry y = 0.30,
///     `gotoPointPath(10.5, 0)`, 120 s, window x = 3..9) was bit-identical to the
///     `offset = 0.30` cell of `testNarrowCorridorCentering`.
///
/// Bit-identical is measured, not inferred: the whole tick sequence of each pair
/// - commanded (v, vy, w) and integrated pose at every one of the 1200 / 2400
/// ticks - hashes to the same value, and every OmniRun field agrees.
///
/// Fourteen assertion signatures went away here and two arrived - counted
/// across the whole suite, which went from 457 distinct signatures to 445 at
/// this change. The fourteen are seven driving assertions (the table below),
/// six from the two `expectLimitsRespected()` calls that belonged to the
/// deleted runs (that function asserts three things, and it was called twice),
/// and one that was REWORDED rather than dropped, "every value the scenario
/// needs" becoming "every value the scenarios need". Inside THIS binary:
/// 150 executed assertions became 126, of which the `expectLimitsRespected()`
/// contribution went from 51 to 45.
///
/// What the seven retired driving assertions were worth, measured over the 52
/// mutations of `src/` and `bac_core.hpp` this suite is scored against - all
/// seven, in the two-per-row form the second corridor pair takes:
///
///   "no body contact" (detour)          kills 0; kept as "the holonomic detour
///                                       has no body contact"
///   "reaches its goal"                  kills 3 (emerg_normalized_lat_removed,
///                                       geo_cx_sign, omni_project_no_vy); all
///                                       three also die on "the holonomic
///                                       vehicle rounds a blocking obstacle"
///   "grants usable lateral authority"   kills 0; the identical predicate
///                                       (max |vy| > 0.05) is asserted in
///                                       testSidestepsInsteadOfYawing
///   "no body contact in a corridor"     kills 0; kept per entry offset there
///   "traverses a 1.2 m corridor" and    kill 2 (emerg_normalized_lat_removed,
///   "reached the measurement window"    geo_cx_sign); both also die on the
///                                       entry-offset 0.30 counterparts
///   "holds the centerline" (< 0.05 m)   kills 0. This was the one predicate
///                                       with no counterpart -
///                                       testNarrowCorridorCentering compares
///                                       against the differential-drive run
///                                       instead of against a fixed bound - and
///                                       it separates no mutation in this set.
///                                       It was NOT worthless: against a yaml
///                                       PERTURBATION, a different threat model
///                                       from a src/ mutation, it was the only
///                                       thing catching the one the file's own
///                                       comment calls out. See below.
///
/// So the kill set is unchanged, and the R15 M5 finding the two worlds answer
/// (with only the detour world, 13 of 16 value perturbations of the shipped yaml
/// passed, `avoid_margin.side: 0.9 -> 0.5` among them) is answered more
/// strongly than before. Measured, by editing that value in the shipped yaml
/// and counting the assertions that fail:
///
///   before   1  "and holds the centerline, which the shipped
///               avoid_margin.side is what buys (mean |y| 0.105089 m)"
///   after    2  "entry offset 0.300000 m: the holonomic vehicle holds the
///               centerline better than the differential-drive reference"
///               and "a heading error past the branch cut turns the short way
///               round (yaw 0.125000 rad/s)"
///
/// The retired absolute bound really was the sole detector of that perturbation
/// before; it is now caught twice, by two different scenarios, one of which
/// could not see a yaml edit at all while the parameters were hand-copied.
/// That is the whole point of the reversal: a perturbation now moves the 26
/// closed-loop runs of 12 scenarios, where it used to move two runs of this
/// one. (12 and 26 counted at the call sites; see omniParams for the other four
/// scenarios and why they do not participate.)
///
/// What is left here is what no scenario can express - that the file PARSES,
/// that every key in it is consumed by the reader or named in
/// kAllowedUnconsumedKeys, that no key is declared twice, that the values live
/// in the FollowPath block users copy, and that it selects the holonomic model.
/// `shippedExampleParams()` asserts all of that while reading; this asks for the
/// verdict.
void
testShippedExampleConfiguration()
{
  const bac::Params params = omniParams();
  expect(g_shipped_config_loaded,
         "every value the scenarios need was read from the shipped configuration");
  expect(params.motion_model.type == bac::MotionModelType::OMNI,
         "the parsed shipped configuration is the one the holonomic scenarios run");
}

}  // namespace

int
main()
{
  testStraightGoalReached();
  testSidestepsInsteadOfYawing();
  testRearGoalWithoutAligningFirst();
  testHeadingRegulatorTracksTheTangent();
  testEmittedCommandCanAlwaysStop();
  testEmittedCommandCanStopWhenAccelerationBinds();
  testStraightCommandsClearTheirOwnFootprint();
  testDeadbandIsSkippedRatherThanBreakingAdmissibility();
  testLateralRowIsContactChecked();
  testEmergencyLayerSeesLateralMotion();
  testStopMeansStoppedAndHeadingTakesTheShortWay();
  testGovernorTreatsSidewaysLikeForward();
  testGoalOrientationIsHeld();
  testZeroGainHoldsHeadingAndStillArrives();
  testNarrowCorridorCentering();
  testSafetyStopHoldsPosition();
  testShippedExampleConfiguration();

  if (failures != 0)
  {
    std::cerr << failures << " holonomic scenario check(s) failed\n";
    return 1;
  }
  std::cout << "All holonomic scenario checks passed\n";
  return 0;
}
