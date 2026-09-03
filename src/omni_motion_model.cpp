/**
 * @file omni_motion_model.cpp
 * @brief Internal holonomic candidate generation and rollout geometry
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "omni_motion_model.hpp"

#include <algorithm>
#include <cmath>

namespace bac::detail
{

namespace
{

/// Scale a velocity vector down to `v_max` without changing its direction, so
/// the speed cap is exact rather than per-axis. A per-axis cap would admit
/// hypot(v_max, vy_max), which is faster than the vehicle is allowed to move.
Twist2D
capSpeed(const Twist2D &command, float v_max)
{
  const float speed = command.speed();
  if (speed <= v_max || speed <= 1e-6f)
  {
    return command;
  }
  const float scale = v_max / speed;
  return { command.v * scale, command.w, command.vy * scale };
}

}  // namespace

OmniMotionModel::OmniMotionModel(const Params &params)
  : params_(params)
{
  // The invariants this class relies on are established by
  // validateMotionModelParams, which every construction path runs first.
}

CandidateBatch
OmniMotionModel::sampleCandidates(const Twist2D &current, float linear_speed_cap,
                                  float yaw_reference) const
{
  const float window_dv = params_.limits.acc_v * params_.window_time;
  float v_lo = std::max(params_.limits.v_min, current.v - window_dv);
  float v_hi = std::min(linear_speed_cap, current.v + window_dv);
  if (v_hi < v_lo)
  {
    v_lo = v_hi = std::max(params_.limits.v_min, v_hi);
  }

  std::vector<float> forward_speeds;
  const int v_samples = std::max(2, params_.v_samples);
  for (int i = 0; i < v_samples; ++i)
  {
    forward_speeds.push_back(v_lo + (v_hi - v_lo) * static_cast<float>(i) /
                                        static_cast<float>(v_samples - 1));
  }
  // The standstill row: a holonomic body may still be commanded to yaw onto
  // the tangent while it holds station.
  forward_speeds.push_back(0.0f);

  // Lateral authority is bounded by the configured cap AND by what one
  // acceleration window can reach from the measured lateral velocity.
  // The lateral authority is bounded by the configured cap, by one
  // acceleration window, AND by the proximity governor's cap. The last one
  // matters on its own: the norm cap below is widened to |limits.v_min| so a
  // reverse candidate survives the governor the way it does for differential
  // drive, and without this line that widening would hand the same licence to
  // the lateral axis - which has no reverse-escape rationale and is the
  // direction of largest swept width (R15 M1).
  const float lateral_cap =
      std::min(params_.limits.vy_max, std::fabs(linear_speed_cap));
  const float vy_reach_lo = current.vy - window_dv;
  const float vy_reach_hi = current.vy + window_dv;
  const float vy_lo = std::max(-lateral_cap, vy_reach_lo);
  const float vy_hi = std::min(lateral_cap, vy_reach_hi);

  std::vector<float> lateral_speeds;
  const int vy_samples = std::max(3, params_.vy_samples);
  if (vy_hi <= vy_lo)
  {
    lateral_speeds.push_back(std::max(-lateral_cap, std::min(lateral_cap, current.vy)));
  }
  else
  {
    for (int i = 0; i < vy_samples; ++i)
    {
      lateral_speeds.push_back(vy_lo + (vy_hi - vy_lo) * static_cast<float>(i) /
                                           static_cast<float>(vy_samples - 1));
    }
    if (vy_lo < 0.0f && vy_hi > 0.0f)
    {
      lateral_speeds.push_back(0.0f);  // always offer the un-crabbed candidate
    }
  }

  // The cap the caller passes is the proximity speed governor's, already
  // moderated for what is close by. It has to bound the velocity VECTOR, not
  // just the forward axis. Capping only the forward component leaves the
  // lateral samples at full authority, so the governor slows the vehicle
  // FORWARD while it still slides sideways at up to limits.vy_max - and the
  // sideways direction is the one whose swept width is largest. Measured in a
  // 1.1-1.3 m corridor: with the cap on the forward axis alone the vehicle
  // leaves the corridor mouth sideways and strands itself against the outer
  // wall at |y| ~ 1.2 m; with the cap on the vector it stops cleanly at the
  // mouth instead. (The set of crab ANGLES is the same either way - capSpeed
  // scales the vector and preserves its direction - what changes is how fast
  // the vehicle commits to one.)
  const float speed_cap = std::max(std::fabs(linear_speed_cap), std::fabs(params_.limits.v_min));

  CandidateBatch batch;
  batch.commands.reserve(forward_speeds.size() * lateral_speeds.size());
  for (float v : forward_speeds)
  {
    for (float vy : lateral_speeds)
    {
      batch.commands.push_back(capSpeed(Twist2D(v, yaw_reference, vy), speed_cap));
    }
  }
  return batch;
}

std::vector<Twist2D>
OmniMotionModel::refinementCandidates(const Twist2D &coarse_best) const
{
  std::vector<Twist2D> commands;
  if (params_.w_refine_steps <= 0 || coarse_best.speed() <= 1e-3f)
  {
    return commands;
  }

  // Refine the avoidance dimension at a finer pitch than the coarse lattice,
  // exactly as the differential-drive model refines yaw rate.
  const int vy_samples = std::max(3, params_.vy_samples);
  const float coarse_step = (2.0f * params_.limits.vy_max) /
                            static_cast<float>(std::max(vy_samples - 1, 1));
  commands.reserve(static_cast<std::size_t>(2 * params_.w_refine_steps));
  for (int i = 1; i <= params_.w_refine_steps; ++i)
  {
    const float dvy = coarse_step * static_cast<float>(i) /
                      static_cast<float>(params_.w_refine_steps + 1);
    for (float vy : { coarse_best.vy - dvy, coarse_best.vy + dvy })
    {
      if (vy >= -params_.limits.vy_max && vy <= params_.limits.vy_max)
      {
        // Refinement adjusts the direction of travel, never the speed: capping
        // to the coarse winner's own speed keeps the refined candidates inside
        // the envelope the governor already admitted.
        commands.push_back(
            capSpeed(Twist2D(coarse_best.v, coarse_best.w, vy), coarse_best.speed()));
      }
    }
  }
  return commands;
}

std::vector<Twist2D>
OmniMotionModel::clearanceProbeCommands(float linear_speed) const
{
  // Probes measure how much clearance the passage affords, so they sample the
  // directions the vehicle can actually take: straight ahead and crabbed to
  // either side. They carry no yaw, so the probe geometry is a straight slab.
  const float lateral = std::min(params_.limits.vy_max, 0.5f * std::fabs(linear_speed));
  return { { linear_speed, 0.0f, -lateral },
           { linear_speed, 0.0f, 0.0f },
           { linear_speed, 0.0f, lateral } };
}

ProjectedPose2D
OmniMotionModel::projectConstantCommand(const Twist2D &command, float duration) const
{
  ProjectedPose2D pose{ 0.0f, 0.0f, command.w * duration };
  if (std::fabs(command.w) < 1e-4f)
  {
    pose.x = command.v * duration;
    pose.y = command.vy * duration;
    return pose;
  }

  // The body orbits the centre of rotation c = (-vy/w, v/w): after rotating
  // by theta, the origin sits at c - R(theta) * c.
  const float cx = -command.vy / command.w;
  const float cy = command.v / command.w;
  const float cos_t = std::cos(pose.theta);
  const float sin_t = std::sin(pose.theta);
  pose.x = cx - (cx * cos_t - cy * sin_t);
  pose.y = cy - (cx * sin_t + cy * cos_t);
  return pose;
}

bool
OmniMotionModel::isCommandKinematicallyValid(const Twist2D &command) const
{
  return std::fabs(command.vy) <= params_.limits.vy_max + 1e-4f &&
         command.speed() <= params_.limits.v_max + 1e-4f &&
         std::fabs(command.w) <= params_.limits.w_max + 1e-4f;
}

bool
OmniMotionModel::acceptsGoalHeading() const
{
  // Yaw is a regulator here, not the steering input, so the body can hold
  // the goal orientation while lateral velocity closes the position error.
  return true;
}

bool
OmniMotionModel::supportsInPlaceRotation() const
{
  // A holonomic body can yaw while holding station, and the pose regulator
  // asks it to whenever translation is blocked but the heading is still off
  // the tangent. The sweep is checked like any other rotation.
  return true;
}

bool
OmniMotionModel::usesRotateBeforeTranslate() const
{
  // Rotating onto the tangent before moving buys a holonomic body nothing: it
  // can already translate in that direction. Forcing the manoeuvre would add
  // yaw motion, and yaw motion sweeps the footprint corners.
  return false;
}

bool
OmniMotionModel::isInPlaceRotationAdmissible(const std::vector<Point2D> &points) const
{
  // A full in-place rotation sweeps the disk of the circumscribed radius.
  // The same conservative test the differential-drive model uses - now the
  // same code, so the two cannot drift apart.
  return circumscribedDiskFree(params_.footprint, points);
}

float
OmniMotionModel::commandChange(const Twist2D &command, const Twist2D &previous) const
{
  // Hysteresis acts on the AVOIDANCE dimension, as it does for the other
  // models: yaw rate for differential drive, curvature for Ackermann, lateral
  // velocity here [score per m/s]. This value does NOT transfer between
  // models - the quantities have different units and different scales.
  return std::fabs(command.vy - previous.vy);
}

Twist2D
OmniMotionModel::limitReachableCommand(const Twist2D &current,
                                       const Twist2D &desired) const
{
  Twist2D limited = desired;
  if (params_.control_period > 1e-4f)
  {
    if (params_.limits.acc_w > 1e-3f)
    {
      const float dw = params_.limits.acc_w * params_.control_period;
      limited.w = std::min(std::max(desired.w, current.w - dw), current.w + dw);
    }
    if (params_.limits.acc_v > 1e-3f)
    {
      // Both translational axes are driven by the same wheels, so the same
      // acceleration bound applies to each.
      const float dv = params_.limits.acc_v * params_.control_period;
      limited.v = std::min(std::max(desired.v, current.v - dv), current.v + dv);
      limited.vy = std::min(std::max(desired.vy, current.vy - dv), current.vy + dv);
    }
  }
  limited.vy = std::min(std::max(limited.vy, -params_.limits.vy_max), params_.limits.vy_max);
  limited.w = std::min(std::max(limited.w, -params_.limits.w_max), params_.limits.w_max);
  return capSpeed(limited, params_.limits.v_max);
}

Twist2D
OmniMotionModel::withLinearSpeed(const Twist2D &command, float requested) const
{
  const float speed = command.speed();
  if (speed <= 1e-3f || std::fabs(requested) <= 1e-3f)
  {
    // No direction of travel to preserve: brake to a standstill rather than
    // inventing one.
    return { 0.0f, 0.0f, 0.0f };
  }
  // Scale the WHOLE twist, yaw included, so the trajectory that was checked
  // for contact keeps its geometry. Holding the yaw rate while slowing would
  // tighten the circle and silently replace the checked path - the same
  // failure the differential-drive model avoids by preserving curvature.
  const float scale = std::fabs(requested) / speed;
  return { command.v * scale, command.w * scale, command.vy * scale };
}

Twist2D
OmniMotionModel::applyCommandDeadband(const Twist2D &command) const
{
  Twist2D result = command;
  if (result.speed() < params_.velocity_min)
  {
    result.v = 0.0f;
    result.vy = 0.0f;
  }
  if (std::fabs(result.w) < params_.angvel_min)
  {
    result.w = 0.0f;
  }
  return result;
}

}  // namespace bac::detail
