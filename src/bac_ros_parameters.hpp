/**
 * @file bac_ros_parameters.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#pragma once

#include <cstdint>
#include <string>

#include "bilateral_arc_clearance_controller/bac_core.hpp"

namespace bac
{
namespace ros_parameters
{

/**
 * Declare (once) and read every parameter shared by the ROS adapters.
 *
 * NodeT is either rclcpp::Node or rclcpp_lifecycle::LifecycleNode. Keeping
 * this list in one place prevents the filter and nav2 plugin from silently
 * exposing different subsets of Params.
 */
template<typename NodeT>
Params declareCoreParameters(NodeT &node, const std::string &prefix = "")
{
  auto declare_float = [&](const std::string &name, float default_value) {
    const std::string full_name = prefix + name;
    if (!node.has_parameter(full_name))
    {
      node.template declare_parameter<double>(full_name, static_cast<double>(default_value));
    }
    return static_cast<float>(node.get_parameter(full_name).as_double());
  };
  auto declare_int = [&](const std::string &name, int default_value) {
    const std::string full_name = prefix + name;
    if (!node.has_parameter(full_name))
    {
      node.template declare_parameter<std::int64_t>(full_name, default_value);
    }
    return static_cast<int>(node.get_parameter(full_name).as_int());
  };

  Params params;
  params.footprint.front     = declare_float("footprint.front", params.footprint.front);
  params.footprint.rear      = declare_float("footprint.rear", params.footprint.rear);
  params.footprint.width     = declare_float("footprint.width", params.footprint.width);
  params.safety_margin.front = declare_float("safety_margin.front", params.safety_margin.front);
  params.safety_margin.rear  = declare_float("safety_margin.rear", params.safety_margin.rear);
  params.safety_margin.side  = declare_float("safety_margin.side", params.safety_margin.side);
  params.avoid_margin.side   = declare_float("avoid_margin.side", params.avoid_margin.side);
  params.ignore_box.front    = declare_float("ignore_box.front", params.ignore_box.front);
  params.ignore_box.back     = declare_float("ignore_box.back", params.ignore_box.back);
  params.ignore_box.width    = declare_float("ignore_box.width", params.ignore_box.width);

  params.limits.v_max = declare_float("limits.v_max", params.limits.v_max);
  params.limits.v_min = declare_float("limits.v_min", params.limits.v_min);
  params.limits.w_max = declare_float("limits.w_max", params.limits.w_max);
  params.limits.acc_v = declare_float("limits.acc_v", params.limits.acc_v);

  params.weights.clearance  = declare_float("weights.clearance", params.weights.clearance);
  params.weights.goal_dist  = declare_float("weights.goal_dist", params.weights.goal_dist);
  params.weights.balance    = declare_float("weights.balance", params.weights.balance);
  params.weights.heading    = declare_float("weights.heading", params.weights.heading);
  params.weights.hysteresis = declare_float("weights.hysteresis", params.weights.hysteresis);
  params.weights.squeeze    = declare_float("weights.squeeze", params.weights.squeeze);

  params.sim_time            = declare_float("sim_time", params.sim_time);
  params.score_lookahead     = declare_float("score_lookahead", params.score_lookahead);
  {
    const std::string name = prefix + "station_goal";
    if (!node.has_parameter(name))
    {
      node.template declare_parameter<bool>(name, params.station_goal);
    }
    params.station_goal = node.get_parameter(name).as_bool();
  }
  params.station_lateral_weight =
      declare_float("station_lateral_weight", params.station_lateral_weight);
  params.goal_los_radius     = declare_float("goal_los_radius", params.goal_los_radius);
  params.cap_adapt_rate      = declare_float("cap_adapt_rate", params.cap_adapt_rate);
  params.los_onpath_radius   = declare_float("los_onpath_radius", params.los_onpath_radius);
  params.min_eval_distance   = declare_float("min_eval_distance", params.min_eval_distance);
  params.turn_radius_min     = declare_float("turn_radius_min", params.turn_radius_min);
  params.eval_angle_max      = declare_float("eval_angle_max", params.eval_angle_max);
  params.eval_lateral_max    = declare_float("eval_lateral_max", params.eval_lateral_max);
  params.blocked_near        = declare_float("blocked_near", params.blocked_near);
  params.blocked_far         = declare_float("blocked_far", params.blocked_far);
  params.margin_scale_floor  = declare_float("margin_scale_floor", params.margin_scale_floor);
  params.margin_scale_speed  = declare_float("margin_scale_speed", params.margin_scale_speed);
  params.window_time         = declare_float("window_time", params.window_time);
  params.v_samples           = declare_int("v_samples", params.v_samples);
  params.w_samples           = declare_int("w_samples", params.w_samples);
  params.w_refine_steps      = declare_int("w_refine_steps", params.w_refine_steps);
  params.stop_decel          = declare_float("stop_decel", params.stop_decel);
  params.brake_reaction_time = declare_float("brake_reaction_time", params.brake_reaction_time);
  params.max_range           = declare_float("max_range", params.max_range);
  params.max_points          = declare_int("max_points", params.max_points);
  params.velocity_min        = declare_float("velocity_min", params.velocity_min);
  params.angvel_min          = declare_float("angvel_min", params.angvel_min);
  params.creep_fraction        = declare_float("creep_fraction", params.creep_fraction);
  {
    const std::string name = prefix + "governor_arc_prediction";
    if (!node.has_parameter(name))
    {
      node.template declare_parameter<bool>(name, params.governor_arc_prediction);
    }
    params.governor_arc_prediction = node.get_parameter(name).as_bool();
  }
  params.tight_cruise_fraction = declare_float("tight_cruise_fraction", params.tight_cruise_fraction);
  params.side_envelope_headroom =
      declare_float("side_envelope_headroom", params.side_envelope_headroom);
  params.side_envelope_lookahead =
      declare_float("side_envelope_lookahead", params.side_envelope_lookahead);
  params.influence_range      = declare_float("influence_range", params.influence_range);
  params.avoiding_latch_ticks = declare_int("avoiding_latch_ticks", params.avoiding_latch_ticks);

  return params;
}

}  // namespace ros_parameters
}  // namespace bac
