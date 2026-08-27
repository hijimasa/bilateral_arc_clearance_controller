/**
 * @file bac_controller.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief nav2 controller plugin wrapping bac_core (DWA-based local planner)
 * @date 2026-08-27
 * @copyright Copyright (c) 2026 REACT Co., Ltd.
 */

#include "bilateral_arc_clearance_controller/bac_controller.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace bac
{

void
BacController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                         std::shared_ptr<tf2_ros::Buffer> tf,
                         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_      = parent;
  name_        = name;
  tf_          = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();

  Params params;
  auto declareF = [&](const std::string &param_name, float default_value) {
    node->declare_parameter<double>(name + "." + param_name, static_cast<double>(default_value));
    return static_cast<float>(node->get_parameter(name + "." + param_name).as_double());
  };
  params.footprint.front     = declareF("footprint.front", params.footprint.front);
  params.footprint.rear      = declareF("footprint.rear", params.footprint.rear);
  params.footprint.width     = declareF("footprint.width", params.footprint.width);
  params.safety_margin.front = declareF("safety_margin.front", params.safety_margin.front);
  params.safety_margin.rear  = declareF("safety_margin.rear", params.safety_margin.rear);
  params.safety_margin.side  = declareF("safety_margin.side", params.safety_margin.side);
  params.avoid_margin.front  = declareF("avoid_margin.front", params.avoid_margin.front);
  params.avoid_margin.rear   = declareF("avoid_margin.rear", params.avoid_margin.rear);
  params.avoid_margin.side   = declareF("avoid_margin.side", params.avoid_margin.side);
  params.limits.v_max        = declareF("limits.v_max", params.limits.v_max);
  params.limits.v_min        = declareF("limits.v_min", params.limits.v_min);
  params.limits.w_max        = declareF("limits.w_max", params.limits.w_max);
  params.limits.acc_v        = declareF("limits.acc_v", params.limits.acc_v);
  params.limits.acc_w        = declareF("limits.acc_w", params.limits.acc_w);
  params.weights.clearance   = declareF("weights.clearance", params.weights.clearance);
  params.weights.goal_dist   = declareF("weights.goal_dist", params.weights.goal_dist);
  params.weights.heading     = declareF("weights.heading", params.weights.heading);
  params.weights.hysteresis  = declareF("weights.hysteresis", params.weights.hysteresis);
  params.weights.squeeze     = declareF("weights.squeeze", params.weights.squeeze);
  params.sim_time            = declareF("sim_time", params.sim_time);
  params.score_lookahead     = declareF("score_lookahead", params.score_lookahead);
  params.window_time         = declareF("window_time", params.window_time);
  params.v_samples           = static_cast<int>(declareF("v_samples", static_cast<float>(params.v_samples)));
  params.w_samples           = static_cast<int>(declareF("w_samples", static_cast<float>(params.w_samples)));
  params.min_eval_distance   = declareF("min_eval_distance", params.min_eval_distance);
  params.turn_radius_min     = declareF("turn_radius_min", params.turn_radius_min);
  params.eval_angle_max      = declareF("eval_angle_max", params.eval_angle_max);
  params.blocked_near        = declareF("blocked_near", params.blocked_near);
  params.blocked_far         = declareF("blocked_far", params.blocked_far);
  params.stop_decel          = declareF("stop_decel", params.stop_decel);
  params.brake_reaction_time = declareF("brake_reaction_time", params.brake_reaction_time);
  params.max_range           = declareF("max_range", params.max_range);
  params.creep_fraction      = declareF("creep_fraction", params.creep_fraction);
  params.proximity_governor_range = declareF("proximity_governor_range", params.proximity_governor_range);
  params.influence_range     = declareF("influence_range", params.influence_range);
  params.margin_scale_floor  = declareF("margin_scale_floor", params.margin_scale_floor);
  params.margin_scale_speed  = declareF("margin_scale_speed", params.margin_scale_speed);
  base_v_max_                = params.limits.v_max;

  // Direct laser input: the core is designed for raw scan points. When
  // scan_topic is set, fresh scans feed it and the costmap is only a fallback
  // (stale scan / no scan yet).
  node->declare_parameter<std::string>(name + ".scan_topic", "");
  scan_topic_   = node->get_parameter(name + ".scan_topic").as_string();
  scan_timeout_ = declareF("scan_timeout", scan_timeout_);
  clock_        = node->get_clock();
  if (!scan_topic_.empty())
  {
    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(scan_mutex_);
          latest_scan_ = msg;
        });
  }

  // Costmap-fed points are cell centers, which can sit up to half a cell
  // inside the true obstacle surface. Deduct that quantization error from the
  // safety margins so a wall seen through the costmap does not trigger the
  // emergency stop earlier than the real geometry would (0 disables the
  // compensation). With a direct scan feed the points are exact, so the
  // default is no compensation (the costmap is then only a stale-scan
  // fallback, where uncompensated margins err on the safe side).
  float default_compensation = 0.0f;
  if (scan_topic_.empty())
  {
    if (nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap())
    {
      default_compensation = static_cast<float>(costmap->getResolution()) / 2.0f;
    }
  }
  float compensation         = declareF("costmap_margin_compensation", default_compensation);
  params.safety_margin.front = std::max(0.05f, params.safety_margin.front - compensation);
  params.safety_margin.rear  = std::max(0.05f, params.safety_margin.rear - compensation);
  params.safety_margin.side  = std::max(0.05f, params.safety_margin.side - compensation);

  core_.setParams(params);
}

void
BacController::cleanup()
{
  scan_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_.reset();
  }
  core_.reset();
}

void
BacController::activate()
{
  core_.reset();
}

void
BacController::deactivate()
{
}

void
BacController::setPlan(const nav_msgs::msg::Path &path)
{
  plan_ = path;
}

std::vector<Point2D>
BacController::collectObstaclePoints(const geometry_msgs::msg::PoseStamped &pose) const
{
  std::vector<Point2D> points;

  nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  double rx = pose.pose.position.x;
  double ry = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);
  double cs = std::cos(-yaw), sn = std::sin(-yaw);
  double range = core_.params().max_range;

  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); my++)
  {
    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); mx++)
    {
      if (costmap->getCost(mx, my) < nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        continue;
      }
      double wx, wy;
      costmap->mapToWorld(mx, my, wx, wy);
      double dx = wx - rx, dy = wy - ry;
      if (dx * dx + dy * dy > range * range)
      {
        continue;
      }
      points.emplace_back(static_cast<float>(cs * dx - sn * dy), static_cast<float>(sn * dx + cs * dy));
    }
  }
  return points;
}

std::optional<std::vector<Point2D>>
BacController::collectScanPoints()
{
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = latest_scan_;
  }
  if (!scan)
  {
    return std::nullopt;
  }
  if ((clock_->now() - rclcpp::Time(scan->header.stamp)).seconds() > scan_timeout_)
  {
    return std::nullopt;
  }

  // Sensor pose in the robot base frame (2D); identity if the scan is already
  // in the base frame.
  double sx = 0.0, sy = 0.0, syaw = 0.0;
  const std::string &base_frame = costmap_ros_->getBaseFrameID();
  if (scan->header.frame_id != base_frame)
  {
    try
    {
      auto tf_msg = tf_->lookupTransform(base_frame, scan->header.frame_id, tf2::TimePointZero);
      sx   = tf_msg.transform.translation.x;
      sy   = tf_msg.transform.translation.y;
      syaw = tf2::getYaw(tf_msg.transform.rotation);
    }
    catch (const tf2::TransformException &)
    {
      return std::nullopt;  // fall back to costmap points
    }
  }

  float max_range = core_.params().max_range;
  std::vector<Point2D> points;
  points.reserve(scan->ranges.size());
  for (size_t i = 0; i < scan->ranges.size(); i++)
  {
    float r = scan->ranges[i];
    if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max || r > max_range)
    {
      continue;
    }
    double angle = syaw + scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    points.emplace_back(static_cast<float>(sx + r * std::cos(angle)),
                        static_cast<float>(sy + r * std::sin(angle)));
  }
  return points;
}

std::vector<Point2D>
BacController::transformPlan(const geometry_msgs::msg::PoseStamped &pose) const
{
  std::vector<Point2D> path;
  path.reserve(plan_.poses.size());
  double rx  = pose.pose.position.x;
  double ry  = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);
  double cs = std::cos(-yaw), sn = std::sin(-yaw);
  double max_range = core_.params().max_range;
  for (const auto &p : plan_.poses)
  {
    double dx = p.pose.position.x - rx;
    double dy = p.pose.position.y - ry;
    if (dx * dx + dy * dy > max_range * max_range)
    {
      break;  // beyond the sensing range: irrelevant for local planning
    }
    path.emplace_back(static_cast<float>(cs * dx - sn * dy), static_cast<float>(sn * dx + cs * dy));
  }
  return path;
}

geometry_msgs::msg::TwistStamped
BacController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                       const geometry_msgs::msg::Twist &velocity,
                                       nav2_core::GoalChecker * /*goal_checker*/)
{
  // Speed limit re-caps the sampled window
  Params params = core_.params();
  float  v_max  = base_v_max_;
  if (speed_limit_ > 0.0f)
  {
    v_max = std::min(v_max, speed_limit_);
  }
  if (params.limits.v_max != v_max)
  {
    params.limits.v_max = v_max;
    core_.setParams(params);
  }

  std::vector<Point2D> points;
  if (auto scan_points = collectScanPoints())
  {
    points = std::move(*scan_points);
  }
  else
  {
    points = collectObstaclePoints(pose);
  }
  std::vector<Point2D> path = transformPlan(pose);
  Twist2D current(static_cast<float>(velocity.linear.x), static_cast<float>(velocity.angular.z));

  Result result = core_.process(points, path, current);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.twist.linear.x  = result.output.v;
  cmd.twist.angular.z = result.output.w;
  return cmd;
}

void
BacController::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
  speed_limit_ = percentage ? static_cast<float>(base_v_max_ * speed_limit / 100.0) : static_cast<float>(speed_limit);
}

}  // namespace bac

PLUGINLIB_EXPORT_CLASS(bac::BacController, nav2_core::Controller)
