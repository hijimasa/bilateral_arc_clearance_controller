/**
 * @file bilateral_arc_clearance_controller.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief nav2 controller plugin wrapping bac_core (UNTESTED SKELETON)
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 REACT Co., Ltd.
 */

#include "bilateral_arc_clearance_controller/bilateral_arc_clearance_controller.hpp"

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
  params.reaction_time       = declareF("reaction_time", params.reaction_time);
  params.stop_decel          = declareF("stop_decel", params.stop_decel);
  params.w_range             = declareF("w_range", params.w_range);
  params.max_range           = declareF("max_range", params.max_range);
  core_.setParams(params);

  lookahead_     = declareF("lookahead", lookahead_);
  desired_speed_ = declareF("desired_speed", desired_speed_);
  k_heading_     = declareF("k_heading", k_heading_);
  w_max_         = declareF("w_max", w_max_);
}

void
BacController::cleanup()
{
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

Twist2D
BacController::planCommand(const geometry_msgs::msg::PoseStamped &pose) const
{
  if (plan_.poses.empty())
  {
    return Twist2D(0.0f, 0.0f);
  }

  double rx  = pose.pose.position.x;
  double ry  = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);

  // Lookahead target: first plan pose at least `lookahead_` away, else the last
  const auto *target = &plan_.poses.back();
  for (const auto &p : plan_.poses)
  {
    double dx = p.pose.position.x - rx, dy = p.pose.position.y - ry;
    if (std::sqrt(dx * dx + dy * dy) >= lookahead_)
    {
      target = &p;
      break;
    }
  }

  double dx            = target->pose.position.x - rx;
  double dy            = target->pose.position.y - ry;
  double heading_error = std::atan2(dy, dx) - yaw;
  while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  float v = desired_speed_;
  if (speed_limit_ > 0.0f && v > speed_limit_)
  {
    v = speed_limit_;
  }
  float w = std::max(-w_max_, std::min(w_max_, static_cast<float>(k_heading_ * heading_error)));
  return Twist2D(v, w);
}

geometry_msgs::msg::TwistStamped
BacController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                            const geometry_msgs::msg::Twist &velocity,
                                            nav2_core::GoalChecker * /*goal_checker*/)
{
  std::vector<Point2D> points  = collectObstaclePoints(pose);
  Twist2D              command = planCommand(pose);
  Twist2D current(static_cast<float>(velocity.linear.x), static_cast<float>(velocity.angular.z));

  Result result = core_.process(points, command, current);
  Twist2D applied = (result.status == Status::CLEAR) ? command : result.output;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.twist.linear.x  = applied.v;
  cmd.twist.angular.z = applied.w;
  return cmd;
}

void
BacController::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
  speed_limit_ = percentage ? static_cast<float>(desired_speed_ * speed_limit / 100.0) : static_cast<float>(speed_limit);
}

}  // namespace bac

PLUGINLIB_EXPORT_CLASS(bac::BacController, nav2_core::Controller)
