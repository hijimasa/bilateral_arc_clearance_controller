/**
 * @file bilateral_arc_clearance_controller.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief nav2 controller plugin wrapping bac_core (UNTESTED SKELETON)
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 REACT Co., Ltd.
 *
 * WARNING: written against the nav2 Humble API but never compiled or run
 * (nav2 is not installed in this workspace). Treat as a starting point:
 * a simple pure-pursuit generates the intent command from the plan, and
 * bac_core shapes it against lethal costmap cells.
 */

#pragma once
#ifndef BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CONTROLLER_HPP_
#define BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace bac
{

class BacController : public nav2_core::Controller
{
public:
  BacController()           = default;
  ~BacController() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path &path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                                           const geometry_msgs::msg::Twist &velocity,
                                                           nav2_core::GoalChecker *goal_checker) override;

  void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

private:
  /// Lethal costmap cells within max_range, in the robot frame
  std::vector<Point2D> collectObstaclePoints(const geometry_msgs::msg::PoseStamped &pose) const;

  /// Pure-pursuit style intent command from the current plan
  Twist2D planCommand(const geometry_msgs::msg::PoseStamped &pose) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr       parent_;
  std::string                                    name_;
  std::shared_ptr<tf2_ros::Buffer>               tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path                            plan_;

  BacCore core_;

  float lookahead_    = 0.8f;   // [m]
  float desired_speed_ = 0.4f;  // [m/s]
  float k_heading_    = 1.5f;   // [1/s]
  float w_max_        = 0.6f;   // [rad/s]
  float speed_limit_  = 0.0f;   // 0 = unlimited
};

}  // namespace bac

#endif  // BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CONTROLLER_HPP_
