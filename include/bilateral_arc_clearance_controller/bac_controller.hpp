/**
 * @file bac_controller.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief nav2 controller plugin wrapping bac_core (DWA-based local planner)
 * @date 2026-08-27
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Thin adapter: transforms the nav2 plan into the robot frame, feeds the core
 * obstacle points (raw laser scan when scan_topic is set; lethal costmap cells
 * as fallback), and publishes the core's (v, w) output. All planning logic
 * lives in the framework-free core.
 */

#pragma once
#ifndef BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CONTROLLER_HPP_
#define BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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

  /// Latest laser scan as robot-frame points (std::nullopt when no fresh scan)
  std::optional<std::vector<Point2D>> collectScanPoints();

  /// The current plan transformed into the robot frame
  std::vector<Point2D> transformPlan(const geometry_msgs::msg::PoseStamped &pose) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr       parent_;
  std::string                                    name_;
  std::shared_ptr<tf2_ros::Buffer>               tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path                            plan_;

  // Direct laser input (the core's native point source). When scan_topic is
  // set, fresh scans feed the core; the costmap is a fallback.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr                  latest_scan_;
  std::mutex                                                   scan_mutex_;
  rclcpp::Clock::SharedPtr                                     clock_;
  std::string                                                  scan_topic_;
  float                                                        scan_timeout_ = 0.5f;  // [s]
  int                                                          scan_downsample_ = 1;
  int                                                          scan_min_points_ = 10;
  bool                                                         scan_inf_is_valid_ = true;

  BacCore core_;

  float base_v_max_  = 0.4f;  // configured limits.v_max (speed limit re-caps it)
  float speed_limit_ = 0.0f;  // 0 = unlimited
};

}  // namespace bac

#endif  // BILATERAL_ARC_CLEARANCE_CONTROLLER__BAC_CONTROLLER_HPP_
