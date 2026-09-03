/**
 * @file ros_adapter_unit.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "bilateral_arc_clearance_controller/bac_controller.hpp"
#include "test_expect.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

namespace
{

using bac_test::expect;
using bac_test::failures;

std::string diagnosticValue(const diagnostic_msgs::msg::DiagnosticStatus &status,
                            const std::string &key)
{
  for (const auto &entry : status.values)
  {
    if (entry.key == key)
    {
      return entry.value;
    }
  }
  return {};
}

void spinFor(rclcpp::executors::SingleThreadedExecutor &executor,
             std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
}

nav_msgs::msg::Path straightPlan(const std::string &frame)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  for (int i = 0; i <= 10; ++i)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.x = 0.2 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path rearPlan(const std::string &frame)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  for (int i = 0; i <= 10; ++i)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.x = -0.2 * static_cast<double>(i);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}


void testControllerAdapter()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("FollowPath.scan_topic", "/scan"),
    rclcpp::Parameter("FollowPath.scan_timeout", 0.5),
    rclcpp::Parameter("FollowPath.scan_min_points", 3),
    rclcpp::Parameter("FollowPath.diagnostics_publish_period", 0.001),
    rclcpp::Parameter("FollowPath.limits.v_min", 0.0)
  });
  auto parent = std::make_shared<rclcpp_lifecycle::LifecycleNode>("bac_adapter_test", options);
  rclcpp::NodeOptions costmap_options;
  costmap_options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "odom"),
    rclcpp::Parameter("robot_base_frame", "base_link"),
    rclcpp::Parameter("rolling_window", true),
    rclcpp::Parameter("width", 4),
    rclcpp::Parameter("height", 4),
    rclcpp::Parameter("resolution", 0.1)
  });
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
  costmap->configure();
  auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());

  bac::BacController controller;
  controller.configure(parent, "FollowPath", tf, costmap);
  controller.activate();

  auto driver = std::make_shared<rclcpp::Node>("bac_adapter_test_driver");
  auto scan_pub = driver->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS());
  diagnostic_msgs::msg::DiagnosticStatus last_diagnostic;
  bool diagnostic_received = false;
  auto diagnostic_sub = driver->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        if (!message->status.empty())
        {
          last_diagnostic = message->status.front();
          diagnostic_received = true;
        }
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parent->get_node_base_interface());
  executor.add_node(driver);
  spinFor(executor, 100ms);

  const std::string base_frame = costmap->getBaseFrameID();
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame;
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist velocity;
  controller.setPlan(straightPlan(base_frame));

  // With no scan received, the configured raw source must visibly fall back.
  controller.computeVelocityCommands(robot_pose, velocity, nullptr);
  spinFor(executor, 50ms);
  expect(diagnostic_received, "controller publishes diagnostics while active");
  expect(last_diagnostic.level == diagnostic_msgs::msg::DiagnosticStatus::WARN,
         "configured raw scan fallback is WARN-level");
  expect(diagnosticValue(last_diagnostic, "obstacle_source") == "costmap_fallback",
         "diagnostics expose costmap fallback");
  expect(diagnosticValue(last_diagnostic, "scan_state") == "not_received",
         "diagnostics expose the fallback reason");

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = base_frame;
  scan.angle_min = -0.5f;
  scan.angle_increment = 0.1f;
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges.assign(11U, std::numeric_limits<float>::infinity());
  scan.header.stamp = driver->now();
  scan_pub->publish(scan);
  spinFor(executor, 100ms);

  diagnostic_received = false;
  const geometry_msgs::msg::TwistStamped command =
      controller.computeVelocityCommands(robot_pose, velocity, nullptr);
  spinFor(executor, 50ms);
  expect(std::isfinite(command.twist.linear.x) && std::isfinite(command.twist.angular.z),
         "fresh clear scan produces a finite command");
  expect(diagnostic_received, "fresh scan decision publishes diagnostics");
  expect(last_diagnostic.level == diagnostic_msgs::msg::DiagnosticStatus::OK,
         "fresh scan is OK-level");
  expect(diagnosticValue(last_diagnostic, "obstacle_source") == "raw_scan",
         "diagnostics expose raw scan use");

  // Half of the model-selection check. The DEFAULT configuration must still be
  // differential drive, which turns on the spot for a plan lying entirely
  // behind the vehicle. testAckermannAdapterConfiguration asserts the other
  // half, so a motion_model.type that silently resolved to the wrong policy
  // fails one of the two.
  controller.setPlan(rearPlan(base_frame));
  const geometry_msgs::msg::TwistStamped diff_rear_command =
      controller.computeVelocityCommands(robot_pose, velocity, nullptr);
  expect(std::fabs(diff_rear_command.twist.linear.x) <= 1e-4 &&
             std::fabs(diff_rear_command.twist.angular.z) > 1e-3,
         "the default Nav2 configuration is differential drive and turns on the spot");
  controller.setPlan(straightPlan(base_frame));

  // Nav2 speed limiting must cap the output selected from an otherwise clear scan.
  controller.setSpeedLimit(0.05, false);
  const geometry_msgs::msg::TwistStamped limited =
      controller.computeVelocityCommands(robot_pose, velocity, nullptr);
  expect(limited.twist.linear.x <= 0.05001,
         "absolute Nav2 speed limit caps the selected command");

  // A once-valid scan must become a visible fallback after its timeout.
  spinFor(executor, 550ms);
  diagnostic_received = false;
  controller.computeVelocityCommands(robot_pose, velocity, nullptr);
  spinFor(executor, 20ms);
  expect(diagnostic_received, "stale scan decision publishes diagnostics");
  expect(diagnosticValue(last_diagnostic, "obstacle_source") == "costmap_fallback" &&
             diagnosticValue(last_diagnostic, "scan_state") == "stale",
         "stale scan falls back with an explicit reason");

  // A fresh message with too few valid rays is a sensor fault, not empty space.
  scan.header.stamp = driver->now();
  scan.ranges.assign(11U, std::numeric_limits<float>::quiet_NaN());
  scan_pub->publish(scan);
  spinFor(executor, 50ms);
  diagnostic_received = false;
  controller.computeVelocityCommands(robot_pose, velocity, nullptr);
  spinFor(executor, 20ms);
  expect(diagnostic_received, "invalid scan decision publishes diagnostics");
  expect(diagnosticValue(last_diagnostic, "obstacle_source") == "costmap_fallback" &&
             diagnosticValue(last_diagnostic, "scan_state") == "insufficient_valid_rays",
         "invalid scan falls back with an explicit reason");

  nav_msgs::msg::Path unavailable_plan = straightPlan("unavailable_frame");
  controller.setPlan(unavailable_plan);
  bool threw_tf_error = false;
  try
  {
    controller.computeVelocityCommands(robot_pose, velocity, nullptr);
  }
  catch (const nav2_core::ControllerTFError &)
  {
    threw_tf_error = true;
  }
  expect(threw_tf_error, "unavailable plan transform raises ControllerTFError");

  controller.deactivate();
  controller.cleanup();
  costmap->cleanup();
  executor.remove_node(parent->get_node_base_interface());
  executor.remove_node(driver);
  (void)diagnostic_sub;
}

/// The Ackermann parameters must reach BacCore through the plugin boundary.
/// Runs on its own lifecycle and costmap nodes so it cannot disturb the
/// default-configuration coverage above.
void testAckermannAdapterConfiguration()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("FollowPath.scan_topic", "/scan"),
    rclcpp::Parameter("FollowPath.scan_timeout", 0.5),
    rclcpp::Parameter("FollowPath.scan_min_points", 3),
    rclcpp::Parameter("FollowPath.limits.v_min", 0.0),
    rclcpp::Parameter("FollowPath.motion_model.type", "ackermann"),
    rclcpp::Parameter("FollowPath.turn_radius_min", 0.8)
  });
  auto parent =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("bac_ackermann_adapter_test", options);
  rclcpp::NodeOptions costmap_options;
  costmap_options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "odom"),
    rclcpp::Parameter("robot_base_frame", "base_link"),
    rclcpp::Parameter("rolling_window", true),
    rclcpp::Parameter("width", 4),
    rclcpp::Parameter("height", 4),
    rclcpp::Parameter("resolution", 0.1)
  });
  // A distinct node name keeps this costmap's parameters and topics separate
  // from the default-configuration test's.
  costmap_options.arguments({ "--ros-args", "-r", "__node:=ackermann_costmap" });
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_options);
  costmap->configure();
  auto tf = std::make_shared<tf2_ros::Buffer>(parent->get_clock());

  bac::BacController controller;
  controller.configure(parent, "FollowPath", tf, costmap);
  controller.activate();

  const std::string base_frame = "base_link";
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame;
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist velocity;

  // The other half of the model-selection check: where differential drive
  // turns on the spot, a forward-only Ackermann vehicle must brake instead.
  controller.setPlan(rearPlan(base_frame));
  const geometry_msgs::msg::TwistStamped rear_command =
      controller.computeVelocityCommands(robot_pose, velocity, nullptr);
  expect(std::fabs(rear_command.twist.linear.x) <= 1e-4 &&
             std::fabs(rear_command.twist.angular.z) <= 1e-4,
         "forward-only Ackermann brakes for a rear plan instead of yawing");

  // Deliberately NOT asserted here: the minimum turning radius. In this
  // fixture the vehicle is at standstill and the selected arc is always
  // straight, so a radius assertion would be unreachable code that reads like
  // coverage it does not provide. The turning-radius bound - including the
  // binding case, the reverse case and the reachability-clamp interaction - is
  // covered by bac_ackermann_motion_model_unit and bac_ackermann_scenarios.

  controller.deactivate();
  controller.cleanup();
  costmap->cleanup();
}

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  testControllerAdapter();
  testAckermannAdapterConfiguration();
  rclcpp::shutdown();

  if (failures != 0)
  {
    std::cerr << failures << " ROS adapter check(s) failed\n";
    return 1;
  }
  std::cout << "All BAC ROS adapter checks passed\n";
  return 0;
}
