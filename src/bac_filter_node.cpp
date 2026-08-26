/**
 * @file bac_filter_node.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief ROS 2 evaluation node for bac_core: cmd_vel filter driven by a laser scan
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 REACT Co., Ltd.
 *
 * Topics:
 *   in:  cmd_vel_in  (geometry_msgs/Twist)  upper-level velocity command
 *        scan        (sensor_msgs/LaserScan) obstacle points
 *        odom        (nav_msgs/Odometry)     current velocity feedback
 *   out: cmd_vel_out (geometry_msgs/Twist)   collision-shaped command
 *        avoid_status (std_msgs/Int8)        0=CLEAR 1=AVOIDING 2=STOP
 *
 * Runs at a fixed 20Hz. While status==CLEAR the input command is passed
 * through unmodified (the avoidance is transparent when nothing is in the
 * way). If the scan is stale the output is forced to zero.
 */

#include <chrono>
#include <cmath>
#include <mutex>
#include <vector>

#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int8.hpp"

using namespace std::chrono_literals;

class BacFilterNode : public rclcpp::Node
{
public:
  BacFilterNode()
    : rclcpp::Node("bac_filter")
  {
    bac::Params params;
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
    params.weights.clearance   = declareF("weights.clearance", params.weights.clearance);
    params.weights.fidelity    = declareF("weights.fidelity", params.weights.fidelity);
    params.weights.hysteresis  = declareF("weights.hysteresis", params.weights.hysteresis);
    params.weights.fidelity_viability_floor =
        declareF("weights.fidelity_viability_floor", params.weights.fidelity_viability_floor);
    core_.setParams(params);

    // 2D pose of the laser in the robot frame
    sensor_x_   = declareF("sensor.x", 0.0f);
    sensor_y_   = declareF("sensor.y", 0.0f);
    sensor_yaw_ = declareF("sensor.yaw", 0.0f);

    scan_timeout_ = declareF("scan_timeout", 0.5f);

    cmd_pub_    = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
    status_pub_ = create_publisher<std_msgs::msg::Int8>("avoid_status", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_in", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          command_ = bac::Twist2D(static_cast<float>(msg->linear.x), static_cast<float>(msg->angular.z));
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          current_ = bac::Twist2D(static_cast<float>(msg->twist.twist.linear.x),
                                        static_cast<float>(msg->twist.twist.angular.z));
        });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          points_.clear();
          points_.reserve(msg->ranges.size());
          float cs = std::cos(sensor_yaw_), sn = std::sin(sensor_yaw_);
          for (size_t i = 0; i < msg->ranges.size(); i++)
          {
            float r = msg->ranges[i];
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            {
              continue;
            }
            float a  = msg->angle_min + msg->angle_increment * static_cast<float>(i);
            float lx = r * std::cos(a), ly = r * std::sin(a);
            points_.emplace_back(sensor_x_ + cs * lx - sn * ly, sensor_y_ + sn * lx + cs * ly);
          }
          last_scan_time_ = now();
        });

    timer_ = create_wall_timer(50ms, [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "bac_filter running (20Hz)");
  }

private:
  float declareF(const std::string &name, float default_value)
  {
    return static_cast<float>(declare_parameter<double>(name, static_cast<double>(default_value)));
  }

  void tick()
  {
    bac::Twist2D           command, current;
    std::vector<bac::Point2D> points;
    bool scan_fresh;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command    = command_;
      current    = current_;
      points     = points_;
      scan_fresh = last_scan_time_.nanoseconds() > 0 &&
                   (now() - last_scan_time_).seconds() < static_cast<double>(scan_timeout_);
    }

    geometry_msgs::msg::Twist out;
    std_msgs::msg::Int8       status;

    if (!scan_fresh)
    {
      core_.forceStop();
      status.data = static_cast<int8_t>(bac::Status::STOP);
      // out stays zero
    }
    else
    {
      bac::Result result = core_.process(points, command, current);
      // Arbitration: transparent while CLEAR
      bac::Twist2D applied = (result.status == bac::Status::CLEAR) ? command : result.output;
      out.linear.x  = applied.v;
      out.angular.z = applied.w;
      status.data   = static_cast<int8_t>(result.status);
    }

    cmd_pub_->publish(out);
    status_pub_->publish(status);
  }

  bac::BacCore core_;

  std::mutex                   mutex_;
  bac::Twist2D           command_;
  bac::Twist2D           current_;
  std::vector<bac::Point2D> points_;
  rclcpp::Time                 last_scan_time_{ 0, 0, RCL_ROS_TIME };

  float sensor_x_ = 0.0f, sensor_y_ = 0.0f, sensor_yaw_ = 0.0f;
  float scan_timeout_ = 0.5f;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr          status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr                               timer_;
};

int
main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BacFilterNode>());
  rclcpp::shutdown();
  return 0;
}
