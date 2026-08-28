/**
 * @file bac_filter_node.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief ROS 2 evaluation node for bac_core: cmd_vel filter driven by a laser scan
 * @date 2026-08-26
 * @copyright Copyright (c) 2026 Masaaki Hijikata
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

#include "bilateral_arc_clearance_controller/adapter_utils.hpp"
#include "bilateral_arc_clearance_controller/bac_core.hpp"
#include "bac_ros_parameters.hpp"
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
    bac::Params params = bac::ros_parameters::declareCoreParameters(*this);
    core_.setParams(params);
    base_v_max_          = params.limits.v_max;
    virtual_path_length_ = declareFloat("virtual_path_length", 3.0f);

    // 2D pose of the laser in the robot frame
    sensor_x_   = declareFloat("sensor.x", 0.0f);
    sensor_y_   = declareFloat("sensor.y", 0.0f);
    sensor_yaw_ = declareFloat("sensor.yaw", 0.0f);

    scan_timeout_ = declareFloat("scan_timeout", 0.5f);
    cmd_timeout_  = declareFloat("cmd_timeout", 0.5f);
    odom_timeout_ = declareFloat("odom_timeout", 0.5f);
    scan_min_points_   = std::max<int64_t>(0, declare_parameter<int64_t>("scan_min_points", 10));
    scan_inf_is_valid_ = declare_parameter<bool>("scan_inf_is_valid", true);

    cmd_pub_    = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
    status_pub_ = create_publisher<std_msgs::msg::Int8>("avoid_status", 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_in", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          command_ = bac::Twist2D(static_cast<float>(msg->linear.x), static_cast<float>(msg->angular.z));
          last_cmd_time_ = now();
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          current_ = bac::Twist2D(static_cast<float>(msg->twist.twist.linear.x),
                                        static_cast<float>(msg->twist.twist.angular.z));
          last_odom_time_ = now();
        });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          bac::ScanProjection projected =
              bac::projectScan(msg->ranges, msg->angle_min, msg->angle_increment,
                               msg->range_min, msg->range_max, core_.params().max_range,
                               1, scan_inf_is_valid_, sensor_x_, sensor_y_, sensor_yaw_);
          points_ = std::move(projected.points);
          valid_rays_ = static_cast<int>(projected.valid_ray_count);
          last_scan_time_ = now();
        });

    timer_ = create_wall_timer(50ms, [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "bac_filter running (20Hz)");
  }

private:
  float declareFloat(const std::string &name, float default_value)
  {
    return static_cast<float>(declare_parameter<double>(name, static_cast<double>(default_value)));
  }

  void tick()
  {
    bac::Twist2D command, current;
    std::vector<bac::Point2D> points;
    bool scan_fresh, cmd_fresh, odom_fresh;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command    = command_;
      current    = current_;
      points     = points_;
      // A scan that is fresh by timestamp but carries almost no valid
      // measurements (hits or explicit no-returns) is a sensor fault, not
      // an empty world.
      scan_fresh = last_scan_time_.nanoseconds() > 0 &&
                   (now() - last_scan_time_).seconds() < static_cast<double>(scan_timeout_) &&
                   valid_rays_ >= scan_min_points_;
      cmd_fresh  = last_cmd_time_.nanoseconds() > 0 &&
                   (now() - last_cmd_time_).seconds() < static_cast<double>(cmd_timeout_);
      odom_fresh = last_odom_time_.nanoseconds() > 0 &&
                   (now() - last_odom_time_).seconds() < static_cast<double>(odom_timeout_);
    }

    geometry_msgs::msg::Twist out;
    std_msgs::msg::Int8       status;

    if (!scan_fresh || !odom_fresh)
    {
      // No valid observation, or no velocity feedback to compute braking
      // distances from: stop rather than act on stale state.
      core_.forceStop();
      status.data = static_cast<int8_t>(bac::Status::STOP);
      // out stays zero
    }
    else if (!cmd_fresh)
    {
      // Upstream went silent: do not keep executing its last command.
      status.data = static_cast<int8_t>(core_.status());
      // out stays zero
    }
    else
    {
      // The upper (v, w) command becomes a virtual local path: the commanded
      // arc extended over virtual_path_length. The core then plans towards it
      // (and never faster than the commanded speed).
      std::vector<bac::Point2D> path;
      float cv = command.v, cw = command.w;
      if (std::fabs(cv) > 0.02f || std::fabs(cw) > 0.02f)
      {
        float v_dir = (cv >= 0.0f) ? 1.0f : -1.0f;
        float v_eff = std::max(std::fabs(cv), 0.15f) * v_dir;  // pure turns get a short arc
        int   n     = 30;
        for (int i = 1; i <= n; i++)
        {
          float s = virtual_path_length_ * static_cast<float>(i) / static_cast<float>(n) * v_dir;
          if (std::fabs(cw) < 1e-3f)
          {
            path.emplace_back(s, 0.0f);
          }
          else
          {
            float radius = v_eff / cw;
            float theta  = s / radius;
            path.emplace_back(radius * std::sin(theta), radius * (1.0f - std::cos(theta)));
          }
        }
      }

      bac::Params params = core_.params();
      float v_cap = std::min(base_v_max_, std::fabs(cv));
      if (params.limits.v_max != v_cap)
      {
        params.limits.v_max = v_cap;
        core_.setParams(params);
      }

      bac::Result result = core_.process(points, path, current);
      // Arbitration: transparent while CLEAR - except the angular rate
      // limit, which is part of the reachability contract and therefore
      // applies to the passed-through command as well.
      bac::Twist2D applied = (result.status == bac::Status::CLEAR) ? command : result.output;
      const bac::Params &p = core_.params();
      if (result.status == bac::Status::CLEAR && p.limits.acc_w > 1e-3f &&
          p.control_period > 1e-4f)
      {
        const float dw = p.limits.acc_w * p.control_period;
        applied.w = std::min(std::max(applied.w, current.w - dw), current.w + dw);
      }
      out.linear.x  = applied.v;
      out.angular.z = applied.w;
      status.data   = static_cast<int8_t>(result.status);
    }

    cmd_pub_->publish(out);
    status_pub_->publish(status);
  }

  bac::BacCore core_;

  std::mutex             mutex_;
  bac::Twist2D           command_;
  bac::Twist2D           current_;
  std::vector<bac::Point2D> points_;
  rclcpp::Time                 last_scan_time_{ 0, 0, RCL_ROS_TIME };

  float sensor_x_ = 0.0f, sensor_y_ = 0.0f, sensor_yaw_ = 0.0f;
  float scan_timeout_ = 0.5f;
  float cmd_timeout_  = 0.5f;
  float odom_timeout_ = 0.5f;
  int   scan_min_points_ = 10;
  bool  scan_inf_is_valid_ = true;
  int   valid_rays_ = 0;
  rclcpp::Time last_cmd_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_odom_time_{ 0, 0, RCL_ROS_TIME };
  float base_v_max_ = 0.4f;
  float virtual_path_length_ = 3.0f;

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
