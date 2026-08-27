/**
 * @file bac_controller.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief nav2 controller plugin wrapping bac_core (DWA-based local planner)
 * @date 2026-08-27
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "bilateral_arc_clearance_controller/bac_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "bac_ros_parameters.hpp"
#include "nav2_core/controller_exceptions.hpp"
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

  auto declare_float = [&](const std::string &param_name, float default_value) {
    const std::string full_name = name + "." + param_name;
    if (!node->has_parameter(full_name))
    {
      node->declare_parameter<double>(full_name, static_cast<double>(default_value));
    }
    return static_cast<float>(node->get_parameter(full_name).as_double());
  };
  Params params = ros_parameters::declareCoreParameters(*node, name + ".");
  base_v_max_   = params.limits.v_max;

  // Direct laser input: the core is designed for raw scan points. When
  // scan_topic is set, fresh scans feed it and the costmap is only a fallback
  // (stale scan / no scan yet).
  if (!node->has_parameter(name + ".scan_topic"))
  {
    node->declare_parameter<std::string>(name + ".scan_topic", "");
  }
  scan_topic_   = node->get_parameter(name + ".scan_topic").as_string();
  scan_timeout_ = declare_float("scan_timeout", scan_timeout_);
  const std::string downsample_name = name + ".scan_downsample";
  if (!node->has_parameter(downsample_name))
  {
    node->declare_parameter<std::int64_t>(downsample_name, scan_downsample_);
  }
  scan_downsample_ = std::max(1, static_cast<int>(node->get_parameter(downsample_name).as_int()));
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
  float compensation         = declare_float("costmap_margin_compensation", default_compensation);
  params.safety_margin.front = std::max(0.05f, params.safety_margin.front - compensation);
  params.safety_margin.rear  = std::max(0.05f, params.safety_margin.rear - compensation);
  params.safety_margin.side  = std::max(0.05f, params.safety_margin.side - compensation);

  {
    const std::string pname = name + ".scan_min_points";
    if (!node->has_parameter(pname))
    {
      node->declare_parameter<std::int64_t>(pname, scan_min_points_);
    }
    scan_min_points_ = std::max(0, static_cast<int>(node->get_parameter(pname).as_int()));
  }
  {
    const std::string pname = name + ".scan_inf_is_valid";
    if (!node->has_parameter(pname))
    {
      node->declare_parameter<bool>(pname, scan_inf_is_valid_);
    }
    scan_inf_is_valid_ = node->get_parameter(pname).as_bool();
  }

  // Reject configurations that cannot produce meaningful candidates - a
  // silent nonsense config is worse than a failed configure.
  if (!(params.footprint.width > 0.0f) || !(params.footprint.front > params.footprint.rear))
  {
    throw std::invalid_argument("bac: footprint must have width > 0 and front > rear");
  }
  if (!(params.limits.v_max > 0.0f) || !(params.limits.w_max > 0.0f) ||
      !(params.limits.acc_v > 0.0f))
  {
    throw std::invalid_argument("bac: limits.v_max / w_max / acc_v must be positive");
  }
  if (params.safety_margin.front < 0.0f || params.safety_margin.rear < 0.0f ||
      params.safety_margin.side < 0.0f || params.avoid_margin.side < 0.0f)
  {
    throw std::invalid_argument("bac: margins must be non-negative");
  }
  if (!(params.sim_time > 0.0f) || !(params.stop_decel > 0.0f))
  {
    throw std::invalid_argument("bac: sim_time and stop_decel must be positive");
  }

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
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
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
  int valid_rays = 0;
  for (size_t i = 0; i < scan->ranges.size(); i += static_cast<size_t>(scan_downsample_))
  {
    float r = scan->ranges[i];
    // A +Inf (or beyond-range_max) return is a VALID "no obstacle in this
    // direction" measurement on many lidars (see the costmap ObstacleLayer's
    // inf_is_valid); it contributes no point but counts towards scan
    // validity. NaN and below-range_min returns are invalid.
    const bool clear_ray =
        scan_inf_is_valid_ && ((std::isinf(r) && r > 0.0f) ||
                               (std::isfinite(r) && r > scan->range_max));
    const bool hit_ray = std::isfinite(r) && r >= scan->range_min && r <= scan->range_max;
    if (clear_ray || hit_ray)
    {
      ++valid_rays;
    }
    if (!hit_ray || r > max_range)
    {
      continue;
    }
    double angle = syaw + scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    points.emplace_back(static_cast<float>(sx + r * std::cos(angle)),
                        static_cast<float>(sy + r * std::sin(angle)));
  }
  if (valid_rays < scan_min_points_)
  {
    // Too few valid MEASUREMENTS (hits or explicit no-returns): a sensor
    // fault, not an empty world - do not treat it as a valid observation.
    return std::nullopt;
  }
  return points;
}

std::vector<Point2D>
BacController::transformPlan(const geometry_msgs::msg::PoseStamped & /*pose*/) const
{
  // The plan lives in its own frame (typically map) while the controller runs
  // in the costmap/base frames - subtracting coordinates across frames would
  // silently cancel exactly the localization error this controller is
  // supposed to tolerate. Transform the plan into the BASE frame through TF
  // and treat any frame problem as a hard controller error.
  std::vector<Point2D> path;
  if (plan_.poses.empty())
  {
    return path;
  }
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  const std::string plan_frame = plan_.header.frame_id;
  if (plan_frame.empty())
  {
    throw nav2_core::ControllerTFError("plan has no frame_id");
  }
  geometry_msgs::msg::TransformStamped tf_msg;
  try
  {
    tf_msg = tf_->lookupTransform(base_frame, plan_frame, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    throw nav2_core::ControllerTFError(std::string("cannot transform plan: ") + ex.what());
  }
  const double tx  = tf_msg.transform.translation.x;
  const double ty  = tf_msg.transform.translation.y;
  const double yaw = tf2::getYaw(tf_msg.transform.rotation);
  const double cs = std::cos(yaw), sn = std::sin(yaw);

  std::vector<Point2D> base_pts;
  base_pts.reserve(plan_.poses.size());
  for (const auto &p : plan_.poses)
  {
    const double px = p.pose.position.x, py = p.pose.position.y;
    base_pts.emplace_back(static_cast<float>(tx + cs * px - sn * py),
                          static_cast<float>(ty + sn * px + cs * py));
  }

  // Prune from the pose NEAREST the robot (a stale or looping plan may start
  // far away - breaking on the first far point there would return an empty
  // path), then keep the local window up to the sensing range.
  size_t nearest = 0;
  float  best_d2 = std::numeric_limits<float>::max();
  for (size_t i = 0; i < base_pts.size(); ++i)
  {
    const float d2 = base_pts[i].x * base_pts[i].x + base_pts[i].y * base_pts[i].y;
    if (d2 < best_d2)
    {
      best_d2 = d2;
      nearest = i;
    }
  }
  const float max_range = core_.params().max_range;
  for (size_t i = nearest; i < base_pts.size(); ++i)
  {
    const float d2 = base_pts[i].x * base_pts[i].x + base_pts[i].y * base_pts[i].y;
    if (d2 > max_range * max_range)
    {
      break;
    }
    path.push_back(base_pts[i]);
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
