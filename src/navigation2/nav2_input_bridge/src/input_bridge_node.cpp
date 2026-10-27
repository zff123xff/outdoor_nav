// Copyright 2026

#include "nav2_input_bridge/input_bridge_node.hpp"

#include <cmath>
#include <algorithm>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <rclcpp/qos.hpp>

namespace nav2_input_bridge
{

InputBridgeNode::InputBridgeNode(const rclcpp::NodeOptions & options)
: Node("nav2_input_bridge", options)
{
  // Topic and action defaults are RELATIVE so the per-robot namespace (set
  // via launch `namespace=robot_namespace`) prefixes them to
  // /{robot_namespace}/... Namespaced endpoints keep multi-robot setups
  // isolated on a shared DDS domain.
  //
  // ROS 2 mavros's local_position plugin publishes ~/odom (relative to the
  // mavros namespace), not ~/local_position/odom — the latter path is
  // ROS 1 legacy.
  local_odom_topic_ = declare_parameter("local_odom_topic", std::string("mavros/odom"));
  goal_input_topic_ = declare_parameter("goal_input_topic", std::string("gp_goal"));
  uav_goal_input_topic_ = declare_parameter("uav_goal_input_topic", std::string("uav/target_gps"));
  target_detected_topic_ = declare_parameter("target_detected_topic", std::string("target_detected"));
  goal_output_topic_ = declare_parameter("goal_output_topic", std::string("goal_pose"));
  goal_output_frame_ = declare_parameter("goal_output_frame", std::string("map"));
  require_goal_fix_ = declare_parameter("require_nav_sat_fix", true);
  goal_yaw_from_bearing_ = declare_parameter("goal_yaw_from_bearing", false);
  follow_path_action_topic_ = declare_parameter("follow_path_action_topic", std::string("follow_path"));

  const double ref_lat = declare_parameter("reference_latitude", 0.0);
  const double ref_lon = declare_parameter("reference_longitude", 0.0);
  const double ref_alt = declare_parameter("reference_altitude", 0.0);
  {
    std::lock_guard<std::mutex> lock(origin_mutex_);
    if (std::abs(ref_lat) > 90.0 || std::abs(ref_lon) > 180.0) {
      origin_valid_ = false;
      RCLCPP_ERROR(
        get_logger(),
        "Invalid ENU reference: latitude in [-90,90], longitude in [-180,180]");
    } else {
      origin_lat_ = ref_lat;
      origin_lon_ = ref_lon;
      origin_alt_ = ref_alt;
      origin_valid_ = true;
      RCLCPP_INFO(
        get_logger(),
        "ENU origin from parameters: lat=%.8f deg, lon=%.8f deg, alt=%.3f m",
        ref_lat, ref_lon, ref_alt);
    }
  }

  rclcpp::QoS odom_qos(10);
  odom_qos.best_effort();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    local_odom_topic_, odom_qos,
    std::bind(&InputBridgeNode::onOdom, this, std::placeholders::_1));

  rclcpp::QoS goal_qos(10);
  goal_qos.best_effort();
  goal_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    goal_input_topic_, goal_qos,
    std::bind(&InputBridgeNode::onGoalFix, this, std::placeholders::_1));

  uav_goal_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    uav_goal_input_topic_, goal_qos,
    std::bind(&InputBridgeNode::onUavGoalFix, this, std::placeholders::_1));

  target_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
    target_detected_topic_, 10,
    std::bind(&InputBridgeNode::onTargetDetected, this, std::placeholders::_1));

  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_output_topic_, 10);

  stop_planner_pub_ = create_publisher<std_msgs::msg::Bool>("stop_planner", 10);

  follow_path_action_client_ = rclcpp_action::create_client<FollowPath>(
    this,
    follow_path_action_topic_);

  // TF broadcaster for odom -> base_link. We broadcast from the mavros odom
  // we already subscribe to (the installed mavros C++ plugin could also do it
  // with local_position.tf.send=true, but doing it here is independent of
  // mavros build/version quirks — see header comment).
  //
  // Frame names must include the namespace prefix WITHOUT a leading slash:
  // get_namespace() returns "/robot_a", and the slash must be stripped so the
  // emitted frame_id is "robot_a/odom". tf2's Buffer strips a leading "/" when
  // RECEIVING (so nav still worked with "/robot_a/odom"), but the slash stays
  // in the raw /tf message — foxglove and other non-tf2 consumers treat
  // "robot_a/odom" and "/robot_a/odom" as DIFFERENT frames, splitting the TF
  // tree (static TF uses slash-less "robot_a/map") and breaking map->base_link.
  // Keep the frames slash-less here to match the static publishers.
  const std::string ns = this->get_namespace();
  const std::string prefix =
    (ns.empty() || ns == "/") ? "" : ns.substr(1);  // strip leading '/'
  tf_frame_id_ = prefix.empty() ? "odom" : prefix + "/odom";
  tf_child_frame_id_ = prefix.empty() ? "base_link" : prefix + "/base_link";
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(
    get_logger(),
    "TF broadcaster: %s -> %s (subscribed to %s)",
    tf_frame_id_.c_str(), tf_child_frame_id_.c_str(), local_odom_topic_.c_str());
}

void InputBridgeNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!tf_broadcaster_) {
    return;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = tf_frame_id_;
  t.child_frame_id = tf_child_frame_id_;
  t.transform.translation.x = msg->pose.pose.position.x;
  t.transform.translation.y = msg->pose.pose.position.y;
  // t.transform.translation.z = msg->pose.pose.position.z;
  t.transform.translation.z = 0.0;
  t.transform.rotation = msg->pose.pose.orientation;
  tf_broadcaster_->sendTransform(t);
}

void InputBridgeNode::llaToEnu(
  double lat_deg, double lon_deg, double alt_m,
  double lat0_deg, double lon0_deg, double alt0_m,
  double * east_m, double * north_m, double * up_m) const
{
  // Spherical tangent-plane projection (same algorithm used in PX4 MapProjection)
  // For 5km+ scenarios, switch to the ellipsoid algorithm (see commented block below)
  const double R = WGS84_R_;
  const double lat_r = lat_deg * M_PI / 180.0;
  const double lon_r = lon_deg * M_PI / 180.0;
  const double lat0_r = lat0_deg * M_PI / 180.0;
  const double lon0_r = lon0_deg * M_PI / 180.0;
  const double sin_lat = sin(lat_r);
  const double cos_lat = cos(lat_r);
  const double sin_lat0 = sin(lat0_r);
  const double cos_lat0 = cos(lat0_r);
  const double dlon = lon_r - lon0_r;
  const double cos_dlon = cos(dlon);

  double arg = sin_lat0 * sin_lat + cos_lat0 * cos_lat * cos_dlon;
  arg = std::min(1.0, std::max(-1.0, arg));
  const double c = acos(arg);
  double k = 1.0;
  if (fabs(c) > 1e-6) {
    k = c / sin(c);
  }
  const double north = k * (cos_lat0 * sin_lat - sin_lat0 * cos_lat * cos_dlon) * R;
  const double east  = k * cos_lat * sin(dlon) * R;

  *east_m  = east;
  *north_m = north;
  *up_m    = alt_m - alt0_m;

#if 0
  // =============================================================================
  // Ellipsoid projection (WGS84) — for scenarios beyond 5km range
  //
  // Replaces the spherical approximation above. The ellipsoid model eliminates
  // the ~0.1-0.3% spherical error (e.g., ~15m at 5km).
  //
  // WGS84 ellipsoid parameters
  // const double a = 6378137.0;                // Semi-major axis (m)
  // const double f = 1.0 / 298.257223563;    // Flattening
  // const double e2 = f * (2.0 - f);         // First eccentricity squared
  //
  // Convert geodetic (lat, lon, alt) to ECEF (X, Y, Z)
  // auto lla_to_ecef = [&](double lat, double lon, double alt,
  //                        double & X, double & Y, double & Z) {
  //   const double phi = lat * M_PI / 180.0;
  //   const double lam = lon * M_PI / 180.0;
  //   const double sin_phi = sin(phi);
  //   const double cos_phi = cos(phi);
  //   const double sin_lam = sin(lam);
  //   const double cos_lam = cos(lam);
  //   const double N = a / sqrt(1.0 - e2 * sin_phi * sin_phi);
  //   X = (N + alt) * cos_phi * cos_lam;
  //   Y = (N + alt) * cos_phi * sin_lam;
  //   Z = (N * (1.0 - e2) + alt) * sin_phi;
  // };
  //
  // double X, Y, Z;
  // double X0, Y0, Z0;
  // lla_to_ecef(lat_deg, lon_deg, alt_m, X, Y, Z);
  // lla_to_ecef(lat0_deg, lon0_deg, alt0_m, X0, Y0, Z0);
  //
  // ECEF delta
  // const double dx = X - X0;
  // const double dy = Y - Y0;
  // const double dz = Z - Z0;
  //
  // Rotate ECEF delta to ENU using origin's geodetic position
  // const double phi0 = lat0_deg * M_PI / 180.0;
  // const double lam0 = lon0_deg * M_PI / 180.0;
  // const double sin_phi0 = sin(phi0);
  // const double cos_phi0 = cos(phi0);
  // const double sin_lam0 = sin(lam0);
  // const double cos_lam0 = cos(lam0);
  //
  // ENU rotation matrix (applied to ECEF delta vector)
  // *east_m  = -sin_lam0 * dx           + cos_lam0 * dy;
  // *north_m = -sin_phi0 * cos_lam0 * dx - sin_phi0 * sin_lam0 * dy + cos_phi0 * dz;
  // *up_m    =  cos_phi0 * cos_lam0 * dx + cos_phi0 * sin_lam0 * dy + sin_phi0 * dz;
  // =============================================================================
#endif
}

// 发布目标位置（GPS 坐标投影转换到 局部ENU坐标）
void InputBridgeNode::processGoalFix(
  const sensor_msgs::msg::NavSatFix::SharedPtr msg,
  const std::string & source)
{
  if (require_goal_fix_ &&
    msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
  {
    // Throttle to 500ms (was 3000ms) so manual-pub failures surface in logs
    // within a second rather than after three.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 500,
      "%s goal has no GPS fix", source.c_str());
    return;
  }

  bool have_origin = false;
  double lat0 = 0.0, lon0 = 0.0, alt0 = 0.0;
  {
    std::lock_guard<std::mutex> lock(origin_mutex_);
    have_origin = origin_valid_;
    lat0 = origin_lat_;
    lon0 = origin_lon_;
    alt0 = origin_alt_;
  }

  if (!have_origin) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "ENU reference invalid: fix reference_latitude, reference_longitude, reference_altitude");
    return;
  }

  double east = 0.0, north = 0.0, up = 0.0;
  llaToEnu(
    msg->latitude, msg->longitude, msg->altitude,
    lat0, lon0, alt0,
    &east, &north, &up);

  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = goal_output_frame_;
  out.pose.position.x = east;
  out.pose.position.y = north;
  out.pose.position.z = up;

  if (goal_yaw_from_bearing_) {
    const double yaw = std::atan2(east, north);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    out.pose.orientation = tf2::toMsg(q);
  } else {
    out.pose.orientation.w = 1.0;
  }

  goal_pub_->publish(out);

  RCLCPP_INFO(
    get_logger(),
    "[%s] Goal published: E=%.2f m, N=%.2f m, U=%.2f m",
    source.c_str(), east, north, up);
}

void InputBridgeNode::onGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  processGoalFix(msg, "manual");
}

void InputBridgeNode::onUavGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  processGoalFix(msg, "uav");
}

// 发送取消导航任务指令
void InputBridgeNode::publishStopGoal()
{
  RCLCPP_WARN(get_logger(), "[yolo] Target detected! Cancelling current FollowPath tasks...");

  // 先通知 straight_planner 停止发送新目标（不依赖 action server）
  if (stop_planner_pub_) {
    std_msgs::msg::Bool stop_msg;
    stop_msg.data = true;
    stop_planner_pub_->publish(stop_msg);
    RCLCPP_INFO(get_logger(), "[yolo] (1) Sent stop signal to straight_planner.");
  }

  // 再尝试取消 FollowPath action
  if (!follow_path_action_client_->action_server_is_ready()) {
    RCLCPP_WARN(get_logger(), "FollowPath action server not ready yet, skipping cancel.");
    return;
  }
  follow_path_action_client_->async_cancel_all_goals();

  RCLCPP_INFO(get_logger(), "[yolo] (2) Sent cancel request to FollowPath action server.");
}

// 处理目标检测信号
void InputBridgeNode::onTargetDetected(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {
    return;
  }

  publishStopGoal();
}

}  // namespace nav2_input_bridge
