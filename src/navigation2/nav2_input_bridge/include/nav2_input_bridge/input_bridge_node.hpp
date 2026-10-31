// Copyright 2026
#ifndef NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_
#define NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace nav2_input_bridge
{

class InputBridgeNode : public rclcpp::Node
{
public:
  explicit InputBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using FollowPath = nav2_msgs::action::FollowPath;

  // Which subscribed topic the odometry currently comes from.
  //   Primary  - local_odom_topic (the contract topic, e.g. mavros/odom)
  //   Fallback - local_odom_fallback_topic (e.g. odom), used only while the
  //              primary has never delivered a message.
  enum class OdomSource { None, Primary, Fallback };

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg, OdomSource source);
  void onGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void checkOdomHealth();
  void onUavGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void processGoalFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg, const std::string & source);

  void onTargetDetected(const std_msgs::msg::Bool::SharedPtr msg);
  void publishStopGoal();

  void llaToEnu(
    double lat_deg, double lon_deg, double alt_m,
    double lat0_deg, double lon0_deg, double alt0_m,
    double * east_m, double * north_m, double * up_m) const;

  std::string local_odom_topic_;
  std::string local_odom_fallback_topic_;
  std::string goal_input_topic_;
  std::string uav_goal_input_topic_;
  std::string target_detected_topic_;
  std::string follow_path_action_topic_;
  std::string goal_output_topic_;
  std::string goal_output_frame_;
  bool require_goal_fix_;
  bool goal_yaw_from_bearing_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_fallback_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr uav_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_detected_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_planner_pub_;

  // Broadcasts /{ns}/odom -> /{ns}/base_link from the mavros odom topic.
  // (The installed mavros C++ local_position plugin CAN also broadcast this
  // TF when local_position.tf.send=true, but we do it here so the TF exists
  // independent of mavros build/version quirks, and from the exact same odom
  // that feeds the costmap. Without this TF the costmap/follow_path time out
  // with "unconnected trees".)
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string tf_frame_id_;
  std::string tf_child_frame_id_;

  // --- Odometry source selection and watchdog ------------------------------
  // The odom -> base_link TF is broadcast ONLY while odometry arrives. If no
  // odometry reaches this node the TF tree splits into two unconnected trees
  // ({map -> odom} and {base_link -> livox_frame}) and nav2's local_costmap
  // reports "Could not find a connection between map and base_link ... Tf has
  // two or more unconnected trees" — a symptom that points at TF instead of at
  // the missing odometry. The watchdog names the actual cause.
  //
  // Two topics are subscribed:
  //   primary  (local_odom_topic)            - the contract topic, e.g. mavros/odom
  //   fallback (local_odom_fallback_topic)   - e.g. odom, used only while the
  //                                            primary has never delivered a message
  // The primary wins permanently as soon as it delivers data.
  OdomSource active_odom_source_{OdomSource::None};
  rclcpp::TimerBase::SharedPtr odom_watchdog_;
  std::atomic<uint64_t> odom_count_{0};
  std::atomic<int64_t> last_odom_ns_{0};
  int64_t last_odom_log_ns_{0};
  int64_t start_ns_{0};
  double odom_timeout_s_{2.0};
  double odom_watchdog_period_s_{1.0};

  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_action_client_;

  std::mutex origin_mutex_;
  bool origin_valid_{false};
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};

  static constexpr double WGS84_R_{6378137.0};
};

}  // namespace nav2_input_bridge

#endif  // NAV2_INPUT_BRIDGE__INPUT_BRIDGE_NODE_HPP_
