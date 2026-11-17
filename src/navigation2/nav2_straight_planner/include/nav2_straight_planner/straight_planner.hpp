// Copyright 2026
#ifndef NAV2_STRAIGHT_PLANNER__STRAIGHT_PLANNER_HPP_
#define NAV2_STRAIGHT_PLANNER__STRAIGHT_PLANNER_HPP_
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
namespace nav2_straight_planner
{
class StraightPlanner : public nav2_util::LifecycleNode
{
public:
  explicit StraightPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
private:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
  using Navigate = nav2_msgs::action::NavigateToPose;
  using NavigationHandle = rclcpp_action::ServerGoalHandle<Navigate>;
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr);
  void onStop(const std_msgs::msg::Bool::SharedPtr);
  void onPlanTick();
  bool isActive();
  void finishGoal(bool success);
  void cancelFollowPath();
  nav_msgs::msg::Path buildStraightPath(const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::PoseStamped &) const;
  std::string global_frame_, robot_frame_, controller_id_, goal_checker_id_, action_name_;
  double path_step_m_, planning_period_s_, max_path_length_m_;
  rclcpp::CallbackGroup::SharedPtr reentrant_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_, area_search_sub_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  rclcpp_action::Server<Navigate>::SharedPtr navigation_server_;
  std::shared_ptr<NavigationHandle> navigation_goal_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex state_mutex_;
  bool goal_valid_{false}, follow_path_pending_{false}, navigation_goal_pending_{false};
  bool area_search_active_{false}, stop_requested_{false};
  uint64_t generation_{0};
  std::chrono::steady_clock::time_point area_search_heartbeat_;
  geometry_msgs::msg::PoseStamped latest_goal_;
  GoalHandleFollowPath::SharedPtr active_goal_handle_;
};
}
#endif
