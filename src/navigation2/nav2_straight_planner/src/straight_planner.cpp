// Copyright 2026

#include "nav2_straight_planner/straight_planner.hpp"

#include <cmath>

#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_straight_planner
{

StraightPlanner::StraightPlanner(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("straight_planner", "", options)
{
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("robot_base_frame", std::string("base_link"));
  declare_parameter("path_step_m", 0.1);
  declare_parameter("goal_xy_tolerance", 0.3);
  declare_parameter("planning_period_s", 1.0);
  declare_parameter("controller_id", std::string("FollowPath"));
  declare_parameter("goal_checker_id", std::string("general_goal_checker"));
  // Relative name so the action resolves to /{robot_namespace}/follow_path,
  // matching controller_server's default action name inside the same namespace.
  declare_parameter("follow_path_action", std::string("follow_path"));
}

nav2_util::CallbackReturn
StraightPlanner::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  get_parameter("global_frame", global_frame_);
  get_parameter("robot_base_frame", robot_frame_);
  get_parameter("path_step_m", path_step_m_);
  get_parameter("goal_xy_tolerance", goal_xy_tolerance_);
  get_parameter("planning_period_s", planning_period_s_);
  get_parameter("controller_id", controller_id_);
  get_parameter("goal_checker_id", goal_checker_id_);
  get_parameter("follow_path_action", action_name_);

  reentrant_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", rclcpp::QoS(10),
    std::bind(&StraightPlanner::onGoal, this, std::placeholders::_1));

  stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "stop_planner", rclcpp::QoS(10),
    std::bind(&StraightPlanner::onStop, this, std::placeholders::_1));

  action_client_ = rclcpp_action::create_client<FollowPath>(
    this, action_name_, reentrant_group_);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(planning_period_s_),
    std::bind(&StraightPlanner::onPlanTick, this),
    reentrant_group_);
  timer_->cancel();

  RCLCPP_INFO(
    get_logger(),
    "Configured: map=%s base=%s action=%s period=%.2fs step=%.3fm",
    global_frame_.c_str(), robot_frame_.c_str(), action_name_.c_str(),
    planning_period_s_, path_step_m_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
StraightPlanner::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  createBond();
  if (timer_) {
    timer_->reset();
  }
  RCLCPP_INFO(get_logger(), "Activated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
StraightPlanner::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  if (timer_) {
    timer_->cancel();
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (active_goal_handle_ && action_client_) {
      action_client_->async_cancel_goal(active_goal_handle_);
      active_goal_handle_.reset();
    }
    goal_valid_ = false;
    wait_fp_finish_after_reached_ = false;
  }

  destroyBond();
  RCLCPP_INFO(get_logger(), "Deactivated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
StraightPlanner::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  goal_sub_.reset();
  stop_sub_.reset();
  action_client_.reset();
  timer_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  reentrant_group_.reset();

  std::lock_guard<std::mutex> lock(state_mutex_);
  active_goal_handle_.reset();
  goal_valid_ = false;
  wait_fp_finish_after_reached_ = false;

  RCLCPP_INFO(get_logger(), "Cleaned up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
StraightPlanner::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutdown");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool StraightPlanner::isActive()
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

void StraightPlanner::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!isActive()) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring goal while not active (lifecycle state != active)");
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);

  // If previously stopped by YOLO, check if this goal matches the stopped goal
  if (goal_was_stopped_) {
    double dx = msg->pose.position.x - stopped_goal_x_;
    double dy = msg->pose.position.y - stopped_goal_y_;
    double dist = std::hypot(dx, dy);

    if (dist < 0.1) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting goal that matches stopped goal (dist=%.3f m < 0.1 m)", dist);
      return;  // Discard, do not update latest_goal_
    }
    // Different goal, clear the stopped state and allow stop to work again
    goal_was_stopped_ = false;
    stop_requested_ = false;
    RCLCPP_INFO(
      get_logger(),
      "New goal is different from stopped goal (dist=%.3f m), accepting", dist);
  }

  wait_fp_finish_after_reached_ = false;
  latest_goal_ = *msg;
  goal_valid_ = true;
  RCLCPP_INFO(
    get_logger(), "New goal [%.3f, %.3f] frame=%s", msg->pose.position.x,
    msg->pose.position.y, msg->header.frame_id.c_str());
}

void StraightPlanner::onStop(const std_msgs::msg::Bool::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (stop_requested_) {
    return;
  }
  stop_requested_ = true;
  RCLCPP_WARN(get_logger(), "Stop requested! Cancelling active FollowPath and blocking new goals");
  if (active_goal_handle_ && action_client_) {
    action_client_->async_cancel_goal(active_goal_handle_);
    active_goal_handle_.reset();
  }
  // Record the stopped goal position before invalidating
  stopped_goal_x_ = latest_goal_.pose.position.x;
  stopped_goal_y_ = latest_goal_.pose.position.y;
  goal_was_stopped_ = true;
  RCLCPP_INFO(
    get_logger(), "Recorded stopped goal position [%.3f, %.3f]", stopped_goal_x_, stopped_goal_y_);
  goal_valid_ = false;
  wait_fp_finish_after_reached_ = false;
}

nav_msgs::msg::Path StraightPlanner::buildStraightPath(
  const geometry_msgs::msg::PoseStamped & start_map,
  const geometry_msgs::msg::PoseStamped & goal_map) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = global_frame_;

  const double sx = start_map.pose.position.x;
  const double sy = start_map.pose.position.y;
  const double gx = goal_map.pose.position.x;
  const double gy = goal_map.pose.position.y;
  const double dx = gx - sx;
  const double dy = gy - sy;
  const double len = std::hypot(dx, dy);

  if (len < 1e-6) {
    geometry_msgs::msg::PoseStamped p = goal_map;
    p.header = path.header;
    path.poses.push_back(p);
    return path;
  }

  const double yaw = std::atan2(dy, dx);
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  int n = static_cast<int>(std::ceil(len / path_step_m_));
  n = std::max(n, 2);
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = sx + t * dx;
    ps.pose.position.y = sy + t * dy;
    ps.pose.position.z = 0.0;
    ps.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(ps);
  }
  path.poses.back().pose.position = goal_map.pose.position;
  path.poses.back().pose.orientation = goal_map.pose.orientation;
  return path;
}

void StraightPlanner::onPlanTick()
{
  if (!isActive()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (stop_requested_) {
      return;
    }
    if (!goal_valid_) {
      return;
    }
    if (wait_fp_finish_after_reached_) {
      if (!active_goal_handle_) {
        goal_valid_ = false;
        wait_fp_finish_after_reached_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Wait FollowPath finish but no active goal; clearing state");
      }
      return;
    }
  }

  if (!action_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "FollowPath action not ready: %s", action_name_.c_str());
    return;
  }

  geometry_msgs::msg::PoseStamped robot_map;
  try {
    const auto tf = tf_buffer_->lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero);
    robot_map.header.frame_id = global_frame_;
    robot_map.header.stamp = tf.header.stamp;
    robot_map.pose.position.x = tf.transform.translation.x;
    robot_map.pose.position.y = tf.transform.translation.y;
    robot_map.pose.position.z = tf.transform.translation.z;
    robot_map.pose.orientation = tf.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF robot: %s", ex.what());
    return;
  }

  geometry_msgs::msg::PoseStamped goal_copy;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    goal_copy = latest_goal_;
  }
  geometry_msgs::msg::PoseStamped goal_map = goal_copy;
  if (goal_map.header.frame_id != global_frame_) {
    try {
      goal_map = tf_buffer_->transform(goal_copy, global_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF goal: %s", ex.what());
      return;
    }
  }

  const double rx = robot_map.pose.position.x;
  const double ry = robot_map.pose.position.y;
  const double gx = goal_map.pose.position.x;
  const double gy = goal_map.pose.position.y;
  const double dist = std::hypot(gx - rx, gy - ry);

  FollowPath::Goal goal_msg;
  GoalHandleFollowPath::SharedPtr cancel_handle;
  bool send_new_goal = false;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!goal_valid_ || wait_fp_finish_after_reached_) {
      return;
    }

    if (dist < goal_xy_tolerance_) {
      if (!active_goal_handle_) {
        goal_valid_ = false;
        RCLCPP_INFO(
          get_logger(), "Goal reached (dist=%.3f m), no active FollowPath; done", dist);
        return;
      }
      wait_fp_finish_after_reached_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Planner reached goal (dist=%.3f m); waiting current FollowPath to finish (no cancel)",
        dist);
      return;
    }

    if (active_goal_handle_) {
      cancel_handle = active_goal_handle_;
      active_goal_handle_.reset();
    }

    nav_msgs::msg::Path path = buildStraightPath(robot_map, goal_map);
    goal_msg.path = path;
    goal_msg.controller_id = controller_id_;
    goal_msg.goal_checker_id = goal_checker_id_;
    send_new_goal = true;
  }

  if (cancel_handle) {
    action_client_->async_cancel_goal(cancel_handle);
  }

  if (!send_new_goal) {
    return;
  }

  rclcpp_action::Client<FollowPath>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](GoalHandleFollowPath::SharedPtr handle) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!handle) {
        RCLCPP_WARN(get_logger(), "FollowPath goal rejected");
        return;
      }
      active_goal_handle_ = handle;
    };
  opts.result_callback = [this](const GoalHandleFollowPath::WrappedResult & result) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (active_goal_handle_ && active_goal_handle_->get_goal_id() == result.goal_id) {
      active_goal_handle_.reset();
    }
    if (wait_fp_finish_after_reached_) {
      goal_valid_ = false;
      wait_fp_finish_after_reached_ = false;
      RCLCPP_INFO(
        get_logger(),
        "FollowPath ended after planner-reached (code=%d); stop sending new goals",
        static_cast<int>(result.code));
      return;
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_DEBUG(
        get_logger(), "FollowPath finished with code %d", static_cast<int>(result.code));
    }
  };

  action_client_->async_send_goal(goal_msg, opts);
  RCLCPP_DEBUG(
    get_logger(), "Sent FollowPath: poses=%zu dist~=%.2f m", goal_msg.path.poses.size(),
    dist);
}

}  // namespace nav2_straight_planner
