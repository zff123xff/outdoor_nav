// Copyright 2026
#include "nav2_straight_planner/straight_planner.hpp"
#include <cmath>
#include <algorithm>
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
  declare_parameter("planning_period_s", 0.5);
  declare_parameter("controller_id", std::string("FollowPath"));
  declare_parameter("goal_checker_id", std::string("general_goal_checker"));
  declare_parameter("follow_path_action", std::string("follow_path"));
}
nav2_util::CallbackReturn StraightPlanner::on_configure(const rclcpp_lifecycle::State &)
{
  get_parameter("global_frame", global_frame_);
  get_parameter("robot_base_frame", robot_frame_);
  get_parameter("path_step_m", path_step_m_);
  get_parameter("planning_period_s", planning_period_s_);
  get_parameter("controller_id", controller_id_);
  get_parameter("goal_checker_id", goal_checker_id_);
  get_parameter("follow_path_action", action_name_);
  if (!std::isfinite(path_step_m_) || path_step_m_ <= 0 || !std::isfinite(planning_period_s_) || planning_period_s_ <= 0) {
    RCLCPP_ERROR(get_logger(), "Path step and planning period must be positive and finite");
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (!reentrant_group_) {
    reentrant_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  action_client_ = rclcpp_action::create_client<FollowPath>(this, action_name_, reentrant_group_);
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10,
    std::bind(&StraightPlanner::onGoal, this, std::placeholders::_1));
  stop_sub_ = create_subscription<std_msgs::msg::Bool>("stop_planner", 10,
    std::bind(&StraightPlanner::onStop, this, std::placeholders::_1));
  area_search_sub_ = create_subscription<std_msgs::msg::Bool>(
    "area_search/active", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::Bool::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      area_search_heartbeat_ = std::chrono::steady_clock::now();
      if (msg->data && !area_search_active_) {cancelFollowPath(); finishGoal(false);}
      area_search_active_ = msg->data;
    });
  navigation_server_ = rclcpp_action::create_server<Navigate>(
    this, "navigate_to_pose",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal> goal) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto & p = goal->pose.pose;
      if (!isActive() || area_search_active_ || navigation_goal_ || navigation_goal_pending_ ||
        !goal->behavior_tree.empty() || goal->pose.header.frame_id.empty() ||
        !std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z) ||
        !std::isfinite(p.orientation.x) || !std::isfinite(p.orientation.y) ||
        !std::isfinite(p.orientation.z) || !std::isfinite(p.orientation.w) ||
        std::abs(p.orientation.x*p.orientation.x + p.orientation.y*p.orientation.y +
        p.orientation.z*p.orientation.z + p.orientation.w*p.orientation.w - 1.0) > 0.01) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      navigation_goal_pending_ = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](std::shared_ptr<NavigationHandle> handle) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (navigation_goal_ != handle) {return rclcpp_action::CancelResponse::REJECT;}
      cancelFollowPath();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](std::shared_ptr<NavigationHandle> handle) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      navigation_goal_pending_ = false;
      cancelFollowPath();
      navigation_goal_ = handle;
      if (!isActive() || area_search_active_) {finishGoal(false); return;}
      latest_goal_ = handle->get_goal()->pose;
      goal_valid_ = true;
      stop_requested_ = false;
    });
  timer_ = create_wall_timer(std::chrono::duration<double>(planning_period_s_),
    std::bind(&StraightPlanner::onPlanTick, this), reentrant_group_);
  timer_->cancel();
  return nav2_util::CallbackReturn::SUCCESS;
}
void StraightPlanner::cancelFollowPath()
{
  ++generation_;
  if (active_goal_handle_ && action_client_) {action_client_->async_cancel_goal(active_goal_handle_);}
  active_goal_handle_.reset();
  follow_path_pending_ = false;
}
void StraightPlanner::finishGoal(bool success)
{
  ++generation_;
  goal_valid_ = false;
  follow_path_pending_ = false;
  active_goal_handle_.reset();
  if (navigation_goal_) {
    auto result = std::make_shared<Navigate::Result>();
    if (navigation_goal_->is_canceling()) {navigation_goal_->canceled(result);}
    else if (success) {navigation_goal_->succeed(result);}
    else {navigation_goal_->abort(result);}
    navigation_goal_.reset();
  }
}
nav2_util::CallbackReturn StraightPlanner::on_activate(const rclcpp_lifecycle::State &)
{
  createBond(); timer_->reset(); return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn StraightPlanner::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_->cancel();
  {std::lock_guard<std::mutex> lock(state_mutex_); cancelFollowPath(); finishGoal(false);}
  destroyBond(); return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn StraightPlanner::on_cleanup(const rclcpp_lifecycle::State &)
{
  {std::lock_guard<std::mutex> lock(state_mutex_); cancelFollowPath(); finishGoal(false); navigation_goal_pending_ = false;}
  navigation_server_.reset(); goal_sub_.reset(); stop_sub_.reset(); area_search_sub_.reset();
  timer_.reset(); action_client_.reset(); tf_listener_.reset(); tf_buffer_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn StraightPlanner::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (timer_) {timer_->cancel();}
  std::lock_guard<std::mutex> lock(state_mutex_); cancelFollowPath(); finishGoal(false);
  return nav2_util::CallbackReturn::SUCCESS;
}
bool StraightPlanner::isActive()
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}
void StraightPlanner::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // An uncorrelated topic goal must not replace an Action owned by a task.
  if (!isActive() || area_search_active_ || navigation_goal_ || navigation_goal_pending_) {return;}
  if (stop_requested_) {
    if (std::hypot(msg->pose.position.x-latest_goal_.pose.position.x, msg->pose.position.y-latest_goal_.pose.position.y) < 0.1) {return;}
    stop_requested_ = false;
  }
  cancelFollowPath(); latest_goal_ = *msg; goal_valid_ = true;
}
void StraightPlanner::onStop(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) {return;}
  std::lock_guard<std::mutex> lock(state_mutex_);
  stop_requested_ = true;
  cancelFollowPath(); finishGoal(false);
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
  if (!isActive()) {return;}
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (navigation_goal_ && navigation_goal_->is_canceling()) {finishGoal(false); return;}
  if (area_search_active_) {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now()-area_search_heartbeat_).count() > 4.0 &&
      action_client_ && action_client_->action_server_is_ready()) {
      action_client_->async_cancel_all_goals();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "area_search heartbeat expired; cancelling FollowPath");
    }
    return;
  }
  if (!goal_valid_ || stop_requested_ || !action_client_->action_server_is_ready()) {return;}
  geometry_msgs::msg::PoseStamped robot, goal = latest_goal_;
  try {
    auto tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
    robot.header.frame_id = global_frame_; robot.header.stamp = tf.header.stamp;
    robot.pose.position.x = tf.transform.translation.x; robot.pose.position.y = tf.transform.translation.y;
    robot.pose.position.z = tf.transform.translation.z; robot.pose.orientation = tf.transform.rotation;
    if (goal.header.frame_id != global_frame_) {goal = tf_buffer_->transform(goal, global_frame_, tf2::durationFromSec(0.2));}
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Navigation TF unavailable: %s", ex.what()); return;
  }
  if (navigation_goal_) {
    auto feedback = std::make_shared<Navigate::Feedback>();
    feedback->current_pose = robot;
    feedback->distance_remaining = std::hypot(goal.pose.position.x-robot.pose.position.x, goal.pose.position.y-robot.pose.position.y);
    navigation_goal_->publish_feedback(feedback);
  }
  if (follow_path_pending_) {return;}
  FollowPath::Goal request;
  request.path = buildStraightPath(robot, goal);
  request.controller_id = controller_id_; request.goal_checker_id = goal_checker_id_;
  follow_path_pending_ = true;
  const auto generation = generation_;
  rclcpp_action::Client<FollowPath>::SendGoalOptions options;
  options.goal_response_callback = [this, generation](GoalHandleFollowPath::SharedPtr handle) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (generation != generation_ || !goal_valid_) {
      if (handle) {action_client_->async_cancel_goal(handle);} return;
    }
    if (!handle) {RCLCPP_ERROR(get_logger(), "FollowPath rejected"); finishGoal(false); return;}
    active_goal_handle_ = handle;
  };
  options.result_callback = [this, generation](const GoalHandleFollowPath::WrappedResult & result) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (generation != generation_ || !goal_valid_) {return;}
    // The controller's Action result is the only authority for arrival.
    const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
    RCLCPP_INFO(get_logger(), "Navigation completed from FollowPath result: %d", static_cast<int>(result.code));
    finishGoal(success);
  };
  action_client_->async_send_goal(request, options);
}
}  // namespace nav2_straight_planner
