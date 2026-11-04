// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace nav2_regulated_pure_pursuit_controller
{

namespace
{
const char * motionModeLabel(const std::string & mode)
{
  if (mode == "backup") {return "后退";}
  if (mode == "rotate_path") {return "掉头（对齐路径）";}
  if (mode == "rotate_goal") {return "掉头（对齐目标朝向）";}
  if (mode == "tracking") {return "跟踪";}
  return "待命";
}
}  // namespace

void RegulatedPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  planner_ = std::make_unique<theta_star::ThetaStar>();
  planner_->costmap_ = costmap_ros->getCostmap();

  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".prefer_supplied_path", rclcpp::ParameterValue(true));
  node->get_parameter(plugin_name_ + ".prefer_supplied_path", prefer_supplied_path_);
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".local_path_smoothing_iterations", rclcpp::ParameterValue(2));
  node->get_parameter(
    plugin_name_ + ".local_path_smoothing_iterations", local_path_smoothing_iterations_);
  if (local_path_smoothing_iterations_ < 0 || local_path_smoothing_iterations_ > 5) {
    throw std::invalid_argument("local_path_smoothing_iterations must be between 0 and 5");
  }
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist",
    rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_collision_detection",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.90));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_backup_recovery", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".backup_recovery_trigger_cost", rclcpp::ParameterValue(200.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".backup_recovery_distance", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".backup_recovery_speed", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".stateful", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_hysteresis", rclcpp::ParameterValue(0.2617993877991494));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_direction_lock_angle",
    rclcpp::ParameterValue(0.17453292519943295));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation",
    rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity",
    min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist",
    approach_velocity_scaling_dist_);
  if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
    RCLCPP_WARN(
      logger_, "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node->get_parameter(
    plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
    max_allowed_time_to_collision_up_to_carrot_);
  node->get_parameter(
    plugin_name_ + ".use_collision_detection",
    use_collision_detection_);
  node->get_parameter(
    plugin_name_ + ".use_regulated_linear_velocity_scaling",
    use_regulated_linear_velocity_scaling_);
  node->get_parameter(
    plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    use_cost_regulated_linear_velocity_scaling_);
  node->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
  node->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
  node->get_parameter(
    plugin_name_ + ".inflation_cost_scaling_factor",
    inflation_cost_scaling_factor_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_radius",
    regulated_linear_scaling_min_radius_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_speed",
    regulated_linear_scaling_min_speed_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  declare_parameter_if_not_declared(node, plugin_name_ + ".rotation_accel_gain", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".rotation_damping_gain", rclcpp::ParameterValue(2.5));
  node->get_parameter(plugin_name_ + ".rotation_accel_gain", rotation_accel_gain_);
  node->get_parameter(plugin_name_ + ".rotation_damping_gain", rotation_damping_gain_);
  declare_parameter_if_not_declared(node, plugin_name_ + ".tracking_angular_vel_limit", rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_rotate_to_goal_heading", rclcpp::ParameterValue(true));
  node->get_parameter(plugin_name_ + ".tracking_angular_vel_limit", tracking_angular_vel_limit_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_goal_heading", use_rotate_to_goal_heading_);
  if (!std::isfinite(rotation_accel_gain_) || rotation_accel_gain_ <= 0.0 ||
    !std::isfinite(rotation_damping_gain_) || rotation_damping_gain_ < 0.0 ||
    !std::isfinite(tracking_angular_vel_limit_) || tracking_angular_vel_limit_ <= 0.0)
  {
    throw std::invalid_argument("Invalid heading acceleration or damping gain");
  }
  node->get_parameter(plugin_name_ + ".use_backup_recovery", use_backup_recovery_);
  node->get_parameter(plugin_name_ + ".backup_recovery_trigger_cost", backup_recovery_trigger_cost_);
  node->get_parameter(plugin_name_ + ".backup_recovery_distance", backup_recovery_distance_);
  node->get_parameter(plugin_name_ + ".backup_recovery_speed", backup_recovery_speed_);
  if (!std::isfinite(backup_recovery_trigger_cost_) || backup_recovery_trigger_cost_ <= 0.0 ||
    backup_recovery_trigger_cost_ >= INSCRIBED_INFLATED_OBSTACLE ||
    !std::isfinite(backup_recovery_distance_) || backup_recovery_distance_ <= 0.0 ||
    !std::isfinite(backup_recovery_speed_) || backup_recovery_speed_ <= 0.0)
  {
    throw std::invalid_argument("Invalid backup recovery cost, distance or speed");
  }
  node->get_parameter(plugin_name_ + ".stateful", stateful_);
  reached_xy_tolerance_ = false;
  rotating_to_path_ = false;
  rotation_direction_ = 0.0;
  backup_active_ = false;
  backup_exhausted_ = false;
  node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_hysteresis", rotate_to_heading_hysteresis_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_direction_lock_angle", rotate_to_heading_direction_lock_angle_);
  if (!std::isfinite(rotate_to_heading_hysteresis_) || rotate_to_heading_hysteresis_ < 0.0 ||
    !std::isfinite(rotate_to_heading_direction_lock_angle_) ||
    rotate_to_heading_direction_lock_angle_ < 0.0 || rotate_to_heading_direction_lock_angle_ >= M_PI_2)
  {
    throw std::invalid_argument("Heading hysteresis must be nonnegative; direction lock angle must be in [0, pi/2)");
  }
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  node->get_parameter("controller_frequency", control_frequency);
  node->get_parameter(
    plugin_name_ + ".max_robot_pose_search_dist",
    max_robot_pose_search_dist_);
  node->get_parameter(
    plugin_name_ + ".use_interpolation",
    use_interpolation_);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  if (inflation_cost_scaling_factor_ <= 0.0) {
    RCLCPP_WARN(
      logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
      "it should be >0. Disabling cost regulated linear velocity scaling.");
    use_cost_regulated_linear_velocity_scaling_ = false;
  }

  /** Possible to drive in reverse direction if and only if
   "use_rotate_to_heading" parameter is set to false **/

  if (use_rotate_to_heading_ && allow_reversing_) {
    RCLCPP_WARN(
      logger_, "Disabling reversing. Both use_rotate_to_heading and allow_reversing "
      "parameter cannot be set to true. By default setting use_rotate_to_heading true");
    allow_reversing_ = false;
  }

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  carrot_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);
}

void RegulatedPurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  carrot_pub_.reset();
  carrot_arc_pub_.reset();
}

void RegulatedPurePursuitController::activate()
{
  last_heading_mode_ = "idle";
  rotation_command_ = 0.0;
  reached_xy_tolerance_ = false;
  rotating_to_path_ = false;
  rotation_direction_ = 0.0;
  backup_active_ = false;
  backup_exhausted_ = false;
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  carrot_pub_->on_activate();
  carrot_arc_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &RegulatedPurePursuitController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void RegulatedPurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  carrot_arc_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

std::unique_ptr<geometry_msgs::msg::PointStamped> RegulatedPurePursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  double backup_linear = 0.0, backup_angular = 0.0;
  if (computeBackupCommand(pose, backup_linear, backup_angular)) {
    if (use_collision_detection_ && isCollisionImminent(
        pose, backup_linear, backup_angular, backup_recovery_distance_))
    {
      throw nav2_core::PlannerException("Backup recovery collision check blocked motion");
    }
    if (last_heading_mode_ != "backup") {
      RCLCPP_INFO(
        logger_, "控制模式: %s -> 后退 heading_mode: %s -> backup command_vx=%.4f",
        motionModeLabel(last_heading_mode_), last_heading_mode_.c_str(), backup_linear);
      last_heading_mode_ = "backup";
    }
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = backup_linear;
    return command;
  }

  // get local plan
  local_plan_ = getLocalPlan(pose);
  
  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (allow_reversing_) {
    // Cusp check
    double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }

  // Setting the velocity direction
  double sign = 1.0;
  if (allow_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  // The carrot and the end of a local detour are not necessarily the mission
  // goal. Only the actual final goal may trigger final-heading alignment.
  geometry_msgs::msg::PoseStamped final_goal = global_plan_.poses.back();
  final_goal.header = global_plan_.header;
  final_goal.header.stamp = pose.header.stamp;
  geometry_msgs::msg::PoseStamped final_goal_in_base;
  if (!transformPose(costmap_ros_->getBaseFrameID(), final_goal, final_goal_in_base)) {
    throw nav2_core::PlannerException("Unable to transform final goal into robot frame");
  }
  const double goal_distance = std::hypot(
    final_goal_in_base.pose.position.x, final_goal_in_base.pose.position.y);
  const double angle_to_goal = tf2::getYaw(final_goal_in_base.pose.orientation);
  const double path_heading_error = std::atan2(
    carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  const char * heading_mode;
  if (shouldRotateToGoalHeading(final_goal_in_base)) {
    heading_mode = "rotate_goal";
    rotating_to_path_ = false;
    if (last_heading_mode_ != heading_mode) {rotation_direction_ = 0.0; rotation_command_ = 0.0;}
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    heading_mode = "rotate_path";
    if (last_heading_mode_ != heading_mode) {rotation_direction_ = 0.0; rotation_command_ = 0.0;}
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    heading_mode = "tracking";
    rotation_command_ = 0.0;
    applyConstraints(
      curvature, speed,
      costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, sign);

    // Apply curvature to angular velocity after constraining linear velocity
    if (std::abs(curvature) > 1e-6) {
      linear_vel = std::min(linear_vel, tracking_angular_vel_limit_ / std::abs(curvature));
    }
    angular_vel = linear_vel * curvature;
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (use_collision_detection_ && isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
    throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!");
  }

  if (last_heading_mode_ != heading_mode) {
    RCLCPP_INFO(
      logger_,
      "控制模式: %s -> %s heading_mode: %s -> %s goal_distance=%.4f carrot_distance=%.4f "
      "path_error=%.6f goal_error=%.6f command_vx=%.4f command_wz=%.4f",
      motionModeLabel(last_heading_mode_), motionModeLabel(heading_mode),
      last_heading_mode_.c_str(), heading_mode, goal_distance,
      std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y),
      path_heading_error, angle_to_goal, linear_vel, angular_vel);
    last_heading_mode_ = heading_mode;
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

bool RegulatedPurePursuitController::isBackupPoseSafe(double x, double y, double yaw)
{
  const double cost = collision_checker_->footprintCostAtPose(
    x, y, yaw, costmap_ros_->getRobotFootprint());
  // Recovery requires observed clearance for the whole footprint, including
  // the current pose; do not waive collisions to escape a lethal cell.
  return cost >= 0.0 && cost < INSCRIBED_INFLATED_OBSTACLE;
}

bool RegulatedPurePursuitController::backupPathIsSafe(
  const geometry_msgs::msg::PoseStamped & pose, double distance)
{
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance/(costmap_->getResolution()/2))));
  for (int i = 0; i <= steps; ++i) {
    const double offset = distance*i/steps;
    const double x = pose.pose.position.x-offset*std::cos(yaw);
    const double y = pose.pose.position.y-offset*std::sin(yaw);
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my) ||
      costmap_->getCost(mx, my) >= INSCRIBED_INFLATED_OBSTACLE ||
      !isBackupPoseSafe(x, y, yaw)) {return false;}
  }
  return true;
}

bool RegulatedPurePursuitController::computeBackupCommand(
  const geometry_msgs::msg::PoseStamped & pose, double & linear, double & angular)
{
  if (!use_backup_recovery_) {return false;}
  unsigned int mx, my;
  if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    throw nav2_core::PlannerException("Robot is outside local costmap");
  }
  const double cost = costmap_->getCost(mx, my);
  double travelled = std::hypot(
    pose.pose.position.x-backup_start_x_, pose.pose.position.y-backup_start_y_);
  if (backup_active_ && (cost < backup_recovery_trigger_cost_ ||
    travelled >= backup_target_distance_))
  {
    backup_active_ = false;
    backup_exhausted_ = true;
    rotating_to_path_ = false;
    rotation_direction_ = 0.0;
    if (cost >= backup_recovery_trigger_cost_) {
      throw nav2_core::PlannerException("Backup distance exhausted without leaving high-cost space");
    }
    return false;
  }
  if (!backup_active_) {
    if (cost < backup_recovery_trigger_cost_ || backup_exhausted_) {return false;}
    const double yaw = tf2::getYaw(pose.pose.orientation);
    bool found = false;
    for (double distance = costmap_->getResolution()/2;
      distance <= backup_recovery_distance_+1e-9; distance += costmap_->getResolution()/2)
    {
      const double x = pose.pose.position.x-distance*std::cos(yaw);
      const double y = pose.pose.position.y-distance*std::sin(yaw);
      unsigned int rx, ry;
      if (!costmap_->worldToMap(x, y, rx, ry)) {break;}
      if (costmap_->getCost(rx, ry) < backup_recovery_trigger_cost_ &&
        backupPathIsSafe(pose, distance))
      {
        backup_target_distance_ = distance;
        found = true;
        break;
      }
    }
    if (!found) {
      throw nav2_core::PlannerException("High-cost start: no collision-free reverse escape within backup distance");
    }
    backup_start_x_ = pose.pose.position.x;
    backup_start_y_ = pose.pose.position.y;
    backup_started_ = clock_->now();
    travelled = 0.0;
    backup_active_ = true;
  }
  const double remaining = std::max(0.0, backup_target_distance_-travelled);
  if ((clock_->now()-backup_started_).seconds() > backup_recovery_distance_/backup_recovery_speed_+2.0 ||
    !backupPathIsSafe(pose, remaining))
  {
    throw nav2_core::PlannerException("Backup recovery timed out or reverse corridor became blocked");
  }
  linear = -backup_recovery_speed_;
  angular = 0.0;
  return true;
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  const double threshold = rotating_to_path_ ?
    std::max(0.0, rotate_to_heading_min_angle_ - rotate_to_heading_hysteresis_) :
    rotate_to_heading_min_angle_;
  rotating_to_path_ = use_rotate_to_heading_ && fabs(angle_to_path) > threshold;
  if (!rotating_to_path_) {rotation_direction_ = 0.0; rotation_command_ = 0.0;}
  return rotating_to_path_;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & goal_pose)
{
  if (!use_rotate_to_heading_ || !use_rotate_to_goal_heading_) {
    return false;
  }
  const double dist_to_goal = std::hypot(goal_pose.pose.position.x, goal_pose.pose.position.y);
  const bool within_xy_tolerance = dist_to_goal <= goal_dist_tol_;
  reached_xy_tolerance_ = reached_xy_tolerance_ || within_xy_tolerance;
  // Match SimpleGoalChecker's stateful position -> heading handover. Position
  // noise around the tolerance must not restart translation during alignment.
  return stateful_ ? reached_xy_tolerance_ : within_xy_tolerance;
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Use the shortest signed angular distance. This also handles equivalent
  // headings outside [-pi, pi] (for example, +350 degrees becomes -10).
  const double shortest_angle = std::atan2(std::sin(angle_to_path), std::cos(angle_to_path));
  const double shortest_sign = shortest_angle >= 0.0 ? 1.0 : -1.0;
  // +/-pi represents the same heading. Preserve the last selected direction
  // only in this ambiguous band; use shortest rotation again outside it.
  const bool ambiguous = rotate_to_heading_direction_lock_angle_ > 0.0 &&
    M_PI - std::abs(shortest_angle) <= rotate_to_heading_direction_lock_angle_;
  if (rotation_direction_ == 0.0 || !ambiguous) {rotation_direction_ = shortest_sign;}
  const double rotation_angle = rotation_direction_ == shortest_sign ? shortest_angle :
    rotation_direction_ * (2.0 * M_PI - std::abs(shortest_angle));

  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = rotation_direction_;
  // Integrate bounded heading-error acceleration from the previous command.
  // Body IMU yaw-rate contains gait impulses: re-anchoring the command to it
  // every cycle can reverse a correctly selected turn direction.
  (void)curr_speed;
  const double angular_acc = std::clamp(
    rotation_accel_gain_ * rotation_angle - rotation_damping_gain_ * rotation_command_,
    -max_angular_accel_, max_angular_accel_);
  angular_vel = std::clamp(
    rotation_command_ + angular_acc * control_duration_,
    -rotate_to_heading_angular_vel_, rotate_to_heading_angular_vel_);

  // Check if we need to slow down to avoid overshooting
  double max_vel_to_stop = std::sqrt(2 * max_angular_accel_ * fabs(rotation_angle));
  if (fabs(angular_vel) > max_vel_to_stop) {
    angular_vel = sign * max_vel_to_stop;
  }
  rotation_command_ = angular_vel;

}

geometry_msgs::msg::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

bool RegulatedPurePursuitController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const double & linear_vel, const double & angular_vel,
  const double & carrot_dist)
{
  // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
  // odom frame and the carrot_pose is in robot base frame.

  // check current point is OK
  if (inCollision(
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      tf2::getYaw(robot_pose.pose.orientation)))
  {
    return true;
  }

  // visualization messages
  nav_msgs::msg::Path arc_pts_msg;
  arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  arc_pts_msg.header.stamp = robot_pose.header.stamp;
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
  pose_msg.header.stamp = arc_pts_msg.header.stamp;

  double projection_time = 0.0;
  if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01) {
    // rotating to heading at goal or toward path
    // Equation finds the angular distance required for the largest
    // part of the robot radius to move to another costmap cell:
    // theta_min = 2.0 * sin ((res/2) / r_max)
    // via isosceles triangle r_max-r_max-resolution,
    // dividing by angular_velocity gives us a timestep.
    double max_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    projection_time =
      2.0 * sin((costmap_->getResolution() / 2) / max_radius) / fabs(angular_vel);
  } else {
    // Normal path tracking
    projection_time = costmap_->getResolution() / fabs(linear_vel);
  }

  const geometry_msgs::msg::Point & robot_xy = robot_pose.pose.position;
  geometry_msgs::msg::Pose2D curr_pose;
  curr_pose.x = robot_pose.pose.position.x;
  curr_pose.y = robot_pose.pose.position.y;
  curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

  // only forward simulate within time requested
  int i = 1;
  while (i * projection_time < max_allowed_time_to_collision_up_to_carrot_) {
    i++;

    // apply velocity at curr_pose over distance
    curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
    curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
    curr_pose.theta += projection_time * angular_vel;

    // check if past carrot pose, where no longer a thoughtfully valid command
    if (hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > carrot_dist) {
      break;
    }

    // store it for visualization
    pose_msg.pose.position.x = curr_pose.x;
    pose_msg.pose.position.y = curr_pose.y;
    pose_msg.pose.position.z = 0.01;
    arc_pts_msg.poses.push_back(pose_msg);

    // check for collision at the projected pose
    if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
      carrot_arc_pub_->publish(arc_pts_msg);
      return true;
    }
  }

  carrot_arc_pub_->publish(arc_pts_msg);

  return false;
}

bool RegulatedPurePursuitController::inCollision(
  const double & x,
  const double & y,
  const double & theta)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 30000,
      "The dimensions of the costmap is too small to successfully check for "
      "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
      "increase your costmap size.");
    return false;
  }

  double footprint_cost = collision_checker_->footprintCostAtPose(
    x, y, theta, costmap_ros_->getRobotFootprint());
  if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
  {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return footprint_cost >= static_cast<double>(LETHAL_OBSTACLE);
}

double RegulatedPurePursuitController::costAtPose(const double & x, const double & y)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_FATAL(
      logger_,
      "The dimensions of the costmap is too small to fully include your robot's footprint, "
      "thusly the robot cannot proceed further");
    throw nav2_core::PlannerException(
            "RegulatedPurePursuitController: Dimensions of the costmap are too small "
            "to encapsulate the robot footprint at current speeds!");
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

double RegulatedPurePursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path
) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void RegulatedPurePursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path,
  double & linear_vel
) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  const double & pose_cost, const nav_msgs::msg::Path & path, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel;
  double cost_vel = linear_vel;

  // limit the linear velocity by curvature
  const double radius = fabs(1.0 / curvature);
  const double & min_rad = regulated_linear_scaling_min_radius_;
  if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
    curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
  }

  // limit the linear velocity by proximity to obstacles
  if (use_cost_regulated_linear_velocity_scaling_ &&
    pose_cost != static_cast<double>(NO_INFORMATION) &&
    pose_cost != static_cast<double>(FREE_SPACE))
  {
    const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
      std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius;

    if (min_distance_to_obstacle < cost_scaling_dist_) {
      cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
    }
  }

  // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
  linear_vel = std::min(cost_vel, curvature_vel);
  linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

  applyApproachVelocityScaling(path, linear_vel);

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
  linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  rotation_command_ = 0.0;
  // ControllerServer resets its goal checker on every setPlan as well.
  reached_xy_tolerance_ = false;
  rotating_to_path_ = false;
  rotation_direction_ = 0.0;
  backup_active_ = false;
  backup_exhausted_ = false;
}

void RegulatedPurePursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    desired_linear_vel_ = base_desired_linear_vel_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      desired_linear_vel_ = speed_limit;
    }
  }
}

nav_msgs::msg::Path RegulatedPurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (local_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(local_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    local_plan_.poses.begin(), local_plan_.poses.end(), max_robot_pose_search_dist_);

  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    local_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, local_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & local_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = local_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = local_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  local_plan_.poses.erase(begin(local_plan_.poses), transformation_begin);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

double RegulatedPurePursuitController::findVelocitySignChange(
  const nav_msgs::msg::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existance of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    if ( (oa_x * ab_x) + (oa_y * ab_y) < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}

bool RegulatedPurePursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double RegulatedPurePursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

nav_msgs::msg::Path RegulatedPurePursuitController::getLocalPlan(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  local_plan_.poses.clear();
  if (global_plan_.poses.size() < 1)
  {
    RCLCPP_WARN(logger_, "Global plan is empty, cannot find goal");
    return local_plan_;
  }

  // A rolling costmap eventually excludes the original start of a long leg.
  // Prune passed points before looking for a local goal; otherwise the first
  // out-of-map point aborts the search even when the remaining path is nearby.
  auto closest = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const auto & point) {return euclidean_distance(robot_pose, point);});
  global_plan_.poses.erase(global_plan_.poses.begin(), closest);

  bool found_goal = false;
  auto goal_end = global_plan_.poses.begin();
  geometry_msgs::msg::PoseStamped goal;
  auto it = global_plan_.poses.begin();
  for (; it != global_plan_.poses.end(); ++it)
  {
    unsigned int cx, cy;
    if (!costmap_->worldToMap(it->pose.position.x, it->pose.position.y, cx, cy))
    {
      break;
    }

    if (costmap_->getCost(cx, cy) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      goal = *it;
      found_goal = true;
      goal_end = std::next(it);
    }
  }

  if (!found_goal)
  {
    RCLCPP_WARN(logger_, "No valid local goal found within local costmap!");
    return local_plan_;
  }

  // if (transformation_end == global_plan_.poses.end())
  //   goal = global_plan_.poses.back();
  // else
  //   goal = *transformation_end;
  
  // Preserve the task planner's optimized guide instead of replacing every
  // curve by a new start-to-end Theta* path. Newly blocked guides still use
  // Theta*; all existing footprint/command collision checks remain in place.
  bool supplied_safe = prefer_supplied_path_;
  std::vector<coordsW> supplied{{robot_pose.pose.position.x, robot_pose.pose.position.y}};
  unsigned int previous_x = 0, previous_y = 0;
  bool have_previous = false;
  for (auto point = global_plan_.poses.begin(); supplied_safe && point != goal_end; ++point) {
    const coordsW next{point->pose.position.x, point->pose.position.y};
    const auto previous = supplied.back();
    const int samples = std::max(1, static_cast<int>(std::ceil(
      std::hypot(next.x - previous.x, next.y - previous.y) / (costmap_->getResolution() * .5))));
    for (int j = 0; j <= samples; ++j) {
      const double t = static_cast<double>(j) / samples;
      unsigned int x, y;
      if (!costmap_->worldToMap(previous.x + t * (next.x - previous.x),
        previous.y + t * (next.y - previous.y), x, y) ||
        costmap_->getCost(x, y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        supplied_safe = false; break;
      }
      if (have_previous && x != previous_x && y != previous_y &&
        (costmap_->getCost(previous_x, y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        costmap_->getCost(x, previous_y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
      {
        supplied_safe = false; break;
      }
      previous_x = x; previous_y = y; have_previous = true;
    }
    if (std::hypot(next.x - previous.x, next.y - previous.y) > 1e-9) {
      supplied.push_back(next);
    }
  }
  if (supplied_safe) {
    local_plan_ = linearInterpolation(supplied, costmap_->getResolution());
  } else {
    planner_->setStartAndGoal(robot_pose, goal);
    getPlan(local_plan_);
  }

  if (local_plan_.poses.empty())
    return local_plan_;

  if (it == global_plan_.poses.end())
    local_plan_.poses.back().pose.orientation = global_plan_.poses.back().pose.orientation;

  local_plan_.header.stamp = clock_->now();
  local_plan_.header.frame_id = global_plan_.header.frame_id;
  return local_plan_;
}

void RegulatedPurePursuitController::getPlan(nav_msgs::msg::Path & path)
{
  std::vector<coordsW> tmp_path;
  // if (planner_->isUnsafeToPlan()) {
  //   RCLCPP_ERROR(logger_, "Either of the start or goal pose are an obstacle! ");
  //   path.poses.clear();
  // } else 
  if (planner_->generatePath(tmp_path)) {
    path = linearInterpolation(smoothLocalPath(tmp_path), planner_->costmap_->getResolution());
  } else {
    RCLCPP_ERROR(logger_, "Could not generate path between the given poses");
    path.poses.clear();
  }
}

std::vector<coordsW> RegulatedPurePursuitController::smoothLocalPath(
  const std::vector<coordsW> & path)
{
  auto safe = [&](const coordsW & a, const coordsW & b, unsigned char & peak) {
      const int count = std::max(1, static_cast<int>(std::ceil(
        std::hypot(b.x - a.x, b.y - a.y) / (costmap_->getResolution() / 3.0))));
      unsigned int px = 0, py = 0;
      bool previous = false;
      for (int i = 0; i <= count; ++i) {
        const double t = static_cast<double>(i) / count;
        unsigned int x, y;
        if (!costmap_->worldToMap(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), x, y)) {
          return false;
        }
        const auto cost = costmap_->getCost(x, y);
        if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {return false;}
        if (previous && x != px && y != py &&
          (costmap_->getCost(px, y) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
          costmap_->getCost(x, py) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
        {
          return false;
        }
        peak = std::max(peak, cost); px = x; py = y; previous = true;
      }
      return true;
    };
  auto current = path;
  for (int iteration = 0; iteration < local_path_smoothing_iterations_ && current.size() > 2;
    ++iteration)
  {
    std::vector<coordsW> result{current.front()};
    for (size_t i = 1; i + 1 < current.size(); ++i) {
      const auto & a = current[i - 1];
      const auto & b = current[i];
      const auto & c = current[i + 1];
      unsigned char original_peak = 0;
      bool rounded = false;
      if (safe(result.back(), b, original_peak) && safe(b, c, original_peak)) {
        for (double factor : {.25, .125, .0625, .03125}) {
          const coordsW entry{b.x + factor * (a.x - b.x), b.y + factor * (a.y - b.y)};
          const coordsW exit{b.x + factor * (c.x - b.x), b.y + factor * (c.y - b.y)};
          unsigned char peak = 0;
          if (safe(result.back(), entry, peak) && safe(entry, exit, peak) &&
            safe(exit, c, peak) && peak <= original_peak)
          {
            result.push_back(entry); result.push_back(exit); rounded = true; break;
          }
        }
      }
      if (!rounded) {result.push_back(b);}
    }
    result.push_back(current.back()); current = std::move(result);
  }
  return current;
}

nav_msgs::msg::Path RegulatedPurePursuitController::linearInterpolation(
  const std::vector<coordsW> & raw_path,
  const double & dist_bw_points)
{
  nav_msgs::msg::Path pa;
  if (raw_path.empty()) {
    return pa;
  }

  geometry_msgs::msg::PoseStamped p1;
  for (size_t j = 0; j + 1 < raw_path.size(); j++) {
    coordsW pt1 = raw_path[j];
    p1.pose.position.x = pt1.x;
    p1.pose.position.y = pt1.y;
    pa.poses.push_back(p1);

    coordsW pt2 = raw_path[j + 1];
    double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    if (distance < 1e-9) {
      continue;
    }
    int loops = static_cast<int>(distance / dist_bw_points);
    double sin_alpha = (pt2.y - pt1.y) / distance;
    double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      p1.pose.position.x = pt1.x + k * dist_bw_points * cos_alpha;
      p1.pose.position.y = pt1.y + k * dist_bw_points * sin_alpha;
      pa.poses.push_back(p1);
    }
  }

  // Keep a valid terminal pose even when Theta* returns a single cell. Dropping
  // it prevents the controller from finishing the final position/yaw check.
  p1.pose.position.x = raw_path.back().x;
  p1.pose.position.y = raw_path.back().y;
  pa.poses.push_back(p1);

  return pa;
}

rcl_interfaces::msg::SetParametersResult
RegulatedPurePursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // Reject invalid new settings before mutating any controller state.
  for (const auto & parameter : parameters) {
    const auto name = parameter.get_name();
    if (name == plugin_name_ + ".local_path_smoothing_iterations" &&
      (parameter.get_type() != ParameterType::PARAMETER_INTEGER ||
      parameter.as_int() < 0 || parameter.as_int() > 5))
    {
      result.successful = false;
      result.reason = "local_path_smoothing_iterations must be between 0 and 5";
      return result;
    }
    if (name == plugin_name_ + ".rotation_accel_gain" ||
      name == plugin_name_ + ".rotation_damping_gain" ||
      name == plugin_name_ + ".tracking_angular_vel_limit")
    {
      if (parameter.get_type() != ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(parameter.as_double()) || parameter.as_double() < 0.0 ||
        (name != plugin_name_ + ".rotation_damping_gain" && parameter.as_double() == 0.0))
      {
        result.successful = false;
        result.reason = "Invalid heading control gain or tracking angular velocity limit";
        return result;
      }
    }
    if (name == plugin_name_ + ".backup_recovery_trigger_cost" ||
      name == plugin_name_ + ".backup_recovery_distance" ||
      name == plugin_name_ + ".backup_recovery_speed")
    {
      if (parameter.get_type() != ParameterType::PARAMETER_DOUBLE ||
        !std::isfinite(parameter.as_double()) || parameter.as_double() <= 0.0 ||
        (name == plugin_name_ + ".backup_recovery_trigger_cost" &&
        parameter.as_double() >= INSCRIBED_INFLATED_OBSTACLE))
      {
        result.successful = false;
        result.reason = "Invalid backup recovery parameter";
        return result;
      }
    }
    if (parameter.get_name() != plugin_name_ + ".rotate_to_heading_hysteresis" &&
      parameter.get_name() != plugin_name_ + ".rotate_to_heading_direction_lock_angle") {continue;}
    if (parameter.get_type() != ParameterType::PARAMETER_DOUBLE ||
      !std::isfinite(parameter.as_double()) || parameter.as_double() < 0.0 ||
      (parameter.get_name() == plugin_name_ + ".rotate_to_heading_direction_lock_angle" &&
      parameter.as_double() >= M_PI_2))
    {
      result.successful = false;
      result.reason = "Invalid heading hysteresis or direction lock angle";
      return result;
    }
  }

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (name == plugin_name_ + ".local_path_smoothing_iterations") {
      local_path_smoothing_iterations_ = static_cast<int>(parameter.as_int());
    }
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".inflation_cost_scaling_factor") {
        if (parameter.as_double() <= 0.0) {
          RCLCPP_WARN(
            logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
            "it should be >0. Ignoring parameter update.");
          continue;
        }
        inflation_cost_scaling_factor_ = parameter.as_double();
      } else if (name == plugin_name_ + ".desired_linear_vel") {
        desired_linear_vel_ = parameter.as_double();
        base_desired_linear_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_angular_vel") {
        rotate_to_heading_angular_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot") {
        max_allowed_time_to_collision_up_to_carrot_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cost_scaling_dist") {
        cost_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cost_scaling_gain") {
        cost_scaling_gain_ = parameter.as_double();
      } else if (name == plugin_name_ + ".regulated_linear_scaling_min_radius") {
        regulated_linear_scaling_min_radius_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".regulated_linear_scaling_min_speed") {
        regulated_linear_scaling_min_speed_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_angular_accel") {
        max_angular_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_accel_gain") {
        rotation_accel_gain_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_damping_gain") {
        rotation_damping_gain_ = parameter.as_double();
      } else if (name == plugin_name_ + ".tracking_angular_vel_limit") {
        tracking_angular_vel_limit_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_min_angle") {
        rotate_to_heading_min_angle_ = parameter.as_double();
      } else if (name == plugin_name_ + ".backup_recovery_trigger_cost") {
        backup_recovery_trigger_cost_ = parameter.as_double();
      } else if (name == plugin_name_ + ".backup_recovery_distance") {
        backup_recovery_distance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".backup_recovery_speed") {
        backup_recovery_speed_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_hysteresis") {
        rotate_to_heading_hysteresis_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_direction_lock_angle") {
        rotate_to_heading_direction_lock_angle_ = parameter.as_double();
        rotation_direction_ = 0.0;
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".prefer_supplied_path") {
        prefer_supplied_path_ = parameter.as_bool();
      }
      if (name == plugin_name_ + ".use_backup_recovery") {
        use_backup_recovery_ = parameter.as_bool();
        backup_active_ = false;
        backup_exhausted_ = false;
      } else if (name == plugin_name_ + ".use_rotate_to_goal_heading") {
        use_rotate_to_goal_heading_ = parameter.as_bool();
        reached_xy_tolerance_ = false;
      } else if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
        reached_xy_tolerance_ = false;
        rotating_to_path_ = false;
        rotation_direction_ = 0.0;
      } else if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_regulated_linear_velocity_scaling") {
        use_regulated_linear_velocity_scaling_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_cost_regulated_linear_velocity_scaling") {
        use_cost_regulated_linear_velocity_scaling_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        if (parameter.as_bool() && allow_reversing_) {
          RCLCPP_WARN(
            logger_, "Both use_rotate_to_heading and allow_reversing "
            "parameter cannot be set to true. Rejecting parameter update.");
          continue;
        }
        use_rotate_to_heading_ = parameter.as_bool();
        rotating_to_path_ = false;
        rotation_direction_ = 0.0;
      } else if (name == plugin_name_ + ".allow_reversing") {
        if (use_rotate_to_heading_ && parameter.as_bool()) {
          RCLCPP_WARN(
            logger_, "Both use_rotate_to_heading and allow_reversing "
            "parameter cannot be set to true. Rejecting parameter update.");
          continue;
        }
        allow_reversing_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_regulated_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController,
  nav2_core::Controller)
