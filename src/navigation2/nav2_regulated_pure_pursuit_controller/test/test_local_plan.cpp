// Copyright 2026 outdoor_nav contributors
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

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

class LocalPlanController :
  public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  explicit LocalPlanController(nav2_costmap_2d::Costmap2D * costmap)
  {
    costmap_ = costmap;
    planner_ = std::make_unique<theta_star::ThetaStar>();
    planner_->costmap_ = costmap;
    clock_ = std::make_shared<rclcpp::Clock>();
  }

  using RegulatedPurePursuitController::getLocalPlan;
  using RegulatedPurePursuitController::smoothLocalPath;
  using RegulatedPurePursuitController::linearInterpolation;
  using RegulatedPurePursuitController::shouldRotateToGoalHeading;
  using RegulatedPurePursuitController::shouldRotateToPath;
  using RegulatedPurePursuitController::rotateToHeading;
  using RegulatedPurePursuitController::dynamicParametersCallback;
  using RegulatedPurePursuitController::computeBackupCommand;

  bool isBackupPoseSafe(double x, double y, double yaw) override
  {
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap_);
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = .4; footprint[0].y = .27;
    footprint[1].x = .4; footprint[1].y = -.27;
    footprint[2].x = -.4; footprint[2].y = -.27;
    footprint[3].x = -.4; footprint[3].y = .27;
    return checker.footprintCostAtPose(x, y, yaw, footprint) < 253;
  }

  bool isRetainedPathPoseSafe(double x, double y, double heading) override
  {
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap_);
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = .4; footprint[0].y = .27;
    footprint[1].x = .4; footprint[1].y = -.27;
    footprint[2].x = -.4; footprint[2].y = -.27;
    footprint[3].x = -.4; footprint[3].y = .27;
    return checker.footprintCostAtPose(x, y, heading, footprint) < 254;
  }
  std::vector<geometry_msgs::msg::Point> localGoalFootprint() const override
  {
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = .4; footprint[0].y = .27;
    footprint[1].x = .4; footprint[1].y = -.27;
    footprint[2].x = -.4; footprint[2].y = -.27;
    footprint[3].x = -.4; footprint[3].y = .27;
    return footprint;
  }
  uint64_t selectedRouteId() const {return path_hysteresis_.currentId();}
  double traversalWeight() const {return planner_->w_traversal_cost_;}
  double lengthWeight() const {return planner_->w_euc_cost_;}
  void configurePathSelection(bool enabled, double interval = 2., bool obstacle_only = false)
  {
    path_hysteresis_.enabled = enabled;
    path_hysteresis_.min_switch_interval = interval;
    path_hysteresis_.switch_only_on_obstacle = obstacle_only;
  }

  void configureHeading(bool stateful = true)
  {
    use_rotate_to_heading_ = true;
    goal_dist_tol_ = 0.5;
    stateful_ = stateful;
    rotate_to_heading_min_angle_ = M_PI/4;
    rotate_to_heading_hysteresis_ = M_PI/12;
    rotate_to_heading_angular_vel_ = 0.8;
    max_angular_accel_ = 3.2;
    control_duration_ = 0.05;
    plugin_name_ = "FollowPath";
  }
};

TEST(LocalPlanRegression, PassedStartOutsideRollingMapDoesNotDiscardForwardPath)
{
  nav2_costmap_2d::Costmap2D costmap(120, 120, 0.1, 4.0, -6.0, 0);
  LocalPlanController controller(&costmap);
  nav_msgs::msg::Path global;
  global.header.frame_id = "map";
  for (int x = 0; x <= 20; ++x) {
    geometry_msgs::msg::PoseStamped point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    global.poses.push_back(point);
  }
  controller.setPlan(global);
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.position.x = 6.0;
  auto local = controller.getLocalPlan(robot);
  ASSERT_FALSE(local.poses.empty());
  EXPECT_GT(local.poses.back().pose.position.x, robot.pose.position.x);
  EXPECT_LT(local.poses.back().pose.position.x, 16.0);
}

TEST(LocalPlanRegression, LongClearGuideKeepsFootprintInsideRollingMap)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  controller.configurePathSelection(true, 2., true);
  nav_msgs::msg::Path global;
  global.header.frame_id = "map";
  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped point;
    point.pose.position.x = 5.0 + i * 0.25;
    point.pose.position.y = 5.0;
    point.pose.orientation.w = 1.0;
    global.poses.push_back(point);
  }
  controller.setPlan(global);
  auto local = controller.getLocalPlan(global.poses.front());
  ASSERT_FALSE(local.poses.empty());
  EXPECT_GT(local.poses.back().pose.position.x, 8.0);
  EXPECT_LE(local.poses.back().pose.position.x, 9.5);
  EXPECT_EQ(controller.selectedRouteId(), 1u);
}

TEST(LocalPlanRegression, BlockedCostmapStillRejectsEveryLocalGoal)
{
  nav2_costmap_2d::Costmap2D costmap(120, 120, 0.1, 4.0, -6.0, 254);
  LocalPlanController controller(&costmap);
  nav_msgs::msg::Path global;
  geometry_msgs::msg::PoseStamped robot;
  robot.pose.position.x = 6.0;
  global.poses.push_back(robot);
  controller.setPlan(global);
  EXPECT_TRUE(controller.getLocalPlan(robot).poses.empty());
}

TEST(LocalPlanRegression, SafeSuppliedCurveIsPreservedInsteadOfReplacedByChord)
{
  nav2_costmap_2d::Costmap2D costmap(50, 50, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  nav_msgs::msg::Path global;
  global.header.frame_id = "map";
  for (const auto & xy : std::vector<coordsW>{{.5, .5}, {.5, 2.}, {2., 2.}}) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = xy.x; p.pose.position.y = xy.y; p.pose.orientation.w = 1.;
    global.poses.push_back(p);
  }
  controller.setPlan(global);
  auto path = controller.getLocalPlan(global.poses.front());
  ASSERT_GT(path.poses.size(), 2u);
  EXPECT_NEAR(path.poses[1].pose.position.x, .5, 1e-6);
  EXPECT_GT(path.poses[1].pose.position.y, .5);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 2.);
}

TEST(LocalPlanRegression, NewlyBlockedSuppliedCurveTriggersLocalAvoidance)
{
  nav2_costmap_2d::Costmap2D costmap(50, 50, 0.1, 0.0, 0.0, 0);
  costmap.setCost(5, 15, 254);
  LocalPlanController controller(&costmap);
  nav_msgs::msg::Path global;
  global.header.frame_id = "map";
  for (const auto & xy : std::vector<coordsW>{{.55, .55}, {.55, 2.05}, {2.05, 2.05}}) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = xy.x; p.pose.position.y = xy.y; p.pose.orientation.w = 1.;
    global.poses.push_back(p);
  }
  controller.setPlan(global);
  auto path = controller.getLocalPlan(global.poses.front());
  ASSERT_FALSE(path.poses.empty());
  for (const auto & p : path.poses) {
    unsigned int x, y;
    ASSERT_TRUE(costmap.worldToMap(p.pose.position.x, p.pose.position.y, x, y));
    EXPECT_LT(costmap.getCost(x, y), 253);
  }
}

TEST(LocalPlanRegression, InterpolationHandlesEmptyAndSingleCellPaths)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  EXPECT_TRUE(controller.linearInterpolation({}, 0.1).poses.empty());
  auto path = controller.linearInterpolation({{1.0, 2.0}}, 0.1);
  ASSERT_EQ(path.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, 2.0);
}

TEST(LocalPlanRegression, InterpolationPreservesFinalEndpoint)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  auto path = controller.linearInterpolation({{0.0, 0.0}, {0.0, 0.0}, {1.0, 2.0}}, 0.1);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, 2.0);
}

TEST(HeadingModeRegression, PositionNoiseDoesNotLeaveFinalHeadingMode)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  controller.configureHeading();
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 0.5043;
  EXPECT_FALSE(controller.shouldRotateToGoalHeading(goal));
  for (double distance : {0.4978, 0.5061, 0.5040, 0.4971, 0.5014, 0.4840, 0.5053}) {
    goal.pose.position.x = distance;
    EXPECT_TRUE(controller.shouldRotateToGoalHeading(goal)) << distance;
  }
}

TEST(HeadingModeRegression, NewPlanResetsPositionArrivalState)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  controller.configureHeading();
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 0.49;
  ASSERT_TRUE(controller.shouldRotateToGoalHeading(goal));
  nav_msgs::msg::Path path;
  goal.pose.position.x = 10.0;
  path.poses.push_back(goal);
  controller.setPlan(path);
  EXPECT_FALSE(controller.shouldRotateToGoalHeading(goal));
}

TEST(HeadingModeRegression, NonStatefulModeRechecksPosition)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  controller.configureHeading(false);
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 0.49;
  EXPECT_TRUE(controller.shouldRotateToGoalHeading(goal));
  goal.pose.position.x = 0.51;
  EXPECT_FALSE(controller.shouldRotateToGoalHeading(goal));
}


TEST(HeadingModeRegression, PathRotationUsesDifferentEntryAndExitThresholds)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  controller.configureHeading();
  auto check = [&controller](double degrees) {
      geometry_msgs::msg::PoseStamped carrot;
      carrot.pose.position.x = std::cos(degrees*M_PI/180);
      carrot.pose.position.y = std::sin(degrees*M_PI/180);
      double error;
      return controller.shouldRotateToPath(carrot, error);
    };
  EXPECT_FALSE(check(44.0));
  EXPECT_TRUE(check(46.0));
  for (double degrees : {44.0, 46.0, 43.0, 45.0, 31.0}) {
    EXPECT_TRUE(check(degrees)) << degrees;
  }
  EXPECT_FALSE(check(29.0));
  EXPECT_FALSE(check(40.0));
  EXPECT_TRUE(check(-46.0));
  EXPECT_TRUE(check(-31.0));
  EXPECT_FALSE(check(-29.0));
}

TEST(HeadingModeRegression, DirectionDoesNotFlipAcrossPiInEitherDirection)
{
  for (double direction : {-1.0, 1.0}) {
    nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
    LocalPlanController controller(&costmap);
    controller.configureHeading();
    geometry_msgs::msg::Twist stopped;
    double linear, angular;
    for (double degrees : {179.0, -179.0, 178.0, -178.0, 175.0}) {
      controller.rotateToHeading(linear, angular, direction*degrees*M_PI/180, stopped);
      EXPECT_DOUBLE_EQ(linear, 0.0);
      EXPECT_GT(direction*angular, 0.0) << degrees;
    }
    // A genuinely different direction beyond the ambiguity band is respected.
    // Decelerate the previous turn before reversing; never jump the command.
    for (int cycle = 0; cycle < 20; ++cycle) {
      controller.rotateToHeading(linear, angular, -direction*160*M_PI/180, stopped);
    }
    EXPECT_LT(direction*angular, 0.0);
  }
}

TEST(HeadingModeRegression, NoisyBodyYawRateCannotReverseRequestedHeadingTurn)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  controller.configureHeading();
  geometry_msgs::msg::Twist measured;
  double linear, angular, previous = 0.0;
  for (int cycle = 0; cycle < 100; ++cycle) {
    measured.angular.z = cycle % 2 ? -3.0 : 3.0;
    controller.rotateToHeading(linear, angular, 2.0, measured);
    EXPECT_GE(angular, 0.0);
    EXPECT_LE(std::abs(angular-previous), 3.2*0.05+1e-9);
    EXPECT_LE(std::abs(angular), 0.8);
    previous = angular;
  }
}

TEST(HeadingModeRegression, NewPlanResetsPathRotationAndDirection)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  controller.configureHeading();
  geometry_msgs::msg::PoseStamped carrot;
  carrot.pose.position.x = -1.0;
  carrot.pose.position.y = 0.01;
  double error, linear, angular;
  ASSERT_TRUE(controller.shouldRotateToPath(carrot, error));
  controller.rotateToHeading(linear, angular, error, geometry_msgs::msg::Twist());
  ASSERT_GT(angular, 0.0);
  controller.setPlan(nav_msgs::msg::Path());
  carrot.pose.position.x = std::cos(40*M_PI/180);
  carrot.pose.position.y = std::sin(40*M_PI/180);
  EXPECT_FALSE(controller.shouldRotateToPath(carrot, error));
  controller.rotateToHeading(linear, angular, -179*M_PI/180, geometry_msgs::msg::Twist());
  EXPECT_LT(angular, 0.0);
}

TEST(HeadingModeRegression, TrackingReleasesDirectionForNextTurn)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  controller.configureHeading();
  double linear, angular, error;
  controller.rotateToHeading(linear, angular, 179*M_PI/180, geometry_msgs::msg::Twist());
  geometry_msgs::msg::PoseStamped carrot;
  carrot.pose.position.x = 1.0;
  EXPECT_FALSE(controller.shouldRotateToPath(carrot, error));
  controller.rotateToHeading(linear, angular, -179*M_PI/180, geometry_msgs::msg::Twist());
  EXPECT_LT(angular, 0.0);
}

TEST(HeadingModeRegression, InvalidLockAndHysteresisParametersAreRejected)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, 0);
  LocalPlanController controller(&costmap);
  controller.configureHeading();
  for (double value : {-0.1, M_PI_2, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_FALSE(controller.dynamicParametersCallback({rclcpp::Parameter(
        "FollowPath.rotate_to_heading_direction_lock_angle", value)}).successful);
  }
  EXPECT_FALSE(controller.dynamicParametersCallback({rclcpp::Parameter(
      "FollowPath.rotate_to_heading_hysteresis", -0.1)}).successful);
  EXPECT_TRUE(controller.dynamicParametersCallback({
      rclcpp::Parameter("FollowPath.rotate_to_heading_hysteresis", 0.2),
      rclcpp::Parameter("FollowPath.rotate_to_heading_direction_lock_angle", 0.1)}).successful);
}


static geometry_msgs::msg::PoseStamped backupPose(double x = 2.0, double y = 2.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x; pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

static void inflatedStart(nav2_costmap_2d::Costmap2D & map)
{
  for (unsigned int x = 36; x <= 46; ++x) {
    for (unsigned int y = 26; y <= 54; ++y) {map.setCost(x, y, 220);}
  }
}

TEST(BackupRecoveryRegression, HighCostStartUsesCheckedReverseBeforePlanning)
{
  nav2_costmap_2d::Costmap2D map(80, 80, .05, 0, 0, 0);
  inflatedStart(map);
  LocalPlanController controller(&map);
  double linear, angular;
  EXPECT_TRUE(controller.computeBackupCommand(backupPose(), linear, angular));
  EXPECT_LT(linear, 0.0);
  EXPECT_DOUBLE_EQ(angular, 0.0);
  EXPECT_FALSE(controller.computeBackupCommand(backupPose(1.7), linear, angular));
  // One bounded recovery per submitted path, rather than endless reversing.
  EXPECT_FALSE(controller.computeBackupCommand(backupPose(), linear, angular));
  controller.setPlan(nav_msgs::msg::Path());
  EXPECT_TRUE(controller.computeBackupCommand(backupPose(), linear, angular));
}

TEST(BackupRecoveryRegression, OrdinaryInflationDoesNotTriggerReverse)
{
  nav2_costmap_2d::Costmap2D map(80, 80, .05, 0, 0, 100);
  LocalPlanController controller(&map);
  double linear, angular;
  EXPECT_FALSE(controller.computeBackupCommand(backupPose(), linear, angular));
}

TEST(BackupRecoveryRegression, RearObstacleInsideSweptFootprintBlocksRecovery)
{
  nav2_costmap_2d::Costmap2D map(80, 80, .05, 0, 0, 0);
  inflatedStart(map);
  // Current footprint starts at x=1.6; reversing would sweep over x=1.5.
  for (unsigned int y = 26; y <= 54; ++y) {map.setCost(30, y, 254);}
  LocalPlanController controller(&map);
  double linear, angular;
  EXPECT_THROW(controller.computeBackupCommand(backupPose(), linear, angular), nav2_core::PlannerException);
}

TEST(BackupRecoveryRegression, CurrentCollisionAndUnknownSpaceNeverAuthorizeReverse)
{
  for (unsigned char cost : {253, 254, 255}) {
    nav2_costmap_2d::Costmap2D map(80, 80, .05, 0, 0, 0);
    map.setCost(40, 40, cost);
    LocalPlanController controller(&map);
    double linear, angular;
    EXPECT_THROW(controller.computeBackupCommand(backupPose(), linear, angular), nav2_core::PlannerException);
  }
}

TEST(BackupRecoveryRegression, NewObstacleStopsAnActiveRecovery)
{
  nav2_costmap_2d::Costmap2D map(80, 80, .05, 0, 0, 0);
  inflatedStart(map);
  LocalPlanController controller(&map);
  double linear, angular;
  ASSERT_TRUE(controller.computeBackupCommand(backupPose(), linear, angular));
  for (unsigned int y = 26; y <= 54; ++y) {map.setCost(30, y, 254);}
  EXPECT_THROW(controller.computeBackupCommand(backupPose(), linear, angular), nav2_core::PlannerException);
}

TEST(BackupRecoveryRegression, NoFreeEndpointWithinLimitDoesNotReverse)
{
  nav2_costmap_2d::Costmap2D map(80, 80, .05, 0, 0, 220);
  LocalPlanController controller(&map);
  double linear, angular;
  EXPECT_THROW(controller.computeBackupCommand(backupPose(), linear, angular), nav2_core::PlannerException);
}

TEST(LocalPathSmoothing, TightCornerDoesNotKeepUnnecessaryCornersElsewhere)
{
  nav2_costmap_2d::Costmap2D costmap(70, 70, 0.1, 0., 0., 0);
  for (unsigned int y = 6; y < 20; ++y) {
    for (unsigned int x = 11; x < 20; ++x) {costmap.setCost(x, y, 254);}
  }
  LocalPlanController controller(&costmap);
  const std::vector<coordsW> guide{
    {1.05, .55}, {1.05, 2.05}, {3.05, 2.05}, {3.05, 4.05}, {5.05, 4.05}};
  auto path = controller.smoothLocalPath(guide);
  ASSERT_GT(path.size(), guide.size());
  EXPECT_DOUBLE_EQ(path.front().x, guide.front().x);
  EXPECT_DOUBLE_EQ(path.back().x, guide.back().x);
  double nearest = 100.;
  for (const auto & p : path) {
    nearest = std::min(nearest, std::hypot(p.x - guide[2].x, p.y - guide[2].y));
  }
  EXPECT_GT(nearest, .15);
  for (size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1];
    const auto & b = path[i];
    const int count = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / .02)));
    for (int j = 0; j <= count; ++j) {
      const double t = static_cast<double>(j) / count;
      unsigned int x, y;
      ASSERT_TRUE(costmap.worldToMap(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), x, y));
      EXPECT_LT(costmap.getCost(x, y), 253);
    }
  }
}

TEST(LocalPathSmoothing, IterationsAreValidatedAndZeroKeepsOriginalPolyline)
{
  nav2_costmap_2d::Costmap2D costmap(50, 50, .1, 0., 0., 0);
  LocalPlanController controller(&costmap);
  controller.configureHeading();
  EXPECT_FALSE(controller.dynamicParametersCallback({
    rclcpp::Parameter("FollowPath.local_path_smoothing_iterations", -1)}).successful);
  EXPECT_TRUE(controller.dynamicParametersCallback({
    rclcpp::Parameter("FollowPath.local_path_smoothing_iterations", 0)}).successful);
  const std::vector<coordsW> guide{{.5, .5}, {.5, 2.}, {2., 2.}};
  auto path = controller.smoothLocalPath(guide);
  ASSERT_EQ(path.size(), guide.size());
  EXPECT_DOUBLE_EQ(path[1].x, guide[1].x);
  EXPECT_DOUBLE_EQ(path[1].y, guide[1].y);
}

TEST(PathSelectionIntegration, RepeatedThetaPlansAroundAWallKeepOneRoute)
{
  nav2_costmap_2d::Costmap2D map(100, 80, .1, 0., 0., 0);
  for (unsigned int x = 41; x <= 59; ++x) {
    for (unsigned int y = 16; y <= 64; ++y) {map.setCost(x, y, 253);}
  }
  for (unsigned int x = 45; x <= 55; ++x) {
    for (unsigned int y = 20; y <= 60; ++y) {map.setCost(x, y, 254);}
  }
  LocalPlanController controller(&map);
  LocalPlanController baseline(&map);
  baseline.configurePathSelection(false, 0.);
  nav_msgs::msg::Path global; global.header.frame_id = "map";
  for (int i = 0; i <= 80; ++i) {
    auto p = backupPose(1.+i*.1, 4.); global.poses.push_back(p);
  }
  controller.setPlan(global);
  baseline.setPlan(global);
  int previous_side = 0, baseline_switches = 0;
  for (int cycle = 0; cycle < 16; ++cycle) {
    for (unsigned int x = 0; x < 100; ++x) {
      for (unsigned int y = 0; y < 80; ++y) {
        if (map.getCost(x, y) < 253) {map.setCost(x, y, (cycle%2 == (y>40)) ? 50 : 0);}
      }
    }
    const auto selected = controller.getLocalPlan(global.poses.front());
    ASSERT_FALSE(selected.poses.empty());
    EXPECT_EQ(controller.selectedRouteId(), 1u);
    const auto original = baseline.getLocalPlan(global.poses.front());
    ASSERT_FALSE(original.poses.empty());
    double deviation = 0.;
    for (const auto & p : original.poses) {deviation += p.pose.position.y-4.;}
    const int side = deviation > 0. ? 1 : -1;
    if (previous_side && side != previous_side) {++baseline_switches;}
    previous_side = side;
  }
  EXPECT_GT(baseline_switches, 0);
}

TEST(PathSelectionParameters, InvalidValuesRejectAtomicallyAndDisableClearsSelection)
{
  nav2_costmap_2d::Costmap2D map(50, 50, .1, 0., 0., 0);
  LocalPlanController controller(&map); controller.configureHeading();
  nav_msgs::msg::Path guide; guide.header.frame_id = "map";
  guide.poses = {backupPose(1., 1.), backupPose(3., 1.)};
  controller.setPlan(guide); controller.getLocalPlan(guide.poses.front());
  ASSERT_NE(controller.selectedRouteId(), 0u);
  for (const auto & key : {"switch_penalty", "min_switch_interval"}) {
    for (double bad : {-1., std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
    {
      EXPECT_FALSE(controller.dynamicParametersCallback({
        rclcpp::Parameter("FollowPath.enable_path_hysteresis", false),
        rclcpp::Parameter(std::string("FollowPath.")+key, bad)}).successful);
      EXPECT_NE(controller.selectedRouteId(), 0u);
    }
    EXPECT_FALSE(controller.dynamicParametersCallback({
      rclcpp::Parameter(std::string("FollowPath.")+key, 1)}).successful);
  }
  EXPECT_FALSE(controller.dynamicParametersCallback({
    rclcpp::Parameter("FollowPath.log_path_switch", 1)}).successful);
  EXPECT_TRUE(controller.dynamicParametersCallback({
    rclcpp::Parameter("FollowPath.enable_path_hysteresis", false),
    rclcpp::Parameter("FollowPath.log_path_switch", true)}).successful);
  EXPECT_EQ(controller.selectedRouteId(), 0u);
}

TEST(LocalPlanRegression, ThetaWeightsUpdatePlannerAndRejectInvalidBatch)
{
  nav2_costmap_2d::Costmap2D costmap(120, 120, 0.1, -6.0, -6.0, 0);
  LocalPlanController controller(&costmap);
  controller.configureHeading();
  EXPECT_TRUE(controller.dynamicParametersCallback({
    rclcpp::Parameter("FollowPath.w_traversal_cost", 12.0),
    rclcpp::Parameter("FollowPath.w_euc_cost", 0.75)}).successful);
  EXPECT_DOUBLE_EQ(controller.traversalWeight(), 12.0);
  EXPECT_DOUBLE_EQ(controller.lengthWeight(), 0.75);
  for (double bad : {-1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_FALSE(controller.dynamicParametersCallback({
      rclcpp::Parameter("FollowPath.w_traversal_cost", bad)}).successful);
  }
  EXPECT_FALSE(controller.dynamicParametersCallback({
    rclcpp::Parameter("FollowPath.w_traversal_cost", 30.0),
    rclcpp::Parameter("FollowPath.w_euc_cost", 0.0)}).successful);
  EXPECT_DOUBLE_EQ(controller.traversalWeight(), 12.0);
  EXPECT_DOUBLE_EQ(controller.lengthWeight(), 0.75);
  EXPECT_FALSE(controller.dynamicParametersCallback({
    rclcpp::Parameter("FollowPath.w_euc_cost", 1)}).successful);
  EXPECT_TRUE(controller.dynamicParametersCallback({
    rclcpp::Parameter("FollowPath.w_traversal_cost", 0.0)}).successful);
  EXPECT_DOUBLE_EQ(controller.traversalWeight(), 0.0);
}
