// Copyright 2026 outdoor_nav contributors
// Licensed under the Apache License, Version 2.0.
#include <cmath>
#include <limits>
#include <iostream>
#include "gtest/gtest.h"
#include "nav2_regulated_pure_pursuit_controller/path_hysteresis.hpp"

using nav2_regulated_pure_pursuit_controller::PathHysteresis;
using Path = nav_msgs::msg::Path;
using Pose = geometry_msgs::msg::PoseStamped;

static Pose pose(double x, double y)
{
  Pose p; p.pose.position.x = x; p.pose.position.y = y; p.pose.orientation.w = 1.; return p;
}
static Path route(double y)
{
  Path p; p.header.frame_id = "map";
  p.poses = {pose(1., 4.), pose(3.5, y), pose(6.5, y), pose(9., 4.)}; return p;
}
class PathHysteresisTest : public ::testing::Test
{
protected:
  nav2_costmap_2d::Costmap2D map{100, 80, .1, 0., 0., 0};
  PathHysteresis selector;
  void SetUp() override
  {
    for (unsigned int x = 45; x <= 55; ++x) {
      for (unsigned int y = 25; y <= 55; ++y) {map.setCost(x, y, 254);}
    }
    selector.setGoal(route(6.5));
  }
  Path choose(const Path & path, double now, Pose robot = pose(1., 4.))
  {return selector.select(path, robot, map, now, 2., 1.);}
  static double extreme(const Path & path)
  {
    double value = 4.;
    for (const auto & p : path.poses) {
      if (std::abs(p.pose.position.y-4.) > std::abs(value-4.)) {value = p.pose.position.y;}
    }
    return value;
  }
};

TEST_F(PathHysteresisTest, EqualLeftRightCostsDoNotAlternate)
{
  choose(route(6.5), 0.);
  const auto id = selector.currentId();
  for (int i = 1; i < 80; ++i) {
    const auto selected = choose(route(i % 2 ? 1.5 : 6.5), .1*i);
    EXPECT_GT(extreme(selected), 4.);
    EXPECT_EQ(selector.currentId(), id);
  }
}

TEST_F(PathHysteresisTest, SlightlyBetterRouteIsHeldAfterTimeLockExpires)
{
  choose(route(6.5), 0.);
  choose(route(1.55), 3.);
  const auto d = selector.decision();
  EXPECT_LT(d.candidate_cost, d.current_cost);
  EXPECT_GT(d.candidate_cost+d.candidate_valid*selector.switch_penalty, d.current_cost);
  EXPECT_EQ(d.reason, "switch_penalty"); EXPECT_FALSE(d.switched);
  std::cout << "calibration near: J_current=" << d.current_cost << " J_new=" << d.candidate_cost <<
    " penalty=" << selector.switch_penalty << std::endl;
}

TEST_F(PathHysteresisTest, ClearImprovementRequiresBothPenaltyAndTime)
{
  choose(route(7.5), 0.);
  EXPECT_GT(extreme(choose(route(2.), 1.)), 4.);
  EXPECT_EQ(selector.decision().reason, "min_switch_interval");
  EXPECT_LT(extreme(choose(route(2.), 2.1)), 4.);
  EXPECT_EQ(selector.decision().reason, "cost_improved");
  EXPECT_TRUE(selector.decision().switched);
  EXPECT_EQ(selector.currentId(), 2u);
  // A hold on the same route must not postpone a future legitimate switch.
  choose(route(2.), 3.);
  EXPECT_EQ(selector.currentId(), 2u);
}

TEST_F(PathHysteresisTest, CollisionBypassesBothVeryLargePenaltyAndTimeLock)
{
  selector.switch_penalty = 1e6; selector.min_switch_interval = 100.;
  choose(route(6.5), 0.);
  map.setCost(50, 65, 254);
  const auto path = choose(route(1.5), .01);
  EXPECT_LT(extreme(path), 4.);
  EXPECT_FALSE(selector.decision().current_valid);
  EXPECT_EQ(selector.decision().reason, "current_invalid");
  EXPECT_TRUE(selector.decision().switched);
}

TEST_F(PathHysteresisTest, ObstacleOnlyModeHoldsClearRouteRegardlessOfCostAndTime)
{
  selector.switch_only_on_obstacle = true;
  choose(route(6.5), 0.);
  const auto id = selector.currentId();
  for (int i = 1; i <= 20; ++i) {
    const auto selected = choose(route(i % 2 ? 1.5 : 7.5), i * .1);
    EXPECT_GT(extreme(selected), 4.);
    EXPECT_EQ(selector.currentId(), id);
    EXPECT_FALSE(selector.decision().switched);
  }
}

TEST_F(PathHysteresisTest, ObstacleOnlyModeSwitchesWhenRetainedRouteIsBlocked)
{
  selector.switch_only_on_obstacle = true;
  choose(route(6.5), 0.);
  map.setCost(50, 65, 254);
  const auto selected = choose(route(1.5), .01);
  EXPECT_LT(extreme(selected), 4.);
  EXPECT_EQ(selector.decision().reason, "current_blocked");
  EXPECT_TRUE(selector.decision().switched);
}

TEST_F(PathHysteresisTest, ObstacleOnlyModeClipsOldHorizonAtRollingMapEdge)
{
  selector.switch_only_on_obstacle = true;
  choose(route(6.5), 0.);
  map.updateOrigin(-4., 0.);
  Path alternate;
  alternate.poses = {pose(1., 4.), pose(3., 1.5), pose(5., 1.5)};
  const auto selected = choose(alternate, .1);
  EXPECT_TRUE(selector.decision().current_valid);
  EXPECT_FALSE(selector.decision().switched);
  EXPECT_GT(extreme(selected), 4.);
  EXPECT_LT(selected.poses.back().pose.position.x, 6.);
}

TEST_F(PathHysteresisTest, ObstacleOnlyModeExtendsCompletedLocalHorizon)
{
  selector.switch_only_on_obstacle = true;
  Path first;
  first.poses = {pose(1., 6.5), pose(3., 6.5)};
  selector.select(first, first.poses.front(), map, 0., 2., 1.);
  Path next;
  next.poses = {pose(2.9, 6.5), pose(4., 6.5)};
  const auto selected = choose(next, .1, pose(2.9, 6.5));
  EXPECT_FALSE(selected.poses.empty());
  EXPECT_EQ(selector.decision().reason, "horizon_continue");
  EXPECT_FALSE(selector.decision().switched);
}

TEST_F(PathHysteresisTest, FootprintFailureAlsoBypassesLock)
{
  choose(route(6.5), 0.);
  auto path = selector.select(route(1.5), pose(1., 4.), map, .01, 2., 1.,
      [](double, double y, double) {return y < 6.0;});
  EXPECT_LT(extreme(path), 4.);
  EXPECT_FALSE(selector.decision().current_valid);
}

TEST_F(PathHysteresisTest, PlannerFailureNeverResurrectsAnOldRoute)
{
  choose(route(6.5), 0.);
  EXPECT_TRUE(choose(Path{}, .01).poses.empty());
  EXPECT_EQ(selector.currentId(), 0u);
  EXPECT_EQ(selector.decision().reason, "planner_failed");
}

TEST_F(PathHysteresisTest, ChangedGoalFrameAndOrientationResetState)
{
  choose(route(6.5), 0.);
  Path goal = route(1.5); goal.poses.back().pose.position.x = 8.;
  selector.setGoal(goal); EXPECT_EQ(selector.currentId(), 0u);
  choose(goal, .01); EXPECT_EQ(selector.decision().reason, "initial");
  goal.header.frame_id = "other_map";
  selector.setGoal(goal); EXPECT_EQ(selector.currentId(), 0u);
  choose(goal, .02);
  goal.poses.back().pose.orientation.z = std::sin(.5);
  goal.poses.back().pose.orientation.w = std::cos(.5);
  selector.setGoal(goal); EXPECT_EQ(selector.currentId(), 0u);
}

TEST_F(PathHysteresisTest, SameRouteWithJitterAndResamplingKeepsIdAndOriginalTimer)
{
  choose(route(7.5), 0.);
  Path jitter = route(7.52);
  jitter.poses.insert(jitter.poses.begin()+2, pose(5., 7.52));
  auto original = selector.currentId();
  selector.setGoal(jitter); choose(jitter, 1.8);
  EXPECT_EQ(selector.currentId(), original);
  EXPECT_EQ(selector.decision().reason, "same_route");
  choose(route(2.), 2.01);
  EXPECT_TRUE(selector.decision().switched);  // Timer was not reset at 1.8 s.
}

TEST_F(PathHysteresisTest, NearbyOppositeSidesOfThinWallAreNotSameRoute)
{
  Path a, b;
  a.poses = {pose(1., 4.), pose(4.95, 3.9), pose(6., 4.)};
  b.poses = {pose(1., 4.), pose(4.95, 4.1), pose(6., 4.)};
  map.setCost(49, 40, 254);
  EXPECT_FALSE(selector.sameRoute(a, b, map));
}

TEST_F(PathHysteresisTest, DisabledUsesEveryOriginalProposalWithoutCache)
{
  selector.enabled = false;
  for (double y : {6.5, 1.5, 6.5, 1.5}) {
    auto candidate = route(y), selected = choose(candidate, .1);
    EXPECT_EQ(candidate.poses, selected.poses);
    EXPECT_EQ(selector.currentId(), 0u);
  }
}

TEST_F(PathHysteresisTest, CurrentMapCostIsRecomputedInsteadOfUsingCachedCost)
{
  choose(route(6.5), 0.);
  for (unsigned int x = 0; x < 100; ++x) {
    for (unsigned int y = 41; y < 80; ++y) {
      if (map.getCost(x, y) < 253) {map.setCost(x, y, 200);}
    }
  }
  choose(route(1.5), 3.);
  EXPECT_TRUE(selector.decision().switched);
  EXPECT_EQ(selector.decision().reason, "cost_improved");
}

TEST_F(PathHysteresisTest, ScoringDoesNotDependOnWaypointDensityOrResolution)
{
  Path sparse; sparse.poses = {pose(.5, .5), pose(4.5, .5)};
  Path dense; for (int i = 0; i <= 40; ++i) {dense.poses.push_back(pose(.5+i*.1, .5));}
  const double expected = 4.*(2.+std::pow(26./252., 2));
  EXPECT_NEAR(selector.score(sparse, map, 2., 1.), expected, 1e-9);
  EXPECT_NEAR(selector.score(dense, map, 2., 1.), expected, 1e-9);
  nav2_costmap_2d::Costmap2D fine(200, 160, .05, 0., 0., 0);
  EXPECT_NEAR(selector.score(sparse, fine, 2., 1.), expected, 1e-9);
  std::cout << "calibration free_4m=" << expected << " penalty=1.0 => equivalent_length=" <<
    1./(2.+std::pow(26./252., 2)) << "m" << std::endl;
}

TEST_F(PathHysteresisTest, PruningDoesNotCutAcrossWallOrResetRouteId)
{
  choose(route(6.5), 0.);
  const Pose robot = pose(4., 6.5);
  Path next; next.header.frame_id = "map";
  next.poses = {robot, pose(6.5, 6.5), pose(9., 4.)};
  choose(next, .5, robot);
  EXPECT_EQ(selector.decision().reason, "same_route");
  EXPECT_EQ(selector.currentId(), 1u);
}

TEST_F(PathHysteresisTest, RobotOffRouteInvalidatesOldSelection)
{
  choose(route(6.5), 0.);
  Path next; next.poses = {pose(1., 1.), pose(6.5, 1.5), pose(9., 4.)};
  choose(next, .1, next.poses.front());
  EXPECT_FALSE(selector.decision().current_valid);
  EXPECT_EQ(selector.decision().reason, "current_invalid");
}

TEST_F(PathHysteresisTest, ShrinkingHorizonCropsOldTailWithoutTurningBack)
{
  choose(route(6.5), 0.);
  Path shorter = route(6.5);
  shorter.poses = {pose(1., 4.), pose(3.5, 6.5), pose(5., 6.5)};
  const auto selected = choose(shorter, .1);
  EXPECT_EQ(selector.decision().reason, "same_route");
  EXPECT_EQ(selector.currentId(), 1u);
  EXPECT_EQ(selected.poses, shorter.poses);
}
