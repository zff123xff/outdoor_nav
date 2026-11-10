// Copyright 2026 outdoor_nav contributors
// Licensed under the Apache License, Version 2.0.
#include <array>
#include <limits>
#include <memory>
#include <vector>
#include "gtest/gtest.h"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rcl/time.h"

class ObstacleDensityTest : public testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  nav2_util::LifecycleNode::SharedPtr node;
  std::unique_ptr<tf2_ros::Buffer> tf;
  nav2_costmap_2d::LayeredCostmap layers{"map", false, false};
  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> layer;
  void SetUp() override
  {
    node = std::make_shared<nav2_util::LifecycleNode>("density_test");
    node->declare_parameter("track_unknown_space", false);
    node->declare_parameter("transform_tolerance", .3);
    node->declare_parameter("obstacles.footprint_clearing_enabled", false);
    node->declare_parameter("obstacles.clear_before_update", true);
    node->declare_parameter("obstacles.max_obstacle_height", .5);
    node->declare_parameter("obstacles.min_points_per_cell", 3);
    tf = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    layers.resizeMap(100, 100, .05, 0., 0.);
    layer = std::make_shared<nav2_costmap_2d::ObstacleLayer>();
    layer->initialize(&layers, "obstacles", tf.get(), node, nullptr);
    layers.addPlugin(layer);
  }
  void setTime(double seconds)
  {
    auto handle = node->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(handle), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(handle,
      static_cast<rcl_time_point_value_t>(seconds*1e9)), RCL_RET_OK);
  }
  void enableWindow(double seconds = .5)
  {
    ASSERT_TRUE(node->set_parameters_atomically({
      rclcpp::Parameter("obstacles.point_count_window_s", seconds)}).successful);
  }
  void add(const std::vector<std::array<float, 3>> & points, double age = 0.)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = node->now()-rclcpp::Duration::from_seconds(age);
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto & p : points) {*x = p[0]; *y = p[1]; *z = p[2]; ++x; ++y; ++z;}
    geometry_msgs::msg::Point origin;
    nav2_costmap_2d::Observation observation(origin, cloud, 4., 0., 4., 0.);
    layer->addStaticObservation(observation, true, false);
  }
  void update()
  {
    layer->addExtraBounds(0., 0., 5., 5.);
    layers.updateMap(.5, .5, 0.);
  }
  unsigned char cost(double x, double y)
  {
    unsigned int mx, my;
    EXPECT_TRUE(layers.getCostmap()->worldToMap(x, y, mx, my));
    return layers.getCostmap()->getCost(mx, my);
  }
};

TEST_F(ObstacleDensityTest, OnlyCellsWithEnoughPointsInOneCloudAreMarked)
{
  add({{1.01f, 1.01f, .1f},
    {1.11f, 1.01f, .1f}, {1.12f, 1.02f, .2f},
    {1.21f, 1.01f, .1f}, {1.22f, 1.02f, .2f}, {1.23f, 1.03f, .3f}});
  update();
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(1.11, 1.01), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(1.21, 1.01), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(ObstacleDensityTest, CachedCloudsAndRepeatedUpdatesDoNotAccumulateHits)
{
  for (int i = 0; i < 10; ++i) {add({{1.01f, 1.01f, .1f}});}
  for (int i = 0; i < 10; ++i) {update();}
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(ObstacleDensityTest, HeightRangeAndFiniteChecksPrecedeCounting)
{
  add({{1.01f, 1.01f, .1f}, {1.02f, 1.02f, .2f}, {1.03f, 1.03f, .9f},
    {1.03f, 1.03f, std::numeric_limits<float>::quiet_NaN()},
    {4.5f, 1.01f, .1f}, {4.51f, 1.02f, .2f}, {4.52f, 1.03f, .3f}});
  update();
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(4.51, 1.01), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(ObstacleDensityTest, OneRestoresOriginalMarkingAndThresholdCanChangeAtRuntime)
{
  add({{1.01f, 1.01f, .1f}});
  ASSERT_TRUE(node->set_parameters_atomically({
    rclcpp::Parameter("obstacles.min_points_per_cell", 1)}).successful);
  update();
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(node->set_parameters_atomically({
    rclcpp::Parameter("obstacles.min_points_per_cell", 3)}).successful);
  update();
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(ObstacleDensityTest, InvalidParametersRejectAtomically)
{
  for (const auto & value : {rclcpp::Parameter("obstacles.min_points_per_cell", 0),
    rclcpp::Parameter("obstacles.min_points_per_cell", -1),
    rclcpp::Parameter("obstacles.min_points_per_cell", 1000001),
    rclcpp::Parameter("obstacles.min_points_per_cell", 2.5)})
  {
    EXPECT_FALSE(node->set_parameters_atomically({
      rclcpp::Parameter("obstacles.enabled", false), value}).successful);
    EXPECT_EQ(node->get_parameter("obstacles.min_points_per_cell").as_int(), 3);
    EXPECT_TRUE(layer->isEnabled());
  }
}

TEST_F(ObstacleDensityTest, RecentSparseFramesCombineWithinWindow)
{
  setTime(100.); enableWindow();
  add({{1.01f, 1.01f, .1f}}, .1);
  add({{1.02f, 1.02f, .2f}}, .2);
  update();
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::FREE_SPACE);
  add({{1.03f, 1.03f, .3f}}, .3);
  update();
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(ObstacleDensityTest, RepeatedUpdatesDoNotTurnTwoPointsIntoThree)
{
  setTime(100.); enableWindow();
  add({{1.01f, 1.01f, .1f}}, .1);
  add({{1.02f, 1.02f, .2f}}, .2);
  for (int i = 0; i < 20; ++i) {update();}
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(ObstacleDensityTest, ExpirationClearsMasterMapEvenWithNoNewCloudOrForcedBounds)
{
  setTime(100.); enableWindow();
  add({{1.01f, 1.01f, .1f}}, .1);
  add({{1.02f, 1.02f, .2f}}, .2);
  add({{1.03f, 1.03f, .3f}}, .3);
  layers.updateMap(.5, .5, 0.);
  ASSERT_EQ(cost(1.01, 1.01), nav2_costmap_2d::LETHAL_OBSTACLE);
  setTime(101.);
  layers.updateMap(.5, .5, 0.);
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(ObstacleDensityTest, OldAndFutureCloudsDoNotContribute)
{
  setTime(100.); enableWindow();
  add({{1.01f, 1.01f, .1f}}, .1);
  add({{1.02f, 1.02f, .2f}}, .6);
  add({{1.03f, 1.03f, .3f}}, -.1);
  update();
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(ObstacleDensityTest, ExpiredEvidenceAlsoClearsInflation)
{
  setTime(100.); enableWindow();
  node->declare_parameter("inflation.inflation_radius", .5);
  auto inflation = std::make_shared<nav2_costmap_2d::InflationLayer>();
  inflation->initialize(&layers, "inflation", tf.get(), node, nullptr);
  layers.addPlugin(inflation);
  for (double age : {.1, .2, .3}) {add({{1.01f, 1.01f, .1f}}, age);}
  layers.updateMap(.5, .5, 0.);
  ASSERT_GT(cost(1.21, 1.01), nav2_costmap_2d::FREE_SPACE);
  setTime(101.);
  layers.updateMap(.5, .5, 0.);
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cost(1.21, 1.01), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(ObstacleDensityTest, WindowCanBeDisabledAndRollingOriginDoesNotMoveEvidence)
{
  setTime(100.); enableWindow();
  for (double age : {.1, .2, .3}) {add({{1.01f, 1.01f, .1f}}, age);}
  update();
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::LETHAL_OBSTACLE);
  layers.getCostmap()->updateOrigin(.5, .5);
  layer->updateOrigin(.5, .5);
  update();
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::LETHAL_OBSTACLE);
  enableWindow(0.);
  update();
  EXPECT_EQ(cost(1.01, 1.01), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(ObstacleDensityTest, InvalidWindowValuesRejectAtomically)
{
  for (const auto & value : {rclcpp::Parameter("obstacles.point_count_window_s", -1.),
    rclcpp::Parameter("obstacles.point_count_window_s", std::numeric_limits<double>::infinity()),
    rclcpp::Parameter("obstacles.point_count_window_s", std::numeric_limits<double>::quiet_NaN()),
    rclcpp::Parameter("obstacles.point_count_window_s", 1)})
  {
    EXPECT_FALSE(node->set_parameters_atomically({
      rclcpp::Parameter("obstacles.enabled", false), value}).successful);
    EXPECT_EQ(node->get_parameter("obstacles.point_count_window_s").as_double(), 0.);
    EXPECT_TRUE(layer->isEnabled());
  }
}
