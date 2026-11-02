// Copyright 2026 outdoor_nav contributors
#include <cmath>
#include <memory>
#include <vector>
#include "gtest/gtest.h"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

class GroundHeightTest : public testing::Test
{
public:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(GroundHeightTest, TiltAndGroundOffsetFilterHeightButPreserveGlobalCoordinates)
{
  for (double pitch : {-0.4, 0.0, 0.4}) {
    for (double ground_z : {0.0, -0.2}) {
      auto node = std::make_shared<nav2_util::LifecycleNode>("ground_height_test");
      tf2_ros::Buffer tf(node->get_clock());
      tf2::Quaternion q;
      q.setRPY(0.15, pitch, 0.7);
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "map";
      transform.child_frame_id = "base_link";
      transform.transform.translation.z = 7.0;
      transform.transform.rotation = tf2::toMsg(q);
      ASSERT_TRUE(tf.setTransform(transform, "test", true));
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id = "map";
      cloud.header.stamp = node->now();
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(3);
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
      std::vector<tf2::Vector3> world;
      for (double height : {0.0, 0.2, 0.6}) {
        auto point = tf2::quatRotate(q, tf2::Vector3(2.0, 0.0, ground_z + height));
        point += tf2::Vector3(0, 0, 7);
        world.push_back(point);
        *x = point.x(); *y = point.y(); *z = point.z();
        ++x; ++y; ++z;
      }
      nav2_costmap_2d::ObservationBuffer buffer(
        node, "cloud", 0, 0, .05, .35, 10, 0, 10, 0, tf,
        "map", "base_link", tf2::durationFromSec(0), "base_link", ground_z);
      buffer.bufferCloud(cloud);
      std::vector<nav2_costmap_2d::Observation> observations;
      buffer.getObservations(observations);
      ASSERT_EQ(observations.size(), 1u);
      ASSERT_EQ(observations.front().cloud_->width, 1u);
      sensor_msgs::PointCloud2ConstIterator<float> out_x(*observations.front().cloud_, "x");
      sensor_msgs::PointCloud2ConstIterator<float> out_z(*observations.front().cloud_, "z");
      EXPECT_NEAR(*out_x, world[1].x(), 1e-5);
      EXPECT_NEAR(*out_z, world[1].z(), 1e-5);
    }
  }
}

TEST_F(GroundHeightTest, MissingTiltTransformDoesNotFallBackToFlatGround)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("missing_ground_height_test");
  tf2_ros::Buffer tf(node->get_clock());
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = node->now();
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(0);
  nav2_costmap_2d::ObservationBuffer buffer(
    node, "cloud", 0, 0, .05, .35, 10, 0, 10, 0, tf,
    "map", "map", tf2::durationFromSec(0), "missing_body", 0);
  buffer.bufferCloud(cloud);
  std::vector<nav2_costmap_2d::Observation> observations;
  buffer.getObservations(observations);
  EXPECT_TRUE(observations.empty());
}
