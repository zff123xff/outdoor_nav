#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <msg_set/msg/drone_state.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <pcl/common/common.h>
#include <random>
#include <string>
#include <vector>

using std::default_random_engine;
using std::map;
using std::normal_distribution;
using std::shared_ptr;
using std::string;
using std::vector;

namespace fast_planner
{
class SDFMap;

class MapROS
{
public:
    MapROS();
    ~MapROS();
    void setMap(SDFMap* map);
    void init();

private:
    void depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr img, const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose);
    void cloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud, const nav_msgs::msg::Odometry::ConstSharedPtr odom);
    void updateESDFCallback();
    void visCallback();
    void droneStateCallback(const msg_set::msg::DroneState::ConstSharedPtr msg);
    // 把队友（排除自己）最新位置同步给 SDFMap，供 hit 侧动态命中标记使用
    void syncPeerStatesToMap();

    void publishMapAll();
    void publishMapLocal();
    void publishESDF();
    void publishUnknown();
    void publishDepth();
    void publishUpdateRange();

    void processDepthImage();

    pcl::PointCloud<pcl::PointXYZ> processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    SDFMap*                                                                                                           map_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped> SyncPolicyImagePose;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>>                                            SynchronizerImagePose;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>   SyncPolicyCloudOdom;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>>                                            SynchronizerCloudOdom;

public:
    rclcpp::Node::SharedPtr                                                  node_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>         depth_sub_;
    shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_sub_;
    SynchronizerImagePose                                                    sync_image_pose_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>   cloud_sub_;
    shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>>         odom_sub_;
    SynchronizerCloudOdom                                                    sync_cloud_odom_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    string                                         robot_frame_;  // e.g. "car4", prepended when cloud frame_id has no '/'
    string                                         lidar_frame_;  // explicit override, e.g. "car4/rslidar"

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   map_local_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   map_local_inflate_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   esdf_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_all_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   unknown_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr update_range_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   debug_lidar_pub_;

    rclcpp::TimerBase::SharedPtr                              esdf_timer_;
    rclcpp::TimerBase::SharedPtr                              vis_timer_;
    rclcpp::Subscription<msg_set::msg::DroneState>::SharedPtr drone_state_sub_;

    // params, depth projection
    double cx_, cy_, fx_, fy_;
    double depth_filter_maxdist_, depth_filter_mindist_;
    int    depth_filter_margin_;
    double k_depth_scaling_factor_;
    int    skip_pixel_;
    string frame_id_;
    // msg publication
    double esdf_slice_height_;
    double visualization_truncate_height_, visualization_truncate_low_;
    bool   show_esdf_time_, show_occ_time_;
    bool   show_all_map_;
    /// Minimum wall time between full-map OccupancyGrid publishes (seconds). <=0 means publish on every vis tick.
    double       global_map_pub_period_sec_;
    bool         global_map_last_pub_valid_{ false };
    rclcpp::Time global_map_last_pub_;

    // data
    bool local_updated_, esdf_need_update_;
    bool has_depth_, has_pose_, has_cloud_;

public:
    Eigen::Vector3d    camera_pos_;
    Eigen::Quaterniond camera_q_;

    // point clouds
    vector<pcl::PointXYZ>          proj_points_;
    pcl::PointCloud<pcl::PointXYZ> point_cloud_;
    int                            proj_points_cnt;

    // cv image
    shared_ptr<cv::Mat>                             depth_image_;
    sensor_msgs::msg::Image::ConstSharedPtr         depth_img_;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr   cloud_;

    // timing
    rclcpp::Time map_start_time_;
    double       fuse_time_, esdf_time_;
    double       max_fuse_time_, max_esdf_time_;
    int          fuse_num_, esdf_num_;

    // random noise
    normal_distribution<double> rand_noise_;
    default_random_engine       eng_;

    // drone states
    map<int, Eigen::Vector4d> drone_states_;
    map<int, Eigen::Vector3d> drone_vels_;  // 队友速度（足迹清除的动/停判定）
    int                       self_drone_id_{ 1 };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif