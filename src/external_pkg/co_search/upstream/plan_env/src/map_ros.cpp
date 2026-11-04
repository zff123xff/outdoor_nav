#include <plan_env/map_ros.h>
#include <plan_env/multi_map_manager.h>
#include <plan_env/sdf_map.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <fstream>
#include <random>

namespace fast_planner
{
namespace
{
    constexpr double kDefaultGlobalMapPubPeriodSec = 5.0;
}  // namespace

MapROS::MapROS()
    : sync_image_pose_(nullptr), local_updated_(false), esdf_need_update_(false), has_depth_(false), has_pose_(false), has_cloud_(false), proj_points_cnt(0),
      fuse_time_(0.0), esdf_time_(0.0), max_fuse_time_(0.0), max_esdf_time_(0.0), fuse_num_(0), esdf_num_(0)
{
}

MapROS::~MapROS() {}

void MapROS::setMap(SDFMap* map)
{
    this->map_ = map;
}

void MapROS::init()
{
    // 相机内参
    fx_ = node_->has_parameter("map_ros/fx") ? node_->get_parameter("map_ros/fx").as_double() : node_->declare_parameter("map_ros/fx", -1.0);
    fy_ = node_->has_parameter("map_ros/fy") ? node_->get_parameter("map_ros/fy").as_double() : node_->declare_parameter("map_ros/fy", -1.0);
    cx_ = node_->has_parameter("map_ros/cx") ? node_->get_parameter("map_ros/cx").as_double() : node_->declare_parameter("map_ros/cx", -1.0);
    cy_ = node_->has_parameter("map_ros/cy") ? node_->get_parameter("map_ros/cy").as_double() : node_->declare_parameter("map_ros/cy", -1.0);

    // 深度值处理
    depth_filter_maxdist_ = node_->has_parameter("map_ros/depth_filter_maxdist")
                                ? node_->get_parameter("map_ros/depth_filter_maxdist").as_double()
                                : node_->declare_parameter("map_ros/depth_filter_maxdist", 5.0);  // 从launch文件中获取典型值
    depth_filter_mindist_ = node_->has_parameter("map_ros/depth_filter_mindist")
                                ? node_->get_parameter("map_ros/depth_filter_mindist").as_double()
                                : node_->declare_parameter("map_ros/depth_filter_mindist", 0.2);  // 从launch文件中获取典型值
    depth_filter_margin_  = node_->has_parameter("map_ros/depth_filter_margin") ? node_->get_parameter("map_ros/depth_filter_margin").as_int()
                                                                                : node_->declare_parameter("map_ros/depth_filter_margin", 2);

    k_depth_scaling_factor_ = node_->has_parameter("map_ros/k_depth_scaling_factor") ? node_->get_parameter("map_ros/k_depth_scaling_factor").as_double()
                                                                                     : node_->declare_parameter("map_ros/k_depth_scaling_factor", 1000.0);
    skip_pixel_ =
        node_->has_parameter("map_ros/skip_pixel") ? node_->get_parameter("map_ros/skip_pixel").as_int() : node_->declare_parameter("map_ros/skip_pixel", 2);

    // ESDF 切片高度
    esdf_slice_height_ = node_->has_parameter("map_ros/esdf_slice_height")
                             ? node_->get_parameter("map_ros/esdf_slice_height").as_double()
                             : node_->declare_parameter("map_ros/esdf_slice_height", 0.3);  // 从launch文件中获取典型值

    // 可视化高度阈值
    visualization_truncate_height_ = node_->has_parameter("map_ros/visualization_truncate_height")
                                         ? node_->get_parameter("map_ros/visualization_truncate_height").as_double()
                                         : node_->declare_parameter("map_ros/visualization_truncate_height", 1.5);  // 从launch文件中获取典型值
    visualization_truncate_low_    = node_->has_parameter("map_ros/visualization_truncate_low")
                                         ? node_->get_parameter("map_ros/visualization_truncate_low").as_double()
                                         : node_->declare_parameter("map_ros/visualization_truncate_low", 0.2);  // 从launch文件中获取典型值

    show_occ_time_             = node_->has_parameter("map_ros/show_occ_time") ? node_->get_parameter("map_ros/show_occ_time").as_bool()
                                                                               : node_->declare_parameter("map_ros/show_occ_time", false);
    show_esdf_time_            = node_->has_parameter("map_ros/show_esdf_time") ? node_->get_parameter("map_ros/show_esdf_time").as_bool()
                                                                                : node_->declare_parameter("map_ros/show_esdf_time", false);
    show_all_map_              = node_->has_parameter("map_ros/show_all_map") ? node_->get_parameter("map_ros/show_all_map").as_bool()
                                                                              : node_->declare_parameter("map_ros/show_all_map", true);
    global_map_pub_period_sec_ = node_->has_parameter("map_ros/global_map_pub_period_sec")
                                     ? node_->get_parameter("map_ros/global_map_pub_period_sec").as_double()
                                     : node_->declare_parameter("map_ros/global_map_pub_period_sec", kDefaultGlobalMapPubPeriodSec);
    frame_id_                  = node_->has_parameter("map_ros/frame_id") ? node_->get_parameter("map_ros/frame_id").as_string()
                                                                          : node_->declare_parameter("map_ros/frame_id", std::string("world"));
    self_drone_id_             = node_->has_parameter("exploration/drone_id") ? node_->get_parameter("exploration/drone_id").as_int()
                                                                              : node_->declare_parameter("exploration/drone_id", 1);
    robot_frame_               = node_->has_parameter("map_ros/robot_frame_prefix") ? node_->get_parameter("map_ros/robot_frame_prefix").as_string()
                                                                                    : node_->declare_parameter("map_ros/robot_frame_prefix", std::string(""));
    lidar_frame_               = node_->has_parameter("map_ros/lidar_frame") ? node_->get_parameter("map_ros/lidar_frame").as_string()
                                                                             : node_->declare_parameter("map_ros/lidar_frame", std::string(""));

    // 分配点云空间
    proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
    point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
    proj_points_cnt = 0;

    fuse_time_     = 0.0;
    esdf_time_     = 0.0;
    max_fuse_time_ = 0.0;
    max_esdf_time_ = 0.0;
    fuse_num_      = 0;
    esdf_num_      = 0;
    depth_image_.reset(new cv::Mat);

    // 随机噪声
    rand_noise_ = std::normal_distribution<double>(0, 0.1);
    std::random_device rd;
    eng_ = std::default_random_engine(rd());

    // 定时器
    esdf_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MapROS::updateESDFCallback, this));
    vis_timer_  = node_->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapROS::visCallback, this));

    // 发布器：全图为 2D OccupancyGrid（RViz Map）；局部仍为点云
    map_all_pub_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/sdf_map/occupancy_all", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    map_local_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local",
                                                                            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    // map_local_inflate_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
    // unknown_pub_           = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/unknown", 10);
    // esdf_pub_              = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", rclcpp::QoS(10));
    // update_range_pub_      = node_->create_publisher<visualization_msgs::msg::Marker>("/sdf_map/update_range", rclcpp::QoS(10));
    // depth_pub_             = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/depth_cloud", 10);
    // debug_lidar_pub_       = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/lidar", 10);

    // TF2
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string obstacle_source = node_->has_parameter("map_ros/obstacle_source") ? node_->get_parameter("map_ros/obstacle_source").as_string()
                                                                                  : node_->declare_parameter("map_ros/obstacle_source", std::string("depth"));

    // 订阅器（高频传感器/状态数据 → BEST_EFFORT + VOLATILE + KeepLast(5) 以降低 DDS CPU）
    drone_state_sub_ =
        node_->create_subscription<msg_set::msg::DroneState>("/swarm_expl/drone_state", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
                                                             std::bind(&MapROS::droneStateCallback, this, std::placeholders::_1));

    if (obstacle_source == "cloud")
    {
        // 同步器 - cloud + odom（高频点云/里程计）
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(node_.get(), "/map_ros/cloud", qos.get_rmw_qos_profile()));
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(node_.get(), "/map_ros/odom", qos.get_rmw_qos_profile()));
        sync_cloud_odom_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudOdom>(MapROS::SyncPolicyCloudOdom(10), *cloud_sub_, *odom_sub_));
        sync_cloud_odom_->registerCallback(std::bind(&MapROS::cloudOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(node_->get_logger(), "[MapROS] obstacle_source=cloud, enabling cloud+odom synchronization.");
    }
    else
    {
        // 默认路径：depth + pose（高频深度/位姿）
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(node_.get(), "/map_ros/depth", qos.get_rmw_qos_profile()));
        pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::msg::PoseStamped>(node_.get(), "/map_ros/pose", qos.get_rmw_qos_profile()));
        sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(MapROS::SyncPolicyImagePose(10), *depth_sub_, *pose_sub_));
        sync_image_pose_->registerCallback(std::bind(&MapROS::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(node_->get_logger(), "[MapROS] obstacle_source=%s, enabling depth+pose synchronization.", obstacle_source.c_str());
    }

    map_start_time_ = node_->now();
}

void MapROS::visCallback()
{
    // publishMapLocal();
    // 已注释以降低 CPU：全图、ESDF、未知区域、更新范围等可视化发布
    if (show_all_map_)
    {
        const rclcpp::Time now         = node_->now();
        const bool         no_throttle = global_map_pub_period_sec_ <= 0.0;
        const bool         due         = no_throttle || !global_map_last_pub_valid_ || (now - global_map_last_pub_).seconds() >= global_map_pub_period_sec_;
        if (due)
        {
            publishMapAll();
            global_map_last_pub_       = now;
            global_map_last_pub_valid_ = true;
        }
    }
    // publishUpdateRange();
}

void MapROS::updateESDFCallback()
{
    if (!esdf_need_update_)
        return;
    auto t1 = node_->now();

    map_->updateESDF3d();
    esdf_need_update_ = false;

    auto t2 = node_->now();
    esdf_time_ += (t2 - t1).seconds();
    max_esdf_time_ = std::max(max_esdf_time_, (t2 - t1).seconds());
    esdf_num_++;
    if (show_esdf_time_)
        RCLCPP_WARN(node_->get_logger(), "ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).seconds(), esdf_time_ / esdf_num_, max_esdf_time_);
}

void MapROS::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr img, const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)
{
    camera_pos_(0) = pose->pose.position.x;
    camera_pos_(1) = pose->pose.position.y;
    camera_pos_(2) = pose->pose.position.z;

    if (!map_->isInMap(camera_pos_))
        return;

    map_->mm_->drone_pos_ = camera_pos_;
    camera_q_             = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
    cv_ptr->image.copyTo(*depth_image_);

    auto t1 = node_->now();
    processDepthImage();
    map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);

    if (local_updated_)
    {
        map_->clearAndInflateLocalMap();
        map_->applyDynamicFootprintClearFromMapROS(camera_pos_, drone_states_, drone_vels_, self_drone_id_);
        esdf_need_update_ = true;
        local_updated_    = false;
    }

    auto t2 = node_->now();
    fuse_time_ += (t2 - t1).seconds();
    max_fuse_time_ = std::max(max_fuse_time_, (t2 - t1).seconds());
    fuse_num_++;
    if (show_occ_time_)
        RCLCPP_WARN(node_->get_logger(), "Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).seconds(), fuse_time_ / fuse_num_, max_fuse_time_);
}

void MapROS::cloudOdomCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    // 1. 从里程计更新相机位置
    camera_pos_(0) = odom_msg->pose.pose.position.x;
    camera_pos_(1) = odom_msg->pose.pose.position.y;
    camera_pos_(2) = 0.01;

    // 使用与节点相同的时钟类型构造 odom_stamp
    const rclcpp::Time odom_stamp(odom_msg->header.stamp, node_->get_clock()->get_clock_type());
    const rclcpp::Time now                = node_->now();
    const double       odom_stamp_lag_sec = (now - odom_stamp).seconds();
    // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Odom stamp: %.9f s, now: %.9f s, lag: %.3f s", odom_stamp.seconds(), now.seconds(),
    //                      odom_stamp_lag_sec);

    if (!map_->isInMap(camera_pos_))
        return;

    // 2. 过滤点云（调用处理函数）
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud = processPointCloud(cloud_msg);

    // 3. 打印输入地图的点云数量（警告级别，可限制频率）
    size_t cloud_size = filtered_cloud.points.size();
    if (cloud_size == 0)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Filtered point cloud is empty, skip mapping");
        return;
    }
    // RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "Input point cloud size to map: %zu", cloud_size);

    // 4. TF 变换（使用滤波后的点云）
    std::string                          source_frame = cloud_msg->header.frame_id;
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
        if (!lidar_frame_.empty())
        {
            source_frame = lidar_frame_;
        }
        else if (source_frame.find('/') == std::string::npos && !robot_frame_.empty())
        {
            source_frame = robot_frame_ + "/" + source_frame;
        }
        transform_stamped = tf_buffer_->lookupTransform("world", source_frame, odom_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

        camera_q_ = Eigen::Quaterniond(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y,
                                       transform_stamped.transform.rotation.z);

        Eigen::Matrix3d rot         = camera_q_.toRotationMatrix();
        Eigen::Matrix4d transform   = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = rot;
        transform(0, 3)             = transform_stamped.transform.translation.x;
        transform(1, 3)             = transform_stamped.transform.translation.y;
        transform(2, 3)             = 0.01;  // 您原有的特殊 z 偏移

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(filtered_cloud, *transformed_cloud, transform);

        map_->inputPointCloud(*transformed_cloud, transformed_cloud->points.size(), camera_pos_);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 100, "Could not transform %s to world: %s", source_frame.c_str(), ex.what());
        return;
    }

    if (local_updated_)
    {
        map_->clearAndInflateLocalMap();
        map_->applyDynamicFootprintClearFromMapROS(camera_pos_, drone_states_, drone_vels_, self_drone_id_);
        esdf_need_update_ = true;
        local_updated_    = false;
    }
}

void MapROS::droneStateCallback(const msg_set::msg::DroneState::ConstSharedPtr msg)
{
    if (msg->pos.size() >= 3)
    {
        Eigen::Vector4d state(msg->pos[0], msg->pos[1], msg->pos[2], msg->yaw);
        drone_states_[msg->drone_id] = state;
        if (msg->vel.size() >= 3)
            drone_vels_[msg->drone_id] = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
        syncPeerStatesToMap();
    }
}

void MapROS::syncPeerStatesToMap()
{
    // 只同步队友（排除自己）：hit 标记若把自车附近的静态障碍误标为动态命中，
    // 会使其 miss 一次就被快清，破坏近处地图稳定性。
    std::map<int, Eigen::Vector4d> peers;
    for (const auto& entry : drone_states_)
    {
        if (entry.first != self_drone_id_)
            peers[entry.first] = entry.second;
    }
    map_->setPeerStates(peers);
}

void MapROS::processDepthImage()
{
    proj_points_cnt = 0;
    uint16_t*       row_ptr;
    int             cols = depth_image_->cols;
    int             rows = depth_image_->rows;
    double          depth;
    Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();

    Eigen::AngleAxisd x_correction(M_PI / 2, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd y_correction(M_PI / 2, Eigen::Vector3d::UnitY());
    Eigen::Matrix3d   correction = x_correction.matrix() * y_correction.matrix();
    camera_r                     = camera_r * correction;

    Eigen::Vector3d pt_cur, pt_world;
    const double    inv_factor = 1.0 / k_depth_scaling_factor_;

    for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_)
    {
        row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
        for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_)
        {
            const uint16_t raw_depth = *row_ptr;
            depth                    = raw_depth * inv_factor;
            row_ptr                  = row_ptr + skip_pixel_;

            if (raw_depth == 0 || depth > depth_filter_maxdist_)
                depth = depth_filter_maxdist_;
            else if (depth < depth_filter_mindist_)
                continue;

            pt_cur(0) = (u - cx_) * depth / fx_;
            pt_cur(1) = (v - cy_) * depth / fy_;
            pt_cur(2) = depth;

            pt_world = camera_r * pt_cur + camera_pos_;

            auto& pt = point_cloud_.points[proj_points_cnt++];
            pt.x     = pt_world[0];
            pt.y     = pt_world[1];
            pt.z     = pt_world[2];
        }
    }

    // publishDepth();  // 已注释（depth_pub_ 已禁用）
}

pcl::PointCloud<pcl::PointXYZ> MapROS::processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ>     pass;

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.05, 4.0);
    pass.filter(*filtered_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ>       voxel;
    voxel.setInputCloud(filtered_cloud);
    voxel.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel.filter(*downsampled_cloud);

    return *downsampled_cloud;
}

void MapROS::publishMapLocal()
{
    pcl::PointXYZ                  pt;
    pcl::PointCloud<pcl::PointXYZ> cloud, cloud2;
    Eigen::Vector3i                min_cut = map_->md_->local_bound_min_;
    Eigen::Vector3i                max_cut = map_->md_->local_bound_max_;
    map_->boundIndex(min_cut);
    map_->boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z)
            {
                if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)
                {
                    Eigen::Vector3d pos;
                    map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
                    if (pos(2) > visualization_truncate_height_ || pos(2) < visualization_truncate_low_)
                        continue;
                    pt.x = pos(0);
                    pt.y = pos(1);
                    pt.z = pos(2);
                    cloud.push_back(pt);
                }
            }

    cloud.width            = cloud.points.size();
    cloud.height           = 1;
    cloud.is_dense         = true;
    cloud.header.frame_id  = frame_id_;
    cloud2.width           = cloud2.points.size();
    cloud2.height          = 1;
    cloud2.is_dense        = true;
    cloud2.header.frame_id = frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    // Set header timestamp
    cloud_msg.header.stamp = node_->now();
    map_local_pub_->publish(cloud_msg);

    // 已注释 map_local_inflate_pub_（对应发布器已禁用）
    // pcl::toROSMsg(cloud2, cloud_msg);
    // cloud_msg.header.stamp = node_->now();
    // map_local_inflate_pub_->publish(cloud_msg);
}

void MapROS::publishMapAll()
{
    Eigen::Vector3i min_idx, max_idx;
    map_->posToIndex(map_->md_->all_min_, min_idx);
    map_->posToIndex(map_->md_->all_max_, max_idx);
    map_->boundIndex(min_idx);
    map_->boundIndex(max_idx);

    const int width  = max_idx[0] - min_idx[0] + 1;
    const int height = max_idx[1] - min_idx[1] + 1;
    if (width <= 0 || height <= 0)
    {
        return;
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp              = node_->now();
    grid.header.frame_id           = frame_id_;
    grid.info.map_load_time        = grid.header.stamp;
    grid.info.resolution           = static_cast<float>(map_->mp_->resolution_);
    grid.info.width                = static_cast<uint32_t>(width);
    grid.info.height               = static_cast<uint32_t>(height);
    grid.info.origin.position.x    = map_->mp_->map_origin_(0) + static_cast<double>(min_idx[0]) * map_->mp_->resolution_;
    grid.info.origin.position.y    = map_->mp_->map_origin_(1) + static_cast<double>(min_idx[1]) * map_->mp_->resolution_;
    grid.info.origin.position.z    = map_->mp_->map_origin_(2);
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;

    int obstacle_z_min = 0, obstacle_z_max = 0;
    map_->getNavColumnZRange(obstacle_z_min, obstacle_z_max);
    const size_t ncells = static_cast<size_t>(width) * static_cast<size_t>(height);
    // ROS OccupancyGrid standard: -1 = unknown, 0 = free, 100 = occupied.
    // Initialize as unknown so unexplored regions show up correctly in RViz.
    grid.data.assign(ncells, -1);

    for (int j = 0; j < height; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            const int ix       = min_idx[0] + i;
            const int iy       = min_idx[1] + j;
            int8_t    cell_val = -1;
            for (int iz = obstacle_z_min; iz <= obstacle_z_max; ++iz)
            {
                const Eigen::Vector3i idx(ix, iy, iz);
                if (!map_->isInMap(idx))
                {
                    continue;
                }
                const int    adr = map_->toAddress(idx);
                const double occ = map_->md_->occupancy_buffer_[adr];
                if (occ > map_->mp_->min_occupancy_log_)
                {
                    cell_val = 100;
                    break;
                }
                // Any observed (non-unknown) voxel in the column marks the cell as free
                // unless an occupied voxel was already found above.
                if (cell_val == -1 && occ >= map_->mp_->clamp_min_log_ - 1e-3)
                {
                    cell_val = 0;
                }
            }
            grid.data[j * width + i] = cell_val;
        }
    }

    map_all_pub_->publish(grid);
}

void MapROS::publishUpdateRange()
{
    Eigen::Vector3d                 esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
    visualization_msgs::msg::Marker mk;
    map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
    map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

    cube_pos              = 0.5 * (esdf_min_pos + esdf_max_pos);
    cube_scale            = esdf_max_pos - esdf_min_pos;
    mk.header.frame_id    = frame_id_;
    mk.header.stamp       = node_->now();
    mk.type               = visualization_msgs::msg::Marker::CUBE;
    mk.action             = visualization_msgs::msg::Marker::ADD;
    mk.id                 = 0;
    mk.pose.position.x    = cube_pos(0);
    mk.pose.position.y    = cube_pos(1);
    mk.pose.position.z    = cube_pos(2);
    mk.scale.x            = cube_scale(0);
    mk.scale.y            = cube_scale(1);
    mk.scale.z            = cube_scale(2);
    mk.color.a            = 0.3;
    mk.color.r            = 1.0;
    mk.color.g            = 0.0;
    mk.color.b            = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;

    update_range_pub_->publish(mk);
}

void MapROS::publishDepth()
{
    pcl::PointXYZ                  pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < proj_points_cnt; ++i)
    {
        cloud.push_back(point_cloud_.points[i]);
    }
    cloud.width           = cloud.points.size();
    cloud.height          = 1;
    cloud.is_dense        = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    // Set header timestamp
    cloud_msg.header.stamp = node_->now();
    depth_pub_->publish(cloud_msg);
}

void MapROS::publishESDF()
{
    double                          dist;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI                  pt;

    const double min_dist = 0.0;
    const double max_dist = 3.0;

    Eigen::Vector3i min_cut =
        map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_, map_->mp_->local_map_margin_, map_->mp_->local_map_margin_);
    Eigen::Vector3i max_cut =
        map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_, map_->mp_->local_map_margin_, map_->mp_->local_map_margin_);
    map_->boundIndex(min_cut);
    map_->boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
        {
            Eigen::Vector3d pos;
            map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
            pos(2)       = esdf_slice_height_;
            dist         = map_->getDistance(pos);
            dist         = std::min(dist, max_dist);
            dist         = std::max(dist, min_dist);
            pt.x         = pos(0);
            pt.y         = pos(1);
            pt.z         = 0.2;
            pt.intensity = (dist - min_dist) / (max_dist - min_dist);
            cloud.push_back(pt);
        }

    cloud.width           = cloud.points.size();
    cloud.height          = 1;
    cloud.is_dense        = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    // Set header timestamp
    cloud_msg.header.stamp = node_->now();
    esdf_pub_->publish(cloud_msg);
}

void MapROS::publishUnknown()
{
    pcl::PointXYZ                  pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Vector3i                min_idx, max_idx;
    map_->posToIndex(map_->md_->all_min_, min_idx);
    map_->posToIndex(map_->md_->all_max_, max_idx);
    map_->boundIndex(min_idx);
    map_->boundIndex(max_idx);

    for (int x = min_idx(0); x <= max_idx(0); ++x)
        for (int y = min_idx(1); y <= max_idx(1); ++y)
            for (int z = min_idx(2); z <= max_idx(2); ++z)
            {
                if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->clamp_min_log_ - 1e-3)
                {
                    Eigen::Vector3d pos;
                    map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
                    if (pos(2) > visualization_truncate_height_ || pos(2) < visualization_truncate_low_)
                        continue;
                    pt.x = pos(0);
                    pt.y = pos(1);
                    pt.z = pos(2);
                    cloud.push_back(pt);
                }
            }

    cloud.width           = cloud.points.size();
    cloud.height          = 1;
    cloud.is_dense        = true;
    cloud.header.frame_id = frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    // Set header timestamp
    cloud_msg.header.stamp = node_->now();
    unknown_pub_->publish(cloud_msg);
}

}  // namespace fast_planner