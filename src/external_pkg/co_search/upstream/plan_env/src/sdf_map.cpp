#include "plan_env/sdf_map.h"
#include "plan_env/map_ros.h"
#include <plan_env/multi_map_manager.h>
#include <plan_env/raycast.h>
#include <cmath>
#include <stdexcept>

namespace fast_planner
{
// 动态足迹清除速度门控：速度高于此阈值的队友视为“移动中”，才允许清除其足迹（m/s）。
constexpr double kDynamicClearSpeedEps = 0.15;

SDFMap::SDFMap() {}

SDFMap::~SDFMap() {}

void SDFMap::initMap(rclcpp::Node::SharedPtr nh)
{
    mp_.reset(new MapParam);         //地图参数
    md_.reset(new MapData);          // 地图数据
    mr_.reset(new MapROS);           // ROS接口
    mm_.reset(new MultiMapManager);  // 多地图管理

    // Params of map properties
    double x_size, y_size, z_size;
    mp_->resolution_          = nh->has_parameter("sdf_map/resolution") ? nh->get_parameter("sdf_map/resolution").as_double()
                                                                        : nh->declare_parameter<double>("sdf_map/resolution", -1.0);  // 地图分辨率
    x_size                    = nh->has_parameter("sdf_map/map_size_x") ? nh->get_parameter("sdf_map/map_size_x").as_double()
                                                                        : nh->declare_parameter<double>("sdf_map/map_size_x", -1.0);  // 地图 x 轴尺寸
    y_size                    = nh->has_parameter("sdf_map/map_size_y") ? nh->get_parameter("sdf_map/map_size_y").as_double()
                                                                        : nh->declare_parameter<double>("sdf_map/map_size_y", -1.0);  // 地图 y 轴尺寸
    z_size                    = nh->has_parameter("sdf_map/map_size_z") ? nh->get_parameter("sdf_map/map_size_z").as_double()
                                                                        : nh->declare_parameter<double>("sdf_map/map_size_z", -1.0);  // 地图 z 轴尺寸
    mp_->obstacles_inflation_ = nh->has_parameter("sdf_map/obstacles_inflation")
                                    ? nh->get_parameter("sdf_map/obstacles_inflation").as_double()
                                    : nh->declare_parameter<double>("sdf_map/obstacles_inflation", -1.0);  // 障碍物膨胀距离
    mp_->local_bound_inflate_ = nh->has_parameter("sdf_map/local_bound_inflate")
                                    ? nh->get_parameter("sdf_map/local_bound_inflate").as_double()
                                    : nh->declare_parameter<double>("sdf_map/local_bound_inflate", 1.0);  // 本地边界膨胀
    mp_->local_map_margin_    = nh->has_parameter("sdf_map/local_map_margin") ? nh->get_parameter("sdf_map/local_map_margin").as_int()
                                                                              : nh->declare_parameter<int>("sdf_map/local_map_margin", 1);  // 本地地图边距
    mp_->ground_height_       = nh->has_parameter("sdf_map/ground_height") ? nh->get_parameter("sdf_map/ground_height").as_double()
                                                                           : nh->declare_parameter<double>("sdf_map/ground_height", 1.0);  // 地面高度
    mp_->min_obstacle_height_ = nh->has_parameter("sdf_map/min_obstacle_height")
        ? nh->get_parameter("sdf_map/min_obstacle_height").as_double()
        : nh->declare_parameter<double>("sdf_map/min_obstacle_height", 0.4);
    mp_->max_obstacle_height_ = nh->has_parameter("sdf_map/max_obstacle_height")
        ? nh->get_parameter("sdf_map/max_obstacle_height").as_double()
        : nh->declare_parameter<double>("sdf_map/max_obstacle_height", 0.8);
    if (!std::isfinite(mp_->min_obstacle_height_) || !std::isfinite(mp_->max_obstacle_height_) ||
        mp_->min_obstacle_height_ < 0.0 || mp_->max_obstacle_height_ <= mp_->min_obstacle_height_)
        throw std::invalid_argument("sdf_map obstacle height requires finite 0 <= min < max");
    mp_->default_dist_        = nh->has_parameter("sdf_map/default_dist") ? nh->get_parameter("sdf_map/default_dist").as_double()
                                                                          : nh->declare_parameter<double>("sdf_map/default_dist", 5.0);  // 默认距离
    mp_->optimistic_          = nh->has_parameter("sdf_map/optimistic") ? nh->get_parameter("sdf_map/optimistic").as_bool()
                                                                        : nh->declare_parameter<bool>("sdf_map/optimistic", true);  // 是否使用乐观更新
    mp_->signed_dist_         = nh->has_parameter("sdf_map/signed_dist") ? nh->get_parameter("sdf_map/signed_dist").as_bool()
                                                                         : nh->declare_parameter<bool>("sdf_map/signed_dist", false);  // 是否计算签名距离
    mp_->no_drone_1_          = nh->has_parameter("sdf_map/no_drone_1") ? nh->get_parameter("sdf_map/no_drone_1").as_bool()
                                                                        : nh->declare_parameter<bool>("sdf_map/no_drone_1", false);  // 是否忽略无人机 1

    mp_->agent_footprint_clear_enable_ = nh->has_parameter("map_ros/agent_footprint_clear_enable")
                                             ? nh->get_parameter("map_ros/agent_footprint_clear_enable").as_bool()
                                             : nh->declare_parameter<bool>("map_ros/agent_footprint_clear_enable", true);
    mp_->ego_clear_radius_             = nh->has_parameter("map_ros/ego_clear_radius") ? nh->get_parameter("map_ros/ego_clear_radius").as_double()
                                                                                       : nh->declare_parameter<double>("map_ros/ego_clear_radius", 0.85);
    mp_->ego_body_clear_radius_        = nh->has_parameter("map_ros/ego_body_clear_radius") ? nh->get_parameter("map_ros/ego_body_clear_radius").as_double()
                                                                                            : nh->declare_parameter<double>("map_ros/ego_body_clear_radius", 0.68);
    mp_->swarm_clear_min_dist_         = nh->has_parameter("map_ros/swarm_clear_min_dist") ? nh->get_parameter("map_ros/swarm_clear_min_dist").as_double()
                                                                                           : nh->declare_parameter<double>("map_ros/swarm_clear_min_dist", 0.8);
    mp_->swarm_clear_radius_           = nh->has_parameter("map_ros/swarm_clear_radius") ? nh->get_parameter("map_ros/swarm_clear_radius").as_double()
                                                                                         : nh->declare_parameter<double>("map_ros/swarm_clear_radius", 0.5);
    mp_->swarm_footprint_clear_enable_ = nh->has_parameter("map_ros/swarm_footprint_clear_enable")
                                             ? nh->get_parameter("map_ros/swarm_footprint_clear_enable").as_bool()
                                             : nh->declare_parameter("map_ros/swarm_footprint_clear_enable", true);
    mp_->swarm_clear_stamp_timeout_    = nh->has_parameter("map_ros/swarm_clear_stamp_timeout")
                                             ? nh->get_parameter("map_ros/swarm_clear_stamp_timeout").as_double()
                                             : nh->declare_parameter("map_ros/swarm_clear_stamp_timeout", 0.6);
    mp_->stuck_clear_radius_scale_ = nh->has_parameter("map_ros/stuck_clear_radius_scale") ? nh->get_parameter("map_ros/stuck_clear_radius_scale").as_double()
                                                                                           : nh->declare_parameter("map_ros/stuck_clear_radius_scale", 1.5);

    // 设置分辨率倒数和地图原点
    mp_->local_bound_inflate_ = max(mp_->resolution_, mp_->local_bound_inflate_);
    mp_->resolution_inv_      = 1 / mp_->resolution_;
    mp_->map_origin_          = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_->ground_height_);
    mp_->map_size_            = Eigen::Vector3d(x_size, y_size, z_size);

    // 计算体素数量
    for (int i = 0; i < 3; ++i)
        mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
    mp_->map_min_boundary_ = mp_->map_origin_;
    mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;

    // Params of raycasting-based fusion
    mp_->p_hit_  = nh->has_parameter("sdf_map/p_hit") ? nh->get_parameter("sdf_map/p_hit").as_double()
                                                      : nh->declare_parameter<double>("sdf_map/p_hit", 0.70);  // 体素被击中的概率
    mp_->p_miss_ = nh->has_parameter("sdf_map/p_miss") ? nh->get_parameter("sdf_map/p_miss").as_double()
                                                       : nh->declare_parameter<double>("sdf_map/p_miss", 0.35);  // 体素未被击中的概率
    mp_->p_min_ =
        nh->has_parameter("sdf_map/p_min") ? nh->get_parameter("sdf_map/p_min").as_double() : nh->declare_parameter<double>("sdf_map/p_min", 0.12);  // 最小概率
    mp_->p_max_ =
        nh->has_parameter("sdf_map/p_max") ? nh->get_parameter("sdf_map/p_max").as_double() : nh->declare_parameter<double>("sdf_map/p_max", 0.97);  // 最大概率
    mp_->p_occ_ =
        nh->has_parameter("sdf_map/p_occ") ? nh->get_parameter("sdf_map/p_occ").as_double() : nh->declare_parameter<double>("sdf_map/p_occ", 0.80);  // 占用阈值
    mp_->max_ray_length_      = nh->has_parameter("sdf_map/max_ray_length") ? nh->get_parameter("sdf_map/max_ray_length").as_double()
                                                                            : nh->declare_parameter<double>("sdf_map/max_ray_length", -0.1);  // 最大光线长度
    mp_->virtual_ceil_height_ = nh->has_parameter("sdf_map/virtual_ceil_height")
                                    ? nh->get_parameter("sdf_map/virtual_ceil_height").as_double()
                                    : nh->declare_parameter<double>("sdf_map/virtual_ceil_height", -0.1);  // 虚拟天花板高度

    // 计算对数几率
    auto logit              = [](const double& x) { return log(x / (1 - x)); };
    mp_->prob_hit_log_      = logit(mp_->p_hit_);
    mp_->prob_miss_log_     = logit(mp_->p_miss_);
    mp_->clamp_min_log_     = logit(mp_->p_min_);
    mp_->clamp_max_log_     = logit(mp_->p_max_);
    mp_->min_occupancy_log_ = logit(mp_->p_occ_);
    mp_->unknown_flag_      = 0.01;
    cout << "hit: " << mp_->prob_hit_log_ << ", miss: " << mp_->prob_miss_log_ << ", min: " << mp_->clamp_min_log_ << ", max: " << mp_->clamp_max_log_
         << ", thresh: " << mp_->min_occupancy_log_ << endl;

    // 初始化地图数据缓冲区
    int buffer_size                = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
    md_->occupancy_buffer_         = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);  // 占用概率
    md_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);                                           // 膨胀占用标志
    md_->footprint_cleared_flag_   = vector<char>(buffer_size, 0);                                           // 足迹清除标志
    md_->dynamic_hit_flag_         = vector<char>(buffer_size, 0);                                           // 动态命中标志
    md_->distance_buffer_neg_      = vector<double>(buffer_size, mp_->default_dist_);                        // 负距离缓冲区
    md_->distance_buffer_          = vector<double>(buffer_size, mp_->default_dist_);                        // 距离缓冲区
    md_->count_hit_and_miss_       = vector<short>(buffer_size, 0);                                          // 击中和未击中计数
    md_->count_hit_                = vector<short>(buffer_size, 0);                                          // 击中计数
    md_->count_miss_               = vector<short>(buffer_size, 0);                                          // 未击中计数
    md_->flag_rayend_              = vector<char>(buffer_size, -1);                                          // 光线终点标志
    md_->flag_visited_             = vector<char>(buffer_size, -1);                                          // 访问标志
    md_->tmp_buffer1_              = vector<double>(buffer_size, 0);                                         // 临时缓冲区 1
    md_->tmp_buffer2_              = vector<double>(buffer_size, 0);                                         // 临时缓冲区 2
    md_->raycast_num_              = 0;                                                                      // 光线投射计数
    md_->reset_updated_box_        = true;                                                                   // 重置更新边界标志

    // 获取地图边界框
    vector<string> axis = { "x", "y", "z" };
    for (int i = 0; i < 3; ++i)
    {
        string min_param_name = "sdf_map/box_min_" + axis[i];
        string max_param_name = "sdf_map/box_max_" + axis[i];

        mp_->box_mind_[i] = nh->has_parameter(min_param_name) ? nh->get_parameter(min_param_name).as_double()
                                                              : nh->declare_parameter<double>(min_param_name, mp_->map_min_boundary_[i]);
        mp_->box_maxd_[i] = nh->has_parameter(max_param_name) ? nh->get_parameter(max_param_name).as_double()
                                                              : nh->declare_parameter<double>(max_param_name, mp_->map_max_boundary_[i]);
    }
    posToIndex(mp_->box_mind_, mp_->box_min_);
    posToIndex(mp_->box_maxd_, mp_->box_max_);

    // 加载 partitioning/inactive_zone 参数，把死区几何同步到 SDF 层
    loadInactiveZones(nh);
    // 死区实体化：把死区导航柱写为 OCCUPIED，让前沿/视点/EDT/A* 统一看到死区
    seedInactiveZonesIntoOccupancy();

    // 初始化 ROS 包装器和多地图管理器
    mr_->setMap(this);
    mr_->node_ = nh;
    mr_->init();

    mm_->setMap(this);
    mm_->node_ = nh;
    mm_->init();

    // 初始化光线投射器
    caster_.reset(new RayCaster);
    caster_->setParams(mp_->resolution_, mp_->map_origin_);

    // 初始化全局边界
    for (int i = 0; i < 3; ++i)
    {
        md_->all_min_[i] = 1000000;
        md_->all_max_[i] = -1000000;
    }

    // 计算初始顶点、边界框和法向量
    mp_->rot_sw_ << cos(0.0), -sin(0.0), 0, sin(0.0), cos(0.0), 0, 0, 0, 1;
    mp_->trans_sw_ << 0, 0, 0;

    Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;
    left_bottom = mp_->box_mind_;
    right_top   = mp_->box_maxd_;

    left_top[0]     = left_bottom[0];
    left_top[1]     = right_top[1];
    left_top[2]     = left_bottom[2];
    right_bottom[0] = right_top[0];
    right_bottom[1] = left_bottom[1];
    right_bottom[2] = left_bottom[2];
    right_top[2]    = left_bottom[2];

    mp_->vertices_ = { left_bottom, right_bottom, right_top, left_top };
    if (mp_->use_swarm_tf_)
    {
        for (auto& vert : mp_->vertices_)
            vert = mp_->rot_sw_ * vert + mp_->trans_sw_;
    }

    mp_->vmin_ = mp_->vmax_ = mp_->vertices_[0];
    for (int j = 1; j < mp_->vertices_.size(); ++j)
    {
        for (int k = 0; k < 2; ++k)
        {
            mp_->vmin_[k] = min(mp_->vmin_[k], mp_->vertices_[j][k]);
            mp_->vmax_[k] = max(mp_->vmax_[k], mp_->vertices_[j][k]);
        }
    }

    for (int j = 0; j < 4; ++j)
    {
        Eigen::Vector3d dir = (mp_->vertices_[(j + 1) % 4] - mp_->vertices_[j]).normalized();
        mp_->normals_.push_back(dir);
    }
}

void SDFMap::resetBuffer()
{
    resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
    md_->local_bound_min_ = Eigen::Vector3i::Zero();
    md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos)
{
    Eigen::Vector3i min_id, max_id;
    posToIndex(min_pos, min_id);
    posToIndex(max_pos, max_id);
    boundIndex(min_id);
    boundIndex(max_id);

    for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
            for (int z = min_id(2); z <= max_id(2); ++z)
            {
                md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
                md_->footprint_cleared_flag_[toAddress(x, y, z)]   = 0;
                md_->dynamic_hit_flag_[toAddress(x, y, z)]         = 0;
                md_->distance_buffer_[toAddress(x, y, z)]          = mp_->default_dist_;
            }
}

template <typename F_get_val, typename F_set_val> void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
    int    v[mp_->map_voxel_num_(dim)];
    double z[mp_->map_voxel_num_(dim) + 1];

    int k        = start;
    v[start]     = start;
    z[start]     = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();

    for (int q = start + 1; q <= end; q++)
    {
        k++;
        double s;

        do
        {
            k--;
            s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
        } while (s <= z[k]);

        k++;

        v[k]     = q;
        z[k]     = s;
        z[k + 1] = std::numeric_limits<double>::max();
    }

    k = start;

    for (int q = start; q <= end; q++)
    {
        while (z[k + 1] < q)
            k++;
        double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
        f_set_val(q, val);
    }
}

void SDFMap::updateESDF3d()
{
    Eigen::Vector3i min_esdf = md_->local_bound_min_;
    Eigen::Vector3i max_esdf = md_->local_bound_max_;

    if (mp_->optimistic_)
    {
        for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
            {
                fillESDF([&](int z) { return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ? 0 : std::numeric_limits<double>::max(); },
                         [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2], max_esdf[2], 2);
            }
    }
    else
    {
        for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
            {
                fillESDF(
                    [&](int z) {
                        int adr = toAddress(x, y, z);
                        return (md_->occupancy_buffer_inflate_[adr] == 1 || md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
                                   ? 0
                                   : std::numeric_limits<double>::max();
                    },
                    [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2], max_esdf[2], 2);
            }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
        {
            fillESDF([&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; }, [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; },
                     min_esdf[1], max_esdf[1], 1);
        }
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
        for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
        {
            fillESDF([&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
                     [&](int x, double val) { md_->distance_buffer_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val); }, min_esdf[0], max_esdf[0], 0);
        }

    if (mp_->signed_dist_)
    {
        // Compute negative distance
        for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
            {
                fillESDF(
                    [&](int z) {
                        return md_->occupancy_buffer_inflate_[x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z] == 0
                                   ? 0
                                   : std::numeric_limits<double>::max();
                    },
                    [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2], max_esdf[2], 2);
            }
        for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
            {
                fillESDF([&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; }, [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; },
                         min_esdf[1], max_esdf[1], 1);
            }
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
            {
                fillESDF([&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
                         [&](int x, double val) { md_->distance_buffer_neg_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val); }, min_esdf[0],
                         max_esdf[0], 0);
            }
        // Merge negative distance with positive
        for (int x = min_esdf(0); x <= max_esdf(0); ++x)
            for (int y = min_esdf(1); y <= max_esdf(1); ++y)
                for (int z = min_esdf(2); z <= max_esdf(2); ++z)
                {
                    int idx = toAddress(x, y, z);
                    if (md_->distance_buffer_neg_[idx] > 0.0)
                        md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
                }
    }
}

void SDFMap::setCacheOccupancy(const int& adr, const int& occ)
{
    // Add to update list if first visited
    if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0)
        md_->cache_voxel_.push(adr);

    if (occ == 0)
        md_->count_miss_[adr] = 1;
    else /* if (occ == 1) */
        md_->count_hit_[adr] += 1;

    // md_->count_hit_and_miss_[adr] += 1;
    // if (occ == 1)
    //   md_->count_hit_[adr] += 1;
    // if (md_->count_hit_and_miss_[adr] == 1)
    //   md_->cache_voxel_.push(adr);
}

void SDFMap::integrateExternalCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const Eigen::Vector3d& sensor_pos)
{
    if (!isInMap(sensor_pos) || points.empty()) return;
    mm_->drone_pos_ = sensor_pos;
    inputPointCloud(points, static_cast<int>(points.size()), sensor_pos);
    if (mr_->local_updated_) {
        clearAndInflateLocalMap();
        updateESDF3d();
        mr_->local_updated_ = false;
        mr_->esdf_need_update_ = false;
    }
}

void SDFMap::inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num, const Eigen::Vector3d& camera_pos)
{
    if (point_num == 0)
        return;
    md_->raycast_num_ += 1;

    Eigen::Vector3d update_min = camera_pos;
    Eigen::Vector3d update_max = camera_pos;
    if (md_->reset_updated_box_)
    {
        md_->update_min_        = camera_pos;
        md_->update_max_        = camera_pos;
        md_->reset_updated_box_ = false;
    }

    Eigen::Vector3d pt_w, tmp;
    Eigen::Vector3i idx;
    int             vox_adr;
    double          length;
    for (int i = 0; i < point_num; ++i)
    {
        auto& pt = points.points[i];
        pt_w << pt.x, pt.y, pt.z;
        int tmp_flag;
        // Set flag for projected point
        if (!isInMap(pt_w))
        {
            // Find closest point in map and set free
            pt_w   = closetPointInMap(pt_w, camera_pos);
            length = (pt_w - camera_pos).norm();
            if (length > mp_->max_ray_length_)
                pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
            if (pt_w[2] < 0.2)
                continue;
            tmp_flag = 0;
        }
        else
        {
            length = (pt_w - camera_pos).norm();
            if (length > mp_->max_ray_length_)
            {
                pt_w = (pt_w - camera_pos) / length * mp_->max_ray_length_ + camera_pos;
                if (pt_w[2] < 0.15)
                    continue;
                tmp_flag = 0;
            }
            else
                tmp_flag = 1;
        }
        // Out-of-band returns still terminate real clearing rays; they never
        // mark obstacles or enter the ESDF used by trajectory optimization.
        const double height_above_ground = pt_w.z() - mp_->ground_height_;
        if (height_above_ground < mp_->min_obstacle_height_ ||
            height_above_ground > mp_->max_obstacle_height_)
            tmp_flag = 0;
        posToIndex(pt_w, idx);
        vox_adr = toAddress(idx);
        setCacheOccupancy(vox_adr, tmp_flag);

        for (int k = 0; k < 3; ++k)
        {
            update_min[k] = min(update_min[k], pt_w[k]);
            update_max[k] = max(update_max[k], pt_w[k]);
        }
        // Raycasting between camera center and point
        if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
            continue;
        else
            md_->flag_rayend_[vox_adr] = md_->raycast_num_;

        caster_->input(pt_w, camera_pos);
        caster_->nextId(idx);
        while (caster_->nextId(idx))
            setCacheOccupancy(toAddress(idx), 0);
    }

    Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
    posToIndex(update_max + bound_inf, md_->local_bound_max_);
    posToIndex(update_min - bound_inf, md_->local_bound_min_);
    boundIndex(md_->local_bound_min_);
    boundIndex(md_->local_bound_max_);
    mr_->local_updated_ = true;

    // Bounding box for subsequent updating and overall updateing
    for (int k = 0; k < 3; ++k)
    {
        md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
        md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);
        md_->all_min_[k]    = min(update_min[k], md_->all_min_[k]);
        md_->all_max_[k]    = max(update_max[k], md_->all_max_[k]);
    }

    // Update map data
    vector<uint32_t> new_voxel_ids_;
    const double     dynamic_hit_r2 = std::pow(mp_->swarm_clear_radius_ + 0.3, 2);  // 车体半径 + 状态误差裕量
    while (!md_->cache_voxel_.empty())
    {
        int adr = md_->cache_voxel_.front();
        md_->cache_voxel_.pop();
        const bool   is_hit          = md_->count_hit_[adr] >= md_->count_miss_[adr];
        const double log_odds_update = is_hit ? mp_->prob_hit_log_ : mp_->prob_miss_log_;
        md_->count_hit_[adr] = md_->count_miss_[adr] = 0;
        const bool was_unknown                       = md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3;
        const bool was_occupied                      = md_->occupancy_buffer_[adr] > mp_->min_occupancy_log_;
        if (was_unknown)
        {
            md_->occupancy_buffer_[adr] = mp_->min_occupancy_log_;
            new_voxel_ids_.push_back(adr);
        }

        // 动态命中快速清除：体素曾被标记为“队友车体命中”。miss 连续两帧确认后
        // 直接置 FREE（绕过概率衰减），保证车头再次扫到无车区域时足迹立即消除；
        // 单帧 miss（传感器抖动/遮挡丢点）只进入待确认态，避免误清仍在原地的车体。
        if (!is_hit && md_->dynamic_hit_flag_[adr])
        {
            if (md_->dynamic_hit_flag_[adr] >= 2)
            {
                md_->dynamic_hit_flag_[adr] = 0;
                md_->occupancy_buffer_[adr] = mp_->clamp_min_log_;
            }
            else
            {
                md_->dynamic_hit_flag_[adr] = 2;
                md_->occupancy_buffer_[adr] = std::min(std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_), mp_->clamp_max_log_);
            }
        }
        else
        {
            if (is_hit && md_->dynamic_hit_flag_[adr] == 2)
                md_->dynamic_hit_flag_[adr] = 1;  // 车仍在原地，恢复标记
            md_->occupancy_buffer_[adr] = std::min(std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_), mp_->clamp_max_log_);
        }

        // 占用状态翻转（occupied<->free）的体素必须重新进入 chunk 流：否则地面站
        // 可视化只能收到该体素首次发现时的状态，动态障碍物（如走开的小车）的
        // 足迹会永久残留在地图上。
        if (!was_unknown && was_occupied != (md_->occupancy_buffer_[adr] > mp_->min_occupancy_log_))
            new_voxel_ids_.push_back(adr);

        // hit 使体素从 free/unknown 翻转为 occupied 时：落点位于任一队友车体附近
        // → 标记为动态命中；否则清除标记（真实障碍覆盖，避免误用快速清除）。
        if (is_hit && md_->occupancy_buffer_[adr] > mp_->min_occupancy_log_ && (was_unknown || !was_occupied))
        {
            bool near_peer = false;
            if (!peer_states_.empty())
            {
                Eigen::Vector3i hit_idx;
                const int       yz_dim = mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
                if (yz_dim > 0 && mp_->map_voxel_num_(2) > 0)  // 防御：地图尺寸未初始化
                {
                    hit_idx(0) = adr / yz_dim;
                    hit_idx(1) = (adr % yz_dim) / mp_->map_voxel_num_(2);
                    hit_idx(2) = adr % mp_->map_voxel_num_(2);
                    Eigen::Vector3d hit_pos;
                    indexToPos(hit_idx, hit_pos);
                    for (const auto& entry : peer_states_)
                    {
                        const double dx = hit_pos.x() - entry.second(0);
                        const double dy = hit_pos.y() - entry.second(1);
                        const double dz = hit_pos.z() - entry.second(2);
                        if (dx * dx + dy * dy + dz * dz <= dynamic_hit_r2)
                        {
                            near_peer = true;
                            break;
                        }
                    }
                }
            }
            md_->dynamic_hit_flag_[adr] = near_peer ? 1 : 0;
        }
    }

    // 激光会把死区（物理上是空域）重新标为 FREE，必须在占用写入之后、膨胀/ESDF 更新之前重新断言
    reassertInactiveZonesInLocalBox();

    mm_->updateMapChunk(new_voxel_ids_);
}

Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt)
{
    Eigen::Vector3d diff   = pt - camera_pt;
    Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
    Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
    double          min_t  = 1000000;
    for (int i = 0; i < 3; ++i)
    {
        if (fabs(diff[i]) > 0)
        {
            double t1 = max_tc[i] / diff[i];
            if (t1 > 0 && t1 < min_t)
                min_t = t1;
            double t2 = min_tc[i] / diff[i];
            if (t2 > 0 && t2 < min_t)
                min_t = t2;
        }
    }
    return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap()
{
    // /*clear outside local*/
    // const int vec_margin = 5;

    // Eigen::Vector3i min_cut = md_->local_bound_min_ -
    //     Eigen::Vector3i(mp_->local_map_margin_, mp_->local_map_margin_,
    //     mp_->local_map_margin_);
    // Eigen::Vector3i max_cut = md_->local_bound_max_ +
    //     Eigen::Vector3i(mp_->local_map_margin_, mp_->local_map_margin_,
    //     mp_->local_map_margin_);
    // boundIndex(min_cut);
    // boundIndex(max_cut);

    // Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin,
    // vec_margin); Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin,
    // vec_margin, vec_margin); boundIndex(min_cut_m); boundIndex(max_cut_m);

    // // clear data outside the local range

    // for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    //   for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

    //     for (int z = min_cut_m(2); z < min_cut(2); ++z) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }

    //     for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }
    //   }

    // for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    //   for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

    //     for (int y = min_cut_m(1); y < min_cut(1); ++y) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }

    //     for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }
    //   }

    // for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    //   for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

    //     for (int x = min_cut_m(0); x < min_cut(0); ++x) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }

    //     for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
    //       int idx                       = toAddress(x, y, z);
    //       md_->occupancy_buffer_[idx]    = mp_->clamp_min_log_ - mp_->unknown_flag_;
    //       md_->distance_buffer_all_[idx] = 10000;
    //     }
    //   }

    // update inflated occupied cells
    // clean outdated occupancy

    int                     inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
    vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
    // inf_pts.resize(4 * inf_step + 3);

    for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
        for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
            for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z)
            {
                md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
            }

    // inflate newest occpuied cells
    for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
        for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
            for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z)
            {
                int id1 = toAddress(x, y, z);
                if (md_->occupancy_buffer_[id1] > mp_->min_occupancy_log_)
                {
                    for (int inf_x = -inf_step; inf_x <= inf_step; ++inf_x)
                        for (int inf_y = -inf_step; inf_y <= inf_step; ++inf_y)
                            for (int inf_z = -inf_step; inf_z <= inf_step; ++inf_z)
                            {
                                Eigen::Vector3i inf_pt(x + inf_x, y + inf_y, z + inf_z);
                                if (!isInMap(inf_pt))
                                    continue;
                                int inf_adr                             = toAddress(inf_pt);
                                md_->occupancy_buffer_inflate_[inf_adr] = 1;
                            }

                    //   inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
                    //   for (auto inf_pt : inf_pts) {
                    //     if (!isInMap(inf_pt)) continue;
                    //     int idx_inf = toAddress(inf_pt);
                    //     if (idx_inf >= 0 &&
                    //         idx_inf <
                    //             mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)) {
                    //       md_->occupancy_buffer_inflate_[idx_inf] = 1;
                    //     }
                    //   }
                }
            }
}

double SDFMap::getResolution()
{
    return mp_->resolution_;
}

int SDFMap::getVoxelNum()
{
    return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
}

bool SDFMap::getBaseCoor(const int& id, Eigen::Vector4d& transform)
{
    if (mp_->no_drone_1_ && id == 1)
    {
        transform = Eigen::Vector4d(0, 0, 0, 0);
        return true;
    }

    auto iter = md_->swarm_transform_.find(id);
    if (iter == md_->swarm_transform_.end())
    {
        RCLCPP_ERROR_STREAM(mr_->node_->get_logger(), "No transform of drone " << id << " yet");
        return false;
    }
    transform = iter->second;
    return true;
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size)
{
    ori = mp_->map_origin_, size = mp_->map_size_;
}

void SDFMap::getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax)
{
    bmin = mp_->box_mind_;
    bmax = mp_->box_maxd_;
    // bmin = mp_->vmin_;
    // bmax = mp_->vmax_;
    // bmin[2] = mp_->box_mind_[2];
    // bmax[2] = mp_->box_maxd_[2];
}

void SDFMap::getExploredBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) const
{
    bmin = md_->all_min_;
    bmax = md_->all_max_;
}

void SDFMap::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset)
{
    bmin = md_->update_min_;
    bmax = md_->update_max_;
    if (reset)
        md_->reset_updated_box_ = true;
}

double SDFMap::getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad)
{
    if (!isInMap(pos))
    {
        grad.setZero();
        return 0;
    }

    /* trilinear interpolation */
    Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
    Eigen::Vector3i idx;
    posToIndex(pos_m, idx);
    Eigen::Vector3d idx_pos, diff;
    indexToPos(idx, idx_pos);
    diff = (pos - idx_pos) * mp_->resolution_inv_;

    double values[2][2][2];
    for (int x = 0; x < 2; x++)
        for (int y = 0; y < 2; y++)
            for (int z = 0; z < 2; z++)
            {
                Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                values[x][y][z]             = getDistance(current_idx);
            }

    double v00  = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
    double v01  = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
    double v10  = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
    double v11  = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
    double v0   = (1 - diff[1]) * v00 + diff[1] * v10;
    double v1   = (1 - diff[1]) * v01 + diff[1] * v11;
    double dist = (1 - diff[2]) * v0 + diff[2] * v1;

    grad[2] = (v1 - v0) * mp_->resolution_inv_;
    grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_->resolution_inv_;
    grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
    grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
    grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
    grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
    grad[0] *= mp_->resolution_inv_;

    return dist;
}

void SDFMap::getNavColumnZRange(int& z_start, int& z_end) const
{
    const int depth = mp_->map_voxel_num_(2);
    z_start         = 0;
    z_end           = depth > 0 ? depth - 1 : 0;
    if (depth <= 0)
        return;

    const double min_detect_height = mp_->ground_height_ + mp_->min_obstacle_height_;
    const double max_detect_height = mp_->ground_height_ + mp_->max_obstacle_height_;
    z_start                        = static_cast<int>(std::ceil((min_detect_height - mp_->map_origin_[2]) * mp_->resolution_inv_));
    z_end                          = static_cast<int>(std::floor((max_detect_height - mp_->map_origin_[2]) * mp_->resolution_inv_));
    z_start                        = std::max(0, std::min(z_start, depth - 1));
    z_end                          = std::max(z_start, std::min(z_end, depth - 1));
}

void SDFMap::clearAgentFootprintAt(const Eigen::Vector3d& pos, const double radius_m, const double radius_scale, vector<uint32_t>* flipped_free_adrs)
{
    if (!mp_->agent_footprint_clear_enable_)
        return;

    const double radius = radius_m * std::max(0.1, radius_scale);
    if (radius <= 1e-3)
        return;

    int z_start = 0;
    int z_end   = 0;
    getNavColumnZRange(z_start, z_end);

    Eigen::Vector3i center_idx;
    posToIndex(pos, center_idx);

    const int    r_cells = static_cast<int>(std::ceil(radius * mp_->resolution_inv_));
    const double r_sq    = radius * radius;
    // 车体 footprint 强制清除半径（覆盖 1.1x0.8 车体半对角），不超过本次清除半径。
    const double body_r    = std::min(radius, std::max(0.0, mp_->ego_body_clear_radius_));
    const double body_r_sq = body_r * body_r;
    const double free_val  = mp_->clamp_min_log_;

    // 清除把占用体素直接置 FREE，绕过概率更新与翻转检测；被清除的体素同时
    // 释放动态命中标记（车体已被清除处理，后续 miss 无需快速清除）。
    for (int dx = -r_cells; dx <= r_cells; ++dx)
    {
        for (int dy = -r_cells; dy <= r_cells; ++dy)
        {
            const int vx = center_idx(0) + dx;
            const int vy = center_idx(1) + dy;
            if (vx < 0 || vy < 0 || vx >= mp_->map_voxel_num_(0) || vy >= mp_->map_voxel_num_(1))
                continue;

            Eigen::Vector3d col_pos;
            indexToPos(Eigen::Vector3i(vx, vy, z_start), col_pos);
            const double dist2 = (col_pos.head<2>() - pos.head<2>()).squaredNorm();
            if (dist2 > r_sq)
                continue;

            const bool under_body = dist2 <= body_r_sq;

            for (int z = z_start; z <= z_end; ++z)
            {
                const int adr = toAddress(vx, vy, z);
                if (adr < 0 || adr >= static_cast<int>(md_->occupancy_buffer_.size()))
                    continue;
                // 静态障碍保护：只清车体 footprint 内的自身回波与动态命中体素；
                // 半径内其余静态墙必须保留，否则 SNN 规划与 0.3m 检测会同时失明。
                if (!under_body && md_->dynamic_hit_flag_[adr] == 0)
                    continue;
                if (md_->occupancy_buffer_[adr] > mp_->min_occupancy_log_)
                {
                    // 曾被标记为“队友车体命中”的体素被清除置 FREE：收集 occupied→free 翻转，
                    // 由调用方经 updateMapChunk 发布，撤销本车在地面站的 occupied 引用（本车开过
                    // 队友旧足迹后，地面站不再残留）。静态障碍（墙等）无动态命中标记，不收集。
                    if (flipped_free_adrs && md_->dynamic_hit_flag_[adr])
                        flipped_free_adrs->push_back(adr);
                    md_->dynamic_hit_flag_[adr] = 0;  // 释放动态命中标记
                }
                md_->occupancy_buffer_[adr]         = free_val;
                md_->occupancy_buffer_inflate_[adr] = 0;
                md_->footprint_cleared_flag_[adr]   = 1;  // 持久标记：该体素曾被动态足迹清除（供 frontier 抑制）
            }
        }
    }

    DynamicFootprintZone zone;
    zone.cx     = pos.x();
    zone.cy     = pos.y();
    zone.radius = radius;
    dynamic_footprint_zones_.push_back(zone);
}

void SDFMap::clearDynamicAgentFootprints(const Eigen::Vector3d& ego_pos, const double ego_radius, const std::vector<Eigen::Vector3d>& peer_positions,
                                         const double peer_radius, const double radius_scale)
{
    if (!mp_->agent_footprint_clear_enable_)
        return;

    dynamic_footprint_zones_.clear();
    clearAgentFootprintAt(ego_pos, ego_radius, radius_scale);
    if (!mp_->swarm_footprint_clear_enable_)
        return;

    for (const auto& peer_pos : peer_positions)
        clearAgentFootprintAt(peer_pos, peer_radius, radius_scale);
}

void SDFMap::applyDynamicFootprintClearFromMapROS(const Eigen::Vector3d& ego_pos, const std::map<int, Eigen::Vector4d>& drone_states,
                                                  const std::map<int, Eigen::Vector3d>& peer_vels, const int self_drone_id, const double radius_scale)
{
    if (!mp_->agent_footprint_clear_enable_)
        return;

    dynamic_footprint_zones_.clear();
    // ego 清除收集翻转体素：本车开过队友旧足迹时，清除圆内的占用体素被置 FREE，
    // 同步发布可撤销本车在地面站的 occupied 引用（否则该引用只能等本车重扫才能
    // 清除）。peer 清除不收集：车体仍被激光反复 hit，发布会造成翻转风暴。
    vector<uint32_t> flipped_free_adrs;
    clearAgentFootprintAt(ego_pos, mp_->ego_clear_radius_, radius_scale, &flipped_free_adrs);

    if (!mp_->swarm_footprint_clear_enable_)
    {
        if (!flipped_free_adrs.empty())
            mm_->updateMapChunk(flipped_free_adrs);
        return;
    }

    for (const auto& entry : drone_states)
    {
        if (entry.first == self_drone_id)
            continue;
        // 只清除移动中的队友（速度门控）：停止车是真实静态障碍，保留占用供规划避开。
        const auto it_vel = peer_vels.find(entry.first);
        if (it_vel == peer_vels.end())
            continue;  // 无速度信息：保守视为停止，不清除
        if (it_vel->second.head<2>().norm() <= kDynamicClearSpeedEps)
            continue;
        const Eigen::Vector3d peer_pos(entry.second(0), entry.second(1), entry.second(2));
        const double          dx    = peer_pos.x() - ego_pos.x();
        const double          dy    = peer_pos.y() - ego_pos.y();
        const double          dz    = peer_pos.z() - ego_pos.z();
        const double          dist2 = dx * dx + dy * dy + dz * dz;
        if (dist2 <= mp_->swarm_clear_min_dist_ * mp_->swarm_clear_min_dist_)
            continue;
        clearAgentFootprintAt(peer_pos, mp_->swarm_clear_radius_, radius_scale);
    }

    if (!flipped_free_adrs.empty())
        mm_->updateMapChunk(flipped_free_adrs);
}

void SDFMap::setPeerStates(const std::map<int, Eigen::Vector4d>& peer_states)
{
    peer_states_ = peer_states;
}

bool SDFMap::isInDynamicAgentFootprint(const Eigen::Vector3d& pos) const
{
    if (!mp_->agent_footprint_clear_enable_ || dynamic_footprint_zones_.empty())
        return false;

    for (const auto& zone : dynamic_footprint_zones_)
    {
        const double dx = pos.x() - zone.cx;
        const double dy = pos.y() - zone.cy;
        if (dx * dx + dy * dy <= zone.radius * zone.radius)
            return true;
    }
    return false;
}

void SDFMap::clearDynamicAgentFootprintsForPlanning(const Eigen::Vector3d& ego_pos, const std::vector<Eigen::Vector3d>& peer_positions,
                                                    const double radius_scale)
{
    clearDynamicAgentFootprints(ego_pos, mp_->ego_clear_radius_, peer_positions, mp_->swarm_clear_radius_, radius_scale);
}

bool SDFMap::agentFootprintClearEnabled() const
{
    return mp_->agent_footprint_clear_enable_;
}

double SDFMap::stuckClearRadiusScale() const
{
    return mp_->stuck_clear_radius_scale_;
}

double SDFMap::swarmClearStampTimeout() const
{
    return mp_->swarm_clear_stamp_timeout_;
}

double SDFMap::swarmClearMinDist() const
{
    return mp_->swarm_clear_min_dist_;
}

void SDFMap::loadInactiveZones(rclcpp::Node::SharedPtr nh)
{
    mp_->inactive_zone_enable_ = nh->has_parameter("partitioning/inactive_zone_enable")
                                     ? nh->get_parameter("partitioning/inactive_zone_enable").as_bool()
                                     : nh->declare_parameter<bool>("partitioning/inactive_zone_enable", false);

    auto load_rect = [&](int idx, const std::string& prefix, bool enable_default) {
        mp_->inactive_zones_[idx].enable = nh->has_parameter(prefix + "_enable") ? nh->get_parameter(prefix + "_enable").as_bool()
                                                                                 : nh->declare_parameter<bool>(prefix + "_enable", enable_default);
        mp_->inactive_zones_[idx].x_min =
            nh->has_parameter(prefix + "_x_min") ? nh->get_parameter(prefix + "_x_min").as_double() : nh->declare_parameter<double>(prefix + "_x_min", 0.0);
        mp_->inactive_zones_[idx].x_max =
            nh->has_parameter(prefix + "_x_max") ? nh->get_parameter(prefix + "_x_max").as_double() : nh->declare_parameter<double>(prefix + "_x_max", 0.0);
        mp_->inactive_zones_[idx].y_min =
            nh->has_parameter(prefix + "_y_min") ? nh->get_parameter(prefix + "_y_min").as_double() : nh->declare_parameter<double>(prefix + "_y_min", 0.0);
        mp_->inactive_zones_[idx].y_max =
            nh->has_parameter(prefix + "_y_max") ? nh->get_parameter(prefix + "_y_max").as_double() : nh->declare_parameter<double>(prefix + "_y_max", 0.0);
    };

    load_rect(0, "partitioning/inactive_zone", false);
    load_rect(1, "partitioning/inactive_zone2", false);

    RCLCPP_INFO(nh->get_logger(), "[SDFMap] inactive_zone enable=%d, zones=[(%.2f,%.2f)x(%.2f,%.2f)] [(%.2f,%.2f)x(%.2f,%.2f)]",
                static_cast<int>(mp_->inactive_zone_enable_), mp_->inactive_zones_[0].x_min, mp_->inactive_zones_[0].x_max, mp_->inactive_zones_[0].y_min,
                mp_->inactive_zones_[0].y_max, mp_->inactive_zones_[1].x_min, mp_->inactive_zones_[1].x_max, mp_->inactive_zones_[1].y_min,
                mp_->inactive_zones_[1].y_max);
}

bool SDFMap::inInactiveZone(double x, double y) const
{
    if (!mp_->inactive_zone_enable_)
        return false;
    auto in_rect = [](const MapParam::InactiveZoneRect& z, double px, double py) {
        const double xmin = std::min(z.x_min, z.x_max);
        const double xmax = std::max(z.x_min, z.x_max);
        const double ymin = std::min(z.y_min, z.y_max);
        const double ymax = std::max(z.y_min, z.y_max);
        return (px >= xmin && px <= xmax && py >= ymin && py <= ymax);
    };
    if (mp_->inactive_zones_[0].enable && in_rect(mp_->inactive_zones_[0], x, y))
        return true;
    if (mp_->inactive_zones_[1].enable && in_rect(mp_->inactive_zones_[1], x, y))
        return true;
    return false;
}

void SDFMap::seedInactiveZonesIntoOccupancy()
{
    if (!mp_->inactive_zone_enable_)
        return;
    int z_start = 0, z_end = 0;
    getNavColumnZRange(z_start, z_end);
    if (z_end < z_start)
        return;

    for (int zi = 0; zi < 2; ++zi)
    {
        const auto& z = mp_->inactive_zones_[zi];
        if (!z.enable)
            continue;
        Eigen::Vector3i min_id, max_id;
        posToIndex(Eigen::Vector3d(std::min(z.x_min, z.x_max), std::min(z.y_min, z.y_max), 0.0), min_id);
        posToIndex(Eigen::Vector3d(std::max(z.x_min, z.x_max), std::max(z.y_min, z.y_max), 0.0), max_id);
        boundIndex(min_id);
        boundIndex(max_id);
        for (int x = min_id(0); x <= max_id(0); ++x)
            for (int y = min_id(1); y <= max_id(1); ++y)
                for (int zz = z_start; zz <= z_end; ++zz)
                {
                    const int adr = toAddress(x, y, zz);
                    if (adr >= 0 && adr < static_cast<int>(md_->occupancy_buffer_.size()))
                        md_->occupancy_buffer_[adr] = mp_->clamp_max_log_;
                }
    }
}

void SDFMap::reassertInactiveZonesInLocalBox()
{
    if (!mp_->inactive_zone_enable_)
        return;
    int z_start = 0, z_end = 0;
    getNavColumnZRange(z_start, z_end);
    if (z_end < z_start)
        return;

    for (int zi = 0; zi < 2; ++zi)
    {
        const auto& z = mp_->inactive_zones_[zi];
        if (!z.enable)
            continue;
        Eigen::Vector3i min_id, max_id;
        posToIndex(Eigen::Vector3d(std::min(z.x_min, z.x_max), std::min(z.y_min, z.y_max), 0.0), min_id);
        posToIndex(Eigen::Vector3d(std::max(z.x_min, z.x_max), std::max(z.y_min, z.y_max), 0.0), max_id);
        boundIndex(min_id);
        boundIndex(max_id);

        // 只断言局部更新盒内的死区列，避免全图开销
        const int lx0 = std::max(min_id(0), md_->local_bound_min_(0));
        const int lx1 = std::min(max_id(0), md_->local_bound_max_(0));
        const int ly0 = std::max(min_id(1), md_->local_bound_min_(1));
        const int ly1 = std::min(max_id(1), md_->local_bound_max_(1));
        if (lx0 > lx1 || ly0 > ly1)
            continue;

        for (int x = lx0; x <= lx1; ++x)
            for (int y = ly0; y <= ly1; ++y)
                for (int zz = z_start; zz <= z_end; ++zz)
                {
                    const int adr = toAddress(x, y, zz);
                    if (adr >= 0 && adr < static_cast<int>(md_->occupancy_buffer_.size()))
                        md_->occupancy_buffer_[adr] = mp_->clamp_max_log_;
                }
    }
}

bool SDFMap::isColumnOccupied(int voxel_x, int voxel_y)
{
    const int width  = mp_->map_voxel_num_(0);
    const int height = mp_->map_voxel_num_(1);
    const int depth  = mp_->map_voxel_num_(2);
    if (voxel_x < 0 || voxel_y < 0 || voxel_x >= width || voxel_y >= height || depth <= 0)
    {
        return true;
    }

    // 死区实体化：只要该列中心落在 inactive_zone 矩形内，即视为占据
    {
        Eigen::Vector3i center_idx(voxel_x, voxel_y, 0);
        Eigen::Vector3d center_pos;
        indexToPos(center_idx, center_pos);
        if (inInactiveZone(center_pos.x(), center_pos.y()))
        {
            return true;
        }
    }

    int z_start = 0;
    int z_end   = 0;
    getNavColumnZRange(z_start, z_end);

    for (int z = z_start; z <= z_end; ++z)
    {
        const int adr = toAddress(voxel_x, voxel_y, z);
        if (adr < 0 || adr >= static_cast<int>(md_->occupancy_buffer_inflate_.size()))
        {
            continue;
        }

        if (md_->occupancy_buffer_inflate_[adr] > 0 || md_->occupancy_buffer_[adr] > mp_->min_occupancy_log_)
        {
            return true;
        }
    }
    return false;
}

bool SDFMap::isSegmentColumnFree(const Eigen::Vector3d& start, const Eigen::Vector3d& end, bool block_unknown)
{
    const double    nav_z = 0.3;  //碰撞检测射线高度
    Eigen::Vector3d p0    = start;
    Eigen::Vector3d p1    = end;
    p0.z()                = nav_z;
    p1.z()                = nav_z;

    if (!caster_->input(p0, p1))
    {
        Eigen::Vector3i start_idx;
        posToIndex(p0, start_idx);
        return !isColumnOccupied(start_idx(0), start_idx(1));
    }

    Eigen::Vector3i idx;
    while (caster_->nextId(idx))
    {
        if (isColumnOccupied(idx(0), idx(1)))
        {
            return false;
        }
        if (block_unknown)
        {
            int z_start = 0, z_end = 0;
            getNavColumnZRange(z_start, z_end);
            for (int z = z_start; z <= z_end; ++z)
            {
                if (getOccupancy(Eigen::Vector3i(idx(0), idx(1), z)) == UNKNOWN)
                {
                    return false;
                }
            }
        }
    }
    return true;
}

std::vector<std::vector<int>> SDFMap::export2DGrid(const Eigen::Vector3d& pos, int target_z_layer, int deadzone_margin_cells)
{
    (void)target_z_layer;
    // 自车足迹已在 3D occupancy 层按“车体正下方 + 动态命中”精确清除；
    // 这里不能再按自车位置做无条件方形置 0，否则会把车前 0.7m 内的静态墙
    // 从 SNN 地图上抹掉（规划看不到、检测看得到，形成原地判碰活锁）。
    (void)pos;
    int width  = mp_->map_voxel_num_(0);  // X 方向长度
    int height = mp_->map_voxel_num_(1);  // Y 方向长度
    int depth  = mp_->map_voxel_num_(2);  // Z 方向长度

    if (width <= 0 || height <= 0 || depth <= 0)
    {
        RCLCPP_ERROR_STREAM(mr_->node_->get_logger(), "[export2DGrid] Invalid map size: " << width << "x" << height << "x" << depth);
        return {};
    }

    // ==================== 创建网格 ====================
    // 为了实现：向上为 X (Rows)，向左为 Y (Cols)
    // grid[x][y_idx]
    // 行数 = width, 列数 = height
    std::vector<std::vector<int>> grid(width, std::vector<int>(height, 0));

    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            // 1. 坐标轴方向处理：
            // 我们希望 X 轴正方向向上（行号增加）
            // 我们希望 Y 轴正方向向左（列号增加意味着物理 Y 减小）
            // 所以 grid 的列索引对应 (height - 1 - y)
            int target_x_idx = x;
            int target_y_idx = (height - 1 - y);

            if (isColumnOccupied(x, y))
            {
                grid[target_x_idx][target_y_idx] = 1;
            }
        }
    }

    // 死区规划 margin：仅在 SNN 规划时启用，对 inactive_zone 边界额外膨胀，
    // 防止 SNN 路径贴死区边走、B-spline 拟合后触发碰撞重规划。
    if (deadzone_margin_cells > 0)
    {
        const int r        = deadzone_margin_cells;
        const int r_sq     = r * r;
        auto      inflated = grid;

        Eigen::Vector3d center_pos;
        Eigen::Vector3i grid_idx;
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                grid_idx << i, j, 0;
                gridIndexToPos(grid_idx, center_pos);
                if (!inInactiveZone(center_pos.x(), center_pos.y()))
                    continue;

                for (int di = -r; di <= r; ++di)
                {
                    for (int dj = -r; dj <= r; ++dj)
                    {
                        if (di * di + dj * dj > r_sq)
                            continue;
                        int ni = i + di;
                        int nj = j + dj;
                        if (ni >= 0 && ni < width && nj >= 0 && nj < height)
                            inflated[ni][nj] = 1;
                    }
                }
            }
        }
        grid = std::move(inflated);
    }

    return grid;
}

}  // namespace fast_planner
// SDFMap
