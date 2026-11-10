#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <array>

#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

namespace cv
{
class Mat;
}

class RayCaster;

namespace fast_planner
{
struct MapParam;
struct MapData;
class MapROS;
class MultiMapManager;

class SDFMap
{
public:
    SDFMap();
    ~SDFMap();

    enum OCCUPANCY
    {
        UNKNOWN,
        FREE,
        OCCUPIED
    };

    void initMap(rclcpp::Node::SharedPtr nh);
    void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num, const Eigen::Vector3d& camera_pos);

    // ROS adapter entry: input is already transformed using the real sensor TF.
    void integrateExternalCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const Eigen::Vector3d& sensor_pos);
    // Borrowed clouds remain alive for this synchronous call. Preserve all ray
    // origins while inflating/updating ESDF only once for the whole batch.
    void integrateExternalCloudBatch(const std::vector<std::pair<const pcl::PointCloud<pcl::PointXYZ>*, Eigen::Vector3d>>& observations);

    void   posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    void   indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    void   posToGridIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
    void   gridIndexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
    void   boundIndex(Eigen::Vector3i& id);
    int    toAddress(const Eigen::Vector3i& id);
    int    toAddress(const int& x, const int& y, const int& z);
    bool   isInMap(const Eigen::Vector3d& pos);
    bool   isInMap(const Eigen::Vector3i& idx);
    bool   isInBox(const Eigen::Vector3i& id);
    bool   isInBox(const Eigen::Vector3d& pos);
    void   boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up);
    int    getOccupancy(const Eigen::Vector3d& pos);
    int    getOccupancy(const Eigen::Vector3i& id);
    void   setOccupied(const Eigen::Vector3d& pos, const int& occ = 1);
    int    getInflateOccupancy(const Eigen::Vector3d& pos);
    int    getInflateOccupancy(const Eigen::Vector3i& id);
    double getDistance(const Eigen::Vector3d& pos);
    double getDistance(const Eigen::Vector3i& id);
    double getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
    void   updateESDF3d();
    void   resetBuffer();
    void   resetBuffer(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

    void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
    void getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax);
    void getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset = false);
    // Explored (coverage) box = bounding box of all observed voxels (all_min_/all_max_).
    // Used by the FSM to broadcast this drone's coverage on the fast drone_state channel (③).
    void   getExploredBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) const;
    double getResolution();
    int    getVoxelNum();
    bool   getBaseCoor(const int& id, Eigen::Vector4d& transform);

    std::vector<std::vector<int>> export2DGrid(const Eigen::Vector3d& pos, int target_z_layer = -1, int deadzone_margin_cells = 0);
    // UGV 2D：在导航高度带内对 (voxel_x, voxel_y) 柱投影，与 export2DGrid 一致
    bool isColumnOccupied(int voxel_x, int voxel_y);
    // 沿 XY 段检查柱投影；block_unknown 为 true 时未知体素视为障碍（路径简化用）
    bool isSegmentColumnFree(const Eigen::Vector3d& start, const Eigen::Vector3d& end, bool block_unknown = true);

    // pos 周围 radius_m 圆内占用体素直接置 FREE（动态车体清除）。flipped_free_adrs 非空时
    // 收集“曾标记动态命中（队友车体）的 occupied→free 翻转”体素地址，由调用方经
    // updateMapChunk 发布，撤销本车在地面站的 occupied 引用（静态障碍无标记，不收集）。
    void clearAgentFootprintAt(const Eigen::Vector3d& pos, double radius_m, double radius_scale = 1.0, vector<uint32_t>* flipped_free_adrs = nullptr);
    void clearDynamicAgentFootprints(const Eigen::Vector3d& ego_pos, double ego_radius, const std::vector<Eigen::Vector3d>& peer_positions, double peer_radius,
                                     double radius_scale = 1.0);
    void clearDynamicAgentFootprintsForPlanning(const Eigen::Vector3d& ego_pos, const std::vector<Eigen::Vector3d>& peer_positions, double radius_scale = 1.0);
    // 传感器链足迹清除：只清除“远处且移动中”的队友（近处保留靠激光 hit/miss 维护，
    // 停止车是真实静态障碍，保留给规划器避开）。peer_vels 为队友速度，用于动/停判定。
    void applyDynamicFootprintClearFromMapROS(const Eigen::Vector3d& ego_pos, const std::map<int, Eigen::Vector4d>& drone_states,
                                              const std::map<int, Eigen::Vector3d>& peer_vels, int self_drone_id, double radius_scale = 1.0);
    bool isInDynamicAgentFootprint(const Eigen::Vector3d& pos) const;
    // 同步队友最新位置：hit 更新据此把“队友车体命中”的体素
    // 标记为动态命中，后续 miss 一次即可直接置 FREE（车走开后扫到就消除）。
    void setPeerStates(const std::map<int, Eigen::Vector4d>& peer_states);
    // 动态足迹清除标志查询：被足迹清除操作置为 FREE 的体素返回 true。
    // frontier 检测据此从根源抑制清除区产生伪前沿（清除逻辑包含队友足迹，天然跨车兼容）。
    bool   isFootprintCleared(const Eigen::Vector3i& idx);
    bool   isFootprintCleared(const Eigen::Vector3d& pos);
    bool   agentFootprintClearEnabled() const;
    double stuckClearRadiusScale() const;
    double swarmClearStampTimeout() const;
    double swarmClearMinDist() const;

    // Swarm
    shared_ptr<MultiMapManager> mm_;
    friend MultiMapManager;

private:
    void                                                   clearAndInflateLocalMap();
    void                                                   inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
    void                                                   setCacheOccupancy(const int& adr, const int& occ);
    Eigen::Vector3d                                        closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
    template <typename F_get_val, typename F_set_val> void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

    unique_ptr<MapParam>  mp_;
    unique_ptr<MapData>   md_;
    unique_ptr<MapROS>    mr_;
    unique_ptr<RayCaster> caster_;

    struct DynamicFootprintZone
    {
        double cx{ 0.0 };
        double cy{ 0.0 };
        double radius{ 0.0 };
    };
    std::vector<DynamicFootprintZone> dynamic_footprint_zones_;

    // 队友最新位置缓存（MapROS::droneStateCallback 同步）：
    // 仅用于 hit 侧标记“车体命中”体素，供 miss 快速清除使用。
    std::map<int, Eigen::Vector4d> peer_states_;

    void getNavColumnZRange(int& z_start, int& z_end) const;

    // 死区（inactive_zone）几何：从 partitioning/ 参数读取，供 2D 栅格/碰撞检测统一使用
    void loadInactiveZones(rclcpp::Node::SharedPtr nh);
    bool inInactiveZone(double x, double y) const;
    // 死区实体化到 occupancy_buffer：
    //   - seedInactiveZonesIntoOccupancy：初始化时全图种子（把死区列写为 OCCUPIED）；
    //   - reassertInactiveZonesInLocalBox：每次传感器更新后，在局部更新盒内重新断言死区占据。
    // 之所以要每帧重断言：死区在物理上是空域，激光射线会把它重新标为 FREE，
    // 只写一次会被传感器冲掉。覆盖 occupancy_buffer_ 后，前沿检测/视点采样/EDT/A* 全部可见。
    void seedInactiveZonesIntoOccupancy();
    void reassertInactiveZonesInLocalBox();

    friend MapROS;

public:
    typedef std::shared_ptr<SDFMap> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MapParam
{
    // map properties
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;
    Eigen::Vector3i map_voxel_num_;
    double          resolution_, resolution_inv_;
    double          obstacles_inflation_;
    double          virtual_ceil_height_, ground_height_;
    Eigen::Vector3i box_min_, box_max_;
    Eigen::Vector3d box_mind_, box_maxd_;
    double          default_dist_;
    bool            optimistic_, signed_dist_;
    // map fusion
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
        min_occupancy_log_;  // logit
    double max_ray_length_;
    double local_bound_inflate_;
    int    local_map_margin_;
    double unknown_flag_;
    bool   no_drone_1_;

    // Dynamic agent footprint clearing (self + swarm peers → not static obstacles)
    bool   agent_footprint_clear_enable_{ true };
    double ego_clear_radius_{ 0.85 };
    // 车体 footprint 强制清除半径：该半径内的自身回波无条件置 FREE（覆盖 1.1x0.8 车体半对角 0.68m）；
    // ego_clear_radius 内、该半径外的静态障碍必须保留，避免规划地图被抹掉。
    double ego_body_clear_radius_{ 0.68 };
    double swarm_clear_min_dist_{ 0.8 };
    double swarm_clear_radius_{ 0.5 };
    bool   swarm_footprint_clear_enable_{ true };
    double swarm_clear_stamp_timeout_{ 0.6 };
    double stuck_clear_radius_scale_{ 1.5 };

    // Swarm tf
    bool            use_swarm_tf_;
    Eigen::Matrix3d rot_sw_;
    Eigen::Vector3d trans_sw_;

    // Vertices, box and normals of map in current drone's frame
    Eigen::Vector3d         vmin_, vmax_;
    vector<Eigen::Vector3d> vertices_;
    vector<Eigen::Vector3d> normals_;

    // 死区（inactive_zone）几何参数
    struct InactiveZoneRect
    {
        bool   enable{ false };
        double x_min{ 0.0 };
        double x_max{ 0.0 };
        double y_min{ 0.0 };
        double y_max{ 0.0 };
    };
    bool                            inactive_zone_enable_{ false };
    std::array<InactiveZoneRect, 2> inactive_zones_;
    // Append configurable heights without moving existing map parameter members.
    double min_obstacle_height_, max_obstacle_height_;
};

struct MapData
{
    // main map data, occupancy of each voxel and Euclidean distance
    std::vector<double> occupancy_buffer_;
    std::vector<char>   occupancy_buffer_inflate_;
    std::vector<double> distance_buffer_neg_;
    std::vector<double> distance_buffer_;
    std::vector<double> tmp_buffer1_;
    std::vector<double> tmp_buffer2_;
    // data for updating
    vector<short> count_hit_, count_miss_, count_hit_and_miss_;
    vector<char>  flag_rayend_, flag_visited_;
    // 动态足迹清除标志：与 occupancy_buffer_ 同尺寸，被 clearAgentFootprintAt 置 FREE 的体素永久标记为 1
    std::vector<char> footprint_cleared_flag_;
    // 动态命中标志：与 occupancy_buffer_ 同尺寸。hit 使体素从 free/unknown 翻转为 occupied
    // 且落点位于队友车体附近时标记为 1；之后 miss 一次即直接置 FREE 并清除标记。
    std::vector<char> dynamic_hit_flag_;
    char              raycast_num_;
    queue<int>        cache_voxel_;
    Eigen::Vector3i   local_bound_min_, local_bound_max_;
    Eigen::Vector3d   update_min_, update_max_;  // Box for recent update
    Eigen::Vector3d   all_min_, all_max_;        // Box for overall update
    bool              reset_updated_box_;

    // Swarm base coordinate transform
    map<int, Eigen::Vector4d> swarm_transform_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
{
    for (int i = 0; i < 3; ++i)
        id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
}

inline void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
{
    for (int i = 0; i < 3; ++i)
        pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
}

inline void SDFMap::boundIndex(Eigen::Vector3i& id)
{
    Eigen::Vector3i id1;
    id1(0) = max(min(id(0), mp_->map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), mp_->map_voxel_num_(1) - 1), 0);
    id1(2) = max(min(id(2), mp_->map_voxel_num_(2) - 1), 0);
    id     = id1;
}

inline void SDFMap::posToGridIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
{
    // 1. 先计算基础的体素索引
    int map_idx_x = floor((pos(0) - mp_->map_origin_(0)) * mp_->resolution_inv_);
    int map_idx_y = floor((pos(1) - mp_->map_origin_(1)) * mp_->resolution_inv_);
    int map_idx_z = floor((pos(2) - mp_->map_origin_(2)) * mp_->resolution_inv_);

    // 2. 映射到 export2DGrid 的二维坐标系
    // Grid X = Map X
    // Grid Y = Height - 1 - Map Y (对应你代码中的向左为Y)
    id(0) = map_idx_x;
    id(1) = mp_->map_voxel_num_(1) - 1 - map_idx_y;
    id(2) = map_idx_z;
}

inline void SDFMap::gridIndexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
{
    // 1. 逆向还原 Map Index
    int map_idx_x = id(0);
    int map_idx_y = mp_->map_voxel_num_(1) - 1 - id(1);

    // 2. 转换为物理坐标 (取体素中心点)
    pos(0) = (map_idx_x + 0.5) * mp_->resolution_ + mp_->map_origin_(0);
    pos(1) = (map_idx_y + 0.5) * mp_->resolution_ + mp_->map_origin_(1);

    // Z 轴通常根据 id(2) 或者你设定的导航高度
    // pos(2) = (id(2) + 0.5) * mp_->resolution_ + mp_->map_origin_(2);
    pos(2) = 0.3;  //	SNN 路径点的输出高度
}

inline int SDFMap::toAddress(const int& x, const int& y, const int& z)
{
    return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z;
}

inline int SDFMap::toAddress(const Eigen::Vector3i& id)
{
    return toAddress(id[0], id[1], id[2]);
}

inline bool SDFMap::isInMap(const Eigen::Vector3d& pos)
{
    if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4 || pos(2) < mp_->map_min_boundary_(2) + 1e-4)
        return false;
    if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 || pos(1) > mp_->map_max_boundary_(1) - 1e-4 || pos(2) > mp_->map_max_boundary_(2) - 1e-4)
        return false;
    return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i& idx)
{
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
        return false;
    if (idx(0) > mp_->map_voxel_num_(0) - 1 || idx(1) > mp_->map_voxel_num_(1) - 1 || idx(2) > mp_->map_voxel_num_(2) - 1)
        return false;
    return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3i& id)
{
    for (int i = 0; i < 3; ++i)
    {
        if (id[i] < mp_->box_min_[i] || id[i] >= mp_->box_max_[i])
        {
            return false;
        }
    }
    return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3d& pos)
{
    for (int i = 0; i < 3; ++i)
    {
        if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i])
        {
            return false;
        }
    }
    return true;
}

inline void SDFMap::boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up)
{
    for (int i = 0; i < 3; ++i)
    {
        low[i] = max(low[i], mp_->box_mind_[i]);
        up[i]  = min(up[i], mp_->box_maxd_[i]);
    }
}

inline int SDFMap::getOccupancy(const Eigen::Vector3i& id)
{
    if (!isInMap(id))
        return -1;
    double occ = md_->occupancy_buffer_[toAddress(id)];
    if (occ < mp_->clamp_min_log_ - 1e-3)
        return UNKNOWN;
    if (occ > mp_->min_occupancy_log_)
        return OCCUPIED;
    return FREE;
}

inline int SDFMap::getOccupancy(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getOccupancy(id);
}

inline void SDFMap::setOccupied(const Eigen::Vector3d& pos, const int& occ)
{
    if (!isInMap(pos))
        return;
    Eigen::Vector3i id;
    posToIndex(pos, id);
    md_->occupancy_buffer_inflate_[toAddress(id)] = occ;
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3i& id)
{
    if (!isInMap(id))
        return -1;
    return int(md_->occupancy_buffer_inflate_[toAddress(id)]);
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getInflateOccupancy(id);
}

inline bool SDFMap::isFootprintCleared(const Eigen::Vector3i& idx)
{
    if (!isInMap(idx))
        return false;
    return md_->footprint_cleared_flag_[toAddress(idx)] == 1;
}

inline bool SDFMap::isFootprintCleared(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return isFootprintCleared(id);
}

inline double SDFMap::getDistance(const Eigen::Vector3i& id)
{
    if (!isInMap(id))
        return -1;
    return md_->distance_buffer_[toAddress(id)];
}

inline double SDFMap::getDistance(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getDistance(id);
}

inline void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts)
{
    int num = 0;

    /* ---------- + shape inflate ---------- */
    // for (int x = -step; x <= step; ++x)
    // {
    //   if (x == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
    // }
    // for (int y = -step; y <= step; ++y)
    // {
    //   if (y == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
    // }
    // for (int z = -1; z <= 1; ++z)
    // {
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
    // }

    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y)
            for (int z = -step; z <= step; ++z)
            {
                pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
            }
}
}  // namespace fast_planner
#endif