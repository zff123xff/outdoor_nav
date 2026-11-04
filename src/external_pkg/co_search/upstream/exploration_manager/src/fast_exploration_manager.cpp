// #include <fstream>
#include <active_perception/frontier_finder.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exploration_manager/fast_exploration_manager.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <unordered_set>
// #include <active_perception/uniform_grid.h>

#include <path_searching/snn_path.h>

#include <active_perception/hgrid.h>
#include <path_searching/astar2.h>
#include <plan_env/edt_environment.h>
#include <plan_env/multi_map_manager.h>

#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
// #include <lkh_tsp_solver/lkh_interface.h>
#include <lkh_mtsp_solver/srv/solve_mtsp.hpp>

int solveMTSPWithLKH3(const char* input_file);

#include <lkh_tsp_solver/srv/solve_tsp.hpp>

#include <exploration_manager/expl_data.h>
#include <exploration_manager/stall_escape.h>
#include <ppu_opt/snn_path_interface.h>
#include <ppu_opt/snn_region_interface.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/msg/marker.hpp>

using namespace Eigen;

namespace fast_planner
{
namespace
{

    int runExternalLkhSolver(const std::string& lkh_binary, const std::string& par_path)
    {
        if (par_path.empty())
            return -1;
        const std::string binary = lkh_binary.empty() ? "LKH" : lkh_binary;
        const std::string cmd    = binary + " " + par_path;
        return std::system(cmd.c_str());
    }

    double GetMaxLocalGoalDist(const ExplorationParam& ep)
    {
        return ep.mtsp_fallback_max_goal_dist_ > 0.0 ? ep.mtsp_fallback_max_goal_dist_ : std::max(1.5, ep.local_traj_radius_);
    }

    int PickBestUtilityViewpoint(const ExplorationData& ed, const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw, double max_dist,
                                 double min_dist = 0.1, const std::function<bool(const Eigen::Vector3d&)>& is_reachable = {},
                                 const std::function<double(const Eigen::Vector3d&)>& cost_penalty = {})
    {
        double                       max_utility = -1.0;
        int                          best_id     = -1;
        std::vector<Eigen::Vector3d> tmp_path;
        const bool                   limit_dist = max_dist > 0.0 && std::isfinite(max_dist);
        for (int i = 0; i < static_cast<int>(ed.points_.size()); ++i)
        {
            Eigen::Vector3d point = ed.points_[i];
            point.z()             = 0.3;
            // P0-2：剔除死区对侧/不可达目标，避免选到 SNN 永远到不了的点
            if (is_reachable && !is_reachable(point))
                continue;
            const double dist = (point - pos).head<2>().norm();
            if (dist < min_dist)
                continue;
            if (limit_dist && dist > max_dist)
                continue;
            // ③ soft anti-clustering: peer-covered targets pay a small extra cost
            const double penalty     = cost_penalty ? cost_penalty(point) : 0.0;
            const double tmp_cost    = ViewNode::computeCost(pos, point, yaw[0], ed.yaws_[i], vel, yaw[1], tmp_path) + penalty;
            const double tmp_utility = 1.0 / (tmp_cost + 1e-3);
            if (tmp_utility > max_utility)
            {
                max_utility = tmp_utility;
                best_id     = i;
            }
        }
        return best_id;
    }

    double PerpDist2D(const Vector3d& p, const Vector3d& a, const Vector3d& b)
    {
        const Vector2d pa   = p.head<2>() - a.head<2>();
        const Vector2d ba   = b.head<2>() - a.head<2>();
        const double   len2 = ba.squaredNorm();
        if (len2 < 1e-12)
            return pa.norm();
        const double   t    = std::max(0.0, std::min(1.0, pa.dot(ba) / len2));
        const Vector2d proj = a.head<2>() + t * ba;
        return (p.head<2>() - proj).norm();
    }

    // SNN may repeat the same grid cell; consecutive duplicates → near-zero segments in traj optimizer.
    constexpr double kSnnPathDedupeConsecutiveEpsM = 1e-3;
    // B-spline 输入路点过多时轨迹时长/折线长度被栅格锯齿放大；实际上限由 snn_traj_max_waypoints_ 覆盖
    constexpr size_t kDefaultSnnTrajMaxWaypoints = 12;
    constexpr size_t kMaxSnnFullPathWaypoints    = 32;
    // 停止状态队友位置的新鲜度上限：超时后位置不可信，不参与静止避让集合（s）。
    constexpr double kStaticPeerStateStaleS = 2.0;
    // 动/停判定速度阈值：低于此值视为停止车（保留占用/合成静止轨迹），与 sdf_map 的
    // kDynamicClearSpeedEps 保持一致（m/s）。
    constexpr double kDynamicClearSpeedEps = 0.15;

    // 动态从 SDF 生成 SNN 2D grid：柱投影 + 车体高度带，与 export2DGrid 一致（避免单层 Z 漏检墙）
    void ClearDynamicAgentFootprintsForPlanning(FastExplorationManager* mgr, const Eigen::Vector3d& robot_pos, const double radius_scale)
    {
        if (!mgr || !mgr->sdf_map_ || !mgr->sdf_map_->agentFootprintClearEnabled())
            return;

        std::vector<Eigen::Vector3d> peer_positions;
        const double                 now_sec       = mgr->node_->now().seconds();
        const double                 stamp_timeout = mgr->sdf_map_->swarmClearStampTimeout();
        for (int i = 0; i < static_cast<int>(mgr->ed_->swarm_state_.size()); ++i)
        {
            const int peer_id = i + 1;
            if (peer_id == mgr->ep_->drone_id_)
                continue;
            const auto& peer = mgr->ed_->swarm_state_[i];
            if (now_sec - peer.stamp_ > stamp_timeout)
                continue;
            // 只清除“远处且移动中”的队友足迹（与传感器链 applyDynamicFootprintClearFromMapROS
            // 门控一致）：停止车是真实静态障碍保留占用，近处车靠激光 hit/miss 维护。
            if (peer.vel_.head<2>().norm() <= kDynamicClearSpeedEps)
                continue;
            if ((peer.pos_ - robot_pos).head<2>().norm() <= mgr->sdf_map_->swarmClearMinDist())
                continue;
            peer_positions.push_back(peer.pos_);
        }

        mgr->sdf_map_->clearDynamicAgentFootprintsForPlanning(robot_pos, peer_positions, radius_scale);
    }

    bool UpdateSnnGridFromSdfImpl(FastExplorationManager* mgr, const Eigen::Vector3d& robot_pos)
    {
        if (!mgr || !mgr->sdf_map_ || !mgr->snn_)
            return false;

        ClearDynamicAgentFootprintsForPlanning(mgr, robot_pos, 1.0);

        const auto grid = mgr->sdf_map_->export2DGrid(robot_pos, -1, mgr->getSnnDeadzoneMarginCells());
        if (grid.empty())
            return false;

        mgr->snn_->setGrid(grid);
        return true;
    }

    void DedupeConsecutivePath(std::vector<Vector3d>& path, double eps_m)
    {
        if (path.size() < 2)
            return;
        std::vector<Vector3d> out;
        out.reserve(path.size());
        for (const auto& p : path)
        {
            if (out.empty() || (p - out.back()).norm() > eps_m)
                out.push_back(p);
        }
        path.swap(out);
    }

    // B-spline / minimum-jerk 轨迹至少需要 3 个路点；与 shortenPath 末尾逻辑一致。
    void EnsureMinThreePathPoints(std::vector<Vector3d>& path)
    {
        if (path.size() != 2)
            return;
        const Vector3d mid = 0.5 * (path[0] + path[1]);
        path.insert(path.begin() + 1, mid);
    }

    // SNN 路径调试：避免单次日志过长截断 ROS 输出。
    constexpr size_t kMaxSnnPathLogCells = 256;

    void LogSnnGridPath(int drone_id, const std::vector<Eigen::Vector3i>& path_grid, const SDFMap::Ptr& sdf_map)
    {
        if (path_grid.empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[SNN Planner][d%d] path cells=0 (empty)", drone_id);
            return;
        }

        std::ostringstream idx_ss;
        std::ostringstream pos_ss;
        idx_ss << "[";
        pos_ss << "[";
        const size_t n_log = std::min(path_grid.size(), kMaxSnnPathLogCells);
        for (size_t i = 0; i < n_log; ++i)
        {
            if (i > 0)
            {
                idx_ss << ", ";
                pos_ss << ", ";
            }
            const auto&     idx = path_grid[i];
            Eigen::Vector3d wp;
            sdf_map->gridIndexToPos(idx, wp);
            idx_ss << "(" << idx.x() << "," << idx.y() << "," << idx.z() << ")";
            pos_ss << "(" << wp.x() << "," << wp.y() << "," << wp.z() << ")";
        }
        if (path_grid.size() > kMaxSnnPathLogCells)
        {
            idx_ss << ", ...(" << (path_grid.size() - kMaxSnnPathLogCells) << " more)";
            pos_ss << ", ...(" << (path_grid.size() - kMaxSnnPathLogCells) << " more)";
        }
        idx_ss << "]";
        pos_ss << "]";

        // RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[SNN Planner][d%d] path cells=%zu grid_idx=%s", drone_id, path_grid.size(),
        //             idx_ss.str().c_str());
        // RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[SNN Planner][d%d] path cells=%zu world_pos=%s", drone_id, path_grid.size(),
        //             pos_ss.str().c_str());
    }

    std::vector<int> RdpPathIndicesImpl(const std::vector<Vector3d>& pts, int i0, int i1, double eps,
                                        const std::function<bool(const Vector3d&, const Vector3d&)>& segFree)
    {
        if (i1 <= i0 + 1)
            return { i0, i1 };

        double max_d = 0.0;
        int    k     = -1;
        for (int i = i0 + 1; i < i1; ++i)
        {
            const double d = PerpDist2D(pts[i], pts[i0], pts[i1]);
            if (d > max_d)
            {
                max_d = d;
                k     = i;
            }
        }

        const bool free = segFree(pts[i0], pts[i1]);
        if (free && max_d <= eps)
            return { i0, i1 };

        if (k < 0)
            k = (i0 + i1) / 2;

        std::vector<int> L = RdpPathIndicesImpl(pts, i0, k, eps, segFree);
        std::vector<int> R = RdpPathIndicesImpl(pts, k, i1, eps, segFree);
        L.insert(L.end(), R.begin() + 1, R.end());
        return L;
    }

    double PathTurnAngleRad(const Vector3d& a, const Vector3d& b, const Vector3d& c)
    {
        const Vector3d v1 = b - a;
        const Vector3d v2 = c - b;
        const double   l1 = v1.head<2>().norm();
        const double   l2 = v2.head<2>().norm();
        if (l1 < 1e-6 || l2 < 1e-6)
            return 0.0;
        double cos_a = v1.dot(v2) / (l1 * l2);
        cos_a        = std::max(-1.0, std::min(1.0, cos_a));
        return std::acos(cos_a);
    }

    std::vector<size_t> FindCornerSplitIndices(const std::vector<Vector3d>& path, double min_angle_rad)
    {
        std::vector<size_t> splits;
        splits.push_back(0);
        if (path.size() < 3)
        {
            splits.push_back(path.size() - 1);
            return splits;
        }
        for (size_t i = 1; i + 1 < path.size(); ++i)
        {
            if (PathTurnAngleRad(path[i - 1], path[i], path[i + 1]) >= min_angle_rad)
                splits.push_back(i);
        }
        splits.push_back(path.size() - 1);
        return splits;
    }

    void DecimatePathKeepCorners(const std::vector<Vector3d>& path, size_t max_pts, std::vector<Vector3d>& out)
    {
        if (path.size() <= max_pts)
        {
            out = path;
            return;
        }

        constexpr double    kCornerRad = 20.0 * M_PI / 180.0;
        std::vector<size_t> keep       = { 0, path.size() - 1 };

        std::vector<std::pair<double, size_t>> ranked;
        ranked.reserve(path.size());
        for (size_t i = 1; i + 1 < path.size(); ++i)
        {
            const double ang = PathTurnAngleRad(path[i - 1], path[i], path[i + 1]);
            if (ang >= kCornerRad)
                ranked.emplace_back(ang, i);
        }
        std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.first > b.first; });

        for (const auto& [ang, idx] : ranked)
        {
            (void)ang;
            if (keep.size() >= max_pts)
                break;
            keep.push_back(idx);
        }
        std::sort(keep.begin(), keep.end());
        keep.erase(std::unique(keep.begin(), keep.end()), keep.end());

        while (keep.size() < max_pts)
        {
            size_t best_idx   = keep.front();
            size_t best_count = 0;
            for (size_t k = 0; k + 1 < keep.size(); ++k)
            {
                const size_t i0 = keep[k];
                const size_t i1 = keep[k + 1];
                if (i1 <= i0 + 1)
                    continue;
                const size_t span = i1 - i0 - 1;
                if (span > best_count)
                {
                    best_count = span;
                    best_idx   = (i0 + i1) / 2;
                }
            }
            if (best_count == 0)
                break;
            keep.push_back(best_idx);
            std::sort(keep.begin(), keep.end());
            keep.erase(std::unique(keep.begin(), keep.end()), keep.end());
        }

        out.clear();
        out.reserve(keep.size());
        for (const size_t idx : keep)
            out.push_back(path[idx]);
        DedupeConsecutivePath(out, 0.04);
        EnsureMinThreePathPoints(out);
    }

    Vector3d PointAlongPolyline(const std::vector<Vector3d>& path, size_t i0, size_t i1, double dist_from_i0)
    {
        double acc = 0.0;
        for (size_t i = i0; i < i1; ++i)
        {
            const Vector3d seg = path[i + 1] - path[i];
            const double   len = seg.head<2>().norm();
            if (len < 1e-6)
                continue;
            if (acc + len >= dist_from_i0 - 1e-9)
            {
                const double t = std::max(0.0, std::min(1.0, (dist_from_i0 - acc) / len));
                Vector3d     p = path[i] + t * seg;
                return p;
            }
            acc += len;
        }
        return path[i1];
    }

    double PolylineLength2D(const std::vector<Vector3d>& path, size_t i0, size_t i1)
    {
        double len = 0.0;
        for (size_t i = i0; i < i1; ++i)
            len += (path[i + 1] - path[i]).head<2>().norm();
        return len;
    }

    // 弧长等距重采样并强制保留拐角，比 RDP 更适合 L 形栅格路径 → B-spline（避免 3 点切角外鼓）。
    void ResamplePathArcLengthPreserveCorners(std::vector<Vector3d>& path, double spacing, double corner_angle_rad)
    {
        if (path.size() < 2 || spacing <= 1e-6)
            return;

        const std::vector<size_t> splits = FindCornerSplitIndices(path, corner_angle_rad);
        if (splits.size() < 2)
            return;

        std::vector<Vector3d> out;
        out.reserve(path.size());
        for (size_t s = 0; s + 1 < splits.size(); ++s)
        {
            const size_t i0 = splits[s];
            const size_t i1 = splits[s + 1];
            if (i1 <= i0)
                continue;

            if (out.empty())
                out.push_back(path[i0]);

            const double seg_len = PolylineLength2D(path, i0, i1);
            for (double d = spacing; d + 1e-6 < seg_len; d += spacing)
            {
                Vector3d p = PointAlongPolyline(path, i0, i1, d);
                if ((p - out.back()).head<2>().norm() > 0.04)
                    out.push_back(p);
            }

            const Vector3d end = path[i1];
            if ((end - out.back()).head<2>().norm() > 0.04)
                out.push_back(end);
        }

        DedupeConsecutivePath(out, 0.04);
        EnsureMinThreePathPoints(out);
        if (out.size() >= 2)
            path.swap(out);
    }

    void CapPathKeepCorners(std::vector<Vector3d>& path, size_t max_pts)
    {
        if (path.size() <= max_pts || max_pts < 2)
            return;
        std::vector<Vector3d> capped;
        DecimatePathKeepCorners(path, max_pts, capped);
        path.swap(capped);
    }

    // allocateGrids：未完成区域掩码在「动态重划」与「代价矩阵软惩罚」之间共享，单独置于匿名命名空间。
    struct PairAllocateGridsSharedState
    {
        bool               unfinished_inited{ false };
        unsigned long long unfinished_obs_mask{ 0 };
        unsigned long long unfinished_stable_mask{ 0 };
        unsigned long long unfinished_applied_mask{ 0 };
        int                unfinished_same_count{ 0 };
    };
    inline PairAllocateGridsSharedState& PairAllocateGridsShared()
    {
        static PairAllocateGridsSharedState s;
        return s;
    }

}  // namespace

// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {}

FastExplorationManager::~FastExplorationManager()
{
    ViewNode::astar_.reset();
    ViewNode::caster_.reset();
    ViewNode::map_.reset();
}

void FastExplorationManager::initialize(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    planner_manager_.reset(new FastPlannerManager);
    planner_manager_->initPlanModules(node);

    edt_environment_ = planner_manager_->edt_environment_;
    sdf_map_         = edt_environment_->sdf_map_;
    frontier_finder_.reset(new FrontierFinder(edt_environment_, node));
    // uniform_grid_.reset(new UniformGrid(edt_environment_, node));
    hgrid_.reset(new HGrid(edt_environment_, node));
    // view_finder_.reset(new ViewFinder(edt_environment_, nh));

    ed_.reset(new ExplorationData);
    ep_.reset(new ExplorationParam);
    region_allocator_ = createRegionAllocator(node);

    const double snn_wave_robot_radius_param     = node->has_parameter("snn/wave_robot_radius") ? node->get_parameter("snn/wave_robot_radius").as_double()
                                                                                                : node->declare_parameter<double>("snn/wave_robot_radius", 2.0);
    const int    snn_obstacle_close_kernel       = node->has_parameter("snn/obstacle_close_kernel") ? node->get_parameter("snn/obstacle_close_kernel").as_int()
                                                                                                    : node->declare_parameter<int>("snn/obstacle_close_kernel", 3);
    const double snn_dead_zone_planning_margin_m = node->has_parameter("snn/dead_zone_planning_margin")
                                                       ? node->get_parameter("snn/dead_zone_planning_margin").as_double()
                                                       : node->declare_parameter<double>("snn/dead_zone_planning_margin", 0.3);
    snn_deadzone_margin_cells_                   = static_cast<int>(std::ceil(snn_dead_zone_planning_margin_m / std::max(1e-6, sdf_map_->getResolution())));

    auto cfg                  = PlannerConfig();
    cfg.max_steps             = 5000000;
    cfg.robot_radius          = static_cast<float>(snn_wave_robot_radius_param);
    cfg.obstacle_close_kernel = snn_obstacle_close_kernel;
    snn_                      = createPathPlanner(node, cfg);
    // 占位栅格；首次 planTrajToView(SNN) 前由 export2DGrid 从实时 SDF 覆盖
    snn_->setGrid({ { 0 } });
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"),
                "[SNN] live 2D grid from SDF (export2DGrid); wave_robot_radius=%.2f cells obstacle_close_kernel=%d dead_zone_margin=%d cells",
                snn_wave_robot_radius_param, snn_obstacle_close_kernel, snn_deadzone_margin_cells_);

    ep_->refine_local_                 = node->has_parameter("exploration/refine_local") ? node->get_parameter("exploration/refine_local").as_bool()
                                                                                         : node->declare_parameter<bool>("exploration/refine_local", true);
    ep_->refined_num_                  = node->has_parameter("exploration/refined_num") ? node->get_parameter("exploration/refined_num").as_int()
                                                                                        : node->declare_parameter<int>("exploration/refined_num", -1);
    ep_->refined_radius_               = node->has_parameter("exploration/refined_radius") ? node->get_parameter("exploration/refined_radius").as_double()
                                                                                           : node->declare_parameter<double>("exploration/refined_radius", -1.0);
    ep_->local_traj_radius_            = node->has_parameter("exploration/local_traj_radius") ? node->get_parameter("exploration/local_traj_radius").as_double()
                                                                                              : node->declare_parameter<double>("exploration/local_traj_radius", 5.0);
    ep_->snn_path_rdp_epsilon_         = node->has_parameter("exploration/snn_path_rdp_epsilon")
                                             ? node->get_parameter("exploration/snn_path_rdp_epsilon").as_double()
                                             : node->declare_parameter<double>("exploration/snn_path_rdp_epsilon", 0.35);
    ep_->snn_path_rdp_max_epsilon_     = node->has_parameter("exploration/snn_path_rdp_max_epsilon")
                                             ? node->get_parameter("exploration/snn_path_rdp_max_epsilon").as_double()
                                             : node->declare_parameter<double>("exploration/snn_path_rdp_max_epsilon", 1.0);
    ep_->snn_path_resample_spacing_    = node->has_parameter("exploration/snn_path_resample_spacing")
                                             ? node->get_parameter("exploration/snn_path_resample_spacing").as_double()
                                             : node->declare_parameter<double>("exploration/snn_path_resample_spacing", 0.5);
    ep_->snn_path_shorten_max_segment_ = node->has_parameter("exploration/snn_path_shorten_max_segment")
                                             ? node->get_parameter("exploration/snn_path_shorten_max_segment").as_double()
                                             : node->declare_parameter<double>("exploration/snn_path_shorten_max_segment", 0.0);
    ep_->snn_path_shorten_angle_deg_   = node->has_parameter("exploration/snn_path_shorten_angle_deg")
                                             ? node->get_parameter("exploration/snn_path_shorten_angle_deg").as_double()
                                             : node->declare_parameter<double>("exploration/snn_path_shorten_angle_deg", 15.0);
    ep_->snn_traj_max_waypoints_       = node->has_parameter("exploration/snn_traj_max_waypoints")
                                             ? node->get_parameter("exploration/snn_traj_max_waypoints").as_int()
                                             : node->declare_parameter<int>("exploration/snn_traj_max_waypoints", 12);
    ep_->snn_fail_astar_trigger_count_ = node->has_parameter("exploration/snn_fail_astar_trigger_count")
                                             ? node->get_parameter("exploration/snn_fail_astar_trigger_count").as_int()
                                             : node->declare_parameter<int>("exploration/snn_fail_astar_trigger_count", 2);
    ep_->viewpoint_arrived_dist_m_     = node->has_parameter("exploration/viewpoint_arrived_dist_m")
                                             ? node->get_parameter("exploration/viewpoint_arrived_dist_m").as_double()
                                             : node->declare_parameter<double>("exploration/viewpoint_arrived_dist_m", 0.55);
    ep_->degenerate_goal_dist_m_       = node->has_parameter("exploration/degenerate_goal_dist_m")
                                             ? node->get_parameter("exploration/degenerate_goal_dist_m").as_double()
                                             : node->declare_parameter<double>("exploration/degenerate_goal_dist_m", 0.05);
    ep_->liveness_enable_              = node->has_parameter("exploration/liveness_enable") ? node->get_parameter("exploration/liveness_enable").as_bool()
                                                                                            : node->declare_parameter<bool>("exploration/liveness_enable", true);
    ep_->stall_escape_after_waits_     = std::max(1, static_cast<int>(node->has_parameter("exploration/stall_escape_after_waits")
                                                                          ? node->get_parameter("exploration/stall_escape_after_waits").as_int()
                                                                          : node->declare_parameter<int>("exploration/stall_escape_after_waits", 6)));
    ep_->stall_escape_interval_sec_    = std::max(0.0, node->has_parameter("exploration/stall_escape_interval_sec")
                                                           ? node->get_parameter("exploration/stall_escape_interval_sec").as_double()
                                                           : node->declare_parameter<double>("exploration/stall_escape_interval_sec", 2.0));
    ep_->stall_escape_min_dist_m_ =
        std::max(0.0, node->has_parameter("exploration/stall_escape_min_dist_m") ? node->get_parameter("exploration/stall_escape_min_dist_m").as_double()
                                                                                 : node->declare_parameter<double>("exploration/stall_escape_min_dist_m", 1.0));
    ep_->stall_escape_max_dist_m_ =
        std::max(ep_->stall_escape_min_dist_m_ + 1.0, node->has_parameter("exploration/stall_escape_max_dist_m")
                                                          ? node->get_parameter("exploration/stall_escape_max_dist_m").as_double()
                                                          : node->declare_parameter<double>("exploration/stall_escape_max_dist_m", 15.0));
    ep_->stall_escape_max_attempts_ = std::max(1, static_cast<int>(node->has_parameter("exploration/stall_escape_max_attempts")
                                                                       ? node->get_parameter("exploration/stall_escape_max_attempts").as_int()
                                                                       : node->declare_parameter<int>("exploration/stall_escape_max_attempts", 3)));
    ep_->dead_work_stall_sec_ =
        std::max(1.0, node->has_parameter("exploration/dead_work_stall_sec") ? node->get_parameter("exploration/dead_work_stall_sec").as_double()
                                                                             : node->declare_parameter<double>("exploration/dead_work_stall_sec", 10.0));
    ep_->no_progress_stall_sec_ =
        std::max(1.0, node->has_parameter("exploration/no_progress_stall_sec") ? node->get_parameter("exploration/no_progress_stall_sec").as_double()
                                                                               : node->declare_parameter<double>("exploration/no_progress_stall_sec", 20.0));
    ep_->stall_return_stagger_sec_         = std::max(0.0, node->has_parameter("exploration/stall_return_stagger_sec")
                                                               ? node->get_parameter("exploration/stall_return_stagger_sec").as_double()
                                                               : node->declare_parameter<double>("exploration/stall_return_stagger_sec", 5.0));
    ep_->target_failure_quarantine_enable_ = node->has_parameter("exploration/target_failure_quarantine_enable")
                                                 ? node->get_parameter("exploration/target_failure_quarantine_enable").as_bool()
                                                 : node->declare_parameter<bool>("exploration/target_failure_quarantine_enable", true);
    ep_->target_failure_quarantine_cell_m_ = std::max(0.05, node->has_parameter("exploration/target_failure_quarantine_cell_m")
                                                                ? node->get_parameter("exploration/target_failure_quarantine_cell_m").as_double()
                                                                : node->declare_parameter<double>("exploration/target_failure_quarantine_cell_m", 0.5));
    ep_->target_failure_quarantine_initial_sec_ =
        std::max(0.1, node->has_parameter("exploration/target_failure_quarantine_initial_sec")
                          ? node->get_parameter("exploration/target_failure_quarantine_initial_sec").as_double()
                          : node->declare_parameter<double>("exploration/target_failure_quarantine_initial_sec", 1.0));
    ep_->target_failure_quarantine_max_sec_ =
        std::max(ep_->target_failure_quarantine_initial_sec_, node->has_parameter("exploration/target_failure_quarantine_max_sec")
                                                                  ? node->get_parameter("exploration/target_failure_quarantine_max_sec").as_double()
                                                                  : node->declare_parameter<double>("exploration/target_failure_quarantine_max_sec", 10.0));
    ep_->target_failure_quarantine_max_entries_ =
        std::max(1, static_cast<int>(node->has_parameter("exploration/target_failure_quarantine_max_entries")
                                         ? node->get_parameter("exploration/target_failure_quarantine_max_entries").as_int()
                                         : node->declare_parameter<int>("exploration/target_failure_quarantine_max_entries", 128)));
    // ③ peer coverage-box anti-clustering: a soft cost (distance-equivalent, m)
    //    added to targets inside a peer's fresh explored box. Breaks the tie when
    //    two cars would otherwise chase the same stale frontier, but never vetoes
    //    the only remaining option (no coverage regression).
    ep_->peer_cov_penalty_     = node->has_parameter("exploration/peer_cov_penalty") ? node->get_parameter("exploration/peer_cov_penalty").as_double()
                                                                                     : node->declare_parameter<double>("exploration/peer_cov_penalty", 20.0);
    ep_->peer_cov_margin_m_    = node->has_parameter("exploration/peer_cov_margin_m") ? node->get_parameter("exploration/peer_cov_margin_m").as_double()
                                                                                      : node->declare_parameter<double>("exploration/peer_cov_margin_m", 2.0);
    ep_->anti_tailgate_enable_ = node->has_parameter("exploration/anti_tailgate_enable")
                                     ? node->get_parameter("exploration/anti_tailgate_enable").as_bool()
                                     : node->declare_parameter<bool>("exploration/anti_tailgate_enable", true);
    if (node->has_parameter("exploration/anti_tailgate_heading_deg"))
    {
        const auto heading_param = node->get_parameter("exploration/anti_tailgate_heading_deg");
        if (heading_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            ep_->anti_tailgate_heading_deg_ = static_cast<double>(heading_param.as_int());
        }
        else
        {
            ep_->anti_tailgate_heading_deg_ = heading_param.as_double();
        }
    }
    else
    {
        ep_->anti_tailgate_heading_deg_ = static_cast<double>(node->declare_parameter<int>("exploration/anti_tailgate_heading_deg", 30));
    }
    ep_->anti_tailgate_sep_weight_       = node->has_parameter("exploration/anti_tailgate_sep_weight")
                                               ? node->get_parameter("exploration/anti_tailgate_sep_weight").as_double()
                                               : node->declare_parameter<double>("exploration/anti_tailgate_sep_weight", 3.0);
    ep_->anti_tailgate_self_cost_weight_ = node->has_parameter("exploration/anti_tailgate_self_cost_weight")
                                               ? node->get_parameter("exploration/anti_tailgate_self_cost_weight").as_double()
                                               : node->declare_parameter<double>("exploration/anti_tailgate_self_cost_weight", 0.8);
    ep_->anti_tailgate_ahead_enter_m_    = node->has_parameter("exploration/anti_tailgate_ahead_enter_m")
                                               ? node->get_parameter("exploration/anti_tailgate_ahead_enter_m").as_double()
                                               : node->declare_parameter<double>("exploration/anti_tailgate_ahead_enter_m", 0.35);
    ep_->anti_tailgate_ahead_exit_m_     = node->has_parameter("exploration/anti_tailgate_ahead_exit_m")
                                               ? node->get_parameter("exploration/anti_tailgate_ahead_exit_m").as_double()
                                               : node->declare_parameter<double>("exploration/anti_tailgate_ahead_exit_m", -0.45);
    ep_->near_split_enable_              = node->has_parameter("exploration/near_split_enable") ? node->get_parameter("exploration/near_split_enable").as_bool()
                                                                                                : node->declare_parameter<bool>("exploration/near_split_enable", true);
    ep_->near_split_dist_                = node->has_parameter("exploration/near_split_dist") ? node->get_parameter("exploration/near_split_dist").as_double()
                                                                                              : node->declare_parameter<double>("exploration/near_split_dist", 10.0);
    ep_->near_split_stamp_timeout_       = node->has_parameter("exploration/near_split_stamp_timeout")
                                               ? node->get_parameter("exploration/near_split_stamp_timeout").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_stamp_timeout", 1.5);
    ep_->near_split_min_target_dist_     = node->has_parameter("exploration/near_split_min_target_dist")
                                               ? node->get_parameter("exploration/near_split_min_target_dist").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_min_target_dist", 4.0);
    ep_->near_split_sep_weight_          = node->has_parameter("exploration/near_split_sep_weight")
                                               ? node->get_parameter("exploration/near_split_sep_weight").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_sep_weight", 2.5);
    ep_->near_split_self_cost_weight_    = node->has_parameter("exploration/near_split_self_cost_weight")
                                               ? node->get_parameter("exploration/near_split_self_cost_weight").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_self_cost_weight", 0.7);
    ep_->near_split_uncovered_bonus_     = node->has_parameter("exploration/near_split_uncovered_bonus")
                                               ? node->get_parameter("exploration/near_split_uncovered_bonus").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_uncovered_bonus", 6.0);
    ep_->near_split_same_region_penalty_ = node->has_parameter("exploration/near_split_same_region_penalty")
                                               ? node->get_parameter("exploration/near_split_same_region_penalty").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_same_region_penalty", 4.0);
    ep_->near_split_cooldown_            = node->has_parameter("exploration/near_split_cooldown")
                                               ? node->get_parameter("exploration/near_split_cooldown").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_cooldown", 2.5);
    ep_->near_split_lock_sec_            = node->has_parameter("exploration/near_split_lock_sec")
                                               ? node->get_parameter("exploration/near_split_lock_sec").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_lock_sec", 3.0);
    ep_->near_split_lock_extend_dist_    = node->has_parameter("exploration/near_split_lock_extend_dist")
                                               ? node->get_parameter("exploration/near_split_lock_extend_dist").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_lock_extend_dist", 6.0);
    ep_->near_split_lock_extend_sec_     = node->has_parameter("exploration/near_split_lock_extend_sec")
                                               ? node->get_parameter("exploration/near_split_lock_extend_sec").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_lock_extend_sec", 1.0);
    ep_->near_split_lock_max_total_sec_  = node->has_parameter("exploration/near_split_lock_max_total_sec")
                                               ? node->get_parameter("exploration/near_split_lock_max_total_sec").as_double()
                                               : node->declare_parameter<double>("exploration/near_split_lock_max_total_sec", 6.0);
    ep_->near_split_region_hysteresis_sec_        = node->has_parameter("exploration/near_split_region_hysteresis_sec")
                                                        ? node->get_parameter("exploration/near_split_region_hysteresis_sec").as_double()
                                                        : node->declare_parameter<double>("exploration/near_split_region_hysteresis_sec", 5.0);
    ep_->near_split_region_switch_penalty_        = node->has_parameter("exploration/near_split_region_switch_penalty")
                                                        ? node->get_parameter("exploration/near_split_region_switch_penalty").as_double()
                                                        : node->declare_parameter<double>("exploration/near_split_region_switch_penalty", 3.5);
    ep_->near_split_uncovered_bonus_decay_dist_   = node->has_parameter("exploration/near_split_uncovered_bonus_decay_dist")
                                                        ? node->get_parameter("exploration/near_split_uncovered_bonus_decay_dist").as_double()
                                                        : node->declare_parameter<double>("exploration/near_split_uncovered_bonus_decay_dist", 7.0);
    ep_->near_split_uncovered_bonus_min_scale_    = node->has_parameter("exploration/near_split_uncovered_bonus_min_scale")
                                                        ? node->get_parameter("exploration/near_split_uncovered_bonus_min_scale").as_double()
                                                        : node->declare_parameter<double>("exploration/near_split_uncovered_bonus_min_scale", 0.2);
    ep_->near_split_uncovered_bonus_density_gain_ = node->has_parameter("exploration/near_split_uncovered_bonus_density_gain")
                                                        ? node->get_parameter("exploration/near_split_uncovered_bonus_density_gain").as_double()
                                                        : node->declare_parameter<double>("exploration/near_split_uncovered_bonus_density_gain", 0.7);
    ep_->near_split_uncovered_bonus_occ_penalty_  = node->has_parameter("exploration/near_split_uncovered_bonus_occ_penalty")
                                                        ? node->get_parameter("exploration/near_split_uncovered_bonus_occ_penalty").as_double()
                                                        : node->declare_parameter<double>("exploration/near_split_uncovered_bonus_occ_penalty", 0.45);
    ep_->near_split_min_peer_target_dist_         = node->has_parameter("exploration/near_split_min_peer_target_dist")
                                                        ? node->get_parameter("exploration/near_split_min_peer_target_dist").as_double()
                                                        : node->declare_parameter<double>("exploration/near_split_min_peer_target_dist", 5.0);
    ep_->near_split_switch_min_improve_ratio_     = node->has_parameter("exploration/near_split_switch_min_improve_ratio")
                                                        ? node->get_parameter("exploration/near_split_switch_min_improve_ratio").as_double()
                                                        : node->declare_parameter<double>("exploration/near_split_switch_min_improve_ratio", 0.15);
    ep_->near_split_switch_min_interval_sec_      = node->has_parameter("exploration/near_split_switch_min_interval_sec")
                                                        ? node->get_parameter("exploration/near_split_switch_min_interval_sec").as_double()
                                                        : node->declare_parameter<double>("exploration/near_split_switch_min_interval_sec", 2.0);
    ep_->pre_split_lookahead_sec_                 = node->has_parameter("exploration/pre_split_lookahead_sec")
                                                        ? node->get_parameter("exploration/pre_split_lookahead_sec").as_double()
                                                        : node->declare_parameter<double>("exploration/pre_split_lookahead_sec", 3.0);
    ep_->pre_split_dist_                     = node->has_parameter("exploration/pre_split_dist") ? node->get_parameter("exploration/pre_split_dist").as_double()
                                                                                                 : node->declare_parameter<double>("exploration/pre_split_dist", 12.0);
    ep_->near_split_incoming_region_penalty_ = node->has_parameter("exploration/near_split_incoming_region_penalty")
                                                   ? node->get_parameter("exploration/near_split_incoming_region_penalty").as_double()
                                                   : node->declare_parameter<double>("exploration/near_split_incoming_region_penalty", 8.0);
    ep_->near_split_log_uncovered_bonus_     = node->has_parameter("exploration/near_split_log_uncovered_bonus")
                                                   ? node->get_parameter("exploration/near_split_log_uncovered_bonus").as_bool()
                                                   : node->declare_parameter<bool>("exploration/near_split_log_uncovered_bonus", false);
    ep_->plan_fail_fallback_trigger_count_   = node->has_parameter("exploration/plan_fail_fallback_trigger_count")
                                                   ? node->get_parameter("exploration/plan_fail_fallback_trigger_count").as_int()
                                                   : node->declare_parameter<int>("exploration/plan_fail_fallback_trigger_count", 3);
    ep_->plan_fail_fallback_min_dist_        = node->has_parameter("exploration/plan_fail_fallback_min_dist")
                                                   ? node->get_parameter("exploration/plan_fail_fallback_min_dist").as_double()
                                                   : node->declare_parameter<double>("exploration/plan_fail_fallback_min_dist", 0.8);
    ep_->plan_fail_circuit_threshold_        = node->has_parameter("exploration/plan_fail_circuit_threshold")
                                                   ? node->get_parameter("exploration/plan_fail_circuit_threshold").as_int()
                                                   : node->declare_parameter<int>("exploration/plan_fail_circuit_threshold", 30);
    ep_->plan_fail_circuit_sec_              = node->has_parameter("exploration/plan_fail_circuit_sec")
                                                   ? node->get_parameter("exploration/plan_fail_circuit_sec").as_double()
                                                   : node->declare_parameter<double>("exploration/plan_fail_circuit_sec", 2.5);
    ep_->top_view_num_                       = node->has_parameter("exploration/top_view_num") ? node->get_parameter("exploration/top_view_num").as_int()
                                                                                               : node->declare_parameter<int>("exploration/top_view_num", -1);
    ep_->max_decay_                          = node->has_parameter("exploration/max_decay") ? node->get_parameter("exploration/max_decay").as_double()
                                                                                            : node->declare_parameter<double>("exploration/max_decay", -1.0);
    ep_->tsp_dir_                            = node->has_parameter("exploration/tsp_dir") ? node->get_parameter("exploration/tsp_dir").as_string()
                                                                                          : node->declare_parameter<std::string>("exploration/tsp_dir", std::string("null"));
    ep_->mtsp_dir_                           = node->has_parameter("exploration/mtsp_dir") ? node->get_parameter("exploration/mtsp_dir").as_string()
                                                                                           : node->declare_parameter<std::string>("exploration/mtsp_dir", std::string("null"));
    ep_->lkh_binary_                         = node->has_parameter("exploration/lkh_binary") ? node->get_parameter("exploration/lkh_binary").as_string()
                                                                                             : node->declare_parameter<std::string>("exploration/lkh_binary", std::string("LKH"));
    ep_->relax_time_                         = node->has_parameter("exploration/relax_time") ? node->get_parameter("exploration/relax_time").as_double()
                                                                                             : node->declare_parameter<double>("exploration/relax_time", 1.0);
    ep_->drone_num_                          = node->has_parameter("exploration/drone_num") ? node->get_parameter("exploration/drone_num").as_int()
                                                                                            : node->declare_parameter<int>("exploration/drone_num", 1);
    ep_->drone_id_                           = node->has_parameter("exploration/drone_id") ? node->get_parameter("exploration/drone_id").as_int()
                                                                                           : node->declare_parameter<int>("exploration/drone_id", 1);
    ep_->solo_assign_enable_    = node->has_parameter("exploration/solo_assign_enable") ? node->get_parameter("exploration/solo_assign_enable").as_bool()
                                                                                        : node->declare_parameter<bool>("exploration/solo_assign_enable", true);
    ep_->solo_assign_max_grids_ = node->has_parameter("exploration/solo_assign_max_grids")
                                      ? node->get_parameter("exploration/solo_assign_max_grids").as_int()
                                      : node->declare_parameter<int>("exploration/solo_assign_max_grids", 8);
    ep_->solo_assign_radius_    = node->has_parameter("exploration/solo_assign_radius") ? node->get_parameter("exploration/solo_assign_radius").as_double()
                                                                                        : node->declare_parameter<double>("exploration/solo_assign_radius", 12.0);
    ep_->mtsp_fallback_max_goal_dist_ = node->has_parameter("exploration/mtsp_fallback_max_goal_dist")
                                            ? node->get_parameter("exploration/mtsp_fallback_max_goal_dist").as_double()
                                            : node->declare_parameter<double>("exploration/mtsp_fallback_max_goal_dist", 0.0);
    ep_->init_plan_num_               = node->has_parameter("exploration/init_plan_num") ? node->get_parameter("exploration/init_plan_num").as_int()
                                                                                         : node->declare_parameter<int>("exploration/init_plan_num", 2);

    ed_->swarm_state_.resize(ep_->drone_num_);
    ed_->pair_opt_stamps_.resize(ep_->drone_num_);
    ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);
    ed_->assigned_regions_.assign(ep_->drone_num_, -1);
    ed_->assigned_regions_stamp_  = 0.0;
    ed_->assigned_regions_epoch_  = 0;
    ed_->assigned_regions_sender_ = ep_->drone_id_;
    ed_->assigned_regions_online_mask_.assign(ep_->drone_num_, 0);
    ed_->peer_assigned_regions_.assign(ep_->drone_num_, {});
    ed_->peer_assigned_regions_stamp_.assign(ep_->drone_num_, 0.0);
    ed_->peer_assigned_regions_epoch_.assign(ep_->drone_num_, 0);
    ed_->peer_assigned_regions_online_mask_.assign(ep_->drone_num_, {});
    for (int i = 0; i < ep_->drone_num_; ++i)
    {
        ed_->swarm_state_[i].stamp_  = 0.0;
        ed_->pair_opt_stamps_[i]     = 0.0;
        ed_->pair_opt_res_stamps_[i] = 0.0;
    }
    planner_manager_->swarm_traj_data_.init(ep_->drone_id_, ep_->drone_num_);

    ViewNode::vm_ =
        node->has_parameter("exploration/vm") ? node->get_parameter("exploration/vm").as_double() : node->declare_parameter<double>("exploration/vm", -1.0);
    ViewNode::am_ =
        node->has_parameter("exploration/am") ? node->get_parameter("exploration/am").as_double() : node->declare_parameter<double>("exploration/am", -1.0);
    ViewNode::yd_ =
        node->has_parameter("exploration/yd") ? node->get_parameter("exploration/yd").as_double() : node->declare_parameter<double>("exploration/yd", -1.0);
    ViewNode::ydd_ =
        node->has_parameter("exploration/ydd") ? node->get_parameter("exploration/ydd").as_double() : node->declare_parameter<double>("exploration/ydd", -1.0);
    ViewNode::w_dir_ = node->has_parameter("exploration/w_dir") ? node->get_parameter("exploration/w_dir").as_double()
                                                                : node->declare_parameter<double>("exploration/w_dir", -1.0);

    ViewNode::astar_.reset(new Astar);
    ViewNode::astar_->init(node, edt_environment_);
    ViewNode::map_ = sdf_map_;

    double          resolution_ = sdf_map_->getResolution();
    Eigen::Vector3d origin, size;
    sdf_map_->getRegion(origin, size);
    ViewNode::caster_.reset(new RayCaster);
    ViewNode::caster_->setParams(resolution_, origin);

    planner_manager_->path_finder_->lambda_heu_ = 1.0;
    // planner_manager_->path_finder_->max_search_time_ = 0.05;
    planner_manager_->path_finder_->max_search_time_ = 1.0;

    // Create service clients for TSP/MTSP solver
    tsp_client_   = node->create_client<lkh_tsp_solver::srv::SolveTSP>("/solve_tsp_" + to_string(ep_->drone_id_));
    acvrp_client_ = node->create_client<lkh_mtsp_solver::srv::SolveMTSP>("/solve_acvrp_" + to_string(ep_->drone_id_));

    // Swarm
    for (auto& state : ed_->swarm_state_)
    {
        state.stamp_                = 0.0;
        state.recent_interact_time_ = 0.0;
        state.recent_attempt_time_  = 0.0;
    }
    ed_->last_grid_ids_  = {};
    ed_->reallocated_    = true;
    ed_->pair_opt_stamp_ = 0.0;
    ed_->wait_response_  = false;
    ed_->plan_num_       = 0;

    if (node->has_parameter("sdf_map/map_size_x"))
    {
        const auto map_size_x_param = node->get_parameter("sdf_map/map_size_x");
        x_size                      = (map_size_x_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) ? static_cast<double>(map_size_x_param.as_int())
                                                                                                                : map_size_x_param.as_double();
    }
    else
    {
        x_size = static_cast<double>(node->declare_parameter<int>("sdf_map/map_size_x", 25));  // 地图总长
    }
    if (node->has_parameter("sdf_map/map_size_y"))
    {
        const auto map_size_y_param = node->get_parameter("sdf_map/map_size_y");
        y_size                      = (map_size_y_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) ? static_cast<double>(map_size_y_param.as_int())
                                                                                                                : map_size_y_param.as_double();
    }
    else
    {
        y_size = static_cast<double>(node->declare_parameter<int>("sdf_map/map_size_y", 25));
    }

    // Stage-2 sync: declare/load launch-injected params to ROS2 node.
    const bool   partitioning_dead_zone_prune_enable    = node->has_parameter("partitioning/dead_zone_prune_enable")
                                                              ? node->get_parameter("partitioning/dead_zone_prune_enable").as_bool()
                                                              : node->declare_parameter<bool>("partitioning/dead_zone_prune_enable", true);
    const double partitioning_dead_zone_edge_sample_res = node->has_parameter("partitioning/dead_zone_edge_sample_res")
                                                              ? node->get_parameter("partitioning/dead_zone_edge_sample_res").as_double()
                                                              : node->declare_parameter<double>("partitioning/dead_zone_edge_sample_res", 0.2);
    const bool   partitioning_inactive_zone_enable      = node->has_parameter("partitioning/inactive_zone_enable")
                                                              ? node->get_parameter("partitioning/inactive_zone_enable").as_bool()
                                                              : node->declare_parameter<bool>("partitioning/inactive_zone_enable", true);
    const double partitioning_inactive_zone_x_min       = node->has_parameter("partitioning/inactive_zone_x_min")
                                                              ? node->get_parameter("partitioning/inactive_zone_x_min").as_double()
                                                              : node->declare_parameter<double>("partitioning/inactive_zone_x_min", -20.0);
    const double partitioning_inactive_zone_x_max       = node->has_parameter("partitioning/inactive_zone_x_max")
                                                              ? node->get_parameter("partitioning/inactive_zone_x_max").as_double()
                                                              : node->declare_parameter<double>("partitioning/inactive_zone_x_max", 20.0);
    const double partitioning_inactive_zone_y_min       = node->has_parameter("partitioning/inactive_zone_y_min")
                                                              ? node->get_parameter("partitioning/inactive_zone_y_min").as_double()
                                                              : node->declare_parameter<double>("partitioning/inactive_zone_y_min", -10.0);
    const double partitioning_inactive_zone_y_max       = node->has_parameter("partitioning/inactive_zone_y_max")
                                                              ? node->get_parameter("partitioning/inactive_zone_y_max").as_double()
                                                              : node->declare_parameter<double>("partitioning/inactive_zone_y_max", 10.0);
    const bool   partitioning_visit_prune_enable        = node->has_parameter("partitioning/visit_prune_enable")
                                                              ? node->get_parameter("partitioning/visit_prune_enable").as_bool()
                                                              : node->declare_parameter<bool>("partitioning/visit_prune_enable", false);
    const int    partitioning_visit_prune_min_hits      = node->has_parameter("partitioning/visit_prune_min_hits")
                                                              ? node->get_parameter("partitioning/visit_prune_min_hits").as_int()
                                                              : node->declare_parameter<int>("partitioning/visit_prune_min_hits", 2);
    const double partitioning_visit_prune_min_sec       = node->has_parameter("partitioning/visit_prune_min_sec")
                                                              ? node->get_parameter("partitioning/visit_prune_min_sec").as_double()
                                                              : node->declare_parameter<double>("partitioning/visit_prune_min_sec", 4.0);

    const double manager_vehicle_length = node->has_parameter("manager/vehicle_length") ? node->get_parameter("manager/vehicle_length").as_double()
                                                                                        : node->declare_parameter<double>("manager/vehicle_length", 1.045);
    const double manager_vehicle_width  = node->has_parameter("manager/vehicle_width") ? node->get_parameter("manager/vehicle_width").as_double()
                                                                                       : node->declare_parameter<double>("manager/vehicle_width", 0.78);
    const bool   optimization_use_vehicle_footprint_distance = node->has_parameter("optimization/use_vehicle_footprint_distance")
                                                                   ? node->get_parameter("optimization/use_vehicle_footprint_distance").as_bool()
                                                                   : node->declare_parameter<bool>("optimization/use_vehicle_footprint_distance", true);
    const double optimization_vehicle_clearance              = node->has_parameter("optimization/vehicle_clearance")
                                                                   ? node->get_parameter("optimization/vehicle_clearance").as_double()
                                                                   : node->declare_parameter<double>("optimization/vehicle_clearance", 0.05);

    const double swarm_collision_robot_radius  = node->has_parameter("swarm_collision/robot_radius")
                                                     ? node->get_parameter("swarm_collision/robot_radius").as_double()
                                                     : node->declare_parameter<double>("swarm_collision/robot_radius", 0.7);
    const double swarm_collision_safe_margin   = node->has_parameter("swarm_collision/safe_margin")
                                                     ? node->get_parameter("swarm_collision/safe_margin").as_double()
                                                     : node->declare_parameter<double>("swarm_collision/safe_margin", 0.35);
    const double swarm_collision_horizon       = node->has_parameter("swarm_collision/horizon") ? node->get_parameter("swarm_collision/horizon").as_double()
                                                                                                : node->declare_parameter<double>("swarm_collision/horizon", 5.0);
    const double swarm_collision_ttc_threshold = node->has_parameter("swarm_collision/ttc_threshold")
                                                     ? node->get_parameter("swarm_collision/ttc_threshold").as_double()
                                                     : node->declare_parameter<double>("swarm_collision/ttc_threshold", 1.5);
    const double swarm_collision_delay_gain    = node->has_parameter("swarm_collision/delay_gain")
                                                     ? node->get_parameter("swarm_collision/delay_gain").as_double()
                                                     : node->declare_parameter<double>("swarm_collision/delay_gain", 1.6);
    const double swarm_collision_box_inflation = node->has_parameter("swarm_collision/box_inflation")
                                                     ? node->get_parameter("swarm_collision/box_inflation").as_double()
                                                     : node->declare_parameter<double>("swarm_collision/box_inflation", 0.15);
    const double swarm_collision_ahead_eps     = node->has_parameter("swarm_collision/ahead_eps")
                                                     ? node->get_parameter("swarm_collision/ahead_eps").as_double()
                                                     : node->declare_parameter<double>("swarm_collision/ahead_eps", 0.25);
    const bool   swarm_collision_adaptive_horizon_enable = node->has_parameter("swarm_collision/adaptive_horizon_enable")
                                                               ? node->get_parameter("swarm_collision/adaptive_horizon_enable").as_bool()
                                                               : node->declare_parameter<bool>("swarm_collision/adaptive_horizon_enable", true);
    const double swarm_collision_adaptive_horizon_max    = node->has_parameter("swarm_collision/adaptive_horizon_max")
                                                               ? node->get_parameter("swarm_collision/adaptive_horizon_max").as_double()
                                                               : node->declare_parameter<double>("swarm_collision/adaptive_horizon_max", 6.0);
    const double swarm_collision_adaptive_horizon_rel_speed_thresh =
        node->has_parameter("swarm_collision/adaptive_horizon_rel_speed_thresh")
            ? node->get_parameter("swarm_collision/adaptive_horizon_rel_speed_thresh").as_double()
            : node->declare_parameter<double>("swarm_collision/adaptive_horizon_rel_speed_thresh", 0.6);
    const double swarm_collision_adaptive_horizon_gain = node->has_parameter("swarm_collision/adaptive_horizon_gain")
                                                             ? node->get_parameter("swarm_collision/adaptive_horizon_gain").as_double()
                                                             : node->declare_parameter<double>("swarm_collision/adaptive_horizon_gain", 2.0);
    const double swarm_collision_static_safe_margin    = node->has_parameter("swarm_collision/static_safe_margin")
                                                             ? node->get_parameter("swarm_collision/static_safe_margin").as_double()
                                                             : node->declare_parameter<double>("swarm_collision/static_safe_margin", 0.199);
    const double swarm_collision_static_horizon        = node->has_parameter("swarm_collision/static_horizon")
                                                             ? node->get_parameter("swarm_collision/static_horizon").as_double()
                                                             : node->declare_parameter<double>("swarm_collision/static_horizon", 5.0);
    const double swarm_collision_static_min_dt         = node->has_parameter("swarm_collision/static_min_dt")
                                                             ? node->get_parameter("swarm_collision/static_min_dt").as_double()
                                                             : node->declare_parameter<double>("swarm_collision/static_min_dt", 0.02);
    const double swarm_collision_static_max_dt         = node->has_parameter("swarm_collision/static_max_dt")
                                                             ? node->get_parameter("swarm_collision/static_max_dt").as_double()
                                                             : node->declare_parameter<double>("swarm_collision/static_max_dt", 0.10);
    const double swarm_collision_static_odom_delay_cap = node->has_parameter("swarm_collision/static_odom_delay_cap")
                                                             ? node->get_parameter("swarm_collision/static_odom_delay_cap").as_double()
                                                             : node->declare_parameter<double>("swarm_collision/static_odom_delay_cap", 0.25);
    const double swarm_collision_static_bias_gain      = node->has_parameter("swarm_collision/static_bias_gain")
                                                             ? node->get_parameter("swarm_collision/static_bias_gain").as_double()
                                                             : node->declare_parameter<double>("swarm_collision/static_bias_gain", 0.8);
    const double swarm_collision_static_ahead_eps      = node->has_parameter("swarm_collision/static_ahead_eps")
                                                             ? node->get_parameter("swarm_collision/static_ahead_eps").as_double()
                                                             : node->declare_parameter<double>("swarm_collision/static_ahead_eps", 0.25);

    const bool   allocate_enable_cross_region_takeover = node->has_parameter("allocate/enable_cross_region_takeover")
                                                             ? node->get_parameter("allocate/enable_cross_region_takeover").as_bool()
                                                             : node->declare_parameter<bool>("allocate/enable_cross_region_takeover", false);
    const double allocate_takeover_idle_sec            = node->has_parameter("allocate/takeover_idle_sec")
                                                             ? node->get_parameter("allocate/takeover_idle_sec").as_double()
                                                             : node->declare_parameter<double>("allocate/takeover_idle_sec", 8.0);
    const int    allocate_region_rows                  = node->has_parameter("allocate/region_rows") ? node->get_parameter("allocate/region_rows").as_int()
                                                                                                     : node->declare_parameter<int>("allocate/region_rows", 2);
    const int    allocate_region_cols                  = node->has_parameter("allocate/region_cols") ? node->get_parameter("allocate/region_cols").as_int()
                                                                                                     : node->declare_parameter<int>("allocate/region_cols", 3);
    const bool   allocate_use_snn_region_assignment    = node->has_parameter("allocate/use_snn_region_assignment")
                                                             ? node->get_parameter("allocate/use_snn_region_assignment").as_bool()
                                                             : node->declare_parameter<bool>("allocate/use_snn_region_assignment", true);
    const double allocate_snn_tau                      = node->has_parameter("allocate/snn_tau") ? node->get_parameter("allocate/snn_tau").as_double()
                                                                                                 : node->declare_parameter<double>("allocate/snn_tau", 2.8);
    const double allocate_snn_dt =
        node->has_parameter("allocate/snn_dt") ? node->get_parameter("allocate/snn_dt").as_double() : node->declare_parameter<double>("allocate/snn_dt", 0.2);
    const double allocate_snn_threshold       = node->has_parameter("allocate/snn_threshold") ? node->get_parameter("allocate/snn_threshold").as_double()
                                                                                              : node->declare_parameter<double>("allocate/snn_threshold", 0.26);
    const int    allocate_snn_max_iter        = node->has_parameter("allocate/snn_max_iter") ? node->get_parameter("allocate/snn_max_iter").as_int()
                                                                                             : node->declare_parameter<int>("allocate/snn_max_iter", 300);
    const double allocate_snn_distance_weight = node->has_parameter("allocate/snn_distance_weight")
                                                    ? node->get_parameter("allocate/snn_distance_weight").as_double()
                                                    : node->declare_parameter<double>("allocate/snn_distance_weight", 1.0);
    const double allocate_snn_load_weight     = node->has_parameter("allocate/snn_load_weight") ? node->get_parameter("allocate/snn_load_weight").as_double()
                                                                                                : node->declare_parameter<double>("allocate/snn_load_weight", 0.8);
    const double allocate_repel_threshold     = node->has_parameter("allocate/repel_threshold") ? node->get_parameter("allocate/repel_threshold").as_double()
                                                                                                : node->declare_parameter<double>("allocate/repel_threshold", 10.0);
    const double allocate_repel_scale         = node->has_parameter("allocate/repel_scale") ? node->get_parameter("allocate/repel_scale").as_double()
                                                                                            : node->declare_parameter<double>("allocate/repel_scale", 3.0);
    const double allocate_pair_overlap_radius = node->has_parameter("allocate/pair_overlap_radius")
                                                    ? node->get_parameter("allocate/pair_overlap_radius").as_double()
                                                    : node->declare_parameter<double>("allocate/pair_overlap_radius", 8.0);
    const double allocate_pair_overlap_penalty_scale              = node->has_parameter("allocate/pair_overlap_penalty_scale")
                                                                        ? node->get_parameter("allocate/pair_overlap_penalty_scale").as_double()
                                                                        : node->declare_parameter<double>("allocate/pair_overlap_penalty_scale", 3.6);
    const bool   allocate_enable_pair_entry_hard_split            = node->has_parameter("allocate/enable_pair_entry_hard_split")
                                                                        ? node->get_parameter("allocate/enable_pair_entry_hard_split").as_bool()
                                                                        : node->declare_parameter<bool>("allocate/enable_pair_entry_hard_split", true);
    const double allocate_pair_entry_hard_split_radius            = node->has_parameter("allocate/pair_entry_hard_split_radius")
                                                                        ? node->get_parameter("allocate/pair_entry_hard_split_radius").as_double()
                                                                        : node->declare_parameter<double>("allocate/pair_entry_hard_split_radius", 9.5);
    const double allocate_pair_entry_hard_split_margin            = node->has_parameter("allocate/pair_entry_hard_split_margin")
                                                                        ? node->get_parameter("allocate/pair_entry_hard_split_margin").as_double()
                                                                        : node->declare_parameter<double>("allocate/pair_entry_hard_split_margin", 1.8);
    const bool   allocate_enable_soft_cross_region_constraint     = node->has_parameter("allocate/enable_soft_cross_region_constraint")
                                                                        ? node->get_parameter("allocate/enable_soft_cross_region_constraint").as_bool()
                                                                        : node->declare_parameter<bool>("allocate/enable_soft_cross_region_constraint", true);
    const double allocate_cross_region_soft_penalty               = node->has_parameter("allocate/cross_region_soft_penalty")
                                                                        ? node->get_parameter("allocate/cross_region_soft_penalty").as_double()
                                                                        : node->declare_parameter<double>("allocate/cross_region_soft_penalty", 180.0);
    const double allocate_cross_region_penalty_gain_per_hop       = node->has_parameter("allocate/cross_region_penalty_gain_per_hop")
                                                                        ? node->get_parameter("allocate/cross_region_penalty_gain_per_hop").as_double()
                                                                        : node->declare_parameter<double>("allocate/cross_region_penalty_gain_per_hop", 0.35);
    const bool   allocate_enable_dynamic_region_relayout          = node->has_parameter("allocate/enable_dynamic_region_relayout")
                                                                        ? node->get_parameter("allocate/enable_dynamic_region_relayout").as_bool()
                                                                        : node->declare_parameter<bool>("allocate/enable_dynamic_region_relayout", true);
    const double allocate_dynamic_region_relayout_cooldown        = node->has_parameter("allocate/dynamic_region_relayout_cooldown")
                                                                        ? node->get_parameter("allocate/dynamic_region_relayout_cooldown").as_double()
                                                                        : node->declare_parameter<double>("allocate/dynamic_region_relayout_cooldown", 4.0);
    const double allocate_dynamic_region_online_timeout           = node->has_parameter("allocate/dynamic_region_online_timeout")
                                                                        ? node->get_parameter("allocate/dynamic_region_online_timeout").as_double()
                                                                        : node->declare_parameter<double>("allocate/dynamic_region_online_timeout", 1.2);
    const int    allocate_dynamic_region_online_stable_cycles     = node->has_parameter("allocate/dynamic_region_online_stable_cycles")
                                                                        ? node->get_parameter("allocate/dynamic_region_online_stable_cycles").as_int()
                                                                        : node->declare_parameter<int>("allocate/dynamic_region_online_stable_cycles", 4);
    const int    allocate_dynamic_region_unfinished_stable_cycles = node->has_parameter("allocate/dynamic_region_unfinished_stable_cycles")
                                                                        ? node->get_parameter("allocate/dynamic_region_unfinished_stable_cycles").as_int()
                                                                        : node->declare_parameter<int>("allocate/dynamic_region_unfinished_stable_cycles", 4);
    const double allocate_dynamic_region_bounds_margin_m          = node->has_parameter("allocate/dynamic_region_bounds_margin_m")
                                                                        ? node->get_parameter("allocate/dynamic_region_bounds_margin_m").as_double()
                                                                        : node->declare_parameter<double>("allocate/dynamic_region_bounds_margin_m", 1.5);
    const double allocate_dynamic_region_bounds_min_span_m        = node->has_parameter("allocate/dynamic_region_bounds_min_span_m")
                                                                        ? node->get_parameter("allocate/dynamic_region_bounds_min_span_m").as_double()
                                                                        : node->declare_parameter<double>("allocate/dynamic_region_bounds_min_span_m", 10.0);
    const bool   allocate_dynamic_region_bounds_use_frontier_aabb =
        node->has_parameter("allocate/dynamic_region_bounds_use_frontier_aabb")
              ? node->get_parameter("allocate/dynamic_region_bounds_use_frontier_aabb").as_bool()
              : node->declare_parameter<bool>("allocate/dynamic_region_bounds_use_frontier_aabb", false);
    const bool allocate_enforce_region_execution_consistency = node->has_parameter("allocate/enforce_region_execution_consistency")
                                                                   ? node->get_parameter("allocate/enforce_region_execution_consistency").as_bool()
                                                                   : node->declare_parameter<bool>("allocate/enforce_region_execution_consistency", true);
    const bool allocate_enable_periodic_global_region_assignment =
        node->has_parameter("allocate/enable_periodic_global_region_assignment")
            ? node->get_parameter("allocate/enable_periodic_global_region_assignment").as_bool()
            : node->declare_parameter<bool>("allocate/enable_periodic_global_region_assignment", true);
    const double allocate_periodic_global_region_assign_sec = node->has_parameter("allocate/periodic_global_region_assign_sec")
                                                                  ? node->get_parameter("allocate/periodic_global_region_assign_sec").as_double()
                                                                  : node->declare_parameter<double>("allocate/periodic_global_region_assign_sec", 2.0);
    const int    allocate_periodic_global_assign_min_online = node->has_parameter("allocate/periodic_global_assign_min_online")
                                                                  ? node->get_parameter("allocate/periodic_global_assign_min_online").as_int()
                                                                  : node->declare_parameter<int>("allocate/periodic_global_assign_min_online", 3);
    const bool   allocate_enable_mtsp_grid_reduction        = node->has_parameter("allocate/enable_mtsp_grid_reduction")
                                                                  ? node->get_parameter("allocate/enable_mtsp_grid_reduction").as_bool()
                                                                  : node->declare_parameter<bool>("allocate/enable_mtsp_grid_reduction", false);
    const int    allocate_mtsp_max_rep_grids                = node->has_parameter("allocate/mtsp_max_rep_grids")
                                                                  ? node->get_parameter("allocate/mtsp_max_rep_grids").as_int()
                                                                  : node->declare_parameter<int>("allocate/mtsp_max_rep_grids", 48);
    const double allocate_mtsp_near_radius_m                = node->has_parameter("allocate/mtsp_near_radius_m")
                                                                  ? node->get_parameter("allocate/mtsp_near_radius_m").as_double()
                                                                  : node->declare_parameter<double>("allocate/mtsp_near_radius_m", 30.0);
    const double allocate_mtsp_cluster_merge_radius_m       = node->has_parameter("allocate/mtsp_cluster_merge_radius_m")
                                                                  ? node->get_parameter("allocate/mtsp_cluster_merge_radius_m").as_double()
                                                                  : node->declare_parameter<double>("allocate/mtsp_cluster_merge_radius_m", 10.0);
    const bool   allocate_enable_grid_cluster_owner         = node->has_parameter("allocate/enable_grid_cluster_owner")
                                                                  ? node->get_parameter("allocate/enable_grid_cluster_owner").as_bool()
                                                                  : node->declare_parameter<bool>("allocate/enable_grid_cluster_owner", true);
    const double allocate_grid_cluster_owner_radius_m       = node->has_parameter("allocate/grid_cluster_owner_radius_m")
                                                                  ? node->get_parameter("allocate/grid_cluster_owner_radius_m").as_double()
                                                                  : node->declare_parameter<double>("allocate/grid_cluster_owner_radius_m", 9.0);
    const double allocate_grid_cluster_owner_penalty        = node->has_parameter("allocate/grid_cluster_owner_penalty")
                                                                  ? node->get_parameter("allocate/grid_cluster_owner_penalty").as_double()
                                                                  : node->declare_parameter<double>("allocate/grid_cluster_owner_penalty", 360.0);
    const double allocate_grid_cluster_owner_timeout_sec    = node->has_parameter("allocate/grid_cluster_owner_timeout_sec")
                                                                  ? node->get_parameter("allocate/grid_cluster_owner_timeout_sec").as_double()
                                                                  : node->declare_parameter<double>("allocate/grid_cluster_owner_timeout_sec", 2.0);

    (void)partitioning_dead_zone_prune_enable;
    (void)partitioning_dead_zone_edge_sample_res;
    (void)partitioning_inactive_zone_enable;
    (void)partitioning_inactive_zone_x_min;
    (void)partitioning_inactive_zone_x_max;
    (void)partitioning_inactive_zone_y_min;
    (void)partitioning_inactive_zone_y_max;
    (void)partitioning_visit_prune_enable;
    (void)partitioning_visit_prune_min_hits;
    (void)partitioning_visit_prune_min_sec;
    (void)manager_vehicle_length;
    (void)manager_vehicle_width;
    (void)optimization_use_vehicle_footprint_distance;
    (void)optimization_vehicle_clearance;
    (void)swarm_collision_robot_radius;
    (void)swarm_collision_safe_margin;
    (void)swarm_collision_horizon;
    (void)swarm_collision_ttc_threshold;
    (void)swarm_collision_delay_gain;
    (void)swarm_collision_box_inflation;
    (void)swarm_collision_ahead_eps;
    (void)swarm_collision_adaptive_horizon_enable;
    (void)swarm_collision_adaptive_horizon_max;
    (void)swarm_collision_adaptive_horizon_rel_speed_thresh;
    (void)swarm_collision_adaptive_horizon_gain;
    (void)swarm_collision_static_safe_margin;
    (void)swarm_collision_static_horizon;
    (void)swarm_collision_static_min_dt;
    (void)swarm_collision_static_max_dt;
    (void)swarm_collision_static_odom_delay_cap;
    (void)swarm_collision_static_bias_gain;
    (void)swarm_collision_static_ahead_eps;
    alloc_cfg_.enable_cross_region_takeover_ = allocate_enable_cross_region_takeover;
    alloc_cfg_.takeover_idle_sec_            = allocate_takeover_idle_sec;
    (void)allocate_region_rows;
    (void)allocate_region_cols;
    (void)allocate_use_snn_region_assignment;
    (void)allocate_snn_tau;
    (void)allocate_snn_dt;
    (void)allocate_snn_threshold;
    (void)allocate_snn_max_iter;
    (void)allocate_snn_distance_weight;
    (void)allocate_snn_load_weight;
    alloc_cfg_.repel_threshold_                          = allocate_repel_threshold;
    alloc_cfg_.repel_scale_                              = allocate_repel_scale;
    alloc_cfg_.pair_overlap_radius_                      = allocate_pair_overlap_radius;
    alloc_cfg_.pair_overlap_penalty_scale_               = allocate_pair_overlap_penalty_scale;
    alloc_cfg_.enable_pair_entry_hard_split_             = allocate_enable_pair_entry_hard_split;
    alloc_cfg_.pair_entry_hard_split_radius_             = allocate_pair_entry_hard_split_radius;
    alloc_cfg_.pair_entry_hard_split_margin_             = allocate_pair_entry_hard_split_margin;
    alloc_cfg_.enable_soft_cross_region_constraint_      = allocate_enable_soft_cross_region_constraint;
    alloc_cfg_.cross_region_soft_penalty_                = allocate_cross_region_soft_penalty;
    alloc_cfg_.cross_region_penalty_gain_per_hop_        = allocate_cross_region_penalty_gain_per_hop;
    alloc_cfg_.enable_dynamic_region_relayout_           = allocate_enable_dynamic_region_relayout;
    alloc_cfg_.dynamic_region_relayout_cooldown_         = allocate_dynamic_region_relayout_cooldown;
    alloc_cfg_.dynamic_region_online_timeout_            = allocate_dynamic_region_online_timeout;
    alloc_cfg_.dynamic_region_online_stable_cycles_      = allocate_dynamic_region_online_stable_cycles;
    alloc_cfg_.dynamic_region_unfinished_stable_cycles_  = allocate_dynamic_region_unfinished_stable_cycles;
    alloc_cfg_.dynamic_region_bounds_margin_m_           = allocate_dynamic_region_bounds_margin_m;
    alloc_cfg_.dynamic_region_bounds_min_span_m_         = allocate_dynamic_region_bounds_min_span_m;
    alloc_cfg_.dynamic_region_bounds_use_frontier_aabb_  = allocate_dynamic_region_bounds_use_frontier_aabb;
    alloc_cfg_.enforce_region_execution_consistency_     = allocate_enforce_region_execution_consistency;
    alloc_cfg_.enable_periodic_global_region_assignment_ = allocate_enable_periodic_global_region_assignment;
    alloc_cfg_.periodic_global_region_assign_sec_        = allocate_periodic_global_region_assign_sec;
    alloc_cfg_.periodic_global_assign_min_online_        = allocate_periodic_global_assign_min_online;
    alloc_cfg_.enable_mtsp_grid_reduction_               = allocate_enable_mtsp_grid_reduction;
    alloc_cfg_.mtsp_max_rep_grids_                       = allocate_mtsp_max_rep_grids;
    alloc_cfg_.mtsp_near_radius_m_                       = allocate_mtsp_near_radius_m;
    alloc_cfg_.mtsp_cluster_merge_radius_m_              = allocate_mtsp_cluster_merge_radius_m;
    alloc_cfg_.enable_grid_cluster_owner_                = allocate_enable_grid_cluster_owner;
    alloc_cfg_.grid_cluster_owner_radius_m_              = allocate_grid_cluster_owner_radius_m;
    alloc_cfg_.grid_cluster_owner_penalty_               = allocate_grid_cluster_owner_penalty;
    alloc_cfg_.grid_cluster_owner_timeout_sec_           = allocate_grid_cluster_owner_timeout_sec;

    region_allocator_->configure(node_, x_size, y_size, ep_->drone_num_);
}

/**
 * @brief 根据当前网格和前沿情况，决定先飞哪个网格的哪个视点，然后调用planTrajToView 生成具体的路径
 */
int FastExplorationManager::planExploreMotion(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, const Eigen::Vector3d& yaw)
{

    auto t1 = node_->now();
    auto t2 = t1;

    ed_->frontier_tour_.clear();
    Eigen::Vector3d                                 next_pos;
    double                                          next_yaw;
    std::vector<int>                                grid_ids, frontier_ids;
    std::vector<std::pair<Eigen::Vector3d, double>> goal_candidates;

    // P0-2：每周期重建 SNN grid 并计算死区可达掩码，供 PickBestUtilityViewpoint /
    // recoverStaleGridAssignment / pickNearestFrontierWithinRadius / 网格中心选择过滤不可达目标。
    // 掩码失败时 IsReachablePoint 放行，保持旧行为。
    UpdateReachabilityMask(pos);

    findGridAndFrontierPath(pos, vel, yaw, grid_ids, frontier_ids);

    // 强制所有视点的 z=0.3 (匹配小车相机/规划高度)
    for (auto& point : ed_->points_)
    {
        point.z() = 0.3;
    }
    for (auto& avg : ed_->averages_)
    {
        avg.z() = 0.3;
    }

    // ============ Helper lambdas for deduplication ============
    auto applyFrontierTarget = [&](int fid) {
        next_pos     = ed_->points_[fid];
        next_pos.z() = 0.3;
        next_yaw     = ed_->yaws_[fid];
        goal_candidates.emplace_back(next_pos, next_yaw);
    };
    auto applyGridCenterTarget = [&](const Eigen::Vector3d& center) {
        next_pos     = center;
        next_pos.z() = 0.3;
        next_yaw     = std::atan2(center.y() - pos.y(), center.x() - pos.x());
        goal_candidates.emplace_back(next_pos, next_yaw);
    };
    auto applyRecoverStaleFallback = [&]() -> bool {
        if (recoverStaleGridAssignment(pos, grid_ids, frontier_ids) && !frontier_ids.empty())
        {
            applyFrontierTarget(frontier_ids[0]);
            return true;
        }
        return false;
    };
    auto applyHover = [&]() {
        next_pos = pos;
        next_yaw = yaw[0];
    };
    // P0-2：网格中心不可达（死区对侧等）时悬停，避免飞向 SNN 到不了的位置
    auto applyGridCenterTargetIfReachable = [&](const Eigen::Vector3d& center) -> bool {
        if (!IsReachablePoint(center))
        {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("exploration_manager"), *node_->get_clock(), 1000,
                                 "[planExploreMotion][d%d] grid center [%.2f, %.2f] unreachable (deadzone?); hover", ep_->drone_id_, center.x(), center.y());
            applyHover();
            return false;
        }
        applyGridCenterTarget(center);
        return true;
    };
    // =========================================================

    //! 情形1：没有网格，遍历所有前沿的最佳视点，计算当前位置到每个视点的成本，找到成本最小的
    if (grid_ids.empty())
    {
        const int min_cost_id = PickBestUtilityViewpoint(
            *ed_, pos, vel, yaw, GetMaxLocalGoalDist(*ep_), 0.1, [this](const Eigen::Vector3d& p) { return IsReachablePoint(p); },
            [this](const Eigen::Vector3d& p) { return peerCoveragePenalty(p); });

        if (min_cost_id != -1)
        {
            applyFrontierTarget(min_cost_id);
        }
        else
        {
            // 如果所有点都被排除 (例如都太近)，保持当前位置
            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion] No viable viewpoint found. Sticking to current position.");
            applyHover();
        }
    }

    //! 情形2：当前网格没有前沿 → 跳转到下一个网格
    else if (frontier_ids.size() == 0)
    {
        if (ed_->grid_tour_.size() < 2)
        {
            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion] No frontier in grid and grid_tour unavailable; using greedy fallback.");
            const int min_cost_id = PickBestUtilityViewpoint(
                *ed_, pos, vel, yaw, -1.0, 0.1, [this](const Eigen::Vector3d& p) { return IsReachablePoint(p); },
                [this](const Eigen::Vector3d& p) { return peerCoveragePenalty(p); });
            if (min_cost_id >= 0)
                applyFrontierTarget(min_cost_id);
            else if (!applyRecoverStaleFallback())
                applyHover();
        }
        else
        {
            Eigen::Vector3d grid_center = ed_->grid_tour_[1];
            grid_center.z()             = 0.3;
            const double max_local_dist = GetMaxLocalGoalDist(*ep_);

            if ((grid_center - pos).head<2>().norm() > max_local_dist)
            {
                const int min_cost_id = PickBestUtilityViewpoint(
                    *ed_, pos, vel, yaw, max_local_dist, 0.1, [this](const Eigen::Vector3d& p) { return IsReachablePoint(p); },
                    [this](const Eigen::Vector3d& p) { return peerCoveragePenalty(p); });
                if (min_cost_id >= 0)
                    applyFrontierTarget(min_cost_id);
                else if (!applyRecoverStaleFallback() && applyGridCenterTargetIfReachable(grid_center))
                {
                    RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                                "[planExploreMotion][d%d] No local frontier; relocate toward grid center [%.2f, %.2f] (dist=%.2fm)", ep_->drone_id_,
                                grid_center.x(), grid_center.y(), (grid_center - pos).head<2>().norm());
                }
            }
            else
            {
                double min_cost    = 100000;
                int    min_cost_id = -1;

                for (int i = 0; i < static_cast<int>(ed_->points_.size()); ++i)
                {
                    // P0-2：剔除死区对侧/不可达前沿，避免最终目标无法规划
                    if (!IsReachablePoint(ed_->points_[i]))
                        continue;
                    Eigen::Vector3d avg = ed_->averages_[i];
                    avg.z()             = 0.3;
                    std::vector<Eigen::Vector3d> path;
                    // ③ soft anti-clustering: peer-covered targets pay extra cost
                    double cost = ViewNode::computeCost(grid_center, avg, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path) + peerCoveragePenalty(avg);
                    if (cost < min_cost)
                    {
                        min_cost    = cost;
                        min_cost_id = i;
                    }
                }
                if (min_cost_id >= 0)
                    applyFrontierTarget(min_cost_id);
                else if (!applyRecoverStaleFallback())
                    applyGridCenterTargetIfReachable(grid_center);
            }
        }
    }

    //! 情形3：单一前沿， 生成候选视点，优化选择（贪婪或 Dijkstra），考虑下一网格以平滑过渡 (保持不变，但仍建议在 refineLocalTour 中优化)
    else if (frontier_ids.size() == 1)
    {
        if (ep_->refine_local_)
        {
            ed_->refined_ids_             = { frontier_ids[0] };
            ed_->unrefined_points_        = { ed_->points_[frontier_ids[0]] };
            ed_->unrefined_points_[0].z() = 0.3;
            ed_->n_points_.clear();
            std::vector<std::vector<double>> n_yaws;

            frontier_finder_->getViewpointsInfo(pos, { frontier_ids[0] }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

            for (auto& points : ed_->n_points_)
            {
                for (auto& p : points)
                    p.z() = 0.3;
            }

            if (grid_ids.size() <= 1)
            {
                const double                 max_local_dist = GetMaxLocalGoalDist(*ep_);
                double                       min_cost       = 100000;
                int                          min_cost_id    = -1;
                int                          fallback_id    = -1;
                double                       fallback_dist  = std::numeric_limits<double>::max();
                std::vector<Eigen::Vector3d> tmp_path;
                if (!ed_->n_points_.empty())
                {
                    for (int i = 0; i < static_cast<int>(ed_->n_points_[0].size()); ++i)
                    {
                        Eigen::Vector3d point = ed_->n_points_[0][i];
                        point.z()             = 0.3;
                        // P0-2：剔除死区对侧/不可达视点，避免 refine 后仍飞向 SNN 到不了的位置
                        if (!IsReachablePoint(point))
                            continue;
                        const double dist_xy = (point - pos).head<2>().norm();
                        if (dist_xy < fallback_dist)
                        {
                            fallback_dist = dist_xy;
                            fallback_id   = i;
                        }
                        if (dist_xy > max_local_dist)
                            continue;
                        auto tmp_cost = ViewNode::computeCost(pos, point, yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
                        if (tmp_cost < min_cost)
                        {
                            min_cost    = tmp_cost;
                            min_cost_id = i;
                        }
                    }
                }
                if (min_cost_id < 0)
                    min_cost_id = fallback_id;
                if (min_cost_id >= 0)
                {
                    next_pos     = ed_->n_points_[0][min_cost_id];
                    next_pos.z() = 0.3;
                    next_yaw     = n_yaws[0][min_cost_id];
                    // P0-2：goal_candidates 也会被 near_split 统一分流使用，只放可达视点
                    for (int i = 0; i < static_cast<int>(ed_->n_points_[0].size()); ++i)
                    {
                        Eigen::Vector3d cand = ed_->n_points_[0][i];
                        cand.z()             = 0.3;
                        if (IsReachablePoint(cand))
                            goal_candidates.emplace_back(ed_->n_points_[0][i], n_yaws[0][i]);
                    }
                }
                else
                {
                    // P0-2：所有候选视点都不可达时，连原始前沿中心也不要硬飞
                    Eigen::Vector3d raw_fp = ed_->points_[frontier_ids[0]];
                    raw_fp.z()             = 0.3;
                    if (IsReachablePoint(raw_fp))
                        applyFrontierTarget(frontier_ids[0]);
                    else
                        applyHover();
                }
            }
            else
            {
                Eigen::Vector3d grid_pos;
                double          grid_yaw;
                bool            has_next_grid = false;
                if (hgrid_->getNextGrid(grid_ids, grid_pos, grid_yaw))  // 获取下一网格的位置和偏航
                {
                    grid_pos.z()  = 0.3;
                    has_next_grid = true;
                }

                // P0-2：过滤当前前沿的不可达视点；若全部不可达，直接尝试下一网格中心。
                // getViewpointsInfo 可能找不到对应 frontier，n_points_ 可能为空，先做保护。
                if (ed_->n_points_.empty())
                {
                    if (has_next_grid)
                        applyGridCenterTargetIfReachable(grid_pos);
                    else
                        applyHover();
                }
                else
                {
                    auto&  ftr_pts  = ed_->n_points_[0];
                    auto&  ftr_yaws = n_yaws[0];
                    size_t write    = 0;
                    for (size_t j = 0; j < ftr_pts.size(); ++j)
                    {
                        Eigen::Vector3d p = ftr_pts[j];
                        p.z()             = 0.3;
                        if (!IsReachablePoint(p))
                            continue;
                        ftr_pts[write]  = ftr_pts[j];
                        ftr_yaws[write] = ftr_yaws[j];
                        ++write;
                    }
                    ftr_pts.resize(write);
                    ftr_yaws.resize(write);

                    if (ftr_pts.empty())
                    {
                        if (has_next_grid)
                            applyGridCenterTargetIfReachable(grid_pos);
                        else
                            applyHover();
                    }
                    else
                    {
                        if (has_next_grid)
                        {
                            ed_->n_points_.push_back({ grid_pos });
                            n_yaws.push_back({ grid_yaw });
                        }

                        ed_->refined_points_.clear();
                        ed_->refined_views_.clear();
                        std::vector<double> refined_yaws;

                        refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);  //优化候选视点顺序
                        for (auto& p : ed_->refined_points_)
                        {
                            p.z() = 0.3;
                        }
                        next_pos     = ed_->refined_points_[0];
                        next_pos.z() = 0.3;
                        next_yaw     = refined_yaws[0];
                        for (int i = 0; i < ed_->refined_points_.size(); ++i)
                            goal_candidates.emplace_back(ed_->refined_points_[i], refined_yaws[i]);
                    }
                }
                ed_->refined_points_ = { next_pos };
                ed_->refined_views_  = { next_pos + 2.0 * Eigen::Vector3d(cos(next_yaw), sin(next_yaw), 0) };
            }
        }
        else
        {
            // P0-2：refine_local_ 关闭时直接飞向原始前沿，仍需过滤死区对侧
            Eigen::Vector3d raw_fp = ed_->points_[frontier_ids[0]];
            raw_fp.z()             = 0.3;
            if (IsReachablePoint(raw_fp))
                applyFrontierTarget(frontier_ids[0]);
            else
                applyHover();
        }
    }

    //! 情形4： 多个前沿， 优化视点访问顺序
    else
    {
        // RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion] Multiple frontier. Refining tour.");
        t1 = node_->now();
        ed_->refined_ids_.clear();
        ed_->unrefined_points_.clear();
        // [优化] 按到当前位置的距离排序，优先选择近视点，避免舍近求远
        std::vector<std::pair<int, double>> ftr_dist;
        for (int fid : frontier_ids)
        {
            double d = (ed_->points_[fid] - pos).norm();
            ftr_dist.push_back({ fid, d });
        }
        std::sort(ftr_dist.begin(), ftr_dist.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) { return a.second < b.second; });
        int knum = std::min(int(frontier_ids.size()), ep_->refined_num_);
        for (int i = 0; i < knum; ++i)
        {
            int  fid = ftr_dist[i].first;
            auto tmp = ed_->points_[fid];
            tmp.z()  = 0.3;
            ed_->unrefined_points_.push_back(tmp);
            ed_->refined_ids_.push_back(fid);

            // [优化] 移除或大幅放宽半径限制，以解决狭长走廊的提前退出问题。
            // [优化] 只有当已经选择了足够的目标点 (>= 2) 时，才考虑退出。
            /* if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2)
                break;
            */
            // [优化] 仅在选择了足够的数量后，如果下一个前沿太远，才考虑 break。
            if (ed_->refined_ids_.size() >= 2 && (tmp - pos).norm() > ep_->refined_radius_)
            {
                // 避免在选择第一个点后立即退出
                if (i + 1 < knum)
                {  // 如果后面还有点，则退出当前循环
                    break;
                }
            }
        }

        // [优化] 如果优化后发现 refined_ids_ 数量太少，可以发警告。
        if (ed_->refined_ids_.size() < 2)
        {
            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                        "[planExploreMotion] Refined frontier count is low (%zu). Check ep_->refined_num_ and ep_->refined_radius_.", ed_->refined_ids_.size());
        }

        ed_->n_points_.clear();
        std::vector<std::vector<double>> n_yaws;
        frontier_finder_->getViewpointsInfo(pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

        // 强制视点 z=0.3
        for (auto& points : ed_->n_points_)
        {
            for (auto& p : points)
            {
                p.z() = 0.3;
            }
        }

        // P0-2：同步过滤不可达视点与对应 frontier id，避免 refineLocalTour 选到死区对侧。
        // 注意：getViewpointsInfo 可能找不到某些 frontier id，导致 n_points_.size() < refined_ids_.size()，
        // 因此循环边界用 n_points_.size()，避免越界访问。
        {
            size_t       write        = 0;
            const size_t n_ftr_groups = ed_->n_points_.size();
            for (size_t i = 0; i < n_ftr_groups; ++i)
            {
                auto&  pts = ed_->n_points_[i];
                auto&  ys  = n_yaws[i];
                size_t w   = 0;
                for (size_t j = 0; j < pts.size(); ++j)
                {
                    if (IsReachablePoint(pts[j]))
                    {
                        pts[w] = pts[j];
                        ys[w]  = ys[j];
                        ++w;
                    }
                }
                pts.resize(w);
                ys.resize(w);
                if (!pts.empty())
                {
                    // 避免 self-move-assignment 导致当前组被清空
                    if (i != write)
                    {
                        ed_->refined_ids_[write]      = ed_->refined_ids_[i];
                        ed_->unrefined_points_[write] = ed_->unrefined_points_[i];
                        ed_->n_points_[write]         = std::move(pts);
                        n_yaws[write]                 = std::move(ys);
                    }
                    ++write;
                }
            }
            ed_->refined_ids_.resize(write);
            ed_->unrefined_points_.resize(write);
            ed_->n_points_.resize(write);
            n_yaws.resize(write);
        }

        if (ed_->refined_ids_.empty())
        {
            applyHover();
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("exploration_manager"), *node_->get_clock(), 2000,
                                 "[planExploreMotion][d%d] all refined frontiers unreachable (deadzone?); hover", ep_->drone_id_);
        }
        else
        {
            ed_->refined_points_.clear();
            ed_->refined_views_.clear();
            std::vector<double> refined_yaws;
            refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
            for (auto& p : ed_->refined_points_)
            {
                p.z() = 0.3;
            }

            // [优化] 确保 refineLocalTour 返回了有效的点
            if (ed_->refined_points_.empty())
            {
                if (!ed_->refined_ids_.empty())
                {
                    applyFrontierTarget(ed_->refined_ids_[0]);
                    RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion] refineLocalTour empty; fallback frontier=%d",
                                ed_->refined_ids_[0]);
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"),
                                 "[planExploreMotion] refineLocalTour failed to return points. Using current position.");
                    applyHover();
                }
            }
            else
            {
                next_pos     = ed_->refined_points_[0];
                next_pos.z() = 0.3;
                next_yaw     = refined_yaws[0];
                for (int i = 0; i < ed_->refined_points_.size(); ++i)
                    goal_candidates.emplace_back(ed_->refined_points_[i], refined_yaws[i]);
            }

            for (int i = 0; i < ed_->refined_points_.size(); ++i)
            {
                Eigen::Vector3d view = ed_->refined_points_[i] + 2.0 * Eigen::Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
                ed_->refined_views_.push_back(view);
            }
            ed_->refined_views1_.clear();
            ed_->refined_views2_.clear();
            for (int i = 0; i < ed_->refined_points_.size(); ++i)
            {
                std::vector<Eigen::Vector3d> v1, v2;
                frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
                frontier_finder_->percep_utils_->getFOV(v1, v2);
                ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
                ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
            }
            double local_time = (node_->now() - t1).seconds();
        }
    }

    // 统一分流：把“同向尾随”与“近距同区拥挤”合并为一次近邻检测 + 一次仲裁 + 一次目标打分，避免双重覆盖改目标
    if (ep_->near_split_enable_ && !goal_candidates.empty())
    {
        static double          force_until_sec = -1.0;
        static double          last_split_sec  = -1.0;
        static int             forced_region   = -1;
        static Eigen::Vector3d forced_target(0, 0, 0);
        static double          forced_yaw = 0.0;
        /// 上次 UnifiedSplit 成功提交的目标分区，用于区域滞回，减轻相邻周期在 region 间来回跳
        static int    last_committed_split_region    = -1;
        static double last_committed_split_stamp_sec = -1.0;
        static double last_committed_split_score     = -1e9;
        static double lock_start_sec_                = -1.0;

        const double    now_sec = node_->now().seconds();
        Eigen::Vector2d self_dir(std::cos(yaw[0]), std::sin(yaw[0]));
        if (vel.head<2>().norm() > 0.2)
            self_dir = vel.head<2>().normalized();

        auto getRegion         = [&](const Eigen::Vector3d& p) -> int { return region_allocator_->ownerRegionOfPoint(p); };
        auto applyForcedRegion = [&](int target_region) -> bool {
            int    best_idx   = -1;
            double best_score = -1e9;
            for (int i = 0; i < goal_candidates.size(); ++i)
            {
                const auto& c  = goal_candidates[i].first;
                double      ds = (c - pos).head<2>().norm();
                if (ds < ep_->near_split_min_target_dist_)
                    continue;
                int reg = getRegion(c);
                if (reg != target_region)
                    continue;
                double score = -ep_->near_split_self_cost_weight_ * ds;
                if (score > best_score)
                {
                    best_score = score;
                    best_idx   = i;
                }
            }
            if (best_idx >= 0)
            {
                next_pos = goal_candidates[best_idx].first;
                next_yaw = goal_candidates[best_idx].second;
                return true;
            }
            return false;
        };

        // 锁定窗口内优先维持同一个“分流目标点”，避免每帧在同一区域内跳点引发轨迹抖动。
        // 若已接近该目标，则提前结束锁定，允许下一轮正常决策。
        if (forced_region >= 0 && force_until_sec > 0.0 && now_sec < force_until_sec)
        {
            const double lock_reach_dist = 1.0;
            double       nearest_live_d  = std::numeric_limits<double>::infinity();
            for (int i = 0; i < ed_->swarm_state_.size(); ++i)
            {
                if (i + 1 == ep_->drone_id_)
                    continue;
                const auto& st = ed_->swarm_state_[i];
                if (now_sec - st.stamp_ > ep_->near_split_stamp_timeout_)
                    continue;
                nearest_live_d = std::min(nearest_live_d, (st.pos_ - pos).head<2>().norm());
            }
            if (std::isfinite(nearest_live_d) && nearest_live_d < ep_->near_split_lock_extend_dist_ && lock_start_sec_ > 0.0
                && (now_sec - lock_start_sec_) < ep_->near_split_lock_max_total_sec_)
                force_until_sec = std::max(force_until_sec, now_sec + std::max(0.0, ep_->near_split_lock_extend_sec_));

            if ((pos - forced_target).head<2>().norm() > lock_reach_dist)
            {
                next_pos = forced_target;
                next_yaw = forced_yaw;
            }
            else
            {
                forced_region   = -1;
                force_until_sec = -1.0;
                lock_start_sec_ = -1.0;
            }
        }

        // 若锁定失效，近距离统一分流（同向尾随仅为其中一种加权更强的子情形）
        if (!(forced_region >= 0 && force_until_sec > 0.0 && now_sec < force_until_sec))
        {
            int                          near_id            = -1;
            double                       near_d             = 1e9;
            bool                         any_predicted_near = false;
            Eigen::Vector3d              near_pos(0, 0, 0);
            std::vector<int>             nearby_ids;
            std::vector<Eigen::Vector3d> nearby_positions;
            const double                 stamp_tol = ep_->near_split_stamp_timeout_;
            const double                 pre_t     = std::max(0.1, ep_->pre_split_lookahead_sec_);
            for (int i = 0; i < ed_->swarm_state_.size(); ++i)
            {
                if (i + 1 == ep_->drone_id_)
                    continue;
                const auto& st = ed_->swarm_state_[i];
                if (now_sec - st.stamp_ > stamp_tol)
                    continue;
                double d = (st.pos_ - pos).head<2>().norm();
                // 预测预分裂：外推两车 pre_t 秒后的位置，提前发现潜在的扎堆
                Eigen::Vector3d pred_self = pos + vel * pre_t;
                Eigen::Vector3d pred_peer = st.pos_ + st.vel_ * pre_t;
                double          pred_d    = (pred_peer - pred_self).head<2>().norm();
                if (d < ep_->near_split_dist_ || pred_d < ep_->pre_split_dist_)
                {
                    nearby_ids.push_back(i + 1);
                    nearby_positions.push_back(st.pos_);
                }
                if (pred_d < ep_->pre_split_dist_)
                    any_predicted_near = true;
                if (d < near_d)
                {
                    near_d   = d;
                    near_id  = i + 1;
                    near_pos = st.pos_;
                }
            }

            const bool cooldown_ready   = last_split_sec < 0.0 || (now_sec - last_split_sec) > ep_->near_split_cooldown_;
            const bool near_split_ready = near_id > 0 && cooldown_ready && (near_d < ep_->near_split_dist_ || any_predicted_near);
            if (near_split_ready)
            {
                int self_geo_reg  = getRegion(pos);
                int other_geo_reg = getRegion(near_pos);
                int self_des_reg  = getRegion(next_pos);

                bool            tailing      = false;
                bool            being_tailed = false;
                Eigen::Vector2d tail_dir(0.0, 0.0);
                bool            has_tail_dir         = false;
                const double    anti_tailgate_cos_th = std::cos(ep_->anti_tailgate_heading_deg_ * M_PI / 180.0);
                if (ep_->anti_tailgate_enable_)
                {
                    const double ahead_enter    = std::max(0.08, ep_->anti_tailgate_ahead_enter_m_);
                    const double ahead_exit     = std::min(-0.08, ep_->anti_tailgate_ahead_exit_m_);
                    static bool  s_tail_phase   = false;
                    static int   s_tail_partner = -1;

                    for (int id : nearby_ids)
                    {
                        const auto&     st = ed_->swarm_state_[id - 1];
                        Eigen::Vector2d cand_dir(std::cos(st.yaw_), std::sin(st.yaw_));
                        if (st.vel_.head<2>().norm() > 0.2)
                            cand_dir = st.vel_.head<2>().normalized();
                        const double          heading_sim   = self_dir.dot(cand_dir);
                        const Eigen::Vector2d rel           = (st.pos_ - pos).head<2>();
                        const double          fwd           = rel.dot(self_dir);
                        const bool            use_exit_band = s_tail_phase && id == s_tail_partner;
                        const bool            ahead_ok      = use_exit_band ? (fwd > ahead_exit) : (fwd > ahead_enter);
                        if (heading_sim > anti_tailgate_cos_th && ahead_ok)
                        {
                            tailing  = true;
                            near_id  = id;
                            near_pos = st.pos_;
                            near_d   = rel.norm();
                            break;
                        }
                    }
                    if (tailing)
                    {
                        s_tail_phase   = true;
                        s_tail_partner = near_id;
                    }
                    else if (s_tail_phase && s_tail_partner > 0)
                    {
                        const int pid = s_tail_partner;
                        if (pid <= 0 || pid > static_cast<int>(ed_->swarm_state_.size())
                            || now_sec - ed_->swarm_state_[pid - 1].stamp_ > ep_->near_split_stamp_timeout_)
                        {
                            s_tail_phase   = false;
                            s_tail_partner = -1;
                        }
                        else
                        {
                            const auto&     st = ed_->swarm_state_[pid - 1];
                            Eigen::Vector2d cand_dir(std::cos(st.yaw_), std::sin(st.yaw_));
                            if (st.vel_.head<2>().norm() > 0.2)
                                cand_dir = st.vel_.head<2>().normalized();
                            const Eigen::Vector2d rel         = (st.pos_ - pos).head<2>();
                            const double          fwd         = rel.dot(self_dir);
                            const double          heading_sim = self_dir.dot(cand_dir);
                            if (heading_sim > anti_tailgate_cos_th && fwd > ahead_exit)
                            {
                                tailing  = true;
                                near_id  = pid;
                                near_pos = st.pos_;
                                near_d   = rel.norm();
                            }
                            else if (fwd <= ahead_exit)
                            {
                                s_tail_phase   = false;
                                s_tail_partner = -1;
                            }
                        }
                    }

                    // ★ 方向一：头车提速外扩 — 检测是否有人在后面同向跟着
                    for (int id : nearby_ids)
                    {
                        const auto&     st = ed_->swarm_state_[id - 1];
                        Eigen::Vector2d cand_dir(std::cos(st.yaw_), std::sin(st.yaw_));
                        if (st.vel_.head<2>().norm() > 0.2)
                            cand_dir = st.vel_.head<2>().normalized();
                        const double          heading_sim = self_dir.dot(cand_dir);
                        const Eigen::Vector2d rel         = (st.pos_ - pos).head<2>();
                        const double          fwd         = rel.dot(self_dir);
                        if (heading_sim > anti_tailgate_cos_th && fwd < -std::max(0.08, ep_->anti_tailgate_ahead_enter_m_))
                        {
                            being_tailed = true;
                            tail_dir     = cand_dir;
                            if (tail_dir.norm() > 1e-6)
                            {
                                tail_dir.normalize();
                                has_tail_dir = true;
                            }
                            break;
                        }
                    }
                }

                const int        total_regions = std::max(1, region_allocator_->layout().total_regions);
                std::vector<int> occupancy_count(static_cast<size_t>(total_regions), 0);
                occupancy_count[std::max(0, std::min(self_geo_reg, total_regions - 1))] += 1;
                occupancy_count[std::max(0, std::min(other_geo_reg, total_regions - 1))] += 1;
                for (int i = 0; i < ed_->swarm_state_.size(); ++i)
                {
                    if (i + 1 == ep_->drone_id_ || i + 1 == near_id)
                        continue;
                    const auto& st = ed_->swarm_state_[i];
                    if (now_sec - st.stamp_ > ep_->near_split_stamp_timeout_)
                        continue;
                    int reg = getRegion(st.pos_);
                    if (reg >= 0 && reg < total_regions)
                        occupancy_count[static_cast<size_t>(reg)] += 1;
                }

                // 方向二：预测各 peer 即将进入的区域（基于速度外推）
                std::vector<int> incoming_count(static_cast<size_t>(total_regions), 0);
                for (int i = 0; i < ed_->swarm_state_.size(); ++i)
                {
                    if (i + 1 == ep_->drone_id_)
                        continue;
                    const auto& st = ed_->swarm_state_[i];
                    if (now_sec - st.stamp_ > stamp_tol)
                        continue;
                    if (st.vel_.head<2>().norm() < 0.1)
                        continue;
                    Eigen::Vector3d pred_peer = st.pos_ + st.vel_ * pre_t;
                    int             pred_reg  = getRegion(pred_peer);
                    int             cur_reg   = getRegion(st.pos_);
                    if (pred_reg >= 0 && pred_reg < total_regions && pred_reg != cur_reg)
                        incoming_count[static_cast<size_t>(pred_reg)]++;
                }

                int             best_idx   = -1;
                int             best_reg   = -1;
                double          best_score = -1e9;
                double          best_ds    = 0.0;
                double          best_dl    = 0.0;
                double          best_bonus = 0.0;
                Eigen::Vector2d near_move_dir(0.0, 0.0);
                bool            has_near_move_dir = false;
                if (tailing && near_id > 0 && near_id <= static_cast<int>(ed_->swarm_state_.size()))
                {
                    const auto& nst = ed_->swarm_state_[near_id - 1];
                    near_move_dir   = Eigen::Vector2d(std::cos(nst.yaw_), std::sin(nst.yaw_));
                    if (nst.vel_.head<2>().norm() > 0.2)
                        near_move_dir = nst.vel_.head<2>().normalized();
                    has_near_move_dir = near_move_dir.norm() > 1e-6;
                    if (has_near_move_dir)
                        near_move_dir.normalize();
                }
                // 尾随子模式略提高最小迁移距离，减少“刚分流又贴回前车”
                double min_target_dist = ep_->near_split_min_target_dist_ * (tailing ? 1.15 : 1.0);

                std::vector<int> region_goal_count(static_cast<size_t>(total_regions), 0);
                for (int i = 0; i < goal_candidates.size(); ++i)
                {
                    int reg = getRegion(goal_candidates[i].first);
                    if (reg >= 0 && reg < total_regions)
                        region_goal_count[static_cast<size_t>(reg)] += 1;
                }
                int max_goal_count = 1;
                for (int reg = 0; reg < total_regions; ++reg)
                    max_goal_count = std::max(max_goal_count, region_goal_count[static_cast<size_t>(reg)]);

                auto scoreCandidates = [&](double min_ds, bool ignore_min) {
                    best_idx   = -1;
                    best_reg   = -1;
                    best_score = -1e9;
                    best_ds    = 0.0;
                    best_dl    = 0.0;
                    best_bonus = 0.0;
                    for (int i = 0; i < goal_candidates.size(); ++i)
                    {
                        const auto& c  = goal_candidates[i].first;
                        double      ds = (c - pos).head<2>().norm();
                        if (!ignore_min && ds < min_ds)
                            continue;
                        double dl     = (c - near_pos).head<2>().norm();
                        double min_dn = 1e9;
                        for (const auto& np : nearby_positions)
                            min_dn = std::min(min_dn, (c - np).head<2>().norm());
                        if (!std::isfinite(min_dn))
                            min_dn = dl;
                        if (min_dn < ep_->near_split_min_peer_target_dist_)
                            continue;
                        int    reg   = getRegion(c);
                        double score = ep_->near_split_sep_weight_ * dl - ep_->near_split_self_cost_weight_ * ds;
                        score += 0.6 * ep_->near_split_sep_weight_ * min_dn;
                        if (tailing)
                        {
                            score += ep_->anti_tailgate_sep_weight_ * dl - ep_->anti_tailgate_self_cost_weight_ * ds;
                            // Encourage heading divergence from the leading drone in tailing mode.
                            if (has_near_move_dir)
                            {
                                Eigen::Vector2d self_to_c = (c - pos).head<2>();
                                if (self_to_c.norm() > 1e-6)
                                {
                                    self_to_c.normalize();
                                    const double cos_align = self_to_c.dot(near_move_dir);  // [-1, 1]
                                    const double dir_div   = 0.5 * (1.0 - cos_align);       // 0: same dir, 1: opposite
                                    score += ep_->anti_tailgate_sep_weight_ * 2.0 * dir_div;
                                }
                            }
                        }
                        // ★ 方向一：头车逃离奖励 — 有人尾随我时鼓励转向不同方向
                        if (being_tailed && has_tail_dir)
                        {
                            Eigen::Vector2d self_to_c = (c - pos).head<2>();
                            if (self_to_c.norm() > 1e-6)
                            {
                                self_to_c.normalize();
                                const double cos_align = self_to_c.dot(tail_dir);
                                const double dir_div   = 0.5 * (1.0 - cos_align);
                                score += ep_->anti_tailgate_sep_weight_ * 4.0 * dir_div;
                            }
                        }
                        double uncovered_bonus_term = 0.0;
                        if (reg >= 0 && reg < total_regions)
                        {
                            const double decay_dist     = std::max(1e-3, ep_->near_split_uncovered_bonus_decay_dist_);
                            const double base_scale     = std::exp(-ds / decay_dist);
                            const double bonus_scale    = std::max(std::max(0.0, std::min(1.0, ep_->near_split_uncovered_bonus_min_scale_)), base_scale);
                            const double density_ratio  = static_cast<double>(region_goal_count[static_cast<size_t>(reg)]) / std::max(1, max_goal_count);
                            const double density_factor = 1.0 + ep_->near_split_uncovered_bonus_density_gain_ * density_ratio;
                            const int    occ_n          = occupancy_count[static_cast<size_t>(reg)];
                            const double occ_factor     = 1.0 / (1.0 + ep_->near_split_uncovered_bonus_occ_penalty_ * std::max(0, occ_n));
                            uncovered_bonus_term        = ep_->near_split_uncovered_bonus_ * bonus_scale * density_factor * occ_factor;
                            score += uncovered_bonus_term;
                        }
                        // 方向二：peer 目标区域惩罚 — 多车目标扎堆同区时压低分数
                        if (!incoming_count.empty() && reg >= 0 && static_cast<size_t>(reg) < incoming_count.size()
                            && incoming_count[static_cast<size_t>(reg)] > 0)
                        {
                            score -= ep_->near_split_incoming_region_penalty_ * static_cast<double>(incoming_count[static_cast<size_t>(reg)]);
                        }
                        if (reg == other_geo_reg || reg == self_des_reg)
                            score -= ep_->near_split_same_region_penalty_;
                        if (ep_->near_split_region_hysteresis_sec_ > 1e-6 && last_committed_split_region >= 0 && reg >= 0 && reg != last_committed_split_region
                            && (now_sec - last_committed_split_stamp_sec) < ep_->near_split_region_hysteresis_sec_)
                        {
                            score -= ep_->near_split_region_switch_penalty_;
                        }
                        // RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[UnifiedSplitScore] drone=%d cand=%d reg=%d ds=%.2f dl=%.2f bonus=%.3f
                        // total=%.3f",
                        //          ep_->drone_id_, i, reg, ds, dl, uncovered_bonus_term, score);
                        if (score > best_score)
                        {
                            best_score = score;
                            best_idx   = i;
                            best_reg   = reg;
                            best_ds    = ds;
                            best_dl    = dl;
                            best_bonus = uncovered_bonus_term;
                        }
                    }
                };

                scoreCandidates(min_target_dist, false);
                if (best_idx < 0)
                    scoreCandidates(std::max(1.5, min_target_dist * 0.5), false);
                if (best_idx < 0)
                    scoreCandidates(0.0, true);

                if (best_idx >= 0)
                {
                    bool keep_last_region_target = false;
                    if (ep_->near_split_switch_min_interval_sec_ > 1e-6 && last_committed_split_region >= 0 && best_reg >= 0
                        && best_reg != last_committed_split_region && (now_sec - last_committed_split_stamp_sec) < ep_->near_split_switch_min_interval_sec_
                        && std::isfinite(last_committed_split_score) && last_committed_split_score > -1e8)
                    {
                        const double improve_ratio = (best_score - last_committed_split_score) / std::max(1e-6, std::fabs(last_committed_split_score));
                        if (improve_ratio < ep_->near_split_switch_min_improve_ratio_ && applyForcedRegion(last_committed_split_region))
                        {
                            forced_region   = last_committed_split_region;
                            forced_target   = next_pos;
                            forced_yaw      = next_yaw;
                            force_until_sec = now_sec + std::max(0.0, ep_->near_split_lock_sec_);
                            lock_start_sec_ = now_sec;
                            last_split_sec  = now_sec;
                            // 保持原提交分区与分数，不更新为新候选，避免“轻微收益”触发抖动换区。
                            best_reg                = forced_region;
                            best_score              = last_committed_split_score;
                            keep_last_region_target = true;
                        }
                    }
                    if (!keep_last_region_target)
                    {
                        next_pos                       = goal_candidates[best_idx].first;
                        next_yaw                       = goal_candidates[best_idx].second;
                        forced_region                  = best_reg;
                        forced_target                  = next_pos;
                        forced_yaw                     = next_yaw;
                        force_until_sec                = now_sec + std::max(0.0, ep_->near_split_lock_sec_);
                        lock_start_sec_                = now_sec;
                        last_split_sec                 = now_sec;
                        last_committed_split_region    = best_reg;
                        last_committed_split_stamp_sec = now_sec;
                        last_committed_split_score     = best_score;
                    }
                    // RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[UnifiedSplit] drone=%d near=%d d=%.2f tailing=%d | cand=%d reg=%d ds=%.2f
                    // dl=%.2f bonus=%.3f score=%.3f -> target=[%.2f, %.2f]",
                    //          ep_->drone_id_, near_id, near_d, static_cast<int>(tailing), best_idx, forced_region, best_ds, best_dl, best_bonus,
                    //          best_score, next_pos.x(), next_pos.y());
                }
                else
                {
                    bool            picked_unknown_patch = false;
                    int             patch_reg            = -1;
                    Eigen::Vector3d patch_target(0, 0, pos.z());
                    double          patch_score = -1e9;

                    auto tryPickUnknownPatchInReg = [&](int reg) {
                        double min_x, max_x, min_y, max_y;
                        region_allocator_->getRegionBounds(reg, min_x, max_x, min_y, max_y);

                        const double coarse_step = 1.2;  // small patch center sampling
                        const double fine_step   = 0.6;  // patch neighborhood probe
                        for (double x = min_x + 0.5; x <= max_x - 0.5; x += coarse_step)
                        {
                            for (double y = min_y + 0.5; y <= max_y - 0.5; y += coarse_step)
                            {
                                Eigen::Vector3d c(x, y, pos.z());
                                if (edt_environment_->sdf_map_->getInflateOccupancy(c) == 1)
                                    continue;

                                int unknown_cnt = 0, free_cnt = 0;
                                for (double dx = -fine_step; dx <= fine_step + 1e-6; dx += fine_step)
                                {
                                    for (double dy = -fine_step; dy <= fine_step + 1e-6; dy += fine_step)
                                    {
                                        Eigen::Vector3d q(x + dx, y + dy, pos.z());
                                        int             occ = edt_environment_->sdf_map_->getOccupancy(q);
                                        if (occ == SDFMap::UNKNOWN)
                                            unknown_cnt++;
                                        else if (occ == SDFMap::FREE)
                                            free_cnt++;
                                    }
                                }

                                // Need nearby unknowns but not a fully blocked patch.
                                if (unknown_cnt < 3 || free_cnt < 2)
                                    continue;

                                const double ds      = (c - pos).head<2>().norm();
                                const double dl      = (c - near_pos).head<2>().norm();
                                double       dir_div = 0.0;
                                if (has_near_move_dir)
                                {
                                    Eigen::Vector2d self_to_c = (c - pos).head<2>();
                                    if (self_to_c.norm() > 1e-6)
                                    {
                                        self_to_c.normalize();
                                        const double cos_align = self_to_c.dot(near_move_dir);  // [-1, 1]
                                        dir_div                = 0.5 * (1.0 - cos_align);       // 0~1
                                    }
                                }
                                const double s = 2.0 * unknown_cnt + 0.8 * dl - 0.25 * ds + 2.0 * dir_div;
                                if (s > patch_score)
                                {
                                    patch_score          = s;
                                    patch_target         = c;
                                    patch_reg            = reg;
                                    picked_unknown_patch = true;
                                }
                            }
                        }
                    };

                    // Tailing fallback: if no valid viewpoint, bias to a small unknown patch.
                    if (tailing)
                    {
                        for (int reg = 0; reg < total_regions; ++reg)
                        {
                            if (occupancy_count[static_cast<size_t>(reg)] > 0)
                                continue;
                            tryPickUnknownPatchInReg(reg);
                        }
                        // If empty regions have no good unknown patch, allow any region.
                        if (!picked_unknown_patch)
                        {
                            for (int reg = 0; reg < total_regions; ++reg)
                                tryPickUnknownPatchInReg(reg);
                        }
                    }
                    // Non-tailing: same unknown-patch probe reduces "everyone to empty-region centroid" clustering.
                    if (!tailing && !picked_unknown_patch)
                    {
                        for (int reg = 0; reg < total_regions; ++reg)
                        {
                            if (occupancy_count[static_cast<size_t>(reg)] > 0)
                                continue;
                            tryPickUnknownPatchInReg(reg);
                        }
                        if (!picked_unknown_patch)
                        {
                            for (int reg = 0; reg < total_regions; ++reg)
                                tryPickUnknownPatchInReg(reg);
                        }
                    }

                    if (picked_unknown_patch)
                    {
                        next_pos                       = patch_target;
                        next_yaw                       = std::atan2(next_pos.y() - pos.y(), next_pos.x() - pos.x());
                        forced_region                  = patch_reg;
                        forced_target                  = next_pos;
                        forced_yaw                     = next_yaw;
                        force_until_sec                = now_sec + std::max(0.0, ep_->near_split_lock_sec_);
                        lock_start_sec_                = now_sec;
                        last_split_sec                 = now_sec;
                        last_committed_split_region    = patch_reg;
                        last_committed_split_stamp_sec = now_sec;
                    }
                    else
                    {
                        int    fallback_reg            = -1;
                        double best_fd                 = -1e9;
                        auto   pickEmptyRegionCentroid = [&](bool require_peer_margin) {
                            fallback_reg = -1;
                            best_fd      = -1e9;
                            for (int reg = 0; reg < total_regions; ++reg)
                            {
                                if (occupancy_count[static_cast<size_t>(reg)] > 0)
                                    continue;
                                double min_x, max_x, min_y, max_y;
                                region_allocator_->getRegionBounds(reg, min_x, max_x, min_y, max_y);
                                Eigen::Vector3d c((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, pos.z());
                                const double    ds     = (c - pos).head<2>().norm();
                                const double    dl     = (c - near_pos).head<2>().norm();
                                double          min_dn = 1e9;
                                for (const auto& np : nearby_positions)
                                    min_dn = std::min(min_dn, (c - np).head<2>().norm());
                                if (!std::isfinite(min_dn))
                                    min_dn = dl;
                                if (require_peer_margin && min_dn < ep_->near_split_min_peer_target_dist_)
                                    continue;
                                // Match UnifiedSplit candidate spirit: separate from paired neighbor, not only from self.
                                double fd = ep_->near_split_sep_weight_ * dl - ep_->near_split_self_cost_weight_ * ds;
                                fd += 0.6 * ep_->near_split_sep_weight_ * min_dn;
                                if (reg == other_geo_reg)
                                    fd -= 0.5 * ep_->near_split_same_region_penalty_;
                                if (fd > best_fd)
                                {
                                    best_fd      = fd;
                                    fallback_reg = reg;
                                }
                            }
                        };
                        pickEmptyRegionCentroid(true);
                        if (fallback_reg < 0)
                            pickEmptyRegionCentroid(false);
                        if (fallback_reg < 0)
                        {
                            // Legacy: farthest empty centroid from self (old behavior).
                            best_fd = -1.0;
                            for (int reg = 0; reg < total_regions; ++reg)
                            {
                                if (occupancy_count[static_cast<size_t>(reg)] > 0)
                                    continue;
                                double min_x, max_x, min_y, max_y;
                                region_allocator_->getRegionBounds(reg, min_x, max_x, min_y, max_y);
                                Eigen::Vector3d c((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, pos.z());
                                const double    d = (c - pos).head<2>().norm();
                                if (d > best_fd)
                                {
                                    best_fd      = d;
                                    fallback_reg = reg;
                                }
                            }
                        }
                        if (fallback_reg >= 0)
                        {
                            double min_x, max_x, min_y, max_y;
                            region_allocator_->getRegionBounds(fallback_reg, min_x, max_x, min_y, max_y);
                            next_pos                       = Eigen::Vector3d((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, pos.z());
                            next_yaw                       = std::atan2(next_pos.y() - pos.y(), next_pos.x() - pos.x());
                            forced_region                  = fallback_reg;
                            forced_target                  = next_pos;
                            forced_yaw                     = next_yaw;
                            force_until_sec                = now_sec + std::max(0.0, ep_->near_split_lock_sec_);
                            lock_start_sec_                = now_sec;
                            last_split_sec                 = now_sec;
                            last_committed_split_region    = fallback_reg;
                            last_committed_split_stamp_sec = now_sec;
                            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                                        "[UnifiedSplit] drone=%d near=%d d=%.2f tailing=%d | no valid candidate, fallback empty region=%d center=[%.2f, %.2f]",
                                        ep_->drone_id_, near_id, near_d, static_cast<int>(tailing), fallback_reg, next_pos.x(), next_pos.y());
                        }
                    }
                }
            }
        }
    }

    {
        const double max_step_dist = GetMaxLocalGoalDist(*ep_);
        const double cur_goal_dist = (next_pos - pos).head<2>().norm();
        if (cur_goal_dist > max_step_dist && !ed_->points_.empty())
        {
            int    best_fid = -1;
            double best_d   = max_step_dist;
            for (int i = 0; i < static_cast<int>(ed_->points_.size()); ++i)
            {
                const double d = (ed_->points_[i].head<2>() - pos.head<2>()).norm();
                if (d < 0.1 || d >= best_d)
                    continue;
                // P0-1：截断候选必须通过与最终目标相同的准入口径。否则墙后/死区对侧的
                // 不可达视点会被当成目标：admitTarget 每周期拒绝后只等地图更新，
                // 车不动→地图不变→再选同一点，直至被 L2 无进展判据甩回起点。
                if (!admitTarget(pos, ed_->points_[i]))
                    continue;
                best_d   = d;
                best_fid = i;
            }
            if (best_fid >= 0)
            {
                // RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion][d%d] clamp far goal %.2fm -> nearest frontier within %.1fm",
                //             ep_->drone_id_, cur_goal_dist, max_step_dist);
                next_pos     = ed_->points_[best_fid];
                next_pos.z() = 0.3;
                next_yaw     = ed_->yaws_[best_fid];
            }
            else
            {
                // 局部范围内没有可准入前沿：保留原目标，交给 planTrajToView 按 local_traj_radius 截弧长。
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger("exploration_manager"), *node_->get_clock(), 5000,
                                     "[planExploreMotion][d%d] clamp far goal %.2fm: no admissible frontier within %.1fm, keep original target", ep_->drone_id_,
                                     cur_goal_dist, max_step_dist);
            }
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[planExploreMotion] 开始位置: [%.2f, %.2f, %.2f], 航向: %.2f", pos.x(), pos.y(), pos.z(), yaw[0]);
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[planExploreMotion] 下一个视点: [%.2f, %.2f, %.2f], 航向: %.2f", next_pos.x(), next_pos.y(),
                next_pos.z(), next_yaw);

    const double goal_dist_xy = (next_pos.head<2>() - pos.head<2>()).norm();
    ed_->next_pos_            = next_pos;
    ed_->next_yaw_            = next_yaw;
    if (goal_dist_xy > ep_->viewpoint_arrived_dist_m_)
        ed_->last_next_pos_ = ed_->next_pos_;
    else
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                    "[planExploreMotion] Next Viewpoint very close (dist_xy=%.3fm <= %.2fm); keeping target, last_next_pos unchanged.", goal_dist_xy,
                    ep_->viewpoint_arrived_dist_m_);

    if (goal_dist_xy < ep_->degenerate_goal_dist_m_)
    {
        std::vector<int> active_grids;
        if (hgrid_)
            hgrid_->getActiveGrids(active_grids);

        if (!active_grids.empty())
        {
            // ---- Have active grids → never return NO_GRID, keep moving ----

            // Try 1: Recover stale grid assignment (find a frontier in an unfinished grid)
            if (recoverStaleGridAssignment(pos, grid_ids, frontier_ids) && !frontier_ids.empty())
            {
                applyFrontierTarget(frontier_ids[0]);
                ed_->next_pos_ = next_pos;
                ed_->next_yaw_ = next_yaw;
                RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion][d%d] degenerate goal; recoverStale => frontier %d, dist=%.2fm",
                            ep_->drone_id_, frontier_ids[0], (next_pos.head<2>() - pos.head<2>()).norm());
            }
            else if (hgrid_)
            {
                // ★ 修复：集群检测 + x/y 偏置散射 + 速度预测式排斥
                std::unordered_set<int>                                  peer_near_grids;
                bool                                                     cluster_scatter  = false;
                double                                                   cluster_center_x = pos.x();
                double                                                   cluster_center_y = pos.y();
                double                                                   x_bias           = 0.0;
                double                                                   y_bias           = 0.0;
                std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> deg_nearby_peers;
                {
                    const double                        dn_sec  = node_->now().seconds();
                    double                              sum_x   = pos.x();
                    double                              sum_y   = pos.y();
                    int                                 n_total = 1;
                    int                                 n_peers = 0;
                    std::vector<std::pair<double, int>> rank_data_deg;
                    rank_data_deg.emplace_back(pos.x(), ep_->drone_id_);
                    for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
                    {
                        if (i + 1 == ep_->drone_id_)
                            continue;
                        const auto& st = ed_->swarm_state_[i];
                        if (dn_sec - st.stamp_ > 2.0)
                            continue;
                        const double d = (st.pos_.head<2>() - pos.head<2>()).norm();
                        if (d < 30.0)
                            deg_nearby_peers.emplace_back(st.pos_, st.vel_);
                        if (d < 15.0)
                        {
                            n_peers++;
                            sum_x += st.pos_.x();
                            sum_y += st.pos_.y();
                            rank_data_deg.emplace_back(st.pos_.x(), i + 1);
                            n_total++;
                            // 反应式：peer 附近的 active grid 标记为不优先
                            for (int gid : active_grids)
                            {
                                const double gd = (hgrid_->getCenter(gid).head<2>() - st.pos_.head<2>()).norm();
                                if (gd < 8.0)
                                    peer_near_grids.insert(gid);
                            }
                        }
                    }
                    if (n_peers >= 1)
                    {
                        cluster_scatter  = true;
                        cluster_center_x = sum_x / n_total;
                        cluster_center_y = sum_y / n_total;
                        // 排名法 x_bias
                        {
                            std::sort(rank_data_deg.begin(), rank_data_deg.end());
                            int my_rank = -1;
                            for (int k = 0; k < static_cast<int>(rank_data_deg.size()); ++k)
                            {
                                if (rank_data_deg[k].second == ep_->drone_id_)
                                {
                                    my_rank = k;
                                    break;
                                }
                            }
                            const int N = rank_data_deg.size();
                            x_bias      = (N > 1) ? -1.0 + 2.0 * my_rank / (N - 1) : 0.0;
                        }
                        const double spread = std::max(5.0, 15.0 * 0.5);
                        y_bias              = std::max(-1.0, std::min(1.0, (pos.y() - cluster_center_y) / spread));
                        if (std::abs(x_bias) >= 0.3)
                            y_bias *= 0.3;
                        {
                            constexpr double kMinBias = 0.3;
                            if (std::abs(y_bias) < kMinBias)
                                y_bias = 0.0;
                        }
                        if (n_peers == 1)
                        {
                            x_bias *= 0.5;
                            y_bias *= 0.5;
                        }
                    }
                }

                // Try 2: Nearest REACHABLE active grid center at least degenerate_goal_dist_m away
                int              best_gid    = -1;
                double           best_score  = -std::numeric_limits<double>::infinity();
                constexpr double kPredHorDeg = 5.0;
                for (int gid : active_grids)
                {
                    const Eigen::Vector3d gc = hgrid_->getCenter(gid);
                    const double          d  = (gc.head<2>() - pos.head<2>()).norm();
                    if (d < ep_->degenerate_goal_dist_m_)
                        continue;
                    if (!IsReachablePoint(gc))
                        continue;  // P0-2：死区对侧/不可达网格中心剔除
                    double score = -d;
                    // 反应式：peer 附近 grid 扣分
                    if (peer_near_grids.count(gid) != 0u)
                        score -= 15.0;
                    // ③ soft anti-clustering: grids inside a peer's fresh explored
                    // box are deprioritized (tie-break, never a hard veto).
                    score -= peerCoveragePenalty(gc);
                    // x 偏置横向散射（集群模式有效）
                    if (cluster_scatter)
                    {
                        const double lateral = x_bias * (gc.x() - cluster_center_x) * 1.0;
                        const double vert    = y_bias * (gc.y() - cluster_center_y) * 0.5;
                        score += lateral + vert;
                    }
                    // 速度预测式排斥：peer 正在飞向这个 grid 区域
                    for (const auto& [pp, pv] : deg_nearby_peers)
                    {
                        if (pv.head<2>().norm() < 0.3)
                            continue;
                        const Eigen::Vector2d pred    = pp.head<2>() + pv.head<2>() * kPredHorDeg;
                        const Eigen::Vector2d gc2     = gc.head<2>();
                        const double          pd_pred = (gc2 - pred).norm();
                        if (pd_pred < 12.0)
                            score -= 8.0;
                        const double g_to_p_dist = (gc2 - pp.head<2>()).norm();
                        if (g_to_p_dist > 1.0)
                        {
                            const Eigen::Vector2d g_to_p = (gc2 - pp.head<2>()) / g_to_p_dist;
                            const double          align  = g_to_p.dot(pv.head<2>().normalized());
                            if (align > 0.3)
                            {
                                const double dist_factor = std::max(0.0, 1.0 - pd_pred / 20.0);
                                score -= 4.0 * align * dist_factor;
                            }
                        }
                    }
                    if (score > best_score)
                    {
                        best_score = score;
                        best_gid   = gid;
                    }
                }
                if (best_gid >= 0)
                {
                    applyGridCenterTarget(hgrid_->getCenter(best_gid));
                    ed_->next_pos_ = next_pos;
                    ed_->next_yaw_ = next_yaw;
                    RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                                "[planExploreMotion][d%d] degenerate goal; relocate to grid %d center (dist=%.2fm, grids=%zu)", ep_->drone_id_, best_gid,
                                (hgrid_->getCenter(best_gid).head<2>() - pos.head<2>()).norm(), active_grids.size());
                }
                else
                {
                    // 无可达网格中心（可能被死区包围）：悬停等待地图更新，绝不返回 NO_GRID（会触发返航）
                    applyHover();
                    ed_->next_pos_ = next_pos;
                    ed_->next_yaw_ = next_yaw;
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("exploration_manager"), *node_->get_clock(), 2000,
                                         "[planExploreMotion][d%d] degenerate goal; no reachable active grid center (deadzone?); hover (grids=%zu)",
                                         ep_->drone_id_, active_grids.size());
                }
            }
        }
        else
        {
            // No active grids → genuinely no work left
            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion][d%d] degenerate goal, return NO_GRID (no active grids)",
                        ep_->drone_id_);
            return NO_GRID;
        }
    }

    if (!admitTarget(pos, ed_->next_pos_))
    {
        std::vector<int> active_grids;
        if (hgrid_)
            hgrid_->getActiveGrids(active_grids);
        if (!active_grids.empty())
        {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("exploration_manager"), *node_->get_clock(), 2000,
                                 "[planExploreMotion][d%d] no admissible target [%.2f, %.2f]; wait for map/frontier update (active_grids=%zu)", ep_->drone_id_,
                                 ed_->next_pos_.x(), ed_->next_pos_.y(), active_grids.size());
            // P0-2：准入失败同样要落隔离表，否则下一周期仍会选出同一目标而无限等待；
            // 已在隔离期内不重复计数，避免指数退避无界增长。
            if (!isTargetQuarantined(ed_->next_pos_, pos))
                recordTargetFailure(ed_->next_pos_);
            return HandleStallBeforeWait(pos, vel, acc, yaw);
        }
        return NO_GRID;
    }

    if (planTrajToView(pos, vel, acc, yaw, ed_->next_pos_, ed_->next_yaw_) == FAIL)
    {
        const Vector3d failed_target = ed_->next_pos_;
        recordTargetFailure(failed_target);
        const double now_sec_pf = node_->now().seconds();
        consecutive_plan_fail_++;
        const bool   fallback_enabled        = ep_->plan_fail_fallback_trigger_count_ > 0;
        const bool   fallback_ready          = fallback_enabled && consecutive_plan_fail_ >= ep_->plan_fail_fallback_trigger_count_;
        const bool   fallback_circuit_closed = now_sec_pf >= fallback_circuit_open_until_sec_;
        const double fallback_dist           = (ed_->last_next_pos_.head<2>() - pos.head<2>()).norm();
        const bool   fallback_valid          = fallback_dist > std::max(ep_->viewpoint_arrived_dist_m_, ep_->plan_fail_fallback_min_dist_);
        if (fallback_ready && fallback_circuit_closed)
        {
            Vector3d alt_pos;
            double   alt_yaw = 0.0;
            if (tryPlanFailRetarget(pos, grid_ids, frontier_ids, alt_pos, alt_yaw) && admitTarget(pos, alt_pos)
                && planTrajToView(pos, vel, acc, yaw, alt_pos, alt_yaw) == SUCCEED)
            {
                ed_->next_pos_         = alt_pos;
                ed_->next_yaw_         = alt_yaw;
                ed_->last_next_pos_    = alt_pos;
                consecutive_plan_fail_ = 0;
                MarkNormalPlanSuccess();
                return SUCCEED;
            }
            if (fallback_valid && admitTarget(pos, ed_->last_next_pos_))
            {
                const double fb_yaw = std::atan2(ed_->last_next_pos_.y() - pos.y(), ed_->last_next_pos_.x() - pos.x());
                RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion][d%d] Plan fail x%d. Retry with last feasible target (dist=%.2f).",
                            ep_->drone_id_, consecutive_plan_fail_, fallback_dist);
                if (planTrajToView(pos, vel, acc, yaw, ed_->last_next_pos_, fb_yaw) == SUCCEED)
                {
                    ed_->next_pos_         = ed_->last_next_pos_;
                    ed_->next_yaw_         = fb_yaw;
                    consecutive_plan_fail_ = 0;
                    MarkNormalPlanSuccess();
                    return SUCCEED;
                }
            }
        }
        if (consecutive_plan_fail_ >= ep_->plan_fail_circuit_threshold_)
        {
            fallback_circuit_open_until_sec_ = now_sec_pf + ep_->plan_fail_circuit_sec_;
            ed_->last_next_pos_              = pos;
            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                        "[planExploreMotion][d%d] Plan fail x%d, open fallback circuit %.1fs to avoid unreachable retry storm.", ep_->drone_id_,
                        consecutive_plan_fail_, ep_->plan_fail_circuit_sec_);
        }
        return HandleStallBeforeWait(pos, vel, acc, yaw);
    }
    clearTargetFailure(ed_->next_pos_);
    consecutive_plan_fail_ = 0;
    MarkNormalPlanSuccess();

    // RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "Total time: %lf", total);
    // ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

    return SUCCEED;
}

/**
 * @brief 规划一条当前点到下一视点的飞行路径（位置、朝向）
 */
bool FastExplorationManager::planSnnPolylineToGoal(const Vector3d& pos_plan, const Vector3d& modified_next_pos, const Vector3d& pos, double goal_dist_xy)
{
    Eigen::Vector3i start, target;
    sdf_map_->posToGridIndex(pos_plan, start);
    sdf_map_->posToGridIndex(modified_next_pos, target);

    // 更新2D投影地图
    UpdateSnnGridFromSdfImpl(this, pos_plan);

    auto t_start1  = node_->now();
    auto path_grid = snn_->plan(start, target);
    if (path_grid.empty() && sdf_map_ && sdf_map_->agentFootprintClearEnabled())
    {
        ClearDynamicAgentFootprintsForPlanning(this, pos_plan, sdf_map_->stuckClearRadiusScale());
        const auto retry_grid = sdf_map_->export2DGrid(pos_plan, -1, snn_deadzone_margin_cells_);

        //*调用SNN路径规划的地方
        if (!retry_grid.empty())
            snn_->setGrid(retry_grid);
        path_grid = snn_->plan(start, target);
        if (!path_grid.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[SNN Planner][d%d] recovered path after stuck footprint clear (scale=%.2f)", ep_->drone_id_,
                        sdf_map_->stuckClearRadiusScale());
        }
    }
    auto   t_end1       = node_->now();
    double duration_ms1 = (t_end1 - t_start1).seconds() * 1000.0;
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[SNN Planner] Search time: %.3f ms", duration_ms1);

    ed_->path_next_goal_.clear();
    ed_->path_next_goal_.reserve(path_grid.size());

    for (const auto& idx : path_grid)
    {
        Eigen::Vector3d wp;
        sdf_map_->gridIndexToPos(idx, wp);
        ed_->path_next_goal_.push_back(wp);
    }
    LogSnnGridPath(ep_->drone_id_, path_grid, sdf_map_);

    DedupeConsecutivePath(ed_->path_next_goal_, kSnnPathDedupeConsecutiveEpsM);

    if (ed_->path_next_goal_.size() < 2)
    {
        const bool straight_free = sdf_map_->isSegmentColumnFree(pos_plan, modified_next_pos);
        if (goal_dist_xy > ep_->degenerate_goal_dist_m_ && straight_free)
        {
            ed_->path_next_goal_ = { pos_plan, 0.5 * (pos_plan + modified_next_pos), modified_next_pos };
            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                        "[planTrajToView][d%d] SNN same-cell fallback: straight polyline dist=%.2fm start=(%.2f,%.2f) goal=(%.2f,%.2f)", ep_->drone_id_,
                        goal_dist_xy, pos_plan.x(), pos_plan.y(), modified_next_pos.x(), modified_next_pos.y());
        }
        else
        {
            if (goal_dist_xy > ep_->degenerate_goal_dist_m_ && !straight_free)
            {
                RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                            "[planTrajToView][d%d] SNN same-cell fallback blocked: segment not column-free dist=%.2fm", ep_->drone_id_, goal_dist_xy);
            }
            else
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("exploration_manager"), "[SNN Planner] Path Planning FAILED! "
                                                                                  << "Pos: (" << pos.head<2>().transpose() << ") -> ("
                                                                                  << modified_next_pos.head<2>().transpose() << "), "
                                                                                  << "Index: [" << start.transpose() << "] -> [" << target.transpose() << "]");
            }
            return false;
        }
    }

    const Eigen::Vector3d p_start = ed_->path_next_goal_.front();
    const Eigen::Vector3d p_end   = ed_->path_next_goal_.back();
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[SNN Planner] Start: [%.2f, %.2f], End: [%.2f, %.2f]", p_start.x(), p_start.y(), p_end.x(),
                p_end.y());

    if (!shortenPath(ed_->path_next_goal_))
    {
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                    "[planTrajToView][d%d] shortenPath failed on SNN polyline after dedupe; abort (avoid SIGSEGV in traj optimizer).", ep_->drone_id_);
        return false;
    }
    return ed_->path_next_goal_.size() >= 2;
}

bool FastExplorationManager::planEdtAstarFallback(const Vector3d& pos_plan, const Vector3d& modified_next_pos, bool optimistic)
{
    if (!planner_manager_->path_finder_)
        return false;

    const int ret = planner_manager_->path_finder_->search(pos_plan, modified_next_pos, optimistic);
    if (ret != Astar::REACH_END)
        return false;

    ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
    DedupeConsecutivePath(ed_->path_next_goal_, kSnnPathDedupeConsecutiveEpsM);
    for (auto& wp : ed_->path_next_goal_)
        wp.z() = pos_plan.z();

    std::function<bool(const Vector3d&, const Vector3d&)> vehicle_seg_free = [this](const Vector3d& a, const Vector3d& b) {
        return planner_manager_->path_finder_ && planner_manager_->path_finder_->isSegmentVehicleFree(a, b, true);
    };
    if (!shortenPath(ed_->path_next_goal_, &vehicle_seg_free) || ed_->path_next_goal_.size() < 2)
        return false;

    const Eigen::Vector3d p_start = ed_->path_next_goal_.front();
    const Eigen::Vector3d p_end   = ed_->path_next_goal_.back();
    RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planTrajToView][d%d] EDT A* fallback Start: [%.2f, %.2f], End: [%.2f, %.2f], pts: %zu",
                ep_->drone_id_, p_start.x(), p_start.y(), p_end.x(), p_end.y(), ed_->path_next_goal_.size());
    return true;
}

int FastExplorationManager::planTrajToView(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos,
                                           const double& next_yaw)
{
    double diff0   = next_yaw - yaw[0];
    double diff1   = fabs(diff0);
    double time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;

    const bool optimistic = true;

    if (planner_manager_->path_finder_)
        planner_manager_->path_finder_->reset();

    // 规划高度与检测高度共用同一参数（static_collision.nav_z），避免两处硬编码漂移。
    const double    z_plan            = node_->has_parameter("static_collision.nav_z") ? node_->get_parameter("static_collision.nav_z").as_double()
                                                                                       : node_->declare_parameter<double>("static_collision.nav_z", 0.3);
    Eigen::Vector3d pos_plan          = pos;
    pos_plan.z()                      = z_plan;
    Eigen::Vector3d modified_next_pos = next_pos;
    modified_next_pos.z()             = z_plan;

    const double goal_dist_xy      = (modified_next_pos - pos_plan).head<2>().norm();
    bool         path_from_arrived = false;

    if (goal_dist_xy < ep_->degenerate_goal_dist_m_)
    {
        // 目标极近(<5cm)，跳过SNN规划，设置极短路径让下个FSM周期自动换目标
        ed_->path_next_goal_ = { pos_plan, modified_next_pos };
        path_from_arrived    = true;
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("exploration_manager"), *node_->get_clock(), 1000,
                             "[planTrajToView][d%d] degenerate goal dist=%.3fm, skip SNN (will replan next cycle)", ep_->drone_id_, goal_dist_xy);
    }
    else if (goal_dist_xy <= ep_->viewpoint_arrived_dist_m_)
    {
        // 5~55cm: 直线可达,3点路径跳过SNN,不做障碍检查(靠safetyCallback兜底)
        ed_->path_next_goal_ = { pos_plan, 0.5 * (pos_plan + modified_next_pos), modified_next_pos };
        path_from_arrived    = true;
        RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[planTrajToView][d%d] viewpoint arrived dist_xy=%.3fm, direct 3-point path skip SNN",
                    ep_->drone_id_, goal_dist_xy);
    }

    if (!path_from_arrived)
    {
        if (planSnnPolylineToGoal(pos_plan, modified_next_pos, pos, goal_dist_xy))
        {
            consecutive_snn_path_fail_ = 0;
        }
        else
        {
            consecutive_snn_path_fail_++;
            const int  trigger         = std::max(0, ep_->snn_fail_astar_trigger_count_);
            const bool straight_free   = sdf_map_->isSegmentColumnFree(pos_plan, modified_next_pos);
            const bool immediate_astar = goal_dist_xy > ep_->degenerate_goal_dist_m_ && !straight_free;
            const bool try_astar       = immediate_astar || (trigger > 0 && consecutive_snn_path_fail_ >= trigger);
            if (try_astar && planEdtAstarFallback(pos_plan, modified_next_pos, optimistic))
            {
                consecutive_snn_path_fail_ = 0;
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planTrajToView][d%d] SNN failed (consecutive=%d, astar_trigger=%d, immediate=%d)",
                            ep_->drone_id_, consecutive_snn_path_fail_, trigger, static_cast<int>(immediate_astar));
                return FAIL;
            }
        }
    }

    if (ed_->path_next_goal_.size() < 2)
    {
        return FAIL;
    }

    {
        simplifySnnPathForTraj(ed_->path_next_goal_, kMaxSnnFullPathWaypoints);
    }

    if (ed_->path_next_goal_.size() < 2)
    {
        return FAIL;
    }

    ed_->kino_path_.clear();

    const double radius = std::max(1.5, ep_->local_traj_radius_);

    // 默认使用完整路径，按弧长截断到局部规划半径
    std::vector<Eigen::Vector3d> traj_path = ed_->path_next_goal_;
    double                       len       = 0.0;
    for (size_t i = 1; i < traj_path.size(); ++i)
    {
        const double seg_len = (traj_path[i] - traj_path[i - 1]).norm();

        if (len + seg_len > radius)
        {
            const double    need   = std::max(0.0, radius - len);
            const double    ratio  = seg_len > 1e-6 ? std::min(1.0, need / seg_len) : 0.0;
            Eigen::Vector3d interp = traj_path[i - 1] + (traj_path[i] - traj_path[i - 1]) * ratio;

            traj_path.resize(i);
            traj_path.push_back(interp);
            break;
        }

        len += seg_len;
    }

    {
        const size_t max_traj_pts =
            static_cast<size_t>(std::max(3, ep_->snn_traj_max_waypoints_ > 0 ? ep_->snn_traj_max_waypoints_ : static_cast<int>(kDefaultSnnTrajMaxWaypoints)));
        const size_t before_cap = traj_path.size();
        simplifySnnPathForTraj(traj_path, max_traj_pts);
        if (before_cap > max_traj_pts && traj_path.size() != before_cap)
        {
            RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[planTrajToView][d%d] traj waypoint cap: pts %zu -> %zu (max=%zu)", ep_->drone_id_,
                        before_cap, traj_path.size(), max_traj_pts);
        }
    }

    EnsureMinThreePathPoints(traj_path);

    ed_->next_goal_ = traj_path.back();

    // 注入停止状态（速度门控）队友的实际位置，供轨迹优化 swarm 项按真实停点避让。
    {
        const double                   now_s = node_->now().seconds();
        std::map<int, Eigen::Vector3d> static_peers;
        for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
        {
            const int id = i + 1;
            if (id == ep_->drone_id_)
                continue;
            const auto& s = ed_->swarm_state_[i];
            if (s.vel_.head<2>().norm() > kDynamicClearSpeedEps)
                continue;  // 移动车由轨迹避让，不入静止集合
            if (now_s - s.stamp_ > kStaticPeerStateStaleS)
                continue;  // 状态过旧：位置不可信，不入静止避让集合
            static_peers[id] = s.pos_;
        }
        planner_manager_->setStaticPeerPoses(static_peers);
    }

    planner_manager_->planExploreTraj(traj_path, vel, acc, time_lb);

    constexpr double kTimeLbSlackSec     = 0.5;
    constexpr double kMinPosDurationSafe = 0.08;
    const double     min_pos_duration    = std::max(time_lb - kTimeLbSlackSec, kMinPosDurationSafe);
    const double     pos_duration_before = planner_manager_->local_data_.position_traj_.getTimeSum();
    planner_manager_->ensurePositionTrajMinDuration(min_pos_duration);
    const double pos_duration = planner_manager_->local_data_.position_traj_.getTimeSum();
    if (pos_duration + 1e-4 < kMinPosDurationSafe)
        return FAIL;
    if (pos_duration > pos_duration_before + 1e-4)
    {
        RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[planTrajToView][d%d] stretch pos_traj: %.4f -> %.4f s (time_lb=%.4f s) for yaw sync.",
                    ep_->drone_id_, pos_duration_before, pos_duration, time_lb);
    }

    planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);

    return SUCCEED;
}

/**
 * @brief 从当前地图中检测边界并计算视点， 为探索规划提供数据
 * @param
 *        in:   当前无人机位置
 *        out： 当前活跃的边界的数量
 **/
int FastExplorationManager::updateFrontierStruct(const Eigen::Vector3d& pos)
{
    auto t1 = node_->now();
    auto t2 = t1;
    ed_->views_.clear();

    // 从SDF地图中检测已知和未知区域的交界，并分组为簇
    frontier_finder_->searchFrontiers(pos);

    double frontier_time = (node_->now() - t1).seconds();  //边界搜索耗时
    t1                   = node_->now();

    // 计算所有边界簇的候选视点 (x, y, z, yaw)，并筛选出信息量大的视点;  //确定哪些边界值得探索
    // frontier_finder_->computeFrontiersToVisit(new_grid);
    frontier_finder_->computeFrontiersToVisit();

    // 获取更新后的边界信息
    frontier_finder_->getFrontiers(ed_->frontiers_);              // 获取活跃边界列表
    frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);  // 获取休眠边界列表
    frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);     // 获取边界框，用于可视化

    // 获取边界的最佳视点信息
    frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);

    // for (int i = 0; i < ed_->points_.size(); ++i)
    // {
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("exploration_manager"),"\033[32m视点位置: " << ed_->points_[i] << ", "
    //                                          << "视点航向: " << ed_->yaws_[i] << "\033[0m");
    // }

    // 生成视点可视化数据,  为每个视点添加一个方向向量 (长度 2.0)，用于 Rviz 显示
    ed_->views_.reserve(ed_->points_.size());
    for (int i = 0; i < ed_->points_.size(); ++i)
        ed_->views_.push_back(ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

    if (ed_->frontiers_.empty())
    {
        // RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "No coverable frontier.");
        return 0;
    }

    double view_time = (node_->now() - t1).seconds();
    t1               = node_->now();

    // 更新边界成本矩阵, 为后续任务分配（如 CVRP）计算边界间的距离或信息增益成本
    frontier_finder_->updateFrontierCostMatrix();

    double mat_time   = (node_->now() - t1).seconds();
    double total_time = frontier_time + view_time + mat_time;
    // RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "Drone %d: frontier t: %lf, viewpoint t: %lf, mat: %lf", ep_->drone_id_, frontier_time, view_time,
    // mat_time);

    // RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "Total t: %lf", (node_->now() - t2).seconds());
    return ed_->frontiers_.size();
}

namespace
{

    void dedupePreserveOrder(std::vector<int>& ids)
    {
        std::unordered_set<int> seen;
        std::vector<int>        out;
        out.reserve(ids.size());
        for (int id : ids)
        {
            if (seen.insert(id).second)
                out.push_back(id);
        }
        ids.swap(out);
    }

    bool readMtspTourRawIds(const std::string& tour_path, std::vector<int>& raw_ids)
    {
        raw_ids.clear();
        std::ifstream fin(tour_path);
        if (!fin.good())
            return false;

        std::string res;
        while (std::getline(fin, res))
        {
            if (res.compare("TOUR_SECTION") == 0)
                break;
        }
        while (std::getline(fin, res))
        {
            if (res.empty())
                continue;
            const int id = std::stoi(res);
            raw_ids.push_back(id - 1);
            if (id == -1)
                break;
        }
        return !raw_ids.empty();
    }

    bool solveAmtspTourInProcess(const std::string& par_path, const std::string& tour_path, int expected_dimension, std::vector<int>& raw_ids)
    {
        solveMTSPWithLKH3(par_path.c_str());
        if (!readMtspTourRawIds(tour_path, raw_ids))
            return false;
        if (expected_dimension > 0 && !raw_ids.empty())
        {
            int max_node = 0;
            for (int id : raw_ids)
                max_node = std::max(max_node, id + 1);
            if (max_node > expected_dimension)
            {
                raw_ids.clear();
                return false;
            }
        }
        return !raw_ids.empty();
    }

    bool parseMtspToursFromRawIds(const std::vector<int>& ids, int dimension, int drone_num, std::vector<std::vector<int>>& tours)
    {
        tours.clear();
        std::vector<int> tour;
        for (int id : ids)
        {
            if (id > 0 && id <= drone_num)
            {
                tour.clear();
                tour.push_back(id);
            }
            else if (id >= dimension || id <= 0)
            {
                tours.push_back(tour);
                tour.clear();
            }
            else
            {
                tour.push_back(id);
            }
        }
        if (!tour.empty())
            tours.push_back(tour);
        return !tours.empty();
    }

}  // namespace

void FastExplorationManager::retainStickyGridIdsAfterPrune(vector<int>& grid_ids)
{
    if (!grid_ids.empty() || ed_->last_grid_ids_.empty() || !hgrid_)
        return;

    std::vector<int> kept;
    kept.reserve(ed_->last_grid_ids_.size());
    std::unordered_set<int> seen;
    for (int gid : ed_->last_grid_ids_)
    {
        if (!seen.insert(gid).second)
            continue;
        std::vector<int> ftrs;
        hgrid_->getFrontiersInGrid({ gid }, ftrs);
        if (!ftrs.empty())
            kept.push_back(gid);
    }
    if (!kept.empty())
    {
        dedupePreserveOrder(kept);
        grid_ids = std::move(kept);
    }
}

bool FastExplorationManager::collectNearbyGridIdsWithFrontiers(const Vector3d& cur_pos, vector<int>& out_grid_ids, double max_radius, int max_grids) const
{
    out_grid_ids.clear();
    if (!hgrid_ || max_grids <= 0 || max_radius <= 0.0)
        return false;

    std::vector<int> active_grids;
    hgrid_->getActiveGrids(active_grids);

    std::vector<std::pair<int, double>> candidates;
    candidates.reserve(active_grids.size());
    for (int gid : active_grids)
    {
        std::vector<int> ftrs;
        hgrid_->getFrontiersInGrid({ gid }, ftrs);
        if (ftrs.empty())
            continue;

        double nearest = std::numeric_limits<double>::max();
        for (int fid : ftrs)
        {
            if (fid < 0 || fid >= static_cast<int>(ed_->points_.size()))
                continue;
            nearest = std::min(nearest, (ed_->points_[fid].head<2>() - cur_pos.head<2>()).norm());
        }
        if (!std::isfinite(nearest) || nearest > max_radius)
            continue;
        candidates.emplace_back(gid, nearest);
    }

    std::sort(candidates.begin(), candidates.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) { return a.second < b.second; });

    std::unordered_set<int> seen;
    for (const auto& [gid, dist] : candidates)
    {
        static_cast<void>(dist);
        if (!seen.insert(gid).second)
            continue;
        out_grid_ids.push_back(gid);
        if (static_cast<int>(out_grid_ids.size()) >= max_grids)
            break;
    }
    return !out_grid_ids.empty();
}

bool FastExplorationManager::pickNearestFrontierWithinRadius(const Vector3d& cur_pos, double max_radius, vector<int>& frontier_ids,
                                                             bool restrict_to_active_grids)
{
    frontier_ids.clear();
    if (max_radius <= 0.0 || ed_->points_.empty())
        return false;

    std::unordered_set<int> allowed_fids;
    if (restrict_to_active_grids && hgrid_)
    {
        std::vector<int> active_grids;
        hgrid_->getActiveGrids(active_grids);
        for (int gid : active_grids)
        {
            std::vector<int> ftrs;
            hgrid_->getFrontiersInGrid({ gid }, ftrs);
            for (int fid : ftrs)
                allowed_fids.insert(fid);
        }
        if (allowed_fids.empty())
            return pickNearestFrontierWithinRadius(cur_pos, max_radius, frontier_ids, false);
    }

    // ---- 收集附近 peer (pos + vel)，用于反应式 + 预测式排斥 ----
    //    30m 内的 peer 参与速度预测，15m 内的计入集群检测
    std::vector<Eigen::Vector3d>                             peer_positions;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> nearby_peers;
    {
        const double now_sec = node_->now().seconds();
        for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
        {
            if (i + 1 == ep_->drone_id_)
                continue;
            const auto& st = ed_->swarm_state_[i];
            if (now_sec - st.stamp_ > 2.0)
                continue;
            peer_positions.push_back(st.pos_);
            if ((st.pos_.head<2>() - cur_pos.head<2>()).norm() < 30.0)
                nearby_peers.emplace_back(st.pos_, st.vel_);
        }
    }

    // ---- 集群检测（排名法 x_bias + y_bias 纵向补充） ----
    int    n_peers_nearby   = 0;
    double cluster_center_x = cur_pos.x();
    double cluster_center_y = cur_pos.y();
    double x_bias           = 0.0;
    double y_bias           = 0.0;
    bool   in_cluster       = false;
    {
        double                              sum_x   = cur_pos.x();
        double                              sum_y   = cur_pos.y();
        int                                 n_total = 1;
        std::vector<std::pair<double, int>> rank_data;
        rank_data.emplace_back(cur_pos.x(), ep_->drone_id_);
        for (const auto& pp : peer_positions)
        {
            const double d = (pp.head<2>() - cur_pos.head<2>()).norm();
            if (d < 15.0)
            {
                n_peers_nearby++;
                sum_x += pp.x();
                sum_y += pp.y();
                n_total++;
            }
            if (d < 15.0)
            {
                // 用 pp 的位置信息；peer_id 未知但排名只需要区分相同 x 的场景
                rank_data.emplace_back(pp.x(), 1000 + n_peers_nearby);
            }
        }
        if (n_peers_nearby >= 1)
        {
            in_cluster       = true;
            cluster_center_x = sum_x / n_total;
            cluster_center_y = sum_y / n_total;
            // ★ 排名法 x_bias：按 x 坐标排序，均匀映射到 [-1, 1]
            {
                std::sort(rank_data.begin(), rank_data.end());
                int my_rank = -1;
                for (int k = 0; k < static_cast<int>(rank_data.size()); ++k)
                {
                    if (rank_data[k].second == ep_->drone_id_)
                    {
                        my_rank = k;
                        break;
                    }
                }
                const int N = rank_data.size();
                x_bias      = (N > 1) ? -1.0 + 2.0 * my_rank / (N - 1) : 0.0;
            }
            // y_bias：当 x_bias 弱时，用 y 方向补充（覆盖垂直一字等场景）
            const double spread = std::max(5.0, 15.0 * 0.5);
            y_bias              = std::max(-1.0, std::min(1.0, (cur_pos.y() - cluster_center_y) / spread));
            if (std::abs(x_bias) >= 0.3)
                y_bias *= 0.3;  // x 有效时 y 仅做微调
            {
                constexpr double kMinBias = 0.3;
                if (std::abs(y_bias) < kMinBias)
                    y_bias = 0.0;
            }
            if (n_peers_nearby == 1)
            {
                x_bias *= 0.5;
                y_bias *= 0.5;
            }
            else
            {
                max_radius = std::max(max_radius, 20.0);
            }
        }
    }

    // ---- 评分选择（x_bias + 反应式 + 速度预测） ----
    int              best_fid         = -1;
    double           best_score       = -std::numeric_limits<double>::infinity();
    constexpr double kPredHorFrontier = 5.0;

    for (int i = 0; i < static_cast<int>(ed_->points_.size()); ++i)
    {
        if (restrict_to_active_grids && !allowed_fids.count(i))
            continue;
        // P0-2：剔除死区对侧/不可达前沿
        if (!admitTarget(cur_pos, ed_->points_[i]))
            continue;
        const Eigen::Vector2d ft = ed_->points_[i].head<2>();
        const double          d  = (ft - cur_pos.head<2>()).norm();
        if (d > max_radius && !in_cluster)
            continue;

        double score = -d;

        // ① x_bias + y_bias 横向/纵向散射（集群模式）
        if (in_cluster)
        {
            const double fx = ed_->points_[i].x();
            const double fy = ed_->points_[i].y();
            score += x_bias * (fx - cluster_center_x) * 0.8;
            score += y_bias * (fy - cluster_center_y) * 0.4;
        }

        // ② 反应式：peers 物理附近的前沿降级
        for (const auto& pp : peer_positions)
        {
            const double pd = (ft - pp.head<2>()).norm();
            if (pd < 8.0)
                score -= 8.0;
        }

        // ③ 速度预测式排斥：peer 正飞向这个前沿
        for (const auto& [pp, pv] : nearby_peers)
        {
            if (pv.head<2>().norm() < 0.3)
                continue;
            const Eigen::Vector2d pred    = pp.head<2>() + pv.head<2>() * kPredHorFrontier;
            const double          pd_pred = (ft - pred).norm();
            if (pd_pred < 8.0)
                score -= 10.0;  // peer 即将到达的前沿

            const double p_to_f_dist = (ft - pp.head<2>()).norm();
            if (p_to_f_dist > 0.5)
            {
                const Eigen::Vector2d p_to_f = (ft - pp.head<2>()) / p_to_f_dist;
                const double          align  = p_to_f.dot(pv.head<2>().normalized());
                if (align > 0.3)
                {
                    const double dist_factor = std::max(0.0, 1.0 - pd_pred / 20.0);
                    score -= 5.0 * align * dist_factor;
                }
            }
        }

        // 统一评分选择
        if (score > best_score)
        {
            best_score = score;
            best_fid   = i;
        }
    }
    if (best_fid < 0 && restrict_to_active_grids)
        return pickNearestFrontierWithinRadius(cur_pos, max_radius, frontier_ids, false);
    if (best_fid < 0)
        return false;
    frontier_ids.push_back(best_fid);
    return true;
}

bool FastExplorationManager::recoverStaleGridAssignment(const Vector3d& cur_pos, vector<int>& grid_ids, vector<int>& frontier_ids)
{
    if (!hgrid_)
        return false;

    const double base_radius = ep_->solo_assign_radius_ > 0.0 ? ep_->solo_assign_radius_ : std::max(8.0, ep_->local_traj_radius_ * 2.0);
    const int    max_grids   = std::max(1, ep_->solo_assign_max_grids_);
    const double max_radius  = base_radius * 8.0;

    std::vector<int> active_grids;
    hgrid_->getActiveGrids(active_grids);

    // ★ 收集 peer 首格目标，恢复时避免多车竞争同网格
    std::unordered_set<int> peer_target_grids;
    {
        const double now_sec = node_->now().seconds();
        for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
        {
            if (i + 1 == ep_->drone_id_)
                continue;
            const auto& st = ed_->swarm_state_[i];
            if (now_sec - st.stamp_ > 2.0)
                continue;
            if (!st.grid_ids_.empty())
                peer_target_grids.insert(st.grid_ids_.front());
        }
    }

    std::vector<std::pair<int, double>> grid_nearest;
    grid_nearest.reserve(active_grids.size());
    for (int gid : active_grids)
    {
        std::vector<int> ftrs;
        hgrid_->getFrontiersInGrid({ gid }, ftrs);
        if (ftrs.empty())
            continue;

        double nearest = std::numeric_limits<double>::max();
        for (int fid : ftrs)
        {
            if (fid < 0 || fid >= static_cast<int>(ed_->points_.size()))
                continue;
            // P0-2：剔除死区对侧/不可达 + 过近(degenerate)前沿，避免死区旁原地 replan 循环
            const Eigen::Vector3d& fp = ed_->points_[fid];
            if (!admitTarget(cur_pos, fp))
                continue;
            const double d = (fp.head<2>() - cur_pos.head<2>()).norm();
            if (d < ep_->degenerate_goal_dist_m_)
                continue;
            nearest = std::min(nearest, d);
        }
        if (!std::isfinite(nearest) || nearest > max_radius)
            continue;

        // 若该网格被 peer 选为首格目标，增加距离（降低优先级）
        if (peer_target_grids.count(gid) != 0u)
            nearest *= 1.8;

        grid_nearest.emplace_back(gid, nearest);
    }

    std::sort(grid_nearest.begin(), grid_nearest.end(), [](const std::pair<int, double>& a, const std::pair<int, double>& b) { return a.second < b.second; });

    for (const double mult : { 1.0, 2.0, 4.0, 8.0 })
    {
        const double            threshold = base_radius * mult;
        vector<int>             nearby;
        std::unordered_set<int> seen;
        for (const auto& [gid, dist] : grid_nearest)
        {
            if (dist > threshold)
                continue;
            if (!seen.insert(gid).second)
                continue;
            nearby.push_back(gid);
            if (static_cast<int>(nearby.size()) >= max_grids)
                break;
        }
        if (nearby.empty())
            continue;

        grid_ids = nearby;
        persistSoloGridAssignment(grid_ids, cur_pos);
        hgrid_->getFrontiersInGrid(grid_ids, frontier_ids);
        // P0-2：剔除不可达/过近前沿，保证返回的第一个目标可规划
        frontier_ids.erase(std::remove_if(frontier_ids.begin(), frontier_ids.end(),
                                          [&](int fid) {
                                              if (fid < 0 || fid >= static_cast<int>(ed_->points_.size()))
                                                  return true;
                                              const Eigen::Vector3d& fp = ed_->points_[fid];
                                              return !admitTarget(cur_pos, fp);
                                          }),
                           frontier_ids.end());
        if (frontier_ids.empty())
            continue;

        std::sort(frontier_ids.begin(), frontier_ids.end(), [&](int a, int b) {
            return (ed_->points_[a].head<2>() - cur_pos.head<2>()).squaredNorm() < (ed_->points_[b].head<2>() - cur_pos.head<2>()).squaredNorm();
        });
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[recoverStaleGridAssignment][d%d] Reassigned %zu grid(s) within %.1fm, frontiers=%zu",
                    ep_->drone_id_, grid_ids.size(), threshold, frontier_ids.size());
        return true;
    }

    // ★ 全局回退：集群检测 + x 偏置散射 + 速度预测式排斥
    bool                                                     cluster_scatter  = false;
    double                                                   cluster_center_x = cur_pos.x();
    double                                                   cluster_center_y = cur_pos.y();
    double                                                   x_bias           = 0.0;
    double                                                   y_bias           = 0.0;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> nearby_peers;
    {
        const double                        now_sec = node_->now().seconds();
        double                              sum_x   = cur_pos.x();
        double                              sum_y   = cur_pos.y();
        int                                 n_total = 1;
        int                                 n_peers = 0;
        std::vector<std::pair<double, int>> rank_data_rec;
        rank_data_rec.emplace_back(cur_pos.x(), ep_->drone_id_);
        for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
        {
            if (i + 1 == ep_->drone_id_)
                continue;
            const auto& st = ed_->swarm_state_[i];
            if (now_sec - st.stamp_ > 2.0)
                continue;
            const double d = (st.pos_.head<2>() - cur_pos.head<2>()).norm();
            if (d < 30.0)
                nearby_peers.emplace_back(st.pos_, st.vel_);
            if (d < 15.0)
            {
                n_peers++;
                sum_x += st.pos_.x();
                sum_y += st.pos_.y();
                rank_data_rec.emplace_back(st.pos_.x(), i + 1);
                n_total++;
            }
        }
        if (n_peers >= 1)
        {
            cluster_scatter  = true;
            cluster_center_x = sum_x / n_total;
            cluster_center_y = sum_y / n_total;
            // 排名法 x_bias：按 x 坐标排序，均匀映射到 [-1, 1]
            {
                std::sort(rank_data_rec.begin(), rank_data_rec.end());
                int my_rank = -1;
                for (int k = 0; k < static_cast<int>(rank_data_rec.size()); ++k)
                {
                    if (rank_data_rec[k].second == ep_->drone_id_)
                    {
                        my_rank = k;
                        break;
                    }
                }
                const int N = rank_data_rec.size();
                x_bias      = (N > 1) ? -1.0 + 2.0 * my_rank / (N - 1) : 0.0;
            }
            const double spread = std::max(5.0, 15.0 * 0.5);
            y_bias              = std::max(-1.0, std::min(1.0, (cur_pos.y() - cluster_center_y) / spread));
            if (std::abs(x_bias) >= 0.3)
                y_bias *= 0.3;
            {
                constexpr double kMinBias = 0.3;
                if (std::abs(y_bias) < kMinBias)
                    y_bias = 0.0;
            }
            if (n_peers == 1)
            {
                x_bias *= 0.5;
                y_bias *= 0.5;
            }
        }
    }

    int              best_gid        = -1;
    int              best_fid        = -1;
    double           best_score      = -std::numeric_limits<double>::infinity();
    constexpr double kPredHorRecover = 5.0;
    for (int gid : active_grids)
    {
        std::vector<int> ftrs;
        hgrid_->getFrontiersInGrid({ gid }, ftrs);
        const Eigen::Vector3d gc = hgrid_->getCenter(gid);
        for (int fid : ftrs)
        {
            if (fid < 0 || fid >= static_cast<int>(ed_->points_.size()))
                continue;
            // P0-2：剔除死区对侧/不可达 + 过近前沿
            const Eigen::Vector3d& fp = ed_->points_[fid];
            if (!admitTarget(cur_pos, fp))
                continue;
            const double d = (fp.head<2>() - cur_pos.head<2>()).norm();
            if (d < ep_->degenerate_goal_dist_m_)
                continue;
            double score = -d;
            // Peer grid penalty: if this grid is a peer's first target, deprioritize
            if (peer_target_grids.count(gid) != 0u)
                score -= 20.0;
            // 横向散射偏置：集群中按队列位置选方向
            if (cluster_scatter)
            {
                const double lateral = x_bias * (gc.x() - cluster_center_x) * 1.2;
                const double vert    = y_bias * (gc.y() - cluster_center_y) * 0.6;
                score += lateral + vert;
            }
            // 速度预测式排斥：peer 正在飞向这个 grid 区域
            for (const auto& [pp, pv] : nearby_peers)
            {
                if (pv.head<2>().norm() < 0.3)
                    continue;
                const Eigen::Vector2d pred    = pp.head<2>() + pv.head<2>() * kPredHorRecover;
                const Eigen::Vector2d gc2     = gc.head<2>();
                const double          pd_pred = (gc2 - pred).norm();
                if (pd_pred < 12.0)
                    score -= 8.0;
                const double g_to_p_dist = (gc2 - pp.head<2>()).norm();
                if (g_to_p_dist > 1.0)
                {
                    const Eigen::Vector2d g_to_p = (gc2 - pp.head<2>()) / g_to_p_dist;
                    const double          align  = g_to_p.dot(pv.head<2>().normalized());
                    if (align > 0.3)
                    {
                        const double dist_factor = std::max(0.0, 1.0 - pd_pred / 20.0);
                        score -= 4.0 * align * dist_factor;
                    }
                }
            }
            if (score > best_score)
            {
                best_score = score;
                best_gid   = gid;
                best_fid   = fid;
            }
        }
    }
    if (best_gid < 0 || best_fid < 0)
        return false;

    grid_ids     = { best_gid };
    frontier_ids = { best_fid };
    persistSoloGridAssignment(grid_ids, cur_pos);
    RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                "[recoverStaleGridAssignment][d%d] Global reassign to grid=%d frontier=%d dist=%.2fm (active_grids=%zu)", ep_->drone_id_, best_gid, best_fid,
                best_fid >= 0 && best_fid < static_cast<int>(ed_->points_.size()) ? (ed_->points_[best_fid].head<2>() - cur_pos.head<2>()).norm() : 0.0,
                active_grids.size());
    return true;
}

double FastExplorationManager::getMaxLocalGoalDist() const
{
    return GetMaxLocalGoalDist(*ep_);
}

bool FastExplorationManager::UpdateReachabilityMask(const Vector3d& pos)
{
    reach_mask_.clear();
    reach_mask_H_     = 0;
    reach_mask_W_     = 0;
    reach_mask_valid_ = false;
    if (!sdf_map_ || !snn_)
        return false;

    // 严格掩码直接复用 SNN 已处理地图：死区 margin、闭运算、半径膨胀和斜向转移规则均与 plan() 一致。
    // 因此被筛选为可达的目标不会仅因安全地图差异而在后续 SNN 搜路失败。
    if (!UpdateSnnGridFromSdfImpl(this, pos))
        return false;

    Eigen::Vector3i start_idx;
    sdf_map_->posToGridIndex(pos, start_idx);
    const auto mask = snn_->computeReachabilityMask(start_idx);
    const int  h    = snn_->gridHeight();
    const int  w    = snn_->gridWidth();
    if (h <= 0 || w <= 0 || mask.size() != static_cast<size_t>(h) * static_cast<size_t>(w))
        return false;

    reach_mask_       = mask;
    reach_mask_H_     = h;
    reach_mask_W_     = w;
    reach_mask_valid_ = true;
    return true;
}

double FastExplorationManager::peerCoveragePenalty(const Eigen::Vector3d& p) const
{
    if (ep_->peer_cov_penalty_ <= 0.0)
        return 0.0;
    if (!sdf_map_ || !sdf_map_->mm_)
        return 0.0;
    if (sdf_map_->mm_->inPeerCoveredBox(p, ep_->peer_cov_margin_m_))
        return ep_->peer_cov_penalty_;
    return 0.0;
}

bool FastExplorationManager::IsReachablePoint(const Eigen::Vector3d& world_pos) const
{
    if (reach_mask_H_ <= 0 || reach_mask_W_ <= 0 || !sdf_map_)
        return true;  // 掩码不可用：放行，保持旧行为

    Eigen::Vector3i idx;
    sdf_map_->posToGridIndex(world_pos, idx);
    const int  x       = idx.x();
    const int  y       = idx.y();
    const auto in_mask = [&](int cx, int cy) {
        if (cx < 0 || cy < 0 || cx >= reach_mask_H_ || cy >= reach_mask_W_)
            return false;
        return reach_mask_[static_cast<size_t>(cx) * static_cast<size_t>(reach_mask_W_) + static_cast<size_t>(cy)] != 0;
    };
    if (in_mask(x, y))
        return true;
    // 目标格被占据/未达：与 plan() 的目标拉回语义一致（find_nearest_free 半径 6），搜索近邻可达格
    for (int r = 1; r <= 6; ++r)
    {
        for (int dx = -r; dx <= r; ++dx)
        {
            for (int dy = -r; dy <= r; ++dy)
            {
                if (std::abs(dx) != r && std::abs(dy) != r)
                    continue;
                if (in_mask(x + dx, y + dy))
                    return true;
            }
        }
    }
    return false;
}

std::int64_t FastExplorationManager::targetFailureKey(const Vector3d& target) const
{
    const double cell = std::max(0.05, ep_->target_failure_quarantine_cell_m_);
    const auto   x    = static_cast<std::int32_t>(std::llround(target.x() / cell));
    const auto   y    = static_cast<std::int32_t>(std::llround(target.y() / cell));
    return (static_cast<std::int64_t>(x) << 32) | static_cast<std::uint32_t>(y);
}

void FastExplorationManager::pruneTargetFailures()
{
    const double now = node_->now().seconds();
    for (auto it = target_failures_.begin(); it != target_failures_.end();)
    {
        if (it->second.blocked_until_sec <= now)
            it = target_failures_.erase(it);
        else
            ++it;
    }
    while (target_failures_.size() > static_cast<size_t>(ep_->target_failure_quarantine_max_entries_))
        target_failures_.erase(target_failures_.begin());
}

bool FastExplorationManager::isTargetQuarantined(const Vector3d& target, const Vector3d& pos)
{
    if (!ep_->target_failure_quarantine_enable_)
        return false;
    pruneTargetFailures();
    const auto it = target_failures_.find(targetFailureKey(target));
    if (it == target_failures_.end())
        return false;

    const double now = node_->now().seconds();
    if (it->second.blocked_until_sec <= now)
        return false;
    if ((pos - it->second.target).head<2>().norm() > std::max(1.0, ep_->local_traj_radius_))
        return false;
    return true;
}

void FastExplorationManager::MarkExplorationStart()
{
    last_normal_plan_success_time_sec_ = node_->now().seconds();
    stall_wait_rounds_                 = 0;
    stall_first_time_sec_              = -1.0;
    escape_attempts_                   = 0;
    last_escape_time_sec_              = -1.0;
}

void FastExplorationManager::MarkNormalPlanSuccess()
{
    stall_wait_rounds_                 = 0;
    stall_first_time_sec_              = -1.0;
    escape_attempts_                   = 0;
    last_escape_time_sec_              = -1.0;
    last_normal_plan_success_time_sec_ = node_->now().seconds();
}

double FastExplorationManager::noProgressSec(const rclcpp::Time& now) const
{
    if (last_normal_plan_success_time_sec_ <= 0.0)
        return 0.0;
    return std::max(0.0, now.seconds() - last_normal_plan_success_time_sec_);
}

void FastExplorationManager::PauseNoProgress(const rclcpp::Time& now)
{
    if (last_normal_plan_success_time_sec_ > 0.0)
        last_normal_plan_success_time_sec_ = now.seconds();
}

void FastExplorationManager::RestartStallWindow()
{
    stall_wait_rounds_    = 0;
    stall_first_time_sec_ = -1.0;
}

int FastExplorationManager::HandleStallBeforeWait(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw)
{
    // 总开关关闭：完全退回改动前的“无限等待”行为，便于回退验证。
    if (!ep_->liveness_enable_)
        return WAIT_FOR_MAP_UPDATE;

    const double now_sec = node_->now().seconds();
    if (stall_wait_rounds_ == 0)
        stall_first_time_sec_ = now_sec;
    ++stall_wait_rounds_;

    last_stall_target_       = ed_->next_pos_;
    last_stall_target_valid_ = true;

    if (stall_wait_rounds_ < ep_->stall_escape_after_waits_)
        return WAIT_FOR_MAP_UPDATE;

    if (last_escape_time_sec_ > 0.0 && now_sec - last_escape_time_sec_ < ep_->stall_escape_interval_sec_)
        return WAIT_FOR_MAP_UPDATE;

    // 掩码无效时禁止盲选：不让 reach_mask_valid_ 影响收敛，失败也计入 attempts。
    const Vector3d failed_target = last_stall_target_valid_ ? last_stall_target_ : ed_->next_pos_;
    if (reach_mask_valid_ && tryStallEscape(pos, vel, acc, yaw, failed_target))
    {
        stall_wait_rounds_    = 0;
        stall_first_time_sec_ = -1.0;
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[stall][d%d] escape succeeded, resume exploration", ep_->drone_id_);
        return SUCCEED;
    }

    if (escape_attempts_ < ep_->stall_escape_max_attempts_)
        ++escape_attempts_;
    last_escape_time_sec_ = now_sec;

    const bool   exhausted = escape_attempts_ >= ep_->stall_escape_max_attempts_;
    const double stagger   = (ep_->drone_id_ - 1) * ep_->stall_return_stagger_sec_;
    if (exhausted && stall_first_time_sec_ >= 0.0 && now_sec >= stall_first_time_sec_ + stagger)
    {
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[stall][d%d] escape %d/%d failed after %.1fs, give up and trigger final return", ep_->drone_id_,
                    escape_attempts_, ep_->stall_escape_max_attempts_, now_sec - stall_first_time_sec_);
        return NO_GRID;
    }

    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("exploration_manager"), *node_->get_clock(), 5000, "[stall][d%d] wait rounds=%d escape %d/%d failed, keep waiting",
                         ep_->drone_id_, stall_wait_rounds_, escape_attempts_, ep_->stall_escape_max_attempts_);
    return WAIT_FOR_MAP_UPDATE;
}

bool FastExplorationManager::tryStallEscape(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw,
                                            const Vector3d& last_failed_target)
{
    if (!reach_mask_valid_ || !hgrid_)
        return false;

    std::vector<Vector3d> candidates;
    candidates.reserve(ed_->points_.size() + 16);
    for (size_t i = 0; i < ed_->points_.size(); ++i)
        candidates.push_back(ed_->points_[i]);

    StallEscapeContext ctx;
    ctx.self_pos        = pos;
    ctx.min_dist_m      = ep_->stall_escape_min_dist_m_;
    ctx.max_dist_m      = ep_->stall_escape_max_dist_m_;
    ctx.has_last_failed = true;
    ctx.last_failed_key = targetFailureKey(last_failed_target);
    ctx.self_region     = region_allocator_ ? region_allocator_->ownerRegionOfPoint(pos) : -1;
    ctx.reachable       = [this](const Vector3d& p) { return IsReachablePoint(p); };
    ctx.region_of       = [this](const Vector3d& p) { return region_allocator_ ? region_allocator_->ownerRegionOfPoint(p) : -1; };
    ctx.key_of          = [this](const Vector3d& p) { return targetFailureKey(p); };
    ctx.extra_penalty   = [this](const Vector3d& p) {
        double penalty = peerCoveragePenalty(p);
        for (size_t i = 0; i < ed_->swarm_state_.size(); ++i)
        {
            if (static_cast<int>(i) + 1 == ep_->drone_id_)
                continue;
            const auto& s = ed_->swarm_state_[i];
            if (s.pos_.head<2>().norm() < 1e-6)
                continue;
            if ((s.pos_ - p).head<2>().norm() < 8.0)
                penalty += 8.0;
            if (!s.grid_ids_.empty() && hgrid_->isValidGlobalGridId(s.grid_ids_.front()))
            {
                if ((hgrid_->getCenter(s.grid_ids_.front()) - p).head<2>().norm() < 3.0)
                    penalty += 8.0;
            }
        }
        return penalty;
    };

    Vector3d best;
    bool     found = SelectStallEscapeCandidate(candidates, ctx, best);
    if (!found)
    {
        // top viewpoint 全部被过滤（或本轮根本没有 viewpoints）时，退化为 active 网格中心。
        // 注意：不能用 getFrontiersInGrid 判空——本轮 inputFrontiers 已按同一份 averages_ 重建，会得出“无前沿”的假象。
        std::vector<int> active_grids;
        hgrid_->getActiveGrids(active_grids);
        for (size_t i = 0; i < active_grids.size(); ++i)
            candidates.push_back(hgrid_->getCenter(active_grids[i]));
        found = SelectStallEscapeCandidate(candidates, ctx, best);
    }
    if (!found)
    {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("exploration_manager"), *node_->get_clock(), 5000,
                             "[stall][d%d] escape: no candidate passes filters (candidates=%zu, region=%d)", ep_->drone_id_, candidates.size(),
                             ctx.self_region);
        return false;
    }

    const double best_yaw  = std::atan2(best.y() - pos.y(), best.x() - pos.x());
    const double best_dist = (best - pos).head<2>().norm();
    ed_->next_pos_         = best;
    ed_->next_yaw_         = best_yaw;

    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("exploration_manager"), *node_->get_clock(), 5000, "[stall][d%d] escape pick target=(%.2f, %.2f) dist=%.2fm",
                         ep_->drone_id_, best.x(), best.y(), best_dist);

    if (planTrajToView(pos, vel, acc, yaw, best, best_yaw) == SUCCEED)
    {
        clearTargetFailure(best);
        consecutive_plan_fail_ = 0;
        ed_->last_next_pos_    = best;
        return true;
    }

    recordTargetFailure(best);
    return false;
}

bool FastExplorationManager::admitTarget(const Vector3d& pos, const Vector3d& target)
{
    const double distance = (target - pos).head<2>().norm();
    return distance >= ep_->degenerate_goal_dist_m_ && IsReachablePoint(target) && !isTargetQuarantined(target, pos);
}

void FastExplorationManager::recordTargetFailure(const Vector3d& target)
{
    if (!ep_->target_failure_quarantine_enable_)
        return;

    const auto   key   = targetFailureKey(target);
    const double now   = node_->now().seconds();
    auto&        state = target_failures_[key];
    state.target       = target;
    state.failure_count++;
    const double backoff =
        std::min(ep_->target_failure_quarantine_max_sec_, ep_->target_failure_quarantine_initial_sec_ * std::pow(2.0, std::max(0, state.failure_count - 1)));
    state.blocked_until_sec = now + backoff;
    RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion][d%d] quarantine failed target [%.2f, %.2f] for %.1fs (failures=%d)",
                ep_->drone_id_, target.x(), target.y(), backoff, state.failure_count);
    pruneTargetFailures();
}

void FastExplorationManager::clearTargetFailure(const Vector3d& target)
{
    target_failures_.erase(targetFailureKey(target));
}

bool FastExplorationManager::tryPlanFailRetarget(const Vector3d& pos, vector<int>& grid_ids, vector<int>& frontier_ids, Vector3d& out_pos, double& out_yaw)
{
    vector<int> tmp_grid = grid_ids;
    vector<int> tmp_ftr;
    if (recoverStaleGridAssignment(pos, tmp_grid, tmp_ftr) && !tmp_ftr.empty())
    {
        grid_ids     = tmp_grid;
        frontier_ids = tmp_ftr;
        out_pos      = ed_->points_[tmp_ftr[0]];
        out_pos.z()  = 0.3;
        out_yaw      = ed_->yaws_[tmp_ftr[0]];
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion][d%d] Plan fail retarget via recoverStale frontier=%d dist=%.2fm",
                    ep_->drone_id_, tmp_ftr[0], (out_pos - pos).head<2>().norm());
        return (out_pos - pos).head<2>().norm() > ep_->degenerate_goal_dist_m_;
    }

    if (pickNearestFrontierWithinRadius(pos, getMaxLocalGoalDist(), tmp_ftr, true) && !tmp_ftr.empty())
    {
        out_pos     = ed_->points_[tmp_ftr[0]];
        out_pos.z() = 0.3;
        out_yaw     = ed_->yaws_[tmp_ftr[0]];
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[planExploreMotion][d%d] Plan fail retarget via nearest frontier=%d dist=%.2fm", ep_->drone_id_,
                    tmp_ftr[0], (out_pos - pos).head<2>().norm());
        return (out_pos - pos).head<2>().norm() > ep_->degenerate_goal_dist_m_;
    }
    return false;
}

void FastExplorationManager::persistSoloGridAssignment(const vector<int>& grid_ids, const Vector3d& cur_pos)
{
    if (grid_ids.empty())
        return;

    vector<int> normalized = grid_ids;
    dedupePreserveOrder(normalized);
    if (hgrid_)
        hgrid_->sanitizeGlobalGridIds(normalized);
    if (normalized.empty())
        return;

    auto& state_grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;
    state_grid_ids       = normalized;
    ed_->last_grid_ids_  = normalized;
    ed_->reallocated_    = false;
    if (hgrid_)
        hgrid_->getGridTour(normalized, cur_pos, ed_->grid_tour_, ed_->grid_tour2_);
}

bool FastExplorationManager::trySoloAssignGridsIfEmpty(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw)
{
    static_cast<void>(cur_vel);
    static_cast<void>(cur_yaw);
    auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;
    if (!grid_ids.empty())
        return true;

    if (!ep_->solo_assign_enable_)
        return false;

    if (ep_->drone_num_ > 1)
    {
        const double now_sec = node_->now().seconds();
        for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
        {
            if (i + 1 == ep_->drone_id_)
                continue;
            const auto& peer = ed_->swarm_state_[i];
            if (now_sec - peer.stamp_ < 2.0 && !peer.grid_ids_.empty())
                return false;
        }
    }

    const double assign_radius = ep_->solo_assign_radius_ > 0.0 ? ep_->solo_assign_radius_ : std::max(8.0, ep_->local_traj_radius_ * 2.0);
    const int    max_grids     = std::max(1, ep_->solo_assign_max_grids_);
    vector<int>  assigned;
    if (!collectNearbyGridIdsWithFrontiers(cur_pos, assigned, assign_radius, max_grids))
        return false;

    persistSoloGridAssignment(assigned, cur_pos);
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[trySoloAssignGridsIfEmpty][d%d] assigned %zu nearby grids within %.1fm (cap=%d)", ep_->drone_id_,
                assigned.size(), assign_radius, max_grids);
    return true;
}

void FastExplorationManager::findGridAndFrontierPath(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids,
                                                     vector<int>& frontier_ids)
{
    auto t1 = node_->now();

    // Select nearby drones according to their states' stamp
    vector<Eigen::Vector3d> positions = { cur_pos };
    // vector<Eigen::Vector3d> velocities = { Eigen::Vector3d(0, 0, 0) };
    vector<Eigen::Vector3d> velocities = { cur_vel };
    vector<double>          yaws       = { cur_yaw[0] };

    // Partitioning-based tour planning
    vector<int>         ego_ids;
    vector<vector<int>> other_ids;
    if (!findGlobalTourOfGrid(positions, velocities, ego_ids, other_ids))
    {
        const double assign_radius = ep_->solo_assign_radius_ > 0.0 ? ep_->solo_assign_radius_ : std::max(8.0, ep_->local_traj_radius_ * 2.0);
        vector<int>  nearby_grids;
        if (collectNearbyGridIdsWithFrontiers(cur_pos, nearby_grids, assign_radius, 1))
        {
            // ★ 修复：集群检测 + x/y 偏置散射 + 速度预测式排斥
            bool                                                     in_cluster       = false;
            double                                                   cluster_center_x = cur_pos.x();
            double                                                   cluster_center_y = cur_pos.y();
            double                                                   x_bias           = 0.0;
            double                                                   y_bias           = 0.0;
            int                                                      n_peers          = 0;
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> fg_nearby_peers;
            {
                const double                        now_sec = node_->now().seconds();
                int                                 n_total = 1;
                double                              sum_x   = cur_pos.x();
                double                              sum_y   = cur_pos.y();
                std::vector<std::pair<double, int>> rank_data_fg;
                rank_data_fg.emplace_back(cur_pos.x(), ep_->drone_id_);
                for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
                {
                    if (i + 1 == ep_->drone_id_)
                        continue;
                    const auto& st = ed_->swarm_state_[i];
                    if (now_sec - st.stamp_ > 2.0)
                        continue;
                    const double d = (st.pos_.head<2>() - cur_pos.head<2>()).norm();
                    if (d < 30.0)
                        fg_nearby_peers.emplace_back(st.pos_, st.vel_);
                    if (d < 15.0)
                    {
                        n_peers++;
                        sum_x += st.pos_.x();
                        sum_y += st.pos_.y();
                        rank_data_fg.emplace_back(st.pos_.x(), i + 1);
                        n_total++;
                    }
                }
                if (n_peers >= 1)
                {
                    in_cluster       = true;
                    cluster_center_x = sum_x / n_total;
                    cluster_center_y = sum_y / n_total;
                    // 排名法 x_bias
                    {
                        std::sort(rank_data_fg.begin(), rank_data_fg.end());
                        int my_rank = -1;
                        for (int k = 0; k < static_cast<int>(rank_data_fg.size()); ++k)
                        {
                            if (rank_data_fg[k].second == ep_->drone_id_)
                            {
                                my_rank = k;
                                break;
                            }
                        }
                        const int N = rank_data_fg.size();
                        x_bias      = (N > 1) ? -1.0 + 2.0 * my_rank / (N - 1) : 0.0;
                    }
                    const double spread = std::max(5.0, 15.0 * 0.5);
                    y_bias              = std::max(-1.0, std::min(1.0, (cur_pos.y() - cluster_center_y) / spread));
                    if (std::abs(x_bias) >= 0.3)
                        y_bias *= 0.3;
                    {
                        constexpr double kMinBias = 0.3;
                        if (std::abs(y_bias) < kMinBias)
                            y_bias = 0.0;
                    }
                    if (n_peers == 1)
                    {
                        x_bias *= 0.5;
                        y_bias *= 0.5;
                    }
                }
            }

            if (in_cluster)
            {
                // 集群模式：用 x_bias + 速度预测从 wider 范围选网格
                vector<int>      wider_grids;
                constexpr double kPredHorFg = 5.0;
                if (collectNearbyGridIdsWithFrontiers(cur_pos, wider_grids, assign_radius * 5.0, 8))
                {
                    int    best_gid      = -1;
                    double best_x_reward = -std::numeric_limits<double>::infinity();
                    for (int gid : wider_grids)
                    {
                        const Eigen::Vector3d gc       = hgrid_->getCenter(gid);
                        double                x_reward = x_bias * (gc.x() - cluster_center_x) * 1.0 + y_bias * (gc.y() - cluster_center_y) * 0.5;
                        // 速度预测式排斥：peer 正在飞向这个网格
                        for (const auto& [pp, pv] : fg_nearby_peers)
                        {
                            if (pv.head<2>().norm() < 0.3)
                                continue;
                            const Eigen::Vector2d pred    = pp.head<2>() + pv.head<2>() * kPredHorFg;
                            const double          pd_pred = (gc.head<2>() - pred).norm();
                            if (pd_pred < 12.0)
                                x_reward -= 6.0;
                            const double g_to_p_dist = (gc.head<2>() - pp.head<2>()).norm();
                            if (g_to_p_dist > 1.0)
                            {
                                const Eigen::Vector2d g_to_p = (gc.head<2>() - pp.head<2>()) / g_to_p_dist;
                                const double          align  = g_to_p.dot(pv.head<2>().normalized());
                                if (align > 0.3)
                                {
                                    const double dist_factor = std::max(0.0, 1.0 - pd_pred / 20.0);
                                    x_reward -= 3.0 * align * dist_factor;
                                }
                            }
                        }
                        if (x_reward > best_x_reward)
                        {
                            best_x_reward = x_reward;
                            best_gid      = gid;
                        }
                    }
                    if (best_gid >= 0)
                        nearby_grids = { best_gid };
                }
            }
            else
            {
                // 非集群模式：用 peer 首格目标解决冲突
                std::unordered_set<int> peer_first_grids;
                {
                    const double now_sec = node_->now().seconds();
                    for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
                    {
                        if (i + 1 == ep_->drone_id_)
                            continue;
                        const auto& st = ed_->swarm_state_[i];
                        if (now_sec - st.stamp_ > 2.0)
                            continue;
                        if (!st.grid_ids_.empty())
                            peer_first_grids.insert(st.grid_ids_.front());
                    }
                }
                if (!nearby_grids.empty() && peer_first_grids.count(nearby_grids.front()) != 0u)
                {
                    vector<int> wider_grids;
                    if (collectNearbyGridIdsWithFrontiers(cur_pos, wider_grids, assign_radius * 3.0, 3))
                    {
                        int             best_gid   = -1;
                        double          best_align = 1.0;
                        Eigen::Vector2d cur_dir(std::cos(cur_yaw[0]), std::sin(cur_yaw[0]));
                        if (cur_vel.head<2>().norm() > 0.2)
                            cur_dir = cur_vel.head<2>().normalized();
                        for (int gid : wider_grids)
                        {
                            if (peer_first_grids.count(gid) != 0u)
                                continue;
                            const Eigen::Vector3d gc    = hgrid_->getCenter(gid);
                            const Eigen::Vector2d to_g  = (gc.head<2>() - cur_pos.head<2>()).normalized();
                            const double          align = cur_dir.dot(to_g);
                            if (align < best_align)
                            {
                                best_align = align;
                                best_gid   = gid;
                            }
                        }
                        if (best_gid >= 0)
                            nearby_grids = { best_gid };
                    }
                }
            }
            // ★ 修复结束

            grid_ids = nearby_grids;
            if (hgrid_)
                hgrid_->getFrontiersInGrid(grid_ids, frontier_ids);
            persistSoloGridAssignment(grid_ids, cur_pos);

            // ★ x_bias 注入前沿排序（覆盖所有路径，包括非集群的纯距离排序）
            if (in_cluster)
            {
                RCLCPP_INFO(rclcpp::get_logger("exploration_manager"),
                            "[findGridAndFrontierPath][d%d] x_bias frontier sort: n_peers=%d x_bias=%.2f y_bias=%.2f cx=%.1f cy=%.1f frontiers=%zu",
                            ep_->drone_id_, n_peers, x_bias, y_bias, cluster_center_x, cluster_center_y, frontier_ids.size());
                std::sort(frontier_ids.begin(), frontier_ids.end(), [&](int a, int b) {
                    double sa = -(ed_->points_[a] - cur_pos).squaredNorm() + x_bias * (ed_->points_[a].x() - cluster_center_x) * 1.5
                                + y_bias * (ed_->points_[a].y() - cluster_center_y) * 0.75;
                    double sb = -(ed_->points_[b] - cur_pos).squaredNorm() + x_bias * (ed_->points_[b].x() - cluster_center_x) * 1.5
                                + y_bias * (ed_->points_[b].y() - cluster_center_y) * 0.75;
                    return sa > sb;
                });
            }
            else
            {
                std::sort(frontier_ids.begin(), frontier_ids.end(),
                          [&](int a, int b) { return (ed_->points_[a] - cur_pos).squaredNorm() < (ed_->points_[b] - cur_pos).squaredNorm(); });
            }

            RCLCPP_INFO(rclcpp::get_logger("exploration_manager"),
                        "[findGridAndFrontierPath][d%d] grid tour unavailable, fallback %zu nearby grid(s) within %.1fm frontiers=%zu", ep_->drone_id_,
                        grid_ids.size(), assign_radius, frontier_ids.size());
            return;
        }

        Eigen::Vector3d box_min, box_max;
        sdf_map_->getBox(box_min, box_max);
        const bool   pos_in_box        = sdf_map_->isInBox(cur_pos);
        const double dist_to_box_max_x = box_max.x() - cur_pos.x();
        const double dist_to_box_max_y = box_max.y() - cur_pos.y();

        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                    "[findGridAndFrontierPath][d%d] findGlobalTourOfGrid FAILED and no active grid with frontier fallback "
                    "pos=(%.2f,%.2f) in_box=%d dist_to_box_max=(%.2f,%.2f) frontiers=%zu",
                    ep_->drone_id_, cur_pos.x(), cur_pos.y(), static_cast<int>(pos_in_box), dist_to_box_max_x, dist_to_box_max_y, ed_->averages_.size());
        grid_ids     = {};
        frontier_ids = {};
        return;
    }
    grid_ids = ego_ids;

    std::ostringstream grid_ids_ss;
    for (size_t i = 0; i < grid_ids.size() && i < 8; ++i)
        grid_ids_ss << grid_ids[i] << (i + 1 < grid_ids.size() && i + 1 < 8 ? "," : "");
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[findGridAndFrontierPath][d%d] grid_ids=%zu [%s]", ep_->drone_id_, grid_ids.size(),
                grid_ids_ss.str().c_str());

    double grid_time = (node_->now() - t1).seconds();

    // Frontier-based single drone tour planning
    // Restrict frontier within the first visited grid
    t1 = node_->now();

    vector<int> ftr_ids;
    // uniform_grid_->getFrontiersInGrid(ego_ids[0], ftr_ids);
    hgrid_->getFrontiersInGrid(ego_ids, ftr_ids);
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "Find frontier tour, %zu involved------------", ftr_ids.size());

    if (ftr_ids.empty())
    {
        const double local_radius = GetMaxLocalGoalDist(*ep_);
        if (pickNearestFrontierWithinRadius(cur_pos, local_radius, frontier_ids))
        {
            RCLCPP_INFO(rclcpp::get_logger("exploration_manager"),
                        "[findGridAndFrontierPath][d%d] no frontier in assigned grids; using nearest frontier within %.1fm", ep_->drone_id_, local_radius);
        }
        else if (recoverStaleGridAssignment(cur_pos, grid_ids, frontier_ids))
        {
            // logged inside recoverStaleGridAssignment
        }
        else
        {
            frontier_ids = {};
        }
        return;
    }

    // Consider next grid in frontier tour planning
    Eigen::Vector3d         grid_pos;
    double                  grid_yaw;
    vector<Eigen::Vector3d> grid_pos_vec;
    if (hgrid_->getNextGrid(ego_ids, grid_pos, grid_yaw))
    {
        grid_pos_vec = { grid_pos };
    }

    findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr_ids, grid_pos_vec, frontier_ids);
    // ★ MTSP 路径注入速度预测排斥：若 tour 前部有 peer 正飞向的 frontier，向前重排一个低冲突的
    {
        const double                 kPredHorTour = 5.0;
        const double                 now_sec_tour = node_->now().seconds();
        std::vector<Eigen::Vector3d> tour_peer_preds;
        for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
        {
            if (i + 1 == ep_->drone_id_)
                continue;
            const auto& st = ed_->swarm_state_[i];
            if (now_sec_tour - st.stamp_ > 2.0)
                continue;
            if ((st.pos_.head<2>() - cur_pos.head<2>()).norm() > 30.0)
                continue;
            if (st.vel_.head<2>().norm() < 0.3)
                continue;
            tour_peer_preds.push_back(st.pos_ + st.vel_ * kPredHorTour);
        }
        if (!tour_peer_preds.empty() && frontier_ids.size() > 1)
        {
            int       best_front  = 0;
            double    min_penalty = std::numeric_limits<double>::infinity();
            const int scan_win    = std::min(6, static_cast<int>(frontier_ids.size()));
            for (int j = 0; j < scan_win; ++j)
            {
                const int fid = frontier_ids[j];
                if (fid < 0 || fid >= static_cast<int>(ed_->points_.size()))
                    continue;
                double penalty = 0.0;
                for (const auto& pp : tour_peer_preds)
                {
                    const double d_pred = (ed_->points_[fid].head<2>() - pp.head<2>()).norm();
                    if (d_pred < 12.0)
                        penalty += 10.0;
                    else if (d_pred < 20.0)
                        penalty += 4.0;
                }
                if (penalty < min_penalty)
                {
                    min_penalty = penalty;
                    best_front  = j;
                }
            }
            if (best_front > 0)
            {
                const int best_fid = frontier_ids[best_front];
                for (int j = best_front; j > 0; --j)
                    frontier_ids[j] = frontier_ids[j - 1];
                frontier_ids[0] = best_fid;
                RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[findGridAndFrontierPath][d%d] MTSP vel-pred reorder: front %d→0 penalty %.1f→%.1f",
                            ep_->drone_id_, best_front, min_penalty, min_penalty);
            }
        }
    }
    double             ftr_time = (node_->now() - t1).seconds();
    std::ostringstream ftr_ids_ss;
    for (size_t i = 0; i < frontier_ids.size() && i < 8; ++i)
        ftr_ids_ss << frontier_ids[i] << (i + 1 < frontier_ids.size() && i + 1 < 8 ? "," : "");
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[findGridAndFrontierPath][d%d] frontier_ids=%zu [%s]", ep_->drone_id_, frontier_ids.size(),
                ftr_ids_ss.str().c_str());
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "Grid tour t: %lf, frontier tour t: %lf.", grid_time, ftr_time);
}

bool FastExplorationManager::shortenPath(std::vector<Eigen::Vector3d>& path, const std::function<bool(const Vector3d&, const Vector3d&)>* seg_free_override)
{
    if (path.size() < 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "%s", "Path too short to shorten");
        return false;
    }

    auto segFree = [this, seg_free_override](const Vector3d& a, const Vector3d& b) {
        if (seg_free_override && *seg_free_override)
            return (*seg_free_override)(a, b);
        return segmentCollisionFreeLine(a, b);
    };

    const double vmax               = ViewNode::vm_ > 0.1 ? ViewNode::vm_ : 1.0;
    const double angle_thresh       = (ep_->snn_path_shorten_angle_deg_ > 0.0 ? ep_->snn_path_shorten_angle_deg_ : 15.0) * M_PI / 180.0;
    const double max_segment_length = ep_->snn_path_shorten_max_segment_ > 1e-3 ? ep_->snn_path_shorten_max_segment_ : std::max(1.0, std::min(2.5, vmax * 1.5));
    const double eps                = 1e-6;

    std::vector<Eigen::Vector3d> short_path;
    short_path.reserve(path.size());
    short_path.push_back(path.front());

    for (int i = 1; i < path.size() - 1; ++i)
    {
        Eigen::Vector3d p_prev = short_path.back();
        Eigen::Vector3d p_curr = path[i];
        Eigen::Vector3d p_next = path[i + 1];

        Eigen::Vector3d v1 = p_curr - p_prev;
        Eigen::Vector3d v2 = p_next - p_curr;

        double len1 = v1.norm();
        double len2 = v2.norm();

        if (len1 < eps || len2 < eps)
            continue;

        // 计算夹角
        double cos_angle = v1.dot(v2) / (len1 * len2);
        cos_angle        = std::max(-1.0, std::min(1.0, cos_angle));
        double angle     = std::acos(cos_angle);

        bool need_keep = false;

        // ===== 条件1：转角大，必须保留 =====
        if (angle > angle_thresh)
            need_keep = true;

        // ===== 条件2：段过长，必须保留 =====
        if ((p_next - p_prev).norm() > max_segment_length)
            need_keep = true;

        // ===== 条件3：XY 柱投影碰撞（与 SNN 2D 栅格一致，避免固定 Z 射线穿墙）=====
        if (!need_keep && !segFree(p_prev, p_next))
            need_keep = true;

        if (need_keep)
            short_path.push_back(p_curr);
    }

    // 加终点
    if ((short_path.back() - path.back()).norm() > 1e-3)
        short_path.push_back(path.back());

    // ===== 保证至少三个点（minimum jerk更稳定）=====
    if (short_path.size() < 2)
    {
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[shortenPath] collapsed to %zu point(s); keep caller path unchanged, return failure.",
                    short_path.size());
        return false;
    }

    if (short_path.size() == 2)
    {
        EnsureMinThreePathPoints(short_path);
    }

    path = short_path;
    return true;
}

bool FastExplorationManager::segmentCollisionFreeLine(const Vector3d& a, const Vector3d& b) const
{
    if (!edt_environment_ || !edt_environment_->sdf_map_)
        return false;
    return edt_environment_->sdf_map_->isSegmentColumnFree(a, b);
}

void FastExplorationManager::simplifyPathSafeRdp(std::vector<Vector3d>& path, double epsilon)
{
    if (path.size() < 3 || epsilon <= 1e-9)
        return;

    const size_t n       = path.size();
    auto         segFree = [this](const Vector3d& p, const Vector3d& q) { return this->segmentCollisionFreeLine(p, q); };

    std::vector<int> idx = RdpPathIndicesImpl(path, 0, static_cast<int>(n) - 1, epsilon, segFree);
    if (idx.size() < 2)
        return;

    std::vector<Vector3d> out;
    out.reserve(idx.size());
    for (int i : idx)
        out.push_back(path[static_cast<size_t>(i)]);
    EnsureMinThreePathPoints(out);
    path.swap(out);
}

void FastExplorationManager::simplifyPathSegmentedRdp(std::vector<Vector3d>& path, double epsilon)
{
    if (path.size() < 4 || epsilon <= 1e-9)
    {
        simplifyPathSafeRdp(path, epsilon);
        return;
    }

    constexpr double          kSplitAngleRad = 22.0 * M_PI / 180.0;
    const std::vector<size_t> splits         = FindCornerSplitIndices(path, kSplitAngleRad);
    if (splits.size() <= 2)
    {
        simplifyPathSafeRdp(path, epsilon);
        return;
    }

    std::vector<Vector3d> merged;
    for (size_t s = 0; s + 1 < splits.size(); ++s)
    {
        std::vector<Vector3d> seg(path.begin() + static_cast<std::ptrdiff_t>(splits[s]), path.begin() + static_cast<std::ptrdiff_t>(splits[s + 1]) + 1);
        simplifyPathSafeRdp(seg, epsilon);
        if (merged.empty())
            merged.insert(merged.end(), seg.begin(), seg.end());
        else if (!seg.empty())
            merged.insert(merged.end(), seg.begin() + 1, seg.end());
    }
    DedupeConsecutivePath(merged, 0.04);
    if (merged.size() < 2)
    {
        simplifyPathSafeRdp(path, epsilon);
        return;
    }
    EnsureMinThreePathPoints(merged);
    path.swap(merged);
}

void FastExplorationManager::capPathWaypointsRdp(std::vector<Vector3d>& path, double base_eps, double max_eps, size_t max_pts)
{
    if (path.size() <= max_pts || max_pts < 2)
        return;

    const size_t before = path.size();
    double       eps    = std::max(base_eps, 1e-3);
    max_eps             = std::max(max_eps, eps);

    auto simplify_step = [this](std::vector<Vector3d>& p, double e) {
        if (p.size() > 8)
            this->simplifyPathSegmentedRdp(p, e);
        else
            this->simplifyPathSafeRdp(p, e);
    };

    for (int iter = 0; iter < 10 && path.size() > max_pts; ++iter)
    {
        simplify_step(path, eps);
        if (path.size() <= max_pts)
            break;
        eps = std::min(max_eps, eps * 1.3);
    }

    for (int iter = 0; iter < 4 && path.size() > max_pts; ++iter)
        simplify_step(path, max_eps * (1.0 + 0.15 * static_cast<double>(iter)));

    if (path.size() > max_pts)
    {
        std::vector<Vector3d> decimated;
        DecimatePathKeepCorners(path, max_pts, decimated);
        path.swap(decimated);
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[capPathWaypointsRdp][d%d] corner-preserving decimate: pts %zu -> %zu (max=%zu)",
                    ep_->drone_id_, before, path.size(), max_pts);
    }
    else if (before > max_pts)
    {
        RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[capPathWaypointsRdp][d%d] pts %zu -> %zu (max=%zu)", ep_->drone_id_, before, path.size(),
                    max_pts);
    }
}

void FastExplorationManager::simplifySnnPathForTraj(std::vector<Vector3d>& path, size_t max_pts)
{
    if (path.size() < 2)
        return;

    const size_t before = path.size();

    if (ep_->snn_path_resample_spacing_ <= 1e-6)
    {
        const double rdp_epsilon = ep_->snn_path_rdp_epsilon_;
        const double rdp_max_eps = ep_->snn_path_rdp_max_epsilon_;
        if (rdp_epsilon <= 1e-9)
        {
            CapPathKeepCorners(path, max_pts);
            return;
        }
        if (path.size() > 8)
            simplifyPathSegmentedRdp(path, rdp_epsilon);
        else
            simplifyPathSafeRdp(path, rdp_epsilon);
        capPathWaypointsRdp(path, rdp_epsilon, rdp_max_eps, max_pts);
        RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[planTrajToView][d%d] snn RDP: pts %zu -> %zu (eps=%.3f~%.3f m)", ep_->drone_id_, before,
                    path.size(), rdp_epsilon, rdp_max_eps);
        return;
    }

    const double vmax       = ViewNode::vm_ > 0.1 ? ViewNode::vm_ : 1.0;
    double       spacing    = ep_->snn_path_resample_spacing_ > 1e-6 ? ep_->snn_path_resample_spacing_ : std::max(0.35, std::min(0.8, 0.5 * vmax));
    const double corner_rad = (ep_->snn_path_shorten_angle_deg_ > 0.0 ? ep_->snn_path_shorten_angle_deg_ : 15.0) * M_PI / 180.0;
    ResamplePathArcLengthPreserveCorners(path, spacing, corner_rad);
    CapPathKeepCorners(path, max_pts);
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[planTrajToView][d%d] snn arc-resample: pts %zu -> %zu (spacing=%.2f m, max=%zu)", ep_->drone_id_,
                before, path.size(), spacing, max_pts);
}

void FastExplorationManager::findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, vector<int>& indices)
{
    auto t1 = node_->now();

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
    const int dimension = cost_mat.rows();
    std::cout << "mat:   " << cost_mat.rows() << std::endl;

    double mat_time = (node_->now() - t1).seconds();
    t1              = node_->now();

    // Initialize TSP par file
    ofstream par_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par");
    par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp\n";
    par_file << "GAIN23 = NO\n";
    par_file << "OUTPUT_TOUR_FILE ="
             << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_)
                    + ".tou"
                      "r\n";
    par_file << "RUNS = 1\n";
    par_file.close();

    // Write params and cost matrix to problem file
    ofstream prob_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp");
    // Problem specification part, follow the format of TSPLIB
    string prob_spec;
    prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension)
                + "\nEDGE_WEIGHT_TYPE : "
                  "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
    prob_file << prob_spec;
    // prob_file << "TYPE : TSP\n";
    // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
    // Problem data part
    const int scale = 100;
    for (int i = 0; i < dimension; ++i)
    {
        for (int j = 0; j < dimension; ++j)
        {
            int int_cost = cost_mat(i, j) * scale;
            prob_file << int_cost << " ";
        }
        prob_file << "\n";
    }
    prob_file << "EOF";
    prob_file.close();

    // solveTSPLKH((ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par").c_str());
    // Call TSP service synchronously using a separate executor to avoid blocking main executor
    const string par_path = ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par";
    if (runExternalLkhSolver(ep_->lkh_binary_, par_path) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "[TSP][d%d] LKH solver failed, par=%s", ep_->drone_id_, par_path.c_str());
        return;
    }

    const string tour_path = ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tour";
    ifstream     res_file(tour_path);
    if (!res_file.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "[TSP][d%d] Failed to open tour file: %s", ep_->drone_id_, tour_path.c_str());
        return;
    }

    string res;
    bool   tour_section_found = false;
    while (getline(res_file, res))
    {
        if (res.compare("TOUR_SECTION") == 0)
        {
            tour_section_found = true;
            break;
        }
    }
    if (!tour_section_found)
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "[TSP][d%d] Missing TOUR_SECTION in tour file.", ep_->drone_id_);
        return;
    }

    while (getline(res_file, res))
    {
        try
        {
            int id = stoi(res);
            if (id == 1)
                continue;
            if (id == -1)
                break;
            indices.push_back(id - 2);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "[TSP][d%d] Error parsing tour file: %s", ep_->drone_id_, e.what());
            indices.clear();
            return;
        }
    }

    res_file.close();

    std::cout << "Tour " << ep_->drone_id_ << ": ";
    for (auto id : indices)
        std::cout << id << ", ";
    std::cout << "" << std::endl;

    // Get the path of optimal tour from path matrix
    frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);

    double tsp_time = (node_->now() - t1).seconds();
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "Cost mat: %lf, TSP: %lf", mat_time, tsp_time);

    // if (tsp_time > 0.1) ROS_BREAK();
}

/**
 * @brief 优化局部巡游路径
 * @param
 *      输入： 当前位置、速度、yaw、候选视点集、候选航向集
 *      输出： 优化的视点位置列表、 对应的航向角列表、完整的巡游路径
 */
void FastExplorationManager::refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
                                             const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts,
                                             vector<double>& refined_yaws)
{
    double create_time, search_time, parse_time;
    auto   t1 = node_->now();

    // Create graph for viewpoints selection
    GraphSearch<ViewNode> g_search;
    vector<ViewNode::Ptr> last_group, cur_group;

    // Add the current state
    ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
    first->vel_ = cur_vel;
    g_search.addNode(first);
    last_group.push_back(first);
    ViewNode::Ptr final_node;

    // Add viewpoints
    std::cout << "Local refine graph size: 1, ";
    for (int i = 0; i < n_points.size(); ++i)
    {
        // Create nodes for viewpoints of one frontier
        for (int j = 0; j < n_points[i].size(); ++j)
        {
            ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
            g_search.addNode(node);
            // Connect a node to nodes in last group
            for (auto nd : last_group)
                g_search.addEdge(nd->id_, node->id_);
            cur_group.push_back(node);

            // Only keep the first viewpoint of the last local frontier
            if (i == n_points.size() - 1)
            {
                final_node = node;
                break;
            }
        }
        // Store nodes for this group for connecting edges
        std::cout << cur_group.size() << ", ";
        last_group = cur_group;
        cur_group.clear();
    }
    std::cout << "" << std::endl;
    create_time = (node_->now() - t1).seconds();
    t1          = node_->now();

    // Search optimal sequence
    vector<ViewNode::Ptr> path;
    g_search.DijkstraSearch(first->id_, final_node->id_, path);

    search_time = (node_->now() - t1).seconds();
    t1          = node_->now();

    // Return searched sequence
    for (int i = 1; i < path.size(); ++i)
    {
        refined_pts.push_back(path[i]->pos_);
        refined_yaws.push_back(path[i]->yaw_);
    }

    ed_->refined_tour_.clear();
    ed_->refined_tour_.push_back(cur_pos);
    ViewNode::astar_->lambda_heu_ = 1.0;
    ViewNode::astar_->setResolution(0.2);

    // A*在这里的作用就是将离散的点生成轨迹
    for (auto pt : refined_pts)
    {
        vector<Vector3d> path;
        if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
            ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
        else
            ed_->refined_tour_.push_back(pt);
    }
    ViewNode::astar_->lambda_heu_ = 10000;

    parse_time = (node_->now() - t1).seconds();
    // RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

void FastExplorationManager::ReduceGridsForPairMtsp(const vector<Eigen::Vector3d>& positions, const vector<vector<int>>& first_ids,
                                                    const vector<vector<int>>& second_ids, vector<int>& grid_ids_inout,
                                                    std::unordered_map<int, vector<int>>& cluster_expand_out, vector<vector<int>>& dropped_clusters_out)
{
    cluster_expand_out.clear();
    dropped_clusters_out.clear();

    const int max_rep = alloc_cfg_.mtsp_max_rep_grids_;
    if (grid_ids_inout.empty())
        return;

    if (max_rep <= 0 || static_cast<int>(grid_ids_inout.size()) <= max_rep)
    {
        for (int id : grid_ids_inout)
            cluster_expand_out[id] = { id };
        return;
    }

    const double merge_r = alloc_cfg_.mtsp_cluster_merge_radius_m_;
    const double near_r  = alloc_cfg_.mtsp_near_radius_m_;

    std::unordered_set<int> id_set(grid_ids_inout.begin(), grid_ids_inout.end());
    vector<int>             ids(id_set.begin(), id_set.end());
    std::sort(ids.begin(), ids.end());
    if (ids.empty())
    {
        grid_ids_inout.clear();
        return;
    }

    const int               n_ids = static_cast<int>(ids.size());
    vector<Eigen::Vector3d> centers;
    centers.reserve(static_cast<size_t>(n_ids));
    for (int id : ids)
        centers.push_back(hgrid_->getCenter(id));

    vector<int> parent(static_cast<size_t>(n_ids));
    std::iota(parent.begin(), parent.end(), 0);
    auto dsu_find = [&](int x) {
        while (parent[static_cast<size_t>(x)] != x)
            x = parent[static_cast<size_t>(x)];
        return x;
    };
    auto dsu_union = [&](int a, int b) {
        a = dsu_find(a);
        b = dsu_find(b);
        if (a != b)
            parent[static_cast<size_t>(a)] = b;
    };

    if (merge_r > 1e-6 && n_ids > 1)
    {
        const double merge_r2 = merge_r * merge_r;
        for (int i = 0; i < n_ids; ++i)
        {
            for (int j = i + 1; j < n_ids; ++j)
            {
                if ((centers[static_cast<size_t>(i)] - centers[static_cast<size_t>(j)]).squaredNorm() <= merge_r2)
                    dsu_union(i, j);
            }
        }
    }

    std::unordered_map<int, vector<int>> root_to_members;
    for (int i = 0; i < n_ids; ++i)
    {
        const int r = dsu_find(i);
        root_to_members[r].push_back(ids[static_cast<size_t>(i)]);
    }
    vector<vector<int>> clusters;
    clusters.reserve(root_to_members.size());
    for (auto& kv : root_to_members)
    {
        auto& cl = kv.second;
        std::sort(cl.begin(), cl.end());
        clusters.push_back(std::move(cl));
    }
    std::sort(clusters.begin(), clusters.end(), [](const vector<int>& a, const vector<int>& b) {
        if (a.empty() || b.empty())
            return a.size() < b.size();
        return a.front() < b.front();
    });

    auto pick_rep = [&](const vector<int>& cl) -> int {
        int best_id = cl.front();
        int best_un = -1;
        for (int gid : cl)
        {
            const int u = hgrid_->getUnknownCellsNum(gid);
            if (u > best_un || (u == best_un && gid < best_id))
            {
                best_un = u;
                best_id = gid;
            }
        }
        return best_id;
    };

    std::unordered_set<int> mandatory_ids;
    auto                    add_mandatory = [&](const vector<int>& v) {
        for (int gid : v)
        {
            if (id_set.count(gid) != 0u)
                mandatory_ids.insert(gid);
        }
    };
    for (const auto& v : first_ids)
        add_mandatory(v);
    for (const auto& v : second_ids)
        add_mandatory(v);

    struct ClusterInfo
    {
        vector<int>     members;
        int             rep{ -1 };
        Eigen::Vector3d centroid{ 0, 0, 0 };
        bool            mandatory{ false };
        double          min_drone_dist{ 0.0 };
    };
    vector<ClusterInfo> infos;
    infos.reserve(clusters.size());
    for (const auto& cl : clusters)
    {
        ClusterInfo ci;
        ci.members = cl;
        ci.rep     = pick_rep(cl);
        for (int gid : cl)
            ci.centroid += hgrid_->getCenter(gid);
        ci.centroid /= static_cast<double>(std::max(1, static_cast<int>(cl.size())));

        for (int mid : cl)
        {
            if (mandatory_ids.count(mid) != 0u)
            {
                ci.mandatory = true;
                break;
            }
        }
        if (!ci.mandatory && !positions.empty())
        {
            const double near_r2 = near_r * near_r;
            for (int gid : cl)
            {
                const Eigen::Vector3d c = hgrid_->getCenter(gid);
                for (const auto& p : positions)
                {
                    if ((c - p).squaredNorm() <= near_r2)
                    {
                        ci.mandatory = true;
                        break;
                    }
                }
                if (ci.mandatory)
                    break;
            }
        }

        ci.min_drone_dist = std::numeric_limits<double>::infinity();
        for (const auto& p : positions)
            ci.min_drone_dist = std::min(ci.min_drone_dist, (ci.centroid - p).norm());

        infos.push_back(std::move(ci));
    }

    vector<int> mandatory_reps;
    vector<int> optional_reps;
    mandatory_reps.reserve(infos.size());
    optional_reps.reserve(infos.size());
    std::unordered_map<int, double> rep_min_dist;
    for (const auto& ci : infos)
    {
        rep_min_dist[ci.rep] = ci.min_drone_dist;
        if (ci.mandatory)
            mandatory_reps.push_back(ci.rep);
        else
            optional_reps.push_back(ci.rep);
    }
    std::sort(mandatory_reps.begin(), mandatory_reps.end());
    std::sort(optional_reps.begin(), optional_reps.end(), [&](int a, int b) {
        const double da = rep_min_dist[a];
        const double db = rep_min_dist[b];
        if (std::abs(da - db) > 1e-6)
            return da < db;
        return a < b;
    });

    vector<int> selected_reps;
    auto        push_unique = [&](int r) {
        if (std::find(selected_reps.begin(), selected_reps.end(), r) == selected_reps.end())
            selected_reps.push_back(r);
    };
    for (int r : mandatory_reps)
        push_unique(r);

    static double s_last_mandatory_overflow_warn_sec = -1e9;
    if (static_cast<int>(selected_reps.size()) > max_rep)
    {
        const double now_sec = node_->now().seconds();
        if (now_sec - s_last_mandatory_overflow_warn_sec > 10.0)
        {
            s_last_mandatory_overflow_warn_sec = now_sec;
            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                        "[Allocate][d%d] MTSP grid reduction: mandatory reps %d exceed mtsp_max_rep_grids=%d; using all mandatory (LKH may be heavy).",
                        ep_->drone_id_, static_cast<int>(selected_reps.size()), max_rep);
        }
    }
    else
    {
        for (int r : optional_reps)
        {
            if (static_cast<int>(selected_reps.size()) >= max_rep)
                break;
            push_unique(r);
        }
    }
    std::sort(selected_reps.begin(), selected_reps.end());

    const std::unordered_set<int> selected_set(selected_reps.begin(), selected_reps.end());
    for (const auto& ci : infos)
    {
        if (selected_set.count(ci.rep) != 0u)
        {
            vector<int> expanded = ci.members;
            std::sort(expanded.begin(), expanded.end());
            vector<int> ordered;
            ordered.push_back(ci.rep);
            for (int gid : expanded)
            {
                if (gid != ci.rep)
                    ordered.push_back(gid);
            }
            cluster_expand_out[ci.rep] = std::move(ordered);
        }
        else
            dropped_clusters_out.push_back(ci.members);
    }

    grid_ids_inout = std::move(selected_reps);
}

void FastExplorationManager::AllocateGridsMaybeDynamicRelayout(const vector<int>& grid_ids, int drone_num, int total_regions_cap)
{
    auto& uf = PairAllocateGridsShared();

    if (!alloc_cfg_.enable_dynamic_region_relayout_)
        return;

    static double last_relayout_sec            = -1.0;
    static bool   last_relayout_was_unfinished = false;
    const double  now_t_sec                    = node_->now().seconds();
    const double  base_cooldown                = std::max(0.0, alloc_cfg_.dynamic_region_relayout_cooldown_);
    const double  effective_cooldown           = last_relayout_was_unfinished ? std::max(base_cooldown * 2.0, base_cooldown + 1.0) : base_cooldown;
    if (last_relayout_sec < 0.0 || (now_t_sec - last_relayout_sec) > effective_cooldown)
    {
        constexpr int kStateFinish   = 5;
        constexpr int kStateWaitBack = 8;

        auto isExplorationOnline = [&](int did, const DroneState& st) -> bool {
            if (did != ep_->drone_id_ && (now_t_sec - st.stamp_) > alloc_cfg_.dynamic_region_online_timeout_)
                return false;
            if (st.cur_state_ == kStateFinish || st.cur_state_ == kStateWaitBack)
                return false;
            return true;
        };

        int online_count = 0;
        for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
        {
            const int         did = i + 1;
            const DroneState& st  = ed_->swarm_state_[i];
            if (isExplorationOnline(did, st))
                online_count++;
        }

        static int s_last_online_observed = -1;
        static int s_online_same_count    = 0;
        static int s_online_stable        = -1;
        if (online_count == s_last_online_observed)
            s_online_same_count++;
        else
        {
            s_last_online_observed = online_count;
            s_online_same_count    = 1;
        }
        if (s_online_stable < 0)
            s_online_stable = online_count;
        if (s_online_same_count >= std::max(1, alloc_cfg_.dynamic_region_online_stable_cycles_))
            s_online_stable = online_count;

        std::unordered_set<int> unfinished_region_set;
        if (!ed_->averages_.empty())
        {
            for (const auto& p : ed_->averages_)
            {
                const int reg = std::max(0, std::min(region_allocator_->ownerRegionOfPoint(p), total_regions_cap - 1));
                unfinished_region_set.insert(reg);
            }
        }
        else
        {
            for (int gid : grid_ids)
            {
                const int reg = std::max(0, std::min(region_allocator_->ownerRegionOfGridCenter(hgrid_->getCenter(gid)), total_regions_cap - 1));
                unfinished_region_set.insert(reg);
            }
        }

        unsigned long long unfinished_mask = 0ULL;
        for (int reg : unfinished_region_set)
        {
            if (reg >= 0 && reg < 63)
                unfinished_mask |= (1ULL << reg);
        }
        if (!uf.unfinished_inited)
        {
            uf.unfinished_inited       = true;
            uf.unfinished_obs_mask     = unfinished_mask;
            uf.unfinished_stable_mask  = unfinished_mask;
            uf.unfinished_applied_mask = unfinished_mask;
            uf.unfinished_same_count   = 1;
        }
        else
        {
            if (unfinished_mask == uf.unfinished_obs_mask)
                uf.unfinished_same_count++;
            else
            {
                uf.unfinished_obs_mask   = unfinished_mask;
                uf.unfinished_same_count = 1;
            }
            if (uf.unfinished_same_count >= std::max(1, alloc_cfg_.dynamic_region_unfinished_stable_cycles_))
                uf.unfinished_stable_mask = uf.unfinished_obs_mask;
        }
        const bool unfinished_changed   = (uf.unfinished_stable_mask != uf.unfinished_applied_mask);
        const bool unfinished_is_active = !unfinished_region_set.empty();
        const bool layout_is_hotspot = unfinished_is_active && uf.unfinished_same_count >= std::max(2, alloc_cfg_.dynamic_region_unfinished_stable_cycles_ + 1);
        const bool layout_is_stuck   = unfinished_is_active && uf.unfinished_same_count >= std::max(1, alloc_cfg_.dynamic_region_unfinished_stable_cycles_)
                                     && s_online_same_count >= std::max(1, alloc_cfg_.dynamic_region_online_stable_cycles_);

        int target_regions = std::min(total_regions_cap, std::max(1, s_online_stable));
        target_regions     = std::max(std::min(drone_num, total_regions_cap), target_regions);

        const int  current_regions               = region_allocator_->layout().total_regions;
        const bool relayout_by_count             = (target_regions != current_regions);
        const bool relayout_by_unfinished_change = unfinished_changed && unfinished_is_active && (layout_is_hotspot || layout_is_stuck);
        const bool relayout_requested            = relayout_by_count || relayout_by_unfinished_change;
        if (relayout_requested)
        {
            double bmin_x = -x_size * 0.5, bmax_x = x_size * 0.5;
            double bmin_y = -y_size * 0.5, bmax_y = y_size * 0.5;
            if (alloc_cfg_.dynamic_region_bounds_use_frontier_aabb_ && !ed_->averages_.empty())
            {
                bmin_x = std::numeric_limits<double>::infinity();
                bmin_y = std::numeric_limits<double>::infinity();
                bmax_x = -std::numeric_limits<double>::infinity();
                bmax_y = -std::numeric_limits<double>::infinity();
                for (const auto& p : ed_->averages_)
                {
                    bmin_x = std::min(bmin_x, p.x());
                    bmax_x = std::max(bmax_x, p.x());
                    bmin_y = std::min(bmin_y, p.y());
                    bmax_y = std::max(bmax_y, p.y());
                }
                const double margin = std::max(0.0, alloc_cfg_.dynamic_region_bounds_margin_m_);
                bmin_x -= margin;
                bmax_x += margin;
                bmin_y -= margin;
                bmax_y += margin;
                const double min_span = std::max(1.0, alloc_cfg_.dynamic_region_bounds_min_span_m_);
                if (bmax_x - bmin_x < min_span)
                {
                    const double c = 0.5 * (bmax_x + bmin_x);
                    bmin_x         = c - 0.5 * min_span;
                    bmax_x         = c + 0.5 * min_span;
                }
                if (bmax_y - bmin_y < min_span)
                {
                    const double c = 0.5 * (bmax_y + bmin_y);
                    bmin_y         = c - 0.5 * min_span;
                    bmax_y         = c + 0.5 * min_span;
                }
                bmin_x = std::max(-x_size * 0.5, bmin_x);
                bmax_x = std::min(x_size * 0.5, bmax_x);
                bmin_y = std::max(-y_size * 0.5, bmin_y);
                bmax_y = std::min(y_size * 0.5, bmax_y);
            }
            region_allocator_->reconfigureDynamic(target_regions, bmin_x, bmax_x, bmin_y, bmax_y);
            last_relayout_sec            = now_t_sec;
            last_relayout_was_unfinished = relayout_by_unfinished_change;
            uf.unfinished_applied_mask   = uf.unfinished_stable_mask;
            const char* relayout_reason  = (relayout_by_count && relayout_by_unfinished_change) ? "region_count+unfinished_layout"
                                                                                                : (relayout_by_count ? "region_count" : "unfinished_layout");
            // RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[Allocate][d%d] Dynamic relayout: reason=%s online_drones=%d regions %d->%d
            // cooldown=%.2f",
            //             ep_->drone_id_, relayout_reason, s_online_stable, current_regions, target_regions, effective_cooldown);
        }
    }
}

void FastExplorationManager::AllocateGridsMaybePeriodicGlobalRegionAssignment(const vector<Eigen::Vector3d>& positions)
{
    if (!alloc_cfg_.enable_periodic_global_region_assignment_)
        return;

    static double last_global_assign_sec = -1.0;
    const double  now_t_sec              = node_->now().seconds();
    if (last_global_assign_sec < 0.0 || (now_t_sec - last_global_assign_sec) > std::max(0.1, alloc_cfg_.periodic_global_region_assign_sec_))
    {
        constexpr int kStateFinish   = 5;
        constexpr int kStateWaitBack = 8;

        std::vector<int>             online_ids;
        std::vector<Eigen::Vector3d> online_pos;
        online_ids.reserve(ed_->swarm_state_.size());
        online_pos.reserve(ed_->swarm_state_.size());
        for (int i = 0; i < static_cast<int>(ed_->swarm_state_.size()); ++i)
        {
            const int         did = i + 1;
            const DroneState& st  = ed_->swarm_state_[i];
            if (did != ep_->drone_id_ && (now_t_sec - st.stamp_) > alloc_cfg_.dynamic_region_online_timeout_)
                continue;
            if (st.cur_state_ == kStateFinish || st.cur_state_ == kStateWaitBack)
                continue;
            online_ids.push_back(did);
            online_pos.push_back(st.pos_);
        }
        for (int i = 0; i < static_cast<int>(online_ids.size()); ++i)
        {
            if (online_ids[i] == ep_->drone_id_ && !positions.empty())
            {
                online_pos[i] = positions[0];
                break;
            }
        }
        if (static_cast<int>(online_ids.size()) >= std::max(1, alloc_cfg_.periodic_global_assign_min_online_))
        {
            // 1. Reach consensus on the current assignment so all drones start SNN
            //    from the same sticky state.
            ApplyRegionAssignmentConsensus();

            // 1b. Compute per-region active grid counts for capacity-normalized load.
            {
                const int        n_regions = region_allocator_->layout().total_regions;
                std::vector<int> active_counts(n_regions, 0);
                std::vector<int> reachable_counts(n_regions, 0);
                std::vector<int> all_active;
                hgrid_->getActiveGrids(all_active);
                UpdateReachabilityMask(online_pos.empty() ? positions.front() : online_pos.front());
                for (const int gid : all_active)
                {
                    if (gid < 0)
                        continue;
                    const int reg = region_allocator_->ownerRegionOfGridCenter(hgrid_->getCenter(gid));
                    if (reg >= 0 && reg < n_regions)
                    {
                        ++active_counts[reg];
                        if (IsReachablePoint(hgrid_->getCenter(gid)))
                            ++reachable_counts[reg];
                    }
                }
                region_allocator_->setRegionActiveGridCounts(active_counts);
                region_allocator_->setRegionReachableGridCounts(reachable_counts);
            }

            //* 2. 调用SNN区域分配的地方
            std::vector<int>     result = region_allocator_->computeDroneRegions(online_ids, online_pos);
            std::vector<int>     candidate(ep_->drone_num_, -1);
            std::vector<uint8_t> online_mask(ep_->drone_num_, 0);
            for (size_t i = 0; i < online_ids.size(); ++i)
            {
                const int did = online_ids[i];
                if (did >= 1 && did <= ep_->drone_num_)
                {
                    candidate[did - 1]   = region_allocator_->layout().total_regions > 0
                                               ? std::max(0, std::min(result[i], region_allocator_->layout().total_regions - 1))
                                               : result[i];
                    online_mask[did - 1] = 1;
                }
            }
            if (!IsCompleteRegionAssignment(candidate, online_mask))
            {
                RCLCPP_WARN(node_->get_logger(), "[SnnRegion] reject incomplete/conflicting global proposal online=%zu", online_ids.size());
                return;
            }
            {
                const int n_online  = static_cast<int>(std::count(online_mask.begin(), online_mask.end(), static_cast<uint8_t>(1)));
                const int n_regions = region_allocator_->layout().total_regions;
                if (n_online < n_regions)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "[SnnRegion] global proposal accepted with only %d/%d regions covered (online=%zu) - vacant region(s) exist", n_online,
                                n_regions, online_ids.size());
                }
            }
            ed_->assigned_regions_             = candidate;
            ed_->assigned_regions_online_mask_ = online_mask;
            ed_->assigned_regions_stamp_       = node_->now().seconds();
            ++ed_->assigned_regions_epoch_;
            ed_->assigned_regions_sender_ = ep_->drone_id_;
            region_allocator_->applyConsensusAssignments(candidate);

            last_global_assign_sec = now_t_sec;
        }
    }
}

bool FastExplorationManager::IsCompleteRegionAssignment(const std::vector<int>& assignments, const std::vector<uint8_t>& online_mask) const
{
    const int n       = ep_ ? ep_->drone_num_ : 0;
    const int regions = region_allocator_ ? region_allocator_->layout().total_regions : 0;
    if (n <= 0 || regions <= 0 || assignments.size() != static_cast<size_t>(n) || online_mask.size() != static_cast<size_t>(n))
        return false;
    std::vector<uint8_t> used(static_cast<size_t>(regions), 0);
    bool                 any = false;
    for (int i = 0; i < n; ++i)
    {
        if (!online_mask[static_cast<size_t>(i)])
            continue;
        any         = true;
        const int r = assignments[static_cast<size_t>(i)];
        if (r < 0 || r >= regions || used[static_cast<size_t>(r)])
            return false;
        used[static_cast<size_t>(r)] = 1;
    }
    return any;
}

bool FastExplorationManager::GetGlobalRegionAssignment(int drone_id, int& region_id) const
{
    if (!ed_ || !ep_ || drone_id < 1 || drone_id > ep_->drone_num_ || ed_->assigned_regions_.size() != static_cast<size_t>(ep_->drone_num_)
        || ed_->assigned_regions_online_mask_.size() != static_cast<size_t>(ep_->drone_num_))
        return false;
    const size_t index = static_cast<size_t>(drone_id - 1);
    if (!ed_->assigned_regions_online_mask_[index] || ed_->assigned_regions_[index] < 0)
        return false;
    region_id = ed_->assigned_regions_[index];
    return true;
}

void FastExplorationManager::ApplyRegionAssignmentConsensus()
{
    const int n = ep_->drone_num_;
    if (n <= 0)
        return;

    if (static_cast<int>(ed_->assigned_regions_.size()) != n)
        ed_->assigned_regions_.assign(n, -1);
    if (static_cast<int>(ed_->assigned_regions_online_mask_.size()) != n)
        ed_->assigned_regions_online_mask_.assign(n, 0);
    if (static_cast<int>(ed_->peer_assigned_regions_.size()) != n)
        ed_->peer_assigned_regions_.assign(n, {});
    if (static_cast<int>(ed_->peer_assigned_regions_stamp_.size()) != n)
        ed_->peer_assigned_regions_stamp_.assign(n, 0.0);
    if (static_cast<int>(ed_->peer_assigned_regions_epoch_.size()) != n)
        ed_->peer_assigned_regions_epoch_.assign(n, 0);
    if (static_cast<int>(ed_->peer_assigned_regions_online_mask_.size()) != n)
        ed_->peer_assigned_regions_online_mask_.assign(n, {});

    std::vector<int>     best        = ed_->assigned_regions_;
    std::vector<uint8_t> best_mask   = ed_->assigned_regions_online_mask_;
    uint64_t             best_epoch  = ed_->assigned_regions_epoch_;
    double               best_stamp  = ed_->assigned_regions_stamp_;
    int                  best_sender = ed_->assigned_regions_sender_ > 0 ? ed_->assigned_regions_sender_ : ep_->drone_id_;

    for (int peer = 1; peer <= n; ++peer)
    {
        if (peer == ep_->drone_id_ || ed_->peer_assigned_regions_[peer - 1].size() != static_cast<size_t>(n)
            || ed_->peer_assigned_regions_online_mask_[peer - 1].size() != static_cast<size_t>(n))
            continue;
        const uint64_t epoch = ed_->peer_assigned_regions_epoch_[peer - 1];
        const double   stamp = ed_->peer_assigned_regions_stamp_[peer - 1];
        if (epoch < best_epoch || (epoch == best_epoch && stamp < best_stamp - 1e-6)
            || (epoch == best_epoch && std::abs(stamp - best_stamp) <= 1e-6 && peer >= best_sender))
            continue;
        if (!IsCompleteRegionAssignment(ed_->peer_assigned_regions_[peer - 1], ed_->peer_assigned_regions_online_mask_[peer - 1]))
            continue;
        best        = ed_->peer_assigned_regions_[peer - 1];
        best_mask   = ed_->peer_assigned_regions_online_mask_[peer - 1];
        best_epoch  = epoch;
        best_stamp  = stamp;
        best_sender = peer;
    }

    if (!IsCompleteRegionAssignment(best, best_mask))
        return;
    ed_->assigned_regions_             = best;
    ed_->assigned_regions_online_mask_ = best_mask;
    ed_->assigned_regions_epoch_       = best_epoch;
    ed_->assigned_regions_stamp_       = best_stamp;
    ed_->assigned_regions_sender_      = best_sender;
    region_allocator_->applyConsensusAssignments(best);
}

void FastExplorationManager::AllocateGridsApplyCostMatrixHeuristics(Eigen::MatrixXd& mat, const vector<Eigen::Vector3d>& positions,
                                                                    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
                                                                    const vector<vector<int>>& second_ids, const vector<int>& active_grid_ids, int drone_num,
                                                                    int grid_size, double infinite_cost, const std::vector<int>& drone_ids_for_log,
                                                                    const std::vector<int>& drone_regions)
{
    const double INFINITE_COST = infinite_cost;
    const auto&  layout        = region_allocator_->layout();
    const int    region_cols   = layout.cols;
    const int    total_regions = std::max(1, layout.total_regions);

    auto clampRegion = [&](int r) -> int {
        if (r < 0)
            return 0;
        if (r >= total_regions)
            return total_regions - 1;
        return r;
    };
    auto regionRow       = [&](int reg) -> int { return reg / std::max(1, region_cols); };
    auto regionCol       = [&](int reg) -> int { return reg % std::max(1, region_cols); };
    auto getRegionBounds = [&](int reg, double& min_x, double& max_x, double& min_y, double& max_y) {
        region_allocator_->getRegionBounds(clampRegion(reg), min_x, max_x, min_y, max_y);
    };

    auto getGridOwnerRegion = [&](int grid_id) -> int { return clampRegion(region_allocator_->ownerRegionOfGridCenter(hgrid_->getCenter(grid_id))); };

    static std::vector<double> owner_region_last_seen_sec;
    if (owner_region_last_seen_sec.size() != static_cast<size_t>(total_regions))
        owner_region_last_seen_sec.assign(static_cast<size_t>(total_regions), node_->now().seconds());
    const double      now_sec = node_->now().seconds();
    std::vector<bool> owner_region_present(static_cast<size_t>(total_regions), false);
    for (int reg : drone_regions)
    {
        if (reg >= 0 && reg < total_regions)
            owner_region_present[static_cast<size_t>(reg)] = true;
    }
    for (int reg = 0; reg < total_regions; ++reg)
    {
        if (owner_region_present[static_cast<size_t>(reg)])
            owner_region_last_seen_sec[static_cast<size_t>(reg)] = now_sec;
    }

    auto canTakeOverRegion = [&](int owner_reg, int assigned_reg) -> bool {
        if (!alloc_cfg_.enable_cross_region_takeover_ || owner_reg == assigned_reg)
            return false;
        if (owner_reg < 0 || owner_reg >= total_regions)
            return false;
        if (owner_region_present[static_cast<size_t>(owner_reg)])
            return false;
        return (now_sec - owner_region_last_seen_sec[static_cast<size_t>(owner_reg)]) >= alloc_cfg_.takeover_idle_sec_;
    };

    // ★ 空区兜底：区域在全队共识中无人认领（含认领车下线）且持续超过 takeover_idle_sec
    //   后，对该区域的格子豁免跨区惩罚。无跨区接管时，空区前沿仍可被其他车公平竞争，
    //   避免"共识缺员导致空区无人探索"永久僵持；有主区域仍强制守区。
    static std::vector<double> region_claimed_last_sec;
    if (region_claimed_last_sec.size() != static_cast<size_t>(total_regions))
        region_claimed_last_sec.assign(static_cast<size_t>(total_regions), now_sec);
    if (ed_ && ep_ && static_cast<int>(ed_->assigned_regions_.size()) == ep_->drone_num_
        && static_cast<int>(ed_->assigned_regions_online_mask_.size()) == ep_->drone_num_)
    {
        for (int i = 0; i < ep_->drone_num_; ++i)
        {
            if (!ed_->assigned_regions_online_mask_[static_cast<size_t>(i)])
                continue;
            const int reg = clampRegion(ed_->assigned_regions_[static_cast<size_t>(i)]);
            if (reg >= 0 && reg < total_regions)
                region_claimed_last_sec[static_cast<size_t>(reg)] = now_sec;
        }
    }
    auto isVacantRegion = [&](int reg) -> bool {
        if (reg < 0 || reg >= total_regions)
            return false;
        return (now_sec - region_claimed_last_sec[static_cast<size_t>(reg)]) >= alloc_cfg_.takeover_idle_sec_;
    };

    auto regionConflictHotspotScore = [&](int reg) -> int {
        if (reg < 0 || reg >= total_regions)
            return 0;
        const auto reg_count_it = std::count(drone_regions.begin(), drone_regions.end(), reg);
        const int  seen_bonus   = owner_region_present[static_cast<size_t>(reg)] ? 0 : 1;
        return static_cast<int>(reg_count_it) + seen_bonus;
    };

    // Lightweight task-cluster ownership: nearby active grids share one owner.
    // This is deliberately local to the cost matrix and never mutates region state.
    std::vector<int> grid_cluster_owner(static_cast<size_t>(grid_size), -1);
    if (alloc_cfg_.enable_grid_cluster_owner_ && grid_size > 0)
    {
        const double radius = std::max(1.0, alloc_cfg_.grid_cluster_owner_radius_m_);
        for (int seed = 0; seed < grid_size; ++seed)
        {
            if (grid_cluster_owner[static_cast<size_t>(seed)] >= 0)
                continue;
            const Eigen::Vector3d seed_center = hgrid_->getCenter(active_grid_ids[seed]);
            const int             grid_reg    = getGridOwnerRegion(active_grid_ids[seed]);
            int                   owner       = -1;
            double                best_cost   = std::numeric_limits<double>::infinity();
            for (int i = 0; i < drone_num; ++i)
            {
                if (!IsReachablePoint(seed_center))
                    continue;
                const double d           = (positions[i] - seed_center).head<2>().norm();
                const double region_bias = clampRegion(drone_regions[i]) == grid_reg ? 0.0 : alloc_cfg_.cross_region_soft_penalty_;
                if (d + region_bias < best_cost)
                {
                    best_cost = d + region_bias;
                    owner     = i;
                }
            }
            if (owner < 0)
                continue;
            for (int k = seed; k < grid_size; ++k)
            {
                if ((hgrid_->getCenter(active_grid_ids[k]) - seed_center).head<2>().norm() <= radius)
                    grid_cluster_owner[static_cast<size_t>(k)] = owner;
            }
        }
    }

    for (int i = 0; i < drone_num; ++i)
    {
        int    reg = drone_regions[i];
        double reg_min_x, reg_max_x, reg_min_y, reg_max_y;
        getRegionBounds(reg, reg_min_x, reg_max_x, reg_min_y, reg_max_y);
        for (int k = 0; k < grid_size; ++k)
        {
            int owner_reg    = getGridOwnerRegion(active_grid_ids[k]);
            int grid_mat_idx = k + drone_num;
            if (alloc_cfg_.enable_grid_cluster_owner_ && grid_cluster_owner[static_cast<size_t>(k)] >= 0 && grid_cluster_owner[static_cast<size_t>(k)] != i)
            {
                const int  owner       = grid_cluster_owner[static_cast<size_t>(k)];
                const bool owner_valid = owner >= 0 && owner < drone_num && IsReachablePoint(hgrid_->getCenter(active_grid_ids[k]));
                if (owner_valid)
                    mat(i, grid_mat_idx) = std::min(INFINITE_COST, mat(i, grid_mat_idx) + alloc_cfg_.grid_cluster_owner_penalty_);
            }
            if (owner_reg != reg && !canTakeOverRegion(owner_reg, reg) && !isVacantRegion(owner_reg))
            {
                const int   hotspot_score        = regionConflictHotspotScore(owner_reg);
                const auto& uf_ps                = PairAllocateGridsShared();
                const bool  unfinished_is_active = (uf_ps.unfinished_stable_mask != 0);
                const bool  layout_is_hotspot_local =
                    unfinished_is_active && uf_ps.unfinished_same_count >= std::max(2, alloc_cfg_.dynamic_region_unfinished_stable_cycles_ + 1);
                if (alloc_cfg_.enable_soft_cross_region_constraint_)
                {
                    const int    dr              = std::abs(regionRow(owner_reg) - regionRow(reg));
                    const int    dc              = std::abs(regionCol(owner_reg) - regionCol(reg));
                    const int    hops            = dr + dc;
                    const double penalty_scale   = 1.0 + alloc_cfg_.cross_region_penalty_gain_per_hop_ * static_cast<double>(hops);
                    double       hotspot_penalty = alloc_cfg_.cross_region_soft_penalty_ * penalty_scale;
                    if (layout_is_hotspot_local && hotspot_score >= 2)
                    {
                        hotspot_penalty *= 1.5 + 0.25 * static_cast<double>(hotspot_score - 2);
                    }
                    else if (hotspot_score >= 2)
                    {
                        hotspot_penalty *= 1.0 + 0.2 * static_cast<double>(hotspot_score - 1);
                    }
                    mat(i, grid_mat_idx) += hotspot_penalty;
                }
                else if (hotspot_score >= 3 && layout_is_hotspot_local)
                {
                    mat(i, grid_mat_idx) = INFINITE_COST;
                }
                else
                {
                    const double base_penalty = 0.5 * alloc_cfg_.cross_region_soft_penalty_;
                    mat(i, grid_mat_idx)      = std::min(INFINITE_COST, mat(i, grid_mat_idx) + base_penalty);
                }
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[Allocate] Drone %d fixed region %d (row=%d,col=%d) X:[%.2f, %.2f] Y:[%.2f, %.2f]",
                    drone_ids_for_log[i], reg, regionRow(reg), regionCol(reg), reg_min_x, reg_max_x, reg_min_y, reg_max_y);
    }

    if (drone_num > 1)
    {
        for (int i = 0; i < drone_num; ++i)
        {
            for (int j = 0; j < drone_num; ++j)
            {
                if (i == j)
                    continue;
                const double dist = (positions[i] - positions[j]).norm();

                // 方向感知的 repel 增强：
                // 两车同向运动时有效阈值放大，提前在代价矩阵中分开
                double eff_repel_threshold = alloc_cfg_.repel_threshold_;
                double eff_repel_scale     = alloc_cfg_.repel_scale_;
                if (velocities.size() > static_cast<size_t>(i) && velocities.size() > static_cast<size_t>(j))
                {
                    const double v_i_n = velocities[i].head<2>().norm();
                    const double v_j_n = velocities[j].head<2>().norm();
                    if (v_i_n > 0.2 && v_j_n > 0.2)
                    {
                        const double vel_sim = velocities[i].head<2>().normalized().dot(velocities[j].head<2>().normalized());
                        // heading 相似（同向，cos > 0.7 ≈ 45°）且距离较近时增强 repel
                        if (vel_sim > 0.7)
                        {
                            eff_repel_threshold *= 1.8;  // 同向跟随：扩大 repel 触发距离
                            eff_repel_scale *= 1.5;      // 同向跟随：repel 强度再增大
                        }
                    }
                }

                if (dist < eff_repel_threshold)
                {
                    for (int k = 0; k < grid_size; ++k)
                    {
                        double dist_to_j = (hgrid_->getCenter(active_grid_ids[k]) - positions[j]).norm();
                        if (dist_to_j < eff_repel_threshold)
                        {
                            // 基础缩放 + 同向增强
                            double scale = eff_repel_scale;
                            // 若自己仍处于分配区域之外，额外惩罚跨区争抢同片前沿
                            mat(i, k + drone_num) *= scale;
                        }
                    }
                }
            }
        }
    }

    if (drone_num > 1 && alloc_cfg_.pair_overlap_radius_ > 1e-3 && alloc_cfg_.pair_overlap_penalty_scale_ > 1.0)
    {
        for (int i = 0; i < drone_num; ++i)
        {
            for (int j = 0; j < drone_num; ++j)
            {
                if (i == j || first_ids.size() <= static_cast<size_t>(j))
                    continue;

                std::vector<int> anchors;
                if (!first_ids[j].empty())
                    anchors.push_back(first_ids[j].front());
                if (second_ids.size() > static_cast<size_t>(j) && !second_ids[j].empty())
                    anchors.push_back(second_ids[j].front());
                if (anchors.empty())
                    continue;

                const double pair_dist        = (positions[i] - positions[j]).norm();
                double       adaptive_penalty = alloc_cfg_.pair_overlap_penalty_scale_;
                if (pair_dist < alloc_cfg_.pair_overlap_radius_)
                {
                    const double ratio = std::max(0.0, (alloc_cfg_.pair_overlap_radius_ - pair_dist) / alloc_cfg_.pair_overlap_radius_);
                    adaptive_penalty *= (1.0 + 0.8 * ratio);
                }

                for (int k = 0; k < grid_size; ++k)
                {
                    Eigen::Vector3d c                  = hgrid_->getCenter(active_grid_ids[k]);
                    bool            near_partner_entry = false;
                    for (int gid : anchors)
                    {
                        if ((c - hgrid_->getCenter(gid)).norm() < alloc_cfg_.pair_overlap_radius_)
                        {
                            near_partner_entry = true;
                            break;
                        }
                    }
                    if (!near_partner_entry)
                        continue;
                    mat(i, k + drone_num) = std::min(INFINITE_COST, mat(i, k + drone_num) * adaptive_penalty);
                }
            }
        }
    }

    if (drone_num == 2 && alloc_cfg_.enable_pair_entry_hard_split_ && alloc_cfg_.pair_entry_hard_split_radius_ > 1e-3)
    {
        std::vector<int> anchors0, anchors1;
        if (!first_ids.empty() && !first_ids[0].empty())
            anchors0.push_back(first_ids[0].front());
        if (!first_ids.empty() && first_ids.size() > 1 && !first_ids[1].empty())
            anchors1.push_back(first_ids[1].front());
        if (!second_ids.empty() && !second_ids[0].empty())
            anchors0.push_back(second_ids[0].front());
        if (!second_ids.empty() && second_ids.size() > 1 && !second_ids[1].empty())
            anchors1.push_back(second_ids[1].front());

        if (!anchors0.empty() && !anchors1.empty())
        {
            for (int k = 0; k < grid_size; ++k)
            {
                Eigen::Vector3d c     = hgrid_->getCenter(active_grid_ids[k]);
                bool            near0 = false, near1 = false;
                for (int gid : anchors0)
                {
                    if ((c - hgrid_->getCenter(gid)).norm() < alloc_cfg_.pair_entry_hard_split_radius_)
                    {
                        near0 = true;
                        break;
                    }
                }
                for (int gid : anchors1)
                {
                    if ((c - hgrid_->getCenter(gid)).norm() < alloc_cfg_.pair_entry_hard_split_radius_)
                    {
                        near1 = true;
                        break;
                    }
                }

                if (!(near0 && near1))
                    continue;

                const int col = k + drone_num;
                if (mat(0, col) >= INFINITE_COST / 2 && mat(1, col) >= INFINITE_COST / 2)
                    continue;

                double       d0 = (c - positions[0]).norm();
                double       d1 = (c - positions[1]).norm();
                const double m  = alloc_cfg_.pair_entry_hard_split_margin_;
                if (d0 + m < d1)
                    mat(1, col) = INFINITE_COST;
                else if (d1 + m < d0)
                    mat(0, col) = INFINITE_COST;
            }
        }
    }
}

/**
 * @brief 合并网格之后再重分配
 * @param
 *      in:  两架无人机的位置、速度、初始网格、次要网格、待分配的网格
 *      out: 分配给自己的网格、分配给伙伴的网格
 */
void FastExplorationManager::allocateGrids(const vector<Eigen::Vector3d>& positions, const vector<Eigen::Vector3d>& velocities,
                                           const vector<vector<int>>& first_ids, const vector<vector<int>>& second_ids, const vector<int>& grid_ids,
                                           vector<int>& ego_ids, vector<int>& other_ids, int ego_drone_id, int partner_drone_id, int* ego_tour_head,
                                           int* other_tour_head)
{
    // RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[Allocate] Dynamic regional assignment started.");

    auto t1 = node_->now();

    const int drone_num = positions.size();  // 本次参与分配的无人机数量（通常为2，两两配对）

    const double kInfiniteCostAllocate = 1e9;
    const int    total_regions_cap     = std::max(1, ep_->drone_num_);

    AllocateGridsMaybeDynamicRelayout(grid_ids, drone_num, total_regions_cap);

    if (grid_ids.empty())
    {
        ego_ids.clear();
        other_ids.clear();
        return;
    }

    if (grid_ids.size() == 1)
    {
        auto                    pt = hgrid_->getCenter(grid_ids.front());
        vector<Eigen::Vector3d> path;
        double                  d1 = ViewNode::computeCost(positions[0], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
        double                  d2 = (drone_num > 1) ? ViewNode::computeCost(positions[1], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path) : 1e9;
        if (d1 < d2)
        {
            ego_ids   = grid_ids;
            other_ids = {};
        }
        else
        {
            ego_ids   = {};
            other_ids = grid_ids;
        }
        return;
    }

    vector<int>                          active_grid_ids = grid_ids;
    std::unordered_map<int, vector<int>> mtsp_cluster_expand;
    vector<vector<int>>                  mtsp_dropped_clusters;
    if (alloc_cfg_.enable_mtsp_grid_reduction_ && drone_num >= 1)
    {
        ReduceGridsForPairMtsp(positions, first_ids, second_ids, active_grid_ids, mtsp_cluster_expand, mtsp_dropped_clusters);
        if (active_grid_ids.size() < grid_ids.size())
        {
            RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[Allocate][d%d] MTSP grid reduction: input=%zu reps=%zu dropped_clusters=%zu",
                        ep_->drone_id_, grid_ids.size(), active_grid_ids.size(), mtsp_dropped_clusters.size());
        }
    }

    const int grid_size = static_cast<int>(active_grid_ids.size());
    const int dimension = drone_num + grid_size;

    if (active_grid_ids.empty())
    {
        ego_ids.clear();
        other_ids.clear();
        return;
    }

    Eigen::MatrixXd mat;
    hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, active_grid_ids, mat);

    if (mat.rows() != dimension || mat.cols() != dimension)
    {
        if (mat.rows() == dimension + 1 && mat.cols() == dimension + 1)
            mat = mat.block(0, 0, dimension, dimension);
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "Matrix dimension mismatch! rows:%ld, cols:%ld, dim:%d", mat.rows(), mat.cols(), dimension);
            return;
        }
    }

    const int total_regions = std::max(1, region_allocator_->layout().total_regions);

    auto clampRegion = [&](int r) -> int {
        if (r < 0)
            return 0;
        if (r >= total_regions)
            return total_regions - 1;
        return r;
    };

    AllocateGridsMaybePeriodicGlobalRegionAssignment(positions);

    std::vector<int> drone_ids_for_log(drone_num);
    for (int i = 0; i < drone_num; ++i)
    {
        drone_ids_for_log[i] =
            (drone_num == 2 && i == 0 && ego_drone_id > 0) ? ego_drone_id : ((drone_num == 2 && i == 1 && partner_drone_id > 0) ? partner_drone_id : (i + 1));
    }

    std::vector<int> drone_regions(drone_num, -1);
    for (int i = 0; i < drone_num; ++i)
    {
        int       reg = -1;
        const int did = drone_ids_for_log[i];
        if (GetGlobalRegionAssignment(did, reg))
            drone_regions[i] = clampRegion(reg);
        else
            drone_regions[i] = clampRegion(region_allocator_->layout().total_regions > 0 ? (did - 1) % total_regions : 0);
    }

    AllocateGridsApplyCostMatrixHeuristics(mat, positions, velocities, first_ids, second_ids, active_grid_ids, drone_num, grid_size, kInfiniteCostAllocate,
                                           drone_ids_for_log, drone_regions);

    // ⭐ 存储原始成本，用于解决分配冲突 (保持不变)
    std::map<int, std::map<int, double>> initial_costs;
    for (int j = 0; j < grid_size; ++j)
    {
        for (int i = 0; i < drone_num; ++i)
        {
            initial_costs[j][i] = mat(i, j + drone_num);
        }
    }

    double mat_time = (node_->now() - t1).seconds();

    // Find optimal path through MTSP
    t1 = node_->now();

    // Capacity and size calculation (保持不变)
    vector<int> unknown_nums;
    int         capacity = 0;
    for (int i = 0; i < active_grid_ids.size(); ++i)
    {
        int unum = hgrid_->getUnknownCellsNum(active_grid_ids[i]);
        if (unum < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "Negative unknown cells number for grid %d: %d", active_grid_ids[i], unum);
            ego_ids.clear();
            other_ids.clear();
            return;
        }
        unknown_nums.push_back(unum);
        capacity += unum;
    }

    // --- LKH 文件生成 ---
    const int prob_type = 1;
    ofstream  file(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : pairopt\n";
    file << "TYPE : ATSP\n";
    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i)
    {
        for (int j = 0; j < dimension; ++j)
        {
            int int_cost;
            if (mat(i, j) >= kInfiniteCostAllocate / 2)
            {
                int_cost = 2000000000;
            }
            else
            {
                int_cost = static_cast<int>(100 * mat(i, j));
            }
            if (int_cost < 0)
                int_cost = 0;
            file << int_cost << " ";
        }
        file << "\n";
    }
    file << "EOF";
    file.close();

    int min_size = grid_size / drone_num;
    int max_size = ceil(grid_size / (double)drone_num);
    file.open(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp\n";
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINMAX\n";
    file << "MTSP_MIN_SIZE = " << to_string(min_size) << "\n";
    file << "MTSP_MAX_SIZE = " << to_string(max_size) << "\n";
    file << "TRACE_LEVEL = 0\n";
    file << "RUNS = 1\n";
    file << "PRECISION = 1\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour\n";
    file.close();

    const string par_path  = ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".par";
    const string tour_path = ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour";
    if (runExternalLkhSolver(ep_->lkh_binary_, par_path) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "[Allocate][d%d] External LKH failed, par=%s", ep_->drone_id_, par_path.c_str());
        ego_ids.clear();
        other_ids.clear();
        return;
    }

    double mtsp_time = (node_->now() - t1).seconds();

    ifstream fin(tour_path);
    if (!fin.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "[Allocate][d%d] Failed to open tour file after MTSP service, path=%s", ep_->drone_id_,
                     tour_path.c_str());
        ego_ids.clear();
        other_ids.clear();
        return;
    }
    string      res;
    vector<int> ids;
    bool        tour_section_found = false;
    while (getline(fin, res))
    {
        if (res.compare("TOUR_SECTION") == 0)
        {
            tour_section_found = true;
            break;
        }
    }
    if (!tour_section_found)
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "[Allocate][d%d] Missing TOUR_SECTION in tour file, skip assignment.", ep_->drone_id_);
        ego_ids.clear();
        other_ids.clear();
        return;
    }
    while (getline(fin, res))
    {
        try
        {
            int id = stoi(res);
            if (id < -1)
            {
                RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "Invalid ID in tour file: %d", id);
                ego_ids.clear();
                other_ids.clear();
                return;
            }
            ids.push_back(id - 1);  // LKH tour 索引从 1 开始
            if (id == -1)
                break;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "Error parsing tour file: %s", e.what());
            ego_ids.clear();
            other_ids.clear();
            return;
        }
    }
    fin.close();

    // 解析 m-tour
    vector<vector<int>> tours;
    vector<int>         tour;
    for (auto id : ids)
    {
        if (id >= 0 && id < drone_num)
        {
            tour.clear();
            tour.push_back(id);
        }
        else if (id == -1 || id >= dimension)
        {
            if (!tour.empty())
                tours.push_back(tour);
        }
        else
        {
            tour.push_back(id);
        }
    }

    // --- 优化分配逻辑：强制独占性 (解决冲突) ---
    std::map<int, int> final_assignment;

    // 1. 根据 LKH 路径结果作为初始分配，并检测冲突
    for (const auto& current_tour : tours)
    {
        if (current_tour.empty())
            continue;
        int drone_idx = current_tour[0];
        if (drone_idx < 0 || drone_idx >= drone_num)
        {
            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[Allocate][d%d] Skip invalid drone index in tour. drone_idx=%d drone_num=%d dimension=%d",
                        ep_->drone_id_, drone_idx, drone_num, dimension);
            continue;
        }

        for (size_t k = 1; k < current_tour.size(); ++k)
        {
            int node_idx = current_tour[k];

            if (node_idx < drone_num || node_idx >= dimension)
                continue;

            int grid_list_index = node_idx - drone_num;

            if (final_assignment.count(grid_list_index))
            {
                // 冲突发生，比较成本
                int current_best_drone = final_assignment.at(grid_list_index);
                if (current_best_drone < 0 || current_best_drone >= drone_num || grid_list_index < 0 || grid_list_index >= grid_size)
                {
                    RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                                "[Allocate][d%d] Skip conflict compare due to invalid index. grid_idx=%d best_drone=%d drone_idx=%d", ep_->drone_id_,
                                grid_list_index, current_best_drone, drone_idx);
                    continue;
                }

                // 使用 initial_costs 比较成本
                auto grid_cost_it = initial_costs.find(grid_list_index);
                if (grid_cost_it == initial_costs.end())
                {
                    RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[Allocate][d%d] Missing cost row for grid index %d, skip conflict compare.",
                                ep_->drone_id_, grid_list_index);
                    continue;
                }
                auto current_cost_it = grid_cost_it->second.find(drone_idx);
                auto best_cost_it    = grid_cost_it->second.find(current_best_drone);
                if (current_cost_it == grid_cost_it->second.end() || best_cost_it == grid_cost_it->second.end())
                {
                    RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[Allocate][d%d] Missing cost entry. grid_idx=%d drone_idx=%d best_drone=%d",
                                ep_->drone_id_, grid_list_index, drone_idx, current_best_drone);
                    continue;
                }
                double cost_current = current_cost_it->second;
                double cost_best    = best_cost_it->second;

                if (cost_current < cost_best)
                {
                    // 当前无人机成本更低，重新分配
                    final_assignment[grid_list_index] = drone_idx;
                }
            }
            else
            {
                if (grid_list_index < 0 || grid_list_index >= grid_size)
                {
                    RCLCPP_WARN(rclcpp::get_logger("exploration_manager"),
                                "[Allocate][d%d] Skip invalid grid index in tour. grid_idx=%d grid_size=%d node_idx=%d drone_num=%d", ep_->drone_id_,
                                grid_list_index, grid_size, node_idx, drone_num);
                    continue;
                }
                final_assignment[grid_list_index] = drone_idx;
            }
        }
    }

    // 2. 按 MTSP tour 访问序输出，保证 [0] 为下一跳首格
    ego_ids.clear();
    other_ids.clear();
    vector<vector<int>> tour_ordered(drone_num);
    for (const auto& current_tour : tours)
    {
        if (current_tour.empty())
            continue;
        const int drone_idx = current_tour[0];
        if (drone_idx < 0 || drone_idx >= drone_num)
            continue;
        for (size_t k = 1; k < current_tour.size(); ++k)
        {
            const int node_idx = current_tour[k];
            if (node_idx < drone_num || node_idx >= dimension)
                continue;
            const int grid_list_index = node_idx - drone_num;
            if (grid_list_index < 0 || grid_list_index >= grid_size)
                continue;
            const auto it = final_assignment.find(grid_list_index);
            if (it == final_assignment.end() || it->second != drone_idx)
                continue;
            const int final_grid_id = active_grid_ids[grid_list_index];
            if (final_grid_id < 0)
                continue;
            tour_ordered[drone_idx].push_back(final_grid_id);
        }
    }

    std::unordered_set<int> tour_grids;
    for (int d = 0; d < drone_num; ++d)
    {
        for (int gid : tour_ordered[d])
            tour_grids.insert(gid);
    }

    vector<int> ego_extra, other_extra;
    for (const auto& pair : final_assignment)
    {
        const int grid_list_index    = pair.first;
        const int assigned_drone_idx = pair.second;
        if (grid_list_index < 0 || grid_list_index >= static_cast<int>(active_grid_ids.size()))
            continue;
        const int final_grid_id = active_grid_ids[grid_list_index];
        if (final_grid_id < 0 || tour_grids.count(final_grid_id))
            continue;
        if (assigned_drone_idx == 0)
            ego_extra.push_back(final_grid_id);
        else
            other_extra.push_back(final_grid_id);
    }

    auto sort_by_dist = [&](vector<int>& ids, const Eigen::Vector3d& pos) {
        if (ids.size() <= 1)
            return;
        vector<pair<int, double>> id_dist_pairs;
        id_dist_pairs.reserve(ids.size());
        for (const int id : ids)
        {
            vector<Eigen::Vector3d> path;
            const double            dist = ViewNode::computeCost(pos, hgrid_->getCenter(id), 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
            id_dist_pairs.emplace_back(id, dist);
        }
        sort(id_dist_pairs.begin(), id_dist_pairs.end(), [](const pair<int, double>& a, const pair<int, double>& b) { return a.second < b.second; });
        ids.clear();
        for (const auto& p : id_dist_pairs)
            ids.push_back(p.first);
    };

    ego_ids = std::move(tour_ordered[0]);
    if (drone_num > 1)
        other_ids = std::move(tour_ordered[1]);
    int ego_head_val   = ego_ids.empty() ? -1 : ego_ids.front();
    int other_head_val = other_ids.empty() ? -1 : other_ids.front();
    sort_by_dist(ego_extra, positions[0]);
    if (drone_num > 1)
        sort_by_dist(other_extra, positions[1]);
    ego_ids.insert(ego_ids.end(), ego_extra.begin(), ego_extra.end());
    other_ids.insert(other_ids.end(), other_extra.begin(), other_extra.end());

    auto expand_cluster_members = [&](vector<int>& ids) {
        if (mtsp_cluster_expand.empty())
            return;
        vector<int>             out;
        std::unordered_set<int> seen;
        out.reserve(ids.size() * 2u);
        for (int gid : ids)
        {
            auto it = mtsp_cluster_expand.find(gid);
            if (it == mtsp_cluster_expand.end())
            {
                if (seen.insert(gid).second)
                    out.push_back(gid);
                continue;
            }
            for (int m : it->second)
            {
                if (seen.insert(m).second)
                    out.push_back(m);
            }
        }
        ids.swap(out);
    };
    expand_cluster_members(ego_ids);
    expand_cluster_members(other_ids);

    for (const auto& cluster : mtsp_dropped_clusters)
    {
        if (cluster.empty())
            continue;
        Eigen::Vector3d cen = Eigen::Vector3d::Zero();
        for (int gid : cluster)
            cen += hgrid_->getCenter(gid);
        cen /= static_cast<double>(cluster.size());
        int best_drone = 0;
        if (positions.size() > 1)
        {
            const double d0 = (cen - positions[0]).norm();
            const double d1 = (cen - positions[1]).norm();
            best_drone      = (d0 <= d1) ? 0 : 1;
        }
        vector<int> sorted_cl = cluster;
        std::sort(sorted_cl.begin(), sorted_cl.end());
        if (best_drone == 0)
        {
            for (int gid : sorted_cl)
                ego_ids.push_back(gid);
        }
        else
        {
            for (int gid : sorted_cl)
                other_ids.push_back(gid);
        }
    }

    auto rotate_head_front = [](vector<int>& ids, int& head) {
        if (ids.empty())
        {
            head = -1;
            return;
        }
        if (head < 0)
        {
            head = ids.front();
            return;
        }
        const auto it = std::find(ids.begin(), ids.end(), head);
        if (it == ids.end())
            head = ids.front();
        else if (it != ids.begin())
            std::rotate(ids.begin(), it, std::next(it));
    };
    rotate_head_front(ego_ids, ego_head_val);
    if (drone_num > 1)
        rotate_head_front(other_ids, other_head_val);
    if (ego_tour_head)
        *ego_tour_head = ego_head_val;
    if (other_tour_head)
        *other_tour_head = other_head_val;

    // 输出分配结果
    // RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "Grid allocation: ego_ids=%zu, other_ids=%zu", ego_ids.size(), other_ids.size());
    for (const auto& id : ego_ids)
    {
        auto                    center = hgrid_->getCenter(id);
        vector<Eigen::Vector3d> path;
        double                  dist = ViewNode::computeCost(positions[0], center, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
        // RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "Ego grid %d: distance=%.2f", id, dist);
    }
}

double FastExplorationManager::computeGridPathCost(const Eigen::Vector3d& pos, const vector<int>& grid_ids, const vector<int>& first,
                                                   const vector<vector<int>>& firsts, const vector<vector<int>>& seconds, const double& w_f)
{
    if (grid_ids.empty())
        return 0.0;

    double                  cost = 0.0;
    vector<Eigen::Vector3d> path;
    cost += hgrid_->getCostDroneToGrid(pos, grid_ids[0], first);
    for (int i = 0; i < grid_ids.size() - 1; ++i)
    {
        cost += hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size());
    }
    return cost;
}

/**
 * @brief 为无人机规划一个访问网格的最优路径
 * @param
 *          in： 无人机当前位置、 无人机当前速度、 是否初始化成功
 *          out：返回无人机要访问的网格ID序列、 多无人机下其他无人机的路径
 */
bool FastExplorationManager::findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions, const vector<Eigen::Vector3d>& velocities, vector<int>& indices,
                                                  vector<vector<int>>& others, bool init)
{
    // RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "Find grid tour!");

    auto t1 = node_->now();

    // 获取当前无人机的网格ID列表
    auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;

    // hgrid_->updateBaseCoor();  // Use the latest basecoor transform of swarm

    // 定义两个辅助列表，分别存储第一级和第二级网格 ID
    vector<int> first_ids, second_ids;

    // 将前沿的平均位置输入到网格管理器
    hgrid_->inputFrontiers(ed_->averages_);

    // 更新网格数据，分配网格给当前无人机
    const Eigen::Vector3d robot_w =
        positions.empty() ? Eigen::Vector3d::Zero() : positions[std::min(ep_->drone_id_ - 1, static_cast<int>(positions.size()) - 1)];
    hgrid_->updateGridData(ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids, robot_w);

    if (grid_ids.empty())
        retainStickyGridIdsAfterPrune(grid_ids);

    if (grid_ids.empty())
    {
        Eigen::Vector3d pseudo_yaw = Eigen::Vector3d::Zero();
        if (!velocities.empty() && velocities[0].head<2>().norm() > 0.2)
            pseudo_yaw[0] = std::atan2(velocities[0].y(), velocities[0].x());
        trySoloAssignGridsIfEmpty(robot_w, velocities.empty() ? Eigen::Vector3d::Zero() : velocities[0], pseudo_yaw);
    }

    // 如果没有分配网格， 返回失败
    if (grid_ids.empty())
    {
        // RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "Empty dominance.");
        ed_->grid_tour_.clear();
        return false;
    }

    dedupePreserveOrder(grid_ids);
    if (hgrid_)
        hgrid_->sanitizeGlobalGridIds(grid_ids);
    if (grid_ids.empty())
    {
        ed_->grid_tour_.clear();
        return false;
    }

    if (!init && !ed_->reallocated_ && grid_ids == ed_->last_grid_ids_ && !ed_->last_grid_ids_.empty())
    {
        indices = ed_->last_grid_ids_;
        others.clear();
        hgrid_->getGridTour(indices, positions[0], ed_->grid_tour_, ed_->grid_tour2_);
        return true;
    }

    // 根据是否初始化模式，计算成本矩阵
    Eigen::MatrixXd mat;
    // uniform_grid_->getCostMatrix(positions, velocities, first_ids, grid_ids, mat);
    if (!init)
        hgrid_->getCostMatrix(positions, velocities, { first_ids }, { second_ids }, grid_ids, mat);
    else
        hgrid_->getCostMatrix(positions, velocities, { {} }, { {} }, grid_ids, mat);

    double mat_time = (node_->now() - t1).seconds();

    // 使用 ATSP 求解最优路径
    t1                  = node_->now();
    const int dimension = mat.rows();
    const int drone_num = 1;

    // Create problem file
    ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
    // std::cout << "*** atsp file: " << ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp" << std::endl;

    file << "NAME : amtsp\n";                              // 任务名
    file << "TYPE : ATSP\n";                               // 单向跑
    file << "DIMENSION : " + to_string(dimension) + "\n";  //多少个点 （几个边界对应几个点）
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";               // 代价写清除
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    file << "EDGE_WEIGHT_SECTION\n";

    for (int i = 0; i < dimension; ++i)
    {
        for (int j = 0; j < dimension; ++j)
        {
            int int_cost = 100 * mat(i, j);
            file << int_cost << " ";
        }
        file << "\n";
    }
    file.close();

    // Create par file
    file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";  //告知任务单地址
    file << "SALESMEN = " << to_string(drone_num) << "\n";                                            //每个表对应单架飞机
    file << "MTSP_OBJECTIVE = MINSUM\n";                                                              //算单价最少

    // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) <<
    // "\n"; file << "MTSP_MAX_SIZE = "
    //      << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
    file << "RUNS = 1\n";
    file << "TRACE_LEVEL = 0\n";
    file << "PRECISION = 1\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
    file.close();

    const std::string par_path  = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par";
    const std::string tour_path = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour";
    t1                          = node_->now();

    vector<int> ids;
    if (!solveAmtspTourInProcess(par_path, tour_path, dimension, ids))
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "[findGlobalTourOfGrid][d%d] Empty grid MTSP tour file: %s", ep_->drone_id_, tour_path.c_str());
        ed_->grid_tour_.clear();
        return false;
    }

    vector<vector<int>> tours;
    if (!parseMtspToursFromRawIds(ids, dimension, drone_num, tours))
    {
        RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"), "[findGlobalTourOfGrid][d%d] Failed to parse grid MTSP tour (dim=%d)", ep_->drone_id_,
                     dimension);
        ed_->grid_tour_.clear();
        return false;
    }

    others.resize(drone_num - 1);
    for (int i = 1; i < static_cast<int>(tours.size()); ++i)
    {
        if (tours[i].empty())
            continue;
        if (tours[i][0] == 1)
        {
            indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
        }
        else if (tours[i][0] >= 2 && tours[i][0] <= drone_num)
        {
            others[tours[i][0] - 2].insert(others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
        }
    }
    for (auto& id : indices)
    {
        id -= 1 + drone_num;
    }
    for (auto& other : others)
    {
        for (auto& id : other)
            id -= 1 + drone_num;
    }
    std::cout << "网格巡游: ";
    vector<int> resolved_tour_indices;
    resolved_tour_indices.reserve(indices.size());
    for (const int idx : indices)
    {
        if (idx < 0 || idx >= static_cast<int>(grid_ids.size()))
        {
            RCLCPP_ERROR(rclcpp::get_logger("exploration_manager"),
                         "[findGlobalTourOfGrid][d%d] invalid tour remap idx=%d grid_ids.size=%zu (check MTSP tour / dimension)", ep_->drone_id_, idx,
                         grid_ids.size());
            ed_->grid_tour_.clear();
            return false;
        }
        resolved_tour_indices.push_back(grid_ids[static_cast<size_t>(idx)]);
        std::cout << resolved_tour_indices.back() << ", ";
    }
    std::cout << "" << std::endl;

    indices = std::move(resolved_tour_indices);
    dedupePreserveOrder(indices);
    grid_ids = indices;
    hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_);

    ed_->last_grid_ids_ = grid_ids;
    ed_->reallocated_   = false;

    // hgrid_->checkFirstGrid(grid_ids.front());

    return true;
}

void FastExplorationManager::findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, const vector<int>& ftr_ids,
                                                const vector<Eigen::Vector3d>& grid_pos, vector<int>& indices)
{
    auto t1 = node_->now();

    vector<Eigen::Vector3d> positions  = { cur_pos };
    vector<Eigen::Vector3d> velocities = { cur_vel };
    vector<double>          yaws       = { cur_yaw[0] };

    // frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, mat);
    Eigen::MatrixXd mat;
    frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
    const int dimension = mat.rows();
    // std::cout << "dim of frontier TSP mat: " << dimension << std::endl;

    double mat_time = (node_->now() - t1).seconds();
    // RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "mat time: %lf", mat_time);

    // Find optimal allocation through AmTSP
    t1 = node_->now();

    // Create problem file
    ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : amtsp\n";
    file << "TYPE : ATSP\n";
    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i)
    {
        for (int j = 0; j < dimension; ++j)
        {
            int int_cost = 100 * mat(i, j);
            file << int_cost << " ";
        }
        file << "\n";
    }
    file.close();

    // Create par file
    const int drone_num = 1;

    file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
    file << "MTSP_MAX_SIZE = " << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
    file << "RUNS = 1\n";
    file << "TRACE_LEVEL = 0\n";
    file << "PRECISION = 1\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
    file.close();

    const std::string par_path  = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par";
    const std::string tour_path = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour";

    vector<int> ids;
    if (!solveAmtspTourInProcess(par_path, tour_path, dimension, ids))
    {
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[findTourOfFrontier][d%d] Empty frontier MTSP tour; fallback to input order", ep_->drone_id_);
        indices = ftr_ids;
        frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
        if (!grid_pos.empty())
            ed_->frontier_tour_.push_back(grid_pos[0]);
        return;
    }

    vector<vector<int>> tours;
    if (!parseMtspToursFromRawIds(ids, dimension, drone_num, tours))
    {
        RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[findTourOfFrontier][d%d] Failed to parse frontier MTSP tour; fallback to input order",
                    ep_->drone_id_);
        indices = ftr_ids;
        frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
        if (!grid_pos.empty())
            ed_->frontier_tour_.push_back(grid_pos[0]);
        return;
    }

    vector<vector<int>> others(drone_num - 1);
    for (int i = 1; i < static_cast<int>(tours.size()); ++i)
    {
        if (tours[i].empty())
            continue;
        if (tours[i][0] == 1)
            indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    }
    for (auto& id : indices)
        id -= 1 + drone_num;

    if (ed_->grid_tour_.size() > 2 && !indices.empty())
        indices.pop_back();

    vector<int> mapped;
    mapped.reserve(indices.size());
    for (int idx : indices)
    {
        if (idx < 0 || idx >= static_cast<int>(ftr_ids.size()))
        {
            RCLCPP_WARN(rclcpp::get_logger("exploration_manager"), "[findTourOfFrontier][d%d] Skip invalid tour idx=%d ftr_ids.size=%zu (stale MTSP tour?)",
                        ep_->drone_id_, idx, ftr_ids.size());
            continue;
        }
        mapped.push_back(ftr_ids[static_cast<size_t>(idx)]);
    }
    if (mapped.empty())
        mapped = ftr_ids;
    indices.swap(mapped);

    // Get the path of optimal tour from path matrix
    frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
    if (!grid_pos.empty())
        ed_->frontier_tour_.push_back(grid_pos[0]);

    // ed_->other_tours_.clear();
    // for (int i = 1; i < positions.size(); ++i) {
    //   ed_->other_tours_.push_back({});
    //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
    // }

    double parse_time = (node_->now() - t1).seconds();
    static_cast<void>(parse_time);
    static_cast<void>(others);
}

}  // namespace fast_planner
