#include <lkh_mtsp_solver/srv/solve_mtsp.hpp>
#include <lkh_tsp_solver/srv/solve_tsp.hpp>
#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <Eigen/Eigen>
#include <cstdint>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>

#include <ppu_opt/snn_path_interface.h>
#include <ppu_opt/snn_region_interface.h>

using Eigen::Vector3d;
using std::pair;
using std::shared_ptr;
using std::tuple;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
// class UniformGrid;
class HGrid;
class FrontierFinder;
struct ExplorationParam;
struct ExplorationData;
enum EXPL_RESULT
{
    NO_FRONTIER,
    FAIL,
    SUCCEED,
    NO_GRID,
    WAIT_FOR_MAP_UPDATE
};

class FastExplorationManager
{
public:
    FastExplorationManager();
    ~FastExplorationManager();

    void initialize(rclcpp::Node::SharedPtr node);

    int getSnnDeadzoneMarginCells() const
    {
        return snn_deadzone_margin_cells_;
    }

    int planExploreMotion(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw);

    int planTrajToView(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw);

    int updateFrontierStruct(const Eigen::Vector3d& pos);

    // Trigger 时重置“最近一次真实规划成功”的基准，供 FSM 的无进展守卫使用。
    void MarkExplorationStart();

    // 距最近一次“非逃逸”规划成功的时长，供 FSM 的 L2 无进展守卫使用。
    double noProgressSec(const rclcpp::Time& now) const;

    // STOP（让行/急停/人工 stop）恢复时把无进展基准推到现在，避免把 STOP 时长算成死锁。
    void PauseNoProgress(const rclcpp::Time& now);

    // STOP 恢复时重置 stall 计数与放弃窗口，避免把 STOP 时长算进错峰后的“逃逸耗尽”。
    void RestartStallWindow();

    void allocateGrids(const vector<Eigen::Vector3d>& positions, const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
                       const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids, vector<int>& other_ids, int ego_drone_id = 0,
                       int partner_drone_id = 0, int* ego_tour_head = nullptr, int* other_tour_head = nullptr);

    // Find optimal tour visiting unknown grid
    bool findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions, const vector<Eigen::Vector3d>& velocities, vector<int>& ids,
                              vector<vector<int>>& others, bool init = false);

    void calcMutualCosts(const Eigen::Vector3d& pos, const double& yaw, const Eigen::Vector3d& vel, const vector<pair<Eigen::Vector3d, double>>& views,
                         vector<float>& costs);

    double computeGridPathCost(const Eigen::Vector3d& pos, const vector<int>& grid_ids, const vector<int>& first, const vector<vector<int>>& firsts,
                               const vector<vector<int>>& seconds, const double& w_f);

    shared_ptr<ExplorationData>    ed_;               //存放本机和伙伴的任务和状态
    shared_ptr<ExplorationParam>   ep_;               //管理搜索规律
    shared_ptr<FastPlannerManager> planner_manager_;  //规划管理
    shared_ptr<FrontierFinder>     frontier_finder_;  //边界相关的处理
    shared_ptr<HGrid>              hgrid_;            // 划分网格相关
    shared_ptr<SDFMap>             sdf_map_;          //获取sdfmap数据

    shared_ptr<ISnnPathPlanner>  snn_;
    shared_ptr<IRegionAllocator> region_allocator_;
    rclcpp::Node::SharedPtr      node_;

    // Decentralized consensus: arbitrate local + peer SNN region proposals and apply
    // the winning assignment to the local SNN sticky state.
    void ApplyRegionAssignmentConsensus();
    bool GetGlobalRegionAssignment(int drone_id, int& region_id) const;
    bool IsCompleteRegionAssignment(const std::vector<int>& assignments, const std::vector<uint8_t>& online_mask) const;

    // shared_ptr<UniformGrid> uniform_grid_;

    // Read-only adapter query; retain the native reachability policy.
    bool isReachableForAdapter(const Eigen::Vector3d& p) const { return IsReachablePoint(p); }
private:
    struct AllocateConfig
    {
        bool   enable_cross_region_takeover_{ false };
        double takeover_idle_sec_{ 8.0 };
        bool   enable_soft_cross_region_constraint_{ true };
        double cross_region_soft_penalty_{ 180.0 };
        double cross_region_penalty_gain_per_hop_{ 0.35 };
        bool   enable_dynamic_region_relayout_{ true };
        double dynamic_region_relayout_cooldown_{ 4.0 };
        double dynamic_region_online_timeout_{ 1.2 };
        int    dynamic_region_online_stable_cycles_{ 4 };
        int    dynamic_region_unfinished_stable_cycles_{ 4 };
        double dynamic_region_bounds_margin_m_{ 1.5 };
        double dynamic_region_bounds_min_span_m_{ 10.0 };
        bool   dynamic_region_bounds_use_frontier_aabb_{ false };
        bool   enforce_region_execution_consistency_{ true };
        bool   enable_periodic_global_region_assignment_{ true };
        double periodic_global_region_assign_sec_{ 2.0 };
        int    periodic_global_assign_min_online_{ 3 };
        double repel_threshold_{ 10.0 };
        double repel_scale_{ 3.0 };
        double pair_overlap_radius_{ 8.0 };
        double pair_overlap_penalty_scale_{ 3.6 };
        bool   enable_pair_entry_hard_split_{ true };
        double pair_entry_hard_split_radius_{ 9.5 };
        double pair_entry_hard_split_margin_{ 1.8 };
        // Pair MTSP (allocateGrids / LKH): optional grid list reduction before ATSP.
        bool   enable_mtsp_grid_reduction_{ false };
        int    mtsp_max_rep_grids_{ 48 };
        double mtsp_near_radius_m_{ 30.0 };
        double mtsp_cluster_merge_radius_m_{ 10.0 };
        bool   enable_grid_cluster_owner_{ true };
        double grid_cluster_owner_radius_m_{ 9.0 };
        double grid_cluster_owner_penalty_{ 360.0 };
        double grid_cluster_owner_timeout_sec_{ 2.0 };
    };

    shared_ptr<EDTEnvironment>                                 edt_environment_;
    rclcpp::Client<lkh_tsp_solver::srv::SolveTSP>::SharedPtr   tsp_client_;
    rclcpp::Client<lkh_mtsp_solver::srv::SolveMTSP>::SharedPtr acvrp_client_;
    double                                                     x_size, y_size;  // 分配用

    AllocateConfig alloc_cfg_;

    void ReduceGridsForPairMtsp(const vector<Eigen::Vector3d>& positions, const vector<vector<int>>& first_ids, const vector<vector<int>>& second_ids,
                                vector<int>& grid_ids_inout, std::unordered_map<int, vector<int>>& cluster_expand_out,
                                vector<vector<int>>& dropped_clusters_out);

    void AllocateGridsMaybeDynamicRelayout(const vector<int>& grid_ids, int drone_num, int total_regions_cap);

    void AllocateGridsMaybePeriodicGlobalRegionAssignment(const vector<Eigen::Vector3d>& positions);

    void AllocateGridsApplyCostMatrixHeuristics(Eigen::MatrixXd& mat, const vector<Eigen::Vector3d>& positions, const vector<Eigen::Vector3d>& velocities,
                                                const vector<vector<int>>& first_ids, const vector<vector<int>>& second_ids, const vector<int>& active_grid_ids,
                                                int drone_num, int grid_size, double infinite_cost, const std::vector<int>& drone_ids_for_log,
                                                const std::vector<int>& drone_regions);

    // Find optimal tour for coarse viewpoints of all frontiers
    void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, vector<int>& indices);

    // Find optimal coordinated tour for quadrotor swarm
    void findGridAndFrontierPath(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids, vector<int>& ftr_ids);

    // Refine local tour for next few frontiers, using more diverse viewpoints
    void refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, const vector<vector<Vector3d>>& n_points,
                         const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts, vector<double>& refined_yaws);

    bool shortenPath(vector<Vector3d>& path, const std::function<bool(const Vector3d&, const Vector3d&)>* seg_free_override = nullptr);
    bool planSnnPolylineToGoal(const Vector3d& pos_plan, const Vector3d& modified_next_pos, const Vector3d& pos, double goal_dist_xy);
    bool planEdtAstarFallback(const Vector3d& pos_plan, const Vector3d& modified_next_pos, bool optimistic);

    bool segmentCollisionFreeLine(const Vector3d& a, const Vector3d& b) const;
    void simplifyPathSafeRdp(vector<Vector3d>& path, double epsilon);
    void simplifyPathSegmentedRdp(vector<Vector3d>& path, double epsilon);
    void capPathWaypointsRdp(vector<Vector3d>& path, double base_eps, double max_eps, size_t max_pts);
    void simplifySnnPathForTraj(vector<Vector3d>& path, size_t max_pts);

    int    consecutive_snn_path_fail_{ 0 };
    int    consecutive_plan_fail_{ 0 };
    double fallback_circuit_open_until_sec_{ -1.0 };

    struct TargetFailureState
    {
        int      failure_count{ 0 };
        double   blocked_until_sec{ -1.0 };
        Vector3d target{ Vector3d::Zero() };
    };

    std::unordered_map<std::int64_t, TargetFailureState> target_failures_;

    std::int64_t targetFailureKey(const Vector3d& target) const;
    bool         isTargetQuarantined(const Vector3d& target, const Vector3d& pos);
    bool         admitTarget(const Vector3d& pos, const Vector3d& target);
    void         recordTargetFailure(const Vector3d& target);
    void         clearTargetFailure(const Vector3d& target);
    void         pruneTargetFailures();

    // P0-2 死区可达性：每周期用实时 SDF 重建 SNN grid 并计算当前位置的可达掩码，
    // 目标选择据此过滤死区对侧的不可达目标。掩码不可用时 IsReachablePoint 放行（保持旧行为）。
    bool UpdateReachabilityMask(const Vector3d& pos);
    bool IsReachablePoint(const Eigen::Vector3d& world_pos) const;

    // ③ soft anti-clustering: distance-equivalent cost of a target point that lies
    //    inside a peer's fresh explored box (fast drone_state channel), else 0.
    double peerCoveragePenalty(const Eigen::Vector3d& p) const;

    std::vector<std::uint8_t> reach_mask_;
    int                       reach_mask_H_{ 0 };
    int                       reach_mask_W_{ 0 };
    bool                      reach_mask_valid_{ false };

    // 死区规划 margin（格数）：仅用于 SNN grid 膨胀，不影响 SDF/可达性掩码
    int snn_deadzone_margin_cells_{ 0 };

    double getMaxLocalGoalDist() const;

    bool tryPlanFailRetarget(const Vector3d& pos, vector<int>& grid_ids, vector<int>& frontier_ids, Vector3d& out_pos, double& out_yaw);

    void findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, const vector<int>& ftr_ids,
                            const vector<Eigen::Vector3d>& grid_pos, vector<int>& ids);

    bool collectNearbyGridIdsWithFrontiers(const Vector3d& cur_pos, vector<int>& out_grid_ids, double max_radius, int max_grids) const;

    bool trySoloAssignGridsIfEmpty(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw);

    void persistSoloGridAssignment(const vector<int>& grid_ids, const Vector3d& cur_pos);

    void retainStickyGridIdsAfterPrune(vector<int>& grid_ids);

    bool pickNearestFrontierWithinRadius(const Vector3d& cur_pos, double max_radius, vector<int>& frontier_ids, bool restrict_to_active_grids = false);

    bool recoverStaleGridAssignment(const Vector3d& cur_pos, vector<int>& grid_ids, vector<int>& frontier_ids);

    // --- WAIT_MAP_UPDATE stall escape（见 2026-10-22 spec） ---
    int  HandleStallBeforeWait(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw);
    bool tryStallEscape(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw, const Vector3d& last_failed_target);
    void MarkNormalPlanSuccess();

    int             stall_wait_rounds_{ 0 };
    int             escape_attempts_{ 0 };
    double          stall_first_time_sec_{ -1.0 };
    double          last_escape_time_sec_{ -1.0 };
    Eigen::Vector3d last_stall_target_{ Eigen::Vector3d::Zero() };
    bool            last_stall_target_valid_{ false };
    double          last_normal_plan_success_time_sec_{ -1.0 };

public:
    typedef shared_ptr<FastExplorationManager> Ptr;
};

}  // namespace fast_planner

#endif