#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <bspline/msg/bspline.hpp>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using Eigen::Vector3d;
using std::string;
using std::vector;

namespace fast_planner
{
struct FSMData
{
    // FSM data
    bool           trigger_, have_odom_, static_state_;
    vector<string> state_str_;

    Eigen::Vector3d    odom_pos_, odom_vel_;  // odometry state
    Eigen::Quaterniond odom_orient_;
    double             odom_yaw_;
    rclcpp::Time       odom_stamp_;

    Eigen::Vector3d         start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
    vector<Eigen::Vector3d> start_poss;
    bspline::msg::Bspline   newest_traj_;

    // Swarm collision avoidance
    bool         avoid_collision_, go_back_;
    rclcpp::Time fsm_init_time_;
    rclcpp::Time last_check_frontier_time_;

    Eigen::Vector3d start_pos_;

    rclcpp::Time exec_start_time_;  //! add by zqh
    bool         exec_started_;     //! add by zqh
    bool         swarm_conflict_replan_pending_{ false };
    rclcpp::Time wait_map_update_until_;
};

struct FSMParam
{
    double replan_thresh1_;
    double replan_thresh2_;
    double replan_thresh3_;
    double replan_thresh_time_;  // second
    double replan_time_;         // second, for replan interval
    bool   replan_skip_if_goal_unchanged_;
    double replan_min_goal_change_m_;
    double replan_min_remaining_path_m_;

    // Swarm
    double attempt_interval_;   // Min interval of opt attempt
    double pair_opt_interval_;  // Min interval of successful pair opt
    int    repeat_send_num_;
    double swarm_traj_keepalive_s_;  // Periodic cross-drone trajectory refresh (s)

    double stop_debounce_s_;
    int    clear_resume_confirm_frames_;           // CLEAR_RESUME consecutive no-conflict frames (40Hz main loop)
    double clear_resume_odom_min_dist_;            // CLEAR_RESUME odom gate: min peer physical distance (m)
    int    swarm_conflict_resume_confirm_frames_;  // SWARM_CONFLICT_STOP consecutive not-yield msgs (10Hz peer traj)
    double priority_resume_base_s_;
    double priority_resume_dist_gain_s_;
    double timeout_replan_base_s_;
    double timeout_replan_dist_gain_s_;
    double timeout_replan_max_s_;
    double pair_conflict_cooldown_s_;

    double final_min_by_drones_factor_;

    // Final return vote: (task load asc, distance-to-return asc, id asc).
    int final_return_load_margin_;          // Load gap (grids) within which loads count as equal
    int final_return_vote_confirm_cycles_;  // Consecutive frames the same winner must hold

    double pair_select_stamp_timeout_;
    double pair_sel_conflict_dist_;
    double pair_sel_first_grid_dist_;
    double pair_sel_conflict_weight_;
    double pair_sel_interval_weight_;
    double pair_sel_first_grid_weight_;
    double static_speed_thresh_;

    // Hysteresis thresholds for online/offline detection in final coordination.
    double final_online_stamp_thresh_;   // Mark online if state is fresher than this (s)
    double final_offline_stamp_thresh_;  // Mark offline only if state is older than this (s)

    // Ping-based gate before entering WAIT_TRIGGER.
    bool   wait_trigger_ping_check_enable_;  // If true, require ping latency < threshold before WAIT_TRIGGER
    double wait_trigger_ping_interval_s_;    // Min interval between ping checks (s)
    double wait_trigger_ping_thresh_ms_;     // Latency threshold (ms)
    string wait_trigger_ping_target_;        // Target IP/hostname to ping

    // Waiting-phase quiet mode (INIT/WAIT_TRIGGER): throttle periodic broadcasts and
    // preheat computations that have no cross-vehicle consumer until the trigger arrives.
    bool   wait_trigger_quiet_enable_;          // If true, enter quiet mode while waiting for trigger
    double wait_trigger_quiet_state_period_s_;  // drone_state keepalive period during quiet mode (s)

    double wait_map_update_retry_sec_;
    double liveness_check_period_sec_;
};

struct DroneState
{
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d start_pos_{ Eigen::Vector3d::Zero() };
    bool            start_pos_valid_{ false };
    double          yaw_;
    double          stamp_;                // Stamp of pos,vel,yaw
    double          recent_attempt_time_;  // Stamp of latest opt attempt with any drone

    vector<int> grid_ids_;              // Id of grid tour
    double      recent_interact_time_;  // Stamp of latest opt with this drone
    int         cur_state_{ -1 };       // FSM state (matches msg_set/DroneState.cur_state)
    bool        returning_{ false };    // Released for final return (incl. FINISH flight leg)
    int         hgrid_cell_count_{ 0 };
};

struct ExplorationData
{
    vector<vector<Vector3d>>         frontiers_;
    vector<vector<Vector3d>>         dead_frontiers_;
    vector<pair<Vector3d, Vector3d>> frontier_boxes_;
    vector<Vector3d>                 points_;
    vector<Vector3d>                 averages_;
    vector<Vector3d>                 views_;
    vector<double>                   yaws_;
    vector<Vector3d>                 frontier_tour_;
    vector<vector<Vector3d>>         other_tours_;

    vector<int>              refined_ids_;
    vector<vector<Vector3d>> n_points_;
    vector<Vector3d>         unrefined_points_;
    vector<Vector3d>         refined_points_;
    vector<Vector3d>         refined_views_;  // points + dir(yaw)
    vector<Vector3d>         refined_views1_, refined_views2_;
    vector<Vector3d>         refined_tour_;

    Vector3d         next_goal_;
    vector<Vector3d> path_next_goal_, kino_path_;
    Vector3d         next_pos_;
    double           next_yaw_;

    // viewpoint planning
    // vector<Vector4d> views_;
    vector<Vector3d> views_vis1_, views_vis2_;
    vector<Vector3d> centers_, scales_;

    // Swarm, other drones' state
    vector<DroneState> swarm_state_;
    vector<double>     pair_opt_stamps_, pair_opt_res_stamps_;
    vector<int>        ego_ids_, other_ids_;
    double             pair_opt_stamp_;
    bool               reallocated_, wait_response_;

    // Coverage planning
    vector<Vector3d> grid_tour_, grid_tour2_;
    // int prev_first_id_;
    vector<int> last_grid_ids_;

    int plan_num_;

    Eigen::Vector3d             last_next_pos_;
    std::deque<Eigen::Vector3d> recent_pos_;
    std::vector<double>         frontier_gains_;

    std::vector<Eigen::Vector3d> full_path_next_goal_;  // SNN 完整路径缓存（接续规划）

    Eigen::Vector3d cached_snn_goal_;
    bool            cached_snn_goal_valid_{ false };

    int    sticky_fallback_grid_{ -1 };
    double sticky_fallback_until_sec_{ -1.0 };

    // Consensus region assignment: local SNN result and peer proposals.
    std::vector<int>                  assigned_regions_;               // [drone_id-1] = region, local computation
    double                            assigned_regions_stamp_{ 0.0 };  // timestamp of local computation
    uint64_t                          assigned_regions_epoch_{ 0 };    // monotonically increasing complete snapshot
    int                               assigned_regions_sender_{ 0 };
    std::vector<uint8_t>              assigned_regions_online_mask_;  // complete snapshot online set
    std::vector<std::vector<int>>     peer_assigned_regions_;         // [drone_id-1] = proposal from that peer
    std::vector<double>               peer_assigned_regions_stamp_;   // [drone_id-1] = proposal stamp
    std::vector<uint64_t>             peer_assigned_regions_epoch_;
    std::vector<std::vector<uint8_t>> peer_assigned_regions_online_mask_;

    // 目标承诺（视点锁）：跨规划周期继续执行同一个目标，抑制近似等价候选间的来回切换。
    Eigen::Vector3d locked_viewpoint_;
    double          locked_viewpoint_yaw_{ 0.0 };
    double          locked_viewpoint_until_sec_{ -1.0 };
    int             locked_grid_id_{ -1 };
    bool            locked_viewpoint_valid_{ false };
};

struct ExplorationParam
{
    // params
    bool   refine_local_;
    int    refined_num_;
    double refined_radius_;
    int    top_view_num_;
    double max_decay_;
    double local_traj_radius_;
    double local_traj_eucl_ratio_;
    double local_traj_min_arc_m_;
    double snn_path_rdp_epsilon_;
    double snn_path_rdp_max_epsilon_;
    double snn_path_resample_spacing_;
    double snn_path_shorten_max_segment_;
    double snn_path_shorten_angle_deg_;
    int    snn_traj_max_waypoints_;
    double snn_traj_rdp_epsilon_;
    // Soft penalties added when selecting a frontier/viewpoint from its A* polyline.
    // They are measured in approximate seconds, matching ViewNode::computeCost.
    double path_turn_penalty_s_per_pi_{ 0.7 };
    double path_corner_penalty_s_{ 0.4 };
    double path_reverse_penalty_s_{ 1.5 };
    double path_detour_penalty_s_per_ratio_{ 1.0 };
    double min_path_length_m_{ 0.0 };
    bool   snn_path_sticky_enable_;
    double snn_max_detour_ratio_;
    double snn_replan_min_move_m_;
    double snn_goal_same_dist_m_;
    double snn_path_deviation_max_m_;
    bool   solo_assign_enable_;
    int    solo_assign_max_grids_;
    double solo_assign_radius_;
    double mtsp_fallback_max_goal_dist_;
    double sticky_fallback_grid_sec_;
    int    sticky_grid_keep_max_;
    double viewpoint_lock_sec_;
    double viewpoint_lock_max_dist_m_;
    double viewpoint_lock_max_shift_m_;
    double viewpoint_lock_switch_improve_ratio_;
    double snn_cache_min_arc_m_;
    double snn_cache_goal_end_tol_m_;
    int    snn_fail_astar_trigger_count_;
    double viewpoint_arrived_dist_m_;
    double degenerate_goal_dist_m_;
    bool   target_failure_quarantine_enable_{ true };
    double target_failure_quarantine_cell_m_{ 0.5 };
    double target_failure_quarantine_initial_sec_{ 1.0 };
    double target_failure_quarantine_max_sec_{ 10.0 };
    int    target_failure_quarantine_max_entries_{ 128 };
    bool   anti_tailgate_enable_;
    double anti_tailgate_heading_deg_;
    double anti_tailgate_ahead_enter_m_;
    double anti_tailgate_ahead_exit_m_;
    double anti_tailgate_sep_weight_;
    double anti_tailgate_self_cost_weight_;

    // ③ peer coverage-box anti-clustering soft penalty (distance-equivalent, m)
    //    applied to targets inside a peer's fresh explored box.
    double peer_cov_penalty_{ 20.0 };
    double peer_cov_margin_m_{ 2.0 };
    bool   near_split_enable_;
    double near_split_dist_;
    double near_split_stamp_timeout_;
    double near_split_min_target_dist_;
    double near_split_sep_weight_;
    double near_split_self_cost_weight_;
    double near_split_uncovered_bonus_;
    double near_split_same_region_penalty_;
    double near_split_cooldown_;
    double near_split_lock_sec_;
    double near_split_lock_extend_dist_;
    double near_split_lock_extend_sec_;
    double near_split_lock_max_total_sec_{ 6.0 };
    double near_split_region_hysteresis_sec_;
    double near_split_region_switch_penalty_;
    double near_split_uncovered_bonus_decay_dist_;
    double near_split_uncovered_bonus_min_scale_;
    double near_split_uncovered_bonus_density_gain_;
    double near_split_uncovered_bonus_occ_penalty_;
    double near_split_min_peer_target_dist_;
    double near_split_switch_min_improve_ratio_;
    double near_split_switch_min_interval_sec_;
    double pre_split_lookahead_sec_{ 3.0 };             // 预测预分裂：速度外推时间窗口(s)
    double pre_split_dist_{ 12.0 };                     // 预测预分裂：外推位置距离阈值(m)
    double near_split_incoming_region_penalty_{ 8.0 };  // 方向二：peer 预测将达到区域的惩罚权重
    bool   near_split_log_uncovered_bonus_;
    int    plan_fail_fallback_trigger_count_;
    double plan_fail_fallback_min_dist_;
    int    plan_fail_circuit_threshold_;
    double plan_fail_circuit_sec_;

    // Stall escape / liveness：WAIT_MAP_UPDATE 活锁根治（见 2026-10-22 spec）
    bool   liveness_enable_{ true };
    int    stall_escape_after_waits_{ 6 };
    double stall_escape_interval_sec_{ 2.0 };
    double stall_escape_min_dist_m_{ 1.0 };
    double stall_escape_max_dist_m_{ 15.0 };
    int    stall_escape_max_attempts_{ 3 };
    double dead_work_stall_sec_{ 10.0 };
    double no_progress_stall_sec_{ 20.0 };
    double stall_return_stagger_sec_{ 5.0 };

    std::string tsp_dir_;     // resource dir of tsp solver
    std::string mtsp_dir_;    // resource dir of mtsp solver
    std::string lkh_binary_;  // external LKH executable for amtsp3 pair allocation
    double      relax_time_;
    int         init_plan_num_;

    // Swarm
    int drone_num_;
    int drone_id_;
};

}  // namespace fast_planner

#endif
