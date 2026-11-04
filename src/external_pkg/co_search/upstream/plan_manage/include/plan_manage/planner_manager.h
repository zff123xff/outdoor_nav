#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>

#include <path_searching/astar2.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/topo_prm.h>

#include <plan_env/edt_environment.h>

#include <active_perception/frontier_finder.h>
#include <active_perception/heading_planner.h>

#include <plan_manage/plan_container.hpp>

#include <rclcpp/rclcpp.hpp>

#include <map>

namespace fast_planner
{
// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager
{
    // SECTION stable
public:
    FastPlannerManager();
    ~FastPlannerManager();

    struct OtherAgentSnapshot
    {
        Eigen::Vector3d pos{ 0, 0, 0 };
        Eigen::Vector3d vel{ 0, 0, 0 };
        double          yaw{ 0.0 };
        bool            is_static{ false };
    };

    /* main planning interface */
    bool kinodynamicReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc, const Eigen::Vector3d& end_pt,
                           const Eigen::Vector3d& end_vel, const double& time_lb = -1);
    void planExploreTraj(const vector<Eigen::Vector3d>& tour, const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc, const double& time_lb = -1);
    void ensurePositionTrajMinDuration(const double min_duration);
    bool planGlobalTraj(const Eigen::Vector3d& start_pos);
    bool topoReplan(bool collide);

    void planYaw(const Eigen::Vector3d& start_yaw);
    void planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw, bool lookfwd, const double& relax_time);

    void initPlanModules(rclcpp::Node::SharedPtr node);
    void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);

    bool checkTrajCollision(double& distance, Eigen::Vector3d& ugv_odom);
    bool checkTrajCollision(double& distance);

    // FSM 注入当前里程计朝向，用于 checkTrajCollision 低速 fallback 的 footprint 方向估计。
    void setCurrentOdomYaw(double yaw)
    {
        current_odom_yaw_       = yaw;
        current_odom_yaw_valid_ = true;
    }

    // 注入停止状态的队友实际位置（非 EXEC_TRAJ），规划前由 exploration_manager 更新；
    // 优化器 swarm 项对它们合成静止轨迹，替代旧轨迹的错位采样。
    void setStaticPeerPoses(const std::map<int, Eigen::Vector3d>& poses)
    {
        static_peer_poses_ = poses;
    }

    void calcNextYaw(const double& last_yaw, double& yaw);

    // 返回：true = 安全；false = 存在碰撞风险。
    // self_yield = true 表示"当前无人机应该让行/减速或停车"，false 表示"当前无人机优先通行"。
    bool checkSwarmCollision(const int& other_id, const Eigen::Vector3d& self_odom_pos, const Eigen::Vector3d& self_odom_vel,
                             const rclcpp::Time& self_odom_stamp, bool& self_yield);

    /// Dynamic traj check, or static footprint vs own trajectory when other is parked/low-speed.
    bool checkSwarmCollisionHybrid(const int& other_id, const Eigen::Vector3d& self_odom_pos, const Eigen::Vector3d& self_odom_vel,
                                   const rclcpp::Time& self_odom_stamp, const OtherAgentSnapshot& other, bool& self_yield);

    PlanParameters               pp_;
    LocalTrajData                local_data_;
    GlobalTrajData               global_data_;
    MidPlanData                  plan_data_;
    EDTEnvironment::Ptr          edt_environment_;
    rclcpp::Node::SharedPtr      node_;  // ROS2 节点句柄
    unique_ptr<Astar>            path_finder_;
    unique_ptr<KinodynamicAstar> kino_path_finder_;
    unique_ptr<TopologyPRM>      topo_prm_;

    SwarmData swarm_traj_data_;  // Data added for swarm trajectory planning

    // Cached odom yaw injected by FSM (used only when trajectory-based yaw is unavailable).
    double current_odom_yaw_{ 0.0 };
    bool   current_odom_yaw_valid_{ false };

    // 停止状态的队友位置快照（id -> 实际位置），供轨迹优化避让使用。
    std::map<int, Eigen::Vector3d> static_peer_poses_;

private:
    /* main planning algorithms & modules */

    shared_ptr<SDFMap>            sdf_map_;
    vector<BsplineOptimizer::Ptr> bspline_optimizers_;

    void updateTrajInfo();

    // 收集优化器 swarm 避让用的队友轨迹：停止车用实际位置合成静止轨迹，移动车用原轨迹。
    void collectSwarmTrajsForOpt(vector<NonUniformBspline>& trajs);

    // topology guided optimization

    void findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end, vector<Eigen::Vector3d>& start_pts,
                            vector<Eigen::Vector3d>& end_pts);

    void            optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path, int traj_id);
    Eigen::MatrixXd paramLocalTraj(double start_t, double& dt, double& duration);
    Eigen::MatrixXd reparamLocalTraj(const double& start_t, const double& duration, const double& dt);

    void selectBestTraj(NonUniformBspline& traj);
    void refineTraj(NonUniformBspline& best_traj);
    void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc);

    // Heading planning

    // !SECTION stable

    // SECTION developing

public:
    typedef shared_ptr<FastPlannerManager> Ptr;

    void planYawActMap(const Eigen::Vector3d& start_yaw);
    void test();
    void searchFrontier(const Eigen::Vector3d& p);

private:
    unique_ptr<FrontierFinder> frontier_finder_;
    unique_ptr<HeadingPlanner> heading_planner_;
    unique_ptr<VisibilityUtil> visib_util_;

    // Benchmark method, local exploration
public:
    bool localExplore(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt);

    // !SECTION
};
}  // namespace fast_planner

#endif