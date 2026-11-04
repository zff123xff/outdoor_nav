// #include <fstream>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>

namespace fast_planner
{
// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager()
{
    std::cout << "des manager" << std::endl;
}

void FastPlannerManager::initPlanModules(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    /* read algorithm parameters */

    if (node->has_parameter("manager/max_vel"))
    {
        pp_.max_vel_ = node->get_parameter("manager/max_vel").as_double();
    }
    else
    {
        pp_.max_vel_ = node->declare_parameter<double>("manager/max_vel", -1.0);
    }
    if (node->has_parameter("manager/max_acc"))
    {
        pp_.max_acc_ = node->get_parameter("manager/max_acc").as_double();
    }
    else
    {
        pp_.max_acc_ = node->declare_parameter<double>("manager/max_acc", -1.0);
    }
    if (node->has_parameter("manager/max_jerk"))
    {
        pp_.max_jerk_ = node->get_parameter("manager/max_jerk").as_double();
    }
    else
    {
        pp_.max_jerk_ = node->declare_parameter<double>("manager/max_jerk", -1.0);
    }

    // Keep ROS1 fallback semantics: accept_* defaults depend on max_*.
    if (node->has_parameter("manager/accept_vel"))
    {
        pp_.accept_vel_ = node->get_parameter("manager/accept_vel").as_double();
    }
    else
    {
        pp_.accept_vel_ = node->declare_parameter<double>("manager/accept_vel", pp_.max_vel_ + 0.5);
    }
    if (node->has_parameter("manager/accept_acc"))
    {
        pp_.accept_acc_ = node->get_parameter("manager/accept_acc").as_double();
    }
    else
    {
        pp_.accept_acc_ = node->declare_parameter<double>("manager/accept_acc", pp_.max_acc_ + 0.5);
    }

    if (node->has_parameter("manager/max_yawdot"))
    {
        pp_.max_yawdot_ = node->get_parameter("manager/max_yawdot").as_double();
    }
    else
    {
        pp_.max_yawdot_ = node->declare_parameter<double>("manager/max_yawdot", -1.0);
    }
    if (node->has_parameter("manager/dynamic_environment"))
    {
        pp_.dynamic_ = node->get_parameter("manager/dynamic_environment").as_int();
    }
    else
    {
        pp_.dynamic_ = node->declare_parameter<int>("manager/dynamic_environment", -1);
    }
    if (node->has_parameter("manager/clearance_threshold"))
    {
        pp_.clearance_ = node->get_parameter("manager/clearance_threshold").as_double();
    }
    else
    {
        pp_.clearance_ = node->declare_parameter<double>("manager/clearance_threshold", -1.0);
    }
    if (node->has_parameter("manager/local_segment_length"))
    {
        pp_.local_traj_len_ = node->get_parameter("manager/local_segment_length").as_double();
    }
    else
    {
        pp_.local_traj_len_ = node->declare_parameter<double>("manager/local_segment_length", -1.0);
    }
    if (node->has_parameter("manager/control_points_distance"))
    {
        pp_.ctrl_pt_dist = node->get_parameter("manager/control_points_distance").as_double();
    }
    else
    {
        pp_.ctrl_pt_dist = node->declare_parameter<double>("manager/control_points_distance", -1.0);
    }
    if (node->has_parameter("manager/bspline_degree"))
    {
        pp_.bspline_degree_ = node->get_parameter("manager/bspline_degree").as_int();
    }
    else
    {
        pp_.bspline_degree_ = node->declare_parameter<int>("manager/bspline_degree", 3);
    }
    if (node->has_parameter("manager/min_time"))
    {
        pp_.min_time_ = node->get_parameter("manager/min_time").as_bool();
    }
    else
    {
        pp_.min_time_ = node->declare_parameter<bool>("manager/min_time", false);
    }
    if (node->has_parameter("manager/relax_time1"))
    {
        pp_.relax_time1_ = node->get_parameter("manager/relax_time1").as_double();
    }
    else
    {
        pp_.relax_time1_ = node->declare_parameter<double>("manager/relax_time1", 0.5);
    }
    if (node->has_parameter("manager/relax_time2"))
    {
        pp_.relax_time2_ = node->get_parameter("manager/relax_time2").as_double();
    }
    else
    {
        pp_.relax_time2_ = node->declare_parameter<double>("manager/relax_time2", 0.5);
    }

    node->declare_parameter<double>("manager/vehicle_length", 1.045);
    node->declare_parameter<double>("manager/vehicle_width", 0.78);
    pp_.vehicle_length_ = node->get_parameter("manager/vehicle_length").as_double();
    pp_.vehicle_width_  = node->get_parameter("manager/vehicle_width").as_double();

    node->declare_parameter<bool>("manager/use_geometric_path", false);
    node->declare_parameter<bool>("manager/use_kinodynamic_path", false);
    node->declare_parameter<bool>("manager/use_topo_path", false);
    node->declare_parameter<bool>("manager/use_optimization", false);
    node->declare_parameter<bool>("manager/use_active_perception", false);

    bool use_geometric_path    = node->get_parameter("manager/use_geometric_path").as_bool();
    bool use_kinodynamic_path  = node->get_parameter("manager/use_kinodynamic_path").as_bool();
    bool use_topo_path         = node->get_parameter("manager/use_topo_path").as_bool();
    bool use_optimization      = node->get_parameter("manager/use_optimization").as_bool();
    bool use_active_perception = node->get_parameter("manager/use_active_perception").as_bool();

    local_data_.traj_id_ = 0;
    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(node);
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setMap(sdf_map_);

    if (use_geometric_path)
    {
        path_finder_.reset(new Astar);
        // path_finder_->setParam(node);
        // path_finder_->setEnvironment(edt_environment_);
        // path_finder_->init();
        path_finder_->init(node, edt_environment_);
    }

    if (use_kinodynamic_path)
    {
        kino_path_finder_.reset(new KinodynamicAstar);
        kino_path_finder_->setParam(node);
        kino_path_finder_->setEnvironment(edt_environment_);
        kino_path_finder_->init();
    }

    if (use_optimization)
    {
        bspline_optimizers_.resize(10);
        for (int i = 0; i < 10; ++i)
        {
            bspline_optimizers_[i].reset(new BsplineOptimizer);
            bspline_optimizers_[i]->setParam(node);
            bspline_optimizers_[i]->setEnvironment(edt_environment_);
            bspline_optimizers_[i]->setVehicleFootprint(pp_.vehicle_length_, pp_.vehicle_width_);
        }
    }

    if (use_topo_path)
    {
        topo_prm_.reset(new TopologyPRM);
        topo_prm_->setEnvironment(edt_environment_);
        topo_prm_->init(node);
    }

    if (use_active_perception)
    {
        frontier_finder_.reset(new FrontierFinder(edt_environment_, node));
        // heading_planner_.reset(new HeadingPlanner(node));
        // heading_planner_->setMap(sdf_map_);
        // visib_util_.reset(new VisibilityUtil(node));
        // visib_util_->setEDTEnvironment(edt_environment_);
        // plan_data_.view_cons_.idx_ = -1;
    }
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints)
{
    plan_data_.global_waypoints_ = waypoints;
}

namespace
{

    // 车矩形四角相对车中心（车体坐标系下 (±hl, ±hw)），旋转 yaw 后加到 pos 上。
    // occ_query: 返回 true 表示该点为“障碍”。使用外部注入 predicate，避免耦合具体占据接口。
    template <typename Predicate> bool VehicleFootprintOccupiedPred(const Eigen::Vector3d& pos, double yaw, double L, double W, Predicate occ_query)
    {
        const double                         hl      = 0.5 * std::max(1e-3, L);
        const double                         hw      = 0.5 * std::max(1e-3, W);
        const double                         c       = std::cos(yaw);
        const double                         s       = std::sin(yaw);
        const std::array<Eigen::Vector2d, 4> corners = { Eigen::Vector2d(pos.x() + hl * c - hw * s, pos.y() + hl * s + hw * c),
                                                         Eigen::Vector2d(pos.x() + hl * c + hw * s, pos.y() + hl * s - hw * c),
                                                         Eigen::Vector2d(pos.x() - hl * c - hw * s, pos.y() - hl * s + hw * c),
                                                         Eigen::Vector2d(pos.x() - hl * c + hw * s, pos.y() - hl * s - hw * c) };
        for (const auto& corner : corners)
        {
            if (occ_query(Eigen::Vector3d(corner.x(), corner.y(), pos.z())))
                return true;
        }
        return false;
    }

}  // namespace

bool FastPlannerManager::checkTrajCollision(double& distance, Eigen::Vector3d& ugv_odom)
{
    double t_now = (node_->now() - local_data_.start_time_).seconds();
    t_now        = std::max(0.0, std::min(t_now, local_data_.duration_));

    Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoorT(t_now);

    // --- static parameter cache (declared once) ---
    static double check_horizon_max = 4.0;
    static double check_horizon_min = 1.5;  // 下限
    static double check_react_time  = 2.0;  // 前瞻= v * react_time + base_margin，再 clamp
    static double check_base_margin = 0.7;  // 与车尺寸同数量级的基础前瞻
    static double base_dt           = 0.06;
    static double safety_margin     = 0.10;  // 车矩形外再加的安全余量，与 obstacles_inflation 解耦
    static double arrival_tol_time  = 0.10;  // 距轨迹结束时间 < 该值时视为“已到达/结束”
    static double nav_z             = 0.3;   // 规划/检测统一高度层（无悬空障碍场景：0.3m 单层）
    static bool   inited            = false;
    if (!inited)
    {
        auto get_or_declare_double = [&](const char* name, double& out) {
            if (node_->has_parameter(name))
                node_->get_parameter(name, out);
            else
                node_->declare_parameter<double>(name, out);
        };
        get_or_declare_double("static_collision.check_horizon", check_horizon_max);
        get_or_declare_double("static_collision.check_horizon_min", check_horizon_min);
        get_or_declare_double("static_collision.react_time", check_react_time);
        get_or_declare_double("static_collision.base_margin", check_base_margin);
        get_or_declare_double("static_collision.base_dt", base_dt);
        get_or_declare_double("static_collision.safety_margin", safety_margin);
        get_or_declare_double("static_collision.arrival_tol_time", arrival_tol_time);
        get_or_declare_double("static_collision.nav_z", nav_z);
        inited = true;
    }

    if (!sdf_map_)
    {
        distance = 0.0;
        return true;
    }

    // 检测高度必须与规划高度一致（fast_exploration_manager 的 z_plan=0.3）：
    // 轨迹/odom 的 z 只表示车体参考高度，不参与占据判定；否则会出现
    // “规划看 0.3m 层、检测看 0.8m 层”的分层不一致。
    cur_pt.z() = nav_z;
    ugv_odom   = cur_pt;

    // (A) 到达终点免检：轨迹已跑到（或超过）末端，交给上层 FSM 走 FINISH/下一目标，
    //     不再由 safety timer 反复触发 replan 造成“到达即抖动”。
    if (local_data_.duration_ <= 1e-3 || t_now >= local_data_.duration_ - arrival_tol_time)
    {
        distance = 0.0;
        return true;
    }

    const double L         = std::max(1e-3, pp_.vehicle_length_);
    const double W         = std::max(1e-3, pp_.vehicle_width_);
    const double half_diag = 0.5 * std::hypot(L, W);
    // 车矩形能贴到的最远角点距离 + 用户可配置的额外安全余量（与 SDFMap 的 obstacles_inflation 解耦）
    const double clearance_thresh = half_diag + std::max(0.0, safety_margin);
    const double max_spatial      = 0.5 * std::min(L, W);
    const double yaw_dur          = local_data_.yaw_traj_.getTimeSum();
    const double res              = sdf_map_->getResolution();

    // (C) 更鲁棒的 yaw 采样：yaw_traj → 速度方向 → 注入的 odom_yaw → 位置差分 → 0
    auto sample_yaw = [&](double ts) {
        ts = std::max(0.0, std::min(ts, local_data_.duration_));
        if (yaw_dur > 1e-6)
            return local_data_.yaw_traj_.evaluateDeBoorT(std::min(ts, yaw_dur))[0];
        Eigen::Vector3d v = local_data_.velocity_traj_.evaluateDeBoorT(ts);
        if (v.head<2>().norm() > 0.05)
            return std::atan2(v(1), v(0));
        if (current_odom_yaw_valid_)
            return current_odom_yaw_;
        const double    eps = 0.03;
        const double    t1  = std::min(ts + eps, local_data_.duration_);
        Eigen::Vector2d d   = (local_data_.position_traj_.evaluateDeBoorT(t1) - local_data_.position_traj_.evaluateDeBoorT(ts)).head<2>();
        if (d.norm() > 1e-4)
            return std::atan2(d(1), d(0));
        return 0.0;
    };

    // (B) 原始占据判据：越界视为“安全”（避免 -1 被误判为占据），仅 OCCUPIED 才算障碍。
    //     不再使用 getInflateOccupancy → 避免与 sdf_map/obstacles_inflation 叠加造成双重膨胀。
    auto raw_occupied = [&](const Eigen::Vector3d& pt) {
        if (!sdf_map_->isInMap(pt))
            return false;
        return sdf_map_->getOccupancy(pt) == SDFMap::OCCUPIED;
    };

    auto pose_occupied = [&](const Eigen::Vector3d& pt, double ts) {
        // 1) 中心点先看是否直接落在实体上（越界不算碰撞，见 raw_occupied）
        if (raw_occupied(pt))
            return true;
        // 2) 用 ESDF 原始距离作粗筛：d>=clearance_thresh 直接放行；d<0 表示越界或障碍物内部
        const double d = sdf_map_->getDistance(pt);
        if (d < 0.0)
        {
            // 地图外：未知区域，保守判碰；地图内 ESDF 负值：障碍物内部，判碰
            return true;
        }
        if (d + 0.5 * res >= clearance_thresh)  // 加半格容忍离散化
            return false;
        // 3) 距离进入车矩形邻域后，用四角原始占据精细判定
        return VehicleFootprintOccupiedPred(pt, sample_yaw(ts), L + 2.0 * safety_margin, W + 2.0 * safety_margin, raw_occupied);
    };

    if (pose_occupied(cur_pt, t_now))
    {
        distance = 0.0;
        RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "[Collision] collision at current pose: [%.2f, %.2f, %.2f]", cur_pt.x(), cur_pt.y(), cur_pt.z());
        return false;
    }

    // (D) 速度自适应前瞻：check_horizon = clamp(v*react_time + base_margin, min, max)
    const double v_head        = local_data_.velocity_traj_.evaluateDeBoorT(t_now).head<2>().norm();
    const double check_horizon = std::max(check_horizon_min, std::min(check_horizon_max, v_head * check_react_time + check_base_margin));

    double    radius   = 0.0;
    double    fut_t    = base_dt;
    const int max_iter = 500;

    for (int iter = 0; iter < max_iter && radius < check_horizon && t_now + fut_t < local_data_.duration_; ++iter)
    {
        const double    ts     = t_now + fut_t;
        Eigen::Vector3d fut_pt = local_data_.position_traj_.evaluateDeBoorT(ts);
        fut_pt.z()             = nav_z;  // 与规划高度层一致

        if (pose_occupied(fut_pt, ts))
        {
            distance = (fut_pt - cur_pt).norm();
            RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "[Collision] collision at: [%.2f, %.2f, %.2f], dist: %.2f", fut_pt.x(), fut_pt.y(), fut_pt.z(),
                        distance);
            return false;
        }

        radius             = (fut_pt - cur_pt).norm();
        const double d_obs = sdf_map_->getDistance(fut_pt);
        double       dt    = base_dt;
        // ESDF<0 (越界/无效) 时按默认步长；有效距离再自适应
        if (d_obs >= 0.0)
        {
            if (d_obs > clearance_thresh + 1.5)
                dt = std::min(0.05, base_dt * 2.0);
            else if (d_obs < clearance_thresh + 0.3)
                dt = base_dt * 0.5;
        }

        const double spd = local_data_.velocity_traj_.evaluateDeBoorT(ts).head<2>().norm();
        dt               = std::min(dt, max_spatial / std::max(spd, 0.05));
        fut_t += dt;
    }

    distance = radius;
    return true;
}

bool FastPlannerManager::checkTrajCollision(double& distance)
{
    Eigen::Vector3d ugv_odom;
    return checkTrajCollision(distance, ugv_odom);
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel, const Eigen::Vector3d& start_acc,
                                           const Eigen::Vector3d& end_pt, const Eigen::Vector3d& end_vel, const double& time_lb)
{
    // std::cout << "[Kino replan]: start: " << start_pt.transpose() << ", " << start_vel.transpose()
    // << ", "
    //           << start_acc.transpose() << ", goal:" << end_pt.transpose() << ", " <<
    //           end_vel.transpose()
    //           << endl;

    if ((start_pt - end_pt).norm() < 1e-2)
    {
        cout << "Close goal" << endl;
        return false;
    }

    Eigen::Vector3d init_pos = start_pt;
    Eigen::Vector3d init_vel = start_vel;
    Eigen::Vector3d init_acc = start_acc;

    // Kinodynamic path searching

    auto t1 = node_->now();

    kino_path_finder_->reset();
    int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);
    if (status == KinodynamicAstar::NO_PATH)
    {
        RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "search 1 fail");
        // Retry
        kino_path_finder_->reset();
        status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);
        if (status == KinodynamicAstar::NO_PATH)
        {
            cout << "[Kino replan]: Can't find path." << endl;
            return false;
        }
    }
    plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

    double t_search = (node_->now() - t1).seconds();
    t1              = node_->now();

    // Parameterize path to B-spline
    double                  ts = pp_.ctrl_pt_dist / pp_.max_vel_;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

    // std::cout << "point set:" << std::endl;
    // for (auto pt : point_set) std::cout << pt.transpose() << std::endl;
    // std::cout << "derivative:" << std::endl;
    // for (auto dr : start_end_derivatives) std::cout << dr.transpose() << std::endl;

    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, pp_.bspline_degree_, ctrl_pts);
    NonUniformBspline init(ctrl_pts, pp_.bspline_degree_, ts);

    // B-spline-based optimization
    int cost_function = BsplineOptimizer::NORMAL_PHASE;
    if (pp_.min_time_)
        cost_function |= BsplineOptimizer::MINTIME;
    vector<Eigen::Vector3d> start, end;
    init.getBoundaryStates(2, 0, start, end);
    bspline_optimizers_[0]->setBoundaryStates(start, end);
    if (time_lb > 0)
        bspline_optimizers_[0]->setTimeLowerBound(time_lb);

    // Swarm collision avoidance term
    cost_function |= BsplineOptimizer::SWARM;
    // Static obstacle collision term (push trajectory away from obstacles)
    cost_function |= BsplineOptimizer::OBS_COLLISION;
    vector<NonUniformBspline> trajs;
    collectSwarmTrajsForOpt(trajs);
    bspline_optimizers_[0]->setSwarmTrajs(trajs);

    bspline_optimizers_[0]->optimize(ctrl_pts, ts, cost_function, 1, 1);
    local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, ts);

    vector<Eigen::Vector3d> start2, end2;
    local_data_.position_traj_.getBoundaryStates(2, 0, start2, end2);
    // std::cout << "State error: (" << (start2[0] - start[0]).norm() << ", "
    //           << (start2[1] - start[1]).norm() << ", " << (start2[2] - start[2]).norm() << ")"
    //           << std::endl;

    double t_opt = (node_->now() - t1).seconds();
    // RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "Kino t: %lf, opt: %lf", t_search, t_opt);

    updateTrajInfo();
    return true;
}

/**
 * @brief 轨迹优化
 * @param 离散途径点，当前速度，加速度，最小转头时间下界
 */
void FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3d>& tour, const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& cur_acc,
                                         const double& time_lb)
{

    if (tour.size() < 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("plan_manage"), "Empty path to traj planner");
        return;
    }

    // Generate traj through waypoints-based method
    const int       pt_num = tour.size();
    Eigen::MatrixXd pos(pt_num, 3);
    for (int i = 0; i < pt_num; ++i)
        pos.row(i) = tour[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd times(pt_num - 1);
    // SNN / grid paths often repeat the same cell; zero-length segments make waypointsTraj ill-conditioned
    // and can crash the downstream B-spline optimizer (SIGSEGV).
    constexpr double kMinWaypointSegLenM = 1e-3;
    for (int i = 0; i < pt_num - 1; ++i)
    {
        const double dist = (pos.row(i + 1) - pos.row(i)).norm();
        times(i)          = std::max(dist, kMinWaypointSegLenM) / (pp_.max_vel_ * 0.5);
    }

    PolynomialTraj init_traj;
    PolynomialTraj::waypointsTraj(pos, cur_vel, zero, cur_acc, zero, times, init_traj);

    // B-spline-based optimization
    vector<Vector3d> points, boundary_deri;
    double           duration = init_traj.getTotalTime();
    int              seg_num  = init_traj.getLength() / pp_.ctrl_pt_dist;
    seg_num                   = max(8, seg_num);
    double dt                 = duration / double(seg_num);

    // std::cout << "duration: " << duration << ", seg_num: " << seg_num << ", dt: " << dt <<
    // std::endl;

    for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
        points.push_back(init_traj.evaluate(ts, 0));
    boundary_deri.push_back(init_traj.evaluate(0.0, 1));
    boundary_deri.push_back(init_traj.evaluate(duration, 1));
    boundary_deri.push_back(init_traj.evaluate(0.0, 2));
    boundary_deri.push_back(init_traj.evaluate(duration, 2));

    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(dt, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
    if (ctrl_pts.rows() < pp_.bspline_degree_ + 1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("plan_manage"),
                     "[planExploreTraj] invalid B-spline ctrl_pts rows=%ld (tour_pts=%d, sample_pts=%zu, duration=%.4f, dt=%.4f)",
                     static_cast<long>(ctrl_pts.rows()), pt_num, points.size(), duration, dt);
        return;
    }
    NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);

    int cost_func = BsplineOptimizer::NORMAL_PHASE;
    if (pp_.min_time_)
        cost_func |= BsplineOptimizer::MINTIME;

    vector<Vector3d> start, end;
    tmp_traj.getBoundaryStates(2, 0, start, end);
    start[0] = tour[0];
    start[1] = cur_vel;
    start[2] = cur_acc;
    bspline_optimizers_[0]->setBoundaryStates(start, end);
    if (time_lb > 0)
        bspline_optimizers_[0]->setTimeLowerBound(time_lb);

    // Swarm collision avoidance term
    cost_func |= BsplineOptimizer::SWARM;
    // Static obstacle collision term (push trajectory away from obstacles)
    cost_func |= BsplineOptimizer::OBS_COLLISION;
    vector<NonUniformBspline> trajs;
    collectSwarmTrajsForOpt(trajs);
    bspline_optimizers_[0]->setSwarmTrajs(trajs);

    bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
    local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

    updateTrajInfo();
}

void FastPlannerManager::collectSwarmTrajsForOpt(vector<NonUniformBspline>& trajs)
{
    trajs.clear();

    // 停止的队友车（非 EXEC_TRAJ）不广播新轨迹，其旧轨迹采样点与实际停点错位；
    // 用实际位置合成一条静止轨迹，让优化器 swarm 项按真实停点避让。
    const double now = node_->now().seconds();
    for (const auto& entry : static_peer_poses_)
    {
        Eigen::MatrixXd ctrl(6, 3);
        for (int r = 0; r < 6; ++r)
            ctrl.row(r) = entry.second.transpose();
        NonUniformBspline static_traj(ctrl, 3, 0.2);  // 有效时长 (6-3)*0.2 = 0.6s，全程同位置
        static_traj.start_time_ = now - 1.0;
        static_traj.duration_   = 0.6;
        trajs.push_back(static_traj);
    }

    // 其余收到轨迹的队友保持原轨迹避让（移动中）。
    for (size_t i = 0; i < swarm_traj_data_.swarm_trajs_.size(); ++i)
    {
        if (!swarm_traj_data_.receive_flags_[i])
            continue;
        if (static_peer_poses_.count(static_cast<int>(i) + 1))
            continue;  // 停止车已由静止轨迹替代，旧轨迹不参与避让
        trajs.push_back(swarm_traj_data_.swarm_trajs_[i]);
    }
}

void FastPlannerManager::ensurePositionTrajMinDuration(const double min_duration)
{
    if (min_duration <= 0)
        return;
    const double dur = local_data_.position_traj_.getTimeSum();
    if (dur >= min_duration)
        return;
    local_data_.position_traj_.lengthenTime(min_duration / dur);
    updateTrajInfo();
}

// !SECTION

// SECTION topological replanning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos)
{
    plan_data_.clearTopoPaths();

    // Generate global reference trajectory
    vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
    if (points.size() == 0)
        std::cout << "no global waypoints!" << std::endl;

    points.insert(points.begin(), start_pos);

    // Insert intermediate points if two waypoints are too far
    vector<Eigen::Vector3d> inter_points;
    const double            dist_thresh = 4.0;

    for (int i = 0; i < points.size() - 1; ++i)
    {
        inter_points.push_back(points.at(i));
        double dist = (points.at(i + 1) - points.at(i)).norm();
        if (dist > dist_thresh)
        {
            int id_num = floor(dist / dist_thresh) + 1;
            for (int j = 1; j < id_num; ++j)
            {
                Eigen::Vector3d inter_pt = points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                inter_points.push_back(inter_pt);
            }
        }
    }
    inter_points.push_back(points.back());

    // At least 3 waypoints are required to solve the problem
    if (inter_points.size() == 2)
    {
        Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
        inter_points.insert(inter_points.begin() + 1, mid);
    }

    int             pt_num = inter_points.size();
    Eigen::MatrixXd pos(pt_num, 3);
    for (int i = 0; i < pt_num; ++i)
        pos.row(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
        time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

    time(0) += pp_.max_vel_ / (2 * pp_.max_acc_);
    time(time.rows() - 1) += pp_.max_vel_ / (2 * pp_.max_acc_);

    PolynomialTraj gl_traj;
    PolynomialTraj::waypointsTraj(pos, zero, zero, zero, zero, time, gl_traj);

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    // truncate a local trajectory

    double            dt, duration;
    Eigen::MatrixXd   ctrl_pts = paramLocalTraj(0.0, dt, duration);
    NonUniformBspline bspline(ctrl_pts, pp_.bspline_degree_, dt);

    std::cout << "ctrl pt: " << ctrl_pts.rows() << std::endl;

    global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
    local_data_.position_traj_ = bspline;
    local_data_.start_time_    = time_now;
    RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "global trajectory generated.");

    updateTrajInfo();

    return true;
}

bool FastPlannerManager::topoReplan(bool collide)
{
    auto t1 = node_->now();
    auto t2 = node_->now();

    /* truncate a new local segment for replanning */
    auto   time_now = node_->now();
    double t_now    = (time_now - global_data_.global_start_time_).seconds();
    double local_traj_dt, local_traj_duration;

    Eigen::MatrixXd   ctrl_pts = paramLocalTraj(t_now, local_traj_dt, local_traj_duration);
    NonUniformBspline init_traj(ctrl_pts, pp_.bspline_degree_, local_traj_dt);
    local_data_.start_time_ = time_now;

    std::cout << "dt: " << local_traj_dt << ", dur: " << local_traj_duration << std::endl;

    if (!collide)
    {
        // No collision detected, but we can further refine the trajectory
        refineTraj(init_traj);
        double time_change         = init_traj.getTimeSum() - local_traj_duration;
        local_data_.position_traj_ = init_traj;
        global_data_.setLocalTraj(local_data_.position_traj_, t_now, local_traj_duration + time_change + t_now, time_change);
        // local_data_.position_traj_ = init_traj;
        // global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);
    }
    else
    {
        // Find topologically distinctive path and guide optimization in parallel
        plan_data_.initial_local_segment_ = init_traj;
        vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
        findCollisionRange(colli_start, colli_end, start_pts, end_pts);

        if (colli_start.size() == 1 && colli_end.size() == 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "Init traj ends in obstacle, no replanning.");
            local_data_.position_traj_ = init_traj;
            global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);
        }
        else
        {
            // Call topological replanning when local segment is in collision
            /* Search topological distinctive paths */
            RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "[Topo]: ---------");
            plan_data_.clearTopoPaths();
            list<GraphNode::Ptr>            graph;
            vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
            topo_prm_->findTopoPaths(colli_start.front(), colli_end.back(), start_pts, end_pts, graph, raw_paths, filtered_paths, select_paths);

            if (select_paths.size() == 0)
            {
                RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "No path.");
                return false;
            }
            plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

            /* Optimize trajectory using different topo guiding paths */
            RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "[Optimize]: ---------");
            t1 = node_->now();

            plan_data_.topo_traj_pos1_.resize(select_paths.size());
            plan_data_.topo_traj_pos2_.resize(select_paths.size());
            vector<thread> optimize_threads;
            for (int i = 0; i < select_paths.size(); ++i)
            {
                optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now, local_traj_duration, select_paths[i], i);
                // optimizeTopoBspline(t_now, local_traj_duration,
                // select_paths[i], origin_len, i);
            }
            for (int i = 0; i < select_paths.size(); ++i)
                optimize_threads[i].join();

            double t_opt = (node_->now() - t1).seconds();
            cout << "[planner]: optimization time: " << t_opt << endl;

            NonUniformBspline best_traj;
            selectBestTraj(best_traj);
            refineTraj(best_traj);
            double time_change = best_traj.getTimeSum() - local_traj_duration;

            local_data_.position_traj_ = best_traj;
            global_data_.setLocalTraj(local_data_.position_traj_, t_now, local_traj_duration + time_change + t_now, time_change);
        }
    }
    updateTrajInfo();

    double tr = (node_->now() - time_now).seconds();
    RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "Replan time: %lf", tr);

    return true;
}

void FastPlannerManager::selectBestTraj(NonUniformBspline& traj)
{
    // sort by jerk
    vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
    sort(trajs.begin(), trajs.end(), [](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
    traj = trajs[0];
}

void FastPlannerManager::refineTraj(NonUniformBspline& best_traj)
{
    auto t1                   = node_->now();
    plan_data_.no_visib_traj_ = best_traj;

    int cost_function = BsplineOptimizer::NORMAL_PHASE;
    if (pp_.min_time_)
        cost_function |= BsplineOptimizer::MINTIME;

    // ViewConstraint view_cons;
    // visib_util_->calcViewConstraint(best_traj, view_cons);
    // plan_data_.view_cons_ = view_cons;
    // if (view_cons.idx_ >= 0)
    // {
    //   cost_function |= BsplineOptimizer::VIEWCONS;
    //   bspline_optimizers_[0]->setViewConstraint(view_cons);
    // }

    // Refine selected best traj
    Eigen::MatrixXd         ctrl_pts = best_traj.getControlPoint();
    double                  dt       = best_traj.getKnotSpan();
    vector<Eigen::Vector3d> start1, end1;
    best_traj.getBoundaryStates(2, 0, start1, end1);

    bspline_optimizers_[0]->setBoundaryStates(start1, end1);
    bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_function, 2, 2);
    best_traj.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

    vector<Eigen::Vector3d> start2, end2;
    best_traj.getBoundaryStates(2, 2, start2, end2);
    for (int i = 0; i < 3; ++i)
        std::cout << "error start: " << (start1[i] - start2[i]).norm() << std::endl;
    for (int i = 0; i < 1; ++i)
        std::cout << "error end  : " << (end1[i] - end2[i]).norm() << std::endl;
}

void FastPlannerManager::updateTrajInfo()
{
    local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_  = local_data_.position_traj_.getTimeSum();

    local_data_.traj_id_ += 1;
}

void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc)
{
    int    prev_num    = bspline.getControlPoint().rows();
    double time_origin = bspline.getTimeSum();

    int seg_num = bspline.getControlPoint().rows() - pp_.bspline_degree_;
    ratio       = min(1.01, ratio);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt              = duration / double(seg_num);
    time_inc        = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
        point_set.push_back(bspline.evaluateDeBoorT(time));
    NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_, pp_.bspline_degree_, ctrl_pts);
    // RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void FastPlannerManager::optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path, int traj_id)
{
    auto t1 = node_->now();

    // Re-parameterize B-spline according to the length of guide path
    int seg_num              = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
    seg_num                  = max(6, seg_num);  // Min number required for optimizing
    double          dt       = duration / double(seg_num);
    Eigen::MatrixXd ctrl_pts = reparamLocalTraj(start_t, duration, dt);

    NonUniformBspline       tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);
    vector<Eigen::Vector3d> start, end;
    tmp_traj.getBoundaryStates(2, 0, start, end);

    // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

    // Discretize the guide path and align it with B-spline control points
    vector<Eigen::Vector3d> tmp_pts, guide_pts;
    if (pp_.bspline_degree_ == 3 || pp_.bspline_degree_ == 5)
    {
        topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2, tmp_pts);
        guide_pts.insert(guide_pts.end(), tmp_pts.begin() + 2, tmp_pts.end() - 2);
        if (guide_pts.size() != int(ctrl_pts.rows()) - 6)
            RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "Incorrect guide for 3 degree");
    }
    else if (pp_.bspline_degree_ == 4)
    {
        topo_prm_->pathToGuidePts(guide_path, int(2 * ctrl_pts.rows()) - 7, tmp_pts);
        for (int i = 0; i < tmp_pts.size(); ++i)
        {
            if (i % 2 == 1 && i >= 5 && i <= tmp_pts.size() - 6)
                guide_pts.push_back(tmp_pts[i]);
        }
        if (guide_pts.size() != int(ctrl_pts.rows()) - 8)
            RCLCPP_WARN(rclcpp::get_logger("plan_manage"), "Incorrect guide for 4 degree");
    }

    // std::cout << "guide pt num: " << guide_pt.size() << std::endl;

    double tm1 = (node_->now() - t1).seconds();
    t1         = node_->now();

    // First phase, path-guided optimization
    bspline_optimizers_[traj_id]->setBoundaryStates(start, end);
    bspline_optimizers_[traj_id]->setGuidePath(guide_pts);
    bspline_optimizers_[traj_id]->optimize(ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);
    plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

    double tm2 = (node_->now() - t1).seconds();
    t1         = node_->now();

    // Second phase, smooth+safety+feasibility
    int cost_func = BsplineOptimizer::NORMAL_PHASE;
    // if (pp_.min_time_)
    //   cost_func |= BsplineOptimizer::MINTIME;
    bspline_optimizers_[traj_id]->setBoundaryStates(start, end);
    bspline_optimizers_[traj_id]->optimize(ctrl_pts, dt, cost_func, 1, 1);
    plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

    double tm3 = (node_->now() - t1).seconds();
    // RCLCPP_INFO(rclcpp::get_logger("plan_manage"), "optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd FastPlannerManager::paramLocalTraj(double start_t, double& dt, double& duration)
{
    vector<Eigen::Vector3d> point_set;
    vector<Eigen::Vector3d> start_end_derivative;
    global_data_.getTrajInfoInSphere(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set, start_end_derivative, dt, duration);

    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, pp_.bspline_degree_, ctrl_pts);
    plan_data_.local_start_end_derivative_ = start_end_derivative;

    return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(const double& start_t, const double& duration, const double& dt)
{
    vector<Eigen::Vector3d> point_set;
    vector<Eigen::Vector3d> start_end_derivative;

    global_data_.getTrajInfoInDuration(start_t, duration, dt, point_set, start_end_derivative);
    plan_data_.local_start_end_derivative_ = start_end_derivative;

    /* parameterization of B-spline */
    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, pp_.bspline_degree_, ctrl_pts);
    // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

    return ctrl_pts;
}

void FastPlannerManager::findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end, vector<Eigen::Vector3d>& start_pts,
                                            vector<Eigen::Vector3d>& end_pts)
{
    bool               last_safe = true, safe;
    double             t_m, t_mp;
    NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
    initial_traj->getTimeSpan(t_m, t_mp);

    /* find range of collision */
    double t_s = -1.0, t_e;
    for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05)
    {
        Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
        safe                = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

        if (last_safe && !safe)
        {
            colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
            if (t_s < 0.0)
                t_s = tc - 0.05;
        }
        else if (!last_safe && safe)
        {
            colli_end.push_back(ptc);
            t_e = tc;
        }

        last_safe = safe;
    }

    if (colli_start.size() == 0)
        return;

    if (colli_start.size() == 1 && colli_end.size() == 0)
        return;

    /* find start and end safe segment */
    double dt = initial_traj->getKnotSpan();
    int    sn = ceil((t_s - t_m) / dt);
    dt        = (t_s - t_m) / sn;

    for (double tc = t_m; tc <= t_s + 1e-4; tc += dt)
    {
        start_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }

    dt = initial_traj->getKnotSpan();
    sn = ceil((t_mp - t_e) / dt);
    dt = (t_mp - t_e) / sn;
    // std::cout << "dt: " << dt << std::endl;
    // std::cout << "sn: " << sn << std::endl;
    // std::cout << "t_m: " << t_m << std::endl;
    // std::cout << "t_mp: " << t_mp << std::endl;
    // std::cout << "t_s: " << t_s << std::endl;
    // std::cout << "t_e: " << t_e << std::endl;

    if (dt > 1e-4)
    {
        for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt)
        {
            end_pts.push_back(initial_traj->evaluateDeBoor(tc));
        }
    }
    else
    {
        end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
    }
}

// !SECTION

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw)
{
    auto t1 = node_->now();
    // calculate waypoints of heading

    auto&  pos      = local_data_.position_traj_;
    double duration = pos.getTimeSum();

    double dt_yaw  = 0.3;
    int    seg_num = ceil(duration / dt_yaw);
    dt_yaw         = duration / seg_num;

    const double            forward_t = 2.0;
    double                  last_yaw  = start_yaw(0);
    vector<Eigen::Vector3d> waypts;
    vector<int>             waypt_idx;

    // seg_num -> seg_num - 1 points for constraint excluding the boundary states

    for (int i = 0; i < seg_num; ++i)
    {
        double          tc = i * dt_yaw;
        Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
        double          tf = min(duration, tc + forward_t);
        Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
        Eigen::Vector3d pd = pf - pc;

        Eigen::Vector3d waypt;
        if (pd.norm() > 1e-6)
        {
            waypt(0) = atan2(pd(1), pd(0));
            waypt(1) = waypt(2) = 0.0;
            calcNextYaw(last_yaw, waypt(0));
        }
        else
        {
            waypt = waypts.back();
        }
        last_yaw = waypt(0);
        waypts.push_back(waypt);
        waypt_idx.push_back(i);
    }

    // calculate initial control points with boundary state constraints

    Eigen::MatrixXd yaw(seg_num + 3, 1);
    yaw.setZero();

    Eigen::Matrix3d states2pts;
    states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
    yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

    Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
    Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
    calcNextYaw(last_yaw, end_yaw(0));
    yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

    // solve
    bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
    int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS | BsplineOptimizer::START | BsplineOptimizer::END;

    vector<Eigen::Vector3d> start = { Eigen::Vector3d(start_yaw[0], 0, 0), Eigen::Vector3d(start_yaw[1], 0, 0), Eigen::Vector3d(start_yaw[2], 0, 0) };
    vector<Eigen::Vector3d> end   = { Eigen::Vector3d(end_yaw[0], 0, 0), Eigen::Vector3d(end_yaw[1], 0, 0), Eigen::Vector3d(end_yaw[2], 0, 0) };
    bspline_optimizers_[1]->setBoundaryStates(start, end);
    bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

    // update traj info
    local_data_.yaw_traj_.setUniformBspline(yaw, pp_.bspline_degree_, dt_yaw);
    local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
    local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

    vector<double> path_yaw;
    for (int i = 0; i < waypts.size(); ++i)
        path_yaw.push_back(waypts[i][0]);
    plan_data_.path_yaw_    = path_yaw;
    plan_data_.dt_yaw_      = dt_yaw;
    plan_data_.dt_yaw_path_ = dt_yaw;

    std::cout << "yaw time: " << (node_->now() - t1).seconds() << std::endl;
}

// only consider smoothness and boundary state
void FastPlannerManager::planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw, bool lookfwd, const double& relax_time)
{
    const int seg_num = 12;
    double    dt_yaw  = local_data_.duration_ / double(seg_num);  // time of B-spline segment

    // double dt_yaw = 0.5;
    // int seg_num = ceil(local_data_.duration_ / dt_yaw);
    // seg_num = max(5, seg_num);  // Restrict the minimal num
    // dt_yaw = local_data_.duration_ / double(seg_num);

    Eigen::Vector3d start_yaw3d = start_yaw;
    // std::cout << "dt_yaw: " << dt_yaw << ", start yaw: " << start_yaw3d[0] << ", end: " << end_yaw
    //           << std::endl;

    while (start_yaw3d[0] < -M_PI)
        start_yaw3d[0] += 2 * M_PI;
    while (start_yaw3d[0] > M_PI)
        start_yaw3d[0] -= 2 * M_PI;
    double last_yaw = start_yaw3d[0];
    int    last_idx = 0;

    // Yaw traj control points
    Eigen::MatrixXd yaw(seg_num + 3, 1);
    yaw.setZero();

    // Initial state
    Eigen::Matrix3d states2pts;
    states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
    yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

    // Add waypoint constraints if look forward is enabled
    vector<Eigen::Vector3d> waypts;
    vector<int>             waypt_idx;
    if (lookfwd)
    {
        const double forward_t  = 2.0;
        const int    relax_num1 = pp_.relax_time1_ / dt_yaw;
        const int    relax_num2 = pp_.relax_time2_ / dt_yaw;
        for (int i = 1; i < seg_num - relax_num2; ++i)
        {
            double          tc = i * dt_yaw;
            Eigen::Vector3d pc = local_data_.position_traj_.evaluateDeBoorT(tc);
            double          tf = min(local_data_.duration_, tc + forward_t);
            Eigen::Vector3d pf = local_data_.position_traj_.evaluateDeBoorT(tf);
            Eigen::Vector3d pd = pf - pc;
            Eigen::Vector3d waypt;
            if (pd.norm() > 1e-6)
            {
                waypt(0) = atan2(pd(1), pd(0));
                waypt(1) = waypt(2) = 0.0;
                calcNextYaw(last_yaw, waypt(0));

                // Check if the waypoint satisfy yawdot constraint
                double dt1 = double(i - last_idx) * dt_yaw;
                double dt2 = double(seg_num + 1 - i) * dt_yaw;

                double diff1 = fabs(waypt(0) - last_yaw);
                double yd1   = min(diff1, 2 * M_PI - diff1) / dt1;

                double tmp2 = end_yaw;
                calcNextYaw(waypt(0), tmp2);
                double diff2 = fabs(waypt(0) - tmp2);
                double yd2   = min(diff2, 2 * M_PI - diff2) / dt2;

                // std::cout << "waypt: " << waypt[0] << ", yd1: " << yd1 << ", yd2: " << yd2 << std::endl;

                if (yd1 < pp_.max_yawdot_ && yd2 < pp_.max_yawdot_)
                {
                    last_yaw = waypt(0);
                    waypts.push_back(waypt);
                    waypt_idx.push_back(i);
                    // std::cout << "add waypt" << std::endl;
                }
                else
                {
                    // std::cout << "ignore waypt" << std::endl;
                }
            }
        }
    }
    // Final state
    Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
    calcNextYaw(last_yaw, end_yaw3d(0));
    yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;

    // Debug rapid change of yaw
    if (fabs(start_yaw3d[0] - end_yaw3d[0]) >= M_PI)
    {
        // RCLCPP_ERROR(rclcpp::get_logger("plan_manage"), "Yaw change rapidly!");
        std::cout << "start yaw: " << start_yaw3d[0] << ", " << end_yaw3d[0] << std::endl;
    }

    // // Interpolate start and end value for smoothness
    // for (int i = 1; i < seg_num; ++i)
    // {
    //   double tc = i * dt_yaw;
    //   Eigen::Vector3d waypt = (1 - double(i) / seg_num) * start_yaw3d + double(i) / seg_num *
    //   end_yaw3d; std::cout << "i: " << i << ", wp: " << waypt[0] << ", "; calcNextYaw(last_yaw,
    //   waypt(0));
    // }
    // std::cout << "" << std::endl;

    auto t1 = node_->now();

    // Call B-spline optimization solver
    int                     cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::START | BsplineOptimizer::END | BsplineOptimizer::WAYPOINTS;
    vector<Eigen::Vector3d> start     = { Eigen::Vector3d(start_yaw3d[0], 0, 0), Eigen::Vector3d(start_yaw3d[1], 0, 0), Eigen::Vector3d(start_yaw3d[2], 0, 0) };
    vector<Eigen::Vector3d> end       = { Eigen::Vector3d(end_yaw3d[0], 0, 0), Eigen::Vector3d(0, 0, 0) };
    bspline_optimizers_[1]->setBoundaryStates(start, end);
    bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
    bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func, 1, 1);

    // std::cout << "2: " << (node_->now() - t1).seconds() << std::endl;

    // Update traj info
    local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
    local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
    local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
    plan_data_.dt_yaw_          = dt_yaw;

    // plan_data_.path_yaw_ = path;
    // plan_data_.dt_yaw_path_ = dt_yaw * subsp;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw)
{
    // round yaw to [-PI, PI]
    double round_last = last_yaw;
    while (round_last < -M_PI)
    {
        round_last += 2 * M_PI;
    }
    while (round_last > M_PI)
    {
        round_last -= 2 * M_PI;
    }

    double diff = yaw - round_last;
    if (fabs(diff) <= M_PI)
    {
        yaw = last_yaw + diff;
    }
    else if (diff > M_PI)
    {
        yaw = last_yaw + diff - 2 * M_PI;
    }
    else if (diff < -M_PI)
    {
        yaw = last_yaw + diff + 2 * M_PI;
    }
}

namespace
{

    double TrajHeadingXY(NonUniformBspline& pos_traj, NonUniformBspline& vel_traj, double t, double duration)
    {
        t                 = std::max(0.0, std::min(t, duration));
        Eigen::Vector3d v = vel_traj.evaluateDeBoorT(t);
        if (v.head<2>().norm() > 0.05)
            return std::atan2(v(1), v(0));
        const double    eps = 0.03;
        double          t1  = std::min(t + eps, duration);
        Eigen::Vector3d p0  = pos_traj.evaluateDeBoorT(t);
        Eigen::Vector3d p1  = pos_traj.evaluateDeBoorT(t1);
        Eigen::Vector2d d   = (p1 - p0).head<2>();
        if (d.norm() > 1e-4)
            return std::atan2(d(1), d(0));
        return 0.0;
    }

    bool ObbOverlap2D(const Eigen::Vector2d& c1, double yaw1, double L1, double W1, const Eigen::Vector2d& c2, double yaw2, double L2, double W2)
    {
        const double hl1 = 0.5 * std::max(0.0, L1), hw1 = 0.5 * std::max(0.0, W1);
        const double hl2 = 0.5 * std::max(0.0, L2), hw2 = 0.5 * std::max(0.0, W2);

        const double r1 = std::hypot(hl1, hw1);
        const double r2 = std::hypot(hl2, hw2);
        if ((c1 - c2).squaredNorm() > (r1 + r2) * (r1 + r2))
            return false;

        const double c1c = std::cos(yaw1), c1s = std::sin(yaw1);
        const double c2c = std::cos(yaw2), c2s = std::sin(yaw2);

        const Eigen::Vector2d a1x(c1c, c1s);
        const Eigen::Vector2d a1y(-c1s, c1c);
        const Eigen::Vector2d a2x(c2c, c2s);
        const Eigen::Vector2d a2y(-c2s, c2c);

        const Eigen::Vector2d t = c2 - c1;

        auto projRadius = [](double hl, double hw, const Eigen::Vector2d& ax, const Eigen::Vector2d& ay, const Eigen::Vector2d& axis) -> double {
            return hl * std::abs(ax.dot(axis)) + hw * std::abs(ay.dot(axis));
        };

        const Eigen::Vector2d axes[4] = { a1x, a1y, a2x, a2y };
        for (int i = 0; i < 4; ++i)
        {
            const Eigen::Vector2d axis = axes[i];
            const double          dist = std::abs(t.dot(axis));
            const double          rA   = projRadius(hl1, hw1, a1x, a1y, axis);
            const double          rB   = projRadius(hl2, hw2, a2x, a2y, axis);
            if (dist > rA + rB)
                return false;
        }
        return true;
    }

}  // namespace

/**
 * @brief 检查当前无人机轨迹与指定 ID 无人机轨迹之间是否存在碰撞风险，并给出"是否由当前无人机让行"的建议。
 * @return true 如果轨迹安全 (无碰撞风险)，false 如果检测到碰撞风险。
 */
bool FastPlannerManager::checkSwarmCollision(const int& other_id, const Eigen::Vector3d& self_odom_pos, const Eigen::Vector3d& self_odom_vel,
                                             const rclcpp::Time& self_odom_stamp, bool& self_yield)
{
    self_yield = false;

    // 1) 基础检查
    if (other_id <= 0 || other_id > (int)swarm_traj_data_.receive_flags_.size())
        return true;
    if (!swarm_traj_data_.receive_flags_[other_id - 1])
        return true;

    // 2) 配置参数 (保持 static 提高效率，全部可 ROS 调参)
    struct Config
    {
        double robot_radius{ 0.6 }, safe_margin{ 0.1 }, min_dt{ 0.01 }, max_dt{ 0.08 };
        double horizon{ 2.0 }, ttc_threshold{ 0.8 }, odom_delay_cap{ 0.25 };
        double delay_gain{ 1.0 }, bias_gain{ 0.8 }, box_inflation{ 0.0 };
        double ahead_eps{ 0.25 };  // m，沿航向判定「明显在前/在后」
    };

    static Config cfg;
    static bool   initialized = false;
    if (!initialized)
    {
        if (node_->has_parameter("swarm_collision/robot_radius"))
            cfg.robot_radius = node_->get_parameter("swarm_collision/robot_radius").as_double();
        else
            cfg.robot_radius = node_->declare_parameter<double>("swarm_collision/robot_radius", cfg.robot_radius);
        if (node_->has_parameter("swarm_collision/safe_margin"))
            cfg.safe_margin = node_->get_parameter("swarm_collision/safe_margin").as_double();
        else
            cfg.safe_margin = node_->declare_parameter<double>("swarm_collision/safe_margin", cfg.safe_margin);
        if (node_->has_parameter("swarm_collision/horizon"))
            cfg.horizon = node_->get_parameter("swarm_collision/horizon").as_double();
        else
            cfg.horizon = node_->declare_parameter<double>("swarm_collision/horizon", cfg.horizon);
        if (node_->has_parameter("swarm_collision/ttc_threshold"))
            cfg.ttc_threshold = node_->get_parameter("swarm_collision/ttc_threshold").as_double();
        else
            cfg.ttc_threshold = node_->declare_parameter<double>("swarm_collision/ttc_threshold", cfg.ttc_threshold);
        if (node_->has_parameter("swarm_collision/odom_delay_cap"))
            cfg.odom_delay_cap = node_->get_parameter("swarm_collision/odom_delay_cap").as_double();
        else
            cfg.odom_delay_cap = node_->declare_parameter<double>("swarm_collision/odom_delay_cap", cfg.odom_delay_cap);
        if (node_->has_parameter("swarm_collision/delay_gain"))
            cfg.delay_gain = node_->get_parameter("swarm_collision/delay_gain").as_double();
        else
            cfg.delay_gain = node_->declare_parameter<double>("swarm_collision/delay_gain", cfg.delay_gain);
        if (node_->has_parameter("swarm_collision/bias_gain"))
            cfg.bias_gain = node_->get_parameter("swarm_collision/bias_gain").as_double();
        else
            cfg.bias_gain = node_->declare_parameter<double>("swarm_collision/bias_gain", cfg.bias_gain);
        if (node_->has_parameter("swarm_collision/min_dt"))
            cfg.min_dt = node_->get_parameter("swarm_collision/min_dt").as_double();
        else
            cfg.min_dt = node_->declare_parameter<double>("swarm_collision/min_dt", cfg.min_dt);
        if (node_->has_parameter("swarm_collision/max_dt"))
            cfg.max_dt = node_->get_parameter("swarm_collision/max_dt").as_double();
        else
            cfg.max_dt = node_->declare_parameter<double>("swarm_collision/max_dt", cfg.max_dt);
        if (node_->has_parameter("swarm_collision/box_inflation"))
            cfg.box_inflation = node_->get_parameter("swarm_collision/box_inflation").as_double();
        else
            cfg.box_inflation = node_->declare_parameter<double>("swarm_collision/box_inflation", cfg.box_inflation);
        if (node_->has_parameter("swarm_collision/ahead_eps"))
            cfg.ahead_eps = node_->get_parameter("swarm_collision/ahead_eps").as_double();
        else
            cfg.ahead_eps = node_->declare_parameter<double>("swarm_collision/ahead_eps", cfg.ahead_eps);

        initialized = true;
    }

    const double base_safe_dist = 2.0 * cfg.robot_radius + cfg.safe_margin;
    const int    self_id        = swarm_traj_data_.drone_id_;
    auto&        other_traj     = swarm_traj_data_.swarm_trajs_[other_id - 1];

    // 3) 空间预筛：控制点包围盒 + 可选膨胀（B-spline 曲线可略超出控制点，膨胀减少漏检）
    Eigen::Vector2d s_min, s_max, o_min, o_max;
    local_data_.position_traj_.getBoundaryXYZ(s_min, s_max);
    other_traj.getBoundaryXYZ(o_min, o_max);
    if (cfg.box_inflation > 0.0)
    {
        s_min -= Eigen::Vector2d::Constant(cfg.box_inflation);
        s_max += Eigen::Vector2d::Constant(cfg.box_inflation);
        o_min -= Eigen::Vector2d::Constant(cfg.box_inflation);
        o_max += Eigen::Vector2d::Constant(cfg.box_inflation);
    }
    if ((s_min.x() - o_max.x() > base_safe_dist) || (o_min.x() - s_max.x() > base_safe_dist) || (s_min.y() - o_max.y() > base_safe_dist)
        || (o_min.y() - s_max.y() > base_safe_dist))
    {
        return true;
    }

    // 4) 优先级：以后面的让前面的为主（沿航向几何），难分胜负时用 ID 打破对称
    auto decideSelfYield = [&](const Eigen::Vector3d& sp, const Eigen::Vector3d& sv, const Eigen::Vector3d& op, const Eigen::Vector3d& ov) -> bool {
        const double    eps_spd = 0.1;
        const double    eps     = cfg.ahead_eps;
        Eigen::Vector2d sv2     = sv.head<2>();
        Eigen::Vector2d ov2     = ov.head<2>();
        double          sv_n    = sv2.norm();
        double          ov_n    = ov2.norm();

        // 对向会车优先采用稳定仲裁（ID 打破对称），避免因微小重规划/抖动导致先后权反复翻转。
        if (sv_n > eps_spd && ov_n > eps_spd)
        {
            const Eigen::Vector2d self_dir        = sv2 / sv_n;
            const Eigen::Vector2d other_dir       = ov2 / ov_n;
            const Eigen::Vector2d rel_so          = (op - sp).head<2>();
            const double          heading_dot     = self_dir.dot(other_dir);  // -1: 对向, +1: 同向
            const bool            face_each_other = (rel_so.dot(self_dir) > 0.0) && ((-rel_so).dot(other_dir) > 0.0);
            if (heading_dot < -0.3 && face_each_other)
                return (self_id > other_id);
        }

        // 本机有明显速度：他机在本机航向正前方 → 本机在后方 → 本机让行
        if (sv_n > eps_spd)
        {
            Eigen::Vector2d self_dir         = sv2 / sv_n;
            double          ahead_along_self = (op - sp).head<2>().dot(self_dir);
            if (ahead_along_self > eps)
            {
                // 对向会车时，双方都可能“看到对方在自己前方”而同时让行。
                // 先检测是否是明显对向，若是则直接用 ID 打破对称，只让一方停车。
                if (ov_n > eps_spd)
                {
                    Eigen::Vector2d other_dir         = ov2 / ov_n;
                    double          heading_dot       = self_dir.dot(other_dir);  // -1: 对向, +1: 同向
                    double          ahead_along_other = (sp - op).head<2>().dot(other_dir);
                    if (heading_dot < -0.3 && ahead_along_other > eps)
                        return (self_id > other_id);
                }
                return true;
            }
            if (ahead_along_self < -eps)
                return false;
        }

        // 侧向/模糊：他机有明显速度时，若本机在其航向正后方 → 本机让行
        if (ov_n > eps_spd)
        {
            Eigen::Vector2d other_dir               = ov2 / ov_n;
            double          self_behind_along_other = (sp - op).head<2>().dot(other_dir);
            if (self_behind_along_other < -eps)
                return true;
            if (self_behind_along_other > eps)
                return false;
        }

        // 对头或近乎静止：双方可能同时判让行（更安全）；仍模糊时小号优先通过
        return (self_id > other_id);
    };

    // 双方在同一采样点都“判让”时（互见对方在各自航向正前方），只让 ID 较大者停车，避免双停。
    auto resolveSelfYield = [&](const Eigen::Vector3d& sp, const Eigen::Vector3d& sv, const Eigen::Vector3d& op, const Eigen::Vector3d& ov,
                                bool raw_yield) -> bool {
        if (!raw_yield)
            return false;
        const double    eps_spd = 0.1;
        const double    eps     = cfg.ahead_eps;
        Eigen::Vector2d sv2     = sv.head<2>();
        Eigen::Vector2d ov2     = ov.head<2>();
        double          sv_n    = sv2.norm();
        double          ov_n    = ov2.norm();
        if (sv_n > eps_spd && ov_n > eps_spd)
        {
            Eigen::Vector2d self_dir              = sv2 / sv_n;
            Eigen::Vector2d other_dir             = ov2 / ov_n;
            double          ahead_other_from_self = (op - sp).head<2>().dot(self_dir);
            double          ahead_self_from_other = (sp - op).head<2>().dot(other_dir);
            if (ahead_other_from_self > eps && ahead_self_from_other > eps)
                return (self_id > other_id);
        }
        return raw_yield;
    };

    // 5) 时间窗口对齐
    double now     = node_->now().seconds();
    double t_start = now;
    double t_end   = std::min({ local_data_.start_time_.seconds() + local_data_.duration_, other_traj.start_time_ + other_traj.duration_, now + cfg.horizon });

    // 6) 偏差与导数准备
    double          odom_delay = std::min(std::max(0.0, (node_->now() - self_odom_stamp).seconds()), cfg.odom_delay_cap);
    Eigen::Vector2d bias =
        (self_odom_pos - local_data_.position_traj_.evaluateDeBoorT(std::max(0.0, self_odom_stamp.seconds() - local_data_.start_time_.seconds()))).head<2>();

    NonUniformBspline other_vel_trj = other_traj.getDerivative();
    NonUniformBspline self_acc_trj  = local_data_.velocity_traj_.getDerivative();
    NonUniformBspline other_acc_trj = other_vel_trj.getDerivative();

    // 低相对速度时扩展预测时间窗，减少“长轨迹交错但早期看不到交汇”的漏判。
    const bool   adaptive_horizon_enable           = node_->has_parameter("swarm_collision/adaptive_horizon_enable")
                                                         ? node_->get_parameter("swarm_collision/adaptive_horizon_enable").as_bool()
                                                         : node_->declare_parameter<bool>("swarm_collision/adaptive_horizon_enable", true);
    const double adaptive_horizon_max              = node_->has_parameter("swarm_collision/adaptive_horizon_max")
                                                         ? node_->get_parameter("swarm_collision/adaptive_horizon_max").as_double()
                                                         : node_->declare_parameter<double>("swarm_collision/adaptive_horizon_max", 6.0);
    const double adaptive_horizon_rel_speed_thresh = node_->has_parameter("swarm_collision/adaptive_horizon_rel_speed_thresh")
                                                         ? node_->get_parameter("swarm_collision/adaptive_horizon_rel_speed_thresh").as_double()
                                                         : node_->declare_parameter<double>("swarm_collision/adaptive_horizon_rel_speed_thresh", 0.6);
    const double adaptive_horizon_gain             = node_->has_parameter("swarm_collision/adaptive_horizon_gain")
                                                         ? node_->get_parameter("swarm_collision/adaptive_horizon_gain").as_double()
                                                         : node_->declare_parameter<double>("swarm_collision/adaptive_horizon_gain", 2.0);

    double horizon_eff = cfg.horizon;
    if (adaptive_horizon_enable)
    {
        double          ts0             = std::max(0.0, now - local_data_.start_time_.seconds());
        double          to0             = std::max(0.0, now - other_traj.start_time_);
        Eigen::Vector3d sv0             = local_data_.velocity_traj_.evaluateDeBoorT(ts0);
        Eigen::Vector3d ov0             = other_vel_trj.evaluateDeBoorT(to0);
        double          rel_speed       = (sv0 - ov0).head<2>().norm();
        double          low_speed_ratio = std::max(0.0, adaptive_horizon_rel_speed_thresh - rel_speed) / std::max(1e-3, adaptive_horizon_rel_speed_thresh);
        horizon_eff                     = std::min(adaptive_horizon_max, cfg.horizon + adaptive_horizon_gain * low_speed_ratio);
    }

    t_end = std::min({ local_data_.start_time_.seconds() + local_data_.duration_, other_traj.start_time_ + other_traj.duration_, now + horizon_eff });
    if (t_end <= t_start)
    {
        // 时间窗口为空（本车/对方轨迹已执行完或过期）：不能直接放行——两车静止贴近时
        // 轨迹过期放行会让动态检测完全失效。退化到双方当前轨迹位置的单点判定。
        double          ts_now = std::max(0.0, now - local_data_.start_time_.seconds());
        double          to_now = std::max(0.0, now - other_traj.start_time_);
        Eigen::Vector3d sp_now = local_data_.position_traj_.evaluateDeBoorT(ts_now);
        sp_now.head<2>() += cfg.bias_gain * bias;
        Eigen::Vector3d op_now   = other_traj.evaluateDeBoorT(to_now);
        Eigen::Vector3d sv_now   = local_data_.velocity_traj_.evaluateDeBoorT(ts_now);
        Eigen::Vector3d ov_now   = other_vel_trj.evaluateDeBoorT(to_now);
        double          d_now    = (sp_now - op_now).head<2>().norm();
        double          safe_now = base_safe_dist + cfg.delay_gain * (sv_now.norm() + ov_now.norm()) * odom_delay + bias.norm();
        if (d_now < safe_now)
        {
            self_yield = resolveSelfYield(sp_now, sv_now, op_now, ov_now, decideSelfYield(sp_now, sv_now, op_now, ov_now));
            return false;
        }
        return true;
    }

    // 7) 采样检测循环（步长随相对速度自适应：接近时采样更密）
    const int max_samples = 500;
    int       n_samples   = 0;
    for (double t = t_start; t < t_end && n_samples < max_samples; ++n_samples)
    {
        double ts = std::max(0.0, t - local_data_.start_time_.seconds());
        double to = std::max(0.0, t - other_traj.start_time_);

        Eigen::Vector3d sp = local_data_.position_traj_.evaluateDeBoorT(ts);
        sp.head<2>() += cfg.bias_gain * bias;
        Eigen::Vector3d sv = local_data_.velocity_traj_.evaluateDeBoorT(ts);
        Eigen::Vector3d sa = self_acc_trj.evaluateDeBoorT(ts);

        Eigen::Vector3d op = other_traj.evaluateDeBoorT(to);
        Eigen::Vector3d ov = other_vel_trj.evaluateDeBoorT(to);
        Eigen::Vector3d oa = other_acc_trj.evaluateDeBoorT(to);

        Eigen::Vector2d P = (sp - op).head<2>();
        Eigen::Vector2d V = (sv - ov).head<2>();
        Eigen::Vector2d A = (sa - oa).head<2>();

        double dist           = P.norm();
        double dynamic_safe_d = base_safe_dist + cfg.delay_gain * (sv.norm() + ov.norm()) * odom_delay + bias.norm();

        // 数值保护：距离过小视为碰撞，避免 TTC 中除零
        if (dist < 1e-6)
        {
            self_yield = resolveSelfYield(sp, sv, op, ov, decideSelfYield(sp, sv, op, ov));
            return false;
        }
        // 7a) 碰撞判定
        if (dist < dynamic_safe_d)
        {
            self_yield = resolveSelfYield(sp, sv, op, ov, decideSelfYield(sp, sv, op, ov));
            return false;
        }

        // 7b) TTC 判定 (考虑加速度)
        double vr = P.dot(V) / dist;  // 径向速度，负值表示靠近
        if (vr < -0.1)
        {
            // 加速度修正：dist + vr*t + 0.5*ar*t^2 = safe_d
            double ar    = (V.squaredNorm() + P.dot(A) - vr * vr) / dist;
            double ttc   = -1.0;
            double delta = vr * vr - 2.0 * ar * (dist - dynamic_safe_d);
            if (std::fabs(ar) < 1e-4)
            {
                // 等速/近等速退化保护：ar≈0 时二次公式分母趋于 0，旧写法 ttc→0
                // 预警完全失效（两车匀速对向恰是最常见场景），必须改用线性公式。
                ttc = (dynamic_safe_d - dist) / vr;  // vr < 0 → ttc > 0
            }
            else if (delta >= 0)
            {
                ttc = (-vr - std::sqrt(delta)) / ar;
                if (ttc < 0)
                    ttc = (-vr + std::sqrt(delta)) / ar;
            }
            else
            {
                ttc = (dynamic_safe_d - dist) / vr;  // 二次无实根，保守退化为线性
            }

            if (ttc > 0 && ttc < cfg.ttc_threshold)
            {
                self_yield = resolveSelfYield(sp, sv, op, ov, decideSelfYield(sp, sv, op, ov));
                return false;
            }
        }

        // 动态步长逻辑
        double step = 0.4 * dynamic_safe_d / (V.norm() + 0.1);
        t += std::min(std::max(cfg.min_dt, step), cfg.max_dt);
    }

    return true;
}

bool FastPlannerManager::checkSwarmCollisionHybrid(const int& other_id, const Eigen::Vector3d& self_odom_pos, const Eigen::Vector3d& self_odom_vel,
                                                   const rclcpp::Time& self_odom_stamp, const OtherAgentSnapshot& other, bool& self_yield)
{
    self_yield = false;

    if (other_id <= 0 || other_id > static_cast<int>(swarm_traj_data_.receive_flags_.size()))
        return true;
    if (other_id == swarm_traj_data_.drone_id_)
        return true;

    const bool has_other_traj = swarm_traj_data_.receive_flags_[other_id - 1];
    if (!other.is_static && has_other_traj)
        return checkSwarmCollision(other_id, self_odom_pos, self_odom_vel, self_odom_stamp, self_yield);

    struct StaticCfg
    {
        double safe_margin{ 0.12 };
        double horizon{ 2.0 };
        double min_dt{ 0.02 };
        double max_dt{ 0.10 };
        double odom_delay_cap{ 0.25 };
        double bias_gain{ 0.8 };
        double ahead_eps{ 0.25 };
        double delay_gain{ 1.6 };  // 速度-延迟安全距离增益，与动态分支 delay_gain 同义
    };
    static StaticCfg cfg;
    static bool      cfg_init = false;
    if (!cfg_init && node_)
    {
        cfg_init = true;
        if (node_->has_parameter("swarm_collision/static_safe_margin"))
            cfg.safe_margin = node_->get_parameter("swarm_collision/static_safe_margin").as_double();
        else
            cfg.safe_margin = node_->declare_parameter<double>("swarm_collision/static_safe_margin", cfg.safe_margin);
        if (node_->has_parameter("swarm_collision/static_horizon"))
            cfg.horizon = node_->get_parameter("swarm_collision/static_horizon").as_double();
        else
            cfg.horizon = node_->declare_parameter<double>("swarm_collision/static_horizon", cfg.horizon);
        if (node_->has_parameter("swarm_collision/static_min_dt"))
            cfg.min_dt = node_->get_parameter("swarm_collision/static_min_dt").as_double();
        else
            cfg.min_dt = node_->declare_parameter<double>("swarm_collision/static_min_dt", cfg.min_dt);
        if (node_->has_parameter("swarm_collision/static_max_dt"))
            cfg.max_dt = node_->get_parameter("swarm_collision/static_max_dt").as_double();
        else
            cfg.max_dt = node_->declare_parameter<double>("swarm_collision/static_max_dt", cfg.max_dt);
        if (node_->has_parameter("swarm_collision/static_odom_delay_cap"))
            cfg.odom_delay_cap = node_->get_parameter("swarm_collision/static_odom_delay_cap").as_double();
        else
            cfg.odom_delay_cap = node_->declare_parameter<double>("swarm_collision/static_odom_delay_cap", cfg.odom_delay_cap);
        if (node_->has_parameter("swarm_collision/static_bias_gain"))
            cfg.bias_gain = node_->get_parameter("swarm_collision/static_bias_gain").as_double();
        else
            cfg.bias_gain = node_->declare_parameter<double>("swarm_collision/static_bias_gain", cfg.bias_gain);
        if (node_->has_parameter("swarm_collision/static_ahead_eps"))
            cfg.ahead_eps = node_->get_parameter("swarm_collision/static_ahead_eps").as_double();
        else
            cfg.ahead_eps = node_->declare_parameter<double>("swarm_collision/static_ahead_eps", cfg.ahead_eps);

        // 速度-延迟安全距离增益：优先使用 static_delay_gain，未配置则复用动态分支 delay_gain
        if (node_->has_parameter("swarm_collision/static_delay_gain"))
            cfg.delay_gain = node_->get_parameter("swarm_collision/static_delay_gain").as_double();
        else if (node_->has_parameter("swarm_collision/delay_gain"))
            cfg.delay_gain = node_->get_parameter("swarm_collision/delay_gain").as_double();
        else
            cfg.delay_gain = node_->declare_parameter<double>("swarm_collision/static_delay_gain", cfg.delay_gain);
    }

    const int self_id         = swarm_traj_data_.drone_id_;
    auto      decideSelfYield = [&](const Eigen::Vector3d& sp, const Eigen::Vector3d& sv, const Eigen::Vector3d& op, const Eigen::Vector3d& ov) -> bool {
        const double    eps_spd = 0.1;
        const double    eps     = cfg.ahead_eps;
        Eigen::Vector2d sv2     = sv.head<2>();
        Eigen::Vector2d ov2     = ov.head<2>();
        double          sv_n    = sv2.norm();
        double          ov_n    = ov2.norm();

        if (sv_n > eps_spd && ov_n > eps_spd)
        {
            const Eigen::Vector2d self_dir        = sv2 / sv_n;
            const Eigen::Vector2d other_dir       = ov2 / ov_n;
            const Eigen::Vector2d rel_so          = (op - sp).head<2>();
            const double          heading_dot     = self_dir.dot(other_dir);
            const bool            face_each_other = (rel_so.dot(self_dir) > 0.0) && ((-rel_so).dot(other_dir) > 0.0);
            if (heading_dot < -0.3 && face_each_other)
                return (self_id > other_id);
        }

        if (sv_n > eps_spd)
        {
            Eigen::Vector2d self_dir         = sv2 / sv_n;
            double          ahead_along_self = (op - sp).head<2>().dot(self_dir);
            if (ahead_along_self > eps)
                return true;
            if (ahead_along_self < -eps)
                return false;
        }
        if (ov_n > eps_spd)
        {
            Eigen::Vector2d other_dir               = ov2 / ov_n;
            double          self_behind_along_other = (sp - op).head<2>().dot(other_dir);
            if (self_behind_along_other < -eps)
                return true;
            if (self_behind_along_other > eps)
                return false;
        }
        return (self_id > other_id);
    };

    if (!node_)
        return true;

    const double now     = node_->now().seconds();
    const double t_start = now;
    const double t_end   = std::min(local_data_.start_time_.seconds() + local_data_.duration_, now + cfg.horizon);
    if (t_end <= t_start + 1e-6)
        return true;

    Eigen::Vector2d bias =
        (self_odom_pos - local_data_.position_traj_.evaluateDeBoorT(std::max(0.0, self_odom_stamp.seconds() - local_data_.start_time_.seconds()))).head<2>();
    const double odom_delay = std::min(std::max(0.0, (node_->now() - self_odom_stamp).seconds()), cfg.odom_delay_cap);

    const double L           = std::max(1e-3, pp_.vehicle_length_);
    const double W           = std::max(1e-3, pp_.vehicle_width_);
    const double base_margin = 2.0 * cfg.safe_margin;

    const Eigen::Vector2d oc = other.pos.head<2>();
    const double          oy = other.yaw;

    const int max_samples = 500;
    int       n_samples   = 0;
    for (double t = t_start; t < t_end && n_samples < max_samples; ++n_samples)
    {
        const double    ts  = std::max(0.0, t - local_data_.start_time_.seconds());
        Eigen::Vector3d sp3 = local_data_.position_traj_.evaluateDeBoorT(std::min(ts, local_data_.duration_));
        sp3.head<2>() += cfg.bias_gain * bias;
        Eigen::Vector3d sv3 = local_data_.velocity_traj_.evaluateDeBoorT(std::min(ts, local_data_.duration_));

        const double sy = TrajHeadingXY(local_data_.position_traj_, local_data_.velocity_traj_, std::min(ts, local_data_.duration_), local_data_.duration_);

        // 速度相关安全距离：补偿延迟、控制响应及轨迹-里程计偏差
        const double spd          = sv3.head<2>().norm();
        const double speed_margin = cfg.delay_gain * spd * odom_delay + bias.norm();
        const double Linf         = std::max(0.0, L + base_margin + speed_margin);
        const double Winf         = std::max(0.0, W + base_margin + speed_margin);

        if (ObbOverlap2D(sp3.head<2>(), sy, Linf, Winf, oc, oy, Linf, Winf))
        {
            const Eigen::Vector3d ov3 = Eigen::Vector3d::Zero();
            self_yield                = decideSelfYield(sp3, sv3, other.pos, ov3);
            return false;
        }

        const double step = 0.35 * std::hypot(Linf, Winf) / (std::max(0.05, spd) + 0.1);
        t += std::min(std::max(cfg.min_dt, step), cfg.max_dt);
    }

    return true;
}
}  // namespace fast_planner
