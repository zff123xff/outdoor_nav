#include <path_searching/kinodynamic_astar.h>
#include <sstream>
#include <plan_env/sdf_map.h>
#include <cmath> // For std::fmod, std::max, std::min

using namespace std;
using namespace Eigen;

namespace fast_planner
{
// 尽管状态和输入在内部被视为 4D/2D，但我们保留了原始的 6D/3D 接口定义。

KinodynamicAstar::~KinodynamicAstar()
{
    for (int i = 0; i < allocate_num_; i++)
    {
        delete path_node_pool_[i];
    }
}

// ----------------------------------------------------------------------
// 接口 1: search - 严格保持原始签名
// ----------------------------------------------------------------------
int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a, Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{
    // 内部状态表示：
    // state[0]: x, state[1]: y, state[2]: theta (使用原始的 pz 占位), state[3]: v (使用原始的 vx 占位)
    // state[4:5]: 始终为 0 (vh, vz)

    // 内部输入表示：
    // input[0]: a (线加速度), input[1]: 0 (ax 占位), input[2]: omega (角速度)

    // 状态填充 (6D)
    Eigen::Matrix<double, 6, 1> start_state, end_state;

    // 映射: pz -> theta, vx -> v
    start_state << start_pt(0), start_pt(1), start_pt(2), start_v(0), 0.0, 0.0;
    end_state << end_pt(0), end_pt(1), end_pt(2), end_v(0), 0.0, 0.0;

    // 初始控制输入映射: ax -> a, az -> omega
    Eigen::Vector3d initial_um;
    initial_um << start_a(0), 0.0, start_a(2);

    PathNodePtr cur_node = path_node_pool_[0];
    cur_node->parent = NULL;
    cur_node->state = start_state;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_score = 0.0;

    Eigen::Vector3i end_index = posToIndex(end_pt);
    double time_to_goal;

    cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
    cur_node->node_state = IN_OPEN_SET;
    open_set_.push(cur_node);
    use_node_num_ += 1;

    if (dynamic)
    {
        time_origin_ = time_start;
        cur_node->time = time_start;
        cur_node->time_idx = timeToIndex(time_start);
        expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    }
    else
        expanded_nodes_.insert(cur_node->index, cur_node);

    PathNodePtr terminate_node = NULL;
    bool init_search = init;
    const int tolerance = ceil(1 / resolution_);

    while (!open_set_.empty())
    {
        cur_node = open_set_.top();

        // Terminate?
        // 仅检查 XY 平面距离
        bool reach_horizon = (cur_node->state.head(2) - start_state.head(2)).norm() >= horizon_;
        // 仅检查 XY 索引
        bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                        abs(cur_node->index(1) - end_index(1)) <= tolerance;

        if (reach_horizon || near_end)
        {
            terminate_node = cur_node;
            retrievePath(terminate_node);
            if (near_end)
            {
                estimateHeuristic(cur_node->state, end_state, time_to_goal);
                // 禁用或使用简化的 shot traj，因为 5阶多项式不适用非全向模型
                computeShotTraj(cur_node->state, end_state, time_to_goal);
                if (init_search) RCLCPP_ERROR(rclcpp::get_logger("kinodynamic"), "Shot in first search loop!");
            }
        }
        if (reach_horizon)
        {
            if (is_shot_succ_)
                return REACH_END;
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("kinodynamic"), "reach horizon");
                return REACH_HORIZON;
            }
        }
        if (near_end)
        {
            if (is_shot_succ_)
                return REACH_END;
            else if (cur_node->parent != NULL)
            {
                RCLCPP_INFO(rclcpp::get_logger("kinodynamic"), "near end");
                return NEAR_END;
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("kinodynamic"), "no path");
                return NO_PATH;
            }
        }
        open_set_.pop();
        cur_node->node_state = IN_CLOSE_SET;
        iter_num_ += 1;

        double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;

        // 保持 6D 状态类型
        Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
        Eigen::Matrix<double, 6, 1> pro_state;
        vector<PathNodePtr> tmp_expand_nodes;

        // 使用 Vector3d 作为接口输入类型
        Eigen::Vector3d um;
        double pro_t;
        vector<Eigen::Vector3d> inputs;
        vector<double> durations;

        double max_acc_car = max_acc_;
        double max_omega_car = max_acc_car; // 假设 max_omega = max_acc

        if (init_search)
        {
            inputs.push_back(initial_um);
            for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
                 tau += time_res_init * init_max_tau_)
                durations.push_back(tau);
            init_search = false;
        }
        else
        {
            // 搜索: 遍历线加速度 a 和角速度 omega，填充到 um(0) 和 um(2)
            double res_omega = 1.0 / 2.0;
            for (double a = -max_acc_car; a <= max_acc_car + 1e-3; a += max_acc_car * res)
                for (double omega = -max_omega_car; omega <= max_omega_car + 1e-3; omega += max_omega_car * res_omega)
                {
                    // um: [a, 0.0, omega]^T
                    um << a, 0.0, omega;
                    inputs.push_back(um);
                }
            for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
                durations.push_back(tau);
        }

        for (int i = 0; i < inputs.size(); ++i)
            for (int j = 0; j < durations.size(); ++j)
            {
                um = inputs[i];
                double tau = durations[j];

                // 接口调用：使用原始的 6D 签名
                stateTransit(cur_state, pro_state, um, tau);
                pro_t = cur_node->time + tau;

                // Check inside map range (仅使用 XY 坐标，Z 设为 0)
                Eigen::Vector3d pro_pos_3d;
                pro_pos_3d << pro_state(0), pro_state(1), 0.0;
                if (!edt_environment_->sdf_map_->isInBox(pro_pos_3d))
                {
                    continue;
                }

                // Check if in close set (仅使用 XY 索引)
                Eigen::Vector3i pro_id = posToIndex(pro_pos_3d);
                int pro_t_id = timeToIndex(pro_t);
                PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
                if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
                {
                    continue;
                }

                // Check maximal velocity (仅检查线速度 v = pro_state(3))
                double pro_v = pro_state(3);
                if (fabs(pro_v) > max_vel_)
                {
                    continue;
                }

                // Check not in the same voxel (仅使用 XY 索引)
                Eigen::Vector3i diff = pro_id - cur_node->index;
                int diff_time = pro_t_id - cur_node->time_idx;
                if (diff.head(2).norm() == 0 && ((!dynamic) || diff_time == 0))
                {
                    continue;
                }

                // Check safety (碰撞检查也只使用 XY 坐标)
                Eigen::Vector3d pos_3d;
                Eigen::Matrix<double, 6, 1> xt; // 保持 6D
                bool is_occ = false;
                for (int k = 1; k <= check_num_; ++k)
                {
                    double dt = tau * double(k) / double(check_num_);
                    stateTransit(cur_state, xt, um, dt);
                    pos_3d << xt(0), xt(1), 0.0;

                    if (edt_environment_->sdf_map_->getInflateOccupancy(pos_3d) == 1)
                    {
                        is_occ = true;
                        break;
                    }
                    if (!optimistic_ && edt_environment_->sdf_map_->getOccupancy(pos_3d) == SDFMap::UNKNOWN)
                    {
                        is_occ = true;
                        break;
                    }
                }
                if (is_occ)
                {
                    continue;
                }

                double time_to_goal, tmp_g_score, tmp_f_score;
                // g_score: 使用控制输入(a, omega)的平方和 (um(0)^2 + um(2)^2) + 时间项
                tmp_g_score = (um(0) * um(0) + um(2) * um(2) + w_time_) * tau + cur_node->g_score;
                tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

                // Compare nodes expanded from the same parent
                bool prune = false;
                for (size_t j = 0; j < tmp_expand_nodes.size(); ++j)
                {
                    PathNodePtr expand_node = tmp_expand_nodes[j];
                    if ((pro_id - expand_node->index).norm() == 0 &&
                        ((!dynamic) || pro_t_id == expand_node->time_idx))
                    {
                        prune = true;
                        if (tmp_f_score < expand_node->f_score)
                        {
                            expand_node->f_score = tmp_f_score;
                            expand_node->g_score = tmp_g_score;
                            expand_node->state = pro_state;
                            expand_node->input = um; // 保持 3D 接口
                            expand_node->duration = tau;
                            if (dynamic) expand_node->time = cur_node->time + tau;
                        }
                        break;
                    }
                }

                if (!prune)
                {
                    if (pro_node == NULL)
                    {
                        pro_node = path_node_pool_[use_node_num_];
                        pro_node->index = pro_id;
                        pro_node->state = pro_state;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->input = um; // 保持 3D 接口
                        pro_node->duration = tau;
                        pro_node->parent = cur_node;
                        pro_node->node_state = IN_OPEN_SET;
                        if (dynamic)
                        {
                            pro_node->time = cur_node->time + tau;
                            pro_node->time_idx = timeToIndex(pro_node->time);
                        }
                        open_set_.push(pro_node);

                        if (dynamic)
                            expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
                        else
                            expanded_nodes_.insert(pro_id, pro_node);

                        tmp_expand_nodes.push_back(pro_node);

                        use_node_num_ += 1;
                        if (use_node_num_ == allocate_num_)
                        {
                            RCLCPP_INFO_STREAM(rclcpp::get_logger("kinodynamic"), "run out of memory.");;
                            return NO_PATH;
                        }
                    }
                    else if (pro_node->node_state == IN_OPEN_SET)
                    {
                        if (tmp_g_score < pro_node->g_score)
                        {
                            pro_node->state = pro_state;
                            pro_node->f_score = tmp_f_score;
                            pro_node->g_score = tmp_g_score;
                            pro_node->input = um;
                            pro_node->duration = tau;
                            pro_node->parent = cur_node;
                            if (dynamic) pro_node->time = cur_node->time + tau;
                        }
                    }
                    else
                    {
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("kinodynamic"), "error type in searching: " << pro_node->node_state);;
                    }
                }
            }
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("kinodynamic"), "open set empty, no path!");;
    return NO_PATH;
}

void KinodynamicAstar::setParam(rclcpp::Node::SharedPtr node)
{
    // ... (保持不变) ...
    max_tau_ = node->has_parameter("search/max_tau") ? 
        node->get_parameter("search/max_tau").as_double() : 
        node->declare_parameter<double>("search/max_tau", -1.0);
    init_max_tau_ = node->has_parameter("search/init_max_tau") ? 
        node->get_parameter("search/init_max_tau").as_double() : 
        node->declare_parameter<double>("search/init_max_tau", -1.0);
    max_vel_ = node->has_parameter("search/max_vel") ? 
        node->get_parameter("search/max_vel").as_double() : 
        node->declare_parameter<double>("search/max_vel", -1.0);
    max_acc_ = node->has_parameter("search/max_acc") ? 
        node->get_parameter("search/max_acc").as_double() : 
        node->declare_parameter<double>("search/max_acc", -1.0);
    w_time_ = node->has_parameter("search/w_time") ? 
        node->get_parameter("search/w_time").as_double() : 
        node->declare_parameter<double>("search/w_time", -1.0);
    horizon_ = node->has_parameter("search/horizon") ? 
        node->get_parameter("search/horizon").as_double() : 
        node->declare_parameter<double>("search/horizon", -1.0);
    resolution_ = node->has_parameter("search/resolution_astar") ? 
        node->get_parameter("search/resolution_astar").as_double() : 
        node->declare_parameter<double>("search/resolution_astar", -1.0);
    time_resolution_ = node->has_parameter("search/time_resolution") ? 
        node->get_parameter("search/time_resolution").as_double() : 
        node->declare_parameter<double>("search/time_resolution", -1.0);
    lambda_heu_ = node->has_parameter("search/lambda_heu") ? 
        node->get_parameter("search/lambda_heu").as_double() : 
        node->declare_parameter<double>("search/lambda_heu", -1.0);
    allocate_num_ = node->has_parameter("search/allocate_num") ? 
        node->get_parameter("search/allocate_num").as_int() : 
        node->declare_parameter<int>("search/allocate_num", -1);
    check_num_ = node->has_parameter("search/check_num") ? 
        node->get_parameter("search/check_num").as_int() : 
        node->declare_parameter<int>("search/check_num", -1);
    optimistic_ = node->has_parameter("search/optimistic") ? 
        node->get_parameter("search/optimistic").as_bool() : 
        node->declare_parameter<bool>("search/optimistic", true);
    tie_breaker_ = 1.0 + 1.0 / 10000;

    double vel_margin = node->has_parameter("search/vel_margin") ? 
        node->get_parameter("search/vel_margin").as_double() : 
        node->declare_parameter<double>("search/vel_margin", 0.0);
    max_vel_ += vel_margin;
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
    PathNodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
        cur_node = cur_node->parent;
        path_nodes_.push_back(cur_node);
    }

    reverse(path_nodes_.begin(), path_nodes_.end());
}

// ----------------------------------------------------------------------
// 接口 2: estimateHeuristic - 严格保持原始签名 (VectorXd 实际上是 6D)
// ----------------------------------------------------------------------
double KinodynamicAstar::estimateHeuristic(
    Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time)
{
    // 映射: x1.head(3) -> [x, y, theta], x1.segment(3, 3) -> [v, 0, 0]
    const Vector2d dp = x2.head(2) - x1.head(2); // 2D 位置差
    const double v0 = x1(3);                     // 初始线速度
    const double v1 = x2(3);                     // 终止线速度
    const double acc_max = max_acc_;

    double t_min_accel = 0.0;
    double dist = dp.norm();

    // 简化的 Minimum Time Heuristic (忽略非全向约束，只关注 XY 距离和线速度)
    if (dist > 1e-3)
    {
        // 假设匀加速到一半距离，匀减速到另一半距离（粗略估计）
        t_min_accel = 2.0 * std::sqrt(dist / acc_max);
    }
    else
    {
        t_min_accel = std::abs(v1 - v0) / acc_max;
    }

    optimal_time = t_min_accel;
    double cost = w_time_ * optimal_time + dp.norm();

    return 1.0 * (1 + tie_breaker_) * cost;
}

// ----------------------------------------------------------------------
// 接口 3: computeShotTraj - 严格保持原始签名 (VectorXd 实际上是 6D)
// ----------------------------------------------------------------------
bool KinodynamicAstar::computeShotTraj(
    Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
    // 保持禁用状态，因为它不适用非全向车辆
    is_shot_succ_ = false;
    return false;
}

// cubic 和 quartic 保持不变
vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
    // ... (保持不变) ...
    vector<double> dts;

    double a2 = b / a;
    double a1 = c / a;
    double a0 = d / a;

    double Q = (3 * a1 - a2 * a2) / 9;
    double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
    double D = Q * Q * Q + R * R;
    if (D > 0)
    {
        double S = std::cbrt(R + sqrt(D));
        double T = std::cbrt(R - sqrt(D));
        dts.push_back(-a2 / 3 + (S + T));
        return dts;
    }
    else if (D == 0)
    {
        double S = std::cbrt(R);
        dts.push_back(-a2 / 3 + S + S);
        dts.push_back(-a2 / 3 - S);
        return dts;
    }
    else
    {
        double theta = acos(R / sqrt(-Q * Q * Q));
        dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
        return dts;
    }
}

vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
    // ... (保持不变) ...
    vector<double> dts;

    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;

    vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
    if (ys.empty()) return dts;
    double y1 = ys.front();
    double r = a3 * a3 / 4 - a2 + y1;
    if (r < 0) return dts;

    double R = sqrt(r);
    double D, E;
    if (R != 0)
    {
        D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
        E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    }
    else
    {
        if (y1 * y1 - 4 * a0 < 0) return dts;
        D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
        E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
    }

    if (!std::isnan(D))
    {
        dts.push_back(-a3 / 4 + R / 2 + D / 2);
        dts.push_back(-a3 / 4 + R / 2 - D / 2);
    }
    if (!std::isnan(E))
    {
        dts.push_back(-a3 / 4 - R / 2 + E / 2);
        dts.push_back(-a3 / 4 - R / 2 - E / 2);
    }

    return dts;
}

void KinodynamicAstar::init()
{
    /* ---------- map params ---------- */
    this->inv_resolution_ = 1.0 / resolution_;
    inv_time_resolution_ = 1.0 / time_resolution_;
    edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("kinodynamic"), "origin_: " << origin_.transpose());;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("kinodynamic"), "map size: " << map_size_3d_.transpose());;

    /* ---------- pre-allocated node ---------- */
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
        path_node_pool_[i] = new PathNode;
        // PathNode::state/input 应该保持 6D/3D 默认大小，因为它们是 VectorXd/Vector3d
        // 如果 PathNode::state 是固定大小的 Matrix<6,1>，则无需 resize
    }

    // 转移矩阵 phi_ 保持 6D，但只使用其 4x4 子矩阵
    phi_ = Eigen::MatrixXd::Identity(6, 6);
    use_node_num_ = 0;
    iter_num_ = 0;
}

void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr &env)
{
    this->edt_environment_ = env;
}

void KinodynamicAstar::reset()
{
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
        PathNodePtr node = path_node_pool_[i];
        node->parent = NULL;
        node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
    has_path_ = false;
}

// ----------------------------------------------------------------------
// 接口 4: getKinoTraj - 保持原始签名 (输出 3D 点)
// ----------------------------------------------------------------------
std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
    vector<Vector3d> state_list;

    /* ---------- get traj of searching ---------- */
    PathNodePtr node = path_nodes_.back();
    // 保持 6D 状态
    Eigen::Matrix<double, 6, 1> x0, xt;
    Eigen::Vector3d ut;

    while (node->parent != NULL)
    {
        ut = node->input;
        double duration = node->duration;
        x0 = node->parent->state;

        for (double t = duration; t >= -1e-5; t -= delta_t)
        {
            stateTransit(x0, xt, ut, t);
            // 仅使用 XY 坐标，Z 设为 0
            state_list.push_back(Vector3d(xt(0), xt(1), 0.0));
        }
        node = node->parent;
    }
    reverse(state_list.begin(), state_list.end());

    /* ---------- get traj of one shot (禁用) ---------- */
    // if (is_shot_succ_) { ... }

    return state_list;
}

// ----------------------------------------------------------------------
// 接口 5: getSamples - 保持原始签名 (Vector3d 列表)
// ----------------------------------------------------------------------
void KinodynamicAstar::getSamples(double &ts, vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivatives)
{
    double T_sum = 0.0;
    PathNodePtr temp_node = path_nodes_.back();
    while (temp_node->parent != NULL)
    {
        T_sum += temp_node->duration;
        temp_node = temp_node->parent;
    }

    // 终点导数：v, theta, a
    double end_v = path_nodes_.back()->state(3);
    double end_theta = path_nodes_.back()->state(2);
    double end_accel = path_nodes_.back()->input(0); // 线加速度

    int seg_num = floor(T_sum / ts);
    seg_num = max(8, seg_num);
    ts = T_sum / double(seg_num);

    PathNodePtr node = path_nodes_.back();
    double t = T_sum;

    for (double ti = T_sum; ti > -1e-5; ti -= ts)
    {
        Eigen::Matrix<double, 6, 1> x0, xt;
        Eigen::Vector3d ut;

        if (node->parent != NULL)
        {
            x0 = node->parent->state;
            ut = node->input;
        }
        else
        {
            x0 = node->state;
            xt = x0;
            ut = Eigen::Vector3d::Zero();
        }

        stateTransit(x0, xt, ut, t);

        point_set.push_back(Vector3d(xt(0), xt(1), 0.0)); // 仅使用 XY，Z=0
        t -= ts;

        if (t < -1e-5 && node->parent != NULL && node->parent->parent != NULL)
        {
            node = node->parent;
            t += node->duration;
        }
        else if (t < -1e-5 && node->parent == NULL)
        {
            break;
        }
    }
    reverse(point_set.begin(), point_set.end());

    // 收集导数 (start_v/end_v, start_theta/end_theta, start_a/end_a)
    double start_v = path_nodes_.front()->state(3);
    double start_theta = path_nodes_.front()->state(2);
    double start_accel = path_nodes_.front()->input(0);

    // 约定：[v, theta, accel]
    start_end_derivatives.push_back(Vector3d(start_v, start_theta, start_accel));
    start_end_derivatives.push_back(Vector3d(end_v, end_theta, end_accel));

    // 保持原接口的 4 个 Vector3d
    start_end_derivatives.push_back(Vector3d::Zero());
    start_end_derivatives.push_back(Vector3d::Zero());
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
    // 仅使用 XY 坐标进行索引 (Z 坐标始终为 0)
    Vector3d pt_2d;
    pt_2d << pt(0), pt(1), 0.0;

    Vector3i idx = ((pt_2d - origin_) * inv_resolution_).array().floor().cast<int>();
    idx(2) = 0;

    return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
    int idx = floor((time - time_origin_) * inv_time_resolution_);
    return idx;
}

// ----------------------------------------------------------------------
// 接口 6: stateTransit - 严格保持原始签名 (6D 状态, 3D 输入)
// ----------------------------------------------------------------------
void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1, Eigen::Vector3d um, double tau)
{
    // 差速轮 (Differential Drive) 或 非完整性 (Unicycle) 模型
    // 状态映射：[x, y, theta, v, 0, 0]^T
    // 输入映射：[a, 0, omega]^T

    // --- 提取状态和输入 ---
    double x = state0(0);
    double y = state0(1);
    double theta = state0(2);
    double v = state0(3);

    // 从 um (Vector3d) 中提取 2D 控制输入
    // um(0) 被解释为 线加速度 'a' (控制 v 的变化率)
    // um(2) 被解释为 角速度 'omega' (控制 theta 的变化率)
    double a = um(0);
    double omega = um(2);

    // --- 积分参数 ---
    double dt_step = 0.005; // 积分步长
    int steps = ceil(tau / dt_step);
    double dt = tau / (double)steps;

    // --- 欧拉积分 (4D 状态转移) ---
    for (int i = 0; i < steps; ++i)
    {
        // 运动学方程 (差速轮/非完整性):
        // dx/dt = v * cos(theta)
        // dy/dt = v * sin(theta)
        // dtheta/dt = omega
        // dv/dt = a (假设恒定的线加速度)

        double d_x = v * cos(theta);
        double d_y = v * sin(theta);
        double d_theta = omega;
        double d_v = a;

        x += d_x * dt;
        y += d_y * dt;
        theta += d_theta * dt;
        v += d_v * dt;

        // 限制航向角在 (-pi, pi]
        theta = std::fmod(theta + M_PI, 2.0 * M_PI);
        if (theta < 0) theta += 2.0 * M_PI;
        theta -= M_PI;

        // 限制速度在 [0, max_vel_] (假设只允许前进，v >= 0)
        v = std::max(0.0, std::min(v, max_vel_));
    }

    // --- 结果填充回 6D 状态 ---
    state1 << x, y, theta, v, 0.0, 0.0;
}

} // namespace fast_planner