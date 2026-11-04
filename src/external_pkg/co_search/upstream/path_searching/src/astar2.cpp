#include <path_searching/astar2.h>

#include <algorithm>
#include <array>
#include <sstream>

#include <plan_env/sdf_map.h>
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
Astar::Astar() {}

Astar::~Astar()
{
    for (int i = 0; i < allocate_num_; i++)
        delete path_node_pool_[i];
}

void Astar::init(rclcpp::Node::SharedPtr node, const EDTEnvironment::Ptr& env)
{
    node_            = node;
    resolution_      = node->has_parameter("astar/resolution_astar") ? node->get_parameter("astar/resolution_astar").as_double()
                                                                     : node->declare_parameter<double>("astar/resolution_astar", 0.2);
    lambda_heu_      = node->has_parameter("astar/lambda_heu") ? node->get_parameter("astar/lambda_heu").as_double()
                                                               : node->declare_parameter<double>("astar/lambda_heu", 10000.0);
    max_search_time_ = node->has_parameter("astar/max_search_time") ? node->get_parameter("astar/max_search_time").as_double()
                                                                    : node->declare_parameter<double>("astar/max_search_time", 0.001);
    allocate_num_    = node->has_parameter("astar/allocate_num") ? node->get_parameter("astar/allocate_num").as_int()
                                                                 : node->declare_parameter<int>("astar/allocate_num", 1000000);

    hybrid_param_.min_turning_radius = node->has_parameter("astar/min_turning_radius") ? node->get_parameter("astar/min_turning_radius").as_double()
                                                                                       : node->declare_parameter<double>("astar/min_turning_radius", 0.0);
    hybrid_param_.max_steering_angle = node->has_parameter("astar/max_steering_angle") ? node->get_parameter("astar/max_steering_angle").as_double()
                                                                                       : node->declare_parameter<double>("astar/max_steering_angle", 1.5);
    hybrid_param_.wheel_base         = node->has_parameter("astar/wheel_base") ? node->get_parameter("astar/wheel_base").as_double()
                                                                               : node->declare_parameter<double>("astar/wheel_base", 0.56);
    hybrid_param_.max_speed =
        node->has_parameter("astar/max_speed") ? node->get_parameter("astar/max_speed").as_double() : node->declare_parameter<double>("astar/max_speed", 1.0);
    hybrid_param_.min_speed = node->has_parameter("astar/min_speed") ? node->get_parameter("astar/min_speed").as_double()
                                                                     : node->declare_parameter<double>("astar/min_speed", -0.0001);
    hybrid_param_.dt        = node->has_parameter("astar/dt") ? node->get_parameter("astar/dt").as_double() : node->declare_parameter<double>("astar/dt", 0.1);
    hybrid_param_.num_steering_angles = node->has_parameter("astar/num_steering_angles") ? node->get_parameter("astar/num_steering_angles").as_int()
                                                                                         : node->declare_parameter<int>("astar/num_steering_angles", 15);
    hybrid_param_.num_speeds =
        node->has_parameter("astar/num_speeds") ? node->get_parameter("astar/num_speeds").as_int() : node->declare_parameter<int>("astar/num_speeds", 7);

    double steering_step = 2.0 * hybrid_param_.max_steering_angle / (hybrid_param_.num_steering_angles - 1);
    for (int i = 0; i < hybrid_param_.num_steering_angles; ++i)
    {
        steering_angles_.push_back(-hybrid_param_.max_steering_angle + i * steering_step);
    }

    double speed_step = (hybrid_param_.max_speed - hybrid_param_.min_speed) / (hybrid_param_.num_speeds - 1);
    for (int i = 0; i < hybrid_param_.num_speeds; ++i)
    {
        speeds_.push_back(hybrid_param_.min_speed + i * speed_step);
    }

    tie_breaker_ = 1.0 + 1.0 / 1000;

    this->edt_env_ = env;

    /* ---------- map params ---------- */
    this->inv_resolution_ = 1.0 / resolution_;
    edt_env_->sdf_map_->getRegion(origin_, map_size_3d_);
    // ROS_INFO_STREAM("Map origin: " << origin_.transpose());
    // ROS_INFO_STREAM("Map size: " << map_size_3d_.transpose());

    /* ---------- initialize z candidates ---------- */
    z_candidates_.clear();
    double z_min  = origin_(2) + 0.3;  // Minimum safe height
    double z_max  = origin_(2) + 0.6;  // Maximum height set to 0.6
    double z_step = 0.1;
    for (double z = z_min; z <= z_max + 1e-6; z += z_step)  // Add small epsilon to include 0.6
    {
        z_candidates_.push_back(z);
    }
    // ROS_INFO_STREAM("Initialized " << z_candidates_.size() << " z candidates: ");
    // for (double z : z_candidates_)
    // {
    //     ROS_INFO_STREAM("  z = " << z);
    // }

    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
        path_node_pool_[i] = new Node;
    }
    use_node_num_         = 0;
    iter_num_             = 0;
    early_terminate_cost_ = 0.0;
}

void Astar::setResolution(const double& res)
{
    resolution_           = res;
    this->inv_resolution_ = 1.0 / resolution_;
}

void Astar::setHybridParam(const HybridAstarParam& param)
{
    hybrid_param_ = param;
    steering_angles_.clear();
    speeds_.clear();

    double steering_step = 2.0 * hybrid_param_.max_steering_angle / (hybrid_param_.num_steering_angles - 1);
    for (int i = 0; i < hybrid_param_.num_steering_angles; ++i)
    {
        steering_angles_.push_back(-hybrid_param_.max_steering_angle + i * steering_step);
    }

    double speed_step = (hybrid_param_.max_speed - hybrid_param_.min_speed) / (hybrid_param_.num_speeds - 1);
    for (int i = 0; i < hybrid_param_.num_speeds; ++i)
    {
        speeds_.push_back(hybrid_param_.min_speed + i * speed_step);
    }
}

void Astar::setLambdaHeu(double lambda_heu)
{
    lambda_heu_ = lambda_heu;
}

void Astar::setMaxSearchTime(double max_search_time)
{
    max_search_time_ = max_search_time;
}

int Astar::search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt, bool optimistic)
{
    const auto t_start  = node_->now();
    NodePtr    cur_node = path_node_pool_[0];
    cur_node->parent    = NULL;

    // Use the first safe z value for the start point
    double start_z = getSafeZ(start_pt, optimistic);
    if (start_z < 0)
    {
        // ROS_ERROR_STREAM("No safe z value found for start point " << start_pt.transpose());
        return NO_PATH;
    }

    cur_node->state.pos    = start_pt;
    cur_node->state.pos(2) = start_z;
    cur_node->state.theta  = atan2(end_pt(1) - start_pt(1), end_pt(0) - start_pt(0));
    cur_node->state.v      = 0.0;
    cur_node->state.phi    = 0.0;

    posToIndex(cur_node->state.pos, cur_node->index);
    cur_node->g_score = 0.0;
    cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->state.pos, end_pt);

    Eigen::Vector3i end_index;
    posToIndex(end_pt, end_index);

    open_set_.push(cur_node);
    open_set_map_.insert(make_pair(cur_node->index, cur_node));
    use_node_num_ += 1;

    const auto t1 = node_->now();

    // ROS_INFO_STREAM("Starting A* search from " << start_pt.transpose() << " to " << end_pt.transpose() << ", start_z=" << start_z << ", optimistic=" <<
    // optimistic);

    /* ---------- search loop ---------- */
    while (!open_set_.empty())
    {
        cur_node = open_set_.top();

        bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 && abs(cur_node->index(1) - end_index(1)) <= 1;
        if (reach_end)
        {
            // ROS_INFO_STREAM("Reached end! Final node pos=" << cur_node->state.pos.transpose());
            backtrack(cur_node, end_pt);
            return REACH_END;
        }

        if ((node_->now() - t1).seconds() > max_search_time_)
        {
            early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->state.pos, end_pt);
            // ROS_WARN_STREAM("Search timed out after " << max_search_time_ << " seconds. Last pos: " << cur_node->state.pos.transpose());
            return NO_PATH;
        }

        open_set_.pop();
        open_set_map_.erase(cur_node->index);
        close_set_map_.insert(make_pair(cur_node->index, 1));
        iter_num_ += 1;

        for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
            for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
            {
                if (abs(dx) < 1e-3 && abs(dy) < 1e-3)
                    continue;

                VehicleState next_state = cur_node->state;
                next_state.pos(0) += dx;
                next_state.pos(1) += dy;
                next_state.theta = atan2(dy, dx);

                // Try all z candidates to find a safe state
                bool safe_state_found = false;
                for (double z : z_candidates_)
                {
                    next_state.pos(2) = z;
                    if (isStateSafe(next_state, optimistic))
                    {
                        safe_state_found = true;
                        break;
                    }
                }

                if (!safe_state_found)
                {
                    continue;
                }

                Eigen::Vector3i next_idx;
                posToIndex(next_state.pos, next_idx);
                if (close_set_map_.find(next_idx) != close_set_map_.end())
                {
                    continue;
                }

                NodePtr neighbor;
                double  tmp_g_score = cur_node->g_score + sqrt(dx * dx + dy * dy);

                auto node_iter = open_set_map_.find(next_idx);
                if (node_iter == open_set_map_.end())
                {
                    neighbor = path_node_pool_[use_node_num_];
                    use_node_num_ += 1;
                    if (use_node_num_ == allocate_num_)
                    {
                        // ROS_ERROR_STREAM("Ran out of node pool! No path found.");
                        return NO_PATH;
                    }
                    neighbor->state = next_state;
                    neighbor->index = next_idx;
                }
                else if (tmp_g_score < node_iter->second->g_score)
                {
                    neighbor = node_iter->second;
                }
                else
                {
                    continue;
                }

                neighbor->parent  = cur_node;
                neighbor->g_score = tmp_g_score;
                neighbor->f_score = neighbor->g_score + lambda_heu_ * getDiagHeu(neighbor->state.pos, end_pt);

                open_set_.push(neighbor);
                open_set_map_.insert(make_pair(neighbor->index, neighbor));
            }
    }
    const auto t_end         = node_->now();
    double     total_time_ms = (t_end - t_start).seconds() * 1000.0;
    RCLCPP_INFO(rclcpp::get_logger("astar2"), " total time: %f", total_time_ms);
    // ROS_WARN_STREAM("Open set empty, no path found! Last node pos: " << cur_node->state.pos.transpose());
    return NO_PATH;
}

double Astar::getSafeZ(const Eigen::Vector3d& pos, bool optimistic)
{
    VehicleState temp_state;
    temp_state.pos   = pos;
    temp_state.theta = 0.0;
    temp_state.v     = 0.0;
    temp_state.phi   = 0.0;

    for (double z : z_candidates_)
    {
        temp_state.pos(2) = z;
        if (isStateSafe(temp_state, optimistic))
        {
            return z;
        }
    }

    return -1.0;
}

bool Astar::isSegmentVehicleFree(const Eigen::Vector3d& a, const Eigen::Vector3d& b, bool optimistic)
{
    const double len_xy = (b.head<2>() - a.head<2>()).norm();
    if (len_xy < 1e-4)
    {
        VehicleState st;
        st.pos         = a;
        st.theta       = 0.0;
        st.v           = 0.0;
        st.phi         = 0.0;
        const double z = getSafeZ(st.pos, optimistic);
        if (z < 0.0)
            return false;
        st.pos.z() = z;
        return isStateSafe(st, optimistic);
    }

    const double theta = atan2(b.y() - a.y(), b.x() - a.x());
    const double step  = std::max(resolution_ * 0.5, 0.08);
    const int    n     = std::max(2, static_cast<int>(std::ceil(len_xy / step)));

    for (int i = 0; i <= n; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(n);
        VehicleState st;
        st.pos              = a + t * (b - a);
        st.theta            = theta;
        st.v                = 0.0;
        st.phi              = 0.0;
        const double safe_z = getSafeZ(st.pos, optimistic);
        if (safe_z < 0.0)
            return false;
        st.pos.z() = safe_z;
        if (!checkVehicleShape(st))
            return false;
    }
    return true;
}

double Astar::getEarlyTerminateCost()
{
    return early_terminate_cost_;
}

void Astar::reset()
{
    open_set_map_.clear();
    close_set_map_.clear();
    path_nodes_.clear();

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);
    for (int i = 0; i < use_node_num_; i++)
    {
        path_node_pool_[i]->parent = NULL;
    }
    use_node_num_ = 0;
    iter_num_     = 0;
}

double Astar::pathLength(const vector<Eigen::Vector3d>& path)
{
    double length = 0.0;
    if (path.size() < 2)
        return length;
    for (int i = 0; i < path.size() - 1; ++i)
        length += (path[i + 1] - path[i]).norm();
    return length;
}

void Astar::backtrack(const NodePtr& end_node, const Eigen::Vector3d& end)
{
    path_nodes_.clear();
    // Path length is bounded by expanded nodes; avoids repeated realloc during push_back (perf).
    const size_t reserve_n = static_cast<size_t>(std::max(0, use_node_num_)) + 4U;
    path_nodes_.reserve(reserve_n);

    path_nodes_.push_back(end);
    path_nodes_.push_back(end_node->state.pos);
    NodePtr cur_node = end_node;
    while (cur_node->parent != NULL)
    {
        cur_node = cur_node->parent;
        path_nodes_.push_back(cur_node->state.pos);
    }
    reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> Astar::getPath()
{
    return path_nodes_;
}

double Astar::getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2)
{
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double h;
    double diag = std::min(dx, dy);
    dx -= diag;
    dy -= diag;
    h = sqrt(2.0) * diag + dx + dy;
    return tie_breaker_ * h;
}

double Astar::getManhHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2)
{
    double dx = fabs(x1(0) - x2(0));
    double dy = fabs(x1(1) - x2(1));
    double dz = fabs(x1(2) - x2(2));
    return tie_breaker_ * (dx + dy + dz);
}

double Astar::getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2)
{
    return tie_breaker_ * (x2 - x1).norm();
}

std::vector<Eigen::Vector3d> Astar::getVisited()
{
    vector<Eigen::Vector3d> visited;
    for (int i = 0; i < use_node_num_; ++i)
        visited.push_back(path_node_pool_[i]->state.pos);
    return visited;
}

void Astar::posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx)
{
    idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

VehicleState Astar::propagateState(const VehicleState& state, double v, double phi, double dt)
{
    VehicleState next;
    double       beta = atan2(tan(phi) * hybrid_param_.wheel_base / 2.0, 1.0);
    next.pos          = state.pos;
    next.pos(0) += v * cos(state.theta + beta) * dt;
    next.pos(1) += v * sin(state.theta + beta) * dt;
    next.theta = state.theta + v * tan(phi) * dt / hybrid_param_.wheel_base;
    next.v     = v;
    next.phi   = phi;
    return next;
}

bool Astar::isStateSafe(const VehicleState& state, bool optimistic)
{
    if (!edt_env_->sdf_map_->isInBox(state.pos))
    {
        return false;
    }

    int inflate_occ = edt_env_->sdf_map_->getInflateOccupancy(state.pos);
    if (inflate_occ == 1)
    {
        return false;
    }

    if (!optimistic && edt_env_->sdf_map_->getOccupancy(state.pos) == SDFMap::UNKNOWN)
    {
        return false;
    }

    return checkVehicleShape(state);
}

bool Astar::checkVehicleShape(const VehicleState& state)
{
    constexpr double kVehicleLength = 1.045;  // 车的长度
    constexpr double kVehicleWidth  = 0.78;   // 车的宽度, 给一定余量
    constexpr int    kMaxAttempts   = 3;      // 检查车辆尝试次数
    double           scale_factor   = 1.0;    // 车辆尺寸缩放因子
    constexpr double kScaleStep     = 0.1;    // 每次尝试减少的步长

    for (int attempt = 0; attempt < kMaxAttempts; ++attempt)
    {
        const double scaled_length = kVehicleLength * scale_factor;
        const double scaled_width  = kVehicleWidth * scale_factor;
        const double cos_theta     = cos(state.theta);
        const double sin_theta     = sin(state.theta);

        const std::array<Eigen::Vector2d, 4> corners = { Eigen::Vector2d(state.pos(0) + (scaled_length / 2 * cos_theta - scaled_width / 2 * sin_theta),
                                                                         state.pos(1) + (scaled_length / 2 * sin_theta + scaled_width / 2 * cos_theta)),
                                                         Eigen::Vector2d(state.pos(0) + (scaled_length / 2 * cos_theta + scaled_width / 2 * sin_theta),
                                                                         state.pos(1) + (scaled_length / 2 * sin_theta - scaled_width / 2 * cos_theta)),
                                                         Eigen::Vector2d(state.pos(0) + (-scaled_length / 2 * cos_theta - scaled_width / 2 * sin_theta),
                                                                         state.pos(1) + (-scaled_length / 2 * sin_theta + scaled_width / 2 * cos_theta)),
                                                         Eigen::Vector2d(state.pos(0) + (-scaled_length / 2 * cos_theta + scaled_width / 2 * sin_theta),
                                                                         state.pos(1) + (-scaled_length / 2 * sin_theta - scaled_width / 2 * cos_theta)) };

        bool all_safe = true;
        for (const auto& corner : corners)
        {
            const Eigen::Vector3d check_pt(corner(0), corner(1), state.pos(2));
            const int             inflate_occ = edt_env_->sdf_map_->getInflateOccupancy(check_pt);

            if (inflate_occ == 1)
            {
                all_safe = false;
                break;
            }
        }

        if (all_safe)
        {
            return true;
        }

        scale_factor -= kScaleStep;
        if (scale_factor < 0.5)
        {
            break;
        }
    }

    return false;
}

double Astar::getTransitionCost(const VehicleState& s1, const VehicleState& s2)
{
    double dist_cost  = (s2.pos - s1.pos).norm();
    double steer_cost = abs(s2.phi - s1.phi) * 0.1;
    double vel_cost   = abs(s2.v - s1.v) * 0.1;
    double dir_cost   = abs(s2.theta - s1.theta) * 0.1;
    return dist_cost + steer_cost + vel_cost + dir_cost;
}

double Astar::getHybridHeu(const VehicleState& state, const Eigen::Vector3d& goal)
{
    double pos_heu = getDiagHeu(state.pos, goal);
    double dir_heu = abs(atan2(goal(1) - state.pos(1), goal(0) - state.pos(0)) - state.theta);
    dir_heu        = min(dir_heu, 2 * M_PI - dir_heu);
    return tie_breaker_ * (pos_heu + 0.1 * dir_heu);
}

bool Astar::isReachGoal(const VehicleState& state, const Eigen::Vector3d& goal)
{
    bool   pos_reached = (state.pos.head<2>() - goal.head<2>()).norm() < resolution_;
    double goal_theta  = atan2(goal(1) - state.pos(1), goal(0) - state.pos(0));
    double theta_diff  = abs(state.theta - goal_theta);
    theta_diff         = min(theta_diff, 2 * M_PI - theta_diff);
    bool theta_reached = theta_diff < 0.1;
    return pos_reached && theta_reached;
}

}  // namespace fast_planner