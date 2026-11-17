#ifndef _ASTAR2_H

#define _ASTAR2_H

#include <Eigen/Eigen>

#include <iostream>

#include <map>

#include <rclcpp/logging.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

#include <unordered_map>

#include <plan_env/edt_environment.h>

#include <boost/functional/hash.hpp>

#include <queue>

#include <path_searching/matrix_hash.h>

#include <vector>
#include <iomanip> // 用于 std::fixed 和 std::setprecision

namespace fast_planner
{
// [Hybrid A*改进] 车辆状态结构体，包含位置、朝向、速度和转向角

struct VehicleState

{
    Eigen::Vector3d pos; // 位置 (x,y,z)

    double theta; // 朝向角 (rad)

    double v; // 速度 (m/s)

    double phi; // 转向角 (rad)
};

// [Hybrid A*改进] Hybrid A*参数结构体

struct HybridAstarParam

{
    double min_turning_radius; // 最小转弯半径 (m)

    double max_steering_angle; // 最大转向角 (rad)

    double wheel_base; // 轴距 (m)

    double max_speed; // 最大速度 (m/s)

    double min_speed; // 最小速度 (m/s)

    double dt; // 时间步长 (s)

    int num_steering_angles; // 转向角离散数量

    int num_speeds; // 速度离散数量
};

// [Hybrid A*改进] 修改Node结构，使用VehicleState替代position

class Node

{
public:
    Eigen::Vector3i index;

    VehicleState state; // 替换原来的position，包含完整状态信息

    double g_score, f_score;

    Node *parent;

    Node()

    {
        parent = NULL;
    }

    ~Node(){};
};

typedef Node *NodePtr;

// [Hybrid A*改进] 节点比较器，考虑更多状态信息

class NodeComparator0

{
public:
    bool operator()(NodePtr node1, NodePtr node2)

    {
        return node1->f_score > node2->f_score;
    }
};

class Astar

{
public:
    Astar();

    ~Astar();

    enum

    {

        REACH_END = 1,

        NO_PATH = 2

    };

    void init(rclcpp::Node::SharedPtr node, const EDTEnvironment::Ptr &env);

    void reset();

    // [Hybrid A*改进] 设置Hybrid A*参数

    void setHybridParam(const HybridAstarParam &param);

    int search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt, bool optimistic = true);

    void setResolution(const double &res);

    static double pathLength(const vector<Eigen::Vector3d> &path);

    std::vector<Eigen::Vector3d> getPath();

    std::vector<Eigen::Vector3d> getVisited();

    double getEarlyTerminateCost();

    double getSafeZ(const Eigen::Vector3d &pos, bool optimistic);

    bool isSegmentVehicleFree(const Eigen::Vector3d &a, const Eigen::Vector3d &b, bool optimistic = true);

    // 新增 setter 函数

    void setLambdaHeu(double lambda_heu);

    void setMaxSearchTime(double max_search_time);

    double lambda_heu_; // 启发式权重

    double max_search_time_; // 最大搜索时间

private:
    // [Hybrid A*改进] 运动学模型相关函数

    VehicleState propagateState(const VehicleState &state, double v, double phi, double dt);

    bool isStateSafe(const VehicleState &state, bool optimistic);

    double getTransitionCost(const VehicleState &s1, const VehicleState &s2);

    double getHybridHeu(const VehicleState &state, const Eigen::Vector3d &goal);

    bool checkVehicleShape(const VehicleState &state);

    bool isReachGoal(const VehicleState &state, const Eigen::Vector3d &goal);

    // 原有函数

    void backtrack(const NodePtr &end_node, const Eigen::Vector3d &end);

    void posToIndex(const Eigen::Vector3d &pt, Eigen::Vector3i &idx);

    double getDiagHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);

    double getManhHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);

    double getEuclHeu(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2);

    // [Hybrid A*改进] 新增成员变量

    HybridAstarParam hybrid_param_; // Hybrid A*参数

    std::vector<double> steering_angles_; // 离散的转向角

    std::vector<double> speeds_; // 离散的速度

    std::vector<double> z_candidates_; // 候选 z 值

    // 原有成员变量

    rclcpp::Node::SharedPtr node_; // ROS2 节点句柄

    std::vector<NodePtr> path_node_pool_;

    int use_node_num_, iter_num_;

    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;

    std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> open_set_map_;

    std::unordered_map<Eigen::Vector3i, int, matrix_hash<Eigen::Vector3i>> close_set_map_;

    std::vector<Eigen::Vector3d> path_nodes_;

    double early_terminate_cost_;

    EDTEnvironment::Ptr edt_env_;

    // 参数

    double margin_;

    int allocate_num_;

    double tie_breaker_;

    double resolution_, inv_resolution_;

    Eigen::Vector3d map_size_3d_, origin_;
};

} // namespace fast_planner

#endif