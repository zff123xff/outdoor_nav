#ifndef _UNIFORM_GRID_H_
#define _UNIFORM_GRID_H_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <array>
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <utility>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

class RayCaster;

namespace fast_planner
{
class EDTEnvironment;
class Astar;
class HGrid;

// struct GridInfo {};

class GridInfo
{
public:
    GridInfo()
    {
    }
    ~GridInfo()
    {
    }

    int unknown_num_;
    int frontier_num_;
    // FREE voxel count in grid (updated in updateGridInfo), used by dead-zone pruning.
    int free_num_{ 0 };
    Eigen::Vector3d center_;
    unordered_map<int, int> frontier_cell_nums_;
    unordered_map<int, int> contained_frontier_ids_;
    bool is_updated_;
    bool need_divide_, active_;

    bool is_prev_relevant_;
    bool is_cur_relevant_;

    // Visit-based pruning (robot passed but map keeps unknown-only).
    double first_visit_stamp_{ -1.0 };
    double last_visit_stamp_{ -1.0 };
    int    visit_hits_{ 0 };

    // Vertices and their box in xy plane, in current drone's frame
    Eigen::Vector3d vmin_, vmax_;
    vector<Eigen::Vector3d> vertices_;

    // Normals of separating lines in xy plane, associated with vertices_
    vector<Eigen::Vector3d> normals_;
};

class UniformGrid
{
public:
    UniformGrid(const shared_ptr<EDTEnvironment> &edt, rclcpp::Node::SharedPtr node, const int &level);
    ~UniformGrid();

    void initGridData();
    void updateBaseCoor();
    void updateGridData(const int &drone_id, vector<int> &grid_ids, vector<int> &parti_ids, vector<int> &parti_ids_all,
                        const Eigen::Vector3d &robot_world, const std::vector<char> *coarse_reach_mask_for_l2 = nullptr);
    void activateGrids(const vector<int> &ids);

    void inputFrontiers(const vector<Eigen::Vector3d> &avgs);
    void getCostMatrix(const vector<Eigen::Vector3d> &positions, const vector<Eigen::Vector3d> &velocities, const vector<int> &prev_first_grid, const vector<int> &grid_ids, Eigen::MatrixXd &mat);
    void getGridTour(const vector<int> &ids, vector<Eigen::Vector3d> &tour);
    void getFrontiersInGrid(const int &grid_id, vector<int> &ftr_ids);
    void getGridMarker(vector<Eigen::Vector3d> &pts1, vector<Eigen::Vector3d> &pts2);

private:
    void updateGridInfo(const Eigen::Vector3i &id);

    int toAddress(const Eigen::Vector3i &id);
    void adrToIndex(const int &adr, Eigen::Vector3i &idx);
    void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
    void indexToPos(const Eigen::Vector3i &id, const double &inc, Eigen::Vector3d &pos);
    bool insideGrid(const Eigen::Vector3i &id);
    bool isRelevant(const GridInfo &grid);
    bool inInactiveZone(const Eigen::Vector3d &p) const;
    void pruneDeadZoneUnknownGrids(const Eigen::Vector3d &robot_world);
    void pruneLevel2ByCoarseReach(const std::vector<char> &coarse_reach);
    bool lazyFaceTraversable(int adr_a, int adr_b);
    void pruneVisitedStaleGrids(const Eigen::Vector3d &robot_world);

    const std::vector<char> &getDeadZoneReachL1() const
    {
        return dead_zone_reach_l1_;
    }

    shared_ptr<EDTEnvironment> edt_;
    rclcpp::Node::SharedPtr node_; // ROS2 节点句柄
    unique_ptr<Astar> path_finder_;
    vector<GridInfo> grid_data_;

    vector<int> relevant_id_;
    unordered_map<int, int> relevant_map_;
    bool initialized_;
    vector<int> extra_ids_;

    Eigen::Vector3d resolution_;
    Eigen::Vector3d min_, max_;
    Eigen::Vector3i grid_num_;
    int level_;

    int min_unknown_, min_frontier_, min_free_;
    double consistent_cost_, inside_ratio_;
    double w_unknown_;
    bool              dead_zone_prune_enable_{ true };
    double            dead_zone_edge_sample_res_{ 0.2 };
    std::vector<char> dead_zone_reach_l1_;

    bool   visit_prune_enable_{ true };
    int    visit_prune_min_hits_{ 2 };
    double visit_prune_min_sec_{ 4.0 };
    struct InactiveZoneRect
    {
        bool   enable{ false };
        double x_min{ 0.0 };
        double x_max{ 0.0 };
        double y_min{ 0.0 };
        double y_max{ 0.0 };
    };

    static constexpr int kMaxInactiveZones = 2;
    bool inactive_zone_enable_{ false };
    std::array<InactiveZoneRect, kMaxInactiveZones> inactive_zones_;

    // Swarm tf
    Eigen::Matrix3d rot_sw_;
    Eigen::Vector3d trans_sw_;
    bool use_swarm_tf_;

    friend HGrid;
};

} // namespace fast_planner
#endif