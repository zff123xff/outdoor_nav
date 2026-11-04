#ifndef _SNN_REGION_ALLOCATOR_H_
#define _SNN_REGION_ALLOCATOR_H_

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include <unordered_map>
#include <vector>

namespace fast_planner
{

class SnnRegionAllocator
{
public:
    struct RegionLayout
    {
        int    rows{ 1 };
        int    cols{ 1 };
        int    total_regions{ 1 };
        double min_x{ 0.0 };
        double max_x{ 0.0 };
        double min_y{ 0.0 };
        double max_y{ 0.0 };
        double width{ 1.0 };
        double height{ 1.0 };
    };

    SnnRegionAllocator() = default;

    void                configure(rclcpp::Node::SharedPtr node, double map_size_x, double map_size_y, int total_drones);
    const RegionLayout& layout() const;
    std::vector<int>    assignDroneRegions(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions);
    // Compute a proposal without mutating sticky state or historical load. Global
    // coordination commits the accepted proposal explicitly; pair allocation only reads it.
    std::vector<int> computeDroneRegions(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions) const;
    int              ownerRegionOfPoint(const Eigen::Vector3d& point) const;
    int              ownerRegionOfGridCenter(const Eigen::Vector3d& center) const;
    void             getRegionBounds(int region_id, double& min_x, double& max_x, double& min_y, double& max_y) const;
    bool             getStickyRegionForDrone(int drone_id, int& region_id) const;
    void             setStickyAssignment(int drone_id, int region_id);
    void             clearSticky();
    // Replace sticky + load counters with a full assignment vector (index = drone_id-1).
    void applyConsensusAssignments(const std::vector<int>& assignments);
    void reconfigureDynamic(int total_regions, double min_x, double max_x, double min_y, double max_y);
    /** Set per-region active grid count for capacity-normalized load balancing.
     *  Regions with fewer active grids (e.g. due to inactive zones) appear more
     *  loaded relative to their remaining work, discouraging over-assignment.
     *  Pass an empty vector to disable normalization (flat load). */
    void setRegionActiveGridCounts(const std::vector<int>& counts);
    void setRegionReachableGridCounts(const std::vector<int>& counts);

private:
    struct Param
    {
        bool   use_snn_region_assignment{ false };
        int    region_rows{ -1 };
        int    region_cols{ -1 };
        double tau{ 2.5 };
        double dt{ 0.2 };
        double threshold{ 0.25 };
        int    max_iter{ 300 };
        double distance_weight{ 1.2 };
        double load_weight{ 0.8 };
        // sticky 惯性：上一轮分配区域的膜电位初始化为 threshold * sticky_ratio
        // 越大越抗抖动（建议 0.55~0.85），越小越容易因周期重算换区
        double sticky_ratio{ 0.55 };
    };

    int  clampRegion(int region_id) const;
    int  fixedRegionOfDrone(int drone_id) const;
    void buildLayout();
    void buildLayoutForRegions(int region_count);

private:
    Param                                 param_;
    RegionLayout                          layout_;
    double                                map_size_x_{ 50.0 };
    double                                map_size_y_{ 50.0 };
    int                                   total_drones_{ 1 };
    std::unordered_map<int, int>          sticky_assignment_;
    std::unordered_map<int, unsigned int> region_use_count_;
    std::vector<int>                      active_grid_counts_;  // per-region, set externally
    std::vector<int>                      reachable_grid_counts_;
};

}  // namespace fast_planner

#endif