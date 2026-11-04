#include <path_searching/snn_path.h>

#include <cassert>
#include <cstdlib>
#include <iostream>

namespace
{
using fast_planner::PlannerConfig;
using fast_planner::WavefrontPlanner;

WavefrontPlanner MakePlanner()
{
    PlannerConfig config;
    config.robot_radius          = 0.0f;
    config.obstacle_close_kernel = 0;
    return WavefrontPlanner(config);
}

void TestOpenGridAndValidDiagonal()
{
    auto planner = MakePlanner();
    planner.setGrid({ { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } });
    const auto path = planner.plan({ 0, 0, 0 }, { 2, 2, 0 });
    assert(path.size() == 3);
    assert(path.front() == Eigen::Vector3i(0, 0, 0));
    assert(path.back() == Eigen::Vector3i(2, 2, 0));
}

void TestBlockedDiagonalCorner()
{
    auto planner = MakePlanner();
    planner.setGrid({ { 0, 1 }, { 1, 0 } });
    assert(planner.plan({ 0, 0, 0 }, { 1, 1, 0 }).empty());
}

void TestObstacleDetour()
{
    auto planner = MakePlanner();
    planner.setGrid({ { 0, 0, 0, 0 }, { 0, 1, 1, 0 }, { 0, 0, 0, 0 } });
    const auto path = planner.plan({ 1, 0, 0 }, { 1, 3, 0 });
    assert(path.size() > 4);
    assert(path.front() == Eigen::Vector3i(1, 0, 0));
    assert(path.back() == Eigen::Vector3i(1, 3, 0));
}

void TestReachableOccupiedGoalFallback()
{
    auto planner = MakePlanner();
    planner.setGrid({ { 0, 0, 0, 0 }, { 0, 1, 1, 0 }, { 0, 0, 1, 0 } });
    const auto path = planner.plan({ 0, 0, 0 }, { 1, 2, 0 });
    assert(!path.empty());
    assert(path.back() == Eigen::Vector3i(0, 2, 0));
}

void TestDisconnectedGoalCandidates()
{
    auto planner = MakePlanner();
    planner.setGrid({ { 0, 0, 1, 0, 0 }, { 0, 0, 1, 0, 0 }, { 0, 0, 1, 0, 0 } });
    assert(planner.plan({ 1, 0, 0 }, { 1, 3, 0 }).empty());
}

void TestProcessedReachabilityMask()
{
    PlannerConfig config;
    config.robot_radius          = 1.0f;
    config.obstacle_close_kernel = 0;
    WavefrontPlanner planner(config);
    planner.setGrid({ { 0, 0, 0, 0, 0 }, { 0, 0, 1, 0, 0 }, { 0, 0, 0, 0, 0 } });

    const auto mask = planner.computeReachabilityMask({ 0, 0, 0 });
    assert(mask.size() == 15);
    // The cell adjacent to the obstacle is free in the raw grid but occupied after SNN inflation.
    assert(mask[static_cast<size_t>(1) * 5 + 1] == 0);
    assert(mask[0] == 1);
}

}  // namespace

int main()
{
    TestOpenGridAndValidDiagonal();
    TestBlockedDiagonalCorner();
    TestObstacleDetour();
    TestReachableOccupiedGoalFallback();
    TestDisconnectedGoalCandidates();
    TestProcessedReachabilityMask();
    std::cout << "wavefront_planner_test passed\n";
    return EXIT_SUCCESS;
}
