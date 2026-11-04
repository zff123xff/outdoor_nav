#ifndef WAVEFRONT_PLANNER_H
#define WAVEFRONT_PLANNER_H

#include <Eigen/Dense>
#include <cstdint>
#include <optional>
#include <vector>

namespace fast_planner
{

struct PlannerConfig
{
    int   beta           = -10;
    float tau_e          = 25.0f;
    int   max_steps      = 5'000'000;
    int   max_wave_steps = 5'000'000;
    /// 膨胀用安全半径（米）；与 grid_resolution 配合换算为格数。
    float robot_radius = 0.5;
    /// 障碍闭运算 kernel 边长（格）：3=填约 0.1m 洞，5=约 0.2m；0/1 关闭。
    int obstacle_close_kernel = 3;
    /// 栅格边长（米），需与 SNN 地图分辨率一致。
    float grid_resolution = 0.1f;
};

class WavefrontPlanner
{
public:
    using Grid  = std::vector<std::vector<int>>;
    using Point = std::pair<int, int>;

    explicit WavefrontPlanner(PlannerConfig cfg = PlannerConfig());

    // 初始化地图
    void setGrid(const Grid& grid);

    // 核心接口：传入起点和终点，返回路径
    std::vector<Eigen::Vector3i> plan(const Eigen::Vector3i& start_pt, const Eigen::Vector3i& goal_pt);

    // 在调用方显式提供的原始栅格上计算粗连通性；不包含 setGrid() 的预处理。
    std::vector<std::uint8_t> computeReachabilityMask(const Grid& grid, const Eigen::Vector3i& start_pt);
    // 在 setGrid() 后的规划栅格上计算严格连通性，与 plan() 使用相同的障碍和转移规则。
    std::vector<std::uint8_t> computeReachabilityMask(const Eigen::Vector3i& start_pt);
    int                       gridHeight() const { return H_; }
    int                       gridWidth() const { return W_; }

private:
    // 形态学闭运算：填墙内小于 kernel 的 2D 洞，再膨胀车体半径。
    Grid closeObstacleGaps(const Grid& grid, int kernel_cells) const;
    Grid dilateObstacles(const Grid& grid, int radius) const;
    Grid erodeObstacles(const Grid& grid, int radius) const;

    // 地图膨胀处理：引入小车尺寸的核心逻辑
    Grid inflateGrid(const Grid& grid, float radius);

    // 内部计算逻辑
    const std::vector<std::vector<float>>& compute_wave_time(const std::vector<Point>& starts);

    bool in_bounds(int i, int j) const;
    bool is_valid_transition(const Point& from, const Point& to) const;
    void alloc_states();
    std::vector<Point> find_free_candidates(Point p) const;

    // 成员变量
    Grid          grid_;  // 存储膨胀后的安全地图
    int           H_ = 0, W_ = 0;
    PlannerConfig cfg_;

    // BFS 缓存，setGrid 时按地图尺寸分配。
    std::vector<std::vector<float>> wave_time_buf_;
    std::vector<std::vector<Point>> parent_buf_;

    // [P0-2] 可达性掩码 BFS 打戳 + 队列（每周期复用，避免分配）
    std::vector<std::uint32_t> reach_stamp_;
    std::uint32_t              reach_gen_ = 1;
    std::vector<Point>         reach_queue_;

    // 使用 8 连通方向可以让路径更平滑，减少锯齿
    const std::vector<Point> DIRS = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 }, { -1, -1 }, { -1, 1 }, { 1, -1 }, { 1, 1 } };
};
}  // namespace fast_planner

#endif  // WAVEFRONT_PLANNER_H