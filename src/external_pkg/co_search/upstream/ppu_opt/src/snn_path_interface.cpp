/*
 * snn_path_interface.cpp — SNN 路径规划器抽象接口的工厂 + 实现。
 *
 * 包含：
 *   LocalSnnPathPlanner — 本地 WavefrontPlanner 适配器
 *   PpuSnnPathPlanner   — 远程 PPU 实现；PPU 忙（3403 code=30）时本次调用回退本地
 *   createPathPlanner   — 工厂函数
 */

#include "ppu_opt/snn_path_interface.h"
#include <path_searching/snn_path.h>

#include <algorithm>
#include <chrono>
#include <queue>
#include <utility>

namespace fast_planner
{

class LocalSnnPathPlanner : public ISnnPathPlanner
{
public:
    explicit LocalSnnPathPlanner(const PlannerConfig& cfg) : planner_(convertConfig(cfg)) {}

    void setGrid(const std::vector<std::vector<int>>& grid) override
    {
        planner_.setGrid(grid);
    }

    std::vector<Eigen::Vector3i> plan(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) override
    {
        return planner_.plan(start, goal);
    }

    std::vector<std::uint8_t> computeReachabilityMask(const Eigen::Vector3i& start) override
    {
        return planner_.computeReachabilityMask(start);
    }

    int gridHeight() const override
    {
        return planner_.gridHeight();
    }
    int gridWidth() const override
    {
        return planner_.gridWidth();
    }

private:
    static ::fast_planner::PlannerConfig convertConfig(const PlannerConfig& cfg)
    {
        ::fast_planner::PlannerConfig out;
        out.beta                  = cfg.beta;
        out.tau_e                 = cfg.tau_e;
        out.max_steps             = cfg.max_steps;
        out.max_wave_steps        = cfg.max_wave_steps;
        out.robot_radius          = cfg.robot_radius;
        out.obstacle_close_kernel = cfg.obstacle_close_kernel;
        out.grid_resolution       = cfg.grid_resolution;
        return out;
    }

    ::fast_planner::WavefrontPlanner planner_;
};

// ── PpuSnnPathPlanner (远程 PPU 实现，忙时回退本地) ──────

class PpuSnnPathPlanner : public ISnnPathPlanner
{
public:
    PpuSnnPathPlanner(const std::string& host, uint16_t port, const PlannerConfig& cfg)
        : client_(ppu_opt::PpuPathOptions{ host, port, 1, 180000 }), local_fallback_(cfg)
    {
    }

    void setGrid(const std::vector<std::vector<int>>& grid) override
    {
        if (grid.empty() || grid[0].empty())
        {
            cached_h_ = cached_w_ = 0;
            return;
        }

        // 同步更新本地回退规划器地图：PPU 忙时用同一张栅格直接规划
        local_fallback_.setGrid(grid);

        ppu_opt::GridMap gmap;
        gmap.size_x = static_cast<uint32_t>(grid.size());
        gmap.size_y = static_cast<uint32_t>(grid[0].size());
        gmap.data.resize(static_cast<size_t>(gmap.size_x) * gmap.size_y);

        for (uint32_t x = 0; x < gmap.size_x; ++x)
            for (uint32_t y = 0; y < gmap.size_y; ++y)
                gmap.data[static_cast<size_t>(x) * gmap.size_y + y] = static_cast<uint8_t>(grid[x][y] > 0 ? 1 : 0);

        client_.setMap(gmap);
        cached_h_    = static_cast<int>(gmap.size_x);
        cached_w_    = static_cast<int>(gmap.size_y);
        cached_grid_ = std::move(gmap.data);
    }

    std::vector<Eigen::Vector3i> plan(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) override
    {
        auto result = client_.plan(start.x(), start.y(), goal.x(), goal.y());

        if (result.busy)
        {
            // PPU 忙（3403 code=30）：本次调用回退本地；长连接保持，下次调用仍优先 PPU
            warnBusyThrottled();
            return local_fallback_.plan(start, goal);
        }

        if (!result.success)
            return {};

        std::vector<Eigen::Vector3i> path;
        path.reserve(result.points.size());
        for (const auto& pt : result.points)
            path.emplace_back(pt.x, pt.y, 0);
        return path;
    }

    std::vector<std::uint8_t> computeReachabilityMask(const Eigen::Vector3i& start) override
    {
        if (cached_h_ <= 0 || cached_w_ <= 0 || cached_grid_.empty())
            return {};

        const int                 H = cached_h_, W = cached_w_;
        const size_t              total = static_cast<size_t>(H) * W;
        std::vector<std::uint8_t> mask(total, 0);

        int sx = start.x(), sy = start.y();
        if (sx < 0 || sx >= H || sy < 0 || sy >= W)
            return mask;

        constexpr int DX[8] = { -1, 1, 0, 0, -1, -1, 1, 1 };
        constexpr int DY[8] = { 0, 0, -1, 1, -1, 1, -1, 1 };

        std::queue<std::pair<int, int>> q;
        auto                            idx = [W](int x, int y) { return static_cast<size_t>(x) * W + y; };

        if (cached_grid_[idx(sx, sy)])
            return mask;  // start is blocked

        q.emplace(sx, sy);
        mask[idx(sx, sy)] = 1;

        while (!q.empty())
        {
            auto [x, y] = q.front();
            q.pop();
            for (int d = 0; d < 8; ++d)
            {
                int nx = x + DX[d], ny = y + DY[d];
                if (nx >= 0 && nx < H && ny >= 0 && ny < W && !mask[idx(nx, ny)] && !cached_grid_[idx(nx, ny)])
                {
                    mask[idx(nx, ny)] = 1;
                    q.emplace(nx, ny);
                }
            }
        }
        return mask;
    }

    int gridHeight() const override
    {
        return cached_h_;
    }
    int gridWidth() const override
    {
        return cached_w_;
    }

private:
    /** 忙回退日志限流（每 2s 最多一条），避免 PPU 持续被占用时刷屏 */
    void warnBusyThrottled()
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_busy_warn_ < std::chrono::seconds(2))
            return;
        last_busy_warn_ = now;
        RCLCPP_WARN(rclcpp::get_logger("PpuSnnPathPlanner"), "PPU busy (code=%u), fallback to local planner for this call", PPU_NET_ERR_BUSY);
    }

    ppu_opt::PpuPathClient                client_;
    LocalSnnPathPlanner                   local_fallback_;  // PPU 忙时的本地兜底
    int                                   cached_h_{ 0 };
    int                                   cached_w_{ 0 };
    std::vector<std::uint8_t>             cached_grid_;  // 本地缓存的障碍栅格（0=自由, 1=障碍），用于 computeReachabilityMask
    std::chrono::steady_clock::time_point last_busy_warn_{};
};

// ── 工厂函数 ──────────────────────────────────────────

std::shared_ptr<ISnnPathPlanner> createPathPlanner(rclcpp::Node::SharedPtr node, const PlannerConfig& cfg)
{
    bool enable_ppu = node->has_parameter("ppu/enable_path_planning") ? node->get_parameter("ppu/enable_path_planning").as_bool()
                                                                      : node->declare_parameter<bool>("ppu/enable_path_planning", false);

    if (enable_ppu)
    {
        std::string host = node->has_parameter("ppu/path_host") ? node->get_parameter("ppu/path_host").as_string()
                                                                : node->declare_parameter<std::string>("ppu/path_host", "192.168.137.62");
        int port = node->has_parameter("ppu/path_port") ? node->get_parameter("ppu/path_port").as_int() : node->declare_parameter<int>("ppu/path_port", 19588);

        RCLCPP_INFO(rclcpp::get_logger("createPathPlanner"), "Using PPU path planner (local fallback on busy): %s:%d", host.c_str(), port);
        return std::make_shared<PpuSnnPathPlanner>(host, static_cast<uint16_t>(port), cfg);
    }

    RCLCPP_INFO(rclcpp::get_logger("createPathPlanner"), "Using local WavefrontPlanner path planner");
    return std::make_shared<LocalSnnPathPlanner>(cfg);
}

}  // namespace fast_planner