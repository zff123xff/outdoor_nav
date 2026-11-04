/*
 * snn_region_interface.cpp — 区域分配器抽象接口的工厂 + 实现。
 *
 * 包含：
 *   LocalRegionAllocator — 包裹原始 SnnRegionAllocator 的本地实现
 *   PpuRegionAllocator   — 远程 PPU 区域分配（含本地 layout/sticky 状态）；
 *                          PPU 忙（3403 code=30）时本次调用回退本地 SNN 分配
 *   createRegionAllocator — 工厂函数
 */

#include "ppu_opt/snn_region_interface.h"
#include "snn_region/snn_region_allocator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace fast_planner
{

// ════════════════════════════════════════════════════════
// 本地布局工具函数（与 SnnRegionAllocator 算法兼容）
// ════════════════════════════════════════════════════════

namespace
{

    void buildLayout(RegionLayout& layout, int total_regions, int rows, int cols, double min_x, double max_x, double min_y, double max_y)
    {
        layout.total_regions = total_regions;
        layout.rows          = rows;
        layout.cols          = cols;
        layout.min_x         = min_x;
        layout.max_x         = max_x;
        layout.min_y         = min_y;
        layout.max_y         = max_y;
        layout.width         = (max_x - min_x) / static_cast<double>(cols);
        layout.height        = (max_y - min_y) / static_cast<double>(rows);
    }

    int ownerOfPoint(const RegionLayout& layout, const Eigen::Vector3d& point)
    {
        if (layout.total_regions <= 1)
            return 0;
        int col = static_cast<int>(std::floor((point.x() - layout.min_x) / layout.width));
        int row = static_cast<int>(std::floor((point.y() - layout.min_y) / layout.height));
        col     = std::max(0, std::min(col, layout.cols - 1));
        row     = std::max(0, std::min(row, layout.rows - 1));
        return row * layout.cols + col;
    }

    void boundsOfRegion(const RegionLayout& layout, int region_id, double& min_x, double& max_x, double& min_y, double& max_y)
    {
        if (region_id < 0 || region_id >= layout.total_regions)
        {
            min_x = layout.min_x;
            max_x = layout.max_x;
            min_y = layout.min_y;
            max_y = layout.max_y;
            return;
        }
        int col = region_id % layout.cols;
        int row = region_id / layout.cols;
        min_x   = layout.min_x + col * layout.width;
        max_x   = min_x + layout.width;
        min_y   = layout.min_y + row * layout.height;
        max_y   = min_y + layout.height;
    }

    // 简单的距离贪心分配（用于本地 fallback 和 computeDroneRegions）
    std::vector<int> greedyAssignByDistance(const RegionLayout& layout, const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions,
                                            const std::unordered_map<int, int>& sticky)
    {
        int              n = static_cast<int>(drone_ids.size());
        std::vector<int> assignments(n, -1);

        for (int i = 0; i < n; ++i)
        {
            int did = drone_ids[i];

            // 优先使用 sticky 分配
            auto it = sticky.find(did);
            if (it != sticky.end() && it->second >= 0 && it->second < layout.total_regions)
            {
                assignments[i] = it->second;
                continue;
            }

            // 距离最近区域
            int    best_region = 0;
            double best_dist   = std::numeric_limits<double>::max();
            for (int r = 0; r < layout.total_regions; ++r)
            {
                double min_x, max_x, min_y, max_y;
                boundsOfRegion(layout, r, min_x, max_x, min_y, max_y);
                double cx = (min_x + max_x) * 0.5;
                double cy = (min_y + max_y) * 0.5;
                double d  = std::hypot(positions[i].x() - cx, positions[i].y() - cy);
                if (d < best_dist)
                {
                    best_dist   = d;
                    best_region = r;
                }
            }
            assignments[i] = best_region;
        }
        return assignments;
    }

}  // anonymous namespace

// ════════════════════════════════════════════════════════
// LocalRegionAllocator（包裹原始 SnnRegionAllocator）
// ════════════════════════════════════════════════════════

class LocalRegionAllocator : public IRegionAllocator
{
public:
    void configure(rclcpp::Node::SharedPtr node, double map_size_x, double map_size_y, int total_drones) override
    {
        alloc_.configure(node, map_size_x, map_size_y, total_drones);
        syncLayout();
    }
    const RegionLayout& layout() const override
    {
        return cached_layout_;
    }
    std::vector<int> assignDroneRegions(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions) override
    {
        return alloc_.assignDroneRegions(drone_ids, positions);
    }
    std::vector<int> computeDroneRegions(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions) override
    {
        return alloc_.computeDroneRegions(drone_ids, positions);
    }
    int ownerRegionOfPoint(const Eigen::Vector3d& point) const override
    {
        return alloc_.ownerRegionOfPoint(point);
    }
    int ownerRegionOfGridCenter(const Eigen::Vector3d& center) const override
    {
        return alloc_.ownerRegionOfGridCenter(center);
    }
    void getRegionBounds(int region_id, double& min_x, double& max_x, double& min_y, double& max_y) const override
    {
        alloc_.getRegionBounds(region_id, min_x, max_x, min_y, max_y);
    }
    bool getStickyRegionForDrone(int drone_id, int& region_id) const override
    {
        return alloc_.getStickyRegionForDrone(drone_id, region_id);
    }
    void setStickyAssignment(int drone_id, int region_id) override
    {
        alloc_.setStickyAssignment(drone_id, region_id);
    }
    void clearSticky() override
    {
        alloc_.clearSticky();
    }
    void applyConsensusAssignments(const std::vector<int>& assignments) override
    {
        alloc_.applyConsensusAssignments(assignments);
    }
    void reconfigureDynamic(int total_regions, double min_x, double max_x, double min_y, double max_y) override
    {
        alloc_.reconfigureDynamic(total_regions, min_x, max_x, min_y, max_y);
        syncLayout();
    }
    void setRegionActiveGridCounts(const std::vector<int>& counts) override
    {
        alloc_.setRegionActiveGridCounts(counts);
    }
    void setRegionReachableGridCounts(const std::vector<int>& counts) override
    {
        alloc_.setRegionReachableGridCounts(counts);
    }

private:
    void syncLayout()
    {
        const auto& src              = alloc_.layout();
        cached_layout_.rows          = src.rows;
        cached_layout_.cols          = src.cols;
        cached_layout_.total_regions = src.total_regions;
        cached_layout_.min_x         = src.min_x;
        cached_layout_.max_x         = src.max_x;
        cached_layout_.min_y         = src.min_y;
        cached_layout_.max_y         = src.max_y;
        cached_layout_.width         = src.width;
        cached_layout_.height        = src.height;
    }
    SnnRegionAllocator alloc_;
    RegionLayout       cached_layout_;
};

// ════════════════════════════════════════════════════════
// PpuRegionAllocator（远程 PPU 实现）
// ════════════════════════════════════════════════════════

class PpuRegionAllocator : public IRegionAllocator
{
public:
    explicit PpuRegionAllocator(const std::string& host, uint16_t port) : client_(ppu_opt::PpuRegionOptions{ host, port, 1, 60000 }) {}

    void configure(rclcpp::Node::SharedPtr node, double map_size_x, double map_size_y, int total_drones) override
    {
        // 读取参数（与原始 SnnRegionAllocator 一致的参数名）
        int region_rows = node->has_parameter("allocate/region_rows") ? node->get_parameter("allocate/region_rows").as_int()
                                                                      : node->declare_parameter<int>("allocate/region_rows", -1);
        int region_cols = node->has_parameter("allocate/region_cols") ? node->get_parameter("allocate/region_cols").as_int()
                                                                      : node->declare_parameter<int>("allocate/region_cols", -1);

        snn_tau_             = node->has_parameter("allocate/snn_tau") ? node->get_parameter("allocate/snn_tau").as_double()
                                                                       : node->declare_parameter<double>("allocate/snn_tau", 2.5);
        snn_dt_              = node->has_parameter("allocate/snn_dt") ? node->get_parameter("allocate/snn_dt").as_double()
                                                                      : node->declare_parameter<double>("allocate/snn_dt", 0.2);
        snn_threshold_       = node->has_parameter("allocate/snn_threshold") ? node->get_parameter("allocate/snn_threshold").as_double()
                                                                             : node->declare_parameter<double>("allocate/snn_threshold", 0.25);
        snn_max_iter_        = node->has_parameter("allocate/snn_max_iter") ? node->get_parameter("allocate/snn_max_iter").as_int()
                                                                            : node->declare_parameter<int>("allocate/snn_max_iter", 300);
        snn_distance_weight_ = node->has_parameter("allocate/snn_distance_weight") ? node->get_parameter("allocate/snn_distance_weight").as_double()
                                                                                   : node->declare_parameter<double>("allocate/snn_distance_weight", 1.2);
        snn_load_weight_     = node->has_parameter("allocate/snn_load_weight") ? node->get_parameter("allocate/snn_load_weight").as_double()
                                                                               : node->declare_parameter<double>("allocate/snn_load_weight", 0.8);

        map_size_x_   = map_size_x;
        map_size_y_   = map_size_y;
        total_drones_ = total_drones;

        // 自动计算行列
        if (region_rows <= 0 || region_cols <= 0)
        {
            int n = 1;
            while (n * n < total_drones)
                ++n;
            region_rows = n;
            region_cols = n;
        }
        rows_ = region_rows;
        cols_ = region_cols;

        double half_x = map_size_x_ * 0.5;
        double half_y = map_size_y_ * 0.5;
        buildLayout(layout_, rows_ * cols_, rows_, cols_, -half_x, half_x, -half_y, half_y);
        occupancy_.assign(layout_.total_regions, 0);

        // 配置本地兜底 allocator（同一套 node 参数），并强制对齐到 PPU 侧布局，
        // 保证 busy 回退产生的 region_id 与 PPU 布局语义一致
        local_fallback_.configure(node, map_size_x_, map_size_y_, total_drones_);
        local_fallback_.reconfigureDynamic(layout_.total_regions, layout_.min_x, layout_.max_x, layout_.min_y, layout_.max_y);
        checkFallbackLayout("configure");
    }

    /** 校验本地兜底 allocator 与 PPU 侧布局是否一致（不一致时 busy 回退的 region_id 会错位） */
    void checkFallbackLayout(const char* where)
    {
        const auto& ll = local_fallback_.layout();
        if (ll.total_regions != layout_.total_regions || ll.rows != layout_.rows || ll.cols != layout_.cols)
            RCLCPP_WARN(rclcpp::get_logger("PpuRegionAllocator"),
                        "[%s] local fallback layout (rows=%d cols=%d n=%d) mismatches PPU layout (rows=%d cols=%d n=%d)", where, ll.rows, ll.cols,
                        ll.total_regions, layout_.rows, layout_.cols, layout_.total_regions);
    }

    const RegionLayout& layout() const override
    {
        return layout_;
    }

    /** 提取的 PPU 调用逻辑，返回分配结果。不修改 sticky/occupancy。 */
    ppu_opt::RegionAllocResult ppuAllocate(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions)
    {
        int n_drones  = static_cast<int>(drone_ids.size());
        int n_regions = layout_.total_regions;

        ppu_opt::PpuFeRegionHeader hdr{};
        hdr.rows            = rows_;
        hdr.cols            = cols_;
        hdr.n_drones        = static_cast<uint32_t>(n_drones);
        hdr.n_regions       = static_cast<uint32_t>(n_regions);
        hdr.algo            = PPU_FE_REGION_FLAG_USE_SNN;
        hdr.max_iter        = static_cast<uint32_t>(snn_max_iter_);
        hdr.min_x           = layout_.min_x;
        hdr.max_x           = layout_.max_x;
        hdr.min_y           = layout_.min_y;
        hdr.max_y           = layout_.max_y;
        hdr.tau             = snn_tau_;
        hdr.dt              = snn_dt_;
        hdr.threshold       = snn_threshold_;
        hdr.distance_weight = snn_distance_weight_;
        hdr.load_weight     = snn_load_weight_;

        hdr.flags = PPU_FE_REGION_FLAG_USE_SNN;
        if (active_counts_valid_)
            hdr.flags |= PPU_FE_REGION_FLAG_ACTIVE_COUNTS_VALID;
        if (reachable_counts_valid_)
            hdr.flags |= PPU_FE_REGION_FLAG_REACHABLE_COUNTS_VALID;

        std::vector<ppu_opt::DroneInput> drone_inputs;
        drone_inputs.reserve(n_drones);
        for (int i = 0; i < n_drones; ++i)
        {
            int                 did = drone_ids[i];
            ppu_opt::DroneInput di;
            di.drone_id      = did;
            di.sticky_region = sticky_.count(did) ? sticky_.at(did) : -1;
            di.pos_x         = positions[i].x();
            di.pos_y         = positions[i].y();
            drone_inputs.push_back(di);
        }

        std::vector<ppu_opt::RegionInput> region_inputs;
        region_inputs.reserve(n_regions);
        for (int i = 0; i < n_regions; ++i)
        {
            ppu_opt::RegionInput ri;
            ri.load                 = static_cast<int32_t>(occupancy_[i]);
            ri.active_grid_count    = (i < static_cast<int>(active_counts_.size())) ? active_counts_[i] : 0;
            ri.reachable_grid_count = (i < static_cast<int>(reachable_counts_.size())) ? reachable_counts_[i] : 0;
            region_inputs.push_back(ri);
        }

        return client_.allocate(hdr, drone_inputs, region_inputs);
    }

    std::vector<int> assignDroneRegions(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions) override
    {
        int  n_drones = static_cast<int>(drone_ids.size());
        auto result   = ppuAllocate(drone_ids, positions);

        std::vector<int> assignments;
        if (result.success)
        {
            assignments.assign(result.assignments.begin(), result.assignments.end());
            assignments.resize(n_drones, -1);
            for (int i = 0; i < n_drones && i < static_cast<int>(assignments.size()); ++i)
            {
                if (assignments[i] >= 0)
                {
                    sticky_[drone_ids[i]] = assignments[i];
                    occupancy_[assignments[i]]++;
                }
            }
        }
        else if (result.busy)
        {
            // PPU 忙：本次提交改由本地 SNN 分配兜底；长连接保持，下次调用仍优先 PPU
            warnBusyThrottled("assignDroneRegions");
            assignments = local_fallback_.assignDroneRegions(drone_ids, positions);
            assignments.resize(n_drones, -1);
            for (int i = 0; i < n_drones; ++i)
            {
                if (assignments[i] >= 0 && assignments[i] < static_cast<int>(occupancy_.size()))
                {
                    sticky_[drone_ids[i]] = assignments[i];
                    occupancy_[assignments[i]]++;
                }
            }
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("PpuRegionAllocator"), "PPU assignDroneRegions failed: %s", result.error_msg.c_str());
            assignments = greedyAssignByDistance(layout_, drone_ids, positions, sticky_);
            for (int i = 0; i < n_drones; ++i)
            {
                if (assignments[i] >= 0)
                {
                    sticky_[drone_ids[i]] = assignments[i];
                    occupancy_[assignments[i]]++;
                }
            }
        }
        return assignments;
    }

    std::vector<int> computeDroneRegions(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions) override
    {
        // 调 PPU 获取 SNN 膜电位计算结果（纯提案，不修改 sticky）
        auto result = ppuAllocate(drone_ids, positions);
        if (result.success)
        {
            std::vector<int> assignments(result.assignments.begin(), result.assignments.end());
            assignments.resize(static_cast<size_t>(drone_ids.size()), -1);
            return assignments;
        }
        if (result.busy)
        {
            // PPU 忙：本次提案改由本地 SNN 计算兜底（纯提案，不修改 sticky）
            warnBusyThrottled("computeDroneRegions");
            return local_fallback_.computeDroneRegions(drone_ids, positions);
        }
        // PPU 失败时 fallback 到本地贪心距离（纯提案，不修改 sticky）
        RCLCPP_WARN(rclcpp::get_logger("PpuRegionAllocator"), "PPU computeDroneRegions failed, using greedy distance fallback: %s", result.error_msg.c_str());
        return greedyAssignByDistance(layout_, drone_ids, positions, sticky_);
    }

    int ownerRegionOfPoint(const Eigen::Vector3d& point) const override
    {
        return ownerOfPoint(layout_, point);
    }

    int ownerRegionOfGridCenter(const Eigen::Vector3d& center) const override
    {
        return ownerOfPoint(layout_, center);
    }

    void getRegionBounds(int region_id, double& min_x, double& max_x, double& min_y, double& max_y) const override
    {
        boundsOfRegion(layout_, region_id, min_x, max_x, min_y, max_y);
    }

    bool getStickyRegionForDrone(int drone_id, int& region_id) const override
    {
        auto it = sticky_.find(drone_id);
        if (it != sticky_.end())
        {
            region_id = it->second;
            return true;
        }
        return false;
    }

    void setStickyAssignment(int drone_id, int region_id) override
    {
        sticky_[drone_id] = region_id;
        local_fallback_.setStickyAssignment(drone_id, region_id);
    }

    void clearSticky() override
    {
        sticky_.clear();
        occupancy_.assign(layout_.total_regions, 0);
        local_fallback_.clearSticky();
    }

    void applyConsensusAssignments(const std::vector<int>& assignments) override
    {
        sticky_.clear();
        occupancy_.assign(layout_.total_regions, 0);
        for (size_t i = 0; i < assignments.size(); ++i)
        {
            int did = static_cast<int>(i) + 1;
            int rid = assignments[i];
            if (rid >= 0)
            {
                rid          = std::max(0, std::min(rid, layout_.total_regions - 1));
                sticky_[did] = rid;
                if (rid < static_cast<int>(occupancy_.size()))
                    occupancy_[rid]++;
            }
        }
        local_fallback_.applyConsensusAssignments(assignments);
    }

    void reconfigureDynamic(int total_regions, double min_x, double max_x, double min_y, double max_y) override
    {
        int r = static_cast<int>(std::sqrt(total_regions));
        int c = r;
        while (r * c < total_regions)
            ++c;
        if (r * c < total_regions)
            ++r;
        rows_ = r;
        cols_ = c;

        buildLayout(layout_, total_regions, r, c, min_x, max_x, min_y, max_y);
        occupancy_.assign(total_regions, 0);

        // 与本地兜底保持 sticky 语义一致：截断越界历史分配
        for (auto& kv : sticky_)
            kv.second = std::max(0, std::min(kv.second, std::max(0, total_regions - 1)));

        // 同步本地兜底布局（形状计算规则不同，少数分区数下可能不一致，由 checkFallbackLayout 告警）
        local_fallback_.reconfigureDynamic(total_regions, min_x, max_x, min_y, max_y);
        checkFallbackLayout("reconfigureDynamic");
    }

    void setRegionActiveGridCounts(const std::vector<int>& counts) override
    {
        active_counts_       = counts;
        active_counts_valid_ = true;
        local_fallback_.setRegionActiveGridCounts(counts);
    }

    void setRegionReachableGridCounts(const std::vector<int>& counts) override
    {
        reachable_counts_       = counts;
        reachable_counts_valid_ = true;
        local_fallback_.setRegionReachableGridCounts(counts);
    }

private:
    /** 忙回退日志限流（每 2s 最多一条），避免 PPU 持续被占用时刷屏 */
    void warnBusyThrottled(const char* method)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_busy_warn_ < std::chrono::seconds(2))
            return;
        last_busy_warn_ = now;
        RCLCPP_WARN(rclcpp::get_logger("PpuRegionAllocator"), "[%s] PPU busy (code=%u), fallback to local SNN allocator for this call", method,
                    PPU_NET_ERR_BUSY);
    }

    ppu_opt::PpuRegionClient client_;
    LocalRegionAllocator     local_fallback_;  // PPU 忙时的本地兜底（布局/sticky 与 PPU 侧保持同步）

    RegionLayout layout_;
    double       map_size_x_{ 50.0 };
    double       map_size_y_{ 50.0 };
    int          total_drones_{ 1 };
    int          rows_{ 1 };
    int          cols_{ 1 };

    double snn_tau_{ 2.5 };
    double snn_dt_{ 0.2 };
    double snn_threshold_{ 0.25 };
    int    snn_max_iter_{ 300 };
    double snn_distance_weight_{ 1.2 };
    double snn_load_weight_{ 0.8 };

    std::unordered_map<int, int>          sticky_;
    std::vector<unsigned int>             occupancy_;
    std::vector<int>                      active_counts_;
    std::vector<int>                      reachable_counts_;
    bool                                  active_counts_valid_{ false };
    bool                                  reachable_counts_valid_{ false };
    std::chrono::steady_clock::time_point last_busy_warn_{};
};

// ════════════════════════════════════════════════════════
// 工厂函数
// ════════════════════════════════════════════════════════

std::shared_ptr<IRegionAllocator> createRegionAllocator(rclcpp::Node::SharedPtr node)
{
    bool enable_ppu = node->has_parameter("ppu/enable_region_allocation") ? node->get_parameter("ppu/enable_region_allocation").as_bool()
                                                                          : node->declare_parameter<bool>("ppu/enable_region_allocation", false);

    if (enable_ppu)
    {
        std::string host = node->has_parameter("ppu/region_host") ? node->get_parameter("ppu/region_host").as_string()
                                                                  : node->declare_parameter<std::string>("ppu/region_host", "192.168.137.62");
        int         port =
            node->has_parameter("ppu/region_port") ? node->get_parameter("ppu/region_port").as_int() : node->declare_parameter<int>("ppu/region_port", 19589);

        RCLCPP_INFO(rclcpp::get_logger("createRegionAllocator"), "Using PPU region allocator (local fallback on busy): %s:%d", host.c_str(), port);
        return std::make_shared<PpuRegionAllocator>(host, static_cast<uint16_t>(port));
    }

    RCLCPP_INFO(rclcpp::get_logger("createRegionAllocator"), "Using local region allocator");
    return std::make_shared<LocalRegionAllocator>();
}

}  // namespace fast_planner