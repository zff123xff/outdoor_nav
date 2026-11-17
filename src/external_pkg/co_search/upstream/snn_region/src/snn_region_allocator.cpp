#include "snn_region/snn_region_allocator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

namespace fast_planner
{

void SnnRegionAllocator::configure(rclcpp::Node::SharedPtr node, double map_size_x, double map_size_y, int total_drones)
{
    map_size_x_   = std::max(1e-3, map_size_x);
    map_size_y_   = std::max(1e-3, map_size_y);
    total_drones_ = std::max(1, total_drones);

    param_.use_snn_region_assignment = node->has_parameter("allocate/use_snn_region_assignment")
                                           ? node->get_parameter("allocate/use_snn_region_assignment").as_bool()
                                           : node->declare_parameter<bool>("allocate/use_snn_region_assignment", param_.use_snn_region_assignment);
    param_.region_rows               = node->has_parameter("allocate/region_rows") ? node->get_parameter("allocate/region_rows").as_int()
                                                                                   : node->declare_parameter<int>("allocate/region_rows", param_.region_rows);
    param_.region_cols               = node->has_parameter("allocate/region_cols") ? node->get_parameter("allocate/region_cols").as_int()
                                                                                   : node->declare_parameter<int>("allocate/region_cols", param_.region_cols);
    param_.tau                       = node->has_parameter("allocate/snn_tau") ? node->get_parameter("allocate/snn_tau").as_double()
                                                                               : node->declare_parameter<double>("allocate/snn_tau", param_.tau);
    param_.dt                        = node->has_parameter("allocate/snn_dt") ? node->get_parameter("allocate/snn_dt").as_double()
                                                                              : node->declare_parameter<double>("allocate/snn_dt", param_.dt);
    param_.threshold                 = node->has_parameter("allocate/snn_threshold") ? node->get_parameter("allocate/snn_threshold").as_double()
                                                                                     : node->declare_parameter<double>("allocate/snn_threshold", param_.threshold);
    param_.max_iter                  = node->has_parameter("allocate/snn_max_iter") ? node->get_parameter("allocate/snn_max_iter").as_int()
                                                                                    : node->declare_parameter<int>("allocate/snn_max_iter", param_.max_iter);
    param_.distance_weight           = node->has_parameter("allocate/snn_distance_weight")
                                           ? node->get_parameter("allocate/snn_distance_weight").as_double()
                                           : node->declare_parameter<double>("allocate/snn_distance_weight", param_.distance_weight);
    param_.load_weight               = node->has_parameter("allocate/snn_load_weight") ? node->get_parameter("allocate/snn_load_weight").as_double()
                                                                                       : node->declare_parameter<double>("allocate/snn_load_weight", param_.load_weight);
    param_.sticky_ratio              = node->has_parameter("allocate/snn_sticky_ratio") ? node->get_parameter("allocate/snn_sticky_ratio").as_double()
                                                                                        : node->declare_parameter<double>("allocate/snn_sticky_ratio", param_.sticky_ratio);

    buildLayout();
}

const SnnRegionAllocator::RegionLayout& SnnRegionAllocator::layout() const
{
    return layout_;
}

void SnnRegionAllocator::buildLayout()
{
    buildLayoutForRegions(total_drones_);
}

void SnnRegionAllocator::buildLayoutForRegions(int region_count)
{
    region_count = std::max(1, region_count);
    int rows     = 1;
    int cols     = region_count;

    if (param_.region_rows > 0 && param_.region_cols > 0 && param_.region_rows * param_.region_cols == region_count)
    {
        rows = param_.region_rows;
        cols = param_.region_cols;
    }
    else
    {
        double best_shape_err = std::numeric_limits<double>::infinity();
        for (int r = 1; r <= region_count; ++r)
        {
            if (region_count % r != 0)
                continue;
            int    c         = region_count / r;
            double cell_w    = map_size_x_ / c;
            double cell_h    = map_size_y_ / r;
            double shape_err = std::abs(cell_w / std::max(1e-6, cell_h) - 1.0);
            if (shape_err < best_shape_err)
            {
                best_shape_err = shape_err;
                rows           = r;
                cols           = c;
            }
        }
    }

    layout_.rows          = rows;
    layout_.cols          = cols;
    layout_.total_regions = rows * cols;
    layout_.min_x         = -map_size_x_ / 2.0;
    layout_.max_x         = map_size_x_ / 2.0;
    layout_.min_y         = -map_size_y_ / 2.0;
    layout_.max_y         = map_size_y_ / 2.0;
    layout_.width         = map_size_x_ / cols;
    layout_.height        = map_size_y_ / rows;
}

void SnnRegionAllocator::reconfigureDynamic(int total_regions, double min_x, double max_x, double min_y, double max_y)
{
    total_drones_ = std::max(1, total_regions);

    if (!(max_x > min_x))
    {
        min_x = -map_size_x_ * 0.5;
        max_x = map_size_x_ * 0.5;
    }
    if (!(max_y > min_y))
    {
        min_y = -map_size_y_ * 0.5;
        max_y = map_size_y_ * 0.5;
    }

    const double span_x = std::max(1e-3, max_x - min_x);
    const double span_y = std::max(1e-3, max_y - min_y);
    map_size_x_         = span_x;
    map_size_y_         = span_y;

    buildLayoutForRegions(total_drones_);
    layout_.min_x  = min_x;
    layout_.max_x  = max_x;
    layout_.min_y  = min_y;
    layout_.max_y  = max_y;
    layout_.width  = (layout_.max_x - layout_.min_x) / std::max(1, layout_.cols);
    layout_.height = (layout_.max_y - layout_.min_y) / std::max(1, layout_.rows);

    for (auto& kv : sticky_assignment_)
        kv.second = clampRegion(kv.second);
}

int SnnRegionAllocator::clampRegion(int region_id) const
{
    return std::max(0, std::min(region_id, layout_.total_regions - 1));
}

int SnnRegionAllocator::fixedRegionOfDrone(int drone_id) const
{
    return clampRegion((drone_id - 1) % std::max(1, layout_.total_regions));
}

std::vector<int> SnnRegionAllocator::assignDroneRegions(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions)
{
    const std::vector<int> result = computeDroneRegions(drone_ids, positions);
    for (size_t i = 0; i < result.size(); ++i)
    {
        const int reg                    = clampRegion(result[i]);
        sticky_assignment_[drone_ids[i]] = reg;
        region_use_count_[reg] += 1;
    }
    return result;
}

std::vector<int> SnnRegionAllocator::computeDroneRegions(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions) const
{
    const int        n = std::min<int>(drone_ids.size(), positions.size());
    std::vector<int> result(n, -1);
    if (n == 0)
        return result;

    if (!param_.use_snn_region_assignment)
    {
        for (int i = 0; i < n; ++i)
            result[i] = fixedRegionOfDrone(drone_ids[i]);
        return result;
    }

    std::vector<double> load(layout_.total_regions, 0.0);
    // Load is current occupancy reconstructed from the latest sticky snapshot,
    // never the cumulative number of historical allocation calls.
    for (const auto& kv : sticky_assignment_)
        load[clampRegion(kv.second)] += 1.0;

    // --- Capacity-normalized load balancing ---
    // Regions with fewer active grids (inactive zones, explored out) receive
    // a proportionally higher effective load to push drones toward regions
    // with more remaining work.  When active_grid_counts_ is empty or
    // uniform, effective load == load (no-op).
    const int n_regions  = layout_.total_regions;
    double    avg_active = 0.0;
    int       valid_cnt  = 0;
    if (static_cast<int>(active_grid_counts_.size()) >= n_regions)
    {
        for (int r = 0; r < n_regions; ++r)
        {
            if (active_grid_counts_[r] > 0)
            {
                avg_active += static_cast<double>(active_grid_counts_[r]);
                ++valid_cnt;
            }
        }
        if (valid_cnt > 0)
            avg_active /= static_cast<double>(valid_cnt);
    }
    auto capRatio = [&](int r) -> double {
        if (valid_cnt == 0 || active_grid_counts_.empty())
            return 1.0;
        const int c = std::max(0, r < n_regions ? active_grid_counts_[r] : 0);
        return std::max(0.1, static_cast<double>(c) / std::max(1.0, avg_active));
    };
    auto regionHasWork = [&](int r) -> bool {
        if (active_grid_counts_.empty() || r < 0 || r >= static_cast<int>(active_grid_counts_.size()))
            return true;
        if (active_grid_counts_[r] <= 0)
            return false;
        if (reachable_grid_counts_.size() >= static_cast<size_t>(n_regions) && reachable_grid_counts_[r] <= 0)
            return false;
        return true;
    };
    bool any_work_region = false;
    for (int r = 0; r < n_regions; ++r)
        any_work_region = any_work_region || regionHasWork(r);

    // -------------------- SNN Lateral Inhibition --------------------
    // 无人机 i 占领 region r 后，其他无人机对 r 的膜电位 × 抑制系数
    constexpr double kLateralInhibitFactor = 0.2;
    // ★ 修复：增强近车的侧向抑制。
    //    当另一无人机在物理上离本机很近时（< 15m），其在 SNN 中的竞争优先度更低，
    //    表现为对其他车抢占同一 region 的抑制更强。
    //    计算所有 pairwise 距离以确定近车对
    std::vector<double> peer_proximity(n, 1.0);  // 1.0 = normal, >1.0 = nearby peer's region should be avoided
    {
        constexpr double kProxRadius   = 15.0;  // 15m 内视为近车
        constexpr double kProxStrength = 2.0;   // 近车对目标 region 的额外排斥强度
        for (int i = 0; i < n; ++i)
        {
            double min_peer_dist = std::numeric_limits<double>::infinity();
            for (int j = 0; j < n; ++j)
            {
                if (i == j)
                    continue;
                min_peer_dist = std::min(min_peer_dist, (positions[i].head<2>() - positions[j].head<2>()).norm());
            }
            if (min_peer_dist < kProxRadius)
                peer_proximity[i] = kProxStrength;
        }
    }
    // -----------------------------------------------------------------

    std::vector<bool>                used(layout_.total_regions, false);
    std::vector<std::vector<double>> v_state(n, std::vector<double>(layout_.total_regions, 0.0));

    // ★ 空区吸引力：有活跃栅格但没有无人机在其中的 region 获得膜电位奖励。
    //    奖励阈值一半以下（0.10 < 0.26），确保仅在有近位置驱动时触发，避免振荡。
    std::vector<double> vacancy_bonus(layout_.total_regions, 0.0);
    {
        std::vector<int> drone_region_occupancy(layout_.total_regions, 0);
        for (int i = 0; i < n; ++i)
        {
            const int reg = ownerRegionOfPoint(positions[i]);
            if (reg >= 0 && reg < layout_.total_regions)
                drone_region_occupancy[reg]++;
        }
        for (int r = 0; r < layout_.total_regions; ++r)
        {
            if (drone_region_occupancy[r] > 0)
                continue;
            if (!active_grid_counts_.empty() && r < static_cast<int>(active_grid_counts_.size()) && active_grid_counts_[r] > 0)
                vacancy_bonus[r] = 0.10;
        }
    }
    // -----------------------------------------------------------------
    for (int i = 0; i < n; ++i)
    {
        const auto it = sticky_assignment_.find(drone_ids[i]);
        if (it != sticky_assignment_.end() && regionHasWork(clampRegion(it->second)))
            // Keep a modest hysteresis bonus, but do not pin a drone near the
            // firing threshold when another region has clearly better work.
            v_state[i][clampRegion(it->second)] = param_.threshold * std::max(0.0, std::min(1.0, param_.sticky_ratio));
    }

    for (int iter = 0; iter < std::max(1, param_.max_iter); ++iter)
    {
        bool all_assigned = true;
        for (int i = 0; i < n; ++i)
        {
            if (result[i] != -1)
                continue;
            all_assigned = false;

            for (int r = 0; r < layout_.total_regions; ++r)
            {
                if (used[r] || (any_work_region && !regionHasWork(r)))
                    continue;

                double min_x, max_x, min_y, max_y;
                getRegionBounds(r, min_x, max_x, min_y, max_y);
                Eigen::Vector2d center((min_x + max_x) * 0.5, (min_y + max_y) * 0.5);
                const double    d              = (positions[i].head<2>() - center).norm();
                const double    distance_scale = std::max(1.0, 0.5 * std::hypot(layout_.width, layout_.height));
                double          distance_score = std::exp(-d / distance_scale);

                // ★ 修复：近车增强排斥 — 如果本机靠近其他无人机，
                //    降低 peer 所在 region 的吸引力，避免扎堆后 SNN 还把车分到一起
                if (peer_proximity[i] > 1.0)
                {
                    for (int j = 0; j < n; ++j)
                    {
                        if (i == j)
                            continue;
                        const double peer_d = (positions[j].head<2>() - center).norm();
                        if (peer_d < 20.0)
                        {
                            // Region near a close peer → penalize distance_score
                            distance_score *= 0.3;
                            break;
                        }
                    }
                }

                // 容量归一化负载：活跃栅格少的 region，有效负载按比例放大
                const double norm_load       = load[r] / capRatio(r);
                const double dyn_load_weight = param_.load_weight * (1.0 + 0.5 * (1.0 - std::exp(-norm_load)));
                double       current         = param_.distance_weight * distance_score - dyn_load_weight * norm_load;
                // ★ 修复：空区吸引力 — 无车的 region 获得额外膜电位提升
                //    奖励大小：vacancy_bonus 已在循环外计算（基于 region 是否有车和活跃栅格）
                current += vacancy_bonus[r];
                v_state[i][r] += param_.dt * (-v_state[i][r] / std::max(1e-3, param_.tau) + current);
            }

            int    best_region = -1;
            double best_val    = -std::numeric_limits<double>::infinity();
            for (int r = 0; r < layout_.total_regions; ++r)
            {
                if (used[r] || (any_work_region && !regionHasWork(r)))
                    continue;
                if (v_state[i][r] > best_val)
                {
                    best_val    = v_state[i][r];
                    best_region = r;
                }
            }

            if (best_region != -1 && best_val >= param_.threshold)
            {
                result[i]         = best_region;
                used[best_region] = true;
                load[best_region] += 1.0;

                // ★ 侧向抑制 (Lateral Inhibition)：
                //    drone i 占领了 region R，其他 drone 对 R 的膜电位永久衰减
                const int chosen_r = best_region;
                for (int other = 0; other < n; ++other)
                {
                    if (other == i)
                        continue;
                    v_state[other][chosen_r] *= kLateralInhibitFactor;
                }
            }
        }

        if (all_assigned)
            break;
    }

    for (int i = 0; i < n; ++i)
    {
        if (result[i] != -1)
            continue;
        int    best_region = -1;
        double best_cost   = std::numeric_limits<double>::infinity();
        for (int r = 0; r < layout_.total_regions; ++r)
        {
            if (used[r])
                continue;
            double min_x, max_x, min_y, max_y;
            getRegionBounds(r, min_x, max_x, min_y, max_y);
            Eigen::Vector2d center((min_x + max_x) * 0.5, (min_y + max_y) * 0.5);
            const double    d = (positions[i].head<2>() - center).norm();
            if (d < best_cost)
            {
                best_cost   = d;
                best_region = r;
            }
        }
        if (best_region == -1)
            best_region = fixedRegionOfDrone(drone_ids[i]);
        result[i]         = best_region;
        used[best_region] = true;
    }

    for (int i = 0; i < n; ++i)
        result[i] = clampRegion(result[i]);

    static std::string last_assign_sig;
    std::ostringstream oss;
    oss << "use_snn=" << (param_.use_snn_region_assignment ? 1 : 0);
    for (int i = 0; i < n; ++i)
        oss << " d" << drone_ids[i] << "->r" << result[i];
    // 打印最终 assignment 的负载（而非 sticky 快照与 SNN 阶段增量的混合值，避免误导诊断）
    std::vector<double> final_load(layout_.total_regions, 0.0);
    for (int i = 0; i < n; ++i)
        final_load[clampRegion(result[i])] += 1.0;
    oss << " | loads=";
    for (int r = 0; r < layout_.total_regions; ++r)
        oss << final_load[r] << (r + 1 < layout_.total_regions ? "," : "");
    const std::string sig = oss.str();
    RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[SnnRegion] %s", sig.c_str());
    if (sig != last_assign_sig)
    {
        RCLCPP_INFO(rclcpp::get_logger("exploration_manager"), "[SnnRegion] assignment CHANGED: %s", sig.c_str());
        last_assign_sig = sig;
    }

    return result;
}

void SnnRegionAllocator::getRegionBounds(int region_id, double& min_x, double& max_x, double& min_y, double& max_y) const
{
    const int reg = clampRegion(region_id);
    const int row = reg / layout_.cols;
    const int col = reg % layout_.cols;
    min_x         = layout_.min_x + col * layout_.width;
    max_x         = min_x + layout_.width;
    min_y         = layout_.min_y + row * layout_.height;
    max_y         = min_y + layout_.height;
}

int SnnRegionAllocator::ownerRegionOfPoint(const Eigen::Vector3d& point) const
{
    int col = static_cast<int>(std::floor((point.x() - layout_.min_x) / std::max(1e-6, layout_.width)));
    int row = static_cast<int>(std::floor((point.y() - layout_.min_y) / std::max(1e-6, layout_.height)));
    col     = std::max(0, std::min(col, layout_.cols - 1));
    row     = std::max(0, std::min(row, layout_.rows - 1));
    return clampRegion(row * layout_.cols + col);
}

int SnnRegionAllocator::ownerRegionOfGridCenter(const Eigen::Vector3d& center) const
{
    return ownerRegionOfPoint(center);
}

bool SnnRegionAllocator::getStickyRegionForDrone(int drone_id, int& region_id) const
{
    const auto it = sticky_assignment_.find(drone_id);
    if (it == sticky_assignment_.end())
        return false;
    region_id = clampRegion(it->second);
    return true;
}

void SnnRegionAllocator::setStickyAssignment(int drone_id, int region_id)
{
    sticky_assignment_[drone_id] = clampRegion(region_id);
}

void SnnRegionAllocator::clearSticky()
{
    sticky_assignment_.clear();
    region_use_count_.clear();
}

void SnnRegionAllocator::setRegionActiveGridCounts(const std::vector<int>& counts)
{
    active_grid_counts_ = counts;
}

void SnnRegionAllocator::setRegionReachableGridCounts(const std::vector<int>& counts)
{
    reachable_grid_counts_ = counts;
}

void SnnRegionAllocator::applyConsensusAssignments(const std::vector<int>& assignments)
{
    sticky_assignment_.clear();
    region_use_count_.clear();
    for (size_t i = 0; i < assignments.size(); ++i)
    {
        if (assignments[i] < 0)
            continue;  // No assignment for this drone yet.
        const int drone_id           = static_cast<int>(i) + 1;
        const int region             = clampRegion(assignments[i]);
        sticky_assignment_[drone_id] = region;
        region_use_count_[region] += 1;
    }
}

}  // namespace fast_planner