#ifndef _STALL_ESCAPE_H_
#define _STALL_ESCAPE_H_

#include <Eigen/Eigen>

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace fast_planner
{
// 逃逸选点的纯逻辑：所有与地图/队友相关的判断通过回调注入，便于单测。
struct StallEscapeContext
{
    Eigen::Vector3d self_pos{ Eigen::Vector3d::Zero() };
    double          min_dist_m{ 1.0 };
    double          max_dist_m{ 15.0 };

    bool         has_last_failed{ false };
    std::int64_t last_failed_key{ 0 };
    int          self_region{ -1 };

    std::function<bool(const Eigen::Vector3d&)>         reachable;      // 可达（与 SNN plan 同源）
    std::function<int(const Eigen::Vector3d&)>          region_of;      // 区域归属，-1=未知
    std::function<std::int64_t(const Eigen::Vector3d&)> key_of;         // 失败目标隔离 key
    std::function<double(const Eigen::Vector3d&)>       extra_penalty;  // 队友覆盖/邻近软代价
};

// 硬过滤：距离区间、可达、非上次失败格、同区域；
// 打分：score = -distance - extra_penalty；同分时距离小者优先，再按 x、y 字典序取小，保证确定性。
inline bool SelectStallEscapeCandidate(const std::vector<Eigen::Vector3d>& candidates, const StallEscapeContext& ctx, Eigen::Vector3d& best)
{
    bool   found      = false;
    double best_score = -std::numeric_limits<double>::infinity();
    double best_dist  = 0.0;

    for (size_t i = 0; i < candidates.size(); ++i)
    {
        const Eigen::Vector3d& c = candidates[i];
        const double           d = (c - ctx.self_pos).head<2>().norm();
        if (d < ctx.min_dist_m || d > ctx.max_dist_m)
            continue;
        if (!ctx.reachable || !ctx.reachable(c))
            continue;
        if (ctx.has_last_failed && ctx.key_of && ctx.key_of(c) == ctx.last_failed_key)
            continue;
        if (ctx.self_region >= 0 && ctx.region_of && ctx.region_of(c) != ctx.self_region)
            continue;

        const double score  = -d - (ctx.extra_penalty ? ctx.extra_penalty(c) : 0.0);
        bool         better = false;
        if (!found)
        {
            better = true;
        }
        else if (score > best_score + 1e-9)
        {
            better = true;
        }
        else if (std::fabs(score - best_score) <= 1e-9)
        {
            if (d < best_dist - 1e-9)
                better = true;
            else if (std::fabs(d - best_dist) <= 1e-9 && (c.x() < best.x() - 1e-9 || (std::fabs(c.x() - best.x()) <= 1e-9 && c.y() < best.y() - 1e-9)))
                better = true;
        }

        if (better)
        {
            found      = true;
            best       = c;
            best_score = score;
            best_dist  = d;
        }
    }
    return found;
}

}  // namespace fast_planner

#endif
