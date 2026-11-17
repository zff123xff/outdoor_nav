#include "path_searching/snn_path.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr int kWavefrontPlanWarnThrottleMs = 2000;

rclcpp::Clock& WavefrontPlanWarnClock()
{
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    return clock;
}

void ThrottleWarnWavefrontPlanNoPath(const char* reason, int start_x, int start_y, int goal_x, int goal_y)
{
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("snn_path.wavefront"), WavefrontPlanWarnClock(), kWavefrontPlanWarnThrottleMs,
                         "[WavefrontPlanner] no path (%s) start_grid=(%d,%d) goal_grid=(%d,%d)", reason, start_x, start_y, goal_x, goal_y);
}
}  // namespace

namespace fast_planner
{
WavefrontPlanner::WavefrontPlanner(PlannerConfig cfg) : cfg_(cfg) {}

void WavefrontPlanner::setGrid(const Grid& grid)
{
    Grid preprocessed = grid;
    if (cfg_.obstacle_close_kernel >= 3)
        preprocessed = closeObstacleGaps(grid, cfg_.obstacle_close_kernel);

    // 在这里进行地图膨胀，引入机器人尺寸
    grid_ = inflateGrid(preprocessed, cfg_.robot_radius);
    H_    = grid_.size();
    W_    = H_ > 0 ? grid_[0].size() : 0;
    alloc_states();
}

WavefrontPlanner::Grid WavefrontPlanner::dilateObstacles(const Grid& grid, const int radius) const
{
    const int h = static_cast<int>(grid.size());
    const int w = (h > 0) ? static_cast<int>(grid[0].size()) : 0;
    Grid      out(h, std::vector<int>(w, 0));
    if (radius <= 0)
    {
        return grid;
    }

    const int r_int = radius;
    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            if (grid[i][j] != 1)
                continue;
            for (int di = -r_int; di <= r_int; ++di)
            {
                for (int dj = -r_int; dj <= r_int; ++dj)
                {
                    const int ni = i + di;
                    const int nj = j + dj;
                    if (ni >= 0 && ni < h && nj >= 0 && nj < w)
                        out[ni][nj] = 1;
                }
            }
        }
    }
    return out;
}

WavefrontPlanner::Grid WavefrontPlanner::erodeObstacles(const Grid& grid, const int radius) const
{
    const int h = static_cast<int>(grid.size());
    const int w = (h > 0) ? static_cast<int>(grid[0].size()) : 0;
    Grid      out(h, std::vector<int>(w, 0));
    if (radius <= 0)
    {
        return grid;
    }

    const int r_int = radius;
    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            if (grid[i][j] != 1)
                continue;

            bool keep = true;
            for (int di = -r_int; di <= r_int && keep; ++di)
            {
                for (int dj = -r_int; dj <= r_int; ++dj)
                {
                    const int ni = i + di;
                    const int nj = j + dj;
                    if (ni < 0 || ni >= h || nj < 0 || nj >= w || grid[ni][nj] != 1)
                    {
                        keep = false;
                        break;
                    }
                }
            }
            if (keep)
                out[i][j] = 1;
        }
    }
    return out;
}

WavefrontPlanner::Grid WavefrontPlanner::closeObstacleGaps(const Grid& grid, const int kernel_cells) const
{
    if (kernel_cells < 3)
        return grid;

    const int radius = kernel_cells / 2;
    return erodeObstacles(dilateObstacles(grid, radius), radius);
}

/**
 * @brief 膨胀算法实现
 * 将所有障碍物点向外扩充 radius 个像素
 */
WavefrontPlanner::Grid WavefrontPlanner::inflateGrid(const Grid& grid, float radius)
{
    int  h             = grid.size();
    int  w             = (h > 0) ? grid[0].size() : 0;
    Grid inflated_grid = grid;

    if (radius <= 0.1f)
        return inflated_grid;

    const float radius_sq = radius * radius;
    int         r_int     = static_cast<int>(std::ceil(radius));
    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            if (grid[i][j] != 1)
                continue;
            for (int di = -r_int; di <= r_int; ++di)
            {
                for (int dj = -r_int; dj <= r_int; ++dj)
                {
                    int ni = i + di, nj = j + dj;
                    if (ni >= 0 && ni < h && nj >= 0 && nj < w)
                    {
                        if (di * di + dj * dj <= radius_sq)
                            inflated_grid[ni][nj] = 1;
                    }
                }
            }
        }
    }
    return inflated_grid;
}

void WavefrontPlanner::alloc_states()
{
    if (H_ <= 0 || W_ <= 0)
    {
        wave_time_buf_.clear();
        parent_buf_.clear();
        return;
    }

    wave_time_buf_.assign(static_cast<size_t>(H_), std::vector<float>(static_cast<size_t>(W_), std::numeric_limits<float>::infinity()));
    parent_buf_.assign(static_cast<size_t>(H_), std::vector<Point>(static_cast<size_t>(W_), Point{ -1, -1 }));
}

bool WavefrontPlanner::in_bounds(int i, int j) const
{
    return i >= 0 && i < H_ && j >= 0 && j < W_;
}

bool WavefrontPlanner::is_valid_transition(const Point& from, const Point& to) const
{
    if (!in_bounds(to.first, to.second) || grid_[to.first][to.second] != 0)
        return false;

    const int di = to.first - from.first;
    const int dj = to.second - from.second;
    if (std::abs(di) != 1 || std::abs(dj) != 1)
        return true;

    return grid_[from.first + di][from.second] == 0 && grid_[from.first][from.second + dj] == 0;
}

std::vector<WavefrontPlanner::Point> WavefrontPlanner::find_free_candidates(const Point p) const
{
    std::vector<Point> candidates;
    if (!in_bounds(p.first, p.second))
        return candidates;
    if (grid_[p.first][p.second] == 0)
    {
        candidates.push_back(p);
        return candidates;
    }

    constexpr int kMaxRadius = 6;
    for (int r = 1; r <= kMaxRadius; ++r)
    {
        for (int dx = -r; dx <= r; ++dx)
        {
            for (int dy = -r; dy <= r; ++dy)
            {
                if (std::abs(dx) != r && std::abs(dy) != r)
                    continue;
                const Point candidate{ p.first + dx, p.second + dy };
                if (in_bounds(candidate.first, candidate.second) && grid_[candidate.first][candidate.second] == 0)
                    candidates.push_back(candidate);
            }
        }
    }
    return candidates;
}

std::vector<Eigen::Vector3i> WavefrontPlanner::plan(const Eigen::Vector3i& start_pt, const Eigen::Vector3i& goal_pt)
{
    const Point start_requested{ start_pt.x(), start_pt.y() };
    const Point goal_requested{ goal_pt.x(), goal_pt.y() };
    if (!in_bounds(start_requested.first, start_requested.second) || !in_bounds(goal_requested.first, goal_requested.second))
    {
        ThrottleWarnWavefrontPlanNoPath("out_of_bounds", start_pt.x(), start_pt.y(), goal_pt.x(), goal_pt.y());
        return {};
    }

    if (start_requested == goal_requested && grid_[start_requested.first][start_requested.second] == 0)
        return { start_pt, goal_pt };

    const std::vector<Point> start_candidates = find_free_candidates(start_requested);
    const std::vector<Point> goal_candidates  = find_free_candidates(goal_requested);
    if (start_candidates.empty() || goal_candidates.empty())
    {
        ThrottleWarnWavefrontPlanNoPath(start_candidates.empty() ? "start_occupied_no_free_neighbor" : "goal_occupied_no_free_neighbor", start_pt.x(), start_pt.y(),
                                        goal_pt.x(), goal_pt.y());
        return {};
    }

    compute_wave_time(start_candidates);
    for (const Point& goal : goal_candidates)
    {
        if (!std::isfinite(wave_time_buf_[goal.first][goal.second]))
            continue;

        std::vector<Eigen::Vector3i> reverse_path;
        Point                        curr = goal;
        while (parent_buf_[curr.first][curr.second].first >= 0)
        {
            reverse_path.emplace_back(curr.first, curr.second, 0);
            curr = parent_buf_[curr.first][curr.second];
        }
        reverse_path.emplace_back(curr.first, curr.second, 0);
        std::reverse(reverse_path.begin(), reverse_path.end());
        return reverse_path;
    }

    ThrottleWarnWavefrontPlanNoPath("unreachable", start_requested.first, start_requested.second, goal_requested.first, goal_requested.second);
    return {};
}

std::vector<std::uint8_t> WavefrontPlanner::computeReachabilityMask(const Grid& grid, const Eigen::Vector3i& start_pt)
{
    const int                 H     = static_cast<int>(grid.size());
    const int                 W     = H > 0 ? static_cast<int>(grid[0].size()) : 0;
    const size_t              total = static_cast<size_t>(H) * static_cast<size_t>(W);
    std::vector<std::uint8_t> mask(total, 0);
    if (H <= 0 || W <= 0)
        return mask;

    const auto inb = [&](int i, int j) { return i >= 0 && i < H && j >= 0 && j < W; };
    Point start{ start_pt.x(), start_pt.y() };
    if (!inb(start.first, start.second))
        return mask;
    if (grid[start.first][start.second] != 0)
    {
        Point best{ -1, -1 };
        for (int r = 1; r <= 6 && best.first < 0; ++r)
        {
            for (int dx = -r; dx <= r && best.first < 0; ++dx)
            {
                for (int dy = -r; dy <= r; ++dy)
                {
                    if ((std::abs(dx) != r && std::abs(dy) != r) || !inb(start.first + dx, start.second + dy))
                        continue;
                    if (grid[start.first + dx][start.second + dy] == 0)
                    {
                        best = { start.first + dx, start.second + dy };
                        break;
                    }
                }
            }
        }
        if (best.first < 0)
            return mask;
        start = best;
    }

    if (reach_stamp_.size() < total)
        reach_stamp_.resize(total, 0u);
    ++reach_gen_;
    if (reach_gen_ == 0u)
    {
        std::fill(reach_stamp_.begin(), reach_stamp_.end(), 0u);
        reach_gen_ = 1u;
    }

    reach_queue_.clear();
    const auto seed = [&](const Point point) {
        const size_t idx = static_cast<size_t>(point.first) * static_cast<size_t>(W) + static_cast<size_t>(point.second);
        if (reach_stamp_[idx] == reach_gen_)
            return;
        reach_stamp_[idx] = reach_gen_;
        reach_queue_.push_back(point);
        mask[idx] = 1;
    };
    seed(start);

    for (size_t head = 0; head < reach_queue_.size(); ++head)
    {
        const Point cur = reach_queue_[head];
        for (const Point& dir : DIRS)
        {
            const Point next{ cur.first + dir.first, cur.second + dir.second };
            if (!inb(next.first, next.second) || grid[next.first][next.second] != 0)
                continue;
            if (std::abs(dir.first) == 1 && std::abs(dir.second) == 1 &&
                (grid[cur.first + dir.first][cur.second] != 0 || grid[cur.first][cur.second + dir.second] != 0))
                continue;
            seed(next);
        }
    }
    return mask;
}

std::vector<std::uint8_t> WavefrontPlanner::computeReachabilityMask(const Eigen::Vector3i& start_pt)
{
    const size_t total = static_cast<size_t>(H_) * static_cast<size_t>(W_);
    std::vector<std::uint8_t> mask(total, 0);
    if (H_ <= 0 || W_ <= 0)
        return mask;

    const std::vector<Point> starts = find_free_candidates({ start_pt.x(), start_pt.y() });
    if (starts.empty())
        return mask;

    if (reach_stamp_.size() < total)
        reach_stamp_.resize(total, 0u);
    ++reach_gen_;
    if (reach_gen_ == 0u)
    {
        std::fill(reach_stamp_.begin(), reach_stamp_.end(), 0u);
        reach_gen_ = 1u;
    }

    reach_queue_.clear();
    const auto seed = [&](const Point point) {
        const size_t idx = static_cast<size_t>(point.first) * static_cast<size_t>(W_) + static_cast<size_t>(point.second);
        if (reach_stamp_[idx] == reach_gen_)
            return;
        reach_stamp_[idx] = reach_gen_;
        reach_queue_.push_back(point);
        mask[idx] = 1;
    };
    for (const Point& start : starts)
        seed(start);

    for (size_t head = 0; head < reach_queue_.size(); ++head)
    {
        const Point cur = reach_queue_[head];
        for (const Point& dir : DIRS)
        {
            const Point next{ cur.first + dir.first, cur.second + dir.second };
            if (is_valid_transition(cur, next))
                seed(next);
        }
    }
    return mask;
}

const std::vector<std::vector<float>>& WavefrontPlanner::compute_wave_time(const std::vector<Point>& starts)
{
    const float inf = std::numeric_limits<float>::infinity();
    for (int i = 0; i < H_; ++i)
    {
        std::fill(wave_time_buf_[i].begin(), wave_time_buf_[i].end(), inf);
        std::fill(parent_buf_[i].begin(), parent_buf_[i].end(), Point{ -1, -1 });
    }

    std::deque<Point> queue;
    for (const Point& start : starts)
    {
        if (std::isfinite(wave_time_buf_[start.first][start.second]))
            continue;
        wave_time_buf_[start.first][start.second] = 0.0f;
        queue.push_back(start);
    }

    const int grid_cells = H_ > 0 && W_ > 0 ? H_ * W_ : 0;
    const int visit_limit = std::max(1, std::min(cfg_.max_wave_steps, grid_cells));
    int       visited     = 0;
    for (size_t head = 0; head < queue.size() && visited < visit_limit; ++head)
    {
        const Point cur = queue[head];
        ++visited;
        for (const Point& dir : DIRS)
        {
            const Point next{ cur.first + dir.first, cur.second + dir.second };
            if (!is_valid_transition(cur, next) || std::isfinite(wave_time_buf_[next.first][next.second]))
                continue;
            wave_time_buf_[next.first][next.second] = wave_time_buf_[cur.first][cur.second] + 1.0f;
            parent_buf_[next.first][next.second]    = cur;
            queue.push_back(next);
        }
    }
    return wave_time_buf_;
}

}  // namespace fast_planner
