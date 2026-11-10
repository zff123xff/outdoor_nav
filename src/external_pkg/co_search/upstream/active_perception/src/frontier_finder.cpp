#include <active_perception/frontier_finder.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
// #include <path_searching/astar2.h>

#include <active_perception/graph_node.h>
#include <active_perception/perception_utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/edt_environment.h>
#include <plan_env/multi_map_manager.h>

// use PCL region growing segmentation
// #include <pcl/point_types.h>
// #include <pcl/search/search.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Eigenvalues>

namespace fast_planner
{
namespace
{
    // Region-growing frontier clusters often reach hundreds–thousands of cells; reserve cuts reallocations.
    constexpr std::size_t kExpandedCellsReserveHint = 2048;
}  // namespace
FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr& edt, rclcpp::Node::SharedPtr node)
{
    node_          = node;
    this->edt_env_ = edt;
    int voxel_num  = edt->sdf_map_->getVoxelNum();
    frontier_flag_ = vector<char>(voxel_num, 0);
    fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

    cluster_min_               = node->has_parameter("frontier/cluster_min") ? node->get_parameter("frontier/cluster_min").as_int()
                                                                             : node->declare_parameter<int>("frontier/cluster_min", 100);
    cluster_size_xy_           = node->has_parameter("frontier/cluster_size_xy") ? node->get_parameter("frontier/cluster_size_xy").as_double()
                                                                                 : node->declare_parameter<double>("frontier/cluster_size_xy", 2.0);
    cluster_size_z_            = node->has_parameter("frontier/cluster_size_z") ? node->get_parameter("frontier/cluster_size_z").as_double()
                                                                                : node->declare_parameter<double>("frontier/cluster_size_z", 0.8);
    min_candidate_dist_        = node->has_parameter("frontier/min_candidate_dist") ? node->get_parameter("frontier/min_candidate_dist").as_double()
                                                                                    : node->declare_parameter<double>("frontier/min_candidate_dist", 0.5);
    min_candidate_clearance_   = node->has_parameter("frontier/min_candidate_clearance")
                                     ? node->get_parameter("frontier/min_candidate_clearance").as_double()
                                     : node->declare_parameter<double>("frontier/min_candidate_clearance", 0.21);
    candidate_dphi_            = node->has_parameter("frontier/candidate_dphi") ? node->get_parameter("frontier/candidate_dphi").as_double()
                                                                                : node->declare_parameter<double>("frontier/candidate_dphi", 0.2618);
    candidate_rmax_            = node->has_parameter("frontier/candidate_rmax") ? node->get_parameter("frontier/candidate_rmax").as_double()
                                                                                : node->declare_parameter<double>("frontier/candidate_rmax", 1.5);
    candidate_rmin_            = node->has_parameter("frontier/candidate_rmin") ? node->get_parameter("frontier/candidate_rmin").as_double()
                                                                                : node->declare_parameter<double>("frontier/candidate_rmin", 1.0);
    candidate_rnum_            = node->has_parameter("frontier/candidate_rnum") ? node->get_parameter("frontier/candidate_rnum").as_int()
                                                                                : node->declare_parameter<int>("frontier/candidate_rnum", 3);
    max_candidate_samples_      = node->has_parameter("frontier/max_candidate_samples")
                                     ? node->get_parameter("frontier/max_candidate_samples").as_int()
                                     : node->declare_parameter<int>("frontier/max_candidate_samples", 24);
    max_candidate_samples_      = std::max(1, max_candidate_samples_);
    down_sample_               = node->has_parameter("frontier/down_sample") ? node->get_parameter("frontier/down_sample").as_int()
                                                                             : node->declare_parameter<int>("frontier/down_sample", 3);
    min_visib_num_             = node->has_parameter("frontier/min_visib_num") ? node->get_parameter("frontier/min_visib_num").as_int()
                                                                               : node->declare_parameter<int>("frontier/min_visib_num", 10);
    min_view_finish_fraction_  = node->has_parameter("frontier/min_view_finish_fraction")
                                     ? node->get_parameter("frontier/min_view_finish_fraction").as_double()
                                     : node->declare_parameter<double>("frontier/min_view_finish_fraction", 0.2);
    revalidate_half_extent_xy_ = node->has_parameter("frontier/revalidate_half_extent_xy")
                                     ? node->get_parameter("frontier/revalidate_half_extent_xy").as_double()
                                     : node->declare_parameter<double>("frontier/revalidate_half_extent_xy", 20.0);
    revalidate_half_extent_z_  = node->has_parameter("frontier/revalidate_half_extent_z")
                                     ? node->get_parameter("frontier/revalidate_half_extent_z").as_double()
                                     : node->declare_parameter<double>("frontier/revalidate_half_extent_z", 1.0);

    // 历史轨迹抑制参数：防止动态足迹清除后的行驶路径被误判为 frontier
    path_frontier_suppress_enable_ = node->has_parameter("frontier/path_frontier_suppress_enable")
                                         ? node->get_parameter("frontier/path_frontier_suppress_enable").as_bool()
                                         : node->declare_parameter<bool>("frontier/path_frontier_suppress_enable", true);
    path_suppress_radius_          = node->has_parameter("frontier/path_suppress_radius") ? node->get_parameter("frontier/path_suppress_radius").as_double()
                                                                                          : node->declare_parameter<double>("frontier/path_suppress_radius", 0.8);
    path_history_length_           = node->has_parameter("frontier/path_history_length") ? node->get_parameter("frontier/path_history_length").as_double()
                                                                                         : node->declare_parameter<double>("frontier/path_history_length", 10.0);
    path_history_sample_dist_      = node->has_parameter("frontier/path_history_sample_dist")
                                         ? node->get_parameter("frontier/path_history_sample_dist").as_double()
                                         : node->declare_parameter<double>("frontier/path_history_sample_dist", 0.1);

    raycaster_.reset(new RayCaster);
    resolution_ = edt_env_->sdf_map_->getResolution();
    Eigen::Vector3d origin, size;
    edt_env_->sdf_map_->getRegion(origin, size);
    raycaster_->setParams(resolution_, origin);

    percep_utils_.reset(new PerceptionUtils(node));
}

FrontierFinder::~FrontierFinder() {}

/**
 * @brief 搜索SDF地图中的新边界，更新活动边界frontiers_，更新休眠边界dormant_frontiers_
 */
void FrontierFinder::searchFrontiers()
{
    const auto t1 = node_->now();
    tmp_frontiers_.clear();

    // 获取SDF地图中更新的区域边界框
    Vector3d update_min, update_max;
    edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, false);

    // 获取其他飞机探索到的边界框列表
    vector<Eigen::Vector3d> chunk_mins, chunk_maxs;
    edt_env_->sdf_map_->mm_->getChunkBoxes(chunk_mins, chunk_maxs, false);

    // 合并更新区域和外部块的边界框到一个列表
    vector<Eigen::Vector3d> mins, maxs;
    mins.push_back(update_min);
    mins.insert(mins.end(), chunk_mins.begin(), chunk_mins.end());
    maxs.push_back(update_max);
    maxs.insert(maxs.end(), chunk_maxs.begin(), chunk_maxs.end());

    // Removed changed frontiers in updated map
    auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
        Eigen::Vector3i idx;
        for (auto cell : iter->cells_)
        {
            edt_env_->sdf_map_->posToIndex(cell, idx);
            frontier_flag_[toadr(idx)] = 0;
        }
        iter = frontiers.erase(iter);
    };

    // std::cout << "Before remove: " << frontiers_.size() << std::endl;

    removed_ids_.clear();
    int rmv_idx = 0;
    for (auto iter = frontiers_.begin(); iter != frontiers_.end();)
    {
        // 如果边界与更新区域或外部块有重叠，并且边界状态已改变
        if (haveAnyOverlap(iter->box_min_, iter->box_max_, mins, maxs) && isFrontierChanged(*iter))
        {
            resetFlag(iter, frontiers_);  //重置标志并移除边界
            removed_ids_.push_back(rmv_idx);
        }
        else
        {
            ++rmv_idx;
            ++iter;
        }
    }
    // std::cout << "After remove: " << frontiers_.size() << std::endl;
    for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();)
    {
        if (haveAnyOverlap(iter->box_min_, iter->box_max_, mins, maxs) && isFrontierChanged(*iter))
            resetFlag(iter, dormant_frontiers_);
        else
            ++iter;
    }

    // 获取 SDF 地图的整体边界框
    Vector3d box_min, box_max;
    edt_env_->sdf_map_->getBox(box_min, box_max);

    //定义搜索区域：将更新区域和外部块边界框稍微膨胀
    vector<Eigen::Vector3d> search_mins, search_maxs;
    for (int i = 0; i < mins.size(); ++i)
    {
        search_mins.push_back(mins[i] - Vector3d(1, 1, 0.2));
        search_maxs.push_back(maxs[i] + Vector3d(1, 1, 0.2));

        // 限制范围在sdfmap范围内
        for (int k = 0; k < 3; ++k)
        {
            search_mins[i][k] = max(search_mins[i][k], box_min[k]);
            search_maxs[i][k] = min(search_maxs[i][k], box_max[k]);
        }
    }

    // 将搜索区域的边界框转换为体素索引
    vector<Eigen::Vector3i> min_ids(mins.size()), max_ids(mins.size());
    for (int i = 0; i < mins.size(); ++i)
    {
        edt_env_->sdf_map_->posToIndex(search_mins[i], min_ids[i]);
        edt_env_->sdf_map_->posToIndex(search_maxs[i], max_ids[i]);
    }

    // 在搜索区域内扫描新的边界种子
    for (int i = 0; i < min_ids.size(); ++i)
    {
        auto min_id = min_ids[i];
        auto max_id = max_ids[i];

        for (int z = min_id(2); z <= max_id(2); ++z)
            for (int x = min_id(0); x <= max_id(0); ++x)
                for (int y = min_id(1); y <= max_id(1); ++y)
                {
                    Eigen::Vector3i cur(x, y, z);
                    // 检查条件：
                    // 1. 未标记为边界 (frontier_flag_ == 0)
                    // 2. 当前体素是已知的空闲区域 (knownfree)
                    // 3. 邻居有未知区域 (isNeighborUnknown)

                    if (frontier_flag_[toadr(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur))
                    {
                        // 从种子单元扩展，生成完整的边界簇
                        expandFrontier(cur);
                    }
                }
    }

    splitLargeFrontiers(tmp_frontiers_);
}

void FrontierFinder::searchFrontiers(const Eigen::Vector3d& ugv_pos)
{
    // --- 1. 基础指针与地图状态检查 ---
    if (!edt_env_ || !edt_env_->sdf_map_)
        return;

    int voxels_num = edt_env_->sdf_map_->getVoxelNum();
    if (voxels_num <= 0)
        return;

    // 关键修复：确保标志位数组大小与当前地图规格绝对一致
    if (frontier_flag_.size() != (size_t)voxels_num)
    {
        frontier_flag_.assign(voxels_num, 0);
    }

    // 更新行驶轨迹，用于抑制动态足迹清除后的路径 frontier
    updatePathHistory(ugv_pos);

    const auto t1 = node_->now();
    tmp_frontiers_.clear();

    // --- 2. 获取更新区域边界框 ---
    Vector3d update_min, update_max;
    edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, false);

    vector<Vector3d> chunk_mins, chunk_maxs;
    if (edt_env_->sdf_map_->mm_)
    {
        edt_env_->sdf_map_->mm_->getChunkBoxes(chunk_mins, chunk_maxs, false);
    }

    vector<Vector3d> mins, maxs;
    mins.push_back(update_min);
    mins.insert(mins.end(), chunk_mins.begin(), chunk_mins.end());
    maxs.push_back(update_max);
    maxs.insert(maxs.end(), chunk_maxs.begin(), chunk_maxs.end());

    // Always re-validate a box around vehicle to aggressively clean stale frontiers
    // when local occupancy updates lag behind global update-box propagation.
    mins.push_back(ugv_pos - Vector3d(revalidate_half_extent_xy_, revalidate_half_extent_xy_, revalidate_half_extent_z_));
    maxs.push_back(ugv_pos + Vector3d(revalidate_half_extent_xy_, revalidate_half_extent_xy_, revalidate_half_extent_z_));

    // --- 3. 重置标志位并移除旧边界 (修复 void 返回值报错) ---
    auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
        Eigen::Vector3i idx;
        for (const auto& cell : iter->cells_)
        {
            // 先检查物理坐标是否在地图内，再进行索引转换
            if (edt_env_->sdf_map_->isInBox(cell))
            {
                edt_env_->sdf_map_->posToIndex(cell, idx);
                int adr = toadr(idx);
                // 严格检查数组边界
                if (adr >= 0 && adr < (int)frontier_flag_.size())
                {
                    frontier_flag_[adr] = 0;
                }
            }
        }
        iter = frontiers.erase(iter);
    };

    removed_ids_.clear();
    int rmv_idx = 0;
    for (auto iter = frontiers_.begin(); iter != frontiers_.end();)
    {
        if (haveAnyOverlap(iter->box_min_, iter->box_max_, mins, maxs) && isFrontierChanged(*iter))
        {
            resetFlag(iter, frontiers_);
            removed_ids_.push_back(rmv_idx);
        }
        else
        {
            ++rmv_idx;
            ++iter;
        }
    }

    for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();)
    {
        if (haveAnyOverlap(iter->box_min_, iter->box_max_, mins, maxs) && isFrontierChanged(*iter))
            resetFlag(iter, dormant_frontiers_);
        else
            ++iter;
    }

    // --- 4. 扫描搜索区域内的边界种子 ---
    Vector3d box_min, box_max;
    edt_env_->sdf_map_->getBox(box_min, box_max);

    for (int i = 0; i < (int)mins.size(); ++i)
    {
        Vector3d s_min = mins[i] - Vector3d(1, 1, 0.2);
        Vector3d s_max = maxs[i] + Vector3d(1, 1, 0.2);

        // 限制在 SDF 地图全局边界内
        for (int k = 0; k < 3; ++k)
        {
            s_min[k] = max(s_min[k], box_min[k]);
            s_max[k] = min(s_max[k], box_max[k]);
        }

        Eigen::Vector3i min_id, max_id;
        edt_env_->sdf_map_->posToIndex(s_min, min_id);
        edt_env_->sdf_map_->posToIndex(s_max, max_id);

        for (int z = min_id(2); z <= max_id(2); ++z)
            for (int x = min_id(0); x <= max_id(0); ++x)
                for (int y = min_id(1); y <= max_id(1); ++y)
                {
                    Eigen::Vector3i cur(x, y, z);

                    // 索引检查：isInBox(cur) 是关键，防止 toadr 计算出越界地址
                    if (edt_env_->sdf_map_->isInBox(cur))
                    {
                        int adr = toadr(cur);
                        if (adr >= 0 && adr < (int)frontier_flag_.size() && frontier_flag_[adr] == 0 && knownfree(cur) && isNeighborUnknown(cur))
                        {
                            expandFrontier(cur, ugv_pos);
                        }
                    }
                }
    }
    splitLargeFrontiers(tmp_frontiers_);
}

void FrontierFinder::expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */)
{
    // Data for clustering
    queue<Eigen::Vector3i>  cell_queue;
    vector<Eigen::Vector3d> expanded;
    Vector3d                pos;

    expanded.reserve(kExpandedCellsReserveHint);
    edt_env_->sdf_map_->indexToPos(first, pos);
    expanded.push_back(pos);
    cell_queue.push(first);
    frontier_flag_[toadr(first)] = 1;

    // Search frontier cluster based on region growing (distance clustering)
    while (!cell_queue.empty())
    {
        auto cur = cell_queue.front();
        cell_queue.pop();
        auto nbrs = allNeighbors(cur);
        for (auto nbr : nbrs)
        {
            // Qualified cell should be inside bounding box and frontier cell not clustered
            int adr = toadr(nbr);
            if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) || !(knownfree(nbr) && isNeighborUnknown(nbr)))
                continue;

            edt_env_->sdf_map_->indexToPos(nbr, pos);

            if (pos[2] < 0.2)
                continue;

            expanded.push_back(pos);
            cell_queue.push(nbr);
            frontier_flag_[adr] = 1;
        }
    }
    if (expanded.size() > cluster_min_)
    {
        // Compute detailed info
        Frontier frontier;
        frontier.cells_ = std::move(expanded);
        computeFrontierInfo(frontier);
        tmp_frontiers_.push_back(std::move(frontier));
    }
}

void FrontierFinder::expandFrontier(const Eigen::Vector3i& first, const Eigen::Vector3d& ugv_pos)
{
    // 种子若位于动态足迹清除区，直接丢弃：清除操作在已知/未知交界制造的人工前沿不应成簇
    if (edt_env_->sdf_map_->isFootprintCleared(first))
        return;

    queue<Eigen::Vector3i> cell_queue;
    vector<Vector3d>       expanded;
    Vector3d               pos;

    expanded.reserve(kExpandedCellsReserveHint);
    edt_env_->sdf_map_->indexToPos(first, pos);
    expanded.push_back(pos);
    cell_queue.push(first);

    int first_adr = toadr(first);
    if (first_adr >= 0 && first_adr < (int)frontier_flag_.size())
        frontier_flag_[first_adr] = 1;

    while (!cell_queue.empty())
    {
        auto cur = cell_queue.front();
        cell_queue.pop();
        auto nbrs = allNeighbors(cur);

        for (const auto& nbr : nbrs)
        {
            // 必须先检查网格索引是否在地图盒子里
            if (!edt_env_->sdf_map_->isInBox(nbr))
                continue;

            int adr = toadr(nbr);
            // 数组越界拦截
            if (adr < 0 || adr >= (int)frontier_flag_.size() || frontier_flag_[adr] == 1)
                continue;

            if (knownfree(nbr) && isNeighborUnknown(nbr))
            {
                edt_env_->sdf_map_->indexToPos(nbr, pos);

                // 过滤地面、靠近车体的点、历史行驶路径附近的点，以及被足迹清除过的体素（防止路径 frontier）
                if (pos[2] < 0.2 || (pos - ugv_pos).norm() < 0.5 || isNearPath(pos) || edt_env_->sdf_map_->isFootprintCleared(nbr))
                    continue;

                expanded.push_back(pos);
                cell_queue.push(nbr);
                frontier_flag_[adr] = 1;
            }
        }
    }

    if (expanded.size() > (size_t)cluster_min_)
    {
        Frontier frontier;
        frontier.cells_ = std::move(expanded);
        computeFrontierInfo(frontier);
        tmp_frontiers_.push_back(std::move(frontier));
    }
}

void FrontierFinder::updatePathHistory(const Eigen::Vector3d& ugv_pos)
{
    if (!path_frontier_suppress_enable_)
        return;

    // 按距离采样，避免历史点过密；同时保证至少保留当前位置
    if (path_history_.empty())
    {
        path_history_.push_back(ugv_pos);
    }
    else if ((ugv_pos - path_history_.back()).norm() >= path_history_sample_dist_)
    {
        path_history_.push_back(ugv_pos);
    }

    // 裁剪距离当前位置超过 path_history_length_ 的历史点
    while (!path_history_.empty())
    {
        if ((ugv_pos - path_history_.front()).norm() > path_history_length_)
            path_history_.pop_front();
        else
            break;
    }
}

bool FrontierFinder::isNearPath(const Eigen::Vector3d& pos) const
{
    if (!path_frontier_suppress_enable_ || path_history_.empty())
        return false;

    const double r2 = path_suppress_radius_ * path_suppress_radius_;
    for (const auto& p : path_history_)
    {
        const double dx = pos[0] - p[0];
        const double dy = pos[1] - p[1];
        if (dx * dx + dy * dy < r2)
            return true;
    }
    return false;
}

void FrontierFinder::splitLargeFrontiers(list<Frontier>& frontiers)
{
    list<Frontier> splits, tmps;
    for (auto it = frontiers.begin(); it != frontiers.end(); ++it)
    {
        // Check if each frontier needs to be split horizontally
        if (splitHorizontally(*it, splits))
        {
            tmps.insert(tmps.end(), splits.begin(), splits.end());
            splits.clear();
        }
        else
            tmps.push_back(std::move(*it));
    }
    frontiers = std::move(tmps);
}

bool FrontierFinder::splitHorizontally(const Frontier& frontier, list<Frontier>& splits)
{
    // Split a frontier into small piece if it is too large
    auto mean       = frontier.average_.head<2>();
    bool need_split = false;
    for (auto cell : frontier.filtered_cells_)
    {
        if ((cell.head<2>() - mean).norm() > cluster_size_xy_)
        {
            need_split = true;
            break;
        }
    }
    if (!need_split)
        return false;

    // Compute principal component
    // Covariance matrix of cells
    Eigen::Matrix2d cov;
    cov.setZero();
    for (auto cell : frontier.filtered_cells_)
    {
        Eigen::Vector2d diff = cell.head<2>() - mean;
        cov += diff * diff.transpose();
    }
    cov /= double(frontier.filtered_cells_.size());

    // Find eigenvector corresponds to maximal eigenvector
    Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
    auto                                values  = es.eigenvalues().real();
    auto                                vectors = es.eigenvectors().real();
    int                                 max_idx;
    double                              max_eigenvalue = -1000000;
    for (int i = 0; i < values.rows(); ++i)
    {
        if (values[i] > max_eigenvalue)
        {
            max_idx        = i;
            max_eigenvalue = values[i];
        }
    }
    Eigen::Vector2d first_pc = vectors.col(max_idx);
    // std::cout << "max idx: " << max_idx << std::endl;
    // std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() <<
    // std::endl;

    // Split the frontier into two groups along the first PC
    Frontier ftr1, ftr2;
    for (const auto& cell : frontier.cells_)
    {
        if ((cell.head<2>() - mean).dot(first_pc) >= 0)
            ftr1.cells_.push_back(cell);
        else
            ftr2.cells_.push_back(cell);
    }
    computeFrontierInfo(ftr1);
    computeFrontierInfo(ftr2);

    // Recursive call to split frontier that is still too large
    list<Frontier> splits2;
    if (splitHorizontally(ftr1, splits2))
    {
        splits.insert(splits.end(), splits2.begin(), splits2.end());
        splits2.clear();
    }
    else
        splits.push_back(std::move(ftr1));

    if (splitHorizontally(ftr2, splits2))
        splits.insert(splits.end(), splits2.begin(), splits2.end());
    else
        splits.push_back(std::move(ftr2));

    return true;
}

bool FrontierFinder::isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx)
{
    Vector3d pt;
    edt_env_->sdf_map_->indexToPos(idx, pt);
    for (auto box : boxes)
    {
        // Check if contained by a box
        bool inbox = true;
        for (int i = 0; i < 3; ++i)
        {
            inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
            if (!inbox)
                break;
        }
        if (inbox)
            return true;
    }
    return false;
}

/**
 * @brief 维护一个成本矩阵，记录所有前沿的路径成本和路径
 */
void FrontierFinder::updateFrontierCostMatrix()
{
    // std::cout << "cost mat size before remove: " << std::endl;
    // for (auto ftr : frontiers_)
    //   std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
    // std::cout << "" << std::endl;

    // 如果有移除的前沿ID，则清理对应的成本和路径
    if (!removed_ids_.empty())
    {
        // Delete path and cost for removed clusters
        for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it)
        {
            auto cost_iter = it->costs_.begin();
            auto path_iter = it->paths_.begin();
            int  iter_idx  = 0;
            for (int i = 0; i < removed_ids_.size(); ++i)
            {
                // Step iterator to the item to be removed
                while (iter_idx < removed_ids_[i])
                {
                    ++cost_iter;
                    ++path_iter;
                    ++iter_idx;
                }
                cost_iter = it->costs_.erase(cost_iter);
                path_iter = it->paths_.erase(path_iter);
            }
            // std::cout << "(" << it->costs_.size() << "," << it->paths_.size() << "), ";
        }
        removed_ids_.clear();
    }

    // No new frontier clusters → pairwise ViewNode/A* work would be empty; skip hot path.
    if (first_new_ftr_ == frontiers_.end())
        return;

    // 定义更新成本的 lambda 函数：计算两个前沿之间的路径和成本
    auto updateCost = [](const list<Frontier>::iterator& it1, const list<Frontier>::iterator& it2) {
        // std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
        // Search path from old cluster's top viewpoint to new cluster'
        Viewpoint&       vui = it1->viewpoints_.front();
        Viewpoint&       vuj = it2->viewpoints_.front();
        vector<Vector3d> path_ij;

        // 根据两个前沿的首个视点计算成本
        double cost_ij = ViewNode::computeCost(vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
        // Insert item for both old and new clusters (move once; it2 gets reversed copy; restore it1 orientation)
        it1->costs_.push_back(cost_ij);
        it1->paths_.push_back(std::move(path_ij));
        vector<Vector3d>& path_ref = it1->paths_.back();
        std::reverse(path_ref.begin(), path_ref.end());
        it2->costs_.push_back(cost_ij);
        it2->paths_.push_back(path_ref);
        std::reverse(path_ref.begin(), path_ref.end());
    };

    // std::cout << "cost mat add: " << std::endl;
    // 计算老前沿和新前沿之间的成本和路径
    for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
        for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
            updateCost(it1, it2);

    // 计算新前沿之间的成本和路径
    for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
        for (auto it2 = it1; it2 != frontiers_.end(); ++it2)
        {
            if (it1 == it2)
            {
                // std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
                it1->costs_.push_back(0);
                it1->paths_.emplace_back();
            }
            else
                updateCost(it1, it2);
        }
    first_new_ftr_ = frontiers_.end();

    // if (!frontiers_.empty())
    //     std::cout << "Frontier cost mat size: " << frontiers_.front().costs_.size() << std::endl;

    // std::cout << "cost mat size final: " << std::endl;
    // for (auto ftr : frontiers_)
    //   std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
    // std::cout << "" << std::endl;
}

void FrontierFinder::mergeFrontiers(Frontier& ftr1, const Frontier& ftr2)
{
    // Merge ftr2 into ftr1
    ftr1.average_ =
        (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) / (double(ftr1.cells_.size() + ftr2.cells_.size()));
    ftr1.cells_.insert(ftr1.cells_.end(), ftr2.cells_.begin(), ftr2.cells_.end());
    computeFrontierInfo(ftr1);
}

bool FrontierFinder::canBeMerged(const Frontier& ftr1, const Frontier& ftr2)
{
    Vector3d merged_avg =
        (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) / (double(ftr1.cells_.size() + ftr2.cells_.size()));
    // Check if it can merge two frontier without exceeding size limit
    for (auto c1 : ftr1.cells_)
    {
        auto diff = c1 - merged_avg;
        if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_)
            return false;
    }
    for (auto c2 : ftr2.cells_)
    {
        auto diff = c2 - merged_avg;
        if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_)
            return false;
    }
    return true;
}

bool FrontierFinder::haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2)
{
    // Check if two box have overlap part
    Vector3d bmin, bmax;
    for (int i = 0; i < 3; ++i)
    {
        bmin[i] = max(min1[i], min2[i]);
        bmax[i] = min(max1[i], max2[i]);
        if (bmin[i] > bmax[i] + 1e-3)
            return false;
    }
    return true;
}

bool FrontierFinder::haveAnyOverlap(const Vector3d& min1, const Vector3d& max1, const vector<Vector3d>& mins, const vector<Vector3d>& maxs)
{
    for (int i = 0; i < mins.size(); ++i)
    {
        if (haveOverlap(min1, max1, mins[i], maxs[i]))
            return true;
    }
    return false;
}

bool FrontierFinder::isFrontierChanged(const Frontier& ft)
{
    for (auto cell : ft.cells_)
    {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)))
            return true;
    }
    return false;
}

void FrontierFinder::computeFrontierInfo(Frontier& ftr)
{
    // Compute average position and bounding box of cluster
    ftr.average_.setZero();
    ftr.box_max_ = ftr.cells_.front();
    ftr.box_min_ = ftr.cells_.front();
    for (auto cell : ftr.cells_)
    {
        ftr.average_ += cell;
        for (int i = 0; i < 3; ++i)
        {
            ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
            ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
        }
    }
    ftr.average_ /= double(ftr.cells_.size());

    // Compute downsampled cluster
    downsample(ftr.cells_, ftr.filtered_cells_);
}

/**
 * @brief 从临时边界列表筛选可访问边界，并为每个边界生成候选视点，无法找到视点的边界移至休眠列表
 * @param
 *          tmp_frontiers_ 从searchFrontiers()中的大边界簇划分来的
 */

void FrontierFinder::computeFrontiersToVisit(const std::vector<std::vector<int>>& grid_2d)
{
    first_new_ftr_      = frontiers_.end();
    int new_num         = 0;
    int new_dormant_num = 0;
    // Try find viewpoints for each cluster and categorize them according to viewpoint number
    for (auto& tmp_ftr : tmp_frontiers_)
    {
        // 尝试在边界附近生成视点
        sampleViewpoints(tmp_ftr, grid_2d);
        if (!tmp_ftr.viewpoints_.empty())
        {
            ++new_num;
            list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), std::move(tmp_ftr));

            // 将能看到最多边界的视点排在前面（选出最佳视点）
            sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
                 [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
            if (first_new_ftr_ == frontiers_.end())
                first_new_ftr_ = inserted;
        }
        else
        {
            // 如果没有找到视点，则将该边界添加到休眠列表
            dormant_frontiers_.push_back(std::move(tmp_ftr));
            ++new_dormant_num;
        }
    }
    // 重置边界索引
    int idx = 0;
    for (auto& ft : frontiers_)
    {
        ft.id_ = idx++;
    }
    // std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
    RCLCPP_DEBUG(rclcpp::get_logger("frontier_finder"), "Frontier num: %zu, dormant: %zu", frontiers_.size(), dormant_frontiers_.size());
}

void FrontierFinder::computeFrontiersToVisit()
{
    first_new_ftr_      = frontiers_.end();
    int new_num         = 0;
    int new_dormant_num = 0;
    // Try find viewpoints for each cluster and categorize them according to viewpoint number
    for (auto& tmp_ftr : tmp_frontiers_)
    {
        // 尝试在边界附近生成视点
        sampleViewpoints(tmp_ftr);
        if (!tmp_ftr.viewpoints_.empty())
        {
            ++new_num;
            list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), std::move(tmp_ftr));

            // 将能看到最多边界的视点排在前面（选出最佳视点）
            sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
                 [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
            if (first_new_ftr_ == frontiers_.end())
                first_new_ftr_ = inserted;
        }
        else
        {
            // 如果没有找到视点，则将该边界添加到休眠列表
            dormant_frontiers_.push_back(std::move(tmp_ftr));
            ++new_dormant_num;
        }
    }
    // 重置边界索引
    int idx = 0;
    for (auto& ft : frontiers_)
    {
        ft.id_ = idx++;
    }
    // std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
    RCLCPP_DEBUG(rclcpp::get_logger("frontier_finder"), "Frontier num: %zu, dormant: %zu", frontiers_.size(), dormant_frontiers_.size());
}

/**
 * @brief 从边界列表中提取边界簇的最佳视点
 * @param
 *        in: 当前无人机的位置
 *        out:
 *              视点的位置集合
 *              视点偏航集合
 *              边界簇的平均位置集合
 */
void FrontierFinder::getTopViewpointsInfo(const Vector3d& cur_pos, vector<Eigen::Vector3d>& points, vector<double>& yaws, vector<Eigen::Vector3d>& averages)
{
    points.clear();
    yaws.clear();
    averages.clear();

    for (const auto& frontier : frontiers_)
    {
        bool no_view = true;
        for (const auto& view : frontier.viewpoints_)
        {
            // 只用 XY 平面距离过滤：视点 z 为 nav_z(0.3) 而车体 z 约 0.7，3D 距离中的 dz 会把 0.5m 阈值压到约 0.14m
            if ((view.pos_.head<2>() - cur_pos.head<2>()).norm() < min_candidate_dist_)
                continue;
            points.push_back(view.pos_);
            yaws.push_back(view.yaw_);
            averages.push_back(frontier.average_);
            no_view = false;
            break;
        }
        if (no_view)
        {
            // 所有视点都太近， 直接使用第一个视点（通常是覆盖率最高的）
            auto view = frontier.viewpoints_.front();
            points.push_back(view.pos_);
            yaws.push_back(view.yaw_);
            averages.push_back(frontier.average_);
        }
    }
}

void FrontierFinder::getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num, const double& max_decay,
                                       vector<vector<Eigen::Vector3d>>& points, vector<vector<double>>& yaws)
{
    points.clear();
    yaws.clear();
    points.reserve(ids.size());
    yaws.reserve(ids.size());
    for (auto id : ids)
    {
        // Scan all frontiers to find one with the same id
        for (const auto& frontier : frontiers_)
        {
            if (frontier.id_ == id)
            {
                // Get several top viewpoints that are far enough
                vector<Eigen::Vector3d> pts;
                vector<double>          ys;
                int                     visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
                for (const auto& view : frontier.viewpoints_)
                {
                    if (pts.size() >= view_num || view.visib_num_ <= visib_thresh)
                        break;
                    // 同上：XY 平面距离，避免 dz 吃掉阈值
                    if ((view.pos_.head<2>() - cur_pos.head<2>()).norm() < min_candidate_dist_)
                        continue;
                    pts.push_back(view.pos_);
                    ys.push_back(view.yaw_);
                }
                if (pts.empty())
                {
                    // All viewpoints are very close, ignore the distance limit
                    for (const auto& view : frontier.viewpoints_)
                    {
                        if (pts.size() >= view_num || view.visib_num_ <= visib_thresh)
                            break;
                        pts.push_back(view.pos_);
                        ys.push_back(view.yaw_);
                    }
                }
                points.push_back(std::move(pts));
                yaws.push_back(std::move(ys));
            }
        }
    }
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters)
{
    clusters.clear();
    clusters.reserve(frontiers_.size());
    for (const auto& frontier : frontiers_)
        clusters.push_back(frontier.cells_);
    // clusters.push_back(frontier.filtered_cells_);
}

void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters)
{
    clusters.clear();
    clusters.reserve(dormant_frontiers_.size());
    for (const auto& ft : dormant_frontiers_)
        clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes)
{
    boxes.clear();
    for (const auto& frontier : frontiers_)
    {
        Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
        Vector3d scale  = frontier.box_max_ - frontier.box_min_;
        boxes.emplace_back(center, scale);
    }
}

void FrontierFinder::getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path)
{
    if (frontier_ids.empty())
        return;

    // Make an frontier_indexer to access the frontier list easier
    vector<list<Frontier>::iterator> frontier_indexer;
    for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
        frontier_indexer.push_back(it);

    // Compute the path from current pos to the first frontier
    vector<Vector3d> segment;
    ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
    path.insert(path.end(), segment.begin(), segment.end());

    // Get paths of tour passing all clusters
    for (int i = 0; i < frontier_ids.size() - 1; ++i)
    {
        // Move to path to next cluster
        auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
        int  next_idx  = frontier_ids[i + 1];
        for (int j = 0; j < next_idx; ++j)
            ++path_iter;
        path.insert(path.end(), path_iter->begin(), path_iter->end());
    }
}

void FrontierFinder::getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, Eigen::MatrixXd& mat)
{
    // Use Asymmetric TSP
    int dimen = frontiers_.size();
    mat.resize(dimen + 1, dimen + 1);
    // std::cout << "mat size: " << mat.rows() << ", " << mat.cols() << std::endl;
    // Fill block for clusters
    int i = 1, j = 1;
    for (const auto& ftr : frontiers_)
    {
        for (const auto& cs : ftr.costs_)
        {
            // std::cout << "(" << i << ", " << j << ")"
            // << ", ";
            mat(i, j++) = cs;
        }
        ++i;
        j = 1;
    }
    // std::cout << "" << std::endl;

    // Fill block from current state to clusters
    mat.leftCols<1>().setZero();
    for (const auto& ftr : frontiers_)
    {
        // std::cout << "(0, " << j << ")"
        // << ", ";
        Viewpoint        vj = ftr.viewpoints_.front();
        vector<Vector3d> path;
        mat(0, j++) = ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
    }
    // std::cout << "" << std::endl;
}

void FrontierFinder::getSwarmCostMatrix(const vector<Vector3d>& positions, const vector<Vector3d>& velocities, const vector<double> yaws, Eigen::MatrixXd& mat)
{
    const int drone_num = positions.size();
    const int ftr_num   = frontiers_.size();
    const int dimen     = 1 + drone_num + ftr_num;
    mat                 = Eigen::MatrixXd::Zero(dimen, dimen);

    // Virtual depot to drones
    for (int i = 0; i < drone_num; ++i)
    {
        mat(0, 1 + i) = -1000;
        mat(1 + i, 0) = 1000;
    }
    // Virtual depot to frontiers
    for (int i = 0; i < ftr_num; ++i)
    {
        mat(0, 1 + drone_num + i) = 1000;
        mat(1 + drone_num + i, 0) = 0;
    }
    // Costs between drones
    for (int i = 0; i < drone_num; ++i)
    {
        for (int j = 0; j < drone_num; ++j)
        {
            mat(1 + i, 1 + j) = 10000;
        }
    }

    // Costs from drones to frontiers
    for (int i = 0; i < drone_num; ++i)
    {
        int j = 0;
        for (const auto& ftr : frontiers_)
        {
            Viewpoint        vj = ftr.viewpoints_.front();
            vector<Vector3d> path;
            mat(1 + i, 1 + drone_num + j) = ViewNode::computeCost(positions[i], vj.pos_, yaws[0], vj.yaw_, velocities[i], 0.0, path);
            mat(1 + drone_num + j, 1 + i) = 0;
            ++j;
        }
    }
    // Costs between frontiers
    int i = 0, j = 0;
    for (const auto& ftr : frontiers_)
    {
        for (const auto& cs : ftr.costs_)
        {
            mat(1 + drone_num + i, 1 + drone_num + j) = cs;
            ++j;
        }
        ++i;
        j = 0;
    }
    // Diag
    for (int i = 0; i < dimen; ++i)
    {
        mat(i, i) = 1000;
    }

    // std::cout << "mat: " << std::endl;
    // std::cout << mat << std::endl;
}

void FrontierFinder::getSwarmCostMatrix(const vector<Vector3d>& positions, const vector<Vector3d>& velocities, const vector<double>& yaws,
                                        const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos, Eigen::MatrixXd& mat)
{
    Eigen::MatrixXd full_mat;
    getSwarmCostMatrix(positions, velocities, yaws, full_mat);

    // Get part of the full matrix according to selected frontier

    const int drone_num = positions.size();
    const int ftr_num   = ftr_ids.size();
    int       dimen     = 1 + drone_num + ftr_num;
    if (!grid_pos.empty())
        dimen += 1;

    mat = Eigen::MatrixXd::Zero(dimen, dimen);

    // Virtual depot to drones
    for (int i = 0; i < drone_num; ++i)
    {
        mat(0, 1 + i) = -1000;
        mat(1 + i, 0) = 1000;
    }
    // Virtual depot to frontiers
    for (int i = 0; i < ftr_num; ++i)
    {
        mat(0, 1 + drone_num + i) = 1000;
        mat(1 + drone_num + i, 0) = 0;
    }
    // Costs between drones
    for (int i = 0; i < drone_num; ++i)
    {
        for (int j = 0; j < drone_num; ++j)
        {
            mat(1 + i, 1 + j) = 10000;
        }
    }

    // Costs from drones to frontiers
    for (int i = 0; i < drone_num; ++i)
    {
        for (int j = 0; j < ftr_num; ++j)
        {
            mat(1 + i, 1 + drone_num + j) = full_mat(1 + i, 1 + drone_num + ftr_ids[j]);
            mat(1 + drone_num + j, 1 + i) = 0;
        }
    }
    // Costs between frontiers
    for (int i = 0; i < ftr_num; ++i)
    {
        for (int j = 0; j < ftr_num; ++j)
        {
            mat(1 + drone_num + i, 1 + drone_num + j) = full_mat(1 + drone_num + ftr_ids[i], 1 + drone_num + ftr_ids[j]);
        }
    }
    // Diag
    for (int i = 0; i < dimen; ++i)
    {
        mat(i, i) = 1000;
    }

    // Consider next grid in global tour
    if (!grid_pos.empty())
    {
        // Depot, 1000, -1000
        mat(0, 1 + drone_num + ftr_num) = 1000;
        mat(1 + drone_num + ftr_num, 0) = -1000;

        // Drone
        for (int i = 0; i < drone_num; ++i)
        {
            mat(1 + i, 1 + drone_num + ftr_num) = 1000;
            mat(1 + drone_num + ftr_num, 1 + i) = 1000;
        }

        // Frontier
        vector<Eigen::Vector3d> points, tmps;
        vector<double>          yaws;
        getTopViewpointsInfo(positions[0], points, yaws, tmps);
        Eigen::Vector3d next_grid = grid_pos[0];

        for (int i = 0; i < ftr_num; ++i)
        {
            double cost                                     = ViewNode::computeCost(next_grid, points[ftr_ids[i]], 0, 0, Eigen::Vector3d(0, 0, 0), 0, tmps);
            mat(1 + drone_num + i, 1 + drone_num + ftr_num) = cost;
            mat(1 + drone_num + ftr_num, 1 + drone_num + i) = cost;
        }
    }
}

int FrontierFinder::computeGainOfView(const Eigen::Vector3d& pos, const double& yaw)
{
    percep_utils_->setPose(pos, yaw);

    // Compute info gain in the FOV
    Eigen::Vector3d bmin, bmax;
    percep_utils_->getFOVBoundingBox(bmin, bmax);

    int gain = 0;
    for (double sx = bmin[0]; sx <= bmax[0]; sx += 0.8)
    {
        for (double sy = bmin[1]; sy <= bmax[1]; sy += 0.8)
        {
            for (double sz = bmin[2]; sz <= bmax[2]; sz += 0.8)
            {
                Eigen::Vector3d sample(sx, sy, sz);
                if (percep_utils_->insideFOV(sample) && edt_env_->sdf_map_->getOccupancy(sample) == SDFMap::UNKNOWN)
                    ++gain;
            }
        }
    }
    return gain;
}

int FrontierFinder::deleteFrontiers(const vector<uint16_t>& ids)
{
    // Remove uninformative cluster and update cost matrix of other clusters
    auto tmp_ids = ids;
    sort(tmp_ids.begin(), tmp_ids.end(), greater<int>());

    removed_ids_.clear();
    int rmv_idx = 0;
    for (auto iter = frontiers_.begin(); iter != frontiers_.end() && !tmp_ids.empty();)
    {
        if (iter->id_ == tmp_ids.back())
        {
            // Reset flag and remove cluster from list
            iter = frontiers_.erase(iter);
            removed_ids_.push_back(rmv_idx);
            tmp_ids.pop_back();
            // Eigen::Vector3i idx;
            // for (auto cell : iter->cells_) {
            //   edt_env_->sdf_map_->posToIndex(cell, idx);
            //   frontier_flag_[toadr(idx)] = 0;
            // }
        }
        else
        {
            ++rmv_idx;
            ++iter;
        }
    }

    // Reset ids
    int idx = 0;
    for (auto& ft : frontiers_)
    {
        ft.id_ = idx++;
    }

    return frontiers_.size();
}

int FrontierFinder::addFrontiers(const vector<pair<Eigen::Vector3d, double>>& views)
{
    first_new_ftr_ = frontiers_.end();
    for (const auto& v : views)
    {
        // Create new frontier with only viewpoint info
        Viewpoint view;
        view.pos_ = v.first;
        view.yaw_ = v.second;

        Frontier ftr;
        ftr.viewpoints_.push_back(view);

        auto inserted = frontiers_.insert(frontiers_.end(), std::move(ftr));
        if (first_new_ftr_ == frontiers_.end())
            first_new_ftr_ = inserted;
    }

    int idx = 0;
    for (auto& ft : frontiers_)
    {
        ft.id_ = idx++;
    }
    for (const auto& ftr : frontiers_)
    {
        std::cout << ftr.viewpoints_.size() << ", ";
    }
    std::cout << "" << std::endl;

    return static_cast<int>(frontiers_.size());
}

void FrontierFinder::findViewpoints(const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps)
{
    if (!edt_env_->sdf_map_->isInBox(sample) || edt_env_->sdf_map_->getInflateOccupancy(sample) == 1 || isNearUnknown(sample))
        return;

    double left_angle_, right_angle_, vertical_angle_, ray_length_;

    // Central yaw is determined by frontier's average position and sample
    auto   dir = ftr_avg - sample;
    double hc  = atan2(dir[1], dir[0]);

    vector<int> slice_gains;
    slice_gains.reserve(24);
    // Evaluate info gain of different slices
    for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18)
    {
        // Compute gain of one slice
        int gain = 0;
        for (double phi_v = -vertical_angle_; phi_v <= vertical_angle_; phi_v += vertical_angle_ / 3)
        {
            // Find endpoint of a ray
            Vector3d end;
            end[0] = sample[0] + ray_length_ * cos(phi_v) * cos(hc + phi_h);
            end[1] = sample[1] + ray_length_ * cos(phi_v) * sin(hc + phi_h);
            end[2] = sample[2] + ray_length_ * sin(phi_v);

            // Do raycasting to check info gain
            Vector3i idx;
            raycaster_->input(sample, end);
            while (raycaster_->nextId(idx))
            {
                // Hit obstacle, stop the ray
                if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx))
                    break;
                // Count number of unknown cells
                if (edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
                    ++gain;
            }
        }
        slice_gains.push_back(gain);
    }

    // Sum up slices' gain to get different yaw's gain
    vector<pair<double, int>> yaw_gains;
    yaw_gains.reserve(6);
    for (int i = 0; i < 6; ++i)  // [-90,-10]-> [10,90], delta_yaw = 20, 6 groups
    {
        double yaw  = hc - M_PI_2 + M_PI / 9.0 * i + right_angle_;
        int    gain = 0;
        for (int j = 2 * i; j < 2 * i + 9; ++j)  // 80 degree hFOV, 9 slices
            gain += slice_gains[j];
        yaw_gains.emplace_back(yaw, gain);
    }

    // Get several yaws with highest gain
    vps.clear();
    sort(yaw_gains.begin(), yaw_gains.end(), [](const pair<double, int>& p1, const pair<double, int>& p2) { return p1.second > p2.second; });
    for (int i = 0; i < 3; ++i)
    {
        if (yaw_gains[i].second < min_visib_num_)
            break;
        Viewpoint vp = { sample, yaw_gains[i].first, yaw_gains[i].second };
        while (vp.yaw_ < -M_PI)
            vp.yaw_ += 2 * M_PI;
        while (vp.yaw_ > M_PI)
            vp.yaw_ -= 2 * M_PI;
        vps.push_back(std::move(vp));
    }
}

/**
 * @brief 在边界簇平均采样视点， 评估其覆盖范， 将合格视点加入到边界视点集中
 * @note：
 */
void FrontierFinder::sampleViewpoints(Frontier& frontier, const std::vector<std::vector<int>>& grid_2d)
{
    // 在圆形区域采样视点
    // 外环控制采样半径
    for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_; rc <= candidate_rmax_ + 1e-3; rc += dr)
        // 内循环 控制角度（phi），在 [-π, π) 范围内采样圆周上的点
        for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_)
        {
            // 以边界簇平均位置为中心， 画一圈圈同心圆
            double cphi = 0.0;
            double sphi = 0.0;
#if defined(__linux__) && (defined(__GNUC__) || defined(__clang__))
            __builtin_sincos(phi, &sphi, &cphi);
#else
            sphi = std::sin(phi);
            cphi = std::cos(phi);
#endif
            const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cphi, sphi, 0);

            Eigen::Vector3i grid_id;
            edt_env_->sdf_map_->posToGridIndex(sample_pos, grid_id);

            // 检查采样点是否合格：
            // 1. 必须在地图边界框内（isInBox）
            // 2. 不能在障碍物膨胀区域内（getInflateOccupancy == 1）
            // 3. 不能靠近未知区域（isNearUnknown）
            if (!edt_env_->sdf_map_->isInBox(sample_pos) || grid_2d[grid_id(1)][grid_id(0)] == 1 || isNearUnknown(sample_pos))
            {
                RCLCPP_WARN_THROTTLE(rclcpp::get_logger("active_perception"), *node_->get_clock(), 0.2, "Grid [%d, %d] is occupied", grid_id(0), grid_id(1));
                continue;
            }

            //* 计算平均yaw角度 */
            auto& cells = frontier.filtered_cells_;  //已知区域和未知区域的边界点（过滤后）
            // std::cout << "test filter cell: " << cells.size() << std::endl;
            Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();  // 参考方向：第一个单元到采样点的方向
            double          avg_yaw = 0.0;
            for (int i = 1; i < cells.size(); ++i)
            {
                Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
                double          yaw = acos(dir.dot(ref_dir));
                if (ref_dir.cross(dir)[2] < 0)
                    yaw = -yaw;
                avg_yaw += yaw;
            }

            avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);  // 计算平均偏航角度， 并加上参考方向的初始角度
            wrapYaw(avg_yaw);                                                  // 规范化到 [-π, π]

            // 计算从该视点能看到的边界单元数量
            int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
            if (visib_num > min_visib_num_)
            {
                frontier.viewpoints_.emplace_back(Viewpoint{ sample_pos, avg_yaw, visib_num });
                // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
            }
            // }
        }
}

void FrontierFinder::sampleViewpoints(Frontier& frontier)
{
    if (frontier.filtered_cells_.empty() || candidate_rnum_ <= 0 || candidate_dphi_ <= 0.0)
        return;

    // 对径向环和角度采样点展平后均匀抽样，限制每个前沿的候选评估次数。
    const int ring_count = candidate_rnum_ + 1;
    const int angles_per_ring = std::max(1, static_cast<int>(std::ceil(2.0 * M_PI / candidate_dphi_)));
    const int total_samples = ring_count * angles_per_ring;
    const int sample_stride = std::max(1, static_cast<int>(std::ceil(
        static_cast<double>(total_samples) / max_candidate_samples_)));
    const double dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
    for (int sample_index = 0; sample_index < total_samples; sample_index += sample_stride)
    {
        const int ring = sample_index / angles_per_ring;
        const int angle_index = sample_index % angles_per_ring;
        const double rc = candidate_rmin_ + ring * dr;
        const double phi = -M_PI + angle_index * (2.0 * M_PI / angles_per_ring);
            // 以边界簇平均位置为中心， 画一圈圈同心圆
            double cphi = 0.0;
            double sphi = 0.0;
#if defined(__linux__) && (defined(__GNUC__) || defined(__clang__))
            __builtin_sincos(phi, &sphi, &cphi);
#else
            sphi = std::sin(phi);
            cphi = std::cos(phi);
#endif
            const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cphi, sphi, 0);

            // 检查采样点是否合格：
            // 1. 必须在地图边界框内（isInBox）
            // 2. 不能在障碍物膨胀区域内（getInflateOccupancy == 1）
            // 3. 不能靠近未知区域（isNearUnknown）
            if (!edt_env_->sdf_map_->isInBox(sample_pos) || edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
                continue;

            //* 计算平均yaw角度 */
            auto& cells = frontier.filtered_cells_;  //已知区域和未知区域的边界点（过滤后）
            // std::cout << "test filter cell: " << cells.size() << std::endl;
            Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();  // 参考方向：第一个单元到采样点的方向
            double          avg_yaw = 0.0;
            for (int i = 1; i < cells.size(); ++i)
            {
                Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
                double          yaw = acos(dir.dot(ref_dir));
                if (ref_dir.cross(dir)[2] < 0)
                    yaw = -yaw;
                avg_yaw += yaw;
            }

            avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);  // 计算平均偏航角度， 并加上参考方向的初始角度
            wrapYaw(avg_yaw);                                                  // 规范化到 [-π, π]

            // 计算从该视点能看到的边界单元数量
            int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
            if (visib_num > min_visib_num_)
            {
                frontier.viewpoints_.emplace_back(Viewpoint{ sample_pos, avg_yaw, visib_num });
                // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
            }
            // }
        }
}

bool FrontierFinder::isFrontierCovered()
{
    Vector3d update_min, update_max;
    edt_env_->sdf_map_->getUpdatedBox(update_min, update_max);

    // Bounding box of external chunks
    vector<Eigen::Vector3d> chunk_mins, chunk_maxs;
    edt_env_->sdf_map_->mm_->getChunkBoxes(chunk_mins, chunk_maxs, false);

    vector<Eigen::Vector3d> mins, maxs;
    mins.push_back(update_min);
    mins.insert(mins.end(), chunk_mins.begin(), chunk_mins.end());
    maxs.push_back(update_max);
    maxs.insert(maxs.end(), chunk_maxs.begin(), chunk_maxs.end());

    auto checkChanges = [&](const list<Frontier>& frontiers) {
        for (const auto& ftr : frontiers)
        {
            // haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)
            if (!haveAnyOverlap(ftr.box_min_, ftr.box_max_, mins, maxs))
                continue;
            const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
            int       change_num    = 0;
            for (const auto& cell : ftr.cells_)
            {
                Eigen::Vector3i idx;
                edt_env_->sdf_map_->posToIndex(cell, idx);
                if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
                    return true;
            }
        }
        return false;
    };

    if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_))
        return true;

    return false;
}

bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos)
{
    const int vox_num = floor(min_candidate_clearance_ / resolution_);
    for (int x = -vox_num; x <= vox_num; ++x)
        for (int y = -vox_num; y <= vox_num; ++y)
            for (int z = -1; z <= 1; ++z)
            {
                Eigen::Vector3d vox;
                vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
                if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN)
                    return true;
            }
    return false;
}

/**
 * @brief 计算从指定视点能看到的边界单元数量
 */
int FrontierFinder::countVisibleCells(const Eigen::Vector3d& pos, const double& yaw, const vector<Eigen::Vector3d>& cluster)
{
    percep_utils_->setPose(pos, yaw);
    int             visib_num = 0;
    Eigen::Vector3i idx;
    for (const Eigen::Vector3d& cell : cluster)
    {
        // 检查当前单元是否在视点范围内
        if (!percep_utils_->insideFOV(cell))
            continue;

        // 检查当前单元是否被障碍物遮挡
        raycaster_->input(cell, pos);

        bool visib = true;
        while (raycaster_->nextId(idx))
        {
            // 检查当前体素是否被占用或未知
            if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
            {
                visib = false;
                break;
            }
        }
        if (visib)
            visib_num += 1;
    }
    return visib_num;
}

void FrontierFinder::downsample(const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out)
{
    // downsamping cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(cluster_in.size());
    for (const auto& cell : cluster_in)
        cloud->points.emplace_back(cell[0], cell[1], cell[2]);

    const double                  leaf_size = edt_env_->sdf_map_->getResolution() * down_sample_;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloudf);

    cluster_out.clear();
    cluster_out.reserve(cloudf->points.size());
    for (auto pt : cloudf->points)
        cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double& yaw)
{
    while (yaw < -M_PI)
        yaw += 2 * M_PI;
    while (yaw > M_PI)
        yaw -= 2 * M_PI;
}

Eigen::Vector3i FrontierFinder::searchClearVoxel(const Eigen::Vector3i& pt)
{
    queue<Eigen::Vector3i>  init_que;
    vector<Eigen::Vector3i> nbrs;
    Eigen::Vector3i         cur, start_idx;
    init_que.push(pt);
    // visited_flag_[toadr(pt)] = 1;

    while (!init_que.empty())
    {
        cur = init_que.front();
        init_que.pop();
        if (knownfree(cur))
        {
            start_idx = cur;
            break;
        }

        nbrs = sixNeighbors(cur);
        for (auto nbr : nbrs)
        {
            int adr = toadr(nbr);
            // if (visited_flag_[adr] == 0)
            // {
            //   init_que.push(nbr);
            //   visited_flag_[adr] = 1;
            // }
        }
    }
    return start_idx;
}

inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i& voxel)
{
    vector<Eigen::Vector3i> neighbors(6);
    Eigen::Vector3i         tmp;

    tmp          = voxel - Eigen::Vector3i(1, 0, 0);
    neighbors[0] = tmp;
    tmp          = voxel + Eigen::Vector3i(1, 0, 0);
    neighbors[1] = tmp;
    tmp          = voxel - Eigen::Vector3i(0, 1, 0);
    neighbors[2] = tmp;
    tmp          = voxel + Eigen::Vector3i(0, 1, 0);
    neighbors[3] = tmp;
    tmp          = voxel - Eigen::Vector3i(0, 0, 1);
    neighbors[4] = tmp;
    tmp          = voxel + Eigen::Vector3i(0, 0, 1);
    neighbors[5] = tmp;

    return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel)
{
    vector<Eigen::Vector3i> neighbors(10);
    Eigen::Vector3i         tmp;
    int                     count = 0;

    for (int x = -1; x <= 1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
        {
            if (x == 0 && y == 0)
                continue;
            tmp                = voxel + Eigen::Vector3i(x, y, 0);
            neighbors[count++] = tmp;
        }
    }
    neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
    neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
    return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel)
{
    vector<Eigen::Vector3i> neighbors(26);
    Eigen::Vector3i         tmp;
    int                     count = 0;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            for (int z = -1; z <= 1; ++z)
            {
                if (x == 0 && y == 0 && z == 0)
                    continue;
                tmp                = voxel + Eigen::Vector3i(x, y, z);
                neighbors[count++] = tmp;
            }
    return neighbors;
}

inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel)
{
    auto nbrs = sixNeighbors(voxel);
    for (const auto& nbr : nbrs)
    {
        // 增加 isInBox 检查，确保邻居在地图有效范围内
        if (edt_env_->sdf_map_->isInBox(nbr))
        {
            if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
                return true;
        }
        // 如果邻居在地图外，通常默认它是未知的（或者根据你的需求处理）
        // else { return true; }
    }
    return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx)
{

    return edt_env_->sdf_map_->toAddress(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i& idx)
{
    // 增加 isInBox 检查，防止非法索引访问 getOccupancy
    if (!edt_env_->sdf_map_->isInBox(idx))
        return false;

    return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
}

inline bool FrontierFinder::inmap(const Eigen::Vector3i& idx)
{
    return edt_env_->sdf_map_->isInMap(idx);
}

}  // namespace fast_planner
