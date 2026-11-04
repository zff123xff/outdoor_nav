#include <active_perception/graph_node.h>
#include <active_perception/uniform_grid.h>
#include <path_searching/astar2.h>
#include <plan_env/edt_environment.h>
#include <plan_env/multi_map_manager.h>
#include <plan_env/sdf_map.h>

#include <algorithm>
#include <cmath>
#include <queue>

namespace fast_planner
{
UniformGrid::UniformGrid(const shared_ptr<EDTEnvironment>& edt, rclcpp::Node::SharedPtr node, const int& level)
{
    node_      = node;
    this->edt_ = edt;

    // Read min, max, resolution here
    min_[0]                    = node->has_parameter("sdf_map/box_min_x") ? node->get_parameter("sdf_map/box_min_x").as_double()
                                                                          : node->declare_parameter<double>("sdf_map/box_min_x", 0.0);
    min_[1]                    = node->has_parameter("sdf_map/box_min_y") ? node->get_parameter("sdf_map/box_min_y").as_double()
                                                                          : node->declare_parameter<double>("sdf_map/box_min_y", 0.0);
    min_[2]                    = node->has_parameter("sdf_map/box_min_z") ? node->get_parameter("sdf_map/box_min_z").as_double()
                                                                          : node->declare_parameter<double>("sdf_map/box_min_z", 0.0);
    max_[0]                    = node->has_parameter("sdf_map/box_max_x") ? node->get_parameter("sdf_map/box_max_x").as_double()
                                                                          : node->declare_parameter<double>("sdf_map/box_max_x", 0.0);
    max_[1]                    = node->has_parameter("sdf_map/box_max_y") ? node->get_parameter("sdf_map/box_max_y").as_double()
                                                                          : node->declare_parameter<double>("sdf_map/box_max_y", 0.0);
    max_[2]                    = node->has_parameter("sdf_map/box_max_z") ? node->get_parameter("sdf_map/box_max_z").as_double()
                                                                          : node->declare_parameter<double>("sdf_map/box_max_z", 0.0);
    min_unknown_               = node->has_parameter("partitioning/min_unknown") ? node->get_parameter("partitioning/min_unknown").as_int()
                                                                                 : node->declare_parameter<int>("partitioning/min_unknown", 10000);
    min_frontier_              = node->has_parameter("partitioning/min_frontier") ? node->get_parameter("partitioning/min_frontier").as_int()
                                                                                  : node->declare_parameter<int>("partitioning/min_frontier", 100);
    min_free_                  = node->has_parameter("partitioning/min_free") ? node->get_parameter("partitioning/min_free").as_int()
                                                                              : node->declare_parameter<int>("partitioning/min_free", 3000);
    consistent_cost_           = node->has_parameter("partitioning/consistent_cost") ? node->get_parameter("partitioning/consistent_cost").as_double()
                                                                                     : node->declare_parameter<double>("partitioning/consistent_cost", 3.5);
    w_unknown_                 = node->has_parameter("partitioning/w_unknown") ? node->get_parameter("partitioning/w_unknown").as_double()
                                                                               : node->declare_parameter<double>("partitioning/w_unknown", 3.5);
    dead_zone_prune_enable_    = node->has_parameter("partitioning/dead_zone_prune_enable")
                                     ? node->get_parameter("partitioning/dead_zone_prune_enable").as_bool()
                                     : node->declare_parameter<bool>("partitioning/dead_zone_prune_enable", true);
    dead_zone_edge_sample_res_ = node->has_parameter("partitioning/dead_zone_edge_sample_res")
                                     ? node->get_parameter("partitioning/dead_zone_edge_sample_res").as_double()
                                     : node->declare_parameter<double>("partitioning/dead_zone_edge_sample_res", 0.2);
    visit_prune_enable_        = node->has_parameter("partitioning/visit_prune_enable") ? node->get_parameter("partitioning/visit_prune_enable").as_bool()
                                                                                        : node->declare_parameter<bool>("partitioning/visit_prune_enable", true);
    visit_prune_min_hits_      = node->has_parameter("partitioning/visit_prune_min_hits") ? node->get_parameter("partitioning/visit_prune_min_hits").as_int()
                                                                                          : node->declare_parameter<int>("partitioning/visit_prune_min_hits", 2);
    visit_prune_min_sec_       = node->has_parameter("partitioning/visit_prune_min_sec") ? node->get_parameter("partitioning/visit_prune_min_sec").as_double()
                                                                                         : node->declare_parameter<double>("partitioning/visit_prune_min_sec", 4.0);
    inactive_zone_enable_ = node->has_parameter("partitioning/inactive_zone_enable") ? node->get_parameter("partitioning/inactive_zone_enable").as_bool()
                                                                                     : node->declare_parameter<bool>("partitioning/inactive_zone_enable", false);
    inactive_zones_[0].x_min = node->has_parameter("partitioning/inactive_zone_x_min") ? node->get_parameter("partitioning/inactive_zone_x_min").as_double()
                                                                                       : node->declare_parameter<double>("partitioning/inactive_zone_x_min", 0.0);
    inactive_zones_[0].x_max = node->has_parameter("partitioning/inactive_zone_x_max") ? node->get_parameter("partitioning/inactive_zone_x_max").as_double()
                                                                                       : node->declare_parameter<double>("partitioning/inactive_zone_x_max", 0.0);
    inactive_zones_[0].y_min = node->has_parameter("partitioning/inactive_zone_y_min") ? node->get_parameter("partitioning/inactive_zone_y_min").as_double()
                                                                                       : node->declare_parameter<double>("partitioning/inactive_zone_y_min", 0.0);
    inactive_zones_[0].y_max = node->has_parameter("partitioning/inactive_zone_y_max") ? node->get_parameter("partitioning/inactive_zone_y_max").as_double()
                                                                                       : node->declare_parameter<double>("partitioning/inactive_zone_y_max", 0.0);
    inactive_zones_[1].enable  = node->has_parameter("partitioning/inactive_zone2_enable") ? node->get_parameter("partitioning/inactive_zone2_enable").as_bool()
                                                                                           : node->declare_parameter<bool>("partitioning/inactive_zone2_enable", false);
    inactive_zones_[1].x_min   = node->has_parameter("partitioning/inactive_zone2_x_min") ? node->get_parameter("partitioning/inactive_zone2_x_min").as_double()
                                                                                          : node->declare_parameter<double>("partitioning/inactive_zone2_x_min", 0.0);
    inactive_zones_[1].x_max   = node->has_parameter("partitioning/inactive_zone2_x_max") ? node->get_parameter("partitioning/inactive_zone2_x_max").as_double()
                                                                                          : node->declare_parameter<double>("partitioning/inactive_zone2_x_max", 0.0);
    inactive_zones_[1].y_min   = node->has_parameter("partitioning/inactive_zone2_y_min") ? node->get_parameter("partitioning/inactive_zone2_y_min").as_double()
                                                                                          : node->declare_parameter<double>("partitioning/inactive_zone2_y_min", 0.0);
    inactive_zones_[1].y_max   = node->has_parameter("partitioning/inactive_zone2_y_max") ? node->get_parameter("partitioning/inactive_zone2_y_max").as_double()
                                                                                          : node->declare_parameter<double>("partitioning/inactive_zone2_y_max", 0.0);

    double grid_size = node->has_parameter("partitioning/grid_size") ? node->get_parameter("partitioning/grid_size").as_double()
                                                                     : node->declare_parameter<double>("partitioning/grid_size", 5.0);

    auto size = max_ - min_;

    // resolution_ = size / 3;
    for (int i = 0; i < 2; ++i)
    {
        int num        = ceil(size[i] / grid_size);
        resolution_[i] = size[i] / double(num);
        for (int j = 1; j < level; ++j)
            resolution_[i] *= 0.5;
    }
    resolution_[2] = size[2];
    initialized_   = false;
    level_         = level;

    // path_finder_.reset(new Astar);
    // path_finder_->init(nh, edt);
}

UniformGrid::~UniformGrid() {}

void UniformGrid::initGridData()
{
    Eigen::Vector3d size = max_ - min_;

    std::cout << "--------------------------------------------" << std::endl;

    for (int i = 0; i < 3; ++i)
    {
        std::cout << "size_i " << i << ": " << size(i) << ", "
                  << "resolution : " << resolution_(i) << std::endl;

        grid_num_(i) = ceil(size(i) / resolution_[i]);
    }
    grid_data_.resize(grid_num_[0] * grid_num_[1] * grid_num_[2]);

    std::cout << "data size: " << grid_data_.size() << std::endl;
    std::cout << "grid num: " << grid_num_.transpose() << std::endl;
    std::cout << "resolution: " << resolution_.transpose() << std::endl;

    // Init each grid info
    for (int x = 0; x < grid_num_[0]; ++x)
    {
        for (int y = 0; y < grid_num_[1]; ++y)
        {
            for (int z = 0; z < grid_num_[2]; ++z)
            {
                Eigen::Vector3i id(x, y, z);
                auto&           grid = grid_data_[toAddress(id)];

                Eigen::Vector3d pos;
                indexToPos(id, 0.5, pos);
                if (use_swarm_tf_)
                {
                    pos = rot_sw_ * pos + trans_sw_;
                }

                grid.center_      = pos;
                grid.unknown_num_ = resolution_[0] * resolution_[1] * resolution_[2] / pow(edt_->sdf_map_->getResolution(), 3);

                grid.is_prev_relevant_ = true;
                grid.is_cur_relevant_  = true;
                grid.need_divide_      = false;
                if (level_ == 1)
                    grid.active_ = true;
                else
                    grid.active_ = false;
                if (inInactiveZone(grid.center_))
                {
                    grid.active_           = false;
                    grid.is_prev_relevant_ = false;
                    grid.is_cur_relevant_  = false;
                    grid.need_divide_      = false;
                }
            }
        }
    }
}

void UniformGrid::updateBaseCoor()
{
    for (int i = 0; i < grid_data_.size(); ++i)
    {
        auto& grid = grid_data_[i];
        // if (!grid.active_) continue;

        Eigen::Vector3i id;
        adrToIndex(i, id);

        // Compute vertices and box of grid in current drone's frame
        Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;
        indexToPos(id, 0.0, left_bottom);
        indexToPos(id, 1.0, right_top);
        left_top[0]     = left_bottom[0];
        left_top[1]     = right_top[1];
        left_top[2]     = left_bottom[2];
        right_bottom[0] = right_top[0];
        right_bottom[1] = left_bottom[1];
        right_bottom[2] = left_bottom[2];
        right_top[2]    = left_bottom[2];

        vector<Eigen::Vector3d> vertices = { left_bottom, right_bottom, right_top, left_top };
        if (use_swarm_tf_)
        {
            for (auto& vert : vertices)
                vert = rot_sw_ * vert + trans_sw_;
        }

        Eigen::Vector3d vmin, vmax;
        vmin = vmax = vertices[0];
        for (int j = 1; j < vertices.size(); ++j)
        {
            for (int k = 0; k < 2; ++k)
            {
                vmin[k] = min(vmin[k], vertices[j][k]);
                vmax[k] = max(vmax[k], vertices[j][k]);
            }
        }
        grid.vertices_ = vertices;
        grid.vmin_     = vmin;
        grid.vmax_     = vmax;

        // Compute normals of four separating lines
        grid.normals_.clear();
        for (int j = 0; j < 4; ++j)
        {
            Eigen::Vector3d dir = (vertices[(j + 1) % 4] - vertices[j]).normalized();
            grid.normals_.push_back(dir);
        }
        // std::cout << "Vertices of grid " << toAddress(id) << std::endl;
        // for (auto v : grid.vertices_)
        //   std::cout << v.transpose() << "; ";
        // std::cout << "\nNormals: " << std::endl;
        // for (auto n : grid.normals_)
        //   std::cout << n.transpose() << "; ";
        // std::cout << "\nbox: " << grid.vmin_.transpose() << ", " << grid.vmax_.transpose()
        //           << std::endl;
    }
}

// TODO：具体是怎么分配网格的？
void UniformGrid::updateGridData(const int& drone_id, vector<int>& grid_ids, vector<int>& parti_ids, vector<int>& parti_ids_all,
                                 const Eigen::Vector3d& robot_world, const std::vector<char>* coarse_reach_mask_for_l2)
{
    static_cast<void>(drone_id);
    // parti_ids are ids of grids that are assigned to THIS drone and should be divided
    // parti_ids_all are ids of ALL grids that should be divided

    for (auto& grid : grid_data_)
    {
        grid.is_updated_ = false;
    }
    parti_ids.clear();

    bool     reset = (level_ == 2);
    Vector3d update_min, update_max;
    edt_->sdf_map_->getUpdatedBox(update_min, update_max, reset);

    vector<Eigen::Vector3d> update_mins, update_maxs;
    edt_->sdf_map_->mm_->getChunkBoxes(update_mins, update_maxs, reset);

    // Rediscovered grid
    vector<int> rediscovered_ids;

    auto have_overlap = [](const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
        for (int m = 0; m < 2; ++m)
        {
            double bmin = max(min1[m], min2[m]);
            double bmax = min(max1[m], max2[m]);
            if (bmin > bmax + 1e-3)
                return false;
        }
        return true;
    };

    // For each grid, check overlap with updated box and update it if necessary
    for (int i = 0; i < grid_data_.size(); ++i)
    {
        auto& grid = grid_data_[i];
        if (!grid.active_)
            continue;

        // Check overlap with updated boxes
        bool overlap = false;
        for (int j = 0; j < update_mins.size(); ++j)
        {
            if (have_overlap(grid.vmin_, grid.vmax_, update_mins[j], update_maxs[j]))
            {
                overlap = true;
                break;
            }
        }
        bool overlap_with_fov = have_overlap(grid.vmin_, grid.vmax_, update_min, update_max);
        if (!overlap && !overlap_with_fov)
            continue;

        // Update the grid
        Eigen::Vector3i idx;
        adrToIndex(i, idx);
        updateGridInfo(idx);

        if (grid.need_divide_)
        {
            parti_ids_all.push_back(i);
            grid.active_ = false;
        }

        // Rediscovered relevant grid
        if (!overlap_with_fov)
            continue;
        if (!grid.is_prev_relevant_ && grid.is_cur_relevant_ && level_ > 1)
        {
            rediscovered_ids.push_back(i);
            // RCLCPP_WARN(rclcpp::get_logger("active_perception"), "Grid %d is rediscovered", i);
        }
    }

    // Patch code. To avoid incomplete update of level 2 grids...
    // extra_ids_ must only index grid_data_; negatives or OOB ids cause UB (unsigned subscript) /
    // SIGSEGV. See activateGrids(): we now store only in-range ids; still guard here if grid or
    // state changed between calls.
    const int grid_count = static_cast<int>(grid_data_.size());
    for (const int id : extra_ids_)
    {
        if (id < 0 || id >= grid_count)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[UniformGrid L%d] Skip extra id=%d (size=%d); stale index", level_, id,
                                 grid_count);
            continue;
        }
        const size_t idx_adr = static_cast<size_t>(id);
        if (grid_data_[idx_adr].is_updated_)
            continue;

        Eigen::Vector3i idx;
        adrToIndex(id, idx);
        updateGridInfo(idx);
    }
    extra_ids_.clear();

    dead_zone_reach_l1_.clear();
    if (dead_zone_prune_enable_)
    {
        if (level_ == 1)
            pruneDeadZoneUnknownGrids(robot_world);
        else if (level_ == 2 && coarse_reach_mask_for_l2 != nullptr && !coarse_reach_mask_for_l2->empty())
            pruneLevel2ByCoarseReach(*coarse_reach_mask_for_l2);
    }

    if (visit_prune_enable_)
        pruneVisitedStaleGrids(robot_world);

    // Keep relevant list consistent with pruning outputs.
    relevant_id_.clear();
    relevant_map_.clear();
    for (int i = 0; i < static_cast<int>(grid_data_.size()); ++i)
    {
        if (grid_data_[i].is_cur_relevant_)
        {
            relevant_id_.push_back(i);
            relevant_map_[i] = 1;
        }
    }

    if (!initialized_)
    {
        grid_ids.clear();
        initialized_ = true;
    }
    else
    {
        for (auto it = grid_ids.begin(); it != grid_ids.end();)
        {
            const int gid = *it;
            if (gid < 0 || gid >= static_cast<int>(grid_data_.size()))
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[UniformGrid L%d] Stripping invalid grid id=%d (size=%zu)", level_, gid,
                                     grid_data_.size());
                it = grid_ids.erase(it);
                continue;
            }
            const size_t gidx = static_cast<size_t>(gid);
            if (relevant_map_.find(gid) == relevant_map_.end())
            {
                // Remove irrelevant ones
                // std::cout << "Remove irrelevant: " << *it << std::endl;
                it = grid_ids.erase(it);
            }
            else if (grid_data_[gidx].need_divide_)
            {
                // Partition coarse grid
                // std::cout << "Remove divided: " << *it << std::endl;
                parti_ids.push_back(gid);
                it = grid_ids.erase(it);
                // grid_data_[gidx].active_ = false;
            }
            else
            {
                ++it;
            }
        }
        // Add rediscovered ones
        grid_ids.insert(grid_ids.end(), rediscovered_ids.begin(), rediscovered_ids.end());

        // sort(grid_ids.begin(), grid_ids.end());
    }
}

void UniformGrid::updateGridInfo(const Eigen::Vector3i& id)
{
    int   adr  = toAddress(id);
    auto& grid = grid_data_[adr];
    if (grid.is_updated_)
    {  // Ensure only one update to avoid repeated computation
        return;
    }
    grid.is_updated_ = true;

    grid.is_prev_relevant_ = grid.is_cur_relevant_;

    Eigen::Vector3d gmin, gmax;
    // indexToPos(id, 0.0, gmin);
    indexToPos(id, 1.0, gmax);  // Only the first 2 values of vmax is useful, should compute max here

    // Check if a voxel is inside the rotated box
    auto inside_box = [](const Eigen::Vector3d& vox, const GridInfo& grid) {
        // Check four separating planes(lines)
        for (int m = 0; m < 4; ++m)
        {
            if ((vox - grid.vertices_[m]).dot(grid.normals_[m]) <= 0.0)
                return false;
        }
        return true;
    };

    // Count known
    const double res = edt_->sdf_map_->getResolution();
    grid.center_.setZero();
    grid.unknown_num_ = 0;
    int free          = 0;
    for (double x = grid.vmin_[0]; x <= grid.vmax_[0]; x += res)
    {
        for (double y = grid.vmin_[1]; y <= grid.vmax_[1]; y += res)
        {
            for (double z = grid.vmin_[2]; z <= gmax[2]; z += res)
            {
                Eigen::Vector3d pos(x, y, z);
                if (!inside_box(pos, grid))
                    continue;

                int state = edt_->sdf_map_->getOccupancy(pos);
                if (state == SDFMap::FREE)
                {
                    free += 1;
                }
                else if (state == SDFMap::UNKNOWN)
                {
                    grid.center_ = (grid.center_ * grid.unknown_num_ + pos) / (grid.unknown_num_ + 1);
                    grid.unknown_num_ += 1;
                }
            }
        }
    }

    grid.free_num_        = free;
    grid.is_cur_relevant_ = isRelevant(grid);
    if (inInactiveZone(grid.center_))
    {
        grid.active_          = false;
        grid.is_cur_relevant_ = false;
    }

    // cout << "level: " << level_ << ", grid id: " << id.transpose() << ", adr: " << adr
    //      << ", unknown: " << grid.unknown_num_ << ", center: " << grid.center_.transpose()
    //      << ", rele: " << grid.is_cur_relevant_ << endl;

    if (level_ == 1 && grid.active_ && free > min_free_)
    {
        grid.need_divide_ = true;
    }
}

int UniformGrid::toAddress(const Eigen::Vector3i& id)
{
    return id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
}

void UniformGrid::adrToIndex(const int& adr, Eigen::Vector3i& idx)
{
    // id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
    int       tmp_adr = adr;
    const int a       = grid_num_(1) * grid_num_(2);
    const int b       = grid_num_(2);

    idx[0]  = tmp_adr / a;
    tmp_adr = tmp_adr % a;
    idx[1]  = tmp_adr / b;
    idx[2]  = tmp_adr % b;
}

void UniformGrid::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
{
    for (int i = 0; i < 3; ++i)
        id(i) = floor((pos(i) - min_(i)) / resolution_[i]);
}

void UniformGrid::indexToPos(const Eigen::Vector3i& id, const double& inc, Eigen::Vector3d& pos)
{
    // inc: 0 for min, 1 for max, 0.5 for mid point
    for (int i = 0; i < 3; ++i)
        pos(i) = (id(i) + inc) * resolution_[i] + min_(i);
}

void UniformGrid::activateGrids(const vector<int>& ids)
{
    const int grid_count = static_cast<int>(grid_data_.size());
    extra_ids_.clear();
    extra_ids_.reserve(ids.size());
    for (const int id : ids)
    {
        if (id < 0 || id >= grid_count)
            continue;
        extra_ids_.push_back(id);
        if (inInactiveZone(grid_data_[static_cast<size_t>(id)].center_))
        {
            grid_data_[static_cast<size_t>(id)].active_          = false;
            grid_data_[static_cast<size_t>(id)].is_cur_relevant_ = false;
            continue;
        }
        grid_data_[static_cast<size_t>(id)].active_ = true;
    }
    // Previously extra_ids_ copied full `ids`, including OOB/negative entries skipped above;
    // updateGridData indexed grid_data_[id] without checks → SIGSEGV. Only schedule in-range ids.
}

bool UniformGrid::insideGrid(const Eigen::Vector3i& id)
{
    // Check inside min max
    for (int i = 0; i < 3; ++i)
    {
        if (id[i] < 0 || id[i] >= grid_num_[i])
        {
            return false;
        }
    }
    return true;
}

bool UniformGrid::inInactiveZone(const Eigen::Vector3d& p) const
{
    if (!inactive_zone_enable_)
        return false;
    auto in_rect = [](const InactiveZoneRect& z, const Eigen::Vector3d& pt) {
        const double xmin = std::min(z.x_min, z.x_max);
        const double xmax = std::max(z.x_min, z.x_max);
        const double ymin = std::min(z.y_min, z.y_max);
        const double ymax = std::max(z.y_min, z.y_max);
        return (pt.x() >= xmin && pt.x() <= xmax && pt.y() >= ymin && pt.y() <= ymax);
    };
    if (in_rect(inactive_zones_[0], p))
        return true;
    if (inactive_zones_[1].enable && in_rect(inactive_zones_[1], p))
        return true;
    return false;
}

void UniformGrid::inputFrontiers(const vector<Eigen::Vector3d>& avgs)
{
    for (auto& grid : grid_data_)
    {
        grid.contained_frontier_ids_.clear();
    }
    Eigen::Vector3i id;
    Eigen::Matrix3d Rt    = rot_sw_.transpose();
    Eigen::Vector3d t_inv = -Rt * trans_sw_;

    for (int i = 0; i < avgs.size(); ++i)
    {
        Eigen::Vector3d pos = avgs[i];
        if (use_swarm_tf_)
        {
            pos = Rt * pos + t_inv;
        }
        posToIndex(pos, id);
        if (!insideGrid(id))
            continue;
        auto& grid                      = grid_data_[toAddress(id)];
        grid.contained_frontier_ids_[i] = 1;
    }
}

bool UniformGrid::isRelevant(const GridInfo& grid)
{
    // return grid.unknown_num_ >= min_unknown_ || grid.frontier_num_ >= min_frontier_;
    // return grid.unknown_num_ >= min_unknown_ || !grid.frontier_cell_nums_.empty();
    return grid.unknown_num_ >= min_unknown_ || !grid.contained_frontier_ids_.empty();
}

bool UniformGrid::lazyFaceTraversable(int adr_a, int adr_b)
{
    if (adr_a < 0 || adr_a >= static_cast<int>(grid_data_.size()) || adr_b < 0 || adr_b >= static_cast<int>(grid_data_.size()))
        return false;
    const GridInfo& A = grid_data_[adr_a];
    const GridInfo& B = grid_data_[adr_b];
    Eigen::Vector3i ia, ib;
    adrToIndex(adr_a, ia);
    adrToIndex(adr_b, ib);
    const int manh = std::abs(ia[0] - ib[0]) + std::abs(ia[1] - ib[1]) + std::abs(ia[2] - ib[2]);
    if (manh != 1)
        return false;

    const double zc   = 0.5 * (A.vmin_.z() + A.vmax_.z());
    const double step = std::max(0.05, dead_zone_edge_sample_res_);

    auto occOpen = [&](const Eigen::Vector3d& p) {
        const int occ = edt_->sdf_map_->getOccupancy(p);
        return occ == SDFMap::FREE || occ == SDFMap::UNKNOWN;
    };

    if (ia[0] != ib[0])
    {
        const double x_edge = 0.5 * ((ia[0] < ib[0] ? A.vmax_.x() + B.vmin_.x() : A.vmin_.x() + B.vmax_.x()));
        const double y0     = std::max(A.vmin_.y(), B.vmin_.y());
        const double y1     = std::min(A.vmax_.y(), B.vmax_.y());
        if (y1 < y0 - 1e-6)
            return false;
        for (double y = y0; y <= y1 + 1e-6; y += step)
        {
            if (occOpen(Eigen::Vector3d(x_edge, y, zc)))
                return true;
        }
    }
    else
    {
        const double y_edge = 0.5 * ((ia[1] < ib[1] ? A.vmax_.y() + B.vmin_.y() : A.vmin_.y() + B.vmax_.y()));
        const double x0     = std::max(A.vmin_.x(), B.vmin_.x());
        const double x1     = std::min(A.vmax_.x(), B.vmax_.x());
        if (x1 < x0 - 1e-6)
            return false;
        for (double x = x0; x <= x1 + 1e-6; x += step)
        {
            if (occOpen(Eigen::Vector3d(x, y_edge, zc)))
                return true;
        }
    }
    return false;
}

void UniformGrid::pruneDeadZoneUnknownGrids(const Eigen::Vector3d& robot_world)
{
    dead_zone_reach_l1_.assign(grid_data_.size(), 0);
    if (level_ != 1 || grid_data_.empty())
        return;

    Eigen::Vector3d rl = robot_world;
    if (use_swarm_tf_)
    {
        const Eigen::Matrix3d Rt = rot_sw_.transpose();
        rl                       = Rt * robot_world - Rt * trans_sw_;
    }

    std::queue<int> q;
    auto            try_seed = [&](int adr) {
        if (adr < 0 || adr >= static_cast<int>(grid_data_.size()))
            return;
        if (!dead_zone_reach_l1_[static_cast<size_t>(adr)])
        {
            dead_zone_reach_l1_[static_cast<size_t>(adr)] = 1;
            q.push(adr);
        }
    };

    Eigen::Vector3i rid;
    posToIndex(rl, rid);
    if (insideGrid(rid))
        try_seed(toAddress(rid));

    for (int i = 0; i < static_cast<int>(grid_data_.size()); ++i)
    {
        if (!grid_data_[i].contained_frontier_ids_.empty())
            try_seed(i);
    }

    if (q.empty())
    {
        dead_zone_reach_l1_.clear();
        return;
    }

    const int dirs[4][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
    while (!q.empty())
    {
        const int u = q.front();
        q.pop();
        Eigen::Vector3i iu;
        adrToIndex(u, iu);
        for (auto& d : dirs)
        {
            Eigen::Vector3i iv = iu;
            iv[0] += d[0];
            iv[1] += d[1];
            if (!insideGrid(iv))
                continue;
            const int v = toAddress(iv);
            if (dead_zone_reach_l1_[static_cast<size_t>(v)])
                continue;
            if (!lazyFaceTraversable(u, v))
                continue;
            dead_zone_reach_l1_[static_cast<size_t>(v)] = 1;
            q.push(v);
        }
    }

    for (int i = 0; i < static_cast<int>(grid_data_.size()); ++i)
    {
        GridInfo& g = grid_data_[i];
        if (!g.active_)
            continue;
        if (!g.contained_frontier_ids_.empty())
            continue;
        if (g.unknown_num_ < min_unknown_)
            continue;
        if (!dead_zone_reach_l1_[static_cast<size_t>(i)])
            g.is_cur_relevant_ = false;
    }
}

void UniformGrid::pruneLevel2ByCoarseReach(const std::vector<char>& coarse_reach)
{
    if (level_ != 2 || coarse_reach.empty())
        return;
    const int cn0 = std::max(1, grid_num_[0] / 2);
    const int cn1 = std::max(1, grid_num_[1] / 2);
    const int cn2 = std::max(1, grid_num_[2]);

    for (int fi = 0; fi < static_cast<int>(grid_data_.size()); ++fi)
    {
        GridInfo& g = grid_data_[fi];
        if (!g.active_)
            continue;
        if (!g.contained_frontier_ids_.empty())
            continue;
        if (g.unknown_num_ < min_unknown_)
            continue;
        Eigen::Vector3i fidx;
        adrToIndex(fi, fidx);
        const Eigen::Vector3i cidx(fidx[0] / 2, fidx[1] / 2, fidx[2]);
        if (cidx[0] < 0 || cidx[0] >= cn0 || cidx[1] < 0 || cidx[1] >= cn1 || cidx[2] < 0 || cidx[2] >= cn2)
            continue;
        const int c_adr = cidx[0] * cn1 * cn2 + cidx[1] * cn2 + cidx[2];
        if (c_adr < 0 || c_adr >= static_cast<int>(coarse_reach.size()))
            continue;
        if (!coarse_reach[static_cast<size_t>(c_adr)])
            g.is_cur_relevant_ = false;
    }
}

void UniformGrid::pruneVisitedStaleGrids(const Eigen::Vector3d& robot_world)
{
    if (grid_data_.empty())
        return;

    Eigen::Vector3d rl = robot_world;
    if (use_swarm_tf_)
    {
        const Eigen::Matrix3d Rt = rot_sw_.transpose();
        rl                       = Rt * robot_world - Rt * trans_sw_;
    }

    Eigen::Vector3i rid;
    posToIndex(rl, rid);
    if (!insideGrid(rid))
        return;

    const int adr = toAddress(rid);
    if (adr < 0 || adr >= static_cast<int>(grid_data_.size()))
        return;

    const double now = node_->now().seconds();
    GridInfo&    g   = grid_data_[adr];

    if (g.first_visit_stamp_ < 0.0)
        g.first_visit_stamp_ = now;
    g.last_visit_stamp_ = now;
    g.visit_hits_ += 1;

    if (!g.is_cur_relevant_ || !g.active_ || !g.contained_frontier_ids_.empty() || g.unknown_num_ < min_unknown_)
        return;

    const bool enough_hits = (g.visit_hits_ >= std::max(1, visit_prune_min_hits_));
    const bool enough_time = ((now - g.first_visit_stamp_) >= std::max(0.0, visit_prune_min_sec_));
    if (enough_hits && enough_time)
        g.is_cur_relevant_ = false;
}

void UniformGrid::getCostMatrix(const vector<Eigen::Vector3d>& positions, const vector<Eigen::Vector3d>& velocities, const vector<int>& prev_first_grid,
                                const vector<int>& grid_ids, Eigen::MatrixXd& mat)
{
}

void UniformGrid::getGridTour(const vector<int>& ids, vector<Eigen::Vector3d>& tour)
{
    tour.clear();
    for (int i = 0; i < ids.size(); ++i)
    {
        tour.push_back(grid_data_[ids[i]].center_);
    }
}

void UniformGrid::getFrontiersInGrid(const int& grid_id, vector<int>& ftr_ids)
{
    // Find frontier having more than 1/4 within the first grid
    auto& first_grid = grid_data_[grid_id];
    ftr_ids.clear();
    // for (auto pair : first_grid.frontier_cell_nums_) {
    //   ftr_ids.push_back(pair.first);
    // }
    for (auto pair : first_grid.contained_frontier_ids_)
    {
        ftr_ids.push_back(pair.first);
    }
}

void UniformGrid::getGridMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2)
{
    Eigen::Vector3d p1 = min_;
    Eigen::Vector3d p2 = min_ + Eigen::Vector3d(max_[0] - min_[0], 0, 0);
    for (int i = 0; i <= grid_num_[1]; ++i)
    {
        Eigen::Vector3d pt1 = p1 + Eigen::Vector3d(0, resolution_[1] * i, 0);
        Eigen::Vector3d pt2 = p2 + Eigen::Vector3d(0, resolution_[1] * i, 0);
        pts1.push_back(pt1);
        pts2.push_back(pt2);
    }

    p1 = min_;
    p2 = min_ + Eigen::Vector3d(0, max_[1] - min_[1], 0);
    for (int i = 0; i <= grid_num_[0]; ++i)
    {
        Eigen::Vector3d pt1 = p1 + Eigen::Vector3d(resolution_[0] * i, 0, 0);
        Eigen::Vector3d pt2 = p2 + Eigen::Vector3d(resolution_[0] * i, 0, 0);
        pts1.push_back(pt1);
        pts2.push_back(pt2);
    }
    for (auto& p : pts1)
        p[2] = 0.5;
    for (auto& p : pts2)
        p[2] = 0.5;
}

}  // namespace fast_planner
