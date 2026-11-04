#include <plan_env/multi_map_manager.h>
#include <plan_env/sdf_map.h>

#include <qos_helpers.h>  // msg_set：仿真/实机 QoS 模式（simulation 参数驱动）
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <fstream>

namespace
{
constexpr int kChunkStampsHistoryDepth = 5;
constexpr int kChunkDataHistoryDepth   = 16;
constexpr int kMarkerHistoryDepth      = 5;
// Per-peer queue capacity (hot + cold). Bounds memory while letting each peer
// keep enough backlog to survive bandwidth gaps; excess is evicted (cold first).
constexpr int kMaxPeerSendQueueSize = 4096;
}  // namespace

namespace fast_planner
{

MultiMapManager::MultiMapManager() {}

MultiMapManager::~MultiMapManager() {}

void MultiMapManager::setMap(SDFMap* map)
{
    this->map_ = map;
}

void MultiMapManager::init()
{
    drone_id_              = node_->has_parameter("exploration/drone_id") ? node_->get_parameter("exploration/drone_id").as_int()
                                                                          : node_->declare_parameter("exploration/drone_id", 1);
    vis_drone_id_          = node_->has_parameter("exploration/vis_drone_id") ? node_->get_parameter("exploration/vis_drone_id").as_int()
                                                                              : node_->declare_parameter("exploration/vis_drone_id", -1);
    map_num_               = node_->has_parameter("exploration/drone_num") ? node_->get_parameter("exploration/drone_num").as_int()
                                                                           : node_->declare_parameter("exploration/drone_num", 2);
    chunk_size_            = node_->has_parameter("multi_map_manager/chunk_size") ? node_->get_parameter("multi_map_manager/chunk_size").as_int()
                                                                                  : node_->declare_parameter("multi_map_manager/chunk_size", 500);
    stamp_period_ms_       = node_->has_parameter("multi_map_manager/stamp_period_ms") ? node_->get_parameter("multi_map_manager/stamp_period_ms").as_int()
                                                                                       : node_->declare_parameter("multi_map_manager/stamp_period_ms", 1000);
    chunk_timer_period_ms_ = node_->has_parameter("multi_map_manager/chunk_timer_period_ms")
                                 ? node_->get_parameter("multi_map_manager/chunk_timer_period_ms").as_int()
                                 : node_->declare_parameter("multi_map_manager/chunk_timer_period_ms", 100);
    tier_near_max_chunks_per_tick_ = node_->has_parameter("multi_map_manager/tier_near_max_chunks_per_tick")
                                         ? node_->get_parameter("multi_map_manager/tier_near_max_chunks_per_tick").as_int()
                                         : node_->declare_parameter("multi_map_manager/tier_near_max_chunks_per_tick", 5);

    // Distance-tiered sync parameters
    tier_near_distance_m_         = node_->has_parameter("multi_map_manager/tier_near_distance_m")
                                        ? node_->get_parameter("multi_map_manager/tier_near_distance_m").as_double()
                                        : node_->declare_parameter("multi_map_manager/tier_near_distance_m", 20.0);
    tier_mid_distance_m_          = node_->has_parameter("multi_map_manager/tier_mid_distance_m")
                                        ? node_->get_parameter("multi_map_manager/tier_mid_distance_m").as_double()
                                        : node_->declare_parameter("multi_map_manager/tier_mid_distance_m", 50.0);
    tier_far_distance_m_          = node_->has_parameter("multi_map_manager/tier_far_distance_m")
                                        ? node_->get_parameter("multi_map_manager/tier_far_distance_m").as_double()
                                        : node_->declare_parameter("multi_map_manager/tier_far_distance_m", 100.0);
    tier_near_interval_s_         = node_->has_parameter("multi_map_manager/tier_near_interval_s")
                                        ? node_->get_parameter("multi_map_manager/tier_near_interval_s").as_double()
                                        : node_->declare_parameter("multi_map_manager/tier_near_interval_s", 1.5);
    tier_mid_interval_s_          = node_->has_parameter("multi_map_manager/tier_mid_interval_s")
                                        ? node_->get_parameter("multi_map_manager/tier_mid_interval_s").as_double()
                                        : node_->declare_parameter("multi_map_manager/tier_mid_interval_s", 6.0);
    tier_far_interval_s_          = node_->has_parameter("multi_map_manager/tier_far_interval_s")
                                        ? node_->get_parameter("multi_map_manager/tier_far_interval_s").as_double()
                                        : node_->declare_parameter("multi_map_manager/tier_far_interval_s", 25.0);
    tier_mid_max_chunks_per_tick_ = node_->has_parameter("multi_map_manager/tier_mid_max_chunks_per_tick")
                                        ? node_->get_parameter("multi_map_manager/tier_mid_max_chunks_per_tick").as_int()
                                        : node_->declare_parameter("multi_map_manager/tier_mid_max_chunks_per_tick", 2);
    tier_far_max_chunks_per_tick_ = node_->has_parameter("multi_map_manager/tier_far_max_chunks_per_tick")
                                        ? node_->get_parameter("multi_map_manager/tier_far_max_chunks_per_tick").as_int()
                                        : node_->declare_parameter("multi_map_manager/tier_far_max_chunks_per_tick", 1);
    peer_position_timeout_s_      = node_->has_parameter("multi_map_manager/peer_position_timeout_s")
                                        ? node_->get_parameter("multi_map_manager/peer_position_timeout_s").as_double()
                                        : node_->declare_parameter("multi_map_manager/peer_position_timeout_s", 5.0);

    // ① stamp discovery is cheap: fixed fast interval (bandwidth throttle is on
    //   the data send only). ②④ global send budget and relevance gating radius.
    tier_stamp_interval_s_   = node_->has_parameter("multi_map_manager/tier_stamp_interval_s")
                                   ? node_->get_parameter("multi_map_manager/tier_stamp_interval_s").as_double()
                                   : node_->declare_parameter("multi_map_manager/tier_stamp_interval_s", 0.5);
    max_send_per_tick_       = node_->has_parameter("multi_map_manager/max_send_per_tick")
                                   ? node_->get_parameter("multi_map_manager/max_send_per_tick").as_int()
                                   : node_->declare_parameter("multi_map_manager/max_send_per_tick", std::max(4, map_num_));
    send_relevance_radius_m_ = node_->has_parameter("multi_map_manager/send_relevance_radius_m")
                                   ? node_->get_parameter("multi_map_manager/send_relevance_radius_m").as_double()
                                   : node_->declare_parameter("multi_map_manager/send_relevance_radius_m", 40.0);
    cov_box_timeout_s_  = node_->has_parameter("multi_map_manager/cov_box_timeout_s") ? node_->get_parameter("multi_map_manager/cov_box_timeout_s").as_double()
                                                                                      : node_->declare_parameter("multi_map_manager/cov_box_timeout_s", 3.0);
    shared_chunk_topic_ = node_->has_parameter("multi_map_manager/shared_chunk_topic")
                              ? node_->get_parameter("multi_map_manager/shared_chunk_topic").as_bool()
                              : node_->declare_parameter("multi_map_manager/shared_chunk_topic", false);

    // Visualization chunk publishing parameters (for ground station)
    visualization_chunk_enable_               = node_->has_parameter("multi_map_manager/visualization_chunk_enable")
                                                    ? node_->get_parameter("multi_map_manager/visualization_chunk_enable").as_bool()
                                                    : node_->declare_parameter("multi_map_manager/visualization_chunk_enable", true);
    visualization_chunk_topic_                = node_->has_parameter("multi_map_manager/visualization_chunk_topic")
                                                    ? node_->get_parameter("multi_map_manager/visualization_chunk_topic").as_string()
                                                    : node_->declare_parameter("multi_map_manager/visualization_chunk_topic", "/multi_map_manager/chunk_data/visualization");
    visualization_replay_max_chunks_per_tick_ = node_->has_parameter("multi_map_manager/visualization_replay_max_chunks_per_tick")
                                                    ? node_->get_parameter("multi_map_manager/visualization_replay_max_chunks_per_tick").as_int()
                                                    : node_->declare_parameter("multi_map_manager/visualization_replay_max_chunks_per_tick", 6);
    visualization_chunk_delay_s_              = node_->has_parameter("multi_map_manager/visualization_chunk_delay_s")
                                                    ? node_->get_parameter("multi_map_manager/visualization_chunk_delay_s").as_double()
                                                    : node_->declare_parameter("multi_map_manager/visualization_chunk_delay_s", 2.0);
    visualization_status_interval_s_          = node_->has_parameter("multi_map_manager/visualization_status_interval_s")
                                                    ? node_->get_parameter("multi_map_manager/visualization_status_interval_s").as_double()
                                                    : node_->declare_parameter("multi_map_manager/visualization_status_interval_s", 1.0);
    chunk_flush_timeout_s_                    = node_->has_parameter("multi_map_manager/chunk_flush_timeout_s")
                                                    ? node_->get_parameter("multi_map_manager/chunk_flush_timeout_s").as_double()
                                                    : node_->declare_parameter("multi_map_manager/chunk_flush_timeout_s", 1.0);
    chunk_flush_timeout_s_                    = std::max(0.2, chunk_flush_timeout_s_);

    stamp_period_ms_               = std::max(200, stamp_period_ms_);
    chunk_timer_period_ms_         = std::max(50, chunk_timer_period_ms_);
    tier_near_max_chunks_per_tick_ = std::max(1, tier_near_max_chunks_per_tick_);
    tier_mid_max_chunks_per_tick_  = std::max(1, std::min(tier_mid_max_chunks_per_tick_, tier_near_max_chunks_per_tick_));
    tier_far_max_chunks_per_tick_  = std::max(1, std::min(tier_far_max_chunks_per_tick_, tier_mid_max_chunks_per_tick_));

    RCLCPP_INFO(rclcpp::get_logger("multi_map_manager"),
                "[distance_tier] drone=%d enabled: near<%.1fm interval=%.1fs/%d, mid<%.1fm interval=%.1fs/%d, far<%.1fm interval=%.1fs/%d, timeout=%.1fs",
                drone_id_, tier_near_distance_m_, tier_near_interval_s_, tier_near_max_chunks_per_tick_, tier_mid_distance_m_, tier_mid_interval_s_,
                tier_mid_max_chunks_per_tick_, tier_far_distance_m_, tier_far_interval_s_, tier_far_max_chunks_per_tick_, peer_position_timeout_s_);
    RCLCPP_INFO(rclcpp::get_logger("multi_map_manager"),
                "[sync] drone=%d stamp_interval=%.2fs global_budget=%d/tick relevance_radius=%.1fm cov_timeout=%.1fs shared_topic=%d", drone_id_,
                tier_stamp_interval_s_, max_send_per_tick_, send_relevance_radius_m_, cov_box_timeout_s_, static_cast<int>(shared_chunk_topic_));

    stamp_timer_ = node_->create_wall_timer(std::chrono::milliseconds(stamp_period_ms_), std::bind(&MultiMapManager::stampTimerCallback, this));
    chunk_timer_ = node_->create_wall_timer(std::chrono::milliseconds(chunk_timer_period_ms_), std::bind(&MultiMapManager::chunkTimerCallback, this));

    const auto chunk_stamps_qos = swarm_qos::make_qos(kChunkStampsHistoryDepth, false);
    const auto chunk_data_qos   = swarm_qos::make_qos(kChunkDataHistoryDepth, false);
    const auto marker_qos       = swarm_qos::make_qos(kMarkerHistoryDepth, true);

    stamp_pub_ = node_->create_publisher<plan_env::msg::ChunkStamps>("/multi_map_manager/chunk_stamps_send", chunk_stamps_qos);
    // marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/multi_map_manager/marker_" + std::to_string(drone_id_), marker_qos);

    stamp_sub_       = node_->create_subscription<plan_env::msg::ChunkStamps>("/multi_map_manager/chunk_stamps_recv", chunk_stamps_qos,
                                                                        std::bind(&MultiMapManager::stampMsgCallback, this, std::placeholders::_1));
    chunk_sub_       = node_->create_subscription<plan_env::msg::ChunkData>("/multi_map_manager/chunk_data_recv", chunk_data_qos,
                                                                      std::bind(&MultiMapManager::chunkCallback, this, std::placeholders::_1));
    drone_state_sub_ = node_->create_subscription<msg_set::msg::DroneState>("/swarm_expl/drone_state", swarm_qos::make_qos(10, false),
                                                                            std::bind(&MultiMapManager::droneStateCallback, this, std::placeholders::_1));

    // Visualization chunk publishers (ground station / external subscribers)
    if (visualization_chunk_enable_)
    {
        const auto visualization_chunk_qos = swarm_qos::make_qos(64, false);
        chunk_visualization_pub_           = node_->create_publisher<plan_env::msg::ChunkData>(visualization_chunk_topic_, visualization_chunk_qos);
        RCLCPP_INFO(rclcpp::get_logger("multi_map_manager"),
                    "[chunk_visualization] drone=%d publishing incremental voxels on %s; delay=%.1fs replay=%d chunks/tick", drone_id_,
                    visualization_chunk_topic_.c_str(), visualization_chunk_delay_s_, visualization_replay_max_chunks_per_tick_);
    }

    multi_map_chunks_.resize(map_num_);
    for (auto& data : multi_map_chunks_)
    {
        data.idx_list_ = {};
    }
    chunk_boxes_.resize(map_num_);
    for (auto& box : chunk_boxes_)
    {
        box.valid_ = false;
    }
    chunk_buffer_.resize(map_num_);
    buffer_map_.resize(map_num_);
    last_chunk_stamp_time_.resize(map_num_);
    for (auto& time : last_chunk_stamp_time_)
        time = 0.0;
    last_chunk_stamp_rx_time_.resize(map_num_);
    for (auto& time : last_chunk_stamp_rx_time_)
        time = 0.0;
    peer_positions_.resize(map_num_);
    peer_position_stamps_.resize(map_num_);
    for (auto& time : peer_position_stamps_)
        time = 0.0;

    send_queues_.resize(map_num_ + 1);  // indexed by to_drone_id (1..map_num_)
    peer_cov_boxes_.resize(map_num_);
    peer_cov_box_stamps_.resize(map_num_);
    for (auto& time : peer_cov_box_stamps_)
        time = 0.0;

    // 初始化 flush 计时基准：避免 wall clock 下首帧点云即触发强制切分，
    // 也避免 sim time 重置后 last_chunk_cut 与当前时刻产生长时间负差抑制 flush。
    last_chunk_cut_time_seconds_ = node_->now().seconds();

    // // Test the idx list operation

    // // Find missed
    // vector<int> self1 = { 1, 1000 };
    // vector<int> self2 = { 5, 100, 105, 105, 120, 300, 400, 600, 700, 1000 };
    // vector<vector<int>> selfs = { self1, self2 };

    // vector<int> other1 = {};
    // vector<int> other2 = { 10, 80 };
    // vector<int> other3 = { 10, 80, 150, 200, 501, 550 };
    // vector<vector<int>> others = { other1, other2, other3 };

    // for (auto sf : selfs) {
    //   for (auto ot : others) {
    //     vector<int> missed;
    //     findMissedChunkIds(sf, ot, missed);
    //     std::cout << "Missed result: ";
    //     for (auto id : missed) std::cout << id << ", ";
    //     std::cout << "" << std::endl;
    //   }
    // }

    // // Merge chunk ids
    // // result: 1, 1000,
    // // result: 1, 9, 81, 1000,
    // // result: 1, 9, 81, 149, 201, 500, 551, 1000,
    // std::cout << "Compute merged ids" << std::endl;
    // vector<int> input1 = { 1, 1000 };
    // vector<int> input2 = { 1, 9, 81, 1000 };
    // vector<int> input3 = { 1, 7, 85, 140, 250, 450, 560, 900 };
    // vector<vector<int>> inputs = { input1, input2, input3 };
    // for (auto ot : others) {
    //   for (auto ip : inputs) {
    //     vector<int> output;
    //     mergeChunkIds(ot, ip, output);
    //     std::cout << "Merged result: ";
    //     for (auto id : output) std::cout << id << ", ";
    //     std::cout << "" << std::endl;
    //   }
    // }
}

void MultiMapManager::updateMapChunk(const vector<uint32_t>& adrs)
{
    if (drone_id_ <= 0 || drone_id_ > map_num_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[updateMapChunk] invalid drone_id=%d map_num=%d", drone_id_, map_num_);
        return;
    }
    if (chunk_size_ <= 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[updateMapChunk] invalid chunk_size=%d", chunk_size_);
        return;
    }

    adr_buffer_.reserve(adr_buffer_.size() + adrs.size());
    for (const auto adr : adrs)
    {
        // Keep only one pending entry per voxel. The occupancy value is read
        // from the map when the chunk is published, so this preserves the
        // newest value without sending duplicate addresses.
        if (pending_adr_keys_.insert(adr).second)
            adr_buffer_.push_back(adr);
    }

    cutPendingChunks(false);
}

void MultiMapManager::cutPendingChunks(const bool force)
{
    if (drone_id_ <= 0 || drone_id_ > map_num_)
        return;
    if (adr_buffer_.empty())
        return;
    if (!force && adr_buffer_.size() < static_cast<size_t>(chunk_size_))
        return;

    // force 时把当前缓冲整体切为一个 chunk，避免占用翻转更新滞留；
    // 常规路径仍按 chunk_size_ 切分。
    const size_t cut_len = force ? adr_buffer_.size() : static_cast<size_t>(chunk_size_);
    size_t       i       = 0;
    for (; i + cut_len <= adr_buffer_.size(); i += cut_len)
    {
        MapChunk chunk;
        chunk.voxel_adrs_.insert(chunk.voxel_adrs_.end(), adr_buffer_.begin() + i, adr_buffer_.begin() + i + cut_len);
        chunk.idx_ = multi_map_chunks_[drone_id_ - 1].chunks_.size() + 1;
        // if (drone_id_ == 1) std::cout << "Drone 1 insert chunk " << chunk.idx_ << std::endl;
        chunk.need_query_   = true;
        chunk.empty_        = false;
        chunk.created_time_ = node_->now().seconds();

        // Precompute the spatial box once; used by relevance-gated sending (④).
        chunk.box_valid_ = false;
        for (const uint32_t adr : chunk.voxel_adrs_)
        {
            Eigen::Vector3i idx;
            Eigen::Vector3d pos;
            adrToIndex(adr, idx);
            map_->indexToPos(idx, pos);
            if (!chunk.box_valid_)
            {
                for (int k = 0; k < 3; ++k)
                    chunk.box_min_[k] = chunk.box_max_[k] = pos[k];
                chunk.box_valid_ = true;
            }
            else
            {
                for (int k = 0; k < 3; ++k)
                {
                    chunk.box_min_[k] = std::min(chunk.box_min_[k], pos[k]);
                    chunk.box_max_[k] = std::max(chunk.box_max_[k], pos[k]);
                }
            }
        }

        multi_map_chunks_[drone_id_ - 1].chunks_.push_back(chunk);
        for (const auto adr : chunk.voxel_adrs_)
            pending_adr_keys_.erase(adr);
    }
    if (multi_map_chunks_[drone_id_ - 1].chunks_.empty())
    {
        return;
    }
    if (multi_map_chunks_[drone_id_ - 1].idx_list_.empty())
    {
        multi_map_chunks_[drone_id_ - 1].idx_list_ = { 1, 1 };
    }
    multi_map_chunks_[drone_id_ - 1].idx_list_.back() = multi_map_chunks_[drone_id_ - 1].chunks_.back().idx_;

    // std::cout << "idx list: ";
    // for (auto id : multi_map_chunks_[drone_id_ - 1].idx_list_) std::cout << id << ", ";
    // std::cout << "" << std::endl;
    // std::cout << "chunk size: " << multi_map_chunks_[drone_id_ - 1].chunks_.size() << std::endl;

    // Remove already inserted data
    vector<uint32_t> tmp;
    tmp.insert(tmp.end(), adr_buffer_.begin() + i, adr_buffer_.end());
    adr_buffer_                  = tmp;
    last_chunk_cut_time_seconds_ = node_->now().seconds();
}

void MultiMapManager::flushPendingChunkIfStale()
{
    if (adr_buffer_.empty())
        return;
    if (node_->now().seconds() - last_chunk_cut_time_seconds_ < chunk_flush_timeout_s_)
        return;
    // 长期凑不满 chunk_size_ 的尾量（多为占用翻转体素）也强制成 chunk，
    // 保证地面站能收到动态障碍物移走后的 FREE 更新。
    cutPendingChunks(true);
}

void MultiMapManager::stampTimerCallback()
{
    // Send stamp of chunks to other drones
    plan_env::msg::ChunkStamps msg;
    msg.from_drone_id = drone_id_;
    msg.time          = node_->now().seconds();

    for (auto chunks : multi_map_chunks_)
    {
        plan_env::msg::IdxList idx_list;
        idx_list.ids = chunks.idx_list_;
        msg.idx_lists.push_back(idx_list);
    }
    stamp_pub_->publish(msg);

    return;

    // Test, visualize the chunks
    static int pub_num = 0;
    static int pub_id  = 0;

    visualization_msgs::msg::Marker mk;
    mk.header.frame_id    = "world";
    mk.ns                 = "own";
    mk.header.stamp       = node_->now();
    mk.type               = visualization_msgs::msg::Marker::CUBE_LIST;
    mk.pose.orientation.w = 1.0;
    mk.color.r            = 1;
    mk.color.g            = 0;
    mk.color.b            = 0;
    mk.color.a            = 0.5;
    mk.scale.x            = 0.1;
    mk.scale.y            = 0.1;
    mk.scale.z            = 0.1;

    auto& data = multi_map_chunks_[drone_id_ - 1];
    if (pub_num < data.chunks_.size())
    {
        for (int i = pub_num; i < data.chunks_.size(); ++i)
        {
            // std::cout << "Chunk size : " << chunk.voxel_adrs_.size() << std::endl;
            auto& chunk = data.chunks_[i];
            // std::cout << chunk.stamp_ - first_stamp << ", ";

            for (auto adr : chunk.voxel_adrs_)
            {
                Eigen::Vector3i idx;
                Eigen::Vector3d pos;
                adrToIndex(adr, idx);
                map_->indexToPos(idx, pos);
                if (map_->getOccupancy(idx) == SDFMap::OCCUPIED)
                {
                    geometry_msgs::msg::Point pt;
                    pt.x = pos[0];
                    pt.y = pos[1];
                    pt.z = pos[2];
                    mk.points.push_back(pt);
                }
            }
        }
        if (!mk.points.empty())
        {
            pub_num = data.chunks_.size();
            mk.id   = pub_id++;
            // marker_pub_->publish(mk);
        }
    }
}

void MultiMapManager::stampMsgCallback(const plan_env::msg::ChunkStamps::ConstSharedPtr& msg)
{
    if (msg->from_drone_id == drone_id_)
        return;
    // if (drone_id_ == map_num_)
    //     return;  // Ground node does not send chunk
    if (msg->from_drone_id <= 0 || msg->from_drone_id > map_num_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[stampMsg] invalid from_drone_id=%d, map_num=%d", msg->from_drone_id, map_num_);
        return;
    }
    if (msg->idx_lists.size() < multi_map_chunks_.size())
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[stampMsg] idx_lists size mismatch: got=%zu expected>=%zu", msg->idx_lists.size(),
                     multi_map_chunks_.size());
        return;
    }

    // auto t1 = rclcpp::Clock().now();

    // ① stamp discovery is cheap: fixed fast interval so missing chunks are found
    //    quickly; the data send budget (drainSendQueue) is the real throttle.
    //    Gate on LOCAL receipt time so sender/receiver clock skew cannot drop stamps.
    const double min_interval = peerMinStampInterval(msg->from_drone_id);
    const double now_rx       = node_->now().seconds();
    if (now_rx - last_chunk_stamp_rx_time_[msg->from_drone_id - 1] < min_interval)
        return;
    last_chunk_stamp_rx_time_[msg->from_drone_id - 1] = now_rx;
    last_chunk_stamp_time_[msg->from_drone_id - 1]    = msg->time;

    vector<int> missed;
    findMissedChunkIds(multi_map_chunks_[drone_id_ - 1].idx_list_, msg->idx_lists[drone_id_ - 1].ids, missed);
    sendChunks(drone_id_, msg->from_drone_id, missed);
}

void MultiMapManager::findMissedChunkIds(const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids)
{
    if ((self_idx_list.size() % 2) != 0 || (other_idx_list.size() % 2) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[findMissedChunkIds] invalid interval list size: self=%zu other=%zu", self_idx_list.size(),
                     other_idx_list.size());
        return;
    }

    // Compute the complement set of other idx
    if (other_idx_list.empty())
    {
        miss_ids = self_idx_list;
        return;
    }

    vector<int> not_in_other;
    if (other_idx_list[0] > 1)
    {
        not_in_other.push_back(1);
        not_in_other.push_back(other_idx_list[0] - 1);
    }
    for (int i = 1; i < other_idx_list.size(); i += 2)
    {
        not_in_other.push_back(other_idx_list[i] + 1);
        if (i == other_idx_list.size() - 1)
        {
            int infinite = std::numeric_limits<int>::max();
            not_in_other.push_back(infinite);
        }
        else
        {
            not_in_other.push_back(other_idx_list[i + 1] - 1);
        }
    }

    // Compute the intersection of self and not_in_other (brute-force, O(n^2))
    for (int i = 0; i < self_idx_list.size(); i += 2)
    {
        for (int j = 0; j < not_in_other.size(); j += 2)
        {
            int minr, maxr;
            if (findIntersect(self_idx_list[i], self_idx_list[i + 1], not_in_other[j], not_in_other[j + 1], minr, maxr))
            {
                miss_ids.push_back(minr);
                miss_ids.push_back(maxr);
            }
        }
    }
}

bool MultiMapManager::findIntersect(const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr)
{
    minr = max(min1, min2);
    maxr = min(max1, max2);
    if (minr <= maxr)
        return true;
    return false;
}

void MultiMapManager::sendChunks(const int& chunk_drone_id, const int& to_drone_id, const vector<int>& idx_list)
{
    if (chunk_drone_id <= 0 || chunk_drone_id > map_num_ || to_drone_id <= 0 || to_drone_id > map_num_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[sendChunks] invalid drone ids: chunk=%d to=%d map_num=%d", chunk_drone_id, to_drone_id,
                     map_num_);
        return;
    }
    if ((idx_list.size() % 2) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[sendChunks] invalid idx interval size=%zu", idx_list.size());
        return;
    }

    // Cold-start throttle: limit the total number of chunks enqueued per stamp
    // round to kSendWindowSize. This caps the MB burst while preserving the
    // existing hot/cold relevance gating inside enqueueChunkSend — chunks are
    // enqueued in creation order (lowest idx first, matching the missed interval
    // list), and enqueueChunkSend routes them to hot_ or cold_ by spatial
    // relevance. The drainSendQueue LIFO then sends the newest (most relevant)
    // hot chunks first, so priority is preserved.
    auto&     data     = multi_map_chunks_[chunk_drone_id - 1];
    int       enqueued = 0;
    const int limit    = kSendWindowSize;

    for (int i = 0; i < idx_list.size(); i += 2)
    {
        if (idx_list[i] <= 0 || idx_list[i + 1] < idx_list[i])
        {
            RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[sendChunks] invalid idx range [%d, %d]", idx_list[i], idx_list[i + 1]);
            continue;
        }
        for (int j = idx_list[i]; j <= idx_list[i + 1]; ++j)
        {
            if (enqueued >= limit)
                break;
            if (j <= 0 || j > static_cast<int>(data.chunks_.size()))
            {
                RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[sendChunks] idx out of range: idx=%d chunk_size=%zu", j, data.chunks_.size());
                continue;
            }
            enqueueChunkSend(chunk_drone_id, to_drone_id, j);
            ++enqueued;
        }
        if (enqueued >= limit)
            break;
    }
}

uint64_t MultiMapManager::sendQueueKey(const int chunk_drone_id, const int to_drone_id, const int chunk_idx)
{
    return (static_cast<uint64_t>(chunk_drone_id) << 42) | (static_cast<uint64_t>(to_drone_id) << 32) | static_cast<uint32_t>(chunk_idx);
}

void MultiMapManager::enqueueChunkSend(const int chunk_drone_id, const int to_drone_id, const int chunk_idx)
{
    if (to_drone_id <= 0 || to_drone_id > map_num_)
        return;
    const uint64_t key = sendQueueKey(chunk_drone_id, to_drone_id, chunk_idx);
    if (send_queue_keys_.find(key) != send_queue_keys_.end())
        return;

    // ② route to the target peer's own queue so one peer's burst (e.g. a far
    //   peer's long missed range) can never starve another peer's urgent data.
    // ④ relevance-gated: chunks near the peer go to the hot queue (sent first),
    //   far-away chunks go to the cold queue (deferred / only when idle).
    auto& peer_q = send_queues_[to_drone_id];
    auto& target = chunkRelevantToPeer(chunk_drone_id, chunk_idx, to_drone_id) ? peer_q.hot_ : peer_q.cold_;

    // Per-peer capacity: evict this peer's oldest (cold first, then hot) instead
    // of dropping the new one. LIFO drain keeps the freshest data anyway.
    while (static_cast<int>(peer_q.hot_.size() + peer_q.cold_.size()) >= kMaxPeerSendQueueSize)
    {
        PendingChunkSend victim;
        bool             found = false;
        if (!peer_q.cold_.empty())
        {
            victim = peer_q.cold_.front();
            peer_q.cold_.pop_front();
            found = true;
        }
        else if (!peer_q.hot_.empty())
        {
            victim = peer_q.hot_.front();
            peer_q.hot_.pop_front();
            found = true;
        }
        if (!found)
            break;
        send_queue_keys_.erase(sendQueueKey(victim.chunk_drone_id, victim.to_drone_id, victim.chunk_idx));
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "[enqueueChunkSend] peer queue full, evicting oldest chunk drone=%d idx=%d to=%d to make room for drone=%d idx=%d to=%d",
                             victim.chunk_drone_id, victim.chunk_idx, victim.to_drone_id, chunk_drone_id, chunk_idx, to_drone_id);
    }
    target.push_back({ chunk_drone_id, to_drone_id, chunk_idx });
    send_queue_keys_.insert(key);
}

rclcpp::Publisher<plan_env::msg::ChunkData>::SharedPtr MultiMapManager::getChunkPublisher(const int to_drone_id)
{
    // ④ optional shared topic: publish once, all peers subscribe and filter by
    //   to_drone_id. Saves N-1 redundant sends IF the network does multicast /
    //   relay dedup. Off by default (keeps current per-peer unicast topics).
    if (shared_chunk_topic_)
    {
        if (!shared_chunk_pub_)
        {
            const auto chunk_data_qos = swarm_qos::make_qos(kChunkDataHistoryDepth, false);
            shared_chunk_pub_         = node_->create_publisher<plan_env::msg::ChunkData>("/multi_map_manager/chunk_data", chunk_data_qos);
        }
        return shared_chunk_pub_;
    }

    const auto it = chunk_pubs_.find(to_drone_id);
    if (it != chunk_pubs_.end())
        return it->second;
    const auto        chunk_data_qos = swarm_qos::make_qos(kChunkDataHistoryDepth, false);
    const std::string topic          = "/multi_map_manager/chunk_data/to_" + std::to_string(to_drone_id);
    auto              pub            = node_->create_publisher<plan_env::msg::ChunkData>(topic, chunk_data_qos);
    chunk_pubs_[to_drone_id]         = pub;
    return pub;
}

bool MultiMapManager::publishChunkMessage(const int chunk_drone_id, const int to_drone_id, const int chunk_idx)
{
    if (chunk_drone_id != drone_id_)
        return false;
    if (chunk_drone_id <= 0 || chunk_drone_id > map_num_ || to_drone_id <= 0 || to_drone_id > map_num_)
        return false;
    if (chunk_idx <= 0 || chunk_idx > static_cast<int>(multi_map_chunks_[chunk_drone_id - 1].chunks_.size()))
        return false;

    auto& data  = multi_map_chunks_[chunk_drone_id - 1];
    auto& chunk = data.chunks_[chunk_idx - 1];

    plan_env::msg::ChunkData msg;
    msg.from_drone_id      = drone_id_;
    msg.to_drone_id        = to_drone_id;
    msg.chunk_drone_id     = chunk_drone_id;
    msg.idx                = chunk.idx_;
    msg.voxel_adrs         = chunk.voxel_adrs_;
    msg.voxel_count        = static_cast<uint32_t>(chunk.voxel_adrs_.size());
    msg.occupancy_encoding = 1;
    if (chunk_drone_id == drone_id_ && chunk.need_query_)
    {
        getOccOfChunk(chunk.voxel_adrs_, chunk.voxel_occ);
        chunk.need_query_ = false;
    }
    msg.voxel_occ_bits.assign((chunk.voxel_adrs_.size() + 7) / 8, 0);
    for (size_t i = 0; i < chunk.voxel_occ.size() && i < chunk.voxel_adrs_.size(); ++i)
    {
        if (chunk.voxel_occ[i] != 0)
            msg.voxel_occ_bits[i / 8] |= static_cast<uint8_t>(1u << (i % 8));
    }
    msg.voxel_occ.clear();
    getChunkPublisher(to_drone_id)->publish(msg);
    return true;
}

void MultiMapManager::drainSendQueue()
{
    // ② per-peer fair scheduling with nearest-peer priority. Peers close to us
    //   (coordination-critical) get their per-peer budget first; far peers get
    //   what remains. LIFO within each peer keeps the freshest chunk first and,
    //   combined with ④ hot/cold gating, hot (near-peer) data dominates.
    std::vector<int> peers;
    for (int to = 1; to <= map_num_; ++to)
    {
        if (!send_queues_[to].hot_.empty() || !send_queues_[to].cold_.empty())
            peers.push_back(to);
    }
    std::sort(peers.begin(), peers.end(), [this](const int a, const int b) { return horizontalDistanceToPeer(a) < horizontalDistanceToPeer(b); });

    int sent_total = 0;
    for (const int to : peers)
    {
        if (sent_total >= max_send_per_tick_)
            break;
        auto&     peer_q    = send_queues_[to];
        const int budget    = peerMaxChunksPerTick(to);
        int       sent_peer = 0;
        while (sent_peer < budget && sent_total < max_send_per_tick_ && (!peer_q.hot_.empty() || !peer_q.cold_.empty()))
        {
            PendingChunkSend item;
            bool             found = false;
            if (!peer_q.hot_.empty())
            {
                item = peer_q.hot_.back();
                peer_q.hot_.pop_back();
                found = true;
            }
            else if (!peer_q.cold_.empty())
            {
                item = peer_q.cold_.back();
                peer_q.cold_.pop_back();
                found = true;
            }
            if (!found)
                break;
            send_queue_keys_.erase(sendQueueKey(item.chunk_drone_id, item.to_drone_id, item.chunk_idx));
            if (publishChunkMessage(item.chunk_drone_id, item.to_drone_id, item.chunk_idx))
            {
                ++sent_total;
                ++sent_peer;
            }
        }
    }
}

void MultiMapManager::getOccOfChunk(const vector<uint32_t>& adrs, vector<uint8_t>& occs)
{
    for (auto adr : adrs)
    {
        uint8_t occ = map_->md_->occupancy_buffer_[adr] > map_->mp_->min_occupancy_log_ ? 1 : 0;
        occs.push_back(occ);
    }
}

void MultiMapManager::filterDynamicFootprintOcc(const MapChunk& chunk, vector<uint8_t>& occs)
{
    if (occs.size() != chunk.voxel_adrs_.size())
        return;

    // ① 逐体素动态命中标记检查：车体被激光 hit 且落点在队友附近时置位，
    // 直接以 free 发布（数组读，开销可忽略）。
    for (size_t i = 0; i < chunk.voxel_adrs_.size(); ++i)
    {
        if (map_->md_->dynamic_hit_flag_[chunk.voxel_adrs_[i]])
            occs[i] = 0;
    }

    // ② 队友当前位置附近的体素同样以 free 发布（与 hit 标记半径一致）；
    // 先用 chunk 盒与各有效队友位置的快速剔除避免逐体素距离计算。
    const double dyn_hit_radius  = map_->mp_->swarm_clear_radius_ + 0.3;
    const double dyn_r2          = dyn_hit_radius * dyn_hit_radius;
    const double now_s           = node_->now().seconds();
    bool         chunk_near_peer = false;
    if (chunk.box_valid_)
    {
        for (int i = 0; i < map_num_; ++i)
        {
            if (peer_position_stamps_[i] <= 1e-6 || now_s - peer_position_stamps_[i] > peer_position_timeout_s_)
                continue;
            const Eigen::Vector2d pc(peer_positions_[i].x(), peer_positions_[i].y());
            double                dx = 0.0, dy = 0.0;
            if (pc.x() < chunk.box_min_[0])
                dx = chunk.box_min_[0] - pc.x();
            else if (pc.x() > chunk.box_max_[0])
                dx = pc.x() - chunk.box_max_[0];
            if (pc.y() < chunk.box_min_[1])
                dy = chunk.box_min_[1] - pc.y();
            else if (pc.y() > chunk.box_max_[1])
                dy = pc.y() - chunk.box_max_[1];
            if (dx * dx + dy * dy <= dyn_r2)
            {
                chunk_near_peer = true;
                break;
            }
        }
    }
    if (!chunk_near_peer)
        return;

    const int yz_dim = map_->mp_->map_voxel_num_(1) * map_->mp_->map_voxel_num_(2);
    if (yz_dim <= 0 || map_->mp_->map_voxel_num_(2) <= 0)
        return;
    for (size_t i = 0; i < chunk.voxel_adrs_.size(); ++i)
    {
        if (occs[i] == 0)
            continue;
        const int       adr = chunk.voxel_adrs_[i];
        Eigen::Vector3i idx(adr / yz_dim, (adr % yz_dim) / map_->mp_->map_voxel_num_(2), adr % map_->mp_->map_voxel_num_(2));
        Eigen::Vector3d voxel_pos;
        map_->indexToPos(idx, voxel_pos);
        for (int p = 0; p < map_num_; ++p)
        {
            if (peer_position_stamps_[p] <= 1e-6 || now_s - peer_position_stamps_[p] > peer_position_timeout_s_)
                continue;
            const Eigen::Vector3d& pp = peer_positions_[p];
            const double           dx = voxel_pos.x() - pp.x();
            const double           dy = voxel_pos.y() - pp.y();
            const double           dz = voxel_pos.z() - pp.z();
            if (dx * dx + dy * dy + dz * dz <= dyn_r2)
            {
                occs[i] = 0;
                break;
            }
        }
    }
}

void MultiMapManager::publishVisualizationChunk(const MapChunk& chunk)
{
    if (!chunk_visualization_pub_ || chunk.empty_ || chunk.idx_ == 0)
        return;

    plan_env::msg::ChunkData msg;
    msg.from_drone_id      = drone_id_;
    msg.to_drone_id        = 0;  // Broadcast stream for visualization only.
    msg.chunk_drone_id     = drone_id_;
    msg.idx                = chunk.idx_;
    msg.latest_idx         = static_cast<uint32_t>(multi_map_chunks_[drone_id_ - 1].chunks_.size());
    msg.voxel_adrs         = chunk.voxel_adrs_;
    msg.voxel_count        = static_cast<uint32_t>(chunk.voxel_adrs_.size());
    msg.occupancy_encoding = 0;
    // Always query current occupancy for the visualization stream instead of
    // freezing the state seen when the voxel was first discovered.
    getOccOfChunk(chunk.voxel_adrs_, msg.voxel_occ);
    // 动态车体足迹过滤：地面站可视化不显示车体痕迹（swarm 互发不受影响）。
    filterDynamicFootprintOcc(chunk, msg.voxel_occ);
    if (chunk_visualization_pub_->get_subscription_count() > 0)
        chunk_visualization_pub_->publish(msg);
    visualization_replay_active_ = true;
}

void MultiMapManager::publishVisualizationHeartbeat()
{
    if (!chunk_visualization_pub_ || drone_id_ <= 0 || drone_id_ > map_num_)
        return;

    const size_t subscriber_count = chunk_visualization_pub_->get_subscription_count();
    const double now_s            = node_->now().seconds();
    if (subscriber_count == 0 || now_s - last_visualization_status_time_ < visualization_status_interval_s_)
        return;

    plan_env::msg::ChunkData msg;
    msg.from_drone_id  = drone_id_;
    msg.to_drone_id    = 0;
    msg.chunk_drone_id = drone_id_;
    msg.idx            = 0;  // Status-only message; no voxel payload.
    msg.latest_idx     = static_cast<uint32_t>(multi_map_chunks_[drone_id_ - 1].chunks_.size());
    chunk_visualization_pub_->publish(msg);
    last_visualization_status_time_ = now_s;
}

void MultiMapManager::drainVisualizationReplay()
{
    if (!chunk_visualization_pub_ || drone_id_ <= 0 || drone_id_ > map_num_)
        return;

    const size_t subscriber_count = chunk_visualization_pub_->get_subscription_count();
    // Replay only on the 0 -> non-zero transition. Starting a bandwidth tool
    // as a second subscriber must not resend the full history to the backend.
    if (last_visualization_subscription_count_ == 0 && subscriber_count > 0)
    {
        visualization_replay_next_idx_ = 0;
        visualization_replay_active_   = true;
        RCLCPP_INFO(rclcpp::get_logger("multi_map_manager"), "[chunk_visualization] new subscriber detected; replaying %zu stored chunks from drone %d",
                    multi_map_chunks_[drone_id_ - 1].chunks_.size(), drone_id_);
    }
    last_visualization_subscription_count_ = subscriber_count;
    if (subscriber_count == 0 || !visualization_replay_active_)
        return;

    const auto& chunks = multi_map_chunks_[drone_id_ - 1].chunks_;
    int         sent   = 0;
    while (visualization_replay_next_idx_ < chunks.size() && sent < visualization_replay_max_chunks_per_tick_)
    {
        if (node_->now().seconds() - chunks[visualization_replay_next_idx_].created_time_ < visualization_chunk_delay_s_)
            break;
        publishVisualizationChunk(chunks[visualization_replay_next_idx_]);
        ++visualization_replay_next_idx_;
        ++sent;
    }
}

void MultiMapManager::chunkCallback(const plan_env::msg::ChunkData::ConstSharedPtr& msg)
{
    // Receive chunks from other drones, store them in chunk buffer
    if (msg->from_drone_id == drone_id_)
        return;
    if (msg->to_drone_id != drone_id_)
        return;
    if (msg->chunk_drone_id <= 0 || msg->chunk_drone_id > map_num_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[chunkCallback] invalid chunk_drone_id=%d map_num=%d", msg->chunk_drone_id, map_num_);
        return;
    }
    if (msg->idx <= 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[chunkCallback] invalid chunk idx=%d from drone=%d", msg->idx, msg->chunk_drone_id);
        return;
    }

    plan_env::msg::ChunkData normalized = *msg;
    if (msg->voxel_count == 0 && msg->occupancy_encoding == 0 && msg->voxel_adrs.size() == msg->voxel_occ.size())
    {
        // Accept pre-compression peers during a rolling deployment.
        normalized.voxel_count = static_cast<uint32_t>(msg->voxel_adrs.size());
    }
    else if (msg->voxel_count != msg->voxel_adrs.size() || msg->voxel_count == 0)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[chunkCallback] invalid voxel_count=%u addresses=%zu idx=%u", msg->voxel_count,
                             msg->voxel_adrs.size(), msg->idx);
        return;
    }
    if (msg->occupancy_encoding > 1)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[chunkCallback] unsupported occupancy encoding=%u idx=%u",
                             msg->occupancy_encoding, msg->idx);
        return;
    }
    if (msg->occupancy_encoding == 1 && msg->voxel_occ_bits.size() != (msg->voxel_count + 7u) / 8u)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[chunkCallback] invalid packed occupancy size=%zu voxel_count=%u idx=%u",
                             msg->voxel_occ_bits.size(), msg->voxel_count, msg->idx);
        return;
    }
    if (msg->occupancy_encoding == 0 && msg->voxel_occ.size() != msg->voxel_count)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[chunkCallback] invalid legacy occupancy size=%zu voxel_count=%u idx=%u",
                             msg->voxel_occ.size(), msg->voxel_count, msg->idx);
        return;
    }

    normalized.voxel_occ.resize(msg->voxel_count);
    if (msg->occupancy_encoding == 1)
    {
        for (uint32_t i = 0; i < msg->voxel_count; ++i)
            normalized.voxel_occ[i] = (msg->voxel_occ_bits[i / 8u] >> (i % 8u)) & 1u;
    }

    if (buffer_map_[msg->chunk_drone_id - 1].find(msg->idx) != buffer_map_[msg->chunk_drone_id - 1].end())
        return;

    // RCLCPP_ERROR("received msg idx: %d, from %d to %d", msg->idx, msg->from_drone_id,
    // msg->to_drone_id);
    chunk_buffer_[msg->chunk_drone_id - 1].push_back(std::move(normalized));
    buffer_map_[msg->chunk_drone_id - 1][msg->idx] = 1;

    return;
}

void MultiMapManager::droneStateCallback(const msg_set::msg::DroneState::SharedPtr msg)
{
    if (msg->drone_id == drone_id_)
        return;
    if (msg->drone_id <= 0 || msg->drone_id > map_num_)
        return;
    if (msg->pos.size() < 3)
        return;

    peer_positions_[msg->drone_id - 1]       = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
    peer_position_stamps_[msg->drone_id - 1] = node_->now().seconds();

    // ③ store the peer's explored box arriving on the fast drone_state channel.
    //    Freshness is judged by LOCAL receipt time, so no clock sync is needed.
    if (msg->cov_valid != 0 && !msg->cov_box_min.empty() && !msg->cov_box_max.empty())
    {
        auto& cb = peer_cov_boxes_[msg->drone_id - 1];
        for (int k = 0; k < 3; ++k)
        {
            cb.min_[k] = msg->cov_box_min[k];
            cb.max_[k] = msg->cov_box_max[k];
        }
        cb.valid_                               = true;
        peer_cov_box_stamps_[msg->drone_id - 1] = node_->now().seconds();
    }
}

double MultiMapManager::horizontalDistanceToPeer(int peer_id) const
{
    if (peer_id <= 0 || peer_id > map_num_)
        return std::numeric_limits<double>::max();
    const double dt = node_->now().seconds() - peer_position_stamps_[peer_id - 1];
    if (dt > peer_position_timeout_s_ || peer_position_stamps_[peer_id - 1] <= 1e-6)
        return 0.0;  // Position unknown: be conservative and sync normally
    const Eigen::Vector3d diff = drone_pos_ - peer_positions_[peer_id - 1];
    return std::sqrt(diff.x() * diff.x() + diff.y() * diff.y());
}

double MultiMapManager::peerMinStampInterval(int) const
{
    // ① stamp discovery is cheap (a few hundred bytes of idx lists) and is the
    //   ONLY way to learn "what chunks are missing". Throttling it by distance
    //   used to delay gap discovery by 20s+, which is what made the shared map
    //   stale and caused clustering. Keep it fast and fixed; the real bandwidth
    //   gate is the data send budget (peerMaxChunksPerTick / max_send_per_tick_).
    return tier_stamp_interval_s_;
}

int MultiMapManager::peerMaxChunksPerTick(int peer_id) const
{
    const double d = horizontalDistanceToPeer(peer_id);
    if (d < tier_near_distance_m_)
        return tier_near_max_chunks_per_tick_;
    if (d < tier_mid_distance_m_)
        return tier_mid_max_chunks_per_tick_;
    return tier_far_max_chunks_per_tick_;
}

bool MultiMapManager::chunkRelevantToPeer(const int chunk_drone_id, const int chunk_idx, const int to_drone_id) const
{
    if (send_relevance_radius_m_ <= 0.0)
        return true;  // gating disabled: everything is hot (legacy behavior)
    if (chunk_drone_id <= 0 || chunk_drone_id > map_num_ || to_drone_id <= 0 || to_drone_id > map_num_)
        return true;
    if (chunk_idx <= 0 || chunk_idx > static_cast<int>(multi_map_chunks_[chunk_drone_id - 1].chunks_.size()))
        return true;
    const auto& chunk = multi_map_chunks_[chunk_drone_id - 1].chunks_[chunk_idx - 1];
    if (!chunk.box_valid_)
        return true;

    // Unknown peer position -> defer (cold). The peer will get its data once it
    // is seen again or when this queue goes idle.
    const double dt = node_->now().seconds() - peer_position_stamps_[to_drone_id - 1];
    if (peer_position_stamps_[to_drone_id - 1] <= 1e-6 || dt > peer_position_timeout_s_)
        return false;

    // Distance from the peer to the chunk's 2D bounding box.
    const Eigen::Vector2d pc(peer_positions_[to_drone_id - 1].x(), peer_positions_[to_drone_id - 1].y());
    double                dx = 0.0, dy = 0.0;
    if (pc.x() < chunk.box_min_[0])
        dx = chunk.box_min_[0] - pc.x();
    else if (pc.x() > chunk.box_max_[0])
        dx = pc.x() - chunk.box_max_[0];
    if (pc.y() < chunk.box_min_[1])
        dy = chunk.box_min_[1] - pc.y();
    else if (pc.y() > chunk.box_max_[1])
        dy = pc.y() - chunk.box_max_[1];
    return (dx * dx + dy * dy) <= send_relevance_radius_m_ * send_relevance_radius_m_;
}

bool MultiMapManager::inPeerCoveredBox(const Eigen::Vector3d& p, double margin) const
{
    const double now = node_->now().seconds();
    for (int i = 0; i < map_num_; ++i)
    {
        if (i + 1 == drone_id_)
            continue;  // own coverage is local, no need to query it
        const auto& cb = peer_cov_boxes_[i];
        if (!cb.valid_)
            continue;
        if (now - peer_cov_box_stamps_[i] > cov_box_timeout_s_)
            continue;  // stale: peer no longer working this area
        // Degenerate / not-yet-explored box (all_min_ initialized to +1e6 etc.)
        bool degenerate = false;
        for (int k = 0; k < 3; ++k)
        {
            if (cb.min_[k] > cb.max_[k])
            {
                degenerate = true;
                break;
            }
        }
        if (degenerate)
            continue;
        bool inside = true;
        for (int k = 0; k < 3; ++k)
        {
            if (p[k] < cb.min_[k] - margin || p[k] > cb.max_[k] + margin)
            {
                inside = false;
                break;
            }
        }
        if (inside)
            return true;
    }
    return false;
}

void MultiMapManager::chunkTimerCallback()
{
    drainSendQueue();
    drainVisualizationReplay();
    publishVisualizationHeartbeat();
    flushPendingChunkIfStale();

    // Not process chunk until swarm basecoor transform is available
    Eigen::Vector4d tmp;
    // if (!map_->getBaseCoor(1, tmp)) {
    //   RCLCPP_WARN("basecoor not available yet.");
    //   return;
    // }

    // auto t1 = rclcpp::Clock().now();

    // Process chunks in buffers
    for (int i = 0; i < chunk_buffer_.size(); ++i)
    {
        auto& buffer = chunk_buffer_[i];
        // 本次融合产生的 occupied→free 翻转体素（队友确认清除的足迹），
        // 循环结束后统一重发，撤销本车在地面站的 occupied 引用。
        vector<uint32_t> flipped_free_adrs;
        if (buffer.empty())
            continue;
        buffer.erase(std::remove_if(buffer.begin(), buffer.end(), [](const plan_env::msg::ChunkData& chunk) { return chunk.idx <= 0; }), buffer.end());
        if (buffer.empty())
        {
            buffer_map_[i].clear();
            continue;
        }

        // Compute the idx list of buffered chunks
        sort(buffer.begin(), buffer.end(),
             [](const plan_env::msg::ChunkData& chunk1, const plan_env::msg::ChunkData& chunk2) { return chunk1.idx < chunk2.idx; });
        vector<int> idx_list = { int(buffer.front().idx) };
        int         last_idx = idx_list[0];
        for (int j = 1; j < buffer.size(); ++j)
        {
            if (buffer[j].idx - last_idx > 1)
            {
                idx_list.push_back(last_idx);
                idx_list.push_back(buffer[j].idx);
            }
            last_idx = buffer[j].idx;
        }
        idx_list.push_back(last_idx);

        // std::cout << "process drone " << i + 1 << "'s chunks, input idx list: ";
        // for (auto id : idx_list) std::cout << id << ", ";
        // std::cout << "" << std::endl;

        // Update ChunksData's chunks_
        auto& chunks_data = multi_map_chunks_[i];

        // std::cout << "self idx list " << i + 1 << ": ";
        // for (auto id : chunks_data.idx_list_) std::cout << id << ", ";
        // std::cout << "" << std::endl;

        // Add placeholder for chunks
        int len_inc = last_idx - static_cast<int>(chunks_data.chunks_.size());
        for (int j = 0; j < len_inc; ++j)
        {
            chunks_data.chunks_.push_back(MapChunk());
            auto& back_chunk       = chunks_data.chunks_.back();
            back_chunk.idx_        = chunks_data.chunks_.size();
            back_chunk.empty_      = true;
            back_chunk.need_query_ = false;
        }

        // Process data in buffer
        for (auto msg : buffer)
        {
            if (msg.idx <= 0 || msg.idx > static_cast<int>(chunks_data.chunks_.size()))
            {
                RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[chunkTimer] idx out of range: idx=%d chunk_size=%zu src_drone=%d", msg.idx,
                             chunks_data.chunks_.size(), msg.chunk_drone_id);
                continue;
            }
            auto& chunk = chunks_data.chunks_[msg.idx - 1];
            if (chunk.empty_)
            {  // Only insert a chunk once
                chunk.voxel_adrs_ = msg.voxel_adrs;
                chunk.voxel_occ   = msg.voxel_occ;
                insertChunkToMap(chunk, msg.chunk_drone_id, flipped_free_adrs);
                chunk.empty_ = false;
            }
        }

        // Update ChunksData's idx_list_
        vector<int> union_list;
        mergeChunkIds(idx_list, chunks_data.idx_list_, union_list);
        chunks_data.idx_list_ = union_list;

        // std::cout << "merged idx list " << i + 1 << ": ";
        // for (auto id : union_list) std::cout << id << ", ";
        // std::cout << "" << std::endl;

        buffer.clear();
        buffer_map_[i].clear();

        // 重发融合导致的 occupied→free 翻转体素：撤销本车在地面站的 occupied
        // 引用（防环：仅本地状态被本次融合改变的体素才进入此列表）。
        if (!flipped_free_adrs.empty())
            updateMapChunk(flipped_free_adrs);
    }
    // RCLCPP_ERROR("chunk time: %lf", (rclcpp::Clock().now() - t1).toSec());
}

void MultiMapManager::mergeChunkIds(const vector<int>& list1, const vector<int>& list2, vector<int>& output)
{
    if ((list1.size() % 2) != 0 || (list2.size() % 2) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[mergeChunkIds] invalid interval list size: list1=%zu list2=%zu", list1.size(), list2.size());
        output = list1;
        return;
    }

    // std::cout << "list1: ";
    // for (auto id : list1) std::cout << id << ", ";
    // std::cout << "" << std::endl;

    // std::cout << "list2: ";
    // for (auto id : list2) std::cout << id << ", ";
    // std::cout << "" << std::endl;

    if (list1.empty())
    {
        output = list2;
        return;
    }

    output = list1;
    int tmp1, tmp2;
    for (int i = 0; i < list2.size(); i += 2)
    {
        // For each interval in list2, merge it into output list
        bool intersect = false;
        for (int j = 0; j < output.size(); j += 2)
        {
            if (findIntersect(output[j], output[j + 1], list2[i], list2[i + 1], tmp1, tmp2))
            {
                output[j]     = min(output[j], list2[i]);
                output[j + 1] = max(output[j + 1], list2[i + 1]);
                intersect     = true;
            }
        }
        if (!intersect)
        {  // Insert the interval in appropriate position
            vector<int> tmp = { list2[i], list2[i + 1] };
            if (list2[i + 1] < output.front())
            {
                output.insert(output.begin(), tmp.begin(), tmp.end());
            }
            else if (list2[i] > output.back())
            {
                output.insert(output.end(), tmp.begin(), tmp.end());
            }
            else
            {
                for (auto iter = output.begin() + 1; iter != output.end(); iter += 2)
                {
                    if (*iter < list2[i] && *(iter + 1) > list2[i + 1])
                    {
                        output.insert(iter + 1, tmp.begin(), tmp.end());
                        break;
                    }
                }
            }
        }
        // Remove redundant idx
        for (auto iter = output.begin() + 1; iter != output.end() - 1;)
        {
            if (*iter >= *(iter + 1) - 1)
            {
                iter = output.erase(iter);
                iter = output.erase(iter);
            }
            else
            {
                iter += 2;
            }
        }
        // std::cout << "output: ";
        // for (auto id : output) std::cout << id << ", ";
        // std::cout << "" << std::endl;
    }
}

void MultiMapManager::adrToIndex(const uint32_t& adr, Eigen::Vector3i& idx)
{
    // x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z
    uint32_t  tmp_adr = adr;
    const int a       = map_->mp_->map_voxel_num_[1] * map_->mp_->map_voxel_num_[2];
    const int b       = map_->mp_->map_voxel_num_[2];

    idx[0]  = tmp_adr / a;
    tmp_adr = tmp_adr % a;
    idx[1]  = tmp_adr / b;
    idx[2]  = tmp_adr % b;
}

void MultiMapManager::insertChunkToMap(const MapChunk& chunk, const int& chunk_drone_id, vector<uint32_t>& flipped_free_adrs)
{
    if (chunk_drone_id <= 0 || chunk_drone_id > map_num_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[insertChunkToMap] invalid drone_id=%d map_num=%d", chunk_drone_id, map_num_);
        return;
    }
    if (chunk.voxel_occ.size() != chunk.voxel_adrs_.size())
    {
        RCLCPP_ERROR(rclcpp::get_logger("multi_map_manager"), "[insertChunkToMap] voxel size mismatch: adrs=%zu occ=%zu drone_id=%d", chunk.voxel_adrs_.size(),
                     chunk.voxel_occ.size(), chunk_drone_id);
    }
    const size_t voxel_count = std::min(chunk.voxel_adrs_.size(), chunk.voxel_occ.size());

    // // Transform from other drone's local frame to this drone's
    // Eigen::Vector4d transform;
    // map_->getBaseCoor(drone_id, transform);

    // double yaw = transform[3];
    // Eigen::Matrix3d rot;
    // rot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    // Eigen::Vector3d trans = transform.head<3>();

    for (size_t i = 0; i < voxel_count; ++i)
    {
        // Insert occ info

        auto& adr = chunk.voxel_adrs_[i];

        Eigen::Vector3i idx;
        adrToIndex(adr, idx);

        Eigen::Vector3d pos;
        map_->indexToPos(idx, pos);

        // pos = rot * pos + trans;
        if (!map_->isInMap(pos))
            continue;

        map_->posToIndex(pos, idx);
        auto adr_tf = map_->toAddress(idx);

        if (chunk.voxel_occ[i] == 1 && map_->isInDynamicAgentFootprint(pos))
            continue;

        // 融合 FREE 导致 occupied→free 翻转（如走开的小车足迹被队友确认清除）：
        // 收集翻转体素，由调用方经 updateMapChunk 重发，撤销本车在地面站的
        // occupied 引用；否则该引用只能等本车自己重扫才能清除。
        const bool was_occupied              = map_->md_->occupancy_buffer_[adr_tf] > map_->mp_->min_occupancy_log_;
        map_->md_->occupancy_buffer_[adr_tf] = chunk.voxel_occ[i] == 1 ? map_->mp_->clamp_max_log_ : map_->mp_->clamp_min_log_;
        if (chunk.voxel_occ[i] == 0 && was_occupied)
            flipped_free_adrs.push_back(adr_tf);

        // Update the chunk box

        if (chunk_boxes_[chunk_drone_id - 1].valid_)
        {
            for (int k = 0; k < 3; ++k)
            {
                chunk_boxes_[chunk_drone_id - 1].min_[k] = min(chunk_boxes_[chunk_drone_id - 1].min_[k], pos[k]);
                chunk_boxes_[chunk_drone_id - 1].max_[k] = max(chunk_boxes_[chunk_drone_id - 1].max_[k], pos[k]);
            }
        }
        else
        {
            chunk_boxes_[chunk_drone_id - 1].min_ = chunk_boxes_[chunk_drone_id - 1].max_ = pos;
            chunk_boxes_[chunk_drone_id - 1].valid_                                       = true;
        }

        // Update the all box
        for (int k = 0; k < 3; ++k)
        {
            map_->md_->all_min_[k] = min(map_->md_->all_min_[k], pos[k]);
            map_->md_->all_max_[k] = max(map_->md_->all_max_[k], pos[k]);
        }
        // Inflate for the occupied
        if (chunk.voxel_occ[i] == 1)
        {
            static const int inf_step = ceil(map_->mp_->obstacles_inflation_ / map_->mp_->resolution_);
            for (int inf_x = -inf_step; inf_x <= inf_step; ++inf_x)
                for (int inf_y = -inf_step; inf_y <= inf_step; ++inf_y)
                    for (int inf_z = -inf_step; inf_z <= inf_step; ++inf_z)
                    {
                        Eigen::Vector3i inf_pt(idx[0] + inf_x, idx[1] + inf_y, idx[2] + inf_z);
                        if (!map_->isInMap(inf_pt))
                            continue;
                        int inf_adr                                   = map_->toAddress(inf_pt);
                        map_->md_->occupancy_buffer_inflate_[inf_adr] = 1;
                    }
            // vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
            // map_->inflatePoint(idx, inf_step, inf_pts);

            // for (auto inf_pt : inf_pts) {
            //   if (!map_->isInMap(inf_pt)) continue;
            //   int idx_inf = map_->toAddress(inf_pt);
            //   if (idx_inf >= 0 &&
            //       idx_inf < map_->mp_->map_voxel_num_(0) * map_->mp_->map_voxel_num_(1) *
            //                     map_->mp_->map_voxel_num_(2)) {
            //     map_->md_->occupancy_buffer_inflate_[idx_inf] = 1;
            //   }
            // }
        }
    }
}

void MultiMapManager::getChunkBoxes(vector<Eigen::Vector3d>& mins, vector<Eigen::Vector3d>& maxs, bool reset)
{
    for (auto& box : chunk_boxes_)
    {
        if (box.valid_)
        {
            mins.push_back(box.min_);
            maxs.push_back(box.max_);
            if (reset)
                box.valid_ = false;
        }
    }
}

// MultiMapManager::
}  // namespace fast_planner