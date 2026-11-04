#ifndef _MULTI_MAP_MANAGER_H
#define _MULTI_MAP_MANAGER_H

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <msg_set/msg/drone_state.hpp>
#include <plan_env/msg/chunk_data.hpp>
#include <plan_env/msg/chunk_stamps.hpp>
#include <plan_env/msg/idx_list.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cstdint>
#include <deque>
#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using std::shared_ptr;
using std::unordered_map;
using std::vector;

namespace fast_planner
{
class SDFMap;
class MapROS;

// A map chunk, the elementary exchange unit between robots
struct MapChunk
{
    uint32_t         idx_;  // Start from 1
    vector<uint32_t> voxel_adrs_;
    vector<uint8_t>  voxel_occ;

    bool need_query_;
    bool empty_;

    double created_time_{ 0.0 };

    // Spatial bounding box of the chunk (plain arrays to avoid Eigen alignment
    // issues inside std::vector<MapChunk>). Computed once at chunk creation and
    // used for relevance-gated sending (④ spatial gating).
    double box_min_[3] = { 0.0, 0.0, 0.0 };
    double box_max_[3] = { 0.0, 0.0, 0.0 };
    bool   box_valid_{ false };
};

struct ChunksData
{
    vector<MapChunk> chunks_;
    vector<int>      idx_list_;
};

struct ChunksBox
{
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
    bool            valid_;
};

struct PendingChunkSend
{
    int chunk_drone_id;
    int to_drone_id;
    int chunk_idx;
};

// Per-peer send queues: hot_ = chunks the peer needs soon (relevance-gated),
// cold_ = chunks that can wait for idle bandwidth. LIFO within each queue keeps
// the freshest data first (② per-peer queues + per-peer LIFO).
struct PeerSendQueues
{
    std::deque<PendingChunkSend> hot_;
    std::deque<PendingChunkSend> cold_;
};

// Peer coverage (explored) box, received on the fast drone_state channel (③).
// Sender fills its all_min_/all_max_; receivers store it with their own receipt
// time so no clock sync is required.
struct PeerCoveredBox
{
    double min_[3] = { 0.0, 0.0, 0.0 };
    double max_[3] = { 0.0, 0.0, 0.0 };
    bool   valid_{ false };
};

class MultiMapManager
{
public:
    MultiMapManager();
    ~MultiMapManager();
    void setMap(SDFMap* map);
    void init();

    void updateMapChunk(const vector<uint32_t>& adrs);
    void getChunkBoxes(vector<Eigen::Vector3d>& mins, vector<Eigen::Vector3d>& maxs, bool reset = true);

    // ③ is the point inside a fresh peer coverage box (anti-clustering signal)?
    //    Consumed by the planner's target-selection penalty.
    bool inPeerCoveredBox(const Eigen::Vector3d& p, double margin) const;

private:
    void                                                   sendChunks(const int& chunk_drone_id, const int& to_drone_id, const vector<int>& idx_list);
    void                                                   enqueueChunkSend(int chunk_drone_id, int to_drone_id, int chunk_idx);
    void                                                   drainSendQueue();
    bool                                                   publishChunkMessage(int chunk_drone_id, int to_drone_id, int chunk_idx);
    rclcpp::Publisher<plan_env::msg::ChunkData>::SharedPtr getChunkPublisher(int to_drone_id);
    static uint64_t                                        sendQueueKey(int chunk_drone_id, int to_drone_id, int chunk_idx);
    void                                                   getOccOfChunk(const vector<uint32_t>& adrs, vector<uint8_t>& occs);
    // 动态车体足迹过滤：仅作用于可视化流。体素被标记为队友车体命中（dynamic_hit_flag_）
    // 或落点位于任一队友当前位置附近时，把 occ 写为 0（free），地面站不显示车体痕迹；
    // swarm 互发通道不受影响，其他车融合到的仍是真实占用，规划避让语义不变。
    void filterDynamicFootprintOcc(const MapChunk& chunk, vector<uint8_t>& occs);
    // 融合 chunk 进本地地图；FREE 融合导致 occupied→free 翻转的体素收集进
    // flipped_free_adrs，由调用方经 updateMapChunk 重发（撤销本车地面站引用）。
    void insertChunkToMap(const MapChunk& chunk, const int& chunk_drone_id, vector<uint32_t>& flipped_free_adrs);
    void adrToIndex(const uint32_t& adr, Eigen::Vector3i& idx);
    // 将 adr_buffer_ 切分为 chunk；force=true 时即使不满 chunk_size_ 也切分，
    // 用于及时发布占用翻转（occupied<->free）体素的更新。
    void cutPendingChunks(bool force);
    // adr_buffer_ 长时间无法凑满 chunk_size_ 时强制切分，避免翻转更新滞留。
    void flushPendingChunkIfStale();

    void stampTimerCallback();
    void chunkTimerCallback();
    void stampMsgCallback(const plan_env::msg::ChunkStamps::ConstSharedPtr& msg);
    void chunkCallback(const plan_env::msg::ChunkData::ConstSharedPtr& msg);
    void droneStateCallback(const msg_set::msg::DroneState::SharedPtr msg);

    // Visualization chunk publishing (for ground station)
    void publishVisualizationChunk(const MapChunk& chunk);
    void publishVisualizationHeartbeat();
    void drainVisualizationReplay();

    // Distance-based hierarchical sync control
    double horizontalDistanceToPeer(int peer_id) const;
    double peerMinStampInterval(int peer_id) const;
    int    peerMaxChunksPerTick(int peer_id) const;

    // ④ relevance-gated sending: is this chunk needed soon by the target peer?
    bool chunkRelevantToPeer(const int chunk_drone_id, const int chunk_idx, const int to_drone_id) const;

    // Cold-start throttle: caps the number of chunks enqueued per stamp round,
    // limiting the MB burst while preserving hot/cold priority routing.
    static constexpr int kSendWindowSize = 10;

    // Operations on the chunk idx list
    void findMissedChunkIds(const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids);
    bool findIntersect(const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr);
    void mergeChunkIds(const vector<int>& list1, const vector<int>& list2, vector<int>& output);

    // data----------------

    int drone_id_, map_num_;
    int vis_drone_id_;
    int chunk_size_;
    int stamp_period_ms_;
    int chunk_timer_period_ms_;
    int tier_near_max_chunks_per_tick_;

    // Distance-tiered rate limiting parameters
    double tier_near_distance_m_;
    double tier_mid_distance_m_;
    double tier_far_distance_m_;
    double tier_near_interval_s_;
    double tier_mid_interval_s_;
    double tier_far_interval_s_;
    int    tier_mid_max_chunks_per_tick_;
    int    tier_far_max_chunks_per_tick_;
    double peer_position_timeout_s_;

    // ① stamp is cheap: fixed fast interval for discovering missing chunks; only
    //   the data send (peerMaxChunksPerTick / max_send_per_tick_) is throttled.
    double tier_stamp_interval_s_{ 0.5 };
    // ② global per-tick send budget (bandwidth guard); per-peer budgets are
    //   enforced inside drainSendQueue with nearest-peer priority.
    int max_send_per_tick_{ 4 };
    // ④ chunks closer than this to a peer are "hot"; farther ones are deferred.
    double send_relevance_radius_m_{ 40.0 };
    // ③ freshness window for peer coverage boxes (local receipt time based).
    double cov_box_timeout_s_{ 3.0 };
    // ④ optionally publish to a single shared chunk topic instead of per-peer
    //   unicast topics. Transport-dependent benefit; keep false unless the
    //   network does true multicast / relay dedup.
    bool shared_chunk_topic_{ false };

    std::vector<PeerSendQueues>                                                     send_queues_;  // indexed by to_drone_id (1..map_num_)
    std::unordered_set<uint64_t>                                                    send_queue_keys_;
    std::unordered_map<int, rclcpp::Publisher<plan_env::msg::ChunkData>::SharedPtr> chunk_pubs_;
    rclcpp::Publisher<plan_env::msg::ChunkData>::SharedPtr                          shared_chunk_pub_;

    std::vector<PeerCoveredBox> peer_cov_boxes_;
    std::vector<double>         peer_cov_box_stamps_;

    // Visualization chunk publishing members
    bool        visualization_chunk_enable_{ false };
    int         visualization_replay_max_chunks_per_tick_{ 10 };
    double      visualization_chunk_delay_s_{ 2.0 };
    double      visualization_status_interval_s_{ 1.0 };
    double      last_visualization_status_time_{ 0.0 };
    size_t      visualization_replay_next_idx_{ 0 };
    size_t      last_visualization_subscription_count_{ 0 };
    bool        visualization_replay_active_{ false };
    std::string visualization_chunk_topic_;

    // adr_buffer_ 超时强制切分（占用翻转体素及时送达地面站）
    double chunk_flush_timeout_s_{ 1.0 };
    double last_chunk_cut_time_seconds_{ 0.0 };

    rclcpp::Publisher<plan_env::msg::ChunkData>::SharedPtr chunk_visualization_pub_;

    SDFMap* map_;

public:
    rclcpp::Node::SharedPtr                                       node_;
    rclcpp::Publisher<plan_env::msg::ChunkStamps>::SharedPtr      stamp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<plan_env::msg::ChunkStamps>::SharedPtr   stamp_sub_;
    rclcpp::Subscription<plan_env::msg::ChunkData>::SharedPtr     chunk_sub_;
    rclcpp::Subscription<msg_set::msg::DroneState>::SharedPtr     drone_state_sub_;
    rclcpp::TimerBase::SharedPtr                                  stamp_timer_;
    rclcpp::TimerBase::SharedPtr                                  chunk_timer_;

    vector<ChunksData>                       multi_map_chunks_;
    vector<uint32_t>                         adr_buffer_;
    std::unordered_set<uint32_t>             pending_adr_keys_;
    vector<vector<plan_env::msg::ChunkData>> chunk_buffer_;
    vector<unordered_map<int, char>>         buffer_map_;
    vector<double>                           last_chunk_stamp_time_;
    vector<double>                           last_chunk_stamp_rx_time_;  // local receipt times (① stamp gate)
    vector<Eigen::Vector3d>                  peer_positions_;
    vector<double>                           peer_position_stamps_;

    vector<ChunksBox> chunk_boxes_;

    vector<Eigen::Vector3i> tmp_ids_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    Eigen::Vector3d drone_pos_;  // Make accessible from MapROS
};

}  // namespace fast_planner

#endif
