#ifndef PPU_PATH_CLIENT_H
#define PPU_PATH_CLIENT_H

/*
 * ppu_path_client.h — PPU 路径规划 TCP 客户端。
 *
 * 封装与 3403 通信的完整流程：
 *   setMap() 预处理并缓存地图
 *   plan()   打包起终点+地图 → TCP 发送 → 接收方向码 → 还原路径点序列
 *
 * 默认连接 192.168.137.62:19588。
 */

#include <Eigen/Dense>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "ppu_protocol.h"

namespace ppu_opt
{

struct GridMap
{
    uint32_t             size_x = 0, size_y = 0;
    std::vector<uint8_t> data;
};

struct PathPoint
{
    int32_t x = 0, y = 0;
    bool    operator==(const PathPoint& o) const
    {
        return x == o.x && y == o.y;
    }
};

struct PpuPathOptions
{
    std::string host{ "192.168.137.62" };
    uint16_t    port{ 19588 };
    uint32_t    first_seq{ 1 };
    int         timeout_ms{ 180000 };
};

/* 路径规划结果 */
struct PathPlanResult
{
    bool                      success{ false };
    bool                      busy{ false };  // 3403 返回 code=30：PPU 忙，本次请求未执行（连接保持）
    int32_t                   status{ 0 };    // PpuSnnPathStatus
    uint32_t                  seq{ 0 };
    std::vector<PathPoint>    points;
    std::chrono::microseconds roundtrip{ 0 };
    std::string               error_msg;
};

/** 地图预处理选项 */
struct PreprocessOptions
{
    int   obstacle_close_kernel{ 3 };
    float robot_radius{ 0.5f };
};

class PpuPathClient
{
public:
    explicit PpuPathClient(const PpuPathOptions& options = PpuPathOptions());
    ~PpuPathClient();

    PpuPathClient(const PpuPathClient&)            = delete;
    PpuPathClient& operator=(const PpuPathClient&) = delete;

    /** 预处理并缓存地图 */
    void setMap(const GridMap& map, const PreprocessOptions& opts = PreprocessOptions());

    /** 不带预处理的原始地图缓存 */
    void setRawMap(const GridMap& map)
    {
        cached_map_ = map;
    }

    /** 规划路径：起点(sx,sy) → 终点(gx,gy) */
    PathPlanResult plan(int32_t sx, int32_t sy, int32_t gx, int32_t gy);

    bool connected() const
    {
        return fd_ >= 0;
    }
    void disconnect();

    static std::vector<PathPoint> restorePath(const PpuSnnPathResult& result, const std::vector<uint8_t>& directions, const GridMap* processed_map = nullptr);
    static GridMap                preprocessMap(const GridMap& map, const PreprocessOptions& opts);

private:
    PpuPathOptions    options_;
    int               fd_{ -1 };
    uint32_t          next_seq_{ 0 };
    GridMap           cached_map_;
    PreprocessOptions preproc_opts_;
    bool              map_valid_{ false };

    void ensureConnected();
    void sendAll(const void* data, size_t len);
    void recvAll(void* data, size_t len);
};

}  // namespace ppu_opt

#endif  // PPU_PATH_CLIENT_H