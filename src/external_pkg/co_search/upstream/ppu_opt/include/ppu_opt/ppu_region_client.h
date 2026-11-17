#ifndef PPU_REGION_CLIENT_H
#define PPU_REGION_CLIENT_H

/*
 * ppu_region_client.h — PPU 区域分配 TCP 客户端。
 *
 * 封装与 3403 区域分配服务的通信：
 *   allocate() 打包无人机+区域数据 → TCP 发送 → 接收分配结果
 *
 * 默认连接 192.168.137.62:19589。
 */

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "ppu_protocol.h"

namespace ppu_opt
{

/* 区域布局信息 */
struct PpuRegionLayout
{
    int    rows{ 1 };
    int    cols{ 1 };
    int    total_regions{ 1 };
    double min_x{ 0.0 };
    double max_x{ 0.0 };
    double min_y{ 0.0 };
    double max_y{ 0.0 };
    double width{ 1.0 };
    double height{ 1.0 };
};

/* SNN 参数 */
struct PpuSnnParams
{
    double tau{ 2.5 };
    double dt{ 0.2 };
    double threshold{ 0.25 };
    int    max_iter{ 300 };
    double distance_weight{ 1.2 };
    double load_weight{ 0.8 };
};

/* 无人机输入 */
struct DroneInput
{
    int32_t drone_id;
    int32_t sticky_region{ -1 };
    double  pos_x{ 0.0 };
    double  pos_y{ 0.0 };
};

/* 区域输入 */
struct RegionInput
{
    int32_t load{ 0 };
    int32_t active_grid_count{ 0 };
    int32_t reachable_grid_count{ 0 };
};

/* 区域分配结果 */
struct RegionAllocResult
{
    bool                      success{ false };
    bool                      busy{ false };  // 3403 返回 code=30：PPU 忙，本次请求未执行（连接保持）
    int32_t                   status{ 0 };
    uint32_t                  seq{ 0 };
    std::vector<int32_t>      assignments;
    std::chrono::microseconds roundtrip{ 0 };
    std::string               error_msg;
};

struct PpuRegionOptions
{
    std::string host{ "192.168.137.62" };
    uint16_t    port{ 19589 };
    uint32_t    first_seq{ 1 };
    int         timeout_ms{ 60000 };
};

class PpuRegionClient
{
public:
    explicit PpuRegionClient(const PpuRegionOptions& options = PpuRegionOptions());
    ~PpuRegionClient();

    PpuRegionClient(const PpuRegionClient&)            = delete;
    PpuRegionClient& operator=(const PpuRegionClient&) = delete;

    /** 核心：打包 Drone/Region 数据 → TCP 发送 → 接收分配结果 */
    RegionAllocResult allocate(const PpuFeRegionHeader& header, const std::vector<DroneInput>& drones, const std::vector<RegionInput>& regions);

    bool connected() const
    {
        return fd_ >= 0;
    }
    void disconnect();

private:
    PpuRegionOptions options_;
    int              fd_{ -1 };
    uint32_t         next_seq_{ 0 };

    void ensureConnected();
    void sendAll(const void* data, size_t len);
    void recvAll(void* data, size_t len);
};

}  // namespace ppu_opt

#endif  // PPU_REGION_CLIENT_H