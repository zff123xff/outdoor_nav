#ifndef PPU_PROTOCOL_H
#define PPU_PROTOCOL_H

/*
 * ppu_protocol.h — 路径规划 + 区域分配 TCP 帧协议定义。
 *
 * 与 PPU/wave/proto/path_messages.h + PPU/region/3588_region/proto/region_messages.h
 * 同构：24 字节网络帧头 + 应用层负载。
 *
 * ── 路径规划 ──
 *   REQ body = [FeSnnPathParams 56B] [map_binary]
 *   RSP body = [FeSnnPathResult 64B] [direction_codes]
 *   ERR body = [SplitPathErrorBody 8B]
 *
 * ── 区域分配 ──
 *   REQ body = [FeRegionHeader 128B] [FeRegionDrone × n] [FeRegionRegion × nr]
 *   RSP body = [RegionRspHeader 24B] [int32 × n]
 *   ERR body = [RegionErrorBody 8B]
 *
 * 所有多字节字段均为小端序。
 */

#include <cstddef>
#include <cstdint>

#pragma pack(push, 1)

namespace ppu_opt
{

// ═══════════════════════════════════════════════════
// 1. 网络帧头（路径规划 + 区域分配共用同构格式）
// ═══════════════════════════════════════════════════

/* 路径规划网络层魔数（已与 PPU 侧确认）。
 * 小端内存序下字节为 'S','S','3','P'，即 ASCII "SS3P"。 */
#define SPLIT_PATH_NET_MAGIC 0x50335353u /* ASCII: SS3P */
#define SPLIT_PATH_NET_VER   2u

/* 区域分配网络层魔数（已与 PPU 侧确认）。 */
#define REGION_NET_MAGIC 0x52334752u /* ASCII: RG3R */
#define REGION_NET_VER   1u

/* 路径规划请求保留标志位：预留给 FeSnnPathProfile 性能剖析段 */
#define SPLIT_PATH_FLAG_PROFILE 0x01u

/* 消息类型。路径/区域协议分别定义为 SPLIT_PATH_MSG_* / REGION_MSG_*，取值相同。 */
enum
{
    PPU_NET_MSG_REQ = 1u, /* SPLIT_PATH_MSG_REQ / REGION_MSG_REQ */
    PPU_NET_MSG_RSP = 2u, /* SPLIT_PATH_MSG_RSP / REGION_MSG_RSP */
    PPU_NET_MSG_ERR = 3u  /* SPLIT_PATH_MSG_ERR / REGION_MSG_ERR */
};

/* PPU 忙错误码（ERR 帧 PpuErrorBody.code，detail=0）。
 * 3403 统一服务中 PPU 只有一个执行 worker：当 PPU 被其他算法占用或队列已满时，
 * 服务端立即返回该错误而不是排队等待（见《3588 端接口调用文档》§7）。
 * 客户端收到该错误时应视为“PPU 忙”：不销毁长连接，本次请求改由本地计算兜底，
 * 后续调用可直接重新尝试 PPU。 */
#define PPU_NET_ERR_BUSY 30u

/* 通用网络帧头（24B）。路径规划使用 SplitPathNetHdr 特化，布局完全相同。 */
struct PpuNetHdr
{
    uint32_t magic;    /* 协议魔数 */
    uint16_t ver;      /* 协议版本 */
    uint16_t msg_type; /* 1=REQ, 2=RSP, 3=ERR */
    uint32_t flags;
    uint32_t seq;        /* 请求序号 */
    uint32_t body_bytes; /* 负载字节数（不含此头） */
    uint32_t reserved;
};
static_assert(sizeof(PpuNetHdr) == 24u, "PpuNetHdr size mismatch");

/* 为保持与原始 path_messages.h 兼容的别名 */
using SplitPathNetHdr = PpuNetHdr;
using RegionNetHdr    = PpuNetHdr;

/* 通用错误帧负载（8B） */
struct PpuErrorBody
{
    uint32_t code;
    int32_t  detail;
};
static_assert(sizeof(PpuErrorBody) == 8u, "PpuErrorBody size mismatch");

using SplitPathErrorBody = PpuErrorBody;
using RegionErrorBody    = PpuErrorBody;

/* 路径规划服务器错误码（SplitPathErrorBody.code），与 PPU 侧确认值一致 */
enum SplitPathServerError
{
    SPLIT_PATH_ERR_BAD_HEADER     = 1,
    SPLIT_PATH_ERR_BAD_MSG_TYPE   = 2,
    SPLIT_PATH_ERR_BAD_BODY_SIZE  = 3,
    SPLIT_PATH_ERR_BAD_MAP_SIZE   = 4,
    SPLIT_PATH_ERR_OUT_OF_BOUNDS  = 5,
    SPLIT_PATH_ERR_BAD_OUTPUT_CAP = 6,
    SPLIT_PATH_ERR_BAD_RESERVED   = 7,
    SPLIT_PATH_ERR_BAD_MAP_VALUE  = 8,
    SPLIT_PATH_ERR_PPU_OPEN       = 10,
    SPLIT_PATH_ERR_PPU_RUN        = 11,
    SPLIT_PATH_ERR_PPU_RESULT     = 12,
    SPLIT_PATH_ERR_INTERNAL       = 20,
    SPLIT_PATH_ERR_PPU_BUSY       = 30 /* 同 PPU_NET_ERR_BUSY */
};

/* 区域分配服务器错误码（RegionErrorBody.code），与 PPU 侧确认值一致 */
enum RegionServerError
{
    REGION_ERR_BAD_HEADER    = 1,
    REGION_ERR_BAD_MSG_TYPE  = 2,
    REGION_ERR_BAD_BODY_SIZE = 3,
    REGION_ERR_BAD_ABI       = 4,
    REGION_ERR_BAD_SHAPE     = 5,
    REGION_ERR_BAD_REGION    = 6,
    REGION_ERR_PPU_OPEN      = 10,
    REGION_ERR_PPU_RUN       = 11,
    REGION_ERR_PPU_RESULT    = 12,
    REGION_ERR_INTERNAL      = 20,
    REGION_ERR_PPU_BUSY      = 30 /* 同 PPU_NET_ERR_BUSY */
};

// ═══════════════════════════════════════════════════
// 2. 路径规划应用层协议（fe_snn_path.h 同名结构体）
// ═══════════════════════════════════════════════════

/* PPU 固件魔数 */
#define PPU_SNN_PATH_MAGIC          0x48544150u /* "PATH" */
#define PPU_SNN_PATH_VERSION        2u
#define PPU_SNN_PATH_RESULT_VERSION 1u

enum PpuSnnPathStatus
{
    PPU_SNN_PATH_OK                = 0,
    PPU_SNN_PATH_ERR_BAD_PARAM     = -1,
    PPU_SNN_PATH_ERR_OUT_OF_BOUNDS = -2,
    PPU_SNN_PATH_ERR_START_BLOCKED = -3,
    PPU_SNN_PATH_ERR_GOAL_BLOCKED  = -4,
    PPU_SNN_PATH_ERR_NO_PATH       = -5,
    PPU_SNN_PATH_ERR_OUTPUT_LIMIT  = -7,
    PPU_SNN_PATH_ERR_SCRATCH_LIMIT = -8
};

/* 路径规划请求参数（56B） */
struct PpuSnnPathParams
{
    uint32_t magic;   /* PPU_SNN_PATH_MAGIC */
    uint32_t version; /* PPU_SNN_PATH_VERSION */
    uint32_t reserved;
    uint32_t map_version;
    uint32_t size_x;  /* 地图宽（格数） */
    uint32_t size_y;  /* 地图高（格数） */
    int32_t  start_x; /* 起点栅格坐标 */
    int32_t  start_y;
    int32_t  goal_x; /* 终点栅格坐标 */
    int32_t  goal_y;
    uint32_t max_wave_steps;  /* BFS 搜索步数上限 */
    uint32_t max_path_points; /* 路径点上限 */
    uint32_t map_bytes;       /* 地图数据字节数 = size_x × size_y */
    uint32_t output_capacity; /* 输出缓冲区容量 */
};
static_assert(sizeof(PpuSnnPathParams) == 56u, "PpuSnnPathParams size mismatch");

/* 路径规划结果（64B），后接 direction_count 字节的方向码 */
struct PpuSnnPathResult
{
    uint32_t magic;   /* PPU_SNN_PATH_MAGIC */
    uint32_t version; /* PPU_SNN_PATH_RESULT_VERSION */
    int32_t  status;  /* PPU_SNN_PATH_OK 或错误码 */
    uint32_t flags;
    uint32_t size_x;
    uint32_t size_y;
    int32_t  requested_start_x;
    int32_t  requested_start_y;
    int32_t  requested_goal_x;
    int32_t  requested_goal_y;
    int32_t  effective_start_x; /* PPU 实际规划的起点 */
    int32_t  effective_start_y;
    int32_t  effective_goal_x; /* PPU 实际规划的终点 */
    int32_t  effective_goal_y;
    uint32_t point_count;     /* 路径点总数 = direction_count + 1 */
    uint32_t direction_count; /* 方向码个数 */
};
static_assert(sizeof(PpuSnnPathResult) == 64u, "PpuSnnPathResult size mismatch");

// ═══════════════════════════════════════════════════
// 3. 区域分配应用层协议（fe_region.h + region_messages.h）
// ═══════════════════════════════════════════════════

/* PPU 固件魔数 */
#define PPU_FE_REGION_MAGIC                       0x4c414752u /* "RGAL" */
#define PPU_FE_REGION_VERSION                     1u
#define PPU_FE_REGION_FLAG_USE_SNN                0x1u
#define PPU_FE_REGION_FLAG_ACTIVE_COUNTS_VALID    0x2u
#define PPU_FE_REGION_FLAG_REACHABLE_COUNTS_VALID 0x4u
#define PPU_FE_REGION_MAX_DRONES                  16u
#define PPU_FE_REGION_MAX_REGIONS                 16u

enum PpuRegionStatus
{
    PPU_FE_REGION_OK                      = 0,
    PPU_FE_REGION_ERR_HEADER              = -1,
    PPU_FE_REGION_ERR_SHAPE               = -2,
    PPU_FE_REGION_ERR_ALGO                = -3,
    PPU_FE_REGION_ERR_NO_REACHABLE_REGION = -4
};

/* 区域分配请求头（128B） */
struct PpuFeRegionHeader
{
    uint32_t magic;
    uint32_t version;
    uint32_t header_bytes;
    uint32_t total_bytes;
    uint32_t algo;
    uint32_t flags;
    uint32_t n_drones;
    uint32_t n_regions;
    uint32_t rows;
    uint32_t cols;
    uint32_t max_iter;
    uint32_t reserved_u32;

    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double tau;
    double dt;
    double threshold;
    double distance_weight;
    double load_weight;
    double reserved_f64;
};
static_assert(sizeof(PpuFeRegionHeader) == 128u, "PpuFeRegionHeader size mismatch");

/* 无人机输入（24B） */
struct PpuFeRegionDrone
{
    int32_t drone_id;
    int32_t sticky_region; /* -1 = 无历史分配 */
    double  pos_x;
    double  pos_y;
};
static_assert(sizeof(PpuFeRegionDrone) == 24u, "PpuFeRegionDrone size mismatch");

/* 区域状态（12B） */
struct PpuFeRegionRegion
{
    int32_t load; /* 当前占用数 */
    int32_t active_grid_count;
    int32_t reachable_grid_count;
};
static_assert(sizeof(PpuFeRegionRegion) == 12u, "PpuFeRegionRegion size mismatch");

/* 区域分配响应头（24B） */
struct PpuRegionRspHeader
{
    uint32_t magic;   /* PPU_FE_REGION_MAGIC */
    uint32_t version; /* PPU_FE_REGION_VERSION */
    int32_t  status;
    uint32_t n_drones;
    uint32_t n_regions;
    uint32_t reserved;
};
static_assert(sizeof(PpuRegionRspHeader) == 24u, "PpuRegionRspHeader size mismatch");

}  // namespace ppu_opt

#pragma pack(pop)

namespace ppu_opt
{

/* input.bin 总字节数计算 */
inline uint32_t ppu_region_total_bytes(uint32_t n_drones, uint32_t n_regions)
{
    return static_cast<uint32_t>(sizeof(PpuFeRegionHeader) + n_drones * sizeof(PpuFeRegionDrone) + n_regions * sizeof(PpuFeRegionRegion));
}

}  // namespace ppu_opt

#endif  // PPU_PROTOCOL_H