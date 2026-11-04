#include "ppu_opt/ppu_path_client.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <memory>
#include <netdb.h>
#include <netinet/tcp.h>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace ppu_opt
{
namespace
{

    constexpr int8_t   DX[9]      = { -1, 1, 0, 0, -1, -1, 1, 1, 0 };
    constexpr int8_t   DY[9]      = { 0, 0, -1, 1, -1, 1, -1, 1, 0 };
    constexpr uint32_t INPUT_CAP  = 819200;
    constexpr uint32_t OUTPUT_CAP = 800000;

    bool inside(int32_t x, int32_t y, uint32_t sx, uint32_t sy)
    {
        return x >= 0 && y >= 0 && static_cast<uint32_t>(x) < sx && static_cast<uint32_t>(y) < sy;
    }

    struct FdCloser
    {
        void operator()(int* p) const
        {
            if (p)
            {
                if (*p >= 0)
                    ::close(*p);
                delete p;
            }
        }
    };
    using FdPtr = std::unique_ptr<int, FdCloser>;

    FdPtr tcpConnect(const PpuPathOptions& o)
    {
        if (o.host.empty() || o.timeout_ms <= 0)
            throw std::invalid_argument("invalid host or timeout");

        addrinfo hints{};
        hints.ai_family   = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* list    = nullptr;
        int       rc      = getaddrinfo(o.host.c_str(), std::to_string(o.port).c_str(), &hints, &list);
        if (rc)
            throw std::runtime_error(std::string("getaddrinfo: ") + gai_strerror(rc));
        std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> guard(list, freeaddrinfo);

        int last_err = ECONNREFUSED;
        for (auto a = list; a; a = a->ai_next)
        {
            FdPtr fd(new int(::socket(a->ai_family, a->ai_socktype, a->ai_protocol)));
            if (*fd < 0)
            {
                last_err = errno;
                continue;
            }
            timeval tv{ o.timeout_ms / 1000, (o.timeout_ms % 1000) * 1000 };
            if (::setsockopt(*fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0 || ::setsockopt(*fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
            {
                last_err = errno;
                continue;
            }
            if (::connect(*fd, a->ai_addr, a->ai_addrlen) == 0)
            {
                int one = 1;
                ::setsockopt(*fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
                return fd;
            }
            last_err = errno;
        }
        errno = last_err;
        throw std::runtime_error(std::string("connect: ") + std::strerror(errno));
    }

    GridMap dilateObstacles(const GridMap& map, int radius)
    {
        if (radius <= 0)
            return map;
        GridMap   out{ map.size_x, map.size_y, map.data };
        const int sx = static_cast<int>(map.size_x);
        const int sy = static_cast<int>(map.size_y);
        for (int x = 0; x < sx; ++x)
        {
            for (int y = 0; y < sy; ++y)
            {
                if (map.data[static_cast<size_t>(x) * sy + y] == 0)
                    continue;
                for (int dx = -radius; dx <= radius; ++dx)
                {
                    for (int dy = -radius; dy <= radius; ++dy)
                    {
                        if (dx * dx + dy * dy > radius * radius)
                            continue;
                        int nx = x + dx, ny = y + dy;
                        if (nx >= 0 && nx < sx && ny >= 0 && ny < sy)
                            out.data[static_cast<size_t>(nx) * sy + ny] = 1;
                    }
                }
            }
        }
        return out;
    }

    GridMap closeObstacles(const GridMap& map, int kernel)
    {
        // 形态学闭运算：先膨胀后腐蚀。kernel 为正方形边长（格）。
        if (kernel < 3)
            return map;
        const int   half = kernel / 2;
        const int   sx   = static_cast<int>(map.size_x);
        const int   sy   = static_cast<int>(map.size_y);
        const auto& md   = map.data;
        GridMap     out{ map.size_x, map.size_y, map.data };
        for (int x = 0; x < sx; ++x)
        {
            for (int y = 0; y < sy; ++y)
            {
                // 检查 kernel 窗口是否全是障碍
                bool all_wall = true;
                for (int dx = -half; dx <= half && all_wall; ++dx)
                    for (int dy = -half; dy <= half && all_wall; ++dy)
                    {
                        int nx = x + dx, ny = y + dy;
                        if (nx >= 0 && nx < sx && ny >= 0 && ny < sy)
                        {
                            if (md[static_cast<size_t>(nx) * sy + ny] == 0)
                                all_wall = false;
                        }
                    }
                if (all_wall)
                {
                    for (int dx = -half; dx <= half; ++dx)
                        for (int dy = -half; dy <= half; ++dy)
                        {
                            int nx = x + dx, ny = y + dy;
                            if (nx >= 0 && nx < sx && ny >= 0 && ny < sy)
                                out.data[static_cast<size_t>(nx) * sy + ny] = 1;
                        }
                }
            }
        }
        return out;
    }

    GridMap erodeObstacles(const GridMap& map, int kernel)
    {
        if (kernel < 2)
            return map;
        const int sx   = static_cast<int>(map.size_x);
        const int sy   = static_cast<int>(map.size_y);
        const int half = kernel / 2;
        GridMap   out{ map.size_x, map.size_y, map.data };
        for (int x = 0; x < sx; ++x)
        {
            for (int y = 0; y < sy; ++y)
            {
                if (out.data[static_cast<size_t>(x) * sy + y] == 0)
                    continue;
                bool neighbor_free = false;
                for (int dx = -half; dx <= half && !neighbor_free; ++dx)
                    for (int dy = -half; dy <= half && !neighbor_free; ++dy)
                    {
                        int nx = x + dx, ny = y + dy;
                        if (nx >= 0 && nx < sx && ny >= 0 && ny < sy && map.data[static_cast<size_t>(nx) * sy + ny] == 0)
                            neighbor_free = true;
                    }
                if (neighbor_free)
                    out.data[static_cast<size_t>(x) * sy + y] = 0;
            }
        }
        return out;
    }

}  // anonymous namespace

// ── PpuPathClient ──────────────────────────────────────

PpuPathClient::PpuPathClient(const PpuPathOptions& options) : options_(options), next_seq_(options.first_seq)
{
    uint16_t probe = 1;
    if (*reinterpret_cast<uint8_t*>(&probe) != 1)
        throw std::runtime_error("PPU protocol requires little-endian host");
}

PpuPathClient::~PpuPathClient()
{
    disconnect();
}

void PpuPathClient::disconnect()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

void PpuPathClient::ensureConnected()
{
    if (fd_ >= 0)
        return;
    FdPtr fd = tcpConnect(options_);
    fd_      = *fd;
    *fd      = -1;  // transfer ownership
}

void PpuPathClient::sendAll(const void* data, size_t len)
{
    auto   p      = static_cast<const uint8_t*>(data);
    size_t offset = 0;
    while (offset < len)
    {
        ssize_t k = ::send(fd_, p + offset, len - offset, MSG_NOSIGNAL);
        if (k < 0)
        {
            if (errno == EINTR)
                continue;
            throw std::runtime_error(std::string("send: ") + std::strerror(errno));
        }
        if (k == 0)
            throw std::runtime_error("send: connection closed");
        offset += static_cast<size_t>(k);
    }
}

void PpuPathClient::recvAll(void* data, size_t len)
{
    auto   p      = static_cast<uint8_t*>(data);
    size_t offset = 0;
    while (offset < len)
    {
        ssize_t k = ::recv(fd_, p + offset, len - offset, 0);
        if (k < 0)
        {
            if (errno == EINTR)
                continue;
            throw std::runtime_error(std::string("recv: ") + std::strerror(errno));
        }
        if (k == 0)
            throw std::runtime_error("recv: connection closed");
        offset += static_cast<size_t>(k);
    }
}

GridMap PpuPathClient::preprocessMap(const GridMap& map, const PreprocessOptions& opts)
{
    // 验证输入
    uint64_t n = static_cast<uint64_t>(map.size_x) * map.size_y;
    if (!map.size_x || !map.size_y || n != map.data.size() || n > INPUT_CAP)
        throw std::invalid_argument("invalid map size");
    for (auto v : map.data)
        if (v > 1)
            throw std::invalid_argument("map values must be 0 or 1");

    // 闭运算填洞
    GridMap result = map;
    if (opts.obstacle_close_kernel >= 3)
        result = closeObstacles(result, opts.obstacle_close_kernel);

    // 膨胀车体半径（与 WavefrontPlanner::inflateGrid 一致：ceil 取整）
    int radius_cells = static_cast<int>(std::ceil(opts.robot_radius));
    if (radius_cells > 0)
        result = dilateObstacles(result, radius_cells);

    return result;
}

void PpuPathClient::setMap(const GridMap& map, const PreprocessOptions& opts)
{
    preproc_opts_ = opts;
    cached_map_   = preprocessMap(map, opts);
    map_valid_    = true;
}

PathPlanResult PpuPathClient::plan(int32_t sx, int32_t sy, int32_t gx, int32_t gy)
{
    PathPlanResult out;
    auto           start_time = std::chrono::steady_clock::now();

    try
    {
        if (!map_valid_ || cached_map_.data.empty())
            throw std::runtime_error("map not set");

        const auto& map   = cached_map_;
        uint32_t    cells = static_cast<uint32_t>(map.data.size());

        if (!inside(sx, sy, map.size_x, map.size_y) || !inside(gx, gy, map.size_x, map.size_y))
            throw std::invalid_argument("endpoint out of map bounds");

        ensureConnected();

        uint32_t cap  = OUTPUT_CAP - sizeof(PpuSnnPathResult);
        uint32_t maxp = std::min(cells, cap);
        if (maxp == 0 || maxp > cap)
            throw std::invalid_argument("invalid max_path_points");

        // 构造请求参数
        PpuSnnPathParams params{};
        params.magic           = PPU_SNN_PATH_MAGIC;
        params.version         = PPU_SNN_PATH_VERSION;
        params.reserved        = 0;
        params.map_version     = 1;
        params.size_x          = map.size_x;
        params.size_y          = map.size_y;
        params.start_x         = sx;
        params.start_y         = sy;
        params.goal_x          = gx;
        params.goal_y          = gy;
        params.max_wave_steps  = cells;
        params.max_path_points = maxp;
        params.map_bytes       = cells;
        params.output_capacity = OUTPUT_CAP;

        // 构造网络帧头
        const uint32_t  seq = next_seq_++;
        SplitPathNetHdr hdr{};
        hdr.magic      = SPLIT_PATH_NET_MAGIC;
        hdr.ver        = SPLIT_PATH_NET_VER;
        hdr.msg_type   = PPU_NET_MSG_REQ;
        hdr.flags      = 0;
        hdr.seq        = seq;
        hdr.body_bytes = static_cast<uint32_t>(sizeof(params) + cells);
        hdr.reserved   = 0;

        // 发送：头 + 参数 + 地图
        sendAll(&hdr, sizeof(hdr));
        sendAll(&params, sizeof(params));
        sendAll(map.data.data(), map.data.size());

        // 接收响应头
        SplitPathNetHdr rh{};
        recvAll(&rh, sizeof(rh));
        if (rh.magic != SPLIT_PATH_NET_MAGIC || rh.ver != SPLIT_PATH_NET_VER || rh.seq != seq)
            throw std::runtime_error("invalid response header");

        // 处理错误响应
        if (rh.msg_type == PPU_NET_MSG_ERR)
        {
            if (rh.body_bytes != sizeof(SplitPathErrorBody))
                throw std::runtime_error("invalid error body size");
            SplitPathErrorBody err_body{};
            recvAll(&err_body, sizeof(err_body));
            out.status = static_cast<int32_t>(err_body.code);
            if (err_body.code == PPU_NET_ERR_BUSY)
            {
                // PPU 忙：不销毁长连接，由上层本次改用本地计算兜底
                out.busy      = true;
                out.error_msg = "3403 busy code=30 detail=" + std::to_string(err_body.detail);
            }
            else
            {
                out.error_msg = "3403 error code=" + std::to_string(err_body.code) + " detail=" + std::to_string(err_body.detail);
            }
            auto end_time = std::chrono::steady_clock::now();
            out.roundtrip = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            out.seq       = seq;
            return out;
        }

        if (rh.msg_type != PPU_NET_MSG_RSP || rh.body_bytes < sizeof(PpuSnnPathResult))
            throw std::runtime_error("invalid response type/size");

        // 接收结果
        PpuSnnPathResult result{};
        recvAll(&result, sizeof(result));
        uint32_t direction_bytes = rh.body_bytes - static_cast<uint32_t>(sizeof(result));

        if (result.magic != PPU_SNN_PATH_MAGIC || result.version != PPU_SNN_PATH_RESULT_VERSION)
            throw std::runtime_error("invalid PPU result magic/version");

        std::vector<uint8_t> directions(direction_bytes);
        if (direction_bytes > 0)
            recvAll(directions.data(), direction_bytes);

        out.seq    = seq;
        out.status = result.status;

        if (result.status == PPU_SNN_PATH_OK)
        {
            out.points  = restorePath(result, directions, &map);
            out.success = true;
        }
        else
        {
            out.error_msg = "PPU path planning failed with status=" + std::to_string(result.status);
        }
    }
    catch (const std::exception& e)
    {
        out.success = false;
        if (out.error_msg.empty())
            out.error_msg = e.what();
        // 连接断开时重置 fd
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    out.roundtrip = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    return out;
}

std::vector<PathPoint> PpuPathClient::restorePath(const PpuSnnPathResult& result, const std::vector<uint8_t>& directions, const GridMap* processed_map)
{
    if (result.magic != PPU_SNN_PATH_MAGIC || result.version != PPU_SNN_PATH_RESULT_VERSION || result.status != PPU_SNN_PATH_OK)
        throw std::runtime_error("invalid/non-success PPU result");

    if (result.direction_count != directions.size() || result.point_count != result.direction_count + 1)
        throw std::runtime_error("PPU point/direction count mismatch");

    if (!inside(result.effective_start_x, result.effective_start_y, result.size_x, result.size_y)
        || !inside(result.effective_goal_x, result.effective_goal_y, result.size_x, result.size_y))
        throw std::runtime_error("effective endpoint out of bounds");

    if (processed_map && (processed_map->size_x != result.size_x || processed_map->size_y != result.size_y))
        throw std::runtime_error("result/map dimension mismatch");

    if (std::find(directions.begin(), directions.end(), uint8_t{ 8 }) != directions.end()
        && (directions.size() != 1 || result.effective_start_x != result.effective_goal_x || result.effective_start_y != result.effective_goal_y))
        throw std::runtime_error("invalid stationary direction");

    std::vector<PathPoint> points;
    points.reserve(result.point_count);
    PathPoint q{ result.effective_start_x, result.effective_start_y };
    points.push_back(q);

    for (size_t i = 0; i < directions.size(); ++i)
    {
        uint8_t d = directions[i];
        if (d > 8)
            throw std::runtime_error("invalid PPU direction at step " + std::to_string(i));

        PathPoint prev = q;
        q.x += DX[d];
        q.y += DY[d];

        if (!inside(q.x, q.y, result.size_x, result.size_y))
            throw std::runtime_error("path out of bounds at step " + std::to_string(i));

        if (processed_map)
        {
            auto&  md  = processed_map->data;
            size_t idx = static_cast<size_t>(q.x) * result.size_y + static_cast<size_t>(q.y);
            if (md[idx])
                throw std::runtime_error("path crosses obstacle at step " + std::to_string(i));
            if (d >= 4 && d < 8)
            {
                size_t pidx = static_cast<size_t>(q.x) * result.size_y + static_cast<size_t>(prev.y);
                size_t nidx = static_cast<size_t>(prev.x) * result.size_y + static_cast<size_t>(q.y);
                if (md[pidx] || md[nidx])
                    throw std::runtime_error("path cuts blocked corner at step " + std::to_string(i));
            }
        }
        points.push_back(q);
    }

    if (!(q == PathPoint{ result.effective_goal_x, result.effective_goal_y }))
        throw std::runtime_error("restored path does not reach effective_goal");

    return points;
}

}  // namespace ppu_opt