#include "ppu_opt/ppu_region_client.h"

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

    FdPtr tcpConnect(const PpuRegionOptions& o)
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

}  // anonymous namespace

// ── PpuRegionClient ──────────────────────────────────────

PpuRegionClient::PpuRegionClient(const PpuRegionOptions& options) : options_(options), next_seq_(options.first_seq)
{
    uint16_t probe = 1;
    if (*reinterpret_cast<uint8_t*>(&probe) != 1)
        throw std::runtime_error("PPU protocol requires little-endian host");
}

PpuRegionClient::~PpuRegionClient()
{
    disconnect();
}

void PpuRegionClient::disconnect()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

void PpuRegionClient::ensureConnected()
{
    if (fd_ >= 0)
        return;
    FdPtr fd = tcpConnect(options_);
    fd_      = *fd;
    *fd      = -1;
}

void PpuRegionClient::sendAll(const void* data, size_t len)
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

void PpuRegionClient::recvAll(void* data, size_t len)
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

RegionAllocResult PpuRegionClient::allocate(const PpuFeRegionHeader& header, const std::vector<DroneInput>& drones, const std::vector<RegionInput>& regions)
{
    RegionAllocResult out;
    auto              start_time = std::chrono::steady_clock::now();

    try
    {
        ensureConnected();

        uint32_t n_drones  = static_cast<uint32_t>(drones.size());
        uint32_t n_regions = static_cast<uint32_t>(regions.size());

        if (n_drones > PPU_FE_REGION_MAX_DRONES || n_regions > PPU_FE_REGION_MAX_REGIONS)
            throw std::invalid_argument("too many drones or regions");

        // 构建请求体
        PpuFeRegionHeader req_hdr = header;
        req_hdr.magic             = PPU_FE_REGION_MAGIC;
        req_hdr.version           = PPU_FE_REGION_VERSION;
        req_hdr.n_drones          = n_drones;
        req_hdr.n_regions         = n_regions;

        uint32_t body_bytes  = ppu_region_total_bytes(n_drones, n_regions);
        req_hdr.header_bytes = sizeof(PpuFeRegionHeader);
        req_hdr.total_bytes  = body_bytes;

        std::vector<uint8_t> body(body_bytes);
        std::memcpy(body.data(), &req_hdr, sizeof(PpuFeRegionHeader));

        // 写入无人机数据
        size_t offset = sizeof(PpuFeRegionHeader);
        for (uint32_t i = 0; i < n_drones; ++i)
        {
            PpuFeRegionDrone d{};
            d.drone_id      = drones[i].drone_id;
            d.sticky_region = drones[i].sticky_region;
            d.pos_x         = drones[i].pos_x;
            d.pos_y         = drones[i].pos_y;
            std::memcpy(body.data() + offset, &d, sizeof(PpuFeRegionDrone));
            offset += sizeof(PpuFeRegionDrone);
        }

        // 写入区域数据
        for (uint32_t i = 0; i < n_regions; ++i)
        {
            PpuFeRegionRegion r{};
            r.load                 = regions[i].load;
            r.active_grid_count    = regions[i].active_grid_count;
            r.reachable_grid_count = regions[i].reachable_grid_count;
            std::memcpy(body.data() + offset, &r, sizeof(PpuFeRegionRegion));
            offset += sizeof(PpuFeRegionRegion);
        }

        // 发送网络帧头 + 请求体
        const uint32_t seq = next_seq_++;
        RegionNetHdr   net_hdr{};
        net_hdr.magic      = REGION_NET_MAGIC;
        net_hdr.ver        = REGION_NET_VER;
        net_hdr.msg_type   = PPU_NET_MSG_REQ;
        net_hdr.flags      = 0;
        net_hdr.seq        = seq;
        net_hdr.body_bytes = body_bytes;
        net_hdr.reserved   = 0;

        sendAll(&net_hdr, sizeof(net_hdr));
        sendAll(body.data(), body.size());

        // 接收响应头
        RegionNetHdr rh{};
        recvAll(&rh, sizeof(rh));
        if (rh.magic != REGION_NET_MAGIC || rh.ver != REGION_NET_VER || rh.seq != seq)
            throw std::runtime_error("invalid response header");

        // 处理错误
        if (rh.msg_type == PPU_NET_MSG_ERR)
        {
            if (rh.body_bytes < sizeof(RegionErrorBody))
                throw std::runtime_error("invalid error body size");
            RegionErrorBody err_body{};
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
                out.error_msg = "3403 region error code=" + std::to_string(err_body.code) + " detail=" + std::to_string(err_body.detail);
            }
            auto end      = std::chrono::steady_clock::now();
            out.roundtrip = std::chrono::duration_cast<std::chrono::microseconds>(end - start_time);
            out.seq       = seq;
            return out;
        }

        if (rh.msg_type != PPU_NET_MSG_RSP || rh.body_bytes < sizeof(PpuRegionRspHeader))
            throw std::runtime_error("invalid response type/size");

        // 接收响应头
        PpuRegionRspHeader rsp_hdr{};
        recvAll(&rsp_hdr, sizeof(rsp_hdr));

        if (rsp_hdr.magic != PPU_FE_REGION_MAGIC || rsp_hdr.version != PPU_FE_REGION_VERSION)
            throw std::runtime_error("invalid region response magic");

        // 接收分配结果
        uint32_t assignment_count      = rsp_hdr.n_drones;
        uint32_t expected_assign_bytes = assignment_count * sizeof(int32_t);
        uint32_t remaining             = rh.body_bytes - static_cast<uint32_t>(sizeof(PpuRegionRspHeader));

        if (remaining != expected_assign_bytes)
            throw std::runtime_error("assignment count mismatch in response");

        out.assignments.resize(assignment_count);
        if (assignment_count > 0)
            recvAll(out.assignments.data(), expected_assign_bytes);

        out.seq     = seq;
        out.status  = rsp_hdr.status;
        out.success = (rsp_hdr.status == PPU_FE_REGION_OK);
    }
    catch (const std::exception& e)
    {
        out.success = false;
        if (out.error_msg.empty())
            out.error_msg = e.what();
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

}  // namespace ppu_opt