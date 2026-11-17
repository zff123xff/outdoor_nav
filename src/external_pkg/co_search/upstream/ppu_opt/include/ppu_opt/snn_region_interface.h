#ifndef SNN_REGION_INTERFACE_H
#define SNN_REGION_INTERFACE_H

/*
 * snn_region_interface.h — 区域分配器的抽象接口。
 *
 * 提供统一接口 IRegionAllocator，和一个工厂函数 createRegionAllocator()，
 * 根据 ROS2 参数 "ppu/enable_region_allocation" 动态选择：
 *   - true  → PpuRegionAllocator（优先远程 PPU；PPU 忙 code=30 时本次调用回退本地 SNN）
 *   - false → LocalRegionAllocator（本地 SnnRegionAllocator，默认）
 */

#include <Eigen/Eigen>
#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ppu_region_client.h"

namespace fast_planner
{

/** 与现有 SnnRegionAllocator::RegionLayout 兼容 */
struct RegionLayout
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

class IRegionAllocator
{
public:
    virtual ~IRegionAllocator() = default;

    virtual void configure(rclcpp::Node::SharedPtr node, double map_size_x, double map_size_y, int total_drones) = 0;

    virtual const RegionLayout& layout() const = 0;

    /** 分配区域：提交 sticky state / 负载更新 */
    virtual std::vector<int> assignDroneRegions(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions) = 0;

    /** 计算区域建议（不修改 sticky state），供共识仲裁 */
    virtual std::vector<int> computeDroneRegions(const std::vector<int>& drone_ids, const std::vector<Eigen::Vector3d>& positions) = 0;

    virtual int ownerRegionOfPoint(const Eigen::Vector3d& point) const       = 0;
    virtual int ownerRegionOfGridCenter(const Eigen::Vector3d& center) const = 0;

    virtual void getRegionBounds(int region_id, double& min_x, double& max_x, double& min_y, double& max_y) const = 0;

    virtual bool getStickyRegionForDrone(int drone_id, int& region_id) const = 0;
    virtual void setStickyAssignment(int drone_id, int region_id)            = 0;
    virtual void clearSticky()                                               = 0;

    /** 用共识结果覆盖 sticky + 负载计数 */
    virtual void applyConsensusAssignments(const std::vector<int>& assignments) = 0;

    /** 动态重配置区域布局 */
    virtual void reconfigureDynamic(int total_regions, double min_x, double max_x, double min_y, double max_y) = 0;

    /** 设置每区域活跃栅格数（用于负载归一化） */
    virtual void setRegionActiveGridCounts(const std::vector<int>& counts)    = 0;
    virtual void setRegionReachableGridCounts(const std::vector<int>& counts) = 0;
};

/**
 * 工厂函数：读取 node param "ppu/enable_region_allocation"
 *   true  → 创建 PpuRegionAllocator（优先 PpuRegionClient 远程调用 PPU；
 *            PPU 忙（3403 code=30）时本次调用自动回退本地 SnnRegionAllocator）
 *   false → 创建 LocalRegionAllocator（使用本地 SnnRegionAllocator，默认）
 */
std::shared_ptr<IRegionAllocator> createRegionAllocator(rclcpp::Node::SharedPtr node);

}  // namespace fast_planner

#endif  // SNN_REGION_INTERFACE_H