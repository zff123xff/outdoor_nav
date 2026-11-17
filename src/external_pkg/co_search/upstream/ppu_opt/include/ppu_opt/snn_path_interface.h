#ifndef SNN_PATH_INTERFACE_H
#define SNN_PATH_INTERFACE_H

/*
 * snn_path_interface.h — SNN 路径规划器的抽象接口。
 *
 * 提供统一接口 ISnnPathPlanner，和一个工厂函数 createPathPlanner()，
 * 根据 ROS2 参数 "ppu/enable_path_planning" 动态选择：
 *   - true  → PpuSnnPathPlanner（优先远程 PPU；PPU 忙 code=30 时本次调用回退本地）
 *   - false → LocalSnnPathPlanner（本地 WavefrontPlanner，默认）
 */

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ppu_path_client.h"

namespace fast_planner
{

struct PlannerConfig;

class ISnnPathPlanner
{
public:
    virtual ~ISnnPathPlanner() = default;

    /** 设置栅格地图（二维 int 栅格，0=自由，1=障碍） */
    virtual void setGrid(const std::vector<std::vector<int>>& grid) = 0;

    /** 规划路径：起点 → 终点，返回路径点序列 */
    virtual std::vector<Eigen::Vector3i> plan(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) = 0;

    /** 计算某起点的可达性掩码（1=BFS 可达） */
    virtual std::vector<std::uint8_t> computeReachabilityMask(const Eigen::Vector3i& start) = 0;

    virtual int gridHeight() const = 0;
    virtual int gridWidth() const  = 0;
};

/**
 * 工厂函数：读取 node param "ppu/enable_path_planning"
 *   true  → 创建 PpuSnnPathPlanner（优先 PpuPathClient 远程调用 PPU；
 *            PPU 忙（3403 code=30）时本次调用自动回退本地 WavefrontPlanner）
 *   false → 创建 LocalSnnPathPlanner（使用本地 WavefrontPlanner，默认）
 */
std::shared_ptr<ISnnPathPlanner> createPathPlanner(rclcpp::Node::SharedPtr node, const PlannerConfig& cfg);

}  // namespace fast_planner

#endif  // SNN_PATH_INTERFACE_H