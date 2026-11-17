#pragma once

#include <rclcpp/rclcpp.hpp>

namespace swarm_qos
{
// 仿真/实机 QoS 模式。由各节点启动时从 ROS 参数 "simulation" 读取并设置：
//   simulation == true  → 保持原有 QoS 策略（本机规划链 RELIABLE、跨车/高频 topic BEST_EFFORT）
//   simulation == false → 项目所有 topic 统一改用 BEST_EFFORT，降低实车 WiFi/现场 DDS 重传拥塞。
// 进程内默认 true，与现有行为一致；未显式设置该参数的节点沿用原策略。
inline bool& simulation_mode()
{
    static bool sim = true;
    return sim;
}

inline void set_simulation_mode(const bool sim)
{
    simulation_mode() = sim;
}

// 便捷：从节点参数 "simulation" 读取并设置全局 QoS 模式。
// 参数尚未声明时按 default_sim 声明（launch 传入的 override 会覆盖该默认值）。
inline void load_simulation_param(const rclcpp::Node::SharedPtr& node, const bool default_sim = true)
{
    if (!node->has_parameter("simulation"))
        node->declare_parameter<bool>("simulation", default_sim);
    set_simulation_mode(node->get_parameter("simulation").as_bool());
}

// 构造 QoS：depth 保持各 topic 原有 KeepLast 深度、durability 保持 VOLATILE。
// reliable 表示"原策略期望 RELIABLE"，仅在仿真模式下生效；实机模式一律 BEST_EFFORT。
inline rclcpp::QoS make_qos(const size_t depth, const bool reliable)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).durability_volatile();
    if (reliable && simulation_mode())
        qos.reliable();
    else
        qos.best_effort();
    return qos;
}

}  // namespace swarm_qos
