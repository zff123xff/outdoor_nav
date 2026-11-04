#ifndef EXPLORATION_MANAGER_RETURN_POLICY_H_
#define EXPLORATION_MANAGER_RETURN_POLICY_H_

// 返航策略纯函数集合：返航场景下的仲裁规则。
// 均为纯函数、无 ROS/Eigen 依赖，便于独立单元测试。

// ===== 收尾返航投票排序（true = a 优先返航）=====
// 收尾阶段"谁先返航"的共识投票排序（升序，先返航者优先）。
// 比较链：任务量(滞回) -> 到返航点距离 -> ID。
inline bool FinalReturnVotePreferred(int load_a, int load_b, double dist_a, double dist_b, int id_a, int id_b, int load_margin = 2)
{
    // 任务量显著更少者先返航；差距在滞回带内视为相当，交由下一级比较。
    if (load_a + load_margin < load_b)
        return true;
    if (load_b + load_margin < load_a)
        return false;
    // 任务量相当：离返航点更近者先返航（1e-6 容差防浮点抖动）。
    if (dist_a + 1e-6 < dist_b)
        return true;
    if (dist_b + 1e-6 < dist_a)
        return false;
    // 完全相当时小 ID 优先，保证所有车辆共识出同一 winner。
    return id_a < id_b;
}

#endif  // EXPLORATION_MANAGER_RETURN_POLICY_H_
