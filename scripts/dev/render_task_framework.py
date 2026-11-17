#!/usr/bin/env python3
"""Regenerate the current task framework and each module's DOT/SVG/PNG."""
import json
from pathlib import Path
import subprocess

OUT = Path(__file__).resolve().parents[2]/'docs/diagrams'
MODULES = OUT/'modules'
MODULES.mkdir(parents=True, exist_ok=True)


def diagram(path, title, labels, edges, direction='TB'):
    q=lambda value:json.dumps(value,ensure_ascii=False)
    lines=['digraph G {',f'graph [rankdir={direction}, bgcolor="#f8fafc", pad="0.3", fontname="Noto Sans CJK SC", fontsize=24, labelloc=t, label={q(title)}];',
        'node [shape=box, style="rounded,filled", fillcolor="#dbeafe", color="#64748b", fontname="Noto Sans CJK SC", fontsize=16, margin="0.18,0.14"];',
        'edge [fontname="Noto Sans CJK SC", fontsize=12, color="#64748b"];']
    for key,label in labels.items():lines.append(f'{key} [label={q(label)}];')
    for source,target,label in edges:lines.append(f'{source} -> {target} [label={q(label)}];')
    lines.append('}')
    path.with_suffix('.dot').write_text('\n'.join(lines))
    for ext in ('svg','png'):
        subprocess.run(['dot','-T'+ext,'-Gdpi=100',str(path.with_suffix('.dot')),'-o',str(path.with_suffix('.'+ext))],check=True)


def chain(name, title, steps):
    labels={f'n{i}':value for i,value in enumerate(steps)}
    diagram(MODULES/name,title,labels,[(f'n{i}',f'n{i+1}','') for i in range(len(steps)-1)])


diagram(OUT/'target_framework','任务系统 · 独立目标巡检框架',dict(
    c='指控端\nROS / MQTT',g='单任务接收 / 忙碌拒收\n固定编号，无 task_type',
    p='140 定点导航\npoint_goal_executor',l='141 两点巡线\npatrol_node',
    a='142 独立区域搜索\narea_search_co_search',m='145 多航点\nmulti_point_executor',
    i='143 目标巡检\ntarget_inspection_executor',n='共享导航\nstraight_planner / controller_server\nlocal_costmap / FollowTarget',
    x='twist_mux → 原速度桥',u='可选 inspection_udp_bridge\n目标输入 / 到达事件输出',ue='UE 仿真',
    t='原传感器桥\nodom / 点云',tf='input_bridge / odom_relay\nTF / 仿真 GPS',
    f='robot_task_status\ntask_feedback',y='保留独立 YOLO\n模型推理 / yolo/detections\n不自动启动'),[
    ('c','g','/<ns>/task'),('g','p','140'),('g','l','141'),('g','a','142'),('g','m','145'),('g','i','143'),
    *[(k,'n','导航 Action') for k in ('p','l','a','m','i')],('n','x','cmd_vel_nav'),('x','ue','速度 UDP'),
    ('ue','u','ENU 目标批次 UDP'),('u','i','targets/assigned'),('i','u','inspection_arrived'),('u','ue','到达事件 UDP'),
    ('ue','t','传感器 UDP'),('t','tf','odom'),('tf','n','定位'),('tf','i','新鲜 TF'),('g','f','任务进度/终态'),('f','c','反馈')], 'LR')

modules=[
 ('01_task_pipeline','指令入口、调度与取消',['ROS Task / MQTT JSON','忙碌直接拒收；空闲再校验参数\n取消只匹配当前 task_id','一个活动基元，无执行队列','调用对应 Action / 接收反馈','取消时等待子 Action 终结\n完成前不接收下一任务','1 Hz robot_task_status\n终态 progress=100 + task_feedback']),
 ('02_point_goal','140 定点导航',['一个 GPS 三元组','NavSatFix → PointGoal','导航参考原点转换 ENU','NavigateToPose → 导航栈','新鲜 GPS / ENU 进度','以关联 Action 结果判定完成']),
 ('03_patrol_line','141 两点巡线',['两个 GPS 三元组','转换 ENU / 获取新鲜机器人 TF','选择最近端点并导航','到另一端点','返回起始端点：完成一轮','三次端点到达 → 终态']),
 ('04_multi_point','145 多航点巡线',['GPS 三元组序列','MultiPoint Action','逐点调用 PointGoal','等当前导航结果再发下一点','取消/失败立即停止后续点\n等待当前请求收尾','全部点成功才返回成功']),
 ('05_area_search','142 独立区域搜索',['GPS center + 长宽 + ENU 方向角','AreaSearch Action','co_search 构建区域与感知地图','选择搜索路径 / 导航执行','更新覆盖与进度 / 确认真实完成','返回结果；不与 143 / YOLO 交互']),
 ('06_target_inspection','143 目标巡检',['启动：快照最新有效目标批次\n无批次最多等待 10 s','根据当前三维位置选择最近待处理目标','超出 10 m：导航至默认 9 m 停靠点\n范围内：保持位置','等待新 TF → 朝向目标','验证 ≤10 m、角度≤0.15 rad\n连续停稳≥0.5 s','发布 inspection_arrived 一次','非阻塞等待 2.5 s → 下一个目标','失败目标跳过并计数\n取消先收尾；全部处理后返回汇总']),
 ('07_target_manager','目标批次管理',['TargetArray：ID + ENU Point','原子校验：正整数唯一 ID / 有限坐标','替换下一任务可用的最新批次','当前任务快照不被后续批次改变','每次用新鲜位姿重新排序','锁定当前目标并从待处理集合移除\n当前快照不重复发布到达']),
 ('08_approach_heading','接近、朝向与停稳检查',['新鲜 map → base_link TF','计算真实三维距离及高差','目标附近停靠点：默认 9 m\n保留导航误差余量','NavigateToPose 接近 / 等待终态','等新 TF 后用 FollowTarget 朝向','再等新 TF，验证实测范围与朝向','用连续样本和单调时间验证停稳','发布巡检观察事件']),
 ('09_motion_control','共享运动控制',['NavigateToPose / FollowPath','straight_planner / 路径','障碍点云 + local_costmap\n已有 inflation_layer','controller_server\nFollowPath / FollowTarget','cmd_vel_nav → twist_mux','原 ROS→UE 桥 / 实机驱动']),
 ('10_udp_bridge','巡检 UDP 传输（可选）',['UE → 独立 UDP 9101','校验 namespace / topic / 批次\n拒绝无效数据','ROS targets/assigned → 巡检节点','ROS targets/inspection_arrived','序列化观察事件 JSON','独立 UDP → UE 9001\n不改变原传感器桥，不做轴向/单位转换']),
 ('11_localization','定位与环境信息',['UE 传感器 → 原 ue_to_ros_bridge','mavros/odom / 点云','nav2_input_bridge 发布机器人 TF','odom_relay 转发 QoS / 仿真 GPS\n参考原点来自导航 YAML','导航栈与巡检读取定位','TF 新鲜性按本机接收进度\n停更时禁止发布到达事件']),
 ('12_feedback','任务与巡检反馈',['执行器 Action feedback / result','活动任务进度单调递增 1…99','约 1 Hz robot_task_status','成功/失败/取消均终态 100\nfailures + task_feedback 区分结果','ROS / 可选 MQTT → 指控端','巡检到达事件独立发往仿真\n不把导航成功直接当观察到达'])]
for name,title,steps in modules:chain(name,title,steps)
# Remove superseded diagrams generated for the previous, unimplemented proposal.
for ext in ('.dot','.svg','.png'):
    old=MODULES/('06_search_shoot'+ext)
    if old.exists():old.unlink()
text=['# 当前任务框架 · 模块内部流程图','','[整体说明](../DATA_FLOW.md) · [总体 SVG](target_framework.svg)','']
for name,title,steps in modules:
    text.extend([f'## {title}','',f'![{title}](modules/{name}.png)','',f'[高清 SVG](modules/{name}.svg) · [DOT](modules/{name}.dot)',''])
(OUT/'MODULES.md').write_text('\n'.join(text))
