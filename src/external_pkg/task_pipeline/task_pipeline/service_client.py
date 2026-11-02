"""Shared CLI client for submitting primitive tasks through ExecuteTask."""

import json
import sys
import time


def resolve_navigation_origin(robot_namespace, timeout, ros_args=None):
    """Read the authoritative ENU origin from the running Nav2 input bridge."""
    import rclpy
    from rclpy.parameter import parameter_value_to_python
    from rcl_interfaces.srv import GetParameters

    namespace = str(robot_namespace).strip('/')
    if not namespace:
        raise RuntimeError('ENU 任务需要明确的 robot_namespace')
    target = f'/{namespace}/nav2_input_bridge'
    rclpy.init(args=ros_args)
    node = rclpy.create_node('task_pipeline_origin_reader')
    client = node.create_client(GetParameters, target + '/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'无法连接 {target} 参数服务；请先启动导航栈')
        future = client.call_async(GetParameters.Request(names=[
            'reference_latitude', 'reference_longitude', 'reference_altitude']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'读取 {target} 导航原点超时')
        values = [parameter_value_to_python(value) for value in future.result().values]
        return tuple(float(value) for value in values)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def submit(task, service_name, robot_namespace, timeout, check=False, ros_args=None):
    import rclpy
    from execute_task_server.srv import ExecuteTask
    from std_msgs.msg import String

    namespace = robot_namespace.strip('/')
    feedback_topic = f'/{namespace}/task_feedback' if namespace else '/task_feedback'
    rclpy.init(args=ros_args)
    node = rclpy.create_node('task_pipeline_manual_client')
    client = node.create_client(ExecuteTask, service_name)
    terminal = {'done': False, 'success': False}

    def on_feedback(message):
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if data.get('mission_id') != task.get('mission_id'):
            return
        event = data.get('event')
        print(f"pipeline feedback: {json.dumps(data, ensure_ascii=False)}", flush=True)
        if event in ('completed', 'partial', 'failed', 'rejected', 'canceled', 'cancelled', 'paused'):
            terminal['done'] = True
            terminal['success'] = event == 'completed'

    subscription = node.create_subscription(String, feedback_topic, on_feedback, 10)
    try:
        print(f'等待 pipeline 服务（最多 {timeout:g} 秒）: {service_name}')
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= deadline:
                print(f'ExecuteTask 服务不可用: {service_name}', file=sys.stderr)
                return 2
        if check:
            print('ExecuteTask 服务在线；仅检查，未下发任务。')
            return 0

        request = ExecuteTask.Request()
        request.request_json = json.dumps(task, ensure_ascii=False, allow_nan=False)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            print('Pipeline 响应超时；接收结果未知，请检查任务反馈，勿重复提交。', file=sys.stderr)
            return 3
        response = future.result()
        if response is None:
            print(f'ExecuteTask 调用失败: {future.exception()}', file=sys.stderr)
            return 3
        try:
            result = json.loads(response.response_json)
        except ValueError:
            print(f'无法解析 ExecuteTask 响应: {response.response_json}', file=sys.stderr)
            return 3
        print(f'pipeline 响应: {json.dumps(result, ensure_ascii=False)}', flush=True)
        if result.get('status') not in ('success', 'accepted'):
            return 3

        # For Action-backed primitives, keep the CLI open until the pipeline
        # publishes the final Action result. Generic waypoint tasks only get an
        # acceptance response from their existing topic-based executor.
        action_backed = task.get('capability_id') in ('area_search', 'patrol_line', 'point_goal', 'goto_waypoints') and (
            task.get('capability_id') in ('patrol_line', 'area_search') or
            len(task.get('params', {}).get('waypoints', [])) == 1)
        if not action_backed:
            return 0
        while rclpy.ok() and not terminal['done']:
            rclpy.spin_once(node, timeout_sec=0.2)
        return 0 if terminal['success'] else 3
    except KeyboardInterrupt:
        print('请求 pipeline 取消当前任务…', file=sys.stderr)
        cancel = ExecuteTask.Request()
        cancel.request_json = json.dumps({
            'mission_id': f"cancel_{task.get('mission_id', '')}",
            'agent_id': task.get('agent_id', namespace),
            'capability_id': 'cancel_area_search' if task.get('capability_id') == 'area_search' else 'cancel_task',
            'params': {'mission_id': task.get('mission_id', '')},
        })
        try:
            future = client.call_async(cancel)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            if future.done() and future.result() is not None:
                print(f'取消响应: {future.result().response_json}', file=sys.stderr)
        except Exception as exc:
            print(f'取消请求失败: {exc}', file=sys.stderr)
        return 130
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
