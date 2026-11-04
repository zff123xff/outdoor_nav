import time
import json

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

    client = node.create_client(
        GetParameters,
        target + '/get_parameters'
    )

    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                f'无法连接 {target} 参数服务；请先启动导航栈'
            )

        future = client.call_async(
            GetParameters.Request(
                names=[
                    'reference_latitude',
                    'reference_longitude',
                    'reference_altitude',
                ]
            )
        )

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=timeout
        )

        if not future.done() or future.result() is None:
            raise RuntimeError(
                f'读取 {target} 导航原点超时'
            )

        values = [
            parameter_value_to_python(value)
            for value in future.result().values
        ]

        return tuple(
            float(value)
            for value in values
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()



def submit(task, namespace, timeout, check=False, cancel=False, ros_args=None):
    import rclpy
    from task_interfaces.msg import Task
    from std_msgs.msg import String
    from task_pipeline.task_schema import to_message

    rclpy.init(args=ros_args)
    node = rclpy.create_node('primitive_example_sender')
    pub = node.create_publisher(Task, f'/{namespace}/task', 10)
    terminal = {}
    def feedback(message):
        data = json.loads(message.data)
        if data.get('task_id') != task['task_id']:
            return
        print(json.dumps(data, ensure_ascii=False), flush=True)
        if data.get('event') in ('completed', 'failed', 'rejected'):
            terminal.update(done=True, success=data['event'] == 'completed')
    node.create_subscription(String, f'/{namespace}/task_feedback', feedback, 10)
    try:
        deadline = time.monotonic()+timeout
        while not pub.get_subscription_count() or (not cancel and not node.count_publishers(f'/{namespace}/task_feedback')):
            if time.monotonic() > deadline:
                print('Task topic subscriber unavailable')
                return 2
            rclpy.spin_once(node, timeout_sec=.1)
        if check:
            print('Task topic subscriber online; no task published')
            return 0
        pub.publish(to_message(dict(task_id=task['task_id'], cancel=True)) if cancel else to_message(task))
        if not pub.wait_for_all_acked(__import__('rclpy.duration', fromlist=['Duration']).Duration(seconds=timeout)):
            print('Task delivery acknowledgment timed out; do not submit blindly again')
            return 3
        if cancel:
            return 0
        while rclpy.ok() and not terminal.get('done'):
            rclpy.spin_once(node, timeout_sec=.2)
        return 0 if terminal.get('success') else 3
    except KeyboardInterrupt:
        cancel_pub = pub
        deadline = time.monotonic()+2.
        while not cancel_pub.get_subscription_count() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        cancel_pub.publish(to_message(dict(task_id=task['task_id'], cancel=True)))
        cancel_pub.wait_for_all_acked(__import__('rclpy.duration', fromlist=['Duration']).Duration(seconds=2.))
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
