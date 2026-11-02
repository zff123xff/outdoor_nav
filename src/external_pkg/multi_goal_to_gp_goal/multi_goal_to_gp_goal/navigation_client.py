"""Correlated navigation Action client shared by tasks and waypoint queues."""
import time
import math
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def navigation_parameters(path):
    """Keep projected task coordinates in the active planner's reference frame."""
    with open(path) as stream:
        data = yaml.safe_load(stream) or {}
    bridge = data['nav2_input_bridge']['ros__parameters']
    values = {key: float(bridge[key]) for key in
              ('reference_latitude', 'reference_longitude', 'reference_altitude')}
    values['global_frame'] = data.get('straight_planner', {}).get('ros__parameters', {}).get(
        'global_frame', bridge.get('goal_output_frame', 'map'))
    return values


class NavigationClient:
    def __init__(self, node, frame=None):
        self.node = node
        if not node.has_parameter('navigation_action'):
            node.declare_parameter('navigation_action', 'navigate_to_pose')
        self.client = ActionClient(node, NavigateToPose, node.get_parameter('navigation_action').value)
        namespace = node.get_namespace().strip('/')
        if not node.has_parameter('global_frame'):
            node.declare_parameter('global_frame', 'map')
        raw_frame = frame or node.get_parameter('global_frame').value
        self.frame = raw_frame.strip('/')
        if namespace and not self.frame.startswith(namespace+'/'):
            self.frame = namespace+'/'+self.frame
        self.generation = 0
        self.handle = None
        self.pending = None
        self.responses = set()
        self.retiring = []
        self.result_future = None
        self.timer = node.create_timer(0.1, self._pump)

    def send(self, position, done, heading=0.0):
        self.cancel()
        self.generation += 1
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(position[0])
        goal.pose.pose.position.y = float(position[1])
        goal.pose.pose.position.z = float(position[2])
        goal.pose.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(heading / 2.0)
        self.pending = (self.generation, goal, done, time.monotonic()+10.0)
        self._pump()

    def cancel(self):
        self.generation += 1
        self.pending = None
        if self.handle is not None:
            self.handle.cancel_goal_async()
            if self.result_future is not None:
                self.retiring.append(self.result_future)
            self.handle = None

    def _pump(self):
        if self.pending is None:
            return
        generation, goal, done, deadline = self.pending
        self.retiring = [future for future in self.retiring if not future.done()]
        if self.responses or self.retiring or not self.client.server_is_ready():
            if time.monotonic() >= deadline:
                self.pending = None
                self.node.get_logger().error('Navigation Action unavailable; task failed without claiming arrival')
                done(GoalStatus.STATUS_ABORTED)
            return
        self.pending = None
        try:
            future = self.client.send_goal_async(goal)
        except Exception as error:
            self.node.get_logger().error(f'Navigation request failed: {error}')
            done(GoalStatus.STATUS_ABORTED)
            return
        self.responses.add(generation)

        def accepted(response):
            self.responses.discard(generation)
            try:
                handle = response.result()
            except Exception as error:
                if generation == self.generation:
                    self.node.get_logger().error(f'Navigation request failed: {error}')
                    done(GoalStatus.STATUS_ABORTED)
                return
            if generation != self.generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                    self.retiring.append(handle.get_result_async())
                return
            if not handle.accepted:
                done(GoalStatus.STATUS_ABORTED)
                return
            self.handle = handle
            result = handle.get_result_async()
            self.result_future = result

            def completed(response):
                if generation != self.generation:
                    return
                self.handle = None
                try:
                    status = response.result().status
                except Exception as error:
                    self.node.get_logger().error(f'Navigation result failed: {error}')
                    status = GoalStatus.STATUS_ABORTED
                done(status)
            result.add_done_callback(completed)
        future.add_done_callback(accepted)
