"""Request native GLOBAL_POSITION_INT at 30 Hz and monitor actual fix receipt."""
from collections import deque
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from sensor_msgs.msg import NavSatFix


class MavrosGlobalRate(Node):
    def __init__(self):
        super().__init__('mavros_global_rate')
        self.connected = False
        self.configured = False
        self.pending = None
        self.pending_since = 0.
        self.last_request = 0.
        self.samples = deque(maxlen=300)
        self.interval_clients = [self.create_client(MessageInterval, name) for name in
                        ('mavros/set_message_interval', 'mavros/sys/set_message_interval')]
        self.create_subscription(State, 'mavros/state', self.state, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, 'mavros/global_position/global', self.fix, qos_profile_sensor_data)
        self.create_timer(5., self.tick)

    def state(self, msg):
        if msg.connected != self.connected:
            self.configured = False
            self.samples.clear()
        self.connected = msg.connected

    def fix(self, msg):
        self.samples.append(time.monotonic())

    def tick(self):
        now = time.monotonic()
        hz = ((len(self.samples)-1)/(self.samples[-1]-self.samples[0])
              if len(self.samples) > 1 and now-self.samples[-1] < 1. else 0.)
        if self.configured and 27. <= hz <= 33.:
            self.get_logger().info(f'GPS global NavSatFix verified: {hz:.1f} Hz')
            return
        if self.pending:
            if now-self.pending_since < 8.:
                return
            self.pending.cancel()
            self.pending = None
        if not self.connected:
            self.get_logger().warning('Waiting for MAVROS FCU connection; GPS 30 Hz not verified')
            return
        if now-self.last_request < 10.:
            return
        client = next((c for c in self.interval_clients if c.service_is_ready()), None)
        if client is None:
            self.get_logger().warning('Waiting for MAVROS set_message_interval service')
            return
        self.last_request = now
        self.pending_since = now
        future = client.call_async(MessageInterval.Request(message_id=33, message_rate=30.))
        self.pending = future
        def done(result):
            if self.pending is not result:
                return
            self.pending = None
            try:
                self.configured = bool(result.result().success)
            except Exception:
                self.configured = False
            self.get_logger().info(f'GLOBAL_POSITION_INT 30 Hz request accepted={self.configured}; measured={hz:.1f} Hz')
            if not self.configured or hz < 27.:
                self.get_logger().warning('GPS rate not yet verified; check FCU stream support, EKF and serial bandwidth')
        future.add_done_callback(done)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosGlobalRate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
