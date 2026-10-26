"""Nonblocking Trigger services with a timer-driven servo state machine."""

import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger

from .servo_pwm import PWMError, ServoPWM


class ServoTrigerNode(Node):
    def __init__(self):
        super().__init__('servo_triger_node')
        defaults = {
            'pwmchip_path': '/sys/class/pwm/pwmchip0', 'pwm_channel': 0,
            'period_ns': 20000000,
            'release_angle': 20.0, 'press_angle': 70.0,
            'min_angle': 0.0, 'max_angle': 180.0,
            'min_pulse_ns': 1000000, 'max_pulse_ns': 2000000,
            'hold_time': 0.3, 'move_time': 0.3,
            'timer_period': 0.01, 'export_timeout': 2.0,
        }
        # Startup-only parameters: reject live edits that would desynchronize hardware.
        from rcl_interfaces.msg import ParameterDescriptor
        for name, default in defaults.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        params = {name: self.get_parameter(name).value for name in defaults}
        for name in ('hold_time', 'move_time', 'timer_period', 'export_timeout'):
            value = params[name]
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if any(params[name] <= 0 for name in ('timer_period', 'export_timeout')):
            raise ValueError('timer_period and export_timeout must be positive')

        self.pwm = ServoPWM(**{key: params[key] for key in (
            'pwmchip_path', 'pwm_channel', 'period_ns', 'min_angle', 'max_angle',
            'min_pulse_ns', 'max_pulse_ns')})
        self.release_angle = params['release_angle']
        self.press_angle = params['press_angle']
        self.pwm.angle_to_pulse(self.release_angle)
        self.pwm.angle_to_pulse(self.press_angle)
        self.hold_time = params['hold_time']
        self.move_time = params['move_time']
        self.state = 'IDLE'
        self.ready = False
        self.fault = None
        self.pending = None
        self.deadline = time.monotonic() + params['export_timeout']
        self.initialized = False
        self.closed = False

        # A suspended service callback must not prevent another request or timer.
        # main() uses one executor thread, so state transitions are serialized.
        self.service_group = ReentrantCallbackGroup()
        self.trigger_services = [self.create_service(
            Trigger, name, self.on_trigger, callback_group=self.service_group)
            for name in ('/triger', '/press_switch')]
        self.timer = self.create_timer(params['timer_period'], self.on_timer)
        try:
            self.pwm.request_export()
        except PWMError as exc:
            self.fail(str(exc))

    async def on_trigger(self, request, response):
        if self.closed:
            response.success, response.message = False, 'Servo is shutting down'
        elif self.pending is not None:
            response.success, response.message = False, 'Servo is busy'
        elif not self.ready:
            response.success, response.message = False, self.fault or 'Servo is initializing'
        else:
            self.pending = Future()
            completion = self.pending
            self.state = 'PRESS'
            self.deadline = None
            self.get_logger().info('IDLE -> PRESS')
            # rclpy yields this coroutine; the executor continues processing timers
            # and other service requests until the action has completed.
            response.success, response.message = await completion
        return response

    def on_timer(self):
        if self.closed or self.fault:
            return
        now = time.monotonic()
        try:
            if not self.ready:
                if not self.pwm.exported():
                    if now >= self.deadline:
                        raise PWMError(f'PWM export timed out: {self.pwm.path} did not appear')
                    return
                self.pwm.initialize(self.release_angle)
                self.initialized = True
                self.ready = True
                self.get_logger().info(
                    f'PWM ready at {self.pwm.path}; release={self.release_angle} deg; '
                    'services: /triger and /press_switch')
                return
            if self.state == 'PRESS':
                if self.deadline is None:
                    self.pwm.set_angle(self.press_angle)
                    self.deadline = now + self.move_time
                elif now >= self.deadline:
                    self.state = 'HOLD'
                    self.deadline = now + self.hold_time
                    self.get_logger().info('PRESS -> HOLD')
            elif self.state == 'HOLD' and now >= self.deadline:
                self.state = 'RELEASE'
                self.pwm.set_angle(self.release_angle)
                self.deadline = now + self.move_time
                self.get_logger().info('HOLD -> RELEASE')
            elif self.state == 'RELEASE' and now >= self.deadline:
                self.state = 'IDLE'
                self.get_logger().info('RELEASE -> IDLE')
                self.finish(True, 'Servo press completed')
        except (PWMError, ValueError) as exc:
            self.fail(str(exc))

    def finish(self, success, message):
        completion, self.pending = self.pending, None
        if completion is not None and not completion.done():
            completion.set_result((success, message))

    def fail(self, message):
        self.fault = message
        self.ready = False
        self.get_logger().error(message)
        self.finish(False, message)
        # Keep the service alive to report the fault, with no further motion.
        self.state = 'IDLE'
        if self.pwm.exported():
            try:
                self.pwm.disable()
            except PWMError as exc:
                self.get_logger().error(f'Could not disable PWM after error: {exc}')

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.timer.cancel()
        self.finish(False, 'Servo is shutting down')
        if not self.pwm.exported():
            return
        try:
            if self.initialized:
                self.pwm.set_angle(self.release_angle)
                # Executor has stopped. Keep pulses active during this final
                # travel allowance; no service/timer callback blocks or sleeps.
                threading.Event().wait(self.move_time)
        except (PWMError, ValueError) as exc:
            self.get_logger().error(f'Failed to restore release angle at shutdown: {exc}')
        finally:
            try:
                self.pwm.disable()
            except PWMError as exc:
                self.get_logger().error(f'Failed to disable PWM at shutdown: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = ServoTrigerNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, PWMError) as exc:
        rclpy.logging.get_logger('servo_triger_node').error(f'Servo startup failed: {exc}')
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.close()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
