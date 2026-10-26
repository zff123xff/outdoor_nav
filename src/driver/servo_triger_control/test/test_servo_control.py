import time
import uuid

import pytest
import rclpy
from std_srvs.srv import Trigger

from servo_triger_control.servo_pwm import PWMError, ServoPWM
from servo_triger_control.servo_triger_node import ServoTrigerNode


def make_pwm(path):
    return ServoPWM(str(path), 0, 20000000, 0.0, 180.0, 1000000, 2000000)


def test_pulse_mapping_and_missing_files(tmp_path):
    pwm = make_pwm(tmp_path)
    assert pwm.angle_to_pulse(0.0) == 1000000
    assert pwm.angle_to_pulse(90.0) == 1500000
    assert pwm.angle_to_pulse(180.0) == 2000000
    assert pwm.angle_to_pulse(20.0) == 1111111
    with pytest.raises(ValueError):
        pwm.angle_to_pulse(181.0)
    with pytest.raises(PWMError, match='does not exist'):
        pwm.set_angle(70.0)
    assert not (tmp_path / 'pwm0' / 'duty_cycle').exists()


def test_permission_error_names_files(tmp_path, monkeypatch):
    pwm = make_pwm(tmp_path)

    def denied(*args):
        raise PermissionError('denied')

    monkeypatch.setattr('servo_triger_control.servo_pwm.os.open', denied)
    with pytest.raises(PWMError, match='Permission denied') as error:
        pwm.set_angle(70.0)
    assert str(pwm.path / 'duty_cycle') in str(error.value)
    assert 'do not run the ROS node with sudo' in str(error.value)


def test_export_and_initialize_existing_channel(tmp_path, monkeypatch):
    pwm = make_pwm(tmp_path)
    # Missing export attribute must not be silently created as an ordinary file.
    with pytest.raises(PWMError, match='does not exist'):
        pwm.request_export()
    assert not (tmp_path / 'export').exists()
    (tmp_path / 'export').write_text('')
    pwm.request_export()
    assert (tmp_path / 'export').read_text().strip() == '0'
    assert not pwm.exported()  # The kernel creates the directory asynchronously.
    pwm.path.mkdir()
    for name, value in [('enable', 1), ('period', 30000000), ('duty_cycle', 25000000)]:
        (pwm.path / name).write_text(str(value))
    writes = []
    monkeypatch.setattr(pwm, '_write', lambda path, value: writes.append((path.name, value)))
    pwm.request_export()
    assert writes == []  # Do not export twice.
    pwm.initialize(20.0)
    assert writes == [('enable', 0), ('duty_cycle', 0), ('period', 20000000),
                      ('duty_cycle', 1111111), ('enable', 1)]


def test_fresh_export_zero_period_and_idempotent_disable(tmp_path, monkeypatch):
    """Model the kernel EINVAL guard, rather than permissive ordinary files."""
    pwm = make_pwm(tmp_path)
    pwm.path.mkdir()
    state = {'enable': 0, 'period': 0, 'duty_cycle': 0}
    writes = []
    monkeypatch.setattr(pwm, '_read', lambda name: state[name])

    def kernel_write(path, value):
        candidate = dict(state, **{path.name: value})
        if candidate['period'] == 0 or candidate['duty_cycle'] > candidate['period']:
            raise PWMError('Invalid argument: zero period or duty exceeds period')
        state.update(candidate)
        writes.append((path.name, value))

    monkeypatch.setattr(pwm, '_write', kernel_write)
    pwm.disable()
    assert writes == []
    pwm.initialize(20.0)
    assert writes[0] == ('period', 20000000)
    assert state == {'enable': 1, 'period': 20000000, 'duty_cycle': 1111111}
    pwm.disable()
    count = len(writes)
    pwm.disable()
    assert len(writes) == count


def test_service_busy_timing_and_failure(tmp_path, monkeypatch):
    """Run real service callbacks and executor against fake sysfs, never hardware."""
    from rclpy.executors import SingleThreadedExecutor
    channel = tmp_path / 'pwm0'
    channel.mkdir()
    for name in ('enable', 'period', 'duty_cycle'):
        (channel / name).write_text('0\n')
    writes = []

    def fake_write(self, path, value):
        writes.append((path.name, value, time.monotonic()))
        path.write_text(str(value))

    monkeypatch.setattr(ServoPWM, '_write', fake_write)
    # Unique service names prevent tests from reaching any deployed servo node
    # visible on the inherited ROS domain.
    prefix = '/servo_test_' + uuid.uuid4().hex
    rclpy.init(args=[
        '--ros-args', '-r', '/triger:=' + prefix + '/triger',
        '-r', '/press_switch:=' + prefix + '/press_switch',
    ])
    # Override declarations to avoid accessing any real PWM path.
    monkeypatch.setattr(ServoTrigerNode, 'declare_parameter',
                        _override_declarations(ServoTrigerNode.declare_parameter, tmp_path))
    node = ServoTrigerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def until(predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), 'executor timed out'

    try:
        until(lambda: node.ready)
        assert [item[:2] for item in writes[:5]] == [
            ('period', 20000000), ('duty_cycle', 0), ('period', 20000000),
            ('duty_cycle', 1111111), ('enable', 1)]
        first = node.create_client(Trigger, '/triger')
        alias = node.create_client(Trigger, '/press_switch')
        until(lambda: first.service_is_ready() and alias.service_is_ready())
        call = first.call_async(Trigger.Request())
        until(lambda: node.pending is not None)
        busy = alias.call_async(Trigger.Request())
        until(busy.done)
        assert not busy.result().success
        assert busy.result().message == 'Servo is busy'
        assert not call.done(), 'first call must wait for the motion to finish'
        until(call.done)
        assert call.result().success
        assert node.state == 'IDLE'
        motion = [entry for entry in writes[5:] if entry[0] == 'duty_cycle']
        assert [entry[1] for entry in motion] == [1388889, 1111111]
        assert motion[1][2] - motion[0][2] >= node.hold_time + node.move_time
        # After returning to IDLE the alias must accept another full action.
        repeat = alias.call_async(Trigger.Request())
        until(repeat.done)
        assert repeat.result().success

        original_set_angle = node.pwm.set_angle

        def failed_duty(angle):
            raise PWMError('duty_cycle write failed')

        monkeypatch.setattr(node.pwm, 'set_angle', failed_duty)
        failed = first.call_async(Trigger.Request())
        until(failed.done)
        assert not failed.result().success
        assert 'duty_cycle' in failed.result().message
        assert channel.joinpath('enable').read_text() == '0'
        monkeypatch.setattr(node.pwm, 'set_angle', original_set_angle)
    finally:
        node.close()
        # Fault cleanup already disabled PWM; shutdown restores release duty
        # without issuing a redundant disable write.
        assert writes[-1][:2] == ('duty_cycle', 1111111)
        assert channel.joinpath('enable').read_text() == '0'
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def _override_declarations(original, path):
    def declare(self, name, value=None, *args, **kwargs):
        overrides = {'pwmchip_path': str(path), 'move_time': 0.03,
                     'hold_time': 0.05, 'timer_period': 0.005}
        return original(self, name, overrides.get(name, value), *args, **kwargs)
    return declare
