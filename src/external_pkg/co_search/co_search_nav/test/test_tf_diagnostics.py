"""Exercise the actual diagnostic methods without DDS or robot motion."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


@pytest.fixture
def probe():
    source = Path(__file__).parents[1] / 'co_search_nav/executor.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CoSearchExecutor')
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
                ('tf_diagnostics', 'monitor_tf')]
    cls.bases = []
    clock = NS(mono=20., ros=100., wall=100.)
    fake_time = NS(monotonic=lambda: clock.mono, time=lambda: clock.wall)
    stamp_type = NS(from_msg=lambda s: NS(nanoseconds=int(s.seconds*1e9)))
    # Time() is only the latest-transform selector here.
    class Time:
        def __new__(cls): return None
        from_msg = stamp_type.from_msg
    namespace = dict(time=fake_time, Time=Time, os=NS(getpid=lambda: 123))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
    node = namespace['CoSearchExecutor']()
    logs = []
    node.frame = 'robot_1/map'; node.base = 'robot_1/base_link'
    node.tf_health_sample = None; node.tf_health_log = 0.
    node.tf_health_bad = False; node.tf_health_started = 0.
    node.config = {'options': {'sensor_timeout_s': 3.}}
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(clock.ros*1e9)))
    node.get_parameter = lambda _: NS(value=False)
    node.get_logger = lambda: NS(warning=lambda m: logs.append(('warn', m)),
                                info=lambda m: logs.append(('info', m)))
    pose = NS(header=NS(stamp=NS(seconds=99.9)))
    node.tf = NS(lookup_transform=lambda *args: pose)
    return node, clock, pose, logs


def test_fresh_tf_and_startup_details(probe):
    node, _, pose, logs = probe
    age, text = node.tf_diagnostics(pose, 3.)
    assert age == pytest.approx(.1)
    for field in ('ros_now=', 'tf_stamp=', 'age_s=', 'allowed_age_s=',
                  'monitor_gap_s=', 'tf_stamp_delta_s=', 'use_sim_time=False', 'pid=123'):
        assert field in text
    node.monitor_tf()
    assert not logs


@pytest.mark.parametrize('stamp', [95., 101.])
def test_old_and_future_tf_warn_then_recover(probe, stamp):
    node, clock, pose, logs = probe
    pose.header.stamp.seconds = stamp
    node.monitor_tf()
    assert node.tf_health_bad and logs[-1][0] == 'warn'
    clock.mono += 1.1
    node.monitor_tf()
    assert len(logs) == 1  # Repeated bad samples are throttled.
    clock.mono += 1.1
    pose.header.stamp.seconds = 99.9
    node.monitor_tf()
    assert not node.tf_health_bad
    assert 'recovered' in logs[-1][1]


def test_executor_delay_is_visible(probe):
    node, clock, pose, logs = probe
    node.monitor_tf()
    clock.mono += 3.
    clock.ros += 3.
    pose.header.stamp.seconds += 3.
    node.monitor_tf()
    assert not node.tf_health_bad  # TF fresh, executor polling delayed.
    assert 'monitor_gap_s=3.000' in logs[-1][1]
    assert 'tf_stamp_delta_s=3.000000' in logs[-1][1]


def test_missing_tf_is_throttled(probe):
    node, clock, _, logs = probe
    def missing(*args): raise RuntimeError('no connection')
    node.tf.lookup_transform = missing
    node.monitor_tf(); node.monitor_tf()
    assert len(logs) == 1
    assert 'TF unavailable' in logs[0][1] and 'no connection' in logs[0][1]
