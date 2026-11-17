"""Review the additive bridge patch without binding UDP or publishing ROS data."""
import ast
import importlib.util
import math
import os
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('origin_bridge_patch', ROOT/'scripts/dev/patch_mavros_origin.py')
patcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(patcher)


@pytest.fixture(scope='module')
def bridge(tmp_path_factory):
    path = Path(os.environ.get('MAVROS_BRIDGE_DIR', str(ROOT.parent/'MavrosBridge')))/'ue_to_ros_bridge.py'
    if not path.exists():
        pytest.skip('External MavrosBridge checkout is not available')
    source = path.read_text()
    patched = patcher.patched_source(source)
    assert patcher.patched_source(patched) == patched
    target = tmp_path_factory.mktemp('origin_bridge')/'bridge.py'
    target.write_text(patched)
    spec = importlib.util.spec_from_file_location('ue_origin_review', target)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, source, patched


def harness(module):
    node = SimpleNamespace(pubs={}, qos=module.QoSProfile(depth=50))
    node.get_logger = Mock(return_value=Mock())
    node.create_publisher = Mock(side_effect=lambda *args: Mock(topic_name=args[1]))
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: module.NavSatFix().header.stamp))
    node._publish_message = lambda publisher, message: publisher.publish(message)
    cls = module.UnifiedUeToRosBridge
    for name in ('_get_publisher', '_fill_header', '_handle_simulation_origin'):
        setattr(node, name, MethodType(getattr(cls, name), node))
    node._msg_type_for_suffix = cls._msg_type_for_suffix
    node._fill_status = cls._fill_status
    return node


def test_origin_conversion_has_dedicated_topic_and_retained_qos(bridge):
    module, _, _ = bridge
    node = harness(module)
    node._handle_simulation_origin('robot_2', dict(latitude=31., longitude=120., altitude=15.))
    message = node.pubs['/robot_2/simulation/gps_origin'].publish.call_args.args[0]
    assert (message.latitude, message.longitude, message.altitude) == (31., 120., 15.)
    qos = node.create_publisher.call_args.args[2]
    assert qos.durability == module.DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.depth == 1
    node._get_publisher('robot_2', module.DRONE_GPS_SUFFIX)
    assert node.create_publisher.call_args.args[2] is node.qos


@pytest.mark.parametrize('data', [{}, {'latitude': 31., 'longitude': 120.},
                                {'latitude': 91., 'longitude': 120., 'altitude': 15.},
                                {'latitude': 31., 'longitude': 120., 'altitude': math.nan},
                                {'latitude': True, 'longitude': 120., 'altitude': 15.}])
def test_invalid_origin_is_not_published(bridge, data):
    node = harness(bridge[0])
    with pytest.raises(ValueError):
        node._handle_simulation_origin('robot_1', data)
    node.create_publisher.assert_not_called()


def test_existing_sensor_handlers_are_unchanged(bridge):
    _, original, patched = bridge
    def methods(source):
        cls = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef) and n.name == 'UnifiedUeToRosBridge')
        return {n.name: ast.dump(n) for n in cls.body if isinstance(n, ast.FunctionDef)}
    old, new = methods(original), methods(patched)
    for key in old:
        if key not in ('__init__', '_get_publisher', '_msg_type_for_suffix', '_handle_simulation_origin'):
            assert old[key] == new[key], key
