"""Shared task schema/geometry; search policy belongs to native co-search."""
from pathlib import Path
import math
import pytest
import yaml
from co_search_nav.task_geometry import Rectangle, gps_to_enu, rectangle_for_task, validate_task
from task_pipeline.examples.sender import load_example

CONFIG = Path(__file__).parents[1]/'config/examples/enu/area_search.yaml'


def task():
    return yaml.safe_load(CONFIG.read_text())


def test_sender_preserves_rotated_rectangle():
    ns, request = load_example(CONFIG, 'area_search', 'robot_2', (40., 113., 1200.))
    assert ns == 'robot_2'
    assert request['task_type'] == 'area_search'
    assert request['area']['length_m'] == task()['rectangle']['length_m']


def test_rotated_rectangle_and_gps_geometry():
    region = Rectangle(10, -5, 30, 12, math.pi/3)
    assert region.local(*region.world(2, -3)) == pytest.approx((2, -3))
    assert all(region.contains(x, y, margin=-1e-8) for x, y in region.corners())
    origin = (40., 113., 1200.)
    assert gps_to_enu(*origin, origin) == pytest.approx((0, 0, 0))
    data = task(); data['coordinate_mode'] = 'gps'
    data['center_gps'] = dict(latitude=40.0001, longitude=113.0001, altitude=1200.)
    east, north, _ = gps_to_enu(40.0001, 113.0001, 1200., origin)
    gps = rectangle_for_task(validate_task(data), origin)
    data.update(coordinate_mode='enu', center_enu=dict(east=east, north=north))
    assert gps == rectangle_for_task(validate_task(data), origin)


@pytest.mark.parametrize('field,value', [('robot_namespace', ''), ('coordinate_mode', 'unknown'),
    ('mission_id', ''), ('search_controller', 'legacy')])
def test_invalid_task_fields_are_rejected(field, value):
    data = task(); data[field] = value
    with pytest.raises(ValueError): validate_task(data)


@pytest.mark.parametrize('value', [0, -1, float('nan'), True])
def test_invalid_sensor_timeout_is_rejected(value):
    data = task(); data['options']['sensor_timeout_s'] = value
    with pytest.raises(ValueError): validate_task(data)
