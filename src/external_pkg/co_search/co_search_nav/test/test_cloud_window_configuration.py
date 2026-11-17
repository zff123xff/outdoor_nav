"""Rolling cloud-window configuration bounds and parameter wiring."""
from pathlib import Path
import pytest
from co_search_nav.configuration import engine_parameters
from co_search_nav.task_geometry import validate_task, rectangle_for_task

SHARE = Path(__file__).resolve().parents[1]


def parameters(tmp_path, settings):
    task = validate_task(dict(robot_namespace='robot_1', coordinate_mode='enu',
        center_enu=dict(east=0., north=0.), rectangle=dict(length_m=10., width_m=10.),
        co_search=dict(cloud_filter=settings)))
    return engine_parameters(task, rectangle_for_task(task, None), 'robot_1/map', tmp_path, SHARE, '/install')


def test_window_parameters_are_forwarded(tmp_path):
    result = parameters(tmp_path, dict(accumulation_window_s=.8, min_accumulated_points=20))
    assert result['adapter/cloud_accumulation_window_s'] == .8
    assert result['adapter/cloud_min_accumulated_points'] == 20


@pytest.mark.parametrize('settings', [
    dict(accumulation_window_s=0), dict(accumulation_window_s=-1),
    dict(accumulation_window_s=float('nan')), dict(accumulation_window_s=True),
    dict(min_accumulated_points=0), dict(min_accumulated_points=-1),
    dict(min_accumulated_points=1.5), dict(min_accumulated_points=True),
    dict(min_accumulated_points=1000001),
])
def test_invalid_window_parameters_are_rejected(tmp_path, settings):
    with pytest.raises(ValueError): parameters(tmp_path, settings)
