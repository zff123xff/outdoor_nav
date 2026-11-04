"""Published task maps exclude native allocation padding and entry storage."""
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid
from co_search_nav.executor import CoSearchExecutor
from co_search_nav.task_geometry import Rectangle


@pytest.mark.parametrize('yaw', [0., .6])
def test_native_storage_is_cropped_to_requested_rectangle(yaw):
    n = NS(task={}, state='executing', region=Rectangle(10., -2., 10., 8., yaw),
           frame='robot_1/map', map_pub=NS(publish=Mock()), metrics={})
    # A valid task is required; values are otherwise independent of policy.
    n.task = {'options': {}}
    msg = OccupancyGrid(); msg.info.resolution = .1
    msg.info.width = 140; msg.info.height = 120
    msg.info.origin.position.x = -7.; msg.info.origin.position.y = -6.
    raw = np.zeros((120, 140), dtype=np.int8); raw[30, 40] = 100; raw[31, 41] = -1
    msg.data = raw.ravel().tolist()
    CoSearchExecutor.map(n, msg)
    published = n.map_pub.publish.call_args.args[0]
    assert (published.info.width, published.info.height) == (100, 80)
    assert (published.info.origin.position.x, published.info.origin.position.y) == pytest.approx(n.region.world(-5., -4.), abs=1e-6)
    assert published.info.origin.orientation.z == pytest.approx(math.sin(yaw/2))
    grid = np.asarray(published.data).reshape(80, 100)
    assert grid[10, 20] == 100 and grid[11, 21] == -1
    assert n.metrics['observed_ratio'] == pytest.approx(7999/8000)
    assert n.metrics['inferred_obstacle_area_m2'] == 0.
