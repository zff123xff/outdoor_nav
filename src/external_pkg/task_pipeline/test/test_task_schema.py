import pytest
from task_pipeline.task_schema import validate

P = dict(latitude=31., longitude=120., altitude=0.)


@pytest.mark.parametrize('kind,points,area', [
    ('point_goal', [P, P], {}), ('multi_point', [P, P], {}),
    ('patrol_line', [P, P], {}),
    ('area_search', [], dict(center=P, length_m=10., width_m=5., yaw_deg=45.))])
def test_four_valid_task_shapes(kind, points, area):
    task = validate(dict(task_id='x', task_type=kind, waypoints=points, area=area))
    assert task['task_type'] == kind


@pytest.mark.parametrize('kind,points', [('point_goal', []), ('multi_point', []),
    ('patrol_line', [P]), ('takeoff', [P])])
def test_incomplete_or_aircraft_task_rejected(kind, points):
    with pytest.raises(ValueError):
        validate(dict(task_id='x', task_type=kind, waypoints=points))


@pytest.mark.parametrize('value', [float('nan'), float('inf'), True, 91.])
def test_bad_gps_rejected(value):
    with pytest.raises(ValueError):
        validate(dict(task_id='x', task_type='point_goal', waypoints=[dict(P, latitude=value)]))


def test_cancel_needs_only_id_and_ignores_navigation_fields():
    request = validate(dict(task_id='cancel_me', cancel=True))
    assert request['cancel'] is True and request['task_type'] == ''
    request = validate(dict(task_id='cancel_me', cancel=True, task_type='ignored',
                            waypoints='ignored', area='ignored'))
    assert request['waypoints'] == []


@pytest.mark.parametrize('value', ['true', 1, None])
def test_cancel_flag_requires_real_boolean(value):
    with pytest.raises(ValueError, match='cancel'):
        validate(dict(task_id='cancel_me', cancel=value))


def test_cancel_requires_nonempty_id():
    with pytest.raises(ValueError, match='task_id'):
        validate(dict(task_id='', cancel=True))


def test_task_target_classes_preserved_and_cancel_ignores_them():
    data=validate(dict(task_id='classes',task_type='point_goal',waypoints=[P],targets=['bottle','chair','bottle']))
    assert data['targets']==['bottle','chair']
    assert validate(dict(task_id='classes',cancel=True,targets='ignored'))['targets']==[]


@pytest.mark.parametrize('value',['bottle',[1],['']])
def test_invalid_class_selection_rejected(value):
    with pytest.raises(ValueError,match='targets'):
        validate(dict(task_id='classes',task_type='point_goal',waypoints=[P],targets=value))
