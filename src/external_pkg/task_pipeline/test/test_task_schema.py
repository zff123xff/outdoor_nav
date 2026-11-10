import math
import struct
import pytest
from task_pipeline.task_schema import validate, to_message, from_message

P = [40.9445353, 113.172819, 1298.734571]

@pytest.mark.parametrize('data', [dict(task_id='140', waypoints=P),
    dict(task_id='141', waypoints=P*2), dict(task_id='145', waypoints=P*3),
    dict(task_id='142', center=P, length_m=10., width_m=5., yaw_deg=90.),
    dict(task_id='143')])
def test_five_wire_tasks_roundtrip(data):
    task = validate(data)
    assert from_message(to_message(task)) == task
    assert 'task_type' not in task

@pytest.mark.parametrize('data', [dict(task_id='140'), dict(task_id='141',waypoints=P),
    dict(task_id='140',waypoints=P*2), dict(task_id='145'), dict(task_id='142',center=P),
    dict(task_id='143',task_type='search_shoot'), dict(task_id='invalid'), dict(task_id=143),
    dict(task_id='140',waypoints=[113.,40.,0.]), dict(task_id='140',waypoints=[1.,2.]),
    dict(task_id='140',waypoints=[1.,2.,float('nan')]), dict(task_id='140',waypoints=[1.,2.,1e100]),
    dict(task_id='143',targets=[1]), dict(task_id='143',cancel='true')])
def test_invalid_command_rejected(data):
    with pytest.raises(ValueError): validate(data)


def test_cancel_ignores_all_non_id_fields():
    assert validate(dict(task_id='143',cancel=True,targets=None,waypoints='bad',task_type='ignored')) == validate(dict(task_id='143',cancel=True))


def test_float32_is_explicit_and_precision_loss_is_visible():
    point = validate(dict(task_id='140',waypoints=P))['waypoints']
    assert point[1] == struct.unpack('f',struct.pack('f',P[1]))[0]
    assert point != P


def test_target_classes_do_not_activate_detector():
    assert validate(dict(task_id='143',targets=['person','car','person']))['targets'] == ['person','car']


def test_reserved_fields_default_to_zero():
    message = to_message(dict(task_id='143'))
    assert message.gait_speed == message.air_route == 0


@pytest.mark.parametrize('value', [-32768, 0, 32767])
def test_reserved_fields_roundtrip(value):
    data = dict(task_id='143', gait_speed=value, air_route=value)
    assert from_message(to_message(data)) == validate(data)


@pytest.mark.parametrize('key', ['gait_speed', 'air_route'])
@pytest.mark.parametrize('value', [-32769, 32768, True, 1.5, '1'])
def test_reserved_fields_require_int16(key, value):
    with pytest.raises(ValueError):
        validate(dict(task_id='143', **{key: value}))


def test_cancel_ignores_reserved_fields():
    assert validate(dict(task_id='143', cancel=True, gait_speed='ignored', air_route=None)) == validate(dict(task_id='143', cancel=True))
