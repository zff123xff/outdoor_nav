import json
import pytest
from geometry_msgs.msg import Point
from task_interfaces.msg import InspectionArrival
from task_pipeline.inspection_udp_bridge import decode_assignment, encode_arrival


def packet(**overrides):
    data=dict(namespace='robot_1',topic='targets/assigned',data=dict(targets=[dict(
        target_id=1,target_position=dict(x=12.,y=-3.,z=0.))]))
    data.update(overrides)
    return json.dumps(data).encode()


def test_assignment_preserves_enu_and_unsigned_id():
    message=decode_assignment(packet(),'robot_1')
    assert message.targets[0].target_id==1
    assert message.targets[0].target_position.x==12.
    assert message.targets[0].target_position.y==-3.


@pytest.mark.parametrize('value',[True,-1,0,2**64,'1'])
def test_invalid_ids_never_get_coerced(value):
    with pytest.raises(ValueError):decode_assignment(packet(data=dict(targets=[dict(
        target_id=value,target_position=dict(x=1.,y=0.,z=0.))])),'robot_1')


def test_other_robot_and_non_assignment_packets_are_rejected():
    with pytest.raises(ValueError):decode_assignment(packet(namespace='robot_2'),'robot_1')
    with pytest.raises(ValueError):decode_assignment(packet(topic='other'),'robot_1')


def test_arrival_contains_observation_fields():
    event=InspectionArrival(task_id='143',target_id=8,target_position=Point(x=20.),
        robot_position=Point(x=11.),distance_m=9.,heading_error_rad=0.)
    data=json.loads(encode_arrival(event,'robot_1'))
    assert data['topic']=='targets/inspection_arrived'
    assert data['data']['target_id']==8
    assert data['data']['distance_m']==9.
    assert data['data']['robot_position']['x']==11.
