import math
from types import SimpleNamespace as NS
import pytest
from task_pipeline.inspection_logic import angle_error, nearest, validate_batch, visit_pose


def test_nearest_uses_current_pose_and_stable_id_tie_break():
    targets={2:(20.,0.,0.),1:(0.,0.,0.)}
    assert nearest((10.,0.,0.,0.),targets)==1
    assert nearest((18.,0.,0.,0.),targets)==2


def test_approach_stays_nine_metres_from_target():
    assert visit_pose((0.,0.,0.,0.),(20.,0.,0.)) == pytest.approx((11.,0.,0.,0.))
    p=visit_pose((0.,0.,0.,0.),(20.,0.,5.))
    assert math.dist(p[:3],(20.,0.,5.))==pytest.approx(9.)


def test_already_inside_range_only_changes_heading():
    assert visit_pose((0.,0.,0.,1.),(0.,8.,0.))==pytest.approx((0.,0.,0.,math.pi/2))
    assert visit_pose((0.,0.,0.,1.),(0.,0.,0.))[3]==1.


@pytest.mark.parametrize('target',[(20.,0.,10.),(0.,0.,11.)])
def test_unreachable_height_is_rejected(target):
    with pytest.raises(ValueError):visit_pose((0.,0.,0.,0.),target)


def test_angles_wrap_at_pi():
    assert angle_error(-math.pi+.1,math.pi-.1)==pytest.approx(.2)


@pytest.mark.parametrize('items', [[(1,0.),(1,1.)],[(0,1.)],[(1,float('nan'))]])
def test_invalid_batch_is_atomic(items):
    message=NS(targets=[NS(target_id=key,target_position=NS(x=x,y=0.,z=0.)) for key,x in items])
    with pytest.raises(ValueError):validate_batch(message)
