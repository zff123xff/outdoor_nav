from types import SimpleNamespace as NS
import math
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo
from yolo_detector.targets import write_targets, read_targets, matches
from yolo_detector.projection import project, box_position, Tracks, enu_to_gps


def test_atomic_class_file_and_selection(tmp_path):
    path=tmp_path/'state/targets.yaml'
    write_targets(path,['person','car','person'],'one')
    assert read_targets(path)==['person','car']
    assert matches(read_targets(path),'Person')
    assert not matches(read_targets(path),'dog')
    write_targets(path,['16'],'two')
    assert matches(read_targets(path),'dog',16)
    assert not matches(read_targets(path),'person',0)
    write_targets(path,[])
    assert matches(read_targets(path),'anything')
    assert not list(path.parent.glob('.targets_*'))


def test_measured_depth_projection_and_no_depth_rejection():
    camera=CameraInfo(width=640,height=480,k=[100.,0.,320.,0.,100.,240.,0.,0.,1.],distortion_model='plumb_bob')
    points=np.array([[0.,0.,5.],[.1,0.,5.],[-.1,0.,5.],[4.,0.,5.]])
    uv=project(points,camera)
    assert np.allclose(uv[0],[320.,240.])
    box=NS(x_min=310,y_min=230,x_max=330,y_max=250)
    target=box_position(box,uv,points,points,min_points=3)
    assert np.allclose(target,[0.,0.,5.])
    assert box_position(box,uv[:1],points[:1],points[:1]) is None
    with pytest.raises(ValueError):project(points,CameraInfo())


def test_track_ids_stable_unique_and_incremental():
    tracks=Tracks(radius=1.)
    assert tracks.update([('bottle',np.array([0.,0.,0.])),('bottle',np.array([2.,0.,0.]))],1.)==[1,2]
    assert tracks.update([('bottle',np.array([2.1,0.,0.])),('bottle',np.array([.1,0.,0.]))],2.)==[2,1]
    assert tracks.update([('chair',np.array([0.,0.,0.]))],3.)==[3]


def test_enu_maps_to_gps_altitude():
    latitude,longitude,altitude=enu_to_gps([10.,0.,2.],(31.,120.,100.))
    assert latitude==pytest.approx(31.,abs=1e-6)
    assert longitude>120.
    assert altitude==102.
