from dataclasses import replace
import sys
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from rtmp_image_bridge.stream import LatestFrame, Settings, StreamWorker, ffmpeg_command
from rtmp_image_bridge.node import RtmpImageBridge


@pytest.mark.parametrize('values', [dict(rtmp_url=''),dict(rtmp_url='/tmp/video.flv'),
    dict(rtmp_url='http://example/live'),dict(rtmp_url='rtmp://localhost:bad/live'),
    dict(mode='other'),dict(mode='listen',rtmp_url='rtmps://localhost/live'),
    dict(width=0),dict(width=True),dict(height=-1),dict(encoding='rgba8'),
    dict(publish_fps=float('nan')),dict(read_timeout_s=0),dict(reconnect_delay_s=-1),
    dict(max_frame_age_s=float('inf')),dict(width=8192,height=8192)])
def test_invalid_settings_fail_before_starting_decoder(values):
    args=dict(rtmp_url='rtmp://127.0.0.1/live/camera');args.update(values)
    with pytest.raises(ValueError):Settings(**args)


def test_argv_is_not_a_shell_and_matches_ros_encoding():
    settings=Settings(rtmp_url='rtmp://localhost/live/a?key=$(echo secret);x',encoding='bgr8')
    command=ffmpeg_command(settings)
    assert command[command.index('-i')+1]==settings.rtmp_url
    assert command[command.index('-pix_fmt')+1]=='bgr24'
    assert command[command.index('-vf')+1]=='scale=640:480'
    assert '-listen' not in command
    assert '-listen' in ffmpeg_command(replace(settings,mode='listen'))


def test_missing_ffmpeg_is_configuration_failure():
    with pytest.raises(ValueError,match='not found'):
        ffmpeg_command(Settings(rtmp_url='rtmp://localhost/live/camera',ffmpeg_path='/nonexistent/ffmpeg'))


def test_latest_frame_never_accumulates_a_queue():
    frames=LatestFrame()
    for i in range(1000):frames.put(i)
    assert frames.take()==999
    assert frames.take() is None
    frames.put('old session');frames.clear()
    assert frames.take() is None


@pytest.mark.parametrize('stamp,age',[(Time(),0.),(Time(sec=1),2.)])
def test_zero_clock_or_stale_frame_is_not_published(stamp,age):
    frames=LatestFrame();frames.put((b'123456',stamp,time.monotonic()-age))
    node=NS(worker=NS(frames=frames),publisher=Mock(),settings=NS(max_frame_age_s=.5))
    RtmpImageBridge.publish_frame(node)
    node.publisher.publish.assert_not_called()


def test_fresh_frame_is_published_once_with_exact_layout():
    frames=LatestFrame();frames.put((b'123456',Time(sec=42),time.monotonic()))
    node=NS(worker=NS(frames=frames),publisher=Mock(),frame_id='robot_2/camera',
            settings=NS(max_frame_age_s=.5,width=2,height=1,encoding='rgb8'))
    RtmpImageBridge.publish_frame(node);RtmpImageBridge.publish_frame(node)
    node.publisher.publish.assert_called_once()
    image=node.publisher.publish.call_args.args[0]
    assert image.header.stamp.sec==42 and image.header.frame_id=='robot_2/camera'
    assert image.step==6 and image.width==2 and image.height==1
    assert bytes(image.data)==b'123456' and image.encoding=='rgb8'


def test_fragmented_raw_frames_stderr_drain_reconnect_and_bounded_shutdown(monkeypatch):
    # Each decoder writes a frame across separate reads, floods stderr, then stalls.
    source="import sys,time;sys.stderr.write('x'*100000);sys.stderr.flush();sys.stdout.buffer.write(b'12');sys.stdout.flush();time.sleep(.03);sys.stdout.buffer.write(b'3456');sys.stdout.flush();time.sleep(10)"
    monkeypatch.setattr('rtmp_image_bridge.stream.ffmpeg_command',lambda settings:[sys.executable,'-u','-c',source])
    captured=[]
    def report(level,text):
        if 'connected' in text:captured.append(worker.frames.take())
    worker=StreamWorker(Settings(rtmp_url='rtmp://localhost/live/test',width=2,height=1,
        read_timeout_s=.2,reconnect_delay_s=.05,decoder='ffmpeg'),stamp=lambda:42,report=report)
    started=time.monotonic();worker.start()
    try:
        deadline=started+3.
        while len(captured)<2 and time.monotonic()<deadline:time.sleep(.02)
        assert len(captured)>=2
        assert all(frame[0]==b'123456' and frame[1]==42 for frame in captured)
    finally:
        worker.close()
    assert not worker.thread.is_alive() and worker.process is None
    assert time.monotonic()-started<4.
