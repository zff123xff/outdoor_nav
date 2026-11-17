from pathlib import Path
from types import SimpleNamespace as NS
from task_pipeline.diagnostics import environment, navigation_processes, origin_service_error


def test_process_report_exposes_only_ros_environment_allowlist(tmp_path):
    process=tmp_path/'123';process.mkdir()
    (process/'cmdline').write_bytes(b'/opt/nav/lib/input_bridge_node\0--ros-args\0-r\0__ns:=/robot_2\0')
    (process/'environ').write_bytes(b'ROS_DOMAIN_ID=17\0PRIVATE_TOKEN=do-not-print\0RMW_IMPLEMENTATION=rmw_fastrtps_cpp\0')
    unrelated=tmp_path/'124';unrelated.mkdir()
    (unrelated/'cmdline').write_bytes(b'/bin/bash\0-c\0input_bridge_node\0')
    data=navigation_processes(tmp_path)
    assert len(data)==1 and data[0]['namespace']=='/robot_2'
    assert data[0]['environment']['ROS_DOMAIN_ID']=='17'
    assert 'PRIVATE_TOKEN' not in repr(data) and 'do-not-print' not in repr(data)


def test_service_error_does_not_claim_running_navigation_is_stopped(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','17')
    node=NS(get_node_names_and_namespaces=lambda:[('nav2_input_bridge','/robot_2')])
    error=origin_service_error(node,'/robot_1/nav2_input_bridge')
    assert '/robot_2/nav2_input_bridge' in error and 'ROS_DOMAIN_ID=17' in error
    assert '请先启动导航栈' not in error
    assert '任务尚未发布' in error


def test_unset_domain_is_reported_without_changing_environment(monkeypatch):
    monkeypatch.delenv('ROS_DOMAIN_ID',raising=False)
    assert environment()['ROS_DOMAIN_ID']=='<unset>'
