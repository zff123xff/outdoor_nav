from pathlib import Path
from task_pipeline.examples.sender import load_example


def test_example_normalizes_gps_and_namespace():
    ns, task = load_example(Path(__file__).parents[1]/'config/examples/gps/patrol_line.yaml',
        'patrol_line', 'robot_2', (40., 113., 1200.))
    assert ns == 'robot_2'
    assert task['task_id'] == '141'
    assert task['waypoints']
    assert -90 <= task['waypoints'][0] <= 90
