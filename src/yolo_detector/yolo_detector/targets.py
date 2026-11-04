"""Shared, atomically replaced class selection for one robot."""
import os
from pathlib import Path
import tempfile
import yaml


def validate_targets(values):
    if not isinstance(values, (list, tuple)) or any(not isinstance(v, str) or not v.strip() for v in values):
        raise ValueError('targets must be a list of nonempty class names or class IDs')
    return list(dict.fromkeys(v.strip() for v in values))


def targets_path(namespace, configured=''):
    if configured:
        return Path(configured).expanduser()
    ns = namespace.strip('/').replace('/', '_') or 'default'
    return Path('/tmp/outdoor_nav')/ns/'detection_targets.yaml'


def write_targets(path, values, task_id=''):
    """Readers see the old complete file or the new complete file."""
    values = validate_targets(values)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile('w', dir=path.parent, prefix='.targets_', delete=False) as stream:
            name = stream.name
            yaml.safe_dump(dict(task_id=task_id, targets=values), stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if name and os.path.exists(name):
            os.unlink(name)


def read_targets(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError('target file must be a YAML mapping')
    return validate_targets(data.get('targets', []))


def matches(targets, class_name, class_id=None):
    allowed = {v.casefold() for v in targets}
    return not allowed or class_name.casefold() in allowed or str(class_id) in allowed
