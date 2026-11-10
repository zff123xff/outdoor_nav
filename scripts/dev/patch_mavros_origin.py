#!/usr/bin/env python3
"""Add the dedicated UE origin route without changing existing bridge handlers."""
import argparse
from datetime import datetime
import difflib
import os
from pathlib import Path
import shutil
import tempfile


def patched_source(source):
    if 'SIMULATION_ORIGIN_SUFFIX' in source:
        if 'def _handle_simulation_origin(' not in source:
            raise ValueError('Partial origin patch detected; inspect the bridge before applying')
        return source

    def replace(old, new):
        nonlocal source
        if source.count(old) != 1:
            raise ValueError('Bridge version differs: expected one occurrence of '+repr(old[:90]))
        source = source.replace(old, new, 1)

    replace('from rclpy.qos import QoSProfile, qos_profile_sensor_data',
            'from rclpy.qos import QoSProfile, qos_profile_sensor_data, DurabilityPolicy')
    replace('DRONE_GPS_SUFFIX = "/mavros/global_position/global"',
            'DRONE_GPS_SUFFIX = "/mavros/global_position/global"\n\n'
            'SIMULATION_ORIGIN_SUFFIX = "/simulation/gps_origin"')
    replace('            DRONE_GPS_SUFFIX: self._handle_gps,',
            '            DRONE_GPS_SUFFIX: self._handle_gps,\n\n'
            '            SIMULATION_ORIGIN_SUFFIX: self._handle_simulation_origin,')
    replace('            DRONE_GPS_SUFFIX: NavSatFix,',
            '            DRONE_GPS_SUFFIX: NavSatFix,\n\n'
            '            SIMULATION_ORIGIN_SUFFIX: NavSatFix,')
    replace('        qos = qos_profile_sensor_data if msg_type is PointCloud2 else self.qos',
            '        qos = qos_profile_sensor_data if msg_type is PointCloud2 else self.qos\n'
            '        if topic_suffix == SIMULATION_ORIGIN_SUFFIX:\n'
            '            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)')
    method = '''    def _handle_simulation_origin(self, ns, data):
        # This is the GPS of UE ENU (0,0,0), not the moving robot's GPS.
        if not isinstance(data, dict):
            raise ValueError("simulation GPS origin data must be an object")
        values = [data.get(key) for key in ("latitude", "longitude", "altitude")]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) for v in values):
            raise ValueError("simulation GPS origin requires finite latitude/longitude/altitude")
        if abs(values[0]) > 90.0 or abs(values[1]) > 180.0:
            raise ValueError("simulation GPS origin latitude/longitude out of range")
        msg = NavSatFix()
        self._fill_header(msg, data, default_frame_id="wgs84")
        self._fill_status(msg, data)
        if msg.status.status < 0:
            raise ValueError("simulation GPS origin must have a valid fix status")
        msg.latitude, msg.longitude, msg.altitude = map(float, values)
        self._publish_message(self._get_publisher(ns, SIMULATION_ORIGIN_SUFFIX), msg)

'''
    replace('    def _handle_gps(self, ns, data):', method+'    def _handle_gps(self, ns, data):')
    compile(source, 'ue_to_ros_bridge.py', 'exec')
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge-dir', type=Path, default='/home/nhy/MavrosBridge')
    parser.add_argument('--output', type=Path, help='Write a review/test copy, leaving the bridge untouched')
    parser.add_argument('--apply', action='store_true', help='Back up and update only ue_to_ros_bridge.py')
    args = parser.parse_args()
    if args.apply and args.output:
        parser.error('--apply and --output cannot be combined')
    path = args.bridge_dir.expanduser()/'ue_to_ros_bridge.py'
    old = path.read_text()
    new = patched_source(old)
    if args.output:
        args.output.write_text(new)
        print('Prepared:', args.output)
    elif args.apply:
        if new == old:
            print('Origin route is already installed:', path)
            return
        backup = path.parent/'backups'/('before_gps_origin_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        backup.mkdir(parents=True)
        shutil.copy2(path, backup/path.name)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile('w', dir=path.parent, prefix='.gps_origin_', delete=False) as stream:
                temporary = stream.name
                stream.write(new)
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary, path.stat().st_mode & 0o777)
            os.replace(temporary, path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
        print('Updated:', path)
        print('Backup:', backup)
        print('Restart the bridges to load the new input route; existing processes were not signalled.')
    else:
        print(''.join(difflib.unified_diff(old.splitlines(True), new.splitlines(True),
                                       fromfile=str(path), tofile=str(path)+' (origin route)')))


if __name__ == '__main__':
    main()
