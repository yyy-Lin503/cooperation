"""Configuration and per-topic ownership shared by GUI and CLI controllers."""
from pathlib import Path
import hashlib
import math
import os
import re
import tempfile


def acquire_topic_lock(topic, role='runtime'):
    import fcntl
    identity = str(int(os.environ.get('ROS_DOMAIN_ID', '0') or '0')) + ':' + topic
    token = hashlib.sha256(identity.encode('utf-8')).hexdigest()[:24]
    stream = (Path(tempfile.gettempdir()) / ('radar_' + role + '_' + token + '.lock')).open('a')
    try:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        stream.close()
        raise RuntimeError('Another ' + role + ' owns ' + topic)
    return stream


def validate_settings(value):
    result = dict(value)
    result['chassis_setup'] = str(Path(result['chassis_setup']).expanduser().resolve())
    for name in ('chassis_package', 'chassis_launch', 'chassis_node'):
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', result[name]):
            raise ValueError('Invalid ' + name)
    for name in ('odom_topic', 'cmd_vel_topic'):
        if not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*', result[name]) or '//' in result[name] or result[name].endswith('/'):
            raise ValueError('Use a fully qualified ROS topic for ' + name)
    for name in ('max_linear_speed_mps', 'max_angular_speed_radps'):
        result[name] = float(result[name])
        if not math.isfinite(result[name]) or result[name] < 0:
            raise ValueError('Invalid ' + name)
    if not Path(result['chassis_setup']).is_file():
        raise ValueError('Chassis setup file not found: ' + result['chassis_setup'])
    return result
