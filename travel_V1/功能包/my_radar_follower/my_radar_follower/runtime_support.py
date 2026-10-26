"""Fail fast on a copied/incorrect ROS overlay; no motion functionality."""
from pathlib import Path
import os

VERSION = 'V8'
DEFAULT_KEY = 123458

def project_root():
    return Path(os.environ.get('RADAR_PROJECT_ROOT', Path(__file__).resolve().parents[3])).resolve()

def verify_loaded_package(module_file):
    root=project_root()
    if (root/'VERSION.txt').read_text(encoding='utf-8').strip()!=VERSION:
        raise RuntimeError('Package/project version marker mismatch')
    expected=root/'功能包/my_radar_follower/my_radar_follower'
    actual=Path(module_file).resolve()
    if not expected.is_dir() or actual.parent!=expected.resolve():
        raise RuntimeError(f'Wrong ROS package: loaded={actual}, expected={expected}. Rebuild and source this project overlay last.')
    return root

def shm_key():
    value=int(os.environ.get('RADAR_ODOM_SHM_KEY',str(DEFAULT_KEY)))
    if not 1<=value<=2147483647:raise ValueError('RADAR_ODOM_SHM_KEY outside positive int32')
    if value in (123456,123457):raise ValueError('Reserved legacy IPC key; use a separate V8/V9 key, never overwrite V4/V5')
    return value
