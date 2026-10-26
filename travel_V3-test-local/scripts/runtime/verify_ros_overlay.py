"""Check actual module and ROS share paths before any controller is launched."""
from pathlib import Path
import json
import os
import sys
root=Path(__file__).resolve().parents[2]
configured = os.environ.get('RADAR_PROJECT_ROOT')
if configured and Path(configured).resolve() != root:
    raise RuntimeError('RADAR_PROJECT_ROOT points to another project')
os.environ['RADAR_PROJECT_ROOT']=str(root)
import my_radar_follower
loaded_package=Path(my_radar_follower.__file__).resolve().parent
expected_package=root/'ros2_ws/src/my_radar_follower/my_radar_follower'
if loaded_package!=expected_package.resolve():
    raise SystemExit(f'Wrong ROS Python package: {loaded_package}, expected {expected_package}. Source this project overlay last.')
from my_radar_follower.runtime_support import VERSION,shm_key,verify_loaded_package
import my_radar_follower.odom_receiver as receiver
import my_radar_follower.txt_tracker_bridge as bridge
from ament_index_python.packages import get_package_share_directory
for module in (receiver,bridge):verify_loaded_package(module.__file__)
share=Path(get_package_share_directory('my_radar_follower')).resolve()
expected=root/'ros2_ws'/'install/my_radar_follower/share/my_radar_follower'
if share!=expected.resolve():raise SystemExit(f'Wrong ROS share directory: {share}, expected {expected}')
print(json.dumps(dict(version=VERSION,root=str(root),python=sys.executable,shm_key=shm_key(),
    receiver=str(Path(receiver.__file__).resolve()),bridge=str(Path(bridge.__file__).resolve()),
    share=str(share),overlay_ok=True),ensure_ascii=False,indent=2))
